/*
 * SGI GR2 / "Express" graphics — register/FIFO shell.
 *
 * See include/hw/misc/sgi_gr2.h for the register map and the presence
 * contract.  This first pass is a register shell: a byte-backed register
 * file for the whole GIO64 board window, with the HQ2 presence magic and the
 * board-version register initialised so the stock `gr2` driver's probe
 * (Gr2Probe: magics + bdvers decode + GE RAM pattern test) recognises the
 * board.  HQ2 token dispatch, microcode download/verify and VC1/XMAP
 * bring-up come in later phases.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_gr2.h"
#include "trace.h"

/* Vertical retrace, as a real CRT would produce it.  The board raises its GIO
 * interrupt at ~60 Hz once started; the pulse is asserted for the blanking
 * interval and then lowered so the HPC3's level-triggered GIO2 source clears,
 * exactly like Newport's VBLANK model (see sgi_newport.c:newport_vblank_timer). */
#define SGI_GR2_RETRACE_HZ       60
#define SGI_GR2_RETRACE_PULSE_NS (500 * 1000) /* ~40 scanlines of blanking */

/* The RAMDAC palette is not a static table: the guest's DDX programs it
 * through the XMAP_PAL_* registers at server start, and the model builds
 * it from that write stream (see the header).  Reset clears it to black. */

/* Extract `size` big-endian bytes starting at byte `byte` of a 32-bit word. */
static uint64_t sgi_gr2_word_read(uint32_t word, unsigned byte, unsigned size)
{
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val = (val << 8) | ((word >> (8 * (3 - byte - i))) & 0xff);
    }
    return val;
}

/* Assemble `size` big-endian bytes from `value` into a 32-bit word. */
static uint32_t sgi_gr2_word_write(uint64_t value, unsigned byte, unsigned size)
{
    uint32_t word = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        word |= (uint8_t)(value >> (8 * (size - 1 - i))) << (8 * (3 - byte - i));
    }
    return word;
}

static void sgi_gr2_update_display(void *opaque);

/* RE3 solid-rectangle fill.  In the 8-bit mode the guest runs, the latched
 * colour is a RAMDAC index, so the fill expands it through the palette and
 * writes the resulting RGB into `scanout` — the same buffer the display
 * update reads, so the RE3 producer and the scanout stage meet in one place
 * and a black screen can only mean the producer wrote nothing.  Returns the
 * RGB written, for tracing. */
static uint32_t sgi_gr2_re3_fill(SGIGr2State *s, uint8_t colour,
                                 int x, int y, int w, int h)
{
    uint32_t rgb = s->ramdac[colour];
    int xx, yy;

    if (!s->scanout) {
        return rgb;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > SGI_GR2_SCREEN_W) {
        w = SGI_GR2_SCREEN_W - x;
    }
    if (y + h > SGI_GR2_SCREEN_H) {
        h = SGI_GR2_SCREEN_H - y;
    }
    for (yy = y; yy < y + h; yy++) {
        for (xx = x; xx < x + w; xx++) {
            s->scanout[yy * SGI_GR2_SCREEN_W + xx] = rgb;
        }
    }
    sgi_gr2_update_display(s);
    return rgb;
}

/* RE3 stippled-rectangle fill: the DDX arms a 32-bit stipple pattern on token
 * 318 and a foreground colour index on token 314, then streams the rectangle.
 * Render one pixel per bit of the pattern, repeating every 32 columns — bit 31
 * is the leftmost pixel of each 32-pixel tile.  This is the grainy root: the
 * X DDX's expStippledFillRects writes pattern 0x10101010 (a 1-in-4 dot). */
static void sgi_gr2_re3_stipple_fill(SGIGr2State *s, uint8_t fg, uint8_t bg,
                                     uint32_t pattern, int x, int y, int w, int h)
{
    uint32_t fg_rgb = s->ramdac[fg];
    uint32_t bg_rgb = s->ramdac[bg];
    int xx, yy;

    if (!s->scanout) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > SGI_GR2_SCREEN_W) {
        w = SGI_GR2_SCREEN_W - x;
    }
    if (y + h > SGI_GR2_SCREEN_H) {
        h = SGI_GR2_SCREEN_H - y;
    }
    for (yy = y; yy < y + h; yy++) {
        for (xx = x; xx < x + w; xx++) {
            bool on = (pattern >> (31 - (xx & 31))) & 1;

            s->scanout[yy * SGI_GR2_SCREEN_W + xx] = on ? fg_rgb : bg_rgb;
        }
    }
    sgi_gr2_update_display(s);
}

/* Evaluate the pending draw op and, if it is a full-screen fill, paint it.
 * Called at each op boundary — token 331 (which starts a new sub-op) and token
 * 490 (the terminator) — because one 490-terminated region can hold several 331
 * sub-ops, and the grainy root's stipple (318) and its full-screen rect live in
 * the FIRST sub-op, before the panel's own 331.  The rect is recognised
 * structurally: a rect-list op (304/318) containing a 1280-then-1024 pair, or
 * any fill op whose final two data words are (1280,1024).  305 is excluded from
 * the pair rule because an expSolidSpans triple list ends (0x500,0x400,0x1) —
 * a span at x=1280,y=1024, not a rectangle. */
static void sgi_gr2_re3_flush_fill(SGIGr2State *s)
{
    bool fs_rect = s->re3_pair_seen && !s->re3_spans_seen &&
                   (s->re3_solid_seen || s->re3_stipple_valid);
    bool fs_tail = (s->re3_solid_seen || s->re3_spans_seen ||
                    s->re3_stipple_valid) &&
                   s->prev_puc == SGI_GR2_SCREEN_W &&
                   s->last_puc == SGI_GR2_SCREEN_H;

    if (!fs_rect && !fs_tail) {
        return;
    }
    if (s->re3_stipple_valid) {
        uint8_t fg = s->re3_fg_valid ? s->re3_fg : s->re3_colour;

        sgi_gr2_re3_stipple_fill(s, fg, s->re3_colour, s->re3_stipple,
                                 0, 0, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
        trace_sgi_gr2_re3_stipple(fg, s->re3_colour, s->re3_stipple,
                                  SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
    } else if (s->re3_colour_valid) {
        uint32_t rgb = sgi_gr2_re3_fill(s, s->re3_colour, 0, 0,
                                        SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);

        trace_sgi_gr2_re3_fill(s->re3_colour, rgb,
                               SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
    }
}

static uint64_t sgi_gr2_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIGr2State *s = SGI_GR2(opaque);
    uint64_t val = 0;
    unsigned i;

    if (!s->present) {
        /* Absent board: the GIO slot reads all-ones, exactly like the
         * empty-slot stub, so a plain `-M indy` boot is unchanged. */
        return ~0ULL;
    }
    if (offset + size > SGI_GR2_REG_SIZE) {
        return ~0ULL;
    }
    /* GE7 instruction load/verify: the four-word window and the load register
     * are per-PC storage, selected by the last gepc write.  Returned verbatim
     * (the driver masks with 0x3dfffff itself). */
    if (offset >= SGI_GR2_GE_WIN_OFF &&
        offset < SGI_GR2_GE_WIN_OFF + SGI_GR2_GE_WIN_WORDS * 4) {
        unsigned idx = (offset - SGI_GR2_GE_WIN_OFF) / 4;

        val = sgi_gr2_word_read(
            s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][idx], offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_UCODELOAD &&
               offset < SGI_GR2_HQ_UCODELOAD + 4) {
        val = sgi_gr2_word_read(
            s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][4], offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_GEPC && offset < SGI_GR2_HQ_GEPC + 4) {
        val = sgi_gr2_word_read(s->gepc, offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_FIFOSTAT &&
               offset < SGI_GR2_HQ_FIFOSTAT + 4) {
        /* HQ2 status: bit 0 = HQ2 idle/ready (the X DDX's expInit spins on
         * it — lw 0x6a040; andi 0x1; beqz), bit 1 = ucode ready (the kernel's
         * _Gr2UcodeReady).  Occupancy stays empty. */
        val = sgi_gr2_word_read(
            SGI_GR2_HQ_IDLE_BIT |
            (s->hq_ready ? SGI_GR2_HQ_READY_BIT : 0), offset & 3, size);
    } else if (offset >= SGI_GR2_XMAP_STATUS &&
               offset < SGI_GR2_XMAP_STATUS + 4) {
        /* XMAP status: bit 1 reports "ready"; _Gr2XMAPInit3 spins on it.
         * The driver byte-reads this (lbu), so the bit must appear in the
         * addressed byte lane, not as bit 1 of the whole 32-bit word. */
        val = s->xmap_ready ? SGI_GR2_XMAP_READY_BIT : 0;
    } else {
        /* GE units at or above the variant's engine count are not populated,
         * so the driver's GE-count pattern test stops counting there. */
        if (offset >= SGI_GR2_GE_OFF &&
            offset < SGI_GR2_GE_OFF + SGI_GR2_GE_UNITS * SGI_GR2_GE_STRIDE) {
            unsigned unit = (offset - SGI_GR2_GE_OFF) / SGI_GR2_GE_STRIDE;

            if (unit >= s->ges) {
                return ~0ULL;
            }
        }
        /* Big-endian byte assembly so byte/half/word accesses agree. */
        for (i = 0; i < size; i++) {
            val = (val << 8) | s->regs[offset + i];
        }
    }
    /* The command/register traffic worth watching (µcode load staging, HQ2,
     * GE and RE3) is all at or above the HQ2 block; the shram and token FIFO
     * are huge and mostly idle. */
    if (offset < 0x40 || offset >= SGI_GR2_HQUCODE_OFF) {
        trace_sgi_gr2_read(offset, size, val);
    }
    /* The HQ2 block is the polled surface: log each access with the reading PC
     * so the poll loop (and the value it expects) can be read off directly. */
    if (offset >= SGI_GR2_HQ_OFF && offset < SGI_GR2_HQ_OFF + 0x80) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_hqread(offset, val, pc);
    }
    /* The token FIFO is written as the command channel; any READ of it is the
     * board's read-back/consumption contract, so log those with the PC too. */
    if (offset >= SGI_GR2_FIFO_OFF && offset < SGI_GR2_FIFO_OFF + 0x20000) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_fiforead(offset, val, pc);
    }
    /* VC1 / XMAP / RE3 / GE / bdvers reads with the PC: the last surface that
     * could hold the "display is up" gate the DDX waits on. */
    if (offset >= SGI_GR2_HQUCODE_OFF) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_regread(offset, val, pc);
    }
    return val;
}

static void sgi_gr2_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    SGIGr2State *s = SGI_GR2(opaque);
    unsigned i;

    if (!s->present || offset + size > SGI_GR2_REG_SIZE) {
        return;
    }
    /* The command/register traffic worth watching (µcode load staging, HQ2,
     * GE and RE3) is all at or above the HQ2 block; the shram and token FIFO
     * are huge and mostly idle. */
    if (offset < 0x40 || offset >= SGI_GR2_HQUCODE_OFF) {
        trace_sgi_gr2_write(offset, value, size);
    }
    /* Token-FIFO traffic (fifo[token] = data) is the producer/consumer channel;
     * trace it separately and cheaply so a draw window does not require tracing
     * the whole register block. */
    if (offset >= SGI_GR2_FIFO_OFF && offset < SGI_GR2_FIFO_OFF + 0x20000) {
        trace_sgi_gr2_fifo(offset, value, size);
    }
    /* RE3 producer, 8-bit mode.  The DDX writes the fill colour (a RAMDAC
     * INDEX) to the RE3 colour token, then pushes the rectangle's geometry as
     * the last two PUC_DATA words.  [ASSUMPTION, from the one captured
     * `xsetroot -solid red` stream: its PUC_DATA tail was 0x500 then 0x400 —
     * 1280 x 1024, the screen size — a full-screen clear; and RED vs BLUE
     * differed in exactly the colour token, 0x1 vs 0x4.]  So latch the colour,
     * and on the (w,h) pair fill that rectangle of the scanout buffer. */
    if (size == 4 && offset == SGI_GR2_RE3_COLOUR_TOKEN) {
        s->re3_colour = value & 0xff;
        s->re3_colour_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_HQ_TOKEN_START) {
        /* The full-screen fill is decided at the op terminator (token 490), not
         * here.  An expSolidSpans payload ends with the span triple
         * (0x500, 0x400, 0x1) — x, y, count — so 0x400 DOES follow 0x500 in the
         * middle of a span list, and testing inline fires on a span that is not
         * a rectangle.  That misfire is what wiped the weave (note 28).  Only
         * the op's FINAL two data words are the rect geometry. */
        /* The generic PUC path: PUC_COLOR chose the colour, PUC_RECTI2D armed
         * a rectangle, and its three PUC_DATA words are (x0, x1, y0) — one
         * horizontal span.  The root weave is 1024 of these. */
        if (s->puc_rect_armed) {
            s->puc_rect[s->puc_rect_n++] = value;
            if (s->puc_rect_n == 3) {
                uint32_t x0 = s->puc_rect[0];
                uint32_t x1 = s->puc_rect[1];
                uint32_t y0 = s->puc_rect[2];

                if (x1 >= x0 && x0 < SGI_GR2_SCREEN_W && y0 < SGI_GR2_SCREEN_H) {
                    uint32_t rgb = sgi_gr2_re3_fill(s, s->puc_colour, x0, y0,
                                                    x1 - x0 + 1, 1);

                    trace_sgi_gr2_puc_span(s->puc_colour, rgb, x0, x1, y0);
                }
                s->puc_rect_armed = false;
                s->puc_rect_n = 0;
            }
        }
        s->prev_puc = s->last_puc;
        s->last_puc = value;
        s->last_puc_valid = true;
        if (s->prev_puc == SGI_GR2_SCREEN_W &&
            s->last_puc == SGI_GR2_SCREEN_H) {
            s->re3_pair_seen = true;
        }
    }
    if (size == 4 && offset == SGI_GR2_PUC_COLOR_TOKEN) {
        s->puc_colour = value & 0xff;
    }
    if (size == 4 && offset == SGI_GR2_PUC_RECTI2D_TOKEN) {
        s->puc_rect_armed = true;
        s->puc_rect_n = 0;
    }
    /* Fill-op markers (DDX token map, note 25/26).  An op starts at token 331,
     * so the per-op markers are cleared there and set by 304 (solid rect),
     * 314 (stipple fg colour) and 318 (stipple pattern); token 490 is the op
     * terminator.  This is what distinguishes a grainy stippled root fill from
     * a flat solid fill without guessing at the PUC_DATA tail. */
    if (size == 4 && offset == SGI_GR2_RE3_OP_TOKEN) {
        sgi_gr2_re3_flush_fill(s);
        s->re3_solid_seen = false;
        s->re3_spans_seen = false;
        s->re3_pair_seen = false;
        s->re3_stipple_valid = false;
        s->re3_fg_valid = false;
    }
    if (size == 4 && offset == SGI_GR2_RE3_SOLID_TOKEN) {
        s->re3_solid_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_SPANS_TOKEN) {
        s->re3_spans_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_FG_TOKEN) {
        s->re3_fg = value & 0xff;
        s->re3_fg_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_STIPPLE_TOKEN) {
        s->re3_stipple = (uint32_t)value;
        s->re3_stipple_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_DONE_TOKEN) {
        /* Op end.  Deciding here — not inline on each PUC_DATA — is what keeps
         * an expSolidSpans triple list from being mistaken for a rect. */
        sgi_gr2_re3_flush_fill(s);
        s->re3_colour_valid = false;
        s->re3_solid_seen = false;
        s->re3_spans_seen = false;
        s->re3_pair_seen = false;
        s->re3_stipple_valid = false;
        s->re3_fg_valid = false;
    }
    /* HQ2-block writes (start / DMA control / FIFO thresholds) with the PC. */
    if (offset >= SGI_GR2_HQ_OFF && offset < SGI_GR2_HQ_OFF + 0x80) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_hqwrite(offset, value, pc);
    }
    /* GE7 instruction load/verify: store the window and load-register words
     * verbatim into the per-PC slot selected by the last gepc write.  No
     * masking or tidying — the driver compares against what it wrote. */
    if (offset >= SGI_GR2_GE_WIN_OFF &&
        offset < SGI_GR2_GE_WIN_OFF + SGI_GR2_GE_WIN_WORDS * 4) {
        unsigned idx = (offset - SGI_GR2_GE_WIN_OFF) / 4;

        s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][idx] =
            sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    if (offset >= SGI_GR2_HQ_UCODELOAD &&
        offset < SGI_GR2_HQ_UCODELOAD + 4) {
        s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][4] =
            sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    if (offset >= SGI_GR2_HQ_GEPC && offset < SGI_GR2_HQ_GEPC + 4) {
        s->gepc = sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    /* Gr2Start kicks the HQ2 sequencer by writing the start token to the
     * command FIFO, then polls the "ucode ready" bit; model the sequencer as
     * ready from that write on.  The token itself is still stored below. */
    if (offset >= SGI_GR2_HQ_TOKEN_START &&
        offset < SGI_GR2_HQ_TOKEN_START + 4) {
        s->hq_ready = true;
    }
    /* Programming the XMAP control registers makes the mode generator ready
     * for the driver's poll. */
    if (offset >= SGI_GR2_XMAP_CTL_OFF && offset < SGI_GR2_XMAP_CTL_END) {
        s->xmap_ready = true;
    }
    /* RAMDAC colour-map programming (XMAP_PAL_*): the DDX writes the entry
     * index to 0x6c1b0, a control byte to 0x6c1b4, and a sliding R,G,B byte
     * stream to 0x6c1a8 — each 32-bit data write carries this entry's R,G,B
     * plus the first byte (R) of the next entry, which is why the index for a
     * completing entry is always written before the write that completes it.
     * Build the palette from those bytes; nothing is assumed about its values. */
    if (offset == SGI_GR2_XMAP_PAL_INDEX) {
        s->ramdac_index = value & 0xff;
        /* Each entry's bytes start fresh after its index write; a 32-bit data
         * write carries a 4th byte (the next entry's R) which the following
         * index write therefore discards.  Validated against the `xwd` oracle:
         * this framing reproduces it (index 1 = red, 4 = blue, 0x10 =
         * (0x4c,0x71,0x9e), 0x20..0x37 = the grey ramp). */
        s->ramdac_stage_n = 0;
    } else if (offset == SGI_GR2_XMAP_PAL_DATA) {
        unsigned k;

        for (k = 0; k < size; k++) {
            uint8_t byte = (value >> (8 * (size - 1 - k))) & 0xff;

            s->ramdac_stage[s->ramdac_stage_n++] = byte;
            if (s->ramdac_stage_n == 3) {
                uint32_t rgb = ((uint32_t)s->ramdac_stage[0] << 16) |
                               ((uint32_t)s->ramdac_stage[1] << 8) |
                               s->ramdac_stage[2];

                /* The visible map takes the normal entries (0x11) and the
                 * weave entries (0x10); 0x00/0x01/0x1c are the overlay/pup/
                 * 24-bit maps.  See the note by SGI_GR2_XMAP_PAL_BANK_*. */
                if (s->ramdac_ctl == SGI_GR2_XMAP_PAL_BANK_INSTALLED ||
                    s->ramdac_ctl == SGI_GR2_XMAP_PAL_BANK_ALT) {
                    s->ramdac[s->ramdac_index] = rgb;
                    trace_sgi_gr2_ramdac(s->ramdac_index, rgb);
                }
                s->ramdac_stage_n = 0;
            }
        }
    } else if (offset == SGI_GR2_XMAP_PAL_CTL) {
        s->ramdac_ctl = value & 0xff;
    }
    /* Unpopulated GE units discard writes. */
    if (offset >= SGI_GR2_GE_OFF &&
        offset < SGI_GR2_GE_OFF + SGI_GR2_GE_UNITS * SGI_GR2_GE_STRIDE) {
        unsigned unit = (offset - SGI_GR2_GE_OFF) / SGI_GR2_GE_STRIDE;

        if (unit >= s->ges) {
            return;
        }
    }
    /* The presence magic and the board-version register latch nothing from
     * the bus; ignore writes there. */
    if (offset < SGI_GR2_HQ_MYSTERY + 4 && offset + size > SGI_GR2_HQ_MYSTERY) {
        return;
    }
    /* The HQ2 FIFO occupancy/status register is read-only (the driver only
     * polls it); a stray write must not corrupt the level/error bits. */
    if (offset >= SGI_GR2_HQ_FIFOSTAT &&
        offset < SGI_GR2_HQ_FIFOSTAT + 4) {
        return;
    }
    for (i = 0; i < size; i++) {
        uint8_t byte = (value >> (8 * (size - 1 - i))) & 0xff;

        s->regs[offset + i] = byte;
    }
}

static const MemoryRegionOps sgi_gr2_ops = {
    .read = sgi_gr2_read,
    .write = sgi_gr2_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/*
 * Scanout (P0.4 step a).  A QEMU display surface proves the OUTPUT stage
 * before the RE3 producer: the framebuffer is filled with a known pattern
 * and must appear on the GR2 console, falsifiably, independent of the guest.
 */
static void sgi_gr2_fill_bars(SGIGr2State *s)
{
    static const uint32_t bars[8] = {
        0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
        0xff00ff, 0xff0000, 0x0000ff, 0x000000,
    };
    int x, y;

    if (!s->scanout) {
        return;
    }
    for (y = 0; y < SGI_GR2_SCREEN_H; y++) {
        for (x = 0; x < SGI_GR2_SCREEN_W; x++) {
            s->scanout[y * SGI_GR2_SCREEN_W + x] = bars[x * 8 / SGI_GR2_SCREEN_W];
        }
    }
}

static void sgi_gr2_update_display(void *opaque)
{
    SGIGr2State *s = opaque;
    DisplaySurface *surface;
    uint32_t *dest;
    int stride, y;

    if (!s->con || !s->scanout) {
        return;
    }
    surface = qemu_console_surface(s->con);
    if (!surface) {
        return;
    }
    dest = (uint32_t *)surface_data(surface);
    stride = surface_stride(surface);
    for (y = 0; y < SGI_GR2_SCREEN_H; y++) {
        memcpy(dest + (y * stride) / 4,
               s->scanout + y * SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_W * 4);
    }
    dpy_gfx_update(s->con, 0, 0, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
}

static const GraphicHwOps sgi_gr2_gfx_ops = {
    .gfx_update = sgi_gr2_update_display,
};

/* Lower the retrace IRQ at the end of the blanking interval. */
static void sgi_gr2_retrace_lower(void *opaque)
{
    SGIGr2State *s = SGI_GR2(opaque);

    s->retrace_active = false;
    qemu_irq_lower(s->irq);
}

/* 60 Hz retrace tick: raise the GIO interrupt for the blanking interval.
 * Only once the driver has started the board (Gr2Start writes the start
 * token), so we never deliver an interrupt before its vector is registered. */
static void sgi_gr2_retrace_tick(void *opaque)
{
    SGIGr2State *s = SGI_GR2(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->present && s->hq_ready && !s->retrace_active) {
        s->retrace_active = true;
        qemu_irq_raise(s->irq);
        timer_mod(s->retrace_lower_timer, now + SGI_GR2_RETRACE_PULSE_NS);
    }
    timer_mod(s->retrace_timer,
              now + NANOSECONDS_PER_SECOND / SGI_GR2_RETRACE_HZ);
}

static void sgi_gr2_reset(DeviceState *dev)
{
    SGIGr2State *s = SGI_GR2(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ucode, 0, sizeof(s->ucode));
    s->gepc = 0;
    s->hq_ready = false;
    s->xmap_ready = false;
    s->retrace_active = false;
    /* Default RAMDAC palette (measured; see the table) and clear the RE3
     * producer latches. */
    memset(s->ramdac, 0, sizeof(s->ramdac));
    s->ramdac_index = 0;
    s->ramdac_ctl = 0;
    s->ramdac_stage_n = 0;
    s->re3_colour = 0;
    s->re3_colour_valid = false;
    s->re3_solid_seen = false;
    s->re3_spans_seen = false;
    s->re3_pair_seen = false;
    s->re3_stipple_valid = false;
    s->re3_stipple = 0;
    s->re3_fg = 0;
    s->re3_fg_valid = false;
    s->last_puc = 0;
    s->prev_puc = 0;
    s->last_puc_valid = false;
    if (s->irq) {
        qemu_irq_lower(s->irq);
    }

    /* HQ2 presence magic (32-bit BE) read by Gr2Probe. */
    s->regs[SGI_GR2_HQ_MYSTERY + 0] = (SGI_GR2_HQ_MAGIC >> 24) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 1] = (SGI_GR2_HQ_MAGIC >> 16) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 2] = (SGI_GR2_HQ_MAGIC >> 8) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 3] = SGI_GR2_HQ_MAGIC & 0xff;

    /* Board version / config bytes decoded by Gr2Probe: one byte per 32-bit
     * slot at 0x6c000/4/8/c, in the addressed byte lane (the kernel probe
     * byte-reads them).  Per-instance so a variant can report its identity.
     * NOTE: storing them in the low lane instead was tried (to suit a 32-bit
     * ARCS read) and did NOT change the reported name, so it was reverted. */
    s->regs[SGI_GR2_BDVERS_OFF + 0] = s->bdvers0;
    s->regs[SGI_GR2_BDVERS_OFF + 4] = s->bdvers1;
    s->regs[SGI_GR2_BDVERS_OFF + 8] = s->bdvers2;
    s->regs[SGI_GR2_BDVERS_OFF + 12] = s->bdvers3;

    /* HQ2 revision register (read >> 16 into gr2_info.HQ2Rev). */
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 0] = 0x00;
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 1] = 0x01;

    /* P0.4 step (a): the synthetic scanout pattern, if the test asked for it.
     * The scanout region exists only when present (allocated in realize). */
    if (s->scanout && s->scanout_bars) {
        sgi_gr2_fill_bars(s);
    }
}

static void sgi_gr2_realize(DeviceState *dev, Error **errp)
{
    SGIGr2State *s = SGI_GR2(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_gr2_ops, s,
                          TYPE_SGI_GR2, SGI_GR2_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    /* GIO interrupt output (retrace → HPC3 "gio-retrace").  Always initialised;
     * the machine only connects it when the board is present. */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->retrace_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    sgi_gr2_retrace_tick, s);
    s->retrace_lower_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          sgi_gr2_retrace_lower, s);
    timer_mod(s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / SGI_GR2_RETRACE_HZ);

    /* Scanout (P0.4 step a) only when the board is present, so plain
     * `-M indy` gains no extra console. */
    if (s->present) {
        s->scanout = g_new0(uint32_t,
                            (size_t)SGI_GR2_SCREEN_W * SGI_GR2_SCREEN_H);
        s->con = graphic_console_init(dev, 0, &sgi_gr2_gfx_ops, s);
        qemu_console_resize(s->con, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
    }
}

static const Property sgi_gr2_properties[] = {
    DEFINE_PROP_BOOL("present", SGIGr2State, present, false),
    DEFINE_PROP_BOOL("scanout-bars", SGIGr2State, scanout_bars, false),
    DEFINE_PROP_UINT8("ges", SGIGr2State, ges, 2),
    DEFINE_PROP_UINT8("bitplanes", SGIGr2State, bitplanes, 24),
    DEFINE_PROP_BOOL("zbuffer", SGIGr2State, zbuffer, true),
    /* Per-variant board identity.  Defaults = XZ (2 GE, 24-bit + Z).  XS-24
     * sets bdvers1=0x10 (24-bit, no Z); XS sets bdvers1=0x00 (8-bit). */
    DEFINE_PROP_UINT8("bdvers0", SGIGr2State, bdvers0, SGI_GR2_BDVERS0),
    DEFINE_PROP_UINT8("bdvers1", SGIGr2State, bdvers1, SGI_GR2_BDVERS1),
    DEFINE_PROP_UINT8("bdvers2", SGIGr2State, bdvers2, SGI_GR2_BDVERS2),
    DEFINE_PROP_UINT8("bdvers3", SGIGr2State, bdvers3, SGI_GR2_BDVERS3),
};

static void sgi_gr2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_gr2_realize;
    device_class_set_legacy_reset(dc, sgi_gr2_reset);
    device_class_set_props(dc, sgi_gr2_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo sgi_gr2_type_info = {
    .name = TYPE_SGI_GR2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGr2State),
    .class_init = sgi_gr2_class_init,
};

static void sgi_gr2_register_types(void)
{
    type_register_static(&sgi_gr2_type_info);
}

type_init(sgi_gr2_register_types)
