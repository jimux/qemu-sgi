/*
 * SGI GR2 / "Express" graphics (Indy XZ, Indigo XS/XZ, Indigo2, Everest) —
 * register/FIFO shell.
 *
 * GR2 is SGI's second-generation workstation graphics family: a host-queue
 * ASIC (HQ2) feeding 1-8 microcode-programmed GE7 geometry engines, whose
 * output a RE3 raster engine displays through a VC1 video controller, five
 * XMAP5 mode generators and three BT457 RAMDACs.  It sits on GIO64: board 0
 * at 0x1f000000, board 1 at 0x1f400000.  The software contract is the
 * token-addressed command FIFO (fifo[token] = data) plus the shared-RAM
 * mailbox, not raw GE/RE register writes; the kernel's stock `gr2` driver
 * probes the board by reading the HQ2 presence magic 0xdeadbeef at 0x6a07c.
 *
 * Presence is a device property, default OFF: with present=off the region is
 * not mapped at all, so a plain `-M indy` invocation is byte-for-byte
 * unchanged (the GIO slot still reads all-ones).  present=on maps the board
 * shell so the PROM/kernel GR2 probe recognises the part.
 *
 * Register map is the authoritative `struct gr2_hw` in sys/gr2hw.h (5.3
 * headers) cross-checked against the 6.5 driver binary (gr2.a).  See
 * progress_notes/gfx_drivers_re/03-gr2-express.md.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_GR2_H
#define HW_MISC_SGI_GR2_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"
#include "qemu/timer.h"

#define TYPE_SGI_GR2 "sgi-gr2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGr2State, SGI_GR2)

/* Scanout geometry for the P0.4 step-(a) synthetic test (matches Newport so a
 * later region-by-region comparison against it is size-compatible). */
#define SGI_GR2_SCREEN_W 1280
#define SGI_GR2_SCREEN_H 1024

/* Byte offsets within the GIO64 board window (gr2hw.h).  The board map is
 * 0x6d000 bytes (GR2_GFX_SIZE); round the aperture up to 0x70000. */
#define SGI_GR2_REG_SIZE    0x70000

#define SGI_GR2_SHRAM_OFF   0x00000 /* 128 KB shared command/constant RAM */
#define SGI_GR2_FIFO_OFF    0x40000 /* 128 KB token-addressed command FIFO */
#define SGI_GR2_HQUCODE_OFF 0x60000 /* 8 K x 32-bit HQ2 microcode RAM */
#define SGI_GR2_GE_OFF      0x68000 /* GE7 units: 8 x 1 KB RAM0, 0x400 stride */
#define SGI_GR2_GE_STRIDE   0x400
#define SGI_GR2_GE_UNITS    8

#define SGI_GR2_HQ_OFF      0x6a000 /* HQ2 register block (mystery at 0x7c) */
#define SGI_GR2_HQ_MYSTERY  0x6a07c /* presence magic, read by Gr2Probe */
#define SGI_GR2_HQ_MAGIC    0xdeadbeefu

/* HQ2 registers, from the 6.5 gr2_init.o disassembly (Gr2FIFOHandler,
 * _Gr2HQ2RegInit, Gr2DownloadHQ2/GE7).  The FIFO status at 0x6a040 carries
 * the current occupancy in bits 6..12 (mask 0x1fc0) and an error flag in bit
 * 5; the driver polls it before/while draining the token FIFO, so it must read
 * as read-only and empty (0), not as whatever the bus last wrote. */
#define SGI_GR2_HQ_FIFOSTAT    0x6a040
#define SGI_GR2_HQ_IDLE_BIT    0x1     /* bit 0: HQ2 idle/ready (Xsgi polls) */
#define SGI_GR2_HQ_READY_BIT   0x2     /* bit 1: ucode ready (polled)       */
#define SGI_GR2_HQ_TOKEN_START 0x4077c /* FIFO token Gr2Start writes to run */
#define SGI_GR2_RE3_COLOUR_TOKEN 0x40530 /* FIFO token: RE3 pixel colour    */
#define SGI_GR2_PUC_COLOR_TOKEN   0x40648 /* FIFO token: PUC_COLOR (index 402) */
#define SGI_GR2_PUC_RECTI2D_TOKEN 0x40654 /* FIFO token: PUC_RECTI2D (405)    */
/* Fill-op markers, read off the X DDX's own FIFO stores (token = offset/4;
 * see progress_notes/indy/xz-gr2/25-*.md and 26-*.md).  Which shape an op
 * draws is named by the marker token it writes, not by the PUC_DATA tail. */
#define SGI_GR2_RE3_SOLID_TOKEN   0x404c0 /* token 304: expDrawSolidRects    */
#define SGI_GR2_RE3_SPANS_TOKEN   0x404c4 /* token 305: expSolidSpans        */
#define SGI_GR2_RE3_FG_TOKEN      0x404e8 /* token 314: stipple fg colour    */
#define SGI_GR2_RE3_STIPPLE_TOKEN 0x404f8 /* token 318: stipple pattern      */
#define SGI_GR2_RE3_OP_TOKEN      0x4052c /* token 331: op/mode              */
#define SGI_GR2_RE3_DONE_TOKEN    0x407a8 /* token 490: op terminator        */
#define SGI_GR2_HQ_NUMGE       0x6a044
#define SGI_GR2_HQ_FIFO_FULL_T 0x6a054 /* full-timeout, driver writes 100    */
#define SGI_GR2_HQ_FIFO_EMPTY_T 0x6a058 /* empty-timeout                     */
#define SGI_GR2_HQ_FIFO_FULL   0x6a05c /* level at which the FIFO is "full" */
#define SGI_GR2_HQ_FIFO_EMPTY  0x6a060 /* level at which the FIFO is "empty"*/
#define SGI_GR2_HQ_UCODELOAD   0x6a064 /* GE7 ucode load data register      */
#define SGI_GR2_HQ_VERSION     0x6a06c /* HQ2 revision, read >> 16          */
#define SGI_GR2_HQ_GEPC        0x6a070 /* GE7 program counter / load address*/
/* The 6.5 driver stages the HQ2 microcode through 0x6a000..0x6a03f (16
 * words) — i.e. immediately below the HQ2 status register at 0x6a040 — not
 * at the 5.3 header's 0x60000 window.  The 6.5 binary is authoritative. */
#define SGI_GR2_HQ_UC_RAM      0x6a000 /* HQ2 ucode load staging (16 words) */

/* GE7 instruction load/verify.  Gr2DownloadGE7 writes a record for a PC into
 * the four-word window at ge[0].ram0[0xf8..0xfb] plus the load register
 * 0x6a064, having first written the PC to gepc (0x6a070); the verify pass
 * re-selects each PC and reads the words back, so all five words are per-PC
 * storage addressed by the current gepc.  Values are stored and returned
 * verbatim (the driver masks with 0x3dfffff itself), never tidied. */
#define SGI_GR2_GE_WIN_OFF     0x683e0 /* ge[0].ram0[0xf8..0xfb]            */
#define SGI_GR2_GE_WIN_WORDS   4
#define SGI_GR2_UCODE_PCS      0x10000 /* PC space the driver walks         */
#define SGI_GR2_UCODE_WORDS    5       /* 4 window words + 1 load register  */
#define SGI_GR2_FIN3_OFF    0x6b000 /* HQ2 fin3 register */
#define SGI_GR2_BDVERS_OFF  0x6c000 /* board version / config / video backend */
#define SGI_GR2_VC1_OFF     0x6c040 /* VC1 video controller */
#define SGI_GR2_XMAP_OFF    0x6c100 /* XMAP5 display-mode generators */
#define SGI_GR2_XMAP_STATUS 0x6c11c /* bit 1 = XMAP ready (polled)       */
#define SGI_GR2_XMAP_READY_BIT 0x2
#define SGI_GR2_XMAP_CTL_OFF  0x6c1a0 /* control/data regs written at init */
#define SGI_GR2_XMAP_CTL_END  0x6c1b8
/* RAMDAC colour-map programming, inside the XMAP control window.  The DDX
 * writes each entry as: the INDEX to 0x6c1b0, a control byte to 0x6c1b4, then a
 * sliding byte stream on 0x6c1a8 whose bytes are R,G,B of the addressed entry
 * followed by the R of the next.  Read off the boot device trace: e.g.
 *   0x6c1b0=0x21 ; 0x6c1b4=0x11 ; 0x6c1a8=0x1414141e  -> entry 0x21 = 0x141414,
 *   and 0x1e begins entry 0x22 (whose index write follows).  The palette is
 * therefore BUILT from the guest's own writes, not a static table. */
#define SGI_GR2_XMAP_PAL_DATA  0x6c1a8 /* R,G,B byte stream (sliding)      */
#define SGI_GR2_XMAP_PAL_INDEX 0x6c1b0 /* entry index being programmed     */
#define SGI_GR2_XMAP_PAL_CTL   0x6c1b4 /* control byte written per entry   */
/* The control byte is the high byte of the DDX's per-entry word (the DDX does
 * `index = entry & 0xff; control = entry >> 8` in expStoreNormalColors), so it
 * tags an entry with its kind, not a whole-map bank.  Replaying the default
 * boot shows why the "installed map" is not a single value:
 *   - entries 96..255 (the root weave) are written under 0x10 and NOWHERE else,
 *     so accepting only 0x11 renders the weave black;
 *   - but the root weave (0x10) and the colour ramp (0x11) are on screen
 *     together, so they must share one map, not two.
 * Committing 0x10 and 0x11 with last-write-wins resolves both: for every index
 * that has both, 0x11 is written last, so the `xwd` oracle is preserved
 * (4sight 18/18, default pseudomap 10/10), while the weave indices come in.
 * 0x00/0x01/0x1c are the overlay / pup / 24-bit maps (expStoreOverlayColors,
 * expStorePupColors, expStore24Colors) and must not touch the visible palette —
 * index 1's last 0x1c write would otherwise overwrite its red.  */
#define SGI_GR2_XMAP_PAL_BANK_INSTALLED 0x11 /* normal map, written last */
#define SGI_GR2_XMAP_PAL_BANK_ALT       0x10 /* same map; the weave lives here */
#define SGI_GR2_RE3_27_OFF  0x6c200 /* RE3 buffered register set */
#define SGI_GR2_RE3_24_OFF  0x6c280 /* RE3 unbuffered register set */
#define SGI_GR2_RE3_32_OFF  0x6c600 /* RE3 32-bit register */

/* Board-version bytes, one per 32-bit slot at 0x6c000/4/8/c.  The probe reads
 * the byte at 0x6c000 for the revision (active-low nibble) and the byte at
 * 0x6c004 for the config: its low two bits must NOT be 3 (or the probe
 * rejects the board), bit 4 selects 24-bit and bit 5 selects the Z buffer.
 * Values below pick the XZ (2 GE, 24-bit + Z) configuration. */
#define SGI_GR2_BDVERS0     0x06 /* rev slot: -> rev = (~0x06) & 0xf = 9 */
#define SGI_GR2_BDVERS1     0x30 /* cfg slot: low2=0, bit4=24-bit, bit5=Z */
#define SGI_GR2_BDVERS2     0x00
#define SGI_GR2_BDVERS3     0x00

struct SGIGr2State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint8_t regs[SGI_GR2_REG_SIZE];
    bool present;

    /* GE7 instruction storage, addressed by the current gepc: 5 words per
     * PC (the ge[0].ram0[0xf8..0xfb] window plus the load register). */
    uint32_t ucode[SGI_GR2_UCODE_PCS][SGI_GR2_UCODE_WORDS];
    uint32_t gepc;
    bool hq_ready;
    bool xmap_ready;

    /* Scanout (P0.4 step a): a QEMU display surface proving the output stage
     * before the RE3 producer.  `scanout` is the framebuffer VC1 would scan
     * (host xRGB32); VC1/RE3 will read it once the producer is emulated. */
    QemuConsole *con;
    uint32_t *scanout;
    bool scanout_bars; /* fill a colour-bar test pattern (P0.4 step a) */

    /* RE3 producer colour latch + RAMDAC palette (8-bit mode).  In the 8-bit
     * mode the guest runs (xwininfo: depth 8 PseudoColor), the RE3 fill colour
     * the DDX writes is a palette INDEX; the RAMDAC maps index -> RGB for
     * scanout.  `scanout` is the framebuffer the RE3 fill writes and VC1 scans
     * out, so the producer and the output stage meet in one buffer.
     *
     * The palette is BUILT from the guest's own RAMDAC programming (see the
     * XMAP_PAL_* registers above) rather than a static table: the DDX programs
     * it at server start, so nothing is assumed and a 4sight pseudomap and a
     * default pseudomap each come out as the server meant them. */
    uint32_t ramdac[256];
    uint8_t ramdac_index;      /* entry selected by XMAP_PAL_INDEX          */
    uint8_t ramdac_ctl;        /* bank select from XMAP_PAL_CTL             */
    uint8_t ramdac_stage[3];   /* R,G,B bytes accumulating from PAL_DATA     */
    unsigned ramdac_stage_n;   /* bytes held (0..2)                          */
    uint8_t re3_colour;   /* last colour latched from the RE3 colour token */
    bool re3_colour_valid;
    uint32_t last_puc;    /* previous PUC_DATA word (rect geometry pair)     */
    bool last_puc_valid;
    /* Generic PUC draw path: PUC_COLOR + PUC_RECTI2D + three PUC_DATA words.
     * [ASSUMPTION, from the captured root-weave stream: 1024 rects of
     * (colour, x0, 0x4ff, x0) — colour indices running 96..255 while x0 and the
     * third word run 0..1023 and the middle word is always 1279, so the three
     * data words read as a horizontal span (x0, x1, y0).] */
    uint8_t puc_colour;
    bool puc_rect_armed;  /* a PUC_RECTI2D is awaiting its three data words  */
    uint32_t puc_rect[3];
    unsigned puc_rect_n;  /* data words collected for the armed rectangle    */

    /* Fill-op markers (note 25/26).  A full-screen rect is drawn stippled when
     * token 318 armed a pattern, flat when token 304 marked a solid rect. */
    bool re3_solid_seen;    /* token 304 seen in the current op              */
    bool re3_spans_seen;    /* token 305 seen in the current op              */
    bool re3_stipple_valid; /* token 318 written: next rect is stippled      */
    uint32_t re3_stipple;   /* 32-bit stipple pattern (token 318)            */
    uint8_t re3_fg;         /* token 314: stipple foreground colour index    */
    bool re3_fg_valid;

    /* Variant params supplied by the machine glue. */
    uint8_t ges;       /* number of GE7 engines (1, 2, 4, 8) */
    uint8_t bitplanes; /* 8 or 24 */
    bool zbuffer;
    /* Board-version/config bytes, one per 32-bit slot at 0x6c000/4/8/c.  These
     * are per-instance so a variant can report its own identity: XZ defaults
     * to 0x06/0x30/0x00/0x00 (24-bit + Z); XS-24 is 0x06/0x10/0x00/0x00
     * (24-bit, no Z).  See the BDVERS defaults below. */
    uint8_t bdvers0;
    uint8_t bdvers1;
    uint8_t bdvers2;
    uint8_t bdvers3;

    /* GIO interrupt output.  The kernel driver registers its retrace handler
     * with setgiovector on GIO vector 2 — the same line the HPC3 exposes as
     * its "gio-retrace" input, which is how Newport's VRINT is delivered.  A
     * real CRT always has a vertical retrace, so the board raises it at ~60 Hz
     * once it has been started; the pulse is asserted then lowered after the
     * blanking interval, mirroring Newport's VBLANK model.  Without it the
     * driver's interrupt-driven paths never run and Xsgi wedges. */
    qemu_irq irq;
    QEMUTimer *retrace_timer;
    QEMUTimer *retrace_lower_timer;
    bool retrace_active;
};

#endif /* HW_MISC_SGI_GR2_H */
