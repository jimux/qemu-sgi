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
