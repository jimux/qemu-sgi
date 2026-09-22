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
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_gr2.h"

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
    /* GE units at or above the variant's engine count are not populated, so
     * the driver's GE-count pattern test stops counting there. */
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

static void sgi_gr2_reset(DeviceState *dev)
{
    SGIGr2State *s = SGI_GR2(dev);

    memset(s->regs, 0, sizeof(s->regs));

    /* HQ2 presence magic (32-bit BE) read by Gr2Probe. */
    s->regs[SGI_GR2_HQ_MYSTERY + 0] = (SGI_GR2_HQ_MAGIC >> 24) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 1] = (SGI_GR2_HQ_MAGIC >> 16) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 2] = (SGI_GR2_HQ_MAGIC >> 8) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 3] = SGI_GR2_HQ_MAGIC & 0xff;

    /* Board version / config bytes decoded by Gr2Probe: one byte per 32-bit
     * slot at 0x6c000/4/8/c. */
    s->regs[SGI_GR2_BDVERS_OFF + 0] = SGI_GR2_BDVERS0;
    s->regs[SGI_GR2_BDVERS_OFF + 4] = SGI_GR2_BDVERS1;
    s->regs[SGI_GR2_BDVERS_OFF + 8] = SGI_GR2_BDVERS2;
    s->regs[SGI_GR2_BDVERS_OFF + 12] = SGI_GR2_BDVERS3;

    /* HQ2 revision register (read >> 16 into gr2_info.HQ2Rev). */
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 0] = 0x00;
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 1] = 0x01;
}

static void sgi_gr2_realize(DeviceState *dev, Error **errp)
{
    SGIGr2State *s = SGI_GR2(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_gr2_ops, s,
                          TYPE_SGI_GR2, SGI_GR2_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static const Property sgi_gr2_properties[] = {
    DEFINE_PROP_BOOL("present", SGIGr2State, present, false),
    DEFINE_PROP_UINT8("ges", SGIGr2State, ges, 2),
    DEFINE_PROP_UINT8("bitplanes", SGIGr2State, bitplanes, 24),
    DEFINE_PROP_BOOL("zbuffer", SGIGr2State, zbuffer, true),
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
