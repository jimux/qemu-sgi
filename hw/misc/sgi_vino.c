/*
 * QEMU SGI VINO (Indy video input) — hardware-authentic register shell.
 *
 * VINO is the video-input ASIC on the Indy (IP24): the digital IndyCam port
 * plus composite/S-Video analog inputs digitised through an external video
 * decoder.  It is probed by the PROM and by IRIX at physical 0x00080000
 * (EISA space): a 32-bit version word at base+4 whose bits [7:4] identify
 * the part as 0xB (IRIX sys/IP22.h VINO_PHYS_BASE1 / VINO_CHIP_ID; IP22.c
 * simplevinoprobe()).  The PROM's own diagnostics also touch offsets 0x42,
 * 0x43 and 0x61.
 *
 * Presence is a device property, DEFAULT OFF.  With present=off the region
 * still exists but every read returns all-ones, which is exactly what an
 * unmapped aperture returns, so a plain `-M indy` invocation behaves
 * byte-for-byte as before (vinoProbe fails, VINO is simply absent).  With
 * present=on the version register answers and the PROM/kernel recognise the
 * part.  This is the "protect -M indy rigorously" gate: the host-source /
 * capture work is developed behind present=on until both the PROM boot and
 * the desktop gates pass on a plain invocation.
 *
 * Register map source: the IRIX 5.3 IP22boot kernel driver vino.a
 * (vino_regs.o master_reg_tbl; 39 direct-MMIO regs id 0..0x26 at stride 8;
 * ids 0x27..0x3d route to the analog decoder over VINO I2C; ids 0x3e..0x47
 * route to the IndyCam over VINO I2C).  Only the direct-MMIO shell and the
 * version register are modelled here; capture DMA / I2C / genlock follow.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_vino.h"
#include "qemu/log.h"

static uint64_t sgi_vino_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIVinoState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (!s->present) {
        /* Absent part: reads look exactly like an unmapped aperture. */
        return ~0ULL;
    }
    if (offset + size > SGI_VINO_REG_SIZE) {
        return ~0ULL;
    }
    /* Big-endian byte assembly, so byte/half/word accesses all agree. */
    for (i = 0; i < size; i++) {
        val = (val << 8) | s->regs[offset + i];
    }
    return val;
}

static void sgi_vino_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    SGIVinoState *s = opaque;
    unsigned i;

    if (!s->present || offset + size > SGI_VINO_REG_SIZE) {
        return;
    }
    for (i = 0; i < size; i++) {
        s->regs[offset + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
}

static const MemoryRegionOps sgi_vino_ops = {
    .read = sgi_vino_read,
    .write = sgi_vino_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sgi_vino_reset(DeviceState *dev)
{
    SGIVinoState *s = SGI_VINO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* Version register (base+4): VINO_CHIP_ID in bits [7:4], revision low. */
    s->regs[SGI_VINO_REV_OFFSET + 3] =
        (SGI_VINO_CHIP_ID << 4) | SGI_VINO_REVISION;
}

static void sgi_vino_realize(DeviceState *dev, Error **errp)
{
    SGIVinoState *s = SGI_VINO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sgi_vino_ops, s,
                          "sgi-vino", SGI_VINO_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property sgi_vino_properties[] = {
    DEFINE_PROP_BOOL("present", SGIVinoState, present, false),
};

static void sgi_vino_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_vino_realize;
    device_class_set_legacy_reset(dc, sgi_vino_reset);
    device_class_set_props(dc, sgi_vino_properties);
}

static const TypeInfo sgi_vino_types[] = {
    {
        .name          = TYPE_SGI_VINO,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(SGIVinoState),
        .class_init    = sgi_vino_class_init,
    },
};

DEFINE_TYPES(sgi_vino_types)
