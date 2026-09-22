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
#include "system/address-spaces.h"
#include "trace.h"

/* The VINO DMA engine reads its descriptor table from guest memory.  Fetch
 * the four 32-bit descriptors pointed at by next_4_desc and record them so
 * the driver's real table can be checked against the spec. */
static void sgi_vino_fetch_desc(SGIVinoState *s, int ch, uint32_t addr)
{
    int i;

    for (i = 0; i < SGI_VINO_DESC_PER_FETCH; i++) {
        s->desc[ch][i] = address_space_ldl_be(&address_space_memory,
                                              (hwaddr)addr + 4 * i,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
    }
    trace_sgi_vino_desc(ch, addr, s->desc[ch][0], s->desc[ch][1],
                        s->desc[ch][2], s->desc[ch][3]);
}

static uint64_t sgi_vino_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIVinoState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (!s->present) {
        /* Absent part: reads look exactly like an unmapped aperture. */
        trace_sgi_vino_read(offset, size, ~0ULL, 0);
        return ~0ULL;
    }
    if (offset + size > SGI_VINO_REG_SIZE) {
        trace_sgi_vino_read(offset, size, ~0ULL, 1);
        return ~0ULL;
    }
    if (offset == SGI_VINO_I2C_CTRL_OFFSET) {
        /* Status: idle, so all of busy/xfer/ack/error read clear.  The driver
         * polls these bits and only cares that a transfer completes. */
        val = 0;
    } else if (offset == SGI_VINO_I2C_DATA_OFFSET) {
        /* Data byte from the addressed slave, advancing the sub-address. */
        uint8_t *f = s->i2c_alt_dev ? s->i2c_alt : s->i2c_dec;
        val = f[s->i2c_ptr++];
    } else {
        /* Big-endian byte assembly, so byte/half/word accesses all agree. */
        for (i = 0; i < size; i++) {
            val = (val << 8) | s->regs[offset + i];
        }
    }
    trace_sgi_vino_read(offset, size, val, 1);
    return val;
}

static void sgi_vino_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    SGIVinoState *s = opaque;
    unsigned i;

    if (!s->present || offset + size > SGI_VINO_REG_SIZE) {
        trace_sgi_vino_write(offset, value, size, s->present ? 1 : 0);
        return;
    }
    trace_sgi_vino_write(offset, value, size, 1);
    if (offset == SGI_VINO_I2C_CTRL_OFFSET) {
        /* Non-zero arms a byte stream; zero forces the bus idle.  Either way
         * the next data byte starts with the device address. */
        s->i2c_first = 1;
        return;
    }
    if (offset == SGI_VINO_I2C_DATA_OFFSET) {
        uint8_t byte = value & 0xff;
        uint8_t *f = s->i2c_alt_dev ? s->i2c_alt : s->i2c_dec;

        if (s->i2c_first) {
            /* Device address byte; bit 0 selects read vs write. */
            s->i2c_alt_dev = ((byte & 0xfe) != SGI_VINO_I2C_ADDR_WRITE);
            s->i2c_first = 0;
            s->i2c_expect_sub = !(byte & 1);
        } else if (s->i2c_expect_sub) {
            s->i2c_ptr = byte;
            s->i2c_expect_sub = 0;
        } else {
            f[s->i2c_ptr++] = byte;
        }
        return;
    }
    for (i = 0; i < size; i++) {
        s->regs[offset + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
    if (offset == SGI_VINO_CH_A_BASE + SGI_VINO_CH_NEXT4DESC ||
        offset == SGI_VINO_CH_B_BASE + SGI_VINO_CH_NEXT4DESC) {
        int ch = offset >= SGI_VINO_CH_B_BASE ? 1 : 0;
        uint32_t addr = 0;

        for (i = 0; i < 4; i++) {
            addr = (addr << 8) | s->regs[offset + i];
        }
        s->dma_base[ch] = addr;
        if (addr) {
            sgi_vino_fetch_desc(s, ch, addr);
        }
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
    memset(s->dma_base, 0, sizeof(s->dma_base));
    memset(s->desc, 0, sizeof(s->desc));
    memset(s->i2c_dec, 0, sizeof(s->i2c_dec));
    memset(s->i2c_alt, 0, sizeof(s->i2c_alt));
    s->i2c_ptr = 0;
    s->i2c_first = 1;
    s->i2c_expect_sub = 0;
    s->i2c_alt_dev = 0;
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
