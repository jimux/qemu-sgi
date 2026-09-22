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
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "trace.h"

/* One captured field per 60 Hz field period. */
#define SGI_VINO_FIELD_PERIOD_NS  (NANOSECONDS_PER_SECOND / 60)

/* Read/write one 32-bit VINO channel register by its byte offset relative to
 * the channel base.  The register file stores big-endian bytes, so go
 * through the byte array to stay access-size agnostic. */
static uint32_t sgi_vino_chreg(SGIVinoState *s, int ch, unsigned rel)
{
    hwaddr o = (ch ? SGI_VINO_CH_B_BASE : SGI_VINO_CH_A_BASE) + rel;

    return ((uint32_t)s->regs[o] << 24) | ((uint32_t)s->regs[o + 1] << 16) |
           ((uint32_t)s->regs[o + 2] << 8) | s->regs[o + 3];
}

static void sgi_vino_chreg_set(SGIVinoState *s, int ch, unsigned rel,
                               uint32_t val)
{
    hwaddr o = (ch ? SGI_VINO_CH_B_BASE : SGI_VINO_CH_A_BASE) + rel;

    s->regs[o] = val >> 24;
    s->regs[o + 1] = val >> 16;
    s->regs[o + 2] = val >> 8;
    s->regs[o + 3] = val;
}

/* Refill the four-entry descriptor cache from the auto-advancing fetch
 * pointer, then step it on by 16 bytes.  The fetch is traced so the driver's
 * real table can be checked against the spec. */
static void sgi_vino_fetch_group(SGIVinoState *s, int ch)
{
    uint32_t addr = s->next_desc[ch];
    int i;

    for (i = 0; i < SGI_VINO_DESC_PER_FETCH; i++) {
        s->desc[ch][i] = address_space_ldl_be(&address_space_memory,
                                              (hwaddr)addr + 4 * i,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
    }
    trace_sgi_vino_desc(ch, addr, s->desc[ch][0], s->desc[ch][1],
                        s->desc[ch][2], s->desc[ch][3]);
    s->next_desc[ch] = addr + 4 * SGI_VINO_DESC_PER_FETCH;
    s->cache_pos[ch] = 0;
    s->cache_valid[ch] = true;
}

/* Emit one captured field for a channel by walking its descriptor table.
 *
 * Per the VINO Design Specification 099-8937-001 section 2.3, the field's
 * height is signalled by the video decoder's end-of-field, not by a register,
 * and the line_size/line_count registers describe bytes within a line
 * (line_count is a running byte counter that starts at zero).  So a field is
 * modelled as one pass over the descriptors the driver built for it: each
 * descriptor names a 4K page that capture DMA fills, a stop descriptor ends
 * the pass with an EOD interrupt, and end of pass raises EOF.  The pixels are
 * a deterministic luma ramp -- nothing is presented as real camera data, but
 * a full field is delivered so the driver's DMA completes.  Recorded host
 * frames replace this once the sgi-video-source seam lands. */
static void sgi_vino_emit_field(SGIVinoState *s, int ch)
{
    uint32_t bytes = 0;
    unsigned pages;

    for (pages = 0; pages < 512; pages++) {
        uint32_t desc, page, off;

        if (!s->cache_valid[ch]) {
            sgi_vino_fetch_group(s, ch);
        }
        desc = s->desc[ch][s->cache_pos[ch]];
        if (desc & SGI_VINO_DESC_STOP) {
            /* Stop descriptor: end of the descriptor table. */
            s->regs[SGI_VINO_INTSTAT_OFFSET + 3] |= ch ? SGI_VINO_INT_B_EOD
                                                       : SGI_VINO_INT_A_EOD;
            s->dma_en[ch] = false;
            sgi_vino_chreg_set(s, ch, SGI_VINO_CH_PAGE_INDEX, 0);
            break;
        }
        if (desc & SGI_VINO_DESC_JUMP) {
            /* Jump descriptor: next fetch resumes at its page address. */
            s->next_desc[ch] = desc & 0x3ffff000u;
            s->cache_valid[ch] = false;
            continue;
        }
        page = desc & 0x3ffff000u;
        if (!page) {
            break;
        }
        for (off = 0; off < 0x1000; off += 4) {
            uint8_t g = (off >> 2) & 0xff;
            uint32_t px = ((uint32_t)g << 24) | ((uint32_t)g << 16) |
                          ((uint32_t)g << 8) | g;

            address_space_stl_be(&address_space_memory, page | off, px,
                                 MEMTXATTRS_UNSPECIFIED, NULL);
        }
        bytes += 0x1000;
        sgi_vino_chreg_set(s, ch, SGI_VINO_CH_PAGE_INDEX, 0);

        if (++s->cache_pos[ch] >= SGI_VINO_DESC_PER_FETCH) {
            s->cache_valid[ch] = false;
        }
    }

    s->field_count[ch]++;
    sgi_vino_chreg_set(s, ch, SGI_VINO_CH_FIELD_COUNT, s->field_count[ch]);
    s->regs[SGI_VINO_INTSTAT_OFFSET + 3] |= ch ? SGI_VINO_INT_B_EOF
                                               : SGI_VINO_INT_A_EOF;
    trace_sgi_vino_field(ch, s->field_count[ch], bytes);
}

static void sgi_vino_field_tick(void *opaque)
{
    SGIVinoState *s = opaque;
    bool any = false;
    int ch;

    for (ch = 0; ch < 2; ch++) {
        if (s->dma_en[ch]) {
            sgi_vino_emit_field(s, ch);
            any = true;
        }
    }
    if (any) {
        timer_mod(s->field_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  SGI_VINO_FIELD_PERIOD_NS);
    }
}

/* Arm or stop the field timer to match the set of DMA-enabled channels. */
static void sgi_vino_update_timer(SGIVinoState *s)
{
    if (s->dma_en[0] || s->dma_en[1]) {
        if (!timer_pending(s->field_timer)) {
            timer_mod(s->field_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SGI_VINO_FIELD_PERIOD_NS);
        }
    } else {
        timer_del(s->field_timer);
    }
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
        /* The driver programmed the descriptor table; the engine fetches it
         * (four at a time, auto-advancing) once capture is enabled. */
        int ch = offset >= SGI_VINO_CH_B_BASE ? 1 : 0;

        s->next_desc[ch] = sgi_vino_chreg(s, ch, SGI_VINO_CH_NEXT4DESC);
        s->cache_valid[ch] = false;
        s->cache_pos[ch] = 0;
    } else if (offset == SGI_VINO_CONTROL_OFFSET) {
        uint32_t ctl = 0;
        int ch;

        for (i = 0; i < 4; i++) {
            ctl = (ctl << 8) | s->regs[offset + i];
        }
        for (ch = 0; ch < 2; ch++) {
            bool en = ctl & (ch ? SGI_VINO_CTRL_B_DMA_ENBL
                                : SGI_VINO_CTRL_A_DMA_ENBL);

            if (en && !s->dma_en[ch]) {
                /* Capture start: the cache refills from next_4_desc (fall
                 * back to start_desc if the driver only set that). */
                if (!s->next_desc[ch]) {
                    s->next_desc[ch] =
                        sgi_vino_chreg(s, ch, SGI_VINO_CH_START_DESC);
                }
                s->cache_valid[ch] = false;
                s->cache_pos[ch] = 0;
                s->dma_en[ch] = true;
            } else if (!en) {
                s->dma_en[ch] = false;
            }
        }
        sgi_vino_update_timer(s);
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
    s->dma_en[0] = s->dma_en[1] = false;
    s->cache_valid[0] = s->cache_valid[1] = false;
    s->cache_pos[0] = s->cache_pos[1] = 0;
    s->next_desc[0] = s->next_desc[1] = 0;
    s->field_count[0] = s->field_count[1] = 0;
    if (s->field_timer) {
        timer_del(s->field_timer);
    }
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
    s->field_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_vino_field_tick, s);
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
