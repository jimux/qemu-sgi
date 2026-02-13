/*
 * SGI GBE (Graphics Back End) emulation
 *
 * Minimal register handling to support IRIX kernel boot.
 *
 * Each DMA channel has two registers:
 *   CTRL (write): software sets the new value
 *   INHWCTRL (read): returns the value active in hardware
 * On real hardware, CTRL propagates to INHWCTRL at VSync.
 * We reflect writes immediately (instant VSync).
 *
 * The i2cfp register returns 0 so that the kernel's I2C bit-bang
 * reads inverted values (SDA=1, SCL=1 = bus idle, no panel).
 *
 * Physical base: 0x16000000 (kseg1: 0xB6000000)
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/display/sgi_gbe.h"
#include "migration/vmstate.h"

/*
 * Simulate the GBE video timing raster position.
 *
 * VT_XY format: bit[31] = freeze, bits[23:12] = Y, bits[11:0] = X
 * Standard 1024x768@60Hz timing (total including blanking):
 *   H total = 1344 pixels, V total = 806 lines
 *   H blank starts at 1024, V blank starts at 768
 *
 * Each read advances the simulated position so that polling loops
 * (like waitForBlanking) see the raster sweep through the frame.
 */
#define GBE_HTOTAL  1344
#define GBE_VTOTAL  806

static uint32_t sgi_gbe_get_vt_xy(SGIGBEState *s)
{
    uint32_t x, y;

    if (s->vt_frozen) {
        return s->vt_xy | 0x80000000;
    }

    /*
     * Advance the simulated raster position. Increment Y by a few
     * lines per read to sweep through the frame quickly. This ensures
     * waitForBlanking() sees the raster enter the blanking region
     * within a reasonable number of reads.
     */
    s->vt_read_count += 7;  /* ~7 lines per read */
    x = (s->vt_read_count * 37) % GBE_HTOTAL;  /* pseudo-random X */
    y = s->vt_read_count % GBE_VTOTAL;

    return (y << 12) | x;
}

static uint64_t sgi_gbe_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIGBEState *s = SGI_GBE(opaque);

    switch (offset) {
    case GBE_CTRLSTAT:
        return s->ctrlstat;

    case GBE_DOTCLOCK:
        return s->dotclock;

    case GBE_ID:
        return 0;

    case GBE_I2C:
    case GBE_I2CFP:
        return 0;

    case GBE_SYSCLK:
        return 0;

    case GBE_VT_XY:
        return sgi_gbe_get_vt_xy(s);

    case GBE_VT_XYMAX:
        return s->vt_xymax;

    /* OVR channel */
    case GBE_OVR_WIDTH_TILE:
        return 0;
    case GBE_OVR_INHWCTRL:
        /* Return control value with DMA enable (bit 0) cleared.
         * No real DMA engine, so DMA is always "complete". */
        return s->ovr_control & ~1u;
    case GBE_OVR_CTRL:
        return s->ovr_control;

    /* FRM channel */
    case GBE_FRM_SIZE_TILE:
    case GBE_FRM_SIZE_PIXEL:
        return 0;
    case GBE_FRM_INHWCTRL:
        /* DMA enable is bit 0 for FRM too */
        return s->frm_control & ~1u;
    case GBE_FRM_CTRL:
        return s->frm_control;

    /* DID channel */
    case GBE_DID_INHWCTRL:
        /* DID DMA enable is bit 16 */
        return s->did_control & ~(1u << 16);
    case GBE_DID_CTRL:
        return s->did_control;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_gbe: read at offset 0x%06" HWADDR_PRIx
                      " (size %d)\n", offset, size);
        return 0;
    }
}

static void sgi_gbe_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    SGIGBEState *s = SGI_GBE(opaque);

    switch (offset) {
    case GBE_CTRLSTAT:
        s->ctrlstat = (uint32_t)value;
        break;

    case GBE_DOTCLOCK:
        s->dotclock = (uint32_t)value;
        break;

    case GBE_VT_XY:
        s->vt_frozen = (value & 0x80000000) != 0;
        if (!s->vt_frozen) {
            s->vt_read_count = 0;
        }
        s->vt_xy = (uint32_t)value;
        break;

    case GBE_VT_XYMAX:
        s->vt_xymax = (uint32_t)value;
        break;

    /* OVR channel: both CTRL and INHWCTRL writes accepted */
    case GBE_OVR_CTRL:
    case GBE_OVR_INHWCTRL:
        s->ovr_control = (uint32_t)value;
        break;

    /* FRM channel: both CTRL and INHWCTRL writes accepted */
    case GBE_FRM_CTRL:
    case GBE_FRM_INHWCTRL:
        s->frm_control = (uint32_t)value;
        break;

    /* DID channel: both CTRL and INHWCTRL writes accepted */
    case GBE_DID_CTRL:
    case GBE_DID_INHWCTRL:
        s->did_control = (uint32_t)value;
        break;

    case GBE_SYSCLK:
    case GBE_I2C:
    case GBE_I2CFP:
    case GBE_FRM_SIZE_TILE:
    case GBE_FRM_SIZE_PIXEL:
    case GBE_OVR_WIDTH_TILE:
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_gbe: write at offset 0x%06" HWADDR_PRIx
                      " value 0x%08" PRIx64 " (size %d)\n",
                      offset, value, size);
        break;
    }
}

static const MemoryRegionOps sgi_gbe_ops = {
    .read = sgi_gbe_read,
    .write = sgi_gbe_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void sgi_gbe_realize(DeviceState *dev, Error **errp)
{
    SGIGBEState *s = SGI_GBE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_gbe_ops, s,
                          "sgi-gbe", GBE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void sgi_gbe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_gbe_realize;
}

static const TypeInfo sgi_gbe_info = {
    .name = TYPE_SGI_GBE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGBEState),
    .class_init = sgi_gbe_class_init,
};

static void sgi_gbe_register_types(void)
{
    type_register_static(&sgi_gbe_info);
}

type_init(sgi_gbe_register_types)
