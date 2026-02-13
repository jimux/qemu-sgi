/*
 * SGI CRIME Rendering Engine (RE) stub
 *
 * Minimal stub for the CRIME rendering engine at physical 0x15000000.
 * The IRIX kernel probes this during early boot (mlreset) and the graphics
 * driver accesses it heavily. This stub reports all subsystems as idle
 * and accepts writes silently, allowing boot to proceed.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/sgi_crime_re.h"
#include "migration/vmstate.h"

static uint64_t sgi_crime_re_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case CRM_RE_STATUS_REG:
        /* Report all subsystems idle, buffer empty */
        return CRMSTAT_ALL_IDLE;

    case CRM_RE_INTFBUF_ADDR0:
        /*
         * crmGetRERev reads bit 0 of the interface buffer address register
         * at offset 0x400 to determine RE revision. Return 0 (rev 0).
         */
        return 0;

    default:
        /*
         * All other reads return 0. This covers:
         * - TLB entries (empty)
         * - Pixel pipe registers (default/disabled)
         * - MTE registers (idle)
         * - Interface buffer data (empty)
         */
        return 0;
    }
}

static void sgi_crime_re_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    /*
     * Accept all writes silently. The kernel and graphics driver write
     * to TLB entries, pixel pipe registers, MTE configuration, etc.
     * Since we don't actually render anything, just absorb the writes.
     */
    switch (offset) {
    case CRM_RE_STATUS_REG:
    case CRM_RE_STATUS_BASE + 0x08: /* SET_STARTPTR */
        qemu_log_mask(LOG_UNIMP,
                      "sgi_crime_re: write status/ctrl offset 0x%03"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, value);
        break;

    default:
        /* Silently absorb TLB, pixel pipe, MTE, FIFO writes */
        break;
    }
}

static const MemoryRegionOps sgi_crime_re_ops = {
    .read = sgi_crime_re_read,
    .write = sgi_crime_re_write,
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

static void sgi_crime_re_realize(DeviceState *dev, Error **errp)
{
    SGICRIMEREState *s = SGI_CRIME_RE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_crime_re_ops, s,
                          "sgi-crime-re", CRIME_RE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void sgi_crime_re_reset(DeviceState *dev)
{
    SGICRIMEREState *s = SGI_CRIME_RE(dev);
    s->status = CRMSTAT_ALL_IDLE;
}

static const VMStateDescription vmstate_sgi_crime_re = {
    .name = "sgi-crime-re",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(status, SGICRIMEREState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_crime_re_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_crime_re_realize;
    device_class_set_legacy_reset(dc, sgi_crime_re_reset);
    dc->vmsd = &vmstate_sgi_crime_re;
}

static const TypeInfo sgi_crime_re_info = {
    .name = TYPE_SGI_CRIME_RE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGICRIMEREState),
    .class_init = sgi_crime_re_class_init,
};

static void sgi_crime_re_register_types(void)
{
    type_register_static(&sgi_crime_re_info);
}

type_init(sgi_crime_re_register_types)
