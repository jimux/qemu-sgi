/*
 * SGI HPC1 (High Performance Controller) for IP20 (Indigo) — Stub
 *
 * The HPC1 is the peripheral controller for the original SGI Indigo (IP20).
 * It predates HPC3 and has a different register layout. This stub allows
 * compilation and machine instantiation but peripheral I/O is not functional.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/misc/sgi_hpc1.h"
#include "chardev/char-fe.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"

#define SGI_HPC1_MMIO_SIZE 0x10000

static uint64_t sgi_hpc1_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "sgi-hpc1: unimplemented read at 0x%" HWADDR_PRIx "\n",
                  addr);
    return 0;
}

static void sgi_hpc1_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "sgi-hpc1: unimplemented write at 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, val);
}

static const MemoryRegionOps sgi_hpc1_ops = {
    .read = sgi_hpc1_read,
    .write = sgi_hpc1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sgi_hpc1_realize(DeviceState *dev, Error **errp)
{
    SGIHPC1State *s = SGI_HPC1(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sgi_hpc1_ops, s,
                          "sgi-hpc1", SGI_HPC1_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    qdev_init_gpio_out_named(dev, s->cpu_irq, "cpu-irq", 2);
    qdev_init_gpio_out_named(dev, s->timer_irq, "timer-irq", 2);
}

static const Property sgi_hpc1_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIHPC1State, serial),
    DEFINE_PROP_STRING("nvram", SGIHPC1State, nvram_filename),
};

static void sgi_hpc1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_hpc1_realize;
    device_class_set_props(dc, sgi_hpc1_properties);
}

static const TypeInfo sgi_hpc1_info = {
    .name = TYPE_SGI_HPC1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHPC1State),
    .class_init = sgi_hpc1_class_init,
};

static void sgi_hpc1_register_types(void)
{
    type_register_static(&sgi_hpc1_info);
}

type_init(sgi_hpc1_register_types)
