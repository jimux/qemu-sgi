/*
 * SGI Paravirtual 66MHz Free-Running Timer
 *
 * Provides a free-running 64-bit counter incrementing at 66MHz.
 * The PROM's usecdelay.s resets the counter by writing 0 to offset 0x38,
 * then polls until the elapsed count exceeds (usec * 66).
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "hw/misc/sgi_pvtimer.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * Return current 66MHz ticks relative to the stored base.
 * 66 ticks/us = 66/1000 ticks/ns.
 */
static inline uint64_t pvtimer_ticks(SGIPVTimerState *s)
{
    uint64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return (ns * 66) / 1000 - s->base;
}

static uint64_t sgi_pvtimer_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIPVTimerState *s = opaque;

    if (addr == SGI_PVTIMER_COUNTER_REG) {
        return pvtimer_ticks(s);
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                  __func__, addr);
    return 0;
}

static void sgi_pvtimer_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGIPVTimerState *s = opaque;

    if (addr == SGI_PVTIMER_COUNTER_REG) {
        /*
         * Set base so that the next read returns val.
         * PROM writes 0 to reset; subsequent reads return elapsed ticks.
         */
        uint64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t raw = (ns * 66) / 1000;
        s->base = raw - val;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }
}

static const MemoryRegionOps sgi_pvtimer_ops = {
    .read = sgi_pvtimer_read,
    .write = sgi_pvtimer_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void sgi_pvtimer_realize(DeviceState *dev, Error **errp)
{
    SGIPVTimerState *s = SGI_PVTIMER(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvtimer_ops, s,
                          "sgi-pvtimer", SGI_PVTIMER_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    s->base = 0;
}

static void sgi_pvtimer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_pvtimer_realize;
}

static const TypeInfo sgi_pvtimer_info = {
    .name = TYPE_SGI_PVTIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPVTimerState),
    .class_init = sgi_pvtimer_class_init,
};

static void sgi_pvtimer_register_types(void)
{
    type_register_static(&sgi_pvtimer_info);
}

type_init(sgi_pvtimer_register_types)
