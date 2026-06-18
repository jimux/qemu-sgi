/*
 * SGI Paravirtual UART
 *
 * Minimal polled UART for the IP54 paravirtual machine.
 * The PROM accesses two byte registers via kseg1:
 *   0xBF62017B (+3): THR on write (send char), RBR on read (receive char)
 *   0xBF62017E (+6): LSR on read  (bit 0 = DR, bit 5 = THRE)
 *
 * No IRQ output — the PROM uses polled I/O only.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "chardev/char-fe.h"
#include "hw/char/sgi_pvuart.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qom/object.h"

static int pvuart_debug_countdown = 200; /* first N reads traced */

static uint64_t sgi_pvuart_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIPVUARTState *s = opaque;

    switch (addr) {
    case SGI_PVUART_THR_RBR:
        if (s->rx_ready) {
            uint8_t ch = s->rx_byte;
            s->rx_ready = false;
            qemu_chr_fe_accept_input(&s->chr);
            fprintf(stderr, "[PVUART] RBR read: 0x%02x '%c'\n",
                    ch, (ch >= 0x20 && ch < 0x7f) ? ch : '.');
            return ch;
        }
        return 0;
    case SGI_PVUART_LSR:
        {
            uint8_t lsr = SGI_PVUART_LSR_THRE; /* TX always ready */
            if (s->rx_ready) {
                lsr |= SGI_PVUART_LSR_DR;
                fprintf(stderr, "[PVUART] LSR read: 0x%02x (DR set)\n", lsr);
            } else if (pvuart_debug_countdown > 0) {
                pvuart_debug_countdown--;
            }
            return lsr;
        }
    default:
        return 0;
    }
}

static void sgi_pvuart_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SGIPVUARTState *s = opaque;

    switch (addr) {
    case SGI_PVUART_THR_RBR:
        {
            uint8_t ch = val & 0xFF;
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps sgi_pvuart_ops = {
    .read = sgi_pvuart_read,
    .write = sgi_pvuart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static int sgi_pvuart_can_rx(void *opaque)
{
    SGIPVUARTState *s = opaque;
    return !s->rx_ready;
}

static void sgi_pvuart_rx(void *opaque, const uint8_t *buf, int size)
{
    SGIPVUARTState *s = opaque;

    fprintf(stderr, "[PVUART] RX: size=%d byte=0x%02x '%c' rx_ready=%d\n",
            size, size > 0 ? buf[0] : 0,
            (size > 0 && buf[0] >= 0x20 && buf[0] < 0x7f) ? buf[0] : '.',
            s->rx_ready);
    if (size > 0 && !s->rx_ready) {
        s->rx_byte = buf[0];
        s->rx_ready = true;
    }
}

static void sgi_pvuart_event(void *opaque, QEMUChrEvent event)
{
    /* No-op: no modem lines or break handling needed */
}

static void sgi_pvuart_realize(DeviceState *dev, Error **errp)
{
    SGIPVUARTState *s = SGI_PVUART(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvuart_ops, s,
                          "sgi-pvuart", SGI_PVUART_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, sgi_pvuart_can_rx, sgi_pvuart_rx,
                                 sgi_pvuart_event, NULL, s, NULL, true);
    }
}

static const Property sgi_pvuart_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIPVUARTState, chr),
};

static void sgi_pvuart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_pvuart_realize;
    device_class_set_props(dc, sgi_pvuart_properties);
}

static const TypeInfo sgi_pvuart_info = {
    .name = TYPE_SGI_PVUART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPVUARTState),
    .class_init = sgi_pvuart_class_init,
};

static void sgi_pvuart_register_types(void)
{
    type_register_static(&sgi_pvuart_info);
}

type_init(sgi_pvuart_register_types)
