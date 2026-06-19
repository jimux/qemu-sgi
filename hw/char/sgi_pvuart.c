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

/* RX ring helpers. One slot is kept empty to distinguish full from empty. */
static bool rxfifo_empty(SGIPVUARTState *s)
{
    return s->rx_head == s->rx_tail;
}

static bool rxfifo_full(SGIPVUARTState *s)
{
    return ((s->rx_head + 1) % SGI_PVUART_RXFIFO_SIZE) == s->rx_tail;
}

static void rxfifo_push(SGIPVUARTState *s, uint8_t ch)
{
    if (!rxfifo_full(s)) {
        s->rx_fifo[s->rx_head] = ch;
        s->rx_head = (s->rx_head + 1) % SGI_PVUART_RXFIFO_SIZE;
    }
}

static uint8_t rxfifo_pop(SGIPVUARTState *s)
{
    uint8_t ch = s->rx_fifo[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % SGI_PVUART_RXFIFO_SIZE;
    return ch;
}

static uint64_t sgi_pvuart_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIPVUARTState *s = opaque;

    switch (addr) {
    case SGI_PVUART_THR_RBR:
        if (!rxfifo_empty(s)) {
            uint8_t ch = rxfifo_pop(s);
            qemu_chr_fe_accept_input(&s->chr);
            return ch;
        }
        return 0;
    case SGI_PVUART_LSR:
        return SGI_PVUART_LSR_THRE |
               (rxfifo_empty(s) ? 0 : SGI_PVUART_LSR_DR);
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
    case SGI_PVUART_RXINJECT:
        /* Host RX-inject: a write here (e.g. via the gdbstub) pushes a byte
         * into the RX FIFO exactly as if it had arrived on the wire. The guest
         * console input path delivers it normally — no serial backend, no TFTP.
         */
        rxfifo_push(s, val & 0xFF);
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
    return rxfifo_full(s) ? 0 : 1;
}

static void sgi_pvuart_rx(void *opaque, const uint8_t *buf, int size)
{
    SGIPVUARTState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        rxfifo_push(s, buf[i]);
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
