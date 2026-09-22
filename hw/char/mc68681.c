/*
 * Motorola MC68681 / SCN2681 dual asynchronous receiver/transmitter.
 *
 * QEMU has no MC68681 model; the SGI IP2 carries two of them (tty0-3). This
 * is a functional, polled-operation model: the mode/clock/aux registers are
 * latched, transmitter and receiver are live through the two chardevs, and
 * the status/interrupt registers behave well enough for period firmware that
 * polls SR or enables the receiver interrupt. Counter/timer and input-port
 * peripherals are latched but not timed.
 *
 * Copyright (c) 2026
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/char/mc68681.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define MC68681_RX_FIFO 4

/* Status register bits */
#define SR_RXRDY   0x01
#define SR_FFULL   0x02
#define SR_TXRDY   0x04
#define SR_TXEMP   0x08
#define SR_OVERRUN 0x10
#define SR_PE      0x20
#define SR_FE      0x40
#define SR_RB      0x80

/* Interrupt status/mask bits */
#define ISR_TXA    0x01
#define ISR_RXA    0x02
#define ISR_COUNTER 0x08
#define ISR_TXB    0x10
#define ISR_RXB    0x20
#define ISR_DCD    0x80

struct MC68681State;

typedef struct MC68681RxCtx {
    struct MC68681State *s;
    int ch;
} MC68681RxCtx;

struct MC68681State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;

    CharFrontend chr[2];
    MC68681RxCtx rx_opaque[2];

    uint8_t mr[2][2];
    uint8_t mr_ptr[2];
    uint8_t csr[2];
    uint8_t cr[2];
    uint8_t sr[2];
    uint8_t fifo[2][MC68681_RX_FIFO];
    int fifo_len[2];
    bool tx_enabled[2];
    bool rx_enabled[2];

    uint8_t acr;
    uint8_t opcr;
    uint8_t imr;
    uint8_t isr;
    uint8_t ipcr;
    uint8_t ctu;
    uint8_t ctl;
    uint8_t iport;
};

static void mc68681_update_irq(MC68681State *s)
{
    s->isr = 0;
    if (s->sr[0] & SR_TXRDY) {
        s->isr |= ISR_TXA;
    }
    if (s->sr[0] & SR_RXRDY) {
        s->isr |= ISR_RXA;
    }
    if (s->sr[1] & SR_TXRDY) {
        s->isr |= ISR_TXB;
    }
    if (s->sr[1] & SR_RXRDY) {
        s->isr |= ISR_RXB;
    }
    qemu_set_irq(s->irq, (s->isr & s->imr) != 0);
}

static void mc68681_rx_push(MC68681State *s, int ch, uint8_t data)
{
    if (s->fifo_len[ch] == MC68681_RX_FIFO) {
        s->sr[ch] |= SR_OVERRUN;
        return;
    }
    s->fifo[ch][s->fifo_len[ch]++] = data;
    s->sr[ch] |= SR_RXRDY;
    if (s->fifo_len[ch] == MC68681_RX_FIFO) {
        s->sr[ch] |= SR_FFULL;
    }
    mc68681_update_irq(s);
}

static uint8_t mc68681_rhr_read(MC68681State *s, int ch)
{
    uint8_t val;
    int i;

    if (s->fifo_len[ch] == 0) {
        return 0;
    }
    val = s->fifo[ch][0];
    s->fifo_len[ch]--;
    for (i = 0; i < s->fifo_len[ch]; i++) {
        s->fifo[ch][i] = s->fifo[ch][i + 1];
    }
    s->sr[ch] &= ~SR_FFULL;
    if (s->fifo_len[ch] == 0) {
        s->sr[ch] &= ~SR_RXRDY;
    }
    mc68681_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr[ch]);
    return val;
}

static void mc68681_do_command(MC68681State *s, int ch, uint8_t cmd)
{
    switch ((cmd >> 4) & 7) {
    case 1: /* reset mode register pointer */
        s->mr_ptr[ch] = 0;
        break;
    case 2: /* reset receiver */
        s->rx_enabled[ch] = false;
        s->fifo_len[ch] = 0;
        s->sr[ch] &= ~(SR_RXRDY | SR_FFULL);
        break;
    case 3: /* reset transmitter */
        s->tx_enabled[ch] = false;
        s->sr[ch] |= SR_TXEMP;
        s->sr[ch] &= ~SR_TXRDY;
        break;
    case 4: /* reset error status */
        s->sr[ch] &= ~(SR_OVERRUN | SR_PE | SR_FE | SR_RB);
        break;
    default:
        break;
    }

    switch ((cmd >> 2) & 3) {
    case 1: /* enable transmitter */
        s->tx_enabled[ch] = true;
        s->sr[ch] |= SR_TXRDY | SR_TXEMP;
        break;
    case 2: /* disable transmitter */
        s->tx_enabled[ch] = false;
        s->sr[ch] &= ~SR_TXRDY;
        break;
    default:
        break;
    }

    switch (cmd & 3) {
    case 1: /* enable receiver */
        s->rx_enabled[ch] = true;
        break;
    case 2: /* disable receiver */
        s->rx_enabled[ch] = false;
        break;
    default:
        break;
    }
    s->cr[ch] = cmd;
    mc68681_update_irq(s);
}

static uint64_t mc68681_read(void *opaque, hwaddr addr, unsigned size)
{
    MC68681State *s = opaque;
    int ch = (addr >> 3) & 1;
    int reg = addr & 7;

    switch (reg) {
    case 0:
        return s->mr[ch][0];
    case 1:
        return s->sr[ch];
    case 2:
        return 0; /* BRG test */
    case 3:
        return mc68681_rhr_read(s, ch);
    case 4:
        return s->ipcr;
    case 5:
        return ch == 0 ? s->isr : s->iport;
    case 6:
        return s->ctu;
    case 7:
        return s->ctl;
    default:
        return 0;
    }
}

static void mc68681_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MC68681State *s = opaque;
    int ch = (addr >> 3) & 1;
    int reg = addr & 7;
    uint8_t data = val;

    switch (reg) {
    case 0:
        s->mr[ch][s->mr_ptr[ch]] = data;
        s->mr_ptr[ch] ^= 1;
        break;
    case 1:
        s->csr[ch] = data;
        break;
    case 2:
        mc68681_do_command(s, ch, data);
        break;
    case 3:
        if (s->tx_enabled[ch]) {
            qemu_chr_fe_write_all(&s->chr[ch], &data, 1);
        }
        s->sr[ch] |= SR_TXRDY | SR_TXEMP;
        break;
    case 4:
        s->acr = data;
        break;
    case 5:
        if (ch == 0) {
            s->imr = data;
        } else {
            s->opcr = data;
        }
        break;
    case 6:
        s->ctu = data;
        break;
    case 7:
        s->ctl = data;
        break;
    default:
        break;
    }
    mc68681_update_irq(s);
}

static const MemoryRegionOps mc68681_ops = {
    .read = mc68681_read,
    .write = mc68681_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static int mc68681_can_receive(void *opaque)
{
    MC68681RxCtx *ctx = opaque;
    MC68681State *s = ctx->s;
    int ch = ctx->ch;

    return s->rx_enabled[ch] ? MC68681_RX_FIFO - s->fifo_len[ch] : 0;
}

static void mc68681_receive(void *opaque, const uint8_t *buf, int size)
{
    MC68681RxCtx *ctx = opaque;
    MC68681State *s = ctx->s;
    int ch = ctx->ch;
    int i;

    for (i = 0; i < size; i++) {
        mc68681_rx_push(s, ch, buf[i]);
    }
}

static void mc68681_reset(DeviceState *dev)
{
    MC68681State *s = MC68681(dev);
    int ch;

    for (ch = 0; ch < 2; ch++) {
        s->mr[ch][0] = s->mr[ch][1] = 0;
        s->mr_ptr[ch] = 0;
        s->csr[ch] = 0;
        s->cr[ch] = 0;
        s->sr[ch] = 0;
        s->fifo_len[ch] = 0;
        s->tx_enabled[ch] = false;
        s->rx_enabled[ch] = false;
    }
    s->acr = s->opcr = s->imr = s->isr = s->ipcr = 0;
    s->ctu = s->ctl = 0;
    s->iport = 0;
    mc68681_update_irq(s);
}

static void mc68681_realize(DeviceState *dev, Error **errp)
{
    MC68681State *s = MC68681(dev);
    int ch;

    for (ch = 0; ch < 2; ch++) {
        s->rx_opaque[ch].s = s;
        s->rx_opaque[ch].ch = ch;
        qemu_chr_fe_set_handlers(&s->chr[ch], mc68681_can_receive,
                                 mc68681_receive, NULL, NULL,
                                 &s->rx_opaque[ch], NULL, true);
    }
}

static void mc68681_init(Object *obj)
{
    MC68681State *s = MC68681(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &mc68681_ops, s, "mc68681", 16);
    sysbus_init_mmio(dev, &s->mmio);
    sysbus_init_irq(dev, &s->irq);
}

MemoryRegion *mc68681_mmio_region(DeviceState *dev)
{
    return sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
}

static const Property mc68681_properties[] = {
    DEFINE_PROP_CHR("chardev-a", MC68681State, chr[0]),
    DEFINE_PROP_CHR("chardev-b", MC68681State, chr[1]),
};

static void mc68681_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mc68681_realize;
    device_class_set_legacy_reset(dc, mc68681_reset);
    device_class_set_props(dc, mc68681_properties);
}

static const TypeInfo mc68681_info = {
    .name          = TYPE_MC68681,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MC68681State),
    .instance_init = mc68681_init,
    .class_init    = mc68681_class_init,
};

static void mc68681_register_types(void)
{
    type_register_static(&mc68681_info);
}

type_init(mc68681_register_types)
