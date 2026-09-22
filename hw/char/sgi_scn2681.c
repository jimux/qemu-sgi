/*
 * SCN2681 dual UART (see sgi_scn2681.h).  Register model follows MAME's
 * mc68681; transmission is immediate (no baud timing) so polled console
 * output works.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/char/sgi_scn2681.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#define SCN2681_REG_MR1A   0x0
#define SCN2681_REG_SRA    0x1
#define SCN2681_REG_CSRA   0x1
#define SCN2681_REG_CRA    0x2
#define SCN2681_REG_THRA   0x3
#define SCN2681_REG_RHRA   0x3
#define SCN2681_REG_IPCR   0x4
#define SCN2681_REG_ACR    0x4
#define SCN2681_REG_ISR    0x5
#define SCN2681_REG_IMR    0x5
#define SCN2681_REG_CTUR   0x6
#define SCN2681_REG_CTLR   0x7
#define SCN2681_REG_MR1B   0x8
#define SCN2681_REG_SRB    0x9
#define SCN2681_REG_CSRB   0x9
#define SCN2681_REG_CRB    0xa
#define SCN2681_REG_THRB   0xb
#define SCN2681_REG_RHRB   0xb
#define SCN2681_REG_IVR    0xc
#define SCN2681_REG_OPCR   0xd
#define SCN2681_REG_IP     0xd
#define SCN2681_REG_OP_SET 0xe
#define SCN2681_REG_OP_RST 0xf
#define SCN2681_REG_START  0xe

static void scn2681_update_irq(SCN2681State *s)
{
    qemu_set_irq(s->irq, (s->isr & s->imr) != 0);
}

static void scn2681_tx(SCN2681State *s, int chn, uint8_t val)
{
    CharFrontend *chr = chn ? &s->chr_b : &s->chr_a;
    /* MR2 bits 7:6 = 10 selects local loopback: the transmitter is wired
     * back to the receiver. The PROM's power-on UART self-test uses it. */
    bool loopback = (s->ch[chn].mr2 & 0xc0) == 0x80;

    if (loopback) {
        s->ch[chn].rx_byte = val;
        s->ch[chn].sr |= SCN2681_SR_RXRDY;
        s->isr |= chn ? SCN2681_ISR_RXRDYB : SCN2681_ISR_RXRDYA;
    } else if (qemu_chr_fe_backend_connected(chr)) {
        qemu_chr_fe_write_all(chr, &val, 1);
    }
    /* Transmitter immediately ready again. */
    s->ch[chn].sr |= SCN2681_SR_TXRDY | SCN2681_SR_TXEMT;
    s->isr |= chn ? SCN2681_ISR_TXRDYB : SCN2681_ISR_TXRDYA;
    scn2681_update_irq(s);
}

static void scn2681_cmd(SCN2681State *s, int chn, uint8_t data)
{
    switch ((data >> 4) & 0x7) {
    case 1: /* reset MR pointer */
        s->ch[chn].mr_ptr = false;
        break;
    case 2: /* reset receiver */
        s->ch[chn].sr &= ~(SCN2681_SR_RXRDY | SCN2681_SR_OVRUN |
                           SCN2681_SR_PERR | SCN2681_SR_FERR |
                           SCN2681_SR_BREAK);
        s->isr &= ~(chn ? SCN2681_ISR_RXRDYB : SCN2681_ISR_RXRDYA);
        scn2681_update_irq(s);
        break;
    case 3: /* reset transmitter */
        s->ch[chn].sr &= ~(SCN2681_SR_TXRDY | SCN2681_SR_TXEMT);
        s->isr &= ~(chn ? SCN2681_ISR_TXRDYB : SCN2681_ISR_TXRDYA);
        s->ch[chn].sr |= SCN2681_SR_TXRDY | SCN2681_SR_TXEMT;
        scn2681_update_irq(s);
        break;
    case 4: /* reset error status */
        s->ch[chn].sr &= ~(SCN2681_SR_OVRUN | SCN2681_SR_PERR |
                           SCN2681_SR_FERR | SCN2681_SR_BREAK);
        break;
    default:
        break;
    }
}

uint8_t scn2681_read(SCN2681State *s, int reg)
{
    uint8_t ret = 0;

    switch (reg & 0xf) {
    case SCN2681_REG_MR1A:
        if (!s->ch[0].mr_ptr) {
            s->ch[0].mr_ptr = true;
            ret = s->ch[0].mr1;
        } else {
            ret = s->ch[0].mr2;
        }
        break;
    case SCN2681_REG_SRA:
        ret = s->ch[0].sr;
        break;
    case SCN2681_REG_THRA: /* RHR A */
        ret = s->ch[0].rx_byte;
        s->ch[0].sr &= ~SCN2681_SR_RXRDY;
        s->isr &= ~SCN2681_ISR_RXRDYA;
        scn2681_update_irq(s);
        break;
    case SCN2681_REG_IPCR:
        ret = 0x0f; /* no input port change pending */
        break;
    case SCN2681_REG_ISR:
        ret = s->isr;
        break;
    case SCN2681_REG_CTUR:
        ret = s->ctur;
        break;
    case SCN2681_REG_CTLR:
        ret = s->ctlr;
        break;
    case SCN2681_REG_MR1B:
        if (!s->ch[1].mr_ptr) {
            s->ch[1].mr_ptr = true;
            ret = s->ch[1].mr1;
        } else {
            ret = s->ch[1].mr2;
        }
        break;
    case SCN2681_REG_SRB:
        ret = s->ch[1].sr;
        break;
    case SCN2681_REG_THRB: /* RHR B */
        ret = s->ch[1].rx_byte;
        s->ch[1].sr &= ~SCN2681_SR_RXRDY;
        s->isr &= ~SCN2681_ISR_RXRDYB;
        scn2681_update_irq(s);
        break;
    case SCN2681_REG_IVR:
        ret = s->ivr;
        break;
    case SCN2681_REG_IP:
        ret = 0xff; /* input port idle (pulled high) */
        break;
    case SCN2681_REG_OP_SET:
    case SCN2681_REG_OP_RST:
        ret = 0;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi-scn2681: read of reserved register 0x%x\n", reg);
        break;
    }

    return ret;
}

void scn2681_write(SCN2681State *s, int reg, uint8_t val)
{
    switch (reg & 0xf) {
    case SCN2681_REG_MR1A:
        if (!s->ch[0].mr_ptr) {
            s->ch[0].mr1 = val;
            s->ch[0].mr_ptr = true;
        } else {
            s->ch[0].mr2 = val;
        }
        break;
    case SCN2681_REG_CSRA:
        s->ch[0].csr = val;
        break;
    case SCN2681_REG_CRA:
        scn2681_cmd(s, 0, val);
        break;
    case SCN2681_REG_THRA:
        scn2681_tx(s, 0, val);
        break;
    case SCN2681_REG_ACR:
        s->acr = val;
        break;
    case SCN2681_REG_IMR:
        s->imr = val;
        scn2681_update_irq(s);
        break;
    case SCN2681_REG_CTUR:
        s->ctur = val;
        break;
    case SCN2681_REG_CTLR:
        s->ctlr = val;
        break;
    case SCN2681_REG_MR1B:
        if (!s->ch[1].mr_ptr) {
            s->ch[1].mr1 = val;
            s->ch[1].mr_ptr = true;
        } else {
            s->ch[1].mr2 = val;
        }
        break;
    case SCN2681_REG_CSRB:
        s->ch[1].csr = val;
        break;
    case SCN2681_REG_CRB:
        scn2681_cmd(s, 1, val);
        break;
    case SCN2681_REG_THRB:
        scn2681_tx(s, 1, val);
        break;
    case SCN2681_REG_IVR:
        s->ivr = val;
        break;
    case SCN2681_REG_OPCR:
        s->opcr = val;
        break;
    case SCN2681_REG_OP_SET:
        s->opr |= val;
        break;
    case SCN2681_REG_OP_RST:
        s->opr &= ~val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi-scn2681: write 0x%02x to reserved register 0x%x\n",
                      val, reg);
        break;
    }
}

/* ---- chardev receive -------------------------------------------------- */

static int scn2681_can_receive(void *opaque)
{
    SCN2681State *s = opaque;

    return !(s->ch[0].sr & SCN2681_SR_RXRDY);
}

static void scn2681_receive(void *opaque, const uint8_t *buf, int size)
{
    SCN2681State *s = opaque;

    s->ch[0].rx_byte = buf[0];
    s->ch[0].sr |= SCN2681_SR_RXRDY;
    s->isr |= SCN2681_ISR_RXRDYA;
    scn2681_update_irq(s);
}

static int scn2681_can_receive_b(void *opaque)
{
    SCN2681State *s = opaque;

    return !(s->ch[1].sr & SCN2681_SR_RXRDY);
}

static void scn2681_receive_b(void *opaque, const uint8_t *buf, int size)
{
    SCN2681State *s = opaque;

    s->ch[1].rx_byte = buf[0];
    s->ch[1].sr |= SCN2681_SR_RXRDY;
    s->isr |= SCN2681_ISR_RXRDYB;
    scn2681_update_irq(s);
}

/* ---- QOM -------------------------------------------------------------- */

static void scn2681_reset(DeviceState *dev)
{
    SCN2681State *s = SGI_SCN2681(dev);
    int i;

    for (i = 0; i < 2; i++) {
        s->ch[i].mr1 = 0;
        s->ch[i].mr2 = 0;
        s->ch[i].csr = 0;
        s->ch[i].sr = SCN2681_SR_TXRDY | SCN2681_SR_TXEMT;
        s->ch[i].mr_ptr = false;
        s->ch[i].rx_byte = 0;
    }
    s->acr = 0;
    s->imr = 0;
    s->isr = 0;
    s->ctur = 0;
    s->ctlr = 0;
}

static void scn2681_realize(DeviceState *dev, Error **errp)
{
    SCN2681State *s = SGI_SCN2681(dev);

    qdev_init_gpio_out_named(dev, &s->irq, "irq", 1);

    if (qemu_chr_fe_backend_connected(&s->chr_a)) {
        qemu_chr_fe_set_handlers(&s->chr_a, scn2681_can_receive,
                                 scn2681_receive, NULL, NULL, s, NULL, true);
    }
    if (qemu_chr_fe_backend_connected(&s->chr_b)) {
        qemu_chr_fe_set_handlers(&s->chr_b, scn2681_can_receive_b,
                                 scn2681_receive_b, NULL, NULL, s, NULL, true);
    }
}

static const Property scn2681_properties[] = {
    DEFINE_PROP_CHR("chardev-a", SCN2681State, chr_a),
    DEFINE_PROP_CHR("chardev-b", SCN2681State, chr_b),
};

static void scn2681_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = scn2681_realize;
    device_class_set_legacy_reset(dc, scn2681_reset);
    device_class_set_props(dc, scn2681_properties);
}

static const TypeInfo scn2681_info = {
    .name          = TYPE_SGI_SCN2681,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(SCN2681State),
    .class_init    = scn2681_class_init,
};

static void scn2681_register_types(void)
{
    type_register_static(&scn2681_info);
}

type_init(scn2681_register_types)
