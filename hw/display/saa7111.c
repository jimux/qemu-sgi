/*
 * Philips SAA7111 analog video decoder I2C slave model.
 *
 * Minimal register-file model: permissive enough for the guest `mvp`
 * driver / IP32 PROM probe + init sequence to complete.  The probe
 * reads the status register (0x1f); we always report horizontal lock so
 * the driver can select an analog input source.  Every write is ACKed.
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/display/saa7111.h"
#include "migration/vmstate.h"
#include "trace.h"

/* Registers the chip does not accept writes to (mvp7111.h RO entries). */
static bool saa7111_reg_ro(unsigned int reg)
{
    return reg == 0x00 || reg == 0x01 || reg == 0x1a ||
           reg == 0x1b || reg == 0x1c || reg == 0x1f;
}

static void saa7111_reset(DeviceState *dev)
{
    SAA7111State *s = SAA7111(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->first = true;

    /*
     * Post-reset defaults from the PROM chipregs_7111 table (mvp7111.h,
     * NTSC column).  Status reports horizontal lock and field id so the
     * driver's default-source detection selects the analog path.
     */
    s->regs[0x02] = 0xd8;    /* AIC1: composite input, clamped      */
    s->regs[0x03] = 0x23;    /* AIC2                                 */
    s->regs[0x04] = 0x98;    /* AIC3                                 */
    s->regs[0x05] = 0xd4;    /* AIC4                                 */
    s->regs[0x06] = 0xeb;    /* HSS                                  */
    s->regs[0x07] = 0xe0;    /* HSTP                                 */
    s->regs[0x08] = 0x88;    /* SC sync control                      */
    s->regs[0x09] = 0x01;    /* LCR luminance control                */
    s->regs[0x0a] = 0x84;    /* brightness                           */
    s->regs[0x0b] = 0x49;    /* contrast                             */
    s->regs[0x0c] = 0x38;    /* saturation                           */
    s->regs[0x0d] = 0xf8;    /* hue                                  */
    s->regs[0x0e] = 0x02;    /* CC chrominance control               */
    s->regs[0x10] = 0xc7;    /* format/delay                         */
    s->regs[0x11] = 0x0c;    /* output control 1                     */
    s->regs[0x12] = 0x81;    /* output control 2                     */
    s->regs[0x1f] = SAA7111_STAT_HLCK;   /* status: locked           */
}

static int saa7111_event(I2CSlave *i2c, enum i2c_event event)
{
    SAA7111State *s = SAA7111(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->first = true;
        break;
    case I2C_START_RECV:
        /* Subaddress was previously latched by a send phase. */
        break;
    case I2C_FINISH:
        s->first = true;
        break;
    default:
        break;
    }
    return 0;
}

static int saa7111_send(I2CSlave *i2c, uint8_t data)
{
    SAA7111State *s = SAA7111(i2c);

    if (s->first) {
        /* First byte after the address is the register pointer. */
        s->ptr = data;
        s->first = false;
        trace_saa7111_write(s->ptr, 0, false);
        return 0;
    }

    if (saa7111_reg_ro(s->ptr)) {
        trace_saa7111_write(s->ptr, data, true);
        s->ptr++;
        return 0;
    }

    trace_saa7111_write(s->ptr, data, false);
    s->regs[s->ptr] = data;
    s->ptr++;
    return 0;
}

static uint8_t saa7111_recv(I2CSlave *i2c)
{
    SAA7111State *s = SAA7111(i2c);
    uint8_t val = s->regs[s->ptr];

    /* Status read always reports a locked signal (HLCK). */
    if (s->ptr == 0x1f) {
        val |= SAA7111_STAT_HLCK;
    }
    trace_saa7111_read(s->ptr, val);
    s->ptr++;
    return val;
}

static const VMStateDescription vmstate_saa7111 = {
    .name = TYPE_SAA7111,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, SAA7111State, 256),
        VMSTATE_UINT8(ptr, SAA7111State),
        VMSTATE_BOOL(first, SAA7111State),
        VMSTATE_END_OF_LIST()
    },
};

static void saa7111_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, saa7111_reset);
    dc->vmsd = &vmstate_saa7111;
    isc->event = saa7111_event;
    isc->send = saa7111_send;
    isc->recv = saa7111_recv;
}

static const TypeInfo saa7111_info = {
    .name = TYPE_SAA7111,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SAA7111State),
    .class_init = saa7111_class_init,
};

static void saa7111_register_types(void)
{
    type_register_static(&saa7111_info);
}

type_init(saa7111_register_types);
