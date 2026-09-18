/*
 * Philips SAA7185 video encoder I2C slave model.
 *
 * Write-only register file (ACK everything).  A bare read transfer (the
 * driver's probe path, no subaddress) returns a status byte; the value
 * is not interpreted by the probe, only the ACK matters.
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/display/saa7185.h"
#include "migration/vmstate.h"
#include "trace.h"

static void saa7185_reset(DeviceState *dev)
{
    SAA7185State *s = SAA7185(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->first = true;

    /* Post-reset defaults from chipregs_7185 (mvp7185.h, NTSC column). */
    s->regs[0x3a] = 0x1f;    /* IPC input port control               */
    s->regs[0x5a] = 0xaa;    /* chroma phase                         */
    s->regs[0x5b] = 0x81;    /* gain U                               */
    s->regs[0x5c] = 0xbd;    /* gain V                               */
    s->regs[0x5d] = 0x3c;    /* gain U / black level                 */
    s->regs[0x5e] = 0x3a;    /* gain V / blank level                 */
    s->regs[0x61] = 0x15;    /* standard control (NTSC)              */
    s->regs[0x62] = 0xee;    /* burst amplitude                      */
    s->regs[0x63] = 0x1f;    /* subcarrier 0                         */
    s->regs[0x64] = 0x7c;    /* subcarrier 1                         */
    s->regs[0x65] = 0xf0;    /* subcarrier 2                         */
    s->regs[0x66] = 0x21;    /* subcarrier 3                         */
    s->regs[0x6b] = 0x11;    /* encoder control / cc line            */
    s->regs[0x6c] = 0x6c;    /* receive port control                 */
    s->regs[0x6e] = 0x9a;    /* h trigger 0                          */
    s->regs[0x6f] = 0x60;    /* h trigger 1                          */
    s->regs[0x7a] = 0x0c;    /* field length                         */
    s->regs[0x7b] = 0x04;    /* first active line                    */
    s->regs[0x7c] = 0x03;    /* last active line                     */
    s->regs[0x7d] = 0x20;    /* msbs field control                   */
}

static int saa7185_event(I2CSlave *i2c, enum i2c_event event)
{
    SAA7185State *s = SAA7185(i2c);

    switch (event) {
    case I2C_START_SEND:
    case I2C_FINISH:
        s->first = true;
        break;
    default:
        break;
    }
    return 0;
}

static int saa7185_send(I2CSlave *i2c, uint8_t data)
{
    SAA7185State *s = SAA7185(i2c);

    if (s->first) {
        s->ptr = data;
        s->first = false;
        trace_saa7185_write(s->ptr, 0, false);
        return 0;
    }

    trace_saa7185_write(s->ptr, data, false);
    s->regs[s->ptr] = data;
    s->ptr++;
    return 0;
}

static uint8_t saa7185_recv(I2CSlave *i2c)
{
    SAA7185State *s = SAA7185(i2c);
    uint8_t val = s->regs[s->ptr];

    trace_saa7185_read(s->ptr, val);
    s->ptr++;
    return val;
}

static const VMStateDescription vmstate_saa7185 = {
    .name = TYPE_SAA7185,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, SAA7185State, 256),
        VMSTATE_UINT8(ptr, SAA7185State),
        VMSTATE_BOOL(first, SAA7185State),
        VMSTATE_END_OF_LIST()
    },
};

static void saa7185_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, saa7185_reset);
    dc->vmsd = &vmstate_saa7185;
    isc->event = saa7185_event;
    isc->send = saa7185_send;
    isc->recv = saa7185_recv;
}

static const TypeInfo saa7185_info = {
    .name = TYPE_SAA7185,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SAA7185State),
    .class_init = saa7185_class_init,
};

static void saa7185_register_types(void)
{
    type_register_static(&saa7185_info);
}

type_init(saa7185_register_types);
