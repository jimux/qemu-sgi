/*
 * SGI IP6 (Personal IRIS 4D/20-4D/25) serial input peers
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_IP6_INPUT_H
#define HW_MISC_SGI_IP6_INPUT_H

#include "chardev/char-fe.h"
#include "hw/core/qdev.h"
#include "ui/input.h"

#define TYPE_SGI_IP6_INPUT "sgi-ip6-input"
OBJECT_DECLARE_SIMPLE_TYPE(SgiIp6InputState, SGI_IP6_INPUT)

struct SgiIp6InputState {
    DeviceState parent_obj;

    /* Character backend shared with the DUART (the DUART is its frontend). */
    Chardev *chr;

    /* QEMU input handler that feeds us host events. */
    QemuInputHandlerState *ih;

    /* Mouse Systems report state. */
    int dx, dy;
    uint8_t buttons;        /* active-low bits 2:0 (1 = released) */
    bool dirty;

    /* Bytes waiting to go back to the guest (backend -> frontend). */
    uint8_t rx_buf[16];
    int rx_pos, rx_len;

    /*
     * This instance is the KEYBOARD HLE rather than the mouse.  Same peer
     * plumbing, different protocol: the guest writes command bytes and we
     * answer / stream key events, instead of emitting mouse reports.
     */
    bool keyboard;

    /* Number of guest command bytes seen (bring-up aid). */
    unsigned kb_cmds;
};

Chardev *sgi_ip6_input_chardev(SgiIp6InputState *s);

#endif /* HW_MISC_SGI_IP6_INPUT_H */
