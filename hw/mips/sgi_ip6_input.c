/*
 * SGI IP6 (Personal IRIS 4D/20-4D/25) serial mouse HLE
 *
 * The IP6 has no PS/2 controller: the mouse hangs off channel B of the
 * first SCN2681 DUART as an asynchronous serial peer (the keyboard is
 * channel A).  The guest programs the DUART, then the mouse speaks the
 * Mouse Systems protocol back at it:
 *
 *   5-byte reports, sent only when "dirty" (movement or a button edge):
 *     byte 0  = 0x80 | buttons, buttons ACTIVE LOW  (bit2 = left,
 *               bit1 = middle, bit0 = right; 0x87 = all released)
 *     byte 1  = x1, byte 2 = y1, byte 3 = x2, byte 4 = y2
 *   x1+x2 is the X delta, y1+y2 the Y delta; each byte is signed and
 *   constrained to -120..127, so a large move is split across the pair.
 *   Y is positive UPWARDS (the host's is downwards, so we negate).
 *   No identification string, 8N1 at 4800 baud.
 *
 * [CROSS-REF] MAME src/mame/sgi/ip6.cpp (m_duart[0]->b_tx_cb() ->
 * mouse_port.write_txd(), mouse_port.rxd_handler() -> scn2681 rx_b_w)
 * and src/devices/bus/rs232/hlemouse.cpp (the SGI mouse is the
 * Mouse Systems subclass overriding the rate to 4800).
 *
 * The peer is a character-backend (ChardevClass) so the DUART talks to
 * it exactly as it would to any other chardev, and a small DeviceState
 * exists only to own the QEMU input handler and give the backend its
 * byte plumbing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "chardev/char.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_ip6_input.h"
#include "ui/input.h"
#include "qapi/error.h"
#include "trace.h"

#define SGI_MOUSE_RX_QUEUE  16

typedef struct SgiIp6PeerChardev {
    Chardev parent;
    SgiIp6InputState *dev;
} SgiIp6PeerChardev;

#define TYPE_CHARDEV_SGI_IP6_PEER "chardev-sgi-ip6-peer"
OBJECT_DECLARE_SIMPLE_TYPE(SgiIp6PeerChardev, CHARDEV_SGI_IP6_PEER)

/* ------------------------------------------------------------------ */
/* Device -> DUART byte path (backend -> frontend)                    */
/* ------------------------------------------------------------------ */

static void sgi_ip6_input_drain(SgiIp6InputState *s)
{
    while (s->rx_len > 0 && qemu_chr_be_can_write(s->chr) > 0) {
        qemu_chr_be_write(s->chr, &s->rx_buf[s->rx_pos], 1);
        s->rx_pos++;
        s->rx_len--;
        trace_sgi_ip6_input_drain(s->rx_len);
    }
    if (s->rx_len == 0) {
        s->rx_pos = 0;
    }
}

static void sgi_ip6_input_queue(SgiIp6InputState *s, const uint8_t *buf,
                                int len)
{
    while (len-- > 0) {
        if (s->rx_len == sizeof(s->rx_buf)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sgi-ip6-input: peer rx overrun, dropping byte\n");
            return;
        }
        s->rx_buf[(s->rx_pos + s->rx_len) % sizeof(s->rx_buf)] = *buf++;
        s->rx_len++;
    }
    sgi_ip6_input_drain(s);
}

/* The DUART tells us it has room again after the guest drains a byte. */
static void sgi_ip6_peer_accept_input(Chardev *chr)
{
    SgiIp6PeerChardev *peer = CHARDEV_SGI_IP6_PEER(chr);

    if (peer->dev) {
        sgi_ip6_input_drain(peer->dev);
    }
}

static int sgi_ip6_peer_write(Chardev *chr, const uint8_t *buf, int len)
{
    SgiIp6PeerChardev *peer = CHARDEV_SGI_IP6_PEER(chr);

    /*
     * The Mouse Systems mouse takes no commands; a real one would simply
     * ignore anything the host sends, so do the same (but leave a trace
     * for bring-up).
     */
    if (peer->dev && len > 0) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi-ip6-input: mouse peer ignoring %d host byte(s)\n",
                      len);
    }
    return len;
}

static void sgi_ip6_peer_class_init(ObjectClass *oc, const void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);

    cc->chr_write = sgi_ip6_peer_write;
    cc->chr_accept_input = sgi_ip6_peer_accept_input;
}

static const TypeInfo sgi_ip6_peer_type_info = {
    .name = TYPE_CHARDEV_SGI_IP6_PEER,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(SgiIp6PeerChardev),
    .class_init = sgi_ip6_peer_class_init,
};

/* ------------------------------------------------------------------ */
/* Mouse report generation                                            */
/* ------------------------------------------------------------------ */

static void sgi_ip6_mouse_report(SgiIp6InputState *s)
{
    int x1, y1;
    uint8_t buf[5];

    x1 = CLAMP(s->dx, -120, 127);
    y1 = CLAMP(s->dy, -120, 127);

    buf[0] = 0x80 | (s->buttons & 0x07);
    buf[1] = x1 & 0xff;
    buf[2] = y1 & 0xff;
    buf[3] = (s->dx - x1) & 0xff;
    buf[4] = (s->dy - y1) & 0xff;

    s->dx = 0;
    s->dy = 0;
    s->dirty = false;

    trace_sgi_ip6_input_report(buf[0], buf[1], buf[2], buf[3], buf[4]);
    sgi_ip6_input_queue(s, buf, sizeof(buf));
}

static void sgi_ip6_mouse_button(SgiIp6InputState *s, InputBtnEvent *btn)
{
    uint8_t mask;

    switch (btn->button) {
    case INPUT_BUTTON_LEFT:
        mask = 0x04;
        break;
    case INPUT_BUTTON_MIDDLE:
        mask = 0x02;
        break;
    case INPUT_BUTTON_RIGHT:
        mask = 0x01;
        break;
    default:
        return;
    }

    /* Active low: a pressed button clears its bit. */
    if (btn->down) {
        s->buttons &= ~mask;
    } else {
        s->buttons |= mask;
    }
    s->dirty = true;
}

static void sgi_ip6_mouse_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt)
{
    SgiIp6InputState *s = SGI_IP6_INPUT(dev);
    InputMoveEvent *move;

    switch (evt->type) {
    case INPUT_EVENT_KIND_REL:
        move = evt->u.rel.data;
        if (move->axis == INPUT_AXIS_X) {
            s->dx += move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->dy -= move->value;   /* SGI Y is up-positive */
        } else {
            return;                 /* no wheel in this protocol */
        }
        s->dirty = true;
        break;
    case INPUT_EVENT_KIND_BTN:
        sgi_ip6_mouse_button(s, evt->u.btn.data);
        break;
    default:
        return;
    }

    /* Reports are dirty-only: emit as soon as there is something to say. */
    if (s->dirty) {
        sgi_ip6_mouse_report(s);
    }
}

static const QemuInputHandler sgi_ip6_mouse_handler = {
    .name  = "SGI IP6 serial mouse",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_REL,
    .event = sgi_ip6_mouse_event,
};

/* ------------------------------------------------------------------ */
/* QOM                                                                */
/* ------------------------------------------------------------------ */

Chardev *sgi_ip6_input_chardev(SgiIp6InputState *s)
{
    return s->chr;
}

static void sgi_ip6_input_realize(DeviceState *dev, Error **errp)
{
    SgiIp6InputState *s = SGI_IP6_INPUT(dev);
    SgiIp6PeerChardev *peer;
    Error *local_err = NULL;

    s->buttons = 0x07;      /* all released, active low */
    s->dx = s->dy = 0;
    s->dirty = false;

    s->chr = qemu_chardev_new(NULL, TYPE_CHARDEV_SGI_IP6_PEER, NULL, NULL,
                              &local_err);
    if (!s->chr) {
        error_propagate(errp, local_err);
        return;
    }
    peer = CHARDEV_SGI_IP6_PEER(s->chr);
    peer->dev = s;

    s->ih = qemu_input_handler_register(dev, &sgi_ip6_mouse_handler);
    qemu_input_handler_activate(s->ih);
}

static void sgi_ip6_input_reset(DeviceState *dev)
{
    SgiIp6InputState *s = SGI_IP6_INPUT(dev);

    s->buttons = 0x07;
    s->dx = s->dy = 0;
    s->dirty = false;
    s->rx_len = s->rx_pos = 0;
}

static void sgi_ip6_input_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sgi_ip6_input_realize;
    device_class_set_legacy_reset(dc, sgi_ip6_input_reset);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
    dc->desc = "SGI IP6 serial mouse HLE";
}

static const TypeInfo sgi_ip6_input_type_info = {
    .name = TYPE_SGI_IP6_INPUT,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SgiIp6InputState),
    .class_init = sgi_ip6_input_class_init,
};

static void sgi_ip6_input_register_types(void)
{
    type_register_static(&sgi_ip6_peer_type_info);
    type_register_static(&sgi_ip6_input_type_info);
}

type_init(sgi_ip6_input_register_types);
