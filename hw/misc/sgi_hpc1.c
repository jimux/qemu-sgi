/*
 * SGI HPC1.5 (High Performance Peripheral Controller) for IP20 (Indigo)
 *
 * Models the HPC1.5 register window at 0x1fb80000: the core DMA/status
 * registers, the SEEQ 8003 ethernet register file, the WD33C93 SCSI
 * indirect interface, the INT2 interrupt multiplexor with its 8254 PIT,
 * three Z85C30 DUARTs (DUART2 channel A is the system console) and the
 * 93C56 serial EEPROM / NVRAM accessed through the aux bit-bang register.
 *
 * References:
 *   - MAME src/mame/sgi/ip20.cpp, hpc1.cpp, int2.cpp
 *   - IP20 PROM source: stand/arcs/IP20prom, stand/arcs/lib/libsk/ml
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
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qapi/error.h"
#include "ui/input.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"

#define HPC1_MMIO_SIZE 0x10000

/* INT2 register indices (32-bit registers, index = byte offset >> 2) */
#define INT2_LIO0_STATUS  0
#define INT2_LIO0_MASK    1
#define INT2_LIO1_STATUS   2
#define INT2_LIO1_MASK     3
#define INT2_VME_STATUS    4
#define INT2_VME0_MASK     5
#define INT2_VME1_MASK     6
#define INT2_CONFIG        7
#define INT2_TIMER_ACK     8
#define INT2_PIT_BASE      0xc

/* INT2 LIO0 input bit assignments (MAME int2.h) */
#define LIO0_SCSI      0x04
#define LIO0_ETHERNET  0x08
#define LIO0_DUART     0x20

/*
 * PIT clocking.
 *
 * The 8254 sits in the INT2. Counter 2 is clocked at 10 MHz; counters 0 and 1
 * are clocked by counter 2's output (a cascade), so IRIX's counter-2 divider
 * controls the slow scheduler tick. Counter 2 itself MUST stay at 10 MHz: the
 * PROM's power-on calibration programs it and reads it back after a fixed
 * delay, and clocking it at INT2_clock/10 = 1 MHz (as MAME's int2.cpp does) is
 * slow enough that the counter appears stalled and the PROM trips a
 * divide-by-zero assert at 0xbfc0aaac (break 0x7). Keeping counter 2 fast and
 * only dividing counters 0/1 boots the PROM and still lets IRIX reach the
 * scheduler idle loop.
 */
#define PIT_BASE_CLK_HZ  10000000

/* ------------------------------------------------------------------ */
/* Z85C30 DUART                                                        */
/* ------------------------------------------------------------------ */

#define SCC_RX_IP      0x04  /* channel B / generic RX char available */
#define SCC_RX_IP_A    0x20
#define SCC_TX_IP      0x02
#define SCC_TX_IP_A    0x10

/* IP20 keyboard HLE (DUART0 channel A, i.e. gfx(0)/KEYBD_PORT). The PROM's
 * libsk/io/sgi_kbd.c config_keyboard() writes CONFIG_REQUEST and expects the
 * standard keyboard's 2-byte reply: 0x6e (CONFIG_BYTE_NEWKB) then a layout id
 * <= 0x0f. The type byte indexes sgi_kbd.c's kbd_types[] directly, where
 * index 0 is "USA" and index 1 is "DEU" (German); so US MUST be 0. */
#define HPC1_KBD_CONFIG_REQUEST 0x10
#define HPC1_KBD_LAYOUT_US      0

static int scc_console_channel(const SGIHPC1State *s, int d, int c)
{
    /* DUART2 channel A is the console; DUART0 channel A carries the
     * PROM's early low-level output. */
    return (d == 2 && c == 0) || (d == 0 && c == 0);
}

static void scc_update_irq(SGIHPC1State *s);
static void sgi_hpc1_enet_tx(SGIHPC1State *s);
static void sgi_hpc1_enet_reset(SGIHPC1State *s);
static void sgi_hpc1_kbd_push(SGIHPC1State *s, uint8_t b);

static void scc_tx(SGIHPC1State *s, int d, int c, uint8_t data)
{
    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_write_all(&s->serial, &data, 1);
    }
    (void)scc_console_channel(s, d, c);

    /*
     * IP20 keyboard HLE: the PROM's config_keyboard() writes CONFIG_REQUEST
     * to the keyboard port (DUART0 channel A) and expects 0x6e + a layout id.
     */
    if (d == 0 && c == 0 && data == HPC1_KBD_CONFIG_REQUEST) {
        sgi_hpc1_kbd_push(s, 0x6e);
        sgi_hpc1_kbd_push(s, HPC1_KBD_LAYOUT_US);
    }

    /*
     * Writing the transmit buffer clears TX IP; once the byte has shifted out
     * the buffer is empty again and the real SCC re-asserts TX IP (if TX
     * interrupts are enabled). Our transmit is instantaneous, so re-assert
     * here — otherwise a console driver that relies on the TX interrupt only
     * ever gets the first one and stalls waiting for the next (observed as the
     * miniroot printing "C\rM" and then going silent).
     */
    {
        SGIHPC1Uart *u = &s->uart[d][c];
        uint8_t bit = (c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
        u->rr3 &= ~bit;
        if (u->wr[1] & 0x02) {   /* WR1 bit 1: TX interrupt enable */
            u->rr3 |= bit;
        }
    }
    scc_update_irq(s);
}

static void scc_ctrl_write(SGIHPC1State *s, int d, int c, uint8_t val)
{
    SGIHPC1Uart *u = &s->uart[d][c];

    if (u->reg_ptr == 0) {
        uint8_t cmd = (val >> 3) & 0x07;
        uint8_t ptr = val & 0x07;

        switch (cmd) {
        case 1: /* Point High */
            ptr |= 0x08;
            break;
        case 2: /* Reset External/Status interrupts */
            u->rr3 &= ~(c == 0 ? 0x08 : 0x01);
            scc_update_irq(s);
            break;
        case 5: /* Reset TX interrupt pending */
            u->rr3 &= ~(c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
            scc_update_irq(s);
            break;
        default:
            break;
        }
        u->reg_ptr = ptr;
        return;
    }

    switch (u->reg_ptr) {
    case 1: /* WR1: interrupt enables */
        u->wr[1] = val;
        if (val & 0x02) {
            /* Enabling TX interrupts with an empty buffer raises TX IP. */
            u->rr3 |= (c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
        } else {
            /* Disabling them withdraws it (otherwise it stays pending). */
            u->rr3 &= ~(c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
        }
        scc_update_irq(s);
        break;
    case 5: /* WR5: TX parameters */
        u->wr[5] = val;
        break;
    case 7:
        /*
         * WR7. On the 85C30, when WR15 bit 0 (Extended Read enable) is set,
         * WR7 is WR7' and RR14/RR9/RR11 can read it back instead of their
         * normal images. The IP20 power-on keyboard/mouse diagnostic uses
         * exactly this: it sets WR15=1, writes WR7'=0x40, then reads RR14 and
         * expects 0x40 -- so without WR7' the DUART looks absent and the
         * diagnostic fails.
         */
        u->wr[7] = val;
        if (u->wr[15] & 0x01) {
            u->wr7p = val;
        }
        break;
    case 8: /* WR8: transmit buffer */
        scc_tx(s, d, c, val);
        break;
    case 9: /* WR9: master interrupt control (shared across the DUART) */
        s->uart[d][0].wr[9] = val;
        s->uart[d][1].wr[9] = val;
        if (val & 0xc0) {
            if (val & 0x80) {
                s->uart[d][0].rr3 = 0;
                s->uart[d][0].wr[1] = 0;
            }
            if (val & 0x40) {
                s->uart[d][1].rr3 = 0;
                s->uart[d][1].wr[1] = 0;
            }
        }
        scc_update_irq(s);
        break;
    default:
        u->wr[u->reg_ptr] = val;
        break;
    }
    u->reg_ptr = 0;
}

static uint8_t scc_ctrl_read(SGIHPC1State *s, int d, int c)
{
    SGIHPC1Uart *u = &s->uart[d][c];
    uint8_t val;

    switch (u->reg_ptr) {
    case 1: /* RR1 */
        val = 0x01; /* all sent */
        break;
    case 2: /* RR2 interrupt vector */
        val = 0;
        break;
    case 3:
        /*
         * RR3 reports interrupt-pending bits, and only channel A's copy is
         * readable; it carries both channels' IP bits (0x20/0x10 = RX/TX A,
         * 0x04/0x02 = RX/TX B). Channel B's own RR3 reads as 0. Returning
         * only channel A's latched bits hid a pending channel-B interrupt
         * from the guest ISR, which then could never identify or clear it.
         */
        val = (c == 0) ? (u->rr3 | s->uart[d][1].rr3) : 0;
        break;
    case 12: /* RR12 = WR12 (BRG time constant, low) */
        val = u->wr[12];
        break;
    case 13: /* RR13 = WR13 (BRG time constant, high) */
        val = u->wr[13];
        break;
    case 14:
        /* 85C30: RR14 reflects WR7' when WR7' bit6 is set, else RR10 (0). */
        val = (u->wr7p & 0x40) ? u->wr7p : 0x00;
        break;
    case 15: /* RR15 = WR15 (external/status IE bits), unused bits 0 */
        val = u->wr[15] & 0xfa;
        break;
    default: /* RR0 status */
        val = 0x2c; /* TX empty, DCD, CTS */
        if (u->rx_count > 0) {
            val |= 0x01;
        }
        break;
    }
    u->reg_ptr = 0;
    return val;
}

static uint8_t scc_data_read(SGIHPC1State *s, int d, int c)
{
    SGIHPC1Uart *u = &s->uart[d][c];
    uint8_t val = 0;

    if (u->rx_count > 0) {
        val = u->rx_fifo[u->rx_tail];
        u->rx_tail = (u->rx_tail + 1) % HPC1_RX_FIFO_SIZE;
        u->rx_count--;
        qemu_chr_fe_accept_input(&s->serial);
        if (u->rx_count == 0) {
            u->rr3 &= ~(c == 0 ? SCC_RX_IP_A : SCC_RX_IP);
            scc_update_irq(s);
        }
    }
    return val;
}

static void scc_push_rx(SGIHPC1State *s, int d, int c, uint8_t b)
{
    SGIHPC1Uart *u = &s->uart[d][c];

    if (u->rx_count >= HPC1_RX_FIFO_SIZE) {
        return;
    }
    u->rx_fifo[u->rx_head] = b;
    u->rx_head = (u->rx_head + 1) % HPC1_RX_FIFO_SIZE;
    u->rx_count++;
    u->rr3 |= (c == 0 ? SCC_RX_IP_A : SCC_RX_IP);
    scc_update_irq(s);
}

static void scc_data_write(SGIHPC1State *s, int d, int c, uint8_t val)
{
    SGIHPC1Uart *u = &s->uart[d][c];

    scc_tx(s, d, c, val);

    /*
     * WR14 bit 4 (0x10) = Local Loopback: the transmitter output is wired
     * internally to the receiver input, so a byte written to the transmit
     * buffer returns on RX with no cable or device present. (SGI's z8530.h:
     * WR14_LCL_LPBK 0x10; 0x02 is WR14_BRG_PCLK.) The IP20 power-on
     * "Keyboard/Mouse diagnostic" (duart_lpbk.c / z8530_func) runs exactly
     * this test -- internal loopback -- and then waits for the RX interrupt.
     */
    if (u->wr[14] & 0x10) {
        scc_push_rx(s, d, c, val);
    }
}

/* ------------------------------------------------------------------ */
/* IP20 keyboard HLE                                                   */
/* ------------------------------------------------------------------ */

/* Push one byte into the keyboard channel's RX FIFO (DUART0 channel A). */
static void sgi_hpc1_kbd_push(SGIHPC1State *s, uint8_t b)
{
    scc_push_rx(s, 0, 0, b);
}

/*
 * Host key -> the PROM's SGI keyboard scancode. Shared with personal-iris's
 * IP6 HLE (branch keyboard-hle): same SGI keyboard protocol family, same US
 * layout table. Label MINUS ONE is emitted (measured on IP6). Keys not in the
 * table are DROPPED rather than guessed.
 */
static int sgi_hpc1_kbd_keycode(QKeyCode q)
{
    switch (q) {
    case Q_KEY_CODE_Q: return 10;  case Q_KEY_CODE_W: return 16;
    case Q_KEY_CODE_E: return 17;  case Q_KEY_CODE_R: return 24;
    case Q_KEY_CODE_T: return 25;  case Q_KEY_CODE_Y: return 32;
    case Q_KEY_CODE_U: return 33;  case Q_KEY_CODE_I: return 40;
    case Q_KEY_CODE_O: return 41;  case Q_KEY_CODE_P: return 48;
    case Q_KEY_CODE_A: return 11;  case Q_KEY_CODE_S: return 12;
    case Q_KEY_CODE_D: return 18;  case Q_KEY_CODE_F: return 19;
    case Q_KEY_CODE_G: return 26;  case Q_KEY_CODE_H: return 27;
    case Q_KEY_CODE_J: return 34;  case Q_KEY_CODE_K: return 35;
    case Q_KEY_CODE_L: return 42;  case Q_KEY_CODE_Z: return 20;
    case Q_KEY_CODE_X: return 21;  case Q_KEY_CODE_C: return 28;
    case Q_KEY_CODE_V: return 29;  case Q_KEY_CODE_B: return 36;
    case Q_KEY_CODE_N: return 37;  case Q_KEY_CODE_M: return 44;
    case Q_KEY_CODE_1: return 8;   case Q_KEY_CODE_2: return 14;
    case Q_KEY_CODE_3: return 15;  case Q_KEY_CODE_4: return 22;
    case Q_KEY_CODE_5: return 23;  case Q_KEY_CODE_6: return 30;
    case Q_KEY_CODE_7: return 31;  case Q_KEY_CODE_8: return 38;
    case Q_KEY_CODE_9: return 39;  case Q_KEY_CODE_0: return 46;
    case Q_KEY_CODE_RET: return 51;        case Q_KEY_CODE_KP_ENTER: return 82;
    case Q_KEY_CODE_SPC: return 83;
    case Q_KEY_CODE_BACKSPACE: return 61;  case Q_KEY_CODE_ESC: return 7;
    case Q_KEY_CODE_TAB: return 9;
    case Q_KEY_CODE_MINUS: return 47;      case Q_KEY_CODE_EQUAL: return 54;
    case Q_KEY_CODE_SLASH: return 53;      case Q_KEY_CODE_DOT: return 52;
    case Q_KEY_CODE_COMMA: return 45;      case Q_KEY_CODE_SEMICOLON: return 43;
    case Q_KEY_CODE_APOSTROPHE: return 50; case Q_KEY_CODE_BRACKET_LEFT: return 49;
    case Q_KEY_CODE_BRACKET_RIGHT: return 56; case Q_KEY_CODE_BACKSLASH: return 57;
    case Q_KEY_CODE_GRAVE_ACCENT: return 55;
    /* Modifiers carry the same guest-side offset (guest X keycode = V+7, from
     * the guest's own keymap): Shift_L=13, Shift_R=12, Control_L=10,
     * Caps_Lock=11, Alt_L/Meta_L=91, Alt_R/Meta_R=92, Control_R=93.  Without
     * these the HLE dropped every modifier, so shift-q arrived as plain q. */
    case Q_KEY_CODE_SHIFT: return 6;       case Q_KEY_CODE_SHIFT_R: return 5;
    case Q_KEY_CODE_CTRL: return 3;        case Q_KEY_CODE_CTRL_R: return 86;
    case Q_KEY_CODE_ALT: return 84;        case Q_KEY_CODE_ALT_R: return 85;
    case Q_KEY_CODE_META_L: return 84;     case Q_KEY_CODE_META_R: return 85;
    case Q_KEY_CODE_CAPS_LOCK: return 4;
    /* Navigation, function and keypad keys.  Same rule, V = X keycode - 7,
     * keycodes read from the guest's own keymap.  The keypad keys carry X's
     * Num_Lock duality -- one physical key, two keysyms -- so both halves of a
     * pair map to one V (KP_1 and KP_End are both keycode 65, hence V 58). */
    case Q_KEY_CODE_INSERT: return 102;    case Q_KEY_CODE_DELETE: return 62;
    case Q_KEY_CODE_HOME: return 103;      case Q_KEY_CODE_END: return 105;
    case Q_KEY_CODE_PGUP: return 104;      case Q_KEY_CODE_PGDN: return 106;
    case Q_KEY_CODE_LEFT: return 73;       case Q_KEY_CODE_RIGHT: return 80;
    case Q_KEY_CODE_UP: return 81;         case Q_KEY_CODE_DOWN: return 74;
    case Q_KEY_CODE_F1: return 87;         case Q_KEY_CODE_F2: return 88;
    case Q_KEY_CODE_F3: return 89;         case Q_KEY_CODE_F4: return 90;
    case Q_KEY_CODE_F5: return 91;         case Q_KEY_CODE_F6: return 92;
    case Q_KEY_CODE_F7: return 93;         case Q_KEY_CODE_F8: return 94;
    case Q_KEY_CODE_F9: return 95;         case Q_KEY_CODE_F10: return 96;
    case Q_KEY_CODE_F11: return 97;        case Q_KEY_CODE_F12: return 98;
    case Q_KEY_CODE_PRINT: return 99;      case Q_KEY_CODE_SYSRQ: return 99;
    case Q_KEY_CODE_SCROLL_LOCK: return 100;
    case Q_KEY_CODE_PAUSE: return 101;     case Q_KEY_CODE_NUM_LOCK: return 107;
    case Q_KEY_CODE_KP_0: return 59;       case Q_KEY_CODE_KP_1: return 58;
    case Q_KEY_CODE_KP_2: return 64;       case Q_KEY_CODE_KP_3: return 65;
    case Q_KEY_CODE_KP_4: return 63;       case Q_KEY_CODE_KP_5: return 69;
    case Q_KEY_CODE_KP_6: return 70;       case Q_KEY_CODE_KP_7: return 67;
    case Q_KEY_CODE_KP_8: return 68;       case Q_KEY_CODE_KP_9: return 75;
    case Q_KEY_CODE_KP_DECIMAL: return 66; case Q_KEY_CODE_KP_SUBTRACT: return 76;
    case Q_KEY_CODE_KP_DIVIDE: return 108; case Q_KEY_CODE_KP_MULTIPLY: return 109;
    case Q_KEY_CODE_KP_ADD: return 110;
    default: return -1;
    }
}

/* QEMU key event -> SGI scancode on the keyboard channel (bit7 = release). */
static void sgi_hpc1_kbd_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    SGIHPC1State *s = SGI_HPC1(dev);
    InputKeyEvent *key;
    int code;

    assert(evt->type == INPUT_EVENT_KIND_KEY);
    key = evt->u.key.data;
    code = sgi_hpc1_kbd_keycode(qemu_input_key_value_to_qcode(key->key));
    if (code < 1) {
        return;                        /* unknown key: drop, never invent */
    }
    sgi_hpc1_kbd_push(s, (uint8_t)((code - 1) | (key->down ? 0x00 : 0x80)));
}

static const QemuInputHandler sgi_hpc1_kbd_handler = {
    .name  = "sgi-hpc1-kbd",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = sgi_hpc1_kbd_event,
};


/* ------------------------------------------------------------------ */
/* IP20 mouse HLE (DUART0B)                                             */
/* ------------------------------------------------------------------ */

/* Emit one Mouse Systems 5-byte packet on the mouse channel (DUART0B). */
static void sgi_hpc1_mouse_packet(SGIHPC1State *s, int dx, int dy)
{
    uint8_t pkt[5];
    int dx1, dx2, dy1, dy2, i;

    /*
     * b1+b3 = dx, b2+b4 = dy; the driver negates dy (sgi_ms.c:
     * deltay = -(report[2]+report[4])), so the wire's Y grows upward while
     * ours (like QEMU's) grows downward. Split each delta into two halves
     * rather than one large byte: a delta byte in 0x80..0x87 satisfies the
     * driver's issync() test ((b & 0xF8) == 0x80) and would reset its packet
     * state mid-frame, so keep every emitted byte within +/-120.
     */
    dx = dx > 240 ? 240 : (dx < -240 ? -240 : dx);
    dx1 = dx / 2;
    dx2 = dx - dx1;

    dy = -dy;                            /* wire Y grows upward */
    dy = dy > 240 ? 240 : (dy < -240 ? -240 : dy);
    dy1 = dy / 2;
    dy2 = dy - dy1;

    pkt[0] = 0x80 | s->mouse_buttons;
    pkt[1] = (uint8_t)dx1;
    pkt[2] = (uint8_t)dy1;
    pkt[3] = (uint8_t)dx2;
    pkt[4] = (uint8_t)dy2;

    for (i = 0; i < 5; i++) {
        scc_push_rx(s, 0, 1, pkt[i]);
    }
}

static void sgi_hpc1_mouse_event(DeviceState *dev, QemuConsole *src,
                                 InputEvent *evt)
{
    SGIHPC1State *s = SGI_HPC1(dev);
    InputMoveEvent *move;
    InputBtnEvent *btn;

    switch (evt->type) {
    case INPUT_EVENT_KIND_REL:
        move = evt->u.rel.data;
        if (move->axis == INPUT_AXIS_X) {
            s->mouse_dx += move->value;
        } else if (move->axis == INPUT_AXIS_Y) {
            s->mouse_dy += move->value;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        btn = evt->u.btn.data;
        /* Active-low button byte, 0x07 = none; MS order left/mid/right. */
        switch (btn->button) {
        case INPUT_BUTTON_LEFT:
            s->mouse_buttons = btn->down ? (s->mouse_buttons & ~0x04)
                                         : (s->mouse_buttons | 0x04);
            break;
        case INPUT_BUTTON_MIDDLE:
            s->mouse_buttons = btn->down ? (s->mouse_buttons & ~0x02)
                                         : (s->mouse_buttons | 0x02);
            break;
        case INPUT_BUTTON_RIGHT:
            s->mouse_buttons = btn->down ? (s->mouse_buttons & ~0x01)
                                         : (s->mouse_buttons | 0x01);
            break;
        default:
            return;                     /* wheel etc.: not on this mouse */
        }
        break;
    default:
        return;
    }

    sgi_hpc1_mouse_packet(s, s->mouse_dx, s->mouse_dy);
    s->mouse_dx = 0;
    s->mouse_dy = 0;
}

static const QemuInputHandler sgi_hpc1_mouse_handler = {
    .name  = "sgi-hpc1-mouse",
    .mask  = INPUT_EVENT_MASK_REL | INPUT_EVENT_MASK_BTN,
    .event = sgi_hpc1_mouse_event,
};


/* ------------------------------------------------------------------ */
/* Interrupt aggregation                                               */
/* ------------------------------------------------------------------ */

static void int2_update(SGIHPC1State *s)
{
    bool irq0 = (s->lio_status[0] & s->lio_mask[0]) != 0;
    bool irq1 = (s->lio_status[1] & s->lio_mask[1]) != 0;

    qemu_set_irq(s->cpu_irq[0], irq0);
    qemu_set_irq(s->cpu_irq[1], irq1);
}

static void scc_update_irq(SGIHPC1State *s)
{
    bool pending = false;
    int d, c;

    for (d = 0; d < HPC1_NUM_DUARTS; d++) {
        for (c = 0; c < HPC1_DUART_CH; c++) {
            SGIHPC1Uart *u = &s->uart[d][c];
            if (u->rr3 && (s->uart[d][0].wr[9] & 0x08)) {
                pending = true;
            }
        }
    }

    if (pending) {
        s->lio_status[0] |= LIO0_DUART;
    } else {
        s->lio_status[0] &= ~LIO0_DUART;
    }
    int2_update(s);
}

static void hpc1_scsi_irq(void *opaque, int n, int level)
{
    SGIHPC1State *s = SGI_HPC1(opaque);

    if (level) {
        s->lio_status[0] |= LIO0_SCSI;
    } else {
        s->lio_status[0] &= ~LIO0_SCSI;
    }
    int2_update(s);
}

/* ------------------------------------------------------------------ */
/* HPC1 SCSI DMA engine (WD33C93 <-> memory descriptor chains)         */
/* ------------------------------------------------------------------ */

#define HPC1_SCSI_BUFADDR 0x0fffffffU
#define HPC1_SCSI_EOX     0x80000000U
#define HPC1_SCSI_BC_MASK 0x1fffU

#define SCSI_CTRL_RESET   0x01
#define SCSI_CTRL_FLUSH   0x02
#define SCSI_CTRL_TO_MEM  0x10
#define SCSI_CTRL_START   0x80

static void hpc1_scsi_chain(SGIHPC1State *s)
{
    uint32_t bdp = s->scsi_nbdp & HPC1_SCSI_BUFADDR;

    s->scsi_bc = address_space_ldl_be(&address_space_memory, bdp,
                                      MEMTXATTRS_UNSPECIFIED, NULL) &
                 HPC1_SCSI_BC_MASK;
    s->scsi_cbp = address_space_ldl_be(&address_space_memory, bdp + 4,
                                       MEMTXATTRS_UNSPECIFIED, NULL) &
                  (HPC1_SCSI_EOX | HPC1_SCSI_BUFADDR);
    s->scsi_nbdp = address_space_ldl_be(&address_space_memory, bdp + 8,
                                        MEMTXATTRS_UNSPECIFIED, NULL) &
                   HPC1_SCSI_BUFADDR;
    s->scsi_dma_count = s->scsi_bc;
}

static void hpc1_scsi_dma_run(SGIHPC1State *s)
{
    WD33C93State *wdc = s->scsi;

    while (wdc && wdc->async_len > 0 && s->scsi_dma_active) {
        uint32_t chunk;

        if (s->scsi_dma_count == 0) {
            if (s->scsi_cbp & HPC1_SCSI_EOX) {
                s->scsi_dma_active = false;
                s->scsi_ctrl &= ~SCSI_CTRL_START;
                break;
            }
            hpc1_scsi_chain(s);
            if (s->scsi_dma_count == 0) {
                break;
            }
        }

        chunk = MIN(wdc->async_len, s->scsi_dma_count);
        if (wdc->transfer_count > 0) {
            chunk = MIN(chunk, wdc->transfer_count);
        }
        if (chunk == 0) {
            break;
        }

        if (s->scsi_dma_to_device) {
            /* SCSI_CTRL bit 0x10 = "to memory": device -> memory (READ) */
            address_space_write(&address_space_memory,
                                s->scsi_cbp & HPC1_SCSI_BUFADDR,
                                MEMTXATTRS_UNSPECIFIED, wdc->async_buf, chunk);
        } else {
            /* memory -> device (WRITE) */
            address_space_read(&address_space_memory,
                               s->scsi_cbp & HPC1_SCSI_BUFADDR,
                               MEMTXATTRS_UNSPECIFIED, wdc->async_buf, chunk);
        }
        s->scsi_cbp += chunk;
        wdc->async_buf += chunk;
        wdc->async_len -= chunk;
        s->scsi_dma_count -= chunk;
        if (wdc->transfer_count > 0) {
            wdc->transfer_count -= chunk;
            wd33c93_set_transfer_count(wdc, wdc->transfer_count);
        }

        if (s->scsi_dma_count == 0) {
            if (s->scsi_cbp & HPC1_SCSI_EOX) {
                s->scsi_dma_active = false;
                s->scsi_ctrl &= ~SCSI_CTRL_START;
            } else {
                hpc1_scsi_chain(s);
            }
        }
    }

    /*
     * Drain trailing zero-count terminal descriptors (mirrors HPC3). The loop
     * above exits as soon as async_len reaches 0, but a pending EOX
     * descriptor with BC=0 still has to be processed to clear dma_active and
     * the START bit; otherwise the DMA stays "active" and the transfer never
     * completes (observed as the miniroot mkfs aborting its read-back).
     */
    while (s->scsi_dma_active && s->scsi_dma_count == 0) {
        if (s->scsi_cbp & HPC1_SCSI_EOX) {
            s->scsi_dma_active = false;
            s->scsi_ctrl &= ~SCSI_CTRL_START;
            break;
        }
        hpc1_scsi_chain(s);
    }

    if (wdc && wdc->current_req &&
        (wdc->async_len == 0 || wdc->transfer_count == 0)) {
        wd33c93_set_drq(wdc, false);
        s->scsi_drq = false;
        if (wdc->transfer_count == 0 && wdc->pending_len > 0) {
            wdc->async_len = 0;
            wdc->async_buf = NULL;
            wdc->aux_status &= ~(ASR_DBR | ASR_CIP | ASR_BSY);
            wdc->regs[WD_COMMAND_PHASE] = 0x46;
            /*
             * scsi_dma_to_device is the CTRL "to memory" bit, i.e. a READ
             * (device -> memory): the chip is the sender, so the correct
             * unexpected-phase status is UNEX_SDATA. UNEX_RDATA (chip
             * receiving) is for writes. Reporting these backwards makes the
             * IRIX driver print "Too much data sent (probable SCSI bus cabling
             * problem). Resetting SCSI bus".
             * Reference: wd33c93_transfer_data()'s TC==0 branch (read ->
             * UNEX_SDATA, write -> UNEX_RDATA).
             */
            wdc->scsi_status = s->scsi_dma_to_device
                               ? SCSI_STATUS_UNEX_SDATA
                               : SCSI_STATUS_UNEX_RDATA;
            wdc->aux_status |= ASR_INT;
            qemu_irq_raise(wdc->irq);
        } else {
            if (wdc->async_len > 0) {
                wdc->async_len = 0;
                wdc->async_buf = NULL;
            }
            scsi_req_continue(wdc->current_req);
        }
    }
}

static void hpc1_scsi_drq(void *opaque, int n, int level)
{
    SGIHPC1State *s = SGI_HPC1(opaque);

    s->scsi_drq = !!level;
    if (level && s->scsi_dma_active) {
        hpc1_scsi_dma_run(s);
    }
}

static void hpc1_scsi_ctrl_write(SGIHPC1State *s, uint32_t val)
{
    if (val & SCSI_CTRL_RESET) {
        s->scsi_ctrl = val & ~(SCSI_CTRL_RESET | SCSI_CTRL_FLUSH);
        s->scsi_dma_active = false;
        return;
    }

    s->scsi_ctrl = val & ~SCSI_CTRL_FLUSH;
    s->scsi_dma_active = !!(val & SCSI_CTRL_START);
    s->scsi_dma_to_device = !!(val & SCSI_CTRL_TO_MEM);

    /*
     * Do NOT re-fetch the chain here: the descriptor is loaded when the
     * PROM writes NBDP (matching MAME hpc1 scsi_nbdp_w -> scsi_chain).
     */
    if (s->scsi_dma_active && s->scsi_drq) {
        hpc1_scsi_dma_run(s);
    }

    if (val & SCSI_CTRL_FLUSH) {
        s->scsi_dma_count = 0;
        s->scsi_dma_active = false;
        s->scsi_ctrl &= ~(SCSI_CTRL_START | SCSI_CTRL_FLUSH);
    }
}

/* ------------------------------------------------------------------ */
/* 8254 PIT inside INT2                                                */
/* ------------------------------------------------------------------ */

/*
 * The 8254 reload value 0 means 65536, not "stopped".  IRIX 6.2 arms its
 * kernel clock with a divisor of 0, so treating 0 as stopped leaves the
 * system with no timer tick (no scheduling, permanent idle).  Keep the
 * explicit pit_programmed gate to distinguish "never armed".
 */
static uint32_t hpc1_pit_reload(const SGIHPC1State *s, int ch)
{
    return s->pit_count[ch] ? s->pit_count[ch] : 65536u;
}

/*
 * Counters 0/1 are clocked by counter 2's output: its 10 MHz input divided by
 * counter 2's reload. Counter 2 itself must stay clocked at 10 MHz (the PROM's
 * power-on timer calibration reads it back and asserts on a stalled counter);
 * only counters 0/1 are divided. IRIX arms counter 2 as a divider, so counter 0
 * then ticks slowly enough for the scheduler.
 */
static uint32_t hpc1_pit_ch01_clk_hz(const SGIHPC1State *s)
{
    uint32_t hz = PIT_BASE_CLK_HZ / hpc1_pit_reload(s, 2);
    return hz ? hz : 1;
}

static int64_t hpc1_pit_period_ns(const SGIHPC1State *s, int ch)
{
    uint32_t clk = (ch < 2) ? hpc1_pit_ch01_clk_hz(s) : PIT_BASE_CLK_HZ;
    int64_t period_ns = (int64_t)hpc1_pit_reload(s, ch) * 1000000000LL / clk;
    return period_ns < 1000 ? 1000 : period_ns;
}

/*
 * Fire one PIT channel.  Per-channel callbacks are used because a shared
 * callback cannot tell which channel fired: QEMU clears a timer's pending
 * flag before invoking its callback, so polling timer_pending() inside the
 * handler always sees false and the channel is never re-armed (the original
 * bug: the PIT fired at most once, so IRIX never got a periodic tick).
 */
static void hpc1_pit_fire(SGIHPC1State *s, int ch)
{
    unsigned mode = (s->pit_control[ch] >> 1) & 0x7;
    int64_t period_ns;

    if (!s->pit_programmed[ch]) {
        return;
    }

    /*
     * Assert the corresponding CPU interrupt via the dedicated timer lines
     * (timer0 -> IP4, timer1 -> IP5).  Raise and HOLD: the CPU latches the
     * interrupt from the line level, so a momentary pulse is lost; the line
     * is lowered when the guest acks via INT2_TIMER_ACK.
     */
    qemu_irq_raise(s->timer_irq[ch]);

    /* Rate generator (mode 2/3) repeats; one-shot modes fire once. */
    if (mode == 2 || mode == 3) {
        period_ns = hpc1_pit_period_ns(s, ch);
        timer_mod(s->pit_timer[ch],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
    }
}

static void hpc1_pit_timer0_cb(void *opaque)
{
    hpc1_pit_fire(opaque, 0);
}

static void hpc1_pit_timer1_cb(void *opaque)
{
    hpc1_pit_fire(opaque, 1);
}

static uint16_t hpc1_pit_remaining(SGIHPC1State *s, int ch)
{
    uint32_t reload = hpc1_pit_reload(s, ch);
    uint32_t clk_hz = (ch < 2) ? hpc1_pit_ch01_clk_hz(s) : PIT_BASE_CLK_HZ;
    int64_t elapsed_ns;
    uint64_t ticks;
    unsigned mode = (s->pit_control[ch] >> 1) & 0x7;

    if (!s->pit_programmed[ch]) {
        return s->pit_count[ch];
    }

    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->pit_load_ns[ch];
    if (elapsed_ns < 0) {
        elapsed_ns = 0;
    }
    ticks = elapsed_ns / (1000000000ULL / clk_hz);

    /*
     * TCG can execute a short guest loop in well under one PIT tick, so a
     * strict real-time counter can read the full reload and make the PROM's
     * speed calibration compute "0 ticks". The real part always sees at
     * least one tick over such a loop, so floor the elapsed time at one tick
     * once the counter has been started.
     */
    if (ticks == 0) {
        ticks = 1;
    }

    if (mode == 2 || mode == 3) {
        /* Rate generator: wrap around. */
        ticks %= reload;
        return (uint16_t)(reload - ticks);
    }
    /* One-shot modes: stop at zero. */
    if (ticks >= reload) {
        return 0;
    }
    return (uint16_t)(reload - ticks);
}

static void hpc1_pit_write(SGIHPC1State *s, int reg, uint8_t val)
{
    int ch, rw;

    if (reg == 3) {
        ch = (val >> 6) & 0x03;
        if (ch < 3) {
            s->pit_control[ch] = val;
            s->pit_rw_state[ch] = 0;
            s->pit_read_state[ch] = 0;
        }
        return;
    }

    if (reg >= 3) {
        return;
    }

    ch = reg;
    rw = (s->pit_control[ch] >> 4) & 0x3;
    switch (rw) {
    case 1: /* LSB only */
        s->pit_count[ch] = val;
        break;
    case 2: /* MSB only */
        s->pit_count[ch] = (uint16_t)val << 8;
        break;
    case 3: /* LSB then MSB */
        if (s->pit_rw_state[ch] == 0) {
            s->pit_low[ch] = val;
            s->pit_rw_state[ch] = 1;
            return;
        }
        s->pit_count[ch] = s->pit_low[ch] | ((uint16_t)val << 8);
        s->pit_rw_state[ch] = 0;
        break;
    default:
        return;
    }
    s->pit_programmed[ch] = true;
    s->pit_load_ns[ch] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * Arm the modelled timer on (re)programming.  The callback re-arms
     * itself, but nothing else starts it, so without this the PIT never
     * fires.  IRIX uses the HPC1 PIT for its kernel clock, so a dead PIT
     * leaves the system with no tick (permanent idle, no init).
     *
     * Counters 0/1 derive their rate from counter 2's reload (cascade), so
     * reprogramming counter 2 must re-arm them at the new rate.
     */
    if (ch < 2) {
        timer_mod(s->pit_timer[ch],
                  s->pit_load_ns[ch] + hpc1_pit_period_ns(s, ch));
    } else {
        int c;
        for (c = 0; c < 2; c++) {
            if (s->pit_programmed[c]) {
                timer_mod(s->pit_timer[c],
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          hpc1_pit_period_ns(s, c));
            }
        }
    }
}

static uint8_t hpc1_pit_read(SGIHPC1State *s, int reg)
{
    int ch, rw;
    uint16_t rem;

    if (reg == 3) {
        return s->pit_control[0];
    }
    if (reg >= 3) {
        return 0;
    }

    ch = reg;
    rw = (s->pit_control[ch] >> 4) & 0x3;
    rem = hpc1_pit_remaining(s, ch);

    switch (rw) {
    case 1: /* LSB only */
        return rem & 0xff;
    case 2: /* MSB only */
        return (rem >> 8) & 0xff;
    default: /* LSB then MSB */
        if (s->pit_read_state[ch] == 0) {
            s->pit_read_state[ch] = 1;
            return rem & 0xff;
        }
        s->pit_read_state[ch] = 0;
        return (rem >> 8) & 0xff;
    }
}

/* ------------------------------------------------------------------ */
/* DP8572 real-time clock                                              */
/* ------------------------------------------------------------------ */

/* Register indices (MAME dp8573a.cpp; DP8572A exposes 0x00-0x1f) */
#define RTC_REG_MSR        0x00
#define RTC_REG_RTMR       0x01
#define RTC_REG_OMR        0x02
#define RTC_REG_PFR        0x03
#define RTC_REG_TSCR       0x04
#define RTC_REG_HUNDREDTH  0x05
#define RTC_REG_SECOND     0x06
#define RTC_REG_MINUTE     0x07
#define RTC_REG_HOUR       0x08
#define RTC_REG_DAY        0x09
#define RTC_REG_MONTH      0x0a
#define RTC_REG_YEAR       0x0b
#define RTC_REG_DAYOFWEEK  0x0e
/* Real-time-mode register (word index 1). Bit 3-4 (3<<3) is RTC_RUN: both the
 * IP20 PROM (libsk/ml/dp8573.c) and the IRIX kernel (ml/IP20.c _clock_func_read)
 * refuse the time if this bit is clear, declaring "Initializing tod clock" /
 * "lost battery backup clock" and resetting the guest clock to epoch 0. The low
 * two bits are the leap-year counter. */
#define RTC_REG_RTIME1     0x01
#define RTC_RUN            0x18 /* (3 << 3): clock running */

static uint8_t rtc_to_bcd(int v)
{
    return ((v / 10) << 4) | (v % 10);
}

static int rtc_from_bcd(uint8_t v)
{
    return ((v >> 4) & 0xf) * 10 + (v & 0xf);
}

static void rtc_guest_tm(SGIHPC1State *s, struct tm *tm)
{
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_HOST);
    time_t t = (time_t)((s->rtc_guest_base_ms +
                         (now - s->rtc_host_base_ms)) / 1000);
    gmtime_r(&t, tm);
}

static void rtc_set_from_tm(SGIHPC1State *s, struct tm *tm)
{
    time_t t = timegm(tm);
    s->rtc_guest_base_ms = (int64_t)t * 1000;
    s->rtc_host_base_ms = qemu_clock_get_ms(QEMU_CLOCK_HOST);
}

/*
 * Time fields as the IP20 PROM maps them (derived from the access pattern
 * of libsk/ml/dp8573.c): ck_counter[0..6] = regs 0x05..0x0b and the Time
 * Save RAM ck_timsav[0..4] = regs 0x19..0x1d mirror seconds..month.
 */
static uint8_t rtc_reg_read(SGIHPC1State *s, unsigned reg)
{
    struct tm tm;

    switch (reg) {
    case RTC_REG_HUNDREDTH: /* ck_counter[0]: polled by counter_not_moving() */
        return rtc_to_bcd((int)((qemu_clock_get_ms(QEMU_CLOCK_HOST) / 10) % 100));
    case RTC_REG_SECOND:
    case RTC_REG_MINUTE:
    case RTC_REG_HOUR:
    case RTC_REG_DAY:
    case RTC_REG_MONTH:
    case RTC_REG_YEAR:
    case RTC_REG_DAYOFWEEK:
    case 0x19: /* ck_timsav[0] - seconds */
    case 0x1a: /* ck_timsav[1] - minutes */
    case 0x1b: /* ck_timsav[2] - hours */
    case 0x1c: /* ck_timsav[3] - day */
    case 0x1d: /* ck_timsav[4] - month */
        rtc_guest_tm(s, &tm);
        switch (reg) {
        case RTC_REG_SECOND:
        case 0x19:
            return rtc_to_bcd(tm.tm_sec);
        case RTC_REG_MINUTE:
        case 0x1a:
            return rtc_to_bcd(tm.tm_min);
        case RTC_REG_HOUR:
        case 0x1b:
            return rtc_to_bcd(tm.tm_hour);
        case RTC_REG_DAY:
        case 0x1c:
            return rtc_to_bcd(tm.tm_mday);
        case RTC_REG_MONTH:
        case 0x1d:
            return rtc_to_bcd(tm.tm_mon + 1);
        case RTC_REG_YEAR:
            /* The DP8573 year counter is 2 BCD digits of years SINCE 1970
             * (IRIX YRREF = 1970; the kernel/PROM add 1970 back). */
            return rtc_to_bcd(((tm.tm_year + 1900) - 1970) % 100);
        default: /* day of week: 1-7, Sunday = 1 */
            return rtc_to_bcd(tm.tm_wday + 1);
        }
    default:
        return s->rtc[reg & 0x7f];
    }
}

static void rtc_reg_write(SGIHPC1State *s, unsigned reg, uint8_t val)
{
    struct tm tm;

    switch (reg) {
    case RTC_REG_SECOND:
    case RTC_REG_MINUTE:
    case RTC_REG_HOUR:
    case RTC_REG_DAY:
    case RTC_REG_MONTH:
    case RTC_REG_YEAR:
        rtc_guest_tm(s, &tm);
        switch (reg) {
        case RTC_REG_SECOND:
            tm.tm_sec = rtc_from_bcd(val & 0x7f);
            break;
        case RTC_REG_MINUTE:
            tm.tm_min = rtc_from_bcd(val & 0x7f);
            break;
        case RTC_REG_HOUR:
            tm.tm_hour = rtc_from_bcd(val & 0x3f);
            break;
        case RTC_REG_DAY:
            tm.tm_mday = rtc_from_bcd(val & 0x3f);
            break;
        case RTC_REG_MONTH:
            tm.tm_mon = rtc_from_bcd(val & 0x1f) - 1;
            break;
        default: { /* year: 2 BCD digits = years since 1970 (IRIX YRREF) */
            int full = 1970 + rtc_from_bcd(val & 0x7f);
            int cur = tm.tm_year + 1900;
            while (full < cur - 50) {
                full += 100;
            }
            while (full > cur + 50) {
                full -= 100;
            }
            tm.tm_year = full - 1900;
            break;
        }
        }
        rtc_set_from_tm(s, &tm);
        break;
    default:
        s->rtc[reg & 0x7f] = val;
        break;
    }
}

static void rtc_reset(SGIHPC1State *s)
{
    struct tm tm;
    time_t t = time(NULL);

    gmtime_r(&t, &tm);
    s->rtc_guest_base_ms = (int64_t)t * 1000;
    s->rtc_host_base_ms = qemu_clock_get_ms(QEMU_CLOCK_HOST);
    memset(s->rtc, 0, sizeof(s->rtc));
    s->rtc[RTC_REG_MSR] = 0;
    /* Report the clock as running (virtualization-native: the DP8573's run
     * state is restored from the host wall clock, not from a battery). Without
     * this the PROM and kernel treat the clock as dead and re-initialize it to
     * epoch 0 on every boot. */
    s->rtc[RTC_REG_RTIME1] = RTC_RUN | ((tm.tm_year + 1900) % 4);
}

/* ------------------------------------------------------------------ */
/* EEPROM bit-bang via the aux register                                */
/* ------------------------------------------------------------------ */

/* Aux register field positions (MAME ip20.cpp) */
#define EE_BIT_CS   1
#define EE_BIT_CLK  2
#define EE_BIT_DI   3
#define EE_BIT_DO   4

/*
 * Persist the 93C56 (128 x 16-bit words) to the NVRAM backing file.
 * Bytes are stored big-endian within each word, matching the HPC3/IP22
 * convention so the file layout is consistent across machines.
 */
static void hpc1_nvram_save(SGIHPC1State *s)
{
    uint8_t table[256];
    FILE *f;
    int i;

    if (!s->nvram_filename) {
        return;
    }
    for (i = 0; i < 128; i++) {
        table[i * 2] = s->nvram[i] >> 8;
        table[i * 2 + 1] = s->nvram[i] & 0xff;
    }
    f = fopen(s->nvram_filename, "wb");
    if (f) {
        fwrite(table, sizeof(table), 1, f);
        fclose(f);
    }
}

static void hpc1_nvram_load(SGIHPC1State *s)
{
    uint8_t table[256];
    FILE *f;
    int i;

    if (!s->nvram_filename) {
        return;
    }
    f = fopen(s->nvram_filename, "rb");
    if (!f) {
        return;
    }
    if (fread(table, sizeof(table), 1, f) == 1) {
        for (i = 0; i < 128; i++) {
            s->nvram[i] = (table[i * 2] << 8) | table[i * 2 + 1];
        }
    }
    fclose(f);
}

/* SGI NVRAM checksum: seed 0xa5, XOR each non-zero byte, rotate on odd. */
static uint8_t hpc1_nvram_checksum(const uint8_t *table, int len)
{
    int8_t checksum = (int8_t)0xa5;
    int i;

    for (i = 0; i < len; i++) {
        if (i != 0) {
            checksum ^= (int8_t)table[i];
        }
        if (i & 1) {
            checksum = (checksum << 1) | ((uint8_t)checksum >> 7);
        }
    }
    return (uint8_t)checksum;
}

/*
 * IP20 NVRAM layout (sys/IP20nvram.h) -- offsets match the HPC3 family.
 * AutoLoad is set to 'N' so a diskless/blank system lands on the System
 * Maintenance Menu instead of looping through an autoboot that fails.
 */
static void hpc1_nvram_init_defaults(SGIHPC1State *s)
{
    uint8_t t[256];
    int i;

    memset(t, 0, sizeof(t));
    t[1] = 6;                    /* NV_CURRENT_REV */
    t[2] = 'd';                  /* console = serial */
    memcpy(&t[116], "9600", 4);  /* dbaud */
    memcpy(&t[128], "PST8PDT", 7); /* timezone */
    t[121] = '0';                /* diskless */
    t[178] = 'Y';                /* autoload */
    t[232] = '8';
    t[233] = '0';                /* volume */
    t[236] = 'y';                /* sgilogo */
    t[240] = 'h';                /* monitor */
    t[250] = 0x08;
    t[251] = 0x00;
    t[252] = 0x69;               /* SGI OUI */
    t[253] = 0x12;
    t[254] = 0x34;
    t[255] = 0x56;
    t[0] = hpc1_nvram_checksum(t, sizeof(t));

    for (i = 0; i < 128; i++) {
        s->nvram[i] = (t[i * 2] << 8) | t[i * 2 + 1];
    }
}

/*
 * 93C56 Microwire bit engine.
 *
 * The PROM drives 0 (start), 1 (start), 2 opcode bits, 7 address bits,
 * then 16 data bits: opcode 10=READ, 01=WRITE, 11=ERASE, 00=EWEN/EWDS
 * (selected by address bits [6:5] = 3/0).
 */
static void hpc1_eeprom_bit(SGIHPC1State *s, int cs, int clk, int di)
{
    bool rising = cs && clk && !s->nv_clk;
    bool falling = s->nv_cs && !cs;
    int i;

    if (!s->nv_cs && cs) {
        s->nv_tick = 0;
        s->nv_opcode = 0;
        s->nv_addr = 0;
        s->nv_data = 0;
        s->nv_do = 1;
    }

    /*
     * Command frame, MSB first, per the PROM's OP20prom/r4k_eerom.s and
     * SER_* constants in sys/IP20nvram.h:
     *   1 start bit (1), 2 opcode bits, 8 address bits, then 16 data bits.
     *   opcode 10=READ, 01=WRITE, 11=ERASE, 00=EWEN/EWDS (addr[7:6]=3/0).
     */
    if (rising) {
        if (s->nv_tick == 0) {
            if (di) {                      /* start bit */
                s->nv_tick = 1;
            }
        } else if (s->nv_tick < 3) {       /* opcode bits */
            s->nv_opcode = (s->nv_opcode << 1) | di;
            s->nv_tick++;
        } else if (s->nv_tick < 11) {      /* 8 address bits (ticks 3..10) */
            s->nv_addr = (s->nv_addr << 1) | di;
            s->nv_tick++;
            if (s->nv_tick == 11) {
                if (s->nv_opcode == 0) {
                    switch (s->nv_addr >> 6) {
                    case 0: /* EWDS */
                        s->nv_writable = 0;
                        break;
                    case 3: /* EWEN */
                        s->nv_writable = 1;
                        break;
                    default:
                        break;
                    }
                } else if (s->nv_opcode == 2) { /* READ: latch word */
                    s->nv_data = s->nvram[s->nv_addr & 0x7f];
                }
            }
        } else if (s->nv_tick < 11 + 16) { /* 16 data bits (ticks 11..26) */
            if (s->nv_opcode == 2) {
                /* DO presents the current bit; the PROM samples SI on the
                 * rising edge it just produced. */
                s->nv_do = (s->nv_data >> 15) & 1;
                s->nv_data <<= 1;
            } else {
                s->nv_data = (s->nv_data << 1) | di;
            }
            s->nv_tick++;
        } else {
            /* Trailing bit(s) are ignored. */
        }
    }

    if (falling && s->nv_writable) {
        bool changed = false;
        switch (s->nv_opcode) {
        case 1: /* WRITE word */
            s->nvram[s->nv_addr & 0x7f] = s->nv_data;
            changed = true;
            break;
        case 3: /* ERASE word */
            s->nvram[s->nv_addr & 0x7f] = 0xffff;
            changed = true;
            break;
        case 0:
            if ((s->nv_addr >> 6) == 1) { /* WRAL */
                for (i = 0; i < 128; i++) {
                    s->nvram[i] = s->nv_data;
                }
                changed = true;
            } else if ((s->nv_addr >> 6) == 2) { /* ERAL */
                for (i = 0; i < 128; i++) {
                    s->nvram[i] = 0xffff;
                }
                changed = true;
            }
            break;
        default:
            break;
        }
        if (changed) {
            hpc1_nvram_save(s);
        }
    }

    s->nv_cs = cs;
    s->nv_clk = clk;
    s->nv_di = di;
}

static uint8_t hpc1_aux_read(SGIHPC1State *s)
{
    return (s->aux & ~(1 << EE_BIT_DO)) | (s->nv_do << EE_BIT_DO);
}

static void hpc1_aux_write(SGIHPC1State *s, uint8_t val)
{
    hpc1_eeprom_bit(s, (val >> EE_BIT_CS) & 1, (val >> EE_BIT_CLK) & 1,
                    (val >> EE_BIT_DI) & 1);
    s->aux = val;
}

/* ------------------------------------------------------------------ */
/* MMIO dispatch                                                       */
/* ------------------------------------------------------------------ */

static uint64_t sgi_hpc1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIHPC1State *s = opaque;
    uint8_t val8 = 0;

    if (addr >= HPC1_DUART_BASE && addr < HPC1_DUART_BASE + 0x40) {
        unsigned off = addr - HPC1_DUART_BASE;
        int d = off >> 4;
        int grp = (off >> 2) & 0x3;
        int c = (grp & 0x2) ? 0 : 1;   /* grp bit1: 1 = channel A */
        bool is_data = grp & 0x1;
        if (d >= HPC1_NUM_DUARTS) {
            return 0; /* headphone MDACs */
        }
        val8 = is_data ? scc_data_read(s, d, c) : scc_ctrl_read(s, d, c);
        return val8;
    }

    if (addr >= HPC1_RTC_BASE && addr < HPC1_RTC_BASE + 0x80) {
        return rtc_reg_read(s, (addr - HPC1_RTC_BASE) >> 2);
    }

    if (addr >= HPC1_INT2_BASE && addr < HPC1_INT2_BASE + 0x40) {
        int idx = ((addr - HPC1_INT2_BASE) >> 2) & 0xf;
        switch (idx) {
        case INT2_LIO0_STATUS:
            return s->lio_status[0];
        case INT2_LIO0_MASK:
            return s->lio_mask[0];
        case INT2_LIO1_STATUS:
            return s->lio_status[1];
        case INT2_LIO1_MASK:
            return s->lio_mask[1];
        case INT2_VME_STATUS:
            return s->vme_status;
        case INT2_VME0_MASK:
            return s->vme_mask[0];
        case INT2_VME1_MASK:
            return s->vme_mask[1];
        case INT2_CONFIG:
            return s->int2_config;
        default:
            if (idx >= INT2_PIT_BASE && idx <= INT2_PIT_BASE + 3) {
                return hpc1_pit_read(s, idx - INT2_PIT_BASE);
            }
            return 0;
        }
    }

    if (addr >= HPC1_SCSI_BASE && addr < HPC1_SCSI_BASE + 8) {
        unsigned off = addr - HPC1_SCSI_BASE;
        if (!s->scsi) {
            return 0;
        }
        return (off & 4) ? wd33c93_data_read(s->scsi)
                         : wd33c93_addr_read(s->scsi);
    }

    if (addr >= HPC1_ENET_XCOUNT && addr < 0x60) {
        uint32_t v;
        switch (addr & ~3ULL) {
        case HPC1_ENET_XCOUNT:   v = s->enet_xcount; break;
        case HPC1_ENET_CXBP:     v = s->enet_cxbp; break;
        case HPC1_ENET_NXBDP:    v = s->enet_nxbdp; break;
        case HPC1_ENET_XBC:      v = s->enet_xbc; break;
        case HPC1_ENET_CXBDP:    v = s->enet_cxbdp; break;
        case HPC1_ENET_CPFXBDP:  v = s->enet_cpfxbdp; break;
        case HPC1_ENET_PPFXBDP:  v = s->enet_ppfxbdp; break;
        case HPC1_ENET_INTDELAY: v = s->enet_intdelay; break;
        case HPC1_ENET_TRSTAT:   v = s->enet_trstat; break;
        case HPC1_ENET_RCVSTAT:  v = s->enet_rcvstat; break;
        case HPC1_ENET_CTL:      v = s->enet_ctl; break;
        case HPC1_ENET_RBC:      v = s->enet_rbc; break;
        case HPC1_ENET_CRBP:     v = s->enet_crbp; break;
        case HPC1_ENET_NRBDP:    v = s->enet_nrbdp; break;
        case HPC1_ENET_CRBDP:    v = s->enet_crbdp; break;
        default:                 v = 0; break;
        }
        return v;
    }

    if (addr >= HPC1_ENET_BASE && addr < HPC1_ENET_BASE + 0x20) {
        int reg = ((addr - HPC1_ENET_BASE) >> 2) & 7;
        uint8_t sv;
        switch (reg) {
        case 6:
            /* SEQ_RS_OLD is set ON READ, so the value returned is the status
             * as it stood before this read (HPC3 behaviour) -- otherwise the
             * driver would never see a fresh (OLD-clear) status. */
            sv = s->seeq_rx_status;
            s->seeq_rx_status |= 0x80;
            return sv;
        case 7:
            sv = s->seeq_tx_status;
            s->seeq_tx_status |= 0x80;
            return sv;
        default:
            /*
             * regs 0-5 are the banked union in seeq.h's struct EHIO
             * (seq_reg.sr): bank 0 = eaddr[6] (station address), bank 1
             * (0x20) = mcast_lsb[6], bank 2 (0x40) = seq_write
             * { mcast_msb[2], pktgap, ctl }.  The driver writes the MAC in
             * bank 0 and the multicast hash in banks 1/2, and reads them
             * back, so the bank must be honoured on the read side too.
             */
            if (reg < 6) {
                switch (s->seeq_tx_cmd & 0x60) {
                case 0x00:
                    return s->seeq_station_addr[reg];
                case 0x20:
                    return s->seeq_mcast_lsb[reg];
                default:                    /* 0x40: bank 2 */
                    /*
                     * On READS bank 2 exposes a DIFFERENT view from the
                     * write one (seeq.h union): seq_read = coll_xmit[2],
                     * coll_total[2], fill6, flags -- the transmit collision
                     * counters.  Returning the write-side view (mcast_msb /
                     * pktgap / ctl) here fed the driver junk collision
                     * counts, which showed up as netstat -i "Coll" in the
                     * hundreds on a link with no real collisions.
                     */
                    return 0;               /* no collisions, no flags */
                }
            }
            return 0;
        }
    }

    /*
     * These core registers are 32-bit spaced; the PROM accesses them via
     * byte lane 3 (e.g. the EEPROM bit-bang at 0x1bf), so match the whole
     * 4-byte slot rather than the base address.
     */
    if (addr >= HPC1_MISCSR && addr < HPC1_MISCSR + 4) {
        return s->miscsr;
    }
    if (addr >= HPC1_AUX && addr < HPC1_AUX + 4) {
        return hpc1_aux_read(s);
    }
    if (addr >= 0x88 && addr < 0x98) {
        switch (addr & ~3ULL) {
        case 0x88:
            return s->scsi_dma_count & HPC1_SCSI_BC_MASK;
        case 0x8c:
            return s->scsi_cbp;
        case 0x90:
            return s->scsi_nbdp;
        default: /* 0x94 */
            return s->scsi_ctrl;
        }
    }

    if (addr + size <= sizeof(s->core_scratch)) {
        uint32_t v = 0;
        unsigned i;
        for (i = 0; i < size; i++) {
            v = (v << 8) | s->core_scratch[addr + i];
        }
        return v;
    }

    qemu_log_mask(LOG_UNIMP, "sgi-hpc1: read addr 0x%" HWADDR_PRIx
                  " size %u\n", addr, size);
    return 0;
}

static void sgi_hpc1_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    SGIHPC1State *s = opaque;
    uint8_t val8 = value & 0xff;

    if (addr >= HPC1_DUART_BASE && addr < HPC1_DUART_BASE + 0x40) {
        unsigned off = addr - HPC1_DUART_BASE;
        int d = off >> 4;
        int grp = (off >> 2) & 0x3;
        int c = (grp & 0x2) ? 0 : 1;
        bool is_data = grp & 0x1;
        if (d >= HPC1_NUM_DUARTS) {
            return;
        }
        if (is_data) {
            scc_data_write(s, d, c, val8);
        } else {
            scc_ctrl_write(s, d, c, val8);
        }
        return;
    }

    if (addr >= HPC1_RTC_BASE && addr < HPC1_RTC_BASE + 0x80) {
        rtc_reg_write(s, (addr - HPC1_RTC_BASE) >> 2, val8);
        return;
    }

    if (addr >= HPC1_INT2_BASE && addr < HPC1_INT2_BASE + 0x40) {
        int idx = ((addr - HPC1_INT2_BASE) >> 2) & 0xf;
        switch (idx) {
        case INT2_LIO0_MASK:
            s->lio_mask[0] = val8;
            int2_update(s);
            break;
        case INT2_LIO1_MASK:
            s->lio_mask[1] = val8;
            int2_update(s);
            break;
        case INT2_VME0_MASK:
            s->vme_mask[0] = val8;
            break;
        case INT2_VME1_MASK:
            s->vme_mask[1] = val8;
            break;
        case INT2_CONFIG:
            s->int2_config = val8 & 0x1f;
            break;
        case INT2_TIMER_ACK:
            if (val8 & 0x1) {
                s->timer_pending[0] = false;
                qemu_irq_lower(s->timer_irq[0]);
            }
            if (val8 & 0x2) {
                s->timer_pending[1] = false;
                qemu_irq_lower(s->timer_irq[1]);
            }
            break;
        default:
            if (idx >= INT2_PIT_BASE && idx <= INT2_PIT_BASE + 3) {
                hpc1_pit_write(s, idx - INT2_PIT_BASE, val8);
            }
            break;
        }
        return;
    }

    if (addr >= HPC1_SCSI_BASE && addr < HPC1_SCSI_BASE + 8) {
        unsigned off = addr - HPC1_SCSI_BASE;
        if (!s->scsi) {
            return;
        }
        if (off & 4) {
            wd33c93_data_write(s->scsi, val8);
        } else {
            wd33c93_addr_write(s->scsi, val8);
        }
        return;
    }

    if (addr >= HPC1_ENET_XCOUNT && addr < 0x60) {
        /*
         * The enet DMA register file is fully writable: the driver
         * programs a pointer/status register and reads it back (e.g.
         * CXBDP after arming RX).  Dropping any write makes the read
         * return 0 and the driver dereferences a NULL descriptor ->
         * guest Data Bus Error, so every register stores its value.
         */
        switch (addr & ~3ULL) {
        case HPC1_ENET_XCOUNT:   s->enet_xcount = value; break;
        case HPC1_ENET_CXBP:     s->enet_cxbp = value; break;
        case HPC1_ENET_NXBDP:
            s->enet_nxbdp = value;
            break;
        case HPC1_ENET_XBC:      s->enet_xbc = value; break;
        case HPC1_ENET_CXBDP:    s->enet_cxbdp = value; break;
        case HPC1_ENET_CPFXBDP:  s->enet_cpfxbdp = value; break;
        case HPC1_ENET_PPFXBDP:  s->enet_ppfxbdp = value; break;
        case HPC1_ENET_INTDELAY: s->enet_intdelay = value; break;
        case HPC1_ENET_TRSTAT:
            /* Writing HPC_STTRDMA starts the transmit channel. */
            s->enet_trstat = value;
            if (value & HPC1_ENET_STTRDMA) {
                sgi_hpc1_enet_tx(s);
            }
            break;
        case HPC1_ENET_RCVSTAT:
            /* HPC_STRCVDMA arms the receive channel. */
            s->enet_rcvstat = value;
            break;
        case HPC1_ENET_CTL:
            /* INTPEND is write-1-to-clear; ERST resets the channel (the
             * driver asserts it then clears it during init). */
            if (value & HPC1_ENET_CTL_ERST) {
                s->enet_trstat = 0;
                s->enet_rcvstat = 0;
                s->enet_ctl = HPC1_ENET_CTL_MODNORM;
            }
            if (value & HPC1_ENET_CTL_INTPEND) {
                s->enet_ctl &= ~HPC1_ENET_CTL_INTPEND;
                s->lio_status[0] &= ~LIO0_ETHERNET;
                scc_update_irq(s);
            }
            break;
        case HPC1_ENET_RBC:      s->enet_rbc = value; break;
        case HPC1_ENET_CRBP:     s->enet_crbp = value; break;
        case HPC1_ENET_NRBDP:
            s->enet_nrbdp = value;
            break;
        case HPC1_ENET_CRBDP:
            s->enet_crbdp = value;
            break;
        default:                 break;
        }
        return;
    }

    if (addr >= HPC1_ENET_BASE && addr < HPC1_ENET_BASE + 0x20) {
        int reg = ((addr - HPC1_ENET_BASE) >> 2) & 7;
        switch (reg) {
        case 6:
            s->seeq_rx_cmd = val8;
            break;
        case 7:
            s->seeq_tx_cmd = val8;   /* bits [6:5] select the register bank */
            break;
        default:
            if (reg < 6) {
                /*
                 * Banked writes (SEEQ_TXC_BANK_MASK): bank 0 holds the
                 * station address; banks 0x20/0x40 the multicast filter and
                 * control.  Without the bank check the driver's hash writes
                 * silently overwrite the MAC, which drops every unicast
                 * packet (the HPC3 lesson, blog_ethernet_bank_selection.md).
                 */
                switch (s->seeq_tx_cmd & 0x60) {
                case 0x00:                      /* bank 0: station address */
                    s->seeq_station_addr[reg] = val8;
                    break;
                case 0x20:                      /* bank 1: mcast lsb */
                    s->seeq_mcast_lsb[reg] = val8;
                    break;
                default:                        /* 0x40: bank 2 */
                    switch (reg) {
                    case 0: s->seeq_mcast_msb[0] = val8; break;
                    case 1: s->seeq_mcast_msb[1] = val8; break;
                    case 2: s->seeq_pktgap = val8; break;
                    case 3: s->seeq_seeqctl = val8; break;
                    default: break;
                    }
                    break;
                }
            }
            break;
        }
        return;
    }

    if (addr >= HPC1_MISCSR && addr < HPC1_MISCSR + 4) {
        s->miscsr = value;
        return;
    }
    if (addr >= HPC1_AUX && addr < HPC1_AUX + 4) {
        hpc1_aux_write(s, val8);
        return;
    }
    if (addr >= 0x88 && addr < 0x98) {
        switch (addr & ~3ULL) {
        case 0x88:
            s->scsi_bc = value & HPC1_SCSI_BC_MASK;
            s->scsi_dma_count = s->scsi_bc;
            break;
        case 0x8c:
            s->scsi_cbp = value;
            break;
        case 0x90:
            s->scsi_nbdp = value & HPC1_SCSI_BUFADDR;
            hpc1_scsi_chain(s);
            break;
        default: /* 0x94 */
            hpc1_scsi_ctrl_write(s, value);
            break;
        }
        return;
    }
    if (addr >= 0x188 && addr < 0x18c) {
        s->dsp_bc = value;
        return;
    }

    switch (addr) {
    default:
        if (addr + size <= sizeof(s->core_scratch)) {
            unsigned i;
            for (i = 0; i < size; i++) {
                s->core_scratch[addr + i] =
                    (value >> (8 * (size - 1 - i))) & 0xff;
            }
        } else {
            qemu_log_mask(LOG_UNIMP, "sgi-hpc1: write addr 0x%" HWADDR_PRIx
                          " = 0x%" PRIx64 " size %u\n", addr, value, size);
        }
        break;
    }
}

/* =====================================================================
 * HPC1 ethernet — SEEQ 8003 EDLC with HPC1-provided DMA.
 *
 * Layout, descriptor format and bit placement come from SGI's own driver
 * header references/stand/arcs/include/net/seeq.h (the `#if IP20` EHIO
 * branch) and the driver references/stand/arcs/lib/libsk/net/if_ec2.c.
 * HPC1 differs from HPC3 in ways this code depends on:
 *   - RX status is in the HIGH byte (RCVSTAT_SHIFT 8);
 *   - "dma started" and TX status bits are in the HIGH half;
 *   - no piocfg/dmacfg indirection (if_ec2.c: ENET_READ(reg) is a plain
 *     dereference on IP20, vs the piocfg indirection on IP22/26/28);
 *   - descriptors are 4 words (16 bytes), not HPC3's 3-word form.
 * ===================================================================== */

#define HPC1_DMA_ADDR(x)        ((x) & 0x1fffffffu)
#define HPC1_ENET_MAXPKT        1536
/*
 * HPC_RSPACE, from the IP20 KERNEL header (infra/kernbuild/kern/bsd/misc/seeq.h),
 * NOT the ARCS one (which says 3):
 *     #define HPC_RSPACE 8
 *     #define MAX_RPKT   (MAX_TPKT + HPC_RSPACE + 64)   == 1586
 * The driver arms r_rbcnt = MAX_RPKT (1586) and derives
 *     rlen = MAX_RPKT - r_rbcnt - HPC_RSPACE
 * then reads the Seeq status byte from MEMORY at eh + rlen.  The BUILT IP20
 * kernel uses HPC_RSPACE = 3 (verified: with 8 the driver read a non-GOOD byte,
 * took the SN_ERROR path in if_ecintr, ether_input++'d again and dropped every
 * frame -> Ierrs = 2*Ipkts and arp stayed incomplete; with 3, Ierrs = 0 and ARP
 * resolves).  With RSPACE = 3
 * the model under-decremented by 5, so the driver read the status 5 bytes
 * early (frame data instead of SEQ_RS_GOOD) and dropped every frame.
 */
#define HPC1_ENET_RSPACE        3u
#define HPC1_ENET_SEQ_RXS_GOOD  0x20  /* SEQ_RS_GOOD */
#define HPC1_ENET_SEQ_RXS_END   0x10  /* SEQ_RS_END */

static void sgi_hpc1_enet_raise_irq(SGIHPC1State *s)
{
    s->enet_ctl |= HPC1_ENET_CTL_INTPEND;
    s->lio_status[0] |= LIO0_ETHERNET;
    scc_update_irq(s);
}

/* TX: walk the 4-word xd_desc chain from nxbdp, assembling and sending. */
static void sgi_hpc1_enet_tx(SGIHPC1State *s)
{
    uint8_t packet[HPC1_ENET_MAXPKT];
    int plen = 0;
    uint32_t desc = HPC1_DMA_ADDR(s->enet_nxbdp);

    if (!desc) {
        return;
    }
    s->enet_cpfxbdp = s->enet_nxbdp;

    while (desc) {
        uint32_t w0 = address_space_ldl_be(&address_space_memory, desc,
                                           MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t w1 = address_space_ldl_be(&address_space_memory, desc + 4,
                                           MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t w2 = address_space_ldl_be(&address_space_memory, desc + 8,
                                           MEMTXATTRS_UNSPECIFIED, NULL);
        bool eoxp = (w0 >> 31) & 1;        /* last descriptor in the chain */
        bool eox  = (w1 >> 31) & 1;        /* last buffer of this packet */
        unsigned len = w0 & 0x1fffu;       /* x_xbcnt */
        uint32_t buf = HPC1_DMA_ADDR(w1 & 0x0fffffffu);  /* x_xbufptr */

        if (len && plen + (int)len <= (int)sizeof(packet)) {
            address_space_read(&address_space_memory, buf,
                               MEMTXATTRS_UNSPECIFIED, packet + plen, len);
            plen += len;
        }
        if (eox) {
            if (plen >= 60) {
                qemu_send_packet(qemu_get_queue(s->nic), packet, plen);
            }
            plen = 0;
        }
        s->enet_nxbdp = w2;
        if (eoxp) {
            break;
        }
        desc = HPC1_DMA_ADDR(w2);
    }

    /* Channel done: clear "dma started" and report success (driver polls). */
    s->enet_trstat = HPC1_ENET_SEQ_XS_OLD | HPC1_ENET_SEQ_XS_SUCCESS;
    sgi_hpc1_enet_raise_irq(s);
}

static bool sgi_hpc1_enet_can_receive(NetClientState *nc)
{
    SGIHPC1State *s = qemu_get_nic_opaque(nc);

    return (s->enet_rcvstat & HPC1_ENET_STRCVDMA) != 0;
}

/* RX: QEMU delivered a frame — write it into the crbdp/nrbdp ring. */
static ssize_t sgi_hpc1_enet_receive(NetClientState *nc,
                                     const uint8_t *buf, size_t size)
{
    SGIHPC1State *s = qemu_get_nic_opaque(nc);
    uint32_t desc, w0, w1, w2, bufaddr, space, used, newbc;
    uint8_t st = HPC1_ENET_SEQ_RXS_GOOD | HPC1_ENET_SEQ_RXS_END;

    if (!(s->enet_rcvstat & HPC1_ENET_STRCVDMA) || !s->enet_nrbdp) {
        return -1;
    }
    /*
     * Deliver into the descriptor the driver ARMED for the next fill
     * (NRBDP), exactly as the HPC3 model delivers into enet_rx_nbdp.
     * CRBDP is the driver's own "current" marker, not the fill target.
     */
    desc = HPC1_DMA_ADDR(s->enet_nrbdp);
    w0 = address_space_ldl_be(&address_space_memory, desc,
                              MEMTXATTRS_UNSPECIFIED, NULL);
    w1 = address_space_ldl_be(&address_space_memory, desc + 4,
                              MEMTXATTRS_UNSPECIFIED, NULL);
    w2 = address_space_ldl_be(&address_space_memory, desc + 8,
                              MEMTXATTRS_UNSPECIFIED, NULL);
    space = w0 & 0x1fffu;                        /* r_rbcnt: room left */
    bufaddr = HPC1_DMA_ADDR(w1 & 0x0fffffffu);   /* r_rbufptr */
    /*
     * The ring is armed with r_rbcnt = MAX_RPKT (if_ec2.c:316) and the driver
     * derives rlen = MAX_RPKT - r_rbcnt - HPC_RSPACE (:716/:749), so the
     * hardware decrements the count by the bytes it consumes in the buffer:
     * the 2-byte offset + the frame + the trailing status = size + RSPACE.
     */
    used = size + HPC1_ENET_RSPACE;

    /*
     * Ownership: the driver arms a buffer with r_own (bit 31) set and the
     * hardware clears it when it fills the buffer.  Deliver only into an
     * armed descriptor.  If the driver has not armed this one yet the ring
     * is full: report it and STOP here -- do not write the frame into an
     * unowned buffer and do not advance past it (that is how the model was
     * clobbering descriptors with buf=0).
     */
    /*
     * Ring-end sentinel: the driver terminates the ring with an extra
     * descriptor whose r_rbufptr is the magic 0xEBAD and whose r_own is left
     * SET (if_ec2.c:344-345), so it looks armed but must never be filled.
     */
    if ((w1 & 0x0fffffffu) == 0xEBADu) {
        s->enet_ctl |= HPC1_ENET_CTL_RBO;
        s->enet_rcvstat &= ~HPC1_ENET_STRCVDMA;
        sgi_hpc1_enet_raise_irq(s);
        return size;
    }

    if (!(w0 & 0x80000000u)) {
        s->enet_ctl |= HPC1_ENET_CTL_RBO;
        s->enet_rcvstat &= ~HPC1_ENET_STRCVDMA;
        sgi_hpc1_enet_raise_irq(s);
        return size;
    }

    if (used > space) {
        s->enet_ctl |= HPC1_ENET_CTL_RBO;
        s->enet_rcvstat &= ~HPC1_ENET_STRCVDMA;
        sgi_hpc1_enet_raise_irq(s);
        return size;
    }

    /* Frame at buf+2 (so the IP header at frame+14 stays 32-bit aligned);
     * the Seeq status byte follows the frame. */
    address_space_write(&address_space_memory, bufaddr + 2,
                        MEMTXATTRS_UNSPECIFIED, buf, size);
    address_space_write(&address_space_memory, bufaddr + 2 + size,
                        MEMTXATTRS_UNSPECIFIED, &st, 1);

    /*
     * r_rown (bit 31) is software ownership of the buffer: the driver spins
     * `while (!rd_chain->r_rown)` and processes a descriptor while r_rown is
     * 0, setting it back to 1 after consuming (if_ec2.c:706/790).  So CLEAR
     * r_rown to publish the frame ("data ready"), exactly as the HPC3 model
     * does, and store the decremented byte count.
     */
    newbc = (w0 & ~0x1fffu & ~0x80000000u) | ((space - used) & 0x1fffu);
    address_space_stl_be(&address_space_memory, desc, newbc,
                         MEMTXATTRS_UNSPECIFIED, NULL);

    /*
     * Advance the fill pointer to the descriptor's own next pointer, exactly
     * as the HPC3 model does (enet_rx_nbdp = nbdp).  The Seeq status goes in
     * the HIGH byte (shift 8).
     */
    /* Advance the fill pointer only.  Do NOT touch crbdp: the kernel driver
     * arms it to ei_rtail ("for ec_watchdog") and its watchdog compares
     * crbdp against ei_rtail to detect a stuck receiver, so overwriting it
     * with the filled descriptor breaks that check. */
    s->enet_nrbdp = w2;              /* next to fill */
    s->seeq_rx_status = st;
    /*
     * HPC_STRCVDMA MUST stay set across a receive.  The kernel driver's
     * ec_watchdog() polls RX and does, on every tick:
     *
     *     if (!(hio->rcvstat & HPC_STRCVDMA)) {  // "recovered D"
     *         ec_init(...);                      // full re-init
     *     }
     *     if_ecintr(ei);                         // walk ei_ract / r_own
     *
     * so clearing STRCVDMA on a frame makes the driver re-initialise the
     * interface every watchdog tick -- which is exactly the observed
     * "NRBDP re-armed to the same head, frame never consumed".  The frame
     * status is read from the buffer (eh + rlen), not from rcvstat, so the
     * high byte is informational only.
     */
    s->enet_rcvstat = (s->enet_rcvstat & HPC1_ENET_STRCVDMA) |
                      ((uint32_t)st << HPC1_ENET_RCVSTAT_SHIFT);
    sgi_hpc1_enet_raise_irq(s);
    return size;
}

static void sgi_hpc1_enet_reset(SGIHPC1State *s)
{
    s->enet_xcount = s->enet_cxbp = s->enet_nxbdp = s->enet_xbc = 0;
    s->enet_cxbdp = s->enet_cpfxbdp = s->enet_ppfxbdp = 0;
    s->enet_intdelay = s->enet_trstat = s->enet_rcvstat = 0;
    s->enet_rbc = s->enet_crbp = s->enet_nrbdp = s->enet_crbdp = 0;
    s->enet_ctl = HPC1_ENET_CTL_MODNORM;
    s->seeq_rx_cmd = s->seeq_tx_cmd = 0;
    memset(s->seeq_mcast_lsb, 0, sizeof(s->seeq_mcast_lsb));
    memset(s->seeq_mcast_msb, 0, sizeof(s->seeq_mcast_msb));
    s->seeq_pktgap = s->seeq_seeqctl = 0;
    s->seeq_rx_status = s->seeq_tx_status = 0;
    memset(s->seeq_station_addr, 0, sizeof(s->seeq_station_addr));
    s->lio_status[0] &= ~LIO0_ETHERNET;
}

static const NetClientInfo sgi_hpc1_enet_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_hpc1_enet_can_receive,
    .receive = sgi_hpc1_enet_receive,
};

static const MemoryRegionOps sgi_hpc1_ops = {
    .read = sgi_hpc1_read,
    .write = sgi_hpc1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* Chardev receive path                                                */
/* ------------------------------------------------------------------ */

static int sgi_hpc1_serial_can_receive(void *opaque)
{
    SGIHPC1State *s = opaque;
    return HPC1_RX_FIFO_SIZE - s->uart[1][1].rx_count;
}

static void sgi_hpc1_serial_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIHPC1State *s = opaque;
    int i;
    /*
     * The IP20 console input is DUART1 channel B, and the PROM/kernel drains
     * that channel. Feed ONLY it: feeding the other channels left undrained
     * bytes in their RX FIFOs, so their RR0 "RX available" bit stayed set and
     * the PROM's input-drain loop (poll RR0 bit0, never read the data port)
     * spun forever after `fx` exited.
     */
    for (i = 0; i < size; i++) {
        SGIHPC1Uart *u = &s->uart[1][1];
        if (u->rx_count >= HPC1_RX_FIFO_SIZE) {
            continue;
        }
        u->rx_fifo[u->rx_head] = buf[i];
        u->rx_head = (u->rx_head + 1) % HPC1_RX_FIFO_SIZE;
        u->rx_count++;
        if (u->wr[1] & 0x18) {
            u->rr3 |= SCC_RX_IP;
        }
    }
    scc_update_irq(s);
}

static void sgi_hpc1_serial_event(void *opaque, QEMUChrEvent event)
{
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

static void sgi_hpc1_reset(DeviceState *dev)
{
    SGIHPC1State *s = SGI_HPC1(dev);
    int d, c, i;

    memset(s->core_scratch, 0, sizeof(s->core_scratch));
    rtc_reset(s);
    s->miscsr = 0;
    s->scsi_ctrl = 0;
    s->scsi_bc = 0;
    s->scsi_cbp = 0;
    s->scsi_nbdp = 0;
    s->scsi_dma_count = 0;
    s->scsi_dma_active = false;
    s->scsi_dma_to_device = false;
    s->scsi_drq = false;
    s->dsp_bc = 0;
    s->seeq_rx_cmd = 0;
    s->seeq_tx_cmd = 0;
    s->seeq_rx_status = 0;
    s->seeq_tx_status = 0;
    s->aux = 0;
    s->nv_cs = 0;
    s->nv_clk = 0;
    s->nv_di = 0;
    s->nv_do = 1;
    s->nv_tick = 0;
    s->nv_opcode = 0;
    s->nv_addr = 0;
    s->nv_data = 0;
    s->nv_writable = 0;
    for (i = 0; i < 6; i++) {
        s->seeq_station_addr[i] = 0;
        s->seeq_mcast_lsb[i] = 0;
    }
    sgi_hpc1_enet_reset(s);
    s->lio_status[0] = s->lio_status[1] = 0;
    s->lio_mask[0] = s->lio_mask[1] = 0;
    s->vme_status = 0;
    s->vme_mask[0] = s->vme_mask[1] = 0;
    s->int2_config = 0;
    for (i = 0; i < 3; i++) {
        s->pit_control[i] = 0;
        s->pit_count[i] = 0;
        s->pit_low[i] = 0;
        s->pit_rw_state[i] = 0;
        s->pit_read_state[i] = 0;
        s->pit_load_ns[i] = 0;
        s->pit_programmed[i] = false;
    }
    s->timer_pending[0] = s->timer_pending[1] = false;

    for (d = 0; d < HPC1_NUM_DUARTS; d++) {
        for (c = 0; c < HPC1_DUART_CH; c++) {
            SGIHPC1Uart *u = &s->uart[d][c];
            memset(u, 0, sizeof(*u));
        }
    }
}

static void sgi_hpc1_realize(DeviceState *dev, Error **errp)
{
    SGIHPC1State *s = SGI_HPC1(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sgi_hpc1_ops, s,
                          "sgi-hpc1", HPC1_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    qdev_init_gpio_out_named(dev, s->cpu_irq, "cpu-irq", 2);
    qdev_init_gpio_out_named(dev, s->timer_irq, "timer-irq", 2);
    qdev_init_gpio_in_named(dev, hpc1_scsi_irq, "scsi-irq", 1);
    qdev_init_gpio_in_named(dev, hpc1_scsi_drq, "scsi-drq", 1);

    /* IP20 keyboard HLE on DUART0 channel A. Register AND activate, or the
     * events are never routed here (another handler grabs them). */
    s->kbd_ih = qemu_input_handler_register(dev, &sgi_hpc1_kbd_handler);
    qemu_input_handler_activate(s->kbd_ih);

    /* IP20 mouse HLE on DUART0 channel B (no handshake; present iff kbd is). */
    s->mouse_buttons = 0x07;            /* Mouse Systems: all buttons up */
    s->mouse_dx = 0;
    s->mouse_dy = 0;
    s->mouse_ih = qemu_input_handler_register(dev, &sgi_hpc1_mouse_handler);
    qemu_input_handler_activate(s->mouse_ih);

    /* WD33C93 SCSI controller */
    s->scsi = WD33C93(qdev_new(TYPE_WD33C93));
    /*
     * Enable the WD33C93 unexpected-phase (UNEX) multi-pass resume, matching
     * the HPC3/Indy path. The IP20 driver caps a DMA chain at 64 descriptors
     * and programs TC to the chain total, which is short of the CDB length by
     * the tail: e.g. a 262144-byte READ whose buffer starts 1504 bytes into a
     * page yields [bc=2592] + 63*[bc=4096] = 260640, leaving 1504 bytes. The
     * driver relies on the UNEX interrupt to transfer that tail in a second
     * pass. The earlier "no-unex" conclusion rested on a stale run that
     * predated the HPC1 DMA drain/SCC/DUART fixes.
     * (See progress_notes/ip20/LEG22-mkfs-finalization-blocker.md.)
     */
    object_property_set_bool(OBJECT(s->scsi), "no-unex", false, &error_fatal);
    qdev_realize(DEVICE(s->scsi), NULL, &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(s->scsi), "irq", 0,
                                qdev_get_gpio_in_named(dev, "scsi-irq", 0));
    qdev_connect_gpio_out_named(DEVICE(s->scsi), "drq", 0,
                                qdev_get_gpio_in_named(dev, "scsi-drq", 0));

    /* 93C56 NVRAM (128 x 16-bit words) */
    hpc1_nvram_init_defaults(s);
    hpc1_nvram_load(s);

    /* SEEQ 8003 ethernet NIC (DMA through HPC1) */
    qemu_macaddr_default_if_unset(&s->enet_conf.macaddr);
    s->nic = qemu_new_nic(&sgi_hpc1_enet_net_info, &s->enet_conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic),
                             s->enet_conf.macaddr.a);

    /* PIT interrupt timers (timer0 -> IP4, timer1 -> IP5) */
    s->pit_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, hpc1_pit_timer0_cb, s);
    s->pit_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, hpc1_pit_timer1_cb, s);

    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_set_handlers(&s->serial,
                                 sgi_hpc1_serial_can_receive,
                                 sgi_hpc1_serial_receive,
                                 sgi_hpc1_serial_event,
                                 NULL, s, NULL, true);
    }
}

static const Property sgi_hpc1_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIHPC1State, serial),
    DEFINE_PROP_STRING("nvram", SGIHPC1State, nvram_filename),
    DEFINE_NIC_PROPERTIES(SGIHPC1State, enet_conf),
};

static const VMStateDescription vmstate_sgihpc1_uart = {
    .name = "sgi-hpc1-uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(reg_ptr, SGIHPC1Uart),
        VMSTATE_UINT8_ARRAY(wr, SGIHPC1Uart, 16),
        VMSTATE_UINT8(wr7p, SGIHPC1Uart),
        VMSTATE_UINT8(rr3, SGIHPC1Uart),
        VMSTATE_UINT8_ARRAY(rx_fifo, SGIHPC1Uart, HPC1_RX_FIFO_SIZE),
        VMSTATE_UINT8(rx_head, SGIHPC1Uart),
        VMSTATE_UINT8(rx_tail, SGIHPC1Uart),
        VMSTATE_UINT8(rx_count, SGIHPC1Uart),
        VMSTATE_END_OF_LIST()
    }
};

/*
 * The HPC1 was previously not migratable at all, so a qcow2/internal snapshot
 * or -loadvm silently reset the whole controller (PIT unprogrammed -> kernel
 * clock dead; SCSI DMA state lost; INT2 masks/status lost).  Restore the
 * device state and re-arm the PIT timers (QEMUTimer objects cannot migrate).
 */
static int sgi_hpc1_post_load(void *opaque, int version_id)
{
    SGIHPC1State *s = opaque;
    int ch;

    for (ch = 0; ch < 2; ch++) {
        if (s->pit_programmed[ch]) {
            timer_mod(s->pit_timer[ch],
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      hpc1_pit_period_ns(s, ch));
        }
    }
    int2_update(s);
    return 0;
}

static const VMStateDescription vmstate_sgi_hpc1 = {
    .name = "sgi-hpc1",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sgi_hpc1_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(miscsr, SGIHPC1State),
        VMSTATE_UINT32(scsi_ctrl, SGIHPC1State),
        VMSTATE_UINT32(scsi_bc, SGIHPC1State),
        VMSTATE_UINT32(scsi_cbp, SGIHPC1State),
        VMSTATE_UINT32(scsi_nbdp, SGIHPC1State),
        VMSTATE_UINT32(scsi_dma_count, SGIHPC1State),
        VMSTATE_BOOL(scsi_dma_active, SGIHPC1State),
        VMSTATE_BOOL(scsi_dma_to_device, SGIHPC1State),
        VMSTATE_BOOL(scsi_drq, SGIHPC1State),
        VMSTATE_UINT32(dsp_bc, SGIHPC1State),
        VMSTATE_BUFFER(core_scratch, SGIHPC1State),
        VMSTATE_BUFFER(rtc, SGIHPC1State),
        VMSTATE_INT64(rtc_host_base_ms, SGIHPC1State),
        VMSTATE_INT64(rtc_guest_base_ms, SGIHPC1State),
        VMSTATE_UINT8_ARRAY(seeq_station_addr, SGIHPC1State, 6),
        VMSTATE_UINT8_ARRAY(seeq_mcast_lsb, SGIHPC1State, 6),
        VMSTATE_UINT8_ARRAY(seeq_mcast_msb, SGIHPC1State, 2),
        VMSTATE_UINT8(seeq_pktgap, SGIHPC1State),
        VMSTATE_UINT8(seeq_seeqctl, SGIHPC1State),
        VMSTATE_UINT8(seeq_rx_cmd, SGIHPC1State),
        VMSTATE_UINT8(seeq_tx_cmd, SGIHPC1State),
        VMSTATE_UINT8(seeq_rx_status, SGIHPC1State),
        VMSTATE_UINT8(seeq_tx_status, SGIHPC1State),
        /* HPC1 ethernet DMA engine */
        VMSTATE_UINT32(enet_nxbdp, SGIHPC1State),
        VMSTATE_UINT32(enet_xbc, SGIHPC1State),
        VMSTATE_UINT32(enet_crbp, SGIHPC1State),
        VMSTATE_UINT32(enet_nrbdp, SGIHPC1State),
        VMSTATE_UINT32(enet_crbdp, SGIHPC1State),
        VMSTATE_UINT32(enet_cpfxbdp, SGIHPC1State),
        VMSTATE_UINT32(enet_ppfxbdp, SGIHPC1State),
        VMSTATE_UINT32(enet_intdelay, SGIHPC1State),
        VMSTATE_UINT32(enet_trstat, SGIHPC1State),
        VMSTATE_UINT32(enet_rcvstat, SGIHPC1State),
        VMSTATE_UINT32(enet_ctl, SGIHPC1State),
        VMSTATE_UINT8(aux, SGIHPC1State),
        VMSTATE_UINT16_ARRAY(nvram, SGIHPC1State, 128),
        VMSTATE_UINT8(nv_cs, SGIHPC1State),
        VMSTATE_UINT8(nv_clk, SGIHPC1State),
        VMSTATE_UINT8(nv_di, SGIHPC1State),
        VMSTATE_UINT8(nv_do, SGIHPC1State),
        VMSTATE_UINT8(nv_tick, SGIHPC1State),
        VMSTATE_UINT8(nv_opcode, SGIHPC1State),
        VMSTATE_UINT8(nv_addr, SGIHPC1State),
        VMSTATE_UINT8(nv_writable, SGIHPC1State),
        VMSTATE_UINT16(nv_data, SGIHPC1State),
        VMSTATE_UINT8_ARRAY(lio_status, SGIHPC1State, 2),
        VMSTATE_UINT8_ARRAY(lio_mask, SGIHPC1State, 2),
        VMSTATE_UINT8(vme_status, SGIHPC1State),
        VMSTATE_UINT8_ARRAY(vme_mask, SGIHPC1State, 2),
        VMSTATE_UINT8(int2_config, SGIHPC1State),
        VMSTATE_UINT8_ARRAY(pit_control, SGIHPC1State, 3),
        VMSTATE_UINT16_ARRAY(pit_count, SGIHPC1State, 3),
        VMSTATE_UINT16_ARRAY(pit_low, SGIHPC1State, 3),
        VMSTATE_UINT8_ARRAY(pit_rw_state, SGIHPC1State, 3),
        VMSTATE_UINT8_ARRAY(pit_read_state, SGIHPC1State, 3),
        VMSTATE_INT64_ARRAY(pit_load_ns, SGIHPC1State, 3),
        VMSTATE_BOOL_ARRAY(pit_programmed, SGIHPC1State, 3),
        VMSTATE_BOOL_ARRAY(timer_pending, SGIHPC1State, 2),
        VMSTATE_UINT8(mouse_buttons, SGIHPC1State),
        VMSTATE_STRUCT_2DARRAY(uart, SGIHPC1State,
                               HPC1_NUM_DUARTS, HPC1_DUART_CH, 0,
                               vmstate_sgihpc1_uart, SGIHPC1Uart),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_hpc1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_hpc1_realize;
    dc->vmsd = &vmstate_sgi_hpc1;
    device_class_set_props(dc, sgi_hpc1_properties);
    device_class_set_legacy_reset(dc, sgi_hpc1_reset);
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
