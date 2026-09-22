/*
 * SGI Personal Iris (IP6) keyboard HLE -- DRAFT, NOT BUILT, NOT GATED.
 *
 * Written offline while the ide.IP6 gate ran, per the Orchestrator.  It has
 * never been compiled or run.  It is a sketch with the protocol from the
 * PROM's own driver (0xbfc148c0), not a finished device.
 *
 * SEAM -- now settled by the Orchestrator (#2960), correcting an earlier
 * framing of mine.  This is NOT "moving" the console: MAME's ip6.cpp has the
 * keyboard on duart[0] A, the mouse on duart[0] B, and the serial ports on
 * duart[1].  The console chardev on duart[0]-A is a STAND-IN this machine
 * invented (sgi_ip6.c:1548 does serial_hd(i*2+ch), so -serial stdio lands on
 * duart[0]-A; serial_hd(1) is discarded outright).  The task is to model the
 * ports as the hardware has them: keyboard peer on 0-A, mouse peer on 0-B
 * (unchanged), serial_hd(0) on duart[1]-A, serial_hd(1) on duart[1]-B.
 *
 * STILL TO BE MEASURED, not assumed: with the keyboard on 0-A and nothing in
 * the console env, which port does the PROM fall back to?  That measurement
 * decides whether the harnesses change at all (the remap keeps serial_hd(0)
 * as the console, so the clean outcome is that no script command line
 * changes -- but confirm before claiming it).
 *
 * Protocol, from the PROM's driver:
 *   the guest WRITES a command byte, then READS a reply:
 *     0x10 -> layout id, must be < 16 (12+ => "no translation table", US
 *             fallback); 0xcc means unknown.  Reply is one byte.
 *     0x09/0x11/0x21/0x41, 0x01, 0x82 -> init/LED handshake, each expects a
 *             status byte back (the driver ORs in bit 1).
 *   after init, the link carries KEY EVENTS, one byte each:
 *     bit 7 = 1 for key-up, 0 for key-down; bits 6:0 = scancode.
 *   the guest polls with a 99 x ~1ms timeout (0xbfc144c0); missing a reply
 *   produces "Error-- keyboard not responding".
 *
 * The scancode map extracted from the PROM's table (US):
 *   6 Esc, 7 '1'/'!', 8 Tab, 9 q/Q, 10 a/A, 11 s/S, 16 e/E, 50 Enter,
 *   82 Space, arrows as CSI escapes.  Full table: translator 0xbfc22940,
 *   layout table 0xa03c1df0, ROM source at 0xbfc3c8bc.
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_ip6_input.h"
#include "qemu/error-report.h"

/*
 * A keyboard peer sits on DUART0 channel A exactly as the mouse peer sits on
 * channel B: the DUART's txd callback delivers guest commands to us, and we
 * queue reply bytes that the DUART drains when the guest reads RHR.
 *
 * State is tiny: the last command awaiting a reply, and a key-event byte
 * pending delivery.  Real hardware is a "real 8031 with its own ROM"
 * (MAME kbd.h) -- see the design note; this HLE reproduces what the guest
 * needs to see, not the microcontroller internals.
 */
typedef enum {
    KB_IDLE = 0,
    KB_AWAIT_LAYOUT,   /* 0x10 sent, reply is the layout id            */
    KB_AWAIT_STATUS,   /* handshake byte sent, reply is a status byte  */
} SgiIp6KbState;

/* A plausible IP6-class layout id: 1 = the US table the PROM already has. */
#define KB_LAYOUT_US 1

/* Handshake commands seen from the PROM's driver, in order. */
static bool kb_is_handshake(uint8_t c)
{
    return c == 0x09 || c == 0x11 || c == 0x21 || c == 0x41 ||
           c == 0x01 || c == 0x82;
}

/*
 * Called with each byte the guest writes to the keyboard port.  Returns the
 * byte to hand back, or -1 if the guest should read nothing yet.
 *
 * NOTE: the real handshake is bidirectional on one wire with framing; over a
 * byte-oriented chardev we can only answer command-with-one-reply.  If the
 * guest ever pipelines commands this needs a state machine with an input
 * queue rather than a single pending state (flagged, not solved).
 */
int sgi_ip6_keyboard_cmd(SgiIp6InputState *s, uint8_t cmd)
{
    switch (cmd) {
    case 0x10:
        s->kb_state = KB_AWAIT_LAYOUT;
        return KB_LAYOUT_US;
    default:
        if (kb_is_handshake(cmd)) {
            s->kb_state = KB_AWAIT_STATUS;
            /* status: bit 1 set (the driver ORs it in anyway), rest clear. */
            return 0x02;
        }
        break;
    }
    /* Unknown command: no reply.  Do NOT invent one -- an unmeasured value
     * here is exactly the fabricated-status failure the doctrine forbids. */
    return -1;
}

/*
 * Translate a QEMU key event into a PROM scancode + up/down, and queue it as
 * one byte.  Only the subset the PROM's US table names is mapped; an unmapped
 * key is DROPPED rather than guessed (again: no fabricated scancodes).
 */
void sgi_ip6_keyboard_key(SgiIp6InputState *s, int qcode, bool down)
{
    int scan = sgi_ip6_scancode_for(qcode);

    if (scan < 0) {
        return;
    }
    sgi_ip6_input_queue(s, (uint8_t)((down ? 0x00 : 0x80) | (scan & 0x7f)));
}

/*
 * TODO(seam): the port remap in sgi_ip6.c:1542-1562 -- attach a keyboard peer
 * on duart[0]-A and move serial_hd(0)/serial_hd(1) to duart[1]-A/B.  A
 * machine-level change, gated on all four (the fourth is the negative the
 * Orchestrator added and is what makes the gate meaningful):
 *   (1) the keyboard error disappears from the PROM diagnostics;
 *   (2) the Command Monitor accepts input from here;
 *   (3) the mouse gate still passes (it uses duart[0]-B);
 *   (4) an UNPLUGGED keyboard (this peer detached) must STILL produce
 *       "keyboard not responding" -- if the error is absent with the HLE
 *       detached, then (1)-(3) were passed by something that is not the
 *       keyboard and the fix is unproven.
 *
 * TODO(parity): the link is 600-8-O-1 odd.  Not modelled, and not
 * representable at this seam; no firmware poll site tests PERR, so reporting
 * it clear is provably safe here.  Same open item as the mouse leg.
 */
