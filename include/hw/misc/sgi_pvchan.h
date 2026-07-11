/*
 * sgi-pvchan — paravirtual host<->guest communication channel for virtuix.
 *
 * A shared-ring doorbell device (simplified sibling of sgi_glaccel's ring)
 * designed to replace serial scraping + gdbstub-halt + slirp dependence with
 * a single resident guest agent (irixga).  Virtuix-only.
 *
 * MMIO layout (4096 bytes, big-endian):
 *   0x000  STATUS         R: device-ready (1) | guest-connected (2)
 *   0x004  H2G_BASE       R/W: guest-physical address of host->guest data ring
 *   0x008  H2G_LEN        R/W: ring size (power-of-two, min 4KB, max 1MB)
 *   0x00C  H2G_WRITE      R: host write position   W: guest ACKs read (advances consumer)
 *   0x010  H2G_READ       W: host ACKs write        R: guest read position
 *   0x014  H2G_DOORBELL   W: guest=0 (clear IRQ) | host=1 (kick guest)
 *   0x018  G2H_BASE       R/W: guest-physical address of guest->host data ring
 *   0x01C  G2H_LEN        R/W: ring size
 *   0x020  G2H_WRITE      W: host ACKs read         R: guest write position
 *   0x024  G2H_READ       R: host read position    W: guest ACKs write
 *   0x028  G2H_DOORBELL   W: host=0 (ack) | guest=1 (host-readable event pending)
 *   0x02C  IRQ_ENABLE     W: 0=off, 1=on — when on, the guest IRQ asserts on
 *                          H2G doorbell host-write until the guest clears it
 *
 * Ring layout (ring lives in guest RAM; both sides maintain indices):
 *   [producer_idx:u32] [consumer_idx:u32]
 * followed by <ring_size> bytes of message data.
 *
 * Message format (in the data region, little-endian for host convenience):
 *   op:u32  status:u32  payload_len:u32  payload:u8[payload_len]
 *   (H2G ops use op; G2H responses use op=echoed_request_op and status)
 *
 * H2G ops:  1=EXEC(command:NUL-term str) 2=PUSH_FILE(path,data) 3=PULL_FILE(path) 4=PING
 * G2H ops echo the request op:
 *   1=EXEC_RESULT(status,output)  2=PUSH_ACK(status)  3=PULL_DATA(status,data)  4=PONG
 *
 * Copyright (c) 2026 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_PVCHAN_H
#define HW_MISC_SGI_PVCHAN_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "chardev/char-fe.h"
#include "qemu/notify.h"
#include "ui/input.h"

#define TYPE_SGI_PVCHAN "sgi-pvchan"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPvChanState, SGI_PVCHAN)

#define SGI_PVCHAN_MMIO_SIZE  0x1000
#define SGI_PVCHAN_IRQ        0

/* registers */
#define PVCHAN_STATUS         0x00
#define PVCHAN_H2G_BASE       0x04
#define PVCHAN_H2G_LEN        0x08
#define PVCHAN_H2G_WRITE      0x0C
#define PVCHAN_H2G_READ       0x10
#define PVCHAN_H2G_DOORBELL   0x14
#define PVCHAN_G2H_BASE       0x18
#define PVCHAN_G2H_LEN        0x1C
#define PVCHAN_G2H_WRITE      0x20
#define PVCHAN_G2H_READ       0x24
#define PVCHAN_G2H_DOORBELL   0x28
#define PVCHAN_IRQ_ENABLE     0x2C

/* status bits */
#define PVCHAN_S_READY        1
#define PVCHAN_S_GUEST_READY  2
#define PVCHAN_S_G2H_PENDING  4

/* ring layout */
#define PVCHAN_RING_HDR_SZ    8
#define PVCHAN_MAX_MSG        65536  /* 64KB max per message */
#define PVCHAN_RING_MIN       4096
#define PVCHAN_NOMSG          (-3)   /* host_read_msg: no complete message ready
                                      * (distinct from a valid zero-payload msg) */

/*
 * Wire ops shared with the guest agent (irixga).  1..4 are the request ops the
 * host sends into H2G; 5/6 are the two host-side-relevant extensions D2/D3 add:
 *   5 = ANNOUNCE  guest→host, unprompted at agent startup (liveness beacon)
 *   6 = SHUTDOWN  host→guest, pushed by the QEMU powerdown notifier so the agent
 *                 runs `sync; sync; init 0` (graceful lifecycle, D3)
 */
#define PVCHAN_OP_EXEC        1
#define PVCHAN_OP_PUSH        2
#define PVCHAN_OP_PULL        3
#define PVCHAN_OP_PING        4
#define PVCHAN_OP_ANNOUNCE    5
#define PVCHAN_OP_SHUTDOWN    6
/*
 * 7 = ABSPTR  host→guest, unsolicited (fire-and-forget, NO reply).  D5 absolute
 *     pointer: the device registers a QEMU absolute input handler; on host
 *     pointer motion over the display it pushes the latest position into the
 *     H2G ring and the resident agent (irixga) XWarpPointer's the guest cursor
 *     there.  Payload [x:u32][y:u32][xmax:u32][ymax:u32] LE, x/y in 0..xmax
 *     (INPUT_EVENT_ABS_MAX = 0x7FFF); the agent scales to its X screen.
 * 8 = CLIP_GRAB  guest→host, unsolicited: the guest asserted a text selection;
 *     payload = UTF-8 text (transcoded from Latin-1 by the agent).  The device
 *     forwards it to the host clipboard (D4 guest→host).
 * 9 = CLIP_DATA  host→guest, unsolicited: the host clipboard changed; payload =
 *     UTF-8 text.  The agent owns PRIMARY+CLIPBOARD and serves it (D4 host→guest).
 */
#define PVCHAN_OP_ABSPTR      7
#define PVCHAN_OP_CLIP_GRAB   8
#define PVCHAN_OP_CLIP_DATA   9

/* Host-side chardev frame sizes: request [op:u32][len:u32][payload],
 * reply [op:u32][status:u32][len:u32][payload].  All little-endian. */
#define PVCHAN_REQ_HDR        8
#define PVCHAN_REP_HDR        12
#define PVCHAN_RX_MAX         (PVCHAN_REQ_HDR + PVCHAN_MAX_MSG)

struct SGIPvChanState {
    SysBusDevice  parent_obj;
    MemoryRegion  mmio;
    qemu_irq      irq;

    /* registers */
    uint32_t status;
    uint32_t h2g_base, h2g_len;
    uint32_t h2g_write, h2g_read;
    uint32_t h2g_dbell;
    uint32_t g2h_base, g2h_len;
    uint32_t g2h_write, g2h_read;
    uint32_t g2h_dbell;
    uint32_t irq_enable;

    /* host-side poll fd (read end of the G2H doorbell event) — set up via qom prop */
    int notify_fd;

    /* D2/D3 proper transport: a persistent chardev socket carrying framed
     * request/reply messages, replacing the per-op HMP monitor reconnect.
     * Default-off: with no chardev bound the HMP path still works. */
    CharFrontend  chr;
    bool          chr_connected;      /* a client socket is actually attached */
    uint8_t       rx[PVCHAN_RX_MAX];  /* request-frame reassembly buffer */
    uint32_t      rx_len;
    Notifier      powerdown_notifier; /* system_powerdown -> OP_SHUTDOWN to guest */
    bool          powerdown_registered;
    bool          powerdown_pending;  /* host pressed power: await guest sync-ack
                                       * then request a clean QEMU shutdown */

    /* D5 absolute pointer: a QEMU abs input handler feeds the latest host
     * pointer position; the sync callback pushes one OP_ABSPTR per batch into
     * H2G (coalesced — dropped if the ring is momentarily full).  ABS-only mask,
     * so buttons/wheel keep flowing to the PS/2 mouse (guest-local clicking). */
    QemuInputHandlerState *input_handler;
    int32_t       abs_x, abs_y;       /* latest, 0..INPUT_EVENT_ABS_MAX */
    bool          abs_dirty;          /* motion since last push */

    /* D4 clipboard: pvchan registers as a QEMU clipboard peer (opt-in via the
     * `clipboard` property, default off).  Guest CLIP_GRAB updates the host
     * clipboard; a host clipboard change pushes CLIP_DATA to the guest.  When
     * enabled, G2H is drained unconditionally (to catch unsolicited guest grabs
     * with no PvChanClient attached), so an unconsumed startup ANNOUNCE is
     * stashed here and re-emitted when a chardev client connects. */
    bool          clip_enabled;       /* `clipboard=on` */
    char         *clip_from_guest;    /* last text the guest grabbed (host serves) */
    bool          pending_ann_valid;  /* a startup ANNOUNCE was consumed pre-client */
    uint32_t      pending_ann_op, pending_ann_status, pending_ann_len;
    uint8_t       pending_ann[128];   /* stashed ANNOUNCE payload */
};

/*
 * Host-side helpers: read/write ring payloads from/to guest RAM.
 * Returns msg length consumed/produced, or -errno on error.
 */
int pvchan_host_read_msg(SGIPvChanState *s, uint32_t *op, uint32_t *status,
                         uint8_t *buf, uint32_t max);
int pvchan_host_write_msg(SGIPvChanState *s, uint32_t op,
                          const uint8_t *payload, uint32_t len);

#endif /* HW_MISC_SGI_PVCHAN_H */
