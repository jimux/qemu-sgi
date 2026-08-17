/*
 * sgi-pvchan — paravirtual host<->guest communication channel (virtuix-only).
 *
 * A pair of shared rings with doorbells: the host writes commands into the
 * H2G (host→guest) ring and kicks the guest IRQ; the guest writes responses
 * into the G2H (guest→host) ring and signals the host.  Designed to replace
 * serial + gdbstub-halt + slirp/TFTP with a single resident guest agent.
 *
 * Modeled on sgi_glaccel.c's ring + sgi_smp.c's simple MMIO/IPI pattern.
 *
 * Copyright (c) 2026 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_pvchan.h"
#include "hw/misc/sgi_virtuix_gpa.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "system/runstate.h"
#include "chardev/char-fe.h"
#include "ui/input.h"
#include "ui/clipboard.h"

/* ---- singleton (for HMP lookups) ------------------------------------------ */
static SGIPvChanState *pvchan_instance;

/*
 * Ring convention (2026-07-07 D1 redesign — replaces the internally inconsistent
 * original that mixed `%rlen` producer publishing with `%(len*2)` consumer
 * publishing and a `pi += len` reader; that could not survive an index wrap).
 *
 * Both rings live in guest RAM: [producer:u32 LE][consumer:u32 LE] then LEN
 * bytes of data.  Indices are FREE-RUNNING uint32 that wrap naturally at 2^32.
 * The byte position in the data region is `idx & (LEN-1)` — so LEN MUST be a
 * power of two, which makes `idx % LEN` seamless across the 2^32 wrap
 * (2^32 % LEN == 0).  Bytes available to the consumer = (uint32)(prod - cons),
 * correct across wrap while < LEN bytes are outstanding.  This is the standard
 * virtio-shaped scheme and is byte-for-byte matched by the guest driver/agent.
 *
 * Message framing in the data region (LE for host convenience):
 *   op:u32  status:u32  payload_len:u32  payload:u8[payload_len]
 */
static uint32_t rd32_le(SGIPvChanState *s, uint64_t pa)
{
    uint32_t v;
    dma_memory_read(&address_space_memory, pa, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void wr32_le(SGIPvChanState *s, uint64_t pa, uint32_t v)
{
    v = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, pa, &v, 4, MEMTXATTRS_UNSPECIFIED);
}

/* read `n` bytes from the data region starting at free-running index `idx`
 * (wraps within LEN); `base` is the ring header physical address. */
static void ring_read(SGIPvChanState *s, uint64_t base, uint32_t len,
                      uint32_t idx, uint8_t *dst, uint32_t n)
{
    uint64_t data = base + PVCHAN_RING_HDR_SZ;
    uint32_t off = idx & (len - 1);
    uint32_t chunk = len - off;
    if (chunk >= n) {
        dma_memory_read(&address_space_memory, data + off, dst, n,
                        MEMTXATTRS_UNSPECIFIED);
    } else {
        dma_memory_read(&address_space_memory, data + off, dst, chunk,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, data, dst + chunk, n - chunk,
                        MEMTXATTRS_UNSPECIFIED);
    }
}

/* write `n` bytes into the data region starting at free-running index `idx`. */
static void ring_write(SGIPvChanState *s, uint64_t base, uint32_t len,
                       uint32_t idx, const uint8_t *src, uint32_t n)
{
    uint64_t data = base + PVCHAN_RING_HDR_SZ;
    uint32_t off = idx & (len - 1);
    uint32_t chunk = len - off;
    if (chunk >= n) {
        dma_memory_write(&address_space_memory, data + off, src, n,
                         MEMTXATTRS_UNSPECIFIED);
    } else {
        dma_memory_write(&address_space_memory, data + off, src, chunk,
                         MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, data, src + chunk, n - chunk,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* LEN must be a power of two >= 4KB and <= 64MB. */
static bool ring_len_ok(uint32_t len)
{
    return len >= PVCHAN_RING_MIN && len <= (64u << 20) &&
           (len & (len - 1)) == 0;
}

/*
 * Translate a guest-programmed ring base to a guest physical address.
 *
 * The guest driver (virtuix/kernel/drivers/pvchan.c) programs kvtophys() of its
 * page-aligned kernel-BSS rings -- a *raw physical* base (bit 31 clear).  On a
 * >256MB guest that base can land in RAM SEG1 (physical bit 29 set); the old
 * `& 0x1FFFFFFF` mask dropped bit 29 and pointed the rings at a wrong low page.
 * SGI_VIRTUIX_GPA preserves SEG1 for physical bases (and still strips KSEG for a
 * virtual one).  Kernel BSS is normally allocated low (SEG0), so this was latent
 * -- but the rings are not pinned low by contract, so the fix hardens the
 * announce / abs-pointer / clipboard channel for the -m 512M configs.
 *
 * PVCHAN_SEG1_TRACE=1 logs (rate-limited) when a base actually lands in SEG1.
 */
static uint64_t pvchan_ring_gpa(uint64_t base)
{
    static int trace = -1;
    if (trace < 0) {
        trace = getenv("PVCHAN_SEG1_TRACE") ? 1 : 0;
    }
    if (trace && (base & 0x20000000ULL) && !(base & 0x80000000ULL)) {
        static uint32_t n;
        if ((n++ & 0xff) == 0) {
            fprintf(stderr, "PVCHAN_SEG1_TRACE: ring base=0x%" PRIx64
                    " in SEG1 (bit29 set) -- old mask would truncate to 0x%"
                    PRIx64 " (hit #%u)\n",
                    base, (uint64_t)(base & 0x1FFFFFFFULL), n);
        }
    }
    return SGI_VIRTUIX_GPA(base);
}

/* ---- host-side helpers (called from Python via QMP/HMP or monitor) -------- */

/* Read one message from the G2H ring (host is consumer).  Returns payload
 * length (>=0) on success, 0 if no complete message is ready, -1 on error. */
int pvchan_host_read_msg(SGIPvChanState *s, uint32_t *op, uint32_t *status,
                         uint8_t *buf, uint32_t max)
{
    uint64_t base = pvchan_ring_gpa(s->g2h_base);
    uint32_t len  = s->g2h_len;
    uint32_t prod, cons, avail, plen;
    uint32_t hdr[3]; /* op, status, plen — LE */
    if (!base || !ring_len_ok(len)) return -1;
    prod = rd32_le(s, base);
    cons = rd32_le(s, base + 4);
    avail = prod - cons;                 /* wrap-correct */
    if (avail > len) return -1;          /* desync guard */
    if (avail < 12) return PVCHAN_NOMSG; /* no complete header yet */
    ring_read(s, base, len, cons, (uint8_t *)hdr, 12);
    plen = le32_to_cpu(hdr[2]);
    if (plen > PVCHAN_MAX_MSG) { s->g2h_read = cons + 12; wr32_le(s, base + 4, cons + 12); return -1; }
    if (avail < 12 + plen) return PVCHAN_NOMSG; /* payload not fully written yet */
    *op = le32_to_cpu(hdr[0]);
    *status = le32_to_cpu(hdr[1]);
    if (plen > 0) {
        ring_read(s, base, len, cons + 12, buf, MIN(plen, max));
    }
    cons += 12 + plen;
    wr32_le(s, base + 4, cons);          /* publish host consumer */
    s->g2h_read = cons;
    return (plen > max) ? -1 : (int)plen;
}

/* Write one message into the H2G ring (host is producer).  Returns total
 * bytes written (>0) or -1 bad-arg / -2 ring-full. */
int pvchan_host_write_msg(SGIPvChanState *s, uint32_t op,
                          const uint8_t *payload, uint32_t len)
{
    uint64_t base = pvchan_ring_gpa(s->h2g_base);
    uint32_t rlen = s->h2g_len;
    uint32_t prod, cons, avail, freeb, total;
    uint8_t hdr[12];
    if (!base || !ring_len_ok(rlen) || len > PVCHAN_MAX_MSG) return -1;
    prod = rd32_le(s, base);
    cons = rd32_le(s, base + 4);
    avail = prod - cons;                 /* wrap-correct */
    freeb = rlen - avail;
    total = 12 + len;
    if (freeb < total) return -2;        /* no room */
    *(uint32_t *)(hdr + 0) = cpu_to_le32(op);
    *(uint32_t *)(hdr + 4) = cpu_to_le32(0);
    *(uint32_t *)(hdr + 8) = cpu_to_le32(len);
    ring_write(s, base, rlen, prod, hdr, 12);
    if (len > 0) ring_write(s, base, rlen, prod + 12, payload, len);
    prod += total;
    wr32_le(s, base, prod);              /* publish host producer */
    s->h2g_write = prod;
    /* kick guest IRQ (only if guest enabled it; agent may instead poll) */
    s->h2g_dbell = 1;
    if (s->irq_enable) qemu_irq_raise(s->irq);
    return (int)total;
}

/* ---- chardev transport (the D2/D3 "off-HMP" persistent socket) ------------
 *
 * A single persistent socket carries length-framed messages, virtio-serial in
 * spirit.  Host client -> device: request frames [op:u32][len:u32][payload].
 * Device -> host client: reply frames [op:u32][status:u32][len:u32][payload],
 * drained from the G2H ring whenever the guest rings the doorbell (or when the
 * client (re)connects, to pick up an unprompted ANNOUNCE).  Both directions are
 * little-endian.  All of this runs under the BQL (MMIO write handlers and
 * chardev callbacks both hold it), so no extra locking is needed. */

static void pvchan_put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xff; p[1] = (v >> 8) & 0xff;
    p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}

static uint32_t pvchan_get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* D4 clipboard: defined below, used by pvchan_process_g2h (guest CLIP_GRAB). */
static void pvchan_clip_from_guest(SGIPvChanState *s, const uint8_t *text,
                                   uint32_t len);

/*
 * Process every complete G2H message the guest produced.  Two consumers:
 *   - if a client socket is attached, each message is forwarded as a reply
 *     frame [op][status][len][payload];
 *   - an OP_SHUTDOWN reply is the agent saying "I have synced; safe to power
 *     off" (D3) — turn it into a clean QEMU shutdown request so the host power
 *     button completes unattended (see pvchan_powerdown_notify).
 *
 * We must NOT consume messages when there is neither a client nor a pending
 * power-off: the unprompted startup ANNOUNCE has to stay in the ring until a
 * client connects (CHR_EVENT_OPENED flushes it).  `powerdown_pending` lets us
 * drain-to-find-shutdown even with no client, without eating the ANNOUNCE in
 * the normal case.
 */
static void pvchan_process_g2h(SGIPvChanState *s)
{
    static uint8_t payload[PVCHAN_MAX_MSG];
    uint8_t hdr[PVCHAN_REP_HDR];
    uint32_t op, status;
    int n;

    /* Drain when a client is attached, a power-off is pending, OR clipboard
     * mode is on (guest CLIP_GRAB is unsolicited — nobody polls for it). */
    if (!s->chr_connected && !s->powerdown_pending && !s->clip_enabled) {
        return;
    }
    for (;;) {
        n = pvchan_host_read_msg(s, &op, &status, payload, sizeof(payload));
        if (n == PVCHAN_NOMSG || n < 0) {
            break;  /* nothing more ready (or a desync we already skipped past) */
        }
        /* guest clipboard grab: with the clipboard peer enabled it goes to the
         * host clipboard; otherwise (clipboard=off) forward it to the chardev
         * client so a host CLI can read the guest selection (BL-51 clipget). */
        if (op == PVCHAN_OP_CLIP_GRAB) {
            if (s->clip_enabled) {
                pvchan_clip_from_guest(s, payload, (uint32_t)n);
                continue;
            }
            /* fall through to the chardev-forwarding path below */
        }
        /* clipboard mode may drain the startup ANNOUNCE before any client is
         * attached; stash it so CHR_EVENT_OPENED can re-emit it (wait_announce). */
        if (op == PVCHAN_OP_ANNOUNCE && !s->chr_connected) {
            s->pending_ann_op = op;
            s->pending_ann_status = status;
            s->pending_ann_len = MIN((uint32_t)n, sizeof(s->pending_ann));
            if (s->pending_ann_len) {
                memcpy(s->pending_ann, payload, s->pending_ann_len);
            }
            s->pending_ann_valid = true;
            continue;
        }
        if (s->chr_connected) {
            pvchan_put_le32(hdr + 0, op);
            pvchan_put_le32(hdr + 4, status);
            pvchan_put_le32(hdr + 8, (uint32_t)n);
            qemu_chr_fe_write_all(&s->chr, hdr, PVCHAN_REP_HDR);
            if (n > 0) {
                qemu_chr_fe_write_all(&s->chr, payload, n);
            }
        }
        if (op == PVCHAN_OP_SHUTDOWN && s->powerdown_pending) {
            qemu_log("sgi-pvchan: guest synced -> requesting QEMU shutdown\n");
            s->powerdown_pending = false;
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
    }
}

/* Host client can always take a full frame's worth. */
static int pvchan_chr_can_receive(void *opaque)
{
    SGIPvChanState *s = opaque;
    return sizeof(s->rx) - s->rx_len;
}

/* Accumulate bytes, dispatch each complete request frame into the H2G ring. */
static void pvchan_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIPvChanState *s = opaque;
    uint32_t op, plen, consumed;

    if (size <= 0) {
        return;
    }
    if ((uint32_t)size > sizeof(s->rx) - s->rx_len) {
        /* impossible given can_receive, but never overrun */
        size = sizeof(s->rx) - s->rx_len;
    }
    memcpy(s->rx + s->rx_len, buf, size);
    s->rx_len += size;

    while (s->rx_len >= PVCHAN_REQ_HDR) {
        op   = pvchan_get_le32(s->rx + 0);
        plen = pvchan_get_le32(s->rx + 4);
        if (plen > PVCHAN_MAX_MSG) {
            /* framing lost — resync by dropping the buffer */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sgi-pvchan: chardev frame len %u > max, resync\n", plen);
            s->rx_len = 0;
            break;
        }
        if (s->rx_len < PVCHAN_REQ_HDR + plen) {
            break;  /* wait for the rest of the payload */
        }
        pvchan_host_write_msg(s, op, s->rx + PVCHAN_REQ_HDR, plen);
        consumed = PVCHAN_REQ_HDR + plen;
        memmove(s->rx, s->rx + consumed, s->rx_len - consumed);
        s->rx_len -= consumed;
    }
}

static void pvchan_chr_event(void *opaque, QEMUChrEvent ev)
{
    SGIPvChanState *s = opaque;
    if (ev == CHR_EVENT_OPENED) {
        /* fresh client: drop any half-frame, then flush pending G2H (announce) */
        s->rx_len = 0;
        s->chr_connected = true;
        /* If clipboard mode already drained the startup ANNOUNCE before any
         * client attached, re-emit it now so wait_announce() still sees it. */
        if (s->pending_ann_valid) {
            uint8_t hdr[PVCHAN_REP_HDR];
            pvchan_put_le32(hdr + 0, s->pending_ann_op);
            pvchan_put_le32(hdr + 4, s->pending_ann_status);
            pvchan_put_le32(hdr + 8, s->pending_ann_len);
            qemu_chr_fe_write_all(&s->chr, hdr, PVCHAN_REP_HDR);
            if (s->pending_ann_len) {
                qemu_chr_fe_write_all(&s->chr, s->pending_ann, s->pending_ann_len);
            }
            s->pending_ann_valid = false;
        }
        pvchan_process_g2h(s);
    } else if (ev == CHR_EVENT_CLOSED) {
        s->rx_len = 0;
        s->chr_connected = false;
    }
}

/* ---- D5 absolute pointer -------------------------------------------------
 *
 * A QEMU absolute input handler (ABS mask only).  With it registered+active,
 * the GTK display switches to absolute mode (no grab) and delivers pointer
 * position as ABS x/y events; BUTTON/wheel events keep their own mask and route
 * to the PS/2 mouse, so guest-local clicking is unchanged.  On each input batch
 * (sync) we push ONE OP_ABSPTR frame with the latest position into the H2G ring;
 * the resident agent (irixga) XWarpPointer's the guest cursor there.  If the
 * ring is momentarily full the push is dropped — positions are idempotent-latest
 * so this coalesces cleanly under backpressure.  Values are 0..INPUT_EVENT_ABS_MAX
 * (0x7FFF); the guest scales to its X screen. */
static void pvchan_ptr_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    InputMoveEvent *move;
    if (evt->type != INPUT_EVENT_KIND_ABS) {
        return;
    }
    move = evt->u.abs.data;
    if (move->axis == INPUT_AXIS_X) {
        s->abs_x = move->value;
        s->abs_dirty = true;
    } else if (move->axis == INPUT_AXIS_Y) {
        s->abs_y = move->value;
        s->abs_dirty = true;
    }
}

static void pvchan_ptr_sync(DeviceState *dev)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    uint8_t pl[16];
    if (!s->abs_dirty) {
        return;
    }
    s->abs_dirty = false;
    pvchan_put_le32(pl + 0,  (uint32_t)s->abs_x);
    pvchan_put_le32(pl + 4,  (uint32_t)s->abs_y);
    pvchan_put_le32(pl + 8,  (uint32_t)INPUT_EVENT_ABS_MAX);
    pvchan_put_le32(pl + 12, (uint32_t)INPUT_EVENT_ABS_MAX);
    /* fire-and-forget; -1 (agent not attached) / -2 (ring full) both just drop */
    (void)pvchan_host_write_msg(s, PVCHAN_OP_ABSPTR, pl, sizeof(pl));
}

static const QemuInputHandler pvchan_ptr_handler = {
    .name  = "sgi-pvchan tablet",
    .mask  = INPUT_EVENT_MASK_ABS,
    .event = pvchan_ptr_event,
    .sync  = pvchan_ptr_sync,
};

/* ---- D4 clipboard (opt-in `clipboard=on`) --------------------------------
 *
 * pvchan registers as a QEMU clipboard peer.  Guest→host: the agent sends
 * OP_CLIP_GRAB with the UTF-8 selection text; we publish it to the host
 * clipboard for BOTH selections (PRIMARY + CLIPBOARD) so a host paste or
 * middle-click both work.  Host→guest: when another peer (GTK) updates the host
 * clipboard we request its text and push OP_CLIP_DATA to the guest, which then
 * owns PRIMARY+CLIPBOARD and serves it to IRIX apps.  Text only; the agent does
 * the Latin-1<->UTF-8 transcode. */
static QemuClipboardPeer pvchan_clip_peer;

/* guest grabbed a selection: make its text the host clipboard content. */
static void pvchan_clip_from_guest(SGIPvChanState *s, const uint8_t *text, uint32_t len)
{
    int sel;
    g_free(s->clip_from_guest);
    s->clip_from_guest = g_strndup((const char *)text, len);
    qemu_log("sgi-pvchan: CLIP_GRAB from guest (%u bytes) -> host clipboard\n", len);
    for (sel = 0; sel < 2; sel++) {   /* CLIPBOARD(0) + PRIMARY(1) */
        QemuClipboardInfo *info =
            qemu_clipboard_info_new(&pvchan_clip_peer, sel);
        info->types[QEMU_CLIPBOARD_TYPE_TEXT].available = true;
        qemu_clipboard_update(info);
        qemu_clipboard_set_data(&pvchan_clip_peer, info,
                                QEMU_CLIPBOARD_TYPE_TEXT, len,
                                s->clip_from_guest, true);
        qemu_clipboard_info_unref(info);
    }
}

/* host clipboard changed (some other peer) -> push text to the guest. */
static void pvchan_clip_notify(Notifier *notifier, void *data)
{
    SGIPvChanState *s = pvchan_instance;
    QemuClipboardNotify *notify = data;
    QemuClipboardInfo *info;
    QemuClipboardContent *c;
    if (!s || notify->type != QEMU_CLIPBOARD_UPDATE_INFO) {
        return;
    }
    info = notify->info;
    if (!info || info->owner == &pvchan_clip_peer) {
        return;   /* our own update — don't echo back to the guest */
    }
    if (info != qemu_clipboard_info(info->selection)) {
        return;   /* not the current clipboard */
    }
    c = &info->types[QEMU_CLIPBOARD_TYPE_TEXT];
    if (!c->available) {
        return;   /* no text on this selection */
    }
    if (!c->data) {
        /* ask the owner to provide the bytes; it re-notifies when ready */
        qemu_clipboard_request(info, QEMU_CLIPBOARD_TYPE_TEXT);
        return;
    }
    (void)pvchan_host_write_msg(s, PVCHAN_OP_CLIP_DATA, c->data, c->size);
}

/* peer request callback: the host UI wants the data behind a grab we announced.
 * We already hold the guest's text in clip_from_guest, so serve it directly. */
static void pvchan_clip_request(QemuClipboardInfo *info, QemuClipboardType type)
{
    SGIPvChanState *s = pvchan_instance;
    if (!s || type != QEMU_CLIPBOARD_TYPE_TEXT || !s->clip_from_guest) {
        return;
    }
    qemu_clipboard_set_data(&pvchan_clip_peer, info, type,
                            strlen(s->clip_from_guest), s->clip_from_guest, true);
}

/* ---- MMIO handlers -------------------------------------------------------- */

static uint64_t sgi_pvchan_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIPvChanState *s = opaque;
    switch (addr) {
    case PVCHAN_STATUS:       return s->status;
    case PVCHAN_H2G_BASE:     return s->h2g_base;
    case PVCHAN_H2G_LEN:      return s->h2g_len;
    case PVCHAN_H2G_WRITE:    return s->h2g_write;
    case PVCHAN_H2G_READ:     return s->h2g_read;
    case PVCHAN_H2G_DOORBELL: return s->h2g_dbell;
    case PVCHAN_G2H_BASE:     return s->g2h_base;
    case PVCHAN_G2H_LEN:      return s->g2h_len;
    case PVCHAN_G2H_WRITE:    return s->g2h_write;
    case PVCHAN_G2H_READ:     return s->g2h_read;
    case PVCHAN_G2H_DOORBELL: return s->g2h_dbell;
    case PVCHAN_IRQ_ENABLE:   return s->irq_enable;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sgi-pvchan: bad read 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void sgi_pvchan_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SGIPvChanState *s = opaque;
    switch (addr) {
    case PVCHAN_STATUS:
        s->status = val & 0xf;
        break;
    case PVCHAN_H2G_BASE:
        s->h2g_base = (uint32_t)val;
        s->status |= PVCHAN_S_GUEST_READY;
        break;
    case PVCHAN_H2G_LEN:
        s->h2g_len = (uint32_t)val;
        break;
    case PVCHAN_H2G_WRITE:
        /* guest writes to acknowledge consuming bytes (advance consumer) */
        s->h2g_read = (uint32_t)val;
        break;
    case PVCHAN_H2G_READ:
        s->h2g_read = (uint32_t)val;
        break;
    case PVCHAN_H2G_DOORBELL:
        if (val == 0) { /* guest clear: deassert IRQ */
            s->h2g_dbell = 0;
            qemu_irq_lower(s->irq);
        }
        /* host-write=1 handled by pvchan_host_write_msg */
        break;
    case PVCHAN_G2H_BASE:
        s->g2h_base = (uint32_t)val;
        break;
    case PVCHAN_G2H_LEN:
        s->g2h_len = (uint32_t)val;
        break;
    case PVCHAN_G2H_WRITE:
        s->g2h_write = (uint32_t)val;
        break;
    case PVCHAN_G2H_READ:
        s->g2h_read = (uint32_t)val;
        break;
    case PVCHAN_G2H_DOORBELL:
        if (val == 1) { /* guest signals data ready */
            s->g2h_dbell = 1;
            s->status |= PVCHAN_S_G2H_PENDING;
            /* host-side notify: write one byte to notify_fd if open */
            if (s->notify_fd >= 0) {
                uint8_t b = 1;
                if (write(s->notify_fd, &b, 1) != 1) { /* ignore - pipe full etc */ }
            }
            /* proper transport: push replies straight out the chardev socket */
            pvchan_process_g2h(s);
        } else { /* host acknowledges */
            s->g2h_dbell = 0;
            s->status &= ~PVCHAN_S_G2H_PENDING;
        }
        break;
    case PVCHAN_IRQ_ENABLE:
        s->irq_enable = val ? 1 : 0;
        if (!s->irq_enable) qemu_irq_lower(s->irq);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sgi-pvchan: bad write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, val);
    }
}

static const MemoryRegionOps sgi_pvchan_ops = {
    .read  = sgi_pvchan_read,
    .write = sgi_pvchan_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- HMP handlers (host→device communication) --------------------------- */

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int hex_decode(const char *hex, uint8_t *out, int max)
{
    int i = 0, len = strlen(hex);
    if (len & 1) return -1; /* must be even */
    for (int j = 0; j < len && i < max; j += 2) {
        int hi = hex_nibble(hex[j]), lo = hex_nibble(hex[j + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i++] = (uint8_t)((hi << 4) | lo);
    }
    return i;
}

void hmp_pvchan_send(Monitor *mon, const QDict *qdict)
{
    uint32_t op = (uint32_t)qdict_get_int(qdict, "op");
    const char *hex = qdict_get_try_str(qdict, "data");  /* NULL => empty payload */
    uint8_t payload[PVCHAN_MAX_MSG];
    int plen;
    SGIPvChanState *s;
    int rc;

    if (!pvchan_instance) {
        monitor_printf(mon, "pvchan: no device (virtuix not running?)\n");
        return;
    }
    s = pvchan_instance;
    plen = hex ? hex_decode(hex, payload, sizeof(payload)) : 0;
    if (plen < 0) {
        monitor_printf(mon, "pvchan-send: invalid hex data\n");
        return;
    }
    rc = pvchan_host_write_msg(s, op, payload, (uint32_t)plen);
    if (rc < 0) {
        monitor_printf(mon, "pvchan-send: failed (ring full or device not ready? rc=%d)\n", rc);
    } else {
        monitor_printf(mon, "pvchan-send: ok, %d bytes written\n", rc);
    }
}

/* Send an op whose payload is the contents of a host file (up to
 * PVCHAN_MAX_MSG bytes).  Avoids the ~4KB HMP input-line limit that makes
 * inline hex impractical for large transfers; this is also the shape real
 * host-side tooling uses (read file bytes, push them). */
void hmp_pvchan_sendfile(Monitor *mon, const QDict *qdict)
{
    uint32_t op = (uint32_t)qdict_get_int(qdict, "op");
    const char *path = qdict_get_str(qdict, "path");
    SGIPvChanState *s;
    uint8_t *payload;
    FILE *f;
    size_t n;
    int rc;

    if (!pvchan_instance) {
        monitor_printf(mon, "pvchan: no device (virtuix not running?)\n");
        return;
    }
    s = pvchan_instance;
    f = fopen(path, "rb");
    if (!f) {
        monitor_printf(mon, "pvchan-sendfile: cannot open %s\n", path);
        return;
    }
    payload = g_malloc(PVCHAN_MAX_MSG);
    n = fread(payload, 1, PVCHAN_MAX_MSG, f);
    fclose(f);
    rc = pvchan_host_write_msg(s, op, payload, (uint32_t)n);
    g_free(payload);
    if (rc < 0) {
        monitor_printf(mon, "pvchan-sendfile: failed (ring full/not ready? rc=%d)\n", rc);
    } else {
        monitor_printf(mon, "pvchan-sendfile: ok, %d bytes written\n", rc);
    }
}

void hmp_pvchan_recv(Monitor *mon, const QDict *qdict)
{
    uint32_t op, status;
    uint8_t buf[PVCHAN_MAX_MSG];
    int plen;
    SGIPvChanState *s;

    if (!pvchan_instance) {
        monitor_printf(mon, "pvchan: no device (virtuix not running?)\n");
        return;
    }
    s = pvchan_instance;
    plen = pvchan_host_read_msg(s, &op, &status, buf, sizeof(buf));
    if (plen == PVCHAN_NOMSG) {
        monitor_printf(mon, "pvchan-recv: no message ready\n");
    } else if (plen < 0) {
        monitor_printf(mon, "pvchan-recv: error %d\n", plen);
    } else {
        monitor_printf(mon, "pvchan-recv: op=%u status=%u plen=%d\n", op, status, plen);
        if (plen > 0) {
            /* emit the FULL payload as one hex line so the host harness can
             * sha256-verify multi-KB replies (no 256-byte truncation).  Build
             * into a heap buffer and print in one call — a per-byte
             * monitor_printf loop is prohibitively slow at 64KB. */
            static const char hx[] = "0123456789abcdef";
            char *line = g_malloc((size_t)plen * 2 + 16);
            int i;
            memcpy(line, "  hex: ", 7);
            for (i = 0; i < plen; i++) {
                line[7 + i * 2]     = hx[(buf[i] >> 4) & 0xf];
                line[7 + i * 2 + 1] = hx[buf[i] & 0xf];
            }
            line[7 + plen * 2] = '\n';
            line[7 + plen * 2 + 1] = '\0';
            monitor_puts(mon, line);
            g_free(line);
        }
    }
}

/* ---- D4 clipboard host-side HMP hooks (test + future host CLI) ------------
 *
 * pvchan-clipget:  print the current host clipboard text (guest->host proof).
 * pvchan-clipset:  simulate a host-side copy (host->guest proof) — registers a
 *                  throwaway "hosttest" peer and updates the clipboard, which
 *                  fires pvchan's notifier and pushes OP_CLIP_DATA to the guest. */
static QemuClipboardPeer pvchan_hosttest_peer;
static char *pvchan_hosttest_text;

/* Every registered peer's notifier is invoked on qemu_clipboard_update — a NULL
 * notify pointer would crash, so the hosttest peer needs a (no-op) notifier. */
static void pvchan_hosttest_notify(Notifier *n, void *data) { }

static void pvchan_hosttest_request(QemuClipboardInfo *info, QemuClipboardType t)
{
    if (t == QEMU_CLIPBOARD_TYPE_TEXT && pvchan_hosttest_text) {
        qemu_clipboard_set_data(&pvchan_hosttest_peer, info, t,
                                strlen(pvchan_hosttest_text),
                                pvchan_hosttest_text, true);
    }
}

void hmp_pvchan_clipget(Monitor *mon, const QDict *qdict)
{
    static const char *names[2] = { "CLIPBOARD", "PRIMARY" };
    int sel;
    for (sel = 0; sel < 2; sel++) {
        QemuClipboardInfo *info = qemu_clipboard_info(sel);
        QemuClipboardContent *c = info ?
            &info->types[QEMU_CLIPBOARD_TYPE_TEXT] : NULL;
        if (c && c->available && c->data) {
            monitor_printf(mon, "%s: %.*s\n", names[sel], (int)c->size,
                           (const char *)c->data);
        } else {
            monitor_printf(mon, "%s: <empty>\n", names[sel]);
        }
    }
}

void hmp_pvchan_clipset(Monitor *mon, const QDict *qdict)
{
    const char *text = qdict_get_str(qdict, "text");
    int sel;
    if (!pvchan_instance || !pvchan_instance->clip_enabled) {
        monitor_printf(mon, "pvchan: clipboard not enabled (clipboard=on?)\n");
        return;
    }
    if (!pvchan_hosttest_peer.name) {
        pvchan_hosttest_peer.name = "hosttest";
        pvchan_hosttest_peer.notifier.notify = pvchan_hosttest_notify;
        pvchan_hosttest_peer.request = pvchan_hosttest_request;
        qemu_clipboard_peer_register(&pvchan_hosttest_peer);
    }
    g_free(pvchan_hosttest_text);
    pvchan_hosttest_text = g_strdup(text);
    for (sel = 0; sel < 2; sel++) {
        QemuClipboardInfo *info =
            qemu_clipboard_info_new(&pvchan_hosttest_peer, sel);
        info->types[QEMU_CLIPBOARD_TYPE_TEXT].available = true;
        qemu_clipboard_update(info);
        qemu_clipboard_set_data(&pvchan_hosttest_peer, info,
                                QEMU_CLIPBOARD_TYPE_TEXT, strlen(text),
                                text, true);
        qemu_clipboard_info_unref(info);
    }
    monitor_printf(mon, "pvchan-clipset: ok (%zu bytes -> guest)\n", strlen(text));
}

/* ---- device lifecycle ---------------------------------------------------- */

static const Property sgi_pvchan_props[] = {
    DEFINE_PROP_INT32("notify-fd", SGIPvChanState, notify_fd, -1),
    DEFINE_PROP_CHR("chardev", SGIPvChanState, chr),
    /* D4 clipboard bridge — default off (drains G2H unconditionally, which
     * changes ANNOUNCE timing; keep the proven A5/D2 config unperturbed). */
    DEFINE_PROP_BOOL("clipboard", SGIPvChanState, clip_enabled, false),
};

/*
 * Host power button (system_powerdown).  Pushes OP_SHUTDOWN into the H2G ring
 * (the agent polls it, so this works with or without a chardev) and arms
 * `powerdown_pending`.  The agent syncs the filesystems and replies OP_SHUTDOWN;
 * pvchan_process_g2h turns that reply into a clean qemu_system_shutdown_request.
 *
 * Why not have the guest halt itself: on this IP22/Mode-K kernel `init 0`,
 * `uadmin(A_SHUTDOWN, AD_HALT)` and `/etc/halt` do NOT reach ARCS Halt from a
 * daemon/non-console context (telinit's runlevel change needs a login session;
 * the halt vector is only reached from the console).  So the disk-safe,
 * unattended power button = guest syncs (crash-consistent under
 * cache=writethrough), then the host cleanly exits QEMU.
 */
static void pvchan_powerdown_notify(Notifier *n, void *opaque)
{
    SGIPvChanState *s = container_of(n, SGIPvChanState, powerdown_notifier);
    int rc;
    s->powerdown_pending = true;
    rc = pvchan_host_write_msg(s, PVCHAN_OP_SHUTDOWN, NULL, 0);
    qemu_log("sgi-pvchan: powerdown -> OP_SHUTDOWN to guest (rc=%d)\n", rc);
    if (rc < 0) {
        /* guest ring not ready (no agent yet) — power off directly */
        s->powerdown_pending = false;
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

static void sgi_pvchan_realize(DeviceState *dev, Error **errp)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvchan_ops, s,
                          "sgi-pvchan", SGI_PVCHAN_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->status = PVCHAN_S_READY;
    s->rx_len = 0;

    /* bind the chardev transport if one was configured (-global .chardev=id) */
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, pvchan_chr_can_receive,
                                 pvchan_chr_receive, pvchan_chr_event,
                                 NULL, s, NULL, true);
    }

    /* graceful lifecycle (D3): fire OP_SHUTDOWN on host system_powerdown */
    s->powerdown_notifier.notify = pvchan_powerdown_notify;
    qemu_register_powerdown_notifier(&s->powerdown_notifier);
    s->powerdown_registered = true;

    /* D5 absolute pointer: register the abs input handler (always).  Only emits
     * H2G frames once the guest agent has programmed the ring, and only under a
     * display that sends abs events — headless/CI boots are unaffected.
     *
     * activate() moves this handler to the HEAD of the global input-handler
     * list so qemu_input_find_handler(REL|ABS) returns it first, making
     * qemu_input_is_absolute() true.  Without this the earlier-registered PS/2
     * REL mouse wins that lookup, so GTK/VNC send RELATIVE motion (to the PS/2
     * mouse) instead of absolute position (to us) for REAL display input — the
     * D5 gate only ever passed because QMP input-send-event abs events route to
     * this handler by event-type, bypassing is_absolute.  Buttons still reach
     * the PS/2 mouse (routed by BTN event-type, independent of head order), so
     * guest-local clicking is unchanged.  Headless/CI boots have no display and
     * are unaffected. */
    s->input_handler = qemu_input_handler_register(dev, &pvchan_ptr_handler);
    qemu_input_handler_activate(s->input_handler);

    pvchan_instance = s;

    /* D4 clipboard peer (opt-in) */
    if (s->clip_enabled) {
        pvchan_clip_peer.name = "sgi-pvchan";
        pvchan_clip_peer.notifier.notify = pvchan_clip_notify;
        pvchan_clip_peer.request = pvchan_clip_request;
        qemu_clipboard_peer_register(&pvchan_clip_peer);
    }
}

static void sgi_pvchan_reset(DeviceState *dev)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    s->h2g_base = s->h2g_len = s->h2g_write = s->h2g_read = 0;
    s->g2h_base = s->g2h_len = s->g2h_write = s->g2h_read = 0;
    s->h2g_dbell = s->g2h_dbell = 0;
    s->irq_enable = 0;
    s->status = PVCHAN_S_READY;
    s->abs_dirty = false;
    s->pending_ann_valid = false;
    qemu_irq_lower(s->irq);
}

static void sgi_pvchan_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    device_class_set_props(dc, sgi_pvchan_props);
    dc->realize = sgi_pvchan_realize;
    device_class_set_legacy_reset(dc, sgi_pvchan_reset);
}

static const TypeInfo sgi_pvchan_type = {
    .name          = TYPE_SGI_PVCHAN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPvChanState),
    .class_init    = sgi_pvchan_class_init,
};

static void sgi_pvchan_register(void) {
    type_register_static(&sgi_pvchan_type);
}
type_init(sgi_pvchan_register);
