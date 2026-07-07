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
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"

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

/* ---- host-side helpers (called from Python via QMP/HMP or monitor) -------- */

/* Read one message from the G2H ring (host is consumer).  Returns payload
 * length (>=0) on success, 0 if no complete message is ready, -1 on error. */
int pvchan_host_read_msg(SGIPvChanState *s, uint32_t *op, uint32_t *status,
                         uint8_t *buf, uint32_t max)
{
    uint64_t base = s->g2h_base & 0x1FFFFFFFULL;
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
    uint64_t base = s->h2g_base & 0x1FFFFFFFULL;
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

/* ---- device lifecycle ---------------------------------------------------- */

static const Property sgi_pvchan_props[] = {
    DEFINE_PROP_INT32("notify-fd", SGIPvChanState, notify_fd, -1),
};

static void sgi_pvchan_realize(DeviceState *dev, Error **errp)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvchan_ops, s,
                          "sgi-pvchan", SGI_PVCHAN_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->status = PVCHAN_S_READY;
    pvchan_instance = s;
}

static void sgi_pvchan_reset(DeviceState *dev)
{
    SGIPvChanState *s = SGI_PVCHAN(dev);
    s->h2g_base = s->h2g_len = s->h2g_write = s->h2g_read = 0;
    s->g2h_base = s->g2h_len = s->g2h_write = s->g2h_read = 0;
    s->h2g_dbell = s->g2h_dbell = 0;
    s->irq_enable = 0;
    s->status = PVCHAN_S_READY;
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
