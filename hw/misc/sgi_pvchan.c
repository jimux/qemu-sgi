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

/* ---- ring helpers (operate on guest RAM) --------------------------------- */
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

static void rd_buf(SGIPvChanState *s, uint64_t pa, void *dst, uint32_t n)
{
    dma_memory_read(&address_space_memory, pa, dst, n, MEMTXATTRS_UNSPECIFIED);
}

static void wr_buf(SGIPvChanState *s, uint64_t pa, const void *src, uint32_t n)
{
    dma_memory_write(&address_space_memory, pa, src, n, MEMTXATTRS_UNSPECIFIED);
}

static void ring_read_bytes(SGIPvChanState *s, uint64_t base, uint32_t len,
                            uint32_t *prod, uint32_t *cons,
                            uint8_t *out, uint32_t want, uint32_t *got)
{
    uint32_t pi = rd32_le(s, base);
    uint32_t ci = rd32_le(s, base + 4);
    uint32_t avail, data_off = base + PVCHAN_RING_HDR_SZ;
    *cons = ci; *prod = pi; *got = 0;
    if (pi <= ci) pi += len;
    avail = pi - ci;
    if (avail > len * 2) avail = 0; /* desync guard */
    if (avail < 4 || want == 0) return;
    want = MIN(want, avail);
    /* linear read if within ring, or two-part wrap */
    uint32_t off = ci % len;
    uint32_t chunk = len - off;
    if (chunk >= want) {
        rd_buf(s, data_off + off, out, want);
    } else {
        rd_buf(s, data_off + off, out, chunk);
        rd_buf(s, data_off, out + chunk, want - chunk);
    }
    *got = want;
}

static void ring_consume(SGIPvChanState *s, uint64_t base, uint32_t len,
                         uint32_t ci, uint32_t bytes)
{
    wr32_le(s, base + 4, (ci + bytes) % (len * 2));
}

static void ring_produce(SGIPvChanState *s, uint64_t base, uint32_t len,
                         uint32_t pi, const uint8_t *data, uint32_t bytes)
{
    uint32_t data_off = base + PVCHAN_RING_HDR_SZ;
    uint32_t off = pi % len;
    uint32_t chunk = len - off;
    if (chunk >= bytes) {
        wr_buf(s, data_off + off, data, bytes);
    } else {
        wr_buf(s, data_off + off, data, chunk);
        wr_buf(s, data_off, data + chunk, bytes - chunk);
    }
    wr32_le(s, base, pi + bytes);
}

/* ---- host-side helpers (called from Python via QMP/HMP or monitor) -------- */

int pvchan_host_read_msg(SGIPvChanState *s, uint32_t *op, uint32_t *status,
                         uint8_t *buf, uint32_t max)
{
    uint64_t base = s->g2h_base & 0x1FFFFFFFULL;
    uint32_t len  = s->g2h_len;
    uint32_t pi, ci, got;
    uint32_t hdr[3]; /* op, status, plen — LE */
    if (!base || !len || len > (64 << 20)) return -1;
    ring_read_bytes(s, base, len, &pi, &ci, (uint8_t *)hdr, 12, &got);
    if (got < 12) return 0;
    uint32_t plen = le32_to_cpu(hdr[2]);
    if (plen > PVCHAN_MAX_MSG || plen > max) { ring_consume(s, base, len, ci, 12); return -1; }
    *op = le32_to_cpu(hdr[0]);
    *status = le32_to_cpu(hdr[1]);
    if (plen > 0) {
        uint32_t g2;
        ring_read_bytes(s, base, len, &pi, &ci, buf, plen, &g2);
        if (g2 < plen) { ring_consume(s, base, len, ci, plen + 12); return -1; }
    }
    ring_consume(s, base, len, ci, plen + 12);
    s->g2h_read = ci + plen + 12;
    return (int)plen;
}

int pvchan_host_write_msg(SGIPvChanState *s, uint32_t op,
                          const uint8_t *payload, uint32_t len)
{
    uint64_t base = s->h2g_base & 0x1FFFFFFFULL;
    uint32_t rlen = s->h2g_len;
    uint32_t pi, ci, space, total;
    if (!base || !rlen || rlen > (64 << 20) || len > PVCHAN_MAX_MSG) return -1;
    pi = rd32_le(s, base);
    ci = rd32_le(s, base + 4);
    if (pi < ci) pi += rlen;
    space = rlen - (pi - ci);
    total = 12 + len;
    if (space < total + 4) return -2; /* no room */
    uint8_t hdr[12];
    *(uint32_t *)(hdr + 0) = cpu_to_le32(op);
    *(uint32_t *)(hdr + 4) = cpu_to_le32(0);
    *(uint32_t *)(hdr + 8) = cpu_to_le32(len);
    uint32_t wpos = pi % rlen;
    ring_produce(s, base, rlen, wpos, hdr, 12);
    if (len > 0) ring_produce(s, base, rlen, wpos + 12, payload, len);
    s->h2g_write = wpos + 12 + len;
    /* kick guest IRQ */
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
    const char *hex = qdict_get_str(qdict, "data");
    uint8_t payload[PVCHAN_MAX_MSG];
    int plen;
    SGIPvChanState *s;
    int rc;

    if (!pvchan_instance) {
        monitor_printf(mon, "pvchan: no device (virtuix not running?)\n");
        return;
    }
    s = pvchan_instance;
    plen = hex_decode(hex, payload, sizeof(payload));
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
    if (plen < 0) {
        monitor_printf(mon, "pvchan-recv: error %d\n", plen);
    } else if (plen == 0) {
        monitor_printf(mon, "pvchan-recv: no message ready\n");
    } else {
        monitor_printf(mon, "pvchan-recv: op=%u status=%u plen=%d\n", op, status, plen);
        if (plen > 0) {
            /* print hex + ASCII preview */
            monitor_printf(mon, "  hex: ");
            for (int i = 0; i < plen && i < 256; i++) {
                monitor_printf(mon, "%02x", buf[i]);
            }
            if (plen > 256) monitor_printf(mon, "...");
            monitor_printf(mon, "\n");
            /* try to print as text if it looks like ASCII */
            bool printable = true;
            for (int i = 0; i < plen && i < 2048; i++) {
                if (buf[i] == 0) { /* NUL — truncate display, not error */
                    break;
                }
                if (buf[i] < 0x20 && buf[i] != '\n' && buf[i] != '\r' && buf[i] != '\t') {
                    printable = false;
                    break;
                }
            }
            if (printable) {
                monitor_printf(mon, "  text: %.*s\n", plen, (const char *)buf);
            }
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
