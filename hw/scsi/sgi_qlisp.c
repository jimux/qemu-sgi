/*
 * SGI QLogic ISP1020/1040 SCSI host adapter emulation.
 *
 * See include/hw/scsi/sgi_qlisp.h. Protocol from the ARCS ql.c driver.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/scsi/sgi_qlisp.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"

/*
 * Optional device-side trace of the request/status entries, for cross-checking
 * a driver's own request/sr_status on another platform.  Enable with
 * QLISP_DEBUG=1 and -d unimp.  Off by default.
 */
static bool qlisp_dbg(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("QLISP_DEBUG") != NULL;
    }
    return on;
}

/* mailbox register byte offsets indexed by number (mbox0..mbox7) */
static const uint8_t ql_mbox_off[8] = {
    QL_MBOX0, QL_MBOX1, QL_MBOX2, QL_MBOX3, QL_MBOX4, QL_MBOX5, QL_MBOX6,
    QL_MBOX7
};

static uint16_t ql_reg_get(SGIQLispState *s, hwaddr off)
{
    return s->reg[(off & (QLISP_REGS_SIZE - 1)) >> 1];
}

static void ql_reg_put(SGIQLispState *s, hwaddr off, uint16_t v)
{
    s->reg[(off & (QLISP_REGS_SIZE - 1)) >> 1] = v;
}

static uint16_t ql_mbox_get(SGIQLispState *s, int n)
{
    return ql_reg_get(s, ql_mbox_off[n]);
}

static void ql_mbox_put(SGIQLispState *s, int n, uint16_t v)
{
    ql_reg_put(s, ql_mbox_off[n], v);
}

/*
 * Drive the interrupt output line: asserted while the RISC-interrupt
 * condition is live (bus_isr RISC_INT set, or a mailbox command finished and
 * is awaiting the host ack).  The ARCS driver polls bus_isr and ignores this,
 * but the IRIX kernel ql driver is interrupt-driven on IP30.
 */
static void ql_update_irq(SGIQLispState *s)
{
    uint16_t isr = ql_reg_get(s, QL_BUS_ISR);
    int level;

    if (s->cmd_pending) {
        isr |= BUS_ISR_RISC_INT;
    }
    level = (isr & BUS_ISR_RISC_INT) ? 1 : 0;
    qemu_set_irq(s->irq, level);
}

/*
 * Command/status entries are byte-reversed per 32-bit word by the driver
 * (munge(), ql.c) so the little-endian RISC reads them correctly.  Un-munge
 * a copy to recover the host (big-endian) struct bytes; the CDB is byte-wise
 * correct in the raw image.
 */
static void ql_munge(uint8_t *p, int n)
{
    int i;

    for (i = 0; i + 4 <= n; i += 4) {
        uint8_t t0 = p[i], t1 = p[i + 1];
        p[i] = p[i + 3];
        p[i + 1] = p[i + 2];
        p[i + 2] = t1;
        p[i + 3] = t0;
    }
}

static uint32_t ql_ld32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static uint16_t ql_ld16(const uint8_t *p)
{
    return (p[0] << 8) | p[1];
}

/* Bridge 32-bit direct-mapped DMA address -> host physical address. */
/*
 * Translate a QLogic DMA address to host physical memory.
 *
 * IP30 (and the SN0 non-PCI64 path) use the Bridge 32-bit direct map, so the
 * address is phys + BRIDGE_DMA_DIRECT_BASE (0x80000000) and the high word is
 * zero. IP27's 64-bit SN0_PCI_64 get_pci64_dma_addr() instead returns a dirmap
 * address whose HIGH word is a PCI64 window selector (e.g. 0x15000000) and
 * whose LOW 32 bits are the physical address; with <=4GB RAM the low word is
 * the physical address, so mask it off distinguishably by the high word.
 */
static uint64_t ql_dma_to_phys(uint64_t a)
{
    if (a >> 32) {
        return a & 0xffffffffULL;
    }
    return a >= QL_DMA_DIRECT_BASE ? a - QL_DMA_DIRECT_BASE : a;
}

/* Read a 64-byte queue entry, returning the un-munged copy in `e`. */
static bool ql_get_entry(SGIQLispState *s, uint64_t addr, uint8_t *raw,
                         uint8_t *e)
{
    if (dma_memory_read(&address_space_memory, ql_dma_to_phys(addr), raw,
                        QL_ENTRY_SIZE,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    memcpy(e, raw, QL_ENTRY_SIZE);
    if (s->control_munge) {
        ql_munge(e, QL_ENTRY_SIZE);
    }
    return true;
}

/* base/count of dseg[i] from an un-munged command entry (A64 layout). */
static void ql_get_dseg(const uint8_t *e, unsigned base, unsigned i,
                        bool a64, QLSG *sg)
{
    if (a64) {
        unsigned off = base + i * 12;

        sg->addr = ((uint64_t)ql_ld32(e + off + 4) << 32) | ql_ld32(e + off);
        sg->len = ql_ld32(e + off + 8);
    } else {
        unsigned off = base + i * 8;

        sg->addr = ql_ld32(e + off);
        sg->len = ql_ld32(e + off + 4);
    }
}

static bool ql_sg_move(SGIQLispState *s, uint8_t *buf, uint32_t len,
                       bool to_host)
{
    while (len) {
        QLSG *g;
        uint32_t n;

        if (s->sg_idx >= s->nsg) {
            return false;   /* guest SG exhausted */
        }
        g = &s->sg[s->sg_idx];
        if (s->sg_off >= g->len) {
            s->sg_idx++;
            s->sg_off = 0;
            continue;
        }
        n = MIN(len, g->len - s->sg_off);
        if (to_host) {
            address_space_write(&address_space_memory,
                                ql_dma_to_phys(g->addr) + s->sg_off,
                                MEMTXATTRS_UNSPECIFIED, buf, n);
        } else {
            address_space_read(&address_space_memory,
                               ql_dma_to_phys(g->addr) + s->sg_off,
                               MEMTXATTRS_UNSPECIFIED, buf, n);
        }
        s->sg_off += n;
        buf += n;
        len -= n;
    }
    return true;
}

/* Post a status entry to the response ring; raise the RISC interrupt. */
static void ql_write_status(SGIQLispState *s, uint16_t completion,
                            uint16_t scsi_status, uint32_t residual,
                            const uint8_t *sense, uint32_t sense_len)
{
    uint8_t st[QL_ENTRY_SIZE];

    if (!s->rsp.count) {
        return;
    }
    memset(st, 0, sizeof(st));
    st[2] = 1;                       /* entry_cnt */
    st[3] = QL_ET_STATUS;
    stl_be_p(st + 0x04, s->cur_handle);
    stw_be_p(st + 0x08, completion);
    stw_be_p(st + 0x0a, scsi_status);
    stw_be_p(st + 0x0c, 0);          /* status_flags */
    stw_be_p(st + 0x0e, QL_SS_GOT_STATUS | QL_SS_TRANSFER_COMPLETE);
    stw_be_p(st + 0x10, sense_len);
    stw_be_p(st + 0x12, 0);          /* time */
    stl_be_p(st + 0x14, residual);
    if (sense && sense_len) {
        memcpy(st + 0x20, sense, MIN(sense_len, 32));
    }

    if (s->control_munge) {
        ql_munge(st, sizeof(st));
    }
    dma_memory_write(&address_space_memory,
                     ql_dma_to_phys(s->rsp.base) +
                         (uint64_t)s->rsp.in * QL_ENTRY_SIZE,
                     st, QL_ENTRY_SIZE, MEMTXATTRS_UNSPECIFIED);

    s->rsp.in = (s->rsp.in + 1) % s->rsp.count;
    ql_mbox_put(s, 5, s->rsp.in);
    ql_reg_put(s, QL_BUS_ISR,
               ql_reg_get(s, QL_BUS_ISR) | BUS_ISR_RISC_INT);
    ql_update_irq(s);

    if (qlisp_dbg()) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi-qlisp: STS bus=%d handle=%u comp=0x%x scsi=0x%x "
                      "resid=%u sense=%u slot=%u rsp_base=0x%llx mbox5=%u\n",
                      s->busnr, s->cur_handle, completion, scsi_status, residual,
                      sense_len, (s->rsp.in + s->rsp.count - 1) % s->rsp.count,
                      (unsigned long long)s->rsp.base, s->rsp.in);
    }
}

static void ql_process_requests(SGIQLispState *s);

static void ql_scsi_transfer_data(SCSIRequest *req, uint32_t len)
{
    SGIQLispState *s = req->hba_private;
    uint8_t *buf = scsi_req_get_buf(req);
    bool to_host = req->cmd.mode == SCSI_XFER_FROM_DEV;

    /*
     * The data stream is word-reversed by the host driver on machines that do
     * not define SWAP_DATA_STREAM (i.e. not IP30).  That is the same split as
     * the control-entry munge: when control_munge is clear (IP27), the driver
     * munges the data too, so the device must pre-munge what it hands over and
     * un-munge what it picks up.  ql_munge is its own inverse, so applying it
     * symmetrically in both directions is correct.
     */
    if (getenv("QLISP_DEBUG") && req->cmd.buf[0] == 0x12) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi-qlisp: INQ len=%u buf=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                      len, buf[0], buf[1], buf[2], buf[3],
                      buf[4], buf[5], buf[6], buf[7]);
    }
    if (!s->control_munge) {
        ql_munge(buf, len);
    }
    ql_sg_move(s, buf, len, to_host);
    scsi_req_continue(req);
}

static void ql_scsi_command_complete(SCSIRequest *req, size_t residual)
{
    SGIQLispState *s = req->hba_private;

    s->cur_req = NULL;
    ql_write_status(s, QL_SCS_COMPLETE, req->status, residual,
                    req->sense, req->sense_len);
    scsi_req_unref(req);
    ql_process_requests(s);
}

static void ql_scsi_request_cancelled(SCSIRequest *req)
{
    SGIQLispState *s = req->hba_private;

    if (s->cur_req == req) {
        s->cur_req = NULL;
        scsi_req_unref(req);
    }
}

static const SCSIBusInfo qlisp_scsi_info = {
    .tcq = false,
    .max_target = 16,
    .max_lun = 8,
    .transfer_data = ql_scsi_transfer_data,
    .complete = ql_scsi_command_complete,
    .cancel = ql_scsi_request_cancelled,
};

/*
 * Fetch and execute request-queue entries from the RISC's out-pointer up to
 * the host's in-pointer (mailbox4).  Commands run one at a time: a command
 * that reaches the SCSI layer stops the scan and is resumed from its
 * completion callback; absent targets complete inline with a transport error.
 */
static void ql_process_requests(SGIQLispState *s)
{
    uint32_t in;
    static unsigned dbg;

    if (dbg < 16) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi-qlisp: process fw=%d cnt=%u base=0x%llx out=%u "
                      "in=%u cur=%p\n",
                      s->firmware_running, s->req.count,
                      (unsigned long long)s->req.base, s->req.out,
                      s->req.count ? ql_mbox_get(s, 4) % s->req.count : 0,
                      (void *)s->cur_req);
        dbg++;
    }

    if (!s->firmware_running || s->cur_req || !s->req.count) {
        return;
    }
    in = ql_mbox_get(s, 4) % s->req.count;

    while (s->req.out != in) {
        uint8_t raw[QL_ENTRY_SIZE], e[QL_ENTRY_SIZE];
        uint64_t eaddr = s->req.base +
                         (uint64_t)s->req.out * QL_ENTRY_SIZE;
        uint8_t cdb[16];
        uint32_t i, cdb_len, seg_cnt, ncont;
        uint8_t etype;
        bool a64;
        unsigned iocb_segs, cont_segs, cmd_dseg, cont_dseg;
        SCSIDevice *sdev;
        int datalen;

        if (!ql_get_entry(s, eaddr, raw, e)) {
            return;
        }
        etype = e[3];
        if (etype == QL_ET_MARKER) {
            s->req.out = (s->req.out + 1) % s->req.count;
            continue;
        }
        if (etype != QL_ET_COMMAND && etype != QL_ET_COMMAND_LEGACY) {
            /* Not a command at the head: stop rather than walk garbage. */
            qemu_log_mask(LOG_UNIMP,
                          "sgi-qlisp: ring head entry_type=0x%x (out=%u in=%u)\n",
                          etype, s->req.out, in);
            return;
        }
        a64 = (etype == QL_ET_COMMAND);
        iocb_segs = a64 ? QL_IOCB_SEGS_A64 : QL_IOCB_SEGS_LEGACY;
        cont_segs = a64 ? QL_CONT_SEGS_A64 : QL_CONT_SEGS_LEGACY;
        cmd_dseg = a64 ? 0x28 : 0x20;
        cont_dseg = a64 ? 0x04 : 0x08;

        cdb_len = ql_ld16(e + 0x08);
        if (cdb_len == 0 || cdb_len > sizeof(cdb)) {
            cdb_len = sizeof(cdb);
        }
        /*
         * The CDB is stored per-32-bit-word reversed in the entry (its words
         * are byte-swapped independently of the control munge). Reading from
         * the already-un-munged copy `e` and reversing each word recovers the
         * natural CDB: for IP30 `e` is un-munged so this reproduces the raw
         * bytes; for IP27 `e` is raw so this undoes the CDB's word reversal.
         * The CDB field is 4-byte aligned, so reverse the whole 16-byte field
         * and then take cdb_len (which may not be a multiple of 4).
         */
        memcpy(cdb, e + 0x14, sizeof(cdb));
        ql_munge(cdb, sizeof(cdb));
        if (cdb_len < sizeof(cdb)) {
            memset(cdb + cdb_len, 0, sizeof(cdb) - cdb_len);
        }
        seg_cnt = ql_ld16(e + 0x10);

        s->cur_handle = ql_ld32(e + 0x04);

        if (qlisp_dbg()) {
            qemu_log_mask(LOG_UNIMP,
                          "sgi-qlisp: CMD bus=%d out=%u in=%u etype=0x%x "
                          "handle=%u tgt=%u lun=%u cdb_len=%u seg=%u "
                          "cdb=%02x%02x%02x%02x%02x%02x\n",
                          s->busnr, s->req.out, in, etype, s->cur_handle,
                          e[0x0a], e[0x0b], cdb_len, seg_cnt, cdb[0], cdb[1],
                          cdb[2], cdb[3], cdb[4], cdb[5]);
        }
        s->nsg = 0;
        s->sg_idx = 0;
        s->sg_off = 0;
        for (i = 0; i < seg_cnt && s->nsg < QL_MAX_SG; i++) {
            if (i < iocb_segs) {
                ql_get_dseg(e, cmd_dseg, i, a64, &s->sg[s->nsg++]);
            } else {
                uint32_t ci = (i - iocb_segs) / cont_segs;
                uint32_t cj = (i - iocb_segs) % cont_segs;
                uint32_t slot = (s->req.out + 1 + ci) % s->req.count;
                uint8_t craw[QL_ENTRY_SIZE], ce[QL_ENTRY_SIZE];
                if (!ql_get_entry(s, s->req.base +
                                     (uint64_t)slot * QL_ENTRY_SIZE,
                                  craw, ce)) {
                    break;
                }
                ql_get_dseg(ce, cont_dseg, cj, a64, &s->sg[s->nsg++]);
            }
        }
        /* Each continuation entry holds cont_segs dsegs. */
        ncont = seg_cnt > iocb_segs ?
                (seg_cnt - iocb_segs + cont_segs - 1) / cont_segs : 0;
        s->req.out = (s->req.out + 1 + ncont) % s->req.count;

        sdev = scsi_device_find(&s->bus, 0, e[0x0a], e[0x0b]);
        if (!sdev) {
            ql_write_status(s, QL_SCS_TRANSPORT_ERROR, 0, 0, NULL, 0);
            continue;
        }
        s->cur_req = scsi_req_new(sdev, 0, e[0x0b], cdb, cdb_len, s);
        datalen = scsi_req_enqueue(s->cur_req);
        if (datalen != 0) {
            scsi_req_continue(s->cur_req);
        }
        return;   /* resume from ql_scsi_command_complete() */
    }
}

/*
 * Process a mailbox command when the host writes HCCR_CMD_SET_HOST_INT.
 * The ARCS driver (WAIT_SEMA path) then polls bus_sema for LOCK (which we
 * set here), reads mailbox0 as the status, then writes bus_sema=0 to release.
 */
static void ql_do_mbox_cmd(SGIQLispState *s)
{
    uint16_t cmd = ql_mbox_get(s, 0);
    uint16_t sts = MBOX_STS_COMMAND_COMPLETE;
    uint64_t base;
    uint32_t depth, idx;

    switch (cmd) {
    case MBOX_CMD_NOP:
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_MAILBOX_REGISTER_TEST:
        /* host wrote 0x1111..0x6666 into mbox1..6; leave them (echo) */
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_LOAD_RAM:
        /*
         * DMA the firmware image from host physical memory into the RISC's
         * instruction RAM. mbox1 = RISC start, mbox2/3 = host ptr hi/lo,
         * mbox4 = length in words. The driver's DMA copy is pairwise swapped
         * (SWAP_DATA_STREAM), so swap adjacent 16-bit words back to recover
         * the logical firmware image.
         */
        {
            uint32_t addr = ql_mbox_get(s, 1);
            uint64_t host = ((uint64_t)ql_mbox_get(s, 2) << 16) |
                            ql_mbox_get(s, 3);
            uint32_t len = ql_mbox_get(s, 4);
            uint32_t rambase = addr - QL_RISC_RAMBASE;
            uint32_t i;
            uint16_t buf[0x8000];

            if (len > ARRAY_SIZE(buf)) {
                len = ARRAY_SIZE(buf);
            }
            if (len && rambase + len <= ARRAY_SIZE(s->risc_ram) &&
                dma_memory_read(&address_space_memory,
                                ql_dma_to_phys(host), buf,
                                len * 2, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                for (i = 0; i < len; i++) {
                    s->risc_ram[rambase + i] = buf[i ^ 1];
                }
                s->risc_loaded = len;
            }
        }
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_WRITE_RAM_WORD:
        /* mbox1 = RISC addr, mbox2 = data */
        {
            uint32_t a = ql_mbox_get(s, 1) - QL_RISC_RAMBASE;
            if (a < ARRAY_SIZE(s->risc_ram)) {
                s->risc_ram[a] = ql_mbox_get(s, 2);
                if (a + 1 > s->risc_loaded) {
                    s->risc_loaded = a + 1;
                }
            }
        }
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_READ_RAM_WORD:
        /* mbox1 = RISC addr; result returned in mbox2 */
        {
            uint32_t a = ql_mbox_get(s, 1) - QL_RISC_RAMBASE;
            ql_mbox_put(s, 2, (a < ARRAY_SIZE(s->risc_ram)) ?
                               s->risc_ram[a] : 0);
        }
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_VERIFY_CHECKSUM:
        /* mbox1 = RISC addr; the ISP's 16-bit sum is returned in mbox2,
         * matching the driver's simple sum over the loaded firmware. */
        {
            uint32_t n = s->risc_loaded ? s->risc_loaded : 0;
            uint32_t i;
            uint16_t sum = 0;
            for (i = 0; i < n && i < ARRAY_SIZE(s->risc_ram); i++) {
                sum += s->risc_ram[i];
            }
            ql_mbox_put(s, 2, sum);
        }
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_EXECUTE_FIRMWARE:
        s->firmware_running = true;
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_ABOUT_FIRMWARE:
        ql_mbox_put(s, 1, 3);
        ql_mbox_put(s, 2, 0x37);
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_INIT_REQUEST_QUEUE:
        depth = ql_mbox_get(s, 1);
        base = ((uint64_t)ql_mbox_get(s, 2) << 16) | ql_mbox_get(s, 3);
        idx = ql_mbox_get(s, 4);
        s->req.base = base;
        s->req.count = depth;
        s->req.in = idx;
        s->req.out = idx;
        ql_mbox_put(s, 5, idx);
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_INIT_RESPONSE_QUEUE:
        depth = ql_mbox_get(s, 1);
        base = ((uint64_t)ql_mbox_get(s, 2) << 16) | ql_mbox_get(s, 3);
        idx = ql_mbox_get(s, 5);
        s->rsp.base = base;
        s->rsp.count = depth;
        s->rsp.out = idx;
        s->rsp.in = 0;
        ql_mbox_put(s, 5, idx);
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_INIT_REQUEST_QUEUE_64:
    case MBOX_CMD_INIT_RESPONSE_QUEUE_64:
        depth = ql_mbox_get(s, 1);
        base = ((uint64_t)ql_mbox_get(s, 6) << 48) |
               ((uint64_t)ql_mbox_get(s, 7) << 32) |
               ((uint64_t)ql_mbox_get(s, 2) << 16) | ql_mbox_get(s, 3);
        idx = ql_mbox_get(s, 5);
        if (cmd == MBOX_CMD_INIT_REQUEST_QUEUE_64) {
            s->req.base = base;
            s->req.count = depth;
            s->req.in = idx;
            s->req.out = idx;
        } else {
            s->rsp.base = base;
            s->rsp.count = depth;
            s->rsp.out = idx;
            s->rsp.in = 0;
        }
        ql_mbox_put(s, 5, idx);
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    case MBOX_CMD_GET_FIRMWARE_STATUS:
        ql_mbox_put(s, 1, 0);
        break;
    case MBOX_CMD_GET_INITIATOR_ID:
        ql_mbox_put(s, 1, 7);
        break;
    case MBOX_CMD_GET_SELECTION_TIMEOUT:
        ql_mbox_put(s, 1, 250);
        break;
    case MBOX_CMD_GET_RETRY_COUNT:
        ql_mbox_put(s, 1, 2);
        break;
    case MBOX_CMD_GET_TAG_AGE_LIMIT:
        ql_mbox_put(s, 1, 8);
        break;
    case MBOX_CMD_GET_CLOCK_RATE:
        ql_mbox_put(s, 1, 40);
        break;
    case MBOX_CMD_GET_ACTIVE_NEG:
        ql_mbox_put(s, 1, 0x00);
        break;
    case MBOX_CMD_GET_ASYNC_DATA_SETUP:
        ql_mbox_put(s, 1, 0);
        break;
    case MBOX_CMD_GET_BUS_CONTROL:
        ql_mbox_put(s, 1, 0);
        break;
    case MBOX_CMD_GET_TARGET_PARAMETERS:
        ql_mbox_put(s, 1, 0);
        break;
    case MBOX_CMD_GET_DEVICE_QUEUE_PARAM:
        ql_mbox_put(s, 1, 0x20);
        break;

    case MBOX_CMD_SET_INITIATOR_ID:
    case MBOX_CMD_SET_SELECTION_TIMEOUT:
    case MBOX_CMD_SET_RETRY_COUNT:
    case MBOX_CMD_SET_TAG_AGE_LIMIT:
    case MBOX_CMD_SET_CLOCK_RATE:
    case MBOX_CMD_SET_ACTIVE_NEG:
    case MBOX_CMD_SET_ASYNC_DATA_SETUP:
    case MBOX_CMD_SET_BUS_CONTROL:
    case MBOX_CMD_SET_TARGET_PARAMETERS:
    case MBOX_CMD_SET_DEVICE_QUEUE_PARAM:
    case MBOX_CMD_WAKE_UP:
    case MBOX_CMD_STOP_FIRMWARE:
    case MBOX_CMD_ABORT:
    case MBOX_CMD_ABORT_DEVICE:
    case MBOX_CMD_ABORT_TARGET:
    case MBOX_CMD_BUS_RESET:
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "sgi-qlisp: unhandled mbox cmd 0x%x\n", cmd);
        sts = MBOX_STS_COMMAND_COMPLETE;
        break;
    }

    ql_mbox_put(s, 0, sts);
    {
        static unsigned long dbg_cnt;
        qemu_log_mask(LOG_UNIMP,
                      "sgi-qlisp: #%lu mbox cmd=0x%x in=%d sts=0x%x\n",
                      dbg_cnt++, cmd, ql_mbox_get(s, 1), sts);
    }
}

static uint64_t qlisp_read(void *opaque, hwaddr off, unsigned size)
{
    SGIQLispState *s = opaque;
    uint64_t v = 0;
    int i;

    if (size == 2) {
        switch (off & ~1) {
        case QL_BUS_ID_LOW:
            /* byte-lane swapped: mem 0x00 is the device id, 0x02 the vendor */
            return QLISP_DEVICE;
        case QL_BUS_ID_HIGH:
            return QLISP_VENDOR;
        case QL_BUS_SEMA:
            /* RISC owns the semaphore while a mailbox reply is unacked. */
            return s->cmd_pending ? BUS_SEMA_LOCK : ql_reg_get(s, off);
        case QL_BUS_ISR:
            /*
             * The RISC services the request ring continuously; the host's
             * ql_poll() reads this register in its wait loop.  Run queued
             * requests here (unless a mailbox command is in flight, whose
             * own poll also reads this register).
             */
            {
                static unsigned dbg_isr;
                if (s->firmware_running && dbg_isr < 16) {
                    qemu_log_mask(LOG_UNIMP,
                                  "sgi-qlisp: rd bus_isr pending=%d fw=%d "
                                  "cnt=%u out=%u mbox4=%u\n",
                                  s->cmd_pending, s->firmware_running,
                                  s->req.count, s->req.out,
                                  ql_mbox_get(s, 4));
                    dbg_isr++;
                }
            }
            if (!s->cmd_pending) {
                ql_process_requests(s);
            }
            return ql_reg_get(s, off) |
                   (s->cmd_pending ? BUS_ISR_RISC_INT : 0);
        default:
            return ql_reg_get(s, off);
        }
    }
    for (i = 0; i < size; i++) {
        v = (v << 8) | ((ql_reg_get(s, off + i) >> ((i & 1) ? 0 : 8)) & 0xff);
    }
    return v;
}

static void qlisp_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SGIQLispState *s = opaque;
    int i;

    if (size != 2) {
        for (i = 0; i < size; i++) {
            hwaddr o = off + i;
            int sh = (o & 1) ? 0 : 8;
            uint16_t cur = ql_reg_get(s, o);
            cur = (cur & ~(0xff << sh)) |
                  (((val >> (8 * (size - 1 - i))) & 0xff) << sh);
            s->reg[(o & (QLISP_REGS_SIZE - 1)) >> 1] = cur;
        }
        return;
    }

    switch (off & ~1) {
    case QL_BUS_SEMA:
    case QL_NVRAM:
        ql_reg_put(s, off, val & 0xffff);
        if (val == 0) {
            s->cmd_pending = false;      /* host releases the semaphore */
            ql_reg_put(s, QL_BUS_ISR, 0);
        }
        break;
    case QL_BUS_ICR:
        ql_reg_put(s, QL_BUS_ICR, val & 0xffff);
        break;
    case QL_MBOX5:
        /* host acks consumed responses by writing response_out */
        ql_reg_put(s, off, val & 0xffff);
        if (s->rsp.count) {
            s->rsp.out = val % s->rsp.count;
        }
        break;
    case QL_MBOX4:
        /*
         * Host publishes the request-queue in-pointer; the real RISC fetches
         * the new entries and posts responses (raising the interrupt).  We
         * process eagerly here so an interrupt-driven driver (the IRIX kernel
         * ql module) completes without polling bus_isr first.
         */
        ql_reg_put(s, off, val & 0xffff);
        ql_process_requests(s);
        break;
    case QL_HCCR:
    case 0xc0:
        switch (val & 0xf000) {
        case HCCR_CMD_SET_HOST_INT:
            ql_do_mbox_cmd(s);
            s->cmd_pending = true;
            break;
        case HCCR_CMD_RESET:
            s->firmware_running = false;
            ql_reg_put(s, QL_HCCR, HCCR_RESET);
            break;
        case HCCR_CMD_RELEASE:
            ql_reg_put(s, QL_HCCR, 0);
            break;
        case HCCR_CMD_PAUSE:
            ql_reg_put(s, QL_HCCR, HCCR_PAUSE);
            break;
        case HCCR_CMD_CLEAR_RISC_INT:
            s->cmd_pending = false;
            ql_reg_put(s, QL_BUS_ISR, 0);
            ql_reg_put(s, QL_HCCR, 0);
            break;
        default:
            ql_reg_put(s, QL_HCCR, val & 0xffff);
            break;
        }
        break;
    default:
        ql_reg_put(s, off, val & 0xffff);
        break;
    }
    ql_update_irq(s);
}

static const MemoryRegionOps qlisp_ops = {
    .read = qlisp_read,
    .write = qlisp_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void qlisp_reset(DeviceState *dev)
{
    SGIQLispState *s = SGI_QLISP(dev);

    memset(s->reg, 0, sizeof(s->reg));
    memset(s->risc_ram, 0, sizeof(s->risc_ram));
    s->firmware_running = false;
    s->pci_cmd = 0;
    s->cur_req = NULL;
    s->nsg = 0;
    s->sg_idx = 0;
    s->sg_off = 0;
    memset(&s->req, 0, sizeof(s->req));
    memset(&s->rsp, 0, sizeof(s->rsp));
    ql_update_irq(s);
}

static void qlisp_realize(DeviceState *dev, Error **errp)
{
    SGIQLispState *s = SGI_QLISP(dev);

    memory_region_init_io(&s->regs, OBJECT(dev), &qlisp_ops, s,
                          "sgi-qlisp-regs", QLISP_REGS_SIZE);
    qdev_init_gpio_out(dev, &s->irq, 1);
    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &qlisp_scsi_info);
    s->bus.busnr = s->busnr;
    scsi_bus_legacy_handle_cmdline(&s->bus);
}

static const Property qlisp_props[] = {
    DEFINE_PROP_DRIVE("drive", SGIQLispState, blk),
    DEFINE_PROP_UINT8("revision", SGIQLispState, pci_rev, QLISP_REV),
    DEFINE_PROP_UINT32("scsi-bus-num", SGIQLispState, busnr, 0),
    /*
     * True for machines whose ARCS driver munges control entries (IP30);
     * false (default) for SN0/IP27, which leave them in natural order.
     */
    DEFINE_PROP_BOOL("control-munge", SGIQLispState, control_munge, false),
};

static void qlisp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = qlisp_realize;
    device_class_set_legacy_reset(dc, qlisp_reset);
    device_class_set_props(dc, qlisp_props);
}

static const TypeInfo qlisp_info = {
    .name = TYPE_SGI_QLISP,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SGIQLispState),
    .class_init = qlisp_class_init,
};

static void qlisp_register_types(void)
{
    type_register_static(&qlisp_info);
}

type_init(qlisp_register_types)
