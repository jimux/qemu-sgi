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
#include "hw/scsi/sgi_qlisp.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

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
            uint32_t base = addr - QL_RISC_RAMBASE;
            uint32_t i;
            uint16_t buf[0x8000];

            if (len > ARRAY_SIZE(buf)) {
                len = ARRAY_SIZE(buf);
            }
            if (len && base + len <= ARRAY_SIZE(s->risc_ram) &&
                dma_memory_read(&address_space_memory, host, buf,
                                len * 2, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                for (i = 0; i < len; i++) {
                    s->risc_ram[base + i] = buf[i ^ 1];
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
    memset(&s->req, 0, sizeof(s->req));
    memset(&s->rsp, 0, sizeof(s->rsp));
}

static void qlisp_realize(DeviceState *dev, Error **errp)
{
    SGIQLispState *s = SGI_QLISP(dev);

    memory_region_init_io(&s->regs, OBJECT(dev), &qlisp_ops, s,
                          "sgi-qlisp-regs", QLISP_REGS_SIZE);
}

static const Property qlisp_props[] = {
    DEFINE_PROP_DRIVE("drive", SGIQLispState, blk),
    DEFINE_PROP_UINT8("revision", SGIQLispState, pci_rev, QLISP_REV),
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
