/*
 * SGI QLogic ISP1020/1040 SCSI host adapter emulation.
 *
 * The ISP1020 is a PCI SCSI controller used on SGI IP30 (Octane) BaseIO
 * (two channels) and other machines. This model implements the register
 * file, the mailbox/RISC command interface, the request/response queues and
 * SCSI command execution against a QEMU block backend.
 *
 * Register layout / protocol from the ARCS standalone driver (ql.c) and
 * sys/ql_standalone.h, sys/ql_firmware_standalone.h.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SCSI_SGI_QLISP_H
#define HW_SCSI_SGI_QLISP_H

#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "hw/scsi/scsi.h"
#include "qom/object.h"
#include "block/block.h"
#include "system/dma.h"

#define TYPE_SGI_QLISP "sgi-qlisp"
OBJECT_DECLARE_SIMPLE_TYPE(SGIQLispState, SGI_QLISP)

#define QLISP_REGS_SIZE   0x100

/* PCI identity */
#define QLISP_VENDOR      0x1077
#define QLISP_DEVICE      0x1020
#define QLISP_REV         0x02
#define QLISP_CLASS       0x010000   /* SCSI controller */

/* Register byte offsets. NOTE: these are the offsets the ARCS driver actually
 * touches (verified from the access trace); the ql_standalone.h struct comments
 * are +2 off from the on-wire byte offsets for several registers. */
#define QL_BUS_ID_LOW     0x00
#define QL_BUS_ID_HIGH    0x02
#define QL_BUS_CONFIG0    0x04
#define QL_BUS_CONFIG1    0x06
#define QL_BUS_ISR        0x08
#define QL_BUS_ICR        0x0a
#define QL_NVRAM          0x0c
#define QL_BUS_SEMA       0x0e
#define QL_MBOX0          0x72
#define QL_MBOX1          0x70
#define QL_MBOX2          0x76
#define QL_MBOX3          0x74
#define QL_MBOX4          0x7a
#define QL_MBOX5          0x78
#define QL_MBOX6          0x7e
#define QL_MBOX7          0x7c
#define QL_HCCR           0xc2

/* HCCR commands (written) */
#define HCCR_CMD_RESET          0x1000
#define HCCR_CMD_PAUSE          0x2000
#define HCCR_CMD_RELEASE        0x3000
#define HCCR_CMD_SET_HOST_INT   0x5000
#define HCCR_CMD_CLEAR_RISC_INT 0x7000
/* HCCR status (read) */
#define HCCR_HOST_INT           0x0080
#define HCCR_RESET              0x0040
#define HCCR_PAUSE              0x0020

#define BUS_ISR_RISC_INT        0x0004
#define BUS_SEMA_LOCK           0x0001

/* Mailbox command/status (ql_standalone.h) */
#define MBOX_CMD_NOP                    0x0000
#define MBOX_CMD_LOAD_RAM               0x0001
#define MBOX_CMD_EXECUTE_FIRMWARE       0x0002
#define MBOX_CMD_DUMP_RAM               0x0003
#define MBOX_CMD_WRITE_RAM_WORD         0x0004
#define MBOX_CMD_READ_RAM_WORD          0x0005
#define MBOX_CMD_MAILBOX_REGISTER_TEST  0x0006
#define MBOX_CMD_VERIFY_CHECKSUM        0x0007
#define MBOX_CMD_ABOUT_FIRMWARE         0x0008
#define MBOX_CMD_INIT_REQUEST_QUEUE     0x0010
#define MBOX_CMD_INIT_RESPONSE_QUEUE    0x0011
/*
 * 64-bit queue-init commands, used by the SN0 (Origin/IP27) build of the
 * driver instead of 0x10/0x11. The base is assembled from four mailboxes:
 *   mbox2 = base>>16, mbox3 = base&0xffff, mbox7 = base>>32, mbox6 = base>>48
 * and the in/out pointer is in mbox5 for both directions.
 */
#define MBOX_CMD_INIT_REQUEST_QUEUE_64  0x0052
#define MBOX_CMD_INIT_RESPONSE_QUEUE_64 0x0053
#define MBOX_CMD_WAKE_UP                0x0013
#define MBOX_CMD_STOP_FIRMWARE          0x0014
#define MBOX_CMD_ABORT                  0x0015
#define MBOX_CMD_ABORT_DEVICE           0x0016
#define MBOX_CMD_ABORT_TARGET           0x0017
#define MBOX_CMD_BUS_RESET              0x0018
#define MBOX_CMD_GET_FIRMWARE_STATUS    0x001f
#define MBOX_CMD_GET_INITIATOR_ID       0x0020
#define MBOX_CMD_GET_SELECTION_TIMEOUT  0x0021
#define MBOX_CMD_GET_RETRY_COUNT        0x0022
#define MBOX_CMD_GET_TAG_AGE_LIMIT      0x0023
#define MBOX_CMD_GET_CLOCK_RATE         0x0024
#define MBOX_CMD_GET_ACTIVE_NEG         0x0025
#define MBOX_CMD_GET_ASYNC_DATA_SETUP   0x0026
#define MBOX_CMD_GET_BUS_CONTROL        0x0027
#define MBOX_CMD_GET_TARGET_PARAMETERS  0x0028
#define MBOX_CMD_GET_DEVICE_QUEUE_PARAM 0x0029
#define MBOX_CMD_SET_INITIATOR_ID       0x0030
#define MBOX_CMD_SET_SELECTION_TIMEOUT  0x0031
#define MBOX_CMD_SET_RETRY_COUNT        0x0032
#define MBOX_CMD_SET_TAG_AGE_LIMIT      0x0033
#define MBOX_CMD_SET_CLOCK_RATE         0x0034
#define MBOX_CMD_SET_ACTIVE_NEG         0x0035
#define MBOX_CMD_SET_ASYNC_DATA_SETUP   0x0036
#define MBOX_CMD_SET_BUS_CONTROL        0x0037
#define MBOX_CMD_SET_TARGET_PARAMETERS  0x0038
#define MBOX_CMD_SET_DEVICE_QUEUE_PARAM 0x0039

#define MBOX_STS_FIRMWARE_ALIVE         0x0000
#define MBOX_STS_CHECKSUM_ERROR         0x0001
#define MBOX_STS_SHADOW_LOAD_ERROR      0x0002
#define MBOX_STS_BUSY                   0x0004
#define MBOX_STS_COMMAND_COMPLETE       0x4000
#define MBOX_STS_INVALID_COMMAND        0x4001
#define MBOX_STS_HOST_INTERFACE_ERROR   0x4002
#define MBOX_STS_TEST_FAILED            0x4003
#define MBOX_STS_COMMAND_ERROR          0x4005
#define MBOX_STS_COMMAND_PARAMETER_ERROR 0x4006

/* Host memory queue state */
#define QL_MAX_QUEUE_ENTRIES 4096

/* RISC instruction RAM base (firmware load/verify addresses are based here) */
#define QL_RISC_RAMBASE   0x1000

/* A64 command/continuation entry geometry (ql_standalone.h) */
#define QL_ENTRY_SIZE        64
/*
 * The A64 ioctl request-entry format is only used by SN0 (Origin/IP27)
 * builds of the driver; IP30 compiles it out (ql.c gates A64_BIT_OPERATION
 * on "#if SN0"). The two differ in the data-segment layout and in how many
 * segments fit per command/continuation entry:
 *
 *              command dseg   stride  IOCB_SEGS  continuation dseg  CONT_SEGS
 *   legacy         0x20         8         4            0x08           7
 *   A64            0x28        12         2            0x04           5
 *
 * Select per entry from entry_type (ET_COMMAND 0x1 legacy / 0x9 A64).
 */
#define QL_IOCB_SEGS_LEGACY  4
#define QL_CONT_SEGS_LEGACY  7
#define QL_IOCB_SEGS_A64     2
#define QL_CONT_SEGS_A64     5
#define QL_MAX_SG            64
#define QL_ET_COMMAND        0x9
#define QL_ET_COMMAND_LEGACY 0x1
#define QL_ET_CONTINUATION   0xa
#define QL_ET_CONT_LEGACY    0x2
#define QL_ET_STATUS         0x3
#define QL_ET_MARKER         0x4

/*
 * IP30 bridge 32-bit direct-mapped DMA window: the driver's ql.c does
 * MAKE_DIRECT_MAPPED_2GIG() = kv_to_bridge32_dirmap(), and the PROM programs
 * the bridge so that DMA address = host physical + 0x80000000.
 */
#define QL_DMA_DIRECT_BASE   0x80000000ULL

/* status entry completion status (ql_standalone.h) */
#define QL_SCS_COMPLETE          0x0000
#define QL_SCS_TRANSPORT_ERROR   0x0003
#define QL_SS_GOT_STATUS         0x1000
#define QL_SS_TRANSFER_COMPLETE  0x4000

typedef struct QLQueue {
    uint64_t base;      /* host physical base address */
    uint32_t count;     /* entries */
    uint32_t in;        /* host in-pointer (RISC fetch) */
    uint32_t out;       /* host out-pointer (RISC post) */
} QLQueue;

typedef struct QLSG {
    uint64_t addr;
    uint32_t len;
} QLSG;

struct SGIQLispState {
    DeviceState parent_obj;

    MemoryRegion regs;

    /* register file: [0x00..0xff] as 16-bit words */
    uint16_t reg[QLISP_REGS_SIZE / 2];

    /* PCI config */
    uint16_t pci_cmd;
    uint32_t pci_bar[2];
    uint8_t pci_rev;

    /* RISC / firmware state */
    bool firmware_running;
    bool cmd_pending;          /* mailbox command completed, host to ack */
    uint16_t risc_ram[0x4000]; /* word addressable, 0x1000-based */
    uint32_t risc_loaded;      /* words loaded by LOAD_RAM (for checksum) */

    /*
     * Control entries (command/continuation/status/marker) are byte-reversed
     * per 32-bit word by the driver's munge() only on IP30 (ql.c gates every
     * call on `#if defined(IP30)`); SN0/IP27 leave them in natural order. The
     * machine that runs an IP30 driver opts in via the "control-munge" prop.
     */
    bool control_munge;

    /* request/response queues */
    QLQueue req;
    QLQueue rsp;

    /* internal SCSI bus (scsi-hd/scsi-cd children) */
    SCSIBus bus;
    uint32_t busnr;

    /* in-flight request (synchronous RISC model) */
    SCSIRequest *cur_req;
    uint32_t cur_handle;
    uint32_t cur_tag;
    bool cur_tgt_present;
    uint32_t nsg;
    QLSG sg[QL_MAX_SG];
    uint32_t sg_idx;
    uint32_t sg_off;
    uint32_t xfer_total;

    /* SCSI backend (legacy -drive property) */
    BlockBackend *blk;
};

/*
 * ISP1020 PCI configuration BARs.  The bridge exposes the ISP config space, and
 * the OS sizes each BAR with the standard all-ones probe: it writes 0xffffffff
 * and reads back the size mask (a 0 answer means "BAR not implemented", so a
 * careful probe then rejects the device).  BAR0 is an I/O BAR (bit 0 reads 1)
 * and BAR1 a memory BAR (type bits [3:1] read 0); the sizes fit the BaseIO
 * windows and match the Linux qla1280 driver's expectations (types confirmed
 * from qla1280, sizes not datasheet-confirmed).
 */
#define QLISP_PCI_BAR0_OFF 0x10 /* I/O BAR: probe reads 0xffffff01 */
#define QLISP_PCI_BAR1_OFF 0x14 /* memory BAR: probe reads 0xfffff000 */
#define QLISP_BAR0_SIZE 256
#define QLISP_BAR1_SIZE 4096

/* Config-space BAR read/write: applies the sizing mask + type on read, and
 * latches only the base bits on write.  See the ISP1020 contract above. */
uint32_t sgi_qlisp_pci_config_read(SGIQLispState *s, unsigned cfg);
void sgi_qlisp_pci_config_write(SGIQLispState *s, unsigned cfg, uint32_t val);

#endif /* HW_SCSI_SGI_QLISP_H */
