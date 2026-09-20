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

typedef struct QLQueue {
    uint64_t base;      /* host physical base address */
    uint32_t count;     /* entries */
    uint32_t in;        /* host in-pointer (RISC fetch) */
    uint32_t out;       /* host out-pointer (RISC post) */
} QLQueue;

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
    uint16_t risc_ram[4096];   /* sparse-ish; word addressable */

    /* request/response queues */
    QLQueue req;
    QLQueue rsp;

    /* SCSI backend */
    BlockBackend *blk;
};

#endif /* HW_SCSI_SGI_QLISP_H */
