/*
 * WD33C93/WD33C93A/WD33C93B SCSI Controller emulation
 *
 * The WD33C93 is a SCSI bus interface controller used in many systems
 * including SGI workstations (Indy, Indigo2) where it interfaces via HPC3.
 *
 * References:
 *   - MAME src/devices/machine/wd33c9x.cpp
 *   - WD33C93B datasheet
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef QEMU_HW_WD33C93_H
#define QEMU_HW_WD33C93_H

#include "hw/scsi/scsi.h"
#include "qemu/fifo8.h"
#include "qom/object.h"

#define WD33C93_REGS 32
#define WD33C93_FIFO_SIZE 12

/*
 * Register addresses (indirect addressing)
 * The WD33C93 uses indirect register access:
 *   - Write to address register to select register
 *   - Read/write data register to access selected register
 */
#define WD_OWN_ID               0x00  /* Own ID / CDB Size */
#define WD_CONTROL              0x01  /* Control Register */
#define WD_TIMEOUT_PERIOD       0x02  /* Timeout Period */
#define WD_CDB_1                0x03  /* CDB Byte 1 (also Total Sectors) */
#define WD_CDB_2                0x04  /* CDB Byte 2 */
#define WD_CDB_3                0x05  /* CDB Byte 3 */
#define WD_CDB_4                0x06  /* CDB Byte 4 */
#define WD_CDB_5                0x07  /* CDB Byte 5 */
#define WD_CDB_6                0x08  /* CDB Byte 6 */
#define WD_CDB_7                0x09  /* CDB Byte 7 */
#define WD_CDB_8                0x0a  /* CDB Byte 8 */
#define WD_CDB_9                0x0b  /* CDB Byte 9 */
#define WD_CDB_10               0x0c  /* CDB Byte 10 */
#define WD_CDB_11               0x0d  /* CDB Byte 11 */
#define WD_CDB_12               0x0e  /* CDB Byte 12 */
#define WD_TARGET_LUN           0x0f  /* Target LUN */
#define WD_COMMAND_PHASE        0x10  /* Command Phase */
#define WD_SYNC_TRANSFER        0x11  /* Synchronous Transfer */
#define WD_TRANSFER_COUNT_MSB   0x12  /* Transfer Count MSB */
#define WD_TRANSFER_COUNT       0x13  /* Transfer Count Middle */
#define WD_TRANSFER_COUNT_LSB   0x14  /* Transfer Count LSB */
#define WD_DESTINATION_ID       0x15  /* Destination ID */
#define WD_SOURCE_ID            0x16  /* Source ID */
#define WD_SCSI_STATUS          0x17  /* SCSI Status (read-only) */
#define WD_COMMAND              0x18  /* Command (write triggers execution) */
#define WD_DATA                 0x19  /* Data Register */
#define WD_QUEUE_TAG            0x1a  /* Queue Tag (33C93B only) */
#define WD_AUXILIARY_STATUS     0x1f  /* Auxiliary Status (read-only) */

/*
 * Auxiliary Status Register (ASR) bits
 * This register is read via the address register port
 */
#define ASR_DBR  0x01  /* Data Buffer Ready */
#define ASR_PE   0x02  /* Parity Error */
#define ASR_FFE  0x04  /* FIFO Full/Empty (33C93B only) */
#define ASR_CIP  0x10  /* Command In Progress */
#define ASR_BSY  0x20  /* Busy */
#define ASR_LCI  0x40  /* Last Command Ignored */
#define ASR_INT  0x80  /* Interrupt Pending */

/*
 * SCSI Status Register values
 * Returned in WD_SCSI_STATUS after command completion
 */
#define SCSI_STATUS_RESET                   0x00
#define SCSI_STATUS_RESET_EAF               0x01
#define SCSI_STATUS_RESELECT_SUCCESS        0x10
#define SCSI_STATUS_SELECT_SUCCESS          0x11
#define SCSI_STATUS_COMMAND_SUCCESS         0x13
#define SCSI_STATUS_SELECT_TRANSFER_SUCCESS 0x16
#define SCSI_STATUS_TRANSFER_SUCCESS        0x18
#define SCSI_STATUS_TRANSFER_INFO_MSG_IN    0x20
#define SCSI_STATUS_INVALID_COMMAND         0x40
#define SCSI_STATUS_UNEXPECTED_DISCONNECT   0x41
#define SCSI_STATUS_SELECTION_TIMEOUT       0x42
#define SCSI_STATUS_PARITY_ERROR            0x43
#define SCSI_STATUS_UNEX_RDATA              0x48  /* TC=0, chip receiving (DATA OUT/write) */
#define SCSI_STATUS_UNEX_SDATA              0x49  /* TC=0, chip sending (DATA IN/read) */
#define SCSI_STATUS_DISCONNECT              0x85

/*
 * Command Register values
 * Written to WD_COMMAND to execute operations
 */
#define CMD_RESET               0x00  /* Reset SBIC */
#define CMD_ABORT               0x01  /* Abort */
#define CMD_ASSERT_ATN          0x02  /* Assert ATN */
#define CMD_NEGATE_ACK          0x03  /* Negate ACK */
#define CMD_DISCONNECT          0x04  /* Disconnect */
#define CMD_RESELECT            0x05  /* Reselect */
#define CMD_SELECT_ATN          0x06  /* Select with ATN */
#define CMD_SELECT              0x07  /* Select without ATN */
#define CMD_SELECT_ATN_XFER     0x08  /* Select with ATN and Transfer */
#define CMD_SELECT_XFER         0x09  /* Select and Transfer */
#define CMD_TRANSFER_INFO       0x20  /* Transfer Info */
#define CMD_SBT                 0x80  /* Single Byte Transfer bit */

/*
 * Own ID Register bits
 */
#define OWN_ID_SCSI_ID          0x07  /* SCSI ID bits */
#define OWN_ID_EAF              0x08  /* Enable Advanced Features */
#define OWN_ID_EHP              0x10  /* Enable Host Parity */
#define OWN_ID_RAF              0x20  /* Really Advanced Features (93B) */
#define OWN_ID_FS_MASK          0xc0  /* Frequency Select */

/*
 * Control Register bits
 */
#define CONTROL_DM_MASK         0xe0  /* DMA Mode Select */
#define CONTROL_DM_POLLED       0x00  /* Polled I/O */
#define CONTROL_DM_BURST        0x20  /* Burst Mode DMA */
#define CONTROL_DM_BUS          0x40  /* WD-Bus Mode */
#define CONTROL_DM_DMA          0x80  /* Single-byte DMA */

/*
 * Destination ID Register bits
 */
#define DEST_ID_MASK            0x07  /* Target SCSI ID */
#define DEST_ID_DPD             0x40  /* Data Phase Direction */
#define DEST_ID_SCC             0x80  /* Select Command Chain */

/*
 * Target LUN Register bits
 */
#define TARGET_LUN_MASK         0x07  /* LUN bits */
#define TARGET_LUN_TLV          0x80  /* Target LUN Valid */

#define TYPE_WD33C93 "wd33c93"
OBJECT_DECLARE_SIMPLE_TYPE(WD33C93State, WD33C93)

struct WD33C93State {
    DeviceState parent_obj;

    /* Indirect addressing registers */
    uint8_t addr_reg;                   /* Address register (selects internal reg) */
    uint8_t regs[WD33C93_REGS];         /* Internal registers */

    /* Status registers (computed, not stored in regs[]) */
    uint8_t scsi_status;                /* SCSI Status (read clears INT) */
    uint8_t aux_status;                 /* Auxiliary Status Register */

    /* Data FIFO for transfers */
    Fifo8 fifo;

    /* SCSI bus interface */
    SCSIBus bus;
    SCSIDevice *current_dev;
    SCSIRequest *current_req;

    /* Transfer state */
    uint32_t transfer_count;            /* Bytes remaining in transfer */
    uint32_t async_len;                 /* Bytes available from device */
    uint8_t *async_buf;                 /* Pointer to device buffer */

    /* Pending data from transfer_data() rejected due to TC=0.
     * Used for multi-pass DMA: the driver reprograms TC and issues
     * TRANSFER_INFO to resume. See IRIX wd93.c ST_UNEX_SDATA/RDATA. */
    uint32_t pending_len;               /* Bytes awaiting next TC */
    uint8_t *pending_buf;               /* Buffer pointer (into SCSI buf) */

    /* Output signals */
    qemu_irq irq;                       /* Interrupt output */
    qemu_irq drq;                       /* DMA Request output */
    bool drq_state;                     /* Current DRQ state */
};

/*
 * API for HPC3 (or other host bus adapters)
 * The WD33C93 uses indirect register access via two ports:
 *   - Address port (offset 0): Write sets register pointer, read returns ASR
 *   - Data port (offset 4): Read/write selected internal register
 */
uint8_t wd33c93_addr_read(WD33C93State *s);
void wd33c93_addr_write(WD33C93State *s, uint8_t val);
uint8_t wd33c93_data_read(WD33C93State *s);
void wd33c93_data_write(WD33C93State *s, uint8_t val);

/* DMA helpers for HPC3 DMA engine */
void wd33c93_set_drq(WD33C93State *s, bool state);
void wd33c93_set_transfer_count(WD33C93State *s, uint32_t count);

#endif /* QEMU_HW_WD33C93_H */
