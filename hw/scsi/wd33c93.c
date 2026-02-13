/*
 * WD33C93/WD33C93A/WD33C93B SCSI Controller emulation
 *
 * The WD33C93 is a SCSI bus interface controller used in many systems
 * including SGI workstations (Indy, Indigo2) where it interfaces via HPC3.
 *
 * This implementation focuses on initiator mode operation as used by
 * workstation PROMs and operating systems.
 *
 * References:
 *   - MAME src/devices/machine/wd33c9x.cpp
 *   - QEMU hw/scsi/esp.c (for QEMU SCSI patterns)
 *   - WD33C93B datasheet
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/scsi/scsi.h"
#include "hw/scsi/wd33c93.h"
#include "scsi/utils.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

/* Debug logging */
#define WD_DEBUG 0
#define WD_DPRINTF(fmt, ...) \
    do { \
        if (WD_DEBUG) { \
            qemu_log("wd33c93: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)

/* Global debug sequence counter shared with sgi_hpc3.c */
int wd33c93_debug_seq = 0;
#define DBG_SEQ_START 8000
#define DBG_SEQ_END   8400
#define DBG_SEQ_ACTIVE() (wd33c93_debug_seq >= DBG_SEQ_START && wd33c93_debug_seq < DBG_SEQ_END)

/*
 * Raise interrupt to host
 */
static void wd33c93_raise_irq_from(WD33C93State *s, const char *caller)
{
    if (!(s->aux_status & ASR_INT)) {
        s->aux_status |= ASR_INT;
        qemu_irq_raise(s->irq);
        wd33c93_debug_seq++;
        if (DBG_SEQ_ACTIVE()) {
            FILE *dbg = fopen("/tmp/qemu_scsi_irq_debug.log", "a");
            if (dbg) {
                fprintf(dbg, "[%d] WD93_IRQ_RAISE[%s]: scsi_status=0x%02x cmd_phase=0x%02x\n",
                        wd33c93_debug_seq, caller, s->scsi_status, s->regs[0x18]);
                fclose(dbg);
            }
        }
        WD_DPRINTF("IRQ raised, status=0x%02x\n", s->scsi_status);
    }
}
#define wd33c93_raise_irq(s) wd33c93_raise_irq_from((s), __func__)

/*
 * Lower interrupt
 */
static void wd33c93_lower_irq(WD33C93State *s)
{
    if (s->aux_status & ASR_INT) {
        s->aux_status &= ~ASR_INT;
        qemu_irq_lower(s->irq);
        WD_DPRINTF("IRQ lowered\n");
    }
}

/*
 * Set DRQ signal state
 */
void wd33c93_set_drq(WD33C93State *s, bool state)
{
    if (s->drq_state != state) {
        s->drq_state = state;
        qemu_set_irq(s->drq, state ? 1 : 0);
        WD_DPRINTF("DRQ %s\n", state ? "raised" : "lowered");
    }
}

/*
 * Get transfer count from registers
 */
static uint32_t wd33c93_get_transfer_count(WD33C93State *s)
{
    return ((uint32_t)s->regs[WD_TRANSFER_COUNT_MSB] << 16) |
           ((uint32_t)s->regs[WD_TRANSFER_COUNT] << 8) |
           s->regs[WD_TRANSFER_COUNT_LSB];
}

/*
 * Set transfer count in registers
 */
void wd33c93_set_transfer_count(WD33C93State *s, uint32_t count)
{
    s->regs[WD_TRANSFER_COUNT_MSB] = (count >> 16) & 0xff;
    s->regs[WD_TRANSFER_COUNT] = (count >> 8) & 0xff;
    s->regs[WD_TRANSFER_COUNT_LSB] = count & 0xff;
    s->transfer_count = count;
}

/*
 * Complete a command with status and interrupt
 */
static void wd33c93_complete_cmd(WD33C93State *s, uint8_t status)
{
    s->scsi_status = status;
    s->aux_status &= ~(ASR_CIP | ASR_BSY);
    wd33c93_raise_irq(s);
}

/*
 * Handle RESET command
 * Resets the SBIC to initial state
 */
static void wd33c93_do_reset(WD33C93State *s)
{
    bool eaf;

    WD_DPRINTF("RESET command\n");

    /* Check if EAF was set before reset */
    eaf = s->regs[WD_OWN_ID] & OWN_ID_EAF;

    /* Clear all registers */
    memset(s->regs, 0, sizeof(s->regs));

    /* Reset FIFO */
    fifo8_reset(&s->fifo);

    /* Clear transfer state */
    s->transfer_count = 0;
    s->async_len = 0;
    s->async_buf = NULL;
    s->pending_len = 0;
    s->pending_buf = NULL;

    /* Cancel any pending SCSI request */
    if (s->current_req) {
        scsi_req_cancel(s->current_req);
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
    }
    s->current_dev = NULL;

    /* Lower DRQ */
    wd33c93_set_drq(s, false);

    /* Complete with RESET status */
    s->scsi_status = eaf ? SCSI_STATUS_RESET_EAF : SCSI_STATUS_RESET;
    s->aux_status = ASR_INT;  /* Only INT set after reset */
    wd33c93_raise_irq(s);
}

/*
 * Handle ABORT command
 */
static void wd33c93_do_abort(WD33C93State *s)
{
    WD_DPRINTF("ABORT command\n");

    /* Cancel any pending request */
    if (s->current_req) {
        scsi_req_cancel(s->current_req);
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
    }
    s->current_dev = NULL;

    /* Reset transfer state */
    s->transfer_count = 0;
    s->pending_len = 0;
    s->pending_buf = NULL;
    fifo8_reset(&s->fifo);
    wd33c93_set_drq(s, false);

    /* For now, complete with disconnect status */
    wd33c93_complete_cmd(s, SCSI_STATUS_DISCONNECT);
}

/*
 * Start a SELECT operation
 * Returns 0 on success, -1 if no device found
 */
static int wd33c93_select(WD33C93State *s)
{
    uint8_t target_id = s->regs[WD_DESTINATION_ID] & DEST_ID_MASK;
    uint8_t lun = s->regs[WD_TARGET_LUN] & TARGET_LUN_MASK;

    WD_DPRINTF("SELECT target=%d lun=%d\n", target_id, lun);

    /* Cancel any existing request */
    if (s->current_req) {
        scsi_req_cancel(s->current_req);
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
    }

    /* Find the target device */
    s->current_dev = scsi_device_find(&s->bus, 0, target_id, lun);

    if (!s->current_dev) {
        WD_DPRINTF("SELECT: no device at target=%d lun=%d\n", target_id, lun);
        return -1;
    }

    return 0;
}

/*
 * Execute a SCSI command after SELECT
 * Used by SELECT_ATN_XFER and SELECT_XFER commands
 */
static void wd33c93_do_select_xfer(WD33C93State *s, bool with_atn)
{
    uint8_t target_id = s->regs[WD_DESTINATION_ID] & DEST_ID_MASK;
    uint8_t lun = s->regs[WD_TARGET_LUN] & TARGET_LUN_MASK;
    uint8_t cdb_len;
    uint8_t cdb[12];
    int32_t datalen;
    int i;

    WD_DPRINTF("SELECT%s_XFER target=%d lun=%d\n",
               with_atn ? "_ATN" : "", target_id, lun);

    s->aux_status |= ASR_CIP | ASR_BSY;

    /*
     * Multi-pass DMA resume: IRIX issues SELECT_ATN_XFER with command
     * phase set to PH_IDENTRECV (0x45) to continue a transfer after
     * an "unexpected phase" interrupt. The target is already selected
     * and we have pending data — just resume the DMA transfer.
     *
     * Reference: IRIX wd93.c:2946-2951 — setdest() after save_datap()
     */
    if (s->pending_len > 0 && s->current_req &&
        (s->regs[WD_COMMAND_PHASE] == 0x44 ||  /* PH_RESELECT */
         s->regs[WD_COMMAND_PHASE] == 0x45)) {  /* PH_IDENTRECV */
        s->transfer_count = wd33c93_get_transfer_count(s);
        s->async_buf = s->pending_buf;
        s->async_len = MIN(s->pending_len, s->transfer_count);
        s->pending_len -= s->async_len;
        if (s->pending_len > 0) {
            s->pending_buf += s->async_len;
        } else {
            s->pending_buf = NULL;
        }
        s->regs[WD_COMMAND_PHASE] = 0x30;  /* data transfer */
        s->aux_status |= ASR_DBR;
        if ((s->regs[WD_CONTROL] & CONTROL_DM_MASK) != CONTROL_DM_POLLED) {
            wd33c93_set_drq(s, true);
        }
        qemu_log_mask(LOG_UNIMP, "wd33c93: SELECT_XFER resume multi-pass: "
                      "TC=%u async_len=%u pending_len=%u\n",
                      s->transfer_count, s->async_len, s->pending_len);
        return;
    }

    /* Try to select target */
    if (wd33c93_select(s) < 0) {
        /* Selection timeout */
        wd33c93_complete_cmd(s, SCSI_STATUS_SELECTION_TIMEOUT);
        return;
    }

    /*
     * Get CDB length from OWN_ID register (when EAF is set)
     * or determine from command byte
     */
    if (s->regs[WD_OWN_ID] & OWN_ID_EAF) {
        cdb_len = s->regs[WD_OWN_ID] & 0x0f;
        if (cdb_len == 0) {
            cdb_len = 6;  /* Default */
        }
    } else {
        /* Determine from command group */
        uint8_t cmd_group = (s->regs[WD_CDB_1] >> 5) & 0x07;
        switch (cmd_group) {
        case 0:
            cdb_len = 6;
            break;
        case 1:
        case 2:
            cdb_len = 10;
            break;
        case 5:
            cdb_len = 12;
            break;
        default:
            cdb_len = 6;
            break;
        }
    }

    /* Limit to maximum */
    if (cdb_len > 12) {
        cdb_len = 12;
    }

    /* Build CDB from registers */
    for (i = 0; i < cdb_len; i++) {
        cdb[i] = s->regs[WD_CDB_1 + i];
    }

    /* Set command phase: CDB bytes being sent (MAME CP_BYTES_0 = 0x10) */
    s->regs[WD_COMMAND_PHASE] = 0x10;

    /* Get transfer count */
    s->transfer_count = wd33c93_get_transfer_count(s);

    /* Log SCSI command details */
    {
        char cdb_str[64];
        int pos = 0;
        for (i = 0; i < cdb_len && pos < (int)sizeof(cdb_str) - 3; i++) {
            pos += snprintf(cdb_str + pos, sizeof(cdb_str) - pos,
                            "%02x ", cdb[i]);
        }
        qemu_log_mask(LOG_UNIMP, "wd33c93: SELECT_XFER target=%d "
                      "cmd=%s(0x%02x) CDB[%d]={%s}tc=%d\n",
                      target_id, scsi_command_name(cdb[0]), cdb[0],
                      cdb_len, cdb_str, s->transfer_count);
    }

    /* Create and enqueue SCSI request */
    s->current_req = scsi_req_new(s->current_dev, 0, lun, cdb, cdb_len, s);
    datalen = scsi_req_enqueue(s->current_req);

    qemu_log_mask(LOG_UNIMP, "wd33c93: SCSI response datalen=%d\n", datalen);

    if (datalen != 0) {
        /*
         * Data transfer expected
         * Wait for transfer_data callback
         */
        scsi_req_continue(s->current_req);
    }
    /* For no-data commands, command_complete will be called */
}

/*
 * Handle SELECT command (without transfer)
 */
static void wd33c93_do_select(WD33C93State *s, bool with_atn)
{
    WD_DPRINTF("SELECT%s\n", with_atn ? "_ATN" : "");

    s->aux_status |= ASR_CIP | ASR_BSY;

    if (wd33c93_select(s) < 0) {
        wd33c93_complete_cmd(s, SCSI_STATUS_SELECTION_TIMEOUT);
        return;
    }

    /* Selection successful */
    wd33c93_complete_cmd(s, SCSI_STATUS_SELECT_SUCCESS);
}

/*
 * Execute command written to command register
 */
static void wd33c93_execute_cmd(WD33C93State *s, uint8_t cmd)
{
    uint8_t cmd_code = cmd & 0x7f;
    /* bool sbt = cmd & CMD_SBT; */ /* Single byte transfer modifier */

    {
        wd33c93_debug_seq++;
        if (DBG_SEQ_ACTIVE()) {
            FILE *dbg = fopen("/tmp/qemu_scsi_irq_debug.log", "a");
            if (dbg) {
                fprintf(dbg, "[%d] WD93_CMD: 0x%02x (code=0x%02x) asr=0x%02x "
                        "tgt=%d ctrl=0x%02x",
                        wd33c93_debug_seq,
                        cmd, cmd_code, s->aux_status,
                        s->regs[WD_DESTINATION_ID] & 0x07,
                        s->regs[WD_CONTROL]);
                if (cmd_code == CMD_SELECT_ATN_XFER ||
                    cmd_code == CMD_SELECT_XFER) {
                    fprintf(dbg, " CDB=%02x%02x%02x%02x%02x%02x tc=%d",
                            s->regs[WD_CDB_1], s->regs[WD_CDB_1+1],
                            s->regs[WD_CDB_1+2], s->regs[WD_CDB_1+3],
                            s->regs[WD_CDB_1+4], s->regs[WD_CDB_1+5],
                            (s->regs[WD_TRANSFER_COUNT_MSB] << 16) |
                            (s->regs[WD_TRANSFER_COUNT] << 8) |
                            s->regs[WD_TRANSFER_COUNT_LSB]);
                }
                fprintf(dbg, "\n");
                fclose(dbg);
            }
        }
    }

    WD_DPRINTF("Command 0x%02x\n", cmd);

    qemu_log_mask(LOG_UNIMP, "wd33c93: CMD 0x%02x asr=0x%02x ctrl=0x%02x "
                  "tgt=%d tc=%d\n", cmd, s->aux_status,
                  s->regs[WD_CONTROL],
                  s->regs[WD_DESTINATION_ID] & 0x07,
                  wd33c93_get_transfer_count(s));


    /* Check if last command was ignored */
    if (s->aux_status & ASR_CIP) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wd33c93: command 0x%02x while CIP set\n", cmd);
        s->aux_status |= ASR_LCI;
        return;
    }

    /*
     * Clear any pending interrupt from previous command.
     * Some drivers issue new commands without first reading SCSI_STATUS
     * to acknowledge the previous interrupt. Clear it now to avoid
     * returning stale status values.
     */
    if (s->aux_status & ASR_INT) {
        WD_DPRINTF("Clearing pending IRQ before new command\n");
        wd33c93_lower_irq(s);
    }

    s->aux_status &= ~ASR_LCI;

    switch (cmd_code) {
    case CMD_RESET:
        wd33c93_do_reset(s);
        break;

    case CMD_ABORT:
        wd33c93_do_abort(s);
        break;

    case CMD_SELECT_ATN:
        wd33c93_do_select(s, true);
        break;

    case CMD_SELECT:
        wd33c93_do_select(s, false);
        break;

    case CMD_SELECT_ATN_XFER:
        wd33c93_do_select_xfer(s, true);
        break;

    case CMD_SELECT_XFER:
        wd33c93_do_select_xfer(s, false);
        break;

    case CMD_DISCONNECT:
        WD_DPRINTF("DISCONNECT\n");
        s->current_dev = NULL;
        if (s->current_req) {
            scsi_req_unref(s->current_req);
            s->current_req = NULL;
        }
        wd33c93_complete_cmd(s, SCSI_STATUS_DISCONNECT);
        break;

    case CMD_TRANSFER_INFO:
        WD_DPRINTF("TRANSFER_INFO\n");
        s->aux_status |= ASR_CIP | ASR_BSY;

        /*
         * TRANSFER_INFO can be used in three ways:
         *
         * 1. After SELECT_ATN_XFER/SELECT_XFER: A request is already active
         *    and we just need to trigger DRQ for remaining data.
         *
         * 2. After SELECT/SELECT_ATN: The target is selected but no CDB has
         *    been sent yet. Build and execute the SCSI command here.
         *
         * 3. After "unexpected phase" interrupt (TC=0 mid-transfer):
         *    The driver has reprogrammed TC and DMA descriptors. Resume
         *    the transfer with the pending data.
         *    Reference: IRIX wd93.c:2936-2956
         */
        if (s->pending_len > 0 && s->current_req) {
            /*
             * Path 3: Resume after "unexpected phase" (TC=0 mid-transfer).
             * Driver has reprogrammed TC and DMA descriptors.
             */
            s->transfer_count = wd33c93_get_transfer_count(s);
            s->async_buf = s->pending_buf;
            s->async_len = MIN(s->pending_len, s->transfer_count);
            s->pending_len = 0;
            s->pending_buf = NULL;
            s->regs[WD_COMMAND_PHASE] = 0x30;  /* data transfer */
            s->aux_status |= ASR_DBR;
            if ((s->regs[WD_CONTROL] & CONTROL_DM_MASK) != CONTROL_DM_POLLED) {
                wd33c93_set_drq(s, true);
            }
            qemu_log_mask(LOG_UNIMP, "wd33c93: TRANSFER_INFO resume: "
                          "new TC=%u async_len=%u\n",
                          s->transfer_count, s->async_len);
        } else if (!s->current_req && s->current_dev) {
            /*
             * Path 2: SELECT was done but no request yet. Build CDB and
             * execute, same as SELECT_ATN_XFER but without the select step.
             */
            uint8_t lun = s->regs[WD_TARGET_LUN] & TARGET_LUN_MASK;
            uint8_t cdb_len_ti;
            uint8_t cdb_ti[12];
            int32_t datalen_ti;

            qemu_log_mask(LOG_UNIMP, "wd33c93: TRANSFER_INFO building SCSI "
                          "request (SELECT+XFER path)\n");

            /* Determine CDB length */
            if (s->regs[WD_OWN_ID] & OWN_ID_EAF) {
                cdb_len_ti = s->regs[WD_OWN_ID] & 0x0f;
                if (cdb_len_ti == 0) {
                    cdb_len_ti = 6;
                }
            } else {
                uint8_t cmd_group_ti = (s->regs[WD_CDB_1] >> 5) & 0x07;
                switch (cmd_group_ti) {
                case 0:  cdb_len_ti = 6;  break;
                case 1:
                case 2:  cdb_len_ti = 10; break;
                case 5:  cdb_len_ti = 12; break;
                default: cdb_len_ti = 6;  break;
                }
            }
            if (cdb_len_ti > 12) {
                cdb_len_ti = 12;
            }

            for (int j = 0; j < cdb_len_ti; j++) {
                cdb_ti[j] = s->regs[WD_CDB_1 + j];
            }

            s->transfer_count = wd33c93_get_transfer_count(s);
            s->regs[WD_COMMAND_PHASE] = 0x10;

            s->current_req = scsi_req_new(s->current_dev, 0, lun,
                                          cdb_ti, cdb_len_ti, s);
            datalen_ti = scsi_req_enqueue(s->current_req);

            if (datalen_ti != 0) {
                scsi_req_continue(s->current_req);
            }
        } else if (s->current_req && s->async_len > 0) {
            /* Path 1: Data already available from active request */
            s->aux_status |= ASR_DBR;
            wd33c93_set_drq(s, true);
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "wd33c93: unimplemented command 0x%02x\n", cmd);
        wd33c93_complete_cmd(s, SCSI_STATUS_INVALID_COMMAND);
        break;
    }
}

/*
 * Read from address register port
 * Returns Auxiliary Status Register (ASR)
 */
uint8_t wd33c93_addr_read(WD33C93State *s)
{
    wd33c93_debug_seq++;
    if (DBG_SEQ_ACTIVE()) {
        FILE *dbg = fopen("/tmp/qemu_scsi_irq_debug.log", "a");
        if (dbg) {
            fprintf(dbg, "[%d] WD93_ASR_READ: 0x%02x (INT=%d DBR=%d BSY=%d)\n",
                    wd33c93_debug_seq, s->aux_status,
                    !!(s->aux_status & 0x80),   /* ASR_INT */
                    !!(s->aux_status & 0x01),   /* ASR_DBR */
                    !!(s->aux_status & 0x20));  /* ASR_BSY */
            fclose(dbg);
        }
    }
    WD_DPRINTF("ASR read: 0x%02x\n", s->aux_status);
    return s->aux_status;
}

/*
 * Write to address register port
 * Sets the register pointer for subsequent data access
 */
void wd33c93_addr_write(WD33C93State *s, uint8_t val)
{
    s->addr_reg = val & 0x1f;  /* Only 5 bits used */
    WD_DPRINTF("addr_reg = 0x%02x\n", s->addr_reg);
}

/*
 * Read from data register port
 * Returns value of currently selected register
 *
 * The WD33C93 auto-increments the address register after each data access,
 * except when reading COMMAND or AUXILIARY_STATUS registers.
 */
uint8_t wd33c93_data_read(WD33C93State *s)
{
    uint8_t val;
    uint8_t reg = s->addr_reg;
    bool auto_inc = true;

    switch (reg) {
    case WD_SCSI_STATUS:
        val = s->scsi_status;
        /* Reading status clears interrupt */
        wd33c93_lower_irq(s);
        {
                wd33c93_debug_seq++;
            if (DBG_SEQ_ACTIVE()) {
                FILE *dbg = fopen("/tmp/qemu_scsi_irq_debug.log", "a");
                if (dbg) {
                    fprintf(dbg, "[%d] WD93_STATUS_READ: 0x%02x (IRQ cleared)\n",
                            wd33c93_debug_seq, val);
                    fclose(dbg);
                }
            }
        }
        WD_DPRINTF("SCSI_STATUS read: 0x%02x (IRQ cleared)\n", val);
        break;

    case WD_AUXILIARY_STATUS:
        val = s->aux_status;
        auto_inc = false;  /* Don't auto-increment for AUX_STATUS */
        WD_DPRINTF("AUX_STATUS read: 0x%02x\n", val);
        break;

    case WD_COMMAND:
        val = s->regs[reg];
        auto_inc = false;  /* Don't auto-increment for COMMAND */
        WD_DPRINTF("reg[0x%02x] read: 0x%02x\n", reg, val);
        break;

    case WD_DATA:
        /* Read from data FIFO */
        if (!fifo8_is_empty(&s->fifo)) {
            val = fifo8_pop(&s->fifo);
        } else if (s->async_buf && s->async_len > 0) {
            /* Read directly from device buffer */
            val = *s->async_buf++;
            s->async_len--;
            if (s->transfer_count > 0) {
                s->transfer_count--;
                wd33c93_set_transfer_count(s, s->transfer_count);
            }
            WD_DPRINTF("DATA read from device: 0x%02x (len=%d)\n",
                       val, s->async_len);
        } else {
            val = 0;
            WD_DPRINTF("DATA read: FIFO empty\n");
        }
        break;

    default:
        if (reg < WD33C93_REGS) {
            val = s->regs[reg];
        } else {
            val = 0;
        }
        WD_DPRINTF("reg[0x%02x] read: 0x%02x\n", reg, val);
        break;
    }

    /* Auto-increment address register */
    if (auto_inc) {
        s->addr_reg = (s->addr_reg + 1) & 0x1f;
    }

    return val;
}

/*
 * Write to data register port
 * Writes to currently selected register
 *
 * The WD33C93 auto-increments the address register after each data write.
 */
void wd33c93_data_write(WD33C93State *s, uint8_t val)
{
    uint8_t reg = s->addr_reg;

    WD_DPRINTF("reg[0x%02x] write: 0x%02x\n", reg, val);

    switch (reg) {
    case WD_SCSI_STATUS:
    case WD_AUXILIARY_STATUS:
        /* Read-only registers */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wd33c93: write to read-only register 0x%02x\n", reg);
        break;

    case WD_COMMAND:
        /* Writing command register triggers execution */
        s->regs[reg] = val;
        wd33c93_execute_cmd(s, val);
        break;

    case WD_DATA:
        /* Write to data FIFO */
        if (!fifo8_is_full(&s->fifo)) {
            fifo8_push(&s->fifo, val);
        }
        /* Also write to async buffer if active */
        if (s->async_buf && s->async_len > 0) {
            *s->async_buf++ = val;
            s->async_len--;
            if (s->transfer_count > 0) {
                s->transfer_count--;
                wd33c93_set_transfer_count(s, s->transfer_count);
            }
        }
        break;

    default:
        if (reg < WD33C93_REGS) {
            s->regs[reg] = val;
        }
        break;
    }

    /* Auto-increment address register after write */
    s->addr_reg = (s->addr_reg + 1) & 0x1f;
}

/*
 * SCSI bus callback: transfer data available
 *
 * The WD33C93 enforces the Transfer Count (TC) register as the maximum
 * number of bytes to transfer per command.  When the SCSI device offers
 * more data than TC allows, we accept only TC bytes.  When TC is already
 * zero, the transfer phase is complete -- signal command completion and
 * do not accept any more data.
 *
 * This matches MAME's behaviour where decrement_transfer_count() stops
 * the transfer when TC reaches zero, and is required for multi-pass I/O
 * used by sash (the IRIX standalone shell) which issues repeated READ(10)
 * commands, each limited to a TC-sized chunk.
 */
static void wd33c93_transfer_data(SCSIRequest *req, uint32_t len)
{
    WD33C93State *s = req->hba_private;

    WD_DPRINTF("transfer_data: len=%d tc=%d\n", len, s->transfer_count);

    s->async_buf = scsi_req_get_buf(req);

    /*
     * If TC has already reached zero but the SCSI device still has data,
     * raise an "unexpected phase" interrupt so the driver can reprogram
     * TC and DMA descriptors for the next chunk.  This is the normal
     * multi-pass DMA flow on real WD33C93B hardware, used by IRIX for
     * transfers >256KB (64 DMA descriptors × 4KB pages).
     *
     * Reference: IRIX wd93.c:2936 — ST_UNEX_SDATA/RDATA handler
     */
    if (s->transfer_count == 0) {
        qemu_log_mask(LOG_UNIMP, "wd33c93: transfer_data: TC=0, "
                      "raising unexpected-phase IRQ (device offered %u bytes)\n",
                      len);
        s->pending_len = len;
        s->pending_buf = scsi_req_get_buf(req);
        s->async_len = 0;
        s->async_buf = NULL;
        wd33c93_set_drq(s, false);
        s->aux_status &= ~(ASR_DBR | ASR_CIP | ASR_BSY);

        /* MAME: COMMAND_PHASE_TRANSFER_COUNT = 0x46 (IRIX: PH_DATA) */
        s->regs[WD_COMMAND_PHASE] = 0x46;

        /*
         * Status codes are named from the WD33C93 chip's perspective:
         *   UNEX_RDATA (0x48) = chip receiving data = DATA OUT (write)
         *   UNEX_SDATA (0x49) = chip sending data   = DATA IN (read)
         * IRIX wd93.c:2930: ST_UNEX_RDATA with !SCDMA_IN (write),
         *                   ST_UNEX_SDATA with SCDMA_IN (read).
         */
        uint8_t status = (req->cmd.mode == SCSI_XFER_TO_DEV)
                       ? SCSI_STATUS_UNEX_RDATA    /* DATA OUT: chip receives */
                       : SCSI_STATUS_UNEX_SDATA;   /* DATA IN: chip sends  */
        s->scsi_status = status;
        wd33c93_raise_irq(s);
        return;
    }

    /* Cap the transfer to the remaining TC */
    if (len > s->transfer_count) {
        qemu_log_mask(LOG_UNIMP, "wd33c93: transfer_data: capping len %u "
                      "to TC %u (pending %u for multi-pass)\n",
                      len, s->transfer_count, len - s->transfer_count);
        /* Save remainder for multi-pass DMA resume after TC reaches 0 */
        s->pending_len = len - s->transfer_count;
        s->pending_buf = s->async_buf + s->transfer_count;
        len = s->transfer_count;
    }
    s->async_len = len;

    /* Set command phase: data transfer in progress (MAME TRANSFER_COUNT = 0x30) */
    s->regs[WD_COMMAND_PHASE] = 0x30;

    /* Signal data available */
    s->aux_status |= ASR_DBR;

    /* Check DMA mode */
    if ((s->regs[WD_CONTROL] & CONTROL_DM_MASK) != CONTROL_DM_POLLED) {
        wd33c93_set_drq(s, true);
    }
}

/*
 * SCSI bus callback: command complete
 */
static void wd33c93_command_complete(SCSIRequest *req, size_t residual)
{
    WD33C93State *s = req->hba_private;
    uint8_t scsi_status = req->status;

    {
        wd33c93_debug_seq++;
        if (DBG_SEQ_ACTIVE()) {
            FILE *dbg = fopen("/tmp/qemu_scsi_irq_debug.log", "a");
            if (dbg) {
                fprintf(dbg, "[%d] WD93_CMD_COMPLETE: scsi_status=%d residual=%zu "
                        "current_req=%p\n", wd33c93_debug_seq,
                        scsi_status, residual, (void *)s->current_req);
                fclose(dbg);
            }
        }
    }

    WD_DPRINTF("command_complete: status=%d residual=%zu\n",
               scsi_status, residual);

    /* Log CDB details when SCSI command fails (CHECK_CONDITION etc.) */
    if (scsi_status != 0) {
        uint8_t target_id = s->regs[WD_DESTINATION_ID] & DEST_ID_MASK;
        uint8_t cmd_byte = s->regs[WD_CDB_1];
        uint8_t cmd_group = (cmd_byte >> 5) & 0x07;
        int cdb_len_log;
        char cdb_str[64];
        int pos = 0;
        int i;

        switch (cmd_group) {
        case 0:  cdb_len_log = 6;  break;
        case 1:
        case 2:  cdb_len_log = 10; break;
        case 5:  cdb_len_log = 12; break;
        default: cdb_len_log = 6;  break;
        }

        for (i = 0; i < cdb_len_log && pos < (int)sizeof(cdb_str) - 3; i++) {
            pos += snprintf(cdb_str + pos, sizeof(cdb_str) - pos,
                            "%02x ", s->regs[WD_CDB_1 + i]);
        }
        qemu_log_mask(LOG_UNIMP,
                      "wd33c93: SCSI CMD FAILED target=%d status=%d "
                      "CDB[%d]={%s}\n",
                      target_id, scsi_status, cdb_len_log, cdb_str);
    }

    /* Update transfer count */
    wd33c93_set_transfer_count(s, residual);

    /* Clear async and pending state */
    s->async_len = 0;
    s->async_buf = NULL;
    s->pending_len = 0;
    s->pending_buf = NULL;

    /* Lower DRQ */
    wd33c93_set_drq(s, false);
    s->aux_status &= ~ASR_DBR;

    /* Release request */
    scsi_req_unref(s->current_req);
    s->current_req = NULL;

    /* Set command phase: command complete (MAME COMMAND_PHASE_COMMAND_COMPLETE = 0x60) */
    s->regs[WD_COMMAND_PHASE] = 0x60;

    /*
     * Store the SCSI device status in the TARGET_LUN register (low 7 bits),
     * preserving the TLV (bit 7) flag. MAME does this at wd33c9x.cpp:1136-1137.
     * IRIX reads TARGET_LUN to check for CHECK_CONDITION etc.
     */
    s->regs[WD_TARGET_LUN] = (s->regs[WD_TARGET_LUN] & TARGET_LUN_TLV) |
                              (scsi_status & 0x1f);

    /*
     * The WD33C93 always reports SELECT_TRANSFER_SUCCESS in its own status
     * register — the actual SCSI device status is in TARGET_LUN. This matches
     * MAME where command completion always pushes CSR_SELECT_XFER_DONE to
     * the IRQ FIFO (wd33c9x.cpp:1406).
     */
    wd33c93_complete_cmd(s, SCSI_STATUS_SELECT_TRANSFER_SUCCESS);
}

/*
 * SCSI bus callback: request cancelled
 */
static void wd33c93_request_cancelled(SCSIRequest *req)
{
    WD33C93State *s = req->hba_private;

    WD_DPRINTF("request_cancelled\n");

    if (req == s->current_req) {
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
        s->current_dev = NULL;
        s->async_len = 0;
        s->async_buf = NULL;
        s->pending_len = 0;
        s->pending_buf = NULL;
    }
}

/*
 * SCSI bus info structure
 */
static const struct SCSIBusInfo wd33c93_scsi_info = {
    .tcq = false,
    .max_target = 7,
    .max_lun = 7,

    .transfer_data = wd33c93_transfer_data,
    .complete = wd33c93_command_complete,
    .cancel = wd33c93_request_cancelled,
};

/*
 * Device reset
 */
static void wd33c93_reset(DeviceState *dev)
{
    WD33C93State *s = WD33C93(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->addr_reg = 0;
    s->scsi_status = SCSI_STATUS_RESET;
    s->aux_status = 0;
    s->transfer_count = 0;
    s->async_len = 0;
    s->async_buf = NULL;
    s->pending_len = 0;
    s->pending_buf = NULL;
    s->current_dev = NULL;
    s->current_req = NULL;
    s->drq_state = false;

    fifo8_reset(&s->fifo);
}

/*
 * Device realize
 */
static void wd33c93_realize(DeviceState *dev, Error **errp)
{
    WD33C93State *s = WD33C93(dev);

    /* Initialize FIFO */
    fifo8_create(&s->fifo, WD33C93_FIFO_SIZE);

    /* Initialize SCSI bus */
    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &wd33c93_scsi_info);
}

/*
 * Device init
 */
static void wd33c93_init(Object *obj)
{
    WD33C93State *s = WD33C93(obj);

    /* Initialize GPIO outputs */
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, "irq", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->drq, "drq", 1);
}

/*
 * VM state for migration
 */
static bool wd33c93_pending_needed(void *opaque)
{
    WD33C93State *s = opaque;
    return s->pending_len != 0;
}

static const VMStateDescription vmstate_wd33c93_pending = {
    .name = "wd33c93/pending",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = wd33c93_pending_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pending_len, WD33C93State),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_wd33c93 = {
    .name = "wd33c93",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(addr_reg, WD33C93State),
        VMSTATE_UINT8_ARRAY(regs, WD33C93State, WD33C93_REGS),
        VMSTATE_UINT8(scsi_status, WD33C93State),
        VMSTATE_UINT8(aux_status, WD33C93State),
        VMSTATE_FIFO8(fifo, WD33C93State),
        VMSTATE_UINT32(transfer_count, WD33C93State),
        VMSTATE_BOOL(drq_state, WD33C93State),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_wd33c93_pending,
        NULL
    }
};

/*
 * Device class init
 */
static void wd33c93_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = wd33c93_realize;
    device_class_set_legacy_reset(dc, wd33c93_reset);
    dc->vmsd = &vmstate_wd33c93;
}

static const TypeInfo wd33c93_info = {
    .name          = TYPE_WD33C93,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(WD33C93State),
    .instance_init = wd33c93_init,
    .class_init    = wd33c93_class_init,
};

static void wd33c93_register_types(void)
{
    type_register_static(&wd33c93_info);
}

type_init(wd33c93_register_types)
