/*
 * QTest for the WD33C93 disconnect / abort completion contract.
 *
 * The controller sits behind the SGI HPC3 (IP22/IP24) SCSI0 register window at
 * 0x1fbc0000, using indirect addressing: offset 0 is the address port (selects
 * an internal register), offset 4 is the data port.  These tests drive the
 * command register directly and assert the completion semantics the IRIX driver
 * (wd93.c:2491-2508; ARCS scsi.c:2170-2185) relies on:
 *
 *   - CMD_ABORT completes with RESET / RESET_EAF, never SCSI_STATUS_DISCONNECT
 *     (a DISCONNECT status with the command register holding C93ABORT trips
 *     "illegal disconnection interrupt" and resets the SCSI bus).
 *   - Our synthetic CMD_DISCONNECT reports PH_NOSELECT (0x00) in the Command
 *     Phase register, the phase the driver accepts when the command register
 *     still holds C93DISC.
 *
 * This is the environment-independent covering gate for the abort path: a
 * normal Indy boot exercises CMD_DISCONNECT but never CMD_ABORT, and the abort
 * path otherwise has no machine that reaches it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/scsi/wd33c93.h"

/* HPC3 SCSI0 (WD33C93) register ports, big-endian byte access. */
#define HPC3_SCSI0_ADDR  0x1fbc0000ULL
#define HPC3_SCSI0_DATA  0x1fbc0004ULL

#define PH_NOSELECT      0x00

static uint8_t wd_reg_read(QTestState *qts, uint8_t reg)
{
    qtest_writeb(qts, HPC3_SCSI0_ADDR, reg);
    return qtest_readb(qts, HPC3_SCSI0_DATA);
}

static void wd_reg_write(QTestState *qts, uint8_t reg, uint8_t val)
{
    qtest_writeb(qts, HPC3_SCSI0_ADDR, reg);
    qtest_writeb(qts, HPC3_SCSI0_DATA, val);
}

static QTestState *wd_start(void)
{
    /* -S: the guest PROM must not touch SCSI under us. */
    QTestState *qts = qtest_init("-M indy -S -serial none -display none");

    /* Bring the chip to a clean idle state through its own reset command. */
    wd_reg_write(qts, WD_COMMAND, CMD_RESET);
    return qts;
}

static void test_abort_completes_reset(void)
{
    QTestState *qts = wd_start();

    /* The chip must be idle (CIP clear) for the command to be accepted. */
    g_assert_false(wd_reg_read(qts, WD_AUXILIARY_STATUS) & ASR_CIP);

    wd_reg_write(qts, WD_COMMAND, CMD_ABORT);

    uint8_t status = wd_reg_read(qts, WD_SCSI_STATUS);

    g_assert_cmphex(status, !=, SCSI_STATUS_DISCONNECT);
    g_assert_true(status == SCSI_STATUS_RESET ||
                  status == SCSI_STATUS_RESET_EAF);

    /* ... and no ST_DISCONNECT phase is left behind either. */
    g_assert_cmphex(wd_reg_read(qts, WD_COMMAND_PHASE), !=, 0x85);

    qtest_quit(qts);
}

static void test_disconnect_reports_noselect(void)
{
    QTestState *qts = wd_start();

    wd_reg_write(qts, WD_COMMAND, CMD_DISCONNECT);

    /*
     * The completion carries SCSI_STATUS_DISCONNECT, but the Command Phase
     * register must hold PH_NOSELECT (the idle phase the driver accepts when
     * the last command was C93DISC), not a stale phase such as 0x60.
     */
    g_assert_cmphex(wd_reg_read(qts, WD_COMMAND_PHASE), ==, PH_NOSELECT);
    g_assert_cmphex(wd_reg_read(qts, WD_SCSI_STATUS), ==,
                    SCSI_STATUS_DISCONNECT);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/sgi/wd33c93/abort-completes-reset",
                   test_abort_completes_reset);
    qtest_add_func("/sgi/wd33c93/disconnect-reports-noselect",
                   test_disconnect_reports_noselect);
    return g_test_run();
}
