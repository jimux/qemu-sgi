/*
 * SGI O2 onboard AIC-7880 PCI SCSI controller — works-in-place
 * placeholder (IP32, -M sgi-o2).
 *
 * The real O2 motherboard carries two Adaptec AIC-7880s behind the
 * MACE PCI host bridge (PCI slots 1 and 2 = SCSI controllers 0 and
 * 1).  The stock IRIX 6.5.5 IP32 kernel drives them with the "adphim"
 * HIM (kern/io/adphim) + adp78 glue, in OPTIMA mode.
 *
 * This is a DELIBERATE placeholder per the O2 plan of record: the
 * register file, sequencer RAM (load + verify), OPTIMA scratch
 * pointers, and the queue-in/queue-out completion protocol are real
 * enough for adphim to attach and run I/O through a QEMU SCSI bus;
 * the sequencer SCRIPTS themselves are NOT interpreted — the
 * driver-observable effects of the sequencer are emulated out of
 * band (see progress_notes/o2_qemu/03-mace-pci-scsi.md).  Strict
 * AIC-7880 fidelity is the last milestone of the effort and will
 * replace the fake sequencer only.
 *
 * Copyright (c) 2026 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SCSI_SGI_AIC7880_H
#define HW_SCSI_SGI_AIC7880_H

#include "hw/pci/pci_device.h"
#include "hw/scsi/scsi.h"
#include "qemu/timer.h"

#define TYPE_SGI_AIC7880 "sgi-aic7880"
OBJECT_DECLARE_SIMPLE_TYPE(SGIAIC7880State, SGI_AIC7880)

/* Sequencer RAM: Seq_01 (OPTIMA) is 1878 bytes; the chip has 4 KB. */
#define AIC7880_SEQRAM_SIZE   4096
/* On-chip SCB banks selected by SCBPTR (driver pokes 0..127). */
#define AIC7880_NUM_SCB_BANKS 128
#define AIC7880_SCB_BANK_SIZE 32
/* Host QOUT_PTR_ARRAY size (bytes) — 64 dword slots. */
#define AIC7880_QOUT_SLOTS    64

struct SGIAIC7880State {
    PCIDevice parent_obj;

    SCSIBus bus;                    /* internal SCSI bus (scsi-hd etc.) */
    MemoryRegion mmio;             /* BAR1: 1 KB register window     */
    MemoryRegion bar0;              /* BAR0: 128 B alias into mmio    */

    uint8_t regs[256];              /* chip register file (chip offs) */
    uint8_t seqram[AIC7880_SEQRAM_SIZE];
    uint32_t seqaddr;               /* SEQRAM auto-increment pointer  */
    uint8_t scb[AIC7880_NUM_SCB_BANKS][AIC7880_SCB_BANK_SIZE];

    /* QINFIFO (host -> sequencer command queue) */
    uint8_t qin[256];
    uint16_t qin_head;
    uint16_t qin_count;

    uint8_t intstat;                /* INTSTAT incl. INTCODE nibble  */

    /*
     * Fake sequencer: QOUT byte slot follows the driver's
     * descending-by-4 order (Ph_checkqout/Ph_OptimaCheckQout start
     * at 4 and subtract 4 per completed entry).
     */
    uint8_t qout_slot;

    QEMUTimer *seq_timer;

    uint32_t scsi_bus_num;          /* -drive if=scsi bus index        */

    /* Current fake-sequencer SCB context */
    uint32_t scb_sram;              /* host addr of the 32-byte seq SCB */
    int cur_scb;                    /* SCB number in flight            */
    SCSIRequest *cur_req;
    uint32_t sg_list;               /* host addr of the SG array        */
    uint8_t sg_count;               /* number of SG elements            */
    uint8_t sg_index;               /* current element                  */
    uint32_t sg_elem_addr;           /* current element data address    */
    uint32_t sg_elem_left;          /* bytes left in current element    */
};

#endif /* HW_SCSI_SGI_AIC7880_H */
