/*
 * SGI O2 onboard AIC-7880 PCI SCSI controller — strict sequencer
 * (IP32, -M sgi-o2).  M12a: the downloaded Seq_01 microcode is
 * INTERPRETED (fetch/decode/execute of the 4-byte-word ISA documented
 * in progress_notes/o2_qemu/35-aic-isa.md); the fake timer-driven
 * sequencer is gone.
 *
 * The real O2 motherboard carries two Adaptec AIC-7880s behind the
 * MACE PCI host bridge (PCI slots 1 and 2 = SCSI controllers 0 and
 * 1).  The stock IRIX 6.5.5 IP32 kernel drives them with the
 * "adphim" HIM (kern/io/adphim) + adp78 glue, in OPTIMA mode.
 *
 * Register model: the AIC-7880 is a little-endian PCI device with a
 * 256-byte register file.  The IRIX driver (MIPS_BE build) reaches
 * register N at bus offset conv(N) = (N & ~3) | (3 - (N & 3)) — the
 * MACE presents the big-endian SGI bus to the CPU and the driver
 * pre-swaps every byte offset (him_equ.h MIPS_BE values, osm.c
 * osm_conv_byteptr).  This device applies the same conversion on
 * every BAR access and implements the register file at chip offsets.
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

/* Sequencer RAM: Seq_01 (OPTIMA) is 1708/1696 bytes; the chip has 4 KB. */
#define AIC7880_SEQRAM_SIZE   4096
/* On-chip SCB banks selected by SCBPTR (driver pokes 0..127). */
#define AIC7880_NUM_SCB_BANKS 128
#define AIC7880_SCB_BANK_SIZE 32
/* Data FIFO depth (classic AIC-78xx: 16 bytes host + 16 SCSI). */
#define AIC7880_FIFO_SIZE     64

/* Sequencer run-slice budget: instructions per scheduler invocation. */
#define AIC7880_SEQ_BUDGET    512

/* Subroutine stack depth (the ISA has one CALL level per the 14 uses). */
#define AIC7880_STACK_DEPTH   4

struct SGIAIC7880State {
    PCIDevice parent_obj;

    SCSIBus bus;                    /* internal SCSI bus (scsi-hd etc.) */
    MemoryRegion mmio;             /* BAR1: 1 KB register window     */
    MemoryRegion bar0;              /* BAR0: 256 B alias into mmio    */

    uint8_t regs[256];              /* chip register file (chip offs) */
    uint8_t seqram[AIC7880_SEQRAM_SIZE];
    uint32_t seqaddr;               /* SEQRAM auto-increment pointer  */
    uint8_t scb[AIC7880_NUM_SCB_BANKS][AIC7880_SCB_BANK_SIZE];

    /* QINFIFO (host -> sequencer command queue; read = dequeue) */
    uint8_t qin[256];
    uint16_t qin_head;
    uint16_t qin_count;

    uint8_t intstat;                /* INTSTAT incl. INTCODE nibble  */

    /*
     * Strict sequencer state.  seq_pc is the live program counter
     * (4-byte word index into seqram); SEQADDR0/1 shadow it in regs[]
     * for the host.  paused covers BOTH the host PAUSE (HCNTRL) and
     * the sequencer's own INTSTAT SEQINT pause.
     */
    uint32_t seq_pc;
    bool seq_running;               /* LOADRAM clear + program present */
    bool seq_host_pause;            /* HCNTRL PAUSE requested by host */
    bool seq_int_pause;             /* sequencer paused itself (SEQINT) */
    uint8_t seq_flags;              /* ZERO 0x02 | CARRY 0x01 */
    uint16_t seq_stack[AIC7880_STACK_DEPTH];
    uint8_t seq_sp;

    /* Data FIFO (DFDAT/DFSTATUS) + host DMA engine */
    uint8_t fifo[AIC7880_FIFO_SIZE];
    uint16_t fifo_head, fifo_tail, fifo_count;
    bool fifo_reset_latch;          /* DFCNTRL FIFORESET seen */
    bool hdma_done;                 /* DFSTATUS HDONE model  */

    /* SCSI bus / phase engine */
    uint8_t cur_phase;              /* current SCSI bus phase (CDO|IOO|MSGO) */
    bool bus_selected;

    /* Current SCSI request (bus side) */
    int cur_scb;                    /* SCB number in flight            */
    SCSIRequest *cur_req;
    uint32_t sg_list;               /* host addr of the SG array        */
    uint8_t sg_count;               /* number of SG elements            */
    uint8_t sg_index;               /* current element                  */
    uint32_t sg_elem_addr;          /* current element data address    */
    uint32_t sg_elem_left;          /* bytes left in current element    */

    /* Trace pacing: fetch counter for bounded-rate seq_fetch events */
    uint32_t seq_fetch_count;

    /* Last host-fetched SCB number (bank-staleness guard) */
    uint8_t last_fetched_scb;

    /*
     * Sequencer PC redirect: a device action inside an ALU write
     * hook (the SELTO path returns the sequencer to the idle loop)
     * sets this; the interpreter's PC commit honors it over the
     * natural successor.  -1 = no redirect.
     */
    int32_t seq_redirect;

    /* Deferred completion posting (timer-deferred, see .complete) */
    bool done_pending;
    int done_status;
    int done_scb;

    /*
     * QOUT completion slot byte: the sequencer's r56 arithmetic
     * (scratch 0x56 = low byte of the driver's 256-aligned QOUT
     * pointer, doubling as the slot index) — descending by 4.
     */
    uint8_t qout_slot;

    /*
     * The 256-aligned QOUT ring base as the driver programmed it
     * (Ph_MovPtrToScratch, himdiopt.c:470).  Kept separate from the
     * scratch dword because the sequencer's r56 byte (mirrored into
     * regs[0x56] as on silicon) would otherwise corrupt the pointer
     * the completion path uses for its host writes — the driver
     * reads the ring through its own fixed kernel address (himd.c:467)
     * and never re-reads the scratch, so only the device needs the
     * pristine base.
     */
    uint32_t qout_base;

    QEMUTimer *seq_timer;           /* scheduler kick (idle re-arm only) */

    uint32_t scsi_bus_num;          /* -drive if=scsi bus index        */
};

#endif /* HW_SCSI_SGI_AIC7880_H */
