/*
 * SGI VICE (Video, Imaging and Compression Engine) emulation
 *
 * VICE is the O2 (IP32) media ASIC: a programmable MSP (media DSP) plus a
 * BSP (bitstream RISC) plus two DMA channels. This is the tier-i skeleton:
 * it models the 1 MB host PIO window (registers, IRAM/DRAM/BSP table, TLB,
 * DMA descriptor storage) and the boot-probe interrupt handshake, but does
 * not interpret MSP/BSP microcode and does not move DMA data.
 *
 * Physical base: 0x17000000 (kseg1: 0xB7000000), 1 MB window.
 * Register access lanes [vice_drv.h]:
 *   32-bit registers live in the big-endian high lane, at offset + 4
 *   16-bit BSP/DMA registers at offset + 6
 *   IRAM/DRAM/table/TLB are accessed at the bare offset
 *
 * References:
 *   - VICE Design Specification 099-0123-003 (April 17, 1997)
 *   - IRIX sys/vice/{vice_drv.h,vice_regs_host.h,vice_dma.h}
 *   - IRIX kern/ml/MOOSEHEAD/{IP32init.c,vice_copy.c}
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_VICE_H
#define HW_MISC_SGI_VICE_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_VICE "sgi-vice"
OBJECT_DECLARE_SIMPLE_TYPE(SGIViceState, SGI_VICE)

#define VICE_REG_SIZE       0x100000    /* host PIO window at 0x17000000 */

/* On-chip register block (0x0000-0x1fff) backed as big-endian bytes. */
#define VICE_CHIP_REGS_SIZE 0x2000

/* RAM/table/TLB backing store sizes [vice_regs_host.h]. */
#define VICE_MSP_IRAM_SIZE  0x1000      /* VICEMSP_IRAM   @ 0x2000 */
#define VICE_BSP_IRAM_SIZE  0x0800      /* VICEBSP_IRAM   @ 0x4000 */
#define VICE_BSP_TABLE_SIZE 0x2000      /* VICEBSP_TABLE  @ 0x5000 */
#define VICE_BSP_FIFO_SIZE  0x0040      /* in/out FIFO    @ 0x7000/0x7800 */
#define VICE_MSP_DRAM_SIZE  0x1800      /* VICEMSP_DRAM   @ 0x8000 */
#define VICE_NTLBENTRIES    0x0080      /* 128 entries (rev.h) @ 0xf000 */
/*
 * Each TLB entry is a 32-bit value (spec 099-0123-003 Table 5: Valid,
 * Writable, 64k Physical Page Number) but the host PIO register slots are
 * 64-bit, so entry N lives at 0xf000 + N*8 with the value in the big-endian
 * high (32-bit register) lane at +4. Observed in a live movie-JPEG run:
 *   0xf004=entry 0 (VICE_DMS_IN) phys 0x13900000
 *   0xf204=entry 64 (VICE_DMS_OUT) phys 0x13b00000
 *   0xf3f4=entry 126 (VICE_DMS_AUX) phys 0x13800000
 * Modelling 4-byte stride would silently drop every mapping >= entry 64.
 */
#define VICE_TLB_STRIDE     8
#define VICE_TLB_SIZE       (VICE_NTLBENTRIES * VICE_TLB_STRIDE)

/*
 * Chip register byte offsets (from the host PIO window base). Access is
 * via the high lanes: a 32-bit register read/write lands at offset + 4,
 * a 16-bit register at offset + 6. The model normalizes the lane so both
 * map onto the same byte slot here.
 */
#define VICE_ID             0x0008      /* part/revision ID; RO e3 Vice-C TRE */
#define HST_BSP_IN_BOX      0x0028
#define HST_BSP_OUT_BOX     0x0030
#define MSP_CTL_STAT        0x0040      /* bit0 GO/HALT (RO 0 = halted) */
#define MSP_ExcpFlag        0x0048
#define MSP_PC              0x0050
#define MSP_BadAddr         0x0058
#define MSP_WatchPoint      0x0060
#define MSP_EPC             0x0068
#define MSP_CAUSE           0x0070
#define BSP_RPAGE           0x0078
#define BSP_SW_INT          0x0080      /* write-only: sets VICE_INT bit 4 */
#define MSP_D_RAM           0x0100
#define VICE_COUNT          0x0108      /* free-running VICE clock counter */
#define BSP_CTL_STAT        0x0110      /* bit1 HALT, bit2 HALT_ACK */
#define BSP_WatchPoint      0x0118
#define BSP_IN_COUNT        0x0120
#define BSP_OUT_COUNT       0x0128
#define BSP_PC              0x0140
#define BSP_EPC             0x0148
#define BSP_HALT_RESET      0x0150      /* write bit0 -> reset+halt BSP */
#define BSP_CAUSE           0x0158
#define VICE_INT            0x0160      /* interrupt status, RO */
#define BSP_FIFO_CTL_STAT   0x0168
#define BSP_AVALID_BITS     0x0170
#define BSP_FVALID_BITS     0x0178
#define VICEDMA_CTL_CH1     0x0180
#define VICEDMA_STAT_CH1    0x0188
#define VICEDMA_DATA_CH1    0x0190
#define VICEDMA_MEM_PT_CH1  0x0198
#define VICEDMA_VICE_PT_CH1 0x01a0
#define VICEDMA_COUNT_CH1   0x01a8
#define MSP_SW_INT          0x01b8      /* write-only: sets VICE_INT bit 2 */
#define VICEDMA_CTL_CH2     0x01c0
#define BSP_IN_BOX          0x01f0
#define BSP_OUT_BOX         0x01f8

/* DMA descriptor block: 2 channels x 4 descriptors x 8 registers @ 0x1000 */
#define VICE_DESC_BASE      0x1000
#define VICE_DESC_END       0x1200

/* Config / interrupt control block. */
#define VICE_CFG            0xe000
#define VICE_INT_RESET      0xe008      /* write-1-to-clear VICE_INT */
#define VICE_INT_EN         0xe010      /* interrupt enable, r/w */

/* MSP/BSP debug and exception-stamp registers (tier-ii territory). */
#define VICE_DEBUG_BASE     0x11000
#define VICE_DEBUG_END      0x13000

/* VICE_INT / VICE_INT_EN bits [vice_regs_host.h]. */
#define VICE_INT_DMA1_DONE  0x0001
#define VICE_INT_DMA1_ERR   0x0002
#define VICE_INT_MSP_INTR   0x0004
#define VICE_INT_MSP_EXC    0x0008
#define VICE_INT_BSP_INTR   0x0010
#define VICE_INT_BSP_EXC    0x0020
#define VICE_INT_SYSADERR   0x0040
#define VICE_INT_DMA2_DONE  0x0080
#define VICE_INT_DMA2_ERR   0x0100
#define VICE_INT_ALL        0x01ff

/* BSP_CTL_STAT bits [vice_regs_host.h]. */
#define VICEBSPCS_RESET     0x0001
#define VICEBSPCS_HALT      0x0002
#define VICEBSPCS_HALT_ACK  0x0004

/*
 * VICE_ID value (spec 099-0123-003 §2.9.1 TABLE 8): low nibble is the
 * part mask (1=Vice-A, 2=Vice-B "DX", 3=Vice-C "TRE"), high nibble is the
 * chip revision. The reset value is 0xE1 (Vice-A/"EN"), but the SDK/disk
 * shipped here is the Vice-C "TRE" part, so the authentic value is 0xE3.
 *
 * @@SEMANTICS@@ identification: VICE_ID is the only thing that selects the
 * codec directory. libvice's vice_file() (independently disassembled:
 * 0xE1 -> "/var/arch/vice/", 0xE2 -> "/var/arch/vicedx/", default ->
 * "/var/arch/vicetre/") commits to a directory without probing it. With the
 * 0xE1 reset value the guest looks in the absent /var/arch/vice/ and every
 * codec load fails ("libvice: getcode failed ..."); the disk only ships
 * /var/arch/vicetre/ (and /var/arch/vicedx/), i.e. the Vice-C part. Return
 * the part the disk actually carries so the native directory is selected.
 */
#define VICE_ID_VALUE       0x000000e3

/* DMA channel control GO bit (VICEDMA_CTL_GO). */
#define VICEDMA_CTL_GO      0x0001

/*
 * DMS TLB entry slots [vice_drv.h]: the input block is always mapped at entry
 * 0, the output at 64 and the auxiliary (quantisation table) at 126 when a
 * dmedia converter runs a VP (vice process).
 */
#define VICE_DMS_IN         0
#define VICE_DMS_OUT        64
#define VICE_DMS_AUX        126

struct SGIViceState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq crime_irq[1];

    /* 0x0000-0x1fff register/descriptor block, big-endian byte storage. */
    uint8_t chip_regs[VICE_CHIP_REGS_SIZE];

    /* On-chip RAMs (bare-offset access). */
    uint8_t msp_iram[VICE_MSP_IRAM_SIZE];
    uint8_t bsp_iram[VICE_BSP_IRAM_SIZE];
    uint8_t bsp_table[VICE_BSP_TABLE_SIZE];
    uint8_t bsp_out_fifo[VICE_BSP_FIFO_SIZE];
    uint8_t bsp_in_fifo[VICE_BSP_FIFO_SIZE];
    uint8_t msp_dram[VICE_MSP_DRAM_SIZE];
    uint8_t tlb[VICE_TLB_SIZE];
    uint8_t debug_regs[VICE_DEBUG_END - VICE_DEBUG_BASE];

    /* VICE_CFG, interrupt control/status, and BSP run state. */
    uint32_t cfg;
    uint32_t int_enable;
    uint32_t int_status;
    uint32_t bsp_ctl_stat;

    /*
     * @@SEMANTICS@@ tier-ii host-codec offload transport.  The external
     * JPEG helper is spawned once (codec-helper) and spoken to over the
     * "vice-codec" chardev, exactly as the O2 video seam drives its decoder
     * helpers.  codec-path is the Unix socket path the helper connects to
     * (the chardev property does not expose the backend path).
     */
    CharFrontend codec_chr;
    char *codec_helper;
    char *codec_path;
    GPid helper_pid;
    guint helper_watch;
};

#endif /* HW_MISC_SGI_VICE_H */
