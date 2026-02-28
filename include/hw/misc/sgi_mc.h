/*
 * SGI Memory Controller (MC) emulation
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_MC_H
#define HW_MISC_SGI_MC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_MC "sgi-mc"
OBJECT_DECLARE_SIMPLE_TYPE(SGIMCState, SGI_MC)

#define MC_NUM_BANKS 4

/*
 * Per-bank memory state.
 *
 * The MC supports 4 SIMM banks (2 per MEMCFG register):
 *   Bank 0: MEMCFG0 upper 16 bits
 *   Bank 1: MEMCFG0 lower 16 bits
 *   Bank 2: MEMCFG1 upper 16 bits
 *   Bank 3: MEMCFG1 lower 16 bits
 *
 * Each bank can be independently configured with a base address and size.
 * The MC dynamically maps/unmaps RAM aliases based on MEMCFG writes.
 */
typedef struct SGIMCBankState {
    MemoryRegion *region;       /* Dynamic alias (NULL = not mapped) */
    MemoryRegion *low_alias;    /* 512KB alias at 0x0 when bank at SEG0 */
    MemoryRegion **wrap_aliases; /* Mirror aliases when cfg_size > installed */
    int num_wrap_aliases;
    uint32_t installed_size;    /* Physical RAM size (0 = empty SIMM slot) */
    uint32_t ram_offset;        /* Offset within machine->ram */
    bool mapped;
    uint32_t mapped_base;
} SGIMCBankState;

/*
 * MC Register offsets (from 0x1fa00000)
 * Based on IRIX kernel sys/mc.h
 *
 * SGI registers are 32-bit values at 64-bit aligned addresses.
 * The PROM/kernel may access either the little-endian offset (base)
 * or big-endian offset (base+4) within each 64-bit doubleword.
 * Our read/write handlers normalize addresses with (addr & ~7) so
 * we use the 64-bit aligned base offsets here.
 *
 * Example: CPU_CTRL0 is at 0x00 (LE) or 0x04 (BE), we use 0x00.
 */
#define MC_CPU_CTRL0           0x0000
#define MC_CPU_CTRL1           0x0008
#define MC_WATCHDOG            0x0010
#define MC_SYSID               0x0018
#define MC_RPSS_DIV            0x0028
#define MC_EEPROM              0x0030
#define MC_REFCNT_PRELOAD      0x0040
#define MC_REFCNT              0x0048
#define MC_GIO64_ARB           0x0080
#define MC_ARB_CPU_TIME        0x0088
#define MC_ARB_BURST_TIME      0x0098
#define MC_MEMCFG0             0x00c0
#define MC_MEMCFG1             0x00c8
#define MC_CPU_MEM_ACCESS      0x00d0
#define MC_GIO_MEM_ACCESS      0x00d8
#define MC_CPU_ERR_ADDR        0x00e0
#define MC_CPU_ERR_STATUS      0x00e8
#define MC_GIO_ERR_ADDR        0x00f0
#define MC_GIO_ERR_STATUS      0x00f8
#define MC_SYS_SEMAPHORE       0x0100
#define MC_GIO_LOCK            0x0108
#define MC_EISA_LOCK           0x0110
#define MC_GIO64_XLATE_MASK    0x0150
#define MC_GIO64_SUBST_BITS    0x0158
#define MC_DMA_INT_CAUSE       0x0160
#define MC_DMA_CONTROL         0x0168
#define MC_DMA_TLB_HI0         0x0180
#define MC_DMA_TLB_LO0         0x0188
#define MC_DMA_TLB_HI1         0x0190
#define MC_DMA_TLB_LO1         0x0198
#define MC_DMA_TLB_HI2         0x01a0
#define MC_DMA_TLB_LO2         0x01a8
#define MC_DMA_TLB_HI3         0x01b0
#define MC_DMA_TLB_LO3         0x01b8

/*
 * QEMU extension: host real-time microseconds counter.
 * Returns QEMU_CLOCK_REALTIME microseconds, wrapping at 2^32 (~71 min).
 * Unlike MC_RPSS_CTR (virtual clock), this never races under -icount sleep=off.
 * Used by patched IRIX kernel for networking/animation timing.
 * Offset 0x0050 is unused in real SGI MC hardware (gap between MC_REFCNT and MC_GIO64_ARB).
 */
#define MC_REALTIME_CTR        0x0050

/* RPSS counter is in a separate page */
#define MC_RPSS_CTR            0x1000

/* DMA registers are in another page */
#define MC_DMA_MEM_ADDR        0x2000
#define MC_DMA_MEM_ADDR_DEF    0x2008
#define MC_DMA_SIZE            0x2010
#define MC_DMA_STRIDE          0x2018
#define MC_DMA_GIO_ADDR        0x2020
#define MC_DMA_GIO_ADDR_START  0x2028
#define MC_DMA_MODE            0x2030
#define MC_DMA_COUNT           0x2038
#define MC_DMA_START           0x2040
#define MC_DMA_RUN             0x2048

/* Semaphores are at 0x10000+ */
#define MC_SEMAPHORE_BASE      0x10000
#define MC_SEMAPHORE_STRIDE    0x1000

/* Total MC region size to cover semaphores */
#define MC_REG_SIZE            0x20000

struct SGIMCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion *system_memory; /* Reference to system memory */
    MemoryRegion *ram;          /* Reference to machine RAM */
    uint64_t last_rpss_update;  /* For RPSS counter timing */

    /* Configuration */
    uint32_t ram_size;          /* RAM size in bytes (set by machine) */
    bool has_eisa;              /* EISA bus present (Full House/IP22) */
    uint8_t revision;           /* MC revision (< 5: shift 22, >= 5: shift 24) */
    SGIMCBankState banks[MC_NUM_BANKS]; /* Per-bank memory state */

    /* Registers */
    uint32_t cpu_ctrl[2];
    uint32_t watchdog;
    uint32_t sysid;
    uint32_t rpss_div;
    uint32_t eeprom_ctrl;
    uint32_t refcnt_preload;
    uint32_t refcnt;
    uint32_t gio64_arb;
    uint32_t arb_cpu_time;
    uint32_t arb_burst_time;
    uint32_t memcfg[2];
    uint32_t cpu_mem_access;
    uint32_t gio_mem_access;
    uint32_t cpu_err_addr;
    uint32_t cpu_err_status;
    uint32_t gio_err_addr;
    uint32_t gio_err_status;
    uint32_t sys_semaphore;
    uint32_t gio_lock;
    uint32_t eisa_lock;
    uint32_t gio64_xlate_mask;
    uint32_t gio64_subst_bits;
    uint32_t dma_int_cause;
    uint32_t dma_control;
    uint32_t dma_tlb_hi[4];
    uint32_t dma_tlb_lo[4];
    uint32_t rpss_ctr;
    uint32_t dma_mem_addr;
    uint32_t dma_size;
    uint32_t dma_stride;
    uint32_t dma_gio_addr;
    uint32_t dma_mode;
    uint32_t dma_count;
    uint32_t dma_run;
    uint32_t semaphore[16];
};

#endif /* HW_MISC_SGI_MC_H */
