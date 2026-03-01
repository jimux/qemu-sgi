/*
 * SGI HEART (RACER) system controller emulation
 *
 * HEART is the central controller in the SGI Octane (IP30). It handles:
 * - Memory controller (8 SDRAM banks)
 * - Interrupt routing (64 vectors across 5 priority levels)
 * - 52-bit free-running counter timer at 12.5MHz
 * - CPU/memory error reporting
 * - Flow control for graphics
 *
 * Physical base: 0x1FF00000 (XKPHYS: 0x00000000FF000000)
 * Reference: IRIX sys/heart.h, MAME src/mame/sgi/crime.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_HEART_H
#define HW_MISC_SGI_HEART_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_HEART "sgi-heart"
OBJECT_DECLARE_SIMPLE_TYPE(SGIHEARTState, SGI_HEART)

#define HEART_NUM_BANKS 4

/*
 * HEART register offsets from 0x1FF00000
 * All registers are 64-bit aligned and accessed as 64-bit.
 *
 * Note: Linux kernel accesses some registers as 32-bit (readl/writel),
 * but QEMU implementation uses 64-bit for simplicity.
 */
#define HEART_MODE                0x00000
#define HEART_SDRAM_MODE          0x00008
#define HEART_MEM_REF             0x00010
#define HEART_MEM_REQ_ARB         0x00018
#define HEART_MEMCFG0             0x00020
#define HEART_MEMCFG(x)           (HEART_MEMCFG0 + (x) * 8)
#define HEART_FC_MODE             0x00040
#define HEART_FC_TIMER_LIMIT      0x00048
#define HEART_FC0_ADDR            0x00050
#define HEART_FC1_ADDR            0x00058
#define HEART_FC0_CR_CNT          0x00060
#define HEART_FC1_CR_CNT          0x00068
#define HEART_FC0_TIMER           0x00070
#define HEART_FC1_TIMER           0x00078
#define HEART_STATUS              0x00080
#define HEART_BERR_ADDR           0x00088
#define HEART_BERR_MISC           0x00090
#define HEART_MEMERR_ADDR         0x00098
#define HEART_MEMERR_DATA         0x000a0
#define HEART_PIUR_ACC_ERR        0x000a8
#define HEART_MLAN_CLK_DIV        0x000b0
#define HEART_MLAN_CTL            0x000b8

/*
 * QEMU extension: host real-time microseconds counter (QEMU_CLOCK_REALTIME).
 * Same semantics as MC_REALTIME_CTR for IP2x machines.
 * Offset 0x000c0 is unused in real HEART hardware (gap before IRQ regs).
 */
#define HEART_REALTIME_CTR        0x000c0

/* Interrupt registers */
#define HEART_IMR0                0x10000
#define HEART_IMR(x)              (HEART_IMR0 + (x) * 8)
#define HEART_SET_ISR             0x10020
#define HEART_CLR_ISR             0x10028
#define HEART_ISR                 0x10030
#define HEART_IMSR                0x10038
#define HEART_CAUSE               0x10040

/* Timer registers */
#define HEART_COUNT               0x20000  /* 52-bit counter */
#define HEART_COMPARE             0x30000
#define HEART_TRIGGER             0x40000
#define HEART_PRID                0x50000
#define HEART_SYNC                0x60000

/* Total register region size */
#define HEART_REG_SIZE            0x70000

/* HEART MODE register bits */
#define HEART_MODE_PROC_DISABLE(x)   (1ULL << ((x) + 60))
#define HEART_MODE_MAX_PSR           (7ULL << 57)
#define HEART_MODE_MAX_IOSR          (7ULL << 54)
#define HEART_MODE_MAX_PEND_IOSR     (7ULL << 51)
#define HEART_MODE_TRIG_SRC_SEL      (7ULL << 48)
#define HEART_MODE_GP_FLAG(x)        (1ULL << ((x) + 36))
#define HEART_MODE_MAX_PROC_HYST     (15ULL << 32)
#define HEART_MODE_LLP_WRST_AFTER_RST (1ULL << 28)
#define HEART_MODE_LLP_LINK_RST      (1ULL << 27)
#define HEART_MODE_LLP_WARM_RST      (1ULL << 26)
#define HEART_MODE_COR_ECC_LCK       (1ULL << 25)
#define HEART_MODE_REDUCED_PWR       (1ULL << 24)
#define HEART_MODE_COLD_RST          (1ULL << 23)
#define HEART_MODE_SW_RST            (1ULL << 22)
#define HEART_MODE_MEM_FORCE_WR      (1ULL << 21)
#define HEART_MODE_DB_ERR_GEN        (1ULL << 20)
#define HEART_MODE_SB_ERR_GEN        (1ULL << 19)
#define HEART_MODE_CACHED_PIO_EN     (1ULL << 18)
#define HEART_MODE_CACHED_PROM_EN    (1ULL << 17)
#define HEART_MODE_PE_SYS_COR_ERE    (1ULL << 16)
#define HEART_MODE_GLOBAL_ECC_EN     (1ULL << 15)
#define HEART_MODE_IO_COH_EN         (1ULL << 14)
#define HEART_MODE_INT_EN            (1ULL << 13)
#define HEART_MODE_DATA_CHK_EN       (1ULL << 12)
#define HEART_MODE_REF_EN            (1ULL << 11)
#define HEART_MODE_BAD_SYSWR_ERE     (1ULL << 10)
#define HEART_MODE_BAD_SYSRD_ERE     (1ULL << 9)
#define HEART_MODE_SYSSTATE_ERE      (1ULL << 8)
#define HEART_MODE_SYSCMD_ERE        (1ULL << 7)
#define HEART_MODE_NCOR_SYS_ERE      (1ULL << 6)
#define HEART_MODE_COR_SYS_ERE       (1ULL << 5)
#define HEART_MODE_DATA_ELMNT_ERE    (1ULL << 4)
#define HEART_MODE_MEM_ADDR_PROC_ERE (1ULL << 3)
#define HEART_MODE_MEM_ADDR_IO_ERE   (1ULL << 2)
#define HEART_MODE_NCOR_MEM_ERE      (1ULL << 1)
#define HEART_MODE_COR_MEM_ERE       (1ULL << 0)

/* Memory configuration register bits */
#define HEART_MEMCFG_VALID       (1U << 31)
#define HEART_MEMCFG_DENSITY     (0x7U << 22)
#define HEART_MEMCFG_SIZE        (0x3fU << 16)
#define HEART_MEMCFG_BASE        (0x1ffU)

/* Status register bits */
#define HEART_STAT_HSTL_SDRV     (1ULL << 14)
#define HEART_STAT_FC_CR_OUT(x)  (1ULL << ((x) + 12))
#define HEART_STAT_DIR_CNNCT     (1ULL << 11)
#define HEART_STAT_TRITON        (1ULL << 10)
#define HEART_STAT_R4K           (1ULL << 9)
#define HEART_STAT_BIG_ENDIAN    (1ULL << 8)
#define HEART_STAT_PROC_ACTIVE(x) (1ULL << ((x) + 4))
#define HEART_STAT_WIDGET_ID     (0xf)

/* Interrupt level masks (64 vectors total) */
#define HEART_INT_LEVEL4         0xfff8000000000000ULL  /* IP7: vectors 63-51 */
#define HEART_INT_LEVEL3         0x0004000000000000ULL  /* IP6: vector 50 */
#define HEART_INT_LEVEL2         0x0003ffff00000000ULL  /* IP5: vectors 49-32 */
#define HEART_INT_LEVEL1         0x00000000ffff0000ULL  /* IP4: vectors 31-16 */
#define HEART_INT_LEVEL0         0x000000000000ffffULL  /* IP3: vectors 15-0 */

/* Interrupt bits within levels */
#define HEART_INT_HEART_EXC      (1ULL << 63)  /* Level 4 - vector 63 */
#define HEART_INT_PBERR          (0xfULL << 59) /* Level 4 - vectors 62-59 */
#define HEART_INT_WIDGET_ERR     (0xffULL << 51) /* Level 4 - vectors 58-51 */
#define HEART_INT_TIMER          (1ULL << 50)   /* Level 3 - vector 50 */
#define HEART_INT_IPI            (0xfULL << 46) /* Level 2 - vectors 49-46 */
#define HEART_INT_DEBUG          (0xfULL << 42) /* Level 2 - vectors 45-42 */
#define HEART_INT_POWER          (1ULL << 41)   /* Level 2 - vector 41 */
#define HEART_INT_L2_LOCAL       (0x3ffULL)     /* Level 2 - vectors 40-32 */
#define HEART_INT_L1_LOCAL       (0xffffULL)    /* Level 1 - vectors 31-16 */
#define HEART_INT_L0_LOCAL       (0xfff8ULL)    /* Level 0 - vectors 15-3 */
#define HEART_INT_IRQ            (1ULL << 0)    /* Level 0 - vector 0 */

struct SGIHEARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* CPU interrupt output (HEART -> CPU IP7/IP6/IP5/IP4/IP3) */
    qemu_irq cpu_irq[5];

    /* Configuration */
    uint32_t ram_size;
    uint32_t num_cpus;

    /* Registers */
    uint64_t mode;
    uint64_t sdram_mode;
    uint64_t mem_ref;
    uint64_t mem_req_arb;
    uint64_t memcfg[HEART_NUM_BANKS];
    uint64_t fc_mode;
    uint64_t fc_timer_limit;
    uint64_t fc_addr[2];
    uint64_t fc_cr_cnt[2];
    uint64_t fc_timer[2];
    uint64_t status;
    uint64_t berr_addr;
    uint64_t berr_misc;
    uint64_t memerr_addr;
    uint64_t memerr_data;
    uint64_t piur_acc_err;
    uint64_t mlan_clk_div;
    uint64_t mlan_ctl;

    /* Interrupt state */
    uint64_t imr[4];      /* Per-CPU interrupt masks */
    uint64_t set_isr;
    uint64_t clr_isr;
    uint64_t isr;
    uint64_t imsr;
    uint64_t cause;

    /* Timer state */
    uint64_t count;
    uint64_t compare;
    uint64_t trigger;
    uint64_t prid;
    uint64_t sync;

    /* Timer state */
    int64_t time_offset;
};

#endif /* HW_MISC_SGI_HEART_H */