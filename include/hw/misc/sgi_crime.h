/*
 * SGI CRIME (CPU, Rendering, I/O, and Memory Engine) emulation
 *
 * CRIME is the central controller in the SGI O2 (IP32). It handles:
 * - Memory controller (8 SDRAM banks)
 * - Interrupt routing (MACE, GBE, RE, VICE, software)
 * - Free-running 66.67MHz timer (CRM_TIME)
 * - CPU/memory error reporting
 * - Watchdog timer (McGriff)
 *
 * Physical base: 0x14000000
 * Reference: IRIX sys/crime.h, sys/IP32.h, MAME src/mame/sgi/crime.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_CRIME_H
#define HW_MISC_SGI_CRIME_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_CRIME "sgi-crime"
OBJECT_DECLARE_SIMPLE_TYPE(SGICRIMEState, SGI_CRIME)

#define CRIME_NUM_BANKS 8

/*
 * CRIME register offsets (from 0x14000000)
 * Based on IRIX sys/crime.h
 *
 * All registers are 64-bit aligned. The PROM accesses them as
 * 64-bit reads/writes at these offsets.
 */
#define CRM_ID                  0x000
#define CRM_CONTROL             0x008
#define CRM_INTSTAT             0x010
#define CRM_INTMASK             0x018
#define CRM_SOFTINT             0x020
#define CRM_HARDINT             0x028
#define CRM_DOG                 0x030
#define CRM_TIME                0x038
#define CRM_CPU_ERROR_ADDR      0x040
#define CRM_CPU_ERROR_STAT      0x048
#define CRM_CPU_ERROR_ENA       0x050
#define CRM_VICE_ERROR_ADDR     0x058

/*
 * QEMU extension: host real-time microseconds counter (QEMU_CLOCK_REALTIME).
 * Same semantics as MC_REALTIME_CTR for IP2x machines.
 * Offset 0x060 is unused in real CRIME hardware (gap before memory regs).
 */
#define CRM_REALTIME_CTR        0x060

#define CRM_MEM_CONTROL         0x200
#define CRM_MEM_BANK_CTRL0      0x208
/* Banks 0-7 at 0x208, 0x210, 0x218, ..., 0x240 (8-byte stride) */
#define CRM_MEM_BANK_CTRL(x)    (0x208 + ((x) * 8))
#define CRM_MEM_REFRESH_CNTR    0x248
#define CRM_MEM_ERROR_STAT      0x250
#define CRM_MEM_ERROR_ADDR      0x258
#define CRM_MEM_ERROR_ECC_SYN   0x260
#define CRM_MEM_ERROR_ECC_CHK   0x268
#define CRM_MEM_ERROR_ECC_REPL  0x270

/* Total region size */
#define CRIME_REG_SIZE          0x280

/* CRIME ID value: ID bits = 0xa0, revision = 0x02 */
#define CRIME_ID_VALUE          0xa2

/* CRIME control register bits [sys/crime.h] */
#define CRM_CONTROL_ENDIAN_BIG  0x0100
#define CRM_CONTROL_DOG_ENA     0x0200
#define CRM_CONTROL_SOFT_RESET  0x0400
#define CRM_CONTROL_HARD_RESET  0x0800

/* CRIME master frequency: 66.67 MHz */
#define CRIME_MASTER_FREQ       66666500
/* Nanoseconds per CRIME tick */
#define CRIME_NS_PER_TICK       15

/*
 * CRIME interrupt source numbering, 32 sources [sys/IP32.h]:
 *   0-15  = MACE (CRM_INT_MACE(i) = 1<<i)
 *   16-19 = GBE0..GBE3 (retrace, preblank, GBE2, GBE3)
 *   20    = CRMERR
 *   21    = MEMERR
 *   22-27 = RE0..RE5 (RE3 = FIFO empty watermark, RE4 = FIFO full)
 *   28-30 = SOFT0..SOFT2
 *   31    = VICE
 * Bit masks [sys/crime.h]: GBE0 0x10000 .. GBE3 0x80000, CRMERR 0x100000,
 * MEMERR 0x200000, RE0 0x400000 .. RE5 0x8000000, SOFT0 0x10000000, VICE
 * 0x80000000.
 */
#define CRM_INT_GBE0            0x00010000ULL
#define CRM_INT_GBE1            0x00020000ULL
#define CRM_INT_GBE2            0x00040000ULL
#define CRM_INT_GBE3            0x00080000ULL
#define CRM_INT_CRMERR          0x00100000ULL
#define CRM_INT_MEMERR          0x00200000ULL
#define CRM_INT_RE0             0x00400000ULL
#define CRM_INT_RE1             0x00800000ULL
#define CRM_INT_RE2             0x01000000ULL
#define CRM_INT_RE3             0x02000000ULL
#define CRM_INT_RE4             0x04000000ULL
#define CRM_INT_RE5             0x08000000ULL
#define CRM_INT_SOFT0           0x10000000ULL
#define CRM_INT_SOFT1           0x20000000ULL
#define CRM_INT_SOFT2           0x40000000ULL
#define CRM_INT_VICE            0x80000000ULL
#define CRM_INT_MACE_MASK       0x0000ffffULL

/* Index (line number) of each gpio-in source; == bit position [sys/IP32.h] */
#define CRM_IRQ_GBE0            16
#define CRM_IRQ_GBE1            17
#define CRM_IRQ_GBE2            18
#define CRM_IRQ_GBE3            19
#define CRM_IRQ_CRMERR          20
#define CRM_IRQ_MEMERR          21
#define CRM_IRQ_RE0             22
#define CRM_IRQ_RE1             23
#define CRM_IRQ_RE2             24
#define CRM_IRQ_RE3             25
#define CRM_IRQ_RE4             26
#define CRM_IRQ_RE5             27
#define CRM_IRQ_SOFT0           28
#define CRM_IRQ_SOFT1           29
#define CRM_IRQ_SOFT2           30
#define CRM_IRQ_VICE            31

/* Number of gpio-in interrupt source lines (MACE 0-15 + the 16 above) */
#define CRM_NUM_IRQS            32

/*
 * McGriff watchdog [sys/crime.h]: 21-bit down-counter.
 * CRM_DOG_POWER_ON_RESET / WARM_RESET are sticky status bits that
 * report why the last reset happened; the counter field is 0x7fff.
 */
#define CRM_DOG_POWER_ON_RESET  0x100000ULL
#define CRM_DOG_WARM_RESET      0x080000ULL
#define CRM_DOG_VALUE           0x7fffULL

/*
 * Memory bank control register format:
 *   Bits [4:0]: Bank base address (physical >> 25, units of 32MB)
 *   Bit  [8]:   SDRAM size (0 = 32MB per bank, 1 = 128MB per bank)
 */
#define CRM_BANK_CTRL_ADDR_MASK 0x1f
#define CRM_BANK_CTRL_SDRAM_SIZE 0x100

struct SGICRIMEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* CPU interrupt output (CRIME -> CPU IP2) */
    qemu_irq cpu_irq;

    /* Watchdog timer (McGriff), NULL until enabled */
    QEMUTimer *dog_timer;
    bool dog_enabled;

    /* Configuration */
    uint32_t ram_size;

    /* Registers */
    uint64_t id;
    uint64_t control;
    uint64_t intstat;
    uint64_t intmask;
    uint64_t softint;
    uint64_t hardint;
    uint64_t watchdog;
    int64_t time_offset;    /* Guest-written offset for CRM_TIME */
    uint64_t last_time_read; /* Last value returned from CRM_TIME */
    uint64_t cpu_error_addr;
    uint64_t cpu_error_stat;
    uint64_t cpu_error_ena;
    uint64_t vice_error_addr;
    uint64_t mem_control;
    uint64_t mem_bank_ctrl[CRIME_NUM_BANKS];
    uint64_t mem_refresh_cntr;
    uint64_t mem_error_stat;
    uint64_t mem_error_addr;
    uint64_t mem_error_ecc_syn;
    uint64_t mem_error_ecc_chk;
    uint64_t mem_error_ecc_repl;
};

#endif /* HW_MISC_SGI_CRIME_H */
