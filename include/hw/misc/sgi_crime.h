/*
 * SGI CRIME (CPU, Rendering, I/O, and Memory Engine) emulation
 *
 * CRIME is the central controller in the SGI O2 (IP32). It handles:
 * - Memory controller (8 SDRAM banks)
 * - Interrupt routing (MACE, GBE, RE, VICE, software)
 * - Free-running 66.67MHz timer (CRM_TIME)
 * - CPU/memory error reporting
 * - Watchdog timer
 *
 * Physical base: 0x14000000
 * Reference: IRIX sys/crime.h, MAME src/mame/sgi/crime.cpp
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

/* CRIME control register bits */
#define CRM_CONTROL_ENDIAN_BIG  0x0100

/* CRIME master frequency: 66.67 MHz */
#define CRIME_MASTER_FREQ       66666500
/* Nanoseconds per CRIME tick */
#define CRIME_NS_PER_TICK       15

/* Interrupt bits */
#define CRM_INT_VICE            0x80000000ULL
#define CRM_INT_SOFT2           0x40000000ULL
#define CRM_INT_SOFT1           0x20000000ULL
#define CRM_INT_SOFT0           0x10000000ULL
#define CRM_INT_MACE_MASK       0x0000ffffULL

/*
 * Memory bank control register format:
 *   Bits [4:0]: Bank base address (physical >> 25)
 *   Bit  [8]:   SDRAM size (0 = 32MB, 1 = 64MB per bank)
 */
#define CRM_BANK_CTRL_ADDR_MASK 0x1f
#define CRM_BANK_CTRL_SDRAM_SIZE 0x100

struct SGICRIMEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* CPU interrupt output (CRIME -> CPU IP2) */
    qemu_irq cpu_irq;

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
