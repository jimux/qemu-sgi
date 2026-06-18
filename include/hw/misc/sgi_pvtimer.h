/*
 * SGI Paravirtual 66MHz Free-Running Timer
 *
 * Provides a free-running 64-bit counter incrementing at 66MHz.
 * The PROM's usecdelay.s resets the counter by writing 0, then polls
 * until the elapsed count exceeds (usec * 66).
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_PVTIMER_H
#define HW_MISC_SGI_PVTIMER_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_PVTIMER "sgi-pvtimer"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPVTimerState, SGI_PVTIMER)

#define SGI_PVTIMER_MMIO_SIZE    0x100
/* Single 64-bit counter register at offset 0x38 */
#define SGI_PVTIMER_COUNTER_REG  0x38

struct SGIPVTimerState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /*
     * Subtracted from raw ticks on each read.
     * Write of value V sets base = raw_now - V, so subsequent reads
     * return elapsed ticks + V (PROM writes 0 to reset to 0).
     */
    uint64_t base;
};

#endif /* HW_MISC_SGI_PVTIMER_H */
