/*
 * SGI HPC1 (High Performance Controller) for IP20 (Indigo) — Stub
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_HPC1_H
#define HW_MISC_SGI_HPC1_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "hw/scsi/wd33c93.h"
#include "qom/object.h"

#define TYPE_SGI_HPC1 "sgi-hpc1"
OBJECT_DECLARE_SIMPLE_TYPE(SGIHPC1State, SGI_HPC1)

struct SGIHPC1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Serial console (stub) */
    CharFrontend serial;

    /* NVRAM backing file (stub) */
    char *nvram_filename;

    /* SCSI controller (WD33C93) — accessed as hpc1->scsi->bus */
    WD33C93State *scsi;

    /* Named IRQ output lines */
    qemu_irq cpu_irq[2];
    qemu_irq timer_irq[2];
};

#endif /* HW_MISC_SGI_HPC1_H */
