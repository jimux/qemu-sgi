/*
 * SGI VINO (Indy video input) — register shell.
 *
 * VINO is the Indy's (IP24) on-board video-input ASIC: the IndyCam digital
 * port plus composite/S-Video analog inputs.  Its register block lives in
 * EISA address space at physical 0x00080000 (IRIX sys/IP22.h
 * VINO_PHYS_BASE1; promoted by IP22.c simplevinoprobe()).
 *
 * Presence is a device property, default OFF: with present=off the region
 * reads all-ones exactly like an unmapped aperture, so a plain `-M indy`
 * invocation is byte-for-byte unchanged.  present=on makes the version
 * register answer so the PROM/kernel VINO probe recognises the part.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_VINO_H
#define HW_MISC_SGI_VINO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_VINO "sgi-vino"
OBJECT_DECLARE_SIMPLE_TYPE(SGIVinoState, SGI_VINO)

/* VINO register aperture.  Pinned from the IRIX 5.3 vino.a driver
 * (vino_regs.o master_reg_tbl): 39 direct-MMIO registers id 0..0x26 at an
 * 8-byte stride with the data word at +4, so the highest direct access is
 * 0x26*8+4 = 0x134.  The PROM diagnostics additionally touch +0x42/+0x43/
 * +0x61.  0x200 covers every access seen; grow only against new evidence. */
#define SGI_VINO_REG_SIZE 0x200

/* Version register (physical base + 0x04). */
#define SGI_VINO_REV_OFFSET   0x04
#define SGI_VINO_CHIP_ID      0x0B
#define SGI_VINO_REVISION     0x01

struct SGIVinoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bool present;
    uint8_t regs[SGI_VINO_REG_SIZE];
};

#endif /* HW_MISC_SGI_VINO_H */
