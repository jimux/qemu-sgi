/*
 * SGI CRIME Rendering Engine (RE) stub
 *
 * The CRIME rendering engine occupies physical 0x15000000-0x15004FFF on the
 * SGI O2 (IP32). It provides the graphics pipeline, framebuffer TLBs,
 * MTE (Memory Transfer Engine), and status registers.
 *
 * This stub returns "idle" for all status queries and accepts all writes
 * silently, allowing the IRIX kernel's graphics driver to probe and
 * initialize without hanging.
 *
 * Register regions (offset from 0x15000000):
 *   0x0000-0x0FFF  Interface Buffer (command FIFO)
 *   0x1000-0x17FF  TLB (framebuffer, texture, CID, linear)
 *   0x2000-0x21FF  Pixel Pipe (drawing pipeline registers)
 *   0x3000-0x307F  MTE (Memory Transfer Engine)
 *   0x4000-0x400F  Status/Control
 *
 * Reference: IRIX sys/crimereg.h, crimedef.h
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_CRIME_RE_H
#define HW_MISC_SGI_CRIME_RE_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_CRIME_RE "sgi-crime-re"
OBJECT_DECLARE_SIMPLE_TYPE(SGICRIMEREState, SGI_CRIME_RE)

/* Total region size: 5 pages (0x0000-0x4FFF) */
#define CRIME_RE_SIZE           0x5000

/* Region offsets from RE base (0x15000000) */
#define CRM_RE_INTFBUF_BASE     0x0000
#define CRM_RE_TLB_BASE         0x1000
#define CRM_RE_PIXPIPE_BASE     0x2000
#define CRM_RE_MTE_BASE         0x3000
#define CRM_RE_STATUS_BASE      0x4000

/* Interface buffer: RE revision can be read at offset 0x400 */
#define CRM_RE_INTFBUF_ADDR0    0x0400

/* Status register at offset 0x4000 */
#define CRM_RE_STATUS_REG       0x4000
#define CRM_RE_SET_STARTPTR     0x4008

/*
 * Status register bits:
 *   Bit 28: RE idle
 *   Bit 27: Setup engine idle
 *   Bit 26: Pixel pipe idle
 *   Bit 25: MTE idle
 *   Bits [24:18]: Interface buffer level (0 = empty)
 *   Bits [17:12]: Read pointer
 *   Bits [11:6]:  Write pointer
 *   Bits [5:0]:   Start pointer
 */
#define CRMSTAT_RE_IDLE         (1U << 28)
#define CRMSTAT_SETUP_IDLE      (1U << 27)
#define CRMSTAT_PIXPIPE_IDLE    (1U << 26)
#define CRMSTAT_MTE_IDLE        (1U << 25)
#define CRMSTAT_ALL_IDLE        (CRMSTAT_RE_IDLE | CRMSTAT_SETUP_IDLE | \
                                 CRMSTAT_PIXPIPE_IDLE | CRMSTAT_MTE_IDLE)

/* MTE registers (offset from MTE base 0x3000) */
#define CRM_MTE_MODE            0x00
#define CRM_MTE_BYTEMASK        0x08
#define CRM_MTE_STIPPLEMASK     0x10
#define CRM_MTE_FGVALUE         0x18
#define CRM_MTE_SRC0            0x20
#define CRM_MTE_SRC1            0x28
#define CRM_MTE_DST0            0x30
#define CRM_MTE_DST1            0x38
#define CRM_MTE_SRCYSTEP        0x40
#define CRM_MTE_DSTYSTEP        0x48
#define CRM_MTE_NULL            0x70
#define CRM_MTE_FLUSH           0x78

struct SGICRIMEREState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Status register (always reports idle) */
    uint32_t status;
};

#endif /* HW_MISC_SGI_CRIME_RE_H */
