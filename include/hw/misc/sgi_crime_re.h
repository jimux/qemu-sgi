/*
 * SGI CRIME Rendering Engine (RE) emulation
 *
 * The CRIME rendering engine occupies physical 0x15000000-0x15004FFF on the
 * SGI O2 (IP32). It provides the graphics pipeline, framebuffer TLBs,
 * MTE (Memory Transfer Engine), and status registers.
 *
 * This model implements what the PROM textport needs:
 *   - the fb TLB (tile page tables, 16-bit descriptors, 4 per u64 entry)
 *   - the pixel-pipe draw state (DrawMode/LogicOp/ColorMask/Stipple/
 *     Vertex/Primitive) and the go-kick (offset | 0x800 within the
 *     pixpipe page, per crmDefs.h crmSetAndGo)
 *   - the MTE fill engine (mode/fgValue/dst0/dst1/byteMask + kick)
 *   - idle status forever (never busy-blocks)
 *
 * Register regions (offset from 0x15000000):
 *   0x0000-0x0FFF  Interface Buffer (command FIFO)
 *   0x1000-0x17FF  TLB (framebuffer fbA/fbB/fbC, tex, cid, linear)
 *   0x2000-0x21FF  Pixel Pipe (drawing pipeline registers)
 *   0x3000-0x307F  MTE (Memory Transfer Engine)
 *   0x4000-0x400F  Status/Control
 *
 * Reference: IRIX sys/crimereg.h, crimedef.h (prom-building copies are
 * byte-identical layouts), crm_tp.c (the textport's op sequence).
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

/* Region offsets from RE base (0x15000000) — crimedef.h */
#define CRM_RE_INTFBUF_BASE     0x0000
#define CRM_RE_TLB_BASE         0x1000
#define CRM_RE_PIXPIPE_BASE     0x2000
#define CRM_RE_MTE_BASE         0x3000
#define CRM_RE_STATUS_BASE      0x4000

/* Interface buffer: RE revision readable at offset 0x400 (crmGetRERev
 * reads the 32-bit word there and takes bit 0; 0 = rev B) */
#define CRM_RE_INTFBUF_ADDR0    0x0400

/* Status register at offset 0x4000 */
#define CRM_RE_STATUS_REG       0x4000
#define CRM_RE_SET_STARTPTR     0x4008

/*
 * Status register bits (crimereg.h CrmStatusReg):
 *   Bit 28: RE idle, 27: Setup idle, 26: Pixel pipe idle, 25: MTE idle
 *   (mte_copy.c additionally polls bit 25 as CRMSTAT_MTE_FLUSHED)
 */
#define CRMSTAT_RE_IDLE         (1U << 28)
#define CRMSTAT_SETUP_IDLE      (1U << 27)
#define CRMSTAT_PIXPIPE_IDLE    (1U << 26)
#define CRMSTAT_MTE_IDLE        (1U << 25)
#define CRMSTAT_ALL_IDLE        (CRMSTAT_RE_IDLE | CRMSTAT_SETUP_IDLE | \
                                 CRMSTAT_PIXPIPE_IDLE | CRMSTAT_MTE_IDLE)

/* TLB sub-offsets (crimedef.h) */
#define CRM_TLB_FB_A_OFFSET     0x000
#define CRM_TLB_FB_B_OFFSET     0x200
#define CRM_TLB_FB_C_OFFSET     0x400
#define CRM_TLB_TEX_OFFSET      0x600
#define CRM_TLB_CID_OFFSET      0x6e0
#define CRM_TLB_LINEAR_A_OFFSET 0x700
#define CRM_TLB_LINEAR_B_OFFSET 0x780

/* Pixel-pipe register offsets from the pixpipe base (crimedef.h) */
#define CRM_BUF_MODE_SRC_REG        0x000
#define CRM_BUF_MODE_DST_REG        0x008
#define CRM_CLIP_MODE_REG           0x010
#define CRM_DRAW_MODE_REG           0x018
#define CRM_SCRMASK0_REG            0x020
#define CRM_SCISSOR_REG             0x048
#define CRM_WINOFFSET_SRC_REG       0x050
#define CRM_WINOFFSET_DST_REG       0x058
#define CRM_PRIMITIVE_REG           0x060
#define CRM_VERTEX_X_XY0_REG        0x070
#define CRM_VERTEX_X_XY1_REG        0x074
#define CRM_VERTEX_X_XY2_REG        0x078
#define CRM_START_SETUP_REG         0x098
#define CRM_PIXELXFER_SRC_ADDR_REG  0x0a0
#define CRM_PIXELXFER_SRC_XSTEP_REG 0x0a8
#define CRM_PIXELXFER_SRC_YSTEP_REG 0x0ac
#define CRM_PIXELXFER_DST_LINADDR_REG  0x0b0
#define CRM_PIXELXFER_DST_LINSTRIDE_REG 0x0b4
#define CRM_STIPPLE_MODE_REG       0x0c0
#define CRM_STIPPLE_PATT_REG       0x0c4
#define CRM_SHADE_FGCOLOR_REG      0x0d0
#define CRM_SHADE_BGCOLOR_REG      0x0d8
#define CRM_LOGICOP_REG            0x1b0
#define CRM_COLORMASK_REG          0x1b8
#define CRM_PIXPIPE_NULL_REG       0x1f0
#define CRM_PIXPIPE_FLUSH_REG      0x1f8

/* DrawMode bits (crimedef.h) */
#define DM_ENPIXXFER            (1U << 21)
#define DM_ENLINESTIPPLE        (1U << 19)
#define DM_ENLOGICOP            (1U << 9)
#define DM_ENCOLORMASK          (1U << 7)
#define DM_ENCOLORBYTEMASK      ((1U << 3) | (1U << 4) | (1U << 5) | (1U << 6))

/* Primitive opcodes (crimedef.h) */
#define PRIM_OPCODE_SHIFT       24
#define PRIM_OPCODE_MASK        (0xFFu << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_POINT       (0u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_LINE        (1u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_TRI         (2u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_RECT        (3u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_FLUSH       (4u << PRIM_OPCODE_SHIFT)

/* MTE registers (offset from MTE base 0x3000) — crimedef.h */
#define CRM_MTE_MODE_REG        0x00
#define CRM_MTE_BYTEMASK_REG    0x08
#define CRM_MTE_STIPPLEMASK_REG 0x10
#define CRM_MTE_FGVALUE_REG     0x18
#define CRM_MTE_SRC0_REG        0x20
#define CRM_MTE_SRC1_REG        0x28
#define CRM_MTE_DST0_REG        0x30
#define CRM_MTE_DST1_REG        0x38
#define CRM_MTE_SRCYSTEP_REG    0x40
#define CRM_MTE_DSTYSTEP_REG    0x48
#define CRM_MTE_NULL_REG        0x70
#define CRM_MTE_FLUSH_REG       0x78

/* MTE mode bits (crimedef.h) */
#define MTE_CLEAR               (0u << 11)
#define MTE_COPY                (1u << 11)
#define MTE_EN_STIPPLE          (1u << 10)
#define MTE_PIX_DEPTH_SHFT      8
#define SRC_TLB_SHIFT           5
#define DST_TLB_SHIFT           2

/* crmSetAndGo go-bit: OR 0x800 into the pixpipe-page register offset */
#define CRM_GO_OFFSET           0x800

/* Framebuffer TLB: 64 entries x 8 bytes each for fbA/fbB/fbC */
#define CRM_TLB_FB_ENTRIES      64
/* Linear TLB: 16 entries x 8 bytes (2 x 32-bit descriptors each) */
#define CRM_TLB_LINEAR_ENTRIES 16

struct SGICRIMEREState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Status shadow (always reads idle) */
    uint32_t status;

    /* Framebuffer TLBs — each entry is a u64 packing four 16-bit tile
     * descriptors (bit 15 = valid, bits 14:0 = phys>>16) per the PROM's
     * initFramebuffer Tlb.fbA[i].dw writes. */
    uint64_t tlb_fb[3][CRM_TLB_FB_ENTRIES];
    /* Linear TLBs — 2 x 32-bit descriptors per u64, bit 31 = valid,
     * low bits = physical page address (mte_copy.c mte_set_tlb). */
    uint64_t tlb_linear[2][CRM_TLB_LINEAR_ENTRIES];

    /* Pixel-pipe draw state (what the textport programs) */
    uint32_t bufmode_src, bufmode_dst;
    uint32_t drawmode;
    uint32_t primitive;
    uint32_t vertex_xy[3];
    uint32_t stipple_mode, stipple_pattern;
    uint32_t shade_fgcolor, shade_bgcolor;
    uint32_t logicop;
    uint32_t colormask;
    uint32_t pixelxfer_src_addr;
    uint32_t pixelxfer_src_xstep, pixelxfer_src_ystep;

    /* MTE state */
    uint32_t mte_mode;
    uint32_t mte_bytemask;
    uint32_t mte_fgvalue;
    uint64_t mte_dst0, mte_dst1;
};

#endif /* HW_MISC_SGI_CRIME_RE_H */
