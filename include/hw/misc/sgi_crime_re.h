/*
 * SGI CRIME Rendering Engine (RE) emulation
 *
 * The CRIME rendering engine occupies physical 0x15000000-0x15004FFF on the
 * SGI O2 (IP32). It provides the graphics pipeline, framebuffer TLBs,
 * MTE (Memory Transfer Engine), and status registers.
 *
 * M11a extends the M7 textport-level model to what Xsgi/crm.so + libGLcore
 * need (progress_notes/o2_qemu/11-mre-vice-scope.md §4 11a):
 *   - the full pixpipe register file (all CrmDrawReg members stored, so
 *     libGLcore's direct register writes never fault — libGLcore is
 *     UNSTRIPPED with DWARF and its store offsets decode exactly onto the
 *     crimedef.h layout; verified __glCrmFillTriangle/Smooth*Triangle)
 *   - TRI primitives: Vertex.GL 13.6 fixed-point, half-plane edge
 *     functions (spec §7.3.6 "intersection of three half-planes"),
 *     Gouraud shade planes (9.12 fixed point, spec §7.3.7.1)
 *   - Scissor/WinOffset/ClipMode + ScrMask[5] clipping
 *   - full 16 logic-op ROP set + ColorMask + byte masking
 *   - PixelXfer (linear/tiled source with xStep/yStep stepping)
 *   - MTE COPY (tiled<->tiled with srcYStep/dstYStep, linear<->linear
 *     via the linear TLBs — the kernel mte_copy path)
 *   - counted interface buffer + CRMSTAT IB level/ptrs + RE3/RE4
 *     watermark interrupt lines (the folded 11b item)
 *
 * Register regions (offset from 0x15000000):
 *   0x0000-0x0FFF  Interface Buffer (command FIFO)
 *   0x1000-0x17FF  TLB (framebuffer fbA/fbB/fbC, tex, cid, linear)
 *   0x2000-0x21FF  Pixel Pipe (drawing pipeline registers)
 *   0x3000-0x307F  MTE (Memory Transfer Engine)
 *   0x4000-0x400F  Status/Control
 *
 * Reference: IRIX sys/crimereg.h, crimedef.h (prom-building copies are
 * byte-identical layouts), crm_tp.c (the textport's op sequence), CRIME 1.5
 * spec ch. 7, gxemul dev_sgi_re.c (2D op semantics), the unstripped
 * libGLcore.so register-write census.
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

/* Interface buffer layout (spec Table 7-1 / crimereg.h CrmIntfBufReg):
 * 64 x 8-byte data slots @0x0, 64 x 8-byte addr slots @0x200, ctl @0x400.
 * crmGetRERev reads the 32-bit word at 0x400 and takes bit 0; 0 = rev B. */
#define CRM_RE_INTFBUF_DATA     0x0000
#define CRM_RE_INTFBUF_ADDR     0x0200
#define CRM_RE_INTFBUF_CTL      0x0400
#define CRIME_FIFO_DEPTH        64

/* Status register at offset 0x4000 (crimereg.h CrmStatusReg) */
#define CRM_RE_STATUS_REG       0x4000
#define CRM_RE_SET_STARTPTR     0x4008

/*
 * Status register bits (crimereg.h):
 *   28 reIdle, 27 setupIdle, 26 pixPipeIdle, 25 mteIdle
 *   18:12 intfBufLevel(7)/RdPtr(6), 11:6 WrPtr, 5:0 StartPtr
 */
#define CRMSTAT_RE_IDLE         (1U << 28)
#define CRMSTAT_SETUP_IDLE      (1U << 27)
#define CRMSTAT_PIXPIPE_IDLE    (1U << 26)
#define CRMSTAT_MTE_IDLE        (1U << 25)
#define CRMSTAT_ALL_IDLE        (CRMSTAT_RE_IDLE | CRMSTAT_SETUP_IDLE | \
                                 CRMSTAT_PIXPIPE_IDLE | CRMSTAT_MTE_IDLE)
#define CRMSTAT_IB_LEVEL_SHIFT  18
#define CRMSTAT_IB_LEVEL_MASK   0x7f
#define CRMSTAT_IB_RDPTR_SHIFT  12
#define CRMSTAT_IB_RDPTR_MASK   0x3f
#define CRMSTAT_IB_WRPTR_SHIFT  6
#define CRMSTAT_IB_WRPTR_MASK   0x3f
#define CRMSTAT_IB_STPTR_SHIFT  0
#define CRMSTAT_IB_STPTR_MASK   0x3f

/* TLB sub-offsets (crimedef.h) */
#define CRM_TLB_FB_A_OFFSET     0x000
#define CRM_TLB_FB_B_OFFSET     0x200
#define CRM_TLB_FB_C_OFFSET     0x400
#define CRM_TLB_TEX_OFFSET      0x600
#define CRM_TLB_CID_OFFSET      0x6e0
#define CRM_TLB_LINEAR_A_OFFSET 0x700
#define CRM_TLB_LINEAR_B_OFFSET 0x780
#define CRM_TLB_PAGE_SIZE       0x800     /* 0x1000 TLB page, 0x800 used */

/* TLB sizes: fbA/B/C 64 entries, tex 28, cid 4, linear 16 (crimereg.h) */
#define CRM_TLB_FB_ENTRIES      64
#define CRM_TLB_TEX_ENTRIES     28
#define CRM_TLB_CID_ENTRIES     4
#define CRM_TLB_LINEAR_ENTRIES  16

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
#define CRM_VERTEX_GL_X0_REG        0x080
#define CRM_VERTEX_GL_Y0_REG        0x084
#define CRM_VERTEX_GL_X1_REG        0x088
#define CRM_VERTEX_GL_Y1_REG        0x08c
#define CRM_VERTEX_GL_X2_REG        0x090
#define CRM_VERTEX_GL_Y2_REG        0x094
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
#define CRM_SHADE_R0_REG           0x0e0
#define CRM_SHADE_G0_REG           0x0e4
#define CRM_SHADE_B0_REG           0x0e8
#define CRM_SHADE_A0_REG           0x0ec
#define CRM_SHADE_DRDX_REG         0x0f0
#define CRM_SHADE_DGDX_REG         0x0f4
#define CRM_SHADE_DRDY_REG         0x0f8
#define CRM_SHADE_DGDY_REG         0x0fc
#define CRM_SHADE_DBDX_REG         0x100
#define CRM_SHADE_DADX_REG         0x104
#define CRM_SHADE_DBDY_REG         0x108
#define CRM_SHADE_DADY_REG         0x10c

/* Texture registers (crimereg.h CrmTextureReg / spec Table 7-3) */
#define CRM_TEXTURE_MODE_REG        0x110
#define CRM_TEXTURE_FORMAT_REG      0x118
#define CRM_TEXTURE_SQ0_REG         0x120
#define CRM_TEXTURE_TQ0_REG         0x128
#define CRM_TEXTURE_Q0_REG          0x130
#define CRM_TEXTURE_STSHIFT_REG     0x134
#define CRM_TEXTURE_DSQDX_REG       0x138
#define CRM_TEXTURE_DSQDY_REG       0x140
#define CRM_TEXTURE_DTQDX_REG       0x148
#define CRM_TEXTURE_DTQDY_REG       0x150
#define CRM_TEXTURE_DQDX_REG        0x158
#define CRM_TEXTURE_DQDY_REG        0x15c
#define CRM_TEXTURE_BORDER_REG      0x160
#define CRM_TEXTURE_ENV_REG         0x168

#define CRM_FOG_COLOR_REG          0x170
#define CRM_LOGICOP_REG            0x1b0
#define CRM_COLORMASK_REG          0x1b8
/*
 * Depth/stencil (crimedef.h CRM_DEPTH_*; spec Table 7-3 "Depth.func
 * 0x1c0, Depth.Zs 0x1c8 (64), Depth.dzdx 0x1d0 (64), Depth.dzdy 0x1d8
 * (64)").  Depth.mode packs (MSB-first, crimereg.h CrmDepthMode):
 *   bits 27:25 func, bit 24 enTagClear, bits 23:0 clear (24-bit far value).
 */
#define CRM_DEPTH_FUNC_REG          0x1c0
#define CRM_DEPTH_Z0_REG            0x1c8
#define CRM_DEPTH_DZDX_REG          0x1d0
#define CRM_DEPTH_DZDY_REG          0x1d8
#define CRM_PIXPIPE_NULL_REG       0x1f0
#define CRM_PIXPIPE_FLUSH_REG      0x1f8

/* Depth.func field (spec §7.3.7.17 Table 7-25, GL_NEVER..GL_ALWAYS) */
#define CRM_DEPTH_FUNC_SHIFT        25
#define CRM_DEPTH_FUNC_MASK         (7u << CRM_DEPTH_FUNC_SHIFT)
#define CRM_DEPTH_FUNC_NEVER        0
#define CRM_DEPTH_FUNC_LESS         1
#define CRM_DEPTH_FUNC_EQUAL        2
#define CRM_DEPTH_FUNC_LEQUAL       3
#define CRM_DEPTH_FUNC_GREATER      4
#define CRM_DEPTH_FUNC_NOTEQUAL     5
#define CRM_DEPTH_FUNC_GEQUAL       6
#define CRM_DEPTH_FUNC_ALWAYS       7

/* SZ pixel: bits 31:24 stencil, bits 23:0 depth (spec §7.3.4.4 Fig 7-6) */
#define CRM_SZ_DEPTH_MASK           0x00ffffffu
#define CRM_SZ_STENCIL_MASK         0xff000000u

/*
 * The depth/stencil buffer is mapped through framebuffer TLB C.  Live
 * evidence (tmp/o2-qemu/depthfix/evidence/measure.trace): libGLcore's
 * depth-buffer setup writes BufMode.src/dst = 0x10030226 and then loads
 * TLB.fbC (bank 2) with the SZ tiles 0x65..0x7c, and glClear runs the
 * depth clear as MTE CLEAR mode 0x208 (dstBufType=2 = framebuffer TLB C,
 * pixDepth=2/32-bit, fgValue=0x00ffffff).  Spec §7.3.4.2: the framebuffer
 * TLB holds "the base physical memory addresses of the tiles which compose
 * a color buffer and the stencil-depth buffer".
 */
#define CRM_DEPTH_TLB_SEL           2

/* DrawMode bits (crimedef.h) */
#define DM_ENNOCONFLICT       (1U << 23)
#define DM_ENGL               (1U << 22)
#define DM_ENPIXXFER          (1U << 21)
#define DM_ENSCISSORTEST      (1U << 20)
#define DM_ENLINESTIPPLE      (1U << 19)
#define DM_ENPOLYSTIPPLE      (1U << 18)
#define DM_ENOPAQSTIPPLE      (1U << 17)
#define DM_ENSMOOTHSHADE      (1U << 16)
#define DM_ENTEXTURE          (1U << 15)    /* spec Table 7-2 enTexture */
#define DM_ENDEPTHTEST        (1U << 2)     /* spec Table 7-7 bit 2 */
#define DM_ENDEPTHMASK        (1U << 1)     /* spec Table 7-7 bit 1 */
#define DM_ENLOGICOP          (1U << 9)
#define DM_ENDITHER           (1U << 8)
#define DM_ENCOLORMASK        (1U << 7)
#define DM_ENCOLORBYTEMASK    ((1U << 3) | (1U << 4) | (1U << 5) | (1U << 6))

/* ClipMode bits (crimedef.h CM_*) */
#define CM_ENCID              (1U << 11)
#define CM_ENSCRMASK_SHIFT    5

/* Primitive opcodes (crimedef.h) */
#define PRIM_OPCODE_SHIFT       24
#define PRIM_OPCODE_MASK        (0xFFu << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_POINT       (0u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_LINE        (1u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_TRI         (2u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_RECT        (3u << PRIM_OPCODE_SHIFT)
#define PRIM_OPCODE_FLUSH       (4u << PRIM_OPCODE_SHIFT)
#define PRIM_EDGE_MASK          (3u << 16)
#define PRIM_SKIPLAST           (1u << 18)

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
#define MTE_SRCECC              (1u << 1)
#define MTE_DSTECC              (1u << 0)

/* BufMode bits (crimedef.h) */
#define BM_BUF_TYPE_SHIFT       10
#define BM_BUF_TYPE_MASK        (7u << BM_BUF_TYPE_SHIFT)
#define BM_BUF_DEPTH_SHIFT      8
#define BM_BUF_DEPTH_MASK      (3u << BM_BUF_DEPTH_SHIFT)
#define BM_PIX_TYPE_SHIFT       4
#define BM_PIX_TYPE_MASK        (3u << BM_PIX_TYPE_SHIFT)
#define BM_PIX_DEPTH_SHIFT      2
#define BM_PIX_DEPTH_MASK      (3u << BM_PIX_DEPTH_SHIFT)

/* Stipple mode bit fields (crimedef.h) */
#define MAX_INDEX               16
#define STIPPLE_INDEX           24

/* crmSetAndGo go-bit: OR 0x800 into the pixpipe-page register offset
 * (crmDefs.h: +0x200 words = 0x800 bytes; any go write commits the
 * pending primitive) */
#define CRM_GO_OFFSET           0x800

/*
 * Shadow (read-back) files.
 *
 * The RE pixel-pipe, MTE and TLB registers are read/write latches: the
 * IRIX driver's crmSavePP() reads them back to save the switching-out
 * context and crmRestorePP() re-writes them on the way in (its copy set
 * is CRMDRAWREGS+MTE, masked by CRM_CXSW_*).  A read of a register must
 * therefore return the last value written, or the save/restore cannot
 * round-trip and the switching-in context runs with zeroed registers.
 *
 * Rather than reconstructing every read from the named state fields, a
 * flat last-written image of each register page is kept.  Every accepted
 * write lands here (including registers whose drawing behaviour is not
 * modelled yet, which must still latch for a faithful round-trip).
 */
#define CRM_RE_PIXPIPE_SIZE     0x200   /* 0x2000-0x21ff, 128 x 32-bit */
#define CRM_RE_MTE_SIZE         0x80    /* 0x3000-0x307f,  32 x 32-bit */

struct SGICRIMEREState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Status shadow (composed on read) */
    uint32_t status;

    /*
     * Interface buffer (host command ring): 64-deep data/addr RAMs + ctl.
     *
     * Spec §7.3.3.1: "host writes to rendering engine registers are
     * stored in a ring buffer called the host interface buffer ... read
     * out of the interface buffer in FIFO order."  The RE's own register
     * pages ARE the ring's data source: every write to the TLB,
     * pixel-pipe or MTE page posts a {data, CrmIntfBufAddr descriptor}
     * entry at WrPtr.  Our engine retires synchronously, so RdPtr tracks
     * WrPtr (IB level stays 0 and crmWaitReFifo never spins), but the
     * entries survive in the RAM so crmSavePP() can harvest them: it
     * computes pending = WrPtr - StartPtr (mod 64) from CRM_RE_STATUS_REG
     * and copies [StartPtr, WrPtr) out of ib_data/ib_addr.
     */
    uint64_t ib_data[CRIME_FIFO_DEPTH];
    uint64_t ib_addr[CRIME_FIFO_DEPTH];
    uint32_t ib_ctl;
    uint32_t ib_count;              /* entries posted since last drain */
    /*
     * Ring pointers (Status bits: level 24:18, RdPtr 17:12, WrPtr 11:6,
     * StartPtr 5:0).  SetStartPtr (@0x4008) is the host's "consume up to
     * here" command: it moves StartPtr (and, since our ring holds the
     * live entries at the tail, base-relative WrPtr/RdPtr) to the written
     * index, so pending = WrPtr - StartPtr counts the writes since the
     * last SetStartPtr.  crmSavePP() clears it to 0 after each harvest.
     */
    uint32_t ib_startptr;
    uint32_t ib_wrptr;
    uint32_t ib_rdptr;

    /*
     * Framebuffer TLBs — each entry is a u64 packing four 16-bit tile
     * descriptors (bit 15 = valid, bits 14:0 = phys>>16) per the PROM's
     * initFramebuffer Tlb.fbA[i].dw writes. tex[28]/cid[4] same format.
     */
    uint64_t tlb_fb[3][CRM_TLB_FB_ENTRIES];
    uint64_t tlb_tex[CRM_TLB_TEX_ENTRIES];
    uint64_t tlb_cid[CRM_TLB_CID_ENTRIES];
    /* Linear TLBs — 2 x 32-bit descriptors per u64, bit 31 = valid,
     * low bits = physical page address (mte_copy.c mte_set_tlb). */
    uint64_t tlb_linear[2][CRM_TLB_LINEAR_ENTRIES];

    /*
     * Full pixel-pipe draw state (CrmDrawReg members; X path uses
     * Vertex.X/fg/bg, GL path uses Vertex.GL/shade planes).
     */
    uint32_t bufmode_src, bufmode_dst;
    uint32_t clipmode;
    uint32_t drawmode;
    uint32_t scrmask[10];           /* 5 x 64-bit: [2i]=min [2i+1]=max */
    uint32_t scissor_lo, scissor_hi;
    uint32_t winoffset_src, winoffset_dst;
    uint32_t primitive;
    uint32_t vertex_xy[3];
    uint32_t vertex_gl[3][2];       /* [i][0]=x [i][1]=y, 13.6 fixed pt */
    uint32_t startsetup;
    uint32_t pixelxfer_src_addr;
    uint32_t pixelxfer_src_xstep, pixelxfer_src_ystep;
    uint32_t pixelxfer_dst_linaddr, pixelxfer_dst_linstride;
    uint32_t stipple_mode, stipple_pattern;
    uint32_t shade_fgcolor, shade_bgcolor;
    uint32_t shade_plane[12];       /* r0 g0 b0 a0 drdx dgdx drdy dgdy dbdx dadx dbdy dady */

    /*
     * Texture state (CrmTextureReg, spec §7.3.1.13).  SQs/TQs/Qs are the
     * homogeneous-coordinate plane initial values (36.12 / 36.12 / 18.12)
     * and dsq/dtq/dq the per-x and per-y slopes (36.12 / 18.12), exactly
     * matching libGLcore __glCrmFillTriangle stores at 0x2120..0x215c.
     */
    uint32_t tex_mode;
    uint64_t tex_format;
    int64_t tex_sq0, tex_tq0;
    int32_t tex_q0;
    int32_t tex_stshift;
    int64_t tex_dsqdx, tex_dsqdy, tex_dtqdx, tex_dtqdy;
    int32_t tex_dqdx, tex_dqdy;
    uint32_t tex_border, tex_env;

    /*
     * Depth/stencil state (CrmDepthReg, spec §7.3.1.18).  mode is the raw
     * Depth.mode word (func/enTagClear/clear); z0/dzdx/dzdy are the 64-bit
     * 25.12 two's-complement plane parameters, loaded by libGLcore's
     * __glCrmFillTriangle at the floored reference vertex.
     */
    uint32_t depth_mode;
    int64_t depth_z0, depth_dzdx, depth_dzdy;

    uint32_t fog_color, fog_f0, fog_dfdx, fog_dfdy;
    uint32_t logicop;
    uint32_t colormask;

    /* MTE state */
    uint32_t mte_mode;
    uint32_t mte_bytemask;
    uint32_t mte_stipplemask;
    uint32_t mte_fgvalue;
    uint64_t mte_src0, mte_src1;
    uint64_t mte_dst0, mte_dst1;
    uint32_t mte_srcystep, mte_dstystep;

    /*
     * Last-written register images (read-back semantics for
     * crmSavePP()/crmRestorePP() and any other reader).  Indexed in
     * 32-bit words from each page base; go-offset aliases are masked
     * off before indexing.
     */
    uint32_t pp_shadow[CRM_RE_PIXPIPE_SIZE / 4];
    uint32_t mte_shadow[CRM_RE_MTE_SIZE / 4];
};

#endif /* HW_MISC_SGI_CRIME_RE_H */
