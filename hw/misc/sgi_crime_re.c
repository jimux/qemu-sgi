/*
 * SGI CRIME Rendering Engine (RE) emulation
 *
 * M11a: extended from the M7 textport-level model to X-desktop level
 * (Xsgi + dyDDX/crm.so 2D + libGLcore.so GL raster). See the header
 * for the register map; implementation notes with sources inline:
 *
 *  - RECT/LINE X-mode semantics preserve the M7 gxemul 0x7ff-masked
 *    modular walks (dev_sgi_re.c draw_primitive) — the installed
 *    kernel's clear-screen RECT with LR_TB edge bits depends on them.
 *  - TRI is new: half-plane edge functions per CRIME 1.5 spec §7.3.6
 *    ("(x,y) is exactly inside the polygon iff Ei(x,y) >= 0"), over
 *    Vertex.GL 13.6 fixed-point coordinates, with Gouraud shade
 *    planes iterated in 9.12 fixed point (§7.3.7.1). libGLcore writes
 *    Vertex.GL as (x<<32)|y 64-bit pairs masked & ~0x3f (verified in
 *    __glCrmFillTriangle/__glCrmSmoothRGBTriangle disassembly: the
 *    kick is a write to StartSetup|0x800 = 0x2898, i.e. any pixpipe
 *    go-write commits the primitive).
 *  - MTE COPY: tiled<->tiled with srcYStep/dstYStep (kernel
 *    mte_copy.c uses src/dst YStep as bytes-per-scanline for linear
 *    copies through the linear TLBs; X crm.so uses them as tile-list
 *    strides for scrolled window copies).
 *  - The interface buffer is counted but always drained (synchronous
 *    execution): level reads 0, RE3/RE4 watermark lines never fire
 *    spuriously (11b folded per scoping §4).
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/sgi_crime_re.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

/* CRM_LOGICOP_* (crimedef.h) */
#define CRM_LOGICOP_ZERO            0
#define CRM_LOGICOP_AND             1
#define CRM_LOGICOP_AND_REVERSE     2
#define CRM_LOGICOP_COPY            3
#define CRM_LOGICOP_AND_INVERTED    4
#define CRM_LOGICOP_NOOP            5
#define CRM_LOGICOP_XOR             6
#define CRM_LOGICOP_OR              7
#define CRM_LOGICOP_NOR             8
#define CRM_LOGICOP_EQUIV           9
#define CRM_LOGICOP_INVERT          10
#define CRM_LOGICOP_OR_REVERSE      11
#define CRM_LOGICOP_COPY_INVERTED   12
#define CRM_LOGICOP_OR_INVERTED     13
#define CRM_LOGICOP_NAND            14
#define CRM_LOGICOP_SET             15

/* ------------------------------------------------------------------ */
/* TLB translation helpers                                              */
/* ------------------------------------------------------------------ */

/*
 * Decode one fb/tex/cid TLB bank entry: bank 0/1/2 = fbA/fbB/fbC,
 * 3 = tex, 6 = cid. Returns the 16-bit tile descriptor for tile_nr.
 */
static uint16_t sgi_crime_re_tile_desc(SGICRIMEREState *s, int tlb_sel,
                                       int tile_nr)
{
    const uint64_t *bank;
    int entries;

    switch (tlb_sel) {
    case 0: case 1: case 2:
        bank = s->tlb_fb[tlb_sel];
        entries = CRM_TLB_FB_ENTRIES * 4;
        break;
    case 3:
        bank = s->tlb_tex;
        entries = CRM_TLB_TEX_ENTRIES * 4;
        break;
    case 6:
        bank = s->tlb_cid;
        entries = CRM_TLB_CID_ENTRIES * 4;
        break;
    default:
        return 0;
    }
    if (tile_nr < 0 || tile_nr >= entries) {
        return 0;
    }
    int entry = tile_nr >> 2;
    int sub = tile_nr & 3;
    return (uint16_t)((bank[entry] >> (48 - 16 * sub)) & 0xffff);
}

static bool sgi_crime_re_tiled_addr(SGICRIMEREState *s, int tlb_sel,
                                    int x, int y, int bpp, hwaddr *phys)
{
    if (x < 0 || y < 0 || x >= 2048 || y >= 2048) {
        return false;
    }
    int tile_w = 512 / bpp;             /* pixels across one tile */
    int tile_nr = (y >> 7) * 16 + x / tile_w;
    uint16_t desc = sgi_crime_re_tile_desc(s, tlb_sel, tile_nr);
    if (!(desc & 0x8000)) {
        return false;                  /* tile not valid/mapped */
    }
    int yin = y & 127;
    int xin = (x % tile_w) * bpp;
    *phys = ((hwaddr)(desc & 0x7fff) << 16) + 512 * yin + xin;
    return true;
}

static bool sgi_crime_re_linear_addr(SGICRIMEREState *s, int lin,
                                     uint32_t byte_off, hwaddr *phys)
{
    int page = byte_off >> 12;
    int entry = page >> 1;
    int sub = page & 1;
    if (entry >= CRM_TLB_LINEAR_ENTRIES) {
        return false;
    }
    uint32_t d = sub ? (uint32_t)s->tlb_linear[lin][entry]
                     : (uint32_t)(s->tlb_linear[lin][entry] >> 32);
    if (!(d & 0x80000000u)) {
        return false;
    }
    *phys = ((hwaddr)(d & 0x7fffffffu) << 12) + (byte_off & 0xfff);
    return true;
}

/*
 * Translate a virtual source/dest coordinate through a BufMode.
 * For tiled banks (bufType 0-3, 6): (x, y) pixel coords.
 * For linear banks (4/5): x is the linear byte offset, y ignored.
 * Returns false if the tile/page is not mapped.
 */
static bool sgi_crime_re_fb_addr(SGICRIMEREState *s, uint32_t bufmode,
                                 int x, int y, hwaddr *phys)
{
    uint32_t tlb_sel = (bufmode >> BM_BUF_TYPE_SHIFT) & 7;
    int depth_code = (bufmode >> BM_BUF_DEPTH_SHIFT) & 3;
    int bpp = 1 << depth_code;         /* 1, 2, or 4 bytes */

    if (tlb_sel <= 3 || tlb_sel == 6) {
        return sgi_crime_re_tiled_addr(s, tlb_sel, x, y, bpp, phys);
    }
    if (tlb_sel == 4 || tlb_sel == 5) {
        if (x < 0) {
            return false;
        }
        return sgi_crime_re_linear_addr(s, tlb_sel - 4, (uint32_t)x, phys);
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Pixel get/put with ROP + masks                                       */
/* ------------------------------------------------------------------ */

/* Read one destination pixel (raw, format-dependent packed value). */
static uint32_t sgi_crime_re_get_pixel(SGICRIMEREState *s, uint32_t bufmode,
                                       int x, int y)
{
    hwaddr phys;
    uint8_t b[4];
    int bpp = 1 << ((bufmode >> BM_BUF_DEPTH_SHIFT) & 3);

    if (!sgi_crime_re_fb_addr(s, bufmode, x, y, &phys)) {
        return 0;
    }
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, bpp, false);

    uint32_t pix_type = (bufmode >> BM_PIX_TYPE_SHIFT) & 3;
    switch (pix_type) {
    case 0:                             /* color index */
        return b[0];
    case 1:                             /* RGB (alpha reads 0) */
        return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
               ((uint32_t)b[2] << 8);
    default:                            /* RGBA / ABGR pack same bytes */
        return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
               ((uint32_t)b[2] << 8) | b[3];
    }
}

/*
 * Apply the 16 logic ops (crimedef.h CRM_LOGICOP_*; gxemul implements
 * COPY/XOR/COPY_INVERTED only, the spec table + X's ROP needs are the
 * full set). src = incoming fragment, dst = old pixel.
 */
static uint32_t sgi_crime_re_apply_rop(uint32_t rop, uint32_t src,
                                       uint32_t dst)
{
    switch (rop & 0xf) {
    case CRM_LOGICOP_ZERO:          return 0;
    case CRM_LOGICOP_AND:           return src & dst;
    case CRM_LOGICOP_AND_REVERSE:   return src & ~dst;
    case CRM_LOGICOP_COPY:          return src;
    case CRM_LOGICOP_AND_INVERTED:  return ~src & dst;
    case CRM_LOGICOP_NOOP:          return dst;
    case CRM_LOGICOP_XOR:           return src ^ dst;
    case CRM_LOGICOP_OR:            return src | dst;
    case CRM_LOGICOP_NOR:           return ~(src | dst);
    case CRM_LOGICOP_EQUIV:         return ~(src ^ dst);
    case CRM_LOGICOP_INVERT:        return ~dst;
    case CRM_LOGICOP_OR_REVERSE:    return src | ~dst;
    case CRM_LOGICOP_COPY_INVERTED: return ~src;
    case CRM_LOGICOP_OR_INVERTED:   return ~src | dst;
    case CRM_LOGICOP_NAND:           return ~(src & dst);
    default: /* CRM_LOGICOP_SET */  return 0xffffffffu;
    }
}

/*
 * Write one pixel through the dst BufMode with ROP/masking applied.
 * Color packing per gxemul getputpixel semantics (CI8/RGB/RGBA).
 */
static void sgi_crime_re_put_pixel(SGICRIMEREState *s, uint32_t bufmode,
                                   int x, int y, uint32_t color)
{
    hwaddr phys;
    uint8_t b[4];
    int bpp;

    if (!sgi_crime_re_fb_addr(s, bufmode, x, y, &phys)) {
        return;
    }

    uint32_t pix_type = (bufmode >> BM_PIX_TYPE_SHIFT) & 3;
    switch (pix_type) {
    case 0:                             /* color index */
        bpp = 1;
        b[0] = color & 0xff;
        break;
    case 1:                             /* RGB */
        bpp = 4;
        b[0] = color >> 24; b[1] = color >> 16; b[2] = color >> 8; b[3] = 0;
        break;
    case 2:                             /* RGBA */
    default:
        bpp = 4;
        b[0] = color >> 24; b[1] = color >> 16; b[2] = color >> 8; b[3] = color;
        break;
    }

    /*
     * ROP + plane mask + byte mask (X's GX* raster ops arrive here;
     * DrawMode.enLogicOp selects). Read-modify-write only when a
     * feature actually needs the old value.
     */
    uint32_t dm = s->drawmode;
    if ((dm & DM_ENLOGICOP) &&
        (s->logicop & 0xf) != CRM_LOGICOP_COPY) {
        uint32_t old = sgi_crime_re_get_pixel(s, bufmode, x, y);
        color = sgi_crime_re_apply_rop(s->logicop, color, old);
        if (pix_type == 0) {
            b[0] = color & 0xff;
        } else {
            b[0] = color >> 24; b[1] = color >> 16;
            b[2] = color >> 8;  b[3] = color;
        }
    }
    if (dm & DM_ENCOLORMASK) {
        /* ColorMask: bit per color component (RGBA planes) — apply to
         * the 4-byte forms by byte-lane; CI8 uses bit 0 of byte 0. */
        uint32_t m = s->colormask;
        uint8_t ob[4];
        address_space_rw(&address_space_memory, phys,
                         MEMTXATTRS_UNSPECIFIED, ob, bpp, false);
        if (bpp == 1) {
            b[0] = (b[0] & m) | (ob[0] & ~m);
        } else {
            if (!(m & 0xff000000u)) { b[0] = ob[0]; }
            if (!(m & 0x00ff0000u)) { b[1] = ob[1]; }
            if (!(m & 0x0000ff00u)) { b[2] = ob[2]; }
            if (!(m & 0x000000ffu)) { b[3] = ob[3]; }
        }
    }
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, bpp, true);
}

/* ------------------------------------------------------------------ */
/* Clip / scissor test                                                  */
/* ------------------------------------------------------------------ */

/*
 * Scissor (window-relative rect; Scissor reg is 64-bit:
 * hi = min(x,y), lo = max(x,y) per spec Table 7-10) + WinOffset.dst
 * translation + screen masks. X's crm.so uses ClipMode enScrMask for
 * window clipping; we implement scissor + the 5 screen masks.
 */
static bool sgi_crime_re_clip_pass(SGICRIMEREState *s, int wx, int wy,
                                   int *fx, int *fy)
{
    uint32_t dm = s->drawmode;

    /* window -> framebuffer translation (WinOffset.dst) */
    int x = wx + (int16_t)(s->winoffset_dst >> 16);
    int y = wy + (int16_t)(s->winoffset_dst & 0xffff);

    if (dm & DM_ENSCISSORTEST) {
        uint32_t lo = s->scissor_lo, hi = s->scissor_hi;
        int minx = (hi >> 16) & 0xffff, miny = hi & 0xffff;
        int maxx = (lo >> 16) & 0xffff, maxy = lo & 0xffff;
        if (wx < minx || wx >= maxx || wy < miny || wy >= maxy) {
            return false;
        }
    }

    /* screen masks: framebuffer-relative rects (ScrMask[i] hi=min lo=max) */
    uint32_t en = (s->clipmode >> CM_ENSCRMASK_SHIFT) & 0x1f;
    for (int i = 0; i < 5; i++) {
        if (en & (1u << i)) {
            uint32_t lo = s->scrmask[2 * i + 1], hi = s->scrmask[2 * i];
            int minx = (hi >> 16) & 0xffff, miny = hi & 0xffff;
            int maxx = (lo >> 16) & 0xffff, maxy = lo & 0xffff;
            bool inside = x >= minx && x < maxx && y >= miny && y < maxy;
            /* scrMaskMode bit i: 1 = pass inside, 0 = pass outside */
            if (inside != ((s->clipmode >> i) & 1)) {
                return false;
            }
        }
    }

    *fx = x;
    *fy = y;
    return true;
}

/* ------------------------------------------------------------------ */
/* Stipple pattern stepping                                             */
/* ------------------------------------------------------------------ */

typedef struct CrimStipple {
    uint32_t pattern;
    int mid_bits;
} CrimStipple;

/* gxemul's strip-left/right construction (crm_tp.c semantics) */
static void sgi_crime_re_stipple_init(SGICRIMEREState *s, CrimStipple *st)
{
    uint32_t pattern = s->stipple_pattern;
    int strip_right = 31 - ((s->stipple_mode >> MAX_INDEX) & 31);
    int strip_left = (s->stipple_mode >> STIPPLE_INDEX) & 31;
    pattern >>= strip_right;
    pattern <<= strip_right;
    pattern <<= strip_left;
    st->pattern = pattern;
    st->mid_bits = 32 - strip_left - strip_right;
}

static inline void crim_stipple_step(CrimStipple *st)
{
    st->pattern = (st->pattern << 1) | (st->pattern >> (st->mid_bits - 1));
}

/* ------------------------------------------------------------------ */
/* Pixel-pipe primitive execution (kicked by the go-bit)               */
/* ------------------------------------------------------------------ */

/* 9.12 fixed-point color stepper, per spec §7.3.7.1: truncate fraction,
 * clamp 9-bit to 8-bit ([8:7]='10' over, '11' under). */
static inline uint8_t crim_shade_clamp(int32_t v9_12)
{
    int32_t t = v9_12 >> 12;           /* truncate fraction (9-bit) */
    if (t < 0) {
        return 0;
    }
    if (t > 255) {
        return 255;
    }
    return (uint8_t)t;
}

/* Emit one fragment: shade -> clip -> stipple -> ROP/mask write. */
static void sgi_crime_re_emit(SGICRIMEREState *s, int wx, int wy,
                              uint32_t color, CrimStipple *st,
                              bool line_stipple)
{
    int x, y;
    if (!sgi_crime_re_clip_pass(s, wx, wy, &x, &y)) {
        return;
    }
    bool draw = true;
    if (line_stipple) {
        if (s->drawmode & DM_ENOPAQSTIPPLE) {
            /* opaque: fg where bit set, else bg */
            color = (st->pattern & 0x80000000u) ? s->shade_fgcolor
                                                : s->shade_bgcolor;
        } else {
            draw = (st->pattern & 0x80000000u) != 0;
        }
    }
    if (draw) {
        sgi_crime_re_put_pixel(s, s->bufmode_dst, x, y, color);
    }
    crim_stipple_step(st);
}

/*
 * TRI: intersection of three half-planes (spec §7.3.6). Edge functions
 * Ei(x,y) = Ai*x + Bi*y + Ci with Ai = dy, Bi = -dx (§7.3.5.2 EQ 2-4);
 * a point is inside iff Ei >= 0 for all i, with edges directed
 * clockwise around the interior (libGLcore orders vertices accordingly
 * and encodes the traversal direction in Primitive.edgeType).
 *
 * Vertices arrive as 13.6 fixed point; we evaluate edge functions at
 * pixel centers in 13.6 space (x*64+32) with 64-bit intermediates.
 */
static void sgi_crime_re_draw_tri(SGICRIMEREState *s)
{
    int64_t vx[3], vy[3];
    for (int i = 0; i < 3; i++) {
        vx[i] = (int64_t)(int32_t)s->vertex_gl[i][0] << 6;  /* to 19.6 */
        vy[i] = (int64_t)(int32_t)s->vertex_gl[i][1] << 6;
    }

    /* edge i = (v[i] -> v[i+1]); A=dy, B=-dx, C computed so E(v[i])=0 */
    int64_t A[3], B[3], C[3];
    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        int64_t dx = vx[j] - vx[i];
        int64_t dy = vy[j] - vy[i];
        A[i] = dy;
        B[i] = -dx;
        C[i] = -(A[i] * vx[i] + B[i] * vy[i]);
    }

    /*
     * Winding: for CCW vertex order the interior is the *negative*
     * half-plane of each edge; flip all signs if the signed area says
     * so (the spec assumes CW; libGLcore's GL coordinate flip can
     * deliver either).
     */
    int64_t area2 = (vx[1] - vx[0]) * (vy[2] - vy[0])
                  - (vx[2] - vx[0]) * (vy[1] - vy[0]);
    if (area2 < 0) {
        for (int i = 0; i < 3; i++) {
            A[i] = -A[i]; B[i] = -B[i]; C[i] = -C[i];
        }
    }

    /* bounding box in pixel space (13.6 -> integer pixels) */
    int64_t minxf = MIN(vx[0], MIN(vx[1], vx[2]));
    int64_t maxxf = MAX(vx[0], MAX(vx[1], vx[2]));
    int64_t minyf = MIN(vy[0], MIN(vy[1], vy[2]));
    int64_t maxyf = MAX(vy[0], MAX(vy[1], vy[2]));
    int minx = (int)(minxf >> 6), maxx = (int)(maxxf >> 6);
    int miny = (int)(minyf >> 6), maxy = (int)(maxyf >> 6);
    maxx++; maxy++;                   /* exclusive bounds */

    const int BOUND = 4096;           /* hard safety bound (2k screen) */
    if (minx < -BOUND) { minx = -BOUND; }
    if (miny < -BOUND) { miny = -BOUND; }
    if (maxx > BOUND) { maxx = BOUND; }
    if (maxy > BOUND) { maxy = BOUND; }

    bool smooth = (s->drawmode & DM_ENSMOOTHSHADE) != 0;
    CrimStipple st;
    sgi_crime_re_stipple_init(s, &st);
    bool poly_stipple = (s->drawmode & DM_ENPOLYSTIPPLE) != 0;

    for (int py = miny; py < maxy; py++) {
        /* edge functions at the row start (pixel centers, 19.6 space) */
        int64_t ex[3];
        for (int i = 0; i < 3; i++) {
            ex[i] = A[i] * ((int64_t)minx * 64 + 32)
                  + B[i] * ((int64_t)py * 64 + 32) + C[i];
        }
        for (int px = minx; px < maxx; px++) {
            bool inside = ex[0] >= 0 && ex[1] >= 0 && ex[2] >= 0;
            if (inside) {
                uint32_t color = s->shade_fgcolor;
                if (smooth) {
                    /*
                     * Gouraud: shade planes are 9.12 fixed point with
                     * origin at the buffer origin (the host computes
                     * z0 + A*(Xs-x0) + B*(Ys-y0) already; the register
                     * values ARE absolute: r0 etc. are "initial value"
                     * at the stepper start). We evaluate the plane at
                     * this pixel directly.
                     */
                    int32_t r = (int32_t)s->shade_plane[0]
                              + (int32_t)s->shade_plane[4] * px
                              + (int32_t)s->shade_plane[6] * py;
                    int32_t g = (int32_t)s->shade_plane[1]
                              + (int32_t)s->shade_plane[5] * px
                              + (int32_t)s->shade_plane[7] * py;
                    int32_t b = (int32_t)s->shade_plane[2]
                              + (int32_t)s->shade_plane[8] * px
                              + (int32_t)s->shade_plane[10] * py;
                    int32_t a = (int32_t)s->shade_plane[3]
                              + (int32_t)s->shade_plane[9] * px
                              + (int32_t)s->shade_plane[11] * py;
                    color = ((uint32_t)crim_shade_clamp(r) << 24)
                          | ((uint32_t)crim_shade_clamp(g) << 16)
                          | ((uint32_t)crim_shade_clamp(b) << 8)
                          | (uint32_t)crim_shade_clamp(a);
                }
                sgi_crime_re_emit(s, px, py, color, &st, poly_stipple);
            }
            ex[0] += A[0]; ex[1] += A[1]; ex[2] += A[2];
        }
    }
}

/*
 * PixelXfer: transfer pixels from src (tiled or linear per
 * BufMode.src, stepped by xStep per pixel / yStep per line) into the
 * destination rect walk (RECT geometry). gxemul's XFER applies inside
 * its RECT loop; X's image upload (crmDrawImage) programs the linear
 * source + steps. Destination goes through BufMode.dst.
 */
static uint32_t sgi_crime_re_xfer_fetch(SGICRIMEREState *s,
                                        int64_t src_off, bool src_linear)
{
    hwaddr phys;
    uint8_t b[4];
    int bpp = 1 << ((s->bufmode_src >> BM_BUF_DEPTH_SHIFT) & 3);

    if (src_linear) {
        if (!sgi_crime_re_linear_addr(s,
                (s->bufmode_src >> BM_BUF_TYPE_SHIFT) & 1,
                (uint32_t)src_off, &phys)) {
            return 0;
        }
    } else {
        int x = (src_off >> 16) & 0xfff;
        int y = src_off & 0xfff;
        if (!sgi_crime_re_tiled_addr(s,
                (s->bufmode_src >> BM_BUF_TYPE_SHIFT) & 7,
                x, y, bpp, &phys)) {
            return 0;
        }
    }
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, bpp, false);
    uint32_t pix_type = (s->bufmode_src >> BM_PIX_TYPE_SHIFT) & 3;
    if (pix_type == 0) {
        return b[0];
    }
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16)
         | ((uint32_t)b[2] << 8) | b[3];
}

static void sgi_crime_re_draw(SGICRIMEREState *s)
{
    uint32_t op = s->primitive;
    uint32_t dm = s->drawmode;
    uint32_t x1 = (s->vertex_xy[0] >> 16) & 0xffff;
    uint32_t y1 = s->vertex_xy[0] & 0xffff;
    uint32_t x2 = (s->vertex_xy[1] >> 16) & 0xffff;
    uint32_t y2 = s->vertex_xy[1] & 0xffff;
    int16_t sx1 = (int16_t)x1, sy1 = (int16_t)y1;
    int16_t sx2 = (int16_t)x2, sy2 = (int16_t)y2;

    trace_sgi_crime_re_draw(op, dm, s->bufmode_dst, sx1, sy1, sx2, sy2,
                            s->shade_fgcolor, s->stipple_pattern);

    /* direction bits: edgeType@16, 2 bits (RL=bit16, TB=bit17) */
    int dx = (op & 0x10000) ? -1 : 1;
    int dy = (op & 0x20000) ? 1 : -1;

    CrimStipple st;
    sgi_crime_re_stipple_init(s, &st);
    bool line_stip = (dm & DM_ENLINESTIPPLE) != 0;
    bool xfer = (dm & DM_ENPIXXFER) != 0;
    bool src_linear = ((s->bufmode_src >> BM_BUF_TYPE_SHIFT) & 7) > 3;
    int32_t xstep = (int32_t)s->pixelxfer_src_xstep;
    int64_t src_off = s->pixelxfer_src_addr;

    switch (op & PRIM_OPCODE_MASK) {
    case PRIM_OPCODE_RECT: {
        /*
         * gxemul semantics (dev_sgi_re.c DE_PRIM_RECTANGLE): coords are
         * masked to the 2048-pixel space and the walk WRAPS modulo
         * 2048 — required by the installed kernel's clear-screen RECT
         * (1280,0)-(0,1024) with LR_TB edge bits (see M7 note). The
         * modular walk terminates by construction; the hard caps are
         * belt-and-braces.
         */
        uint32_t endx = (x2 + (uint32_t)dx) & 0x7ff;
        uint32_t endy = (y2 + (uint32_t)dy) & 0x7ff;
        uint32_t startx = x1 & 0x7ff;
        uint32_t starty = y1 & 0x7ff;
        const uint32_t MOD = 0x800;            /* 2048-pixel space */
        const int MAX_ITER = 2 * MOD;          /* hard cap per axis */

        int ity = 0;
        for (uint32_t yy = starty; yy != endy && ity < MAX_ITER;
             yy = (yy + dy) & (MOD - 1), ity++) {
            int64_t row_src = src_off;
            int itx = 0;
            for (uint32_t xx = startx; xx != endx && itx < MAX_ITER;
                 xx = (xx + dx) & (MOD - 1), itx++) {
                uint32_t color = s->shade_fgcolor;
                if (xfer) {
                    color = sgi_crime_re_xfer_fetch(s, row_src, src_linear);
                }
                int fx, fy;
                if (sgi_crime_re_clip_pass(s, (int)xx, (int)yy, &fx, &fy)) {
                    bool draw = true;
                    if (line_stip) {
                        if (dm & DM_ENOPAQSTIPPLE) {
                            color = (st.pattern & 0x80000000u)
                                  ? s->shade_fgcolor : s->shade_bgcolor;
                        } else {
                            draw = (st.pattern & 0x80000000u) != 0;
                        }
                    }
                    if (draw) {
                        sgi_crime_re_put_pixel(s, s->bufmode_dst,
                                               fx, fy, color);
                    }
                }
                crim_stipple_step(&st);
                row_src += xstep;
            }
        }
        break;
    }

    case PRIM_OPCODE_LINE: {
        /*
         * gxemul semantics (DE_PRIM_LINE): vertices masked to the
         * 2048-pixel space, linelen = max(|dx|,|dy|) min 1, interpolate
         * along the span; PRIM_SKIPLAST drops the endpoint pixel.
         */
        uint32_t mx1 = x1 & 0x7ff, my1 = y1 & 0x7ff;
        uint32_t mx2 = x2 & 0x7ff, my2 = y2 & 0x7ff;
        int lx = abs((int)mx2 - (int)mx1);
        int ly = abs((int)my2 - (int)my1);
        int len = lx > ly ? lx : ly;
        if (len == 0) {
            len = 1;
        }
        int last = (op & PRIM_SKIPLAST) ? len : len + 1;
        if (last > 2 * 0x800) {
            last = 2 * 0x800;
        }

        for (int i = 0; i < last; i++) {
            int x = (int)mx1 + (int)(((int64_t)((int)mx2 - (int)mx1) * i) / len);
            int y = (int)my1
                  + (int)(((int64_t)((int)my2 - (int)my1) * i) / len);
            uint32_t color = s->shade_fgcolor;
            bool draw = true;
            if (line_stip) {
                if (dm & DM_ENOPAQSTIPPLE) {
                    color = (st.pattern & 0x80000000u)
                          ? s->shade_fgcolor : s->shade_bgcolor;
                } else {
                    draw = (st.pattern & 0x80000000u) != 0;
                }
            }
            if (draw) {
                int fx, fy;
                if (sgi_crime_re_clip_pass(s, x, y, &fx, &fy)) {
                    sgi_crime_re_put_pixel(s, s->bufmode_dst, fx, fy, color);
                }
            }
            crim_stipple_step(&st);
        }
        break;
    }

    case PRIM_OPCODE_TRI:
        sgi_crime_re_draw_tri(s);
        break;

    case PRIM_OPCODE_POINT: {
        int fx, fy;
        if (sgi_crime_re_clip_pass(s, (int16_t)x1, (int16_t)y1, &fx, &fy)) {
            sgi_crime_re_put_pixel(s, s->bufmode_dst, fx, fy,
                                  s->shade_fgcolor);
        }
        break;
    }

    default:
        /* FLUSH and others: nothing to execute */
        break;
    }
}

/* ------------------------------------------------------------------ */
/* MTE execution (kicked by the go-bit on an MTE register)              */
/* ------------------------------------------------------------------ */

/*
 * Linear-window transfer (CLEAR or COPY): src0/src1 and dst0/dst1 are
 * linear byte addresses mapped through the linear TLBs; y-steps are
 * the byte stride per 4096-byte "line" chunk (kernel mte_copy.c walks
 * in <=128KB chunks with mte_set_tlb programming both linear TLBs).
 * CLEAR replicates fgValue bytes; COPY moves via a bounce buffer.
 */
static void sgi_crime_re_mte_linear(SGICRIMEREState *s, bool is_copy)
{
    int src_tlb = (s->mte_mode >> SRC_TLB_SHIFT) & 7;
    int dst_tlb = (s->mte_mode >> DST_TLB_SHIFT) & 7;
    int slin = src_tlb - 4, dlin = dst_tlb - 4;
    uint64_t src = s->mte_src0;
    uint64_t dst = s->mte_dst0;
    uint64_t nbytes = (s->mte_dst1 >= s->mte_dst0)
                    ? s->mte_dst1 - s->mte_dst0 + 1 : 0;
    const uint64_t MAX_XFER = 128 * 1024 * 1024; /* sanity cap */
    uint32_t fg = s->mte_fgvalue;
    uint8_t fill[4] = { fg, fg >> 8, fg >> 16, fg >> 24 };

    if (nbytes == 0 || nbytes > MAX_XFER) {
        return;
    }
    if (!is_copy) {
        /* zero/fill through the dst linear TLB (mte_zero) */
        int lin = dlin;
        if (lin < 0 || lin > 1) {
            return;
        }
        uint8_t buf[4096];
        memset(buf, fill[0], sizeof(buf));
        for (uint64_t a = dst; nbytes; ) {
            hwaddr phys;
            if (!sgi_crime_re_linear_addr(s, lin, (uint32_t)a, &phys)) {
                /* skip to the next page on an unmapped entry */
                uint64_t next = (a | 0xfffULL) + 1;
                uint64_t skip = next - a;
                if (skip > nbytes) { skip = nbytes; }
                a += skip; nbytes -= skip;
                continue;
            }
            uint64_t n = 0x1000 - (a & 0xfff);
            if (n > nbytes) { n = nbytes; }
            address_space_rw(&address_space_memory, phys,
                             MEMTXATTRS_UNSPECIFIED, buf, n, true);
            a += n; nbytes -= n;
        }
        return;
    }

    /* COPY linear -> linear through both TLBs (mte_copy) */
    if (slin < 0 || slin > 1 || dlin < 0 || dlin > 1) {
        return;
    }
    uint8_t buf[4096];
    while (nbytes) {
        hwaddr sphys, dphys;
        if (!sgi_crime_re_linear_addr(s, slin, (uint32_t)src, &sphys)) {
            uint64_t next = (src | 0xfffULL) + 1;
            uint64_t skip = next - src;
            if (skip > nbytes) { skip = nbytes; }
            src += skip; dst += skip; nbytes -= skip;
            continue;
        }
        if (!sgi_crime_re_linear_addr(s, dlin, (uint32_t)dst, &dphys)) {
            uint64_t next = (dst | 0xfffULL) + 1;
            uint64_t skip = next - dst;
            if (skip > nbytes) { skip = nbytes; }
            src += skip; dst += skip; nbytes -= skip;
            continue;
        }
        uint64_t n = 0x1000 - (src & 0xfff);
        uint64_t dn = 0x1000 - (dst & 0xfff);
        if (dn < n) { n = dn; }
        if (n > sizeof(buf)) { n = sizeof(buf); }
        if (n > nbytes) { n = nbytes; }
        address_space_rw(&address_space_memory, sphys,
                         MEMTXATTRS_UNSPECIFIED, buf, n, false);
        address_space_rw(&address_space_memory, dphys,
                         MEMTXATTRS_UNSPECIFIED, buf, n, true);
        src += n; dst += n; nbytes -= n;
    }
}

/*
 * Tiled MTE (CLEAR fill or COPY blit). dst0/dst1 hold (x,y) vertex
 * pairs in *byte* x coordinates (gxemul divides by depth/8; NetBSD
 * crmfb_fill_rect / the M7 PROM fill path). srcYStep/dstYStep stride
 * the src/dst tile lists per row for COPY.
 */
static void sgi_crime_re_mte_run(SGICRIMEREState *s)
{
    uint32_t mode = s->mte_mode;
    bool is_copy = (mode & (1u << 11)) != 0;
    int src_tlb = (mode >> SRC_TLB_SHIFT) & 7;
    int dst_tlb = (mode >> DST_TLB_SHIFT) & 7;
    uint32_t fg = s->mte_fgvalue;

    trace_sgi_crime_re_mte(mode, s->mte_dst0, s->mte_dst1, fg);

    /* linear destination (or linear<->linear copy) */
    if ((dst_tlb == 4 || dst_tlb == 5) && !is_copy) {
        sgi_crime_re_mte_linear(s, false);
        return;
    }
    if (is_copy && (dst_tlb == 4 || dst_tlb == 5)) {
        sgi_crime_re_mte_linear(s, true);
        return;
    }

    if (dst_tlb > 6) {
        return;                 /* reserved */
    }

    int x1 = (s->mte_dst0 >> 16) & 0xfff;
    int y1 = s->mte_dst0 & 0xfff;
    int x2 = (s->mte_dst1 >> 16) & 0xfff;
    int y2 = (s->mte_dst1 & 0xfff);
    int depth_code = (mode >> MTE_PIX_DEPTH_SHFT) & 3;
    int bpp = 1 << depth_code;
    x1 /= bpp; x2 /= bpp;
    uint32_t bufmode = (dst_tlb << BM_BUF_TYPE_SHIFT)
                     | (depth_code << BM_BUF_DEPTH_SHIFT)
                     | (bpp == 1 ? 0 : 1 << BM_PIX_TYPE_SHIFT);
    int dx = x1 > x2 ? -1 : 1;
    int dy = y1 > y2 ? -1 : 1;

    /*
     * byteMask gates per-byte writes (kernel copies use 0xffffffff;
     * partial-byte tail masks land here).
     */
    bool use_bytemask = (s->mte_bytemask != 0xffffffffu);

    if (!is_copy) {
        /* tiled fill: same walk as M7 (mte_zero / PROM textport) */
        for (int y = y1; y != y2 + dy; y += dy) {
            for (int x = x1; x != x2 + dx; x += dx) {
                sgi_crime_re_put_pixel(s, bufmode, x, y, fg);
            }
        }
        return;
    }

    /* COPY: src rect in the src TLB (tiled or linear) + y-step strides */
    bool src_linear = src_tlb == 4 || src_tlb == 5;
    int sx1 = (s->mte_src0 >> 16) & 0xfff;
    int sy1 = s->mte_src0 & 0xfff;
    sx1 /= bpp;
    int64_t lin_off = s->mte_src0;      /* linear src: byte offset */
    int32_t systep = (int32_t)s->mte_srcystep;
    int32_t dystep = (int32_t)s->mte_dstystep;
    int width = (x2 > x1 ? x2 - x1 : x1 - x2) + 1;
    int height = (y2 > y1 ? y2 - y1 : y1 - y2) + 1;
    const int MAX_RUN = 4096;           /* hard cap */

    for (int row = 0; row < height && row < MAX_RUN; row++) {
        int sy = sy1 + dy * row;
        if (src_linear) {
            int64_t off = lin_off + (int64_t)systep * row;
            int x = x1;
            for (int i = 0; i < width && i < MAX_RUN; i++, x += dx) {
                hwaddr phys;
                if (!sgi_crime_re_linear_addr(s, src_tlb - 4,
                                              (uint32_t)off, &phys)) {
                    off += bpp; continue;
                }
                uint8_t sv[4];
                address_space_rw(&address_space_memory, phys,
                                 MEMTXATTRS_UNSPECIFIED, sv, bpp, false);
                int fx = x, fy = y1 + dy * row;
                /* honor dx direction for the linear source walk */
                hwaddr dp;
                if (sgi_crime_re_fb_addr(s, bufmode, fx, fy, &dp)) {
                    if (use_bytemask) {
                        uint8_t dv[4];
                        address_space_rw(&address_space_memory, dp,
                                         MEMTXATTRS_UNSPECIFIED, dv, bpp,
                                         false);
                        for (int k = 0; k < bpp; k++) {
                            if (!((s->mte_bytemask >> (8 * k)) & 0xff)) {
                                sv[k] = dv[k];
                            }
                        }
                    }
                    address_space_rw(&address_space_memory, dp,
                                     MEMTXATTRS_UNSPECIFIED, sv, bpp, true);
                }
                off += bpp;
            }
        } else {
            int sx = sx1;
            int x = x1;
            for (int i = 0; i < width && i < MAX_RUN; i++, x += dx, sx += dx) {
                uint32_t color;
                hwaddr sp;
                uint8_t sv[4];
                if (!sgi_crime_re_tiled_addr(s, src_tlb, sx, sy, bpp, &sp)) {
                    continue;
                }
                address_space_rw(&address_space_memory, sp,
                                 MEMTXATTRS_UNSPECIFIED, sv, bpp, false);
                uint32_t pix_type = (bufmode >> BM_PIX_TYPE_SHIFT) & 3;
                if (pix_type == 0) {
                    color = sv[0];
                } else {
                    color = ((uint32_t)sv[0] << 24) | ((uint32_t)sv[1] << 16)
                          | ((uint32_t)sv[2] << 8) | sv[3];
                }
                int fy = y1 + dy * row + (dystep ? 0 : 0);
                sgi_crime_re_put_pixel(s, bufmode, x, fy, color);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

/* Store one 32-bit pixpipe register (offset relative to pixpipe base) */
static void sgi_crime_re_pp_store(SGICRIMEREState *s, hwaddr p, uint32_t v)
{
    switch (p) {
    /* ---- bufmodes / clip / draw ---- */
    case CRM_BUF_MODE_SRC_REG:
        s->bufmode_src = v;
        trace_sgi_crime_re_bufmode(v, s->bufmode_dst);
        return;
    case CRM_BUF_MODE_DST_REG:
        s->bufmode_dst = v;
        trace_sgi_crime_re_bufmode(s->bufmode_src, v);
        return;
    case CRM_CLIP_MODE_REG:      s->clipmode = v; return;
    case CRM_DRAW_MODE_REG:      s->drawmode = v; return;

    /* ---- screen masks (64-bit: hi=min, lo=max) ---- */
    case CRM_SCRMASK0_REG: case CRM_SCRMASK0_REG + 8:
    case CRM_SCRMASK0_REG + 16: case CRM_SCRMASK0_REG + 24:
    case CRM_SCRMASK0_REG + 32:
        s->scrmask[(p - CRM_SCRMASK0_REG) / 8] = v;
        return;

    /* ---- scissor + winoffset ---- */
    case CRM_SCISSOR_REG:
        s->scissor_lo = v;
        return;
    case CRM_SCISSOR_REG + 4:
        s->scissor_hi = v;
        return;
    case CRM_WINOFFSET_SRC_REG: s->winoffset_src = v; return;
    case CRM_WINOFFSET_DST_REG: s->winoffset_dst = v; return;

    /* ---- primitives / vertices ---- */
    case CRM_PRIMITIVE_REG:      s->primitive = v; return;
    case CRM_VERTEX_X_XY0_REG:   s->vertex_xy[0] = v; return;
    case CRM_VERTEX_X_XY1_REG:   s->vertex_xy[1] = v; return;
    case CRM_VERTEX_X_XY2_REG:   s->vertex_xy[2] = v; return;
    case CRM_VERTEX_GL_X0_REG:   s->vertex_gl[0][0] = v; return;
    case CRM_VERTEX_GL_Y0_REG:   s->vertex_gl[0][1] = v; return;
    case CRM_VERTEX_GL_X1_REG:   s->vertex_gl[1][0] = v; return;
    case CRM_VERTEX_GL_Y1_REG:   s->vertex_gl[1][1] = v; return;
    case CRM_VERTEX_GL_X2_REG:   s->vertex_gl[2][0] = v; return;
    case CRM_VERTEX_GL_Y2_REG:   s->vertex_gl[2][1] = v; return;
    case CRM_START_SETUP_REG:    s->startsetup = v; return;

    /* ---- pixel xfer ---- */
    case CRM_PIXELXFER_SRC_ADDR_REG:  s->pixelxfer_src_addr = v; return;
    case CRM_PIXELXFER_SRC_XSTEP_REG: s->pixelxfer_src_xstep = v; return;
    case CRM_PIXELXFER_SRC_YSTEP_REG: s->pixelxfer_src_ystep = v; return;
    case CRM_PIXELXFER_DST_LINADDR_REG:  s->pixelxfer_dst_linaddr = v; return;
    case CRM_PIXELXFER_DST_LINSTRIDE_REG: s->pixelxfer_dst_linstride = v; return;

    /* ---- stipple / shade ---- */
    case CRM_STIPPLE_MODE_REG:   s->stipple_mode = v; return;
    case CRM_STIPPLE_PATT_REG:   s->stipple_pattern = v; return;
    case CRM_SHADE_FGCOLOR_REG:  s->shade_fgcolor = v; return;
    case CRM_SHADE_BGCOLOR_REG:  s->shade_bgcolor = v; return;
    case CRM_SHADE_R0_REG: case CRM_SHADE_G0_REG: case CRM_SHADE_B0_REG:
    case CRM_SHADE_A0_REG:
    case CRM_SHADE_DRDX_REG: case CRM_SHADE_DGDX_REG:
    case CRM_SHADE_DRDY_REG: case CRM_SHADE_DGDY_REG:
    case CRM_SHADE_DBDX_REG: case CRM_SHADE_DADX_REG:
    case CRM_SHADE_DBDY_REG: case CRM_SHADE_DADY_REG:
        s->shade_plane[(p - CRM_SHADE_R0_REG) / 4] = v;
        return;

    /* ---- fog / logicop / colormask ---- */
    case CRM_FOG_COLOR_REG:      s->fog_color = v; return;
    case CRM_LOGICOP_REG:        s->logicop = v; return;
    case CRM_COLORMASK_REG:      s->colormask = v; return;

    default:
        /* texture/depth/stencil/alphatest/blend/antialias registers:
         * absorbed (stored nowhere yet — GL correctness for those
         * stages is future work; the writes must not fault) */
        return;
    }
}

/* Store one 32-bit MTE register (offset relative to MTE page base) */
static void sgi_crime_re_mte_store(SGICRIMEREState *s, hwaddr m, uint32_t v)
{
    switch (m) {
    case CRM_MTE_MODE_REG:        s->mte_mode = v; return;
    case CRM_MTE_BYTEMASK_REG:    s->mte_bytemask = v; return;
    case CRM_MTE_STIPPLEMASK_REG: s->mte_stipplemask = v; return;
    case CRM_MTE_FGVALUE_REG:     s->mte_fgvalue = v; return;
    case CRM_MTE_SRCYSTEP_REG:    s->mte_srcystep = v; return;
    case CRM_MTE_DSTYSTEP_REG:    s->mte_dstystep = v; return;
    case CRM_MTE_SRC0_REG:        s->mte_src0 = v; return;
    case CRM_MTE_SRC1_REG:        s->mte_src1 = v; return;
    case CRM_MTE_DST0_REG:        s->mte_dst0 = v; return;
    case CRM_MTE_DST1_REG:        s->mte_dst1 = v; return;
    default:
        return;                    /* null/flush: absorb */
    }
}

static uint64_t sgi_crime_re_read(void *opaque, hwaddr offset, unsigned size)
{
    SGICRIMEREState *s = SGI_CRIME_RE(opaque);

    switch (offset) {
    case CRM_RE_STATUS_REG: {
        /*
         * Compose the status word: all idle (crmWaitReIdle spins bits
         * 27|25; libGLcore FlushAndConfirm spins bit 28 — both must
         * see idle immediately), IB level 0 with ptrs equal (our RE
         * executes synchronously so the FIFO is always drained), and
         * the programmed start ptr echoed back.
         */
        uint32_t st = CRMSTAT_ALL_IDLE;
        uint32_t stptr = s->ib_ctl & CRMSTAT_IB_STPTR_MASK; /* echo */
        st |= stptr & CRMSTAT_IB_STPTR_MASK;
        return st;
    }

    case CRM_RE_INTFBUF_CTL:
        return s->ib_ctl;

    case CRM_RE_INTFBUF_DATA ... CRM_RE_INTFBUF_DATA + 0x1f8:
        if (size == 8) {
            return s->ib_data[(offset - CRM_RE_INTFBUF_DATA) / 8];
        }
        return 0;

    case CRM_RE_INTFBUF_ADDR ... CRM_RE_INTFBUF_ADDR + 0x1f8:
        if (size == 8) {
            return s->ib_addr[(offset - CRM_RE_INTFBUF_ADDR) / 8];
        }
        return 0;

    default:
        /*
         * crmGetRERev reads 0x0400 (IntfBuf.ctl) and takes bit 0 to
         * decide RE revision; 0 = rev B ("SGI-CRM, Rev B"). ib_ctl bit
         * 0 stays 0 unless programmed — keep the read faithful.
         */
        if (offset == CRM_RE_INTFBUF_CTL) {
            return s->ib_ctl;
        }
        return 0;
    }
}

static void sgi_crime_re_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    SGICRIMEREState *s = SGI_CRIME_RE(opaque);
    bool go = (offset & CRM_GO_OFFSET) != 0;
    hwaddr off = offset & ~CRM_GO_OFFSET;

    /* ---- Interface buffer (0x0000-0x0FFF) ---- */
    if (off < CRM_RE_TLB_BASE) {
        if (off < CRM_RE_INTFBUF_DATA + 8 * (hwaddr)CRIME_FIFO_DEPTH) {
            s->ib_data[(off - CRM_RE_INTFBUF_DATA) / 8] = value;
            return;
        }
        if (off >= CRM_RE_INTFBUF_ADDR &&
            off < CRM_RE_INTFBUF_ADDR + 8 * (hwaddr)CRIME_FIFO_DEPTH) {
            s->ib_addr[(off - CRM_RE_INTFBUF_ADDR) / 8] = value;
            return;
        }
        if (off == CRM_RE_INTFBUF_CTL) {
            s->ib_ctl = (uint32_t)value;
            /*
             * Counted-buffer bookkeeping (11b): track the posted-entry
             * count. Our engine drains instantly, so the level exposed
             * in the status word stays 0 and the RE3 (empty)/RE4
             * (full) watermark lines never fire spuriously — the
             * kernel's ibctl programming simply round-trips.
             */
            s->ib_count = 0;
            return;
        }
        return;
    }

    /* ---- TLB page (0x1000-0x17FF) ---- */
    if (off >= CRM_RE_TLB_BASE && off < CRM_RE_PIXPIPE_BASE) {
        hwaddr t = off - CRM_RE_TLB_BASE;
        if (size == 8) {
            if (t < 0x200) {
                s->tlb_fb[0][t / 8] = value;
                trace_sgi_crime_re_tlb_write(0, t / 8, value);
            } else if (t < 0x400) {
                s->tlb_fb[1][(t - 0x200) / 8] = value;
                trace_sgi_crime_re_tlb_write(1, (t - 0x200) / 8, value);
            } else if (t < 0x600) {
                s->tlb_fb[2][(t - 0x400) / 8] = value;
                trace_sgi_crime_re_tlb_write(2, (t - 0x400) / 8, value);
            } else if (t < CRM_TLB_CID_OFFSET) {
                s->tlb_tex[(t - CRM_TLB_TEX_OFFSET) / 8] = value;
            } else if (t < CRM_TLB_LINEAR_A_OFFSET) {
                s->tlb_cid[(t - CRM_TLB_CID_OFFSET) / 8] = value;
            } else if (t < CRM_TLB_LINEAR_A_OFFSET + 0x80) {
                s->tlb_linear[0][(t - CRM_TLB_LINEAR_A_OFFSET) / 8] = value;
            } else if (t < CRM_TLB_LINEAR_B_OFFSET + 0x80) {
                s->tlb_linear[1][(t - CRM_TLB_LINEAR_B_OFFSET) / 8] = value;
            }
        }
        return;
    }

    /* ---- Pixel pipe (0x2000-0x21FF, +go at 0x2800) ---- */
    if (off >= CRM_RE_PIXPIPE_BASE && off < CRM_RE_MTE_BASE) {
        hwaddr p = off - CRM_RE_PIXPIPE_BASE;
        uint32_t v = (uint32_t)value;

        if (size == 8) {
            /*
             * 64-bit write covers two adjacent 32-bit regs (BE: the
             * value's low word is at the lower address — the M7 note:
             * "v = high word (first BE word)" for the *register file*
             * order). libGLcore's 64-bit Vertex.GL stores are
             * (x << 32) | y written to 0x2080/0x2088/0x2090, i.e. the
             * first word at p is the value's HIGH half.
             */
            sgi_crime_re_pp_store(s, p, (uint32_t)(value >> 32));
            sgi_crime_re_pp_store(s, p + 4, v);
        } else {
            sgi_crime_re_pp_store(s, p, v);
        }

        /*
         * crmSetAndGo ORs 0x800 into the register offset — the kick.
         * Any pixpipe-page write with the go bit executes the pending
         * primitive (the last register write of the batch commits it,
         * spec §7.3.3.1 "the host indicates ... by offsetting its
         * last register write by START_OFFSET").
         */
        if (go) {
            sgi_crime_re_draw(s);
        }
        return;
    }

    /* ---- MTE (0x3000-0x307F, +go at 0x3800) ---- */
    if (off >= CRM_RE_MTE_BASE && off < CRM_RE_STATUS_BASE) {
        hwaddr m = off - CRM_RE_MTE_BASE;
        uint32_t v = (uint32_t)value;

        if (size == 8) {
            switch (m) {
            case CRM_MTE_SRC0_REG: s->mte_src0 = value; break;
            case CRM_MTE_SRC1_REG: s->mte_src1 = value; break;
            case CRM_MTE_DST0_REG: s->mte_dst0 = value; break;
            case CRM_MTE_DST1_REG: s->mte_dst1 = value; break;
            default: {
                hwaddr m_hi = m, m_lo = m + 4;
                sgi_crime_re_mte_store(s, m_hi, (uint32_t)(value >> 32));
                sgi_crime_re_mte_store(s, m_lo, v);
                break;
            }
            }
        } else {
            sgi_crime_re_mte_store(s, m, v);
        }

        /* MTE_SET32_AND_GO(MTE_MODE, op) — the go bit starts it */
        if (go) {
            sgi_crime_re_mte_run(s);
        }
        return;
    }

    /* ---- Status page (0x4000) ---- */
    if (off >= CRM_RE_STATUS_BASE) {
        return;                        /* read-only; absorb writes */
    }
}

static const MemoryRegionOps sgi_crime_re_ops = {
    .read = sgi_crime_re_read,
    .write = sgi_crime_re_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void sgi_crime_re_realize(DeviceState *dev, Error **errp)
{
    SGICRIMEREState *s = SGI_CRIME_RE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_crime_re_ops, s,
                          "sgi-crime-re", CRIME_RE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void sgi_crime_re_reset(DeviceState *dev)
{
    SGICRIMEREState *s = SGI_CRIME_RE(dev);
    memset(s->tlb_fb, 0, sizeof(s->tlb_fb));
    memset(s->tlb_tex, 0, sizeof(s->tlb_tex));
    memset(s->tlb_cid, 0, sizeof(s->tlb_cid));
    memset(s->tlb_linear, 0, sizeof(s->tlb_linear));
    memset(s->ib_data, 0, sizeof(s->ib_data));
    memset(s->ib_addr, 0, sizeof(s->ib_addr));
    s->ib_ctl = 0;
    s->ib_count = 0;
    s->bufmode_src = 0;
    s->bufmode_dst = 0;
    s->clipmode = 0;
    s->drawmode = 0;
    memset(s->scrmask, 0, sizeof(s->scrmask));
    s->scissor_lo = s->scissor_hi = 0;
    s->winoffset_src = 0;
    s->winoffset_dst = 0;
    s->primitive = 0;
    memset(s->vertex_xy, 0, sizeof(s->vertex_xy));
    memset(s->vertex_gl, 0, sizeof(s->vertex_gl));
    s->startsetup = 0;
    s->pixelxfer_src_addr = 0;
    s->pixelxfer_src_xstep = 0;
    s->pixelxfer_src_ystep = 0;
    s->pixelxfer_dst_linaddr = 0;
    s->pixelxfer_dst_linstride = 0;
    s->stipple_mode = 0;
    s->stipple_pattern = 0;
    s->shade_fgcolor = 0;
    s->shade_bgcolor = 0;
    memset(s->shade_plane, 0, sizeof(s->shade_plane));
    s->fog_color = 0;
    s->logicop = 0;
    s->colormask = 0;
    s->mte_mode = 0;
    s->mte_bytemask = 0;
    s->mte_stipplemask = 0;
    s->mte_fgvalue = 0;
    s->mte_src0 = 0;
    s->mte_src1 = 0;
    s->mte_dst0 = 0;
    s->mte_dst1 = 0;
    s->mte_srcystep = 0;
    s->mte_dstystep = 0;
    s->status = CRMSTAT_ALL_IDLE;
}

static const VMStateDescription vmstate_sgi_crime_re = {
    .name = "sgi-crime-re",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(status, SGICRIMEREState),
        VMSTATE_UINT64_ARRAY(ib_data, SGICRIMEREState, CRIME_FIFO_DEPTH),
        VMSTATE_UINT64_ARRAY(ib_addr, SGICRIMEREState, CRIME_FIFO_DEPTH),
        VMSTATE_UINT32(ib_ctl, SGICRIMEREState),
        VMSTATE_UINT32(ib_count, SGICRIMEREState),
        VMSTATE_UINT64_2DARRAY(tlb_fb, SGICRIMEREState, 3,
                               CRM_TLB_FB_ENTRIES),
        VMSTATE_UINT64_ARRAY(tlb_tex, SGICRIMEREState, CRM_TLB_TEX_ENTRIES),
        VMSTATE_UINT64_ARRAY(tlb_cid, SGICRIMEREState, CRM_TLB_CID_ENTRIES),
        VMSTATE_UINT64_2DARRAY(tlb_linear, SGICRIMEREState, 2,
                               CRM_TLB_LINEAR_ENTRIES),
        VMSTATE_UINT32(bufmode_src, SGICRIMEREState),
        VMSTATE_UINT32(bufmode_dst, SGICRIMEREState),
        VMSTATE_UINT32(clipmode, SGICRIMEREState),
        VMSTATE_UINT32(drawmode, SGICRIMEREState),
        VMSTATE_UINT32_ARRAY(scrmask, SGICRIMEREState, 5),
        VMSTATE_UINT32(scissor_lo, SGICRIMEREState),
        VMSTATE_UINT32(scissor_hi, SGICRIMEREState),
        VMSTATE_UINT32(winoffset_src, SGICRIMEREState),
        VMSTATE_UINT32(winoffset_dst, SGICRIMEREState),
        VMSTATE_UINT32(primitive, SGICRIMEREState),
        VMSTATE_UINT32_ARRAY(vertex_xy, SGICRIMEREState, 3),
        VMSTATE_UINT32_2DARRAY(vertex_gl, SGICRIMEREState, 3, 2),
        VMSTATE_UINT32(startsetup, SGICRIMEREState),
        VMSTATE_UINT32(pixelxfer_src_addr, SGICRIMEREState),
        VMSTATE_UINT32(pixelxfer_src_xstep, SGICRIMEREState),
        VMSTATE_UINT32(pixelxfer_src_ystep, SGICRIMEREState),
        VMSTATE_UINT32(pixelxfer_dst_linaddr, SGICRIMEREState),
        VMSTATE_UINT32(pixelxfer_dst_linstride, SGICRIMEREState),
        VMSTATE_UINT32(stipple_mode, SGICRIMEREState),
        VMSTATE_UINT32(stipple_pattern, SGICRIMEREState),
        VMSTATE_UINT32(shade_fgcolor, SGICRIMEREState),
        VMSTATE_UINT32(shade_bgcolor, SGICRIMEREState),
        VMSTATE_UINT32_ARRAY(shade_plane, SGICRIMEREState, 12),
        VMSTATE_UINT32(fog_color, SGICRIMEREState),
        VMSTATE_UINT32(logicop, SGICRIMEREState),
        VMSTATE_UINT32(colormask, SGICRIMEREState),
        VMSTATE_UINT32(mte_mode, SGICRIMEREState),
        VMSTATE_UINT32(mte_bytemask, SGICRIMEREState),
        VMSTATE_UINT32(mte_stipplemask, SGICRIMEREState),
        VMSTATE_UINT32(mte_fgvalue, SGICRIMEREState),
        VMSTATE_UINT64(mte_src0, SGICRIMEREState),
        VMSTATE_UINT64(mte_src1, SGICRIMEREState),
        VMSTATE_UINT64(mte_dst0, SGICRIMEREState),
        VMSTATE_UINT64(mte_dst1, SGICRIMEREState),
        VMSTATE_UINT32(mte_srcystep, SGICRIMEREState),
        VMSTATE_UINT32(mte_dstystep, SGICRIMEREState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_crime_re_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_crime_re_realize;
    device_class_set_legacy_reset(dc, sgi_crime_re_reset);
    dc->vmsd = &vmstate_sgi_crime_re;
}

static const TypeInfo sgi_crime_re_info = {
    .name = TYPE_SGI_CRIME_RE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGICRIMEREState),
    .class_init = sgi_crime_re_class_init,
};

static void sgi_crime_re_register_types(void)
{
    type_register_static(&sgi_crime_re_info);
}

type_init(sgi_crime_re_register_types)
