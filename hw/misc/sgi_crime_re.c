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
#include "exec/cpu-common.h"
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

/*
 * @@SEMANTICS@@ — byte lane of an 8-bit CI pixel inside a 32-bit-word CI
 * buffer.
 *
 * The O2 X server runs its 8bpp PseudoColor screen through the 32bpp
 * normal planes using the GBE's "8+8 split": one pixel per 32-bit word,
 * held in a byte lane.  The DDX writes a pixel's index as the LOW byte of
 * the word's 16-bit pixel slot, i.e. byte1, NOT the word's first byte.
 * Decisive live evidence (gr_osview's meter, the one window that mixes
 * widths): its background is filled with a 16-bit CI store (BufMode.dst
 * pixDepth=1, fg=0x2e), which lands at byte0=0x00 / byte1=0x2e — the
 * 16-bit value big-endian in bytes [0,1].  Its meter bars are filled with
 * an 8-bit CI store (pixDepth=0, fg=0x00..0x14) whose index must therefore
 * occupy the same low byte (byte1) of that slot; the old code wrote it to
 * byte0, so the GBE's I12 scan (which reads bytes [0,1] as the 12-bit
 * index) saw (0x00 << 8) | 0x2e for a black bar — i.e. the background
 * index 0x2e — and the bars vanished, leaving a flat field.  Moving the
 * 8-bit store to byte1 (and the matching CI read paths) restores the bars
 * and leaves the main screen byte-identical on screen (verified against
 * the Indy oracle: granite #868686 78711 + #69b5b5 76713 exactly, and the
 * Toolchest / Icon-Catalog text unchanged).
 */
static inline int crim_ci_lane(uint32_t bufmode)
{
    return (((bufmode >> BM_BUF_DEPTH_SHIFT) & 3) == 2 &&
            ((bufmode >> BM_PIX_DEPTH_SHIFT) & 3) == 0) ? 1 : 0;
}

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
    /*
     * M11k: the coordinate space is 2048x2048 PIXELS, but the MTE path
     * feeds BYTE-x coordinates (gxemul's dev_sgi_re divides by depth/8 —
     * and the X DDX's 8bpp fills reach byte-x 0x13ff = 5119 for a
     * 1280-word row). The old 2048 bound rejected every byte-x past 2047,
     * silently truncating the DDX's full-screen fills to the left 1024
     * bytes (= the 256-pixel strip observed on the v3 gate). The physical
     * space still caps at 16 tile columns x 16 rows (64KB*256 = the full
     * RE TLB reach, 512px/col at 8bpp = 8192 byte-x); accept the wider
     * x and let the tile_nr math + TLB-valid decode handle the bounds.
     */
    if (x < 0 || y < 0 || x >= 8192 || y >= 2048) {
        return false;
    }
    int tile_w = 512 / bpp;             /* pixels across one tile */
    int tile_nr = (y >> 7) * 16 + x / tile_w;
    if (tile_nr >= 256) {
        return false;                  /* beyond the 64-entry/256-tile TLB */
    }
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
/* Depth / stencil (SZ) buffer                                          */
/* ------------------------------------------------------------------ */

/* Evaluate one depth-test comparison (spec §7.3.7.17 Table 7-25). */
static bool crim_depth_func_pass(uint32_t func, uint32_t zs, uint32_t zd)
{
    switch (func & 7) {
    case CRM_DEPTH_FUNC_NEVER:    return false;
    case CRM_DEPTH_FUNC_LESS:     return zs <  zd;
    case CRM_DEPTH_FUNC_EQUAL:    return zs == zd;
    case CRM_DEPTH_FUNC_LEQUAL:   return zs <= zd;
    case CRM_DEPTH_FUNC_GREATER:  return zs >  zd;
    case CRM_DEPTH_FUNC_NOTEQUAL: return zs != zd;
    case CRM_DEPTH_FUNC_GEQUAL:   return zs >= zd;
    default:                      return true;   /* ALWAYS */
    }
}

/*
 * Read the 24-bit Z field of the SZ pixel at (x, y), a *buffer*-space
 * coordinate, through framebuffer TLB C (see CRM_DEPTH_TLB_SEL).  The SZ
 * pixel is 32-bit big-endian: byte0 = stencil, bytes 1..3 = 24-bit Z
 * (spec §7.3.4.4 Fig 7-6 "S | Z").  Returns false if the tile is
 * unmapped, in which case there is no depth buffer to test against.
 */
static bool sgi_crime_re_depth_read(SGICRIMEREState *s, int x, int y,
                                    uint32_t *z)
{
    hwaddr phys;
    uint8_t b[4];

    if (!sgi_crime_re_tiled_addr(s, CRM_DEPTH_TLB_SEL, x, y, 4, &phys)) {
        return false;
    }
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, 4, false);
    *z = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    return true;
}

static void sgi_crime_re_depth_write(SGICRIMEREState *s, int x, int y,
                                     uint32_t z)
{
    hwaddr phys;
    uint8_t b[4];

    if (!sgi_crime_re_tiled_addr(s, CRM_DEPTH_TLB_SEL, x, y, 4, &phys)) {
        return;
    }
    /* Preserve the stencil byte; replace only the 24-bit Z field. */
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, 4, false);
    b[1] = (z >> 16) & 0xff;
    b[2] = (z >> 8) & 0xff;
    b[3] = z & 0xff;
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, 4, true);
}

/*
 * Truncate the 25.12 plane-stepped z to the 24-bit value assigned to the
 * fragment, clamping to [0, 2^24) (spec §7.3.7.16: "generated by
 * truncating the fractional part of this 25.12 value and then clamping it
 * to the 24-bit range [0,2^24)").
 */
static inline uint32_t crim_depth_value(int64_t z25)
{
    int64_t z = z25 >> 12;              /* arithmetic shift: floor */
    if (z < 0) {
        return 0;
    }
    if (z > (int64_t)CRM_SZ_DEPTH_MASK) {
        return CRM_SZ_DEPTH_MASK;
    }
    return (uint32_t)z;
}

/*
 * Apply the depth test to one fragment at *buffer* coordinates (x, y)
 * (the rasteriser works in GL/window space and clip_pass() has already
 * applied WinOffset.dst).  z is the fragment's 24-bit depth.  Returns
 * true if the fragment survives; writes the new depth when it does and
 * DrawMode.enDepthMask is set.  On an unmapped depth tile the fragment is
 * passed through (the desktop 2D path uses depth-less BufModes).
 */
static bool sgi_crime_re_depth_test(SGICRIMEREState *s, int x, int y,
                                    uint32_t z)
{
    uint32_t zd;
    uint32_t func = (s->depth_mode & CRM_DEPTH_FUNC_MASK)
                    >> CRM_DEPTH_FUNC_SHIFT;

    if (!sgi_crime_re_depth_read(s, x, y, &zd)) {
        return true;
    }
    if (!crim_depth_func_pass(func, z, zd)) {
        return false;
    }
    if (s->drawmode & DM_ENDEPTHMASK) {
        sgi_crime_re_depth_write(s, x, y, z);
    }
    return true;
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
    uint32_t pix_type = (bufmode >> BM_PIX_TYPE_SHIFT) & 3;
    uint32_t buf_d = (bufmode >> BM_BUF_DEPTH_SHIFT) & 3;
    uint32_t pix_d = (bufmode >> BM_PIX_DEPTH_SHIFT) & 3;
    int px_bpp = 1 << pix_d;          /* PIXEL width (1/2/4) */
    int word_bpp = 1 << buf_d;        /* WORD width (1/2/4) */
    int bpp = word_bpp;

    if (!sgi_crime_re_fb_addr(s, bufmode, x, y, &phys)) {
        return 0;
    }

    /*
     * Mirror put_pixel()'s addressing: an 8-bit CI pixel in a 32-bit-word
     * CI buffer lives in byte1, and a 16-bit pixel in a double-buffered
     * word owns one half selected by doublePixSel.  Reading the whole
     * 32-bit word at the un-offset address (the old behaviour) returned a
     * different half's pixel for a 16-bit double-buffered destination, so
     * a ROP read-modify-write or an X/GL read-back of the GL window read
     * the wrong pixel.
     */
    if (pix_type == 0 && crim_ci_lane(bufmode)) {
        phys += 1;
    }
    if (pix_type != 0 && ((bufmode >> 1) & 1) && px_bpp == 2 &&
        px_bpp < word_bpp) {
        phys += (bufmode & 1) ? (word_bpp - px_bpp) : 0;
    }

    /*
     * 16-bit RGB(A) pixel in a wider word: A1_RGB5 (spec Figure 7-5,
     * bits [15]=A [14:10]=R [9:5]=G [4:0]=B).  Unpack to the canonical
     * RGBA fragment (R=31:24..A=7:0), expanding each 5-bit component to 8
     * by bit replication exactly as the GBE scanout does.
     */
    if (pix_type != 0 && px_bpp == 2 && px_bpp < word_bpp) {
        uint32_t v;
        address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                         b, 2, false);
        v = ((uint32_t)b[0] << 8) | b[1];
        uint32_t r5 = (v >> 10) & 0x1f, g5 = (v >> 5) & 0x1f, b5 = v & 0x1f;
        uint32_t a = (v >> 15) & 1;
        return (((r5 << 3) | (r5 >> 2)) << 24)
             | (((g5 << 3) | (g5 >> 2)) << 16)
             | (((b5 << 3) | (b5 >> 2)) << 8)
             | (a ? 0xff : 0);
    }

    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, bpp, false);

    switch (pix_type) {
    case 0:                             /* color index (8/16-bit) */
        if (((bufmode >> BM_PIX_DEPTH_SHIFT) & 3) == 1) {
            return ((uint32_t)b[0] << 8) | b[1];
        }
        return b[crim_ci_lane(bufmode)];
    case 1:                             /* RGB: ABGR in memory, alpha=0 */
        return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) |
               ((uint32_t)b[1] << 8);
    default:                            /* ABGR in memory -> canonical RGBA */
        return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) |
               ((uint32_t)b[1] << 8) | b[0];
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
 * Ordered dither (DrawMode.enDither, bit 8).
 *
 * The CRIME 1.5 spec does NOT define the algorithm — §7.3.7.13
 * "Dithering" is literally the single word "TBD" — but it does bound the
 * feature twice: the pipeline-order section §7.3.7.14 says a logic op is
 * applied "between the fragment's source color value *after dithering*
 * and the corresponding destination color value read from the
 * framebuffer" (so dither precedes the logic op), and the feature list
 * scopes it to "Dithering for 8-bit and 16-bit RGB pixels".
 *
 * We therefore use the conventional ordered (Bayer) dither: a 4x4 matrix
 * applied to the low bits of each 8-bit component when the component is
 * being reduced to fewer bits.  For a 24-bit RGB destination stored as
 * RGB5 (the 16-bit case) the low 3 bits of R/B and low 2 bits of G are
 * added to a threshold; for a plain 8-bit component the dither resolves
 * the fractional error introduced by rounding to the framebuffer's
 * precision.
 *
 * @@SEMANTICS@@ — spec-silent; this is the conventional model, marked so
 * the next reader can challenge it against a real-hardware reference.
 * NOT applied to colour-index (pix_type 0) pixels: the feature list scopes
 * enDither to 8/16-bit RGB, and dithering an index is not meaningful
 * (the index is a table lookup, not an intensity).  Callers gate on
 * pix_type; see sgi_crime_re_put_pixel.
 */
static const uint8_t crim_bayer4[4][4] = {
    {  0,  8,  2, 10 },
    { 12,  4, 14,  6 },
    {  3, 11,  1,  9 },
    { 15,  7, 13,  5 },
};

/*
 * Dither the three 8-bit RGB components of a packed 0xRRGGBB(A) fragment
 * for a destination of `bits` bits per component (5 or 8).  The Bayer
 * threshold is scaled to the discarded range so that the average value
 * is preserved; a component already at 8 bits is left unchanged.
 */
static inline uint32_t crim_dither_rgb(uint32_t color, int x, int y, int bits)
{
    if (bits >= 8) {
        return color;                   /* already 8 bits/component */
    }
    int discard = 8 - bits;             /* bits dropped per component */
    int range = 1 << discard;           /* number of dropped codes */
    int thr = crim_bayer4[y & 3][x & 3];   /* 0..15 */
    /* Map the 16-level Bayer threshold onto the dropped range. */
    int bias = (thr * range) >> 4;
    uint32_t r = (color >> 24) & 0xff;
    uint32_t g = (color >> 16) & 0xff;
    uint32_t bl = (color >> 8) & 0xff;
    r = (r + bias) > 0xff ? 0xff : r + bias;
    g = (g + bias) > 0xff ? 0xff : g + bias;
    bl = (bl + bias) > 0xff ? 0xff : bl + bias;
    /* Preserve the alpha/attribute byte (bits 7:0) untouched. */
    return (color & 0x000000ffu) | (r << 24) | (g << 16) | (bl << 8);
}

/*
 * RGB component precision a BufMode denotes, per BufMode.pixDepth:
 * 0 = 8-bit, 1 = 16-bit, 2 = 32-bit (spec §7.3.4 BufMode).  Only the
 * 16-bit form (RGB5) actually discards component precision, so that is
 * the only case where ordered dithering has anything to resolve.
 */
static inline int crim_bufmode_rgb_bits(uint32_t bufmode)
{
    switch ((bufmode >> BM_BUF_DEPTH_SHIFT) & 3) {
    case 1:  return 5;                  /* 16-bit RGB5 */
    default: return 8;                  /* 8/32-bit: no reduction */
    }
}

/*
 * Write one pixel through the dst BufMode with ROP/masking applied.
 *
 * @@SEMANTICS@@ — the canonical fragment colour is RGBA (R=31:24..A=7:0;
 * see crim_dither_rgb).  The O2 normal-plane 32bpp surface is ABGR in
 * memory (bytes A,B,G,R), matching commit 79ecd6a182's tile order and
 * the GBE RGB8 scanout (r=buf[3], g=buf[2], b=buf[1]).  put_pixel packs
 * RGBA->ABGR here; get_pixel and xfer_fetch reverse it.  RGB surfaces
 * (pixType 1) carry no alpha, so their byte0 is written 0 and reads are
 * alpha-less.  8/16-bit CI (pixType 0) is unaffected.
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
    uint32_t buf_d = (bufmode >> BM_BUF_DEPTH_SHIFT) & 3;
    uint32_t pix_d = (bufmode >> BM_PIX_DEPTH_SHIFT) & 3;
    int px_bpp = 1 << pix_d;          /* destination PIXEL width (1/2/4) */
    int word_bpp = 1 << buf_d;        /* destination WORD width (1/2/4)  */
    bool dpix = (bufmode >> 1) & 1;   /* doublePix: word split front/back */
    int dpix_sel = bufmode & 1;       /* which half is drawn             */

    /*
     * @@SEMANTICS@@ — an 8-bit CI pixel in a 32-bit-word CI buffer lives in
     * the low byte of its 16-bit pixel slot (byte1; see crim_ci_lane), so
     * offset the address once here: every read (ROP/byte-mask) and the final
     * store then target the same byte.
     */
    if (pix_type == 0 && crim_ci_lane(bufmode)) {
        phys += 1;
    }

    /*
     * @@SEMANTICS@@ — 16+16 / 8+8 double buffering.  CRIME 1.5 Table 7-5:
     * BufMode.bufDepth is the buffer WORD depth, pixDepth the PIXEL depth,
     * and doublePix "each word of the color buffer is split into a front and
     * back half for double buffering" with doublePixSel choosing the half.
     * The O2 GL visual is the 16-bit A1_RGB5 pixel (spec Fig 7-5) packed two
     * per 32-bit word (the DDX programs dst BufMode 0x426/0x427: bufDepth=32,
     * pixDepth=16, pixType=RGBA, doublePix=1, sel 0/1), and the GBE scans the
     * same half via WID typ RGB5 + buf BOTTOM(1)/TOP(2).  A pixel-depth below
     * the word depth therefore owns one half of the word, selected by
     * doublePixSel (lower half when clear, upper when set).
     */
    if (pix_type != 0 && dpix && px_bpp == 2 && px_bpp < word_bpp) {
        phys += dpix_sel ? (word_bpp - px_bpp) : 0;
    }

    /*
     * Dithering (spec §7.3.7.14: dither precedes the logic op).  RGB
     * destinations only — the feature list scopes enDither to 8/16-bit
     * RGB, and a colour index carries no intensity to dither.
     */
    if ((s->drawmode & DM_ENDITHER) && pix_type != 0) {
        color = crim_dither_rgb(color, x, y, crim_bufmode_rgb_bits(bufmode));
    }

    switch (pix_type) {
    case 0: {                           /* color index / packed 16-bit */
        /*
         * BufMode.pixDepth sets the stored CI width.  A 16-bit CI
         * destination (pixDepth=1, e.g. the O2 depth-15 RGB5 windows
         * whose packed value is not an index but an RGB5 pixel) must
         * store the full 16-bit value: truncating to the low byte made
         * the GBE read an unallocated cmap index and render black.
         * 8-bit CI (pixDepth=0) stays byte-identical.
         */
        if (((bufmode >> BM_PIX_DEPTH_SHIFT) & 3) == 1) {
            bpp = 2;
            b[0] = (color >> 8) & 0xff;
            b[1] = color & 0xff;
        } else {
            bpp = 1;
            b[0] = color & 0xff;
        }
        break;
    }
    case 1:                             /* RGB canonical -> ABGR memory */
    case 2:                             /* RGBA canonical -> ABGR memory */
    default:
        if (pix_type != 0 && px_bpp < word_bpp && px_bpp == 2) {
            /*
             * 16-bit RGB(A) pixel in a 32-bit word (16+16 double buffer):
             * pack the canonical RGBA into A1_RGB5 (spec CRIME 1.5 Fig 7-5,
             * bits [15]=A [14:10]=R [9:5]=G [4:0]=B) and store it, big-
             * endian, in the doublePixSel half selected above.  The GBE
             * WID typ RGB5 + buf reads exactly this half.
             */
            uint32_t R = (color >> 24) & 0xff;
            uint32_t G = (color >> 16) & 0xff;
            uint32_t B = (color >> 8) & 0xff;
            uint32_t A = color & 0xff;
            uint32_t v = (((A >> 7) & 1) << 15) | ((R >> 3) << 10) |
                         ((G >> 3) << 5) | (B >> 3);
            bpp = 2;
            b[0] = (v >> 8) & 0xff;
            b[1] = v & 0xff;
        } else {
            bpp = 4;
            b[0] = (pix_type == 1) ? 0 : color;
            b[1] = color >> 8;
            b[2] = color >> 16;
            b[3] = color >> 24;
        }
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
            if (bpp == 2) {
                b[0] = (color >> 8) & 0xff;
                b[1] = color & 0xff;
            } else {
                b[0] = color & 0xff;
            }
        } else if (bpp == 2) {
            /* 16-bit A1_RGB5 in a double-buffered word (see the switch). */
            uint32_t R = (color >> 24) & 0xff;
            uint32_t G = (color >> 16) & 0xff;
            uint32_t B = (color >> 8) & 0xff;
            uint32_t A = color & 0xff;
            uint32_t v = (((A >> 7) & 1) << 15) | ((R >> 3) << 10) |
                         ((G >> 3) << 5) | (B >> 3);
            b[0] = (v >> 8) & 0xff;
            b[1] = v & 0xff;
        } else {
            b[0] = (pix_type == 1) ? 0 : color;
            b[1] = color >> 8;
            b[2] = color >> 16; b[3] = color >> 24;
        }
    }
    if (dm & DM_ENCOLORMASK) {
        /* ColorMask: bit per color component (RGBA planes) — apply to
         * the 4-byte forms by byte-lane; CI covers its 8/16-bit width. */
        uint32_t m = s->colormask;
        uint8_t ob[4];
        address_space_rw(&address_space_memory, phys,
                         MEMTXATTRS_UNSPECIFIED, ob, bpp, false);
        if (bpp == 1) {
            b[0] = (b[0] & m) | (ob[0] & ~m);
        } else if (pix_type == 0) {
            b[0] = (b[0] & m) | (ob[0] & ~m);
            b[1] = (b[1] & (m >> 8)) | (ob[1] & ~(m >> 8));
        } else {
            if (!(m & 0xff000000u)) { b[0] = ob[0]; }
            if (!(m & 0x00ff0000u)) { b[1] = ob[1]; }
            if (!(m & 0x0000ff00u)) { b[2] = ob[2]; }
            if (!(m & 0x000000ffu)) { b[3] = ob[3]; }
        }
    }

    /*
     * @@SEMANTICS@@ — DrawMode.enColorByteMask (crimedef.h
     * DM_ENCOLORBYTEMASK = bits [6:3]): one write-enable bit per
     * destination byte lane, bit 3 -> the first byte b[0], bit 6 -> the
     * last b[3] (the same low-bit-first lane order the MTE.byteMask uses,
     * where a clear bit leaves the destination byte untouched).  A clear
     * bit preserves the destination byte.
     *
     * The O2 X DDX builds the clogin greeter panel's 1x1 dither by
     * pixel-transferring a small pattern pixmap with fg 0xC0C0C000 /
     * 0xC1C1C100 and byte mask 0xe: the three colour lanes are written
     * and the alpha lane is preserved, so the RE composes the grey over
     * whatever alpha the surface already holds.  With the canonical RGBA
     * fragment packed ABGR into memory (R in the last byte), mask 0xe
     * enables bytes 1..3 = B,G,R and leaves byte0 = alpha.
     *
     * The previous code ignored the field entirely and wrote all four
     * bytes of every fill.  That is only benign once the pixel lane order
     * is right (which it now is); before the ABGR pack it clobbered the
     * lanes the guest was preserving.
     */
    {
        uint32_t bmask = (dm >> 3) & 0xf;
        if (bmask != 0xf) {
            uint8_t ob[4];
            address_space_rw(&address_space_memory, phys,
                             MEMTXATTRS_UNSPECIFIED, ob, bpp, false);
            for (int k = 0; k < bpp; k++) {
                if (!((bmask >> k) & 1)) {
                    b[k] = ob[k];
                }
            }
        }
    }
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                      b, bpp, true);
}

/*
 * @@SEMANTICS@@ — an MTE fill's per-byte write-enable is MTE.byteMask, NOT
 * DrawMode.enColorByteMask.  The two are separate registers and the MTE
 * engine has no DrawMode; routing an MTE fill's store through
 * sgi_crime_re_put_pixel() made it apply whatever DrawMode.enColorByteMask
 * happened to hold, which is the pixel-pipe's mask.
 *
 * Observed live (fm dirview canvas): the DDX paints the background with a
 * stippled MTE CLEAR at fg=0x729c9c00 (scheme AlternateBackground4) while
 * DrawMode.enColorByteMask is 0x1 / 0x3, so put_pixel wrote only bytes 0/1
 * of the ABGR word (A, B) and dropped R and G — the canvas scanned out as
 * B-only blue (0,0,191) instead of teal (159,191,191 after gamma).  The
 * MTE's own byteMask was 0xeeeeeeee (every byte lane enabled), i.e. the
 * guest asked for all four bytes.  Derive the nibble from MTE.byteMask
 * (per-byte enable: a nonzero byte lane writes) and impose it for the fill.
 */
static void sgi_crime_re_put_pixel_mte(SGICRIMEREState *s, uint32_t bufmode,
                                       int x, int y, uint32_t color)
{
    uint32_t saved = s->drawmode;
    uint32_t nib = 0;

    for (int k = 0; k < 4; k++) {
        if ((s->mte_bytemask >> (8 * k)) & 0xff) {
            nib |= (1u << k);
        }
    }
    s->drawmode = (saved & ~DM_ENCOLORBYTEMASK) | ((nib & 0xf) << 3);
    sgi_crime_re_put_pixel(s, bufmode, x, y, color);
    s->drawmode = saved;
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
        /*
         * @@SEMANTICS@@ — Scissor (CRIME 1.5 spec §7.3.1.9 Table 7-10): a
         * 64-bit window-relative rectangle, bits [63:48]=min.x,
         * [47:32]=min.y, [31:16]=max.x, [15:0]=max.y.  crmSetAndGo writes it
         * as the same two-word pair as ScrMask: the register at 0x048 (held
         * here in scissor_lo) is the HIGH word = the min corner, and 0x04c
         * (scissor_hi) is the low word = the max corner.  Measured live: a
         * 640x512 GL window is programmed 0x1000100012801200 = min
         * (4096,4096), max (4736,4608), exactly the window in +4096 GL
         * window space; the clip is applied in window coordinates.
         *
         * clip_pass read min from scissor_hi and max from scissor_lo, so
         * every enabled scissor was empty (min 4736 >= max 4096) and
         * rejected every fragment.  Measured on atlantis: 10826 triangles,
         * inside>0, wrote=0 (WinOffset.dst=-4032, window on screen);
         * powerflip, solidview and mandel (which also set enScissorTest)
         * stayed at the clear colour for the same reason.  tex_cube never
         * sets enScissorTest, which is why it alone rendered.
         */
        uint32_t mn = s->scissor_lo, mx = s->scissor_hi;
        int minx = (mn >> 16) & 0xffff, miny = mn & 0xffff;
        int maxx = (mx >> 16) & 0xffff, maxy = mx & 0xffff;
        if (wx < minx || wx >= maxx || wy < miny || wy >= maxy) {
            return false;
        }
    }

    /*
     * Screen masks: framebuffer-relative rects, ScrMask[i] = [2i]=min,
     * [2i+1]=max (each a 64-bit register written as two 32-bit halves).
     *
     * @@SEMANTICS@@ — the enabled masks form a clip *list*, not an
     * intersection: crm.so programs a window's clip region as the union
     * of up to 5 rects. A 4Dwm window frame is exactly such a list — e.g.
     * the Icon Catalog frame is programmed as four disjoint border rects
     * (top bar, left, right, bottom). A pixel passes if it is inside ANY
     * enabled pass-inside mask (mode bit 1), and outside ALL enabled
     * pass-outside masks (mode bit 0). With no pass-inside mask enabled
     * the base region is the whole screen. The previous all-and
     * implementation rejected every multi-rect clip list; single-rect
     * lists (the common case) were unaffected because union == intersect
     * for one rect — except that the dropped high half made even those
     * read the wrong max. This is what dropped 4Dwm's window titles.
     */
    uint32_t en = (s->clipmode >> CM_ENSCRMASK_SHIFT) & 0x1f;
    bool any_in = false, in_union = false, out_union = false;
    for (int i = 0; i < 5; i++) {
        if (!(en & (1u << i))) {
            continue;
        }
        uint32_t mn = s->scrmask[2 * i], mx = s->scrmask[2 * i + 1];
        int minx = (mn >> 16) & 0xffff, miny = mn & 0xffff;
        int maxx = (mx >> 16) & 0xffff, maxy = mx & 0xffff;
        bool inside = x >= minx && x < maxx && y >= miny && y < maxy;
        /* scrMaskMode bit i: 1 = pass inside, 0 = pass outside */
        if ((s->clipmode >> i) & 1) {
            any_in = true;
            in_union |= inside;
        } else {
            out_union |= inside;
        }
    }
    if ((any_in && !in_union) || out_union) {
        return false;
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
    /*
     * Harden the rotate: strip_left + strip_right can reach 32 (e.g.
     * mode with maxIndex=0 -> strip_right=31 and index=1 ->
     * strip_left=1), which makes mid_bits 0 and the step's right shift
     * `>> (mid_bits - 1)` a shift by -1 — undefined behaviour. Clamp to
     * a 1-bit window, where the step degenerates to a pure left shift.
     */
    if (st->mid_bits < 1) {
        st->mid_bits = 1;
    }
}

static inline void crim_stipple_step(CrimStipple *st)
{
    st->pattern = (st->pattern << 1) |
                  (st->mid_bits > 1 ? (st->pattern >> (st->mid_bits - 1))
                                    : 0);
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

/*
 * Emit one fragment: shade -> clip -> stipple -> depth test -> ROP/mask
 * write.  depth_enable/z25 carry the triangle's depth-plane test; the
 * spec's pipeline orders the depth test after stipple and before the
 * colour write (§7.3.7.16 Fig 7-17 SZ pipeline).
 */
#define EMIT_CLIPPED  0
#define EMIT_WROTE    1
#define EMIT_STIPPLED 2       /* stipple masked the fragment out */
#define EMIT_DEPTH    3       /* depth test rejected the fragment */

static int sgi_crime_re_emit(SGICRIMEREState *s, int wx, int wy,
                             uint32_t color, CrimStipple *st,
                             bool line_stipple, bool depth_enable,
                             int64_t z25)
{
    int x, y;
    if (!sgi_crime_re_clip_pass(s, wx, wy, &x, &y)) {
        return EMIT_CLIPPED;
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
    int status = draw ? EMIT_WROTE : EMIT_STIPPLED;
    if (draw && depth_enable) {
        if (!sgi_crime_re_depth_test(s, x, y, crim_depth_value(z25))) {
            draw = false;
            status = EMIT_DEPTH;
        }
    }
    if (draw) {
        sgi_crime_re_put_pixel(s, s->bufmode_dst, x, y, color);
    }
    crim_stipple_step(st);
    return status;
}

/* ------------------------------------------------------------------ */
/* Texture look-up                                                      */
/* ------------------------------------------------------------------ */

/*
 * @@SEMANTICS@@ — CRIME texture texel look-up.  Spec §7.3.7.6 "Texel
 * Look-up" and §7.3.7.7 "Filtering" are TBD in CRIME 1.5, so the storage
 * map here was measured against the r4 golden; the texel tile map is
 * exactly:
 *
 *   i = (u>>5)*2048 + (v>>6)*8192 + ((v>>2)&3)*512 + ((v>>4)&3)*128
 *     + ((u&31)>>1)*8 + (v&3)*2 + (u&1)
 *
 * where i is a 32-bit texel WORD index.  tile = i>>14 selects the n-th
 * valid 64 KB TLB.tex tile in descriptor order (bit 15 = valid, bits
 * 14:0 = phys>>16), and the word sits at base + (i & 0x3fff)*4.  Each
 * word is big-endian [R,G,B,A].
 *
 * This is level 0 of the texture; the O2 GL window magnifies the 64x64
 * flowers2 texture (LOD 0), so mip-level selection does not arise here.
 * Measured exact (tmp/o2-qemu-emulation/2026-09-23-crime-texcoords/
 * REPORT.md §1a): the per-fragment coordinates below land on 0/31/63 at
 * the quad corners and reconstruct a perfect 64x64 flower.
 */
static bool sgi_crime_re_tex_lookup(SGICRIMEREState *s, int u, int v,
                                    uint32_t *out)
{
    uint32_t i;
    int tile, cnt = 0, tn;
    uint8_t b[4];

    if (u < 0 || v < 0) {
        return false;
    }
    i = (uint32_t)((u >> 5) * 2048) + (uint32_t)((v >> 6) * 8192)
      + (uint32_t)(((v >> 2) & 3) * 512) + (uint32_t)(((v >> 4) & 3) * 128)
      + (uint32_t)(((u & 31) >> 1) * 8) + (uint32_t)((v & 3) * 2)
      + (uint32_t)(u & 1);

    tile = (int)(i >> 14);
    for (tn = 0; tn < CRM_TLB_TEX_ENTRIES * 4; tn++) {
        uint16_t d = sgi_crime_re_tile_desc(s, 3, tn);
        if (!(d & 0x8000)) {
            continue;
        }
        if (cnt == tile) {
            hwaddr phys = ((hwaddr)(d & 0x7fff) << 16)
                        + (hwaddr)((i & 0x3fff) * 4);
            address_space_rw(&address_space_memory, phys,
                             MEMTXATTRS_UNSPECIFIED, b, 4, false);
            *out = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16)
                 | ((uint32_t)b[2] << 8) | b[3];
            return true;
        }
        cnt++;
    }
    return false;
}

/*
 * Evaluate the homogeneous texture-coordinate planes at pixel (px,py).
 * SQs/TQs are 36.12 and Qs 18.12 (spec Table 7-3); like the depth and
 * shade planes they are anchored at the FLOORED reference vertex:
 * __glCrmFillTriangle forms SQs/TQs/Qs at (floor(ref) - ref) offsets (the
 * same f20/f22 it uses for z0 and the shade planes).
 */
static inline void crim_tex_plane(SGICRIMEREState *s, int px, int py,
                                  int refx, int refy,
                                  int64_t *sq, int64_t *tq, int64_t *q)
{
    *sq = s->tex_sq0 + s->tex_dsqdx * (px - refx)
                    + s->tex_dsqdy * (py - refy);
    *tq = s->tex_tq0 + s->tex_dtqdx * (px - refx)
                    + s->tex_dtqdy * (py - refy);
    *q  = (int64_t)s->tex_q0 + (int64_t)s->tex_dqdx * (px - refx)
                             + (int64_t)s->tex_dqdy * (py - refy);
}

/*
 * Per-fragment texel of a textured GL fragment.  s = SQs/Qs, t = TQs/Qs,
 * and the texture coordinate is (s - 2^14)/2^8 (the +4096 rounding the
 * writer applies to each initial value, §7.3.7.1).  Clamp to the texture
 * width/height encoded in Texture.format bits [39:36]/[35:32].
 */
static bool sgi_crime_re_tex_sample(SGICRIMEREState *s, int px, int py,
                                    int refx, int refy, uint32_t *out)
{
    int64_t sq, tq, q;
    int uw = 1 << (int)((s->tex_format >> 36) & 0xf);
    int vh = 1 << (int)((s->tex_format >> 32) & 0xf);
    int u, v;

    if (uw <= 0 || vh <= 0) {
        return false;
    }
    crim_tex_plane(s, px, py, refx, refy, &sq, &tq, &q);
    if (q == 0) {
        return false;
    }
    /*
     * Normalise the perspective-divided coordinate.  (s - 2^14) spans 2^14
     * units per *full* texture map, so the texel index is
     * (s - 2^14) * size / 2^14.  The former hard-coded /256 is that only
     * when size == 64: on blast's 256x256 planet map it left u,v four times
     * too small, so the whole primitive sampled the top-left 1/16 of the
     * map (the near-black corner of nebula.rgb) and drew a black silhouette.
     */
    u = (int)((((sq / q) - 16384) * (int64_t)uw) / 16384);
    v = (int)((((tq / q) - 16384) * (int64_t)vh) / 16384);
    if (u < 0) {
        u = 0;
    }
    if (u >= uw) {
        u = uw - 1;
    }
    if (v < 0) {
        v = 0;
    }
    if (v >= vh) {
        v = vh - 1;
    }
    return sgi_crime_re_tex_lookup(s, u, v, out);
}

/*
 * TRI: intersection of three half-planes (spec §7.3.6). Edge functions
 * Ei(x,y) = Ai*x + Bi*y + Ci with Ai = dy, Bi = -dx (§7.3.5.2 EQ 2-4);
 * a point is inside iff Ei >= 0 for all i, with edges directed
 * clockwise around the interior (libGLcore orders vertices accordingly
 * and encodes the traversal direction in Primitive.edgeType).
 *
 * Vertices arrive as 13.6 fixed point; we evaluate edge functions at
 * pixel centers in 13.6 space (x*64+32) with 64-bit intermediates, and
 * step the x coefficient by A*64 per pixel (one pixel is 64 units).
 */
static void sgi_crime_re_draw_tri(SGICRIMEREState *s)
{
    /*
     * @@SEMANTICS@@ — the O2 IP32CRM32 libGLcore uploads a GL triangle vertex
     * as a 13.6 fixed-point coordinate in Vertex.GL[n].x/y, expressed against
     * the **+4096 window-space origin** (the same convention the PixelXfer
     * image path uses), with bit 23 set as a valid/format flag.  Live data:
     * a textured-cube vertex reads 0x00840380; masking bit 23 and shifting
     * the 13.6 value gives (0x040380 >> 6) = 4110 = 4096 + 14, and the
     * window's WinOffset.dst (-4064) maps it to framebuffer 32+14 — exactly
     * the window at +32.  (gr_osview's 0x48040c1f has the same structure:
     * & 0x7fffff = 0x040c1f -> 4096+48.)
     *
     * The previous code masked to the LOW 16 bits, which throws away the
     * +4096 origin (bit 18 / 0x40000): it produced window-local 14 instead of
     * 4110, so the WinOffset translation drove every fragment negative and
     * the screen-mask clip rejected the whole triangle (measured: inside>0
     * but wrote=0 for every tex_cube triangle).  Decode the 23-bit field;
     * the rasterizer below still evaluates edge functions at px*64+32 (13.6)
     * and emits integer px, and clip_pass applies WinOffset.dst.
     */
    int64_t vx[3], vy[3];
    for (int i = 0; i < 3; i++) {
        vx[i] = (int64_t)(int32_t)(s->vertex_gl[i][0] & 0x007fffff);
        vy[i] = (int64_t)(int32_t)(s->vertex_gl[i][1] & 0x007fffff);
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
     * Winding: the edge functions above are the NEGATIVE of the standard
     * cross-product edge function, so the interior is where Ei >= 0 only
     * for the winding that puts the opposite vertex on the positive side.
     * E_i(v[i+2]) == -area2, so the interior is the positive half-plane of
     * all three edges exactly when area2 <= 0.  Flip all signs only when
     * area2 > 0.  (The previous test flipped on area2 < 0, which inverted
     * the interior for the winding libGLcore actually emits: every GL
     * triangle evaluated to zero inside pixels — live libGLcore data for a
     * textured cube gave vertices v0=(49,29) v1=(29,29) v2=(29,49),
     * area2<0, and the flip rejected the whole triangle.  X-mode 2D fills
     * do not use this path, which is why the desktop never exposed it.)
     */
    int64_t area2 = (vx[1] - vx[0]) * (vy[2] - vy[0])
                  - (vx[2] - vx[0]) * (vy[1] - vy[0]);
    if (area2 > 0) {
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

    /* GL vertex space carries a +4096 origin, so the hard safety bound must
     * clear it (an X-mode triangle sits below 2k; a GL one at 4096..4176). */
    const int BOUND = 8192;
    if (minx < -BOUND) { minx = -BOUND; }
    if (miny < -BOUND) { miny = -BOUND; }
    if (maxx > BOUND) { maxx = BOUND; }
    if (maxy > BOUND) { maxy = BOUND; }

    bool smooth = (s->drawmode & DM_ENSMOOTHSHADE) != 0;
    if (s->drawmode & DM_ENTEXTURE) {
        trace_sgi_crime_re_tex_tri(s->tex_mode, s->tex_format,
                                   s->tex_sq0, s->tex_tq0, s->tex_q0,
                                   s->tex_dqdx, s->tex_dqdy,
                                   s->tex_stshift, s->winoffset_dst);
        trace_sgi_crime_re_tex_slope(s->tex_dsqdx, s->tex_dsqdy,
                                     s->tex_dtqdx, s->tex_dtqdy);
    }
    CrimStipple st;
    sgi_crime_re_stipple_init(s, &st);
    bool poly_stipple = (s->drawmode & DM_ENPOLYSTIPPLE) != 0;
    int inside_n = 0, wrote_n = 0;
    /*
     * Depth test (DrawMode.enDepthTest, spec §7.3.7.17 Table 7-25).  The
     * plane registers are 25.12 (§7.3.7.16); libGLcore programs z0 at the
     * floored *reference* vertex of the primitive — measured in
     * __glCrmFillTriangle: it clears the low 6 bits of the reference
     * vertex's x/y (f20/f22 = floor(ref) - ref) and forms
     * z0 = (z_ref + A*f20 + B*f22) * 4096.
     *
     * The reference is NOT always Vertex.GL[0].  __glCrmFillTriangle (and
     * __glCrmFlatZTriangle) pick the reference pointer t0 = a1 (vertex 0)
     * or a3 (vertex 2) with a movn/movz pair keyed on the sign of the first
     * vertex's x and a 5th "flag" argument, and the SAME choice is encoded
     * in the value they store to the Primitive register (RE + 0x2060):
     *   flag==0, x0>=0 -> prim 0x203, ref = v0     flag!=0, x0>=0 -> 0x200, ref = v2
     *   flag==0, x0<0  -> prim 0x201, ref = v2     flag!=0, x0<0  -> 0x202, ref = v0
     * i.e. ref = v0 exactly when (prim & 2) != 0 (disasm of __glCrmFlatTriangle
     * 0xda43d30 and __glCrmFlatZTriangle 0x0da43e1c; the stored prim value
     * reaches us as (t2 << 16)).  Assuming v0 unconditionally put a
     * per-triangle constant offset (dzdx*dvx + dzdy*dvy) on every triangle
     * whose reference is v2 — half of them here — so the depth test picked
     * the wrong triangle per pixel and the object broke up into a patchwork
     * of different triangles' Gouraud colours (the "rainbow streaks").
     */
    bool depth_enable = (s->drawmode & DM_ENDEPTHTEST) != 0;
    bool ref_v0 = (((s->primitive >> 16) & 0xff) & 0x2) != 0;
    int ref_idx = ref_v0 ? 0 : 2;
    int64_t refx = vx[ref_idx] >> 6, refy = vy[ref_idx] >> 6;
    int zfail_n = 0, zpass_n = 0;
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
                inside_n++;
                uint32_t color = s->shade_fgcolor;
                if (smooth) {
                    /*
                     * Gouraud: the shade planes are 9.12 two's-complement
                     * (spec §7.3.7.1 Table 16) and, exactly like Depth.Zs,
                     * are anchored at the FLOORED reference vertex:
                     * __glCrmFillTriangle forms R0 = refc + dRdx*f20 +
                     * dRdy*f22 with f20/f22 = floor(ref) - ref (the same
                     * fractional offsets it uses for z0).  So the value at
                     * pixel p is R0 + dRdx*(p - floor(ref)).  Evaluating
                     * from a fixed origin instead (the old px + WinOffset.dst)
                     * adds a per-triangle constant dRdx*(shx + floor(refx))
                     * — tens of pixels' worth of gradient on this scene —
                     * so the object's sub-pixel mesh shaded to a pastel
                     * rainbow with saturated white patches.  refx/refy are
                     * the same floored reference used by the depth plane.
                     */
                    int32_t r = (int32_t)s->shade_plane[0]
                              + (int32_t)s->shade_plane[4] * (px - refx)
                              + (int32_t)s->shade_plane[6] * (py - refy);
                    int32_t g = (int32_t)s->shade_plane[1]
                              + (int32_t)s->shade_plane[5] * (px - refx)
                              + (int32_t)s->shade_plane[7] * (py - refy);
                    int32_t b = (int32_t)s->shade_plane[2]
                              + (int32_t)s->shade_plane[8] * (px - refx)
                              + (int32_t)s->shade_plane[10] * (py - refy);
                    int32_t a = (int32_t)s->shade_plane[3]
                              + (int32_t)s->shade_plane[9] * (px - refx)
                              + (int32_t)s->shade_plane[11] * (py - refy);
                    color = ((uint32_t)crim_shade_clamp(r) << 24)
                          | ((uint32_t)crim_shade_clamp(g) << 16)
                          | ((uint32_t)crim_shade_clamp(b) << 8)
                          | (uint32_t)crim_shade_clamp(a);
                }
                if (s->drawmode & DM_ENTEXTURE) {
                    /*
                     * Textured fragment: the texel replaces the shaded
                     * colour.  Same floored reference vertex as the depth
                     * and shade planes.
                     */
                    uint32_t tex;
                    if (sgi_crime_re_tex_sample(s, px, py, refx, refy,
                                                &tex)) {
                        color = tex;
                    }
                }
                int64_t z25 = 0;
                if (depth_enable) {
                    z25 = s->depth_z0
                        + s->depth_dzdx * (px - refx)
                        + s->depth_dzdy * (py - refy);
                }
                int est = sgi_crime_re_emit(s, px, py, color, &st,
                                            poly_stipple, depth_enable, z25);
                if (est == EMIT_WROTE) {
                    wrote_n++;
                    if (depth_enable) {
                        zpass_n++;
                    }
                } else if (est == EMIT_DEPTH) {
                    zfail_n++;
                }
            }
            /*
             * Step in x by one PIXEL.  The sample point is in 13.6 fixed
             * point, so advancing to the next pixel adds 64 to x, and the
             * edge function E = A*x + B*y + C (A = dy, B = -dx, both 13.6)
             * therefore changes by A*64 — NOT by A.  Stepping by A made the
             * effective x-gradient 64x too shallow, so on the tiny (~1-3 px)
             * triangles of a GL scene the three half-planes almost never
             * intersected: measured live on `ideas`, 14421/17504 triangles
             * logged inside=0 though clearly non-degenerate, and the ones
             * that did fill drew stretched horizontal slivers.  (Large
             * triangles such as tex_cube's happened to fill their bbox, which
             * is why it looked like a flat white square.)
             */
            ex[0] += A[0] * 64; ex[1] += A[1] * 64; ex[2] += A[2] * 64;
        }
    }
    trace_sgi_crime_re_tri((int)(vx[0] >> 6), (int)(vy[0] >> 6),
                           (int)(vx[1] >> 6), (int)(vy[1] >> 6),
                           (int)(vx[2] >> 6), (int)(vy[2] >> 6),
                           inside_n, wrote_n,
                           s->winoffset_dst, s->clipmode);
    if (depth_enable) {
        trace_sgi_crime_re_depth_tri(s->drawmode, s->depth_mode,
                                     s->depth_z0, s->depth_dzdx,
                                     s->depth_dzdy, inside_n,
                                     zpass_n, zfail_n, wrote_n, s->primitive);
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
        /*
         * PixelXfer's linear source is addressed through the same 16-entry
         * (x2 paired page) 128 KB linear-TLB aperture the kernel programs in
         * irix-655/.../ml/MOOSEHEAD/mte_copy.c:289:
         *     index = (page & 0x1f) >> 1;
         *     CRIME_SET64(..., (1<<31 | page) << 32 | (1<<31 | page+1));
         * i.e. the low FIVE page bits select the slot and the entry itself
         * holds the physical page.  Xsgi's icon source is a virtual address
         * (0x105ff380 -> page 0x105ff, (0x105ff & 0x1f)>>1 = 15) and it
         * programs exactly entry 15; the un-masked `page >> 1` rejected every
         * address past the first 128 KB, so xfer_fetch() returned 0 and the
         * toolchest menu icons were solid black.  Scoped to this path (not
         * sgi_crime_re_linear_addr): the MTE linear path drives real physical
         * addresses and its current mapping is load-bearing for the PROM menu.
         */
        int page = (int)((uint32_t)src_off >> 12);
        int entry = (page & 0x1f) >> 1;
        int sub = page & 1;
        int lin = (s->bufmode_src >> BM_BUF_TYPE_SHIFT) & 1;
        uint32_t d = sub ? (uint32_t)s->tlb_linear[lin][entry]
                         : (uint32_t)(s->tlb_linear[lin][entry] >> 32);
        if (!(d & 0x80000000u)) {
            return 0;
        }
        phys = ((hwaddr)(d & 0x7fffffffu) << 12) + ((uint32_t)src_off & 0xfff);
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
        return b[crim_ci_lane(s->bufmode_src)];
    }
    if (pix_type == 1) {
        /* RGB src is ABGR in memory (unused byte0); return canonical RGBA. */
        return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16)
             | ((uint32_t)b[1] << 8);
    }
    if (pix_type == 2 || pix_type == 3) {
        /*
         * @@SEMANTICS@@ — the O2 normal-plane 32bpp surfaces are ABGR in
         * memory: bytes A,B,G,R.  This holds for pixType 2 (RGBA) and 3
         * (ABGR) alike — the earlier note that pixType 2 was "memory
         * R,G,B,A" was wrong and is the defect this fixes: the clogin
         * pattern pixmap was read back un-reordered, so the canonical R
         * came from the alpha byte (always 0) and the panel scanned out
         * cyan.  Return the canonical internal RGBA (the convention
         * get_pixel/put_pixel and the shade/dither arithmetic use:
         * R=31:24..A=7:0); put_pixel performs the reverse.
         */
        return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16)
             | ((uint32_t)b[1] << 8) | b[0];
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
                            s->shade_fgcolor, s->stipple_pattern,
                            s->stipple_mode);

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
         * @@SEMANTICS@@ — OpenGL pixel transfer (glDrawPixels /
         * glCopyPixels; DrawMode.enGL=1 + enPixelXfer=1).  The spec ties
         * the coordinate register set to the mode: §7.3.1.6 DrawMode.enGL
         * selects "GL mode" vs "X mode", and §7.3.6.1 says a rectangle's
         * window coordinates come from Rasterize.vertexX for X primitives
         * *or* Rasterize.vertexGL for OpenGL primitives.  libGLcore's
         * video upload programs a 640x480 window as ~10 horizontal bands
         * in Vertex.GL as 13.6 values against a +4096 window-space origin
         * (x 4096..4735, y 4096..4575) with WinOffset.dst = -3846, which
         * compose (fb = window + offset) to the on-screen window
         * 250..889 x 250..729 — the exact rectangle of the GBE DID window.
         *
         * The engine must walk this rect in the GL vertex space (no
         * X-mode 2048-pixel modular wrap: a GL coord of 4096 masked to
         * 0x7ff becomes 0, and the whole transfer then translated to
         * fb -3846..-2567, which the screen-mask clip rejected — leaving
         * the video window at the MTE clear colour).  We therefore take a
         * dedicated bounded walk with the source row pitch derived from
         * the GL extent.  X-mode PixelXfer (the toolchest-icon upload,
         * enGL=0) keeps the existing modular path byte-for-byte, and a
         * (not yet observed) tiled GL source falls back to that path
         * rather than being misread as a linear walk.
         */
        if (xfer && (dm & DM_ENGL) && src_linear) {
            int64_t gx1 = (int64_t)((int32_t)s->vertex_gl[0][0] >> 6);
            int64_t gy1 = (int64_t)((int32_t)s->vertex_gl[0][1] >> 6);
            int64_t gx2 = (int64_t)((int32_t)s->vertex_gl[1][0] >> 6);
            int64_t gy2 = (int64_t)((int32_t)s->vertex_gl[1][1] >> 6);
            if (gx2 < gx1) { int64_t t = gx1; gx1 = gx2; gx2 = t; }
            if (gy2 < gy1) { int64_t t = gy1; gy1 = gy2; gy2 = t; }
            trace_sgi_crime_re_xfer(s->bufmode_src, s->bufmode_dst,
                                    s->pixelxfer_src_addr,
                                    s->pixelxfer_src_xstep,
                                    s->pixelxfer_src_ystep,
                                    s->clipmode, s->winoffset_dst, dm,
                                    (int)gx1, (int)gy1);
            int64_t gl_xstep = (int32_t)s->pixelxfer_src_xstep;
            int64_t gl_pitch = (gx2 - gx1) * gl_xstep
                             + (int32_t)s->pixelxfer_src_ystep;
            int64_t gl_row = src_off;
            for (int64_t gy = gy1; gy <= gy2; gy++) {
                int64_t gl_col = gl_row;
                for (int64_t gx = gx1; gx <= gx2; gx++) {
                    int fx, fy;
                    if (sgi_crime_re_clip_pass(s, (int)gx, (int)gy,
                                               &fx, &fy)) {
                        uint32_t color = sgi_crime_re_xfer_fetch(s, gl_col,
                                                                 src_linear);
                        sgi_crime_re_put_pixel(s, s->bufmode_dst, fx, fy,
                                               color);
                    }
                    gl_col += gl_xstep;
                }
                gl_row += gl_pitch;
            }
            break;
        }
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

        /*
         * PixelXfer source row stride (linear sources only).
         *
         * @@SEMANTICS@@ — the spec gives only the field names ("x-direction
         * step size" / "y-direction step size", §7.3.1.3).  Derived from a
         * real toolchest menu-icon transfer: dest RECT (85,41)-(102,56)
         * (w=18), src.addr=0x105ff380, xStep=1, yStep=3, src BufMode linear
         * 8-bit CI.  Dumping the mapped source page showed the 18x16 bitmap
         * has a 20-byte row pitch, and 20 is the ONLY candidate that reads a
         * clean palette over the whole glyph (stride 20: 288/288 valid
         * indices; 18: 260, 21: 247).  So the engine advances by xStep
         * between pixels WITHIN a row and by yStep from a row's LAST pixel to
         * the next row's FIRST: pitch = (w-1)*xStep + yStep = 17*1 + 3 = 20.
         * (gxemul does not model a y-step stride at all — it resets the
         * source per row — which is why the icons stayed single-row.)
         */
        int xfer_w = (dx > 0)
                   ? (int)((endx + MOD - startx) & (MOD - 1))
                   : (int)((startx + MOD - endx) & (MOD - 1));
        int64_t xfer_row = src_off;
        int64_t xfer_pitch = (src_linear && xfer_w > 0)
                           ? (int64_t)(xfer_w - 1) * xstep
                             + (int32_t)s->pixelxfer_src_ystep
                           : 0;

        /*
         * Tiled (framebuffer) source: PixelXfer.src.addr packs the source
         * pixel (x in bits 31:16, y in bits 15:0) and the engine walks it in
         * LOCKSTEP with the destination.  The spec's edgeType (§7.3.1.7
         * Table 7-8) gives the shared traversal direction, so x advances by
         * the primitive's dx between pixels (resetting at each row) and y
         * advances by dy between rows.  gxemul does exactly this in its
         * DE_PRIM_RECTANGLE walk (saved_src_x per row, src_x += dx,
         * src_y += dy).  Evidence: the 6 granite backdrop ops carry
         * src=(255,0) with dx=-1 (tile 0 copied right-to-left) and
         * src=(0,255) with dy=-1 (band copied bottom-to-top); treating the
         * packed address as a fixed (x,y) and stepping it by xStep only
         * smeared one source row down the whole rect (the tear).
         * @@SEMANTICS@@
         */
        int src_x = (int)(((uint32_t)src_off >> 16) & 0x7ff);
        int src_y = (int)((uint32_t)src_off & 0x7ff);

        if (xfer) {
            trace_sgi_crime_re_xfer(s->bufmode_src, s->bufmode_dst,
                                    (uint32_t)src_off,
                                    s->pixelxfer_src_xstep,
                                    s->pixelxfer_src_ystep,
                                    s->clipmode, s->winoffset_dst, dm,
                                    (int)sx1, (int)sy1);
        }

        int ity = 0;
        for (uint32_t yy = starty; yy != endy && ity < MAX_ITER;
             yy = (yy + dy) & (MOD - 1), ity++) {
            int64_t row_src = src_linear ? xfer_row : src_off;
            int sx = src_x;
            int itx = 0;
            for (uint32_t xx = startx; xx != endx && itx < MAX_ITER;
                 xx = (xx + dx) & (MOD - 1), itx++) {
                uint32_t color = s->shade_fgcolor;
                if (xfer) {
                    int64_t fetch_off = src_linear ? row_src
                                      : (((int64_t)sx << 16) | (src_y & 0x7ff));
                    color = sgi_crime_re_xfer_fetch(s, fetch_off, src_linear);
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
                if (src_linear) {
                    row_src += xstep;
                } else {
                    sx = (sx + dx) & (MOD - 1);
                }
            }
            if (src_linear) {
                xfer_row += xfer_pitch;
            } else {
                src_y = (src_y + dy) & (MOD - 1);
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

    trace_sgi_crime_re_mte(mode, s->mte_dst0, s->mte_dst1, fg,
                           s->mte_stipplemask, s->mte_bytemask);

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

    /*
     * M11k: the MTE dst/src x fields hold BYTE coordinates wider than
     * 12 bits — the X DDX's 8bpp full-screen fill reaches byte-x 0x13ff
     * (5119 = a full 1280-word row's bytes). The old 0xfff mask truncated
     * every fill past byte 4095 (observed live as the 256-px red strip on
     * the left of the v3 gate screen). Take the full 16-bit field; the
     * 0x1000+ bits are byte-x, not flags (the byte-lane-rotating weave
     * fills use exactly 0x13ff_xxxx for their rows).
     */
    int x1 = (s->mte_dst0 >> 16) & 0xffff;
    int y1 = s->mte_dst0 & 0xfff;
    int x2 = (s->mte_dst1 >> 16) & 0xffff;
    int y2 = (s->mte_dst1 & 0xfff);
    /*
     * dst0/dst1 x fields are BYTE addresses into the destination buffer,
     * so the byte-x -> pixel-x stride is the destination BufMode's buffer
     * word depth (bufDepth), not MTE.mode.pixDepth.  The guest sends
     * pixDepth=0 for its 32-bit fills (e.g. the Icon Catalog panel), which
     * only BufMode.dst records correctly.  This is placement-invariant
     * with the tiled address walk (which scales byte-x by bpp), but it is
     * what makes the synthesized BufMode below carry the true word depth
     * so the pixel store lands at the right width.
     */
    int depth_code = (s->bufmode_dst >> BM_BUF_DEPTH_SHIFT) & 3;
    int bpp = 1 << depth_code;
    x1 /= bpp; x2 /= bpp;

    /*
     * Destination pixel FORMAT comes from BufMode.dst, not from MTE.mode:
     * the spec's MTE.mode register (§7.3.2.1) carries only opCode, enStipple,
     * pixDepth and the src/dst buffer-TYPE (TLB select) — there is no pixType
     * field.  The previous code forced RGB for every bpp>1 transfer, so an
     * MTE CLEAR at 32-bit depth into a COLOR_INDEX destination packed the
     * 8-bit colour index as big-endian RGB (b[0]=color>>24=0, the index in
     * b[3]) and left byte 0 — the byte get_pixel()/the GBE read — at 0, i.e.
     * black.
     *
     * @@SEMANTICS@@ Derived, not guessed.  Decisive internal consistency:
     * the SAME fg index 0x29 paints (102,102,102) through an 8-bit-depth MTE
     * CLEAR (xterm body) but black through a 32-bit-depth one (xterm text
     * row) — same index, same CMAP, two depths.  A COLOR_INDEX destination
     * must map both through the CMAP, so the 32-bit op must write the index.
     * Corroboration: Xsgi programs BufMode.dst=0x00000200 (pixType=0 CI,
     * bufDepth=32) and the DRAW path, which already uses s->bufmode_dst,
     * renders the same 0x29 index correctly.  This is why the toolchest
     * "Toolchest" title (bg index 0x0f + fg 0), the xterm white text
     * (fg 0x07) and the xterm text-row background (fg 0x29) were all black.
     *
     * Both the word depth and the pixel depth come from BufMode.dst: the
     * former addresses the buffer, the latter (BM_PIX_DEPTH, 8/16/32-bit)
     * sizes the CI store so a packed 16-bit RGB5 destination is written in
     * full instead of being truncated to its low byte.
     */
    uint32_t bufmode = (dst_tlb << BM_BUF_TYPE_SHIFT)
                     | (depth_code << BM_BUF_DEPTH_SHIFT)
                     | (s->bufmode_dst & (BM_PIX_TYPE_MASK |
                                          BM_PIX_DEPTH_MASK))
                     /*
                      * Keep doublePix/doublePixSel so a 16+16 double-
                      * buffered clear lands in the same word half as the
                      * GL draws (and as the GBE WID's buf selects).  The
                      * old mask dropped them, so the GL clear always hit
                      * the lower half and the scanned (upper) buffer came
                      * up stale.
                      */
                     | (s->bufmode_dst & 0x3u);
    int dx = x1 > x2 ? -1 : 1;
    int dy = y1 > y2 ? -1 : 1;

    /*
     * byteMask gates per-byte writes (kernel copies use 0xffffffff;
     * partial-byte tail masks land here).
     */
    bool use_bytemask = (s->mte_bytemask != 0xffffffffu);

    if (!is_copy) {
        /*
         * @@SEMANTICS@@ — MTE CLEAR into the depth buffer.  TLB C
         * (CRM_DEPTH_TLB_SEL) is the SZ/depth buffer and its pixel is a
         * 32-bit big-endian word (byte0 = stencil, bytes1..3 = 24-bit Z;
         * spec §7.3.4.4 Fig 7-6).  The MTE fill value is that whole SZ
         * word, NOT a colour: routing a depth clear through the *colour*
         * BufMode packs fgValue into the destination pixel format, so a
         * GL zclear() to far (fg=0x00ffffff) was stored as a 16-bit
         * A1_RGB5 (0x83ff) — the depth buffer came up near instead of
         * far, and since depth func LEQUAL compares z <= zd, every
         * depth-tested fragment was rejected.  Measured on demograph:
         * every map fragment died at the depth test (clipped=0 depth=N),
         * the window stayed at the clear colour, while only the
         * un-depth-tested background triangles drew.  The clear must land
         * as the raw 32-bit SZ value for any MTE fill targetting the
         * depth TLB.  (A fill of fg=0, e.g. powerflip's, is unchanged.)
         */
        if (dst_tlb == CRM_DEPTH_TLB_SEL) {
            uint8_t w[4];
            w[0] = (fg >> 24) & 0xff;      /* stencil   */
            w[1] = (fg >> 16) & 0xff;      /* Z[23:16]  */
            w[2] = (fg >> 8) & 0xff;       /* Z[15:8]   */
            w[3] = fg & 0xff;              /* Z[7:0]    */
            for (int y = y1; y != y2 + dy; y += dy) {
                for (int x = x1; x != x2 + dx; x += dx) {
                    hwaddr phys;
                    if (!sgi_crime_re_tiled_addr(s, CRM_DEPTH_TLB_SEL,
                                                 x, y, 4, &phys)) {
                        continue;
                    }
                    address_space_rw(&address_space_memory, phys,
                                     MEMTXATTRS_UNSPECIFIED, w, 4, true);
                }
            }
            return;
        }
        /*
         * Tiled fill: same walk as M7 (mte_zero / PROM textport), but
         * honor MTE.enStipple — the spec (CRIME 1.5 §7.3.2.1) calls bit
         * 10 "enStipple: enable/disable stipple pixel mask application
         * during clear operations" and MTE.stippleMask (0x10) a
         * "32-bit stipple mask".
         *
         * This is the FONT-GLYPH path: Xsgi's DDX renders 1-bit glyph
         * bitmaps by programming the glyph's bitmap rows into
         * MTE.stippleMask and issuing an enStipple CLEAR per cell row
         * (pixDepth=2/32-bit, fgValue=the text colour). Ignoring the
         * mask filled the whole cell rectangle → solid bars.
         *
         * Mask semantics (@@SEMANTICS@@ — the CRIME 1.5 spec gives no
         * stipple-to-pixel algorithm, so this is derived from the trace):
         * the 32-bit mask repeats every 32 pixels, one bit per pixel,
         * MSB-first, anchored to the destination 32-bit word.  The run's
         * starting phase is therefore `x1 mod (32 / bpp)` and the bit for
         * pixel x is `31 - ((phase + x - x1) & 31)` (see the loop below
         * for the derivation and the oracle-scored evidence).
         */
        bool en_stipple = (mode & MTE_EN_STIPPLE) != 0;
        uint32_t mask = s->mte_stipplemask;
        for (int y = y1; y != y2 + dy; y += dy) {
            /*
             * @@SEMANTICS@@ — the 32-bit mask is anchored to the
             * destination 32-bit WORD (4 bytes), so the run's phase is the
             * start pixel's byte offset within the word, i.e.
             * x1 mod (32 / bpp) pixels.  Then one bit per pixel MSB-first,
             * wrapping mod 32 across the run.  The previous revision
             * hard-coded 8 (x1 & 7), which is the right phase only at
             * bpp=4: 8bpp glyphs (popup menus) run 32 px per mask repeat,
             * so the phase there must be x1 & 31 and the old rule read the
             * pattern shifted by (x1 & 24) bits, garbling the Toolchest
             * "Desktop" popup's upper labels.  The phase is genuinely
             * DEPTH-DEPENDENT: applying x1 & 31 unconditionally (an
             * intermediate revision) fixed the popup but garbled the
             * 32bpp Toolchest main-window / Icon Catalog text, so derive
             * it from bpp rather than hard-coding either constant.
             * Derived from the trace:
             * scoring the popup glyphs against the Indy oracle bitmap
             * (toolchest/01_toolchest__menu-desktop__open.png,
             * x=115..205, y=45..99) gives `x1 & 31` 0 false positives /
             * 184 misses vs the old rule's 165 / 780 (the misses are
             * oracle pixels whose MTE was outside the traced window); the
             * 32bpp Toolchest main-window labels still render correctly
             * with `x1 & 7`, so the phase is depth-dependent.
             */
            int bit = x1 & ((32 / bpp) - 1);
            for (int x = x1; x != x2 + dx; x += dx, bit++) {
                if (en_stipple &&
                    !((mask >> (31 - (bit & 31))) & 1)) {
                    continue;
                }
                sgi_crime_re_put_pixel_mte(s, bufmode, x, y, fg);
            }
        }
        return;
    }

    /* COPY: src rect in the src TLB (tiled or linear) + y-step strides */
    bool src_linear = src_tlb == 4 || src_tlb == 5;
    int sx1 = (s->mte_src0 >> 16) & 0xffff;
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
                    /* CI byte lane must match put_pixel's store lane. */
                    color = sv[crim_ci_lane(bufmode)];
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
    /* latch every pixel-pipe write for read-back (crmSavePP) */
    if (p < CRM_RE_PIXPIPE_SIZE) {
        s->pp_shadow[p >> 2] = v;
    }

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

    /* ---- screen masks: 5 x 64-bit, each two 32-bit stores (min,max) ---- */
    case CRM_SCRMASK0_REG + 0:  case CRM_SCRMASK0_REG + 4:
    case CRM_SCRMASK0_REG + 8:  case CRM_SCRMASK0_REG + 12:
    case CRM_SCRMASK0_REG + 16: case CRM_SCRMASK0_REG + 20:
    case CRM_SCRMASK0_REG + 24: case CRM_SCRMASK0_REG + 28:
    case CRM_SCRMASK0_REG + 32: case CRM_SCRMASK0_REG + 36:
        s->scrmask[(p - CRM_SCRMASK0_REG) / 4] = v;
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

    /* ---- texture ---- */
    case CRM_TEXTURE_MODE_REG:
        s->tex_mode = v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_FORMAT_REG:
        s->tex_format = (s->tex_format & 0xffffffffu) | ((uint64_t)v << 32);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_FORMAT_REG + 4:
        s->tex_format = (s->tex_format & ~0xffffffffULL) | v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    /* 64-bit coordinate planes: high word at the even offset, low at +4 */
    case CRM_TEXTURE_SQ0_REG:
        s->tex_sq0 = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_sq0);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_SQ0_REG + 4:
        s->tex_sq0 = (int64_t)(((uint64_t)s->tex_sq0 & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_TQ0_REG:
        s->tex_tq0 = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_tq0);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_TQ0_REG + 4:
        s->tex_tq0 = (int64_t)(((uint64_t)s->tex_tq0 & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_Q0_REG:
        s->tex_q0 = (int32_t)v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_STSHIFT_REG:
        s->tex_stshift = (int32_t)v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DSQDX_REG:
        s->tex_dsqdx = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_dsqdx);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DSQDX_REG + 4:
        s->tex_dsqdx = (int64_t)(((uint64_t)s->tex_dsqdx & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DSQDY_REG:
        s->tex_dsqdy = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_dsqdy);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DSQDY_REG + 4:
        s->tex_dsqdy = (int64_t)(((uint64_t)s->tex_dsqdy & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DTQDX_REG:
        s->tex_dtqdx = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_dtqdx);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DTQDX_REG + 4:
        s->tex_dtqdx = (int64_t)(((uint64_t)s->tex_dtqdx & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DTQDY_REG:
        s->tex_dtqdy = (int64_t)(((uint64_t)v << 32) | (uint32_t)s->tex_dtqdy);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DTQDY_REG + 4:
        s->tex_dtqdy = (int64_t)(((uint64_t)s->tex_dtqdy & ~0xffffffffULL) | v);
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DQDX_REG:
        s->tex_dqdx = (int32_t)v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_DQDY_REG:
        s->tex_dqdy = (int32_t)v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_BORDER_REG:
        s->tex_border = v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;
    case CRM_TEXTURE_ENV_REG:
        s->tex_env = v;
        trace_sgi_crime_re_texreg((int)p, v);
        return;

    /* ---- depth / stencil (spec §7.3.1.18) ---- */
    case CRM_DEPTH_FUNC_REG:
        s->depth_mode = v;
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    /* 64-bit 25.12 planes: high word at the even offset, low at +4 */
    case CRM_DEPTH_Z0_REG:
        s->depth_z0 = (int64_t)(((uint64_t)v << 32) |
                                (uint32_t)s->depth_z0);
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    case CRM_DEPTH_Z0_REG + 4:
        s->depth_z0 = (int64_t)(((uint64_t)s->depth_z0 & ~0xffffffffULL) | v);
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    case CRM_DEPTH_DZDX_REG:
        s->depth_dzdx = (int64_t)(((uint64_t)v << 32) |
                                  (uint32_t)s->depth_dzdx);
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    case CRM_DEPTH_DZDX_REG + 4:
        s->depth_dzdx = (int64_t)(((uint64_t)s->depth_dzdx & ~0xffffffffULL)
                                  | v);
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    case CRM_DEPTH_DZDY_REG:
        s->depth_dzdy = (int64_t)(((uint64_t)v << 32) |
                                  (uint32_t)s->depth_dzdy);
        trace_sgi_crime_re_depthreg((int)p, v);
        return;
    case CRM_DEPTH_DZDY_REG + 4:
        s->depth_dzdy = (int64_t)(((uint64_t)s->depth_dzdy & ~0xffffffffULL)
                                  | v);
        trace_sgi_crime_re_depthreg((int)p, v);
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
    /* latch every MTE write for read-back (crmSavePP saves MTE too) */
    if (m < CRM_RE_MTE_SIZE) {
        s->mte_shadow[m >> 2] = v;
    }

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

static uint64_t sgi_crime_re_tlb_read(SGICRIMEREState *s, hwaddr off)
{
    hwaddr t = off - CRM_RE_TLB_BASE;

    if (t < 0x200) {
        return s->tlb_fb[0][t / 8];
    } else if (t < 0x400) {
        return s->tlb_fb[1][(t - 0x200) / 8];
    } else if (t < 0x600) {
        return s->tlb_fb[2][(t - 0x400) / 8];
    } else if (t < CRM_TLB_CID_OFFSET) {
        return s->tlb_tex[(t - CRM_TLB_TEX_OFFSET) / 8];
    } else if (t < CRM_TLB_LINEAR_A_OFFSET) {
        return s->tlb_cid[(t - CRM_TLB_CID_OFFSET) / 8];
    } else if (t < CRM_TLB_LINEAR_A_OFFSET + 0x80) {
        return s->tlb_linear[0][(t - CRM_TLB_LINEAR_A_OFFSET) / 8];
    } else if (t < CRM_TLB_LINEAR_B_OFFSET + 0x80) {
        return s->tlb_linear[1][(t - CRM_TLB_LINEAR_B_OFFSET) / 8];
    }
    return 0;
}

/*
 * Interface-buffer occupancy (status register intfBufLevel, bits 24:18).
 *
 * CRIME 1.5 spec §7.3.3.1: the host "can avoid overflows by examining the
 * interface buffer FIFO level indicated in the intfBufLevel field Status
 * register" — i.e. the level is the number of posted entries the rendering
 * engine has not yet retired, (WrPtr - RdPtr) mod 64.  We give the RE an
 * independent RdPtr and advance it as each posted write is applied to the
 * register file; because the engine in this model retires a write on the
 * host MMIO store that posted it (never back-pressured), RdPtr tracks WrPtr
 * and the level reads 0, below every CRMIBCTL_* watermark — so the RE3/RE4
 * FIFO interrupts never fire.  That is the truthful level for a
 * synchronous engine, not a hardcoded constant.
 */
static uint32_t sgi_crime_re_ib_level(SGICRIMEREState *s)
{
    return (s->ib_wrptr - s->ib_rdptr) & CRMSTAT_IB_LEVEL_MASK;
}

static uint64_t sgi_crime_re_read_impl(void *opaque, hwaddr offset, unsigned size)
{
    SGICRIMEREState *s = SGI_CRIME_RE(opaque);

    switch (offset) {
    case CRM_RE_STATUS_REG: {
        /*
         * Compose the status word: all idle (crmWaitReIdle spins bits
         * 27|25; libGLcore FlushAndConfirm spins bit 28 — both must
         * see idle immediately), the live ring pointers, and the IB
         * level = WrPtr - RdPtr.
         *
         * crmSavePP() harvests pending = WrPtr - StartPtr (mod 64)
         * interface-buffer addr/data slots as the switching-out
         * context's pixel-pipe descriptor list, so every slot in
         * [StartPtr, WrPtr) must hold a real descriptor posted by an RE
         * register write (a zero slot decodes to wmask 0 and panics
         * "pcxswap wmask 0").  Our engine retires synchronously, so
         * RdPtr tracks WrPtr and the level reads 0 — crmWaitReFifo
         * never spins — while the entries remain in the RAM until
         * overwritten.
         *
         * NOTE: do NOT derive the start pointer from ib_ctl — bits 5:0
         * of the interface-buffer ctl register are the stall-count field
         * (crimeInit programs full/empty/stall thresholds, e.g. the
         * 0x0fefff0a written to 0x400), not the FIFO start pointer.
         */
        uint32_t st = CRMSTAT_ALL_IDLE;
        uint32_t wr = s->ib_wrptr & CRMSTAT_IB_WRPTR_MASK;
        uint32_t rd = s->ib_rdptr & CRMSTAT_IB_RDPTR_MASK;
        uint32_t stptr = s->ib_startptr & CRMSTAT_IB_STPTR_MASK;
        uint32_t level = sgi_crime_re_ib_level(s);

        st |= level << CRMSTAT_IB_LEVEL_SHIFT;
        st |= rd << CRMSTAT_IB_RDPTR_SHIFT;
        st |= wr << CRMSTAT_IB_WRPTR_SHIFT;
        st |= stptr << CRMSTAT_IB_STPTR_SHIFT;
        return st;
    }

    case CRM_RE_INTFBUF_CTL:
        return s->ib_ctl;

    case CRM_RE_INTFBUF_DATA ... CRM_RE_INTFBUF_DATA + 0x1f8:
        /*
         * crmSavePP() reads each data slot as two 32-bit words (lw at +0
         * and +4) — the 64-bit value is big-endian, so the lower address
         * is the high word.  Returning 0 here saved a zero data payload
         * for every harvested descriptor and crmRestorePP() replayed
         * zeros, which is how the earlier unequal-pointer attempt
         * damaged the switching-out context.
         */
        if (size == 8) {
            return s->ib_data[(offset - CRM_RE_INTFBUF_DATA) / 8];
        }
        if ((offset & 4) == 0) {
            return (uint32_t)(s->ib_data[(offset - CRM_RE_INTFBUF_DATA) / 8]
                              >> 32);
        }
        return (uint32_t)s->ib_data[(offset - CRM_RE_INTFBUF_DATA) / 8];

    case CRM_RE_INTFBUF_ADDR ... CRM_RE_INTFBUF_ADDR + 0x1f8:
        if (size == 8) {
            return s->ib_addr[(offset - CRM_RE_INTFBUF_ADDR) / 8];
        }
        if ((offset & 4) == 0) {
            return (uint32_t)(s->ib_addr[(offset - CRM_RE_INTFBUF_ADDR) / 8]
                              >> 32);
        }
        return (uint32_t)s->ib_addr[(offset - CRM_RE_INTFBUF_ADDR) / 8];

    default:
        /*
         * crmGetRERev reads 0x0400 (IntfBuf.ctl) and takes bit 0 to
         * decide RE revision; 0 = rev B ("SGI-CRM, Rev B"). ib_ctl bit
         * 0 stays 0 unless programmed — keep the read faithful.
         */
        if (offset == CRM_RE_INTFBUF_CTL) {
            return s->ib_ctl;
        }

        /*
         * Read-back of the latched register pages.  crmSavePP() reads
         * the pixel-pipe and MTE registers to save the switching-out
         * context; returning 0 made every saved shadow all-zero, so
         * the matching crmRestorePP() loaded zeros and the resumed
         * context silently lost its draws.  Return the last write.
         */
        if (offset >= CRM_RE_TLB_BASE && offset < CRM_RE_PIXPIPE_BASE) {
            return sgi_crime_re_tlb_read(s, offset);
        }
        if (offset >= CRM_RE_PIXPIPE_BASE && offset < CRM_RE_MTE_BASE) {
            hwaddr p = (offset & ~(hwaddr)CRM_GO_OFFSET) - CRM_RE_PIXPIPE_BASE;

            if (p + 4 <= CRM_RE_PIXPIPE_SIZE) {
                uint64_t lo = s->pp_shadow[p >> 2];

                if (size == 8 && p + 8 <= CRM_RE_PIXPIPE_SIZE) {
                    return (lo << 32) | s->pp_shadow[(p >> 2) + 1];
                }
                return lo;
            }
            return 0;
        }
        if (offset >= CRM_RE_MTE_BASE && offset < CRM_RE_STATUS_BASE) {
            hwaddr m = (offset & ~(hwaddr)CRM_GO_OFFSET) - CRM_RE_MTE_BASE;

            if (m + 4 <= CRM_RE_MTE_SIZE) {
                uint64_t lo = s->mte_shadow[m >> 2];

                if (size == 8 && m + 8 <= CRM_RE_MTE_SIZE) {
                    return (lo << 32) | s->mte_shadow[(m >> 2) + 1];
                }
                return lo;
            }
            return 0;
        }
        return 0;
    }
}

/*
 * Post a host write to an RE register page into the interface-buffer
 * ring, exactly as the CRIME RE does for every write to the TLB,
 * pixel-pipe and MTE pages (spec §7.3.3.1).  The 64-bit descriptor is
 * CrmIntfBufAddr (sys/crimereg.h):
 *
 *   bits 39:32  offset   register byte offset within its 4 KB page, >> 3
 *   bits 42:40  pageId   TLB = 1, PixelPipe = 2, MTE = 3
 *   bits 44:43  wmask    write mask: 1 = high 32-bit word (target + 4),
 *                        2 = low 32-bit word (target), 3 = full 64-bit
 *   bit  45     start    set on a START_OFFSET (+0x800) commit write
 *
 * The field positions are the `#else` macros in crimereg.h
 * (CRMIBADDR_OFFSET/PAGE/WMASK/START and CRMIBADDR_TO_PHYS), and each is
 * confirmed by the IRIX kernel's own decoder: crmRestorePP() at VA
 * 0x8028a1e4..0x8028a238 extracts bits 44:43/42:40/39:32 and
 * crmFindPP() at VA 0x80299f6c tests bit 45 as the batch-start flag.
 *
 * The data slot holds the write value in the word the wmask selects:
 * crmRestorePP() reads (wmask 2) the low word, (wmask 1) the high word,
 * or (wmask 3) the whole 64 bits and stores it at the decoded target.
 *
 * Context save/restore is driven by the guest kernel (gf_PcxSwap ->
 * crmSavePP -> crmLoadTlbABC -> crmRestorePP), which harvests
 * pending = (WrPtr - StartPtr) mod 64 descriptors and replays them.  The
 * replay honours the per-context copy mask CRM_CXSW_* (crimedef.h):
 *   LSTIPPLE 0x01, MTE 0x04, TEXTURE 0x08, SHADE 0x10, SZ 0x20, MISC 0x40,
 *   CRM_CXSW_ALL = 0x7d, and CRM_CXSW_NOT_GL 0x80.
 * crmRestorePP()'s disassembly gates each block on those bits (LSTIPPLE at
 * 0x802698dc, MTE at 0x802698f8, SHADE at 0x802699d0, SZ at 0x802699a8,
 * MISC at 0x80269978, TEXTURE at 0x802699d4); for an X context (NOT_GL) it
 * deliberately does NOT round-trip Vertex.GL, Shade, Scissor, Fog,
 * Antialias, Blend, Depth, AlphaTest, Stencil or Texture, keeping only
 * Vertex.X and Shade.fgColor (crimedef.h comment; spec Table 7-3 marks the
 * per-register G/NG class).  The device honours the mask by storing every
 * write and returning the last write on read-back, so whichever subset the
 * kernel chooses to save/restore round-trips exactly.
 */
static void sgi_crime_re_ib_post(SGICRIMEREState *s, unsigned page,
                                 hwaddr reg_off, unsigned size,
                                 uint64_t value, bool start)
{
    unsigned wmask;
    uint64_t data = value;
    uint64_t desc;

    if (size == 8) {
        wmask = 3;
    } else if (reg_off & 4) {
        /*
         * High 32-bit register word.  crmRestorePP()'s wmask==1 path reads
         * the data slot's word at +4 (ld/sd view: the low 32 bits) and
         * stores it at target+4, so the value must sit in word +4.
         */
        wmask = 1;
        data = (uint32_t)value;
    } else {
        /*
         * Low 32-bit register word.  crmRestorePP()'s wmask==2 path reads
         * the data slot's word at +0 (the high 32 bits) and stores it at
         * the target, so the value must sit in word +0.
         */
        wmask = 2;
        data = value << 32;
    }

    desc = ((uint64_t)((reg_off >> 3) & 0xff) << 32)
         | ((uint64_t)(page & 7) << 40)
         | ((uint64_t)wmask << 43)
         | ((uint64_t)(start ? 1 : 0) << 45);

    s->ib_data[s->ib_wrptr] = data;
    s->ib_addr[s->ib_wrptr] = desc;
    trace_sgi_crime_re_ibpost(s->ib_wrptr, (int)page, (int)wmask,
                              (int)reg_off, data, start ? 1 : 0);
    s->ib_wrptr = (s->ib_wrptr + 1) & (CRIME_FIFO_DEPTH - 1);
    s->ib_rdptr = s->ib_wrptr;          /* retired synchronously */
}

static void sgi_crime_re_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    SGICRIMEREState *s = SGI_CRIME_RE(opaque);
    bool go = (offset & CRM_GO_OFFSET) != 0;
    hwaddr off = offset & ~CRM_GO_OFFSET;

    if (off < CRM_RE_TLB_BASE) {
        trace_sgi_crime_re_ibwrite((int)off, (int)size, value);
    }

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
                trace_sgi_crime_re_textlb((int)((t - CRM_TLB_TEX_OFFSET) / 8),
                                          value);
            } else if (t < CRM_TLB_LINEAR_A_OFFSET) {
                s->tlb_cid[(t - CRM_TLB_CID_OFFSET) / 8] = value;
            } else if (t < CRM_TLB_LINEAR_A_OFFSET + 0x80) {
                s->tlb_linear[0][(t - CRM_TLB_LINEAR_A_OFFSET) / 8] = value;
                trace_sgi_crime_re_ltlb_write(0,
                        (t - CRM_TLB_LINEAR_A_OFFSET) / 8, value);
            } else if (t < CRM_TLB_LINEAR_B_OFFSET + 0x80) {
                s->tlb_linear[1][(t - CRM_TLB_LINEAR_B_OFFSET) / 8] = value;
                trace_sgi_crime_re_ltlb_write(1,
                        (t - CRM_TLB_LINEAR_B_OFFSET) / 8, value);
            }
        }
        sgi_crime_re_ib_post(s, 1, t, size, value, go);
        return;
    }

    /* ---- Pixel pipe (0x2000-0x21FF, +go at 0x2800) ---- */
    if (off >= CRM_RE_PIXPIPE_BASE && off < CRM_RE_MTE_BASE) {
        hwaddr p = off - CRM_RE_PIXPIPE_BASE;
        uint32_t v = (uint32_t)value;

        trace_sgi_crime_re_ppwrite((int)p, value, (int)size);

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
            /*
             * Diagnostic only: log the raw store that touches the
             * Primitive register plus its source code site.  The `pc`
             * field is the host translated-block address (mem_io_pc),
             * used as a fingerprint to tell which guest store site
             * wrote a Primitive value; it is not a guest PC.
             */
            if (p == CRM_PRIMITIVE_REG || p + 4 == CRM_PRIMITIVE_REG) {
                trace_sgi_crime_re_prim(s->primitive, (int)go,
                                        s->winoffset_dst, s->drawmode,
                                        (int)p, (int)size, value,
                                        (uint64_t)(current_cpu ? current_cpu->mem_io_pc : 0));
            }
        } else {
            sgi_crime_re_pp_store(s, p, v);
            if (p == CRM_PRIMITIVE_REG) {
                trace_sgi_crime_re_prim(s->primitive, (int)go,
                                        s->winoffset_dst, s->drawmode,
                                        (int)p, (int)size, value,
                                        (uint64_t)(current_cpu ? current_cpu->mem_io_pc : 0));
            }
        }

        /*
         * Post the write to the host command ring before the go runs, so
         * a context switch between a primitive's parameters and its go
         * finds the whole batch in [StartPtr, WrPtr) and crmSavePP()
         * harvests it.
         */
        sgi_crime_re_ib_post(s, 2, p, size, value, go);

        /*
         * crmSetAndGo ORs 0x800 into the register offset — the kick.
         * Any pixpipe-page write with the go bit executes the pending
         * primitive (the last register write of the batch commits it,
         * spec §7.3.3.1 "the host indicates ... by offsetting its
         * last register write by START_OFFSET").
         */
        if (go) {
            trace_sgi_crime_re_go(off, s->primitive, s->drawmode,
                                  s->winoffset_dst);
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
            /*
             * The 64-bit src/dst registers bypass mte_store(); latch
             * both halves here so the read-back image stays complete.
             */
            if (m + 8 <= CRM_RE_MTE_SIZE) {
                s->mte_shadow[m >> 2] = (uint32_t)(value >> 32);
                s->mte_shadow[(m >> 2) + 1] = (uint32_t)v;
            }
        } else {
            sgi_crime_re_mte_store(s, m, v);
        }

        sgi_crime_re_ib_post(s, 3, m, size, value, go);

        /* MTE_SET32_AND_GO(MTE_MODE, op) — the go bit starts it */
        if (go) {
            trace_sgi_crime_re_go(off, s->primitive, s->drawmode,
                                  s->winoffset_dst);
            sgi_crime_re_mte_run(s);
        }
        return;
    }

    /* ---- Status page (0x4000) ---- */
    if (off >= CRM_RE_STATUS_BASE) {
        /*
         * SetStartPtr (@0x4008): the CRIME driver writes the FIFO start
         * pointer here (and clears it to 0 around crmSavePP()/context
         * switches).  Track it so the status register's WrPtr/StartPtr
         * fields stay coherent; everything else in the status page is
         * read-only and absorbed.
         */
        if (off == CRM_RE_SET_STARTPTR) {
            /*
             * The host's "consume up to here": move the ring base to the
             * written index.  crmSavePP() writes 0 after harvesting, and
             * the driver writes 0 after its own direct register
             * programming (crmLoadTlbABC, crimeRestore, …); keeping
             * WrPtr/RdPtr relative to it makes pending = WrPtr - StartPtr
             * the writes since the last checkpoint.
             */
            s->ib_startptr = (uint32_t)value & CRMSTAT_IB_STPTR_MASK;
            s->ib_wrptr = s->ib_startptr;
            s->ib_rdptr = s->ib_startptr;
            trace_sgi_crime_re_ctxsw((int)value, (int)s->ib_wrptr,
                                     (int)s->ib_rdptr, (int)s->ib_startptr);
        }
        return;
    }
}

static uint64_t sgi_crime_re_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t v = sgi_crime_re_read_impl(opaque, offset, size);

    trace_sgi_crime_re_regread((int)offset, (int)size, v);
    return v;
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
    s->ib_startptr = 0;
    s->ib_wrptr = 0;
    s->ib_rdptr = 0;
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
    s->depth_mode = 0;
    s->depth_z0 = 0;
    s->depth_dzdx = 0;
    s->depth_dzdy = 0;
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
    memset(s->pp_shadow, 0, sizeof(s->pp_shadow));
    memset(s->mte_shadow, 0, sizeof(s->mte_shadow));
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
        VMSTATE_UINT32(ib_startptr, SGICRIMEREState),
        VMSTATE_UINT32(ib_wrptr, SGICRIMEREState),
        VMSTATE_UINT32(ib_rdptr, SGICRIMEREState),
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
        VMSTATE_UINT32_ARRAY(scrmask, SGICRIMEREState, 10),
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
        VMSTATE_UINT32(depth_mode, SGICRIMEREState),
        VMSTATE_INT64(depth_z0, SGICRIMEREState),
        VMSTATE_INT64(depth_dzdx, SGICRIMEREState),
        VMSTATE_INT64(depth_dzdy, SGICRIMEREState),
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
        VMSTATE_UINT32_ARRAY(pp_shadow, SGICRIMEREState,
                             CRM_RE_PIXPIPE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(mte_shadow, SGICRIMEREState, CRM_RE_MTE_SIZE / 4),
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
