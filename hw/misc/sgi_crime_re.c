/*
 * SGI CRIME Rendering Engine (RE) emulation
 *
 * Implements what the IP32 PROM textport (crm_tp.c) and bring-up
 * (crm_init.c) actually issue:
 *
 *  - fb/linear TLB stores (initFramebuffer maps fb tiles via Tlb.fbA,
 *    mte_zero maps linear pages via Tlb.linearA)
 *  - pixel-pipe drawing: RECT fills (CrmTpSboxfi) and stippled LINEs
 *    (CrmTpDrawbitmap glyph rows), kicked by a write with the 0x800
 *    go-bit ORed into the pixpipe-page offset (crmSetAndGo)
 *  - MTE fills (mte_zero / MTE CLEAR via the linear TLB)
 *  - status reads: always idle (never busy-blocks)
 *
 * Tile translation (gxemul dev_sgi_re.c horrible_getputpixel, agreeing
 * with the PROM's TLB fill order in crm_init.c initFramebuffer):
 * tiles are 512 BYTES wide x 128 lines; tile_nr = (y >> 7) * 16 +
 * (x * bpp) / 512; within a tile the line offset is 512*y' + x'*bpp.
 * The fb TLB entry (u64) packs four 16-bit tile descriptors, bit 15 of
 * each = valid, bits 14:0 = physical address >> 16.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/sgi_crime_re.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

/* BufMode bits (crimedef.h) */
#define BM_BUF_TYPE_SHIFT       10
#define BM_BUF_TYPE_MASK        (7u << BM_BUF_TYPE_SHIFT)
#define BM_BUF_DEPTH_SHIFT      8
#define BM_BUF_DEPTH_MASK       (3u << BM_BUF_DEPTH_SHIFT)
#define BM_PIX_TYPE_SHIFT       4
#define BM_PIX_TYPE_MASK        (3u << BM_PIX_TYPE_SHIFT)
#define BM_PIX_DEPTH_SHIFT      2
#define BM_PIX_DEPTH_MASK       (3u << BM_PIX_DEPTH_SHIFT)

/* Stipple mode bit fields (crimedef.h) */
#define MAX_INDEX               16
#define STIPPLE_INDEX           24

/* CRM_LOGICOP_COPY = 3 */
#define CRM_LOGICOP_COPY        3

/* ------------------------------------------------------------------ */
/* Pixel get/put through the fb TLB                                    */
/* ------------------------------------------------------------------ */

/*
 * Translate a virtual framebuffer (x, y) + BufMode into a physical
 * address. Returns false if the tile is not mapped (invalid TLB entry).
 * bufmode selects the TLB bank (fbA/fbB/fbC/linearA/linearB) and the
 * pixel depth; gxemul's horrible_getputpixel semantics.
 */
static bool sgi_crime_re_fb_addr(SGICRIMEREState *s, uint32_t bufmode,
                                 int x, int y, hwaddr *phys)
{
    uint32_t tlb_sel = (bufmode >> BM_BUF_TYPE_SHIFT) & 7;
    int depth_code = (bufmode >> BM_BUF_DEPTH_SHIFT) & 3;
    int bpp = 1 << depth_code;         /* 1, 2, or 4 bytes */

    if (tlb_sel <= 2) {
        /* tiled fb buffer: 512-byte-wide tiles, 128 lines */
        if (x < 0 || y < 0 || x >= 2048 || y >= 2048) {
            return false;
        }
        int tile_w = 512 / bpp;        /* pixels across one tile */
        int tile_nr = (y >> 7) * 16 + x / tile_w;
        int entry = tile_nr >> 2;
        int sub = tile_nr & 3;
        if (entry >= CRM_TLB_FB_ENTRIES) {
            return false;
        }
        uint64_t e = s->tlb_fb[tlb_sel][entry];
        uint16_t desc = (e >> (48 - 16 * sub)) & 0xffff;
        if (!(desc & 0x8000)) {
            return false;              /* tile not valid/mapped */
        }
        int yin = y & 127;
        int xin = (x % tile_w) * bpp;
        *phys = ((hwaddr)(desc & 0x7fff) << 16) + 512 * yin + xin;
        return true;
    }

    if (tlb_sel == 4 || tlb_sel == 5) {
        /* linear buffer: y ignored, x is the linear byte offset */
        int page = x >> 12;
        int lin = tlb_sel - 4;
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
        *phys = ((hwaddr)(d & 0x7fffffffu) << 12) + (x & 0xfff);
        return true;
    }

    return false;                       /* tex/cid: not modeled */
}

/*
 * Put one pixel through the dst BufMode. colormode: CI8/RGB/RGBA per
 * the pix-type bits (gxemul getputpixel packing).
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
    address_space_rw(&address_space_memory, phys, MEMTXATTRS_UNSPECIFIED,
                     b, bpp, true);
}

/* ------------------------------------------------------------------ */
/* Pixel-pipe primitive execution (kicked by the go-bit)               */
/* ------------------------------------------------------------------ */

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

    /* direction bits: RL = bit 17? (edgeType@16, 2 bits: RL=bit16, TB=bit17
     * per crimedef PRIM_EDGE_*). gxemul: dx = RL ? -1 : 1, dy = TB ? 1 : -1 */
    int dx = (op & 0x10000) ? -1 : 1;
    int dy = (op & 0x20000) ? 1 : -1;

    /* stipple setup (crm_tp.c: mode = (maxIndex<<16)|(stippleIndex<<24)) */
    uint32_t pattern = s->stipple_pattern;
    int strip_right = 31 - ((s->stipple_mode >> MAX_INDEX) & 31);
    int strip_left = (s->stipple_mode >> STIPPLE_INDEX) & 31;
    pattern >>= strip_right;
    pattern <<= strip_right;
    pattern <<= strip_left;
    int mid_bits = 32 - strip_left - strip_right;

    switch (op & PRIM_OPCODE_MASK) {
    case PRIM_OPCODE_RECT: {
        /*
         * gxemul semantics (dev_sgi_re.c draw_primitive, DE_PRIM_RECTANGLE
         * at ~line 575): coordinates are masked to the 2048-pixel space
         * (x & 0x7ff, y & 0x7ff) and the walk WRAPS modulo 2048 —
         *   endx = (x2 + dx) & 0x7ff; endy = (y2 + dy) & 0x7ff;
         *   for (y = y1 & 0x7ff; y != endy; y = (y + dy) & 0x7ff)
         *     for (x = x1 & 0x7ff; x != endx; x = (x + dx) & 0x7ff)
         * This terminates for every (start, end, dir) because both wrap
         * in the same modulus. It is what lets gxemul survive the
         * installed kernel's clear-screen RECT (1280,0)-(0,1024) with
         * the LR_TB edge bits: x1 = 1280 & 0x7ff = 256, x2 = 0, dx = +1
         * — a plain walk from 1280 would run away (1280, 1281, ...
         * never equals 1); the masked walk is 256..2047, 0, 1 = 1790
         * steps of put_pixel clipped by the fb TLB (tiles beyond the
         * 1280x1024 console are unmapped and put_pixel just returns).
         *
         * Termination is by construction (modular walk == finite cycle),
         * but we add an explicit hard cap as a belt-and-braces guard so
         * no future register value can ever spin the vCPU forever.
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
            int itx = 0;
            for (uint32_t xx = startx; xx != endx && itx < MAX_ITER;
                 xx = (xx + dx) & (MOD - 1), itx++) {
                uint32_t color = s->shade_fgcolor;
                bool draw = true;
                if (dm & DM_ENLINESTIPPLE) {
                    /* opaque stipple: fg where pattern bit set, else bg */
                    draw = (pattern & 0x80000000u) != 0;
                }
                if (draw) {
                    sgi_crime_re_put_pixel(s, s->bufmode_dst, (int)xx,
                                          (int)yy, color);
                }
                pattern = (pattern << 1) | (pattern >> (mid_bits - 1));
            }
        }
        break;
    }

    case PRIM_OPCODE_LINE: {
        /*
         * gxemul semantics (dev_sgi_re.c draw_primitive, DE_PRIM_LINE
         * at ~line 493): vertices are masked to the 2048-pixel space
         * (x1 & 0x7ff etc.), linelen = max(|x2-x1|, |y2-y1|) (min 1),
         * and each point interpolates along the span — inherently
         * bounded by linelen. The textport draws zero-width stippled
         * lines: one horizontal span per glyph row, pattern = the
         * row's 16 (or 32) bits.
         */
        uint32_t mx1 = x1 & 0x7ff, my1 = y1 & 0x7ff;
        uint32_t mx2 = x2 & 0x7ff, my2 = y2 & 0x7ff;
        int lx = abs((int)mx2 - (int)mx1);
        int ly = abs((int)my2 - (int)my1);
        int len = lx > ly ? lx : ly;
        if (len == 0) {
            len = 1;                    /* gxemul: linelen == 0 -> 1 */
        }
        const int MAX_ITER = 2 * 0x800; /* hard cap, bounded anyway */

        for (int i = 0; i < len + 1 && i < MAX_ITER; i++) {
            /* horizontal zero-width line: y constant, x from x1 to x2 */
            int x = (int)mx1 + (int)(((int64_t)((int)mx2 - (int)mx1) * i) / len);
            int y = (int)my1;
            uint32_t color = s->shade_fgcolor;
            bool draw = true;

            if (dm & DM_ENLINESTIPPLE) {
                draw = (pattern & 0x80000000u) != 0;
            }
            if (draw) {
                sgi_crime_re_put_pixel(s, s->bufmode_dst, x, y, color);
            }
            pattern = (pattern << 1) | (pattern >> (mid_bits - 1));
        }
        break;
    }

    case PRIM_OPCODE_POINT:
        sgi_crime_re_put_pixel(s, s->bufmode_dst, (int16_t)x1, (int16_t)y1,
                              s->shade_fgcolor);
        break;

    default:
        /* FLUSH and others: nothing to execute */
        break;
    }
}

/* ------------------------------------------------------------------ */
/* MTE execution (kicked by the go-bit on an MTE register)              */
/* ------------------------------------------------------------------ */

static void sgi_crime_re_mte_run(SGICRIMEREState *s)
{
    uint32_t mode = s->mte_mode;
    bool is_copy = (mode & (1u << 11)) != 0;
    int dst_tlb = (mode >> DST_TLB_SHIFT) & 7;
    uint32_t fg = s->mte_fgvalue;
    uint8_t byte = fg & 0xff;   /* 8-bit fills replicate the byte */

    trace_sgi_crime_re_mte(mode, s->mte_dst0, s->mte_dst1, fg);

    if (dst_tlb == 4 || dst_tlb == 5) {
        /* linear destination (mte_zero: zero memory via linear TLB) */
        int lin = dst_tlb - 4;
        uint64_t start = s->mte_dst0;
        uint64_t end = s->mte_dst1;
        for (uint64_t addr = start; addr <= end && addr < (1ULL << 32);
             addr += 0x1000) {
            int page = addr >> 12;
            int entry = page >> 1;
            int sub = page & 1;
            if (entry >= CRM_TLB_LINEAR_ENTRIES) {
                break;
            }
            uint32_t d = sub
                ? (uint32_t)s->tlb_linear[lin][entry]
                : (uint32_t)(s->tlb_linear[lin][entry] >> 32);
            if (!(d & 0x80000000u)) {
                continue;
            }
            hwaddr phys = ((hwaddr)(d & 0x7fffffffu) << 12);
            uint64_t n = 0x1000;
            if (addr + n - 1 > end) {
                n = end - addr + 1;
            }
            uint8_t buf[4096];
            memset(buf, byte, sizeof(buf));
            address_space_rw(&address_space_memory, phys,
                             MEMTXATTRS_UNSPECIFIED, buf, n, true);
        }
        return;
    }

    if (dst_tlb <= 2 && !is_copy) {
        /* tiled fb destination: dst0/dst1 are (x,y) vertex pairs
         * (NetBSD crmfb_fill_rect semantics, gxemul do_mte_transfer) */
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
        for (int y = y1; y != y2 + dy; y += dy) {
            for (int x = x1; x != x2 + dx; x += dx) {
                sgi_crime_re_put_pixel(s, bufmode, x, y, fg);
            }
        }
    }
    /* MTE COPY (blit): not needed by the PROM textport; absorb */
}

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

/* Store one 32-bit MTE register (offset relative to MTE page base) */
static void sgi_crime_re_mte_store(SGICRIMEREState *s, hwaddr m, uint32_t v)
{
    switch (m) {
    case CRM_MTE_MODE_REG:        s->mte_mode = v; return;
    case CRM_MTE_BYTEMASK_REG:    s->mte_bytemask = v; return;
    case CRM_MTE_FGVALUE_REG:     s->mte_fgvalue = v; return;
    case CRM_MTE_DST0_REG:        s->mte_dst0 = v; return;
    case CRM_MTE_DST1_REG:        s->mte_dst1 = v; return;
    default:
        return;                    /* src regs/stipple mask/y-steps: absorb */
    }
}

static void sgi_crime_re_pp_store(SGICRIMEREState *s, hwaddr p, uint32_t v)
{
    switch (p) {
    case CRM_BUF_MODE_SRC_REG:
        s->bufmode_src = v;
        trace_sgi_crime_re_bufmode(v, s->bufmode_dst);
        return;
    case CRM_BUF_MODE_DST_REG:
        s->bufmode_dst = v;
        trace_sgi_crime_re_bufmode(s->bufmode_src, v);
        return;
    case CRM_DRAW_MODE_REG:      s->drawmode = v; return;
    case CRM_PRIMITIVE_REG:      s->primitive = v; return;
    case CRM_VERTEX_X_XY0_REG:   s->vertex_xy[0] = v; return;
    case CRM_VERTEX_X_XY1_REG:   s->vertex_xy[1] = v; return;
    case CRM_VERTEX_X_XY2_REG:   s->vertex_xy[2] = v; return;
    case CRM_STIPPLE_MODE_REG:   s->stipple_mode = v; return;
    case CRM_STIPPLE_PATT_REG:   s->stipple_pattern = v; return;
    case CRM_SHADE_FGCOLOR_REG:  s->shade_fgcolor = v; return;
    case CRM_SHADE_BGCOLOR_REG:  s->shade_bgcolor = v; return;
    case CRM_LOGICOP_REG:        s->logicop = v; return;
    case CRM_COLORMASK_REG:      s->colormask = v; return;
    case CRM_PIXELXFER_SRC_ADDR_REG: s->pixelxfer_src_addr = v; return;
    case CRM_PIXELXFER_SRC_XSTEP_REG: s->pixelxfer_src_xstep = v; return;
    case CRM_PIXELXFER_SRC_YSTEP_REG: s->pixelxfer_src_ystep = v; return;
    default:
        /* scrmask/scissor/winoffset/startsetup/texture/fog/blend/
         * depth/stencil/PixPipeNull/Flush: absorb */
        return;
    }
}

static uint64_t sgi_crime_re_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case CRM_RE_STATUS_REG:
        /* Report all subsystems idle, buffer empty — the RE never
         * busy-blocks (mte_spin / crmWaitReFifo / crmWaitReIdle all
         * poll these bits and must see them set immediately). */
        return CRMSTAT_ALL_IDLE;

    case CRM_RE_INTFBUF_ADDR0:
        /*
         * crmGetRERev reads bit 0 here to decide RE revision; 0 = rev B
         * (crm_config.c "SGI-CRM, Rev B"). Keep 0.
         */
        return 0;

    default:
        return 0;
    }
}

static void sgi_crime_re_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    SGICRIMEREState *s = SGI_CRIME_RE(opaque);
    bool go = (offset & CRM_GO_OFFSET) != 0;
    hwaddr off = offset & ~CRM_GO_OFFSET;

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
            } else if (t >= CRM_TLB_LINEAR_A_OFFSET &&
                       t < CRM_TLB_LINEAR_A_OFFSET + 0x80) {
                s->tlb_linear[0][(t - CRM_TLB_LINEAR_A_OFFSET) / 8] = value;
            } else if (t >= CRM_TLB_LINEAR_B_OFFSET &&
                       t < CRM_TLB_LINEAR_B_OFFSET + 0x80) {
                s->tlb_linear[1][(t - CRM_TLB_LINEAR_B_OFFSET) / 8] = value;
            }
            /* tex/cid TLB entries: absorb */
        }
        return;
    }

    /* ---- Pixel pipe (0x2000-0x21FF, +go at 0x2800) ---- */
    if (off >= CRM_RE_PIXPIPE_BASE && off < CRM_RE_MTE_BASE) {
        hwaddr p = off - CRM_RE_PIXPIPE_BASE;
        uint32_t v = (uint32_t)value;

        if (size == 8) {
            /* 64-bit write covers two adjacent 32-bit regs. The pairs
             * the PROM writes via crmSet64: Vertex.X[0]/X[1] (CrmTpBmove)
             * and the TLB-style doublewords. Store into the reg file via
             * the 32-bit path for both halves. */
            hwaddr p_hi = p;
            hwaddr p_lo = p + 4;
            /* v = high word (first BE word), value>>32 = low word */
            sgi_crime_re_pp_store(s, p_hi, v);
            sgi_crime_re_pp_store(s, p_lo, (uint32_t)(value >> 32));
        } else {
            sgi_crime_re_pp_store(s, p, v);
        }

        /*
         * crmSetAndGo ORs 0x800 into the register offset — the kick.
         * The textport's kick is a write to PixPipeNull|0x800; the
         * start-setup path uses StartSetup|0x800. Any pixpipe-page
         * write with the go bit executes the pending primitive.
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
            /* dst0/dst1 are 64-bit capable (CrmVaddrType); the low
             * register (+0) holds the low 32 bits per gxemul's note */
            switch (m) {
            case CRM_MTE_DST0_REG: s->mte_dst0 = value; break;
            case CRM_MTE_DST1_REG: s->mte_dst1 = value; break;
            case CRM_MTE_SRC0_REG: case CRM_MTE_SRC1_REG: break;
            default: {
                /* store both halves through the 32-bit path */
                hwaddr m_hi = m, m_lo = m + 4;
                sgi_crime_re_mte_store(s, m_hi, v);
                sgi_crime_re_mte_store(s, m_lo, (uint32_t)(value >> 32));
                break;
            }
            }
        } else {
            sgi_crime_re_mte_store(s, m, v);
        }

        /* mte_zero kicks via MTE_SET32_AND_GO(CRM_MTE_MODE_REG, op) —
         * the go bit on the MODE write starts the transfer */
        if (go) {
            sgi_crime_re_mte_run(s);
        }
        return;
    }

    /* ---- Status page (0x4000) ---- */
    if (off >= CRM_RE_STATUS_BASE) {
        return;                        /* read-only; absorb writes */
    }

    /* Interface buffer (0x0000-0x0FFF): absorb */
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
    memset(s->tlb_linear, 0, sizeof(s->tlb_linear));
    s->bufmode_src = 0;
    s->bufmode_dst = 0;
    s->drawmode = 0;
    s->primitive = 0;
    memset(s->vertex_xy, 0, sizeof(s->vertex_xy));
    s->stipple_mode = 0;
    s->stipple_pattern = 0;
    s->shade_fgcolor = 0;
    s->shade_bgcolor = 0;
    s->logicop = 0;
    s->colormask = 0;
    s->mte_mode = 0;
    s->mte_bytemask = 0;
    s->mte_fgvalue = 0;
    s->mte_dst0 = 0;
    s->mte_dst1 = 0;
    s->status = CRMSTAT_ALL_IDLE;
}

static const VMStateDescription vmstate_sgi_crime_re = {
    .name = "sgi-crime-re",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(status, SGICRIMEREState),
        VMSTATE_UINT64_2DARRAY(tlb_fb, SGICRIMEREState, 3,
                               CRM_TLB_FB_ENTRIES),
        VMSTATE_UINT64_2DARRAY(tlb_linear, SGICRIMEREState, 2,
                               CRM_TLB_LINEAR_ENTRIES),
        VMSTATE_UINT32(bufmode_src, SGICRIMEREState),
        VMSTATE_UINT32(bufmode_dst, SGICRIMEREState),
        VMSTATE_UINT32(drawmode, SGICRIMEREState),
        VMSTATE_UINT32(primitive, SGICRIMEREState),
        VMSTATE_UINT32_ARRAY(vertex_xy, SGICRIMEREState, 3),
        VMSTATE_UINT32(stipple_mode, SGICRIMEREState),
        VMSTATE_UINT32(stipple_pattern, SGICRIMEREState),
        VMSTATE_UINT32(shade_fgcolor, SGICRIMEREState),
        VMSTATE_UINT32(shade_bgcolor, SGICRIMEREState),
        VMSTATE_UINT32(logicop, SGICRIMEREState),
        VMSTATE_UINT32(colormask, SGICRIMEREState),
        VMSTATE_UINT32(mte_mode, SGICRIMEREState),
        VMSTATE_UINT32(mte_bytemask, SGICRIMEREState),
        VMSTATE_UINT32(mte_fgvalue, SGICRIMEREState),
        VMSTATE_UINT64(mte_dst0, SGICRIMEREState),
        VMSTATE_UINT64(mte_dst1, SGICRIMEREState),
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
