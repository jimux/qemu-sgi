/*
 * SGI Newport (XL) graphics controller emulation
 *
 * Newport is the graphics subsystem used in SGI Indy (IP24) and
 * some Indigo2 (IP22) workstations. It consists of:
 *   - REX3: Raster Engine - 2D drawing engine and bus interface
 *   - RB2: Frame buffer input controller
 *   - RO1: Frame buffer output controller
 *   - XMAP9: Final display generator
 *   - CMAP: Color palette mapper
 *   - VC2: Video timing controller / CRTC
 *
 * This implements the REX3 drawing engine with VRAM, block/span/line
 * drawing, host-data text rendering, and CMAP palette display output.
 *
 * The critical concept is the "Go" register: REX3 register space
 * 0x0000-0x07FF is "Set" (store only), 0x0800-0x0FFF is "Go"
 * (store + execute command). MAME: `if (offset & 0x100)` in 8-byte
 * granularity, i.e. bit 11 in byte address space.
 *
 * References:
 *   - MAME src/devices/bus/gio64/newport.cpp
 *   - Linux arch/mips/sgi-ip22/
 *   - NetBSD sys/arch/sgimips/dev/newport.c
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/display/sgi_newport.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "trace.h"

/* NewView binary log record (20 bytes, MAME-compatible layout) */
static void newport_newview_log(SGINewportState *s, uint32_t offset,
                                uint32_t data)
{
    if (s->newview_log_file) {
        uint32_t record[5];
        record[0] = offset;       /* register offset; bit 30=read, bit 31=frame */
        record[1] = 0;            /* data_hi (unused for 32-bit regs) */
        record[2] = data;         /* data_lo */
        record[3] = 0;            /* mask_hi */
        record[4] = 0xFFFFFFFF;   /* mask_lo */
        fwrite(record, sizeof(record), 1, s->newview_log_file);
    }
}

/* REX3 Go register offset — writes to 0x0800-0x0FFF trigger commands */
#define REX3_GO_OFFSET      0x0800

/* DRAWMODE0 field extraction */
#define DM0_OPCODE(dm0)     ((dm0) & 3)
#define DM0_ADRMODE(dm0)    (((dm0) >> 2) & 7)
#define DM0_COLORHOST(dm0)  (((dm0) >> 6) & 1)
#define DM0_STOPONX(dm0)    (((dm0) >> 8) & 1)
#define DM0_STOPONY(dm0)    (((dm0) >> 9) & 1)

/* DRAWMODE0 opcodes */
#define DM0_OP_NOOP         0
#define DM0_OP_READ         1
#define DM0_OP_DRAW         2
#define DM0_OP_SCR2SCR      3

/* DRAWMODE0 address modes */
#define DM0_ADR_SPAN        0
#define DM0_ADR_BLOCK       1
#define DM0_ADR_ILINE       2
#define DM0_ADR_FLINE       3

/* DRAWMODE0 bit positions */
#define DM0_DOSETUP         (1 << 5)
#define DM0_SKIPFIRST       (1 << 10)
#define DM0_SKIPLAST        (1 << 11)
#define DM0_LENGTH32        (1 << 15)
#define DM0_SHADE           (1 << 18)
#define DM0_LR_ABORT        (1 << 19)
#define DM0_CICLAMP         (1 << 21)

/* Host depth table — bits per pixel indexed by dm1_hostdepth */
static const uint32_t host_depth_bpp[4] = { 8, 8, 16, 32 };

/*
 * Convert two's complement to sign-magnitude format.
 * The REX3 slope registers store values in sign-magnitude rather than
 * two's complement. Writing a two's complement value is converted:
 *   negative: sign_bit | (abs(val) & magnitude_mask)
 *   positive: val & magnitude_mask
 *
 * IRIX ref: test_rex3_slopecolor() in rex3.c
 * MAME ref: convert_to_sm() in newport.cpp
 */
static uint32_t newport_twos_to_sm(uint32_t data, int nbits)
{
    uint32_t sign = 1U << (nbits - 1);
    uint32_t mask = sign - 1;

    if (data & 0x80000000) {
        return sign | ((-data) & mask);
    } else {
        return data & mask;
    }
}

/*
 * Host color mask table — MAME ref: s_color_masks[] at newport.cpp
 * At hostdepth=0: 4-bit pixel values sit in 8-bit-aligned slots,
 * so we need mask 0xf (not 0xff as computed from depth).
 */
static const uint32_t host_color_masks[4] = { 0xf, 0xff, 0xfff, 0xffffffff };

/*
 * ============================================================
 * Depth conversion functions
 * ============================================================
 *
 * Convert pixel color between different bit depths (4, 8, 12, 24 bpp).
 * Used when hostdepth != drawdepth (plane depth) to convert between
 * host data format and VRAM storage format.
 *
 * Color components are BGR order (B in high bits, R in low bits),
 * matching the Newport hardware convention.
 *
 * MAME ref: convert_*bpp_bgr_to_*bpp() at newport.cpp:1559-1653
 */

/* 4bpp -> 8/12/24bpp */
static uint32_t convert_4bpp_to_8bpp(uint8_t p)
{
    uint8_t r = 0xff * ((p >> 0) & 1);
    uint8_t g = (0xaa * ((p >> 2) & 1)) | (0x55 * ((p >> 1) & 1));
    uint8_t b = 0xff * ((p >> 3) & 1);
    return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

static uint32_t convert_4bpp_to_12bpp(uint8_t p)
{
    uint32_t r = 0xff * ((p >> 0) & 1);
    uint32_t g = (0xaa * ((p >> 2) & 1)) | (0x55 * ((p >> 1) & 1));
    uint32_t b = 0xff * ((p >> 3) & 1);
    return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

static uint32_t convert_4bpp_to_24bpp(uint8_t p)
{
    uint8_t r = 0xff * ((p >> 0) & 1);
    uint8_t g = (0xaa * ((p >> 2) & 1)) | (0x55 * ((p >> 1) & 1));
    uint8_t b = 0xff * ((p >> 3) & 1);
    return (b << 16) | (g << 8) | r;
}

/* 8bpp -> 4/12/24bpp */
static uint32_t convert_8bpp_to_4bpp(uint8_t p)
{
    uint8_t r = (0x92 * ((p >> 2) & 1)) | (0x49 * ((p >> 1) & 1))
              | (0x24 * ((p >> 0) & 1));
    uint8_t g = (0x92 * ((p >> 5) & 1)) | (0x49 * ((p >> 4) & 1))
              | (0x24 * ((p >> 3) & 1));
    uint8_t b = (0xaa * ((p >> 7) & 1)) | (0x55 * ((p >> 6) & 1));
    return (((b >> 7) & 1) << 3) | ((g & 0xc0) >> 5) | ((r >> 7) & 1);
}

static uint32_t convert_8bpp_to_12bpp(uint8_t p)
{
    uint8_t r = (0x92 * ((p >> 2) & 1)) | (0x49 * ((p >> 1) & 1))
              | (0x24 * ((p >> 0) & 1));
    uint8_t g = (0x92 * ((p >> 5) & 1)) | (0x49 * ((p >> 4) & 1))
              | (0x24 * ((p >> 3) & 1));
    uint8_t b = (0xaa * ((p >> 7) & 1)) | (0x55 * ((p >> 6) & 1));
    return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

static uint32_t convert_8bpp_to_24bpp(uint8_t p)
{
    uint8_t r = (0x92 * ((p >> 2) & 1)) | (0x49 * ((p >> 1) & 1))
              | (0x24 * ((p >> 0) & 1));
    uint8_t g = (0x92 * ((p >> 5) & 1)) | (0x49 * ((p >> 4) & 1))
              | (0x24 * ((p >> 3) & 1));
    uint8_t b = (0xaa * ((p >> 7) & 1)) | (0x55 * ((p >> 6) & 1));
    return (b << 16) | (g << 8) | r;
}

/* 12bpp -> 4/8/24bpp */
static uint32_t convert_12bpp_to_4bpp(uint16_t p)
{
    uint8_t r = 0x11 * ((p >> 0) & 0xf);
    uint8_t g = 0x11 * ((p >> 4) & 0xf);
    uint8_t b = 0x11 * ((p >> 8) & 0xf);
    return (((b >> 7) & 1) << 3) | ((g & 0xc0) >> 5) | ((r >> 7) & 1);
}

static uint32_t convert_12bpp_to_8bpp(uint16_t p)
{
    uint8_t r = 0x11 * ((p >> 0) & 0xf);
    uint8_t g = 0x11 * ((p >> 4) & 0xf);
    uint8_t b = 0x11 * ((p >> 8) & 0xf);
    return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

static uint32_t convert_12bpp_to_24bpp(uint16_t p)
{
    uint8_t r = 0x11 * ((p >> 0) & 0xf);
    uint8_t g = 0x11 * ((p >> 4) & 0xf);
    uint8_t b = 0x11 * ((p >> 8) & 0xf);
    return (b << 16) | (g << 8) | r;
}

/* 24bpp -> 4/8/12bpp */
static uint32_t convert_24bpp_to_4bpp(uint32_t p)
{
    uint8_t r = (uint8_t)(p >> 0);
    uint8_t g = (uint8_t)(p >> 8);
    uint8_t b = (uint8_t)(p >> 16);
    return (((b >> 7) & 1) << 3) | ((g & 0xc0) >> 5) | ((r >> 7) & 1);
}

static uint32_t convert_24bpp_to_8bpp(uint32_t p)
{
    uint8_t r = (uint8_t)(p >> 0);
    uint8_t g = (uint8_t)(p >> 8);
    uint8_t b = (uint8_t)(p >> 16);
    return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

static uint32_t convert_24bpp_to_12bpp(uint32_t p)
{
    uint8_t r = (uint8_t)(p >> 0);
    uint8_t g = (uint8_t)(p >> 8);
    uint8_t b = (uint8_t)(p >> 16);
    return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

/*
 * Apply depth conversion from host depth to plane (draw) depth.
 * convert_index = (hostdepth << 2) | drawdepth.
 * When hostdepth == drawdepth, no conversion is needed (identity cases).
 * MAME ref: get_host_color() at newport.cpp:2202-2244
 */
static uint32_t newport_convert_host_to_draw(uint32_t color,
                                              uint8_t hostdepth,
                                              uint8_t drawdepth)
{
    uint8_t idx = (hostdepth << 2) | drawdepth;
    switch (idx & 15) {
    case 1:  return convert_4bpp_to_8bpp((uint8_t)color);
    case 2:  return convert_4bpp_to_12bpp((uint8_t)color);
    case 3:  return convert_4bpp_to_24bpp((uint8_t)color);
    case 4:  return convert_8bpp_to_4bpp((uint8_t)color);
    case 6:  return convert_8bpp_to_12bpp((uint8_t)color);
    case 7:  return convert_8bpp_to_24bpp((uint8_t)color);
    case 8:  return convert_12bpp_to_4bpp((uint16_t)color);
    case 9:  return convert_12bpp_to_8bpp((uint16_t)color);
    case 11: return convert_12bpp_to_24bpp((uint16_t)color);
    case 12: return convert_24bpp_to_4bpp(color);
    case 13: return convert_24bpp_to_8bpp(color);
    case 14: return convert_24bpp_to_12bpp(color);
    default: return color;  /* Identity (0, 5, 10, 15) */
    }
}

/*
 * Apply depth conversion from plane (draw) depth to host depth.
 * Used in the pixel read path — inverse of host-to-draw.
 * convert_index = (drawdepth << 2) | hostdepth.
 * MAME ref: do_pixel_read() at newport.cpp:3080-3122
 */
static uint32_t newport_convert_draw_to_host(uint32_t color,
                                              uint8_t drawdepth,
                                              uint8_t hostdepth)
{
    uint8_t idx = (drawdepth << 2) | hostdepth;
    switch (idx & 15) {
    case 1:  return convert_4bpp_to_8bpp((uint8_t)color);
    case 2:  return convert_4bpp_to_12bpp((uint8_t)color);
    case 3:  return convert_4bpp_to_24bpp((uint8_t)color);
    case 4:  return convert_8bpp_to_4bpp((uint8_t)color);
    case 6:  return convert_8bpp_to_12bpp((uint8_t)color);
    case 7:  return convert_8bpp_to_24bpp((uint8_t)color);
    case 8:  return convert_12bpp_to_4bpp((uint16_t)color);
    case 9:  return convert_12bpp_to_8bpp((uint16_t)color);
    case 11: return convert_12bpp_to_24bpp((uint16_t)color);
    case 12: return convert_24bpp_to_4bpp(color);
    case 13: return convert_24bpp_to_8bpp(color);
    case 14: return convert_24bpp_to_12bpp(color);
    default: return color;  /* Identity */
    }
}

/*
 * ============================================================
 * Pixel output pipeline
 * ============================================================
 */

/*
 * Get the default drawing color based on depth and mode.
 * Expands color_i or color_vram to fill all lanes.
 * MAME ref: get_default_color() at newport.cpp:3276-3305
 */
static uint32_t newport_get_default_color(SGINewportState *s)
{
    uint32_t color;

    if (s->dm1_fastclear) {
        color = s->color_vram;
    } else {
        color = s->color_i;
    }

    switch (s->dm1_drawdepth) {
    case 0: /* 4bpp */
        color &= 0xf;
        color |= (color << 4);
        color |= (color << 8);
        color |= (color << 16);
        break;
    case 1: /* 8bpp */
        color &= 0xff;
        color |= (color << 8);
        color |= (color << 16);
        break;
    case 2: /* 12bpp */
        color &= 0xfff;
        color |= (color << 12);
        break;
    case 3: /* 24bpp */
        color &= 0xffffff;
        break;
    }

    return color;
}

/*
 * Extract a pixel color from the host data port.
 * Used when dm0_colorhost is set (text rendering, bitmap blits).
 * MAME ref: get_host_color() at newport.cpp:2191-2248
 */
static uint32_t newport_get_host_color(SGINewportState *s)
{
    uint32_t depth = host_depth_bpp[s->dm1_hostdepth];
    uint32_t color;

    color = (uint32_t)(s->host_dataport >> s->host_shift) & host_color_masks[s->dm1_hostdepth];

    /*
     * Only advance shift when rwpacked is set.
     * Without rwpacked, every pixel read returns the same value.
     * MAME ref: get_host_color() at newport.cpp:2195
     */
    if (s->dm1_rwpacked) {
        if ((s->dm1_rwdouble && s->host_shift > 0) || s->host_shift > 32) {
            s->host_shift -= depth;
        } else {
            s->host_shift = 64 - depth;
        }
    }

    /*
     * Convert from host depth to draw (plane) depth when they differ.
     * MAME ref: get_host_color() at newport.cpp:2202-2244
     */
    color = newport_convert_host_to_draw(color, s->dm1_hostdepth,
                                          s->dm1_drawdepth);

    /*
     * Endian swap — byte-reverse 32-bit color when DM1 bit 11 is set.
     * MAME ref: get_host_color() at newport.cpp:2245-2246
     */
    if (s->dm1_swapendian) {
        color = bswap32(color);
    }

    return color;
}

/*
 * Get RGB color from current color slope accumulators.
 * Extracts 9-bit value from bits [19:11] of each color register,
 * clamps negative (>= 0x180 or sign bit set) to 0, overflow (> 0xff) to 0xff.
 * MAME ref: get_rgb_color() at newport.cpp:2548-2654
 */
static uint32_t newport_get_rgb_color(SGINewportState *s)
{
    uint32_t red   = (s->curr_color_red >> 11) & 0x1ff;
    uint32_t green = (s->curr_color_green >> 11) & 0x1ff;
    uint32_t blue  = (s->curr_color_blue >> 11) & 0x1ff;

    if (red >= 0x180 || (s->curr_color_red & 0x80000000)) {
        red = 0;
    } else if (red > 0xff) {
        red = 0xff;
    }

    if (green >= 0x180 || (s->curr_color_green & 0x80000000)) {
        green = 0;
    } else if (green > 0xff) {
        green = 0xff;
    }

    if (blue >= 0x180 || (s->curr_color_blue & 0x80000000)) {
        blue = 0;
    } else if (blue > 0xff) {
        blue = 0xff;
    }

    if (!s->dm1_rgbmode) {
        /* CI mode — fall back to default color */
        return newport_get_default_color(s);
    }

    return (red << 16) | (green << 8) | blue;
}

/*
 * Iterate shade — advance color accumulators by slope values.
 * Called per-pixel when DM0 SHADE bit (18) is set.
 * Slopes are stored in sign-magnitude; convert to signed for addition.
 * MAME ref: iterate_shade() at newport.cpp:3212-3251
 */
static int32_t newport_sm_to_signed(int32_t val, int nbits)
{
    int32_t sign = 1 << (nbits - 1);
    int32_t mask = sign - 1;
    if (val & sign) {
        return -(val & mask);
    }
    return val & mask;
}

static void newport_iterate_shade(SGINewportState *s)
{
    if (s->slope_red & 0x7fffff) {
        s->curr_color_red += newport_sm_to_signed(s->slope_red, 24);
    }
    if (s->slope_green & 0x7ffff) {
        s->curr_color_green += newport_sm_to_signed(s->slope_green, 20);
    }
    if (s->slope_blue & 0x7ffff) {
        s->curr_color_blue += newport_sm_to_signed(s->slope_blue, 20);
    }
    if (s->slope_alpha & 0x7ffff) {
        s->curr_color_alpha += newport_sm_to_signed(s->slope_alpha, 20);
    }

    /* CIClamp (DM0 bit 21) — clamp color accumulators */
    if (s->drawmode0 & DM0_CICLAMP) {
        if (s->dm1_rgbmode) {
            uint32_t val;

            val = (s->curr_color_red >> 11) & 0x1ff;
            if (val >= 0x180 || (s->curr_color_red & 0x80000000)) {
                s->curr_color_red = 0;
            } else if (val > 0xff) {
                s->curr_color_red = 0x7ffff;
            }

            val = (s->curr_color_green >> 11) & 0x1ff;
            if (val >= 0x180 || (s->curr_color_green & 0x80000000)) {
                s->curr_color_green = 0;
            } else if (val > 0xff) {
                s->curr_color_green = 0x7ffff;
            }

            val = (s->curr_color_blue >> 11) & 0x1ff;
            if (val >= 0x180 || (s->curr_color_blue & 0x80000000)) {
                s->curr_color_blue = 0;
            } else if (val > 0xff) {
                s->curr_color_blue = 0x7ffff;
            }

            val = (s->curr_color_alpha >> 11) & 0x1ff;
            if (val >= 0x180 || (s->curr_color_alpha & 0x80000000)) {
                s->curr_color_alpha = 0;
            } else if (val > 0xff) {
                s->curr_color_alpha = 0x7ffff;
            }
        }
    }
}

/*
 * Compute octant from start/end coordinates and write into bres_octant_inc1.
 * Called when DM0 DOSETUP bit (5) is set before each command.
 * MAME ref: do_setup() at newport.cpp:2724-2740, get_octant() at line 2693
 */
static void newport_do_setup(SGINewportState *s)
{
    int32_t x1 = (int32_t)(s->x_start << 5) >> 12;  /* sign-extend 20-bit from bits [26:7] */
    int32_t y1 = (int32_t)(s->y_start << 5) >> 12;
    int32_t x2 = (int32_t)(s->x_end << 5) >> 12;
    int32_t y2 = (int32_t)(s->y_end << 5) >> 12;
    int32_t dx = abs(x1 - x2);
    int32_t dy = abs(y1 - y2);
    uint8_t octant = 0;

    if (y1 > y2) {
        octant |= 1;  /* YDEC */
    }
    if (x1 > x2) {
        octant |= 2;  /* XDEC */
    }
    if (dx > dy) {
        octant |= 4;  /* XMAJOR */
    }

    s->bres_octant_inc1 = (s->bres_octant_inc1 & ~(7U << 24)) |
                          ((uint32_t)octant << 24);
}

/*
 * Reset current color accumulators to base color values.
 * Called at end of scanline or when X reaches end in span/block.
 * MAME ref: newport.cpp lines 3408-3411, 3480-3483
 */
static void newport_reset_curr_colors(SGINewportState *s)
{
    s->curr_color_red   = s->color_red;
    s->curr_color_alpha = s->color_alpha;
    s->curr_color_green = s->color_green;
    s->curr_color_blue  = s->color_blue;
}

/*
 * Apply logic operation and write masked pixel to VRAM.
 * MAME ref: logic_pixel() at newport.cpp:1039-1062,
 *           store_pixel() at newport.cpp:1064-1069
 */
static void newport_logic_pixel(SGINewportState *s, uint32_t addr,
                                uint32_t src)
{
    uint32_t *buf;
    uint32_t dst, result, mask;

    /* Select buffer based on plane enable */
    switch (s->dm1_planes) {
    case 0: /* No planes enabled */
        return;
    case 1: /* RGBCI planes (8-bit CI or 24-bit RGB) */
    case 2:
    case 4: /* Overlay planes — write to cidaux */
    case 5:
        break;
    default:
        break;
    }

    /*
     * For now, always write to rgbci buffer. Overlay/CID planes
     * would go to cidaux in a full implementation.
     */
    if (s->dm1_planes == 4 || s->dm1_planes == 5) {
        buf = s->vram_cidaux;
    } else {
        buf = s->vram_rgbci;
    }

    if (addr >= (uint32_t)(NEWPORT_VRAM_W * NEWPORT_VRAM_H)) {
        return;
    }

    dst = buf[addr];

    /* 16 ROP logic operations */
    switch (s->dm1_logicop) {
    case 0x0: result = 0; break;
    case 0x1: result = src & dst; break;
    case 0x2: result = src & ~dst; break;
    case 0x3: result = src; break;              /* SRC — most common */
    case 0x4: result = ~src & dst; break;
    case 0x5: result = dst; break;              /* DST (noop) */
    case 0x6: result = src ^ dst; break;        /* XOR */
    case 0x7: result = src | dst; break;
    case 0x8: result = ~(src | dst); break;     /* NOR */
    case 0x9: result = ~(src ^ dst); break;     /* XNOR */
    case 0xa: result = ~dst; break;
    case 0xb: result = src | ~dst; break;
    case 0xc: result = ~src; break;
    case 0xd: result = ~src | dst; break;
    case 0xe: result = ~(src & dst); break;     /* NAND */
    case 0xf: result = 0xffffffff; break;
    default:  result = src; break;
    }

    mask = s->write_mask & s->global_mask;
    buf[addr] = (buf[addr] & ~mask) | (result & mask);
}

/*
 * Pixel clipping test against screenmasks.
 * Returns true if the pixel passes all enabled clip masks.
 *
 * Mask 0 operates in pre-window-offset coordinates (raw x,y).
 * Masks 1-4 operate in post-window-offset coordinates.
 * A pixel passes masks 1-4 if it's inside ANY enabled mask (OR logic).
 *
 * MAME ref: pixel_clip_pass() at newport.cpp:2262-2343
 */
static bool newport_pixel_clip_pass(SGINewportState *s, int16_t x, int16_t y)
{
    /* Mask 0: check raw coords before window offset */
    if (s->clip_mode & 1) {
        int16_t min_x = (int16_t)(s->smask_x[0] >> 16);
        int16_t min_y = (int16_t)(s->smask_y[0] >> 16);
        int16_t max_x = (int16_t)(s->smask_x[0] & 0xffff);
        int16_t max_y = (int16_t)(s->smask_y[0] & 0xffff);

        if (x < min_x || x > max_x || y < min_y || y > max_y) {
            return false;
        }
    }

    /* Apply window offset for masks 1-4 */
    int wx = x + (int16_t)((s->xy_window >> 16) & 0xffff) - 0x1000;
    int wy = y + (int16_t)(s->xy_window & 0xffff) - 0x1000;

    /* Masks 1-4: pixel passes if inside ANY enabled mask */
    if (s->clip_mode & 0x1e) {
        int bit;
        for (bit = 1; bit < 5; bit++) {
            if (!(s->clip_mode & (1 << bit))) {
                continue;
            }

            int16_t min_x = (int16_t)(s->smask_x[bit] >> 16) - 0x1000;
            int16_t min_y = (int16_t)(s->smask_y[bit] >> 16) - 0x1000;
            int16_t max_x = (int16_t)(s->smask_x[bit] & 0xffff) - 0x1000;
            int16_t max_y = (int16_t)(s->smask_y[bit] & 0xffff) - 0x1000;

            if (wx >= min_x && wx <= max_x && wy >= min_y && wy <= max_y) {
                break;  /* Passed this mask */
            }
        }
        if (bit == 5) {
            return false;  /* Failed all enabled masks 1-4 */
        }
    }

    return true;
}

/*
 * Output a pixel at coordinates (x, y), applying clipping and window offset.
 * MAME ref: output_pixel() at newport.cpp:2345-2363
 */
static void newport_output_pixel(SGINewportState *s, int16_t x, int16_t y,
                                 uint32_t color)
{
    int wx, wy;
    uint32_t addr;

    /* Check screenmask clipping before output */
    if (s->clip_mode && !newport_pixel_clip_pass(s, x, y)) {
        return;
    }

    /* Apply window offset — MAME subtracts 0x1000 after adding */
    wx = x + (int16_t)((s->xy_window >> 16) & 0xffff) - 0x1000;
    wy = y + (int16_t)(s->xy_window & 0xffff) - 0x1000;

    /* Bounds check against VRAM */
    if (wx < 0 || wx >= NEWPORT_VRAM_W || wy < 0 || wy >= NEWPORT_VRAM_H) {
        return;
    }

    addr = (uint32_t)wy * NEWPORT_VRAM_W + (uint32_t)wx;
    newport_logic_pixel(s, addr, color);
}

/*
 * ============================================================
 * Drawing commands
 * ============================================================
 */

/*
 * Update X start coordinate registers.
 * Takes raw XSTART-format value: bits 26:11=integer, 10:7=fraction.
 * MAME ref: write_x_start() at newport.cpp:3552-3558
 */
static void newport_write_x_start(SGINewportState *s, int32_t val)
{
    s->x_start = val & 0x07ffff80;
    s->x_start_i = s->x_start;
    s->x_start_f = (uint32_t)val & 0x007fff80;
    s->x_start_int = (int16_t)(val >> 11);
    s->xy_start_i = (s->xy_start_i & 0x0000ffff) |
                     ((uint16_t)s->x_start_int << 16);
}

/*
 * Update Y start coordinate registers.
 * Takes raw YSTART-format value: bits 26:11=integer, 10:7=fraction.
 * MAME ref: write_y_start() at newport.cpp:3561-3567
 */
static void newport_write_y_start(SGINewportState *s, int32_t val)
{
    s->y_start = val & 0x07ffff80;
    s->y_start_f = (uint32_t)val & 0x007fff80;
    s->y_start_int = (int16_t)(val >> 11);
    s->xy_start_i = (s->xy_start_i & 0xffff0000) |
                     (uint16_t)s->y_start_int;
}

/*
 * Update X end coordinate registers.
 * Takes raw XEND-format value: bits 26:11=integer, 10:7=fraction.
 * MAME ref: write_x_end() at newport.cpp:3570-3576
 */
static void newport_write_x_end(SGINewportState *s, int32_t val)
{
    s->x_end = val & 0x07ffff80;
    s->x_end_f = (uint32_t)val & 0x007fff80;
    s->x_end_int = (int16_t)(val >> 11);
    s->xy_end_i = (s->xy_end_i & 0x0000ffff) |
                   ((uint16_t)s->x_end_int << 16);
}

/*
 * Update Y end coordinate registers.
 * Takes raw YEND-format value: bits 26:11=integer, 10:7=fraction.
 * MAME ref: write_y_end() at newport.cpp:3579-3585
 */
static void newport_write_y_end(SGINewportState *s, int32_t val)
{
    s->y_end = val & 0x07ffff80;
    s->y_end_f = (uint32_t)val & 0x007fff80;
    s->y_end_int = (int16_t)(val >> 11);
    s->xy_end_i = (s->xy_end_i & 0xffff0000) |
                   (uint16_t)s->y_end_int;
}

/* rwpacked pixel limit per host word — MAME ref: newport.cpp:3358-3365 */
static const int16_t rwpacked_max_len[2][4] = {
    { 4, 4, 2, 1 },  /* rwdouble=0: 4@4bpp, 4@8bpp, 2@16bpp, 1@32bpp */
    { 8, 8, 4, 2 }   /* rwdouble=1 */
};

/*
 * Block fill — nested loop over Y then X.
 * The PROM's most-used operation (screen clear, rectangle fill).
 * MAME ref: newport.cpp:3418-3492
 */
static void newport_draw_block(SGINewportState *s)
{
    int16_t start_x = s->x_start_int;
    int16_t start_y = s->y_start_int;
    int16_t end_x = s->x_end_int;
    int16_t end_y = s->y_end_int;
    uint8_t octant = (s->bres_octant_inc1 >> 24) & 7;
    int16_t dx = (octant & 2) ? -1 : 1;
    int16_t dy = (octant & 1) ? -1 : 1;
    uint32_t color;
    int16_t sx, sy;
    int16_t prim_end_x;
    bool stop_on_x = s->dm0_stoponx;
    bool stop_on_y = s->dm0_stopony;
    uint32_t pattern;
    uint32_t pat_bit;
    bool shade = !!(s->drawmode0 & DM0_SHADE);
    bool lr_abort = !!(s->drawmode0 & DM0_LR_ABORT) && dx < 0;

    trace_sgi_newport_draw_block(start_x, start_y, end_x, end_y);
    color = newport_get_default_color(s);

    /* Select pattern source — MAME ref: newport.cpp:3444-3445 */
    if (s->dm0_zpattern) {
        pattern = s->z_pattern;
    } else if (s->dm0_lspattern) {
        pattern = s->ls_pattern;
    } else {
        pattern = 0xffffffff; /* All pixels pass */
    }

    /* End is inclusive — add one step to make it exclusive */
    end_x += dx;
    end_y += dy;
    prim_end_x = end_x;

    /* LENGTH32: clamp span to 32 pixels — MAME ref: line 3427 */
    if ((s->drawmode0 & DM0_LENGTH32) && abs(end_x - start_x) >= 32) {
        prim_end_x = start_x + 32 * dx;
    }

    /* rwpacked limit: clamp pixels per host word */
    if (s->dm0_colorhost && s->dm1_rwpacked) {
        stop_on_x = true;
        int16_t ml = rwpacked_max_len[s->dm1_rwdouble ? 1 : 0]
                                     [s->dm1_hostdepth];
        if (abs(prim_end_x - start_x) > ml) {
            prim_end_x = start_x + dx * ml;
        }
    }

    /*
     * Block fill outer/inner loop — matches MAME structure exactly.
     *
     * MAME uses start_x/start_y directly (modified in-place). We use
     * separate sx/sy variables but follow the same flow:
     * - Inner loop advances sx until prim_end_x, end_x, or !stop_on_x
     * - Y only advances when X completes a full row (reaches end_x)
     * - sx is NOT reset at the top of the outer loop; it carries forward
     *   so that partial rows (rwpacked, LENGTH32) continue where they
     *   left off on the next host data write.
     *
     * MAME ref: newport.cpp lines 3449-3487
     */
    sx = start_x;
    sy = start_y;
    do {
        pat_bit = 31; /* Reset pattern bit — MAME ref: line 3451 */
        do {
            if (lr_abort) {
                break;
            }
            if (shade) {
                newport_iterate_shade(s);
            }
            if (pattern & (1U << pat_bit)) {
                if (s->dm0_colorhost) {
                    newport_output_pixel(s, sx, sy,
                                         newport_get_host_color(s));
                } else if ((shade || s->dm1_rgbmode) && !s->dm1_fastclear) {
                    newport_output_pixel(s, sx, sy,
                                         newport_get_rgb_color(s));
                } else {
                    newport_output_pixel(s, sx, sy, color);
                }
            } else if (s->dm0_opaque) {
                newport_output_pixel(s, sx, sy, s->color_back);
            }
            pat_bit = (pat_bit - 1) & 0x1f;
            sx += dx;
        } while (sx != prim_end_x && sx != end_x && stop_on_x);

        /*
         * Y advance is conditional on X reaching end of row.
         * This is critical for rwpacked/LENGTH32 modes where the X server
         * writes one host word at a time: each write draws a partial row
         * (e.g. 8 pixels), and the hardware must NOT advance Y until the
         * full row is complete.
         * MAME ref: lines 3478-3486
         */
        if ((dx > 0 && sx >= end_x) || (dx < 0 && sx <= end_x) || lr_abort) {
            newport_reset_curr_colors(s);
            sx = s->x_save_int;
            sy += dy;
            /* Recalculate prim_end_x for the new row */
            prim_end_x = end_x;
            if ((s->drawmode0 & DM0_LENGTH32) && abs(end_x - sx) >= 32) {
                prim_end_x = sx + 32 * dx;
            }
            if (s->dm0_colorhost && s->dm1_rwpacked) {
                int16_t ml = rwpacked_max_len[s->dm1_rwdouble ? 1 : 0]
                                             [s->dm1_hostdepth];
                if (abs(prim_end_x - sx) > ml) {
                    prim_end_x = sx + dx * ml;
                }
            }
        }
    } while (sy != end_y && stop_on_y);

    /* Update coordinate registers — MAME ref: lines 3489-3490 */
    s->iter_x = sx;
    s->iter_y = sy;
    newport_write_x_start(s, (int32_t)sx << 11);
    newport_write_y_start(s, (int32_t)sy << 11);
}

/*
 * Span drawing — single scanline, same as block but no Y iteration.
 * Used for text rendering with host data.
 * MAME ref: newport.cpp:3348-3415
 */
static void newport_draw_span(SGINewportState *s)
{
    int16_t start_x = s->x_start_int;
    int16_t end_x = s->x_end_int;
    int16_t y = s->y_start_int;
    uint8_t octant = (s->bres_octant_inc1 >> 24) & 7;
    int16_t dx = (octant & 2) ? -1 : 1;
    uint32_t color;
    int16_t sx;
    int16_t prim_end_x;
    bool stop_on_x = s->dm0_stoponx;
    uint32_t pattern;
    uint32_t pat_bit = 31;
    bool shade = !!(s->drawmode0 & DM0_SHADE);
    bool lr_abort = !!(s->drawmode0 & DM0_LR_ABORT) && dx < 0;

    color = newport_get_default_color(s);

    /* Select pattern source — MAME ref: newport.cpp:3379-3380 */
    if (s->dm0_zpattern) {
        pattern = s->z_pattern;
    } else if (s->dm0_lspattern) {
        pattern = s->ls_pattern;
    } else {
        pattern = 0xffffffff; /* All pixels pass */
    }

    prim_end_x = end_x + dx;

    /* LENGTH32: clamp span to 32 pixels — MAME ref: line 3355 */
    if ((s->drawmode0 & DM0_LENGTH32) && abs(end_x - start_x) > 32) {
        prim_end_x = start_x + 32 * dx;
    }

    /* rwpacked limit: clamp pixels per host word */
    if (s->dm0_colorhost && s->dm1_rwpacked) {
        stop_on_x = true;
        int16_t ml = rwpacked_max_len[s->dm1_rwdouble ? 1 : 0]
                                     [s->dm1_hostdepth];
        if (abs(prim_end_x - start_x) > ml) {
            prim_end_x = start_x + dx * ml;
        }
    }

    sx = start_x;
    do {
        if (lr_abort) {
            break;
        }
        if (shade) {
            newport_iterate_shade(s);
        }
        if (pattern & (1U << pat_bit)) {
            if (s->dm0_colorhost) {
                newport_output_pixel(s, sx, y, newport_get_host_color(s));
            } else if ((shade || s->dm1_rgbmode) && !s->dm1_fastclear) {
                newport_output_pixel(s, sx, y, newport_get_rgb_color(s));
            } else {
                newport_output_pixel(s, sx, y, color);
            }
        } else if (s->dm0_opaque) {
            newport_output_pixel(s, sx, y, s->color_back);
        }
        pat_bit = (pat_bit - 1) & 0x1f;
        sx += dx;
    } while (sx != prim_end_x && stop_on_x);

    if ((dx > 0 && sx >= end_x + dx) || (dx < 0 && sx <= end_x + dx) ||
        lr_abort) {
        newport_reset_curr_colors(s);
    }

    s->iter_x = sx;
    s->iter_y = y;

    /* Update X coordinate — span only updates X, not Y.
     * MAME ref: line 3414 — write_x_start(start_x << 11) */
    newport_write_x_start(s, (int32_t)sx << 11);
}

/*
 * Integer line drawing (Bresenham).
 * MAME ref: do_iline() at newport.cpp:2944-3069
 */
static void newport_draw_iline(SGINewportState *s)
{
    int16_t x0 = s->x_start_int;
    int16_t y0 = s->y_start_int;
    int16_t x1 = s->x_end_int;
    int16_t y1 = s->y_end_int;
    uint32_t color = newport_get_default_color(s);
    uint8_t octant = (s->bres_octant_inc1 >> 24) & 7;
    bool skip_first = !!(s->drawmode0 & DM0_SKIPFIRST);
    bool skip_last  = !!(s->drawmode0 & DM0_SKIPLAST);
    bool shade = !!(s->drawmode0 & DM0_SHADE);
    bool first = true;

    trace_sgi_newport_draw_line(x0, y0, x1, y1, octant);
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int e2;

    for (;;) {
        bool is_last = (x0 == x1 && y0 == y1);
        bool skip = (first && skip_first) || (is_last && skip_last);

        if (!skip) {
            if (shade) {
                newport_iterate_shade(s);
            }
            if (s->dm0_colorhost) {
                newport_output_pixel(s, x0, y0, newport_get_host_color(s));
            } else if (shade && !s->dm1_fastclear) {
                newport_output_pixel(s, x0, y0, newport_get_rgb_color(s));
            } else {
                newport_output_pixel(s, x0, y0, color);
            }
        }
        first = false;
        if (is_last) {
            break;
        }
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    s->iter_x = x0;
    s->iter_y = y0;
    newport_write_x_start(s, (int32_t)x0 << 11);
    newport_write_y_start(s, (int32_t)y0 << 11);
}

/*
 * Fractional line drawing — uses fractional X/Y start/end registers.
 * For this implementation, convert to integer and use Bresenham.
 * MAME ref: do_fline() at newport.cpp:2741-2942
 */
static void newport_draw_fline(SGINewportState *s)
{
    int16_t x0 = (int16_t)(s->x_start >> 16);
    int16_t y0 = (int16_t)(s->y_start >> 16);
    int16_t x1 = (int16_t)(s->x_end >> 16);
    int16_t y1 = (int16_t)(s->y_end >> 16);
    uint32_t color = newport_get_default_color(s);
    bool skip_first = !!(s->drawmode0 & DM0_SKIPFIRST);
    bool skip_last  = !!(s->drawmode0 & DM0_SKIPLAST);
    bool shade = !!(s->drawmode0 & DM0_SHADE);
    bool first = true;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int e2;

    for (;;) {
        bool is_last = (x0 == x1 && y0 == y1);
        bool skip = (first && skip_first) || (is_last && skip_last);

        if (!skip) {
            if (shade) {
                newport_iterate_shade(s);
            }
            if (s->dm0_colorhost) {
                newport_output_pixel(s, x0, y0, newport_get_host_color(s));
            } else if (shade && !s->dm1_fastclear) {
                newport_output_pixel(s, x0, y0, newport_get_rgb_color(s));
            } else {
                newport_output_pixel(s, x0, y0, color);
            }
        }
        first = false;
        if (is_last) {
            break;
        }
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    s->iter_x = x0;
    s->iter_y = y0;
    newport_write_x_start(s, (int32_t)x0 << 11);
    newport_write_y_start(s, (int32_t)y0 << 11);
}

/*
 * Screen-to-screen copy.
 * Read pixel from (start_x, start_y) with window offset, write to
 * (start_x + x_move, start_y + y_move) via output_pixel (which applies
 * window offset).
 * MAME ref: newport.cpp:3510-3548
 */
static void newport_draw_scr2scr(SGINewportState *s)
{
    int16_t start_x = s->x_start_int;
    int16_t start_y = s->y_start_int;
    int16_t end_x = s->x_end_int;
    int16_t end_y = s->y_end_int;
    int16_t move_x = (int16_t)((s->xy_move >> 16) & 0xffff);
    int16_t move_y = (int16_t)(s->xy_move & 0xffff);
    uint8_t octant = (s->bres_octant_inc1 >> 24) & 7;
    int16_t dx = (octant & 2) ? -1 : 1;
    int16_t dy = (octant & 1) ? -1 : 1;
    bool stop_on_x = s->dm0_stoponx;
    bool stop_on_y = s->dm0_stopony;
    int src_wx, src_wy;
    uint32_t src_addr, pixel;

    end_x += dx;
    end_y += dy;

    do {
        do {
            /*
             * Read source pixel at (start_x + window_offset).
             * MAME ref: line 3530 — reads from start_x/start_y with window.
             */
            src_wx = start_x +
                     (int16_t)((s->xy_window >> 16) & 0xffff) - 0x1000;
            src_wy = start_y +
                     (int16_t)(s->xy_window & 0xffff) - 0x1000;

            if (src_wx >= 0 && src_wx < NEWPORT_VRAM_W &&
                src_wy >= 0 && src_wy < NEWPORT_VRAM_H) {
                src_addr = (uint32_t)src_wy * NEWPORT_VRAM_W +
                           (uint32_t)src_wx;
                pixel = s->vram_rgbci[src_addr];
            } else {
                pixel = 0;
            }

            /*
             * Write destination at (start_x + move, start_y + move).
             * output_pixel applies window offset internally.
             * MAME ref: line 3533 — output_pixel(start_x + x_move, ...)
             */
            newport_output_pixel(s, start_x + move_x, start_y + move_y,
                                 pixel);

            start_x += dx;
        } while (start_x != end_x && stop_on_x);

        if (start_x == end_x) {
            start_x = s->x_save_int;
            start_y += dy;
        }
    } while (start_y != end_y && stop_on_y);

    newport_write_x_start(s, (int32_t)start_x << 11);
    newport_write_y_start(s, (int32_t)start_y << 11);
}

/*
 * Read a single pixel from VRAM at current (x_start_i, y_start_i),
 * advance position, and return the pixel value.
 * MAME ref: do_pixel_read() at newport.cpp:3071-3136
 */
static uint32_t newport_read_one_pixel(SGINewportState *s)
{
    int wx, wy;
    uint32_t ret = 0;
    int16_t dy;

    wx = s->x_start_int +
         (int16_t)((s->xy_window >> 16) & 0xffff) - 0x1000;
    wy = s->y_start_int +
         (int16_t)(s->xy_window & 0xffff) - 0x1000;

    if (wx >= 0 && wx < NEWPORT_VRAM_W && wy >= 0 && wy < NEWPORT_VRAM_H) {
        uint32_t addr = (uint32_t)wy * NEWPORT_VRAM_W + (uint32_t)wx;
        if (s->dm1_planes == 4 || s->dm1_planes == 5) {
            ret = s->vram_cidaux[addr];
        } else {
            ret = s->vram_rgbci[addr];
        }
    }

    /*
     * Convert from draw (plane) depth to host depth when they differ.
     * Only for RGB/CI planes (plane_enable 1 or 2).
     * MAME ref: do_pixel_read() at newport.cpp:3080-3122
     */
    if (s->dm1_planes == 1 || s->dm1_planes == 2) {
        ret = newport_convert_draw_to_host(ret, s->dm1_drawdepth,
                                            s->dm1_hostdepth);
    }

    /* Advance x_start_i, wrap to x_save + y advance at x_end */
    s->x_start_int++;
    dy = (s->y_end_int < s->y_start_int) ? -1 : 1;
    if (s->x_start_int > s->x_end_int) {
        s->y_start_int += dy;
        s->x_start_int = s->x_save_int;
    }

    newport_write_x_start(s, (int32_t)s->x_start_int << 11);
    newport_write_y_start(s, (int32_t)s->y_start_int << 11);

    return ret;
}

/*
 * Pixel word read — READ opcode packs multiple pixels into host_dataport
 * based on hostdepth/rwdouble settings.
 * MAME ref: do_pixel_word_read() at newport.cpp:3138-3210
 */
static void newport_do_pixel_read(SGINewportState *s)
{
    uint16_t x_start = s->x_start_int;
    uint16_t x_end = s->x_end_int;
    uint16_t width = (x_end >= x_start) ? (x_end - x_start + 1) : 1;
    uint64_t ret = 0;
    uint64_t shift;
    uint16_t i;
    uint16_t max_width;

    switch (s->dm1_hostdepth) {
    case 0: /* 4bpp */
        max_width = s->dm1_rwdouble ? 16 : 8;
        if (width > max_width) {
            width = max_width;
        }
        shift = 60;
        for (i = 0; i < width; i++) {
            ret |= (uint64_t)(newport_read_one_pixel(s) & 0x0f) << shift;
            shift -= 4;
        }
        break;

    case 1: /* 8bpp */
        max_width = s->dm1_rwdouble ? 8 : 4;
        if (width > max_width) {
            width = max_width;
        }
        shift = 56;
        for (i = 0; i < width; i++) {
            ret |= (uint64_t)(newport_read_one_pixel(s) & 0xff) << shift;
            shift -= 8;
        }
        break;

    case 2: /* 12bpp */
        max_width = s->dm1_rwdouble ? 4 : 2;
        if (width > max_width) {
            width = max_width;
        }
        shift = 48;
        for (i = 0; i < width; i++) {
            ret |= (uint64_t)(newport_read_one_pixel(s) & 0xfff) << shift;
            shift -= 16;
        }
        break;

    case 3: /* 32bpp */
        max_width = s->dm1_rwdouble ? 2 : 1;
        if (width > max_width) {
            width = max_width;
        }
        shift = 32;
        for (i = 0; i < width; i++) {
            ret |= (uint64_t)newport_read_one_pixel(s) << shift;
            shift -= 32;
        }
        break;
    }

    s->host_dataport = ret;
}

/*
 * Command dispatch — called when a "Go" register is accessed.
 * MAME ref: do_rex3_command() at newport.cpp:3307-3549
 */
static void newport_do_rex3_command(SGINewportState *s)
{
    uint8_t opcode = DM0_OPCODE(s->drawmode0);
    uint8_t adrmode = DM0_ADRMODE(s->drawmode0);

    trace_sgi_newport_rex3_cmd(s->drawmode0, s->drawmode1);

    /* DOSETUP (DM0 bit 5): compute octant from coordinates before draw.
     * MAME ref: lines 3334, 3340, 3516 — called for every command type. */
    if (s->drawmode0 & DM0_DOSETUP) {
        newport_do_setup(s);
    }

    switch (opcode) {
    case DM0_OP_NOOP:
        break;
    case DM0_OP_READ:
        newport_do_pixel_read(s);
        break;
    case DM0_OP_DRAW:
        switch (adrmode) {
        case DM0_ADR_SPAN:
            newport_draw_span(s);
            break;
        case DM0_ADR_BLOCK:
            newport_draw_block(s);
            break;
        case DM0_ADR_ILINE:
            newport_draw_iline(s);
            break;
        case DM0_ADR_FLINE:
            newport_draw_fline(s);
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "newport: unimplemented draw adrmode %d\n", adrmode);
            break;
        }
        break;
    case DM0_OP_SCR2SCR:
        newport_draw_scr2scr(s);
        break;
    }

    s->display_dirty = true;
}

/*
 * ============================================================
 * DCB (Display Control Bus) sub-device handlers
 * ============================================================
 */

/*
 * DCB bus timeout callback — clears BACKBUSY for non-existent devices.
 * MAME ref: newport.cpp uses 1ms timer for PCD timeout.
 */
static void newport_dcb_timeout(void *opaque)
{
    SGINewportState *s = SGI_NEWPORT(opaque);
    s->status &= ~REX3_STATUS_BACKBUSY;
}

/*
 * VRINT deassert callback — lowers the IRQ line and re-enables
 * VRINT generation for the next frame.
 */
static void newport_vrint_deassert(void *opaque)
{
    SGINewportState *s = SGI_NEWPORT(opaque);
    s->vrint_active = false;
    qemu_irq_lower(s->irq);
}

/*
 * VBLANK timer callback — fires at 60Hz.
 * If VC2 vertical interrupt is enabled (DC_CONTROL bit 0), sets
 * STATUS_VRINT and asserts the IRQ line for a brief VBLANK period.
 *
 * MAME ref: vc2_device::vblank_w() at newport.cpp:816,
 *           newport_base_device::vrint_w() at line 1700.
 *
 * On real hardware, the VRINT signal asserts at the start of vertical
 * blanking (~40 scanlines, roughly 500us at 60Hz/1024 lines) then
 * deasserts when active video resumes. The IRIX ng1 kernel driver
 * handles the interrupt by reading INT3 local1_stat (bit 7), toggling
 * the mask bit — it does NOT read REX3 STATUS to deassert the GIO
 * interrupt. The hardware deasserts it on its own when VBLANK ends.
 *
 * We model this by asserting the IRQ, then scheduling a deassert after
 * a VBLANK duration. The flag vrint_active prevents re-asserting while
 * the current VBLANK pulse is still active.
 */
static void newport_vblank_timer(void *opaque)
{
    SGINewportState *s = SGI_NEWPORT(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if ((s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_VINTR) &&
        !s->vrint_active) {
        s->status |= REX3_STATUS_VRINT;
        s->vrint_active = true;
        qemu_irq_raise(s->irq);

        /*
         * Schedule IRQ deassert after ~500us (VBLANK duration).
         * Real hardware: ~40 scanlines at ~15.7kHz line rate ≈ 2.5ms.
         * Use 500us as a practical balance — long enough for the CPU
         * to see the interrupt, short enough to not overlap next frame.
         */
        timer_mod(s->vrint_deassert_timer,
                  now + 500 * 1000);  /* 500us */
    }

    /* Re-arm at 60Hz */
    timer_mod(s->vblank_timer, now + NANOSECONDS_PER_SECOND / 60);
}

/*
 * Handle DCB read from sub-devices.
 * The DCB connects REX3 to VC2, XMAP, CMAP, and RAMDAC.
 */
static uint32_t newport_dcb_read(SGINewportState *s)
{
    uint32_t slave = (s->dcb_mode >> DCB_MODE_CSADDR_SHIFT) & 0xf;
    uint32_t reg = (s->dcb_mode >> DCB_MODE_REGSEL_SHIFT) & 0x7;
    uint32_t val = 0;

    switch (slave) {
    case DCB_ADDR_VC2:
        switch (reg) {
        case 0: /* VC2 register index */
            val = s->vc2_reg_idx;
            break;
        case 1: /* VC2 register data */
            if (s->vc2_reg_idx < 32) {
                val = (uint32_t)s->vc2_reg[s->vc2_reg_idx] << 16;
            }
            break;
        case 2: /* VC2 RAM address */
            val = s->vc2_ram_addr << 16;
            break;
        case 3: /* VC2 RAM data — auto-increment */
            if (s->vc2_ram_addr < 32768) {
                val = (uint32_t)s->vc2_ram[s->vc2_ram_addr] << 16;
            }
            s->vc2_ram_addr = (s->vc2_ram_addr + 1) & 0x7fff;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_CMAP0:
    case DCB_ADDR_CMAP1:
    case DCB_ADDR_CMAP01:
        switch (reg) {
        case 0: /* Palette address */
            val = (uint32_t)s->cmap_palette_idx << 16;
            break;
        case 1: /* Palette data read */
            if (s->cmap_palette_idx < 8192) {
                val = s->cmap0_palette[s->cmap_palette_idx] << 8;
            }
            s->cmap_palette_idx++;
            break;
        case 4: /* Status — MAME ref: cmap::read() CRS=4, reset value=8 */
            val = 0x8;
            break;
        case 6: /* Revision — MAME ref: cmap::read() CRS=6 */
            val = s->cmap_revision;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_XMAP0:
    case DCB_ADDR_XMAP1:
    case DCB_ADDR_XMAP01:
        switch (reg) {
        case 0: /* Config */
            val = s->xmap_config;
            break;
        case 1: /* Revision */
            val = s->xmap_revision;
            break;
        case 2: /* FIFO availability */
            val = 2; /* FIFO has space */
            break;
        case 3: /* Cursor CMAP MSB — MAME ref: xmap9::read() CRS=3 */
            val = s->xmap_cursor_cmap;
            break;
        case 4: /* Popup CMAP MSB — MAME ref: xmap9::read() CRS=4 */
            val = s->xmap_popup_cmap;
            break;
        case 5: { /* Mode table read — byte-indexed via CRS 7 */
            uint8_t mode_idx = (s->xmap_mode_table_idx & 0x7c) >> 2;
            uint8_t byte_sel = s->xmap_mode_table_idx & 3;
            uint32_t entry = s->xmap_mode_table[mode_idx];
            switch (byte_sel) {
            case 0: val = (uint8_t)(entry >> 16); break;
            case 1: val = (uint8_t)(entry >> 8); break;
            case 2: val = (uint8_t)entry; break;
            default: val = 0; break;
            }
            break;
        }
        case 7: /* Mode table address — MAME ref: xmap9::read() CRS=7 */
            val = s->xmap_mode_table_idx;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_RAMDAC:
        val = 0;
        break;

    case DCB_ADDR_CC1:
    case DCB_ADDR_AB1:
    case DCB_ADDR_I2C:
        /* Optional devices not present on Indy */
        val = 0;
        break;

    case DCB_ADDR_PCD:
        /* Presenter — not present on Indy, use timer to clear BACKBUSY */
        s->status |= REX3_STATUS_BACKBUSY;
        timer_mod(s->dcb_timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 1000);
        val = 0;
        break;

    default:
        /* Unknown slaves return last written MSW — MAME ref: newport.cpp:2055-2058 */
        val = s->dcb_data_msw;
        break;
    }

    /* Auto-increment register select if enabled */
    if (s->dcb_mode & DCB_MODE_ENCRSINC) {
        uint32_t new_reg = (((s->dcb_mode >> DCB_MODE_REGSEL_SHIFT) & 0x7) + 1) & 0x7;
        s->dcb_mode = (s->dcb_mode & ~DCB_MODE_REGSEL_MASK) |
                      (new_reg << DCB_MODE_REGSEL_SHIFT);
    }

    return val;
}

/*
 * Convert packed RGB pixel to 24-bit RGB based on pixel size.
 *
 * Newport RGB modes pack R, G, B into fewer than 24 bits for small
 * pixel sizes.  The BIT_SEL field (bit 0 of the XMAP mode entry)
 * selects which portion of the 32-bit VRAM word contains the pixel.
 *
 * MAME ref: convert_{4,8,12}bpp_bgr_to_24bpp_rgb()
 */
static uint32_t newport_rgb_unpack(uint32_t pixel, uint8_t pix_size,
                                   uint32_t mode_entry)
{
    /* BIT_SEL: selects upper or lower portion of VRAM word */
    int bit_sel = mode_entry & 1;

    switch (pix_size) {
    case 0: { /* 4bpp — 1-2-1 BGR */
        int shift = bit_sel ? 4 : 0;
        uint8_t p = (pixel >> shift) & 0xf;
        uint8_t r = (p & 1) ? 0xff : 0x00;
        uint8_t g = ((p >> 1) & 3) * 0x55;
        uint8_t b = (p & 8) ? 0xff : 0x00;
        return (r << 16) | (g << 8) | b;
    }
    case 1: { /* 8bpp — 3-3-2 BGR */
        int shift = bit_sel ? 8 : 0;
        uint8_t p = (pixel >> shift) & 0xff;
        uint8_t r = (0x92 * ((p >> 2) & 1)) |
                    (0x49 * ((p >> 1) & 1)) |
                    (0x24 * (p & 1));
        uint8_t g = (0x92 * ((p >> 5) & 1)) |
                    (0x49 * ((p >> 4) & 1)) |
                    (0x24 * ((p >> 3) & 1));
        uint8_t b = (0xaa * ((p >> 7) & 1)) |
                    (0x55 * ((p >> 6) & 1));
        return (r << 16) | (g << 8) | b;
    }
    case 2: { /* 12bpp — 4-4-4 BGR */
        int shift = bit_sel ? 12 : 0;
        uint16_t p = (pixel >> shift) & 0xfff;
        uint8_t r = (p & 0xf) * 0x11;
        uint8_t g = ((p >> 4) & 0xf) * 0x11;
        uint8_t b = ((p >> 8) & 0xf) * 0x11;
        return (r << 16) | (g << 8) | b;
    }
    default: { /* 24bpp — full BGR bytes in VRAM word */
        uint8_t r = pixel & 0xff;
        uint8_t g = (pixel >> 8) & 0xff;
        uint8_t b = (pixel >> 16) & 0xff;
        return (r << 16) | (g << 8) | b;
    }
    }
}

/*
 * Handle DCB write to sub-devices.
 */
static void newport_dcb_write(SGINewportState *s, uint32_t val)
{
    uint32_t slave = (s->dcb_mode >> DCB_MODE_CSADDR_SHIFT) & 0xf;
    uint32_t reg = (s->dcb_mode >> DCB_MODE_REGSEL_SHIFT) & 0x7;
    uint32_t dw = s->dcb_mode & DCB_MODE_DATAWIDTH_MASK;

    /*
     * DCB data width masking.
     *
     * MAME ref: REX3 m_dcb_mask passed to slave write() as mem_mask.
     * In MAME, each slave receives the FULL 32-bit data and the mask
     * separately — the slave uses the mask to select its write mode.
     *
     * VC2 uses the full 32-bit word for ALL operations (combined writes
     * have reg index in bits [31:24], SRAM/register data writes have
     * data in bits [31:16]). The data width selects the VC2 write MODE,
     * not which bits are valid. Applying the mask would destroy data.
     *
     * CMAP needs masking: the CPU uses halfword stores to DCBDATA0 which
     * may contain stale bytes from prior VC2 operations. With dw=1 only
     * bits [7:0] should reach the CMAP address register.
     *
     * XMAP similarly only uses the lower byte for most register writes.
     *
     * dw=3 maps to 0xffffffff (matching MAME): used for VC2 combined
     * register+data writes where reg index is in bits [31:24].
     */
    static const uint32_t dw_mask[4] = {
        0xffffffff,  /* 0 = 4 bytes */
        0x000000ff,  /* 1 = 1 byte  */
        0x0000ffff,  /* 2 = 2 bytes */
        0xffffffff,  /* 3 = full pass-through per MAME */
    };
    uint32_t masked_val = val & dw_mask[dw];

    trace_sgi_newport_dcb_write(slave, val);

    switch (slave) {
    case DCB_ADDR_VC2:
        /*
         * VC2 receives FULL unmasked value.
         *
         * MAME ref: vc2_device::write() dispatches by mem_mask:
         *   mask=0x000000ff (dw=1): 1-byte register select, data in [7:0]
         *   mask=0x0000ffff (dw=2): 2-byte SRAM/reg write, data in [15:0]
         *   mask=0xffffffff (dw=0/3): 4-byte combined write, layout varies
         *
         * Our code dispatches by CRS (reg field) instead of mask, but
         * must still extract 16-bit data from the correct position:
         *   dw=2: CPU halfword at byte_offset=2 → data in bits [15:0]
         *   dw=0/3: CPU fullword → data in bits [31:16] (or [31:24]+[23:8])
         */
        trace_sgi_newport_vc2(reg, val, 1);
        {
            /*
             * Extract 16-bit data from VC2 value based on DCB data width.
             * MAME ref: mask=0x0000ffff uses (uint16_t)data (lower 16),
             *           mask=0xffffffff uses (data >> 16) (upper 16).
             */
            uint16_t vc2_data = (dw == 2) ? (uint16_t)val
                                          : (uint16_t)(val >> 16);
            switch (reg) {
            case 0:
                if (dw == 1) {
                    /*
                     * 1-byte register select.
                     * MAME: mem_mask=0x000000ff → m_reg_idx = data & 0xff
                     */
                    s->vc2_reg_idx = val & 0x1f;
                } else {
                    /*
                     * Combined register index + data write (dw=0/3).
                     * val bits [31:24] = register index
                     * val bits [23:8]  = 16-bit register data
                     * MAME ref: mem_mask=0xffffffff, offset=0
                     */
                    s->vc2_reg_idx = (val >> 24) & 0x1f;
                    s->vc2_reg[s->vc2_reg_idx] = (val >> 8) & 0xffff;
                    if (s->vc2_reg_idx == VC2_RAM_ADDR) {
                        s->vc2_ram_addr = (val >> 8) & 0x7fff;
                    }
                }
                break;
            case 1: /* Register data write */
                s->vc2_reg_data = val;
                if (s->vc2_reg_idx < 32) {
                    s->vc2_reg[s->vc2_reg_idx] = vc2_data;
                }
                break;
            case 2: /* RAM address */
                s->vc2_ram_addr = vc2_data & 0x7fff;
                break;
            case 3: /* SRAM data write — auto-increment */
                if (s->vc2_ram_addr < 32768) {
                    s->vc2_ram[s->vc2_ram_addr] = vc2_data;
                }
                s->vc2_ram_addr = (s->vc2_ram_addr + 1) & 0x7fff;
                break;
            default:
                break;
            }
        }
        break;

    case DCB_ADDR_CMAP0:
    case DCB_ADDR_CMAP1:
    case DCB_ADDR_CMAP01:
        trace_sgi_newport_cmap(s->cmap_palette_idx, masked_val);
        switch (reg) {
        case 0: /* Palette address — MAME ref: cmap::write() CRS=0 */
            s->cmap_palette_idx = (uint16_t)masked_val;
            break;
        case 2: /* Palette data write (MAME CRS=2) */
            trace_sgi_newport_cmap_palette_write(s->cmap_palette_idx,
                                                  val, masked_val,
                                                  masked_val >> 8, dw);
            if (s->cmap_palette_idx < 8192) {
                s->cmap0_palette[s->cmap_palette_idx] = masked_val >> 8;
                s->display_dirty = true;
            }
            s->cmap_palette_idx++;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_XMAP0:
    case DCB_ADDR_XMAP1:
    case DCB_ADDR_XMAP01:
        /* XMAP uses full value: mode table index is in bits [28:24] */
        trace_sgi_newport_xmap(reg, val);
        switch (reg) {
        case 0: /* Config */
            s->xmap_config = val & 0xff;
            break;
        case 3: /* Cursor CMAP MSB — MAME ref: xmap9::write() CRS=3 */
            s->xmap_cursor_cmap = (uint8_t)val;
            break;
        case 4: /* Popup CMAP MSB — MAME ref: xmap9::write() CRS=4 */
            s->xmap_popup_cmap = (uint8_t)val;
            break;
        case 5: /* Mode table write — MAME ref: xmap9::write() CRS=5 */
            /*
             * val bits [28:24] = mode table index
             * val bits [23:0]  = mode entry value
             */
            s->xmap_mode_table[(val >> 24) & 0x1f] = val & 0xffffff;
            break;
        case 7: /* Mode table address — MAME ref: xmap9::write() CRS=7 */
            s->xmap_mode_table_idx = (uint8_t)val;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_RAMDAC:
        switch (reg) {
        case 0: /* LUT index */
            s->ramdac_lut_index = (uint8_t)val;
            break;
        case 1: /* LUT data (RGB packed) */
            /* IRIX Bt445SetRGB() packs as (r << 24) | (g << 16) | (b << 8) */
            s->ramdac_lut_r[s->ramdac_lut_index] = (uint8_t)(val >> 24);
            s->ramdac_lut_g[s->ramdac_lut_index] = (uint8_t)(val >> 16);
            s->ramdac_lut_b[s->ramdac_lut_index] = (uint8_t)(val >> 8);
            s->ramdac_lut_index++;
            break;
        default:
            break;
        }
        break;

    case DCB_ADDR_CC1:
    case DCB_ADDR_AB1:
    case DCB_ADDR_I2C:
        /* Optional devices — ignore writes */
        break;

    case DCB_ADDR_PCD:
        /* Presenter — not present on Indy, use timer to clear BACKBUSY */
        s->status |= REX3_STATUS_BACKBUSY;
        timer_mod(s->dcb_timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 1000);
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "newport: DCB write unknown slave %d reg %d: 0x%08x\n",
                      slave, reg, val);
        break;
    }

    /* Auto-increment register select if enabled */
    if (s->dcb_mode & DCB_MODE_ENCRSINC) {
        uint32_t new_reg = (((s->dcb_mode >> DCB_MODE_REGSEL_SHIFT) & 0x7) + 1) & 0x7;
        s->dcb_mode = (s->dcb_mode & ~DCB_MODE_REGSEL_MASK) |
                      (new_reg << DCB_MODE_REGSEL_SHIFT);
    }
}

/*
 * ============================================================
 * Register decode helpers
 * ============================================================
 */

/*
 * Decode DRAWMODE1 bit fields into cached state for fast access.
 * MAME ref: newport.cpp:3786-3807
 */
static void newport_decode_drawmode1(SGINewportState *s)
{
    uint32_t val = s->drawmode1;
    s->dm1_planes    = val & 7;
    s->dm1_drawdepth = (val >> 3) & 3;
    s->dm1_rwpacked  = (val >> 7) & 1;
    s->dm1_hostdepth = (val >> 8) & 3;
    s->dm1_rwdouble  = (val >> 10) & 1;
    s->dm1_swapendian = (val >> 11) & 1;
    s->dm1_rgbmode   = (val >> 15) & 1;
    s->dm1_fastclear = (val >> 17) & 1;
    s->dm1_blend     = (val >> 18) & 1;
    s->dm1_logicop   = (val >> 28) & 0xf;
}

/*
 * Decode DRAWMODE0 fields.
 */
static void newport_decode_drawmode0(SGINewportState *s)
{
    uint32_t val = s->drawmode0;
    s->dm0_colorhost = (val >> 6) & 1;
    s->dm0_adrmode   = (val >> 2) & 7;
    s->dm0_stoponx   = (val >> 8) & 1;
    s->dm0_stopony   = (val >> 9) & 1;
    s->dm0_zpattern  = (val >> 12) & 1;
    s->dm0_lspattern = (val >> 13) & 1;
    s->dm0_opaque    = ((val >> 16) & 1) || ((val >> 17) & 1);
}

/*
 * ============================================================
 * REX3 register read/write
 * ============================================================
 */

static uint64_t sgi_newport_read(void *opaque, hwaddr addr, unsigned size)
{
    SGINewportState *s = SGI_NEWPORT(opaque);
    uint32_t val = 0;
    bool is_go;
    hwaddr reg;
    unsigned byte_offset;

    /*
     * REX3 registers are 32-bit values. Two consecutive 32-bit registers
     * share each 64-bit word (e.g., DRAWMODE1 at +0 and DRAWMODE0 at +4).
     * Unlike MC/HPC3 which have single 32-bit regs in 64-bit slots,
     * Newport has DIFFERENT registers at +0 and +4 — so we must align
     * to 4-byte boundaries, not 8-byte.
     */
    byte_offset = addr & 3;
    addr &= ~3ULL;

    /*
     * Check for "Go" register access — offset 0x0800-0x0FFF.
     * Reading from Go space triggers a command (for READ opcode).
     * MAME ref: newport.cpp:2166-2169
     */
    is_go = (addr >= REX3_GO_OFFSET && addr < 0x1000);
    reg = is_go ? (addr - REX3_GO_OFFSET) : addr;

    switch (reg) {
    /* Drawing registers */
    case REX3_DRAWMODE0:
        val = s->drawmode0;
        break;
    case REX3_DRAWMODE1:
        val = s->drawmode1;
        break;
    case REX3_LSMODE:
        val = s->ls_mode;
        break;
    case REX3_LSPATTERN:
        val = s->ls_pattern;
        break;
    case REX3_LSPATSAVE:
        val = s->ls_pattern_saved;
        break;
    case REX3_ZPATTERN:
        val = s->z_pattern;
        break;
    case REX3_COLORBACK:
        val = s->color_back;
        break;
    case REX3_COLORVRAM:
        val = s->color_vram;
        break;
    case REX3_ALPHAREF:
        val = s->alpha_ref;
        break;
    case REX3_SMASK0X:
        val = s->smask_x[0];
        break;
    case REX3_SMASK0Y:
        val = s->smask_y[0];
        break;
    case REX3_SETUP:
        val = s->setup;
        break;

    /* Coordinate registers */
    case REX3_XSTART:
        val = s->x_start;
        break;
    case REX3_YSTART:
        val = s->y_start;
        break;
    case REX3_XEND:
        val = s->x_end;
        break;
    case REX3_YEND:
        val = s->y_end;
        break;
    case REX3_XSAVE:
        val = s->x_save;
        break;
    case REX3_XYMOVE:
        val = s->xy_move;
        break;
    case REX3_BRESD:
        val = s->bres_d;
        break;
    case REX3_BRESS1:
        val = s->bres_s1;
        break;
    case REX3_BRESOCTINC1:
        val = s->bres_octant_inc1;
        break;
    case REX3_BRESRNDINC2:
        val = s->bres_round_inc2;
        break;
    case REX3_BRESE1:
        val = s->bres_e1;
        break;
    case REX3_BRESS2:
        val = s->bres_s2;
        break;
    case REX3_AWEIGHT0:
        val = s->a_weight0;
        break;
    case REX3_AWEIGHT1:
        val = s->a_weight1;
        break;
    case REX3_XSTARTF:
        val = s->x_start_f;
        break;
    case REX3_YSTARTF:
        val = s->y_start_f;
        break;
    case REX3_XENDF:
        val = s->x_end_f;
        break;
    case REX3_YENDF:
        val = s->y_end_f;
        break;
    case REX3_XSTARTI:
        val = s->x_start_i;
        break;
    case REX3_XYSTARTI:
        val = s->xy_start_i;
        break;
    case REX3_XYENDI:
        val = s->xy_end_i;
        break;
    case REX3_XSTARTENDI:
        val = s->x_start_end_i;
        break;

    /* Color registers */
    case REX3_COLORRED:
        val = s->color_red;
        break;
    case REX3_COLORALPHA:
        val = s->color_alpha;
        break;
    case REX3_COLORGREEN:
        val = s->color_green;
        break;
    case REX3_COLORBLUE:
        val = s->color_blue;
        break;
    case REX3_SLOPERED:
        val = s->slope_red;
        break;
    case REX3_SLOPEALPHA:
        val = s->slope_alpha;
        break;
    case REX3_SLOPEGREEN:
        val = s->slope_green;
        break;
    case REX3_SLOPEBLUE:
        val = s->slope_blue;
        break;
    case REX3_WRITEMASK:
        val = s->write_mask;
        break;
    case REX3_COLORI:
        val = s->color_i;
        break;
    case REX3_ZEROOVERFLOW:
        val = s->zero_overflow;
        break;
    case REX3_HOSTRW0:
        val = (uint32_t)(s->host_dataport >> 32);
        break;
    case REX3_HOSTRW1:
        val = (uint32_t)(s->host_dataport & 0xffffffff);
        break;

    /* DCB registers */
    case REX3_DCBMODE:
        val = s->dcb_mode;
        break;
    case REX3_DCBDATA0:
        val = newport_dcb_read(s);
        break;
    case REX3_DCBDATA1:
        val = s->dcb_data_lsw;
        break;

    /* Additional registers */
    case REX3_SMASK1X:
        val = s->smask_x[1];
        break;
    case REX3_SMASK1Y:
        val = s->smask_y[1];
        break;
    case REX3_SMASK2X:
        val = s->smask_x[2];
        break;
    case REX3_SMASK2Y:
        val = s->smask_y[2];
        break;
    case REX3_SMASK3X:
        val = s->smask_x[3];
        break;
    case REX3_SMASK3Y:
        val = s->smask_y[3];
        break;
    case REX3_SMASK4X:
        val = s->smask_x[4];
        break;
    case REX3_SMASK4Y:
        val = s->smask_y[4];
        break;
    case REX3_TOPSCAN:
        val = s->top_scanline;
        break;
    case REX3_XYWIN:
        val = s->xy_window;
        break;
    case REX3_CLIPMODE:
        val = s->clip_mode;
        break;
    case REX3_CONFIG:
        val = s->config;
        break;
    case REX3_STATUS:
    case REX3_STATUS_ALIAS:
        /*
         * Status register — critical for PROM progress.
         * Report REX3 version 3 (Indy), all FIFOs empty, not busy.
         * MAME returns same value for both halves of 64-bit word.
         * Reading clears VRINT status bit — MAME ref: newport.cpp:2147-2154
         *
         * Note: we do NOT lower the IRQ here. The IRQ line is managed
         * by the VBLANK timer (assert) and vrint_deassert_timer (lower).
         * The STATUS read only clears the software-visible VRINT flag.
         */
        val = s->status | REX3_VERSION_INDY;
        s->status &= ~REX3_STATUS_VRINT;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "newport: unimplemented read at 0x%04" HWADDR_PRIx "\n",
                      addr);
        val = 0;
        break;
    }

    if (is_go) {
        newport_do_rex3_command(s);
    }

    /*
     * Handle sub-word reads: extract the appropriate byte(s) from the
     * 32-bit register value. The PROM uses lbu/lhu to read specific
     * bytes from Newport registers (e.g., byte 3 of DCBDATA0 to check
     * XMAP FIFO status). For big-endian, byte 0 is the MSB.
     */
    if (size < 4) {
        unsigned shift = (4 - size - (byte_offset & 3)) * 8;
        val = (val >> shift) & ((1U << (size * 8)) - 1);
    }

    trace_sgi_newport_rex3_read((uint64_t)addr, (uint64_t)val);
    newport_newview_log(s, (uint32_t)addr | 0x40000000, val);
    return val;
}

static void sgi_newport_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGINewportState *s = SGI_NEWPORT(opaque);
    bool is_go;
    hwaddr reg;
    unsigned byte_offset = addr & 3;

    /* Normalize address to 32-bit register boundary */
    addr &= ~3ULL;

    trace_sgi_newport_rex3_write((uint64_t)addr, val);
    newport_newview_log(s, (uint32_t)addr, (uint32_t)val);

    /*
     * Check for "Go" register access — offset 0x0800-0x0FFF.
     * Writing to Go space stores the value AND triggers command execution.
     * MAME ref: newport.cpp:4462-4465
     */
    is_go = (addr >= REX3_GO_OFFSET && addr < 0x1000);
    reg = is_go ? (addr - REX3_GO_OFFSET) : addr;

    switch (reg) {
    /* Drawing registers */
    case REX3_DRAWMODE0:
        s->drawmode0 = val;
        newport_decode_drawmode0(s);
        break;
    case REX3_DRAWMODE1:
        s->drawmode1 = val;
        newport_decode_drawmode1(s);
        /* Reset host shift — MAME ref: newport.cpp:3797 */
        s->host_shift = 64 - host_depth_bpp[s->dm1_hostdepth];
        break;
    case REX3_LSMODE:
        s->ls_mode = val & 0x0fffffff;
        break;
    case REX3_LSPATTERN:
        s->ls_pattern = val;
        break;
    case REX3_LSPATSAVE:
        s->ls_pattern_saved = val;
        break;
    case REX3_ZPATTERN:
        s->z_pattern = val;
        break;
    case REX3_COLORBACK:
        s->color_back = val;
        break;
    case REX3_COLORVRAM:
        s->color_vram = val;
        break;
    case REX3_ALPHAREF:
        s->alpha_ref = val & 0xff;
        break;
    case REX3_SMASK0X:
        s->smask_x[0] = val;
        break;
    case REX3_SMASK0Y:
        s->smask_y[0] = val;
        break;
    case REX3_SETUP:
        s->setup = val;
        break;
    case REX3_STEPZ:
        s->step_z = val;
        break;
    case REX3_LSPATUPDATE:
        /*
         * Update trigger — swap ls_pattern and ls_pattern_saved.
         * MAME ref: newport.cpp offset 0x0038/8
         * High word write: ls_pattern = ls_pattern_saved
         * Low word write: ls_pattern_saved = ls_pattern
         * We handle both directions since we normalize to high word.
         */
        s->ls_pattern = s->ls_pattern_saved;
        break;

    /* Coordinate registers — raw fractional format, pass directly */
    case REX3_XSTART:
        newport_write_x_start(s, (int32_t)val);
        s->x_save_int = s->x_start_int;
        break;
    case REX3_YSTART:
        newport_write_y_start(s, (int32_t)val);
        break;
    case REX3_XEND:
        newport_write_x_end(s, (int32_t)val);
        break;
    case REX3_YEND:
        newport_write_y_end(s, (int32_t)val);
        break;
    case REX3_XSAVE:
        s->x_save = val & 0xffff;  /* 16-bit — MAME ref: line 3986 */
        s->x_save_int = (int16_t)val;
        break;
    case REX3_XYMOVE:
        s->xy_move = val;
        break;
    case REX3_BRESD:
        s->bres_d = val & 0x07ffffff;
        break;
    case REX3_BRESS1:
        s->bres_s1 = val & 0x0001ffff;
        break;
    case REX3_BRESOCTINC1:
        s->bres_octant_inc1 = val & 0x070fffff;
        break;
    case REX3_BRESRNDINC2:
        s->bres_round_inc2 = val & 0xff1fffff;
        break;
    case REX3_BRESE1:
        s->bres_e1 = val & 0x0000ffff;
        break;
    case REX3_BRESS2:
        s->bres_s2 = val & 0x03ffffff;
        break;
    case REX3_AWEIGHT0:
        s->a_weight0 = val;
        break;
    case REX3_AWEIGHT1:
        s->a_weight1 = val;
        break;
    case REX3_XSTARTF:
        s->x_start_f = val;
        newport_write_x_start(s, (int32_t)val & 0x007fff80);
        s->x_save_int = s->x_start_int;
        break;
    case REX3_YSTARTF:
        s->y_start_f = val;
        newport_write_y_start(s, (int32_t)val & 0x007fff80);
        break;
    case REX3_XENDF:
        s->x_end_f = val;
        newport_write_x_end(s, (int32_t)val & 0x007fff80);
        break;
    case REX3_YENDF:
        s->y_end_f = val;
        newport_write_y_end(s, (int32_t)val & 0x007fff80);
        break;
    case REX3_XSTARTI:
        newport_write_x_start(s, ((int32_t)(int16_t)val) << 11);
        s->x_save_int = s->x_start_int;
        break;
    case REX3_XYSTARTI:
        s->xy_start_i = val;
        newport_write_x_start(s, ((int32_t)(int16_t)(val >> 16)) << 11);
        newport_write_y_start(s, ((int32_t)(int16_t)(val & 0xffff)) << 11);
        s->x_save_int = s->x_start_int;
        break;
    case REX3_XYENDI:
        s->xy_end_i = val;
        newport_write_x_end(s, ((int32_t)(int16_t)(val >> 16)) << 11);
        newport_write_y_end(s, ((int32_t)(int16_t)(val & 0xffff)) << 11);
        break;
    case REX3_XSTARTENDI:
        s->x_start_end_i = val;
        newport_write_x_start(s, ((int32_t)(int16_t)(val >> 16)) << 11);
        newport_write_x_end(s, ((int32_t)(int16_t)(val & 0xffff)) << 11);
        s->x_save_int = s->x_start_int;
        break;

    /* Color registers — also set curr_color accumulators.
     * MAME ref: newport.cpp line 4112-4113 */
    case REX3_COLORRED:
        s->color_red = val & 0x00ffffff;
        s->curr_color_red = s->color_red;
        break;
    case REX3_COLORALPHA:
        s->color_alpha = val & 0x000fffff;
        s->curr_color_alpha = s->color_alpha;
        break;
    case REX3_COLORGREEN:
        s->color_green = val & 0x000fffff;
        s->curr_color_green = s->color_green;
        break;
    case REX3_COLORBLUE:
        s->color_blue = val & 0x000fffff;
        s->curr_color_blue = s->color_blue;
        break;
    case REX3_SLOPERED:
        s->slope_red = newport_twos_to_sm((uint32_t)val, 24);
        break;
    case REX3_SLOPEALPHA:
        s->slope_alpha = newport_twos_to_sm((uint32_t)val, 20);
        break;
    case REX3_SLOPEGREEN:
        s->slope_green = newport_twos_to_sm((uint32_t)val, 20);
        break;
    case REX3_SLOPEBLUE:
        s->slope_blue = newport_twos_to_sm((uint32_t)val, 20);
        break;
    case REX3_WRITEMASK:
        s->write_mask = val & 0x00ffffff;
        break;
    case REX3_COLORI:
        s->color_i = val;
        break;
    case REX3_ZEROOVERFLOW:
        s->zero_overflow = val;
        break;
    case REX3_HOSTRW0:
        s->host_dataport = ((uint64_t)(uint32_t)val << 32) |
                           (s->host_dataport & 0xffffffffULL);
        /* Reset host shift position on new data write */
        s->host_shift = 64 - host_depth_bpp[s->dm1_hostdepth];
        break;
    case REX3_HOSTRW1:
        s->host_dataport = (s->host_dataport & 0xffffffff00000000ULL) |
                           (uint32_t)val;
        break;
    case REX3_SLOPEREDCOPY:
        s->slope_red = newport_twos_to_sm((uint32_t)val, 24);
        break;

    /* DCB registers */
    case REX3_DCBMODE:
        s->dcb_mode = val;
        break;
    case REX3_DCBDATA0:
        /*
         * Handle sub-word writes: merge written bytes into dcb_data_msw.
         * On big-endian, byte at offset 0 = bits [31:24], offset 3 = bits [7:0].
         * A halfword at offset 0 = bits [31:16], at offset 2 = bits [15:0].
         * A full-word write replaces all 32 bits.
         */
        /*
         * Clear dcb_data_msw before merging sub-word writes.
         *
         * Each write to DCBDATA0 triggers a DCB bus transfer.  The IRIX
         * driver uses byte writes (sb to offset 3) for narrow-width DCB
         * slaves like the CMAP.  Without clearing, stale bytes from
         * previous DCB operations corrupt the slave data.  For example,
         * the CMAP palette address register uses 2-byte DCB width, but
         * the driver writes only the address byte to offset 3 — stale
         * data in byte 2 (from a previous VC2/XMAP operation) would be
         * interpreted as the high byte of the palette index.
         *
         * Since every DCBDATA0 write triggers an immediate DCB transfer,
         * the driver cannot build up multi-byte values across separate
         * writes.  Clearing before merge ensures only the CPU-written
         * byte(s) are non-zero.
         */
        if (size == 4) {
            s->dcb_data_msw = val;
        } else if (size == 2) {
            s->dcb_data_msw = 0;
            if (byte_offset == 0) {
                s->dcb_data_msw = (uint32_t)val << 16;
            } else {
                s->dcb_data_msw = (uint32_t)val & 0xffff;
            }
            trace_sgi_newport_dcbdata0_subword(size, byte_offset,
                                                val, s->dcb_data_msw);
        } else if (size == 1) {
            s->dcb_data_msw = 0;
            uint32_t shift = (3 - byte_offset) * 8;
            s->dcb_data_msw = ((uint32_t)val & 0xff) << shift;
            trace_sgi_newport_dcbdata0_subword(size, byte_offset,
                                                val, s->dcb_data_msw);
        }
        trace_sgi_newport_rex3_write(REX3_DCBDATA0, s->dcb_data_msw);
        newport_dcb_write(s, s->dcb_data_msw);
        break;
    case REX3_DCBDATA1:
        s->dcb_data_lsw = val;
        break;

    /* Additional registers */
    case REX3_SMASK1X:
        s->smask_x[1] = val;
        break;
    case REX3_SMASK1Y:
        s->smask_y[1] = val;
        break;
    case REX3_SMASK2X:
        s->smask_x[2] = val;
        break;
    case REX3_SMASK2Y:
        s->smask_y[2] = val;
        break;
    case REX3_SMASK3X:
        s->smask_x[3] = val;
        break;
    case REX3_SMASK3Y:
        s->smask_y[3] = val;
        break;
    case REX3_SMASK4X:
        s->smask_x[4] = val;
        break;
    case REX3_SMASK4Y:
        s->smask_y[4] = val;
        break;
    case REX3_TOPSCAN:
        s->top_scanline = val & 0x3ff;
        break;
    case REX3_XYWIN:
        s->xy_window = val;
        break;
    case REX3_CLIPMODE:
        s->clip_mode = val & 0x1fff;
        break;
    case REX3_CONFIG:
        s->config = val;
        break;
    case REX3_STATUS:
        /* Status is mostly read-only; writes may clear some bits */
        s->status &= ~(val & (REX3_STATUS_VRINT | REX3_STATUS_VIDEOINT));
        break;
    case REX3_DCBRESET:
        /* Reset DCB bus and flush BFIFO — write-only command, no-op */
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "newport: unimplemented write at 0x%04" HWADDR_PRIx
                      " value 0x%08" PRIx64 "\n", addr, val);
        break;
    }

    if (is_go) {
        newport_do_rex3_command(s);
    }
}

static const MemoryRegionOps sgi_newport_ops = {
    .read = sgi_newport_read,
    .write = sgi_newport_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * ============================================================
 * Framebuffer dump (debugging / OCR)
 * ============================================================
 */

/*
 * Dump raw VRAM through the full compositing pipeline to a PPM file.
 * This bypasses the display surface and writes directly from VRAM,
 * running the same DID/XMAP/CMAP/RAMDAC logic as newport_update_display().
 * Useful for inspecting what's actually in VRAM when the display shows black.
 */
static void newport_dump_vram_ppm(SGINewportState *s, const char *path)
{
    FILE *f;
    int x, y;
    uint16_t did_entry_ptr;
    bool use_did;
    uint16_t popup_msb = (uint16_t)s->xmap_popup_cmap << 5;

    if (!s->vram_rgbci) {
        qemu_log("newport: fb-dump: VRAM not allocated\n");
        return;
    }

    f = fopen(path, "wb");
    if (!f) {
        qemu_log("newport: fb-dump: cannot open '%s': %s\n",
                 path, strerror(errno));
        return;
    }

    fprintf(f, "P6\n%d %d\n255\n", NEWPORT_SCREEN_W, NEWPORT_SCREEN_H);

    did_entry_ptr = s->vc2_reg[VC2_DID_ENTRY];
    use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS)
              && did_entry_ptr != 0;

    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        const uint32_t *src_rgbci = &s->vram_rgbci[y * NEWPORT_VRAM_W];
        const uint32_t *src_cidaux = &s->vram_cidaux[y * NEWPORT_VRAM_W];
        uint8_t pix_mode = 0;
        uint8_t pix_size = 1;
        uint16_t ci_msb = 0;
        uint16_t aux_msb = 0;
        uint32_t mode_entry = 0;
        uint8_t aux_pix_mode = 0;
        uint16_t did_line_ptr = 0;
        uint16_t next_did_entry = 0;

        if (use_did) {
            uint16_t frame_ptr = did_entry_ptr + (uint16_t)y;
            did_line_ptr = s->vc2_ram[frame_ptr & 0x7fff];
            uint16_t entry = s->vc2_ram[did_line_ptr & 0x7fff];
            mode_entry = s->xmap_mode_table[entry & 0x1f];
            pix_mode = (mode_entry >> 8) & 3;
            pix_size = (mode_entry >> 10) & 3;
            aux_pix_mode = (mode_entry >> 16) & 7;
            aux_msb = (mode_entry >> 11) & 0x1f00;
            switch (pix_mode) {
            case 0: ci_msb = (mode_entry & 0xf8) << 5; break;
            case 1: ci_msb = 0x1d00; break;
            case 2: ci_msb = 0x1e00; break;
            case 3: ci_msb = 0x1f00; break;
            }
            did_line_ptr++;
            next_did_entry = s->vc2_ram[did_line_ptr & 0x7fff];
        }

        for (x = 0; x < NEWPORT_SCREEN_W; x++) {
            uint32_t pixel = src_rgbci[x];
            uint32_t cidaux = src_cidaux[x];
            uint32_t rgb;
            uint8_t r, g, b;

            if (use_did && (uint16_t)x == (next_did_entry >> 5)) {
                mode_entry =
                    s->xmap_mode_table[next_did_entry & 0x1f];
                pix_mode = (mode_entry >> 8) & 3;
                pix_size = (mode_entry >> 10) & 3;
                aux_pix_mode = (mode_entry >> 16) & 7;
                aux_msb = (mode_entry >> 11) & 0x1f00;
                switch (pix_mode) {
                case 0: ci_msb = (mode_entry & 0xf8) << 5; break;
                case 1: ci_msb = 0x1d00; break;
                case 2: ci_msb = 0x1e00; break;
                case 3: ci_msb = 0x1f00; break;
                }
                did_line_ptr++;
                next_did_entry = s->vc2_ram[did_line_ptr & 0x7fff];
            }

            if (cidaux & 0xcc) {
                uint8_t popup_ci = (cidaux >> 2) & 3;
                rgb = s->cmap0_palette[(popup_msb | popup_ci) & 0x1fff];
            } else if (aux_pix_mode != 0) {
                bool overlay_hit = false;
                switch (aux_pix_mode) {
                case 1:
                    rgb = s->cmap0_palette[
                        (aux_msb | ((cidaux >> 8) & 3)) & 0x1fff];
                    overlay_hit = true;
                    break;
                case 2: {
                    uint32_t ovl = (cidaux >> 8) & 3;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                        overlay_hit = true;
                    }
                    break;
                }
                case 6: {
                    uint32_t shift = (mode_entry & 2) ? 9 : 8;
                    uint32_t ovl = (cidaux >> shift) & 1;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                        overlay_hit = true;
                    }
                    break;
                }
                case 7: {
                    uint32_t ovl = (cidaux >> 8) & 1;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                    } else {
                        rgb = s->cmap0_palette[
                            (aux_msb | ((cidaux >> 9) & 1)) & 0x1fff];
                    }
                    overlay_hit = true;
                    break;
                }
                default:
                    break;
                }
                if (!overlay_hit) {
                    goto ppm_main_pixel;
                }
            } else {
            ppm_main_pixel:
            if (pix_mode == 0) {
                uint16_t ci;
                switch (pix_size) {
                case 0: ci = pixel & 0xf; break;
                case 1: ci = pixel & 0xff; break;
                case 2: ci = pixel & 0xfff; break;  /* 12bpp CI */
                default: ci = pixel & 0xff; break;
                }
                rgb = s->cmap0_palette[(ci_msb | ci) & 0x1fff];
            } else {
                rgb = newport_rgb_unpack(pixel, pix_size, mode_entry);
            }
            }

            r = s->ramdac_lut_r[(rgb >> 16) & 0xff];
            g = s->ramdac_lut_g[(rgb >> 8) & 0xff];
            b = s->ramdac_lut_b[rgb & 0xff];

            fputc(r, f);
            fputc(g, f);
            fputc(b, f);
        }
    }

    fclose(f);

    /* Also log VRAM statistics for debugging */
    {
        uint32_t non_zero_rgbci = 0, non_zero_cidaux = 0;
        uint32_t first_nz_idx = 0;
        uint32_t first_nz_val = 0;
        for (int i = 0; i < NEWPORT_VRAM_W * NEWPORT_VRAM_H; i++) {
            if (s->vram_rgbci[i]) {
                if (!non_zero_rgbci) {
                    first_nz_idx = i;
                    first_nz_val = s->vram_rgbci[i];
                }
                non_zero_rgbci++;
            }
            if (s->vram_cidaux[i]) {
                non_zero_cidaux++;
            }
        }
        /* Check DID/XMAP/CMAP chain for line 0 */
        uint16_t diag_ci_msb = 0;
        uint8_t diag_pix_mode = 0;
        uint32_t diag_mode_entry = 0;
        uint16_t diag_did_val = 0;
        uint16_t diag_did_line_ptr = 0;
        bool diag_use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS)
                            && s->vc2_reg[VC2_DID_ENTRY] != 0;
        if (diag_use_did) {
            uint16_t fp = s->vc2_reg[VC2_DID_ENTRY];
            diag_did_line_ptr = s->vc2_ram[fp & 0x7fff];
            diag_did_val = s->vc2_ram[diag_did_line_ptr & 0x7fff];
            diag_mode_entry = s->xmap_mode_table[diag_did_val & 0x1f];
            diag_pix_mode = (diag_mode_entry >> 8) & 3;
            switch (diag_pix_mode) {
            case 0: diag_ci_msb = (diag_mode_entry & 0xf8) << 5; break;
            case 1: diag_ci_msb = 0x1d00; break;
            case 2: diag_ci_msb = 0x1e00; break;
            case 3: diag_ci_msb = 0x1f00; break;
            }
        }

        /* Count programmed CMAP entries (non-zero) and find ranges */
        uint32_t cmap_nonzero = 0;
        int cmap_first_nz = -1, cmap_last_nz = -1;
        for (int ci = 0; ci < 8192; ci++) {
            if (s->cmap0_palette[ci]) {
                cmap_nonzero++;
                if (cmap_first_nz < 0) cmap_first_nz = ci;
                cmap_last_nz = ci;
            }
        }

        /* Sample CMAP at the ci_msb offset */
        uint32_t cmap_sample[4];
        for (int ci = 0; ci < 4; ci++) {
            cmap_sample[ci] = s->cmap0_palette[(diag_ci_msb + ci) & 0x1fff];
        }

        qemu_log("newport: fb-dump: wrote %dx%d PPM to '%s'\n"
                 "  VRAM stats: rgbci non-zero=%u/%u cidaux non-zero=%u/%u\n"
                 "  First non-zero RGBCI: idx=%u val=0x%08x (x=%d,y=%d)\n"
                 "  DID_ENTRY=0x%04x DC_CONTROL=0x%04x use_did=%d\n"
                 "  DID line0: ptr=0x%04x val=0x%04x mode_entry=0x%08x\n"
                 "  pix_mode=%d ci_msb=0x%04x\n"
                 "  CMAP: %u/8192 entries non-zero (range %d..%d)\n"
                 "  CMAP[ci_msb+0..3]: 0x%06x 0x%06x 0x%06x 0x%06x\n"
                 "  CMAP[0..3]: 0x%06x 0x%06x 0x%06x 0x%06x\n"
                 "  RAMDAC LUT[0..3]: R=%d,%d,%d,%d\n",
                 NEWPORT_SCREEN_W, NEWPORT_SCREEN_H, path,
                 non_zero_rgbci, NEWPORT_VRAM_W * NEWPORT_VRAM_H,
                 non_zero_cidaux, NEWPORT_VRAM_W * NEWPORT_VRAM_H,
                 first_nz_idx, first_nz_val,
                 first_nz_idx % NEWPORT_VRAM_W, first_nz_idx / NEWPORT_VRAM_W,
                 s->vc2_reg[VC2_DID_ENTRY], s->vc2_reg[VC2_DC_CONTROL],
                 diag_use_did,
                 diag_did_line_ptr, diag_did_val, diag_mode_entry,
                 diag_pix_mode, diag_ci_msb,
                 cmap_nonzero, cmap_first_nz, cmap_last_nz,
                 cmap_sample[0], cmap_sample[1], cmap_sample[2], cmap_sample[3],
                 s->cmap0_palette[0], s->cmap0_palette[1],
                 s->cmap0_palette[2], s->cmap0_palette[3],
                 s->ramdac_lut_r[0], s->ramdac_lut_r[1],
                 s->ramdac_lut_r[2], s->ramdac_lut_r[3]);

        /* Dump all 32 XMAP mode table entries */
        qemu_log("  XMAP mode_table[0..31]:\n");
        for (int i = 0; i < 32; i += 4) {
            qemu_log("    [%2d..%2d]: 0x%06x 0x%06x 0x%06x 0x%06x\n",
                     i, i + 3,
                     s->xmap_mode_table[i], s->xmap_mode_table[i + 1],
                     s->xmap_mode_table[i + 2], s->xmap_mode_table[i + 3]);
        }

        /* Dump DID line pointers for first 4 lines */
        if (diag_use_did) {
            uint16_t fp = s->vc2_reg[VC2_DID_ENTRY];
            qemu_log("  DID line pointers (from 0x%04x):\n", fp);
            for (int ln = 0; ln < 4; ln++) {
                uint16_t lp = s->vc2_ram[(fp + ln) & 0x7fff];
                qemu_log("    line%d: ptr=0x%04x", ln, lp);
                /* Dump DID data at that pointer (up to 8 words) */
                for (int di = 0; di < 8; di++) {
                    uint16_t dw = s->vc2_ram[(lp + di) & 0x7fff];
                    qemu_log(" [%04x]", dw);
                    if (dw == 0) break;
                }
                qemu_log("\n");
            }
        }

        /* Dump CMAP entries at the first non-zero range */
        if (cmap_first_nz >= 0) {
            qemu_log("  CMAP[%d..%d]: ", cmap_first_nz,
                     cmap_first_nz + 7 < 8192 ? cmap_first_nz + 7 : 8191);
            for (int ci = cmap_first_nz; ci < cmap_first_nz + 8 && ci < 8192; ci++) {
                qemu_log("0x%06x ", s->cmap0_palette[ci]);
            }
            qemu_log("\n");
        }

        /* XMAP config and popup/cursor cmap */
        qemu_log("  XMAP config=0x%02x popup_cmap=0x%02x cursor_cmap=0x%02x\n",
                 s->xmap_config, s->xmap_popup_cmap, s->xmap_cursor_cmap);
    }
}

/*
 * ============================================================
 * Display update
 * ============================================================
 */

static void newport_invalidate(void *opaque)
{
    SGINewportState *s = opaque;
    s->display_dirty = true;
}

/*
 * Draw hardware cursor overlay onto the display surface.
 * Supports 32x32 (2-plane, 4-color) and 64x64 (1-plane, monochrome) cursors.
 * MAME ref: get_cursor_pixel() at newport.cpp:782-814
 */
static void newport_draw_cursor(SGINewportState *s, uint32_t *dest)
{
    uint16_t dc = s->vc2_reg[VC2_DC_CONTROL];
    int16_t cx, cy;
    uint16_t cursor_entry;
    bool is_64;
    int size, gx, gy, sx, sy;

    if (!(dc & VC2_DC_ENA_CURSOR) || !(dc & VC2_DC_CURSOR_DISP)) {
        return;
    }

    cx = (int16_t)s->vc2_reg[VC2_CURSOR_X];
    cy = (int16_t)s->vc2_reg[VC2_CURSOR_Y];
    cursor_entry = s->vc2_reg[VC2_CURSOR_ENTRY];
    is_64 = dc & VC2_DC_CURSOR_SIZE64;
    size = is_64 ? 64 : 32;

    for (gy = 0; gy < size; gy++) {
        sy = cy - (size - 1) + gy;
        if (sy < 0 || sy >= NEWPORT_SCREEN_H) {
            continue;
        }

        for (gx = 0; gx < size; gx++) {
            uint8_t pixel;
            int shift;

            sx = cx - (size - 1) + gx;
            if (sx < 0 || sx >= NEWPORT_SCREEN_W) {
                continue;
            }

            shift = 15 - (gx & 15);

            if (is_64) {
                /* 64x64 monochrome: 1 plane, 4 words per row */
                int addr = gy * 4 + (gx / 16);
                uint16_t word = s->vc2_ram[(cursor_entry + addr) & 0x7fff];
                pixel = (word >> shift) & 1;
            } else {
                /* 32x32 two-plane: 2 words per row, plane1 at +64 */
                int addr = gy * 2 + (gx / 16);
                uint16_t w0 = s->vc2_ram[(cursor_entry + addr) & 0x7fff];
                uint16_t w1 = s->vc2_ram[(cursor_entry + addr + 64) & 0x7fff];
                pixel = ((w0 >> shift) & 1) | (((w1 >> shift) & 1) << 1);
            }

            if (pixel) {
                uint16_t cursor_msb = (uint16_t)s->xmap_cursor_cmap << 5;
                uint32_t rgb = s->cmap0_palette[
                    (cursor_msb | (pixel & 0x3)) & 0x1fff];
                uint8_t r = s->ramdac_lut_r[(rgb >> 16) & 0xff];
                uint8_t g = s->ramdac_lut_g[(rgb >> 8) & 0xff];
                uint8_t b = s->ramdac_lut_b[rgb & 0xff];
                dest[sy * NEWPORT_SCREEN_W + sx] = rgb_to_pixel32(r, g, b);
            }
        }
    }
}

/*
 * Convert VRAM contents to display surface pixels.
 * Uses VC2 DID table and XMAP mode entries for proper pixel rendering.
 * Supports per-scanline DID mode changes, overlay/popup planes, and
 * RAMDAC gamma correction.
 * MAME ref: screen_update() at newport.cpp:1282-1529
 */
static void newport_update_display(void *opaque)
{
    SGINewportState *s = opaque;
    DisplaySurface *surface;
    uint32_t *dest;
    int x, y;
    uint16_t did_entry_ptr;
    bool use_did;
    uint16_t popup_msb = (uint16_t)s->xmap_popup_cmap << 5;

    if (!s->display_dirty) {
        return;
    }

    surface = qemu_console_surface(s->con);
    if (!surface) {
        return;
    }

    dest = (uint32_t *)surface_data(surface);

    did_entry_ptr = s->vc2_reg[VC2_DID_ENTRY];
    use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS)
              && did_entry_ptr != 0;

    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        const uint32_t *src_rgbci = &s->vram_rgbci[y * NEWPORT_VRAM_W];
        const uint32_t *src_cidaux = &s->vram_cidaux[y * NEWPORT_VRAM_W];
        uint8_t pix_mode = 0;   /* 0=CI, 1=RGB Map0, 2=Map1, 3=Map2 */
        uint8_t pix_size = 1;   /* 0=4bpp, 1=8bpp, 2=12bpp, 3=24bpp */
        uint16_t ci_msb = 0;
        uint16_t aux_msb = 0;
        uint32_t mode_entry = 0;
        uint8_t aux_pix_mode = 0;
        uint16_t did_line_ptr = 0;
        uint16_t next_did_entry = 0;

        if (use_did) {
            /*
             * DID per-scanline mode: follow frame → line table.
             * Each DID entry: bits [15:5] = X pos of next mode change,
             *                 bits [4:0]  = DID (mode table index).
             * MAME ref: begin_did_line() at newport.cpp:656
             */
            uint16_t frame_ptr = did_entry_ptr + (uint16_t)y;
            did_line_ptr = s->vc2_ram[frame_ptr & 0x7fff];
            uint16_t entry = s->vc2_ram[did_line_ptr & 0x7fff];
            mode_entry = s->xmap_mode_table[entry & 0x1f];
            pix_mode = (mode_entry >> 8) & 3;
            pix_size = (mode_entry >> 10) & 3;
            aux_pix_mode = (mode_entry >> 16) & 7;
            aux_msb = (mode_entry >> 11) & 0x1f00;
            switch (pix_mode) {
            case 0: ci_msb = (mode_entry & 0xf8) << 5; break;
            case 1: ci_msb = 0x1d00; break;
            case 2: ci_msb = 0x1e00; break;
            case 3: ci_msb = 0x1f00; break;
            }
            /* Prepare next DID entry — MAME ref: next_did_line_entry() */
            did_line_ptr++;
            next_did_entry = s->vc2_ram[did_line_ptr & 0x7fff];
        }

        for (x = 0; x < NEWPORT_SCREEN_W; x++) {
            uint32_t pixel = src_rgbci[x];
            uint32_t cidaux = src_cidaux[x];
            uint32_t rgb;
            uint8_t r, g, b;

            /*
             * Check for DID mode change at this X position.
             * MAME ref: screen_update() at newport.cpp:1298-1312
             */
            if (use_did && (uint16_t)x == (next_did_entry >> 5)) {
                mode_entry =
                    s->xmap_mode_table[next_did_entry & 0x1f];
                pix_mode = (mode_entry >> 8) & 3;
                pix_size = (mode_entry >> 10) & 3;
                aux_pix_mode = (mode_entry >> 16) & 7;
                aux_msb = (mode_entry >> 11) & 0x1f00;
                switch (pix_mode) {
                case 0: ci_msb = (mode_entry & 0xf8) << 5; break;
                case 1: ci_msb = 0x1d00; break;
                case 2: ci_msb = 0x1e00; break;
                case 3: ci_msb = 0x1f00; break;
                }
                did_line_ptr++;
                next_did_entry = s->vc2_ram[did_line_ptr & 0x7fff];
            }

            /*
             * Compositing priority (MAME ref: screen_update lines 1338-1438):
             *  1. Cursor pixel (handled in newport_draw_cursor post-loop)
             *  2. Popup planes (cidaux bits 3,2,6,7 → popup_msb lookup)
             *  3. Overlay/aux planes (underlay 1,7 always; overlay 2-6
             *     only when aux pixel is non-zero — zero is transparent)
             *  4. Main RGBCI pixel
             *
             * XL8 cidaux bit layout (8-bit mode, xmap_config bit 2):
             *   [3:0] = CID (per-window visual mode tag)
             *   [7:4] = popup planes (bits 2,3) + reserved (bits 6,7)
             *   [9:8] = overlay/underlay planes
             */
            if (cidaux & 0xcc) {
                /* Popup planes active — MAME ref: lines 1343-1347 */
                uint8_t popup_ci = (cidaux >> 2) & 3;
                rgb = s->cmap0_palette[(popup_msb | popup_ci) & 0x1fff];
            } else if (aux_pix_mode != 0) {
                /*
                 * Overlay/underlay compositing — per-mode handling.
                 * XL8 (8-bit mode): overlay bits at cidaux[9:8].
                 * MAME ref: newport.cpp lines 1350-1393
                 */
                bool overlay_hit = false;
                switch (aux_pix_mode) {
                case 1: /* 2-Bit Underlay — always drawn */
                    rgb = s->cmap0_palette[
                        (aux_msb | ((cidaux >> 8) & 3)) & 0x1fff];
                    overlay_hit = true;
                    break;
                case 2: /* 2-Bit Overlay — zero is transparent */ {
                    uint32_t ovl = (cidaux >> 8) & 3;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                        overlay_hit = true;
                    }
                    break;
                }
                case 6: /* 1-Bit Overlay */ {
                    uint32_t shift = (mode_entry & 2) ? 9 : 8;
                    uint32_t ovl = (cidaux >> shift) & 1;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                        overlay_hit = true;
                    }
                    break;
                }
                case 7: /* 1-Bit Overlay + 1-Bit Underlay */ {
                    uint32_t ovl = (cidaux >> 8) & 1;
                    if (ovl) {
                        rgb = s->cmap0_palette[
                            (aux_msb | ovl) & 0x1fff];
                    } else {
                        rgb = s->cmap0_palette[
                            (aux_msb | ((cidaux >> 9) & 1)) & 0x1fff];
                    }
                    overlay_hit = true;
                    break;
                }
                default:
                    break;
                }
                if (!overlay_hit) {
                    goto main_pixel;
                }
            } else {
            main_pixel:
            if (pix_mode == 0) {
                /* CI mode — look up in CMAP palette */
                uint16_t ci;
                switch (pix_size) {
                case 0: ci = pixel & 0xf; break;
                case 1: ci = pixel & 0xff; break;
                case 2: ci = pixel & 0xfff; break;  /* 12bpp CI */
                default: ci = pixel & 0xff; break;
                }
                rgb = s->cmap0_palette[(ci_msb | ci) & 0x1fff];
            } else {
                /* RGB mode — unpack packed BGR pixel (MAME ref:
                 * convert_{4,8,12}bpp_bgr_to_24bpp_rgb()) */
                rgb = newport_rgb_unpack(pixel, pix_size, mode_entry);
            }
            }

            /* RAMDAC gamma correction — MAME ref: ramdac_remap() line 1550 */
            r = s->ramdac_lut_r[(rgb >> 16) & 0xff];
            g = s->ramdac_lut_g[(rgb >> 8) & 0xff];
            b = s->ramdac_lut_b[rgb & 0xff];

            dest[y * NEWPORT_SCREEN_W + x] = rgb_to_pixel32(r, g, b);
        }
    }

    /* Cursor overlay (drawn on top of everything) */
    newport_draw_cursor(s, dest);

    /* NewView frame boundary marker */
    newport_newview_log(s, 0x80000000, 0);

    dpy_gfx_update(s->con, 0, 0, NEWPORT_SCREEN_W, NEWPORT_SCREEN_H);
    s->display_dirty = false;
}

static const GraphicHwOps newport_gfx_ops = {
    .invalidate = newport_invalidate,
    .gfx_update = newport_update_display,
};

/*
 * ============================================================
 * Device lifecycle
 * ============================================================
 */

static void sgi_newport_reset(DeviceState *dev)
{
    SGINewportState *s = SGI_NEWPORT(dev);

    /* Clear all registers */
    s->drawmode0 = 0;
    s->drawmode1 = 0;
    s->ls_mode = 0;
    s->ls_pattern = 0;
    s->ls_pattern_saved = 0;
    s->z_pattern = 0;
    s->color_back = 0;
    s->color_vram = 0;
    s->alpha_ref = 0;
    for (int i = 0; i < 5; i++) {
        s->smask_x[i] = 0;
        s->smask_y[i] = 0;
    }
    s->setup = 0;
    s->step_z = 0;

    /* Decoded DRAWMODE fields */
    newport_decode_drawmode0(s);
    newport_decode_drawmode1(s);

    s->x_start = 0;
    s->y_start = 0;
    s->x_end = 0;
    s->y_end = 0;
    s->x_save = 0;
    s->xy_move = 0;
    s->bres_d = 0;
    s->bres_s1 = 0;
    s->bres_octant_inc1 = 0;
    s->bres_round_inc2 = 0;
    s->bres_e1 = 0;
    s->bres_s2 = 0;
    s->a_weight0 = 0;
    s->a_weight1 = 0;
    s->x_start_f = 0;
    s->y_start_f = 0;
    s->x_end_f = 0;
    s->y_end_f = 0;
    s->x_start_i = 0;
    s->xy_start_i = 0;
    s->xy_end_i = 0;
    s->x_start_end_i = 0;

    /* Integer coordinates */
    s->x_start_int = 0;
    s->y_start_int = 0;
    s->x_end_int = 0;
    s->y_end_int = 0;
    s->x_save_int = 0;
    s->iter_x = 0;
    s->iter_y = 0;

    s->color_red = 0;
    s->color_alpha = 0;
    s->color_green = 0;
    s->color_blue = 0;
    s->curr_color_red = 0;
    s->curr_color_alpha = 0;
    s->curr_color_green = 0;
    s->curr_color_blue = 0;
    s->slope_red = 0;
    s->slope_alpha = 0;
    s->slope_green = 0;
    s->slope_blue = 0;
    s->write_mask = 0x00ffffff;
    s->color_i = 0;
    s->zero_overflow = 0;
    s->host_dataport = 0;
    s->host_shift = 0;
    s->global_mask = 0xff; /* XL8 (Indy) */

    s->dcb_mode = 0;
    s->dcb_data_msw = 0;
    s->dcb_data_lsw = 0;

    s->top_scanline = 0;
    s->xy_window = 0;
    s->clip_mode = 0;

    s->config = 0;
    s->status = REX3_VERSION_INDY;

    /* VC2 */
    s->vc2_ram_addr = 0;
    s->vc2_reg_idx = 0;
    s->vc2_reg_data = 0;
    memset(s->vc2_ram, 0, sizeof(s->vc2_ram));
    memset(s->vc2_reg, 0, sizeof(s->vc2_reg));

    /* XMAP */
    s->xmap_config = 0;
    s->xmap_revision = 1;
    memset(s->xmap_mode_table, 0, sizeof(s->xmap_mode_table));
    s->xmap_cursor_cmap = 0;
    s->xmap_popup_cmap = 0;
    s->xmap_mode_table_idx = 0;

    /* CMAP */
    s->cmap_revision = 0xa1;
    s->cmap_palette_idx = 0;
    memset(s->cmap0_palette, 0, sizeof(s->cmap0_palette));

    /* RAMDAC — initialize gamma LUTs to identity mapping.
     * The PROM's SetGammaIdentity() programs these via DCB writes,
     * but we need a sane default so the display isn't black before
     * the PROM runs (or if a reset occurs without PROM re-init). */
    s->ramdac_lut_index = 0;
    for (int i = 0; i < 256; i++) {
        s->ramdac_lut_r[i] = i;
        s->ramdac_lut_g[i] = i;
        s->ramdac_lut_b[i] = i;
    }

    /* Clear VRAM */
    if (s->vram_rgbci) {
        memset(s->vram_rgbci, 0,
               NEWPORT_VRAM_W * NEWPORT_VRAM_H * sizeof(uint32_t));
    }
    if (s->vram_cidaux) {
        memset(s->vram_cidaux, 0,
               NEWPORT_VRAM_W * NEWPORT_VRAM_H * sizeof(uint32_t));
    }

    s->display_dirty = true;
}

static void sgi_newport_realize(DeviceState *dev, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(dev);

    /* Allocate VRAM */
    s->vram_rgbci = g_malloc0(NEWPORT_VRAM_W * NEWPORT_VRAM_H *
                              sizeof(uint32_t));
    s->vram_cidaux = g_malloc0(NEWPORT_VRAM_W * NEWPORT_VRAM_H *
                               sizeof(uint32_t));
    s->global_mask = 0xff; /* XL8 (Indy default) */

    /* DCB bus timeout timer */
    s->dcb_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         newport_dcb_timeout, s);

    /* VBLANK timer — 60Hz vertical retrace interrupt */
    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    newport_vblank_timer, s);
    s->vrint_deassert_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                            newport_vrint_deassert, s);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / 60);

    /* Create display console */
    s->con = graphic_console_init(dev, 0, &newport_gfx_ops, s);
    qemu_console_resize(s->con, NEWPORT_SCREEN_W, NEWPORT_SCREEN_H);

    /* Open NewView binary log file if property is set */
    if (s->newview_log_path && s->newview_log_path[0] != '\0') {
        s->newview_log_file = fopen(s->newview_log_path, "wb");
        if (!s->newview_log_file) {
            qemu_log_mask(LOG_UNIMP,
                          "newport: failed to open NewView log '%s'\n",
                          s->newview_log_path);
        }
    }

    s->display_dirty = true;
}

static void sgi_newport_init(Object *obj)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &sgi_newport_ops, s,
                          "sgi-newport", REX3_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* IRQ output for vertical retrace → INT3_LOCAL1 bit 7 */
    sysbus_init_irq(sbd, &s->irq);
}

static void sgi_newport_finalize(Object *obj)
{
    SGINewportState *s = SGI_NEWPORT(obj);

    if (s->newview_log_file) {
        fclose(s->newview_log_file);
        s->newview_log_file = NULL;
    }
    g_free(s->vram_rgbci);
    g_free(s->vram_cidaux);
}

/*
 * ============================================================
 * QOM diagnostic properties — queryable at runtime via qom-get
 * ============================================================
 */

static char *newport_get_diag_cmap(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(4096);
    int first_nz = -1, last_nz = -1;
    uint32_t nz_count = 0;

    for (int i = 0; i < 8192; i++) {
        if (s->cmap0_palette[i]) {
            nz_count++;
            if (first_nz < 0) {
                first_nz = i;
            }
            last_nz = i;
        }
    }

    g_string_append_printf(buf,
        "CMAP revision=0x%02x palette_idx=%u\n"
        "Non-zero: %u/8192 entries",
        s->cmap_revision, s->cmap_palette_idx, nz_count);

    if (first_nz >= 0) {
        g_string_append_printf(buf, " (range %d..%d)\n", first_nz, last_nz);
    } else {
        g_string_append(buf, "\n");
    }

    /* Dump all 256 entries in blocks of 8 for each non-zero 256-entry page */
    for (int page = 0; page < 32; page++) {
        int base = page << 8;
        bool has_data = false;
        for (int i = 0; i < 256; i++) {
            if (s->cmap0_palette[base + i]) {
                has_data = true;
                break;
            }
        }
        if (!has_data) {
            continue;
        }
        g_string_append_printf(buf, "Page %d (0x%04x-0x%04x):\n", page,
                               base, base + 255);
        for (int row = 0; row < 256; row += 8) {
            g_string_append_printf(buf, "  [%3d]", row);
            for (int col = 0; col < 8; col++) {
                g_string_append_printf(buf, " %06x",
                    s->cmap0_palette[base + row + col] & 0xffffff);
            }
            g_string_append_c(buf, '\n');
        }
    }

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_xmap(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(1024);

    g_string_append_printf(buf,
        "XMAP config=0x%08x revision=%u\n"
        "cursor_cmap=0x%02x popup_cmap=0x%02x mode_table_idx=%u\n",
        s->xmap_config, s->xmap_revision,
        s->xmap_cursor_cmap, s->xmap_popup_cmap, s->xmap_mode_table_idx);

    g_string_append(buf, "Mode table (non-zero entries):\n");
    for (int i = 0; i < 32; i++) {
        uint32_t entry = s->xmap_mode_table[i];
        if (entry == 0) {
            continue;
        }
        uint8_t pix_mode = (entry >> 8) & 3;
        uint8_t pix_size = (entry >> 10) & 3;
        uint8_t ci_msb = entry & 0xf8;
        uint8_t aux_pix_mode = (entry >> 16) & 7;
        const char *pm_name[] = {"CI", "RGB-map1", "RGB-map2", "RGB-direct"};
        const char *ps_name[] = {"4bpp", "8bpp", "12bpp", "24bpp"};
        g_string_append_printf(buf,
            "  [%2d] 0x%08x  pix_mode=%s pix_size=%s ci_msb=0x%02x"
            " aux_pix_mode=%d\n",
            i, entry, pm_name[pix_mode], ps_name[pix_size],
            ci_msb, aux_pix_mode);
    }

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_vc2(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(2048);

    static const char *reg_names[] = {
        [0x00] = "VIDEO_ENTRY",   [0x01] = "CURSOR_ENTRY",
        [0x02] = "CURSOR_X",      [0x03] = "CURSOR_Y",
        [0x04] = "CUR_CURSOR_X",  [0x05] = "DID_ENTRY",
        [0x06] = "SCANLINE_LEN",  [0x07] = "RAM_ADDR",
        [0x08] = "VT_FRAME_PTR",  [0x09] = "VT_LINE_PTR",
        [0x0a] = "VT_LINE_RUN",   [0x0b] = "VT_LINE_COUNT",
        [0x0c] = "CURSOR_TABLE",  [0x0d] = "WORK_CURSOR_Y",
        [0x0e] = "DID_FRAME_PTR", [0x0f] = "DID_LINE_PTR",
        [0x10] = "DC_CONTROL",    [0x1f] = "CONFIG",
    };

    g_string_append_printf(buf,
        "VC2 reg_idx=%u reg_data=0x%04x ram_addr=0x%04x\n",
        s->vc2_reg_idx, s->vc2_reg_data, s->vc2_ram_addr);

    g_string_append(buf, "Registers:\n");
    for (int i = 0; i < 32; i++) {
        const char *name = (i < 0x11) ? reg_names[i] :
                           (i == 0x1f) ? reg_names[0x1f] : NULL;
        if (!name && s->vc2_reg[i] == 0) {
            continue;
        }
        g_string_append_printf(buf, "  [0x%02x] %-16s = 0x%04x",
                               i, name ? name : "???", s->vc2_reg[i]);
        if (i == 0x10) {
            uint16_t dc = s->vc2_reg[i];
            g_string_append_printf(buf, "  (vintr=%d display=%d dids=%d"
                " cursor=%d cursor_disp=%d)",
                !!(dc & VC2_DC_ENA_VINTR), !!(dc & VC2_DC_ENA_DISPLAY),
                !!(dc & VC2_DC_ENA_DIDS), !!(dc & VC2_DC_ENA_CURSOR),
                !!(dc & VC2_DC_CURSOR_DISP));
        }
        g_string_append_c(buf, '\n');
    }

    /* DID table summary: show all unique line patterns */
    uint16_t did_entry = s->vc2_reg[VC2_DID_ENTRY];
    if (did_entry != 0) {
        /* Collect unique DID line pointer values with line ranges */
        uint16_t unique_ptrs[64];
        int unique_first[64], unique_last[64];
        int n_unique = 0;

        for (int i = 0; i < 1024 && (did_entry + i) < 32768; i++) {
            uint16_t line_ptr = s->vc2_ram[(did_entry + i) & 0x7fff];
            int found = -1;
            for (int u = 0; u < n_unique; u++) {
                if (unique_ptrs[u] == line_ptr) {
                    found = u;
                    break;
                }
            }
            if (found >= 0) {
                unique_last[found] = i;
            } else if (n_unique < 64) {
                unique_ptrs[n_unique] = line_ptr;
                unique_first[n_unique] = i;
                unique_last[n_unique] = i;
                n_unique++;
            }
        }

        g_string_append_printf(buf, "DID table (%d unique line patterns):\n",
                               n_unique);
        for (int u = 0; u < n_unique; u++) {
            g_string_append_printf(buf, "  lines %d-%d: ptr=0x%04x entries:",
                                   unique_first[u], unique_last[u],
                                   unique_ptrs[u]);
            for (int j = 0; j < 16; j++) {
                uint16_t e = s->vc2_ram[(unique_ptrs[u] + j) & 0x7fff];
                g_string_append_printf(buf, " 0x%04x(did=%d,x=%d)",
                                       e, e & 0x1f, e >> 5);
                if ((e >> 5) >= 1280) break;
            }
            g_string_append_c(buf, '\n');
        }
    }

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_rex3(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(2048);
    const char *op_names[] = {"NOOP", "READ", "DRAW", "SCR2SCR"};
    const char *adr_names[] = {"SPAN", "BLOCK", "ILINE", "FLINE",
                               "ADR4", "ADR5", "ADR6", "ADR7"};
    const char *logicop_names[] = {
        "ZERO", "SRC&DST", "SRC&~DST", "SRC",
        "~SRC&DST", "DST", "SRC^DST", "SRC|DST",
        "~(SRC|DST)", "~(SRC^DST)", "~DST", "SRC|~DST",
        "~SRC", "~SRC|DST", "~(SRC&DST)", "ONE"
    };

    g_string_append_printf(buf,
        "DRAWMODE0=0x%08x  opcode=%s adrmode=%s dosetup=%d\n"
        "  colorhost=%d stoponx=%d stopony=%d shade=%d\n"
        "  skipfirst=%d skiplast=%d length32=%d\n",
        s->drawmode0,
        op_names[DM0_OPCODE(s->drawmode0)],
        adr_names[DM0_ADRMODE(s->drawmode0)],
        !!(s->drawmode0 & DM0_DOSETUP),
        DM0_COLORHOST(s->drawmode0),
        DM0_STOPONX(s->drawmode0), DM0_STOPONY(s->drawmode0),
        !!(s->drawmode0 & DM0_SHADE),
        !!(s->drawmode0 & DM0_SKIPFIRST),
        !!(s->drawmode0 & DM0_SKIPLAST),
        !!(s->drawmode0 & DM0_LENGTH32));

    g_string_append_printf(buf,
        "DRAWMODE1=0x%08x  planes=%d drawdepth=%d rgbmode=%d\n"
        "  rwpacked=%d hostdepth=%d rwdouble=%d fastclear=%d\n"
        "  blend=%d logicop=%s(%d)\n",
        s->drawmode1, s->dm1_planes, s->dm1_drawdepth, s->dm1_rgbmode,
        s->dm1_rwpacked, s->dm1_hostdepth, s->dm1_rwdouble,
        s->dm1_fastclear, s->dm1_blend,
        logicop_names[s->dm1_logicop & 0xf], s->dm1_logicop);

    g_string_append_printf(buf,
        "Coords: iter=(%d,%d) start=(%d,%d) end=(%d,%d) save=%d\n"
        "  xy_start_i=0x%08x xy_end_i=0x%08x\n",
        s->iter_x, s->iter_y, s->x_start_int, s->y_start_int,
        s->x_end_int, s->y_end_int, s->x_save_int,
        s->xy_start_i, s->xy_end_i);

    g_string_append_printf(buf,
        "Colors: red=0x%08x green=0x%08x blue=0x%08x alpha=0x%08x\n"
        "  color_i=0x%08x color_back=0x%08x\n"
        "Write_mask=0x%08x global_mask=0x%08x\n"
        "Clip_mode=0x%08x status=0x%08x config=0x%08x\n",
        s->color_red, s->color_green, s->color_blue, s->color_alpha,
        s->color_i, s->color_back,
        s->write_mask, s->global_mask,
        s->clip_mode, s->status, s->config);

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_dcb(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(512);
    uint32_t m = s->dcb_mode;
    const char *dw_names[] = {"4bit", "1byte", "2byte", "4byte"};
    const char *slave_names[] = {
        "VC2", "CMAP01", "CMAP0", "CMAP1",
        "XMAP01", "XMAP0", "XMAP1", "RAMDAC",
        "CC1", "AB1", "10", "I2C", "PCD", "13", "14", "15"
    };

    g_string_append_printf(buf,
        "DCB_MODE=0x%08x\n"
        "  slave=%s(%d) CRS=%d data_width=%s\n"
        "  encrsinc=%d ensyncack=%d enasyncack=%d\n"
        "  cswidth=%d cshold=%d cssetup=%d swapendian=%d\n"
        "DCB_DATA_MSW=0x%08x DCB_DATA_LSW=0x%08x\n",
        m,
        slave_names[(m >> 7) & 0xf], (m >> 7) & 0xf,
        (m >> 4) & 7, dw_names[m & 3],
        !!(m & DCB_MODE_ENCRSINC),
        !!(m & DCB_MODE_ENSYNCACK), !!(m & DCB_MODE_ENASYNCACK),
        (m >> 13) & 0x1f, (m >> 18) & 0x1f, (m >> 23) & 0x1f,
        !!(m & DCB_MODE_SWAPENDIAN),
        s->dcb_data_msw, s->dcb_data_lsw);

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_all(Object *obj, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);
    GString *buf = g_string_sized_new(1024);

    /* CMAP summary */
    uint32_t cmap_nz = 0;
    int cmap_first = -1, cmap_last = -1;
    for (int i = 0; i < 8192; i++) {
        if (s->cmap0_palette[i]) {
            cmap_nz++;
            if (cmap_first < 0) {
                cmap_first = i;
            }
            cmap_last = i;
        }
    }
    g_string_append_printf(buf, "CMAP: %u non-zero entries", cmap_nz);
    if (cmap_first >= 0) {
        g_string_append_printf(buf, " (%d=0x%04x..%d=0x%04x)",
                               cmap_first, cmap_first, cmap_last, cmap_last);
    }
    g_string_append_c(buf, '\n');
    /* Per-page summary: 32 pages of 256 entries each = 8192 */
    g_string_append(buf, "CMAP pages with data:");
    for (int page = 0; page < 32; page++) {
        int cnt = 0;
        for (int j = 0; j < 256; j++) {
            if (s->cmap0_palette[page * 256 + j]) {
                cnt++;
            }
        }
        if (cnt) {
            g_string_append_printf(buf, " p%d(0x%04x,%d)",
                                   page, page * 256, cnt);
        }
    }
    g_string_append_c(buf, '\n');

    /* XMAP mode table — dump all non-zero entries with decoded ci_msb */
    g_string_append_printf(buf, "XMAP config=0x%02x modes:\n", s->xmap_config);
    for (int i = 0; i < 32; i++) {
        uint32_t me = s->xmap_mode_table[i];
        if (me) {
            uint8_t pm = (me >> 8) & 3;
            uint8_t ps = (me >> 10) & 3;
            uint16_t cmsb;
            switch (pm) {
            case 0: cmsb = (me & 0xf8) << 5; break;
            case 1: cmsb = 0x1d00; break;
            case 2: cmsb = 0x1e00; break;
            default: cmsb = 0x1f00; break;
            }
            g_string_append_printf(buf,
                "  mode[%2d]=0x%06x pm=%d ps=%d ci_msb=0x%04x\n",
                i, me, pm, ps, cmsb);
        }
    }

    /* VC2 summary */
    g_string_append_printf(buf,
        "VC2: cursor=(%d,%d) DC=0x%04x DID_ENTRY=0x%04x\n",
        s->vc2_reg[VC2_CURSOR_X], s->vc2_reg[VC2_CURSOR_Y],
        s->vc2_reg[VC2_DC_CONTROL], s->vc2_reg[VC2_DID_ENTRY]);

    /* REX3 summary */
    const char *op_names[] = {"NOOP", "READ", "DRAW", "SCR2SCR"};
    const char *adr_names[] = {"SPAN", "BLOCK", "ILINE", "FLINE",
                               "ADR4", "ADR5", "ADR6", "ADR7"};
    g_string_append_printf(buf,
        "REX3: dm0=%s/%s dm1=logicop%d/%s iter=(%d,%d) status=0x%08x\n",
        op_names[DM0_OPCODE(s->drawmode0)],
        adr_names[DM0_ADRMODE(s->drawmode0)],
        s->dm1_logicop,
        s->dm1_rgbmode ? "RGB" : "CI",
        s->iter_x, s->iter_y, s->status);

    /* DCB summary */
    g_string_append_printf(buf,
        "DCB: mode=0x%08x data_msw=0x%08x\n",
        s->dcb_mode, s->dcb_data_msw);

    /* VRAM stats */
    uint32_t vram_nz = 0, cidaux_nz = 0;
    uint32_t vram_first_idx = 0, vram_first_val = 0;
    uint32_t vram_histogram[256] = {0};
    for (int i = 0; i < NEWPORT_VRAM_W * NEWPORT_VRAM_H; i++) {
        uint32_t pix = s->vram_rgbci[i];
        if (pix) {
            if (!vram_nz) {
                vram_first_idx = i;
                vram_first_val = pix;
            }
            vram_nz++;
            vram_histogram[pix & 0xff]++;
        }
        if (s->vram_cidaux[i]) {
            cidaux_nz++;
        }
    }
    g_string_append_printf(buf,
        "VRAM: rgbci_nonzero=%u cidaux_nonzero=%u (of %u)\n",
        vram_nz, cidaux_nz, NEWPORT_VRAM_W * NEWPORT_VRAM_H);
    if (vram_nz) {
        g_string_append_printf(buf,
            "VRAM: first_nz idx=%u val=0x%08x (x=%u,y=%u)\n",
            vram_first_idx, vram_first_val,
            vram_first_idx % NEWPORT_VRAM_W,
            vram_first_idx / NEWPORT_VRAM_W);
        /* Show top 5 CI values */
        g_string_append(buf, "VRAM: top CI values:");
        for (int pass = 0; pass < 5; pass++) {
            int best = -1;
            uint32_t best_cnt = 0;
            for (int ci = 0; ci < 256; ci++) {
                if (vram_histogram[ci] > best_cnt) {
                    best = ci;
                    best_cnt = vram_histogram[ci];
                }
            }
            if (best < 0) break;
            g_string_append_printf(buf, " CI=%d(%u)", best, best_cnt);
            vram_histogram[best] = 0;
        }
        g_string_append_c(buf, '\n');
    }

    /* CMAP sample at the DID-selected ci_msb */
    bool use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS)
                   && s->vc2_reg[VC2_DID_ENTRY] != 0;
    if (use_did) {
        uint16_t fp = s->vc2_reg[VC2_DID_ENTRY];
        uint16_t lp = s->vc2_ram[fp & 0x7fff];
        uint16_t de = s->vc2_ram[lp & 0x7fff];
        uint32_t me = s->xmap_mode_table[de & 0x1f];
        uint16_t cmsb = (me & 0xf8) << 5;
        g_string_append_printf(buf,
            "DID chain: entry=0x%04x did=%d mode=0x%08x ci_msb=0x%04x\n"
            "CMAP[ci_msb+0..7]: %06x %06x %06x %06x %06x %06x %06x %06x\n",
            de, de & 0x1f, me, cmsb,
            s->cmap0_palette[(cmsb+0) & 0x1fff],
            s->cmap0_palette[(cmsb+1) & 0x1fff],
            s->cmap0_palette[(cmsb+2) & 0x1fff],
            s->cmap0_palette[(cmsb+3) & 0x1fff],
            s->cmap0_palette[(cmsb+4) & 0x1fff],
            s->cmap0_palette[(cmsb+5) & 0x1fff],
            s->cmap0_palette[(cmsb+6) & 0x1fff],
            s->cmap0_palette[(cmsb+7) & 0x1fff]);
    }

    return g_string_free(buf, FALSE);
}

static char *newport_get_fb_dump(Object *obj, Error **errp)
{
    return g_strdup("");
}

static void newport_set_fb_dump(Object *obj, const char *value, Error **errp)
{
    SGINewportState *s = SGI_NEWPORT(obj);

    if (value && value[0] != '\0') {
        /* Force display dirty so the surface gets updated too */
        s->display_dirty = true;
        newport_dump_vram_ppm(s, value);
    }
}

static const Property sgi_newport_properties[] = {
    DEFINE_PROP_STRING("newview-log", SGINewportState, newview_log_path),
};

static int sgi_newport_post_load(void *opaque, int version_id)
{
    SGINewportState *s = SGI_NEWPORT(opaque);

    /* Rebuild decoded drawmode fields from saved register values */
    newport_decode_drawmode1(s);
    newport_decode_drawmode0(s);

    /* Force display refresh */
    s->display_dirty = true;

    return 0;
}

#define NEWPORT_VRAM_BYTES \
    (NEWPORT_VRAM_W * NEWPORT_VRAM_H * sizeof(uint32_t))

static const VMStateDescription vmstate_sgi_newport = {
    .name = "sgi-newport",
    .version_id = 5,
    .minimum_version_id = 5,
    .post_load = sgi_newport_post_load,
    .fields = (const VMStateField[]) {
        /* REX3 drawing registers */
        VMSTATE_UINT32(drawmode0, SGINewportState),
        VMSTATE_UINT32(drawmode1, SGINewportState),
        VMSTATE_UINT32(ls_mode, SGINewportState),
        VMSTATE_UINT32(ls_pattern, SGINewportState),
        VMSTATE_UINT32(ls_pattern_saved, SGINewportState),
        VMSTATE_UINT32(z_pattern, SGINewportState),
        VMSTATE_UINT32(color_back, SGINewportState),
        VMSTATE_UINT32(color_vram, SGINewportState),
        VMSTATE_UINT32(alpha_ref, SGINewportState),
        VMSTATE_UINT32_ARRAY(smask_x, SGINewportState, 5),
        VMSTATE_UINT32_ARRAY(smask_y, SGINewportState, 5),
        VMSTATE_UINT32(setup, SGINewportState),
        VMSTATE_UINT32(step_z, SGINewportState),

        /* Coordinate registers */
        VMSTATE_INT32(x_start, SGINewportState),
        VMSTATE_INT32(y_start, SGINewportState),
        VMSTATE_INT32(x_end, SGINewportState),
        VMSTATE_INT32(y_end, SGINewportState),
        VMSTATE_INT32(x_save, SGINewportState),
        VMSTATE_UINT32(xy_move, SGINewportState),
        VMSTATE_UINT32(bres_d, SGINewportState),
        VMSTATE_UINT32(bres_s1, SGINewportState),
        VMSTATE_UINT32(bres_octant_inc1, SGINewportState),
        VMSTATE_UINT32(bres_round_inc2, SGINewportState),
        VMSTATE_UINT32(bres_e1, SGINewportState),
        VMSTATE_UINT32(bres_s2, SGINewportState),
        VMSTATE_UINT32(a_weight0, SGINewportState),
        VMSTATE_UINT32(a_weight1, SGINewportState),
        VMSTATE_UINT32(x_start_f, SGINewportState),
        VMSTATE_UINT32(y_start_f, SGINewportState),
        VMSTATE_UINT32(x_end_f, SGINewportState),
        VMSTATE_UINT32(y_end_f, SGINewportState),
        VMSTATE_INT32(x_start_i, SGINewportState),
        VMSTATE_UINT32(xy_start_i, SGINewportState),
        VMSTATE_UINT32(xy_end_i, SGINewportState),
        VMSTATE_UINT32(x_start_end_i, SGINewportState),

        /* Integer coordinate state */
        VMSTATE_INT16(iter_x, SGINewportState),
        VMSTATE_INT16(iter_y, SGINewportState),
        VMSTATE_INT16(x_start_int, SGINewportState),
        VMSTATE_INT16(y_start_int, SGINewportState),
        VMSTATE_INT16(x_end_int, SGINewportState),
        VMSTATE_INT16(y_end_int, SGINewportState),
        VMSTATE_INT16(x_save_int, SGINewportState),

        /* Color registers */
        VMSTATE_UINT32(write_mask, SGINewportState),
        VMSTATE_UINT32(color_i, SGINewportState),
        VMSTATE_UINT32(zero_overflow, SGINewportState),
        VMSTATE_UINT32(color_red, SGINewportState),
        VMSTATE_UINT32(color_alpha, SGINewportState),
        VMSTATE_UINT32(color_green, SGINewportState),
        VMSTATE_UINT32(color_blue, SGINewportState),
        VMSTATE_UINT32(curr_color_red, SGINewportState),
        VMSTATE_UINT32(curr_color_alpha, SGINewportState),
        VMSTATE_UINT32(curr_color_green, SGINewportState),
        VMSTATE_UINT32(curr_color_blue, SGINewportState),
        VMSTATE_INT32(slope_red, SGINewportState),
        VMSTATE_INT32(slope_alpha, SGINewportState),
        VMSTATE_INT32(slope_green, SGINewportState),
        VMSTATE_INT32(slope_blue, SGINewportState),
        VMSTATE_UINT64(host_dataport, SGINewportState),
        VMSTATE_UINT32(host_shift, SGINewportState),
        VMSTATE_UINT32(global_mask, SGINewportState),

        /* DCB */
        VMSTATE_UINT32(dcb_mode, SGINewportState),
        VMSTATE_UINT32(dcb_data_msw, SGINewportState),
        VMSTATE_UINT32(dcb_data_lsw, SGINewportState),

        /* Screenmask and clipping */
        VMSTATE_UINT32(top_scanline, SGINewportState),
        VMSTATE_UINT32(xy_window, SGINewportState),
        VMSTATE_UINT32(clip_mode, SGINewportState),

        /* Config and status */
        VMSTATE_UINT32(config, SGINewportState),
        VMSTATE_UINT32(status, SGINewportState),

        /* VC2 */
        VMSTATE_UINT32(vc2_ram_addr, SGINewportState),
        VMSTATE_UINT32(vc2_reg_idx, SGINewportState),
        VMSTATE_UINT32(vc2_reg_data, SGINewportState),
        VMSTATE_UINT16_ARRAY(vc2_ram, SGINewportState, 32768),
        VMSTATE_UINT16_ARRAY(vc2_reg, SGINewportState, 32),

        /* XMAP9 */
        VMSTATE_UINT32(xmap_config, SGINewportState),
        VMSTATE_UINT32(xmap_revision, SGINewportState),
        VMSTATE_UINT8(xmap_cursor_cmap, SGINewportState),
        VMSTATE_UINT8(xmap_popup_cmap, SGINewportState),
        VMSTATE_UINT8(xmap_mode_table_idx, SGINewportState),
        VMSTATE_UINT32_ARRAY(xmap_mode_table, SGINewportState, 32),

        /* CMAP */
        VMSTATE_UINT32(cmap_revision, SGINewportState),
        VMSTATE_UINT16(cmap_palette_idx, SGINewportState),
        VMSTATE_UINT32_ARRAY(cmap0_palette, SGINewportState, 8192),

        /* RAMDAC */
        VMSTATE_UINT8(ramdac_lut_index, SGINewportState),
        VMSTATE_UINT32_ARRAY(ramdac_lut_r, SGINewportState, 256),
        VMSTATE_UINT32_ARRAY(ramdac_lut_g, SGINewportState, 256),
        VMSTATE_UINT32_ARRAY(ramdac_lut_b, SGINewportState, 256),

        /* VRAM (heap-allocated, fixed size) */
        VMSTATE_BUFFER_POINTER_UNSAFE(vram_rgbci, SGINewportState, 0,
                                      NEWPORT_VRAM_BYTES),
        VMSTATE_BUFFER_POINTER_UNSAFE(vram_cidaux, SGINewportState, 0,
                                      NEWPORT_VRAM_BYTES),

        /* Display dirty flag */
        VMSTATE_BOOL(display_dirty, SGINewportState),

        VMSTATE_END_OF_LIST()
    }
};

static void sgi_newport_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_newport_realize;
    device_class_set_legacy_reset(dc, sgi_newport_reset);
    dc->vmsd = &vmstate_sgi_newport;
    device_class_set_props(dc, sgi_newport_properties);

    /* fb-dump: set to a file path to dump raw VRAM as PPM (via qom-set) */
    object_class_property_add_str(klass, "fb-dump",
                                  newport_get_fb_dump,
                                  newport_set_fb_dump);

    /* Runtime diagnostic properties — read-only via qom-get */
    object_class_property_add_str(klass, "diag-cmap",
                                  newport_get_diag_cmap, NULL);
    object_class_property_add_str(klass, "diag-xmap",
                                  newport_get_diag_xmap, NULL);
    object_class_property_add_str(klass, "diag-vc2",
                                  newport_get_diag_vc2, NULL);
    object_class_property_add_str(klass, "diag-rex3",
                                  newport_get_diag_rex3, NULL);
    object_class_property_add_str(klass, "diag-dcb",
                                  newport_get_diag_dcb, NULL);
    object_class_property_add_str(klass, "diag-all",
                                  newport_get_diag_all, NULL);
}

static const TypeInfo sgi_newport_info = {
    .name              = TYPE_SGI_NEWPORT,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(SGINewportState),
    .instance_init     = sgi_newport_init,
    .instance_finalize = sgi_newport_finalize,
    .class_init        = sgi_newport_class_init,
};

static void sgi_newport_register_types(void)
{
    type_register_static(&sgi_newport_info);
}

type_init(sgi_newport_register_types)
