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
#include "hw/display/sgi_newport_virtuix.h"
#include "hw/display/sgi_glaccel.h"
#include "hw/misc/sgi_virtuix_gpa.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "trace.h"

/* PVDISPLAY_VC2_FULL=1 — restore the pre-fix behaviour of the VC2-SRAM (DID
 * table) write handler: whole-screen invalidate per changed word.  The one-binary
 * A/B control for the bounded VC2 invalidation, and its escape hatch. */
static bool newport_vc2_full_oracle(void)
{
    static int on = -1;
    if (on < 0) on = getenv("PVDISPLAY_VC2_FULL") ? 1 : 0;
    return on != 0;
}

/* Phase D dirty-rect forward declarations (used in REX3 dispatch, defined below).
 * newport_dirty_full() is a macro over newport_dirty_full_at() so the gated
 * NEWPORT_DIRTYFULL_STATS histogram can attribute every whole-screen invalidation
 * to its call site — the instrument that found the VC2-SRAM saturator (note 27
 * §6d).  Call sites keep writing newport_dirty_full(s). */
static void newport_dirty_full_at(SGINewportVirtuixState *s, int line);
#define newport_dirty_full(s) newport_dirty_full_at((s), __LINE__)
/* Phase D per-row DID resolution cache (used by the VC2/XMAP write handlers) */
static void newport_did_cache_drop(SGINewportVirtuixState *s);
static void newport_dirty_rect(SGINewportVirtuixState *s, int rx, int ry, int rw, int rh);
/* Phase E hardware cursor (used in VC2 write handler) */
static void newport_update_hw_cursor(SGINewportVirtuixState *s);
/* NP_2D_ROWHIST diagnostic (used by output_pixel + fast fills, defined below) */
static inline void newport_rowhist_bump(int wy);

/* NewView binary log record (20 bytes, MAME-compatible layout) */
static void newport_newview_log(SGINewportVirtuixState *s, uint32_t offset,
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
static uint32_t newport_get_default_color(SGINewportVirtuixState *s)
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
static uint32_t newport_get_host_color(SGINewportVirtuixState *s)
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
 * clamps negative (>= 0x180 or sign bit set) to 0, overflow (> 0xff) to 0xff,
 * then PACKS the result to the framebuffer format of the current plane depth
 * (Bayer-dithered when DM1 bit 16 is set), so what lands in VRAM is a pixel
 * the scanout unpack understands.  BL-82: the pre-fix version returned the
 * raw 24-bit value (channel-swapped besides), so 8-bit RGB windows stored
 * 24-bit words whose scanned-out byte decoded as "the red field replicated"
 * — the measured 8-colour collapse (indigo_linux notes 117/119).
 * MAME ref: get_rgb_color() at gio64/newport.cpp:2543-2686 (incl. the
 * s_bayer dither table and both pack switches; channel order matches
 * newport_rgb_unpack: R low, B high).  x/y feed the dither matrix.
 */
static uint32_t newport_get_rgb_color(SGINewportVirtuixState *s,
                                      int16_t x, int16_t y)
{
    static const uint8_t bayer[4][4] = {
        { 0, 12, 3, 15 }, { 8, 4, 11, 7 }, { 2, 14, 1, 13 }, { 10, 6, 9, 5 }
    };
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
        /* CI mode — fall back to default color.  (MAME derives the CI value
         * from the red iterator here; our color_i-based fallback predates
         * this fix and is left as-is — divergence noted in the BL-82 log.) */
        return newport_get_default_color(s);
    }

    if (s->drawmode1 & (1 << 16)) { /* Dithering */
        switch (s->dm1_drawdepth) {
        case 0: { /* 4bpp 1-2-1 */
            const uint8_t sr = (red >> 3) - (red >> 4);
            const uint8_t sg = (green >> 2) - (green >> 4);
            const uint8_t sb = (blue >> 3) - (blue >> 4);
            uint8_t dr = (sr >> 4) & 1;
            uint8_t dg = (sg >> 4) & 3;
            uint8_t db = (sb >> 4) & 1;
            if ((sr & 0xf) > bayer[x & 3][y & 3]) dr++;
            if ((sg & 0xf) > bayer[x & 3][y & 3]) dg++;
            if ((sb & 0xf) > bayer[x & 3][y & 3]) db++;
            if (dr > 1) dr = 1;
            if (dg > 3) dg = 3;
            if (db > 1) db = 1;
            uint32_t color = (db << 3) | (dg << 1) | dr;
            return (color << 4) | color;
        }
        case 1: { /* 8bpp 3-3-2 */
            const uint8_t sr = (red >> 1) - (red >> 4);
            const uint8_t sg = (green >> 1) - (green >> 4);
            const uint8_t sb = (blue >> 2) - (blue >> 4);
            uint8_t dr = (sr >> 4) & 7;
            uint8_t dg = (sg >> 4) & 7;
            uint8_t db = (sb >> 4) & 3;
            if ((sr & 0xf) > bayer[x & 3][y & 3]) dr++;
            if ((sg & 0xf) > bayer[x & 3][y & 3]) dg++;
            if ((sb & 0xf) > bayer[x & 3][y & 3]) db++;
            if (dr > 7) dr = 7;
            if (dg > 7) dg = 7;
            if (db > 3) db = 3;
            return (db << 6) | (dg << 3) | dr;
        }
        case 2: { /* 12bpp 4-4-4 */
            const uint32_t sr = red - (red >> 4);
            const uint32_t sg = green - (green >> 4);
            const uint32_t sb = blue - (blue >> 4);
            uint32_t dr = (sr >> 4) & 15;
            uint32_t dg = (sg >> 4) & 15;
            uint32_t db = (sb >> 4) & 15;
            if ((sr & 0xf) > bayer[x & 3][y & 3]) dr++;
            if ((sg & 0xf) > bayer[x & 3][y & 3]) dg++;
            if ((sb & 0xf) > bayer[x & 3][y & 3]) db++;
            if (dr > 15) dr = 15;
            if (dg > 15) dg = 15;
            if (db > 15) db = 15;
            uint32_t color = (db << 8) | (dg << 4) | dr;
            return (color << 12) | color;
        }
        case 3: /* 24bpp */
            return (blue << 16) | (green << 8) | red;
        default:
            return 0;
        }
    } else {
        switch (s->dm1_drawdepth) {
        case 0: /* 4bpp 1-2-1 */
            return (((blue >> 7) & 1) << 3) | ((green & 0xc0) >> 5) |
                   ((red >> 7) & 1);
        case 1: /* 8bpp 3-3-2 */
            return (blue & 0xc0) | ((green & 0xe0) >> 2) | ((red & 0xe0) >> 5);
        case 2: /* 12bpp 4-4-4 */
            return ((blue & 0xf0) << 4) | (green & 0xf0) | ((red & 0xf0) >> 4);
        case 3: /* 24bpp */
            return (blue << 16) | (green << 8) | red;
        default:
            return 0;
        }
    }
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

static void newport_iterate_shade(SGINewportVirtuixState *s)
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
static void newport_do_setup(SGINewportVirtuixState *s)
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
static void newport_reset_curr_colors(SGINewportVirtuixState *s)
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
static void newport_logic_pixel(SGINewportVirtuixState *s, uint32_t addr,
                                uint32_t src)
{
    uint32_t *buf;
    uint32_t dst, result, mask;

    /* Select buffer based on plane enable */
    if (s->dm1_planes == 0) {
        return;                      /* No planes enabled */
    }

    /*
     * Main RGB/CI planes live in vram_rgbci; overlay(4)/popup(5)/CID(6)
     * planes are packed into vram_cidaux.  MAME ref: setup_draw() dest_buf
     * selection (newport.cpp:1008-1015).
     */
    if (s->dm1_planes == 4 || s->dm1_planes == 5 || s->dm1_planes == 6) {
        buf = s->vram_cidaux;
    } else {
        buf = s->vram_rgbci;
    }

    /*
     * Plane-group bit placement.  The driver targets these planes with the
     * natural pixel value (bits low) plus a write_mask selecting the packed
     * bit positions, so the source must be shifted into the plane's lane
     * before the logic op / mask — otherwise popup writes (value in [1:0],
     * write_mask 0xcc) get masked to zero and the menu is invisible.
     * MAME ref: expand_to_all_lanes() plane switch (newport.cpp:931-940):
     * OLAY << 8 (bits [9:8]), PUP << 2 (bits [3:2]).
     */
    if (s->dm1_planes == 5) {
        src <<= 2;
    } else if (s->dm1_planes == 4) {
        src <<= 8;
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
static bool newport_pixel_clip_pass(SGINewportVirtuixState *s, int16_t x, int16_t y)
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
static void newport_output_pixel(SGINewportVirtuixState *s, int16_t x, int16_t y,
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
    newport_rowhist_bump(wy);
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
static void newport_write_x_start(SGINewportVirtuixState *s, int32_t val)
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
static void newport_write_y_start(SGINewportVirtuixState *s, int32_t val)
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
static void newport_write_x_end(SGINewportVirtuixState *s, int32_t val)
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
static void newport_write_y_end(SGINewportVirtuixState *s, int32_t val)
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
 * ============================================================
 * Phase C 2D instrumentation + A/B oracle
 * ============================================================
 *
 * NP_2D_FORCE_SLOW=1 bypasses every host-native 2D fast path (scr2scr memmove
 * and fast_block_fill), forcing the per-pixel reference paths. It is the
 * bit-exact A/B oracle and must change nothing else.
 *
 * NP_2D_STATS=1 accumulates a per-primitive histogram (keyed so fills / copies /
 * spans / images are distinguishable — this doubles as the Stage 2 op-ranking
 * input) with a fast-vs-slow split inside each primitive that has a host fast
 * path. Counted once per primitive at dispatch — NEVER per pixel (Phase C
 * pitfall "instrumentation in the hot path"). Dumped to stderr roughly every
 * 10s (checked on a primitive tick, not with a timer/per-pixel fprintf) and at
 * process exit, one parseable "NP_2D_STATS:" line per non-zero bucket.
 */
static int np_2d_force_slow = -1;
static inline bool newport_2d_force_slow(void)
{
    if (np_2d_force_slow < 0) {
        np_2d_force_slow = getenv("NP_2D_FORCE_SLOW") ? 1 : 0;
    }
    return np_2d_force_slow;
}

/*
 * NP_2D_VERIFY=1 (diagnostic): for each primitive that would take a host fast
 * path, speculatively run the fast path into a scratch, restore VRAM, then run
 * the faithful slow path, and byte-compare the two over the affected VRAM rows.
 * The FIRST divergence is logged with the op parameters (op, extent, pattern,
 * clip, logic, planes, window) so a fast-path bug can be pinpointed in one boot.
 * Only the base rgbci plane is compared (the fast paths only touch it).
 */
/* Fine-grained bisect gates: disable a single host fast path (keep the others)
 * to localize a fast-path regression that per-op differential testing misses. */
static int np_2d_no_block = -1, np_2d_no_scr2scr = -1;
static inline bool newport_2d_no_block(void)
{
    if (np_2d_no_block < 0) {
        np_2d_no_block = getenv("NP_2D_NO_BLOCK") ? 1 : 0;
    }
    return np_2d_no_block;
}
static inline bool newport_2d_no_scr2scr(void)
{
    if (np_2d_no_scr2scr < 0) {
        np_2d_no_scr2scr = getenv("NP_2D_NO_SCR2SCR") ? 1 : 0;
    }
    return np_2d_no_scr2scr;
}

/* NP_2D_ROWHIST=1: per-VRAM-row count of pixel writes, dumped at exit. Compares
 * the VRAM-row COVERAGE of a pure-fast vs pure-slow boot (catches op-sequence /
 * coverage divergence the per-op pixel verify cannot see). */
static uint64_t g_rowwrite[NEWPORT_VRAM_H];
static int np_2d_rowhist;    /* 0 default; set once in realize from env (cheap
                             * inline branch at the per-pixel call sites) */
static void newport_rowhist_dump(void)
{
    int y;
    if (!np_2d_rowhist) {
        return;
    }
    for (y = 0; y < NEWPORT_VRAM_H; y++) {
        if (g_rowwrite[y]) {
            fprintf(stderr, "NP_ROWHIST %d %" PRIu64 "\n", y, g_rowwrite[y]);
        }
    }
}
/* one predictable, never-taken-when-off branch; wy is already in-bounds */
static inline void newport_rowhist_bump(int wy)
{
    if (np_2d_rowhist) {
        g_rowwrite[wy]++;
    }
}

/*
 * NP_HOSTRW_TRACE=1 (BL-44 diagnostic): trace the REX3 colorhost / HOSTRW
 * host-data image-blit handshake — the path that paints the tiled granite
 * desktop backdrop and drops scanlines under concurrent-emulator jitter.
 * Gated, colorhost-only, and PER-PRIMITIVE (one summary line per GO command,
 * never a per-pixel fprintf in the inner loop — Phase C pitfall). Logs setup
 * writes (XSTARTI-family, drawmode0), each HOSTRW push, and each colorhost
 * draw's extent/host-shift/pixels so a banded boot's granite upload can be
 * diffed line-for-line against a clean boot's.
 */
static int np_hostrw_trace = -1;
static inline bool newport_hostrw_trace(void)
{
    if (np_hostrw_trace < 0) {
        np_hostrw_trace = getenv("NP_HOSTRW_TRACE") ? 1 : 0;
    }
    return np_hostrw_trace;
}
static uint64_t g_hrw_seq;      /* monotonic colorhost-primitive counter */

/*
 * NP_HOSTRW_COUNT=1 (BL-44 discriminator, gated, off by default): count the
 * HOSTRW words the device actually RECEIVES per colorhost host-data episode and
 * compare them with the pixels actually drawn and the setup-implied w*h of the
 * episode's first primitive.  Answers "were words lost upstream of the device?"
 * with one number per episode instead of a 4 MB per-write trace.
 * An "episode" runs from the DRAWMODE0 write that turns colorhost on to the one
 * that turns it off.
 */
static int np_hostrw_count = -1;
static inline bool newport_hostrw_count(void)
{
    if (np_hostrw_count < 0) {
        np_hostrw_count = getenv("NP_HOSTRW_COUNT") ? 1 : 0;
    }
    return np_hostrw_count;
}
static uint64_t hc_words, hc_pixels, hc_prims, hc_episode;
static int hc_x0, hc_y0, hc_x1, hc_y1;
static bool hc_have_extent;

static void newport_hostrw_count_flush(void)
{
    if (hc_prims == 0 && hc_words == 0) {
        return;
    }
    if (hc_have_extent) {
        int64_t w = (int64_t)hc_x1 - hc_x0 + 1;
        int64_t h = (int64_t)hc_y1 - hc_y0 + 1;
        fprintf(stderr, "NP_HC EPISODE #%" PRIu64 " words=%" PRIu64
                " pixels=%" PRIu64 " prims=%" PRIu64
                " rect=(%d,%d)-(%d,%d) implied_wh=%" PRId64 "\n",
                hc_episode, hc_words, hc_pixels, hc_prims,
                hc_x0, hc_y0, hc_x1, hc_y1, w * h);
    } else {
        fprintf(stderr, "NP_HC EPISODE #%" PRIu64 " words=%" PRIu64
                " pixels=%" PRIu64 " prims=%" PRIu64 " rect=none\n",
                hc_episode, hc_words, hc_pixels, hc_prims);
    }
    hc_words = hc_pixels = hc_prims = 0;
    hc_have_extent = false;
    hc_episode++;
}

static int np_2d_verify = -1;
static int np_verify_reported;
static inline bool newport_2d_verify(void)
{
    if (np_2d_verify < 0) {
        np_2d_verify = getenv("NP_2D_VERIFY") ? 1 : 0;
    }
    return np_2d_verify;
}
/* Compare rows [ry0..ry1] full-width of `fast` (captured) vs current vram_rgbci
 * (holding the slow result). Logs the first differing (x,y) + count. */
static void newport_verify_cmp(SGINewportVirtuixState *s, const uint32_t *fast,
                               int ry0, int ry1, const char *op, int wx0,
                               int wy0, int wx1, int wy1, uint32_t pattern)
{
    int y, x, first_x = -1, first_y = -1, ndiff = 0, ndiffrows = 0;
    for (y = ry0; y <= ry1; y++) {
        const uint32_t *fr = &fast[(size_t)(y - ry0) * NEWPORT_VRAM_W];
        const uint32_t *sr = &s->vram_rgbci[(size_t)y * NEWPORT_VRAM_W];
        int rowdiff = 0;
        for (x = 0; x < NEWPORT_VRAM_W; x++) {
            if (fr[x] != sr[x]) {
                if (first_x < 0) { first_x = x; first_y = y; }
                ndiff++; rowdiff = 1;
            }
        }
        ndiffrows += rowdiff;
    }
    if (ndiff && !np_verify_reported) {
        np_verify_reported = 1;
        fprintf(stderr,
            "NP_2D_VERIFY: DIVERGE op=%s ndiff=%d diffrows=%d first=(%d,%d) "
            "extent[wx %d..%d wy %d..%d] pat=0x%08x logic=0x%x planes=%d "
            "clip_mode=0x%x opaque=%d win=(%d,%d)\n",
            op, ndiff, ndiffrows, first_x, first_y, wx0, wx1, wy0, wy1,
            pattern, s->dm1_logicop, s->dm1_planes, s->clip_mode, s->dm0_opaque,
            (int16_t)((s->xy_window >> 16) & 0xffff),
            (int16_t)(s->xy_window & 0xffff));
    }
}

enum {
    NP2D_NOOP, NP2D_READ,
    NP2D_BLOCK_FAST, NP2D_BLOCK_SLOW,   /* fills */
    NP2D_SPAN,                          /* plain spans */
    NP2D_IMAGE,                         /* colorhost span/block = image/text blit */
    NP2D_ILINE, NP2D_FLINE,             /* lines */
    NP2D_SCR2SCR_FAST, NP2D_SCR2SCR_SLOW, /* copies */
    NP2D_DRAW_OTHER,
    NP2D_NBUCKET
};
static const char *const np2d_names[NP2D_NBUCKET] = {
    "noop", "read", "block_fast", "block_slow", "span", "image",
    "iline", "fline", "scr2scr_fast", "scr2scr_slow", "draw_other"
};
static uint64_t np2d_count[NP2D_NBUCKET];
static int np_2d_stats = -1;

/* NP_2D_DIAG: reason a fill/copy fell to the slow path (temporary). */
enum {
    DIAG_BF_STOP, DIAG_BF_STOPX, DIAG_BF_STOPY,
    DIAG_BF_SHADE, DIAG_BF_RGB, DIAG_BF_PAT, DIAG_BF_PEND,
    DIAG_BF_CLIP, DIAG_BF_PLANE, DIAG_BF_XSAVE, DIAG_BF_LOGIC,
    DIAG_SS_STOP, DIAG_SS_CLIP, DIAG_SS_LOGIC, DIAG_SS_HOST, DIAG_SS_PAT,
    DIAG_SS_SHADE, DIAG_SS_PLANE, DIAG_SS_OOB,
    DIAG_N
};
static const char *const diag_names[DIAG_N] = {
    "bf_1px", "bf_1col", "bf_1row",
    "bf_shade", "bf_rgb", "bf_pat", "bf_pend", "bf_clip",
    "bf_plane", "bf_xsave", "bf_logic",
    "ss_stop", "ss_clip", "ss_logic", "ss_host", "ss_pat", "ss_shade",
    "ss_plane", "ss_oob"
};
static uint64_t diag_count[DIAG_N];
static int np_2d_diag = -1;
static inline void newport_2d_diag(int r)
{
    if (np_2d_diag < 0) {
        np_2d_diag = getenv("NP_2D_DIAG") ? 1 : 0;
    }
    if (np_2d_diag) {
        diag_count[r]++;
    }
}

static void newport_2d_stats_dump(void)
{
    int i;
    for (i = 0; i < NP2D_NBUCKET; i++) {
        if (np2d_count[i]) {
            fprintf(stderr, "NP_2D_STATS: %-13s %" PRIu64 "\n",
                    np2d_names[i], np2d_count[i]);
        }
    }
    for (i = 0; i < DIAG_N; i++) {
        if (diag_count[i]) {
            fprintf(stderr, "NP_2D_DIAG: %-10s %" PRIu64 "\n",
                    diag_names[i], diag_count[i]);
        }
    }
}

static void newport_2d_stats_atexit(void)
{
    if (np_2d_stats > 0) {
        newport_2d_stats_dump();
    }
}

static inline void newport_2d_stat(int bucket)
{
    if (np_2d_stats < 0) {
        np_2d_stats = getenv("NP_2D_STATS") ? 1 : 0;
        if (np_2d_stats) {
            atexit(newport_2d_stats_atexit);
        }
    }
    if (!np_2d_stats) {
        return;
    }
    np2d_count[bucket]++;
    /* time-based dump ~every 10s, checked on a primitive tick (never per pixel) */
    {
        static int64_t last_us;
        int64_t now = g_get_monotonic_time();   /* microseconds */
        if (last_us == 0) {
            last_us = now;
        } else if (now - last_us >= (int64_t)10 * 1000 * 1000) {
            last_us = now;
            newport_2d_stats_dump();
        }
    }
}

/*
 * If the enabled screenmask clip region is expressible as a SINGLE rectangle,
 * write its inclusive bounds in RAW (pre-window-offset) fill coordinates — the
 * same space as x_start_int/y_start_int — to *x0..*y1 and return true.  Returns
 * false when more than one of masks 1-4 is enabled (their pass-if-inside-ANY
 * semantics form a union, not a rectangle) so the caller must use the slow
 * per-pixel path.  clip_mode==0 is handled by the caller (unbounded); this
 * helper is only called when clip_mode != 0.
 *
 * Derivation matches newport_pixel_clip_pass():
 *   mask 0    — compares raw x in [smask_x[0]>>16 .. smask_x[0]&0xffff]
 *   masks 1-4 — compare wx = x+winx-0x1000 in [(smask>>16)-0x1000 ..
 *               (smask&0xffff)-0x1000], i.e. raw x in
 *               [(smask>>16)-winx .. (smask&0xffff)-winx].
 */
typedef struct { int x0, y0, x1, y1; } NPClipRect;   /* inclusive raw coords */

/*
 * Decompose the enabled screenmask clip (clip_mode != 0) into a set of
 * rectangles in the RAW coordinate space that output_pixel receives (for a
 * fill: the draw coords; for scr2scr: the destination coords).  Overall pass =
 *   (inside mask0, if mask0 enabled) AND (inside ANY of the enabled masks 1-4).
 * So the passing region is the union of (mask0 ∩ maski) over the enabled i, or
 * just mask0 when none of 1-4 are enabled.  Returns the sub-rect count (1..4);
 * out[] holds inclusive raw bounds.  Coordinate derivation matches
 * newport_pixel_clip_pass() exactly.  Only valid when clip_mode != 0.
 */
static int newport_clip_subrects(SGINewportVirtuixState *s, NPClipRect *out)
{
    int winx = (int16_t)((s->xy_window >> 16) & 0xffff);
    int winy = (int16_t)(s->xy_window & 0xffff);
    int bx0 = INT_MIN, by0 = INT_MIN, bx1 = INT_MAX, by1 = INT_MAX;
    int n = 0, bit;

    if (s->clip_mode & 1) {                    /* mask 0: raw coordinates */
        bx0 = (int16_t)(s->smask_x[0] >> 16);
        bx1 = (int16_t)(s->smask_x[0] & 0xffff);
        by0 = (int16_t)(s->smask_y[0] >> 16);
        by1 = (int16_t)(s->smask_y[0] & 0xffff);
    }
    for (bit = 1; bit < 5; bit++) {
        if (!(s->clip_mode & (1 << bit))) {
            continue;
        }
        /* masks 1-4: wx = x+winx-0x1000 in [(smask>>16)-0x1000 ..
         * (smask&0xffff)-0x1000] ⇒ raw x in [(smask>>16)-winx ..
         * (smask&0xffff)-winx]. */
        int a = (int16_t)(s->smask_x[bit] >> 16) - winx;
        int b = (int16_t)(s->smask_x[bit] & 0xffff) - winx;
        int c = (int16_t)(s->smask_y[bit] >> 16) - winy;
        int d = (int16_t)(s->smask_y[bit] & 0xffff) - winy;
        out[n].x0 = MAX(bx0, a); out[n].x1 = MIN(bx1, b);
        out[n].y0 = MAX(by0, c); out[n].y1 = MIN(by1, d);
        n++;
    }
    if (n == 0) {
        out[0].x0 = bx0; out[0].y0 = by0; out[0].x1 = bx1; out[0].y1 = by1;
        n = 1;
    }
    return n;
}

/*
 * Host-offloaded fast path for solid rectangle / single-row fills (root weave,
 * window clears, opaque panel fills, and — dominant on the 4Dwm desktop — the
 * stop-on-x-only single-row fills Xsgi emits row by row).  Every covered pixel
 * is the same constant store to a CONTIGUOUS run of VRAM, so we fill each row
 * with a tight word loop instead of ~4 function calls per pixel.  Output is
 * bit-identical to the newport_output_pixel path.
 *
 * The WRITE extent [fx0..fx1] x [fy0..fy1] (raw, inclusive, pre-clip) and the
 * register end-state (reg_x, reg_y) are computed by the caller from the stop
 * bits — clipping only suppresses writes, never changes the register end-state,
 * so the two are passed independently.  clip[] is the (possibly multi-rect)
 * clip decomposition intersected with the write extent; pass one INT_MIN/INT_MAX
 * rect when unclipped.  Registers are written once by the caller.
 */
/* Apply plane-lane shift + logic op to a raw pixel value, matching
 * newport_logic_pixel (only dst-independent logic ops reach the fast path). */
static inline uint32_t np_fast_cval(SGINewportVirtuixState *s, uint32_t v)
{
    if (s->dm1_planes == 5) {
        v <<= 2;
    } else if (s->dm1_planes == 4) {
        v <<= 8;
    }
    switch (s->dm1_logicop) {
    case 0x0: return 0;
    case 0xc: return ~v;
    case 0xf: return 0xffffffff;
    default:  return v;            /* 0x3 SRC — the overwhelmingly common case */
    }
}

static void newport_fast_fill_rect(SGINewportVirtuixState *s,
                                   int fx0, int fy0, int fx1, int fy1,
                                   uint32_t color, uint32_t color_back,
                                   uint32_t pattern, bool opaque,
                                   int start_x, int dx)
{
    uint32_t *buf = (s->dm1_planes == 4 || s->dm1_planes == 5 ||
                     s->dm1_planes == 6) ? s->vram_cidaux : s->vram_rgbci;
    uint32_t cval = np_fast_cval(s, color);
    uint32_t cbak = np_fast_cval(s, color_back);
    uint32_t mask = s->write_mask & s->global_mask;
    uint32_t cm = cval & mask, cbm = cbak & mask, nm = ~mask;
    int winx = (int16_t)((s->xy_window >> 16) & 0xffff);
    int winy = (int16_t)(s->xy_window & 0xffff);
    int y;

    for (y = fy0; y <= fy1; y++) {
        int wy = y + winy - 0x1000;
        uint32_t rowoff;

        if (wy < 0 || wy >= NEWPORT_VRAM_H) {
            continue;                  /* out-of-bounds row: slow path skips too */
        }
        newport_rowhist_bump(wy);
        rowoff = (uint32_t)wy * NEWPORT_VRAM_W;

        if (pattern == 0xffffffff) {
            /* solid: one contiguous run per row */
            int wxa = fx0 + winx - 0x1000;
            int wxb = fx1 + winx - 0x1000;
            uint32_t base;
            int n, i;
            if (wxa < 0) wxa = 0;
            if (wxb >= NEWPORT_VRAM_W) wxb = NEWPORT_VRAM_W - 1;
            if (wxa > wxb) continue;
            base = rowoff + (uint32_t)wxa;
            n = wxb - wxa + 1;
            if (mask == 0xffffffff) {
                for (i = 0; i < n; i++) buf[base + i] = cval;
            } else {
                for (i = 0; i < n; i++) buf[base + i] = (buf[base + i] & nm) | cm;
            }
        } else {
            /* stipple: per-pixel pattern (32-bit, phase reset each row to bit 31
             * at start_x).  Bit for raw x = (31 - |x-start_x|) & 31. */
            int lo = 0x1000 - winx;                       /* raw x for wx=0 */
            int hi = lo + NEWPORT_VRAM_W - 1;             /* raw x for wx=W-1 */
            int xa = MAX(fx0, lo), xb = MIN(fx1, hi), x;
            for (x = xa; x <= xb; x++) {
                int i = (dx > 0) ? (x - start_x) : (start_x - x);
                int bit = (31 - (i & 31)) & 31;
                uint32_t addr = rowoff + (uint32_t)(x + winx - 0x1000);
                if (pattern & (1u << bit)) {
                    buf[addr] = (buf[addr] & nm) | cm;
                } else if (opaque) {
                    buf[addr] = (buf[addr] & nm) | cbm;
                }
            }
        }
    }

    if (fx0 <= fx1 && fy0 <= fy1) {
        /* dirty rects are in scanout/dst (screen) space; the window offset only
         * relocates the VRAM write, which the scanout applies on read (BL-83) */
        newport_dirty_rect(s, fx0, fy0, fx1 - fx0 + 1, fy1 - fy0 + 1);
    }
}

/*
 * Fill the (raw, inclusive) extent [wx0..wx1] x [wy0..wy1], honouring the
 * screenmask clip: unclipped fills the whole extent; clipped intersects the
 * extent with each clip sub-rectangle and fills each (constant colour, so
 * overlapping sub-rects are idempotent).  Does NOT touch coordinate registers.
 */
static void newport_fast_block_fill(SGINewportVirtuixState *s,
                                    int wx0, int wy0, int wx1, int wy1,
                                    uint32_t color, uint32_t color_back,
                                    uint32_t pattern, bool opaque,
                                    int start_x, int dx)
{
    if (s->clip_mode == 0) {
        newport_fast_fill_rect(s, wx0, wy0, wx1, wy1, color, color_back,
                               pattern, opaque, start_x, dx);
        return;
    }
    {
        NPClipRect cr[4];
        int nc = newport_clip_subrects(s, cr), ci;
        for (ci = 0; ci < nc; ci++) {
            int fx0 = MAX(wx0, cr[ci].x0), fx1 = MIN(wx1, cr[ci].x1);
            int fy0 = MAX(wy0, cr[ci].y0), fy1 = MIN(wy1, cr[ci].y1);
            if (fx0 <= fx1 && fy0 <= fy1) {
                newport_fast_fill_rect(s, fx0, fy0, fx1, fy1, color, color_back,
                                       pattern, opaque, start_x, dx);
            }
        }
    }
}

/*
 * Block fill — nested loop over Y then X.
 * The PROM's most-used operation (screen clear, rectangle fill).
 * MAME ref: newport.cpp:3418-3492
 */
static void newport_draw_block(SGINewportVirtuixState *s)
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
    /* NP_2D_VERIFY stash (fast-vs-slow compare deferred to end of the slow loop) */
    uint32_t *vf_snap = NULL, *vf_fast = NULL, vf_pat = 0;
    int vf_ry0 = 0, vf_ry1 = -1, vf_wx0 = 0, vf_wx1 = 0, vf_wy0 = 0, vf_wy1 = 0;
    /* BL-44 host-data trace: per-primitive pixel/row counters (colorhost only) */
    bool hrw_tr = s->dm0_colorhost && newport_hostrw_trace();
    uint32_t hrw_pixels = 0, hrw_rows = 0, hrw_shift_in = s->host_shift;
    uint32_t hrw_c0 = 0, hrw_cmax = 0, hrw_cmin = 0xffffffff;
    uint64_t hrw_seq = hrw_tr ? ++g_hrw_seq : 0;

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
    /*
     * Host-offloaded fast path: solid, opaque, unclipped, no-stipple,
     * full-rectangle fill with a destination-independent logic op. Every
     * condition here guarantees the per-pixel loop below would write the same
     * constant value to every covered pixel, so newport_fast_block_fill()
     * reproduces it with a per-row word fill. Anything else falls through to
     * the faithful per-pixel path unchanged.
     */
    {
        /*
         * Fast path gate.  Requires stop-on-x (so the inner loop draws a full
         * contiguous row and the row-complete branch runs, leaving x at
         * x_save_int); stop-on-y is optional:
         *   stop_on_y  ⇒ fill the rectangle [x_start..x_end] x [y_start..y_end],
         *                registers end at (x_save_int, y_end+dy);
         *   !stop_on_y ⇒ the outer loop runs exactly once, so ONE row
         *                (y=start_y) is filled regardless of y_end, and
         *                registers end at (x_save_int, start_y+dy).  This is the
         *                dominant Xsgi form (row-by-row fills).
         * Clipping (any clip_mode) is honoured inside newport_fast_block_fill by
         * intersecting with the clip sub-rectangles; it never changes the
         * register end-state (the slow loop iterates the full extent regardless
         * of clip).  Gated to the main RGBCI planes (1/2) so no overlay/popup/
         * CID plane-lane shift is involved.
         */
        bool fast_ok = stop_on_x && !shade && !lr_abort &&
            !s->dm0_colorhost && (!s->dm1_rgbmode || s->dm1_fastclear) &&
            prim_end_x == end_x &&
            (s->dm1_planes == 1 || s->dm1_planes == 2) &&
            s->x_save_int == start_x &&
            (s->dm1_logicop == 0x3 || s->dm1_logicop == 0x0 ||
             s->dm1_logicop == 0xc || s->dm1_logicop == 0xf);
        if (fast_ok && !newport_2d_force_slow() && !newport_2d_no_block()) {
            int16_t orig_end_x = s->x_end_int;   /* end_x is already exclusive */
            int16_t orig_end_y = s->y_end_int;
            int wx0 = MIN(start_x, orig_end_x);
            int wx1 = MAX(start_x, orig_end_x);
            int wy0, wy1, reg_y;
            if (stop_on_y) {
                wy0 = MIN(start_y, orig_end_y);
                wy1 = MAX(start_y, orig_end_y);
                reg_y = orig_end_y + dy;
            } else {
                wy0 = wy1 = start_y;             /* single row */
                reg_y = start_y + dy;
            }
            newport_2d_stat(NP2D_BLOCK_FAST);
            if (newport_2d_verify() && !np_verify_reported) {
                /* speculative fast run for comparison, then fall through to slow */
                int winy = (int16_t)(s->xy_window & 0xffff);
                int ry0 = wy0 + winy - 0x1000, ry1 = wy1 + winy - 0x1000;
                if (ry0 < 0) ry0 = 0;
                if (ry1 >= NEWPORT_VRAM_H) ry1 = NEWPORT_VRAM_H - 1;
                if (ry0 <= ry1) {
                    size_t nrows = (size_t)(ry1 - ry0 + 1);
                    size_t nb = nrows * NEWPORT_VRAM_W * 4;
                    vf_snap = g_malloc(nb);
                    vf_fast = g_malloc(nb);
                    memcpy(vf_snap, &s->vram_rgbci[(size_t)ry0 * NEWPORT_VRAM_W],
                           nb);
                    newport_fast_block_fill(s, wx0, wy0, wx1, wy1, color,
                                            s->color_back, pattern,
                                            s->dm0_opaque, start_x, dx);
                    memcpy(vf_fast, &s->vram_rgbci[(size_t)ry0 * NEWPORT_VRAM_W],
                           nb);
                    memcpy(&s->vram_rgbci[(size_t)ry0 * NEWPORT_VRAM_W], vf_snap,
                           nb);
                    vf_ry0 = ry0; vf_ry1 = ry1; vf_pat = pattern;
                    vf_wx0 = wx0; vf_wx1 = wx1; vf_wy0 = wy0; vf_wy1 = wy1;
                }
                /* do NOT return: run the faithful slow loop below as ground truth */
            } else {
                newport_fast_block_fill(s, wx0, wy0, wx1, wy1, color,
                                        s->color_back, pattern, s->dm0_opaque,
                                        start_x, dx);
                /* register end-state (independent of clip) */
                s->iter_x = s->x_save_int;
                s->iter_y = reg_y;
                newport_write_x_start(s, (int32_t)s->x_save_int << 11);
                newport_write_y_start(s, (int32_t)reg_y << 11);
                return;
            }
        }
        newport_2d_stat(s->dm0_colorhost ? NP2D_IMAGE : NP2D_BLOCK_SLOW);
        if (!s->dm0_colorhost && !newport_2d_force_slow()) {
            if (!stop_on_x) {
                if (!stop_on_y) newport_2d_diag(DIAG_BF_STOP);       /* 1 pixel */
                else newport_2d_diag(DIAG_BF_STOPX);                 /* 1 col */
            }
            else if (shade || lr_abort) newport_2d_diag(DIAG_BF_SHADE);
            else if (s->dm1_rgbmode && !s->dm1_fastclear) newport_2d_diag(DIAG_BF_RGB);
            else if (prim_end_x != end_x) newport_2d_diag(DIAG_BF_PEND);
            else if (!(s->dm1_planes == 1 || s->dm1_planes == 2)) newport_2d_diag(DIAG_BF_PLANE);
            else if (s->x_save_int != start_x) newport_2d_diag(DIAG_BF_XSAVE);
            else newport_2d_diag(DIAG_BF_LOGIC);
        }
    }

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
                    uint32_t hc = newport_get_host_color(s);
                    newport_output_pixel(s, sx, sy, hc);
                    if (hrw_tr) {
                        if (!hrw_pixels) hrw_c0 = hc;
                        if (hc > hrw_cmax) hrw_cmax = hc;
                        if (hc < hrw_cmin) hrw_cmin = hc;
                    }
                    hrw_pixels++;
                } else if ((shade || s->dm1_rgbmode) && !s->dm1_fastclear) {
                    newport_output_pixel(s, sx, sy,
                                         newport_get_rgb_color(s, sx, sy));
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
            if (hrw_tr) hrw_rows++;
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

    if (s->dm0_colorhost && newport_hostrw_count()) {
        hc_prims++;
        hc_pixels += hrw_pixels;
        if (!hc_have_extent) {
            hc_have_extent = true;
            hc_x0 = start_x;
            hc_y0 = start_y;
            hc_x1 = s->x_end_int;
            hc_y1 = s->y_end_int;
        }
    }

    if (hrw_tr) {
        int16_t ml = s->dm1_rwpacked
            ? rwpacked_max_len[s->dm1_rwdouble ? 1 : 0][s->dm1_hostdepth] : -1;
        fprintf(stderr,
            "NP_HRW BLOCK #%" PRIu64 " start=(%d,%d) end=(%d,%d) prim_end=%d "
            "ml=%d xsave=%d win=(%d,%d) clip=0x%x pl=%d hd=%d rwp=%d rwd=%d "
            "shift_in=%u shift_out=%u px=%u rows=%u iter=(%d,%d) "
            "c0=0x%x cmin=0x%x cmax=0x%x dp=0x%016" PRIx64 "\n",
            hrw_seq, start_x, start_y, s->x_end_int, s->y_end_int, prim_end_x,
            ml, s->x_save_int, (int16_t)((s->xy_window >> 16) & 0xffff),
            (int16_t)(s->xy_window & 0xffff), s->clip_mode, s->dm1_planes,
            s->dm1_hostdepth, s->dm1_rwpacked, s->dm1_rwdouble,
            hrw_shift_in, s->host_shift, hrw_pixels, hrw_rows, sx, sy,
            hrw_c0, (hrw_cmin == 0xffffffff ? 0 : hrw_cmin), hrw_cmax,
            s->host_dataport);
    }

    if (vf_ry1 >= vf_ry0 && vf_fast) {
        newport_verify_cmp(s, vf_fast, vf_ry0, vf_ry1, "block",
                           vf_wx0, vf_wy0, vf_wx1, vf_wy1, vf_pat);
    }
    g_free(vf_snap);
    g_free(vf_fast);
}

/*
 * Span drawing — single scanline, same as block but no Y iteration.
 * Used for text rendering with host data.
 * MAME ref: newport.cpp:3348-3415
 */
static void newport_draw_span(SGINewportVirtuixState *s)
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
    /* BL-44 host-data trace (colorhost only) */
    bool hrw_tr = s->dm0_colorhost && newport_hostrw_trace();
    uint32_t hrw_pixels = 0, hrw_shift_in = s->host_shift;
    uint64_t hrw_seq = hrw_tr ? ++g_hrw_seq : 0;

    trace_sgi_newport_draw_span(start_x, y, end_x);
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

    newport_2d_stat(s->dm0_colorhost ? NP2D_IMAGE : NP2D_SPAN);

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
                hrw_pixels++;
            } else if ((shade || s->dm1_rgbmode) && !s->dm1_fastclear) {
                newport_output_pixel(s, sx, y, newport_get_rgb_color(s, sx, y));
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

    if (s->dm0_colorhost && newport_hostrw_count()) {
        hc_prims++;
        hc_pixels += hrw_pixels;
        if (!hc_have_extent) {
            hc_have_extent = true;
            hc_x0 = start_x;
            hc_y0 = y;
            hc_x1 = s->x_end_int;
            hc_y1 = y;
        }
    }

    if (hrw_tr) {
        int16_t ml = s->dm1_rwpacked
            ? rwpacked_max_len[s->dm1_rwdouble ? 1 : 0][s->dm1_hostdepth] : -1;
        fprintf(stderr,
            "NP_HRW SPAN  #%" PRIu64 " start=(%d,%d) end=%d prim_end=%d ml=%d "
            "xsave=%d win=(%d,%d) clip=0x%x pl=%d hd=%d rwp=%d rwd=%d "
            "shift_in=%u shift_out=%u px=%u iter=(%d,%d) dp=0x%016" PRIx64 "\n",
            hrw_seq, start_x, y, s->x_end_int, prim_end_x, ml, s->x_save_int,
            (int16_t)((s->xy_window >> 16) & 0xffff),
            (int16_t)(s->xy_window & 0xffff), s->clip_mode, s->dm1_planes,
            s->dm1_hostdepth, s->dm1_rwpacked, s->dm1_rwdouble,
            hrw_shift_in, s->host_shift, hrw_pixels, sx, y, s->host_dataport);
    }
}

/*
 * Integer line drawing (Bresenham).
 * MAME ref: do_iline() at newport.cpp:2944-3069
 */
static void newport_draw_iline(SGINewportVirtuixState *s)
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
                newport_output_pixel(s, x0, y0, newport_get_rgb_color(s, x0, y0));
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
static void newport_draw_fline(SGINewportVirtuixState *s)
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
                newport_output_pixel(s, x0, y0, newport_get_rgb_color(s, x0, y0));
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
static void newport_draw_scr2scr(SGINewportVirtuixState *s)
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
    /* NP_2D_VERIFY stash for scr2scr (compare deferred to end of slow loop) */
    uint32_t *sf_snap = NULL, *sf_fast = NULL;
    int sf_ry0 = 0, sf_ry1 = -1, sf_wx0 = 0, sf_wx1 = 0;

    trace_sgi_newport_draw_scr2scr(start_x, start_y, end_x, end_y,
                                   move_x, move_y);

    /*
     * ---- Phase C host-fast scr2scr (the REAL desktop copy form) ----
     *
     * 4Dwm window drags and terminal scrolls arrive as OP_SCR2SCR that is
     * BLOCK-addressed (stop-on-x AND stop-on-y set), logic op SRC, no
     * colorhost/pattern/shade, on the main RGBCI planes.  Reproduce the slow
     * loop's result with a per-row memmove:
     *   - iterate source rows in the slow loop's dy order (this is what makes
     *     vertical overlap safe: for a downward copy the guest picks dy<0 so
     *     the bottom rows move first);
     *   - memmove copies each contiguous run and handles horizontal overlap
     *     within a row; the masked variant does a read-modify-write in the
     *     non-clobbering direction;
     *   - leave x_start/y_start exactly where the slow loop leaves them
     *     (x_save_int, end_y+dy) — nothing else (the slow scr2scr does not
     *     touch iter_x/iter_y).
     * Clip (any clip_mode) is honoured by intersecting the DESTINATION extent
     * with a SINGLE clip sub-rectangle (multi-rect clip stays slow: filling a
     * copy region out of raw y-order across disjoint sub-rects could clobber an
     * overlapping source — not worth the risk for the rare case).  Gated to the
     * main RGBCI planes and to copies whose (clipped) source AND destination
     * rectangles are fully on-screen, so none of the per-pixel clipping
     * subtleties the slow loop has (OOB source reads as 0, OOB dest write
     * skipped) can diverge; anything partially off-screen or on the overlay/
     * popup/CID planes falls through to the faithful per-pixel loop.  The
     * register end-state (x_save_int, end_y+dy) is independent of clip.
     */
    if (!newport_2d_force_slow() && !newport_2d_no_scr2scr() &&
        stop_on_x && stop_on_y &&
        s->dm1_logicop == 0x3 &&
        !s->dm0_colorhost &&
        !s->dm0_lspattern && !s->dm0_zpattern &&
        !(s->drawmode0 & DM0_SHADE) &&
        (s->dm1_planes == 1 || s->dm1_planes == 2)) {

        int winx = (int16_t)((s->xy_window >> 16) & 0xffff);
        int winy = (int16_t)(s->xy_window & 0xffff);
        int xlo = MIN(start_x, end_x), xhi = MAX(start_x, end_x);
        int ylo = MIN(start_y, end_y), yhi = MAX(start_y, end_y);
        /* destination extent in raw (output_pixel) coords */
        int Dxl = xlo + move_x, Dxh = xhi + move_x;
        int Dyl = ylo + move_y, Dyh = yhi + move_y;
        bool clip_ok = true;

        if (s->clip_mode != 0) {
            NPClipRect cr[4];
            if (newport_clip_subrects(s, cr) != 1) {
                clip_ok = false;                 /* multi-rect copy → slow */
            } else {
                Dxl = MAX(Dxl, cr[0].x0); Dxh = MIN(Dxh, cr[0].x1);
                Dyl = MAX(Dyl, cr[0].y0); Dyh = MIN(Dyh, cr[0].y1);
            }
        }

        if (clip_ok) {
            bool empty = (Dxl > Dxh) || (Dyl > Dyh);
            bool inb = true;
            if (!empty) {
                int dsx0 = Dxl + winx - 0x1000, dsx1 = Dxh + winx - 0x1000;
                int ssx0 = (Dxl - move_x) + winx - 0x1000;
                int ssx1 = (Dxh - move_x) + winx - 0x1000;
                int dsy0 = Dyl + winy - 0x1000, dsy1 = Dyh + winy - 0x1000;
                int ssy0 = (Dyl - move_y) + winy - 0x1000;
                int ssy1 = (Dyh - move_y) + winy - 0x1000;
                inb = dsx0 >= 0 && dsx1 < NEWPORT_VRAM_W &&
                      ssx0 >= 0 && ssx1 < NEWPORT_VRAM_W &&
                      dsy0 >= 0 && dsy1 < NEWPORT_VRAM_H &&
                      ssy0 >= 0 && ssy1 < NEWPORT_VRAM_H;
            }

            if (inb) {
                bool vmode = newport_2d_verify() && !np_verify_reported &&
                             !empty;
                int vry0 = 0, vry1 = -1;
                if (vmode) {
                    vry0 = Dyl + winy - 0x1000;
                    vry1 = Dyh + winy - 0x1000;
                    if (vry0 < 0) vry0 = 0;
                    if (vry1 >= NEWPORT_VRAM_H) vry1 = NEWPORT_VRAM_H - 1;
                    if (vry0 <= vry1) {
                        size_t nb = (size_t)(vry1 - vry0 + 1) *
                                    NEWPORT_VRAM_W * 4;
                        sf_snap = g_malloc(nb);
                        sf_fast = g_malloc(nb);
                        memcpy(sf_snap,
                               &s->vram_rgbci[(size_t)vry0 * NEWPORT_VRAM_W],
                               nb);
                    }
                }
                newport_2d_stat(NP2D_SCR2SCR_FAST);
                if (!empty) {
                    uint32_t *buf = s->vram_rgbci;
                    uint32_t mask = s->write_mask & s->global_mask;
                    int row_w = Dxh - Dxl + 1;
                    int dst_x0 = Dxl + winx - 0x1000;
                    int src_x0 = (Dxl - move_x) + winx - 0x1000;
                    int Yr_start = (dy > 0) ? Dyl : Dyh;
                    int Yr_end   = (dy > 0) ? Dyh : Dyl;
                    int Yr;

                    for (Yr = Yr_start; (dy > 0) ? Yr <= Yr_end : Yr >= Yr_end;
                         Yr += dy) {
                        int dst_y = Yr + winy - 0x1000;
                        int src_y = (Yr - move_y) + winy - 0x1000;
                        newport_rowhist_bump(dst_y);
                        uint32_t *drow =
                            &buf[(size_t)dst_y * NEWPORT_VRAM_W + dst_x0];
                        uint32_t *srow =
                            &buf[(size_t)src_y * NEWPORT_VRAM_W + src_x0];
                        if (mask == 0xffffffff) {
                            memmove(drow, srow, (size_t)row_w * 4);
                        } else {
                            uint32_t nm = ~mask;
                            int xi;
                            if (move_x <= 0) {   /* dest left of src: L→R */
                                for (xi = 0; xi < row_w; xi++) {
                                    drow[xi] = (drow[xi] & nm) | (srow[xi] & mask);
                                }
                            } else {             /* dest right of src: R→L */
                                for (xi = row_w - 1; xi >= 0; xi--) {
                                    drow[xi] = (drow[xi] & nm) | (srow[xi] & mask);
                                }
                            }
                        }
                    }
                    newport_dirty_rect(s, Dxl, Dyl, row_w, Dyh - Dyl + 1);
                }
                if (vmode && sf_fast && vry0 <= vry1) {
                    /* capture fast result, restore VRAM, fall through to slow
                     * (ground truth); compare at end of function. */
                    size_t nb = (size_t)(vry1 - vry0 + 1) * NEWPORT_VRAM_W * 4;
                    memcpy(sf_fast,
                           &s->vram_rgbci[(size_t)vry0 * NEWPORT_VRAM_W], nb);
                    memcpy(&s->vram_rgbci[(size_t)vry0 * NEWPORT_VRAM_W],
                           sf_snap, nb);
                    sf_ry0 = vry0; sf_ry1 = vry1;
                    sf_wx0 = Dxl + winx - 0x1000;
                    sf_wx1 = Dxh + winx - 0x1000;
                    /* fall through to the faithful slow loop below */
                } else {
                    /* register end-state (independent of clip) */
                    newport_write_x_start(s, (int32_t)s->x_save_int << 11);
                    newport_write_y_start(s, (int32_t)(end_y + dy) << 11);
                    return;
                }
            }
        }
    }

    newport_2d_stat(NP2D_SCR2SCR_SLOW);
    if (!newport_2d_force_slow()) {
        NPClipRect cr[4];
        if (!(stop_on_x && stop_on_y)) newport_2d_diag(DIAG_SS_STOP);
        else if (s->dm1_logicop != 0x3) newport_2d_diag(DIAG_SS_LOGIC);
        else if (s->dm0_colorhost) newport_2d_diag(DIAG_SS_HOST);
        else if (s->dm0_lspattern || s->dm0_zpattern) newport_2d_diag(DIAG_SS_PAT);
        else if (s->drawmode0 & DM0_SHADE) newport_2d_diag(DIAG_SS_SHADE);
        else if (!(s->dm1_planes == 1 || s->dm1_planes == 2)) newport_2d_diag(DIAG_SS_PLANE);
        else if (s->clip_mode != 0 && newport_clip_subrects(s, cr) != 1)
            newport_2d_diag(DIAG_SS_CLIP);
        else newport_2d_diag(DIAG_SS_OOB);
    }

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

    if (sf_ry1 >= sf_ry0 && sf_fast) {
        newport_verify_cmp(s, sf_fast, sf_ry0, sf_ry1, "scr2scr",
                           sf_wx0, sf_ry0, sf_wx1, sf_ry1, 0);
    }
    g_free(sf_snap);
    g_free(sf_fast);
}

/*
 * Read a single pixel from VRAM at current (x_start_i, y_start_i),
 * advance position, and return the pixel value.
 * MAME ref: do_pixel_read() at newport.cpp:3071-3136
 */
static uint32_t newport_read_one_pixel(SGINewportVirtuixState *s)
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
static void newport_do_pixel_read(SGINewportVirtuixState *s)
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
static void newport_do_rex3_command(SGINewportVirtuixState *s)
{
    uint8_t opcode = DM0_OPCODE(s->drawmode0);
    uint8_t adrmode = DM0_ADRMODE(s->drawmode0);
    /*
     * Phase D dirty tracking.  Capture the command's coordinate extent BEFORE
     * the primitive runs (primitives advance x_start/y_start as they draw), and
     * remember the dirty-touch counter so we can tell afterwards whether the
     * primitive already reported an exact bbox (fast fills / scr2scr do).  If it
     * did not (the slow per-pixel paths), we fall back to a conservative bbox
     * derived from the start/end registers (over-approximation is always safe —
     * the incremental scanout just repaints a little extra).
     */
    int cap_sx = s->x_start_int, cap_sy = s->y_start_int;
    int cap_ex = s->x_end_int,   cap_ey = s->y_end_int;
    int cap_mx = (int16_t)((s->xy_move >> 16) & 0xffff);
    int cap_my = (int16_t)(s->xy_move & 0xffff);
    uint32_t touch_before = s->dirty_touch;

    trace_sgi_newport_rex3_cmd(s->drawmode0, s->drawmode1);

    /* DOSETUP (DM0 bit 5): compute octant from coordinates before draw.
     * MAME ref: lines 3334, 3340, 3516 — called for every command type. */
    if (s->drawmode0 & DM0_DOSETUP) {
        newport_do_setup(s);
    }

    switch (opcode) {
    case DM0_OP_NOOP:
        newport_2d_stat(NP2D_NOOP);
        break;
    case DM0_OP_READ:
        newport_2d_stat(NP2D_READ);
        newport_do_pixel_read(s);
        break;
    case DM0_OP_DRAW:
        /* span/block classify (fast/slow, image) inside the primitive */
        switch (adrmode) {
        case DM0_ADR_SPAN:
            newport_draw_span(s);
            break;
        case DM0_ADR_BLOCK:
            newport_draw_block(s);
            break;
        case DM0_ADR_ILINE:
            newport_2d_stat(NP2D_ILINE);
            newport_draw_iline(s);
            break;
        case DM0_ADR_FLINE:
            newport_2d_stat(NP2D_FLINE);
            newport_draw_fline(s);
            break;
        default:
            newport_2d_stat(NP2D_DRAW_OTHER);
            qemu_log_mask(LOG_UNIMP,
                          "newport: unimplemented draw adrmode %d\n", adrmode);
            break;
        }
        break;
    case DM0_OP_SCR2SCR:
        /* scr2scr classify (fast/slow) inside the primitive */
        newport_draw_scr2scr(s);
        break;
    }

    /*
     * Phase D: if the primitive did not already report an exact dirty bbox
     * (i.e. it went down a slow per-pixel path), mark a conservative bbox for
     * the drawing opcodes.  READ/NOOP touch no pixels, so they dirty nothing.
     */
    if (s->dirty_touch == touch_before &&
        (opcode == DM0_OP_DRAW || opcode == DM0_OP_SCR2SCR)) {
        int x0 = MIN(cap_sx, cap_ex), x1 = MAX(cap_sx, cap_ex);
        int y0 = MIN(cap_sy, cap_ey), y1 = MAX(cap_sy, cap_ey);
        if (opcode == DM0_OP_SCR2SCR) {
            /* the write target is the source extent shifted by the move vector */
            x0 += cap_mx; x1 += cap_mx;
            y0 += cap_my; y1 += cap_my;
        }
        /* raw draw coords are already scanout/dst (screen) space; the window
         * offset only relocates the VRAM write, which the scanout applies on
         * read (BL-83) */
        newport_dirty_rect(s, x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    }
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);
    s->status &= ~REX3_STATUS_BACKBUSY;
}

/*
 * VRINT deassert callback — lowers the IRQ line and re-enables
 * VRINT generation for the next frame.
 */
static void newport_vrint_deassert(void *opaque)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);
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
static uint32_t newport_dcb_read(SGINewportVirtuixState *s)
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
 * True for the VC2 registers that move/enable the hardware cursor. Writing
 * these changes no VRAM, so the display must be forced dirty for the cursor
 * to track at the refresh rate (see newport_update_display's dirty gate).
 */
static inline bool vc2_reg_affects_cursor(uint8_t idx)
{
    return idx == VC2_CURSOR_X || idx == VC2_CURSOR_Y ||
           idx == VC2_CURSOR_ENTRY || idx == VC2_DC_CONTROL;
}

/*
 * Handle DCB write to sub-devices.
 */
static void newport_dcb_write(SGINewportVirtuixState *s, uint32_t val)
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
                    if (vc2_reg_affects_cursor(s->vc2_reg_idx)) {
                        newport_dirty_full(s);
                        newport_update_hw_cursor(s);
                    } else if (s->vc2_reg_idx == VC2_DID_ENTRY ||
                               s->vc2_reg_idx == VC2_DC_CONTROL) {
                        /* Phase D: DID table base / DID-enable change re-maps every
                         * scanline's visual → the whole screen must be re-walked,
                         * and the per-row resolution cache now describes a
                         * different table. */
                        newport_did_cache_drop(s);
                        newport_dirty_full(s);
                    }
                }
                break;
            case 1: /* Register data write */
                s->vc2_reg_data = val;
                if (s->vc2_reg_idx < 32) {
                    s->vc2_reg[s->vc2_reg_idx] = vc2_data;
                    if (s->vc2_reg_idx == VC2_DID_ENTRY ||
                        s->vc2_reg_idx == VC2_DC_CONTROL) {
                        newport_did_cache_drop(s);
                        newport_dirty_full(s);   /* Phase D: see above */
                    }
                    /*
                     * The VC2 hardware cursor is moved by writing CURSOR_X/Y
                     * (and enabled via DC_CONTROL). These touch no VRAM, so
                     * without forcing a redraw the periodic display update
                     * (gated on display_dirty) would never re-composite the
                     * cursor at its new position — the pointer would only
                     * "jump" when X redraws its shape. Mark dirty so the
                     * cursor tracks at the display refresh rate.
                     */
                    if (vc2_reg_affects_cursor(s->vc2_reg_idx)) {
                        newport_dirty_full(s);
                    }
                }
                break;
            case 2: /* RAM address */
                s->vc2_ram_addr = vc2_data & 0x7fff;
                break;
            case 3: /* SRAM data write — auto-increment */
                if (s->vc2_ram_addr < 32768) {
                    if (s->vc2_ram[s->vc2_ram_addr] != vc2_data) {
                        s->vc2_ram[s->vc2_ram_addr] = vc2_data;
                        /*
                         * Phase D — THE #1 pitfall.  The per-scanline DID frame
                         * and line tables live in VC2 RAM (written through this
                         * DCB data port, NOT via VC2 registers).  Xsgi rewrites
                         * them whenever windows move / restack, changing which
                         * visual each scanline shows with no REX3 draw at all.
                         * So the change MUST invalidate — but bounding it matters:
                         * this site alone produced 99.05 % of all whole-screen
                         * invalidations (814 per rendered frame) during a window-op
                         * storm, which kept the desktop permanently full-dirty and
                         * made Phase D's bounded scanout inoperative
                         * (progress_notes/ip55/pvdisplay/27-bl81-bounded-repaint.md
                         * §6d).  Instead of invalidating here, accumulate the
                         * changed word's ADDRESS into a pending range and let
                         * newport_vc2_ram_resolve() (run once per render) map it to
                         * the scanlines whose DID walk consults those words.  That
                         * both bounds the invalidation and coalesces the burst —
                         * Xsgi writes these tables as runs through this
                         * auto-incrementing port, so one resolution replaces
                         * hundreds of full-invalidates.  Cursor sprite RAM lives
                         * here too; the resolver detects that range and keeps the
                         * old full-invalidate for it (rare event).
                         */
                        s->vc2_ram_writes++;
                        if (newport_vc2_full_oracle()) {
                            /* PVDISPLAY_VC2_FULL=1 — restore the pre-fix
                             * per-word whole-screen invalidate.  The one-binary
                             * A/B control for this change (and the escape hatch
                             * if a DID/colormap artifact ever shows up). */
                            newport_dirty_full(s);
                            s->vc2_ram_addr = (s->vc2_ram_addr + 1) & 0x7fff;
                            break;
                        }
                        if (s->vc2_ram_dirty_lo > s->vc2_ram_dirty_hi) {
                            s->vc2_ram_dirty_lo = s->vc2_ram_addr;
                            s->vc2_ram_dirty_hi = s->vc2_ram_addr;
                        } else {
                            if (s->vc2_ram_addr < s->vc2_ram_dirty_lo) {
                                s->vc2_ram_dirty_lo = s->vc2_ram_addr;
                            }
                            if (s->vc2_ram_addr > s->vc2_ram_dirty_hi) {
                                s->vc2_ram_dirty_hi = s->vc2_ram_addr;
                            }
                        }
                        s->dirty_touch++;    /* parity with the invalidators */
                        s->display_dirty = true;
                    }
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
                /* Only invalidate when the entry actually CHANGES.  Xsgi rewrites
                 * whole palette blocks with identical values, and this site was the
                 * #2 whole-screen-invalidation source once the VC2-SRAM one was
                 * bounded: 11 341 invalidations (~7 per rendered frame) in one
                 * wmbench run — on its own enough to keep the desktop full-dirty. */
                if (s->cmap0_palette[s->cmap_palette_idx] !=
                    (uint16_t)(masked_val >> 8)) {
                    s->cmap0_palette[s->cmap_palette_idx] = masked_val >> 8;
                    if (newport_vc2_full_oracle()) {
                        newport_dirty_full(s);      /* pre-fix granularity */
                    } else {
                        /* Record the 256-entry bucket; newport_pal_resolve() (once
                         * per render) turns the burst into the rows that read it. */
                        s->pal_dirty_mask |=
                            1u << ((s->cmap_palette_idx >> 8) & 31);
                        s->dirty_touch++;
                        s->display_dirty = true;
                    }
                }
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
            /* Phase D: affects cursor colours.  Guarded on an actual change —
             * re-writing the same value repaints nothing (same reasoning as the
             * CMAP palette site above). */
            if (s->xmap_cursor_cmap != (uint8_t)val) {
                s->xmap_cursor_cmap = (uint8_t)val;
                newport_dirty_full(s);
            }
            break;
        case 4: /* Popup CMAP MSB — MAME ref: xmap9::write() CRS=4 */
            if (s->xmap_popup_cmap != (uint8_t)val) {   /* recolours popup planes */
                s->xmap_popup_cmap = (uint8_t)val;
                newport_dirty_full(s);
            }
            break;
        case 5: /* Mode table write — MAME ref: xmap9::write() CRS=5 */
            /*
             * val bits [28:24] = mode table index
             * val bits [23:0]  = mode entry value
             */
            /* Phase D: a mode-table change re-resolves every scanline's pixel
             * mode / CI-MSB → whole screen, and the per-row DID resolution cache
             * is keyed on mode-table INDICES, so it must be dropped too. */
            if (s->xmap_mode_table[(val >> 24) & 0x1f] != (val & 0xffffff)) {
                s->xmap_mode_table[(val >> 24) & 0x1f] = val & 0xffffff;
                newport_did_cache_drop(s);
                newport_dirty_full(s);
            }
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
            /* Phase D: the gamma LUT recolours every already-drawn pixel with no
             * REX3 draw — full-invalidate so the incremental path re-applies it.
             * Guarded on an actual change: Xsgi rewrites the whole 256-entry LUT
             * with identical values (measured 1024 invalidations, one per rendered
             * frame, in one wmbench run). */
            if (s->ramdac_lut_r[s->ramdac_lut_index] != (uint8_t)(val >> 24) ||
                s->ramdac_lut_g[s->ramdac_lut_index] != (uint8_t)(val >> 16) ||
                s->ramdac_lut_b[s->ramdac_lut_index] != (uint8_t)(val >> 8)) {
                s->ramdac_lut_r[s->ramdac_lut_index] = (uint8_t)(val >> 24);
                s->ramdac_lut_g[s->ramdac_lut_index] = (uint8_t)(val >> 16);
                s->ramdac_lut_b[s->ramdac_lut_index] = (uint8_t)(val >> 8);
                newport_dirty_full(s);
            }
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
static void newport_decode_drawmode1(SGINewportVirtuixState *s)
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
static void newport_decode_drawmode0(SGINewportVirtuixState *s)
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

static uint64_t sgi_newport_virtuix_read(void *opaque, hwaddr addr, unsigned size)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);
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

static void sgi_newport_virtuix_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);
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

    /*
     * BL-44 host-data handshake trace.  Log every REX3 write that occurs while a
     * colorhost image-blit is in flight (or the DRAWMODE0 write that toggles it),
     * so foreign interleaving (context-switch / another gfx client) and the
     * HOSTRW push cadence are visible relative to the per-primitive draw lines.
     */
    if (newport_hostrw_trace() &&
        (s->dm0_colorhost || reg == REX3_DRAWMODE0)) {
        fprintf(stderr, "NP_HRW WR reg=0x%03x val=0x%08x go=%d shift=%u ch=%d\n",
                (unsigned)reg, (uint32_t)val, is_go, s->host_shift,
                s->dm0_colorhost);
    }

    switch (reg) {
    /* Drawing registers */
    case REX3_DRAWMODE0: {
        bool was_ch = s->dm0_colorhost;
        s->drawmode0 = val;
        newport_decode_drawmode0(s);
        /* BL-44 discriminator: a colorhost episode ends when colorhost drops. */
        if (newport_hostrw_count() && was_ch && !s->dm0_colorhost) {
            newport_hostrw_count_flush();
        }
        break;
    }
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
        /*
         * BL-82b: in RGB mode the packed-color write also loads the RGB
         * iterators (packed layout: R low byte, G mid, B high; iterator
         * integer part sits at bits [19:11]).  Xsgi programs stipple/pattern
         * foregrounds through this register, so without the expansion
         * every stippled fill on an RGB window drew with zeroed iterators
         * (= black, regardless of fg).  MAME ref: gio64/newport.cpp
         * case 0x0220/8 lower half.
         */
        if (s->dm1_rgbmode) {
            s->color_red   = (val & 0xff) << 11;
            s->color_green = (val & 0xff00) << 3;
            s->color_blue  = (val & 0xff0000) >> 5;
            s->curr_color_red   = s->color_red;
            s->curr_color_green = s->color_green;
            s->curr_color_blue  = s->color_blue;
        }
        break;
    case REX3_ZEROOVERFLOW:
        s->zero_overflow = val;
        break;
    case REX3_HOSTRW0:
        if (s->dm0_colorhost && newport_hostrw_count()) {
            hc_words++;         /* BL-44: word RECEIVED by the device */
        }
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

static const MemoryRegionOps sgi_newport_virtuix_ops = {
    .read = sgi_newport_virtuix_read,
    .write = sgi_newport_virtuix_write,
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

/* BL-83: the visible scanout window in VRAM is offset by the XYWIN window
 * origin (winx - 0x1000).  Xsgi programs winx = 0x1000 + bt445_xbias (2 for
 * the BT445 Rev A "extra black pixels" workaround, see ng1_init.c
 * Ng1DacInit), leaving that many black bias columns at the start of each VRAM
 * scanline that the real scanout skips.  Reading VRAM from x=0 shifted the
 * whole desktop +2px right and clipped 2 columns off the right edge.
 * (Y is not biased: the timing tables fold bt445_xbias into winx only, winy
 * stays 0x1000.) */
static inline int newport_scanout_xoff(const SGINewportVirtuixState *s)
{
    int xoff = (int16_t)(s->xy_window >> 16) - 0x1000;
    /* Guard the pre-init state: xy_window resets to 0 (winx=0), giving -4096,
     * far outside the VRAM window.  Only a small bias is ever valid — the
     * bt445_xbias is at most the VRAM/screen width delta (64) — so anything
     * outside [0, VRAM_W - SCREEN_W) falls back to the un-biased origin. */
    if (xoff < 0 || xoff >= NEWPORT_VRAM_W - NEWPORT_SCREEN_W) {
        return 0;
    }
    return xoff;
}

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
static void newport_dump_vram_ppm(SGINewportVirtuixState *s, const char *path)
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

    int xoff = newport_scanout_xoff(s);
    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        const uint32_t *src_rgbci = &s->vram_rgbci[y * NEWPORT_VRAM_W + xoff];
        const uint32_t *src_cidaux = &s->vram_cidaux[y * NEWPORT_VRAM_W + xoff];
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
            /* BL-83: skip DID transitions in the pre-window bias columns */
            while ((uint16_t)(next_did_entry >> 5) <= (uint16_t)xoff) {
                mode_entry = s->xmap_mode_table[next_did_entry & 0x1f];
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
        }

        for (x = 0; x < NEWPORT_SCREEN_W; x++) {
            uint32_t pixel = src_rgbci[x];
            uint32_t cidaux = src_cidaux[x];
            uint32_t rgb;
            uint8_t r, g, b;

            if (use_did && (uint16_t)(x + xoff) == (next_did_entry >> 5)) {
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
    SGINewportVirtuixState *s = opaque;
    newport_dirty_full(s);
}

/*
 * Phase E: build a QEMUCursor from VC2 cursor state and push it to the
 * unified display engine's console via dpy_cursor_define + dpy_mouse_set.
 * Kept behind PVDISPLAY_SOFT_CURSOR=1 — when set, the software cursor
 * (newport_draw_cursor) is used instead of the hardware cursor.
 */
static void newport_update_hw_cursor(SGINewportVirtuixState *s)
{
    static int soft_cursor = -1;
    /* Track cursor content to avoid redundant dpy_cursor_define per frame */
    static uint16_t last_entry = 0xffff, last_cmap = 0xffff;
    static bool     last_64 = false;
    static uint32_t last_ram_hash = 0;
    QemuConsole *con;
    uint16_t dc, cx, cy, cursor_entry, cursor_msb;
    bool is_64, visible, content_changed;
    int size, gx, gy, sx, sy;
    uint32_t ram_hash;
    QEMUCursor *cur;

    if (soft_cursor < 0) {
        soft_cursor = getenv("PVDISPLAY_SOFT_CURSOR") ? 1 : 0;
    }
    if (soft_cursor) return;

    con = sgi_glaccel_get_console();
    if (!con) return;

    dc = s->vc2_reg[VC2_DC_CONTROL];
    visible = (dc & VC2_DC_ENA_CURSOR) && (dc & VC2_DC_CURSOR_DISP);
    cx = (int16_t)s->vc2_reg[VC2_CURSOR_X];
    cy = (int16_t)s->vc2_reg[VC2_CURSOR_Y];
    cursor_entry = s->vc2_reg[VC2_CURSOR_ENTRY];
    is_64 = dc & VC2_DC_CURSOR_SIZE64;
    size = is_64 ? 64 : 32;
    cursor_msb = (uint16_t)s->xmap_cursor_cmap << 5;

    /* NP_CURSOR trace */
    {
        static int np_cursor_trace = -1;
        if (np_cursor_trace < 0)
            np_cursor_trace = getenv("NP_CURSOR") ? 1 : 0;
        if (np_cursor_trace) {
            static int16_t last_cx = -32768, last_cy = -32768;
            if (cx != last_cx || cy != last_cy) {
                last_cx = cx; last_cy = cy;
                fprintf(stderr, "cursor=(%d,%d)\n", cx, cy);
            }
        }
    }

    if (!visible) {
        dpy_mouse_set(con, cx, cy, false);
        return;
    }

    /* Detect content change: entry, cmap, size, or RAM content */
    ram_hash = 0;
    {
        int n_words = is_64 ? 256 : 128;  /* 64x64=4 words/row*64, 32x32=2 words/row*32=64 words * 2 planes=128 */
        for (int i = 0; i < n_words; i++)
            ram_hash = ram_hash * 31 + s->vc2_ram[(cursor_entry + i) & 0x7fff];
    }
    content_changed = (cursor_entry != last_entry || cursor_msb != last_cmap ||
                       is_64 != last_64 || ram_hash != last_ram_hash);
    if (content_changed) {
        last_entry = cursor_entry; last_cmap = cursor_msb;
        last_64 = is_64; last_ram_hash = ram_hash;
    }

    if (!content_changed) {
        dpy_mouse_set(con, cx, cy, true);  /* position-only update */
        return;
    }

    /* Build QEMUCursor: allocate, fill ARGB data */
    cur = cursor_alloc(size, size);
    /* hotspot: VC2 CURSOR_X/Y is the bottom-right corner (anchor).
     * QEMU positions the hotspot, so hot = size-1 per the plan doc. */
    cur->hot_x = cur->hot_y = size - 1;

    for (gy = 0; gy < size; gy++) {
        sy = cy - (size - 1) + gy;
        for (gx = 0; gx < size; gx++) {
            uint8_t pixel = 0;
            int shift = 15 - (gx & 15);
            uint32_t argb = 0;  /* transparent */

            if (sy >= 0 && sy < NEWPORT_SCREEN_H) {
                sx = cx - (size - 1) + gx;
                if (sx >= 0 && sx < NEWPORT_SCREEN_W) {
                    if (is_64) {
                        int addr = gy * 4 + (gx / 16);
                        uint16_t word =
                            s->vc2_ram[(cursor_entry + addr) & 0x7fff];
                        pixel = (word >> shift) & 1;
                    } else {
                        int addr = gy * 2 + (gx / 16);
                        uint16_t w0 =
                            s->vc2_ram[(cursor_entry + addr) & 0x7fff];
                        uint16_t w1 =
                            s->vc2_ram[(cursor_entry + addr + 64) & 0x7fff];
                        pixel = ((w0 >> shift) & 1) |
                                (((w1 >> shift) & 1) << 1);
                    }
                }
            }
            if (pixel) {
                uint32_t rgb = s->cmap0_palette[
                    (cursor_msb | (pixel & 0x3)) & 0x1fff];
                uint8_t r = s->ramdac_lut_r[(rgb >> 16) & 0xff];
                uint8_t g = s->ramdac_lut_g[(rgb >> 8) & 0xff];
                uint8_t b = s->ramdac_lut_b[rgb & 0xff];
                argb = (0xffu << 24) | (r << 16) | (g << 8) | b;
            }
            cur->data[(size_t)gy * size + gx] = argb;
        }
    }

    dpy_cursor_define(con, cur);
    cursor_unref(cur);  /* console holds its own ref */
    dpy_mouse_set(con, cx, cy, true);
}

/*
 * Draw hardware cursor overlay onto the display surface.
 * Supports 32x32 (2-plane, 4-color) and 64x64 (1-plane, monochrome) cursors.
 * MAME ref: get_cursor_pixel() at newport.cpp:782-814
 */
static void newport_draw_cursor(SGINewportVirtuixState *s, uint32_t *dest)
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

    /*
     * Virtuix-only diagnostic: emit the VC2 hardware-cursor (x,y) to stderr when
     * env NP_CURSOR is set, so host-side tooling can closed-loop servo the guest
     * cursor to an exact position (immune to X11 pointer acceleration). Off by
     * default -> zero overhead/behavior change. Indy's sgi_newport.c has no such
     * trace (kept pristine).
     */
    {
        static int np_cursor_trace = -1;
        if (np_cursor_trace < 0) {
            np_cursor_trace = getenv("NP_CURSOR") ? 1 : 0;
        }
        if (np_cursor_trace) {
            static int16_t last_cx = -32768, last_cy = -32768;
            if (cx != last_cx || cy != last_cy) {
                last_cx = cx;
                last_cy = cy;
                fprintf(stderr, "cursor=(%d,%d)\n", cx, cy);
            }
        }
    }

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
/*
 * NP_SCANOUT_FULL=1 is the Phase D A/B oracle: it forces the full-screen
 * per-pixel regen (the pre-Phase-D behaviour) on every dirty frame, so the
 * incremental scanout can be diffed byte-for-byte against it.  Like the Phase C
 * oracle it must change nothing else.
 */
static int np_scanout_full = -1;
static inline bool newport_scanout_full(void)
{
    if (np_scanout_full < 0) {
        np_scanout_full = getenv("NP_SCANOUT_FULL") ? 1 : 0;
    }
    return np_scanout_full;
}

/* PVDisplay session 22 (BL-43): disable the VRAM->shadow seed at flip
 * activation (A/B escape hatch — reproduces the pre-session-22 blank-shadow
 * behaviour where pre-flip app-cached content was lost to index-0 black). */
static inline bool newport_scanout_noseed(void)
{
    static int v = -1;
    if (v < 0) {
        v = getenv("NP_SCANOUT_NOSEED") ? 1 : 0;
    }
    return v;
}

/*
 * NEWPORT_DIRTYFULL_STATS=1 — per-call-site histogram of whole-screen
 * invalidations, printed once per N renders (N = the value, default 512).
 * "Which site saturated the dirty list?" is the first question whenever bounded
 * scanout stops paying, and guessing it wrong cost note 27 two probe binaries.
 * Process-wide statics: this is a diagnostic, not device state.
 */
#define NEWPORT_DFULL_SITES 32
static struct { int line; uint64_t n; } newport_dfull_site[NEWPORT_DFULL_SITES];
static uint64_t newport_dfull_total;

static bool newport_dfull_stats_on(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("NEWPORT_DIRTYFULL_STATS");
        on = (e && *e && strcmp(e, "0") != 0) ? 1 : 0;
    }
    return on != 0;
}

/* Phase D: saturate the dirty-rect list to full-screen.  Used by the broad
 * invalidators (palette / DID-table base / mode-table / RAMDAC-LUT changes,
 * cursor moves, backend invalidate) whose effect can't be bounded to a small
 * rect.  NOTE: the VC2-SRAM (DID table) write path no longer comes here — it
 * bounds itself via newport_vc2_ram_resolve() below. */
static void newport_dirty_full_at(SGINewportVirtuixState *s, int line)
{
    if (newport_dfull_stats_on()) {
        int i;
        newport_dfull_total++;
        for (i = 0; i < NEWPORT_DFULL_SITES; i++) {
            if (newport_dfull_site[i].line == line) {
                newport_dfull_site[i].n++;
                break;
            }
            if (newport_dfull_site[i].line == 0) {
                newport_dfull_site[i].line = line;
                newport_dfull_site[i].n = 1;
                break;
            }
        }
    }
    /* Note 29: dirty_full is the saturation signal (a full LIST is now an
     * ordinary bounded state — see newport_dirty_rect).  dirty_n is still
     * saturated so any code reading it as "everything" keeps working. */
    s->dirty_full = true;
    s->dirty_n = NEWPORT_DIRTY_MAX;
    s->display_dirty = true;
    s->dirty_touch++;
}

/*
 * Phase D: add a dirty rect (post-window-offset VRAM space == scanout/dst
 * space).  Called by every drawing primitive (fast paths report an exact bbox,
 * slow paths an over-approximation) and by the VC2/palette resolvers' row runs.
 *
 * Note 29 — COALESCING, not capacity.  The original rule was "union with an
 * existing rect only if it OVERLAPS, else append, and saturate to full-screen
 * when the list fills".  Note 28 measured that rule saturating on 415 of 1181
 * remaining full renders even with the list at 64 slots, and note 27 §6d had
 * already refuted growing the list (16 → 64 bought ~0: `rrects` 271 → 278).
 * Capacity is not the problem — the *merge rule* is.  Two fixes:
 *
 *  (1) Merge on CHEAPNESS, not overlap.  For each candidate the added area of
 *      the union ("waste") is computed; the cheapest merge is taken when its
 *      waste is under NEWPORT_DIRTY_WASTE.  Overlap is the waste≈0 case, so this
 *      strictly subsumes the old rule, and it also folds the vertically-adjacent
 *      full-width row runs the VC2 resolver emits (waste = the gap between them)
 *      which the overlap test could never merge.
 *  (2) A FULL LIST NO LONGER SATURATES.  The new rect is merged into whichever
 *      slot it is cheapest to widen — the list stays bounded at
 *      NEWPORT_DIRTY_MAX while coverage only ever grows, which is the safe
 *      direction (repaint more, never less).
 *
 * The only remaining route to full-screen from here is the AREA guard: once the
 * rects cover most of the screen, N bounded row walks plus N bounded blits cost
 * more than one straight walk, so saturating is the cheaper *and* wider choice.
 * Counted separately (rd_area) so it never hides inside "sat".
 *
 * Cost: O(dirty_n) per call with dirty_n ≤ 64 and integer arithmetic only — the
 * same order as the old overlap scan, which also walked the whole list.
 */
#define NEWPORT_DIRTY_WASTE  (NEWPORT_SCREEN_W * 8)   /* ≈8 full rows of slack */
/* Saturate to full once the accumulated rects cover this fraction (>= 3/4) of
 * the screen: at that point the bounded path is not cheaper than a full walk. */
#define NEWPORT_DIRTY_AREA_FULL \
    ((int64_t)NEWPORT_SCREEN_W * NEWPORT_SCREEN_H * 3 / 4)

static int64_t newport_dirty_area(SGINewportVirtuixState *s)
{
    int64_t a = 0;
    int i;
    for (i = 0; i < s->dirty_n; i++) {
        a += (int64_t)s->dirty_rects[i].w * s->dirty_rects[i].h;
    }
    return a;
}

static void
newport_dirty_rect(SGINewportVirtuixState *s, int rx, int ry, int rw, int rh)
{
    int i, best = -1;
    int64_t best_waste = 0;
    s->dirty_touch++;
    if (rw <= 0 || rh <= 0) return;
    if (s->dirty_full) {
        s->display_dirty = true;    /* already saturated to full-screen */
        return;
    }
    /* clip to screen */
    if (rx < 0) { rw += rx; rx = 0; }
    if (ry < 0) { rh += ry; ry = 0; }
    if (rx + rw > NEWPORT_SCREEN_W) rw = NEWPORT_SCREEN_W - rx;
    if (ry + rh > NEWPORT_SCREEN_H) rh = NEWPORT_SCREEN_H - ry;
    if (rw <= 0 || rh <= 0) return;
    s->display_dirty = true;

    /*
     * PVDISPLAY_DIRTY_SHRINK=<n> — the DETECTOR SANITY control (note 29 §5).
     *
     * Every deliberate narrowing of a specific invalidation request tried so far
     * (VC2 row runs shrunk by 40, VC2 row runs dropped entirely, BL-81 geometry
     * rects inset by 40, each with and without the coalescer) leaves
     * PVDISPLAY_BOUND_VERIFY reporting ZERO mismatching pixels.  Two readings are
     * possible and they matter very differently: either the oracle cannot see an
     * under-repaint at all (note 27 §3's "toothless probe"), or those particular
     * requests are REDUNDANT in the workload because the guest's own REX3 expose
     * redraws already report the same damage through this very function.
     *
     * This knob distinguishes them: it insets EVERY dirty rect — including the
     * REX3 primitives' — by n px on each side, so nothing in the incremental path
     * covers the border.  If the oracle still reports 0, it is blind and no
     * bounded-invalidation claim can rest on it.  If it fires, the oracle has
     * teeth and the earlier controls passed because they were redundant, not
     * because nothing is being checked.  Debug only, never on a shipping path.
     */
    {
        static int dshrink = -1;
        if (dshrink < 0) {
            const char *e = getenv("PVDISPLAY_DIRTY_SHRINK");
            dshrink = e ? atoi(e) : 0;
        }
        if (dshrink > 0) {
            int sx = rw > 2 * dshrink ? dshrink : 0;
            int sy = rh > 2 * dshrink ? dshrink : 0;
            rx += sx; rw -= 2 * sx;
            ry += sy; rh -= 2 * sy;
            if (rw <= 0 || rh <= 0) return;
        }
    }

    /* PVDISPLAY_DIRTY_NOCOALESCE=1— one-binary control: restore the pre-note-29
     * overlap-only merge that saturates on a full list, so the coalescer's effect
     * (and the correctness of what it merges) can be A/B'd in a single build. */
    {
        static int nocoal = -1;
        if (nocoal < 0) nocoal = getenv("PVDISPLAY_DIRTY_NOCOALESCE") ? 1 : 0;
        if (nocoal) {
            for (i = 0; i < s->dirty_n; i++) {
                int *dr = (int *)&s->dirty_rects[i];
                if (rx < dr[0] + dr[2] && rx + rw > dr[0] &&
                    ry < dr[1] + dr[3] && ry + rh > dr[1]) {
                    int x2 = MAX(rx + rw, dr[0] + dr[2]);
                    int y2 = MAX(ry + rh, dr[1] + dr[3]);
                    dr[0] = MIN(rx, dr[0]); dr[1] = MIN(ry, dr[1]);
                    dr[2] = x2 - dr[0];     dr[3] = y2 - dr[1];
                    return;
                }
            }
            if (s->dirty_n < NEWPORT_DIRTY_MAX) {
                s->dirty_rects[s->dirty_n].x = rx; s->dirty_rects[s->dirty_n].y = ry;
                s->dirty_rects[s->dirty_n].w = rw; s->dirty_rects[s->dirty_n].h = rh;
                s->dirty_n++;
            } else {
                newport_dirty_full(s);
            }
            return;
        }
    }

    /* cheapest merge candidate */
    for (i = 0; i < s->dirty_n; i++) {
        const int *dr = (const int *)&s->dirty_rects[i];
        int ux = MIN(rx, dr[0]), uy = MIN(ry, dr[1]);
        int ux2 = MAX(rx + rw, dr[0] + dr[2]);
        int uy2 = MAX(ry + rh, dr[1] + dr[3]);
        /* waste = union area - (area of the two, less their overlap) */
        int ox = MIN(rx + rw, dr[0] + dr[2]) - MAX(rx, dr[0]);
        int oy = MIN(ry + rh, dr[1] + dr[3]) - MAX(ry, dr[1]);
        int64_t overlap = (ox > 0 && oy > 0) ? (int64_t)ox * oy : 0;
        int64_t waste = (int64_t)(ux2 - ux) * (uy2 - uy)
                        - (int64_t)rw * rh - (int64_t)dr[2] * dr[3] + overlap;
        if (best < 0 || waste < best_waste) {
            best = i; best_waste = waste;
        }
    }

    if (best >= 0 && (best_waste <= NEWPORT_DIRTY_WASTE ||
                      s->dirty_n >= NEWPORT_DIRTY_MAX)) {
        int *dr = (int *)&s->dirty_rects[best];
        int x2 = MAX(rx + rw, dr[0] + dr[2]), y2 = MAX(ry + rh, dr[1] + dr[3]);
        dr[0] = MIN(rx, dr[0]); dr[1] = MIN(ry, dr[1]);
        dr[2] = x2 - dr[0];     dr[3] = y2 - dr[1];
    } else {
        s->dirty_rects[s->dirty_n].x = rx; s->dirty_rects[s->dirty_n].y = ry;
        s->dirty_rects[s->dirty_n].w = rw; s->dirty_rects[s->dirty_n].h = rh;
        s->dirty_n++;
    }

    /* area guard — the bounded path stopped being the cheap one */
    if (newport_dirty_area(s) >= NEWPORT_DIRTY_AREA_FULL) {
        s->rd_area++;
        newport_dirty_full(s);
    }
}

/*
 * ============================================================
 * Phase D / BL-81 follow-up: bounding the VC2 DID-table invalidation
 * ============================================================
 *
 * Xsgi rewrites the per-scanline DID frame/line tables (in VC2 SRAM, through the
 * DCB auto-incrementing data port) on every window move, resize, restack and
 * clip change.  Phase D's original broad invalidator called newport_dirty_full()
 * per changed word, which is correct but produced 814 whole-screen invalidations
 * per rendered frame during a window-op storm — 99.05 % of all of them — leaving
 * the desktop permanently full-dirty and the bounded scanout inoperative
 * (`progress_notes/ip55/pvdisplay/27-bl81-bounded-repaint.md` §6d).
 *
 * Instead the write handler only records that VC2 SRAM changed (plus the changed
 * address range, for the cursor-RAM test), and this resolver — run ONCE per render
 * — decides which scanlines that actually affected.  Two things had to be right,
 * and the first attempt only got one of them:
 *
 *  1. COALESCE THE BURST.  Xsgi writes these tables as long runs through the
 *     auto-incrementing data port: measured 1 239 884 changed words in 806 bursts
 *     (~1538 words/burst) over one wmbench run.  Resolving once per render turns
 *     807 whole-screen invalidations per frame into 0.5 resolutions per frame.
 *
 *  2. ASK THE RIGHT QUESTION.  Bounding by "which words were written" does NOT
 *     work: measured, it dirtied 1022 of 1024 rows per resolution, because a
 *     window op rewrites essentially the whole table (line tables get rebuilt at
 *     fresh addresses, so every frame-table pointer changes) even where the
 *     resulting per-scanline visual is identical.  What decides whether a row
 *     renders differently is its DID *resolution* — the sequence of entry words
 *     its walk consults — not which addresses hold them.  So we cache that
 *     sequence per row (note 01 Phase D item 2's "per-scanline mode cache") and
 *     dirty exactly the rows whose sequence changed.
 *
 * Direction of safety: this may dirty MORE rows than strictly required, never
 * fewer.  A row is dirtied whenever its consulted-word sequence differs from the
 * cached one, and every case that cannot be represented — DIDs disabled, no DID
 * base, the range touching cursor sprite RAM, a run longer than
 * NEWPORT_DID_SIGLEN, too many disjoint row runs — is wider (a row run spanning
 * everything marked, or the old whole-screen invalidate).  Dirtied rows are always
 * dirtied full-width.  The cache is dropped wholesale on DID-base, DC_CONTROL,
 * XMAP-mode-table, reset and post-load, i.e. everywhere its inputs change
 * out-of-band.
 *
 * NOT changed: the DID walk itself, the XMAP/CMAP/popup/overlay semantics, and
 * the VC2 *register* invalidators (DID base, cursor, mode table) — only the
 * granularity of the dirty marking for SRAM writes.
 */
#define NEWPORT_DID_MAXRUNS   24   /* disjoint row runs before merging to one */
#define NEWPORT_DID_RUNGAP    12   /* merge runs separated by <= this many
                                    * clean rows: a few extra rows cost far
                                    * less than an extra rect (and than
                                    * saturating the list to full-screen) */
#define NEWPORT_VC2_CURSOR_WORDS 256 /* cursor sprite RAM window (2 banks of 64
                                      * 2-word rows, rounded up) */

/*
 * Walk row y's DID entry run exactly as newport_convert_row() does and return the
 * sequence of entry words consulted, i.e. the row's DID *resolution*.  That
 * sequence (plus xmap_mode_table, which is invalidated separately) is the complete
 * input to how the row renders, so comparing it against the cached one answers the
 * only question that matters: did THIS row's rendering change?
 *
 * Returns the number of words written to out[], or -1 for "cannot represent"
 * (run longer than the cache) — which the caller treats as "always dirty".
 */
static int newport_did_row_sig(SGINewportVirtuixState *s, int y,
                              uint16_t did_entry_ptr, uint16_t *out,
                              uint32_t *pal_mask)
{
    uint16_t fa = (uint16_t)(did_entry_ptr + (uint16_t)y) & 0x7fff;
    uint16_t lp = s->vc2_ram[fa];
    int m;

    *pal_mask = 0;
    for (m = 0; m < NEWPORT_DID_SIGLEN; m++) {
        uint16_t w = s->vc2_ram[(uint16_t)(lp + m) & 0x7fff];
        out[m] = w;
        /* Which palette buckets this segment's mode can read.  Mirrors
         * newport_convert_row()'s index composition exactly:
         *   base CI (pix_mode 0): cmap0_palette[(ci_msb | ci) & 0x1fff], ci width
         *       4/8/12 bits by pix_size, ci_msb a multiple of 0x100 → the OR can
         *       reach buckets (ci_msb>>8) | k for k up to (ci_max>>8);
         *   overlay (aux_pix_mode != 0): cmap0_palette[(aux_msb | 0..3)], aux_msb
         *       a multiple of 0x100 → one bucket.
         *   RGB modes (pix_mode != 0) read no palette at all.
         * Segments after the first are included whether or not the walk reaches
         * them — wider is the safe direction. */
        {
            uint32_t me = s->xmap_mode_table[w & 0x1f];
            uint8_t pmode = (me >> 8) & 3, psize = (me >> 10) & 3;
            uint8_t amode = (me >> 16) & 7;
            if (pmode == 0) {
                uint16_t ci_msb = (uint16_t)((me & 0xf8) << 5);
                uint32_t cimax = (psize == 0) ? 0xf : (psize == 2) ? 0xfff : 0xff;
                uint32_t base = ci_msb >> 8, k;
                for (k = 0; k <= (cimax >> 8); k++) {
                    *pal_mask |= 1u << ((base | k) & 31);
                }
            } else {
                /* pix_mode 1/2/3 select the fixed CI banks 0x1d/0x1e/0x1f only
                 * when the pixel is CI; the RGB unpack path reads no palette, but
                 * mark the bank anyway (wider is safe). */
                *pal_mask |= 1u << ((0x1c + pmode) & 31);
            }
            if (amode != 0) {
                *pal_mask |= 1u << (((me >> 11) & 0x1f00) >> 8);
            }
        }
        /* entry[0] is the row's initial mode; entry[m>=1] is a segment boundary.
         * The walk advances only when x reaches (entry >> 5), so the run ends at
         * the first boundary whose x-start is off-screen — that word is read (it
         * is the compared next_did_entry) but never applied. */
        if (m >= 1 && (w >> 5) >= NEWPORT_SCREEN_W) {
            return m + 1;
        }
    }
    return -1;                  /* run does not terminate within the cache */
}

/* true if row y's DID resolution differs from the cached one; updates the cache */
static bool newport_did_row_changed(SGINewportVirtuixState *s, int y,
                                    uint16_t did_entry_ptr)
{
    uint16_t sig[NEWPORT_DID_SIGLEN];
    uint32_t pal = 0;
    int n = newport_did_row_sig(s, y, did_entry_ptr, sig, &pal);

    if (n < 0) {
        s->did_sig_n[y] = 0xff;         /* unmappable → always dirty */
        s->did_row_pal[y] = 0xffffffffu;
        return true;
    }
    if (s->did_sig_n[y] == (uint8_t)n &&
        memcmp(s->did_sig[y], sig, (size_t)n * sizeof(uint16_t)) == 0) {
        return false;                   /* same resolution → renders identically */
    }
    s->did_sig_n[y] = (uint8_t)n;
    s->did_row_pal[y] = pal;
    memcpy(s->did_sig[y], sig, (size_t)n * sizeof(uint16_t));
    return true;
}

/* Drop the per-row DID cache (forces the next resolve to dirty every row).  Used
 * wherever the DID base / mode table / whole device state changes underneath it. */
static void newport_did_cache_drop(SGINewportVirtuixState *s)
{
    memset(s->did_sig_n, 0xff, sizeof(s->did_sig_n));
    /* until a row is resolved again, assume it can read any palette bucket */
    memset(s->did_row_pal, 0xff, sizeof(s->did_row_pal));
}

/* Emit the marked rows as full-width dirty rects (few runs), or one spanning rect
 * when they are too fragmented for the list.  `rows` is the marked count.
 * PVDISPLAY_VC2_SHRINK=<n> is the POSITIVE CONTROL: it dirties n rows FEWER at
 * each end of every run, i.e. under-invalidates.  A probe that cannot fail proves
 * nothing (note 27 s3), so every oracle for this change is self-tested with it. */
static void newport_dirty_marked_rows(SGINewportVirtuixState *s,
                                      const uint8_t *marked, int rows)
{
    struct { int y0, y1; } runs[NEWPORT_DID_MAXRUNS];
    int nruns = 0, cur = -1, y, i, ymin = -1, ymax = -1;
    bool overflow = false;
    static int shrink = -1;

    if (rows <= 0) return;
    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        if (!marked[y]) continue;
        if (ymin < 0) ymin = y;
        ymax = y;
        if (cur >= 0 && y - runs[cur].y1 <= NEWPORT_DID_RUNGAP) {
            runs[cur].y1 = y;      /* extend across a small clean gap */
        } else if (nruns < NEWPORT_DID_MAXRUNS) {
            cur = nruns++;
            runs[cur].y0 = runs[cur].y1 = y;
        } else {
            overflow = true;
        }
    }
    if (shrink < 0) {
        const char *e = getenv("PVDISPLAY_VC2_SHRINK");
        shrink = e ? atoi(e) : 0;
    }
    if (shrink > 0) {
        for (i = 0; i < nruns; i++) {
            runs[i].y0 += shrink;
            runs[i].y1 -= shrink;
            if (runs[i].y0 > runs[i].y1) runs[i].y0 = runs[i].y1 = -1;
        }
        if (overflow) { ymin += shrink; ymax -= shrink; }
    }
    if (overflow) {
        if (ymin <= ymax) {
            newport_dirty_rect(s, 0, ymin, NEWPORT_SCREEN_W, ymax - ymin + 1);
        }
        return;
    }
    for (i = 0; i < nruns; i++) {
        if (runs[i].y0 < 0) continue;
        newport_dirty_rect(s, 0, runs[i].y0, NEWPORT_SCREEN_W,
                           runs[i].y1 - runs[i].y0 + 1);
    }
}

/* Resolve any pending VC2-SRAM change window into dirty rows (or full). */
static void newport_vc2_ram_resolve(SGINewportVirtuixState *s)
{
    uint32_t lo = s->vc2_ram_dirty_lo, hi = s->vc2_ram_dirty_hi;
    uint16_t did_entry_ptr, cursor_entry;
    bool use_did;
    static uint8_t marked[NEWPORT_SCREEN_H];
    int y, rows = 0;

    if (lo > hi) {
        return;                 /* nothing pending */
    }
    s->vc2_ram_dirty_lo = 0x8000;   /* consume the window */
    s->vc2_ram_dirty_hi = 0;
    s->vc2_resolve_calls++;

    did_entry_ptr = s->vc2_reg[VC2_DID_ENTRY];
    use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS) &&
              did_entry_ptr != 0;
    cursor_entry = s->vc2_reg[VC2_CURSOR_ENTRY];

    /* Cursor sprite RAM shares this SRAM and is not row-mappable from here. */
    if (hi >= (uint32_t)cursor_entry &&
        lo <= (uint32_t)cursor_entry + NEWPORT_VC2_CURSOR_WORDS - 1) {
        s->vc2_resolve_full++;
        newport_dirty_full(s);
        return;
    }
    if (!use_did) {
        /* the renderer isn't consulting the DID tables at all right now; the
         * safe reading is "we cannot say what this changed" -> full. */
        s->vc2_resolve_full++;
        newport_dirty_full(s);
        return;
    }

    memset(marked, 0, sizeof(marked));
    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        if (newport_did_row_changed(s, y, did_entry_ptr)) {
            marked[y] = 1;
            rows++;
        }
    }
    if (rows == 0) {
        return;                 /* the change is invisible (e.g. an inactive or
                                 * non-DID region of VC2 SRAM) */
    }
    s->vc2_resolve_bounded++;
    s->vc2_resolve_rows += rows;
    newport_dirty_marked_rows(s, marked, rows);
}

/*
 * Resolve pending CMAP-palette changes.  A palette entry recolours pixels with no
 * REX3 draw, so it must invalidate — but only the rows that can READ it.  Which
 * 256-entry buckets a row can read is decided by its DID resolution (base-plane
 * ci_msb per segment, overlay aux_msb), cached in did_row_pal[] alongside the
 * signature.  Cases that are not row-decidable fall back to whole-screen:
 *   - the popup bucket (popup pixels are selected per-pixel by cidaux, anywhere)
 *   - the hardware-cursor bucket
 *   - DIDs disabled (every row resolves to bucket 0)
 * This is the #2 saturator: with the DID tables bounded it was still forcing
 * 3058 whole-screen invalidations (~2.4 per rendered frame) on its own.
 */
static void newport_pal_resolve(SGINewportVirtuixState *s)
{
    uint32_t mask = s->pal_dirty_mask;
    uint16_t popup_msb, cursor_msb;
    static uint8_t marked[NEWPORT_SCREEN_H];
    int y, rows = 0;

    if (!mask) return;
    s->pal_dirty_mask = 0;

    popup_msb = (uint16_t)s->xmap_popup_cmap << 5;
    cursor_msb = (uint16_t)s->xmap_cursor_cmap << 5;
    if (!((s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS) &&
          s->vc2_reg[VC2_DID_ENTRY] != 0) ||
        (mask & (1u << ((popup_msb >> 8) & 31))) ||
        (mask & (1u << ((cursor_msb >> 8) & 31)))) {
        s->pal_resolve_full++;
        newport_dirty_full(s);
        return;
    }

    memset(marked, 0, sizeof(marked));
    for (y = 0; y < NEWPORT_SCREEN_H; y++) {
        if (s->did_row_pal[y] & mask) {
            marked[y] = 1;
            rows++;
        }
    }
    if (rows == 0) return;      /* no row can read the changed entries */
    s->pal_resolve_bounded++;
    s->pal_resolve_rows += rows;
    newport_dirty_marked_rows(s, marked, rows);
}

/*
 * Phase D: convert ONE scanline's pixels [x_begin,x_end) into dst[y*w + x],
 * running the full DID/XMAP/CMAP/RAMDAC pipeline.  The per-scanline DID mode
 * walk always starts at x=0 (the visual mode changes mid-row at DID segment
 * boundaries), but only x in [x_begin,x_end) are written — that is what lets
 * the incremental path repaint a sub-span byte-for-byte identically to the full
 * walk (the full walk is just this with [0,W)).  The pixel SOURCE is passed as
 * src_rgbci/src_cidaux row pointers and is the ONLY coupling to where the pixels
 * live: VRAM today, a shadowfb-derived CI row for Stage 2 (do not hardcode). */
static void newport_convert_row(SGINewportVirtuixState *s, uint32_t *dst, int w,
                                int y, int x_begin, int x_end, int xoff,
                                const uint32_t *src_rgbci,
                                const uint32_t *src_cidaux,
                                uint16_t did_entry_ptr, bool use_did,
                                uint16_t popup_msb)
{
    int x;
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
        /* BL-83: the visible window begins at scanline pixel xoff (the
         * bt445_xbias extra columns Xsgi leaves black).  Skip DID transitions
         * before it so the mode at screen x=0 is the mode at scanline xoff. */
        while ((uint16_t)(next_did_entry >> 5) <= (uint16_t)xoff) {
            mode_entry = s->xmap_mode_table[next_did_entry & 0x1f];
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
    }

    for (x = 0; x < x_end; x++) {
        uint32_t pixel, cidaux, rgb;
        uint8_t r, g, b;

        /* DID-segment boundary: advance the visual mode (must run for EVERY x
         * from 0 so the mode at x_begin is correct — this is the whole reason
         * the walk starts at 0 even when only a sub-span is written).
         * Scanline pixel = x + xoff (the visible window is offset by the
         * bt445_xbias). */
        if (use_did && (uint16_t)(x + xoff) == (next_did_entry >> 5)) {
            mode_entry = s->xmap_mode_table[next_did_entry & 0x1f];
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

        if (x < x_begin) {
            continue;   /* walk the mode state only; don't touch this pixel */
        }

        pixel = src_rgbci[x];
        cidaux = src_cidaux[x];


        if (cidaux & 0xcc) {
            uint8_t popup_ci = (cidaux >> 2) & 3;
            rgb = s->cmap0_palette[(popup_msb | popup_ci) & 0x1fff];
        } else if (aux_pix_mode != 0) {
            bool overlay_hit = false;
            switch (aux_pix_mode) {
            case 1:
                rgb = s->cmap0_palette[(aux_msb | ((cidaux >> 8) & 3)) & 0x1fff];
                overlay_hit = true;
                break;
            case 2: {
                uint32_t ovl = (cidaux >> 8) & 3;
                if (ovl) {
                    rgb = s->cmap0_palette[(aux_msb | ovl) & 0x1fff];
                    overlay_hit = true;
                }
                break;
            }
            case 6: {
                uint32_t shift = (mode_entry & 2) ? 9 : 8;
                uint32_t ovl = (cidaux >> shift) & 1;
                if (ovl) {
                    rgb = s->cmap0_palette[(aux_msb | ovl) & 0x1fff];
                    overlay_hit = true;
                }
                break;
            }
            case 7: {
                uint32_t ovl = (cidaux >> 8) & 1;
                rgb = s->cmap0_palette[
                    (aux_msb | (ovl ? ovl : ((cidaux >> 9) & 1))) & 0x1fff];
                overlay_hit = true;
                break;
            }
            default: break;
            }
            if (!overlay_hit) goto main_pixel;
        } else {
        main_pixel:
        if (pix_mode == 0) {
            uint16_t ci;
            switch (pix_size) {
            case 0: ci = pixel & 0xf; break;
            case 1: ci = pixel & 0xff; break;
            case 2: ci = pixel & 0xfff; break;
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

        dst[(size_t)y * w + x] = rgb_to_pixel32(r, g, b);
    }
}

/* ============================================================
 * Stage 2 Phase 2a — paravirtual shadowfb scanout (Variant B)
 * ============================================================
 * SCANOUT_SET registers a linear shadow framebuffer in guest RAM; when active,
 * the base CI plane's pixels for the DID walk come from it instead of VRAM.  The
 * DID/XMAP/CMAP/RAMDAC pipeline and the popup/overlay cidaux compositing are
 * UNCHANGED — only the src_rgbci row handed to newport_convert_row differs.  This
 * keeps per-window DID palettes, private colormaps and colormap animation correct
 * by construction (no global CI8->xRGB LUT, per the director's design review). */

/* Sanity/hostile-value bounds for guest-supplied shadowfb geometry.  Anything
 * out of range disables the shadowfb (falls back to VRAM) rather than trusting
 * the guest — a lying base/stride can't over-read (dma_memory_read clamps) but a
 * huge w/h/stride could blow up the scratch allocations, so cap them here. */
#define NP_SCANOUT_MAX_W      4096
#define NP_SCANOUT_MAX_H      4096
#define NP_SCANOUT_MAX_STRIDE (NP_SCANOUT_MAX_W * 4)

/*
 * Translate the guest-supplied shadowfb base to a guest physical address.
 *
 * The DDX registers a *raw physical* base (PVGPU_MAP_FB -> kvtophys of the
 * driver's contiguous shadow fb; bit 31 clear).  A >256MB guest can place that
 * fb in RAM SEG1 (physical bit 29 set); the historical `& 0x1FFFFFFF` mask here
 * dropped bit 29 and pointed scanout at a wrong low page -- the BL-54 class
 * (progress_notes/ip55/seg1_mask_sweep.md).  SGI_VIRTUIX_GPA preserves SEG1 for
 * physical bases while still stripping KSEG for any virtual base.
 *
 * NP_SEG1_TRACE=1 logs (rate-limited) whenever the base actually lands in SEG1
 * -- the ground-truth signal that this site was a *live* truncation before the
 * fix rather than merely latent.
 */
static inline uint64_t newport_scanout_base_gpa(SGINewportVirtuixState *s)
{
    uint64_t base = s->scanout_base;
    static int trace = -1;
    if (trace < 0) {
        trace = getenv("NP_SEG1_TRACE") ? 1 : 0;
    }
    if (trace && (base & 0x20000000ULL) && !(base & 0x80000000ULL)) {
        static uint32_t n;
        if ((n++ & 0x3ff) == 0) {
            fprintf(stderr, "NP_SEG1_TRACE: scanout base=0x%" PRIx64
                    " in SEG1 (bit29 set) -- old 29-bit mask would truncate to "
                    "0x%" PRIx64 " (hit #%u)\n",
                    base, (uint64_t)(base & 0x1FFFFFFFULL), n);
        }
    }
    return SGI_VIRTUIX_GPA(base);
}

/* SCANOUT_SET handler (invoked from glaccel via the desk_scanout callback).
 * Validates the guest values, (re)allocates the per-row DMA scratch, latches the
 * geometry, and forces a full repaint so the first shadowfb frame is complete. */
static void newport_set_scanout(void *opaque, uint64_t base, uint32_t w,
                                uint32_t h, uint32_t stride, uint32_t fmt,
                                bool active)
{
    SGINewportVirtuixState *s = opaque;
    uint32_t bpp;

    /* PVDisplay Phase 2b/3: log every scanout enable/disable transition so the
     * cold-boot timeline can be verified (the flip must go live only AFTER the
     * desktop session is up, never during clogin — session 12 desktop-gating). */
    if (active != s->scanout_active) {
        fprintf(stderr, "PVDISPLAY_SCANOUT: %s base=0x%" PRIx64 " w=%u h=%u fmt=%u\n",
                active ? "ENABLE" : "disable", base, w, h, fmt);
    }

    if (!active) {
        s->scanout_active = false;
        newport_dirty_full(s);
        return;
    }

    /* format: 0=CI8 (1 byte/pixel), 1=xRGB32 (4 bytes/pixel, experimental) */
    bpp = (fmt == 1) ? 4 : 1;
    if (stride == 0) {
        stride = w * bpp;   /* tightly packed default */
    }

    /* hostile-value validation → disable on any failure */
    if (base == 0 || w == 0 || h == 0 ||
        w > NP_SCANOUT_MAX_W || h > NP_SCANOUT_MAX_H ||
        stride < w * bpp || stride > NP_SCANOUT_MAX_STRIDE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "newport: SCANOUT_SET rejected (base=0x%" PRIx64
                      " w=%u h=%u stride=%u fmt=%u)\n",
                      base, w, h, stride, fmt);
        s->scanout_active = false;
        newport_dirty_full(s);
        return;
    }

    /* Note: base is a guest phys addr translated via newport_scanout_base_gpa()
     * (SGI_VIRTUIX_GPA) at read time -- SEG1-preserving, so a >256MB shadowfb is
     * addressed correctly; an out-of-RAM base can't over-read (dma_memory_read
     * clamps to the address space, returning zeros), so the geometry caps above
     * are the real defense against hostile values (they bound the scratch
     * allocations). */

    s->scanout_base   = base;
    s->scanout_w      = w;
    s->scanout_h      = h;
    s->scanout_stride = stride;
    s->scanout_format = fmt;
    s->scanout_rowbytes = g_realloc(s->scanout_rowbytes, stride);
    if (!s->scanout_row) {
        s->scanout_row = g_new0(uint32_t, NEWPORT_SCREEN_W);
    }

    /* PVDisplay session 22 (BL-43 fix): SEED the shadow framebuffer from the
     * currently-visible VRAM base plane at flip activation.  The DDX's flip
     * memsets its shadow to 0 (black) and scans it out blank, so any content
     * painted into VRAM BEFORE the flip goes live — the app-cached Icon-Catalog
     * body (labels, menu bar, status line, "pedestal" region) and every other
     * pre-flip pixel — was lost to index-0 black (the "added black bars" / missing
     * body text of BL-43).  We own both surfaces here, so copying VRAM->shadow
     * is generic and race-free: it runs synchronously inside the SCANOUT_SET
     * handler BEFORE scanout_active goes true, hence completes before the first
     * shadow frame is scanned out (and after the guest's memset, which precedes
     * the ring doorbell that drove us here).  CI8 only: the shadow byte is the
     * VRAM rgbci low byte, which convert_row renders byte-identically whether the
     * base plane is sourced from the shadow or VRAM (see newport_scanout_ci_row /
     * newport_scanout_ab_check).  The aux planes (popup/overlay/CID) always source
     * from VRAM regardless, so only the base plane needs seeding.  Post-flip live
     * draws paint over the seed as before.  NP_SCANOUT_NOSEED=1 restores the old
     * blank-shadow behaviour for A/B. */
    /* NP_VRAM_DUMP diagnostic: write the raw vram_rgbci low-byte plane (index
     * plane, pre-CMAP) to a PGM at seed time, so a banded run can be checked for
     * whether the bands live in the VRAM CONTENT (guest/emulation) or only in the
     * convert/display path.  Off unless NP_VRAM_DUMP names a path. */
    {
        const char *vd = getenv("NP_VRAM_DUMP");
        if (vd && s->vram_rgbci) {
            FILE *f = fopen(vd, "wb");
            if (f) {
                int yy2, xx2;
                fprintf(f, "P5\n%d %d\n255\n", NEWPORT_SCREEN_W,
                        NEWPORT_SCREEN_H);
                for (yy2 = 0; yy2 < NEWPORT_SCREEN_H; yy2++) {
                    for (xx2 = 0; xx2 < NEWPORT_SCREEN_W; xx2++) {
                        uint8_t b = (uint8_t)
                            (s->vram_rgbci[(size_t)yy2 * NEWPORT_VRAM_W + xx2]
                             & 0xff);
                        fputc(b, f);
                    }
                }
                fclose(f);
                fprintf(stderr, "NP_VRAM_DUMP: wrote %s\n", vd);
            }
        }
    }
    if (fmt == 0 && s->vram_rgbci && !newport_scanout_noseed()) {
        uint32_t cols = MIN(w, (uint32_t)NEWPORT_SCREEN_W);
        uint32_t rows = MIN(h, (uint32_t)NEWPORT_SCREEN_H);
        int xoff = newport_scanout_xoff(s);
        uint32_t yy, xx;
        for (yy = 0; yy < rows; yy++) {
            const uint32_t *vrow = &s->vram_rgbci[(size_t)yy * NEWPORT_VRAM_W + xoff];
            for (xx = 0; xx < cols; xx++) {
                s->scanout_rowbytes[xx] = (uint8_t)(vrow[xx] & 0xff);
            }
            for (; xx < stride; xx++) {
                s->scanout_rowbytes[xx] = 0;   /* pad cols beyond VRAM width */
            }
            dma_memory_write(&address_space_memory,
                             (hwaddr)(newport_scanout_base_gpa(s) +
                                      (uint64_t)yy * stride),
                             s->scanout_rowbytes, stride,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }

    s->scanout_active = true;
    newport_dirty_full(s);   /* first shadowfb frame repaints everything */
}

/* DAMAGE handler (invoked from glaccel via the desk_damage callback): a region
 * of the shadowfb changed → feed the existing dirty-rect machinery so only those
 * rows are re-walked next frame.  No DAMAGE between frames = nothing redrawn. */
static void newport_scanout_damage(void *opaque, int x, int y, int w, int h)
{
    SGINewportVirtuixState *s = opaque;
    if (!s->scanout_active) {
        return;
    }
    newport_dirty_rect(s, x, y, w, h);
}

/* BL-81 bounded geometry repaint (invoked from glaccel via the desk_repaint
 * callback): a server-published window model moved/resized a GL window, so the
 * desktop under the union of its old and new rects must be re-walked from VRAM
 * next render — but nothing else.  Feed the rects straight into the same Phase D
 * dirty-rect machinery every REX3 primitive uses, so newport_render_desktop()
 * re-walks exactly them and reports them back for a bounded blit.
 *
 * Unlike newport_scanout_damage() this is NOT gated on scanout_active: the
 * requirement is about the desktop under a GL overlay, which exists whether or
 * not a paravirtual shadowfb is registered.  Coalescing and saturation
 * (dirty_n == NEWPORT_DIRTY_MAX => full re-walk) are handled by
 * newport_dirty_rect(), i.e. the safe direction: never repaint less than asked. */
static void newport_desk_repaint(void *opaque, const PVDeskRect *rects, int n)
{
    SGINewportVirtuixState *s = opaque;
    int i;
    for (i = 0; i < n; i++) {
        newport_dirty_rect(s, rects[i].x, rects[i].y, rects[i].w, rects[i].h);
    }
}

/* Build one CI8 shadowfb scanline as rgbci words for the DID walk: DMA row y of
 * the shadow fb from guest RAM and zero-extend each CI byte into the low 8 bits
 * of an rgbci word (newport_convert_row's pix_size=1 path reads pixel & 0xff).
 * Rows/columns outside the shadowfb are sourced as 0.  Returns a pointer to the
 * SCREEN_W-wide scratch row (valid until the next call). */
static const uint32_t *newport_scanout_ci_row(SGINewportVirtuixState *s, int y)
{
    uint32_t *row = s->scanout_row;
    int x, cols;

    if ((uint32_t)y >= s->scanout_h) {
        memset(row, 0, NEWPORT_SCREEN_W * sizeof(uint32_t));
        return row;
    }
    dma_memory_read(&address_space_memory,
                    (hwaddr)(newport_scanout_base_gpa(s) +
                             (uint64_t)y * s->scanout_stride),
                    s->scanout_rowbytes, s->scanout_stride,
                    MEMTXATTRS_UNSPECIFIED);
    cols = (int)MIN((uint32_t)NEWPORT_SCREEN_W, s->scanout_w);
    for (x = 0; x < cols; x++) {
        row[x] = s->scanout_rowbytes[x];
    }
    for (; x < NEWPORT_SCREEN_W; x++) {
        row[x] = 0;
    }
    return row;
}

/* Gate 2a criterion (a) oracle, env-gated NP_SCANOUT_AB=1 (debug only): for a
 * shadowfb row just rendered into dst, copy the SAME CI values into the real
 * vram_rgbci row and render through the standard VRAM source; byte-compare.
 * Same CI values + same DID/XMAP/CMAP walk must render byte-identically
 * regardless of pixel source — the "same pattern drawn via REX3 into VRAM"
 * equivalence with the REX3 store replaced by a direct CI copy (the store path
 * itself is covered by Gates C/D).  VRAM row saved/restored. */
static bool newport_scanout_ab(void)
{
    static int v = -1;
    if (v < 0) {
        v = getenv("NP_SCANOUT_AB") ? 1 : 0;
    }
    return v;
}

static void newport_scanout_ab_check(SGINewportVirtuixState *s,
                                     const uint32_t *dst, int w, int y,
                                     int x_begin, int x_end,
                                     const uint32_t *shadow_row,
                                     uint16_t did_entry_ptr, bool use_did,
                                     uint16_t popup_msb)
{
    static uint32_t *scratch;       /* full frame: convert_row indexes dst[y*w+x] */
    static uint32_t n_rows, n_bad;
    uint32_t saved[NEWPORT_SCREEN_W];
    int xoff = newport_scanout_xoff(s);
    uint32_t *vrow = &s->vram_rgbci[y * NEWPORT_VRAM_W + xoff];
    int x;
    bool ok = true;

    if (!scratch) {
        scratch = g_new0(uint32_t, (size_t)NEWPORT_SCREEN_W * NEWPORT_SCREEN_H);
    }
    memcpy(saved, vrow, sizeof(saved));
    memcpy(vrow, shadow_row, sizeof(saved));   /* same CI values, now in VRAM */
    newport_convert_row(s, scratch, w, y, x_begin, x_end, xoff,
                        vrow, &s->vram_cidaux[y * NEWPORT_VRAM_W + xoff],
                        did_entry_ptr, use_did, popup_msb);
    memcpy(vrow, saved, sizeof(saved));

    for (x = x_begin; x < x_end; x++) {
        if (scratch[(size_t)y * w + x] != dst[(size_t)y * w + x]) {
            ok = false;
            break;
        }
    }
    n_rows++;
    if (!ok) {
        n_bad++;
        fprintf(stderr, "NP_SCANOUT_AB MISMATCH y=%d x=%d ci=%02x "
                "shadow_out=%08x vram_out=%08x\n", y, x,
                shadow_row[x] & 0xff, dst[(size_t)y * w + x],
                scratch[(size_t)y * w + x]);
    }
    if ((n_rows % 1024) == 0 || (!ok && n_bad < 8)) {
        fprintf(stderr, "NP_SCANOUT_AB rows=%u mismatches=%u\n", n_rows, n_bad);
    }
}

/* xRGB32 shadowfb (experimental >8bpp path): DMA row y and copy it straight to
 * dst, bypassing the CI/DID/CMAP LUT entirely.  cidaux is NOT composited here —
 * this path is a placeholder for the future TrueColor screen (Phase 2b+). */
static void newport_scanout_xrgb_row(SGINewportVirtuixState *s, uint32_t *dst,
                                     int w, int y, int x_begin, int x_end)
{
    int x, cols;
    if ((uint32_t)y >= s->scanout_h) {
        return;
    }
    dma_memory_read(&address_space_memory,
                    (hwaddr)(newport_scanout_base_gpa(s) +
                             (uint64_t)y * s->scanout_stride),
                    s->scanout_rowbytes, s->scanout_stride,
                    MEMTXATTRS_UNSPECIFIED);
    cols = (int)MIN((uint32_t)x_end, s->scanout_w);
    for (x = x_begin; x < cols; x++) {
        uint32_t px = ldl_be_p(s->scanout_rowbytes + (size_t)x * 4);
        dst[(size_t)y * w + x] = px;
    }
}

/* Render the desktop REX3/VC2/CMAP/RAMDAC pipeline into *dst (w*h xRGB32).
 * Called by the unified display engine on every frame tick.  force_full means
 * repaint every pixel (the caller discarded the previous buffer).
 *
 * Phase D: when not forced, repaint only the accumulated dirty rects and report
 * them in out_rects[] so the engine can bound its blit + dpy_gfx_update.  Return
 * value follows PVDeskRenderFn: PVDESK_FULL (whole screen repainted), 0 (idle),
 * or n>0 (n dirty rects written). */
static int newport_render_desktop(void *opaque, uint32_t *dst, int w, int h,
                                  bool force_full, PVDeskRect *out_rects,
                                  int max_rects)
{
    SGINewportVirtuixState *s = opaque;
    uint16_t did_entry_ptr;
    bool use_did, do_full, soft_cursor;
    uint16_t popup_msb = (uint16_t)s->xmap_popup_cmap << 5;
    int nrects = 0, i, y;

    if (!force_full && !s->display_dirty) {
        return 0;  /* nothing changed; engine can skip the blit */
    }

    /* Phase D: fold any pending VC2 DID-table change into the dirty-rect list
     * before do_full is decided (a burst of SRAM writes resolves to a handful of
     * row ranges here instead of hundreds of full invalidations at write time). */
    newport_vc2_ram_resolve(s);
    newport_pal_resolve(s);

    {
        static int sc = -1;
        if (sc < 0) sc = getenv("PVDISPLAY_SOFT_CURSOR") ? 1 : 0;
        soft_cursor = sc;
    }

    did_entry_ptr = s->vc2_reg[VC2_DID_ENTRY];
    use_did = (s->vc2_reg[VC2_DC_CONTROL] & VC2_DC_ENA_DIDS)
              && did_entry_ptr != 0;

    /*
     * Full walk when: the caller forced it, the NP_SCANOUT_FULL oracle demands
     * it (A/B reference), the dirty list saturated to full, or the software
     * cursor is active (its old-position restore is only correct under a full
     * repaint — soft cursor is the debug escape hatch, so we don't chase the
     * incremental win there).
     *
     * Note 29: the saturation test is now the explicit s->dirty_full flag.  A
     * FULL LIST is no longer a saturated one — newport_dirty_rect() merges into
     * the cheapest existing slot instead of giving up (§the coalescing comment
     * there), so dirty_n == NEWPORT_DIRTY_MAX is an ordinary bounded state.
     *
     * Also note 29, and a latent correctness fix: the bounded branch renders
     * every dirty rect but can only REPORT max_rects of them, and the engine
     * blits only what is reported — so more rects than the caller can carry must
     * go full rather than leave rendered-but-unblitted rows on screen.  (It has
     * been unreachable in practice since PVDESK_MAX_RECTS was pinned equal to
     * NEWPORT_DIRTY_MAX, but the two are separate constants in separate files.)
     *
     * Each reason gets its own counter (why_full in the NEWPORT_DIRTYFULL_STATS
     * line).  The increments were lost when note 28 §5b's probe knob was removed
     * from this block, which left the printed why_full triple reading 0/0/0.
     */
    {
        bool oracle = newport_scanout_full() || soft_cursor;
        bool overflow = (out_rects && s->dirty_n > max_rects);
        do_full = force_full || oracle || s->dirty_full || overflow;
        if (do_full) {
            if (force_full)         s->rd_force++;
            else if (oracle)        s->rd_oracle++;
            else if (s->dirty_full) s->rd_sat++;
            else                    s->rd_overflow++;
        }
    }
    /* Stage 2 Phase 2a: pixel-source selection.  When a CI8 shadowfb is active
     * the base plane's rgbci row comes from guest RAM (via the DID walk); the
     * xRGB32 shadowfb takes an experimental straight-copy path that bypasses the
     * LUT.  With no shadowfb, everything is exactly as before (VRAM source). */
    bool shadow_ci8   = s->scanout_active && s->scanout_format == 0;
    bool shadow_xrgb  = s->scanout_active && s->scanout_format == 1;

    int xoff = newport_scanout_xoff(s);
    if (do_full) {
        for (y = 0; y < NEWPORT_SCREEN_H; y++) {
            if (shadow_xrgb) {
                newport_scanout_xrgb_row(s, dst, w, y, 0, NEWPORT_SCREEN_W);
            } else {
                const uint32_t *src = shadow_ci8
                    ? newport_scanout_ci_row(s, y)
                    : &s->vram_rgbci[y * NEWPORT_VRAM_W + xoff];
                newport_convert_row(s, dst, w, y, 0, NEWPORT_SCREEN_W, xoff,
                                    src,
                                    &s->vram_cidaux[y * NEWPORT_VRAM_W + xoff],
                                    did_entry_ptr, use_did, popup_msb);
                if (shadow_ci8 && newport_scanout_ab()) {
                    newport_scanout_ab_check(s, dst, w, y, 0, NEWPORT_SCREEN_W,
                                             src, did_entry_ptr, use_did,
                                             popup_msb);
                }
            }
        }
    } else {
        for (i = 0; i < s->dirty_n; i++) {
            int rx = s->dirty_rects[i].x, ry = s->dirty_rects[i].y;
            int rw = s->dirty_rects[i].w, rh = s->dirty_rects[i].h;
            int xe = MIN(rx + rw, NEWPORT_SCREEN_W);
            int ye = MIN(ry + rh, NEWPORT_SCREEN_H);
            for (y = ry; y < ye; y++) {
                if (shadow_xrgb) {
                    newport_scanout_xrgb_row(s, dst, w, y, rx, xe);
                } else {
                    const uint32_t *src = shadow_ci8
                        ? newport_scanout_ci_row(s, y)
                        : &s->vram_rgbci[y * NEWPORT_VRAM_W + xoff];
                    newport_convert_row(s, dst, w, y, rx, xe, xoff,
                                        src,
                                        &s->vram_cidaux[y * NEWPORT_VRAM_W + xoff],
                                        did_entry_ptr, use_did, popup_msb);
                    if (shadow_ci8 && newport_scanout_ab()) {
                        newport_scanout_ab_check(s, dst, w, y, rx, xe,
                                                 src, did_entry_ptr, use_did,
                                                 popup_msb);
                    }
                }
            }
            if (out_rects && nrects < max_rects) {
                out_rects[nrects].x = rx;      out_rects[nrects].y = ry;
                out_rects[nrects].w = xe - rx; out_rects[nrects].h = ye - ry;
                nrects++;
            }
        }
    }

    /* Cursor overlay — hardware path (Phase E) or software fallback */
    if (soft_cursor) {
        newport_draw_cursor(s, dst);
    } else {
        newport_update_hw_cursor(s);
    }

    /* NewView frame boundary marker */
    newport_newview_log(s, 0x80000000, 0);

    /* BL-44 live VRAM dump: when NP_VRAM_DUMP_LIVE names a path, write the raw
     * vram_rgbci index plane every render (last write = settled state), so a
     * banded settled desktop can be checked: banded VRAM => draw/guest bug;
     * clean VRAM => the bands live only in the incremental composite/display. */
    {
        static const char *vdl = (const char *)-1;
        if (vdl == (const char *)-1) vdl = getenv("NP_VRAM_DUMP_LIVE");
        if (vdl && s->vram_rgbci) {
            FILE *f = fopen(vdl, "wb");
            if (f) {
                int yy, xx;
                fprintf(f, "P5\n%d %d\n255\n", NEWPORT_SCREEN_W, NEWPORT_SCREEN_H);
                for (yy = 0; yy < NEWPORT_SCREEN_H; yy++)
                    for (xx = 0; xx < NEWPORT_SCREEN_W; xx++)
                        fputc((uint8_t)(s->vram_rgbci[(size_t)yy * NEWPORT_VRAM_W
                                                      + xx] & 0xff), f);
                fclose(f);
            }
        }
    }

    /* NEWPORT_DIRTYFULL_STATS=<n>: dump the per-call-site whole-screen-invalidation
     * histogram plus the VC2 resolver's own counters every n renders (n>=2, or
     * 512 for "1"), so "who is saturating the dirty list" is answerable from a log. */
    if (newport_dfull_stats_on()) {
        static uint64_t renders;
        static int period = -1;
        if (period < 0) {
            const char *e = getenv("NEWPORT_DIRTYFULL_STATS");
            period = e ? atoi(e) : 0;
            if (period < 2) period = 512;
        }
        if (++renders % (uint64_t)period == 0) {
            int k;
            fprintf(stderr, "newport: DIRTYFULL renders=%llu dfull_total=%llu "
                    "vc2_writes=%u vc2_resolve=%u (full=%u bounded=%u rows=%u) "
                    "pal(full=%u bounded=%u rows=%u) "
                    "why_full(force=%u sat=%u oracle=%u ovf=%u area=%u) sites:",
                    (unsigned long long)renders,
                    (unsigned long long)newport_dfull_total,
                    s->vc2_ram_writes, s->vc2_resolve_calls,
                    s->vc2_resolve_full, s->vc2_resolve_bounded,
                    s->vc2_resolve_rows,
                    s->pal_resolve_full, s->pal_resolve_bounded,
                    s->pal_resolve_rows,
                    s->rd_force, s->rd_sat, s->rd_oracle, s->rd_overflow,
                    s->rd_area);
            for (k = 0; k < NEWPORT_DFULL_SITES && newport_dfull_site[k].line; k++) {
                fprintf(stderr, " L%d=%llu", newport_dfull_site[k].line,
                        (unsigned long long)newport_dfull_site[k].n);
            }
            fprintf(stderr, "\n");
        }
    }

    s->display_dirty = false;
    s->dirty_n = 0;  /* Phase D: clear dirty rects */
    s->dirty_full = false;
    return do_full ? PVDESK_FULL : nrects;
}

/*
 * ============================================================
 * Device lifecycle
 * ============================================================
 */

static void sgi_newport_virtuix_reset(DeviceState *dev)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(dev);

    /* Stage 2 Phase 2a: drop any registered shadowfb (guest re-issues SCANOUT_SET) */
    s->scanout_active = false;

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
    s->vc2_ram_dirty_lo = 0x8000;   /* lo > hi == "no pending DID change" */
    s->vc2_ram_dirty_hi = 0;
    newport_did_cache_drop(s);      /* per-row DID resolution cache */

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

    newport_dirty_full(s);
}

static void sgi_newport_virtuix_realize(DeviceState *dev, Error **errp)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(dev);

    /* NP_2D_ROWHIST diagnostic: latch the gate once so the per-pixel call sites
     * are a single predictable branch (never taken when off). */
    if (getenv("NP_2D_ROWHIST")) {
        np_2d_rowhist = 1;
        atexit(newport_rowhist_dump);
    }

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

    /* Register as the desktop renderer with the unified display engine.
     * The engine owns the QemuConsole; we only render pixels into its buffer. */
    sgi_glaccel_register_desktop(newport_render_desktop, newport_invalidate,
                                  newport_set_scanout, newport_scanout_damage,
                                  newport_desk_repaint,
                                  s, NEWPORT_SCREEN_W, NEWPORT_SCREEN_H);

    /* Open NewView binary log file if property is set */
    if (s->newview_log_path && s->newview_log_path[0] != '\0') {
        s->newview_log_file = fopen(s->newview_log_path, "wb");
        if (!s->newview_log_file) {
            qemu_log_mask(LOG_UNIMP,
                          "newport: failed to open NewView log '%s'\n",
                          s->newview_log_path);
        }
    }

    newport_dirty_full(s);
}

static void sgi_newport_virtuix_init(Object *obj)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &sgi_newport_virtuix_ops, s,
                          "sgi-newport-virtuix", REX3_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* IRQ output for vertical retrace → INT3_LOCAL1 bit 7 */
    sysbus_init_irq(sbd, &s->irq);
}

static void sgi_newport_virtuix_finalize(Object *obj)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);

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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
        "Clip_mode=0x%08x status=0x%08x config=0x%08x\n"
        "XYWIN=0x%08x (winx=0x%04x winy=0x%04x scanout_xoff=%d)\n",
        s->color_red, s->color_green, s->color_blue, s->color_alpha,
        s->color_i, s->color_back,
        s->write_mask, s->global_mask,
        s->clip_mode, s->status, s->config,
        s->xy_window, (uint16_t)(s->xy_window >> 16),
        (uint16_t)s->xy_window, newport_scanout_xoff(s));

    return g_string_free(buf, FALSE);
}

static char *newport_get_diag_dcb(Object *obj, Error **errp)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);
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
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(obj);

    if (value && value[0] != '\0') {
        /* Force display dirty so the surface gets updated too */
        newport_dirty_full(s);
        newport_dump_vram_ppm(s, value);
    }
}

static const Property sgi_newport_virtuix_properties[] = {
    DEFINE_PROP_STRING("newview-log", SGINewportVirtuixState, newview_log_path),
};

static int sgi_newport_virtuix_post_load(void *opaque, int version_id)
{
    SGINewportVirtuixState *s = SGI_NEWPORT_VIRTUIX(opaque);

    /* Rebuild decoded drawmode fields from saved register values */
    newport_decode_drawmode1(s);
    newport_decode_drawmode0(s);

    /* Force display refresh.  The pending VC2-SRAM change window is transient and
     * not migrated, so clear it explicitly (a zeroed pair would read as "word 0
     * pending"); the full invalidate below covers anything it would have named. */
    s->vc2_ram_dirty_lo = 0x8000;
    s->vc2_ram_dirty_hi = 0;
    newport_did_cache_drop(s);
    newport_dirty_full(s);

    return 0;
}

#define NEWPORT_VRAM_BYTES \
    (NEWPORT_VRAM_W * NEWPORT_VRAM_H * sizeof(uint32_t))

static const VMStateDescription vmstate_sgi_newport = {
    .name = "sgi-newport-virtuix",
    .version_id = 5,
    .minimum_version_id = 5,
    .post_load = sgi_newport_virtuix_post_load,
    .fields = (const VMStateField[]) {
        /* REX3 drawing registers */
        VMSTATE_UINT32(drawmode0, SGINewportVirtuixState),
        VMSTATE_UINT32(drawmode1, SGINewportVirtuixState),
        VMSTATE_UINT32(ls_mode, SGINewportVirtuixState),
        VMSTATE_UINT32(ls_pattern, SGINewportVirtuixState),
        VMSTATE_UINT32(ls_pattern_saved, SGINewportVirtuixState),
        VMSTATE_UINT32(z_pattern, SGINewportVirtuixState),
        VMSTATE_UINT32(color_back, SGINewportVirtuixState),
        VMSTATE_UINT32(color_vram, SGINewportVirtuixState),
        VMSTATE_UINT32(alpha_ref, SGINewportVirtuixState),
        VMSTATE_UINT32_ARRAY(smask_x, SGINewportVirtuixState, 5),
        VMSTATE_UINT32_ARRAY(smask_y, SGINewportVirtuixState, 5),
        VMSTATE_UINT32(setup, SGINewportVirtuixState),
        VMSTATE_UINT32(step_z, SGINewportVirtuixState),

        /* Coordinate registers */
        VMSTATE_INT32(x_start, SGINewportVirtuixState),
        VMSTATE_INT32(y_start, SGINewportVirtuixState),
        VMSTATE_INT32(x_end, SGINewportVirtuixState),
        VMSTATE_INT32(y_end, SGINewportVirtuixState),
        VMSTATE_INT32(x_save, SGINewportVirtuixState),
        VMSTATE_UINT32(xy_move, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_d, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_s1, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_octant_inc1, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_round_inc2, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_e1, SGINewportVirtuixState),
        VMSTATE_UINT32(bres_s2, SGINewportVirtuixState),
        VMSTATE_UINT32(a_weight0, SGINewportVirtuixState),
        VMSTATE_UINT32(a_weight1, SGINewportVirtuixState),
        VMSTATE_UINT32(x_start_f, SGINewportVirtuixState),
        VMSTATE_UINT32(y_start_f, SGINewportVirtuixState),
        VMSTATE_UINT32(x_end_f, SGINewportVirtuixState),
        VMSTATE_UINT32(y_end_f, SGINewportVirtuixState),
        VMSTATE_INT32(x_start_i, SGINewportVirtuixState),
        VMSTATE_UINT32(xy_start_i, SGINewportVirtuixState),
        VMSTATE_UINT32(xy_end_i, SGINewportVirtuixState),
        VMSTATE_UINT32(x_start_end_i, SGINewportVirtuixState),

        /* Integer coordinate state */
        VMSTATE_INT16(iter_x, SGINewportVirtuixState),
        VMSTATE_INT16(iter_y, SGINewportVirtuixState),
        VMSTATE_INT16(x_start_int, SGINewportVirtuixState),
        VMSTATE_INT16(y_start_int, SGINewportVirtuixState),
        VMSTATE_INT16(x_end_int, SGINewportVirtuixState),
        VMSTATE_INT16(y_end_int, SGINewportVirtuixState),
        VMSTATE_INT16(x_save_int, SGINewportVirtuixState),

        /* Color registers */
        VMSTATE_UINT32(write_mask, SGINewportVirtuixState),
        VMSTATE_UINT32(color_i, SGINewportVirtuixState),
        VMSTATE_UINT32(zero_overflow, SGINewportVirtuixState),
        VMSTATE_UINT32(color_red, SGINewportVirtuixState),
        VMSTATE_UINT32(color_alpha, SGINewportVirtuixState),
        VMSTATE_UINT32(color_green, SGINewportVirtuixState),
        VMSTATE_UINT32(color_blue, SGINewportVirtuixState),
        VMSTATE_UINT32(curr_color_red, SGINewportVirtuixState),
        VMSTATE_UINT32(curr_color_alpha, SGINewportVirtuixState),
        VMSTATE_UINT32(curr_color_green, SGINewportVirtuixState),
        VMSTATE_UINT32(curr_color_blue, SGINewportVirtuixState),
        VMSTATE_INT32(slope_red, SGINewportVirtuixState),
        VMSTATE_INT32(slope_alpha, SGINewportVirtuixState),
        VMSTATE_INT32(slope_green, SGINewportVirtuixState),
        VMSTATE_INT32(slope_blue, SGINewportVirtuixState),
        VMSTATE_UINT64(host_dataport, SGINewportVirtuixState),
        VMSTATE_UINT32(host_shift, SGINewportVirtuixState),
        VMSTATE_UINT32(global_mask, SGINewportVirtuixState),

        /* DCB */
        VMSTATE_UINT32(dcb_mode, SGINewportVirtuixState),
        VMSTATE_UINT32(dcb_data_msw, SGINewportVirtuixState),
        VMSTATE_UINT32(dcb_data_lsw, SGINewportVirtuixState),

        /* Screenmask and clipping */
        VMSTATE_UINT32(top_scanline, SGINewportVirtuixState),
        VMSTATE_UINT32(xy_window, SGINewportVirtuixState),
        VMSTATE_UINT32(clip_mode, SGINewportVirtuixState),

        /* Config and status */
        VMSTATE_UINT32(config, SGINewportVirtuixState),
        VMSTATE_UINT32(status, SGINewportVirtuixState),

        /* VC2 */
        VMSTATE_UINT32(vc2_ram_addr, SGINewportVirtuixState),
        VMSTATE_UINT32(vc2_reg_idx, SGINewportVirtuixState),
        VMSTATE_UINT32(vc2_reg_data, SGINewportVirtuixState),
        VMSTATE_UINT16_ARRAY(vc2_ram, SGINewportVirtuixState, 32768),
        VMSTATE_UINT16_ARRAY(vc2_reg, SGINewportVirtuixState, 32),

        /* XMAP9 */
        VMSTATE_UINT32(xmap_config, SGINewportVirtuixState),
        VMSTATE_UINT32(xmap_revision, SGINewportVirtuixState),
        VMSTATE_UINT8(xmap_cursor_cmap, SGINewportVirtuixState),
        VMSTATE_UINT8(xmap_popup_cmap, SGINewportVirtuixState),
        VMSTATE_UINT8(xmap_mode_table_idx, SGINewportVirtuixState),
        VMSTATE_UINT32_ARRAY(xmap_mode_table, SGINewportVirtuixState, 32),

        /* CMAP */
        VMSTATE_UINT32(cmap_revision, SGINewportVirtuixState),
        VMSTATE_UINT16(cmap_palette_idx, SGINewportVirtuixState),
        VMSTATE_UINT32_ARRAY(cmap0_palette, SGINewportVirtuixState, 8192),

        /* RAMDAC */
        VMSTATE_UINT8(ramdac_lut_index, SGINewportVirtuixState),
        VMSTATE_UINT32_ARRAY(ramdac_lut_r, SGINewportVirtuixState, 256),
        VMSTATE_UINT32_ARRAY(ramdac_lut_g, SGINewportVirtuixState, 256),
        VMSTATE_UINT32_ARRAY(ramdac_lut_b, SGINewportVirtuixState, 256),

        /* VRAM (heap-allocated, fixed size) */
        VMSTATE_BUFFER_POINTER_UNSAFE(vram_rgbci, SGINewportVirtuixState, 0,
                                      NEWPORT_VRAM_BYTES),
        VMSTATE_BUFFER_POINTER_UNSAFE(vram_cidaux, SGINewportVirtuixState, 0,
                                      NEWPORT_VRAM_BYTES),

        /* Display dirty flag */
        VMSTATE_BOOL(display_dirty, SGINewportVirtuixState),

        VMSTATE_END_OF_LIST()
    }
};

static void sgi_newport_virtuix_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_newport_virtuix_realize;
    device_class_set_legacy_reset(dc, sgi_newport_virtuix_reset);
    dc->vmsd = &vmstate_sgi_newport;
    device_class_set_props(dc, sgi_newport_virtuix_properties);

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

static const TypeInfo sgi_newport_virtuix_info = {
    .name              = TYPE_SGI_NEWPORT_VIRTUIX,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(SGINewportVirtuixState),
    .instance_init     = sgi_newport_virtuix_init,
    .instance_finalize = sgi_newport_virtuix_finalize,
    .class_init        = sgi_newport_virtuix_class_init,
};

static void sgi_newport_virtuix_register_types(void)
{
    type_register_static(&sgi_newport_virtuix_info);
}

type_init(sgi_newport_virtuix_register_types)
