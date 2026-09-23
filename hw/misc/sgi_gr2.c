/*
 * SGI GR2 / "Express" graphics — register/FIFO shell.
 *
 * See include/hw/misc/sgi_gr2.h for the register map and the presence
 * contract.  This first pass is a register shell: a byte-backed register
 * file for the whole GIO64 board window, with the HQ2 presence magic and the
 * board-version register initialised so the stock `gr2` driver's probe
 * (Gr2Probe: magics + bdvers decode + GE RAM pattern test) recognises the
 * board.  HQ2 token dispatch, microcode download/verify and VC1/XMAP
 * bring-up come in later phases.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_gr2.h"
#include "trace.h"

/* Vertical retrace, as a real CRT would produce it.  The board raises its GIO
 * interrupt at ~60 Hz once started; the pulse is asserted for the blanking
 * interval and then lowered so the HPC3's level-triggered GIO2 source clears,
 * exactly like Newport's VBLANK model (see sgi_newport.c:newport_vblank_timer). */
#define SGI_GR2_RETRACE_HZ       60
#define SGI_GR2_RETRACE_PULSE_NS (500 * 1000) /* ~40 scanlines of blanking */

/* The RAMDAC palette is not a static table: the guest's DDX programs it
 * through the XMAP_PAL_* registers at server start, and the model builds
 * it from that write stream (see the header).  Reset clears it to black. */

/* Extract `size` big-endian bytes starting at byte `byte` of a 32-bit word. */
static uint64_t sgi_gr2_word_read(uint32_t word, unsigned byte, unsigned size)
{
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val = (val << 8) | ((word >> (8 * (3 - byte - i))) & 0xff);
    }
    return val;
}

/* Assemble `size` big-endian bytes from `value` into a 32-bit word. */
static uint32_t sgi_gr2_word_write(uint64_t value, unsigned byte, unsigned size)
{
    uint32_t word = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        word |= (uint8_t)(value >> (8 * (size - 1 - i))) << (8 * (3 - byte - i));
    }
    return word;
}

static void sgi_gr2_update_display(void *opaque);
static void sgi_gr2_vc1_advance(SGIGr2State *s);

/* Direct-colour 3-3-2 expansion (expDrawImage24).  In the 8-bit 3-3-2 visual the
 * stored byte is an RGB triple: bits 7-5 red, bits 4-3 blue, bits 2-0 green, each
 * a level into a gamma ramp.  The levels below are the SGI 8-bit visual's ramp as
 * measured from the reference and from the guest's own BT457 DAC writes (the
 * paltram sequence cycles 81,122,134,155,184,201,209,233).  The cube image,
 * EZsetup's icon, decodes 33/33 against this formula (see note 49). */
static const uint8_t sgi_gr2_ramp_rg[8] = { 0, 81, 122, 155, 184, 209, 233, 255 };
static const uint8_t sgi_gr2_ramp_b[4]  = { 0, 134, 201, 255 };

static uint32_t sgi_gr2_re3_332(uint8_t v)
{
    uint8_t r = sgi_gr2_ramp_rg[(v >> 5) & 7];
    uint8_t g = sgi_gr2_ramp_rg[v & 7];
    uint8_t b = sgi_gr2_ramp_b[(v >> 3) & 3];

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/* Every scanout write goes through one of these, so the direct-colour flag is
 * kept in step with the byte.  A later fill (for example `xsetroot -solid`) over
 * a pixel an image op once drew must clear the flag, or that pixel keeps being
 * expanded 3-3-2 and the fill comes out speckled. */
static inline void sgi_gr2_put(SGIGr2State *s, int x, int y, uint8_t idx)
{
    size_t o = (size_t)y * SGI_GR2_SCREEN_W + x;

    s->scanout[o] = idx;
    if (s->scanout332) {
        s->scanout332[o] = 0;
    }
}

static inline void sgi_gr2_put332(SGIGr2State *s, int x, int y, uint8_t idx)
{
    size_t o = (size_t)y * SGI_GR2_SCREEN_W + x;

    s->scanout[o] = idx;
    if (s->scanout332) {
        s->scanout332[o] = 1;
    }
}


/* RE3 solid-rectangle fill.  In the 8-bit mode the guest runs, the latched
 * colour is a RAMDAC index, and that is what the framebuffer stores — the
 * palette is applied at scanout, not here.  Returns the RGB the index maps to
 * right now, for tracing only. */
static uint32_t sgi_gr2_re3_fill(SGIGr2State *s, uint8_t colour,
                                 int x, int y, int w, int h)
{
    uint32_t rgb = s->ramdac[colour];
    int xx, yy;

    if (!s->scanout) {
        return rgb;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > SGI_GR2_SCREEN_W) {
        w = SGI_GR2_SCREEN_W - x;
    }
    if (y + h > SGI_GR2_SCREEN_H) {
        h = SGI_GR2_SCREEN_H - y;
    }
    for (yy = y; yy < y + h; yy++) {
        for (xx = x; xx < x + w; xx++) {
            sgi_gr2_put(s, xx, yy, colour);
        }
    }
    sgi_gr2_update_display(s);
    return rgb;
}

/* RE3 line segment (expSegmentSS/expLineSS).  In the 8-bit mode the pen colour
 * is a RAMDAC index, so the framebuffer stores the index traced here.  Bresenham
 * clipped to the screen. */
static void sgi_gr2_re3_line(SGIGr2State *s, uint8_t colour,
                             int x0, int y0, int x1, int y1)
{
    int dx, dy, sx, sy, err, e2;

    if (!s->scanout) {
        return;
    }
    dx = x1 > x0 ? x1 - x0 : x0 - x1;
    dy = y1 > y0 ? y1 - y0 : y0 - y1;
    sx = x0 < x1 ? 1 : -1;
    sy = y0 < y1 ? 1 : -1;
    err = dx - dy;
    for (;;) {
        if (x0 >= 0 && x0 < SGI_GR2_SCREEN_W &&
            y0 >= 0 && y0 < SGI_GR2_SCREEN_H) {
            sgi_gr2_put(s, x0, y0, colour);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        /* Bound the walk so a wild coordinate cannot spin. */
        if (x0 < -SGI_GR2_SCREEN_W || x0 > 2 * SGI_GR2_SCREEN_W ||
            y0 < -SGI_GR2_SCREEN_H || y0 > 2 * SGI_GR2_SCREEN_H) {
            break;
        }
        /* Both step decisions must see the SAME error term.  Re-reading err
         * after the first step (the old code did) makes a line overshoot its
         * endpoint and then wander to the bound below, painting a spurious
         * trail across the screen - 4421 of 20000 random segments in test. */
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
    sgi_gr2_update_display(s);
}

/* RE3 stippled-rectangle fill: the DDX arms a 32-bit stipple pattern on token
 * 318 and a foreground colour index on token 314, then streams the rectangle.
 * Render one pixel per bit of the pattern, repeating every 32 columns — bit 31
 * is the leftmost pixel of each 32-pixel tile.  This is the grainy root: the
 * X DDX's expStippledFillRects writes pattern 0x10101010 (a 1-in-4 dot). */
static void sgi_gr2_re3_stipple_fill(SGIGr2State *s, uint8_t fg, uint8_t bg,
                                     uint32_t pattern, int x, int y, int w, int h)
{
    int xx, yy;

    if (!s->scanout) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > SGI_GR2_SCREEN_W) {
        w = SGI_GR2_SCREEN_W - x;
    }
    if (y + h > SGI_GR2_SCREEN_H) {
        h = SGI_GR2_SCREEN_H - y;
    }
    for (yy = y; yy < y + h; yy++) {
        for (xx = x; xx < x + w; xx++) {
            bool on = (pattern >> (31 - (xx & 31))) & 1;

            sgi_gr2_put(s, xx, yy, on ? fg : bg);
        }
    }
    sgi_gr2_update_display(s);
}

/* expTileRects repeats the tile across its destination.  The destination is the
 * clip rectangle list set by the preceding expValidateClip ops (one per exposed
 * rect, seen as a 304 with four coordinate words, no colour token, 490
 * terminated).  The list persists until a tile consumes it; the startup
 * clusters that precede any list are left alone rather than filled full-screen,
 * which is what keeps the toolchest. */
static void sgi_gr2_re3_tile_rects(SGIGr2State *s)
{
    unsigned n = s->re3_data_n;
    uint32_t w, h, nwords, c0, c1;
    unsigned r;

    if (n < 7) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    w = s->re3_data[3];
    h = s->re3_data[4];
    if (w == 0 || h == 0 || (w & 3) || w > SGI_GR2_SCREEN_W ||
        h > SGI_GR2_SCREEN_H) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    nwords = (w * h) / 4;
    if (nwords == 0 || 7 + nwords - 1 > n) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    if (!s->scanout) {
        return;
    }
    /* The two tile colours are given by the op header, not by the tile pixels:
     * data[2] is the base and data[1] indexes the second colour.  The DDX forms
     * the base as (byte << 3) ("sll v1,v1,0x3" in expTileRects), so the root
     * weave's header (data[1]=3, data[2]=1) gives 1<<3 = 8 and 8 | (3<<1) = 14.
     * Index 8 is grey and 14 teal, matching the weave the expDrawImage24 restore
     * paints in the exposed rects (the same grey/teal the control shows). */
    c0 = s->re3_data[2] << 3;
    c1 = c0 | (s->re3_data[1] << 1);
    if (s->re3_nclip == 0) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    for (r = 0; r < s->re3_nclip; r++) {
        int rx1, ry1, rx2, ry2, x, y;

        rx1 = s->re3_clip[r][0];
        ry1 = s->re3_clip[r][1];
        rx2 = s->re3_clip[r][2];
        ry2 = s->re3_clip[r][3];
        if (rx1 < 0) {
            rx1 = 0;
        }
        if (ry1 < 0) {
            ry1 = 0;
        }
        if (rx2 > SGI_GR2_SCREEN_W) {
            rx2 = SGI_GR2_SCREEN_W;
        }
        if (ry2 > SGI_GR2_SCREEN_H) {
            ry2 = SGI_GR2_SCREEN_H;
        }
        for (y = ry1; y < ry2; y++) {
            unsigned row = (unsigned)(y % h) * w;

            for (x = rx1; x < rx2; x++) {
                uint32_t word, p = row + (unsigned)(x % w);
                uint8_t v, idx;

                word = (p / 4 == 0) ? s->re3_tile_word0
                                    : s->re3_data[7 + (p / 4 - 1)];
                v = (word >> (8 * (3 - (p % 4)))) & 0xff;
                if (v == (s->re3_data[1] & 0xff)) {
                    idx = c1;
                } else if (v == (s->re3_data[2] & 0xff)) {
                    idx = c0;
                } else {
                    idx = v;
                }
                sgi_gr2_put(s, x, y, idx);
            }
        }
    }
    trace_sgi_gr2_re3_tile(w, h, s->re3_nclip);
    s->re3_nclip = 0;
    sgi_gr2_update_display(s);
}

/* Number of trailing rectangle groups in the current sub-op's PUC_DATA.  The
 * DDX's expDrawSolidRects/expStippledFillRects store the geometry as groups of
 * four (x1,y1,x2,y2) after a short non-rectangle prefix ("0xff 0x3 0x0", and for
 * some ops the clip bounds 0x4ff 0x0 0x3ff before it).  Rather than assume the
 * prefix length, walk back from the end while each group is a sane rectangle:
 * the longest valid suffix is the list, and the prefix is left alone.  This is
 * the layout read off the DDX store sequences (note 31), not a guessed tail. */
static unsigned sgi_gr2_re3_rect_groups(SGIGr2State *s)
{
    unsigned n = s->re3_data_n, groups = 0;

    while ((groups + 1) * 4 <= n) {
        unsigned o = n - (groups + 1) * 4;
        uint32_t x1 = s->re3_data[o], y1 = s->re3_data[o + 1];
        uint32_t x2 = s->re3_data[o + 2], y2 = s->re3_data[o + 3];

        if (x1 >= x2 || y1 >= y2 || x1 >= SGI_GR2_SCREEN_W ||
            y1 >= SGI_GR2_SCREEN_H || x2 > SGI_GR2_SCREEN_W ||
            y2 > SGI_GR2_SCREEN_H) {
            break;
        }
        groups++;
    }
    return groups;
}

/* Paint a sub-op's rectangle list, flat (expDrawSolidRects) or stippled
 * (expStippledFillRects).  The rectangles are painted in payload order. */
static void sgi_gr2_re3_paint_rects(SGIGr2State *s, bool stippled)
{
    unsigned n = s->re3_data_n;
    unsigned groups = sgi_gr2_re3_rect_groups(s), g;
    uint8_t fg = s->re3_fg_valid ? s->re3_fg : s->re3_colour;

    if (groups == 0) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    for (g = groups; g > 0; g--) {
        unsigned o = n - g * 4;
        uint32_t x1 = s->re3_data[o], y1 = s->re3_data[o + 1];
        uint32_t x2 = s->re3_data[o + 2], y2 = s->re3_data[o + 3];
        int w = x2 - x1 + 1, h = y2 - y1 + 1;

        if (stippled) {
            sgi_gr2_re3_stipple_fill(s, fg, s->re3_colour, s->re3_stipple,
                                     x1, y1, w, h);
            trace_sgi_gr2_re3_rect(fg, s->ramdac[fg], x1, y1, x2, y2);
        } else {
            uint32_t rgb = sgi_gr2_re3_fill(s, s->re3_colour, x1, y1, w, h);

            /* The name-label bars are drawn as solid rects in colour 222 just
             * before their glyphs; remember the row so the glyph blit has a
             * baseline (the DDX does not put the text y on the wire). */
            if (s->re3_colour == 222) {
                s->re3_label_y = (int)y1;
                s->re3_label_valid = true;
            }
            trace_sgi_gr2_re3_rect(s->re3_colour, rgb, x1, y1, x2, y2);
        }
    }
}

/* Where the current sub-op's real payload begins: just past the LAST
 * "0xff 0x3 0x0" prefix.  A sub-op can hold more than one primitive (a nested
 * 331 leaves two prefixes in the buffer), and the payload belongs to the last
 * one — searching forward instead reads the pen colour/pattern words of the
 * inner primitive as geometry (that produced a stray diagonal across the
 * panel).  Returns the word count unchanged when no prefix is found. */
static unsigned sgi_gr2_re3_payload_start(SGIGr2State *s)
{
    unsigned n = s->re3_data_n, i;

    for (i = n; i >= 3; i--) {
        if (s->re3_data[i - 3] == 0xff && s->re3_data[i - 2] == 3 &&
            s->re3_data[i - 1] == 0) {
            return i;
        }
    }
    return n;
}

/* Draw a segment-list sub-op (expSegmentSS/expLineSS).  The DDX writes the pen
 * colour once, then the "0xff 0x3 0x0" prefix, then the geometry as groups of
 * four (x0,y0,x1,y1), ending with one or more (1280,1024) sentinels.  Locate the
 * prefix rather than assume its offset (some ops carry the clip bounds before
 * it), then read groups of four until a coordinate leaves the screen.  This is
 * the panel outline and the label strokes. */
static void sgi_gr2_re3_draw_segments(SGIGr2State *s)
{
    unsigned n = s->re3_data_n, i, start = sgi_gr2_re3_payload_start(s);

    if (start >= n || (n - start) % 4 != 0) {
        /* A payload that does not tile into (x0,y0,x1,y1) groups exactly is a
         * framing we do not understand — flag it rather than read a stray
         * diagonal out of it. */
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    for (i = start; i + 3 < n; i += 4) {
        uint32_t x0 = s->re3_data[i], y0 = s->re3_data[i + 1];
        uint32_t x1 = s->re3_data[i + 2], y1 = s->re3_data[i + 3];

        if (x0 >= SGI_GR2_SCREEN_W || y0 >= SGI_GR2_SCREEN_H ||
            x1 >= SGI_GR2_SCREEN_W || y1 >= SGI_GR2_SCREEN_H) {
            break;
        }
        sgi_gr2_re3_line(s, s->re3_colour, x0, y0, x1, y1);
        trace_sgi_gr2_re3_seg(s->re3_colour, s->ramdac[s->re3_colour],
                              x0, y0, x1, y1);
    }
}

/* Draw a stippled-span sub-op (expStippledSpans, token 347).  After the same
 * "0xff 0x3 0x0" prefix the geometry is groups of four (x, y, count, pattern):
 * a horizontal span of `count` pixels at row y, each pixel set when its bit in
 * the 32-bit pattern is 1 — the guest emits the root's discs as alternating
 * 0xaaaaaaaa/0x55555555 rows, a checkerboard.  Set bits take the 312 colour;
 * clear bits are left alone (the stipple is transparent). */
static void sgi_gr2_re3_draw_stippled_spans(SGIGr2State *s)
{
    unsigned n = s->re3_data_n, i, k, start = sgi_gr2_re3_payload_start(s);

    if (start >= n || (n - start) % 4 != 0) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    for (i = start; i + 3 < n; i += 4) {
        uint32_t x = s->re3_data[i], y = s->re3_data[i + 1];
        uint32_t count = s->re3_data[i + 2], pattern = s->re3_data[i + 3];

        if (x >= SGI_GR2_SCREEN_W || y >= SGI_GR2_SCREEN_H ||
            count == 0 || count > 64) {
            trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
            break;
        }
        trace_sgi_gr2_re3_spanstip(s->re3_colour, s->ramdac[s->re3_colour],
                                   x, y, count);
        for (k = 0; k < count; k++) {
            uint32_t xx = x + k;

            if (xx >= SGI_GR2_SCREEN_W) {
                break;
            }
            if ((pattern >> (31 - (k & 31))) & 1) {
                sgi_gr2_put(s, xx, y, s->re3_colour);
            }
        }
    }
    sgi_gr2_update_display(s);
}

/* Draw a colour image (expDrawImage24, token 342).  Each 342 starts one run and
 * streams, through PUC_DATA, a group of (y, width, height, nwords_per_row, 2, 0)
 * followed by height*nwords_per_row 32-bit words, each holding four 8-bit
 * palette indices, most significant byte first - so one row is nwords_per_row
 * words, four pixels each.  The run x is 342's value; its data index is recorded
 * at write time, so the decoder needs no guessed geometry.
 *
 * The 98x98 icon arrives as scanlines, each one row high, two runs per row
 * (x=459 w=64 and x=523 w=34); the text cursor in the login field arrives as
 * 8-row blocks of 6-px rows at x=464.  Every group is padded with 0xdeadbeef to
 * a fixed 22 PUC_DATA words.  The indices are written to scanout as-is; the
 * palette is applied there, like every other RE3 draw (see sgi_gr2_re3_332). */
static void sgi_gr2_re3_draw_image(SGIGr2State *s)
{
    unsigned k;

    if (!s->scanout) {
        return;
    }
    for (k = 0; k < s->re3_nimg; k++) {
        unsigned off = s->re3_img_off[k];
        uint32_t x = s->re3_img_x[k];
        uint32_t y, w, height, nrow, j, p;
        bool m332;

        if (off + 6 > s->re3_data_n) {
            break;
        }
        y = s->re3_data[off];
        w = s->re3_data[off + 1];
        height = s->re3_data[off + 2];
        nrow = s->re3_data[off + 3];
        /* The group header's sixth word selects the pixel interpretation: 0 means
         * the bytes are RAMDAC indices (the root weave, grey/teal), 2 means they
         * are direct 3-3-2 (the EZsetup cube and the desktop icons).  Both arrive
         * as expDrawImage24, so the header is the only discriminator. */
        m332 = (s->re3_data[off + 5] == 2);
        if (w == 0 || w > SGI_GR2_SCREEN_W || y >= SGI_GR2_SCREEN_H ||
            x >= SGI_GR2_SCREEN_W || height == 0 ||
            height > SGI_GR2_SCREEN_H || nrow == 0 ||
            nrow * height > SGI_GR2_RE3_DATA_MAX) {
            trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
            continue;
        }
        if (off + 6 + nrow * height > s->re3_data_n) {
            break;
        }
        if (k == 0) {
            trace_sgi_gr2_re3_image(x, y, w, s->re3_nimg);
        }
        for (j = 0; j < height; j++) {
            uint32_t r;

            for (r = 0; r < nrow; r++) {
                uint32_t word = s->re3_data[off + 6 + j * nrow + r];

                for (p = 0; p < 4; p++) {
                    uint32_t px = x + r * 4 + p, py = y + j;

                    if (px >= x + w || px >= SGI_GR2_SCREEN_W ||
                        py >= SGI_GR2_SCREEN_H) {
                        continue;
                    }
                    if (m332) {
                        sgi_gr2_put332(s, px, py, (word >> (24 - 8 * p)) & 0xff);
                    } else {
                        sgi_gr2_put(s, px, py, (word >> (24 - 8 * p)) & 0xff);
                    }
                }
            }
        }
    }
    sgi_gr2_update_display(s);
}

/* Draw a polyline (libgd token 302).  After the same "0xff 0x3 0x0" prefix come
 * three header words (0x3ab, 0xc6, 0x2a0 in every op) then one or more contours
 * as (x,y) vertex pairs, each closed by repeating its first vertex.  These are
 * the account icons: the reference shows light interiors with thin black edges,
 * so the contours are STROKED (fill is the fallback, behind sgi-gr2.poly-stroke).
 * The payload list is split on the repeated first vertex so one contour is never
 * joined to the next. */
#define SGI_GR2_POLY_MAX 64
static void sgi_gr2_re3_draw_polygon(SGIGr2State *s)
{
    int vx[SGI_GR2_POLY_MAX], vy[SGI_GR2_POLY_MAX];
    unsigned n = s->re3_data_n, i, start = n, nv = 0;
    int ymin, ymax, sy;

    start = sgi_gr2_re3_payload_start(s);
    if (start + 3 < n && s->re3_data[start] == 0x3ab &&
        s->re3_data[start + 1] == 0xc6 && s->re3_data[start + 2] == 0x2a0) {
        /* The three-word header that precedes the outline in every op. */
        start += 3;
    }
    if (start >= n || (n - start) % 2 != 0) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    for (i = start; i + 1 < n && nv < SGI_GR2_POLY_MAX; i += 2) {
        int x = s->re3_data[i], y = s->re3_data[i + 1];

        if (x < 0 || x >= SGI_GR2_SCREEN_W || y < 0 || y >= SGI_GR2_SCREEN_H) {
            break;
        }
        vx[nv] = x;
        vy[nv] = y;
        nv++;
    }
    if (nv < 3) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, n);
        return;
    }
    if (s->poly_stroke) {
        /* Test seam: stroke each contour (split where a vertex repeats the
         * contour's first) instead of filling.  Kept behind a runtime property
         * so one build can be A/B'd with no rebuild or trace differences. */
        unsigned cstart = 0;

        for (i = 1; i < nv; i++) {
            if (i - cstart >= 2 && vx[i] == vx[cstart] && vy[i] == vy[cstart]) {
                unsigned k;

                for (k = cstart; k + 1 <= i; k++) {
                    trace_sgi_gr2_re3_seg(s->re3_colour, s->ramdac[s->re3_colour],
                                          vx[k], vy[k], vx[k + 1], vy[k + 1]);
                    sgi_gr2_re3_line(s, s->re3_colour, vx[k], vy[k],
                                     vx[k + 1], vy[k + 1]);
                }
                cstart = i + 1;
            }
        }
        sgi_gr2_update_display(s);
        return;
    }
    ymin = ymax = vy[0];
    for (i = 1; i < nv; i++) {
        ymin = MIN(ymin, vy[i]);
        ymax = MAX(ymax, vy[i]);
    }
    for (sy = ymin; sy <= ymax; sy++) {
        int xs[SGI_GR2_POLY_MAX];
        unsigned m = 0, k;
        int j;

        for (i = 0; i < nv; i++) {
            unsigned jj = (i + 1) % nv;
            int y0 = vy[i], y1 = vy[jj], x0 = vx[i], x1 = vx[jj];

            if ((y0 <= sy && y1 > sy) || (y1 <= sy && y0 > sy)) {
                xs[m++] = x0 + (int)(((int64_t)(sy - y0) * (x1 - x0)) /
                                     (y1 - y0));
            }
        }
        for (k = 1; k < m; k++) {
            int t = xs[k];

            for (j = k; j > 0 && xs[j - 1] > t; j--) {
                xs[j] = xs[j - 1];
            }
            xs[j] = t;
        }
        for (k = 0; k + 1 < m; k += 2) {
            int xa = MAX(xs[k], 0), xb = MIN(xs[k + 1], SGI_GR2_SCREEN_W - 1);
            int xx;

            for (xx = xa; xx <= xb; xx++) {
                sgi_gr2_put(s, xx, sy, s->re3_colour);
            }
        }
    }
    sgi_gr2_update_display(s);
}

/* Draw the text glyphs (expDrawMonoImage).  Token 349 is the pen x; it is
 * followed by a piece header (f0, f1, h) and the piece's bitmap.  The first
 * header word f0 is the piece's DESTINATION Y - verified by placing every glyph
 * run at y=f0: "Login name:", the four account names, and the "Log in"/"Help"
 * buttons all land correctly, and the second piece of a glyph carries f0+8, one
 * piece-height lower, so no separate stacking rule is needed.  (The old code
 * used a y latched from a colour-222 bar, which only existed for the account
 * names, so the static labels were dropped.)
 *
 * A piece is h rows tall and SIXTEEN pixels wide, packed TWO rows per 32-bit
 * word: row 2k is the high half (bits 31-16), row 2k+1 the low half (bits 15-0),
 * bit 15 the leftmost pixel.  The word count is ceil(h/2), so an odd h keeps its
 * last row in the high half.  Set bits take the current colour. */
static void sgi_gr2_re3_draw_text(SGIGr2State *s)
{
    unsigned p;
    bool any = false;

    for (p = 0; p < s->re3_npens; p++) {
        unsigned off = s->re3_pen_off[p];
        uint32_t pen = s->re3_pen_val[p];
        uint32_t f0, h, k, nwords;

        if (off + 3 > s->re3_data_n) {
            continue;
        }
        f0 = s->re3_data[off];       /* destination y */
        h = s->re3_data[off + 2];
        nwords = (h + 1) / 2;
        if (f0 >= SGI_GR2_SCREEN_H || pen >= SGI_GR2_SCREEN_W ||
            h < 1 || h > 64 ||
            off + 3 + nwords > s->re3_data_n) {
            continue;
        }
        for (k = 0; k < nwords; k++) {
            uint32_t w = s->re3_data[off + 3 + k];
            int r0 = (int)f0 + 2 * (int)k;
            int b;

            for (b = 0; b < 16; b++) {
                int xx = (int)pen + b;

                if (xx >= SGI_GR2_SCREEN_W) {
                    break;
                }
                if (r0 < SGI_GR2_SCREEN_H && ((w >> (31 - b)) & 1)) {
                    sgi_gr2_put(s, xx, r0, s->re3_colour);
                }
                if (2 * k + 1 < h && r0 + 1 < SGI_GR2_SCREEN_H &&
                    ((w >> (15 - b)) & 1)) {
                    sgi_gr2_put(s, xx, r0 + 1, s->re3_colour);
                }
            }
        }
        any = true;
    }
    if (!any) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
    sgi_gr2_update_display(s);
}

/* expCopyRect screen-to-screen (token 340).  The DDX writes 340 = words-per-row
 * and then seven PUC_DATA words: stride, src_x, src_y, width, height, dst_x,
 * dst_y.  Both source and destination are on the scanout, which is the
 * window-move / scroll case; an off-screen pixmap source is not modelled yet.
 * Overlapping copies run in the direction that does not clobber unread source
 * rows.  Indices and the direct-colour flag move together. */
static void sgi_gr2_re3_copy_rect(SGIGr2State *s, int sx, int sy, int w, int h,
                                  int dx, int dy)
{
    int r;

    if (w <= 0 || h <= 0) {
        return;
    }
    if (sx < 0) { w += sx; dx -= sx; sx = 0; }
    if (sy < 0) { h += sy; dy -= sy; sy = 0; }
    if (dx < 0) { w += dx; sx -= dx; dx = 0; }
    if (dy < 0) { h += dy; sy -= dy; dy = 0; }
    if (sx + w > SGI_GR2_SCREEN_W) { w = SGI_GR2_SCREEN_W - sx; }
    if (dx + w > SGI_GR2_SCREEN_W) { w = SGI_GR2_SCREEN_W - dx; }
    if (sy + h > SGI_GR2_SCREEN_H) { h = SGI_GR2_SCREEN_H - sy; }
    if (dy + h > SGI_GR2_SCREEN_H) { h = SGI_GR2_SCREEN_H - dy; }
    if (w <= 0 || h <= 0) {
        return;
    }
    if (dy < sy || (dy == sy && dx < sx)) {
        for (r = 0; r < h; r++) {
            size_t so = (size_t)(sy + r) * SGI_GR2_SCREEN_W + sx;
            size_t dofs = (size_t)(dy + r) * SGI_GR2_SCREEN_W + dx;

            memmove(s->scanout + dofs, s->scanout + so, w);
            if (s->scanout332) {
                memmove(s->scanout332 + dofs, s->scanout332 + so, w);
            }
        }
    } else {
        for (r = h - 1; r >= 0; r--) {
            size_t so = (size_t)(sy + r) * SGI_GR2_SCREEN_W + sx;
            size_t dofs = (size_t)(dy + r) * SGI_GR2_SCREEN_W + dx;

            memmove(s->scanout + dofs, s->scanout + so, w);
            if (s->scanout332) {
                memmove(s->scanout332 + dofs, s->scanout332 + so, w);
            }
        }
    }
    trace_sgi_gr2_re3_copy(sx, sy, w, h, dx, dy);
    sgi_gr2_update_display(s);
}

/* Evaluate the pending draw sub-op.  Called at token 331 (which starts a new
 * sub-op) and token 490 (the terminator), because one 490-terminated region can
 * hold several 331 sub-ops: the grainy root's stipple and the panel's own
 * solid-rect list live in the SAME region, the panel as a nested 331.  The
 * sub-op is classified by the markers the DDX store order leaves behind — 318
 * stipple, 304 solid rect, 305 spans — per the dispatch table in note 31, and
 * anything matching none of those is traced as unmatched rather than guessed
 * at. */
static void sgi_gr2_re3_flush_fill(SGIGr2State *s)
{
    if (s->re3_solid_seen && s->re3_data_n == 4 && !s->re3_colour_valid) {
        /* expValidateClip: one exposed rectangle for a following tile fill.  It
         * carries the 304 marker and four (x1,y1,x2,y2) words but no colour, so
         * it is a clip rectangle, not a fill.  The tile op that follows repeats
         * its bitmap inside this list. */
        if (s->re3_nclip < ARRAY_SIZE(s->re3_clip)) {
            s->re3_clip[s->re3_nclip][0] = s->re3_data[0];
            s->re3_clip[s->re3_nclip][1] = s->re3_data[1];
            s->re3_clip[s->re3_nclip][2] = s->re3_data[2];
            s->re3_clip[s->re3_nclip][3] = s->re3_data[3];
            s->re3_nclip++;
            trace_sgi_gr2_re3_clip(s->re3_data[0], s->re3_data[1],
                                   s->re3_data[2], s->re3_data[3]);
        }
        return;
    }
    if (s->re3_tile_seen) {
        /* expTileRects: tile a bitmap across the screen (the root weave).  The
         * op streams the tile via token 315 and PUC_DATA, so decide it here from
         * the packet itself rather than from any rect heuristic. */
        sgi_gr2_re3_tile_rects(s);
        return;
    }
    if (s->re3_image_seen) {
        /* expDrawImage24: a colour image, not a fill.  Checked first because its
         * 4316 PUC_DATA words and 196 342 markers match no other shape. */
        sgi_gr2_re3_draw_image(s);
        return;
    }
    if (s->re3_stipple_valid) {
        /* Stippled rect list: the root backdrop is one full-screen rect, but a
         * stippled sub-op with its own small rects (a cursor, a shade band) must
         * paint only those — painting the whole screen here would wipe them. */
        sgi_gr2_re3_paint_rects(s, true);
        return;
    }
    if (s->re3_spans_seen) {
        /* expSolidSpans: a span list, not a fill.  The PUC path draws the
         * weave spans inline; the DDX span op itself paints nothing here. */
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
    if (s->re3_spanstip_seen) {
        /* Checked before the mono-image path: token 312 marks BOTH
         * expImageGlyphBlt and expOpStippledFillRects, so a span op carries the
         * 312 marker without any glyph in it.  The 347 span token is the one
         * that actually says "span list", and drawing such an op as text would
         * drop the whole cube (EZsetup's icon). */
        sgi_gr2_re3_draw_stippled_spans(s);
        return;
    }
    if (s->re3_mono_seen && s->re3_npens) {
        /* Glyphs need a 349 pen to be placed; a 312 with no pen is not text. */
        sgi_gr2_re3_draw_text(s);
        return;
    }
    if (s->re3_poly_seen) {
        sgi_gr2_re3_draw_polygon(s);
        return;
    }
    if (s->re3_line_seen) {
        sgi_gr2_re3_draw_segments(s);
        return;
    }
    if (s->re3_solid_seen) {
        /* Token 309 is the sub-op's GC/ROP mode.  Real fills seen so far carry
         * 0; the weave op's trailing full-screen black rect carries 0xf, and
         * painting it would wipe the weave, so a non-zero mode is flagged, not
         * painted, until its meaning is pinned. */
        if (s->re3_rop == 0) {
            sgi_gr2_re3_paint_rects(s, false);
        } else {
            trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        }
        return;
    }
    if (s->re3_data_n || s->re3_data_overflow) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
    }
}

/* Clear the per-sub-op state.  The data buffer and the ROP latch belong to one
 * sub-op, so they reset with the markers; the PUC span state does not. */
static void sgi_gr2_re3_reset_subop(SGIGr2State *s)
{
    s->re3_solid_seen = false;
    s->re3_spans_seen = false;
    s->re3_line_seen = false;
    s->re3_spanstip_seen = false;
    s->re3_poly_seen = false;
    s->re3_mono_seen = false;
    s->re3_image_seen = false;
    s->re3_nimg = 0;
    s->re3_npens = 0;
    s->re3_pair_seen = false;
    s->re3_stipple_valid = false;
    s->re3_fg_valid = false;
    s->re3_rop_valid = false;
    s->re3_data_n = 0;
    s->re3_data_overflow = false;
    s->re3_copy_active = false;
    s->re3_copy_n = 0;
    s->re3_tile_seen = false;
    s->re3_tile_word0 = 0;
}

static uint64_t sgi_gr2_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIGr2State *s = SGI_GR2(opaque);
    uint64_t val = 0;
    unsigned i;

    if (!s->present) {
        /* Absent board: the GIO slot reads all-ones, exactly like the
         * empty-slot stub, so a plain `-M indy` boot is unchanged. */
        return ~0ULL;
    }
    if (offset + size > SGI_GR2_REG_SIZE) {
        return ~0ULL;
    }
    /* GE7 instruction load/verify: the four-word window and the load register
     * are per-PC storage, selected by the last gepc write.  Returned verbatim
     * (the driver masks with 0x3dfffff itself). */
    if (offset >= SGI_GR2_GE_WIN_OFF &&
        offset < SGI_GR2_GE_WIN_OFF + SGI_GR2_GE_WIN_WORDS * 4) {
        unsigned idx = (offset - SGI_GR2_GE_WIN_OFF) / 4;

        val = sgi_gr2_word_read(
            s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][idx], offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_UCODELOAD &&
               offset < SGI_GR2_HQ_UCODELOAD + 4) {
        val = sgi_gr2_word_read(
            s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][4], offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_GEPC && offset < SGI_GR2_HQ_GEPC + 4) {
        val = sgi_gr2_word_read(s->gepc, offset & 3, size);
    } else if (offset >= SGI_GR2_HQ_FIFOSTAT &&
               offset < SGI_GR2_HQ_FIFOSTAT + 4) {
        /* HQ2 status: bit 0 = HQ2 idle/ready (the X DDX's expInit spins on
         * it — lw 0x6a040; andi 0x1; beqz), bit 1 = ucode ready (the kernel's
         * _Gr2UcodeReady).  Occupancy stays empty. */
        val = sgi_gr2_word_read(
            SGI_GR2_HQ_IDLE_BIT |
            (s->hq_ready ? SGI_GR2_HQ_READY_BIT : 0), offset & 3, size);
    } else if (offset >= SGI_GR2_XMAP_STATUS &&
               offset < SGI_GR2_XMAP_STATUS + 4) {
        /* XMAP status: bit 1 reports "ready"; _Gr2XMAPInit3 spins on it.
         * The driver byte-reads this (lbu), so the bit must appear in the
         * addressed byte lane, not as bit 1 of the whole 32-bit word. */
        val = s->xmap_ready ? SGI_GR2_XMAP_READY_BIT : 0;
    } else {
        /* GE units at or above the variant's engine count are not populated,
         * so the driver's GE-count pattern test stops counting there. */
        if (offset >= SGI_GR2_GE_OFF &&
            offset < SGI_GR2_GE_OFF + SGI_GR2_GE_UNITS * SGI_GR2_GE_STRIDE) {
            unsigned unit = (offset - SGI_GR2_GE_OFF) / SGI_GR2_GE_STRIDE;

            if (unit >= s->ges) {
                return ~0ULL;
            }
        }
        /* Big-endian byte assembly so byte/half/word accesses agree. */
        for (i = 0; i < size; i++) {
            val = (val << 8) | s->regs[offset + i];
        }
    }
    /* The command/register traffic worth watching (µcode load staging, HQ2,
     * GE and RE3) is all at or above the HQ2 block; the shram and token FIFO
     * are huge and mostly idle. */
    if (offset < 0x40 || offset >= SGI_GR2_HQUCODE_OFF) {
        trace_sgi_gr2_read(offset, size, val);
    }
    /* The HQ2 block is the polled surface: log each access with the reading PC
     * so the poll loop (and the value it expects) can be read off directly. */
    if (offset >= SGI_GR2_HQ_OFF && offset < SGI_GR2_HQ_OFF + 0x80) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_hqread(offset, val, pc);
    }
    /* The token FIFO is written as the command channel; any READ of it is the
     * board's read-back/consumption contract, so log those with the PC too. */
    if (offset >= SGI_GR2_FIFO_OFF && offset < SGI_GR2_FIFO_OFF + 0x20000) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_fiforead(offset, val, pc);
    }
    /* VC1 / XMAP / RE3 / GE / bdvers reads with the PC: the last surface that
     * could hold the "display is up" gate the DDX waits on. */
    if (offset >= SGI_GR2_HQUCODE_OFF) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_regread(offset, val, pc);
    }
    return val;
}

static void sgi_gr2_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    SGIGr2State *s = SGI_GR2(opaque);
    unsigned i;

    if (!s->present || offset + size > SGI_GR2_REG_SIZE) {
        return;
    }
    /* The command/register traffic worth watching (µcode load staging, HQ2,
     * GE and RE3) is all at or above the HQ2 block; the shram and token FIFO
     * are huge and mostly idle. */
    if (offset < 0x40 || offset >= SGI_GR2_HQUCODE_OFF) {
        trace_sgi_gr2_write(offset, value, size);
    }
    /* Token-FIFO traffic (fifo[token] = data) is the producer/consumer channel;
     * trace it separately and cheaply so a draw window does not require tracing
     * the whole register block. */
    if (offset >= SGI_GR2_FIFO_OFF && offset < SGI_GR2_FIFO_OFF + 0x20000) {
        trace_sgi_gr2_fifo(offset, value, size);
        /* Arm the drain flush; it fires only once the token stream pauses, which
         * is when real hardware would have executed the op. */
        if (s->fifo_flush_timer) {
            timer_mod(s->fifo_flush_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                      SGI_GR2_FIFO_FLUSH_MS);
        }
    }
    /* VC1: an addressed 16-bit register file and SRAM, both auto-incrementing.
     * addrlo/addrhi carry a byte address; cmd0 is the register port (the cursor's
     * x/y go here) and sram the bitmap port (Gr2LoadVC1SRAM streams the cursor). */
    if (offset == SGI_GR2_VC1_ADDRLO) {
        s->vc1_addrlo = value & 0xff;
        return;
    }
    if (offset == SGI_GR2_VC1_ADDRHI) {
        s->vc1_addrhi = value & 0xff;
        return;
    }
    if (offset == SGI_GR2_VC1_CMD0) {
        unsigned a = ((((unsigned)s->vc1_addrhi << 8) | s->vc1_addrlo) >> 1);

        if (a < SGI_GR2_VC1_REG_WORDS) {
            s->vc1_reg[a] = value & 0xffff;
        }
        sgi_gr2_vc1_advance(s);
        sgi_gr2_update_display(s);
        return;
    }
    if (offset == SGI_GR2_VC1_SRAM) {
        unsigned a = ((((unsigned)s->vc1_addrhi << 8) | s->vc1_addrlo) >> 1);

        if (a < SGI_GR2_VC1_SRAM_WORDS) {
            s->vc1_sram[a] = value & 0xffff;
        }
        sgi_gr2_vc1_advance(s);
        return;
    }
    /* RE3 producer, 8-bit mode.  The DDX writes the fill colour (a RAMDAC
     * INDEX) to the RE3 colour token, then pushes the rectangle's geometry as
     * the last two PUC_DATA words.  [ASSUMPTION, from the one captured
     * `xsetroot -solid red` stream: its PUC_DATA tail was 0x500 then 0x400 —
     * 1280 x 1024, the screen size — a full-screen clear; and RED vs BLUE
     * differed in exactly the colour token, 0x1 vs 0x4.]  So latch the colour,
     * and on the (w,h) pair fill that rectangle of the scanout buffer. */
    if (size == 4 && offset == SGI_GR2_RE3_COLOUR_TOKEN) {
        s->re3_colour = value & 0xff;
        s->re3_colour_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_HQ_TOKEN_START) {
        /* The full-screen fill is decided at the op terminator (token 490), not
         * here.  An expSolidSpans payload ends with the span triple
         * (0x500, 0x400, 0x1) — x, y, count — so 0x400 DOES follow 0x500 in the
         * middle of a span list, and testing inline fires on a span that is not
         * a rectangle.  That misfire is what wiped the weave (note 28).  Only
         * the op's FINAL two data words are the rect geometry. */
        /* The generic PUC path: PUC_COLOR chose the colour, PUC_RECTI2D armed
         * a rectangle, and its three PUC_DATA words are (x0, x1, y0) — one
         * horizontal span.  The root weave is 1024 of these. */
        if (s->puc_rect_armed) {
            s->puc_rect[s->puc_rect_n++] = value;
            if (s->puc_rect_n == 3) {
                uint32_t x0 = s->puc_rect[0];
                uint32_t x1 = s->puc_rect[1];
                uint32_t y0 = s->puc_rect[2];

                if (x1 >= x0 && x0 < SGI_GR2_SCREEN_W && y0 < SGI_GR2_SCREEN_H) {
                    uint32_t rgb = sgi_gr2_re3_fill(s, s->puc_colour, x0, y0,
                                                    x1 - x0 + 1, 1);

                    trace_sgi_gr2_puc_span(s->puc_colour, rgb, x0, x1, y0);
                }
                s->puc_rect_armed = false;
                s->puc_rect_n = 0;
            }
        }
        s->prev_puc = s->last_puc;
        s->last_puc = value;
        s->last_puc_valid = true;
        /* expCopyRect: the seven words after 340 close the op (no 490). */
        if (s->re3_copy_active) {
            s->re3_copy_v[s->re3_copy_n++] = value;
            if (s->re3_copy_n == 7) {
                sgi_gr2_re3_copy_rect(s, (int)s->re3_copy_v[1],
                                      (int)s->re3_copy_v[2],
                                      (int)s->re3_copy_v[3],
                                      (int)s->re3_copy_v[4],
                                      (int)s->re3_copy_v[5],
                                      (int)s->re3_copy_v[6]);
                s->re3_copy_active = false;
            }
        }
        /* Keep the sub-op's PUC_DATA for the rect-list decoder.  The weave op's
         * 1024 spans arrive before its own 331, so they fill and then reset the
         * buffer at the 331; only the tail that belongs to the 331 matters. */
        if (s->re3_data_n < SGI_GR2_RE3_DATA_MAX) {
            s->re3_data[s->re3_data_n++] = value;
        } else {
            s->re3_data_overflow = true;
        }
        if (s->prev_puc == SGI_GR2_SCREEN_W &&
            s->last_puc == SGI_GR2_SCREEN_H) {
            s->re3_pair_seen = true;
        }
    }
    if (size == 4 && offset == SGI_GR2_PUC_COLOR_TOKEN) {
        s->puc_colour = value & 0xff;
    }
    if (size == 4 && offset == SGI_GR2_PUC_RECTI2D_TOKEN) {
        s->puc_rect_armed = true;
        s->puc_rect_n = 0;
    }
    /* Fill-op markers (DDX token map, note 25/26).  An op starts at token 331,
     * so the per-op markers are cleared there and set by 304 (solid rect),
     * 314 (stipple fg colour) and 318 (stipple pattern); token 490 is the op
     * terminator.  This is what distinguishes a grainy stippled root fill from
     * a flat solid fill without guessing at the PUC_DATA tail. */
    if (size == 4 && offset == SGI_GR2_RE3_OP_TOKEN) {
        sgi_gr2_re3_flush_fill(s);
        sgi_gr2_re3_reset_subop(s);
        /* The op type is (type | 0x1000); 0x100b is expTileRects, which streams
         * its tile bitmap and repaints the root.  Remember it for the flush that
         * runs when the next op's 331 arrives.  Any other op ends the clip list
         * the tile fill consumes. */
        s->re3_tile_seen = (value == SGI_GR2_RE3_TILE_OP);
    }
    if (size == 4 && offset == SGI_GR2_RE3_MODE_TOKEN) {
        s->re3_rop = (uint32_t)value;
        s->re3_rop_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_LINE_TOKEN) {
        s->re3_line_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_SPANSTIP_TOKEN) {
        s->re3_spanstip_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_POLY_TOKEN) {
        s->re3_poly_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_MONO_TOKEN) {
        s->re3_mono_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_IMAGE_TOKEN) {
        /* One image run: remember its x and where its PUC_DATA group begins. */
        if (s->re3_nimg < SGI_GR2_RE3_IMG_MAX) {
            s->re3_img_x[s->re3_nimg] = (uint32_t)value;
            s->re3_img_off[s->re3_nimg] = s->re3_data_n;
            s->re3_nimg++;
        }
        s->re3_image_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_COPY_TOKEN) {
        /* expCopyRect: 340 = words-per-row, then seven PUC_DATA words.  No 490,
         * so the copy fires on the seventh. */
        s->re3_copy_wpr = (uint32_t)value;
        s->re3_copy_n = 0;
        s->re3_copy_active = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_PEN_TOKEN) {
        /* The pen x for one glyph piece; remember where its data starts. */
        if (s->re3_npens < SGI_GR2_RE3_PEN_MAX) {
            s->re3_pen_val[s->re3_npens] = (uint32_t)value & 0xffff;
            s->re3_pen_off[s->re3_npens] = s->re3_data_n;
            s->re3_npens++;
        }
        s->re3_mono_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_SOLID_TOKEN) {
        s->re3_solid_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_SPANS_TOKEN) {
        s->re3_spans_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_TILE_TOKEN) {
        /* expTileRects streams its tile bitmap's first word on the tile data
         * port; the rest arrives as PUC_DATA.  Keep it so the tile can be
         * reconstructed from both streams. */
        if (s->re3_tile_seen) {
            s->re3_tile_word0 = (uint32_t)value;
        }
    }
    if (size == 4 && offset == SGI_GR2_RE3_FG_TOKEN) {
        s->re3_fg = value & 0xff;
        s->re3_fg_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_STIPPLE_TOKEN) {
        s->re3_stipple = (uint32_t)value;
        s->re3_stipple_valid = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_DONE_TOKEN) {
        /* Op end.  Deciding here — not inline on each PUC_DATA — is what keeps
         * an expSolidSpans triple list from being mistaken for a rect. */
        sgi_gr2_re3_flush_fill(s);
        s->re3_colour_valid = false;
        sgi_gr2_re3_reset_subop(s);
    }
    /* HQ2-block writes (start / DMA control / FIFO thresholds) with the PC. */
    if (offset >= SGI_GR2_HQ_OFF && offset < SGI_GR2_HQ_OFF + 0x80) {
        uint32_t pc = current_cpu ? (uint32_t)current_cpu->mem_io_pc : 0;

        trace_sgi_gr2_hqwrite(offset, value, pc);
    }
    /* GE7 instruction load/verify: store the window and load-register words
     * verbatim into the per-PC slot selected by the last gepc write.  No
     * masking or tidying — the driver compares against what it wrote. */
    if (offset >= SGI_GR2_GE_WIN_OFF &&
        offset < SGI_GR2_GE_WIN_OFF + SGI_GR2_GE_WIN_WORDS * 4) {
        unsigned idx = (offset - SGI_GR2_GE_WIN_OFF) / 4;

        s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][idx] =
            sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    if (offset >= SGI_GR2_HQ_UCODELOAD &&
        offset < SGI_GR2_HQ_UCODELOAD + 4) {
        s->ucode[s->gepc & (SGI_GR2_UCODE_PCS - 1)][4] =
            sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    if (offset >= SGI_GR2_HQ_GEPC && offset < SGI_GR2_HQ_GEPC + 4) {
        s->gepc = sgi_gr2_word_write(value, offset & 3, size);
        return;
    }
    /* Gr2Start kicks the HQ2 sequencer by writing the start token to the
     * command FIFO, then polls the "ucode ready" bit; model the sequencer as
     * ready from that write on.  The token itself is still stored below. */
    if (offset >= SGI_GR2_HQ_TOKEN_START &&
        offset < SGI_GR2_HQ_TOKEN_START + 4) {
        s->hq_ready = true;
    }
    /* Programming the XMAP control registers makes the mode generator ready
     * for the driver's poll. */
    if (offset >= SGI_GR2_XMAP_CTL_OFF && offset < SGI_GR2_XMAP_CTL_END) {
        s->xmap_ready = true;
    }
    /* RAMDAC colour-map programming (XMAP_PAL_*): the DDX writes the entry
     * index to 0x6c1b0, a control byte to 0x6c1b4, and a sliding R,G,B byte
     * stream to 0x6c1a8 — each 32-bit data write carries this entry's R,G,B
     * plus the first byte (R) of the next entry, which is why the index for a
     * completing entry is always written before the write that completes it.
     * Build the palette from those bytes; nothing is assumed about its values. */
    if (offset == SGI_GR2_XMAP_PAL_INDEX) {
        s->ramdac_index = value & 0xff;
        /* Each entry's bytes start fresh after its index write; a 32-bit data
         * write carries a 4th byte (the next entry's R) which the following
         * index write therefore discards.  Validated against the `xwd` oracle:
         * this framing reproduces it (index 1 = red, 4 = blue, 0x10 =
         * (0x4c,0x71,0x9e), 0x20..0x37 = the grey ramp). */
        s->ramdac_stage_n = 0;
    } else if (offset == SGI_GR2_XMAP_PAL_DATA) {
        unsigned k;

        for (k = 0; k < size; k++) {
            uint8_t byte = (value >> (8 * (size - 1 - k))) & 0xff;

            s->ramdac_stage[s->ramdac_stage_n++] = byte;
            if (s->ramdac_stage_n == 3) {
                uint32_t rgb = ((uint32_t)s->ramdac_stage[0] << 16) |
                               ((uint32_t)s->ramdac_stage[1] << 8) |
                               s->ramdac_stage[2];

                /* The visible map takes the normal entries (0x11) and the
                 * weave entries (0x10); 0x00/0x01/0x1c are the overlay/pup/
                 * 24-bit maps.  See the note by SGI_GR2_XMAP_PAL_BANK_*. */
                if (s->ramdac_ctl == SGI_GR2_XMAP_PAL_BANK_INSTALLED ||
                    s->ramdac_ctl == SGI_GR2_XMAP_PAL_BANK_ALT) {
                    s->ramdac[s->ramdac_index] = rgb;
                    trace_sgi_gr2_ramdac(s->ramdac_index, rgb);
                }
                s->ramdac_stage_n = 0;
            }
        }
    } else if (offset == SGI_GR2_XMAP_PAL_CTL) {
        s->ramdac_ctl = value & 0xff;
    }
    /* BT457 DAC palette/gamma RAM (SGI_GR2_DAC0_OFF): the colour byte at +4 is
     * written to the current address at +0 and the address auto-increments, so
     * a run after an address write is a ramp load.  Built from the guest, never
     * assumed: the golden streams an identity ramp at the boot DAC probe and
     * the display gamma when Xsgi starts. */
    if (offset >= SGI_GR2_DAC0_OFF &&
        offset < SGI_GR2_DAC0_OFF + SGI_GR2_DAC_NDAC * SGI_GR2_DAC_STRIDE) {
        unsigned k;

        for (k = 0; k < size; k++) {
            uint64_t o = offset + k;
            unsigned rel = o - SGI_GR2_DAC0_OFF;
            unsigned dac = rel / SGI_GR2_DAC_STRIDE;
            unsigned reg = rel % SGI_GR2_DAC_STRIDE;
            uint8_t byte = (value >> (8 * (size - 1 - k))) & 0xff;

            if (reg == SGI_GR2_DAC_ADDR) {
                s->dac_addr[dac] = byte;
            } else if (reg == SGI_GR2_DAC_PALT) {
                uint8_t idx = s->dac_addr[dac];

                s->dac_ramp[dac][idx] = byte;
                s->dac_addr[dac] = (idx + 1) & 0xff;
                s->dac_ramp_set = true;
                trace_sgi_gr2_dac(dac, idx, byte);
            }
        }
        return;
    }
    /* Unpopulated GE units discard writes. */
    if (offset >= SGI_GR2_GE_OFF &&
        offset < SGI_GR2_GE_OFF + SGI_GR2_GE_UNITS * SGI_GR2_GE_STRIDE) {
        unsigned unit = (offset - SGI_GR2_GE_OFF) / SGI_GR2_GE_STRIDE;

        if (unit >= s->ges) {
            return;
        }
    }
    /* The presence magic and the board-version register latch nothing from
     * the bus; ignore writes there. */
    if (offset < SGI_GR2_HQ_MYSTERY + 4 && offset + size > SGI_GR2_HQ_MYSTERY) {
        return;
    }
    /* The HQ2 FIFO occupancy/status register is read-only (the driver only
     * polls it); a stray write must not corrupt the level/error bits. */
    if (offset >= SGI_GR2_HQ_FIFOSTAT &&
        offset < SGI_GR2_HQ_FIFOSTAT + 4) {
        return;
    }
    for (i = 0; i < size; i++) {
        uint8_t byte = (value >> (8 * (size - 1 - i))) & 0xff;

        s->regs[offset + i] = byte;
    }
}

static const MemoryRegionOps sgi_gr2_ops = {
    .read = sgi_gr2_read,
    .write = sgi_gr2_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/*
 * Scanout (P0.4 step a).  A QEMU display surface proves the OUTPUT stage
 * before the RE3 producer: the framebuffer is filled with a known pattern
 * and must appear on the GR2 console, falsifiably, independent of the guest.
 */
static void sgi_gr2_fill_bars(SGIGr2State *s)
{
    static const uint32_t bars[8] = {
        0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
        0xff00ff, 0xff0000, 0x0000ff, 0x000000,
    };
    int x, y;

    if (!s->scanout) {
        return;
    }
    /* Test seam: the bars stand in for a drawn image, so put the bar colours in
     * the first eight palette entries and write those indices — the scanout
     * stage then resolves them like any other pixel. */
    for (x = 0; x < 8; x++) {
        s->ramdac[x] = bars[x];
    }
    for (y = 0; y < SGI_GR2_SCREEN_H; y++) {
        for (x = 0; x < SGI_GR2_SCREEN_W; x++) {
            sgi_gr2_put(s, x, y, x * 8 / SGI_GR2_SCREEN_W);
        }
    }
}

/* VC1 byte address -> the next address after a 16-bit port write. */
static void sgi_gr2_vc1_advance(SGIGr2State *s)
{
    unsigned a = ((((unsigned)s->vc1_addrhi << 8) | s->vc1_addrlo) + 2) & 0xffff;

    s->vc1_addrlo = a & 0xff;
    s->vc1_addrhi = (a >> 8) & 0xff;
}

/* The hardware cursor: VC1 reg 0x20 holds the bitmap address (0x0a00), 0x22/0x24
 * the x/y, 0x26 the mode.  The registers are RASTER coordinates, so the visible
 * position is reg minus the horizontal/vertical backporch (GR2_CURS_*OFF_1280);
 * at reset they read 0, which puts the sprite off-screen, so a fresh boot shows
 * no cursor.  The image is 16x16 at 2 bpp - 0 transparent, 1 black, 2 white,
 * 3 invert - loaded through the sram port by Gr2LoadVC1SRAM. */
static bool sgi_gr2_vc1_cursor(SGIGr2State *s, int *cx, int *cy)
{
    unsigned base = s->vc1_reg[0x20 >> 1]; /* byte addr 0x20 = cursor image base */

    if (base != SGI_GR2_VC1_CURSOR_ADDR) {
        return false;
    }
    *cx = (int)s->vc1_reg[0x22 >> 1] - SGI_GR2_VC1_CURS_XOFF;
    *cy = (int)s->vc1_reg[0x24 >> 1] - SGI_GR2_VC1_CURS_YOFF;
    return true;
}

static unsigned sgi_gr2_vc1_cursor_code(SGIGr2State *s, int x, int y)
{
    unsigned p = y * SGI_GR2_VC1_CURSOR_W + x;
    unsigned base = SGI_GR2_VC1_CURSOR_ADDR >> 1;
    uint16_t w = s->vc1_sram[base + (p >> 3)];
    unsigned byte = (w >> (8 * (1 - ((p >> 2) & 1)))) & 0xff;

    return (byte >> (6 - 2 * (p & 3))) & 3;
}

static void sgi_gr2_update_display(void *opaque)
{
    SGIGr2State *s = opaque;
    DisplaySurface *surface;
    uint32_t *dest;
    int stride, y;

    if (!s->con || !s->scanout) {
        return;
    }
    surface = qemu_console_surface(s->con);
    if (!surface) {
        return;
    }
    dest = (uint32_t *)surface_data(surface);
    stride = surface_stride(surface);
    for (y = 0; y < SGI_GR2_SCREEN_H; y++) {
        uint32_t *row = dest + (y * stride) / 4;
        const uint8_t *src = s->scanout + y * SGI_GR2_SCREEN_W;
        int x;

        /* The palette is applied HERE, at scanout, so a pixel drawn before its
         * entry was programmed still shows the entry's final colour.  A pixel
         * flagged as direct-colour (from an image op) is expanded 3-3-2 instead,
         * so the cube and the CLUT-indexed panel share one screen. */
        for (x = 0; x < SGI_GR2_SCREEN_W; x++) {
            uint8_t idx = src[x];

            if (s->scanout332 && s->scanout332[y * SGI_GR2_SCREEN_W + x]) {
                row[x] = sgi_gr2_re3_332(idx);
            } else {
                /* The CLUT byte is pre-gamma: run each channel through the
                 * guest's BT457 output ramp.  The 332 path is left alone — its
                 * ramp is already the final display value (measured from the
                 * control), so applying gamma again would over-brighten it. */
                uint32_t rgb = s->ramdac[idx];
                uint8_t r = s->dac_ramp[0][(rgb >> 16) & 0xff];
                uint8_t g = s->dac_ramp[1][(rgb >> 8) & 0xff];
                uint8_t b = s->dac_ramp[2][rgb & 0xff];

                row[x] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
            }
        }
    }
    /* The cursor is a hardware sprite: drawn on top here, never in the buffer. */
    {
        int cx, cy;

        if (sgi_gr2_vc1_cursor(s, &cx, &cy)) {
            for (y = 0; y < SGI_GR2_SCREEN_H; y++) {
                uint32_t *row = dest + (y * stride) / 4;
                int py = y - cy;
                int x;

                if (py < 0 || py >= SGI_GR2_VC1_CURSOR_W) {
                    continue;
                }
                for (x = 0; x < SGI_GR2_SCREEN_W; x++) {
                    int px = x - cx;
                    unsigned code;

                    if (px < 0 || px >= SGI_GR2_VC1_CURSOR_W) {
                        continue;
                    }
                    code = sgi_gr2_vc1_cursor_code(s, px, py);
                    if (code == 1) {
                        row[x] = 0x000000;
                    } else if (code == 2) {
                        row[x] = 0xffffff;
                    } else if (code == 3) {
                        row[x] ^= 0xffffff;
                    }
                }
            }
        }
    }
    dpy_gfx_update(s->con, 0, 0, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
}

static const GraphicHwOps sgi_gr2_gfx_ops = {
    .gfx_update = sgi_gr2_update_display,
};

/* Lower the retrace IRQ at the end of the blanking interval. */
static void sgi_gr2_retrace_lower(void *opaque)
{
    SGIGr2State *s = SGI_GR2(opaque);

    s->retrace_active = false;
    qemu_irq_lower(s->irq);
}

/* 60 Hz retrace tick: raise the GIO interrupt for the blanking interval.
 * Only once the driver has started the board (Gr2Start writes the start
 * token), so we never deliver an interrupt before its vector is registered. */
static void sgi_gr2_retrace_tick(void *opaque)
{
    SGIGr2State *s = SGI_GR2(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->present && s->hq_ready && !s->retrace_active) {
        s->retrace_active = true;
        qemu_irq_raise(s->irq);
        timer_mod(s->retrace_lower_timer, now + SGI_GR2_RETRACE_PULSE_NS);
    }
    timer_mod(s->retrace_timer,
              now + NANOSECONDS_PER_SECOND / SGI_GR2_RETRACE_HZ);
}

static void sgi_gr2_reset(DeviceState *dev)
{
    SGIGr2State *s = SGI_GR2(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ucode, 0, sizeof(s->ucode));
    s->gepc = 0;
    s->hq_ready = false;
    s->xmap_ready = false;
    s->retrace_active = false;
    /* Default RAMDAC palette (measured; see the table) and clear the RE3
     * producer latches. */
    memset(s->ramdac, 0, sizeof(s->ramdac));
    s->ramdac_index = 0;
    s->ramdac_ctl = 0;
    s->ramdac_stage_n = 0;
    /* Output ramps start as the identity until the guest programs them. */
    for (i = 0; i < 256; i++) {
        s->dac_ramp[0][i] = i;
        s->dac_ramp[1][i] = i;
        s->dac_ramp[2][i] = i;
    }
    memset(s->dac_addr, 0, sizeof(s->dac_addr));
    s->dac_ramp_set = false;
    s->re3_colour = 0;
    s->re3_colour_valid = false;
    s->re3_solid_seen = false;
    s->re3_spans_seen = false;
    s->re3_line_seen = false;
    s->re3_spanstip_seen = false;
    s->re3_poly_seen = false;
    s->re3_mono_seen = false;
    s->re3_image_seen = false;
    s->re3_nimg = 0;
    s->re3_npens = 0;
    s->re3_label_y = 0;
    s->re3_label_valid = false;
    s->re3_pair_seen = false;
    s->re3_stipple_valid = false;
    s->re3_stipple = 0;
    s->re3_fg = 0;
    s->re3_fg_valid = false;
    s->re3_rop = 0;
    s->re3_rop_valid = false;
    s->re3_data_n = 0;
    s->re3_data_overflow = false;
    s->last_puc = 0;
    s->prev_puc = 0;
    s->last_puc_valid = false;
    if (s->irq) {
        qemu_irq_lower(s->irq);
    }

    /* HQ2 presence magic (32-bit BE) read by Gr2Probe. */
    s->regs[SGI_GR2_HQ_MYSTERY + 0] = (SGI_GR2_HQ_MAGIC >> 24) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 1] = (SGI_GR2_HQ_MAGIC >> 16) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 2] = (SGI_GR2_HQ_MAGIC >> 8) & 0xff;
    s->regs[SGI_GR2_HQ_MYSTERY + 3] = SGI_GR2_HQ_MAGIC & 0xff;

    /* Board version / config bytes decoded by Gr2Probe: one byte per 32-bit
     * slot at 0x6c000/4/8/c, in the addressed byte lane (the kernel probe
     * byte-reads them).  Per-instance so a variant can report its identity.
     * NOTE: storing them in the low lane instead was tried (to suit a 32-bit
     * ARCS read) and did NOT change the reported name, so it was reverted. */
    s->regs[SGI_GR2_BDVERS_OFF + 0] = s->bdvers0;
    s->regs[SGI_GR2_BDVERS_OFF + 4] = s->bdvers1;
    s->regs[SGI_GR2_BDVERS_OFF + 8] = s->bdvers2;
    s->regs[SGI_GR2_BDVERS_OFF + 12] = s->bdvers3;

    /* HQ2 revision register (read >> 16 into gr2_info.HQ2Rev). */
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 0] = 0x00;
    s->regs[SGI_GR2_HQ_OFF + 0x6c + 1] = 0x01;

    /* P0.4 step (a): the synthetic scanout pattern, if the test asked for it.
     * The scanout region exists only when present (allocated in realize). */
    if (s->scanout && s->scanout_bars) {
        sgi_gr2_fill_bars(s);
    }
}

/* The FIFO has gone quiet: run the glyph op still sitting in the buffer.  Glyph
 * ops are the ones the DDX leaves unterminated - it flushes each by starting the
 * next glyph's 331 and writes no 490 - so the last glyph of a run (the 't' of
 * "guest") had no successor to flush it and was dropped.  Every other shape
 * carries its 490; flushing those on idle would draw ops the DDX never committed
 * (the root stipple op has no 490, and the pre-registered no-draw negative
 * expects it absent), so only a pending glyph op is run here. */
static void sgi_gr2_fifo_flush_cb(void *opaque)
{
    SGIGr2State *s = opaque;

    if (s->re3_npens > 0) {
        sgi_gr2_re3_flush_fill(s);
        sgi_gr2_re3_reset_subop(s);
    }
}

static void sgi_gr2_realize(DeviceState *dev, Error **errp)
{
    SGIGr2State *s = SGI_GR2(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_gr2_ops, s,
                          TYPE_SGI_GR2, SGI_GR2_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    /* GIO interrupt output (retrace → HPC3 "gio-retrace").  Always initialised;
     * the machine only connects it when the board is present. */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->retrace_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    sgi_gr2_retrace_tick, s);
    s->retrace_lower_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          sgi_gr2_retrace_lower, s);
    s->fifo_flush_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                       sgi_gr2_fifo_flush_cb, s);
    timer_mod(s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / SGI_GR2_RETRACE_HZ);

    /* Scanout (P0.4 step a) only when the board is present, so plain
     * `-M indy` gains no extra console. */
    if (s->present) {
        s->scanout = g_new0(uint8_t,
                            (size_t)SGI_GR2_SCREEN_W * SGI_GR2_SCREEN_H);
        s->scanout332 = g_new0(uint8_t,
                               (size_t)SGI_GR2_SCREEN_W * SGI_GR2_SCREEN_H);
        s->con = graphic_console_init(dev, 0, &sgi_gr2_gfx_ops, s);
        qemu_console_resize(s->con, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
    }
}

static const Property sgi_gr2_properties[] = {
    DEFINE_PROP_BOOL("present", SGIGr2State, present, false),
    DEFINE_PROP_BOOL("scanout-bars", SGIGr2State, scanout_bars, false),
    DEFINE_PROP_BOOL("poly-stroke", SGIGr2State, poly_stroke, true),
    DEFINE_PROP_UINT8("ges", SGIGr2State, ges, 2),
    DEFINE_PROP_UINT8("bitplanes", SGIGr2State, bitplanes, 24),
    DEFINE_PROP_BOOL("zbuffer", SGIGr2State, zbuffer, true),
    /* Per-variant board identity.  Defaults = XZ (2 GE, 24-bit + Z).  XS-24
     * sets bdvers1=0x10 (24-bit, no Z); XS sets bdvers1=0x00 (8-bit). */
    DEFINE_PROP_UINT8("bdvers0", SGIGr2State, bdvers0, SGI_GR2_BDVERS0),
    DEFINE_PROP_UINT8("bdvers1", SGIGr2State, bdvers1, SGI_GR2_BDVERS1),
    DEFINE_PROP_UINT8("bdvers2", SGIGr2State, bdvers2, SGI_GR2_BDVERS2),
    DEFINE_PROP_UINT8("bdvers3", SGIGr2State, bdvers3, SGI_GR2_BDVERS3),
};

static void sgi_gr2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_gr2_realize;
    device_class_set_legacy_reset(dc, sgi_gr2_reset);
    device_class_set_props(dc, sgi_gr2_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo sgi_gr2_type_info = {
    .name = TYPE_SGI_GR2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGr2State),
    .class_init = sgi_gr2_class_init,
};

static void sgi_gr2_register_types(void)
{
    type_register_static(&sgi_gr2_type_info);
}

type_init(sgi_gr2_register_types)
