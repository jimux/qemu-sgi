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
#include <math.h>

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

/* expOpStippledFillRects (token 331 == the stipple fill op) draws the WxH 8bpp
 * PATTERN bitmap streamed in PUC_DATA into the rectangles that follow it.  The
 * DDX stores each rectangle as SIX words - the stipple phase, then the
 * coordinate, per axis, then the far corner:
 *     [x1 % W, x1, y1 % H, y1, x2, y2]
 * so the rectangle is (data[n-5], data[n-3], data[n-2], data[n-1]).  The
 * four-word walkback in rect_groups cannot see this: it drops the two phase
 * words and reads (data[n-4], data[n-3], data[n-2], data[n-1]), which turns the
 * Console icon's (210,16)-(295,83) into (16,16)-(295,83) and smears the pattern
 * left by 194px.  The phase words are a self-check, not magic: for a genuine
 * rect, x1 % W == data[n-6] and y1 % H == data[n-4] (210%85=40, 16%67=16), so
 * an op that is not this shape is left to the rect-list path untouched. */
static bool sgi_gr2_re3_pattern_fill(SGIGr2State *s)
{
    unsigned n = s->re3_data_n, nwords, x, y, x1, y1, x2, y2;
    uint32_t w, h, c0, c1;

    if (n < 13 || s->re3_data[0] != 0xff || s->re3_data[1] != 3 ||
        s->re3_data[5] != 2 || s->re3_data[6] != 0xd022) {
        return false;
    }
    w = s->re3_data[3];
    h = s->re3_data[4];
    if (w == 0 || h == 0 || w > SGI_GR2_SCREEN_W || h > SGI_GR2_SCREEN_H) {
        return false;
    }
    nwords = (w * h + 3) / 4;
    if (7 + nwords + 6 > n) {
        return false;
    }
    x1 = s->re3_data[n - 5];
    y1 = s->re3_data[n - 3];
    x2 = s->re3_data[n - 2];
    y2 = s->re3_data[n - 1];
    if (x2 <= x1 || y2 <= y1 ||
        s->re3_data[n - 6] != x1 % w || s->re3_data[n - 4] != y1 % h) {
        return false;
    }
    if (x1 >= SGI_GR2_SCREEN_W || y1 >= SGI_GR2_SCREEN_H) {
        return false;
    }
    if (x2 > SGI_GR2_SCREEN_W) {
        x2 = SGI_GR2_SCREEN_W;
    }
    if (y2 > SGI_GR2_SCREEN_H) {
        y2 = SGI_GR2_SCREEN_H;
    }
    if (!s->scanout) {
        return true;
    }
    /* Same two-colour form as the tile: data[2] is the base colour, data[1]
     * indexes the second, and any other pattern byte is a direct 8bpp index. */
    c0 = s->re3_data[2] << 3;
    c1 = c0 | (s->re3_data[1] << 1);
    for (y = y1; y < y2; y++) {
        for (x = x1; x < x2; x++) {
            unsigned p = (y - y1) * w + (x - x1);
            uint32_t word = s->re3_data[7 + p / 4];
            uint8_t v = (word >> (8 * (3 - (p % 4)))) & 0xff;
            uint8_t idx;

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
    trace_sgi_gr2_re3_rect((uint8_t)c1, 0, x1, y1, x2, y2);
    sgi_gr2_update_display(s);
    return true;
}

/* Number of trailing rectangle groups in the current sub-op's PUC_DATA.  The
 * DDX's expDrawSolidRects stores the geometry as groups of four (x1,y1,x2,y2)
 * after a short non-rectangle prefix ("0xff 0x3 0x0", and for some ops the clip
 * bounds 0x4ff 0x0 0x3ff before it).  Rather than assume the
 * prefix length, walk back from the end while each group is a sane rectangle:
 * the longest valid suffix is the list, and the prefix is left alone.  This is
 * the layout read off the DDX store sequences (note 31), not a guessed tail.
 * expOpStippledFillRects does NOT use this shape - its rects carry interleaved
 * stipple phases and are handled by sgi_gr2_re3_pattern_fill. */
static unsigned sgi_gr2_re3_rect_groups(SGIGr2State *s, bool stippled)
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
        /* A stippled fill carries a stipple pattern/mask header ahead of its
         * rect list.  That header can end in words that, together with the
         * run of zeros before them, look like one more rectangle starting at
         * the screen origin - the backward walk reads it as a spurious
         * (0,0,40,210) rect.  The DDX keeps the real rect count in a register
         * and writes only four words per rect, with no count in the FIFO
         * payload (expStippledFillRects), so the count cannot be recovered
         * from the stream; a further origin-anchored group after the first is
         * a parse artifact, not geometry.  Stop there. */
        if (stippled && groups > 0 && x1 == 0 && y1 == 0) {
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
    unsigned groups = sgi_gr2_re3_rect_groups(s, stippled), g;
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

/* Draw ONE IP20 glyph.  Its pen x is X; its y, w, h and bitmap begin at
 * re3_data[off].  Token 0x40510 is per-glyph — one 490-terminated region can
 * carry several pens (the shell redraws the prompt as a run of them), and only
 * keeping the last pen lost the earlier glyphs of such a run, so this is called
 * both as each new pen arrives and once more for the final pen at the region
 * terminator. */
static void sgi_gr2_re3_draw_mono_pen(SGIGr2State *s, unsigned X, unsigned off)
{
    uint8_t col = s->re3_monocol_valid ? (uint8_t)s->re3_monocol
                                       : s->re3_colour;
    int Y, W, H, k, r, run;

    if (off + 3 > s->re3_data_n) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
    Y = (int)s->re3_data[off];
    W = (int)s->re3_data[off + 1];
    H = (int)s->re3_data[off + 2];
    if (W <= 0 || W > 12) {
        W = 12;
    }
    if (H <= 0 || H > 64) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
    trace_sgi_gr2_re3_mono((int)X, Y, W, H, col);
    for (k = 0; k < H; k++) {
        unsigned wi = off + 3 + (unsigned)(k / 2);
        uint32_t w = (wi < s->re3_data_n) ? s->re3_data[wi] : 0;
        uint32_t rowbits;

        /* Each 32-bit word carries TWO rows of 12 bits: bits 31..20 first,
         * then bits 15..4.  Taking one row per word read only the low half and
         * halved the glyph's height (the "squashed" render). */
        rowbits = ((k & 1) ? (w >> 4) : (w >> 20)) & 0xfff;

        /* Coalesce the row's set bits into runs and paint each through the
         * rect fill helper, so clipping and the scanout update stay in one
         * place rather than poking s->scanout here. */
        for (r = 0; r < W; ) {
            if (!((rowbits >> (11 - r)) & 1)) {
                r++;
                continue;
            }
            run = 1;
            while (r + run < W && ((rowbits >> (11 - (r + run))) & 1)) {
                run++;
            }
            sgi_gr2_re3_fill(s, col, (int)X + r, Y + k, run, 1);
            r += run;
        }
    }
}

static void sgi_gr2_re3_draw_text(SGIGr2State *s)
{
    unsigned p;
    bool any = false;

    if (s->re3_monox_valid) {
        sgi_gr2_re3_draw_mono_pen(s, s->re3_monox, s->re3_mono_off);
        return;
    }


    for (p = 0; p < s->re3_npens; p++) {
        unsigned off = s->re3_pen_off[p];
        uint32_t pen = s->re3_pen_val[p];
        uint32_t f0, f1, h, k, nwords;
        unsigned gw;

        if (off + 3 > s->re3_data_n) {
            continue;
        }
        f0 = s->re3_data[off];       /* destination y */
        f1 = s->re3_data[off + 1];   /* glyph width in pixels */
        h = s->re3_data[off + 2];
        nwords = (h + 1) / 2;
        if (f0 >= SGI_GR2_SCREEN_H || pen >= SGI_GR2_SCREEN_W ||
            h < 1 || h > 64 ||
            off + 3 + nwords > s->re3_data_n) {
            continue;
        }
        gw = (f1 >= 1 && f1 <= 16) ? f1 : 8;
        for (k = 0; k < nwords; k++) {
            uint32_t w = s->re3_data[off + 3 + k];
            int r0 = (int)f0 + 2 * (int)k;
            int b;

            for (b = 0; b < (int)gw; b++) {
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
        /* Stippled op.  Two shapes share token 318: a rect list (the root
         * backdrop is one full-screen rect, and a stippled sub-op with its own
         * small rects must paint only those), and expOpStippledFillRects, whose
         * payload is an 8bpp WxH PATTERN bitmap plus its phase-encoded rects -
         * this is how IP20 draws the Console icon.  Painting the pattern op as
         * rect stipples smears the bitmap across the screen (the icon showed as
         * vertical stripes spilling 194px left), so try the pattern renderer
         * first; it only claims the op on a self-checked shape, and anything
         * else falls through to the rect list. */
        if (sgi_gr2_re3_pattern_fill(s)) {
            return;
        }
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
        /* No known marker: an op we do not model (3D geometry, for instance).
         * Log its opening words so the token/payload family can be named. */
        if (s->re3_data_n >= 2) {
            trace_sgi_gr2_re3_unknown(s->re3_data[0], s->re3_data[1],
                                      s->re3_data_n);
        }
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
    s->re3_monox_valid = false;
    s->re3_monocol_valid = false;
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

/* The guest PC of the instruction performing the current MMIO access.  cpu->mem_io_pc
 * is the HOST return address (see include/hw/core/cpu.h), useless for attributing a
 * poll to guest code; cc->get_pc() returns the guest's own PC instead. */
static uint32_t sgi_gr2_guest_pc(void)
{
    if (current_cpu && current_cpu->cc->get_pc) {
        return (uint32_t)current_cpu->cc->get_pc(current_cpu);
    }
    return 0;
}

/* ------------------------------------------------------------------------- *
 * GE7 3D: transform + Z-raster for the immediate-mode GL token stream.
 *
 * The offline decoder (tmp/indy-video-xz/b2/tools/gr2_ge7_decode.py) is the
 * oracle for this: it established that object-space vertices are sent on token
 * 2659, three floats at a time; that a polygon is a triangle fan delimited by
 * bgnpolygon/endpolygon (420/1454 .. 65/1125); and that the transform is
 * projection * modelview applied as column-major M*v (libgl emits the
 * transpose of IRIS GL's internal row-major matrices).  The offline numbers
 * agree that MV is a rigid rotation (det +1), that every vertex has w>0, and
 * that the whole bust lands inside the canonical volume.  The viewport is the
 * one thing the FIFO carries that the offline pass did not use: gl_g_viewport
 * puts x on token 60 and (y,w,h) on the next three PUC_DATA words.
 * ------------------------------------------------------------------------- */

static inline float sgi_gr2_u2f(uint32_t v)
{
    union { uint32_t u; float f; } x;

    x.u = v;
    return x.f;
}

/* gl_clear: fill a drawable rectangle with the palette's black and reset its
 * depth, so the window shows the GL client's background instead of the desktop
 * underneath.  powerflip's window is black, which is the palette entry nearest
 * luminance zero. */
static void sgi_gr2_ge7_clear_rect(SGIGr2State *s, int vx, int vy, int vw,
                                   int vh)
{
    int x, y, i;
    uint8_t idx = 0;
    int bestd = 1 << 30;

    if (!s->scanout) {
        return;
    }
    for (i = 0; i < 256; i++) {
        uint32_t rgb = s->ramdac[i];
        int lum = (((rgb >> 16) & 0xff) * 77 + ((rgb >> 8) & 0xff) * 150 +
                   (rgb & 0xff) * 29) >> 8;

        if (lum < bestd) {
            bestd = lum;
            idx = (uint8_t)i;
        }
    }
    for (y = MAX(vy, 0); y < MIN(vy + vh, SGI_GR2_SCREEN_H); y++) {
        for (x = MAX(vx, 0); x < MIN(vx + vw, SGI_GR2_SCREEN_W); x++) {
            sgi_gr2_put(s, x, y, idx);
            if (s->ge_zbuf) {
                s->ge_zbuf[(size_t)y * SGI_GR2_SCREEN_W + x] = 0x7f7f7f7f;
            }
        }
    }
    s->ge_3d_seen = true;
}

/* True when the MSINGLE combined matrix (token 54) is the current transform,
 * i.e. it was written more recently than the separate modelview/projection.
 * powerflip sets 54 only at init and drives 55/56, so it is unaffected; ideas
 * draws its lines/meshes in MSINGLE and relies on 54. */
static bool sgi_gr2_ge7_single_active(const SGIGr2State *s)
{
    return s->ge_single_valid && s->ge_seq54 > s->ge_seq55 &&
           s->ge_seq54 > s->ge_seq56;
}

/* Transform an object-space point by projection*modelview.  Returns false if
 * the point is behind the eye (w <= 0) so it is skipped rather than projected
 * through the eye. */
static bool sgi_gr2_ge7_xform(const SGIGr2State *s, const float p[3],
                              float *sx, float *sy, float *sz)
{
    static const float ident[16] = { 1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1 };
    const float *mv, *proj;
    float cam[4], clip[4];
    int r, c;

    if (sgi_gr2_ge7_single_active(s)) {
        mv = s->ge_single;
        proj = ident;
    } else {
        if (!s->ge_mv_valid || !s->ge_proj_valid) {
            return false;
        }
        mv = s->ge_mv;
        proj = s->ge_proj;
    }
    for (r = 0; r < 4; r++) {
        cam[r] = mv[3 * 4 + r]; /* v[3] = 1 */
        for (c = 0; c < 3; c++) {
            cam[r] += mv[c * 4 + r] * p[c];
        }
    }
    for (r = 0; r < 4; r++) {
        clip[r] = 0.0f;
        for (c = 0; c < 4; c++) {
            clip[r] += proj[c * 4 + r] * cam[c];
        }
    }
    if (clip[3] <= 0.0f) {
        return false;
    }
    *sx = clip[0] / clip[3];
    *sy = clip[1] / clip[3];
    *sz = clip[2] / clip[3];
    return true;
}

/* Rasterise the buffered polygon (a triangle fan) into `scanout`, Z-buffered.
 * The viewport maps NDC to pixels; if the guest has not yet sent one, fall
 * back to the whole screen. */
/* The GE's specular is a table lookup, not an exponent computed on the fly:
 * __glExpCreateSpecLUT builds 128 entries pow(t, shininess) and the guest
 * uploads that table to token 116.  Index the guest's own curve so the
 * shininess need not be guessed; fall back to the held exponent only until
 * the table has arrived. */
static float sgi_gr2_ge7_spec_term(const SGIGr2State *s, float ndh,
                                   float shininess)
{
    int i;

    if (s->ge_spec_lut_n < 128) {
        return powf(ndh, shininess);
    }
    i = (int)(ndh * 127.0f + 0.5f);
    i = MIN(MAX(i, 0), 127);
    return s->ge_spec_lut[i];
}

/* Evaluate the guest's Phong model for one normal and return the shaded RGB.
 * Shared by the triangle and line rasterisers.
 *
 * The guest may bind several lights (ideas binds three and animates them), so
 * every valid light is summed, each with its own half-vector against the eye
 * at (0,0,1).  A guest with one light fills one slot and the sum is the old
 * single-light result; if no light has been seen the eye-space fallback
 * (0,0,1) is used so an unlit stream still draws. */
static void sgi_gr2_ge7_shade(const SGIGr2State *s, const float n[3],
                              float shininess, float col[3])
{
    float nn[3], ndl, ndh, l[3], h[3], ll, hl;
    float nl = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    int c, li;
    bool any = false;

    if (nl > 1e-9f) {
        nn[0] = n[0] / nl;
        nn[1] = n[1] / nl;
        nn[2] = n[2] / nl;
    } else {
        nn[0] = 0.0f;
        nn[1] = 0.0f;
        nn[2] = 1.0f;
    }
    /* Two-sided, as for the triangles: a normal facing away is flipped. */
    if (nn[2] < 0.0f) {
        nn[0] = -nn[0];
        nn[1] = -nn[1];
        nn[2] = -nn[2];
    }
    for (c = 0; c < 3; c++) {
        col[c] = s->ge_emission[c] +
                 s->ge_ambient[c] * s->ge_ambient_sum[c];
    }
    for (li = 0; li < SGI_GR2_GE7_MAX_LIGHTS; li++) {
        if (!s->ge_light_valid[li]) {
            continue;
        }
        ll = s->ge_lights[li][0] * s->ge_lights[li][0] +
             s->ge_lights[li][1] * s->ge_lights[li][1] +
             s->ge_lights[li][2] * s->ge_lights[li][2];
        if (ll <= 1e-6f) {
            continue; /* a light at the origin is off */
        }
        ll = sqrtf(ll);
        l[0] = s->ge_lights[li][0] / ll;
        l[1] = s->ge_lights[li][1] / ll;
        l[2] = s->ge_lights[li][2] / ll;
        h[0] = l[0]; h[1] = l[1]; h[2] = l[2] + 1.0f;
        hl = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
        if (hl > 1e-6f) {
            h[0] /= hl; h[1] /= hl; h[2] /= hl;
        }
        ndl = MAX(nn[0] * l[0] + nn[1] * l[1] + nn[2] * l[2], 0.0f);
        ndh = MAX(nn[0] * h[0] + nn[1] * h[1] + nn[2] * h[2], 0.0f);
        for (c = 0; c < 3; c++) {
            col[c] += s->ge_light_color[li][c] * (s->ge_diffuse[c] * ndl +
                                                  s->ge_specular[c] *
                                                  sgi_gr2_ge7_spec_term(s, ndh,
                                                              shininess));
        }
        any = true;
    }
    if (!any) {
        /* No light seen yet: a headlight at the eye so something draws. */
        ndl = MAX(nn[2], 0.0f);
        for (c = 0; c < 3; c++) {
            col[c] += s->ge_lcolor[c] * (s->ge_diffuse[c] * ndl +
                                         s->ge_specular[c] *
                                         sgi_gr2_ge7_spec_term(s, ndl,
                                                             shininess));
        }
    }
    for (c = 0; c < 3; c++) {
        col[c] = MIN(MAX(col[c], 0.0f), 1.0f);
    }
}

/* 4x4 ordered-dither matrix, read from __glDitherTable in the guest's own
 * libGLcore.so (the IP22GR2NG1 library the ideas/atlantis/powerflip binaries
 * link: /usr/gfx/arch/IP22GR2NG1/libGLcore.so).  Its first 16 bytes are this
 * matrix; the 48 that follow are per-channel threshold tables.  It is a
 * rotation/transpose of the Newport matrix, not the same table - which is why
 * the Newport one must not be reused.  Row-major as stored. */
static const uint8_t sgi_gr2_ge7_bayer[4][4] = {
    {  0,  8,  2, 10 },
    { 12,  4, 14,  6 },
    {  3, 11,  1,  9 },
    { 15,  7, 13,  5 },
};

/* Pick the ramp level for a linear 0..255 channel value.  `thresh` >= 0 is a
 * Bayer threshold: when the value lies between two ramp levels the matrix
 * decides which one this pixel takes, so a gradient renders as a stipple of
 * the two adjacent levels instead of a hard band.  `thresh` < 0 means plain
 * nearest-level (dithering off).  The ramp levels are not evenly spaced
 * (0,81,122,155,184,209,233,255), so the fraction is taken against the actual
 * gap, not a uniform step. */
static uint8_t sgi_gr2_ge7_lvl(const uint8_t *ramp, int n, int t, int thresh)
{
    int i, lo, hi, span, frac;

    if (thresh < 0) {
        int best = 0, bestd = 1 << 30;

        for (i = 0; i < n; i++) {
            int d = (int)ramp[i] - t;

            if (d < 0) {
                d = -d;
            }
            if (d < bestd) {
                bestd = d;
                best = i;
            }
        }
        return (uint8_t)best;
    }
    for (i = 0; i < n; i++) {
        if (ramp[i] >= t) {
            break;
        }
    }
    if (i <= 0) {
        return 0;
    }
    if (i >= n) {
        return (uint8_t)(n - 1);
    }
    lo = i - 1;
    hi = i;
    span = (int)ramp[hi] - (int)ramp[lo];
    frac = span > 0 ? ((t - (int)ramp[lo]) * 16) / span : 0;
    return (uint8_t)(frac > thresh ? hi : lo);
}

/* Quantise a shaded RGB to the 8-bit 3-3-2 direct-colour byte the GL window
 * stores.  The screen is 8-bit PseudoColor (root 1280x1024x8, visual 0x25) and
 * an RGBmode window does not go through the CLUT: it is the "24-bit" visual,
 * an 8-bit 3-3-2 RGB triple the scanout expands through the direct-colour ramp
 * (see sgi_gr2_re3_332).  This is the same representation the image path and
 * the EZsetup cube use, so polygons and lines land in the same colour space.
 * x/y drive the ordered dither. */
static uint8_t sgi_gr2_ge7_332(const SGIGr2State *s, const float col[3],
                               int x, int y)
{
    int r = (int)(MIN(MAX(col[0], 0.0f), 1.0f) * 255.0f + 0.5f);
    int g = (int)(MIN(MAX(col[1], 0.0f), 1.0f) * 255.0f + 0.5f);
    int b = (int)(MIN(MAX(col[2], 0.0f), 1.0f) * 255.0f + 0.5f);
    int thresh = s->ge_dither ? sgi_gr2_ge7_bayer[x & 3][y & 3] : -1;

    return (uint8_t)((sgi_gr2_ge7_lvl(sgi_gr2_ramp_rg, 8, r, thresh) << 5) |
                     (sgi_gr2_ge7_lvl(sgi_gr2_ramp_b, 4, b, thresh) << 3) |
                     sgi_gr2_ge7_lvl(sgi_gr2_ramp_rg, 8, g, thresh));
}

static void sgi_gr2_ge7_draw(SGIGr2State *s)
{
    float sx[SGI_GR2_GE7_MAX_VERTS], sy[SGI_GR2_GE7_MAX_VERTS];
    float sz[SGI_GR2_GE7_MAX_VERTS];
    float vcol[SGI_GR2_GE7_MAX_VERTS][3]; /* per-vertex shaded RGB           */
    int vx, vy, vw, vh;
    unsigned i;
    const float shininess = 8.0f;

    if (s->ge_poly_n < 3 || !s->scanout || !s->ge_zbuf) {
        return;
    }
    if (s->vp_valid && s->vp_w > 0 && s->vp_h > 0) {
        vx = s->vp_x;
        vy = s->vp_y;
        vw = s->vp_w;
        vh = s->vp_h;
    } else {
        vx = 0;
        vy = 0;
        vw = SGI_GR2_SCREEN_W;
        vh = SGI_GR2_SCREEN_H;
    }
    /* The client writes window-relative coordinates; the window's origin on
     * screen is the X server's state (the client never emits winposition into
     * the FIFO and the GE has no window-origin register - see note 78).  With
     * no origin source the drawable sits at the screen origin. */
    vx += s->ge_win_x;
    vy += s->ge_win_y;
    if (s->ge_need_clear && !sgi_gr2_ge7_single_active(s)) {
        /* Only the 3D modelview/projection path clears per frame; a MSINGLE
         * app (ideas) writes 55 per object, so clearing there would wipe its
         * line art mid-frame. */
        sgi_gr2_ge7_clear_rect(s, vx, vy, vw, vh);
        s->ge_need_clear = false;
    }
    for (i = 0; i < s->ge_poly_n; i++) {
        float cx, cy, cz;
        float px, py;
        int c;

        if (!sgi_gr2_ge7_xform(s, s->ge_poly[i], &cx, &cy, &cz)) {
            return; /* part of the polygon is behind the eye: skip it whole */
        }
        px = vx + (cx + 1.0f) * 0.5f * vw;
        py = vy + (1.0f - (cy + 1.0f) * 0.5f) * vh; /* viewport y from the top */
        sx[i] = px;
        sy[i] = py;
        sz[i] = cz;
        /* Rotate this vertex's own normal into eye space (the object-space
         * normal was captured with the vertex, not shared per polygon), then
         * shade it with the guest's material and light. */
        {
            float een[3];

            for (c = 0; c < 3; c++) {
                een[c] = s->ge_mv[c * 4 + 0] * s->ge_vnormal[i][0] +
                         s->ge_mv[c * 4 + 1] * s->ge_vnormal[i][1] +
                         s->ge_mv[c * 4 + 2] * s->ge_vnormal[i][2];
            }
            sgi_gr2_ge7_shade(s, een, shininess, vcol[i]);
        }
    }
    for (i = 1; i + 1 < s->ge_poly_n; i++) {
        unsigned ia = s->ge_strip ? i - 1 : 0; /* fan apex, or strip prev-2 */
        float ax = sx[ia], ay = sy[ia], az = sz[ia];
        float bx = sx[i], by = sy[i], bz = sz[i];
        float cx = sx[i + 1], cy = sy[i + 1], cz = sz[i + 1];
        const float *c0 = vcol[ia], *c1 = vcol[i], *c2 = vcol[i + 1];
        int64_t fx0, fy0, fx1, fy1, fx2, fy2;
        int64_t area, e0, e1, e2;
        float w0, w1, w2, inv;
        int minx, maxx, miny, maxy, x, y;

        /* Fixed-point edge functions with four sub-pixel bits.  Coordinates
         * become multiples of 1/16 and pixel centres sit at (x*16+8).  Done in
         * integers so a shared edge is bit-identical for the two triangles that
         * meet there and the top-left rule can hand it to exactly one of them,
         * which no epsilon in floating point can guarantee. */
        fx0 = (int64_t)lroundf(ax * 16.0f);
        fy0 = (int64_t)lroundf(ay * 16.0f);
        fx1 = (int64_t)lroundf(bx * 16.0f);
        fy1 = (int64_t)lroundf(by * 16.0f);
        fx2 = (int64_t)lroundf(cx * 16.0f);
        fy2 = (int64_t)lroundf(cy * 16.0f);
        area = (fx1 - fx0) * (fy2 - fy0) - (fy1 - fy0) * (fx2 - fx0);
        if (area == 0) {
            continue;
        }
        /* Back-face culling (tokens 27/28).  This edge-function area is in
         * window space with y downward, so a GL front face (CCW in the y-up
         * window) comes out negative and a back face positive; keep whichever
         * winding the front-face polarity says is the front one. */
        if (s->ge_cull_ccw || s->ge_cull_cw) {
            bool front = s->ge_cull_ccw ? (area < 0) : (area > 0);

            if (!front) {
                continue;
            }
        }
        if (area < 0) {
            /* Normalise winding so all three edge functions are non-negative
             * inside the triangle.  Swap the B and C vertices and everything
             * interpolated with them. */
            float tf;

            tf = bx; bx = cx; cx = tf;
            tf = by; by = cy; cy = tf;
            tf = bz; bz = cz; cz = tf;
            {
                const float *tc = c1; c1 = c2; c2 = tc;
            }
            fx1 = (int64_t)lroundf(bx * 16.0f);
            fy1 = (int64_t)lroundf(by * 16.0f);
            fx2 = (int64_t)lroundf(cx * 16.0f);
            fy2 = (int64_t)lroundf(cy * 16.0f);
            area = -area;
        }
        inv = 1.0f / (float)area;
        /* Pixel-centre bounding box, one pixel of slack for the fixed-point
         * rounding. */
        minx = (int)(MIN(fx0, MIN(fx1, fx2)) >> 4) - 1;
        maxx = (int)(MAX(fx0, MAX(fx1, fx2)) >> 4) + 1;
        miny = (int)(MIN(fy0, MIN(fy1, fy2)) >> 4) - 1;
        maxy = (int)(MAX(fy0, MAX(fy1, fy2)) >> 4) + 1;
        /* Clip to the drawable: the GL viewport (plus its window origin) is the
         * only region a GL client may paint.  Without this the transformed
         * vertices spill over the window frame and neighbouring windows. */
        minx = MAX(minx, vx);
        miny = MAX(miny, vy);
        maxx = MIN(maxx, vx + vw - 1);
        maxy = MIN(maxy, vy + vh - 1);
        minx = MAX(minx, 0);
        miny = MAX(miny, 0);
        maxx = MIN(maxx, SGI_GR2_SCREEN_W - 1);
        maxy = MIN(maxy, SGI_GR2_SCREEN_H - 1);
        if (minx > maxx || miny > maxy) {
            continue;
        }
        /* Top-left tie-break for the edges: a pixel whose centre lies exactly
         * on a shared edge is claimed by the triangle for which that edge is a
         * top or left edge, so exactly one of the two writes it. */
        for (y = miny; y <= maxy; y++) {
            for (x = minx; x <= maxx; x++) {
                int64_t px = (int64_t)x * 16 + 8;
                int64_t py = (int64_t)y * 16 + 8;
                float col[3], z;
                size_t o;
                int c;

                /* e0 for edge A->B, e1 for B->C, e2 for C->A. */
                e0 = (fx1 - fx0) * (py - fy0) - (fy1 - fy0) * (px - fx0);
                e1 = (fx2 - fx1) * (py - fy1) - (fy2 - fy1) * (px - fx1);
                e2 = (fx0 - fx2) * (py - fy2) - (fy0 - fy2) * (px - fx2);
                if (e0 < 0 || e1 < 0 || e2 < 0) {
                    continue;
                }
                if (e0 == 0 &&
                    !((fy1 == fy0 && fx1 > fx0) || fy1 > fy0)) {
                    continue;
                }
                if (e1 == 0 &&
                    !((fy2 == fy1 && fx2 > fx1) || fy2 > fy1)) {
                    continue;
                }
                if (e2 == 0 &&
                    !((fy0 == fy2 && fx0 > fx2) || fy0 > fy2)) {
                    continue;
                }
                /* Barycentrics from the integer edge functions: e1 is weight
                 * of A, e2 of B, e0 of C. */
                w0 = (float)e1 * inv;
                w1 = (float)e2 * inv;
                w2 = (float)e0 * inv;
                for (c = 0; c < 3; c++) {
                    col[c] = w0 * c0[c] + w1 * c1[c] + w2 * c2[c];
                }
                z = w0 * az + w1 * bz + w2 * cz;
                o = (size_t)y * SGI_GR2_SCREEN_W + x;
                if (z < s->ge_zbuf[o]) {
                    s->ge_zbuf[o] = z;
                    sgi_gr2_put332(s, x, y, sgi_gr2_ge7_332(s, col, x, y));
                }
            }
        }
        s->ge_polys++;
    }
    s->ge_3d_seen = true;
}

/* Draw the buffered line vertices (BGNLINE 380/1110 .. ENDLINE 87).  Each
 * vertex is a gl_v2f pair; it is promoted to 3D with z=0 and carried through
 * the same projection*modelview and viewport as the triangles, then the
 * segment is plotted.  Intensity comes from the same material/light state. */
static void sgi_gr2_ge7_draw_lines(SGIGr2State *s)
{
    int vx, vy, vw, vh;
    float nx[3];
    float px[SGI_GR2_GE7_MAX_LVERTS], py[SGI_GR2_GE7_MAX_LVERTS];
    unsigned i;
    unsigned ink_px = 0; /* pixels this run paints (trace diagnostic) */
    const float shininess = 8.0f;
    float col[3];

    if (s->ge_line_n < 2 || !s->scanout || !s->ge_zbuf) {
        return;
    }
    if (s->vp_valid && s->vp_w > 0 && s->vp_h > 0) {
        vx = s->vp_x; vy = s->vp_y; vw = s->vp_w; vh = s->vp_h;
    } else {
        vx = 0; vy = 0; vw = SGI_GR2_SCREEN_W; vh = SGI_GR2_SCREEN_H;
    }
    vx += s->ge_win_x;
    vy += s->ge_win_y;
    /* A 2D line has no normal of its own, so IRIS GL shades it with the
     * current normal (n3f).  The guest sets that immediately before the line
     * run: ideas' curves carry (1,0,0) or (0.766,0,-0.643), which face the
     * lights it sends (all in the xy plane).  A hard-coded (0,0,1) normal is
     * perpendicular to every one of those lights, so the curves shaded to
     * black. */
    nx[0] = s->ge_normal[0];
    nx[1] = s->ge_normal[1];
    nx[2] = s->ge_normal[2];
    sgi_gr2_ge7_shade(s, nx, shininess, col);
    for (i = 0; i < s->ge_line_n; i++) {
        float p[3];
        float cx, cy, cz;

        p[0] = s->ge_line[i][0];
        p[1] = s->ge_line[i][1];
        p[2] = 0.0f;
        if (!sgi_gr2_ge7_xform(s, p, &cx, &cy, &cz)) {
            px[i] = -1e9f;
            continue;
        }
        px[i] = vx + (cx + 1.0f) * 0.5f * vw;
        py[i] = vy + (1.0f - (cy + 1.0f) * 0.5f) * vh;
    }
    for (i = 1; i < s->ge_line_n; i++) {
        float x0 = px[i - 1], y0 = py[i - 1];
        float x1 = px[i], y1 = py[i];
        int steps, st;

        if (x0 < -1e8f || x1 < -1e8f) {
            continue;
        }
        steps = (int)MAX(fabsf(x1 - x0), fabsf(y1 - y0));
        if (steps <= 0) {
            continue;
        }
        if (steps > 4096) {
            steps = 4096;
        }
        for (st = 0; st <= steps; st++) {
            float t = (float)st / (float)steps;
            int x = (int)(x0 + (x1 - x0) * t + 0.5f);
            int y = (int)(y0 + (y1 - y0) * t + 0.5f);
            float z = -1.0f; /* 2D lines on top: they are the app's ink, not
                              * depth-sorted geometry (ideas). */
            size_t o;

            x = MAX(x, vx); y = MAX(y, vy);
            if (x >= SGI_GR2_SCREEN_W || y >= SGI_GR2_SCREEN_H ||
                x >= vx + vw || y >= vy + vh) {
                continue;
            }
            o = (size_t)y * SGI_GR2_SCREEN_W + x;
            if (z < s->ge_zbuf[o]) {
                s->ge_zbuf[o] = z;
                sgi_gr2_put332(s, x, y, sgi_gr2_ge7_332(s, col, x, y));
                ink_px++;
            }
        }
    }
    trace_sgi_gr2_ge7_ink(s->ge_line_n, (int)ink_px,
                          (int)(col[0] * 255.0f + 0.5f),
                          (int)(col[1] * 255.0f + 0.5f),
                          (int)(col[2] * 255.0f + 0.5f));
    trace_sgi_gr2_ge7_inkstate((int)(s->ge_normal[0] * 100.0f),
                               (int)(s->ge_normal[1] * 100.0f),
                               (int)(s->ge_normal[2] * 100.0f),
                               (int)(s->ge_diffuse[0] * 255.0f + 0.5f),
                               (int)(s->ge_diffuse[1] * 255.0f + 0.5f),
                               (int)(s->ge_diffuse[2] * 255.0f + 0.5f),
                               (int)s->puc_colour, (int)s->re3_colour);
    s->ge_3d_seen = true;
}

/* Feed one FIFO token word to the GE7 3D path.  Returns true if the token was
 * a 3D command port (so the caller can skip unrelated paths if it wants). */
/* Accumulate one component of a material/light vector.  The client repeats
 * the token for every component, so a change of token starts a new vector;
 * components past the third (e.g. diffuse's alpha) are collected and dropped
 * rather than spilling into the next vector. */
static void sgi_gr2_ge7_mat_word(SGIGr2State *s, float *dst, hwaddr tok,
                                 float f)
{
    if (s->ge_mat_tok != tok) {
        s->ge_mat_tok = tok;
        s->ge_mat_n = 0;
    }
    if (s->ge_mat_n < 3) {
        dst[s->ge_mat_n] = f;
    }
    s->ge_mat_n++;
    if (s->ge_mat_n >= 3) {
        s->ge_mat_valid = true;
    }
}

/* One component of a light position (token 127).  A run is four floats
 * (x, y, z, w); the position is the first three.  The run belongs to the light
 * selected by the last (5.0, index) pair on token 128 - the index the wire
 * does carry - so it lands in that light's own slot, and a light is valid once
 * it has a position. */
static void sgi_gr2_ge7_light_word(SGIGr2State *s, float f)
{
    unsigned li = s->ge_light_cur;

    s->ge_lpos[s->ge_lpos_n++] = f;
    if (s->ge_lpos_n < 4) {
        return;
    }
    s->ge_lpos_n = 0;
    s->ge_lights[li][0] = s->ge_lpos[0];
    s->ge_lights[li][1] = s->ge_lpos[1];
    s->ge_lights[li][2] = s->ge_lpos[2];
    s->ge_light_valid[li] = true;
    s->ge_mat_valid = true;
}

static void sgi_gr2_ge7_token(SGIGr2State *s, hwaddr offset, uint64_t value)
{
    uint32_t v = (uint32_t)value;

    /* Any token that is not the viewport's own data word closes an armed
     * viewport early (its three words are always contiguous). */
    if (s->vp_armed && offset != SGI_GR2_GE7_VIEWPORT &&
        offset != SGI_GR2_HQ_TOKEN_START) {
        s->vp_armed = false;
    }
    /* The window rect is the same shape: token 485 carries x, the next three
     * PUC_DATA words are y_bottom, w and h. */
    if (s->ge_clip_armed && offset != SGI_GR2_GE7_WINRECT &&
        offset != SGI_GR2_HQ_TOKEN_START) {
        s->ge_clip_armed = false;
    }
    /* A matrix operand is always 16 consecutive words on its own port.  Any
     * other token ends a run, so a partial run cannot leak into the next
     * matrix and misalign it. */
    if (offset != SGI_GR2_GE7_MV) {
        s->ge_mv_n = 0;
    }
    if (offset != SGI_GR2_GE7_PROJ) {
        s->ge_proj_n = 0;
    }
    if (offset != SGI_GR2_GE7_SINGLE) {
        s->ge_single_n = 0;
    }

    switch (offset) {
    case SGI_GR2_GE7_WINRECT:
        /* The GL window's screen rect.  The DDX emits it per frame, so the
         * origin we latch here follows a window move without any extra
         * instrumentation.  y is bottom-origin (the raster path below is
         * top-origin), so convert once the height is known. */
        s->ge_clip_x = (int)v;
        s->ge_clip_n = 1;
        s->ge_clip_armed = true;
        break;
    case SGI_GR2_GE7_VIEWPORT:
        s->vp_x = (int)v;
        s->vp_n = 1;
        s->vp_armed = true;
        s->vp_valid = false;
        break;
    case SGI_GR2_HQ_TOKEN_START: /* token 479: viewport/window (y,w,h) or GE data */
        if (s->vp_armed) {
            if (s->vp_n == 1) {
                s->vp_y = (int)v;
            } else if (s->vp_n == 2) {
                s->vp_w = (int)v;
            } else if (s->vp_n == 3) {
                s->vp_h = (int)v;
                s->vp_valid = true;
                s->vp_armed = false;
            }
            s->vp_n++;
        } else if (s->ge_clip_armed) {
            if (s->ge_clip_n == 1) {
                s->ge_clip_y = (int)v;
            } else if (s->ge_clip_n == 2) {
                s->ge_clip_w = (int)v;
            } else if (s->ge_clip_n == 3) {
                int wy;

                s->ge_clip_h = (int)v;
                wy = SGI_GR2_SCREEN_H - (s->ge_clip_y + s->ge_clip_h);
                if (s->ge_clip_x != s->ge_win_x || wy != s->ge_win_y ||
                    s->ge_clip_w != s->ge_win_w ||
                    s->ge_clip_h != s->ge_win_h) {
                    trace_sgi_gr2_ge7_winrect(s->ge_clip_x, wy, s->ge_clip_w,
                                              s->ge_clip_h);
                }
                s->ge_win_x = s->ge_clip_x;
                s->ge_win_y = wy;
                s->ge_win_w = s->ge_clip_w;
                s->ge_win_h = s->ge_clip_h;
                s->ge_clip_armed = false;
            }
            s->ge_clip_n++;
        }
        break;
    case SGI_GR2_GE7_MV:
        s->ge_mv[s->ge_mv_n++] = sgi_gr2_u2f(v);
        if (s->ge_mv_n == 16) {
            s->ge_mv_n = 0;
            s->ge_mv_valid = true;
            s->ge_seq55 = ++s->ge_seq;
            /* A fresh modelview begins a frame: clear the Z-buffer to "far"
             * (0x7f7f7f7f ~ +3.4e38) so the previous frame does not occlude
             * it, and mark the drawable for a colour clear.  The FIFO stream
             * carries gl_clear (token 158) only once at start-up, so the
             * per-frame double-buffer clear is stood in for here. */
            if (s->ge_zbuf) {
                memset(s->ge_zbuf, 0x7f, (size_t)SGI_GR2_SCREEN_W *
                       SGI_GR2_SCREEN_H * sizeof(float));
            }
            s->ge_need_clear = true;
        }
        break;
    case SGI_GR2_GE7_SINGLE:
        /* MSINGLE combined matrix: the whole transform in one 16-float run. */
        s->ge_single[s->ge_single_n++] = sgi_gr2_u2f(v);
        if (s->ge_single_n == 16) {
            s->ge_single_n = 0;
            s->ge_single_valid = true;
            s->ge_seq54 = ++s->ge_seq;
        }
        break;
    case SGI_GR2_GE7_PROJ:
        s->ge_proj[s->ge_proj_n++] = sgi_gr2_u2f(v);
        if (s->ge_proj_n == 16) {
            s->ge_proj_n = 0;
            s->ge_proj_valid = true;
            s->ge_seq56 = ++s->ge_seq;
        }
        break;
    case SGI_GR2_GE7_TEX:
        break; /* texture matrix: not modelled, the bust is untextured */
    case SGI_GR2_GE7_AMBIENT:
        sgi_gr2_ge7_mat_word(s, s->ge_ambient, offset, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_DIFFUSE:
        sgi_gr2_ge7_mat_word(s, s->ge_diffuse, offset, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_SPECULAR:
        sgi_gr2_ge7_mat_word(s, s->ge_specular, offset, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_EMISSION:
        sgi_gr2_ge7_mat_word(s, s->ge_emission, offset, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_SPEC_LUT:
        /* Token 116 is the specular table: __glExpCreateSpecLUT builds 128
         * entries pow(t, shininess) and the guest uploads them here one word
         * at a time, 128 in a row.  Keep the guest's own curve verbatim and
         * cycle, so the latest upload wins. */
        if (s->ge_spec_lut_n >= 128) {
            s->ge_spec_lut_n = 0;
        }
        s->ge_spec_lut[s->ge_spec_lut_n++] = sgi_gr2_u2f(v);
        break;
    case SGI_GR2_GE7_CULL_FACE:
        /* Tokens 27 and 28 are a PAIR, both written by the guest's own
         * libGLcore.so (IP22GR2NG1): __glExpPassCullFace writes 27=1,28=0 for
         * front-face GL_CW (1028), 27=0,28=1 for GL_CCW (1029), and both=1 for
         * the 1032 mode; __glExpEnableCullFace writes both to 0 when the
         * context's cull flag is clear.  So culling is on iff either is set,
         * and which winding to keep is the polarity.  Phase 3 once keyed the
         * light slots on token 28 by mistake. */
        s->ge_cull_ccw = (v != 0);
        s->ge_mat_n = 0;
        s->ge_lpos_n = 0;
        break;
    case SGI_GR2_GE7_CULL_FACE_CW:
        s->ge_cull_cw = (v != 0);
        break;
    case SGI_GR2_GE7_SPOTLIGHT:
        /* Token 128 is __glExpUpdateLightingState / __glExpValidateLighting.
         * ValidateLighting walks the enabled lights in ascending index order
         * and writes pairs (5.0, index) and (6.0, index) to this port: the
         * first word is a parameter id, the second is the LIGHT INDEX.  That
         * is the index the wire carries; use it to select the slot the colour
         * and position runs that follow belong to. */
        if (s->ge_light_pend) {
            if (v < SGI_GR2_GE7_MAX_LIGHTS) {
                s->ge_light_cur = v;
            }
            s->ge_light_pend = false;
        } else if (v == 0x40a00000u) { /* 5.0 */
            s->ge_light_pend = true;
        }
        break;

    case SGI_GR2_GE7_LCOLOR:
        sgi_gr2_ge7_mat_word(s, s->ge_light_color[s->ge_light_cur], offset,
                             sgi_gr2_u2f(v));
        /* Keep the flat ge_lcolor too: it is the colour the no-lights
         * headlight fallback uses in shade(). */
        memcpy(s->ge_lcolor, s->ge_light_color[s->ge_light_cur],
               sizeof(s->ge_lcolor));
        break;
    case SGI_GR2_GE7_LPOS:
        sgi_gr2_ge7_light_word(s, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_AMBIENT_SUM:
        sgi_gr2_ge7_mat_word(s, s->ge_ambient_sum, offset, sgi_gr2_u2f(v));
        break;
    case SGI_GR2_GE7_LMCOLOR:
        break; /* lighting-model selector: the material above is what we use */
    case SGI_GR2_GE7_DITHER:
        /* gl_d_dither(): 0 turns ordered dithering off.  libgl never sends it
         * in any stream we have captured, so this only matters for a guest
         * that calls dither() explicitly. */
        s->ge_dither = (v != 0);
        break;
    case SGI_GR2_GE7_NORMAL:
        switch (s->ge_n_n++) {
        case 0: s->ge_normal[0] = sgi_gr2_u2f(v); break;
        case 1: s->ge_normal[1] = sgi_gr2_u2f(v); break;
        case 2: s->ge_normal[2] = sgi_gr2_u2f(v); s->ge_n_n = 0; break;
        }
        break;
    case SGI_GR2_GE7_VTX:
        switch (s->ge_v_n++) {
        case 0: s->ge_vx = sgi_gr2_u2f(v); break;
        case 1: s->ge_vy = sgi_gr2_u2f(v); break;
        case 2:
            s->ge_vz = sgi_gr2_u2f(v);
            s->ge_v_n = 0;
            if (s->ge_poly_n < SGI_GR2_GE7_MAX_VERTS) {
                s->ge_poly[s->ge_poly_n][0] = s->ge_vx;
                s->ge_poly[s->ge_poly_n][1] = s->ge_vy;
                s->ge_poly[s->ge_poly_n][2] = s->ge_vz;
                /* the normal that preceded this vertex belongs to it */
                s->ge_vnormal[s->ge_poly_n][0] = s->ge_normal[0];
                s->ge_vnormal[s->ge_poly_n][1] = s->ge_normal[1];
                s->ge_vnormal[s->ge_poly_n][2] = s->ge_normal[2];
                s->ge_poly_n++;
            }
            break;
        }
        break;
    case SGI_GR2_GE7_BGN:
    case SGI_GR2_GE7_BGN_B:
        s->ge_poly_n = 0;
        s->ge_strip = false;
        break;
    case SGI_GR2_GE7_END:
    case SGI_GR2_GE7_END_B:
        sgi_gr2_ge7_draw(s);
        s->ge_poly_n = 0;
        s->ge_strip = false;
        break;
    case SGI_GR2_GE7_BGNLINE:
    case SGI_GR2_GE7_BGNLINE_B:
        s->ge_line_n = 0;
        s->ge_line_pn = 0;
        break;
    case SGI_GR2_GE7_V2F:
        /* gl_v2f/gl_v2i: two coordinates per vertex (token 4707). */
        s->ge_line_pending[s->ge_line_pn++] = sgi_gr2_u2f(v);
        if (s->ge_line_pn == 2) {
            s->ge_line_pn = 0;
            if (s->ge_line_n < SGI_GR2_GE7_MAX_LVERTS) {
                s->ge_line[s->ge_line_n][0] = s->ge_line_pending[0];
                s->ge_line[s->ge_line_n][1] = s->ge_line_pending[1];
                s->ge_line_n++;
            }
        }
        break;
    case SGI_GR2_GE7_ENDLINE:
        sgi_gr2_ge7_draw_lines(s);
        s->ge_line_n = 0;
        break;
    case SGI_GR2_GE7_BGNTMESH:
    case SGI_GR2_GE7_BGNTMESH_B:
        s->ge_poly_n = 0;
        s->ge_strip = true;
        break;
    case SGI_GR2_GE7_ENDTMESH:
        sgi_gr2_ge7_draw(s);
        s->ge_poly_n = 0;
        s->ge_strip = false;
        break;
    case SGI_GR2_GE7_ZCLEAR:
        if (s->ge_zbuf) {
            memset(s->ge_zbuf, 0x7f, (size_t)SGI_GR2_SCREEN_W *
                   SGI_GR2_SCREEN_H * sizeof(float));
        }
        break;
    default:
        break;
    }
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
        uint32_t pc = sgi_gr2_guest_pc();

        trace_sgi_gr2_hqread(offset, val, pc);
    }
    /* The token FIFO is written as the command channel; any READ of it is the
     * board's read-back/consumption contract, so log those with the PC too. */
    if (offset >= SGI_GR2_FIFO_OFF && offset < SGI_GR2_FIFO_OFF + 0x20000) {
        uint32_t pc = sgi_gr2_guest_pc();

        trace_sgi_gr2_fiforead(offset, val, pc);
    }
    /* VC1 / XMAP / RE3 / GE / bdvers reads with the PC: the last surface that
     * could hold the "display is up" gate the DDX waits on. */
    if (offset >= SGI_GR2_HQUCODE_OFF) {
        uint32_t pc = sgi_gr2_guest_pc();

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
        /* The GE7 3D command ports ride the same FIFO as the 2D RE3 ops; feed
         * them the token before the 2D paths see it. */
        if (size == 4) {
            sgi_gr2_ge7_token(s, offset, value);
        }
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
        /* The MONO token's value is the glyph FOREGROUND colour.  A controlled
         * -fg/-bg experiment showed it tracks -fg, while 0x404dc (== the RE3
         * colour) tracks the cell background — so 0x404dc is the background, and
         * reading it painted the glyphs in the background colour. */
        s->re3_monocol = (uint32_t)value & 0xff;
        s->re3_monocol_valid = true;
        s->re3_mono_seen = true;
    }
    if (size == 4 && offset == SGI_GR2_RE3_MONOX_TOKEN) {
        /* IP20: the glyph piece's pen x; y/w/h follow in PUC_DATA.  A new pen
         * means the previous one's data is complete, so draw it now — one
         * region can carry several pens and the terminator only sees the last. */
        if (s->re3_monox_valid) {
            sgi_gr2_re3_draw_mono_pen(s, s->re3_monox, s->re3_mono_off);
        }
        s->re3_monox = (uint32_t)value & 0xffff;
        s->re3_mono_off = s->re3_data_n;
        s->re3_monox_valid = true;
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
    /* bdvers is strapping: the bitplane/Z/revision bits are wired by the board
     * and must survive the PROM/ARCS and driver config writes to the same
     * register.  The ARCS Gr2InitInfo and the kernel Gr2Probe read
     * (~rd0)&0xf for the revision and rd1 bits 4/5 for 24-bit and Z; if a
     * config write is allowed to clear those, the board is reported as
     * "missing bitplanes missing Z" and powerflip refuses.  Preserve the
     * strapped bits, take the rest from the write. */
    if (offset >= SGI_GR2_BDVERS_OFF &&
        offset < SGI_GR2_BDVERS_OFF + 16) {
        static const uint8_t strap_mask[16] = {
            0x0f, 0x0f, 0x0f, 0x0f,  /* 0x6c000: revision nibble strapped */
            0x30, 0x30, 0x30, 0x30,  /* 0x6c004: bit4=24-bit, bit5=Z       */
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        };
        static const uint8_t strap_val[16] = {
            SGI_GR2_BDVERS0, SGI_GR2_BDVERS0, SGI_GR2_BDVERS0, SGI_GR2_BDVERS0,
            SGI_GR2_BDVERS1, SGI_GR2_BDVERS1, SGI_GR2_BDVERS1, SGI_GR2_BDVERS1,
            SGI_GR2_BDVERS2, SGI_GR2_BDVERS2, SGI_GR2_BDVERS2, SGI_GR2_BDVERS2,
            SGI_GR2_BDVERS3, SGI_GR2_BDVERS3, SGI_GR2_BDVERS3, SGI_GR2_BDVERS3,
        };
        unsigned k;

        for (k = 0; k < size; k++) {
            uint64_t o = offset + k - SGI_GR2_BDVERS_OFF;
            uint8_t byte = (value >> (8 * (size - 1 - k))) & 0xff;

            if (o < 16) {
                s->regs[offset + k] = (byte & ~strap_mask[o]) |
                                      (strap_val[o] & strap_mask[o]);
            } else {
                s->regs[offset + k] = byte;
            }
        }
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
    /* Ordered dithering defaults ON: IRIS GL's dither() is on initially and
     * libgl never sends token 506, so the GE's power-up state is what the
     * guest gets.  A guest that calls dither(0) clears it. */
    s->ge_dither = true;
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
    s->re3_monox_valid = false;
    s->re3_monocol_valid = false;
    s->re3_mono_off = 0;
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
    /* A paused token stream is the end of a draw batch.  For the 3D path the
     * whole bust was just rasterised, so refresh the display once here rather
     * than on every endpolygon (which would rescan 1280x1024 5000 times). */
    if (s->ge_3d_seen) {
        sgi_gr2_update_display(s);
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
        /* GE7 Z-buffer.  Allocated up front and cleared to "far" (0x7f7f7f7f
         * ~ +3.4e38) so the first frame draws. */
        s->ge_zbuf = g_new(float,
                           (size_t)SGI_GR2_SCREEN_W * SGI_GR2_SCREEN_H);
        memset(s->ge_zbuf, 0x7f, (size_t)SGI_GR2_SCREEN_W *
               SGI_GR2_SCREEN_H * sizeof(float));
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
