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
            s->scanout[yy * SGI_GR2_SCREEN_W + xx] = colour;
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
    int dx, dy, sx, sy, err;

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
            s->scanout[y0 * SGI_GR2_SCREEN_W + x0] = colour;
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        /* Bound the walk so a wild coordinate cannot spin. */
        if (x0 < -SGI_GR2_SCREEN_W || x0 > 2 * SGI_GR2_SCREEN_W ||
            y0 < -SGI_GR2_SCREEN_H || y0 > 2 * SGI_GR2_SCREEN_H) {
            break;
        }
        if (2 * err >= -dy) {
            err -= dy;
            x0 += sx;
        }
        if (2 * err <= dx) {
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

            s->scanout[yy * SGI_GR2_SCREEN_W + xx] = on ? fg : bg;
        }
    }
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
            break;
        }
        for (k = 0; k < count; k++) {
            uint32_t xx = x + k;

            if (xx >= SGI_GR2_SCREEN_W) {
                break;
            }
            if ((pattern >> (31 - (k & 31))) & 1) {
                s->scanout[y * SGI_GR2_SCREEN_W + xx] = s->re3_colour;
            }
        }
    }
    sgi_gr2_update_display(s);
}

/* Draw a filled polygon (libgd token 302).  After the same "0xff 0x3 0x0"
 * prefix come three header words (0x3ab, 0xc6, 0x2a0 in every op) then the
 * outline as (x,y) vertex pairs, the last pair repeating the first to close the
 * loop.  Filled with an even-odd scanline walk.  These are the glyph outlines —
 * the X server's text goes through expPolyGlyphBlt, which fills glyphs the same
 * way. */
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
                s->scanout[sy * SGI_GR2_SCREEN_W + xx] = s->re3_colour;
            }
        }
    }
    sgi_gr2_update_display(s);
}

/* Draw the name-label glyphs (expDrawMonoImage).  Token 349 is written TWICE
 * per glyph with the pen x, each followed by a piece (f0,f1,h) and h/2 bitmap
 * words.  A glyph is the two pieces STACKED from its top row (drawing them at
 * the same x is what makes the letters readable, not side by side); each piece
 * is 8 px wide, high byte = row 2k, low byte = row 2k+1, bit 15 the leftmost.
 * Set bits take the current colour.  The text y is not on the wire — the DDX
 * bakes the origin in — so it comes from the label bar (colour 222) drawn just
 * before the glyphs; without one the op is flagged, not placed by guesswork. */
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

    /* IP20 path: the origin is on the wire.  Token 0x40510 is the pen x and the
     * three PUC_DATA words after it are y, w, h; token 0x404e0 is the colour.
     * The bitmap is two rows per word, 12 px in bits 31..20 then 15..4, bit 15
     * the leftmost.  This is tried first and, when it fires, replaces the XZ
     * label-bar link entirely — the XZ path cannot reach here (0x40510 is never
     * written there), so its output is unchanged. */
    if (s->re3_monox_valid) {
        sgi_gr2_re3_draw_mono_pen(s, s->re3_monox, s->re3_mono_off);
        return;
    }

    if (!s->re3_label_valid) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
    for (p = 0; p + 1 < s->re3_npens; p += 2) {
        uint32_t pen = s->re3_pen_val[p];
        int y = s->re3_label_y + 3;
        unsigned half;

        for (half = 0; half < 2; half++) {
            unsigned off = s->re3_pen_off[p + half];
            uint32_t f0, f1, h, k;

            if (off + 3 > s->re3_data_n) {
                continue;
            }
            f0 = s->re3_data[off];
            f1 = s->re3_data[off + 1];
            h = s->re3_data[off + 2];
            if (f0 >= SGI_GR2_SCREEN_W || f1 >= SGI_GR2_SCREEN_H ||
                h < 2 || h > 64 || (h & 1) ||
                off + 3 + h / 2 > s->re3_data_n) {
                continue;
            }
            for (k = 0; k < h / 2; k++) {
                uint32_t w = s->re3_data[off + 3 + k];
                int b;

                for (b = 0; b < 8; b++) {
                    int xx = (int)pen + b, r0 = y + 2 * (int)k;

                    if (xx >= SGI_GR2_SCREEN_W) {
                        break;
                    }
                    if (r0 < SGI_GR2_SCREEN_H && ((w >> (15 - b)) & 1)) {
                        s->scanout[r0 * SGI_GR2_SCREEN_W + xx] = s->re3_colour;
                    }
                    if (r0 + 1 < SGI_GR2_SCREEN_H && ((w >> (7 - b)) & 1)) {
                        s->scanout[(r0 + 1) * SGI_GR2_SCREEN_W + xx] =
                            s->re3_colour;
                    }
                }
            }
            y += (int)h;
            any = true;
        }
    }
    if (!any) {
        trace_sgi_gr2_re3_unmatched(s->re3_rop, s->re3_data_n);
        return;
    }
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
    if (s->re3_mono_seen) {
        sgi_gr2_re3_draw_text(s);
        return;
    }
    if (s->re3_poly_seen) {
        sgi_gr2_re3_draw_polygon(s);
        return;
    }
    if (s->re3_spanstip_seen) {
        sgi_gr2_re3_draw_stippled_spans(s);
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
    s->re3_monox_valid = false;
    s->re3_monocol_valid = false;
    s->re3_npens = 0;
    s->re3_pair_seen = false;
    s->re3_stipple_valid = false;
    s->re3_fg_valid = false;
    s->re3_rop_valid = false;
    s->re3_data_n = 0;
    s->re3_data_overflow = false;
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
            s->scanout[y * SGI_GR2_SCREEN_W + x] = x * 8 / SGI_GR2_SCREEN_W;
        }
    }
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
         * entry was programmed still shows the entry's final colour. */
        for (x = 0; x < SGI_GR2_SCREEN_W; x++) {
            row[x] = s->ramdac[src[x]];
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
    s->re3_colour = 0;
    s->re3_colour_valid = false;
    s->re3_solid_seen = false;
    s->re3_spans_seen = false;
    s->re3_line_seen = false;
    s->re3_spanstip_seen = false;
    s->re3_poly_seen = false;
    s->re3_mono_seen = false;
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
    timer_mod(s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / SGI_GR2_RETRACE_HZ);

    /* Scanout (P0.4 step a) only when the board is present, so plain
     * `-M indy` gains no extra console. */
    if (s->present) {
        s->scanout = g_new0(uint8_t,
                            (size_t)SGI_GR2_SCREEN_W * SGI_GR2_SCREEN_H);
        s->con = graphic_console_init(dev, 0, &sgi_gr2_gfx_ops, s);
        qemu_console_resize(s->con, SGI_GR2_SCREEN_W, SGI_GR2_SCREEN_H);
    }
}

static const Property sgi_gr2_properties[] = {
    DEFINE_PROP_BOOL("present", SGIGr2State, present, false),
    DEFINE_PROP_BOOL("scanout-bars", SGIGr2State, scanout_bars, false),
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
