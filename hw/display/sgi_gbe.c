/*
 * SGI GBE (Graphics Back End) emulation
 *
 * Full raster/timing + tile-scanout model for the O2 (IP32) graphics
 * back end. The raster is derived from real (virtual-clock) time: a
 * periodic QEMUTimer at the frame rate sweeps the full total extent
 * (active + blanking), so the PROM's waitForBlanking() poll and the
 * turnOnGbe() freeze-poll both exit naturally.
 *
 * Register contract (crm_init.c bring-up order):
 *   gbeSetTimingRegs writes vsync/hsync/vblank/hblank/hcmap/vcmap,
 *   fp_de/fp_hdrv/fp_vdrv, did/crs/vc_start_xy, frm_size_tile,
 *   frm_size_pixel, dotclock, 11ms delay, vpixen/hpixen, vt_xymax
 *   ((vtotal<<12)|htotal), frm_size_tile toggle, ovr_width_tile toggle,
 *   frm_control (list ptr | enable), did_control, then spins on
 *   frm_inhwctrl bit 0 until set. Our VSync latch (frame timer) copies
 *   ctrl->inhwctrl (enable bit INCLUDED) once per frame, so the spin
 *   exits within one frame period.
 *
 * Tile scanout: frm_ctrl = descriptor-list pointer | enable bit.
 * The list is guest RAM (UMA), big-endian uint16 tile numbers
 * (physical address = tilenum << 16, 64KB tiles; 0x8000 = valid).
 * Tiles are walked per gxemul: width-tiles across, 128 lines per tile
 * row, 512/256/128 contiguous pixels per tile line at 8/16/32bpp.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/display/sgi_gbe.h"
#include "hw/display/edid.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/pixel_ops.h"
#include "system/address-spaces.h"
#include "trace.h"
#include "framebuffer.h"

/*
 * Update the derived raster geometry from the programmed timing regs.
 * vt_xymax = (vtotal << 12) | htotal (written last by gbeSetTimingRegs).
 */
static void sgi_gbe_update_geometry(SGIGBEState *s)
{
    uint32_t xmax = s->vt_xymax & 0xfff;
    uint32_t ymax = (s->vt_xymax >> 12) & 0xfff;

    if (xmax == 0 || ymax == 0) {
        /* not programmed yet: keep defaults */
        xmax = GBE_DEF_HTOTAL;
        ymax = GBE_DEF_VTOTAL;
    }
    s->htotal = xmax;
    s->vtotal = ymax;

    /* hblank: on = bits [23:12], off = bits [11:0] (gbeSetTimingRegs) */
    uint32_t hb = s->vt_regs[GBE_VT_IDX(GBE_VT_HBLANK)];
    s->hblank_start = (hb >> 12) & 0xfff;
    s->hblank_end = hb & 0xfff;

    uint32_t vb = s->vt_regs[GBE_VT_IDX(GBE_VT_VBLANK)];
    s->vblank_start = (vb >> 12) & 0xfff;
    s->vblank_end = vb & 0xfff;

    /*
     * Frame period: nominal 60Hz. Refine against the dotclock if it
     * looks sane (run bit 0x100000 set) using the simple PLL reading
     * dotclock = base * m / (n * pdiv) — do NOT chase exactness.
     */
    s->refresh_hz = GBE_DEF_REFRESH;
    uint32_t dot = s->dotclock;
    if ((dot & 0x100000) && (dot & 0xff)) {
        uint32_t m = (dot & 0xff) + 1;
        uint32_t n = ((dot >> 8) & 0x3f) + 1;
        uint32_t pdiv = 1u << ((dot >> 14) & 3);
        /* base ~ 66.67MHz/8 ≈ 8.33MHz per unit */
        double khz = 8333.0 * m / n / pdiv;
        if (khz > 1000.0 && khz < 400000.0) {
            double hz = khz * 1000.0 / ((double)s->htotal * s->vtotal);
            if (hz > 40.0 && hz < 160.0) {
                s->refresh_hz = (uint32_t)(hz + 0.5);
            }
        }
    }
}

/*
 * Current raster position from real time.  The beam sweeps the full
 * htotal*vtotal dot extent once per 1/refresh seconds, phase-locked
 * to the virtual clock (see the comment inside for why the position
 * is a pure clock function rather than frame-tick-relative).
 * X bits [11:0], Y bits [23:12]; bit 31 = freeze flag (NetBSD
 * crmfbreg.h CRMFB_VT_XY_X/Y_MASK — the same layout the PROM's
 * waitForBlanking decodes).
 */
static uint32_t sgi_gbe_current_xy(SGIGBEState *s, int64_t now)
{
    int64_t frame_ns = NANOSECONDS_PER_SECOND / (int64_t)s->refresh_hz;
    int64_t sweep = ((int64_t)s->htotal * s->vtotal);
    uint64_t dot;

    /*
     * Free-running raster: the sweep position is a pure function of
     * the virtual clock (dot = phase of now within the frame period,
     * scaled to the full htotal*vtotal sweep, wrapping per frame).
     * The old frame_start_ns-relative computation stalled at the top
     * of the frame whenever the QEMU frame timer fired late under
     * load (the kernel's gbeStartVideoTiming poll, which waits for
     * VT_XY to pass through the top-of-frame window, then wedged
     * forever seeing only clamped end-of-frame positions — observed
     * as a pre-login hang at crime_init.c:815).  Deriving the
     * position directly from the clock keeps the raster sweeping
     * every frame_ns regardless of timer lag; the frame tick (which
     * still maintains frame_start_ns for the scanout/vsync latch)
     * stays aligned because it re-arms on the same frame_ns period.
     */
    if (frame_ns <= 0 || sweep <= 0 || s->htotal <= 0) {
        return 0;
    }
    dot = (((uint64_t)now % (uint64_t)frame_ns) * (uint64_t)sweep) / frame_ns;
    if (dot >= (uint64_t)sweep) {
        dot = (uint64_t)sweep - 1;
    }
    uint32_t x = dot % s->htotal;
    uint32_t y = dot / s->htotal;
    /*
     * Layout per NetBSD crmfbreg.h (via gxemul thirdparty) and the
     * PROM's decoders: X = bits [11:0], Y = bits [23:12], bit 31 =
     * freeze. crm_init.c waitForBlanking reads tempY = (val &
     * 0x00fff000) >> 12 and compares against the vblank window; the
     * turnOnGbe freeze-poll only looks at bit 31. (vt_xymax keeps its
     * own separate encoding (vtotal<<12)|htotal.)
     */
    return (y << 12) | x;
}

/* ---------------- frame timer: raster tick + vsync latch -------------- */

static void sgi_gbe_invalidate(void *opaque);
static void sgi_gbe_update(void *opaque);

/*
 * Called at each frame boundary (VSync edge):
 *  1. latch ovr/frm/did ctrl -> inhwctrl (enable bits kept)
 *  2. raise the vt_intr line-compare interrupts (M11a): GBE0 when the
 *     programmed vt_intr0 line equals the retrace line, GBE1 for
 *     vt_intr1, GBE2/GBE3 for vt_intr2/vt_intr3 — replacing the M7
 *     unconditional GBE0/GBE1 per-frame pulses. When no compare lines
 *     are programmed (vt_intr01/23 == 0, the PROM/textport case), GBE0
 *     (retrace) still fires at vsync so the kernel's retrace handler
 *     sees frames (matches the M7 behavior for unprogrammed regs).
 *  3. run the tile scanout into the QEMU console
 *  4. re-arm the timer for the next frame
 *
 * vt_intr01 packs intr1:intr0 as 24-bit halves: bits 27:16 = intr1,
 * bits 11:0 = intr0 (VT_7 diagram: "vt_int1 / vt_intr0"; same layout
 * for vt_intr23 = VT_9: intr3:intr2).
 */
static void sgi_gbe_frame_tick(void *opaque)
{
    SGIGBEState *s = SGI_GBE(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t frame_ns = NANOSECONDS_PER_SECOND / (int64_t)s->refresh_hz;

    trace_sgi_gbe_frame_tick(s->htotal, s->vtotal, s->refresh_hz);

    /* VSync edge: latch the ctrl values into the in-hardware copies */
    s->ovr_inhwctrl = s->ovr_ctrl;
    s->frm_inhwctrl = s->frm_ctrl;
    s->did_inhwctrl = s->did_ctrl;
    trace_sgi_gbe_latch(s->ovr_ctrl, s->frm_ctrl, s->did_ctrl);

    /*
     * Line-compare interrupts. The compares are against vt_y during
     * the frame sweep; we evaluate at the vsync boundary: a programmed
     * line is "crossed" by the end of the frame. Unprogrammed (0)
     * compare lines never match (line 0 is the top of the frame —
     * treat 0 as "not programmed" for intr0/intr1 so the legacy
     * per-frame retrace pulse is preserved for the textport).
     */
    uint32_t intr0 = s->vt_intr01 & 0xfff;
    uint32_t intr1 = (s->vt_intr01 >> 16) & 0xfff;
    uint32_t intr2 = s->vt_intr23 & 0xfff;
    uint32_t intr3 = (s->vt_intr23 >> 16) & 0xfff;
    bool fired[4] = { false, false, false, false };

    if (intr0 == 0) {
        fired[0] = true;         /* GBE0 retrace default at vsync */
    } else if (intr0 < s->vtotal) {
        fired[0] = true;
    }
    if (intr1 != 0 && intr1 < s->vtotal) {
        fired[1] = true;
    }
    if (intr2 != 0 && intr2 < s->vtotal) {
        fired[2] = true;
    }
    if (intr3 != 0 && intr3 < s->vtotal) {
        fired[3] = true;
    }

    /*
     * Interrupt lines: the old code raised AND lowered within this
     * one callback — a zero-virtual-time pulse the vCPU could never
     * observe, so the kernel's GBE0 (retrace) / GBE1 (preblank)
     * vectors never fired and Xsgi stalled in its retrace waits
     * until the gthread watchdog (GfxKiller) SIGKILLed it.
     *
     * The line must stay asserted long enough for the vCPU to take
     * the interrupt (the kernel's crime_intr masks the source in
     * CRM_INTMASK before signalling its ithread), then drop before
     * the ithread re-enables the mask — otherwise a held level
     * re-interrupts on every unmask and the guest spins in an
     * interrupt storm (observed: boot stalls in splint dispatch).
     * GBE0/GBE1 are edge-style in software terms: nobody writes
     * INTSTAT to clear them, so the line itself must go low.  A
     * 100 us pulse satisfies both (about 6000 CPU instructions at
     * the emulated clock — far more than the take-and-mask path
     * needs, far less than the ithread service time).
     */
    for (int i = 0; i < 4; i++) {
        if (fired[i]) {
            qemu_irq_raise(s->crime_irq[i]);
        } else {
            qemu_irq_lower(s->crime_irq[i]);
        }
    }
    if (fired[0] || fired[1] || fired[2] || fired[3]) {
        trace_sgi_gbe_irq_raise();
        if (s->irq_timer) {
            timer_mod(s->irq_timer, now + 100000);   /* 100 us */
        }
    }

    /* Scanout + display update while the beam is in blanking */
    if (s->con) {
        sgi_gbe_update(s);
    }

    s->frame_start_ns = now;
    timer_mod(s->frame_timer, now + frame_ns);
}

/* Drop the GBE interrupt lines at the end of the pulse window. */
static void sgi_gbe_irq_pulse_end(void *opaque)
{
    SGIGBEState *s = SGI_GBE(opaque);

    for (int i = 0; i < 4; i++) {
        qemu_irq_lower(s->crime_irq[i]);
    }
    trace_sgi_gbe_irq_lower();
}

/* -------------------- tile scanout into the console ------------------- */

/*
 * Composite the hardware cursor over the scanned-out frame (spec
 * §2.10 Cursor): 32x32 glyph, 2 bits/pixel packed 16-per-u32 in
 * crs_glyph[64], position from crs_pos with the (31,31) offset —
 * "the lower right pixel of the cursor glyph corresponds to the upper
 * left corner of the active raster", i.e. screen pixel (sx,sy) samples
 * glyph pixel (sx - posx + 31, sy - posy + 31). Glyph value 0 is
 * transparent; 1-3 index crs_cmap[0..2] (packed RGB). crs_ctrl bit 0
 * = enable, bit 1 = crosshair mode (crosshair uses color 1).
 */
static void sgi_gbe_composite_cursor(SGIGBEState *s, DisplaySurface *surface)
{
    if (!(s->crs_ctrl & 1) || !surface) {
        return;
    }
    int posx = (s->crs_pos >> 16) & 0xfff;
    int posy = s->crs_pos & 0xfff;
    int sw = surface_width(surface);
    int sh = surface_height(surface);
    int stride = surface_stride(surface) / sizeof(uint32_t);
    uint32_t *dst = (uint32_t *)surface_data(surface);

    if (s->crs_ctrl & 2) {
        /* crosshair: full-width/height lines in crs_cmap[0] (color 1) */
        uint32_t ent = s->crs_cmap[0];
        uint32_t c = rgb_to_pixel32((ent >> 24) & 0xff,
                                    (ent >> 16) & 0xff,
                                    (ent >> 8) & 0xff);
        if (posy < sh) {
            for (int x = 0; x < sw; x++) {
                dst[posy * stride + x] = c;
            }
        }
        if (posx < sw) {
            for (int y = 0; y < sh; y++) {
                dst[y * stride + posx] = c;
            }
        }
        return;
    }

    for (int gy = 0; gy < 32; gy++) {
        int sy = posy - 31 + gy;
        if (sy < 0 || sy >= sh) {
            continue;
        }
        for (int gx = 0; gx < 32; gx++) {
            int sx = posx - 31 + gx;
            if (sx < 0 || sx >= sw) {
                continue;
            }
            /* glyph row gy: crs_glyph[2*gy] = left 16 px, [2*gy+1] = right */
            uint32_t word = s->crs_glyph[2 * gy + (gx >= 16 ? 1 : 0)];
            int bit_off = (gx & 15) * 2;
            uint32_t val = (word >> (30 - bit_off)) & 3;
            if (val == 0) {
                continue;               /* transparent */
            }
            uint32_t ent = s->crs_cmap[val - 1];
            dst[sy * stride + sx] =
                rgb_to_pixel32((ent >> 24) & 0xff, (ent >> 16) & 0xff,
                               (ent >> 8) & 0xff);
        }
    }
}

/*
 * Decode one pixel through the FRM channel. The PROM programs all 32
 * WIDs to I8/CM0 (initFramebuffer), so 8bpp cmap lookups are the gate
 * path; 16bpp RGB5 and 32bpp direct are supported for the kernel.
 *
 * M11a adds the deferred M7 items:
 *  - OVR overlay channel scanout: 8bpp, always indexes cmap entries
 *    4352..4607 (GBE_OVR_CMAP_OFFSET 0x1100), pixel 0x00 = transparent
 *    (GBE spec §Overlay planes). Preferred over FRM when its DMA is
 *    enabled (gxemul does the same). The 0x1100 base is FIXED by the
 *    spec ("GBE does not support display ID bits for the overlay
 *    planes") — NOT WID-derived (m11o: kernel tp mapcolor writes land
 *    at exactly 0x1100+col+1).
 *  - cursor compositing: 32x32 2bpp glyph, position (crs_posx,crs_posy)
 *    with the (31,31) offset convention, color from crs_cmap[0..2]
 *    (glyph value 0 transparent; spec §2.10 Cursor).
 *  - I8 FRM pixels index cmap through the WID CM field (spec §2.7
 *    "cm(4:0) & i8(7:0)"); with the DID stream off all pixels take
 *    WID 0 / mode_regs[0] (cm=0 → identical to the old window-0 lookup).
 */
static void sgi_gbe_scanout(SGIGBEState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    if (!surface) {
        return;
    }

    int depth = (s->frm_size_tile >> 13) & 3;   /* 0=8 1=16 2=32bpp */
    int bpp = (depth == 0) ? 1 : (depth == 1) ? 2 : 4;
    int width_tiles = (s->frm_size_tile >> 5) & 0xff;
    int rhs_pixels = (s->frm_size_tile & 0x1f) * 32 / bpp;
    int height = s->frm_size_pixel >> 16;

    /* tile width in pixels: 512@8bpp, 256@16bpp, 128@32bpp */
    int pix_per_tile = (bpp == 1) ? 512 : (bpp == 2) ? 256 : 128;
    int width = width_tiles * pix_per_tile + rhs_pixels;
    if (width <= 0 || height <= 0) {
        return;
    }
    width = MIN(width, 2048);
    height = MIN(height, 2048);

    if (s->scan_width != width || s->scan_height != height) {
        qemu_console_resize(s->con, width, height);
        s->scan_width = width;
        s->scan_height = height;
        surface = qemu_console_surface(s->con);
        if (!surface) {
            return;
        }
    }

    /*
     * Tile-descriptor-list pointers: FRM_2/OVR_1 field "frm_tile_ptr"
     * is bits 31:5 of the ctrl register (GBE spec §3.2.3/§3.2.4 — 32
     * byte aligned; gbedefs.h GBE_FRM_TILE_PTR_MASK 0xFFFFFFE0).
     * The old 0xFFFFFFC0 mask rounded 32-byte-aligned lists (e.g. the
     * overlay list at 0x7d1120) down 64 bytes — into the middle of the
     * FRM list — so the overlay channel composited garbage tiles.
     */
    hwaddr list_ptr = s->frm_ctrl & 0xffffffe0;
    bool frm_on = (s->frm_ctrl & 1) != 0 && list_ptr != 0;
    if (!frm_on && !(s->ovr_ctrl & 1)) {
        return;     /* neither channel's DMA enabled */
    }
    trace_sgi_gbe_scanout(width, height, width_tiles, list_ptr);

    /*
     * Overlay channel: ovr_width_tile packs (rhs<<16)|width_tiles like
     * frm_size_tile's tile geometry; overlay is 8bpp so a tile is 512
     * pixels wide. Enable = ovr_inhwctrl bit 0 (latched at vsync).
     * The overlay has its OWN tile geometry: the descriptor list is
     * indexed with the overlay's tiles-per-row (not the normal planes'
     * tilenr), per spec §2.2 "tiles ordered top to bottom, left to
     * right" — a 1280px overlay at 8bpp is 3 tile columns (2 full +
     * rhs 256px), independent of the 10-tile 32bpp normal-plane row.
     */
    bool ovr_on = (s->ovr_inhwctrl & 1) != 0;
    int ovr_tiles = (s->ovr_width_tile >> 5) & 0xff;
    int ovr_rhs_px = (s->ovr_width_tile & 0x1f) * 32;    /* 8bpp: bytes == px */
    int ovr_cols = ovr_tiles + (ovr_rhs_px > 0 ? 1 : 0); /* tile columns */
    hwaddr ovr_list = s->ovr_ctrl & 0xffffffe0;

    int w = width_tiles + (rhs_pixels > 0 ? 1 : 0);
    int stride = surface_stride(surface) / sizeof(uint32_t);
    uint32_t *dst = (uint32_t *)surface_data(surface);

    /*
     * Walk tile ROWS, not pixel columns: one tile row covers 128 LINES
     * (each tile is 64KB = 128 lines x 512 bytes; a "row" of width_tiles
     * tiles spans the full screen width at any bpp). The loop bound is
     * therefore over HEIGHT: row * 128 < height, i.e. 1024 lines = 8 rows
     * of 16 tiles at 1280x1024@8bpp (gxemul dev_sgi_gbe.c dev_sgi_gbe_tick
     * walks tiley = 0..255, aborting once the screen is filled; we size
     * the walk from frm_size_pixel directly).
     */
    int tile_rows = (height + 127) / 128;
    for (int row = 0; row * 128 < height && row < tile_rows; row++) {
        /* tile row y range: lines 0..127 within the tile */
        for (int line = 0; line < 128; line++) {
            int y = row * 128 + line;
            if (y >= height) {
                break;
            }
            int x = 0;
            for (int tx = 0; tx < w; tx++) {
                int tilenr = tx + row * w;
                if (tilenr >= 256) {
                    break;
                }
                /* descriptor: big-endian u16 at list_ptr + 2*tilenr.
                 * PROM initFramebuffer writes plain tile numbers
                 * (phys>>16) with NO valid bit; an empty slot is 0.
                 * (The 0x8000 valid marker is an RE-TLB-entry-only
                 * convention; a stray flagged entry still decodes as a
                 * tile number through the 0x7fff mask.) */
                uint16_t desc = 0;
                if (frm_on) {
                    address_space_read(&address_space_memory,
                                       list_ptr + 2 * tilenr,
                                       MEMTXATTRS_UNSPECIFIED, &desc, 2);
                    desc = be16_to_cpu(desc);
                }
                if (desc == 0) {
                    x += pix_per_tile;
                    continue;
                }
                hwaddr tile_base = (hwaddr)(desc & 0x7fff) << 16;

                int pix_here = pix_per_tile;
                if (tx == width_tiles && rhs_pixels > 0) {
                    pix_here = rhs_pixels;
                }
                uint8_t buf[512 * 4];
                int nbytes = pix_here * bpp;
                address_space_read(&address_space_memory, tile_base + 512 * line,
                                   MEMTXATTRS_UNSPECIFIED, buf, MIN(nbytes, 512 * 4));

                /* overlay tile line for this span (8bpp: 512 px/tile) */
                uint8_t ovr_buf[512];
                bool have_ovr = false;
                if (ovr_on && ovr_list != 0) {
                    /*
                     * Overlay descriptor index: the overlay's own
                     * tile-row/column geometry (row * ovr_cols + col),
                     * NOT the normal planes' tilenr.
                     */
                    int ov_col = x / 512;
                    int ov_tilenr = (y >> 7) * ovr_cols + ov_col;
                    if (ov_tilenr < 256) {
                        uint16_t odesc;
                        address_space_read(&address_space_memory,
                                           ovr_list + 2 * ov_tilenr,
                                           MEMTXATTRS_UNSPECIFIED, &odesc, 2);
                        odesc = be16_to_cpu(odesc);
                        if (odesc != 0) {
                            hwaddr obase = (hwaddr)(odesc & 0x7fff) << 16;
                            address_space_read(&address_space_memory,
                                               obase + 512 * (y & 127),
                                               MEMTXATTRS_UNSPECIFIED, ovr_buf,
                                               MIN(pix_here, 512));
                            have_ovr = true;
                        }
                    }
                }

                for (int i = 0; i < pix_here && x < width; i++, x++) {
                    uint32_t r, g, b;
                    bool ov_used = false;
                    if (have_ovr) {
                        uint32_t oidx = ovr_buf[i];
                        if (oidx != 0) {
                            /*
                             * Overlay cmap window: FIXED at entries 4352..4607
                             * (0x1100..0x11ff). GBE spec §2.3 Overlay planes:
                             * "GBE does not support display ID bits for the
                             * overlay planes. The 8 bits of overlay color index
                             * always point to entries 4352 to 4607 in the color
                             * map." Verified against the guest: the kernel
                             * textport's mapcolor (0x80277444) writes
                             * cmap[0x1101 + col] while its color() (0x80277418)
                             * draws pixel col+1 — self-consistent only with
                             * this fixed base (m11o run2 trace: writes at
                             * 0x1101/0x1102 land exactly here).
                             */
                            uint32_t ent = s->cmap[0x1100 + oidx];
                            r = (ent >> 24) & 0xff;
                            g = (ent >> 16) & 0xff;
                            b = (ent >> 8) & 0xff;
                            ov_used = true;
                        }
                    }
                    if (!ov_used) {
                        /*
                         * Pixel decode is governed by the effective WID's
                         * typ field (spec §2.4 window table: typ 0=I8,
                         * 1=I12, 2=RG3B2, 3=RGB4, 4=RGB5, 5=RGB8; cm =
                         * the upper 5 cmap bits for I8; buf = the 8+8 /
                         * 16+16 double-buffer half). With the DID stream's
                         * DMA off every pixel takes WID 0 / mode_regs[0]
                         * (the M11a/M11o rule, kept below for the 8bpp
                         * fetch geometry).
                         *
                         * M11k: the O2 X server runs its 8bpp PseudoColor
                         * screen through the 32bpp NORMAL planes using the
                         * spec's 8+8 split ("The normal stream can be 16 or
                         * 32 bits deep, and can be split using the DIDs to
                         * provide 8+8 and 16+16 double buffering" — §1
                         * Overview): the DDX writes its 8bpp pixels into the
                         * byte lanes of the 32-bit words (observed live:
                         * byte-lane-rotated per-scanline MTE fills; the
                         * xsetroot red fill = byte 1 in every lane) and the
                         * WID's I8 typ + cm=16 (WID0=0x203) makes GBE decode
                         * each fetched word's 8bpp index through the cmap
                         * window the DDX loaded (cmap[4096+byte]). The old
                         * direct-RGB decode of the 32bpp fetch ignored the
                         * WID typ entirely, so X's 8bpp content decoded as
                         * near-black RGB (byte 1 -> (1,1,1)) and the screen
                         * stayed dark. Honor the WID typ for every fetch
                         * depth; the byte lane for the 8+8 split follows
                         * the WID buf bits (11=both -> lane 0; 01 lower /
                         * 10 upper half), and 'both' composites lane 0 (the
                         * DDX fills replicate to all lanes, so any fixed
                         * lane choice renders its content).
                         */
                        uint32_t wid = s->mode_regs[0];
                        uint32_t typ = (wid >> 2) & 0x7;   /* WID[4:2] typ */
                        uint32_t cm = (wid >> 5) & 0x1f;   /* WID[9:5] cm */
                        uint32_t bufsel = wid & 0x3;       /* WID[1:0] buf */
                        if (bpp == 1) {
                            /*
                             * 8bpp fetch: the old M11o decode (WID-CM
                             * window) — unchanged semantics.
                             */
                            uint32_t idx = (cm << 8) | buf[i];
                            uint32_t ent = s->cmap[idx];
                            r = (ent >> 24) & 0xff;
                            g = (ent >> 16) & 0xff;
                            b = (ent >> 8) & 0xff;
                        } else if (typ == 0) {
                            /*
                             * I8 WID on a 16/32-bit fetch (the 8+8 / 16+16
                             * split): the pixel index is the 8bpp byte in
                             * the WID-selected half of the fetched word.
                             * Lane per buf: 01 = lower half (byte 0 of the
                             * half), 10 = upper half, 11 = both -> byte 0.
                             */
                            int lane;
                            if (bpp == 2) {
                                lane = (bufsel == 2) ? 1 : 0;
                            } else {
                                lane = (bufsel == 2) ? 2 : 0;
                            }
                            uint32_t idx = (cm << 8) | buf[bpp * i + lane];
                            uint32_t ent = s->cmap[idx];
                            r = (ent >> 24) & 0xff;
                            g = (ent >> 16) & 0xff;
                            b = (ent >> 8) & 0xff;
                        } else if (bpp == 2) {
                            uint16_t p = (buf[2 * i] << 8) | buf[2 * i + 1];
                            r = ((p >> 10) & 0x1f) << 3;
                            g = ((p >> 5) & 0x1f) << 3;
                            b = (p & 0x1f) << 3;
                        } else {
                            r = buf[4 * i];
                            g = buf[4 * i + 1];
                            b = buf[4 * i + 2];
                        }
                    }
                    if (x >= 0 && x < surface_width(surface) &&
                        y < surface_height(surface)) {
                        dst[y * stride + x] = rgb_to_pixel32(r, g, b);
                    }
                }
            }
        }
    }

    /* ---- cursor compositing (spec §2.10) ---- */
    sgi_gbe_composite_cursor(s, surface);

    s->scan_dirty = false;
}

static void sgi_gbe_invalidate(void *opaque)
{
    SGIGBEState *s = opaque;
    if (s->con) {
        sgi_gbe_scanout(s);
        dpy_gfx_update(s->con, 0, 0, surface_width(qemu_console_surface(s->con)),
                       surface_height(qemu_console_surface(s->con)));
    }
}

static void sgi_gbe_update(void *opaque)
{
    SGIGBEState *s = opaque;
    /*
     * Repaint when EITHER channel's DMA is enabled: during the console
     * switch the kernel runs with the overlay channel alone (frm DMA
     * off, ovr on); gating on frm only froze the display.
     */
    if (s->con && ((s->frm_ctrl & 1) || (s->ovr_ctrl & 1))) {
        sgi_gbe_scanout(s);
        dpy_gfx_update(s->con, 0, 0,
                       surface_width(qemu_console_surface(s->con)),
                       surface_height(qemu_console_surface(s->con)));
    }
}

static const GraphicHwOps sgi_gbe_gfx_ops = {
    .invalidate = sgi_gbe_invalidate,
    .gfx_update = sgi_gbe_update,
};




/* --------------------------- DDC/EDID on I2C ---------------------------- */
/*
 * The display's physical size reaches X through the monitor's EDID, which
 * crmQryMonitor (the DDX) and the PROM's crime_i2cMonitorProbe read by
 * bit-banging SDA/SCL in the GBE I2C window.  With no device on the bus the
 * probe fails, the screen gets no mm size (Screen.mwidth == 0), and libXt's
 * SgiResetScheme divides by zero -- SIGTRAP for every Motif app.  So we
 * answer as a DDC2B slave carrying a standard 376x301 mm panel.
 *
 * Register/line convention (crm_i2c.c): the guest writes the COMPLEMENT of
 * the line state and its own I2C_READ macro complements it back, so the
 * register holds the raw value and a read returns it unmodified.  Real bits:
 * 0 = SDA, 1 = SCL, 1 = released/high.  The slave can only pull SDA LOW.
 *
 * Bus model: the master's access pattern is known and fixed
 * (crime_i2cMonitorProbe): START, 0xA0, 0x00, START, 0xA1, then 128 read
 * bytes.  We therefore drive a frame counter on real SCL edges rather than
 * trying to detect START/STOP at the register level -- in this bit-banged
 * protocol the master moves SDA while SCL is high both for bit setup and to
 * release the line for each ACK, which is indistinguishable from a START or
 * STOP and resets the state machine on every bit.
 *
 * Per frame: 9 SCL clocks -- clocks 1..8 carry the data (MSB first), clock 9
 * is the ACK clock.  During a READ frame the master samples SDA on each of
 * the first eight clocks, so the bit index advances at frame start, not
 * frame end, or the whole stream comes out one byte ahead.
 *
 * Verified offline against a transcription of crm_i2c.c before landing:
 * tmp/o2-qemu/m13i/sim.py decodes all 128 bytes bit-exactly.
 */
enum { DDC_ADDR = 0, DDC_OFFSET, DDC_READ };

/* EDID bytes 0x15/0x16 are the max image size in cm; 38x30 is 380x300 mm. */
#define DDC_PANEL_MM_W      376
#define DDC_PANEL_MM_H      301

static void sgi_gbe_ddc_build_edid(SGIGBEState *s)
{
    qemu_edid_info info = {
        .vendor       = "SGI",
        .name         = "O2",
        .width_mm     = DDC_PANEL_MM_W,
        .height_mm    = DDC_PANEL_MM_H,
        .prefx        = 1280,
        .prefy        = 1024,
        .maxx         = 1280,
        .maxy         = 1024,
        .refresh_rate = 60,
    };

    qemu_edid_generate(s->ddc_edid, sizeof(s->ddc_edid), &info);

    /*
     * Established-timings byte 36 ("Established Timings II").  QEMU advertises
     * 1024x768@60 here, which crm_init.c:matchEDIDTiming prefers over the
     * preferred timing descriptor and which would put the display into
     * 1024x768 -- whereas the PROM's no-EDID fallback is hardcoded to
     * 1280x1024@60 (crm_init.c:328) and that is the mode the machine has
     * always come up in.  Advertise 1280x1024@75 (bit 0) instead so both
     * paths agree and this change is purely "the screen now reports a
     * physical size", not a resolution change.
     */
    s->ddc_edid[36] &= ~0x08;   /* clear 1024x768@60 */
    s->ddc_edid[36] |=  0x01;   /* set   1280x1024@75 */
    s->ddc_edid[127] = 0;       /* recompute the checksum */
    for (int i = 0; i < 127; i++) {
        s->ddc_edid[127] = (uint8_t)(s->ddc_edid[127] + s->ddc_edid[i]);
    }
    s->ddc_edid[127] = (uint8_t)(0x100 - s->ddc_edid[127]);

    /* The header is what crime_i2cValidEdid checks. */
    assert(s->ddc_edid[0] == 0x00 && s->ddc_edid[7] == 0x00);
    /* A bad checksum would make the probe fail and drop us back to mwidth 0. */
    {
        int sum = 0;
        for (int i = 0; i < 128; i++) {
            sum += s->ddc_edid[i];
        }
        assert((sum & 0xff) == 0);
    }
}

/*
 * Advance the slave to the given REAL line state and return 1 if the slave
 * pulls SDA low.  Called from BOTH handlers: the master's END of message is
 * synchronised with i2c_sync_clk (a read poll while SCL is high), so the
 * last byte of every sendbyte is only observable on a read -- the machine
 * must therefore run on reads as well as writes.
 */
static int sgi_gbe_ddc_step(SGIGBEState *s, int real)
{
    int sda = real & 1, scl = (real >> 1) & 1;
    int start, rising, falling;

    if (!s->ddc_started) {
        s->ddc_started = true;
        s->ddc_pclk = scl;
        s->ddc_pdat = sda;
        return 0;
    }

    start   = s->ddc_pclk && scl && s->ddc_pdat && !sda;
    rising  = !s->ddc_pclk && scl;
    falling = s->ddc_pclk && !scl;

    if (start) {
        trace_sgi_gbe_i2c_start(s->ddc_phase);
        s->ddc_phase = DDC_ADDR;
        s->ddc_clocks = 0;
        s->ddc_shift = 0;
        s->ddc_off = 0;
        s->ddc_frame_ack = false;
        s->ddc_read_frame = false;
    } else if (rising) {
        s->ddc_clocks++;
        if (s->ddc_clocks == 1) {
            /* Latch at frame start: the master samples the read bits on
             * clocks 1..8 and the offset must be current for all of them. */
            s->ddc_read_frame = (s->ddc_phase == DDC_READ);
        }
        if (s->ddc_clocks <= 8) {
            if (s->ddc_phase != DDC_READ) {
                s->ddc_shift = (uint8_t)((s->ddc_shift << 1) | sda);
            }
            if (s->ddc_clocks == 8) {
                /* Eight data bits in: interpret the frame, decide the ACK. */
                uint8_t b = s->ddc_shift;
                trace_sgi_gbe_i2c_byte(b, s->ddc_phase);
                if (s->ddc_phase == DDC_ADDR) {
                    if (b == 0xa0) {
                        s->ddc_phase = DDC_OFFSET;
                        s->ddc_frame_ack = true;
                    } else if (b == 0xa1) {
                        s->ddc_phase = DDC_READ;
                        s->ddc_off = 0;
                        s->ddc_frame_ack = true;
                    }
                } else if (s->ddc_phase == DDC_OFFSET) {
                    s->ddc_off = b & 0x7f;
                    s->ddc_frame_ack = true;
                }
            }
        }
    } else if (falling) {
        if (s->ddc_clocks >= 9) {
            /* The ACK clock has ended: advance and re-arm for the next frame. */
            if (s->ddc_read_frame) {
                s->ddc_off = (s->ddc_off + 1) & 0x7f;
            }
            s->ddc_clocks = 0;
            s->ddc_shift = 0;
            s->ddc_frame_ack = false;
            s->ddc_read_frame = false;
        }
    }

    s->ddc_pclk = scl;
    s->ddc_pdat = sda;

    /* The slave only drives while SCL is high. */
    if (!scl) {
        return 0;
    }
    if (s->ddc_phase == DDC_READ && s->ddc_clocks >= 1 && s->ddc_clocks <= 8) {
        uint8_t b = s->ddc_edid[s->ddc_off & 0x7f];
        /* SDA is open-drain: the slave can only pull it low. */
        return !((b >> (7 - (s->ddc_clocks - 1))) & 1);
    }
    if (s->ddc_clocks == 9) {
        return s->ddc_frame_ack;
    }
    return 0;
}

static uint64_t sgi_gbe_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIGBEState *s = SGI_GBE(opaque);

    switch (offset) {
    case GBE_CTRLSTAT:
        /* chip ID in the low nibble; we emulate pre-Arsenic GBE (1) */
        return (s->ctrlstat & ~GBE_CTRLSTAT_CHIPID_MASK) | GBE_CHIPID;

    case GBE_DOTCLOCK:
        return s->dotclock;

    case GBE_ID:
        return GBE_ID_VALUE;

    case GBE_I2C: {
        /*
         * Return the RAW register value.  The guest's own I2C_READ macro
         * complements it -- crm_i2c.c:
         *   #define I2C_READ(hwp, buf) gbeGetReg(hwp, i2c, buf); buf = ((~buf) & 3)
         * -- so complementing in the handler too inverted the line state
         * twice.  The visible symptom was that crime_i2cMonitorProbe's reset
         * loop ("while rbuf != I2C_REALVAL(I2C_CLK_HIGH_DATA_HIGH)", i.e.
         * rbuf != 3) never saw the bus idle-high after i2c_stop(), and spun
         * to its retry limit: the PROM/Xsgi always reported "no DDC monitor",
         * leaving Screen.mwidth == 0.
         */
        uint32_t raw = s->i2c;
        /* The DDC slave must run on reads too: the master's sync_clk poll is
         * a read while SCL is high, and it is the only observation point for
         * the last bit of each byte. */
        if (sgi_gbe_ddc_step(s, (~raw) & 3)) {
            raw |= 1;       /* slave pulls SDA low */
        }
        trace_sgi_gbe_i2c_read(s->i2c, raw);
        return raw;
    }

    case GBE_I2CFP:
        return (~s->i2cfp) & 3;

    case GBE_SYSCLK:
        return 0;

    case GBE_VT_XY: {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint32_t xy = sgi_gbe_current_xy(s, now);
        if (s->vt_frozen) {
            xy = s->vt_xy_frozen;
        }
        trace_sgi_gbe_vt_xy_read(xy);
        return s->vt_frozen ? (xy | 0x80000000u) : xy;
    }

    case GBE_VT_XYMAX:
        return s->vt_xymax;

    case GBE_OVR_WIDTH_TILE:
        return s->ovr_width_tile;
    case GBE_OVR_INHWCTRL:
        return s->ovr_inhwctrl;
    case GBE_OVR_CTRL:
        return s->ovr_ctrl;

    case GBE_FRM_SIZE_TILE:
        return s->frm_size_tile;
    case GBE_FRM_SIZE_PIXEL:
        return s->frm_size_pixel;
    case GBE_FRM_INHWCTRL:
        return s->frm_inhwctrl;
    case GBE_FRM_CTRL:
        return s->frm_ctrl;

    case GBE_DID_INHWCTRL:
        return s->did_inhwctrl;
    case GBE_DID_CTRL:
        return s->did_ctrl;

    /* CMAP FIFO: pre-Arsenic GBE — 0x0 means "full OR empty" = ready */
    case GBE_CM_FIFO:
        return 0x0;

    case GBE_CRS_POS:
        return s->crs_pos;
    case GBE_CRS_CTRL:
        return s->crs_ctrl;
    case GBE_CRS_CMAP0:
        return s->crs_cmap[0];
    case GBE_CRS_CMAP1:
        return s->crs_cmap[1];
    case GBE_CRS_CMAP2:
        return s->crs_cmap[2];

    default:
        break;
    }

    /* Video timing registers (0x10008-0x1004C) */
    if (offset >= GBE_VT_VSYNC && offset <= GBE_VT_VCSTARTXY) {
        int idx = GBE_VT_IDX(offset);
        if (idx >= 0 && idx < GBE_VT_REG_COUNT) {
            return s->vt_regs[idx];
        }
    }

    /* WID mode registers (0x48000) */
    if (offset >= GBE_MODE_REGS_BASE &&
        offset < GBE_MODE_REGS_BASE + GBE_MODE_REGS_SIZE * 4) {
        return s->mode_regs[(offset - GBE_MODE_REGS_BASE) / 4];
    }

    /* CMAP (0x50000) */
    if (offset >= GBE_CMAP_BASE &&
        offset < GBE_CMAP_BASE + GBE_CMAP_SIZE * 4) {
        return s->cmap[(offset - GBE_CMAP_BASE) / 4];
    }

    /* GMAP (0x60000) */
    if (offset >= GBE_GMAP_BASE &&
        offset < GBE_GMAP_BASE + GBE_GMAP_SIZE * 4) {
        return s->gmap[(offset - GBE_GMAP_BASE) / 4];
    }

    /* Cursor glyphs (0x78000) */
    if (offset >= GBE_CRS_GLYPH_BASE &&
        offset < GBE_CRS_GLYPH_BASE + GBE_CRS_GLYPH_COUNT * 4) {
        return s->crs_glyph[(offset - GBE_CRS_GLYPH_BASE) / 4];
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_gbe: read at offset 0x%06" HWADDR_PRIx
                  " (size %d)\n", offset, size);
    return 0;
}

static void sgi_gbe_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    SGIGBEState *s = SGI_GBE(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case GBE_CTRLSTAT:
        /* keep the chip ID bits ours; store the rest verbatim so the
         * PROM's (val & 0x020aa000) == 0x020aa000 test round-trips */
        s->ctrlstat = (v & ~GBE_CTRLSTAT_CHIPID_MASK) | GBE_CHIPID;
        return;

    case GBE_DOTCLOCK:
        s->dotclock = v;
        sgi_gbe_update_geometry(s);
        return;

    case GBE_SYSCLK:
        return;     /* write-only PLL, no model needed */

    case GBE_I2C:
        s->i2c = v & 3;
        /* START/STOP are write-only sequences (the master writes the two
         * line states back to back with no read between), so the slave must
         * see writes as well as reads. */
        sgi_gbe_ddc_step(s, (~s->i2c) & 3);
        trace_sgi_gbe_i2c_write(s->i2c);
        return;
    case GBE_I2CFP:
        s->i2cfp = v & 3;
        return;

    case GBE_VT_XY:
        /* bit 31 = freeze; the PROM writes 0x80000000 to freeze and
         * 0 to unfreeze (turnOnGbe polls bit 31 read-back) */
        s->vt_frozen = (v & 0x80000000u) != 0;
        if (s->vt_frozen) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->vt_xy_frozen = sgi_gbe_current_xy(s, now) & 0x7fffffff;
        }
        return;

    case GBE_VT_XYMAX:
        s->vt_xymax = v;
        sgi_gbe_update_geometry(s);
        trace_sgi_gbe_xymax(v);
        return;

    case GBE_VT_INTR01:
        s->vt_intr01 = v;
        return;
    case GBE_VT_INTR23:
        s->vt_intr23 = v;
        return;

    case GBE_OVR_WIDTH_TILE:
        s->ovr_width_tile = v;
        return;
    case GBE_OVR_CTRL:
        s->ovr_ctrl = v;
        return;
    case GBE_OVR_INHWCTRL:
        /* inhwctrl is a read-only hardware view; accept the write */
        return;

    case GBE_FRM_SIZE_TILE:
        s->frm_size_tile = v;
        s->scan_dirty = true;
        return;
    case GBE_FRM_SIZE_PIXEL:
        s->frm_size_pixel = v;
        s->scan_dirty = true;
        return;
    case GBE_FRM_CTRL:
        s->frm_ctrl = v;
        s->scan_dirty = true;
        trace_sgi_gbe_frm_ctrl(v);
        return;
    case GBE_FRM_INHWCTRL:
        return;

    case GBE_DID_CTRL:
        s->did_ctrl = v;
        return;
    case GBE_DID_INHWCTRL:
        return;

    /* Cursor */
    case GBE_CRS_POS:
        s->crs_pos = v;
        s->scan_dirty = true;
        return;
    case GBE_CRS_CTRL:
        s->crs_ctrl = v;
        s->scan_dirty = true;
        return;
    case GBE_CRS_CMAP0:
        s->crs_cmap[0] = v;
        return;
    case GBE_CRS_CMAP1:
        s->crs_cmap[1] = v;
        return;
    case GBE_CRS_CMAP2:
        s->crs_cmap[2] = v;
        return;

    default:
        break;
    }

    /* Video timing registers */
    if (offset >= GBE_VT_VSYNC && offset <= GBE_VT_VCSTARTXY) {
        int idx = GBE_VT_IDX(offset);
        if (idx >= 0 && idx < GBE_VT_REG_COUNT) {
            s->vt_regs[idx] = v;
            sgi_gbe_update_geometry(s);
        }
        return;
    }

    /* WID mode registers */
    if (offset >= GBE_MODE_REGS_BASE &&
        offset < GBE_MODE_REGS_BASE + GBE_MODE_REGS_SIZE * 4) {
        s->mode_regs[(offset - GBE_MODE_REGS_BASE) / 4] = v;
        return;
    }

    /* CMAP: accepted into the array immediately (real HW delays the
     * load to vsync; instant is fine — never clear/modify at vsync) */
    if (offset >= GBE_CMAP_BASE &&
        offset < GBE_CMAP_BASE + GBE_CMAP_SIZE * 4) {
        uint32_t idx = (offset - GBE_CMAP_BASE) / 4;
        s->cmap[idx] = v;
        trace_sgi_gbe_cmap_write(idx, v);
        s->scan_dirty = true;
        return;
    }

    /* GMAP */
    if (offset >= GBE_GMAP_BASE &&
        offset < GBE_GMAP_BASE + GBE_GMAP_SIZE * 4) {
        s->gmap[(offset - GBE_GMAP_BASE) / 4] = v;
        return;
    }

    /* Cursor glyphs */
    if (offset >= GBE_CRS_GLYPH_BASE &&
        offset < GBE_CRS_GLYPH_BASE + GBE_CRS_GLYPH_COUNT * 4) {
        s->crs_glyph[(offset - GBE_CRS_GLYPH_BASE) / 4] = v;
        s->scan_dirty = true;
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_gbe: write at offset 0x%06" HWADDR_PRIx
                  " value 0x%08" PRIx64 " (size %d)\n",
                  offset, value, size);
}

static const MemoryRegionOps sgi_gbe_ops = {
    .read = sgi_gbe_read,
    .write = sgi_gbe_write,
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

/* ------------------------------- lifecycle ------------------------------ */

static void sgi_gbe_reset(DeviceState *dev)
{
    SGIGBEState *s = SGI_GBE(dev);

    /* gxemul default while running: 0x300ae001 (chip ID 1). Reset
     * value before programming: display-off variant so the PROM's
     * turnOffGbeDma "already off" check does not early-return. */
    s->ctrlstat = 0x000ae000 | GBE_CHIPID;
    s->dotclock = 0;
    s->i2c = 0;
    s->i2cfp = 0;

    /* DDC slave: rebuild the EDID and re-arm the bus machine. */
    sgi_gbe_ddc_build_edid(s);
    s->ddc_phase = DDC_ADDR;
    s->ddc_clocks = 0;
    s->ddc_shift = 0;
    s->ddc_off = 0;
    s->ddc_started = false;
    s->ddc_frame_ack = false;
    s->ddc_read_frame = false;
    /* Idle-high: the guest's reset loop spins until a read reports 3. */
    s->ddc_pclk = 1;
    s->ddc_pdat = 1;

    memset(s->vt_regs, 0, sizeof(s->vt_regs));
    s->vt_xymax = 0;
    s->vt_intr01 = 0;
    s->vt_intr23 = 0;
    s->vt_frozen = false;
    s->vt_xy_frozen = 0;

    s->ovr_width_tile = 0;
    s->ovr_ctrl = 0;
    s->ovr_inhwctrl = 0;
    s->frm_size_tile = 0;
    s->frm_size_pixel = 0;
    s->frm_ctrl = 0;
    s->frm_inhwctrl = 0;
    s->did_ctrl = 0;
    s->did_inhwctrl = 0;

    memset(s->mode_regs, 0, sizeof(s->mode_regs));
    memset(s->cmap, 0, sizeof(s->cmap));
    memset(s->gmap, 0, sizeof(s->gmap));
    s->crs_pos = 0;
    s->crs_ctrl = 0;
    memset(s->crs_cmap, 0, sizeof(s->crs_cmap));
    memset(s->crs_glyph, 0, sizeof(s->crs_glyph));

    s->scan_width = 0;
    s->scan_height = 0;
    s->scan_dirty = true;

    /* Default raster geometry until the PROM programs real timing */
    s->htotal = GBE_DEF_HTOTAL;
    s->vtotal = GBE_DEF_VTOTAL;
    s->hblank_start = 0;
    s->hblank_end = 0;
    s->vblank_start = 0;
    s->vblank_end = 0;
    s->refresh_hz = GBE_DEF_REFRESH;
    sgi_gbe_update_geometry(s);

    s->frame_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->frame_timer) {
        timer_del(s->frame_timer);
        timer_mod(s->frame_timer, s->frame_start_ns +
                  NANOSECONDS_PER_SECOND / s->refresh_hz);
    }
}

static void sgi_gbe_realize(DeviceState *dev, Error **errp)
{
    SGIGBEState *s = SGI_GBE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_gbe_ops, s,
                          "sgi-gbe", GBE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Interrupt outputs to CRIME, sysbus irqs 0-3 = GBE0-3
     * (GBE0 = vertical retrace, GBE1 = pre-blank [sys/IP32.h GBE_INTR]). */
    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", 4);

    /* Frame timer drives the raster sweep, the ctrl->inhwctrl vsync
     * latch, the GBE0/GBE1 pulses, and the tile scanout. The irq timer
     * ends the interrupt pulse window (see sgi_gbe_frame_tick). */
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_gbe_frame_tick, s);
    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_gbe_irq_pulse_end, s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / GBE_DEF_REFRESH);

    /* QEMU graphical console (Newport pattern) */
    s->con = graphic_console_init(dev, 0, &sgi_gbe_gfx_ops, s);
    qemu_console_resize(s->con, GBE_DEF_HTOTAL >= 1280 ? 1280 : GBE_DEF_HTOTAL,
                         1024);
}

static const VMStateDescription vmstate_sgi_gbe = {
    .name = "sgi-gbe",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrlstat, SGIGBEState),
        VMSTATE_UINT32(dotclock, SGIGBEState),
        VMSTATE_UINT32(i2c, SGIGBEState),
        VMSTATE_UINT32(i2cfp, SGIGBEState),
        VMSTATE_UINT8_ARRAY(ddc_edid, SGIGBEState, 128),
        VMSTATE_UINT8(ddc_phase, SGIGBEState),
        VMSTATE_UINT8(ddc_clocks, SGIGBEState),
        VMSTATE_UINT8(ddc_shift, SGIGBEState),
        VMSTATE_UINT8(ddc_off, SGIGBEState),
        VMSTATE_UINT8(ddc_pclk, SGIGBEState),
        VMSTATE_UINT8(ddc_pdat, SGIGBEState),
        VMSTATE_BOOL(ddc_started, SGIGBEState),
        VMSTATE_BOOL(ddc_frame_ack, SGIGBEState),
        VMSTATE_BOOL(ddc_read_frame, SGIGBEState),
        VMSTATE_UINT32_ARRAY(vt_regs, SGIGBEState, GBE_VT_REG_COUNT),
        VMSTATE_UINT32(vt_xymax, SGIGBEState),
        VMSTATE_UINT32(vt_intr01, SGIGBEState),
        VMSTATE_UINT32(vt_intr23, SGIGBEState),
        VMSTATE_BOOL(vt_frozen, SGIGBEState),
        VMSTATE_UINT32(vt_xy_frozen, SGIGBEState),
        VMSTATE_UINT32(ovr_width_tile, SGIGBEState),
        VMSTATE_UINT32(ovr_ctrl, SGIGBEState),
        VMSTATE_UINT32(ovr_inhwctrl, SGIGBEState),
        VMSTATE_UINT32(frm_size_tile, SGIGBEState),
        VMSTATE_UINT32(frm_size_pixel, SGIGBEState),
        VMSTATE_UINT32(frm_ctrl, SGIGBEState),
        VMSTATE_UINT32(frm_inhwctrl, SGIGBEState),
        VMSTATE_UINT32(did_ctrl, SGIGBEState),
        VMSTATE_UINT32(did_inhwctrl, SGIGBEState),
        VMSTATE_UINT32_ARRAY(mode_regs, SGIGBEState, GBE_MODE_REGS_SIZE),
        VMSTATE_UINT32_ARRAY(cmap, SGIGBEState, GBE_CMAP_SIZE),
        VMSTATE_UINT32_ARRAY(gmap, SGIGBEState, GBE_GMAP_SIZE),
        VMSTATE_UINT32(crs_pos, SGIGBEState),
        VMSTATE_UINT32(crs_ctrl, SGIGBEState),
        VMSTATE_UINT32_ARRAY(crs_cmap, SGIGBEState, 3),
        VMSTATE_UINT32_ARRAY(crs_glyph, SGIGBEState, GBE_CRS_GLYPH_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_gbe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_gbe_realize;
    device_class_set_legacy_reset(dc, sgi_gbe_reset);
    dc->vmsd = &vmstate_sgi_gbe;
}

static const TypeInfo sgi_gbe_info = {
    .name = TYPE_SGI_GBE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGBEState),
    .class_init = sgi_gbe_class_init,
};

static void sgi_gbe_register_types(void)
{
    type_register_static(&sgi_gbe_info);
}

type_init(sgi_gbe_register_types)
