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
                        if (bpp == 1) {
                            /*
                             * I8 pixels index the cmap through the WID's CM
                             * field: spec §2.7 Color map — "I8 pixels are
                             * concatenated with the DID cm field to produce a
                             * 13 bit color index : cm(4 downto 0) & i8(7
                             * downto 0)" (gbedefs.h GBE_WID_CM_SHIFT 5,
                             * GBE_WID_CM_MASK 0x3e0). With the DID stream's
                             * DMA off (the long-standing observed state,
                             * did_ctrl bit16=0) there is no per-pixel DID, so
                             * every pixel takes WID 0 / mode_regs[0]. The
                             * PROM programs all 32 WIDs I8/cm=0
                             * (initFramebuffer), where this reduces to the
                             * old cmap[pixel] lookup — behavior preserved.
                             * Xsgi installs its colormap at cm=16
                             * (cmap[4096+pixel], WID1=0x203); when its 8bpp
                             * pixels flow, they now decode through the window
                             * the guest actually loaded instead of window 0.
                             */
                            uint32_t wid = s->mode_regs[0];
                            uint32_t cm = (wid >> 5) & 0x1f;
                            uint32_t idx = (cm << 8) | buf[i];
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

    case GBE_I2C:
        /* open-drain lines idle-high, no DDC device: invert low 2 bits
         * of the last written value (crm_i2c.c I2C_READ semantics) */
        return (~s->i2c) & 3;

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
