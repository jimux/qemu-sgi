/*
 * SGI GBE (Graphics Back End) emulation
 *
 * GBE is the graphics framebuffer controller in the SGI O2 (IP32).
 * It handles video timing, tile-based framebuffer, colormap, and cursor.
 *
 * Physical base: 0x16000000 (kseg1: 0xB6000000)
 *
 * Register offsets cross-verified against three independent sources:
 *   - PROM prom-building/include/ip32/sys/crime_gbe.h (struct gbechip)
 *   - IRIX arcs gbedefs.h + crm_init.c's gbeSetTimingRegs write order
 *   - NetBSD crmfbreg.h (via gxemul-0.7.0 thirdparty)
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SGI_GBE_H
#define HW_DISPLAY_SGI_GBE_H

#include "ui/console.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_GBE "sgi-gbe"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGBEState, SGI_GBE)

#define GBE_REG_SIZE    0x100000    /* 1MB region */

/*
 * DMA channel CTRL/INHWCTRL split (verified correct — NetBSD crmfbreg.h
 * via gxemul agrees):
 *   OVR: width_tile 0x20000, inhwctrl 0x20004 (read), ctrl 0x20008 (write)
 *   FRM: size_tile  0x30000, size_pixel 0x30004,
 *        inhwctrl  0x30008 (read), ctrl 0x3000C (write)
 *   DID: inhwctrl  0x40000 (read), ctrl 0x40004 (write)
 * On real hardware CTRL propagates to INHWCTRL at VSync (DMA-enable bit
 * included — the PROM's initTiming spins on frm_inhwctrl bit 0 until set).
 */

/* Control block (0x00000-0x0001F) */
#define GBE_CTRLSTAT        0x000000
#define GBE_DOTCLOCK       0x000004
#define GBE_I2C            0x000008
#define GBE_SYSCLK         0x00000C
#define GBE_I2CFP          0x000010
#define GBE_ID             0x000014

/* Video timing page (0x10000-0x1004C) — matches crime_gbe.h struct order.
 * vt_xy read layout (NetBSD crmfbreg.h, PROM crm_init.c waitForBlanking):
 * X = bits [11:0], Y = bits [23:12], bit 31 = freeze. vt_xymax uses its
 * own encoding (vtotal<<12)|htotal. */
#define GBE_VT_XY           0x010000
#define GBE_VT_XYMAX        0x010004
#define GBE_VT_VSYNC       0x010008
#define GBE_VT_HSYNC        0x01000C
#define GBE_VT_VBLANK       0x010010
#define GBE_VT_HBLANK       0x010014
#define GBE_VT_FLAGS        0x010018
#define GBE_VT_F2RF_LOCK    0x01001C
#define GBE_VT_INTR01       0x010020
#define GBE_VT_INTR23       0x010024
#define GBE_FP_HDRV         0x010028
#define GBE_FP_VDRV         0x01002C
#define GBE_FP_DE           0x010030
#define GBE_VT_HPIXEN       0x010034
#define GBE_VT_VPIXEN       0x010038
#define GBE_VT_HCMAP        0x01003C
#define GBE_VT_VCMAP        0x010040
#define GBE_VT_DIDSTARTXY   0x010044
#define GBE_VT_CRSSTARTXY   0x010048
#define GBE_VT_VCSTARTXY    0x01004C

/* Index of a VT-page register inside s->vt_regs[] (0x10008-based array) */
#define GBE_VT_IDX(off)     (((off) - GBE_VT_VSYNC) / 4)
#define GBE_VT_REG_COUNT    18  /* 0x10008 .. 0x1004C */

/* OVR (overlay) channel */
#define GBE_OVR_WIDTH_TILE  0x020000
#define GBE_OVR_INHWCTRL    0x020004
#define GBE_OVR_CTRL        0x020008

/* FRM (frame) channel */
#define GBE_FRM_SIZE_TILE   0x030000
#define GBE_FRM_SIZE_PIXEL  0x030004
#define GBE_FRM_INHWCTRL    0x030008
#define GBE_FRM_CTRL        0x03000C

/* DID (display ID) channel */
#define GBE_DID_INHWCTRL    0x040000
#define GBE_DID_CTRL        0x040004

/* Mode registers / WIDs (0x48000-0x4807F) — 32 entries */
#define GBE_MODE_REGS_BASE  0x048000
#define GBE_MODE_REGS_SIZE  32

/* CMAP (0x50000 + 4*4608 bytes) */
#define GBE_CMAP_BASE       0x050000
#define GBE_CMAP_SIZE       4608

/* CMAP FIFO status (0x58000) — 0x0 = "full OR empty" on pre-Arsenic GBE */
#define GBE_CM_FIFO         0x058000

/* GMAP / gamma (0x60000, 256 entries) */
#define GBE_GMAP_BASE       0x060000
#define GBE_GMAP_SIZE       256

/* Cursor (0x70000) + glyph table (0x78000, 64 words = 32x32 @ 2bpp) */
#define GBE_CRS_POS         0x070000
#define GBE_CRS_CTRL        0x070004
#define GBE_CRS_CMAP0       0x070008
#define GBE_CRS_CMAP1       0x07000C
#define GBE_CRS_CMAP2       0x070010
#define GBE_CRS_GLYPH_BASE  0x078000
#define GBE_CRS_GLYPH_COUNT 64

/* GBE device ID value (from crime_gbe.h GBE_ID) */
#define GBE_ID_VALUE        0x00000666

/* Default raster: 1280x1024@60 (crm_timing.h CRM_PROM_VT_1280_1024_60) —
 * the PROM's fallback when no EDID and no `monitor` env var. */
#define GBE_DEF_HTOTAL      1680
#define GBE_DEF_VTOTAL      1065
#define GBE_DEF_REFRESH     60

/* GBE chip ID in ctrlstat low nibble — we emulate pre-Arsenic GBE (1) */
#define GBE_CHIPID          1

/* ctrlstat video-enable related bits used by the PROM (crm_init.c):
 *  0x300aa000 = display on (written by gbeSetTimingRegs / CrmTpBlankscreen)
 *  0x020aa000 = the "is the display on?" test mask
 *  0x000aa000 = blank (crm_tp.c CrmTpBlankscreen mode!=0)
 */
#define GBE_CTRLSTAT_CHIPID_MASK    0xF
#define GBE_CTRLSTAT_DISP_ON        0x30000000u

struct SGIGBEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Interrupt outputs to CRIME (GBE0 = retrace, GBE1 = preblank,
     * GBE2/GBE3). Driven as level pulses by the frame timer. */
    qemu_irq crime_irq[4];

    /* QEMU graphical console */
    QemuConsole *con;

    /* Frame (raster) timer */
    QEMUTimer *frame_timer;
    int64_t frame_start_ns;     /* start of the current frame sweep */

    /* Control block */
    uint32_t ctrlstat;
    uint32_t dotclock;
    uint32_t i2c;
    uint32_t i2cfp;

    /* Video timing */
    uint32_t vt_xymax;      /* (vtotal<<12)|htotal as written by the PROM */
    uint32_t vt_intr01;
    uint32_t vt_intr23;
    bool vt_frozen;         /* bit 31 written to vt_xy */
    uint32_t vt_xy_frozen;  /* latched position while frozen */
    uint32_t vt_regs[GBE_VT_REG_COUNT]; /* vsync..vc_start_xy */

    /* Derived raster geometry (from vt_regs + vt_xymax) */
    uint32_t htotal, vtotal;
    uint32_t hblank_start, hblank_end;
    uint32_t vblank_start, vblank_end;
    uint32_t refresh_hz;

    /* DMA channels: CTRL (written) latches to INHWCTRL at vsync */
    uint32_t ovr_width_tile, ovr_ctrl, ovr_inhwctrl;
    uint32_t frm_size_tile, frm_size_pixel, frm_ctrl, frm_inhwctrl;
    uint32_t did_ctrl, did_inhwctrl;

    /* WID mode registers */
    uint32_t mode_regs[GBE_MODE_REGS_SIZE];

    /* CMAP entries (raw 0xRRGGBB00, decoded r[31:24] g[23:16] b[15:8]) */
    uint32_t cmap[GBE_CMAP_SIZE];

    /* GMAP entries */
    uint32_t gmap[GBE_GMAP_SIZE];

    /* Cursor */
    uint32_t crs_pos;
    uint32_t crs_ctrl;
    uint32_t crs_cmap[3];
    uint32_t crs_glyph[GBE_CRS_GLYPH_COUNT];

    /* Last latched DMA geometry, for scanout */
    int scan_width, scan_height;   /* active pixels */
    int scan_dirty;
};

#endif /* HW_DISPLAY_SGI_GBE_H */
