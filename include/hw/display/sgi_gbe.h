/*
 * SGI GBE (Graphics Back End) emulation
 *
 * GBE is the graphics framebuffer controller in the SGI O2 (IP32).
 * It handles video timing, tile-based framebuffer, colormap, and cursor.
 *
 * Physical base: 0x16000000 (kseg1: 0xB6000000)
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SGI_GBE_H
#define HW_DISPLAY_SGI_GBE_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_GBE "sgi-gbe"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGBEState, SGI_GBE)

#define GBE_REG_SIZE    0x100000    /* 1MB region */

/*
 * GBE register offsets
 *
 * Each DMA channel has two register offsets:
 *   - INHWCTRL (read): returns the value currently active in hardware
 *   - CTRL (write): sets the value to take effect at next VSync
 *
 * The PROM's crime_gbe.h names the INHWCTRL register "control" because
 * the PROM only reads these to check DMA status. The real CTRL (write)
 * register is at the adjacent offset, used by the kernel's crmInitTp().
 *
 * Control block (0x00000-0x0001F):
 *   0x00 ctrlstat    - Control/status (chip ID in low nibble)
 *   0x04 dotclock    - Dot clock PLL
 *   0x08 i2c         - I2C (DDC/EDID)
 *   0x0C sysclk      - System clock PLL
 *   0x10 i2cfp       - I2C flat panel (bit 0=SDA, bit 1=SCL)
 *   0x14 id          - Device ID/revision
 *
 * Video timing (0x10000-0x1FFFF):
 *   0x10000 vt_xy      - Current dot coordinates
 *   0x10004 vt_xymax   - Maximum dot coordinates
 *
 * DMA channels:
 *   OVR (overlay):   0x20000-0x2FFFF
 *     0x20000 ovr_width_tile
 *     0x20004 ovr_inhwctrl  (read: hardware state)
 *     0x20008 ovr_ctrl      (write: software command)
 *   FRM (frame):     0x30000-0x3FFFF
 *     0x30000 frm_size_tile
 *     0x30004 frm_size_pixel
 *     0x30008 frm_inhwctrl  (read: hardware state)
 *     0x3000C frm_ctrl      (write: software command)
 *   DID (display ID): 0x40000-0x4FFFF
 *     0x40000 did_inhwctrl  (read: hardware state)
 *     0x40004 did_ctrl      (write: software command)
 */
#define GBE_CTRLSTAT        0x000000
#define GBE_DOTCLOCK        0x000004
#define GBE_I2C             0x000008
#define GBE_SYSCLK          0x00000C
#define GBE_I2CFP           0x000010
#define GBE_ID              0x000014

#define GBE_VT_XY           0x010000
#define GBE_VT_XYMAX        0x010004

#define GBE_OVR_WIDTH_TILE  0x020000
#define GBE_OVR_INHWCTRL    0x020004
#define GBE_OVR_CTRL        0x020008

#define GBE_FRM_SIZE_TILE   0x030000
#define GBE_FRM_SIZE_PIXEL  0x030004
#define GBE_FRM_INHWCTRL    0x030008
#define GBE_FRM_CTRL        0x03000C

/*
 * Video timing registers (0x10000-0x1004C)
 * These control sync, blanking, and pixel enable timing.
 */
#define GBE_VT_VSYNC         0x010008
#define GBE_VT_HSYNC         0x01000C
#define GBE_VT_VBLANK        0x010010
#define GBE_VT_HBLANK        0x010014
#define GBE_VT_FLAGS         0x010018
#define GBE_VT_F2RF_LOCK     0x01001C
#define GBE_VT_INTR01        0x010020
#define GBE_VT_INTR23        0x010024
#define GBE_VT_HPIXEN        0x010028
#define GBE_VT_VPIXEN        0x01002C
#define GBE_VT_HCMAP         0x010030
#define GBE_VT_VCMAP         0x010034
#define GBE_VT_DIDSTARTXY    0x010038
#define GBE_VT_CRSSTARTXY    0x01003C
#define GBE_VT_VCSTARTXY     0x010040
#define GBE_VT_VCENDXY       0x010044
#define GBE_VT_HPIXENF      0x010048
#define GBE_VT_VPIXENF      0x01004C

/* DID channel (0x40000-0x4FFFF) */
#define GBE_DID_INHWCTRL    0x040000
#define GBE_DID_CTRL        0x040004

/* Mode registers (0x48000-0x4807F) — 32 entries at 4-byte spacing */
#define GBE_MODE_REGS_BASE  0x048000
#define GBE_MODE_REGS_SIZE  32

/* CMAP (color map) entries (0x50000-0x54800) — up to 0x1200 entries */
#define GBE_CMAP_BASE       0x050000
#define GBE_CMAP_SIZE       0x1200

/* CMAP FIFO status (0x58000) */
#define GBE_CM_FIFO         0x058000

/* GMAP (gamma map) entries (0x60000-0x603FF) — 256 entries */
#define GBE_GMAP_BASE       0x060000
#define GBE_GMAP_SIZE       256

/* Cursor registers (0x70000-0x70010) */
#define GBE_CRS_POS         0x070000
#define GBE_CRS_CTRL        0x070004
#define GBE_CRS_CMAP0       0x070008
#define GBE_CRS_CMAP1       0x07000C
#define GBE_CRS_CMAP2       0x070010

/* GBE device ID value (from crime_gbe.h) */
#define GBE_ID_VALUE        0x00000666

/* Number of video timing registers */
#define GBE_VT_REG_COUNT    18

struct SGIGBEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* DMA channel control registers */
    uint32_t frm_control;
    uint32_t ovr_control;
    uint32_t did_control;
    uint32_t ctrlstat;
    uint32_t dotclock;

    /* Video timing simulation */
    uint32_t vt_xy;          /* Current raster position */
    uint32_t vt_xymax;       /* Maximum raster coords */
    bool vt_frozen;          /* Bit 31 of VT_XY freezes counter */
    uint32_t vt_read_count;  /* Counter for simulating raster sweep */

    /* Video timing registers (vsync, hsync, blanking, pixel enable, etc.) */
    uint32_t vt_regs[GBE_VT_REG_COUNT];

    /* Mode registers */
    uint32_t mode_regs[GBE_MODE_REGS_SIZE];

    /* CMAP entries */
    uint32_t cmap[GBE_CMAP_SIZE];

    /* GMAP entries */
    uint32_t gmap[GBE_GMAP_SIZE];

    /* Cursor registers */
    uint32_t crs_pos;
    uint32_t crs_ctrl;
    uint32_t crs_cmap[3];
};

#endif /* HW_DISPLAY_SGI_GBE_H */
