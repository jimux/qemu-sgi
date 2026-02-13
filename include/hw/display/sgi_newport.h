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
 * References:
 *   - MAME src/devices/bus/gio64/newport.cpp
 *   - Linux arch/mips/sgi-ip22/ip22-gio.c
 *   - NetBSD sys/arch/sgimips/dev/newport.c
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SGI_NEWPORT_H
#define HW_DISPLAY_SGI_NEWPORT_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "ui/console.h"

/* VRAM dimensions — matches MAME layout (1280+64 guard band) */
#define NEWPORT_VRAM_W      1344    /* 1280 + 64 */
#define NEWPORT_VRAM_H      1088    /* 1024 + 64 */
#define NEWPORT_SCREEN_W    1280
#define NEWPORT_SCREEN_H    1024

#define TYPE_SGI_NEWPORT "sgi-newport"
OBJECT_DECLARE_SIMPLE_TYPE(SGINewportState, SGI_NEWPORT)

/*
 * REX3 Register offsets (from GIO slot base 0x1f000000)
 *
 * REX3 registers are at 0x0f0000-0x0f1fff within the GIO graphics slot.
 * They can be accessed as 32-bit or 64-bit; the register layout pairs
 * registers at offsets N+4 and N+0 into 64-bit words.
 *
 * Offsets listed are relative to REX3 base (0x1f0f0000).
 */

/* Drawing registers (0x0000-0x02ff) */
#define REX3_DRAWMODE1          0x0000  /* High 32 bits */
#define REX3_DRAWMODE0          0x0004  /* Low 32 bits */
#define REX3_LSMODE             0x0008  /* Line stipple mode */
#define REX3_LSPATTERN          0x000c  /* Line stipple pattern */
#define REX3_LSPATSAVE          0x0010  /* Line stipple pattern save */
#define REX3_ZPATTERN           0x0014  /* Z pattern */
#define REX3_LSPATUPDATE        0x0038  /* Update LSPATTERN/LSPATSAVE */
#define REX3_COLORBACK          0x0018  /* Background color */
#define REX3_COLORVRAM          0x001c  /* VRAM fastclear color */
#define REX3_ALPHAREF           0x0020  /* Alpha reference */
#define REX3_SMASK0X            0x0028  /* Screenmask 0 X */
#define REX3_SMASK0Y            0x002c  /* Screenmask 0 Y */
#define REX3_SETUP              0x0030  /* Line/span setup */
#define REX3_STEPZ              0x0034  /* Z step */

/* Coordinate registers (0x0100-0x01ff) */
#define REX3_XSTART             0x0100  /* X start (fractional) */
#define REX3_YSTART             0x0104  /* Y start (fractional) */
#define REX3_XEND               0x0108  /* X end (fractional) */
#define REX3_YEND               0x010c  /* Y end (fractional) */
#define REX3_XSAVE              0x0110  /* X save */
#define REX3_XYMOVE             0x0114  /* XY move */
#define REX3_BRESD              0x0118  /* Bresenham D */
#define REX3_BRESS1             0x011c  /* Bresenham S1 */
#define REX3_BRESOCTINC1        0x0120  /* Bresenham octant & incr1 */
#define REX3_BRESRNDINC2        0x0124  /* Bresenham round & incr2 */
#define REX3_BRESE1             0x0128  /* Bresenham E1 */
#define REX3_BRESS2             0x012c  /* Bresenham S2 */
#define REX3_AWEIGHT0           0x0130  /* AA weight 0 */
#define REX3_AWEIGHT1           0x0134  /* AA weight 1 */
#define REX3_XSTARTF            0x0138  /* X start (GL float) */
#define REX3_YSTARTF            0x013c  /* Y start (GL float) */
#define REX3_XENDF              0x0140  /* X end (GL float) */
#define REX3_YENDF              0x0144  /* Y end (GL float) */
#define REX3_XSTARTI            0x0148  /* X start (integer) */
#define REX3_XENDF2             0x014c  /* X end (copy) */
#define REX3_XYSTARTI           0x0150  /* XY start (integer) */
#define REX3_XYENDI             0x0154  /* XY end (integer) */
#define REX3_XSTARTENDI         0x0158  /* X start/end (integer) */

/* Color registers (0x0200-0x02ff) */
#define REX3_COLORRED           0x0200  /* Red/CI color */
#define REX3_COLORALPHA         0x0204  /* Alpha color */
#define REX3_COLORGREEN         0x0208  /* Green color */
#define REX3_COLORBLUE          0x020c  /* Blue color */
#define REX3_SLOPERED           0x0210  /* Red slope */
#define REX3_SLOPEALPHA         0x0214  /* Alpha slope */
#define REX3_SLOPEGREEN         0x0218  /* Green slope */
#define REX3_SLOPEBLUE          0x021c  /* Blue slope */
#define REX3_WRITEMASK          0x0220  /* Write mask */
#define REX3_COLORI             0x0224  /* Packed color fractions */
#define REX3_ZEROOVERFLOW       0x0228  /* Zero overflow */
#define REX3_SLOPEREDCOPY       0x022c  /* Red slope (copy) */
#define REX3_HOSTRW0            0x0230  /* Host data port (high) */
#define REX3_HOSTRW1            0x0234  /* Host data port (low) */
#define REX3_DCBMODE            0x0238  /* DCB mode */
#define REX3_DCBDATA0           0x0240  /* DCB data MSW */
#define REX3_DCBDATA1           0x0244  /* DCB data LSW */

/* Additional registers (0x1300-0x133f) */
#define REX3_SMASK1X            0x1300  /* Screenmask 1 X */
#define REX3_SMASK1Y            0x1304  /* Screenmask 1 Y */
#define REX3_SMASK2X            0x1308  /* Screenmask 2 X */
#define REX3_SMASK2Y            0x130c  /* Screenmask 2 Y */
#define REX3_SMASK3X            0x1310  /* Screenmask 3 X */
#define REX3_SMASK3Y            0x1314  /* Screenmask 3 Y */
#define REX3_SMASK4X            0x1318  /* Screenmask 4 X */
#define REX3_SMASK4Y            0x131c  /* Screenmask 4 Y */
#define REX3_TOPSCAN            0x1320  /* Top scanline */
#define REX3_XYWIN              0x1324  /* XY window */
#define REX3_CLIPMODE           0x1328  /* Clipping mode */
#define REX3_CONFIG             0x1330  /* Configuration */
#define REX3_STATUS             0x1338  /* Status register (high 32) */
#define REX3_STATUS_ALIAS       0x133c  /* Status register (low 32) */
#define REX3_DCBRESET           0x1340  /* Reset DCB bus + flush BFIFO */

/* REX3 memory region size (covers 0x0f0000-0x0f1fff within GIO slot) */
#define REX3_REG_SIZE           0x2000
#define REX3_REG_OFFSET         0x0f0000  /* Offset within GIO slot */

/*
 * Status register bits (offset 0x1338)
 * Read from high 32 bits of 64-bit access at 0x1338
 */
#define REX3_STATUS_VERSION_MASK    0x00000007  /* REX3 version (3 or 4) */
#define REX3_STATUS_GFXBUSY         (1 << 3)    /* Graphics engine busy */
#define REX3_STATUS_BACKBUSY        (1 << 4)    /* Backend busy */
#define REX3_STATUS_VRINT           (1 << 5)    /* Vertical retrace int */
#define REX3_STATUS_VIDEOINT        (1 << 6)    /* Video interrupt */
#define REX3_STATUS_GFIFO_SHIFT     7           /* GFIFO level (6 bits) */
#define REX3_STATUS_GFIFO_MASK      (0x3f << REX3_STATUS_GFIFO_SHIFT)
#define REX3_STATUS_BFIFO_SHIFT     13          /* BFIFO level (5 bits) */
#define REX3_STATUS_BFIFO_MASK      (0x1f << REX3_STATUS_BFIFO_SHIFT)
#define REX3_STATUS_BFIFO_INT       (1 << 18)   /* BFIFO interrupt */
#define REX3_STATUS_GFIFO_INT       (1 << 19)   /* GFIFO interrupt */

/* DCB mode register bits (offset 0x0238)
 * Ref: IRIX mgras_tv.c:82-89, MAME newport.cpp:4274-4305 */
#define DCB_MODE_DATAWIDTH_SHIFT    0           /* bits [1:0] */
#define DCB_MODE_DATAWIDTH_MASK     0x03
#define DCB_MODE_ENDATAPACK         (1 << 2)    /* bit 2 */
#define DCB_MODE_ENCRSINC           (1 << 3)    /* bit 3 — auto-increment CRS */
#define DCB_MODE_REGSEL_SHIFT       4           /* bits [6:4] — CRS */
#define DCB_MODE_REGSEL_MASK        (0x07 << 4)
#define DCB_MODE_CSADDR_SHIFT       7           /* bits [10:7] — slave address */
#define DCB_MODE_CSADDR_MASK        (0x0f << 7)
#define DCB_MODE_ENSYNCACK          (1 << 11)   /* bit 11 */
#define DCB_MODE_ENASYNCACK         (1 << 12)   /* bit 12 */
#define DCB_MODE_CSWIDTH_SHIFT      13          /* bits [17:13] */
#define DCB_MODE_CSWIDTH_MASK       (0x1f << 13)
#define DCB_MODE_CSHOLD_SHIFT       18          /* bits [22:18] */
#define DCB_MODE_CSHOLD_MASK        (0x1f << 18)
#define DCB_MODE_CSSETUP_SHIFT      23          /* bits [27:23] */
#define DCB_MODE_CSSETUP_MASK       (0x1f << 23)
#define DCB_MODE_SWAPENDIAN         (1 << 28)   /* bit 28 */

/* DCB slave addresses */
#define DCB_ADDR_VC2        0   /* Video timing controller */
#define DCB_ADDR_CMAP01     1   /* Both CMAPs */
#define DCB_ADDR_CMAP0      2   /* CMAP 0 */
#define DCB_ADDR_CMAP1      3   /* CMAP 1 */
#define DCB_ADDR_XMAP01     4   /* Both XMAPs */
#define DCB_ADDR_XMAP0      5   /* XMAP 0 */
#define DCB_ADDR_XMAP1      6   /* XMAP 1 */
#define DCB_ADDR_RAMDAC     7   /* RAMDAC */
#define DCB_ADDR_CC1        8   /* CC1 */
#define DCB_ADDR_AB1        9   /* AB1 */
#define DCB_ADDR_I2C        11  /* I2C / panel link */
#define DCB_ADDR_PCD        12  /* Presenter (not present on Indy) */

/* REX3 version for Indy */
#define REX3_VERSION_INDY   3

/* VC2 named register indices */
#define VC2_VIDEO_ENTRY      0x00
#define VC2_CURSOR_ENTRY     0x01
#define VC2_CURSOR_X         0x02
#define VC2_CURSOR_Y         0x03
#define VC2_CUR_CURSOR_X     0x04
#define VC2_DID_ENTRY        0x05
#define VC2_SCANLINE_LEN     0x06
#define VC2_RAM_ADDR         0x07
#define VC2_VT_FRAME_PTR     0x08
#define VC2_VT_LINE_PTR      0x09
#define VC2_VT_LINE_RUN      0x0a
#define VC2_VT_LINE_COUNT    0x0b
#define VC2_CURSOR_TABLE     0x0c
#define VC2_WORK_CURSOR_Y    0x0d
#define VC2_DID_FRAME_PTR    0x0e
#define VC2_DID_LINE_PTR     0x0f
#define VC2_DC_CONTROL       0x10
#define VC2_CONFIG           0x1f

/* DC_CONTROL bits */
#define VC2_DC_ENA_VINTR     (1 << 0)
#define VC2_DC_ENA_DISPLAY   (1 << 1)
#define VC2_DC_ENA_DIDS      (1 << 3)
#define VC2_DC_ENA_CURSOR    (1 << 4)
#define VC2_DC_CURSOR_DISP   (1 << 7)
#define VC2_DC_CURSOR_SIZE64 (1 << 9)

struct SGINewportState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Display console */
    QemuConsole *con;

    /* VRAM — matches MAME layout */
    uint32_t *vram_rgbci;           /* RGB/CI pixel data */
    uint32_t *vram_cidaux;          /* CID/overlay/popup data */
    bool display_dirty;

    /* REX3 drawing registers */
    uint32_t drawmode0;
    uint32_t drawmode1;
    uint32_t ls_mode;
    uint32_t ls_pattern;
    uint32_t ls_pattern_saved;
    uint32_t z_pattern;
    uint32_t color_back;
    uint32_t color_vram;
    uint32_t alpha_ref;
    uint32_t smask_x[5];
    uint32_t smask_y[5];
    uint32_t setup;
    uint32_t step_z;

    /* Decoded DRAWMODE1 fields (cached on write for fast access) */
    uint8_t dm1_planes;             /* bits 2:0 — plane enable */
    uint8_t dm1_drawdepth;          /* bits 4:3 — 0=4,1=8,2=12,3=24 bpp */
    bool    dm1_rwpacked;           /* bit 7 */
    uint8_t dm1_hostdepth;          /* bits 9:8 */
    bool    dm1_rwdouble;           /* bit 10 */
    bool    dm1_swapendian;         /* bit 11 */
    bool    dm1_rgbmode;            /* bit 15 */
    bool    dm1_fastclear;          /* bit 17 */
    bool    dm1_blend;              /* bit 18 */
    uint8_t dm1_logicop;            /* bits 31:28 */

    /* Decoded DRAWMODE0 fields */
    bool    dm0_colorhost;          /* bit 6 — pixel color from host data */
    uint8_t dm0_adrmode;            /* bits 4:2 — address mode */
    bool    dm0_stoponx;            /* bit 8 */
    bool    dm0_stopony;            /* bit 9 */
    bool    dm0_zpattern;           /* bit 12 — use z_pattern as source */
    bool    dm0_lspattern;         /* bit 13 — use ls_pattern as source */
    bool    dm0_opaque;             /* bit 16 OR bit 17 — draw color_back */

    /* Coordinate registers */
    int32_t x_start, y_start;
    int32_t x_end, y_end;
    int32_t x_save;
    uint32_t xy_move;
    uint32_t bres_d, bres_s1;
    uint32_t bres_octant_inc1;
    uint32_t bres_round_inc2;
    uint32_t bres_e1, bres_s2;
    uint32_t a_weight0, a_weight1;
    uint32_t x_start_f, y_start_f;
    uint32_t x_end_f, y_end_f;
    int32_t x_start_i;
    uint32_t xy_start_i;
    uint32_t xy_end_i;
    uint32_t x_start_end_i;

    /* Integer coordinate state (derived from register writes) */
    int16_t iter_x, iter_y;        /* Current drawing position */
    int16_t x_start_int, y_start_int;
    int16_t x_end_int, y_end_int;
    int16_t x_save_int;

    /* Color registers */
    uint32_t color_red;
    uint32_t color_alpha;
    uint32_t color_green;
    uint32_t color_blue;
    uint32_t curr_color_red;        /* Current shading accumulators */
    uint32_t curr_color_alpha;
    uint32_t curr_color_green;
    uint32_t curr_color_blue;
    int32_t slope_red;
    int32_t slope_alpha;
    int32_t slope_green;
    int32_t slope_blue;
    uint32_t write_mask;
    uint32_t color_i;
    uint32_t zero_overflow;
    uint64_t host_dataport;

    /* Host data port shift tracking */
    uint32_t host_shift;

    /* Global write mask (0xff for XL8, 0xffffff for XL24) */
    uint32_t global_mask;

    /* DCB (Display Control Bus) */
    uint32_t dcb_mode;
    uint32_t dcb_data_msw;
    uint32_t dcb_data_lsw;

    /* Screenmask and clipping */
    uint32_t top_scanline;
    uint32_t xy_window;
    uint32_t clip_mode;

    /* Config and status */
    uint32_t config;
    uint32_t status;

    /* VC2 (Video Controller 2) */
    uint32_t vc2_ram_addr;
    uint32_t vc2_reg_idx;
    uint32_t vc2_reg_data;
    uint16_t vc2_ram[32768];        /* 32K x 16-bit SRAM */
    uint16_t vc2_reg[32];           /* Named registers */

    /* XMAP9 */
    uint32_t xmap_config;
    uint32_t xmap_revision;
    uint32_t xmap_mode_table[32];   /* 32 mode table entries */
    uint8_t xmap_cursor_cmap;       /* CRS 3 — cursor colormap MSB */
    uint8_t xmap_popup_cmap;        /* CRS 4 — popup colormap MSB */
    uint8_t xmap_mode_table_idx;    /* CRS 7 — mode table byte index */

    /* CMAP — 8192 entries (13-bit index: 5-bit MSB from XMAP + 8-bit CI) */
    uint32_t cmap_revision;
    uint16_t cmap_palette_idx;
    uint32_t cmap0_palette[8192];   /* MAME: palette_entries() returns 0x2000 */

    /* DCB timeout timer — clears BACKBUSY after bus timeout */
    QEMUTimer *dcb_timeout_timer;

    /* VBLANK / vertical retrace interrupt */
    QEMUTimer *vblank_timer;
    QEMUTimer *vrint_deassert_timer; /* Deasserts IRQ after VBLANK pulse */
    bool vrint_active;               /* True while VBLANK IRQ is asserted */
    qemu_irq irq;

    /* RAMDAC (Bt445) state */
    uint8_t ramdac_lut_index;
    uint32_t ramdac_lut_r[256];
    uint32_t ramdac_lut_g[256];
    uint32_t ramdac_lut_b[256];

    /* NewView binary logger — records every REX3 register access */
    char *newview_log_path;
    FILE *newview_log_file;
};

#endif /* HW_DISPLAY_SGI_NEWPORT_H */
