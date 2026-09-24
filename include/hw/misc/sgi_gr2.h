/*
 * SGI GR2 / "Express" graphics (Indy XZ, Indigo XS/XZ, Indigo2, Everest) —
 * register/FIFO shell.
 *
 * GR2 is SGI's second-generation workstation graphics family: a host-queue
 * ASIC (HQ2) feeding 1-8 microcode-programmed GE7 geometry engines, whose
 * output a RE3 raster engine displays through a VC1 video controller, five
 * XMAP5 mode generators and three BT457 RAMDACs.  It sits on GIO64: board 0
 * at 0x1f000000, board 1 at 0x1f400000.  The software contract is the
 * token-addressed command FIFO (fifo[token] = data) plus the shared-RAM
 * mailbox, not raw GE/RE register writes; the kernel's stock `gr2` driver
 * probes the board by reading the HQ2 presence magic 0xdeadbeef at 0x6a07c.
 *
 * Presence is a device property, default OFF: with present=off the region is
 * not mapped at all, so a plain `-M indy` invocation is byte-for-byte
 * unchanged (the GIO slot still reads all-ones).  present=on maps the board
 * shell so the PROM/kernel GR2 probe recognises the part.
 *
 * Register map is the authoritative `struct gr2_hw` in sys/gr2hw.h (5.3
 * headers) cross-checked against the 6.5 driver binary (gr2.a).  See
 * progress_notes/gfx_drivers_re/03-gr2-express.md.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_GR2_H
#define HW_MISC_SGI_GR2_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"
#include "qemu/timer.h"

#define TYPE_SGI_GR2 "sgi-gr2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGr2State, SGI_GR2)

/* Scanout geometry for the P0.4 step-(a) synthetic test (matches Newport so a
 * later region-by-region comparison against it is size-compatible). */
#define SGI_GR2_SCREEN_W 1280
#define SGI_GR2_SCREEN_H 1024

/* Byte offsets within the GIO64 board window (gr2hw.h).  The board map is
 * 0x6d000 bytes (GR2_GFX_SIZE); round the aperture up to 0x70000. */
#define SGI_GR2_REG_SIZE    0x70000

#define SGI_GR2_SHRAM_OFF   0x00000 /* 128 KB shared command/constant RAM */
#define SGI_GR2_FIFO_OFF    0x40000 /* 128 KB token-addressed command FIFO */
#define SGI_GR2_FIFO_FLUSH_MS 100   /* pause before an unterminated op is run */

/* VC1: an addressed register file plus SRAM, which drives the hardware cursor.
 * addrlo/addrhi hold a byte address; cmd0 is the register data port and sram the
 * bitmap port, both 16-bit and auto-incrementing by 2.  Gr2PositionCursor writes
 * 0x22/0x24 (x/y) through cmd0; Gr2LoadVC1SRAM streams the cursor bitmap through
 * sram.  The sprite is composited at scanout, never written into the framebuffer. */
#define SGI_GR2_VC1_CMD0    0x6c040
#define SGI_GR2_VC1_SRAM    0x6c048
#define SGI_GR2_VC1_TESTREG 0x6c04c
#define SGI_GR2_VC1_ADDRLO  0x6c050
#define SGI_GR2_VC1_ADDRHI  0x6c054
#define SGI_GR2_VC1_SYSCTL  0x6c058
#define SGI_GR2_VC1_REG_WORDS  128 /* byte address >> 1                       */
#define SGI_GR2_VC1_SRAM_WORDS 8192
#define SGI_GR2_VC1_CURSOR_ADDR 0x0a00 /* byte address of the cursor bitmap   */
#define SGI_GR2_VC1_CURSOR_W    16     /* 16x16, 2 bpp                        */
/* Raster backporch for 1280x1024 H60 (gr2hw.h GR2_CURS_*OFF_1280): the cursor
 * registers are raster coordinates, so the visible position is reg minus these. */
#define SGI_GR2_VC1_CURS_XOFF   250
#define SGI_GR2_VC1_CURS_YOFF   35
#define SGI_GR2_HQUCODE_OFF 0x60000 /* 8 K x 32-bit HQ2 microcode RAM */
#define SGI_GR2_GE_OFF      0x68000 /* GE7 units: 8 x 1 KB RAM0, 0x400 stride */
#define SGI_GR2_GE_STRIDE   0x400
#define SGI_GR2_GE_UNITS    8

/* GE7 3D command ports in the token FIFO (offset = FIFO base + token*4).  The
 * vocabulary is the immediate-mode IRIS GL path libgl emits; names and token
 * numbers are read off libgl's own store sites (progress_notes/indy/xz-gr2/74,
 * 76) and confirmed against the captured powerflip stream:
 *   60   gl_g_viewport : x on the token, then (y,w,h) as three PUC_DATA words
 *   53   gl_i_n3f      : unit normal, three floats
 *   2659 gl_i_v3f      : object-space vertex, three floats
 *   55/56/57           : modelview / projection / texture matrix slots
 *   420/1454           : bgnpolygon / bgnpolygon_b
 *   65/1125            : endpolygon / endpolygon_b
 * A polygon is a triangle fan of object-space vertices the GE7 transforms by
 * projection*modelview and rasterises into the framebuffer. */
#define SGI_GR2_GE7_VIEWPORT 0x400f0 /* token 60                          */
#define SGI_GR2_GE7_NORMAL   0x400d4 /* token 53                          */
#define SGI_GR2_GE7_MV       0x400dc /* token 55, 16 floats               */
#define SGI_GR2_GE7_SINGLE   0x400d8 /* token 54, 16 floats: the MSINGLE  */
                                     /* combined (projection*viewing)     */
                                     /* matrix.  IRIS GL puts the whole    */
                                     /* transform here when mmode=MSINGLE, */
                                     /* which is what ideas' line/tmesh    */
                                     /* pass uses (note 90).               */
#define SGI_GR2_GE7_PROJ     0x400e0 /* token 56, 16 floats               */
#define SGI_GR2_GE7_TEX      0x400e4 /* token 57, texture matrix          */
#define SGI_GR2_GE7_VTX      0x4298c /* token 2659, three floats          */
#define SGI_GR2_GE7_BGN      0x40690 /* token 420                         */
#define SGI_GR2_GE7_BGN_B    0x416b8 /* token 1454                        */
#define SGI_GR2_GE7_END      0x40104 /* token 65                          */
#define SGI_GR2_GE7_END_B    0x41194 /* token 1125                        */
#define SGI_GR2_GE7_CLEAR    0x40278 /* token 158, gl_clear               */
#define SGI_GR2_GE7_WINRECT  0x40794 /* token 485, the GL window's screen  */
                                     /* rect (x, y_bottom, w, h), re-sent  */
                                     /* by the DDX each frame so it tracks */
                                     /* a window move (note 79)            */
#define SGI_GR2_GE7_MAX_VERTS 64     /* vertices buffered per polygon     */

/* Line and triangle-mesh primitives, from the libgl disassembly: gl_i_bgnline
 * writes 380 and 1110, gl_i_endline writes 87, gl_i_bgntmesh writes 70 and
 * 1095, gl_i_endtmesh writes 74, gl_i_zclear writes 159.  The line vertex is
 * gl_v2f/gl_v2i (token 4707) carrying two coordinates; the mesh vertex is the
 * ordinary position token 2659, with no normals in the stream. */
#define SGI_GR2_GE7_BGNLINE    0x405f0 /* token 380                        */
#define SGI_GR2_GE7_BGNLINE_B  0x41158 /* token 1110                       */
#define SGI_GR2_GE7_ENDLINE    0x4015c /* token 87                         */
#define SGI_GR2_GE7_V2F        0x4498c /* token 4707, two coordinates      */
#define SGI_GR2_GE7_BGNTMESH   0x40118 /* token 70                         */
#define SGI_GR2_GE7_BGNTMESH_B 0x4111c /* token 1095                       */
#define SGI_GR2_GE7_ENDTMESH   0x40128 /* token 74                         */
#define SGI_GR2_GE7_ZCLEAR     0x4027c /* token 159                        */
#define SGI_GR2_GE7_MAX_LVERTS 256    /* line vertices buffered            */

/* Phong material/light state, one float per token write (the client repeats
 * the token for each component).  See note 75: 117 ambient_sum, 118/119
 * emission, 120/121 ambient, 122/123 diffuse (+alpha), 124/125 specular,
 * 126 lcolor, 127 lightpos, 129 lmcolor.  The 12x pair are the front/back
 * material sides; we shade the front. */
#define SGI_GR2_GE7_AMBIENT_SUM 0x401d4 /* token 117, RGB                  */
#define SGI_GR2_GE7_EMISSION    0x401d8 /* token 118, RGB                  */
#define SGI_GR2_GE7_AMBIENT     0x401e0 /* token 120, RGB                  */
#define SGI_GR2_GE7_DIFFUSE     0x401e8 /* token 122, RGBA                 */
#define SGI_GR2_GE7_SPECULAR    0x401f0 /* token 124, RGB                  */
#define SGI_GR2_GE7_LCOLOR      0x401f8 /* token 126, light colour RGB     */
#define SGI_GR2_GE7_LPOS        0x401fc /* token 127, light position XYZ   */
#define SGI_GR2_GE7_LMCOLOR     0x40204 /* token 129, lighting model       */

#define SGI_GR2_HQ_OFF      0x6a000 /* HQ2 register block (mystery at 0x7c) */
#define SGI_GR2_HQ_MYSTERY  0x6a07c /* presence magic, read by Gr2Probe */
#define SGI_GR2_HQ_MAGIC    0xdeadbeefu

/* HQ2 registers, from the 6.5 gr2_init.o disassembly (Gr2FIFOHandler,
 * _Gr2HQ2RegInit, Gr2DownloadHQ2/GE7).  The FIFO status at 0x6a040 carries
 * the current occupancy in bits 6..12 (mask 0x1fc0) and an error flag in bit
 * 5; the driver polls it before/while draining the token FIFO, so it must read
 * as read-only and empty (0), not as whatever the bus last wrote. */
#define SGI_GR2_HQ_FIFOSTAT    0x6a040
#define SGI_GR2_HQ_IDLE_BIT    0x1     /* bit 0: HQ2 idle/ready (Xsgi polls) */
#define SGI_GR2_HQ_READY_BIT   0x2     /* bit 1: ucode ready (polled)       */
#define SGI_GR2_HQ_TOKEN_START 0x4077c /* FIFO token Gr2Start writes to run */
#define SGI_GR2_RE3_COLOUR_TOKEN 0x40530 /* FIFO token: RE3 pixel colour    */
#define SGI_GR2_PUC_COLOR_TOKEN   0x40648 /* FIFO token: PUC_COLOR (index 402) */
#define SGI_GR2_PUC_RECTI2D_TOKEN 0x40654 /* FIFO token: PUC_RECTI2D (405)    */
/* Fill-op markers, read off the X DDX's own FIFO stores (token = offset/4;
 * see progress_notes/indy/xz-gr2/25-*.md and 26-*.md).  Which shape an op
 * draws is named by the marker token it writes, not by the PUC_DATA tail. */
#define SGI_GR2_RE3_SOLID_TOKEN   0x404c0 /* token 304: expDrawSolidRects    */
#define SGI_GR2_RE3_SPANS_TOKEN   0x404c4 /* token 305: expSolidSpans        */
#define SGI_GR2_RE3_MODE_TOKEN    0x404d4 /* token 309: GC/ROP mode per sub-op */
#define SGI_GR2_RE3_LINE_TOKEN    0x40564 /* token 345: expSegmentSS/expLineSS */
#define SGI_GR2_RE3_SPANSTIP_TOKEN 0x4056c /* token 347: expStippledSpans    */
#define SGI_GR2_RE3_POLY_TOKEN    0x404b8 /* token 302: libgd filled polygon */
#define SGI_GR2_RE3_MONO_TOKEN    0x404e0 /* token 312: expDrawMonoImage     */
#define SGI_GR2_RE3_PEN_TOKEN     0x40574 /* token 349: glyph pen x          */
#define SGI_GR2_RE3_PEN_MAX       64      /* pens recorded per sub-op        */
/* IP20 Xsgi's expDrawMonoImage carries the origin ON the wire (the XZ DDX does
 * not): token 0x40510 is the glyph pen x, the three PUC_DATA words that follow
 * are y, w, h, and the MONO token (312) value is the glyph foreground colour.
 * Token 0x404dc (== the running RE3 colour) is the cell BACKGROUND, not the
 * glyph — a controlled -fg/-bg experiment proved which is which.  The bitmap is
 * two rows per 32-bit word, 12 pixels each in bits 31..20 then 15..4 (bit 31 /
 * bit 15 the leftmost of its row). */
#define SGI_GR2_RE3_MONOCOL_TOKEN 0x404dc /* token 311: IP20 cell background */
#define SGI_GR2_RE3_MONOX_TOKEN   0x40510 /* token 324: IP20 glyph pen x     */
#define SGI_GR2_RE3_FG_TOKEN      0x404e8 /* token 314: stipple fg colour    */
#define SGI_GR2_RE3_TILE_TOKEN    0x404ec /* token 315: tile/pattern data port */
#define SGI_GR2_RE3_STIPPLE_TOKEN 0x404f8 /* token 318: stipple pattern      */
#define SGI_GR2_RE3_IMAGE_TOKEN   0x40558 /* token 342: expDrawImage24       */
#define SGI_GR2_RE3_IMG_MAX       256     /* 342 groups recorded per sub-op  */
#define SGI_GR2_RE3_COPY_TOKEN    0x40550 /* token 340: expCopyRect marker   */
#define SGI_GR2_RE3_OP_TOKEN      0x4052c /* token 331: op/mode              */
/* The value token 331 carries is the DDX op type | 0x1000.  0x100b is
 * expTileRects (ref/GR2-DDX-STORE-SEQUENCES.txt:213), the op that tiles the
 * root weave: a short header whose words [3] and [4] are the tile width and
 * height, then (w*h/4) words of an 8bpp bitmap, one tile pixel per byte, four
 * bytes per word MSB first.  Repeated over the destination. */
#define SGI_GR2_RE3_TILE_OP       0x100b  /* token 331 value: expTileRects   */
#define SGI_GR2_RE3_DONE_TOKEN    0x407a8 /* token 490: op terminator        */
/* The PUC_DATA words of one sub-op, kept so the rect list can be decoded from
 * the DDX store order rather than from a guessed tail heuristic.  A solid-rect
 * sub-op carries a short prefix then groups of four; the weave op's 1024 span
 * spans arrive before its own 331, so a per-sub-op buffer of this size is ample
 * and the span words are discarded when the 331 resets the buffer.  The image op
 * (expDrawImage24) is the exception: one 98x98 8bpp icon streams 4316 data words
 * in 196 groups, so the buffer must hold a whole image. */
#define SGI_GR2_RE3_DATA_MAX      4608
#define SGI_GR2_HQ_NUMGE       0x6a044
#define SGI_GR2_HQ_FIFO_FULL_T 0x6a054 /* full-timeout, driver writes 100    */
#define SGI_GR2_HQ_FIFO_EMPTY_T 0x6a058 /* empty-timeout                     */
#define SGI_GR2_HQ_FIFO_FULL   0x6a05c /* level at which the FIFO is "full" */
#define SGI_GR2_HQ_FIFO_EMPTY  0x6a060 /* level at which the FIFO is "empty"*/
#define SGI_GR2_HQ_UCODELOAD   0x6a064 /* GE7 ucode load data register      */
#define SGI_GR2_HQ_VERSION     0x6a06c /* HQ2 revision, read >> 16          */
#define SGI_GR2_HQ_GEPC        0x6a070 /* GE7 program counter / load address*/
/* The 6.5 driver stages the HQ2 microcode through 0x6a000..0x6a03f (16
 * words) — i.e. immediately below the HQ2 status register at 0x6a040 — not
 * at the 5.3 header's 0x60000 window.  The 6.5 binary is authoritative. */
#define SGI_GR2_HQ_UC_RAM      0x6a000 /* HQ2 ucode load staging (16 words) */

/* GE7 instruction load/verify.  Gr2DownloadGE7 writes a record for a PC into
 * the four-word window at ge[0].ram0[0xf8..0xfb] plus the load register
 * 0x6a064, having first written the PC to gepc (0x6a070); the verify pass
 * re-selects each PC and reads the words back, so all five words are per-PC
 * storage addressed by the current gepc.  Values are stored and returned
 * verbatim (the driver masks with 0x3dfffff itself), never tidied. */
#define SGI_GR2_GE_WIN_OFF     0x683e0 /* ge[0].ram0[0xf8..0xfb]            */
#define SGI_GR2_GE_WIN_WORDS   4
#define SGI_GR2_UCODE_PCS      0x10000 /* PC space the driver walks         */
#define SGI_GR2_UCODE_WORDS    5       /* 4 window words + 1 load register  */
#define SGI_GR2_FIN3_OFF    0x6b000 /* HQ2 fin3 register */
#define SGI_GR2_BDVERS_OFF  0x6c000 /* board version / config / video backend */
#define SGI_GR2_VC1_OFF     0x6c040 /* VC1 video controller */
#define SGI_GR2_XMAP_OFF    0x6c100 /* XMAP5 display-mode generators */
#define SGI_GR2_XMAP_STATUS 0x6c11c /* bit 1 = XMAP ready (polled)       */
#define SGI_GR2_XMAP_READY_BIT 0x2
#define SGI_GR2_XMAP_CTL_OFF  0x6c1a0 /* control/data regs written at init */
#define SGI_GR2_XMAP_CTL_END  0x6c1b8
/* RAMDAC colour-map programming, inside the XMAP control window.  The DDX
 * writes each entry as: the INDEX to 0x6c1b0, a control byte to 0x6c1b4, then a
 * sliding byte stream on 0x6c1a8 whose bytes are R,G,B of the addressed entry
 * followed by the R of the next.  Read off the boot device trace: e.g.
 *   0x6c1b0=0x21 ; 0x6c1b4=0x11 ; 0x6c1a8=0x1414141e  -> entry 0x21 = 0x141414,
 *   and 0x1e begins entry 0x22 (whose index write follows).  The palette is
 * therefore BUILT from the guest's own writes, not a static table. */
#define SGI_GR2_XMAP_PAL_DATA  0x6c1a8 /* R,G,B byte stream (sliding)      */
#define SGI_GR2_XMAP_PAL_INDEX 0x6c1b0 /* entry index being programmed     */
#define SGI_GR2_XMAP_PAL_CTL   0x6c1b4 /* control byte written per entry   */
/* The control byte is the high byte of the DDX's per-entry word (the DDX does
 * `index = entry & 0xff; control = entry >> 8` in expStoreNormalColors), so it
 * tags an entry with its kind, not a whole-map bank.  Replaying the default
 * boot shows why the "installed map" is not a single value:
 *   - entries 96..255 (the root weave) are written under 0x10 and NOWHERE else,
 *     so accepting only 0x11 renders the weave black;
 *   - but the root weave (0x10) and the colour ramp (0x11) are on screen
 *     together, so they must share one map, not two.
 * Committing 0x10 and 0x11 with last-write-wins resolves both: for every index
 * that has both, 0x11 is written last, so the `xwd` oracle is preserved
 * (4sight 18/18, default pseudomap 10/10), while the weave indices come in.
 * 0x00/0x01/0x1c are the overlay / pup / 24-bit maps (expStoreOverlayColors,
 * expStorePupColors, expStore24Colors) and must not touch the visible palette —
 * index 1's last 0x1c write would otherwise overwrite its red.  */
#define SGI_GR2_XMAP_PAL_BANK_INSTALLED 0x11 /* normal map, written last */
#define SGI_GR2_XMAP_PAL_BANK_ALT       0x10 /* same map; the weave lives here */
/* The XMAP CLUT bank selected by addrhi 0x1c is the 4-entry OVERLAY map
 * (measured while a 4Dwm menu is posted: entries 1/2/3 = red/white/black, entry
 * 0 left unprogrammed = transparent).  This bank is what makes overlay windows
 * visible, not the 24-bit map. */
#define SGI_GR2_XMAP_PAL_BANK_OVERLAY   0x1c
#define SGI_GR2_OVL_COLOURS             4
/* The three BT457 RAMDACs, one per colour channel.  Each has an address
 * register at +0 and a 256-entry palette/gamma RAM at +4 which auto-increments
 * its address after every write (the +8/+0xc registers are command/overlay and
 * unused here).  The XMAP CLUT (above) maps an 8-bit screen index to an 8-bit
 * RGB triple; these RAMs then map each of those R/G/B bytes through a 256-entry
 * ramp on the way to the DAC.  The kernel's Gr2SetGammaRamp writes that ramp:
 * it sets the address to 0 and streams 256 bytes per channel.  The stock golden
 * loads an identity ramp during the boot DAC probe and then the display gamma
 * (exactly 255*(v/255)^(1/1.7), measured) when Xsgi starts, so the model builds
 * the ramp from the guest's own writes and applies it at scanout. */
#define SGI_GR2_DAC0_OFF    0x6c0a0 /* red   DAC: addr / paltram            */
#define SGI_GR2_DAC_STRIDE  0x20    /* one DAC every 0x20 bytes             */
#define SGI_GR2_DAC_NDAC    3       /* red, green, blue                     */
#define SGI_GR2_DAC_ADDR    0x0
#define SGI_GR2_DAC_PALT    0x4

#define SGI_GR2_RE3_27_OFF  0x6c200 /* RE3 buffered register set */
#define SGI_GR2_RE3_24_OFF  0x6c280 /* RE3 unbuffered register set */
#define SGI_GR2_RE3_32_OFF  0x6c600 /* RE3 32-bit register */

/* Board-version bytes, one per 32-bit slot at 0x6c000/4/8/c.  The probe reads
 * the byte at 0x6c000 for the revision (active-low nibble) and the byte at
 * 0x6c004 for the config: its low two bits must NOT be 3 (or the probe
 * rejects the board), bit 4 selects 24-bit and bit 5 selects the Z buffer.
 * Values below pick the XZ (2 GE, 24-bit + Z) configuration. */
#define SGI_GR2_BDVERS0     0x06 /* rev slot: -> rev = (~0x06) & 0xf = 9 */
#define SGI_GR2_BDVERS1     0x30 /* cfg slot: low2=0, bit4=24-bit, bit5=Z */
#define SGI_GR2_BDVERS2     0x00
#define SGI_GR2_BDVERS3     0x00

struct SGIGr2State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint8_t regs[SGI_GR2_REG_SIZE];
    bool present;

    /* GE7 instruction storage, addressed by the current gepc: 5 words per
     * PC (the ge[0].ram0[0xf8..0xfb] window plus the load register). */
    uint32_t ucode[SGI_GR2_UCODE_PCS][SGI_GR2_UCODE_WORDS];
    uint32_t gepc;
    bool hq_ready;
    bool xmap_ready;

    /* Scanout (P0.4 step a): a QEMU display surface proving the output stage
     * before the RE3 producer.  `scanout` holds palette INDICES, exactly as the
     * hardware framebuffer does: the RAMDAC is applied when the buffer is
     * scanned out, not when it is drawn.  That ordering matters — the DDX draws
     * the root before it programs the entries the root uses, so resolving RGB
     * at draw time would bake in whatever the palette happened to hold then. */
    QemuConsole *con;
    uint8_t *scanout;
    /* Per-pixel flag: this pixel came from a direct-colour image op
     * (expDrawImage24), whose byte is a 3-3-2 RGB triple rather than a palette
     * index.  At scanout a flagged pixel is expanded 3-3-2, an unflagged one
     * goes through ramdac[], so both visual modes coexist on the same screen. */
    uint8_t *scanout332;
    /* Overlay plane (2 bits/pixel; one byte per pixel here).  0 is TRANSPARENT
     * — the main plane shows through — and 1..3 index the 4-entry overlay
     * colormap.  4Dwm menus are depth-2 override-redirect windows: the DDX
     * draws them with the SAME RE3 rect ops as the main plane but under mode
     * 309 == 3, and those ops were previously dropped, which is why the menu
     * was invisible.  They are routed to this buffer and composited over the
     * main plane at scanout, overlay-over-main, index 0 transparent. */
    uint8_t *overlay;
    uint32_t ovl_ramdac[SGI_GR2_OVL_COLOURS];
    bool ovl_ramdac_set;
    bool scanout_bars; /* fill a colour-bar test pattern (P0.4 step a) */
    bool poly_stroke;  /* 302 op: stroke its path instead of filling (test seam) */

    /* RE3 producer colour latch + RAMDAC palette (8-bit mode).  In the 8-bit
     * mode the guest runs (xwininfo: depth 8 PseudoColor), the RE3 fill colour
     * the DDX writes is a palette INDEX; the RAMDAC maps index -> RGB for
     * scanout.  `scanout` is the framebuffer the RE3 fill writes and VC1 scans
     * out, so the producer and the output stage meet in one buffer.
     *
     * The palette is BUILT from the guest's own RAMDAC programming (see the
     * XMAP_PAL_* registers above) rather than a static table: the DDX programs
     * it at server start, so nothing is assumed and a 4sight pseudomap and a
     * default pseudomap each come out as the server meant them. */
    uint32_t ramdac[256];
    uint8_t ramdac_index;      /* entry selected by XMAP_PAL_INDEX          */
    uint8_t ramdac_ctl;        /* bank select from XMAP_PAL_CTL             */
    uint8_t ramdac_stage[3];   /* R,G,B bytes accumulating from PAL_DATA     */
    unsigned ramdac_stage_n;   /* bytes held (0..2)                          */
    /* Per-channel output ramp (the BT457 DACs' palette/gamma RAM): index ->
     * byte, applied to each of the CLUT's R/G/B bytes at scanout.  Built from
     * the guest's DAC writes (see SGI_GR2_DAC0_OFF); reset to the identity. */
    uint8_t dac_ramp[SGI_GR2_DAC_NDAC][256];
    uint8_t dac_addr[SGI_GR2_DAC_NDAC]; /* current paltram write address     */
    bool dac_ramp_set;                  /* guest has programmed any entry    */
    uint8_t re3_colour;   /* last colour latched from the RE3 colour token */
    bool re3_colour_valid;
    uint32_t last_puc;    /* previous PUC_DATA word (rect geometry pair)     */
    uint32_t prev_puc;    /* the word before last_puc                        */
    bool last_puc_valid;
    /* Generic PUC draw path: PUC_COLOR + PUC_RECTI2D + three PUC_DATA words.
     * [ASSUMPTION, from the captured root-weave stream: 1024 rects of
     * (colour, x0, 0x4ff, x0) — colour indices running 96..255 while x0 and the
     * third word run 0..1023 and the middle word is always 1279, so the three
     * data words read as a horizontal span (x0, x1, y0).] */
    uint8_t puc_colour;
    bool puc_rect_armed;  /* a PUC_RECTI2D is awaiting its three data words  */
    uint32_t puc_rect[3];
    unsigned puc_rect_n;  /* data words collected for the armed rectangle    */

    /* Fill-op markers (note 25/26).  A full-screen rect is drawn stippled when
     * token 318 armed a pattern, flat when token 304 marked a solid rect. */
    bool re3_solid_seen;    /* token 304 seen in the current op              */
    bool re3_spans_seen;    /* token 305 seen in the current op              */
    bool re3_line_seen;     /* token 345 seen: the op is a segment list      */
    bool re3_spanstip_seen; /* token 347 seen: the op is a stippled span list */
    bool re3_poly_seen;     /* token 302 seen: the op is a filled polygon     */
    bool re3_mono_seen;     /* token 312/349 seen: a 1-bpp glyph blit         */
    /* Glyph blits (expDrawMonoImage).  Token 349 is written TWICE per glyph,
     * both times with the pen x; the piece that follows each is (f0,f1,h) plus
     * h/2 words of an 8-px-wide bitmap (high byte = row 2k).  The pen value and
     * the PUC_DATA index it was written at are recorded so the decoder can find
     * each piece, since the pen tokens are not PUC_DATA themselves. */
    uint32_t re3_pen_val[SGI_GR2_RE3_PEN_MAX];
    unsigned re3_pen_off[SGI_GR2_RE3_PEN_MAX];
    unsigned re3_npens;
    /* Colour images (expDrawImage24).  Token 342 starts one run: the value is the
     * run's x, and the PUC_DATA that follows is (y, width, 1, nwords, 2, 0) then
     * nwords 32-bit words of 4 8-bit palette indices each (MSB byte first).
     * Scanlines are streamed as two runs (x=459 w=64 and x=523 w=34 for the
     * 98x98 EZsetup icon); the run position and its data index are recorded so
     * the decoder can walk the groups, and rows are padded with 0xdeadbeef. */
    uint32_t re3_img_x[SGI_GR2_RE3_IMG_MAX];
    unsigned re3_img_off[SGI_GR2_RE3_IMG_MAX];
    unsigned re3_nimg;
    bool re3_image_seen;
    /* expCopyRect (token 340).  When the type dispatch picks the stream-copy
     * path the DDX writes 340 = words-per-row, then seven PUC_DATA words:
     * stride, src_x, src_y, width, height, dst_x, dst_y.  There is no 490, so
     * the op is closed by the seventh word.  A screen-to-screen copy is the
     * window-move/scroll case; off-screen sources are not modelled yet. */
    uint32_t re3_copy_wpr;
    uint32_t re3_copy_v[7];
    unsigned re3_copy_n;
    bool re3_copy_active;
    /* expTileRects (token 331 value 0x100b): the op tiles a bitmap over the
     * destination.  The root weave is a 16x16 tile repeated across the screen. */
    bool re3_tile_seen;
    uint32_t re3_tile_word0; /* the first tile word, streamed via token 315 */
    /* The clip rectangle list that precedes expTileRects: the DDX writes one
     * expValidateClip per exposed rect (token 304, four (x1,y1,x2,y2) words, no
     * colour token, 490-terminated) and then the tile fill.  The tile is
     * repeated inside this list, not over the whole screen — it is what leaves
     * the toolchest and Console untouched. */
    uint32_t re3_clip[64][4];
    unsigned re3_nclip;
    /* The name-label's row: the top y of the last solid rect drawn in the label
     * bar colour (222).  The DDX does not put the text y on the wire, and the
     * label bar is drawn just before its glyphs, so this is the structural link
     * for the glyph baseline. */
    int re3_label_y;
    bool re3_label_valid;
    /* IP20 Xsgi's glyph piece, read straight from the stream (see the token
     * defines above): pen x, foreground colour (from the MONO token), and the
     * PUC_DATA index at which the piece's y/w/h begin.  Used in preference to
     * the XZ label-bar link when present. */
    uint32_t re3_monox;
    uint32_t re3_monocol;
    unsigned re3_mono_off;
    bool re3_monox_valid;
    bool re3_monocol_valid;
    bool re3_pair_seen;     /* a 1280-then-1024 pair appeared in the payload */
    bool re3_stipple_valid; /* token 318 written: next rect is stippled      */
    uint32_t re3_stipple;   /* 32-bit stipple pattern (token 318)            */
    uint8_t re3_fg;         /* token 314: stipple foreground colour index    */
    bool re3_fg_valid;
    uint32_t re3_rop;       /* token 309 value of the current sub-op         */
    bool re3_rop_valid;
    /* PUC_DATA of the current sub-op, so the rect list is decoded from the
     * DDX store order (a short prefix then groups of four (x1,y1,x2,y2)) and
     * not from a guessed (w,h) tail.  Reset at every 331 and 490. */
    uint32_t re3_data[SGI_GR2_RE3_DATA_MAX];
    unsigned re3_data_n;
    bool re3_data_overflow;

    /* Variant params supplied by the machine glue. */
    uint8_t ges;       /* number of GE7 engines (1, 2, 4, 8) */
    uint8_t bitplanes; /* 8 or 24 */
    bool zbuffer;
    /* Board-version/config bytes, one per 32-bit slot at 0x6c000/4/8/c.  These
     * are per-instance so a variant can report its own identity: XZ defaults
     * to 0x06/0x30/0x00/0x00 (24-bit + Z); XS-24 is 0x06/0x10/0x00/0x00
     * (24-bit, no Z).  See the BDVERS defaults below. */
    uint8_t bdvers0;
    uint8_t bdvers1;
    uint8_t bdvers2;
    uint8_t bdvers3;

    /* GIO interrupt output.  The kernel driver registers its retrace handler
     * with setgiovector on GIO vector 2 — the same line the HPC3 exposes as
     * its "gio-retrace" input, which is how Newport's VRINT is delivered.  A
     * real CRT always has a vertical retrace, so the board raises it at ~60 Hz
     * once it has been started; the pulse is asserted then lowered after the
     * blanking interval, mirroring Newport's VBLANK model.  Without it the
     * driver's interrupt-driven paths never run and Xsgi wedges. */
    qemu_irq irq;
    QEMUTimer *retrace_timer;
    QEMUTimer *retrace_lower_timer;
    /* Flush a whole sub-op once the FIFO goes quiet.  Every op but the last of a
     * batch is terminated by the next op's 331 (or a 490), but the DDX does not
     * always write a 490, so the final op has no successor to flush it - the last
     * glyph of "guest" was being dropped.  Real hardware drains the FIFO and runs
     * the op; this timer models that drain, rescheduled on every token write and
     * firing only after the stream pauses. */
    QEMUTimer *fifo_flush_timer;
    /* VC1 register file and SRAM.  Addresses are byte addresses; the arrays are
     * indexed by address>>1 (16-bit entries). */
    uint8_t vc1_addrlo;
    uint8_t vc1_addrhi;
    uint16_t vc1_reg[SGI_GR2_VC1_REG_WORDS];
    uint16_t vc1_sram[SGI_GR2_VC1_SRAM_WORDS];
    bool retrace_active;

    /* GE7 3D path (the powerflip bust).  See the SGI_GR2_GE7_* ports above.
     * Matrices arrive 16 words at a time on their slot; a matrix is only valid
     * once a whole 16-word run has landed, so partial runs are never used.
     * Vertices/normals arrive three floats at a time.  A completed polygon is
     * transformed by projection*modelview and Z-rasterised into `scanout`. */
    /* The matrices and polygon buffer are read by GCC's auto-vectoriser with
     * ALIGNED SSE ops, so their offset in the struct must actually be 16-byte
     * aligned; without this the misaligned `mulps` raises #GP. */
    float ge_mv[16] __attribute__((aligned(16)));   /* token 55: modelview  */
    float ge_proj[16] __attribute__((aligned(16))); /* token 56: projection */
    unsigned ge_mv_n;              /* words of the current modelview run     */
    unsigned ge_proj_n;            /* words of the current projection run    */
    bool ge_mv_valid;
    bool ge_proj_valid;
    /* MSINGLE combined matrix (token 54).  When it is the most recently
     * written matrix port it is the whole transform, so it takes precedence
     * over the separate modelview/projection pair. */
    float ge_single[16] __attribute__((aligned(16)));
    unsigned ge_single_n;
    bool ge_single_valid;
    unsigned long ge_seq54, ge_seq55, ge_seq56, ge_seq;
    int vp_x;                      /* token 60 + 3 PUC_DATA: viewport x,y,w,h */
    int vp_y;
    int vp_w;
    int vp_h;
    unsigned vp_n;                 /* viewport words collected               */
    bool vp_armed;                 /* token 60 seen, awaiting three words    */
    bool vp_valid;
    int ge_win_x;                  /* GL window content origin on screen,    */
    int ge_win_y;                  /* latched from token 485 (SGI_GR2_GE7_   */
                                   /* WINRECT): the DDX streams the window's */
                                   /* screen rect each frame, so the origin  */
                                   /* follows a window move.  Token cell =   */
                                   /* x, then PUC_DATA y_bottom, w, h; y is  */
                                   /* bottom-origin, converted on latch.     */
    int ge_clip_x, ge_clip_y;      /* token 485 words collected              */
    int ge_clip_w, ge_clip_h;
    unsigned ge_clip_n;
    bool ge_clip_armed;
    bool ge_need_clear;            /* a fresh frame: clear the drawable first */
    float ge_poly[SGI_GR2_GE7_MAX_VERTS][3]; /* current polygon, object space  */
    float ge_vnormal[SGI_GR2_GE7_MAX_VERTS][3]; /* its per-vertex normals     */
    unsigned ge_poly_n;
    bool ge_strip;                 /* current run is a triangle mesh (strip)  */
    float ge_line[SGI_GR2_GE7_MAX_LVERTS][2]; /* buffered line vertices      */
    unsigned ge_line_n;
    float ge_line_pending[2];      /* line vertex coordinates collected       */
    unsigned ge_line_pn;
    float ge_normal[3];            /* current vertex normal                  */
    float ge_nx, ge_ny, ge_nz;     /* normal words collected                 */
    float ge_vx, ge_vy, ge_vz;     /* vertex words collected                 */
    unsigned ge_n_n, ge_v_n;       /* words collected for normal / vertex     */
    /* Phong material/light state (tokens 116..129).  Each is a small vector
     * filled one component per repeated token write; ge_mat_tok detects the
     * run boundary so a 4-component (RGBA) vector does not leak its alpha
     * into the next.  Values are the guest's, not assumed. */
    float ge_ambient[3];           /* token 120: material ambient RGB        */
    float ge_diffuse[3];           /* token 122: material diffuse RGB        */
    float ge_specular[3];          /* token 124: material specular RGB       */
    float ge_emission[3];          /* token 118: material emission RGB       */
    float ge_lcolor[3];            /* token 126: light colour RGB            */
    float ge_lpos[3];              /* token 127: light position XYZ          */
    float ge_ambient_sum[3];       /* token 117: summed ambient RGB          */
    hwaddr ge_mat_tok;             /* last material token (run boundary)     */
    unsigned ge_mat_n;             /* components collected in the run        */
    bool ge_mat_valid;
    float *ge_zbuf;                /* SCREEN_W*SCREEN_H depth, lazily made   */
    bool ge_3d_seen;               /* a polygon has been rasterised          */
    unsigned long ge_polys;        /* polygons rasterised (trace/diagnostics) */
};

#endif /* HW_MISC_SGI_GR2_H */
