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
#define SGI_GR2_HQUCODE_OFF 0x60000 /* 8 K x 32-bit HQ2 microcode RAM */
#define SGI_GR2_GE_OFF      0x68000 /* GE7 units: 8 x 1 KB RAM0, 0x400 stride */
#define SGI_GR2_GE_STRIDE   0x400
#define SGI_GR2_GE_UNITS    8

#define SGI_GR2_HQ_OFF      0x6a000 /* HQ2 register block (mystery at 0x7c) */
#define SGI_GR2_HQ_MYSTERY  0x6a07c /* presence magic, read by Gr2Probe */
#define SGI_GR2_HQ_MAGIC    0xdeadbeefu

/* HQ2 registers, from the 6.5 gr2_init.o disassembly (Gr2FIFOHandler,
 * _Gr2HQ2RegInit, Gr2DownloadHQ2/GE7).  The FIFO status at 0x6a040 carries
 * the current occupancy in bits 6..12 (mask 0x1fc0) and an error flag in bit
 * 5; the driver polls it before/while draining the token FIFO, so it must read
 * as read-only and empty (0), not as whatever the bus last wrote. */
#define SGI_GR2_HQ_FIFOSTAT    0x6a040
#define SGI_GR2_HQ_READY_BIT   0x2     /* bit 1: ucode ready (polled)       */
#define SGI_GR2_HQ_TOKEN_START 0x4077c /* FIFO token Gr2Start writes to run */
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
     * before the RE3 producer.  `scanout` is the framebuffer VC1 would scan
     * (host xRGB32); VC1/RE3 will read it once the producer is emulated. */
    QemuConsole *con;
    uint32_t *scanout;
    bool scanout_bars; /* fill a colour-bar test pattern (P0.4 step a) */

    /* Variant params supplied by the machine glue. */
    uint8_t ges;       /* number of GE7 engines (1, 2, 4, 8) */
    uint8_t bitplanes; /* 8 or 24 */
    bool zbuffer;
};

#endif /* HW_MISC_SGI_GR2_H */
