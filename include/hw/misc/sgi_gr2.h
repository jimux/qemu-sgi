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

#define TYPE_SGI_GR2 "sgi-gr2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGr2State, SGI_GR2)

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
#define SGI_GR2_FIN3_OFF    0x6b000 /* HQ2 fin3 register */
#define SGI_GR2_BDVERS_OFF  0x6c000 /* board version / config / video backend */
#define SGI_GR2_VC1_OFF     0x6c040 /* VC1 video controller */
#define SGI_GR2_XMAP_OFF    0x6c100 /* XMAP5 display-mode generators */
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

    /* Variant params supplied by the machine glue. */
    uint8_t ges;       /* number of GE7 engines (1, 2, 4, 8) */
    uint8_t bitplanes; /* 8 or 24 */
    bool zbuffer;
};

#endif /* HW_MISC_SGI_GR2_H */
