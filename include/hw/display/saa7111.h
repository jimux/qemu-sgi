/*
 * Philips SAA7111 "Video Input Processor" (analog video decoder) I2C model
 *
 * The O2 AV1 Analog A/V module wires the S-Video (mini-DIN Y/C) and
 * Composite (RCA) jacks into a SAA7111 decoder; its digital D1 output
 * feeds MACE port A.  Composite-vs-S-Video selection is a SAA7111 I2C
 * register write (AIC1), not a MACE register.  The guest `mvp` driver
 * probes and initialises the chip over the MACE I2C master.
 *
 * Register map recovered from the IP32 PROM firmware (mvp7111.h):
 *   addr 0x02 AIC1 analog input control 1 (bit2 selects Y/C input)
 *   addr 0x09 LCR  luminance control     (bit7 selects Y/C input)
 *   addr 0x0e CC   chrominance control   (bit7 subcarrier/genlock)
 *   addr 0x1f STAT status (RO; bit6 HLCK, bit5 FIDT)
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SAA7111_H
#define HW_DISPLAY_SAA7111_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_SAA7111 "saa7111"
OBJECT_DECLARE_SIMPLE_TYPE(SAA7111State, SAA7111)

/* SAA7111 status register bits (mvp7111.h MVP_CHIP_7111_STAT_*) */
#define SAA7111_STAT_FIDT   0x20   /* bit5: odd/even field id */
#define SAA7111_STAT_HLCK   0x40   /* bit6: horizontal lock    */

struct SAA7111State {
    I2CSlave parent_obj;

    uint8_t regs[256];   /* decoder register file          */
    uint8_t ptr;         /* current register pointer       */
    bool first;          /* next send byte is the subaddr  */
};

#endif /* HW_DISPLAY_SAA7111_H */
