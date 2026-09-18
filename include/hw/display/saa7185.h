/*
 * Philips SAA7185 "Digital Video Encoder" I2C model
 *
 * The O2 AV1 module drives the S-Video / Composite outputs from a
 * SAA7185 encoder fed by MACE output port E.  The guest `mvp` driver
 * probes it over the MACE I2C master; the probe uses a plain address
 * read (no subaddress), so the slave must ACK a bare read transfer.
 *
 * Register map recovered from the IP32 PROM firmware (mvp7185.h):
 *   addr 0x3a IPC input port control (0x80 = internal colour bars)
 *   addr 0x61 SC  standard control    (NTSC / PAL / subcarrier)
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SAA7185_H
#define HW_DISPLAY_SAA7185_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_SAA7185 "saa7185"
OBJECT_DECLARE_SIMPLE_TYPE(SAA7185State, SAA7185)

struct SAA7185State {
    I2CSlave parent_obj;

    uint8_t regs[256];   /* encoder register file          */
    uint8_t ptr;         /* current register pointer       */
    bool first;          /* next send byte is the subaddr  */
};

#endif /* HW_DISPLAY_SAA7185_H */
