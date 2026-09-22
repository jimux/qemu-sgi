/*
 * SGI VINO (Indy video input) — register shell.
 *
 * VINO is the Indy's (IP24) on-board video-input ASIC: the IndyCam digital
 * port plus composite/S-Video analog inputs.  Its register block lives in
 * EISA address space at physical 0x00080000 (IRIX sys/IP22.h
 * VINO_PHYS_BASE1; promoted by IP22.c simplevinoprobe()).
 *
 * Presence is a device property, default OFF: with present=off the region
 * reads all-ones exactly like an unmapped aperture, so a plain `-M indy`
 * invocation is byte-for-byte unchanged.  present=on makes the version
 * register answer so the PROM/kernel VINO probe recognises the part.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_VINO_H
#define HW_MISC_SGI_VINO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_VINO "sgi-vino"
OBJECT_DECLARE_SIMPLE_TYPE(SGIVinoState, SGI_VINO)

/* VINO register aperture.  Pinned from the IRIX 5.3 vino.a driver
 * (vino_regs.o master_reg_tbl): 39 direct-MMIO registers id 0..0x26 at an
 * 8-byte stride with the data word at +4, so the highest direct access is
 * 0x26*8+4 = 0x134.  The PROM diagnostics additionally touch +0x42/+0x43/
 * +0x61.  0x200 covers every access seen; grow only against new evidence. */
#define SGI_VINO_REG_SIZE 0x200

/* Version register (physical base + 0x04). */
#define SGI_VINO_REV_OFFSET   0x04
#define SGI_VINO_CHIP_ID      0x0B
#define SGI_VINO_REVISION     0x01

/* VINO I2C master: control/status at id 3 (offset 0x1c), data at id 4
 * (offset 0x24).  Recovered from the IRIX 5.3 driver (vino_i2c.o): the
 * control byte 0x5 issues a byte transfer, 0x0 forces the bus idle; status
 * bits read back from 0x1c are 0x01 bus-busy, 0x10 transfer-in-progress,
 * 0x20 ack, 0x80 error.  The analog decoder sits at 7-bit address 0x45
 * (write 0x8a / read 0x8b); a second device answers at 0x2b. */
#define SGI_VINO_I2C_CTRL_OFFSET  0x1c
#define SGI_VINO_I2C_DATA_OFFSET  0x24
#define SGI_VINO_I2C_ADDR_WRITE   0x8a   /* SAA7191 */
#define SGI_VINO_I2C_ADDR_ALT     0x56   /* second I2C device */

struct SGIVinoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bool present;
    uint8_t regs[SGI_VINO_REG_SIZE];

    /* Modelled SAA7191 / second-device I2C register files and the master
     * transfer state (write byte stream: address, sub-address, then data). */
    uint8_t i2c_dec[256];
    uint8_t i2c_alt[256];
    uint8_t i2c_ptr;
    uint8_t i2c_first;   /* next data byte is the device address */
    uint8_t i2c_expect_sub; /* next data byte is the sub-address */
    uint8_t i2c_alt_dev; /* current transaction targets the second device */
};


#endif /* HW_MISC_SGI_VINO_H */
