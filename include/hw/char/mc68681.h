/*
 * Motorola MC68681 / SCN2681 dual UART (SGI IP2 tty0-3).
 *
 * This code is licensed under the GPL.
 */

#ifndef HW_CHAR_MC68681_H
#define HW_CHAR_MC68681_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"

#define TYPE_MC68681 "mc68681"
OBJECT_DECLARE_SIMPLE_TYPE(MC68681State, MC68681)

/*
 * The DUART's 16-byte register file (channel A at +0, channel B at +8) is
 * exposed as a sysbus MMIO region so the board can place it inside its
 * system-segment container.
 */
MemoryRegion *mc68681_mmio_region(DeviceState *dev);

#endif
