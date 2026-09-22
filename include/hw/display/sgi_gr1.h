/*
 * SGI GR1 ("Eclipse") graphics for the Personal IRIS (IP6)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SGI_GR1_H
#define HW_DISPLAY_SGI_GR1_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_GR1 "sgi-gr1"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGR1State, SGI_GR1)

/*
 * sysbus outputs (active high: 1 = asserted; the board's LIO status bits
 * are active low, the machine inverts):
 *   0  GE5 interrupt       -> LIO_GE
 *   1  FIFO half-full      -> LIO_FIFO
 *   2  vertical blank      -> LIO_VR / LIO_VRSTAT
 * GPIO input 0 is the board reset line (cpuauxctl bit 7).
 *
 * MMIO 0: the 32 KB register window (the machine maps it at 0x1f000000 and
 * mirrors it at 0x1f008000, as MAME ip6.cpp does).
 */
#define SGI_GR1_IRQ_GE      0
#define SGI_GR1_IRQ_FIFO    1
#define SGI_GR1_IRQ_VBLANK  2

#endif
