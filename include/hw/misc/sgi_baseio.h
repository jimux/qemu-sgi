/*
 * SGI IP27 BaseIO board model (minimal, PROM-driven).
 *
 * A standalone Origin 200 attaches a BaseIO (Bridge + IOC3 + SCSI) as an XIO
 * widget.  The IP27 PROM discovers it by reading the widget-ID word (bits
 * [27:12] = part; 0xc002 = Bridge), then reads the board configuration and
 * inits the IOC3 whose SuperIO 16550 UART A is the serial console.
 *
 * This device models the widget window; the register set grows only from
 * observed PROM accesses (translate-or-omit).  The IOC3 UART byte-reversal
 * matches the Octane BRIDGE model (std_reg = offset ^ 3).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_BASEIO_H
#define HW_MISC_SGI_BASEIO_H

#include "hw/char/serial.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_BASEIO "sgi-baseio"
OBJECT_DECLARE_SIMPLE_TYPE(SGIBaseIOState, SGI_BASEIO)

/* XIO widget window modelled by the BaseIO. */
#define SGI_BASEIO_WINDOW_SIZE 0x1000000ULL

/* Widget ID word (read at offset 4): part 0xc002 (Bridge). */
#define SGI_BASEIO_WIDGET_PART 0xc002

struct SGIBaseIOState {
  SysBusDevice parent_obj;

  MemoryRegion iomem;

  uint32_t nasid;
  uint32_t widget;
};

#endif /* HW_MISC_SGI_BASEIO_H */
