/*
 * Silicon Graphics IRIS 3130 GL2 graphics (UC4 + DC4 + BP3).
 *
 * This code is licensed under the GPL.
 */

#ifndef HW_DISPLAY_SGI_GL2_H
#define HW_DISPLAY_SGI_GL2_H

#include "hw/core/sysbus.h"

#define TYPE_SGI_GL2 "sgi-gl2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGL2State, SGI_GL2)

/*
 * The board's graphics register window (GF2/FBC, UC4 and DC4), to be placed
 * inside the IP2 Multibus I/O region at MBIO + 0x2000.
 */
MemoryRegion *sgi_gl2_mmio_region(DeviceState *dev);

/*
 * The GF2 Geometry Engine command pipe, at GEPORT (write-only, in the GE
 * segment at 0x60001000) and GETOKEN (0x60000000).  Placed by the machine
 * in the system address map.
 */
MemoryRegion *sgi_gl2_ge_region(DeviceState *dev);

/*
 * Diagnostic hook implemented by the machine (which can see the target CPU):
 * report whether the currently executing guest is in user mode, and if so
 * return its PC.  Used only for the SGI_MMIO_LOG read tracing.
 */
bool sgi_gl2_user_read(uint32_t *pc_out);

#endif
