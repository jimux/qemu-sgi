/*
 * Silicon Graphics IRIS IP2 processor board.
 *
 * This code is licensed under the GPL.
 */

#ifndef HW_MISC_SGI_IP2_H
#define HW_MISC_SGI_IP2_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"

#define TYPE_SGI_IP2 "sgi-ip2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIIP2State, SGI_IP2)

/*
 * The system-segment container (PROM, registers, NVRAM, page SRAM). The
 * machine places the DUART regions inside it; the reset-time boot mirror is
 * an alias of the whole container.
 */
MemoryRegion *sgi_ip2_sys_region(DeviceState *dev);

#endif
