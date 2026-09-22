/*
 * Silicon Graphics IRIS IP2 processor board.
 *
 * This code is licensed under the GPL.
 */

#ifndef HW_MISC_SGI_IP2_H
#define HW_MISC_SGI_IP2_H

#include "hw/core/sysbus.h"
#include "hw/core/cpu.h"
#include "exec/cpu-common.h"
#include "exec/memattrs.h"
#include "chardev/char-fe.h"

#define TYPE_SGI_IP2 "sgi-ip2"
OBJECT_DECLARE_SIMPLE_TYPE(SGIIP2State, SGI_IP2)

/*
 * The system-segment container (PROM, registers, NVRAM, page SRAM). The
 * machine places the DUART regions inside it; the reset-time boot mirror is
 * an alias of the whole container.
 */
MemoryRegion *sgi_ip2_sys_region(DeviceState *dev);

/*
 * The Multibus I/O segment (64 KB). Board subregions -- the 2190 disk and the
 * GL2 graphics window -- are placed inside it by the machine.
 */
MemoryRegion *sgi_ip2_mbio_region(DeviceState *dev);

/*
 * The board's custom-MMU translation, installed on the CPU by the machine
 * (which can see the target's CPUM68KState) via sgi_ip2_set_cpu().
 */
int sgi_ip2_ext_tlb_fill(void *opaque, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx, bool probe,
                         hwaddr *physical, int *prot);
void sgi_ip2_set_cpu(DeviceState *dev, CPUState *cpu);

#endif
