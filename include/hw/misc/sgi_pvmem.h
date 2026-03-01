#ifndef SGI_PVMEM_H
#define SGI_PVMEM_H

#include "exec/cpu-common.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_PVMEM "sgi-pvmem"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPVMEMState, SGI_PVMEM)

#define SGI_PVMEM_BASE_ADDR 0x1f480100

/* Register offsets */
#define SGI_PVMEM_TOTAL_RAM 0x00
#define SGI_PVMEM_HIGH_BASE 0x08
#define SGI_PVMEM_HIGH_SIZE 0x10

struct SGIPVMEMState {
  SysBusDevice parent_obj;

  MemoryRegion mmio;

  uint64_t total_ram;
  uint64_t high_base;
  uint64_t high_size;
};

#endif /* SGI_PVMEM_H */
