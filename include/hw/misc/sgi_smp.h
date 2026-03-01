#ifndef SGI_SMP_H
#define SGI_SMP_H

#include "exec/cpu-common.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_SMP "sgi-smp"
OBJECT_DECLARE_SIMPLE_TYPE(SGISMPState, SGI_SMP)

#define SGI_SMP_MAXCPU 128
#define SGI_SMP_BASE_ADDR 0x1f480000

/* Register offsets */
#define SGI_SMP_CPU_COUNT 0x00
#define SGI_SMP_CPU_ID 0x08
#define SGI_SMP_IPI_SET 0x10
#define SGI_SMP_IPI_CLEAR 0x18
#define SGI_SMP_IPI_STATUS 0x20
#define SGI_SMP_BOOT_ADDR 0x28
#define SGI_SMP_BOOT_GO 0x30
#define SGI_SMP_BOOT_STATUS 0x38

struct SGISMPState {
  SysBusDevice parent_obj;

  MemoryRegion mmio;
  uint32_t num_cpus;
  uint32_t boot_addr;
  uint32_t boot_status;

  /* Per-CPU state */
  bool ipi_pending[SGI_SMP_MAXCPU];
  bool started[SGI_SMP_MAXCPU];
  qemu_irq ipi_irq[SGI_SMP_MAXCPU];

  /* CPU references for BOOT_GO */
  CPUState *cpus[SGI_SMP_MAXCPU];
};

#endif /* SGI_SMP_H */
