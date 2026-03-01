/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/misc/sgi_pvmem.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qom/object.h"

#define SGI_PVMEM_MMIO_SIZE 0x100

static uint64_t sgi_pvmem_read(void *opaque, hwaddr addr, unsigned size) {
  SGIPVMEMState *s = opaque;
  addr &= ~7ULL; /* 64-bit aligned offsets */

  switch (addr) {
  case SGI_PVMEM_TOTAL_RAM:
    return s->total_ram;
  case SGI_PVMEM_HIGH_BASE:
    return s->high_base;
  case SGI_PVMEM_HIGH_SIZE:
    return s->high_size;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    return 0;
  }
}

static void sgi_pvmem_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size) {
  qemu_log_mask(LOG_GUEST_ERROR,
                "%s: PV-MEM registers are read-only! Offset 0x%" HWADDR_PRIx
                "\n",
                __func__, addr);
}

static const MemoryRegionOps sgi_pvmem_ops = {
    .read = sgi_pvmem_read,
    .write = sgi_pvmem_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
};

static void sgi_pvmem_realize(DeviceState *dev, Error **errp) {
  SGIPVMEMState *s = SGI_PVMEM(dev);

  /* Setup MMIO region */
  memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvmem_ops, s, "sgi-pvmem",
                        SGI_PVMEM_MMIO_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

  /* Total RAM read from current machine state, calculate base and size */
  s->total_ram = current_machine->ram_size;
  /*
   * High RAM is aliased at physical 0x40000000 (O2_HIGH_RAM_BASE) in
   * sgi_o2.c. Report that same address so the PROM passes correct ARCS
   * FreeMemory descriptors to the kernel.
   */
  s->high_base = 0x40000000ULL;

  if (s->total_ram > (256 * 1024 * 1024)) {
    s->high_size = s->total_ram - (256 * 1024 * 1024);
  } else {
    s->high_size = 0;
  }
}

static void sgi_pvmem_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_pvmem_realize;
}

static const TypeInfo sgi_pvmem_info = {
    .name = TYPE_SGI_PVMEM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPVMEMState),
    .class_init = sgi_pvmem_class_init,
};

static void sgi_pvmem_register_types(void) {
  type_register_static(&sgi_pvmem_info);
}

type_init(sgi_pvmem_register_types)
