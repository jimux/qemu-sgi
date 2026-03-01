#include "qemu/osdep.h"

#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_smp.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "target/mips/cpu.h"

#define SGI_SMP_MMIO_SIZE 0x100

static uint64_t sgi_smp_read(void *opaque, hwaddr addr, unsigned size) {
  SGISMPState *s = opaque;
  addr &= ~7ULL; /* 64-bit aligned offsets */

  switch (addr) {
  case SGI_SMP_CPU_COUNT:
    return s->num_cpus;
  case SGI_SMP_CPU_ID:
    if (current_cpu) {
      return current_cpu->cpu_index;
    }
    return 0;
  case SGI_SMP_IPI_STATUS: {
    uint32_t status = 0;
    if (current_cpu && current_cpu->cpu_index < SGI_SMP_MAXCPU) {
      if (s->ipi_pending[current_cpu->cpu_index]) {
        status = 1;
      }
    }
    return status;
  }
  case SGI_SMP_BOOT_STATUS:
    return s->boot_status;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    return 0;
  }
}

static void sgi_smp_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size) {
  SGISMPState *s = opaque;
  addr &= ~7ULL;

  switch (addr) {
  case SGI_SMP_IPI_SET: {
    uint32_t mask = val;
    for (int i = 0; i < s->num_cpus; i++) {
      if (mask & (1U << i)) {
        s->ipi_pending[i] = true;
        qemu_irq_raise(s->ipi_irq[i]);
      }
    }
  } break;
  case SGI_SMP_IPI_CLEAR:
    if (current_cpu && current_cpu->cpu_index < SGI_SMP_MAXCPU) {
      int cpu = current_cpu->cpu_index;
      s->ipi_pending[cpu] = false;
      qemu_irq_lower(s->ipi_irq[cpu]);
    }
    break;
  case SGI_SMP_BOOT_ADDR:
    s->boot_addr = val;
    break;
  case SGI_SMP_BOOT_GO: {
    uint32_t mask = val;
    for (int i = 0; i < s->num_cpus; i++) {
      if ((mask & (1U << i)) && !s->started[i]) {
        if (s->cpus[i]) {
          CPUState *cs = s->cpus[i];
          cpu_reset(cs);
          MIPSCPU *mips_cpu = MIPS_CPU(cs);
          mips_cpu->env.active_tc.PC = s->boot_addr;
          cs->halted = 0;
          qemu_cpu_kick(cs);
          s->started[i] = true;
          s->boot_status |= (1U << i);
        }
      }
    }
  } break;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    break;
  }
}

static const MemoryRegionOps sgi_smp_ops = {
    .read = sgi_smp_read,
    .write = sgi_smp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
};

static void sgi_smp_realize(DeviceState *dev, Error **errp) {
  SGISMPState *s = SGI_SMP(dev);
  int i;

  memory_region_init_io(&s->mmio, OBJECT(s), &sgi_smp_ops, s, "sgi-smp",
                        SGI_SMP_MMIO_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

  for (i = 0; i < s->num_cpus; i++) {
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->ipi_irq[i]);
  }
}

static void sgi_smp_reset(DeviceState *dev) {
  SGISMPState *s = SGI_SMP(dev);
  int i;

  s->boot_addr = 0;
  s->boot_status = 0;
  for (i = 0; i < SGI_SMP_MAXCPU; i++) {
    s->ipi_pending[i] = false;
    s->started[i] = false;
    if (s->ipi_irq[i]) {
      qemu_irq_lower(s->ipi_irq[i]);
    }
  }
  s->started[0] = true;
  s->boot_status = 1;
}

static const Property sgi_smp_properties[] = {
    DEFINE_PROP_UINT32("num-cpus", SGISMPState, num_cpus, 1),
};

static void sgi_smp_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_smp_realize;
  device_class_set_legacy_reset(dc, sgi_smp_reset);
  device_class_set_props(dc, sgi_smp_properties);
}

static const TypeInfo sgi_smp_info = {
    .name = TYPE_SGI_SMP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGISMPState),
    .class_init = sgi_smp_class_init,
};

static void sgi_smp_register_types(void) {
  type_register_static(&sgi_smp_info);
}

type_init(sgi_smp_register_types)
