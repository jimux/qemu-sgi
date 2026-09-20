/*
 * SGI IP27 "Hub" (Bedrock) ASIC model.
 *
 * See include/hw/misc/sgi_hub.h for provenance.  This is deliberately a
 * faithful-but-partial register model: known registers return values derived
 * from the emulated node (nasid, CPU presence, memory config, RTC); unknown
 * accesses are logged as unimplemented and read as zero rather than being
 * faked.  The register set grows only in response to observed PROM/kernel
 * accesses (translate-or-omit).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_hub.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "target/mips/cpu.h"

/* --- Hub PI (processor interface) offsets, from Linux sn0/hubpi.h --- */
#define PI_CPU_PROTECT 0x000000
#define PI_PROT_OVRRD 0x000008
#define PI_IO_PROTECT 0x000010
#define PI_REGION_PRESENT 0x000018
#define PI_CPU_NUM 0x000020
#define PI_CALIAS_SIZE 0x000028
#define PI_CPU_PRESENT_A 0x000040
#define PI_CPU_PRESENT_B 0x000048
#define PI_CPU_ENABLE_A 0x000050
#define PI_CPU_ENABLE_B 0x000058
#define PI_INT_PEND_MOD 0x000090
#define PI_INT_PEND0 0x000098
#define PI_INT_PEND1 0x0000a0
#define PI_INT_MASK0_A 0x0000a8
#define PI_INT_MASK1_A 0x0000b0
#define PI_INT_MASK0_B 0x0000b8
#define PI_INT_MASK1_B 0x0000c0
#define PI_CC_PEND_SET_A 0x0000c8
#define PI_CC_PEND_SET_B 0x0000d0
#define PI_CC_PEND_CLR_A 0x0000d8
#define PI_CC_PEND_CLR_B 0x0000e0
#define PI_CC_MASK 0x0000e8
#define PI_RT_COMPARE_A 0x000108
#define PI_RT_COMPARE_B 0x000110
#define PI_RT_EN_A 0x000140
#define PI_RT_EN_B 0x000148
#define PI_RT_LOCAL_CTRL 0x000160
#define PI_RT_COUNT 0x030100

/* --- Hub MD (memory/directory) offsets --- */
#define MD_MEMORY_CONFIG 0x200018
#define MD_REFRESH_CONTROL 0x200020
/*
 * MicroLAN (1-wire) control.  The PROM bit-bangs the hub NIC EEPROM through
 * this register: it writes PULSE<19:10>/SAMPLE<9:2>, then spins on DONE
 * (bit1), returning RD_DATA (bit0).  With no EEPROM device present the line
 * floats high, so RD_DATA reads 1 and the PROM's part search reports
 * "not present" instead of hanging.
 */
#define MD_MLAN_CTL 0x2000a8
#define MD_UREG0_0 0x220000
#define MD_UREG0_7 0x220038
#define MD_SLOTID_USTAT 0x220048
#define MD_UREG1_0 0x220080
#define MD_UREG1_15 0x2200f8

/* --- Hub NI (network interface) offsets --- */
#define NI_STATUS_REV_ID 0x600000
#define NI_PORT_RESET 0x600008
#define NI_PROTECTION 0x600010
#define NI_SCRATCH_REG0 0x600100
#define NI_SCRATCH_REG1 0x600108

/* --- Hub II (I/O interface) offsets --- */
#define IIO_WID 0x400000
#define IIO_WSTAT 0x400008
#define IIO_WCR 0x400020
#define IIO_ILAPR 0x400100
#define IIO_ILAPO 0x400108
#define IIO_IOWA 0x400110
#define IIO_IIWA 0x400118
#define IIO_ILCSR 0x400128
#define IIO_SCRATCH_REG0 0x400150
#define IIO_SCRATCH_REG1 0x400158

/* Hub widget identification: part 0xc101 (hub). */
#define HUB_WIDGET_PART_NUM 0xc101
#define HUB_WIDGET_REV 1
/*
 * IIO_ILCSR.  FUN_bfc56ac0 (the PROM's IOC3 finder) selects the IOC3 console
 * only if, for the local node, (ILCSR >> 12) & 2 (bit13, LLP status "up") and
 * the sign of (ILCSR << 53) (bit10, LLP enable) are both set.  Also set the
 * hub's own lnk_stat field (bits 17:16) since other paths read it.
 */
#define IIO_ILCSR_LINK_WORKING ((2ULL << 16) | (1ULL << 13) | (1ULL << 10))
/* IIO_WCR widget id field [3:0]. */
#define HUB_XIO_WIDGET_ID 8

/* NI_STATUS_REV_ID helpers */
#define NSRI_NODEID_SHFT 8
#define NSRI_REV_SHFT 4
#define NSRI_CHIPID_SHFT 0
#define NSRI_LINKUP (1ULL << 29)

/* HUB_REV_2_0 = 2 */
#define SGI_HUB_REV 2
#define SGI_HUB_CHIPID_HUB 0

/* PI_INT_PEND_MOD: bit 8 selects "set"; otherwise the value is a clear mask. */
#define PI_INT_PEND_SET_BIT 0x100

/* The hub RTC is a free-running counter; model it at 1 MHz (microseconds). */
static uint64_t sgi_hub_rtc_count(void) {
  return (uint64_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000);
}

static void sgi_hub_update_irqs(SGIHubState *s) {
  int i;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    qemu_set_irq(s->irq[i][0], (s->int_pend0 != 0));
    qemu_set_irq(s->irq[i][1], (s->int_pend1 != 0));
    if (s->rt_enable[i] && s->rt_compare[i] &&
        sgi_hub_rtc_count() >= s->rt_compare[i]) {
      qemu_set_irq(s->irq[i][2], 1);
    }
  }
}

static uint64_t sgi_hub_pi_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case PI_CPU_PROTECT:
  case PI_PROT_OVRRD:
  case PI_IO_PROTECT:
    return 0;
  case PI_REGION_PRESENT:
    return s->region_present;
  case PI_CPU_NUM:
    /* Slice of the CPU performing this access. */
    return current_cpu ? (current_cpu->cpu_index & 1) : 0;
  case PI_CALIAS_SIZE:
    return s->calias_size;
  case PI_CPU_PRESENT_A:
    return s->cpu_present[0];
  case PI_CPU_PRESENT_B:
    return s->cpu_present[1];
  case PI_CPU_ENABLE_A:
    return s->cpu_enable[0];
  case PI_CPU_ENABLE_B:
    return s->cpu_enable[1];
  case PI_INT_PEND0:
    return s->int_pend0;
  case PI_INT_PEND1:
    return s->int_pend1;
  case PI_INT_MASK0_A:
    return s->int_mask[0];
  case PI_INT_MASK1_A:
    return s->int_mask[1];
  case PI_INT_MASK0_B:
    return s->int_mask[2];
  case PI_INT_MASK1_B:
    return s->int_mask[3];
  case PI_CC_MASK:
    return s->cc_mask;
  case PI_RT_COMPARE_A:
    return s->rt_compare[0];
  case PI_RT_COMPARE_B:
    return s->rt_compare[1];
  case PI_RT_EN_A:
    return s->rt_enable[0];
  case PI_RT_EN_B:
    return s->rt_enable[1];
  case PI_RT_COUNT:
    return sgi_hub_rtc_count();
  case PI_RT_LOCAL_CTRL:
    return 0;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented PI read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_pi_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  switch (off) {
  case PI_CPU_PROTECT:
  case PI_PROT_OVRRD:
  case PI_IO_PROTECT:
  case PI_REGION_PRESENT:
  case PI_CPU_PRESENT_A:
  case PI_CPU_PRESENT_B:
  case PI_CPU_ENABLE_A:
  case PI_CPU_ENABLE_B:
    break;
  case PI_CALIAS_SIZE:
    s->calias_size = val;
    break;
  case PI_INT_PEND_MOD: {
    /* Set/clear a pending interrupt level. */
    uint64_t bit = val & 0x3f;
    bool set = (val & PI_INT_PEND_SET_BIT) != 0;
    if (bit < 64) {
      if (set) {
        s->int_pend0 |= (1ULL << bit);
      } else {
        s->int_pend0 &= ~(1ULL << bit);
      }
    }
    sgi_hub_update_irqs(s);
    break;
  }
  case PI_INT_MASK0_A:
    s->int_mask[0] = val;
    break;
  case PI_INT_MASK1_A:
    s->int_mask[1] = val;
    break;
  case PI_INT_MASK0_B:
    s->int_mask[2] = val;
    break;
  case PI_INT_MASK1_B:
    s->int_mask[3] = val;
    break;
  case PI_CC_PEND_SET_A:
  case PI_CC_PEND_SET_B:
  case PI_CC_PEND_CLR_A:
  case PI_CC_PEND_CLR_B:
    /* Crosscall pending set/clear; held in INT_PEND0 CC bits by the hub. */
    break;
  case PI_CC_MASK:
    s->cc_mask = val;
    break;
  case PI_RT_COMPARE_A:
    s->rt_compare[0] = val;
    break;
  case PI_RT_COMPARE_B:
    s->rt_compare[1] = val;
    break;
  case PI_RT_EN_A:
    s->rt_enable[0] = val;
    break;
  case PI_RT_EN_B:
    s->rt_enable[1] = val;
    break;
  case PI_RT_LOCAL_CTRL:
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented PI write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

static uint64_t sgi_hub_md_read(SGIHubState *s, hwaddr off) {
  if (off == MD_MEMORY_CONFIG) {
    return s->mem_config;
  }
  if (off == MD_REFRESH_CONTROL) {
    return s->refresh_ctl;
  }
  if (off == MD_SLOTID_USTAT) {
    return s->slotid_ustat;
  }
  if (off == MD_MLAN_CTL) {
    /* DONE (bit1) always set; RD_DATA (bit0) idle high (no EEPROM). */
    return 0x3;
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    /* uController/UART and MLAN registers: state held, semantic model later. */
    return 0;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD read @0x%" HWADDR_PRIx
                           "\n",
                off);
  return 0;
}

static void sgi_hub_md_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  if (off == MD_MEMORY_CONFIG) {
    s->mem_config = val;
    return;
  }
  if (off == MD_REFRESH_CONTROL) {
    s->refresh_ctl = val;
    return;
  }
  if (off == MD_MLAN_CTL) {
    /* PULSE/SAMPLE command; the line model is stateless for now. */
    return;
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    return;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD write @0x%" HWADDR_PRIx
                           " = 0x%" PRIx64 "\n",
                off, val);
}

static uint64_t sgi_hub_ni_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case NI_STATUS_REV_ID:
    return ((uint64_t)(s->nasid & 0x1ff) << NSRI_NODEID_SHFT) |
           ((uint64_t)SGI_HUB_REV << NSRI_REV_SHFT) |
           ((uint64_t)SGI_HUB_CHIPID_HUB << NSRI_CHIPID_SHFT) | NSRI_LINKUP;
  case NI_PROTECTION:
    return 0;
  case NI_SCRATCH_REG0:
    return s->ni_scratch[0];
  case NI_SCRATCH_REG1:
    return s->ni_scratch[1];
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented NI read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_ni_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  switch (off) {
  case NI_PORT_RESET:
  case NI_PROTECTION:
    break;
  case NI_SCRATCH_REG0:
    s->ni_scratch[0] = val;
    break;
  case NI_SCRATCH_REG1:
    s->ni_scratch[1] = val;
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented NI write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

static uint64_t sgi_hub_ii_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case IIO_WID:
    return ((uint64_t)HUB_WIDGET_PART_NUM << 16) |
           ((uint64_t)HUB_WIDGET_REV << 12);
  case IIO_WSTAT:
    return 0;
  case IIO_WCR:
    return s->ii_wcr;
  case IIO_ILAPR:
  case IIO_ILAPO:
    return 0;
  case IIO_IOWA:
    return s->ii_iowa;
  case IIO_IIWA:
    return s->ii_iiwa;
  case IIO_ILCSR:
    return s->ii_ilcsr;
  case IIO_SCRATCH_REG0:
    return s->ii_scratch[0];
  case IIO_SCRATCH_REG1:
    return s->ii_scratch[1];
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented II read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_ii_write(SGIHubState *s, hwaddr off, uint64_t val) {
  switch (off) {
  case IIO_WSTAT:
  case IIO_WID:
  case IIO_ILAPR:
  case IIO_ILAPO:
    break;
  case IIO_WCR:
    s->ii_wcr = val;
    break;
  case IIO_IOWA:
    s->ii_iowa = val;
    break;
  case IIO_IIWA:
    s->ii_iiwa = val;
    break;
  case IIO_ILCSR:
    /* Keep the modelled link reported as working. */
    s->ii_ilcsr = val | IIO_ILCSR_LINK_WORKING;
    break;
  case IIO_SCRATCH_REG0:
    s->ii_scratch[0] = val;
    break;
  case IIO_SCRATCH_REG1:
    s->ii_scratch[1] = val;
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented II write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

/*
 * Hub registers are 64-bit but the PROM and kernel also access them as 32-bit
 * halves (big-endian: base+0 is the high word, base+4 the low word).  We model
 * the 64-bit register and fold 32-bit accesses onto it, otherwise a low-word
 * read (e.g. IIO_ILCSR's link status) silently returns 0.
 */
static uint64_t sgi_hub_read_off(SGIHubState *s, hwaddr off) {
  if (off < SGI_HUB_MD_BASE) {
    return sgi_hub_pi_read(s, off - SGI_HUB_PI_BASE);
  } else if (off < SGI_HUB_II_BASE) {
    return sgi_hub_md_read(s, off);
  } else if (off < SGI_HUB_NI_BASE) {
    return sgi_hub_ii_read(s, off);
  } else {
    return sgi_hub_ni_read(s, off);
  }
}

static void sgi_hub_write_off(SGIHubState *s, hwaddr off, uint64_t val) {
  if (off < SGI_HUB_MD_BASE) {
    sgi_hub_pi_write(s, off - SGI_HUB_PI_BASE, val, 8);
  } else if (off < SGI_HUB_II_BASE) {
    sgi_hub_md_write(s, off, val, 8);
  } else if (off < SGI_HUB_NI_BASE) {
    sgi_hub_ii_write(s, off, val);
  } else {
    sgi_hub_ni_write(s, off, val, 8);
  }
}

static uint64_t sgi_hub_read(void *opaque, hwaddr addr, unsigned size) {
  SGIHubState *s = opaque;
  hwaddr off = addr & (SGI_HUB_WINDOW_SIZE - 1);

  if (off >= SGI_HUB_REMOTE_ALIAS) {
    off -= SGI_HUB_REMOTE_ALIAS;
  }
  if (size == 4) {
    uint64_t full = sgi_hub_read_off(s, off & ~7ULL);
    return (off & 4) ? (full & 0xffffffffULL) : (full >> 32);
  }
  return sgi_hub_read_off(s, off);
}

static void sgi_hub_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size) {
  SGIHubState *s = opaque;
  hwaddr off = addr & (SGI_HUB_WINDOW_SIZE - 1);

  if (off >= SGI_HUB_REMOTE_ALIAS) {
    off -= SGI_HUB_REMOTE_ALIAS;
  }
  if (size == 4) {
    uint64_t cur = sgi_hub_read_off(s, off & ~7ULL);
    if (off & 4) {
      cur = (cur & 0xffffffff00000000ULL) | (val & 0xffffffffULL);
    } else {
      cur = (cur & 0xffffffffULL) | ((val & 0xffffffffULL) << 32);
    }
    sgi_hub_write_off(s, off & ~7ULL, cur);
    return;
  }
  sgi_hub_write_off(s, off, val);
}

static const MemoryRegionOps sgi_hub_ops = {
    .read = sgi_hub_read,
    .write = sgi_hub_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
};

static void sgi_hub_reset(DeviceState *dev) {
  SGIHubState *s = SGI_HUB(dev);
  int i;

  s->int_pend0 = 0;
  s->int_pend1 = 0;
  for (i = 0; i < 4; i++) {
    s->int_mask[i] = 0;
  }
  s->cc_mask = 0;
  s->region_present = 1;
  s->calias_size = 0;

  s->cpu_present[0] = 1;
  s->cpu_enable[0] = 1;
  s->cpu_present[1] = (s->num_cpus > 1) ? 1 : 0;
  s->cpu_enable[1] = (s->num_cpus > 1) ? 1 : 0;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    s->rt_compare[i] = 0;
    s->rt_enable[i] = 0;
  }

  /* MD_SLOTID_USTAT: FPGA/flash ready, slot id 0. */
  s->slotid_ustat = 0x10;

  /* II: hub widget id, working XIO link, all widgets accessible. */
  s->ii_wcr = HUB_XIO_WIDGET_ID;
  s->ii_iowa = 0x1ff;
  s->ii_iiwa = 0x1ff;
  s->ii_ilcsr = IIO_ILCSR_LINK_WORKING;
  s->ii_scratch[0] = 0;
  s->ii_scratch[1] = 0;

  sgi_hub_update_irqs(s);
}

static void sgi_hub_realize(DeviceState *dev, Error **errp) {
  SGIHubState *s = SGI_HUB(dev);
  int i, j;

  memory_region_init_io(&s->iomem, OBJECT(s), &sgi_hub_ops, s, "sgi-hub",
                        SGI_HUB_WINDOW_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    for (j = 0; j < 5; j++) {
      sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i][j]);
    }
  }
}

static const Property sgi_hub_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIHubState, nasid, 0),
    DEFINE_PROP_UINT32("num-cpus", SGIHubState, num_cpus, 1),
    DEFINE_PROP_UINT64("mem-config", SGIHubState, mem_config, 0),
};

static void sgi_hub_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_hub_realize;
  device_class_set_legacy_reset(dc, sgi_hub_reset);
  device_class_set_props(dc, sgi_hub_properties);
}

static const TypeInfo sgi_hub_info = {
    .name = TYPE_SGI_HUB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHubState),
    .class_init = sgi_hub_class_init,
};

static void sgi_hub_register_types(void) { type_register_static(&sgi_hub_info); }

type_init(sgi_hub_register_types)
