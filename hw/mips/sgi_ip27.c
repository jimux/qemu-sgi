/*
 * SGI IP27 (Origin 200 / Origin 2000 / Onyx2, "SN0") — headless hub/NUMA model.
 *
 * IP27 is radically unlike the IP22/IP24/IP32 lineage already modelled here:
 * each node pairs one or two R10000/R12000 CPUs with local memory and a Hub
 * (Bedrock) ASIC; nodes are linked by CrayLink/NUMAlink and the hub owns global
 * addressing, routing and coherence.  Booting goes through the SN0-container
 * ARCS PROM (PROM_library/bins/cpu/ip27/ip27prom.img), which discovers nodes/IO
 * through the hub registers and klconfig.
 *
 * Scope: headless.  No graphics, no audio.  This file models the SN0 address
 * spaces and one (or more, later) node; the hub register file lives in
 * hw/misc/sgi_hub.c.
 *
 * Address facts are from Linux arch/mips/include/asm/sn/sn0/addrs.h and
 * arch/mips/include/asm/mach-ip27/spaces.h; PROM images are SN0 "JFKSWCSM"
 * containers (see analysis_tools/prom/sn_container.py).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_baseio.h"
#include "hw/misc/sgi_hub.h"
#include "hw/misc/unimp.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/*
 * SN0 address spaces (XKPHYS).  These are direct-mapped segments; the space
 * is selected by address bits [58:56], which is why the CPU must form physical
 * addresses with a >= 59-bit mask (see the PAMask override in the init).
 */
#define IP27_HSPEC_BASE 0x9000000000000000ULL
#define IP27_IO_BASE 0x9200000000000000ULL
#define IP27_MSPEC_BASE 0x9400000000000000ULL
#define IP27_UNCAC_BASE 0x9600000000000000ULL
#define IP27_CAC_BASE 0xa800000000000000ULL

/* Preserve bits [58:0]; bits [58:56] carry the address-space selector. */
#define IP27_PAMASK 0x07FFFFFFFFFFFFFFULL

/* M-mode: 4 GB per node, nasid at bits [39:32], 16 MB small window. */
#define IP27_NODE_SIZE 0x100000000ULL
#define IP27_SWIN_SIZE 0x1000000ULL
#define IP27_HUB_WIDGET 1ULL

/* PROM window: file code (0x1fc00000) is mapped here. */
#define IP27_PROM_BASE 0x1fc00000ULL
#define IP27_PROM_SIZE (1 * MiB)

/*
 * LBOOT window: the local hub's boot flash (and other directory-bus devices).
 * LBOOT_BASE = HSPEC_BASE + 0x10000000; direct-mapped XKPHYS so the masked
 * physical base is 0x10000000.  The IP27 flash holds the same SN0 container
 * image (header + code), so the PROM reads its identity from here.
 */
#define IP27_LBOOT_PHYS (IP27_HSPEC_BASE + 0x10000000ULL)
#define IP27_FLASH_SIZE (1 * MiB)

#define IP27_MAX_CPUS 2

/* MD_MEMORY_CONFIG bank size codes (hubmd.h MD_SIZE_*). */
#define IP27_MD_SIZE_8MB 1
#define IP27_MD_SIZE_1GB 8

static uint64_t ip27_phys(uint64_t va) { return va & IP27_PAMASK; }

/* Physical base of node `nasid`'s widget-`wid` small window. */
static uint64_t ip27_swin_phys(uint32_t nasid, uint32_t wid) {
  return ip27_phys(IP27_IO_BASE | ((uint64_t)nasid << 32) |
                   ((uint64_t)wid << 24));
}

static void main_cpu_reset(void *opaque) {
  MIPSCPU *cpu = opaque;
  cpu_reset(CPU(cpu));
  /*
   * The SN0 PROM is entered from the flash sloader, which leaves gp set to
   * the PROM's data pointer and has already mapped the PROM's 1 MB XKSEG
   * image (vaddr 0xc00000001fc00000 -> phys 0x1fc00000, ASID 1, uncached,
   * read-only) via a wired TLB entry.  Model that handoff so early
   * gp-relative PROM accesses resolve; the PROM later re-wires the same entry
   * itself.
   */
  cpu->env.active_tc.gpr[28] = 0xc00000001fce4fd0ULL;
  cpu->env.active_tc.gpr[29] = 0xa800000000100000ULL; /* PROMDATA stack */
  /* Default console-device descriptor ($f3), as left by the sloader. */
  cpu->env.active_fpu.fpr[3].d = 0xc00000001fc74ce0ULL;
  mips_cpu_install_mapping(cpu, 0xc00000001fc00000ULL, 0x1fc00000ULL,
                           0x001fe000, 1, 0x12);
}

/*
 * Encode an amount of RAM into MD_MEMORY_CONFIG bank size codes (8 banks).
 *
 * The hub reports one 3-bit size code per bank; the PROM/kernel derive total
 * memory from these.  Decompose into the largest power-of-two banks, in
 * descending address order as the hardware would.
 */
static uint64_t sgi_ip27_mem_config(uint64_t ram_size) {
  uint64_t cfg = 0;
  int bank = 0;

  while (ram_size && bank < 8) {
    int code = IP27_MD_SIZE_8MB; /* 8 MB */
    while (code < IP27_MD_SIZE_1GB &&
           (0x800000ULL << (code - IP27_MD_SIZE_8MB)) * 2 <= ram_size) {
      code++;
    }
    cfg |= (uint64_t)code << (bank * 3);
    ram_size -= 0x800000ULL << (code - IP27_MD_SIZE_8MB);
    bank++;
  }
  return cfg;
}

/* Read a big-endian 64-bit field from a buffer. */
static uint64_t sgi_ip27_be64(const uint8_t *p) {
  return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
         ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
         ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
         ((uint64_t)p[6] << 8) | (uint64_t)p[7];
}

/*
 * Load an SN0-container PROM (magic "JFKSWCSM" at 0x40).  The header carries
 * the load address (0xA0), code offset (0x98) and code size (0xB0); the code
 * is copied to the IP27 PROM window.  The image is never modified.
 */
static void sgi_ip27_load_prom(const char *filename, MemoryRegion *prom,
                               MemoryRegion *flash) {
  g_autoptr(GError) err = NULL;
  gsize len = 0;
  gchar *data = NULL;
  uint64_t load_addr, code_off, code_size, load_phys;
  uint8_t *dst = memory_region_get_ram_ptr(prom);
  uint8_t *flash_dst = memory_region_get_ram_ptr(flash);

  if (!g_file_get_contents(filename, &data, &len, &err)) {
    error_report("sgi-ip27: could not read PROM '%s': %s", filename,
                 err->message);
    exit(EXIT_FAILURE);
  }

  if (len < 0x100) {
    error_report("sgi-ip27: PROM '%s' too small (%zu bytes)", filename, len);
    exit(EXIT_FAILURE);
  }
  if (memcmp(data + 0x40, "JFKSWCSM", 8) != 0) {
    error_report("sgi-ip27: PROM '%s' is not an SN0 (JFKSWCSM) container",
                 filename);
    exit(EXIT_FAILURE);
  }

  load_addr = sgi_ip27_be64((const uint8_t *)data + 0xA0);
  code_off = sgi_ip27_be64((const uint8_t *)data + 0x98);
  code_size = sgi_ip27_be64((const uint8_t *)data + 0xB0);
  load_phys = ip27_phys(load_addr);

  if (code_off + code_size > (uint64_t)len || code_size > IP27_PROM_SIZE) {
    error_report("sgi-ip27: PROM '%s' code range out of bounds "
                 "(off=0x%" PRIx64 " size=0x%" PRIx64 ", file=%zu)",
                 filename, code_off, code_size, len);
    exit(EXIT_FAILURE);
  }

  memcpy(dst, data + code_off, code_size);

  /* The flash holds the whole container image (header + code). */
  memcpy(flash_dst, data, MIN((gsize)IP27_FLASH_SIZE, len));

  qemu_log_mask(LOG_GUEST_ERROR,
                "sgi-ip27: loaded SN0 PROM '%s': load=0x%" PRIx64
                " -> phys 0x%" PRIx64 ", code off=0x%" PRIx64
                " size=0x%" PRIx64 "\n",
                filename, load_addr, load_phys, code_off, code_size);
}

static void sgi_ip27_init(MachineState *machine) {
  Clock *cpuclk;
  MemoryRegion *prom;
  MemoryRegion *flash;
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *ram = machine->ram;
  MemoryRegion *ram_uncac, *ram_mspec;
  DeviceState *hub;
  DeviceState *baseio;
  SGIHubState *hub_state;
  int ncpus = machine->smp.cpus;
  int i;

  if (ncpus < 1) {
    ncpus = 1;
  }
  if (ncpus > IP27_MAX_CPUS) {
    ncpus = IP27_MAX_CPUS;
  }
  machine->smp.cpus = ncpus;

  /* CPU clock: R10000-era.  Overridable for bring-up. */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  {
    const char *e = getenv("IP27_CPU_HZ");
    clock_set_hz(cpuclk, e && *e ? strtoull(e, NULL, 0) : 200000000ULL);
  }

  /*
   * Create the node's CPUs.  IP27 relies on XKPHYS bits [58:56] to select its
   * address spaces, so each CPU gets the machine-scoped wide PAMask override
   * (zero on every other machine, which therefore keeps the default PAMask).
   */
  for (i = 0; i < ncpus; i++) {
    CPUMIPSState *env;
    MIPSCPU *c = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);

    env = &c->env;
    env->PAMask_override = IP27_PAMASK;
    env->PAMask = IP27_PAMASK;
    /* Model a cold reset (ErrorEPC clear) for the PROM's reset dispatch. */
    env->cold_erre_clear = true;

    cpu_mips_irq_init_cpu(c);
    cpu_mips_clock_init(c);
    qemu_register_reset(main_cpu_reset, c);
    if (i != 0) {
      CPU(c)->start_powered_off = true;
    }
  }

  /* Node-local memory at physical 0, aliased uncached (UNCAC/MSPEC spaces). */
  memory_region_add_subregion(system_memory, 0, ram);

  ram_uncac = g_new(MemoryRegion, 1);
  memory_region_init_alias(ram_uncac, NULL, "sgi-ip27.ram.uncac", ram,
                           0, machine->ram_size);
  memory_region_add_subregion(system_memory, ip27_phys(IP27_UNCAC_BASE),
                              ram_uncac);

  ram_mspec = g_new(MemoryRegion, 1);
  memory_region_init_alias(ram_mspec, NULL, "sgi-ip27.ram.mspec", ram, 0,
                           machine->ram_size);
  memory_region_add_subregion(system_memory, ip27_phys(IP27_MSPEC_BASE),
                              ram_mspec);

  /* Hub ASIC in the node's widget-1 small window. */
  hub = qdev_new(TYPE_SGI_HUB);
  qdev_prop_set_uint32(hub, "nasid", 0);
  qdev_prop_set_uint32(hub, "num-cpus", ncpus);
  qdev_prop_set_uint64(hub, "mem-config",
                       sgi_ip27_mem_config(machine->ram_size));
  sysbus_realize_and_unref(SYS_BUS_DEVICE(hub), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(hub), 0, ip27_swin_phys(0, IP27_HUB_WIDGET));

  /*
   * BaseIO board as XIO widget 0 (Bridge part 0xc002 + IOC3).  The PROM
   * probes this widget's ID once the hub link reads up and then targets the
   * node's IO widget (widget 8) for the Bridge registers/config.
   */
  baseio = qdev_new(TYPE_SGI_BASEIO);
  qdev_prop_set_uint32(baseio, "nasid", 0);
  qdev_prop_set_uint32(baseio, "widget", 0);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(baseio), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(baseio), 0, ip27_swin_phys(0, 0));

  {
    DeviceState *baseio8 = qdev_new(TYPE_SGI_BASEIO);
    qdev_prop_set_uint32(baseio8, "nasid", 0);
    qdev_prop_set_uint32(baseio8, "widget", 8);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(baseio8), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(baseio8), 0, ip27_swin_phys(0, 8));
  }

  /*
   * Remaining XIO widget space of the node (widgets 2..7, 9..), so stray
   * probes read 0 instead of raising a data bus error.
   */
  create_unimplemented_device("ip27-xio-low", ip27_phys(IP27_IO_BASE) + 0x2000000ULL,
                              0x6000000ULL);
  create_unimplemented_device("ip27-xio-high", ip27_phys(IP27_IO_BASE) + 0x9000000ULL,
                              0x100000000ULL - 0x9000000ULL);

  /*
   * Hub RBOOT window (HSPEC + 0x30000000): the PROM reads its reboot/status
   * word there.  Model as zero-returning until a hub detail is needed.
   */
  create_unimplemented_device("ip27-rboot",
                              ip27_phys(IP27_HSPEC_BASE + 0x30000000ULL),
                              0x10000000ULL);

  hub_state = SGI_HUB(hub);
  {
    CPUState *c;
    i = 0;
    CPU_FOREACH(c) {
      if (i >= IP27_MAX_CPUS) {
        break;
      }
      hub_state->cpus[i] = c;
      /*
       * Hub interrupt levels map to CP0 lines: INT_PEND0 -> IP2,
       * INT_PEND1 -> IP3, RTC -> IP4, PROF -> IP5, error -> IP6.
       */
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 0,
                         MIPS_CPU(c)->env.irq[2]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 1,
                         MIPS_CPU(c)->env.irq[3]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 2,
                         MIPS_CPU(c)->env.irq[4]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 3,
                         MIPS_CPU(c)->env.irq[5]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 4,
                         MIPS_CPU(c)->env.irq[6]);
      i++;
    }
  }

  /*
   * PROM window.  The IP27 PROM executes from this window and also keeps
   * writable data/BSS there (it is RAM-backed after the flash image is
   * copied in), so it must be writable.
   */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_ram(prom, NULL, "sgi-ip27.prom", IP27_PROM_SIZE,
                         &error_fatal);
  memory_region_add_subregion(system_memory, IP27_PROM_BASE, prom);

  /* Boot flash in the LBOOT window (identity + the container image). */
  flash = g_new(MemoryRegion, 1);
  memory_region_init_ram(flash, NULL, "sgi-ip27.flash", IP27_FLASH_SIZE,
                         &error_fatal);
  memory_region_add_subregion(system_memory, ip27_phys(IP27_LBOOT_PHYS),
                              flash);

  if (machine->firmware) {
    g_autofree char *filename =
        qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename) {
      error_report("sgi-ip27: could not find firmware '%s'",
                   machine->firmware);
      exit(EXIT_FAILURE);
    }
    sgi_ip27_load_prom(filename, prom, flash);
  } else {
    warn_report("sgi-ip27: no firmware specified; use -bios ip27prom.img");
  }
}

static void sgi_ip27_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Origin 200/2000 (IP27, headless)";
  mc->init = sgi_ip27_init;
  mc->default_ram_size = 256 * MiB;
  mc->default_ram_id = "sgi-ip27.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
  mc->default_cpus = 2;
  mc->max_cpus = IP27_MAX_CPUS;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
  mc->no_parallel = 1;
}

static const TypeInfo sgi_ip27_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip27"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip27_class_init,
};

static void sgi_ip27_machine_init(void) { type_register_static(&sgi_ip27_type); }

type_init(sgi_ip27_machine_init)
