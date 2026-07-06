/*
 * QEMU Virtuix (IP55) machine emulation
 *
 * Virtuix is an SGI-Indy-derived, *virtualization-native* SMP workstation. It
 * began life as the Indy/IP24 base (chosen for its excellent emulation
 * coverage), but it is deliberately a SEPARATE machine with its OWN device
 * copies so that virtualization-native changes here never touch the authentic
 * Indy (`-M indy`, hw/mips/sgi_indy.c) and vice-versa:
 *
 *   - sgi_mc_virtuix.c       (TYPE_SGI_MC_VIRTUIX)      -- MC + host-RT counter
 *   - sgi_hpc3_virtuix.c     (TYPE_SGI_HPC3_VIRTUIX)    -- HPC3 + PS/2 (sgi-..-virtuix)
 *   - sgi_newport_virtuix.c  (TYPE_SGI_NEWPORT_VIRTUIX) -- Newport graphics
 *   - sgi_smp.c              (TYPE_SGI_SMP)             -- paravirtual SMP/IPI
 *
 * Leaf-peripheral boundary: the standalone leaf devices instantiated BY-REFERENCE
 * (WD33C93 SCSI, Z85C30 serial, Seeq Ethernet, Dallas RTC) remain SHARED -- they
 * are generic peripherals Virtuix does not diverge on, and copying them would add
 * no isolation value. Only the SGI-custom top-level controllers (above) are
 * Virtuix-private. Revisit if Virtuix ever needs to diverge on a leaf device.
 *
 * Divergence from authentic Indy that lives here (NOT in sgi_indy.c):
 *   - CPU clock defaults to 66.67 MHz (IP55_CPU_HZ-overridable); Indy stays 100 MHz.
 *   - RAM cap is 2 GiB; Indy stays 256 MiB.
 *   - MC exposes MC_REALTIME_CTR64 (host-sourced wall clock); Indy's MC does not.
 *   - SMP (sgi-smp / -accel tcg,thread=multi) is a first-class feature here.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "cpu.h"
#include "elf.h"
#include "hw/char/serial.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/sgi_glaccel.h"
#include "hw/display/sgi_newport_virtuix.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_arcs.h"
#include "hw/misc/sgi_hpc3_virtuix.h"
#include "hw/misc/sgi_mc_virtuix.h"
#include "hw/misc/sgi_smp.h"
#include "hw/misc/unimp.h"
#include "hw/scsi/scsi.h"
#include "monitor/qdev.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/*
 * Memory map (Guinness/IP24-derived; identical base addresses to Indy, but a
 * private copy so Virtuix can relocate things without disturbing Indy).
 */
#define SGI_RAM_LOW_BASE 0x08000000ULL
#define SGI_RAM_HIGH_BASE 0x20000000ULL
#define SGI_GIO_GFX_BASE 0x1f000000ULL
#define SGI_GIO_EXP0_BASE 0x1f400000ULL
#define SGI_GIO_EXP1_BASE 0x1f600000ULL
#define SGI_MC_BASE 0x1fa00000ULL
#define SGI_HPC3_BASE 0x1fb80000ULL
#define SGI_PROM_BASE 0x1fc00000ULL

/*
 * Paravirtual SMP controller: free MMIO hole between the MC (ends 0x1fa1ffff)
 * and HPC3 (0x1fb80000). MUST match the IP55-MP kernel header
 * (virtuix/kernel/ip55mp_addrs.h) as the single source of truth.
 */
#define SGI_VIRTUIX_SMP_BASE 0x1fa80000ULL

#define SGI_PROM_SIZE (512 * KiB)
/* Virtuix RAM cap: 2 GiB (Indy stays at the authentic 256 MiB). */
#define SGI_RAM_MAX (2048 * MiB)

/* Forward declaration for kernel boot trampoline */
static void write_kernel_trampoline(uint32_t kernel_entry_32);

/*
 * GIO empty-slot stub: an undriven GIO slot reads back as all-ones, telling the
 * PROM "no device present" without modeling bus-timeout exceptions.
 */
typedef struct {
  MemoryRegion iomem;
  const char *name;
} GIOEmptySlotState;

static uint64_t gio_empty_slot_read(void *opaque, hwaddr offset,
                                    unsigned size) {
  GIOEmptySlotState *s = opaque;

  qemu_log_mask(LOG_UNIMP,
                "%s: empty slot read (size %d, offset 0x%06" HWADDR_PRIx
                ") -> 0xffffffff\n",
                s->name, size, offset);
  return 0xffffffffffffffffULL;
}

static void gio_empty_slot_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size) {
  GIOEmptySlotState *s = opaque;

  qemu_log_mask(LOG_UNIMP,
                "%s: empty slot write (size %d, offset 0x%06" HWADDR_PRIx
                ", value 0x%08" PRIx64 ")\n",
                s->name, size, offset, value);
}

static const MemoryRegionOps gio_empty_slot_ops = {
    .read = gio_empty_slot_read,
    .write = gio_empty_slot_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void create_gio_empty_slot(MemoryRegion *sysmem, const char *name,
                                  hwaddr base, hwaddr size) {
  GIOEmptySlotState *s = g_new0(GIOEmptySlotState, 1);

  s->name = name;
  memory_region_init_io(&s->iomem, NULL, &gio_empty_slot_ops, s, name, size);
  memory_region_add_subregion(sysmem, base, &s->iomem);
}

static void main_cpu_reset(void *opaque) {
  MIPSCPU *cpu = opaque;

  cpu_reset(CPU(cpu));
}

/*
 * Write a MIPS boot trampoline to the PROM area (physical 0x1FC00000) for
 * -kernel direct boot: clears BEV/ERL, sets ARCS args, sets sp, jumps to entry.
 */
static void write_kernel_trampoline(uint32_t kernel_entry_32) {
#define MIPS_MFC0(rt, rd) (0x40000000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_MTC0(rt, rd) (0x40800000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_AND(rd, rs, rt)                                                   \
  (0x00000024 | ((rs) << 21) | ((rt) << 16) | ((rd) << 11))
#define MIPS_LUI(rt, imm) (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, im)                                                   \
  (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((im) & 0xFFFF))
#define MIPS_JR(rs) (0x00000008 | ((rs) << 21))
#define MIPS_NOP 0x00000000
#define MIPS_MOVE(rd, rs) (0x00000025 | ((rs) << 21) | ((rd) << 11))
#define T0 8
#define T1 9
#define A0 4
#define A1 5
#define A2 6
#define SP 29
#define CP0_STATUS 12

  uint32_t trampoline[16];
  int i = 0;

  uint16_t entry_hi = (kernel_entry_32 >> 16) & 0xFFFF;
  uint16_t entry_lo = kernel_entry_32 & 0xFFFF;

  /* Clear BEV (bit 22) and ERL (bit 2): mask ~((1<<22)|(1<<2)) = 0xFFBFFFFB */
  trampoline[i++] = cpu_to_be32(MIPS_MFC0(T0, CP0_STATUS));
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFBF));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xFFFB));
  trampoline[i++] = cpu_to_be32(MIPS_AND(T0, T0, T1));
  trampoline[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_STATUS));

  /* a0=0 (argc), a1=0 (argv), a2=environ pointer array */
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A0, 0));
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A1, 0));
  trampoline[i++] =
      cpu_to_be32(MIPS_LUI(A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) >> 16));
  trampoline[i++] =
      cpu_to_be32(MIPS_ORI(A2, A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) & 0xFFFF));

  /* sp = 8MB into kseg0 (0x80800000) */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(SP, 0x8080));

  /* jump to kernel entry */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T0, entry_hi));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T0, T0, entry_lo));
  trampoline[i++] = cpu_to_be32(MIPS_JR(T0));
  trampoline[i++] = cpu_to_be32(MIPS_NOP);

  rom_add_blob_fixed("kernel-trampoline", trampoline, i * 4, SGI_PROM_BASE);

  qemu_log("Virtuix: Wrote kernel trampoline at 0x%08x -> entry 0x%08x\n",
           (unsigned)SGI_PROM_BASE, kernel_entry_32);

#undef MIPS_MFC0
#undef MIPS_MTC0
#undef MIPS_AND
#undef MIPS_LUI
#undef MIPS_ORI
#undef MIPS_JR
#undef MIPS_NOP
#undef MIPS_MOVE
#undef T0
#undef T1
#undef A0
#undef A1
#undef A2
#undef SP
#undef CP0_STATUS
}

static void sgi_virtuix_init(MachineState *machine) {
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *prom;
  DeviceState *mc_dev;
  DeviceState *hpc3_dev;
  MIPSCPU *cpu;
  Clock *cpuclk;
  char *filename;
  int bios_size;

  /* Validate RAM size (Virtuix cap = 2 GiB) */
  if (machine->ram_size > SGI_RAM_MAX) {
    error_report("RAM size more than 2GB is not supported");
    exit(EXIT_FAILURE);
  }

  /*
   * Virtualization-native time: drive CP0 Count/Compare (the source of the HZ
   * scheduling tick, IRQ7) off the host monotonic clock instead of
   * QEMU_CLOCK_VIRTUAL. Under MTTCG at high -smp counts, virtual time advances
   * in bursts (each vCPU thread progresses its clock in chunks), so the tick
   * fires erratically -> the UI stalls for seconds (measured: window-drag lag,
   * with the guest 100% idle, was bursty/stalled HZ-tick delivery). Routing
   * Count to the realtime clock makes the tick smooth. Respects an explicit
   * user override (overwrite=0). See target/mips/system/cp0_timer.c. Authentic
   * Indy keeps the default VIRTUAL clock (sgi_indy.c sets nothing).
   */
  setenv("QEMU_MIPS_COUNT_REALTIME", "1", 0);

  /*
   * Virtualization-native CPU clock. The modeled frequency sets the CP0 Count
   * rate (Count = cpu_clock / CCRes); 66.67 MHz matches the frequency IRIX
   * believes (hinv "66 MHZ IP22"), so cycle->time conversion tracks real time
   * (~1.0x). The CPU still executes flat-out (TCG ignores the modeled clock).
   * Overridable via IP55_CPU_HZ. (Authentic Indy keeps 100 MHz in sgi_indy.c.)
   */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  {
    const char *e = getenv("IP55_CPU_HZ");
    clock_set_hz(cpuclk, e && *e ? strtoull(e, NULL, 0) : 66666666);
  }

  /*
   * Create N CPUs (IP55 SMP). default_cpus=1 keeps uniprocessor boots
   * byte-identical; -smp >1 creates secondaries that start powered off until
   * the SMP controller's BOOT_GO kicks them. Each CPU gets its own CP0 IRQ
   * lines and Count/Compare timer; EBase is auto-set from cpu_index.
   */
  int ncpus = machine->smp.cpus;
  MIPSCPU **cpus = g_new0(MIPSCPU *, ncpus);
  for (int i = 0; i < ncpus; i++) {
    cpus[i] = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpus[i]);
    cpu_mips_clock_init(cpus[i]);
    qemu_register_reset(main_cpu_reset, cpus[i]);
    if (i != 0) {
      CPU(cpus[i])->start_powered_off = true;
    }
  }
  cpu = cpus[0];

  /*
   * Paravirtual SMP controller. Only instantiated for true SMP boots so that
   * uniprocessor boots remain byte-identical. IPIs are delivered on CP0 IP6
   * (env.irq[6]) -- a hardware interrupt line nothing else drives on the
   * emulated Indy -- avoiding the CP0_Cause cross-thread race that a
   * software-interrupt bit would reintroduce (an IP54 lesson).
   */
  if (ncpus > 1) {
    DeviceState *smp = qdev_new(TYPE_SGI_SMP);
    qdev_prop_set_uint32(smp, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(smp), 0, SGI_VIRTUIX_SMP_BASE);

    SGISMPState *smp_state = SGI_SMP(smp);
    for (int i = 0; i < ncpus; i++) {
      smp_state->cpus[i] = CPU(cpus[i]);
      sysbus_connect_irq(SYS_BUS_DEVICE(smp), i, cpus[i]->env.irq[6]);
    }
  }

  /* PROM at 0x1fc00000 */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_rom(prom, NULL, "sgi.prom", SGI_PROM_SIZE, &error_fatal);
  memory_region_add_subregion(system_memory, SGI_PROM_BASE, prom);

  /*
   * Load PROM (skipped for -kernel direct boot, where ARCS stubs provide the
   * firmware interface). Default PROM name is ip55prom.bin; in practice Virtuix
   * boots via -kernel or an explicit -bios.
   */
  if (machine->kernel_filename) {
    filename = NULL;
  } else if (machine->firmware) {
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename) {
      error_report("Could not find firmware '%s'", machine->firmware);
      exit(EXIT_FAILURE);
    }
  } else {
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip55prom.bin");
    if (!filename) {
      warn_report("No firmware specified, use -bios to specify IP55 PROM");
      filename = NULL;
    }
  }

  if (filename) {
    bios_size =
        load_image_targphys(filename, SGI_PROM_BASE, SGI_PROM_SIZE, NULL);
    g_free(filename);
    if (bios_size < 0) {
      error_report("Could not load PROM image");
      exit(EXIT_FAILURE);
    }
  }

  /* Memory Controller (Virtuix variant, with host-RT counter) at 0x1fa00000 */
  mc_dev = qdev_new(TYPE_SGI_MC_VIRTUIX);
  qdev_prop_set_uint32(mc_dev, "ram-size", machine->ram_size);
  object_property_set_link(OBJECT(mc_dev), "ram", OBJECT(machine->ram),
                           &error_fatal);
  object_property_set_link(OBJECT(mc_dev), "system-memory",
                           OBJECT(system_memory), &error_fatal);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(mc_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(mc_dev), 0, SGI_MC_BASE);

  /* HPC3 + IOC2/INT3 (Virtuix variant) at 0x1fb80000 */
  hpc3_dev = qdev_new(TYPE_SGI_HPC3_VIRTUIX);
  qdev_prop_set_chr(hpc3_dev, "chardev", serial_hd(0));
  qdev_prop_set_uint8(hpc3_dev, "board-type", BOARD_IP24);
  {
    SGIHPC3VirtuixState *hpc3 = SGI_HPC3_VIRTUIX(hpc3_dev);
    if (!hpc3->nvram_filename) {
      qdev_prop_set_string(hpc3_dev, "nvram-file", "sgi_virtuix_nvram.bin");
    }
  }
  {
    NICInfo *nd = qemu_find_nic_info(TYPE_SGI_HPC3_VIRTUIX, true, NULL);
    if (nd) {
      qdev_set_nic_properties(hpc3_dev, nd);
    }
  }
  sysbus_realize_and_unref(SYS_BUS_DEVICE(hpc3_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(hpc3_dev), 0, SGI_HPC3_BASE);

  {
    SGIHPC3VirtuixState *hpc3 = SGI_HPC3_VIRTUIX(hpc3_dev);
    scsi_bus_legacy_handle_cmdline(&hpc3->scsi[0]->bus);
  }

  /*
   * Wire HPC3 interrupts to MIPS CPU IRQ lines:
   *   INT3 Local0 (SCSI, ethernet) -> IP2; Local1 (panel, DMA) -> IP3;
   *   PIT Timer 0 -> IP4; PIT Timer 1 -> IP5.
   */
  qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 0, cpu->env.irq[2]);
  qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 1, cpu->env.irq[3]);
  qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 0, cpu->env.irq[4]);
  qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 1, cpu->env.irq[5]);

  /* MC GIO DMA-complete -> HPC3 LIO_GDMA (Local0 bit 4) -> IP2 */
  sysbus_connect_irq(SYS_BUS_DEVICE(mc_dev), 0,
                     qdev_get_gpio_in_named(hpc3_dev, "mc-dma-irq", 0));

  /*
   * Newport graphics at the GIO graphics slot (0x1f000000); REX3 registers at
   * offset 0x0f0000. Empty-slot stubs cover the rest of the slot.
   */
  {
    DeviceState *newport_dev;

    create_gio_empty_slot(system_memory, "gio-gfx-low", SGI_GIO_GFX_BASE,
                          REX3_REG_OFFSET);

    /* Paravirtual GL accelerator: must realize BEFORE Newport because Newport
     * registers as a desktop renderer with glaccel during its own realize. */
    {
        DeviceState *glaccel_dev = qdev_new(TYPE_SGI_GLACCEL);
        qdev_set_id(glaccel_dev, g_strdup("glaccel"), &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(glaccel_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(glaccel_dev), 0, 0x1fa20000ULL);
    }

    newport_dev = qdev_new(TYPE_SGI_NEWPORT_VIRTUIX);
    object_property_add_child(OBJECT(machine), "newport", OBJECT(newport_dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(newport_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(newport_dev), 0,
                    SGI_GIO_GFX_BASE + REX3_REG_OFFSET);

    /* Newport VRINT -> HPC3 retrace interrupt */
    sysbus_connect_irq(SYS_BUS_DEVICE(newport_dev), 0,
                       qdev_get_gpio_in_named(hpc3_dev, "gio-retrace", 0));

    create_gio_empty_slot(system_memory, "gio-gfx-high",
                          SGI_GIO_GFX_BASE + REX3_REG_OFFSET + REX3_REG_SIZE,
                          4 * MiB - REX3_REG_OFFSET - REX3_REG_SIZE);
  }

  create_gio_empty_slot(system_memory, "gio-exp0", SGI_GIO_EXP0_BASE, 2 * MiB);
  create_gio_empty_slot(system_memory, "gio-exp1", SGI_GIO_EXP1_BASE, 4 * MiB);

  {
  }

  /*
   * Memory probe areas. The MC dynamically maps RAM aliases (priority 1) over
   * these unimplemented stubs (priority -1000); unmapped banks show through as 0.
   */
  create_unimplemented_device("low-mem-probe", SGI_RAM_LOW_BASE, 256 * MiB);
  create_unimplemented_device("high-mem-probe", SGI_RAM_HIGH_BASE, 256 * MiB);
  create_unimplemented_device("zero-mem-probe", 0x00000000, 512 * KiB);

  /* Map high RAM natively if > 256MB */
  if (machine->ram_size > 256 * MiB) {
    MemoryRegion *high_ram = g_new(MemoryRegion, 1);
    memory_region_init_alias(high_ram, NULL, "high-ram", machine->ram,
                             256 * MiB, machine->ram_size - 256 * MiB);
    memory_region_add_subregion(system_memory, 0x20000000, high_ram);
  }

  create_unimplemented_device("extended-mem-probe0", 0x18000000, 0x07000000);
  create_unimplemented_device("extended-mem-probe1", 0x30000000, 0x50000000);

  /*
   * Direct kernel boot via -kernel: load the IRIX ELF and set up ARCS firmware
   * stubs (GetMemoryDescriptor, GetEnvironmentVariable, Write, ...).
   */
  if (machine->kernel_filename) {
    uint64_t kernel_entry;
    uint64_t kernel_high = 0;
    long kernel_size;
    DeviceState *arcs_dev;
    SGIARCSState *arcs;

    kernel_size = load_elf(machine->kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL, &kernel_entry, NULL,
                           &kernel_high, NULL, ELFDATA2MSB, EM_MIPS, 1, 0);
    if (kernel_size < 0) {
      error_report("could not load kernel '%s': %s", machine->kernel_filename,
                   load_elf_strerror(kernel_size));
      exit(1);
    }

    uint64_t kernel_high_phys = kernel_high & 0x1FFFFFFF;

    qemu_log("Virtuix: Loaded kernel '%s' (%ld bytes)\n",
             machine->kernel_filename, kernel_size);
    qemu_log("Virtuix: Kernel entry: 0x%016" PRIx64 ", highest addr: 0x%016"
             PRIx64 " (phys 0x%08" PRIx64 ")\n",
             kernel_entry, kernel_high, kernel_high_phys);

    write_kernel_trampoline((uint32_t)kernel_entry);

    arcs_dev = qdev_new(TYPE_SGI_ARCS);
    qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(arcs_dev, "kernel-end", (uint32_t)kernel_high_phys);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
    memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                        &SGI_ARCS(arcs_dev)->iomem, 10);

    arcs = SGI_ARCS(arcs_dev);
    sgi_arcs_setup_stubs(arcs, &address_space_memory);
  }
}

static void sgi_virtuix_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "Virtuix (IP55) - SGI Indy-derived virtualization-native SMP";
  mc->init = sgi_virtuix_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 256 * MiB;
  mc->default_ram_id = "sgi.ram";
  /* R5000 = MIPS IV so IP55 runs the full (mips4) nekoware catalog. Same
   * R4x00/UP-era lineage + software-managed caches as the R4600 we ran SMP on,
   * and the IP22/IP55 kernel already compiles the R5000 errata WARs; the R5000
   * code paths are FP/cache, not SMP coherency. (R10000/R12000 would trip
   * IS_R10000() into L2/speculation paths we don't model.) */
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R5000");
  mc->default_cpus = 1;
  mc->max_cpus = 32; /* IP55 SMP: paravirtual IPI via sgi-smp on env.irq[6] */
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static const TypeInfo sgi_virtuix_type = {
    .name = MACHINE_TYPE_NAME("virtuix"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_virtuix_class_init,
};

static void sgi_virtuix_machine_register(void) {
  type_register_static(&sgi_virtuix_type);
}

type_init(sgi_virtuix_machine_register)
