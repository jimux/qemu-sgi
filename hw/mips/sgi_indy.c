/*
 * QEMU SGI IP20/IP22/IP24/IP26/IP28 machine emulation
 *
 * Supported machines:
 *   - Indy (IP24): R4600/R5000, MC/HPC3/IOC2-Guinness
 *   - Indigo2 (IP22): R4400, MC/HPC3/IOC2-Full House
 *   - Indigo2 Power (IP26): R8000, MC/HPC3/IOC2-Full House (stub)
 *   - Indigo2 Impact (IP28): R10000, MC/HPC3/IOC2-Full House (stub)
 *   - Indigo (IP20): R4000, MC/HPC1/INT2
 *
 * Memory map (shared by IP22/IP24/IP26/IP28):
 *   0x00000000-0x0007ffff  Alias for first 512KB of RAM
 *   0x08000000-0x17ffffff  Low System Memory (256MB max)
 *   0x1f000000-0x1f3fffff  GIO64 - GFX
 *   0x1f400000-0x1f5fffff  GIO64 - EXP0
 *   0x1f600000-0x1f9fffff  GIO64 - EXP1
 *   0x1fa00000-0x1fa1ffff  Memory Controller (MC)
 *   0x1fb80000-0x1fbfffff  HPC3
 *   0x1fc00000-0x1fc7ffff  PROM (512KB)
 *   0x20000000-0x2fffffff  High System Memory
 *
 * IP20 (Indigo) uses the same memory map but with HPC1 instead of HPC3
 * and INT2 instead of IOC2.
 *
 * References:
 *   - MAME src/mame/sgi/indy_indigo2.cpp
 *   - Linux arch/mips/sgi-ip22/
 *   - NetBSD sys/arch/sgimips/
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
#include "hw/display/sgi_newport.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_arcs.h"
#include "hw/misc/sgi_hpc1.h"
#include "hw/misc/sgi_hpc3.h"
#include "hw/misc/sgi_mc.h"
#include "hw/misc/unimp.h"
#include "hw/scsi/scsi.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/* Memory map addresses (shared across IP2x platforms) */
#define SGI_RAM_LOW_BASE 0x08000000ULL
#define SGI_RAM_LOW_ALIAS 0x00000000ULL
#define SGI_RAM_HIGH_BASE 0x20000000ULL
#define SGI_GIO_GFX_BASE 0x1f000000ULL
#define SGI_GIO_EXP0_BASE 0x1f400000ULL
#define SGI_GIO_EXP1_BASE 0x1f600000ULL
#define SGI_MC_BASE 0x1fa00000ULL
#define SGI_HPC3_BASE 0x1fb80000ULL
#define SGI_PROM_BASE 0x1fc00000ULL

/* Full House EISA I/O space (IP22/IP26/IP28) */
#define SGI_EISA_IO_BASE 0x00080000ULL
#define SGI_EISA_IO_SIZE (512 * KiB)

#define SGI_PROM_SIZE (512 * KiB)
#define SGI_RAM_MAX (2048 * MiB)

/* Forward declaration for kernel boot trampoline */
static void write_kernel_trampoline(uint32_t kernel_entry_32);

enum sgi_ip2x_model {
  SGI_IP20, /* Indigo */
  SGI_IP22, /* Indigo2 */
  SGI_IP24, /* Indy */
  SGI_IP26, /* Indigo2 Power (R8000) */
  SGI_IP28, /* Indigo2 Impact (R10000) */
};

/*
 * GIO Empty Slot Stub
 *
 * On real SGI hardware, reading from an empty GIO slot causes a bus timeout
 * after ~25us which triggers a CPU bus error exception. Rather than implement
 * full bus error handling, we return 0xffffffff which is what an undriven bus
 * typically reads as. This tells the PROM "no device present" without needing
 * exception handling.
 *
 * This is critical for graphics probing - the PROM probes for GR2 graphics
 * (used in Indigo/Crimson) by reading HQ2 registers at offset 0x6a000.
 * Returning 0 would make it think GR2 is present but not responding correctly.
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
  return 0xffffffffffffffffULL; /* All bits set = no device */
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

/*
 * Create a GIO empty slot memory region that returns 0xffffffff on reads.
 * This signals "no device present" to the PROM during graphics probing.
 */
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
 * Write a MIPS boot trampoline to the PROM area (physical 0x1FC00000).
 *
 * When -kernel is used, the PROM ROM region is empty. The CPU starts
 * at the reset vector 0xBFC00000 (kseg1 mapping of 0x1FC00000).
 * This trampoline:
 *   1. Clears BEV and ERL in CP0_Status (required before kernel entry)
 *   2. Sets a0=0, a1=0, a2=0 (ARCS argc/argv/environ)
 *   3. Sets sp to a valid stack in kseg0
 *   4. Jumps to the kernel entry point
 *
 * This approach is more reliable than overriding PC in the reset handler
 * because QEMU's Resettable mechanism may re-reset the CPU after
 * qemu_register_reset callbacks run.
 */
static void write_kernel_trampoline(uint32_t kernel_entry_32) {
  /*
   * MIPS instruction encoding helpers for COP0 instructions.
   * MFC0 rt, rd: 010000 00000 rt rd 00000 000000
   * MTC0 rt, rd: 010000 00100 rt rd 00000 000000
   */
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

  /* Register numbers */
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

  /*
   * Clear BEV (bit 22) and ERL (bit 2) in CP0_Status.
   * BEV=0: exception vectors in RAM (0x80000xxx)
   * ERL=0: normal operation mode
   */
  trampoline[i++] = cpu_to_be32(MIPS_MFC0(T0, CP0_STATUS));
  /* Build mask ~((1<<22)|(1<<2)) = 0xFFBFFFFB */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFBF));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xFFFB));
  trampoline[i++] = cpu_to_be32(MIPS_AND(T0, T0, T1));
  trampoline[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_STATUS));

  /*
   * Set a0=0 (argc), a1=0 (argv),
   * a2=environ pointer array (K0SEG address of ARCS_ENVIRON_PHYS).
   * The kernel's getargs() parses environ for "key=value" strings
   * like dbaud, console, cpufreq. Without this, kopt_find() returns
   * empty strings causing divide-by-zero in UART baud rate setup.
   */
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A0, 0)); /* move a0, zero */
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A1, 0)); /* move a1, zero */
  trampoline[i++] =
      cpu_to_be32(MIPS_LUI(A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) >> 16));
  trampoline[i++] =
      cpu_to_be32(MIPS_ORI(A2, A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) & 0xFFFF));

  /* Set stack pointer to 8MB into kseg0 (0x80800000) */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(SP, 0x8080));

  /* Jump to kernel entry point */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T0, entry_hi));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T0, T0, entry_lo));
  trampoline[i++] = cpu_to_be32(MIPS_JR(T0));
  trampoline[i++] = cpu_to_be32(MIPS_NOP); /* delay slot */

  rom_add_blob_fixed("kernel-trampoline", trampoline, i * 4, SGI_PROM_BASE);

  qemu_log("SGI: Wrote kernel trampoline at 0x%08x -> entry 0x%08x\n",
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

/*
 * Default PROM filenames per platform.
 * Used when no -bios is specified on the command line.
 */
static const char *sgi_default_prom_name(enum sgi_ip2x_model model) {
  switch (model) {
  case SGI_IP20:
    return "ip20prom.bin";
  case SGI_IP22:
    return "ip22prom.bin";
  case SGI_IP24:
    return "ip24prom.bin";
  case SGI_IP26:
    return "ip26prom.bin";
  case SGI_IP28:
    return "ip28prom.bin";
  default:
    return "ip24prom.bin";
  }
}

/*
 * Per-machine NVRAM backing file names.
 * Each machine model gets its own NVRAM file so settings don't collide.
 * Files are created in the current working directory on first run.
 */
static const char *sgi_nvram_name(enum sgi_ip2x_model model) {
  switch (model) {
  case SGI_IP24:
    return "sgi_indy_nvram.bin";
  case SGI_IP22:
    return "sgi_indigo2_nvram.bin";
  case SGI_IP28:
    return "sgi_indigo2_r10k_nvram.bin";
  case SGI_IP26:
    return "sgi_indigo2_r8k_nvram.bin";
  case SGI_IP20:
    return "sgi_indigo_nvram.bin";
  default:
    return "sgi_nvram.bin";
  }
}

static const char *sgi_model_name(enum sgi_ip2x_model model) {
  switch (model) {
  case SGI_IP20:
    return "IP20";
  case SGI_IP22:
    return "IP22";
  case SGI_IP24:
    return "IP24";
  case SGI_IP26:
    return "IP26";
  case SGI_IP28:
    return "IP28";
  default:
    return "IP2x";
  }
}

static void sgi_ip2x_init(MachineState *machine, enum sgi_ip2x_model model) {
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *prom;
  DeviceState *mc_dev;
  DeviceState *hpc3_dev = NULL;
  DeviceState *hpc1_dev = NULL;
  MIPSCPU *cpu;
  Clock *cpuclk;
  char *filename;
  int bios_size;
  bool is_fullhouse =
      (model == SGI_IP22 || model == SGI_IP26 || model == SGI_IP28);
  bool is_ip20 = (model == SGI_IP20);

  /* Validate RAM size */
  if (machine->ram_size > SGI_RAM_MAX) {
    error_report("RAM size more than 2GB is not supported");
    exit(EXIT_FAILURE);
  }

  /* Warn about CPU mismatch for IP26/IP28 */
  if (model == SGI_IP26) {
    warn_report("IP26 requires R8000 which is not available in QEMU. "
                "Using R4000 instead. PROM will fail CPU PRId check.");
  }

  /* Create CPU clock */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  clock_set_hz(cpuclk, 100000000); /* 100 MHz default */

  /* Create CPU */
  cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
  cpu_mips_irq_init_cpu(cpu);
  cpu_mips_clock_init(cpu);
  qemu_register_reset(main_cpu_reset, cpu);

  /*
   * RAM is NOT mapped statically. The MC dynamically maps RAM aliases
   * at SEG0 (0x08000000) and SEG1 (0x20000000) based on MEMCFG register
   * writes. This allows per-bank memory probing to work correctly:
   * each bank gets its own MemoryRegion alias, and unmapped regions
   * return 0 (via unimplemented device probe areas below).
   *
   * The MC reset handler sets up default MEMCFG0 to map bank 0 at SEG0,
   * so RAM is available at boot for exception vectors.
   */

  /* PROM at 0x1fc00000 */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_rom(prom, NULL, "sgi.prom", SGI_PROM_SIZE, &error_fatal);
  memory_region_add_subregion(system_memory, SGI_PROM_BASE, prom);

  /*
   * Load PROM/BIOS (not needed when using -kernel for direct boot).
   * When -kernel is specified, we skip PROM loading entirely since the
   * ARCS stubs provide the firmware interface the kernel needs.
   */
  if (machine->kernel_filename) {
    /* Direct kernel boot: no PROM needed */
    filename = NULL;
  } else if (machine->firmware) {
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename) {
      error_report("Could not find firmware '%s'", machine->firmware);
      exit(EXIT_FAILURE);
    }
  } else {
    /* Try default PROM name for this platform */
    filename =
        qemu_find_file(QEMU_FILE_TYPE_BIOS, sgi_default_prom_name(model));
    if (!filename) {
      warn_report("No firmware specified, use -bios to specify %s PROM",
                  sgi_model_name(model));
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

  /* Memory Controller at 0x1fa00000 */
  mc_dev = qdev_new(TYPE_SGI_MC);
  qdev_prop_set_uint32(mc_dev, "ram-size", machine->ram_size);
  if (is_fullhouse) {
    qdev_prop_set_bit(mc_dev, "has-eisa", true);
  }
  if (model == SGI_IP28) {
    qdev_prop_set_uint8(mc_dev, "revision", 5);
  }
  object_property_set_link(OBJECT(mc_dev), "ram", OBJECT(machine->ram),
                           &error_fatal);
  object_property_set_link(OBJECT(mc_dev), "system-memory",
                           OBJECT(system_memory), &error_fatal);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(mc_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(mc_dev), 0, SGI_MC_BASE);

  if (is_ip20) {
    /*
     * IP20 (Indigo): HPC1 + INT2 at 0x1fb80000
     *
     * HPC1 is the predecessor to HPC3 with a different register layout.
     * INT2 is integrated within the HPC1 device (at offset 0x1c0).
     * The same sub-devices (WD33C93, Z85C30, Seeq 8003) are present
     * but at HPC1 register offsets instead of HPC3 offsets.
     */
    hpc1_dev = qdev_new(TYPE_SGI_HPC1);
    qdev_prop_set_chr(hpc1_dev, "chardev", serial_hd(0));
    qdev_prop_set_string(hpc1_dev, "nvram", sgi_nvram_name(model));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(hpc1_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(hpc1_dev), 0, SGI_HPC3_BASE);

    /* Attach SCSI drives from command line */
    {
      SGIHPC1State *hpc1 = SGI_HPC1(hpc1_dev);
      scsi_bus_legacy_handle_cmdline(&hpc1->scsi->bus);
    }

    /*
     * DSP RAM at 0x1fbe0000-0x1fbfffff (128KB)
     * Used by the PROM for temporary storage during CPU speed detection
     * and by the DSP56001 audio processor. We model it as plain RAM.
     * Reference: MAME ip20.cpp maps this as "dsp_ram" (3xTC55328J-35).
     */
    {
      MemoryRegion *dsp_ram = g_new(MemoryRegion, 1);
      memory_region_init_ram(dsp_ram, NULL, "ip20-dsp-ram", 128 * KiB,
                             &error_fatal);
      memory_region_add_subregion(get_system_memory(), 0x1fbe0000, dsp_ram);
    }

    /*
     * Board revision register at 0x1fbd0000 and other misc I/O.
     * Use unimplemented-device to prevent bus errors.
     * The board rev value (0x8000) is returned by the HPC1 device
     * if the region is extended, but for now we just prevent DBE.
     */
    create_unimplemented_device("ip20-misc-io", 0x1fb90000, 0x50000);

    /*
     * Wire HPC1/INT2 interrupts to MIPS CPU IRQ lines:
     *   INT2 LIO0 → IP2 (env.irq[2])
     *   INT2 LIO1 → IP3 (env.irq[3])
     *   Timer 0   → IP4 (env.irq[4])
     *   Timer 1   → IP5 (env.irq[5])
     */
    qdev_connect_gpio_out_named(hpc1_dev, "cpu-irq", 0, cpu->env.irq[2]);
    qdev_connect_gpio_out_named(hpc1_dev, "cpu-irq", 1, cpu->env.irq[3]);
    qdev_connect_gpio_out_named(hpc1_dev, "timer-irq", 0, cpu->env.irq[4]);
    qdev_connect_gpio_out_named(hpc1_dev, "timer-irq", 1, cpu->env.irq[5]);
  } else {
    /* IP22/IP24/IP26/IP28: HPC3 + IOC2/INT3 at 0x1fb80000 */
    hpc3_dev = qdev_new(TYPE_SGI_HPC3);
    /* Connect serial port 0 to the first serial device */
    qdev_prop_set_chr(hpc3_dev, "chardev", serial_hd(0));
    /* Set board type: Guinness for IP24, Full House for IP22/IP26/IP28 */
    qdev_prop_set_uint8(hpc3_dev, "board-type",
                        is_fullhouse ? BOARD_IP22 : BOARD_IP24);
    /* IP26/IP28 use NVRAM revision 9 (IP22/IP24 default to 8) */
    if (model == SGI_IP26 || model == SGI_IP28) {
      qdev_prop_set_uint8(hpc3_dev, "nvram-rev", 9);
    }
    /* Set per-machine NVRAM backing file (only if not overridden via -global)
     */
    {
      SGIHPC3State *hpc3 = SGI_HPC3(hpc3_dev);
      if (!hpc3->nvram_filename) {
        qdev_prop_set_string(hpc3_dev, "nvram-file", sgi_nvram_name(model));
      }
    }
    /* Wire ethernet NIC from command line (-nic / -net nic) */
    {
      NICInfo *nd = qemu_find_nic_info(TYPE_SGI_HPC3, true, NULL);
      if (nd) {
        qdev_set_nic_properties(hpc3_dev, nd);
      }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(hpc3_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(hpc3_dev), 0, SGI_HPC3_BASE);

    /* Attach SCSI drives from command line to SCSI bus 0 */
    {
      SGIHPC3State *hpc3 = SGI_HPC3(hpc3_dev);
      scsi_bus_legacy_handle_cmdline(&hpc3->scsi[0]->bus);
    }

    /*
     * Wire HPC3 interrupts to MIPS CPU IRQ lines:
     *   INT3 Local0 (SCSI, ethernet) → IP2 (env.irq[2])
     *   INT3 Local1 (panel, DMA)     → IP3 (env.irq[3])
     *   PIT Timer 0 (sched clock)    → IP4 (env.irq[4])
     *   PIT Timer 1 (prof clock)     → IP5 (env.irq[5])
     */
    qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 0, cpu->env.irq[2]);
    qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 1, cpu->env.irq[3]);
    qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 0, cpu->env.irq[4]);
    qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 1, cpu->env.irq[5]);
  }

  /*
   * GIO slots - create empty slot stubs that return 0xffffffff on reads.
   * This signals "no device present" to the PROM during graphics probing.
   *
   * On real hardware, empty GIO slots cause bus timeout/error. We simulate
   * this by returning all 1s which is what an undriven bus typically reads.
   */

  /*
   * Newport graphics at GIO graphics slot (0x1f000000).
   * REX3 registers are at offset 0x0f0000 within the slot.
   * Create empty slot stubs for the parts not covered by Newport.
   */
  {
    DeviceState *newport_dev;

    /* Cover GIO slot area before Newport REX3 (0x1f000000-0x1f0effff) */
    create_gio_empty_slot(system_memory, "gio-gfx-low", SGI_GIO_GFX_BASE,
                          REX3_REG_OFFSET);

    /* Newport REX3 at 0x1f0f0000 */
    newport_dev = qdev_new(TYPE_SGI_NEWPORT);
    object_property_add_child(OBJECT(machine), "newport", OBJECT(newport_dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(newport_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(newport_dev), 0,
                    SGI_GIO_GFX_BASE + REX3_REG_OFFSET);

    /* Wire Newport VRINT → INT2/INT3 retrace interrupt */
    {
      DeviceState *irq_target = is_ip20 ? hpc1_dev : hpc3_dev;
      sysbus_connect_irq(SYS_BUS_DEVICE(newport_dev), 0,
                         qdev_get_gpio_in_named(irq_target, "gio-retrace", 0));
    }

    /* Cover GIO slot area after Newport REX3 (0x1f0f2000-0x1f3fffff) */
    create_gio_empty_slot(system_memory, "gio-gfx-high",
                          SGI_GIO_GFX_BASE + REX3_REG_OFFSET + REX3_REG_SIZE,
                          4 * MiB - REX3_REG_OFFSET - REX3_REG_SIZE);
  }

  create_gio_empty_slot(system_memory, "gio-exp0", SGI_GIO_EXP0_BASE, 2 * MiB);
  create_gio_empty_slot(system_memory, "gio-exp1", SGI_GIO_EXP1_BASE, 4 * MiB);

  /*
   * Full House machines (IP22/IP26/IP28) have EISA I/O space at 0x00080000.
   * An empty EISA bus returns 0xFFFFFFFF (no device present) on reads.
   */
  if (is_fullhouse) {
    create_gio_empty_slot(system_memory, "eisa-io", SGI_EISA_IO_BASE,
                          SGI_EISA_IO_SIZE);
  }

  /*
   * Memory probe areas - the PROM probes memory by writing to addresses
   * and reading back. To prevent bus errors when probing beyond actual RAM,
   * create unimplemented device stubs that return 0 on reads.
   *
   * The MC dynamically maps RAM aliases with priority 1, which overlay
   * these unimplemented devices (default priority -1000). When a bank
   * is unmapped, the probe area shows through, returning 0 on reads.
   *
   * SEG0: Full 256MB coverage at 0x08000000
   * SEG1: Full 256MB coverage at 0x20000000
   * Zero: 512KB at 0x00000000 (fallback for low alias)
   */
  create_unimplemented_device("low-mem-probe", SGI_RAM_LOW_BASE, 256 * MiB);
  create_unimplemented_device("high-mem-probe", SGI_RAM_HIGH_BASE, 256 * MiB);
  create_unimplemented_device("zero-mem-probe", 0x00000000, 512 * KiB);

  /* Map high RAM natively if > 256MB */
  if (machine->ram_size > 256 * MiB) {
    MemoryRegion *high_ram = g_new(MemoryRegion, 1);
    memory_region_init_alias(high_ram, NULL, "high-ram", machine->ram,
                             256 * MiB, machine->ram_size - 256 * MiB);
    /* PV-MEM advertises high_base = 0x20000000, map it there. */
    memory_region_add_subregion(system_memory, 0x20000000, high_ram);
  }

  /*
   * Extended probe areas for MIPS64 XKPHYS memory probing.
   * The IP28/IP26 PROMs use XKPHYS (0x9000_0000_xxxx_xxxx) to probe
   * physical addresses directly, bypassing TLB. The probe address is
   * base << 24, where base is the 8-bit MEMCFG field. This can reach
   * PA 0x00000000–0xFF000000. Cover the gaps so probes don't cause
   * bus errors.
   */
  create_unimplemented_device("extended-mem-probe0", 0x18000000,
                              0x07000000); /* 0x18M–0x1EFFFFFF */
  create_unimplemented_device("extended-mem-probe1", 0x30000000,
                              0x50000000); /* 0x30M–0x7FFFFFFF */

  /*
   * Direct kernel boot via -kernel flag.
   *
   * Loads an IRIX kernel ELF directly into memory and sets up minimal
   * ARCS firmware stubs so the kernel can call GetMemoryDescriptor,
   * GetEnvironmentVariable, Write, etc.
   *
   * This bypasses the PROM -> sash -> SCSI boot chain, allowing
   * kernel-level hardware bring-up without a working SCSI path.
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

    /* Convert kernel_high from virtual to physical */
    uint64_t kernel_high_phys = kernel_high & 0x1FFFFFFF;

    qemu_log("SGI: Loaded kernel '%s' (%ld bytes)\n", machine->kernel_filename,
             kernel_size);
    qemu_log("SGI: Kernel entry: 0x%016" PRIx64 ", highest addr: 0x%016" PRIx64
             " (phys 0x%08" PRIx64 ")\n",
             kernel_entry, kernel_high, kernel_high_phys);

    /*
     * Write a boot trampoline to the PROM area. The CPU starts
     * at the reset vector (0xBFC00000). The trampoline clears
     * BEV/ERL, sets up args, and jumps to the kernel entry.
     * This is more reliable than overriding PC in a reset callback
     * because QEMU's Resettable mechanism controls CPU reset order.
     */
    write_kernel_trampoline((uint32_t)kernel_entry);

    /*
     * Create ARCS hypercall MMIO device at 0x1F000100.
     * This sits in the GIO graphics slot space below where Newport
     * REX3 registers live (0x1F0F0000), in an area that returns
     * 0xFFFFFFFF from the empty slot stub. We need to create the
     * ARCS device before the empty slot covers this range, so we
     * place it as an overlapping region with higher priority.
     */
    arcs_dev = qdev_new(TYPE_SGI_ARCS);
    qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(arcs_dev, "kernel-end", (uint32_t)kernel_high_phys);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
    /*
     * Map with overlap priority so it takes precedence over the
     * gio-gfx-low empty slot region at the same address range.
     */
    memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                        &SGI_ARCS(arcs_dev)->iomem, 10);

    /* Set up SPB, FirmwareVector, stubs, memory descriptors, env vars */
    arcs = SGI_ARCS(arcs_dev);
    sgi_arcs_setup_stubs(arcs, &address_space_memory);
  }
}

/* Per-machine init wrappers */

static void sgi_indy_init(MachineState *machine) {
  sgi_ip2x_init(machine, SGI_IP24);
}

static void sgi_indigo2_init(MachineState *machine) {
  sgi_ip2x_init(machine, SGI_IP22);
}

static void sgi_indigo2_r8k_init(MachineState *machine) {
  sgi_ip2x_init(machine, SGI_IP26);
}

static void sgi_indigo2_r10k_init(MachineState *machine) {
  sgi_ip2x_init(machine, SGI_IP28);
}

static void sgi_indigo_init(MachineState *machine) {
  sgi_ip2x_init(machine, SGI_IP20);
}

/* Per-machine class_init functions */

static void sgi_indy_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Indy (IP24)";
  mc->init = sgi_indy_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R4600");
  mc->default_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static void sgi_indigo2_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Indigo2 (IP22)";
  mc->init = sgi_indigo2_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R4000");
  mc->default_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static void sgi_indigo2_r8k_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Indigo2 Power (IP26) [stub - R8000 not available]";
  mc->init = sgi_indigo2_r8k_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R4000");
  mc->default_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static void sgi_indigo2_r10k_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Indigo2 Impact (IP28)";
  mc->init = sgi_indigo2_r10k_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
  mc->default_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static void sgi_indigo_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Indigo (IP20)";
  mc->init = sgi_indigo_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R4000");
  mc->default_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

/* Machine type registration */

static const TypeInfo sgi_indy_type = {
    .name = MACHINE_TYPE_NAME("indy"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indy_class_init,
};

static const TypeInfo sgi_indigo2_type = {
    .name = MACHINE_TYPE_NAME("indigo2"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo2_class_init,
};

static const TypeInfo sgi_indigo2_r8k_type = {
    .name = MACHINE_TYPE_NAME("indigo2-r8k"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo2_r8k_class_init,
};

static const TypeInfo sgi_indigo2_r10k_type = {
    .name = MACHINE_TYPE_NAME("indigo2-r10k"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo2_r10k_class_init,
};

static const TypeInfo sgi_indigo_type = {
    .name = MACHINE_TYPE_NAME("indigo"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo_class_init,
};

static void sgi_machine_init(void) {
  type_register_static(&sgi_indy_type);
  type_register_static(&sgi_indigo2_type);
  type_register_static(&sgi_indigo2_r8k_type);
  type_register_static(&sgi_indigo2_r10k_type);
  type_register_static(&sgi_indigo_type);
}

type_init(sgi_machine_init)
