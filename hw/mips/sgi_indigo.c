/*
 * QEMU SGI Indigo (IP20) machine emulation
 *
 * The Indigo R4000 (IP20) is a uniprocessor workstation using:
 *   - MIPS R4000 CPU
 *   - MC (Memory Controller)
 *   - HPC1.5 (High Performance Peripheral Controller) with INT2
 *   - GIO64 bus for graphics/expansion
 *
 * This machine is deliberately separate from sgi_indy.c (IP22/IP24/...):
 * IP20 has its own peripheral controller (HPC1.5, not HPC3), its own
 * interrupt multiplexor (INT2, not INT3/IOC2) and its own register
 * layout.  Keeping it in its own file guarantees -M indy is untouched.
 *
 * Memory map:
 *   0x00000000-0x0007ffff  Alias for first 512KB of RAM
 *   0x08000000-0x17ffffff  Low System Memory (256MB max)
 *   0x1f000000-0x1f3fffff  GIO64 - GFX
 *   0x1f400000-0x1f5fffff  GIO64 - EXP0
 *   0x1f600000-0x1f9fffff  GIO64 - EXP1
 *   0x1fa00000-0x1fa1ffff  Memory Controller (MC)
 *   0x1fb80000-0x1fb8ffff  HPC1.5 (SCSI/ENET/DUART/INT2/RTC)
 *   0x1fbd0000-0x1fbd0003  Board revision register
 *   0x1fbe0000-0x1fbfffff  DSP RAM (128KB)
 *   0x1fc00000-0x1fc7ffff  PROM (512KB)
 *   0x20000000-0x2fffffff  High System Memory
 *
 * References:
 *   - MAME src/mame/sgi/ip20.cpp, hpc1.cpp, int2.cpp
 *   - IP20 PROM source: stand/arcs/IP20prom
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "cpu.h"
#include "hw/char/serial.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_hpc1.h"
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

/* Memory map addresses */
#define SGI_RAM_LOW_BASE    0x08000000ULL
#define SGI_RAM_HIGH_BASE   0x20000000ULL
#define SGI_GIO_GFX_BASE    0x1f000000ULL
#define SGI_GIO_EXP0_BASE   0x1f400000ULL
#define SGI_GIO_EXP1_BASE   0x1f600000ULL
#define SGI_MC_BASE         0x1fa00000ULL
#define SGI_HPC1_BASE       0x1fb80000ULL
#define SGI_BOARD_REV_BASE  0x1fbd0000ULL
#define SGI_DSP_RAM_BASE    0x1fbe0000ULL
#define SGI_DSP_RAM_SIZE    (128 * KiB)
#define SGI_PROM_BASE       0x1fc00000ULL
#define SGI_PROM_SIZE       (512 * KiB)

#define SGI_RAM_MAX         (256 * MiB)

/* IP20 board revision (MAME ip20.cpp: 0x8000) */
#define SGI_IP20_BOARD_REV  0x8000

/*
 * GIO empty slot: return all-ones so the PROM concludes no device is
 * present, rather than a bus timeout/error.
 */
typedef struct {
    MemoryRegion iomem;
    const char *name;
} GIOEmptySlotState;

static uint64_t gio_empty_slot_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0xffffffffffffffffULL;
}

static void gio_empty_slot_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
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
                                  hwaddr base, hwaddr size)
{
    GIOEmptySlotState *s = g_new0(GIOEmptySlotState, 1);

    s->name = name;
    memory_region_init_io(&s->iomem, NULL, &gio_empty_slot_ops, s, name, size);
    memory_region_add_subregion(sysmem, base, &s->iomem);
}

/* Board revision register: always reads 0x8000 on IP20. */
static uint64_t sgi_ip20_board_rev_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    return SGI_IP20_BOARD_REV;
}

static const MemoryRegionOps sgi_ip20_board_rev_ops = {
    .read = sgi_ip20_board_rev_read,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sgi_indigo_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    MemoryRegion *board_rev;
    MemoryRegion *dsp_ram;
    DeviceState *mc_dev;
    DeviceState *hpc1_dev;
    MIPSCPU *cpu;
    Clock *cpuclk;
    char *filename;
    int bios_size;

    if (machine->ram_size > SGI_RAM_MAX) {
        error_report("IP20 (Indigo) RAM size more than 256MB is not supported");
        exit(EXIT_FAILURE);
    }

    /* R4000 Indigo runs at 50 MHz; the modeled clock only sets CP0 Count. */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 50000000);

    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    /* PROM at 0x1fc00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi.prom", SGI_PROM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, SGI_PROM_BASE, prom);

    if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip20prom.bin");
        if (!filename) {
            warn_report("No IP20 firmware specified, use -bios to specify it");
        }
    }

    if (filename) {
        bios_size = load_image_targphys(filename, SGI_PROM_BASE, SGI_PROM_SIZE,
                                        NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load IP20 PROM image");
            exit(EXIT_FAILURE);
        }
    }

    /* Memory Controller */
    mc_dev = qdev_new(TYPE_SGI_MC);
    qdev_prop_set_uint32(mc_dev, "ram-size", machine->ram_size);
    object_property_set_link(OBJECT(mc_dev), "ram", OBJECT(machine->ram),
                             &error_fatal);
    object_property_set_link(OBJECT(mc_dev), "system-memory",
                             OBJECT(system_memory), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mc_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mc_dev), 0, SGI_MC_BASE);

    /* HPC1.5 */
    hpc1_dev = qdev_new(TYPE_SGI_HPC1);
    qdev_prop_set_chr(hpc1_dev, "chardev", serial_hd(0));
    qdev_prop_set_string(hpc1_dev, "nvram", "sgi_indigo_nvram.bin");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(hpc1_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(hpc1_dev), 0, SGI_HPC1_BASE);

    /* Attach SCSI drives from the command line */
    scsi_bus_legacy_handle_cmdline(&SGI_HPC1(hpc1_dev)->scsi->bus);

    /* INT2 interrupt lines -> CPU IP2/IP3, PIT timers -> IP4/IP5 */
    qdev_connect_gpio_out_named(hpc1_dev, "cpu-irq", 0, cpu->env.irq[2]);
    qdev_connect_gpio_out_named(hpc1_dev, "cpu-irq", 1, cpu->env.irq[3]);
    qdev_connect_gpio_out_named(hpc1_dev, "timer-irq", 0, cpu->env.irq[4]);
    qdev_connect_gpio_out_named(hpc1_dev, "timer-irq", 1, cpu->env.irq[5]);

    /* Board revision register */
    board_rev = g_new(MemoryRegion, 1);
    memory_region_init_io(board_rev, NULL, &sgi_ip20_board_rev_ops, NULL,
                          "ip20-board-rev", 4);
    memory_region_add_subregion(system_memory, SGI_BOARD_REV_BASE, board_rev);

    /* DSP RAM (plain RAM) */
    dsp_ram = g_new(MemoryRegion, 1);
    memory_region_init_ram(dsp_ram, NULL, "ip20-dsp-ram", SGI_DSP_RAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, SGI_DSP_RAM_BASE, dsp_ram);

    /* GIO slots: empty (unknown devices read all-ones) */
    create_gio_empty_slot(system_memory, "gio-gfx", SGI_GIO_GFX_BASE, 4 * MiB);
    create_gio_empty_slot(system_memory, "gio-exp0", SGI_GIO_EXP0_BASE, 2 * MiB);
    create_gio_empty_slot(system_memory, "gio-exp1", SGI_GIO_EXP1_BASE, 4 * MiB);

    /*
     * Memory probe areas: the MC dynamically maps RAM aliases over these.
     * Unmapped banks read as zero instead of causing a bus error.
     */
    create_unimplemented_device("low-mem-probe", SGI_RAM_LOW_BASE, 256 * MiB);
    create_unimplemented_device("high-mem-probe", SGI_RAM_HIGH_BASE, 256 * MiB);
    create_unimplemented_device("zero-mem-probe", 0x00000000, 512 * KiB);
    create_unimplemented_device("extended-mem-probe0", 0x18000000, 0x07000000);
    create_unimplemented_device("extended-mem-probe1", 0x30000000, 0x50000000);
}

static void sgi_indigo_class_init(ObjectClass *oc, const void *data)
{
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

static const TypeInfo sgi_indigo_type = {
    .name = MACHINE_TYPE_NAME("sgi-indigo"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo_class_init,
};

static const TypeInfo sgi_indigo_alias_type = {
    .name = MACHINE_TYPE_NAME("indigo"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_indigo_class_init,
};

static void sgi_indigo_machine_init(void)
{
    type_register_static(&sgi_indigo_type);
    type_register_static(&sgi_indigo_alias_type);
}

type_init(sgi_indigo_machine_init)
