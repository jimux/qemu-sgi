/*
 * QEMU SGI O2 (IP32) machine emulation
 *
 * The SGI O2 is a uniprocessor workstation based on the CRIME/MACE/GBE
 * chipset. It supports R5000, R10000, R12000, and R14000 CPUs.
 *
 * Memory map:
 *   0x00000000-0x0FFFFFFF  Main memory (256MB max)
 *   0x14000000-0x140002FF  CRIME base (CPU, memory, interrupts, timer)
 *   0x15000000-0x15004FFF  CRIME RE (rendering engine, TLBs, pixel pipe)
 *   0x16000000-0x160FFFFF  GBE (graphics framebuffer)
 *   0x17000000-0x170003FF  Virtual boot disk
 *   0x1F000000-0x1FFFFFFF  MACE (peripherals)
 *   0x1FC00000-0x1FC7FFFF  PROM (512KB, overlaps MACE region)
 *
 * Interrupt routing:
 *   MACE peripherals → CRIME INTSTAT bits [15:0] → CPU IP2 (IRQ 2)
 *
 * References:
 *   - MAME src/mame/sgi/ip32.cpp, crime.cpp, mace.cpp
 *   - IRIX IP32 PROM source
 *   - Linux arch/mips/sgi-ip32/
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_crime.h"
#include "hw/misc/sgi_crime_re.h"
#include "hw/misc/sgi_mace.h"
#include "hw/misc/sgi_bootdisk.h"
#include "hw/display/sgi_gbe.h"
#include "hw/misc/unimp.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/char/serial.h"
#include "hw/char/serial-mm.h"
#include "system/system.h"
#include "system/reset.h"
#include "cpu.h"
#include "qemu/log.h"

/* O2 memory map addresses */
#define O2_RAM_BASE           0x00000000ULL
#define O2_CRIME_BASE         0x14000000ULL
#define O2_CRIME_RE_BASE      0x15000000ULL
#define O2_BOOTDISK_BASE      0x17000000ULL
#define O2_GBE_BASE           0x16000000ULL
#define O2_MACE_BASE          0x1F000000ULL
#define O2_PROM_BASE          0x1FC00000ULL

#define O2_PROM_SIZE          (512 * KiB)
#define O2_RAM_MAX            (256 * MiB)

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

static void sgi_o2_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    DeviceState *crime_dev;
    DeviceState *mace_dev;
    DeviceState *gbe_dev;
    MIPSCPU *cpu;
    Clock *cpuclk;
    char *filename;
    int bios_size;

    /* Validate RAM size */
    if (machine->ram_size > O2_RAM_MAX) {
        error_report("RAM size more than 256MB is not supported");
        exit(EXIT_FAILURE);
    }

    /* Create CPU clock (R5000 @ 180MHz default) */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 180000000);

    /* Create CPU */
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    qemu_register_reset(main_cpu_reset, cpu);

    /*
     * RAM at physical 0x00000000.
     * IP32 has RAM starting at address 0 (unlike IP24 which starts at 0x08000000).
     * machine->ram is created automatically by QEMU via default_ram_id.
     */
    memory_region_add_subregion(system_memory, O2_RAM_BASE, machine->ram);

    /* PROM at 0x1FC00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi-o2.prom", O2_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, O2_PROM_BASE, prom);

    /* Load PROM/BIOS */
    if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip54.bin");
        if (!filename) {
            warn_report("No firmware specified, use -bios to specify IP32 PROM");
            filename = NULL;
        }
    }

    if (filename) {
        bios_size = load_image_targphys(filename, O2_PROM_BASE,
                                        O2_PROM_SIZE, NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load PROM image");
            exit(EXIT_FAILURE);
        }
    }

    /* CRIME at 0x14000000 */
    crime_dev = qdev_new(TYPE_SGI_CRIME);
    qdev_prop_set_uint32(crime_dev, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(crime_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(crime_dev), 0, O2_CRIME_BASE);

    /* Wire CRIME interrupt output to CPU IP2 (IRQ index 2) */
    sysbus_connect_irq(SYS_BUS_DEVICE(crime_dev), 0, cpu->env.irq[2]);

    /* CRIME Rendering Engine at 0x15000000 */
    {
        DeviceState *crime_re_dev = qdev_new(TYPE_SGI_CRIME_RE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(crime_re_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(crime_re_dev), 0, O2_CRIME_RE_BASE);
    }

    /* MACE at 0x1F000000 */
    mace_dev = qdev_new(TYPE_SGI_MACE);
    /* Don't pass chardev to MACE — serial port 0 is handled by serial_mm below */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mace_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mace_dev), 0, O2_MACE_BASE);

    /*
     * MACE serial port 0 (console) using QEMU's built-in 16550 device.
     *
     * The MACE serial port is a 16550-compatible UART with 256-byte register
     * spacing (regshift=8). We create a serial_mm device and add its memory
     * region as a subregion of the MACE iomem. Subregions take priority over
     * the parent, so serial port accesses go to serial_mm instead of the
     * MACE handler. This gives us proper TX buffering and chardev interaction
     * that the custom MACE serial code lacks.
     *
     * Serial port 0 is at MACE_BASE + 0x390000 = physical 0x1F390000.
     */
    if (serial_hd(0)) {
        MemoryRegion *mace_mr = sysbus_mmio_get_region(
            SYS_BUS_DEVICE(mace_dev), 0);
        serial_mm_init(mace_mr, MACE_SER1_OFFSET, 8,
                       NULL, 115200, serial_hd(0), DEVICE_BIG_ENDIAN);
    }

    /*
     * Wire MACE interrupt outputs to CRIME interrupt inputs.
     * MACE has 16 interrupt lines that map to CRIME INTSTAT bits [15:0].
     */
    {
        int i;
        for (i = 0; i < 16; i++) {
            qdev_connect_gpio_out_named(mace_dev, "crime-irq", i,
                                        qdev_get_gpio_in(crime_dev, i));
        }
    }

    /*
     * Virtual Boot Disk at 0x15000000.
     * Simple MMIO block device for PROM to load kernels without SCSI.
     * Attach via: -drive file=disk.img,if=mtd,format=raw
     */
    {
        DeviceState *bootdisk_dev;
        DriveInfo *dinfo;

        bootdisk_dev = qdev_new(TYPE_SGI_BOOTDISK);
        /* Try multiple interfaces: mtd, pflash, then scsi bus=0 unit=1 */
        dinfo = drive_get(IF_MTD, 0, 0);
        if (!dinfo) {
            dinfo = drive_get(IF_PFLASH, 0, 0);
        }
        if (!dinfo) {
            /* MCP tool uses -drive if=scsi,bus=0,unit=1 for first disk */
            dinfo = drive_get(IF_SCSI, 0, 1);
        }
        if (dinfo) {
            qdev_prop_set_drive_err(bootdisk_dev, "drive",
                                    blk_by_legacy_dinfo(dinfo),
                                    &error_fatal);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bootdisk_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(bootdisk_dev), 0, O2_BOOTDISK_BASE);
    }

    /* GBE at 0x16000000 */
    gbe_dev = qdev_new(TYPE_SGI_GBE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gbe_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(gbe_dev), 0, O2_GBE_BASE);

    /*
     * Unimplemented device stubs for memory probing.
     * The PROM probes memory by writing patterns and reading back.
     * Unmapped regions need to return 0 (pattern mismatch) instead
     * of causing bus errors.
     *
     * Cover the gaps in the physical address space that the PROM
     * might probe. The RAM region at 0x00000000 handles actual RAM;
     * these cover addresses beyond the installed RAM size.
     */
    if (machine->ram_size < O2_RAM_MAX) {
        create_unimplemented_device("mem-probe-high",
                                    O2_RAM_BASE + machine->ram_size,
                                    O2_RAM_MAX - machine->ram_size);
    }

    /*
     * Program loading RAM at 0x10000000.
     * IRIX standalone programs (sash, ide, etc.) are ECOFF binaries linked
     * at text_start=0x10000000. The PROM loads them here via identity-mapped
     * kuseg. This 32MB region provides backing memory for program loading
     * regardless of the PROM-detected RAM size.
     */
    {
        MemoryRegion *loader_ram = g_new(MemoryRegion, 1);
        memory_region_init_ram(loader_ram, NULL, "sgi-o2.loader-ram",
                               32 * MiB, &error_fatal);
        memory_region_add_subregion(system_memory, 0x10000000, loader_ram);
    }

    /*
     * Cover the gap between loader RAM and CRIME (0x12000000-0x13FFFFFF).
     * Also cover gaps between CRIME and GBE, GBE and MACE, etc.
     */
    create_unimplemented_device("gap-loader-crime",
                                0x12000000, 0x02000000);
    /* Gap between CRIME base regs and CRIME RE */
    create_unimplemented_device("gap-crime-re",
                                0x14000280, 0x00FFFD80);
    /* Gap between CRIME RE and GBE */
    create_unimplemented_device("gap-re-gbe",
                                0x15005000, 0x00FFB000);
    /* Gap between GBE and bootdisk */
    create_unimplemented_device("gap-gbe-bootdisk",
                                0x16100000, 0x00F00000);
    /* Gap between bootdisk and MACE */
    create_unimplemented_device("gap-bootdisk-mace",
                                0x17000400, 0x07FFFC00);

    /*
     * Cover the gap after MACE (0x1F400000-0x1FFFFFFF).
     * The PROM ROM subregion at 0x1FC00000 takes priority, but the
     * kernel scans one word past the end of PROM (0x1FC80000) looking
     * for "SHDR" markers. Without this, that read causes a bus error.
     */
    create_unimplemented_device("gap-post-mace",
                                0x1F400000, 0x00C00000);
}

static void sgi_o2_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI O2 (IP32)";
    mc->init = sgi_o2_init;
    mc->block_default_type = IF_MTD;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "sgi-o2.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R5000");
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
}

static const TypeInfo sgi_o2_type = {
    .name = MACHINE_TYPE_NAME("sgi-o2"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_o2_class_init,
};

static void sgi_o2_machine_init(void)
{
    type_register_static(&sgi_o2_type);
}

type_init(sgi_o2_machine_init)
