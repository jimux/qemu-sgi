/*
 * QEMU SGI Octane (IP30) machine emulation
 *
 * The SGI Octane is a uniprocessor (dual-CPU-capable) workstation based on the
 * HEART/BRIDGE/XIO chipset. It supports R10000/R12000 CPUs at 250-400MHz.
 *
 * XIO widget bus (16MB per widget):
 *   0x10000000-0x10FFFFFF  Xbow crossbar (widget 0)
 *   0x18000000-0x18FFFFFF  HEART XIO widget window (widget 8)
 *   0x1F000000-0x1FBFFFFF  BRIDGE (widget 0xF; PCI/IOC3/flash window)
 *   0x0FF00000-0x0FF6FFFF  HEART PIU (processor-side registers)
 *   0x1FC00000-0x1FCFFFFF  PROM flash (BRIDGE+0xC00000, standard MIPS vector)
 *   0x20000000-...         System RAM (SEG0)
 *
 * Interrupt routing:
 *   BRIDGE peripherals -> HEART ISR bits -> CPU IP3-IP7
 *
 * The `octane` machine below is the authentic IP30 bring-up. The bogus
 * `sgi-ip55` type (an IP54-era paravirtual carrier) is kept isolated in
 * sgi_ip54pv_init() and must not leak into the authentic machine.
 *
 * References:
 *   - MAME src/mame/sgi/octane.cpp
 *   - Linux arch/mips/sgi-ip30/
 *   - IRIX sys/RACER/heart.h, sys/RACER/bridge.h, sys/PCI/ioc3.h
 *   - resolved-notes/platform/ip30-octane.md
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "cpu.h"
#include "hw/char/serial.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/display/sgi_glaccel.h"
#include "hw/misc/sgi_heart.h"
#include "hw/misc/sgi_bridge.h"
#include "hw/misc/sgi_pvaudio.h"
#include "hw/misc/sgi_pvmem.h"
#include "hw/misc/sgi_pvnet.h"
#include "hw/misc/sgi_smp.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/* Octane physical address map (XIO widget bus) */
#define OCTANE_RAM_BASE    0x20000000ULL   /* System RAM (SEG0) */
#define OCTANE_HEART_BASE  0x0FF00000ULL   /* HEART PIU (processor regs) */
#define OCTANE_HEART_WIDGET 0x18000000ULL  /* HEART XIO widget 8 window */
#define OCTANE_BRIDGE_BASE 0x1F000000ULL   /* BRIDGE widget 0xF */
#define OCTANE_XBOW_BASE   0x10000000ULL   /* Xbow crossbar widget 0 */
#define OCTANE_PROM_BASE   0x1FC00000ULL   /* PROM (BRIDGE+0xC00000) */
#define OCTANE_PROM_SIZE   (1 * MiB)       /* IP30 PROM is 1MB */
#define OCTANE_RAM_MAX     (128ULL * GiB)

/* Paravirtual device base addresses in GIO64 expansion space (sgi-ip55) */
#define OCTANE_PV_BASE     0x1F480000ULL
#define OCTANE_PV_SMP      (OCTANE_PV_BASE + 0x000)
#define OCTANE_PV_MEM      (OCTANE_PV_BASE + 0x100)
#define OCTANE_PV_NET      (OCTANE_PV_BASE + 0x200)
#define OCTANE_PV_GLACCEL  (OCTANE_PV_BASE + 0x300)
#define OCTANE_PV_AUDIO    (OCTANE_PV_BASE + 0x400)

#define OCTANE_PV_NET_IRQ_BIT     20
#define OCTANE_PV_GLACCEL_IRQ_BIT 21
#define OCTANE_PV_AUDIO_IRQ_BIT   22

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

/* Load the PROM image into the ROM region. */
static void sgi_octane_load_prom(MachineState *machine, MemoryRegion *prom)
{
    char *filename = NULL;
    int bios_size;

    if (machine->kernel_filename) {
        return; /* Direct kernel boot: no PROM needed */
    } else if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip30prom.bin");
        if (!filename) {
            warn_report("No firmware specified, use -bios to specify Octane PROM");
            return;
        }
    }

    bios_size = load_image_targphys(filename, OCTANE_PROM_BASE,
                                    OCTANE_PROM_SIZE, NULL);
    g_free(filename);
    if (bios_size < 0) {
        error_report("Could not load PROM image");
        exit(EXIT_FAILURE);
    }
}

/*
 * Authentic IP30 (Octane) bring-up: HEART + Xbow + BRIDGE, uniprocessor.
 */
static void sgi_octane_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    DeviceState *heart_dev;
    DeviceState *bridge_dev;
    Clock *cpuclk;

    if (machine->ram_size > OCTANE_RAM_MAX) {
        error_report("RAM size more than 128GB is not supported");
        exit(EXIT_FAILURE);
    }

    /* R10000 @ 300MHz default. */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 300000000);

    machine->smp.cpus = 1;
    MIPSCPU *cpu = mips_cpu_create_with_clock(MIPS_CPU_TYPE_NAME("R10000"),
                                              cpuclk, true);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    qemu_register_reset(main_cpu_reset, cpu);

    /* HEART PIU at 0x0FF00000. */
    heart_dev = qdev_new(TYPE_SGI_HEART);
    qdev_prop_set_uint32(heart_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(heart_dev, "num-cpus", 1);
    object_property_set_link(OBJECT(heart_dev), "mem", OBJECT(machine->ram),
                             &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(heart_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(heart_dev), 0, OCTANE_HEART_BASE);
    /* MEMCFG-decoded memory-probe window at 0xA0000000 (sysbus region 1). */
    sysbus_mmio_map(SYS_BUS_DEVICE(heart_dev), 1, HEART_PROBE_BASE);

    /*
     * HEART interrupt outputs -> CPU IP3-IP7. Level 4 (errors/widget) -> IP7,
     * level 3 (timer) -> IP6, level 2 -> IP5, level 1 -> IP4, level 0 -> IP3.
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 0, cpu->env.irq[7]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 1, cpu->env.irq[6]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 2, cpu->env.irq[5]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 3, cpu->env.irq[4]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 4, cpu->env.irq[3]);

    /* BRIDGE (widget 0xF) at 0x1F000000, 12MB covering PCI/IOC3/devio. */
    bridge_dev = qdev_new(TYPE_SGI_BRIDGE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(bridge_dev), 0, OCTANE_BRIDGE_BASE);

    /* PROM at 0x1FC00000 (1MB). */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi.prom", OCTANE_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, OCTANE_PROM_BASE, prom);
    sgi_octane_load_prom(machine, prom);

    /* System RAM at 0x20000000 (SEG0; first 512MB). */
    if (machine->ram_size <= (512 * MiB)) {
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE,
                                    machine->ram);
    } else {
        MemoryRegion *seg0 = g_new(MemoryRegion, 1);
        memory_region_init_alias(seg0, OBJECT(machine), "seg0-ram",
                                 machine->ram, 0, 512 * MiB);
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE, seg0);
    }

    /*
     * The HEART memory-probe window at 0xA0000000 is owned by the HEART device
     * (see sgi_heart.c): it decodes the per-bank MEMCFG base/size and forwards
     * to the installed RAM. SEG0 RAM is mapped flat above for normal operation.
     */

    /*
     * Xbow crossbar at widget 0 (0x10000000). The PROM's pon_xbow POST runs
     * register read/write (walking-bit) tests before anything else and halts
     * into the fault-LED path if a scratch register does not retain its value.
     * A zero-initialised writable region satisfies those tests for bring-up;
     * it is a scaffold, not the real Xbow register model (widget IDs and link
     * status still read 0).
     */
    {
        MemoryRegion *xbow = g_new(MemoryRegion, 1);
        memory_region_init_ram(xbow, NULL, "sgi.xbow", 16 * MiB,
                               &error_fatal);
        memory_region_add_subregion(system_memory, OCTANE_XBOW_BASE, xbow);
    }

    /* HEART XIO widget-8 window (XIO config side, not the PIU). */
    create_unimplemented_device("heart-widget", OCTANE_HEART_WIDGET, 16 * MiB);

    /*
     * Low physical alias of RAM. The PROM copies its resident code/data to the
     * RAM "physical" base, which on this machine resolves to physical 0 (the
     * KSEG0/K1 view), and then executes it there via the fixed KSEG0 mapping.
     * Without this alias those writes land in the void and the CPU fetches
     * zeros. Sized to stop below the HEART PIU at 0x0FF00000.
     */
    {
        MemoryRegion *low = g_new(MemoryRegion, 1);
        uint64_t low_size = MIN(machine->ram_size, (uint64_t)0x0FF00000ULL);

        memory_region_init_alias(low, NULL, "sgi.seg0-low", machine->ram, 0,
                                 low_size);
        memory_region_add_subregion(system_memory, 0x00000000ULL, low);
    }
}

/*
 * Legacy IP54-era paravirtual carrier, registered as the bogus `sgi-ip55`
 * machine type. Not authentic IP30 silicon; kept only so the old type keeps
 * launching. Do not add IP54 devices to the authentic `octane` machine.
 */
static void sgi_ip54pv_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    DeviceState *heart_dev;
    DeviceState *bridge_dev;
    DeviceState *smp_dev;
    DeviceState *pvmem_dev;
    DeviceState *pvnet_dev;
    DeviceState *glaccel_dev;
    Clock *cpuclk;
    char *filename;
    int bios_size;
    int ncpus = machine->smp.cpus;

    if (machine->ram_size > OCTANE_RAM_MAX) {
        error_report("RAM size more than 128GB is not supported");
        exit(EXIT_FAILURE);
    }

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 300000000);

    MIPSCPU **cpus = g_new0(MIPSCPU *, ncpus);
    for (int i = 0; i < ncpus; i++) {
        cpus[i] = mips_cpu_create_with_clock(MIPS_CPU_TYPE_NAME("R10000"),
                                             cpuclk, true);
        cpu_mips_irq_init_cpu(cpus[i]);
        cpu_mips_clock_init(cpus[i]);

        if (i == 0) {
            qemu_register_reset(main_cpu_reset, cpus[i]);
        } else {
            CPUState *cs = CPU(cpus[i]);
            cs->start_powered_off = true;
            qemu_register_reset(main_cpu_reset, cpus[i]);
        }
    }

    heart_dev = qdev_new(TYPE_SGI_HEART);
    qdev_prop_set_uint32(heart_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(heart_dev, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(heart_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(heart_dev), 0, OCTANE_HEART_BASE);

    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 0, cpus[0]->env.irq[7]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 1, cpus[0]->env.irq[6]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 2, cpus[0]->env.irq[5]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 3, cpus[0]->env.irq[4]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 4, cpus[0]->env.irq[3]);

    bridge_dev = qdev_new(TYPE_SGI_BRIDGE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(bridge_dev), 0, OCTANE_BRIDGE_BASE);

    /* Paravirtual device bank (0x1f480000-0x1f4807ff). */
    smp_dev = qdev_new(TYPE_SGI_SMP);
    qdev_prop_set_uint32(smp_dev, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smp_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(smp_dev), 0, OCTANE_PV_SMP);
    {
        SGISMPState *smp = SGI_SMP(smp_dev);
        for (int i = 0; i < ncpus && i < SGI_SMP_MAXCPU; i++) {
            smp->cpus[i] = CPU(cpus[i]);
        }
    }

    pvmem_dev = qdev_new(TYPE_SGI_PVMEM);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pvmem_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pvmem_dev), 0, OCTANE_PV_MEM);

    pvnet_dev = qdev_new(TYPE_SGI_PVNET);
    qemu_configure_nic_device(pvnet_dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pvnet_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pvnet_dev), 0, OCTANE_PV_NET);
    sysbus_connect_irq(SYS_BUS_DEVICE(pvnet_dev), 0,
                       qdev_get_gpio_in(heart_dev, OCTANE_PV_NET_IRQ_BIT));

    glaccel_dev = qdev_new(TYPE_SGI_GLACCEL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(glaccel_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(glaccel_dev), 0, OCTANE_PV_GLACCEL);
    sysbus_connect_irq(SYS_BUS_DEVICE(glaccel_dev), 0,
                       qdev_get_gpio_in(heart_dev, OCTANE_PV_GLACCEL_IRQ_BIT));

    {
        DeviceState *pvaudio_dev = qdev_new(TYPE_SGI_PVAUDIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvaudio_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvaudio_dev), 0, OCTANE_PV_AUDIO);
        sysbus_connect_irq(SYS_BUS_DEVICE(pvaudio_dev), 0,
                           qdev_get_gpio_in(heart_dev, OCTANE_PV_AUDIO_IRQ_BIT));
    }

    create_unimplemented_device("pv-expansion",
                                OCTANE_PV_AUDIO + SGI_PVAUDIO_MMIO_SIZE,
                                0x8000 - 0x500);

    /* PROM at 0x1fc00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi.prom", OCTANE_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, OCTANE_PROM_BASE, prom);

    if (machine->kernel_filename) {
        filename = NULL;
    } else if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip30prom.bin");
        if (!filename) {
            warn_report("No firmware specified, use -bios to specify Octane PROM");
            filename = NULL;
        }
    }

    if (filename) {
        bios_size = load_image_targphys(filename, OCTANE_PROM_BASE,
                                        OCTANE_PROM_SIZE, NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load PROM image");
            exit(EXIT_FAILURE);
        }
    }

    if (machine->ram_size <= (512 * MiB)) {
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE,
                                    machine->ram);
    } else {
        MemoryRegion *seg0 = g_new(MemoryRegion, 1);
        memory_region_init_alias(seg0, OBJECT(machine), "seg0-ram",
                                 machine->ram, 0, 512 * MiB);
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE, seg0);
    }

    create_unimplemented_device("xbow", OCTANE_XBOW_BASE, 16 * MiB);
    create_unimplemented_device("mem-probe", 0x00000000, 512 * KiB);
}

static void sgi_octane_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI Octane (IP30)";
    mc->init = sgi_octane_init;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "sgi.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
    mc->default_cpus = 1;
    mc->max_cpus = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static void sgi_ip54_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI IP54 Paravirtual Workstation";
    mc->init = sgi_ip54pv_init;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "sgi.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
    mc->default_cpus = 1;
    mc->max_cpus = 128;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static const TypeInfo sgi_octane_type = {
    .name = MACHINE_TYPE_NAME("octane"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_octane_class_init,
};

static const TypeInfo sgi_ip55_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip55"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip54_class_init,
};

static void sgi_octane_machine_init(void)
{
    type_register_static(&sgi_octane_type);
    type_register_static(&sgi_ip55_type);
}

type_init(sgi_octane_machine_init)
