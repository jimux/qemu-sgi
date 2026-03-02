/*
 * QEMU SGI Octane (IP30) machine emulation
 *
 * The SGI Octane is a uniprocessor/dual-CPU workstation based on the
 * HEART/BRIDGE/XIO chipset. It supports R10000/R12000 CPUs at 250-400MHz.
 *
 * XIO widget bus (MAIN_IO_SPACE = 0x10000000, 16MB per widget):
 *   0x10000000-0x17FFFFFF  Xbow crossbar (widget 0)
 *   0x0FF00000-0x0FF6FFFF  HEART PIU (processor-side registers)
 *   0x1F000000-0x1FBFFFFF  BRIDGE (widget 0xF, covers PCI/IOC3/flash window)
 *   0x1FC00000-0x1FCFFFFF  PROM flash (BRIDGE+0xC00000, standard MIPS vector)
 *   0x20000000-...         System RAM (XKPHYS for >512MB)
 *
 * Interrupt routing:
 *   BRIDGE peripherals → HEART ISR bits → CPU IP3-IP7
 *
 * References:
 *   - MAME src/mame/sgi/octane.cpp
 *   - Linux arch/mips/sgi-ip30/
 *   - IRIX sys/heart.h, sys/bridge.h
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

/* Octane physical address map (XIO widget bus, MAIN_IO_SPACE=0x10000000) */
#define OCTANE_RAM_BASE    0x20000000ULL   /* System RAM (SEG0) */
#define OCTANE_HEART_BASE  0x0FF00000ULL   /* HEART PIU (processor regs) */
#define OCTANE_BRIDGE_BASE 0x1F000000ULL   /* BRIDGE widget 0xF (MAIN_WIDGET(0xF)) */
#define OCTANE_XBOW_BASE   0x10000000ULL   /* Xbow crossbar widget 0 */
#define OCTANE_PROM_BASE   0x1FC00000ULL   /* PROM (BRIDGE+0xC00000) */
#define OCTANE_PROM_SIZE   (1 * MiB)       /* IP30 PROM is 1MB */
#define OCTANE_RAM_MAX     (128ULL * GiB)

/* Paravirtual device base addresses in GIO64 expansion space */
#define OCTANE_PV_BASE     0x1F480000ULL   /* PV device region */
#define OCTANE_PV_SMP      (OCTANE_PV_BASE + 0x000)  /* sgi-smp */
#define OCTANE_PV_MEM      (OCTANE_PV_BASE + 0x100)  /* sgi-pvmem */
#define OCTANE_PV_NET      (OCTANE_PV_BASE + 0x200)  /* sgi-pvnet */
#define OCTANE_PV_GLACCEL  (OCTANE_PV_BASE + 0x300)  /* sgi-glaccel */
#define OCTANE_PV_AUDIO    (OCTANE_PV_BASE + 0x400)  /* sgi-pvaudio */

/* HEART ISR bits for PV device IRQs */
#define OCTANE_PV_NET_IRQ_BIT     20   /* Level 1 (IP4) */
#define OCTANE_PV_GLACCEL_IRQ_BIT 21   /* Level 1 (IP4) */
#define OCTANE_PV_AUDIO_IRQ_BIT   22   /* Level 1 (IP4) */

static void main_cpu_reset(void *opaque) {
    MIPSCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

static void sgi_octane_init(MachineState *machine) {
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

    /* Validate RAM size */
    if (machine->ram_size > OCTANE_RAM_MAX) {
        error_report("RAM size more than 128GB is not supported");
        exit(EXIT_FAILURE);
    }

    /* Create CPU clock (R10000 @ 300MHz default) */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 300000000);

    /* Create N CPUs */
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

    /* Create HEART device */
    heart_dev = qdev_new(TYPE_SGI_HEART);
    qdev_prop_set_uint32(heart_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(heart_dev, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(heart_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(heart_dev), 0, OCTANE_HEART_BASE);

    /*
     * Wire all 5 HEART CPU IRQ outputs to CPU0 interrupt pins:
     *   cpu_irq[0] → CPU0 IP7 (level 4: errors/widget)
     *   cpu_irq[1] → CPU0 IP6 (level 3: timer)
     *   cpu_irq[2] → CPU0 IP5 (level 2: IPI/local)
     *   cpu_irq[3] → CPU0 IP4 (level 1: local)
     *   cpu_irq[4] → CPU0 IP3 (level 0: local)
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 0, cpus[0]->env.irq[7]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 1, cpus[0]->env.irq[6]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 2, cpus[0]->env.irq[5]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 3, cpus[0]->env.irq[4]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 4, cpus[0]->env.irq[3]);

    /* Create BRIDGE device */
    bridge_dev = qdev_new(TYPE_SGI_BRIDGE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(bridge_dev), 0, OCTANE_BRIDGE_BASE);

    /*
     * Paravirtual device bank (0x1f480000-0x1f4807ff)
     */

    /* SMP controller at PV_BASE+0x000 */
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

    /* PV memory info at PV_BASE+0x100 */
    pvmem_dev = qdev_new(TYPE_SGI_PVMEM);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pvmem_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pvmem_dev), 0, OCTANE_PV_MEM);

    /* PV network at PV_BASE+0x200, IRQ → HEART ISR bit 20 */
    pvnet_dev = qdev_new(TYPE_SGI_PVNET);
    qemu_configure_nic_device(pvnet_dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pvnet_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pvnet_dev), 0, OCTANE_PV_NET);
    sysbus_connect_irq(SYS_BUS_DEVICE(pvnet_dev), 0,
                       qdev_get_gpio_in(heart_dev, OCTANE_PV_NET_IRQ_BIT));

    /* GL accelerator at PV_BASE+0x300, IRQ → HEART ISR bit 21 */
    glaccel_dev = qdev_new(TYPE_SGI_GLACCEL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(glaccel_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(glaccel_dev), 0, OCTANE_PV_GLACCEL);
    sysbus_connect_irq(SYS_BUS_DEVICE(glaccel_dev), 0,
                       qdev_get_gpio_in(heart_dev, OCTANE_PV_GLACCEL_IRQ_BIT));

    /* PV audio at PV_BASE+0x400, IRQ → HEART ISR bit 22 */
    {
        DeviceState *pvaudio_dev = qdev_new(TYPE_SGI_PVAUDIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvaudio_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvaudio_dev), 0, OCTANE_PV_AUDIO);
        sysbus_connect_irq(SYS_BUS_DEVICE(pvaudio_dev), 0,
                           qdev_get_gpio_in(heart_dev, OCTANE_PV_AUDIO_IRQ_BIT));
    }

    /* Cover the rest of the PV expansion window beyond audio */
    create_unimplemented_device("pv-expansion",
                                OCTANE_PV_AUDIO + SGI_PVAUDIO_MMIO_SIZE,
                                0x8000 - 0x500);

    /* PROM at 0x1fc00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi.prom", OCTANE_PROM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, OCTANE_PROM_BASE, prom);

    /* Load PROM/BIOS */
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

    /* RAM at physical 0x20000000 (SEG0) */
    if (machine->ram_size <= (512 * MiB)) {
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE, machine->ram);
    } else {
        /* For >512MB RAM, map first 512MB at SEG0 */
        MemoryRegion *seg0 = g_new(MemoryRegion, 1);
        memory_region_init_alias(seg0, OBJECT(machine), "seg0-ram",
                                 machine->ram, 0, 512 * MiB);
        memory_region_add_subregion(system_memory, OCTANE_RAM_BASE, seg0);
    }

    /*
     * Xbow crossbar stub at widget 0 (0x10000000). The PROM enumerates
     * widgets via Xbow registers early in POST. Return 0 on reads to
     * prevent bus errors during widget discovery.
     */
    create_unimplemented_device("xbow", OCTANE_XBOW_BASE, 16 * MiB);

    /* Catch null-pointer accesses in early boot */
    create_unimplemented_device("mem-probe", 0x00000000, 512 * KiB);
}

static void sgi_octane_class_init(ObjectClass *oc, const void *data) {
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI Octane (IP30)";
    mc->init = sgi_octane_init;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "sgi.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
    mc->default_cpus = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static void sgi_ip54_class_init(ObjectClass *oc, const void *data) {
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI IP54 Paravirtual Workstation";
    mc->init = sgi_octane_init;
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

static const TypeInfo sgi_ip54_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip54"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip54_class_init,
};

static const TypeInfo sgi_ip55_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip55"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip54_class_init,
};

static void sgi_octane_machine_init(void) {
    type_register_static(&sgi_octane_type);
    type_register_static(&sgi_ip54_type);
    type_register_static(&sgi_ip55_type);
}

type_init(sgi_octane_machine_init)