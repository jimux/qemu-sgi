/*
 * Silicon Graphics IRIS 3130 (IP2, 68020, GL2-W3.x).
 *
 * Hardware-authentic machine: the period IP2 boot PROMs and the GL2-W3.6 OS
 * are fixed binaries and the real IP2 register model is presented. The frame
 * of reference is MAME's `iris3130` driver, which is the phase-1 oracle.
 *
 * Copyright (c) 2026
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "exec/cpu-common.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/char/mc68681.h"
#include "hw/misc/sgi_ip2.h"
#include "qapi/error.h"

#define TYPE_IRIS3130_MACHINE MACHINE_TYPE_NAME("iris3130")
OBJECT_DECLARE_SIMPLE_TYPE(IRIS3130MachineState, IRIS3130_MACHINE)

struct IRIS3130MachineState {
    MachineState parent;

    M68kCPU cpu;
    bool irq_pending[4];
};

static void iris3130_cpu_reset(void *opaque)
{
    M68kCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);
    /*
     * The 68020 takes SP from address 0 and PC from address 4; the IP2 boot
     * mirror presents the system segment there at reset.
     */
    cpu->env.aregs[7] = ldl_phys(cs->as, 0);
    cpu->env.pc = ldl_phys(cs->as, 4);
}

/*
 * The IP2 has no autovectored interrupt controller: every source gets a
 * user-defined vector from the U118 vector ROM (sys/ipII/evec.h) and asserts
 * a CPU IPL. The m68k CPU tracks only one pending interrupt, so keep per-line
 * state here and hand it the highest asserted source -- a lower line
 * deasserting must not clear a higher one.
 */
static const struct {
    int level;
    uint8_t vector;
} iris3130_ip2_lines[] = {
    { 6, 0x53 },        /* MC146818A RTC periodic -> Xclock (83)      */
    { 5, 0x45 },        /* 2190 disk completion -> multibus 5 (69)    */
    { 6, 0x50 },        /* DUART0 -> uart0 (80)                       */
    { 6, 0x51 },        /* DUART1 -> uart1 (81)                       */
};

static void iris3130_ip2_irq(void *opaque, int n, int level)
{
    IRIS3130MachineState *s = opaque;
    int best = -1;
    int i;

    s->irq_pending[n] = level != 0;

    for (i = 0; i < ARRAY_SIZE(iris3130_ip2_lines); i++) {
        if (s->irq_pending[i] &&
            (best < 0 ||
             iris3130_ip2_lines[i].level > iris3130_ip2_lines[best].level)) {
            best = i;
        }
    }
    if (best < 0) {
        m68k_set_irq_level(&s->cpu, 0, 0);
    } else {
        m68k_set_irq_level(&s->cpu, iris3130_ip2_lines[best].level,
                           iris3130_ip2_lines[best].vector);
    }
}

static void iris3130_init(MachineState *machine)
{
    IRIS3130MachineState *s = IRIS3130_MACHINE(machine);
    DeviceState *board;
    MemoryRegion *sys;
    Chardev *serial[4];
    qemu_irq *irq;
    int i;

    object_initialize_child(OBJECT(machine), "cpu", &s->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&s->cpu), NULL, &error_fatal);
    qemu_register_reset(iris3130_cpu_reset, &s->cpu);

    board = qdev_new(TYPE_SGI_IP2);
    qdev_prop_set_string(board, "prom",
                         machine->firmware ? machine->firmware : "");

    DriveInfo *dinfo = drive_get(IF_NONE, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(board, "drive", blk_by_legacy_dinfo(dinfo));
    }

    sysbus_realize(SYS_BUS_DEVICE(board), &error_fatal);
    sys = sgi_ip2_sys_region(board);

    /* IP2 interrupt lines -> CPU (level/vector per the U118 vector ROM). */
    irq = qemu_allocate_irqs(iris3130_ip2_irq, s,
                             ARRAY_SIZE(iris3130_ip2_lines));
    sysbus_connect_irq(SYS_BUS_DEVICE(board), 0, irq[0]);       /* RTC   */
    sysbus_connect_irq(SYS_BUS_DEVICE(board), 1, irq[1]);       /* 2190  */

    /* Install the IP2 custom-MMU translation fast path on the CPU. */
    s->cpu.env.ext_tlb_fill = sgi_ip2_ext_tlb_fill;
    s->cpu.env.ext_tlb_opaque = SGI_IP2(board);
    sgi_ip2_set_cpu(board, CPU(&s->cpu));

    /*
     * tty0 is DUART0 channel A (keyboard), tty1 DUART0 channel B (the
     * diagnostic/serial console), tty2/tty3 DUART1 A/B.
     */
    serial[0] = serial_hd(1);   /* tty0: keyboard    */
    serial[1] = serial_hd(0);   /* tty1: serial console (the -serial chardev) */
    serial[2] = serial_hd(2);   /* tty2 */
    serial[3] = serial_hd(3);   /* tty3 */

    for (i = 0; i < 2; i++) {
        DeviceState *d = qdev_new(TYPE_MC68681);
        g_autofree char *name = g_strdup_printf("duart%d", i);

        qdev_prop_set_chr(d, "chardev-a", serial[i * 2]);
        qdev_prop_set_chr(d, "chardev-b", serial[i * 2 + 1]);
        object_property_add_child(OBJECT(machine), name, OBJECT(d));
        sysbus_realize(SYS_BUS_DEVICE(d), &error_fatal);
        memory_region_add_subregion_overlap(sys,
                                            0x02000000 + i * 0x00800000,
                                            mc68681_mmio_region(d), 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, irq[2 + i]);   /* uart0/1 */
    }
}

static void iris3130_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char *const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68020"),
        NULL,
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Silicon Graphics IRIS 3130 (IP2)";

    mc->init = iris3130_init;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68020");
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    /*
     * This default RAM is declared for the machine core but is intentionally
     * NOT mapped by iris3130_init(): the IP2 board owns the guest RAM as its
     * own MemoryRegion ("ip2.ram", backed by SGIIP2State::ram, size
     * IP2_RAM_SIZE) at physical 0. Consequently -m sizes only this unmapped
     * region and is a no-op for the RAM the guest actually sees; the machine
     * RAM size is the board's fixed 4 MiB.
     */
    mc->default_ram_size = 4 * MiB;
    mc->default_ram_id = "iris3130.ram";
}

static const TypeInfo iris3130_machine_typeinfo = {
    .name          = MACHINE_TYPE_NAME("iris3130"),
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(IRIS3130MachineState),
    .class_init    = iris3130_machine_class_init,
};

static void iris3130_machine_register_types(void)
{
    type_register_static(&iris3130_machine_typeinfo);
}

type_init(iris3130_machine_register_types)
