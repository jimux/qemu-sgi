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
#include "system/address-spaces.h"
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
#include "hw/display/sgi_gl2.h"
#include "qemu/timer.h"
#include "qapi/error.h"

#define TYPE_IRIS3130_MACHINE MACHINE_TYPE_NAME("iris3130")
OBJECT_DECLARE_SIMPLE_TYPE(IRIS3130MachineState, IRIS3130_MACHINE)

struct IRIS3130MachineState {
    MachineState parent;

    M68kCPU cpu;
    bool irq_pending[5];
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
    { 3, 0x43 },        /* GF2 FBC retrace -> multibus 3 (67)         */
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
    if (getenv("SGI_IP2IRQ_TRACE")) {
        fprintf(stderr, "IP2IRQ n=%d lvl=%d pend=[%d%d%d%d%d] best=%d "
                "-> set %d/%#x sr=%#x\n",
                n, level, s->irq_pending[0], s->irq_pending[1],
                s->irq_pending[2], s->irq_pending[3], s->irq_pending[4],
                best, best < 0 ? 0 : iris3130_ip2_lines[best].level,
                best < 0 ? 0 : iris3130_ip2_lines[best].vector,
                (unsigned)s->cpu.env.sr);
    }
}

bool sgi_gl2_user_read(uint32_t *pc_out)
{
    M68kCPU *cpu = (M68kCPU *)current_cpu;

    if (!cpu || (cpu->env.sr & 0x2000)) {
        return false;
    }
    if (pc_out) {
        *pc_out = cpu->env.pc;
    }
    return true;
}

static QEMUTimer *sgi_pc_timer;

static void sgi_pc_sample(void *opaque)
{
    M68kCPU *cpu = opaque;
    static int kern = -1;
    static int interval = -1;

    if (kern < 0) {
        const char *iv = getenv("SGI_PC_SAMPLE_MS");
        kern = getenv("SGI_PC_SAMPLE_KERN") != NULL;
        interval = iv ? atoi(iv) : 100;
    }
    if (!(cpu->env.sr & 0x2000)) {   /* user mode: supervisor bit clear */
        fprintf(stderr, "PCSAMPLE pc=%08x sr=%04x\n",
                cpu->env.pc, cpu->env.sr);
    } else if (kern) {
        fprintf(stderr, "PCSAMPLEK pc=%08x sr=%04x\n",
                cpu->env.pc, cpu->env.sr);
    }
    timer_mod(sgi_pc_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + interval);
}

static QEMUTimer *sgi_shmd_timer;

static uint32_t sgi_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

/*
 * Diagnostic: dump the GL shared-memory handshake word as seen through the
 * kernel's gl_shmemptr and through the client's fixed USER SHMEM address,
 * so a mismatch (two different pages) is visible.  Off unless SGI_SHMD.
 */
static void sgi_shmd(void *opaque)
{
    M68kCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    uint8_t b[4];
    uint32_t shmptr = 0, shmpa = 0;
    unsigned kw = 0, uw = 0;
    int krc = -1, urc;

    if (cpu_memory_rw_debug(cs, 0x2006f418, b, 4, 0) == 0) {
        shmptr = sgi_be32(b);
    }
    if (cpu_memory_rw_debug(cs, 0x2006ef30, b, 4, 0) == 0) {
        shmpa = sgi_be32(b);
    }
    if (shmptr) {
        krc = cpu_memory_rw_debug(cs, shmptr, b, 2, 0);
        if (krc == 0) {
            kw = (b[0] << 8) | b[1];
        }
    }
    urc = cpu_memory_rw_debug(cs, 0x1fffe000, b, 2, 0);
    if (urc == 0) {
        uw = (b[0] << 8) | b[1];
    }
    fprintf(stderr, "SHMD shmptr=%08x pa=%08x kernEOF=%u(rc%d) "
            "userEOF=%u(rc%d)\n", shmptr, shmpa, kw, krc, uw, urc);
    timer_mod(sgi_shmd_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 200);
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
    /*
     * The IP2 bus must report an unmodelled Multibus I/O access as a bus
     * error, not a zero read, so the PROM's board probe and the kernel's
     * autoconfig conclude an absent board is absent.  The window lives here,
     * in the machine, not in shared target/m68k code; the Multibus I/O
     * segment is 0x50000000..0x5fffffff.
     */
    s->cpu.env.bus_error_decode_base = 0x50000000;
    s->cpu.env.bus_error_decode_size = 0x10000000;
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
     * Diagnostic: sample the CPU program counter on a wall-clock timer so a
     * guest that spins in user mode (making no syscalls) still reveals its
     * hot loop.  A guest in supervisor mode is skipped.  Off unless
     * SGI_PC_SAMPLE is set in the environment.
     */
    if (getenv("SGI_PC_SAMPLE")) {
        sgi_pc_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sgi_pc_sample,
                                    &s->cpu);
        timer_mod(sgi_pc_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
    }
    if (getenv("SGI_SHMD")) {
        sgi_shmd_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sgi_shmd,
                                      &s->cpu);
        timer_mod(sgi_shmd_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 200);
    }

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

    /*
     * GL2 graphics: the GF2/UC4/DC4 register window lives in the Multibus
     * I/O segment at MBIO + 0x2000. The keyboard (DUART0 channel A) is what
     * makes the PROM open this console (see stand/mon/init.c con_config).
     */
    {
        DeviceState *gl2 = qdev_new(TYPE_SGI_GL2);

        object_property_add_child(OBJECT(machine), "gl2", OBJECT(gl2));
        sysbus_realize(SYS_BUS_DEVICE(gl2), &error_fatal);
        memory_region_add_subregion_overlap(sgi_ip2_mbio_region(board),
                                            0x2000,
                                            sgi_gl2_mmio_region(gl2), 1);
        /*
         * The GF2 FBC vertical-retrace interrupt is Multibus level 3, which
         * the IP2 vector ROM maps to vector 0x43 -> Xmbintr3 -> ivectors[3]
         * = fbc_intr (sys/ipII/evec.h, sys/ipII/locore.c, autoconf.c).  It is
         * NOT the disk's level 5 / 0x45.
         */
        sysbus_connect_irq(SYS_BUS_DEVICE(gl2), 0, irq[4]);
        /* The GF2 Geometry Engine command pipe (GEPORT/GETOKEN). */
        memory_region_add_subregion_overlap(get_system_memory(), 0x60000000,
                                            sgi_gl2_ge_region(gl2), 1);
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
