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
#include "hw/misc/sgi_ds2502.h"
#include "hw/misc/sgi_sflash.h"
#include "target/mips/internal.h"
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
#define OCTANE_MGRAS_WIDGET 0x19000000ULL  /* MGRAS/Impact graphics, XIO widget 9 */
#define OCTANE_PROM_BASE   0x1FC00000ULL   /* PROM (BRIDGE+0xC00000) */
#define OCTANE_FLASH_ALT_BASE 0x1FE00000ULL /* alt flash window (BRIDGE+0xE00000) */
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
    /*
     * Octane fit R10000 revs 2.6/2.7/3.0 (later R12000).  QEMU's shared R10000
     * model reports CP0_PRid 0x900 (rev 0.0), which the IP30 kernel rejects:
     * allowboot() panics when mpconf->pr_id < C0_MAKE_REVID(R10000,2,6), and
     * the PROM copies CP0_PRid into that MP config block.  Report rev 2.7 --
     * past the check, and a real Octane part; our R10000 model has no T5 bug,
     * so 2.7 (no HEART_INVALIDATE_WAR) is truthful rather than claiming 2.6's
     * errata.  Implementation byte (imp=9) is unchanged, so ABI resolution that
     * keys on the implementation field is unaffected.  Kept octane-local rather
     * than editing the shared cpu-defs.c.inc R10000 entry (used by IP28 and
     * the virtuix machine, which must stay as they are).
     */
    cpu->env.CP0_PRid = (0x09 << 8) | (2 << 4) | 7;    /* R10000 rev 2.7 */
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
 * Xbow crossbar (widget 0) link/presence model.
 *
 * The PROM's widget discovery (heart_install -> heart_do_port ->
 * xtalk_probe -> xlink_check) only registers a widget as a bus slot if the
 * xbow reports its link as WIDGET_PRESENT and LINK_ALIVE. Without a real
 * link model the Bridge (and hence IOC3/tty) is never discovered.
 *
 * Register layout from hwreg_xbow.hwreg: link base = 0x100 + 0x40*(port-8);
 * LINK_STAT +0x14 bit31 LINK_ALIVE, LINK_STAT_CLR +0x2c, LINK_AUX_STAT +0x3c
 * bit5 WIDGET_PRESENT / bit6 LINK_FAILURE.
 */
#define XBOW_REG_WORDS        (0x1000 / 4)
#define XBOW_LINK_BASE(port)  (0x100 + 0x40 * ((port) - 8))
#define XBOW_LINK_STAT_OFF    0x14
#define XBOW_LINK_STAT_CLR_OFF 0x2c
#define XBOW_LINK_AUX_OFF     0x3c
#define XB_STAT_LINKALIVE     0x80000000u
#define XB_AUX_STAT_PRESENT   0x00000020u
#define XBOW_WIDGET_PART_NUM  0x0000u

typedef struct SGIXbowRegs {
    uint32_t regs[XBOW_REG_WORDS];
} SGIXbowRegs;

static bool xbow_link_present(int port)
{
    /* XIO widget 9 (MGRAS/Impact graphics) is behind OCTANE_MGRAS: presenting
     * it makes the PROM discover and run the MGRAS init, which currently
     * panics on the unmodelled register file (I0 -> I1 boundary).  Off by
     * default so the met unattended-boot item stays green. */
    if (port == 9) {
        return getenv("OCTANE_MGRAS") != NULL;
    }
    return port == 8 || port == 0xf;
}

static uint64_t xbow_read(void *opaque, hwaddr off, unsigned size)
{
    SGIXbowRegs *s = opaque;
    uint32_t id = (1u << 28) | (XBOW_WIDGET_PART_NUM << 12); /* rev 1, part [27:12] */
    int p;

    if (off < 8) {
        return (off >= 4) ? id : 0; /* 64-bit w_id: low word carries the part */
    }
    for (p = 8; p <= 0xf; p++) {
        hwaddr lb = XBOW_LINK_BASE(p);

        if (!xbow_link_present(p)) {
            continue;
        }
        if (off == lb + XBOW_LINK_STAT_OFF ||
            off == lb + XBOW_LINK_STAT_CLR_OFF) {
            return XB_STAT_LINKALIVE;
        }
        if (off == lb + XBOW_LINK_AUX_OFF) {
            return XB_AUX_STAT_PRESENT;
        }
    }
    return s->regs[(off >> 2) & (XBOW_REG_WORDS - 1)];
}

static void xbow_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SGIXbowRegs *s = opaque;

    if (off < 8) {
        return; /* widget ID is read-only */
    }
    s->regs[(off >> 2) & (XBOW_REG_WORDS - 1)] = val;
}

static const MemoryRegionOps xbow_ops = {
    .read = xbow_read,
    .write = xbow_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

/*
 * HEART widget ID word (heart.h HEART_WID_ID): part 0xc001 at [27:12],
 * mfg 0x036 at [10:1], rev at [31:28].  The kernel's heart_rev() reads this
 * nibble and PANICs for rev <= HEART_REV_C ("Prototype Heart chip"), so report
 * a production rev: shipping Octanes are rev D or later.
 */
#define HEART_WID_ID_VAL 0x000000004c00106cULL

static uint64_t heart_widget_id_read(void *opaque, hwaddr off, unsigned size)
{
    if (off >= 8) {
        return 0;
    }
    if (size == 8) {
        return HEART_WID_ID_VAL;
    }
    /* 32-bit access: +0 is the high word, +4 the low word. */
    return (off == 0) ? (uint32_t)(HEART_WID_ID_VAL >> 32)
                      : (uint32_t)HEART_WID_ID_VAL;
}

static const MemoryRegionOps heart_widget_id_ops = {
    .read = heart_widget_id_read,
    .write = NULL,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

/*
 * MGRAS / IMPACT (HQ4) graphics widget-ID word, from sys/xtalk/hq4.h:
 * part 0xc003 at [27:12], mfg 0x2aa at [10:1], rev at [31:28].  The ARCS
 * graphics drivers match on mfg+part (MGRAS/mgras_init.c), so presenting this
 * word at the widget-1 window base is what makes the board discoverable.
 */
#define MGRAS_WID_ID_VAL 0x0000000010c003554ULL

static uint64_t mgras_widget_id_read(void *opaque, hwaddr off, unsigned size)
{
    if (off >= 8) {
        return 0;
    }
    if (size == 8) {
        return MGRAS_WID_ID_VAL;
    }
    /* 32-bit access: +0 is the high word, +4 the low word. */
    return (off == 0) ? (uint32_t)(MGRAS_WID_ID_VAL >> 32)
                      : (uint32_t)MGRAS_WID_ID_VAL;
}

static const MemoryRegionOps mgras_widget_id_ops = {
    .read = mgras_widget_id_read,
    .write = NULL,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/* Temporary I0 instrumentation: log every access to the MGRAS window so the
 * POST hang shows which register the PROM polls.  Offset 0x11098 is the HQ4
 * "number in a can" (sys/xtalk/hq4.h HQ4_WIDGET_NIC) MicroLAN register, read
 * by the widget probe with the same MCR protocol as the HEART/bridge. */
#define MGRAS_HQ4_NIC 0x11098

typedef struct SGIMgrasWin {
    SGIDS2502 nic;
    uint8_t reg[0x100000];   /* register-level bring-up store (mgras_hw) */
} SGIMgrasWin;

static uint64_t mgras_win_read(void *opaque, hwaddr off, unsigned size)
{
    SGIMgrasWin *s = opaque;
    unsigned i;
    uint64_t v = 0;

    if (off == MGRAS_HQ4_NIC || off == MGRAS_HQ4_NIC + 4) {
        return 0x2 | (s->nic.data_bit & 1);   /* MCR_DONE | MCR_DATA */
    }
    if (off + size <= sizeof(s->reg)) {
        for (i = 0; i < size; i++) {
            v = (v << 8) | s->reg[off + i];   /* big-endian, as the PROM reads */
        }
        return v;
    }
    qemu_log_mask(LOG_UNIMP, "mgras-win R off=0x%llx size=%u\n",
                  (unsigned long long)off, size);
    return 0;
}

static void mgras_win_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SGIMgrasWin *s = opaque;
    unsigned i;

    if (off == MGRAS_HQ4_NIC || off == MGRAS_HQ4_NIC + 4) {
        sgi_ds2502_mcr(&s->nic, val);
        return;
    }
    if (off + size <= sizeof(s->reg)) {
        for (i = 0; i < size; i++) {
            s->reg[off + i] = (val >> (8 * (size - 1 - i))) & 0xff;
        }
        return;
    }
    qemu_log_mask(LOG_UNIMP, "mgras-win W off=0x%llx val=0x%llx size=%u\n",
                  (unsigned long long)off, (unsigned long long)val, size);
}

static const MemoryRegionOps mgras_win_ops = {
    .read = mgras_win_read,
    .write = mgras_win_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

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
    MIPSCPU *octane_cpus[2];

    if (machine->ram_size > OCTANE_RAM_MAX) {
        error_report("RAM size more than 128GB is not supported");
        exit(EXIT_FAILURE);
    }

    /*
     * IP30 IRIX writes CP0_Compare a few counts past-due; the CP0 timer must
     * then fire immediately rather than defer a full Count wrap, or lbolt
     * freezes.  Opt this machine in (see #3404 / cpu_mips_timer_update()).
     */
    mips_cp0_fire_immediate = true;

    /* R10000 @ 300MHz default. */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 300000000);

    {
        int ncpus = machine->smp.cpus;

        if (ncpus > 2) {
            ncpus = 2;
        }
        machine->smp.cpus = ncpus;
        for (int i = 0; i < ncpus; i++) {
            MIPSCPU *c = mips_cpu_create_with_clock(
                MIPS_CPU_TYPE_NAME("R10000"), cpuclk, true);

            cpu_mips_irq_init_cpu(c);
            cpu_mips_clock_init(c);
            qemu_register_reset(main_cpu_reset, c);
            octane_cpus[i] = c;
        }
    }
    MIPSCPU *cpu = octane_cpus[0];

    /* HEART PIU at 0x0FF00000. */
    heart_dev = qdev_new(TYPE_SGI_HEART);
    qdev_prop_set_uint32(heart_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(heart_dev, "num-cpus", machine->smp.cpus);
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
    /*
     * HEART per-CPU interrupt outputs: [cpu][level] -> IP7/IP6/IP5/IP4/IP3.
     * Both CPUs get their own lines so vectors the kernel enables in imr[i]
     * are delivered on CPU i.
     */
    for (int c = 0; c < (int)machine->smp.cpus; c++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), c * 5 + 0,
                           octane_cpus[c]->env.irq[6]);
        sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), c * 5 + 1,
                           octane_cpus[c]->env.irq[5]);
        sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), c * 5 + 2,
                           octane_cpus[c]->env.irq[4]);
        sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), c * 5 + 3,
                           octane_cpus[c]->env.irq[3]);
        sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), c * 5 + 4,
                           octane_cpus[c]->env.irq[2]);
    }

    /* BRIDGE (widget 0xF) at 0x1F000000, 12MB covering PCI/IOC3/devio. */
    bridge_dev = qdev_new(TYPE_SGI_BRIDGE);
    qemu_configure_nic_device(bridge_dev, true, NULL);
    /*
     * Back the DS1687 NVRAM (the PROM's environment and its battery-backed
     * RAM) with a file so it persists across runs; without it the PROM
     * reports its NVRAM checksum bad and reinitializes the env and clock.
     * Honour OCTANE_NVRAM for tests.
     */
    {
        const char *nv = getenv("OCTANE_NVRAM");

        object_property_set_str(OBJECT(bridge_dev), "nvram",
                                nv ? nv : "sgi_octane_ds1687.bin",
                                &error_abort);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(bridge_dev), 0, OCTANE_BRIDGE_BASE);

    /* Device-line interrupts are sent to HEART as their b_int_addr[] vector. */
    SGI_BRIDGE(bridge_dev)->heart = SGI_HEART(heart_dev);

    /*
     * BRIDGE PCI interrupt aggregation -> HEART widget-error BASEIO vector
     * (IP30_HVEC_WIDERR_BASEIO = 57).  The two QLogic ISP channels are BaseIO
     * devices 0/1; the IRIX kernel ql driver is interrupt-driven (the ARCS
     * driver polls), so without this its INQUIRY never completes and no
     * target/lun/disk vertices appear under /hw/ql/0.
     */
    qdev_connect_gpio_out(DEVICE(bridge_dev), 0,
                          qdev_get_gpio_in(DEVICE(heart_dev), 57));

    /* PROM at 0x1FC00000 (1MB). */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi.prom", OCTANE_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, OCTANE_PROM_BASE, prom);
    sgi_octane_load_prom(machine, prom);

    /*
     * The PROM is the BRIDGE serial flash. The primary flash window is the
     * read-only ROM the CPU executes from; the flash command interface and the
     * writable PDS (NVRAM) segment live in the ALTERNATE flash window
     * (0x1FE00000), which flash_init falls back to when the primary part-ID
     * read does not answer. Alias the PROM image there so FPROM/RPROM header
     * reads resolve, then overlay the command window and PDS segment.
     */
    {
        MemoryRegion *alt = g_new(MemoryRegion, 1);
        DeviceState *sflash = qdev_new(TYPE_SGI_SFLASH);

        memory_region_init_alias(alt, NULL, "sgi.prom.alt", prom, 0,
                                 OCTANE_PROM_SIZE);
        memory_region_add_subregion(system_memory, OCTANE_FLASH_ALT_BASE, alt);

        object_property_set_link(OBJECT(sflash), "rom", OBJECT(prom),
                                 &error_abort);
        /*
         * Back the flash PDS (NVRAM) segment with a file so the PROM's
         * environment (OSLoadFilename, SystemPartition, ...) and its
         * initialized time-of-day clock persist across runs.  Without it the
         * segment starts erased and the PROM reinitializes both on every boot.
         * Honour OCTANE_PDS for tests; default to the Indy-style filename.
         */
        {
            const char *pds = getenv("OCTANE_PDS");

            object_property_set_str(OBJECT(sflash), "pds",
                                    pds ? pds : "sgi_octane_nvram.bin",
                                    &error_abort);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sflash), &error_fatal);
        /* Alternate-base command window + PDS segment (segment 15). */
        sysbus_mmio_map(SYS_BUS_DEVICE(sflash), 0, OCTANE_FLASH_ALT_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(sflash), 1,
                        OCTANE_FLASH_ALT_BASE + 0xF0000);
    }

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
     * Xbow crossbar at widget 0 (0x10000000). RAM-backed for the register
     * file, with the widget-ID and link status/presence registers overlaid so
     * the PROM's widget discovery can find the Bridge on link 0xF.
     */
    {
        MemoryRegion *xbow = g_new(MemoryRegion, 1);
        SGIXbowRegs *xr = g_new0(SGIXbowRegs, 1);

        memory_region_init_ram(xbow, NULL, "sgi.xbow", 16 * MiB,
                               &error_fatal);
        memory_region_add_subregion(system_memory, OCTANE_XBOW_BASE, xbow);

        {
            MemoryRegion *xbow_regs = g_new(MemoryRegion, 1);
            memory_region_init_io(xbow_regs, NULL, &xbow_ops, xr,
                                  "sgi.xbow-regs", 0x1000);
            memory_region_add_subregion(xbow, 0, xbow_regs);
        }
    }

    /*
     * HEART XIO widget-8 window (XIO config side, not the PIU).
     *
     * The window's first 8 bytes are the widget ID word HEART_WID_ID
     * (heart.h): bits [27:12] part 0xc001, [10:1] mfg 0x036, [31:28] rev.
     * The PROM's widget discovery reads it (64-bit at +0, low word at +4)
     * and will not identify the node IO widget without it, so the rest of
     * the 16MB window is RAM-backed and the ID is overlaid at offset 0.
     */
    {
        MemoryRegion *hw = g_new(MemoryRegion, 1);
        MemoryRegion *hw_id = g_new(MemoryRegion, 1);

        memory_region_init_ram(hw, NULL, "sgi.heart-widget", 16 * MiB,
                               &error_fatal);
        memory_region_add_subregion(system_memory, OCTANE_HEART_WIDGET, hw);

        memory_region_init_io(hw_id, NULL, &heart_widget_id_ops, NULL,
                              "sgi.heart-widget-id", 8);
        memory_region_add_subregion(hw, 0, hw_id);
    }

    /*
     * MGRAS / IMPACT (HQ4) graphics at XIO widget 9 (0x19000000).  Only the
     * widget-ID word is modelled for now: the xbow reports the link present
     * and this window answers with the HQ4 part/mfg, so the PROM's widget
     * discovery identifies an IMPACT board.  The rest of the 16MB window is
     * RAM-backed; the MGRAS register file (I1) will overlay it.
     */
    {
        MemoryRegion *mg = g_new(MemoryRegion, 1);
        MemoryRegion *mg_id = g_new(MemoryRegion, 1);

        if (getenv("OCTANE_MGRAS")) {
            static const uint8_t nic_serial[6] = { 0, 0, 0, 0, 0, 1 };
            SGIMgrasWin *mgs = g_new0(SGIMgrasWin, 1);

            sgi_ds2502_build_nic(&mgs->nic, nic_serial);
            memory_region_init_io(mg, NULL, &mgras_win_ops, mgs,
                                  "sgi.mgras-widget", 16 * MiB);
            memory_region_add_subregion(system_memory, OCTANE_MGRAS_WIDGET, mg);

            memory_region_init_io(mg_id, NULL, &mgras_widget_id_ops, NULL,
                                  "sgi.mgras-widget-id", 8);
            memory_region_add_subregion(mg, 0, mg_id);
        }
    }

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

    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 0, cpus[0]->env.irq[6]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 1, cpus[0]->env.irq[5]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 2, cpus[0]->env.irq[4]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 3, cpus[0]->env.irq[3]);
    sysbus_connect_irq(SYS_BUS_DEVICE(heart_dev), 4, cpus[0]->env.irq[2]);

    bridge_dev = qdev_new(TYPE_SGI_BRIDGE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(bridge_dev), 0, OCTANE_BRIDGE_BASE);

    /* Device-line interrupts are sent to HEART as their b_int_addr[] vector. */
    SGI_BRIDGE(bridge_dev)->heart = SGI_HEART(heart_dev);

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
    mc->max_cpus = 2;
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

static void sgi_ip55_class_init(ObjectClass *oc, const void *data)
{
    sgi_ip54_class_init(oc, data);
    /* The IP55 reuses the IP54 carrier but must not report the IP54 name:
     * the description is guest-visible (a config UI reading -M help would
     * otherwise configure an IP54). */
    MACHINE_CLASS(oc)->desc = "SGI IP55 Paravirtual Workstation";
}

static const TypeInfo sgi_octane_type = {
    .name = MACHINE_TYPE_NAME("octane"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_octane_class_init,
};

static const TypeInfo sgi_ip55_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip55"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip55_class_init,
};

static void sgi_octane_machine_init(void)
{
    type_register_static(&sgi_octane_type);
    type_register_static(&sgi_ip55_type);
}

type_init(sgi_octane_machine_init)
