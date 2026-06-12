/*
 * QEMU SGI IP54 Paravirtual Machine
 *
 * A pure paravirtual machine for the IP54 PROM and kernel. Unlike the
 * Octane (IP30) machine which brings in HEART/BRIDGE hardware, this machine
 * exposes only the three MMIO devices the IP54 PROM actually touches:
 *
 *   0x14000038  pvtimer  — 66MHz free-running counter (usecdelay)
 *   0x1F62017B  pvuart   — console UART byte registers (THR/RBR/LSR)
 *   0x1F480000+ PV bank  — sgi-smp, sgi-pvmem, sgi-pvnet, sgi-glaccel,
 *                          sgi-pvaudio
 *
 * Plus sgi-bootdisk at 0x17000000 for PROM disk I/O without full SCSI.
 *
 * IRQ routing — through HEART shim interrupt controller:
 *   pvnet   → HEART ISR bit 20 ─┐
 *   glaccel → HEART ISR bit 21 ─┼→ LEVEL1 (bits 16-31) → CPU IP4
 *   pvaudio → HEART ISR bit 22 ─┘
 *   IP7 (irq[7]): CP0 Count/Compare (automatic)
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "cpu.h"
#include "hw/char/sgi_pvuart.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/sgi_glaccel.h"
#include "hw/display/sgi_pvrex3.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_bootdisk.h"
#include "hw/misc/sgi_ioc2_kbd.h"
#include "hw/misc/sgi_pvaudio.h"
#include "hw/misc/sgi_pvmem.h"
#include "hw/misc/sgi_pvnet.h"
#include "hw/misc/sgi_pvtimer.h"
#include "hw/misc/sgi_smp.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "qapi/error.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "qemu/timer.h"

/* IP54PV physical address map */
#define IP54PV_RAM_BASE      0x08000000ULL  /* Main memory (SGI convention: RAM at 0x08000000) */
#define IP54PV_RAM_ALIAS     0x00000000ULL  /* Low alias of RAM (exception vectors, PROM data/BSS) */
#define IP54PV_TIMER_BASE    0x1F480500ULL  /* pvtimer (66MHz counter) — in PV bank */
#define IP54PV_BOOTDISK_BASE 0x1F480600ULL  /* sgi-bootdisk — in PV bank */
#define IP54PV_PROM_BASE     0x1FC00000ULL  /* PROM reset vector */
#define IP54PV_PROM_SIZE     (512 * KiB)
#define IP54PV_UART_BASE     0x1F620178ULL  /* pvuart (console) */

/* Paravirtual device bank */
#define IP54PV_PV_BASE       0x1F480000ULL
#define IP54PV_PV_SMP        (IP54PV_PV_BASE + 0x000)  /* sgi-smp */
#define IP54PV_PV_MEM        (IP54PV_PV_BASE + 0x100)  /* sgi-pvmem */
#define IP54PV_PV_NET        (IP54PV_PV_BASE + 0x200)  /* sgi-pvnet */
#define IP54PV_PV_GLACCEL    (IP54PV_PV_BASE + 0x300)  /* sgi-glaccel */
#define IP54PV_PV_AUDIO      (IP54PV_PV_BASE + 0x400)  /* sgi-pvaudio */

/* PVRex3 — Newport-compatible REX3 graphics at a separate 8KB region */
#define IP54PV_PV_REX3       0x1F490000ULL             /* sgi-pvrex3 (8KB REX3 regs) */

/* Max addressable via kseg0/kseg1 (2GB) */
#define IP54PV_RAM_MAX       (2ULL * GiB)

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    cpu_reset(CPU(cpu));
}

/* -----------------------------------------------------------------------
 * Paravirtual 100 Hz clock — delivers one clock() tick per 10ms by
 * writing cause_ip5_count=1 to guest RAM and asserting SW2.
 *
 * IRIX intr() IOC1 path: cause_ip5_count > 0 → synthesize IP5 → clock().
 *
 * WARNING — this PA is a GUEST KERNEL SYMBOL ADDRESS and MUST track the
 * running /unix.new.  It is cause_ip5_count's KSEG0 VA masked to physical
 * (VA & 0x1FFFFFFF).  A stale value here is catastrophic: this routine writes
 * to it 100x/second, so a wrong address is a relentless wild write that
 * silently corrupts whatever kernel variable now lives there (manifested as
 * the .dt-desktop zone_shake / kernel-stack / userspace crashes — see
 * progress_notes/ip54/dt_desktop_zone_corruption.md).  When the kernel is
 * rebuilt, re-derive from ip54_kernel_symbols_disk.json:
 *     cause_ip5_count VA 0x8829EDC0 → PA 0x0829EDC0  (golden disk kernel, 2026-06).
 * (Older builds had it at 0x8829ED00 / 0x8829F150 — those constants rotted.)
 * ----------------------------------------------------------------------- */
#define IP54PV_CAUSE_IP5_COUNT_PA  0x0829EDC0ULL
#define IP54PV_PVCLOCK_INTERVAL_NS 10000000ULL  /* 10 ms = 100 Hz */
#define IP54PV_PVCLOCK_LOWER_NS     2000000ULL  /* 2 ms lower pulse */
#define IP54PV_CAUSE_SW2_BIT       (1u << 9)    /* CP0 Cause SW2 */

typedef struct {
    QEMUTimer *raise_timer;
    QEMUTimer *lower_timer;
    CPUMIPSState *env;
} PVClockState;

/*
 * CP0_Cause bits 8-9 (SW0/SW1) are guest-writable: IRIX's softint
 * machinery RMWs them via mtc0 constantly (pokesoftclk runs on every
 * clock tick).  helper_mtc0_cause executes on the vCPU thread WITHOUT
 * the BQL, so mutating CP0_Cause from these iothread timer callbacks
 * raced it — lost updates could erase pending IP bits or resurrect
 * stale ones mid-exception-delivery, randomly corrupting interrupt
 * dispatch (processes resuming at EPC 0, spurious vectors).  All Cause
 * manipulation therefore runs ON the vCPU thread via async_run_on_cpu,
 * which executes at a translation-block boundary with the CPU halted.
 */
static void pvclock_raise_work(CPUState *cs, run_on_cpu_data data)
{
    PVClockState *s = data.host_ptr;

    /* Write cause_ip5_count = 1 into guest RAM (MIPS big-endian) */
    uint32_t one = cpu_to_be32(1);
    cpu_physical_memory_write(IP54PV_CAUSE_IP5_COUNT_PA, &one, sizeof(one));

    /* Assert SW2 to drive intr() at the next TB boundary */
    s->env->CP0_Cause |= IP54PV_CAUSE_SW2_BIT;
    cpu_interrupt(cs, CPU_INTERRUPT_HARD);
}

static void pvclock_lower_work(CPUState *cs, run_on_cpu_data data)
{
    PVClockState *s = data.host_ptr;
    /* Safety fallback: clear SW2 if pokesoftclk didn't do it */
    s->env->CP0_Cause &= ~IP54PV_CAUSE_SW2_BIT;
}

static void pvclock_lower_cb(void *opaque)
{
    PVClockState *s = opaque;
    async_run_on_cpu(env_cpu(s->env), pvclock_lower_work,
                     RUN_ON_CPU_HOST_PTR(s));
}

static void pvclock_raise_cb(void *opaque)
{
    PVClockState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    async_run_on_cpu(env_cpu(s->env), pvclock_raise_work,
                     RUN_ON_CPU_HOST_PTR(s));

    /* Schedule lower callback 2ms from now */
    timer_mod(s->lower_timer, now + IP54PV_PVCLOCK_LOWER_NS);
    /* Schedule next raise 10ms from now */
    timer_mod(s->raise_timer, now + IP54PV_PVCLOCK_INTERVAL_NS);
}

/* -----------------------------------------------------------------------
 * Diagnostic timer — logs CP0 Count/Compare/Cause/Status once per second
 * of virtual time to identify why hardware interrupts never fire.
 * Remove this once the timer interrupt issue is resolved.
 * ---------------------------------------------------------------------- */
typedef struct {
    QEMUTimer *timer;
    CPUMIPSState *env;
    int tick;
    uint32_t last_compare;
} IP54DiagTimerState;

static void ip54_diag_timer_cb(void *opaque)
{
    IP54DiagTimerState *s = opaque;
    CPUMIPSState *env = s->env;
    CPUState *cs = env_cpu(env);

    uint32_t count = env->CP0_Count +
        (uint32_t)clock_ns_to_ticks(env->count_clock,
                                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    uint32_t compare = env->CP0_Compare;
    uint32_t cause = env->CP0_Cause;
    uint32_t status = env->CP0_Status;
    int32_t ip_bits = (cause >> 8) & 0xFF;
    int32_t im_bits = (status >> 8) & 0xFF;
    bool ie = status & (1 << 0);
    bool exl = status & (1 << 1);
    bool erl = status & (1 << 2);
    bool cpu_irq_hard = cs->interrupt_request & CPU_INTERRUPT_HARD;
    uint64_t pc = env->active_tc.PC;
    int32_t cmp_delta = (int32_t)(compare - s->last_compare);

    fprintf(stderr, "[IP54-DIAG t=%d] Count=0x%08x Cmp=0x%08x "
            "diff=%d IP=0x%02x IM=0x%02x "
            "IE=%d EXL=%d ERL=%d HARD=%d PC=0x%016lx cmpD=%d\n",
            s->tick, count, compare,
            (int32_t)(compare - count),
            ip_bits, im_bits,
            ie, exl, erl, cpu_irq_hard,
            (unsigned long)pc, cmp_delta);

    s->last_compare = compare;
    s->tick++;
    /* Reschedule for next second */
    timer_mod(s->timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND);
}

/* -----------------------------------------------------------------------
 * Fake IP22 Memory Controller (MC) at 0x1FA00000
 *
 * The IP22-based kernel's szmem() reads MEMCFG0/MEMCFG1 here to discover
 * physical RAM.  Without this, the unimp device returns 0 for all MEMCFG
 * reads, physmem stays 0, and the kernel panics with IS_KSEG2(vaddr) in
 * tlbmiss() before setup_lowmem() can initialise putbuf.
 *
 * IP22 memory map: SEG0 at physical 0x08000000 (identical to IP54).
 * MEMCFG register 16-bit bank format:
 *   bits[12:8]  DRAM_MASK: (bank_MB/4 - 1) × 0x0100
 *   bits[7:0]   ADDR:      phys_base >> 22
 *
 * mc_rev_level=0 (SYSID returns 0) → addr_shift=22 → bank_addrlo uses <<22.
 * MEMCFG0[31:16]=bank0, MEMCFG0[15:0]=bank1.
 * ---------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * HEART counter compatibility shim at PA 0x0FF00000
 *
 * The IRIX IP30 kernel has hardcoded references to HEART_COUNT at
 * PA 0x0FF00080 that are dynamically installed (by _hook_exceptions or
 * similar) and cannot all be patched from the PROM.  This shim maps
 * the HEART address space and redirects h_count reads to the pvtimer.
 * ---------------------------------------------------------------------- */

#define IP54PV_HEART_BASE   0x0FF00000ULL
#define IP54PV_HEART_SIZE   0x00070000  /* Cover full PIU range */

#define IP54PV_HEART_CFG_BASE  0x18000000ULL  /* HEART widget config space */
#define IP54PV_HEART_CFG_SIZE  0x00001000     /* Widget registers (WID_ID, ERR_TYPE, etc.) */

/* HEART PIU register offsets */
#define HEART_OFF_MODE       0x00000
#define HEART_OFF_STATUS     0x00080
#define HEART_OFF_IMR0       0x10000
#define HEART_OFF_IMR1       0x10008
#define HEART_OFF_IMR2       0x10010
#define HEART_OFF_IMR3       0x10018
#define HEART_OFF_SET_ISR    0x10020
#define HEART_OFF_CLR_ISR    0x10028
#define HEART_OFF_ISR        0x10030
#define HEART_OFF_IMSR       0x10038
#define HEART_OFF_CAUSE      0x10040
#define HEART_OFF_COUNT      0x20000
#define HEART_OFF_COMPARE    0x30000
#define HEART_OFF_TRIGGER    0x40000
#define HEART_OFF_PRID       0x50000
#define HEART_OFF_SYNC       0x60000

typedef struct IP54HeartShimState {
    SGIPVTimerState *pvtimer;
    MemoryRegion *ram;       /* Underlying RAM for passthrough */
    uint64_t ram_offset;     /* Offset of HEART base within RAM region */
    uint64_t isr;            /* Interrupt Status Register */
    uint64_t imr[4];         /* Interrupt Mask Registers (per-CPU) */
    uint64_t compare;        /* HEART Compare register */
    qemu_irq cpu_irq_ip4;   /* Output to CPU IP4 (LEVEL1 interrupts) */
    qemu_irq cpu_irq_pvnet;   /* Output to CPU IP2 (pvnet dedicated) */
    uint64_t gpio_level;     /* Current GPIO input levels from PV devices */
    bool irq_level;          /* Current CPU IP4 output state (for edge detection) */
    bool irq_level_pvnet;     /* Current CPU IP2 output state (pvnet) */
} IP54HeartShimState;

/*
 * Update CPU interrupt lines based on ISR & IMR state.
 * HEART LEVEL1 interrupts (bits 16-31) map to CPU IP4.
 *
 * Only call qemu_irq_raise/lower on TRANSITIONS to avoid interfering
 * with other interrupt sources (pvclock SW2, Count/Compare) that share
 * the CPU_INTERRUPT_HARD flag.  Without edge detection, every GPIO
 * state change would bounce IP4 and potentially clear HARD set by
 * the pvclock, losing clock ticks.
 */
static void ip54_heart_update_irq(IP54HeartShimState *s)
{
    uint64_t imsr = s->isr & s->imr[0];  /* CPU 0 only */

    /* ISR bit 20 (pvnet) → dedicated CPU IP3 line.
     * Routed to IP3 (not IP4) because:
     *   IP4 → c0vec_tbl[5] (clock handler) which crashes in semaphore code
     *   IP3 → c0vec_tbl[4] (INT2/INT3 handler, safe to replace on IP54)
     * The PROM patches c0vec_tbl[4].isr = pvnet_intr. */
    bool pvnet_level = (imsr & (1ULL << 20)) != 0;
    if (pvnet_level != s->irq_level_pvnet) {
        s->irq_level_pvnet = pvnet_level;
        if (pvnet_level) {
            qemu_irq_raise(s->cpu_irq_pvnet);
        } else {
            qemu_irq_lower(s->cpu_irq_pvnet);
        }
    }

    /* Remaining LEVEL1 bits (16-31 except 20) → IP4 */
    bool level = (imsr & 0xFFFF0000ULL & ~(1ULL << 20)) != 0;
    if (level != s->irq_level) {
        s->irq_level = level;
        if (level) {
            qemu_irq_raise(s->cpu_irq_ip4);
        } else {
            qemu_irq_lower(s->cpu_irq_ip4);
        }
    }
}

/*
 * GPIO input handler: PV devices raise/lower ISR bits through here.
 * n=0 → ISR bit 20 (pvnet), n=1 → bit 21 (glaccel), n=2 → bit 22 (pvaudio).
 */
static void ip54_heart_irq_handler(void *opaque, int n, int level)
{
    IP54HeartShimState *s = opaque;
    uint64_t bit = 1ULL << (20 + n);

    if (level) {
        s->gpio_level |= bit;
        s->isr |= bit;
    } else {
        s->gpio_level &= ~bit;
        s->isr &= ~bit;
    }
    ip54_heart_update_irq(s);
}

/*
 * Check if an offset corresponds to a HEART register.
 * HEART registers are at specific offsets within the 0x70000 range,
 * spaced 0x10000 apart.  Non-register offsets are RAM passthrough.
 */
static bool ip54_heart_is_register(hwaddr addr)
{
    hwaddr aligned = addr & ~7ULL;
    switch (aligned) {
    case HEART_OFF_MODE:
    case HEART_OFF_STATUS:
    case HEART_OFF_IMR0:
    case HEART_OFF_IMR1:
    case HEART_OFF_IMR2:
    case HEART_OFF_IMR3:
    case HEART_OFF_SET_ISR:
    case HEART_OFF_CLR_ISR:
    case HEART_OFF_ISR:
    case HEART_OFF_IMSR:
    case HEART_OFF_CAUSE:
    case HEART_OFF_COUNT:
    case HEART_OFF_COMPARE:
    case HEART_OFF_TRIGGER:
    case HEART_OFF_PRID:
    case HEART_OFF_SYNC:
        return true;
    default:
        return false;
    }
}

static uint64_t ip54_heart_read(void *opaque, hwaddr addr, unsigned size)
{
    IP54HeartShimState *s = opaque;

    /* Non-register offsets: pass through to underlying RAM */
    if (!ip54_heart_is_register(addr)) {
        uint64_t val = 0;
        memory_region_dispatch_read(s->ram, s->ram_offset + addr,
                                    &val, size_memop(size) | MO_BE,
                                    MEMTXATTRS_UNSPECIFIED);
        return val;
    }

    switch (addr & ~7ULL) {  /* Align to 8-byte boundary */
    case HEART_OFF_COUNT:
    {
        /* Read pvtimer counter (66 MHz free-running) */
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t ticks = muldiv64(now, 66000000, NANOSECONDS_PER_SECOND);
        if (s->pvtimer) {
            ticks -= s->pvtimer->base;
        }
        return ticks;
    }
    case HEART_OFF_ISR:
        return s->isr;
    case HEART_OFF_IMSR:
        return s->isr & s->imr[0];  /* Masked ISR for CPU 0 */
    case HEART_OFF_CAUSE:
        return s->isr & s->imr[0];
    case HEART_OFF_IMR0:
        return s->imr[0];
    case HEART_OFF_IMR1:
        return s->imr[1];
    case HEART_OFF_IMR2:
        return s->imr[2];
    case HEART_OFF_IMR3:
        return s->imr[3];
    case HEART_OFF_STATUS:
        return 0;  /* No errors */
    case HEART_OFF_MODE:
        return 0;
    case HEART_OFF_COMPARE:
        return s->compare;
    case HEART_OFF_PRID:
        return 0x0c000000;  /* HEART rev C */
    default:
        return 0;
    }
}

static void ip54_heart_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    IP54HeartShimState *s = opaque;

    /* Non-register offsets: pass through to underlying RAM */
    if (!ip54_heart_is_register(addr)) {
        memory_region_dispatch_write(s->ram, s->ram_offset + addr,
                                     val, size_memop(size) | MO_BE,
                                     MEMTXATTRS_UNSPECIFIED);
        return;
    }

    switch (addr & ~7ULL) {
    case HEART_OFF_CLR_ISR:
        s->isr &= ~val;
        /* Re-assert any bits whose GPIO inputs are still held high.
         * heart_intr() writes CLR_ISR BEFORE calling the device handler;
         * without this, a level-triggered device that is still asserting
         * would have its ISR bit cleared and the interrupt lost. */
        s->isr |= s->gpio_level;
        break;
    case HEART_OFF_SET_ISR:
        s->isr |= val;
        break;
    case HEART_OFF_IMR0:
        if (size >= 8) {
            s->imr[0] = val;
        } else if (size == 4) {
            /* Merge 4-byte write into correct half (big-endian) */
            if ((addr & 7) == 0) {
                s->imr[0] = (s->imr[0] & 0xFFFFFFFFULL) | (val << 32);
            } else {
                s->imr[0] = (s->imr[0] & 0xFFFFFFFF00000000ULL)
                           | (val & 0xFFFFFFFF);
            }
        }
        /*
         * IMR change may unmask a pending ISR bit — re-evaluate.
         * Only the GPIO handler path touches the CPU IRQ pin directly;
         * here we just update the IRQ state based on new masking.
         */
        ip54_heart_update_irq(s);
        break;
    case HEART_OFF_IMR1:
        s->imr[1] = val;
        break;
    case HEART_OFF_IMR2:
        s->imr[2] = val;
        break;
    case HEART_OFF_IMR3:
        s->imr[3] = val;
        break;
    case HEART_OFF_COMPARE:
        s->compare = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ip54_heart_ops = {
    .read  = ip54_heart_read,
    .write = ip54_heart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * HEART widget config space (PA 0x18000000).
 * The IRIX badaddr clearbuserrm/checkbuserrm macros access registers here.
 * Return sane defaults so badaddr doesn't falsely report bus errors.
 */
static uint64_t ip54_heart_cfg_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr) {
    case 0x04:  /* WIDGET_ID (actually HEART_WID_ID) */
        return 0x0c000000;  /* HEART rev C */
    default:
        return 0;  /* All error registers read as 0 (no errors) */
    }
}

static void ip54_heart_cfg_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    /* Absorb all writes (error clear, etc.) */
}

static const MemoryRegionOps ip54_heart_cfg_ops = {
    .read  = ip54_heart_cfg_read,
    .write = ip54_heart_cfg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

typedef struct IP54FakeMCState {
    uint32_t memcfg0;
    uint32_t memcfg1;
} IP54FakeMCState;

static uint16_t ip54_mc_bank(uint64_t phys_base, uint64_t size_bytes)
{
    uint32_t addr_field = (uint32_t)(phys_base >> 22) & 0xff;
    uint64_t mb = size_bytes / (1024ULL * 1024ULL);
    uint16_t dram_field;

    /* (bank_MB/4 - 1) × 0x0100; capped at 128 MB (0x1f00) per IP22 bank */
    if (mb <= 8)        dram_field = 0x0100;
    else if (mb <= 16)  dram_field = 0x0300;
    else if (mb <= 32)  dram_field = 0x0700;
    else if (mb <= 64)  dram_field = 0x0f00;
    else                dram_field = 0x1f00;

    return (uint16_t)(dram_field | addr_field);
}

static uint64_t ip54_mc_read(void *opaque, hwaddr addr, unsigned size)
{
    IP54FakeMCState *s = opaque;

    /* MEMCFG0: offset 0xC0 (newer MC revision) or 0xC4 (older MC revision) */
    if (addr == 0xC0 || addr == 0xC4) {
        return s->memcfg0;
    }
    /* MEMCFG1: offset 0xC8 or 0xCC */
    if (addr == 0xC8 || addr == 0xCC) {
        return s->memcfg1;
    }
    /* CPUCTRL0, SYSID, RPSS_DIVIDER, etc.: return 0 */
    return 0;
}

static void ip54_mc_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    /* Silently absorb MC register writes */
}

static const MemoryRegionOps ip54_mc_ops = {
    .read  = ip54_mc_read,
    .write = ip54_mc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sgi_ip54pv_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    DeviceState *smp_dev;
    Clock *cpuclk;
    char *filename;
    int bios_size;
    int ncpus = machine->smp.cpus;

    /* Validate RAM */
    if (machine->ram_size > IP54PV_RAM_MAX) {
        error_report("RAM size more than 2GB is not supported");
        exit(EXIT_FAILURE);
    }

    /* CPU clock: R10000 @ 200MHz (matches PROM cpufreq=200) */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 200000000);

    /* Create CPUs */
    MIPSCPU **cpus = g_new0(MIPSCPU *, ncpus);
    for (int i = 0; i < ncpus; i++) {
        cpus[i] = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
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

    /* RAM at physical 0x08000000 (SGI convention — IRIX kernels link at kseg0 0x88xxxxxx) */
    memory_region_add_subregion(system_memory, IP54PV_RAM_BASE, machine->ram);

    /*
     * Low memory alias at physical 0x00000000.
     *
     * On real SGI hardware, PA 0x00000000 and PA 0x08000000 address the same
     * physical RAM.  The MIPS R4000/R5000 hardwires exception vectors at
     * PA 0x00000000 (VA 0x80000000 kseg0), so the kernel installs handlers
     * there.  The general exception dispatcher then jumps to kernel functions
     * via kseg0 addresses (VA 0x80xxxxxx = PA 0x00xxxxxx), which must
     * resolve to the same memory as the kernel text at PA 0x08xxxxxx.
     *
     * The kernel text starts at PA 0x08002000 (VA 0x88002000); exception
     * vectors occupy PA 0x08000000-0x08000400 (below kernel text), so the
     * alias does not cause corruption.  PROM .data/.bss at PA 0x03F00000
     * aliases to PA 0x0BF00000, safely above the kernel image.
     */
    /*
     * Low alias size: min(ram_size, 128MB).  Capped at 128MB because
     * beyond that the alias (at PA 0x00000000) would overlap with the
     * main RAM region (at PA 0x08000000).
     */
    uint64_t lowmem_size = MIN(machine->ram_size, 128 * MiB);
    MemoryRegion *lowmem = g_new(MemoryRegion, 1);
    memory_region_init_alias(lowmem, NULL, "sgi.ram.lo",
                             machine->ram, 0, lowmem_size);
    memory_region_add_subregion(system_memory, IP54PV_RAM_ALIAS, lowmem);

    /* sgi-pvtimer at 0x14000000 (66MHz usecdelay counter) */
    SGIPVTimerState *pvtimer_state;
    {
        DeviceState *timer_dev = qdev_new(TYPE_SGI_PVTIMER);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(timer_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(timer_dev), 0, IP54PV_TIMER_BASE);
        pvtimer_state = SGI_PVTIMER(timer_dev);
    }

    /* HEART counter compatibility shim at PA 0x0FF00000.
     * Redirects HEART_COUNT reads (offset 0x80) to the pvtimer counter.
     * This handles dynamically-installed code that reads the IP30 HEART
     * counter and cannot be patched from the PROM.
     *
     * Also provides IRQ interposition: PV devices raise/lower ISR bits
     * (20-22) which the kernel reads via h_imsr.  When ISR & IMR[0]
     * has LEVEL1 bits set (16-31), CPU IP4 is asserted.
     */
    static IP54HeartShimState heart_shim;
    qemu_irq *heart_irqs;
    {
        MemoryRegion *heart_mem = g_new(MemoryRegion, 1);
        heart_shim.pvtimer = pvtimer_state;
        heart_shim.ram = machine->ram;
        /*
         * RAM offset: HEART_BASE (0x0FF00000) relative to RAM_BASE (0x08000000).
         * Non-register accesses within the HEART range pass through to
         * the underlying RAM at this offset so the kernel can freely
         * allocate pages in the overlapping physical address range.
         */
        heart_shim.ram_offset = IP54PV_HEART_BASE - IP54PV_RAM_BASE;
        heart_shim.cpu_irq_ip4 = cpus[0]->env.irq[4];
        heart_shim.cpu_irq_pvnet = cpus[0]->env.irq[3];  /* IP3 for pvnet */
        memory_region_init_io(heart_mem, NULL, &ip54_heart_ops, &heart_shim,
                              "sgi-heart-shim", IP54PV_HEART_SIZE);
        memory_region_add_subregion_overlap(system_memory, IP54PV_HEART_BASE,
                                            heart_mem, 1);

        /* Create 3 GPIO inputs for PV device IRQs → HEART ISR bits 20-22 */
        heart_irqs = qemu_allocate_irqs(ip54_heart_irq_handler, &heart_shim, 3);
    }

    /* HEART widget config space at PA 0x18000000.
     * The IRIX badaddr macro's clearbuserrm/checkbuserrm reads HEART
     * widget error registers here. Without this region, those reads
     * hit unmapped memory and badaddr always reports failure. */
    {
        MemoryRegion *hcfg_mem = g_new(MemoryRegion, 1);
        memory_region_init_io(hcfg_mem, NULL, &ip54_heart_cfg_ops, NULL,
                              "sgi-heart-cfg", IP54PV_HEART_CFG_SIZE);
        memory_region_add_subregion_overlap(system_memory,
                                            IP54PV_HEART_CFG_BASE,
                                            hcfg_mem, 1);
    }

    /* sgi-bootdisk at 0x17000000 (virtual disk for PROM) */
    {
        DeviceState *bd_dev = qdev_new(TYPE_SGI_BOOTDISK);
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
        if (!dinfo) {
            dinfo = drive_get(IF_PFLASH, 0, 0);
        }
        if (!dinfo) {
            dinfo = drive_get(IF_SCSI, 0, 0);
        }
        if (dinfo) {
            qdev_prop_set_drive_err(bd_dev, "drive",
                                    blk_by_legacy_dinfo(dinfo), &error_fatal);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bd_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(bd_dev), 0, IP54PV_BOOTDISK_BASE);
    }

    /* sgi-pvuart at 0x1F620178 (console, polled) */
    {
        DeviceState *uart_dev = qdev_new(TYPE_SGI_PVUART);
        if (serial_hd(0)) {
            qdev_prop_set_chr(uart_dev, "chardev", serial_hd(0));
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(uart_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(uart_dev), 0, IP54PV_UART_BASE);
    }

    /* sgi-smp at PV_BASE+0x000 */
    smp_dev = qdev_new(TYPE_SGI_SMP);
    qdev_prop_set_uint32(smp_dev, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smp_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(smp_dev), 0, IP54PV_PV_SMP);
    {
        SGISMPState *smp = SGI_SMP(smp_dev);
        for (int i = 0; i < ncpus && i < SGI_SMP_MAXCPU; i++) {
            smp->cpus[i] = CPU(cpus[i]);
        }
    }

    /* sgi-pvmem at PV_BASE+0x100 */
    {
        DeviceState *pvmem_dev = qdev_new(TYPE_SGI_PVMEM);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvmem_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvmem_dev), 0, IP54PV_PV_MEM);
    }

    /* sgi-pvnet at PV_BASE+0x200, IRQ → HEART ISR bit 20 → IP4 */
    {
        DeviceState *pvnet_dev = qdev_new(TYPE_SGI_PVNET);
        qemu_configure_nic_device(pvnet_dev, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvnet_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvnet_dev), 0, IP54PV_PV_NET);
        sysbus_connect_irq(SYS_BUS_DEVICE(pvnet_dev), 0, heart_irqs[0]);
    }

    /* sgi-glaccel at PV_BASE+0x300, IRQ → HEART ISR bit 21 → IP4 */
    {
        DeviceState *glaccel_dev = qdev_new(TYPE_SGI_GLACCEL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(glaccel_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(glaccel_dev), 0, IP54PV_PV_GLACCEL);
        sysbus_connect_irq(SYS_BUS_DEVICE(glaccel_dev), 0, heart_irqs[1]);
    }

    /* sgi-pvaudio at PV_BASE+0x400, IRQ → HEART ISR bit 22 → IP4 */
    {
        DeviceState *pvaudio_dev = qdev_new(TYPE_SGI_PVAUDIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvaudio_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvaudio_dev), 0, IP54PV_PV_AUDIO);
        sysbus_connect_irq(SYS_BUS_DEVICE(pvaudio_dev), 0, heart_irqs[2]);
    }

    /* sgi-pvrex3 at 0x1F490000 (8KB REX3 register space), IRQ → HEART ISR bit 21 → IP4 */
    {
        DeviceState *pvrex3_dev = qdev_new(TYPE_SGI_PVREX3);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pvrex3_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pvrex3_dev), 0, IP54PV_PV_REX3);
        sysbus_connect_irq(SYS_BUS_DEVICE(pvrex3_dev), 0, heart_irqs[1]);
    }

    /*
     * Absorb IP32/MACE legacy addresses that the IP54 kernel (compiled from
     * IP32 sources) may still access, e.g. the MACE ISA LED control register
     * at 0x1FBD9870.  Using unimp devices silently absorbs reads/writes so
     * the kernel does not take an AdES exception for non-existent hardware.
     *
     * Covered ranges (physical):
     *   0x1F000000–0x1F480000  MACE chip + GIO bus area (before PV bank)
     *   0x1F490000–0x1F620000  gap between PV bank and UART
     *   0x1F630000–0x1FC00000  MACE ISA bus area (includes LED at 0x1FBD9870)
     */
    {
        DeviceState *d;

        d = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
        qdev_prop_set_string(d, "name", "sgi-mace-base");
        qdev_prop_set_uint64(d, "size", 0x480000ULL);  /* 4.5 MB */
        sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, 0x1F000000ULL);

        /* Gap after pvrex3 (0x1F492000) to UART area */
        d = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
        qdev_prop_set_string(d, "name", "sgi-mace-gap1");
        qdev_prop_set_uint64(d, "size", 0x18E000ULL);  /* ~1.6 MB minus 8KB for pvrex3 */
        sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, 0x1F492000ULL);

        d = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
        qdev_prop_set_string(d, "name", "sgi-mace-isa");
        qdev_prop_set_uint64(d, "size", 0x3D0000ULL);  /* ~3.8 MB */
        sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, 0x1F830000ULL);
    }

    /*
     * IOC2-style 8042 PS/2 keyboard/mouse controller at 0x1FBD9840,
     * overlapping the sgi-mace-isa unimp region (priority 1 wins).
     * The stock IRIX pckm driver probes these fixed addresses
     * (data 0x1FBD9843, status/cmd 0x1FBD9847).  IRQ output is left
     * unconnected: the kernel drains the controller from the pvuart
     * callout poll, so no interrupt wiring is needed.
     */
    {
        DeviceState *kbd_dev = qdev_new(TYPE_SGI_IOC2_KBD);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(kbd_dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(kbd_dev), 0,
                                0x1FBD9840ULL, 1);
    }

    /*
     * Fake IP22 MC at 0x1FA00000, overlapping the sgi-mace-isa unimp region.
     * Priority 1 beats the unimp device's default priority 0 so that MEMCFG
     * reads return the correct bank configuration rather than 0.
     *
     * IP22 SEG0 base is 0x08000000 — identical to IP54PV_RAM_BASE.
     * Up to two 128 MB banks fit in MEMCFG0 (banks 0 and 1).
     */
    {
        IP54FakeMCState *mc_state = g_new0(IP54FakeMCState, 1);
        MemoryRegion *mc_mem = g_new(MemoryRegion, 1);
        const uint64_t bank_max  = 128ULL * 1024 * 1024;
        const uint64_t seg0_base = 0x08000000ULL;
        uint64_t ram = machine->ram_size;

        if (ram <= bank_max) {
            /* Single bank covers full RAM */
            mc_state->memcfg0 = (uint32_t)ip54_mc_bank(seg0_base, ram) << 16;
        } else {
            /* Two banks: bank0 = 128 MB, bank1 = remainder (≤ 128 MB) */
            uint64_t b1_size = MIN(ram - bank_max, bank_max);
            mc_state->memcfg0 =
                ((uint32_t)ip54_mc_bank(seg0_base,           bank_max) << 16) |
                 (uint32_t)ip54_mc_bank(seg0_base + bank_max, b1_size);
        }
        mc_state->memcfg1 = 0;

        memory_region_init_io(mc_mem, NULL, &ip54_mc_ops, mc_state,
                              "sgi-ip54-fake-mc", 0x1000);
        memory_region_add_subregion_overlap(system_memory, 0x1FA00000ULL,
                                            mc_mem, 1);
    }

    /* Diagnostic timer — logs CP0 state every second */
    {
        IP54DiagTimerState *diag = g_new0(IP54DiagTimerState, 1);
        diag->env = &cpus[0]->env;
        diag->tick = 0;
        diag->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ip54_diag_timer_cb, diag);
        timer_mod(diag->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND);
    }

    /* Paravirtual 100Hz clock — delivers clock() ticks via cause_ip5_count + SW2 */
    {
        PVClockState *pvc = g_new0(PVClockState, 1);
        pvc->env = &cpus[0]->env;
        pvc->raise_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pvclock_raise_cb, pvc);
        pvc->lower_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pvclock_lower_cb, pvc);
        /* First tick fires 100ms after boot to give kernel time to initialize */
        timer_mod(pvc->raise_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000ULL);
    }

    /* PROM ROM at 0x1FC00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi-ip54pv.prom", IP54PV_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, IP54PV_PROM_BASE, prom);

    /* Load PROM image */
    if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip54prom.bin");
        if (!filename) {
            warn_report("No firmware specified; use -bios ip54prom.bin");
            filename = NULL;
        }
    }

    if (filename) {
        bios_size = load_image_targphys(filename, IP54PV_PROM_BASE,
                                        IP54PV_PROM_SIZE, NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load PROM image");
            exit(EXIT_FAILURE);
        }
    }
}

static void sgi_ip54pv_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI IP54 Paravirtual Workstation";
    mc->init = sgi_ip54pv_init;
    mc->block_default_type = IF_MTD;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "sgi.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
    mc->default_cpus = 1;
    mc->max_cpus = 128;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static const TypeInfo sgi_ip54pv_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip54"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip54pv_class_init,
};

static void sgi_ip54pv_machine_init(void)
{
    type_register_static(&sgi_ip54pv_type);
}

type_init(sgi_ip54pv_machine_init)
