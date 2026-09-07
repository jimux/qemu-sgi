/*
 * SGI CRIME (CPU, Rendering, I/O, and Memory Engine) emulation
 *
 * CRIME is the central controller in the SGI O2 (IP32). This implementation
 * provides:
 * - Free-running 66.67MHz timer (CRM_TIME) - critical for PROM boot
 * - Memory bank configuration registers
 * - Interrupt status/mask (stub)
 * - CPU/memory error reporting (stub)
 *
 * Physical base: 0x14000000 (kseg1: 0xB4000000)
 *
 * Reference:
 *   - IRIX sys/crime.h
 *   - MAME src/mame/sgi/crime.cpp
 *   - IRIX IP32 PROM source (IP32k.c, crime.h)
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/runstate.h"
#include "hw/misc/sgi_crime.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

/* Verbose debug logging - set to 1 to enable */
#define DEBUG_SGI_CRIME 0

#if DEBUG_SGI_CRIME
#define CRIME_DPRINTF(fmt, ...) \
    fprintf(stderr, "CRIME: " fmt, ## __VA_ARGS__)
#else
#define CRIME_DPRINTF(fmt, ...) do {} while ( 0)
#endif

/*
 * CRM_HARDINT readable view mask [sys/crime.h CRM_HARDINT_MSK 0xf0ffffff]:
 * the 16 MACE sources (bits 15:0) are ganged — the kernel never reads them
 * from HARDINT (it reads MACE's own per-source status instead) — and bits
 * 23:16 of the mask disambiguate the ganged view. Hardware sources visible
 * here are GBE0-3 (16-19), CRMERR (20), MEMERR (21), RE0-5 (22-27).
 * SOFT (28-30) and VICE (31) are outside the readable mask.
 */
#define CRM_HARDINT_VIEW_MASK 0xf0ffffffULL

static void sgi_crime_update_irq(SGICRIMEState *s)
{
    /*
     * Pending interrupts = (hardware intstat | software softint) & mask.
     * SOFTINT bits are kept separate from INTSTAT (not OR'd in).
     * The kernel clears soft interrupts by reading CRM_SOFTINT,
     * clearing the desired bit, and writing it back.
     *
     * The mask is 64-bit as written by the kernel (high 32 bits carry the
     * interrupt delivery level in ef_crmmsk); only the low 32 bits gate
     * delivery (INTSTAT/INTMASK are 32-bit registers [sys/crime.h]).
     *
     * HARDINT mirrors the pending hardware sources (see the HARDINT read
     * below); keep it in sync whenever intstat changes.
     */
    uint64_t pending = (s->intstat | s->softint) & s->intmask & 0xffffffffULL;
    s->hardint = s->intstat & CRM_HARDINT_VIEW_MASK;
    qemu_set_irq(s->cpu_irq, pending ? 1 : 0);
}

/*
 * CRM_TIME: 48-bit free-running counter at 66.67 MHz.
 * The PROM uses this for timing calibration and delay loops.
 * Scale QEMU virtual clock nanoseconds to CRIME ticks.
 *
 * The IRIX kernel writes 0 to CRM_TIME to reset the counter before
 * calibrating CPU frequency. We support this by maintaining a
 * time_offset that's applied to the raw clock value.
 *
 * Without -icount mode, QEMU_CLOCK_VIRTUAL doesn't advance within
 * translation blocks, so consecutive CRM_TIME reads return the same
 * value. The IRIX kernel's _ticksper1024inst() reads CRM_TIME twice
 * with 1024 instructions between reads; if the delta is too small,
 * cache_preempt_limit() divides by a zero result and triggers a break
 * exception. We ensure each read advances by at least MIN_TIME_ADVANCE
 * ticks (~300 ticks = 4.5µs), which is realistic for MMIO access
 * frequency with intervening computation.
 */
static uint64_t sgi_crime_raw_time(void)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return (uint64_t)(ns / CRIME_NS_PER_TICK);
}

/*
 * Minimum per-read timer advancement in ticks.
 * At 66.67 MHz, 300 ticks ≈ 4.5 µs — a reasonable minimum for the
 * wall-clock time consumed by 1024 instructions on a ~180 MHz R5000.
 * This ensures _ticksper1024inst() returns a delta large enough for
 * cache_preempt_limit() to compute a non-zero result:
 *   (DNS_PER_TICK * IPL * 300) / 1024 = (15 * 2 * 300) / 1024 = 8
 */
#define MIN_TIME_ADVANCE 300

static uint64_t sgi_crime_get_time(SGICRIMEState *s)
{
    uint64_t val = (sgi_crime_raw_time() + s->time_offset) & 0xffffffffffffULL;

    /* Ensure monotonic advancement by at least MIN_TIME_ADVANCE per read */
    if (val < s->last_time_read + MIN_TIME_ADVANCE) {
        val = s->last_time_read + MIN_TIME_ADVANCE;
    }
    s->last_time_read = val;

    return val;
}

/*
 * McGriff watchdog (CRM_DOG) [sys/crime.h].
 *
 * The register is a 21-bit down-counter at CRM_MASTER_FREQ (66.67 MHz,
 * 15 ns/tick) with two sticky status bits (POWER_ON_RESET 0x100000 /
 * WARM_RESET 0x080000) that record the cause of the last reset. The
 * kernel's non-USE_McGriff build clears DOG_ENA in CRM_CONTROL and
 * writes 0 to CRM_DOG at init (IP32init.c "clear CRIME watchdog
 * timer"), so a faithful inert-by-default watchdog cannot perturb the
 * stock boot path; it only bites when software explicitly arms it.
 *
 * On expiry with DOG_ENA set, real hardware resets the machine. We
 * record WARM_RESET in the sticky bits and request a system reset.
 */
static void sgi_crime_dog_expired(void *opaque)
{
    SGICRIMEState *s = SGI_CRIME(opaque);

    if (!s->dog_enabled) {
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "sgi_crime: McGriff watchdog expired "
                  "- hardware reset\n");
    /* Sticky status bits survive the reset to report the cause. */
    s->watchdog = (s->watchdog & ~(CRM_DOG_POWER_ON_RESET | CRM_DOG_WARM_RESET |
                                   CRM_DOG_VALUE))
                  | CRM_DOG_WARM_RESET;
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void sgi_crime_dog_rearm(SGICRIMEState *s)
{
    uint64_t count;

    if (s->dog_timer) {
        timer_del(s->dog_timer);
    }
    if (!s->dog_enabled) {
        return;
    }

    count = s->watchdog & CRM_DOG_VALUE;
    if (count == 0) {
        /* Count 0 with the dog enabled: expires immediately. */
        timer_mod_ns(s->dog_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return;
    }
    /* count ticks at 15 ns each; +1 so a count of N runs N ticks. */
    timer_mod_ns(s->dog_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 (int64_t)(count + 1) * CRIME_NS_PER_TICK);
}

static uint64_t sgi_crime_read(void *opaque, hwaddr offset, unsigned size)
{
    SGICRIMEState *s = SGI_CRIME(opaque);
    hwaddr raw_offset = offset;

    /*
     * SGI uses 64-bit bus with 32-bit registers. The PROM accesses
     * registers at both the LE offset (base) and BE offset (base+4)
     * within each 64-bit doubleword. Normalize to 64-bit aligned offset.
     */
    offset &= ~7ULL;

    uint64_t val;

    switch (offset) {
    case CRM_ID:
        /*
         * CRM_ID is a 64-bit big-endian register whose meaningful byte
         * sits in the LOW word (offset +4). The PROM's crmGetRev/
         * crmGetType read the 32-bit word at 0x14000004 and compare
         * against 0x000000a1 (CRIME rev 1 with graphics, per gxemul's
         * dev_sgi_ip32 "0xa1 for machines with graphics"); a 4-byte
         * read must therefore return the low 32 bits, not the full
         * 64-bit register. 8-byte reads (kernel READ_REG64) get the
         * whole value. s->id is 0xa1 so both views work: the kernel's
         * get_crimerev() sees rev 1 (CRM_REV_11) either way.
         */
        if (size == 4) {
            /* +0 selects the BE high word, +4 the BE low word */
            val = (raw_offset & 4) ? (s->id & 0xffffffff)
                                   : (s->id >> 32);
            CRIME_DPRINTF("read  CRM_ID[32] @+%"PRIx64" = 0x%" PRIx64 "\n",
                          raw_offset & 7, val);
            return val;
        }
        val = s->id;
        CRIME_DPRINTF("read  CRM_ID = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_CONTROL:
        val = s->control;
        CRIME_DPRINTF("read  CRM_CONTROL = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_INTSTAT:
        /*
         * INTSTAT returns hardware interrupts | software interrupts.
         * The IRIX kernel reads this to determine which interrupts are pending.
         */
        val = s->intstat | s->softint;
        CRIME_DPRINTF("read  CRM_INTSTAT = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_INTMASK:
        val = s->intmask;
        CRIME_DPRINTF("read  CRM_INTMASK = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_SOFTINT:
        val = s->softint;
        CRIME_DPRINTF("read  CRM_SOFTINT = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_HARDINT:
        /*
         * CRM_HARDINT is a live (not latched) view of the currently-asserted
         * hardware sources, masked to 0xf0ffffff [sys/crime.h]. Refresh
         * from intstat so it never reports stale state. The kernel's
         * addrprobe.s reads it to sample CRMERR (bit 20, CAUSE_BERRINTR)
         * during bus-error probing.
         */
        val = s->intstat & CRM_HARDINT_VIEW_MASK;
        s->hardint = val;
        CRIME_DPRINTF("read  CRM_HARDINT = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_DOG:
        val = s->watchdog;
        CRIME_DPRINTF("read  CRM_DOG = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_TIME:
        /* Don't log every CRM_TIME read - too frequent */
        return sgi_crime_get_time(s);

    case CRM_CPU_ERROR_ADDR:
        val = s->cpu_error_addr;
        CRIME_DPRINTF("read  CRM_CPU_ERROR_ADDR = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_CPU_ERROR_STAT:
        val = s->cpu_error_stat;
        CRIME_DPRINTF("read  CRM_CPU_ERROR_STAT = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_CPU_ERROR_ENA:
        val = s->cpu_error_ena;
        CRIME_DPRINTF("read  CRM_CPU_ERROR_ENA = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_VICE_ERROR_ADDR:
        val = s->vice_error_addr;
        CRIME_DPRINTF("read  CRM_VICE_ERROR_ADDR = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_REALTIME_CTR:
        /*
         * QEMU extension: host wall-clock microseconds (QEMU_CLOCK_REALTIME).
         * Advances at true real-time speed regardless of -icount sleep=off.
         * Used by patched IRIX kernel for networking/animation timing.
         */
        val = (uint32_t)(qemu_clock_get_us(QEMU_CLOCK_REALTIME) & 0xFFFFFFFF);
        CRIME_DPRINTF("read  CRM_REALTIME_CTR = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_CONTROL:
        val = s->mem_control;
        CRIME_DPRINTF("read  CRM_MEM_CONTROL = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_BANK_CTRL(0) ... CRM_MEM_BANK_CTRL(7):
    {
        int bank = (offset - CRM_MEM_BANK_CTRL0) / 8;
        val = s->mem_bank_ctrl[bank];
        CRIME_DPRINTF("read  CRM_MEM_BANK_CTRL[%d] = 0x%" PRIx64
                      " (ram_size=%uMB)\n",
                      bank, val, s->ram_size / (1024 * 1024));
        return val;
    }

    case CRM_MEM_REFRESH_CNTR:
        val = s->mem_refresh_cntr;
        CRIME_DPRINTF("read  CRM_MEM_REFRESH_CNTR = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_ERROR_STAT:
        val = s->mem_error_stat;
        CRIME_DPRINTF("read  CRM_MEM_ERROR_STAT = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_ERROR_ADDR:
        val = s->mem_error_addr;
        CRIME_DPRINTF("read  CRM_MEM_ERROR_ADDR = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_ERROR_ECC_SYN:
        val = s->mem_error_ecc_syn;
        CRIME_DPRINTF("read  CRM_MEM_ERROR_ECC_SYN = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_ERROR_ECC_CHK:
        val = s->mem_error_ecc_chk;
        CRIME_DPRINTF("read  CRM_MEM_ERROR_ECC_CHK = 0x%" PRIx64 "\n", val);
        return val;

    case CRM_MEM_ERROR_ECC_REPL:
        val = s->mem_error_ecc_repl;
        CRIME_DPRINTF("read  CRM_MEM_ERROR_ECC_REPL = 0x%" PRIx64 "\n", val);
        return val;

    default:
        CRIME_DPRINTF("read  UNKNOWN offset 0x%03" HWADDR_PRIx "\n", offset);
        /* Log the raw offset so stray BE/LE sub-word accesses inside the
         * CRIME page are distinguishable from genuinely missing regs. */
        qemu_log_mask(LOG_UNIMP,
                      "sgi_crime: unimplemented read at offset 0x%03"
                      HWADDR_PRIx " (aligned 0x%03" HWADDR_PRIx ", size %u)\n",
                      raw_offset, offset, size);
        return 0;
    }
}

static void sgi_crime_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    SGICRIMEState *s = SGI_CRIME(opaque);
    hwaddr raw_offset = offset;

    /* Normalize BE/LE offset to 64-bit aligned */
    offset &= ~7ULL;

    switch (offset) {
    case CRM_CONTROL:
        CRIME_DPRINTF("write CRM_CONTROL = 0x%" PRIx64 "\n", value);
        s->control = value & 0x3fffULL;
        /*
         * Watchdog enable (CRM_CONTROL_DOG_ENA [sys/crime.h]). The stock
         * IRIX kernel clears this bit (IP32init.c, non-USE_McGriff build),
         * so the dog stays inert unless software explicitly arms it.
         */
        {
            bool enable = (s->control & CRM_CONTROL_DOG_ENA) != 0;
            if (enable != s->dog_enabled) {
                s->dog_enabled = enable;
                sgi_crime_dog_rearm(s);
            }
        }
        /*
         * HARD_RESET / SOFT_RESET [sys/crime.h]: reset strobes. The PROM's
         * reset path writes SOFT_RESET (IP32asm.s _coldstart / sl_csu.s)
         * or HARD_RESET ("reboot"); gxemul treats HARD_RESET as a machine
         * reboot. Self-clearing strobes: drop them from the stored value
         * after firing.
         */
        if (value & (CRM_CONTROL_HARD_RESET | CRM_CONTROL_SOFT_RESET)) {
            bool hard = (value & CRM_CONTROL_HARD_RESET) != 0;
            s->control &= ~(CRM_CONTROL_HARD_RESET | CRM_CONTROL_SOFT_RESET);
            if (hard) {
                qemu_log_mask(LOG_GUEST_ERROR, "sgi_crime: HARD_RESET\n");
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "sgi_crime: SOFT_RESET\n");
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
        }
        break;

    case CRM_INTSTAT:
        CRIME_DPRINTF("write CRM_INTSTAT (clear) = 0x%" PRIx64 "\n", value);
        s->intstat &= ~value;
        sgi_crime_update_irq(s);
        break;

    case CRM_INTMASK:
        CRIME_DPRINTF("write CRM_INTMASK = 0x%" PRIx64 "\n", value);
        /*
         * CRM_INTMASK is a 64-bit register as read/written by the kernel
         * (_crmreg_t, e.g. 0x2800000000): the low 32 bits are the per-source
         * enable mask, the high 32 bits carry the interrupt delivery level.
         * Keep the full value so the kernel's readback round-trips; update_irq
         * only tests the low mask bits against intstat anyway.
         */
        s->intmask = value;
        sgi_crime_update_irq(s);
        break;

    case CRM_SOFTINT:
        CRIME_DPRINTF("write CRM_SOFTINT = 0x%" PRIx64 "\n", value);
        /*
         * SOFTINT is a read/write register for software-generated interrupts.
         * The kernel sets bits (e.g., CRM_INT_SOFT0) to trigger soft interrupts,
         * and clears them by reading, masking off the bit, and writing back.
         * These bits are NOT merged into intstat — they are kept separate.
         */
        s->softint = value & 0x70000000ULL;
        sgi_crime_update_irq(s);
        break;

    case CRM_HARDINT:
        CRIME_DPRINTF("write CRM_HARDINT (clear) = 0x%" PRIx64 "\n", value);
        /*
         * HARDINT is a read-only view of live hardware sources; accept
         * the write (sources clear themselves by deasserting their gpio
         * line) but do not let it clear intstat behind a live source's
         * back. Only allow clearing bits that are no longer asserted.
         */
        break;

    case CRM_DOG:
        CRIME_DPRINTF("write CRM_DOG = 0x%" PRIx64 "\n", value);
        /* Counter field is writable; the reset-cause bits are sticky. */
        s->watchdog = (s->watchdog & (CRM_DOG_POWER_ON_RESET |
                                      CRM_DOG_WARM_RESET))
                      | (value & CRM_DOG_VALUE);
        sgi_crime_dog_rearm(s);
        break;

    case CRM_TIME:
        /*
         * The IRIX kernel writes 0 to CRM_TIME to reset the counter
         * before calibrating CPU frequency (_cpuclkper100ticks).
         * Compute offset so that reads return (raw_time + offset).
         * Reset last_time_read so monotonic advancement starts fresh.
         */
        s->time_offset = (int64_t)(value & 0xffffffffffffULL) -
                          (int64_t)sgi_crime_raw_time();
        s->last_time_read = 0;
        CRIME_DPRINTF("write CRM_TIME = 0x%" PRIx64
                      " (offset = %" PRId64 ")\n",
                      value, s->time_offset);
        break;

    case CRM_CPU_ERROR_ADDR:
        CRIME_DPRINTF("write CRM_CPU_ERROR_ADDR = 0x%" PRIx64 "\n", value);
        s->cpu_error_addr = value;
        break;

    case CRM_CPU_ERROR_STAT:
        CRIME_DPRINTF("write CRM_CPU_ERROR_STAT (clear) = 0x%" PRIx64 "\n",
                      value);
        s->cpu_error_stat &= ~value;
        break;

    case CRM_CPU_ERROR_ENA:
        CRIME_DPRINTF("write CRM_CPU_ERROR_ENA = 0x%" PRIx64 "\n", value);
        s->cpu_error_ena = value;
        break;

    case CRM_VICE_ERROR_ADDR:
        CRIME_DPRINTF("write CRM_VICE_ERROR_ADDR = 0x%" PRIx64 "\n", value);
        s->vice_error_addr = value;
        break;

    case CRM_REALTIME_CTR:
        /* Read-only QEMU extension — ignore writes */
        break;

    case CRM_MEM_CONTROL:
        CRIME_DPRINTF("write CRM_MEM_CONTROL = 0x%" PRIx64 "\n", value);
        s->mem_control = value & 0x3ULL;
        break;

    case CRM_MEM_BANK_CTRL(0) ... CRM_MEM_BANK_CTRL(7):
    {
        int bank = (offset - CRM_MEM_BANK_CTRL0) / 8;
        CRIME_DPRINTF("write CRM_MEM_BANK_CTRL[%d] = 0x%" PRIx64 "\n",
                      bank, value);
        s->mem_bank_ctrl[bank] = value & 0x11fULL;
        break;
    }

    case CRM_MEM_REFRESH_CNTR:
        CRIME_DPRINTF("write CRM_MEM_REFRESH_CNTR = 0x%" PRIx64 "\n", value);
        s->mem_refresh_cntr = value & 0x7ffULL;
        break;

    case CRM_MEM_ERROR_STAT:
        CRIME_DPRINTF("write CRM_MEM_ERROR_STAT (clear) = 0x%" PRIx64 "\n",
                      value);
        s->mem_error_stat &= ~value;
        break;

    case CRM_MEM_ERROR_ADDR:
        CRIME_DPRINTF("write CRM_MEM_ERROR_ADDR = 0x%" PRIx64 "\n", value);
        s->mem_error_addr = value;
        break;

    case CRM_MEM_ERROR_ECC_SYN:
        CRIME_DPRINTF("write CRM_MEM_ERROR_ECC_SYN = 0x%" PRIx64 "\n",
                      value);
        s->mem_error_ecc_syn = value;
        break;

    case CRM_MEM_ERROR_ECC_CHK:
        CRIME_DPRINTF("write CRM_MEM_ERROR_ECC_CHK = 0x%" PRIx64 "\n",
                      value);
        s->mem_error_ecc_chk = value;
        break;

    case CRM_MEM_ERROR_ECC_REPL:
        CRIME_DPRINTF("write CRM_MEM_ERROR_ECC_REPL = 0x%" PRIx64 "\n",
                      value);
        s->mem_error_ecc_repl = value;
        break;

    default:
        CRIME_DPRINTF("write UNKNOWN offset 0x%03" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, value);
        qemu_log_mask(LOG_UNIMP,
                      "sgi_crime: unimplemented write at offset 0x%03"
                      HWADDR_PRIx " (aligned 0x%03" HWADDR_PRIx
                      ", size %u) value 0x%016" PRIx64 "\n",
                      raw_offset, offset, size, value);
        break;
    }
}

static const MemoryRegionOps sgi_crime_ops = {
    .read = sgi_crime_read,
    .write = sgi_crime_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void sgi_crime_reset(DeviceState *dev)
{
    SGICRIMEState *s = SGI_CRIME(dev);

    s->id = CRIME_ID_VALUE;
    s->control = CRM_CONTROL_ENDIAN_BIG;
    s->intstat = 0;
    s->intmask = 0;
    s->softint = 0;
    s->hardint = 0;
    /*
     * Power-on: the McGriff records POWER_ON_RESET [sys/crime.h], and the
     * dog starts disabled (CRM_CONTROL has no DOG_ENA at reset). The
     * counter loads its max value (CRM_DOG_VALUE) per the kernel's
     * McGriff kick idiom (IP32intr.c writes 0x7fff under USE_McGriff).
     */
    s->watchdog = CRM_DOG_POWER_ON_RESET | CRM_DOG_VALUE;
    s->dog_enabled = false;
    if (s->dog_timer) {
        timer_del(s->dog_timer);
    }
    s->time_offset = 0;
    s->last_time_read = 0;
    s->cpu_error_addr = 0;
    s->cpu_error_stat = 0;
    s->cpu_error_ena = 0;
    s->vice_error_addr = 0;
    s->mem_control = 0;
    s->mem_refresh_cntr = 0;
    s->mem_error_stat = 0;
    s->mem_error_addr = 0;
    s->mem_error_ecc_syn = 0;
    s->mem_error_ecc_chk = 0;
    s->mem_error_ecc_repl = 0;

    /*
     * Configure memory banks based on RAM size.
     * Bank format: bits [4:0] = base address >> 25, bit [8] = SDRAM 128MB.
     *
     * IRIX szmem() reads CRM_MEM_BANK_CTRL[0..7] and skips banks
     * whose ctrl value matches bank 0.  It calls bank_size(ctrl):
     *   bit 8 set → 128 MB, bit 8 clear → 32 MB.
     *
     * So unused banks MUST have the same ctrl as bank 0 to be skipped.
     *
     * For ≤256MB, use 32MB banks (max 8 × 32MB = 256MB):
     *   64MB  = 2 × 32MB: bank0=0x000, bank1=0x001, others=0x000
     *   256MB = 8 × 32MB: bank0..7 = 0x000..0x007
     *
     * For >256MB, switch to 128MB banks (max 8 × 128MB = 1GB):
     *   512MB = 4 × 128MB: bank0=0x100, bank1=0x104, bank2=0x108, bank3=0x10C
     *   (base_unit steps by 4 since each 128MB = 4 × 32MB units)
     */
    uint32_t remaining = s->ram_size;
    int bank = 0;
    uint32_t base_unit = 0;  /* in units of 32MB */
    CRIME_DPRINTF("reset: ram_size=%u (%uMB)\n", s->ram_size,
                  s->ram_size / (1024 * 1024));
    uint32_t bank_size = (s->ram_size > (uint32_t)(8 * 32 * 1024 * 1024))
                         ? (128 * 1024 * 1024) : (32 * 1024 * 1024);
    uint64_t size_flag = (bank_size == 128 * 1024 * 1024)
                         ? CRM_BANK_CTRL_SDRAM_SIZE : 0;
    uint32_t bank_step = bank_size / (32 * 1024 * 1024);

    while (remaining >= bank_size && bank < CRIME_NUM_BANKS) {
        s->mem_bank_ctrl[bank] = (base_unit & CRM_BANK_CTRL_ADDR_MASK)
                                 | size_flag;
        remaining -= bank_size;
        base_unit += bank_step;
        bank++;
    }

    /* Unused banks get same ctrl as bank 0 so szmem() skips them */
    for (; bank < CRIME_NUM_BANKS; bank++) {
        s->mem_bank_ctrl[bank] = s->mem_bank_ctrl[0];
    }
}

/*
 * Set/clear one of the 32 CRIME interrupt sources.
 *
 * Line number == INTSTAT bit position [sys/IP32.h]: 0-15 = MACE (ganged,
 * driven by the MACE device), 16-19 = GBE0-3, 20 = CRMERR, 21 = MEMERR,
 * 22-27 = RE0-5, 31 = VICE. Level-triggered: the source holds the bit
 * asserted until it deasserts its line (matches the MACE model and the
 * kernel's RE3/RE5 "level trigger" comments in IP32intr.c is_thd()).
 *
 * Called by the MACE device for lines 0-15 and by GBE/RE devices for
 * their own lines in later milestones.
 */
static void sgi_crime_set_irq(void *opaque, int irq, int level)
{
    SGICRIMEState *s = SGI_CRIME(opaque);

    assert(irq >= 0 && irq < CRM_NUM_IRQS);

    if (level) {
        s->intstat |= (1ULL << irq);
    } else {
        s->intstat &= ~(1ULL << irq);
    }
    sgi_crime_update_irq(s);
}

static void sgi_crime_realize(DeviceState *dev, Error **errp)
{
    SGICRIMEState *s = SGI_CRIME(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_crime_ops, s,
                          "sgi-crime", CRIME_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Output IRQ to CPU (IP2) */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cpu_irq);

    /*
     * Input IRQs: all 32 CRIME interrupt sources. Lines 0-15 are the
     * ganged MACE sources (unchanged wiring from the MACE device);
     * 16-19 = GBE0-3 (retrace/preblank), 20 = CRMERR, 21 = MEMERR,
     * 22-27 = RE0-5 (FIFO watermarks), 31 = VICE. SOFT0-2 (28-30) are
     * generated by software writes to CRM_SOFTINT and have no line.
     */
    qdev_init_gpio_in(dev, sgi_crime_set_irq, CRM_NUM_IRQS);

    s->dog_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_crime_dog_expired, s);
}

static const Property sgi_crime_properties[] = {
    DEFINE_PROP_UINT32("ram-size", SGICRIMEState, ram_size, 64 * 1024 * 1024),
};

static const VMStateDescription vmstate_sgi_crime = {
    .name = "sgi-crime",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(id, SGICRIMEState),
        VMSTATE_UINT64(control, SGICRIMEState),
        VMSTATE_UINT64(intstat, SGICRIMEState),
        VMSTATE_UINT64(intmask, SGICRIMEState),
        VMSTATE_UINT64(softint, SGICRIMEState),
        VMSTATE_UINT64(hardint, SGICRIMEState),
        VMSTATE_UINT64(watchdog, SGICRIMEState),
        VMSTATE_BOOL(dog_enabled, SGICRIMEState),
        VMSTATE_INT64(time_offset, SGICRIMEState),
        VMSTATE_UINT64(last_time_read, SGICRIMEState),
        VMSTATE_UINT64(cpu_error_addr, SGICRIMEState),
        VMSTATE_UINT64(cpu_error_stat, SGICRIMEState),
        VMSTATE_UINT64(cpu_error_ena, SGICRIMEState),
        VMSTATE_UINT64(vice_error_addr, SGICRIMEState),
        VMSTATE_UINT64(mem_control, SGICRIMEState),
        VMSTATE_UINT64_ARRAY(mem_bank_ctrl, SGICRIMEState, CRIME_NUM_BANKS),
        VMSTATE_UINT64(mem_refresh_cntr, SGICRIMEState),
        VMSTATE_UINT64(mem_error_stat, SGICRIMEState),
        VMSTATE_UINT64(mem_error_addr, SGICRIMEState),
        VMSTATE_UINT64(mem_error_ecc_syn, SGICRIMEState),
        VMSTATE_UINT64(mem_error_ecc_chk, SGICRIMEState),
        VMSTATE_UINT64(mem_error_ecc_repl, SGICRIMEState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_crime_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_crime_realize;
    device_class_set_legacy_reset(dc, sgi_crime_reset);
    dc->vmsd = &vmstate_sgi_crime;
    device_class_set_props(dc, sgi_crime_properties);
}

static const TypeInfo sgi_crime_info = {
    .name = TYPE_SGI_CRIME,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGICRIMEState),
    .class_init = sgi_crime_class_init,
};

static void sgi_crime_register_types(void)
{
    type_register_static(&sgi_crime_info);
}

type_init(sgi_crime_register_types)
