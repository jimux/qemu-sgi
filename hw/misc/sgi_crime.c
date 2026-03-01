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
#include "hw/misc/sgi_crime.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

/* Verbose debug logging - set to 1 to enable */
#define DEBUG_SGI_CRIME 1

#if DEBUG_SGI_CRIME
#define CRIME_DPRINTF(fmt, ...) \
    fprintf(stderr, "CRIME: " fmt, ## __VA_ARGS__)
#else
#define CRIME_DPRINTF(fmt, ...) do {} while (0)
#endif

static void sgi_crime_update_irq(SGICRIMEState *s)
{
    /*
     * Pending interrupts = (hardware intstat | software softint) & mask.
     * SOFTINT bits are kept separate from INTSTAT (not OR'd in).
     * The kernel clears soft interrupts by reading CRM_SOFTINT,
     * clearing the desired bit, and writing it back.
     */
    uint64_t pending = (s->intstat | s->softint) & s->intmask;
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

static uint64_t sgi_crime_read(void *opaque, hwaddr offset, unsigned size)
{
    SGICRIMEState *s = SGI_CRIME(opaque);

    /*
     * SGI uses 64-bit bus with 32-bit registers. The PROM accesses
     * registers at both the LE offset (base) and BE offset (base+4)
     * within each 64-bit doubleword. Normalize to 64-bit aligned offset.
     */
    offset &= ~7ULL;

    uint64_t val;

    switch (offset) {
    case CRM_ID:
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
        val = s->hardint;
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
        qemu_log_mask(LOG_UNIMP,
                      "sgi_crime: unimplemented read at offset 0x%03"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void sgi_crime_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    SGICRIMEState *s = SGI_CRIME(opaque);

    /* Normalize BE/LE offset to 64-bit aligned */
    offset &= ~7ULL;

    switch (offset) {
    case CRM_CONTROL:
        CRIME_DPRINTF("write CRM_CONTROL = 0x%" PRIx64 "\n", value);
        s->control = value & 0x3fffULL;
        break;

    case CRM_INTSTAT:
        CRIME_DPRINTF("write CRM_INTSTAT (clear) = 0x%" PRIx64 "\n", value);
        s->intstat &= ~value;
        sgi_crime_update_irq(s);
        break;

    case CRM_INTMASK:
        CRIME_DPRINTF("write CRM_INTMASK = 0x%" PRIx64 "\n", value);
        s->intmask = value & 0xffffffffULL;
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
        s->hardint &= ~value;
        break;

    case CRM_DOG:
        CRIME_DPRINTF("write CRM_DOG = 0x%" PRIx64 "\n", value);
        s->watchdog = value & 0x1fffffULL;
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
                      HWADDR_PRIx " value 0x%016" PRIx64 "\n",
                      offset, value);
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
    s->watchdog = 0;
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
 * Set a MACE interrupt bit in CRIME's INTSTAT.
 * Called by the MACE device when a peripheral interrupt fires.
 */
static void sgi_crime_set_mace_irq(void *opaque, int irq, int level)
{
    SGICRIMEState *s = SGI_CRIME(opaque);

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

    /* Input IRQs from MACE (16 lines) */
    qdev_init_gpio_in(dev, sgi_crime_set_mace_irq, 16);
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
