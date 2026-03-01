/*
 * SGI HEART (RACER) system controller emulation
 *
 * HEART is the central controller in the SGI Octane (IP30). It handles:
 * - Memory controller (8 SDRAM banks)
 * - Interrupt routing (64 vectors across 5 priority levels)
 * - 52-bit free-running counter timer at 12.5MHz
 * - CPU/memory error reporting
 * - Flow control for graphics
 *
 * Physical base: 0x1FF00000 (XKPHYS: 0x00000000FF000000)
 * Reference: IRIX sys/heart.h, MAME src/mame/sgi/crime.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_heart.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

#define VERBOSE (0)
#if VERBOSE
#define HEART_DPRINTF(fmt, ...) \
    fprintf(stderr, "HEART: " fmt, ## __VA_ARGS__)
#else
#define HEART_DPRINTF(fmt, ...) do {} while (0)
#endif

/* HEART timer runs at 12.5MHz (80ns period) */
#define HEART_COUNT_FREQ 12500000
#define HEART_COUNT_PERIOD_NS (1000000000ULL / HEART_COUNT_FREQ)

/* HEART register region size */
#define HEART_REG_SIZE 0x70000

/*
 * Update HEART interrupt line based on current state.
 *
 * HEART has 64 interrupt vectors across 5 priority levels:
 *   Level 4 (IP7): vectors 63-51 (errors, widget errors)
 *   Level 3 (IP6): vector 50 (timer)
 *   Level 2 (IP5): vectors 49-32 (IPI, debug, power, local)
 *   Level 1 (IP4): vectors 31-16 (local)
 *   Level 0 (IP3): vectors 15-0 (local)
 *
 * We map these to QEMU's 5 IRQ lines (IP7-IP3).
 */
static void sgi_heart_update_irq(SGIHEARTState *s)
{
    uint64_t isr = s->isr;
    uint64_t imr = s->imr[0];  /* CPU 0 mask for now */
    uint64_t masked = isr & imr;

    /* Level 4 (IP7) - vectors 63-51 */
    if (masked & HEART_INT_LEVEL4) {
        qemu_irq_raise(s->cpu_irq[0]);  /* IP7 */
    } else {
        qemu_irq_lower(s->cpu_irq[0]);
    }

    /* Level 3 (IP6) - vector 50 */
    if (masked & HEART_INT_LEVEL3) {
        qemu_irq_raise(s->cpu_irq[1]);  /* IP6 */
    } else {
        qemu_irq_lower(s->cpu_irq[1]);
    }

    /* Level 2 (IP5) - vectors 49-32 */
    if (masked & HEART_INT_LEVEL2) {
        qemu_irq_raise(s->cpu_irq[2]);  /* IP5 */
    } else {
        qemu_irq_lower(s->cpu_irq[2]);
    }

    /* Level 1 (IP4) - vectors 31-16 */
    if (masked & HEART_INT_LEVEL1) {
        qemu_irq_raise(s->cpu_irq[3]);  /* IP4 */
    } else {
        qemu_irq_lower(s->cpu_irq[3]);
    }

    /* Level 0 (IP3) - vectors 15-0 */
    if (masked & HEART_INT_LEVEL0) {
        qemu_irq_raise(s->cpu_irq[4]);  /* IP3 */
    } else {
        qemu_irq_lower(s->cpu_irq[4]);
    }
}

static uint64_t sgi_heart_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIHEARTState *s = opaque;
    uint64_t val = 0;

    switch (offset) {
    case HEART_MODE:
        val = s->mode;
        break;
    case HEART_SDRAM_MODE:
        val = s->sdram_mode;
        break;
    case HEART_MEM_REF:
        val = s->mem_ref;
        break;
    case HEART_MEM_REQ_ARB:
        val = s->mem_req_arb;
        break;
    case HEART_MEMCFG0 ... HEART_MEMCFG(3):
        {
            int idx = (offset - HEART_MEMCFG0) / 8;
            if (idx < HEART_NUM_BANKS) {
                val = s->memcfg[idx];
            }
        }
        break;
    case HEART_FC_MODE:
        val = s->fc_mode;
        break;
    case HEART_FC_TIMER_LIMIT:
        val = s->fc_timer_limit;
        break;
    case HEART_FC0_ADDR ... HEART_FC1_ADDR:
        {
            int idx = (offset - HEART_FC0_ADDR) / 8;
            if (idx < 2) {
                val = s->fc_addr[idx];
            }
        }
        break;
    case HEART_FC0_CR_CNT ... HEART_FC1_CR_CNT:
        {
            int idx = (offset - HEART_FC0_CR_CNT) / 8;
            if (idx < 2) {
                val = s->fc_cr_cnt[idx];
            }
        }
        break;
    case HEART_FC0_TIMER ... HEART_FC1_TIMER:
        {
            int idx = (offset - HEART_FC0_TIMER) / 8;
            if (idx < 2) {
                val = s->fc_timer[idx];
            }
        }
        break;
    case HEART_STATUS:
        val = s->status;
        break;
    case HEART_BERR_ADDR:
        val = s->berr_addr;
        break;
    case HEART_BERR_MISC:
        val = s->berr_misc;
        break;
    case HEART_MEMERR_ADDR:
        val = s->memerr_addr;
        break;
    case HEART_MEMERR_DATA:
        val = s->memerr_data;
        break;
    case HEART_PIUR_ACC_ERR:
        val = s->piur_acc_err;
        break;
    case HEART_MLAN_CLK_DIV:
        val = s->mlan_clk_div;
        break;
    case HEART_MLAN_CTL:
        val = s->mlan_ctl;
        break;

    case HEART_REALTIME_CTR:
    case HEART_REALTIME_CTR + 4:
        /*
         * QEMU extension: host wall-clock microseconds (QEMU_CLOCK_REALTIME).
         * Advances at true real-time speed regardless of -icount sleep=off.
         * Used by patched IRIX kernel for networking/animation timing.
         */
        val = (uint32_t)(qemu_clock_get_us(QEMU_CLOCK_REALTIME) & 0xFFFFFFFF);
        break;

    /* Interrupt registers */
    case HEART_IMR0 ... HEART_IMR(3):
        {
            int idx = (offset - HEART_IMR0) / 8;
            if (idx < 4) {
                val = s->imr[idx];
            }
        }
        break;
    case HEART_SET_ISR:
    case HEART_CLR_ISR:
        /* Read returns current ISR */
        val = s->isr;
        break;
    case HEART_ISR:
        val = s->isr;
        break;
    case HEART_IMSR:
        val = s->imsr;
        break;
    case HEART_CAUSE:
        val = s->cause;
        break;

    /* Timer registers */
    case HEART_COUNT:
        {
            /* Get current count value */
            int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            val = (ns + s->time_offset) / HEART_COUNT_PERIOD_NS;
            val &= 0xffffffffffffULL;  /* 52-bit mask */
        }
        break;
    case HEART_COUNT + 4:
        /*
         * Low 32 bits of HEART_COUNT when accessed as a 4-byte read.
         * Normally the PROM uses LD (8-byte) and the .impl.max_access_size=8
         * ensures QEMU passes it as a single 8-byte access.  This case is a
         * belt-and-suspenders fallback for any 32-bit kernel accesses.
         */
        {
            int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint64_t count = (ns + s->time_offset) / HEART_COUNT_PERIOD_NS;
            val = count & 0xFFFFFFFFULL;  /* low 32 bits */
        }
        break;
    case HEART_COMPARE:
        val = s->compare;
        break;
    case HEART_TRIGGER:
        val = s->trigger;
        break;
    case HEART_PRID:
        val = s->prid;
        break;
    case HEART_SYNC:
        val = s->sync;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "HEART: unimplemented read at offset 0x%05" HWADDR_PRIx "\n",
                      offset);
        break;
    }

    HEART_DPRINTF("read offset 0x%05" HWADDR_PRIx " -> 0x%016" PRIx64 "\n",
                  offset, val);
    return val;
}

static void sgi_heart_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    SGIHEARTState *s = opaque;

    HEART_DPRINTF("write offset 0x%05" HWADDR_PRIx " <- 0x%016" PRIx64 "\n",
                  offset, val);

    switch (offset) {
    case HEART_MODE:
        s->mode = val;
        break;
    case HEART_SDRAM_MODE:
        s->sdram_mode = val;
        break;
    case HEART_MEM_REF:
        s->mem_ref = val;
        break;
    case HEART_MEM_REQ_ARB:
        s->mem_req_arb = val;
        break;
    case HEART_MEMCFG0 ... HEART_MEMCFG(3):
        {
            int idx = (offset - HEART_MEMCFG0) / 8;
            if (idx < HEART_NUM_BANKS) {
                s->memcfg[idx] = val;
            }
        }
        break;
    case HEART_FC_MODE:
        s->fc_mode = val;
        break;
    case HEART_FC_TIMER_LIMIT:
        s->fc_timer_limit = val;
        break;
    case HEART_FC0_ADDR ... HEART_FC1_ADDR:
        {
            int idx = (offset - HEART_FC0_ADDR) / 8;
            if (idx < 2) {
                s->fc_addr[idx] = val;
            }
        }
        break;
    case HEART_FC0_CR_CNT ... HEART_FC1_CR_CNT:
        {
            int idx = (offset - HEART_FC0_CR_CNT) / 8;
            if (idx < 2) {
                s->fc_cr_cnt[idx] = val;
            }
        }
        break;
    case HEART_FC0_TIMER ... HEART_FC1_TIMER:
        {
            int idx = (offset - HEART_FC0_TIMER) / 8;
            if (idx < 2) {
                s->fc_timer[idx] = val;
            }
        }
        break;
    case HEART_STATUS:
        s->status = val;
        break;
    case HEART_BERR_ADDR:
        s->berr_addr = val;
        break;
    case HEART_BERR_MISC:
        s->berr_misc = val;
        break;
    case HEART_MEMERR_ADDR:
        s->memerr_addr = val;
        break;
    case HEART_MEMERR_DATA:
        s->memerr_data = val;
        break;
    case HEART_PIUR_ACC_ERR:
        s->piur_acc_err = val;
        break;
    case HEART_MLAN_CLK_DIV:
        s->mlan_clk_div = val;
        break;
    case HEART_MLAN_CTL:
        s->mlan_ctl = val;
        break;

    case HEART_REALTIME_CTR:
    case HEART_REALTIME_CTR + 4:
        /* Read-only QEMU extension — ignore writes */
        break;

    /* Interrupt registers */
    case HEART_IMR0 ... HEART_IMR(3):
        {
            int idx = (offset - HEART_IMR0) / 8;
            if (idx < 4) {
                s->imr[idx] = val;
                sgi_heart_update_irq(s);
            }
        }
        break;
    case HEART_SET_ISR:
        s->isr |= val;
        sgi_heart_update_irq(s);
        break;
    case HEART_CLR_ISR:
        s->isr &= ~val;
        sgi_heart_update_irq(s);
        break;
    case HEART_ISR:
        /* Writing to ISR register clears those bits */
        s->isr &= ~val;
        sgi_heart_update_irq(s);
        break;
    case HEART_IMSR:
        s->imsr = val;
        break;
    case HEART_CAUSE:
        s->cause = val;
        break;

    /* Timer registers */
    case HEART_COUNT:
        /* Read-only timer, writes are ignored */
        break;
    case HEART_COMPARE:
        s->compare = val;
        /* Timer comparison is checked on each read */
        break;
    case HEART_TRIGGER:
        s->trigger = val;
        break;
    case HEART_PRID:
        s->prid = val;
        break;
    case HEART_SYNC:
        s->sync = val;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "HEART: unimplemented write at offset 0x%05" HWADDR_PRIx "\n",
                      offset);
        break;
    }
}

static const MemoryRegionOps sgi_heart_ops = {
    .read = sgi_heart_read,
    .write = sgi_heart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        /*
         * Allow 4-byte accesses from Linux/IRIX kernels (readl/writel),
         * and 8-byte accesses from PROM (LD/SD instructions).
         */
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        /*
         * Tell QEMU our handler can accept 4- and 8-byte accesses natively.
         * Without this, QEMU defaults to 4-byte impl size and splits an 8-byte
         * LD for HEART_COUNT into two 4-byte reads — the second at offset
         * HEART_COUNT+4 falls through to the default case returning 0,
         * making the PROM delay loop run forever.
         */
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void sgi_heart_reset(DeviceState *dev)
{
    SGIHEARTState *s = SGI_HEART(dev);

    /* Reset values from real hardware */
    s->mode = 0;
    s->sdram_mode = 0;
    s->mem_ref = 0;
    s->mem_req_arb = 0;
    memset(s->memcfg, 0, sizeof(s->memcfg));
    s->fc_mode = 0;
    s->fc_timer_limit = 0;
    memset(s->fc_addr, 0, sizeof(s->fc_addr));
    memset(s->fc_cr_cnt, 0, sizeof(s->fc_cr_cnt));
    memset(s->fc_timer, 0, sizeof(s->fc_timer));
    /* Widget ID 8 in bits [3:0] — HEART is hardwired to XIO port 8 */
    s->status = 8;
    s->berr_addr = 0;
    s->berr_misc = 0;
    s->memerr_addr = 0;
    s->memerr_data = 0;
    s->piur_acc_err = 0;
    s->mlan_clk_div = 0;
    s->mlan_ctl = 0;

    memset(s->imr, 0, sizeof(s->imr));
    s->set_isr = 0;
    s->clr_isr = 0;
    s->isr = 0;
    s->imsr = 0;
    s->cause = 0;

    s->count = 0;
    s->compare = 0;
    s->trigger = 0;
    s->prid = 0;
    s->sync = 0;

    s->time_offset = 0;
}

static void sgi_heart_realize(DeviceState *dev, Error **errp)
{
    SGIHEARTState *s = SGI_HEART(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_heart_ops, s,
                          "sgi-heart", HEART_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Output IRQs to CPU (IP7, IP6, IP5, IP4, IP3) */
    for (int i = 0; i < 5; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->cpu_irq[i]);
    }
}

static const Property sgi_heart_properties[] = {
    DEFINE_PROP_UINT32("ram-size", SGIHEARTState, ram_size, 64 * 1024 * 1024),
    DEFINE_PROP_UINT32("num-cpus", SGIHEARTState, num_cpus, 1),
};

static const VMStateDescription vmstate_sgi_heart = {
    .name = "sgi-heart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(mode, SGIHEARTState),
        VMSTATE_UINT64(sdram_mode, SGIHEARTState),
        VMSTATE_UINT64(mem_ref, SGIHEARTState),
        VMSTATE_UINT64(mem_req_arb, SGIHEARTState),
        VMSTATE_UINT64_ARRAY(memcfg, SGIHEARTState, HEART_NUM_BANKS),
        VMSTATE_UINT64(fc_mode, SGIHEARTState),
        VMSTATE_UINT64(fc_timer_limit, SGIHEARTState),
        VMSTATE_UINT64_ARRAY(fc_addr, SGIHEARTState, 2),
        VMSTATE_UINT64_ARRAY(fc_cr_cnt, SGIHEARTState, 2),
        VMSTATE_UINT64_ARRAY(fc_timer, SGIHEARTState, 2),
        VMSTATE_UINT64(status, SGIHEARTState),
        VMSTATE_UINT64(berr_addr, SGIHEARTState),
        VMSTATE_UINT64(berr_misc, SGIHEARTState),
        VMSTATE_UINT64(memerr_addr, SGIHEARTState),
        VMSTATE_UINT64(memerr_data, SGIHEARTState),
        VMSTATE_UINT64(piur_acc_err, SGIHEARTState),
        VMSTATE_UINT64(mlan_clk_div, SGIHEARTState),
        VMSTATE_UINT64(mlan_ctl, SGIHEARTState),
        VMSTATE_UINT64_ARRAY(imr, SGIHEARTState, 4),
        VMSTATE_UINT64(set_isr, SGIHEARTState),
        VMSTATE_UINT64(clr_isr, SGIHEARTState),
        VMSTATE_UINT64(isr, SGIHEARTState),
        VMSTATE_UINT64(imsr, SGIHEARTState),
        VMSTATE_UINT64(cause, SGIHEARTState),
        VMSTATE_UINT64(count, SGIHEARTState),
        VMSTATE_UINT64(compare, SGIHEARTState),
        VMSTATE_UINT64(trigger, SGIHEARTState),
        VMSTATE_UINT64(prid, SGIHEARTState),
        VMSTATE_UINT64(sync, SGIHEARTState),
        VMSTATE_INT64(time_offset, SGIHEARTState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_heart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_heart_realize;
    device_class_set_legacy_reset(dc, sgi_heart_reset);
    dc->vmsd = &vmstate_sgi_heart;
    device_class_set_props(dc, sgi_heart_properties);
}

static const TypeInfo sgi_heart_info = {
    .name = TYPE_SGI_HEART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHEARTState),
    .class_init = sgi_heart_class_init,
};

static void sgi_heart_register_types(void)
{
    type_register_static(&sgi_heart_info);
}

type_init(sgi_heart_register_types)