/*
 * SGI VICE (Video, Imaging and Compression Engine) — tier-i skeleton
 *
 * Models the O2 (IP32) VICE host PIO window at 0x17000000 (1 MB): the chip
 * register/descriptor block, MSP/BSP IRAM, BSP table and FIFOs, MSP DRAM,
 * and the 128-entry DMA TLB. It answers the kernel's boot probe
 * (PROBE_AND_INVENTORY_VICE): a write to MSP_SW_INT latches VICE_INT bit 2
 * and, when enabled, raises the CRIME VICE interrupt line, so IRIX records
 * INV_COMPRESSION / INV_VICE.
 *
 * Deliberately NOT modelled here (tier-ii): MSP/BSP microcode interpretation,
 * DMA data movement, and host-codec offload. DMA "GO" writes are logged and
 * the descriptor/register state is kept as storage only.
 *
 * @@SEMANTICS@@ scope: this file is the separable tier-i seam. Chip/RAM/TLB
 * state is stored faithfully, identification and the interrupt handshake are
 * functional, and every deliberately-unimplemented path is tagged below so
 * the tier-ii host-offload work can attach without reworking this layer.
 *
 * Register access lanes [vice_drv.h]:
 *   32-bit register: big-endian high lane, offset + 4
 *   16-bit register: offset + 6
 *   IRAM/DRAM/table/TLB/FIFO: bare offset
 * The chip register block is therefore stored as big-endian bytes indexed by
 * the raw (lane-inclusive) offset, which makes both lanes land in the right
 * place; register identity for the few semantic registers is recovered with
 * (offset & ~7).
 *
 * References:
 *   - VICE Design Specification 099-0123-003 (April 17, 1997)
 *   - IRIX sys/vice/{vice_drv.h,vice_regs_host.h,vice_dma.h}
 *   - IRIX kern/ml/MOOSEHEAD/{IP32init.c,vice_copy.c}
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_vice.h"
#include "migration/vmstate.h"
#include "trace.h"

/*
 * VICE internal clock is the CRIME master clock (66.67 MHz, ~15 ns/tick);
 * VICE_COUNT is a free-running counter off that clock. We derive it from the
 * virtual clock rather than inventing a constant.
 */
#define VICE_NS_PER_TICK 15

static uint64_t sgi_vice_read_chip(SGIViceState *s, hwaddr off, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | s->chip_regs[off + i];
    }
    return v;
}

static void sgi_vice_write_chip(SGIViceState *s, hwaddr off, uint64_t value,
                                unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        s->chip_regs[off + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
}

static uint64_t sgi_vice_read_bytes(const uint8_t *buf, hwaddr off,
                                    unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | buf[off + i];
    }
    return v;
}

static void sgi_vice_write_bytes(uint8_t *buf, hwaddr off, uint64_t value,
                                 unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        buf[off + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
}

static void sgi_vice_update_irq(SGIViceState *s)
{
    int level = (s->int_status & s->int_enable) != 0;

    trace_sgi_vice_int(s->int_status, s->int_enable, level);
    qemu_set_irq(s->crime_irq[0], level);
}

static uint64_t sgi_vice_read(void *opaque, hwaddr off, unsigned size)
{
    SGIViceState *s = opaque;
    hwaddr reg = off & ~7ULL;
    uint64_t v;

    trace_sgi_vice_reg_read(off, size);

    if (off >= VICE_CFG && off < VICE_CFG + 0x20) {
        switch (reg) {
        case VICE_CFG:
            return s->cfg;
        case VICE_INT_EN:
            return s->int_enable;
        default:
            return 0;
        }
    }

    if (off >= VICE_DEBUG_BASE && off < VICE_DEBUG_END) {
        /* MSP/BSP debug + exception-stamp block: storage only (tier-ii). */
        return sgi_vice_read_bytes(s->debug_regs, off - VICE_DEBUG_BASE, size);
    }

    if (off < VICE_CHIP_REGS_SIZE) {
        switch (reg) {
        case VICE_ID:
            return VICE_ID_VALUE;
        case MSP_CTL_STAT:
            /*
             * @@SEMANTICS@@ tier-i: no MSP interpreter exists yet, so the
             * only truthful state is "halted" (bit0 GO/HALT reads 0).
             */
            return 0;
        case VICE_COUNT:
            v = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / VICE_NS_PER_TICK;
            return (uint32_t)v;
        case BSP_CTL_STAT:
            return s->bsp_ctl_stat;
        case VICE_INT:
            return s->int_status;
        default:
            return sgi_vice_read_chip(s, off, size);
        }
    }

    if (off >= 0x2000 && off < 0x2000 + VICE_MSP_IRAM_SIZE) {
        return sgi_vice_read_bytes(s->msp_iram, off - 0x2000, size);
    }
    if (off >= 0x4000 && off < 0x4000 + VICE_BSP_IRAM_SIZE) {
        return sgi_vice_read_bytes(s->bsp_iram, off - 0x4000, size);
    }
    if (off >= 0x5000 && off < 0x5000 + VICE_BSP_TABLE_SIZE) {
        return sgi_vice_read_bytes(s->bsp_table, off - 0x5000, size);
    }
    if (off >= 0x7000 && off < 0x7000 + VICE_BSP_FIFO_SIZE) {
        return sgi_vice_read_bytes(s->bsp_out_fifo, off - 0x7000, size);
    }
    if (off >= 0x7800 && off < 0x7800 + VICE_BSP_FIFO_SIZE) {
        return sgi_vice_read_bytes(s->bsp_in_fifo, off - 0x7800, size);
    }
    if (off >= 0x8000 && off < 0x8000 + VICE_MSP_DRAM_SIZE) {
        return sgi_vice_read_bytes(s->msp_dram, off - 0x8000, size);
    }
    if (off >= 0xf000 && off < 0xf000 + sizeof(s->tlb)) {
        return sgi_vice_read_bytes(s->tlb, off - 0xf000, size);
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_vice: unimplemented read off=0x%" HWADDR_PRIx
                  " size=%u\n", off, size);
    return 0;
}

static void sgi_vice_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    SGIViceState *s = opaque;
    hwaddr reg = off & ~7ULL;

    trace_sgi_vice_reg_write(off, value, size);

    if (off >= VICE_CFG && off < VICE_CFG + 0x20) {
        switch (reg) {
        case VICE_CFG:
            s->cfg = (uint32_t)value;
            break;
        case VICE_INT_RESET:
            /* Write-1-to-clear the corresponding VICE_INT bits. */
            s->int_status &= ~(uint32_t)value;
            sgi_vice_update_irq(s);
            break;
        case VICE_INT_EN:
            s->int_enable = (uint32_t)value & VICE_INT_ALL;
            sgi_vice_update_irq(s);
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_vice: unimplemented cfg write off=0x%"
                          HWADDR_PRIx "\n", off);
            break;
        }
        return;
    }

    if (off >= VICE_DEBUG_BASE && off < VICE_DEBUG_END) {
        sgi_vice_write_bytes(s->debug_regs, off - VICE_DEBUG_BASE, value, size);
        return;
    }

    if (off < VICE_CHIP_REGS_SIZE) {
        switch (reg) {
        case MSP_SW_INT:
            /*
             * @@SEMANTICS@@ interrupt: write-only, any write latches
             * VICE_INT bit 2 regardless of VICE_INT_EN (spec 2.9.7); the
             * enable bit only gates the CRIME output pin.
             */
            s->int_status |= VICE_INT_MSP_INTR;
            trace_sgi_vice_sw_int("MSP");
            sgi_vice_update_irq(s);
            return;
        case BSP_SW_INT:
            s->int_status |= VICE_INT_BSP_INTR;
            trace_sgi_vice_sw_int("BSP");
            sgi_vice_update_irq(s);
            return;
        case BSP_HALT_RESET:
            if (value & VICEBSPCS_RESET) {
                s->bsp_ctl_stat = VICEBSPCS_HALT | VICEBSPCS_HALT_ACK;
            }
            break;
        case VICE_INT:
            return; /* read-only */
        case MSP_CTL_STAT:
            if (value & 0x1) {
                qemu_log_mask(LOG_UNIMP, "sgi_vice: MSP GO ignored "
                              "(no interpreter)\n");
            }
            return;
        case VICEDMA_CTL_CH1:
        case VICEDMA_CTL_CH2:
            if (value & VICEDMA_CTL_GO) {
                /*
                 * @@SEMANTICS@@ tier-ii seam: DMA descriptors are stored,
                 * but no transfer is performed and no DMA_DONE interrupt is
                 * raised. Code that actually waits on DMA completion lives
                 * in the next milestone.
                 */
                trace_sgi_vice_dma_go(reg == VICEDMA_CTL_CH1 ? 1 : 2,
                                      (uint32_t)value);
                qemu_log_mask(LOG_UNIMP,
                              "sgi_vice: DMA ch%d transfer not implemented\n",
                              reg == VICEDMA_CTL_CH1 ? 1 : 2);
            }
            break;
        default:
            break;
        }
        sgi_vice_write_chip(s, off, value, size);
        return;
    }

    if (off >= 0x2000 && off < 0x2000 + VICE_MSP_IRAM_SIZE) {
        sgi_vice_write_bytes(s->msp_iram, off - 0x2000, value, size);
        return;
    }
    if (off >= 0x4000 && off < 0x4000 + VICE_BSP_IRAM_SIZE) {
        sgi_vice_write_bytes(s->bsp_iram, off - 0x4000, value, size);
        return;
    }
    if (off >= 0x5000 && off < 0x5000 + VICE_BSP_TABLE_SIZE) {
        sgi_vice_write_bytes(s->bsp_table, off - 0x5000, value, size);
        return;
    }
    if (off >= 0x7000 && off < 0x7000 + VICE_BSP_FIFO_SIZE) {
        sgi_vice_write_bytes(s->bsp_out_fifo, off - 0x7000, value, size);
        return;
    }
    if (off >= 0x7800 && off < 0x7800 + VICE_BSP_FIFO_SIZE) {
        sgi_vice_write_bytes(s->bsp_in_fifo, off - 0x7800, value, size);
        return;
    }
    if (off >= 0x8000 && off < 0x8000 + VICE_MSP_DRAM_SIZE) {
        sgi_vice_write_bytes(s->msp_dram, off - 0x8000, value, size);
        return;
    }
    if (off >= 0xf000 && off < 0xf000 + sizeof(s->tlb)) {
        sgi_vice_write_bytes(s->tlb, off - 0xf000, value, size);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_vice: unimplemented write off=0x%" HWADDR_PRIx
                  " size=%u val=0x%" PRIx64 "\n", off, size, value);
}

static const MemoryRegionOps sgi_vice_ops = {
    .read = sgi_vice_read,
    .write = sgi_vice_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void sgi_vice_reset(DeviceState *dev)
{
    SGIViceState *s = SGI_VICE(dev);

    memset(s->chip_regs, 0, sizeof(s->chip_regs));
    memset(s->msp_iram, 0, sizeof(s->msp_iram));
    memset(s->bsp_iram, 0, sizeof(s->bsp_iram));
    memset(s->bsp_table, 0, sizeof(s->bsp_table));
    memset(s->bsp_out_fifo, 0, sizeof(s->bsp_out_fifo));
    memset(s->bsp_in_fifo, 0, sizeof(s->bsp_in_fifo));
    memset(s->msp_dram, 0, sizeof(s->msp_dram));
    memset(s->tlb, 0, sizeof(s->tlb));
    memset(s->debug_regs, 0, sizeof(s->debug_regs));

    s->cfg = 0;
    s->int_enable = 0;
    s->int_status = 0;
    /*
     * @@SEMANTICS@@ tier-i: with no BSP interpreter the only honest state
     * is "halted"; the driver's BSP_HALT_RESET sequence (assert RESET ->
     * HALT|HALT_ACK, spec 2.6.1) is modelled on top of this.
     */
    s->bsp_ctl_stat = VICEBSPCS_HALT | VICEBSPCS_HALT_ACK;
}

static void sgi_vice_realize(DeviceState *dev, Error **errp)
{
    SGIViceState *s = SGI_VICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_vice_ops, s,
                          "sgi-vice", VICE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Single interrupt output -> CRIME line 31 (CRM_INT_VICE). */
    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", 1);
}

static const VMStateDescription vmstate_sgi_vice = {
    .name = "sgi-vice",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(chip_regs, SGIViceState, VICE_CHIP_REGS_SIZE),
        VMSTATE_UINT8_ARRAY(msp_iram, SGIViceState, VICE_MSP_IRAM_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_iram, SGIViceState, VICE_BSP_IRAM_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_table, SGIViceState, VICE_BSP_TABLE_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_out_fifo, SGIViceState, VICE_BSP_FIFO_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_in_fifo, SGIViceState, VICE_BSP_FIFO_SIZE),
        VMSTATE_UINT8_ARRAY(msp_dram, SGIViceState, VICE_MSP_DRAM_SIZE),
        VMSTATE_UINT8_ARRAY(tlb, SGIViceState, VICE_NTLBENTRIES * 4),
        VMSTATE_UINT8_ARRAY(debug_regs, SGIViceState, VICE_DEBUG_END -
                            VICE_DEBUG_BASE),
        VMSTATE_UINT32(cfg, SGIViceState),
        VMSTATE_UINT32(int_enable, SGIViceState),
        VMSTATE_UINT32(int_status, SGIViceState),
        VMSTATE_UINT32(bsp_ctl_stat, SGIViceState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_vice_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_vice_realize;
    device_class_set_legacy_reset(dc, sgi_vice_reset);
    dc->vmsd = &vmstate_sgi_vice;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo sgi_vice_info = {
    .name = TYPE_SGI_VICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIViceState),
    .class_init = sgi_vice_class_init,
};

static void sgi_vice_register_types(void)
{
    type_register_static(&sgi_vice_info);
}

type_init(sgi_vice_register_types)
