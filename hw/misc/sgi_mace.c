/*
 * SGI MACE (Multimedia, Audio, and Communications Engine) emulation
 *
 * Phase 0 implementation:
 * - Serial port 0 (console) with 16550-compatible registers
 *   and 256-byte register spacing, chardev backend
 * - ISA interface (interrupt status/mask, flash/NIC/LED)
 * - UST/MSC timer (free-running counter)
 * - DS17287 RTC stub
 * - All other sub-blocks accept writes and return 0 on reads
 *
 * Physical base: 0x1F000000 (kseg1: 0xBF000000)
 *
 * The MACE serial port is a 16550-compatible UART with unusual register
 * spacing. Each 16550 register occupies a 256-byte block, with the actual
 * byte-sized register at offset +7 (big-endian byte within a doubleword).
 * The PROM accesses registers as byte reads/writes at:
 *   SERIAL_PORT0_BASE = kseg1(ISA_SER1_BASE + 7)
 *   register N at SERIAL_PORT0_BASE + N*256
 *
 * In QEMU's physical address space, serial port 0 registers are at:
 *   0x1F390000 + N*256 + 7  (byte access)
 *
 * Reference:
 *   - IRIX sys/mace.h
 *   - IRIX mace_16c550.c (PROM serial driver)
 *   - MAME src/mame/sgi/mace.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/misc/sgi_mace.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

/* Verbose debug logging - set to 1 to enable */
#define DEBUG_SGI_MACE 1

#if DEBUG_SGI_MACE
#define MACE_DPRINTF(fmt, ...) \
    fprintf(stderr, "MACE: " fmt, ## __VA_ARGS__)
#else
#define MACE_DPRINTF(fmt, ...) do {} while (0)
#endif

/*
 * UST (Universal System Time): free-running counter at 960ns period.
 * Returns a 64-bit value: upper 32 bits = UST, lower 32 bits = MSC.
 * MSC ticks at 1ms intervals.
 */
static uint64_t sgi_mace_get_ust_msc(void)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t ust = ns / MACE_UST_PERIOD_NS;
    uint32_t msc = ns / 1000000;  /* 1ms ticks */
    return ((uint64_t)ust << 32) | msc;
}

/*
 * Serial port helpers
 */

static void sgi_mace_serial_update_irq(SGIMACEState *s, int port)
{
    /* Phase 0: no interrupt routing yet, just update IIR */
    uint8_t iir = IIR_NO_INT | IIR_FIFO_ENA;

    if (s->serial_port[port].ier & 0x02) {
        /* TX empty interrupt enabled and THR is empty */
        if (s->serial_port[port].lsr & LSR_THRE) {
            iir = 0x02 | IIR_FIFO_ENA;  /* THR empty */
        }
    }
    if (s->serial_port[port].ier & 0x01) {
        /* RX data available interrupt enabled */
        if (s->serial_port[port].lsr & LSR_DR) {
            iir = 0x04 | IIR_FIFO_ENA;  /* RX data available (higher priority) */
        }
    }

    s->serial_port[port].iir = iir;
}

static void sgi_mace_serial_rx_push(SGIMACEState *s, int port, uint8_t ch)
{
    if (s->serial_port[port].rx_fifo_count < MACE_SERIAL_FIFO_SIZE) {
        s->serial_port[port].rx_fifo[s->serial_port[port].rx_fifo_tail] = ch;
        s->serial_port[port].rx_fifo_tail =
            (s->serial_port[port].rx_fifo_tail + 1) % MACE_SERIAL_FIFO_SIZE;
        s->serial_port[port].rx_fifo_count++;
        s->serial_port[port].lsr |= LSR_DR;
        sgi_mace_serial_update_irq(s, port);
    }
}

static uint8_t sgi_mace_serial_rx_pop(SGIMACEState *s, int port)
{
    uint8_t ch = 0;

    if (s->serial_port[port].rx_fifo_count > 0) {
        ch = s->serial_port[port].rx_fifo[s->serial_port[port].rx_fifo_head];
        s->serial_port[port].rx_fifo_head =
            (s->serial_port[port].rx_fifo_head + 1) % MACE_SERIAL_FIFO_SIZE;
        s->serial_port[port].rx_fifo_count--;
        if (s->serial_port[port].rx_fifo_count == 0) {
            s->serial_port[port].lsr &= ~LSR_DR;
        }
        sgi_mace_serial_update_irq(s, port);
    }
    return ch;
}

/*
 * Serial port register read.
 * reg_index: 0-7 (16550 register number)
 */
static const char *uart_reg_names[] = {
    "RBR/THR", "IER", "IIR/FCR", "LCR", "MCR", "LSR", "MSR", "SCR"
};

static uint64_t sgi_mace_serial_read(SGIMACEState *s, int port, int reg_index)
{
    uint8_t val = 0;

    if (s->serial_port[port].lcr & LCR_DLAB) {
        /* DLAB=1: divisor latch access */
        switch (reg_index) {
        case UART_REG_THR:  /* DLL */
            return s->serial_port[port].dll;
        case UART_REG_IER:  /* DLH */
            return s->serial_port[port].dlh;
        }
    }

    switch (reg_index) {
    case UART_REG_THR:  /* RBR - Receive Buffer Register */
        val = sgi_mace_serial_rx_pop(s, port);
        break;

    case UART_REG_IER:
        val = s->serial_port[port].ier;
        break;

    case UART_REG_IIR:
        val = s->serial_port[port].iir;
        break;

    case UART_REG_LCR:
        val = s->serial_port[port].lcr;
        break;

    case UART_REG_MCR:
        val = s->serial_port[port].mcr;
        break;

    case UART_REG_LSR:
        val = s->serial_port[port].lsr;
        break;

    case UART_REG_MSR:
        val = s->serial_port[port].msr;
        break;

    case UART_REG_SCR:
        val = s->serial_port[port].scr;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mace: serial%d unimplemented read reg %d\n",
                      port, reg_index);
        break;
    }

    /* Don't log LSR reads - too frequent (polled for TX ready) */
    if (reg_index != UART_REG_LSR) {
        MACE_DPRINTF("serial%d read %s = 0x%02x\n", port,
                     uart_reg_names[reg_index], val);
    }

    return val;
}

/*
 * Serial port register write.
 */
static void sgi_mace_serial_write(SGIMACEState *s, int port,
                                   int reg_index, uint8_t val)
{
    if (s->serial_port[port].lcr & LCR_DLAB) {
        /* DLAB=1: divisor latch access */
        switch (reg_index) {
        case UART_REG_THR:  /* DLL */
            s->serial_port[port].dll = val;
            return;
        case UART_REG_IER:  /* DLH */
            s->serial_port[port].dlh = val;
            return;
        }
    }

    switch (reg_index) {
    case UART_REG_THR:  /* THR - Transmit Holding Register */
        s->serial_port[port].thr = val;
        /* Send character to chardev backend */
        if (port == 0) {
            uint8_t ch = val;
            MACE_DPRINTF("serial%d TX: 0x%02x '%c'\n", port, ch,
                         (ch >= 0x20 && ch < 0x7f) ? ch : '.');
            qemu_chr_fe_write_all(&s->serial, &ch, 1);
        } else {
            MACE_DPRINTF("serial%d TX: 0x%02x\n", port, val);
        }
        /* THR is immediately "empty" after write */
        s->serial_port[port].lsr |= LSR_THRE | LSR_TEMT;
        sgi_mace_serial_update_irq(s, port);
        break;

    case UART_REG_IER:
        MACE_DPRINTF("serial%d IER = 0x%02x\n", port, val);
        s->serial_port[port].ier = val & 0x0f;
        sgi_mace_serial_update_irq(s, port);
        break;

    case UART_REG_IIR:  /* FCR - FIFO Control Register (write-only) */
        MACE_DPRINTF("serial%d FCR = 0x%02x\n", port, val);
        s->serial_port[port].fcr = val;
        if (val & 0x02) {
            /* Clear RX FIFO */
            s->serial_port[port].rx_fifo_head = 0;
            s->serial_port[port].rx_fifo_tail = 0;
            s->serial_port[port].rx_fifo_count = 0;
            s->serial_port[port].lsr &= ~LSR_DR;
        }
        if (val & 0x04) {
            /* Clear TX FIFO */
            s->serial_port[port].lsr |= LSR_THRE | LSR_TEMT;
        }
        break;

    case UART_REG_LCR:
        MACE_DPRINTF("serial%d LCR = 0x%02x%s\n", port, val,
                     (val & LCR_DLAB) ? " (DLAB)" : "");
        s->serial_port[port].lcr = val;
        break;

    case UART_REG_MCR:
        MACE_DPRINTF("serial%d MCR = 0x%02x\n", port, val);
        s->serial_port[port].mcr = val;
        break;

    case UART_REG_LSR:
        /* LSR is mostly read-only, ignore writes */
        break;

    case UART_REG_MSR:
        /* MSR is mostly read-only, ignore writes */
        break;

    case UART_REG_SCR:
        s->serial_port[port].scr = val;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mace: serial%d unimplemented write reg %d = 0x%02x\n",
                      port, reg_index, val);
        break;
    }
}

/*
 * Main MACE read handler.
 * Dispatches to sub-blocks based on offset from MACE_BASE.
 */
static uint64_t sgi_mace_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIMACEState *s = SGI_MACE(opaque);

    /* PCI interface (0x080000-0x0FFFFF) */
    if (offset >= MACE_PCI_OFFSET &&
        offset < MACE_PCI_OFFSET + 0x80000) {
        hwaddr pci_off = offset - MACE_PCI_OFFSET;
        switch (pci_off) {
        case 0x0:   return s->pci_error_addr;
        case 0x4:   return s->pci_error_flags;
        case 0x8:   return s->pci_control;
        case 0xC:   return s->pci_rev_info;
        case 0xCF8: return s->pci_config_addr;
        case 0xCFC:
            /* PCI config data: no devices present, return master abort */
            return 0xFFFFFFFF;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mace: PCI read at offset 0x%06" HWADDR_PRIx "\n",
                          offset);
            return 0;
        }
    }

    /* ISA interface (0x310000-0x31FFFF) */
    if (offset >= MACE_ISA_OFFSET &&
        offset < MACE_ISA_OFFSET + 0x10000) {
        hwaddr isa_off = offset - MACE_ISA_OFFSET;
        switch (isa_off) {
        case ISA_RINGBASE_REG:
        case ISA_RINGBASE_REG + 4:
            return s->isa_ringbase;
        case ISA_FLASH_NIC_REG:
        case ISA_FLASH_NIC_REG + 4:
            return s->isa_flash_nic;
        case ISA_INT_STS_REG:
        case ISA_INT_STS_REG + 4:
            return s->isa_int_status;
        case ISA_INT_MSK_REG:
        case ISA_INT_MSK_REG + 4:
            return s->isa_int_mask;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mace: ISA read at offset 0x%06" HWADDR_PRIx "\n",
                          offset);
            return 0;
        }
    }

    /* UST/MSC timer (0x340000-0x34FFFF) */
    if (offset >= MACE_UST_MSC_OFFSET &&
        offset < MACE_UST_MSC_OFFSET + 0x10000) {
        hwaddr ust_off = offset - MACE_UST_MSC_OFFSET;
        switch (ust_off) {
        case UST_MSC_REG:
        case UST_MSC_REG + 4:
            return sgi_mace_get_ust_msc();
        case UST_COMPARE1:
        case UST_COMPARE1 + 4:
            return s->ust_compare[0];
        case UST_COMPARE2:
        case UST_COMPARE2 + 4:
            return s->ust_compare[1];
        case UST_COMPARE3:
        case UST_COMPARE3 + 4:
            return s->ust_compare[2];
        default:
            return sgi_mace_get_ust_msc();
        }
    }

    /*
     * Serial ports (ISA extended space).
     * Serial 1: 0x390000-0x391FFF (port 0, console)
     * Serial 2: 0x398000-0x399FFF (port 1)
     *
     * Within each serial block, registers are at 256-byte intervals.
     * The actual byte is at offset +7 within each 8-byte doubleword.
     * We accept any access within the 256-byte block for a given register.
     */
    if (offset >= MACE_SER1_OFFSET &&
        offset < MACE_SER1_OFFSET + MACE_SERIAL_SIZE) {
        hwaddr ser_off = offset - MACE_SER1_OFFSET;
        int reg_index = ser_off >> MACE_SERIAL_REG_SHIFT;
        if (reg_index < 8) {
            return sgi_mace_serial_read(s, 0, reg_index);
        }
        return 0;
    }
    if (offset >= MACE_SER2_OFFSET &&
        offset < MACE_SER2_OFFSET + MACE_SERIAL_SIZE) {
        hwaddr ser_off = offset - MACE_SER2_OFFSET;
        int reg_index = ser_off >> MACE_SERIAL_REG_SHIFT;
        if (reg_index < 8) {
            return sgi_mace_serial_read(s, 1, reg_index);
        }
        return 0;
    }

    /* RTC (0x3A0000-0x3A7FFF) */
    if (offset >= MACE_RTC_OFFSET &&
        offset < MACE_RTC_OFFSET + 0x8000) {
        hwaddr rtc_off = offset - MACE_RTC_OFFSET;
        /*
         * DS17287 RTC is accessed via address/data port pair.
         * The PROM accesses at ISA_RTC_BASE+7, using byte accesses
         * within 8-byte doublewords. We decode the register from
         * the offset within the RTC region.
         *
         * Register 0 (index): at base
         * Register 1 (data):  at base + 1 (or base + 8 in 8-byte spacing)
         */
        int rtc_reg = (rtc_off >> 3) & 0x1;  /* 0 = index, 1 = data */
        if (rtc_reg == 0) {
            return s->rtc_index;
        } else {
            /* Data port - read from indexed register */
            uint8_t idx = s->rtc_index & 0x7f;
            if (idx < sizeof(s->rtc_ram)) {
                return s->rtc_ram[idx];
            }
            return 0;
        }
    }

    /* All other accesses return 0 with a log message */
    qemu_log_mask(LOG_UNIMP,
                  "sgi_mace: unimplemented read at offset 0x%06" HWADDR_PRIx
                  " (size %d)\n", offset, size);
    return 0;
}

static void sgi_mace_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    SGIMACEState *s = SGI_MACE(opaque);

    /* PCI interface */
    if (offset >= MACE_PCI_OFFSET &&
        offset < MACE_PCI_OFFSET + 0x80000) {
        hwaddr pci_off = offset - MACE_PCI_OFFSET;
        switch (pci_off) {
        case 0x0:   s->pci_error_addr = value; break;
        case 0x4:   s->pci_error_flags = value; break;
        case 0x8:   s->pci_control = value; break;
        case 0xCF8: s->pci_config_addr = value; break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mace: PCI write at offset 0x%06" HWADDR_PRIx
                          " value 0x%08" PRIx64 "\n", offset, value);
            break;
        }
        return;
    }

    /* ISA interface */
    if (offset >= MACE_ISA_OFFSET &&
        offset < MACE_ISA_OFFSET + 0x10000) {
        hwaddr isa_off = offset - MACE_ISA_OFFSET;
        switch (isa_off) {
        case ISA_RINGBASE_REG:
        case ISA_RINGBASE_REG + 4:
            s->isa_ringbase = value;
            break;
        case ISA_FLASH_NIC_REG:
        case ISA_FLASH_NIC_REG + 4:
            s->isa_flash_nic = value;
            break;
        case ISA_INT_STS_REG:
        case ISA_INT_STS_REG + 4:
            /* Write to clear */
            s->isa_int_status &= ~value;
            break;
        case ISA_INT_MSK_REG:
        case ISA_INT_MSK_REG + 4:
            s->isa_int_mask = value;
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mace: ISA write at offset 0x%06" HWADDR_PRIx
                          " value 0x%08" PRIx64 "\n", offset, value);
            break;
        }
        return;
    }

    /* UST/MSC timer */
    if (offset >= MACE_UST_MSC_OFFSET &&
        offset < MACE_UST_MSC_OFFSET + 0x10000) {
        hwaddr ust_off = offset - MACE_UST_MSC_OFFSET;
        switch (ust_off) {
        case UST_COMPARE1:
        case UST_COMPARE1 + 4:
            s->ust_compare[0] = value;
            break;
        case UST_COMPARE2:
        case UST_COMPARE2 + 4:
            s->ust_compare[1] = value;
            break;
        case UST_COMPARE3:
        case UST_COMPARE3 + 4:
            s->ust_compare[2] = value;
            break;
        default:
            /* UST counter is read-only */
            break;
        }
        return;
    }

    /* Serial port 0 */
    if (offset >= MACE_SER1_OFFSET &&
        offset < MACE_SER1_OFFSET + MACE_SERIAL_SIZE) {
        hwaddr ser_off = offset - MACE_SER1_OFFSET;
        int reg_index = ser_off >> MACE_SERIAL_REG_SHIFT;
        if (reg_index < 8) {
            sgi_mace_serial_write(s, 0, reg_index, value & 0xff);
        }
        return;
    }

    /* Serial port 1 */
    if (offset >= MACE_SER2_OFFSET &&
        offset < MACE_SER2_OFFSET + MACE_SERIAL_SIZE) {
        hwaddr ser_off = offset - MACE_SER2_OFFSET;
        int reg_index = ser_off >> MACE_SERIAL_REG_SHIFT;
        if (reg_index < 8) {
            sgi_mace_serial_write(s, 1, reg_index, value & 0xff);
        }
        return;
    }

    /* RTC */
    if (offset >= MACE_RTC_OFFSET &&
        offset < MACE_RTC_OFFSET + 0x8000) {
        hwaddr rtc_off = offset - MACE_RTC_OFFSET;
        int rtc_reg = (rtc_off >> 3) & 0x1;
        if (rtc_reg == 0) {
            s->rtc_index = value & 0x7f;
        } else {
            uint8_t idx = s->rtc_index & 0x7f;
            if (idx < sizeof(s->rtc_ram)) {
                s->rtc_ram[idx] = value & 0xff;
            }
        }
        return;
    }

    /* All other writes are no-ops with logging */
    qemu_log_mask(LOG_UNIMP,
                  "sgi_mace: unimplemented write at offset 0x%06" HWADDR_PRIx
                  " value 0x%08" PRIx64 " (size %d)\n",
                  offset, value, size);
}

static const MemoryRegionOps sgi_mace_ops = {
    .read = sgi_mace_read,
    .write = sgi_mace_write,
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

/* Chardev receive handler - incoming data from host → guest serial port 0 */
static int sgi_mace_serial_can_receive(void *opaque)
{
    SGIMACEState *s = SGI_MACE(opaque);
    return MACE_SERIAL_FIFO_SIZE - s->serial_port[0].rx_fifo_count;
}

static void sgi_mace_serial_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIMACEState *s = SGI_MACE(opaque);
    int i;

    for (i = 0; i < size; i++) {
        sgi_mace_serial_rx_push(s, 0, buf[i]);
    }
}

static void sgi_mace_reset(DeviceState *dev)
{
    SGIMACEState *s = SGI_MACE(dev);
    int i;

    s->isa_ringbase = 0;
    s->isa_flash_nic = 0;
    s->isa_int_status = 0;
    s->isa_int_mask = 0;
    memset(s->ust_compare, 0, sizeof(s->ust_compare));

    s->pci_error_addr = 0;
    s->pci_error_flags = 0;
    s->pci_control = 0;
    s->pci_rev_info = 0;
    s->pci_config_addr = 0;

    s->rtc_index = 0;
    memset(s->rtc_ram, 0, sizeof(s->rtc_ram));

    for (i = 0; i < MACE_NUM_SERIAL; i++) {
        memset(&s->serial_port[i], 0, sizeof(s->serial_port[i]));
        /* LSR: TX empty + TX holding register empty at reset */
        s->serial_port[i].lsr = LSR_THRE | LSR_TEMT;
        /* IIR: no interrupt pending, FIFOs not enabled */
        s->serial_port[i].iir = IIR_NO_INT;
        /* MSR: CTS + DSR asserted (loopback-friendly) */
        s->serial_port[i].msr = 0x30;
    }
}

static void sgi_mace_realize(DeviceState *dev, Error **errp)
{
    SGIMACEState *s = SGI_MACE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_mace_ops, s,
                          "sgi-mace", MACE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Output IRQs to CRIME (16 lines) */
    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", 16);

    /* Connect chardev to serial port 0 */
    qemu_chr_fe_set_handlers(&s->serial,
                              sgi_mace_serial_can_receive,
                              sgi_mace_serial_receive,
                              NULL, NULL, s, NULL, true);
}

static const Property sgi_mace_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIMACEState, serial),
};

static const VMStateDescription vmstate_sgi_mace = {
    .name = "sgi-mace",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(isa_ringbase, SGIMACEState),
        VMSTATE_UINT64(isa_flash_nic, SGIMACEState),
        VMSTATE_UINT64(isa_int_status, SGIMACEState),
        VMSTATE_UINT64(isa_int_mask, SGIMACEState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_mace_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_mace_realize;
    device_class_set_legacy_reset(dc, sgi_mace_reset);
    dc->vmsd = &vmstate_sgi_mace;
    device_class_set_props(dc, sgi_mace_properties);
}

static const TypeInfo sgi_mace_info = {
    .name = TYPE_SGI_MACE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIMACEState),
    .class_init = sgi_mace_class_init,
};

static void sgi_mace_register_types(void)
{
    type_register_static(&sgi_mace_info);
}

type_init(sgi_mace_register_types)
