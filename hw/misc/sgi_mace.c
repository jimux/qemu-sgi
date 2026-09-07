/*
 * SGI MACE (Multimedia, Audio, and Communications Engine) emulation
 *
 * Phase 0 implementation:
 * - Serial port 0 (console) with 16550-compatible registers
 *   and 256-byte register spacing, chardev backend
 * - ISA serial DMA ring-buffer engine (spec §5.1.5/§5.1.6) feeding
 *   the UART through the memory bus, with the ISA serial interrupt
 *   bits (Tx/Rx DMA threshold) gated by the ISA mask and routed to
 *   CRIME INTSTAT bit 4 (MACE_PERIPH_SERIAL) -> CPU IP2
 * - ISA interface (interrupt status/mask, flash/NIC/LED, ring base)
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
 * (sgi_o2.c overlays a serial_mm device on this window, which the ISA
 * serial DMA engine reaches through the memory bus.)
 *
 * Reference:
 *   - MACE ASIC spec, ISA Bus Interface chapter (§5)
 *   - IRIX sys/mace.h, sys/uart16550.h, io/sio_ti16550.c, io/mace.c
 *   - MAME src/mame/sgi/mace.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/misc/sgi_mace.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

/* Verbose debug logging - set to 1 to enable */
#define DEBUG_SGI_MACE 0

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
#if DEBUG_SGI_MACE
static const char *uart_reg_names[] = {
    "RBR/THR", "IER", "IIR/FCR", "LCR", "MCR", "LSR", "MSR", "SCR"
};
#endif

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
 * ============================================================
 *                  PCI host bridge
 *
 * The MACE is the PCI host bridge of the O2 (spec §9): a QEMU PCI
 * root bus ("mace-pci", devices on PCI slots 1..5) plus the six
 * host-bridge registers at MACE_PCI_OFFSET that forward config
 * cycles (CONFIG_ADDRESS/CONFIG_DATA) to it.  CPU windows:
 *   0x18000000 + n  -> PCI I/O  space n
 *   0x1A000000 + n  -> PCI memory 0x80000000 + n (kernel BAR region)
 * Interrupts: PCI INTx -> map_irq (pcimh mace_ivec table) -> PCI
 * interrupt input 0..7 -> PCI_CONTROL enable bit -> CRIME bit 8+n.
 * Bridge error interrupts (master abort etc.) -> CRIME bit 7.
 */

/*
 * Re-evaluate the CRIME line for one PCI interrupt input (level
 * latched, gated by the PCI_CONTROL enable bit).
 */
static void sgi_mace_pci_int_update(SGIMACEState *s, int input)
{
    int level = s->pci_int_level[input] &&
        (s->pci_control & (1 << input));
    qemu_set_irq(s->crime_irq[8 + input], level);
}

/*
 * CRIME bit 7 (MACE_PCI_BRIDGE): error flags ANDed with their
 * PCI_CONTROL interrupt-enable bits (spec TABLE 85 bits 24..31).
 */
static void sgi_mace_pci_error_irq_update(SGIMACEState *s)
{
    uint32_t ena = s->pci_control & MACE_PCI_CONTROL_ERR_INT_MASK;
    uint32_t flags = s->pci_error_flags;
    int level = 0;

    if (flags & MACE_PERR_MASTER_ABORT) {
        level |= !!(ena & 0x80000000);
    }
    if (flags & MACE_PERR_TARGET_ABORT) {
        level |= !!(ena & 0x40000000);
    }
    if (flags & MACE_PERR_DATA_PARITY_ERR) {
        level |= !!(ena & 0x20000000);
    }
    if (flags & MACE_PERR_RETRY_ERR) {
        level |= !!(ena & 0x10000000);
    }
    if (flags & MACE_PERR_ILLEGAL_CMD) {
        level |= !!(ena & 0x08000000);
    }
    if (flags & MACE_PERR_SYSTEM_ERR) {
        level |= !!(ena & 0x04000000);
    }
    if (flags & MACE_PERR_PARITY_ERR) {
        level |= !!(ena & 0x02000000);
    }
    if (flags & MACE_PERR_OVERRUN) {
        level |= !!(ena & 0x01000000);
    }
    qemu_set_irq(s->crime_irq[7], level);
}

/* PCI bus set_irq: input is the MACE interrupt number 0..7 */
static void sgi_mace_pci_set_irq(void *opaque, int irq, int level)
{
    SGIMACEState *s = SGI_MACE(opaque);

    if (irq < 0 || irq >= MACE_PCI_NUM_INTS) {
        return;
    }
    s->pci_int_level[irq] = level;
    sgi_mace_pci_int_update(s, irq);
}

/*
 * Map PCI device INTA..INTD to the MACE interrupt inputs, per the
 * kernel's slot->vector table (pcimh.c mace_ivec) and spec §9.6:
 *   slot 1 (SCSI ctlr 0) INTA -> input 0 (CRIME bit 8, MACE_PCI_SCSI0)
 *   slot 2 (SCSI ctlr 1) INTA -> input 1 (CRIME bit 9, MACE_PCI_SCSI1)
 *   slots 3..5 INTA -> inputs 2..4; INTB/C/D share inputs 5..7
 * (spiral: {0,5,6,7}, {1,7,5,6}, {2,6,7,5} relative to the slot row).
 */
static int sgi_mace_pci_map_irq(PCIDevice *pci_dev, int irq_num)
{
    static const int mace_ivec[5][4] = {
        /* slot 1: SCSI controller 0 (INTA only; B/C/D not routable) */
        { 0, 0, 0, 0 },
        /* slot 2: SCSI controller 1 */
        { 1, 1, 1, 1 },
        /* slot 3: PCI expansion slot 0 */
        { 2, 5, 6, 7 },
        /* slot 4: PCI expansion slot 1 */
        { 3, 7, 5, 6 },
        /* slot 5: PCI expansion slot 2 */
        { 4, 6, 7, 5 },
    };
    int slot = PCI_SLOT(pci_dev->devfn);

    if (slot < 1 || slot > 5 || irq_num < 0 || irq_num > 3) {
        qemu_log_mask(LOG_UNIMP, "sgi_mace: PCI IRQ from slot %d pin %d\n",
                      slot, irq_num);
        return 0;
    }
    return mace_ivec[slot - 1][irq_num];
}

/*
 * Record a master abort on a config cycle (spec §9.4.2): set the
 * error flags and capture the PCI address (type-0 image for bus 0:
 * one-hot device select on AD[15:11], function AD[10:8], reg AD[7:2]).
 */
static void sgi_mace_pci_master_abort(SGIMACEState *s, uint32_t addr)
{
    int slot = (addr >> 11) & 0x1f;
    uint32_t pci_ad;

    if (((addr >> 16) & 0xff) == 0) {
        pci_ad = (1u << (11 + slot)) | (addr & 0xfffffc);
    } else {
        pci_ad = addr;
    }

    if (!(s->pci_error_flags & MACE_PERR_MASTER_ABORT)) {
        s->pci_error_addr = pci_ad;
    }
    s->pci_error_flags |= MACE_PERR_MASTER_ABORT | MACE_PERR_CONFIG_ADDR |
        MACE_PERR_MASTER_ABORT_AV;
    sgi_mace_pci_error_irq_update(s);
}

/* Perform one config-space read through CONFIG_DATA (32-bit word) */
static uint32_t sgi_mace_pci_config_read(SGIMACEState *s)
{
    uint32_t addr = s->pci_config_addr;

    if (!(addr & 0x80000000)) {
        /* No config cycle enabled: no bus transaction, no error */
        return 0xFFFFFFFF;
    }
    if (!pci_find_device(s->pci_bus, (addr >> 16) & 0xff,
                         ((addr >> 11) & 0x1f) << 3 | ((addr >> 8) & 7))) {
        sgi_mace_pci_master_abort(s, addr);
        return 0xFFFFFFFF;
    }
    return pci_data_read(s->pci_bus, addr, 4);
}

/* Perform one config-space write through CONFIG_DATA */
static void sgi_mace_pci_config_write(SGIMACEState *s, uint32_t value)
{
    uint32_t addr = s->pci_config_addr;

    if (!(addr & 0x80000000)) {
        return;
    }
    if (!pci_find_device(s->pci_bus, (addr >> 16) & 0xff,
                         ((addr >> 11) & 0x1f) << 3 | ((addr >> 8) & 7))) {
        sgi_mace_pci_master_abort(s, addr);
        return;
    }
    pci_data_write(s->pci_bus, addr, value, 4);
}

/*
 * PCI register read (offset relative to MACE_PCI_OFFSET).
 * Spec §9.4: only six registers; everything else reads 0xFF.
 */
static uint64_t sgi_mace_pci_reg_read(SGIMACEState *s, hwaddr pci_off,
                                      unsigned size)
{
    switch (pci_off) {
    case MACE_PCI_ERROR_ADDR:
        return s->pci_error_addr;
    case MACE_PCI_ERROR_FLAGS:
        /* bits 1,3:2 are fixed capability status (FBC, DEVSEL=medium) */
        return s->pci_error_flags | 0x06;
    case MACE_PCI_CONTROL:
        return s->pci_control;
    case MACE_PCI_REV_INFO_R:
        return s->pci_rev_info;
    case MACE_PCI_CONFIG_DATA:
        return sgi_mace_pci_config_read(s);
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mace: PCI read at offset 0x%06" HWADDR_PRIx
                      " (unimplemented -> 0xff)\n", pci_off + MACE_PCI_OFFSET);
        return 0xFFFFFFFF;
    }
}

static void sgi_mace_pci_reg_write(SGIMACEState *s, hwaddr pci_off,
                                   uint64_t value, unsigned size)
{
    switch (pci_off) {
    case MACE_PCI_ERROR_ADDR:
        /* read-only */
        break;
    case MACE_PCI_ERROR_FLAGS: {
        /*
         * Bits 4 and 23..31 clear on writing 0, preserve on 1;
         * clearing 28..31 also clears the matching address-valid
         * flag 16..19 (spec §9.4.2).
         */
        uint32_t old = s->pci_error_flags;
        uint32_t wclear = 0xff800010;
        uint32_t cleared = old & wclear & ~((uint32_t)value & wclear);
        uint32_t av;

        s->pci_error_flags = old & ~cleared;
        for (av = 0; av < 4; av++) {
            if (cleared & (0x80000000 >> av)) {
                s->pci_error_flags &= ~(0x00080000 >> av);
            }
        }
        sgi_mace_pci_error_irq_update(s);
        break;
    }
    case MACE_PCI_CONTROL: {
        int i;
        uint32_t diff = s->pci_control ^ (uint32_t)value;

        s->pci_control = (uint32_t)value;
        if (diff & 0xff) {
            /* interrupt gate bits changed: re-evaluate the lines */
            for (i = 0; i < MACE_PCI_NUM_INTS; i++) {
                if (diff & (1 << i)) {
                    sgi_mace_pci_int_update(s, i);
                }
            }
        }
        if (diff & MACE_PCI_CONTROL_ERR_INT_MASK) {
            sgi_mace_pci_error_irq_update(s);
        }
        break;
    }
    case MACE_PCI_FLUSH_W:
        /* write = invalidate read buffers (we have none); discard */
        break;
    case MACE_PCI_CONFIG_ADDR:
        s->pci_config_addr = (uint32_t)value;
        break;
    case MACE_PCI_CONFIG_DATA:
        sgi_mace_pci_config_write(s, (uint32_t)value);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mace: PCI write at offset 0x%06" HWADDR_PRIx
                      " value 0x%08" PRIx64 " (discarded)\n",
                      pci_off + MACE_PCI_OFFSET, value);
        break;
    }
}
/*
 * ============================================================
 *                ISA serial DMA engine
 *
 * The two MACE ISA serial ports are served by DMA ring buffers in
 * guest RAM (spec §5.1.5/§5.1.6): eight 4KB rings hang off the
 * 32KB-aligned base programmed into ISA_RINGBASE; the serial ports
 * use ring IDs 4..7 (S1 TX, S1 RX, S2 TX, S2 RX).  Each 32-byte ring
 * block carries 16 ctrl/data pairs, packed big-endian four pairs per
 * 64-bit word with the control bytes in the high half (spec §5.2.2).
 *
 * The IRIX driver (kern/io/sio_ti16550.c, "mh16550") never touches
 * THR/RBR once DMA is active: writes go into the TX ring and the
 * hardware write pointer is bumped; the engine drains the ring to the
 * UART and raises the "Tx DMA threshold" ISA interrupt when the
 * selected threshold (here: ring empty) is reached.  Received
 * characters are packed into the RX ring by the engine, which owns
 * the RX write pointer, and the "Rx DMA threshold" (ring not empty)
 * interrupt wakes the STREAMS read path.  The ISA interrupt bits are
 * gated by ISA_INT_MSK and fan out to CRIME INTSTAT bit 4
 * (MACE_PERIPH_SERIAL, kernel sys/mace.h) -> CPU IP2.
 *
 * The UART itself is the serial_mm device that sgi_o2.c overlays at
 * MACE_SER1_OFFSET, so this engine reaches it the way the real DMA
 * engine does: through the memory bus, one register access at a time.
 */
static uint32_t sgi_mace_isa_dma_depth(SGIMACEState *s, int port, int dir)
{
    /*
     * Channel Current Ring Depth (spec TABLE 59): write pointer minus
     * read pointer, i.e. bytes pending (always a multiple of 32 since
     * both pointers are block-aligned).  0 = empty, all-ones (127
     * blocks) = full per the N-1 rule of spec §1.2.1.
     */
    return (s->isa_dma_wptr[port][dir] + ISA_RING_SIZE
            - s->isa_dma_rptr[port][dir]) & (ISA_RING_SIZE - 1);
}

/*
 * FIFO-threshold style interrupt condition (spec §1.2.2, TABLE 58).
 * dir 0 = TX (output channel: level conditions are "< N%"), dir 1 =
 * RX (input channel: ">= N%").  A held-in-reset channel keeps its
 * interrupt output inactive.
 */
static bool sgi_mace_isa_dma_threshold(SGIMACEState *s, int port, int dir)
{
    uint64_t ctrl = s->isa_dma_ctrl[port][dir];
    unsigned thr = (ctrl & ISA_DMA_CTRL_THR_MASK) >> ISA_DMA_CTRL_THR_SHIFT;
    uint32_t blocks = sgi_mace_isa_dma_depth(s, port, dir) / ISA_RING_BLOCK_SIZE;
    bool is_output = (dir == 0);

    if (ctrl & ISA_DMA_CTRL_RESET) {
        return false;
    }
    switch (thr) {
    case ISA_DMA_THR_NONE:
        return false;
    case ISA_DMA_THR_25:
        return is_output ? blocks < ISA_RING_BLOCKS / 4
                         : blocks >= ISA_RING_BLOCKS / 4;
    case ISA_DMA_THR_50:
        return is_output ? blocks < ISA_RING_BLOCKS / 2
                         : blocks >= ISA_RING_BLOCKS / 2;
    case ISA_DMA_THR_75:
        return is_output ? blocks < ISA_RING_BLOCKS * 3 / 4
                         : blocks >= ISA_RING_BLOCKS * 3 / 4;
    case ISA_DMA_THR_EMPTY:
        return blocks == 0;
    case ISA_DMA_THR_NEMPTY:
        return blocks != 0;
    case ISA_DMA_THR_FULL:
        return blocks >= ISA_RING_BLOCKS - 1;
    case ISA_DMA_THR_NFULL:
        return blocks < ISA_RING_BLOCKS - 1;
    }
    return false;
}

/*
 * Re-evaluate the gated MACE ISA interrupt output for the CRIME
 * lines.  The ISA bit groups map onto MACE vectors (kernel
 * sys/mace.h): audio bits 0..7 -> 6, misc (RTC/PS2/timers) bits 8..15
 * -> 5, peripheral (parallel + serial DMA) bits 16..31 -> 4.  Only
 * the serial DMA levels are modeled; the other groups stay 0.
 */
static void sgi_mace_isa_int_update(SGIMACEState *s)
{
    uint64_t pending = s->isa_int_status & s->isa_int_mask;

    qemu_set_irq(s->crime_irq[6], !!(pending & ISA_INT_AUDIO_MASK));
    qemu_set_irq(s->crime_irq[5], !!(pending & ISA_INT_MISC_MASK));
    qemu_set_irq(s->crime_irq[4], !!(pending & ISA_INT_PERIPH_MASK));
}

/* Recompute one serial port's live ISA status bits. */
static void sgi_mace_isa_serial_update(SGIMACEState *s, int port)
{
    uint64_t tx_thir = port ? ISA_INT_S1_TX_THIR : ISA_INT_S0_TX_THIR;
    uint64_t rx_thir = port ? ISA_INT_S1_RX_THIR : ISA_INT_S0_RX_THIR;
    uint64_t preq = port ? ISA_INT_S1_TX_PREQ : ISA_INT_S0_TX_PREQ;
    uint64_t bits = 0;

    if (sgi_mace_isa_dma_threshold(s, port, 0)) {
        bits |= tx_thir;
    }
    if (sgi_mace_isa_dma_threshold(s, port, 1)) {
        bits |= rx_thir;
    }
    if (s->isa_dma_preq[port]) {
        bits |= preq;
    }

    /*
     * The device-interrupt-request and error bits (DIR/MEMERR/OVR)
     * stay 0: the UART's own interrupt line is not wired to MACE in
     * this machine, and the IRIX DMA-mode driver never enables them
     * (kern/io/sio_ti16550.c sio_drivermask/sio_currntmask).
     */
    s->isa_int_status = (s->isa_int_status & ~(tx_thir | rx_thir | preq)) | bits;
    sgi_mace_isa_int_update(s);
}

/*
 * One UART register access over the memory bus (the DMA engine's
 * view of the external Super I/O chip).  Register N lives at
 * port base + N*256 + 7 (big-endian byte lane).
 */
static void sgi_mace_uart_bus_write(SGIMACEState *s, int port,
                                    int reg, uint8_t val)
{
    hwaddr a = MACE_BASE + (port ? MACE_SER2_OFFSET : MACE_SER1_OFFSET)
             + ((hwaddr)reg << MACE_SERIAL_REG_SHIFT) + 7;

    address_space_write(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                        &val, 1);
}

static uint8_t sgi_mace_uart_bus_read(SGIMACEState *s, int port, int reg)
{
    hwaddr a = MACE_BASE + (port ? MACE_SER2_OFFSET : MACE_SER1_OFFSET)
             + ((hwaddr)reg << MACE_SERIAL_REG_SHIFT) + 7;
    uint8_t val = 0;

    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       &val, 1);
    return val;
}

/*
 * Drain the TX ring of one serial port: walk 32-byte blocks from the
 * hardware read pointer up to the software write pointer and perform
 * each pair's opcode against the UART.  The engine owns the TX read
 * pointer (output channel, spec §5.1.6).  Draining is synchronous:
 * with the emulated UART's THR always ready there is no baud-rate
 * pacing to model, so the ring is empty again by the time the write
 * to the pointer register retires -- exactly the condition the IRIX
 * "Tx DMA threshold = empty" interrupt selection asks for.
 */
static void sgi_mace_isa_tx_drain(SGIMACEState *s, int port)
{
    uint64_t ctrl = s->isa_dma_ctrl[port][0];
    hwaddr rbase = s->isa_ringbase & 0xffff8000ULL;

    if ((ctrl & (ISA_DMA_CTRL_RESET | ISA_DMA_CTRL_ENABLE))
        != ISA_DMA_CTRL_ENABLE) {
        return;    /* held in reset, or disabled (frozen) */
    }
    while (s->isa_dma_rptr[port][0] != s->isa_dma_wptr[port][0]) {
        uint8_t blk[ISA_RING_BLOCK_SIZE];
        hwaddr a = rbase + (((hwaddr)ISA_RING_ID_S0_TX + port * 2) << 12)
                 + (s->isa_dma_rptr[port][0]
                    & (ISA_RING_SIZE - ISA_RING_BLOCK_SIZE));
        int i;

        address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                           blk, sizeof(blk));
        /*
         * Four 64-bit words per block, each word = four pairs packed
         * big-endian: [ctrl0..3][data0..3] (spec §5.2.2 figure).
         */
        for (i = 0; i < ISA_RING_BLOCK_SIZE / 2; i++) {
            int w = (i >> 2) * 8;        /* word base offset in block */
            uint8_t opc = blk[w + (i & 3)];
            uint8_t data = blk[w + 4 + (i & 3)];

            switch (opc & 0xc0) {
            case ISA_DMA_OC_WTHR:
                sgi_mace_uart_bus_write(s, port, UART_REG_THR, data);
                break;
            case ISA_DMA_OC_WMCR:
                sgi_mace_uart_bus_write(s, port, UART_REG_MCR, data);
                break;
            case ISA_DMA_OC_DELAY:
                /* inter-pair idle (MIDI spacing): nothing to pace */
                break;
            default:
                /* invalid pair: skip */
                break;
            }
            if (opc & ISA_DMA_OC_TxINTR) {
                /* pair-request interrupt: sticky until channel reset */
                s->isa_dma_preq[port] = true;
            }
        }
        s->isa_dma_rptr[port][0] = (s->isa_dma_rptr[port][0]
                                    + ISA_RING_BLOCK_SIZE)
                                  & (ISA_RING_SIZE - 1);
    }
    sgi_mace_isa_serial_update(s, port);
}

/*
 * RX DMA engine poll.  Physically the DMA engine watches the UART's
 * rx-FIFO ready request; the UART here is the serial_mm device, so
 * the engine polls LSR/RBR through the bus.  Each received character
 * is written to the RX ring as one valid pair padded with invalid
 * pairs (spec §5.2.2 allows this for short runs), consuming one
 * 32-byte block, and the hardware RX write pointer advances.  The
 * kernel reads whole blocks and advances the software RX read
 * pointer (kern/io/sio_ti16550.c dma_rx_block/INC_RCV_PTR).
 *
 * Only serial port #1 (the console, the port with a chardev behind
 * the UART) is polled; port #2 has no input source in this machine.
 */
static void sgi_mace_isa_rx_poll(void *opaque)
{
    SGIMACEState *s = SGI_MACE(opaque);
    const int port = 0;
    bool progressed = false;

    if ((s->isa_dma_ctrl[port][1] & (ISA_DMA_CTRL_RESET | ISA_DMA_CTRL_ENABLE))
        == ISA_DMA_CTRL_ENABLE) {
        while (sgi_mace_isa_dma_depth(s, port, 1) / ISA_RING_BLOCK_SIZE
               < ISA_RING_BLOCKS - 1) {
            uint8_t lsr = sgi_mace_uart_bus_read(s, port, UART_REG_LSR);
            uint8_t blk[ISA_RING_BLOCK_SIZE];
            hwaddr a;
            uint8_t ch;

            if (!(lsr & LSR_DR)) {
                break;
            }
            ch = sgi_mace_uart_bus_read(s, port, UART_REG_THR); /* RBR */

            /*
             * One valid pair padded with invalid pairs.  Pair 0 of
             * the block: ctrl byte at word 0 offset 0, data byte at
             * word 0 offset 4 (spec §5.2.2 packing).
             */
            memset(blk, 0, sizeof(blk));
            blk[0] = ISA_DMA_IC_VALID;
            blk[4] = ch;
            a = (s->isa_ringbase & 0xffff8000ULL)
              + (((hwaddr)ISA_RING_ID_S0_RX + port * 2) << 12)
              + (s->isa_dma_wptr[port][1]
                 & (ISA_RING_SIZE - ISA_RING_BLOCK_SIZE));
            address_space_write(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, blk, sizeof(blk));
            s->isa_dma_wptr[port][1] = (s->isa_dma_wptr[port][1]
                                        + ISA_RING_BLOCK_SIZE)
                                      & (ISA_RING_SIZE - 1);
            progressed = true;
        }
        if (progressed) {
            sgi_mace_isa_serial_update(s, port);
        }
        timer_mod_ns(s->isa_rx_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                     + ISA_DMA_RX_POLL_NS);
    }
}

/* Arm or stop the console RX poll timer with the channel state. */
static void sgi_mace_isa_rx_arm(SGIMACEState *s)
{
    bool active = (s->isa_dma_ctrl[0][1]
                   & (ISA_DMA_CTRL_RESET | ISA_DMA_CTRL_ENABLE))
                  == ISA_DMA_CTRL_ENABLE;

    if (active) {
        timer_mod_ns(s->isa_rx_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                     + ISA_DMA_RX_POLL_NS);
    } else {
        timer_del(s->isa_rx_timer);
    }
}

/*
 * ISA register space read (offset relative to MACE_ISA_OFFSET).
 * Four 16KB pages (spec §5.1 TABLE 52): page 0 = interrupt/ring
 * base, page 1 = parallel DMA (unmodeled), pages 2/3 = the two
 * serial DMA channels.
 */
static uint64_t sgi_mace_isa_read(SGIMACEState *s, hwaddr isa_off,
                                  unsigned size)
{
    int port, reg;

    if (isa_off >= ISA_SER0_DMA_PAGE) {
        port = (isa_off >= ISA_SER1_DMA_PAGE) ? 1 : 0;
        /* unused space in a page aliases onto the registers (§5.1) */
        reg = (isa_off - (port ? ISA_SER1_DMA_PAGE : ISA_SER0_DMA_PAGE))
              & 0x38;
        switch (reg) {
        case ISA_DMA_TX_CTRL:
            return s->isa_dma_ctrl[port][0];
        case ISA_DMA_TX_RPTR:
            return s->isa_dma_rptr[port][0];
        case ISA_DMA_TX_WPTR:
            return s->isa_dma_wptr[port][0];
        case ISA_DMA_TX_DEPTH:
            return sgi_mace_isa_dma_depth(s, port, 0);
        case ISA_DMA_RX_CTRL:
            return s->isa_dma_ctrl[port][1];
        case ISA_DMA_RX_RPTR:
            return s->isa_dma_rptr[port][1];
        case ISA_DMA_RX_WPTR:
            return s->isa_dma_wptr[port][1];
        case ISA_DMA_RX_DEPTH:
            return sgi_mace_isa_dma_depth(s, port, 1);
        default:
            return 0;
        }
    }

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
        /* parallel DMA context regs / DP-RAM: not modeled, read 0 */
        return 0;
    }
}

static void sgi_mace_isa_write(SGIMACEState *s, hwaddr isa_off,
                               uint64_t value, unsigned size)
{
    int port, reg;

    if (isa_off >= ISA_SER0_DMA_PAGE) {
        port = (isa_off >= ISA_SER1_DMA_PAGE) ? 1 : 0;
        reg = (isa_off - (port ? ISA_SER1_DMA_PAGE : ISA_SER0_DMA_PAGE))
              & 0x38;
        switch (reg) {
        case ISA_DMA_TX_CTRL:
            s->isa_dma_ctrl[port][0] = value & 0x7ff;
            if (value & ISA_DMA_CTRL_RESET) {
                /* reset channel: pointers cleared, int output off */
                s->isa_dma_rptr[port][0] = 0;
                s->isa_dma_wptr[port][0] = 0;
                s->isa_dma_preq[port] = false;
                sgi_mace_isa_serial_update(s, port);
                return;
            }
            sgi_mace_isa_tx_drain(s, port);
            return;
        case ISA_DMA_TX_WPTR:
            if (s->isa_dma_ctrl[port][0] & ISA_DMA_CTRL_RESET) {
                return;
            }
            s->isa_dma_wptr[port][0] = value & 0xfff;
            sgi_mace_isa_tx_drain(s, port);
            return;
        case ISA_DMA_TX_RPTR:
        case ISA_DMA_TX_DEPTH:
            /* hardware-owned / read-only on an output channel */
            return;
        case ISA_DMA_RX_CTRL:
            s->isa_dma_ctrl[port][1] = value & 0x7ff;
            if (value & ISA_DMA_CTRL_RESET) {
                s->isa_dma_rptr[port][1] = 0;
                s->isa_dma_wptr[port][1] = 0;
            }
            if (port == 0) {
                sgi_mace_isa_rx_arm(s);
            }
            sgi_mace_isa_serial_update(s, port);
            return;
        case ISA_DMA_RX_RPTR:
            if (s->isa_dma_ctrl[port][1] & ISA_DMA_CTRL_RESET) {
                return;
            }
            s->isa_dma_rptr[port][1] = value & 0xfff;
            sgi_mace_isa_serial_update(s, port);
            return;
        case ISA_DMA_RX_WPTR:
        case ISA_DMA_RX_DEPTH:
            /* hardware-owned / read-only on an input channel */
            return;
        default:
            return;
        }
    }

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
        /*
         * Spec §5.1.3: all serial/DMA bits are read-only live levels
         * (only the ECP1284 device bit 16 is RW).  Keep the historical
         * write-to-clear for compatibility, then restore the live
         * serial levels so a stray write cannot drop a pending one.
         */
        s->isa_int_status &= ~value;
        sgi_mace_isa_serial_update(s, 0);
        sgi_mace_isa_serial_update(s, 1);
        break;
    case ISA_INT_MSK_REG:
    case ISA_INT_MSK_REG + 4:
        s->isa_int_mask = value;
        sgi_mace_isa_int_update(s);
        break;
    default:
        /* parallel DMA / DP-RAM: accept and ignore */
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
        return sgi_mace_pci_reg_read(s, offset - MACE_PCI_OFFSET, size);
    }

    /* ISA interface (0x310000-0x31FFFF) — includes serial DMA channels */
    if (offset >= MACE_ISA_OFFSET &&
        offset < MACE_ISA_OFFSET + 0x10000) {
        return sgi_mace_isa_read(s, offset - MACE_ISA_OFFSET, size);
    }

    /* Keyboard/mouse (0x320000-0x32FFFF) */
    if (offset >= MACE_KBDMS_OFFSET &&
        offset < MACE_KBDMS_OFFSET + 0x10000) {
        /* kbdinit() probes this region — return 0 for no device */
        return 0;
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

    /* RTC (0x3A0000-0x3A7FFF) — DS17287 direct-mapped, 256-byte stride */
    if (offset >= MACE_RTC_OFFSET &&
        offset < MACE_RTC_OFFSET + 0x8000) {
        hwaddr rtc_off = offset - MACE_RTC_OFFSET;
        /*
         * DS17287 RTC on IP32 is direct-mapped with 256-byte register stride.
         * Register N is at offset N*256 within the RTC region.
         * The PROM accesses at ISA_RTC_BASE+7 (BE byte lane within 8-byte
         * doubleword), so effective offset = N*256+7. We derive the register
         * index by dividing the full offset by 256.
         */
        int rtc_reg = rtc_off / 256;
        if (rtc_reg < 128) {
            uint8_t val = s->rtc_regs[rtc_reg];
            /* Register D bit 7 = VRT (Valid RAM and Time) = battery OK */
            if (rtc_reg == 13) {
                val |= 0x80;
            }
            /* Register C: reading clears interrupt flags */
            if (rtc_reg == 12) {
                s->rtc_regs[12] = 0;
            }
            MACE_DPRINTF("RTC read reg %d = 0x%02x\n", rtc_reg, val);
            return val;
        }
        return 0;
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
        sgi_mace_pci_reg_write(s, offset - MACE_PCI_OFFSET, value, size);
        return;
    }

    /* ISA interface — includes serial DMA channels */
    if (offset >= MACE_ISA_OFFSET &&
        offset < MACE_ISA_OFFSET + 0x10000) {
        sgi_mace_isa_write(s, offset - MACE_ISA_OFFSET, value, size);
        return;
    }

    /* Keyboard/mouse (0x320000-0x32FFFF) */
    if (offset >= MACE_KBDMS_OFFSET &&
        offset < MACE_KBDMS_OFFSET + 0x10000) {
        /* kbdinit() writes here — accept silently */
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

    /* RTC — DS17287 direct-mapped, 256-byte stride */
    if (offset >= MACE_RTC_OFFSET &&
        offset < MACE_RTC_OFFSET + 0x8000) {
        hwaddr rtc_off = offset - MACE_RTC_OFFSET;
        int rtc_reg = rtc_off / 256;
        if (rtc_reg < 128) {
            MACE_DPRINTF("RTC write reg %d = 0x%02x\n", rtc_reg,
                         (uint8_t)value);
            s->rtc_regs[rtc_reg] = value & 0xff;
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

    memset(s->isa_dma_ctrl, 0, sizeof(s->isa_dma_ctrl));
    memset(s->isa_dma_rptr, 0, sizeof(s->isa_dma_rptr));
    memset(s->isa_dma_wptr, 0, sizeof(s->isa_dma_wptr));
    memset(s->isa_dma_preq, 0, sizeof(s->isa_dma_preq));
    if (s->isa_rx_timer) {
        timer_del(s->isa_rx_timer);
    }

    s->pci_error_addr = 0;
    s->pci_error_flags = 0;
    s->pci_control = 0;
    s->pci_rev_info = 0;
    s->pci_config_addr = 0;
    memset(s->pci_int_level, 0, sizeof(s->pci_int_level));

    memset(s->rtc_regs, 0, sizeof(s->rtc_regs));

    /*
     * DS17287 RTC reset values:
     *   Reg A (10): 0x20 = oscillator running, divider chain on
     *   Reg B (11): 0x06 = binary mode (bit 2), 24-hour (bit 1)
     *   Reg D (13): 0x80 = VRT (Valid RAM and Time, battery OK)
     * Set a sensible time: 2026-02-14 12:00:00 Saturday (day=7)
     */
    s->rtc_regs[0]  = 0x00;    /* seconds */
    s->rtc_regs[2]  = 0x00;    /* minutes */
    s->rtc_regs[4]  = 0x12;    /* hours (12, binary mode) */
    s->rtc_regs[6]  = 0x07;    /* day of week (Saturday=7) */
    s->rtc_regs[7]  = 0x14;    /* date (14) */
    s->rtc_regs[8]  = 0x02;    /* month (February) */
    s->rtc_regs[9]  = 0x26;    /* year (26 = 2026 with century byte) */
    s->rtc_regs[10] = 0x20;    /* Register A: oscillator on */
    s->rtc_regs[11] = 0x06;    /* Register B: binary, 24h */
    s->rtc_regs[13] = 0x80;    /* Register D: VRT */

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
    MemoryRegion *sys_mem = get_system_memory();
    MemoryRegion *ram_alias;
    char *name;

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_mace_ops, s,
                          "sgi-mace", MACE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Output IRQs to CRIME (16 lines) */
    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", 16);

    /* RX DMA engine poll timer (console serial port) */
    s->isa_rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_mace_isa_rx_poll, s);

    /*
     * PCI root bus.  Devices live on PCI slots 1..5 (the kernel scans
     * exactly those); CONFIG_ADDRESS/CONFIG_DATA in the register file
     * above forward config cycles to this bus.  pci_mem is the flat
     * PCI memory space (device BARs are assigned by the guest at
     * 0x80000000+, pcimh INITIAL_MSPACE); pci_io is PCI I/O space.
     */
    memory_region_init(&s->pci_mem, OBJECT(dev), "mace-pci-mem", 4 * GiB);
    memory_region_init(&s->pci_io, OBJECT(dev), "mace-pci-io", 4 * GiB);
    s->pci_bus = pci_register_root_bus(dev, "mace-pci",
                                       sgi_mace_pci_set_irq,
                                       sgi_mace_pci_map_irq, s,
                                       &s->pci_mem, &s->pci_io,
                                       PCI_DEVFN(1, 0), 16, TYPE_PCI_BUS);
    PCI_HOST_BRIDGE(dev)->bus = s->pci_bus;

    /* CPU windows onto the PCI spaces (spec §9.3) */
    memory_region_init_alias(&s->pci_mem_alias, OBJECT(dev),
                             "mace-pci-low-mem", &s->pci_mem,
                             MACE_PCI_DEV_MEM_BASE, MACE_PCI_LOW_WINDOW_SIZE);
    memory_region_add_subregion(sys_mem, MACE_PCI_LOW_MEM_BASE,
                                &s->pci_mem_alias);
    memory_region_init_alias(&s->pci_io_alias, OBJECT(dev),
                             "mace-pci-low-io", &s->pci_io,
                             0, MACE_PCI_LOW_WINDOW_SIZE);
    memory_region_add_subregion(sys_mem, MACE_PCI_LOW_IO_BASE,
                                &s->pci_io_alias);

    /*
     * Host RAM as seen by PCI masters (spec §9.3.1): the native view
     * at 0x40000000 and the byte-swapped view at 0x0.  QEMU RAM
     * aliases cannot byte-swap, so both are plain aliases of system
     * memory and byte-order handling lives in the DMA devices (the
     * placeholder AIC-7880 interprets the view bit itself).  Alias
     * system memory rather than the RAM region so DMA reaches all of
     * low physical space exactly like a pass-through window.
     */
    ram_alias = g_new(MemoryRegion, 2);
    name = g_strdup_printf("mace-pci-ram-native");
    memory_region_init_alias(&ram_alias[0], OBJECT(dev), name, sys_mem,
                             0, MACE_PCI_HOST_RAM_SIZE);
    memory_region_add_subregion_overlap(&s->pci_mem, MACE_PCI_HOST_RAM_BASE,
                                        &ram_alias[0], 0);
    g_free(name);
    name = g_strdup_printf("mace-pci-ram-swapped");
    memory_region_init_alias(&ram_alias[1], OBJECT(dev), name, sys_mem,
                            0, MACE_PCI_HOST_RAM_SIZE);
    memory_region_add_subregion_overlap(&s->pci_mem, 0, &ram_alias[1], 0);
    g_free(name);

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
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(isa_ringbase, SGIMACEState),
        VMSTATE_UINT64(isa_flash_nic, SGIMACEState),
        VMSTATE_UINT64(isa_int_status, SGIMACEState),
        VMSTATE_UINT64(isa_int_mask, SGIMACEState),
        VMSTATE_UINT64_2DARRAY(isa_dma_ctrl, SGIMACEState, MACE_NUM_SERIAL, 2),
        VMSTATE_UINT32_2DARRAY(isa_dma_rptr, SGIMACEState, MACE_NUM_SERIAL, 2),
        VMSTATE_UINT32_2DARRAY(isa_dma_wptr, SGIMACEState, MACE_NUM_SERIAL, 2),
        VMSTATE_BOOL_ARRAY(isa_dma_preq, SGIMACEState, MACE_NUM_SERIAL),
        VMSTATE_UINT32(pci_error_addr, SGIMACEState),
        VMSTATE_UINT32(pci_error_flags, SGIMACEState),
        VMSTATE_UINT32(pci_control, SGIMACEState),
        VMSTATE_UINT32(pci_config_addr, SGIMACEState),
        VMSTATE_UINT32(pci_rev_info, SGIMACEState),
        VMSTATE_UINT8_ARRAY(pci_int_level, SGIMACEState, MACE_PCI_NUM_INTS),
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
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(SGIMACEState),
    .class_init = sgi_mace_class_init,
};

static void sgi_mace_register_types(void)
{
    type_register_static(&sgi_mace_info);
}

type_init(sgi_mace_register_types)
