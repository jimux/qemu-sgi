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
#include "trace.h"

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

/* Defined with the ISA serial DMA engine below. */
static void sgi_mace_isa_int_update(SGIMACEState *s);

/*
 * ============================================================
 *              PS/2 keyboard & mouse (spec §6)
 *
 * The MACE PS/2 interface is NOT an i8042: each port (keyboard at
 * MACE_KBDMS_OFFSET, mouse at +0x20) is a bare PS/2 serial-cell
 * transport with four 64-bit registers (tx_buf, rx_buf, control,
 * status) — no controller command byte, no output-port bits, no
 * muxed data stream.  Host->device bytes are written to tx_buf with
 * PS2_CTRL_TX_EN in control; device->host bytes are read from
 * rx_buf when status bit PS2_SR_RBF is set.  The PS/2 protocol
 * itself (command ACKs 0xFA, BAT 0xAA/0xFC, keyboard ID 0xAB 0x83,
 * mouse reset/sample-rate negotiation) belongs to the *device*
 * behind the port, so QEMU's PS/2 core (hw/input/ps2.c) implements
 * it; this model implements the transport registers and the ISA
 * interrupt bits.
 *
 * Driver contract (both probed against this model):
 *  - PROM mh_kbd.c: reset_kbd() polls CMD_DISABLE (0xF5) then
 *    programs scancode set 3 (CMD_SELSCAN 0xF0, 3) with make/break
 *    on caps/num lock; reset_pcms() polls CMD_DEFAULT (0xF6);
 *    pckm_setleds() sends CMD_SETLEDS (0xED).  pckm_sendok() waits
 *    for !(status & (TIP|RIP)) then TBE; pckm_pollcmd() reads
 *    rx_buf when RBF, treating KBD_RESEND 0xFE as retry and
 *    KBD_OVERRUN 0xFF as keep-waiting.
 *  - kernel io/mhpckm.c: probes both ports with CMD_DISABLE then
 *    CMD_ID (0xF2) — a keyboard answers ACK + 0xAB 0x83, a mouse
 *    ACK + 0x00 0x00 (ps2.c AUX_GET_TYPE queues mouse_type=0);
 *    init sets typematic/make-break via 0xFA/0xFC, then CMD_ENABLE
 *    0xF4; the mouse gets CMD_DEFAULT, CMD_MSRES 0xE8/0x03,
 *    CMD_ENABLE.  pckm_enable_interrupt() sets PS2_CMD_RxIEN on
 *    both ports and ISA_INT_MSK |= 0xA00.
 *  - Interrupts: rx-data available raises ISA_INT_STS bit 9
 *    (keyboard) / bit 11 (mouse), gated by control RX_IEN and the
 *    ISA mask, fanning into CRIME bit 5 (MACE_PERIPH_MISC,
 *    MACE_INTR(5)) — mhpckm.c setmaceisavector(MACE_INTR(5),
 *    PCKM_MACEMASK 0xA00, pckm_intr).
 *
 * Virtualization of the serial-cell timing: the real interface
 * shifts 11 bits on a device-supplied clock at ~10-16 kHz.  With
 * the transmit buffer always empty by the time the register write
 * retires (the PS/2 core consumes the command synchronously) the
 * drivers' "wait for TBE / !(TIP|RIP)" polling loops complete on
 * their first status read, and a device reply is already in the
 * queue when the driver first polls for RBF.  The clock signal
 * (status bit 0) reads asserted whenever the port is not held in
 * clock-inhibit (matching an idle device clock line).
 */
static PS2State *sgi_mace_ps2(SGIMACEState *s, int port)
{
    return port ? PS2_DEVICE(&s->ps2mouse) : PS2_DEVICE(&s->ps2kbd);
}

static void sgi_mace_ps2_int_update(SGIMACEState *s, int port)
{
    bool pending = s->ps2_port[port].irq_level &&
        (s->ps2_port[port].control & PS2_CTRL_RX_IEN);
    uint64_t bit = port ? ISA_INT_MOUSE : ISA_INT_KEYBOARD;

    if (pending) {
        s->isa_int_status |= bit;
    } else {
        s->isa_int_status &= ~bit;
    }
    trace_sgi_mace_ps2_irq(port, pending ? 1 : 0, s->isa_int_status);
    sgi_mace_isa_int_update(s);
}

/*
 * PS/2 core -> MACE glue: the core raises its output IRQ (GPIO
 * PS2_DEVICE_IRQ) when its queue goes non-empty, and lowers it once
 * the queue is drained by rx_buf reads.
 */
static void sgi_mace_ps2_irq(void *opaque, int n, int level)
{
    SGIMACEState *s = opaque;
    int port = n;

    s->ps2_port[port].irq_level = level;
    sgi_mace_ps2_int_update(s, port);
}

static uint8_t sgi_mace_ps2_status(SGIMACEState *s, int port)
{
    PS2State *ps2 = sgi_mace_ps2(s, port);
    uint8_t status = s->ps2_port[port].tx_pending ? 0 : PS2_STATUS_TBE;

    if (!ps2_queue_empty(ps2)) {
        status |= PS2_STATUS_RBF;
    }
    if (s->ps2_port[port].control & PS2_CTRL_CLKASS) {
        status |= PS2_STATUS_CLKINH;
    } else if (s->ps2_port[port].control & PS2_CTRL_CLKINH) {
        /* clock inhibited (paused): device clock reads de-asserted */
        status |= PS2_STATUS_CLKINH;
    } else {
        status |= PS2_STATUS_CLKSIG;
    }
    trace_sgi_mace_ps2_status(port, status);
    return status;
}

static uint64_t sgi_mace_ps2_read(SGIMACEState *s, int port, hwaddr reg_off)
{
    switch (reg_off) {
    case MACE_PS2_RX_BUF:
        /*
         * Spec TABLE 72: bits 15:8 alias the status register; bits
         * 7:0 are the received byte.  The drivers mask with 0xff, and
         * the PROM reads the status separately, so the alias is kept
         * faithful for any 16-bit-wide consumer.
         */
        {
            uint64_t status = sgi_mace_ps2_status(s, port);
            if (status & PS2_STATUS_RBF) {
                uint8_t data = ps2_read_data(sgi_mace_ps2(s, port));

                trace_sgi_mace_ps2_rx(port, data);
                return (status << 8) | data;
            }
            return status << 8;
        }
    case MACE_PS2_CONTROL:
        return s->ps2_port[port].control;
    case MACE_PS2_STATUS:
        return sgi_mace_ps2_status(s, port);
    default:
        /* tx_buf is write-only; reads return 0 per TABLE 70 */
        return 0;
    }
}

static void sgi_mace_ps2_tx_byte(SGIMACEState *s, int port, uint8_t data)
{
    trace_sgi_mace_ps2_tx(port, data);
    if (port == 0) {
        ps2_write_keyboard(&s->ps2kbd, data);
    } else {
        ps2_write_mouse(&s->ps2mouse, data);
    }
}

static void sgi_mace_ps2_write(SGIMACEState *s, int port, hwaddr reg_off,
                               uint64_t value)
{
    switch (reg_off) {
    case MACE_PS2_TX_BUF:
        /*
         * A byte written to the transmit buffer is LATCHED, not
         * shifted: the real interface starts shifting only when
         * TxEN is asserted in the control register.  Both drivers
         * write the data first with control cleared to 0 and then
         * set TxEN|CLKASS in a separate control write to launch the
         * transfer (PROM mh_kbd.c outb() steps 3-4; kernel mhpckm.c
         * outb() likewise).  Latch the byte here; the control
         * write below performs the shift.
         */
        s->ps2_port[port].tx_byte = value & 0xff;
        s->ps2_port[port].tx_pending = true;
        break;
    case MACE_PS2_CONTROL: {
        uint8_t old = s->ps2_port[port].control;
        uint8_t ctl = value & PS2_CTRL_IMPLEMENTED;

        /*
         * Transmission starts on the TxEN rising edge with a byte
         * latched (the control write after the tx_buf write — see
         * outb()).  The PS/2 core consumes the byte synchronously
         * (command/data state machine), so by the time the driver's
         * next pckm_sendok() polls for TBE the buffer is empty and
         * any reply is already queued.
         */
        if ((ctl & PS2_CTRL_TX_EN) && !(old & PS2_CTRL_TX_EN) &&
            s->ps2_port[port].tx_pending) {
            sgi_mace_ps2_tx_byte(s, port, s->ps2_port[port].tx_byte);
            s->ps2_port[port].tx_pending = false;
        }
        s->ps2_port[port].control = ctl;
        sgi_mace_ps2_int_update(s, port);
        break;
    }
    default:
        /* rx_buf/status are read-only */
        break;
    }
}

/*
 * Keyboard/mouse sub-address map (spec §12.1.1 TABLE 93): within the
 * 0x20000 peripheral-controller slice, PIO A[5] selects the port and
 * A[4:3] the register; the rest of each 32-byte port slice aliases.
 * The PROM/kernel drivers use exact 64-bit offsets 0x00/0x08/0x10/0x18.
 */
static uint64_t sgi_mace_kbdms_read(SGIMACEState *s, hwaddr km_off)
{
    int port = (km_off >> 5) & 1;
    hwaddr reg_off = km_off & 0x18;

    return sgi_mace_ps2_read(s, port, reg_off);
}

static void sgi_mace_kbdms_write(SGIMACEState *s, hwaddr km_off,
                                 uint64_t value)
{
    int port = (km_off >> 5) & 1;
    hwaddr reg_off = km_off & 0x18;

    sgi_mace_ps2_write(s, port, reg_off, value);
}

/*
 * ============================================================
 *              MAC110 fast ethernet (spec §4)
 *
 * The ethernet interface is the LSI Cascade-110 MAC core plus DMA
 * channels inside the MACE (spec §4, "Fast Ethernet Interface"), at
 * MACE_ENET_OFFSET (0x280000, kernel MACE_ETHER_ADDRESS 0xBF280000).
 * The IRIX driver is kern/bsd/mips/if_me.c ("ec0") + sys/if_me.h —
 * the driver contract:
 *
 *  - attach (if_meedtinit): resets the MAC, reads the revision from
 *    MAC_CONTROL bits 31:29 (must be 1: "first revision with improved
 *    transmit concatenation" — rev 0 forces the driver down the
 *    MACE1.0 100Mb-forced path), registers the CRIME vector
 *    MACE_INTR(MACE_ETHERNET) = 3, and parks a 128-entry TX ring in
 *    contig memory.
 *  - PHY probe (mace_ether_mdio_probe): for each of the 32 MDIO
 *    addresses, reads PHY regs 2/3 and accepts the identity
 *    (p2<<12)|(p3>>4) if it matches QS6612/ICS1889/ICS1890/DP83840.
 *    Then (rev!=0) writes MII_R0_AUTOEN and reads reg 1 once to
 *    discard latched status; the watchdog later polls reg 1
 *    (LINKSTAT must be 1) and reads 4/5 for ANLPAR.
 *  - hardware init (mace_hdwrether_init): reset+0 the MAC, set
 *    mode, program TX_RING_BASE, push 16 mcl cluster addresses
 *    into the RX FIFO, set the timer, then DMA_CONTROL =
 *    TX_DMA_EN | TX_16K | RX_DMA_EN | RX_INTR_EN | (off<<12) |
 *    (16<<4).  off = (sizeof(etherbufhead)-14)/8 = 4 dwords = 32
 *    bytes, so packets land at mcl+34 (status vector at +0, 2 bytes
 *    padding, ethernet header at +32+2).
 *  - TX (mace_ether_output): builds a 128-byte descriptor (cmd
 *    header + up to 3 concat pointers), then pokes the TX ring
 *    write pointer (16-bit write at 0x36).  Hardware advances the
 *    read pointer (0x34) as descriptors complete, writing the
 *    status vector into descriptor word 0 (bit 63 = finished).
 *  - RX (mace_ether_intr -> mace_ether_receive): reads the 64-bit
 *    dispatch register at 0x08 (must return {0, status} where the
 *    word packs isf[7:0] + rx-mcl-rptr[12:8] + tx-rptr[24:16] +
 *    rx-seqnum[29:25]); while non-zero, reclaims TX, processes
 *    received clusters (rxrptr from the dispatch register), and
 *    clears interrupt bits with 32-bit write-1-to-clear at 0x0C.
 *    A processed cluster holds {cksum, stats} at its first two
 *    words: stats[15:0] = total packet length, bit 63 of the first
 *    word = RX_VALID_PACKET.  The driver then refills the FIFO by
 *    writing a new cluster address to the FIFO data port (0x104+,
 *    32 aliases), each write followed by a "pioflush" (write+write
 *    +read of 0x304018).
 *
 * Virtualization: no wire timing, collisions, or IPG — the MDIO
 * register file is a static DP83840 image with link always up, the
 * TX DMA drains descriptors synchronously on write-pointer poke
 * (status vectors land before the driver next polls), and RX
 * packets are delivered as they arrive from the net backend into
 * the next queued mcl cluster, raising INTR_RX_DMA_REQ
 * (threshold-exceeded semantics: FIFO count != programmed
 * threshold) straight to CRIME bit 3.
 */

/* Re-evaluate the ethernet interrupt output to CRIME bit 3. */
static void sgi_mace_ec_irq_update(SGIMACEState *s)
{
    int level = !!(s->ec_int_status & 0xff) && !s->ec_force_off;

    qemu_set_irq(s->crime_irq[3], level);
    trace_sgi_mace_ec_irq(s->ec_int_status, level);
}

static void sgi_mace_ec_int_raise(SGIMACEState *s, uint32_t bits)
{
    s->ec_int_status |= bits & 0xff;
    sgi_mace_ec_irq_update(s);
}

/* Current TX ring geometry from the DMA control register. */
static int sgi_mace_ec_tx_entries(SGIMACEState *s)
{
    switch (s->ec_dma_control & DMA_CTRL_TX_RING_MASK) {
    case 0x0: return 8 * 1024 / MAC_TX_DESC_SIZE;    /* 8K ring:  64  */
    case 0x4: return 16 * 1024 / MAC_TX_DESC_SIZE;   /* 16K ring: 128 */
    case 0x8: return 32 * 1024 / MAC_TX_DESC_SIZE;   /* 32K ring: 256 */
    default:  return 64 * 1024 / MAC_TX_DESC_SIZE;    /* 64K ring: 512 */
    }
}

static inline uint32_t sgi_mace_ec_tx_rptr(SGIMACEState *s)
{
    return (s->ec_tx_ring >> MAC_TX_RPTR_SHIFT) & MAC_TX_PTR_MASK;
}

static inline uint32_t sgi_mace_ec_tx_wptr(SGIMACEState *s)
{
    return s->ec_tx_ring & MAC_TX_PTR_MASK;
}

static inline void sgi_mace_ec_set_tx_rptr(SGIMACEState *s, uint32_t rptr)
{
    s->ec_tx_ring = (s->ec_tx_ring & ~(MAC_TX_PTR_MASK << MAC_TX_RPTR_SHIFT))
                  | ((rptr & MAC_TX_PTR_MASK) << MAC_TX_RPTR_SHIFT);
}

static inline uint32_t sgi_mace_ec_rx_count(SGIMACEState *s)
{
    return (s->ec_rx_wptr - s->ec_rx_rptr) & MAC_RX_MCL_CNT_MASK;
}

/*
 * RX threshold condition (spec TABLE 40 bits 9,8:4): the interrupt
 * output follows !(Threshold != FIFO Count) — the RX interrupt is
 * asserted whenever the live FIFO count differs from the programmed
 * threshold.  The IRIX driver sets threshold=16 (fill the whole
 * FIFO), so any used cluster raises the interrupt until all are
 * replaced.
 */
static bool sgi_mace_ec_rx_threshold(SGIMACEState *s)
{
    unsigned thr = (s->ec_dma_control & DMA_CTRL_RX_THRESH_MASK)
                   >> DMA_CTRL_RX_THRESH_SHIFT;

    if (!(s->ec_dma_control & DMA_CTRL_RX_DMA_EN) ||
        !(s->ec_dma_control & DMA_CTRL_RX_INTR_EN)) {
        return false;
    }
    return sgi_mace_ec_rx_count(s) != thr;
}

/* Recompute the RX threshold interrupt after FIFO/enable changes. */
static void sgi_mace_ec_rx_int_update(SGIMACEState *s)
{
    if (sgi_mace_ec_rx_threshold(s)) {
        s->ec_int_status |= MAC_INTR_RX_DMA_REQ;
    } else {
        s->ec_int_status &= ~MAC_INTR_RX_DMA_REQ;
    }
    sgi_mace_ec_irq_update(s);
}

static ssize_t sgi_mace_ec_rx_deliver(SGIMACEState *s, const uint8_t *buf,
                                      size_t size);
static void sgi_mace_ec_rx_timer_cb(void *opaque);

/*
 * The physical station address register holds the 48-bit MAC in
 * bits 47:0 of the 64-bit slot (the driver bcopy's its eaddr[] into
 * bytes 2..7 of the union before the 64-bit write).  Extract it in
 * wire order (byte 0 = first on the wire) for the RX filter.
 */
static void sgi_mace_ec_get_mac(SGIMACEState *s, uint8_t mac[6])
{
    uint64_t v = s->ec_physaddr;
    int i;

    for (i = 0; i < 6; i++) {
        mac[i] = extract64(v, (5 - i) * 8, 8);
    }
}

/*
 * Ethernet CRC for the multicast hash (spec §4.3.3.1 — non-reflected
 * CRC-32, top 6 bits used as hash index) and the RX status vector's
 * IP checksum (§4.5.3 — 16-bit one's-complement wrap-around sum of
 * all packet bytes).
 */
static uint32_t sgi_mace_ec_laf_hash(const uint8_t *addr)
{
    uint32_t crc = 0xffffffff;
    int i, b;

    for (i = 0; i < 6; i++) {
        uint8_t byte = addr[i];
        for (b = 0; b < 8; b++) {
            uint32_t msb = crc >> 31;

            crc <<= 1;
            if (msb ^ (byte & 1)) {
                crc ^= 0x04c11db6;
                crc |= 1;
            }
            byte >>= 1;
        }
    }
    return crc >> 26;
}

static uint32_t sgi_mace_ec_ip_cksum(const uint8_t *buf, int len)
{
    uint32_t sum = 0;
    int i;

    for (i = 0; i + 1 < len; i += 2) {
        sum += (buf[i] << 8) | buf[i + 1];
    }
    if (len & 1) {
        sum += buf[len - 1] << 8;
    }
    sum = (sum & 0xffff) + (sum >> 16);
    return (~sum) & 0xffff;
}

/*
 * Net-backend receive: stash the frame and deliver it after a short
 * virtual-time wire delay (below).  Real wire latency means a reply
 * never lands in the same guest PIO window as the transmit that
 * provoked it; delivering synchronously (inside the guest's TX-ring
 * write, which is where the net backend replies run) drops the
 * follow-on interrupt work on the floor while the kernel network
 * thread still holds the socket locks it needs — observed as the
 * IRIX raw-socket layer discarding the reply between m_copy and
 * sbappendaddr.
 */
static ssize_t sgi_mace_ec_receive(NetClientState *nc, const uint8_t *buf,
                                   size_t size)
{
    SGIMACEState *s = qemu_get_nic_opaque(nc);

    if (size < 14 || size > MAC_MAX_FRAME) {
        return size;
    }
    if (s->ec_rx_pending_len >= 0) {
        /* one pending frame: overwrite (last arrival wins) */
    }
    memcpy(s->ec_rx_pending, buf, size);
    s->ec_rx_pending_len = size;
    timer_mod_ns(s->ec_rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                 + 200000);   /* 200us wire delay */
    return size;
}

static void sgi_mace_ec_rx_timer_cb(void *opaque)
{
    SGIMACEState *s = opaque;

    if (s->ec_rx_pending_len >= 0) {
        uint8_t pkt[MAC_MAX_FRAME];
        int len = s->ec_rx_pending_len;

        memcpy(pkt, s->ec_rx_pending, len);
        s->ec_rx_pending_len = -1;
        sgi_mace_ec_rx_deliver(s, pkt, len);
    }
}

/*
 * Pass one received packet from the net backend into the next queued
 * mcl cluster (see the driver-contract block comment above): 64-bit
 * status vector at cluster+0, the ethernet frame at cluster +
 * off*8 + 2 (off = the DMA-control RX starting offset).
 */
static ssize_t sgi_mace_ec_rx_deliver(SGIMACEState *s, const uint8_t *buf,
                                      size_t size)
{
    uint32_t off = (s->ec_dma_control & DMA_CTRL_RX_OFFSET_MASK)
                   >> DMA_CTRL_RX_OFFSET_SHIFT;
    uint32_t mcl, dest;
    uint8_t frame[MAC_MAX_FRAME + 8];
    uint64_t vector;
    uint32_t stats, cksum, hashbit, filter_mode;
    int frame_off, i, fill;
    bool is_broadcast = true, is_multicast = (buf[0] & 1) != 0;

    if (size < 14 || size > MAC_MAX_FRAME) {
        return size;
    }
    if (!(s->ec_dma_control & DMA_CTRL_RX_DMA_EN) ||
        (s->ec_mac_control & MAC_CTRL_RESET)) {
        return 0;
    }

    /* Destination-address filter (spec §4.3 + MAC_CONTROL bits 6:5) */
    filter_mode = (s->ec_mac_control >> 5) & 3;
    {
        uint8_t mac[6];
        bool accept = false;

        sgi_mace_ec_get_mac(s, mac);
        for (i = 0; i < 6; i++) {
            if (buf[i] != 0xff) {
                is_broadcast = false;
            }
        }
        /* Physical match: station address vs wire-order bytes */
        if (memcmp(buf, mac, 6) == 0) {
            accept = true;
        } else if (filter_mode >= 1 && is_broadcast) {
            accept = true;
        } else if (filter_mode == 1 && is_multicast) {
            hashbit = sgi_mace_ec_laf_hash(buf);
            if (s->ec_mlaf & (1ULL << (hashbit & 0x3f))) {
                accept = true;
            }
        } else if (filter_mode == 2 && is_multicast) {
            accept = true;
        }
        if (filter_mode != 3 && !accept) {
            trace_sgi_mace_ec_rx_drop(0);
            return size;
        }
    }

    if (sgi_mace_ec_rx_count(s) == 0) {
        /* No free cluster: FIFO underflow event, packet dropped. */
        sgi_mace_ec_int_raise(s, MAC_INTR_RX_UNDERFLOW);
        trace_sgi_mace_ec_rx_drop(1);
        return size;
    }

    /*
     * Build the packet image at its final mcl-relative offset:
     * 2 bytes of padding after the software header area (spec
     * §4.5.2), frame padded to the 60-byte wire minimum (the MAC
     * pads on TX, so a short frame arrives as 60 bytes + 4 FCS).
     * The status-vector length must report the padded length + 4
     * (FCS) — the IRIX driver flags anything shorter than 64
     * (ETHERMINLEN) as RX_VEC_BAD_PACKET and snoop swallows it.
     */
    frame_off = off * 8 + 2;
    memset(frame, 0, sizeof(frame));
    memcpy(frame + frame_off, buf, size);
    fill = 60 - (int)size;
    if (fill > 0) {
        memset(frame + frame_off + size, 0, fill);
        size += fill;
    }
    /* 4 trailing FCS bytes (zeros) for the length the vector claims */
    size += 4;

    stats = size & RX_VEC_LENGTH_MASK;
    if (is_multicast) {
        if (is_broadcast) {
            stats |= RX_VEC_BROADCAST;
        } else {
            stats |= RX_VEC_MULTICAST;
            if (s->ec_mlaf & (1ULL << (sgi_mace_ec_laf_hash(buf) & 0x3f))) {
                stats |= RX_VEC_MULTICAST_MATCH;
            }
        }
    }
    {
        uint8_t mac[6];

        sgi_mace_ec_get_mac(s, mac);
        if (memcmp(buf, mac, 6) == 0) {
            stats |= RX_VEC_PHYSICAL_MATCH;
        }
    }
    stats |= (s->ec_rx_seq << RX_VEC_SEQNUM_SHIFT);
    s->ec_rx_seq = (s->ec_rx_seq + 1) & 0x1f;
    cksum = sgi_mace_ec_ip_cksum(frame + frame_off, size);
    vector = ((uint64_t)cksum << RX_VEC_CKSUM_SHIFT) | stats
           | RX_VALID_PACKET;

    mcl = s->ec_rx_fifo[s->ec_rx_rptr & MAC_RX_MCL_IDX_MASK];
    dest = mcl & ~(MAC_RX_MCL_SIZE - 1);
    {
        uint8_t vecbuf[8];

        stq_be_p(vecbuf, vector);
        if (dma_memory_write(&address_space_memory, dest, vecbuf, 8,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
            dma_memory_write(&address_space_memory, dest + frame_off,
                             frame + frame_off, size,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            trace_sgi_mace_ec_rx_drop(2);
            return size;
        }
    }

    /* Pop the cluster: hardware RX FIFO read pointer advances. */
    s->ec_rx_rptr = (s->ec_rx_rptr + 1) & MAC_RX_MCL_CNT_MASK;
    trace_sgi_mace_ec_rx_packet(dest, size, 0, s->ec_rx_rptr);
    trace_sgi_mace_ec_frame(0, ldl_be_p(frame + frame_off),
                            ldl_be_p(frame + frame_off + 4),
                            ldl_be_p(frame + frame_off + 8),
                            ldl_be_p(frame + frame_off + 12),
                            vector, size);
    sgi_mace_ec_rx_int_update(s);
    return size;
}

static bool sgi_mace_ec_can_receive(NetClientState *nc)
{
    SGIMACEState *s = qemu_get_nic_opaque(nc);

    return s->nic_present && (s->ec_dma_control & DMA_CTRL_RX_DMA_EN)
           && !(s->ec_mac_control & MAC_CTRL_RESET);
}

static NetClientInfo net_sgi_mace_ec_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_mace_ec_can_receive,
    .receive = sgi_mace_ec_receive,
};

/*
 * TX DMA engine: drain descriptors from the TX ring (base +
 * rptr*128) up to the software write pointer.  Each 128-byte
 * descriptor: word 0 = cmd header (see the block comment), words
 * 1..3 = optional concat pointers (address [31:3], len-1 [47:32]),
 * bytes from the ring data area hold the packet TAIL back-filled so
 * it ends at offset 128-start on an 8-byte boundary.  Hardware
 * overwrites word 0 with the status vector (bit 63 set) once the
 * packet has been sent.
 */
static void sgi_mace_ec_tx_drain(SGIMACEState *s)
{
    int entries = sgi_mace_ec_tx_entries(s);
    hwaddr base = s->ec_tx_ring_base & ~0x1fffULL;
    uint8_t pkt[MAC_TX_DESC_SIZE + MAC_MAX_FRAME];
    uint32_t ring_mask = entries - 1;
    uint32_t wptr, rptr;
    NetClientState *nc;

    if (!(s->ec_dma_control & DMA_CTRL_TX_DMA_EN) ||
        (s->ec_mac_control & MAC_CTRL_RESET)) {
        return;
    }
    wptr = sgi_mace_ec_tx_wptr(s) & ring_mask;
    rptr = sgi_mace_ec_tx_rptr(s) & ring_mask;

    while (rptr != wptr) {
        uint8_t desc[MAC_TX_DESC_SIZE];
        hwaddr a = base + (hwaddr)rptr * MAC_TX_DESC_SIZE;
        uint64_t cmd, cptr, vector;
        uint32_t tlen, doff, cats, clen, status;
        int plen = 0, local, c;

        if (dma_memory_read(&address_space_memory, a, desc,
                            sizeof(desc), MEMTXATTRS_UNSPECIFIED)
                != MEMTX_OK) {
            return;
        }
        cmd = ldq_be_p(desc);
        tlen = (cmd & TX_CMD_LENGTH_MASK) + 1;         /* packet length */
        doff = (cmd & TX_CMD_OFFSET_MASK) >> TX_CMD_OFFSET_SHIFT;
        cats = (cmd >> TX_CMD_CONCAT_SHIFT) & 7;

        trace_sgi_mace_ec_tx_desc(rptr, cmd, tlen);

        /*
         * Dead / consumed descriptors (the driver marks reclaimed
         * slots with DEADPACKET 0x7E0001, and boot-time ring memory
         * reads back zero) carry no command.  Real hardware never
         * fetches them (the ring between the hardware pointers only
         * holds queued packets); skip rather than send garbage if a
         * resync race ever leaves one in range.
         */
        if (cmd == 0x7e0001ULL || (cmd & ~TX_CMD_OFFSET_MASK) == 0) {
            rptr = (rptr + 1) & ring_mask;
            continue;
        }

        /* Concatenation buffers first (header part of the packet) */
        for (c = 0; c < 3 && c < (int)cats; c++) {
            cptr = ldq_be_p(desc + 8 * (c + 1));
            clen = ((cptr >> 32) & 0xffff) + 1;
            if (plen + (int)clen > MAC_MAX_FRAME) {
                clen = MAC_MAX_FRAME - plen;
            }
            trace_sgi_mace_ec_tx_concat(c, (uint32_t)(cptr & ~7ULL), clen);
            if (dma_memory_read(&address_space_memory, cptr & ~7ULL,
                                pkt + plen, clen,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                return;
            }
            plen += clen;
        }
        /* Then the ring data area (the packet's tail bytes) */
        if (doff >= 8 && doff <= 120) {
            local = MAC_TX_DESC_SIZE - doff;
            if (plen + local > MAC_MAX_FRAME) {
                local = MAC_MAX_FRAME - plen;
            }
            memcpy(pkt + plen, desc + doff, local);
            plen += local;
        }

        if (s->nic_present) {
            nc = qemu_get_queue(s->nic);
            /*
             * Pad short frames to the 64-byte ethernet minimum
             * (MAC110 "automatic transmit padding", spec §4): the
             * packet on the wire is 60 bytes + 4 FCS.
             */
            if (plen < 60) {
                memset(pkt + plen, 0, 60 - plen);
                plen = 60;
            }
            qemu_send_packet(nc, pkt, plen);
        }
        trace_sgi_mace_ec_tx_packet(plen);
        trace_sgi_mace_ec_frame(1, ldl_be_p(pkt), ldl_be_p(pkt + 4),
                                ldl_be_p(pkt + 8), ldl_be_p(pkt + 12),
                                cmd, plen);

        /* Status vector overwrites the command header (spec §4.4.4) */
        status = TX_VEC_COMPLETED | (plen & 0x7fff);
        vector = TX_VEC_FINISHED | status;
        s->ec_last_tx_vector = vector;
        stq_be_p(desc, vector);
        if (dma_memory_write(&address_space_memory, a, desc, 8,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return;
        }

        if (cmd & TX_CMD_SENT_INT_EN) {
            sgi_mace_ec_int_raise(s, MAC_INTR_TX_PKT_REQ);
        }
        rptr = (rptr + 1) & ring_mask;
    }

    sgi_mace_ec_set_tx_rptr(s, rptr);

    /*
     * "TX ring empty" interrupt (DMA_CONTROL bit 0): raised whenever
     * the drained ring is empty and the TX interrupt enable is on.
     */
    if ((s->ec_dma_control & DMA_CTRL_TX_INTR_EN) && rptr == wptr) {
        s->ec_int_status |= MAC_INTR_TX_DMA_REQ;
    }
    sgi_mace_ec_irq_update(s);
}

/* Build the 64-bit interrupt dispatch value (the isr.lsr read). */
static uint64_t sgi_mace_ec_dispatch(SGIMACEState *s)
{
    uint64_t v = 0;

    /*
     * While the 0x58 force-off is parked (driver reset window),
     * report no pending interrupt flags — the pointer fields stay
     * live.  A spinning ISR ithread polls this register; without
     * the mask it would never observe the force-off.
     */
    v |= s->ec_force_off ? 0 : (s->ec_int_status & 0xff);
    v |= ((uint64_t)(s->ec_rx_rptr & 0x1f) << 8);
    v |= ((uint64_t)(sgi_mace_ec_tx_rptr(s) & 0x1ff) << 16);
    v |= ((uint64_t)s->ec_rx_seq << 25);
    return v;
}

/* Reset the PHY register file to the DP83840 link-up image. */
static void sgi_mace_ec_phy_reset(SGIMACEState *s)
{
    memset(s->ec_phy_reg, 0, sizeof(s->ec_phy_reg));

    /* reg 0 control: all zero (auto-neg enabled after reset) */
    s->ec_phy_reg[1] = 0x7c2d;  /* 100T4/100TX-FD/HD/10T-FD/HD able,
                                 * auto-neg done, link up, AN able,
                                 * extended caps */
    s->ec_phy_reg[2] = MAC_PHY_REG2_OUI;
    s->ec_phy_reg[3] = MAC_PHY_REG3_ID;
    s->ec_phy_reg[4] = 0x01e1;  /* advertise 100TX-FD/HD + 10T-FD/HD,
                                 * selector 802.3 */
    s->ec_phy_reg[5] = 0x01e1;  /* partner: same (ideal link) */
    s->ec_phy_reg[6] = 0x0005;  /* NP able, page received, partner NWable */
}

/*
 * Ethernet register window read (offset relative to MACE_ENET_OFFSET,
 * spec TABLE 37).  64-bit slots; the 32-bit registers live in the
 * high lane (slot+4) per the drivers' struct mac110 layout.
 */
static uint64_t sgi_mace_ec_read(SGIMACEState *s, hwaddr ec_off,
                                 unsigned size)
{
    switch (ec_off) {
    case MAC_REG_MAC_CONTROL:
        /* implementation revision [31:29] = 1: "first revision with
         * improved transmit concatenation support" — the driver keys
         * its PHY autonegotiation path off this being non-zero. */
        return s->ec_mac_control | MAC_CTRL_REV1;
    case MAC_REG_INT_STATUS:
        /* 32-bit alias of the low interrupt-status word */
        return s->ec_int_status;
    case 0x08:
        /* 64-bit interrupt dispatch (isr.lsr): {0, packed status} */
        return sgi_mace_ec_dispatch(s);
    case MAC_REG_DMA_CONTROL:
        return s->ec_dma_control;
    case MAC_REG_TIMER:
        return s->ec_timer;
    case MAC_REG_TX_RING:
    case MAC_REG_TX_RING + 8:
        return s->ec_tx_ring;
    case MAC_REG_RX_FIFO_INFO:
    case MAC_REG_RX_FIFO_INFO + 8:
    case MAC_REG_RX_FIFO_INFO + 16:
        /*
         * mcl FIFO write/read pointers + depth (TABLE 45).  The
         * pointer fields carry the 4-bit index in [19:16]/[11:8]
         * with a generation bit at [20]/[12]; the driver reads only
         * the depth (bits [4:0]) via the dispatch register's rptr
         * alias, so the generation bits are kept simple.
         */
        return (((s->ec_rx_wptr & 0x1f) >> 4) << RXFIFO_GEN2_SHIFT)
             | ((s->ec_rx_wptr & 0xf) << RXFIFO_WPTR_SHIFT)
             | (((s->ec_rx_rptr & 0x1f) >> 4) << RXFIFO_GEN1_SHIFT)
             | ((s->ec_rx_rptr & 0xf) << RXFIFO_RPTR_SHIFT)
             | (sgi_mace_ec_rx_count(s) << RXFIFO_DEPTH_SHIFT);
    /*
     * Byte/16-bit reads of the rx_info bytes (sys/if_me.h struct
     * mac110: rx_info.u.bd at slot 0x44 — wptr byte 0x45, rptr
     * byte 0x46, depth byte 0x47).  mace_ether_watchdog reads the
     * FIFO rptr with an 8-bit access at 0x46.
     */
    case 0x45:
        return s->ec_rx_wptr & 0xf;
    case 0x46:
        return s->ec_rx_rptr & 0x1f;
    case 0x47:
        return sgi_mace_ec_rx_count(s);
    case MAC_REG_LAST_TX_VECTOR:
        /* 64-bit read of the diagnostic last-TX-vector slot */
        return s->ec_last_tx_vector;
    case MAC_REG_PHY_DATAIO:
        /* busy always clear (transfers complete instantly); data
         * holds the last-read/written register of the addressed PHY
         * — only MDIO device 1 carries a real register file. */
        return ((s->ec_phy_addr >> 5) == MAC_PHY_ADDR)
             ? s->ec_phy_reg[s->ec_phy_addr & 0x1f] : 0;
    case MAC_REG_PHY_ADDRESS:
        return s->ec_phy_addr;
    /*
     * The 64-bit DP-RAM registers (spec TABLE 37): 32-bit accesses
     * at +0 address the HIGH half (BE lane), +4 the low half.
     */
    case MAC_REG_PHYSADDR:
    case MAC_REG_PHYSADDR + 4:
        if (size == 8) {
            return s->ec_physaddr;
        }
        return (ec_off & 4) ? (s->ec_physaddr & 0xffffffffULL)
                            : (s->ec_physaddr >> 32);
    case MAC_REG_SECPHYSADDR:
    case MAC_REG_SECPHYSADDR + 4:
        if (size == 8) {
            return s->ec_secphysaddr;
        }
        return (ec_off & 4) ? (s->ec_secphysaddr & 0xffffffffULL)
                            : (s->ec_secphysaddr >> 32);
    case MAC_REG_MLAF:
    case MAC_REG_MLAF + 4:
        if (size == 8) {
            return s->ec_mlaf;
        }
        return (ec_off & 4) ? (s->ec_mlaf & 0xffffffffULL)
                            : (s->ec_mlaf >> 32);
    case MAC_REG_TX_RING_BASE:
    case MAC_REG_TX_RING_BASE + 4:
        if (size == 8) {
            return s->ec_tx_ring_base;
        }
        return (ec_off & 4) ? (s->ec_tx_ring_base & 0xffffffffULL)
                            : (s->ec_tx_ring_base >> 32);
    default:
        /* diagnostic TX descriptors / msgqueue: read as 0 */
        return 0;
    }
}

static void sgi_mace_ec_write(SGIMACEState *s, hwaddr ec_off,
                              uint64_t value, unsigned size)
{
    trace_sgi_mace_ec_reg(1, ec_off, (uint32_t)value);

    switch (ec_off) {
    case MAC_REG_MAC_CONTROL:
        if (value & MAC_CTRL_RESET) {
            /*
             * Core reset (spec TABLE 38 bit 0, "Global reset signal
             * to MAC110 core is active"): resets the DMA engines.
             * The IRIX driver's reset/init path relies on this —
             * mace_hdwrether_init restarts its TX ring at 0 and
             * pushes 16 fresh RX clusters without any pointer
             * sync, which only works if the reset cleared the
             * hardware TX ring pointers and the RX mcl FIFO.
             */
            s->ec_mac_control = MAC_CTRL_RESET;
            s->ec_int_status = 0;
            s->ec_tx_ring = 0;
            s->ec_rx_wptr = 0;
            s->ec_rx_rptr = 0;
            sgi_mace_ec_irq_update(s);
            return;
        }
        s->ec_mac_control = value & 0x1fffffff;
        return;
    case MAC_REG_INT_STATUS:
        /* write-1-to-clear (spec TABLE 39 note); the live RX
         * threshold level must survive the clear. */
        s->ec_int_status &= ~(value & MAC_INTR_W1C_MASK);
        sgi_mace_ec_rx_int_update(s);
        return;
    case MAC_REG_DMA_CONTROL:
        s->ec_dma_control = value & 0xffff;
        s->ec_force_off = false;    /* re-init: interrupts re-enabled */
        sgi_mace_ec_rx_int_update(s);
        sgi_mace_ec_tx_drain(s);
        return;
    case MAC_REG_TIMER:
        s->ec_timer = value & 0x3f;
        return;
    case MAC_REG_TX_ALIAS:
        /* WO alias of the DMA_CONTROL TX interrupt enable (§4.2.5) */
        s->ec_dma_control = (s->ec_dma_control & ~DMA_CTRL_TX_INTR_EN)
                          | (value & DMA_CTRL_TX_INTR_EN);
        return;
    case MAC_REG_RX_ALIAS:
        /* WO alias of the DMA_CONTROL RX enable + threshold (§4.2.6) */
        s->ec_dma_control = (s->ec_dma_control
                             & ~(DMA_CTRL_RX_INTR_EN
                                 | DMA_CTRL_RX_THRESH_MASK))
                          | (value & (DMA_CTRL_RX_INTR_EN
                                      | DMA_CTRL_RX_THRESH_MASK));
        sgi_mace_ec_rx_int_update(s);
        return;
    case MAC_REG_TX_RING:
    case MAC_REG_TX_RING + 2:
        /* 16/32-bit write pointer poke (driver: write16 at +2) */
        s->ec_tx_ring = (s->ec_tx_ring
                         & ~(MAC_TX_PTR_MASK << MAC_TX_WPTR_SHIFT))
                      | ((value & MAC_TX_PTR_MASK) << MAC_TX_WPTR_SHIFT);
        sgi_mace_ec_tx_drain(s);
        return;
    case MAC_REG_TX_RING + 8:
        s->ec_tx_ring = (uint32_t)value;
        sgi_mace_ec_tx_drain(s);
        return;
    case MAC_REG_INT_REQUEST:
        /*
         * Interrupt Request / diag register aliased with the last
         * TX vector (spec TABLE 37, offset 0x58).  The IRIX reset
         * path writes 1<<(MACE_ETHERNET+16) here to force the CRIME
         * line off while it rebuilds the rings — without this, the
         * core reset emptying the mcl FIFO instantly re-asserts the
         * RX threshold interrupt and the ISR deadlocks the init
         * thread.  The force-off releases when the re-init programs
         * DMA_CONTROL (interrupts re-enabled).
         */
        s->ec_force_off = true;
        sgi_mace_ec_irq_update(s);
        return;
    case MAC_REG_PHY_DATAIO:
        /* PHY register write (spec §4.2.9.2) */
        if ((s->ec_phy_addr >> 5) == MAC_PHY_ADDR) {
            int reg = s->ec_phy_addr & 0x1f;

            if (reg == 0) {
                s->ec_phy_reg[0] = value & 0x7f;
            } else if (reg == 4) {
                s->ec_phy_reg[4] = value & 0x01ff;
            }
        }
        trace_sgi_mace_ec_mdio(1, s->ec_phy_addr, (uint32_t)value);
        return;
    case MAC_REG_PHY_ADDRESS:
        s->ec_phy_addr = value & 0x3ff;
        return;
    case MAC_REG_PHY_READ_START:
        /* read completes instantly: data already in PHY_DATAIO */
        trace_sgi_mace_ec_mdio(0, s->ec_phy_addr,
                               (s->ec_phy_addr >> 5) == MAC_PHY_ADDR
                               ? s->ec_phy_reg[s->ec_phy_addr & 0x1f] : 0);
        return;
    case MAC_REG_BACKOFF:
        /* backoff LFSR seed: no collisions to randomize */
        return;
    /*
     * The 64-bit DP-RAM registers (spec TABLE 37): a 64-bit write
     * stores the whole value; a 32-bit write at +0 sets the HIGH
     * half (BE lane), at +4 the low half.  (The driver programs
     * the station address with one 64-bit write of eau.laddr —
     * the eaddr bytes live in bits 47:0.)
     */
    case MAC_REG_PHYSADDR:
    case MAC_REG_PHYSADDR + 4:
        if (size == 8) {
            s->ec_physaddr = value;
        } else if (ec_off & 4) {
            s->ec_physaddr = (s->ec_physaddr & ~0xffffffffULL)
                           | (value & 0xffffffff);
        } else {
            s->ec_physaddr = (s->ec_physaddr & 0xffffffffULL)
                           | (value << 32);
        }
        return;
    case MAC_REG_SECPHYSADDR:
    case MAC_REG_SECPHYSADDR + 4:
        if (size == 8) {
            s->ec_secphysaddr = value;
        } else if (ec_off & 4) {
            s->ec_secphysaddr = (s->ec_secphysaddr & ~0xffffffffULL)
                              | (value & 0xffffffff);
        } else {
            s->ec_secphysaddr = (s->ec_secphysaddr & 0xffffffffULL)
                              | (value << 32);
        }
        return;
    case MAC_REG_MLAF:
    case MAC_REG_MLAF + 4:
        if (size == 8) {
            s->ec_mlaf = value;
        } else if (ec_off & 4) {
            s->ec_mlaf = (s->ec_mlaf & ~0xffffffffULL)
                       | (value & 0xffffffff);
        } else {
            s->ec_mlaf = (s->ec_mlaf & 0xffffffffULL)
                       | (value << 32);
        }
        return;
    case MAC_REG_TX_RING_BASE:
    case MAC_REG_TX_RING_BASE + 4:
        if (size == 8) {
            s->ec_tx_ring_base = value;
        } else if (ec_off & 4) {
            s->ec_tx_ring_base = (s->ec_tx_ring_base & ~0xffffffffULL)
                               | (value & 0xffffffff);
        } else {
            s->ec_tx_ring_base = (s->ec_tx_ring_base & 0xffffffffULL)
                               | (value << 32);
        }
        sgi_mace_ec_tx_drain(s);
        return;
    default:
        /*
         * RX mcl FIFO data port: 32 aliases at 0x100-0x1F8 (spec
         * §4.2.8): push a 4KB cluster base address.  Spec §4.2.8:
         * "hardware does not prevent the system software from
         * overrunning ... the FIFO from the PIO access port" —
         * pushes into a full FIFO discard the OLDEST entries (the
         * newest 16 win).  The IRIX error-recovery path
         * (rt_not_valid / deadman) runs mace_ether_reset +
         * mace_hdwrether_init, which frees the old clusters and
         * pushes 16 fresh ones without any hardware FIFO reset;
         * discard-oldest makes the driver's view and the FIFO
         * self-consistent again after exactly 16 pushes.
         */
        if (ec_off >= MAC_REG_RX_FIFO_ALIAS_S
            && ec_off <= MAC_REG_RX_FIFO_ALIAS_E && (ec_off & 4)) {
            if (sgi_mace_ec_rx_count(s) >= MAC_RX_MCL_ENTRIES) {
                s->ec_rx_rptr = (s->ec_rx_rptr + 1) & MAC_RX_MCL_CNT_MASK;
            }
            {
                int slot = s->ec_rx_wptr & MAC_RX_MCL_IDX_MASK;

                s->ec_rx_fifo[slot] = value & 0xfffff000;
                s->ec_rx_wptr = (s->ec_rx_wptr + 1) & MAC_RX_MCL_CNT_MASK;
                trace_sgi_mace_ec_rx_fifo_push((uint32_t)value,
                                               sgi_mace_ec_rx_count(s),
                                               s->ec_rx_wptr & 15,
                                               s->ec_rx_rptr & 15);
                sgi_mace_ec_rx_int_update(s);
            }
            return;
        }
        /* diagnostic TX descriptor slots: read-only */
        return;
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

    /* Ethernet (0x280000-0x2FFFFF, spec §4) */
    if (offset >= MACE_ENET_OFFSET &&
        offset < MACE_ENET_OFFSET + 0x80000) {
        uint64_t v = sgi_mace_ec_read(s, offset - MACE_ENET_OFFSET, size);
        trace_sgi_mace_ec_reg(0, offset - MACE_ENET_OFFSET, (uint32_t)v);
        return v;
    }

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
        return sgi_mace_kbdms_read(s, offset - MACE_KBDMS_OFFSET);
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

    /* Ethernet (0x280000-0x2FFFFF, spec §4) */
    if (offset >= MACE_ENET_OFFSET &&
        offset < MACE_ENET_OFFSET + 0x80000) {
        sgi_mace_ec_write(s, offset - MACE_ENET_OFFSET, value, size);
        return;
    }

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
        sgi_mace_kbdms_write(s, offset - MACE_KBDMS_OFFSET, value);
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

    /*
     * PS/2 transport reset (spec TABLE 73/74 reset values: control
     * all-zero except nothing, status Clken asserted).  The PS/2
     * core devices reset themselves through the resettable
     * machinery (BAT state, scancode set 2, queue flush).
     */
    for (i = 0; i < 2; i++) {
        s->ps2_port[i].control = 0;
        s->ps2_port[i].tx_byte = 0;
        s->ps2_port[i].tx_pending = false;
        s->ps2_port[i].irq_level = 0;
    }

    s->pci_error_addr = 0;
    s->pci_error_flags = 0;
    s->pci_control = 0;
    s->pci_rev_info = 0;
    s->pci_config_addr = 0;
    memset(s->pci_int_level, 0, sizeof(s->pci_int_level));

    memset(s->rtc_regs, 0, sizeof(s->rtc_regs));

    /* MAC110 ethernet (spec §4): all state zeros at reset; the
     * station address register is pre-loaded with the ARCS env
     * MAC (kernel init_sysid writes its own copy anyway) and the
     * PHY file with the DP83840 link-up image. */
    memset(s->ec_rx_fifo, 0, sizeof(s->ec_rx_fifo));
    s->ec_mac_control = 0;
    s->ec_int_status = 0;
    s->ec_dma_control = 0;
    s->ec_timer = 0;
    s->ec_tx_ring = 0;
    s->ec_tx_ring_base = 0;
    s->ec_rx_wptr = 0;
    s->ec_rx_rptr = 0;
    s->ec_rx_seq = 0;
    /*
     * Default station address 08:00:69:de:ad:01 in the register's
     * big-endian wire order (the same byte order the driver
     * bcopy's in from kernel eaddr[]).  Overwritten below from the
     * NIC backend's MAC if one was instantiated.
     */
    {
        const uint8_t defmac[6] = { 0x08, 0x00, 0x69, 0xde, 0xad, 0x01 };
        memcpy(&s->ec_physaddr, defmac, 6);
    }
    s->ec_secphysaddr = 0;
    s->ec_mlaf = 0;
    s->ec_last_tx_vector = 0;
    s->ec_phy_addr = 0;
    s->ec_phy_busy = false;
    s->ec_force_off = false;
    s->ec_rx_pending_len = -1;
    if (s->ec_rx_timer) {
        timer_del(s->ec_rx_timer);
    }
    sgi_mace_ec_phy_reset(s);
    if (s->nic_present) {
        memcpy(&s->ec_physaddr, s->nic_conf.macaddr.a, 6);
    }

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

    /*
     * PS/2 core devices (hw/input/ps2.c), embedded as children (the
     * sgi_hpc3.c pattern): the keyboard registers the KEY input
     * handler (host keys -> make/break scancodes), the mouse the
     * BTN+REL handler (host pointer -> 3-byte packets).  The MACE
     * port index rides along on the GPIO line index so one handler
     * serves both ports.
     */
    qdev_init_gpio_in(dev, sgi_mace_ps2_irq, 2);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2kbd), errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->ps2kbd), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in(dev, 0));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2mouse), errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->ps2mouse), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in(dev, 1));

    /* RX DMA engine poll timer (console serial port) */
    s->isa_rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_mace_isa_rx_poll, s);

    /*
     * MAC110 ethernet NIC: always present on the O2 motherboard.
     * The machine file claims the default -nic/-netdev backend
     * (qemu_configure_nic_device) before realize; if none was
     * given (e.g. -nodefaults tests) we still model the register
     * file so the driver attaches, with no packet transport.
     */
    if (s->nic_conf.peers.ncs[0]) {
        s->nic_present = true;
        s->nic = qemu_new_nic(&net_sgi_mace_ec_info, &s->nic_conf,
                              object_get_typename(OBJECT(dev)), dev->id,
                              &dev->mem_reentrancy_guard, s);
        qemu_format_nic_info_str(qemu_get_queue(s->nic),
                                 s->nic_conf.macaddr.a);
        memcpy(&s->ec_physaddr, s->nic_conf.macaddr.a, 6);
    } else {
        s->nic_present = false;
        s->nic = NULL;
    }
    s->ec_rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_mace_ec_rx_timer_cb, s);
    s->ec_rx_pending_len = -1;

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
    DEFINE_NIC_PROPERTIES(SGIMACEState, nic_conf),
};

static bool sgi_mace_ps2_needed(void *opaque)
{
    return true;
}

static const VMStateDescription vmstate_sgi_mace_ps2 = {
    .name = "sgi-mace/ps2",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = sgi_mace_ps2_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(control, MACEPS2PortState),
        VMSTATE_UINT8(tx_byte, MACEPS2PortState),
        VMSTATE_BOOL(tx_pending, MACEPS2PortState),
        VMSTATE_END_OF_LIST()
    },
};

static bool sgi_mace_ec_needed(void *opaque)
{
    SGIMACEState *s = opaque;

    return s->nic_present;
}

static const VMStateDescription vmstate_sgi_mace_ec = {
    .name = "sgi-mace/ec",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = sgi_mace_ec_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ec_mac_control, SGIMACEState),
        VMSTATE_BOOL(ec_force_off, SGIMACEState),
        VMSTATE_UINT32(ec_int_status, SGIMACEState),
        VMSTATE_UINT32(ec_dma_control, SGIMACEState),
        VMSTATE_UINT32(ec_timer, SGIMACEState),
        VMSTATE_UINT32(ec_tx_ring, SGIMACEState),
        VMSTATE_UINT64(ec_tx_ring_base, SGIMACEState),
        VMSTATE_UINT32_ARRAY(ec_rx_fifo, SGIMACEState, MAC_RX_MCL_ENTRIES),
        VMSTATE_UINT32(ec_rx_wptr, SGIMACEState),
        VMSTATE_UINT32(ec_rx_rptr, SGIMACEState),
        VMSTATE_UINT32(ec_rx_seq, SGIMACEState),
        VMSTATE_UINT16_ARRAY(ec_phy_reg, SGIMACEState, 32),
        VMSTATE_UINT32(ec_phy_addr, SGIMACEState),
        VMSTATE_UINT64(ec_physaddr, SGIMACEState),
        VMSTATE_UINT64(ec_secphysaddr, SGIMACEState),
        VMSTATE_UINT64(ec_mlaf, SGIMACEState),
        VMSTATE_UINT64(ec_last_tx_vector, SGIMACEState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_sgi_mace = {
    .name = "sgi-mace",
    .version_id = 4,
    .minimum_version_id = 4,
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
        VMSTATE_STRUCT_ARRAY(ps2_port, SGIMACEState, 2, 1,
                             vmstate_sgi_mace_ps2, MACEPS2PortState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_sgi_mace_ec,
        NULL
    }
};

static void sgi_mace_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_mace_realize;
    device_class_set_legacy_reset(dc, sgi_mace_reset);
    dc->vmsd = &vmstate_sgi_mace;
    device_class_set_props(dc, sgi_mace_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void sgi_mace_init(Object *obj)
{
    SGIMACEState *s = SGI_MACE(obj);

    object_initialize_child(obj, "ps2kbd", &s->ps2kbd,
                            TYPE_PS2_KBD_DEVICE);
    object_initialize_child(obj, "ps2mouse", &s->ps2mouse,
                            TYPE_PS2_MOUSE_DEVICE);
}

static const TypeInfo sgi_mace_info = {
    .name = TYPE_SGI_MACE,
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(SGIMACEState),
    .instance_init = sgi_mace_init,
    .class_init = sgi_mace_class_init,
};

static void sgi_mace_register_types(void)
{
    type_register_static(&sgi_mace_info);
}

type_init(sgi_mace_register_types)
