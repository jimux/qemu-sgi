/*
 * SGI BRIDGE (PCI/XTalk Bridge) emulation
 *
 * BRIDGE is the peripheral controller in the SGI Octane (IP30). It handles:
 * - PCI configuration space access
 * - XTLink (XTalk) interface for graphics
 * - BaseIO slot control
 * - Power button and ACFail detection
 *
 * Physical base: 0x1F000000
 *
 * The IOC3 chip sits behind BRIDGE at the devio window (BRIDGE+0x600000 =
 * physical 0x1F600000). The IOC3 has a SuperIO block with two 16550-compatible
 * UARTs. The PROM and OS use UART A as the serial console.
 *
 * IOC3 UART register layout:
 *   Registers are byte-accessible at stride 1, but byte-reversed within
 *   32-bit words (big-endian MIPS byte addressing). The standard 16550
 *   register mapping is: std_reg = physical_offset ^ 3.
 *
 *   Physical address         16550 register
 *   IOC3_BASE + 0x20178      LCR   (reg 3)
 *   IOC3_BASE + 0x20179      IIR/FCR (reg 2)
 *   IOC3_BASE + 0x2017A      IER/DLM (reg 1)
 *   IOC3_BASE + 0x2017B      RBR/THR/DLL (reg 0)
 *   IOC3_BASE + 0x2017C      SCR   (reg 7)
 *   IOC3_BASE + 0x2017D      MSR   (reg 6)
 *   IOC3_BASE + 0x2017E      LSR   (reg 5)
 *   IOC3_BASE + 0x2017F      MCR   (reg 4)
 *
 * Reference: IRIX sys/PCI/ioc3.h, Linux arch/mips/sgi-ip30/ip30-console.c
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/misc/sgi_bridge.h"
#include "hw/misc/sgi_heart.h"
#include "hw/char/serial.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "chardev/char.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qapi/error.h"
#include "qemu/log.h"

/*
 * BRIDGE covers widget 0xF: physical 0x1F000000-0x1FFFFFFF (16MB XIO widget).
 *   +0x000000: BRIDGE control registers
 *   +0x022000: IOC3 PCI config space
 *   +0x600000: IOC3 devio window (serial, kbd, mouse, ethernet)
 *   +0xC00000: PROM flash (mapped separately as ROM)
 *   +0xE00000: extended flash/status window (addressed by the PROM)
 */
#define BRIDGE_REG_SIZE 0x1000000

/*
 * IOC3 SuperIO UART console, byte-spaced 16550.
 *
 * The IP30 PROM uses UART B at BRIDGE+0x620170 for its console struct (LSR at
 * base+5; a live poll loops there). The earlier pon_initio path and the IP27
 * model use UART A at BRIDGE+0x620178. Both are mapped to the same 16550 so
 * either base works.
 */
#define IOC3_UART_A_OFFSET  0x620170
#define IOC3_UART_B_OFFSET  0x620178

/*
 * IOC3 serial DMA control (SSCR) for the two ports, and its pause
 * handshake bits.  Offsets are offsetof(ioc3_mem_t, port_a/b.sscr) from
 * IRIX sys/PCI/ioc3.h (port_a at 0xb8, port_b at 0xd4 -- the Ethernet
 * block eregs that follows at 0xf0 is SGI_BRIDGE_ETH_OFF).
 */
#define IOC3_PORT_A_SSCR_OFF  0x6000b8
#define IOC3_PORT_B_SSCR_OFF  0x6000d4
#define IOC3_SSCR_DMA_EN      0x10000000u /* enable ring buffer DMA         */
#define IOC3_SSCR_DMA_PAUSE   0x20000000u /* pause the DMA channel          */
#define IOC3_SSCR_PAUSE_STATE 0x40000000u /* sets when pause takes effect   */
/*
 * Self-clearing command bits: the guest writes one to start a reset / RX
 * drain, then spins until the hardware clears it (sio_ioc3.c hardware_init
 * waits on SSCR_RESET via sio_cr; ioc3_read spins on SSCR_RX_DRAIN at
 * :1768).  Report them clear so those loops always terminate.
 */
#define IOC3_SSCR_RX_DRAIN    0x08000000u /* drain RX buffer to memory      */
#define IOC3_SSCR_RESET       0x80000000u /* reset DMA channels             */
#define IOC3_SSCR_SELFCLR     (IOC3_SSCR_RX_DRAIN | IOC3_SSCR_RESET)

/*
 * IOC3 serial DMA TX ring (the kernel console data path).
 *
 * io/sio_ioc3.c do_ioc3_write() packs console bytes into a 4K ring in guest
 * RAM, one 8-byte entry (4 data bytes + 4 status/control) at a time, then
 * stores the new producer index in stpir and enables the SIO interrupt.  The
 * hardware is expected to consume the entries (advancing stcir) and, when the
 * ring drains, raise SIO_IR_SA_TX_MT; ioc3_wrflush() spins on that bit and the
 * TX ISR (ioc3_serial_intr) uses it to wake the blocked console output.  Hb
 * the spec/IRIX, the ring base comes from sbbr_h/sbbr_l and the serial
 * interrupt is bridge bvec 4 -> HEART vector IP30_HVEC_IOC3_SERIAL (16).
 */
#define IOC3_SIO_IR_OFF       0x60001c
#define IOC3_SIO_IES_OFF      0x600020
#define IOC3_SIO_IEC_OFF      0x600024
#define IOC3_SBBR_H_OFF       0x6000b0
#define IOC3_SBBR_L_OFF       0x6000b4
#define IOC3_PORT_A_STPIR_OFF 0x6000bc
#define IOC3_PORT_A_STCIR_OFF 0x6000c0
#define IOC3_PORT_B_STPIR_OFF 0x6000d8
#define IOC3_PORT_B_STCIR_OFF 0x6000dc

#define IOC3_SIO_IR_SA_TX_MT        0x00000001u /* port A TX empty         */
#define IOC3_SIO_IR_SA_RX_HIGH      0x00000004u /* port A RX hi-water      */
#define IOC3_SIO_IR_SA_RX_TIMER     0x00000008u /* port A RX timeout       */
#define IOC3_SIO_IR_SA_TX_EXPLICIT  0x00000080u /* port A explicit TX intr */
#define IOC3_SIO_IR_SB_TX_MT        0x00000200u /* port B TX empty         */
#define IOC3_SIO_IR_SB_TX_EXPLICIT  0x00010000u /* port B explicit TX intr */

/*
 * IOC3 serial RX ring (port A console input).  Same shape as the TX half but
 * the hardware is the producer: received bytes are written into 8-byte entries
 * (4 data + 4 status) at srpir and the guest consumes them via srcir
 * (io/sio_ioc3.c ioc3_read()).  RX_A is the second 4K ring (struct ring_buffer
 * is TX_A, RX_A, TX_B, RX_B), and RXSB_DATA_VALID marks a byte.
 */
#define IOC3_PORT_A_SRPIR_OFF 0x6000c4
#define IOC3_PORT_A_SRCIR_OFF 0x6000c8
#define IOC3_SRCIR_ARM        0x80000000u /* arm RX timer                   */
#define IOC3_RXSB_DATA_VALID  0x80u       /* ring SC: data byte is valid    */
#define IOC3_SIO_RX_RING      4096        /* RX_A is the 2nd 4K ring        */

#define IOC3_SBBR_L_SIZE      0x00000001u /* 1 = 4K rings (not 1K)          */
#define IOC3_TXCB_VALID       0x40        /* ring SC: byte is valid        */
#define IOC3_TXCB_INT_WHEN_DONE 0x20      /* ring SC: interrupt when sent  */
#define IOC3_SIO_RING_BYTES   4096        /* RING_BUF_SIZE                 */
#define IOC3_SIO_RING_MASK    0x0ff8u     /* PROD_CONS_MASK (4K)           */
#define IOC3_SIO_PORT_B_RING  8192        /* TX_B is the 3rd 4K ring       */

/* Bridge bvec for the IOC3 serial (IP30_BVEC_IOC3_SERIAL = BRIDGE_IOC3_SPKM_ID). */
#define BRIDGE_BVEC_IOC3_SERIAL 4

/*
 * IOC3 UART custom MemoryRegion ops.
 *
 * Physical byte offset → standard 16550 register: std_reg = offset ^ 3
 * This accounts for big-endian byte ordering within 32-bit words on MIPS.
 */
/*
 * IOC3 SuperIO UART in 16550-compatibility mode is byte-spaced: the 16550
 * register index equals the byte offset (RBR/THR=0, IER=1, IIR/FCR=2, LCR=3,
 * MCR=4, LSR=5, MSR=6, SCR=7). This matches the IP27 BaseIO model, which was
 * verified against the IRIX ARCS ioc3uart.c init and console path; the older
 * `offset ^ 3` byte-reversal was an unverified assumption and dropped the
 * console entirely.
 */
static uint64_t ioc3_uart_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIBRIDGEState *s = opaque;
    return serial_io_ops.read(&s->ioc3_uart, addr, size);
}

static void ioc3_uart_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    SGIBRIDGEState *s = opaque;
    serial_io_ops.write(&s->ioc3_uart, addr, value, size);
}

static const MemoryRegionOps ioc3_uart_ops = {
    .read = ioc3_uart_read,
    .write = ioc3_uart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* --- DS2502 1-wire board-config EEPROM (bridge NIC line) ------------- */

/* Dallas/Maxim 1-wire CRC-8 (poly 0x8C, reflected). */
static uint8_t sgi_bridge_crc8(const uint8_t *p, int n)
{
    uint8_t crc = 0;
    int i, j;

    for (i = 0; i < n; i++) {
        uint8_t c = p[i];
        for (j = 0; j < 8; j++) {
            if ((crc ^ c) & 1) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
            c >>= 1;
        }
    }
    return crc;
}

/* NIC 16-bit CRC (poly 0xC001), from libsk/ml/nic.c. */
static const int sgi_bridge_oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                             1, 0, 0, 1, 0, 1, 1, 0};

static uint16_t sgi_bridge_crc16_step(uint16_t crc, uint8_t in)
{
    uint16_t data = in;

    data = (data ^ (crc & 0xff)) & 0xff;
    crc >>= 8;
    if (sgi_bridge_oddparity[data & 0xf] ^ sgi_bridge_oddparity[data >> 4]) {
        crc ^= 0xc001;
    }
    data <<= 6;
    crc ^= data;
    data <<= 1;
    crc ^= data;
    return crc;
}

static uint16_t sgi_bridge_crc16(const uint8_t *p, int n)
{
    uint16_t crc = 0;
    int i;

    for (i = 0; i < n; i++) {
        crc = sgi_bridge_crc16_step(crc, p[i]);
    }
    return crc;
}

static void sgi_bridge_put(uint8_t *dst, const char *s, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        dst[i] = s[i] ? (uint8_t)s[i] : ' ';
    }
}

/*
 * Build the DS2502 board-config EEPROM image: an IP30 board manufacturing
 * record. Page 0 holds the serial and part number, page 1 the
 * revision/group/name; each 32-byte page carries a trailing 16-bit CRC so
 * crc16(page) == 0xb001.
 */
static void sgi_bridge_ds_board_init(SGIDS *ds)
{
    static const char part[] = "030-1457-001"; /* IP30 system board */
    static const char name[] = "IP30";
    int a, b;

    memset(ds->mem, 0xff, sizeof(ds->mem));

    ds->mem[0] = 0x01;
    sgi_bridge_put(&ds->mem[1], "1234567890", 10);
    sgi_bridge_put(&ds->mem[11], part, 19);

    sgi_bridge_put(&ds->mem[32 + 0], "", 6);
    sgi_bridge_put(&ds->mem[32 + 6], "0001", 4);
    ds->mem[32 + 10] = 0x00;
    memset(&ds->mem[32 + 11], 0x00, 4);
    ds->mem[32 + 15] = 0x00;
    sgi_bridge_put(&ds->mem[32 + 16], name, 14);

    for (int page = 0; page < 2; page++) {
        uint8_t *pg = &ds->mem[page * 32];

        pg[30] = 0;
        pg[31] = 0;
        for (a = 0; a < 256; a++) {
            for (b = 0; b < 256; b++) {
                pg[30] = a;
                pg[31] = b;
                if (sgi_bridge_crc16(pg, 32) == 0xb001) {
                    goto done;
                }
            }
        }
    done:;
    }

    ds->rom[0] = 0x09;
    ds->rom[1] = 0x01;
    ds->rom[2] = 0x02;
    ds->rom[3] = 0x03;
    ds->rom[4] = 0x04;
    ds->rom[5] = 0x05;
    ds->rom[6] = 0x06;
    ds->rom[7] = sgi_bridge_crc8(ds->rom, 7);
    ds->extra_bits = 8; /* DS1982 returns a status byte before the data */
}

/*
 * Build the IOC3 MAC-address EEPROM image.  Format read by nic_eaddr()
 * (libsk/ml/nic.c): family 0x09; 14 bytes from address 0:
 *   [0]=0x8d command CRC, [1]=0x0a length, [6..11] = MAC (LSB..MSB),
 *   [12..13] = CRC16 solving crc16(bytes[1..13]) == 0xb001.
 */
static void sgi_bridge_ds_mac_init(SGIDS *ds)
{
    static const uint8_t mac[6] = {0x08, 0x00, 0x69, 0x12, 0x34, 0x56};
    int a, b;

    memset(ds->mem, 0xff, sizeof(ds->mem));
    ds->mem[0] = 0x8d;
    ds->mem[1] = 0x0a;
    ds->mem[2] = ds->mem[3] = ds->mem[4] = ds->mem[5] = 0x00;
    /* eaddr[i] = byte[11-i], so store MSB first at [6]. */
    for (int i = 0; i < 6; i++) {
        ds->mem[6 + i] = mac[5 - i];
    }
    ds->mem[12] = 0;
    ds->mem[13] = 0;
    for (a = 0; a < 256; a++) {
        for (b = 0; b < 256; b++) {
            ds->mem[12] = a;
            ds->mem[13] = b;
            if (sgi_bridge_crc16(&ds->mem[1], 13) == 0xb001) {
                goto done;
            }
        }
    }
done:;

    ds->rom[0] = 0x09;
    ds->rom[1] = 0x01;
    ds->rom[2] = 0x02;
    ds->rom[3] = 0x03;
    ds->rom[4] = 0x04;
    ds->rom[5] = 0x05;
    ds->rom[6] = 0x06;
    ds->rom[7] = sgi_bridge_crc8(ds->rom, 7);
    ds->extra_bits = 0; /* nic_eaddr reads the record from byte 0 */
}

static void sgi_bridge_ds_reset(SGIDS *ds)
{
    ds->state = 1; /* CMD */
    ds->cmd = 0;
    ds->cmd_bits = 0;
    ds->in = 0;
    ds->in_bits = 0;
    ds->out_index = 0;
    ds->search_phase = 0;
    ds->addr = 0;
    ds->extra = 0;
    ds->data_bit = 0;
}

static void sgi_bridge_ds_decode(SGIDS *ds)
{
    switch (ds->cmd) {
    case 0x33: /* READ ROM */
        ds->state = 2;
        ds->out_index = 0;
        break;
    case 0x55: /* MATCH ROM */
        ds->state = 3;
        ds->in = 0;
        ds->in_bits = 0;
        break;
    case 0xcc: /* SKIP ROM */
        ds->state = 1;
        break;
    case 0xf0: /* read-memory (or search-ROM) */
        ds->state = 5;
        break;
    default:
        ds->state = 1;
        break;
    }
    ds->cmd = 0;
    ds->cmd_bits = 0;
}

static void sgi_bridge_ds_write_bit(SGIDS *ds, int bit)
{
    switch (ds->state) {
    case 1: /* CMD */
        ds->cmd |= (bit & 1) << ds->cmd_bits;
        if (++ds->cmd_bits == 8) {
            sgi_bridge_ds_decode(ds);
        }
        break;
    case 3: /* MATCHROM */
        ds->in |= (bit & 1) << ds->in_bits;
        if (++ds->in_bits == 64) {
            ds->state = 1;
        }
        break;
    case 4: /* SEARCH */
        if (ds->search_phase == 2) {
            ds->search_phase = 0;
            if (++ds->out_index == 64) {
                ds->state = 1;
            }
        }
        break;
    case 5: /* F0_PENDING: a write means the 16-bit address follows */
        ds->state = 6;
        ds->addr = 0;
        ds->in_bits = 0;
        /* fall through */
    case 6: /* RMEM_ADDR */
        ds->addr |= (bit & 1) << ds->in_bits;
        if (++ds->in_bits == 16) {
            ds->state = 7;
            ds->out_index = 0;
            ds->extra = ds->extra_bits;
        }
        break;
    default:
        break;
    }
}

static int sgi_bridge_ds_read_bit(SGIDS *ds)
{
    int bit;

    switch (ds->state) {
    case 2: /* READROM */
        bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
        if (++ds->out_index == 64) {
            ds->state = 1;
        }
        return bit;
    case 4: /* SEARCH */
        bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
        if (ds->search_phase == 0) {
            ds->search_phase = 1;
            return bit;
        }
        ds->search_phase = 2;
        return bit ^ 1;
    case 5: /* F0_PENDING: a read starts a ROM search */
        ds->state = 4;
        ds->out_index = 0;
        ds->search_phase = 0;
        return sgi_bridge_ds_read_bit(ds);
    case 7: /* RMEM_DATA */
        if (ds->extra > 0) {
            bit = (0xff >> (8 - ds->extra)) & 1;
            ds->extra--;
            return bit;
        }
        bit = (ds->mem[ds->addr + ds->out_index / 8] >>
               (ds->out_index % 8)) & 1;
        if (++ds->out_index == 32 * 8) {
            ds->state = 1;
        }
        return bit;
    default:
        return 0;
    }
}

/* MCR line state: returns DATA bit; DONE is always set for the host poll. */
static uint64_t sgi_bridge_ds_line_read(SGIDS *ds)
{
    return 0x2 | (ds->data_bit & 1);
}

static void sgi_bridge_ds_line_write(SGIDS *ds, uint64_t val)
{
    unsigned pulse = (val >> 10) & 0x3ff;
    unsigned sample = (val >> 2) & 0xff;

    if (pulse >= 480) {
        /* reset/presence pulse; single device => presence bit 0 */
        sgi_bridge_ds_reset(ds);
        ds->data_bit = 0;
    } else if (sample == 30) {
        sgi_bridge_ds_write_bit(ds, 0);
        ds->data_bit = 0;
    } else if (sample == 110) {
        sgi_bridge_ds_write_bit(ds, 1);
        ds->data_bit = 0;
    } else if (sample == 13) {
        ds->data_bit = sgi_bridge_ds_read_bit(ds);
    } else {
        ds->data_bit = 0;
    }
}

/*
 * IOC3 Ethernet (10/100 MAC) register and DMA model.
 *
 * The ring layouts are those of the ARCS standalone `ef` driver
 * (references/stand/arcs/lib/libsk/net/if_ef.c):
 *   - TX ring: NTXD 128- or 512-byte-offset descriptors of 128 bytes
 *     (cmd, bufcnt, p1, p2, then 104 bytes of inline data). ETPIR/ETCIR
 *     are byte offsets into the ring (index * TXDSZ).
 *   - RX ring: 512 entries of 8 bytes, each the IO address of an
 *     `efrxbuf` (ioc3_erxbuf { w0, err } followed by the frame at
 *     EMCR.RXOFF halfwords). ERPIR is a byte offset (index * 8).
 */
#define IOC3_EMCR_DUPLEX    0x00000001
#define IOC3_EMCR_PROMISC   0x00000002
#define IOC3_EMCR_PADEN     0x00000004
#define IOC3_EMCR_RXOFF_MASK 0x000001f8
#define IOC3_EMCR_RXOFF_SHIFT 3
#define IOC3_EMCR_TXDMAEN   0x00002000
#define IOC3_EMCR_TXEN      0x00004000
#define IOC3_EMCR_RXDMAEN   0x00008000
#define IOC3_EMCR_RXEN      0x00010000
#define IOC3_EMCR_LOOPBACK  0x00020000
#define IOC3_EMCR_ARB_DIAG_IDLE 0x00200000
#define IOC3_EMCR_RST       0x80000000

#define IOC3_EISR_RXTIMERINT 0x00000001
#define IOC3_EISR_RXTHRESHINT 0x00000002
#define IOC3_EISR_TXEMPTY    0x00010000

#define IOC3_ETXD_D0V       0x00010000
#define IOC3_ETXD_B1V       0x00020000
#define IOC3_ETXD_B2V       0x00040000

#define IOC3_ETBR_L_RINGSZ_MASK 0x00000001
#define IOC3_ETBR_L_TXRINGBASE_MASK 0xffffc000
#define IOC3_ETPIR_TXPRODUCE_MASK 0x0000ffff

#define IOC3_ERXBUF_V           0x80000000
#define IOC3_ERXBUF_BYTECNT_SHIFT 16
#define IOC3_ERXBUF_GOODPKT     0x40000000
#define IOC3_ERXBUF_LONGEVENT   0x10000000
#define IOC3_ERXBUF_BROADCAST   0x08000000
#define IOC3_ERXBUF_MULTICAST   0x04000000

#define IOC3_RXDSZ 8
#define IOC3_NRXD  512
#define IOC3_TXDSZ 128
#define IOC3_RX_RING_BYTES (IOC3_NRXD * IOC3_RXDSZ)

/*
 * The ARCS ef driver programs the ring base and buffer pointers with
 * kv_to_bridge32_dirmap() addresses, which on IP30 come out as K1-segment
 * addresses (0xa0xxxxxx). The PROM runs from the low RAM alias at physical 0,
 * so translate those to physical addresses for DMA.
 */
static uint64_t ioc3_dma_addr(uint64_t a)
{
    if ((a & 0xf0000000ULL) == 0xa0000000ULL) {
        return a & 0x0fffffffULL;
    }
    return a;
}

/* ioc3_regs[] is indexed by (IOC3 devio offset - 0x600000) / 4. */
#define IOC3_IDX(off) (((off) - 0x600000) >> 2)

/*
 * SIO TX-ring base.  hardware_init() programs sbbr_h/sbbr_l from
 * pciio_dmatrans_addr(); on IP30 the result is XIO-tagged (top word
 * 0x80000000) with the system physical address in the low word.  sbbr_l bit 0
 * is SBBR_L_SIZE, not address.
 */
static uint64_t ioc3_sio_ring_phys(uint32_t h, uint32_t l)
{
    uint64_t a = ((uint64_t)h << 32) | (l & ~IOC3_SBBR_L_SIZE);

    if (a >> 32) {
        a &= 0xffffffffULL;
    }
    return ioc3_dma_addr(a);
}

static void sgi_bridge_dev_irq(void *opaque, int n, int level);

/*
 * IOC3 serial DMA, TX half.  do_ioc3_write() packs console bytes into a ring
 * in guest RAM (8-byte entries: 4 data + 4 status/control) and stores the new
 * producer in stpir; the hardware consumes the entries (advancing stcir) and,
 * once drained, reports TX-empty (SIO_IR_SA_TX_MT).  The kernel's console
 * output blocks until that happens -- ioc3_wrflush() spins on the bit and the
 * TX ISR wakes the output queue -- so without this the first kernel printf
 * after the console switches to interrupt-driven output stalls every thread.
 */
static void sgi_bridge_ioc3_sio_irq_update(SGIBRIDGEState *s)
{
    uint32_t pending = s->ioc3_regs[IOC3_IDX(IOC3_SIO_IR_OFF)] & s->ioc3_sio_ienb;
    int level;

    /*
     * The RX (input) interrupts ARE delivered by default: the console tty's
     * reader sleeps until RX_TIMER/RX_HIGH wakes it, and unlike TX that cannot
     * be covered by the guest polling.
     *
     * The TX-completion interrupts are NOT delivered by default.
     *
     * This model drains the TX ring synchronously, so the transmitter is
     * instantly "empty" after every write and SIO_IR_SA_TX_MT / _TX_EXPLICIT
     * are set on every console write.  The guest polls that bit (ioc3_wrflush
     * SPINs on SA_TX_MT) and, because the ring never fills, its output queue
     * never blocks -- so the status bits alone are sufficient.  Delivering a
     * CPU interrupt per write instead drives the console tty's output
     * notification (csio_output_lowat -> sv_broadcast) far faster than the
     * upper layer re-arms it, overflowing its sv semaphore (kernel assertion
     * sema.c "s_st.count < SHRT_MAX") right after root/swap mount.  Measured:
     * without the interrupt the miniroot boots to the installer prompt;
     * with it, the guest panics.  A faithful interrupt needs an asynchronous
     * (baud-rate) TX drain so TX-empty tracks a real idle transition; until
     * then the guest's polled path is what it actually uses.  Set
     * SGIBRIDGE_SIO_IRQ=1 to deliver it anyway (investigation only).
     */
    if (!getenv("SGIBRIDGE_SIO_IRQ")) {
        pending &= ~(IOC3_SIO_IR_SA_TX_MT | IOC3_SIO_IR_SA_TX_EXPLICIT |
                     IOC3_SIO_IR_SB_TX_MT | IOC3_SIO_IR_SB_TX_EXPLICIT);
    }
    level = pending != 0;
    if (level != s->ioc3_sio_irq_level) {
        if (level) {
            s->dbg_irq_raises++;
            if (getenv("SGIBRIDGE_RXDBG") &&
                (s->dbg_irq_raises <= 30 ||
                 (s->dbg_irq_raises & 0x3ff) == 0)) {
                fprintf(stderr, "RXIRQ#%llu pending=%08x raises=%llu bytes=%llu "
                        "sir_reads=%llu rx_timer_acks=%llu\n",
                        (unsigned long long)s->dbg_irq_raises, pending,
                        (unsigned long long)s->dbg_irq_raises,
                        (unsigned long long)s->dbg_rx_bytes,
                        (unsigned long long)s->dbg_sir_reads,
                        (unsigned long long)s->dbg_rx_timer_acks);
            }
        }
        s->ioc3_sio_irq_level = level;
        sgi_bridge_dev_irq(s, BRIDGE_BVEC_IOC3_SERIAL, level);
    }
}

static void sgi_bridge_ioc3_sio_tx_drain(SGIBRIDGEState *s, int port)
{
    uint32_t stpir_off = port ? IOC3_PORT_B_STPIR_OFF : IOC3_PORT_A_STPIR_OFF;
    uint32_t stcir_off = port ? IOC3_PORT_B_STCIR_OFF : IOC3_PORT_A_STCIR_OFF;
    uint32_t prod = s->ioc3_regs[IOC3_IDX(stpir_off)] & IOC3_SIO_RING_MASK;
    uint32_t cons = s->ioc3_regs[IOC3_IDX(stcir_off)] & IOC3_SIO_RING_MASK;
    uint64_t base = ioc3_sio_ring_phys(s->ioc3_regs[IOC3_IDX(IOC3_SBBR_H_OFF)],
                                       s->ioc3_regs[IOC3_IDX(IOC3_SBBR_L_OFF)]);
    int guard;

    if (port) {
        base += IOC3_SIO_PORT_B_RING;
    }

    for (guard = 0; cons != prod && guard < 4096; guard++) {
        uint8_t entry[8];
        int x;

        if (dma_memory_read(&address_space_memory, base + cons, entry,
                            sizeof(entry), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            break;
        }
        for (x = 0; x < 4; x++) {
            if (entry[4 + x] & IOC3_TXCB_VALID) {
                serial_io_ops.write(&s->ioc3_uart, 0, entry[x], 1);
            }
        }
        cons = (cons + sizeof(entry)) & IOC3_SIO_RING_MASK;
    }

    s->ioc3_regs[IOC3_IDX(stcir_off)] = cons;
    if (cons == prod) {
        /*
         * Transmitter idle.  The hardware signals this with SIO_IR TX_MT and
         * its ISR disables DMA; only the DMA disable is delivered here.
         *
         * Why TX_MT is NOT latched (measured, install-leg.md cont18bv): the
         * guest's ISR reads the TX condition out of sio_ir, so latching TX_MT
         * -- which a synchronous drain makes true after EVERY write -- makes
         * the ISR run its lowat upcall (UP_OUTPUT_LOWAT -> csio_output_lowat
         * -> sv_broadcast) on every console interrupt.  With no sleeping writer
         * to consume it, the tty's sv semaphore overflows (kernel assertion
         * sema.c "s_st.count < SHRT_MAX") as soon as any RX interrupt runs the
         * ISR.  ioc3_wrflush() accepts EITHER TX_MT or DMA disabled, so the
         * slow-polled path is still satisfied.  A faithful TX_MT wants an
         * asynchronous (baud-paced) drain so it fires once per real idle
         * transition rather than once per write; that remains the queued debt.
         */
        uint32_t sscr_off = port ? IOC3_PORT_B_SSCR_OFF : IOC3_PORT_A_SSCR_OFF;

        s->ioc3_regs[IOC3_IDX(sscr_off)] &= ~IOC3_SSCR_DMA_EN;
    }
}

/*
 * IOC3 serial DMA, RX half: console input.
 *
 * The kernel's console input path (io/sio_ioc3.c ioc3_read) drains the SIO RX
 * ring, NOT the 16550 RBR.  Bytes arriving on the shared chardev are written
 * as one entry per byte (RXSB_DATA_VALID) at srpir and the producer advances;
 * the RX_TIMER condition is what wakes the reader (the guest acks it by
 * writing sio_ir and re-arms via srcir).  Only port A (the console) is fed.
 * The gate is a configured ring (nonzero sbbr base): the PROM leaves it zero
 * and reads the 16550, so nothing steals its input.
 *
 * Simplification: the datasheet drains fewer than 4 packed bytes when the RX
 * timer expires; here the byte is written immediately and RX_TIMER is raised
 * with it (no srtr interval modelled).  Console input is not latency-critical,
 * and the condition the guest services is the same one.
 */
static void sgi_bridge_ioc3_sio_rx(SGIBRIDGEState *s, const uint8_t *buf,
                                   int size)
{
    uint32_t sscr = s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SSCR_OFF)];
    uint32_t cons, prod, irq = 0;
    uint64_t base;
    int i;

    if (getenv("SGIBRIDGE_RXDBG")) {
        fprintf(stderr, "RXDBG byte=%02x sscr=%08x base_h=%08x base_l=%08x "
                "ienb=%08x sir=%08x srpir=%08x srcir=%08x\n",
                size > 0 ? buf[0] : 0, sscr,
                s->ioc3_regs[IOC3_IDX(IOC3_SBBR_H_OFF)],
                s->ioc3_regs[IOC3_IDX(IOC3_SBBR_L_OFF)], s->ioc3_sio_ienb,
                s->ioc3_regs[IOC3_IDX(IOC3_SIO_IR_OFF)],
                s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SRPIR_OFF)],
                s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SRCIR_OFF)]);
    }
    /*
     * Gate on the ring being configured, NOT on SSCR_DMA_EN: the transmitter's
     * idle transition clears DMA_EN (see the TX drain), and the PROM runs with
     * the ring base zero.  A zero base means the kernel has not set up the RX
     * ring yet, so the 16550 keeps the input.
     */
    base = ioc3_sio_ring_phys(s->ioc3_regs[IOC3_IDX(IOC3_SBBR_H_OFF)],
                              s->ioc3_regs[IOC3_IDX(IOC3_SBBR_L_OFF)]);
    if (!base) {
        return;
    }
    base += IOC3_SIO_RX_RING;
    prod = s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SRPIR_OFF)] & IOC3_SIO_RING_MASK;

    for (i = 0; i < size; i++) {
        uint8_t entry[8] = { 0 };

        cons = s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SRCIR_OFF)] & IOC3_SIO_RING_MASK;
        /*
         * A pointer-only ring cannot distinguish full from empty by equality:
         * prod == cons means EMPTY (the guest reads while prod != cons, see
         * ioc3_read).  It is full when the next slot would collide with cons.
         */
        if (((prod + (uint32_t)sizeof(entry)) & IOC3_SIO_RING_MASK) == cons) {
            break;              /* ring full: drop, as the hardware would */
        }
        entry[0] = buf[i];
        entry[4] = IOC3_RXSB_DATA_VALID;
        if (!getenv("SGIBRIDGE_RX_NOWRITE")) {
            if (dma_memory_write(&address_space_memory, base + prod, entry,
                                 sizeof(entry), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                break;
            }
        }
        prod = (prod + sizeof(entry)) & IOC3_SIO_RING_MASK;
        s->ioc3_regs[IOC3_IDX(IOC3_PORT_A_SRPIR_OFF)] = prod;
        s->dbg_rx_bytes++;
        irq |= IOC3_SIO_IR_SA_RX_TIMER;
    }

    if (irq) {
        s->ioc3_regs[IOC3_IDX(IOC3_SIO_IR_OFF)] |= irq;
        if (getenv("SGIBRIDGE_RXDBG")) {
            fprintf(stderr, "SIRSET rx idx=%d -> %08x\n",
                    (int)IOC3_IDX(IOC3_SIO_IR_OFF),
                    s->ioc3_regs[IOC3_IDX(IOC3_SIO_IR_OFF)]);
        }
        if (!getenv("SGIBRIDGE_RX_NOIRQ")) {
            sgi_bridge_ioc3_sio_irq_update(s);
        }
    }
}

static int sgi_bridge_serial_can_receive(void *opaque)
{
    return 1;
}

static void sgi_bridge_serial_receive(void *opaque, const uint8_t *buf,
                                      int size)
{
    SGIBRIDGEState *s = opaque;

    serial_receive_bytes(&s->ioc3_uart, buf, size); /* PROM 16550 console */
    sgi_bridge_ioc3_sio_rx(s, buf, size);           /* kernel SIO RX ring */
}

static void sgi_bridge_serial_event(void *opaque, QEMUChrEvent event)
{
    /* Input is delivered through sgi_bridge_serial_receive; no events needed. */
}

/* MII management interface (IOC3 MICR/MIDR) and IEEE 802.3 PHY bits. */
#define IOC3_MICR_REGADDR_MASK   0x0000001f
#define IOC3_MICR_PHYADDR_MASK   0x000003e0
#define IOC3_MICR_PHYADDR_SHIFT  5
#define IOC3_MICR_READTRIG       0x00000400
#define IOC3_MICR_BUSY           0x00000800
#define IOC3_MIDR_DATA_MASK      0x0000ffff

#define MII_R0_AUTOEN            0x1000
#define MII_R1_AUTODONE          0x0020
#define MII_R1_LINKSTAT          0x0004
#define MII_R6_LPNWABLE          0x0001

static void sgi_bridge_phy_init(SGIBRIDGEState *s)
{
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    s->phy_regs[0] = 0x1000;  /* BMCR: auto-negotiation enabled */
    s->phy_regs[1] = 0x7824;  /* BMSR: caps + autoneg done + link up */
    /*
     * PHY identity: the IRIX ef driver's ef_phyprobe() computes
     *   val = (reg2 << 12) | (reg3 >> 4)
     * and accepts it only if it equals a known PHY (if_ef.h): ICS1890 is
     * 0x0015F42.  So reg2=0x0015 and reg3>>4=0xF42, i.e. reg3=0xF42r with r
     * the revision in the low nibble (ef_phyrev = reg3 & 0xf).  A reg3 of
     * 0xF400 gives 0x15F40, which matches nothing -> "ef0: PHY not found".
     */
    s->phy_regs[2] = 0x0015;  /* PHY ID 1 */
    s->phy_regs[3] = 0xf422;  /* PHY ID 2: ICS1890 (0x0015F42), rev 2 */
    s->phy_regs[4] = 0x01e0;  /* ANAR: 10/10FD/100/100FD */
    s->phy_regs[5] = 0x01e0;  /* ANLPAR: same */
    s->phy_regs[6] = 0x0001;  /* ANER: link partner auto-neg able */
    s->phy_write_data = 0;
    s->phy_read_data = 0;
}

/* SGIBRIDGE_RXTRACE=1: per-delivered-frame [RXTRACE] line (see install-leg.md
 * cont18t). Field names are shared verbatim with sgi_baseio.c's mirror so the
 * two map-free net lanes can be compared per-field. Zero cost when off. */
static int sgi_bridge_rxtrace = -1;

static void sgi_bridge_eth_deliver(SGIBRIDGEState *s, const uint8_t *buf,
                                   size_t len)
{
    uint32_t emcr = s->eth_regs[IOC3_EMCR];
    uint32_t rxoff = ((emcr & IOC3_EMCR_RXOFF_MASK) >> IOC3_EMCR_RXOFF_SHIFT) * 2;
    uint64_t erbr = ioc3_dma_addr(((uint64_t)s->eth_regs[IOC3_ERBR_H] << 32) |
                                  s->eth_regs[IOC3_ERBR_L]);
    uint64_t slot = 0, raw_desc = 0, xlated = 0;
    uint32_t w0, err;
    uint8_t frame[2048];
    const char *reason = "deliver";
    bool ok = true;

    /*
     * Log EVERY exit, including the refusal paths -- an instrument positioned
     * downstream of a drop reports a clean run (fleet note #2378).  If a
     * branch refuses a frame we must be able to see it.
     */
    if (sgi_bridge_rxtrace < 0) {
        sgi_bridge_rxtrace = getenv("SGIBRIDGE_RXTRACE") != NULL;
    }

    if (!(emcr & IOC3_EMCR_RXEN)) {
        reason = "refused:RXDISABLED"; ok = false;
    } else if (erbr == 0) {
        reason = "refused:ERBR0"; ok = false;
    } else if (len > sizeof(frame)) {
        reason = "refused:LEN>2048"; ok = false;
    } else if (dma_memory_read(&address_space_memory,
                               erbr + s->eth_rxprod, &slot, sizeof(slot),
                               MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        reason = "refused:DESCREAD"; ok = false;
    } else if (slot == 0) {
        reason = "refused:SLOT0"; ok = false;
    } else {
        raw_desc = be64_to_cpu(slot);
        xlated = ioc3_dma_addr(raw_desc);
        slot = xlated;
    }

    if (sgi_bridge_rxtrace) {
        uint64_t ram_size = MACHINE(qdev_get_machine())->ram_size;

        fprintf(stderr,
                "[RXTRACE] %s prod=%u ringbase=0x%llx slotoff=0x%x "
                "raw_desc=0x%llx xlated_phys=0x%llx len=%u "
                "inram=%d (ram_size=0x%llx) "
                "guest_ercir=0x%x guest_erpir=0x%x emcr=0x%x rxoff=0x%x\n",
                reason, s->eth_rxprod, (unsigned long long)erbr, s->eth_rxprod,
                (unsigned long long)raw_desc, (unsigned long long)xlated,
                (unsigned)len,
                (xlated != 0) && (xlated + len) <= ram_size,
                (unsigned long long)ram_size,
                s->guest_ercir, s->guest_erpir, emcr, rxoff);
    }
    if (!ok) {
        return;
    }

    w0 = IOC3_ERXBUF_V | ((uint32_t)(len + 4) << IOC3_ERXBUF_BYTECNT_SHIFT);
    err = IOC3_ERXBUF_GOODPKT | IOC3_ERXBUF_LONGEVENT;
    if (len >= 6 && (buf[0] & 1)) {
        err |= IOC3_ERXBUF_MULTICAST;
        if (buf[0] == 0xff && buf[1] == 0xff && buf[2] == 0xff &&
            buf[3] == 0xff && buf[4] == 0xff && buf[5] == 0xff) {
            err |= IOC3_ERXBUF_BROADCAST;
        }
    }

    memcpy(frame, buf, len);
    /* stl_p()/stq_p() already store in target (big-endian) order. */
    stl_p(&w0, w0);
    stl_p(&err, err);

    dma_memory_write(&address_space_memory, slot, &w0, 4,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, slot + 4, &err, 4,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, slot + rxoff, frame, len,
                     MEMTXATTRS_UNSPECIFIED);

    s->eth_rxprod = (s->eth_rxprod + IOC3_RXDSZ) % IOC3_RX_RING_BYTES;
    s->eth_regs[IOC3_ERPIR] = s->eth_rxprod;
    s->eth_regs[IOC3_EISR] |= IOC3_EISR_RXTHRESHINT;
}

static void sgi_bridge_eth_tx_drain(SGIBRIDGEState *s)
{
    uint32_t etpir = s->eth_regs[IOC3_ETPIR] & IOC3_ETPIR_TXPRODUCE_MASK;
    uint32_t etbr_l = s->eth_regs[IOC3_ETBR_L];
    uint64_t base = ioc3_dma_addr(((uint64_t)s->eth_regs[IOC3_ETBR_H] << 32) |
                    (etbr_l & IOC3_ETBR_L_TXRINGBASE_MASK));
    int ntxd = (etbr_l & IOC3_ETBR_L_RINGSZ_MASK) ? 512 : 128;
    uint32_t ring_bytes = ntxd * IOC3_TXDSZ;
    uint32_t emcr = s->eth_regs[IOC3_EMCR];

    while (s->eth_txcons != etpir) {
        uint8_t desc[IOC3_TXDSZ];
        uint32_t cmd, bufcnt, d0cnt, b1cnt, b2cnt;
        uint64_t p1, p2;
        uint8_t frame[2048];
        size_t flen = 0;

        if (dma_memory_read(&address_space_memory, base + s->eth_txcons,
                            desc, sizeof(desc),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            break;
        }
        cmd = ldl_be_p(desc);
        bufcnt = ldl_be_p(desc + 4);
        p1 = ioc3_dma_addr(ldq_be_p(desc + 8));
        p2 = ioc3_dma_addr(ldq_be_p(desc + 16));
        d0cnt = bufcnt & 0x7f;
        b1cnt = (bufcnt >> 8) & 0x7ff;
        b2cnt = (bufcnt >> 20) & 0x7ff;

        if ((cmd & IOC3_ETXD_D0V) && d0cnt <= sizeof(frame)) {
            memcpy(frame, desc + 24, d0cnt);
            flen = d0cnt;
        }
        if ((cmd & IOC3_ETXD_B1V) && flen + b1cnt <= sizeof(frame)) {
            dma_memory_read(&address_space_memory, p1, frame + flen, b1cnt,
                            MEMTXATTRS_UNSPECIFIED);
            flen += b1cnt;
        }
        if ((cmd & IOC3_ETXD_B2V) && flen + b2cnt <= sizeof(frame)) {
            dma_memory_read(&address_space_memory, p2, frame + flen, b2cnt,
                            MEMTXATTRS_UNSPECIFIED);
            flen += b2cnt;
        }

        if (emcr & IOC3_EMCR_LOOPBACK) {
            sgi_bridge_eth_deliver(s, frame, flen);
        } else if (s->nic && flen > 0) {
            qemu_send_packet(qemu_get_queue(s->nic), frame, flen);
        }

        s->eth_txcons = (s->eth_txcons + IOC3_TXDSZ) % ring_bytes;
    }

    s->eth_regs[IOC3_ETCIR] = s->eth_txcons;
    s->eth_regs[IOC3_EISR] |= IOC3_EISR_TXEMPTY;
}

static bool sgi_bridge_eth_can_receive(NetClientState *nc)
{
    SGIBRIDGEState *s = qemu_get_nic_opaque(nc);

    return s->eth_regs[IOC3_EMCR] & IOC3_EMCR_RXEN;
}

static ssize_t sgi_bridge_eth_receive(NetClientState *nc, const uint8_t *buf,
                                      size_t size)
{
    SGIBRIDGEState *s = qemu_get_nic_opaque(nc);

    sgi_bridge_eth_deliver(s, buf, size);
    return size;
}

static NetClientInfo net_sgi_bridge_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_bridge_eth_can_receive,
    .receive = sgi_bridge_eth_receive,
};

static uint64_t sgi_bridge_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIBRIDGEState *s = opaque;
    uint64_t val = 0;

    /*
     * The PCI I/O aperture (a device's I/O BAR mapped via pciio_piotrans_addr)
     * aliases the on-board IOC3 devio window: the IRIX ethernet driver reaches
     * the IOC3 MCR/SIO_CR/GPCR_S at 0x500030/0x500028/0x500034 and spins on the
     * MCR DONE bit.  Fold the aperture onto the 0x600000 register block.
     */
    if (offset >= 0x500000 && offset < 0x520000) {
        offset += 0x100000;
    }

    if (getenv("IOC3_SIO_DEBUG") && offset >= 0x600000 && offset < 0x600100) {
        static unsigned long rdburst;
        if ((rdburst++ & 0x3ffff) == 0) {
            fprintf(stderr, "IOC3SIO R rd#%lu [%06llx]\n",
                    rdburst, (unsigned long long)offset);
        }
    }

    switch (offset) {
    /*
     * BRIDGE widget ID (w_id, XIO config register at +4): part 0xc002 at
     * [27:12], mfg 0x036 (=54, SGI) at [10:1], rev 1 at [31:28]. The PROM's
     * widget discovery (heart_do_port) and init_bridge read it to identify
     * the Bridge, and the kernel's xwidget/cdl match requires BOTH the part
     * (BRIDGE_WIDGET_PART_NUM) and the mfg (SGI 0x036) to match pcibr_init's
     * xwidget_driver_register(0xc002, 54, "pcibr", ...).
     */
    case 0x0000 ... 0x0007:
        {
            uint64_t wid = 0x1c00206cULL;

            if (size == 8) {
                val = wid;
            } else if (size == 4) {
                /* Big-endian: +0 is the high word, +4 the low (part) word. */
                val = (offset == 0) ? (uint32_t)(wid >> 32) : (uint32_t)wid;
            } else {
                val = 0;
            }
        }
        break;
    /*
     * WIDGET_STATUS (0x0c). Bit 5 selects PCI (set) vs GIO mode; the PROM's
     * real_init_bridge only scans the PCI bus (and so finds the IOC3) when
     * this bit is set.
     */
    case 0x000c:
        val = s->regs[offset >> 2] | 0x00000020u;
        break;
    /*
     * BRIDGE local registers: 0x000008-0x00FFFF
     *
     * offset 0x000: widget config (w_id etc.)
     * offset 0x104: b_int_status (Interrupt Status register).
     *   The early POST bus-error handler at PROM 0xbfc00d40 reads this and,
     *   if bit 6 (0x40) is SET, jumps straight into the fault-LED spin at
     *   0xbfc178f4. Bit 6 must therefore read CLEAR on a healthy machine; the
     *   earlier stub returned 0x40 and hung the PROM there before any console
     *   output. All status bits read clear unless a device raises one.
     *
     * The PROM's pon_bridge POST walks control registers (e.g. EVEN_RESP at
     * 0x284) with write/read tests, so the general register file must retain
     * writes. regs[] is 0x1000 words, covering the whole 0x0000-0x3FFF span.
     */
    case 0x0104:
        val = s->regs[offset >> 2] & ~0x40ULL;
        break;

    /*
     * BRIDGE_NIC (MicroLAN/1-wire line control) at 0xb4. The PROM polls bit 1
     * (0x2) as the "transaction done" flag; without it the board-config read
     * spins forever. Bit 0 is the 1-wire data line.
     */
    case 0x00b4:
        val = sgi_bridge_ds_line_read(&s->bridge_ds);
        break;

    case 0x0008 ... 0x000b:
    case 0x000d ... 0x00b3:
    case 0x00b5 ... 0x0103:
    case 0x0105 ... 0x2FFF:
        val = s->regs[offset >> 2];
        break;
    /*
     * BRIDGE type-0 PCI configuration windows (one 4KB slot per device).
     * The IOC3 is device 2 with vendor 0x10A9 / device 0x0003. Absent slots
     * read 0 so the PROM's PCI scan treats them as empty (non-zero reads are
     * registered as bogus devices); the PROM's device graph / SAIO install
     * needs the IOC3 entry to register the serial console.
     */
    case 0x20000 ... 0x2FFFF:
        {
            unsigned dev = (offset - 0x20000) >> 12;
            hwaddr cfg = offset & 0xfff;

            if (dev == BRIDGE_SCSI0_ID || dev == BRIDGE_SCSI1_ID) {
                SGIQLispState *isp = &s->isp[dev];

                /*
                 * The ISP register file is 16-bit big-endian with a byte-lane
                 * swap, so a config dword reads back halves swapped: a 16-bit
                 * access at cfg 0x00 returns the *device* id and one at 0x02
                 * the *vendor* id. ql_init_board reads vendor from struct
                 * offset 2 and device from offset 0, so mirror that here.
                 */
                switch (cfg) {
                case 0x00:
                    val = (size == 2) ? QLISP_DEVICE
                                      : ((QLISP_DEVICE << 16) | QLISP_VENDOR);
                    break;
                case 0x02:
                    val = QLISP_VENDOR;
                    break;
                case 0x04:
                    val = isp->pci_cmd;
                    break;
                case 0x06:
                    val = 0; /* status */
                    break;
                case 0x08:
                    val = (QLISP_CLASS << 8) | isp->pci_rev;
                    break;
                case 0x0c:
                    val = 0; /* header type 0, single function */
                    break;
                case 0x10:
                    /* BAR0: I/O space, 256-byte decode.  Bit 0 reads back
                     * set (I/O indicator); the sizing probe returns the size
                     * mask | 0x1 = 0xffffff01. */
                    val = isp->pci_bar[0] | 0x1;
                    break;
                case 0x14:
                    /* BAR1: memory space, 4 KB decode (0xfffff000). */
                    val = isp->pci_bar[1];
                    break;
                default:
                    val = 0;
                    break;
                }
                break;
            }
            if (dev != BRIDGE_IOC3_ID) {
                val = 0;
                break;
            }
            switch (cfg) {
            case 0x00:
                val = 0x000310a9; /* IOC3 vendor/device */
                break;
            case 0x04:
                val = s->pci_cmd;
                break;
            case 0x08:
                val = 0x00000001; /* class/revision */
                break;
            case 0x10:
                val = s->pci_bar[0];
                break;
            case 0x14:
                val = s->pci_bar[1];
                break;
            default:
                val = 0;
                break;
            }
        }
        break;

    case 0x600000 ... 0x61FFFF:
        if (offset == 0x600028) {
            /*
             * IOC3 SIO_CR (SuperIO Control Register, IOC3 offset 0x028).
             * Bit 22 = SIO_CR_ARB_DIAG_IDLE (0x00400000).
             * The IRIX kernel polls this bit waiting for the SuperIO bus
             * arbiter to become idle before using the UART.  Return it
             * always set so the kernel does not spin here.
             */
            val = 0x00400000; /* SIO_CR_ARB_DIAG_IDLE */
        } else if (offset == 0x600030) {
            /* IOC3 MCR: 1-wire line to the MAC-address EEPROM. */
            val = sgi_bridge_ds_line_read(&s->ioc3_ds);
        } else if (offset == IOC3_PORT_A_SSCR_OFF || offset == IOC3_PORT_B_SSCR_OFF) {
            /*
             * IOC3 serial DMA control (SSCR).  SSCR_PAUSE_STATE reflects
             * SSCR_DMA_PAUSE: ioc3_open() asserts DMA_PAUSE and spins until
             * PAUSE_STATE reads back set (io/sio_ioc3.c SPIN).  Without this
             * the kernel's console open times out (~1e6 iterations) and the
             * serial console goes silent right after the boot banner.
             */
            uint32_t sscr = s->ioc3_regs[(offset - 0x600000) >> 2];

            sscr &= ~IOC3_SSCR_SELFCLR;
            val = sscr | ((sscr & IOC3_SSCR_DMA_PAUSE) ? IOC3_SSCR_PAUSE_STATE : 0);
        } else if (offset >= SGI_BRIDGE_ETH_OFF &&
                   offset < SGI_BRIDGE_ETH_OFF + SGI_BRIDGE_ETH_SIZE) {
            unsigned idx = (offset - SGI_BRIDGE_ETH_OFF) >> 2;

            val = s->eth_regs[idx];
            if (idx == IOC3_EMCR) {
                /* The driver spins on ARB_DIAG_IDLE after asserting RST. */
                val |= IOC3_EMCR_ARB_DIAG_IDLE;
            } else if (idx == IOC3_MICR) {
                val &= ~IOC3_MICR_BUSY; /* management ops complete instantly */
            } else if (idx == IOC3_MIDR_R) {
                val = s->phy_read_data & IOC3_MIDR_DATA_MASK;
            }
        } else {
            val = s->ioc3_regs[(offset - 0x600000) >> 2];
            if (offset == IOC3_SIO_IR_OFF) {
                /* Instrumentation: each ISR PENDING() poll reads this. */
                s->dbg_sir_reads++;
                if (getenv("SGIBRIDGE_RXDBG") && s->dbg_rx_bytes > 0 &&
                    s->dbg_sir_reads <= 200) {
                    fprintf(stderr, "SIRRD#%llu val=%08x\n",
                            (unsigned long long)s->dbg_sir_reads, (unsigned)val);
                }
            }
        }
        break;

    /*
     * IOC3 device window extended range: 0x620000-0x6FFFFF.
     * SuperIO index/data pair; the UART subregion at 0x620178 takes
     * priority over this range via its own subregion.
     */
    case 0x620000 ... 0x6FFFFF:
        if (offset == 0x6C0000) {
            val = s->sio_regs[s->sio_index];
        } else {
            val = 0;
        }
        break;

    /*
     * Bridge internal ATE RAM (0x10000..0x103ff) and external SSRAM
     * (0x80000..0xfffff). The ARCS size_bridge_ssram() probe writes an
     * SSRAM size marker at [0], [64k] and [128k] and reads it back; the
     * value retained at [0] selects the fitted size. Back both as plain
     * storage so writes persist and the probe sizes them correctly.
     */
    case 0x10000 ... 0x103ff:
        {
            unsigned n;

            for (n = 0; n < size; n++) {
                val |= (uint64_t)s->ate_ram[(offset - 0x10000) + n] << (8 * n);
            }
        }
        break;

    case 0x80000 ... 0xfffff:
        {
            unsigned n;

            for (n = 0; n < size; n++) {
                val |= (uint64_t)s->ext_ssram[(offset - 0x80000) + n] << (8 * n);
            }
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "BRIDGE: unimplemented read at offset 0x%05"
                      HWADDR_PRIx "\n", offset);
        break;
    }

    return val;
}

static void sgi_bridge_update_irq(SGIBRIDGEState *s);

static void sgi_bridge_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    SGIBRIDGEState *s = opaque;

    /* PCI I/O aperture aliases the IOC3 devio window (see sgi_bridge_read). */
    if (offset >= 0x500000 && offset < 0x520000) {
        offset += 0x100000;
    }

    if (getenv("SGIBRIDGE_RXDBG") && offset >= 0x130 && offset < 0x170) {
        fprintf(stderr, "BINTADDR W [%06llx] = %08llx\n",
                (unsigned long long)offset, (unsigned long long)val);
    }
    /*
     * Broad register-write trace for the RX-interrupt retry loop: log every
     * bridge/IOC3 register store once the first RX byte has arrived, so we can
     * see where ioc3_write_ireg() actually lands (the dispatcher calls it on
     * every retry, before vsema()).
     */
    if (getenv("SGIBRIDGE_RXDBG") && s->dbg_rx_bytes > 0) {
        static unsigned regw;
        if (regw < 240) {
            regw++;
            fprintf(stderr, "REGW#%u off=%06llx val=%08llx size=%u\n",
                    regw, (unsigned long long)offset,
                    (unsigned long long)val, size);
        }
    }
    if (getenv("IOC3_SIO_DEBUG") && offset >= 0x600000 && offset < 0x600100) {
        fprintf(stderr, "IOC3SIO W [%06llx] = %08llx size=%u\n",
                (unsigned long long)offset, (unsigned long long)val, size);
    }

    switch (offset) {
    case 0x0104:
        /* Interrupt status is read-only. */
        break;

    case 0x0000 ... 0x0007:
        /* Widget ID is read-only. */
        break;

    case 0x00b4:
        sgi_bridge_ds_line_write(&s->bridge_ds, val);
        break;

    case 0x0008 ... 0x00b3:
    case 0x00b5 ... 0x0103:
    case 0x0105 ... 0x2FFF:
        /* General register file (POST write/read tests land here). */
        s->regs[offset >> 2] = val;
        break;

    case 0x20000 ... 0x2FFFF:
        {
            unsigned dev = (offset - 0x20000) >> 12;
            hwaddr cfg = offset & 0xfff;

            if (dev == BRIDGE_SCSI0_ID || dev == BRIDGE_SCSI1_ID) {
                SGIQLispState *isp = &s->isp[dev];

                switch (cfg) {
                case 0x04:
                    isp->pci_cmd = val;
                    break;
                case 0x10:
                    /* BAR0: I/O space, 256 bytes.  The implemented bits are
                     * [15:8]; bits [7:1] read back as the size mask (zero for
                     * 256 B) and bit 0 is the I/O indicator (added on read).
                     * The all-ones probe therefore returns 0xffffff01. */
                    isp->pci_bar[0] = val & 0xffffff00;
                    break;
                case 0x14:
                    /* BAR1: memory space, 4 KB decode -> mask 0xfffff000. */
                    isp->pci_bar[1] = val & 0xfffff000;
                    break;
                default:
                    break;
                }
                break;
            }
            if (dev == BRIDGE_IOC3_ID) {
                /*
                 * PCI BAR size mask.  A real BAR carries a write mask for its
                 * decode size: SW writes all-ones and reads back a value with the
                 * implemented low bits cleared, so size = ~value + 1.  The IOC3
                 * sits in a 1 MB BRIDGE DevIO window (BRIDGE_DEVIO2, see
                 * sys/PCI/bridge.h), so its mask is ~0xfffff = 0xfff00000.
                 *
                 * Masking the stored value makes the all-ones probe return the
                 * size mask and a normal aligned base write pass through unchanged.
                 * Without it the guest cannot learn the window size, so it places
                 * the BAR arbitrarily (observed 0x00500000, which then falls inside
                 * SCSI1's DevIO window and gets mapped through it -- pcibr.c
                 * pcibr_piotrans_addr's first-pass "covering window" rule -- so
                 * pciio_pio_addr() never equals the console's BRIDGE_DEVIO2 base).
                 */
                switch (cfg) {
                case 0x04:
                    s->pci_cmd = val;
                    break;
                case 0x10:
                    s->pci_bar[0] = val & 0xfff00000;
                    break;
                case 0x14:
                    s->pci_bar[1] = val & 0xfff00000;
                    break;
                default:
                    break;
                }
            }
        }
        break;

    case 0x600000 ... 0x61FFFF:
        if (offset == 0x600030) {
            /* IOC3 MCR: 1-wire line to the MAC-address EEPROM. */
            sgi_bridge_ds_line_write(&s->ioc3_ds, val);
        } else if (offset == IOC3_SIO_IR_OFF) {
            /* SIO_IR is 1-to-clear: the ISR acks its source bits by writing. */
            s->ioc3_regs[IOC3_IDX(offset)] &= ~val;
            if (getenv("SGIBRIDGE_RXDBG") && s->dbg_sir_writes <= 60) {
                s->dbg_sir_writes++;
                fprintf(stderr, "SIRWR#%llu val=%08x -> sir=%08x\n",
                        (unsigned long long)s->dbg_sir_writes, (unsigned)val,
                        (unsigned)s->ioc3_regs[IOC3_IDX(offset)]);
            }
            if (val & IOC3_SIO_IR_SA_RX_TIMER) {
                s->dbg_rx_timer_acks++;
                if (getenv("SGIBRIDGE_RXDBG") &&
                    (s->dbg_rx_timer_acks <= 10 ||
                     (s->dbg_rx_timer_acks & 0x3ff) == 0)) {
                    fprintf(stderr, "RXACK#%llu rx_timer val=%08x raises=%llu "
                            "bytes=%llu sir_reads=%llu\n",
                            (unsigned long long)s->dbg_rx_timer_acks, (unsigned)val,
                            (unsigned long long)s->dbg_irq_raises,
                            (unsigned long long)s->dbg_rx_bytes,
                            (unsigned long long)s->dbg_sir_reads);
                }
            }
            sgi_bridge_ioc3_sio_irq_update(s);
        } else if (offset == IOC3_SIO_IES_OFF || offset == IOC3_SIO_IEC_OFF) {
            /*
             * IES and IEC are a set/clear pair over ONE interrupt-enable
             * register: a write sets (IES) or clears (IEC) bits, and BOTH
             * offsets read back the current mask (ioc3.h exposes the same
             * offsets as the read-only sio_ies_ro/sio_iec_ro).
             * The driver reads the enable via 0x20, so storing the result only
             * at the written offset leaves the reader with a stale mask.
             * That mattered: ioc3_intr() disables the SIO sources by writing
             * sio_iec and then re-reads pending = sio_ir & enable@0x20 to
             * decide whether to retry -- with the stale 0x20 the pending never
             * cleared and the dispatcher spun until its semaphore overflowed
             * (sema.c vsema assert).  Keep both in step.
             */
            if (offset == IOC3_SIO_IES_OFF) {
                s->ioc3_sio_ienb |= val;    /* enable set   */
            } else {
                s->ioc3_sio_ienb &= ~val;   /* enable clear */
            }
            s->ioc3_regs[IOC3_IDX(IOC3_SIO_IES_OFF)] = s->ioc3_sio_ienb;
            s->ioc3_regs[IOC3_IDX(IOC3_SIO_IEC_OFF)] = s->ioc3_sio_ienb;
            sgi_bridge_ioc3_sio_irq_update(s);
        } else if (offset == IOC3_PORT_A_STPIR_OFF ||
                   offset == IOC3_PORT_B_STPIR_OFF) {
            /* Serial TX producer index: drain the ring to stcir. */
            s->ioc3_regs[IOC3_IDX(offset)] = val;
            sgi_bridge_ioc3_sio_tx_drain(s, offset == IOC3_PORT_B_STPIR_OFF);
        } else if (offset != 0x600028) {
            if (offset >= SGI_BRIDGE_ETH_OFF &&
                offset < SGI_BRIDGE_ETH_OFF + SGI_BRIDGE_ETH_SIZE) {
                unsigned idx = (offset - SGI_BRIDGE_ETH_OFF) >> 2;

                if (idx == IOC3_EMCR) {
                    bool was_rxen = s->eth_regs[idx] & IOC3_EMCR_RXEN;
                    /* RST and the idle status bit are handled, not stored. */
                    s->eth_regs[idx] = val & ~(IOC3_EMCR_RST |
                                               IOC3_EMCR_ARB_DIAG_IDLE);
                    if (val & IOC3_EMCR_RST) {
                        s->eth_rxprod = 0;
                        s->eth_txcons = 0;
                        memset(s->eth_regs, 0, sizeof(s->eth_regs));
                    }
                    /*
                     * While RX is disabled, can_receive() makes the net layer
                     * queue incoming frames.  Deliver them once the guest
                     * re-enables the receiver, so a frame that raced the
                     * driver's reset/enable sequence is not lost.
                     */
                    if (s->nic && !was_rxen &&
                        (s->eth_regs[idx] & IOC3_EMCR_RXEN)) {
                        qemu_flush_queued_packets(qemu_get_queue(s->nic));
                    }
                } else if (idx == IOC3_ETPIR) {
                    s->eth_regs[idx] = val;
                    sgi_bridge_eth_tx_drain(s);
                } else if (idx == IOC3_MIDR_W) {
                    s->phy_write_data = val & IOC3_MIDR_DATA_MASK;
                    s->eth_regs[idx] = val;
                } else if (idx == IOC3_MICR) {
                    unsigned reg = val & IOC3_MICR_REGADDR_MASK;
                    unsigned phyaddr = (val & IOC3_MICR_PHYADDR_MASK) >>
                                       IOC3_MICR_PHYADDR_SHIFT;

                    if (val & IOC3_MICR_READTRIG) {
                        s->phy_read_data = s->phy_regs[reg];
                    } else if (phyaddr == 0) {
                        s->phy_regs[reg] = s->phy_write_data;
                    }
                    /* Management ops complete instantly (BUSY stays clear). */
                    s->eth_regs[idx] = val & ~IOC3_MICR_BUSY;
                } else if (idx == IOC3_ERPIR) {
                    s->guest_erpir = val;
                    s->eth_regs[idx] = val;
                } else if (idx == IOC3_ERCIR) {
                    s->guest_ercir = val;
                    s->eth_regs[idx] = val;
                } else {
                    s->eth_regs[idx] = val;
                }
            } else {
                /* SIO_CR bit 22 is a live status bit; store everything else. */
                s->ioc3_regs[(offset - 0x600000) >> 2] = val;
            }
        }
        break;

    /*
     * IOC3 device window extended range: 0x620000-0x6FFFFF.
     * SuperIO index/data pair at BRIDGE+0x6A0000/+0x6C0000 (a PC-style
     * bank of 8-bit SuperIO registers). Backed by storage so the PROM's
     * init sequence retains its writes. The IOC3 UART at 0x620178 is
     * handled by its own subregion (ioc3_uart_mr), which takes priority.
     */
    case 0x620000 ... 0x6FFFFF:
        if (offset == 0x6A0000) {
            s->sio_index = val & 0xff;
        } else if (offset == 0x6C0000) {
            s->sio_regs[s->sio_index] = val & 0xff;
        }
        break;

    /*
     * Bridge internal ATE RAM (0x10000..0x103ff) and external SSRAM
     * (0x80000..0xfffff); see the read side for why they are backed.
     */
    case 0x10000 ... 0x103ff:
        {
            unsigned n;

            for (n = 0; n < size; n++) {
                s->ate_ram[(offset - 0x10000) + n] = (val >> (8 * n)) & 0xff;
            }
            if (getenv("BRIDGE_DEBUG")) {
                fprintf(stderr, "BRIDGE: ATE wr off=0x%x val=0x%llx size=%u\n",
                        (unsigned)offset, (unsigned long long)val, size);
            }
        }
        break;

    case 0x80000 ... 0xfffff:
        {
            unsigned n;

            for (n = 0; n < size; n++) {
                s->ext_ssram[(offset - 0x80000) + n] = (val >> (8 * n)) & 0xff;
            }
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "BRIDGE: unimplemented write at offset 0x%05"
                      HWADDR_PRIx "\n", offset);
        break;
    }

    /* b_int_enable (or any write) can change whether an IRQ is asserted. */
    sgi_bridge_update_irq(s);
}

static const MemoryRegionOps sgi_bridge_ops = {
    .read = sgi_bridge_read,
    .write = sgi_bridge_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        /*
         * Allow byte-sized accesses (min=1).  The PROM uses sb/lb for
         * SuperIO index/data register pairs in the IOC3 device window
         * (BRIDGE+0x6A0000 and 0x6C0000).  With min_access_size=4, QEMU's
         * memory_region_access_valid() returns MEMTX_ERROR for any sb/lb,
         * causing an infinite DBE exception loop at the faulting instruction.
         *
         * Allow 8-byte accesses too (max=8): the PROM scans the flash window
         * (BRIDGE+0xC00000.., mapped by this region) with 64-bit ld/sd, and
         * max=4 turned every such access into a data-bus error.
         */
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void sgi_bridge_reset(DeviceState *dev)
{
    SGIBRIDGEState *s = SGI_BRIDGE(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ioc3_regs, 0, sizeof(s->ioc3_regs));
    memset(s->sio_regs, 0, sizeof(s->sio_regs));
    s->sio_index = 0;
    sgi_bridge_ds_board_init(&s->bridge_ds);
    sgi_bridge_ds_mac_init(&s->ioc3_ds);
    sgi_bridge_ds_reset(&s->bridge_ds);
    sgi_bridge_ds_reset(&s->ioc3_ds);
    memset(s->eth_regs, 0, sizeof(s->eth_regs));
    s->eth_rxprod = 0;
    s->eth_txcons = 0;
    sgi_bridge_phy_init(s);
}

/*
 * BRIDGE PCI interrupt aggregation.
 *
 * On IP30 the BRIDGE collects the PCI device interrupt requests (the two
 * QLogic ISP channels are BaseIO devices 0 and 1) into b_int_status and, when
 * one is pending, raises a single line that the machine routes to HEART's
 * IP30_HVEC_WIDERR_BASEIO (57).  The IRIX kernel's pcibr handler then reads
 * b_int_status and dispatches to the device's registered handler (qlintr).
 *
 * b_int_status is the low 8 bits (bvec: SCSI0=0, SCSI1=1, IOC3 eth=2, ...);
 * the high bits are bridge error conditions gated by b_int_enable.  Device
 * requests are level driven: the device line drops when the driver acks it.
 */
#define BRIDGE_INT_STATUS_OFF 0x104
#define BRIDGE_INT_ENABLE_OFF 0x10c
#define BRIDGE_ISR_INT_MSK    0x000000ffu

static void sgi_bridge_update_irq(SGIBRIDGEState *s)
{
    uint32_t status = s->regs[BRIDGE_INT_STATUS_OFF >> 2];
    uint32_t enable = s->regs[BRIDGE_INT_ENABLE_OFF >> 2];
    /*
     * Only the bridge ERROR bits (high) go to the HEART widget-error vector
     * (57) via this single output.  Device-line interrupts (low 8) are sent
     * per-line as their own HEART vector in sgi_bridge_dev_irq().
     */
    int level = (status & enable & ~BRIDGE_ISR_INT_MSK) != 0;

    qemu_set_irq(s->cpu_irq, level);
}

static void sgi_bridge_dev_irq(void *opaque, int n, int level)
{
    SGIBRIDGEState *s = opaque;
    uint32_t *status = &s->regs[BRIDGE_INT_STATUS_OFF >> 2];
    /* HEART vector the kernel programmed for this device line. */
    unsigned vec = s->regs[(0x134 + n * 8) >> 2] & 0xff;

    if (level) {
        *status |= (1u << n);
    } else {
        *status &= ~(1u << n);
    }
    /*
     * A device asserting its PCI interrupt sends the xtalk vector in
     * b_int_addr[line] (pcibr_xintr_preset / pcibr_intr_connect).  The kernel
     * dispatches that HEART vector to pcibr_intr_list_func -> the driver's
     * handler (e.g. qlintr).  Drive the HEART directly.
     */
    if (getenv("SGIBRIDGE_RXDBG") && n == 4) {
        fprintf(stderr, "DEVIRQ n=%d level=%d vec=%02x b_int_addr=%08x "
                "b_int_enable=%08x bytes=%llu raises=%llu sir_reads=%llu "
                "sir_writes=%llu rxacks=%llu\n", n, level, vec,
                s->regs[(0x134 + n * 8) >> 2],
                s->regs[BRIDGE_INT_ENABLE_OFF >> 2],
                (unsigned long long)s->dbg_rx_bytes,
                (unsigned long long)s->dbg_irq_raises,
                (unsigned long long)s->dbg_sir_reads,
                (unsigned long long)s->dbg_sir_writes,
                (unsigned long long)s->dbg_rx_timer_acks);
    }
    if (vec) {
        sgi_heart_raise_vector(s->heart, vec, level);
    }
    sgi_bridge_update_irq(s);
}

/*
 * Translate a BRIDGE ATE-mapped PCI DMA address to a system physical address.
 *
 * The IRIX ql driver programs the ISP ring bases with addresses from
 * pciio_dmatrans_addr(), which on IP30 fall in the BRIDGE ATE-mapped PCI window
 * (BRIDGE_DMA_MAPPED_BASE 0x40000000, 4K pages).  The guest fills the bridge
 * ATE RAM (b_int_ate_ram, mapped at 0x10000) itself, so read the ATE it wrote:
 *   ate  = ate_ram[(pci - BASE) >> 12]          (proto | port<<8 | xio pfn)
 *   xio  = (ate & ~0xfff) + (pci & 0xfff)
 *   phys = xio                                  (heart: XIO>=0x20000000 == addr)
 * Returns the input unchanged if the address is outside the window or the ATE
 * is not valid.
 */
static uint64_t sgi_bridge_dma_xlate(void *arg, uint64_t pci_addr)
{
    SGIBRIDGEState *s = arg;
    uint32_t idx, i, w0, w1;
    uint64_t ate;

    if (pci_addr < 0x40000000ULL || pci_addr >= 0x80000000ULL) {
        return pci_addr;
    }
    /* BRIDGE_DMA_MAPPED_BASE 0x40000000, IOPGSIZE 0x4000 (IOPFNSHIFT 14). */
    idx = (pci_addr - 0x40000000ULL) >> 14;
    if (idx >= 0x80) {                 /* external ATEs not modelled */
        return pci_addr;
    }
    /*
     * The guest writes the 64-bit ATE as a big-endian store; QEMU splits it
     * into two 32-bit word accesses, which the register handler stores
     * little-endian per word.  Recombine the two words (first = high).
     */
    w0 = 0;
    w1 = 0;
    for (i = 0; i < 4; i++) {
        w0 |= (uint32_t)s->ate_ram[idx * 8 + i] << (8 * i);
        w1 |= (uint32_t)s->ate_ram[idx * 8 + 4 + i] << (8 * i);
    }
    ate = ((uint64_t)w0 << 32) | w1;
    if (!(ate & 0x01)) {               /* ATE_V */
        return pci_addr;
    }
    if (getenv("BRIDGE_DEBUG")) {
        fprintf(stderr, "BRIDGE: xlate pci=0x%llx idx=%u ate=0x%llx -> phys=0x%llx\n",
                (unsigned long long)pci_addr, idx, (unsigned long long)ate,
                (unsigned long long)((ate & ~0x3fffULL) + (pci_addr & 0x3fff)));
    }
    return (ate & ~0x3fffULL) + (pci_addr & 0x3fff);
}

static void sgi_bridge_realize(DeviceState *dev, Error **errp)
{
    SGIBRIDGEState *s = SGI_BRIDGE(dev);
    Chardev *chr;
    int i;
    /* Output line to HEART IP30_HVEC_WIDERR_BASEIO; inputs are PCI devices. */
    qdev_init_gpio_out(dev, &s->cpu_irq, 1);
    qdev_init_gpio_in(dev, sgi_bridge_dev_irq, 8);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_bridge_ops, s,
                          "sgi-bridge", BRIDGE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /*
     * IOC3 UART A (serial console).
     *
     * Connect to the "ser0" socket chardev if it exists (created by the MCP
     * launch infrastructure), otherwise fall back to "serial0" (a plain
     * -serial stdio/file backend). Without this fallback the UART has no
     * backend and the PROM console is silently discarded.
     */
    chr = qemu_chr_find("ser0");
    if (!chr) {
        chr = qemu_chr_find("serial0");
    }
    if (chr) {
        qdev_prop_set_chr(DEVICE(&s->ioc3_uart), "chardev", chr);
    }

    if (!qdev_realize(DEVICE(&s->ioc3_uart), NULL, errp)) {
        return;
    }

    /*
     * Re-register the 16550's chardev handlers on the same frontend so ONE
     * backend feeds both consumers of console input: the 16550 (the PROM menu
     * reads its RBR) and the IOC3 SIO RX ring (the kernel reads that after
     * boot).  A chardev has a single frontend, so the 16550's handlers
     * (serial_receive1) cannot remain registered alongside ours; we override
     * them and forward bytes to serial_receive_bytes().  Any 'ser0'/fallback
     * chardev given to the UART above is picked up here.
     */
    qemu_chr_fe_set_handlers(&s->ioc3_uart.chr, sgi_bridge_serial_can_receive,
                             sgi_bridge_serial_receive, sgi_bridge_serial_event,
                             NULL, s, NULL, true);

    /*
     * Map the IOC3 UART at BRIDGE offsets 0x620170 (UART B; the PROM console)
     * and 0x620178 (UART A; the early pon_initio path). Both subregions take
     * priority over the parent sgi_bridge_ops and share one 16550 state.
     */
    memory_region_init_io(&s->ioc3_uart_mr, OBJECT(dev), &ioc3_uart_ops, s,
                          "ioc3-uart-b", 8);
    memory_region_add_subregion(&s->iomem, IOC3_UART_A_OFFSET,
                                &s->ioc3_uart_mr);

    memory_region_init_io(&s->ioc3_uart_mr2, OBJECT(dev), &ioc3_uart_ops, s,
                          "ioc3-uart-a", 8);
    memory_region_add_subregion(&s->iomem, IOC3_UART_B_OFFSET,
                                &s->ioc3_uart_mr2);

    /*
     * IP30 BaseIO on-board QLogic ISP1020 SCSI channels (PCI slots 0 and 1),
     * register (DevIO) windows at 0x200000 and 0x400000.
     */
    for (i = 0; i < 2; i++) {
        object_property_set_uint(OBJECT(&s->isp[i]), "scsi-bus-num", i,
                                 &error_abort);
        /* IP30's ARCS ql.c munges control entries; IP27's does not. */
        object_property_set_bool(OBJECT(&s->isp[i]), "control-munge", true,
                                 &error_abort);
        /*
         * IP30 RAM sits at 0x20000000, so the bridge direct-map range for RAM
         * (host phys + 0x80000000 = 0xa0000000..0xb0000000) overlaps the K1
         * range: data dsegs must decode as direct-map, not K1.  Declared here
         * (the IP30 board), so the shared ISP model carries no per-machine rule.
         */
        object_property_set_bool(OBJECT(&s->isp[i]), "data-dma-direct", true,
                                 &error_abort);
        /* Let the ISP resolve its ATE-mapped PCI DMA ring addresses. */
        s->isp[i].dma_xlate = sgi_bridge_dma_xlate;
        s->isp[i].dma_xlate_arg = s;
        if (!qdev_realize(DEVICE(&s->isp[i]), NULL, errp)) {
            return;
        }
        memory_region_add_subregion(&s->iomem,
                                    i == 0 ? BRIDGE_QLISP0_OFF : BRIDGE_QLISP1_OFF,
                                    &s->isp[i].regs);
    }
    /* Route the two ISP interrupt lines into b_int_status bits 0/1. */
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out(DEVICE(&s->isp[i]), 0, qdev_get_gpio_in(dev, i));
    }

    /*
     * IOC3 Ethernet NIC. The machine claims the default -nic/-netdev backend
     * (qemu_configure_nic_device) before realize; if none was given we still
     * model the register file and DMA engines, just without a transport. The
     * MAC address the driver programs into EMAR comes from the DS2502 EEPROM,
     * not from the backend, so the two need not agree.
     */
    if (s->nic_conf.peers.ncs[0]) {
        s->nic = qemu_new_nic(&net_sgi_bridge_eth_info, &s->nic_conf,
                              object_get_typename(OBJECT(dev)), dev->id,
                              &dev->mem_reentrancy_guard, s);
        qemu_format_nic_info_str(qemu_get_queue(s->nic), s->nic_conf.macaddr.a);
    } else {
        s->nic = NULL;
    }
}

static void sgi_bridge_instance_init(Object *obj)
{
    SGIBRIDGEState *s = SGI_BRIDGE(obj);
    object_initialize_child(obj, "ioc3-uart", &s->ioc3_uart, TYPE_SERIAL);
    object_initialize_child(obj, "qlisp0", &s->isp[0], TYPE_SGI_QLISP);
    object_initialize_child(obj, "qlisp1", &s->isp[1], TYPE_SGI_QLISP);
}

static const Property sgi_bridge_properties[] = {
    DEFINE_NIC_PROPERTIES(SGIBRIDGEState, nic_conf),
};

static void sgi_bridge_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_bridge_realize;
    device_class_set_props(dc, sgi_bridge_properties);
    device_class_set_legacy_reset(dc, sgi_bridge_reset);
}

static const TypeInfo sgi_bridge_info = {
    .name = TYPE_SGI_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIBRIDGEState),
    .instance_init = sgi_bridge_instance_init,
    .class_init = sgi_bridge_class_init,
};

static void sgi_bridge_register_types(void)
{
    type_register_static(&sgi_bridge_info);
}

type_init(sgi_bridge_register_types)
