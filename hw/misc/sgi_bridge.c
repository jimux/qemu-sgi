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
#include "hw/misc/sgi_bridge.h"
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
    s->phy_regs[2] = 0x0015;  /* PHY ID 1: ICS1890 OUI */
    s->phy_regs[3] = 0xf400;  /* PHY ID 2 */
    s->phy_regs[4] = 0x01e0;  /* ANAR: 10/10FD/100/100FD */
    s->phy_regs[5] = 0x01e0;  /* ANLPAR: same */
    s->phy_regs[6] = 0x0001;  /* ANER: link partner auto-neg able */
    s->phy_write_data = 0;
    s->phy_read_data = 0;
}

static void sgi_bridge_eth_deliver(SGIBRIDGEState *s, const uint8_t *buf,
                                   size_t len)
{
    uint32_t emcr = s->eth_regs[IOC3_EMCR];
    uint32_t rxoff = ((emcr & IOC3_EMCR_RXOFF_MASK) >> IOC3_EMCR_RXOFF_SHIFT) * 2;
    uint64_t erbr = ioc3_dma_addr(((uint64_t)s->eth_regs[IOC3_ERBR_H] << 32) |
                                  s->eth_regs[IOC3_ERBR_L]);
    uint64_t slot = 0;
    uint32_t w0, err;
    uint8_t frame[2048];

    if (!(emcr & IOC3_EMCR_RXEN) || erbr == 0 || len > sizeof(frame)) {
        return;
    }

    if (dma_memory_read(&address_space_memory,
                        erbr + s->eth_rxprod, &slot, sizeof(slot),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
        slot == 0) {
        return;
    }
    slot = ioc3_dma_addr(be64_to_cpu(slot));

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

    switch (offset) {
    /*
     * BRIDGE widget ID (w_id, XIO config register at +0): part 0xc002 at
     * [27:12], rev at [31:28]. The PROM's widget discovery (heart_do_port)
     * and init_bridge read it to identify the Bridge.
     */
    case 0x0000 ... 0x0007:
        {
            uint64_t wid = 0x1c002000ULL;

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
                    val = isp->pci_bar[0];
                    break;
                case 0x14:
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

static void sgi_bridge_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    SGIBRIDGEState *s = opaque;

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
                    isp->pci_bar[0] = val;
                    break;
                case 0x14:
                    isp->pci_bar[1] = val;
                    break;
                default:
                    break;
                }
                break;
            }
            if (dev == BRIDGE_IOC3_ID) {
                switch (cfg) {
                case 0x04:
                    s->pci_cmd = val;
                    break;
                case 0x10:
                    s->pci_bar[0] = val;
                    break;
                case 0x14:
                    s->pci_bar[1] = val;
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

static void sgi_bridge_realize(DeviceState *dev, Error **errp)
{
    SGIBRIDGEState *s = SGI_BRIDGE(dev);
    Chardev *chr;
    int i;

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
        if (!qdev_realize(DEVICE(&s->isp[i]), NULL, errp)) {
            return;
        }
        memory_region_add_subregion(&s->iomem,
                                    i == 0 ? BRIDGE_QLISP0_OFF : BRIDGE_QLISP1_OFF,
                                    &s->isp[i].regs);
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
