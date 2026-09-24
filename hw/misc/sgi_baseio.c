/*
 * SGI IP27 BaseIO board model.
 *
 * See include/hw/misc/sgi_baseio.h for provenance.  This models the XIO
 * widget-ID word, the bridge configuration 1-wire (MicroLAN) register at
 * +0xb4, and a DS2502 (family 0x09) whose memory holds the BaseIO
 * manufacturing record.
 *
 * The 1-wire bit protocol and the record layout come from the IRIX 6.5.7m
 * ARCS source: libsk/ml/nic.c (nic_presence/nic_read/nic_write/nic_read_one_page
 * /nic_mfg_next) and libkl/ml/bridge_nic.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/char/serial.h"
#include "hw/misc/sgi_baseio.h"
#include "hw/misc/sgi_hub.h"
#include "chardev/char.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "system/address-spaces.h"
#include "system/dma.h"

/* --- Dallas/Maxim 1-wire CRC-8 (poly 0x8C, reflected) --- */
static uint8_t sgi_baseio_crc8(const uint8_t *p, int n) {
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

/* --- NIC 16-bit CRC (poly 0xC001), from libsk/ml/nic.c --- */
static const int sgi_baseio_oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                             1, 0, 0, 1, 0, 1, 1, 0};

static uint16_t sgi_baseio_crc16_step(uint16_t crc, uint8_t in) {
  uint16_t data = in;
  data = (data ^ (crc & 0xff)) & 0xff;
  crc >>= 8;
  if (sgi_baseio_oddparity[data & 0xf] ^ sgi_baseio_oddparity[data >> 4]) {
    crc ^= 0xc001;
  }
  data <<= 6;
  crc ^= data;
  data <<= 1;
  crc ^= data;
  return crc;
}

static uint16_t sgi_baseio_crc16(const uint8_t *p, int n) {
  uint16_t crc = 0;
  int i;
  for (i = 0; i < n; i++) {
    crc = sgi_baseio_crc16_step(crc, p[i]);
  }
  return crc;
}

static void sgi_baseio_put(uint8_t *dst, const char *s, int n) {
  int i;
  for (i = 0; i < n; i++) {
    dst[i] = s[i] ? (uint8_t)s[i] : ' ';
  }
}

/*
 * Build the DS2502 image: a BaseIO manufacturing record.
 *   page 0 (pageA): [0]=0x01, [1..10]=serial, [11..29]=part number (19)
 *   page 1 (pageB): [0..5]=part cont., [6..9]=rev, [10]=group,
 *                   [11..14]=capability, [15]=variety, [16..29]=name
 * Each 32-byte page carries a trailing 16-bit CRC so that the NIC driver's
 * crc16 over the whole page equals 0xb001.
 */
static void sgi_baseio_ds_board_init(SGIBaseIODS *ds) {
  static const char part[] = "030-0734-001"; /* IO6 BaseIO */
  static const char name[] = "BASEIO";
  int a, b;

  memset(ds->mem, 0xff, sizeof(ds->mem));

  /* Page 0. */
  ds->mem[0] = 0x01;
  sgi_baseio_put(&ds->mem[1], "1234567890", 10);
  sgi_baseio_put(&ds->mem[11], part, 19);

  /* Page 1. */
  sgi_baseio_put(&ds->mem[32 + 0], "", 6);
  sgi_baseio_put(&ds->mem[32 + 6], "0001", 4);
  ds->mem[32 + 10] = 0x00;
  memset(&ds->mem[32 + 11], 0x00, 4);
  ds->mem[32 + 15] = 0x00;
  sgi_baseio_put(&ds->mem[32 + 16], name, 14);

  /* Solve each page's trailing 16-bit CRC so crc16(page) == 0xb001. */
  for (int page = 0; page < 2; page++) {
    uint8_t *pg = &ds->mem[page * 32];
    pg[30] = 0;
    pg[31] = 0;
    for (a = 0; a < 256; a++) {
      for (b = 0; b < 256; b++) {
        pg[30] = a;
        pg[31] = b;
        if (sgi_baseio_crc16(pg, 32) == 0xb001) {
          goto done;
        }
      }
    }
  done:;
  }

  /* ROM id: family 0x09 + 48-bit serial + Dallas CRC-8. */
  ds->rom[0] = 0x09;
  ds->rom[1] = 0x01;
  ds->rom[2] = 0x02;
  ds->rom[3] = 0x03;
  ds->rom[4] = 0x04;
  ds->rom[5] = 0x05;
  ds->rom[6] = 0x06;
  ds->rom[7] = sgi_baseio_crc8(ds->rom, 7);
  ds->extra_bits = 8; /* board record page read expects the status byte */
}

/*
 * Build the IOC3 MAC-address EEPROM image read by nic_eaddr() (libsk/ml/
 * nic.c): family 0x09; 14 bytes from address 0:
 *   [0]=0x8d command CRC, [1]=0x0a length, [6..11]=MAC (MSB..LSB),
 *   [12..13]=CRC16 solving crc16(bytes[1..13]) == 0xb001.
 * The MAC matches the octane lane and the netboot responder default
 * 08:00:69:12:34:56.
 */
static void sgi_baseio_ds_mac_init(SGIBaseIODS *ds) {
  static const uint8_t mac[6] = {0x08, 0x00, 0x69, 0x12, 0x34, 0x56};
  int i;

  memset(ds->mem, 0xff, sizeof(ds->mem));
  ds->mem[0] = 0x8d;
  ds->mem[1] = 0x0a;
  ds->mem[2] = ds->mem[3] = ds->mem[4] = ds->mem[5] = 0x00;
  /* eaddr[i] = byte[11-i], so store MSB first at [6]. */
  for (i = 0; i < 6; i++) {
    ds->mem[6 + i] = mac[5 - i];
  }
  ds->mem[12] = 0;
  ds->mem[13] = 0;
  for (int a = 0; a < 256; a++) {
    for (int b = 0; b < 256; b++) {
      ds->mem[12] = a;
      ds->mem[13] = b;
      if (sgi_baseio_crc16(&ds->mem[1], 13) == 0xb001) {
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
  ds->rom[7] = sgi_baseio_crc8(ds->rom, 7);
  ds->extra_bits = 0; /* nic_eaddr reads the record from byte 0 */
}

static void sgi_baseio_ds_reset(SGIBaseIODS *ds) {
  ds->state = SGI_DS_CMD;
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

static void sgi_baseio_ds_decode(SGIBaseIODS *ds) {
  switch (ds->cmd) {
  case 0x33: /* READ ROM */
    ds->state = SGI_DS_READROM;
    ds->out_index = 0;
    break;
  case 0x55: /* MATCH ROM */
    ds->state = SGI_DS_MATCHROM;
    ds->in = 0;
    ds->in_bits = 0;
    break;
  case 0xcc: /* SKIP ROM */
    ds->state = SGI_DS_CMD;
    break;
  case 0xf0: /* read-memory (or search-ROM; decided by next op) */
    ds->state = SGI_DS_F0_PENDING;
    break;
  default:
    ds->state = SGI_DS_CMD;
    break;
  }
  ds->cmd = 0;
  ds->cmd_bits = 0;
}

static void sgi_baseio_ds_write_bit(SGIBaseIODS *ds, int bit) {
  switch (ds->state) {
  case SGI_DS_CMD:
    ds->cmd |= (bit & 1) << ds->cmd_bits;
    if (++ds->cmd_bits == 8) {
      sgi_baseio_ds_decode(ds);
    }
    break;
  case SGI_DS_MATCHROM:
    ds->in |= (bit & 1) << ds->in_bits;
    if (++ds->in_bits == 64) {
      ds->state = SGI_DS_CMD;
    }
    break;
  case SGI_DS_SEARCH:
    /* Host choice for the current bit; advance to the next ROM bit. */
    if (ds->search_phase == 2) {
      ds->search_phase = 0;
      if (++ds->out_index == 64) {
        ds->state = SGI_DS_CMD;
      }
    }
    break;
  case SGI_DS_F0_PENDING:
    /* A write after 0xf0 means the 16-bit memory address follows. */
    ds->state = SGI_DS_RMEM_ADDR;
    ds->addr = 0;
    ds->in_bits = 0;
    /* fall through */
  case SGI_DS_RMEM_ADDR:
    ds->addr |= (bit & 1) << ds->in_bits;
    if (++ds->in_bits == 16) {
      ds->state = SGI_DS_RMEM_DATA;
      ds->out_index = 0;
      ds->extra = ds->extra_bits;
    }
    break;
  default:
    break;
  }
}

static int sgi_baseio_ds_read_bit(SGIBaseIODS *ds) {
  int bit;
  switch (ds->state) {
  case SGI_DS_READROM:
    bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
    if (++ds->out_index == 64) {
      ds->state = SGI_DS_CMD;
    }
    return bit;
  case SGI_DS_SEARCH:
    bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
    if (ds->search_phase == 0) {
      ds->search_phase = 1;
      return bit;
    }
    ds->search_phase = 2;
    return bit ^ 1;
  case SGI_DS_F0_PENDING:
    /* A read after 0xf0 starts a ROM search. */
    ds->state = SGI_DS_SEARCH;
    ds->out_index = 0;
    ds->search_phase = 0;
    return sgi_baseio_ds_read_bit(ds);
  case SGI_DS_RMEM_DATA:
    if (ds->extra > 0) {
      bit = (0xff >> (8 - ds->extra)) & 1;
      ds->extra--;
      return bit;
    }
    bit = (ds->mem[ds->addr + ds->out_index / 8] >>
           (ds->out_index % 8)) &
          1;
    if (++ds->out_index == 32 * 8) {
      ds->state = SGI_DS_CMD;
    }
    return bit;
  default:
    return 0;
  }
}

/* MCR line state: returns DATA bit; DONE is always set for the host poll. */
static uint64_t sgi_baseio_mcr_read(SGIBaseIODS *ds) {
  return 0x2 | (ds->data_bit & 1);
}

static void sgi_baseio_mcr_write(SGIBaseIODS *ds, uint64_t val) {
  unsigned pulse = (val >> 10) & 0x3ff;
  unsigned sample = (val >> 2) & 0xff;

  if (pulse >= 480) {
    /* reset/presence pulse; single device => presence bit 0 */
    sgi_baseio_ds_reset(ds);
    ds->data_bit = 0;
  } else if (sample == 30) {
    sgi_baseio_ds_write_bit(ds, 0);
    ds->data_bit = 0;
  } else if (sample == 110) {
    sgi_baseio_ds_write_bit(ds, 1);
    ds->data_bit = 0;
  } else if (sample == 13) {
    ds->data_bit = sgi_baseio_ds_read_bit(ds);
  } else {
    ds->data_bit = 0;
  }
}

/*
 * IOC3 Ethernet (10/100 MAC) register and DMA model.  Ported from the octane
 * lane's hw/misc/sgi_bridge.c; the ring layouts are those of the ARCS
 * standalone ef driver (references/stand/arcs/lib/libsk/net/if_ef.c):
 *   - TX ring: 128- or 512-entry descriptors of 128 bytes (cmd, bufcnt, p1,
 *     p2, then 104 bytes of inline data).  ETPIR/ETCIR are byte offsets.
 *   - RX ring: 512 entries of 8 bytes, each the IO address of an efrxbuf
 *     (ioc3_erxbuf { w0, err } followed by the frame at EMCR.RXOFF halfwords).
 */
#define IOC3_EMCR_DUPLEX 0x00000001
#define IOC3_EMCR_RXOFF_MASK 0x000001f8
#define IOC3_EMCR_RXOFF_SHIFT 3
#define IOC3_EMCR_LOOPBACK 0x00020000
#define IOC3_EMCR_RST 0x80000000
#define IOC3_EISR_RXTHRESHINT 0x00000002
#define IOC3_EISR_TXEMPTY 0x00010000
#define IOC3_EISR_TXEXPLICIT 0x00400000
#define IOC3_ETXD_D0V 0x00010000
#define IOC3_ETXD_B1V 0x00020000
#define IOC3_ETXD_B2V 0x00040000
#define IOC3_ETXD_INTWHENDONE 0x00001000
#define IOC3_ETCIR_IDLE 0x80000000
#define IOC3_ETBR_L_RINGSZ_MASK 0x00000001
#define IOC3_ETBR_L_TXRINGBASE_MASK 0xffffc000
#define IOC3_ETPIR_TXPRODUCE_MASK 0x0000ffff
#define IOC3_ERXBUF_V 0x80000000
#define IOC3_ERXBUF_BYTECNT_SHIFT 16
#define IOC3_ERXBUF_GOODPKT 0x40000000
#define IOC3_ERXBUF_LONGEVENT 0x10000000
#define IOC3_ERXBUF_BROADCAST 0x08000000
#define IOC3_ERXBUF_MULTICAST 0x04000000
#define IOC3_RXDSZ 8
#define IOC3_NRXD 512
#define IOC3_TXDSZ 128
#define IOC3_RX_RING_BYTES (IOC3_NRXD * IOC3_RXDSZ)

/*
 * The ef driver programs the ring base and buffer pointers with
 * kv_to_bridge32_dirmap() addresses, which on IP27 come out as K1-segment
 * addresses (0xa0xxxxxx).  Translate those to physical for DMA.
 */
static uint64_t sgi_baseio_dma_addr(uint64_t a) {
  /*
   * IP27 programs ring bases via kv_to_bridge32_dirmap(), which yields a
   * 64-bit value whose high word is the bridge dirmap selector and whose low
   * word is the physical offset (e.g. 0x15000000_01ce0000).  IP30 instead
   * yields a K1-segment address (0xa0xxxxxx) in a zero-extended 64-bit value.
   */
  if ((a >> 32) != 0) {
    return a & 0xffffffffULL; /* dirmap: low word is the physical address */
  }
  if ((a & 0xf0000000ULL) == 0xa0000000ULL) {
    return a & 0x0fffffffULL; /* K1 segment */
  }
  return a & 0x1fffffffULL;
}

static void sgi_baseio_phy_init(SGIBaseIOState *s) {
  memset(s->phy_regs, 0, sizeof(s->phy_regs));
  s->phy_regs[0] = 0x1000; /* BMCR: auto-negotiation enabled */
  s->phy_regs[1] = 0x7824; /* BMSR: caps + autoneg done + link up */
  /*
   * PHY identity: the IRIX ef driver's ef_phyprobe() computes
   *   val = (reg2 << 12) | (reg3 >> 4)
   * and accepts it only if it equals a known PHY (if_ef.h): ICS1890 is
   * 0x0015F42.  So reg2=0x0015 and reg3>>4=0xF42, i.e. reg3=0xF42r with r
   * the revision in the low nibble (ef_phyrev = reg3 & 0xf).  A reg3 of
   * 0xF400 gives 0x15F40, which matches nothing -> "ef0: PHY not found".
   * (Mirrors octane's sgi_bridge.c e4b043bb3d.)
   */
  s->phy_regs[2] = 0x0015; /* PHY ID 1: ICS1890 OUI */
  s->phy_regs[3] = 0xf422; /* PHY ID 2: ICS1890 (0x0015F42), rev 2 */
  s->phy_regs[4] = 0x01e0; /* ANAR: 10/10FD/100/100FD */
  s->phy_regs[5] = 0x01e0; /* ANLPAR: same */
  s->phy_regs[6] = 0x0001; /* ANER: link partner auto-neg able */
  s->phy_write_data = 0;
  s->phy_read_data = 0;
}

static int sgi_baseio_rxtrace = -1;
static int sgi_baseio_txtrace = -1;

/* Defined below with the bridge-interrupt delivery block; the RX/TX engines
 * latch EISR asynchronously and must re-evaluate the Ethernet line. */
static void sgi_baseio_int_sync(SGIBaseIOState *s);
static void sgi_baseio_eth_irq_sync(SGIBaseIOState *s);

static void sgi_baseio_eth_deliver(SGIBaseIOState *s, const uint8_t *buf,
                                   size_t len) {
  uint32_t emcr = s->eth_regs[SGI_IOC3_EMCR];
  uint32_t rxoff = ((emcr & IOC3_EMCR_RXOFF_MASK) >> IOC3_EMCR_RXOFF_SHIFT) * 2;
  uint64_t erbr = sgi_baseio_dma_addr(
      ((uint64_t)s->eth_regs[SGI_IOC3_ERBR_H] << 32) |
      s->eth_regs[SGI_IOC3_ERBR_L]);
  uint64_t slot = 0, raw_desc = 0;
  uint32_t w0, err;
  uint8_t frame[2048];

  if (sgi_baseio_rxtrace < 0) {
    sgi_baseio_rxtrace = getenv("SGIBASEIO_RXTRACE") != NULL;
  }
  if (!(emcr & 0x00010000) || erbr == 0 || len > sizeof(frame)) {
    /*
     * Log the REFUSAL, not only what landed: a size cap that silently drops is
     * invisible to any instrument placed after it (iris3130's rung on this).
     */
    if (sgi_baseio_rxtrace) {
      printf("[RXTRACE] REFUSED len=%u frame_cap=%zu rxen=%d erbr=0x%" PRIx64
             " emcr=0x%x\n", (unsigned)len, sizeof(frame),
             !!(emcr & 0x00010000), erbr, emcr);
    }
    return; /* RXEN off / frame cap */
  }
  if (dma_memory_read(&address_space_memory, erbr + s->eth_rxprod, &raw_desc,
                      sizeof(raw_desc), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
    if (sgi_baseio_rxtrace) {
      printf("[RXTRACE] DESC-FAIL(memtx) erbr=0x%" PRIx64 " slotoff=0x%x\n",
             erbr, s->eth_rxprod);
    }
    return;
  }
  if (raw_desc == 0) {
    if (sgi_baseio_rxtrace) {
      printf("[RXTRACE] DESC-ZERO erbr=0x%" PRIx64 " slotoff=0x%x\n",
             erbr, s->eth_rxprod);
    }
    return;
  }
  slot = sgi_baseio_dma_addr(be64_to_cpu(raw_desc));
  /*
   * Shared netboot instrument (shape mirrored verbatim from sgi_bridge.c so
   * the two ARCS lanes are comparable): per delivered frame, the RAW
   * descriptor value before translation, the translated physical, whether it
   * falls inside RAM BY SIZE, and our producer index beside the guest's own.
   * Settles, in one run: branch mis-handle vs ring bookkeeping vs neither.
   */
  if (sgi_baseio_rxtrace < 0) {
    sgi_baseio_rxtrace = getenv("SGIBASEIO_RXTRACE") != NULL;
  }
  if (sgi_baseio_rxtrace) {
    uint64_t ramsz = MACHINE(qdev_get_machine())->ram_size;
    printf("[RXTRACE] prod=%u ringbase=0x%" PRIx64 " slotoff=0x%x "
           "raw_desc=0x%" PRIx64 " xlated_phys=0x%" PRIx64 " len=%u "
           "inram=%d (ram_size=0x%" PRIx64 ") guest_erxpi=0x%x "
           "emcr=0x%x rxoff=0x%x\n",
           s->eth_rxprod, erbr, s->eth_rxprod, be64_to_cpu(raw_desc), slot,
           (unsigned)len, (slot + len) <= ramsz, ramsz,
           s->eth_regs[SGI_IOC3_ERPIR], emcr, rxoff);
  }

  w0 = IOC3_ERXBUF_V | ((uint32_t)(len + 4) << IOC3_ERXBUF_BYTECNT_SHIFT);
  err = IOC3_ERXBUF_GOODPKT | IOC3_ERXBUF_LONGEVENT;
  if (len >= 6 && (buf[0] & 1)) {
    err |= IOC3_ERXBUF_MULTICAST;
    if (buf[0] == 0xff && buf[1] == 0xff && buf[2] == 0xff && buf[3] == 0xff &&
        buf[4] == 0xff && buf[5] == 0xff) {
      err |= IOC3_ERXBUF_BROADCAST;
    }
  }
  memcpy(frame, buf, len);
  stl_p(&w0, w0);
  stl_p(&err, err);

  dma_memory_write(&address_space_memory, slot, &w0, 4,
                   MEMTXATTRS_UNSPECIFIED);
  dma_memory_write(&address_space_memory, slot + 4, &err, 4,
                   MEMTXATTRS_UNSPECIFIED);
  dma_memory_write(&address_space_memory, slot + rxoff, frame, len,
                   MEMTXATTRS_UNSPECIFIED);

  s->eth_rxprod = (s->eth_rxprod + IOC3_RXDSZ) % IOC3_RX_RING_BYTES;
  s->eth_regs[SGI_IOC3_ERPIR] = s->eth_rxprod;
  s->eth_regs[SGI_IOC3_EISR] |= IOC3_EISR_RXTHRESHINT;
  sgi_baseio_eth_irq_sync(s);
}

static void sgi_baseio_eth_tx_drain(SGIBaseIOState *s) {
  uint32_t etpir = s->eth_regs[SGI_IOC3_ETPIR] & IOC3_ETPIR_TXPRODUCE_MASK;
  uint32_t etbr_l = s->eth_regs[SGI_IOC3_ETBR_L];
  uint64_t base = sgi_baseio_dma_addr(
      ((uint64_t)s->eth_regs[SGI_IOC3_ETBR_H] << 32) |
      (etbr_l & IOC3_ETBR_L_TXRINGBASE_MASK));
  int ntxd = (etbr_l & IOC3_ETBR_L_RINGSZ_MASK) ? 512 : 128;
  uint32_t ring_bytes = ntxd * IOC3_TXDSZ;
  uint32_t emcr = s->eth_regs[SGI_IOC3_EMCR];
  bool explicit_int = false;

  if (sgi_baseio_txtrace < 0) {
    sgi_baseio_txtrace = getenv("SGIBASEIO_TXTRACE") != NULL;
  }
  if (sgi_baseio_txtrace) {
    printf("[TXTRACE] drain etpir=0x%x cons=0x%x base=0x%" PRIx64
           " ntxd=%d emcr=0x%x nic=%d\n",
           etpir, (unsigned)s->eth_txcons, base, ntxd, emcr, !!s->nic);
  }

  while (s->eth_txcons != etpir) {
    uint8_t desc[IOC3_TXDSZ];
    uint32_t cmd, bufcnt, d0cnt, b1cnt, b2cnt;
    uint64_t p1, p2;
    uint8_t frame[2048];
    size_t flen = 0;

    if (dma_memory_read(&address_space_memory, base + s->eth_txcons, desc,
                        sizeof(desc), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
      if (sgi_baseio_txtrace) {
        printf("[TXTRACE] desc-read FAIL cons=0x%x base=0x%" PRIx64 "\n",
               (unsigned)s->eth_txcons, base);
      }
      break;
    }
    cmd = ldl_be_p(desc);
    bufcnt = ldl_be_p(desc + 4);
    p1 = sgi_baseio_dma_addr(ldq_be_p(desc + 8));
    p2 = sgi_baseio_dma_addr(ldq_be_p(desc + 16));
    d0cnt = bufcnt & 0x7f;
    b1cnt = (bufcnt >> 8) & 0x7ff;
    b2cnt = (bufcnt >> 20) & 0x7ff;
    if (cmd & IOC3_ETXD_INTWHENDONE) {
      explicit_int = true; /* real IOC3 latches EISR_TXEXPLICIT on completion */
    }

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

    if (sgi_baseio_txtrace) {
      printf("[TXTRACE] desc cons=0x%x cmd=0x%x bufcnt=0x%x p1=0x%" PRIx64
             " p2=0x%" PRIx64 " d0cnt=%u b1cnt=%u b2cnt=%u flen=%zu\n",
             (unsigned)s->eth_txcons, cmd, bufcnt, p1, p2, d0cnt, b1cnt, b2cnt, flen);
    }

    if (emcr & IOC3_EMCR_LOOPBACK) {
      sgi_baseio_eth_deliver(s, frame, flen);
    } else if (s->nic && flen > 0) {
      qemu_send_packet(qemu_get_queue(s->nic), frame, flen);
    }
    s->eth_txcons = (s->eth_txcons + IOC3_TXDSZ) % ring_bytes;
  }
  s->eth_regs[SGI_IOC3_ETCIR] = s->eth_txcons | IOC3_ETCIR_IDLE;
  s->eth_regs[SGI_IOC3_EISR] |= IOC3_EISR_TXEMPTY;
  if (explicit_int) {
    s->eth_regs[SGI_IOC3_EISR] |= IOC3_EISR_TXEXPLICIT;
  }
  sgi_baseio_eth_irq_sync(s);
}

static bool sgi_baseio_eth_can_receive(NetClientState *nc) {
  SGIBaseIOState *s = qemu_get_nic_opaque(nc);
  return s->eth_regs[SGI_IOC3_EMCR] & 0x00010000; /* RXEN */
}

static ssize_t sgi_baseio_eth_receive(NetClientState *nc, const uint8_t *buf,
                                      size_t size) {
  SGIBaseIOState *s = qemu_get_nic_opaque(nc);
  sgi_baseio_eth_deliver(s, buf, size);
  return size;
}

static NetClientInfo net_sgi_baseio_eth_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_baseio_eth_can_receive,
    .receive = sgi_baseio_eth_receive,
};

/*
 * IOC3 byte-bus time-of-day / NVRAM part (Dallas DS1386-class timekeeping RAM).
 *
 * The IP27 kernel reaches it through the IOC3 byte bus: ml/SN/klclock.h sets
 * RTC_BASE_ADDR = IOC3_BYTEBUS_DEV0 + ioc3base, and ml/SN/nvram.c sets
 * nvram_base = memory_base + IOC3_NVRAM_OFFSET, both = 0x80000 within the IOC3
 * (sys/SN/SN0/klhwinit.h: IOC3_DEV_SEL_0 0x80000).  The IOC3 sits at
 * bridge+0x200000, so the chip lands at bridge+0x280000 -- the low 0xe bytes
 * are the clock registers, the bytes above are the battery-backed NVRAM
 * (sys/SN/nvram.h: NVOFF_DALLAS_CLOCK 0, NVLEN_DALLAS_CLOCK 0xe,
 * NVOFF_REVISION 0xe).  The register offsets, the update-disable/enable
 * protocol and the BCD encoding are from ml/SN/klclock.{c,h}; the map is
 * non-contiguous (SEC +0x1, MIN +0x2, HOUR +0x4, DAY +0x6, DATE +0x8,
 * MONTH +0x9, YEAR +0xa, CONTROL +0xb).
 *
 * rtodc() autodetects the part by writing 0xff to DAY and checking the
 * read-back: a DS1386 stores a BCD day and never returns 0xff, so present a
 * live Dallas calendar and the autodetect stays on the Dallas branch (the SGS
 * M48T35 alternative keeps its clock at the top of the region, +0x7ff8).
 * Reads return the HOST wall clock (project clock doctrine); wtodc() writes
 * re-base the calendar so a guest set-time is reflected afterwards.
 */
#define SGI_BASEIO_RTC_OFF 0x280000ULL
#define SGI_BASEIO_RTC_LEN 0xe /* NVLEN_DALLAS_CLOCK: NVRAM starts at 0xe */
#define RTC_DAL_SEC_OFF 0x1
#define RTC_DAL_MIN_OFF 0x2
#define RTC_DAL_HOUR_OFF 0x4
#define RTC_DAL_DAY_OFF 0x6
#define RTC_DAL_DATE_OFF 0x8
#define RTC_DAL_MONTH_OFF 0x9
#define RTC_DAL_YEAR_OFF 0xa
#define RTC_DAL_CONTROL_OFF 0xb
#define RTC_DAL_USER_OFF 0xe
#define RTC_DAL_UPDATE_ENABLE 0x80

static int sgi_baseio_bcd_decode(uint8_t v) {
  return ((v >> 4) & 0xf) * 10 + (v & 0xf);
}

static uint8_t sgi_baseio_bcd_encode(int v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

/* Days since 1970-01-01 from a proleptic Gregorian date (Howard Hinnant). */
static int64_t sgi_baseio_days_from_civil(int y, int m, int d) {
  int64_t era;
  int yoe, doy, doe;

  y -= (m <= 2);
  era = (y >= 0 ? y : y - 399) / 400;
  yoe = y - (int)era * 400;
  doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void sgi_baseio_civil_from_days(int64_t z, int *y, int *m, int *d) {
  int64_t era;
  int doe, yoe, doy, mp;

  z += 719468;
  era = (z >= 0 ? z : z - 146096) / 146097;
  doe = (int)(z - era * 146097);
  yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  *y = (int)(yoe + era * 400);
  doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp + (mp < 10 ? 3 : -9);
  *y += (*m <= 2);
}

/*
 * Current calendar fields, derived from the Unix epoch latched at reset plus
 * elapsed HOST wall-clock time (so the clock tracks real time and genuinely
 * ticks).  `year` is the full year; `wday` is 1..7 as the DS1386 expects.
 */
static void sgi_baseio_tod_fields(SGIBaseIOState *s, int *sec, int *min,
                                  int *hour, int *wday, int *mday, int *mon,
                                  int *year) {
  int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
  int64_t t = s->tod_epoch_sec + (now - s->tod_epoch_ns) / 1000000000LL;
  int64_t days = t / 86400;
  int y, mo, d;

  if (t < 0) {
    t = 0;
    days = 0;
  }
  sgi_baseio_civil_from_days(days, &y, &mo, &d);
  *sec = (int)(t % 60);
  *min = (int)((t / 60) % 60);
  *hour = (int)((t / 3600) % 24);
  *wday = (int)(((days + 4) % 7 + 7) % 7) + 1; /* 1970-01-01 was a Thursday */
  *mday = d;
  *mon = mo;
  *year = y;
}

static uint64_t sgi_baseio_tod_read(SGIBaseIOState *s, hwaddr off) {
  int sec, min, hour, wday, mday, mon, year;

  switch (off) {
  case RTC_DAL_CONTROL_OFF:
    return s->tod_control;
  case RTC_DAL_USER_OFF:
    return s->tod_user;
  default:
    break;
  }
  sgi_baseio_tod_fields(s, &sec, &min, &hour, &wday, &mday, &mon, &year);
  switch (off) {
  case RTC_DAL_SEC_OFF:
    return sgi_baseio_bcd_encode(sec);
  case RTC_DAL_MIN_OFF:
    return sgi_baseio_bcd_encode(min);
  case RTC_DAL_HOUR_OFF:
    return sgi_baseio_bcd_encode(hour);
  case RTC_DAL_DAY_OFF:
    return sgi_baseio_bcd_encode(wday);
  case RTC_DAL_DATE_OFF:
    return sgi_baseio_bcd_encode(mday);
  case RTC_DAL_MONTH_OFF:
    return sgi_baseio_bcd_encode(mon);
  case RTC_DAL_YEAR_OFF:
    /*
     * IRIX's clock only holds years relative to YRREF (1970): rtodc() does
     * `year += YRREF`, wtodc() does `year -= YRREF` (ml/SN/klclock.c).  So the
     * chip field is YEAR-1970, not YEAR%100.
     */
    return sgi_baseio_bcd_encode((year - 1970) % 100);
  default:
    return 0;
  }
}

static void sgi_baseio_tod_write(SGIBaseIOState *s, hwaddr off, uint64_t val) {
  int sec, min, hour, wday, mday, mon, year;
  int64_t days;

  switch (off) {
  case RTC_DAL_CONTROL_OFF:
    s->tod_control = val;
    return;
  case RTC_DAL_USER_OFF:
    s->tod_user = val;
    return;
  case RTC_DAL_SEC_OFF:
  case RTC_DAL_MIN_OFF:
  case RTC_DAL_HOUR_OFF:
  case RTC_DAL_DAY_OFF:
  case RTC_DAL_DATE_OFF:
  case RTC_DAL_MONTH_OFF:
  case RTC_DAL_YEAR_OFF:
    break;
  default:
    return;
  }
  sgi_baseio_tod_fields(s, &sec, &min, &hour, &wday, &mday, &mon, &year);
  switch (off) {
  case RTC_DAL_SEC_OFF:
    sec = sgi_baseio_bcd_decode(val) % 60;
    break;
  case RTC_DAL_MIN_OFF:
    min = sgi_baseio_bcd_decode(val) % 60;
    break;
  case RTC_DAL_HOUR_OFF:
    hour = sgi_baseio_bcd_decode(val) % 24;
    break;
  case RTC_DAL_DAY_OFF:
    /* Clamp so the NVRAM autodetect (writes 0xff) reads back a valid day. */
    wday = sgi_baseio_bcd_decode(val);
    if (wday < 1 || wday > 7) {
      wday = 1;
    }
    break;
  case RTC_DAL_DATE_OFF:
    mday = sgi_baseio_bcd_decode(val);
    if (mday < 1 || mday > 31) {
      mday = 1;
    }
    break;
  case RTC_DAL_MONTH_OFF:
    mon = sgi_baseio_bcd_decode(val);
    if (mon < 1 || mon > 12) {
      mon = 1;
    }
    break;
  case RTC_DAL_YEAR_OFF: {
    /* wtodc() writes year - YRREF (1970); see the read side. */
    int yy = sgi_baseio_bcd_decode(val);
    year = 1970 + yy;
    break;
  }
  }
  /*
   * Re-base the epoch so later reads reflect the write (wtodc() sets the
   * calendar field by field, so the epoch must move under it).
   */
  (void)wday;
  days = sgi_baseio_days_from_civil(year, mon, mday);
  s->tod_epoch_sec = days * 86400 + hour * 3600 + min * 60 + sec;
  s->tod_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
}

/*
 * IOC3 serial DMA TX ring -- the kernel console data path.  io/sio_ioc3.c
 * do_ioc3_write() packs console bytes into a ring of 8-byte entries (4 data
 * bytes + 4 status/control, IOC3_TXCB_VALID in the SC byte), then stores the
 * new producer in STPIR.  The kernel's console output BLOCKS until the
 * hardware consumes the entries, advances STCIR and reports TX-empty
 * (SIO_IR_SA_TX_MT) -- so without this model the first kernel printf after the
 * console switches to interrupt-driven output stalls (observed: everything
 * stops right after "Setting rbaud to 19200").  We drain synchronously: walk
 * STCIR up to STPIR, emit each valid data byte on the console chardev, and the
 * read path keeps SIO_IR reporting SA_TX_MT.  Ported from octane's sgi_bridge
 * (84b3de6551) at the IP27 IOC3 offsets (IOC3 base 0x200000; A STPIR 0xbc /
 * STCIR 0xc0, B 0xd8/0xdc, SBBR 0xb0/0xb4).
 */
#define SGI_BASEIO_SIO_RING_MASK 0x0ff8u
#define SGI_BASEIO_TXCB_VALID 0x40

/*
 * IOC3 SIO interrupt source bits we model (sys/PCI/ioc3.h).  We drain TX
 * synchronously so SA_TX_MT is permanently true; the RX bits are the only
 * ones the device raises and the driver's ISR can retire.
 */
#define SGI_BASEIO_SIO_IR_SA_TX_MT 0x00000001u
#define SGI_BASEIO_SIO_IR_SA_RX_HIGH 0x00000004u
#define SGI_BASEIO_SIO_IR_SA_RX_TIMER 0x00000008u
#define SGI_BASEIO_SIO_IR_SA_TX_EXPLICIT 0x00000080u
#define SGI_BASEIO_SIO_IR_SB_TX_MT 0x00000200u
#define SGI_BASEIO_SIO_IR_SB_TX_EXPLICIT 0x00010000u
#define SGI_BASEIO_SIO_IR_TX_BITS                                          \
  (SGI_BASEIO_SIO_IR_SA_TX_MT | SGI_BASEIO_SIO_IR_SA_TX_EXPLICIT |         \
   SGI_BASEIO_SIO_IR_SB_TX_MT | SGI_BASEIO_SIO_IR_SB_TX_EXPLICIT)

/* RX ring SC: this byte of the entry is a valid received data byte. */
#define SGI_BASEIO_RXSB_DATA_VALID 0x80u
/* struct ring_buffer is TX_A, RX_A, TX_B, RX_B -- RX_A is the 2nd 4K ring. */
#define SGI_BASEIO_SIO_RX_RING 4096

static void sgi_baseio_sio_tx_drain(SGIBaseIOState *s, int port) {
  uint64_t base = ((uint64_t)s->sbbr_h << 32) | (s->sbbr_l & ~1u);
  uint32_t prod = s->stpir[port] & SGI_BASEIO_SIO_RING_MASK;
  uint32_t cons = s->stcir[port] & SGI_BASEIO_SIO_RING_MASK;
  int guard;

  /* IP27: hardware_init() programs sbbr_l with the system physical address in
   * the low word; a nonzero top word is a DMA alias we drop (cf. sgi_qlisp). */
  base &= 0xffffffffULL;
  if (!base) {
    return; /* ring not configured yet: do not read phys 0 */
  }
  if (port) {
    base += 8192; /* TX_B is the third 4K ring */
  }
  for (guard = 0; cons != prod && guard < 4096; guard++) {
    uint8_t entry[8];
    int x;

    if (dma_memory_read(&address_space_memory, base + cons, entry,
                        sizeof(entry), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
      break;
    }
    for (x = 0; x < 4; x++) {
      if (entry[4 + x] & SGI_BASEIO_TXCB_VALID) {
        uint8_t b = entry[x];

        qemu_chr_fe_write_all(&s->ioc3_uart.chr, &b, 1);
      }
    }
    cons = (cons + sizeof(entry)) & SGI_BASEIO_SIO_RING_MASK;
  }
  s->stcir[port] = cons;
}

/*
 * Bridge PCI-interrupt delivery (sys/PCI/bridge.h).
 *
 * A BaseIO device asserts one of the bridge's eight device lines.  While that
 * line is enabled in b_int_enable the bridge sends the interrupt vector the
 * kernel programmed in b_int_addr[line] to the hub, which latches it into
 * INT_PEND0/1 (mirroring the real XIO interrupt message); on deassert the
 * vector is cleared.  The kernel's pcibr then reads b_int_status to find the
 * line and its driver clears the device at the source.
 */
/* Enabled with SGI_BASEIO_INTDBG=1 (mirrors the irq6/7/8 evidence runs). */
static bool sgi_baseio_intdbg(void) {
  static int on = -1;

  if (on < 0) {
    on = getenv("SGI_BASEIO_INTDBG") != NULL;
  }
  return on;
}

static void sgi_baseio_int_sync(SGIBaseIOState *s) {
  int n;

  for (n = 0; n < 8; n++) {
    bool asserted = (s->int_line & s->int_enable & (1u << n)) != 0;

    if (asserted) {
      unsigned vec = s->int_addr[n] & 0xff;

      if (vec && s->int_delivered[n] < 0) {
        if (s->hub) {
          sgi_hub_raise_vector(s->hub, vec, 1);
        }
        s->int_delivered[n] = vec;
        if (sgi_baseio_intdbg()) {
          qemu_log_mask(LOG_UNIMP,
                        "BASEIO int line=%d RAISE vec=%u (enable=0x%x "
                        "line=0x%x)\n",
                        n, vec, s->int_enable, s->int_line);
        }
      }
    } else if (s->int_delivered[n] >= 0) {
      if (s->hub) {
        sgi_hub_raise_vector(s->hub, s->int_delivered[n], 0);
      }
      if (sgi_baseio_intdbg()) {
        qemu_log_mask(LOG_UNIMP,
                      "BASEIO int line=%d CLEAR vec=%u (enable=0x%x "
                      "line=0x%x)\n",
                      n, s->int_delivered[n], s->int_enable, s->int_line);
      }
      s->int_delivered[n] = -1;
    }
  }
  qemu_set_irq(s->int_out, (s->int_line & s->int_enable & 0xffff) != 0);
}

/* A device line (wired as a qemu_irq input) asserted or deasserted. */
static void sgi_baseio_dev_irq(void *opaque, int n, int level) {
  SGIBaseIOState *s = opaque;

  if (level) {
    s->int_line |= 1u << n;
  } else {
    s->int_line &= ~(1u << n);
  }
  if (sgi_baseio_intdbg()) {
    qemu_log_mask(LOG_UNIMP,
                  "BASEIO dev_irq line=%d level=%d line=0x%x enable=0x%x\n",
                  n, level, s->int_line, s->int_enable);
  }
  sgi_baseio_int_sync(s);
}

/*
 * IOC3 SuperIO interrupt condition: sio_ir & sio_ienb.  We drain the serial
 * TX synchronously so sio_ir reports SA_TX_MT; the kernel enables that bit in
 * do_ioc3_write() and expects the serial ISR to run to disable DMA / deliver
 * the low-water notification, so the IOC3 line must follow the enable mask.
 */
static void sgi_baseio_ioc3_irq_sync(SGIBaseIOState *s) {
  uint32_t pending = s->sio_ir_pending & s->sio_ienb;

  /*
   * TX completion is NOT delivered: our synchronous drain makes SA_TX_MT true
   * after every write, and the driver's ISR services that condition with its
   * low-water upcall (UP_OUTPUT_LOWAT -> csio_output_lowat -> sv_broadcast).
   * Latching it would run that upcall on every console interrupt -- including
   * each RX interrupt -- and with no sleeping writer to consume it the tty's
   * sv semaphore overflows (kernel assertion sema.c "s_st.count < SHRT_MAX").
   * ioc3_wrflush() accepts SA_TX_MT (polled via SIO_IR) or DMA disabled, so
   * the polled output path is unaffected.  Same finding as octane's bridge.
   */
  pending &= ~SGI_BASEIO_SIO_IR_TX_BITS;
  if (pending) {
    s->int_line |= 1u << SGI_BASEIO_INT_DEV_IOC3;
  } else {
    s->int_line &= ~(1u << SGI_BASEIO_INT_DEV_IOC3);
  }
  sgi_baseio_int_sync(s);
}

/*
 * IOC3 serial DMA, RX half: console input.  The kernel's console input path
 * (io/sio_ioc3.c ioc3_read) drains the SIO RX ring, NOT the 16550 RBR.  Bytes
 * arriving on the shared chardev are written as one entry per byte
 * (RXSB_DATA_VALID) at SRPIR and the producer advances; the RX_TIMER condition
 * wakes the reader, which acks SIO_IR and re-arms via SRCIR_ARM.  Only port A
 * (the console) is fed, and only once the kernel has claimed the ring (the
 * rx_kernel latch): the PROM reads the 16550 and its serial_dma diagnostic
 * leaves SBBR programmed, so a configured ring alone cannot mean "kernel".
 *
 * Simplification: the datasheet parks fewer than 4 packed bytes in a
 * construction buffer until the RX timer expires; here the byte is written
 * immediately with RX_TIMER (no SRTR interval modelled).  Console input is not
 * latency-critical and the condition the guest services is the same one.
 * Ported from octane's sgi_bridge (199d2bdd53) at the IP27 IOC3 offsets.
 */
static void sgi_baseio_sio_rx(SGIBaseIOState *s, const uint8_t *buf, int size) {
  uint64_t base = ((uint64_t)s->sbbr_h << 32) | (s->sbbr_l & ~1u);
  uint32_t prod;
  int i;

  /* IP27: the low word holds the system physical address (cf. TX drain). */
  base &= 0xffffffffULL;
  if (!base) {
    return; /* RX ring not configured yet: the 16550 keeps the input */
  }
  base += SGI_BASEIO_SIO_RX_RING;
  prod = s->srpir & SGI_BASEIO_SIO_RING_MASK;

  for (i = 0; i < size; i++) {
    uint8_t entry[8] = {0};
    uint32_t cons = s->srcir & SGI_BASEIO_SIO_RING_MASK;

    /*
     * A pointer-only ring cannot distinguish full from empty by equality:
     * prod == cons is EMPTY (ioc3_read loops while prod != cons).  It is full
     * when the next slot would collide with the guest's consumer.
     */
    if (((prod + (uint32_t)sizeof(entry)) & SGI_BASEIO_SIO_RING_MASK) ==
        cons) {
      break; /* ring full: drop, as the hardware would */
    }
    entry[0] = buf[i];
    entry[4] = SGI_BASEIO_RXSB_DATA_VALID;
    if (dma_memory_write(&address_space_memory, base + prod, entry,
                         sizeof(entry), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
      break;
    }
    prod = (prod + sizeof(entry)) & SGI_BASEIO_SIO_RING_MASK;
    s->srpir = prod;
    s->sio_ir_pending |= SGI_BASEIO_SIO_IR_SA_RX_TIMER;
  }
  sgi_baseio_ioc3_irq_sync(s);
}

/* The configured RX ring base (0 until SBBR is programmed). */
static uint64_t sgi_baseio_sio_rx_base(SGIBaseIOState *s) {
  uint64_t base = ((uint64_t)s->sbbr_h << 32) | (s->sbbr_l & ~1u);

  return base & 0xffffffffULL;
}

/*
 * The console input consumer changes at the boot handoff.  The PROM reads the
 * 16550 RBR (and its serial_dma diagnostic leaves SBBR programmed, so a
 * nonzero ring base cannot mark the kernel); the kernel reads the SIO RX ring
 * and, only once its console driver is up, enables the RX interrupts in IES
 * and arms the timer via SRCIR.  Latch that so input is routed to the ring
 * only after the kernel owns it; until then the 16550 keeps the input exactly
 * as the shared serial frontend did.
 */
static bool sgi_baseio_rxdbg(void) {
  static int on = -1;

  if (on < 0) {
    on = getenv("SGI_BASEIO_RXDBG") != NULL;
  }
  return on;
}

/*
 * One backend feeds both consumers of console input: the 16550 (the PROM menu
 * reads its RBR) and the IOC3 SIO RX ring (the kernel reads that after boot).
 * A chardev has a single frontend, so we own the handlers and forward to the
 * UART's receive path through serial_receive_bytes().
 *
 * The consumer changes at the boot handoff, and it decides who paces input:
 * until the kernel claims the ring (rx_kernel) the PROM reads the 16550, so
 * advertise the UART's own capacity (a burst into a non-FIFO 16550 keeps only
 * its first byte, and the original frontend paced exactly this way); once the
 * kernel owns the ring it is the console reader and we accept freely.
 */
static int sgi_baseio_serial_can_receive(void *opaque) {
  SGIBaseIOState *s = opaque;

  if (s->rx_kernel) {
    return 1;
  }
  return serial_can_receive_bytes(&s->ioc3_uart);
}

static void sgi_baseio_serial_receive(void *opaque, const uint8_t *buf,
                                      int size) {
  SGIBaseIOState *s = opaque;

  if (sgi_baseio_rxdbg()) {
    qemu_log_mask(LOG_UNIMP,
                  "BASEIO RX byte=%02x rx_kernel=%d base=0x%llx srcir=0x%x "
                  "ienb=0x%x\n",
                  size > 0 ? buf[0] : 0, s->rx_kernel,
                  (unsigned long long)sgi_baseio_sio_rx_base(s), s->srcir,
                  s->sio_ienb);
  }
  if (s->rx_kernel) {
    sgi_baseio_sio_rx(s, buf, size); /* kernel SIO RX ring */
  } else if (serial_can_receive_bytes(&s->ioc3_uart)) {
    serial_receive_bytes(&s->ioc3_uart, buf, size); /* PROM 16550 console */
  }
}

static void sgi_baseio_serial_event(void *opaque, QEMUChrEvent event) {
  /* Input is delivered through sgi_baseio_serial_receive; no events needed. */
}

/*
 * IOC3 Ethernet interrupt condition: EISR & EIER (sys/PCI/ioc3.h; the ef
 * driver programs EIER in ef_init and clears EISR write-1-to-clear in ef_intr).
 * The Ethernet MAC sits on PCI slot 0 INTA = bridge line 0, whose connected
 * vector is 11 (ef_intr).  Keeping the line in int_line -- rather than only
 * OR-ing it into the INT_STATUS read -- is what lets sgi_baseio_int_sync send
 * the vector to the hub and, crucially, deassert it once the driver clears
 * EISR; a level held past the handler's clear is an interrupt livelock.
 */
static void sgi_baseio_eth_irq_sync(SGIBaseIOState *s) {
  if (s->eth_regs[SGI_IOC3_EISR] & s->eth_regs[SGI_IOC3_EIER]) {
    s->int_line |= 1u << SGI_BASEIO_INT_DEV_ETH;
  } else {
    s->int_line &= ~(1u << SGI_BASEIO_INT_DEV_ETH);
  }
  sgi_baseio_int_sync(s);
}

/*
 * IO6 boot flash (AMD Am29F080-class NOR) at bridge DevIO +0xC00000.
 *
 * The IRIX flash(1M) tool (cmd/flashio/fprom_sn0.c, FPROM_DEV_IO6_P1) drives
 * the same AMD command protocol as the hub flash, but with the IO6 P1 access
 * pattern: commands are 16-bit stores at WORD-indexed offsets (SHCMD_IO6_P1
 * writes `*(ushort_t *)base + offset`, so word 0x5555 lands at byte 0xAAAA and
 * word 0x2AAA at byte 0x5554), autoselect is read back with 16-bit loads at raw
 * byte offsets 0 (manufacturer) and 2 (device), and data/erase/status use raw
 * byte offsets (LB_IO6_P1).  do_probe accepts the Am29F080 pair (manu 0x01,
 * dev 0xd5); without it fprom_probe returns FPROM_ERROR_DEVICE and
 * flash_writeprom sets EINVAL (flashio_sn0.c:1372) -- the "errno: 22" that
 * wedged the install's flash exitop.  Measured: the tool's accesses land
 * exactly here (window offs 0xC0AAAA/0xC05554/0xC00000).
 */
static uint8_t sgi_baseio_ioprom_byte(SGIBaseIOState *s, uint64_t off) {
  if (s->ioprom_autoselect) {
    /* 16-bit BE reads: LH(0)=manu, LH(2)=dev.  0x0001/0x00d5 is accepted. */
    switch (off) {
    case 0: return 0x00;
    case 1: return 0x01;
    case 2: return 0x00;
    case 3: return 0xd5;
    default: return 0xff;
    }
  }
  return off < SGI_BASEIO_IOPROM_SIZE ? s->ioprom[off] : 0xff;
}

static uint64_t sgi_baseio_ioprom_read(void *opaque, hwaddr off, unsigned size) {
  SGIBaseIOState *s = opaque;
  uint64_t v = 0;
  unsigned i;
  for (i = 0; i < size; i++) {
    v = (v << 8) | sgi_baseio_ioprom_byte(s, off + i);
  }
  if (s->ioprom_dbg) {
    qemu_log_mask(LOG_GUEST_ERROR, "ioprom[%u] R off=0x%" PRIx64 " size=%u -> "
                  "0x%" PRIx64 "\n", (unsigned)s->widget, (uint64_t)off, size, v);
  }
  return v;
}

static void sgi_baseio_ioprom_write(void *opaque, hwaddr off, uint64_t val,
                                    unsigned size) {
  SGIBaseIOState *s = opaque;
  uint8_t b = val & 0xff; /* command macros store the byte zero-extended */

  if (s->ioprom_dbg) {
    qemu_log_mask(LOG_GUEST_ERROR, "ioprom[%u] W off=0x%" PRIx64 " size=%u "
                  "val=0x%" PRIx64 "\n", (unsigned)s->widget, (uint64_t)off, size, val);
  }

  /* SHDATA_IO6_P1: after 0xa0 the next 16-bit store is program data at the
   * raw byte offset; the flash can only clear bits, so AND it in (BE order). */
  if (s->ioprom_program) {
    if (off + 1 < SGI_BASEIO_IOPROM_SIZE) {
      s->ioprom[off] &= (val >> 8) & 0xff;
      s->ioprom[off + 1] &= val & 0xff;
    } else if (off < SGI_BASEIO_IOPROM_SIZE) {
      s->ioprom[off] &= b;
    }
    s->ioprom_program = 0;
    return;
  }
  if (off == 0 && b == 0xf0) { /* reset to read mode */
    s->ioprom_autoselect = 0;
    s->ioprom_unlock = 0;
    s->ioprom_program = 0;
    s->ioprom_erase = 0;
    return;
  }
  if (off == 0xaaaa && b == 0xaa) {
    s->ioprom_unlock = 1;
    return;
  }
  if (off == 0x5554 && b == 0x55 && s->ioprom_unlock == 1) {
    s->ioprom_unlock = 2;
    return;
  }
  if (off == 0xaaaa && s->ioprom_unlock == 2 && b == 0x90) {
    s->ioprom_autoselect = 1;
    s->ioprom_erase = 0;
    s->ioprom_unlock = 0;
    return;
  }
  if (off == 0xaaaa && s->ioprom_unlock == 2 && b == 0xa0) {
    s->ioprom_program = 1;
    s->ioprom_erase = 0;
    s->ioprom_unlock = 0;
    return;
  }
  if (off == 0xaaaa && s->ioprom_unlock == 2 && b == 0x80) {
    s->ioprom_erase = 1;
    s->ioprom_program = 0;
    s->ioprom_unlock = 0;
    return;
  }
  if (s->ioprom_erase) {
    if (b == 0x30) {
      /* Erase the 64 KiB block containing the target byte (the flasher also
       * issues sub-block addresses in the top block; over-erasing to 0xff is
       * harmless -- the tool programs the file immediately after). */
      uint64_t sec = off & ~(uint64_t)0xffff;
      if (sec < SGI_BASEIO_IOPROM_SIZE) {
        memset(s->ioprom + sec, 0xff,
               MIN((uint64_t)0x10000, SGI_BASEIO_IOPROM_SIZE - sec));
      }
    }
    return;
  }
  s->ioprom_unlock = 0;
}

static const MemoryRegionOps sgi_baseio_ioprom_ops = {
  .read = sgi_baseio_ioprom_read,
  .write = sgi_baseio_ioprom_write,
  .endianness = DEVICE_BIG_ENDIAN,
  .valid = { .min_access_size = 1, .max_access_size = 2 },
  .impl = { .min_access_size = 1, .max_access_size = 2 },
};

static bool sgi_baseio_rtcdbg(void) {
  static int dbg = -1;

  if (dbg < 0) {
    dbg = getenv("IP27_RTC_DBG") != NULL;
  }
  return dbg;
}

static uint64_t sgi_baseio_read(void *opaque, hwaddr off, unsigned size) {
  SGIBaseIOState *s = opaque;

  if (s->win_dbg && off >= 0x400000ULL) {
    qemu_log_mask(LOG_GUEST_ERROR,
                  "baseio-win[%u] R off=0x%" HWADDR_PRIx " size=%u\n",
                  (unsigned)s->widget, off, size);
  }

  /*
   * XIO widget identification, low word (offset 4): bits [27:12] are the
   * widget part number; 0xc002 identifies a Bridge.
   */
  if (off == 4) {
    return (uint64_t)((SGI_BASEIO_WIDGET_PART << 12) |
                      (SGI_BASEIO_WIDGET_MFGR << 1));
  }
  /*
   * Widget control (WIDGET_CONTROL = 0x24): low nibble is the board's XIO
   * widget id (WIDGET_WIDGET_ID).  The IRIX kernel's iograph reads this at
   * SWIN widget 0 to derive basew_id (ml/SN/iograph.c:849) and then addresses
   * the bridge's PCI devices at that widget -- so the discovery alias must
   * report the real board's id (see the wid_id property).
   */
  if (off == 0x24) {
    uint32_t id = (s->wid_id == UINT32_MAX) ? s->widget : s->wid_id;
    return (uint64_t)(id & 0xf);
  }
  /*
   * Widget status (WIDGET_STATUS = 0x0c): bit 5 is the PCI/GIO mode select
   * (BRIDGE_STAT_PCI_GIO_N).  bridge_discover (libkl/ml/iodiscover.c:410)
   * only proceeds to PCI discovery -- and thus to init_klcfg_ioc3, creating
   * the BaseIO lboard + KLSTRUCT_IOC3 component -- when this bit says the
   * bridge has a PCI interface.
   */
  if (off == 0x0c) {
    return 0x20;
  }
  /*
   * Bridge free-running counter/timer at +0x100.  The PROM reads it as the
   * timeout source for its wait-for-value loops (e.g. the poll in the IP27
   * PROM at VA 0x...1fc58490: it reads this, computes count/10*8, and spins
   * until the value changes).  Returning 0 makes every timeout instantly
   * expire-or-never-expire and spins forever, so return a value that advances
   * with the virtual clock.  Bit 0 is the enable (set by the PROM via
   * `ld; ori 1; sd`).
   */
  if (off == 0x100) {
    uint32_t c = (uint32_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000);
    return (c << 1) | (s->timer_en & 1);
  }
  /* Bridge MicroLAN control register (1-wire): board manufacturing record. */
  if (off == 0xb4) {
    return sgi_baseio_mcr_read(&s->ds_board);
  }
  /*
   * IOC3 register block at bridge+0x200000.  Its MCR (0x200030) is a second
   * 1-wire controller, for the IOC3's MAC-address EEPROM.
   */
  if (off == SGI_BASEIO_IOC3_MCR) {
    return sgi_baseio_mcr_read(&s->ds_mac);
  }
  /* IOC3 SuperIO control register: bus arbiter idle. */
  if (off == SGI_BASEIO_IOC3_SIO_CR) {
    return 0x00400000; /* SIO_CR_ARB_DIAG_IDLE */
  }
  /*
   * IOC3 SuperIO interrupt registers (sys/PCI/ioc3.h: SIO_IR 0x01c, IES 0x020,
   * IEC 0x024).  We drain the serial TX synchronously, so SIO_IR reports
   * SA_TX_MT (TX empty) at all times -- ioc3_wrflush() spins on that bit and
   * the console output path stalls without it.  IES/IEC are a set/clear pair
   * over ONE enable register and BOTH offsets read back the current mask; the
   * driver reads the enable at 0x20, so storing only the written offset leaves
   * it stale and the interrupt dispatcher retries forever.
   */
  if (off == 0x20001c) {
    /*
     * SIO_IR source status.  SA_TX_MT is always set (synchronous TX drain) so
     * ioc3_wrflush()'s poll exits; the RX bits are latched by the RX engine and
     * cleared by the driver writing them back (write-1-to-clear).
     */
    return s->sio_ir_pending | SGI_BASEIO_SIO_IR_SA_TX_MT;
  }
  if (off == 0x200020 || off == 0x200024) {
    return s->sio_ienb;
  }
  /* IOC3 serial DMA ring base (SBBR) and per-port producer/consumer. */
  if (off == 0x2000b0) {
    return s->sbbr_h;
  }
  if (off == 0x2000b4) {
    return s->sbbr_l;
  }
  if (off == 0x2000bc) {
    return s->stpir[0];
  }
  if (off == 0x2000c0) {
    return s->stcir[0];
  }
  if (off == 0x2000d8) {
    return s->stpir[1];
  }
  if (off == 0x2000dc) {
    return s->stcir[1];
  }
  /* IOC3 serial DMA RX half: hardware producer, guest consumer, RX timer. */
  if (off == 0x2000c4) {
    return s->srpir;
  }
  if (off == 0x2000c8) {
    return s->srcir;
  }
  if (off == 0x2000cc) {
    return s->srtr;
  }
  /*
   * IOC3 serial DMA control (SSCR_A 0xb8 / SSCR_B 0xd4).  SSCR_PAUSE_STATE
   * reflects SSCR_DMA_PAUSE: ioc3_open() asserts DMA_PAUSE and spins until
   * PAUSE_STATE reads back set, so without this the console open times out and
   * the serial console goes silent right after the banner.  The reset /
   * RX-drain command bits are self-clearing.
   */
  if (off == 0x2000b8 || off == 0x2000d4) {
    int port = (off == 0x2000d4);
    uint32_t v = s->sscr[port] & ~0x88000000u; /* RX_DRAIN|RESET self-clear */

    if (v & 0x20000000u) { /* SSCR_DMA_PAUSE */
      v |= 0x40000000u;    /* SSCR_PAUSE_STATE */
    }
    return v;
  }
  /*
   * IOC3 GenericPIO block: GPCR (control; set at +0x34, clear at +0x38) and
   * GPDR (data, +0x3c).  The PROM drives a PHY reset through GPCR; serve the
   * latched control/data.
   */
  if (off == 0x200034 || off == 0x200038) {
    return s->ioc3_gpcr;
  }
  if (off == 0x20003c) {
    return s->ioc3_gpdr;
  }
  if (off == 0x20002c) {
    return 0; /* second SuperIO register; unused */
  }
  /* IOC3 Ethernet MAC register file + MII management. */
  if (off >= SGI_BASEIO_ETH_OFF &&
      off < SGI_BASEIO_ETH_OFF + SGI_BASEIO_ETH_SIZE) {
    unsigned idx = (off - SGI_BASEIO_ETH_OFF) >> 2;
    uint64_t v = s->eth_regs[idx];

    if (idx == SGI_IOC3_EMCR) {
      /* The ef driver polls ARB_DIAG_IDLE after asserting RST. */
      v |= 0x00200000; /* IOC3_EMCR_ARB_DIAG_IDLE */
    } else if (idx == SGI_IOC3_MICR) {
      v &= ~0x00000800; /* management ops complete instantly (BUSY clear) */
    } else if (idx == SGI_IOC3_MIDR_R) {
      v = s->phy_read_data & 0xffff;
    } else if (idx == SGI_IOC3_ETCIR) {
      if (sgi_baseio_txtrace < 0) {
        sgi_baseio_txtrace = getenv("SGIBASEIO_TXTRACE") != NULL;
      }
      if (sgi_baseio_txtrace) {
        printf("[TXTRACE] ETCIR read=0x%x\n", (unsigned)v);
      }
    }
    return v;
  }
  /*
   * IOC3 SuperIO UART B (16550, byte-spaced) at SIO+0x170.  UART A
   * (0x220178) is a separate region; UART B is otherwise unmodelled and the
   * PROM polls its LSR (offset 5), so return THRE|TEMT to let the poll exit.
   */
  if (off >= 0x220170 && off < 0x220178) {
    switch (off - 0x220170) {
    case 2: return 0x01;  /* IIR: no interrupt pending */
    case 5: return 0x60;  /* LSR: THRE | TEMT */
    default: return 0;
    }
  }
  /*
   * PCI config slots: the Bridge exposes each PCI device's config dword 0 at
   * bridge+0x20000 + slot*0x1000 (BRIDGE_TYPE0_CFG_DEV(slot); see ARCS
   * include/pci/bridge.h).  ARCS read_pcilink_status/iodiscover.c pci_discover
   * read the 32-bit pci_id here (device id in [31:16], vendor in [15:0]) and
   * switch on it to build the KL config components:
   *   slot 0    IOC3   (device 0x0003, vendor 0x10a9) -> KLSTRUCT_IOC3
   *   slot 1,2  QLogic ISP1020 (device 0x1020, vendor 0x1077) ->
   *             KLSTRUCT_SCSI.  ARCS expects at least two QLogic devices on a
   *             BaseIO (diag_io6config.c: io6confSpace_sanity, "expected 2 or
   *             more").  With no SCSI component, sn0_dump_diag calls
   *             dump_scsi_diags(NULL), which reads scsi->scsi_info.diagval
   *             before its NULL check -- the observed POD TLB refill.
   *   other     no device (0xffffffff).
   */
  if (off >= 0x20000 && off < 0x30000) {
    unsigned slot, func, cfg;

    /*
     * 0x28000..0x30000 is Type-1 configuration space (BRIDGE_TYPE1_CFG): no bus
     * exists behind the bridge, so every read must return all-ones (vendor
     * 0xffff = "no device").  Returning 0 instead makes the kernel believe a
     * device sits on a phantom bus and walk it forever.
     */
    if (off >= 0x28000) {
      return 0xffffffff;
    }
    slot = (off - 0x20000) >> 12;
    func = ((off - 0x20000) >> 8) & 0xf;
    cfg = off & 0xff;
    /* The BaseIO devices are single-function; absent functions read no device. */
    if (func != 0) {
      return 0xffffffff;
    }

    if (slot == 0) {
      /* IOC3: vendor 0x10a9 / device 0x0003.  Other config dwords reflect
       * what the PROM wrote (command/latency/BARs) or read 0. */
      return (cfg == 0x00) ? 0x000310a9 : s->pci_cfg0[(cfg >> 2) & 0x3f];
    }
    if (slot == 1 || slot == 2) {
      /*
       * QLogic ISP1020 (SGI_QLISP device) on PCI slots 1,2.  The ISP register
       * file is 16-bit big-endian with a byte-lane swap, so a config dword
       * reads back halves swapped: a 16-bit access at cfg 0x00 returns the
       * device id and one at 0x02 the vendor id.  ql_init_board reads vendor
       * from struct offset 2 and device from offset 0, so mirror that here
       * (same convention as the IP30 bridge).
       */
      SGIQLispState *isp = &s->isp[slot - 1];

      switch (cfg) {
      case 0x00:
        return (size == 2) ? QLISP_DEVICE
                           : ((QLISP_DEVICE << 16) | QLISP_VENDOR);
      case 0x02:
        return QLISP_VENDOR;
      case 0x04:
        return isp->pci_cmd;
      case 0x06:
        return 0; /* status */
      case 0x08:
        return (QLISP_CLASS << 8) | isp->pci_rev;
      case 0x0c:
        return 0; /* header type 0, single function */
      case QLISP_PCI_BAR0_OFF:
      case QLISP_PCI_BAR1_OFF:
        {
          uint32_t bar = sgi_qlisp_pci_config_read(isp, cfg);

          if (getenv("SGI_BASEIO_PCILOG"))
            fprintf(stderr, "sgi-baseio: PCI READ slot=%u cfg=0x%x bar=0x%x\n",
                    (unsigned)slot, cfg, bar);
          return bar;
        }
      default:
        return 0;
      }
    }
    /* Absent slot: all-ones so the kernel sees no device. */
    return 0xffffffff;
  }
  /*
   * Bridge interrupt registers (sys/PCI/bridge.h): status 0x104, enable
   * 0x10c, reset-status 0x114, mode 0x11c, device 0x124, host-error 0x12c,
   * and one address register per device line at 0x134 + n*8.  pcibr reads
   * status to identify the asserting line; the address register holds the
   * host|vector the bridge sends to the hub.  The IOC3 Ethernet line (bit 0)
   * is maintained in int_line by sgi_baseio_eth_irq_sync, so status is a
   * faithful mirror of the asserted device lines (and falls when the driver
   * clears EISR).
   */
  if (off == 0x104) {
    uint32_t st = s->int_line;
    if (sgi_baseio_intdbg()) {
      qemu_log_mask(LOG_UNIMP,
                    "BASEIO rd INT_STATUS -> 0x%x (line=0x%x enable=0x%x "
                    "delivered=%d,%d,%d,%d,%d,%d,%d,%d)\n",
                    st, s->int_line, s->int_enable,
                    s->int_delivered[0], s->int_delivered[1],
                    s->int_delivered[2], s->int_delivered[3],
                    s->int_delivered[4], s->int_delivered[5],
                    s->int_delivered[6], s->int_delivered[7]);
    }
    return st;
  }
  if (off == 0x10c) {
    return s->int_enable;
  }
  if (off == 0x114) {
    return s->int_rst_stat;
  }
  if (off == 0x11c) {
    return s->int_mode;
  }
  if (off == 0x124) {
    return s->int_device;
  }
  if (off == 0x12c) {
    return s->int_host_err;
  }
  if (off >= 0x134 && off < 0x174 && ((off - 0x134) & 7) == 0) {
    return s->int_addr[(off - 0x134) >> 3];
  }
  /*
   * IOC3 byte-bus window at bridge+0x280000 (IOC3_BYTEBUS_DEV0 = 0x80000
   * within the IOC3; DEV0..3 span 512 KB).  This is the DS1386-class
   * timekeeping NVRAM: its low 0xe bytes are the clock registers (handled
   * below) and the remainder is battery-backed NVRAM.  On IP27 ARCS
   * size_bridge_ssram() writes a size marker at [0]/[64k]/[128k] and reads it
   * back to pick the fitted size, then runs an alternating-pattern memory test
   * over the array.  Back the NVRAM with storage so those accesses persist;
   * without it they read back 0, the part looks absent, and the kernel loops
   * forever.
   */
  if (off >= SGI_BASEIO_BR_SSRAM_OFF + SGI_BASEIO_RTC_LEN &&
      off < SGI_BASEIO_BR_SSRAM_OFF + SGI_BASEIO_BR_SSRAM_SIZE) {
    hwaddr o = off - SGI_BASEIO_BR_SSRAM_OFF;
    unsigned n;
    uint64_t v = 0;
    for (n = 0; n < size; n++) {
      v |= (uint64_t)s->br_ssram[o + n] << (8 * n);
    }
    if (sgi_baseio_rtcdbg() && o < 0x20) {
      fprintf(stderr, "BASEIO-RTC-DBG ssram R off=0x%" HWADDR_PRIx
              " o=0x%" HWADDR_PRIx " size=%u -> 0x%" PRIx64 "\n",
              off, o, size, v);
    }
    return v;
  }
  /* Bridge internal ATE RAM (bridge+0x10000..0x103ff); see the write side. */
  if (off >= SGI_BASEIO_BR_ATE_OFF &&
      off < SGI_BASEIO_BR_ATE_OFF + sizeof(s->ate_ram)) {
    hwaddr o = off - SGI_BASEIO_BR_ATE_OFF;
    unsigned n;
    uint64_t v = 0;

    for (n = 0; n < size; n++) {
      v |= (uint64_t)s->ate_ram[o + n] << (8 * n);
    }
    return v;
  }
  /* IOC3 byte-bus clock registers (Dallas DS1386) at bridge+0x280000. */
  if (off >= SGI_BASEIO_RTC_OFF && off < SGI_BASEIO_RTC_OFF + SGI_BASEIO_RTC_LEN) {
    uint64_t v = sgi_baseio_tod_read(s, off - SGI_BASEIO_RTC_OFF);

    if (sgi_baseio_rtcdbg()) {
      fprintf(stderr, "BASEIO-RTC-DBG sio R off=0x%" HWADDR_PRIx
              " size=%u -> 0x%" PRIx64 "\n", off, size, v);
    }
    return v;
  }
  qemu_log_mask(LOG_UNIMP,
                "sgi-baseio: unimplemented read @0x%" HWADDR_PRIx
                " (size %u)\n",
                off, size);
  return 0;
}

static void sgi_baseio_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size) {
  SGIBaseIOState *s = opaque;

  if (s->win_dbg &&
      (off >= 0x400000ULL /* high window incl. flash/QLISP */
       || (size <= 2 && ((val & 0xff) == 0xaa || (val & 0xff) == 0x55 ||
                         (val & 0xff) == 0x90 || (val & 0xff) == 0xf0 ||
                         (val & 0xff) == 0xa0 || (val & 0xff) == 0x80)))) {
    qemu_log_mask(LOG_GUEST_ERROR,
                  "baseio-win[%u] W off=0x%" HWADDR_PRIx " size=%u val=0x%"
                  PRIx64 "\n", (unsigned)s->widget, off, size, (uint64_t)val);
  }

  if (off == 0xb4) {
    sgi_baseio_mcr_write(&s->ds_board, val);
    return;
  }
  if (off == SGI_BASEIO_IOC3_MCR) {
    sgi_baseio_mcr_write(&s->ds_mac, val);
    return;
  }
  /* IOC3 SuperIO control/data registers (SIO_CR 0x200028, +0x2c): accepted. */
  if (off == SGI_BASEIO_IOC3_SIO_CR || off == 0x20002c) {
    return;
  }
  /*
   * IOC3 SuperIO interrupt-enable set/clear pair (ioc3.h: IES 0x020 sets,
   * IEC 0x024 clears the bits of one register).  Both offsets read back the
   * same mask (handled in the read path).
   */
  if (off == 0x200020) {
    s->sio_ienb |= val;
    /* Enabling the RX interrupts is the console driver owning the ring. */
    if (val & (SGI_BASEIO_SIO_IR_SA_RX_HIGH | SGI_BASEIO_SIO_IR_SA_RX_TIMER)) {
      s->rx_kernel = true;
    }
    sgi_baseio_ioc3_irq_sync(s);
    return;
  }
  if (off == 0x200024) {
    s->sio_ienb &= ~val;
    sgi_baseio_ioc3_irq_sync(s);
    return;
  }
  /*
   * Bridge PCI-interrupt registers: latch b_int_enable / b_int_addr and
   * re-deliver; absorb the mode/device/host-error/reset-status registers the
   * kernel programs (int_rst_stat is write-1-to-clear and we hold no error
   * bits).
   */
  if (off == 0x10c) {
    s->int_enable = val;
    sgi_baseio_int_sync(s);
    return;
  }
  if (off == 0x114) {
    s->int_rst_stat &= ~val;
    return;
  }
  if (off == 0x11c) {
    s->int_mode = val;
    return;
  }
  if (off == 0x124) {
    s->int_device = val;
    return;
  }
  if (off == 0x12c) {
    s->int_host_err = val;
    return;
  }
  if (off >= 0x134 && off < 0x174 && ((off - 0x134) & 7) == 0) {
    s->int_addr[(off - 0x134) >> 3] = val;
    sgi_baseio_int_sync(s);
    return;
  }
  /* IOC3 serial DMA ring: latch the base (SBBR) and drain on a STPIR write. */
  if (off == 0x2000b0) {
    s->sbbr_h = val;
    return;
  }
  if (off == 0x2000b4) {
    s->sbbr_l = val;
    return;
  }
  if (off == 0x2000bc || off == 0x2000d8) {
    int port = (off == 0x2000d8);

    s->stpir[port] = val;
    sgi_baseio_sio_tx_drain(s, port);
    return;
  }
  /* IOC3 serial DMA control: latch SSCR, clearing the self-clearing cmd bits. */
  if (off == 0x2000b8 || off == 0x2000d4) {
    int port = (off == 0x2000d4);

    s->sscr[port] = val & ~0x88000000u;
    return;
  }
  /* IOC3 serial DMA RX half: latch the guest consumer (SRCIR, written with
   * SRCIR_ARM) and the RX timer interval.  The hardware producer SRPIR is
   * advanced by sgi_baseio_sio_rx(). */
  if (off == 0x2000c8) {
    /* SRCIR_ARM marks the kernel's console reader (ioc3_read re-arms here). */
    if (val & 0x80000000u) {
      s->rx_kernel = true;
    }
    s->srcir = val;
    return;
  }
  if (off == 0x2000cc) {
    s->srtr = val;
    return;
  }
  /*
   * SIO_IR (0x1c) is write-1-to-clear: the driver's ISR acks its source bits
   * (e.g. RX_TIMER) by writing them back.  Without clearing the latched bit the
   * IOC3 line stays asserted and the ISR livelocks.
   */
  if (off == 0x20001c) {
    s->sio_ir_pending &= ~val;
    sgi_baseio_ioc3_irq_sync(s);
    return;
  }
  /* IOC3 GenericPIO block: GPCR set (+0x34) / clear (+0x38), GPDR (+0x3c). */
  if (off == 0x200034) {
    s->ioc3_gpcr |= val;
    return;
  }
  if (off == 0x200038) {
    s->ioc3_gpcr &= ~val;
    return;
  }
  if (off == 0x20003c) {
    s->ioc3_gpdr = val;
    return;
  }
  /*
   * PCI config-space writes through the bridge window (0x20000 + slot*0x1000).
   * Slot 0 is the IOC3: latch command/latency/BARs so reads reflect them.
   */
  if (off >= 0x20000 && off < 0x30000) {
    unsigned slot, func, cfg;

    /* Type-1 (0x28000..): no bus behind the bridge; writes go nowhere. */
    if (off >= 0x28000) {
      return;
    }
    slot = (off - 0x20000) >> 12;
    func = ((off - 0x20000) >> 8) & 0xf;
    cfg = off & 0xff;
    if (func != 0) {
      return;                     /* the BaseIO devices are single-function */
    }
    if (getenv("SGI_BASEIO_PCILOG") && (slot == 1 || slot == 2) &&
        (cfg == QLISP_PCI_BAR0_OFF || cfg == QLISP_PCI_BAR1_OFF)) {
      fprintf(stderr, "sgi-baseio: PCI WRITE slot=%u cfg=0x%x val=0x%x\n",
              slot, cfg, (unsigned)val);
    }
    if (slot == 0) {
      /*
       * IOC3: latch command/latency/BARs so config reads reflect them.  The
       * two memory BARs decode 1 MB each, matching the bridge DevIO layout
       * (and octane's sgi_bridge): the all-ones sizing probe must read back a
       * size mask, not 0xffffffff, or the guest computes a bogus BAR size and
       * the ioc3 driver reports "unable to get PIO mapping for my MEM space".
       */
      if (cfg == 0x10 || cfg == 0x14) {
        s->pci_cfg0[cfg >> 2] = val & 0xfff00000u;
      } else if (cfg <= 0xfc) {
        s->pci_cfg0[cfg >> 2] = val;
      }
    } else if (slot == 1 || slot == 2) {
      /* QLogic ISP1020: latch the BARs (per-BAR sizing contract). */
      sgi_qlisp_pci_config_write(&s->isp[slot - 1], cfg, val);
    }
    return;
  }
  /* IOC3 SuperIO UART B (see the read side): accept writes, no model. */
  if (off >= 0x220170 && off < 0x220178) {
    return;
  }
  /* IOC3 Ethernet MAC register file + MII management. */
  if (off >= SGI_BASEIO_ETH_OFF &&
      off < SGI_BASEIO_ETH_OFF + SGI_BASEIO_ETH_SIZE) {
    unsigned idx = (off - SGI_BASEIO_ETH_OFF) >> 2;

    if (idx == SGI_IOC3_EMCR) {
      uint32_t old = s->eth_regs[idx];

      /* RST and the idle status bit are handled, not stored. */
      s->eth_regs[idx] = val & ~(0x80000000u | 0x00200000u);
      if (val & 0x80000000u) {
        s->eth_rxprod = 0;
        s->eth_txcons = 0;
        memset(s->eth_regs, 0, sizeof(s->eth_regs));
        sgi_baseio_eth_irq_sync(s);
      } else if (!(old & 0x00010000u) && (val & 0x00010000u) && s->nic) {
        /*
         * RXEN 0->1: the ef driver clears RXEN in ef_close and re-enables it
         * on the next open; while it was clear can_receive() was false so QEMU
         * queued (rather than dropped) incoming frames.  Flush them now or the
         * frame that arrived during the close is never delivered.
         */
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
      }
    } else if (idx == SGI_IOC3_EISR) {
      /*
       * EISR is write-1-to-clear: both the kernel if_ef and the ARCS ef driver
       * read it and write the read value back to clear exactly those bits
       * (if_ef.c ef_intr).  Storing the value instead would leave every
       * acknowledged status bit set, holding bridge line 0 and vector 11
       * asserted forever.
       */
      s->eth_regs[idx] &= ~val;
      sgi_baseio_eth_irq_sync(s);
    } else if (idx == SGI_IOC3_EIER) {
      /*
       * EIER is the interrupt enable mask; the ef driver enables all but
       * TXEMPTY in ef_init.  The Ethernet line follows EISR & EIER.
       */
      s->eth_regs[idx] = val;
      sgi_baseio_eth_irq_sync(s);
    } else if (idx == SGI_IOC3_ETPIR) {
      s->eth_regs[idx] = val;
      if (sgi_baseio_txtrace < 0) {
        sgi_baseio_txtrace = getenv("SGIBASEIO_TXTRACE") != NULL;
      }
      if (sgi_baseio_txtrace) {
        printf("[TXTRACE] ETPIR write=0x%x (cons=0x%x)\n",
               (unsigned)(val & 0xffff), (unsigned)s->eth_txcons);
      }
      sgi_baseio_eth_tx_drain(s);
    } else if (idx == SGI_IOC3_MIDR_W) {
      s->phy_write_data = val & 0xffff;
      s->eth_regs[idx] = val;
    } else if (idx == SGI_IOC3_MICR) {
      unsigned reg = val & 0x1f;

      if (val & 0x400) { /* READTRIG: latch the addressed PHY register. */
        s->phy_read_data = s->phy_regs[reg];
      } else {
        /* Write cycle.  The PROM diag and the ef driver address the PHY at
         * 0x1f (SGI_PHY_ADDR); accept writes at any address. */
        s->phy_regs[reg] = s->phy_write_data;
        /* BMCR (reg 0) bit 15 is a self-clearing soft reset. */
        if (reg == 0) {
          s->phy_regs[0] &= ~0x8000;
        }
      }
      s->eth_regs[idx] = val & ~0x800u; /* BUSY stays clear */
    } else {
      s->eth_regs[idx] = val;
    }
    return;
  }
  /* Bridge timer: bit0 = enable (see the read side). */
  if (off == 0x100) {
    s->timer_en = val & 1;
    return;
  }
  /* IOC3 byte-bus time-of-day chip (Dallas DS1386). */
  /*
   * IOC3 byte-bus NVRAM window (see the read side).
   * Writes land in the backing storage so the ARCS size probe and pattern test
   * hold.
   */
  if (off >= SGI_BASEIO_BR_SSRAM_OFF + SGI_BASEIO_RTC_LEN &&
      off < SGI_BASEIO_BR_SSRAM_OFF + SGI_BASEIO_BR_SSRAM_SIZE) {
    hwaddr o = off - SGI_BASEIO_BR_SSRAM_OFF;
    unsigned n;

    if (sgi_baseio_rtcdbg() && o < 0x20) {
      fprintf(stderr, "BASEIO-RTC-DBG ssram W off=0x%" HWADDR_PRIx
              " o=0x%" HWADDR_PRIx " size=%u val=0x%" PRIx64 "\n",
              off, o, size, (uint64_t)val);
    }
    for (n = 0; n < size; n++) {
      s->br_ssram[o + n] = (val >> (8 * n)) & 0xff;
    }
    return;
  }
  /*
   * Bridge internal ATE RAM (bridge+0x10000..0x103ff).  The kernel fills the
   * address-translation entries for the ISP's ATE-mapped DMA window
   * (BRIDGE_DMA_MAPPED_BASE 0x40000000); sgi_baseio_dma_xlate() reads them back
   * to translate qlisp ring/SG DMA addresses.  Back with plain storage.
   */
  if (off >= SGI_BASEIO_BR_ATE_OFF &&
      off < SGI_BASEIO_BR_ATE_OFF + sizeof(s->ate_ram)) {
    hwaddr o = off - SGI_BASEIO_BR_ATE_OFF;
    unsigned n;

    for (n = 0; n < size; n++) {
      s->ate_ram[o + n] = (val >> (8 * n)) & 0xff;
    }
    return;
  }
  /* IOC3 byte-bus clock registers (Dallas DS1386) at bridge+0x280000. */
  if (off >= SGI_BASEIO_RTC_OFF && off < SGI_BASEIO_RTC_OFF + SGI_BASEIO_RTC_LEN) {
    if (sgi_baseio_rtcdbg()) {
      fprintf(stderr, "BASEIO-RTC-DBG sio W off=0x%" HWADDR_PRIx
              " size=%u val=0x%" PRIx64 "\n", off, size, (uint64_t)val);
    }
    sgi_baseio_tod_write(s, off - SGI_BASEIO_RTC_OFF, val);
    return;
  }
  qemu_log_mask(LOG_UNIMP,
                "sgi-baseio: unimplemented write @0x%" HWADDR_PRIx
                " = 0x%" PRIx64 " (size %u)\n",
                off, val, size);
}

/*
 * IOC3 SSRAM diagnostic region.  Word = 16 data bits (0..15) + parity bit (16);
 * a read returns the parity-error bit (17) set when the stored data's parity
 * disagrees with the stored parity bit.  The in-band write control bit (17) and
 * the read error bit share bit 17; writers ignore what we return for it (the ef
 * driver masks reads with IOC3_SSRAM_DM = 0xffff), while the PROM enet_ssram
 * diagnostic checks it explicitly.
 */
static uint64_t sgi_baseio_ssram_read(void *opaque, hwaddr off, unsigned size) {
  SGIBaseIOState *s = opaque;
  uint32_t v = s->ssram[(off >> 2) & (SGI_BASEIO_IOC3_SSRAM_WORDS - 1)];
  uint32_t data = v & 0xffff;
  uint32_t par = (v >> 16) & 1;
  uint32_t err = (__builtin_parity(data) ^ par) & 1;

  return data | (par << 16) | (err << 17);
}

static void sgi_baseio_ssram_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size) {
  SGIBaseIOState *s = opaque;
  s->ssram[(off >> 2) & (SGI_BASEIO_IOC3_SSRAM_WORDS - 1)] = val & 0x1ffff;
}

static const MemoryRegionOps sgi_baseio_ssram_ops = {
    .read = sgi_baseio_ssram_read,
    .write = sgi_baseio_ssram_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 4,
        },
    .impl =
        {
            .min_access_size = 4,
            .max_access_size = 4,
        },
};

static const MemoryRegionOps sgi_baseio_ops = {
    .read = sgi_baseio_read,
    .write = sgi_baseio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

/*
 * IOC3 SuperIO UART A: byte-spaced 16550 (reg == offset; the IP27 PROM reads
 * LSR at +5 and writes THR at +0).
 */
static uint64_t sgi_baseio_uart_read(void *opaque, hwaddr addr, unsigned size) {
  SGIBaseIOState *s = opaque;
  return serial_io_ops.read(&s->ioc3_uart, addr, size);
}

static void sgi_baseio_uart_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size) {
  SGIBaseIOState *s = opaque;
  serial_io_ops.write(&s->ioc3_uart, addr, value, size);
}

static const MemoryRegionOps sgi_baseio_uart_ops = {
    .read = sgi_baseio_uart_read,
    .write = sgi_baseio_uart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 1,
        },
};

static void sgi_baseio_reset(DeviceState *dev) {
  SGIBaseIOState *s = SGI_BASEIO(dev);
  int i;

  s->int_enable = 0;
  s->int_line = 0;
  s->int_rst_stat = 0;
  s->int_mode = 0;
  s->int_device = 0;
  s->int_host_err = 0;
  memset(s->int_addr, 0, sizeof(s->int_addr));
  memset(s->ate_ram, 0, sizeof(s->ate_ram));
  for (i = 0; i < 8; i++) {
    s->int_delivered[i] = -1;
  }
  /*
   * We drain the serial TX synchronously, so SA_TX_MT is reported on every
   * SIO_IR read rather than latched here.  Only the RX condition bits live in
   * sio_ir_pending (and feed the interrupt level).
   */
  s->sio_ir_pending = 0;
  s->srpir = 0;
  s->srcir = 0;
  s->srtr = 0;
  s->rx_kernel = false;

  sgi_baseio_ds_board_init(&s->ds_board);
  sgi_baseio_ds_reset(&s->ds_board);
  sgi_baseio_ds_mac_init(&s->ds_mac);
  sgi_baseio_ds_reset(&s->ds_mac);
  memset(s->eth_regs, 0, sizeof(s->eth_regs));
  s->eth_rxprod = 0;
  s->eth_txcons = 0;
  sgi_baseio_phy_init(s);

  /*
   * Start the time-of-day chip from the host's clock so it presents a valid,
   * advancing calendar from the first read (ml/clksupport.c warns otherwise).
   * CONTROL defaults to update-enable.
   */
  s->tod_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
  s->tod_epoch_sec = (int64_t)time(NULL);
  s->tod_control = RTC_DAL_UPDATE_ENABLE;
  s->tod_user = 0;
}

/*
 * Translate a BRIDGE ATE-mapped PCI DMA address to a system physical address.
 *
 * The IRIX ql driver programs the ISP request/response ring bases with
 * addresses from pciio_dmatrans_addr(), which fall in the BRIDGE ATE-mapped
 * PCI window (BRIDGE_DMA_MAPPED_BASE 0x40000000, IOPAGE 0x4000).  The guest
 * itself fills the bridge ATE RAM (bridge+0x10000), so read the ATE it wrote:
 *   ate  = ate_ram[((pci - 0x40000000) >> 14)]   (proto | port<<8 | xio pfn)
 *   phys = (ate & ~0x3fff) + (pci & 0x3fff)
 * Returns the input unchanged when the address is outside the window or the
 * ATE is not valid, so the qlisp falls back to its direct/K1 handling.  Same
 * contract as octane's sgi_bridge_dma_xlate().
 */
static uint64_t sgi_baseio_dma_xlate(void *arg, uint64_t pci_addr) {
  SGIBaseIOState *s = arg;
  uint32_t idx, i, w0, w1;
  uint64_t ate;

  if (pci_addr < 0x40000000ULL || pci_addr >= 0x80000000ULL) {
    return pci_addr;
  }
  idx = (pci_addr - 0x40000000ULL) >> 14;
  if (idx >= sizeof(s->ate_ram) / 8) { /* external ATEs not modelled */
    return pci_addr;
  }
  /*
   * The guest writes the 64-bit ATE as a big-endian store; QEMU splits it into
   * two 32-bit word accesses, which the register handler stores little-endian
   * per word.  Recombine the two words (first = high).
   */
  w0 = 0;
  w1 = 0;
  for (i = 0; i < 4; i++) {
    w0 |= (uint32_t)s->ate_ram[idx * 8 + i] << (8 * i);
    w1 |= (uint32_t)s->ate_ram[idx * 8 + 4 + i] << (8 * i);
  }
  ate = ((uint64_t)w0 << 32) | w1;
  if (!(ate & 0x01)) { /* ATE_V */
    return pci_addr;
  }
  if (getenv("SGI_BASEIO_ATEDBG")) {
    qemu_log_mask(LOG_UNIMP,
                  "sgi-baseio: xlate pci=0x%llx idx=%u ate=0x%llx -> phys=0x%llx\n",
                  (unsigned long long)pci_addr, idx,
                  (unsigned long long)ate,
                  (unsigned long long)((ate & ~0x3fffULL) +
                                       (pci_addr & 0x3fff)));
  }
  return (ate & ~0x3fffULL) + (pci_addr & 0x3fff);
}

static void sgi_baseio_realize(DeviceState *dev, Error **errp) {
  SGIBaseIOState *s = SGI_BASEIO(dev);
  Chardev *chr;
  int i;

  s->win_dbg = getenv("IP27_BASEIO_WINDBG") != NULL;

  /*
   * Bridge PCI-interrupt aggregation: eight device-line inputs (BaseIO
   * devices assert them) and one aggregate output the machine may route.  The
   * per-line vectors are delivered to the hub through the `hub` pointer.
   */
  qdev_init_gpio_in(dev, sgi_baseio_dev_irq, 8);
  qdev_init_gpio_out(dev, &s->int_out, 1);

  memory_region_init_io(&s->iomem, OBJECT(s), &sgi_baseio_ops, s, "sgi-baseio",
                        SGI_BASEIO_WINDOW_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

  /*
   * Connect the IOC3 UART to the launch chardev ("ser0" in the MCP harness,
   * "serial0" for a plain -serial).  Only the node's IO widget (8) owns the
   * real console; the widget-0 instance is the discovery alias.
   */
  chr = (s->widget == 8) ? qemu_chr_find("ser0") : NULL;
  if (s->widget == 8 && !chr) {
    chr = qemu_chr_find("serial0");
  }
  if (chr) {
    qdev_prop_set_chr(DEVICE(&s->ioc3_uart), "chardev", chr);
  }
  if (!qdev_realize(DEVICE(&s->ioc3_uart), NULL, errp)) {
    return;
  }
  if (chr) {
    /*
     * Own the chardev handlers on the UART's frontend so input feeds BOTH the
     * 16550 (PROM console) and the IOC3 SIO RX ring (kernel console).  The
     * UART's own serial_receive1 handlers cannot remain registered alongside
     * ours; we forward to it via serial_receive_bytes().
     */
    qemu_chr_fe_set_handlers(&s->ioc3_uart.chr, sgi_baseio_serial_can_receive,
                             sgi_baseio_serial_receive, sgi_baseio_serial_event,
                             NULL, s, NULL, true);
  }
  memory_region_init_io(&s->ioc3_uart_mr, OBJECT(s), &sgi_baseio_uart_ops, s,
                        "sgi-baseio-uart", SGI_BASEIO_IOC3_UART_SIZE);
  memory_region_add_subregion(&s->iomem, SGI_BASEIO_IOC3_UART,
                              &s->ioc3_uart_mr);

  /* IOC3 SSRAM diagnostic region (256 KB). */
  memory_region_init_io(&s->ssram_mr, OBJECT(s), &sgi_baseio_ssram_ops, s,
                        "sgi-baseio-ssram", SGI_BASEIO_IOC3_SSRAM_LEN);
  memory_region_add_subregion(&s->iomem, SGI_BASEIO_IOC3_SSRAM_OFF,
                              &s->ssram_mr);

  /*
   * IO6 boot flash.  Exposed on every BaseIO instance (the kernel's mmap of
   * the xtalk/pci/controller vertex can be backed by either the discovery
   * alias or the real IO widget).  Blank NOR reads 0xff.  See the
   * access-pattern notes above sgi_baseio_ioprom_read.
   */
  {
    memset(s->ioprom, 0xff, SGI_BASEIO_IOPROM_SIZE);
    s->ioprom_dbg = getenv("SGIBASEIO_IOPROM_DBG") != NULL;
    memory_region_init_io(&s->ioprom_mr, OBJECT(s), &sgi_baseio_ioprom_ops, s,
                          "sgi-baseio-ioprom", SGI_BASEIO_IOPROM_SIZE);
    memory_region_add_subregion(&s->iomem, SGI_BASEIO_IOPROM_OFF,
                                &s->ioprom_mr);
  }

  /*
   * On-board QLogic ISP1020 SCSI channels.  The children must always be
   * realized (qdev asserts on unrealized children); their register windows are
   * exposed only on the node's IO widget (8) -- the widget-0 instance is the
   * discovery alias.  At BaseIO QLogic DevIO offsets (mem_base 0x08400000 /
   * 0x08600000).  Firmware/drive binding is the SGI_QLISP device's (shared with
   * octane).
   */
  for (i = 0; i < 2; i++) {
    /*
     * Give the node's IO widget (8) the SCSI bus numbers 0/1 so legacy
     * -drive if=scsi,bus=N,unit=M drives attach here; the widget-0 alias
     * claims no bus number so it never steals a drive from the real ones.
     */
    object_property_set_uint(OBJECT(&s->isp[i]), "scsi-bus-num",
                             s->widget == 8 ? (uint32_t)i : 0xffffffffu,
                             &error_abort);
    if (!qdev_realize(DEVICE(&s->isp[i]), NULL, errp)) {
      return;
    }
    /*
     * Let the ISP translate its ring/SG DMA addresses through this bridge's
     * ATE RAM (BRIDGE_DMA_MAPPED_BASE).  The standalone driver's 64-bit
     * dirmap addresses do not fall in that window, so they are unaffected.
     */
    s->isp[i].dma_xlate = sgi_baseio_dma_xlate;
    s->isp[i].dma_xlate_arg = s;
  }
  if (s->widget == 8) {
    memory_region_add_subregion(&s->iomem, SGI_BASEIO_QLISP0_OFF,
                                &s->isp[0].regs);
    memory_region_add_subregion(&s->iomem, SGI_BASEIO_QLISP1_OFF,
                                &s->isp[1].regs);
    /*
     * The kernel pcibr packs the ISP memory BAR into the 2 MB DevIO1 window at
     * +0x100000, reaching channel 1 at 0x500000.  Expose the same register file
     * there as an alias (a MemoryRegion can only have one parent).
     */
    memory_region_init_alias(&s->isp1_kern_alias_mr, OBJECT(s),
                             "sgi-baseio-qlisp1-kern-alias",
                             &s->isp[1].regs, 0, QLISP_REGS_SIZE);
    memory_region_add_subregion(&s->iomem, SGI_BASEIO_QLISP1_KERN_OFF,
                                &s->isp1_kern_alias_mr);
  }
  /* Each ISP channel asserts its bridge PCI-interrupt device line. */
  qdev_connect_gpio_out(DEVICE(&s->isp[0]), 0,
                        qdev_get_gpio_in(dev, SGI_BASEIO_INT_DEV_QLISP0));
  qdev_connect_gpio_out(DEVICE(&s->isp[1]), 0,
                        qdev_get_gpio_in(dev, SGI_BASEIO_INT_DEV_QLISP1));

  /*
   * IOC3 Ethernet NIC.  The machine claims the default -nic/-netdev backend
   * before realize; if none was given we still model the register file and DMA
   * engines, just without a transport.  The MAC the driver programs into EMAR
   * comes from the DS2502 EEPROM, not the backend, so the two need not agree.
   */
  if (s->nic_conf.peers.ncs[0]) {
    s->nic = qemu_new_nic(&net_sgi_baseio_eth_info, &s->nic_conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->nic_conf.macaddr.a);
  } else {
    s->nic = NULL;
  }
}

static void sgi_baseio_instance_init(Object *obj) {
  SGIBaseIOState *s = SGI_BASEIO(obj);
  s->wid_id = UINT32_MAX;
  object_initialize_child(obj, "ioc3-uart", &s->ioc3_uart, TYPE_SERIAL);
  object_initialize_child(obj, "qlisp0", &s->isp[0], TYPE_SGI_QLISP);
  object_initialize_child(obj, "qlisp1", &s->isp[1], TYPE_SGI_QLISP);
}

static const Property sgi_baseio_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIBaseIOState, nasid, 0),
    DEFINE_PROP_UINT32("widget", SGIBaseIOState, widget, 0),
    DEFINE_PROP_UINT32("wid-id", SGIBaseIOState, wid_id, UINT32_MAX),
    DEFINE_NIC_PROPERTIES(SGIBaseIOState, nic_conf),
};

static void sgi_baseio_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_baseio_realize;
  device_class_set_legacy_reset(dc, sgi_baseio_reset);
  device_class_set_props(dc, sgi_baseio_properties);
}

static const TypeInfo sgi_baseio_info = {
    .name = TYPE_SGI_BASEIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIBaseIOState),
    .instance_init = sgi_baseio_instance_init,
    .class_init = sgi_baseio_class_init,
};

static void sgi_baseio_register_types(void) {
  type_register_static(&sgi_baseio_info);
}

type_init(sgi_baseio_register_types)
