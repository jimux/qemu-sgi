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
#include "hw/char/serial.h"
#include "hw/misc/sgi_baseio.h"
#include "chardev/char.h"
#include "qapi/error.h"
#include "qemu/log.h"

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
static void sgi_baseio_ds_init(SGIBaseIOState *s) {
  static const char part[] = "030-0734-001"; /* IO6 BaseIO */
  static const char name[] = "BASEIO";
  int a, b;

  memset(s->ds_mem, 0xff, sizeof(s->ds_mem));

  /* Page 0. */
  s->ds_mem[0] = 0x01;
  sgi_baseio_put(&s->ds_mem[1], "1234567890", 10);
  sgi_baseio_put(&s->ds_mem[11], part, 19);

  /* Page 1. */
  sgi_baseio_put(&s->ds_mem[32 + 0], "", 6);
  sgi_baseio_put(&s->ds_mem[32 + 6], "0001", 4);
  s->ds_mem[32 + 10] = 0x00;
  memset(&s->ds_mem[32 + 11], 0x00, 4);
  s->ds_mem[32 + 15] = 0x00;
  sgi_baseio_put(&s->ds_mem[32 + 16], name, 14);

  /* Solve each page's trailing 16-bit CRC so crc16(page) == 0xb001. */
  for (int page = 0; page < 2; page++) {
    uint8_t *pg = &s->ds_mem[page * 32];
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
  s->ds_rom[0] = 0x09;
  s->ds_rom[1] = 0x01;
  s->ds_rom[2] = 0x02;
  s->ds_rom[3] = 0x03;
  s->ds_rom[4] = 0x04;
  s->ds_rom[5] = 0x05;
  s->ds_rom[6] = 0x06;
  s->ds_rom[7] = sgi_baseio_crc8(s->ds_rom, 7);
}

static void sgi_baseio_ds_reset(SGIBaseIOState *s) {
  s->ds_state = SGI_DS_CMD;
  s->ds_cmd = 0;
  s->ds_cmd_bits = 0;
  s->ds_in = 0;
  s->ds_in_bits = 0;
  s->ds_out_index = 0;
  s->ds_search_phase = 0;
  s->ds_addr = 0;
  s->ds_extra = 0;
  s->ds_data_bit = 0;
}

static void sgi_baseio_ds_decode(SGIBaseIOState *s) {
  switch (s->ds_cmd) {
  case 0x33: /* READ ROM */
    s->ds_state = SGI_DS_READROM;
    s->ds_out_index = 0;
    break;
  case 0x55: /* MATCH ROM */
    s->ds_state = SGI_DS_MATCHROM;
    s->ds_in = 0;
    s->ds_in_bits = 0;
    break;
  case 0xcc: /* SKIP ROM */
    s->ds_state = SGI_DS_CMD;
    break;
  case 0xf0: /* read-memory (or search-ROM; decided by next op) */
    s->ds_state = SGI_DS_F0_PENDING;
    break;
  default:
    s->ds_state = SGI_DS_CMD;
    break;
  }
  s->ds_cmd = 0;
  s->ds_cmd_bits = 0;
}

static void sgi_baseio_ds_write_bit(SGIBaseIOState *s, int bit) {
  switch (s->ds_state) {
  case SGI_DS_CMD:
    s->ds_cmd |= (bit & 1) << s->ds_cmd_bits;
    if (++s->ds_cmd_bits == 8) {
      sgi_baseio_ds_decode(s);
    }
    break;
  case SGI_DS_MATCHROM:
    s->ds_in |= (bit & 1) << s->ds_in_bits;
    if (++s->ds_in_bits == 64) {
      s->ds_state = SGI_DS_CMD;
    }
    break;
  case SGI_DS_SEARCH:
    /* Host choice for the current bit; advance to the next ROM bit. */
    if (s->ds_search_phase == 2) {
      s->ds_search_phase = 0;
      if (++s->ds_out_index == 64) {
        s->ds_state = SGI_DS_CMD;
      }
    }
    break;
  case SGI_DS_F0_PENDING:
    /* A write after 0xf0 means the 16-bit memory address follows. */
    s->ds_state = SGI_DS_RMEM_ADDR;
    s->ds_addr = 0;
    s->ds_in_bits = 0;
    /* fall through */
  case SGI_DS_RMEM_ADDR:
    s->ds_addr |= (bit & 1) << s->ds_in_bits;
    if (++s->ds_in_bits == 16) {
      s->ds_state = SGI_DS_RMEM_DATA;
      s->ds_out_index = 0;
      s->ds_extra = (s->ds_rom[0] == 0x09) ? 8 : 0;
    }
    break;
  default:
    break;
  }
}

static int sgi_baseio_ds_read_bit(SGIBaseIOState *s) {
  int bit;
  switch (s->ds_state) {
  case SGI_DS_READROM:
    bit = (s->ds_rom[s->ds_out_index / 8] >> (s->ds_out_index % 8)) & 1;
    if (++s->ds_out_index == 64) {
      s->ds_state = SGI_DS_CMD;
    }
    return bit;
  case SGI_DS_SEARCH:
    bit = (s->ds_rom[s->ds_out_index / 8] >> (s->ds_out_index % 8)) & 1;
    if (s->ds_search_phase == 0) {
      s->ds_search_phase = 1;
      return bit;
    }
    s->ds_search_phase = 2;
    return bit ^ 1;
  case SGI_DS_F0_PENDING:
    /* A read after 0xf0 starts a ROM search. */
    s->ds_state = SGI_DS_SEARCH;
    s->ds_out_index = 0;
    s->ds_search_phase = 0;
    return sgi_baseio_ds_read_bit(s);
  case SGI_DS_RMEM_DATA:
    if (s->ds_extra > 0) {
      bit = (0xff >> (8 - s->ds_extra)) & 1;
      s->ds_extra--;
      return bit;
    }
    bit = (s->ds_mem[s->ds_addr + s->ds_out_index / 8] >>
           (s->ds_out_index % 8)) &
          1;
    if (++s->ds_out_index == 32 * 8) {
      s->ds_state = SGI_DS_CMD;
    }
    return bit;
  default:
    return 0;
  }
}

/* MCR line state: returns DATA bit; DONE is always set for the host poll. */
static uint64_t sgi_baseio_mcr_read(SGIBaseIOState *s) {
  return 0x2 | (s->ds_data_bit & 1);
}

static void sgi_baseio_mcr_write(SGIBaseIOState *s, uint64_t val) {
  unsigned pulse = (val >> 10) & 0x3ff;
  unsigned sample = (val >> 2) & 0xff;

  if (pulse >= 480) {
    /* reset/presence pulse; single device => presence bit 0 */
    sgi_baseio_ds_reset(s);
    s->ds_data_bit = 0;
  } else if (sample == 30) {
    sgi_baseio_ds_write_bit(s, 0);
    s->ds_data_bit = 0;
  } else if (sample == 110) {
    sgi_baseio_ds_write_bit(s, 1);
    s->ds_data_bit = 0;
  } else if (sample == 13) {
    s->ds_data_bit = sgi_baseio_ds_read_bit(s);
  } else {
    s->ds_data_bit = 0;
  }
}

static uint64_t sgi_baseio_read(void *opaque, hwaddr off, unsigned size) {
  SGIBaseIOState *s = opaque;

  /*
   * XIO widget identification, low word (offset 4): bits [27:12] are the
   * widget part number; 0xc002 identifies a Bridge.
   */
  if (off == 4) {
    return (uint64_t)(SGI_BASEIO_WIDGET_PART << 12);
  }
  /* Bridge MicroLAN control register (1-wire). */
  if (off == 0xb4) {
    return sgi_baseio_mcr_read(s);
  }
  /*
   * IOC3 register block at bridge+0x200000.  Its MCR (0x200030) is a second
   * 1-wire controller for the IOC3's MAC-address NIC.  Report done for now.
   */
  if (off == 0x200030) {
    return 0x2;
  }
  /* IOC3 SuperIO control register: bus arbiter idle. */
  if (off == SGI_BASEIO_IOC3_SIO_CR) {
    return 0x00400000; /* SIO_CR_ARB_DIAG_IDLE */
  }
  /*
   * BaseIO component slots: the Bridge exposes per-slot component IDs at
   * bride+0x20000 + slot*0x1000; the PROM's BaseIO init scans them and inits
   * the component whose part is 0x310a9 (the IOC3/SuperIO).  Slot 0 is the
   * IOC3; the bridge status words read 0 (ok).
   */
  if (off >= 0x20000 && off < 0x28000 && (off & 0xfff) == 0) {
    unsigned slot = (off - 0x20000) >> 12;
    return (slot == 0) ? 0x310a9 : 0xffffffff;
  }
  if (off == 0x104 || off == 0x114) {
    return 0;
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

  if (off == 0xb4) {
    sgi_baseio_mcr_write(s, val);
    return;
  }
  qemu_log_mask(LOG_UNIMP,
                "sgi-baseio: unimplemented write @0x%" HWADDR_PRIx
                " = 0x%" PRIx64 " (size %u)\n",
                off, val, size);
}

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

  sgi_baseio_ds_init(s);
  sgi_baseio_ds_reset(s);
}

static void sgi_baseio_realize(DeviceState *dev, Error **errp) {
  SGIBaseIOState *s = SGI_BASEIO(dev);
  Chardev *chr;

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
  memory_region_init_io(&s->ioc3_uart_mr, OBJECT(s), &sgi_baseio_uart_ops, s,
                        "sgi-baseio-uart", SGI_BASEIO_IOC3_UART_SIZE);
  memory_region_add_subregion(&s->iomem, SGI_BASEIO_IOC3_UART,
                              &s->ioc3_uart_mr);
}

static void sgi_baseio_instance_init(Object *obj) {
  SGIBaseIOState *s = SGI_BASEIO(obj);
  object_initialize_child(obj, "ioc3-uart", &s->ioc3_uart, TYPE_SERIAL);
}

static const Property sgi_baseio_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIBaseIOState, nasid, 0),
    DEFINE_PROP_UINT32("widget", SGIBaseIOState, widget, 0),
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
