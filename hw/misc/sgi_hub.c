/*
 * SGI IP27 "Hub" (Bedrock) ASIC model.
 *
 * See include/hw/misc/sgi_hub.h for provenance.  This is deliberately a
 * faithful-but-partial register model: known registers return values derived
 * from the emulated node (nasid, CPU presence, memory config, RTC); unknown
 * accesses are logged as unimplemented and read as zero rather than being
 * faked.  The register set grows only in response to observed PROM/kernel
 * accesses (translate-or-omit).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_hub.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "target/mips/cpu.h"

/* --- Hub PI (processor interface) offsets, from Linux sn0/hubpi.h --- */
#define PI_CPU_PROTECT 0x000000
#define PI_PROT_OVRRD 0x000008
#define PI_IO_PROTECT 0x000010
#define PI_REGION_PRESENT 0x000018
#define PI_CPU_NUM 0x000020
#define PI_CALIAS_SIZE 0x000028
#define PI_CPU_PRESENT_A 0x000040
#define PI_CPU_PRESENT_B 0x000048
#define PI_CPU_ENABLE_A 0x000050
#define PI_CPU_ENABLE_B 0x000058
#define PI_INT_PEND_MOD 0x000090
#define PI_INT_PEND0 0x000098
#define PI_INT_PEND1 0x0000a0
#define PI_INT_MASK0_A 0x0000a8
#define PI_INT_MASK1_A 0x0000b0
#define PI_INT_MASK0_B 0x0000b8
#define PI_INT_MASK1_B 0x0000c0
#define PI_CC_PEND_SET_A 0x0000c8
#define PI_CC_PEND_SET_B 0x0000d0
#define PI_CC_PEND_CLR_A 0x0000d8
#define PI_CC_PEND_CLR_B 0x0000e0
#define PI_CC_MASK 0x0000e8
#define PI_RT_COMPARE_A 0x000108
#define PI_RT_COMPARE_B 0x000110
#define PI_RT_EN_A 0x000140
#define PI_RT_EN_B 0x000148
#define PI_RT_LOCAL_CTRL 0x000160
#define PI_RT_COUNT 0x030100

/* --- Hub MD (memory/directory) offsets --- */
#define MD_MEMORY_CONFIG 0x200018
#define MD_REFRESH_CONTROL 0x200020
/*
 * MicroLAN (1-wire) control.  The PROM bit-bangs the hub NIC EEPROM through
 * this register: it writes PULSE<19:10>/SAMPLE<9:2>, then spins on DONE
 * (bit1), returning RD_DATA (bit0).  With no EEPROM device present the line
 * floats high, so RD_DATA reads 1 and the PROM's part search reports
 * "not present" instead of hanging.
 */
#define MD_MLAN_CTL 0x2000a8
#define MD_UREG0_0 0x220000
#define MD_UREG0_7 0x220038
#define MD_SLOTID_USTAT 0x220048
#define MD_UREG1_0 0x220080
#define MD_UREG1_15 0x2200f8

/* --- Hub NI (network interface) offsets --- */
#define NI_STATUS_REV_ID 0x600000
#define NI_PORT_RESET 0x600008
#define NI_PROTECTION 0x600010
#define NI_SCRATCH_REG0 0x600100
#define NI_SCRATCH_REG1 0x600108
/*
 * NI vector/PIO engine.  The PROM performs remote ("vector") register reads
 * (libkl/ml/vector.c) by arming NI_VECTOR/NI_VECTOR_PARMS and polling
 * NI_VECTOR_STATUS for VALID; a vector path of 0 addresses the local node.
 */
#define NI_VECTOR_PARMS 0x600200
#define NI_VECTOR 0x600208
#define NI_VECTOR_DATA 0x600210
#define NI_VECTOR_STATUS 0x600300
#define NI_RETURN_VECTOR 0x600308
#define NI_VECTOR_READ_DATA 0x600310
#define NI_VECTOR_CLEAR 0x600380
#define NI_AGE_CPU0_MEMORY 0x600500
#define NI_AGE_CPU0_PIO 0x600508
#define NI_AGE_CPU1_MEMORY 0x600510
#define NI_AGE_CPU1_PIO 0x600518

/* NI_VECTOR_PARMS / NI_VECTOR_STATUS fields. */
#define NVP_PIOID_SHFT 40
#define NVP_WRITEID_SHFT 32
#define NVP_ADDRESS_MASK 0x00000000000ffff8ULL /* <19:03> */
#define NVP_TYPE_MASK 0x3ULL
#define NVS_VALID (1ULL << 63)
#define NVS_PIOID_SHFT 40
#define NVS_PIOID_MASK (0x7ffULL << NVS_PIOID_SHFT)
#define NVS_WRITEID_SHFT 32
#define NVS_ADDRESS_MASK 0x00000000fffffff8ULL /* <31:03> */
#define NVS_TYPE_MASK 0x7ULL
#define PIOTYPE_READ 0
#define PIOTYPE_WRITE 1
#define PIOTYPE_XCHG 3

/* --- Hub II (I/O interface) offsets --- */
#define IIO_WID 0x400000
#define IIO_WSTAT 0x400008
#define IIO_WCR 0x400020
#define IIO_ILAPR 0x400100
#define IIO_ILAPO 0x400108
#define IIO_IOWA 0x400110
#define IIO_IIWA 0x400118
#define IIO_ILCSR 0x400128
#define IIO_SCRATCH_REG0 0x400150
#define IIO_SCRATCH_REG1 0x400158

/* Hub widget identification: part 0xc101 (hub). */
#define HUB_WIDGET_PART_NUM 0xc101
#define HUB_WIDGET_REV 1
/*
 * IIO_ILCSR.  FUN_bfc56ac0 (the PROM's IOC3 finder) selects the IOC3 console
 * only if, for the local node, (ILCSR >> 12) & 2 (bit13, LLP status "up") and
 * the sign of (ILCSR << 53) (bit10, LLP enable) are both set.  Also set the
 * hub's own lnk_stat field (bits 17:16) since other paths read it.
 */
#define IIO_ILCSR_LINK_WORKING ((2ULL << 16) | (1ULL << 13) | (1ULL << 10))
/* IIO_WCR widget id field [3:0]. */
#define HUB_XIO_WIDGET_ID 8

/* NI_STATUS_REV_ID helpers */
#define NSRI_NODEID_SHFT 8
#define NSRI_REV_SHFT 4
#define NSRI_CHIPID_SHFT 0
#define NSRI_LINKUP (1ULL << 29)

/* HUB_REV_2_0 = 2 */
#define SGI_HUB_REV 2
#define SGI_HUB_CHIPID_HUB 0

/* PI_INT_PEND_MOD: bit 8 selects "set"; otherwise the value is a clear mask. */
#define PI_INT_PEND_SET_BIT 0x100

/* The hub RTC is a free-running counter; model it at 1 MHz (microseconds). */
static uint64_t sgi_hub_rtc_count(void) {
  return (uint64_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000);
}

static void sgi_hub_update_irqs(SGIHubState *s) {
  int i;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    qemu_set_irq(s->irq[i][0], (s->int_pend0 != 0));
    qemu_set_irq(s->irq[i][1], (s->int_pend1 != 0));
    if (s->rt_enable[i] && s->rt_compare[i] &&
        sgi_hub_rtc_count() >= s->rt_compare[i]) {
      qemu_set_irq(s->irq[i][2], 1);
    }
  }
}

static uint64_t sgi_hub_pi_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case PI_CPU_PROTECT:
  case PI_PROT_OVRRD:
  case PI_IO_PROTECT:
    return 0;
  case PI_REGION_PRESENT:
    return s->region_present;
  case PI_CPU_NUM:
    /* Slice of the CPU performing this access. */
    return current_cpu ? (current_cpu->cpu_index & 1) : 0;
  case PI_CALIAS_SIZE:
    return s->calias_size;
  case PI_CPU_PRESENT_A:
    return s->cpu_present[0];
  case PI_CPU_PRESENT_B:
    return s->cpu_present[1];
  case PI_CPU_ENABLE_A:
    return s->cpu_enable[0];
  case PI_CPU_ENABLE_B:
    return s->cpu_enable[1];
  case PI_INT_PEND0:
    return s->int_pend0;
  case PI_INT_PEND1:
    return s->int_pend1;
  case PI_INT_MASK0_A:
    return s->int_mask[0];
  case PI_INT_MASK1_A:
    return s->int_mask[1];
  case PI_INT_MASK0_B:
    return s->int_mask[2];
  case PI_INT_MASK1_B:
    return s->int_mask[3];
  case PI_CC_MASK:
    return s->cc_mask;
  case PI_RT_COMPARE_A:
    return s->rt_compare[0];
  case PI_RT_COMPARE_B:
    return s->rt_compare[1];
  case PI_RT_EN_A:
    return s->rt_enable[0];
  case PI_RT_EN_B:
    return s->rt_enable[1];
  case PI_RT_COUNT:
    return sgi_hub_rtc_count();
  case PI_RT_LOCAL_CTRL:
    return 0;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented PI read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_pi_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  switch (off) {
  case PI_CPU_PROTECT:
  case PI_PROT_OVRRD:
  case PI_IO_PROTECT:
  case PI_REGION_PRESENT:
  case PI_CPU_PRESENT_A:
  case PI_CPU_PRESENT_B:
  case PI_CPU_ENABLE_A:
  case PI_CPU_ENABLE_B:
    break;
  case PI_CALIAS_SIZE:
    s->calias_size = val;
    break;
  case PI_INT_PEND_MOD: {
    /* Set/clear a pending interrupt level. */
    uint64_t bit = val & 0x3f;
    bool set = (val & PI_INT_PEND_SET_BIT) != 0;
    if (bit < 64) {
      if (set) {
        s->int_pend0 |= (1ULL << bit);
      } else {
        s->int_pend0 &= ~(1ULL << bit);
      }
    }
    sgi_hub_update_irqs(s);
    break;
  }
  case PI_INT_MASK0_A:
    s->int_mask[0] = val;
    break;
  case PI_INT_MASK1_A:
    s->int_mask[1] = val;
    break;
  case PI_INT_MASK0_B:
    s->int_mask[2] = val;
    break;
  case PI_INT_MASK1_B:
    s->int_mask[3] = val;
    break;
  case PI_CC_PEND_SET_A:
  case PI_CC_PEND_SET_B:
  case PI_CC_PEND_CLR_A:
  case PI_CC_PEND_CLR_B:
    /* Crosscall pending set/clear; held in INT_PEND0 CC bits by the hub. */
    break;
  case PI_CC_MASK:
    s->cc_mask = val;
    break;
  case PI_RT_COMPARE_A:
    s->rt_compare[0] = val;
    break;
  case PI_RT_COMPARE_B:
    s->rt_compare[1] = val;
    break;
  case PI_RT_EN_A:
    s->rt_enable[0] = val;
    break;
  case PI_RT_EN_B:
    s->rt_enable[1] = val;
    break;
  case PI_RT_LOCAL_CTRL:
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented PI write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

/*
 * Hub NIC DS2502 1-wire EEPROM (family 0x09), bit-banged by the PROM through
 * MD_MLAN_CTL.  get_hub_nic_info() (libsk/ml/nic.c) reads this record and
 * derives the hub NIC from the DS2502 ROM serial (reported as "Laser:<hex>");
 * the Name field carries "IP27" so the record is accepted.  The bit protocol
 * and state machine mirror the ARCS source and the BaseIO model.
 */
enum {
  SGI_HUB_DS_IDLE = 0,
  SGI_HUB_DS_CMD,        /* shifting in an 8-bit ROM command */
  SGI_HUB_DS_READROM,    /* shifting out the 64-bit ROM id */
  SGI_HUB_DS_MATCHROM,   /* shifting in a 64-bit match-ROM sequence */
  SGI_HUB_DS_SEARCH,     /* ROM search: read bit, complement, write choice */
  SGI_HUB_DS_F0_PENDING, /* 0xf0 seen: next op decides search vs read-memory */
  SGI_HUB_DS_RMEM_ADDR,  /* shifting in the 16-bit memory address */
  SGI_HUB_DS_RMEM_DATA,  /* shifting out memory bytes */
};

/* --- Dallas/Maxim 1-wire CRC-8 (poly 0x8C, reflected) --- */
static uint8_t sgi_hub_crc8(const uint8_t *p, int n) {
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
static const int sgi_hub_oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                          1, 0, 0, 1, 0, 1, 1, 0};

static uint16_t sgi_hub_crc16_step(uint16_t crc, uint8_t in) {
  uint16_t data = in;
  data = (data ^ (crc & 0xff)) & 0xff;
  crc >>= 8;
  if (sgi_hub_oddparity[data & 0xf] ^ sgi_hub_oddparity[data >> 4]) {
    crc ^= 0xc001;
  }
  data <<= 6;
  crc ^= data;
  data <<= 1;
  crc ^= data;
  return crc;
}

static uint16_t sgi_hub_crc16(const uint8_t *p, int n) {
  uint16_t crc = 0;
  int i;
  for (i = 0; i < n; i++) {
    crc = sgi_hub_crc16_step(crc, p[i]);
  }
  return crc;
}

static void sgi_hub_ds_put(uint8_t *dst, const char *s, int n) {
  int i;
  for (i = 0; i < n; i++) {
    dst[i] = s[i] ? (uint8_t)s[i] : ' ';
  }
}

/*
 * Build the DS2502 image.  page 0: [0]=0x01, [1..10]=serial, [11..29]=part;
 * page 1: [0..5]=part cont., [6..9]=rev, [16..29]=name ("IP27 ...").
 * Each 32-byte page carries a trailing 16-bit CRC so the NIC driver's crc16
 * over the page equals 0xb001.
 */
static void sgi_hub_ds_init(SGIHubState *s) {
  static const char part[] = "030-1055-001";
  static const char name[] = "IP27 ORIGIN2K";
  int a, b;

  memset(s->ds_mem, 0xff, sizeof(s->ds_mem));

  s->ds_mem[0] = 0x01;
  sgi_hub_ds_put(&s->ds_mem[1], "1234567890", 10);
  sgi_hub_ds_put(&s->ds_mem[11], part, 19);

  sgi_hub_ds_put(&s->ds_mem[32 + 0], "", 6);
  sgi_hub_ds_put(&s->ds_mem[32 + 6], "0001", 4);
  s->ds_mem[32 + 10] = 0x00;
  memset(&s->ds_mem[32 + 11], 0x00, 4);
  s->ds_mem[32 + 15] = 0x00;
  sgi_hub_ds_put(&s->ds_mem[32 + 16], name, 14);

  for (int page = 0; page < 2; page++) {
    uint8_t *pg = &s->ds_mem[page * 32];
    pg[30] = 0;
    pg[31] = 0;
    for (a = 0; a < 256; a++) {
      for (b = 0; b < 256; b++) {
        pg[30] = a;
        pg[31] = b;
        if (sgi_hub_crc16(pg, 32) == 0xb001) {
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
  s->ds_rom[7] = sgi_hub_crc8(s->ds_rom, 7);
}

static void sgi_hub_ds_reset(SGIHubState *s) {
  s->ds_state = SGI_HUB_DS_CMD;
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

static void sgi_hub_ds_decode(SGIHubState *s) {
  switch (s->ds_cmd) {
  case 0x33: /* READ ROM */
    s->ds_state = SGI_HUB_DS_READROM;
    s->ds_out_index = 0;
    break;
  case 0x55: /* MATCH ROM */
    s->ds_state = SGI_HUB_DS_MATCHROM;
    s->ds_in = 0;
    s->ds_in_bits = 0;
    break;
  case 0xcc: /* SKIP ROM */
    s->ds_state = SGI_HUB_DS_CMD;
    break;
  case 0xf0: /* read-memory (or search-ROM; decided by next op) */
    s->ds_state = SGI_HUB_DS_F0_PENDING;
    break;
  default:
    s->ds_state = SGI_HUB_DS_CMD;
    break;
  }
  s->ds_cmd = 0;
  s->ds_cmd_bits = 0;
}

static void sgi_hub_ds_write_bit(SGIHubState *s, int bit) {
  switch (s->ds_state) {
  case SGI_HUB_DS_CMD:
    s->ds_cmd |= (bit & 1) << s->ds_cmd_bits;
    if (++s->ds_cmd_bits == 8) {
      sgi_hub_ds_decode(s);
    }
    break;
  case SGI_HUB_DS_MATCHROM:
    s->ds_in |= (bit & 1) << s->ds_in_bits;
    if (++s->ds_in_bits == 64) {
      s->ds_state = SGI_HUB_DS_CMD;
    }
    break;
  case SGI_HUB_DS_SEARCH:
    if (s->ds_search_phase == 2) {
      s->ds_search_phase = 0;
      if (++s->ds_out_index == 64) {
        s->ds_state = SGI_HUB_DS_CMD;
      }
    }
    break;
  case SGI_HUB_DS_F0_PENDING:
    s->ds_state = SGI_HUB_DS_RMEM_ADDR;
    s->ds_addr = 0;
    s->ds_in_bits = 0;
    /* fall through */
  case SGI_HUB_DS_RMEM_ADDR:
    s->ds_addr |= (bit & 1) << s->ds_in_bits;
    if (++s->ds_in_bits == 16) {
      s->ds_state = SGI_HUB_DS_RMEM_DATA;
      s->ds_out_index = 0;
      s->ds_extra = (s->ds_rom[0] == 0x09) ? 8 : 0;
    }
    break;
  default:
    break;
  }
}

static int sgi_hub_ds_read_bit(SGIHubState *s) {
  int bit;
  switch (s->ds_state) {
  case SGI_HUB_DS_READROM:
    bit = (s->ds_rom[s->ds_out_index / 8] >> (s->ds_out_index % 8)) & 1;
    if (++s->ds_out_index == 64) {
      s->ds_state = SGI_HUB_DS_CMD;
    }
    return bit;
  case SGI_HUB_DS_SEARCH:
    bit = (s->ds_rom[s->ds_out_index / 8] >> (s->ds_out_index % 8)) & 1;
    if (s->ds_search_phase == 0) {
      s->ds_search_phase = 1;
      return bit;
    }
    s->ds_search_phase = 2;
    return bit ^ 1;
  case SGI_HUB_DS_F0_PENDING:
    s->ds_state = SGI_HUB_DS_SEARCH;
    s->ds_out_index = 0;
    s->ds_search_phase = 0;
    return sgi_hub_ds_read_bit(s);
  case SGI_HUB_DS_RMEM_DATA:
    if (s->ds_extra > 0) {
      bit = (0xff >> (8 - s->ds_extra)) & 1;
      s->ds_extra--;
      return bit;
    }
    bit = (s->ds_mem[s->ds_addr + s->ds_out_index / 8] >>
           (s->ds_out_index % 8)) &
          1;
    if (++s->ds_out_index == 32 * 8) {
      s->ds_state = SGI_HUB_DS_CMD;
    }
    return bit;
  default:
    return 1;
  }
}

/*
 * MD_MLAN_CTL command: PULSE<19:10>/SAMPLE<9:2>; DONE (bit1) is always set,
 * RD_DATA (bit0) is the latched 1-wire line state.
 */
#define SGI_HUB_MLAN_PULSE(v) (((v) >> 10) & 0x3ff)
#define SGI_HUB_MLAN_SAMPLE(v) (((v) >> 2) & 0xff)

static void sgi_hub_mlan_write(SGIHubState *s, uint64_t val) {
  unsigned pulse = SGI_HUB_MLAN_PULSE(val);
  unsigned sample = SGI_HUB_MLAN_SAMPLE(val);

  if (pulse >= 480) {
    /* reset/presence pulse; single device => presence bit 0 */
    sgi_hub_ds_reset(s);
    s->ds_data_bit = 0;
  } else if (sample == 30) {
    sgi_hub_ds_write_bit(s, 0);
    s->ds_data_bit = 0;
  } else if (sample == 110) {
    sgi_hub_ds_write_bit(s, 1);
    s->ds_data_bit = 0;
  } else if (sample == 13) {
    s->ds_data_bit = sgi_hub_ds_read_bit(s);
  } else {
    s->ds_data_bit = 0;
  }
}

static uint64_t sgi_hub_md_read(SGIHubState *s, hwaddr off) {
  if (off == MD_MEMORY_CONFIG) {
    return s->mem_config;
  }
  if (off == MD_REFRESH_CONTROL) {
    return s->refresh_ctl;
  }
  if (off == MD_SLOTID_USTAT) {
    return s->slotid_ustat;
  }
  if (off == MD_MLAN_CTL) {
    /* DONE (bit1) always set; RD_DATA (bit0) is the latched 1-wire line. */
    return 0x2 | (s->ds_data_bit & 1);
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    /* uController/UART and MLAN registers: state held, semantic model later. */
    return 0;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD read @0x%" HWADDR_PRIx
                           "\n",
                off);
  return 0;
}

static void sgi_hub_md_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  if (off == MD_MEMORY_CONFIG) {
    s->mem_config = val;
    return;
  }
  if (off == MD_REFRESH_CONTROL) {
    s->refresh_ctl = val;
    return;
  }
  if (off == MD_MLAN_CTL) {
    sgi_hub_mlan_write(s, val);
    return;
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    return;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD write @0x%" HWADDR_PRIx
                           " = 0x%" PRIx64 "\n",
                off, val);
}

static uint64_t sgi_hub_read_off(SGIHubState *s, hwaddr off);
static void sgi_hub_write_off(SGIHubState *s, hwaddr off, uint64_t val);

/*
 * Execute a NI vector PIO operation (armed via NI_VECTOR/NI_VECTOR_PARMS).
 * Only the local node (vector path 0) is modelled; a remote vector has no
 * peer in a single-node system, so VALID is left clear and the caller times
 * out (the honest "no remote node" result).
 */
static void sgi_hub_ni_vector_go(SGIHubState *s, uint64_t parms) {
  uint64_t pioid = (parms >> NVP_PIOID_SHFT) & 0x7ff;
  uint64_t wid = (parms >> NVP_WRITEID_SHFT) & 0xff;
  uint64_t addr = parms & NVP_ADDRESS_MASK;
  unsigned type = parms & NVP_TYPE_MASK;
  hwaddr reg;

  /*
   * Only the local node (vector path 0) is modelled; a remote vector has no
   * peer in a single-node system, so VALID is left clear and the caller
   * times out (the honest "no remote node" result).
   */
  if (s->ni_vector != 0) {
    s->ni_vector_status = 0;
    return;
  }

  /*
   * Register addresses in the vector parameter block are NI-block-relative
   * for the low range (e.g. NI_SCRATCH_REG0 = 0x100); larger values are the
   * full hub-window offset.
   */
  reg = (addr < SGI_HUB_NI_BASE) ? SGI_HUB_NI_BASE + addr : addr;

  switch (type) {
  case PIOTYPE_READ:
    s->ni_vector_rd_data = sgi_hub_read_off(s, reg);
    break;
  case PIOTYPE_WRITE:
    sgi_hub_write_off(s, reg, s->ni_vector_data);
    break;
  case PIOTYPE_XCHG:
    s->ni_vector_rd_data = sgi_hub_read_off(s, reg);
    sgi_hub_write_off(s, reg, s->ni_vector_data);
    break;
  default:
    s->ni_vector_status = 0;
    return;
  }

  s->ni_vector_status = NVS_VALID | (pioid << NVS_PIOID_SHFT) |
                        (wid << NVS_WRITEID_SHFT) |
                        (addr & NVS_ADDRESS_MASK) | (type & NVS_TYPE_MASK);
}

static uint64_t sgi_hub_ni_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case NI_STATUS_REV_ID:
    return ((uint64_t)(s->nasid & 0x1ff) << NSRI_NODEID_SHFT) |
           ((uint64_t)SGI_HUB_REV << NSRI_REV_SHFT) |
           ((uint64_t)SGI_HUB_CHIPID_HUB << NSRI_CHIPID_SHFT) | NSRI_LINKUP;
  case NI_PROTECTION:
    return 0;
  case NI_SCRATCH_REG0:
    return s->ni_scratch[0];
  case NI_SCRATCH_REG1:
    return s->ni_scratch[1];
  case NI_VECTOR:
  case NI_RETURN_VECTOR:
    return s->ni_vector;
  case NI_VECTOR_PARMS:
    return s->ni_vector_parms;
  case NI_VECTOR_DATA:
    return s->ni_vector_data;
  case NI_VECTOR_STATUS:
    return s->ni_vector_status;
  case NI_VECTOR_READ_DATA:
    return s->ni_vector_rd_data;
  case NI_VECTOR_CLEAR:
    /* Read-clear: reading it invalidates the previous vector status. */
    s->ni_vector_status = 0;
    return 0;
  case NI_AGE_CPU0_MEMORY:
    return s->ni_age[0];
  case NI_AGE_CPU0_PIO:
    return s->ni_age[1];
  case NI_AGE_CPU1_MEMORY:
    return s->ni_age[2];
  case NI_AGE_CPU1_PIO:
    return s->ni_age[3];
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented NI read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_ni_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  switch (off) {
  case NI_PORT_RESET:
  case NI_PROTECTION:
    break;
  case NI_SCRATCH_REG0:
    s->ni_scratch[0] = val;
    break;
  case NI_SCRATCH_REG1:
    s->ni_scratch[1] = val;
    break;
  case NI_VECTOR:
    s->ni_vector = val;
    break;
  case NI_VECTOR_DATA:
    s->ni_vector_data = val;
    break;
  case NI_VECTOR_PARMS:
    s->ni_vector_parms = val;
    sgi_hub_ni_vector_go(s, val);
    break;
  case NI_VECTOR_CLEAR:
    s->ni_vector_status = 0;
    break;
  case NI_AGE_CPU0_MEMORY:
    s->ni_age[0] = val;
    break;
  case NI_AGE_CPU0_PIO:
    s->ni_age[1] = val;
    break;
  case NI_AGE_CPU1_MEMORY:
    s->ni_age[2] = val;
    break;
  case NI_AGE_CPU1_PIO:
    s->ni_age[3] = val;
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented NI write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

static uint64_t sgi_hub_ii_read(SGIHubState *s, hwaddr off) {
  switch (off) {
  case IIO_WID:
    return ((uint64_t)HUB_WIDGET_PART_NUM << 16) |
           ((uint64_t)HUB_WIDGET_REV << 12);
  case IIO_WSTAT:
    return 0;
  case IIO_WCR:
    return s->ii_wcr;
  case IIO_ILAPR:
  case IIO_ILAPO:
    return 0;
  case IIO_IOWA:
    return s->ii_iowa;
  case IIO_IIWA:
    return s->ii_iiwa;
  case IIO_ILCSR:
    return s->ii_ilcsr;
  case IIO_SCRATCH_REG0:
    return s->ii_scratch[0];
  case IIO_SCRATCH_REG1:
    return s->ii_scratch[1];
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented II read @0x%" HWADDR_PRIx
                             "\n",
                  off);
    return 0;
  }
}

static void sgi_hub_ii_write(SGIHubState *s, hwaddr off, uint64_t val) {
  switch (off) {
  case IIO_WSTAT:
  case IIO_WID:
  case IIO_ILAPR:
  case IIO_ILAPO:
    break;
  case IIO_WCR:
    s->ii_wcr = val;
    break;
  case IIO_IOWA:
    s->ii_iowa = val;
    break;
  case IIO_IIWA:
    s->ii_iiwa = val;
    break;
  case IIO_ILCSR:
    /* Keep the modelled link reported as working. */
    s->ii_ilcsr = val | IIO_ILCSR_LINK_WORKING;
    break;
  case IIO_SCRATCH_REG0:
    s->ii_scratch[0] = val;
    break;
  case IIO_SCRATCH_REG1:
    s->ii_scratch[1] = val;
    break;
  default:
    qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented II write @0x%" HWADDR_PRIx
                             " = 0x%" PRIx64 "\n",
                  off, val);
    break;
  }
}

/*
 * Hub registers are 64-bit but the PROM and kernel also access them as 32-bit
 * halves (big-endian: base+0 is the high word, base+4 the low word).  We model
 * the 64-bit register and fold 32-bit accesses onto it, otherwise a low-word
 * read (e.g. IIO_ILCSR's link status) silently returns 0.
 */
static uint64_t sgi_hub_read_off(SGIHubState *s, hwaddr off) {
  if (off < SGI_HUB_MD_BASE) {
    return sgi_hub_pi_read(s, off - SGI_HUB_PI_BASE);
  } else if (off < SGI_HUB_II_BASE) {
    return sgi_hub_md_read(s, off);
  } else if (off < SGI_HUB_NI_BASE) {
    return sgi_hub_ii_read(s, off);
  } else {
    return sgi_hub_ni_read(s, off);
  }
}

static void sgi_hub_write_off(SGIHubState *s, hwaddr off, uint64_t val) {
  if (off < SGI_HUB_MD_BASE) {
    sgi_hub_pi_write(s, off - SGI_HUB_PI_BASE, val, 8);
  } else if (off < SGI_HUB_II_BASE) {
    sgi_hub_md_write(s, off, val, 8);
  } else if (off < SGI_HUB_NI_BASE) {
    sgi_hub_ii_write(s, off, val);
  } else {
    sgi_hub_ni_write(s, off, val, 8);
  }
}

static uint64_t sgi_hub_read(void *opaque, hwaddr addr, unsigned size) {
  SGIHubState *s = opaque;
  hwaddr off = addr & (SGI_HUB_WINDOW_SIZE - 1);

  if (off >= SGI_HUB_REMOTE_ALIAS) {
    off -= SGI_HUB_REMOTE_ALIAS;
  }
  if (size == 4) {
    uint64_t full = sgi_hub_read_off(s, off & ~7ULL);
    return (off & 4) ? (full & 0xffffffffULL) : (full >> 32);
  }
  return sgi_hub_read_off(s, off);
}

static void sgi_hub_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size) {
  SGIHubState *s = opaque;
  hwaddr off = addr & (SGI_HUB_WINDOW_SIZE - 1);

  if (off >= SGI_HUB_REMOTE_ALIAS) {
    off -= SGI_HUB_REMOTE_ALIAS;
  }
  if (size == 4) {
    uint64_t cur = sgi_hub_read_off(s, off & ~7ULL);
    if (off & 4) {
      cur = (cur & 0xffffffff00000000ULL) | (val & 0xffffffffULL);
    } else {
      cur = (cur & 0xffffffffULL) | ((val & 0xffffffffULL) << 32);
    }
    sgi_hub_write_off(s, off & ~7ULL, cur);
    return;
  }
  sgi_hub_write_off(s, off, val);
}

static const MemoryRegionOps sgi_hub_ops = {
    .read = sgi_hub_read,
    .write = sgi_hub_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
};

static void sgi_hub_reset(DeviceState *dev) {
  SGIHubState *s = SGI_HUB(dev);
  int i;

  s->int_pend0 = 0;
  s->int_pend1 = 0;
  for (i = 0; i < 4; i++) {
    s->int_mask[i] = 0;
  }
  s->cc_mask = 0;
  s->region_present = 1;
  s->calias_size = 0;

  s->cpu_present[0] = 1;
  s->cpu_enable[0] = 1;
  s->cpu_present[1] = (s->num_cpus > 1) ? 1 : 0;
  s->cpu_enable[1] = (s->num_cpus > 1) ? 1 : 0;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    s->rt_compare[i] = 0;
    s->rt_enable[i] = 0;
  }

  /* MD_SLOTID_USTAT: FPGA/flash ready, slot id 0. */
  s->slotid_ustat = 0x10;

  /* II: hub widget id, working XIO link, all widgets accessible. */
  s->ii_wcr = HUB_XIO_WIDGET_ID;
  s->ii_iowa = 0x1ff;
  s->ii_iiwa = 0x1ff;
  s->ii_ilcsr = IIO_ILCSR_LINK_WORKING;
  s->ii_scratch[0] = 0;
  s->ii_scratch[1] = 0;

  sgi_hub_update_irqs(s);
}

static void sgi_hub_realize(DeviceState *dev, Error **errp) {
  SGIHubState *s = SGI_HUB(dev);
  int i, j;

  memory_region_init_io(&s->iomem, OBJECT(s), &sgi_hub_ops, s, "sgi-hub",
                        SGI_HUB_WINDOW_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

  /* Hub NIC 1-wire EEPROM and its bit-bang state. */
  sgi_hub_ds_init(s);
  sgi_hub_ds_reset(s);

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    for (j = 0; j < 5; j++) {
      sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i][j]);
    }
  }
}

static const Property sgi_hub_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIHubState, nasid, 0),
    DEFINE_PROP_UINT32("num-cpus", SGIHubState, num_cpus, 1),
    DEFINE_PROP_UINT64("mem-config", SGIHubState, mem_config, 0),
};

static void sgi_hub_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_hub_realize;
  device_class_set_legacy_reset(dc, sgi_hub_reset);
  device_class_set_props(dc, sgi_hub_properties);
}

static const TypeInfo sgi_hub_info = {
    .name = TYPE_SGI_HUB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHubState),
    .class_init = sgi_hub_class_init,
};

static void sgi_hub_register_types(void) { type_register_static(&sgi_hub_info); }

type_init(sgi_hub_register_types)
