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
#include "qemu/main-loop.h"
#include "system/runstate.h"
#include "system/address-spaces.h"

static void sgi_hub_reset_bh(void *opaque);
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
#define PI_RT_PEND_A 0x000120
#define PI_RT_PEND_B 0x000128
#define PI_RT_EN_A 0x000140
#define PI_RT_EN_B 0x000148
#define PI_RT_LOCAL_CTRL 0x000160
#define PI_RT_COUNT 0x030100
/* PI error-stack addresses; entry.s resets once unless _B holds "Rst0". */
#define PI_ERR_STACK_ADDR_A 0x000418
#define PI_ERR_STACK_ADDR_B 0x000420
#define PI_ERR_STACK_RST0 0x52737430ULL

/* --- Hub MD (memory/directory) offsets --- */
#define MD_MEMORY_CONFIG 0x200018
#define MD_REFRESH_CONTROL 0x200020
/* MD error-status / clear block (hubmd.h).  No errors pending; reads return 0
 * and writes are the "clear" side-effects, which we absorb. */
#define MD_DIR_ERROR_CLR 0x200058
#define MD_PROTOCOL_ERROR 0x200060
#define MD_PROTOCOL_ERROR_CLR 0x200068
#define MD_MEM_ERROR_CLR 0x200078
#define MD_MISC_ERROR_CLR 0x200088
/* DIMM-mode init pair at the end of the same walk (hubmd.h). */
#define MD_MEM_DIMM_INIT 0x200090
#define MD_DIR_DIMM_INIT 0x200098
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
/* Front-panel LED (hubmd.h MD_LED0): read/write, low 8 bits. */
#define MD_LED0 0x220050
/* Performance monitor: MD_PERF_SEL + 6 free-running counters. */
#define MD_PERF_SEL 0x210000
#define MD_PERF_CNT0 0x210010
#define MD_PERF_CNT5 0x210038
#define MD_UREG1_0 0x220080
#define MD_UREG1_15 0x2200f8

/*
 * PCF8584 I2C bus controller (libkl/ml/i2c.c).  A0 (data register, S0) is at
 * KL_I2C_REG = MD_UREG0_0; A1 (control/status, S1) is at MD_UREG0_0 + 8.
 */
#define I2C_A0_OFF 0x220000
#define I2C_A1_OFF 0x220008

/* Control register write-only bits. */
#define I2C_CTL_PIN 0x80
#define I2C_CTL_ESO 0x40
#define I2C_CTL_ES1 0x20
#define I2C_CTL_ES2 0x10
#define I2C_CTL_STA 0x04
#define I2C_CTL_STO 0x02
#define I2C_CTL_ACK 0x01

/* Status register read-only bits. */
#define I2C_STA_PIN 0x80
#define I2C_STA_BER 0x10
#define I2C_STA_LRB 0x08 /* last received (ACK) bit */
#define I2C_STA_AAS 0x04
#define I2C_STA_LAB 0x02
#define I2C_STA_BNB 0x01

/* ELSC NVRAM slave addresses (I2C_ADDR_RAM | page) and the arb address. */
#define I2C_ADDR_RAM 0x50
#define I2C_ARB_ADDR 0x70

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

/* --- Hub II block-transfer engine (BTE0/BTE1) --- */
#define IIO_IBLS_0 0x410000 /* length/status */
#define IIO_IBSA_0 0x410008 /* source address */
#define IIO_IBDA_0 0x410010 /* destination address */
#define IIO_IBCT_0 0x410018 /* control/terminate */
#define IIO_IBNA_0 0x410020 /* notification address */
#define IIO_IBIA_0 0x410028 /* interrupt address */
#define IIO_BTE_STRIDE 0x10000
#define IBLS_BUSY (1ULL << 20)
#define IBLS_ERROR (1ULL << 16)
#define IBLS_LENGTH_MASK 0xffffULL
#define BTE_LEN_SHIFT 7

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

/* NI routing tables (hubni.h): written by the PROM's distribute_tables(). */
#define NI_META_TABLE0 0x638000
#define NI_META_ENTRIES 32
#define NI_LOCAL_TABLE0 0x638100
#define NI_LOCAL_ENTRIES 16

/*
 * SN0 router register offsets (sys/SN/router.h) and field layout.  Routers are
 * reached only through the hub NI vector PIO engine, so they are not memory
 * mapped; the machine owns the SGIRouterState and both hubs point at it.
 */
#define RR_STATUS_REV_ID 0x00000
#define RR_PORT_RESET 0x00008
#define RR_PROT_CONF 0x00010
#define RR_GLOBAL_PARMS 0x00018
#define RR_SCRATCH_REG0 0x00020
#define RR_SCRATCH_REG1 0x00028
#define RR_DIAG_PARMS 0x00030
#define RR_NIC_ULAN 0x00038
#define RR_META_TABLE0 0x70000
#define RR_LOCAL_TABLE0 0x70100
#define RR_PORT_REG(_l, _o) (((_l) << 16) | (_o))
#define RR_PORT_PARMS(_l) RR_PORT_REG(_l, 0x0000)
#define RR_STATUS_ERROR(_l) RR_PORT_REG(_l, 0x0008)
#define RR_HISTOGRAM(_l) RR_PORT_REG(_l, 0x0010)
#define RR_RESET_MASK(_l) RR_PORT_REG(_l, 0x0018)

/* RSRI field layout. */
#define RSRI_INPORT_SHFT 46
#define RSRI_LINKWORKING(_l) (1ULL << (26 + 3 * (_l)))
#define RSRI_LINK8BIT(_l) (1ULL << (27 + 3 * (_l)))
#define RSRI_CHIPIN_SHFT 8
#define RSRI_CHIPREV_SHFT 4
#define RSRI_CHIPID_SHFT 0
#define RSRI_CHIPID_ROUTER 1

#define RSCR0_NIC_MASK 0xffffffffffffULL
#define RSCR0_BOOTED_MASK (1ULL << 60)
#define RSCR0_LOCALID_SHFT 56
#define RPCONF_METAIDVALID (1ULL << 11)
#define RPCONF_METAID_SHFT 6
#define RPCONF_FLOCAL_SHFT 12

/*
 * Hub revision reported in NI_STATUS_REV_ID.  sys/SN/SN0/hub.h numbers these
 * HUB_REV_1_0=1, HUB_REV_2_0=2, HUB_REV_2_1=3, ...; ml/SN/klgraph.c warns
 * "INVALID CUSTOMER CONFIGURATION - DOWNREV Hub ASIC" when the revision is
 * below HUB_REV_2_1, and the same field drives mp.c's force_fire_and_forget
 * (PIO conveyor-belt) decision, so report 2.1 rather than 2.0.
 */
#define SGI_HUB_REV 3
#define SGI_HUB_CHIPID_HUB 0

/* PI_INT_PEND_MOD: bit 8 selects "set"; otherwise the value is a clear mask. */
#define PI_INT_PEND_SET_BIT 0x100

/* The hub RTC is a free-running counter; an IP27 runs it at 1.25 MHz, i.e.
 * 800 ns per tick.  The IRIX kernel hard-codes that rate: us_delay() waits
 * us * 1250000 / 1000000 RT_COUNT ticks and tstoclock() converts nanoseconds to
 * ticks by dividing by 800, so the period must be right, not merely monotonic.
 * (The "RTC frequency incorrect" panic is a separate, config-table comparison
 * in ml/SN/mp.c, not a measurement of this counter.) */
#define SGI_HUB_RTC_HZ 1250000
static uint64_t sgi_hub_rtc_count(void) {
  /*
   * Count from the HOST clock at the documented SN0 rate (project clock
   * doctrine: guest time is locked to the host wall clock, not to emulated CPU
   * speed).  With QEMU_CLOCK_VIRTUAL the count races ahead of the guest's
   * modeled rate, so a COMPARE the guest reloads from a read of this counter is
   * instantly in the past and PI_RT_PEND_A/B re-latches at once -- a
   * level-interrupt livelock on IP4 (the same class as the CP0 IP7 storm).
   */
  return (uint64_t)(qemu_clock_get_ns(QEMU_CLOCK_REALTIME) /
                    (1000000000ULL / SGI_HUB_RTC_HZ));
}

/* Latch PI_RT_PEND_x once a freshly armed, enabled COMPARE_x is reached. */
static void sgi_hub_rtc_poll(SGIHubState *s) {
  uint64_t now = sgi_hub_rtc_count();
  int i;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    if (s->rt_armed[i] && s->rt_enable[i] && now >= s->rt_compare[i]) {
      s->rt_pend[i] = 1;
      s->rt_armed[i] = 0;
    }
  }
}

static void sgi_hub_update_irqs(SGIHubState *s) {
  int i;

  sgi_hub_rtc_poll(s);
  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    /*
     * CPU slice A (even CPU) is gated by INT_MASK0_A/1_A, slice B by the
     * _B pair.  int_mask[] is ordered [0]=A mask0, [1]=A mask1, [2]=B mask0,
     * [3]=B mask1.  The kernel programs the mask in intr_connect_level(),
     * so a pending vector asserts the CPU line only once its handler is
     * connected -- and a masked pending bit must not keep IP2/IP3 asserted
     * or the dispatcher (which masks INT_PENDx by the same value) would
     * livelock.
     */
    int slice = i & 1;

    qemu_set_irq(s->irq[i][0],
                 (s->int_pend0 & s->int_mask[slice * 2 + 0]) != 0);
    qemu_set_irq(s->irq[i][1],
                 (s->int_pend1 & s->int_mask[slice * 2 + 1]) != 0);
    /* L4 (RTC): asserted while a pend bit is latched and the slice is
     * enabled.  The OS reads PI_RT_PEND_x and acks by writing it 0. */
    qemu_set_irq(s->irq[i][2], s->rt_pend[i] && s->rt_enable[i]);
  }
}

void sgi_hub_raise_vector(SGIHubState *s, unsigned vec, int level) {
  if (vec < 64) {
    if (level) {
      s->int_pend0 |= 1ULL << vec;
    } else {
      s->int_pend0 &= ~(1ULL << vec);
    }
  } else if (vec < 128) {
    uint64_t bit = 1ULL << (vec - 64);

    if (level) {
      s->int_pend1 |= bit;
    } else {
      s->int_pend1 &= ~bit;
    }
  } else {
    return;
  }
  if (getenv("SGI_HUB_INTDBG")) {
    qemu_log_mask(LOG_UNIMP,
                  "HUB raise vec=%u level=%d pend0=0x%llx mask0A=0x%llx\n", vec,
                  level, (unsigned long long)s->int_pend0,
                  (unsigned long long)s->int_mask[0]);
  }
  sgi_hub_update_irqs(s);
}

/* The RTC interrupt can fire while the CPU is idle, so poll it on a timer. */
static void sgi_hub_rtc_timer(void *opaque) {
  SGIHubState *s = opaque;

  sgi_hub_update_irqs(s);
  timer_mod(s->rt_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1000000);
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
  case PI_INT_PEND_MOD:
    /*
     * Write-only modify register per hubpi.h, but firmware polls it here;
     * return the current INT_PEND0 state so the poll makes progress.
     */
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
  case PI_RT_PEND_A:
    return s->rt_pend[0];
  case PI_RT_PEND_B:
    return s->rt_pend[1];
  case PI_RT_EN_A:
    return s->rt_enable[0];
  case PI_RT_EN_B:
    return s->rt_enable[1];
  case PI_RT_COUNT:
    return sgi_hub_rtc_count();
  case PI_ERR_STACK_ADDR_A:
    return s->pi_err_stack[0];
  case PI_ERR_STACK_ADDR_B:
    return s->pi_err_stack[1];
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
  case PI_ERR_STACK_ADDR_A:
    s->pi_err_stack[0] = val;
    break;
  case PI_ERR_STACK_ADDR_B:
    s->pi_err_stack[1] = val;
    break;
  case PI_INT_PEND_MOD: {
    /* Set/clear a pending interrupt level. */
    uint64_t bit = val & 0x3f;
    bool set = (val & PI_INT_PEND_SET_BIT) != 0;
    if (getenv("SGI_HUB_INTDBG")) {
      qemu_log_mask(LOG_UNIMP,
                    "HUB INT_PEND_MOD val=0x%llx bit=%llu set=%d "
                    "pend0=0x%llx\n",
                    (unsigned long long)val, (unsigned long long)bit, set,
                    (unsigned long long)s->int_pend0);
    }
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
    qemu_log_mask(LOG_GUEST_ERROR,
                  "hub RT COMPARE_A=%llu rt_count=%llu pend=%u en=%u\n",
                  (unsigned long long)val,
                  (unsigned long long)sgi_hub_rtc_count(),
                  (unsigned)s->rt_pend[0], (unsigned)s->rt_enable[0]);
    s->rt_compare[0] = val;
    s->rt_pend[0] = 0;
    s->rt_armed[0] = (val != 0);
    sgi_hub_update_irqs(s);
    break;
  case PI_RT_COMPARE_B:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "hub RT COMPARE_B=%llu rt_count=%llu pend=%u en=%u\n",
                  (unsigned long long)val,
                  (unsigned long long)sgi_hub_rtc_count(),
                  (unsigned)s->rt_pend[1], (unsigned)s->rt_enable[1]);
    s->rt_compare[1] = val;
    s->rt_pend[1] = 0;
    s->rt_armed[1] = (val != 0);
    sgi_hub_update_irqs(s);
    break;
  case PI_RT_PEND_A:
    /* The OS acks the RTC interrupt by writing the pend bit 0; it must not
     * re-assert until COMPARE_A is armed again. */
    qemu_log_mask(LOG_GUEST_ERROR, "hub RT PEND_A write=%llu pend=%u\n",
                  (unsigned long long)val, (unsigned)s->rt_pend[0]);
    s->rt_pend[0] = 0;
    sgi_hub_update_irqs(s);
    break;
  case PI_RT_PEND_B:
    qemu_log_mask(LOG_GUEST_ERROR, "hub RT PEND_B write=%llu pend=%u\n",
                  (unsigned long long)val, (unsigned)s->rt_pend[1]);
    s->rt_pend[1] = 0;
    sgi_hub_update_irqs(s);
    break;
  case PI_RT_EN_A:
    s->rt_enable[0] = val;
    sgi_hub_update_irqs(s);
    break;
  case PI_RT_EN_B:
    s->rt_enable[1] = val;
    sgi_hub_update_irqs(s);
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
 * Dallas DS2502 1-wire slave (family 0x09).  One of these sits on the hub's
 * MD_MLAN_CTL line and another on the SN0 router's RR_NIC_ULAN line.  The
 * hub record is read by get_hub_nic_info() (libsk/ml/nic.c), which derives
 * the hub NIC from the DS2502 ROM serial (reported as "Laser:<hex>"); the
 * Name field carries "IP27" so the record is accepted.  The router record is
 * read by cache_router_nic() -> router_nic_get() -> nic_next(), which uses
 * only the ROM serial.  The bit protocol and state machine mirror the ARCS
 * source (nic.c) and the octane BaseIO model.
 */
enum {
  SGI_DS_IDLE = 0,
  SGI_DS_CMD,        /* shifting in an 8-bit ROM command */
  SGI_DS_READROM,    /* shifting out the 64-bit ROM id */
  SGI_DS_MATCHROM,   /* shifting in a 64-bit match-ROM sequence */
  SGI_DS_SEARCH,     /* ROM search: read bit, complement, write choice */
  SGI_DS_F0_PENDING, /* 0xf0 seen: next op decides search vs read-memory */
  SGI_DS_RMEM_ADDR,  /* shifting in the 16-bit memory address */
  SGI_DS_RMEM_DATA,  /* shifting out memory bytes */
};

/* --- Dallas/Maxim 1-wire CRC-8 (poly 0x8C, reflected) --- */
static uint8_t sgi_ds_crc8(const uint8_t *p, int n) {
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
static const int sgi_ds_oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                         1, 0, 0, 1, 0, 1, 1, 0};

static uint16_t sgi_ds_crc16_step(uint16_t crc, uint8_t in) {
  uint16_t data = in;
  data = (data ^ (crc & 0xff)) & 0xff;
  crc >>= 8;
  if (sgi_ds_oddparity[data & 0xf] ^ sgi_ds_oddparity[data >> 4]) {
    crc ^= 0xc001;
  }
  data <<= 6;
  crc ^= data;
  data <<= 1;
  crc ^= data;
  return crc;
}

static uint16_t sgi_ds_crc16(const uint8_t *p, int n) {
  uint16_t crc = 0;
  int i;
  for (i = 0; i < n; i++) {
    crc = sgi_ds_crc16_step(crc, p[i]);
  }
  return crc;
}

static void sgi_ds_put(uint8_t *dst, const char *s, int n) {
  int i;
  for (i = 0; i < n; i++) {
    dst[i] = s[i] ? (uint8_t)s[i] : ' ';
  }
}

/*
 * Build a DS2502 board record.  page 0: [0]=0x01, [1..10]=serial, [11..29]=
 * part; page 1: [0..5]=part cont., [6..9]=rev, [16..29]=name.  Each 32-byte
 * page carries a trailing 16-bit CRC so the NIC driver's crc16 over the page
 * equals 0xb001.  rom[0] is the family code and rom[7] the Dallas CRC-8.
 */
void sgi_ds2502_build_board(SGIDS2502 *ds, const char *serial,
                            const char *part, const char *name,
                            const uint8_t rom_serial[6]) {
  int a, b;

  memset(ds->mem, 0xff, sizeof(ds->mem));

  ds->mem[0] = 0x01;
  sgi_ds_put(&ds->mem[1], serial, 10);
  sgi_ds_put(&ds->mem[11], part, 19);

  sgi_ds_put(&ds->mem[32 + 0], "", 6);
  sgi_ds_put(&ds->mem[32 + 6], "0001", 4);
  ds->mem[32 + 10] = 0x00;
  memset(&ds->mem[32 + 11], 0x00, 4);
  ds->mem[32 + 15] = 0x00;
  sgi_ds_put(&ds->mem[32 + 16], name, 14);

  for (int page = 0; page < 2; page++) {
    uint8_t *pg = &ds->mem[page * 32];
    pg[30] = 0;
    pg[31] = 0;
    for (a = 0; a < 256; a++) {
      for (b = 0; b < 256; b++) {
        pg[30] = a;
        pg[31] = b;
        if (sgi_ds_crc16(pg, 32) == 0xb001) {
          goto done;
        }
      }
    }
  done:;
  }

  /* ROM id: family 0x09 + 48-bit serial + Dallas CRC-8. */
  ds->rom[0] = 0x09;
  memcpy(&ds->rom[1], rom_serial, 6);
  ds->rom[7] = sgi_ds_crc8(ds->rom, 7);
}

void sgi_ds2502_reset(SGIDS2502 *ds) {
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

static void sgi_ds2502_decode(SGIDS2502 *ds) {
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

void sgi_ds2502_write_bit(SGIDS2502 *ds, int bit) {
  switch (ds->state) {
  case SGI_DS_CMD:
    ds->cmd |= (bit & 1) << ds->cmd_bits;
    if (++ds->cmd_bits == 8) {
      sgi_ds2502_decode(ds);
    }
    break;
  case SGI_DS_MATCHROM:
    ds->in |= (bit & 1) << ds->in_bits;
    if (++ds->in_bits == 64) {
      ds->state = SGI_DS_CMD;
    }
    break;
  case SGI_DS_SEARCH:
    if (ds->search_phase == 2) {
      ds->search_phase = 0;
      if (++ds->out_index == 64) {
        ds->state = SGI_DS_CMD;
      }
    }
    break;
  case SGI_DS_F0_PENDING:
    ds->state = SGI_DS_RMEM_ADDR;
    ds->addr = 0;
    ds->in_bits = 0;
    /* fall through */
  case SGI_DS_RMEM_ADDR:
    ds->addr |= (bit & 1) << ds->in_bits;
    if (++ds->in_bits == 16) {
      ds->state = SGI_DS_RMEM_DATA;
      ds->out_index = 0;
      ds->extra = (ds->rom[0] == 0x09) ? 8 : 0;
    }
    break;
  default:
    break;
  }
}

int sgi_ds2502_read_bit(SGIDS2502 *ds) {
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
    ds->state = SGI_DS_SEARCH;
    ds->out_index = 0;
    ds->search_phase = 0;
    return sgi_ds2502_read_bit(ds);
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
    return 1;
  }
}

/*
 * MCR_PACK command: PULSE<19:10>/SAMPLE<9:2>.  Returns the latched line state
 * (MCR_DATA, bit 0); the caller asserts MCR_DONE (bit 1).
 */
int sgi_ds2502_mcr(SGIDS2502 *ds, uint64_t val) {
  unsigned pulse = (val >> 10) & 0x3ff;
  unsigned sample = (val >> 2) & 0xff;

  if (pulse >= 480) {
    /* reset/presence pulse; single device => presence bit 0 */
    sgi_ds2502_reset(ds);
  } else if (sample == 30) {
    sgi_ds2502_write_bit(ds, 0);
    ds->data_bit = 0;
  } else if (sample == 110) {
    sgi_ds2502_write_bit(ds, 1);
    ds->data_bit = 0;
  } else if (sample == 13) {
    ds->data_bit = sgi_ds2502_read_bit(ds);
  } else {
    ds->data_bit = 0;
  }
  return ds->data_bit;
}

/* Hub NIC record: board serial/part/name + family-0x09 ROM id. */
static void sgi_hub_ds_init(SGIHubState *s) {
  static const uint8_t rom_serial[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  sgi_ds2502_build_board(&s->ds, "1234567890", "030-1055-001",
                         "IP27 ORIGIN2K", rom_serial);
}

static void sgi_hub_mlan_write(SGIHubState *s, uint64_t val) {
  s->ds.data_bit = sgi_ds2502_mcr(&s->ds, val);
}

/* ---- PCF8584 I2C controller + ELSC NVRAM (libkl/ml/i2c.c, elsc.c) ---- */

/* Does this hub port an ELSC NVRAM page?  The supervisor EEPROM answers at
 * I2C_ADDR_RAM..I2C_ADDR_RAM+7 (pages 0..7); everything else NAKs. */
static int sgi_hub_i2c_slave_present(const SGIHubState *s) {
  return s->elsc && s->i2c_slave >= I2C_ADDR_RAM &&
         s->i2c_slave <= (I2C_ADDR_RAM + 7);
}

/*
 * The ELSC is the I2C super-master: rather than one master owning the bus it
 * writes a token (I2C_ARB_ADDR * 2 + I2C_CPU_CODE, optionally + the message
 * indication bit) into every PCF8584's data register to hand out a time slice.
 * i2c_arb() first waits for the token to drop, then for one to appear and be
 * held for I2C_SIGNAL_TIME (250 us).  Present it for 1 ms in every 2 ms so both
 * edges are observable; the local master is slot n1 / CPU 0, so cpu_code is 0.
 */
static uint8_t sgi_hub_i2c_sig(void) { return (uint8_t)(I2C_ARB_ADDR * 2); }

static uint64_t sgi_hub_i2c_token(void) {
  uint64_t t_us = qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1000;

  return ((t_us % 2000) < 1000) ? (uint64_t)sgi_hub_i2c_sig() : 0;
}

static bool sgi_hub_i2c_trace(void) {
  static int enabled = -1;

  if (enabled < 0) {
    enabled = getenv("IP27_I2C_TRACE") ? 1 : 0;
  }
  return enabled;
}

/* Latch a slave address/direction byte (S0) as the transaction address. */
static void sgi_hub_i2c_address(SGIHubState *s, uint8_t v) {
  s->i2c_slave = (v >> 1) & 0x7f;
  s->i2c_rw = v & 1;
  s->i2c_phase = 0;
  s->i2c_rx_count = 0;
  s->i2c_active = 1;
  if (sgi_hub_i2c_trace()) {
    fprintf(stderr, "sgi-hub i2c addr n%d: slave=0x%02x rw=%d (byte 0x%02x)\n",
            s->nasid, s->i2c_slave, s->i2c_rw, v);
  }
}

static uint64_t sgi_hub_i2c_a0_read(SGIHubState *s) {
  uint64_t ret;

  if (s->i2c_ctl & I2C_CTL_ESO) {
    /* Data register (S0). */
    if (s->i2c_active && s->i2c_rw) {
      if (s->i2c_rx_count == 0) {
        /* The slave address byte is read (and discarded) first. */
        s->i2c_rx_count = 1;
        ret = (uint8_t)((s->i2c_slave << 1) | 1);
      } else if (sgi_hub_i2c_slave_present(s)) {
        ret = s->elsc->nvram[s->i2c_memaddr];
        if (sgi_hub_i2c_trace() && s->i2c_rx_count <= 1) {
          fprintf(stderr, "sgi-elsc n%d read 0x%03x = 0x%02x\n", s->nasid,
                  s->i2c_memaddr, (unsigned)ret);
        }
        s->i2c_memaddr = (s->i2c_memaddr + 1) & (SGI_ELSC_NVRAM_SIZE - 1);
        s->i2c_rx_count++;
      } else {
        ret = 0xff;
      }
    } else {
      /* Idle: the super-master's token lives in the data register. */
      ret = sgi_hub_i2c_token();
    }
  } else if (s->i2c_ctl & I2C_CTL_ES1) {
    ret = s->i2c_clock;
  } else if (s->i2c_ctl & I2C_CTL_ES2) {
    ret = 0; /* interrupt vector, unused */
  } else {
    ret = s->i2c_own;
  }

  return ret;
}

static void sgi_hub_i2c_a0_write(SGIHubState *s, uint8_t v) {
  if (s->i2c_ctl & I2C_CTL_ESO) {
    /* Data register (S0).  The driver echoes the super-master's token into S0
     * (and reads it back) before every address/data byte; the token is never a
     * real payload byte here, so drop it and keep waiting for the real one. */
    s->i2c_data = v;
    if (v == sgi_hub_i2c_sig()) {
      return;
    }
    if (!s->i2c_active) {
      /* Before the start the driver writes the arbitration token and then the
       * real address; the PCF8584 transmits only the last byte written, so
       * remember it and turn it into the address when the start arrives. */
      s->i2c_have_addr = 1;
    } else if (s->i2c_need_addr) {
      /* A repeated start was issued; this byte is the new address. */
      s->i2c_need_addr = 0;
      sgi_hub_i2c_address(s, v);
    } else if (s->i2c_rw == 0) {
      if (s->i2c_phase == 0) {
        /* Master transmit: first byte after the address is the NVRAM offset. */
        s->i2c_memaddr =
            (((s->i2c_slave & 7) << 8) | v) & (SGI_ELSC_NVRAM_SIZE - 1);
        s->i2c_phase = 1;
        if (sgi_hub_i2c_trace()) {
          fprintf(stderr, "sgi-elsc n%d offset 0x%03x\n", s->nasid,
                  s->i2c_memaddr);
        }
      } else if (sgi_hub_i2c_slave_present(s)) {
        if (sgi_hub_i2c_trace()) {
          fprintf(stderr, "sgi-elsc n%d write 0x%03x = 0x%02x\n", s->nasid,
                  s->i2c_memaddr, v);
        }
        s->elsc->nvram[s->i2c_memaddr] = v;
        s->i2c_memaddr = (s->i2c_memaddr + 1) & (SGI_ELSC_NVRAM_SIZE - 1);
      }
    }
  } else if (s->i2c_ctl & I2C_CTL_ES1) {
    s->i2c_clock = v;
  } else if (s->i2c_ctl & I2C_CTL_ES2) {
    /* interrupt vector, unused */
  } else {
    s->i2c_own = v;
  }
}

static uint64_t sgi_hub_i2c_a1_read(SGIHubState *s) {
  /* Bus free, no error/arbitration-loss/addressed-as-slave; operations
   * complete instantly so PIN is never pending.  A transaction to a slave
   * that does not answer NAKs (LRB set) so the driver bails immediately. */
  uint64_t status = I2C_STA_BNB;

  if (s->i2c_active && !sgi_hub_i2c_slave_present(s)) {
    status |= I2C_STA_LRB;
  }
  return status;
}

static void sgi_hub_i2c_a1_write(SGIHubState *s, uint8_t v) {
  if (sgi_hub_i2c_trace()) {
    fprintf(stderr, "sgi-hub i2c A1w n%d: ctl=0x%02x\n", s->nasid, v);
  }
  s->i2c_ctl = v;
  if (v & I2C_CTL_STA) {
    if (s->i2c_active) {
      /* A repeated start: the address byte follows the start. */
      s->i2c_need_addr = 1;
    } else if (s->i2c_have_addr) {
      /* The address was written before its start (last byte wins). */
      sgi_hub_i2c_address(s, s->i2c_data);
      s->i2c_have_addr = 0;
    }
  }
  if (v & I2C_CTL_STO) {
    s->i2c_active = 0;
    s->i2c_rw = 0;
    s->i2c_have_addr = 0;
    s->i2c_need_addr = 0;
  }
}

void sgi_elsc_init(SGIElscState *e, uint8_t module, uint8_t partition) {
  memset(e->nvram, 0, sizeof(e->nvram));
  e->nvram[SGI_ELSC_MAGIC_AD] = SGI_ELSC_MAGIC_NO;
  e->nvram[SGI_ELSC_MODULE_AD] = module;
  e->nvram[SGI_ELSC_PARTITION_AD] = partition;
}

void sgi_hub_set_elsc(SGIHubState *s, SGIElscState *e) { s->elsc = e; }

static uint64_t sgi_hub_md_read(SGIHubState *s, hwaddr off) {
  if (off == I2C_A0_OFF) {
    return sgi_hub_i2c_a0_read(s);
  }
  if (off == I2C_A1_OFF) {
    return sgi_hub_i2c_a1_read(s);
  }
  if (off == MD_MEMORY_CONFIG) {
    return s->mem_config;
  }
  if (off == MD_REFRESH_CONTROL) {
    return s->refresh_ctl;
  }
  if (off == MD_SLOTID_USTAT) {
    return s->slotid_ustat;
  }
  /* MD_LED0 (0x220050): 8-bit front-panel LED; the PROM blinks/polls it. */
  if (off == MD_LED0) {
    return s->md_led0 & 0xff;
  }
  /*
   * Performance counters (MD_PERF_CNT0..5): with MD_PERF_SEL=0 they are not
   * counting, and the PROM uses them as read-modify-write scratch (hub lock
   * register, memory-test disable mask, pass counter).  Hold the last value
   * written; do not free-run, or the lock/disable bits it stores change under
   * it.
   */
  if (off == MD_PERF_SEL) {
    return s->md_perf_sel;
  }
  if (off >= MD_PERF_CNT0 && off <= MD_PERF_CNT5 && ((off - MD_PERF_CNT0) % 8) == 0) {
    return s->md_perf_cnt[(off - MD_PERF_CNT0) / 8];
  }
  if (off == MD_MLAN_CTL) {
    /* DONE (bit1) always set; RD_DATA (bit0) is the latched 1-wire line. */
    return 0x2 | (s->ds.data_bit & 1);
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    /* uController/UART and MLAN registers: state held, semantic model later. */
    return 0;
  }
  if (off >= 0x200040 && off < 0x2000a0) {
    /*
     * MD error-status/clear walk (hubmd.h): the PROM reads each status
     * register then writes its clear.  No directory/protocol/mem/misc/DIMM-init
     * errors are ever pending, so reads return 0 and clears are no-ops.
     */
    return 0;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD read @0x%" HWADDR_PRIx
                           "\n",
                off);
  return 0;
}

static void sgi_hub_md_write(SGIHubState *s, hwaddr off, uint64_t val,
                             unsigned size) {
  if (off == I2C_A0_OFF) {
    sgi_hub_i2c_a0_write(s, val & 0xff);
    return;
  }
  if (off == I2C_A1_OFF) {
    sgi_hub_i2c_a1_write(s, val & 0xff);
    return;
  }
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
  if (off == MD_LED0) {
    s->md_led0 = val & 0xff;
    return;
  }
  if (off == MD_PERF_SEL) {
    s->md_perf_sel = val;
    return;
  }
  if (off >= MD_PERF_CNT0 && off <= MD_PERF_CNT5 && ((off - MD_PERF_CNT0) % 8) == 0) {
    s->md_perf_cnt[(off - MD_PERF_CNT0) / 8] = val;
    return;
  }
  if ((off >= MD_UREG0_0 && off <= MD_UREG0_7) ||
      (off >= MD_UREG1_0 && off <= MD_UREG1_15)) {
    return;
  }
  if (off >= 0x200040 && off < 0x2000a0) {
    /* MD error-status/clear walk (see the read side): clears absorbed. */
    return;
  }
  qemu_log_mask(LOG_UNIMP, "sgi-hub: unimplemented MD write @0x%" HWADDR_PRIx
                           " = 0x%" PRIx64 "\n",
                off, val);
}

static uint64_t sgi_hub_read_off(SGIHubState *s, hwaddr off);
static void sgi_hub_write_off(SGIHubState *s, hwaddr off, uint64_t val);

/* ---- SN0 router register file (reached only through vectors) ---- */

void sgi_router_init(SGIRouterState *r, uint32_t nic, uint32_t chipin,
                     uint32_t revision) {
  if (!r->regs) {
    r->regs = g_new0(uint64_t, SGI_ROUTER_REG_WORDS);
  }
  r->nic = nic;
  r->chipin = chipin;
  r->revision = revision;
  /*
   * Cache the NIC in RR_SCRATCH_REG0 so cache_router_nic() can read it
   * directly and skip the ~0.235 s 1-wire read.  Discovery may still clear
   * SCR0 (the router register walk) and re-read through router_nic_get() ->
   * RR_NIC_ULAN, so the router's own DS2502 slave is modelled as well.
   */
  r->regs[RR_SCRATCH_REG0 / 8] = nic & RSCR0_NIC_MASK;

  /* Router NIC DS2502 record: a distinct serial/part/name from the hub's. */
  {
    static const uint8_t rom_serial[6] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

    sgi_ds2502_build_board(&r->ds, "2000000001", "030-1100-001",
                           "IP27 ROUTER", rom_serial);
    sgi_ds2502_reset(&r->ds);
  }
}

void sgi_router_connect(SGIRouterState *r, int port, SGIHubState *hub) {
  if (port >= 1 && port <= SGI_ROUTER_PORTS) {
    r->port[port] = hub;
  }
}

void sgi_router_reset(SGIRouterState *r) {
  if (!r || !r->regs) {
    return;
  }
  memset(r->regs, 0, SGI_ROUTER_REG_WORDS * sizeof(uint64_t));
  r->regs[RR_SCRATCH_REG0 / 8] = r->nic & RSCR0_NIC_MASK;
  sgi_ds2502_reset(&r->ds);
}

void sgi_hub_set_router(SGIHubState *s, SGIRouterState *r) { s->router = r; }

/* Which router port this hub is plugged into (0 = not connected). */
static int sgi_router_hub_port(SGIRouterState *r, SGIHubState *hub) {
  int p;

  for (p = 1; p <= SGI_ROUTER_PORTS; p++) {
    if (r->port[p] == hub) {
      return p;
    }
  }
  return 0;
}

static uint64_t sgi_router_read(SGIRouterState *r, uint64_t off, int inport) {
  uint64_t idx = off / 8;

  if (idx >= SGI_ROUTER_REG_WORDS) {
    return 0;
  }
  if (off == RR_STATUS_REV_ID) {
    uint64_t v = ((uint64_t)(inport & 7) << RSRI_INPORT_SHFT) |
                 ((uint64_t)(r->chipin & 0xf) << RSRI_CHIPIN_SHFT) |
                 ((uint64_t)(r->revision & 0xf) << RSRI_CHIPREV_SHFT) |
                 RSRI_CHIPID_ROUTER;
    int p;

    for (p = 1; p <= SGI_ROUTER_PORTS; p++) {
      if (r->port[p]) {
        v |= RSRI_LINKWORKING(p) | RSRI_LINK8BIT(p);
      }
    }
    return v;
  }
  if (off == RR_SCRATCH_REG1) {
    return r->regs[idx] & 0xffff;
  }
  if (off == RR_NIC_ULAN) {
    /*
     * Router Dallas 1-wire line (router_nic_access, nic.c).  MCR_DONE (bit 1)
     * is always set so the PROM's poll terminates; MCR_DATA (bit 0) is the
     * latched state of the DS2502 slave.
     */
    return 0x2 | (r->ds.data_bit & 1);
  }
  /*
   * RR_HISTOGRAM(_l): the high 16 bits are a free-running network-clock sample
   * counter.  router_test() uses a nonzero value to detect the backplane type;
   * model it from the host clock (1 microsecond granularity) rather than a
   * fixed constant, since it is a measured rate, not an identity.
   */
  if ((off & 0xffff) == 0x0010 && off >= 0x10000) {
    uint64_t c = (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1000) & 0xffff;

    return c << 48;
  }
  return r->regs[idx];
}

static void sgi_router_write(SGIRouterState *r, uint64_t off, uint64_t val,
                             int inport) {
  uint64_t idx = off / 8;

  if (idx >= SGI_ROUTER_REG_WORDS) {
    return;
  }
  if (off == RR_SCRATCH_REG1) {
    r->regs[idx] = val & 0xffff;
    return;
  }
  if (off == RR_NIC_ULAN) {
    /* Drive the router 1-wire line: decode one MCR_PACK(pulse, sample). */
    sgi_ds2502_mcr(&r->ds, val);
    return;
  }
  r->regs[idx] = val;
}

/*
 * Execute a NI vector PIO operation (armed via NI_VECTOR/NI_VECTOR_PARMS).
 *
 * Single node (no router attached): only the local node answers, and a path-0
 * NSRI read deliberately reports an unknown chip id so discover_object() leaves
 * the sole hub unlinked and nasid_assign() keeps it at NASID 0.
 *
 * Router attached: the vector path is the source route the PROM/kernel builds
 * with discover_route().  Path 0 addresses the hub's own link, i.e. the router;
 * a nonzero path is the sequence of router exit ports to take (nibble 0 is the
 * first hop), and the object reached by the last hop is the destination.  This
 * is a pure register-level switch -- no latency, no coherence.
 */
static void sgi_hub_ni_vector_go(SGIHubState *s, uint64_t parms) {
  uint64_t pioid = (parms >> NVP_PIOID_SHFT) & 0x7ff;
  uint64_t wid = (parms >> NVP_WRITEID_SHFT) & 0xff;
  uint64_t addr = parms & NVP_ADDRESS_MASK;
  unsigned type = parms & NVP_TYPE_MASK;
  uint64_t ret = 0;
  hwaddr reg;

  if (!s->router) {
    if (s->ni_vector != 0) {
      s->ni_vector_status = 0;
      return;
    }
    reg = (addr < SGI_HUB_NI_BASE) ? SGI_HUB_NI_BASE + addr : addr;
    switch (type) {
    case PIOTYPE_READ:
      if (reg == NI_STATUS_REV_ID) {
        s->ni_vector_rd_data =
            ((uint64_t)(s->nasid & 0x1ff) << NSRI_NODEID_SHFT) |
            ((uint64_t)SGI_HUB_REV << NSRI_REV_SHFT) | 0x5;
      } else {
        s->ni_vector_rd_data = sgi_hub_read_off(s, reg);
      }
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
    return;
  }

  {
    SGIRouterState *rtr = s->router;
    SGIHubState *dest_hub = NULL;
    SGIRouterState *dest_rtr = NULL;
    int inport = sgi_router_hub_port(rtr, s);
    uint64_t path = s->ni_vector;

    if (path == 0) {
      dest_rtr = rtr;
    } else {
      int len = 0, i;

      while ((path >> (4 * len)) & 0xf) {
        len++;
      }
      for (i = 0; i < len; i++) {
        int port = (path >> (4 * i)) & 0xf;
        SGIHubState *h = (port >= 1 && port <= SGI_ROUTER_PORTS)
                             ? rtr->port[port]
                             : NULL;

        if (!h || i != len - 1) {
          /* Only a single router is modelled. */
          s->ni_vector_status = 0;
          return;
        }
        dest_hub = h;
        inport = port;
      }
    }

    if (dest_rtr) {
      switch (type) {
      case PIOTYPE_READ:
        ret = sgi_router_read(dest_rtr, addr, inport);
        break;
      case PIOTYPE_WRITE:
        sgi_router_write(dest_rtr, addr, s->ni_vector_data, inport);
        break;
      case PIOTYPE_XCHG:
        /*
         * A vector PIO exchange is a conditional swap, not a write: the
         * SN0 PROM builds the router lock on it (libkl/ml/vector.c,
         * vector_exch_node): it is defined to write the supplied value
         * "provided that the current content of the register is zero", and
         * to return the prior content.  router_lock() (router.c) reads a
         * zero back as "lock acquired" and spins while it reads nonzero, so
         * honouring the zero-only write is what makes RR_SCRATCH_REG1 lock
         * and unlock like the hardware the PROM expects.
         */
        ret = sgi_router_read(dest_rtr, addr, inport);
        if (ret == 0) {
          sgi_router_write(dest_rtr, addr, s->ni_vector_data, inport);
        }
        break;
      default:
        s->ni_vector_status = 0;
        return;
      }
    } else {
      reg = (addr < SGI_HUB_NI_BASE) ? SGI_HUB_NI_BASE + addr : addr;
      switch (type) {
      case PIOTYPE_READ:
        ret = sgi_hub_read_off(dest_hub, reg);
        break;
      case PIOTYPE_WRITE:
        sgi_hub_write_off(dest_hub, reg, s->ni_vector_data);
        break;
      case PIOTYPE_XCHG:
        ret = sgi_hub_read_off(dest_hub, reg);
        sgi_hub_write_off(dest_hub, reg, s->ni_vector_data);
        break;
      default:
        s->ni_vector_status = 0;
        return;
      }
    }
    s->ni_vector_rd_data = ret;
  }

  s->ni_vector_status = NVS_VALID | (pioid << NVS_PIOID_SHFT) |
                        (wid << NVS_WRITEID_SHFT) |
                        (addr & NVS_ADDRESS_MASK) | (type & NVS_TYPE_MASK);
}

/* Implemented by the IP27 machine: CPU-local reset re-running the PROM. */
extern void sgi_ip27_local_reset(void);

static void sgi_hub_reset_bh(void *opaque) {
  /* Deferred so the reset runs in the main loop with vCPUs stopped. */
  qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static uint64_t sgi_hub_ni_read(SGIHubState *s, hwaddr off) {
  /* Routing tables distributed by distribute_tables() (hubni.h). */
  if (off >= NI_META_TABLE0 && off < NI_META_TABLE0 + NI_META_ENTRIES * 8) {
    return s->ni_meta_table[(off - NI_META_TABLE0) / 8];
  }
  if (off >= NI_LOCAL_TABLE0 && off < NI_LOCAL_TABLE0 + NI_LOCAL_ENTRIES * 8) {
    return s->ni_local_table[(off - NI_LOCAL_TABLE0) / 8];
  }
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
    /*
     * Single node (no router): the PROM's port-0 self-probe can leave the
     * ADVERT_NASID field [15:0] as 1, which makes the PROM remap its window to
     * a node-1 address.  Report the field as 0 so the node stays at NASID 0.
     *
     * Two nodes: the same low 16 bits are ADVERT_OBJECTS_MASK, and the peer
     * discovery handshake reads a remote hub's DISCDONE flag and object count
     * from here, so the value must be returned intact.
     */
    if (s->router) {
      return s->ni_scratch[1];
    }
    return s->ni_scratch[1] & ~0xffffULL;
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
  if (off >= NI_META_TABLE0 && off < NI_META_TABLE0 + NI_META_ENTRIES * 8) {
    s->ni_meta_table[(off - NI_META_TABLE0) / 8] = val;
    return;
  }
  if (off >= NI_LOCAL_TABLE0 && off < NI_LOCAL_TABLE0 + NI_LOCAL_ENTRIES * 8) {
    s->ni_local_table[(off - NI_LOCAL_TABLE0) / 8] = val;
    return;
  }
  switch (off) {
  case NI_PORT_RESET:
    /*
     * reset_system() writes NPR_PORTRESET|NPR_LOCALRESET; NPR_LOCALRESET
     * (bit 0) resets the hub/system.  Defer a CPU-local reset (re-run the
     * PROM from the reset vector); no full machine/device reset, so the
     * serial chardev and the flash (ip27log) survive.
     */
    if (val & 0x1) {
      qemu_bh_schedule(s->reset_bh);
    }
    break;
  case NI_PROTECTION:
    break;
  case NI_SCRATCH_REG0:
    s->ni_scratch[0] = val;
    break;
  case NI_SCRATCH_REG1:
    if (sgi_hub_i2c_trace()) {
      fprintf(stderr, "sgi-hub n%d SR1 = 0x%" PRIx64 "\n", s->nasid, val);
    }
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
  case NI_VECTOR_READ_DATA:
    /* Write-only vector-PIO probe (hubni.h); fire-and-forget, no readback. */
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

/*
 * Hub II block-transfer engine.  Per BTE (0/1): IBLS (length/status) at +0,
 * IBSA (source) +8, IBDA (destination) +0x10, IBCT (control/terminate) +0x18,
 * IBNA (notification) +0x20, IBIA (interrupt) +0x28 (sys/SN/SN0/hubio.h).
 * The kernel programs SRC/DEST, writes IBLS=IBLS_BUSY|(len>>7), then IBCT to
 * start; on completion the engine must (a) have moved the data and (b) post the
 * status word to the IBNA physical address, because bte_wait_for_status() spins
 * on ctx->status (the IBNA word) until it is no longer -1.  A model that only
 * latched the registers would report "busy" forever, or (if it cleared BUSY)
 * claim success while silently dropping the transfer -- which is exactly how
 * the page-migration copies were being lost on two-node.
 */
static bool sgi_hub_bte_offset(hwaddr off, int *n, hwaddr *reg) {
  if (off >= IIO_IBLS_0 && off < IIO_IBLS_0 + 6 * 8) {
    *n = 0;
    *reg = off - IIO_IBLS_0;
    return true;
  }
  if (off >= IIO_IBLS_0 + IIO_BTE_STRIDE &&
      off < IIO_IBLS_0 + IIO_BTE_STRIDE + 6 * 8) {
    *n = 1;
    *reg = off - (IIO_IBLS_0 + IIO_BTE_STRIDE);
    return true;
  }
  return false;
}

static void sgi_hub_bte_run(SGIHubState *s, int n) {
  uint64_t src = s->bte_src[n];
  uint64_t dest = s->bte_dest[n];
  uint64_t len = (s->bte_stat[n] & IBLS_LENGTH_MASK) << BTE_LEN_SHIFT;
  uint64_t status = 0; /* IBLS success */

  /*
   * Diagnostic (SGI_HUB_BTE_FAIL=1): report the transfer as failed without
   * copying, so the kernel's bte_pbcopy() falls back to its CPU bcopy.  Lets a
   * run A/B whether a lost migration copy is the BTE or something else.
   */
  if (getenv("SGI_HUB_BTE_FAIL")) {
    status = IBLS_ERROR;
    goto complete;
  }

  if (len) {
    uint8_t *buf = g_malloc(len);

    address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
    address_space_write(&address_space_memory, dest, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
    g_free(buf);
  } else {
    status = IBLS_ERROR;
  }

complete:
  /* Completion: clear busy/length, post the status to the notification word. */
  s->bte_stat[n] = status;
  if (s->bte_notify[n]) {
    uint64_t be = cpu_to_be64(status);

    address_space_write(&address_space_memory, s->bte_notify[n],
                        MEMTXATTRS_UNSPECIFIED, &be, sizeof(be));
  }

  if (getenv("SGI_HUB_BTEDBG")) {
    /* fprintf(stderr): keep BTE copies in the same ordered stream as the
     * IP27_WW plugin and IP27_TLB printer so a store/dest/remap can be
     * correlated per ASID. */
    fprintf(stderr, "sgi-hub: BTE%d copy src=0x%016" PRIx64 " dest=0x%016"
            PRIx64 " len=%" PRIu64 "\n",
            n, src, dest, len);
  }
}

static uint64_t sgi_hub_ii_read(SGIHubState *s, hwaddr off) {
  int n;
  hwaddr reg;

  if (sgi_hub_bte_offset(off, &n, &reg)) {
    switch (reg) {
    case 0x00:
      return s->bte_stat[n];
    case 0x08:
      return s->bte_src[n];
    case 0x10:
      return s->bte_dest[n];
    case 0x18:
      return 0; /* IBCT reads back 0 */
    case 0x20:
      return s->bte_notify[n];
    case 0x28:
      return s->bte_int[n];
    }
  }

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
  int n;
  hwaddr reg;

  if (sgi_hub_bte_offset(off, &n, &reg)) {
    switch (reg) {
    case 0x00: /* IBLS: length/status (IBLS_BUSY | len>>7) */
      s->bte_stat[n] = val & (IBLS_BUSY | IBLS_ERROR | IBLS_LENGTH_MASK);
      break;
    case 0x08:
      s->bte_src[n] = val;
      break;
    case 0x10:
      s->bte_dest[n] = val;
      break;
    case 0x18: /* IBCT: start the transfer if armed */
      if (s->bte_stat[n] & IBLS_BUSY) {
        sgi_hub_bte_run(s, n);
      }
      break;
    case 0x20:
      s->bte_notify[n] = val;
      break;
    case 0x28:
      s->bte_int[n] = val;
      break;
    }
    return;
  }

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
    /*
     * Keep the modelled link reported as working only when an IO board is
     * actually attached; a node with nothing on its xtalk port reports the
     * link down so the kernel's early_probe_for_widget() takes the
     * XWIDGET_PART_NUM_NONE path instead of probing an unbacked widget.
     */
    s->ii_ilcsr = val | (s->io_attached ? IIO_ILCSR_LINK_WORKING : 0);
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

/* Public register-file access, used by the router when it targets a hub. */
uint64_t sgi_hub_reg_read(SGIHubState *s, uint64_t off) {
  return sgi_hub_read_off(s, off);
}

void sgi_hub_reg_write(SGIHubState *s, uint64_t off, uint64_t val) {
  sgi_hub_write_off(s, off, val);
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

  s->md_perf_sel = 0;
  for (i = 0; i < 6; i++) {
    s->md_perf_cnt[i] = 0;
  }

  s->cpu_present[0] = (s->num_cpus >= 1) ? 1 : 0;
  s->cpu_enable[0] = (s->num_cpus >= 1) ? 1 : 0;
  s->cpu_present[1] = (s->num_cpus > 1) ? 1 : 0;
  s->cpu_enable[1] = (s->num_cpus > 1) ? 1 : 0;

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    s->rt_compare[i] = 0;
    s->rt_enable[i] = 0;
    s->rt_pend[i] = 0;
    s->rt_armed[i] = 0;
  }

  /*
   * The PROM's entry (entry.s) performs a one-time reset unless
   * PI_ERR_STACK_ADDR_B already reads "Rst0"; model the post-first-reset
   * state so it goes straight to normal init.
   */
  s->pi_err_stack[0] = 0;
  s->pi_err_stack[1] = PI_ERR_STACK_RST0;

  for (i = 0; i < NI_META_ENTRIES; i++) {
    s->ni_meta_table[i] = 0;
  }
  for (i = 0; i < NI_LOCAL_ENTRIES; i++) {
    s->ni_local_table[i] = 0;
  }

  /*
   * MD_SLOTID_USTAT: bit 4 = FPGA/flash ready; bits [2:0] are the node board's
   * slot id.  The kernel's hub_slotbits_to_slot() masks MSU_SN0_SLOTID_MASK (7)
   * and indexes nodeslot_table[], whose entries 0 and 1 are
   * SLOTNUM_INVALID_CLASS -- so slot id 0 produced "/hw/module/../slot/Invalid0"
   * for every board, which is why no hwgraph bridge vertex could be built.  A
   * single-node Origin 200 is the MotherBoard, whose node slot is 1 (index 7,
   * SLOTNUM_NODE_CLASS|1).
   */
  s->slotid_ustat = 0x10 | (s->slot_id & 0x7);

  /* PCF8584 I2C controller: powered up idle (no transaction, no own address). */
  s->i2c_ctl = 0;
  s->i2c_own = 0;
  s->i2c_clock = 0;
  s->i2c_data = 0;
  s->i2c_have_addr = 0;
  s->i2c_need_addr = 0;
  s->i2c_active = 0;
  s->i2c_rw = 0;
  s->i2c_slave = 0;
  s->i2c_phase = 0;
  s->i2c_memaddr = 0;
  s->i2c_rx_count = 0;

  /* II: hub widget id, working XIO link, all widgets accessible. */
  s->ii_wcr = HUB_XIO_WIDGET_ID;
  s->ii_iowa = 0x1ff;
  s->ii_iiwa = 0x1ff;
  s->ii_ilcsr = s->io_attached ? IIO_ILCSR_LINK_WORKING : 0;
  s->ii_scratch[0] = 0;
  s->ii_scratch[1] = 0;

  for (i = 0; i < 2; i++) {
    s->bte_src[i] = 0;
    s->bte_dest[i] = 0;
    s->bte_stat[i] = 0;
    s->bte_notify[i] = 0;
    s->bte_int[i] = 0;
  }

  /*
   * The attached router is machine-owned, not a DeviceState, so it is not
   * reset by the device-reset walk.  The kernel's use of reset_system() (and
   * the panic-restart path) ends in qemu_system_reset_request(), which resets
   * this hub but otherwise leaves the router register file intact --
   * including RR_SCRATCH_REG1 held at 0xffff by discovery's fence, which then
   * makes the recovery boot's cache_router_nic() time out and fall back to
   * single-node.  Reset the router's volatile state here so every device reset
   * also releases the router lock and re-seeds RR_SCRATCH_REG0.
   */
  sgi_router_reset(s->router);

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
  sgi_ds2502_reset(&s->ds);
  /* Deferred CPU-local reset (NI_PORT_RESET); the BH lives for the machine. */
  s->reset_bh = qemu_bh_new(sgi_hub_reset_bh, s);

  for (i = 0; i < SGI_HUB_MAX_CPUS; i++) {
    for (j = 0; j < 5; j++) {
      sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i][j]);
    }
  }

  /*
   * The RTC counts QEMU_CLOCK_REALTIME (see sgi_hub_rtc_count), so its poll
   * timer must live on the same clock: a timer's clock type is fixed at
   * creation, and timer_mod() interprets the deadline in that type.  With a
   * VIRTUAL timer rescheduled with a REALTIME deadline the next expiry sat
   * ~host-uptime ns in the future (never, on the small virtual timeline), so
   * PI_RT_PEND_x was never latched: the L4 clock tick stopped after the first
   * arm, lbolt/time froze, and every timeout waiter (e.g. XFS log flush)
   * slept forever.
   */
  s->rt_timer = timer_new_ns(QEMU_CLOCK_REALTIME, sgi_hub_rtc_timer, s);
  timer_mod(s->rt_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1000000);
}

static const Property sgi_hub_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIHubState, nasid, 0),
    DEFINE_PROP_UINT32("num-cpus", SGIHubState, num_cpus, 1),
    DEFINE_PROP_UINT32("slot-id", SGIHubState, slot_id, 7),
    DEFINE_PROP_UINT64("mem-config", SGIHubState, mem_config, 0),
    DEFINE_PROP_BOOL("io-attached", SGIHubState, io_attached, true),
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
