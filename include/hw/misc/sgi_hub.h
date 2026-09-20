/*
 * SGI IP27 (Origin 200 / Origin 2000 / Onyx2) "Hub" (Bedrock) ASIC model.
 *
 * Each IP27 node pairs one or two R10000/R12000 CPUs with local memory and a
 * Hub ASIC; nodes are linked by CrayLink/NUMAlink.  The hub owns global
 * addressing, routing, the processor interface (PI), the memory/directory
 * interface (MD), the network interface (NI) and the I/O interface (II).
 *
 * Register offsets and semantics are taken from the Hub spec as mirrored by
 * Linux arch/mips/include/asm/sn/sn0/hub{pi,md,ni,io}.h.  This device exposes
 * the hub register file (widget 1 of a node's small window); address decode of
 * the surrounding SN0 spaces lives in hw/mips/sgi_ip27.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_HUB_H
#define HW_MISC_SGI_HUB_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_HUB "sgi-hub"
OBJECT_DECLARE_SIMPLE_TYPE(SGIHubState, SGI_HUB)

/* Hub widget window: 16 MB.  Sub-blocks are offsets within it. */
#define SGI_HUB_WINDOW_SIZE 0x1000000ULL

/* Sub-block bases (offsets within the widget-1 small window). */
#define SGI_HUB_PI_BASE 0x000000ULL
#define SGI_HUB_MD_BASE 0x200000ULL
#define SGI_HUB_II_BASE 0x400000ULL
#define SGI_HUB_NI_BASE 0x600000ULL

/* The remote alias of the same register file is +0x800000. */
#define SGI_HUB_REMOTE_ALIAS 0x800000ULL

#define SGI_HUB_MAX_CPUS 2

struct SGIHubState {
  SysBusDevice parent_obj;

  MemoryRegion iomem;
  QEMUBH *reset_bh;

  uint32_t nasid;
  uint32_t num_cpus;

  /* PI: CPU presence/enable and per-slice state. */
  uint64_t cpu_present[SGI_HUB_MAX_CPUS];
  uint64_t cpu_enable[SGI_HUB_MAX_CPUS];

  /* PI: interrupt pending sets (INT_PEND0 / INT_PEND1), 64 bits each. */
  uint64_t int_pend0;
  uint64_t int_pend1;

  /* PI: masks.  [0]=A mask0, [1]=A mask1, [2]=B mask0, [3]=B mask1 */
  uint64_t int_mask[4];
  uint64_t cc_mask;

  /* PI: real-time counter compare/enable, per slice. */
  uint64_t rt_compare[SGI_HUB_MAX_CPUS];
  uint64_t rt_enable[SGI_HUB_MAX_CPUS];

  /* PI: error-stack addresses (A=CPU0, B=CPU1).  The PROM's entry does a
   * deliberate one-time reset unless PI_ERR_STACK_ADDR_B holds "Rst0". */
  uint64_t pi_err_stack[2];

  /* MD: memory configuration (bank sizes) and refresh. */
  uint64_t mem_config;
  uint64_t refresh_ctl;
  uint64_t slotid_ustat;
  /* MD_LED0 (0x220050): front-panel LED, read/write. */
  uint64_t md_led0;
  /* MD performance-monitor event selector (0x210000). */
  uint64_t md_perf_sel;
  /*
   * MD_PERF_CNT0..5 (0x210010..0x210038).  With MD_PERF_SEL=0 these are not
   * counting, and the PROM deliberately clears them at startup "so they can
   * be used as scratch registers that are initialized to 0" (main.c).  It
   * uses CNT0 as HUB_LOCK_REG, CNT1 as the memory-test disable mask, and
   * CNT2..5 as a pass counter -- all read-modify-write, so they must read
   * back what was written.
   */
  uint64_t md_perf_cnt[6];

  /* NI: identity / scratch. */
  uint64_t ni_scratch[2];

  /* NI: vector/PIO engine (remote register reads/writes). */
  uint64_t ni_vector;
  uint64_t ni_vector_parms;
  uint64_t ni_vector_data;
  uint64_t ni_vector_status;
  uint64_t ni_vector_rd_data;
  /* NI: AGE registers (CPU0/1 memory, CPU0/1 PIO). */
  uint64_t ni_age[4];

  /* NI: hub NIC DS2502 1-wire EEPROM, bit-banged via MD_MLAN_CTL. */
  uint8_t ds_rom[8];
  uint8_t ds_mem[64];
  int ds_state;
  uint8_t ds_cmd;
  int ds_cmd_bits;
  uint8_t ds_in;
  int ds_in_bits;
  int ds_out_index;
  int ds_search_phase;
  int ds_addr;
  int ds_extra;
  int ds_data_bit;

  /* II: I/O interface (widget identity/link/access). */
  uint64_t ii_wcr;
  uint64_t ii_iowa;
  uint64_t ii_iiwa;
  uint64_t ii_ilcsr;
  uint64_t ii_scratch[2];

  /* Generic scratch for registers we only need to hold state for. */
  uint64_t calias_size;
  uint64_t region_present;

  /* IRQ lines to each CPU: [0]=INT_PEND0, [1]=INT_PEND1, [2]=RTC, [3]=PROF,
   * [4]=error -- mirroring the Linux IP27 hard-coded interrupt levels. */
  qemu_irq irq[SGI_HUB_MAX_CPUS][5];

  /* CPUs, for reset / timer handling. */
  CPUState *cpus[SGI_HUB_MAX_CPUS];
};

#endif /* HW_MISC_SGI_HUB_H */
