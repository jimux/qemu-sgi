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

/*
 * Dallas DS2502 1-wire slave (family 0x09): a minimal add-only memory part.
 * The master bit-bangs an MCR_PACK(pulse, sample) command onto the line; the
 * part answers reset/presence, READ ROM (0x33), MATCH ROM (0x55), SKIP ROM
 * (0xcc) and READ MEMORY (0xf0, 16-bit address).  Two lines carry one in this
 * machine: the hub's MD_MLAN_CTL and the SN0 router's RR_NIC_ULAN.  Both share
 * this state machine; the record contents (rom serial, memory pages) are
 * owner-specific, built with sgi_ds2502_build_board().
 *
 * Protocol/CRC from libsk/ml/nic.c (and the matching octane DS2502 model);
 * MCR_PACK/MCR_DONE/MCR_DATA from sys/nic.h.
 */
typedef struct SGIDS2502 {
  uint8_t rom[8];      /* family + 48-bit serial + Dallas CRC-8 */
  uint8_t mem[64];     /* two 32-byte pages, each with a trailing CRC-16 */
  int state;
  uint8_t cmd;
  int cmd_bits;
  uint8_t in;
  int in_bits;
  int out_index;
  int search_phase;
  int addr;
  int extra;
  int data_bit;        /* latched line state (MCR_DATA) */
} SGIDS2502;

/* Fill rom[]/mem[] with a board record: a 10-char serial string, part and
 * name go in the two memory pages, and rom_serial[6] becomes the 48-bit ROM
 * serial.  All CRCs are computed so the PROM's nic_next()/page check passes. */
void sgi_ds2502_build_board(SGIDS2502 *ds, const char *serial,
                            const char *part, const char *name,
                            const uint8_t rom_serial[6]);
/* Reset the part to the command state (a 1-wire reset pulse). */
void sgi_ds2502_reset(SGIDS2502 *ds);
/* Clock one master bit into the part. */
void sgi_ds2502_write_bit(SGIDS2502 *ds, int bit);
/* Clock one bit out of the part. */
int sgi_ds2502_read_bit(SGIDS2502 *ds);
/*
 * Decode one MCR_PACK(pulse, sample) access and return the latched line data
 * bit.  pulse >= 480 is reset/presence; sample 30/110 is a write 0/1; sample
 * 13 is a read (nic.c nic_presence 520/65, nic_write 80/30 or 6/110,
 * nic_read 6/13).  The caller supplies MCR_DONE and the MCR_DATA bit.
 */
int sgi_ds2502_mcr(SGIDS2502 *ds, uint64_t val);

struct SGIHubState {
  SysBusDevice parent_obj;

  MemoryRegion iomem;
  QEMUBH *reset_bh;

  uint32_t nasid;
  uint32_t num_cpus;
  /*
   * Node-board slot id reported in MD_SLOTID_USTAT[2:0] (indexes
   * nodeslot_table[]): 7 = n1, 6 = n2.  Both node boards of a two-node module
   * must differ, otherwise their hwgraph paths collide and the kernel's
   * klhwg_connect_hubs() gets GRAPH_DUP adding the hub->router edge.
   */
  uint32_t slot_id;
  /*
   * Whether an IO board (BaseIO bridge) is attached to this hub's xtalk port.
   * The hub's IIO_LLP_CSR reports the link up only then.  On a two-node
   * Origin the second node here is a CPU-dead memory brick with no IO board,
   * and the kernel's io_graph_init() -> early_probe_for_widget() must observe
   * the link DOWN so it sets XWIDGET_PART_NUM_NONE and skips the widget probe
   * instead of reading an unbacked node-1 widget and taking a PIO error
   * (ml/SN/iograph.c:395-417).  Default true keeps node 0 / single node
   * unchanged.
   */
  bool io_attached;

  /* PI: CPU presence/enable and per-slice state. */
  uint64_t cpu_present[SGI_HUB_MAX_CPUS];
  uint64_t cpu_enable[SGI_HUB_MAX_CPUS];

  /* PI: interrupt pending sets (INT_PEND0 / INT_PEND1), 64 bits each. */
  uint64_t int_pend0;
  uint64_t int_pend1;

  /* PI: masks.  [0]=A mask0, [1]=A mask1, [2]=B mask0, [3]=B mask1 */
  uint64_t int_mask[4];
  uint64_t cc_mask;

  /* PI: real-time counter compare/enable/pending, per slice.
   * PI_RT_PEND_x latches when the free-running PI_RT_COUNT reaches a freshly
   * armed PI_RT_COMPARE_x; the OS raises its L4 (RTC) interrupt while the
   * pend bit is set and acks by writing PI_RT_PEND_x=0.  Once acked the pend
   * must NOT re-assert until PI_RT_COMPARE_x is written again -- rt_armed
   * tracks that one-shot latch (klclock.c hub_rtc_init/acktmoclock/COMPARE). */
  uint64_t rt_compare[SGI_HUB_MAX_CPUS];
  uint64_t rt_enable[SGI_HUB_MAX_CPUS];
  uint64_t rt_pend[SGI_HUB_MAX_CPUS];
  uint64_t rt_armed[SGI_HUB_MAX_CPUS];

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
  SGIDS2502 ds;

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

  /* Polls the free-running RTC against each slice's armed COMPARE. */
  QEMUTimer *rt_timer;

  /*
   * SN0 network fabric: this hub's NI link is the router it is plugged into
   * (NULL on a single-node machine).  When a router is attached a vector PIO
   * with path 0 addresses the router itself, and a nonzero path names the
   * router exit ports to traverse (see sgi_hub_ni_vector_go).  The router's
   * register file and port graph live in the machine, not in the hub.
   */
  struct SGIRouterState *router;

  /* NI routing tables written by distribute_tables(): 32 meta + 16 local. */
  uint64_t ni_meta_table[32];
  uint64_t ni_local_table[16];

  /*
   * PCF8584 I2C bus controller at MD_UREG0_0 (A0, data) and MD_UREG0_0+8
   * (A1, control/status).  The hub drives the module's I2C bus, on which the
   * ELSC (entry-level system controller) sits; the PROM reads its module
   * number and partition out of the ELSC's NVRAM through this chip.  `elsc`
   * points at the module-owned NVRAM (NULL before the machine wires it).
   */
  struct SGIElscState *elsc;
  uint8_t i2c_ctl;       /* last control byte written to A1 */
  uint8_t i2c_own;       /* own-address register (S0') */
  uint8_t i2c_clock;     /* clock-control register (S2) */
  uint8_t i2c_data;      /* last byte written to the data register (S0) */
  int i2c_have_addr;     /* a pre-start data write is waiting to be the address */
  int i2c_need_addr;     /* a repeated start was issued: next A0 write is addr */
  int i2c_active;        /* a start condition has begun a transaction */
  int i2c_rw;            /* 0 = master transmit, 1 = master receive */
  int i2c_slave;         /* latched 7-bit slave address */
  int i2c_phase;         /* transmit: 0 = memory offset, 1 = data stream */
  uint16_t i2c_memaddr;  /* 11-bit ELSC NVRAM address */
  int i2c_rx_count;      /* bytes returned since the read address phase */
};

/*
 * ELSC NVRAM (the module's system controller).  The PROM reads it over I2C
 * (libkl/io/elsc.c): a 2 KB serial EEPROM addressed in pages of 256 bytes,
 * where page 7 (NVRAM address 0x700..0x70f) holds the magic, module number,
 * partition and DIP-switch bytes.  One ELSC serves a whole module (its two
 * node boards share it), so the machine owns this and both hubs point at it.
 */
#define SGI_ELSC_NVRAM_SIZE 2048
#define SGI_ELSC_MAGIC_AD 0x700
#define SGI_ELSC_MAGIC_NO 0x37
/* The RUNNER ip27prom.img reads the module number at NVRAM 0x70a (observed:
 * the SR1 ADVERT_MODULE field tracks the byte at 0x70a, not 0x708).  The
 * partition sits at 0x708 in that build. */
#define SGI_ELSC_MODULE_AD 0x70a
#define SGI_ELSC_PARTITION_AD 0x708

typedef struct SGIElscState {
  uint8_t nvram[SGI_ELSC_NVRAM_SIZE];
} SGIElscState;

/* Seed the ELSC NVRAM with a magic number, module number and partition. */
void sgi_elsc_init(SGIElscState *e, uint8_t module, uint8_t partition);
/* Attach the module's ELSC to a hub's I2C controller. */
void sgi_hub_set_elsc(SGIHubState *s, SGIElscState *e);

/*
 * SN0 router (the R-brick crossbar linking hubs).  Only the register file and
 * the port graph are modelled; there is no latency or coherence.
 *
 * Router registers are accessed exclusively through the hubs' NI vector PIO
 * engine, not by memory mapping, so this is a plain structure the machine
 * owns and both hubs reference.  `regs` is indexed by register offset / 8.
 */
#define SGI_ROUTER_PORTS 6
#define SGI_ROUTER_REG_WORDS (0x80000 / 8)

typedef struct SGIRouterState {
  uint64_t *regs;
  uint32_t nic;
  uint32_t chipin;
  uint32_t revision;
  struct SGIHubState *port[SGI_ROUTER_PORTS + 1]; /* ports 1..6 */
  /* Router NIC DS2502 1-wire EEPROM, bit-banged via RR_NIC_ULAN. */
  SGIDS2502 ds;
} SGIRouterState;

void sgi_router_init(SGIRouterState *r, uint32_t nic, uint32_t chipin,
                     uint32_t revision);
void sgi_router_connect(SGIRouterState *r, int port, SGIHubState *hub);

/*
 * Re-initialise a router's volatile state on a system reset: clear the
 * register file (RR_SCRATCH_REG1 in particular, which discovery's fence uses
 * as a lock and can leave held at 0xffff after a panic) and re-seed
 * RR_SCRATCH_REG0 with the cached NIC, then reset the 1-wire slave.  The port
 * graph is topology, not volatile state, and is left intact.  Called from the
 * hub's device reset (the path the kernel's reset_system() reaches) and from
 * sgi_ip27_local_reset(), so a recovery boot re-acquires the router lock
 * instead of degrading to single-node.
 */
void sgi_router_reset(SGIRouterState *r);

/* Attach a router as this hub's NI link (NULL = single-node, no peer). */
void sgi_hub_set_router(SGIHubState *s, SGIRouterState *r);

/* Register-file access used by the router when it targets a hub. */
uint64_t sgi_hub_reg_read(SGIHubState *s, uint64_t off);
void sgi_hub_reg_write(SGIHubState *s, uint64_t off, uint64_t val);

/*
 * Raise or clear one interrupt vector in the hub's INT_PEND0/1 sets.
 *
 * A device on an XIO widget (e.g. the BaseIO bridge) sends an interrupt to the
 * hub carrying the vector the kernel programmed into that device's bridge
 * b_int_addr register.  The hub latches it here; the matching INT_MASK bit
 * then asserts the CPU's IP2 (INT_PEND0) or IP3 (INT_PEND1) line, and the
 * kernel's intpend0()/intpend1() dispatch reads the bit and clears it via
 * PI_INT_PEND_MOD.  `vec` is the raw field (0..127): 0..63 index INT_PEND0,
 * 64..127 index INT_PEND1.
 */
void sgi_hub_raise_vector(SGIHubState *s, unsigned vec, int level);

#endif /* HW_MISC_SGI_HUB_H */
