/*
 * SGI HPC1.5 (High Performance Peripheral Controller) for IP20 (Indigo)
 *
 * The HPC1.5 is the peripheral controller for the R4000 Indigo (IP20).
 * It is the predecessor of the HPC3 found in Indy/Indigo2 and has a
 * different register layout.  The sub-devices hanging off it are the
 * same family: WD33C93 SCSI, SEEQ 8003 Ethernet, Z85C30 DUARTs, a
 * 93C56 serial EEPROM / NVRAM, and (separately addressed) the INT2
 * local I/O interrupt multiplexor with an 8254 PIT.
 *
 * Register map (from MAME ip20.cpp/hpc1.cpp and the IP20 PROM source):
 *   0x1fb80000 + 0x000  HPC1 core DMA engine registers
 *   0x1fb80000 + 0x100  SEEQ 8003 ethernet
 *   0x1fb80000 + 0x120  WD33C93 SCSI indirect interface
 *   0x1fb80000 + 0x1b0  HPC1 misc status
 *   0x1fb80000 + 0x1bc  aux / EEPROM bit-bang
 *   0x1fb80000 + 0x1c0  INT2 + 8254 PIT
 *   0x1fb80000 + 0xd00  DUART0 (0xd10 DUART1, 0xd20 DUART2)
 *   0x1fb80000 + 0xe00  DP8572 RTC
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_HPC1_H
#define HW_MISC_SGI_HPC1_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "hw/scsi/wd33c93.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_SGI_HPC1 "sgi-hpc1"
OBJECT_DECLARE_SIMPLE_TYPE(SGIHPC1State, SGI_HPC1)

/* HPC1 core register offsets within the 0x1fb80000 window */
#define HPC1_SCSI_CTRL   0x0094
#define HPC1_ENET_BASE   0x0100
#define HPC1_SCSI_BASE   0x0120
#define HPC1_MISCSR      0x01b0
#define HPC1_AUX         0x01bc
#define HPC1_INT2_BASE   0x01c0
#define HPC1_DUART_BASE  0x0d00
#define HPC1_RTC_BASE    0x0e00

/*
 * HPC1 ethernet (SEEQ 8003) DMA / status registers.
 *
 * Layout is `struct EHIO` from SGI's own driver header
 * (references/stand/arcs/include/net/seeq.h, the `#if IP20` branch), which
 * documents both the HPC1 and HPC3 variants.  Note the HPC1 quirk: the
 * receive status sits in the HIGH byte (RCVSTAT_SHIFT 8) and the transmit
 * status / "dma started" bits sit in the high half -- unlike HPC3, which
 * uses the low bits and adds a piocfg/dmacfg indirection HPC1 does not have.
 */
#define HPC1_ENET_XCOUNT   0x0008  /* hpc debug */
#define HPC1_ENET_CXBP     0x000c  /* current xmit buffer pointer */
#define HPC1_ENET_NXBDP    0x0010  /* next xmit buffer descriptor pointer */
#define HPC1_ENET_XBC      0x0014  /* xmit byte count */
#define HPC1_ENET_CXBDP    0x0020  /* current xmit buffer descriptor ptr */
#define HPC1_ENET_CPFXBDP  0x0024  /* current packet 1st xmit desc ptr */
#define HPC1_ENET_PPFXBDP  0x0028  /* previous packet 1st xmit desc ptr */
#define HPC1_ENET_INTDELAY 0x002c  /* interrupt delay count */
#define HPC1_ENET_TRSTAT   0x0034  /* xmit status */
#define HPC1_ENET_RCVSTAT  0x0038  /* receive status */
#define HPC1_ENET_CTL      0x003c  /* interrupt / channel reset / buf oflow */
#define HPC1_ENET_RBC      0x0048  /* receive byte count */
#define HPC1_ENET_CRBP     0x004c  /* current receive buffer pointer */
#define HPC1_ENET_NRBDP    0x0050  /* next receive buffer descriptor pointer */
#define HPC1_ENET_CRBDP    0x0054  /* current receive buffer descriptor ptr */

/* HPC1 enet control register bits (seeq.h, `#else` of IP22/26/28) */
#define HPC1_ENET_CTL_RBO      0x08  /* receive buffer overflow */
#define HPC1_ENET_CTL_MODNORM  0x04  /* mode: 0=loopback, 1=normal */
#define HPC1_ENET_CTL_INTPEND  0x02  /* interrupt pending (write 1 clears) */
#define HPC1_ENET_CTL_ERST     0x01  /* ethernet channel reset */

/* HPC1 status placement (high half/byte, unlike HPC3) */
#define HPC1_ENET_RCVSTAT_SHIFT   8
#define HPC1_ENET_STRCVDMA        0x00004000  /* receive dma started */
#define HPC1_ENET_STTRDMA         0x00400000  /* xmit dma started (0x40<<16) */
#define HPC1_ENET_SEQ_XS_OLD      0x00800000  /* xmit old status (0x80<<16) */
#define HPC1_ENET_SEQ_XS_SUCCESS  0x00080000  /* xmit success (0x08<<16) */

#define HPC1_NUM_DUARTS  3
#define HPC1_DUART_CH   2
#define HPC1_RX_FIFO_SIZE 16

/* One Z85C30 channel (two per DUART) */
typedef struct SGIHPC1Uart {
    uint8_t reg_ptr;            /* selected WR/RR register (0-15) */
    uint8_t wr[16];             /* write registers */
    uint8_t rr3;                /* interrupt-pending bits (channel A) */
    uint8_t rx_fifo[HPC1_RX_FIFO_SIZE];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;
} SGIHPC1Uart;

struct SGIHPC1State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend serial;
    char *nvram_filename;

    /* HPC1 core */
    uint32_t miscsr;
    uint32_t scsi_ctrl;
    uint32_t scsi_bc;
    uint32_t scsi_cbp;
    uint32_t scsi_nbdp;
    uint32_t scsi_dma_count;
    bool scsi_dma_active;
    bool scsi_dma_to_device;
    bool scsi_drq;
    uint32_t dsp_bc;

    /* Scratch for the parallel / DSP-interface register RAM areas */
    uint8_t core_scratch[0x200];

    /* DP8572 RTC register/RAM file (32-bit spaced: reg = offset >> 2) */
    uint8_t rtc[0x80];

    /* DP8572 time base: guest wall-clock = guest_base + (host - host_base) */
    int64_t rtc_host_base_ms;
    int64_t rtc_guest_base_ms;
    QEMUTimer *rtc_timer;

    /* SEEQ 8003 registers (read side only, TX/RX DMA is a later stage) */
    uint8_t seeq_station_addr[6];
    /* Banked Seeq union (seeq.h EHIO seq_reg.sr): bank1 = mcast_lsb[6],
     * bank2 = seq_write { mcast_msb[2], pktgap, ctl }. */
    uint8_t seeq_mcast_lsb[6];
    uint8_t seeq_mcast_msb[2];
    uint8_t seeq_pktgap;
    uint8_t seeq_seeqctl;
    uint8_t seeq_rx_cmd;
    uint8_t seeq_tx_cmd;
    uint8_t seeq_rx_status;
    uint8_t seeq_tx_status;

    /*
     * HPC1 ethernet DMA / status registers (struct EHIO, seeq.h IP20 branch).
     * Register file only so far; the descriptor DMA engine, the NICState and
     * the INT2 LIO0 Ethernet interrupt are the next increment (LEG44).
     */
    uint32_t enet_xcount;
    uint32_t enet_cxbp;
    uint32_t enet_nxbdp;
    uint32_t enet_xbc;
    uint32_t enet_cxbdp;
    uint32_t enet_cpfxbdp;
    uint32_t enet_ppfxbdp;
    uint32_t enet_intdelay;
    uint32_t enet_trstat;
    uint32_t enet_rcvstat;
    uint32_t enet_ctl;
    uint32_t enet_rbc;
    uint32_t enet_crbp;
    uint32_t enet_nrbdp;
    uint32_t enet_crbdp;

    /* QEMU NIC (SEEQ 8003 EDLC, DMA through HPC1) */
    NICState *nic;
    NICConf enet_conf;

    /* WD33C93 SCSI controller */
    WD33C93State *scsi;

    /*
     * Aux / 93C56 serial EEPROM bit-bang.
     *
     * QEMU's generic eeprom93xx model uses 8 address bits for 128 words,
     * but the 93C56 has 7; that one-bit mismatch makes the PROM's EWEN
     * command decode as a WRITE-ALL. Model the 93C56 here instead, with
     * the exact 7-bit protocol the PROM drives.
     */
    uint8_t aux;
    uint16_t nvram[128];
    uint8_t nv_cs;
    uint8_t nv_clk;
    uint8_t nv_di;
    uint8_t nv_do;
    uint8_t nv_tick;
    uint8_t nv_opcode;
    uint8_t nv_addr;
    uint8_t nv_writable;
    uint16_t nv_data;

    /* INT2 interrupt multiplexor */
    uint8_t lio_status[2];
    uint8_t lio_mask[2];
    uint8_t vme_status;
    uint8_t vme_mask[2];
    uint8_t int2_config;

    /* 8254 PIT (inside INT2) */
    uint8_t pit_control[3];
    uint16_t pit_count[3];     /* reload value */
    uint16_t pit_low[3];       /* partial (LSB) write */
    uint8_t pit_rw_state[3];   /* write/read byte FSM */
    uint8_t pit_read_state[3];
    int64_t pit_load_ns[3];
    bool pit_programmed[3];
    QEMUTimer *pit_timer[2];
    bool timer_pending[2];

    /* Z85C30 DUARTs [duart][channel] */
    SGIHPC1Uart uart[HPC1_NUM_DUARTS][HPC1_DUART_CH];

    /* Named IRQ outputs (wired by the machine) */
    qemu_irq cpu_irq[2];    /* INT2 LIO0 -> IP2, LIO1 -> IP3 */
    qemu_irq timer_irq[2];  /* PIT timer0 -> IP4, timer1 -> IP5 */
};

#endif /* HW_MISC_SGI_HPC1_H */
