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
    uint8_t seeq_rx_cmd;
    uint8_t seeq_tx_cmd;
    uint8_t seeq_rx_status;
    uint8_t seeq_tx_status;

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
