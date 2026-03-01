/*
 * SGI HPC3 (High Performance Peripheral Controller) emulation
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_HPC3_H
#define HW_MISC_SGI_HPC3_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/scsi/wd33c93.h"
#include "hw/nvram/eeprom93xx.h"
#include "hw/input/ps2.h"
#include "net/net.h"
#include "qom/object.h"

/*
 * HPC3 DMA uses physical addresses, but IRIX passes KSEG0/KSEG1
 * virtual addresses to DMA registers. Strip the top bits to get
 * physical addresses (MAME uses the CPU address space instead).
 */
#define HPC3_DMA_ADDR(a)  ((uint32_t)(a) & 0x1fffffffU)

/*
 * SGI-specific PS/2 keyboard subtype with real-time typematic repeat.
 *
 * Under -icount shift=0,sleep=off guest virtual time races far ahead of real
 * time, so the X11 server's autorepeat timer fires many times per physical
 * key press.  This subtype drives typematic from QEMU_CLOCK_REALTIME so
 * repeat rates are correct regardless of icount speed.
 */
#define TYPE_SGI_PS2_KBD "sgi-ps2-kbd"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPs2KbdState, SGI_PS2_KBD)

struct SGIPs2KbdState {
    PS2KbdState parent_obj;         /* must be first — QOM/C inheritance */
    QEMUTimer  *typematic_timer;    /* drives repeat on QEMU_CLOCK_REALTIME */
    int         typematic_qcode;    /* held key qcode, or -1 if none held */
    uint32_t    typematic_delay_ms; /* initial delay before first repeat */
    uint32_t    typematic_period_ms;/* interval between repeats */
};

#define TYPE_SGI_HPC3 "sgi-hpc3"
OBJECT_DECLARE_SIMPLE_TYPE(SGIHPC3State, SGI_HPC3)

/*
 * HPC3 Register offsets (from 0x1fb80000)
 * Based on IRIX kernel sys/hpc3.h
 */

/* PBUS DMA channels base (channels 0-7) */
#define HPC3_PBUS_BP_BASE      0x00000  /* Buffer pointer */
#define HPC3_PBUS_DP_BASE      0x00004  /* Descriptor pointer */
#define HPC3_PBUS_CTRL_BASE    0x01000  /* Control */
#define HPC3_PBUS_STRIDE       0x02000  /* Stride per channel */

/* SCSI channels */
#define HPC3_SCSI0_CBP         0x10000  /* Channel 0 buffer pointer */
#define HPC3_SCSI0_NBDP        0x10004  /* Next buffer desc pointer */
#define HPC3_SCSI0_BC          0x11000  /* Byte count */
#define HPC3_SCSI0_CTRL        0x11004  /* Control */
#define HPC3_SCSI0_GIO_FIFO    0x11008  /* GIO FIFO pointer */
#define HPC3_SCSI0_DEV_FIFO    0x1100c  /* Device FIFO pointer */
#define HPC3_SCSI0_DMACFG      0x11010  /* DMA config */
#define HPC3_SCSI0_PIOCFG      0x11014  /* PIO config */

#define HPC3_SCSI1_CBP         0x12000
#define HPC3_SCSI1_NBDP        0x12004
#define HPC3_SCSI1_BC          0x13000
#define HPC3_SCSI1_CTRL        0x13004
#define HPC3_SCSI1_GIO_FIFO   0x13008
#define HPC3_SCSI1_DEV_FIFO   0x1300c
#define HPC3_SCSI1_DMACFG     0x13010
#define HPC3_SCSI1_PIOCFG     0x13014

/* Ethernet */
#define HPC3_ENET_RX_CBP       0x14000
#define HPC3_ENET_RX_NBDP      0x14004
#define HPC3_ENET_RX_BC        0x15000
#define HPC3_ENET_RX_CTRL      0x15004
#define HPC3_ENET_MISC         0x15014
#define HPC3_ENET_DMACFG       0x15018
#define HPC3_ENET_PIOCFG       0x1501c

#define HPC3_ENET_RX_GIO       0x15008
#define HPC3_ENET_RX_DEV       0x1500c

#define HPC3_ENET_TX_CBP       0x16000
#define HPC3_ENET_TX_NBDP      0x16004
#define HPC3_ENET_TX_BC        0x17000
#define HPC3_ENET_TX_CTRL      0x17004
#define HPC3_ENET_TX_GIO       0x17008
#define HPC3_ENET_TX_DEV       0x1700c

#define HPC3_ENET_RX_CBDP      0x18000
#define HPC3_ENET_TX_CPFBDP    0x1a000
#define HPC3_ENET_TX_PPFBDP    0x1a004

/*
 * HPC3 Ethernet DMA control register bits (from MAME hpc3.h / IRIX hpc3.h)
 */

/* RX control register bits */
#define HPC3_ENET_RXC_ST      0x00bf  /* Seeq status mask (bits 0-5, 7) */
#define HPC3_ENET_RXC_LC      0x0040  /* Late collision */
#define HPC3_ENET_RXC_LE      0x0100  /* Little endian */
#define HPC3_ENET_RXC_CA      0x0200  /* Channel active */
#define HPC3_ENET_RXC_CAM     0x0400  /* Channel active mask */
#define HPC3_ENET_RXC_RBO     0x0800  /* RX buffer overflow */

/* TX control register bits */
#define HPC3_ENET_TXC_ST      0x00ef  /* Seeq status mask */
#define HPC3_ENET_TXC_LC      0x0010  /* Late collision */
#define HPC3_ENET_TXC_LE      0x0100  /* Little endian */
#define HPC3_ENET_TXC_CA      0x0200  /* Channel active */
#define HPC3_ENET_TXC_CAM     0x0400  /* Channel active mask */

/* DMA descriptor byte count bits */
#define HPC3_ENET_BC_COUNT    0x3fff  /* Byte count (14 bits) */
#define HPC3_ENET_BC_ROWN     0x4000  /* RX: driver owns descriptor */
#define HPC3_ENET_BC_TXD      0x8000  /* TX done */
#define HPC3_ENET_BC_IPG      0x00ff0000  /* Inter-packet gap */
#define HPC3_ENET_BC_XIE      0x20000000  /* Interrupt enable */
#define HPC3_ENET_BC_EOXP     0x40000000  /* End of packet */
#define HPC3_ENET_BC_EOX      0x80000000  /* End of chain */

/* ENET_MISC register bits */
#define HPC3_ENET_MISC_RESET     0x01  /* Channel reset (1=assert reset) */
#define HPC3_ENET_MISC_INT       0x02  /* Interrupt pending (W1C) */
#define HPC3_ENET_MISC_LOOPBACK  0x04  /* Loopback mode */

/* Seeq RX command bits */
#define SEEQ_RXC_MODE_MASK   0xc0
#define SEEQ_RXC_MODE_OFF    0x00  /* Receiver disabled */
#define SEEQ_RXC_MODE_PROMISC 0x40 /* Promiscuous */
#define SEEQ_RXC_MODE_NORMAL  0x80 /* Station + broadcast */
#define SEEQ_RXC_MODE_MULTI   0xc0 /* Station + broadcast + multicast */

/* Seeq RX status bits */
#define SEEQ_RXS_OLD    0x80  /* Old status (already consumed) */
#define SEEQ_RXS_GOOD   0x20  /* Good frame */
#define SEEQ_RXS_END    0x10  /* End of frame */

/* Seeq TX status bits */
#define SEEQ_TXS_OLD    0x80  /* Old status (already consumed) */
#define SEEQ_TXS_SUCCESS 0x08 /* TX success */

/* Seeq 80C03 TX command bank select (bits [6:5]) */
#define SEEQ_TXC_BANK_MASK    0x60
#define SEEQ_TXC_BANK_STATION 0x00  /* Bank 0: station address */
#define SEEQ_TXC_BANK_MCAST_L 0x20  /* Bank 1: multicast filter low */
#define SEEQ_TXC_BANK_MCAST_H 0x40  /* Bank 2: multicast filter high + control */

/* Max ethernet packet size */
#define ENET_MAX_PACKET  1536

/* GIO Bus Interface Control (at 0x1fbb0000) */
#define HPC3_INTSTAT           0x30000
#define HPC3_MISC              0x30004
#define HPC3_EEPROM            0x30008
#define HPC3_BUSERR_STAT       0x30010

/* SCSI controller registers (WD33C93) */
#define HPC3_SCSI0_REG         0x40000  /* 0x1fbc0000 */
#define HPC3_SCSI1_REG         0x48000  /* 0x1fbc8000 */

/* Ethernet controller (SEEQ 8003) */
#define HPC3_ENET_REG          0x54000  /* 0x1fbd4000 */

/* PBUS PIO config registers */
#define HPC3_PBUS_CFGDMA_BASE  0x5c000  /* 0x1fbdc000 */
#define HPC3_PBUS_CFGPIO_BASE  0x5d000  /* 0x1fbdd000 */
#define HPC3_PBUS_CFG_STRIDE   0x00200

/*
 * PBUS Channel 6 - IOC registers (at 0x1fbd9800)
 * Offset from HPC3 base = 0x59800
 */
#define HPC3_IOC_BASE          0x59800
#define HPC3_PAR_DATA          0x59800  /* 6,0x00 - Parallel data */
#define HPC3_PAR_CONTROL       0x59804  /* 6,0x01 - Parallel control */
#define HPC3_PAR_STAT          0x59808  /* 6,0x02 - Parallel status */
#define HPC3_PAR_DMA_CONTROL   0x5980c  /* 6,0x03 */
#define HPC3_PAR_INT_STAT      0x59810  /* 6,0x04 */
#define HPC3_PAR_INT_MASK      0x59814  /* 6,0x05 */
#define HPC3_PAR_TIMER1        0x59818  /* 6,0x06 */
#define HPC3_PAR_TIMER2        0x5981c  /* 6,0x07 */
#define HPC3_PAR_TIMER3        0x59820  /* 6,0x08 */
#define HPC3_PAR_TIMER4        0x59824  /* 6,0x09 */
#define HPC3_SERIAL1_CMD       0x59830  /* 6,0x0c - Serial port 1 command */
#define HPC3_SERIAL1_DATA      0x59834  /* 6,0x0d - Serial port 1 data */
#define HPC3_SERIAL0_CMD       0x59838  /* 6,0x0e - Serial port 0 command */
#define HPC3_SERIAL0_DATA      0x5983c  /* 6,0x0f - Serial port 0 data */
#define HPC3_KBD_MOUSE0        0x59840  /* 6,0x10 */
#define HPC3_KBD_MOUSE1        0x59844  /* 6,0x11 */
#define HPC3_GC_SELECT         0x59848  /* 6,0x12 */
#define HPC3_GEN_CONTROL       0x5984c  /* 6,0x13 */
#define HPC3_PANEL             0x59850  /* 6,0x14 - Front panel */
#define HPC3_SYS_ID            0x59858  /* 6,0x16 - System ID */
#define HPC3_READ              0x59860  /* 6,0x18 */
#define HPC3_DMA_SELECT        0x59868  /* 6,0x1a */
#define HPC3_WRITE1            0x59870  /* 6,0x1c - Reset (IP24) */
#define HPC3_WRITE2            0x59878  /* 6,0x1e - Write (IP24) */

/* INT3 registers (at 0x1fbd9880) */
#define HPC3_INT3_BASE         0x59880
#define HPC3_INT3_LOCAL0_STAT  0x59880
#define HPC3_INT3_LOCAL0_MASK  0x59884
#define HPC3_INT3_LOCAL1_STAT  0x59888
#define HPC3_INT3_LOCAL1_MASK  0x5988c
#define HPC3_INT3_MAP_STATUS   0x59890
#define HPC3_INT3_MAP_MASK0    0x59894
#define HPC3_INT3_MAP_MASK1    0x59898
#define HPC3_INT3_MAP_POLARITY 0x5989c
#define HPC3_INT3_TIMER_CLEAR  0x598a0
#define HPC3_INT3_ERROR_STATUS 0x598a4

/* 8254 PIT timer registers (at 0x1fbd98b0) */
#define HPC3_INT3_PIT_COUNTER0 0x598b0
#define HPC3_INT3_PIT_COUNTER1 0x598b4
#define HPC3_INT3_PIT_COUNTER2 0x598b8
#define HPC3_INT3_PIT_CONTROL  0x598bc

/*
 * Full House (IP22/IP26/IP28) INT3 registers at PIO4 base (0x59000)
 * On Guinness (IP24/Indy), INT3 is at PIO6+0x80 (0x59880).
 * On Full House, INT3 is at PIO4 (0x59000).
 * PIT timer and other IOC2 registers stay at PIO6 (0x59800) for both.
 */
#define HPC3_FH_INT3_LOCAL0_STAT  0x59000
#define HPC3_FH_INT3_LOCAL0_MASK  0x59004
#define HPC3_FH_INT3_LOCAL1_STAT  0x59008
#define HPC3_FH_INT3_LOCAL1_MASK  0x5900c
#define HPC3_FH_INT3_MAP_STATUS   0x59010
#define HPC3_FH_INT3_MAP_MASK0    0x59014
#define HPC3_FH_INT3_MAP_MASK1    0x59018
#define HPC3_FH_INT3_TIMER_CLEAR  0x59020
#define HPC3_FH_INT3_PIT_COUNTER0 0x59030
#define HPC3_FH_INT3_PIT_COUNTER1 0x59034
#define HPC3_FH_INT3_PIT_COUNTER2 0x59038
#define HPC3_FH_INT3_PIT_CONTROL  0x5903c

/* HAL2 Audio Controller registers (PBUS PIO chip select 0) */
#define HAL2_PIO_BASE       0x58000   /* Offset from HPC3 base */
#define HAL2_REG_ISR         0x10     /* Interrupt/Status Register */
#define HAL2_REG_REV         0x20     /* Revision Register */
#define HAL2_REG_IAR         0x30     /* Indirect Address Register */
#define HAL2_REG_IDR0        0x40     /* Indirect Data Register 0 */
#define HAL2_REG_IDR1        0x50     /* Indirect Data Register 1 */
#define HAL2_REG_IDR2        0x60     /* Indirect Data Register 2 */
#define HAL2_REG_IDR3        0x70     /* Indirect Data Register 3 */

/* HAL2 ISR bits */
#define HAL2_ISR_TSTATUS        0x01  /* Transaction status (busy) */
#define HAL2_ISR_USTATUS        0x02  /* Utime status */
#define HAL2_ISR_CODEC_MODE     0x04  /* Codec mode (quad/indigo) */
#define HAL2_ISR_GLOBAL_RESET_N 0x08  /* Global reset (active low) */
#define HAL2_ISR_CODEC_RESET_N  0x10  /* Codec reset (active low) */

/* HAL2 revision: 0x4010 = HAL2 rev A */
#define HAL2_REV_VALUE       0x4010

/* Volume DAC (PBUS PIO chip select 2) */
#define HAL2_VOLUME_RIGHT    0x800    /* Offset from HAL2_PIO_BASE */
#define HAL2_VOLUME_LEFT     0x804    /* Offset from HAL2_PIO_BASE */

/* Total HPC3 region size */
#define HPC3_REG_SIZE          0x80000

/*
 * INT3 Local0 interrupt bits (from MAME ioc2.h)
 * These are the interrupt sources multiplexed onto IP0
 */
#define INT3_LOCAL0_FIFO      0x01  /* PBUS FIFO */
#define INT3_LOCAL0_SCSI0     0x02  /* SCSI channel 0 (WD33C93) */
#define INT3_LOCAL0_SCSI1     0x04  /* SCSI channel 1 (WD33C93) */
#define INT3_LOCAL0_ETHERNET  0x08  /* Ethernet (SEEQ 8003) */
#define INT3_LOCAL0_MC_DMA    0x10  /* MC DMA complete */
#define INT3_LOCAL0_PARALLEL  0x20  /* Parallel port */
#define INT3_LOCAL0_GRAPHICS  0x40  /* Graphics (GIO) */
#define INT3_LOCAL0_MAPPABLE0 0x80  /* Mappable interrupt 0 */

/*
 * INT3 Local1 interrupt bits
 * These are the interrupt sources multiplexed onto IP1
 */
#define INT3_LOCAL1_ISDN_A    0x01  /* ISDN channel A / GP0 */
#define INT3_LOCAL1_POWER     0x02  /* Power button / Panel */
#define INT3_LOCAL1_ISDN_B    0x04  /* ISDN channel B / GP2 */
#define INT3_LOCAL1_LCL0      0x08  /* Local0 cascade / Mappable1 */
#define INT3_LOCAL1_HPC_DMA   0x10  /* HPC DMA complete (MAME: INT3_LOCAL1_HPC_DMA) */
#define INT3_LOCAL1_GIO0      0x20  /* GIO slot 0 / AC Fail */
#define INT3_LOCAL1_EISA      0x40  /* EISA bus / Video */
#define INT3_LOCAL1_GIO2      0x80  /* GIO slot 2 / Retrace */

/* System ID bits */
#define SYSID_CHIP_REV_MASK    0xe0
#define SYSID_BOARD_REV_MASK   0x1e
#define SYSID_BOARD_ID_MASK    0x01
#define BOARD_IP24             0       /* Indy/Guinness */
#define BOARD_IP22             1       /* Indigo2/Fullhouse */

struct SGIHPC3State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend serial;

    /* GIO Bus Interface registers */
    uint32_t intstat;
    uint32_t misc;
    uint32_t eeprom;
    uint32_t buserr_stat;

    /* SCSI state */
    uint32_t scsi_cbp[2];
    uint32_t scsi_nbdp[2];
    uint32_t scsi_bc[2];
    uint32_t scsi_ctrl[2];
    uint32_t scsi_dmacfg[2];
    uint32_t scsi_piocfg[2];

    /* Ethernet state */
    uint32_t enet_rx_cbp;
    uint32_t enet_rx_nbdp;
    uint32_t enet_rx_bc;
    uint32_t enet_rx_ctrl;
    uint32_t enet_tx_cbp;
    uint32_t enet_tx_nbdp;
    uint32_t enet_tx_bc;
    uint32_t enet_tx_ctrl;
    uint32_t enet_misc;
    uint32_t enet_dmacfg;
    uint32_t enet_piocfg;

    /* SCSI DMA runtime state */
    uint16_t scsi_dma_count[2];      /* Remaining bytes in current descriptor */
    bool scsi_dma_active[2];         /* DMA engine running */
    bool scsi_dma_to_device[2];      /* Transfer direction */
    bool scsi_dma_big_endian[2];     /* Endian mode */
    bool scsi_drq[2];                /* Current DRQ state from WD33C93 */

    /* WD33C93 SCSI controllers */
    WD33C93State *scsi[2];

    /* CPU interrupt outputs (Local0 → IP2, Local1 → IP3) */
    qemu_irq cpu_irq[2];

    /* Direct timer IRQ outputs to CPU (Timer0 → IP4, Timer1 → IP5) */
    qemu_irq timer_irq[2];
    bool timer_pending[2];

    /* PBUS config */
    uint32_t pbus_dmacfg[8];
    uint32_t pbus_piocfg[10];

    /* PBUS DMA channel registers (8 channels) */
    uint32_t pbus_bp[8];    /* Buffer pointer */
    uint32_t pbus_dp[8];    /* Descriptor pointer */
    uint32_t pbus_ctrl[8];  /* Control */

    /* IOC/INT3 registers */
    uint32_t par_data;
    uint32_t par_control;
    uint32_t gen_control;
    uint32_t panel;
    uint32_t sysid;
    uint8_t board_type;   /* BOARD_IP24 (Guinness) or BOARD_IP22 (Full House) */
    uint8_t nvram_rev;    /* NVRAM revision: 8 for IP22/IP24, 9 for IP26/IP28 */
    uint32_t read_reg;
    uint32_t dma_select;
    uint32_t write1;
    uint32_t write2;

    /* INT3 interrupt controller */
    uint32_t int3_local0_stat;
    uint32_t int3_local0_mask;
    uint32_t int3_local1_stat;
    uint32_t int3_local1_mask;
    uint32_t int3_map_status;
    uint32_t int3_map_mask0;
    uint32_t int3_map_mask1;
    uint32_t int3_map_polarity;
    uint32_t int3_error_status;

    /* Serial port state (Z85C30 compatible) */
    uint8_t serial_cmd[2];
    uint8_t serial_data[2];
    uint8_t serial_reg_ptr[2];  /* Which register is selected */

    /* Serial RX FIFO (for chardev input → guest reads) */
#define SCC_RX_FIFO_SIZE 16
    uint8_t serial_rx_fifo[2][SCC_RX_FIFO_SIZE];
    uint8_t serial_rx_fifo_head[2];
    uint8_t serial_rx_fifo_tail[2];
    uint8_t serial_rx_fifo_count[2];

    /*
     * Z85C30 SCC interrupt state
     *
     * Channel mapping: port 0 = Channel A (HPC3_SERIAL0)
     *                  port 1 = Channel B (HPC3_SERIAL1, console)
     *
     * RR3 bits (channel A only):
     *   Bit 0: Ch B External/Status IP
     *   Bit 1: Ch B TX Buffer Empty IP
     *   Bit 2: Ch B RX Char Available IP
     *   Bit 3: Ch A External/Status IP
     *   Bit 4: Ch A TX Buffer Empty IP
     *   Bit 5: Ch A RX Char Available IP
     */
    uint8_t scc_wr1[2];    /* WR1 per channel: interrupt enables */
    uint8_t scc_wr5[2];    /* WR5 per channel: TX controls */
    uint8_t scc_wr9;       /* WR9: master interrupt control (shared) */
    uint8_t scc_rr3;       /* RR3: interrupt pending (channel A only) */

    /* SCC TX interrupt timer: fires after TX completes to signal TX empty */
    QEMUTimer *scc_tx_timer[2];   /* One per channel (0=A, 1=B) */

    bool autoload;  /* NVRAM AutoLoad: true='Y', false='N' */

    /* Seeq 80C03 EDLC state */
    uint8_t seeq_station_addr[6];  /* MAC address (regs 0-5) */
    uint8_t seeq_rx_cmd;           /* RX command register (mode + IE bits) */
    uint8_t seeq_tx_cmd;           /* TX command register (bank + IE bits) */
    uint8_t seeq_rx_status;        /* RX status (last received) */
    uint8_t seeq_tx_status;        /* TX status (last transmitted) */

    /* Ethernet DMA runtime */
    uint32_t enet_rx_cbdp;          /* Current RX buffer descriptor pointer */
    uint32_t enet_tx_cpfbdp;        /* Current/previous first TX buffer desc ptr */
    uint32_t enet_tx_ppfbdp;        /* Previous first TX buffer desc ptr */

    /* NIC */
    NICState *nic;
    NICConf enet_conf;

    /* 93CS56 serial EEPROM (Full House NVRAM, 128 × 16-bit words) */
    eeprom_t *serial_eeprom;

    /* BBRAM (battery-backed RAM) - 8KB, includes DS1386 RTC at offset 0 */
    uint8_t bbram[8192];

    /* NVRAM persistence */
    char *nvram_filename;       /* Backing file path (NULL = no persistence) */
    bool nvram_loaded;          /* True if loaded from existing file */

    /* RTC state */
    uint8_t rtc_command;
    int64_t rtc_time_offset;  /* Offset from host clock for guest-set time */

    /*
     * 8254 PIT timer state
     * Channel 2: 1 MHz master clock
     * Channel 0/1: Clocked by Channel 2 output, generate CPU interrupts
     */
    uint16_t pit_count[3];        /* Counter reload values */
    uint16_t pit_latch[3];        /* Latched counter values */
    uint8_t pit_control[3];       /* Control word for each channel */
    uint8_t pit_rw_state[3];      /* Read/write FSM state (for 16-bit access) */
    bool pit_latched[3];          /* Counter value is latched */
    int64_t pit_load_time[3];     /* Time when counter was loaded (ns) */
    QEMUTimer *pit_timer[2];      /* Timers for interrupt generation (ch 0,1) */

    /*
     * 8042 PS/2 Keyboard Controller (embedded in IOC2)
     * The controller is accessed via IOC registers at 0x59840/0x59844
     * and also via PBUS PIO data area at 0x58484/0x58488
     *
     * Embeds QEMU PS/2 keyboard and mouse devices which register
     * input handlers automatically, so host events flow through.
     */
    SGIPs2KbdState ps2kbd;        /* PS/2 keyboard device (SGI typematic subtype) */
    PS2MouseState ps2mouse;       /* PS/2 mouse device */
    uint8_t kbd_cmd;              /* Last 8042 controller command written */
    uint8_t kbd_cmd_byte;         /* 8042 controller command byte (via 0x60) */
    bool kbd_write_to_mouse;      /* Next data write goes to mouse (0xD4) */
    bool kbd_irq_level;           /* Current keyboard IRQ state */
    bool mouse_irq_level;         /* Current mouse IRQ state */
    uint8_t kbd_pending_source;   /* 0=keyboard, 1=mouse data pending */

    /* HAL2 audio stub state */
    uint32_t hal2_isr;          /* ISR register */
    uint32_t hal2_iar;          /* Indirect address register */
    uint32_t hal2_idr[4];       /* Indirect data registers */
    uint8_t  hal2_volume_left;  /* Volume DAC left */
    uint8_t  hal2_volume_right; /* Volume DAC right */
};

#endif /* HW_MISC_SGI_HPC3_H */
