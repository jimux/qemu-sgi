/*
 * SGI MACE (Multimedia, Audio, and Communications Engine) emulation
 *
 * MACE is the peripheral controller in the SGI O2 (IP32). It contains:
 * - PCI bridge (for SCSI, additional cards)
 * - Video input/output interfaces
 * - Ethernet MAC (MAC110)
 * - Audio controller
 * - ISA peripherals (flash, LED, NIC, interrupts)
 * - Keyboard/mouse interface
 * - I2C bus
 * - UST/MSC timers
 * - Serial ports (16550-compatible, 256-byte register spacing)
 * - DS17287 RTC/NVRAM
 *
 * Physical base: 0x1F000000
 * Reference: IRIX sys/mace.h, MAME src/mame/sgi/mace.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_MACE_H
#define HW_MISC_SGI_MACE_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_SGI_MACE "sgi-mace"
OBJECT_DECLARE_SIMPLE_TYPE(SGIMACEState, SGI_MACE)

/*
 * MACE sub-block offsets from MACE_BASE (0x1F000000)
 */
#define MACE_PCI_OFFSET         0x080000
#define MACE_VIN1_OFFSET        0x100000
#define MACE_VIN2_OFFSET        0x180000
#define MACE_VOUT_OFFSET        0x200000
#define MACE_ENET_OFFSET        0x280000
#define MACE_PERIF_OFFSET       0x300000
#define MACE_ISA_EXT_OFFSET     0x380000

/* Peripheral sub-blocks (relative to MACE_BASE) */
#define MACE_AUDIO_OFFSET       (MACE_PERIF_OFFSET + 0x00000)
#define MACE_ISA_OFFSET         (MACE_PERIF_OFFSET + 0x10000)
#define MACE_KBDMS_OFFSET       (MACE_PERIF_OFFSET + 0x20000)
#define MACE_I2C_OFFSET         (MACE_PERIF_OFFSET + 0x30000)
#define MACE_UST_MSC_OFFSET     (MACE_PERIF_OFFSET + 0x40000)

/* ISA extended sub-blocks (relative to MACE_BASE) */
#define MACE_EPP_OFFSET         (MACE_ISA_EXT_OFFSET + 0x00000)
#define MACE_ECP_OFFSET         (MACE_ISA_EXT_OFFSET + 0x08000)
#define MACE_SER1_OFFSET        (MACE_ISA_EXT_OFFSET + 0x10000)
#define MACE_SER2_OFFSET        (MACE_ISA_EXT_OFFSET + 0x18000)
#define MACE_RTC_OFFSET         (MACE_ISA_EXT_OFFSET + 0x20000)
#define MACE_GAME_OFFSET        (MACE_ISA_EXT_OFFSET + 0x30000)

/* ISA registers (relative to MACE_ISA_OFFSET) */
#define ISA_RINGBASE_REG        0x0000
#define ISA_FLASH_NIC_REG       0x0008
#define ISA_INT_STS_REG         0x0010
#define ISA_INT_MSK_REG         0x0018

/* ISA interrupt bits */
#define ISA_INT_RTC_IRQ         0x00000100

/* Serial port register spacing */
#define MACE_SERIAL_REG_SHIFT   8  /* 256-byte spacing between 16550 regs */
#define MACE_SERIAL_SIZE        0x2000  /* 8 registers * 256 bytes each */

/*
 * 16550 register indices (multiplied by 256 for actual offset)
 */
#define UART_REG_THR    0   /* TX Hold / RX Buffer (data) */
#define UART_REG_IER    1   /* Interrupt Enable */
#define UART_REG_IIR    2   /* Interrupt Ident (read) / FIFO Ctrl (write) */
#define UART_REG_LCR    3   /* Line Control */
#define UART_REG_MCR    4   /* Modem Control */
#define UART_REG_LSR    5   /* Line Status */
#define UART_REG_MSR    6   /* Modem Status */
#define UART_REG_SCR    7   /* Scratch */

/* LSR bits */
#define LSR_DR          0x01    /* Data Ready */
#define LSR_THRE        0x20    /* TX Holding Register Empty */
#define LSR_TEMT        0x40    /* Transmitter Empty */

/* LCR bits */
#define LCR_DLAB        0x80    /* Divisor Latch Access Bit */

/* IIR bits */
#define IIR_NO_INT      0x01    /* No interrupt pending */
#define IIR_FIFO_ENA    0xC0    /* FIFOs enabled */

/* MCR bits */
#define MCR_DTR         0x01
#define MCR_RTS         0x02

/* UST/MSC timer registers (relative to MACE_UST_MSC_OFFSET) */
#define UST_MSC_REG     0x00
#define UST_COMPARE1    0x08
#define UST_COMPARE2    0x10
#define UST_COMPARE3    0x18

/* UST period in nanoseconds */
#define MACE_UST_PERIOD_NS  960

/* Total MACE region size */
#define MACE_REG_SIZE   0x400000

/* Number of serial ports */
#define MACE_NUM_SERIAL 2

/* Serial RX FIFO */
#define MACE_SERIAL_FIFO_SIZE 16

struct SGIMACEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend serial;  /* Console serial port chardev */

    /* MACE interrupt output to CRIME */
    qemu_irq crime_irq[16];

    /* ISA registers */
    uint64_t isa_ringbase;
    uint64_t isa_flash_nic;
    uint64_t isa_int_status;
    uint64_t isa_int_mask;

    /* UST/MSC timer */
    uint64_t ust_compare[3];

    /* Serial ports */
    struct {
        uint8_t thr;        /* TX holding register */
        uint8_t ier;        /* Interrupt enable */
        uint8_t iir;        /* Interrupt identification */
        uint8_t fcr;        /* FIFO control (write-only) */
        uint8_t lcr;        /* Line control */
        uint8_t mcr;        /* Modem control */
        uint8_t lsr;        /* Line status */
        uint8_t msr;        /* Modem status */
        uint8_t scr;        /* Scratch */
        uint8_t dll;        /* Divisor latch low */
        uint8_t dlh;        /* Divisor latch high */

        /* RX FIFO */
        uint8_t rx_fifo[MACE_SERIAL_FIFO_SIZE];
        int rx_fifo_head;
        int rx_fifo_tail;
        int rx_fifo_count;
    } serial_port[MACE_NUM_SERIAL];

    /* DS17287 RTC */
    uint8_t rtc_index;          /* Address register */
    uint8_t rtc_ram[128];       /* RTC RAM (includes time registers + NVRAM) */

    /* PCI stub registers */
    uint32_t pci_error_addr;
    uint32_t pci_error_flags;
    uint32_t pci_control;
    uint32_t pci_rev_info;
    uint32_t pci_config_addr;
};

#endif /* HW_MISC_SGI_MACE_H */
