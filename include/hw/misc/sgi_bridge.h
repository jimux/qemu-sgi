/*
 * SGI BRIDGE (PCI/XTalk Bridge) emulation
 *
 * BRIDGE is the peripheral controller in the SGI Octane (IP30). It handles:
 * - PCI configuration space access
 * - XTLink (XTalk) interface for graphics
 * - BaseIO slot control
 * - Power button and ACFail detection
 *
 * Physical base: 0x1F400000
 * Reference: IRIX sys/bridge.h, MAME src/mame/sgi/mace.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_BRIDGE_H
#define HW_MISC_SGI_BRIDGE_H

#include "hw/char/serial.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_BRIDGE "sgi-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(SGIBRIDGEState, SGI_BRIDGE)

#define BRIDGE_NUM_REGS 0x1000

/* DS2502-family 1-wire device state (bridge board EEPROM, IOC3 MAC EEPROM). */
typedef struct SGIDS {
    uint8_t rom[8];
    uint8_t mem[128];
    uint8_t state;
    uint8_t cmd;
    int cmd_bits;
    uint8_t in;
    int in_bits;
    int out_index;
    int search_phase;
    int addr;
    int extra;
    int extra_bits; /* status/extra bits before read-memory data */
    uint8_t data_bit;
} SGIDS;

/*
 * BRIDGE register offsets (from 0x1F400000)
 *
 * Minimal implementation - these are placeholders for the actual register map.
 * Full BRIDGE register set is complex and includes:
 * - PCI configuration space (0x0000-0x0FFF)
 * - XTLink registers (0x1000-0x1FFF)
 * - BaseIO slot registers (0x2000-0x2FFF)
 * - Power/ACFail registers (0x3000-0x3FFF)
 */

struct SGIBRIDGEState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* CPU interrupt output (BRIDGE -> CPU IP3) */
    qemu_irq cpu_irq;

    /* DS2502 board-config EEPROM on the bridge MicroLAN line (+0xb4). */
    SGIDS bridge_ds;
    /* DS2502 MAC-address EEPROM on the IOC3 MicroLAN line (+0x600030). */
    SGIDS ioc3_ds;

    /*
     * IOC3 PCI configuration (BRIDGE type-0 config, device 2 at +0x22000):
     * vendor/device 0x10A9/0x0003 and the writable command/BAR registers.
     */
    uint32_t pci_cmd;
    uint32_t pci_bar[2];

    /* Register storage */
    uint32_t regs[BRIDGE_NUM_REGS];

    /*
     * IOC3 devio register block (BRIDGE+0x600000-0x61FFFF). The PROM's
     * pon_ioc3 POST write/read tests land across the whole block, so back it
     * with storage; specific registers (SIO_CR, MCR) have real semantics on
     * read. 0x20000 bytes = 0x8000 words.
     */
    uint32_t ioc3_regs[0x8000];

    /*
     * IOC3 SuperIO index/data register pair (BRIDGE+0x6A0000 index,
     * +0x6C0000 data), a PC-style bank of 8-bit SuperIO registers.
     */
    uint8_t sio_index;
    uint8_t sio_regs[256];

    /*
     * IOC3 SuperIO UART A (serial console).
     *
     * In 16550-compatibility mode the IOC3 UART is byte-spaced: the register
     * index equals the byte offset (0=RBR/THR, 1=IER, 2=IIR/FCR, 3=LCR,
     * 4=MCR, 5=LSR, 6=MSR, 7=SCR). Matches the verified IP27 BaseIO model.
     *
     * Mapped at BRIDGE offset 0x620178 (= IOC3 devio 0x600000 + 0x20178).
     */
    SerialState ioc3_uart;
    MemoryRegion ioc3_uart_mr;
    MemoryRegion ioc3_uart_mr2;
};

#endif /* HW_MISC_SGI_BRIDGE_H */