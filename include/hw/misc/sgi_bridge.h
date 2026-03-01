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

    /* Register storage */
    uint32_t regs[BRIDGE_NUM_REGS];

    /*
     * IOC3 SuperIO UART A (serial console).
     *
     * IOC3 registers are byte-accessible at 1-byte stride, but byte-reversed
     * within 32-bit words (big-endian MIPS):
     *   IOC3 physical offset 0 (0x20178) → 16550 reg 3 (LCR)
     *   IOC3 physical offset 3 (0x2017B) → 16550 reg 0 (RBR/THR/DLL)
     *   IOC3 physical offset 6 (0x2017E) → 16550 reg 5 (LSR)
     * Formula: std_reg = offset ^ 3
     *
     * Mapped at BRIDGE offset 0x620178 (= IOC3_BASE 0x1F600000 + SIO_UA 0x20178).
     */
    SerialState ioc3_uart;
    MemoryRegion ioc3_uart_mr;
};

#endif /* HW_MISC_SGI_BRIDGE_H */