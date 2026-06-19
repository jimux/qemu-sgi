/*
 * SGI Paravirtual UART
 *
 * Minimal polled UART for the IP54 paravirtual machine.
 * Mapped at 0x1F620178; the PROM accesses two byte registers:
 *   +3 (0x17B): THR on write (send char), RBR on read (receive char)
 *   +6 (0x17E): LSR on read — bit 0 (DR) = data ready, bit 5 (THRE) = TX empty
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_SGI_PVUART_H
#define HW_CHAR_SGI_PVUART_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_PVUART "sgi-pvuart"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPVUARTState, SGI_PVUART)

#define SGI_PVUART_MMIO_SIZE  0x8

/* Byte register offsets from MMIO base (0x1F620178) */
#define SGI_PVUART_THR_RBR   0x3   /* TX Holding / RX Buffer */
#define SGI_PVUART_RXINJECT  0x4   /* Host RX-inject: write a byte to push it
                                    * into the RX FIFO as if received. Lets the
                                    * host drive console input directly via a
                                    * gdbstub MMIO write — no serial backend,
                                    * no TFTP. (host_channel.py / file xfer) */
#define SGI_PVUART_LSR       0x6   /* Line Status Register */

/* LSR bits */
#define SGI_PVUART_LSR_DR    (1 << 0)  /* Data Ready */
#define SGI_PVUART_LSR_THRE  (1 << 5)  /* TX Holding Register Empty */

#define SGI_PVUART_RXFIFO_SIZE 8192    /* host->guest RX ring */

struct SGIPVUARTState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    CharFrontend chr;

    /* RX ring: fed by the chardev backend AND by host RX-inject writes. */
    uint8_t  rx_fifo[SGI_PVUART_RXFIFO_SIZE];
    uint32_t rx_head;   /* next write index */
    uint32_t rx_tail;   /* next read index  */
};

#endif /* HW_CHAR_SGI_PVUART_H */
