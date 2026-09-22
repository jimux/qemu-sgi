/*
 * SCN2681 dual asynchronous receiver/transmitter (DUART).
 *
 * Used as the console/serial and SGI serial keyboard/mouse controller on
 * the SGI Personal IRIS (IP6).  Register-compatible with the Signetics
 * SCN2681 / MC68681 family.
 *
 * The SGI board wires the register file as a 32-bit-strided window with the
 * chip select in address bit 0, so the machine decodes the address and calls
 * scn2681_read()/scn2681_write() with the 4-bit register index (the same
 * convention as MAME's mc68681).  Channel A/B transmit and receive are
 * backed by host chardevs; the mode/baud timing is accepted but not used to
 * gate transfer (transmission is immediate, polling-friendly).
 *
 * References:
 *   - MAME src/devices/machine/mc68681.cpp
 *   - Philips SCN2681 dual UART data sheet
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_SGI_SCN2681_H
#define HW_CHAR_SGI_SCN2681_H

#include "chardev/char-fe.h"
#include "hw/core/qdev.h"
#include "hw/core/irq.h"
#include "qom/object.h"

#define TYPE_SGI_SCN2681 "sgi-scn2681"
OBJECT_DECLARE_SIMPLE_TYPE(SCN2681State, SGI_SCN2681)

/* Status register bits */
#define SCN2681_SR_RXRDY  0x01
#define SCN2681_SR_FFULL  0x02
#define SCN2681_SR_TXRDY  0x04
#define SCN2681_SR_TXEMT  0x08
#define SCN2681_SR_OVRUN  0x10
#define SCN2681_SR_PERR   0x20
#define SCN2681_SR_FERR   0x40
#define SCN2681_SR_BREAK  0x80

/* Interrupt status/mask bits */
#define SCN2681_ISR_INPUT 0x80
#define SCN2681_ISR_DBRKB 0x40
#define SCN2681_ISR_RXRDYB 0x20
#define SCN2681_ISR_TXRDYB 0x10
#define SCN2681_ISR_CNTR  0x08
#define SCN2681_ISR_DBRKA 0x04
#define SCN2681_ISR_RXRDYA 0x02
#define SCN2681_ISR_TXRDYA 0x01

typedef struct SCN2681Channel {
    uint8_t mr1;
    uint8_t mr2;
    uint8_t csr;
    uint8_t sr;
    bool mr_ptr;      /* next mode-register read returns MR2 */
    uint8_t rx_byte;
} SCN2681Channel;

struct SCN2681State {
    DeviceState parent_obj;

    CharFrontend chr_a;
    CharFrontend chr_b;

    SCN2681Channel ch[2];

    uint8_t acr;
    uint8_t imr;
    uint8_t isr;
    uint8_t ctur;
    uint8_t ctlr;
    uint8_t ivr;
    uint8_t opcr;
    uint8_t opr;

    qemu_irq irq;
};

uint8_t scn2681_read(SCN2681State *s, int reg);
void scn2681_write(SCN2681State *s, int reg, uint8_t val);

#endif /* HW_CHAR_SGI_SCN2681_H */
