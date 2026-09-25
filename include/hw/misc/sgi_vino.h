/*
 * SGI VINO (Indy video input) — register shell.
 *
 * VINO is the Indy's (IP24) on-board video-input ASIC: the IndyCam digital
 * port plus composite/S-Video analog inputs.  Its register block lives in
 * EISA address space at physical 0x00080000 (IRIX sys/IP22.h
 * VINO_PHYS_BASE1; promoted by IP22.c simplevinoprobe()).
 *
 * Presence is a device property, default OFF: with present=off the region
 * reads all-ones exactly like an unmapped aperture, so a plain `-M indy`
 * invocation is byte-for-byte unchanged.  present=on makes the version
 * register answer so the PROM/kernel VINO probe recognises the part.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SGI_VINO_H
#define HW_MISC_SGI_VINO_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_SGI_VINO "sgi-vino"
OBJECT_DECLARE_SIMPLE_TYPE(SGIVinoState, SGI_VINO)

/* Largest MVPF frame we accept from the host video-source helper. */
#define SGI_VINO_FRAME_MAX (4u * 1024 * 1024)

/* VINO register aperture.  Register layout is the VINO Design Specification
 * 099-8937-001 (table 1), cross-checked against the Linux VINO driver
 * (vino.h): a block of 64-bit doubleword register slots at an 8-byte stride
 * with the 32-bit value at +4, so the top register (channel B FIFO write
 * pointer, 0x134) plus word is 0x138.  The IRIX 5.3 driver's master_reg_tbl
 * (39 direct-MMIO regs id 0..0x26) is consistent with this.  0x200 covers
 * every access seen. */
#define SGI_VINO_REG_SIZE 0x200

/* Version register (physical base + 0x04). */
#define SGI_VINO_REV_OFFSET   0x04
#define SGI_VINO_CHIP_ID      0x0B
#define SGI_VINO_REVISION     0x01

/* Full register map, from the VINO Design Specification 099-8937-001
 * (table 1) and cross-checked against the Linux VINO driver (vino.h).
 * Every register is a 64-bit doubleword slot; the 32-bit value lives at the
 * +4 word (except the Rev/ID at +4 too), so these are the byte offsets the
 * driver and PROM actually touch. */
#define SGI_VINO_REV_ID_OFFSET    0x04
#define SGI_VINO_CONTROL_OFFSET   0x0c
#define SGI_VINO_INTSTAT_OFFSET   0x14
#define SGI_VINO_CH_A_BASE        0x28
#define SGI_VINO_CH_B_BASE        0xb0

/* Per-channel register offsets, relative to the channel base. */
#define SGI_VINO_CH_ALPHA         0x04
#define SGI_VINO_CH_CLIP_START    0x0c
#define SGI_VINO_CH_CLIP_END      0x14
#define SGI_VINO_CH_FRAME_RATE    0x1c
#define SGI_VINO_CH_FIELD_COUNT   0x24   /* read-only */
#define SGI_VINO_CH_LINE_SIZE     0x2c
#define SGI_VINO_CH_LINE_COUNT    0x34
#define SGI_VINO_CH_PAGE_INDEX    0x3c
#define SGI_VINO_CH_NEXT4DESC     0x44   /* writing this starts a DMA fetch */
#define SGI_VINO_CH_START_DESC    0x4c
#define SGI_VINO_CH_DESC0         0x54
#define SGI_VINO_CH_DESC1         0x5c
#define SGI_VINO_CH_DESC2         0x64
#define SGI_VINO_CH_DESC3         0x6c
#define SGI_VINO_CH_FIFO_THRES    0x74
#define SGI_VINO_CH_FIFO_READ     0x7c   /* read-only */
#define SGI_VINO_CH_FIFO_WRITE    0x84   /* read-only */

/* Control register (0x0c).  Two identical channels A (low half) and B. */
#define SGI_VINO_CTRL_LITTLE_ENDIAN   (1u << 0)
#define SGI_VINO_CTRL_A_EOF_INT       (1u << 1)
#define SGI_VINO_CTRL_A_FIFO_INT      (1u << 2)
#define SGI_VINO_CTRL_A_EOD_INT       (1u << 3)
#define SGI_VINO_CTRL_B_EOF_INT       (1u << 4)
#define SGI_VINO_CTRL_B_FIFO_INT      (1u << 5)
#define SGI_VINO_CTRL_B_EOD_INT       (1u << 6)
#define SGI_VINO_CTRL_A_DMA_ENBL      (1u << 7)
#define SGI_VINO_CTRL_A_INTERLEAVE    (1u << 8)
#define SGI_VINO_CTRL_A_SYNC_ENBL     (1u << 9)
#define SGI_VINO_CTRL_A_SELECT        (1u << 10)
#define SGI_VINO_CTRL_A_RGB           (1u << 11)
#define SGI_VINO_CTRL_A_LUMA_ONLY     (1u << 12)
#define SGI_VINO_CTRL_A_DEC_ENBL      (1u << 13)
#define SGI_VINO_CTRL_A_DITHER        (1u << 18)
#define SGI_VINO_CTRL_B_DMA_ENBL      (1u << 19)
#define SGI_VINO_CTRL_B_INTERLEAVE    (1u << 20)
#define SGI_VINO_CTRL_B_SYNC_ENBL     (1u << 21)
#define SGI_VINO_CTRL_B_SELECT        (1u << 22)
#define SGI_VINO_CTRL_B_RGB           (1u << 23)
#define SGI_VINO_CTRL_B_LUMA_ONLY     (1u << 24)
#define SGI_VINO_CTRL_B_DEC_ENBL      (1u << 25)
#define SGI_VINO_CTRL_B_DITHER        (1u << 30)

/* Interrupt status (0x14). */
#define SGI_VINO_INT_A_EOF    (1u << 0)
#define SGI_VINO_INT_A_FIFO   (1u << 1)
#define SGI_VINO_INT_A_EOD    (1u << 2)
#define SGI_VINO_INT_B_EOF    (1u << 3)
#define SGI_VINO_INT_B_FIFO   (1u << 4)
#define SGI_VINO_INT_B_EOD    (1u << 5)

/* DMA descriptor word (32 bits in memory, fetched four at a time from
 * next_4_desc).  Bits 29:0 are the 4K page address, bit 30 jump, bit 31
 * stop; the valid bit is bit 32 and is maintained by the hardware cache,
 * not stored in the memory word. */
#define SGI_VINO_DESC_JUMP    (1u << 30)
#define SGI_VINO_DESC_STOP    (1u << 31)
#define SGI_VINO_DESC_PER_FETCH  4

/* VINO I2C master: control/status at id 3 (offset 0x1c), data at id 4
 * (offset 0x24).  Recovered from the IRIX 5.3 driver (vino_i2c.o): the
 * control byte 0x5 issues a byte transfer, 0x0 forces the bus idle; status
 * bits read back from 0x1c are 0x01 bus-busy, 0x10 transfer-in-progress,
 * 0x20 ack, 0x80 error.  The analog decoder sits at 7-bit address 0x45
 * (write 0x8a / read 0x8b); a second device answers at 0x2b. */
#define SGI_VINO_I2C_CTRL_OFFSET  0x1c
#define SGI_VINO_I2C_DATA_OFFSET  0x24
#define SGI_VINO_I2C_ADDR_WRITE   0x8a   /* SAA7191 */
#define SGI_VINO_I2C_ADDR_ALT     0x56   /* second I2C device */

struct SGIVinoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bool present;
    uint8_t regs[SGI_VINO_REG_SIZE];

    /* Modelled SAA7191 / second-device I2C register files and the master
     * transfer state (write byte stream: address, sub-address, then data). */
    uint8_t i2c_dec[256];
    uint8_t i2c_alt[256];
    uint8_t i2c_ptr;
    uint8_t i2c_first;   /* next data byte is the device address */
    uint8_t i2c_expect_sub; /* next data byte is the sub-address */
    uint8_t i2c_alt_dev; /* current transaction targets the second device */

    /* DMA: last descriptor group fetched per channel, plus the base pointer
     * the driver programmed (channel A next_4_desc / B mirror). */
    uint32_t dma_base[2];
    uint32_t desc[2][SGI_VINO_DESC_PER_FETCH];

    /* Capture DMA engine state, one per channel (A = 0, B = 1).  The
     * hardware keeps a four-entry descriptor cache per channel and fetches
     * the next group of four from next_4_desc (auto-advancing by 16 bytes);
     * each descriptor names a 4K page that capture DMA fills.  A field ends
     * with a field-count increment and an EOF interrupt, a stop descriptor
     * with an EOD interrupt. */
    QEMUTimer *field_timer;
    qemu_irq irq;                       /* INT3 local1 bit 6 via HPC3 */
    bool dma_en[2];
    bool cache_valid[2];
    int cache_pos[2];
    uint32_t next_desc[2];              /* auto-advancing fetch pointer */
    uint32_t field_count[2];

    /* Host video source (sgi-video-source interface).  An external helper
     * (the shared o2helpers decoder wrapper, reused unchanged) connects to
     * the video-in chardev and streams MVPF frames; a received frame replaces
     * the internal luma ramp on the next field.  With nothing attached the
     * device falls back to the ramp, so existing capture tests are unchanged. */
    CharFrontend video_in;
    char *video_in_path;
    char *video_helper;
    GPid helper_pid;
    guint helper_watch;
    char *helper_source;
    bool helper_is_url;
    uint8_t *rx_buf;
    size_t rx_len;
    size_t rx_payload_len;
    uint8_t *frame_buf;
    size_t frame_len;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_fourcc;
};


#endif /* HW_MISC_SGI_VINO_H */
