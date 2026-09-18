/*
 * SGI O2 MACE video block: VIN1 / VIN2 / VOUT + I2C master
 *
 * MACE's video section is four MMIO windows inside the MACE region:
 *
 *   0x1F100000  Video Input Channel 1   (VIN1)
 *   0x1F180000  Video Input Channel 2   (VIN2)
 *   0x1F200000  Video Output Channel    (VOUT)
 *   0x1F330000  I2C master              (AV1: SAA7111 + SAA7185)
 *
 * plus the per-channel UST/MSC pair in the MACE UST/MSC window
 * (VIN1 +0x38, VIN2 +0x40, VOUT +0x48).
 *
 * This device is modelled separately from sgi_mace.c and mapped as an
 * overlay of the MACE register region (the same technique the machine
 * uses for the 16550 serial port).  Register layout and I2C transaction
 * protocol come from the IP32 PROM firmware (src/fw/video.c,
 * mvpregs.h) and the MACE spec TABLE 5/6/8/10/24/76.
 *
 * The external analog video ports live on the AV1 module: the S-Video
 * and Composite jacks feed a Philips SAA7111 decoder (MACE port A), and
 * MACE output port E drives a SAA7185 encoder.  Both are I2C slaves of
 * the MACE I2C master modelled here.
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_MACE_VIDEO_H
#define HW_MISC_SGI_MACE_VIDEO_H

#include "chardev/char-fe.h"
#include "hw/i2c/i2c.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_SGI_MACE_VIDEO "sgi-mace-video"
OBJECT_DECLARE_SIMPLE_TYPE(SGIMACEVideoState, SGI_MACE_VIDEO)

/* Per-channel register offsets, relative to the channel window (mvpregs.h) */
#define MVP_REG_CONTROL       0x00   /* RW: DMA/int enables [9:0]           */
#define MVP_REG_STATUS        0x08   /* RO: DMA/int status                  */
#define MVP_REG_CONFIG        0x10   /* RW: channel configuration           */
#define MVP_REG_NEXT_DESC     0x18   /* RW: next descriptor table address   */
#define MVP_REG_FIELD_OFFSET  0x20   /* RW: field offset within the pages   */
#define MVP_REG_LINE_WIDTH    0x28   /* input:  bytes per line (11:3)       */
#define MVP_REG_FIELD_SIZE    0x28   /* output: lines (21:12) width (11:3)  */
#define MVP_REG_HCLIP_ODD     0x30   /* input H clip / output H pad, odd    */
#define MVP_REG_VCLIP_ODD     0x38   /* input V clip / output V pad, odd    */
#define MVP_REG_ALPHA_ODD     0x40   /* input alpha / output H pad even     */
#define MVP_REG_HCLIP_EVEN    0x48
#define MVP_REG_VCLIP_EVEN    0x50
#define MVP_REG_ALPHA_EVEN    0x58
#define MVP_REG_VHW_CFG       0x58   /* output only: port selects + rev     */
#define MVP_REG_DMA_DESC      0x80   /* 32 x 16-bit page pointers           */
#define MVP_MAX_PAGES         32

/* CONTROL register bits (mvpregs.h control_reg_s) */
#define MVP_CONTROL_ENABLE_DMA        0x001
#define MVP_CONTROL_ENABLE_VERTSYNC   0x002
#define MVP_CONTROL_ENABLE_DMACOMPLETE 0x004
#define MVP_CONTROL_ENABLE_ERRORS     0x1f8
#define MVP_CONTROL_ENABLE_GPI        0x200

/* STATUS register bits (mvpregs.h status_reg_s) */
#define MVP_STATUS_DMA_ACTIVE         (1u << 0)
#define MVP_STATUS_VERTICAL_SYNC      (1u << 1)
#define MVP_STATUS_DMA_COMPLETE       (1u << 2)
#define MVP_STATUS_LOST_SYNC          (1u << 3)
#define MVP_STATUS_BUFFER_OVERFLOW    (1u << 4)
#define MVP_STATUS_HORIZONTAL_OVERFLOW (1u << 5)
#define MVP_STATUS_VERTICAL_OVERFLOW  (1u << 6)
#define MVP_STATUS_FIFO_OVERFLOW      (1u << 7)
#define MVP_STATUS_CRIME_MEM_ERROR    (1u << 8)
#define MVP_STATUS_GPI                (1u << 9)
#define MVP_STATUS_SYNC_PRESENT       (1u << 10)
#define MVP_STATUS_INT_MASK           0x3fe   /* bits 1..9 clear on read */

/* Input CONFIG bits (mvpregs.h input_config_reg_s) */
#define MVP_ICONFIG_CHANNEL_RESET  (1u << 0)
#define MVP_ICONFIG_D1_RESET       (1u << 1)
#define MVP_ICONFIG_VIN_SOURCE_MASK (3u << 2)
#define MVP_ICONFIG_VIN_SOURCE_AB  0
#define MVP_ICONFIG_VIN_SOURCE_CD  1
#define MVP_ICONFIG_PRECISION      (1u << 4)
#define MVP_ICONFIG_ECC            (1u << 5)
#define MVP_ICONFIG_FORMAT_MASK    (7u << 10)
#define MVP_ICONFIG_FORMAT_SHIFT   10
#define MVP_ICONFIG_MEM_MODE_MASK  (3u << 14)
#define MVP_ICONFIG_INTERLEAVED    (1u << 16)

/* Pixel formats (input CONFIG [12:10]) */
#define MVP_FORMAT_RGBA32      0
#define MVP_FORMAT_RGBA16      1
#define MVP_FORMAT_YUV422      2
#define MVP_FORMAT_YUV422_10   3
#define MVP_FORMAT_ABGR32      4

/* NEXT_DESC register (mvpregs.h next_desc_reg_s) */
#define MVP_NDA_ADDRESS_MASK  0xffffffc0u
#define MVP_NDA_VALID         (1u << 2)
#define MVP_NDA_CAPTURE_MASK  0x3

/* VHW_CFG revision code (bits 35:32) reported to the driver */
#define MVP_VHW_CFG_REVISION  0x100000000ULL
#define MVP_VHW_CFG_AB_RESET  (1ULL << 0)
#define MVP_VHW_CFG_CD_RESET  (1ULL << 2)

/* I2C master registers, relative to the I2C window (spec TABLE 76) */
#define MACE_I2C_CONFIG       0x00
#define MACE_I2C_CONFIG_ALIAS 0x08
#define MACE_I2C_CONTROL      0x10
#define MACE_I2C_DATA         0x18

/* I2C control/status bits (mvpregs.h i2c_control_s) */
#define I2C_NOT_IDLE     (1 << 0)
#define I2C_READ         (1 << 1)
#define I2C_HOLD_BUS     (1 << 2)
#define I2C_XFER_BUSY    (1 << 4)
#define I2C_NACK         (1 << 5)
#define I2C_BUS_ERR      (1 << 7)

/* I2C config bits (mvpregs.h i2c_config_s) */
#define I2C_CFG_RESET      (1 << 0)
#define I2C_CFG_FAST_MODE  (1 << 1)
#define I2C_CFG_DATA_OVR   (1 << 2)
#define I2C_CFG_CLOCK_OVR  (1 << 3)

/* 7-bit slave addresses (8-bit table addresses >> 1) */
#define MACE_I2C_ADDR_SAA7111  0x24
#define MACE_I2C_ADDR_SAA7185  0x44

/* Video UST/MSC pairs in the MACE UST/MSC window (kernel sys/mace.h) */
#define MVP_UST_MSC_VIN1   0x38
#define MVP_UST_MSC_VIN2   0x40
#define MVP_UST_MSC_VOUT   0x48
#define MVP_UST_MSC_SIZE   0x18

/* Interrupt output lines to CRIME (kernel sys/mace.h MACE_VID_*) */
#define MVP_CRIME_IRQ_VIN1  0
#define MVP_CRIME_IRQ_VIN2  1
#define MVP_CRIME_IRQ_VOUT  2
#define MVP_NUM_IRQS        3

/* Capture field rate: NTSC 59.94 Hz fields. */
#define MVP_FIELD_PERIOD_NS 16683333

/* Host frame protocol fed over the optional "video-in" chardev. */
#define MVP_FRAME_MAGIC   0x4d565046u   /* "MVPF" */
#define MVP_FRAME_MAX     (4u * 1024 * 1024)

/* Fourcc values for the chardev frame payload */
#define MVP_FOURCC_RGBA  0x52474241u    /* "RGBA" */
#define MVP_FOURCC_UYVY  0x55595659u    /* "UYVY" */

typedef struct MVPChannelState {
    uint64_t control;      /* RW 0x00                              */
    uint64_t config;       /* RW 0x10                              */
    uint64_t next_desc;    /* RW 0x18                              */
    uint64_t field_offset; /* RW 0x20                              */
    uint64_t line_width;   /* RW 0x28 (input) / field_size (out)   */
    uint64_t hclip_odd;    /* RW 0x30                              */
    uint64_t vclip_odd;    /* RW 0x38                              */
    uint64_t alpha_odd;    /* RW 0x40                              */
    uint64_t hclip_even;   /* RW 0x48                              */
    uint64_t vclip_even;   /* RW 0x50                              */
    uint64_t alpha_even;   /* RW 0x58                              */
    uint64_t vhw_cfg;      /* RW 0x58 (output only)                */

    uint16_t dma_desc[MVP_MAX_PAGES];  /* RW 0x80..0xb8            */

    /* Runtime (not guest-visible except through STATUS / UST-MSC) */
    uint64_t status;       /* latched interrupt bits (clear on read) */
    bool dma_running;      /* CONTROL.ENABLE_DMA latched             */
    uint32_t msc;          /* field counter; LSB is the field id    */
    uint64_t ust;          /* MACE uptime at start of last field    */
} MVPChannelState;

struct SGIMACEVideoState {
    SysBusDevice parent_obj;

    MemoryRegion vin_mr;   /* 0x100000..0x17ffff: VIN1 + VIN2         */
    MemoryRegion vout_mr;  /* 0x200000 window                        */
    MemoryRegion i2c_mr;   /* 0x330000 window                        */
    MemoryRegion ustmsc_mr;/* UST/MSC video pairs (overlay only)     */

    qemu_irq crime_irq[MVP_NUM_IRQS];

    MVPChannelState vin[2];
    MVPChannelState vout;

    /* I2C master state machine */
    I2CBus *i2c_bus;
    uint8_t i2c_config;
    uint8_t i2c_ctrl;      /* last written control bits              */
    uint8_t i2c_data;
    bool xfer_open;        /* a bus transfer is held                 */
    uint8_t xfer_addr;     /* 7-bit address of the open transfer     */
    bool xfer_recv;        /* open transfer is a read                */
    bool recv_pending;     /* data register holds an unread RX byte  */
    bool nack;             /* last byte was not acknowledged         */
    bool bus_err;

    /* Host video source + field pacing */
    CharFrontend video_in;
    bool video_in_connected;
    QEMUTimer *field_timer;
    uint8_t *frame_buf;    /* last decoded host frame (host order)   */
    size_t frame_len;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_fourcc;
    uint8_t *rx_buf;       /* header assembly buffer                 */
    size_t rx_len;
    uint8_t *rx_payload;
    size_t rx_payload_len;

    /*
     * Host decoder helper driven by the GTK "Video" menu (the
     * sgi-video-source interface).  QEMU itself never decodes video: the
     * helper is an external process (ffmpeg) that connects to the
     * video-in chardev and streams MVPF frames.
     */
    char *video_helper;    /* property: helper executable path       */
    char *video_in_path;   /* property: unix socket the helper uses  */
    GPid helper_pid;       /* running helper, 0 when none            */
    guint helper_watch;    /* g_child_watch source id                */
    char *helper_source;   /* attached source, for describe()        */
    bool helper_is_url;
};

/**
 * sgi_mace_video_map_into: overlay the video MMIO windows onto the MACE
 * register region.  Called by the O2 machine after the MACE is realized
 * so video accesses take priority over the generic MACE handler.
 */
void sgi_mace_video_map_into(SGIMACEVideoState *s, MemoryRegion *mace_mr);

#endif /* HW_MISC_SGI_MACE_VIDEO_H */
