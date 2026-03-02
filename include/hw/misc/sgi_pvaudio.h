/*
 * SGI Paravirtual Audio Device
 *
 * Simple ring-buffer PCM audio device for the IP30/IP54 machine.
 * Guest fills a ring buffer with PCM samples; QEMU drains it via
 * the host audio backend at the configured sample rate.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_PVAUDIO_H
#define HW_MISC_SGI_PVAUDIO_H

#include "hw/core/sysbus.h"
#include "qemu/audio.h"
#include "qom/object.h"

#define TYPE_SGI_PVAUDIO "sgi-pvaudio"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPVAudioState, SGI_PVAUDIO)

#define SGI_PVAUDIO_MMIO_SIZE   0x100
#define SGI_PVAUDIO_BASE_ADDR   0x1f480400
#define SGI_PVAUDIO_IRQ         8

/* Register offsets (4-byte aligned, big-endian) */
#define PVAUDIO_CTRL            0x00
#define PVAUDIO_STATUS          0x04
#define PVAUDIO_INTR_STAT       0x08
#define PVAUDIO_INTR_MASK       0x0C
#define PVAUDIO_BUF_BASE        0x10
#define PVAUDIO_BUF_SIZE        0x14
#define PVAUDIO_BUF_HEAD        0x18   /* Guest write pointer */
#define PVAUDIO_BUF_TAIL        0x1C   /* QEMU read pointer (read-only) */
#define PVAUDIO_SAMPLE_RATE     0x20
#define PVAUDIO_CHANNELS        0x24
#define PVAUDIO_BITS            0x28

/* CTRL bits */
#define PVAUDIO_CTRL_PLAY       (1 << 0)
#define PVAUDIO_CTRL_RESET      (1 << 1)

/* STATUS bits */
#define PVAUDIO_STATUS_UNDERRUN (1 << 0)
#define PVAUDIO_STATUS_PLAYING  (1 << 1)

/* Interrupt bits */
#define PVAUDIO_INTR_BUF_DONE  (1 << 0)

struct SGIPVAudioState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;

    /* Audio backend (configured via -audiodev / DEFINE_AUDIO_PROPERTIES) */
    AudioBackend *audio_be;
    SWVoiceOut *voice;

    /* Registers */
    uint32_t ctrl;
    uint32_t status;
    uint32_t intr_stat;
    uint32_t intr_mask;
    uint32_t buf_base;
    uint32_t buf_size;
    uint32_t buf_head;
    uint32_t buf_tail;
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t bits;
};

#endif /* HW_MISC_SGI_PVAUDIO_H */
