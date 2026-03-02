/*
 * SGI Paravirtual Audio Device
 *
 * Simple ring-buffer PCM audio device. The guest writes PCM data into
 * a physically contiguous ring buffer and advances BUF_HEAD. QEMU's
 * audio subsystem drains from BUF_TAIL via AUD_open_out callbacks.
 * BUF_DONE IRQ fires whenever QEMU drains a chunk from the buffer,
 * signaling that buffer space is available.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/misc/sgi_pvaudio.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"

static void sgi_pvaudio_update_irq(SGIPVAudioState *s)
{
    if (s->intr_stat & s->intr_mask) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

/*
 * Compute available bytes in the ring buffer (head - tail, mod size).
 */
static uint32_t pvaudio_avail(SGIPVAudioState *s)
{
    if (s->buf_size == 0) {
        return 0;
    }
    return (s->buf_head - s->buf_tail) % s->buf_size;
}

/*
 * Audio output callback: QEMU's audio subsystem calls this with
 * the number of bytes it can accept from us.
 */
static void sgi_pvaudio_out_cb(void *opaque, int avail)
{
    SGIPVAudioState *s = opaque;
    uint32_t to_read, chunk;
    uint8_t buf[4096];

    if (!(s->ctrl & PVAUDIO_CTRL_PLAY) || s->buf_size == 0) {
        return;
    }

    to_read = MIN(pvaudio_avail(s), (uint32_t)avail);
    if (to_read == 0) {
        /* Underrun */
        s->status |= PVAUDIO_STATUS_UNDERRUN;
        return;
    }

    while (to_read > 0) {
        /* How much until we wrap around the ring buffer? */
        uint32_t tail_offset = s->buf_tail % s->buf_size;
        chunk = MIN(to_read, s->buf_size - tail_offset);
        chunk = MIN(chunk, sizeof(buf));

        dma_memory_read(&address_space_memory,
                        s->buf_base + tail_offset,
                        buf, chunk, MEMTXATTRS_UNSPECIFIED);

        size_t written = AUD_write(s->voice, buf, chunk);
        if (written == 0) {
            break;
        }

        s->buf_tail = (s->buf_tail + written) % s->buf_size;
        to_read -= written;

        if (written < chunk) {
            break;
        }
    }

    /* Fire BUF_DONE to wake any blocked guest writers */
    s->intr_stat |= PVAUDIO_INTR_BUF_DONE;
    sgi_pvaudio_update_irq(s);
}

/*
 * (Re)open the audio voice with current parameters.
 */
static void sgi_pvaudio_open_voice(SGIPVAudioState *s)
{
    struct audsettings as;

    if (s->voice) {
        AUD_close_out(s->audio_be, s->voice);
        s->voice = NULL;
    }

    as.freq = s->sample_rate ? s->sample_rate : 44100;
    as.nchannels = s->channels ? (int)s->channels : 2;
    as.fmt = (s->bits == 8) ? AUDIO_FORMAT_U8 : AUDIO_FORMAT_S16;
    as.endianness = 1; /* big-endian (MIPS) */

    s->voice = AUD_open_out(s->audio_be, NULL, "sgi-pvaudio",
                             s, sgi_pvaudio_out_cb, &as);
}

static uint64_t sgi_pvaudio_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIPVAudioState *s = opaque;

    switch (addr) {
    case PVAUDIO_CTRL:
        return s->ctrl;
    case PVAUDIO_STATUS:
        return s->status;
    case PVAUDIO_INTR_STAT:
        return s->intr_stat;
    case PVAUDIO_INTR_MASK:
        return s->intr_mask;
    case PVAUDIO_BUF_BASE:
        return s->buf_base;
    case PVAUDIO_BUF_SIZE:
        return s->buf_size;
    case PVAUDIO_BUF_HEAD:
        return s->buf_head;
    case PVAUDIO_BUF_TAIL:
        return s->buf_tail;
    case PVAUDIO_SAMPLE_RATE:
        return s->sample_rate;
    case PVAUDIO_CHANNELS:
        return s->channels;
    case PVAUDIO_BITS:
        return s->bits;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }
}

static void sgi_pvaudio_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGIPVAudioState *s = opaque;

    switch (addr) {
    case PVAUDIO_CTRL:
        if (val & PVAUDIO_CTRL_RESET) {
            s->ctrl = 0;
            s->status = 0;
            s->intr_stat = 0;
            s->buf_head = 0;
            s->buf_tail = 0;
            if (s->voice) {
                AUD_set_active_out(s->voice, 0);
            }
            sgi_pvaudio_update_irq(s);
            return;
        }
        s->ctrl = val;
        if (s->voice) {
            if (val & PVAUDIO_CTRL_PLAY) {
                s->status |= PVAUDIO_STATUS_PLAYING;
                s->status &= ~PVAUDIO_STATUS_UNDERRUN;
                AUD_set_active_out(s->voice, 1);
            } else {
                s->status &= ~PVAUDIO_STATUS_PLAYING;
                AUD_set_active_out(s->voice, 0);
            }
        }
        break;
    case PVAUDIO_STATUS:
        s->status &= ~val;  /* W1C */
        break;
    case PVAUDIO_INTR_STAT:
        s->intr_stat &= ~val;  /* W1C */
        sgi_pvaudio_update_irq(s);
        break;
    case PVAUDIO_INTR_MASK:
        s->intr_mask = val;
        sgi_pvaudio_update_irq(s);
        break;
    case PVAUDIO_BUF_BASE:
        s->buf_base = val;
        break;
    case PVAUDIO_BUF_SIZE:
        s->buf_size = val;
        s->buf_head = 0;
        s->buf_tail = 0;
        break;
    case PVAUDIO_BUF_HEAD:
        s->buf_head = val;
        break;
    case PVAUDIO_BUF_TAIL:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Write to read-only BUF_TAIL\n", __func__);
        break;
    case PVAUDIO_SAMPLE_RATE:
        s->sample_rate = val;
        if (s->ctrl & PVAUDIO_CTRL_PLAY) {
            sgi_pvaudio_open_voice(s);
            if (s->voice) {
                AUD_set_active_out(s->voice, 1);
            }
        }
        break;
    case PVAUDIO_CHANNELS:
        s->channels = val;
        if (s->ctrl & PVAUDIO_CTRL_PLAY) {
            sgi_pvaudio_open_voice(s);
            if (s->voice) {
                AUD_set_active_out(s->voice, 1);
            }
        }
        break;
    case PVAUDIO_BITS:
        s->bits = val;
        if (s->ctrl & PVAUDIO_CTRL_PLAY) {
            sgi_pvaudio_open_voice(s);
            if (s->voice) {
                AUD_set_active_out(s->voice, 1);
            }
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps sgi_pvaudio_ops = {
    .read = sgi_pvaudio_read,
    .write = sgi_pvaudio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sgi_pvaudio_realize(DeviceState *dev, Error **errp)
{
    SGIPVAudioState *s = SGI_PVAUDIO(dev);

    if (!AUD_backend_check(&s->audio_be, errp)) {
        return;
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvaudio_ops, s,
                          "sgi-pvaudio", SGI_PVAUDIO_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    /* Default parameters */
    s->sample_rate = 44100;
    s->channels = 2;
    s->bits = 16;

    sgi_pvaudio_open_voice(s);
}

static const Property sgi_pvaudio_properties[] = {
    DEFINE_AUDIO_PROPERTIES(SGIPVAudioState, audio_be),
};

static void sgi_pvaudio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_pvaudio_realize;
    device_class_set_props(dc, sgi_pvaudio_properties);
}

static const TypeInfo sgi_pvaudio_info = {
    .name = TYPE_SGI_PVAUDIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPVAudioState),
    .class_init = sgi_pvaudio_class_init,
};

static void sgi_pvaudio_register_types(void)
{
    type_register_static(&sgi_pvaudio_info);
}

type_init(sgi_pvaudio_register_types)
