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

    /* No host backend wired (no explicit audiodev): stay silent. */
    if (!s->audio_be) {
        return;
    }

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

/* Captured-but-unread bytes in the input ring (head - tail, mod size). */
static uint32_t pvaudio_cap_avail(SGIPVAudioState *s)
{
    if (s->cap_size == 0) {
        return 0;
    }
    return (s->cap_head - s->cap_tail) % s->cap_size;
}

/* Bytes of free space left in the capture ring. */
static uint32_t pvaudio_cap_free(SGIPVAudioState *s)
{
    if (s->cap_size == 0) {
        return 0;
    }
    return s->cap_size - pvaudio_cap_avail(s) - 1;  /* keep 1 byte guard */
}

/*
 * Audio input callback: the host backend has `avail` bytes of captured audio
 * for us.  Copy them into the capture ring at CAP_HEAD; if the ring is full,
 * drop the excess and flag an overrun (real capture hardware clips rather
 * than blocking).
 */
static void sgi_pvaudio_in_cb(void *opaque, int avail)
{
    SGIPVAudioState *s = opaque;
    uint8_t buf[4096];
    size_t n;

    if (!(s->ctrl & PVAUDIO_CTRL_RECORD) || s->cap_size == 0) {
        return;
    }

    n = AUD_read(s->voice_in, buf, sizeof(buf));

    while (n > 0) {
        uint32_t free_b = pvaudio_cap_free(s);
        uint32_t chunk = MIN((uint32_t)n, free_b);

        if (chunk == 0) {
            s->status |= PVAUDIO_STATUS_OVERRUN;
            break;  /* drop the rest; the ring is full */
        }

        uint32_t head_offset = s->cap_head % s->cap_size;
        uint32_t first = MIN(chunk, s->cap_size - head_offset);
        dma_memory_write(&address_space_memory,
                         s->cap_base + head_offset,
                         buf, first, MEMTXATTRS_UNSPECIFIED);
        if (chunk > first) {
            dma_memory_write(&address_space_memory, s->cap_base,
                             buf + first, chunk - first, MEMTXATTRS_UNSPECIFIED);
        }
        s->cap_head = (s->cap_head + chunk) % s->cap_size;
        n -= chunk;
    }

    if (pvaudio_cap_avail(s) > 0) {
        s->intr_stat |= PVAUDIO_INTR_CAP_READY;
        sgi_pvaudio_update_irq(s);
    }
}

/*
 * (Re)open the capture voice with current parameters.
 */
static void sgi_pvaudio_open_capture_voice(SGIPVAudioState *s)
{
    struct audsettings as;

    if (!s->audio_be) {
        return;
    }

    if (s->voice_in) {
        AUD_close_in(s->audio_be, s->voice_in);
        s->voice_in = NULL;
    }

    as.freq = s->sample_rate ? s->sample_rate : 44100;
    as.nchannels = s->channels ? (int)s->channels : 2;
    as.fmt = (s->bits == 8) ? AUDIO_FORMAT_U8 : AUDIO_FORMAT_S16;
    as.endianness = 1; /* big-endian (MIPS) */

    s->voice_in = AUD_open_in(s->audio_be, NULL, "sgi-pvaudio-in",
                               s, sgi_pvaudio_in_cb, &as);
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
    case PVAUDIO_CAP_BASE:
        return s->cap_base;
    case PVAUDIO_CAP_SIZE:
        return s->cap_size;
    case PVAUDIO_CAP_HEAD:
        return s->cap_head;
    case PVAUDIO_CAP_TAIL:
        return s->cap_tail;
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
            s->cap_head = 0;
            s->cap_tail = 0;
            if (s->voice) {
                AUD_set_active_out(s->voice, 0);
            }
            if (s->voice_in) {
                AUD_set_active_in(s->voice_in, 0);
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
        if (s->voice_in) {
            AUD_set_active_in(s->voice_in, (val & PVAUDIO_CTRL_RECORD) != 0);
            if (!(val & PVAUDIO_CTRL_RECORD)) {
                s->status &= ~PVAUDIO_STATUS_OVERRUN;
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
    case PVAUDIO_CAP_BASE:
        s->cap_base = val;
        break;
    case PVAUDIO_CAP_SIZE:
        s->cap_size = val;
        s->cap_head = 0;
        s->cap_tail = 0;
        break;
    case PVAUDIO_CAP_HEAD:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Write to read-only CAP_HEAD\n", __func__);
        break;
    case PVAUDIO_CAP_TAIL:
        s->cap_tail = val;
        break;
    case PVAUDIO_SAMPLE_RATE:
        s->sample_rate = val;
        if (s->ctrl & PVAUDIO_CTRL_PLAY) {
            sgi_pvaudio_open_voice(s);
            if (s->voice) {
                AUD_set_active_out(s->voice, 1);
            }
        }
        if (s->ctrl & PVAUDIO_CTRL_RECORD) {
            sgi_pvaudio_open_capture_voice(s);
            if (s->voice_in) {
                AUD_set_active_in(s->voice_in, 1);
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
        if (s->ctrl & PVAUDIO_CTRL_RECORD) {
            sgi_pvaudio_open_capture_voice(s);
            if (s->voice_in) {
                AUD_set_active_in(s->voice_in, 1);
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
        if (s->ctrl & PVAUDIO_CTRL_RECORD) {
            sgi_pvaudio_open_capture_voice(s);
            if (s->voice_in) {
                AUD_set_active_in(s->voice_in, 1);
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

    /* MMIO + IRQ are always present so the guest driver can attach to the
     * device regardless of whether a host audio backend is wired.  The MMIO
     * region responds; playback is only produced when an audiodev is
     * EXPLICITLY configured (`-global sgi-pvaudio.audiodev=aud0`).  This keeps
     * default (no-audiodev) boots byte-for-byte silent and side-effect free —
     * we do NOT fall back to audio_get_default_audio_be() (which would try to
     * open a real host device on headless/CI boots). */
    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvaudio_ops, s,
                          "sgi-pvaudio", SGI_PVAUDIO_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    /* Default parameters */
    s->sample_rate = 44100;
    s->channels = 2;
    s->bits = 16;

    /* audio_be is set by DEFINE_AUDIO_PROPERTIES only when audiodev= is given.
     * Open the host voice only in that explicit case. */
    if (s->audio_be) {
        sgi_pvaudio_open_voice(s);
    }
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
