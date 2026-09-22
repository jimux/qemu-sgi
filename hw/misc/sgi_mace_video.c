/*
 * SGI O2 MACE video block: VIN1 / VIN2 / VOUT + I2C master.
 *
 * See include/hw/misc/sgi_mace_video.h for the hardware map.  The
 * register semantics and the I2C transaction protocol are taken from
 * the IP32 PROM firmware (prom-building/src/fw/video.c, mvpregs.h),
 * which is the same contract the loadable guest driver `mvp.o` uses.
 *
 * M0: VIN/VOUT register file, capture descriptor-DMA block, I2C master
 *     transaction engine, SAA7111/SAA7185 slaves, video UST/MSC,
 *     interrupt path.
 * M1: capture DMA that moves frames from a source buffer into the
 *     guest's 32x64K page descriptor list, driven by an internal test
 *     pattern or by an optional host "video-in" chardev.
 *
 * Copyright (c) 2025 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/display/saa7111.h"
#include "hw/display/saa7185.h"
#include "hw/misc/sgi_mace_video.h"
#include "hw/misc/sgi_video_source.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "migration/vmstate.h"
#include "qobject/qdict.h"
#include "trace.h"

#include <signal.h>
#include <sys/wait.h>

/* ------------------------------------------------------------------ */
/* helpers                                                             */

static uint64_t mvp_ust_now(void)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return (uint32_t)(ns / 960);   /* MACE_UST_PERIOD_NS */
}

static void mvp_guest_write(uint64_t addr, const uint8_t *buf, size_t len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

static bool mvp_fetch_pages(MVPChannelState *ch, uint16_t pages[MVP_MAX_PAGES])
{
    uint8_t raw[MVP_MAX_PAGES * 2];
    uint64_t addr = ch->next_desc & MVP_NDA_ADDRESS_MASK;
    unsigned i;

    if (!(ch->next_desc & MVP_NDA_VALID)) {
        return false;
    }
    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, raw, sizeof(raw)) !=
        MEMTX_OK) {
        return false;
    }
    for (i = 0; i < MVP_MAX_PAGES; i++) {
        pages[i] = lduw_be_p(raw + i * 2);
    }
    return true;
}

/*
 * Write a run of bytes across the 32x64K descriptor pages, starting at
 * buffer byte offset @base.  @base is the MACE FIELD_OFFSET (the first
 * pixel of the first page of the buffer); in interleaved mode successive
 * video lines are written as separate runs, each stepping a frame line.
 */
static size_t mvp_pages_write(const uint16_t pages[MVP_MAX_PAGES],
                              const uint8_t *buf, size_t len, size_t base)
{
    size_t off = base;

    while (len) {
        unsigned page = off >> 16;
        unsigned poff = off & 0xffff;
        size_t chunk;

        if (page >= MVP_MAX_PAGES || pages[page] == 0) {
            break;   /* zero descriptor mid-field = overflow */
        }
        chunk = MIN(len, (size_t)0x10000 - poff);
        mvp_guest_write(((uint64_t)pages[page] << 16) | poff, buf, chunk);
        off += chunk;
        buf += chunk;
        len -= chunk;
    }
    return off - base;
}

/*
 * @@SEMANTICS@@ Read a run of bytes across the 32x64K descriptor pages,
 * starting at buffer byte offset @base.  This is the output (VOUT) mirror
 * of mvp_pages_write: the guest's output descriptor DMA reads the frame
 * the driver programmed out of the same page list the input path writes
 * into, so the device forwards the real guest bytes, not a synthetic
 * signal.  Returns the number of bytes transferred.
 */
static size_t mvp_pages_read(const uint16_t pages[MVP_MAX_PAGES],
                             uint8_t *buf, size_t len, size_t base)
{
    size_t off = base;

    while (len) {
        unsigned page = off >> 16;
        unsigned poff = off & 0xffff;
        size_t chunk;

        if (page >= MVP_MAX_PAGES || pages[page] == 0) {
            break;   /* zero descriptor mid-frame = end/overflow */
        }
        chunk = MIN(len, (size_t)0x10000 - poff);
        address_space_read(&address_space_memory,
                           ((uint64_t)pages[page] << 16) | poff,
                           MEMTXATTRS_UNSPECIFIED, buf, chunk);
        off += chunk;
        buf += chunk;
        len -= chunk;
    }
    return off - base;
}

static unsigned mvp_format_bpp(unsigned fmt)
{
    return (fmt == MVP_FORMAT_YUV422 || fmt == MVP_FORMAT_YUV422_10) ? 2 : 4;
}

static uint32_t mvp_format_fourcc(unsigned fmt)
{
    switch (fmt) {
    case MVP_FORMAT_YUV422:
    case MVP_FORMAT_YUV422_10:
        return MVP_FOURCC_UYVY;
    case MVP_FORMAT_ABGR32:
        return MVP_FOURCC_ABGR;
    default:
        return MVP_FOURCC_RGBA;
    }
}

/* CCIR-601 8-bit YUV for the standard 8 SMPTE bars (75% white). */
static const uint8_t mvp_bars_y[8] = { 235, 210, 170, 145, 106,  81,  41,  16 };
static const uint8_t mvp_bars_u[8] = { 128,  16, 166,  54, 202,  90, 240, 128 };
static const uint8_t mvp_bars_v[8] = { 128, 146,  16,  34, 222, 240, 110, 128 };
static const uint8_t mvp_bars_r[8] = { 235, 235,  16,  16, 235, 235,  16,  16 };
static const uint8_t mvp_bars_g[8] = { 235, 235, 235, 235,  16,  16,  16,  16 };
static const uint8_t mvp_bars_b[8] = { 235,  16, 235,  16, 235,  16, 235,  16 };

static void mvp_fill_test_pattern(uint8_t *buf, unsigned fmt, unsigned stride,
                                  unsigned lines)
{
    unsigned x, y;

    if (stride == 0) {
        return;
    }
    for (y = 0; y < lines; y++) {
        uint8_t *row = buf + (size_t)y * stride;

        if (fmt == MVP_FORMAT_YUV422) {
            /* UYVY: U Y V Y, two pixels per 4-byte group. */
            for (x = 0; x + 3 < stride; x += 4) {
                unsigned bar = (x * 8) / stride;

                row[x + 0] = mvp_bars_u[bar];
                row[x + 1] = mvp_bars_y[bar];
                row[x + 2] = mvp_bars_v[bar];
                row[x + 3] = mvp_bars_y[bar];
            }
        } else if (fmt == MVP_FORMAT_ABGR32) {
            /*
             * @@SEMANTICS@@ Open GL ABGR 32-bit (MACE spec FIGURE 9): the
             * big-endian memory bytes are A, B, G, R.  The pattern
             * previously wrote R,G,B,0xff for every 32-bit format, so the
             * guest converted it as if byte0 were alpha and the bars came
             * out with one channel pinned at 0xff.  The host helper emits
             * ABGR too, so this keeps the no-host negative control the
             * genuine SMPTE bars.
             */
            for (x = 0; x + 3 < stride; x += 4) {
                unsigned bar = (x * 8) / stride;

                row[x + 0] = 0xff;
                row[x + 1] = mvp_bars_b[bar];
                row[x + 2] = mvp_bars_g[bar];
                row[x + 3] = mvp_bars_r[bar];
            }
        } else {
            /* Open GL RGBA 32: big-endian bytes R, G, B, A. */
            for (x = 0; x + 3 < stride; x += 4) {
                unsigned bar = (x * 8) / stride;

                row[x + 0] = mvp_bars_r[bar];
                row[x + 1] = mvp_bars_g[bar];
                row[x + 2] = mvp_bars_b[bar];
                row[x + 3] = 0xff;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* channel register file                                               */

static uint64_t mvp_channel_status(const MVPChannelState *ch, bool is_out)
{
    uint64_t st = ch->status;

    if (ch->dma_running) {
        st |= MVP_STATUS_DMA_ACTIVE;
    }
    if (!is_out) {
        st |= MVP_STATUS_SYNC_PRESENT;   /* analog decoder reports lock */
    }
    return st;
}

/*
 * @@SEMANTICS@@ A MACE video channel drives its CRIME interrupt line at
 * level, asserted while an enabled status bit is pending and dropped when
 * the driver acknowledges by reading STATUS (clear-on-read).  Modelling
 * it as a level that follows the pending status, rather than a latch that
 * stays high once enabled, is what the hardware does and is required by
 * the guest `mvp` driver's interrupt handler.
 */
static void mvp_update_irq(SGIMACEVideoState *s, int idx)
{
    MVPChannelState *ch = (idx == MVP_CRIME_IRQ_VOUT) ? &s->vout
                                                      : &s->vin[idx];
    bool level = false;

    if (ch->control & MVP_CONTROL_ENABLE_VERTSYNC) {
        level |= !!(ch->status & MVP_STATUS_VERTICAL_SYNC);
    }
    if (ch->control & MVP_CONTROL_ENABLE_DMACOMPLETE) {
        level |= !!(ch->status & MVP_STATUS_DMA_COMPLETE);
    }
    if (ch->control & MVP_CONTROL_ENABLE_ERRORS) {
        level |= !!(ch->status & (MVP_STATUS_LOST_SYNC |
                                  MVP_STATUS_BUFFER_OVERFLOW |
                                  MVP_STATUS_HORIZONTAL_OVERFLOW |
                                  MVP_STATUS_VERTICAL_OVERFLOW |
                                  MVP_STATUS_FIFO_OVERFLOW |
                                  MVP_STATUS_CRIME_MEM_ERROR));
    }

    qemu_set_irq(s->crime_irq[idx], level);
}

static uint64_t mvp_channel_read(MVPChannelState *ch, bool is_out,
                                 hwaddr off, unsigned size)
{
    hwaddr reg = off & ~7ULL;
    uint64_t v;

    switch (reg) {
    case MVP_REG_CONTROL:
        v = ch->control;
        break;
    case MVP_REG_STATUS:
        v = mvp_channel_status(ch, is_out);
        /* Interrupt-status bits clear on read; active/sync bits do not. */
        ch->status &= ~MVP_STATUS_INT_MASK;
        break;
    case MVP_REG_CONFIG:
        v = ch->config;
        break;
    case MVP_REG_NEXT_DESC:
        v = ch->next_desc;
        break;
    case MVP_REG_FIELD_OFFSET:
        v = ch->field_offset;
        break;
    case MVP_REG_LINE_WIDTH:
        v = ch->line_width;
        break;
    case MVP_REG_HCLIP_ODD:
        v = ch->hclip_odd;
        break;
    case MVP_REG_VCLIP_ODD:
        v = ch->vclip_odd;
        break;
    case MVP_REG_ALPHA_ODD:
        v = ch->alpha_odd;
        break;
    case MVP_REG_HCLIP_EVEN:
        v = ch->hclip_even;
        break;
    case MVP_REG_VCLIP_EVEN:
        v = ch->vclip_even;
        break;
    case MVP_REG_ALPHA_EVEN:
        v = is_out ? ch->vhw_cfg : ch->alpha_even;
        break;
    default:
        if (reg >= MVP_REG_DMA_DESC &&
            reg < MVP_REG_DMA_DESC + MVP_MAX_PAGES * 2) {
            /* 32 16-bit page pointers at 16-bit stride but 64-bit slots:
             * each descriptor entry occupies one 64-bit register slot
             * 0x80 + i*8 in the driver's struct, yet the spec packs them
             * at 16-bit offsets.  The driver addresses entry i at
             * 0x80 + i*8, so expose entry i in the low half. */
            unsigned idx = (reg - MVP_REG_DMA_DESC) / 8;
            v = (idx < MVP_MAX_PAGES) ? ch->dma_desc[idx] : 0;
        } else {
            v = 0;
        }
        break;
    }

    if (size == 4) {
        return (off & 4) ? (v >> 32) : (v & 0xffffffffu);
    }
    return v;
}

static void mvp_channel_write(MVPChannelState *ch, bool is_out,
                              hwaddr off, uint64_t value, unsigned size)
{
    hwaddr reg = off & ~7ULL;

    if (size == 4) {
        uint64_t cur = mvp_channel_read(ch, is_out, reg, 8);

        if (off & 4) {
            value = (cur & 0xffffffffu) | (value << 32);
        } else {
            value = (cur & 0xffffffff00000000ULL) | (value & 0xffffffffu);
        }
    }

    switch (reg) {
    case MVP_REG_CONTROL:
        ch->control = value & 0x3ff;
        ch->dma_running = !!(value & MVP_CONTROL_ENABLE_DMA);
        if (!ch->dma_running) {
            ch->status = 0;
        }
        break;
    case MVP_REG_STATUS:
        break;   /* RO */
    case MVP_REG_CONFIG:
        /* channel-reset / d1-reset are self-clearing */
        ch->config = value & ~(uint64_t)(MVP_ICONFIG_CHANNEL_RESET |
                                         MVP_ICONFIG_D1_RESET);
        break;
    case MVP_REG_NEXT_DESC:
        ch->next_desc = value & 0xffffffffu;
        /*
         * @@SEMANTICS@@ The descriptor selects which field the
         * FIELD_OFFSET programmed with it applies to; latch the current
         * field_offset into the per-field slot because the FIELD_OFFSET
         * register only holds the last write and the driver programs both
         * descriptors back-to-back.  Input uses the 2-bit field-capture
         * bits (spec TABLE 12: 10 = next odd, 11 = next even, 0x = either);
         * output uses bit0 only (TABLE 13: 0 = send as next odd field,
         * 1 = send as next even field).
         */
        if (value & MVP_NDA_VALID) {
            if (is_out) {
                if (value & 1) {
                    ch->flofs_even = ch->field_offset;
                    ch->flofs_even_valid = true;
                    ch->field_parity = 0;   /* even */
                } else {
                    ch->flofs_odd = ch->field_offset;
                    ch->flofs_odd_valid = true;
                    ch->field_parity = 1;   /* odd */
                }
            } else {
                switch (value & MVP_NDA_CAPTURE_MASK) {
                case 0x2:      /* capture next odd field */
                    ch->flofs_odd = ch->field_offset;
                    ch->flofs_odd_valid = true;
                    break;
                case 0x3:      /* capture next even field */
                    ch->flofs_even = ch->field_offset;
                    ch->flofs_even_valid = true;
                    break;
                default:       /* capture next field (either type) */
                    ch->flofs_odd = ch->flofs_even = ch->field_offset;
                    ch->flofs_odd_valid = ch->flofs_even_valid = true;
                    break;
                }
            }
        }
        break;
    case MVP_REG_FIELD_OFFSET:
        ch->field_offset = value;
        break;
    case MVP_REG_LINE_WIDTH:
        ch->line_width = is_out ? (value & 0x3fffff) : (value & 0xff8);
        if (is_out) {
            /* field_size is a distinct output-only register */
            ch->line_width = value;
        }
        break;
    case MVP_REG_HCLIP_ODD:
        ch->hclip_odd = value;
        break;
    case MVP_REG_VCLIP_ODD:
        ch->vclip_odd = value;
        break;
    case MVP_REG_ALPHA_ODD:
        ch->alpha_odd = value;
        break;
    case MVP_REG_HCLIP_EVEN:
        ch->hclip_even = value;
        break;
    case MVP_REG_VCLIP_EVEN:
        ch->vclip_even = value;
        break;
    case MVP_REG_ALPHA_EVEN:
        if (is_out) {
            /* VHW_CFG: preserve the RO revision code nibble */
            ch->vhw_cfg = (value & ~0xf00000000ULL) | MVP_VHW_CFG_REVISION;
        } else {
            ch->alpha_even = value;
        }
        break;
    default:
        if (reg >= MVP_REG_DMA_DESC &&
            reg < MVP_REG_DMA_DESC + MVP_MAX_PAGES * 2) {
            unsigned idx = (reg - MVP_REG_DMA_DESC) / 8;
            if (idx < MVP_MAX_PAGES) {
                ch->dma_desc[idx] = value & 0xffff;
            }
        }
        break;
    }

    trace_sgi_mace_video_reg(is_out, reg, value, size);
}

/* ------------------------------------------------------------------ */
/* capture / field pacing                                              */

static void mvp_vin_geometry(const MVPChannelState *ch, unsigned *fmt,
                             unsigned *stride, unsigned *lines)
{
    *fmt = (ch->config & MVP_ICONFIG_FORMAT_MASK) >> MVP_ICONFIG_FORMAT_SHIFT;
    *stride = ch->line_width & 0xff8;
    if (*stride == 0) {
        *stride = (*fmt == MVP_FORMAT_YUV422) ? 1440 : 2880;
    }

    {
        unsigned start = ch->vclip_odd & 0x3ff;
        unsigned end = (ch->vclip_odd >> 20) & 0x3ff;

        /*
         * @@SEMANTICS@@ VCLIP_ODD/_EVEN (spec 2.3.5.4 TABLE 18) encode
         * VStart in bits 9:0 and VEnd in bits 29:20 as an INCLUSIVE line
         * range: VEnd = VStart + (#lines to clip) - 1.  The captured
         * field height is therefore end - start + 1.  The guest driver
         * programs the square-pixel 525 field this way -- VCLIP_ODD
         * 0x10004411 -> start 17, end 256 (240 lines) and VCLIP_EVEN
         * 0xff04010 -> start 16, end 255 (240 lines) -- and
         * VL_CAPTURE_INTERLEAVED doubles that to the documented 480-line
         * frame.  The previous end - start count lost the last line of
         * each field, so an interleaved frame wrote only rows 0..477.
         */
        *lines = (end > start) ? (end - start + 1) : 243;
    }
}

static void mvp_capture_field(SGIMACEVideoState *s, MVPChannelState *ch,
                              int idx)
{
    MVPVideoInput *in = &s->input[idx];
    uint16_t pages[MVP_MAX_PAGES];
    unsigned fmt, stride, lines;
    size_t frame_size, written, field_base = 0;
    bool interleaved;
    uint8_t *tmp;

    if (!ch->dma_running) {
        return;
    }
    if (!mvp_fetch_pages(ch, pages)) {
        ch->dma_running = false;
        return;
    }

    mvp_vin_geometry(ch, &fmt, &stride, &lines);
    frame_size = (size_t)stride * lines;
    /*
     * @@SEMANTICS@@ Interleaved (frame) capture is a linear-memory mode
     * only (spec 2.3.1: MEM_MODE linear vs tiled; 2.3.5.3.10 INTERLEAVED
     * enables frame mode).  Observed guest captures are CONFIG bit 16 set
     * with MEM_MODE 00, so the tiled case is deliberately left on the old
     * linear field path rather than guessed.
     */
    interleaved = (ch->config & MVP_ICONFIG_INTERLEAVED) &&
                  ((ch->config & MVP_ICONFIG_MEM_MODE_MASK) == 0);

    tmp = g_malloc(frame_size);
    if (in->frame_buf && in->frame_len && in->frame_width && in->frame_height) {
        /* copy the host frame (top-left aligned, clamped) in place */
        unsigned bpp = (fmt == MVP_FORMAT_YUV422) ? 2 : 4;
        unsigned src_stride = in->frame_width * bpp;
        unsigned copy_lines = MIN(lines, in->frame_height);
        unsigned y;

        memset(tmp, 0, frame_size);
        for (y = 0; y < copy_lines; y++) {
            size_t n = MIN((size_t)stride,
                           MIN((size_t)src_stride, in->frame_len -
                               (size_t)y * src_stride));
            if ((size_t)y * src_stride >= in->frame_len) {
                break;
            }
            memcpy(tmp + (size_t)y * stride,
                   in->frame_buf + (size_t)y * src_stride, n);
        }
    } else {
        mvp_fill_test_pattern(tmp, fmt, stride, lines);
    }

    if (interleaved) {
        /*
         * @@SEMANTICS@@ Frame (interleaved) capture, MACE spec 2.3.2.2
         * FIGURE 11 and 2.3.5.6: in linear interleave mode LINE_WIDTH is
         * added to the current pixel address at the end of a line so an
         * empty line is left for the other field, and FIELD_OFFSET points
         * at the field's first line.  A field therefore occupies every
         * other line: each captured line is written at
         * flofs[parity] + y*2*stride, and the even/odd bases come from
         * the FIELD_OFFSET the driver programmed with each descriptor
         * (latched by NEXT_DESC[1:0], TABLE 12).  This is why the guest
         * frame was previously only one 239-line field with the lower
         * half black: both fields were written from offset 0.
         */
        unsigned p = ch->field_parity & 1;
        size_t pitch = (size_t)stride * 2;
        unsigned y;

        field_base = (p == 1)
            ? (ch->flofs_odd_valid ? ch->flofs_odd : ch->field_offset)
            : (ch->flofs_even_valid ? ch->flofs_even : ch->field_offset);

        written = 0;
        for (y = 0; y < lines; y++) {
            written += mvp_pages_write(pages, tmp + (size_t)y * stride,
                                       stride, field_base + (size_t)y * pitch);
        }
        ch->field_parity ^= 1;
    } else {
        written = mvp_pages_write(pages, tmp, frame_size, 0);
    }
    g_free(tmp);

    trace_sgi_mace_video_vin_field(idx, fmt, stride, lines, written, frame_size,
                                   interleaved ? 1 : 0, field_base);

    ch->status |= MVP_STATUS_DMA_COMPLETE;
}

/*
 * @@SEMANTICS@@ Forward one output field to the host sink.  The output
 * (VOUT) channel is the mirror of a VIN channel: when its descriptor DMA
 * is enabled the device reads the guest frame out of the 32x64K page list
 * (spec TABLE 24 for video DMA) and sends it to the AV1 SAA7185 encoder.
 * QEMU has no analog medium, so it serialises the exact bytes the guest
 * programmed as an MVPF frame on the "video-out" chardev; an external
 * helper records/decodes it.  No frame is ever synthesised: nothing is
 * sent unless the guest armed NEXT_DESC and enabled the DMA.
 *
 * Geometry comes from the output-only FIELD_SIZE register (0x28): lines
 * at bits[21:12], byte line width at bits[11:3]; the pixel format is
 * output CONFIG bits[16:14], the same encoding as the input CONFIG.  The
 * interleaved (frame) layout alternates the field base from FIELD_OFFSET
 * (latched per field type by NEXT_DESC[1:0], exactly like the input path),
 * so the two fields read every other frame line.
 */
static void mvp_output_field(SGIMACEVideoState *s)
{
    MVPChannelState *ch = &s->vout;
    uint16_t pages[MVP_MAX_PAGES];
    unsigned fmt, stride, lines, bpp;
    size_t frame_size, field_base = 0, got;
    bool interleaved;
    uint8_t *tmp;

    if (!ch->dma_running) {
        return;
    }
    /*
     * No valid descriptor means the next field is skipped (spec TABLE
     * 13); leave DMA enabled and wait for the driver to arm the next one.
     */
    if (!mvp_fetch_pages(ch, pages)) {
        return;
    }

    lines = (ch->line_width >> 12) & 0x3ff;
    stride = ch->line_width & 0xff8;
    /* Output CONFIG pixel format is bits[16:14] (MACE spec TABLE 11),
     * not the input [12:10]; the same 3-bit encoding otherwise. */
    fmt = (ch->config >> 14) & 7;
    bpp = mvp_format_bpp(fmt);
    if (stride == 0) {
        stride = (fmt == MVP_FORMAT_YUV422) ? 1280 : 2560;
    }
    if (lines == 0) {
        lines = 240;
    }
    frame_size = (size_t)stride * lines;
    /* Output interleaved mode: CONFIG bit20 INTERLEAVED, MEM_MODE bit21. */
    interleaved = (ch->config & (1u << 20)) &&
                  (((ch->config >> 21) & 1) == 0);

    tmp = g_malloc(frame_size);
    memset(tmp, 0, frame_size);

    if (interleaved) {
        unsigned p = ch->field_parity & 1;
        size_t pitch = (size_t)stride * 2;
        unsigned y;

        field_base = (p == 1)
            ? (ch->flofs_odd_valid ? ch->flofs_odd : ch->field_offset)
            : (ch->flofs_even_valid ? ch->flofs_even : ch->field_offset);
        got = 0;
        for (y = 0; y < lines; y++) {
            got += mvp_pages_read(pages, tmp + (size_t)y * stride, stride,
                                  field_base + (size_t)y * pitch);
        }
    } else {
        got = mvp_pages_read(pages, tmp, frame_size, ch->field_offset);
    }

    /* Hardware zeroes the descriptor Valid bit once it has been copied. */
    ch->next_desc &= ~(uint64_t)MVP_NDA_VALID;

    if (got == 0) {
        g_free(tmp);
        return;
    }

    if (qemu_chr_fe_get_driver(&s->sink.video_out)) {
        uint8_t hdr[20];
        unsigned width = stride / bpp;
        uint32_t fourcc = mvp_format_fourcc(fmt);

        stl_be_p(hdr, MVP_FRAME_MAGIC);
        stl_be_p(hdr + 4, width);
        stl_be_p(hdr + 8, lines);
        stl_be_p(hdr + 12, fourcc);
        stl_be_p(hdr + 16, got);
        qemu_chr_fe_write_all(&s->sink.video_out, hdr, sizeof(hdr));
        qemu_chr_fe_write_all(&s->sink.video_out, tmp, got);
        s->sink.frames_sent++;
    }
    g_free(tmp);

    trace_sgi_mace_video_vout_field(fmt, stride, lines, got, frame_size,
                                    interleaved ? 1 : 0, field_base);
    ch->status |= MVP_STATUS_DMA_COMPLETE;
}

static void mvp_field_tick(void *opaque)
{
    SGIMACEVideoState *s = opaque;
    int i;

    /*
     * @@SEMANTICS@@ The analog decoder is free-running: an input channel
     * receives a field boundary every field period whether or not its
     * capture DMA is enabled.  Advance MSC/UST and latch VSYNC each field
     * so the driver's frontier/sync handshake can complete before it
     * programs the descriptor DMA.  (Previously MSC only advanced inside
     * mvp_capture_field, which returns early when DMA is off, so the
     * driver blocked after enabling the VSYNC interrupt.)
     */
    for (i = 0; i < 2; i++) {
        MVPChannelState *ch = &s->vin[i];

        mvp_capture_field(s, ch, i);
        ch->status |= MVP_STATUS_VERTICAL_SYNC;
        ch->msc++;
        ch->ust = mvp_ust_now();
        mvp_update_irq(s, i);
    }
    /*
     * @@SEMANTICS@@ VOUT's timing generator free-runs exactly like the
     * input channels: the output field boundary (VSYNC) and the VOUT
     * frame counter/UST advance every field period whether or not output
     * DMA is enabled.  The mvp driver arms the output channel by enabling
     * only the VSYNC interrupt and then waits for the frontier MSC before
     * programming the descriptor DMA, so gating the tick on ENABLE_DMA
     * (as this used to) starved it of the handshake and the driver never
     * armed the output transfer.  The interrupt line itself still follows
     * CONTROL.ENABLE_VERTSYNC via mvp_update_irq.
     */
    mvp_output_field(s);
    s->vout.msc++;
    s->vout.ust = mvp_ust_now();
    s->vout.status |= MVP_STATUS_VERTICAL_SYNC;
    mvp_update_irq(s, MVP_CRIME_IRQ_VOUT);

    timer_mod(s->field_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MVP_FIELD_PERIOD_NS);
}

/* ------------------------------------------------------------------ */
/* I2C master                                                          */

static void mvp_i2c_end(SGIMACEVideoState *s)
{
    if (s->xfer_open) {
        i2c_end_transfer(s->i2c_bus);
        s->xfer_open = false;
        s->xfer_recv = false;
        s->recv_pending = false;
    }
}

static uint64_t mvp_i2c_read(SGIMACEVideoState *s, hwaddr off, unsigned size)
{
    uint64_t v;
    hwaddr reg = off & ~7ULL;

    switch (reg) {
    case MACE_I2C_CONFIG:
    case MACE_I2C_CONFIG_ALIAS:
        /* RO input bits report the idle-high bus state. */
        v = s->i2c_config | (1 << 4) | (1 << 5);
        break;
    case MACE_I2C_CONTROL:
        v = (s->xfer_open ? I2C_NOT_IDLE : 0) |
            (s->i2c_ctrl & (I2C_READ | I2C_HOLD_BUS)) |
            (s->nack ? I2C_NACK : 0) |
            (s->bus_err ? I2C_BUS_ERR : 0);
        break;
    case MACE_I2C_DATA:
        if (s->xfer_open && s->xfer_recv && !s->recv_pending) {
            s->i2c_data = i2c_recv(s->i2c_bus);
            s->recv_pending = true;
        }
        v = s->i2c_data;
        /* A data read consumes the latched byte. */
        s->recv_pending = false;
        break;
    default:
        v = 0;
        break;
    }

    trace_sgi_mace_i2c(0, reg, v);
    return (size == 4 && (off & 4)) ? (v >> 32) : (v & 0xffffffffu);
}

static void mvp_i2c_write(SGIMACEVideoState *s, hwaddr off, uint64_t value,
                          unsigned size)
{
    hwaddr reg = off & ~7ULL;
    uint8_t byte = value & 0xff;

    trace_sgi_mace_i2c(1, reg, value);

    switch (reg) {
    case MACE_I2C_CONFIG:
    case MACE_I2C_CONFIG_ALIAS:
        s->i2c_config = byte & 0x0f;
        if (byte & I2C_CFG_RESET) {
            mvp_i2c_end(s);
            s->nack = false;
            s->bus_err = false;
        }
        break;
    case MACE_I2C_CONTROL:
        if (!(byte & I2C_NOT_IDLE)) {
            /* force idle / release */
            mvp_i2c_end(s);
            s->nack = false;
            s->bus_err = false;
        }
        s->i2c_ctrl = byte & 0xff;
        if ((byte & I2C_NOT_IDLE) && (byte & I2C_READ) &&
            s->xfer_open && s->xfer_recv && !s->recv_pending) {
            s->i2c_data = i2c_recv(s->i2c_bus);
            s->recv_pending = true;
        }
        break;
    case MACE_I2C_DATA:
        s->i2c_data = byte;
        if (!s->xfer_open) {
            /* address phase: bit0 = R/W */
            uint8_t addr = byte >> 1;
            bool recv = byte & 1;
            int rc = recv ? i2c_start_recv(s->i2c_bus, addr)
                          : i2c_start_send(s->i2c_bus, addr);

            if (rc) {
                s->nack = true;
                s->bus_err = true;
            } else {
                s->xfer_open = true;
                s->xfer_addr = addr;
                s->xfer_recv = recv;
                s->recv_pending = false;
                s->nack = false;
            }
        } else if (!s->xfer_recv) {
            /* payload phase (write): subaddress then data */
            if (i2c_send(s->i2c_bus, byte)) {
                s->nack = true;
            }
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* UST/MSC overlay (VIN1 +0x38, VIN2 +0x40, VOUT +0x48)                */

static MVPChannelState *mvp_ust_video_channel(SGIMACEVideoState *s,
                                              hwaddr off, unsigned *pair)
{
    /*
     * @@SEMANTICS@@ The UST/MSC MemoryRegion is mapped at the VIN1 pair's
     * window offset (MACE + 0x340038), so the offset the device receives is
     * relative to VIN1 (0x00/0x08/0x10), not to the 0x1f340000 MACE base.
     * Re-base it to the window-absolute pair offsets the constants name
     * (0x38/0x40/0x48).  Without this the switch matched nothing, every
     * UST/MSC read selected no channel and returned 0, and the loadable mvp
     * driver's frontier helper (mvptransfer_interrupt -> .text+0x1908) saw a
     * zero UST and never advanced the transfer past CONTROL=0x2.
     */
    *pair = (off & ~7ULL) + MVP_UST_MSC_VIN1;
    switch (*pair) {
    case MVP_UST_MSC_VIN1:
        return &s->vin[0];
    case MVP_UST_MSC_VIN2:
        return &s->vin[1];
    case MVP_UST_MSC_VOUT:
        return &s->vout;
    default:
        return NULL;
    }
}

static uint64_t mvp_ust_read(void *opaque, hwaddr off, unsigned size)
{
    SGIMACEVideoState *s = opaque;
    unsigned pair;
    MVPChannelState *ch = mvp_ust_video_channel(s, off, &pair);
    uint64_t v;

    if (!ch) {
        return 0;
    }
    v = ((uint64_t)ch->msc << 32) | ch->ust;
    trace_sgi_mace_video_ust(0, pair, v, size);
    /*
     * @@SEMANTICS@@ The pair is a big-endian 64-bit register: the lower
     * address (the pair base, off 0) holds the MSB word -- the media
     * stream counter / frame count -- and the upper address (off 4)
     * holds the UST (kernel sys/mace.h: MACE_VIN1_MSC_UST; the loadable
     * mvp driver reads the high word as the frame counter and the low
     * word as the UST).  The device is DEVICE_BIG_ENDIAN, so a 4-byte
     * access at off 0 is the high word.
     *
     * This function also previously collapsed every access to a single
     * 32-bit word: an 8-byte `ld` (the mvp frontier helper's
     * pciio_pio_read64 at the pair) returned only v[31:0], so the
     * helper's frame-counter word was always 0 and its wrap / field-id
     * never advanced.  A 64-bit read must return the whole pair.
     */
    if (size == 4) {
        return (off & 4) ? (v & 0xffffffffu) : (v >> 32);
    }
    return v;
}

static void mvp_ust_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    SGIMACEVideoState *s = opaque;
    unsigned pair;
    MVPChannelState *ch = mvp_ust_video_channel(s, off, &pair);

    if (!ch) {
        return;
    }
    trace_sgi_mace_video_ust(1, pair, value, size);
    /*
     * @@SEMANTICS@@ Big-endian register: off 0 is the frame-count (MSC)
     * word, off 4 is the UST word.  The loadable mvp driver's ISR writes
     * its 64-bit-extended field frontier back to the pair with a 32-bit
     * `sw` at the pair base (mvptransfer_interrupt +0x1ec: `sw a4,56(at)`),
     * i.e. off 0, and reads it back as the frame counter it uses for the
     * field-id/wrap logic.  Writing it to `ust` (the old little-endian
     * convention) left the counter the helper read unchanged and the
     * wrap/field-id never advanced.  A full 64-bit write sets both words.
     */
    if (size == 4 && (off & 4)) {
        ch->ust = value;
    } else if (size == 4) {
        ch->msc = value;
    } else {
        ch->msc = value >> 32;
        ch->ust = value & 0xffffffffu;
    }
}

/* ------------------------------------------------------------------ */
/* MMIO plumbing                                                       */

static uint64_t mvp_vin_read(void *opaque, hwaddr off, unsigned size)
{
    SGIMACEVideoState *s = opaque;
    int idx = (off >= 0x80000) ? 1 : 0;
    uint64_t v = mvp_channel_read(&s->vin[idx], false, off & 0x7ffff, size);

    trace_sgi_mace_video_rd(false, (off & 0x7ffff) & ~7ULL, v, size);
    /* Reading STATUS acknowledges (clears) the pending interrupt bits. */
    if (((off & 0x7ffff) & ~7ULL) == MVP_REG_STATUS) {
        mvp_update_irq(s, idx);
    }
    return v;
}

static void mvp_vin_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    SGIMACEVideoState *s = opaque;
    int idx = (off >= 0x80000) ? 1 : 0;

    mvp_channel_write(&s->vin[idx], false, off & 0x7ffff, value, size);
    mvp_update_irq(s, idx);
}

static uint64_t mvp_vout_read(void *opaque, hwaddr off, unsigned size)
{
    SGIMACEVideoState *s = opaque;
    uint64_t v = mvp_channel_read(&s->vout, true, off, size);

    trace_sgi_mace_video_rd(true, off & ~7ULL, v, size);
    if ((off & ~7ULL) == MVP_REG_STATUS) {
        mvp_update_irq(s, MVP_CRIME_IRQ_VOUT);
    }
    return v;
}

static void mvp_vout_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    SGIMACEVideoState *s = opaque;

    mvp_channel_write(&s->vout, true, off, value, size);
    mvp_update_irq(s, MVP_CRIME_IRQ_VOUT);
}

static const MemoryRegionOps mvp_vin_ops = {
    .read = mvp_vin_read,
    .write = mvp_vin_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 8 },
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static const MemoryRegionOps mvp_vout_ops = {
    .read = mvp_vout_read,
    .write = mvp_vout_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 8 },
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static uint64_t mvp_i2c_read_op(void *opaque, hwaddr off, unsigned size)
{
    return mvp_i2c_read(opaque, off, size);
}

static void mvp_i2c_write_op(void *opaque, hwaddr off, uint64_t value,
                             unsigned size)
{
    mvp_i2c_write(opaque, off, value, size);
}

static const MemoryRegionOps mvp_i2c_mem_ops = {
    .read = mvp_i2c_read_op,
    .write = mvp_i2c_write_op,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 8 },
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static const MemoryRegionOps mvp_ust_ops_impl = {
    .read = mvp_ust_read,
    .write = mvp_ust_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 8 },
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

void sgi_mace_video_map_into(SGIMACEVideoState *s, MemoryRegion *mace_mr)
{
    /*
     * The MACE region already spans the video windows, so overlay them
     * with priority 1: VIN (both channels) 0x100000..0x1fffff, VOUT at
     * 0x200000, I2C at 0x330000, and the video UST/MSC pairs at 0x340038.
     */
    memory_region_add_subregion_overlap(mace_mr, 0x100000, &s->vin_mr, 1);
    memory_region_add_subregion_overlap(mace_mr, 0x200000, &s->vout_mr, 1);
    memory_region_add_subregion_overlap(mace_mr, 0x330000, &s->i2c_mr, 1);
    memory_region_add_subregion_overlap(mace_mr, 0x340038, &s->ustmsc_mr, 1);
}

/* ------------------------------------------------------------------ */
/* host video source chardev                                           */

static int mvp_video_in_can_receive(void *opaque)
{
    MVPVideoInput *in = opaque;

    return MVP_FRAME_MAX - in->rx_len;
}

static void mvp_video_in_reset_frame(MVPVideoInput *in)
{
    in->rx_len = 0;
    in->rx_payload_len = 0;
}

static void mvp_video_in_receive(void *opaque, const uint8_t *buf, int size)
{
    MVPVideoInput *in = opaque;

    while (size > 0) {
        size_t space = MVP_FRAME_MAX - in->rx_len;
        size_t n = MIN((size_t)size, space);

        memcpy(in->rx_buf + in->rx_len, buf, n);
        in->rx_len += n;
        buf += n;
        size -= n;

        if (in->rx_len >= 20 && in->rx_payload_len == 0) {
            uint32_t magic = ldl_be_p(in->rx_buf);
            uint32_t w = ldl_be_p(in->rx_buf + 4);
            uint32_t h = ldl_be_p(in->rx_buf + 8);
            uint32_t fourcc = ldl_be_p(in->rx_buf + 12);
            uint32_t plen = ldl_be_p(in->rx_buf + 16);

            if (magic != MVP_FRAME_MAGIC || plen > MVP_FRAME_MAX - 20) {
                mvp_video_in_reset_frame(in);
                continue;
            }
            in->rx_payload_len = plen;
            in->frame_width = w;
            in->frame_height = h;
            in->frame_fourcc = fourcc;
        }

        if (in->rx_payload_len && in->rx_len >= 20 + in->rx_payload_len) {
            size_t plen = in->rx_payload_len;

            memcpy(in->frame_buf, in->rx_buf + 20, plen);
            in->frame_len = plen;
            trace_sgi_mace_video_frame(in->index, in->frame_width,
                                       in->frame_height, in->frame_fourcc,
                                       plen);
            /* keep any trailing bytes for the next frame */
            memmove(in->rx_buf, in->rx_buf + 20 + plen,
                    in->rx_len - 20 - plen);
            in->rx_len -= 20 + plen;
            in->rx_payload_len = 0;
        } else if (in->rx_len == MVP_FRAME_MAX) {
            mvp_video_in_reset_frame(in);
        }
    }
}

/* ------------------------------------------------------------------ */
/* host source attach/detach (sgi-video-source interface)              */
/*
 * @@SEMANTICS@@ The GTK "Video" menu drives this.  QEMU never links or
 * embeds a video decoder: attaching spawns an external helper process
 * (ffmpeg, wrapped by a small script) that connects to the video-in
 * chardev and streams MVPF frames; detaching kills it.  The emulated
 * capture DMA is unchanged -- it copies whatever frame the chardev last
 * delivered, so an attach is visible to the guest on the next field.
 */

/*
 * Map an input name to its channel index.  Accepts the documented
 * "vin1"/"vin2" names and, for the monitor command, the bare "1"/"2"
 * spelling.  Returns -1 for anything else.
 */
static int mvp_video_input_index(const char *input)
{
    if (!input) {
        return -1;
    }
    if (!strcmp(input, "vin1") || !strcmp(input, "1")) {
        return 0;
    }
    if (!strcmp(input, "vin2") || !strcmp(input, "2")) {
        return 1;
    }
    return -1;
}

static const char *mvp_video_input_name(int idx)
{
    return idx == 0 ? "vin1" : "vin2";
}

static void mvp_video_helper_exit(GPid pid, gint status, gpointer opaque)
{
    MVPVideoInput *in = opaque;
    char *src = g_strdup(in->helper_source);

    g_spawn_close_pid(pid);
    if (in->helper_pid == pid) {
        in->helper_pid = 0;
        in->helper_watch = 0;
        g_free(in->helper_source);
        in->helper_source = NULL;
        in->helper_is_url = false;
        in->frame_len = 0;   /* fall back to the internal test pattern */
        /*
         * @@SEMANTICS@@ A helper that exits non-zero means the source could
         * not be opened (unreachable URL, missing ffmpeg, bad path): the
         * helper validates the source before it ever connects, so this is a
         * real error, not a transient EOF.  Report it (QEMU stderr + -D log)
         * rather than silently showing the internal pattern.
         */
        if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "video: helper for %s exited (code %d); source "
                          "'%s' unavailable, reverting to internal pattern\n",
                          mvp_video_input_name(in->index),
                          WEXITSTATUS(status), src ? src : "(none)");
        } else if (WIFSIGNALED(status)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "video: helper for %s killed by signal %d; source "
                          "'%s' stopped\n",
                          mvp_video_input_name(in->index),
                          WTERMSIG(status), src ? src : "(none)");
        }
    }
    g_free(src);
}

static void mvp_video_out_helper_exit(GPid pid, gint status, gpointer opaque)
{
    MVPVideoSink *out = opaque;

    g_spawn_close_pid(pid);
    if (out->helper_pid == pid) {
        out->helper_pid = 0;
        out->helper_watch = 0;
        g_free(out->helper_source);
        out->helper_source = NULL;
        out->helper_is_url = false;
    }
}

static void mvp_video_out_detach(SGIMACEVideoState *s)
{
    MVPVideoSink *out = &s->sink;
    GPid pid = out->helper_pid;
    int i;

    if (out->helper_watch) {
        g_source_remove(out->helper_watch);
        out->helper_watch = 0;
    }
    out->helper_pid = 0;
    g_free(out->helper_source);
    out->helper_source = NULL;
    out->helper_is_url = false;

    if (pid <= 0) {
        return;
    }
    kill(pid, SIGTERM);
    for (i = 0; i < 50; i++) {
        if (waitpid(pid, NULL, WNOHANG) == pid) {
            g_spawn_close_pid(pid);
            return;
        }
        g_usleep(20000);
    }
    kill(pid, SIGKILL);
    waitpid(pid, NULL, 0);
    g_spawn_close_pid(pid);
}

static bool mvp_video_out_attach(SGIMACEVideoState *s, const char *source,
                                 bool is_url, Error **errp)
{
    MVPVideoSink *out = &s->sink;
    char *argv[8];
    GError *err = NULL;

    if (!out->video_helper) {
        error_setg(errp, "no helper configured for vout: pass "
                   "-global sgi-mace-video.video-helper-out=<wrapper>");
        return false;
    }
    if (!out->video_out_path) {
        error_setg(errp, "no video-out path configured for vout: pass "
                   "-global sgi-mace-video.video-out-path=<socket>");
        return false;
    }
    if (!source || !*source) {
        error_setg(errp, "empty video output destination");
        return false;
    }

    mvp_video_out_detach(s);

    argv[0] = out->video_helper;
    argv[1] = (char *)"--socket";
    argv[2] = out->video_out_path;
    argv[3] = (char *)"--source";
    argv[4] = (char *)source;
    argv[5] = is_url ? (char *)"--url" : NULL;
    argv[6] = NULL;

    if (!g_spawn_async(NULL, argv, NULL,
                       G_SPAWN_DO_NOT_REAP_CHILD |
                       G_SPAWN_STDOUT_TO_DEV_NULL |
                       G_SPAWN_STDERR_TO_DEV_NULL,
                       NULL, NULL, &out->helper_pid, &err)) {
        error_setg(errp, "cannot launch video sink helper '%s': %s",
                   out->video_helper, err->message);
        g_error_free(err);
        return false;
    }
    out->helper_watch = g_child_watch_add(out->helper_pid,
                                          mvp_video_out_helper_exit, out);
    out->helper_source = g_strdup(source);
    out->helper_is_url = is_url;
    return true;
}

static void mvp_video_detach(SGIVideoSource *src, const char *input)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(src);
    int idx = mvp_video_input_index(input);
    MVPVideoInput *in;
    GPid pid;
    int i;

    if (input && (!strcmp(input, "vout") || !strcmp(input, "3"))) {
        mvp_video_out_detach(s);
        return;
    }
    if (idx < 0) {
        return;
    }
    in = &s->input[idx];
    pid = in->helper_pid;

    if (in->helper_watch) {
        g_source_remove(in->helper_watch);
        in->helper_watch = 0;
    }
    in->helper_pid = 0;
    g_free(in->helper_source);
    in->helper_source = NULL;
    in->helper_is_url = false;
    in->frame_len = 0;

    if (pid <= 0) {
        return;
    }
    kill(pid, SIGTERM);
    /* Reap synchronously so detach is complete when it returns. */
    for (i = 0; i < 50; i++) {
        if (waitpid(pid, NULL, WNOHANG) == pid) {
            g_spawn_close_pid(pid);
            return;
        }
        g_usleep(20000);
    }
    kill(pid, SIGKILL);
    waitpid(pid, NULL, 0);
    g_spawn_close_pid(pid);
}

static bool mvp_video_attach(SGIVideoSource *src, const char *input,
                             const char *source, bool is_url, Error **errp)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(src);
    int idx = mvp_video_input_index(input);
    MVPVideoInput *in;
    char *argv[8];
    GError *err = NULL;

    if (input && (!strcmp(input, "vout") || !strcmp(input, "3"))) {
        return mvp_video_out_attach(s, source, is_url, errp);
    }
    if (idx < 0) {
        error_setg(errp, "unknown video input '%s' (use vin1, vin2 or vout)",
                   input ? input : "");
        return false;
    }
    in = &s->input[idx];
    if (!in->video_helper) {
        error_setg(errp, "no helper configured for %s: pass "
                   "-global sgi-mace-video.video-helper%s=<wrapper>",
                   mvp_video_input_name(idx), idx == 0 ? "" : "2");
        return false;
    }
    if (!in->video_in_path) {
        error_setg(errp, "no video-in path configured for %s: pass "
                   "-global sgi-mace-video.video-in-path%s=<socket>",
                   mvp_video_input_name(idx), idx == 0 ? "" : "2");
        return false;
    }
    if (!source || !*source) {
        error_setg(errp, "empty video source");
        return false;
    }

    mvp_video_detach(src, input);

    argv[0] = in->video_helper;
    argv[1] = (char *)"--socket";
    argv[2] = in->video_in_path;
    argv[3] = (char *)"--source";
    argv[4] = (char *)source;
    argv[5] = is_url ? (char *)"--url" : NULL;
    argv[6] = NULL;

    /*
     * Keep the helper's stderr: its preflight prints a clear one-line
     * reason when ffmpeg is missing or the source/URL cannot be opened.
     */
    if (!g_spawn_async(NULL, argv, NULL,
                       G_SPAWN_DO_NOT_REAP_CHILD |
                       G_SPAWN_STDOUT_TO_DEV_NULL,
                       NULL, NULL, &in->helper_pid, &err)) {
        error_setg(errp, "cannot launch video helper '%s': %s",
                   in->video_helper, err->message);
        g_error_free(err);
        return false;
    }
    in->helper_watch = g_child_watch_add(in->helper_pid,
                                         mvp_video_helper_exit, in);
    in->helper_source = g_strdup(source);
    in->helper_is_url = is_url;
    return true;
}

static bool mvp_video_is_attached(SGIVideoSource *src, const char *input)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(src);
    int idx = mvp_video_input_index(input);

    if (input && (!strcmp(input, "vout") || !strcmp(input, "3"))) {
        return s->sink.helper_pid > 0;
    }
    return idx >= 0 && s->input[idx].helper_pid > 0;
}

static const char *mvp_video_describe(SGIVideoSource *src, const char *input)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(src);
    int idx = mvp_video_input_index(input);

    if (input && (!strcmp(input, "vout") || !strcmp(input, "3"))) {
        return s->sink.helper_source;
    }
    return idx >= 0 ? s->input[idx].helper_source : NULL;
}

/*
 * Scriptable attach/detach path (HMP).  It reuses exactly the same
 * sgi_video_source_* helpers the QMP video-attach / video-detach commands
 * call (defined in ui/ui-qmp-cmds.c), which in turn call the same
 * sgi-video-source methods the GTK menu uses.  One implementation, three
 * front-ends; headless (-display none) use needs no GTK.
 */
void hmp_video_attach(Monitor *mon, const QDict *qdict)
{
    const char *input = qdict_get_str(qdict, "input");
    const char *source = qdict_get_str(qdict, "source");
    Error *err = NULL;

    if (!sgi_video_source_attach(input, source, &err)) {
        monitor_printf(mon, "video_attach: %s\n", error_get_pretty(err));
        error_free(err);
        return;
    }
    monitor_printf(mon, "video_attach: %s <- %s%s\n", input, source,
                   strstr(source, "://") ? " (url)" : "");
}

void hmp_video_detach(Monitor *mon, const QDict *qdict)
{
    const char *input = qdict_get_str(qdict, "input");
    Error *err = NULL;

    if (!sgi_video_source_detach(input, &err)) {
        monitor_printf(mon, "video_detach: %s\n", error_get_pretty(err));
        error_free(err);
        return;
    }
    monitor_printf(mon, "video_detach: %s detached\n", input);
}

/* ------------------------------------------------------------------ */
/* reset / realize                                                     */

static void mvp_channel_reset(MVPChannelState *ch)
{
    memset(ch, 0, sizeof(*ch));
}

static void sgi_mace_video_reset(DeviceState *dev)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(dev);

    mvp_channel_reset(&s->vin[0]);
    mvp_channel_reset(&s->vin[1]);
    mvp_channel_reset(&s->vout);
    s->vout.vhw_cfg = MVP_VHW_CFG_REVISION;

    mvp_i2c_end(s);
    s->i2c_config = 0;
    s->i2c_ctrl = 0;
    s->i2c_data = 0;
    s->recv_pending = false;
    s->nack = false;
    s->bus_err = false;

    for (int i = 0; i < MVP_NUM_VIN; i++) {
        mvp_video_in_reset_frame(&s->input[i]);
        s->input[i].frame_len = 0;
    }

    /*
     * @@SEMANTICS@@ reset hygiene: a guest reset must not leave a decoder
     * helper streaming into a chardev the machine is about to reinitialise.
     * Detach every endpoint (a no-op when nothing is attached), which SIGTERMs
     * and reaps any running helper.  A re-attach after reset starts a fresh
     * one.
     */
    for (int i = 0; i < MVP_NUM_VIN; i++) {
        mvp_video_detach(SGI_VIDEO_SOURCE(s), mvp_video_input_name(i));
    }
    mvp_video_detach(SGI_VIDEO_SOURCE(s), "vout");

    timer_mod(s->field_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MVP_FIELD_PERIOD_NS);
}

static void sgi_mace_video_realize(DeviceState *dev, Error **errp)
{
    SGIMACEVideoState *s = SGI_MACE_VIDEO(dev);

    /*
     * One region covers both input channels: VIN1 at +0x00000 and VIN2
     * at +0x08000 within it.
     */
    memory_region_init_io(&s->vin_mr, OBJECT(dev), &mvp_vin_ops, s,
                          "sgi-mace-vin", 0x100000);
    memory_region_init_io(&s->vout_mr, OBJECT(dev), &mvp_vout_ops, s,
                          "sgi-mace-vout", 0x8000);
    memory_region_init_io(&s->i2c_mr, OBJECT(dev), &mvp_i2c_mem_ops, s,
                          "sgi-mace-i2c", 0x10000);
    memory_region_init_io(&s->ustmsc_mr, OBJECT(dev), &mvp_ust_ops_impl, s,
                          "sgi-mace-video-ustmsc", MVP_UST_MSC_SIZE);

    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", MVP_NUM_IRQS);

    /*
     * MACE I2C master bus.  The AV1 Analog A/V module provides the
     * SAA7111 decoder (MACE port A) and SAA7185 encoder (MACE port E)
     * as slaves.
     */
    s->i2c_bus = i2c_init_bus(dev, "mace-i2c");
    i2c_slave_create_simple(s->i2c_bus, TYPE_SAA7111, MACE_I2C_ADDR_SAA7111);
    i2c_slave_create_simple(s->i2c_bus, TYPE_SAA7185, MACE_I2C_ADDR_SAA7185);

    for (int i = 0; i < MVP_NUM_VIN; i++) {
        s->input[i].index = i;
        s->input[i].frame_buf = g_malloc(MVP_FRAME_MAX);
        s->input[i].rx_buf = g_malloc(MVP_FRAME_MAX);
    }

    s->field_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mvp_field_tick, s);

    /*
     * Install the chardev receive handlers unconditionally: the backend
     * (e.g. a Unix socket helper) normally connects after realize, and
     * qemu_chr_fe_set_handlers starts delivery whenever it does.  Each
     * input has its own chardev and its own frame assembly, so the two
     * VIN channels can carry independent host sources.
     */
    for (int i = 0; i < MVP_NUM_VIN; i++) {
        qemu_chr_fe_set_handlers(&s->input[i].video_in,
                                 mvp_video_in_can_receive,
                                 mvp_video_in_receive,
                                 NULL, NULL, &s->input[i], NULL, true);
    }
}

static const Property sgi_mace_video_properties[] = {
    /* VIN1 host source */
    DEFINE_PROP_CHR("video-in", SGIMACEVideoState, input[0].video_in),
    DEFINE_PROP_STRING("video-helper", SGIMACEVideoState,
                       input[0].video_helper),
    DEFINE_PROP_STRING("video-in-path", SGIMACEVideoState,
                       input[0].video_in_path),
    /* VIN2 host source (independent chardev + helper) */
    DEFINE_PROP_CHR("video-in2", SGIMACEVideoState, input[1].video_in),
    DEFINE_PROP_STRING("video-helper2", SGIMACEVideoState,
                       input[1].video_helper),
    DEFINE_PROP_STRING("video-in2-path", SGIMACEVideoState,
                       input[1].video_in_path),
    /* VOUT host sink (o2 video-out; reverse of the inputs) */
    DEFINE_PROP_CHR("video-out", SGIMACEVideoState, sink.video_out),
    DEFINE_PROP_STRING("video-helper-out", SGIMACEVideoState,
                       sink.video_helper),
    DEFINE_PROP_STRING("video-out-path", SGIMACEVideoState,
                       sink.video_out_path),
};

static const VMStateDescription vmstate_sgi_mace_video_chan = {
    .name = "sgi-mace-video/chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(control, MVPChannelState),
        VMSTATE_UINT64(config, MVPChannelState),
        VMSTATE_UINT64(next_desc, MVPChannelState),
        VMSTATE_UINT64(field_offset, MVPChannelState),
        VMSTATE_UINT64(line_width, MVPChannelState),
        VMSTATE_UINT64(hclip_odd, MVPChannelState),
        VMSTATE_UINT64(vclip_odd, MVPChannelState),
        VMSTATE_UINT64(alpha_odd, MVPChannelState),
        VMSTATE_UINT64(hclip_even, MVPChannelState),
        VMSTATE_UINT64(vclip_even, MVPChannelState),
        VMSTATE_UINT64(alpha_even, MVPChannelState),
        VMSTATE_UINT64(vhw_cfg, MVPChannelState),
        VMSTATE_UINT16_ARRAY(dma_desc, MVPChannelState, MVP_MAX_PAGES),
        VMSTATE_UINT64(flofs_odd, MVPChannelState),
        VMSTATE_UINT64(flofs_even, MVPChannelState),
        VMSTATE_BOOL(flofs_odd_valid, MVPChannelState),
        VMSTATE_BOOL(flofs_even_valid, MVPChannelState),
        VMSTATE_UINT32(field_parity, MVPChannelState),
        VMSTATE_UINT64(status, MVPChannelState),
        VMSTATE_BOOL(dma_running, MVPChannelState),
        VMSTATE_UINT32(msc, MVPChannelState),
        VMSTATE_UINT64(ust, MVPChannelState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_sgi_mace_video = {
    .name = "sgi-mace-video",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(vin, SGIMACEVideoState, 2, 1,
                             vmstate_sgi_mace_video_chan, MVPChannelState),
        VMSTATE_STRUCT(vout, SGIMACEVideoState, 1,
                       vmstate_sgi_mace_video_chan, MVPChannelState),
        VMSTATE_UINT8(i2c_config, SGIMACEVideoState),
        VMSTATE_UINT8(i2c_ctrl, SGIMACEVideoState),
        VMSTATE_UINT8(i2c_data, SGIMACEVideoState),
        VMSTATE_BOOL(xfer_open, SGIMACEVideoState),
        VMSTATE_BOOL(xfer_recv, SGIMACEVideoState),
        VMSTATE_BOOL(recv_pending, SGIMACEVideoState),
        VMSTATE_BOOL(nack, SGIMACEVideoState),
        VMSTATE_BOOL(bus_err, SGIMACEVideoState),
        VMSTATE_END_OF_LIST()
    },
};

static void sgi_mace_video_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SGIVideoSourceClass *vsc = SGI_VIDEO_SOURCE_CLASS(klass);

    dc->realize = sgi_mace_video_realize;
    device_class_set_legacy_reset(dc, sgi_mace_video_reset);
    dc->vmsd = &vmstate_sgi_mace_video;
    device_class_set_props(dc, sgi_mace_video_properties);

    vsc->attach = mvp_video_attach;
    vsc->detach = mvp_video_detach;
    vsc->is_attached = mvp_video_is_attached;
    vsc->describe = mvp_video_describe;
}

static const TypeInfo sgi_mace_video_info = {
    .name = TYPE_SGI_MACE_VIDEO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIMACEVideoState),
    .class_init = sgi_mace_video_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_SGI_VIDEO_SOURCE },
        { }
    },
};

static const TypeInfo sgi_video_source_info = {
    .name = TYPE_SGI_VIDEO_SOURCE,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(SGIVideoSourceClass),
};

static void sgi_mace_video_register_types(void)
{
    type_register_static(&sgi_video_source_info);
    type_register_static(&sgi_mace_video_info);
}

type_init(sgi_mace_video_register_types);
