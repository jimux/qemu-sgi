/*
 * QEMU SGI VINO (Indy video input) — hardware-authentic register shell.
 *
 * VINO is the video-input ASIC on the Indy (IP24): the digital IndyCam port
 * plus composite/S-Video analog inputs digitised through an external video
 * decoder.  It is probed by the PROM and by IRIX at physical 0x00080000
 * (EISA space): a 32-bit version word at base+4 whose bits [7:4] identify
 * the part as 0xB (IRIX sys/IP22.h VINO_PHYS_BASE1 / VINO_CHIP_ID; IP22.c
 * simplevinoprobe()).  The PROM's own diagnostics also touch offsets 0x42,
 * 0x43 and 0x61.
 *
 * Presence is a device property, DEFAULT OFF.  With present=off the region
 * still exists but every read returns all-ones, which is exactly what an
 * unmapped aperture returns, so a plain `-M indy` invocation behaves
 * byte-for-byte as before (vinoProbe fails, VINO is simply absent).  With
 * present=on the version register answers and the PROM/kernel recognise the
 * part.  This is the "protect -M indy rigorously" gate: the host-source /
 * capture work is developed behind present=on until both the PROM boot and
 * the desktop gates pass on a plain invocation.
 *
 * Register map source: the IRIX 5.3 IP22boot kernel driver vino.a
 * (vino_regs.o master_reg_tbl; 39 direct-MMIO regs id 0..0x26 at stride 8;
 * ids 0x27..0x3d route to the analog decoder over VINO I2C; ids 0x3e..0x47
 * route to the IndyCam over VINO I2C).  Only the direct-MMIO shell and the
 * version register are modelled here; capture DMA / I2C / genlock follow.
 *
 * Copyright (c) 2026 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/misc/sgi_vino.h"
#include "hw/misc/sgi_video_source.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "trace.h"

#include <signal.h>
#include <sys/wait.h>

/* One captured field per 60 Hz field period. */
#define SGI_VINO_FIELD_PERIOD_NS  (NANOSECONDS_PER_SECOND / 60)

/* Host video source: the MVPF wire format and the two packings the shared
 * helper emits (sgi-irix-re/o2helpers/vin_helper.py).  The packing is carried
 * in the frame header, so honour it rather than assuming one. */
#define SGI_VINO_FRAME_MAGIC  0x4d565046u   /* 'MVPF' */
#define SGI_VINO_FOURCC_ABGR  0x41424752u   /* 'ABGR' */
#define SGI_VINO_FOURCC_UYVY  0x55595659u   /* 'UYVY' */

/* VINO has a single video input (the IndyCam); channel A/B are DMA channels,
 * not inputs.  Accept the natural name plus the bare index. */
static bool sgi_vino_input_is(const char *input)
{
    return input && (!strcmp(input, "vino") || !strcmp(input, "indycam") ||
                     !strcmp(input, "1"));
}

static int sgi_vino_video_can_receive(void *opaque)
{
    SGIVinoState *s = opaque;

    return SGI_VINO_FRAME_MAX - s->rx_len;
}

static void sgi_vino_video_reset_rx(SGIVinoState *s)
{
    s->rx_len = 0;
    s->rx_payload_len = 0;
}

/* Parse the MVPF stream the helper sends: a 20-byte big-endian header
 * (magic, width, height, fourcc, payload length) then the payload.  The most
 * recent complete frame is kept; anything malformed is dropped. */
static void sgi_vino_video_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIVinoState *s = opaque;

    while (size > 0) {
        size_t space = SGI_VINO_FRAME_MAX - s->rx_len;
        size_t n = MIN((size_t)size, space);

        memcpy(s->rx_buf + s->rx_len, buf, n);
        s->rx_len += n;
        buf += n;
        size -= n;

        if (s->rx_len >= 20 && s->rx_payload_len == 0) {
            uint32_t magic = ldl_be_p(s->rx_buf);
            uint32_t plen = ldl_be_p(s->rx_buf + 16);

            if (magic != SGI_VINO_FRAME_MAGIC ||
                plen > SGI_VINO_FRAME_MAX - 20) {
                sgi_vino_video_reset_rx(s);
                continue;
            }
            s->frame_width = ldl_be_p(s->rx_buf + 4);
            s->frame_height = ldl_be_p(s->rx_buf + 8);
            s->frame_fourcc = ldl_be_p(s->rx_buf + 12);
            s->rx_payload_len = plen;
        }

        if (s->rx_payload_len && s->rx_len >= 20 + s->rx_payload_len) {
            size_t plen = s->rx_payload_len;

            memcpy(s->frame_buf, s->rx_buf + 20, plen);
            s->frame_len = plen;
            trace_sgi_vino_frame(s->frame_width, s->frame_height,
                                 s->frame_fourcc, plen);
            memmove(s->rx_buf, s->rx_buf + 20 + plen, s->rx_len - 20 - plen);
            s->rx_len -= 20 + plen;
            s->rx_payload_len = 0;
        } else if (s->rx_len == SGI_VINO_FRAME_MAX) {
            sgi_vino_video_reset_rx(s);
        }
    }
}

/* Read/write one 32-bit VINO channel register by its byte offset relative to
 * the channel base.  The register file stores big-endian bytes, so go
 * through the byte array to stay access-size agnostic. */
static uint32_t sgi_vino_chreg(SGIVinoState *s, int ch, unsigned rel)
{
    hwaddr o = (ch ? SGI_VINO_CH_B_BASE : SGI_VINO_CH_A_BASE) + rel;

    return ((uint32_t)s->regs[o] << 24) | ((uint32_t)s->regs[o + 1] << 16) |
           ((uint32_t)s->regs[o + 2] << 8) | s->regs[o + 3];
}

static void sgi_vino_chreg_set(SGIVinoState *s, int ch, unsigned rel,
                               uint32_t val)
{
    hwaddr o = (ch ? SGI_VINO_CH_B_BASE : SGI_VINO_CH_A_BASE) + rel;

    s->regs[o] = val >> 24;
    s->regs[o + 1] = val >> 16;
    s->regs[o + 2] = val >> 8;
    s->regs[o + 3] = val;
}

static uint32_t sgi_vino_control(SGIVinoState *s)
{
    return ((uint32_t)s->regs[SGI_VINO_CONTROL_OFFSET] << 24) |
           ((uint32_t)s->regs[SGI_VINO_CONTROL_OFFSET + 1] << 16) |
           ((uint32_t)s->regs[SGI_VINO_CONTROL_OFFSET + 2] << 8) |
           (uint32_t)s->regs[SGI_VINO_CONTROL_OFFSET + 3];
}

static uint32_t sgi_vino_intstat(SGIVinoState *s)
{
    return ((uint32_t)s->regs[SGI_VINO_INTSTAT_OFFSET] << 24) |
           ((uint32_t)s->regs[SGI_VINO_INTSTAT_OFFSET + 1] << 16) |
           ((uint32_t)s->regs[SGI_VINO_INTSTAT_OFFSET + 2] << 8) |
           (uint32_t)s->regs[SGI_VINO_INTSTAT_OFFSET + 3];
}

/* Assert the interrupt line while any latched status bit has its matching
 * per-channel enable set in the control register.  The driver latches EOF/EOD
 * in the status word (0x14) and clears it by writing 0x14; that write and any
 * control write re-evaluate this.  The line is a direct INT3 local1 bit on
 * Indy (VECTOR_VIDEO), delivered through the HPC3. */
static void sgi_vino_update_irq(SGIVinoState *s)
{
    uint32_t stat = sgi_vino_intstat(s);
    uint32_t ctl = sgi_vino_control(s);
    bool pending = false;

    if ((stat & SGI_VINO_INT_A_EOF) && (ctl & SGI_VINO_CTRL_A_EOF_INT)) {
        pending = true;
    }
    if ((stat & SGI_VINO_INT_A_FIFO) && (ctl & SGI_VINO_CTRL_A_FIFO_INT)) {
        pending = true;
    }
    if ((stat & SGI_VINO_INT_A_EOD) && (ctl & SGI_VINO_CTRL_A_EOD_INT)) {
        pending = true;
    }
    if ((stat & SGI_VINO_INT_B_EOF) && (ctl & SGI_VINO_CTRL_B_EOF_INT)) {
        pending = true;
    }
    if ((stat & SGI_VINO_INT_B_FIFO) && (ctl & SGI_VINO_CTRL_B_FIFO_INT)) {
        pending = true;
    }
    if ((stat & SGI_VINO_INT_B_EOD) && (ctl & SGI_VINO_CTRL_B_EOD_INT)) {
        pending = true;
    }

    qemu_set_irq(s->irq, pending ? 1 : 0);
}

/* Refill the four-entry descriptor cache from the auto-advancing fetch
 * pointer, then step it on by 16 bytes.  The fetch is traced so the driver's
 * real table can be checked against the spec. */
static void sgi_vino_fetch_group(SGIVinoState *s, int ch)
{
    uint32_t addr = s->next_desc[ch];
    int i;

    for (i = 0; i < SGI_VINO_DESC_PER_FETCH; i++) {
        s->desc[ch][i] = address_space_ldl_be(&address_space_memory,
                                              (hwaddr)addr + 4 * i,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
    }
    trace_sgi_vino_desc(ch, addr, s->desc[ch][0], s->desc[ch][1],
                        s->desc[ch][2], s->desc[ch][3]);
    s->next_desc[ch] = addr + 4 * SGI_VINO_DESC_PER_FETCH;
    s->cache_pos[ch] = 0;
    s->cache_valid[ch] = true;
}

/* Emit one captured field for a channel by walking its descriptor table.
 *
 * Per the VINO Design Specification 099-8937-001 section 2.3, the field's
 * height is signalled by the video decoder's end-of-field, not by a register,
 * and the line_size/line_count registers describe bytes within a line
 * (line_count is a running byte counter that starts at zero).  So a field is
 * modelled as one pass over the descriptors the driver built for it: each
 * descriptor names a 4K page that capture DMA fills, a stop descriptor ends
 * the pass with an EOD interrupt, and end of pass raises EOF.  The pixels are
 * a deterministic luma ramp -- nothing is presented as real camera data, but
 * a full field is delivered so the driver's DMA completes.  Recorded host
 * frames replace this once the sgi-video-source seam lands. */
/* The 32-bit capture word at linear field byte offset @o: taken from the
 * latest host frame when one is attached, else from the deterministic luma
 * ramp.  The host frame is tiled across the capture field (source and capture
 * need not match in size); bytes past the source line are left black. */
static uint32_t sgi_vino_field_word(SGIVinoState *s, uint32_t o,
                                    uint32_t row_bytes, bool have_row)
{
    uint8_t b[4];
    int i;

    if (s->frame_len && s->frame_width && s->frame_height && have_row) {
        unsigned bpp = (s->frame_fourcc == SGI_VINO_FOURCC_UYVY) ? 2 : 4;
        uint32_t srow_bytes = s->frame_width * bpp;

        for (i = 0; i < 4; i++) {
            uint32_t p = o + i;
            uint32_t c = p % row_bytes;

            if (c >= srow_bytes) {
                b[i] = 0;                               /* right padding */
            } else {
                uint32_t sr = (p / row_bytes) % s->frame_height;

                b[i] = s->frame_buf[sr * srow_bytes + c];
            }
        }
    } else {
        uint8_t g = (o >> 2) & 0xff;                    /* luma ramp */

        b[0] = b[1] = b[2] = b[3] = g;
    }

    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | b[3];
}

static void sgi_vino_emit_field(SGIVinoState *s, int ch)
{
    uint32_t bytes = 0;
    uint32_t line_size = sgi_vino_chreg(s, ch, SGI_VINO_CH_LINE_SIZE);
    uint32_t row_bytes = line_size + 8;   /* bytes per line (spec: -8) */
    bool have_row = row_bytes > 8;        /* driver programmed a geometry */
    unsigned pages;

    trace_sgi_vino_field_src(ch, row_bytes, s->frame_len,
                             have_row && s->frame_len ? 1 : 0);

    for (pages = 0; pages < 512; pages++) {
        uint32_t desc, page, off;

        if (!s->cache_valid[ch]) {
            sgi_vino_fetch_group(s, ch);
        }
        desc = s->desc[ch][s->cache_pos[ch]];
        if (desc & SGI_VINO_DESC_STOP) {
            /* Stop descriptor: end of the descriptor table. */
            s->regs[SGI_VINO_INTSTAT_OFFSET + 3] |= ch ? SGI_VINO_INT_B_EOD
                                                       : SGI_VINO_INT_A_EOD;
            s->dma_en[ch] = false;
            sgi_vino_chreg_set(s, ch, SGI_VINO_CH_PAGE_INDEX, 0);
            break;
        }
        if (desc & SGI_VINO_DESC_JUMP) {
            /* Jump descriptor: next fetch resumes at its page address. */
            s->next_desc[ch] = desc & 0x3ffff000u;
            s->cache_valid[ch] = false;
            continue;
        }
        page = desc & 0x3ffff000u;
        if (!page) {
            break;
        }
        for (off = 0; off < 0x1000; off += 4) {
            uint32_t px = sgi_vino_field_word(s, bytes + off, row_bytes,
                                              have_row);

            address_space_stl_be(&address_space_memory, page | off, px,
                                 MEMTXATTRS_UNSPECIFIED, NULL);
        }
        bytes += 0x1000;
        sgi_vino_chreg_set(s, ch, SGI_VINO_CH_PAGE_INDEX, 0);

        if (++s->cache_pos[ch] >= SGI_VINO_DESC_PER_FETCH) {
            s->cache_valid[ch] = false;
        }
    }

    s->field_count[ch]++;
    sgi_vino_chreg_set(s, ch, SGI_VINO_CH_FIELD_COUNT, s->field_count[ch]);
    s->regs[SGI_VINO_INTSTAT_OFFSET + 3] |= ch ? SGI_VINO_INT_B_EOF
                                               : SGI_VINO_INT_A_EOF;
    trace_sgi_vino_field(ch, s->field_count[ch], bytes);
    sgi_vino_update_irq(s);
}

static void sgi_vino_field_tick(void *opaque)
{
    SGIVinoState *s = opaque;
    bool any = false;
    int ch;

    for (ch = 0; ch < 2; ch++) {
        if (s->dma_en[ch]) {
            sgi_vino_emit_field(s, ch);
            any = true;
        }
    }
    if (any) {
        timer_mod(s->field_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  SGI_VINO_FIELD_PERIOD_NS);
    }
}

/* Arm or stop the field timer to match the set of DMA-enabled channels. */
static void sgi_vino_update_timer(SGIVinoState *s)
{
    if (s->dma_en[0] || s->dma_en[1]) {
        if (!timer_pending(s->field_timer)) {
            timer_mod(s->field_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SGI_VINO_FIELD_PERIOD_NS);
        }
    } else {
        timer_del(s->field_timer);
    }
}

static uint64_t sgi_vino_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIVinoState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (!s->present) {
        /* Absent part: reads look exactly like an unmapped aperture. */
        trace_sgi_vino_read(offset, size, ~0ULL, 0);
        return ~0ULL;
    }
    if (offset + size > SGI_VINO_REG_SIZE) {
        trace_sgi_vino_read(offset, size, ~0ULL, 1);
        return ~0ULL;
    }
    if (offset == SGI_VINO_I2C_CTRL_OFFSET) {
        /* Status: idle, so all of busy/xfer/ack/error read clear.  The driver
         * polls these bits and only cares that a transfer completes. */
        val = 0;
    } else if (offset == SGI_VINO_I2C_DATA_OFFSET) {
        /* Data byte from the addressed slave, advancing the sub-address. */
        uint8_t *f = s->i2c_alt_dev ? s->i2c_alt : s->i2c_dec;
        val = f[s->i2c_ptr++];
    } else {
        /* Big-endian byte assembly, so byte/half/word accesses all agree. */
        for (i = 0; i < size; i++) {
            val = (val << 8) | s->regs[offset + i];
        }
    }
    trace_sgi_vino_read(offset, size, val, 1);
    return val;
}

static void sgi_vino_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    SGIVinoState *s = opaque;
    unsigned i;

    if (!s->present || offset + size > SGI_VINO_REG_SIZE) {
        trace_sgi_vino_write(offset, value, size, s->present ? 1 : 0);
        return;
    }
    trace_sgi_vino_write(offset, value, size, 1);
    if (offset == SGI_VINO_I2C_CTRL_OFFSET) {
        /* Non-zero arms a byte stream; zero forces the bus idle.  Either way
         * the next data byte starts with the device address. */
        s->i2c_first = 1;
        return;
    }
    if (offset == SGI_VINO_I2C_DATA_OFFSET) {
        uint8_t byte = value & 0xff;
        uint8_t *f = s->i2c_alt_dev ? s->i2c_alt : s->i2c_dec;

        if (s->i2c_first) {
            /* Device address byte; bit 0 selects read vs write. */
            s->i2c_alt_dev = ((byte & 0xfe) != SGI_VINO_I2C_ADDR_WRITE);
            s->i2c_first = 0;
            s->i2c_expect_sub = !(byte & 1);
        } else if (s->i2c_expect_sub) {
            s->i2c_ptr = byte;
            s->i2c_expect_sub = 0;
        } else {
            f[s->i2c_ptr++] = byte;
        }
        return;
    }
    for (i = 0; i < size; i++) {
        uint8_t byte = (value >> (8 * (size - 1 - i))) & 0xff;

        if (offset + i >= SGI_VINO_INTSTAT_OFFSET &&
            offset + i < SGI_VINO_INTSTAT_OFFSET + 4) {
            /* Interrupt status is write-0-to-clear: the bits are set by
             * hardware and cleared by software writing a zero (spec, Interrupt
             * register); writing a one has no effect.  Storing the value here
             * instead latched the upper bits as 1 and the driver never saw a
             * clearable status. */
            s->regs[offset + i] &= byte;
        } else {
            s->regs[offset + i] = byte;
        }
    }
    if (offset == SGI_VINO_CH_A_BASE + SGI_VINO_CH_NEXT4DESC ||
        offset == SGI_VINO_CH_B_BASE + SGI_VINO_CH_NEXT4DESC) {
        /* The driver programmed the descriptor table; the engine fetches it
         * (four at a time, auto-advancing) once capture is enabled. */
        int ch = offset >= SGI_VINO_CH_B_BASE ? 1 : 0;

        s->next_desc[ch] = sgi_vino_chreg(s, ch, SGI_VINO_CH_NEXT4DESC);
        s->cache_valid[ch] = false;
        s->cache_pos[ch] = 0;
    } else if (offset == SGI_VINO_CONTROL_OFFSET) {
        uint32_t ctl = 0;
        int ch;

        for (i = 0; i < 4; i++) {
            ctl = (ctl << 8) | s->regs[offset + i];
        }
        for (ch = 0; ch < 2; ch++) {
            bool en = ctl & (ch ? SGI_VINO_CTRL_B_DMA_ENBL
                                : SGI_VINO_CTRL_A_DMA_ENBL);

            if (en && !s->dma_en[ch]) {
                /* Capture start: the cache refills from next_4_desc (fall
                 * back to start_desc if the driver only set that). */
                if (!s->next_desc[ch]) {
                    s->next_desc[ch] =
                        sgi_vino_chreg(s, ch, SGI_VINO_CH_START_DESC);
                }
                s->cache_valid[ch] = false;
                s->cache_pos[ch] = 0;
                s->dma_en[ch] = true;
            } else if (!en) {
                s->dma_en[ch] = false;
            }
        }
        sgi_vino_update_timer(s);
    }
    /* A write to the status word (0x14) clears latched bits; a control write
     * changes the enables.  Re-evaluate the interrupt line either way. */
    sgi_vino_update_irq(s);
}

static const MemoryRegionOps sgi_vino_ops = {
    .read = sgi_vino_read,
    .write = sgi_vino_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sgi_vino_reset(DeviceState *dev)
{
    SGIVinoState *s = SGI_VINO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->dma_base, 0, sizeof(s->dma_base));
    memset(s->desc, 0, sizeof(s->desc));
    s->dma_en[0] = s->dma_en[1] = false;
    s->cache_valid[0] = s->cache_valid[1] = false;
    s->cache_pos[0] = s->cache_pos[1] = 0;
    s->next_desc[0] = s->next_desc[1] = 0;
    s->field_count[0] = s->field_count[1] = 0;
    if (s->field_timer) {
        timer_del(s->field_timer);
    }
    qemu_set_irq(s->irq, 0);
    memset(s->i2c_dec, 0, sizeof(s->i2c_dec));
    memset(s->i2c_alt, 0, sizeof(s->i2c_alt));
    s->i2c_ptr = 0;
    s->i2c_first = 1;
    s->i2c_expect_sub = 0;
    s->i2c_alt_dev = 0;
    /* Version register (base+4): VINO_CHIP_ID in bits [7:4], revision low. */
    s->regs[SGI_VINO_REV_OFFSET + 3] =
        (SGI_VINO_CHIP_ID << 4) | SGI_VINO_REVISION;
}

/* ------------------------------------------------------------------ */
/* Host video source: sgi-video-source interface                       */
/*
 * The GTK "Video" menu and the HMP/QMP video_attach commands drive this
 * generically.  QEMU carries no decoder: attaching spawns the shared helper
 * wrapper (sgi-irix-re/o2helpers/vin_attach.sh, unchanged) which decodes the
 * source and streams MVPF frames into the video-in chardev; detaching kills
 * it.  The emulated capture is unchanged -- it copies whatever frame the
 * chardev last delivered, so an attach shows up on the next field.
 */

static void sgi_vino_helper_exit(GPid pid, gint status, gpointer opaque)
{
    SGIVinoState *s = opaque;

    s->helper_pid = 0;
    s->helper_watch = 0;
    g_spawn_close_pid(pid);
}

static void sgi_vino_video_detach(SGIVideoSource *src, const char *input)
{
    SGIVinoState *s = SGI_VINO(src);
    int i;

    if (!sgi_vino_input_is(input)) {
        return;
    }
    if (s->helper_watch) {
        g_source_remove(s->helper_watch);
        s->helper_watch = 0;
    }
    if (s->helper_pid > 0) {
        kill(s->helper_pid, SIGTERM);
        /* Reap synchronously so detach is complete when it returns. */
        for (i = 0; i < 50; i++) {
            if (waitpid(s->helper_pid, NULL, WNOHANG) == s->helper_pid) {
                g_spawn_close_pid(s->helper_pid);
                s->helper_pid = 0;
                goto done;
            }
            g_usleep(20000);
        }
        kill(s->helper_pid, SIGKILL);
        waitpid(s->helper_pid, NULL, 0);
        g_spawn_close_pid(s->helper_pid);
        s->helper_pid = 0;
    }
done:
    g_free(s->helper_source);
    s->helper_source = NULL;
    s->helper_is_url = false;
    s->frame_len = 0;   /* fall back to the internal test pattern */
}

static bool sgi_vino_video_attach(SGIVideoSource *src, const char *input,
                                  const char *source, bool is_url,
                                  Error **errp)
{
    SGIVinoState *s = SGI_VINO(src);
    char *argv[8];
    GError *err = NULL;

    if (!sgi_vino_input_is(input)) {
        error_setg(errp, "unknown video input '%s' (use vino)",
                   input ? input : "");
        return false;
    }
    if (!s->video_helper) {
        error_setg(errp, "no helper configured for vino: pass "
                   "-global sgi-vino.video-helper=<wrapper>");
        return false;
    }
    if (!s->video_in_path) {
        error_setg(errp, "no video-in path configured for vino: pass "
                   "-global sgi-vino.video-in-path=<socket>");
        return false;
    }
    if (!source || !*source) {
        error_setg(errp, "empty video source");
        return false;
    }

    sgi_vino_video_detach(src, input);

    argv[0] = s->video_helper;
    argv[1] = (char *)"--socket";
    argv[2] = s->video_in_path;
    argv[3] = (char *)"--source";
    argv[4] = (char *)source;
    argv[5] = is_url ? (char *)"--url" : NULL;
    argv[6] = NULL;

    /* Keep the helper's stderr: its preflight prints a clear one-line reason
     * when ffmpeg is missing or the source cannot be opened. */
    if (!g_spawn_async(NULL, argv, NULL,
                       G_SPAWN_DO_NOT_REAP_CHILD | G_SPAWN_STDOUT_TO_DEV_NULL,
                       NULL, NULL, &s->helper_pid, &err)) {
        error_setg(errp, "cannot launch video helper '%s': %s",
                   s->video_helper, err->message);
        g_error_free(err);
        return false;
    }
    s->helper_watch = g_child_watch_add(s->helper_pid, sgi_vino_helper_exit, s);
    g_free(s->helper_source);
    s->helper_source = g_strdup(source);
    s->helper_is_url = is_url;
    return true;
}

static bool sgi_vino_video_is_attached(SGIVideoSource *src, const char *input)
{
    SGIVinoState *s = SGI_VINO(src);

    return sgi_vino_input_is(input) && s->helper_pid > 0;
}

static const char *sgi_vino_video_describe(SGIVideoSource *src,
                                           const char *input)
{
    SGIVinoState *s = SGI_VINO(src);

    return sgi_vino_input_is(input) ? s->helper_source : NULL;
}

static void sgi_vino_realize(DeviceState *dev, Error **errp)
{
    SGIVinoState *s = SGI_VINO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sgi_vino_ops, s,
                          "sgi-vino", SGI_VINO_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->field_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_vino_field_tick, s);

    s->rx_buf = g_malloc(SGI_VINO_FRAME_MAX);
    s->frame_buf = g_malloc(SGI_VINO_FRAME_MAX);
    qemu_chr_fe_set_handlers(&s->video_in, sgi_vino_video_can_receive,
                             sgi_vino_video_receive, NULL, NULL, s, NULL, true);
}

static const Property sgi_vino_properties[] = {
    DEFINE_PROP_BOOL("present", SGIVinoState, present, false),
    DEFINE_PROP_CHR("video-in", SGIVinoState, video_in),
    DEFINE_PROP_STRING("video-in-path", SGIVinoState, video_in_path),
    DEFINE_PROP_STRING("video-helper", SGIVinoState, video_helper),
};

static void sgi_vino_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SGIVideoSourceClass *vsc = SGI_VIDEO_SOURCE_CLASS(klass);

    dc->realize = sgi_vino_realize;
    device_class_set_legacy_reset(dc, sgi_vino_reset);
    device_class_set_props(dc, sgi_vino_properties);

    vsc->attach = sgi_vino_video_attach;
    vsc->detach = sgi_vino_video_detach;
    vsc->is_attached = sgi_vino_video_is_attached;
    vsc->describe = sgi_vino_video_describe;
}

static const TypeInfo sgi_vino_types[] = {
    {
        .name          = TYPE_SGI_VINO,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(SGIVinoState),
        .class_init    = sgi_vino_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { TYPE_SGI_VIDEO_SOURCE },
            { }
        },
    },
};

DEFINE_TYPES(sgi_vino_types)
