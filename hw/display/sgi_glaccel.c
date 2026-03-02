/*
 * SGI Paravirtual GL Accelerator — Dumb Framebuffer Mode
 *
 * This device provides a simple DMA-based framebuffer for the IP30/IP54
 * machine. The guest writes pixel data into a physically contiguous
 * buffer and programs FB_BASE/WIDTH/HEIGHT/FORMAT. The device DMAs
 * the guest framebuffer into the QEMU console surface at 60Hz.
 *
 * A future phase will add OpenGL command passthrough.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/display/sgi_glaccel.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "framebuffer.h"

#define SGI_GLACCEL_MMIO_SIZE 0x100

/*
 * Return bytes per pixel for the current format.
 */
static int glaccel_bpp(SGIGLAccelState *s)
{
    switch (s->format) {
    case GLACCEL_FMT_RGB565:
        return 2;
    case GLACCEL_FMT_RGBA8888:
    default:
        return 4;
    }
}

/*
 * Return the stride (bytes per scanline). If the STRIDE register is 0,
 * compute from width * bpp.
 */
static uint32_t glaccel_stride(SGIGLAccelState *s)
{
    if (s->stride != 0) {
        return s->stride;
    }
    return s->width * glaccel_bpp(s);
}

/*
 * Convert one scanline from guest format to host PIXMAN_x8r8g8b8.
 */
static void glaccel_draw_line_rgba8888(void *opaque, uint8_t *dst,
                                        const uint8_t *src, int width,
                                        int pitch)
{
    /* Guest RGBA8888 (R,G,B,A bytes) → host xRGB (PIXMAN_x8r8g8b8) */
    for (int x = 0; x < width; x++) {
        uint8_t r = src[0];
        uint8_t g = src[1];
        uint8_t b = src[2];
        /* A byte ignored */
        *(uint32_t *)dst = (r << 16) | (g << 8) | b;
        src += 4;
        dst += 4;
    }
}

static void glaccel_draw_line_rgb565(void *opaque, uint8_t *dst,
                                      const uint8_t *src, int width,
                                      int pitch)
{
    /* Guest RGB565 big-endian → host xRGB */
    for (int x = 0; x < width; x++) {
        uint16_t pixel = (src[0] << 8) | src[1];
        uint8_t r = ((pixel >> 11) & 0x1f) << 3;
        uint8_t g = ((pixel >> 5) & 0x3f) << 2;
        uint8_t b = (pixel & 0x1f) << 3;
        *(uint32_t *)dst = (r << 16) | (g << 8) | b;
        src += 2;
        dst += 4;
    }
}

/*
 * gfx_update callback — called at ~60Hz by the QEMU display subsystem.
 * DMA the guest framebuffer and blit to the console surface.
 */
static void sgi_glaccel_update(void *opaque)
{
    SGIGLAccelState *s = opaque;
    DisplaySurface *surface;
    int width, height, src_stride;

    if (s->fb_base == 0 || s->width == 0 || s->height == 0) {
        return;
    }

    width = s->width;
    height = s->height;
    src_stride = glaccel_stride(s);

    surface = qemu_console_surface(s->con);

    /* Resize console if needed */
    if (!surface ||
        surface_width(surface) != width ||
        surface_height(surface) != height) {
        qemu_console_resize(s->con, width, height);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }

    if (!surface) {
        return;
    }

    /*
     * DMA the entire guest framebuffer and convert scanline by scanline.
     * For a dumb framebuffer this is simple and correct. A future
     * optimization could use dirty-page tracking.
     */
    int dst_stride = surface_stride(surface);
    uint8_t *dst = surface_data(surface);
    uint8_t *src_line = g_malloc(src_stride);

    for (int y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        s->fb_base + (uint64_t)y * src_stride,
                        src_line, src_stride, MEMTXATTRS_UNSPECIFIED);

        if (s->format == GLACCEL_FMT_RGB565) {
            glaccel_draw_line_rgb565(s, dst, src_line, width, src_stride);
        } else {
            glaccel_draw_line_rgba8888(s, dst, src_line, width, src_stride);
        }
        dst += dst_stride;
    }

    g_free(src_line);
    dpy_gfx_update_full(s->con);
}

static void sgi_glaccel_invalidate(void *opaque)
{
    SGIGLAccelState *s = opaque;
    s->invalidate = true;
}

static uint64_t sgi_glaccel_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIGLAccelState *s = opaque;

    switch (addr) {
    case SGI_GLACCEL_STATUS:
        return s->status;
    case SGI_GLACCEL_WIDTH:
        return s->width;
    case SGI_GLACCEL_HEIGHT:
        return s->height;
    case SGI_GLACCEL_CMD_BASE:
        return s->cmd_base;
    case SGI_GLACCEL_CMD_LEN:
        return s->cmd_len;
    case SGI_GLACCEL_FB_BASE:
        return s->fb_base;
    case SGI_GLACCEL_FORMAT:
        return s->format;
    case SGI_GLACCEL_STRIDE:
        return s->stride;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }
}

static void sgi_glaccel_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGIGLAccelState *s = opaque;

    switch (addr) {
    case SGI_GLACCEL_STATUS:
        s->status &= ~val; /* W1C */
        if (s->status == 0) {
            qemu_irq_lower(s->irq);
        }
        break;
    case SGI_GLACCEL_WIDTH:
        s->width = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_HEIGHT:
        s->height = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_CMD_BASE:
        s->cmd_base = val;
        break;
    case SGI_GLACCEL_CMD_LEN:
        s->cmd_len = val;
        break;
    case SGI_GLACCEL_FB_BASE:
        s->fb_base = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_FORMAT:
        if (val <= GLACCEL_FMT_RGB565) {
            s->format = val;
            s->invalidate = true;
        }
        break;
    case SGI_GLACCEL_STRIDE:
        s->stride = val;
        break;
    case SGI_GLACCEL_EXEC:
        if (val & GLACCEL_CMD_RESET) {
            s->status = 0;
            s->width = 0;
            s->height = 0;
            s->cmd_base = 0;
            s->cmd_len = 0;
            s->fb_base = 0;
            s->format = GLACCEL_FMT_RGBA8888;
            s->stride = 0;
            s->invalidate = true;
            qemu_irq_lower(s->irq);
        }
        if (val & GLACCEL_CMD_PROCESS) {
            /*
             * Trigger an immediate framebuffer update and signal completion.
             * OpenGL command-list processing is deferred to a future phase.
             */
            if (s->fb_base && s->width && s->height) {
                sgi_glaccel_update(s);
            }
            s->status |= GLACCEL_STATUS_DONE;
            qemu_irq_raise(s->irq);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps sgi_glaccel_ops = {
    .read = sgi_glaccel_read,
    .write = sgi_glaccel_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const GraphicHwOps sgi_glaccel_hw_ops = {
    .gfx_update = sgi_glaccel_update,
    .invalidate = sgi_glaccel_invalidate,
};

static void sgi_glaccel_realize(DeviceState *dev, Error **errp)
{
    SGIGLAccelState *s = SGI_GLACCEL(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_glaccel_ops, s,
                          "sgi-glaccel", SGI_GLACCEL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

    s->con = graphic_console_init(dev, 0, &sgi_glaccel_hw_ops, s);
    s->format = GLACCEL_FMT_RGBA8888;
    s->invalidate = true;
}

static void sgi_glaccel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = sgi_glaccel_realize;
}

static const TypeInfo sgi_glaccel_info = {
    .name = TYPE_SGI_GLACCEL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGLAccelState),
    .class_init = sgi_glaccel_class_init,
};

static void sgi_glaccel_register_types(void)
{
    type_register_static(&sgi_glaccel_info);
}

type_init(sgi_glaccel_register_types)
