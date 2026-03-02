#ifndef SGI_GLACCEL_H
#define SGI_GLACCEL_H

#include "exec/cpu-common.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_SGI_GLACCEL "sgi-glaccel"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGLAccelState, SGI_GLACCEL)

#define SGI_GLACCEL_BASE_ADDR 0x1f480300
#define SGI_GLACCEL_IRQ 7

/* Register Offsets (all 4-byte aligned, big-endian) */
#define SGI_GLACCEL_STATUS   0x00
#define SGI_GLACCEL_WIDTH    0x04
#define SGI_GLACCEL_HEIGHT   0x08
#define SGI_GLACCEL_CMD_BASE 0x0C
#define SGI_GLACCEL_CMD_LEN  0x10
#define SGI_GLACCEL_FB_BASE  0x14
#define SGI_GLACCEL_EXEC     0x18
#define SGI_GLACCEL_FORMAT   0x1C   /* Pixel format: 0=RGBA8888, 1=RGB565 */
#define SGI_GLACCEL_STRIDE   0x20   /* Bytes per scanline (0 = width*bpp) */

/* Execution Commands */
#define GLACCEL_CMD_RESET   (1 << 0)
#define GLACCEL_CMD_PROCESS (1 << 1)

/* Pixel formats */
#define GLACCEL_FMT_RGBA8888 0
#define GLACCEL_FMT_RGB565   1

/* Status bits */
#define GLACCEL_STATUS_DONE  (1 << 0)

struct SGIGLAccelState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    QemuConsole *con;

    /* Registers */
    uint32_t status;
    uint32_t width;
    uint32_t height;
    uint32_t cmd_base;
    uint32_t cmd_len;
    uint32_t fb_base;
    uint32_t format;
    uint32_t stride;

    /* Internal state */
    bool invalidate;
};

#endif /* SGI_GLACCEL_H */
