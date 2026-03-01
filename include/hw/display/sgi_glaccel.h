#ifndef SGI_GLACCEL_H
#define SGI_GLACCEL_H

#include "exec/cpu-common.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_SGI_GLACCEL "sgi-glaccel"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGLAccelState, SGI_GLACCEL)

#define SGI_GLACCEL_BASE_ADDR 0x1f480300
#define SGI_GLACCEL_IRQ 7

/* Register Offsets */
#define SGI_GLACCEL_STATUS 0x00
#define SGI_GLACCEL_WIDTH 0x04
#define SGI_GLACCEL_HEIGHT 0x08
#define SGI_GLACCEL_CMD_BASE 0x0C
#define SGI_GLACCEL_CMD_LEN 0x10
#define SGI_GLACCEL_FB_BASE 0x14
#define SGI_GLACCEL_EXEC 0x18

/* Execution Commands */
#define GLACCEL_CMD_RESET (1 << 0)
#define GLACCEL_CMD_PROCESS (1 << 1)

struct SGIGLAccelState {
  SysBusDevice parent_obj;

  MemoryRegion mmio;
  qemu_irq irq;
  QemuConsole *con;
  QEMUGLContext gl_ctx;

  uint32_t status;
  uint32_t width;
  uint32_t height;
  uint32_t cmd_base;
  uint32_t cmd_len;
  uint32_t fb_base;
};

#endif /* SGI_GLACCEL_H */
