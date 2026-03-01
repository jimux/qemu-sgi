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

#define SGI_GLACCEL_MMIO_SIZE 0x100

static uint64_t sgi_glaccel_read(void *opaque, hwaddr addr, unsigned size) {
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
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    return 0;
  }
}

static void sgi_glaccel_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size) {
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
    break;
  case SGI_GLACCEL_HEIGHT:
    s->height = val;
    break;
  case SGI_GLACCEL_CMD_BASE:
    s->cmd_base = val;
    break;
  case SGI_GLACCEL_CMD_LEN:
    s->cmd_len = val;
    break;
  case SGI_GLACCEL_FB_BASE:
    s->fb_base = val;
    break;
  case SGI_GLACCEL_EXEC:
    if (val & GLACCEL_CMD_RESET) {
      s->status = 0;
      s->width = 0;
      s->height = 0;
      s->cmd_base = 0;
      s->cmd_len = 0;
      s->fb_base = 0;
      qemu_irq_lower(s->irq);
    }
    if (val & GLACCEL_CMD_PROCESS) {
      /*
       * In Phase 7: Software rendering placeholder.
       * Later in Phase 8: Hardware passthrough.
       */
      if (s->cmd_len > 0) {
        uint8_t *cmd_buf = g_malloc(s->cmd_len);
        dma_memory_read(&address_space_memory, s->cmd_base, cmd_buf, s->cmd_len,
                        MEMTXATTRS_UNSPECIFIED);
        if (s->gl_ctx) {
          /* Hardware passthrough using dpy_gl_* */
          dpy_gl_ctx_make_current(s->con, s->gl_ctx);
          /* Parsed OpenGL commands from cmd_buf would be executed natively here
           */

          /* dpy_gl_update(s->con, 0, 0, s->width, s->height); */
        } else {
          /* Software rendering fallback */
        }
        g_free(cmd_buf);

        s->status |= 1; /* Done */
        qemu_irq_raise(s->irq);
      }
    }
    break;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    break;
  }
}

static const MemoryRegionOps sgi_glaccel_ops = {
    .read = sgi_glaccel_read,
    .write = sgi_glaccel_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 4,
        },
};

static const GraphicHwOps sgi_glaccel_hw_ops = {
    .gfx_update = NULL,
};

static void sgi_glaccel_realize(DeviceState *dev, Error **errp) {
  SGIGLAccelState *s = SGI_GLACCEL(dev);

  memory_region_init_io(&s->mmio, OBJECT(s), &sgi_glaccel_ops, s, "sgi-glaccel",
                        SGI_GLACCEL_MMIO_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
  sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

  s->con = graphic_console_init(dev, 0, &sgi_glaccel_hw_ops, s);

  if (console_has_gl(s->con)) {
    QEMUGLParams gl_params = {
        .major_ver = 3,
        .minor_ver = 0,
    };
    s->gl_ctx = dpy_gl_ctx_create(s->con, &gl_params);
  } else {
    qemu_log_mask(LOG_GUEST_ERROR, "%s: Host UI doesn't support OpenGL!\n",
                  __func__);
  }
}

static void sgi_glaccel_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);
  dc->realize = sgi_glaccel_realize;
}

static const TypeInfo sgi_glaccel_info = {
    .name = TYPE_SGI_GLACCEL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGLAccelState),
    .class_init = sgi_glaccel_class_init,
    .instance_init = NULL,
};

static void sgi_glaccel_register_types(void) {
  type_register_static(&sgi_glaccel_info);
}

type_init(sgi_glaccel_register_types)
