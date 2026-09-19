/*
 * SGI IP27 BaseIO board model (minimal, PROM-driven).
 *
 * See header.  Started as the widget-identification probe target so the PROM's
 * own discovery (FUN_bfc37c68 -> FUN_bfc37a94) finds a Bridge; the register set
 * grows from observed accesses.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_baseio.h"
#include "qemu/log.h"

static uint64_t sgi_baseio_read(void *opaque, hwaddr off, unsigned size) {
  /*
   * XIO widget identification, low word (offset 4): bits [27:12] are the
   * widget part number; 0xc002 identifies a Bridge.
   */
  if (off == 4) {
    return (uint64_t)(SGI_BASEIO_WIDGET_PART << 12);
  }
  /*
   * Bridge configuration transfer register (offset 0xb4).  The PROM writes a
   * command (observed 0x82104) then polls bit1 (done) and reads bit0 (data).
   * Reporting "done, data 0" reads back an all-zero configuration string,
   * which the PROM's parser treats as the default board (KLTYPE_BASEIO).
   */
  if (off == 0xb4) {
    return 0x2;
  }
  qemu_log_mask(LOG_UNIMP,
                "sgi-baseio: unimplemented read @0x%" HWADDR_PRIx
                " (size %u)\n",
                off, size);
  return 0;
}

static void sgi_baseio_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size) {
  if (off == 0xb4) {
    return; /* configuration command; response is modelled on read */
  }
  qemu_log_mask(LOG_UNIMP,
                "sgi-baseio: unimplemented write @0x%" HWADDR_PRIx
                " = 0x%" PRIx64 " (size %u)\n",
                off, val, size);
}

static const MemoryRegionOps sgi_baseio_ops = {
    .read = sgi_baseio_read,
    .write = sgi_baseio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

static void sgi_baseio_realize(DeviceState *dev, Error **errp) {
  SGIBaseIOState *s = SGI_BASEIO(dev);

  memory_region_init_io(&s->iomem, OBJECT(s), &sgi_baseio_ops, s, "sgi-baseio",
                        SGI_BASEIO_WINDOW_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property sgi_baseio_properties[] = {
    DEFINE_PROP_UINT32("nasid", SGIBaseIOState, nasid, 0),
    DEFINE_PROP_UINT32("widget", SGIBaseIOState, widget, 0),
};

static void sgi_baseio_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_baseio_realize;
  device_class_set_props(dc, sgi_baseio_properties);
}

static const TypeInfo sgi_baseio_info = {
    .name = TYPE_SGI_BASEIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIBaseIOState),
    .class_init = sgi_baseio_class_init,
};

static void sgi_baseio_register_types(void) {
  type_register_static(&sgi_baseio_info);
}

type_init(sgi_baseio_register_types)
