/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/misc/sgi_pvnet.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define SGI_PVNET_MMIO_SIZE 0x100

static void sgi_pvnet_update_irq(SGIPVNetState *s) {
  if (s->intr_status & s->intr_mask) {
    qemu_irq_raise(s->irq);
  } else {
    qemu_irq_lower(s->irq);
  }
}

static uint64_t sgi_pvnet_read(void *opaque, hwaddr addr, unsigned size) {
  SGIPVNetState *s = opaque;
  addr &= ~7ULL; /* 64-bit aligned offsets */

  switch (addr) {
  case SGI_PVNET_STATUS:
    return s->status;
  case SGI_PVNET_INTR_STATUS:
    return s->intr_status;
  case SGI_PVNET_INTR_MASK:
    return s->intr_mask;
  case SGI_PVNET_TX_BASE:
    return s->tx_base;
  case SGI_PVNET_TX_LEN:
    return s->tx_len;
  case SGI_PVNET_RX_BASE:
    return s->rx_base;
  case SGI_PVNET_RX_LEN:
    return s->rx_len;
  case SGI_PVNET_MAC_HI:
    return (uint64_t)s->conf.macaddr.a[0] << 8 | s->conf.macaddr.a[1];
  case SGI_PVNET_MAC_LO:
    return (uint64_t)s->conf.macaddr.a[2] << 24 |
           (uint64_t)s->conf.macaddr.a[3] << 16 |
           (uint64_t)s->conf.macaddr.a[4] << 8 | (uint64_t)s->conf.macaddr.a[5];
  case SGI_PVNET_RX_ACTUAL:
    return s->rx_actual;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    return 0;
  }
}

static void sgi_pvnet_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size) {
  SGIPVNetState *s = opaque;
  addr &= ~7ULL;

  switch (addr) {
  case SGI_PVNET_CMD:
    if (val & PVNET_CMD_RESET) {
      s->intr_status = 0;
      s->status = 0;
      sgi_pvnet_update_irq(s);
    }
    if (val & PVNET_CMD_TX_START) {
      if (s->tx_len > 0) {
        uint8_t *buf = g_malloc(s->tx_len);
        dma_memory_read(&address_space_memory, s->tx_base, buf, s->tx_len,
                        MEMTXATTRS_UNSPECIFIED);
        qemu_send_packet(qemu_get_queue(s->nic), buf, s->tx_len);
        g_free(buf);
        s->intr_status |= PVNET_INTR_TX_DONE;
        sgi_pvnet_update_irq(s);
      }
    }
    break;
  case SGI_PVNET_INTR_STATUS:
    s->intr_status &= ~val; /* W1C */
    sgi_pvnet_update_irq(s);
    break;
  case SGI_PVNET_INTR_MASK:
    s->intr_mask = val;
    sgi_pvnet_update_irq(s);
    break;
  case SGI_PVNET_TX_BASE:
    s->tx_base = val;
    break;
  case SGI_PVNET_TX_LEN:
    s->tx_len = val;
    break;
  case SGI_PVNET_RX_BASE:
    s->rx_base = val;
    break;
  case SGI_PVNET_RX_LEN:
    s->rx_len = val;
    break;
  default:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad register offset 0x%" HWADDR_PRIx "\n", __func__,
                  addr);
    break;
  }
}

static const MemoryRegionOps sgi_pvnet_ops = {
    .read = sgi_pvnet_read,
    .write = sgi_pvnet_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 4,
            .max_access_size = 8,
        },
};

static ssize_t sgi_pvnet_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size) {
  SGIPVNetState *s = qemu_get_nic_opaque(nc);

  /* If no RX buffer, drop packet */
  if (s->rx_len < size) {
    return size;
  }

  dma_memory_write(&address_space_memory, s->rx_base, buf, size,
                   MEMTXATTRS_UNSPECIFIED);
  s->rx_actual = size;
  s->intr_status |= PVNET_INTR_RX_DONE;
  sgi_pvnet_update_irq(s);

  /* Consume the buffer, expecting driver to update RX_BASE and RX_LEN */
  s->rx_len = 0;
  return size;
}

static NetClientInfo net_sgi_pvnet_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = sgi_pvnet_receive,
};

static void sgi_pvnet_realize(DeviceState *dev, Error **errp) {
  SGIPVNetState *s = SGI_PVNET(dev);

  memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvnet_ops, s, "sgi-pvnet",
                        SGI_PVNET_MMIO_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
  sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

  qemu_macaddr_default_if_unset(&s->conf.macaddr);
  s->nic = qemu_new_nic(&net_sgi_pvnet_info, &s->conf,
                        object_get_typename(OBJECT(dev)), dev->id, NULL, s);
  qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void sgi_pvnet_reset(DeviceState *dev) {
  SGIPVNetState *s = SGI_PVNET(dev);

  s->status = 0;
  s->intr_status = 0;
  s->intr_mask = 0;
  s->tx_base = 0;
  s->tx_len = 0;
  s->rx_base = 0;
  s->rx_len = 0;
  s->rx_actual = 0;
  sgi_pvnet_update_irq(s);
}

static const Property sgi_pvnet_properties[] = {
    DEFINE_NIC_PROPERTIES(SGIPVNetState, conf),
};

static void sgi_pvnet_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = sgi_pvnet_realize;
  device_class_set_legacy_reset(dc, sgi_pvnet_reset);
  device_class_set_props(dc, sgi_pvnet_properties);
}

static const TypeInfo sgi_pvnet_info = {
    .name = TYPE_SGI_PVNET,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIPVNetState),
    .class_init = sgi_pvnet_class_init,
};

static void sgi_pvnet_register_types(void) {
  type_register_static(&sgi_pvnet_info);
}

type_init(sgi_pvnet_register_types)
