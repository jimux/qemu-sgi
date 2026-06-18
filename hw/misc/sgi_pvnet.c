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

/*
 * TX bottom half — runs outside the MMIO write handler so that
 * qemu_send_packet() / SLIRP can safely call sgi_pvnet_receive()
 * without reentering the MMIO dispatch path.  TX_DONE is NOT set
 * here: the CMD handler already set it after copying the frame, so
 * the guest's TX-completion spin never waits on this BH.
 */
static void sgi_pvnet_tx_bh(void *opaque) {
  SGIPVNetState *s = opaque;

  while (s->tx_ring_tail != s->tx_ring_head) {
    unsigned slot = s->tx_ring_tail % PVNET_TX_RING_SLOTS;
    qemu_send_packet(qemu_get_queue(s->nic), s->tx_ring[slot],
                     s->tx_ring_len[slot]);
    s->tx_ring_tail++;
  }
}

/*
 * Copy the frame out of guest memory and complete TX immediately.
 * Called from the CMD MMIO write handler; the copy makes it safe to
 * signal TX_DONE before the BH has actually sent the packet.  If the
 * ring is full (BH starved), the frame is dropped — acceptable
 * Ethernet semantics, and the guest can't tell.
 */
static void sgi_pvnet_tx_start(SGIPVNetState *s) {
  if (s->tx_len == 0 || s->tx_len > PVNET_TX_SLOT_SIZE) {
    return;
  }

  if (s->tx_ring_head - s->tx_ring_tail < PVNET_TX_RING_SLOTS) {
    unsigned slot = s->tx_ring_head % PVNET_TX_RING_SLOTS;
    MemTxResult r = dma_memory_read(&address_space_memory, s->tx_base,
                                    s->tx_ring[slot], s->tx_len,
                                    MEMTXATTRS_UNSPECIFIED);
    if (r != MEMTX_OK) {
      qemu_log_mask(LOG_GUEST_ERROR,
                    "pvnet: TX DMA read failed at 0x%" PRIx64 "\n",
                    s->tx_base);
    } else {
      s->tx_ring_len[slot] = s->tx_len;
      s->tx_ring_head++;
      qemu_bh_schedule(s->tx_bh);
    }
  } else {
    qemu_log_mask(LOG_GUEST_ERROR, "pvnet: TX ring full, frame dropped\n");
  }

  s->intr_status |= PVNET_INTR_TX_DONE;
  sgi_pvnet_update_irq(s);
}

static uint64_t sgi_pvnet_read(void *opaque, hwaddr addr, unsigned size) {
  SGIPVNetState *s = opaque;
  addr &= ~7ULL;

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
           (uint64_t)s->conf.macaddr.a[4] << 8 |
           (uint64_t)s->conf.macaddr.a[5];
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
      s->tx_ring_head = s->tx_ring_tail = 0;
      sgi_pvnet_update_irq(s);
    }
    if (val & PVNET_CMD_TX_START) {
      sgi_pvnet_tx_start(s);
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
    if (val > 0) {
      /* RX buffer re-armed — deliver any packets queued while
       * can_receive() returned false (rx_len was 0). */
      qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
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

static bool sgi_pvnet_can_receive(NetClientState *nc) {
  SGIPVNetState *s = qemu_get_nic_opaque(nc);
  return s->rx_len > 0;
}

static ssize_t sgi_pvnet_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size) {
  SGIPVNetState *s = qemu_get_nic_opaque(nc);

  if (s->rx_len == 0) {
    return 0;  /* buffer not armed — queue for retry via flush */
  }
  if ((uint64_t)size > s->rx_len) {
    return size;  /* too large — drop */
  }

  MemTxResult r = dma_memory_write(&address_space_memory, s->rx_base, buf,
                                    size, MEMTXATTRS_UNSPECIFIED);
  if (r != MEMTX_OK) {
    qemu_log_mask(LOG_GUEST_ERROR,
                  "pvnet: RX DMA write failed at 0x%" PRIx64 "\n", s->rx_base);
    return size;
  }

  s->rx_actual = size;
  s->intr_status |= PVNET_INTR_RX_DONE;
  sgi_pvnet_update_irq(s);

  s->rx_len = 0;
  return size;
}

static NetClientInfo net_sgi_pvnet_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_pvnet_can_receive,
    .receive = sgi_pvnet_receive,
};

static void sgi_pvnet_realize(DeviceState *dev, Error **errp) {
  SGIPVNetState *s = SGI_PVNET(dev);

  memory_region_init_io(&s->mmio, OBJECT(s), &sgi_pvnet_ops, s, "sgi-pvnet",
                        SGI_PVNET_MMIO_SIZE);
  sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
  sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

  s->tx_bh = qemu_bh_new(sgi_pvnet_tx_bh, s);

  qemu_macaddr_default_if_unset(&s->conf.macaddr);
  s->nic = qemu_new_nic(&net_sgi_pvnet_info, &s->conf,
                        object_get_typename(OBJECT(dev)), dev->id,
                        &s->reentrancy_guard, s);
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
  s->tx_ring_head = s->tx_ring_tail = 0;
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
