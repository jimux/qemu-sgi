#ifndef SGI_PVNET_H
#define SGI_PVNET_H

#include "exec/cpu-common.h"
#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_SGI_PVNET "sgi-pvnet"
OBJECT_DECLARE_SIMPLE_TYPE(SGIPVNetState, SGI_PVNET)

#define SGI_PVNET_BASE_ADDR 0x1f480200
#define SGI_PVNET_IRQ 6

/* Registers */
#define SGI_PVNET_CMD 0x00
#define SGI_PVNET_STATUS 0x08
#define SGI_PVNET_INTR_STATUS 0x10
#define SGI_PVNET_INTR_MASK 0x18
#define SGI_PVNET_TX_BASE 0x20
#define SGI_PVNET_TX_LEN 0x28
#define SGI_PVNET_RX_BASE 0x30
#define SGI_PVNET_RX_LEN 0x38
#define SGI_PVNET_MAC_HI 0x40
#define SGI_PVNET_MAC_LO 0x48
#define SGI_PVNET_RX_ACTUAL 0x50  /* actual byte count of last received packet */

/* Commands */
#define PVNET_CMD_TX_START (1 << 0)
#define PVNET_CMD_RX_START (1 << 1)
#define PVNET_CMD_RESET (1 << 2)

/* Interrupts */
#define PVNET_INTR_TX_DONE (1 << 0)
#define PVNET_INTR_RX_DONE (1 << 1)

struct SGIPVNetState {
  SysBusDevice parent_obj;

  MemoryRegion mmio;
  NICState *nic;
  NICConf conf;
  qemu_irq irq;

  uint64_t status;
  uint64_t intr_status;
  uint64_t intr_mask;

  uint64_t tx_base;
  uint64_t tx_len;

  uint64_t rx_base;
  uint64_t rx_len;
  uint64_t rx_actual;  /* actual byte count of last received packet */
};

#endif /* SGI_PVNET_H */
