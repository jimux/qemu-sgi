/*
 * SGI IOC2-style 8042 PS/2 keyboard/mouse controller (standalone)
 *
 * A standalone sysbus version of the 8042 controller embedded in the
 * HPC3/IOC2 (see sgi_hpc3.c), for machines that have no HPC3 but run
 * a kernel whose pckm driver probes the IOC2 keyboard registers at
 * fixed physical addresses (data 0x1FBD9843, status/cmd 0x1FBD9847).
 *
 * Used by the IP54 paravirtual machine so the stock IRIX pckm driver
 * finds a working controller.  Data delivery on IP54 is polled by the
 * guest (pvuart_cn poll shim), so the IRQ output is optional and may
 * be left unconnected.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_IOC2_KBD_H
#define HW_MISC_SGI_IOC2_KBD_H

#include "hw/core/sysbus.h"
#include "hw/input/ps2.h"
#include "hw/misc/sgi_hpc3.h"   /* SGIPs2KbdState / TYPE_SGI_PS2_KBD */
#include "qom/object.h"

#define TYPE_SGI_IOC2_KBD "sgi-ioc2-kbd"
OBJECT_DECLARE_SIMPLE_TYPE(SGIIOC2KbdState, SGI_IOC2_KBD)

struct SGIIOC2KbdState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;          /* 8 bytes: data word at +0, status/cmd at +4 */

    SGIPs2KbdState ps2kbd;       /* PS/2 keyboard (SGI realtime-typematic subtype) */
    PS2MouseState ps2mouse;      /* PS/2 mouse */

    uint8_t kbd_cmd;             /* Last 8042 controller command written */
    uint8_t kbd_cmd_byte;        /* 8042 controller command byte (via 0x60) */
    bool kbd_irq_level;          /* Current keyboard IRQ state */
    bool mouse_irq_level;        /* Current mouse IRQ state */
    uint8_t kbd_pending_source;  /* 0=keyboard, 1=mouse data pending */

    qemu_irq irq;                /* Combined kbd/mouse IRQ (optional) */
};

#endif /* HW_MISC_SGI_IOC2_KBD_H */
