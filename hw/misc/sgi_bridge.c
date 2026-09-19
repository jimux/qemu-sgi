/*
 * SGI BRIDGE (PCI/XTalk Bridge) emulation
 *
 * BRIDGE is the peripheral controller in the SGI Octane (IP30). It handles:
 * - PCI configuration space access
 * - XTLink (XTalk) interface for graphics
 * - BaseIO slot control
 * - Power button and ACFail detection
 *
 * Physical base: 0x1F000000
 *
 * The IOC3 chip sits behind BRIDGE at the devio window (BRIDGE+0x600000 =
 * physical 0x1F600000). The IOC3 has a SuperIO block with two 16550-compatible
 * UARTs. The PROM and OS use UART A as the serial console.
 *
 * IOC3 UART register layout:
 *   Registers are byte-accessible at stride 1, but byte-reversed within
 *   32-bit words (big-endian MIPS byte addressing). The standard 16550
 *   register mapping is: std_reg = physical_offset ^ 3.
 *
 *   Physical address         16550 register
 *   IOC3_BASE + 0x20178      LCR   (reg 3)
 *   IOC3_BASE + 0x20179      IIR/FCR (reg 2)
 *   IOC3_BASE + 0x2017A      IER/DLM (reg 1)
 *   IOC3_BASE + 0x2017B      RBR/THR/DLL (reg 0)
 *   IOC3_BASE + 0x2017C      SCR   (reg 7)
 *   IOC3_BASE + 0x2017D      MSR   (reg 6)
 *   IOC3_BASE + 0x2017E      LSR   (reg 5)
 *   IOC3_BASE + 0x2017F      MCR   (reg 4)
 *
 * Reference: IRIX sys/PCI/ioc3.h, Linux arch/mips/sgi-ip30/ip30-console.c
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_bridge.h"
#include "hw/char/serial.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "chardev/char.h"
#include "qapi/error.h"
#include "qemu/log.h"

/*
 * BRIDGE covers widget 0xF: physical 0x1F000000-0x1FBFFFFF (12MB).
 *   +0x000000: BRIDGE control registers
 *   +0x022000: IOC3 PCI config space
 *   +0x600000: IOC3 devio window (serial, kbd, mouse, ethernet)
 *   +0xC00000: PROM flash (mapped separately as ROM)
 */
#define BRIDGE_REG_SIZE 0xC00000

/*
 * IOC3 UART A (SuperIO/legacy mode):
 *   Base within BRIDGE: 0x620178 (= IOC3_devio 0x600000 + SIO_UA 0x20178)
 *   8 bytes, stride 1, byte-reversed within 32-bit words.
 */
#define IOC3_UART_A_OFFSET  0x620178

/*
 * IOC3 UART custom MemoryRegion ops.
 *
 * Physical byte offset → standard 16550 register: std_reg = offset ^ 3
 * This accounts for big-endian byte ordering within 32-bit words on MIPS.
 */
/*
 * IOC3 SuperIO UART in 16550-compatibility mode is byte-spaced: the 16550
 * register index equals the byte offset (RBR/THR=0, IER=1, IIR/FCR=2, LCR=3,
 * MCR=4, LSR=5, MSR=6, SCR=7). This matches the IP27 BaseIO model, which was
 * verified against the IRIX ARCS ioc3uart.c init and console path; the older
 * `offset ^ 3` byte-reversal was an unverified assumption and dropped the
 * console entirely.
 */
static uint64_t ioc3_uart_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIBRIDGEState *s = opaque;
    return serial_io_ops.read(&s->ioc3_uart, addr, size);
}

static void ioc3_uart_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    SGIBRIDGEState *s = opaque;
    serial_io_ops.write(&s->ioc3_uart, addr, value, size);
}

static const MemoryRegionOps ioc3_uart_ops = {
    .read = ioc3_uart_read,
    .write = ioc3_uart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static uint64_t sgi_bridge_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIBRIDGEState *s = opaque;
    uint64_t val = 0;

    switch (offset) {
    /*
     * BRIDGE local registers: 0x000000-0x00FFFF
     *
     * offset 0x000: widget config (w_id etc.)
     * offset 0x104: b_int_status (Interrupt Status register).
     *   The early POST bus-error handler at PROM 0xbfc00d40 reads this and,
     *   if bit 6 (0x40) is SET, jumps straight into the fault-LED spin at
     *   0xbfc178f4. Bit 6 must therefore read CLEAR on a healthy machine; the
     *   earlier stub returned 0x40 and hung the PROM there before any console
     *   output. All status bits read clear unless a device raises one.
     *
     * The PROM's pon_bridge POST walks control registers (e.g. EVEN_RESP at
     * 0x284) with write/read tests, so the general register file must retain
     * writes. regs[] is 0x1000 words, covering the whole 0x0000-0x3FFF span.
     */
    case 0x0104:
        val = s->regs[offset >> 2] & ~0x40ULL;
        break;

    case 0x0000 ... 0x0103:
    case 0x0105 ... 0x2FFF:
        val = s->regs[offset >> 2];
        break;

    case 0x600000 ... 0x61FFFF:
        if (offset == 0x600028) {
            /*
             * IOC3 SIO_CR (SuperIO Control Register, IOC3 offset 0x028).
             * Bit 22 = SIO_CR_ARB_DIAG_IDLE (0x00400000).
             * The IRIX kernel polls this bit waiting for the SuperIO bus
             * arbiter to become idle before using the UART.  Return it
             * always set so the kernel does not spin here.
             */
            val = 0x00400000; /* SIO_CR_ARB_DIAG_IDLE */
        } else if (offset == 0x600030) {
            /* IOC3 MCR: 1-wire done bit set (nothing pending). */
            val = 0x2;
        } else {
            val = s->ioc3_regs[(offset - 0x600000) >> 2];
        }
        break;

    /*
     * IOC3 device window extended range: 0x620000-0x6FFFFF.
     * SuperIO index/data pair; the UART subregion at 0x620178 takes
     * priority over this range via its own subregion.
     */
    case 0x620000 ... 0x6FFFFF:
        if (offset == 0x6C0000) {
            val = s->sio_regs[s->sio_index];
        } else {
            val = 0;
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "BRIDGE: unimplemented read at offset 0x%05"
                      HWADDR_PRIx "\n", offset);
        break;
    }

    return val;
}

static void sgi_bridge_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    SGIBRIDGEState *s = opaque;

    switch (offset) {
    case 0x0104:
        /* Interrupt status is read-only. */
        break;

    case 0x0000 ... 0x0103:
    case 0x0105 ... 0x2FFF:
        /* General register file (POST write/read tests land here). */
        s->regs[offset >> 2] = val;
        break;

    case 0x600000 ... 0x61FFFF:
        if (offset != 0x600028) {
            /* SIO_CR bit 22 is a live status bit; store everything else. */
            s->ioc3_regs[(offset - 0x600000) >> 2] = val;
        }
        break;

    /*
     * IOC3 device window extended range: 0x620000-0x6FFFFF.
     * SuperIO index/data pair at BRIDGE+0x6A0000/+0x6C0000 (a PC-style
     * bank of 8-bit SuperIO registers). Backed by storage so the PROM's
     * init sequence retains its writes. The IOC3 UART at 0x620178 is
     * handled by its own subregion (ioc3_uart_mr), which takes priority.
     */
    case 0x620000 ... 0x6FFFFF:
        if (offset == 0x6A0000) {
            s->sio_index = val & 0xff;
        } else if (offset == 0x6C0000) {
            s->sio_regs[s->sio_index] = val & 0xff;
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "BRIDGE: unimplemented write at offset 0x%05"
                      HWADDR_PRIx "\n", offset);
        break;
    }
}

static const MemoryRegionOps sgi_bridge_ops = {
    .read = sgi_bridge_read,
    .write = sgi_bridge_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        /*
         * Allow byte-sized accesses (min=1).  The PROM uses sb/lb for
         * SuperIO index/data register pairs in the IOC3 device window
         * (BRIDGE+0x6A0000 and 0x6C0000).  With min_access_size=4, QEMU's
         * memory_region_access_valid() returns MEMTX_ERROR for any sb/lb,
         * causing an infinite DBE exception loop at the faulting instruction.
         *
         * Control registers (0x0000-0x2FFF) still only respond meaningfully
         * to 4-byte accesses; byte accesses to them just return 0 / are
         * silently ignored by the handler.
         */
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_bridge_reset(DeviceState *dev)
{
    SGIBRIDGEState *s = SGI_BRIDGE(dev);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ioc3_regs, 0, sizeof(s->ioc3_regs));
    memset(s->sio_regs, 0, sizeof(s->sio_regs));
    s->sio_index = 0;
}

static void sgi_bridge_realize(DeviceState *dev, Error **errp)
{
    SGIBRIDGEState *s = SGI_BRIDGE(dev);
    Chardev *chr;

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_bridge_ops, s,
                          "sgi-bridge", BRIDGE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /*
     * IOC3 UART A (serial console).
     *
     * Connect to the "ser0" socket chardev if it exists (created by the MCP
     * launch infrastructure), otherwise fall back to "serial0" (a plain
     * -serial stdio/file backend). Without this fallback the UART has no
     * backend and the PROM console is silently discarded.
     */
    chr = qemu_chr_find("ser0");
    if (!chr) {
        chr = qemu_chr_find("serial0");
    }
    if (chr) {
        qdev_prop_set_chr(DEVICE(&s->ioc3_uart), "chardev", chr);
    }

    if (!qdev_realize(DEVICE(&s->ioc3_uart), NULL, errp)) {
        return;
    }

    /*
     * Map the IOC3 UART at BRIDGE offset 0x620178.
     * This subregion takes priority over the parent sgi_bridge_ops for
     * that address range, routing byte accesses to our custom handler.
     */
    memory_region_init_io(&s->ioc3_uart_mr, OBJECT(dev), &ioc3_uart_ops, s,
                          "ioc3-uart", 8);
    memory_region_add_subregion(&s->iomem, IOC3_UART_A_OFFSET,
                                &s->ioc3_uart_mr);
}

static void sgi_bridge_instance_init(Object *obj)
{
    SGIBRIDGEState *s = SGI_BRIDGE(obj);
    object_initialize_child(obj, "ioc3-uart", &s->ioc3_uart, TYPE_SERIAL);
}

static void sgi_bridge_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_bridge_realize;
    device_class_set_legacy_reset(dc, sgi_bridge_reset);
}

static const TypeInfo sgi_bridge_info = {
    .name = TYPE_SGI_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIBRIDGEState),
    .instance_init = sgi_bridge_instance_init,
    .class_init = sgi_bridge_class_init,
};

static void sgi_bridge_register_types(void)
{
    type_register_static(&sgi_bridge_info);
}

type_init(sgi_bridge_register_types)
