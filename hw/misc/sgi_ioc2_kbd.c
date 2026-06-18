/*
 * SGI IOC2-style 8042 PS/2 keyboard/mouse controller (standalone)
 *
 * The 8042 state machine is transplanted from the IOC2 emulation in
 * sgi_hpc3.c so the byte-level behavior seen by the IRIX pckm driver
 * is identical to the (working) Indy path.  Register layout matches
 * IOC2: a data word and a status/command word, each accessed by the
 * guest as the byte at offset +3 within the word.
 *
 *   base + 0x0..0x3  data port      (IRIX KB_REG_60, PA 0x1FBD9843)
 *   base + 0x4..0x7  status/command (IRIX KB_REG_64, PA 0x1FBD9847)
 *
 * The IRQ output reflects (OBF && interrupts enabled in the command
 * byte) like IRQ1/IRQ12 on a PC.  On IP54 it is left unconnected:
 * the guest drains the controller from a callout-context poll.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/sgi_ioc2_kbd.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

/* 8042 controller commands */
#define KBD_CMD_READ_CTRL      0x20  /* Read controller command byte */
#define KBD_CMD_WRITE_CTRL     0x60  /* Write controller command byte */
#define KBD_CMD_DISABLE_MOUSE  0xA7  /* Disable mouse port */
#define KBD_CMD_ENABLE_MOUSE   0xA8  /* Enable mouse port */
#define KBD_CMD_TEST_MOUSE     0xA9  /* Test mouse port */
#define KBD_CMD_SELF_TEST      0xAA  /* Controller self-test */
#define KBD_CMD_IFACE_TEST     0xAB  /* Interface test */
#define KBD_CMD_DISABLE_KBD    0xAD  /* Disable keyboard */
#define KBD_CMD_ENABLE_KBD     0xAE  /* Enable keyboard */
#define KBD_CMD_READ_INPUT     0xC0  /* Read input port */
#define KBD_CMD_READ_OUTPUT    0xD0  /* Read output port */
#define KBD_CMD_WRITE_OUTPUT   0xD1  /* Write output port */
#define KBD_CMD_WRITE_KBD      0xD2  /* Write to keyboard output buffer */
#define KBD_CMD_WRITE_MOUSE    0xD3  /* Write to mouse output buffer */
#define KBD_CMD_WRITE_MOUSE_P  0xD4  /* Write to mouse */

/* 8042 responses (for controller-generated replies only) */
#define KBD_RESP_SELF_TEST_OK  0x55  /* Self-test passed */
#define KBD_RESP_IFACE_OK      0x00  /* Interface test passed */

/* Status register bits */
#define KBD_STAT_OBF           0x01  /* Output buffer full */
#define KBD_STAT_SYS           0x04  /* System flag */
#define KBD_STAT_UNLOCKED      0x10  /* Keyboard unlocked */
#define KBD_STAT_MOUSE_OBF     0x20  /* Mouse output buffer full */

/* Controller command byte bits */
#define KBD_MODE_KBD_INT       0x01  /* Keyboard data generates IRQ */
#define KBD_MODE_MOUSE_INT     0x02  /* Mouse data generates IRQ */
#define KBD_MODE_SYS           0x04  /* System flag */
#define KBD_MODE_NO_KEYLOCK    0x08  /* Override keyboard lock */
#define KBD_MODE_DISABLE_KBD   0x10  /* Disable keyboard interface */
#define KBD_MODE_DISABLE_MOUSE 0x20  /* Disable mouse interface */
#define KBD_MODE_KCC           0x40  /* Scan code conversion */

/* Register word offsets within the 8-byte region */
#define IOC2_KBD_REG_DATA      0x0   /* guest reads/writes byte at +3 */
#define IOC2_KBD_REG_STATUS    0x4   /* guest reads/writes byte at +7 */

/*
 * Queue controller-generated data into the keyboard output path
 * (self-test, interface test responses, command byte readback).
 */
static void sgi_ioc2_kbd_queue_ctrl(SGIIOC2KbdState *s, uint8_t data)
{
    ps2_queue(PS2_DEVICE(&s->ps2kbd), data);
}

/*
 * Build the 8042 status register dynamically from PS/2 queue state.
 *
 * The IRIX pckm interrupt handler checks (status & (SR_MSFULL|SR_OBF)):
 *   0x21 → mouse data pending → read from ps2mouse
 *   0x01 → keyboard data pending → read from ps2kbd
 */
static uint8_t sgi_ioc2_kbd_status(SGIIOC2KbdState *s)
{
    uint8_t status = KBD_STAT_SYS | KBD_STAT_UNLOCKED;

    if (!ps2_queue_empty(PS2_DEVICE(&s->ps2kbd))) {
        status |= KBD_STAT_OBF;
        s->kbd_pending_source = 0;  /* keyboard */
    } else if (!ps2_queue_empty(PS2_DEVICE(&s->ps2mouse))) {
        status |= KBD_STAT_OBF | KBD_STAT_MOUSE_OBF;
        s->kbd_pending_source = 1;  /* mouse */
    }

    return status;
}

/*
 * Combined IRQ: asserted when a device has data pending AND the
 * matching interrupt-enable bit is set in the command byte.  IRIX
 * pckm_reinit_lock() clears the enable bits during polled init with
 * pckm_mutex held; gating on them avoids interrupting into a
 * deadlock if this output is ever wired up.
 */
static void sgi_ioc2_kbd_update_irq(SGIIOC2KbdState *s)
{
    bool kbd_active = s->kbd_irq_level &&
                      (s->kbd_cmd_byte & KBD_MODE_KBD_INT);
    bool mouse_active = s->mouse_irq_level &&
                        (s->kbd_cmd_byte & KBD_MODE_MOUSE_INT);

    qemu_set_irq(s->irq, kbd_active || mouse_active);
}

static void sgi_ioc2_kbd_ps2_kbd_irq(void *opaque, int n, int level)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(opaque);
    s->kbd_irq_level = level;
    sgi_ioc2_kbd_update_irq(s);
}

static void sgi_ioc2_kbd_ps2_mouse_irq(void *opaque, int n, int level)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(opaque);
    s->mouse_irq_level = level;
    sgi_ioc2_kbd_update_irq(s);
}

/*
 * Read the 8042 output buffer: keyboard data takes priority over
 * mouse data, matching standard 8042 behavior.
 */
static uint8_t sgi_ioc2_kbd_read_data(SGIIOC2KbdState *s)
{
    if (!ps2_queue_empty(PS2_DEVICE(&s->ps2kbd))) {
        uint8_t b = ps2_read_data(PS2_DEVICE(&s->ps2kbd));
        if (getenv("SGI_KBD_DEBUG"))
            fprintf(stderr, "[ioc2-kbd] <- KBD read 0x%02x\n", b);
        return b;
    } else if (!ps2_queue_empty(PS2_DEVICE(&s->ps2mouse))) {
        uint8_t b = ps2_read_data(PS2_DEVICE(&s->ps2mouse));
        if (getenv("SGI_KBD_DEBUG"))
            fprintf(stderr, "[ioc2-kbd] <- MOUSE read 0x%02x\n", b);
        return b;
    }
    return 0;
}

/* Process an 8042 controller command (write to the command port) */
static void sgi_ioc2_kbd_command(SGIIOC2KbdState *s, uint8_t cmd)
{
    s->kbd_cmd = cmd;

    /* env-gated mouse-init tracing: shows whether the guest pckm ever talks to the aux/mouse
     * port (0xA7 disable, 0xA8 enable, 0xD4 write-to-mouse). Set SGI_KBD_DEBUG=1 to enable. */
    if (getenv("SGI_KBD_DEBUG") &&
        (cmd == KBD_CMD_DISABLE_MOUSE || cmd == KBD_CMD_ENABLE_MOUSE ||
         cmd == KBD_CMD_TEST_MOUSE || cmd == KBD_CMD_WRITE_MOUSE_P)) {
        fprintf(stderr, "[ioc2-kbd] cmd 0x%02x (%s)\n", cmd,
                cmd == KBD_CMD_DISABLE_MOUSE ? "DISABLE_MOUSE" :
                cmd == KBD_CMD_ENABLE_MOUSE ? "ENABLE_MOUSE" :
                cmd == KBD_CMD_TEST_MOUSE ? "TEST_MOUSE" : "WRITE_MOUSE_P");
    }

    switch (cmd) {
    case 0x00:  /* NOP - some PROMs send this */
        break;

    case KBD_CMD_SELF_TEST:
        sgi_ioc2_kbd_queue_ctrl(s, KBD_RESP_SELF_TEST_OK);
        break;

    case KBD_CMD_IFACE_TEST:
    case KBD_CMD_TEST_MOUSE:
        sgi_ioc2_kbd_queue_ctrl(s, KBD_RESP_IFACE_OK);
        break;

    case KBD_CMD_DISABLE_KBD:
        s->kbd_cmd_byte |= KBD_MODE_DISABLE_KBD;
        break;

    case KBD_CMD_ENABLE_KBD:
        s->kbd_cmd_byte &= ~KBD_MODE_DISABLE_KBD;
        break;

    case KBD_CMD_DISABLE_MOUSE:
        s->kbd_cmd_byte |= KBD_MODE_DISABLE_MOUSE;
        break;

    case KBD_CMD_ENABLE_MOUSE:
        s->kbd_cmd_byte &= ~KBD_MODE_DISABLE_MOUSE;
        break;

    case KBD_CMD_READ_CTRL:
        sgi_ioc2_kbd_queue_ctrl(s, s->kbd_cmd_byte);
        break;

    case KBD_CMD_WRITE_CTRL:
        /* Wait for data write */
        break;

    case KBD_CMD_READ_INPUT:
        sgi_ioc2_kbd_queue_ctrl(s, 0x80);
        break;

    case KBD_CMD_READ_OUTPUT:
        sgi_ioc2_kbd_queue_ctrl(s, 0x02);
        break;

    case KBD_CMD_WRITE_OUTPUT:
    case KBD_CMD_WRITE_KBD:
    case KBD_CMD_WRITE_MOUSE:
    case KBD_CMD_WRITE_MOUSE_P:
        /* These expect a data byte to follow */
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_ioc2_kbd: unimplemented 8042 command 0x%02x\n",
                      cmd);
        break;
    }
}

/*
 * Process a write to the 8042 data port.  Meaning depends on the
 * last controller command (see sgi_hpc3.c for the original).
 */
static void sgi_ioc2_kbd_data_write(SGIIOC2KbdState *s, uint8_t data)
{
    switch (s->kbd_cmd) {
    case KBD_CMD_WRITE_CTRL:
        s->kbd_cmd_byte = data;
        ps2_keyboard_set_translation(&s->ps2kbd.parent_obj,
                                     (s->kbd_cmd_byte & KBD_MODE_KCC) != 0);
        s->kbd_cmd = 0;
        sgi_ioc2_kbd_update_irq(s);  /* fire pending IRQ if now enabled */
        break;

    case KBD_CMD_WRITE_OUTPUT:
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_KBD:
        ps2_queue(PS2_DEVICE(&s->ps2kbd), data);
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_MOUSE:
        ps2_queue(PS2_DEVICE(&s->ps2mouse), data);
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_MOUSE_P:
        if (getenv("SGI_KBD_DEBUG")) {
            fprintf(stderr, "[ioc2-kbd] -> mouse byte 0x%02x%s\n", data,
                    data == 0xf4 ? " (ENABLE data reporting)" :
                    data == 0xff ? " (RESET)" :
                    data == 0xf2 ? " (GET ID)" : "");
        }
        ps2_write_mouse(&s->ps2mouse, data);
        s->kbd_cmd = 0;
        break;

    default:
        /* No pending controller command — send directly to keyboard */
        ps2_write_keyboard(&s->ps2kbd.parent_obj, data);
        break;
    }
}

static uint64_t sgi_ioc2_kbd_mmio_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(opaque);

    switch (addr & ~3) {
    case IOC2_KBD_REG_DATA:
        return sgi_ioc2_kbd_read_data(s);
    case IOC2_KBD_REG_STATUS:
        return sgi_ioc2_kbd_status(s);
    default:
        return 0;
    }
}

static void sgi_ioc2_kbd_mmio_write(void *opaque, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(opaque);

    switch (addr & ~3) {
    case IOC2_KBD_REG_DATA:
        sgi_ioc2_kbd_data_write(s, val & 0xff);
        break;
    case IOC2_KBD_REG_STATUS:
        sgi_ioc2_kbd_command(s, val & 0xff);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps sgi_ioc2_kbd_mmio_ops = {
    .read = sgi_ioc2_kbd_mmio_read,
    .write = sgi_ioc2_kbd_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_ioc2_kbd_reset(DeviceState *dev)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(dev);

    /*
     * Default command byte 0x47 = SYS | NO_KEYLOCK | KCC | KBD_INT,
     * matching the IOC2 reset state in sgi_hpc3.c.  The PS/2 devices
     * are reset by QOM's reset mechanism automatically.
     */
    s->kbd_cmd = 0;
    s->kbd_cmd_byte = KBD_MODE_SYS | KBD_MODE_NO_KEYLOCK
                    | KBD_MODE_KCC | KBD_MODE_KBD_INT;
    s->kbd_irq_level = false;
    s->mouse_irq_level = false;
    s->kbd_pending_source = 0;
}

static void sgi_ioc2_kbd_init(Object *obj)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    object_initialize_child(obj, "ps2kbd", &s->ps2kbd, TYPE_SGI_PS2_KBD);
    object_initialize_child(obj, "ps2mouse", &s->ps2mouse,
                            TYPE_PS2_MOUSE_DEVICE);

    qdev_init_gpio_in_named(DEVICE(s), sgi_ioc2_kbd_ps2_kbd_irq,
                            "ps2-kbd-irq", 1);
    qdev_init_gpio_in_named(DEVICE(s), sgi_ioc2_kbd_ps2_mouse_irq,
                            "ps2-mouse-irq", 1);

    memory_region_init_io(&s->iomem, obj, &sgi_ioc2_kbd_mmio_ops, s,
                          "sgi-ioc2-kbd", 8);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void sgi_ioc2_kbd_realize(DeviceState *dev, Error **errp)
{
    SGIIOC2KbdState *s = SGI_IOC2_KBD(dev);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2kbd), errp)) {
        return;
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2mouse), errp)) {
        return;
    }

    qdev_connect_gpio_out(DEVICE(&s->ps2kbd), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in_named(dev, "ps2-kbd-irq", 0));
    qdev_connect_gpio_out(DEVICE(&s->ps2mouse), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in_named(dev, "ps2-mouse-irq", 0));
}

static const VMStateDescription vmstate_sgi_ioc2_kbd = {
    .name = "sgi-ioc2-kbd",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(kbd_cmd, SGIIOC2KbdState),
        VMSTATE_UINT8(kbd_cmd_byte, SGIIOC2KbdState),
        VMSTATE_BOOL(kbd_irq_level, SGIIOC2KbdState),
        VMSTATE_BOOL(mouse_irq_level, SGIIOC2KbdState),
        VMSTATE_UINT8(kbd_pending_source, SGIIOC2KbdState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_ioc2_kbd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_ioc2_kbd_realize;
    device_class_set_legacy_reset(dc, sgi_ioc2_kbd_reset);
    dc->vmsd = &vmstate_sgi_ioc2_kbd;
}

static const TypeInfo sgi_ioc2_kbd_info = {
    .name          = TYPE_SGI_IOC2_KBD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIIOC2KbdState),
    .instance_init = sgi_ioc2_kbd_init,
    .class_init    = sgi_ioc2_kbd_class_init,
};

static void sgi_ioc2_kbd_register_types(void)
{
    type_register_static(&sgi_ioc2_kbd_info);
}

type_init(sgi_ioc2_kbd_register_types)
