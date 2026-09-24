/*
 * SGI IP30 (Octane) BRIDGE serial-flash controller.  See sgi_sflash.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/sgi_sflash.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "system/memory.h"

/* SHARP/Intel compatibility commands (sflash.h). */
#define SFLASH_CMD_READ          0xffff
#define SFLASH_CMD_ID_READ       0x9090
#define SFLASH_CMD_STATUS_READ   0x7070
#define SFLASH_CMD_STATUS_CLEAR  0x5050
#define SFLASH_CMD_WRITE         0x4040
#define SFLASH_CMD_ERASE         0x2020
#define SFLASH_CMD_ERASE_CONFIRM 0xd0d0
#define SFLASH_CMD_ESR_READ      0x7171

#define SFLASH_STATUS_READY      0x80
#define SFLASH_MFG_ID            0x00b0
#define SFLASH_DEV_ID            0x66a8

/* Read `size` bytes (big-endian) from a 16-bit value at halfword `hw`. */
static uint64_t sflash_read16(uint16_t val, hwaddr off, unsigned size)
{
    unsigned sh = 8 * (2 - size - (off & 1));

    return (size >= 2) ? val : ((val >> sh) & 0xff);
}

/* Read `size` bytes (big-endian) from a buffer. */
static uint64_t sflash_read_bytes(const uint8_t *p, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | p[i];
    }
    return v;
}

/* --- Flash command window (base) ------------------------------------- */

static uint64_t sgi_sflash_cmd_read(void *opaque, hwaddr off, unsigned size)
{
    SGISflashState *s = opaque;
    uint16_t hw = (off & ~1ULL) & 0xff;
    uint16_t val;

    if (s->rom) {
        uint8_t *p = memory_region_get_ram_ptr(s->rom) + off;

        /* Commands only shape 16-bit data reads, not code fetches. */
        if (size > 2) {
            return sflash_read_bytes(p, size);
        }
        switch (s->cmd) {
        case SFLASH_CMD_ID_READ:
            val = (hw == 0) ? SFLASH_MFG_ID
                            : (hw == 2) ? SFLASH_DEV_ID : 0xffff;
            return sflash_read16(val, off, size);
        case SFLASH_CMD_STATUS_READ:
        case SFLASH_CMD_ESR_READ:
            /* CSR/GSR/BSR all report "ready". */
            return sflash_read16(SFLASH_STATUS_READY, off, size);
        default:
            return sflash_read_bytes(p, size);
        }
    }
    return 0;
}

static void sgi_sflash_cmd_write(void *opaque, hwaddr off, uint64_t value,
                                 unsigned size)
{
    SGISflashState *s = opaque;
    uint16_t v = value & 0xffff;

    switch (v) {
    case SFLASH_CMD_ID_READ:
    case SFLASH_CMD_STATUS_READ:
    case SFLASH_CMD_ESR_READ:
    case SFLASH_CMD_STATUS_CLEAR:
    case SFLASH_CMD_READ:
        s->cmd = v;
        s->cmd_off = off;
        break;
    default:
        /* Data write to the command window: store into the PROM backing? The
         * code segments are read-only, so ignore. */
        break;
    }
}

/* --- PDS segment (segment 15) ---------------------------------------- */

/* Reload the PDS image from its backing file (or erase it if unbacked). */
static void sgi_sflash_pds_load(SGISflashState *s)
{
    gchar *data = NULL;
    gsize len = 0;

    memset(s->pds, 0xff, sizeof(s->pds));
    if (s->pds_file &&
        g_file_get_contents(s->pds_file, &data, &len, NULL) && len > 0) {
        memcpy(s->pds, data, MIN(len, sizeof(s->pds)));
    }
    g_free(data);
}

/* Persist the PDS image to its backing file, if any. */
static void sgi_sflash_pds_save(SGISflashState *s)
{
    if (s->pds_file) {
        g_file_set_contents(s->pds_file, (const gchar *)s->pds,
                            sizeof(s->pds), NULL);
    }
}

static uint64_t sgi_sflash_pds_read(void *opaque, hwaddr off, unsigned size)
{
    SGISflashState *s = opaque;
    hwaddr o = off & (SGI_SFLASH_SEG_SIZE - 1);

    uint64_t v;

    if (size <= 2 &&
        (s->cmd == SFLASH_CMD_STATUS_READ || s->cmd == SFLASH_CMD_ESR_READ)) {
        return sflash_read16(SFLASH_STATUS_READY, off, size);
    }
    v = sflash_read_bytes(&s->pds[o], size < 8 ? size : 8);
    if (getenv("SGISFLASH_PDS_DBG")) {
        fprintf(stderr, "SFLASH-PDS: R o=0x%04" HWADDR_PRIx " size=%u -> 0x%"
                PRIx64 "\n", o, size, v);
    }
    return v;
}

static void sgi_sflash_pds_write(void *opaque, hwaddr off, uint64_t value,
                                 unsigned size)
{
    SGISflashState *s = opaque;
    uint16_t v = value & 0xffff;
    hwaddr o = off & (SGI_SFLASH_SEG_SIZE - 1);

    if (s->write_pending) {
        s->pds[o] = (v >> 8) & 0xff;
        if (o + 1 < SGI_SFLASH_SEG_SIZE) {
            s->pds[o + 1] = v & 0xff;
        }
        s->write_pending = false;
        sgi_sflash_pds_save(s);
        return;
    }
    if (s->erase_pending && v == SFLASH_CMD_ERASE_CONFIRM) {
        memset(s->pds, 0xff, sizeof(s->pds));
        s->erase_pending = false;
        s->cmd = 0;
        sgi_sflash_pds_save(s);
        return;
    }

    switch (v) {
    case SFLASH_CMD_WRITE:
        s->write_pending = true;
        s->cmd = v;
        break;
    case SFLASH_CMD_ERASE:
        s->erase_pending = true;
        s->cmd = v;
        break;
    case SFLASH_CMD_STATUS_READ:
    case SFLASH_CMD_ESR_READ:
    case SFLASH_CMD_STATUS_CLEAR:
    case SFLASH_CMD_READ:
        s->cmd = v;
        s->cmd_off = off;
        break;
    default:
        /* Plain data write. */
        s->pds[o] = (v >> 8) & 0xff;
        if (o + 1 < SGI_SFLASH_SEG_SIZE) {
            s->pds[o + 1] = v & 0xff;
        }
        sgi_sflash_pds_save(s);
        break;
    }
}

/* --- boilerplate ----------------------------------------------------- */

static const MemoryRegionOps sgi_sflash_cmd_ops = {
    .read = sgi_sflash_cmd_read,
    .write = sgi_sflash_cmd_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static const MemoryRegionOps sgi_sflash_pds_ops = {
    .read = sgi_sflash_pds_read,
    .write = sgi_sflash_pds_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void sgi_sflash_reset(DeviceState *dev)
{
    SGISflashState *s = SGI_SFLASH(dev);

    sgi_sflash_pds_load(s);
    s->cmd = 0;
    s->cmd_off = 0;
    s->write_pending = false;
    s->erase_pending = false;
}

static void sgi_sflash_realize(DeviceState *dev, Error **errp)
{
    SGISflashState *s = SGI_SFLASH(dev);

    /* Command window at the flash base. */
    memory_region_init_io(&s->cmd_iomem, OBJECT(s), &sgi_sflash_cmd_ops, s,
                          "sgi-sflash-cmd", 16);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->cmd_iomem);

    /* PDS segment window. */
    memory_region_init_io(&s->pds_iomem, OBJECT(s), &sgi_sflash_pds_ops, s,
                          "sgi-sflash-pds", SGI_SFLASH_SEG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->pds_iomem);
}

static const Property sgi_sflash_properties[] = {
    DEFINE_PROP_LINK("rom", SGISflashState, rom, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_STRING("pds", SGISflashState, pds_file),
};

static void sgi_sflash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_sflash_realize;
    device_class_set_legacy_reset(dc, sgi_sflash_reset);
    device_class_set_props(dc, sgi_sflash_properties);
}

static const TypeInfo sgi_sflash_info = {
    .name = TYPE_SGI_SFLASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGISflashState),
    .class_init = sgi_sflash_class_init,
};

static void sgi_sflash_register_types(void)
{
    type_register_static(&sgi_sflash_info);
}

type_init(sgi_sflash_register_types)
