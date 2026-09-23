/*
 * SGI O2 (IP32) system flash — writable env segment + NVRAM sidecar.
 *
 * See include/hw/misc/sgi_o2flash.h for the full rationale and the list of
 * authoritative references.  In short: the IP32 PROM and the IRIX kernel
 * program the flash with plain byte stores after asserting the MACE ISA
 * write-enable bit; this device honours that bit, and persists the "env"
 * segment to a sidecar file so it survives a fresh QEMU.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_mace.h"
#include "hw/misc/sgi_o2flash.h"
#include "system/memory.h"
#include "qapi/error.h"

/*
 * Validate one flash segment in place: the header words up to (and
 * including) the checksum at +0x3c must sum to zero, and the body words up
 * to (and including) the segment's last aligned word must sum to zero.
 * This mirrors the PROM's validHdr()/validBody() (IP32flash.h, libsk
 * flash.c) and sgi_o2_validate_prom_checksums() in hw/mips/sgi_o2.c.
 */
static bool sgi_o2_flash_seg_valid(const uint8_t *seg, uint32_t seg_len)
{
    uint32_t sum, w;
    int off, body_end, last_off;

    if (seg_len < SGI_O2_FLASH_SEG_HDR + 8) {
        return false;
    }
    if (ldl_be_p(seg + 8) != SGI_O2_FLASH_SEG_MAGIC) {
        return false;
    }
    if (ldl_be_p(seg + 12) != seg_len) {
        return false;
    }

    /* Header checksum: [0, 0x3c) + the word at 0x3c sums to zero. */
    sum = ldl_be_p(seg + 0x3c);
    for (off = 0; off < 0x3c; off += 4) {
        sum += ldl_be_p(seg + off);
    }
    if (sum != 0) {
        return false;
    }

    /* Body checksum: body words + the segment's last aligned word. */
    body_end = (int)seg_len;
    last_off = ((body_end + 3) & ~3) - 4;
    if (last_off < SGI_O2_FLASH_SEG_HDR + 4) {
        return false;
    }
    sum = ldl_be_p(seg + last_off);
    for (off = SGI_O2_FLASH_SEG_HDR; off < last_off; off += 4) {
        w = ldl_be_p(seg + off);
        sum += w;
    }
    return sum == 0;
}

/*
 * Locate the "env" segment in the flash array.  Segments start at 256-byte
 * page boundaries and carry an 'SHDR' header (IP32flash.h).
 */
static void sgi_o2_flash_find_env(SGIO2FlashState *s)
{
    uint32_t off;

    s->env_found = false;
    for (off = 0; off + SGI_O2_FLASH_SEG_HDR <= s->size;
         off += SGI_O2_FLASH_PAGE_SIZE) {
        uint32_t seg_len;
        const uint8_t *seg = s->storage + off;

        if (ldl_be_p(seg + 8) != SGI_O2_FLASH_SEG_MAGIC) {
            continue;
        }
        seg_len = ldl_be_p(seg + 12);
        if (seg_len < SGI_O2_FLASH_SEG_HDR || off + seg_len > s->size) {
            continue;
        }
        if (seg[0x10] == 3 && !memcmp(seg + 0x14, "env", 3)) {
            s->env_offset = off;
            s->env_len = seg_len;
            s->env_found = true;
            return;
        }
    }
    warn_report("sgi-o2-flash: no 'env' segment found; env is read-only");
}

static void sgi_o2_flash_env_save(SGIO2FlashState *s)
{
    FILE *f;

    if (!s->nvram_filename || !s->env_found) {
        return;
    }
    f = fopen(s->nvram_filename, "wb");
    if (!f) {
        warn_report("sgi-o2-flash: cannot write env sidecar '%s'",
                    s->nvram_filename);
        return;
    }
    if (fwrite(s->storage + s->env_offset, 1, s->env_len, f) != s->env_len) {
        warn_report("sgi-o2-flash: short write to env sidecar '%s'",
                    s->nvram_filename);
    }
    fflush(f);
    fclose(f);
}

static void sgi_o2_flash_env_load(SGIO2FlashState *s)
{
    g_autofree uint8_t *buf = NULL;
    FILE *f;
    size_t n;

    if (!s->nvram_filename || !s->env_found) {
        return;
    }
    f = fopen(s->nvram_filename, "rb");
    if (!f) {
        return;
    }
    buf = g_malloc(s->env_len);
    n = fread(buf, 1, s->env_len, f);
    fclose(f);
    if (n != s->env_len) {
        warn_report("sgi-o2-flash: env sidecar '%s' is %zu bytes, expected %u "
                    "— ignoring", s->nvram_filename, n, s->env_len);
        return;
    }
    /*
     * A power cut mid-rewrite leaves a partially-programmed segment with
     * broken checksums.  Never adopt such a sidecar: the PROM's init_env()
     * would drop *every* variable and silently boot to defaults.  Falling
     * back to the compiled-in env is strictly better.
     */
    if (!sgi_o2_flash_seg_valid(buf, s->env_len)) {
        warn_report("sgi-o2-flash: env sidecar '%s' fails segment checksums — "
                    "ignoring it", s->nvram_filename);
        return;
    }
    memcpy(s->storage + s->env_offset, buf, s->env_len);
    qemu_log("sgi-o2-flash: restored env segment from '%s'\n",
             s->nvram_filename);
}

/*
 * Rebuild the env body with AutoLoad forced to "N", recomputing the body
 * checksum.  This is the guest's own serialisation (name=value\0 strings,
 * zero-padded body, negated word sum at the body's last aligned word), so
 * the result is exactly what `setenv AutoLoad N` would persist.
 */
static bool sgi_o2_flash_force_autoload_off(SGIO2FlashState *s)
{
    const uint8_t *seg = s->storage + s->env_offset;
    uint32_t seg_len = s->env_len;
    uint32_t body_end = seg_len & ~3u;
    uint32_t last_off = body_end - 4;
    g_autofree uint8_t *nb = g_malloc0(seg_len);
    const char *p, *pend;
    uint32_t off, sum;

    if (!s->env_found || seg_len < SGI_O2_FLASH_SEG_HDR + 8) {
        return false;
    }

    memcpy(nb, seg, SGI_O2_FLASH_SEG_HDR); /* header (and its checksum) */
    off = SGI_O2_FLASH_SEG_HDR;
    p = (const char *)(seg + SGI_O2_FLASH_SEG_HDR);
    pend = (const char *)(seg + last_off);
    while (p < pend && *p) {
        const char *eq = strchr(p, '=');
        const char *val;
        size_t nlen, vlen;

        if (!eq || eq >= pend) {
            break;
        }
        val = eq + 1;
        nlen = eq - p;
        vlen = strlen(val);
        if (nlen == 8 && !memcmp(p, "AutoLoad", 8)) {
            val = "N";
            vlen = 1;
        }
        if (off + nlen + 1 + vlen + 1 > last_off) {
            warn_report("sgi-o2-flash: env too small to rewrite AutoLoad");
            return false;
        }
        memcpy(nb + off, p, nlen);
        off += nlen;
        nb[off++] = '=';
        memcpy(nb + off, val, vlen);
        off += vlen;
        nb[off++] = '\0';
        p += strlen(p) + 1;
    }

    sum = 0;
    for (off = SGI_O2_FLASH_SEG_HDR; off < last_off; off += 4) {
        sum += ldl_be_p(nb + off);
    }
    stl_be_p(nb + last_off, (uint32_t)(0 - sum));

    memcpy(s->storage + s->env_offset, nb, seg_len);
    return true;
}

void sgi_o2_flash_set_mace(DeviceState *dev, DeviceState *mace)
{
    SGI_O2_FLASH(dev)->mace = mace;
}

void sgi_o2_flash_load_prom(DeviceState *dev, const uint8_t *data,
                            uint32_t len)
{
    SGIO2FlashState *s = SGI_O2_FLASH(dev);

    memset(s->storage, 0, s->size);
    memcpy(s->storage, data, MIN(len, s->size));

    sgi_o2_flash_find_env(s);
    sgi_o2_flash_env_load(s);
    if (!s->autoload) {
        sgi_o2_flash_force_autoload_off(s);
    }
}

/*
 * Write path: honour MACE's write-enable latch, program the byte, and when a
 * full 256-byte sector has been written, persist the env segment if it now
 * checksums (i.e. the guest has finished its whole-segment rewrite).
 */
static void sgi_o2_flash_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    SGIO2FlashState *s = opaque;

    if (addr >= s->size || size != 1) {
        return;
    }
    if (!s->mace || !sgi_mace_flash_write_enabled(SGI_MACE(s->mace))) {
        /* Programming is gated on ISA_FLASH_NIC_REG bit 0. */
        return;
    }

    s->storage[addr] = (uint8_t)value;

    if (s->env_found && addr >= s->env_offset &&
        addr < s->env_offset + s->env_len &&
        ((addr + 1) & (SGI_O2_FLASH_PAGE_SIZE - 1)) == 0) {
        if (sgi_o2_flash_seg_valid(s->storage + s->env_offset, s->env_len)) {
            sgi_o2_flash_env_save(s);
        }
    }
}

static uint64_t sgi_o2_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIO2FlashState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    /* romd mode serves reads directly; this is the non-romd fallback. */
    for (i = 0; i < size && addr + i < s->size; i++) {
        val = (val << 8) | s->storage[addr + i];
    }
    return val;
}

static const MemoryRegionOps sgi_o2_flash_ops = {
    .read = sgi_o2_flash_read,
    .write = sgi_o2_flash_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1, /* split every access into bytes */
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void sgi_o2_flash_realize(DeviceState *dev, Error **errp)
{
    SGIO2FlashState *s = SGI_O2_FLASH(dev);

    s->size = SGI_O2_FLASH_SIZE;

    /*
     * A rom_device keeps reads and instruction fetches direct (RAM) while
     * routing all writes to sgi_o2_flash_write(); romd stays on because the
     * IP32 flash has no read-status/command mode.
     */
    if (!memory_region_init_rom_device(&s->mmio, OBJECT(dev),
                                       &sgi_o2_flash_ops, s,
                                       TYPE_SGI_O2_FLASH, s->size, errp)) {
        return;
    }
    s->storage = memory_region_get_ram_ptr(&s->mmio);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static const Property sgi_o2_flash_properties[] = {
    DEFINE_PROP_STRING("nvram-file", SGIO2FlashState, nvram_filename),
    DEFINE_PROP_BOOL("autoload", SGIO2FlashState, autoload, true),
};

static void sgi_o2_flash_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "SGI O2 system flash";
    dc->realize = sgi_o2_flash_realize;
    device_class_set_props(dc, sgi_o2_flash_properties);
}

static const TypeInfo sgi_o2_flash_info = {
    .name = TYPE_SGI_O2_FLASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIO2FlashState),
    .class_init = sgi_o2_flash_class_init,
};

static void sgi_o2_flash_register_types(void)
{
    type_register_static(&sgi_o2_flash_info);
}

type_init(sgi_o2_flash_register_types)
