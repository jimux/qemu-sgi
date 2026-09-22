/*
 * Silicon Graphics IRIS IP2 processor board (IRIS 2[345]00T and 3xxx).
 *
 * Board-level model of the IP2 used by the IRIS 3130 machine: the system
 * segment (PROM, DUARTs, RTC, NVRAM, status/switch/parity registers, the
 * page-table SRAM), the custom segment MMU and its 4 MB of RAM, and the
 * reset-time boot mirror that presents the system segment at address zero.
 *
 * The register layout and the page-table/protection semantics follow the
 * GL2-W3.7 kernel sources (sys/ipII/cpureg.h, pte.h) and MAME's ip2 model
 * (reference/mame_ip2.cpp), which is the phase-1 oracle.
 *
 * Copyright (c) 2026
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/sgi_ip2.h"

#define IP2_PROM_BASE       0x30000000
#define IP2_PROM_SIZE       0x18000
#define IP2_SYS_SIZE        0x10000000
#define IP2_RAM_SIZE        (4 * MiB)
#define IP2_PAGE_ENTRIES    16384
#define IP2_NVRAM_SIZE      0x800

#define IP2_SEG_TD          0
#define IP2_SEG_STK         1
#define IP2_SEG_OS          2

/* status register bits (cpureg.h / mame_ip2.cpp) */
#define ST_MBINIT           0x0040
#define ST_BOOT_            0x0080
#define ST_SYS_SEG_ONLY     0x0080  /* ST_SYSSEG_ on the 3.x board */

/* page table entry masks (pte.h / mame_ip2.cpp) */
#define PAGE_PFNUM          0x00001fff
#define PAGE_PROT           0x30000000
#define PAGE_M              0x80000000
#define PAGE_R              0x40000000
#define PAGE_ALL            0xf0001fff

/* RTC control register bits (tod.h TOD_* / mame_ip2.cpp RTC_*) */
#define RTC_AS              0x01
#define RTC_DS              0x02
#define RTC_RE              0x04
#define RTC_CE              0x08

#define REG_MBTN            0x00800000
#define REG_MLOC            0x01000000
#define REG_SWITCH          0x01800000
#define REG_NVRAM           0x03000000
#define REG_RTC_CTRL        0x04000000
#define REG_RTC_DATA        0x05000000
#define REG_KBASE           0x06000000
#define REG_STATUS          0x08000000
#define REG_PARITY          0x09000000
#define REG_MBPROT          0x0a000000
#define REG_PAGE            0x0b000000
#define REG_TDBASE          0x0c000000
#define REG_TDLIMIT         0x0d000000
#define REG_STKBASE         0x0e000000
#define REG_STKLIMIT        0x0f000000

typedef struct SGIIP2Seg {
    struct SGIIP2State *s;
    int seg;
} SGIIP2Seg;

struct SGIIP2State {
    SysBusDevice parent_obj;

    MemoryRegion sys;
    MemoryRegion boot;
    MemoryRegion prom;
    MemoryRegion regs;
    MemoryRegion nvram;
    MemoryRegion page;
    MemoryRegion seg[3];
    SGIIP2Seg seg_ctx[3];

    uint8_t *ram;
    uint32_t page_tbl[IP2_PAGE_ENTRIES];

    uint16_t base[3];
    uint16_t limit[3];
    uint16_t swreg;
    uint16_t status;
    uint8_t pctrl;
    uint8_t mbprot;
    uint8_t rtcregs[64];
    uint8_t rtc_ctrl;
    uint8_t rtc_addr;

    char *prom_file;
};

static void ip2_set_boot(SGIIP2State *s, bool enabled)
{
    memory_region_set_enabled(&s->boot, enabled);
}

static uint32_t ip2_ram_read(SGIIP2State *s, uint32_t phys, unsigned size)
{
    if (phys + size > IP2_RAM_SIZE) {
        return 0;
    }
    switch (size) {
    case 1:
        return s->ram[phys];
    case 2:
        return lduw_be_p(s->ram + phys);
    default:
        return ldl_be_p(s->ram + phys);
    }
}

static void ip2_ram_write(SGIIP2State *s, uint32_t phys, uint32_t val,
                          unsigned size)
{
    if (phys + size > IP2_RAM_SIZE) {
        return;
    }
    switch (size) {
    case 1:
        s->ram[phys] = val;
        break;
    case 2:
        stw_be_p(s->ram + phys, val);
        break;
    default:
        stl_be_p(s->ram + phys, val);
        break;
    }
}

static bool ip2_translate(SGIIP2State *s, int seg, uint32_t offset,
                          uint32_t *phys)
{
    uint16_t page, page_number;
    uint32_t pte;

    if (seg == IP2_SEG_STK) {
        page = extract32(offset, 12, 14) ^ 0x3fff;
        if (s->limit[seg] && page > s->limit[seg]) {
            return false;
        }
        page_number = s->base[seg] - page;
    } else {
        page = extract32(offset, 12, 14);
        if (s->limit[seg] && page > s->limit[seg]) {
            return false;
        }
        page_number = s->base[seg] + page;
    }

    pte = s->page_tbl[page_number & (IP2_PAGE_ENTRIES - 1)];
    if ((pte & PAGE_PROT) == 0) {
        return false;
    }
    *phys = ((pte & PAGE_PFNUM) << 12) | (offset & 0xfff);
    return true;
}

static uint64_t ip2_seg_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2Seg *ctx = opaque;
    SGIIP2State *s = ctx->s;
    uint32_t phys;

    if (!ip2_translate(s, ctx->seg, addr, &phys)) {
        return 0;
    }
    return ip2_ram_read(s, phys, size);
}

static void ip2_seg_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    SGIIP2Seg *ctx = opaque;
    SGIIP2State *s = ctx->s;
    uint32_t phys;

    if (!ip2_translate(s, ctx->seg, addr, &phys)) {
        return;
    }
    ip2_ram_write(s, phys, val, size);
}

static const MemoryRegionOps ip2_seg_ops = {
    .read = ip2_seg_read,
    .write = ip2_seg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint8_t ip2_rtc_direct_read(SGIIP2State *s, uint8_t addr)
{
    switch (addr & 0x3f) {
    case 0x0a:                      /* register A */
        return 0x26;
    case 0x0c:                      /* register C: keep the periodic flag up */
        return 0x40;
    case 0x0d:                      /* register D: valid RAM and time */
        return 0x80;
    default:
        return s->rtcregs[addr & 0x3f];
    }
}

static void ip2_rtc_direct_write(SGIIP2State *s, uint8_t addr, uint8_t val)
{
    s->rtcregs[addr & 0x3f] = val;
}

static uint64_t ip2_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2State *s = opaque;

    switch (addr) {
    case REG_MBTN:
        return 0;
    case REG_MLOC:
        return 0;
    case REG_SWITCH:
        return s->swreg;
    case REG_RTC_CTRL:
        return s->rtc_ctrl;
    case REG_RTC_DATA:
        if (s->rtc_ctrl == (RTC_RE | RTC_DS)) {
            return ip2_rtc_direct_read(s, s->rtc_addr);
        }
        return s->rtc_addr;
    case REG_KBASE:
        return s->base[IP2_SEG_OS] >> 8;
    case REG_STATUS:
        return s->status;
    case REG_PARITY:
        return s->pctrl;
    case REG_MBPROT:
        return s->mbprot;
    case REG_TDBASE:
        return s->base[IP2_SEG_TD];
    case REG_TDLIMIT:
        return s->limit[IP2_SEG_TD];
    case REG_STKBASE:
        return s->base[IP2_SEG_STK];
    case REG_STKLIMIT:
        return s->limit[IP2_SEG_STK];
    default:
        return 0;
    }
}

static void ip2_regs_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    SGIIP2State *s = opaque;
    uint16_t data = val;

    switch (addr) {
    case REG_SWITCH:
        s->swreg = data;
        break;
    case REG_RTC_CTRL:
        s->rtc_ctrl = val;
        break;
    case REG_RTC_DATA:
        if (s->rtc_ctrl == RTC_CE) {
            ip2_rtc_direct_write(s, s->rtc_addr, val);
        } else {
            s->rtc_addr = val;
        }
        break;
    case REG_KBASE:
        s->base[IP2_SEG_OS] = (uint16_t)val << 8;
        break;
    case REG_STATUS:
        if ((data ^ s->status) & ST_BOOT_) {
            ip2_set_boot(s, !(data & ST_BOOT_));
        }
        s->status = data;
        break;
    case REG_PARITY:
        s->pctrl = val;
        break;
    case REG_MBPROT:
        s->mbprot = val;
        break;
    case REG_TDBASE:
        s->base[IP2_SEG_TD] = data;
        break;
    case REG_TDLIMIT:
        s->limit[IP2_SEG_TD] = data;
        break;
    case REG_STKBASE:
        s->base[IP2_SEG_STK] = data;
        break;
    case REG_STKLIMIT:
        s->limit[IP2_SEG_STK] = data;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ip2_regs_ops = {
    .read = ip2_regs_read,
    .write = ip2_regs_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t ip2_page_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2State *s = opaque;

    return s->page_tbl[(addr >> 2) & (IP2_PAGE_ENTRIES - 1)];
}

static void ip2_page_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    SGIIP2State *s = opaque;

    s->page_tbl[(addr >> 2) & (IP2_PAGE_ENTRIES - 1)] = val & PAGE_ALL;
}

static const MemoryRegionOps ip2_page_ops = {
    .read = ip2_page_read,
    .write = ip2_page_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void ip2_load_prom(SGIIP2State *s, Error **errp)
{
    g_autofree gchar *data = NULL;
    gsize len = 0;
    GError *gerr = NULL;

    if (!s->prom_file) {
        error_setg(errp, "sgi-ip2: no PROM image supplied (use -bios)");
        return;
    }
    if (!g_file_get_contents(s->prom_file, &data, &len, &gerr)) {
        error_setg(errp, "sgi-ip2: cannot read PROM '%s': %s",
                   s->prom_file, gerr->message);
        g_error_free(gerr);
        return;
    }
    if (len != IP2_PROM_SIZE) {
        error_setg(errp, "sgi-ip2: PROM '%s' is %zu bytes, expected %u",
                   s->prom_file, len, IP2_PROM_SIZE);
        return;
    }
    memory_region_init_rom_nomigrate(&s->prom, OBJECT(s), "ip2.prom",
                                     IP2_PROM_SIZE, &error_fatal);
    memcpy(memory_region_get_ram_ptr(&s->prom), data, IP2_PROM_SIZE);
}

static void ip2_realize(DeviceState *dev, Error **errp)
{
    SGIIP2State *s = SGI_IP2(dev);
    int i;

    s->ram = g_malloc0(IP2_RAM_SIZE);

    ip2_load_prom(s, errp);
    if (*errp) {
        return;
    }

    memory_region_init(&s->sys, OBJECT(s), "ip2.sys", IP2_SYS_SIZE);
    memory_region_init_io(&s->regs, OBJECT(s), &ip2_regs_ops, s, "ip2.regs",
                          IP2_SYS_SIZE);
    memory_region_add_subregion_overlap(&s->sys, 0, &s->regs, 0);
    memory_region_add_subregion_overlap(&s->sys, 0, &s->prom, 1);

    memory_region_init_ram_nomigrate(&s->nvram, OBJECT(s), "ip2.nvram",
                                     IP2_NVRAM_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(&s->sys, REG_NVRAM, &s->nvram, 1);

    memory_region_init_io(&s->page, OBJECT(s), &ip2_page_ops, s, "ip2.page",
                          0x4000 * 4);
    memory_region_add_subregion_overlap(&s->sys, REG_PAGE, &s->page, 1);

    memory_region_add_subregion(get_system_memory(), IP2_PROM_BASE, &s->sys);

    memory_region_init_alias(&s->boot, OBJECT(s), "ip2.boot", &s->sys, 0,
                             IP2_SYS_SIZE);
    memory_region_add_subregion_overlap(get_system_memory(), 0, &s->boot, 1);

    for (i = 0; i < 3; i++) {
        s->seg_ctx[i].s = s;
        s->seg_ctx[i].seg = i;
        memory_region_init_io(&s->seg[i], OBJECT(s), &ip2_seg_ops,
                              &s->seg_ctx[i], "ip2.seg", IP2_SYS_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            (hwaddr)i * IP2_SYS_SIZE,
                                            &s->seg[i], (i == 0) ? 0 : 1);
    }
}

static void ip2_reset(DeviceState *dev)
{
    SGIIP2State *s = SGI_IP2(dev);

    s->status = 0;
    s->base[0] = s->base[1] = s->base[2] = 0;
    s->limit[0] = s->limit[1] = s->limit[2] = 0;
    s->rtc_ctrl = 0;
    s->rtc_addr = 0;
    memset(s->page_tbl, 0, sizeof(s->page_tbl));
    if (s->ram) {
        memset(s->ram, 0, IP2_RAM_SIZE);
    }
    ip2_set_boot(s, true);
}

MemoryRegion *sgi_ip2_sys_region(DeviceState *dev)
{
    return &SGI_IP2(dev)->sys;
}

static const Property ip2_properties[] = {
    DEFINE_PROP_STRING("prom", SGIIP2State, prom_file),
    DEFINE_PROP_UINT16("swreg", SGIIP2State, swreg, 0x0005),
};

static void ip2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ip2_realize;
    device_class_set_legacy_reset(dc, ip2_reset);
    device_class_set_props(dc, ip2_properties);
}

static const TypeInfo ip2_info = {
    .name          = TYPE_SGI_IP2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIIP2State),
    .class_init    = ip2_class_init,
};

static void ip2_register_types(void)
{
    type_register_static(&ip2_info);
}

type_init(ip2_register_types)
