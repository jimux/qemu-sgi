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
#include "system/block-backend.h"
#include "exec/cputlb.h"
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
#define REG_PGFLTCLR        0x03800000
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

/* Multibus: 1 MB memory window, 1 MB map registers, 64 KB I/O (cpureg.h) */
#define IP2_MB_MEM_BASE     0x40000000
#define IP2_MB_REG_BASE     0x40100000
#define IP2_MB_IO_BASE      0x50000000
#define IP2_MB_MAP_ENTRIES  1024

/* Interphase 2190 SMD controller in Multibus I/O space (iphreg.h) */
#define IP2190_PORT         0x7010
#define IP2190_R1           (IP2190_PORT + 0)
#define IP2190_R0           (IP2190_PORT + 1)
#define IP2190_R3           (IP2190_PORT + 2)
#define IP2190_R2           (IP2190_PORT + 3)
#define IP2190_GO           0x21
#define IP2190_CLEAR        0x22
#define IP2190_BUSY         0x01
#define IP2190_DONE         0x02
#define IP2190_S_OK         0x80
#define IP2190_S_ERROR      0x82
#define IP2190_C_READ       0x81
#define IP2190_C_WRITE      0x82
#define IP2190_C_VERIFY     0x83
#define IP2190_C_INIT       0x87
#define IP2190_C_RESTORE    0x89
#define IP2190_C_SEEK       0x8a
#define IP2190_C_READABS    0x93
#define IP2190_C_READNOCACHE 0x94
#define IP2190_SECTOR       512

struct SGIIP2State {
    SysBusDevice parent_obj;

    MemoryRegion sys;
    MemoryRegion boot;
    MemoryRegion prom;
    MemoryRegion regs;
    MemoryRegion nvram;
    MemoryRegion page;
    MemoryRegion ram_mr;
    MemoryRegion mbmem;
    MemoryRegion mbreg;
    MemoryRegion mbio;

    uint8_t *ram;
    uint32_t page_tbl[IP2_PAGE_ENTRIES];
    uint16_t mb_map[IP2_MB_MAP_ENTRIES];

    /* Interphase 2190 state */
    uint8_t ip_iopb_addr[3];
    uint8_t ip_reg0;
    bool ip_done;
    uint8_t ip_heads;
    uint8_t ip_spt;
    BlockBackend *disk;

    uint16_t base[3];
    uint16_t limit[3];
    uint16_t swreg;
    uint16_t status;
    bool boot_enabled;
    CPUState *cpu;
    uint8_t pctrl;
    uint8_t mbprot;
    uint8_t rtcregs[64];
    uint8_t rtc_ctrl;
    uint8_t rtc_addr;

    char *prom_file;
};

static void ip2_tlb_flush(SGIIP2State *s)
{
    if (s->cpu) {
        tlb_flush(s->cpu);
    }
}

static void ip2_set_boot(SGIIP2State *s, bool enabled)
{
    memory_region_set_enabled(&s->boot, enabled);
    s->boot_enabled = enabled;
    ip2_tlb_flush(s);
}

/*
 * Board-provided translation for the IP2's custom page-table MMU, consulted
 * by m68k_cpu_tlb_fill before the default path. Only segments 0/1/2 are MMU
 * controlled; segment 0 is left to the boot mirror while it is enabled, and
 * segments >= 3 are left to the memory map.
 */
int sgi_ip2_ext_tlb_fill(void *opaque, vaddr address, int size,
                         MMUAccessType access_type, int mmu_idx, bool probe,
                         hwaddr *physical, int *prot)
{
    SGIIP2State *s = opaque;
    int seg = (address >> 28) & 0xf;
    uint32_t offset, page, page_number, pte;
    int p = 0;

    (void)size;
    (void)access_type;
    (void)mmu_idx;
    (void)probe;

    if (seg > IP2_SEG_OS) {
        return 0;
    }
    if (seg == IP2_SEG_TD && s->boot_enabled) {
        return 0;
    }

    offset = address & 0x0fffffff;
    if (seg == IP2_SEG_STK) {
        page = extract32(offset, 12, 14) ^ 0x3fff;
        if (s->limit[seg] && page > s->limit[seg]) {
            return s->boot_enabled ? 0 : -1;
        }
        page_number = s->base[seg] - page;
    } else {
        page = extract32(offset, 12, 14);
        if (s->limit[seg] && page > s->limit[seg]) {
            return s->boot_enabled ? 0 : -1;
        }
        page_number = s->base[seg] + page;
    }

    pte = s->page_tbl[page_number & (IP2_PAGE_ENTRIES - 1)];
    switch (pte & PAGE_PROT) {
    case 0x10000000:            /* read only */
        p = PAGE_READ;
        break;
    case 0x20000000:            /* system only */
        /*
         * User/supervisor separation is deferred: distinguishing it needs a
         * target-specific MMU index, and the monitor and kernel both run in
         * supervisor mode. Revisit with P4's userland.
         */
        p = PAGE_READ | PAGE_WRITE;
        break;
    case 0x30000000:            /* read/write */
        p = PAGE_READ | PAGE_WRITE;
        break;
    default:                    /* no access */
        /*
         * Once the kernel owns segment 0 the monitor's boot mirror is gone,
         * so an unmapped page is a real fault the kernel's bus-error handler
         * must see (fill-on-demand); before that, fall through so the mirror
         * keeps serving the PROM.
         */
        return s->boot_enabled ? 0 : -1;
    }
    if (p & PAGE_READ) {
        p |= PAGE_EXEC;
    }
    *physical = (hwaddr)(((pte & PAGE_PFNUM) << 12) | (offset & 0xfff));
    *prot = p;
    return 1;
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
    case REG_PGFLTCLR:
        ip2_tlb_flush(s);
        return 0;
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
    case REG_PGFLTCLR:
        ip2_tlb_flush(s);
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
        ip2_tlb_flush(s);
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
        ip2_tlb_flush(s);
        break;
    case REG_TDLIMIT:
        s->limit[IP2_SEG_TD] = data;
        ip2_tlb_flush(s);
        break;
    case REG_STKBASE:
        s->base[IP2_SEG_STK] = data;
        ip2_tlb_flush(s);
        break;
    case REG_STKLIMIT:
        s->limit[IP2_SEG_STK] = data;
        ip2_tlb_flush(s);
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
    ip2_tlb_flush(s);
}

static const MemoryRegionOps ip2_page_ops = {
    .read = ip2_page_read,
    .write = ip2_page_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Multibus memory access by a card (the 2190 DMA engine). A 24-bit Multibus
 * address is translated through the map registers into system RAM, mirroring
 * what a CPU access to the segment-4 window does.
 */
static void ip2_mb_xfer(SGIIP2State *s, uint32_t mbaddr, uint8_t *buf,
                        int len, bool write)
{
    int i;

    for (i = 0; i < len; i++) {
        uint32_t a = mbaddr + i;
        uint32_t page = s->mb_map[(a >> 12) & (IP2_MB_MAP_ENTRIES - 1)] & 0x3fff;
        uint32_t phys = (page << 12) | (a & 0xfff);

        if (write) {
            ip2_ram_write(s, phys, buf[i], 1);
        } else {
            buf[i] = ip2_ram_read(s, phys, 1);
        }
    }
}

static uint64_t ip2_mbmem_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2State *s = opaque;
    uint32_t page = s->mb_map[(addr >> 12) & (IP2_MB_MAP_ENTRIES - 1)] & 0x3fff;
    uint32_t phys = (page << 12) | (addr & 0xfff);

    return ip2_ram_read(s, phys, size);
}

static void ip2_mbmem_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    SGIIP2State *s = opaque;
    uint32_t page = s->mb_map[(addr >> 12) & (IP2_MB_MAP_ENTRIES - 1)] & 0x3fff;
    uint32_t phys = (page << 12) | (addr & 0xfff);

    ip2_ram_write(s, phys, val, size);
}

static const MemoryRegionOps ip2_mbmem_ops = {
    .read = ip2_mbmem_read,
    .write = ip2_mbmem_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t ip2_mbreg_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2State *s = opaque;

    return s->mb_map[(addr >> 12) & (IP2_MB_MAP_ENTRIES - 1)];
}

static void ip2_mbreg_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    SGIIP2State *s = opaque;

    s->mb_map[(addr >> 12) & (IP2_MB_MAP_ENTRIES - 1)] = val;
}

static const MemoryRegionOps ip2_mbreg_ops = {
    .read = ip2_mbreg_read,
    .write = ip2_mbreg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Interphase 2190 SMD controller. The driver (stand/lib/dev/iph.c and the
 * kernel's sys/multibus/iph.c) builds an IOPB in Multibus memory, points the
 * controller at it with R1/R2/R3, then writes IP_GO to R0 and polls the IOPB
 * status and R0's DONE bit. Commands complete synchronously here.
 */
static void ip2190_go(SGIIP2State *s)
{
    uint32_t iopb_mb = ((uint32_t)(s->ip_iopb_addr[0] & 0x0f) << 16) |
                       ((uint32_t)s->ip_iopb_addr[1] << 8) |
                       s->ip_iopb_addr[2];
    uint8_t iopb[24];
    uint8_t cmd, head, status = IP2190_S_OK, error = 0;
    uint16_t cyl, sec, cnt;
    uint32_t buf;
    uint8_t heads, spt;
    uint64_t lba;
    int len;

    ip2_mb_xfer(s, iopb_mb, iopb, sizeof(iopb), false);
    /*
     * The 2190 sees the IOPB through 16-bit words, so each byte pair is
     * swapped relative to the m68k's struct layout; the field offsets in
     * iphreg.h are the controller's. CPU offsets are one earlier in each
     * pair (cmd at 1, status at 3, unit at 5, bufh at 0xc, ...).
     */
    cmd = iopb[1];
    head = iopb[4];
    cyl = (iopb[7] << 8) | iopb[6];
    sec = (iopb[9] << 8) | iopb[8];
    cnt = (iopb[0xb] << 8) | iopb[0xa];
    buf = ((uint32_t)iopb[0xc] << 16) | ((uint32_t)iopb[0xf] << 8) | iopb[0xe];

    switch (cmd) {
    case IP2190_C_INIT: {
        uint8_t uib[2];

        ip2_mb_xfer(s, buf, uib, sizeof(uib), false);
        s->ip_heads = uib[1];   /* struct uib order: spt@0, hds@1 */
        s->ip_spt = uib[0];
        if (!s->ip_heads || !s->ip_spt) {
            s->ip_heads = 1;
            s->ip_spt = 64;
        }
        break;
    }
    case IP2190_C_READ:
    case IP2190_C_READABS:
    case IP2190_C_READNOCACHE:
    case IP2190_C_VERIFY:
    case IP2190_C_WRITE: {
        uint8_t *tmp;

        heads = s->ip_heads ? s->ip_heads : 1;
        spt = s->ip_spt ? s->ip_spt : 64;
        lba = (uint64_t)((uint32_t)cyl * heads + head) * spt + sec;
        len = (int)cnt * IP2190_SECTOR;
        /*
         * cnt is the IOPB's 16-bit sector count, so the controller places no
         * upper bound tighter than that; only a zero-length transfer is
         * invalid.  The stand's aligned fsread path DMAs a whole block run
         * straight into the load address (vmunix: 247/248/146 sectors
         * = 126464/126976/74752 bytes), so a 64 KB cap silently refused the
         * load -- _devread returns b_bcount unconditionally and never saw
         * the error.
         */
        if (len <= 0) {
            status = IP2190_S_ERROR;
            error = 0x16; /* invalid sector in command */
            break;
        }
        tmp = g_malloc(len);
        if (!s->disk) {
            status = IP2190_S_ERROR;
            error = 0x10; /* disk not ready */
        } else if (cmd == IP2190_C_WRITE) {
            ip2_mb_xfer(s, buf, tmp, len, false);
            if (blk_pwrite(s->disk, lba * IP2190_SECTOR, len, tmp, 0) < 0) {
                status = IP2190_S_ERROR;
                error = 0x18; /* bus timeout */
            }
        } else if (blk_pread(s->disk, lba * IP2190_SECTOR, len, tmp, 0) < 0) {
            status = IP2190_S_ERROR;
            error = 0x10;
        } else if (cmd != IP2190_C_VERIFY) {
            ip2_mb_xfer(s, buf, tmp, len, true);
        }
        g_free(tmp);
        break;
    }
    case IP2190_C_RESTORE:
    case IP2190_C_SEEK:
        break;
    default:
        status = IP2190_S_ERROR;
        error = 0x14; /* invalid command code */
        break;
    }

    iopb[3] = status;   /* controller i_status */
    iopb[2] = error;    /* controller i_error  */
    ip2_mb_xfer(s, iopb_mb, iopb, sizeof(iopb), true);
    s->ip_done = true;
}

static uint64_t ip2_mbio_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIIP2State *s = opaque;

    if (addr == IP2190_R0) {
        return s->ip_done ? IP2190_DONE : 0;
    }
    return 0;
}

static void ip2_mbio_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    SGIIP2State *s = opaque;

    switch (addr) {
    case IP2190_R0:
        if (val == IP2190_CLEAR) {
            s->ip_done = false;
        } else if (val == IP2190_GO) {
            s->ip_done = false;
            ip2190_go(s);
        }
        break;
    case IP2190_R1:
        s->ip_iopb_addr[0] = val;
        break;
    case IP2190_R3:
        s->ip_iopb_addr[2] = val;
        break;
    case IP2190_R2:
        s->ip_iopb_addr[1] = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ip2_mbio_ops = {
    .read = ip2_mbio_read,
    .write = ip2_mbio_write,
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

    /*
     * Physical RAM, reachable by the CPU through the translation fast path
     * (sgi_ip2_ext_tlb_fill), which fills the TLB with the pte-resolved
     * physical addresses. The boot mirror above it shadows this while enabled.
     */
    memory_region_init_ram_ptr(&s->ram_mr, OBJECT(s), "ip2.ram",
                               IP2_RAM_SIZE, s->ram);
    memory_region_add_subregion_overlap(get_system_memory(), 0, &s->ram_mr, 0);

    memory_region_init_io(&s->mbmem, OBJECT(s), &ip2_mbmem_ops, s,
                          "ip2.mbmem", 0x100000);
    memory_region_add_subregion(get_system_memory(), IP2_MB_MEM_BASE,
                                &s->mbmem);
    memory_region_init_io(&s->mbreg, OBJECT(s), &ip2_mbreg_ops, s,
                          "ip2.mbreg", 0x100000);
    memory_region_add_subregion(get_system_memory(), IP2_MB_REG_BASE,
                                &s->mbreg);
    memory_region_init_io(&s->mbio, OBJECT(s), &ip2_mbio_ops, s,
                          "ip2.mbio", 0x10000);
    memory_region_add_subregion(get_system_memory(), IP2_MB_IO_BASE,
                                &s->mbio);
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
    memset(s->mb_map, 0, sizeof(s->mb_map));
    s->ip_iopb_addr[0] = s->ip_iopb_addr[1] = s->ip_iopb_addr[2] = 0;
    s->ip_done = false;
    s->ip_heads = 0;
    s->ip_spt = 0;
    if (s->ram) {
        memset(s->ram, 0, IP2_RAM_SIZE);
    }
    ip2_set_boot(s, true);
}

MemoryRegion *sgi_ip2_sys_region(DeviceState *dev)
{
    return &SGI_IP2(dev)->sys;
}

void sgi_ip2_set_cpu(DeviceState *dev, CPUState *cpu)
{
    SGIIP2State *s = SGI_IP2(dev);

    s->cpu = cpu;
    tlb_flush(cpu);
}

static const Property ip2_properties[] = {
    DEFINE_PROP_STRING("prom", SGIIP2State, prom_file),
    DEFINE_PROP_UINT16("swreg", SGIIP2State, swreg, 0x0005),
    DEFINE_PROP_DRIVE("drive", SGIIP2State, disk),
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
