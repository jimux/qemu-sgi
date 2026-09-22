/*
 * QEMU SGI Personal IRIS (IP6) machine emulation
 *
 * The Personal IRIS 4D/20 (R2000) and 4D/25 (R3000) share the IP6 CPU
 * board: CTL1 memory/config gate array, Am7990 LANCE Ethernet, WD33C93
 * SCSI, two SCN2681 DUARTs, DP8572A RTC, 8254 PIT, 93C56 serial EEPROM
 * and a GR1 ("Eclipse") graphics board.
 *
 * This machine currently implements the 4D/25 configuration: the CTL1
 * register shell (memcfg/sysid/cpucfg/cpuauxctl + DMA map), the serial
 * EEPROM the PROM uses for board/memory detection, flat RAM, the boot
 * PROM and the LIO interrupt block.  The remaining devices (SCSI, LANCE,
 * graphics, audio, DUART, PIT) are reachable but unimplemented.
 *
 * References:
 *   - MAME src/mame/sgi/ip6.cpp and ctl1.cpp (hardware reference)
 *   - SGI IP-6 schematic; VME-Eclipse CPU (VIP10) specification
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/qdev.h"
#include "hw/core/loader.h"
#include "hw/char/sgi_scn2681.h"
#include "hw/misc/sgi_ip6_input.h"
#include "trace.h"
#include "hw/isa/isa.h"
#include "hw/mips/mips.h"
#include "hw/misc/unimp.h"
#include "hw/nvram/eeprom93xx.h"
#include "hw/scsi/scsi.h"
#include "hw/scsi/wd33c93.h"
#include "hw/timer/i8254_internal.h"
#include "net/net.h"
#include "hw/net/pcnet.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"

#define SGI_IP6_PROM_BASE   0x1fc00000ULL
#define SGI_IP6_PROM_SIZE   (256 * KiB)
#define SGI_IP6_RAM_MAX     (256 * MiB)

/*
 * Memory decode (CTL1), modelled on MAME ip6.cpp / ctl1.cpp.  The guest
 * configures memcfg, which installs RAM per bank at b * conf_size, each
 * bank holding min(host SIMM size, conf_size) with the same host RAM
 * mirrored to fill the conf_size window.  Anything not covered reads 0
 * (MAME's noprw placeholder) - the PROM's sizing probe relies on that.
 */
#define SGI_IP6_RAM_WINDOW     (256 * MiB)
#define SGI_IP6_HOST_SIMM_SIZE (16 * MiB)
#define SGI_IP6_RAM_ALIAS_MAX  128

/* CTL1 control register block */
#define SGI_IP6_CTL1_BASE   0x1f800000ULL
#define SGI_IP6_CTL1_SIZE   0x100000
#define SGI_IP6_CTL1_MEMCFG 0x00000
#define SGI_IP6_CTL1_CPUCFG 0x80000
#define SGI_IP6_CTL1_CPUAUX 0xe0000

/* CTL1 DMA address-mapping registers (MAME ip6.cpp layout):
 *   0x1f900000  dmalo    (16-bit, resets mapindex to 0 on write)
 *   0x1f910000  mapindex (8-bit)
 *   0x1f920000  dmahi    (1024/2048 x 16-bit SRAM, A10 strapping) */
#define SGI_IP6_DMALO_BASE    0x1f900000ULL
#define SGI_IP6_MAPINDEX_BASE 0x1f910000ULL
#define SGI_IP6_DMAHI_BASE    0x1f920000ULL
#define SGI_IP6_DMAHI_SIZE    0x1000

/* LIO interrupt controller */
#define SGI_IP6_LIO_BASE    0x1f980000ULL
#define SGI_IP6_LIO_SIZE    0x10

/* CTL1 error / refresh / graphics-DMA registers */
#define SGI_IP6_ERR_BASE    0x1fa40000ULL
#define SGI_IP6_ERR_SIZE    0x20
#define SGI_IP6_CLRERR_BASE 0x1faa0000ULL
#define SGI_IP6_CLRERR_SIZE 0x8

/* DP8572A RTC */
#define SGI_IP6_RTC_BASE    0x1fbc0000ULL
#define SGI_IP6_RTC_SIZE    0x80

/* Am7990 LANCE Ethernet (registers and reset lines) */
#define SGI_IP6_LANCE_BASE      0x1f950000ULL
#define SGI_IP6_LANCE_REG_SIZE  0x200
#define SGI_IP6_LANCE_RST_BASE  0x1f960000ULL
#define SGI_IP6_LANCE_RST_SIZE  0x8

/* WD33C93 SCSI (indirect address/data ports) and its reset lines */#define SGI_IP6_SCSI_BASE   0x1fb00000ULL
#define SGI_IP6_SCSI_SIZE   0x200
#define SGI_IP6_SCSIRST_BASE 0x1fa80000ULL
#define SGI_IP6_SCSIRST_SIZE 0x10

/* LIO interrupt bits */
#define LIO_SCSI            4
#define LIO_ENET            5

/* SCN2681 DUARTs */
#define SGI_IP6_DUART_BASE  0x1fb80000ULL
#define SGI_IP6_DUART_SIZE  0x100

/* 8254 PIT: byte registers in the top bus lane, ports 0..3 at +0/+4/+8/+12 */
#define SGI_IP6_PIT_BASE    0x1fb40000ULL

/* PIT interrupt acknowledge registers (read clears the CPU IRQ line) */
#define SGI_IP6_TIMER0_ACK  0x1fa20000ULL   /* PIT channel 0 -> CPU IRQ2 */
#define SGI_IP6_TIMER1_ACK  0x1fa00000ULL   /* PIT channel 1 -> CPU IRQ4 */

/* cpuauxctl bits */
#define CPUAUX_EEPROM_CS    0x20
#define CPUAUX_EEPROM_CLK   0x40

/* cpucfg bits */
#define CPUCFG_SERDATA      0x0100
#define CPUCFG_SIN          0x0200

/* sysid bits */
#define SYSID_SERDATA       0x01
#define SYSID_FPPRES        0x02

/* memcfg bits */
#define MEMCFG_MEMSIZE      0x0f
#define MEMCFG_4MRAM        0x10
#define MEMCFG_TIMERDIS     0x20

typedef struct SGIip6State {
    MemoryRegion ctl1;
    MemoryRegion ram_win;
    MemoryRegion ram_zero;
    MemoryRegion bank_alias[SGI_IP6_RAM_ALIAS_MAX];
    unsigned bank_alias_count;
    uint64_t ram_size;
    MemoryRegion dmalo_reg;
    MemoryRegion mapindex_reg;
    MemoryRegion dmahi_reg;
    MemoryRegion lio;
    MemoryRegion err;
    MemoryRegion clrerr;
    MemoryRegion rtc;
    MemoryRegion scsi_regs;
    MemoryRegion scsi_reset;
    MemoryRegion duart_regs;
    MemoryRegion lance_regs;
    MemoryRegion lance_reset;
    MemoryRegion pit_reg;
    MemoryRegion timer0_ack;
    MemoryRegion timer1_ack;

    ISABus *isa;
    DeviceState *pit;

    MIPSCPU *cpu;
    WD33C93State *scsi;
    SCN2681State *duart[2];
    SgiIp6InputState *mouse;
    MemoryRegion gr1_regs;
    uint32_t gr1_win[0x800];  /* upper bank 0x8000..0x9fff (2048 words) */
    uint8_t gr1_bank;         /* mar_msb: bank selected at 0x0e00..0x0e07 */
    uint8_t gr1_dr[5];        /* dr0..dr4 display registers */

    PCNetState *lance;
    DeviceState *lance_dev;

    eeprom_t *eeprom;

    uint8_t rtc_regs[SGI_IP6_RTC_SIZE];
    QEMUTimer *rtc_timer;

    bool lio_int;
    bool pit0_level;
    bool pit0_programmed;

    uint8_t memcfg;
    uint16_t cpucfg;
    uint8_t cpuauxctl;

    uint16_t dmalo;
    uint8_t mapindex;
    uint16_t dmahi[2048];

    uint32_t erradr;
    uint32_t refadr;
    int64_t ref_load_time;

    uint8_t vme_isr;
    uint8_t vme_imr;

    uint16_t lio_isr;
    uint8_t lio_imr;
} SGIip6State;

static SGIip6State ip6_state;

static void sgi_ip6_lio_update(SGIip6State *s);

/* ---- Memory decode (CTL1) -------------------------------------------- */

/* MAME's noprw placeholder: uncovered RAM addresses read 0, writes ignored,
 * and never fault - the PROM/IDE sizing probe depends on that tolerance. */
static uint64_t sgi_ip6_ram_zero_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void sgi_ip6_ram_zero_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size)
{
}

static const MemoryRegionOps sgi_ip6_ram_zero_ops = {
    .read = sgi_ip6_ram_zero_read,
    .write = sgi_ip6_ram_zero_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/*
 * (Re)install the RAM decode from memcfg.  The bank geometry (count and
 * conf_size) is guest-supplied via memcfg; the per-bank population is
 * host-supplied (SGI_IP6_HOST_SIMM_SIZE).  A bank is installed at
 * b * conf_size, its host RAM sized min(host SIMM, conf_size) and mirrored
 * to fill the conf_size window.  The caller must undecode first.
 */
static void sgi_ip6_ram_decode(SGIip6State *s)
{
    uint64_t conf_size = (s->memcfg & MEMCFG_4MRAM) ? 16 * MiB : 4 * MiB;
    unsigned banks = (s->memcfg & MEMCFG_MEMSIZE) + 1;
    unsigned i = 0, b;

    for (b = 0; b < banks; b++) {
        uint64_t host_off = (uint64_t)b * SGI_IP6_HOST_SIMM_SIZE;
        uint64_t size, base, addr;

        if (host_off >= s->ram_size) {
            break;
        }
        size = MIN(conf_size, s->ram_size - host_off);
        base = (uint64_t)b * conf_size;
        for (addr = base; addr + size <= base + conf_size; addr += size) {
            MemoryRegion *a;

            if (i >= SGI_IP6_RAM_ALIAS_MAX || addr + size > SGI_IP6_RAM_WINDOW) {
                break;
            }
            a = &s->bank_alias[i];
            memory_region_set_alias_offset(a, host_off);
            memory_region_set_size(a, size);
            memory_region_add_subregion_overlap(&s->ram_win, addr, a, 1);
            i++;
        }
    }
    s->bank_alias_count = i;
}

static void sgi_ip6_ram_undecode(SGIip6State *s)
{
    unsigned i;

    for (i = 0; i < s->bank_alias_count; i++) {
        memory_region_del_subregion(&s->ram_win, &s->bank_alias[i]);
    }
    s->bank_alias_count = 0;
}

/* ---- CTL1 control registers ------------------------------------------ */

static uint64_t sgi_ip6_ctl1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = (addr & (SGI_IP6_CTL1_SIZE - 1)) & ~3u;
    uint32_t val = 0;

    switch (off) {
    case SGI_IP6_CTL1_MEMCFG: {
        /* memcfg in the top byte lane, sysid in the next. */
        uint8_t sysid = s->eeprom ? eeprom93xx_read(s->eeprom) : 0;

        val = ((uint32_t)s->memcfg << 24) | ((uint32_t)sysid << 16);
        break;
    }
    case SGI_IP6_CTL1_CPUCFG:
        val = s->cpucfg;
        break;
    case SGI_IP6_CTL1_CPUAUX:
        val = (uint32_t)s->cpuauxctl << 24;
        break;
    case 0x40000: /* VME interrupt status register */
        val = s->vme_isr;
        break;
    case 0x40008: /* VME interrupt mask register */
        val = s->vme_imr;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi-ip6: unimplemented ctl1 read 0x%" HWADDR_PRIx
                      " size %u\n", addr, size);
        break;
    }

    /* Registers sit in a byte lane selected by the low address bits on the
     * SGI 32-bit bus; shift out the byte the CPU asked for. */
    if (size == 1) {
        val >>= 8 * (3 - (addr & 3));
    } else if (size == 2) {
        val >>= 8 * (2 - (addr & 2));
    }
    return val;
}

static void sgi_ip6_ctl1_write(void *opaque, hwaddr addr, uint64_t data,
                               unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = (addr & (SGI_IP6_CTL1_SIZE - 1)) & ~3u;
    uint32_t val = data;

    /* Normalise a sub-word access into the word-wide lane it targets. */
    if (size == 1) {
        val = (val & 0xff) << (8 * (3 - (addr & 3)));
    } else if (size == 2) {
        val = (val & 0xffff) << (8 * (2 - (addr & 2)));
    }

    switch (off) {
    case SGI_IP6_CTL1_MEMCFG:
        s->memcfg = (val >> 24) & 0xff;
        /* The guest programs the memory decode through memcfg. */
        sgi_ip6_ram_undecode(s);
        sgi_ip6_ram_decode(s);
        break;
    case SGI_IP6_CTL1_CPUCFG: {
        uint16_t cfg = val & 0xffff;

        if (s->eeprom) {
            eeprom93xx_write(s->eeprom,
                             !!(s->cpuauxctl & CPUAUX_EEPROM_CS),
                             !!(s->cpuauxctl & CPUAUX_EEPROM_CLK),
                             !!(cfg & CPUCFG_SERDATA));
        }
        if ((cfg & CPUCFG_SIN) && !(s->cpucfg & CPUCFG_SIN)) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        s->cpucfg = cfg;
        break;
    }
    case SGI_IP6_CTL1_CPUAUX:
        s->cpuauxctl = (val >> 24) & 0xff;
        if (s->eeprom) {
            eeprom93xx_write(s->eeprom,
                             !!(s->cpuauxctl & CPUAUX_EEPROM_CS),
                             !!(s->cpuauxctl & CPUAUX_EEPROM_CLK),
                             !!(s->cpucfg & CPUCFG_SERDATA));
        }
        break;
    case 0x40000: /* VME interrupt status register */
        s->vme_isr = val & 0xff;
        break;
    case 0x40008: /* VME interrupt mask register */
        s->vme_imr = val & 0xff;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi-ip6: unimplemented ctl1 write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 " size %u\n", addr, data, size);
        break;
    }
}

static const MemoryRegionOps sgi_ip6_ctl1_ops = {
    .read = sgi_ip6_ctl1_read,
    .write = sgi_ip6_ctl1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- CTL1 DMA address-mapping registers ------------------------------ */

static uint64_t sgi_ip6_dmalo_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t v = s->dmalo;

    if (size == 1) {
        v = (addr & 1) ? (v & 0xff) : ((v >> 8) & 0xff);
    }
    return v;
}

static void sgi_ip6_dmalo_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SGIip6State *s = opaque;

    if (size == 1) {
        if (addr & 1) {
            s->dmalo = (s->dmalo & 0xff00) | (data & 0xff);
        } else {
            s->dmalo = (s->dmalo & 0x00ff) | ((data & 0xff) << 8);
        }
    } else {
        s->dmalo = data & 0xffff;
    }
    s->mapindex = 0;
}

static const MemoryRegionOps sgi_ip6_dmalo_ops = {
    .read = sgi_ip6_dmalo_read,
    .write = sgi_ip6_dmalo_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

static uint64_t sgi_ip6_mapindex_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;

    return s->mapindex;
}

static void sgi_ip6_mapindex_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size)
{
    SGIip6State *s = opaque;

    s->mapindex = data & 0xff;
}

static const MemoryRegionOps sgi_ip6_mapindex_ops = {
    .read = sgi_ip6_mapindex_read,
    .write = sgi_ip6_mapindex_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static uint64_t sgi_ip6_dmahi_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t idx = ((addr & (SGI_IP6_DMAHI_SIZE - 1)) >> 2) & 0x3ff;

    return s->dmahi[idx];
}

static void sgi_ip6_dmahi_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t idx = ((addr & (SGI_IP6_DMAHI_SIZE - 1)) >> 2) & 0x3ff;

    s->dmahi[idx] = data & 0xffff;
}

static const MemoryRegionOps sgi_ip6_dmahi_ops = {
    .read = sgi_ip6_dmahi_read,
    .write = sgi_ip6_dmahi_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

/* ---- LIO interrupt block --------------------------------------------- */

static uint64_t sgi_ip6_lio_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & 0xf;

    if (off < 4) {
        /* 16-bit interrupt status register */
        uint32_t v = s->lio_isr;

        if (size == 1) {
            v >>= 8 * (3 - (addr & 3));
        }
        return v;
    }
    if (off >= 8 && off < 0xc) {
        /* 8-bit interrupt mask register */
        return s->lio_imr;
    }
    return 0;
}

static void sgi_ip6_lio_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & 0xf;

    if (off >= 8 && off < 0xc) {
        s->lio_imr = data & 0xff;
        sgi_ip6_lio_update(s);
    }
}

static const MemoryRegionOps sgi_ip6_lio_ops = {
    .read = sgi_ip6_lio_read,
    .write = sgi_ip6_lio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- CTL1 error / refresh registers ---------------------------------- */

static uint64_t sgi_ip6_err_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_ERR_SIZE - 1);

    switch (off) {
    case 0x00:
        /* Reading erradr clears the CPU bus-error interrupt. */
        return s->erradr;
    case 0x04:
        /*
         * Refresh address counter.  While the memory timer is enabled it
         * free-runs: one refresh every 64us (15.625 kHz), each advancing the
         * address by 4096 words.  The PROM measures delays by watching this.
         */
        if (s->memcfg & MEMCFG_TIMERDIS) {
            int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->ref_load_time;
            uint64_t refreshes = (uint64_t)ns * 15625 / 1000000000ULL;

            return s->refadr + (uint32_t)(refreshes * 4096 * 4);
        }
        return s->refadr;
    default:
        return 0;
    }
}

static void sgi_ip6_err_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_ERR_SIZE - 1);

    if (off == 0x04) {
        s->refadr = data;
        s->ref_load_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static const MemoryRegionOps sgi_ip6_err_ops = {
    .read = sgi_ip6_err_read,
    .write = sgi_ip6_err_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t sgi_ip6_clrerr_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void sgi_ip6_clrerr_write(void *opaque, hwaddr addr, uint64_t data,
                                 unsigned size)
{
}

static const MemoryRegionOps sgi_ip6_clrerr_ops = {
    .read = sgi_ip6_clrerr_read,
    .write = sgi_ip6_clrerr_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- DP8572A RTC ------------------------------------------------------ */

/*
 * Placeholder register file for the DP8572A real-time clock.  The PROM uses
 * a control bit (0x4c bit 7) as a one-shot latch during its FPU/reset
 * sequencing, so the registers must persist writes even before the calendar
 * is modelled.
 */
/*
 * DP8572A register file.  The PROM maps the chip as 32 registers at 4-byte
 * stride with the register byte in the top bus lane (bits 31:24): it writes
 * `sw value<<24` and reads back `srl ...,0x18`.  Indexing by byte offset and
 * masking `data & 0xff` therefore stored zeros, so the PROM's set-then-verify
 * never matched and it looped on "can't set tod clock".  Decode the lane.
 */
static uint64_t sgi_ip6_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    unsigned reg = (addr & 0x7f) >> 2;
    uint8_t v;

    /*
     * The reader (PROM 0xbfc13138) zero-writes regs 0x19..0x1d and then reads
     * those same offsets as the calendar - a write-then-read at one offset can
     * only be the chip's latched time window, not the register just written.
     * Present the live calendar there (setter wrote regs 6..11).
     */
    switch (reg) {
    case 0x19: v = s->rtc_regs[6]; break;   /* seconds */
    case 0x1a: v = s->rtc_regs[7]; break;   /* minutes */
    case 0x1b: v = s->rtc_regs[8]; break;   /* hours */
    case 0x1c: v = s->rtc_regs[10]; break;  /* month */
    case 0x1d: v = s->rtc_regs[9]; break;   /* day */
    case 0x0b: v = s->rtc_regs[11]; break;  /* year */
    default: v = s->rtc_regs[reg]; break;
    }

    switch (size) {
    case 4:
        return (uint32_t)v << 24;
    case 2:
        return (uint32_t)v << 8;
    default:
        return v;
    }
}

static void sgi_ip6_rtc_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;
    unsigned reg = (addr & 0x7f) >> 2;
    uint8_t v;

    /* The reader's zero-writes in this window are a latch strobe, not data. */
    if (reg >= 0x19 && reg <= 0x1d) {
        return;
    }

    switch (size) {
    case 4:
        v = (data >> 24) & 0xff;
        break;
    case 2:
        v = (data >> 8) & 0xff;
        break;
    default:
        v = data & 0xff;
        break;
    }
    s->rtc_regs[reg] = v;
}

/* BCD increment with roll-over at @max. */
static uint8_t sgi_ip6_rtc_bcd_inc(uint8_t v, uint8_t max)
{
    v++;
    if ((v & 0x0f) > 9) {
        v += 6;
    }
    if (v > max) {
        v = 0;
    }
    return v;
}

/*
 * The DP8572A counts hundredths of a second, and the PROM polls that register
 * (reg 5) waiting for the clock to run, so a seconds-granular tick would leave
 * it frozen.  Advance at 100 Hz and roll the writable BCD set up.
 */
static void sgi_ip6_rtc_tick(void *opaque)
{
    SGIip6State *s = opaque;
    uint8_t *r = s->rtc_regs;

    r[5] = sgi_ip6_rtc_bcd_inc(r[5], 0x99);             /* hundredths */
    if (r[5] == 0) {
        r[6] = sgi_ip6_rtc_bcd_inc(r[6], 0x59);         /* seconds */
        if (r[6] == 0) {
            r[7] = sgi_ip6_rtc_bcd_inc(r[7], 0x59);     /* minutes */
            if (r[7] == 0) {
                r[8] = sgi_ip6_rtc_bcd_inc(r[8], 0x23); /* hours */
                if (r[8] == 0) {
                    r[9] = sgi_ip6_rtc_bcd_inc(r[9], 0x31);  /* day */
                    if (r[9] == 0) {
                        r[10] = sgi_ip6_rtc_bcd_inc(r[10], 0x12); /* month */
                        if (r[10] == 0) {
                            r[11] = sgi_ip6_rtc_bcd_inc(r[11], 0x99);
                        }
                    }
                }
            }
        }
    }

    timer_mod(s->rtc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / 100);
}

static const MemoryRegionOps sgi_ip6_rtc_ops = {
    .read = sgi_ip6_rtc_read,
    .write = sgi_ip6_rtc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- LIO interrupt line ---------------------------------------------- */

static void sgi_ip6_lio_update(SGIip6State *s)
{
    bool level = (~s->lio_isr) & s->lio_imr;

    if (level != s->lio_int && s->cpu) {
        s->lio_int = level;
        qemu_set_irq(s->cpu->env.irq[1], level);
    }
}

/* ---- WD33C93 SCSI ----------------------------------------------------- */

static uint64_t sgi_ip6_scsi_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_SCSI_SIZE - 1);

    if (off < 0x100) {
        return wd33c93_addr_read(s->scsi);   /* address port: ASR */
    }
    return wd33c93_data_read(s->scsi);       /* data port */
}

static void sgi_ip6_scsi_write(void *opaque, hwaddr addr, uint64_t data,
                               unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_SCSI_SIZE - 1);

    if (off < 0x100) {
        wd33c93_addr_write(s->scsi, data & 0xff);
    } else {
        wd33c93_data_write(s->scsi, data & 0xff);
    }
}

static const MemoryRegionOps sgi_ip6_scsi_ops = {
    .read = sgi_ip6_scsi_read,
    .write = sgi_ip6_scsi_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * SCSI control lines: 0x1fa80004 asserts reset, 0x1fa80000 releases it.
 * The WD33C93 hardware reset raises the reset-status interrupt (MAME
 * wd33c9x device_reset pushes SCSI_STATUS_RESET), which the PROM waits for
 * by polling ASR bit 7.  QEMU's wd33c93 device reset leaves the interrupt
 * clear, so drive the chip's own reset command, which does set it.
 */
static uint64_t sgi_ip6_scsi_reset_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    SGIip6State *s = opaque;

    if ((addr & (SGI_IP6_SCSIRST_SIZE - 1)) == 0x4) {
        wd33c93_addr_write(s->scsi, WD_COMMAND);
        wd33c93_data_write(s->scsi, CMD_RESET);
    }
    return 0;
}

static void sgi_ip6_scsi_reset_write(void *opaque, hwaddr addr, uint64_t data,
                                     unsigned size)
{
}

static const MemoryRegionOps sgi_ip6_scsi_reset_ops = {
    .read = sgi_ip6_scsi_reset_read,
    .write = sgi_ip6_scsi_reset_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_ip6_scsi_irq(void *opaque, int n, int level)
{
    SGIip6State *s = opaque;

    /* LIO status bits are active low: set = idle, clear = pending. */
    if (level) {
        s->lio_isr &= ~(1u << LIO_SCSI);
    } else {
        s->lio_isr |= (1u << LIO_SCSI);
    }
    sgi_ip6_lio_update(s);
}

/*
 * WD33C93 data request.  The IP6 has no descriptor chain: the CTL1 supplies
 * a byte address as (dmahi[mapindex] << 12) | (dmalo & 0xfff), and dmalo bit
 * 15 selects the direction (set = device -> memory, i.e. a SCSI read).  Both
 * advance per byte; mapindex steps every 4 KB.  MAME services one byte per
 * DRQ; QEMU's WD33C93 exposes the transfer buffer directly, so we drain it.
 */
static void sgi_ip6_scsi_drq(void *opaque, int n, int level)
{
    SGIip6State *s = opaque;
    WD33C93State *wdc = s->scsi;

    if (!level || !wdc) {
        return;
    }

    while (wdc->async_len > 0) {
        uint32_t addr = ((uint32_t)s->dmahi[s->mapindex & 0x3ff] << 12) |
                        (s->dmalo & 0x0fff);

        if (s->dmalo & 0x8000) {
            /* Device -> memory (SCSI data-in). */
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, wdc->async_buf, 1);
        } else {
            /* Memory -> device (SCSI data-out). */
            address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, wdc->async_buf, 1);
        }
        wdc->async_buf++;
        wdc->async_len--;
        if (wdc->transfer_count > 0) {
            wdc->transfer_count--;
            wd33c93_set_transfer_count(wdc, wdc->transfer_count);
        }

        s->dmalo = (s->dmalo + 1) & 0x8fff;
        if (!(s->dmalo & 0xfff)) {
            s->mapindex++;
        }
    }

    /* Buffer drained: drop DRQ and let the chip continue or complete. */
    wd33c93_set_drq(wdc, false);
    if (wdc->current_req && wdc->transfer_count == 0 && wdc->pending_len > 0) {
        /*
         * Transfer count exhausted but the SCSI buffer is not fully consumed
         * (multi-pass DMA): raise unexpected-phase, as the IRIX driver
         * expects to reprogram TC and issue TRANSFER_INFO.  Left as a known
         * gap for the disk path; the PROM's early SCSI use does not hit it.
         */
        qemu_log_mask(LOG_UNIMP, "sgi-ip6: SCSI DMA TC=0 with %u bytes "
                      "pending\n", wdc->pending_len);
    } else if (wdc->current_req) {
        scsi_req_continue(wdc->current_req);
    }
}

/* ---- SCN2681 DUARTs --------------------------------------------------- */

static uint64_t sgi_ip6_duart_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    int chip = addr & 1;
    int reg = (addr >> 4) & 0xf;

    return scn2681_read(s->duart[chip], reg);
}

static void sgi_ip6_duart_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SGIip6State *s = opaque;
    int chip = addr & 1;
    int reg = (addr >> 4) & 0xf;

    scn2681_write(s->duart[chip], reg, data & 0xff);
}

static const MemoryRegionOps sgi_ip6_duart_ops = {
    .read = sgi_ip6_duart_read,
    .write = sgi_ip6_duart_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * ---- GR1 ("Eclipse") graphics: probe instrument ----------------------
 *
 * No registers are modelled yet.  Every access is traced with the guest PC
 * and the value returned, on the read path as well as the write path, so a
 * probe that still sees zero says so in the trace rather than leaving the
 * subsequent fault as the only evidence.  The response the firmware expects
 * is to be determined from its own use of the value, not invented here.
 */
/*
 * The upper bank (selected by mar_msb, which the firmware writes at 0x0e04)
 * holds the display registers.  MAME's map is authoritative:
 *   dr1 0x84c0  dr0 0x84e0  dr4 0x85a0  dr3 0x85c0  dr2 0x85e0
 * each a byte in lane 3 (umask32(0xff000000)), with the reset values below.
 * The RAMDAC at 0x8500..0x850f is what the firmware's presence probe actually
 * pokes (walking value in at 0x8500/0x8508 under bank 4), which is why the
 * probe window still has to return what was written.
 */
static const struct {
    uint16_t off;             /* bank-relative, lane 3 */
    uint8_t reset;
    uint8_t wmask, rmask;
} gr1_drs[5] = {
    { 0x84e0, 0x09, 0xf7, 0xff },   /* dr0: GRF1EN | SMALLMON0 */
    { 0x84c0, 0x08, 0xe7, 0xff },   /* dr1: TURBO */
    { 0x85e0, 0x00, 0xe7, 0xff },   /* dr2 */
    { 0x85c0, 0x00, 0xe7, 0xff },   /* dr3 */
    { 0x85a0, 0x08, 0xe7, 0x9f },   /* dr4: MEGOPT */
};

/*
 * Return the display-register index whose lane-3 byte this access touches,
 * given the effective (bank-relative) offset.  The DR offsets in gr1_drs are
 * already effective offsets, so they must not be re-based.
 */
static int gr1_dr_hit(uint32_t eff, unsigned size)
{
    int i;

    for (i = 0; i < 5; i++) {
        uint32_t b = gr1_drs[i].off + 3;

        if (b >= eff && b < eff + size) {
            return i;
        }
    }
    return -1;
}

static uint64_t sgi_ip6_gr1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t base = (uint32_t)s->gr1_bank * 0x2000;
    uint32_t eff = base + (uint32_t)addr;
    uint64_t val = 0;
    int i;

    i = gr1_dr_hit(eff, size);
    if (i >= 0) {
        /* Modelled register: returns its own value, not window contents. */
        val = (uint32_t)s->gr1_dr[i] & gr1_drs[i].rmask;
    } else if (eff >= 0x8000 && eff < 0xa000) {
        /*
         * Upper bank: the RAMDAC/cursor windows the firmware pokes.  These
         * return what was written (the presence probe's contract).
         */
        val = s->gr1_win[(eff - 0x8000) >> 2];
    }
    trace_sgi_ip6_gr1_read((uint32_t)s->cpu->env.active_tc.PC,
                           (uint32_t)addr, (uint32_t)val);
    return val;
}

static void sgi_ip6_gr1_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t base = (uint32_t)s->gr1_bank * 0x2000;
    uint32_t eff = base + (uint32_t)addr;
    int i;

    if (addr >= 0x0e00 && addr <= 0x0e07) {
        /* mar_msb: selects which 0x2000 bank the window shows. */
        s->gr1_bank = (uint8_t)addr & 7;
    }

    i = gr1_dr_hit(eff, size);
    if (i >= 0) {
        s->gr1_dr[i] = (s->gr1_dr[i] & ~gr1_drs[i].wmask)
                     | ((uint8_t)val & gr1_drs[i].wmask);
    } else if (eff >= 0x8000 && eff < 0xa000) {
        s->gr1_win[(eff - 0x8000) >> 2] = val;
    }
    trace_sgi_ip6_gr1_write((uint32_t)s->cpu->env.active_tc.PC,
                            (uint32_t)addr, (uint32_t)val);
}

static const MemoryRegionOps sgi_ip6_gr1_ops = {
    .read = sgi_ip6_gr1_read,
    .write = sgi_ip6_gr1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_ip6_duart_irq(void *opaque, int n, int level)
{
    SGIip6State *s = opaque;
    int bit = n & 1;   /* LIO_D0 = bit 0, LIO_D1 = bit 1 */

    /* LIO status bits are active low: set = idle, clear = pending. */
    if (level) {
        s->lio_isr &= ~(1u << bit);
    } else {
        s->lio_isr |= (1u << bit);
    }
    sgi_ip6_lio_update(s);
}

/* ---- 8254 PIT and timer interrupt acknowledge ------------------------- */

static void sgi_ip6_pit_out0(void *opaque, int n, int level)
{
    SGIip6State *s = opaque;

    /* The PROM wires the PIT outputs as set-only latches: a rising edge
     * asserts the CPU IRQ, and only the ack register clears it.  QEMU's PIT
     * leaves channel 0 toggling out of reset, so wait until the PROM has
     * actually programmed the chip before propagating edges. */
    if (level && !s->pit0_level && s->pit0_programmed && s->cpu) {
        qemu_set_irq(s->cpu->env.irq[2], 1);
    }
    s->pit0_level = level;
}

/*
 * The 8254 ports sit one per 32-bit bus word (+0/+4/+8/+12), not packed.
 * Forward each to the ISA PIT's byte ports, and note that software has
 * begun programming the chip so its reset state stops driving IRQ2.
 */
static uint64_t sgi_ip6_pit_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint64_t val = 0;
    unsigned port = (addr & 0xf) >> 2;

    if (addr & 3) {
        return 0;
    }
    memory_region_dispatch_read(&PIT_COMMON(s->pit)->ioports, port, &val,
                                size, MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void sgi_ip6_pit_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;
    unsigned port = (addr & 0xf) >> 2;

    if (addr & 3) {
        return;
    }
    s->pit0_programmed = true;
    memory_region_dispatch_write(&PIT_COMMON(s->pit)->ioports, port, data,
                                 size, MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps sgi_ip6_pit_ops = {
    .read = sgi_ip6_pit_read,
    .write = sgi_ip6_pit_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static uint64_t sgi_ip6_timer0_ack_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;

    if (s->cpu) {
        qemu_set_irq(s->cpu->env.irq[2], 0);
    }
    return 0;
}

static uint64_t sgi_ip6_timer1_ack_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;

    if (s->cpu) {
        qemu_set_irq(s->cpu->env.irq[4], 0);
    }
    return 0;
}

static void sgi_ip6_timer_ack_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size)
{
}

static const MemoryRegionOps sgi_ip6_timer0_ack_ops = {
    .read = sgi_ip6_timer0_ack_read,
    .write = sgi_ip6_timer_ack_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps sgi_ip6_timer1_ack_ops = {
    .read = sgi_ip6_timer1_ack_read,
    .write = sgi_ip6_timer_ack_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- Am7990 LANCE (Ethernet) ----------------------------------------- */

#define TYPE_SGI_IP6_LANCE "sgi-ip6-lance"
OBJECT_DECLARE_SIMPLE_TYPE(SGIip6LanceState, SGI_IP6_LANCE)

struct SGIip6LanceState {
    DeviceState parent_obj;
    PCNetState net;
};

/*
 * The LANCE DMAs through the CTL1 address-mapping table: entries 0x200..0x2ff
 * map the LANCE's 4 KB pages, so the physical address is
 * (dmahi[0x200 + ((addr >> 12) & 0xff)] << 12) | (addr & 0xfff).  Each 16-bit
 * word is translated separately, as MAME does.  When the driver leaves BSWP
 * clear the LANCE's byte order differs from the host's, so swap in that case.
 *
 * The map is a 32-bit-spaced array of 16-bit entries (the PROM stores each
 * entry in the low half of a 32-bit slot, so a 16-bit-half index would land on
 * every other entry); sgi_ip6_dmahi_read/write index by the 32-bit word, which
 * is why the LANCE base is word 0x200 and the SCSI base is word 0.
 */
static void sgi_ip6_lance_dma_read(void *opaque, hwaddr addr, uint8_t *buf,
                                   int len, int do_bswap)
{
    SGIip6State *s = &ip6_state;
    int i;

    for (i = 0; i + 1 < len; i += 2) {
        hwaddr a = addr + i;
        unsigned page = 0x200 + ((a >> 12) & 0xff);
        hwaddr pa = ((hwaddr)s->dmahi[page & 0x3ff] << 12) | (a & 0xfff);

        address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                           buf + i, 2);
    }
    if (!do_bswap) {
        for (i = 0; i + 1 < len; i += 2) {
            bswap16s((uint16_t *)(buf + i));
        }
    }
}

static void sgi_ip6_lance_dma_write(void *opaque, hwaddr addr, uint8_t *buf,
                                    int len, int do_bswap)
{
    SGIip6State *s = &ip6_state;
    int i;

    if (!do_bswap) {
        for (i = 0; i + 1 < len; i += 2) {
            bswap16s((uint16_t *)(buf + i));
        }
    }
    for (i = 0; i + 1 < len; i += 2) {
        hwaddr a = addr + i;
        unsigned page = 0x200 + ((a >> 12) & 0xff);
        hwaddr pa = ((hwaddr)s->dmahi[page & 0x3ff] << 12) | (a & 0xfff);

        address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                            buf + i, 2);
    }
}

static void sgi_ip6_lance_irq(void *opaque, int n, int level)
{
    SGIip6State *s = opaque;

    /* LIO status bits are active low: set = idle, clear = pending. */
    if (level) {
        s->lio_isr &= ~(1u << LIO_ENET);
    } else {
        s->lio_isr |= (1u << LIO_ENET);
    }
    sgi_ip6_lio_update(s);
}

static ssize_t sgi_ip6_lance_receive(NetClientState *nc, const uint8_t *buf,
                                     size_t size)
{
    /*
     * Real Ethernet pads frames shorter than the 60-byte minimum at the PHY,
     * not in the LANCE, and the PROM's driver rejects short frames
     * ("receive packet too small").  Pad here as the wire would, so senders
     * that emit raw short frames (e.g. a hand-written ARP reply) still look
     * like a real link to the guest.
     */
    uint8_t pad[60] = { 0 };

    if (size < sizeof(pad)) {
        memcpy(pad, buf, size);
        buf = pad;
        size = sizeof(pad);
    }

    return pcnet_receive(nc, buf, size);
}

static NetClientInfo net_sgi_ip6_lance_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = sgi_ip6_lance_receive,
    .link_status_changed = pcnet_set_link_status,
};

static void sgi_ip6_lance_realize(DeviceState *dev, Error **errp)
{
    SGIip6LanceState *l = SGI_IP6_LANCE(dev);
    PCNetState *s = &l->net;

    s->phys_mem_read = sgi_ip6_lance_dma_read;
    s->phys_mem_write = sgi_ip6_lance_dma_write;
    s->irq = qemu_allocate_irq(sgi_ip6_lance_irq, &ip6_state, 0);

    pcnet_common_init(dev, s, &net_sgi_ip6_lance_info);
}

static void sgi_ip6_lance_reset(DeviceState *dev)
{
    pcnet_h_reset(&SGI_IP6_LANCE(dev)->net);
}

static const Property sgi_ip6_lance_props[] = {
    DEFINE_NIC_PROPERTIES(SGIip6LanceState, net.conf),
};

static void sgi_ip6_lance_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_ip6_lance_realize;
    device_class_set_legacy_reset(dc, sgi_ip6_lance_reset);
    device_class_set_props(dc, sgi_ip6_lance_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo sgi_ip6_lance_type = {
    .name = TYPE_SGI_IP6_LANCE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SGIip6LanceState),
    .class_init = sgi_ip6_lance_class_init,
};

static void sgi_ip6_lance_register_types(void)
{
    type_register_static(&sgi_ip6_lance_type);
}

type_init(sgi_ip6_lance_register_types)

/*
 * Am7990 register access: RDP at +0x000 and RAP at +0x100 (MAME maps the
 * block 0x1f950000-0x1f9501ff), with the 16-bit register in the high bus
 * lane.  Translate to the PCnet ioport model's RDP (0x00) / RAP (0x02).
 */
static unsigned sgi_ip6_lance_port(hwaddr addr)
{
    return (addr & 0x100) ? 0x02 : 0x00;
}

static uint64_t sgi_ip6_lance_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t v = pcnet_ioport_readw(s->lance, sgi_ip6_lance_port(addr));

    if (size == 1) {
        v = (addr & 1) ? (v & 0xff) : ((v >> 8) & 0xff);
    }
    return v;
}

static void sgi_ip6_lance_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SGIip6State *s = opaque;
    unsigned port = sgi_ip6_lance_port(addr);
    uint32_t v = data & 0xffff;

    if (size == 1) {
        uint16_t cur = pcnet_ioport_readw(s->lance, port);

        if (addr & 1) {
            v = (cur & 0xff00) | (data & 0xff);
        } else {
            v = (cur & 0x00ff) | ((data & 0xff) << 8);
        }
    }
    pcnet_ioport_writew(s->lance, port, v);
}

static const MemoryRegionOps sgi_ip6_lance_ops = {
    .read = sgi_ip6_lance_read,
    .write = sgi_ip6_lance_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

/*
 * Ethernet reset lines: reading 0x1f960000 asserts hardware reset and reading
 * 0x1f960004 releases it (MAME ip6.cpp etherrdy/etherrst).
 */
static uint64_t sgi_ip6_lance_reset_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    SGIip6State *s = opaque;

    if ((addr & (SGI_IP6_LANCE_RST_SIZE - 1)) == 0) {
        pcnet_h_reset(s->lance);
    }
    return 0;
}

static void sgi_ip6_lance_reset_write(void *opaque, hwaddr addr, uint64_t data,
                                      unsigned size)
{
}

static const MemoryRegionOps sgi_ip6_lance_reset_ops = {
    .read = sgi_ip6_lance_reset_read,
    .write = sgi_ip6_lance_reset_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- machine ---------------------------------------------------------- */

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static void sgi_ip6_init(MachineState *machine)
{
    SGIip6State *s = &ip6_state;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    Clock *cpuclk;
    char *filename;
    int bios_size;
    unsigned ram_mb = machine->ram_size / MiB;
    uint8_t memcfg;
    unsigned ai;

    if (machine->ram_size > SGI_IP6_RAM_MAX) {
        error_report("RAM size more than 256MB is not supported");
        exit(EXIT_FAILURE);
    }

    if (strcmp(machine->cpu_type, MIPS_CPU_TYPE_NAME("R2000")) != 0 &&
        strcmp(machine->cpu_type, MIPS_CPU_TYPE_NAME("R3000")) != 0) {
        warn_report("IP6 expects an R2000 or R3000 CPU");
    }

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 20000000); /* 4D/25: R3000 at 20 MHz */

    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    qemu_register_reset(main_cpu_reset, cpu);
    s->cpu = cpu;

    /*
     * CTL1 memory decode, as MAME models it: a RAM window whose covered
     * banks are aliases of the host RAM, with a read-0/write-ignore
     * placeholder for everything else (MAME's noprw).  machine->ram is NOT
     * mapped flat - sgi_ip6_ram_decode() installs the guest-configured
     * geometry, so the PROM/IDE sizing probe sees the real per-bank decode
     * (and no bus error above installed RAM) instead of a flat alias.
     */
    s->ram_size = machine->ram_size;
    memory_region_init(&s->ram_win, OBJECT(machine), "sgi-ip6-ramwin",
                       SGI_IP6_RAM_WINDOW);
    memory_region_init_io(&s->ram_zero, OBJECT(machine), &sgi_ip6_ram_zero_ops,
                          s, "sgi-ip6-ramzero", SGI_IP6_RAM_WINDOW);
    memory_region_add_subregion_overlap(&s->ram_win, 0, &s->ram_zero, -1);
    for (ai = 0; ai < SGI_IP6_RAM_ALIAS_MAX; ai++) {
        char name[32];

        snprintf(name, sizeof(name), "sgi-ip6-bank%u", ai);
        memory_region_init_alias(&s->bank_alias[ai], OBJECT(machine), name,
                                 machine->ram, 0,
                                 MIN((uint64_t)SGI_IP6_HOST_SIMM_SIZE,
                                     s->ram_size));
    }
    memory_region_add_subregion(system_memory, 0, &s->ram_win);
    sgi_ip6_ram_decode(s);

    /* Boot PROM at 0x1fc00000 */
    memory_region_init_rom(prom, NULL, "sgi-ip6.prom", SGI_IP6_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, SGI_IP6_PROM_BASE, prom);

    if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                                  "4D25_ip6prom.070-8000-007.bin");
        if (!filename) {
            filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                                      "ip10prom.007BE.bin");
        }
        if (!filename) {
            warn_report("No IP6 PROM specified; use -bios");
        }
    }
    if (filename) {
        bios_size = load_image_targphys(filename, SGI_IP6_PROM_BASE,
                                        SGI_IP6_PROM_SIZE, NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load PROM image");
            exit(EXIT_FAILURE);
        }
    }

    /*
     * 93C56 serial EEPROM (128 x 16-bit words).  The PROM reads its system
     * configuration - including the Ethernet address - from it: config bytes
     * 122..127 (words 61..63) are the eaddr, fetched word-at-a-time by the
     * PROM reader at 0xbfc13490.  A blank EEPROM makes the PROM report
     * "bad ethernet address 0:0:0:0:0:0" and refuse to netboot, so seed a
     * factory-style address.
     * [ASSUMPTION] word offsets and byte order inferred from the PROM read
     * path, not from a documented IP6 EEPROM layout; revisit if a real dump
     * or the SAIO PROM source appears.
     */
    s->eeprom = eeprom93xx_new(DEVICE(cpu), 128);
    {
        uint16_t *ee = eeprom93xx_data(s->eeprom);

        ee[61] = 0x0800;
        ee[62] = 0x6912;
        ee[63] = 0x3456;
    }

    memory_region_init_io(&s->ctl1, OBJECT(machine), &sgi_ip6_ctl1_ops, s,
                          "sgi-ip6-ctl1", SGI_IP6_CTL1_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_CTL1_BASE, &s->ctl1);

    memory_region_init_io(&s->dmalo_reg, OBJECT(machine), &sgi_ip6_dmalo_ops, s,
                          "sgi-ip6-dmalo", 4);
    memory_region_add_subregion(system_memory, SGI_IP6_DMALO_BASE, &s->dmalo_reg);

    memory_region_init_io(&s->mapindex_reg, OBJECT(machine), &sgi_ip6_mapindex_ops,
                          s, "sgi-ip6-mapindex", 4);
    memory_region_add_subregion(system_memory, SGI_IP6_MAPINDEX_BASE,
                                &s->mapindex_reg);

    memory_region_init_io(&s->dmahi_reg, OBJECT(machine), &sgi_ip6_dmahi_ops, s,
                          "sgi-ip6-dmahi", SGI_IP6_DMAHI_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_DMAHI_BASE, &s->dmahi_reg);

    memory_region_init_io(&s->lio, OBJECT(machine), &sgi_ip6_lio_ops, s,
                          "sgi-ip6-lio", SGI_IP6_LIO_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_LIO_BASE, &s->lio);

    memory_region_init_io(&s->err, OBJECT(machine), &sgi_ip6_err_ops, s,
                          "sgi-ip6-err", SGI_IP6_ERR_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_ERR_BASE, &s->err);

    memory_region_init_io(&s->clrerr, OBJECT(machine), &sgi_ip6_clrerr_ops, s,
                          "sgi-ip6-clrerr", SGI_IP6_CLRERR_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_CLRERR_BASE,
                                &s->clrerr);

    memory_region_init_io(&s->rtc, OBJECT(machine), &sgi_ip6_rtc_ops, s,
                          "sgi-ip6-rtc", SGI_IP6_RTC_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_RTC_BASE, &s->rtc);

    /* WD33C93 SCSI controller. */
    s->scsi = WD33C93(qdev_new(TYPE_WD33C93));
    qdev_realize(DEVICE(s->scsi), NULL, &error_fatal);
    scsi_bus_legacy_handle_cmdline(&s->scsi->bus);
    qdev_connect_gpio_out_named(DEVICE(s->scsi), "irq", 0,
                                qemu_allocate_irq(sgi_ip6_scsi_irq, s, 0));
    qdev_connect_gpio_out_named(DEVICE(s->scsi), "drq", 0,
                                qemu_allocate_irq(sgi_ip6_scsi_drq, s, 0));

    memory_region_init_io(&s->scsi_regs, OBJECT(machine), &sgi_ip6_scsi_ops,
                          s, "sgi-ip6-scsi", SGI_IP6_SCSI_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_SCSI_BASE,
                                &s->scsi_regs);

    memory_region_init_io(&s->scsi_reset, OBJECT(machine),
                          &sgi_ip6_scsi_reset_ops, s, "sgi-ip6-scsirst",
                          SGI_IP6_SCSIRST_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_SCSIRST_BASE,
                                &s->scsi_reset);

    /* IP6 mouse: Mouse Systems peer on DUART0-B (serial chardevs unused). */
    s->mouse = SGI_IP6_INPUT(qdev_new(TYPE_SGI_IP6_INPUT));
    qdev_realize(DEVICE(s->mouse), NULL, &error_fatal);

    /* Two SCN2681 DUARTs: 0 = keyboard/mouse, 1 = serial ports. */
    for (int i = 0; i < 2; i++) {
        int ch;

        s->duart[i] = SGI_SCN2681(qdev_new(TYPE_SGI_SCN2681));
        for (ch = 0; ch < 2; ch++) {
            Chardev *chr = serial_hd(i * 2 + ch);

            if (i == 0 && ch == 1) {
                chr = sgi_ip6_input_chardev(s->mouse);
            }
            if (chr) {
                qdev_prop_set_chr(DEVICE(s->duart[i]),
                                  ch ? "chardev-b" : "chardev-a", chr);
            }
        }
        qdev_realize(DEVICE(s->duart[i]), NULL, &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(s->duart[i]), "irq", 0,
                                    qemu_allocate_irq(sgi_ip6_duart_irq, s,
                                                      i));
    }

    memory_region_init_io(&s->duart_regs, OBJECT(machine),
                          &sgi_ip6_duart_ops, s, "sgi-ip6-duart",
                          SGI_IP6_DUART_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_DUART_BASE,
                                &s->duart_regs);

    /* Am7990 LANCE: PCnet-backed NIC whose DMA goes through the CTL1. */
    s->lance_dev = qdev_new(TYPE_SGI_IP6_LANCE);
    qemu_configure_nic_device(s->lance_dev, true, NULL);
    qdev_realize(s->lance_dev, NULL, &error_fatal);
    s->lance = &SGI_IP6_LANCE(s->lance_dev)->net;

    memory_region_init_io(&s->lance_regs, OBJECT(machine), &sgi_ip6_lance_ops,
                          s, "sgi-ip6-lance", SGI_IP6_LANCE_REG_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_LANCE_BASE,
                                &s->lance_regs);

    memory_region_init_io(&s->lance_reset, OBJECT(machine),
                          &sgi_ip6_lance_reset_ops, s, "sgi-ip6-lance-rst",
                          SGI_IP6_LANCE_RST_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_LANCE_RST_BASE,
                                &s->lance_reset);

    /* 8254 PIT at 0x1fb40000.  Its four byte-wide ports sit one per 32-bit
     * bus word (+0/+4/+8/+12); channel 0's output drives CPU IRQ2, which the
     * PROM acknowledges by reading the timer0 ack register. */
    s->isa = isa_bus_new(NULL, get_system_memory(), get_system_io(),
                         &error_fatal);
    s->pit = DEVICE(isa_create_simple(s->isa, TYPE_I8254));
    qdev_connect_gpio_out(s->pit, 0,
                          qemu_allocate_irq(sgi_ip6_pit_out0, s, 0));
    memory_region_init_io(&s->pit_reg, OBJECT(machine), &sgi_ip6_pit_ops, s,
                          "sgi-ip6-pit", 0x10);
    memory_region_add_subregion(system_memory, SGI_IP6_PIT_BASE, &s->pit_reg);

    memory_region_init_io(&s->timer0_ack, OBJECT(machine),
                          &sgi_ip6_timer0_ack_ops, s, "sgi-ip6-timer0-ack", 4);
    memory_region_add_subregion_overlap(system_memory, SGI_IP6_TIMER0_ACK,
                                        &s->timer0_ack, 1);
    memory_region_init_io(&s->timer1_ack, OBJECT(machine),
                          &sgi_ip6_timer1_ack_ops, s, "sgi-ip6-timer1-ack", 4);
    memory_region_add_subregion_overlap(system_memory, SGI_IP6_TIMER1_ACK,
                                        &s->timer1_ack, 1);

    /* Devices not yet implemented: LANCE and GR1 graphics.  Map them
     * as unimplemented so accesses are logged rather than aborting. */
    create_unimplemented_device("sgi-ip6-timer", 0x1fa00000, 0x30000);
    create_unimplemented_device("sgi-ip6-vrrst", 0x1fac0000, 0x4);
    memory_region_init_io(&s->gr1_regs, OBJECT(machine), &sgi_ip6_gr1_ops, s,
                          "sgi-ip6-gr1", 0x8000);
    memory_region_add_subregion(system_memory, 0x1f000000, &s->gr1_regs);

    /* GR1 display-register reset values (MAME sgi_gr1_device::device_reset). */
    s->gr1_bank = 0;
    s->gr1_dr[0] = 0x09;
    s->gr1_dr[1] = 0x08;
    s->gr1_dr[2] = 0x00;
    s->gr1_dr[3] = 0x00;
    s->gr1_dr[4] = 0x08;
    create_unimplemented_device("sgi-ip6-audio", 0x1f9c0000, 0x40000);
    create_unimplemented_device("sgi-ip6-dmaflush", 0x1f940000, 0x1000);
    create_unimplemented_device("sgi-ip6-gio", 0x1f400000, 0x400000);
    create_unimplemented_device("sgi-ip6-vme", 0x1fa60000, 0x20000);

    /* memcfg defaults to the populated banks (4 MB SIMMs). */
    if (ram_mb >= 64) {
        memcfg = ((ram_mb / 16) - 1) & MEMCFG_MEMSIZE;
        memcfg |= MEMCFG_4MRAM;
    } else {
        memcfg = ((ram_mb / 4) - 1) & MEMCFG_MEMSIZE;
    }
    s->memcfg = memcfg;
    s->cpucfg = 0;
    s->cpuauxctl = 0;
    s->dmalo = 0;
    s->mapindex = 0;
    memset(s->dmahi, 0, sizeof(s->dmahi));
    s->erradr = 0;
    s->refadr = 0;
    s->ref_load_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->vme_isr = 0;
    s->vme_imr = 0;
    memset(s->rtc_regs, 0, sizeof(s->rtc_regs));
    s->rtc_regs[1] = 0x18;   /* status: clock running + RAM/time valid */
    s->rtc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgi_ip6_rtc_tick, s);
    timer_mod(s->rtc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / 100);
    s->lio_int = false;
    s->pit0_level = true;   /* PIT output is high out of reset; see out0 */
    s->pit0_programmed = false;
    s->lio_isr = 0x3ff;
    s->lio_imr = 0;
}

static void sgi_ip6_machine_init(MachineClass *mc)
{
    mc->desc = "SGI Personal IRIS 4D/25 (IP6, R3000)";
    mc->init = sgi_ip6_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R3000");
    mc->default_ram_size = 16 * MiB;
    mc->default_ram_id = "sgi-ip6.ram";
    mc->max_cpus = 1;
}

DEFINE_MACHINE("pi4d25", sgi_ip6_machine_init)
