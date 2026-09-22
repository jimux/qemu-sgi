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
#include "hw/core/loader.h"
#include "hw/mips/mips.h"
#include "hw/misc/unimp.h"
#include "hw/nvram/eeprom93xx.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"

#define SGI_IP6_PROM_BASE   0x1fc00000ULL
#define SGI_IP6_PROM_SIZE   (256 * KiB)
#define SGI_IP6_RAM_MAX     (256 * MiB)

/* CTL1 control register block */
#define SGI_IP6_CTL1_BASE   0x1f800000ULL
#define SGI_IP6_CTL1_SIZE   0x100000
#define SGI_IP6_CTL1_MEMCFG 0x00000
#define SGI_IP6_CTL1_CPUCFG 0x80000
#define SGI_IP6_CTL1_CPUAUX 0xe0000

/* CTL1 DMA address-mapping table */
#define SGI_IP6_DMA_BASE    0x1f900000ULL
#define SGI_IP6_DMA_SIZE    0x3000

/* LIO interrupt controller */
#define SGI_IP6_LIO_BASE    0x1f980000ULL
#define SGI_IP6_LIO_SIZE    0x10

/* CTL1 error / refresh / graphics-DMA registers */
#define SGI_IP6_ERR_BASE    0x1fa40000ULL
#define SGI_IP6_ERR_SIZE    0x20
#define SGI_IP6_CLRERR_BASE 0x1faa0000ULL
#define SGI_IP6_CLRERR_SIZE 0x8

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

typedef struct SGIip6State {
    MemoryRegion ctl1;
    MemoryRegion dma;
    MemoryRegion lio;
    MemoryRegion err;
    MemoryRegion clrerr;

    eeprom_t *eeprom;

    uint8_t memcfg;
    uint16_t cpucfg;
    uint8_t cpuauxctl;

    uint16_t dmalo;
    uint8_t mapindex;
    uint16_t dmahi[1024];

    uint32_t erradr;
    uint32_t refadr;

    uint16_t lio_isr;
    uint8_t lio_imr;
} SGIip6State;

static SGIip6State ip6_state;

/* ---- CTL1 control registers ------------------------------------------ */

static uint64_t sgi_ip6_ctl1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = (addr & (SGI_IP6_CTL1_SIZE - 1)) & ~3u;
    uint32_t val = 0;

    switch (off) {
    case SGI_IP6_CTL1_MEMCFG: {
        /* memcfg in the top byte lane, sysid in the next.  sysid bit1
         * (SYSID_FPPRES) reads as 1 here, which is the state the 4D/25
         * PROM expects on a normal power-up (it takes the FPU-init/reset
         * path otherwise). */
        uint8_t sysid = (s->eeprom ? eeprom93xx_read(s->eeprom) : 0)
                        | SYSID_FPPRES;

        val = ((uint32_t)s->memcfg << 24) | ((uint32_t)sysid << 16);
        break;
    }
    case SGI_IP6_CTL1_CPUCFG:
        val = s->cpucfg;
        break;
    case SGI_IP6_CTL1_CPUAUX:
        val = (uint32_t)s->cpuauxctl << 24;
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

/* ---- CTL1 DMA address-mapping table ---------------------------------- */

static uint64_t sgi_ip6_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_DMA_SIZE - 1);

    if (off == 0) {
        return s->dmalo;
    } else if (off == 0x1000) {
        return s->mapindex;
    } else if (off >= 0x2000) {
        return s->dmahi[(off - 0x2000) >> 1];
    }
    return 0;
}

static void sgi_ip6_dma_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;
    uint32_t off = addr & (SGI_IP6_DMA_SIZE - 1);

    if (off == 0) {
        s->dmalo = data & 0xffff;
        s->mapindex = 0;
    } else if (off == 0x1000) {
        s->mapindex = data & 0xff;
    } else if (off >= 0x2000) {
        s->dmahi[(off - 0x2000) >> 1] = data & 0xffff;
    }
}

static const MemoryRegionOps sgi_ip6_dma_ops = {
    .read = sgi_ip6_dma_read,
    .write = sgi_ip6_dma_write,
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

    if ((addr & 0xf) == 0) {
        return s->lio_isr;
    } else if ((addr & 0xf) == 8) {
        return s->lio_imr;
    }
    return 0;
}

static void sgi_ip6_lio_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SGIip6State *s = opaque;

    if ((addr & 0xf) == 8) {
        s->lio_imr = data & 0xff;
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

    /* Flat RAM at physical 0 (KSEG0/KSEG1 map onto it). The CTL1's per-bank
     * mapping is approximated by mapping all RAM. */
    memory_region_add_subregion(system_memory, 0, machine->ram);

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

    /* 93C56 serial EEPROM (128 x 16-bit words), blank by default.  The
     * eeprom93xx helper wants a DeviceState owner for vmstate; the CPU is
     * the only one this machine has so far. */
    s->eeprom = eeprom93xx_new(DEVICE(cpu), 128);

    memory_region_init_io(&s->ctl1, OBJECT(machine), &sgi_ip6_ctl1_ops, s,
                          "sgi-ip6-ctl1", SGI_IP6_CTL1_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_CTL1_BASE, &s->ctl1);

    memory_region_init_io(&s->dma, OBJECT(machine), &sgi_ip6_dma_ops, s,
                          "sgi-ip6-dma", SGI_IP6_DMA_SIZE);
    memory_region_add_subregion(system_memory, SGI_IP6_DMA_BASE, &s->dma);

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

    /* Devices not yet implemented: PIT, SCSI (WD33C93), DUARTs (SCN2681),
     * RTC (DP8572A), LANCE and GR1 graphics.  Map them as unimplemented so
     * accesses are logged rather than aborting. */
    create_unimplemented_device("sgi-ip6-pit", 0x1fb40000, 0x10);
    create_unimplemented_device("sgi-ip6-scsi", 0x1fb00000, 0x800);
    create_unimplemented_device("sgi-ip6-scsictl", 0x1fa80000, 0x10);
    create_unimplemented_device("sgi-ip6-timer", 0x1fa00000, 0x30000);
    create_unimplemented_device("sgi-ip6-vrrst", 0x1fac0000, 0x4);
    create_unimplemented_device("sgi-ip6-duart", 0x1fb80000, 0x100);
    create_unimplemented_device("sgi-ip6-rtc", 0x1fbc0000, 0x80);
    create_unimplemented_device("sgi-ip6-lance", 0x1f950000, 0x20000);
    create_unimplemented_device("sgi-ip6-gr1", 0x1f000000, 0x8000);

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
