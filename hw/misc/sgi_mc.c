/*
 * SGI Memory Controller (MC) emulation
 *
 * The MC is the central memory controller for SGI Indy (IP24) and
 * Indigo2 (IP22) workstations. It handles:
 *   - Memory configuration and timing
 *   - DMA address translation (TLB)
 *   - RPSS counter (high-resolution timer)
 *   - CPU/GIO bus error detection
 *   - Watchdog timer
 *   - EEPROM interface
 *
 * References:
 *   - MAME src/mame/sgi/mc.cpp
 *   - Linux arch/mips/include/asm/sgi/mc.h
 *   - IRIX kernel source irix/kern/
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/sgi_mc.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "trace.h"

/* SEG0 base address for 512KB alias */
#define SEG0_BASE         0x08000000
#define SEG0_ALIAS_BASE   0x00000000
#define SEG0_ALIAS_SIZE   (512 * 1024)

/* MC System ID values */
#define MC_SYSID_REV_MASK    0x0f   /* MC revision */
#define MC_SYSID_REV_C       0x03   /* Revision C (most common) */
#define MC_SYSID_EISA_MASK   0x10   /* EISA present bit */

/* MEMCFG register format (per 16-bit bank field) */
#define MEMCFG_ADDR_MASK     0x00ff  /* Base address (physical >> 22) */
#define MEMCFG_SIZE_MASK     0x1f00  /* Size mask (shifted by 8) */
#define MEMCFG_VLD           0x2000  /* Bank valid bit */
#define MEMCFG_BNK           0x4000  /* 2 subbanks */

/* Size codes: (code + 1) * unit_size = bank size */
#define MEMCFG_4MB           0x0000  /* 4 MB */
#define MEMCFG_8MB           0x0100  /* 8 MB */
#define MEMCFG_16MB          0x0300  /* 16 MB */
#define MEMCFG_32MB          0x0700  /* 32 MB */
#define MEMCFG_64MB          0x0f00  /* 64 MB */
#define MEMCFG_128MB         0x1f00  /* 128 MB */

/*
 * MC rev >= 5 uses 16MB memory units (shift 24).
 * MC rev < 5 uses 4MB memory units (shift 22).
 * Source: IRIX stand/arcs/IP22prom/lmem_conf.s setup_regs
 */
static int sgi_mc_addr_shift(SGIMCState *s)
{
    return (s->revision >= 5) ? 24 : 22;
}

/*
 * Unmap a single bank's memory regions (alias + low alias).
 */
static void sgi_mc_unmap_bank(SGIMCState *s, int bank_idx)
{
    SGIMCBankState *bank = &s->banks[bank_idx];

    if (bank->mapped && bank->region) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mc: unmapping bank %d RAM at 0x%08x\n",
                      bank_idx, bank->mapped_base);
        memory_region_del_subregion(s->system_memory, bank->region);
        object_unparent(OBJECT(bank->region));
        g_free(bank->region);
        bank->region = NULL;
    }

    if (bank->low_alias) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi_mc: unmapping bank %d low alias at 0x0\n",
                      bank_idx);
        memory_region_del_subregion(s->system_memory, bank->low_alias);
        object_unparent(OBJECT(bank->low_alias));
        g_free(bank->low_alias);
        bank->low_alias = NULL;
    }

    for (int w = 0; w < bank->num_wrap_aliases; w++) {
        if (bank->wrap_aliases[w]) {
            memory_region_del_subregion(s->system_memory,
                                        bank->wrap_aliases[w]);
            object_unparent(OBJECT(bank->wrap_aliases[w]));
            g_free(bank->wrap_aliases[w]);
        }
    }
    g_free(bank->wrap_aliases);
    bank->wrap_aliases = NULL;
    bank->num_wrap_aliases = 0;

    bank->mapped = false;
    bank->mapped_base = 0;
}

/*
 * Extract bank configuration from a 16-bit MEMCFG field.
 */
static void sgi_mc_extract_bank_config(uint16_t field, bool *valid,
                                        uint32_t *base, uint32_t *size,
                                        int addr_shift)
{
    *valid = (field >> 13) & 1;
    *base = (field & MEMCFG_ADDR_MASK) << addr_shift;
    *size = (((field >> 8) & 0x1f) + 1) << addr_shift;
}

/*
 * Update RAM mapping based on MEMCFG configuration.
 *
 * Like MAME's mc.cpp memcfg_w(), this unmaps and remaps ALL 4 banks
 * on every MEMCFG write. Each bank gets its own MemoryRegion alias
 * pointing to its portion of the machine's RAM (at ram_offset).
 *
 * Banks are mapped with priority 1 so they overlay the unimplemented
 * device regions (priority -1000) used for probe coverage.
 *
 * When a bank maps to SEG0 (0x08000000), a 512KB alias is also
 * created at 0x00000000 (matching MAME behavior).
 */
static void sgi_mc_update_ram_mapping(SGIMCState *s)
{
    int i;
    uint16_t fields[MC_NUM_BANKS];
    int addr_shift = sgi_mc_addr_shift(s);
    uint32_t seg0_base = (addr_shift >= 24) ? 0x20000000 : 0x08000000;

    /* Extract all 4 bank fields from MEMCFG0 and MEMCFG1 */
    fields[0] = (s->memcfg[0] >> 16) & 0xffff;  /* MEMCFG0 upper */
    fields[1] = s->memcfg[0] & 0xffff;           /* MEMCFG0 lower */
    fields[2] = (s->memcfg[1] >> 16) & 0xffff;   /* MEMCFG1 upper */
    fields[3] = s->memcfg[1] & 0xffff;           /* MEMCFG1 lower */

    /* Unmap all banks first */
    for (i = 0; i < MC_NUM_BANKS; i++) {
        sgi_mc_unmap_bank(s, i);
    }

    /* Remap each bank based on current MEMCFG */
    for (i = 0; i < MC_NUM_BANKS; i++) {
        SGIMCBankState *bank = &s->banks[i];
        bool valid;
        uint32_t base, cfg_size, map_size;
        char name[32];

        sgi_mc_extract_bank_config(fields[i], &valid, &base, &cfg_size,
                                    addr_shift);

        /* Only map if valid, bank has physical RAM, and we have RAM backend */
        if (!valid || bank->installed_size == 0 || s->ram == NULL) {
            continue;
        }

        /* Use the smaller of configured size and installed size */
        map_size = MIN(cfg_size, bank->installed_size);

        trace_sgi_mc_memcfg(i, fields[i], base / (1024 * 1024),
                            map_size / (1024 * 1024));

        /* Create alias into the machine's RAM at the bank's offset */
        bank->region = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "mc.ram.bank%d", i);
        memory_region_init_alias(bank->region, OBJECT(s), name,
                                 s->ram, bank->ram_offset, map_size);
        memory_region_add_subregion_overlap(s->system_memory, base,
                                            bank->region, 1);
        bank->mapped = true;
        bank->mapped_base = base;

        /*
         * If bank maps to SEG0 base, create a 512KB alias at physical
         * address 0x0. This matches MAME behavior and is needed for
         * exception vectors and early boot code.
         * SEG0 is 0x08000000 for MC rev < 5, 0x20000000 for rev >= 5.
         */
        if (base == seg0_base) {
            uint32_t alias_size = MIN(map_size, SEG0_ALIAS_SIZE);
            bank->low_alias = g_new(MemoryRegion, 1);
            snprintf(name, sizeof(name), "mc.ram.bank%d.low", i);
            memory_region_init_alias(bank->low_alias, OBJECT(s), name,
                                     s->ram, bank->ram_offset, alias_size);
            memory_region_add_subregion_overlap(s->system_memory,
                                                SEG0_ALIAS_BASE,
                                                bank->low_alias, 1);
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mc: bank %d: 512KB alias at 0x0\n", i);
        }

        /*
         * SIMM address wrapping for MC rev >= 5 (IP28):
         *
         * When the configured size exceeds the physical SIMM size,
         * create wrap aliases so accesses beyond the physical RAM
         * wrap back (modulo installed_size). The IP28 PROM's memory
         * sizing algorithm depends on this: it configures a large
         * size, writes test patterns at progressively larger offsets,
         * and checks whether they alias back to the base address.
         *
         * For MC rev < 5 (IP22/IP24), the PROM's data test instead
         * relies on reads beyond the SIMM returning 0 (from the
         * unimplemented device probe stubs). Enabling wrap aliases
         * for these older PROMs causes the BNK subbank test to see
         * aliased data and incorrectly detect 2 subbanks.
         *
         * MAME reference: mc.cpp uses a mirror mask with install_ram()
         * for all revisions, but MAME's PROM/SIMM configuration avoids
         * the mismatch we see here.
         */
        if (addr_shift >= 24 && cfg_size > map_size && map_size > 0) {
            int num_wraps = (cfg_size / map_size) - 1;
            bank->wrap_aliases = g_new0(MemoryRegion *, num_wraps);
            bank->num_wrap_aliases = num_wraps;
            for (int w = 0; w < num_wraps; w++) {
                uint32_t wrap_offset = (w + 1) * map_size;
                uint32_t wrap_size = MIN(map_size, cfg_size - wrap_offset);
                bank->wrap_aliases[w] = g_new(MemoryRegion, 1);
                snprintf(name, sizeof(name), "mc.ram.bank%d.wrap%d", i, w);
                memory_region_init_alias(bank->wrap_aliases[w], OBJECT(s),
                                         name, s->ram, bank->ram_offset,
                                         wrap_size);
                memory_region_add_subregion_overlap(s->system_memory,
                                                    base + wrap_offset,
                                                    bank->wrap_aliases[w], 1);
            }
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mc: bank %d: %d wrap aliases "
                          "(cfg=%u MB, physical=%u MB)\n",
                          i, num_wraps,
                          cfg_size / (1024 * 1024),
                          map_size / (1024 * 1024));
        }
    }
}

static uint64_t sgi_mc_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIMCState *s = SGI_MC(opaque);
    uint32_t val = 0;

    /* Registers are 32-bit but spaced at 64-bit intervals */
    addr &= ~7ULL;

    switch (addr) {
    case MC_CPU_CTRL0:
        val = s->cpu_ctrl[0];
        break;
    case MC_CPU_CTRL1:
        val = s->cpu_ctrl[1];
        break;
    case MC_WATCHDOG:
        val = s->watchdog;
        break;
    case MC_SYSID:
        val = s->sysid;
        break;
    case MC_RPSS_DIV:
        val = s->rpss_div;
        break;
    case MC_EEPROM:
        val = s->eeprom_ctrl;
        break;
    case MC_REFCNT_PRELOAD:
        val = s->refcnt_preload;
        break;
    case MC_REFCNT:
        val = s->refcnt;
        break;
    case MC_REALTIME_CTR:
        /*
         * QEMU extension: host wall-clock microseconds (QEMU_CLOCK_REALTIME).
         * Advances at true real-time speed regardless of -icount sleep=off.
         * Used by patched IRIX kernel for networking/animation timing so that
         * select() timeouts, nanosleep(), and TCP retransmit timers use real
         * time rather than racing virtual time.
         */
        val = (uint32_t)(qemu_clock_get_us(QEMU_CLOCK_REALTIME) & 0xFFFFFFFF);
        break;
    case MC_GIO64_ARB:
        val = s->gio64_arb;
        break;
    case MC_ARB_CPU_TIME:
        val = s->arb_cpu_time;
        break;
    case MC_ARB_BURST_TIME:
        val = s->arb_burst_time;
        break;
    case MC_MEMCFG0:
        val = s->memcfg[0];
        break;
    case MC_MEMCFG1:
        val = s->memcfg[1];
        break;
    case MC_CPU_MEM_ACCESS:
        val = s->cpu_mem_access;
        break;
    case MC_GIO_MEM_ACCESS:
        val = s->gio_mem_access;
        break;
    case MC_CPU_ERR_ADDR:
        val = s->cpu_err_addr;
        break;
    case MC_CPU_ERR_STATUS:
        val = s->cpu_err_status;
        break;
    case MC_GIO_ERR_ADDR:
        val = s->gio_err_addr;
        break;
    case MC_GIO_ERR_STATUS:
        val = s->gio_err_status;
        break;
    case MC_SYS_SEMAPHORE:
        val = s->sys_semaphore;
        break;
    case MC_GIO_LOCK:
        val = s->gio_lock;
        break;
    case MC_EISA_LOCK:
        val = s->eisa_lock;
        break;
    case MC_GIO64_XLATE_MASK:
        val = s->gio64_xlate_mask;
        break;
    case MC_GIO64_SUBST_BITS:
        val = s->gio64_subst_bits;
        break;
    case MC_DMA_INT_CAUSE:
        val = s->dma_int_cause;
        break;
    case MC_DMA_CONTROL:
        val = s->dma_control;
        break;
    case MC_DMA_TLB_HI0:
    case MC_DMA_TLB_HI1:
    case MC_DMA_TLB_HI2:
    case MC_DMA_TLB_HI3:
        val = s->dma_tlb_hi[(addr - MC_DMA_TLB_HI0) >> 4];
        break;
    case MC_DMA_TLB_LO0:
    case MC_DMA_TLB_LO1:
    case MC_DMA_TLB_LO2:
    case MC_DMA_TLB_LO3:
        val = s->dma_tlb_lo[(addr - MC_DMA_TLB_LO0) >> 4];
        break;
    case MC_DMA_MEM_ADDR_DEF:
    case MC_DMA_GIO_ADDR_START:
    case MC_DMA_START:
        /* Write-only registers, read returns 0 */
        val = 0;
        break;
    case MC_RPSS_CTR:
        /* RPSS counter - high-resolution timer used by IRIX */
        {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint32_t divider = (s->rpss_div & 0xff) + 1;
            uint32_t increment = (s->rpss_div >> 8) & 0xff;
            if (increment == 0) {
                increment = 1;
            }
            /* Calculate ticks since last update */
            int64_t elapsed_ns = now - s->last_rpss_update;
            /* Assume 50MHz base clock, divided by divider */
            uint64_t ticks = (elapsed_ns * 50) / (1000 * divider);

            /*
             * Always advance by at least 1 tick per read to prevent hangs
             * in busy-wait loops where virtual time doesn't advance.
             * This is necessary because the PROM polls the timer in tight
             * loops during diagnostics, and QEMU's virtual clock may not
             * advance between reads.
             */
            if (ticks == 0) {
                ticks = 1;
            }

            s->rpss_ctr += ticks * increment;
            s->last_rpss_update = now;
            val = s->rpss_ctr;
            trace_sgi_mc_rpss(val);
        }
        break;
    case MC_DMA_MEM_ADDR:
        val = s->dma_mem_addr;
        break;
    case MC_DMA_SIZE:
        val = s->dma_size;
        break;
    case MC_DMA_STRIDE:
        val = s->dma_stride;
        break;
    case MC_DMA_GIO_ADDR:
        val = s->dma_gio_addr;
        break;
    case MC_DMA_MODE:
        val = s->dma_mode;
        break;
    case MC_DMA_COUNT:
        val = s->dma_count;
        break;
    case MC_DMA_RUN:
        /*
         * DMA_RUN indicates if DMA is currently running.
         * Return current value, then clear it to simulate instant completion.
         * This allows PROM to see that DMA started (1) on first read after
         * DMA_START, then completed (0) on subsequent reads.
         */
        val = s->dma_run;
        s->dma_run = 0;  /* Clear after read - DMA "completed" */
        break;
    default:
        if (addr >= MC_SEMAPHORE_BASE &&
            addr < MC_SEMAPHORE_BASE + 16 * MC_SEMAPHORE_STRIDE) {
            /* Semaphores are at 0x10000, 0x11000, ..., 0x1f000 */
            int idx = (addr - MC_SEMAPHORE_BASE) / MC_SEMAPHORE_STRIDE;
            if (idx < 16) {
                /* Read returns current value, then sets to 1 (locked) */
                val = s->semaphore[idx];
                s->semaphore[idx] = 1;
            }
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mc: unimplemented read at 0x%04" HWADDR_PRIx
                          "\n", addr);
        }
        break;
    }

    trace_sgi_mc_read((uint64_t)addr, (uint64_t)val);
    return val;
}

static void sgi_mc_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    SGIMCState *s = SGI_MC(opaque);

    /* Registers are 32-bit but spaced at 64-bit intervals */
    addr &= ~7ULL;

    trace_sgi_mc_write((uint64_t)addr, val);

    switch (addr) {
    case MC_CPU_CTRL0:
        s->cpu_ctrl[0] = val;
        break;
    case MC_CPU_CTRL1:
        s->cpu_ctrl[1] = val;
        break;
    case MC_WATCHDOG:
        s->watchdog = val;
        break;
    case MC_SYSID:
        /* Read-only */
        break;
    case MC_RPSS_DIV:
        s->rpss_div = val;
        break;
    case MC_EEPROM:
        s->eeprom_ctrl = val;
        break;
    case MC_REFCNT_PRELOAD:
        s->refcnt_preload = val;
        break;
    case MC_REFCNT:
        s->refcnt = val;
        break;
    case MC_REALTIME_CTR:
        /* Read-only QEMU extension — ignore writes */
        break;
    case MC_GIO64_ARB:
        s->gio64_arb = val;
        break;
    case MC_ARB_CPU_TIME:
        s->arb_cpu_time = val;
        break;
    case MC_ARB_BURST_TIME:
        s->arb_burst_time = val;
        break;
    case MC_MEMCFG0:
        s->memcfg[0] = val;
        sgi_mc_update_ram_mapping(s);
        break;
    case MC_MEMCFG1:
        s->memcfg[1] = val;
        sgi_mc_update_ram_mapping(s);
        break;
    case MC_CPU_MEM_ACCESS:
        s->cpu_mem_access = val;
        break;
    case MC_GIO_MEM_ACCESS:
        s->gio_mem_access = val;
        break;
    case MC_CPU_ERR_ADDR:
        /* Read-only? Write to clear? */
        s->cpu_err_addr = 0;
        break;
    case MC_CPU_ERR_STATUS:
        /* Write to clear */
        s->cpu_err_status = 0;
        break;
    case MC_GIO_ERR_ADDR:
        s->gio_err_addr = 0;
        break;
    case MC_GIO_ERR_STATUS:
        s->gio_err_status = 0;
        break;
    case MC_SYS_SEMAPHORE:
        s->sys_semaphore = val;
        break;
    case MC_GIO_LOCK:
        s->gio_lock = val;
        break;
    case MC_EISA_LOCK:
        s->eisa_lock = val;
        break;
    case MC_GIO64_XLATE_MASK:
        s->gio64_xlate_mask = val;
        break;
    case MC_GIO64_SUBST_BITS:
        s->gio64_subst_bits = val;
        break;
    case MC_DMA_INT_CAUSE:
        /* Write to clear */
        s->dma_int_cause &= ~val;
        break;
    case MC_DMA_CONTROL:
        s->dma_control = val;
        break;
    case MC_DMA_TLB_HI0:
    case MC_DMA_TLB_HI1:
    case MC_DMA_TLB_HI2:
    case MC_DMA_TLB_HI3:
        s->dma_tlb_hi[(addr - MC_DMA_TLB_HI0) >> 4] = val;
        break;
    case MC_DMA_TLB_LO0:
    case MC_DMA_TLB_LO1:
    case MC_DMA_TLB_LO2:
    case MC_DMA_TLB_LO3:
        s->dma_tlb_lo[(addr - MC_DMA_TLB_LO0) >> 4] = val;
        break;
    case MC_RPSS_CTR:
        /* Writable for synchronization purposes */
        s->rpss_ctr = val;
        s->last_rpss_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case MC_DMA_MEM_ADDR:
        s->dma_mem_addr = val;
        break;
    case MC_DMA_SIZE:
        s->dma_size = val;
        break;
    case MC_DMA_STRIDE:
        s->dma_stride = val;
        break;
    case MC_DMA_GIO_ADDR:
        s->dma_gio_addr = val;
        break;
    case MC_DMA_MODE:
        s->dma_mode = val;
        break;
    case MC_DMA_COUNT:
        s->dma_count = val;
        break;
    case MC_DMA_RUN:
        s->dma_run = val;
        /* TODO: Actually perform DMA when enabled */
        break;
    case MC_DMA_MEM_ADDR_DEF:
        s->dma_mem_addr = val;
        /* TODO: Also set default parameters */
        break;
    case MC_DMA_GIO_ADDR_START:
        s->dma_gio_addr = val;
        /*
         * Start DMA when GIO address is written with "start" variant.
         * The PROM checks bit 6 (0x40) of DMA_RUN to verify DMA started.
         * Set 0x40 to indicate running, then clear after first read
         * to indicate completion.
         */
        s->dma_run = 0x40;
        trace_sgi_mc_dma(val, s->dma_size);
        break;
    case MC_DMA_START:
        /* Start virtual DMA - PROM checks bit 6 (0x40) of DMA_RUN */
        s->dma_run = 0x40;
        trace_sgi_mc_dma(val, s->dma_size);
        break;
    default:
        if (addr >= MC_SEMAPHORE_BASE &&
            addr < MC_SEMAPHORE_BASE + 16 * MC_SEMAPHORE_STRIDE) {
            /* Semaphores are at 0x10000, 0x11000, ..., 0x1f000 */
            int idx = (addr - MC_SEMAPHORE_BASE) / MC_SEMAPHORE_STRIDE;
            if (idx < 16) {
                s->semaphore[idx] = val;
            }
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "sgi_mc: unimplemented write at 0x%04" HWADDR_PRIx
                          " value 0x%08" PRIx64 "\n", addr, val);
        }
        break;
    }
}

static const MemoryRegionOps sgi_mc_ops = {
    .read = sgi_mc_read,
    .write = sgi_mc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Compute MEMCFG register value for a memory bank.
 * Each MEMCFG register holds configuration for 2 banks (16 bits each).
 *
 * Format per 16-bit bank:
 *   Bits 0-7:   Base address (physical address >> 22)
 *   Bits 8-12:  Size code ((size_in_4MB_units - 1) & 0x1f)
 *   Bit 13:     Valid
 *   Bit 14:     2 subbanks
 */
static uint16_t sgi_mc_memcfg_bank(uint32_t base_addr, uint32_t size_bytes,
                                    int addr_shift)
{
    uint16_t val = 0;
    uint32_t unit_size = 1U << addr_shift;
    uint32_t size_units, size_code;

    if (size_bytes == 0) {
        return 0;  /* Invalid bank */
    }

    /* Base address in memory units */
    val |= (base_addr >> addr_shift) & MEMCFG_ADDR_MASK;

    /* Size code: (code + 1) * unit_size = size, so code = units - 1 */
    size_units = size_bytes / unit_size;
    if (size_units > 0) {
        size_code = size_units - 1;
        if (size_code > 0x1f) {
            size_code = 0x1f;  /* Max per bank */
        }
        val |= (size_code << 8) & MEMCFG_SIZE_MASK;
    }

    /* Mark bank as valid */
    val |= MEMCFG_VLD;

    return val;
}

static void sgi_mc_reset(DeviceState *dev)
{
    SGIMCState *s = SGI_MC(dev);

    /* System ID: revision from property, optionally with EISA */
    s->sysid = s->revision;
    if (s->has_eisa) {
        s->sysid |= MC_SYSID_EISA_MASK;
    }

    s->cpu_ctrl[0] = 0;
    s->cpu_ctrl[1] = 0;
    s->watchdog = 0;
    s->rpss_div = 0x0104;  /* Default divider */
    s->eeprom_ctrl = 0;
    s->refcnt_preload = 0;
    s->refcnt = 0;
    s->gio64_arb = 0;
    s->arb_cpu_time = 0;
    s->arb_burst_time = 0;

    /*
     * Initialize per-bank state.
     * Bank 0 gets all the installed RAM; banks 1-3 are empty SIMM slots.
     * The PROM will probe banks by writing MEMCFG and testing addresses.
     */
    for (int i = 0; i < MC_NUM_BANKS; i++) {
        sgi_mc_unmap_bank(s, i);
        s->banks[i].installed_size = 0;
        s->banks[i].ram_offset = 0;
    }
    s->banks[0].installed_size = s->ram_size;
    s->banks[0].ram_offset = 0;

    /*
     * Set default MEMCFG0 with bank 0 configured at SEG0 base.
     * SEG0 is 0x08000000 for MC rev < 5, 0x20000000 for rev >= 5.
     * This ensures RAM is mapped at boot so exception vectors work.
     * MEMCFG1 starts at 0 (no banks configured).
     */
    {
        int shift = sgi_mc_addr_shift(s);
        uint32_t seg0_base = (shift >= 24) ? 0x20000000 : 0x08000000;
        uint16_t bank0_cfg = sgi_mc_memcfg_bank(seg0_base, s->ram_size,
                                                  shift);
        s->memcfg[0] = (uint32_t)bank0_cfg << 16;
    }
    s->memcfg[1] = 0;

    /* Apply the initial mapping */
    sgi_mc_update_ram_mapping(s);

    qemu_log_mask(LOG_UNIMP,
                  "sgi_mc: reset with ram_size=%u\n", s->ram_size);
    s->cpu_mem_access = 0;
    s->gio_mem_access = 0;
    s->cpu_err_addr = 0;
    s->cpu_err_status = 0;
    s->gio_err_addr = 0;
    s->gio_err_status = 0;
    s->sys_semaphore = 0;
    s->gio_lock = 0;
    s->eisa_lock = 0;
    s->gio64_xlate_mask = 0;
    s->gio64_subst_bits = 0;
    s->dma_int_cause = 0;
    s->dma_control = 0;
    for (int i = 0; i < 4; i++) {
        s->dma_tlb_hi[i] = 0;
        s->dma_tlb_lo[i] = 0;
    }
    s->rpss_ctr = 0;
    s->last_rpss_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->dma_mem_addr = 0;
    s->dma_size = 0;
    s->dma_stride = 0;
    s->dma_gio_addr = 0;
    s->dma_mode = 0;
    s->dma_count = 0;
    s->dma_run = 0;
    for (int i = 0; i < 16; i++) {
        s->semaphore[i] = 0;
    }
}

static void sgi_mc_init(Object *obj)
{
    SGIMCState *s = SGI_MC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &sgi_mc_ops, s,
                          "sgi-mc", MC_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_sgi_mc = {
    .name = "sgi-mc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cpu_ctrl, SGIMCState, 2),
        VMSTATE_UINT32(watchdog, SGIMCState),
        VMSTATE_UINT32(sysid, SGIMCState),
        VMSTATE_UINT32(rpss_div, SGIMCState),
        VMSTATE_UINT32(eeprom_ctrl, SGIMCState),
        VMSTATE_UINT32(refcnt_preload, SGIMCState),
        VMSTATE_UINT32(refcnt, SGIMCState),
        VMSTATE_UINT32(gio64_arb, SGIMCState),
        VMSTATE_UINT32(arb_cpu_time, SGIMCState),
        VMSTATE_UINT32(arb_burst_time, SGIMCState),
        VMSTATE_UINT32_ARRAY(memcfg, SGIMCState, 2),
        VMSTATE_UINT32(cpu_mem_access, SGIMCState),
        VMSTATE_UINT32(gio_mem_access, SGIMCState),
        VMSTATE_UINT32(cpu_err_addr, SGIMCState),
        VMSTATE_UINT32(cpu_err_status, SGIMCState),
        VMSTATE_UINT32(gio_err_addr, SGIMCState),
        VMSTATE_UINT32(gio_err_status, SGIMCState),
        VMSTATE_UINT32(sys_semaphore, SGIMCState),
        VMSTATE_UINT32(gio_lock, SGIMCState),
        VMSTATE_UINT32(eisa_lock, SGIMCState),
        VMSTATE_UINT32(gio64_xlate_mask, SGIMCState),
        VMSTATE_UINT32(gio64_subst_bits, SGIMCState),
        VMSTATE_UINT32(dma_int_cause, SGIMCState),
        VMSTATE_UINT32(dma_control, SGIMCState),
        VMSTATE_UINT32_ARRAY(dma_tlb_hi, SGIMCState, 4),
        VMSTATE_UINT32_ARRAY(dma_tlb_lo, SGIMCState, 4),
        VMSTATE_UINT32(rpss_ctr, SGIMCState),
        VMSTATE_UINT32(dma_mem_addr, SGIMCState),
        VMSTATE_UINT32(dma_size, SGIMCState),
        VMSTATE_UINT32(dma_stride, SGIMCState),
        VMSTATE_UINT32(dma_gio_addr, SGIMCState),
        VMSTATE_UINT32(dma_mode, SGIMCState),
        VMSTATE_UINT32(dma_count, SGIMCState),
        VMSTATE_UINT32(dma_run, SGIMCState),
        VMSTATE_UINT32_ARRAY(semaphore, SGIMCState, 16),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sgi_mc_properties[] = {
    DEFINE_PROP_UINT32("ram-size", SGIMCState, ram_size, 64 * 1024 * 1024),
    DEFINE_PROP_BOOL("has-eisa", SGIMCState, has_eisa, false),
    DEFINE_PROP_UINT8("revision", SGIMCState, revision, MC_SYSID_REV_C),
    DEFINE_PROP_LINK("ram", SGIMCState, ram, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("system-memory", SGIMCState, system_memory,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void sgi_mc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, sgi_mc_reset);
    dc->vmsd = &vmstate_sgi_mc;
    device_class_set_props(dc, sgi_mc_properties);
}

static const TypeInfo sgi_mc_info = {
    .name          = TYPE_SGI_MC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIMCState),
    .instance_init = sgi_mc_init,
    .class_init    = sgi_mc_class_init,
};

static void sgi_mc_register_types(void)
{
    type_register_static(&sgi_mc_info);
}

type_init(sgi_mc_register_types)
