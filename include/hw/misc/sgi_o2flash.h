/*
 * SGI O2 (IP32) system flash — 512 KiB at physical 0x1FC00000.
 *
 * The O2 boot PROM lives in a 512 KiB flash part that the CPU reads and
 * executes directly at 0x1FC00000.  The part is organised into 256-byte
 * sectors carrying 'SHDR' segments (see include/hw/misc/sgi_mace.h and the
 * IP32 PROM source <sys/IP32flash.h>); the last segment, "env", holds the
 * persistent environment variables (AutoLoad, console, netaddr, ...).
 *
 * Programming model (authoritative references, not inferred):
 *
 *   - IRIX kernel driver ml/MOOSEHEAD/IP32flash.c:flash_write_sector()
 *     clears the write-protect via flash_write_enable(), then strobes
 *     FLASH_PAGE_SIZE bytes into the flash window with plain byte stores
 *     (pciio_pio_write8), then re-asserts write-protect.  There is NO
 *     CFI command sequence: the chip's program/erase "protocol" on IP32
 *     is write-enable + direct byte program, with the caller rewriting a
 *     whole sector.  (flash_write_env() zeroes its staging buffer first,
 *     which is the erase.)
 *   - PROM source stand/arcs/IP32prom/lib/IP32k.c:flashWrite()/flashEnable()
 *     and stand/arcs/IP32prom/sloader/sloader.c do the same: assert bit 0
 *     (ISA_FLASH_WE) of the MACE ISA "Flash/LED/DPRAM/NIC Control" register
 *     at 0x1F310008, then write bytes.
 *   - The standalone simulator stand/arcs/IP32prom/sim2/svrFlash.c models
 *     exactly this and says so: the write map has ignoreIfZero tied to the
 *     write-enable word, and "We don't emulate the FLASH 'magic handshakes'".
 *
 * So this device is a rom_device: reads/instruction fetches are served
 * directly from RAM (fast), while writes land in sgi_o2_flash_write(), which
 * applies them only when MACE has the write-enable bit set and persists the
 * env segment to the sidecar when a completed rewrite makes it valid.
 *
 * The env sidecar is a raw copy of the env segment (header + body), in the
 * style of the hpc3/Indy NVRAM backing files.  It is loaded over the PROM's
 * compiled-in env at power-on, so a fresh QEMU sees the last saved env, and
 * it is never touched when no sidecar exists (stock boot is byte-identical).
 *
 * The sidecar path is PER-INSTANCE.  The device property "nvram-file" (or
 * "-global sgi-o2-flash.nvram-file=<path>") sets it explicitly; when unset,
 * the sgi-o2 machine derives it from the first disk's image as
 * "<image>.o2nvram".  A fixed name would be shared by every O2 launched from
 * the same working directory, so one VM's `setenv` would silently change
 * another's env.  Only a diskless (PROM-only) O2, which has no disk to key
 * on, falls back to the cwd-relative "sgi_o2_nvram.bin"; pass the global
 * explicitly if such an instance needs isolation.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_O2FLASH_H
#define HW_MISC_SGI_O2FLASH_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_O2_FLASH "sgi-o2-flash"
OBJECT_DECLARE_SIMPLE_TYPE(SGIO2FlashState, SGI_O2_FLASH)

#define SGI_O2_FLASH_SIZE       (512 * 1024)
#define SGI_O2_FLASH_PAGE_SIZE  0x100   /* programming sector (FSecSize)   */
#define SGI_O2_FLASH_SEG_HDR    0x40    /* 'SHDR' header size              */
#define SGI_O2_FLASH_SEG_MAGIC  0x53484452 /* 'SHDR'                       */

struct SGIO2FlashState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /*
     * 512 KiB flash array.  This is the qemu_ram_block backing the rom_device
     * region (obtained with memory_region_get_ram_ptr); direct reads and
     * guest instruction fetches go straight here.
     */
    uint8_t *storage;
    uint32_t size;

    /* Env sidecar path (NULL = no persistence). */
    char *nvram_filename;

    /* MACE (ISA_FLASH_NIC_REG owns the flash write-enable latch). */
    DeviceState *mace;

    /*
     * Write-protect latch sourced from MACE ISA_FLASH_NIC_REG bit 0.  The
     * flash gates programming on it, exactly as the hardware does.
     */
    bool write_enable;

    /*
     * -global sgi-o2-flash.autoload=false / -machine autoload=off force the
     * env AutoLoad value to "N" at power-on (a convenience override; the
     * persisted env is left untouched).
     */
    bool autoload;

    /* The "env" segment, located when the PROM image is loaded. */
    bool env_found;
    uint32_t env_offset;
    uint32_t env_len;
};

/*
 * Populate the flash from an already container-stripped and patched PROM
 * image, locate the env segment, overlay the sidecar (if any) and apply the
 * autoload override.
 */
void sgi_o2_flash_load_prom(DeviceState *dev, const uint8_t *data,
                            uint32_t len);

/*
 * Wire the MACE whose ISA_FLASH_NIC_REG bit 0 gates programming.  A plain
 * setter, not a QOM link: the machine sets it during init, and the flash
 * reads the latch live (so a MACE reset re-locks the flash for free).
 */
void sgi_o2_flash_set_mace(DeviceState *dev, DeviceState *mace);

#endif /* HW_MISC_SGI_O2FLASH_H */
