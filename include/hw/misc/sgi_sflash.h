/*
 * SGI IP30 (Octane) BRIDGE serial-flash controller model.
 *
 * The IP30 boot PROM is an 8-bit-wide flash in the BRIDGE's external-flash
 * window (BRIDGE+0xC00000 = physical 0x1FC00000, alternate +0xE00000). It holds
 * 16 x 64 KB segments: RPROM (0..2), FPROM (3..14) and the flash Persistent
 * Data Storage / NVRAM segment (15). The device is memory-mapped for reads and
 * accepts SHARP/Intel-compatibility commands written as 16-bit words to the
 * flash window; the address of the command selects the segment.
 *
 * Only the pieces the PROM needs are modelled: the part-ID read, the status
 * read, and a writable PDS segment. The FPROM code segments stay backed by the
 * PROM image (read-only). Command values and IDs are from
 * irix/kern/sys/RACER/sflash.h; behaviour from libsk/ml/sflash.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_SFLASH_H
#define HW_MISC_SGI_SFLASH_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_SGI_SFLASH "sgi-sflash"
OBJECT_DECLARE_SIMPLE_TYPE(SGISflashState, SGI_SFLASH)

#define SGI_SFLASH_SEG_SIZE (64 * 1024)

struct SGISflashState {
    SysBusDevice parent_obj;

    /* Flash command window (base; 16 bytes is ample for ID/status). */
    MemoryRegion cmd_iomem;
    /* PDS segment (segment 15), 64 KB, writable. */
    MemoryRegion pds_iomem;

    /* Backing PROM image, for reads of the code segments. */
    MemoryRegion *rom;

    /*
     * Optional file backing the PDS (NVRAM) segment, so PROM environment
     * variables (OSLoadFilename, SystemPartition, ...) persist across runs --
     * otherwise the segment starts erased and the PROM reinitializes its env
     * and the time-of-day clock on every boot.  Unset means volatile.
     */
    char *pds_file;

    uint8_t pds[SGI_SFLASH_SEG_SIZE];

    uint32_t cmd;
    uint32_t cmd_off;      /* command offset within its window */
    bool write_pending;    /* next window write is data */
    bool erase_pending;    /* next command is the erase confirm */
};

#endif /* HW_MISC_SGI_SFLASH_H */
