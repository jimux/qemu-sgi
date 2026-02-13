/*
 * SGI Virtual Boot Disk
 *
 * Simple MMIO block device for PROM disk access without full SCSI emulation.
 * Placed at physical 0x17000000 (between GBE at 0x16000000 and MACE at
 * 0x1F000000) in the SGI O2 memory map.
 *
 * Register layout (32-bit registers at 4-byte spacing):
 *   0x000  SECTOR_LO  (W)  Low 32 bits of sector number
 *   0x004  SECTOR_HI  (W)  High 32 bits of sector number
 *   0x008  COUNT      (W)  Byte count for transfer (max 512)
 *   0x00C  COMMAND    (W)  1=READ
 *   0x010  STATUS     (R)  bit 31=READY, bit 0=ERROR
 *   0x014  DISK_SIZE_LO (R) Total sectors (low 32)
 *   0x018  DISK_SIZE_HI (R) Total sectors (high 32)
 *   0x200-0x3FF DATA  (R)  512-byte sector data window
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SGI_BOOTDISK_H
#define HW_SGI_BOOTDISK_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_BOOTDISK "sgi-bootdisk"
OBJECT_DECLARE_SIMPLE_TYPE(SGIBootDiskState, SGI_BOOTDISK)

#define SGI_BOOTDISK_MMIO_SIZE    0x400
#define SGI_BOOTDISK_SECTOR_SIZE  512

/* Register offsets */
#define REG_SECTOR_LO    0x000
#define REG_SECTOR_HI    0x004
#define REG_COUNT        0x008
#define REG_COMMAND      0x00C
#define REG_STATUS       0x010
#define REG_DISK_SIZE_LO 0x014
#define REG_DISK_SIZE_HI 0x018
#define REG_DATA_BASE    0x200

/* Commands */
#define CMD_READ         1

/* Status bits */
#define STATUS_READY     (1U << 31)
#define STATUS_ERROR     (1U << 0)

struct SGIBootDiskState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    BlockBackend *blk;

    /* Registers */
    uint32_t sector_lo;
    uint32_t sector_hi;
    uint32_t count;
    uint32_t status;

    /* Data buffer */
    uint8_t data[SGI_BOOTDISK_SECTOR_SIZE];
};

#endif /* HW_SGI_BOOTDISK_H */
