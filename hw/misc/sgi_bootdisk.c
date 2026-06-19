/*
 * SGI Virtual Boot Disk
 *
 * Simple MMIO block device for PROM disk access. Provides a sector-at-a-time
 * read interface that the PROM firmware can use to load kernels and bootloaders
 * from disk images without needing full PCI/SCSI controller emulation.
 *
 * The device is mapped at 0x17000000 in the O2 physical address space.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/misc/sgi_bootdisk.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/block-backend.h"
#include "qapi/error.h"

static uint64_t sgi_bootdisk_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIBootDiskState *s = SGI_BOOTDISK(opaque);

    switch (addr) {
    case REG_STATUS:
        return s->status;

    case REG_DISK_SIZE_LO:
        if (s->blk) {
            uint64_t nb_sectors = blk_getlength(s->blk) / SGI_BOOTDISK_SECTOR_SIZE;
            return (uint32_t)nb_sectors;
        }
        return 0;

    case REG_DISK_SIZE_HI:
        if (s->blk) {
            uint64_t nb_sectors = blk_getlength(s->blk) / SGI_BOOTDISK_SECTOR_SIZE;
            return (uint32_t)(nb_sectors >> 32);
        }
        return 0;

    default:
        /* Data window: 0x200-0x3FF */
        if (addr >= REG_DATA_BASE &&
            addr < REG_DATA_BASE + SGI_BOOTDISK_SECTOR_SIZE) {
            uint32_t offset = addr - REG_DATA_BASE;
            if (size == 1) {
                return s->data[offset];
            } else if (size == 2) {
                return (s->data[offset] << 8) | s->data[offset + 1];
            } else {
                return ((uint32_t)s->data[offset] << 24) |
                       ((uint32_t)s->data[offset + 1] << 16) |
                       ((uint32_t)s->data[offset + 2] << 8) |
                       (uint32_t)s->data[offset + 3];
            }
        }
        qemu_log_mask(LOG_UNIMP, "sgi_bootdisk: read from 0x%03" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void sgi_bootdisk_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    SGIBootDiskState *s = SGI_BOOTDISK(opaque);

    switch (addr) {
    case REG_SECTOR_LO:
        s->sector_lo = (uint32_t)val;
        break;

    case REG_SECTOR_HI:
        s->sector_hi = (uint32_t)val;
        break;

    case REG_COUNT:
        s->count = (uint32_t)val;
        if (s->count > SGI_BOOTDISK_SECTOR_SIZE) {
            s->count = SGI_BOOTDISK_SECTOR_SIZE;
        }
        break;

    case REG_COMMAND:
        if ((uint32_t)val == CMD_READ) {
            if (!s->blk) {
                fprintf(stderr, "sgi_bootdisk: CMD_READ but no blk!\n");
                s->status = STATUS_READY | STATUS_ERROR;
                break;
            }

            uint64_t sector = ((uint64_t)s->sector_hi << 32) | s->sector_lo;
            int64_t offset = sector * SGI_BOOTDISK_SECTOR_SIZE;

            memset(s->data, 0, SGI_BOOTDISK_SECTOR_SIZE);

            if (blk_pread(s->blk, offset, SGI_BOOTDISK_SECTOR_SIZE,
                          s->data, 0) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sgi_bootdisk: read error at sector %"
                              PRIu64 "\n", sector);
                s->status = STATUS_READY | STATUS_ERROR;
            } else {
                s->status = STATUS_READY;
            }
            fprintf(stderr, "BDRD sec=%" PRIu64 " d=%02x%02x%02x%02x\n",
                    sector, s->data[0], s->data[1],
                    s->data[2], s->data[3]);
            /*
             * INODE BUFFER DETECTOR: if any 256-byte slot in this sector has
             * 'IN' magic (0x494e) AND any other slot has magic 0x0000, log
             * it as suspicious. This catches the "Bad magic in XFS inode
             * buffer" pattern at the point of issue (read return).
             */
            {
                int has_ino_magic = 0, has_zero_slot = 0;
                for (int k = 0; k < SGI_BOOTDISK_SECTOR_SIZE; k += 256) {
                    if (s->data[k] == 0x49 && s->data[k+1] == 0x4e) {
                        has_ino_magic = 1;
                    } else if (s->data[k] == 0 && s->data[k+1] == 0
                               && s->data[k+2] == 0 && s->data[k+3] == 0) {
                        has_zero_slot = 1;
                    }
                }
                if (has_ino_magic && has_zero_slot) {
                    fprintf(stderr, "BDRD-MIX sec=%" PRIu64
                            " slot0=%02x%02x slot1=%02x%02x\n",
                            sector, s->data[0], s->data[1],
                            s->data[256], s->data[257]);
                }
            }
        } else if ((uint32_t)val == CMD_WRITE) {
            if (!s->blk) {
                fprintf(stderr, "sgi_bootdisk: CMD_WRITE but no blk!\n");
                s->status = STATUS_READY | STATUS_ERROR;
                break;
            }

            uint64_t sector = ((uint64_t)s->sector_hi << 32) | s->sector_lo;
            int64_t offset = sector * SGI_BOOTDISK_SECTOR_SIZE;

            if (blk_pwrite(s->blk, offset, SGI_BOOTDISK_SECTOR_SIZE,
                           s->data, 0) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sgi_bootdisk: write error at sector %"
                              PRIu64 "\n", sector);
                s->status = STATUS_READY | STATUS_ERROR;
            } else {
                s->status = STATUS_READY;
            }
        }
        break;

    default:
        /* Data window writes: kernel fills the sector buffer before CMD_WRITE */
        if (addr >= REG_DATA_BASE &&
            addr < REG_DATA_BASE + SGI_BOOTDISK_SECTOR_SIZE) {
            uint32_t offset = addr - REG_DATA_BASE;
            if (size == 1) {
                s->data[offset] = (uint8_t)val;
            } else if (size == 2) {
                s->data[offset]     = (uint8_t)(val >> 8);
                s->data[offset + 1] = (uint8_t)(val & 0xFF);
            } else {
                s->data[offset]     = (uint8_t)(val >> 24);
                s->data[offset + 1] = (uint8_t)(val >> 16);
                s->data[offset + 2] = (uint8_t)(val >> 8);
                s->data[offset + 3] = (uint8_t)(val & 0xFF);
            }
            break;
        }
        qemu_log_mask(LOG_UNIMP, "sgi_bootdisk: write 0x%08" PRIx64
                      " to 0x%03" HWADDR_PRIx "\n", val, addr);
        break;
    }
}

static const MemoryRegionOps sgi_bootdisk_ops = {
    .read = sgi_bootdisk_read,
    .write = sgi_bootdisk_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_bootdisk_realize(DeviceState *dev, Error **errp)
{
    SGIBootDiskState *s = SGI_BOOTDISK(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &sgi_bootdisk_ops, s,
                          TYPE_SGI_BOOTDISK, SGI_BOOTDISK_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    if (s->blk) {
        uint64_t perm = BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE;
        int ret;

        ret = blk_set_perm(s->blk, perm, BLK_PERM_ALL, errp);
        if (ret < 0) {
            return;
        }
    }

    s->status = STATUS_READY;
    memset(s->data, 0, SGI_BOOTDISK_SECTOR_SIZE);

    if (s->blk) {
        int64_t len = blk_getlength(s->blk);
        fprintf(stderr, "sgi_bootdisk: drive attached, size=%" PRId64 " bytes\n",
                len);
    } else {
        fprintf(stderr, "sgi_bootdisk: no drive attached\n");
    }
}

static const Property sgi_bootdisk_properties[] = {
    DEFINE_PROP_DRIVE("drive", SGIBootDiskState, blk),
};

static void sgi_bootdisk_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "SGI Virtual Boot Disk";
    dc->realize = sgi_bootdisk_realize;
    device_class_set_props(dc, sgi_bootdisk_properties);
}

static const TypeInfo sgi_bootdisk_info = {
    .name          = TYPE_SGI_BOOTDISK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIBootDiskState),
    .class_init    = sgi_bootdisk_class_init,
};

static void sgi_bootdisk_register_types(void)
{
    type_register_static(&sgi_bootdisk_info);
}

type_init(sgi_bootdisk_register_types)
