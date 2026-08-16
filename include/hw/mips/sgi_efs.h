/*
 * Host-side read-only EFS (Extent File System) reader for the Virtuix (IP55)
 * paravirtual ARCS PROM (Mode C install boot chain).
 *
 * The paravirtual firmware loads install-media standalone programs (fx.ARCS,
 * the miniroot kernel dist/miniroot/unix.IP*) from the SGI install CD's EFS
 * partition (partition 7) through the QEMU block layer, WITHOUT executing the
 * guest's own sash. This intentionally re-implements the minimum of the EFS
 * on-disk format (superblock -> inode -> extents -> 0xBEEF slot directory)
 * needed to resolve a path and read a file's bytes.
 *
 * Read-only. Big-endian on-disk. 512-byte blocks, 128-byte inodes, inode 2 =
 * root. Indirect extents (numextents > 12) follow the SGI convention: the
 * first extent's `offset` field holds the indirect-block count.
 *
 * Mirrors the sgi_xfs.c API (mount / lookup / read / inode_put).
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_SGI_EFS_H
#define HW_MIPS_SGI_EFS_H

#include "system/block-backend.h"

/* One on-disk extent descriptor (8 bytes), decoded. */
typedef struct {
    uint64_t bn;      /* 24-bit block number */
    uint8_t  length;  /* length in 512-byte blocks */
    uint64_t offset;  /* 24-bit logical byte offset / 512 */
} SGIEFSExtent;

/* A mounted EFS filesystem (geometry cached from the superblock). */
typedef struct {
    BlockBackend *blk;
    uint64_t part_byte_off; /* byte offset of the partition start on the disk */
    uint32_t fs_firstcg;    /* first cylinder group block (fs_firstcg) */
    uint32_t fs_cgfsize;    /* cylinder group size in blocks (fs_cgfsize) */
    uint32_t fs_cgisize;    /* cylinder group inode size (fs_cgisize) */
    uint16_t fs_ncg;        /* number of cylinder groups (fs_ncg) */
    bool     mounted;
} SGIEFS;

/* A resolved inode: size/mode plus the data map (extents). */
typedef struct {
    uint64_t size;          /* inode size */
    uint16_t mode;          /* inode mode (S_IFMT type bits) */
    SGIEFSExtent *extents;  /* malloc'd array */
    int nextents;
} SGIEFSInode;

/*
 * Mount the EFS filesystem whose partition starts at disk block
 * `part_first_lbn` (512-byte units, from the SGI volume-header partition
 * table). Reads + validates the superblock (block 1 of the partition).
 * Returns 0 on success, negative on failure.
 */
int sgi_efs_mount(BlockBackend *blk, uint64_t part_first_lbn, SGIEFS *fs);

/*
 * Resolve an absolute path (e.g. "/stand/fx.ARCS") to its inode, filling
 * `out`. Follows 0xBEEF slot directories. Does NOT follow symlinks.
 * Returns 0 on success, negative on failure. On success the caller must
 * sgi_efs_inode_put(out).
 */
int sgi_efs_lookup(SGIEFS *fs, const char *path, SGIEFSInode *out);

/*
 * Read `len` bytes from file `ino` at byte offset `off` into `buf`. Clamps to
 * size. Sets *got to bytes returned (may be < len at EOF). Returns 0 on
 * success, negative on I/O error.
 */
int sgi_efs_read(SGIEFS *fs, const SGIEFSInode *ino, uint64_t off,
                 uint64_t len, void *buf, uint64_t *got);

/* Free the buffers held by a resolved inode. */
void sgi_efs_inode_put(SGIEFSInode *ino);

#endif /* HW_MIPS_SGI_EFS_H */
