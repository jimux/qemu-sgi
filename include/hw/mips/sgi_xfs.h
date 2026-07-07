/*
 * Host-side read-only XFS (IRIX V1) file reader for the Virtuix (IP55)
 * paravirtual ARCS PROM (Mode C boot chain).
 *
 * This is the generic substrate the Track C paravirtual firmware uses to read
 * files off an SGI disk's XFS root partition through the QEMU block layer,
 * WITHOUT executing the guest's own sash. It intentionally re-implements the
 * minimum of stand/arcs libsc/xfs semantics (superblock -> inode -> extents ->
 * V1 directory lookup) needed to resolve a path and read a file's bytes.
 *
 * Read-only; no attrs, no writes. Big-endian on-disk (matches the golden's
 * IRIX 6.5 XFS V1: versionnum 0x1094, 4KB blocks, V1 leaf/shortform dirs).
 *
 * The same three primitives (mount / lookup / read) back both:
 *   - the Mode C "/unix-direct" boot (load the disk kernel host-side), and
 *   - the future ARCS Open/Read/Seek/Close firmware services (Path A sash).
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_SGI_XFS_H
#define HW_MIPS_SGI_XFS_H

#include "system/block-backend.h"

/* One on-disk extent (bmbt record), decoded. */
typedef struct {
    uint64_t startoff;      /* logical file block */
    uint64_t startblock;    /* filesystem block */
    uint64_t blockcount;
} SGIXfsExtent;

/* A mounted XFS filesystem (geometry cached from the superblock). */
typedef struct {
    BlockBackend *blk;
    uint64_t part_byte_off; /* byte offset of the partition start on the disk */
    uint32_t blocksize;
    uint8_t  blocklog;
    uint32_t agblocks;
    uint32_t agcount;
    uint8_t  agblklog;
    uint8_t  inopblog;
    uint16_t inodesize;
    uint64_t rootino;
    uint16_t versionnum;
    bool     mounted;
} SGIXfs;

/* A resolved inode: its size/mode/format plus the data map (extents) or, for
 * LOCAL format, the inline fork bytes. */
typedef struct {
    uint64_t size;          /* di_size */
    uint16_t mode;          /* di_mode */
    uint8_t  format;        /* di_format: 1 LOCAL, 2 EXTENTS, 3 BTREE */
    SGIXfsExtent *extents;  /* EXTENTS/BTREE: malloc'd array */
    int nextents;
    uint8_t *local;         /* LOCAL: malloc'd inline bytes (di_size long) */
    int local_len;
} SGIXfsInode;

/*
 * Mount the XFS filesystem whose partition starts at disk block
 * `part_first_lbn` (512-byte units, from the SGI volume-header partition
 * table). Reads + validates the superblock and caches geometry.
 * Returns 0 on success, negative on failure.
 */
int sgi_xfs_mount(BlockBackend *blk, uint64_t part_first_lbn, SGIXfs *fs);

/*
 * Resolve an absolute path (e.g. "/unix") to its inode, filling `out`.
 * Follows V1 shortform + V1 leaf directories. Does NOT follow symlinks.
 * Returns 0 on success, negative on failure. On success the caller must
 * sgi_xfs_inode_put(out) to free the extent/local buffers.
 */
int sgi_xfs_lookup(SGIXfs *fs, const char *path, SGIXfsInode *out);

/*
 * Read `len` bytes from file `ino` at byte offset `off` into `buf`. Zero-fills
 * holes; clamps to di_size. Sets *got to bytes returned (may be < len at EOF).
 * Returns 0 on success, negative on I/O error.
 */
int sgi_xfs_read(SGIXfs *fs, const SGIXfsInode *ino, uint64_t off,
                 uint64_t len, void *buf, uint64_t *got);

/* Free the buffers held by a resolved inode. */
void sgi_xfs_inode_put(SGIXfsInode *ino);

#endif /* HW_MIPS_SGI_XFS_H */
