/*
 * Host-side read-only XFS (IRIX V1) file reader — see include/hw/mips/sgi_xfs.h.
 *
 * Ported from the concrete on-disk layout in pyirix.xfs (validated against the
 * canonical golden) and the algorithm in stand/arcs/lib/libsc/xfs/xfs.c (the
 * PROM's own reader). Covers what the Mode C boot chain needs: superblock,
 * inode read (V1), EXTENTS + BTREE data forks, and V1 shortform/leaf directory
 * lookup. Read-only; no dir2, no attrs, no writes.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/mips/sgi_xfs.h"
#include "system/block-backend.h"

/* On-disk magics / constants (pyirix.xfs.constants). */
#define XFS_SB_MAGIC        0x58465342u   /* 'XFSB' */
#define XFS_DINODE_MAGIC    0x494Eu       /* 'IN'   */
#define XFS_BMAP_MAGIC      0x424D4150u   /* 'BMAP' */
#define XFS_DIR_LEAF_MAGIC  0xFEEBu       /* V1 leaf directory block */
#define XFS_DATA_FORK_OFFSET 100          /* core(96) + di_next_unlinked(4) */
#define XFS_SECTOR          512
#define XFS_DIR_SF_HDR_SIZE 9             /* parent(8) + count(1) */
#define XFS_DIR_LEAF_HDR_SIZE   32
#define XFS_DIR_LEAF_ENTRY_SIZE 8

#define XFS_DINODE_FMT_LOCAL   1
#define XFS_DINODE_FMT_EXTENTS 2
#define XFS_DINODE_FMT_BTREE   3

#define S_IFMT_  0170000
#define S_IFDIR_ 0040000

/* Guardrails so a corrupt/hostile disk can't make us loop or allocate wildly. */
#define XFS_MAX_EXTENTS   (256 * 1024)    /* ~256K extents (huge file) */
#define XFS_MAX_PATH_COMP 64

static uint16_t be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}
static uint64_t be64(const uint8_t *p)
{
    return ((uint64_t)be32(p) << 32) | be32(p + 4);
}

/* ---- address conversions (pyirix.xfs.ondisk) ---- */

static uint64_t ino_to_offset(const SGIXfs *fs, uint64_t ino)
{
    unsigned shift = fs->agblklog + fs->inopblog;
    uint64_t agno = ino >> shift;
    uint64_t agino = ino & (((uint64_t)1 << shift) - 1);
    uint64_t agbno = agino >> fs->inopblog;
    uint64_t slot = agino & (((uint64_t)1 << fs->inopblog) - 1);
    uint64_t phys_block = agno * fs->agblocks + agbno;
    return fs->part_byte_off + phys_block * fs->blocksize +
           slot * fs->inodesize;
}

static uint64_t fsblock_to_offset(const SGIXfs *fs, uint64_t fsb)
{
    uint64_t agno = fsb >> fs->agblklog;
    uint64_t agbno = fsb & (((uint64_t)1 << fs->agblklog) - 1);
    uint64_t phys_block = agno * fs->agblocks + agbno;
    return fs->part_byte_off + phys_block * fs->blocksize;
}

/* Decode a 16-byte packed bmbt extent record. */
static void parse_bmbt(const uint8_t *p, SGIXfsExtent *e)
{
    uint64_t l0 = be64(p);
    uint64_t l1 = be64(p + 8);
    e->startoff   = (l0 >> 9) & 0x3FFFFFFFFFFFFFULL;
    e->startblock = ((l0 & 0x1FF) << 43) | (l1 >> 21);
    e->blockcount = l1 & 0x1FFFFF;
}

/* ---- mount ---- */

int sgi_xfs_mount(BlockBackend *blk, uint64_t part_first_lbn, SGIXfs *fs)
{
    uint8_t sb[XFS_SECTOR];

    memset(fs, 0, sizeof(*fs));
    fs->blk = blk;
    fs->part_byte_off = part_first_lbn * XFS_SECTOR;

    if (blk_pread(blk, fs->part_byte_off, XFS_SECTOR, sb, 0) < 0) {
        return -1;
    }
    if (be32(&sb[0x00]) != XFS_SB_MAGIC) {
        return -2;
    }

    fs->blocksize  = be32(&sb[0x04]);
    fs->agblocks   = be32(&sb[0x54]);
    fs->agcount    = be32(&sb[0x58]);
    fs->versionnum = be16(&sb[0x64]);
    fs->inodesize  = be16(&sb[0x68]);
    fs->blocklog   = sb[0x78];
    fs->agblklog   = sb[0x7C];
    fs->inopblog   = sb[0x7B];
    fs->rootino    = be64(&sb[0x38]);

    if (fs->blocksize == 0 || fs->inodesize == 0 || fs->agblocks == 0) {
        return -3;
    }
    /* Mode C only supports V1 directories (no DIRV2 bit 0x2000). */
    if (fs->versionnum & 0x2000) {
        qemu_log("sgi_xfs: dir2 (versionnum 0x%04x) not supported\n",
                 fs->versionnum);
        return -4;
    }
    fs->mounted = true;
    return 0;
}

/* ---- extent-fork collection (EXTENTS + BTREE) ---- */

typedef struct {
    SGIXfsExtent *v;
    int n;
    int cap;
    bool err;
} ExtVec;

static void ext_push(ExtVec *ev, const SGIXfsExtent *e)
{
    if (e->blockcount == 0) {
        return;
    }
    if (ev->n >= XFS_MAX_EXTENTS) {
        ev->err = true;
        return;
    }
    if (ev->n == ev->cap) {
        ev->cap = ev->cap ? ev->cap * 2 : 8;
        ev->v = g_renew(SGIXfsExtent, ev->v, ev->cap);
    }
    ev->v[ev->n++] = *e;
}

/*
 * Walk a bmap B+tree rooted in the inode data fork (di_format BTREE) and
 * collect all leaf extents. Mirrors pyirix.xfs.inode._btree_get_extents.
 */
static void btree_collect(SGIXfs *fs, const uint8_t *fork, int forklen,
                          ExtVec *ev)
{
    uint8_t *blk = NULL;
    uint64_t bno;
    int level, numrecs, i;
    uint64_t visited_first = 0;
    bool have_visited = false;

    if (forklen < 4) {
        return;
    }
    level = be16(&fork[0]);
    numrecs = be16(&fork[2]);
    if (numrecs == 0 || level > 10) {
        return;
    }
    if (level == 0) {
        for (i = 0; i < numrecs; i++) {
            int off = 4 + i * 16;
            SGIXfsExtent e;
            if (off + 16 > forklen) {
                break;
            }
            parse_bmbt(&fork[off], &e);
            ext_push(ev, &e);
        }
        return;
    }

    /* Internal in-inode root: keys then pointers. dmxr from fork capacity. */
    {
        int header = 4, key = 8, ptr = 8;
        int dmxr = (forklen - header) / (key + ptr);
        int ptrs_off = header + dmxr * key;
        if (ptrs_off + 8 > forklen) {
            return;
        }
        bno = be64(&fork[ptrs_off]);
    }

    blk = g_malloc(fs->blocksize);

    /* Descend to the leaf level following the first pointer at each node. */
    for (;;) {
        if (bno == 0 || bno == UINT64_MAX) {
            goto done;
        }
        if (blk_pread(fs->blk, fsblock_to_offset(fs, bno), fs->blocksize,
                      blk, 0) < 0) {
            ev->err = true;
            goto done;
        }
        if (be32(&blk[0]) != XFS_BMAP_MAGIC) {
            ev->err = true;
            goto done;
        }
        level = be16(&blk[4]);
        numrecs = be16(&blk[6]);
        if (level == 0) {
            break;
        }
        /* internal node: 24-byte hdr, keys(numrecs*8), ptrs */
        {
            int ptr_start = 24 + numrecs * 8;
            if (ptr_start + 8 > (int)fs->blocksize) {
                ev->err = true;
                goto done;
            }
            bno = be64(&blk[ptr_start]);
        }
    }

    /* Walk the leaf sibling chain to the right. */
    while (bno != 0 && bno != UINT64_MAX) {
        uint64_t rightsib;
        if (have_visited && bno == visited_first) {
            break; /* cycle guard */
        }
        if (!have_visited) {
            visited_first = bno;
            have_visited = true;
        }
        if (blk_pread(fs->blk, fsblock_to_offset(fs, bno), fs->blocksize,
                      blk, 0) < 0) {
            ev->err = true;
            goto done;
        }
        if (be32(&blk[0]) != XFS_BMAP_MAGIC) {
            break;
        }
        numrecs = be16(&blk[6]);
        rightsib = be64(&blk[16]);
        for (i = 0; i < numrecs; i++) {
            int off = 24 + i * 16;
            SGIXfsExtent e;
            if (off + 16 > (int)fs->blocksize) {
                break;
            }
            parse_bmbt(&blk[off], &e);
            ext_push(ev, &e);
        }
        bno = rightsib;
        if (ev->n >= XFS_MAX_EXTENTS) {
            break;
        }
    }

done:
    g_free(blk);
}

/* ---- inode read ---- */

/*
 * Read inode `ino`, fill `out`. Returns 0 ok. On success caller frees via
 * sgi_xfs_inode_put.
 */
static int xfs_read_inode(SGIXfs *fs, uint64_t ino, SGIXfsInode *out)
{
    uint8_t *raw;
    uint16_t magic;
    uint8_t forkoff;
    int dfork_off = XFS_DATA_FORK_OFFSET;
    int dfork_size;
    const uint8_t *fork;

    memset(out, 0, sizeof(*out));
    raw = g_malloc(fs->inodesize);
    if (blk_pread(fs->blk, ino_to_offset(fs, ino), fs->inodesize, raw, 0) < 0) {
        g_free(raw);
        return -1;
    }
    magic = be16(&raw[0]);
    if (magic != XFS_DINODE_MAGIC) {
        qemu_log("sgi_xfs: inode %" PRIu64 " bad magic 0x%04x\n", ino, magic);
        g_free(raw);
        return -2;
    }

    out->mode   = be16(&raw[2]);
    out->format = raw[5];
    out->size   = be64(&raw[56]);
    forkoff     = raw[82];
    dfork_size  = forkoff ? forkoff * 8 : (fs->inodesize - dfork_off);
    if (dfork_off + dfork_size > (int)fs->inodesize) {
        dfork_size = fs->inodesize - dfork_off;
    }
    fork = &raw[dfork_off];

    if (out->format == XFS_DINODE_FMT_LOCAL) {
        int n = out->size;
        if (n < 0 || n > dfork_size) {
            n = dfork_size;
        }
        out->local = g_malloc(n ? n : 1);
        out->local_len = n;
        memcpy(out->local, fork, n);
    } else if (out->format == XFS_DINODE_FMT_EXTENTS) {
        uint32_t nextents = be32(&raw[76]);
        ExtVec ev = { 0 };
        uint32_t i;
        for (i = 0; i < nextents; i++) {
            int off = i * 16;
            SGIXfsExtent e;
            if (off + 16 > dfork_size) {
                break;
            }
            parse_bmbt(&fork[off], &e);
            ext_push(&ev, &e);
        }
        out->extents = ev.v;
        out->nextents = ev.n;
    } else if (out->format == XFS_DINODE_FMT_BTREE) {
        ExtVec ev = { 0 };
        btree_collect(fs, fork, dfork_size, &ev);
        out->extents = ev.v;
        out->nextents = ev.n;
    }
    /* other formats (DEV, UUID): leave empty */

    g_free(raw);
    return 0;
}

void sgi_xfs_inode_put(SGIXfsInode *ino)
{
    if (!ino) {
        return;
    }
    g_free(ino->extents);
    g_free(ino->local);
    ino->extents = NULL;
    ino->local = NULL;
    ino->nextents = 0;
    ino->local_len = 0;
}

/* ---- file read ---- */

int sgi_xfs_read(SGIXfs *fs, const SGIXfsInode *ino, uint64_t off,
                 uint64_t len, void *buf, uint64_t *got)
{
    uint8_t *out = buf;
    uint64_t produced = 0;

    if (got) {
        *got = 0;
    }
    if (off >= ino->size) {
        return 0;
    }
    if (off + len > ino->size) {
        len = ino->size - off;
    }

    if (ino->format == XFS_DINODE_FMT_LOCAL) {
        uint64_t avail = ino->local_len > off ? ino->local_len - off : 0;
        uint64_t n = len < avail ? len : avail;
        memcpy(out, ino->local + off, n);
        if (n < len) {
            memset(out + n, 0, len - n);
        }
        if (got) {
            *got = len;
        }
        return 0;
    }

    /* Extent-mapped: satisfy [off, off+len) block-by-block, zero-filling
     * holes (offsets not covered by any extent). */
    while (produced < len) {
        uint64_t cur = off + produced;
        uint64_t fileblk = cur / fs->blocksize;
        uint64_t blkoff = cur % fs->blocksize;
        int i;
        bool mapped = false;

        for (i = 0; i < ino->nextents; i++) {
            const SGIXfsExtent *e = &ino->extents[i];
            if (fileblk >= e->startoff &&
                fileblk < e->startoff + e->blockcount) {
                uint64_t within = fileblk - e->startoff;   /* blocks into ext */
                uint64_t disk = fsblock_to_offset(fs, e->startblock + within)
                                + blkoff;
                uint64_t ext_left = (e->blockcount - within) * fs->blocksize
                                    - blkoff;
                uint64_t chunk = len - produced;
                if (chunk > ext_left) {
                    chunk = ext_left;
                }
                if (blk_pread(fs->blk, disk, chunk, out + produced, 0) < 0) {
                    return -1;
                }
                produced += chunk;
                mapped = true;
                break;
            }
        }
        if (!mapped) {
            /* hole: zero one block's worth (from blkoff to block end) */
            uint64_t chunk = fs->blocksize - blkoff;
            if (chunk > len - produced) {
                chunk = len - produced;
            }
            memset(out + produced, 0, chunk);
            produced += chunk;
        }
    }

    if (got) {
        *got = produced;
    }
    return 0;
}

/* ---- directory lookup ---- */

/*
 * Find `name` (length namelen) in directory inode `dir`. On success set *child.
 * Supports V1 shortform (LOCAL) and V1 leaf (0xFEEB) block directories.
 * Returns 0 on found, negative on not-found/error.
 */
static int xfs_dir_lookup(SGIXfs *fs, const SGIXfsInode *dir,
                          const char *name, int namelen, uint64_t *child)
{
    if ((dir->mode & S_IFMT_) != S_IFDIR_) {
        return -1;
    }

    if (dir->format == XFS_DINODE_FMT_LOCAL) {
        const uint8_t *d = dir->local;
        int len = dir->local_len;
        int count, off, k;
        if (len < XFS_DIR_SF_HDR_SIZE) {
            return -1;
        }
        count = d[8];
        off = XFS_DIR_SF_HDR_SIZE;
        for (k = 0; k < count; k++) {
            uint64_t ino;
            int nl;
            if (off + 9 > len) {
                break;
            }
            ino = be64(&d[off]);
            nl = d[off + 8];
            if (off + 9 + nl > len) {
                break;
            }
            if (nl == namelen && memcmp(&d[off + 9], name, namelen) == 0) {
                *child = ino;
                return 0;
            }
            off += 9 + nl;
        }
        return -1;
    }

    /* Block-format directory: scan each data block for a V1 leaf. */
    {
        uint8_t *blk = g_malloc(fs->blocksize);
        int i;
        int rc = -1;
        for (i = 0; i < dir->nextents && rc != 0; i++) {
            const SGIXfsExtent *e = &dir->extents[i];
            uint64_t b;
            for (b = 0; b < e->blockcount; b++) {
                uint64_t disk = fsblock_to_offset(fs, e->startblock + b);
                int count, j;
                if (blk_pread(fs->blk, disk, fs->blocksize, blk, 0) < 0) {
                    break;
                }
                if (be16(&blk[8]) != XFS_DIR_LEAF_MAGIC) {
                    continue; /* not a V1 leaf block (dir2 unsupported) */
                }
                count = be16(&blk[12]);
                if (count < 0 || count > 512) {
                    continue;
                }
                for (j = 0; j < count; j++) {
                    int eoff = XFS_DIR_LEAF_HDR_SIZE + j * XFS_DIR_LEAF_ENTRY_SIZE;
                    int nameidx, nl;
                    uint64_t ino;
                    if (eoff + 8 > (int)fs->blocksize) {
                        break;
                    }
                    nameidx = be16(&blk[eoff + 4]);
                    nl = blk[eoff + 6];
                    if (nl == 0 || nameidx + 8 + nl > (int)fs->blocksize) {
                        continue;
                    }
                    ino = be64(&blk[nameidx]);
                    if (nl == namelen &&
                        memcmp(&blk[nameidx + 8], name, namelen) == 0 &&
                        ino > 0) {
                        *child = ino;
                        rc = 0;
                        break;
                    }
                }
                if (rc == 0) {
                    break;
                }
            }
        }
        g_free(blk);
        return rc;
    }
}

int sgi_xfs_lookup(SGIXfs *fs, const char *path, SGIXfsInode *out)
{
    SGIXfsInode cur;
    const char *p = path;
    int rc;

    if (!fs->mounted) {
        return -1;
    }

    rc = xfs_read_inode(fs, fs->rootino, &cur);
    if (rc < 0) {
        return rc;
    }

    /* Walk each '/'-separated component. */
    while (*p) {
        char comp[256];
        int nl = 0;
        uint64_t child;

        while (*p == '/') {
            p++;
        }
        if (!*p) {
            break;
        }
        while (*p && *p != '/' && nl < (int)sizeof(comp) - 1) {
            comp[nl++] = *p++;
        }
        comp[nl] = '\0';
        while (*p && *p != '/') { /* overlong component: skip remainder */
            p++;
        }

        rc = xfs_dir_lookup(fs, &cur, comp, nl, &child);
        sgi_xfs_inode_put(&cur);
        if (rc < 0) {
            return -2; /* not found */
        }
        rc = xfs_read_inode(fs, child, &cur);
        if (rc < 0) {
            return rc;
        }
    }

    *out = cur;
    return 0;
}
