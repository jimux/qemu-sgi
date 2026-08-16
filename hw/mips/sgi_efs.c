/*
 * Host-side read-only EFS (Extent File System) reader — see include/hw/mips/sgi_efs.h.
 *
 * Ported from the on-disk layout in pyirix.efs.reader (validated against the
 * 6.5.5 Installation-Tools CD). Covers what the Mode C install boot needs:
 * superblock, inode read (128-byte, 12 direct + indirect extents), 0xBEEF slot
 * directory lookup, and file read. Read-only.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/mips/sgi_efs.h"
#include "system/block-backend.h"

/* On-disk constants (pyirix.efs.reader). */
#define EFS_MAGIC        0x072959u
#define EFS_MAGIC_NEW    0x07295Au
#define EFS_BLOCK_SIZE   512
#define EFS_INODE_SIZE   128
#define EFS_INOPBB       4          /* inodes per basic block */
#define EFS_ROOT_INODE   2
#define EFS_MAX_EXTENTS  12
#define EFS_DIRBLK_MAGIC 0xBEEFu

#define S_IFMT_  0170000
#define S_IFDIR_ 0040000
#define S_IFREG_ 0100000

/* Guardrails so a corrupt/hostile disk can't make us loop or allocate wildly. */
#define EFS_MAX_TOTAL_EXTENTS (256 * 1024)
#define EFS_MAX_PATH_COMP     64

static uint16_t be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

/* ---- address conversion (pyirix.efs.reader.inode_to_bb) ---- */

static uint64_t ino_to_block(const SGIEFS *fs, uint64_t ino, int *slot)
{
    uint64_t ipcg = (uint64_t)fs->fs_cgisize * EFS_INOPBB; /* inodes per cg */
    uint64_t cg = ino / ipcg;
    uint64_t cgbb = (ino >> 2) % fs->fs_cgisize; /* INOPBBSHIFT = 2 */
    *slot = ino & (EFS_INOPBB - 1);
    return fs->fs_firstcg + cg * fs->fs_cgfsize + cgbb;
}

/* Decode an 8-byte EFS extent: magic[1] bn[3BE] length[1] offset[2BE]. */
static void parse_extent(const uint8_t *p, SGIEFSExtent *e)
{
    uint32_t w1 = be32(p);
    uint32_t w2 = be32(p + 4);
    e->bn     = w1 & 0xFFFFFF;
    e->length = (uint8_t)(w2 >> 24);
    e->offset = w2 & 0xFFFFFF;
}

/* ---- mount ---- */

int sgi_efs_mount(BlockBackend *blk, uint64_t part_first_lbn, SGIEFS *fs)
{
    uint8_t sb[EFS_BLOCK_SIZE];
    uint32_t magic;

    memset(fs, 0, sizeof(*fs));
    fs->blk = blk;
    fs->part_byte_off = part_first_lbn * 512ULL;

    /* Superblock is at block 1 (offset 512) of the partition. */
    if (blk_pread(blk, fs->part_byte_off + EFS_BLOCK_SIZE, EFS_BLOCK_SIZE,
                  sb, 0) < 0) {
        error_report("sgi_efs: cannot read superblock");
        return -1;
    }
    magic = be32(sb + 28);
    if (magic != EFS_MAGIC && magic != EFS_MAGIC_NEW) {
        error_report("sgi_efs: bad magic 0x%08x", magic);
        return -1;
    }
    fs->fs_firstcg = be32(sb + 4);
    fs->fs_cgfsize = be32(sb + 8);
    fs->fs_cgisize = be16(sb + 12);
    fs->fs_ncg     = be16(sb + 18);

    if (fs->fs_cgisize == 0 || fs->fs_cgfsize == 0) {
        error_report("sgi_efs: implausible geometry (cgisize=%u cgfsize=%u)",
                     fs->fs_cgisize, fs->fs_cgfsize);
        return -1;
    }
    fs->mounted = true;
    qemu_log("sgi_efs: mounted (firstcg=%u cgfsize=%u cgisize=%u ncg=%u)\n",
             fs->fs_firstcg, fs->fs_cgfsize, fs->fs_cgisize, fs->fs_ncg);
    return 0;
}

/* ---- inode read ---- */

/*
 * Read inode `ino`, filling `out` (mode/size + decoded extents, incl. indirect
 * extents when numextents > EFS_MAX_EXTENTS). On success the caller frees via
 * sgi_efs_inode_put.
 */
static int efs_read_inode(SGIEFS *fs, uint64_t ino, SGIEFSInode *out)
{
    uint8_t raw[EFS_INODE_SIZE];
    uint8_t blk[EFS_BLOCK_SIZE];
    uint64_t bb;
    int slot;
    uint16_t mode, numextents;
    int i;

    memset(out, 0, sizeof(*out));

    bb = ino_to_block(fs, ino, &slot);
    if (blk_pread(fs->blk, fs->part_byte_off + bb * EFS_BLOCK_SIZE,
                  EFS_BLOCK_SIZE, blk, 0) < 0) {
        error_report("sgi_efs: cannot read inode block for %" PRIu64, ino);
        return -1;
    }
    memcpy(raw, blk + slot * EFS_INODE_SIZE, EFS_INODE_SIZE);

    mode = be16(raw + 0);
    if (mode == 0) {
        return -1; /* free inode */
    }
    out->mode = mode;
    out->size = (uint32_t)be32(raw + 8);
    numextents = be16(raw + 28);

    if (numextents <= EFS_MAX_EXTENTS) {
        out->nextents = numextents;
        out->extents = g_malloc0(sizeof(SGIEFSExtent) * (numextents ? numextents : 1));
        for (i = 0; i < numextents; i++) {
            parse_extent(raw + 32 + i * 8, &out->extents[i]);
        }
        return 0;
    }

    /*
     * Indirect extents: the direct slots point to blocks holding the extent
     * table; the FIRST extent's `offset` field is the indirect-block count
     * (SGI convention). Read those blocks and decode `numextents` extents.
     */
    {
        uint8_t *ind;
        uint64_t ind_bytes;
        int num_indirect;
        int j;
        uint64_t got;

        /* First direct extent's offset field holds the indirect-block count. */
        {
            SGIEFSExtent first;
            parse_extent(raw + 32, &first);
            num_indirect = (int)first.offset;
        }
        if (num_indirect <= 0 || num_indirect > EFS_MAX_EXTENTS) {
            error_report("sgi_efs: inode %" PRIu64 " bad indirect count %d",
                         ino, num_indirect);
            return -1;
        }
        ind_bytes = 0;
        for (i = 0; i < num_indirect; i++) {
            SGIEFSExtent e;
            parse_extent(raw + 32 + i * 8, &e);
            ind_bytes += (uint64_t)e.length * EFS_BLOCK_SIZE;
        }
        if (ind_bytes == 0 || ind_bytes > 64 * 1024 * 1024) {
            error_report("sgi_efs: inode %" PRIu64 " implausible indirect size %"
                         PRIu64, ino, ind_bytes);
            return -1;
        }
        ind = g_malloc(ind_bytes);
        got = 0;
        for (i = 0; i < num_indirect; i++) {
            SGIEFSExtent e;
            parse_extent(raw + 32 + i * 8, &e);
            if (blk_pread(fs->blk, fs->part_byte_off + e.bn * EFS_BLOCK_SIZE,
                          (uint64_t)e.length * EFS_BLOCK_SIZE,
                          ind + got, 0) < 0) {
                error_report("sgi_efs: indirect extent read failed");
                g_free(ind);
                return -1;
            }
            got += (uint64_t)e.length * EFS_BLOCK_SIZE;
        }
        out->nextents = numextents;
        out->extents = g_malloc0(sizeof(SGIEFSExtent) * numextents);
        for (j = 0; j < numextents; j++) {
            parse_extent(ind + j * 8, &out->extents[j]);
        }
        g_free(ind);
    }
    return 0;
}

void sgi_efs_inode_put(SGIEFSInode *ino)
{
    g_free(ino->extents);
    ino->extents = NULL;
    ino->nextents = 0;
}

/* ---- directory lookup ---- */

/* Read all directory entries of a directory inode into (name, ino) pairs. */
static int efs_read_dir(SGIEFS *fs, const SGIEFSInode *dir,
                        int (*cb)(const char *name, uint64_t ino, void *opaque),
                        void *opaque)
{
    SGIEFSExtent *extents;
    int nextents;
    int i, j;

    /* Sort extents by offset so directory blocks come out in order. */
    extents = g_memdup2(dir->extents, sizeof(SGIEFSExtent) * (dir->nextents ? dir->nextents : 1));
    nextents = dir->nextents;
    /* insertion sort by offset */
    for (i = 1; i < nextents; i++) {
        SGIEFSExtent key = extents[i];
        j = i - 1;
        while (j >= 0 && extents[j].offset > key.offset) {
            extents[j + 1] = extents[j];
            j--;
        }
        extents[j + 1] = key;
    }

    for (i = 0; i < nextents; i++) {
        uint64_t ext_bytes = (uint64_t)extents[i].length * EFS_BLOCK_SIZE;
        uint8_t *data = g_malloc(ext_bytes ? ext_bytes : 1);
        if (blk_pread(fs->blk, fs->part_byte_off + extents[i].bn * EFS_BLOCK_SIZE,
                      ext_bytes, data, 0) < 0) {
            g_free(data);
            g_free(extents);
            return -1;
        }
        /* walk each 512-byte dir block inside the extent */
        for (uint64_t bo = 0; bo + EFS_BLOCK_SIZE <= ext_bytes;
             bo += EFS_BLOCK_SIZE) {
            uint8_t *db = data + bo;
            if (be16(db) != EFS_DIRBLK_MAGIC) {
                continue;
            }
            int firstused = db[2];
            int slots = db[3];
            for (int s = 0; s < slots && 4 + s < EFS_BLOCK_SIZE; s++) {
                int sv = db[4 + s];
                if (sv < firstused) {
                    continue;
                }
                int eoff = sv * 2;
                if (eoff + 5 > EFS_BLOCK_SIZE) {
                    continue;
                }
                uint64_t cino = be32(db + eoff);
                int namelen = db[eoff + 4];
                if (eoff + 5 + namelen > EFS_BLOCK_SIZE) {
                    continue;
                }
                char name[256];
                if (namelen >= (int)sizeof(name)) {
                    continue;
                }
                memcpy(name, db + eoff + 5, namelen);
                name[namelen] = '\0';
                if (name[0] == '.' && (namelen == 1 ||
                    (namelen == 2 && name[1] == '.'))) {
                    continue;
                }
                if (cb(name, cino, opaque) != 0) {
                    g_free(data);
                    g_free(extents);
                    return 1; /* found */
                }
            }
        }
        g_free(data);
    }
    g_free(extents);
    return 0;
}

struct lookup_ctx {
    const char *component;
    uint64_t found_ino;
};

static int lookup_cb(const char *name, uint64_t ino, void *opaque)
{
    struct lookup_ctx *c = opaque;
    if (strcmp(name, c->component) == 0) {
        c->found_ino = ino;
        return 1;
    }
    return 0;
}

int sgi_efs_lookup(SGIEFS *fs, const char *path, SGIEFSInode *out)
{
    char buf[512];
    const char *p;
    uint64_t ino = EFS_ROOT_INODE;
    int depth = 0;

    if (!fs->mounted) {
        return -1;
    }
    /* Strip leading slashes and split into components. */
    while (*path == '/') {
        path++;
    }
    if (*path == '\0') {
        return efs_read_inode(fs, EFS_ROOT_INODE, out);
    }
    p = path;
    while (*p) {
        const char *slash = strchr(p, '/');
        size_t n = slash ? (size_t)(slash - p) : strlen(p);
        struct lookup_ctx ctx;
        SGIEFSInode dir;

        if (n == 0 || n >= sizeof(buf) - 1 || ++depth > EFS_MAX_PATH_COMP) {
            return -1;
        }
        memcpy(buf, p, n);
        buf[n] = '\0';

        if (efs_read_inode(fs, ino, &dir) < 0) {
            return -1;
        }
        if ((dir.mode & S_IFMT_) != S_IFDIR_) {
            sgi_efs_inode_put(&dir);
            return -1;
        }
        ctx.component = buf;
        ctx.found_ino = 0;
        if (efs_read_dir(fs, &dir, lookup_cb, &ctx) != 1) {
            sgi_efs_inode_put(&dir);
            return -1;
        }
        sgi_efs_inode_put(&dir);
        ino = ctx.found_ino;
        p = slash ? slash + 1 : p + n;
    }
    return efs_read_inode(fs, ino, out);
}

/* ---- file read ---- */

int sgi_efs_read(SGIEFS *fs, const SGIEFSInode *ino, uint64_t off,
                 uint64_t len, void *buf, uint64_t *got)
{
    SGIEFSExtent *extents;
    int nextents;
    uint64_t produced = 0;
    int i, j;

    *got = 0;
    if (off >= ino->size) {
        return 0;
    }
    if (len > ino->size - off) {
        len = ino->size - off;
    }

    /* Sort extents by offset. */
    extents = g_memdup2(ino->extents, sizeof(SGIEFSExtent) * (ino->nextents ? ino->nextents : 1));
    nextents = ino->nextents;
    for (i = 1; i < nextents; i++) {
        SGIEFSExtent key = extents[i];
        j = i - 1;
        while (j >= 0 && extents[j].offset > key.offset) {
            extents[j + 1] = extents[j];
            j--;
        }
        extents[j + 1] = key;
    }

    /* Skip extents fully before `off`, then copy. */
    for (i = 0; i < nextents && produced < len; i++) {
        uint64_t ext_bytes = (uint64_t)extents[i].length * EFS_BLOCK_SIZE;
        uint64_t ext_file_off = extents[i].offset * EFS_BLOCK_SIZE;
        uint64_t ext_end = ext_file_off + ext_bytes;

        if (ext_end <= off) {
            continue;
        }
        if (ext_file_off >= off + len) {
            break;
        }
        {
            uint64_t skip = off > ext_file_off ? off - ext_file_off : 0;
            uint64_t take = ext_bytes - skip;
            uint64_t want = len - produced;
            if (take > want) {
                take = want;
            }
            if (blk_pread(fs->blk,
                          fs->part_byte_off + extents[i].bn * EFS_BLOCK_SIZE + skip,
                          take, (uint8_t *)buf + produced, 0) < 0) {
                g_free(extents);
                return -1;
            }
            produced += take;
        }
    }
    g_free(extents);
    *got = produced;
    return 0;
}
