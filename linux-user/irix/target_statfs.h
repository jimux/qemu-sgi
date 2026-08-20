/*
 *  IRIX statfs(2) marshalling for linux-user
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * ---------------------------------------------------------------------------
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * IRIX `df` printed structurally-correct output with nonsense numbers
 * (5516700842977591296 blocks) because `statfs` was falling through to
 * linux-user's generic handler, which marshals Linux/MIPS-N32's
 * `struct target_statfs` -- an entirely different object from IRIX's. IRIX's
 * has a 2-byte `f_fstyp` where Linux has a 4-byte `f_type`, 64-bit counters
 * where Linux has 32-bit ones, no `f_bavail`/`f_fsid`/`f_namelen`/`f_flags`
 * at all, and 12 bytes of volume/pack name at the end. It is also 64 bytes
 * where the Linux N32 struct is 68, so the generic path wrote four bytes past
 * the guest's object.
 *
 * The aliasing was exact enough to reconstruct: the old output equals
 * ((linux_f_bfree << 32) | linux_f_files) * (f_frsize/512), because IRIX reads
 * a 64-bit `f_blocks` at offset 16 where Linux wrote `f_bfree` at 16 and
 * `f_files` at 20.
 *
 * THE LAYOUT, TRANSCRIBED
 * -----------------------
 * software_library/irix-655-source/f/root/usr/include/sys/statfs.h:35-51
 *
 *     #if (_MIPS_SIM == _ABIN32) || defined(_KERNEL)
 *     typedef __int64_t _statfs_bc_t, _statfs_fc_t;
 *     #else
 *     typedef long      _statfs_bc_t, _statfs_fc_t;
 *     #endif
 *
 *     struct statfs {
 *             short        f_fstyp;    // File system type
 *             long         f_bsize;    // Block size
 *             long         f_frsize;   // Fragment size (if supported)
 *             _statfs_bc_t f_blocks;   // Total number of blocks
 *             _statfs_bc_t f_bfree;    // Total number of free blocks
 *             _statfs_fc_t f_files;    // Total number of file nodes (inodes)
 *             _statfs_fc_t f_ffree;    // Total number of free file nodes
 *             char         f_fname[6]; // Volume name
 *             char         f_fpack[6]; // Pack name
 *     };
 *     extern int statfs(const char *, struct statfs *, int, int);
 *
 * so for n32: f_fstyp @0 (2 bytes + 2 pad), f_bsize @4, f_frsize @8, 4 bytes
 * of pad @12 for the 8-byte alignment of the counters, f_blocks @16,
 * f_bfree @24, f_files @32, f_ffree @40, f_fname @48, f_fpack @54, and 4 bytes
 * of tail pad: sizeof == 64. The same header's `struct irix5_n32_statfs`
 * (statfs.h:69-80) is the kernel's own copy-out struct for an n32 caller and
 * confirms it field for field.
 *
 * Independently confirmed against the binary that consumes it. The staged
 * sysroot's `sbin/df` (the only statfs caller -- its sole relevant dynamic
 * import is `statfs`, and there is no `statvfs`) does:
 *
 *   10003894  jalr t9              ; statfs(name, buf, 64, fstyp)
 *   10003898  li   a2,64           ;   <- sizeof(struct statfs) == 64
 *   10003834  lh   s0,0(s2)        ; f_fstyp as a *short* at offset 0
 *   100031d4  lw   v0,8(sp)        ; f_frsize, 32-bit, at offset 8
 *   100031e4  ld   v1,16(sp)       ; f_blocks, 64-bit, at offset 16
 *   100031f4  ld   v0,24(sp)       ; f_bfree,  64-bit, at offset 24
 *
 * THE FOUR-ARGUMENT SIGNATURE
 * ---------------------------
 * IRIX's statfs takes a length and a filesystem type, and the kernel honours
 * both (f/irix/kern/os/vfs.c:651-652):
 *
 *     if (len < 0 || len > sizeof(struct statfs))
 *             return EINVAL;
 *     ...
 *     return copyout(&sfs, sbp, len);
 *
 * so we validate `len` the same way and copy out exactly that many bytes
 * rather than assuming the guest passed sizeof.
 *
 * `fstyp` is only consulted when the path names an *unmounted* filesystem
 * device (vfs.c:657-671, "hack for backward compat ... only works for efs and
 * xfs"). There are no IRIX filesystem devices on a Linux host, so we ignore it
 * -- df passes 0 for every mounted entry (df.c:763) and only walks nonzero
 * type indices for a raw device argument, which cannot succeed here anyway.
 *
 * WHAT WE FILL IN, AND WHAT WE HONESTLY LEAVE EMPTY
 * -------------------------------------------------
 * The block accounting is transcribed from the kernel's own statvfs->statfs
 * conversion (vfs.c:676-692), which is why this handler calls the host's
 * statvfs() rather than statfs(): statvfs is the POSIX interface with exactly
 * the f_bsize/f_frsize/f_blocks semantics IRIX's own VFS layer feeds into
 * this struct, so the arithmetic below is the kernel's, unchanged.
 *
 * `f_fstyp` is 0. It is an index into the running kernel's `vfssw[]` table
 * (vfs.c:713-717 `sfs.f_fstyp = vswp - vfssw`, and 0 when the base type is not
 * in the table), which has no meaning on a Linux host -- there is no honest
 * number to put there, and 0 is the value the IRIX kernel itself uses for
 * "not a type I know". This costs nothing in practice: df takes the type name
 * from /etc/mtab and only falls back to sysfs(GETFSTYP, f_fstyp) when the mtab
 * entry has no type field (df.c:812-819).
 *
 * `f_fname`/`f_fpack` are left NUL. On IRIX they are carved out of
 * statvfs's `f_fstr` (vfs.c:695-712), the volume and pack name, which Linux's
 * statvfs does not report at all. The IRIX kernel bzero()s the struct first
 * (vfs.c:675) so an empty name is a state it produces itself.
 */
#ifndef IRIX_TARGET_STATFS_H
#define IRIX_TARGET_STATFS_H

/* f/root/usr/include/sys/statfs.h:41-51, n32 (_statfs_bc_t = __int64_t) */
struct target_irix_statfs {
    abi_short   f_fstyp;        /*  0 */
    abi_short   __pad0;         /*  2 */
    abi_int     f_bsize;        /*  4 */
    abi_int     f_frsize;       /*  8 */
    abi_int     __pad1;         /* 12 */
    abi_llong   f_blocks;       /* 16 */
    abi_llong   f_bfree;        /* 24 */
    abi_llong   f_files;        /* 32 */
    abi_llong   f_ffree;        /* 40 */
    char        f_fname[6];     /* 48 */
    char        f_fpack[6];     /* 54 */
    abi_int     __pad2;         /* 60 -> sizeof 64 */
};

static abi_long host_to_target_irix_statfs(abi_ulong target_addr, abi_long len,
                                           const struct statvfs *hst)
{
    struct target_irix_statfs sfs;
    void *p;

    /* vfs.c:651-652 */
    if (len < 0 || len > (abi_long)sizeof(sfs)) {
        return -TARGET_EINVAL;
    }
    if (len == 0) {
        return 0;
    }

    /* vfs.c:675, the kernel zeroes the whole struct before filling it */
    memset(&sfs, 0, sizeof(sfs));

    /* vfs.c:676-692, verbatim -- note that when the fragment size equals the
     * block size the kernel leaves f_frsize at its bzero'd 0, which df reads
     * as "no fragment scaling" (df.c:628-629). */
    __put_user(hst->f_bsize, &sfs.f_bsize);
    if (hst->f_frsize != hst->f_bsize && hst->f_frsize != 0) {
        uint64_t mult = hst->f_bsize / hst->f_frsize;
        __put_user(hst->f_frsize, &sfs.f_frsize);
        __put_user((uint64_t)hst->f_blocks * mult, &sfs.f_blocks);
        __put_user((uint64_t)hst->f_bfree * mult, &sfs.f_bfree);
    } else {
        __put_user((uint64_t)hst->f_blocks, &sfs.f_blocks);
        __put_user((uint64_t)hst->f_bfree, &sfs.f_bfree);
    }
    __put_user((uint64_t)hst->f_files, &sfs.f_files);
    __put_user((uint64_t)hst->f_ffree, &sfs.f_ffree);

    p = lock_user(VERIFY_WRITE, target_addr, len, 0);
    if (!p) {
        return -TARGET_EFAULT;
    }
    memcpy(p, &sfs, len);
    unlock_user(p, target_addr, len);
    return 0;
}

#endif /* IRIX_TARGET_STATFS_H */
