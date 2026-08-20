/*
 *  IRIX pseudo-terminal translation for linux-user
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
 * WHAT IRIX ACTUALLY DOES
 * -----------------------
 * IRIX has no /dev/ptmx and no /dev/pts/N pair-per-open model. Every IRIX
 * terminal emulator goes through libc's _getpty(3), and the whole protocol is
 * visible in the shipped source, transcribed here rather than inferred:
 *
 *   software_library/irix-655-source/f/irix/lib/libc/src/gen/getpty.c
 *
 *     :140      open(_PATH_DEVPTC, oflag)          -- the *clone* device
 *     :205-211  fstat(masterfd); mc = minor(st_rdev); require mc < 200
 *     :212-236  switch (major(st_rdev)) { PTC_MAJOR -> slv_dev =
 *                 makedev(PTS_MAJOR, mc), fam = 0; PTC1..PTC4 -> fam 2,4,6,8 }
 *     :237-241  fam += mc / 100; mc %= 100;
 *               sprintf(slave0, "/dev/tty%c%d", "qrstuvwxyz"[fam], mc);
 *               sprintf(slave1, "/dev/pts/%d", fam * 100 + mc);
 *     :246-325  for BOTH name forms: stat() it, mknod()/link() it if absent,
 *               require st_rdev == slv_dev, chown() it to (getuid(), pty_gid),
 *               chmod() it to the requested mode
 *     :146      unlockpt(masterfd) -> ioctl(fd, I_STR, {ic_cmd = UNLKPT})
 *
 *   f/root/usr/include/paths.h:52         _PATH_DEVPTC "/dev/ptc"
 *   f/irix/kern/sys/major.h:35-36         PTC_MAJOR 14, PTS_MAJOR 15
 *   f/irix/kern/sys/major.h:86            PTC1_MAJOR 104 (and PTS1..PTC4 on)
 *   f/root/usr/include/sys/sysmacros.h
 *              :128-135                   L_BITSMINOR 18, L_MAXMIN 0x3ffff
 *              :152-160                   major(x) = (x >> 18) & 0x1ff
 *                                         minor(x) = x & 0x3ffff
 *                                         makedev(x,y) = (x << 18) | y
 *   f/root/usr/include/sys/stropts.h:147  STR ('S' << 8)
 *                                  :156  I_STR (STR|010) == 0x5308
 *                              :207-212  struct strioctl
 *                                          { int ic_cmd; int ic_timout;
 *                                            int ic_len; char *ic_dp; }
 *   f/root/usr/include/sys/termios.h
 *              :520-523                   ISPTM  (('P'<<8)|1)   == 0x5001
 *                                         UNLKPT (('P'<<8)|2)   == 0x5002
 *                                         SVR4SOPEN (('P'<<8)|100) == 0x5064
 *
 * Confirmed live: /home/jimmy/qemu-sgi/tmp/legT2/xterm-strace-before.txt ends
 *
 *     open("/dev/tty",O_RDWR) = -1 errno=6
 *     open("/dev/ptc",O_RDWR) = -1 errno=2
 *     write(2,...)xterm: no available ptys
 *
 * so /dev/ptc is the single door, exactly as getpty.c says. Both terminal
 * emulators in the staged root reach it through libc: `strings` on
 * usr/sbin/xwsh and usr/bin/X11/xterm both show the _getpty import, and
 * neither contains a /dev/pty?? scan string that it actually uses first.
 *
 * WHAT WE DO INSTEAD
 * ------------------
 * Nothing here emulates STREAMS. The whole IRIX dance is *replaced* by the
 * host's Unix98 pty machinery, and the guest is told the answers it expects:
 *
 *   open("/dev/ptc")        -> open("/dev/ptmx"), then unlockpt() eagerly
 *   fstat(master)           -> st_rdev rewritten to makedev(PTC_MAJOR, n)
 *   "/dev/ttyq<n>"          -> the host "/dev/pts/<n>" node, for open and stat
 *   "/dev/pts/<n>"          -> itself, but with the IRIX st_rdev
 *   stat(slave)             -> st_rdev rewritten to makedev(PTS_MAJOR, n)
 *   chown/chmod(slave)      -> success, no-op (see below)
 *   ioctl(I_STR, UNLKPT)    -> host unlockpt()
 *   ioctl(I_STR, ISPTM)     -> success iff the fd really is a pty master
 *   ioctl(I_STR, SVR4SOPEN) -> success, no-op (it only selects a name form)
 *
 * THE MINOR NUMBER IS THE HOST PTY INDEX, IDENTICALLY.
 * There is no mapping table, deliberately. IRIX's own limit is mc < 200
 * (getpty.c:208), so a host devpts index below 200 can be handed to the guest
 * verbatim; the guest then derives the slave name from it with the arithmetic
 * above and we invert that arithmetic to get back to /dev/pts/<n>. Being
 * stateless means the translation also holds across fork *and* execve, and for
 * a pty inherited by an unrelated process -- which a table in this process's
 * memory would not survive. If the host hands out an index >= 200 (i.e. 200+
 * concurrent ptys already open on the machine) we fail the /dev/ptc open with
 * EAGAIN, which is precisely the condition IRIX itself reports as
 * "no available ptys"; the alternative would be to lie about the minor and
 * then be unable to invert it.
 *
 * WHY chown/chmod ON A SLAVE ARE NO-OPS.
 * getpty.c:300-325 exists to take a BSD-era world-accessible /dev/ttyq* node
 * and hand it to one user for the life of the session. The host's devpts does
 * that for us at open time: the slave is created owned by the opener and
 * (mode 620, group tty) not accessible to other users. So the ownership dance
 * has nothing left to accomplish, while executing it literally would fail --
 * chown() to gid 0 needs CAP_CHOWN we do not have, and getpty.c reacts to that
 * failure by exec'ing the setuid helper /usr/sbin/mkpts, which cannot work
 * either. Reporting success is not a lie about the outcome: after the call the
 * slave *is* private to this user, which is what the caller wanted.
 */
#ifndef IRIX_TARGET_PTY_H
#define IRIX_TARGET_PTY_H

/* f/root/usr/include/paths.h:52 */
#define TARGET_IRIX_PATH_DEVPTC     "/dev/ptc"

/* f/irix/kern/sys/major.h:35-36 */
#define TARGET_IRIX_PTC_MAJOR       14
#define TARGET_IRIX_PTS_MAJOR       15

/* f/root/usr/include/sys/sysmacros.h:128-160 */
#define TARGET_IRIX_L_BITSMINOR     18
#define TARGET_IRIX_L_MAXMIN        0x3ffff
#define target_irix_makedev(maj, min)                                   \
    ((((uint32_t)(maj)) << TARGET_IRIX_L_BITSMINOR) |                   \
     ((uint32_t)(min) & TARGET_IRIX_L_MAXMIN))

/* getpty.c:208 -- the guest itself rejects a minor >= 200 */
#define TARGET_IRIX_PTY_MAXMINOR    200

/* f/root/usr/include/sys/termios.h:520-523, carried on the I_STR ic_cmd */
#define TARGET_IRIX_ISPTM           0x5001
#define TARGET_IRIX_UNLKPT          0x5002
#define TARGET_IRIX_SVR4SOPEN       0x5064

/* f/root/usr/include/sys/stropts.h:207-212, n32 layout (ic_dp is 4 bytes) */
struct target_irix_strioctl {
    abi_int   ic_cmd;
    abi_int   ic_timout;
    abi_int   ic_len;
    abi_uint  ic_dp;
};

/*
 * The host devpts index of a pty master, or -1 if this fd is not one.
 * TIOCGPTN only succeeds on a master, so this doubles as the "is it a master"
 * test and needs no bookkeeping of our own.
 */
static int irix_pty_master_index(int fd)
{
    unsigned int n;

    if (ioctl(fd, TIOCGPTN, &n) < 0) {
        return -1;
    }
    return (int)n;
}

/*
 * Decode an IRIX pty slave node name into its minor number, inverting
 * getpty.c:237-241. Returns the minor, or -1 if the path is not one of the two
 * IRIX slave name forms.
 *
 *   form 0: /dev/tty<fam><mc>   fam in "qrstuvwxyz", 0 <= mc < 100
 *           minor = fam * 100 + mc
 *   form 1: /dev/pts/<n>        minor = n
 */
static int irix_pty_slave_minor(const char *guest_path)
{
    const char *s;
    char *end;
    long n;

    if (!guest_path) {
        return -1;
    }

    if (!strncmp(guest_path, "/dev/pts/", 9)) {
        s = guest_path + 9;
        if (*s < '0' || *s > '9') {
            return -1;
        }
        n = strtol(s, &end, 10);
        if (*end != '\0' || n < 0 || n >= TARGET_IRIX_PTY_MAXMINOR) {
            return -1;
        }
        return (int)n;
    }

    if (!strncmp(guest_path, "/dev/tty", 8)) {
        int fam = guest_path[8];
        if (fam < 'q' || fam > 'z') {
            return -1;
        }
        s = guest_path + 9;
        if (*s < '0' || *s > '9') {
            return -1;
        }
        n = strtol(s, &end, 10);
        if (*end != '\0' || n < 0 || n >= 100) {
            return -1;
        }
        n += (fam - 'q') * 100;
        if (n >= TARGET_IRIX_PTY_MAXMINOR) {
            return -1;
        }
        return (int)n;
    }

    return -1;
}

/*
 * Translate an IRIX pty slave node name to the host devpts path. Returns 1 and
 * fills @out (plus @minor if non-NULL) when the name is one we own, else 0.
 *
 * The node has to exist on the host for us to claim it: a bare
 * stat("/dev/ttyq3") on a machine with no third pty open must still fail
 * ENOENT rather than be answered out of thin air.
 */
static int irix_pty_hostpath(const char *guest_path, char *out, size_t outlen,
                             int *minor)
{
    struct stat st;
    int m = irix_pty_slave_minor(guest_path);

    if (m < 0) {
        return 0;
    }
    if (snprintf(out, outlen, "/dev/pts/%d", m) >= (int)outlen) {
        return 0;
    }
    if (stat(out, &st) < 0 || !S_ISCHR(st.st_mode)) {
        return 0;
    }
    if (minor) {
        *minor = m;
    }
    return 1;
}

/*
 * Give the caller a controlling terminal, if it has none.
 *
 * This is the one place where the host tty layer is genuinely less generous
 * than IRIX's, and it is the difference between an xterm that maps and an
 * xterm that runs a shell. Under SVR4 STREAMS, opening a pty slave acquires it
 * as the controlling terminal; xterm's forked child relies on exactly that --
 * it never calls setsid(), it opens the slave and then probes its own success
 * with open("/dev/tty"). Traced without this call, the child walks every slave
 * node in turn, gets ENXIO from /dev/tty on each, and eventually reports
 * "child process can find no available ptys" having successfully opened 196 of
 * them (tmp/legT2/hx.799724). Linux never makes a tty the controlling terminal
 * on open() unless the opener is already a session leader, and a forked child
 * is not one, so the guest's assumption silently fails.
 *
 * So do what the IRIX slave-open does: setsid() to become a session leader and
 * TIOCSCTTY to claim the terminal. Both are gated on the process having NO
 * controlling terminal already -- this only ever adds one, never steals or
 * replaces one -- and O_NOCTTY is honoured, so a caller that explicitly does
 * not want a controlling terminal does not get one. Claiming session
 * leadership is not a lie to the guest either: under the semantics the guest
 * believes it is running, acquiring a controlling terminal implies it.
 */
static void irix_pty_acquire_ctty(int fd)
{
    int t = open("/dev/tty", O_RDWR | O_NOCTTY);

    if (t >= 0) {           /* already has one: leave everything alone */
        close(t);
        return;
    }
    if (errno != ENXIO) {
        return;
    }
    /*
     * setsid() fails with EPERM when we are already a process-group leader,
     * and then TIOCSCTTY will fail too; there is nothing better to do than
     * leave the guest with the terminal but no /dev/tty, which is what it has
     * today. Both are best-effort by design.
     */
    (void)setsid();
    (void)ioctl(fd, TIOCSCTTY, 0);
}

/*
 * open() hook. Returns a host fd, or -1 with errno set, or -2 for "not a pty
 * path, carry on normally" (the same three-way convention maybe_do_fake_open()
 * uses).
 */
static int irix_pty_open(const char *guest_path, int flags, mode_t mode)
{
    char hostpath[64];
    int fd, n;

    if (!guest_path) {
        return -2;
    }

    if (!strcmp(guest_path, TARGET_IRIX_PATH_DEVPTC)) {
        fd = open("/dev/ptmx", flags);
        if (fd < 0) {
            return -1;
        }
        n = irix_pty_master_index(fd);
        if (n < 0 || n >= TARGET_IRIX_PTY_MAXMINOR) {
            /*
             * Either it is not really a ptmx (a host without devpts) or the
             * index is outside the range IRIX can name. Both are genuinely
             * "no pty available" as far as the guest is concerned.
             */
            qemu_log_mask(LOG_UNIMP,
                          "irix: /dev/ptc: host pty index %d unusable "
                          "(IRIX minors are limited to < %d)\n",
                          n, TARGET_IRIX_PTY_MAXMINOR);
            close(fd);
            errno = EAGAIN;
            return -1;
        }
        /*
         * Unlock now rather than waiting for the guest's unlockpt(): callers
         * that skip it (or that inherit the master across an exec) would
         * otherwise get EIO when they open the slave. Unlocking early is
         * strictly permissive -- IRIX has no locked-pty state to observe.
         */
        unlockpt(fd);
        return fd;
    }

    if (irix_pty_hostpath(guest_path, hostpath, sizeof(hostpath), NULL)) {
        fd = open(hostpath, flags, mode);
        if (fd >= 0 && !(flags & O_NOCTTY)) {
            int err = errno;
            irix_pty_acquire_ctty(fd);
            errno = err;
        }
        return fd;
    }

    return -2;
}

/*
 * stat()/lstat() hook. Returns 1 when the path is one of ours, with the host
 * result in *res (0 or -1/errno) and @st filled in and rewritten into IRIX's
 * device numbering; 0 when the path is not ours.
 */
static int irix_pty_stat(const char *guest_path, struct stat *st, int *res)
{
    char hostpath[64];
    int minor = -1;

    if (!irix_pty_hostpath(guest_path, hostpath, sizeof(hostpath), &minor)) {
        return 0;
    }
    *res = stat(hostpath, st);
    if (*res == 0) {
        st->st_rdev = target_irix_makedev(TARGET_IRIX_PTS_MAJOR, minor);
    }
    return 1;
}

/*
 * fstat() hook: rewrite st_rdev for a pty master or slave fd so that the
 * major/minor the guest reads back are IRIX's, which is what getpty.c:205-236
 * switches on. Called on any successful fstat; a no-op for everything else.
 */
static void irix_pty_fstat_fixup(int fd, struct stat *st)
{
    int n;

    if (!S_ISCHR(st->st_mode)) {
        return;
    }
    n = irix_pty_master_index(fd);
    if (n >= 0) {
        st->st_rdev = target_irix_makedev(TARGET_IRIX_PTC_MAJOR, n);
        return;
    }
    /*
     * Unix98 pty slaves live on major 136.. (UNIX98_PTY_SLAVE_MAJOR) with the
     * devpts index in the minor, so the index is readable straight off st_rdev.
     */
    if (major(st->st_rdev) == 136) {
        st->st_rdev = target_irix_makedev(TARGET_IRIX_PTS_MAJOR,
                                          minor(st->st_rdev));
    }
}

/*
 * chown()/chmod() hook -- see the header comment for why these are no-ops.
 * Returns 1 if the path was a pty slave (the caller should report success),
 * 0 otherwise.
 */
static int irix_pty_is_slave_path(const char *guest_path)
{
    char hostpath[64];

    return irix_pty_hostpath(guest_path, hostpath, sizeof(hostpath), NULL);
}

/*
 * ioctl() hook for the STREAMS I_STR wrapper that IRIX's grantpt/unlockpt/
 * ptsname are built on. Registered as an IOCTL_SPECIAL handler so it goes
 * through the normal ioctl table.
 */
static abi_long do_ioctl_irix_i_str(const IOCTLEntry *ie, uint8_t *buf_temp,
                                    int fd, int cmd, abi_long arg)
{
    struct target_irix_strioctl *istr;
    int ic_cmd;

    istr = lock_user(VERIFY_READ, arg, sizeof(*istr), 1);
    if (!istr) {
        return -TARGET_EFAULT;
    }
    ic_cmd = tswap32(istr->ic_cmd);
    unlock_user(istr, arg, 0);

    switch (ic_cmd) {
    case TARGET_IRIX_UNLKPT:
        if (irix_pty_master_index(fd) < 0) {
            return -TARGET_EINVAL;
        }
        return get_errno(unlockpt(fd));
    case TARGET_IRIX_ISPTM:
        return irix_pty_master_index(fd) >= 0 ? 0 : -TARGET_EINVAL;
    case TARGET_IRIX_SVR4SOPEN:
        /* Only selects which of the two slave name forms ptsname() returns. */
        return irix_pty_master_index(fd) >= 0 ? 0 : -TARGET_EINVAL;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "irix: unsupported STREAMS I_STR ic_cmd=0x%04x on fd %d\n",
                      ic_cmd, fd);
        return -TARGET_EINVAL;
    }
}

/*
 * I_PUSH/I_POP/I_LOOK/I_FIND on a pty. IRIX terminal code pushes the "ptem",
 * "ldterm" and "ttcompat" STREAMS modules onto a freshly opened slave to get
 * line discipline behaviour that the host tty layer already provides
 * unconditionally, so the correct translation of the push is "nothing".
 * Scoped to pty fds: pretending a push succeeded on some other kind of stream
 * would be a lie with consequences.
 */
static abi_long do_ioctl_irix_i_push(const IOCTLEntry *ie, uint8_t *buf_temp,
                                     int fd, int cmd, abi_long arg)
{
    struct stat st;

    if (fstat(fd, &st) < 0) {
        return get_errno(-1);
    }
    if (!S_ISCHR(st.st_mode)) {
        return -TARGET_EINVAL;
    }
    if (irix_pty_master_index(fd) < 0 && major(st.st_rdev) != 136) {
        return -TARGET_EINVAL;
    }
    return 0;
}

#endif /* IRIX_TARGET_PTY_H */
