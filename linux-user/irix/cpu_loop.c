/*
 *  qemu user cpu loop (IRIX N32)
 *
 *  Copyright (c) 2003-2008 Fabrice Bellard
 *
 *  Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>;
 *  n64decomp/qemu-irix), GPLv2. This is the IRIX MIPS cpu_loop, derived
 *  from the qemu-irix linux-user/main.c cpu_loop, adapted to the QEMU 10.x
 *  linux-user API.
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
 */

#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "user/cpu_loop.h"
#include "signal-common.h"
#include "elf.h"
#include "internal.h"
#include "fpu_helper.h"

/* PRDA emulation gate, consumed by target/mips/translate.c. */
int irix_emulate_prda;

/*
 * IRIX syscall argument descriptor table. See IRIX:/usr/include/sys.s.
 * Encodes, per syscall: number of (32-bit) argument registers, the position
 * of a 64-bit argument that must be split across an even/odd register pair
 * for the N32 ABI, whether the syscall returns a 64-bit value, and an
 * optional mapping to a 64-bit variant of the syscall.
 */
# define SYSCALL_ARGS(n, a64, r64, s) ((n) | (a64) << 4 | ((r64) << 8) | ((s) << 16))
# define SYSCALL_NARGS(v)           ((v) & 0xf)        /* #registers, incl. padding */
# define SYSCALL_ARG64(v)           (((v) >> 4) & 0xf) /* position of 64bit arg */
# define SYSCALL_RET64(v)           (((v) >> 8) & 0x1) /* returns a 64bit value */
# define SYSCALL_MAP(v)             ((v) >> 16)        /* N32 32bit syscall to 64bit */
# define _      0   /* for a better overview */
# define X      8   /* place holder for "don't know" for proprietary syscalls */
static const uint32_t mips_syscall_args[] = { /* see IRIX:/usr/include/sys.s */
	SYSCALL_ARGS(8, _, _, _),                   /*   0: syscall */
	SYSCALL_ARGS(1, _, _, _),                   /*   1: exit */
	SYSCALL_ARGS(0, _, _, _),                   /*   2: fork */
	SYSCALL_ARGS(3, _, _, _),                   /*   3: read */
	SYSCALL_ARGS(3, _, _, _),                   /*   4: write */
	SYSCALL_ARGS(3, _, _, _),                   /*   5: open */
	SYSCALL_ARGS(1, _, _, _),                   /*   6: close */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(2, _, _, _),                   /*   8: creat */
	SYSCALL_ARGS(2, _, _, _),                   /*   9: link */
	SYSCALL_ARGS(1, _, _, _),                   /*  10: unlink */
	SYSCALL_ARGS(2, _, _, _),                   /*  11: execv */
	SYSCALL_ARGS(1, _, _, _),                   /*  12: chdir */
	SYSCALL_ARGS(0, _, _, _),                   /*  13: time */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(2, _, _, _),                   /*  15: chmod */
	SYSCALL_ARGS(3, _, _, _),                   /*  16: chown */
	SYSCALL_ARGS(1, _, _, _),                   /*  17: brk */
	SYSCALL_ARGS(2, _, _, _),                   /*  18: stat */
	SYSCALL_ARGS(3, _, _, TARGET_NR_lseek64),   /*  19: lseek */
	SYSCALL_ARGS(0, _, _, _),                   /*  20: getpid */
	SYSCALL_ARGS(6, _, _, _),                   /*  21: mount */
	SYSCALL_ARGS(1, _, _, _),                   /*  22: umount */
	SYSCALL_ARGS(1, _, _, _),                   /*  23: setuid */
	SYSCALL_ARGS(0, _, _, _),                   /*  24: getuid */
	SYSCALL_ARGS(1, _, _, _),                   /*  25: stime */
	SYSCALL_ARGS(4, _, _, _),                   /*  26: ptrace */
	SYSCALL_ARGS(1, _, _, _),                   /*  27: alarm */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(0, _, _, _),                   /*  29: pause */
	SYSCALL_ARGS(2, _, _, _),                   /*  30: utime */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(2, _, _, _),                   /*  33: access */
	SYSCALL_ARGS(1, _, _, _),                   /*  34: nice */
	SYSCALL_ARGS(4, _, _, _),                   /*  35: statfs */
	SYSCALL_ARGS(0, _, _, _),                   /*  36: sync */
	SYSCALL_ARGS(2, _, _, _),                   /*  37: kill */
	SYSCALL_ARGS(4, _, _, _),                   /*  38: fstatfs */
	SYSCALL_ARGS(1, _, _, _),                   /*  39: pgrpsys */
	SYSCALL_ARGS(X, _, _, _),                   /*  40: syssgi */
	SYSCALL_ARGS(1, _, _, _),                   /*  41: dup */
	SYSCALL_ARGS(0, _, _, _),                   /*  42: pipe */
	SYSCALL_ARGS(1, _, _, _),                   /*  43: times */
	SYSCALL_ARGS(4, _, _, _),                   /*  44: profil */
	SYSCALL_ARGS(1, _, _, _),                   /*  45: plock */
	SYSCALL_ARGS(1, _, _, _),                   /*  46: setgid */
	SYSCALL_ARGS(0, _, _, _),                   /*  47: getgid */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(6, _, _, _),                   /*  49: msgsys */
	SYSCALL_ARGS(4, _, _, _),                   /*  50: sysmips */
	SYSCALL_ARGS(1, _, _, _),                   /*  51: acct */
	SYSCALL_ARGS(5, _, _, _),                   /*  52: shmsys */
	SYSCALL_ARGS(5, 5, _, _),                   /*  53: semsys */
	SYSCALL_ARGS(3, _, _, _),                   /*  54: ioctl */
	SYSCALL_ARGS(3, _, _, _),                   /*  55: uadmin */
	SYSCALL_ARGS(X, _, _, _),                   /*  56: sysmp */
	SYSCALL_ARGS(3, _, _, _),                   /*  57: utssys */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(3, _, _, _),                   /*  59: execve */
	SYSCALL_ARGS(1, _, _, _),                   /*  60: umask */
	SYSCALL_ARGS(1, _, _, _),                   /*  61: chroot */
	SYSCALL_ARGS(3, _, _, _),                   /*  62: fcntl */
	SYSCALL_ARGS(2, _, _, _),                   /*  63: ulimit */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(2, _, _, _),                   /*  75: getrlimit64 */
	SYSCALL_ARGS(2, _, _, _),                   /*  76: setrlimit64 */
	SYSCALL_ARGS(2, _, _, _),                   /*  77: nanosleep */
	SYSCALL_ARGS(5, 2, 1, _),                   /*  78: lseek64 */
	SYSCALL_ARGS(1, _, _, _),                   /*  79: rmdir */
	SYSCALL_ARGS(2, _, _, _),                   /*  80: mkdir */
	SYSCALL_ARGS(3, _, _, TARGET_NR_getdents64),/*  81: getdents */
	SYSCALL_ARGS(1, _, _, _),                   /*  82: sginap */
	SYSCALL_ARGS(3, _, _, _),                   /*  83: sgikopt */
	SYSCALL_ARGS(3, _, _, _),                   /*  84: sysfs */
	SYSCALL_ARGS(4, _, _, _),                   /*  85: getmsg */
	SYSCALL_ARGS(4, _, _, _),                   /*  86: putmsg */
	SYSCALL_ARGS(3, _, _, _),                   /*  87: poll */
	SYSCALL_ARGS(3, _, _, _),                   /*  88: sigreturn */
	SYSCALL_ARGS(3, _, _, _),                   /*  89: accept */
	SYSCALL_ARGS(3, _, _, _),                   /*  90: bind */
	SYSCALL_ARGS(3, _, _, _),                   /*  91: connect */
	SYSCALL_ARGS(0, _, _, _),                   /*  92: gethostid */
	SYSCALL_ARGS(3, _, _, _),                   /*  93: getpeername */
	SYSCALL_ARGS(3, _, _, _),                   /*  94: getsockname */
	SYSCALL_ARGS(5, _, _, _),                   /*  95: getsockopt */
	SYSCALL_ARGS(2, _, _, _),                   /*  96: listen */
	SYSCALL_ARGS(4, _, _, _),                   /*  97: recv */
	SYSCALL_ARGS(6, _, _, _),                   /*  98: recvfrom */
	SYSCALL_ARGS(3, _, _, _),                   /*  99: recvmsg */
	SYSCALL_ARGS(5, _, _, _),                   /* 100: select */
	SYSCALL_ARGS(4, _, _, _),                   /* 101: send */
	SYSCALL_ARGS(3, _, _, _),                   /* 102: sendmsg */
	SYSCALL_ARGS(6, _, _, _),                   /* 103: sendto */
	SYSCALL_ARGS(1, _, _, _),                   /* 104: sethostid */
	SYSCALL_ARGS(5, _, _, _),                   /* 105: setsockopt */
	SYSCALL_ARGS(2, _, _, _),                   /* 106: shutdown */
	SYSCALL_ARGS(3, _, _, _),                   /* 107: socket */
	SYSCALL_ARGS(2, _, _, _),                   /* 108: gethostname */
	SYSCALL_ARGS(2, _, _, _),                   /* 109: sethostname */
	SYSCALL_ARGS(2, _, _, _),                   /* 110: getdomainname */
	SYSCALL_ARGS(2, _, _, _),                   /* 111: setdomainname */
	SYSCALL_ARGS(2, _, _, TARGET_NR_truncate64),/* 112: truncate */
	SYSCALL_ARGS(2, _, _, TARGET_NR_ftruncate64),/* 113: ftruncate */
	SYSCALL_ARGS(2, _, _, _),                   /* 114: rename */
	SYSCALL_ARGS(2, _, _, _),                   /* 115: symlink */
	SYSCALL_ARGS(3, _, _, _),                   /* 116: readlink */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(X, _, _, _),                   /* 119: nfssvc */
	SYSCALL_ARGS(X, _, _, _),                   /* 120: getfh */
	SYSCALL_ARGS(X, _, _, _),                   /* 121: async_daemon */
	SYSCALL_ARGS(X, _, _, _),                   /* 122: exportfs */
	SYSCALL_ARGS(2, _, _, _),                   /* 123: setregid */
	SYSCALL_ARGS(2, _, _, _),                   /* 123: setreuid */
	SYSCALL_ARGS(2, _, _, _),                   /* 125: getitimer */
	SYSCALL_ARGS(3, _, _, _),                   /* 126: setitimer */
	SYSCALL_ARGS(2, _, _, _),                   /* 127: adjtime */
	SYSCALL_ARGS(1, _, _, _),                   /* 128: gettimeofday */
	SYSCALL_ARGS(3, _, _, _),                   /* 129: sproc */
	SYSCALL_ARGS(3, _, _, _),                   /* 130: prctl */
	SYSCALL_ARGS(3, _, _, _),                   /* 131: procblk */
	SYSCALL_ARGS(5, _, _, _),                   /* 132: sprocsp */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(6, _, _, TARGET_NR_mmap64),    /* 134: mmap */
	SYSCALL_ARGS(2, _, _, _),                   /* 135: munmap */
	SYSCALL_ARGS(3, _, _, _),                   /* 136: mprotect */
	SYSCALL_ARGS(3, _, _, _),                   /* 137: msync */
	SYSCALL_ARGS(3, _, _, _),                   /* 138: madvise */
	SYSCALL_ARGS(3, _, _, _),                   /* 139: pagelock */
	SYSCALL_ARGS(0, _, _, _),                   /* 140: getpagesize */
	SYSCALL_ARGS(4, _, _, _),                   /* 141: quotactl */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(1, _, _, _),                   /* 143: getpgid */
	SYSCALL_ARGS(2, _, _, _),                   /* 144: setpgid */
	SYSCALL_ARGS(0, _, _, _),                   /* 145: vhangup */
	SYSCALL_ARGS(1, _, _, _),                   /* 146: fsync */
	SYSCALL_ARGS(1, _, _, _),                   /* 147: fchdir */
	SYSCALL_ARGS(2, _, _, TARGET_NR_getrlimit64),/* 148: getrlimit */
	SYSCALL_ARGS(2, _, _, TARGET_NR_setrlimit64),/* 149: setrlimit */
	SYSCALL_ARGS(3, _, _, _),                   /* 150: cacheflush */
	SYSCALL_ARGS(3, _, _, _),                   /* 151: cachectl */
	SYSCALL_ARGS(3, _, _, _),                   /* 152: fchown */
	SYSCALL_ARGS(2, _, _, _),                   /* 153: fchmod */
	SYSCALL_ARGS(_, _, _, _),
	SYSCALL_ARGS(4, _, _, _),                   /* 155: socketpair */
	SYSCALL_ARGS(3, _, _, _),                   /* 156: sysinfo */
	SYSCALL_ARGS(1, _, _, _),                   /* 157: uname */
	SYSCALL_ARGS(3, _, _, _),                   /* 158: xstat */
	SYSCALL_ARGS(3, _, _, _),                   /* 159: lxstat */
	SYSCALL_ARGS(3, _, _, _),                   /* 160: fxstat */
	SYSCALL_ARGS(4, _, _, _),                   /* 161: xmknod */
	SYSCALL_ARGS(4, _, _, _),                   /* 162: sigaction */
	SYSCALL_ARGS(1, _, _, _),                   /* 163: sigpending */
	SYSCALL_ARGS(3, _, _, _),                   /* 164: sigprocmask */
	SYSCALL_ARGS(1, _, _, _),                   /* 165: sigsuspend */
	SYSCALL_ARGS(3, _, _, _),                   /* 166: sigpoll */
	SYSCALL_ARGS(2, _, _, _),                   /* 167: swapctl */
	SYSCALL_ARGS(1, _, _, _),                   /* 168: getcontext */
	SYSCALL_ARGS(1, _, _, _),                   /* 169: setcontext */
	SYSCALL_ARGS(5, _, _, _),                   /* 170: waitsys */
	SYSCALL_ARGS(2, _, _, _),                   /* 171: sigstack */
	SYSCALL_ARGS(2, _, _, _),                   /* 172: sigaltstack */
	SYSCALL_ARGS(2, _, _, _),                   /* 173: sigsendset */
	SYSCALL_ARGS(2, _, _, TARGET_NR_statvfs64), /* 174: statvfs */
	SYSCALL_ARGS(2, _, _, TARGET_NR_fstatvfs64),/* 175: fstatvfs */
	SYSCALL_ARGS(5, _, _, _),                   /* 176: getpmsg */
	SYSCALL_ARGS(5, _, _, _),                   /* 177: putpmsg */
	SYSCALL_ARGS(3, _, _, _),                   /* 178: lchown */
	SYSCALL_ARGS(0, _, _, _),                   /* 179: priocntl */
	SYSCALL_ARGS(X, _, _, _),                   /* 180: ksigqueue */
	SYSCALL_ARGS(3, _, _, _),                   /* 181: readv */
	SYSCALL_ARGS(3, _, _, _),                   /* 182: writev */
	SYSCALL_ARGS(4, 2, _, _),                   /* 183: truncate64 */
	SYSCALL_ARGS(4, 2, _, _),                   /* 184: ftruncate64 */
	SYSCALL_ARGS(8, 6, _, _),                   /* 185: mmap64 */
	SYSCALL_ARGS(X, _, _, _),                   /* 186: dmi */
	SYSCALL_ARGS(6, 4, _, _),                   /* 187: pread64 */
	SYSCALL_ARGS(6, 4, _, _),                   /* 188: pwrite64 */
	SYSCALL_ARGS(1, _, _, _),                   /* 189: fdatasync */
	SYSCALL_ARGS(X, _, _, _),                   /* 190: sgifastpath */
	SYSCALL_ARGS(5, _, _, _),                   /* 191: attr_get */
	SYSCALL_ARGS(5, _, _, _),                   /* 192: attr_getf */
	SYSCALL_ARGS(5, _, _, _),                   /* 193: attr_set */
	SYSCALL_ARGS(5, _, _, _),                   /* 194: attr_setf */
	SYSCALL_ARGS(3, _, _, _),                   /* 195: attr_remove */
	SYSCALL_ARGS(3, _, _, _),                   /* 196: attr_removef */
	SYSCALL_ARGS(5, _, _, _),                   /* 197: attr_list */
	SYSCALL_ARGS(5, _, _, _),                   /* 198: attr_listf */
	SYSCALL_ARGS(4, _, _, _),                   /* 199: attr_multi */
	SYSCALL_ARGS(4, _, _, _),                   /* 200: attr_multif */
	SYSCALL_ARGS(2, _, _, _),                   /* 201: statvfs64 */
	SYSCALL_ARGS(2, _, _, _),                   /* 202: fstatvfs64 */
	SYSCALL_ARGS(2, _, _, _),                   /* 203: getmountid */
	SYSCALL_ARGS(5, _, _, _),                   /* 204: nsproc */
	SYSCALL_ARGS(3, _, _, _),                   /* 205: getdents64 */
	SYSCALL_ARGS(X, _, _, _),                   /* 206: afs_syscall */
	SYSCALL_ARGS(4, _, _, TARGET_NR_ngetdents64),/* 207: ngetdents */
	SYSCALL_ARGS(4, _, _, _),                   /* 208: ngetdents64 */
	SYSCALL_ARGS(X, _, _, _),                   /* 209: sgi_sesmgr */
	SYSCALL_ARGS(X, _, _, _),                   /* 210: pidsprocsp */
	SYSCALL_ARGS(X, _, _, _),                   /* 211: rexec */
	SYSCALL_ARGS(3, _, _, _),                   /* 212: timer_create */
	SYSCALL_ARGS(1, _, _, _),                   /* 213: timer_delete */
	SYSCALL_ARGS(4, _, _, _),                   /* 214: timer_settime */
	SYSCALL_ARGS(2, _, _, _),                   /* 215: timer_gettime */
	SYSCALL_ARGS(1, _, _, _),                   /* 216: timer_getoverrun */
	SYSCALL_ARGS(2, _, _, _),                   /* 217: sched_rr_get_interval */
	SYSCALL_ARGS(0, _, _, _),                   /* 218: sched_yield */
	SYSCALL_ARGS(1, _, _, _),                   /* 219: sched_getscheduler */
	SYSCALL_ARGS(3, _, _, _),                   /* 220: sched_setscheduler */
	SYSCALL_ARGS(2, _, _, _),                   /* 221: sched_getparam */
	SYSCALL_ARGS(2, _, _, _),                   /* 222: sched_setparam */
	SYSCALL_ARGS(2, _, _, _),                   /* 223: usync_cntl */
	SYSCALL_ARGS(5, _, _, _),                   /* 224: psema_cntl */
	SYSCALL_ARGS(X, _, _, _),                   /* 225: restartreturn */
	SYSCALL_ARGS(5, _, _, _),                   /* 226: sysget */
	SYSCALL_ARGS(3, _, _, _),                   /* 227: xpg4_recvmsg */
	SYSCALL_ARGS(X, _, _, _),                   /* 228: umfscall */
	SYSCALL_ARGS(X, _, _, _),                   /* 229: nsproctid */
	SYSCALL_ARGS(X, _, _, _),                   /* 230: rexec_complete */
	SYSCALL_ARGS(2, _, _, _),                   /* 231: xpg4_sigaltstack */
	SYSCALL_ARGS(5, _, _, _),                   /* 232: xpg4_select */
	SYSCALL_ARGS(2, _, _, _),                   /* 233: xpg4_setregid */
	SYSCALL_ARGS(2, _, _, _),                   /* 234: linkfollow */
};

/*
 * split the arg64'th arg, which is a 64 bit arg in a 64 bit register, into an
 * even/odd 32 bit register pair, moving the other args up as necessary. This is
 * needed because the syscall ABI for TARGET_ABI32 only knows about 32 bit args.
 */
static void get_args_n32(target_ulong *regs, int arg64, int num, abi_ulong args[8])
{
    int i, j;

    if (arg64) {
        for (i = 0; i < arg64 - 1; i++) {
            args[i] = regs[i];
        }
        args[i] = 0;
        i += (i & 1); /* align to even register */
        args[i++] = regs[arg64 - 1] >> 32;
        args[i++] = regs[arg64 - 1];
        /* at most <num> registers are needed for the expanded args */
        for (j = arg64; i < num; j++) {
            args[i++] = regs[j];
        }
    } else {
        for (i = 0; i < num; i++) {
            args[i] = regs[i];
        }
    }
}

void cpu_loop(CPUMIPSState *env)
{
    CPUState *cs = env_cpu(env);
    TaskState *ts = get_task_state(cs);
    target_siginfo_t info;
    int trapnr;
    abi_long ret;
    unsigned int syscall_num;
    int offset = 0;

    __put_user(ts->ts_tid, (abi_int *)&ts->prda[0xe00]);
    __put_user(ts->ts_tid, (abi_int *)&ts->prda[0xe40]);

    for (;;) {
        cpu_exec_start(cs);
        trapnr = cpu_exec(cs);
        cpu_exec_end(cs);
        qemu_process_cpu_events(cs);

        switch (trapnr) {
        case EXCP_SYSCALL:
            env->active_tc.PC += 4;
            syscall_num = env->active_tc.gpr[2] - TARGET_NR_Linux;
            /* handle indirect syscalls here, else N32 64 bit args are passed
             * incorrectly */
            offset = (syscall_num ==
                      (unsigned)(TARGET_NR_syscall - TARGET_NR_Linux));
            if (offset) {
                syscall_num = env->active_tc.gpr[4] - TARGET_NR_Linux;
            }
            if (syscall_num >= ARRAY_SIZE(mips_syscall_args)) {
                ret = -TARGET_ENOSYS;
            } else {
                int nb_args;
                int arg64;
                abi_ulong args[8];

                /* map certain syscalls to their 64 bit version */
                if (SYSCALL_MAP(mips_syscall_args[syscall_num])) {
                    syscall_num = SYSCALL_MAP(mips_syscall_args[syscall_num])
                                  - TARGET_NR_Linux;
                }
                nb_args = SYSCALL_NARGS(mips_syscall_args[syscall_num]);
                arg64 = SYSCALL_ARG64(mips_syscall_args[syscall_num]);
                get_args_n32(&env->active_tc.gpr[4 + offset], arg64, nb_args,
                             args);

                ret = do_syscall(env, syscall_num + TARGET_NR_Linux,
                                 args[0], args[1], args[2], args[3],
                                 args[4], args[5], args[6], args[7]);
            }
            if (ret == -QEMU_ERESTARTSYS) {
                env->active_tc.PC -= 4;
                break;
            }
            if (ret == -QEMU_ESIGRETURN) {
                /* Returning from a successful sigreturn syscall.
                   Avoid clobbering register state.  */
                break;
            }
            /* on return: gpr7 = error flag, gpr2/3 = value(s) or error code */
            if (syscall_num < ARRAY_SIZE(mips_syscall_args) &&
                SYSCALL_RET64(mips_syscall_args[syscall_num])) {
                /* restore a 64 bit retval for N32 */
                target_ulong tret = ((target_ulong)ret << 32)
                                    | (uint32_t)env->active_tc.gpr[3];
                env->active_tc.gpr[7] = (tret >= (target_ulong)-1700);
                env->active_tc.gpr[2] =
                    (env->active_tc.gpr[7] ? -tret : tret);
            } else {
                env->active_tc.gpr[7] = ((abi_ulong)ret >= (abi_ulong)-1700);
                env->active_tc.gpr[2] =
                    (env->active_tc.gpr[7] ? -ret : ret);
            }
            break;
        case EXCP_TLBL:
        case EXCP_TLBS:
        case EXCP_AdEL:
        case EXCP_AdES:
            info.si_signo = TARGET_SIGSEGV;
            info.si_errno = 0;
            /* XXX: check env->error_code */
            info.si_code = TARGET_SEGV_MAPERR;
            info._sifields._sigfault._addr = env->CP0_BadVAddr;
            queue_signal(env, info.si_signo, QEMU_SI_FAULT, &info);
            break;
        case EXCP_CpU:
        case EXCP_RI:
        case EXCP_DSPDIS:
            info.si_signo = TARGET_SIGILL;
            info.si_errno = 0;
            info.si_code = 0;
            queue_signal(env, info.si_signo, QEMU_SI_FAULT, &info);
            break;
        case EXCP_FPE:
            info.si_signo = TARGET_SIGFPE;
            info.si_errno = 0;
            info.si_code = 0;
            queue_signal(env, info.si_signo, QEMU_SI_FAULT, &info);
            break;
        case EXCP_INTERRUPT:
            /* just indicate that signals should be handled asap */
            break;
        case EXCP_DEBUG:
            info.si_signo = TARGET_SIGTRAP;
            info.si_errno = 0;
            info.si_code = TARGET_TRAP_BRKPT;
            queue_signal(env, info.si_signo, QEMU_SI_FAULT, &info);
            break;
        case EXCP_BREAK:
        case EXCP_TRAP:
            info.si_signo = TARGET_SIGTRAP;
            info.si_errno = 0;
            info.si_code = TARGET_TRAP_BRKPT;
            queue_signal(env, info.si_signo, QEMU_SI_FAULT, &info);
            break;
        case EXCP_ATOMIC:
            cpu_exec_step_atomic(cs);
            break;
        default:
            EXCP_DUMP(env, "qemu: unhandled CPU exception 0x%x - aborting\n",
                      trapnr);
            abort();
        }
        process_pending_signals(env);
    }
}

void init_main_thread(CPUState *cs, struct image_info *info)
{
    CPUArchState *env = cpu_env(cs);
    target_ulong entry = info->entry;

    env->active_tc.gpr[29] = info->start_stack;
    env->active_tc.PC = entry & ~(target_ulong)1;
    if (entry & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }

    /* enable MIPS IV COP1X instructions for N32 */
    if ((info->elf_flags & EF_MIPS_ARCH) == EF_MIPS_ARCH_4) {
        env->CP0_Status |= (1 << CP0St_CU3);
        env->hflags |= MIPS_HFLAG_COP1X;
    }

    /* check if PRDA emulation is requested */
    if (getenv("QEMU_IRIXPRDA")) {
        irix_emulate_prda = 1;
    }
}
