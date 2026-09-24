/*
 *  MIPS emulation helpers for qemu.
 *
 *  Copyright (c) 2004-2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "cpu.h"
#include "internal.h"
#include "exec/helper-proto.h"
#include "exec/memop.h"
#include "accel/tcg/cpu-ldst.h"
#include "fpu_helper.h"
#include "qemu/crc32c.h"
#include <zlib.h>

static inline target_ulong bitswap(target_ulong v)
{
    v = ((v >> 1) & (target_ulong)0x5555555555555555ULL) |
              ((v & (target_ulong)0x5555555555555555ULL) << 1);
    v = ((v >> 2) & (target_ulong)0x3333333333333333ULL) |
              ((v & (target_ulong)0x3333333333333333ULL) << 2);
    v = ((v >> 4) & (target_ulong)0x0F0F0F0F0F0F0F0FULL) |
              ((v & (target_ulong)0x0F0F0F0F0F0F0F0FULL) << 4);
    return v;
}

#ifdef TARGET_MIPS64
target_ulong helper_dbitswap(target_ulong rt)
{
    return bitswap(rt);
}
#endif

target_ulong helper_bitswap(target_ulong rt)
{
    return (int32_t)bitswap(rt);
}

target_ulong helper_rotx(target_ulong rs, uint32_t shift, uint32_t shiftx,
                        uint32_t stripe)
{
    int i;
    uint64_t tmp0 = ((uint64_t)rs) << 32 | ((uint64_t)rs & 0xffffffff);
    uint64_t tmp1 = tmp0;
    for (i = 0; i <= 46; i++) {
        int s;
        if (i & 0x8) {
            s = shift;
        } else {
            s = shiftx;
        }

        if (stripe != 0 && !(i & 0x4)) {
            s = ~s;
        }
        if (s & 0x10) {
            if (tmp0 & (1LL << (i + 16))) {
                tmp1 |= 1LL << i;
            } else {
                tmp1 &= ~(1LL << i);
            }
        }
    }

    uint64_t tmp2 = tmp1;
    for (i = 0; i <= 38; i++) {
        int s;
        if (i & 0x4) {
            s = shift;
        } else {
            s = shiftx;
        }

        if (s & 0x8) {
            if (tmp1 & (1LL << (i + 8))) {
                tmp2 |= 1LL << i;
            } else {
                tmp2 &= ~(1LL << i);
            }
        }
    }

    uint64_t tmp3 = tmp2;
    for (i = 0; i <= 34; i++) {
        int s;
        if (i & 0x2) {
            s = shift;
        } else {
            s = shiftx;
        }
        if (s & 0x4) {
            if (tmp2 & (1LL << (i + 4))) {
                tmp3 |= 1LL << i;
            } else {
                tmp3 &= ~(1LL << i);
            }
        }
    }

    uint64_t tmp4 = tmp3;
    for (i = 0; i <= 32; i++) {
        int s;
        if (i & 0x1) {
            s = shift;
        } else {
            s = shiftx;
        }
        if (s & 0x2) {
            if (tmp3 & (1LL << (i + 2))) {
                tmp4 |= 1LL << i;
            } else {
                tmp4 &= ~(1LL << i);
            }
        }
    }

    uint64_t tmp5 = tmp4;
    for (i = 0; i <= 31; i++) {
        int s;
        s = shift;
        if (s & 0x1) {
            if (tmp4 & (1LL << (i + 1))) {
                tmp5 |= 1LL << i;
            } else {
                tmp5 &= ~(1LL << i);
            }
        }
    }

    return (int64_t)(int32_t)(uint32_t)tmp5;
}

/* these crc32 functions are based on target/loongarch/tcg/op_helper.c */
target_ulong helper_crc32(target_ulong val, target_ulong m, uint32_t sz)
{
    uint8_t buf[8];
    target_ulong mask = ((sz * 8) == 64) ?
                        (target_ulong) -1ULL :
                        ((1ULL << (sz * 8)) - 1);

    m &= mask;
    stq_le_p(buf, m);
    return (int32_t) (crc32(val ^ 0xffffffff, buf, sz) ^ 0xffffffff);
}

target_ulong helper_crc32c(target_ulong val, target_ulong m, uint32_t sz)
{
    uint8_t buf[8];
    target_ulong mask = ((sz * 8) == 64) ?
                        (target_ulong) -1ULL :
                        ((1ULL << (sz * 8)) - 1);
    m &= mask;
    stq_le_p(buf, m);
    return (int32_t) (crc32c(val, buf, sz) ^ 0xffffffff);
}

void helper_fork(target_ulong arg1, target_ulong arg2)
{
    /*
     * arg1 = rt, arg2 = rs
     * TODO: store to TC register
     */
}

target_ulong helper_yield(CPUMIPSState *env, target_ulong arg)
{
    target_long arg1 = arg;

    if (arg1 < 0) {
        /* No scheduling policy implemented. */
        if (arg1 != -2) {
            if (env->CP0_VPEControl & (1 << CP0VPECo_YSI) &&
                env->active_tc.CP0_TCStatus & (1 << CP0TCSt_DT)) {
                env->CP0_VPEControl &= ~(0x7 << CP0VPECo_EXCPT);
                env->CP0_VPEControl |= 4 << CP0VPECo_EXCPT;
                do_raise_exception(env, EXCP_THREAD, GETPC());
            }
        }
    } else if (arg1 == 0) {
        if (0) {
            /* TODO: TC underflow */
            env->CP0_VPEControl &= ~(0x7 << CP0VPECo_EXCPT);
            do_raise_exception(env, EXCP_THREAD, GETPC());
        } else {
            /* TODO: Deallocate TC */
        }
    } else if (arg1 > 0) {
        /* Yield qualifier inputs not implemented. */
        env->CP0_VPEControl &= ~(0x7 << CP0VPECo_EXCPT);
        env->CP0_VPEControl |= 2 << CP0VPECo_EXCPT;
        do_raise_exception(env, EXCP_THREAD, GETPC());
    }
    return env->CP0_YQMask;
}

static inline void check_hwrena(CPUMIPSState *env, int reg, uintptr_t pc)
{
    if ((env->hflags & MIPS_HFLAG_CP0) || (env->CP0_HWREna & (1 << reg))) {
        return;
    }
    do_raise_exception(env, EXCP_RI, pc);
}

target_ulong helper_rdhwr_cpunum(CPUMIPSState *env)
{
    check_hwrena(env, 0, GETPC());
    return env->CP0_EBase & 0x3ff;
}

target_ulong helper_rdhwr_synci_step(CPUMIPSState *env)
{
    check_hwrena(env, 1, GETPC());
    return env->SYNCI_Step;
}

target_ulong helper_rdhwr_cc(CPUMIPSState *env)
{
    check_hwrena(env, 2, GETPC());
#ifdef CONFIG_USER_ONLY
    return env->CP0_Count;
#else
    return (int32_t)cpu_mips_get_count(env);
#endif
}

target_ulong helper_rdhwr_ccres(CPUMIPSState *env)
{
    check_hwrena(env, 3, GETPC());
    return env->CCRes;
}

target_ulong helper_rdhwr_performance(CPUMIPSState *env)
{
    check_hwrena(env, 4, GETPC());
    return env->CP0_Performance0;
}

target_ulong helper_rdhwr_xnp(CPUMIPSState *env)
{
    check_hwrena(env, 5, GETPC());
    return (env->CP0_Config5 >> CP0C5_XNP) & 1;
}

void helper_pmon(CPUMIPSState *env, int function)
{
    function /= 2;
    switch (function) {
    case 2: /* TODO: char inbyte(int waitflag); */
        if (env->active_tc.gpr[4] == 0) {
            env->active_tc.gpr[2] = -1;
        }
        /* Fall through */
    case 11: /* TODO: char inbyte (void); */
        env->active_tc.gpr[2] = -1;
        break;
    case 3:
    case 12:
        printf("%c", (char)(env->active_tc.gpr[4] & 0xFF));
        break;
    case 17:
        break;
    case 158:
        {
            unsigned char *fmt = (void *)(uintptr_t)env->active_tc.gpr[4];
            printf("%s", fmt);
        }
        break;
    }
}

#ifdef TARGET_MIPS64
target_ulong helper_lcsr_cpucfg(CPUMIPSState *env, target_ulong rs)
{
    switch (rs) {
    case 0:
        return env->CP0_PRid;
    case 1:
        return env->lcsr_cpucfg1;
    case 2:
        return env->lcsr_cpucfg2;
    default:
        return 0;
    }
}
#endif

#if !defined(CONFIG_USER_ONLY)

void mips_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                  MMUAccessType access_type,
                                  int mmu_idx, uintptr_t retaddr)
{
    CPUMIPSState *env = cpu_env(cs);
    int error_code = 0;
    int excp;

    if (!(env->hflags & MIPS_HFLAG_DM)) {
        env->CP0_BadVAddr = addr;
    }

    if (access_type == MMU_DATA_STORE) {
        excp = EXCP_AdES;
    } else {
        excp = EXCP_AdEL;
        if (access_type == MMU_INST_FETCH) {
            error_code |= EXCP_INST_NOTAVAIL;
        }
    }

    do_raise_exception_err(env, excp, error_code, retaddr);
}

void mips_cpu_do_transaction_failed(CPUState *cs, hwaddr physaddr,
                                    vaddr addr, unsigned size,
                                    MMUAccessType access_type,
                                    int mmu_idx, MemTxAttrs attrs,
                                    MemTxResult response, uintptr_t retaddr)
{
    MIPSCPUClass *mcc = MIPS_CPU_GET_CLASS(cs);
    CPUMIPSState *env = cpu_env(cs);

    if (getenv("IP6ERR_DBG")) {
        fprintf(stderr, "IP6DBE: phys=%08llx va=%08llx size=%u type=%d "
                "pc=%08llx\n",
                (unsigned long long)physaddr, (unsigned long long)addr, size,
                access_type, (unsigned long long)env->active_tc.PC);
    }

    if (access_type == MMU_INST_FETCH) {
        do_raise_exception(env, EXCP_IBE, retaddr);
    } else if (!mcc->no_data_aborts) {
        do_raise_exception(env, EXCP_DBE, retaddr);
    }
}

/*
 * IP6 bring-up probe: dump full machine state when the guest reaches one of
 * the hooked PCs (0x80002838 mfc0 Cause, 0x8000ab58 idle_err entry,
 * 0x8008fa60 gr1_retr_intr positive control).  Env-gated by IP6HOOK.
 *
 * Output goes to IP6HOOK_LOGFILE when set (else stderr) so a heavy hook does
 * not back-pressure the vCPU's stderr pty.
 */
static void ip6hook_log(const char *fmt, ...) G_GNUC_PRINTF(1, 2);
static void ip6hook_log(const char *fmt, ...)
{
    static FILE *f;
    static bool tried;
    va_list ap;

    if (!tried) {
        const char *p = getenv("IP6HOOK_LOGFILE");

        tried = true;
        if (p && *p) {
            f = fopen(p, "a");
            if (f) {
                setvbuf(f, NULL, _IOLBF, 0);
            }
        }
    }
    if (f) {
        va_start(ap, fmt);
        vfprintf(f, fmt, ap);
        va_end(ap);
        return;
    }
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

void helper_ip6_pchook(CPUMIPSState *env)
{
    int i;

    ip6hook_log("IP6HOOK: pc=%08x cause=%08x sr=%08x epc=%08x badv=%08x "
            "ra=%08x\n", (uint32_t)env->active_tc.PC, env->CP0_Cause,
            env->CP0_Status, (uint32_t)env->CP0_EPC,
            (uint32_t)env->CP0_BadVAddr, (uint32_t)env->active_tc.gpr[31]);

    /* PID-1 syscall trace: entry (0x80016ea8) logs the args, the instruction
     * after the handler returns (0x800173a0) logs the return value/errno. */
    if ((uint32_t)env->active_tc.PC == 0x80016ea8u) {
        uint32_t fp = (uint32_t)env->active_tc.gpr[4];
        uint32_t num = (uint32_t)cpu_ldl_data(env, fp + 20);
        uint32_t ua0 = (uint32_t)cpu_ldl_data(env, fp + 28);
        uint32_t ua1 = (uint32_t)cpu_ldl_data(env, fp + 32);
        uint32_t ua2 = (uint32_t)cpu_ldl_data(env, fp + 36);
        uint32_t ua3 = (uint32_t)cpu_ldl_data(env, fp + 40);
        ip6hook_log("IP6SYS call n=%08x a0=%08x a1=%08x a2=%08x a3=%08x "
                    "ra=%08x\n", num, ua0, ua1, ua2, ua3,
                    (uint32_t)env->active_tc.gpr[31]);
        if (num == 0x3edu) {   /* open: log the path string */
            char sbuf[40];
            int k;
            for (k = 0; k < 39; k++) {
                int c = cpu_ldub_data(env, ua0 + k);
                if (c == 0) {
                    break;
                }
                sbuf[k] = c;
            }
            sbuf[k] = 0;
            ip6hook_log("IP6SYS open path=\"%s\" flags=%x mode=%x\n", sbuf,
                        ua1, ua2);
        }
        if (num == 0x3ecu) {   /* write: log fd, count, and the bytes */
            char wbuf[48];
            int k, n = (int)ua2;
            if (n > 47) {
                n = 47;
            }
            for (k = 0; k < n; k++) {
                int c = cpu_ldub_data(env, ua1 + k);
                wbuf[k] = (c >= 0x20 && c < 0x7f) ? c : '.';
            }
            wbuf[k] = 0;
            ip6hook_log("IP6SYS write fd=%08x buf=%08x n=%08x \"%s\"\n",
                        ua0, ua1, ua2, wbuf);
        }
    }
    if ((uint32_t)env->active_tc.PC == 0x80084260u) {
        ip6hook_log("IP6CON cnwrite a0=%08x a1=%08x a2=%08x ra=%08x\n",
                    (uint32_t)env->active_tc.gpr[4],
                    (uint32_t)env->active_tc.gpr[5],
                    (uint32_t)env->active_tc.gpr[6],
                    (uint32_t)env->active_tc.gpr[31]);
    }
    if ((uint32_t)env->active_tc.PC == 0x80084480u) {
        ip6hook_log("IP6CON cn_write flag=%08x a0=%08x a1=%08x a2=%08x\n",
                    (uint32_t)cpu_ldl_data(env, 0x80147000u),
                    (uint32_t)env->active_tc.gpr[4],
                    (uint32_t)env->active_tc.gpr[5],
                    (uint32_t)env->active_tc.gpr[6]);
    }
    if ((uint32_t)env->active_tc.PC == 0x800499e8u) {
        uint32_t s0 = (uint32_t)env->active_tc.gpr[16];
        uint32_t ops = (uint32_t)cpu_ldl_data(env, s0 + 64);
        ip6hook_log("IP6CON WRITEOP s0=%08x ops=%08x op=%08x type=%04x "
                    "foff=%08x\n", s0, ops,
                    (uint32_t)env->active_tc.gpr[12],
                    (uint32_t)cpu_ldl_data(env, s0 + 24) & 0xffff,
                    (uint32_t)cpu_ldl_data(env, s0 + 8));
    }
    if ((uint32_t)env->active_tc.PC == 0x800499b8u) {
        uint32_t s0 = (uint32_t)env->active_tc.gpr[16];
        uint32_t ops = (uint32_t)cpu_ldl_data(env, s0 + 64);
        ip6hook_log("IP6CON READOP s0=%08x ops=%08x op=%08x type=%04x\n",
                    s0, ops,
                    (uint32_t)env->active_tc.gpr[9],
                    (uint32_t)cpu_ldl_data(env, s0 + 24) & 0xffff);
    }
    if ((uint32_t)env->active_tc.PC == 0x800497b8u) {
        uint32_t s0 = (uint32_t)env->active_tc.gpr[16];
        ip6hook_log("IP6CON rdwr_type type=%04x s0=%08x ftype=%04x "
                    "w0=%08x w4=%08x w8=%08x\n",
                    (uint32_t)env->active_tc.gpr[6],
                    s0,
                    (uint32_t)cpu_ldl_data(env, s0 + 24) & 0xffff,
                    (uint32_t)cpu_ldl_data(env, s0),
                    (uint32_t)cpu_ldl_data(env, s0 + 4),
                    (uint32_t)cpu_ldl_data(env, s0 + 8));
    }
    if ((uint32_t)env->active_tc.PC == 0x80049750u) {
        uint32_t uv = (uint32_t)cpu_ldl_data(env, 0xffffc354u);
        ip6hook_log("IP6CON rdwr mode=%08x uv=%08x fd=%08x\n",
                    (uint32_t)env->active_tc.gpr[4], uv,
                    (uint32_t)cpu_ldl_data(env, uv));
    }
    if ((uint32_t)env->active_tc.PC == 0x8007c2c0u) {
        uint32_t s1 = (uint32_t)env->active_tc.gpr[4];
        uint32_t ty = (cpu_ldl_data(env, s1 + 24) >> 16) & 0xffff;
        uint32_t dv = (cpu_ldl_data(env, s1 + 48) >> 16) & 0xffff;
        ip6hook_log("IP6CON com_writei s1=%08x type=%04x dev=%04x major=%02x "
                    "ra=%08x\n", s1, ty, dv, (dv >> 8) & 0x7f,
                    (uint32_t)env->active_tc.gpr[31]);
    }
    if ((uint32_t)env->active_tc.PC == 0x80020940u) {
        ip6hook_log("IP6CON cdrv a0=%08x a1=%08x a2=%08x\n",
                    (uint32_t)env->active_tc.gpr[4],
                    (uint32_t)env->active_tc.gpr[5],
                    (uint32_t)env->active_tc.gpr[6]);
    }
    if ((uint32_t)env->active_tc.PC == 0x80084228u) {
        ip6hook_log("IP6CON cnclose\n");
    }
    if ((uint32_t)env->active_tc.PC == 0x800173a0u) {
        ip6hook_log("IP6SYS ret v0=%08x uinv0=%08x errcell=%08x\n",
                    (uint32_t)env->active_tc.gpr[2],
                    (uint32_t)cpu_ldl_data(env, 0xffffc114u),
                    (uint32_t)cpu_ldl_data(env, 0xffffc304u));
    }

    /* Semaphore lost-wakeup probe: dump the sema object at each psema/vsema. */
    if ((uint32_t)env->active_tc.PC == 0x800089e0u ||
        (uint32_t)env->active_tc.PC == 0x800089e8u) {
        uint32_t sa = (uint32_t)env->active_tc.gpr[4];

        ip6hook_log("IP6SEMA pc=%08x sem=%08x [%08x %08x %08x %08x] "
                    "w12=%08x w20=%08x t=%lld\n",
                    (uint32_t)env->active_tc.PC, sa,
                    (uint32_t)cpu_ldl_data(env, sa),
                    (uint32_t)cpu_ldl_data(env, sa + 4),
                    (uint32_t)cpu_ldl_data(env, sa + 8),
                    (uint32_t)cpu_ldl_data(env, sa + 12),
                    (uint32_t)cpu_ldl_data(env, sa - 0x2c + 12),
                    (uint32_t)cpu_ldl_data(env, sa - 0x2c + 0x20),
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }

    /* clock(): which timer does frame[140] claim? (bit 0x08 = the 100 Hz tick) */
    if ((uint32_t)env->active_tc.PC == 0x80013778u) {
        static int clkc;
        uint32_t a0 = (uint32_t)env->active_tc.gpr[4];

        if ((clkc++ % 500) == 0) {
            ip6hook_log("IP6CLK f140=%08x lo=%08x\n",
                        (uint32_t)cpu_ldl_data(env, a0 + 140),
                        (uint32_t)cpu_ldl_data(env, 0xffffb0a4u));
        }
    }

    /* LIVE SCSI ISR preconditions at 0x8008c7b0 (a0=chan). Address-guarded
     * so a NULL/low pointer cannot fault the helper. */
    if ((uint32_t)env->active_tc.PC == 0x8008c7b0u) {
        uint32_t t1 = (uint32_t)env->active_tc.gpr[4];
        uint32_t t2 = 0, v1 = 0;
        unsigned v1b0 = 0;

        if (t1 >= 0x80000000u && t1 < 0xa0000000u) {
            t2 = (uint32_t)cpu_ldl_data(env, t1 + 12);
            v1 = (uint32_t)cpu_ldl_data(env, t1 + 0x150);
            if (v1 >= 0x80000000u && v1 < 0xa0000000u) {
                v1b0 = (unsigned)cpu_ldub_data(env, v1);
            }
        }
        ip6hook_log("IP6ISR t1=%08x t2=%08x v1=%08x v1b0=%02x b40=%d "
                    "t2b21=%02x b0=%d t=%lld\n", t1, t2, v1, v1b0,
                    !!(v1b0 & 0x40),
                    (t2 >= 0x80000000u && t2 < 0xa0000000u) ?
                        (unsigned)cpu_ldub_data(env, t2 + 0x21) : 0,
                    (t2 >= 0x80000000u && t2 < 0xa0000000u) ?
                        !!(cpu_ldub_data(env, t2 + 0x21) & 1) : 0,
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }

    /* callout[] insertion trace (dotimeout). t0 ($8) = entry address. */
    if ((uint32_t)env->active_tc.PC == 0x800148f8u) {
        uint32_t e = (uint32_t)env->active_tc.gpr[8];
        uint32_t sp = (uint32_t)env->active_tc.gpr[29];

        ip6hook_log("IP6CALL ins entry=%08x cpu=%02x "
                    "h0=%08x h1=%08x h2=%08x h3=%08x t=%lld\n",
                    e, (unsigned)cpu_ldub_data(env, 0xffffb013u),
                    (uint32_t)cpu_ldl_data(env, 0x8011fc60),
                    (uint32_t)cpu_ldl_data(env, 0x8011fc80),
                    (uint32_t)cpu_ldl_data(env, 0x8011fca0),
                    (uint32_t)cpu_ldl_data(env, 0x8011fcc0),
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        (void)sp;
    }
    if ((uint32_t)env->active_tc.PC == 0x80014818u) {
        uint32_t sp = (uint32_t)env->active_tc.gpr[29];

        ip6hook_log("IP6CALL dotimeout targcpu=%08x a1=%08x a2=%08x a3=%08x "
                    "stk16=%08x stk20=%08x cpu=%02x t=%lld\n",
                    (uint32_t)env->active_tc.gpr[4],
                    (uint32_t)env->active_tc.gpr[5],
                    (uint32_t)env->active_tc.gpr[6],
                    (uint32_t)env->active_tc.gpr[7],
                    (uint32_t)cpu_ldl_data(env, sp + 16),
                    (uint32_t)cpu_ldl_data(env, sp + 20),
                    (unsigned)cpu_ldub_data(env, 0xffffb013u),
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if ((uint32_t)env->active_tc.PC == 0x80014c48u) {
        uint32_t sp = (uint32_t)env->active_tc.gpr[29];

        ip6hook_log("IP6CALL untimeout a0=%08x ra=%08x spra=%08x t=%lld\n",
                    (uint32_t)env->active_tc.gpr[4],
                    (uint32_t)env->active_tc.gpr[31],
                    sp ? (uint32_t)cpu_ldl_data(env, sp + 20) : 0,
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    /* scsiintr delegate: the ASR byte the code actually tests for LCI */
    if ((uint32_t)env->active_tc.PC == 0x8008c7fcu) {
        ip6hook_log("IP6ASR a2=%02x (in-code ASR at LCI test) t=%lld\n",
                    (unsigned)(env->active_tc.gpr[6] & 0xff),
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    /* scsiintr entry (0x8008a5c0): ASR BEFORE the driver's status read */
    if ((uint32_t)env->active_tc.PC == 0x8008a5c0u) {
        uint32_t c = (uint32_t)env->active_tc.gpr[4];
        uint32_t p = 0;
        unsigned asr = 0;

        if (c >= 0x80000000u && c < 0xc0000000u) {
            p = (uint32_t)cpu_ldl_data(env, c + 0x150);
            if (p >= 0xa0000000u && p < 0xc0000000u) {
                asr = (unsigned)cpu_ldub_data(env, p);
            }
        }
        ip6hook_log("IP6INT chan=%08x asr_ptr=%08x asr=%02x cause=%08x "
                    "sr=%08x t=%lld\n", c, p, asr, env->CP0_Cause,
                    env->CP0_Status,
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    ip6hook_log("IP6HOOK: entryhi=%08x entrylo0=%08x entrylo1=%08x "
            "context=%08x index=%08x random=%08x\n",
            (uint32_t)env->CP0_EntryHi, (uint32_t)env->CP0_EntryLo0,
            (uint32_t)env->CP0_EntryLo1, (uint32_t)env->CP0_Context,
            (uint32_t)env->CP0_Index, (uint32_t)env->CP0_Random);
    ip6hook_log("IP6HOOK: rootdev=%08x masterpid=%08x "
            "nprocs=%08x\n", (uint32_t)cpu_ldl_data(env, 0x800fc4c8),
            (uint32_t)cpu_ldl_data(env, 0x800fdf20),
            (uint32_t)cpu_ldl_data(env, 0x80045038));
    for (i = 0; i < 32; i += 4) {
        ip6hook_log("IP6HOOK: r%02d=%08x r%02d=%08x r%02d=%08x r%02d=%08x\n",
                i, (uint32_t)env->active_tc.gpr[i],
                i + 1, (uint32_t)env->active_tc.gpr[i + 1],
                i + 2, (uint32_t)env->active_tc.gpr[i + 2],
                i + 3, (uint32_t)env->active_tc.gpr[i + 3]);
    }
}
#endif /* !CONFIG_USER_ONLY */
