/*
 *  Emulation of IRIX signals
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 *  Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>;
 *  n64decomp/qemu-irix), GPLv2. This is a standalone IRIX MIPS signal frame
 *  implementation, derived from the qemu-irix linux-user/signal.c IRIX blocks
 *  and the QEMU 10.x linux-user/mips/signal.c, merged and adapted to the 10.x
 *  linux-user signal API.
 *
 *  Unlike Linux/MIPS, the IRIX kernel does not install a signal return
 *  trampoline in the frame: IRIX libc supplies its own trampoline whose
 *  address is handed to the kernel via sigaction (stored in ts->sigtramp).
 *  The kernel jumps to that trampoline with the real handler address in a3.
 *  The trampoline invokes the handler then issues sigreturn. Frame recovery
 *  on sigreturn therefore uses the sigcontext/ucontext pointer (a1), not $sp.
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
#include "signal-common.h"
#include "linux-user/trace.h"

/* IRIX N32/N64 sigcontext (64-bit register slots). */
struct target_sigcontext {
    uint64_t sc_regs[32];
    uint64_t sc_fpregs[32];
    uint64_t sc_mdhi;
    uint64_t sc_hi1;
    uint64_t sc_hi2;
    uint64_t sc_hi3;
    uint64_t sc_mdlo;
    uint64_t sc_lo1;
    uint64_t sc_lo2;
    uint64_t sc_lo3;
    uint64_t sc_pc;
    uint32_t sc_fpc_csr;
    uint32_t sc_used_math;
    uint32_t sc_dsp;
    uint32_t sc_reserved;
};

/* IRIX omits the o32 arg-save space and the in-frame trampoline. */
struct sigframe {
    struct target_sigcontext sf_sc;
    target_sigset_t sf_mask;
};

/* struct target_ucontext is defined in irix/target_signal.h (mcontext union) */

struct target_rt_sigframe {
    struct target_siginfo rs_info;
    struct target_ucontext rs_uc;
};

/*
 * IRIX mcontext save/restore (getcontext/setcontext + rt signal frames).
 * Ported verbatim from qemu-irix linux-user/signal.c.
 * gregs[0]=zero, gregs[1..31]=GPRs, gregs[32]=LO, gregs[33]=HI, gregs[35]=PC.
 */
int save_context(CPUMIPSState *regs, struct target_mcontext *sc, int setret)
{
    int err = 0;
    int i;

    /* O32 saves a 32 bit context, N32 and N64 save a 64 bit context */
#ifdef TARGET_ABI_MIPSO32
    struct target_mcontext32 *mc = &sc->mc._32;
#else
    struct target_mcontext64 *mc = &sc->mc._64;
#endif

    /* store cpu registers */
    __put_user(regs->active_tc.PC, &mc->gregs[35]);

    __put_user(0, &mc->gregs[0]);
    for (i = 1; i < 32; i++) {
        __put_user(regs->active_tc.gpr[i], &mc->gregs[i]);
    }

    __put_user(regs->active_tc.LO[0], &mc->gregs[32]);
    __put_user(regs->active_tc.HI[0], &mc->gregs[33]);

    /* store fpu registers */
    for (i = 0; i < (int)(sizeof(mc->fregs) / sizeof(mc->fregs[0])); i++) {
        __put_user(regs->active_fpu.fpr[i].d, (uint64_t *)&mc->fregs[i]);
    }

    __put_user(regs->active_fpu.fcr31, &mc->fpcsr);

    /* set return code from getcontext if required */
    if (setret) {
        __put_user(0, &mc->gregs[2]);
        __put_user(0, &mc->gregs[7]);
    }

    return err;
}

int restore_context(CPUMIPSState *regs, struct target_mcontext *sc)
{
    int err = 0;
    int i;

    /* O32 restores a 32 bit context, N32 and N64 restore a 64 bit context */
#ifdef TARGET_ABI_MIPSO32
    struct target_mcontext32 *mc = &sc->mc._32;
#else
    struct target_mcontext64 *mc = &sc->mc._64;
#endif

    /* restore cpu registers */
    __get_user(regs->active_tc.PC, &mc->gregs[35]);

    for (i = 1; i < 32; i++) {
        __get_user(regs->active_tc.gpr[i], &mc->gregs[i]);
    }

    __get_user(regs->active_tc.LO[0], &mc->gregs[32]);
    __get_user(regs->active_tc.HI[0], &mc->gregs[33]);

    /* restore fpu registers */
    for (i = 0; i < (int)(sizeof(mc->fregs) / sizeof(mc->fregs[0])); i++) {
        __get_user(regs->active_fpu.fpr[i].d, (uint64_t *)&mc->fregs[i]);
    }

    return err;
}

static inline void setup_sigcontext(CPUMIPSState *regs,
                                    struct target_sigcontext *sc)
{
    int i;

    __put_user(exception_resume_pc(regs), &sc->sc_pc);
    regs->hflags &= ~MIPS_HFLAG_BMASK;

    __put_user(0, &sc->sc_regs[0]);
    for (i = 1; i < 32; ++i) {
        __put_user(regs->active_tc.gpr[i], &sc->sc_regs[i]);
    }

    __put_user(regs->active_tc.HI[0], &sc->sc_mdhi);
    __put_user(regs->active_tc.LO[0], &sc->sc_mdlo);

    __put_user(1, &sc->sc_used_math);

    for (i = 0; i < 32; ++i) {
        __put_user(regs->active_fpu.fpr[i].d, &sc->sc_fpregs[i]);
    }
}

static inline void
restore_sigcontext(CPUMIPSState *regs, struct target_sigcontext *sc)
{
    int i;

    __get_user(regs->CP0_EPC, &sc->sc_pc);

    __get_user(regs->active_tc.HI[0], &sc->sc_mdhi);
    __get_user(regs->active_tc.LO[0], &sc->sc_mdlo);

    for (i = 1; i < 32; ++i) {
        __get_user(regs->active_tc.gpr[i], &sc->sc_regs[i]);
    }

    for (i = 0; i < 32; ++i) {
        __get_user(regs->active_fpu.fpr[i].d, &sc->sc_fpregs[i]);
    }
}

/*
 * Determine which stack to use..
 */
static inline abi_ulong
get_sigframe(struct target_sigaction *ka, CPUMIPSState *regs, size_t frame_size)
{
    unsigned long sp;

    sp = target_sigsp(get_sp_from_cpustate(regs) - 32, ka);

    return (sp - frame_size) & ~7;
}

static void mips_set_hflags_isa_mode_from_pc(CPUMIPSState *env)
{
    if (env->insn_flags & (ASE_MIPS16 | ASE_MICROMIPS)) {
        env->hflags &= ~MIPS_HFLAG_M16;
        env->hflags |= (env->active_tc.PC & 1) << MIPS_HFLAG_M16_SHIFT;
        env->active_tc.PC &= ~(target_ulong) 1;
    }
}

void setup_frame(int sig, struct target_sigaction *ka,
                 target_sigset_t *set, CPUMIPSState *regs)
{
    TaskState *ts = get_task_state(env_cpu(regs));
    struct sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, regs, sizeof(*frame));
    trace_user_setup_frame(regs, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    setup_sigcontext(regs, &frame->sf_sc);

    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->sf_mask.sig[i]);
    }

    /*
     * Arguments to signal handler:
     *
     *   a0 = signal number
     *   a1 = 0 (should be cause)
     *   a2 = pointer to struct sigcontext
     *   a3 = pointer to signal handler (IRIX)
     *
     * IRIX: $25 and PC point to the libc signal trampoline (ts->sigtramp),
     * not the handler. $29 points to the struct sigframe.
     */
    regs->active_tc.gpr[ 4] = sig;
    regs->active_tc.gpr[ 5] = 0;
    regs->active_tc.gpr[ 6] = frame_addr + offsetof(struct sigframe, sf_sc);
    regs->active_tc.gpr[29] = frame_addr;
    regs->active_tc.gpr[ 7] = ka->_sa_handler;
    regs->active_tc.gpr[31] = 0;
    regs->active_tc.PC = regs->active_tc.gpr[25] = ts->sigtramp;
    mips_set_hflags_isa_mode_from_pc(regs);
    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_sigreturn(CPUMIPSState *regs)
{
    struct sigframe *frame;
    abi_ulong frame_addr;
    sigset_t blocked;
    target_sigset_t target_set;
    int i;

    /* IRIX: frame recovered from the sigcontext pointer in a1 ($5). */
    frame_addr = regs->active_tc.gpr[ 5] - offsetof(struct sigframe, sf_sc);
    trace_user_do_sigreturn(regs, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &frame->sf_mask.sig[i]);
    }

    target_to_host_sigset_internal(&blocked, &target_set);
    set_sigmask(&blocked);

    restore_sigcontext(regs, &frame->sf_sc);

    if (regs->CP0_EPC) {
        regs->active_tc.PC = regs->CP0_EPC;
        mips_set_hflags_isa_mode_from_pc(regs);
        regs->CP0_EPC = 0;
    }
    return -QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}

void setup_rt_frame(int sig, struct target_sigaction *ka,
                    target_siginfo_t *info,
                    target_sigset_t *set, CPUMIPSState *env)
{
    TaskState *ts = get_task_state(env_cpu(env));
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    frame->rs_info = *info;

    if (save_context(env, &frame->rs_uc.tuc_mcontext, 0)) {
        goto give_sigsegv;
    }
    __put_user(ts->ctx_link, (abi_ulong *)&frame->rs_uc.tuc_link);
    ts->ctx_link = frame_addr + offsetof(struct target_rt_sigframe, rs_uc);
    __put_user(0x0f, &frame->rs_uc.tuc_flags);
    target_save_altstack(&frame->rs_uc.tuc_stack, env);

    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->rs_uc.tuc_sigmask.sig[i]);
    }

    /*
     * Arguments to signal handler:
     *
     *   a0 = signal number (| 0x80000000 rt flag for the libc trampoline)
     *   a1 = pointer to siginfo_t
     *   a2 = pointer to ucontext_t
     *   a3 = pointer to signal handler (IRIX)
     */
    env->active_tc.gpr[ 4] = sig | 0x80000000L;
    env->active_tc.gpr[ 5] = frame_addr
                             + offsetof(struct target_rt_sigframe, rs_info);
    env->active_tc.gpr[ 6] = frame_addr
                             + offsetof(struct target_rt_sigframe, rs_uc);
    env->active_tc.gpr[29] = frame_addr;
    env->active_tc.gpr[ 7] = ka->_sa_handler;
    env->active_tc.gpr[31] = 0;
    env->active_tc.PC = env->active_tc.gpr[25] = ts->sigtramp;
    mips_set_hflags_isa_mode_from_pc(env);
    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    unlock_user_struct(frame, frame_addr, 1);
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUMIPSState *env)
{
    TaskState *ts = get_task_state(env_cpu(env));
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    sigset_t blocked;

    /* IRIX: frame recovered from the ucontext pointer in a1 ($5). */
    frame_addr = env->active_tc.gpr[ 5]
                 - offsetof(struct target_rt_sigframe, rs_uc);
    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    target_to_host_sigset(&blocked, &frame->rs_uc.tuc_sigmask);
    set_sigmask(&blocked);

    if (restore_context(env, &frame->rs_uc.tuc_mcontext)) {
        goto badframe;
    }
    __get_user(ts->ctx_link, (abi_ulong *)&frame->rs_uc.tuc_link);
    target_restore_altstack(&frame->rs_uc.tuc_stack, env);

    if (env->CP0_EPC) {
        env->active_tc.PC = env->CP0_EPC;
        mips_set_hflags_isa_mode_from_pc(env);
        env->CP0_EPC = 0;
    }
    return -QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -QEMU_ESIGRETURN;
}

/*
 * IRIX uses a libc-supplied signal trampoline (ts->sigtramp), so there is no
 * kernel sigtramp page. This stub satisfies the linker; it is only invoked
 * when TARGET_ARCH_HAS_SIGTRAMP_PAGE is set, which IRIX does not do.
 */
void setup_sigtramp(abi_ulong sigtramp_page)
{
}
