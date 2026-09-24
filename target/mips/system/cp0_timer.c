/*
 * QEMU MIPS timer support
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "qemu/timer.h"
#include "system/kvm.h"
#include "internal.h"

/*
 * On real R4000-class hardware CP0_Count advances at a fixed hardware rate
 * regardless of how busy the CPU is.  QEMU's QEMU_CLOCK_VIRTUAL is fine for
 * a CPU keeping up with its modeled frequency, but on sgi-ip54 (and any host
 * where icount sleep=off is needed for UI smoothness) virtual time races
 * ahead of wall time and dilates everything the kernel reads — networking
 * timeouts, lbolt-derived gettimeofday(), animations.
 *
 * Setting QEMU_MIPS_COUNT_REALTIME=1 makes COUNT/COMPARE-IRQ7 read
 * QEMU_CLOCK_REALTIME (host monotonic) instead, so wall time is the source
 * of truth for the guest's perception of time.  Default keeps VIRTUAL to
 * preserve behaviour for other MIPS machines under test.
 * See progress_notes/time_decoupling_investigation_2026-06-19.md.
 */
static QEMUClockType mips_count_clock_type(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("QEMU_MIPS_COUNT_REALTIME");
        cached = (e && *e && *e != '0') ? 1 : 0;
    }
    return cached ? QEMU_CLOCK_REALTIME : QEMU_CLOCK_VIRTUAL;
}

/* MIPS R4K timer */
static uint32_t cpu_mips_get_count_val(CPUMIPSState *env)
{
    int64_t now_ns;
    now_ns = qemu_clock_get_ns(mips_count_clock_type());
    return env->CP0_Count +
            (uint32_t)clock_ns_to_ticks(env->count_clock, now_ns);
}

/*
 * IP30 (SGI Octane) sets this true (see sgi_octane.c).  IRIX there writes a
 * CP0_Compare a few counts past-due on its lateness corrections, and without
 * firing immediately the tick is deferred by a full 32-bit Count wrap, which
 * freezes lbolt ("hasn't seen a scheduler clock interrupt").  Kept false for
 * every other machine until each is verified independently.  See #3404.
 */
bool mips_cp0_fire_immediate;

static int timer_dbg(void)
{
    static int c = -1;
    if (c < 0) { const char *e = getenv("TIMER_DEBUG"); c = (e && *e && *e != '0'); }
    return c;
}

static void cpu_mips_timer_update(CPUMIPSState *env)
{
    uint64_t now_ns, next_ns;
    uint32_t wait, cnt;

    now_ns = qemu_clock_get_ns(mips_count_clock_type());
    cnt = cpu_mips_get_count_val(env);
    wait = env->CP0_Compare - cnt;
    if (timer_dbg() && wait > 2000000u) {
        fprintf(stderr, "cp0timer: BIG wait=%u compare=0x%08x count=0x%08x delta=%d\n",
                wait, env->CP0_Compare, cnt, (int32_t)(env->CP0_Compare - cnt));
    }
    /*
     * If the deadline has already passed (Compare is at or behind the current
     * Count), the timer interrupt is pending *now*.  The unsigned subtraction
     * above would otherwise underflow to ~2^32 and defer the interrupt by a
     * full 32-bit wrap.  IRIX on IP30 writes a Compare that is a few counts
     * past-due on its lateness corrections, so there this must fire
     * immediately (wait == 1) rather than wait a whole wrap for the Count to
     * come back around.
     *
     * Gated: only the IP30 machine (sgi_octane) sets mips_cp0_fire_immediate.
     * Other machines keep the stock behaviour until each is verified -- see
     * #3404.
     */
    if (mips_cp0_fire_immediate && (wait == 0 || wait > 0x7fffffffu)) {
        wait = 1;
    }
    next_ns = now_ns + clock_ticks_to_ns(env->count_clock, wait);
    timer_mod(env->timer, next_ns);
}

/* Expire the timer.  */
static void cpu_mips_timer_expire(CPUMIPSState *env)
{
    cpu_mips_timer_update(env);
    if (env->insn_flags & ISA_MIPS_R2) {
        env->CP0_Cause |= 1 << CP0Ca_TI;
    }
    qemu_irq_raise(env->irq[(env->CP0_IntCtl >> CP0IntCtl_IPTI) & 0x7]);
}

uint32_t cpu_mips_get_count(CPUMIPSState *env)
{
    if (env->CP0_Cause & (1 << CP0Ca_DC)) {
        return env->CP0_Count;
    } else {
        uint64_t now_ns;

        now_ns = qemu_clock_get_ns(mips_count_clock_type());
        if (timer_pending(env->timer)
            && timer_expired(env->timer, now_ns)) {
            /* The timer has already expired.  */
            cpu_mips_timer_expire(env);
        }

        return cpu_mips_get_count_val(env);
    }
}

void cpu_mips_store_count(CPUMIPSState *env, uint32_t count)
{
    /*
     * This gets called from cpu_state_reset(), potentially before timer init.
     * So env->timer may be NULL, which is also the case with KVM enabled so
     * treat timer as disabled in that case.
     */
    if (env->CP0_Cause & (1 << CP0Ca_DC) || !env->timer) {
        env->CP0_Count = count;
    } else {
        /* Store new count register */
        env->CP0_Count = count - (uint32_t)clock_ns_to_ticks(env->count_clock,
                        qemu_clock_get_ns(mips_count_clock_type()));
        /* Update timer timer */
        cpu_mips_timer_update(env);
    }
}

void cpu_mips_store_compare(CPUMIPSState *env, uint32_t value)
{
    if (timer_dbg()) {
        uint32_t c = cpu_mips_get_count_val(env);
        static int64_t c0, last; static long n; int64_t now = qemu_clock_get_ns(mips_count_clock_type());
        if (!n) c0 = now;
        if (n % 500 == 0)
            fprintf(stderr, "cp0timer: store_compare=0x%08x count=0x%08x delta=%d t=%.3fs n=%ld\n",
                    value, c, (int32_t)(value - c), (now - c0)/1e9, n);
        last = now; n++;
    }
    env->CP0_Compare = value;
    if (!(env->CP0_Cause & (1 << CP0Ca_DC))) {
        cpu_mips_timer_update(env);
    }
    if (env->insn_flags & ISA_MIPS_R2) {
        env->CP0_Cause &= ~(1 << CP0Ca_TI);
    }
    qemu_irq_lower(env->irq[(env->CP0_IntCtl >> CP0IntCtl_IPTI) & 0x7]);
}

void cpu_mips_start_count(CPUMIPSState *env)
{
    cpu_mips_store_count(env, env->CP0_Count);
}

void cpu_mips_stop_count(CPUMIPSState *env)
{
    /* Store the current value */
    env->CP0_Count += (uint32_t)clock_ns_to_ticks(env->count_clock,
                        qemu_clock_get_ns(mips_count_clock_type()));
}

static void mips_timer_cb(void *opaque)
{
    CPUMIPSState *env = opaque;

    if (env->CP0_Cause & (1 << CP0Ca_DC)) {
        return;
    }

    cpu_mips_timer_expire(env);
}

/*
 * vCPU-thread self-service for the host-clock (realtime) CP0 timer.
 *
 * mips_timer_cb runs in the main loop, which can be BQL-starved for seconds
 * during MMIO/TLB-shootdown-heavy guest activity (e.g. a window drag on an
 * -smp virtuix desktop): the HZ tick interrupt then arrives seconds late and
 * the UI freezes even though every vCPU is busy. When the timer is driven off
 * QEMU_CLOCK_REALTIME we can safely deliver an already-expired tick from the
 * calling vCPU context (called from mips_cpu_has_work, which is re-evaluated
 * whenever a CPU is kicked -- and the drag's cross-CPU shootdowns kick them
 * constantly), so the tick no longer depends solely on main-loop latency.
 *
 * Idempotent and gated to the realtime clock, so the VIRTUAL-clock default
 * (authentic indy and every other MIPS machine) is completely unaffected.
 */
void cpu_mips_timer_catchup(CPUMIPSState *env)
{
    if (mips_count_clock_type() != QEMU_CLOCK_REALTIME) {
        return;
    }
    if (!env->timer || (env->CP0_Cause & (1 << CP0Ca_DC))) {
        return;
    }
    if (timer_pending(env->timer) &&
        timer_expired(env->timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME))) {
        cpu_mips_timer_expire(env);
    }
}

void cpu_mips_clock_init(MIPSCPU *cpu)
{
    CPUMIPSState *env = &cpu->env;

    /*
     * If we're in KVM mode, don't create the periodic timer, that is handled in
     * kernel.
     */
    if (!kvm_enabled()) {
        env->timer = timer_new_ns(mips_count_clock_type(),
                                  &mips_timer_cb, env);
    }
}
