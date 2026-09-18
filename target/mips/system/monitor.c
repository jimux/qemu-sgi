/*
 * QEMU MIPS monitor commands
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later.  See the COPYING file in the top-level directory.
 *
 * @@SEMANTICS@@: Debug-only monitor surface for the SGI/O2 kernel-heap
 * bring-up.  QEMU's MIPS gdbstub translates virtual addresses through the
 * live TLB (cpu_memory_rw_debug -> mips_cpu_get_phys_page_debug), but until
 * now there was no way to *see* the TLB entries being used, unlike i386 /
 * PPC / SH4 / SPARC / Xtensa which all expose `info tlb`.  This file adds
 * the MIPS equivalent.  It only reads CPU/MMU state and never changes
 * emulated behaviour.
 */
#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qobject/qdict.h"
#include "cpu.h"
#include "internal.h"
#include "monitor/monitor.h"
#include "monitor/hmp-target.h"
#include "monitor/hmp.h"

void dump_mmu(CPUMIPSState *env)
{
    int i;

    if (!env->tlb) {
        qemu_printf("No TLB for this CPU\n");
        return;
    }

    if (!env->tlb->helper_tlbwi) {
        qemu_printf("No R4000-style TLB for this CPU\n");
        return;
    }

    qemu_printf("TLB entries: %u in use of %u\n",
                env->tlb->tlb_in_use, env->tlb->nb_tlb);
    qemu_printf(" idx  VPN2             ASID  MMID  Mask        "
                "PFN0       PFN1       V0 V1 D0 D1 G XI0 XI1 RI0 RI1\n");
    for (i = 0; i < env->tlb->nb_tlb; i++) {
        r4k_tlb_t *tlb = &env->tlb->mmu.r4k.tlb[i];

        if (tlb->EHINV) {
            continue;
        }
        qemu_printf(" %3d  %016" PRIx64 " %04x  %04x  0x%08x  "
                    HWADDR_FMT_plx " " HWADDR_FMT_plx "  "
                    " %d  %d  %d  %d  %d  %d  %d  %d  %d\n",
                    i,
                    (uint64_t)tlb->VPN, tlb->ASID, tlb->MMID, tlb->PageMask,
                    tlb->PFN[0], tlb->PFN[1],
                    tlb->V0, tlb->V1, tlb->D0, tlb->D1, tlb->G,
                    tlb->XI0, tlb->XI1, tlb->RI0, tlb->RI1);
    }
}

void hmp_info_tlb(Monitor *mon, const QDict *qdict)
{
    CPUMIPSState *env = mon_get_cpu_env(mon);

    if (!env) {
        monitor_printf(mon, "No CPU available\n");
        return;
    }
    dump_mmu(env);
}

/*
 * @@SEMANTICS@@ Debug-only VA->PA translator.  gdbstub `m` and HMP `x`
 * already translate VAs via the live TLB, but when a KSEG2 address fails
 * it is useful to see *why*.  This runs get_physical_address() with the
 * CPU's current mmu_idx and, for comparison, with kernel/user modes, and
 * prints the MMU state that governs the segment.  No emulated-behaviour
 * change.
 */
void hmp_mips_va(Monitor *mon, const QDict *qdict)
{
    CPUMIPSState *env = mon_get_cpu_env(mon);
    int64_t addr = qdict_get_int(qdict, "addr");
    static const struct {
        const char *name;
        int idx;
    } modes[] = { { "cur", -1 }, { "km", MIPS_HFLAG_KM },
                  { "sm", MIPS_HFLAG_SM }, { "um", MIPS_HFLAG_UM } };
    int i;

    if (!env) {
        monitor_printf(mon, "No CPU available\n");
        return;
    }

    monitor_printf(mon, "VA %016" PRIx64 "\n", (uint64_t)addr);
    monitor_printf(mon, "  Status=%08x EntryHi=%016" PRIx64
                   " PageMask=%08x SEGMask=%016" PRIx64 "\n",
                   env->CP0_Status, (uint64_t)env->CP0_EntryHi,
                   env->CP0_PageMask, (uint64_t)env->SEGMask);
    monitor_printf(mon, "  SegCtl0=%016" PRIx64 " SegCtl1=%016" PRIx64
                   " SegCtl2=%016" PRIx64 " mmu_idx=%d\n",
                   (uint64_t)env->CP0_SegCtl0, (uint64_t)env->CP0_SegCtl1,
                   (uint64_t)env->CP0_SegCtl2, mips_env_mmu_index(env));

    for (i = 0; i < ARRAY_SIZE(modes); i++) {
        hwaddr phys = 0;
        int prot = 0;
        int idx = modes[i].idx < 0 ? mips_env_mmu_index(env) : modes[i].idx;
        int ret = get_physical_address(env, &phys, &prot, (target_ulong)addr,
                                       MMU_DATA_LOAD, idx);

        if (ret == TLBRET_MATCH) {
            monitor_printf(mon, "  [%s] -> PA " HWADDR_FMT_plx
                           " prot=%d\n", modes[i].name, phys, prot);
        } else {
            monitor_printf(mon, "  [%s] -> TLBRET %d\n",
                           modes[i].name, ret);
        }
    }
}

