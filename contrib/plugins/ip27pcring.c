/*
 * ip27pcring: record a ring of the last N translation-block PCs per vCPU and
 * dump it the moment execution enters a watched address range.
 *
 * Built for the IP27 two-node rc2 core-dump hunt: the guest faults with the PC
 * already inside rld's .rodata (a pointer/GOT-like table), i.e. downstream of a
 * corrupted control transfer.  Watching the data range prints the last PCs --
 * the call site that jumped there -- with no timing perturbation from a
 * QEMU -d trace.
 *
 * Env: IP27_PCRING_LO / IP27_PCRING_HI select the watched range (hex);
 *      default is rld's first LOAD data part 0x0fb60000..0x0fb672e0.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

#define RING_N 32

/* Per-vCPU-thread ring; callbacks run on the vCPU thread. */
static __thread uint64_t ring[RING_N];
static __thread uint64_t ridx;

/* Default: nothing watched; set IP27_PCRING_LO/HI to arm. */
static uint64_t watch_lo = 1;
static uint64_t watch_hi = 0;

static void vcpu_tb_exec(unsigned int cpu_index, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;
    int k;

    ring[ridx & (RING_N - 1)] = pc;
    ridx++;

    if (pc >= watch_lo && pc < watch_hi) {
        fprintf(stderr, "IP27_PCRING cpu%u enter watch pc=0x%016" PRIx64
                        " last PCs (newest first):\n", cpu_index, pc);
        for (k = 0; k < RING_N; k++) {
            unsigned idx = (unsigned)((ridx - 1 - k) & (RING_N - 1));
            uint64_t v = ring[idx];

            if (v == 0 && k > 0) {
                break;
            }
            fprintf(stderr, "  [%2d] 0x%016" PRIx64 "\n", k, v);
        }
    }
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    uint64_t pc = qemu_plugin_tb_vaddr(tb);

    qemu_plugin_register_vcpu_tb_exec_cb(tb, vcpu_tb_exec,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         (void *)(uintptr_t)pc);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *lo = getenv("IP27_PCRING_LO");
    const char *hi = getenv("IP27_PCRING_HI");

    if (lo) {
        watch_lo = strtoull(lo, NULL, 16);
    }
    if (hi) {
        watch_hi = strtoull(hi, NULL, 16);
    }
    fprintf(stderr, "IP27_PCRING watch 0x%016" PRIx64 "..0x%016" PRIx64 "\n",
            watch_lo, watch_hi);

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    return 0;
}
