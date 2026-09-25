/*
 * ip27wrwatch: log guest stores to a watched virtual-address range, with the
 * storing PC and value.  Built for the IP27 two-node rc2 hunt to catch the
 * code that writes (or fails to write) rld's pObj_Head at VA 0x0fbdbbd8, whose
 * later read is 0 in crashing processes.
 *
 * Env: IP27_WW_LO / IP27_WW_HI select the watched range (hex, required).
 * Plugins can only see user/kernel virtual stores, so this names the userspace
 * writer (rld) exactly; a kernel copy through its own direct map appears on a
 * different VA and is not watched by design.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t ww_lo = 1;
static uint64_t ww_hi = 0;

static void vcpu_mem(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;
    qemu_plugin_mem_value mv;
    uint64_t val;

    if (vaddr < ww_lo || vaddr >= ww_hi) {
        return;
    }
    mv = qemu_plugin_mem_get_value(info);
    switch (mv.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:
        val = mv.data.u8;
        break;
    case QEMU_PLUGIN_MEM_VALUE_U16:
        val = mv.data.u16;
        break;
    case QEMU_PLUGIN_MEM_VALUE_U32:
        val = mv.data.u32;
        break;
    case QEMU_PLUGIN_MEM_VALUE_U128:
        val = mv.data.u128.low;
        break;
    default:
        val = mv.data.u64;
        break;
    }
    {
        struct qemu_plugin_hwaddr *h = qemu_plugin_get_hwaddr(info, vaddr);
        uint64_t pa = h ? qemu_plugin_hwaddr_phys_addr(h) : 0;

        fprintf(stderr, "IP27_WW %s pc=0x%016" PRIx64
                        " vaddr=0x%016" PRIx64 " pa=0x%016" PRIx64
                        " val=0x%016" PRIx64 " size=%u\n",
                qemu_plugin_mem_is_store(info) ? "W" : "R",
                pc, vaddr, pa, val,
                1u << qemu_plugin_mem_size_shift(info));
    }
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    size_t i;

    for (i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t pc = qemu_plugin_insn_vaddr(insn);

        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_W,
                                         (void *)(uintptr_t)pc);
        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_R,
                                         (void *)(uintptr_t)pc);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *lo = getenv("IP27_WW_LO");
    const char *hi = getenv("IP27_WW_HI");

    if (lo) {
        ww_lo = strtoull(lo, NULL, 16);
    }
    if (hi) {
        ww_hi = strtoull(hi, NULL, 16);
    }
    fprintf(stderr, "IP27_WW watch 0x%016" PRIx64 "..0x%016" PRIx64 "\n",
            ww_lo, ww_hi);

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    return 0;
}
