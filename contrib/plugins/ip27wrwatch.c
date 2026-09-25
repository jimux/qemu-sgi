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

/*
 * Physical cells where the watched VA was successfully stored.  Any LATER
 * store to one of those cells, whatever its virtual address, is the in-place
 * clobber we are hunting (the observed failure zeroes a live user page via a
 * different, kernel mapping) -- its PC names the writer.
 */
#define WW_MAX_PHYS 1024
static uint64_t ww_phys[WW_MAX_PHYS];
static int ww_nphys;

/*
 * Normalise to the CELL two physical addresses share: node (tag parity at bit
 * 32), bank (bit 29) and the offset within the bank.  The machine's tag-alias
 * loop makes every tag k at the same bank/offset the same cell, so a clobber
 * arriving through a different tag must still compare equal.
 */
static uint64_t ww_cell(uint64_t pa)
{
    uint64_t node = (pa >> 32) & 1;
    uint64_t bank = (pa >> 29) & 1;
    uint64_t off = pa & 0x1fffffffULL;

    return (off << 2) | (bank << 1) | node;
}

static void ww_remember(uint64_t pa)
{
    uint64_t c = ww_cell(pa);
    int i;

    for (i = 0; i < ww_nphys; i++) {
        if (ww_phys[i] == c) {
            return;
        }
    }
    if (ww_nphys < WW_MAX_PHYS) {
        ww_phys[ww_nphys++] = c;
    }
}

static bool ww_known(uint64_t pa)
{
    uint64_t c = ww_cell(pa);
    int i;

    for (i = 0; i < ww_nphys; i++) {
        if (ww_phys[i] == c) {
            return true;
        }
    }
    return false;
}

static uint64_t ww_val(qemu_plugin_meminfo_t info)
{
    qemu_plugin_mem_value mv = qemu_plugin_mem_get_value(info);

    switch (mv.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:
        return mv.data.u8;
    case QEMU_PLUGIN_MEM_VALUE_U16:
        return mv.data.u16;
    case QEMU_PLUGIN_MEM_VALUE_U32:
        return mv.data.u32;
    case QEMU_PLUGIN_MEM_VALUE_U128:
        return mv.data.u128.low;
    default:
        return mv.data.u64;
    }
}

static void vcpu_mem(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;
    bool is_store = qemu_plugin_mem_is_store(info);
    struct qemu_plugin_hwaddr *h = qemu_plugin_get_hwaddr(info, vaddr);
    uint64_t pa = h ? qemu_plugin_hwaddr_phys_addr(h) : 0;
    uint32_t asid = qemu_plugin_current_asid();
    uint64_t val;

    if (vaddr >= ww_lo && vaddr < ww_hi) {
        val = ww_val(info);
        fprintf(stderr, "IP27_WW %s pc=0x%016" PRIx64
                        " asid=%04x vaddr=0x%016" PRIx64 " pa=0x%016" PRIx64
                        " val=0x%016" PRIx64 " size=%u\n",
                is_store ? "W" : "R", pc, asid, vaddr, pa, val,
                1u << qemu_plugin_mem_size_shift(info));
        if (is_store) {
            ww_remember(pa);
        }
        return;
    }
    /*
     * A store to a cell we have seen the watched VA stored to, via any other
     * virtual address: the clobber.
     */
    if (is_store && pa && ww_known(pa)) {
        val = ww_val(info);
        fprintf(stderr, "IP27_WWCLOBBER pc=0x%016" PRIx64
                        " asid=%04x vaddr=0x%016" PRIx64 " pa=0x%016" PRIx64
                        " val=0x%016" PRIx64 " size=%u\n",
                pc, asid, vaddr, pa, val,
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
