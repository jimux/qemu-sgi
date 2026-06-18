/*
 * memwatch - trap guest CPU writes (or reads) to a physical address range and
 * log the writing instruction's PC + the value.  This is the durable
 * replacement for gdb hardware watchpoints, which do not fire on the sgi-ip54
 * build (kernel data lives in MIPS KSEG0/KSEG1 unmapped segments that this
 * QEMU's TCG watchpoint check does not cover).  See
 * progress_notes/ip54/replay_debugging.md.
 *
 * Catches the "wild write" class: e.g. who is clobbering cause_ip5_count.
 * NOTE: it sees GUEST-CPU accesses only (translated instructions).  A write
 * issued by device C code (cpu_physical_memory_write, as the pvclock does) is
 * NOT a translated instruction and is invisible here.
 *
 * Usage:
 *   -plugin <build>/contrib/plugins/libmemwatch.so,addr=0x0829edc0,len=4,rw=w
 *     addr : guest PHYSICAL address to watch (required), hex or decimal
 *     len  : byte length of the window (default 4)
 *     rw   : w=stores (default), r=loads, a=both
 *
 * License: GNU GPL, version 2 or later.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t watch_lo;            /* inclusive */
static uint64_t watch_hi;            /* exclusive */
static enum qemu_plugin_mem_rw want_rw = QEMU_PLUGIN_MEM_W;
static uint64_t n_hits;

static void fmt_value(qemu_plugin_mem_value v, char *buf, size_t n)
{
    switch (v.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:
        snprintf(buf, n, "0x%02" PRIx8, v.data.u8); break;
    case QEMU_PLUGIN_MEM_VALUE_U16:
        snprintf(buf, n, "0x%04" PRIx16, v.data.u16); break;
    case QEMU_PLUGIN_MEM_VALUE_U32:
        snprintf(buf, n, "0x%08" PRIx32, v.data.u32); break;
    case QEMU_PLUGIN_MEM_VALUE_U64:
        snprintf(buf, n, "0x%016" PRIx64, v.data.u64); break;
    case QEMU_PLUGIN_MEM_VALUE_U128:
        snprintf(buf, n, "0x%016" PRIx64 "%016" PRIx64,
                 v.data.u128.high, v.data.u128.low); break;
    default:
        snprintf(buf, n, "?"); break;
    }
}

static void vcpu_mem(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    struct qemu_plugin_hwaddr *hw = qemu_plugin_get_hwaddr(info, vaddr);
    if (!hw || qemu_plugin_hwaddr_is_io(hw)) {
        return;                       /* RAM accesses only */
    }
    uint64_t pa = qemu_plugin_hwaddr_phys_addr(hw);
    if (pa < watch_lo || pa >= watch_hi) {
        return;
    }
    bool store = qemu_plugin_mem_is_store(info);
    uint64_t pc = (uint64_t)(uintptr_t)udata;   /* the accessing instruction */
    unsigned int sz = 1u << qemu_plugin_mem_size_shift(info);
    char vbuf[48];
    fmt_value(qemu_plugin_mem_get_value(info), vbuf, sizeof(vbuf));

    g_autoptr(GString) s = g_string_new(NULL);
    g_string_printf(s,
        "[memwatch] %s pa=0x%08" PRIx64 " va=0x%016" PRIx64
        " by pc=0x%016" PRIx64 " cpu=%u size=%u val=%s\n",
        store ? "WRITE" : "READ ", pa, vaddr, pc, cpu_index, sz, vbuf);
    qemu_plugin_outs(s->str);
    __atomic_fetch_add(&n_hits, 1, __ATOMIC_RELAXED);
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        /* Pass the instruction's vaddr through udata so the callback can report
         * which PC issued the access (the callback only gets the data vaddr). */
        void *pc = (void *)(uintptr_t)qemu_plugin_insn_vaddr(insn);
        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         want_rw, pc);
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    g_autoptr(GString) s = g_string_new(NULL);
    g_string_printf(s, "[memwatch] %" PRIu64 " hit(s) on [0x%08" PRIx64
                    ", 0x%08" PRIx64 ")\n", n_hits, watch_lo, watch_hi);
    qemu_plugin_outs(s->str);
}

QEMU_PLUGIN_EXPORT
int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info,
                        int argc, char **argv)
{
    uint64_t addr = 0, len = 4;
    bool have_addr = false;

    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) tok = g_strsplit(argv[i], "=", 2);
        if (g_strcmp0(tok[0], "addr") == 0 && tok[1]) {
            addr = g_ascii_strtoull(tok[1], NULL, 0);
            have_addr = true;
        } else if (g_strcmp0(tok[0], "len") == 0 && tok[1]) {
            len = g_ascii_strtoull(tok[1], NULL, 0);
        } else if (g_strcmp0(tok[0], "rw") == 0 && tok[1]) {
            if (g_strcmp0(tok[1], "w") == 0) {
                want_rw = QEMU_PLUGIN_MEM_W;
            } else if (g_strcmp0(tok[1], "r") == 0) {
                want_rw = QEMU_PLUGIN_MEM_R;
            } else if (g_strcmp0(tok[1], "a") == 0) {
                want_rw = QEMU_PLUGIN_MEM_RW;
            } else {
                fprintf(stderr, "memwatch: rw must be w|r|a\n");
                return -1;
            }
        } else {
            fprintf(stderr, "memwatch: bad arg '%s'\n", argv[i]);
            return -1;
        }
    }
    if (!have_addr) {
        fprintf(stderr, "memwatch: addr=<phys> is required\n");
        return -1;
    }
    if (len == 0) {
        len = 4;
    }
    watch_lo = addr;
    watch_hi = addr + len;
    fprintf(stderr, "memwatch: watching phys [0x%08" PRIx64 ", 0x%08" PRIx64
            ") for %s\n", watch_lo, watch_hi,
            want_rw == QEMU_PLUGIN_MEM_W ? "writes" :
            want_rw == QEMU_PLUGIN_MEM_R ? "reads" : "read+write");

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
