/*
 *  IRIX hardware inventory synthesis for syssgi(SGI_INVENT, ...)
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
 * IRIX's kernel builds a linked list of "inventory" records as it probes
 * hardware at boot; user space reads that list back through syssgi() and
 * /sbin/hinv formats it. There is no hardware here to probe, so we synthesise
 * the table -- and we synthesise it out of *host* truth dressed in SGI
 * clothing (real CPU count, real clock, real RAM, real cache sizes), which is
 * the same virtualization-native doctrine the rest of this project follows.
 * Nothing here pretends to be a specific Indy's part numbers; it reports the
 * machine the binary is actually running on, in the only vocabulary IRIX has.
 *
 * Everything below is transcribed from the IRIX 6.5.5 sources in
 * software_library/irix-655-source/, not guessed:
 *
 *   f/irix/kern/sys/syssgi.h:34       SGI_INVENT 5
 *   f/irix/kern/sys/syssgi.h:525-526  SGI_INV_SIZEOF 1, SGI_INV_READ 2
 *   f/irix/kern/os/syssgi.c:898-935   the SGI_INVENT handler:
 *                                       rc = syssgi(SGI_INVENT, SGI_INV_SIZEOF)
 *                                       syssgi(SGI_INVENT, SGI_INV_READ, buf, len)
 *                                     READ walks the list copying out records
 *                                     while (len remaining >= isize), and
 *                                     returns (len - remaining), i.e. the byte
 *                                     count written. Each record is passed
 *                                     through invent_to_irix5() for a 32-bit
 *                                     ABI caller.
 *   f/irix/kern/os/syssgi.c:4452-4478 invent_to_irix5(): copies class, type,
 *                                     controller, unit and state across and
 *                                     forces inv_next to 0.
 *   f/irix/kern/sys/invent.h:620-628  irix5_inventory_t, the 32-bit-ABI record:
 *                                       app32_ptr_t inv_next
 *                                       int         inv_class
 *                                       int         inv_type
 *                                       major_t     inv_controller
 *                                       minor_t     inv_unit
 *                                       int         inv_state
 *                                     with app32_ptr_t = __uint32_t
 *                                     (f/irix/kern/sys/types.h:418) and
 *                                     major_t/minor_t 32-bit for a 32-bit ABI
 *                                     (types.h:86-93) -- so 24 bytes, six
 *                                     big-endian 32-bit words.
 *   f/irix/kern/os/invent.c:86-96     get_sizeof_inventory(): an n32 caller
 *                                     always gets sizeof(irix5_inventory_t).
 *   f/irix/kern/sys/invent.h:76,199-207,104-118  the class/type/state constants
 *   f/irix/kern/sys/sbd.h:770-824     rev_id: bits 15:8 implementation id,
 *                                     7:4 major rev, 3:0 minor rev; the
 *                                     C0_IMP_* table (R5000 = 0x23)
 *   f/irix/cmd/hinv/hinv.c:66,604-806 how hinv formats each record (ALL_8BIT,
 *                                     "%d MHZ IP22 Processors", the cache and
 *                                     main-memory lines)
 *
 * The record set is deliberately limited to the classes whose contents are a
 * real host fact. Disks, network interfaces, SCSI buses, serial ports and
 * graphics boards are *not* reported: there is no emulated device behind them
 * here, so a record would be an invention rather than a translation.
 */

#ifndef IRIX_TARGET_INVENT_H
#define IRIX_TARGET_INVENT_H

/* sys/syssgi.h */
#define TARGET_SGI_INV_SIZEOF   1
#define TARGET_SGI_INV_READ     2

/* sys/invent.h -- classes */
#define TARGET_INV_PROCESSOR    1
#define TARGET_INV_MEMORY       3

/* sys/invent.h -- types for class INV_PROCESSOR */
#define TARGET_INV_CPUBOARD     1
#define TARGET_INV_CPUCHIP      2
#define TARGET_INV_FPUCHIP      3

/* sys/invent.h -- cpuboard states */
#define TARGET_INV_IP22BOARD    12

/* sys/invent.h -- types for class INV_MEMORY */
#define TARGET_INV_DCACHE       3
#define TARGET_INV_ICACHE       4
#define TARGET_INV_SDCACHE      6
#define TARGET_INV_MAIN_MB      9

/* hinv.c:66 -- "all CPUs are identical" marker in inv_unit */
#define TARGET_INV_ALL_8BIT     0xff

/* sys/sbd.h -- C0_MAKE_REVID(imp, majrev, minrev) */
#define TARGET_C0_MAKE_REVID(i, j, n)   (((i) << 8) | ((j) << 4) | (n))
#define TARGET_C0_IMP_R5000             0x23

struct target_irix_inventory {
    abi_uint inv_next;
    abi_int  inv_class;
    abi_int  inv_type;
    abi_uint inv_controller;
    abi_uint inv_unit;
    abi_int  inv_state;
};

QEMU_BUILD_BUG_ON(sizeof(struct target_irix_inventory) != 24);

/* Number of host CPUs, reported to the guest as its processor count. */
static int irix_host_nprocs(void)
{
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? (int)n : 1;
}

/* Host RAM in MB. */
static int irix_host_memory_mb(void)
{
    long pages = sysconf(_SC_PHYS_PAGES);
    long pgsz = sysconf(_SC_PAGESIZE);

    if (pages <= 0 || pgsz <= 0) {
        return 0;
    }
    return (int)(((uint64_t)pages * (uint64_t)pgsz) >> 20);
}

/*
 * Host CPU clock in MHz. hinv prints the MHZ token only when inv_controller is
 * non-zero, so returning 0 on failure degrades to a clock-less "IP22
 * Processors" line rather than a lie.
 */
static int irix_host_cpu_mhz(void)
{
    static const char *paths[] = {
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq",
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
    };
    unsigned long khz;
    FILE *f;
    int i;

    for (i = 0; i < ARRAY_SIZE(paths); i++) {
        f = fopen(paths[i], "r");
        if (!f) {
            continue;
        }
        if (fscanf(f, "%lu", &khz) == 1 && khz) {
            fclose(f);
            return (int)(khz / 1000);
        }
        fclose(f);
    }

    /* No cpufreq (VM, or a kernel without it): fall back to /proc/cpuinfo. */
    f = fopen("/proc/cpuinfo", "r");
    if (f) {
        char line[256];
        while (fgets(line, sizeof(line), f)) {
            double mhz;
            if (sscanf(line, "cpu MHz : %lf", &mhz) == 1 ||
                sscanf(line, "clock : %lfMHz", &mhz) == 1) {
                fclose(f);
                return (int)mhz;
            }
        }
        fclose(f);
    }
    return 0;
}

/*
 * Build the inventory table. Returns the number of records; if recs is NULL
 * only the count is computed.
 */
static int irix_build_inventory(struct target_irix_inventory *recs, int max)
{
    int n = 0;
    int i, mb, mhz, ncpu;
    long sz;

#define INV_EMIT(cls, typ, ctlr, unit, state)                   \
    do {                                                        \
        if (recs) {                                             \
            if (n >= max) {                                     \
                return n;                                       \
            }                                                   \
            recs[n].inv_next = 0;                               \
            recs[n].inv_class = (cls);                          \
            recs[n].inv_type = (typ);                           \
            recs[n].inv_controller = (ctlr);                    \
            recs[n].inv_unit = (unit);                          \
            recs[n].inv_state = (state);                        \
        }                                                       \
        n++;                                                    \
    } while (0)

    ncpu = irix_host_nprocs();
    mhz = irix_host_cpu_mhz();

    /*
     * One CPU/FPU pair per host processor, in the order and pairing real hinv
     * expects (see the oracle capture in tmp/sysutils-review/oracle-outputs.txt,
     * which shows CPU:/FPU: alternating once per processor). The revision words
     * name an R5000 because that is the CPU this target actually emulates
     * (qemu-sgi's default MIPS model for SGI machines is R5000/mips4, and n32
     * mips4 binaries are what we run); the revision digits, 2.0 for the CPU and
     * 0.0 for the FPU, are the ones a real Indy reports.
     */
    for (i = 0; i < ncpu; i++) {
        INV_EMIT(TARGET_INV_PROCESSOR, TARGET_INV_CPUCHIP, i, 0,
                 TARGET_C0_MAKE_REVID(TARGET_C0_IMP_R5000, 2, 0));
        INV_EMIT(TARGET_INV_PROCESSOR, TARGET_INV_FPUCHIP, i, 0,
                 TARGET_C0_MAKE_REVID(TARGET_C0_IMP_R5000, 0, 0));
    }

    /*
     * The board record. inv_unit == ALL_8BIT tells hinv "every CPU is the
     * same", which makes it print the processor count (from sysmp(MP_NPROCS))
     * and the clock from inv_controller.
     */
    INV_EMIT(TARGET_INV_PROCESSOR, TARGET_INV_CPUBOARD, mhz,
             TARGET_INV_ALL_8BIT, TARGET_INV_IP22BOARD);

    mb = irix_host_memory_mb();
    if (mb > 0) {
        /* inv_unit 0 == not interleaved, so hinv omits the "N-way" suffix. */
        INV_EMIT(TARGET_INV_MEMORY, TARGET_INV_MAIN_MB, 0, 0, mb);
    }

    /*
     * Caches, in bytes -- hinv divides by 1024 itself. IRIX's vocabulary stops
     * at a secondary cache, so a host L3 has nowhere to go and is omitted.
     */
    sz = sysconf(_SC_LEVEL1_ICACHE_SIZE);
    if (sz > 0) {
        INV_EMIT(TARGET_INV_MEMORY, TARGET_INV_ICACHE, 0, 0, (int)sz);
    }
    sz = sysconf(_SC_LEVEL1_DCACHE_SIZE);
    if (sz > 0) {
        INV_EMIT(TARGET_INV_MEMORY, TARGET_INV_DCACHE, 0, 0, (int)sz);
    }
    sz = sysconf(_SC_LEVEL2_CACHE_SIZE);
    if (sz > 0) {
        INV_EMIT(TARGET_INV_MEMORY, TARGET_INV_SDCACHE, 0, 0, (int)sz);
    }

#undef INV_EMIT
    return n;
}

/* syssgi(SGI_INVENT, subcmd, buf, len) */
static abi_long do_syssgi_invent(abi_long subcmd, abi_ulong buf, abi_long len)
{
    const int isize = sizeof(struct target_irix_inventory);
    struct target_irix_inventory *recs;
    int nmax, n, i;
    void *p;

    switch (subcmd) {
    case TARGET_SGI_INV_SIZEOF:
        return isize;

    case TARGET_SGI_INV_READ:
        if (len < 0) {
            return -TARGET_EINVAL;
        }
        nmax = len / isize;
        if (nmax == 0) {
            return 0;
        }
        recs = g_new0(struct target_irix_inventory, nmax);
        n = irix_build_inventory(recs, nmax);

        p = lock_user(VERIFY_WRITE, buf, (abi_ulong)n * isize, 0);
        if (!p) {
            g_free(recs);
            return -TARGET_EFAULT;
        }
        for (i = 0; i < n; i++) {
            struct target_irix_inventory *t =
                (struct target_irix_inventory *)p + i;
            __put_user(recs[i].inv_next, &t->inv_next);
            __put_user(recs[i].inv_class, &t->inv_class);
            __put_user(recs[i].inv_type, &t->inv_type);
            __put_user(recs[i].inv_controller, &t->inv_controller);
            __put_user(recs[i].inv_unit, &t->inv_unit);
            __put_user(recs[i].inv_state, &t->inv_state);
        }
        unlock_user(p, buf, (abi_ulong)n * isize);
        g_free(recs);
        return n * isize;

    default:
        return -TARGET_EINVAL;
    }
}

#endif /* IRIX_TARGET_INVENT_H */
