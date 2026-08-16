/*
 * SGI ARCS Firmware Stubs for Direct Kernel Boot
 *
 * Provides minimal ARCS firmware emulation so IRIX kernels loaded via
 * QEMU's -kernel flag can boot without a real PROM/sash chain.
 *
 * Architecture:
 *   1. Small MIPS stub routines are written into guest memory at init.
 *   2. Each stub writes its function ID to the ARCS hypercall MMIO device.
 *   3. The MMIO write handler implements the ARCS function in host C code.
 *   4. The stub reads back the result and returns to the caller.
 *
 * Implemented ARCS callbacks:
 *   - GetMemoryDescriptor: enumerates RAM layout
 *   - GetEnvironmentVariable: returns eaddr, cpufreq, console, etc.
 *   - Write: outputs to QEMU serial/log
 *   - GetRelativeTime: returns monotonic tick count
 *   - FlushAllCaches: no-op
 *   - Halt/PowerDown/Restart/Reboot: exits QEMU
 *   - All others: return error (EINVAL or NULL)
 *
 * References:
 *   - IRIX kern/sys/arcs/spb.h (SPB structure)
 *   - IRIX kern/sys/arcs/tvectors.h (FirmwareVector layout)
 *   - IRIX kern/sys/arcs/hinv.h (MEMORYDESCRIPTOR, memory types)
 *   - NetBSD sys/arch/sgimips/sgimips/arcemu.c (ARCS emulation)
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "hw/block/block.h"
#include "hw/core/cpu.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "hw/misc/sgi_arcs.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/runstate.h"
#include "qom/object.h"

/*
 * ARCS environment variables.
 * The kernel queries these via GetEnvironmentVariable().
 * Key variables needed for IP22/IP24 boot:
 *   - eaddr: Ethernet MAC address (SGI OUI 08:00:69)
 *   - cpufreq: CPU frequency in MHz
 *   - console: 'd' for serial debug console
 *   - ConsoleOut: ARCS console device path
 *   - dbaud: serial baud rate
 *   - OSLoadPartition: boot device path
 */
static const struct {
    const char *key;
    const char *value;
} arcs_env_vars[] = {
    { "eaddr",          "08:00:69:de:ad:01" },
    { "cpufreq",        "175" },
    { "console",        "d" },
    { "ConsoleOut",     "serial(0)" },
    { "ConsoleIn",      "serial(0)" },
    { "dbaud",          "9600" },
    { "rbaud",          "9600" },
    { "OSLoadPartition", "dksc(0,1,0)" },
    { "OSLoadFilename", "unix" },
    { "OSLoadOptions",  "" },
    { "SystemPartition", "dksc(0,1,8)" },
    { "AutoLoad",       "No" },
    { "nogfxkbd",       "1" },
    { "monitor",        "h" },
    { "gfx",            "alive" },
    { "netaddr",        "0.0.0.0" },
    { NULL,             NULL }
};

/*
 * ARCS memory descriptor table.
 * Built dynamically during setup based on actual RAM size and kernel placement.
 * The kernel iterates this via GetMemoryDescriptor(prev) returning next,
 * starting with GetMemoryDescriptor(NULL) which returns the first entry.
 *
 * Each descriptor is written to guest memory at ARCS_MEMDESC_PHYS as a
 * 12-byte big-endian structure: { Type(4), BasePage(4), PageCount(4) }
 */
#define MAX_MEMDESCS 8
#define MEMDESC_STRUCT_SIZE 12  /* 3 x uint32_t */
#define ARCS_PAGE_SIZE 4096

/*
 * Environment string storage in guest memory.
 * Strings are packed consecutively at ARCS_ENVDATA_PHYS.
 * The hypercall handler returns K0 pointers into this area.
 * 288 bytes holds all 16 arcs_env_vars entries (259 bytes actual) with margin;
 * shrunk from 512 to make room for the 44-byte stubs below.
 */
#define ARCS_ENVDATA_SIZE 288

/* SGI volume header (dvh) partition table — for Open()'s dksc(c,u,p) lookup. */
#define SGI_VH_MAGIC        0x0be5a941u
#define SGI_VH_PARTAB_OFF   0x138
#define SGI_VH_NPARTAB      16
#define SGI_VH_PT_ENTSZ     12   /* int nblks + int firstlbn + int type */
#define SGI_VH_PTYPE_XFS    10

/* ARCS errno (arcs/errno.h) */
#define ARCS_ESUCCESS       0
#define ARCS_EINVAL         7
#define ARCS_EIO            8
#define ARCS_ENODEV         13
#define ARCS_ENOENT         14

/* Sentinel the poll stub loops on: a blocking console Read() returns this and
 * the stub executes a MIPS `wait` until the chardev input arrives. -1. */
#define ARCS_RESULT_RETRY   0xFFFFFFFFu

#define ARCS_MAX_FDS        20   /* ARCS_FOPEN_MAX */

/* ------------------------------------------------------------------ */
/* Hypercall MMIO handlers                                            */
/* ------------------------------------------------------------------ */

/*
 * Read a NUL-terminated string from guest physical memory.
 * Returns a g_malloc'd buffer (caller must free).
 */
static char *read_guest_string(hwaddr phys_addr, int max_len)
{
    char *buf = g_malloc(max_len + 1);
    int i;

    for (i = 0; i < max_len; i++) {
        buf[i] = address_space_ldub(&address_space_memory, phys_addr + i,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
        if (buf[i] == '\0') {
            break;
        }
    }
    buf[i] = '\0';
    return buf;
}

/*
 * Sign-extend a guest pointer that arrived through the ARCS stub's `sw a0,4(t0)`.
 * The stub stores only the low 32 bits of the 64-bit GPR, so a kseg0/kseg1
 * pointer (e.g. the kernel's 0xffffffff882c652d) arrives here as 0x882c652d and
 * must be sign-extended back to its 64-bit compat form before cpu_memory_rw_debug
 * can translate it. Kuseg pointers (sash, < 0x80000000) are unchanged.
 */
static inline vaddr arcs_guest_va(uint32_t va32)
{
    return (vaddr)(int32_t)va32;
}

/*
 * Read a NUL-terminated string from a guest VIRTUAL address (handles kuseg ->
 * TLB -> physical, unlike the raw & 0x1FFFFFFF mask which only works for
 * kseg0/kseg1). Used for ARCS args that sash passes from its kuseg memory.
 */
static char *read_guest_string_va(uint32_t va, int max_len)
{
    char *buf = g_malloc(max_len + 1);
    CPUState *cs = first_cpu;
    int i;

    if (cs == NULL) {
        buf[0] = '\0';
        return buf;
    }
    for (i = 0; i < max_len; i++) {
        if (cpu_memory_rw_debug(cs, arcs_guest_va(va) + i, (uint8_t *)&buf[i],
                                1, 0) != 0) {
            buf[i] = '\0';
            break;
        }
        if (buf[i] == '\0') {
            break;
        }
    }
    buf[i] = '\0';
    return buf;
}

/*
 * Handle ARCS_FN_GETMEMORYDESC hypercall.
 *
 * arg0 = guest pointer to previous MEMORYDESCRIPTOR, or 0 for first.
 * Returns K0SEG pointer to next descriptor, or 0 (NULL) if no more.
 *
 * The descriptors are pre-written to guest memory at ARCS_MEMDESC_PHYS
 * during setup. We determine which descriptor arg0 points to by
 * computing its offset from the base.
 */
static uint32_t arcs_get_memory_desc(SGIARCSState *s, uint32_t arg0)
{
    int index;

    if (arg0 == 0) {
        /* First call: return pointer to descriptor 0 */
        index = 0;
    } else {
        /* Convert K0/K1 pointer back to physical address */
        uint32_t phys = arg0 & 0x1FFFFFFF;

        if (phys < ARCS_MEMDESC_PHYS ||
            phys >= ARCS_MEMDESC_PHYS + MAX_MEMDESCS * MEMDESC_STRUCT_SIZE) {
            return 0;  /* Invalid pointer */
        }
        index = (phys - ARCS_MEMDESC_PHYS) / MEMDESC_STRUCT_SIZE + 1;
    }

    /* Check if this descriptor is valid (Type != -1 sentinel) */
    uint32_t desc_phys = ARCS_MEMDESC_PHYS + index * MEMDESC_STRUCT_SIZE;
    uint32_t type = address_space_ldl_be(&address_space_memory, desc_phys,
                                         MEMTXATTRS_UNSPECIFIED, NULL);

    if (type == 0xFFFFFFFF) {
        return 0;  /* End of list */
    }

    /* Return K0SEG pointer to this descriptor */
    return MIPS_K0BASE + desc_phys;
}

/*
 * Handle ARCS_FN_GETENVVAR hypercall.
 *
 * arg0 = guest pointer to variable name string (K0/K1).
 * Returns K0SEG pointer to value string, or 0 (NULL) if not found.
 *
 * The value strings are pre-written to guest memory at ARCS_ENVDATA_PHYS.
 * We scan through guest memory to find matching key/value pairs that were
 * stored as "key\0value\0key\0value\0..." during setup.
 */
static uint32_t arcs_get_env_var(SGIARCSState *s, uint32_t arg0)
{
    char *name;
    uint32_t result = 0;
    hwaddr scan_addr;

    if (arg0 == 0) {
        return 0;
    }

    /* Read the variable name from guest memory (a guest VA — kuseg for sash,
     * kseg0 for the kernel; read_guest_string_va handles the TLB). */
    name = read_guest_string_va(arg0, 128);

    qemu_log("ARCS: GetEnvironmentVariable(\"%s\") -> 0x%08x\n", name, result);

    /*
     * Scan through the environment data area.
     * Layout: key\0value\0key\0value\0...\0\0 (double-NUL terminates)
     */
    scan_addr = ARCS_ENVDATA_PHYS;
    while (scan_addr < ARCS_ENVDATA_PHYS + ARCS_ENVDATA_SIZE) {
        char *key = read_guest_string(scan_addr, 64);
        size_t key_len = strlen(key);

        if (key_len == 0) {
            g_free(key);
            break;  /* End of data */
        }

        hwaddr val_addr = scan_addr + key_len + 1;
        char *val = read_guest_string(val_addr, 128);
        size_t val_len = strlen(val);

        if (strcasecmp(name, key) == 0) {
            result = MIPS_K0BASE + (uint32_t)val_addr;
            qemu_log_mask(LOG_UNIMP, "ARCS:   -> \"%s\"\n", val);
            g_free(key);
            g_free(val);
            break;
        }

        /* Skip to next key: key\0value\0 */
        scan_addr = val_addr + val_len + 1;
        g_free(key);
        g_free(val);
    }

    if (result == 0) {
        qemu_log_mask(LOG_UNIMP, "ARCS:   -> (not found)\n");
    }

    g_free(name);
    return result;
}

/* Host-file env persistence (defined below). */
static void arcs_env_persist(const char *name, const char *value);
static void arcs_env_buf_override(char *buf, int *len, int cap,
                                  const char *name, const char *value);
static int arcs_env_apply_persisted(char *env_buf, int *env_len, int env_size);

/*
 * SetEnvironmentVariable(name, value) — update the firmware environment in
 * guest memory (ARCS_ENVDATA_PHYS). Rebuilds the "key\0value\0..." blob so
 * GetEnvironmentVariable returns the new value, and persists to a host file.
 *
 * arg0 = guest VA of the variable name; arg1 = guest VA of the value.
 * Returns ESUCCESS (0) or ARCS_EINVAL.
 */
static uint32_t arcs_set_env_var_impl(SGIARCSState *s, const char *name,
                                      const char *value)
{
    uint8_t *blob;
    uint8_t *out;
    size_t out_len = 0;
    hwaddr scan_addr;
    bool replaced = false;
    int n;

    if (name[0] == '\0') {
        return ARCS_EINVAL;
    }

    /* Read the current env blob and rebuild it with this entry updated. */
    blob = g_malloc0(ARCS_ENVDATA_SIZE);
    for (n = 0; n < ARCS_ENVDATA_SIZE; n++) {
        blob[n] = address_space_ldub(&address_space_memory,
                                     ARCS_ENVDATA_PHYS + n,
                                     MEMTXATTRS_UNSPECIFIED, NULL);
    }
    out = g_malloc0(ARCS_ENVDATA_SIZE);

    /* Copy every existing entry except a same-named one. */
    scan_addr = 0;
    while (scan_addr < ARCS_ENVDATA_SIZE && blob[scan_addr] != '\0') {
        size_t key_len = strlen((char *)&blob[scan_addr]);
        size_t val_len;
        hwaddr val_off;
        if (key_len == 0) {
            break;
        }
        val_off = scan_addr + key_len + 1;
        val_len = strlen((char *)&blob[val_off]);

        if (strcasecmp(name, (char *)&blob[scan_addr]) != 0) {
            /* keep this entry */
            memcpy(out + out_len, blob + scan_addr, key_len + 1 + val_len + 1);
            out_len += key_len + 1 + val_len + 1;
        } else {
            replaced = true;
        }
        scan_addr = val_off + val_len + 1;
    }

    /* Append the (new or updated) entry. */
    if (out_len + strlen(name) + 1 + strlen(value) + 1 < ARCS_ENVDATA_SIZE - 1) {
        memcpy(out + out_len, name, strlen(name));
        out_len += strlen(name);
        out[out_len++] = '\0';
        memcpy(out + out_len, value, strlen(value));
        out_len += strlen(value);
        out[out_len++] = '\0';
    } else {
        g_free(blob);
        g_free(out);
        return ARCS_EIO;  /* env area exhausted */
    }

    /* Write the rebuilt blob back (the double-NUL terminator comes from the
     * zeroed buffer). */
    address_space_write(&address_space_memory, ARCS_ENVDATA_PHYS,
                        MEMTXATTRS_UNSPECIFIED, out, out_len + 1);

    qemu_log("ARCS: SetEnvironmentVariable(\"%s\") = \"%s\" (%s)\n",
             name, value, replaced ? "replaced" : "appended");
    arcs_env_persist(name, value);  /* survive reboot (host-file NVRAM) */
    g_free(blob);
    g_free(out);
    return ARCS_ESUCCESS;
}

static uint32_t arcs_set_env_var(SGIARCSState *s, uint32_t name_va,
                                 uint32_t value_va)
{
    char *name, *value;
    uint32_t rc;

    if (name_va == 0 || value_va == 0) {
        return ARCS_EINVAL;
    }
    name = read_guest_string_va(name_va, 64);
    value = read_guest_string_va(value_va, 256);
    rc = arcs_set_env_var_impl(s, name, value);
    g_free(name);
    g_free(value);
    return rc;
}

/*
 * Host-file persistence for SetEnvironmentVariable ("env set with NVRAM
 * persistence", C1). Appends "name=value\n" to a host file (relative, like the
 * virtuix NVRAM); on the NEXT boot sgi_arcs_setup_stubs() replays those lines
 * over the static env. Last occurrence wins.
 */
#define ARCS_ENV_PERSIST_FILE "sgi_arcs_env.conf"

static void arcs_env_persist(const char *name, const char *value)
{
    FILE *f = fopen(ARCS_ENV_PERSIST_FILE, "a");
    if (!f) {
        return;
    }
    fprintf(f, "%s=%s\n", name, value);
    fclose(f);
}

/*
 * Override/add a "key\0value\0" entry in a HOST "key\0value\0..." blob, in
 * place. Same rebuild semantics as arcs_set_env_var_impl but on host memory,
 * so persisted env can be replayed before the blob is written to guest RAM.
 */
static void arcs_env_buf_override(char *buf, int *len, int cap,
                                  const char *name, const char *value)
{
    char out[ARCS_ENVDATA_SIZE];
    int out_len = 0;
    int i = 0;

    while (i < *len && buf[i] != '\0') {
        int klen = strlen(buf + i);
        int vlen = strlen(buf + i + klen + 1);
        if (strcasecmp(name, buf + i) != 0) {
            memcpy(out + out_len, buf + i, klen + 1 + vlen + 1);
            out_len += klen + 1 + vlen + 1;
        }
        i += klen + 1 + vlen + 1;
    }
    if (out_len + (int)strlen(name) + 1 + (int)strlen(value) + 1 < cap) {
        memcpy(out + out_len, name, strlen(name));
        out_len += strlen(name);
        out[out_len++] = '\0';
        memcpy(out + out_len, value, strlen(value));
        out_len += strlen(value);
        out[out_len++] = '\0';
        memcpy(buf, out, out_len + 1);
        *len = out_len;
    }
}

/*
 * Replay any persisted env lines (from a prior boot) over the host env blob
 * before it is written to guest RAM. Returns the number of overrides applied.
 */
static int arcs_env_apply_persisted(char *env_buf, int *env_len, int env_size)
{
    FILE *f = fopen(ARCS_ENV_PERSIST_FILE, "r");
    char line[256];
    int applied = 0;

    if (!f) {
        return 0;
    }
    while (fgets(line, sizeof(line), f)) {
        char *eq = strchr(line, '=');
        char *nl;
        if (!eq) {
            continue;
        }
        nl = strchr(line, '\n');
        if (nl) {
            *nl = '\0';
        }
        *eq = '\0';
        arcs_env_buf_override(env_buf, env_len, env_size, line, eq + 1);
        applied++;
    }
    fclose(f);
    return applied;
}

/* Raw block-device file descriptor table (shared by Open/Read/Write/Seek). */
typedef struct ARCSFd {
    bool in_use;
    bool writable;
    BlockBackend *blk;
    uint64_t base;      /* partition start, in bytes (512*firstlbn) */
    uint64_t offset;    /* byte offset within the partition */
    uint64_t size;      /* partition size, in bytes */
} ARCSFd;

static ARCSFd arcs_fds[ARCS_MAX_FDS];

/*
 * Handle ARCS_FN_WRITE hypercall.
 *
 * arg0 = file descriptor (1 = stdout, 2 = stderr)
 * arg1 = guest pointer to buffer
 * arg2 = byte count
 *
 * Copies data from guest memory and writes it to the serial console chardev
 * (fds 1/2), so sash's banner/prompt appear on the same console the kernel
 * later uses. Returns 0 (ESUCCESS).
 */
static uint32_t arcs_write(SGIARCSState *s, uint32_t fd, uint32_t buf_ptr,
                           uint32_t count, uint32_t count_va)
{
    uint32_t nwritten = 0;
    uint32_t rc = ARCS_ESUCCESS;

    if (count == 0) {
        goto out_count;
    }

    if (fd >= 3 && fd < ARCS_MAX_FDS && arcs_fds[fd].in_use) {
        /* Raw block write to a SCSI partition slice (fx / mrboot mr_copy). */
        ARCSFd *f = &arcs_fds[fd];
        uint8_t *buf;
        uint32_t avail;

        if (!f->writable) {
            return ARCS_EINVAL;
        }
        if (count > 1024 * 1024) {
            goto out_count;
        }
        buf = g_malloc(count);
        if (cpu_memory_rw_debug(first_cpu, arcs_guest_va(buf_ptr), buf, count,
                                0) != 0) {
            g_free(buf);
            goto out_count;
        }
        if (f->offset >= f->size) {
            g_free(buf);
            goto out_count;  /* past end of partition: 0 bytes written */
        }
        avail = MIN((uint64_t)count, f->size - f->offset);
        if (blk_pwrite(f->blk, f->base + f->offset, avail, buf, 0) < 0) {
            g_free(buf);
            return ARCS_EIO;
        }
        f->offset += avail;
        nwritten = avail;
        g_free(buf);
        qemu_log("ARCS: Write(fd=%u, %u) -> %u bytes\n", fd, count, nwritten);
        goto out_count;
    }

    /* Console write (fd 1/2). */
    if (count <= 4096) {
        char *buf = g_malloc(count + 1);
        if (cpu_memory_rw_debug(first_cpu, arcs_guest_va(buf_ptr),
                                (uint8_t *)buf, count, 0) != 0) {
            g_free(buf);
            goto out_count;
        }
        buf[count] = '\0';
        if ((fd == 1 || fd == 2) && qemu_chr_fe_backend_connected(&s->chr)) {
            qemu_chr_fe_write_all(&s->chr, (uint8_t *)buf, count);
        } else {
            qemu_log("ARCS Write(fd=%u): %s", fd, buf);
        }
        g_free(buf);
        nwritten = count;
    }

out_count:
    if (count_va) {
        uint32_t count_be = cpu_to_be32(nwritten);
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(count_va),
                            (uint8_t *)&count_be, 4, 1);
    }
    return rc;
}

/*
 * Relative time counter for GetRelativeTime.
 * Returns a monotonically increasing value (1ms resolution simulated).
 */
static uint32_t arcs_relative_time;

/* Execute() implementation, registered by the machine (sgi_virtuix.c). */
static void (*arcs_execute_cb)(SGIARCSState *s, uint32_t path_va,
                               uint32_t argc, uint32_t argv_va,
                               uint32_t envp_va);

void sgi_arcs_set_execute_cb(void (*cb)(SGIARCSState *s, uint32_t path_va,
                                        uint32_t argc, uint32_t argv_va,
                                        uint32_t envp_va))
{
    arcs_execute_cb = cb;
}

/*
 * Append a "name=value" entry to the kernel environ. See the header decl.
 * Writes into guest RAM (below phys 0x2000) so it survives the kernel's own
 * LOAD segment; big-endian pointer array + NUL-terminated strings.
 */
void sgi_arcs_set_kernel_env(SGIARCSState *s, const char *name,
                             const char *value)
{
    int name_len = strlen(name);
    int value_len = strlen(value);
    int entry_len = name_len + 1 + value_len + 1;  /* "name=value\0" */
    uint8_t ptr[4];
    char *str;

    if (s->kernel_env_count + 1 >= 18) {
        qemu_log("ARCS: kernel environ full, cannot add %s\n", name);
        return;
    }
    if (s->kernel_env_str_off + entry_len > ARCS_ENVSTRS_SIZE) {
        qemu_log("ARCS: kernel environ strings full, cannot add %s\n", name);
        return;
    }

    /* Pointer (big-endian K0SEG VA of the string) at the next free slot. */
    stl_be_p(ptr, MIPS_K0BASE + ARCS_ENVSTRS_PHYS + s->kernel_env_str_off);
    address_space_write(&address_space_memory,
                        ARCS_ENVIRON_PHYS + s->kernel_env_count * 4,
                        MEMTXATTRS_UNSPECIFIED, ptr, 4);

    /* "name=value\0" at the next free string byte. */
    str = g_malloc(entry_len);
    snprintf(str, entry_len, "%s=%s", name, value);
    address_space_write(&address_space_memory,
                        ARCS_ENVSTRS_PHYS + s->kernel_env_str_off,
                        MEMTXATTRS_UNSPECIFIED, str, entry_len);
    g_free(str);

    s->kernel_env_count++;
    s->kernel_env_str_off += entry_len;

    /* NULL terminator after the new entry. */
    stl_be_p(ptr, 0);
    address_space_write(&address_space_memory,
                        ARCS_ENVIRON_PHYS + s->kernel_env_count * 4,
                        MEMTXATTRS_UNSPECIFIED, ptr, 4);
}

/* ------------------------------------------------------------------ */
/* Raw-device file services (Open/Read/Seek/Close/GetReadStatus)       */
/* ------------------------------------------------------------------ */
/*
 * These implement the firmware's raw block I/O: Open("dksc(c,u,p)") opens a
 * whole-disk or partition slice on a SCSI disk and returns a small fd; Read/
 * Seek/Close/GetReadStatus operate on that fd. sash's own filesystem layers
 * (sdvh/xfs/efs install) build on these to read the volume header + XFS/EFS.
 */

static uint32_t arcs_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static BlockBackend *arcs_scsi_backend(int bus, int unit)
{
    DriveInfo *dinfo = drive_get(IF_SCSI, bus, unit);
    return dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
}

/*
 * Parse an ARCS device path into (bus, unit, part). Accepts both the old
 * "dksc(bus,unit,part)" form and the canonical "scsi(c)disk(u)[rdisk(r)]
 * partition(p)" form (plus the short "scsi(c)disk(u)part(p)" fx emits).
 * Returns 0 on success, -1 if no disk specifier is found.
 */
static int arcs_devpath_parse(const char *name, int *bus, int *unit, int *part)
{
    int c = 0, u = 1, p = 0;
    const char *q;

    if (sscanf(name, "dksc(%d,%d,%d)", &c, &u, &p) == 3) {
        *bus = c;
        *unit = u;
        *part = p;
        return 0;
    }
    if ((q = strstr(name, "scsi(")) != NULL) {
        c = atoi(q + 5);
    }
    if ((q = strstr(name, "disk(")) == NULL) {
        return -1;
    }
    u = atoi(q + 5);
    if ((q = strstr(name, "partition(")) != NULL) {
        p = atoi(q + 10);
    } else if ((q = strstr(name, "part(")) != NULL) {
        p = atoi(q + 5);
    }
    *bus = c;
    *unit = u;
    *part = p;
    return 0;
}

static uint32_t arcs_open(SGIARCSState *s, uint32_t path_va, uint32_t mode,
                          uint32_t fd_va)
{
    char *name;
    int bus, unit, part;
    BlockBackend *blk;
    uint8_t vh[512];
    const uint8_t *pt;
    uint32_t nblks = 0, firstlbn = 0;
    int fd;
    bool writable;

    /* OpenReadOnly=0, OpenWriteOnly=1, OpenReadWrite=2, OpenDirectory=7. */
    if (mode == 0) {
        writable = false;
    } else if (mode == 1 || mode == 2) {
        writable = true;
    } else if (mode == 7) {
        writable = false;
    } else {
        return ARCS_EINVAL;
    }

    name = read_guest_string_va(path_va, 128);
    if (arcs_devpath_parse(name, &bus, &unit, &part) != 0) {
        g_free(name);
        return ARCS_ENODEV;
    }
    g_free(name);

    blk = arcs_scsi_backend(bus, unit);
    if (!blk) {
        return ARCS_ENODEV;
    }

    /* Allocate an fd slot first (we write the fd only on success). ARCS
     * reserves StandardIn/Out/Err = 0/1/2, so disk fds start at 3 — otherwise
     * Read(fd=0) would be ambiguous with the console read. */
    for (fd = 3; fd < ARCS_MAX_FDS && arcs_fds[fd].in_use; fd++) {
    }
    if (fd == ARCS_MAX_FDS) {
        return ARCS_EIO;
    }

    if (blk_pread(blk, 0, 512, vh, 0) < 0 || arcs_be32(&vh[0]) != SGI_VH_MAGIC) {
        /* Blank disk (no volume header yet, e.g. the C3 install target): allow
         * whole-disk access via the volume (10) / volhdr (8) / root (0) names
         * so fx can write the initial dvh + partition table. */
        int64_t len;
        if (part != 0 && part != 8 && part != 10) {
            return ARCS_EIO;
        }
        len = blk_getlength(blk);
        if (len <= 0) {
            return ARCS_EIO;
        }
        arcs_fds[fd].base = 0;
        arcs_fds[fd].size = (uint64_t)len;
        nblks = len / 512;
        firstlbn = 0;
    } else {
        if (part < 0 || part >= SGI_VH_NPARTAB) {
            return ARCS_EINVAL;
        }
        pt = &vh[SGI_VH_PARTAB_OFF + part * SGI_VH_PT_ENTSZ];
        nblks = arcs_be32(pt + 0);
        firstlbn = arcs_be32(pt + 4);
        arcs_fds[fd].base = (uint64_t)firstlbn * 512;
        arcs_fds[fd].size = (uint64_t)nblks * 512;
    }

    arcs_fds[fd].in_use = true;
    arcs_fds[fd].writable = writable;
    arcs_fds[fd].blk = blk;
    arcs_fds[fd].offset = 0;

    /* Write the fd back to the guest's output pointer. */
    uint32_t fd_be = cpu_to_be32(fd);
    cpu_memory_rw_debug(first_cpu, arcs_guest_va(fd_va), (uint8_t *)&fd_be,
                        4, 1);
    qemu_log("ARCS: Open(bus=%d unit=%d part=%d) -> fd %d (lbn %u, %u blks, "
             "%s)\n", bus, unit, part, fd, firstlbn, nblks,
             writable ? "rw" : "ro");
    return ARCS_ESUCCESS;
}

static uint32_t arcs_read(SGIARCSState *s, uint32_t fd, uint32_t buf_va,
                          uint32_t cnt, uint32_t count_va)
{
    ARCSFd *f;
    uint8_t *buf;
    uint32_t got = 0;
    uint32_t count_be;

    if (fd == 0) {
        /* StandardIn: drain the console input FIFO (filled by the chardev
         * receive callback). When idle, return ARCS_RESULT_RETRY so the poll
         * stub executes a MIPS `wait` (yields to the main loop) and re-triggers. */
        got = MIN((uint32_t)cnt, s->console_rx_len);
        if (got == 0) {
            return ARCS_RESULT_RETRY;
        }
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(buf_va),
                            s->console_rx, got, 1);
        memmove(s->console_rx, s->console_rx + got,
                s->console_rx_len - got);
        s->console_rx_len -= got;
        count_be = cpu_to_be32(got);
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(count_va),
                            (uint8_t *)&count_be, 4, 1);
        return ARCS_ESUCCESS;
    }

    if (fd >= ARCS_MAX_FDS || !arcs_fds[fd].in_use) {
        return ARCS_EINVAL;
    }
    f = &arcs_fds[fd];
    if (f->offset >= f->size) {
        /* EOF: report 0 bytes read */
        count_be = cpu_to_be32(0);
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(count_va),
                            (uint8_t *)&count_be, 4, 1);
        return ARCS_ESUCCESS;
    }
    uint32_t avail = MIN((uint64_t)cnt, f->size - f->offset);
    buf = g_malloc(avail);
    if (blk_pread(f->blk, f->base + f->offset, avail, buf, 0) < 0) {
        g_free(buf);
        return ARCS_EIO;
    }
    cpu_memory_rw_debug(first_cpu, arcs_guest_va(buf_va), buf, avail, 1);
    f->offset += avail;
    got = avail;
    g_free(buf);

    count_be = cpu_to_be32(got);
    cpu_memory_rw_debug(first_cpu, arcs_guest_va(count_va),
                        (uint8_t *)&count_be, 4, 1);
    qemu_log("ARCS: Read(fd=%u, %u) -> %u bytes\n", fd, cnt, got);
    return ARCS_ESUCCESS;
}

static uint32_t arcs_seek(SGIARCSState *s, uint32_t fd, uint32_t off_va,
                          uint32_t whence)
{
    ARCSFd *f;
    uint8_t li[8];
    int64_t off;

    if (fd >= ARCS_MAX_FDS || !arcs_fds[fd].in_use) {
        return ARCS_EINVAL;
    }
    f = &arcs_fds[fd];
    if (cpu_memory_rw_debug(first_cpu, arcs_guest_va(off_va), li, 8, 0) != 0) {
        return ARCS_EINVAL;
    }
    /* LARGEINTEGER is big-endian {hi (int32), lo (uint32)}. */
    off = ((int64_t)(int32_t)be32_to_cpu(*(uint32_t *)li) << 32) |
          be32_to_cpu(*(uint32_t *)(li + 4));

    if (whence == 0 /* SeekAbsolute */) {
        f->offset = (uint64_t)off;
    } else /* SeekRelative */ {
        f->offset += off;
    }
    qemu_log("ARCS: Seek(fd=%u, %s %" PRId64 ") -> offset %" PRIu64 "\n",
             fd, whence == 0 ? "abs" : "rel", off, f->offset);
    return ARCS_ESUCCESS;
}

static uint32_t arcs_close(SGIARCSState *s, uint32_t fd)
{
    if (fd >= ARCS_MAX_FDS || !arcs_fds[fd].in_use) {
        return ARCS_EINVAL;
    }
    arcs_fds[fd].in_use = false;
    qemu_log("ARCS: Close(fd=%u)\n", fd);
    return ARCS_ESUCCESS;
}

/*
 * SGI PrivateVector Ioctl: the raw-disk ioctl service fx / mrboot use for
 * volume-header get/set and drive capacity. arg0=fd, arg1=cmd (DIOC_*),
 * arg2=guest pointer to the data. Serves the DIOC commands the install chain
 * needs (capacity + dvh get/set + drive type); everything else fails EINVAL.
 */
#define SGI_DIOC_(x)            (('d' << 8) | (x))
#define SGI_DIOCGETVH           SGI_DIOC_(6)
#define SGI_DIOCSETVH           SGI_DIOC_(7)
#define SGI_DIOCSENSE           SGI_DIOC_(11)
#define SGI_DIOCREADCAPACITY    SGI_DIOC_(13)
#define SGI_DIOCDRIVETYPE       SGI_DIOC_(24)

static uint32_t arcs_ioctl(SGIARCSState *s, uint32_t fd, uint32_t cmd,
                           uint32_t arg_va)
{
    ARCSFd *f;
    uint8_t buf[512];

    if (fd >= ARCS_MAX_FDS || !arcs_fds[fd].in_use) {
        return ARCS_EINVAL;
    }
    f = &arcs_fds[fd];

    switch (cmd) {
    case SGI_DIOCREADCAPACITY: {
        int64_t len = blk_getlength(f->blk);
        uint32_t cap;
        if (len <= 0) {
            return ARCS_EIO;
        }
        cap = cpu_to_be32((uint32_t)(len / 512));
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(arg_va), (uint8_t *)&cap,
                            4, 1);
        qemu_log("ARCS: Ioctl(DIOCREADCAPACITY) -> %u blocks\n",
                 (uint32_t)(len / 512));
        return ARCS_ESUCCESS;
    }
    case SGI_DIOCGETVH:
        /* Read the on-disk volume header (sector 0) into the guest buffer.
         * A blank disk reads as zeros, which fx treats as "no volume header". */
        memset(buf, 0, sizeof(buf));
        blk_pread(f->blk, 0, sizeof(buf), buf, 0);
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(arg_va), buf, sizeof(buf),
                            1);
        qemu_log("ARCS: Ioctl(DIOCGETVH)\n");
        return ARCS_ESUCCESS;
    case SGI_DIOCSETVH:
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(arg_va), buf, sizeof(buf),
                            0);
        if (blk_pwrite(f->blk, 0, sizeof(buf), buf, 0) < 0) {
            return ARCS_EIO;
        }
        qemu_log("ARCS: Ioctl(DIOCSETVH)\n");
        return ARCS_ESUCCESS;
    case SGI_DIOCDRIVETYPE: {
        /* struct { char name[28]; ... } — a minimal SCSI drive type name. */
        uint8_t dt[28] = {0};
        memcpy(dt, "scsi", 4);
        cpu_memory_rw_debug(first_cpu, arcs_guest_va(arg_va), dt, sizeof(dt),
                            1);
        return ARCS_ESUCCESS;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "ARCS: Ioctl(cmd=0x%x) unimplemented\n", cmd);
        return ARCS_EINVAL;
    }
}

static uint32_t arcs_get_read_status(SGIARCSState *s, uint32_t fd)
{
    ARCSFd *f;

    if (fd >= ARCS_MAX_FDS || !arcs_fds[fd].in_use) {
        return ARCS_EINVAL;
    }
    f = &arcs_fds[fd];
    return (f->offset < f->size) ? (uint32_t)(f->size - f->offset) : 0;
}

/* ---- console chardev receive callbacks (input FIFO for Read(StandardIn)) -- */

static int arcs_chr_can_receive(void *opaque)
{
    SGIARCSState *s = SGI_ARCS(opaque);
    return sizeof(s->console_rx) - s->console_rx_len;
}

static void arcs_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIARCSState *s = SGI_ARCS(opaque);
    int room = sizeof(s->console_rx) - s->console_rx_len;
    int n = MIN(size, room);

    memcpy(s->console_rx + s->console_rx_len, buf, n);
    s->console_rx_len += n;
}

static void arcs_chr_event(void *opaque, QEMUChrEvent ev)
{
    /* console open/close needs no special handling */
}

static void arcs_hypercall(SGIARCSState *s)
{
    switch (s->func) {
    case ARCS_FN_GETMEMORYDESC:
        s->result = arcs_get_memory_desc(s, s->arg0);
        break;

    case ARCS_FN_GETENVVAR:
        s->result = arcs_get_env_var(s, s->arg0);
        break;

    case ARCS_FN_WRITE:
        s->result = arcs_write(s, s->arg0, s->arg1, s->arg2, s->arg3);
        break;

    case ARCS_FN_GETRELATIVETIME:
        s->result = arcs_relative_time++;
        break;

    case ARCS_FN_FLUSHALLCACHES:
        /* No-op in emulation */
        s->result = 0;
        break;

    case ARCS_FN_HALT:
    case ARCS_FN_POWERDOWN:
        /* Clean guest-initiated shutdown (`init 0`/halt): exit QEMU. The
         * kernel ELF + trampoline are registered ROMs, so under -kernel boot
         * there is no firmware to return to — shutting down is correct and
         * supports the disk-safety "graceful init 0" stop workflow. */
        qemu_log("ARCS: %s called -> QEMU shutdown\n",
                 s->func == ARCS_FN_HALT ? "Halt" : "PowerDown");
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        s->result = 0;
        break;

    case ARCS_FN_RESTART:
    case ARCS_FN_REBOOT:
        /* Guest reboot (`init 6`/reboot): request a system reset. On reset the
         * CPU restarts at the PROM trampoline and QEMU re-applies the kernel
         * ELF + trampoline ROM blobs, so the guest boots fresh (no PROM
         * needed). Without this the guest's ARCS reboot was a no-op and it
         * spun retrying. */
        qemu_log("ARCS: %s called -> system reset (reboot)\n",
                 s->func == ARCS_FN_RESTART ? "Restart" : "Reboot");
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        s->result = 0;
        break;

    case ARCS_FN_ENTER_INTERACTIVE:
        qemu_log("ARCS: EnterInteractiveMode called\n");
        s->result = 0;
        break;

    case ARCS_FN_GETSYSTEMID:
        /*
         * Return pointer to a SystemID struct in scratch area.
         * We pre-write it during setup.
         */
        s->result = MIPS_K0BASE + ARCS_SCRATCH_PHYS;
        break;

    case ARCS_FN_GETTIME:
        /* Return pointer to a TIMEINFO struct — just zeros for now */
        s->result = MIPS_K0BASE + ARCS_SCRATCH_PHYS + 0x10;
        break;

    case ARCS_FN_EXECUTE:
        /* Load + run the named /unix (sash autoboot). The implementation lives
         * in the machine (sgi_virtuix.c) via the registered callback. */
        if (arcs_execute_cb) {
            arcs_execute_cb(s, s->arg0, s->arg1, s->arg2, s->arg3);
        }
        s->result = 0;  /* not reached if the jump succeeds */
        break;

    case ARCS_FN_OPEN:
        s->result = arcs_open(s, s->arg0, s->arg1, s->arg2);
        break;

    case ARCS_FN_CLOSE:
        s->result = arcs_close(s, s->arg0);
        break;

    case ARCS_FN_READ:
        s->result = arcs_read(s, s->arg0, s->arg1, s->arg2, s->arg3);
        break;

    case ARCS_FN_GETREADSTATUS:
        s->result = arcs_get_read_status(s, s->arg0);
        break;

    case ARCS_FN_SEEK:
        s->result = arcs_seek(s, s->arg0, s->arg1, s->arg2);
        break;

    case ARCS_FN_SETENVVAR:
        s->result = arcs_set_env_var(s, s->arg0, s->arg1);
        break;

    case ARCS_FN_MOUNT:
    case ARCS_FN_GETDIRENTRY:
    case ARCS_FN_GETFILEINFO:
    case ARCS_FN_SETFILEINFO:
    case ARCS_FN_SAVECONFIGURATION:
    case ARCS_FN_LOAD:
    case ARCS_FN_INVOKE:
        s->result = ARCS_EIO;  /* not implemented */
        break;

    case ARCS_FN_GETPEER:
    case ARCS_FN_GETCHILD:
    case ARCS_FN_GETPARENT:
    case ARCS_FN_ADDCHILD:
    case ARCS_FN_DELETECOMPONENT:
    case ARCS_FN_GETCOMPONENT:
    case ARCS_FN_GETCONFIGDATA:
        /* Component tree queries — return NULL */
        s->result = 0;
        break;

    /* ---- PrivateVector functions ---- */

    case ARCS_PFN_GETNVRAMTAB:
        /*
         * GetNvramTab(char *addr, int size)
         * Copies PROM's env_table to kernel buffer.
         * Returns bytes NOT copied (0 = all fit).
         * We return 0 with empty data — kernel gets an empty nvram_tab.
         */
        qemu_log_mask(LOG_UNIMP,
                      "ARCS: GetNvramTab(buf=0x%08x, size=%d)\n",
                      s->arg0, s->arg1);
        s->result = 0;
        break;

    case ARCS_PFN_SGIVERS:
        /* Returns firmware version capability level (3 = relocatable ELF) */
        s->result = 3;
        break;

    case ARCS_PFN_CPUID:
        /* Returns current CPU ID (0 for single-processor Indy) */
        s->result = 0;
        break;

    case ARCS_PFN_CPUFREQ:
        /* cpufreq(int cpuid) — returns CPU frequency in MHz */
        s->result = 175;  /* R4600 @ 175MHz for Indy */
        break;

    case ARCS_PFN_GETHTP:
        /* GetHistoryTablePointer — return NULL (no history table) */
        s->result = 0;
        break;

    case ARCS_PFN_BUSINFO:
        /* BusInfo — return 0 (no bus info available) */
        s->result = 0;
        break;

    case ARCS_PFN_IOCTL:
        s->result = arcs_ioctl(s, s->arg0, s->arg1, s->arg2);
        break;

    case ARCS_PFN_FSREG:
    case ARCS_PFN_FSUNREG:
    case ARCS_PFN_SIGNAL:
        /*
         * sash's startup calls FsReg (to register its EFS/XFS/volhdr parsers),
         * and Signal. For Mode C we serve the file I/O host-side ourselves, so
         * FsReg/FsUnReg are no-op-success; Signal is a no-op too.
         */
        qemu_log("ARCS: PV function %d (no-op success)\n", s->func);
        s->result = 0;  /* ESUCCESS */
        break;

    case ARCS_PFN_LOADABS:
    case ARCS_PFN_INVOKEABS:
    case ARCS_PFN_EXECABS:
        /* Unimplemented PV functions — return error */
        qemu_log_mask(LOG_UNIMP,
                      "ARCS: unimplemented PrivateVector function %d\n",
                      s->func);
        s->result = 6;  /* EIO */
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "ARCS: unimplemented function %d\n",
                      s->func);
        s->result = 0;
        break;
    }
}

static uint64_t sgi_arcs_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIARCSState *s = SGI_ARCS(opaque);

    switch (offset) {
    case ARCS_REG_RESULT:
        return s->result;
    case ARCS_REG_FUNC:
        return s->func;
    case ARCS_REG_ARG0:
        return s->arg0;
    case ARCS_REG_ARG1:
        return s->arg1;
    case ARCS_REG_ARG2:
        return s->arg2;
    case ARCS_REG_ARG3:
        return s->arg3;
    default:
        qemu_log_mask(LOG_UNIMP, "ARCS: read from unknown offset 0x%x\n",
                      (unsigned)offset);
        return 0;
    }
}

static void sgi_arcs_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    SGIARCSState *s = SGI_ARCS(opaque);

    switch (offset) {
    case ARCS_REG_ARG0:
        s->arg0 = (uint32_t)value;
        break;
    case ARCS_REG_ARG1:
        s->arg1 = (uint32_t)value;
        break;
    case ARCS_REG_ARG2:
        s->arg2 = (uint32_t)value;
        break;
    case ARCS_REG_ARG3:
        s->arg3 = (uint32_t)value;
        break;
    case ARCS_REG_FUNC:
        /* Writing the function ID triggers the hypercall */
        s->func = (uint32_t)value;
        arcs_hypercall(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ARCS: write to unknown offset 0x%x = 0x%x\n",
                      (unsigned)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps sgi_arcs_ops = {
    .read = sgi_arcs_read,
    .write = sgi_arcs_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* MIPS stub code generation                                          */
/* ------------------------------------------------------------------ */

/*
 * Generate a MIPS stub routine for an ARCS function.
 *
 * Each stub is 10 instructions (40 bytes):
 *   lui  t0, 0xBF00       # MMIO base high (kseg1: 0xBF000100)
 *   ori  t0, t0, 0x0100   # MMIO base low
 *   sw   a0, 4(t0)        # ARG0 = a0
 *   sw   a1, 8(t0)        # ARG1 = a1
 *   sw   a2, 12(t0)       # ARG2 = a2
 *   li   t1, <func_id>    # function ID
 *   sw   t1, 0(t0)        # FUNC = trigger hypercall
 *   lw   v0, 16(t0)       # v0 = RESULT
 *   jr   ra               # return
 *   nop                   # branch delay slot
 *
 * All values are big-endian (MIPS BE). 11 instructions (44 B) so the 4-arg
 * Read/Write count-out pointer (a3) can be passed via ARCS_REG_ARG3.
 */
#define STUB_INSN_COUNT 11
#define STUB_SIZE       (STUB_INSN_COUNT * 4)

/* MIPS instruction encoding helpers */
#define MIPS_LUI(rt, imm)    (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, imm) (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ADDIU(rt, rs, imm) (0x24000000 | ((rs) << 21) | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_BNE(rs, rt, off) (0x14000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
#define MIPS_B(off)           (0x10000000 | ((off) & 0xFFFF))
#define MIPS_SW(rt, off, rs) (0xAC000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
#define MIPS_LW(rt, off, rs) (0x8C000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
#define MIPS_JR(rs)          (0x00000008 | ((rs) << 21))
#define MIPS_NOP             0x00000000
#define MIPS_WAIT            0x40800020

/* Register numbers */
#define REG_V0  2
#define REG_A0  4
#define REG_A1  5
#define REG_A2  6
#define REG_A3  7
#define REG_T0  8
#define REG_T1  9
#define REG_T2  10
#define REG_RA  31

static void generate_arcs_stub(uint32_t *buf, int func_id)
{
    /*
     * MMIO base in kseg1: physical 0x1F000100 -> kseg1 0xBF000100
     * lui  t0, 0xBF00
     * ori  t0, t0, 0x0100
     */
    buf[0] = cpu_to_be32(MIPS_LUI(REG_T0, 0xBF00));
    buf[1] = cpu_to_be32(MIPS_ORI(REG_T0, REG_T0, 0x0100));

    /* Store arguments */
    buf[2] = cpu_to_be32(MIPS_SW(REG_A0, ARCS_REG_ARG0, REG_T0));
    buf[3] = cpu_to_be32(MIPS_SW(REG_A1, ARCS_REG_ARG1, REG_T0));
    buf[4] = cpu_to_be32(MIPS_SW(REG_A2, ARCS_REG_ARG2, REG_T0));
    buf[5] = cpu_to_be32(MIPS_SW(REG_A3, ARCS_REG_ARG3, REG_T0));

    /* Load function ID into t1 and write to trigger */
    buf[6] = cpu_to_be32(MIPS_ORI(REG_T1, 0, func_id));
    buf[7] = cpu_to_be32(MIPS_SW(REG_T1, ARCS_REG_FUNC, REG_T0));

    /* Read result */
    buf[8] = cpu_to_be32(MIPS_LW(REG_V0, ARCS_REG_RESULT, REG_T0));

    /* Return */
    buf[9] = cpu_to_be32(MIPS_JR(REG_RA));
    buf[10] = cpu_to_be32(MIPS_NOP);
}

/*
 * Polling stub for Read/Write: like generate_arcs_stub, but on an
 * ARCS_RESULT_RETRY result it executes a MIPS `wait` (halts the vCPU -> main
 * loop delivers chardev input -> CP0 timer interrupt wakes us) then re-triggers
 * the hypercall. This is the blocking console read.
 */
#define POLL_STUB_INSN_COUNT 17
#define POLL_STUB_SIZE       (POLL_STUB_INSN_COUNT * 4)

static void generate_arcs_stub_poll(uint32_t *buf, int func_id)
{
    buf[0] = cpu_to_be32(MIPS_LUI(REG_T0, 0xBF00));
    buf[1] = cpu_to_be32(MIPS_ORI(REG_T0, REG_T0, 0x0100));
    buf[2] = cpu_to_be32(MIPS_SW(REG_A0, ARCS_REG_ARG0, REG_T0));
    buf[3] = cpu_to_be32(MIPS_SW(REG_A1, ARCS_REG_ARG1, REG_T0));
    buf[4] = cpu_to_be32(MIPS_SW(REG_A2, ARCS_REG_ARG2, REG_T0));
    buf[5] = cpu_to_be32(MIPS_SW(REG_A3, ARCS_REG_ARG3, REG_T0));
    buf[6] = cpu_to_be32(MIPS_ORI(REG_T1, 0, func_id));
    /* loop: */
    buf[7] = cpu_to_be32(MIPS_SW(REG_T1, ARCS_REG_FUNC, REG_T0));
    buf[8] = cpu_to_be32(MIPS_LW(REG_V0, ARCS_REG_RESULT, REG_T0));
    buf[9] = cpu_to_be32(MIPS_ADDIU(REG_T2, 0, (uint16_t)ARCS_RESULT_RETRY));
    buf[10] = cpu_to_be32(MIPS_BNE(REG_V0, REG_T2, 4));  /* != RETRY -> done */
    buf[11] = cpu_to_be32(MIPS_NOP);                     /* delay slot */
    buf[12] = cpu_to_be32(MIPS_WAIT);                    /* == RETRY: halt+yield */
    buf[13] = cpu_to_be32(MIPS_B(-7));                   /* resume: back to loop */
    buf[14] = cpu_to_be32(MIPS_NOP);                     /* delay slot */
    buf[15] = cpu_to_be32(MIPS_JR(REG_RA));              /* done */
    buf[16] = cpu_to_be32(MIPS_NOP);                     /* delay slot */
}

/* ------------------------------------------------------------------ */
/* Guest memory setup                                                 */
/* ------------------------------------------------------------------ */

/*
 * Write a big-endian uint32_t to a buffer at the given offset.
 */
static inline void put_be32(uint8_t *buf, int offset, uint32_t val)
{
    buf[offset + 0] = (val >> 24) & 0xFF;
    buf[offset + 1] = (val >> 16) & 0xFF;
    buf[offset + 2] = (val >>  8) & 0xFF;
    buf[offset + 3] =  val        & 0xFF;
}

static inline void put_be16(uint8_t *buf, int offset, uint16_t val)
{
    buf[offset + 0] = (val >> 8) & 0xFF;
    buf[offset + 1] =  val       & 0xFF;
}

void sgi_arcs_setup_stubs(SGIARCSState *s, AddressSpace *as)
{
    uint8_t spb[128];
    uint32_t stub_code[ARCS_PFN_TOTAL_COUNT * STUB_INSN_COUNT];
    uint32_t fv[ARCS_FV_SLOTS];
    uint32_t pv[ARCS_PV_SLOTS];
    uint8_t memdesc_buf[MAX_MEMDESCS * MEMDESC_STRUCT_SIZE];
    uint8_t env_buf[ARCS_ENVDATA_SIZE];
    uint8_t scratch[64];
    int stub_offset;
    int i, env_offset;

    qemu_log("ARCS: Setting up firmware stubs for direct kernel boot\n");
    qemu_log("ARCS: RAM size: %u MB, kernel end: 0x%08x\n",
             s->ram_size / (1024 * 1024), s->kernel_end_phys);

    /* ---- Generate MIPS stub routines for FV + PV ---- */
    for (i = 0; i < ARCS_PFN_TOTAL_COUNT; i++) {
        generate_arcs_stub(&stub_code[i * STUB_INSN_COUNT], i);
    }
    rom_add_blob_fixed("arcs-stubs", stub_code, sizeof(stub_code),
                       ARCS_STUBS_PHYS);

    /* Read/Write get a POLLING stub (waits on ARCS_RESULT_RETRY via MIPS wait)
     * for the blocking console read. Placed just past the uniform array; the
     * FV slots for Read/Write are re-pointed at these after the table below. */
    {
        uint32_t poll_stub[2 * POLL_STUB_INSN_COUNT];
        uint32_t poll_base = ARCS_STUBS_PHYS + ARCS_PFN_TOTAL_COUNT * STUB_SIZE;
        generate_arcs_stub_poll(&poll_stub[0], ARCS_FN_READ);
        generate_arcs_stub_poll(&poll_stub[POLL_STUB_INSN_COUNT], ARCS_FN_WRITE);
        rom_add_blob_fixed("arcs-stubs-poll", poll_stub, sizeof(poll_stub),
                           poll_base);
    }

    /* ---- Build FirmwareVector (array of 35 K0SEG function pointers) ---- */
    for (i = 0; i < ARCS_FV_SLOTS; i++) {
        stub_offset = ARCS_STUBS_PHYS + i * STUB_SIZE;
        fv[i] = cpu_to_be32(MIPS_K0BASE + stub_offset);
    }
    /* Re-point Read/Write at the polling stubs (overrides the uniform slots). */
    {
        uint32_t poll_base = ARCS_STUBS_PHYS + ARCS_PFN_TOTAL_COUNT * STUB_SIZE;
        fv[ARCS_FN_READ] = cpu_to_be32(MIPS_K0BASE + poll_base);
        fv[ARCS_FN_WRITE] = cpu_to_be32(MIPS_K0BASE + poll_base +
                                        POLL_STUB_SIZE);
    }
    rom_add_blob_fixed("arcs-fv", fv, sizeof(fv), ARCS_FV_PHYS);

    /* ---- Build PrivateVector (array of 13 K0SEG function pointers) ---- */
    for (i = 0; i < ARCS_PV_SLOTS; i++) {
        stub_offset = ARCS_STUBS_PHYS + (ARCS_FN_COUNT + i) * STUB_SIZE;
        pv[i] = cpu_to_be32(MIPS_K0BASE + stub_offset);
    }
    rom_add_blob_fixed("arcs-pv", pv, sizeof(pv), ARCS_PV_PHYS);

    /* ---- Build SPB (System Parameter Block) ---- */
    memset(spb, 0, sizeof(spb));

    /*
     * 32-bit SPB layout (from spb.h __USE_SPB32 version):
     *   +0x00: Signature (int32)     = SPBMAGIC
     *   +0x04: Length (uint32)       = sizeof(SPB)
     *   +0x08: Version (uint16)      = 1
     *   +0x0A: Revision (uint16)     = 10
     *   +0x0C: RestartBlock (int32)  = 0
     *   +0x10: DebugBlock (int32)    = 0
     *   +0x14: GEVector (int32)      = 0
     *   +0x18: UTLBMissVector (int32)= 0
     *   +0x1C: TVLength (uint32)     = sizeof(FirmwareVector)
     *   +0x20: TransferVector (int32)= K0 pointer to FV
     *   +0x24: PTVLength (uint32)    = sizeof(PrivateVector)
     *   +0x28: PrivateVector (int32) = K0 pointer to PV
     *   +0x2C: AdapterCount (int32)  = 0
     */
    put_be32(spb, 0x00, ARCS_SPB_MAGIC);
    put_be32(spb, 0x04, 0x44);  /* Length: 68 bytes (17 x uint32) */
    put_be16(spb, 0x08, ARCS_VERSION);
    put_be16(spb, 0x0A, ARCS_REVISION);
    put_be32(spb, 0x0C, MIPS_K1BASE + ARCS_RESTARTBLOCK_PHYS);  /* RestartBlock */
    put_be32(spb, 0x10, 0);     /* DebugBlock */
    put_be32(spb, 0x14, 0);     /* GEVector */
    put_be32(spb, 0x18, 0);     /* UTLBMissVector */
    put_be32(spb, 0x1C, ARCS_FV_SLOTS * 4);  /* TVLength */
    put_be32(spb, 0x20, MIPS_K0BASE + ARCS_FV_PHYS);  /* TransferVector */
    put_be32(spb, 0x24, ARCS_PV_SLOTS * 4);  /* PTVLength */
    put_be32(spb, 0x28, MIPS_K0BASE + ARCS_PV_PHYS);  /* PrivateVector */
    put_be32(spb, 0x2C, 0);     /* AdapterCount */

    rom_add_blob_fixed("arcs-spb", spb, sizeof(spb), ARCS_SPB_PHYS);

    /* ---- Build RestartBlock (sash's rbclrbs dereferences it) ---- */
    {
        uint8_t rb[ARCS_RESTARTBLOCK_SIZE];
        memset(rb, 0, sizeof(rb));
        put_be32(rb, 0x00, ARCS_RB_SIGNATURE);  /* Signature */
        put_be32(rb, 0x04, sizeof(rb));          /* Length */
        put_be16(rb, 0x08, ARCS_VERSION);        /* Version */
        put_be16(rb, 0x0A, ARCS_REVISION);       /* Revision */
        /* Next/RestartAddress/BootMasterID/ProcessorID/BootStatus/Checksum
         * all 0 (Checksum is recomputed by sash's checksum_rb — which sums to
         * 0 due to its sizeof(rb) quirk). SSALength: */
        put_be32(rb, 0x24, 512);                  /* SSALength = SSA_WORDS*4 */
        rom_add_blob_fixed("arcs-restart", rb, sizeof(rb),
                           ARCS_RESTARTBLOCK_PHYS);
    }

    /* ---- Build sash argv/envp (argc=2 + environ so getenv/kernel_name work) */
    {
        uint8_t sash_args[0x300];
        uint32_t env_ptrs[16];
        uint32_t cursor;
        int nenv = 0, j;

        memset(sash_args, 0, sizeof(sash_args));
        memcpy(&sash_args[ARCS_SASH_ARGS_STR_OFF], "dksc(0,1,0)/sash",
               sizeof("dksc(0,1,0)/sash"));
        /* argv[1] selects autoboot vs sash command monitor. "OSLoadOptions=auto"
         * autoboots; any other "OSLoadOptions=..." value falls through sash's
         * main() to command_parser (the "sash: " prompt) — the C3/recovery and
         * file-service test path. */
        const char *osopts = getenv("SGI_MODE_C_SASH_CMD") ? "OSLoadOptions=cmd"
                                                           : "OSLoadOptions=auto";
        memcpy(&sash_args[ARCS_SASH_ARGS_STR1_OFF], osopts, strlen(osopts) + 1);

        /* environ: the same vars the ARCS GetEnvironmentVariable serves. */
        cursor = ARCS_SASH_ARGS_ENV_OFF;
        for (j = 0; arcs_env_vars[j].key != NULL && nenv < 15; j++) {
            int n = snprintf((char *)&sash_args[cursor],
                             sizeof(sash_args) - cursor, "%s=%s",
                             arcs_env_vars[j].key, arcs_env_vars[j].value);
            if (n < 0 || cursor + n + 1 > ARCS_SASH_ARGS_ARGV_OFF) {
                break;
            }
            env_ptrs[nenv++] = cursor;
            cursor += n + 1;
        }

        /* argv array (fixed offset, used by the trampoline). */
        put_be32(sash_args, ARCS_SASH_ARGS_ARGV_OFF,
                 MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_STR_OFF);
        put_be32(sash_args, ARCS_SASH_ARGS_ARGV_OFF + 4,
                 MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_STR1_OFF);
        /* argv[2] = NULL (already 0). */

        /* envp array (fixed offset). */
        for (j = 0; j < nenv; j++) {
            put_be32(sash_args, ARCS_SASH_ARGS_ENVP_OFF + j * 4,
                     MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + env_ptrs[j]);
        }
        /* envp[nenv] = NULL (already 0). */

        rom_add_blob_fixed("arcs-sash-args", sash_args, sizeof(sash_args),
                           ARCS_SASH_ARGS_PHYS);
    }

    /* ---- Build memory descriptors ---- */
    memset(memdesc_buf, 0xFF, sizeof(memdesc_buf));  /* Fill with sentinel */

    /*
     * MEMORYDESCRIPTOR layout (12 bytes each, big-endian):
     *   +0: Type (uint32) — MEMORYTYPE enum
     *   +4: BasePage (int32) — physical page number (page = 4KB)
     *   +8: PageCount (int32) — number of pages
     *
     * Memory layout we describe:
     *   Desc 0: ExceptionBlock — pages 0-1 (8KB, vectors + SPB area)
     *   Desc 1: SPBPage — page 2 (our SPB/stubs extend into page 2+)
     *   Desc 2: FirmwarePermanent — pages 2-7 (our stub code area, 24KB)
     *            (overlaps SPBPage slightly, but kernel expects SPBPage first)
     *   Desc 3: FreeContiguous — rest of low memory up to RAM limit
     *
     * Note: We intentionally don't mark the kernel as LoadedProgram.
     * The kernel was loaded by our loader (not by ARCS Load()), and it
     * doesn't check its own memory descriptor type.
     */
    uint32_t kernel_start_page = s->kernel_start_phys / ARCS_PAGE_SIZE;
    uint32_t kernel_end_page = (s->kernel_end_phys + ARCS_PAGE_SIZE - 1)
                                / ARCS_PAGE_SIZE;
    uint32_t ram_pages = s->ram_size / ARCS_PAGE_SIZE;
    uint32_t firmware_end_page = 8;  /* pages 0-7 are firmware-owned */
    int desc_idx = 0;

    /* Descriptor 0: ExceptionBlock - first page (vectors) */
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
             ARCS_MEM_EXCEPTION_BLOCK);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, 0);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8, 1);
    desc_idx++;

    /* Descriptor 1: SPBPage */
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
             ARCS_MEM_SPB_PAGE);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, 1);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8, 1);
    desc_idx++;

    /* Descriptor 2: FirmwarePermanent - pages 2-7 */
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
             ARCS_MEM_FIRMWARE_PERMANENT);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, 2);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8, 6);
    desc_idx++;

    /* FreeContiguous between firmware and the kernel (if any). */
    if (kernel_start_page > firmware_end_page) {
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
                 ARCS_MEM_FREE_CONTIGUOUS);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4,
                 firmware_end_page);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8,
                 kernel_start_page - firmware_end_page);
        desc_idx++;
    }

    /* LoadedProgram - the kernel image. */
    if (kernel_end_page > kernel_start_page) {
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
                 ARCS_MEM_LOADED_PROGRAM);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4,
                 kernel_start_page);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8,
                 kernel_end_page - kernel_start_page);
        desc_idx++;
    }

    /* FreeContiguous after the kernel. */
    if (kernel_end_page < ram_pages) {
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
                 ARCS_MEM_FREE_CONTIGUOUS);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4,
                 kernel_end_page);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8,
                 ram_pages - kernel_end_page);
        desc_idx++;
    }

    /* Sentinel (0xFFFFFFFF) is already there from memset */

    rom_add_blob_fixed("arcs-memdesc", memdesc_buf, sizeof(memdesc_buf),
                       ARCS_MEMDESC_PHYS);

    qemu_log("ARCS: %d memory descriptors, kernel pages %u..%u\n",
             desc_idx, kernel_start_page, kernel_end_page);

    /* ---- Build environment variable data ---- */
    memset(env_buf, 0, sizeof(env_buf));
    env_offset = 0;

    for (i = 0; arcs_env_vars[i].key != NULL; i++) {
        int key_len = strlen(arcs_env_vars[i].key);
        int val_len = strlen(arcs_env_vars[i].value);

        if (env_offset + key_len + 1 + val_len + 1 > ARCS_ENVDATA_SIZE - 1) {
            break;  /* Out of space */
        }

        memcpy(&env_buf[env_offset], arcs_env_vars[i].key, key_len + 1);
        env_offset += key_len + 1;
        memcpy(&env_buf[env_offset], arcs_env_vars[i].value, val_len + 1);
        env_offset += val_len + 1;
    }

    /* Replay env persisted by a prior boot's SetEnvironmentVariable. */
    {
        int overrides = arcs_env_apply_persisted((char *)env_buf, &env_offset,
                                                 ARCS_ENVDATA_SIZE);
        if (overrides) {
            qemu_log("ARCS: applied %d persisted env override(s)\n", overrides);
        }
    }

    rom_add_blob_fixed("arcs-env", env_buf, sizeof(env_buf),
                       ARCS_ENVDATA_PHYS);

    /* ---- Build scratch area (SystemID, TimeInfo, etc.) ---- */
    memset(scratch, 0, sizeof(scratch));

    /*
     * SystemID at scratch+0 (16 bytes):
     *   +0: VendorId[8] = "SGI\0\0\0\0\0"
     *   +8: ProductId[8] = "IP55\0\0\0\0"  (IP55 branding, C4)
     */
    memcpy(&scratch[0], "SGI\0\0\0\0\0", 8);
    memcpy(&scratch[8], "IP55\0\0\0\0", 8);

    /* TimeInfo at scratch+0x10 (all zeros = epoch) */

    rom_add_blob_fixed("arcs-scratch", scratch, sizeof(scratch),
                       ARCS_SCRATCH_PHYS);

    /* (PrivateVector already written above with FV stubs) */

    /* ---- Build kernel environ array ("key=value" strings) ---- */
    /*
     * The kernel's getargs() parses environ (a2) as an array of
     * "key=value\0" string pointers. kopt_find() then searches this
     * parsed table. Without it, kernel options like dbaud, console,
     * cpufreq remain empty strings causing divide-by-zero in UART init.
     */
    {
        uint8_t env_strs[ARCS_ENVSTRS_SIZE];
        uint32_t env_ptrs[18];  /* up to 17 environ entries + NULL (kernname) */
        int str_off = 0;
        int ptr_idx = 0;

        memset(env_strs, 0, sizeof(env_strs));
        memset(env_ptrs, 0, sizeof(env_ptrs));

        for (i = 0; arcs_env_vars[i].key != NULL; i++) {
            int key_len = strlen(arcs_env_vars[i].key);
            int val_len = strlen(arcs_env_vars[i].value);
            int entry_len = key_len + 1 + val_len + 1; /* "key=value\0" */

            if (str_off + entry_len > (int)sizeof(env_strs) - 1) {
                break;
            }
            if (ptr_idx >= 17) {
                break;
            }

            /* Write pointer to this string (K0SEG address) */
            env_ptrs[ptr_idx++] = cpu_to_be32(
                MIPS_K0BASE + ARCS_ENVSTRS_PHYS + str_off);

            /* Write "key=value\0" */
            memcpy(&env_strs[str_off], arcs_env_vars[i].key, key_len);
            str_off += key_len;
            env_strs[str_off++] = '=';
            memcpy(&env_strs[str_off], arcs_env_vars[i].value, val_len);
            str_off += val_len;
            env_strs[str_off++] = '\0';
        }

        /* NULL terminator for the pointer array */
        env_ptrs[ptr_idx] = 0;

        rom_add_blob_fixed("arcs-environ-ptrs", env_ptrs,
                           (ptr_idx + 1) * 4, ARCS_ENVIRON_PHYS);
        rom_add_blob_fixed("arcs-environ-strs", env_strs,
                           str_off, ARCS_ENVSTRS_PHYS);

        /* Record the fill level so Execute can append kernname (the one env
         * var sash adds via setenv before calling Execute). */
        s->kernel_env_count = ptr_idx;
        s->kernel_env_str_off = str_off;

        qemu_log("ARCS: %d environ entries for kernel getargs()\n", ptr_idx);
    }

    /* Host-side self-test of the raw-device file services (gated, no guest
     * console needed): look up dksc(0,1,0) (the root XFS partition) in the
     * volume header and read its first 16 bytes, expecting "XFSB". (The full
     * arcs_open/arcs_read FV path is already exercised by sash's own startup
     * Open/Close on dksc(0,1,8) during sdvh_install.) */
    if (getenv("SGI_MODE_C_ARCS_SELFTEST")) {
        BlockBackend *blk = arcs_scsi_backend(0, 1);
        uint8_t vh[512];
        if (!blk) {
            qemu_log("ARCS SELFTEST: no scsi disk (bus0 unit1) -> FAIL\n");
        } else if (blk_pread(blk, 0, 512, vh, 0) < 0 ||
                   arcs_be32(&vh[0]) != SGI_VH_MAGIC) {
            qemu_log("ARCS SELFTEST: bad volume header -> FAIL\n");
        } else {
            const uint8_t *pt = &vh[SGI_VH_PARTAB_OFF + 0 * SGI_VH_PT_ENTSZ];
            uint32_t firstlbn = arcs_be32(pt + 4);
            uint32_t nblks = arcs_be32(pt + 0);
            uint8_t sb[16] = {0};
            int r = blk_pread(blk, (uint64_t)firstlbn * 512, 16, sb, 0);
            qemu_log("ARCS SELFTEST: dksc(0,1,0) firstlbn=%u nblks=%u "
                     "read_rc=%d magic=\"%.4s\"\n", firstlbn, nblks, r, sb);
            qemu_log("ARCS SELFTEST: %s\n",
                     (r == 0 && sb[0] == 'X' && sb[1] == 'F' &&
                      sb[2] == 'S' && sb[3] == 'B') ? "PASS" : "FAIL");
        }
    }
}

/* ------------------------------------------------------------------ */
/* QOM device infrastructure                                          */
/* ------------------------------------------------------------------ */

static void sgi_arcs_init(Object *obj)
{
    SGIARCSState *s = SGI_ARCS(obj);

    memory_region_init_io(&s->iomem, obj, &sgi_arcs_ops, s,
                          "sgi-arcs", ARCS_REG_SIZE);
    /*
     * The Execute hypercall (registered by sgi_virtuix.c) swaps the CPU PC and
     * calls cpu_loop_exit() from inside this region's write handler, which
     * longjmps out before access_with_adjusted_size can clear the device's
     * mem_reentrancy_guard. Disable the guard: the handler is re-entrancy-safe
     * (it only sets scalar state + does host-side XFS/ELF work), and leaving the
     * guard engaged would reject the guest kernel's own subsequent hypercalls as
     * "Blocked re-entrant IO".
     */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->console_rx_len = 0;
}

static void sgi_arcs_realize(DeviceState *dev, Error **errp)
{
    SGIARCSState *s = SGI_ARCS(dev);

    /* Bind the firmware console input FIFO to the chardev (if configured). */
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, arcs_chr_can_receive,
                                 arcs_chr_receive, arcs_chr_event,
                                 NULL, s, NULL, true);
    }
}

static void sgi_arcs_reset(DeviceState *dev)
{
    SGIARCSState *s = SGI_ARCS(dev);

    s->func = 0;
    s->arg0 = 0;
    s->arg1 = 0;
    s->arg2 = 0;
    s->arg3 = 0;
    s->result = 0;
    s->memdesc_index = 0;
    memset(arcs_fds, 0, sizeof(arcs_fds));
}

static const Property sgi_arcs_properties[] = {
    DEFINE_PROP_UINT32("ram-size", SGIARCSState, ram_size, 64 * 1024 * 1024),
    DEFINE_PROP_UINT32("kernel-start", SGIARCSState, kernel_start_phys, 0),
    DEFINE_PROP_UINT32("kernel-end", SGIARCSState, kernel_end_phys, 0),
    DEFINE_PROP_CHR("chardev", SGIARCSState, chr),
};

static void sgi_arcs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_arcs_realize;
    device_class_set_legacy_reset(dc, sgi_arcs_reset);
    device_class_set_props(dc, sgi_arcs_properties);
}

static const TypeInfo sgi_arcs_info = {
    .name = TYPE_SGI_ARCS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIARCSState),
    .instance_init = sgi_arcs_init,
    .class_init = sgi_arcs_class_init,
};

static void sgi_arcs_register_types(void)
{
    type_register_static(&sgi_arcs_info);
}

type_init(sgi_arcs_register_types)
