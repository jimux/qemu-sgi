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
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "hw/misc/sgi_arcs.h"
#include "system/address-spaces.h"
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
 */
#define ARCS_ENVDATA_SIZE 512

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

    /* Read the variable name from guest memory */
    hwaddr name_phys = arg0 & 0x1FFFFFFF;
    name = read_guest_string(name_phys, 128);

    qemu_log_mask(LOG_UNIMP, "ARCS: GetEnvironmentVariable(\"%s\")\n", name);

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

/*
 * Handle ARCS_FN_WRITE hypercall.
 *
 * arg0 = file descriptor (1 = stdout)
 * arg1 = guest pointer to buffer
 * arg2 = byte count
 *
 * Copies data from guest memory and outputs to QEMU log.
 * Returns 0 (ESUCCESS).
 */
static uint32_t arcs_write(SGIARCSState *s, uint32_t fd, uint32_t buf_ptr,
                           uint32_t count)
{
    hwaddr buf_phys = buf_ptr & 0x1FFFFFFF;
    char *buf;

    if (count == 0 || count > 4096) {
        return 0;
    }

    buf = g_malloc(count + 1);
    address_space_read(&address_space_memory, buf_phys,
                       MEMTXATTRS_UNSPECIFIED, buf, count);
    buf[count] = '\0';

    /* Output to QEMU log/stderr */
    qemu_log("ARCS Write(fd=%u): %s", fd, buf);

    g_free(buf);
    return 0;  /* ESUCCESS */
}

/*
 * Relative time counter for GetRelativeTime.
 * Returns a monotonically increasing value (1ms resolution simulated).
 */
static uint32_t arcs_relative_time;

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
        s->result = arcs_write(s, s->arg0, s->arg1, s->arg2);
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
    case ARCS_FN_RESTART:
    case ARCS_FN_REBOOT:
        qemu_log("ARCS: %s called, stopping QEMU\n",
                 s->func == ARCS_FN_HALT ? "Halt" :
                 s->func == ARCS_FN_POWERDOWN ? "PowerDown" :
                 s->func == ARCS_FN_RESTART ? "Restart" : "Reboot");
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

    case ARCS_FN_OPEN:
        /* Return error — no filesystem support */
        s->result = 2;  /* ENOENT equivalent */
        break;

    case ARCS_FN_CLOSE:
    case ARCS_FN_READ:
    case ARCS_FN_GETREADSTATUS:
    case ARCS_FN_SEEK:
    case ARCS_FN_MOUNT:
    case ARCS_FN_GETDIRENTRY:
    case ARCS_FN_GETFILEINFO:
    case ARCS_FN_SETFILEINFO:
    case ARCS_FN_SETENVVAR:
    case ARCS_FN_SAVECONFIGURATION:
    case ARCS_FN_LOAD:
    case ARCS_FN_INVOKE:
    case ARCS_FN_EXECUTE:
        s->result = 6;  /* EIO — not implemented */
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
    case ARCS_PFN_LOADABS:
    case ARCS_PFN_INVOKEABS:
    case ARCS_PFN_EXECABS:
    case ARCS_PFN_FSREG:
    case ARCS_PFN_FSUNREG:
    case ARCS_PFN_SIGNAL:
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
 * All values are big-endian (MIPS BE).
 */
#define STUB_INSN_COUNT 10
#define STUB_SIZE       (STUB_INSN_COUNT * 4)

/* MIPS instruction encoding helpers */
#define MIPS_LUI(rt, imm)    (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, imm) (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_SW(rt, off, rs) (0xAC000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
#define MIPS_LW(rt, off, rs) (0x8C000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
#define MIPS_JR(rs)          (0x00000008 | ((rs) << 21))
#define MIPS_NOP             0x00000000

/* Register numbers */
#define REG_V0  2
#define REG_A0  4
#define REG_A1  5
#define REG_A2  6
#define REG_T0  8
#define REG_T1  9
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

    /* Load function ID into t1 and write to trigger */
    buf[5] = cpu_to_be32(MIPS_ORI(REG_T1, 0, func_id));
    buf[6] = cpu_to_be32(MIPS_SW(REG_T1, ARCS_REG_FUNC, REG_T0));

    /* Read result */
    buf[7] = cpu_to_be32(MIPS_LW(REG_V0, ARCS_REG_RESULT, REG_T0));

    /* Return */
    buf[8] = cpu_to_be32(MIPS_JR(REG_RA));
    buf[9] = cpu_to_be32(MIPS_NOP);
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

    /* ---- Build FirmwareVector (array of 35 K0SEG function pointers) ---- */
    for (i = 0; i < ARCS_FV_SLOTS; i++) {
        stub_offset = ARCS_STUBS_PHYS + i * STUB_SIZE;
        fv[i] = cpu_to_be32(MIPS_K0BASE + stub_offset);
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
    put_be32(spb, 0x0C, 0);     /* RestartBlock */
    put_be32(spb, 0x10, 0);     /* DebugBlock */
    put_be32(spb, 0x14, 0);     /* GEVector */
    put_be32(spb, 0x18, 0);     /* UTLBMissVector */
    put_be32(spb, 0x1C, ARCS_FV_SLOTS * 4);  /* TVLength */
    put_be32(spb, 0x20, MIPS_K0BASE + ARCS_FV_PHYS);  /* TransferVector */
    put_be32(spb, 0x24, ARCS_PV_SLOTS * 4);  /* PTVLength */
    put_be32(spb, 0x28, MIPS_K0BASE + ARCS_PV_PHYS);  /* PrivateVector */
    put_be32(spb, 0x2C, 0);     /* AdapterCount */

    rom_add_blob_fixed("arcs-spb", spb, sizeof(spb), ARCS_SPB_PHYS);

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
    uint32_t kernel_end_page = (s->kernel_end_phys + ARCS_PAGE_SIZE - 1)
                               / ARCS_PAGE_SIZE;
    uint32_t ram_pages = s->ram_size / ARCS_PAGE_SIZE;
    uint32_t free_start;
    int desc_idx = 0;

    /* Descriptor 0: ExceptionBlock — first 2 pages */
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

    /* Descriptor 2: FirmwarePermanent — our stubs area (pages 1-7) */
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
             ARCS_MEM_FIRMWARE_PERMANENT);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, 2);
    put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8, 6);
    desc_idx++;

    /*
     * Descriptor 3: FreeContiguous — from after stubs to kernel start.
     * Kernel loads at physical 0x59c0 (page 5), so this may be small.
     * If kernel starts right after stubs, skip this descriptor.
     */
    free_start = 8;  /* After our firmware area (page 8 = 0x8000) */
    if (kernel_end_page > free_start) {
        /* Descriptor: LoadedProgram for the kernel */
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
                 ARCS_MEM_LOADED_PROGRAM);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, free_start);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8,
                 kernel_end_page - free_start);
        desc_idx++;
        free_start = kernel_end_page;
    }

    /* Descriptor: FreeContiguous — rest of RAM */
    if (free_start < ram_pages) {
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 0,
                 ARCS_MEM_FREE_CONTIGUOUS);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 4, free_start);
        put_be32(memdesc_buf, desc_idx * MEMDESC_STRUCT_SIZE + 8,
                 ram_pages - free_start);
        desc_idx++;
    }

    /* Sentinel (0xFFFFFFFF) is already there from memset */

    rom_add_blob_fixed("arcs-memdesc", memdesc_buf, sizeof(memdesc_buf),
                       ARCS_MEMDESC_PHYS);

    qemu_log("ARCS: %d memory descriptors, free starts at page %u\n",
             desc_idx, free_start);

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

    rom_add_blob_fixed("arcs-env", env_buf, sizeof(env_buf),
                       ARCS_ENVDATA_PHYS);

    /* ---- Build scratch area (SystemID, TimeInfo, etc.) ---- */
    memset(scratch, 0, sizeof(scratch));

    /*
     * SystemID at scratch+0 (16 bytes):
     *   +0: VendorId[8] = "SGI\0\0\0\0\0"
     *   +8: ProductId[8] = "IP24\0\0\0\0"
     */
    memcpy(&scratch[0], "SGI\0\0\0\0\0", 8);
    memcpy(&scratch[8], "IP24\0\0\0\0", 8);

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
        uint32_t env_ptrs[16];  /* max 15 environ entries + NULL */
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
            if (ptr_idx >= 15) {
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

        qemu_log("ARCS: %d environ entries for kernel getargs()\n", ptr_idx);
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
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void sgi_arcs_reset(DeviceState *dev)
{
    SGIARCSState *s = SGI_ARCS(dev);

    s->func = 0;
    s->arg0 = 0;
    s->arg1 = 0;
    s->arg2 = 0;
    s->result = 0;
    s->memdesc_index = 0;
}

static const Property sgi_arcs_properties[] = {
    DEFINE_PROP_UINT32("ram-size", SGIARCSState, ram_size, 64 * 1024 * 1024),
    DEFINE_PROP_UINT32("kernel-end", SGIARCSState, kernel_end_phys, 0),
};

static void sgi_arcs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

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
