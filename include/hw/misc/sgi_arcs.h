/*
 * SGI ARCS Firmware Stubs for Direct Kernel Boot
 *
 * Provides minimal ARCS firmware emulation so IRIX kernels loaded via
 * QEMU's -kernel flag can boot without a real PROM/sash chain.
 *
 * The implementation writes small MIPS stub routines into guest memory.
 * Each stub communicates with a host-side MMIO "hypercall" device to
 * implement ARCS callbacks (GetMemoryDescriptor, GetEnvironmentVariable,
 * Write, etc.) in C.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_ARCS_H
#define HW_MISC_SGI_ARCS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_ARCS "sgi-arcs"
OBJECT_DECLARE_SIMPLE_TYPE(SGIARCSState, SGI_ARCS)

/*
 * ARCS hypercall MMIO device registers (offsets from base).
 * MIPS stub code writes to these to invoke host-side ARCS functions.
 */
#define ARCS_REG_FUNC       0x00    /* Function ID (write triggers call) */
#define ARCS_REG_ARG0       0x04    /* First argument */
#define ARCS_REG_ARG1       0x08    /* Second argument */
#define ARCS_REG_ARG2       0x0C    /* Third argument */
#define ARCS_REG_RESULT     0x10    /* Return value (read) */
#define ARCS_REG_SIZE       0x20

/*
 * ARCS hypercall MMIO base address.
 * Placed in GIO space below Newport, in an area the PROM doesn't probe.
 */
#define SGI_ARCS_MMIO_BASE  0x1f000100ULL

/*
 * Guest physical memory layout for ARCS structures.
 * All fit within the first 8KB (pages 0-1 = ExceptionBlock).
 */
#define ARCS_SPB_PHYS       0x00001000  /* System Parameter Block */
#define ARCS_FV_PHYS        0x00001080  /* FirmwareVector (35 pointers) */
#define ARCS_PV_PHYS        0x00001110  /* PrivateVector (after FV: 35*4=0x8C) */
#define ARCS_MEMDESC_PHYS   0x00001200  /* Memory descriptors */
#define ARCS_ENVDATA_PHYS   0x00001400  /* Environment variable strings */
#define ARCS_STUBS_PHYS     0x00001600  /* MIPS stub code */
#define ARCS_SCRATCH_PHYS   0x00001E00  /* Scratch/return buffers */
/*
 * Kernel environ data — must stay below 0x2000!
 * Physical 0x2000+ aliases to 0x08002000+ where the kernel LOAD segment
 * starts, so any data there gets overwritten by the kernel image.
 */
#define ARCS_ENVIRON_PHYS   0x00001E40  /* Kernel environ pointer array */
#define ARCS_ENVSTRS_PHYS   0x00001E80  /* Kernel environ "key=value" strings */
#define ARCS_ENVSTRS_SIZE   384         /* Max bytes for environ strings */

/*
 * ARCS function IDs for the hypercall interface.
 * These match the FirmwareVector slot indices.
 */
enum {
    ARCS_FN_LOAD = 0,
    ARCS_FN_INVOKE,
    ARCS_FN_EXECUTE,
    ARCS_FN_HALT,
    ARCS_FN_POWERDOWN,
    ARCS_FN_RESTART,
    ARCS_FN_REBOOT,
    ARCS_FN_ENTER_INTERACTIVE,
    ARCS_FN_RESERVED1,
    ARCS_FN_GETPEER,
    ARCS_FN_GETCHILD,
    ARCS_FN_GETPARENT,
    ARCS_FN_GETCONFIGDATA,
    ARCS_FN_ADDCHILD,
    ARCS_FN_DELETECOMPONENT,
    ARCS_FN_GETCOMPONENT,
    ARCS_FN_SAVECONFIGURATION,
    ARCS_FN_GETSYSTEMID,
    ARCS_FN_GETMEMORYDESC,
    ARCS_FN_RESERVED2,
    ARCS_FN_GETTIME,
    ARCS_FN_GETRELATIVETIME,
    ARCS_FN_GETDIRENTRY,
    ARCS_FN_OPEN,
    ARCS_FN_CLOSE,
    ARCS_FN_READ,
    ARCS_FN_GETREADSTATUS,
    ARCS_FN_WRITE,
    ARCS_FN_SEEK,
    ARCS_FN_MOUNT,
    ARCS_FN_GETENVVAR,
    ARCS_FN_SETENVVAR,
    ARCS_FN_GETFILEINFO,
    ARCS_FN_SETFILEINFO,
    ARCS_FN_FLUSHALLCACHES,
    ARCS_FN_COUNT
};

/*
 * SGI PrivateVector function IDs (pvector.h).
 * These continue after ARCS_FN_COUNT in the hypercall function space.
 * The PrivateVector has 13 entries for 32-bit systems.
 */
enum {
    ARCS_PFN_IOCTL = ARCS_FN_COUNT,    /* 35 */
    ARCS_PFN_GETNVRAMTAB,               /* 36 */
    ARCS_PFN_LOADABS,                    /* 37 */
    ARCS_PFN_INVOKEABS,                  /* 38 */
    ARCS_PFN_EXECABS,                    /* 39 */
    ARCS_PFN_FSREG,                      /* 40 */
    ARCS_PFN_FSUNREG,                    /* 41 */
    ARCS_PFN_SIGNAL,                     /* 42 */
    ARCS_PFN_GETHTP,                     /* 43 */
    ARCS_PFN_SGIVERS,                    /* 44 */
    ARCS_PFN_CPUID,                      /* 45 */
    ARCS_PFN_BUSINFO,                    /* 46 */
    ARCS_PFN_CPUFREQ,                    /* 47 */
    ARCS_PFN_TOTAL_COUNT                 /* 48 — total FV + PV stubs */
};

#define ARCS_PV_SLOTS       13  /* 32-bit PrivateVector entries */

/* ARCS memory types (from IRIX hinv.h) */
enum {
    ARCS_MEM_EXCEPTION_BLOCK = 0,
    ARCS_MEM_SPB_PAGE,
    ARCS_MEM_FREE_CONTIGUOUS,
    ARCS_MEM_FREE_MEMORY,
    ARCS_MEM_BAD_MEMORY,
    ARCS_MEM_LOADED_PROGRAM,
    ARCS_MEM_FIRMWARE_TEMPORARY,
    ARCS_MEM_FIRMWARE_PERMANENT,
};

/* SPB magic signature: "ARCS" in big-endian */
#define ARCS_SPB_MAGIC      0x53435241

/* ARCS version/revision for 32-bit */
#define ARCS_VERSION        1
#define ARCS_REVISION       10

/* Number of FirmwareVector slots */
#define ARCS_FV_SLOTS       ARCS_FN_COUNT

/* K0/K1 segment base addresses (32-bit MIPS) */
#define MIPS_K0BASE         0x80000000
#define MIPS_K1BASE         0xA0000000

struct SGIARCSState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Hypercall registers */
    uint32_t func;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t result;

    /* Memory descriptor iteration state */
    int memdesc_index;

    /* Machine configuration */
    uint32_t ram_size;
    uint32_t kernel_end_phys;   /* Physical address after kernel image */
};

/*
 * Set up ARCS firmware stubs in guest memory.
 * Called from machine init when -kernel is used.
 *
 * Writes SPB, FirmwareVector, MIPS stub routines, memory descriptors,
 * and environment variable data into guest physical memory.
 */
void sgi_arcs_setup_stubs(SGIARCSState *s, AddressSpace *as);

#endif /* HW_MISC_SGI_ARCS_H */
