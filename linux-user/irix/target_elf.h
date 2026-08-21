/*
 * IRIX N32 target ELF definitions.
 *
 * Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>;
 * n64decomp/qemu-irix), GPLv2. Adapted to the QEMU 10.x target_elf.h shape:
 * IRIX N32 binaries are 32-bit ELF (ELFCLASS32), EM_MIPS, big-endian, and
 * carry the MIPS N32 ABI flag (EF_MIPS_ABI2).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef IRIX_TARGET_ELF_H
#define IRIX_TARGET_ELF_H

#include "target_ptrace.h"

#define ELF_CLASS               ELFCLASS32
#define ELF_MACHINE             EM_MIPS
#define EXSTACK_DEFAULT         true

/*
 * EF_MIPS_ABI2 is the o32/n32 discriminator (the same bit the binfmt_misc
 * registration masks on): n32 carries it, o32 does not. Each target accepts
 * exactly its own ABI so an o32 binary handed to qemu-irixn32 (or vice versa)
 * is refused with "Invalid ELF image", never mis-run.
 */
#ifdef TARGET_ABI_MIPSO32
#define elf_check_abi(x)        (!((x) & EF_MIPS_ABI2))
#else
#define elf_check_abi(x)        ((x) & EF_MIPS_ABI2)
#endif

#define HAVE_ELF_HWCAP          1
#define HAVE_ELF_BASE_PLATFORM  1
#define HAVE_ELF_CORE_DUMP      1

/* See linux kernel: arch/mips/include/asm/elf.h.  */
typedef struct target_elf_gregset_t {
    union {
        target_ulong reserved[45];
        struct target_pt_regs pt;
    };
} target_elf_gregset_t;

#endif
