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

#define elf_check_abi(x)        ((x) & EF_MIPS_ABI2)

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
