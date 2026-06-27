/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * IRIX shares the MIPS per-arch ELF helpers. The IRIX-specific ELF image
 * loading deltas (fixed stack address, 4+4 AUXV, SGI_ELFMAP, PRDA mmap) live
 * in the common linux-user/elfload.c guarded by TARGET_ABI_IRIX.
 *
 * Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>;
 * n64decomp/qemu-irix), GPLv2.
 */
#include "../mips/elfload.c"
