# IRIX O32 userland emulation target.
# The o32 sibling of irixn32-linux-user, restoring qemu-irix's `irix`
# target (Kai-Uwe Bloem <derkub@gmail.com>; n64decomp/qemu-irix, GPLv2):
# TARGET_ARCH=mips (32-bit CPU model, o32 is a MIPS-I/II ABI) with the
# shared linux-user/irix/ ABI directory and the IRIX syscall_nr.h.
# The corpus this target exists for is measured mips1/mips2 only with
# zero doubleword instructions (tmp/leg-o32 sweep, 2026-08-21).
TARGET_ARCH=mips
TARGET_ABI_MIPSO32=y
TARGET_ABI_IRIX=y
TARGET_ABI_DIR=irix
TARGET_BASE_ARCH=mips
TARGET_BIG_ENDIAN=y
TARGET_LONG_BITS=32
