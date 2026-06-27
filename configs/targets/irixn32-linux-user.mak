# IRIX N32 userland emulation target.
# IRIX user-mode support is forward-ported from qemu-irix
# (Kai-Uwe Bloem <derkub@gmail.com>; n64decomp/qemu-irix), GPLv2.
# Based on mipsn32-linux-user; uses its own linux-user/irix/ ABI dir and
# IRIX syscall_nr.h (not the Linux syscall .tbl).
TARGET_ARCH=mips64
TARGET_ABI_MIPSN32=y
TARGET_ABI32=y
TARGET_ABI_IRIX=y
TARGET_ABI_DIR=irix
TARGET_BASE_ARCH=mips
TARGET_BIG_ENDIAN=y
TARGET_LONG_BITS=64
