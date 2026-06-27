#ifndef IRIX_TARGET_SYSCALL_H
#define IRIX_TARGET_SYSCALL_H

/* struct target_pt_regs lives in target_ptrace.h (10.x convention). */

#define UNAME_MACHINE "irix"
#define UNAME_MINIMUM_RELEASE "2.6.32"

#define TARGET_FORCE_SHMLBA

static inline abi_ulong target_shmlba(CPUMIPSState *env)
{
    return 0x40000;
}

#endif
