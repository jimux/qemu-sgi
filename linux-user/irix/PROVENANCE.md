# IRIX user-mode emulation — provenance

The `irixn32-linux-user` target (IRIX N32 MIPS user-mode emulation) is a
forward-port of the IRIX userland support from the **qemu-irix** project into
modern QEMU (10.x).

## Upstream lineage

- **Project:** qemu-irix — <https://github.com/n64decomp/qemu-irix>
- **Author:** Kai-Uwe Bloem `<derkub@gmail.com>`
- **License:** GPLv2 (same as QEMU).

All IRIX-specific code in this tree is derived from qemu-irix and carries the
attribution line:

> Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>; n64decomp/qemu-irix), GPLv2.

Original copyright headers are preserved verbatim where whole files were
carried over (e.g. `target/mips/irix_helper.c`, "Copyright (c) 2015 Kai-Uwe
Bloem").

## What was ported, and where it now lives

The qemu-irix fork was based on QEMU 2.11, where the IRIX MIPS `cpu_loop` lived
in a monolithic `linux-user/main.c` and the IRIX deltas were `#ifdef
TARGET_ABI_IRIX` blocks inside the then-shared `signal.c`/`elfload.c`/
`syscall.c`. QEMU 10.x split these into per-ABI files and refactored the APIs.
The port therefore distributes the IRIX code as follows:

### New per-ABI files (this directory, `linux-user/irix/`)

- `cpu_loop.c` — standalone IRIX MIPS `cpu_loop` + N32 64-bit argument
  splitting (`get_args_n32`) + the 235-entry `mips_syscall_args[]` descriptor
  table (from qemu-irix `main.c`). Defines the `irix_emulate_prda` gate.
- `signal.c` — standalone IRIX signal frame handling: `setup_frame`,
  `do_sigreturn`, `setup_rt_frame`, `do_rt_sigreturn`, `save_context`,
  `restore_context`. IRIX uses a libc-supplied signal trampoline
  (`ts->sigtramp`), not an in-frame kernel trampoline; sigreturn recovers the
  frame from the sigcontext/ucontext pointer in a1.
- `target_signal.h` — IRIX signal numbers (shared with MIPS), IRIX/SVR4 SA_*
  flags, sigaltstack, and the IRIX mcontext/ucontext layout.
- `target_errno_defs.h` — IRIX/SVR4 errno table (1..158) plus out-of-band
  values for Linux-only errnos referenced by shared code.
- `target_elf.h`, `target_cpu.h`, `target_syscall.h`, `termbits.h`,
  `syscall_nr.h` — IRIX ABI headers (N32 = 32-bit ELF, EM_MIPS, big-endian).
- `elfload.c` — thin shim onto the shared MIPS per-arch ELF helpers.

### `#ifdef TARGET_ABI_IRIX` blocks added to shared files

- `linux-user/elfload.c` — fixed user stack at `0x7fff8000`, IRIX 4+4 AUXV
  layout, `sgi_map_elf_image()` (`syssgi(SGI_ELFMAP)`), and the PRDA page
  mmap at `0x200000`.
- `linux-user/syscall.c` — the IRIX syscall implementations (syssgi, sysmp,
  sproc/sprocsp/nsproc, procblk, usync_cntl, sgiprctl, psema_cntl, sginap,
  getmountid, sysinfosgi, utssyssgi, swapctl, xstat/lxstat/fxstat, IRIX
  stat/mmap64), the usync/psema helper state, the IRIX stat marshalling, and
  the `do_fork()` `entry`/`arg` extension used by `sproc()`.
- `linux-user/syscall_defs.h` — IRIX structs/macros (`target_utsname`,
  `target_usync`, `target_prthread`, IRIX `target_irix_stat`/`stat64`,
  `TARGET_PR_*`, `TARGET_US_TIMEOUT`, `TARGET_MAP_AUTOGROW`,
  `TARGET_STAT64_VER`).
- `linux-user/qemu.h` — IRIX `TaskState` fields (`prda`, `procblk_*`,
  `is_pthread`, `is_blocked`, `termchild_sig`, `exit_sig`, `parent_task`,
  `sigtramp`, `ctx_link`) and the `find_task_state`/`find_cpu_state`/
  `sgi_map_elf_image` declarations.
- `linux-user/main.c` — `find_task_state`/`find_cpu_state` and the IRIX
  procblk condvar init in `init_task_state`.

### PRDA emulation

The IRIX Per-region Data Area (PRDA) is provided as a **real anonymous page
mapped at the fixed guest address `0x200000`** (see the elfload IRIX block and
the per-thread tid fill in `cpu_loop.c`/`clone_func`). Normal guest loads and
stores to the PRDA therefore work through ordinary guest memory, which is
sufficient for the validated workloads (static `sh`, dynamically-linked
MIPSpro `cc` via rld).

The `irix_emulate_prda` flag is defined and wired (enabled by
`QEMU_IRIXPRDA=1` in `cpu_loop.c:init_main_thread`) as the gate for the
optional per-access PRDA interposition from qemu-irix
(`target/mips/irix_helper.c` + the `tcg_gen_qemu_ld/st` wrapping in
`target/mips/translate.c`). That TCG-level helper has **not** been forward-
ported — it depended on the QEMU 2.11 `TCGMemOp`/`tcg_temp_local_*`/`cpu_env`
APIs that were removed in 10.x, and the real-page approach already satisfies
the validated programs. Porting the helper is a future optimization should a
workload require PRDA accesses to bypass the mapped page.

## Validation

Differentially validated against the working qemu-irix 2.11 binary
(`qemu-irix/build-x86/irixn32-linux-user/qemu-irixn32`):

- a static N32 IRIX `sh` runs `echo`, loops and `pwd` with byte-identical
  output;
- the dynamically-linked MIPSpro `cc` driver (via rld + `SGI_ELFMAP`) prints
  `MIPSpro Compilers: Version 7.2.1`.
