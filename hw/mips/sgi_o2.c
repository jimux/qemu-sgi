/*
 * SGI O2 (IP32, "Moosehead") — authentic CRIME/MACE/GBE model (disentangled
 * from the IP54 paravirtual machine 2026-09; IP54 now lives in
 * hw/mips/sgi_ip54pv.c, machine "sgi-ip54").
 *
 * Uniprocessor (max_cpus=1), UMA SDRAM (1 GB cap, 8 × 128 MB CRIME banks).
 * Real IP32 PROM is loaded via -bios (e.g.
 * PROM_library/bins/cpu/ip32/O2_ip32prom.rev4.18.bin); there is no bundled
 * default firmware. Do NOT re-introduce paravirtual devices here.
 *
 * Authoritative facts: resolved-notes/platform/ip32-o2.md and the wiki
 * o2-{crime,mace,gbe,vice} pages (SGI CRIME/MACE/GBE/VICE ASIC specs).
 *
 * The original QEMU O2 header follows (memory map + interrupt routing).
 *
 * QEMU SGI O2 (IP32) machine emulation
 *
 * The SGI O2 is a uniprocessor workstation based on the CRIME/MACE/GBE
 * chipset. It supports R5000, R10000, R12000, and R14000 CPUs.
 *
 * Memory map:
 *   0x00000000-0x0FFFFFFF  Main memory (256MB max)
 *   0x14000000-0x140002FF  CRIME base (CPU, memory, interrupts, timer)
 *   0x15000000-0x15004FFF  CRIME RE (rendering engine, TLBs, pixel pipe)
 *   0x16000000-0x160FFFFF  GBE (graphics framebuffer)
 *   0x17000000-0x17FFFFFF  VICE (video/image compression; stubbed)
 *   0x1F000000-0x1FFFFFFF  MACE (peripherals)
 *   0x1FC00000-0x1FC7FFFF  PROM (512KB, overlaps MACE region)
 *
 * Interrupt routing:
 *   MACE peripherals → CRIME INTSTAT bits [15:0] → CPU IP2 (IRQ 2)
 *
 * References:
 *   - MAME src/mame/sgi/ip32.cpp, crime.cpp, mace.cpp
 *   - IRIX IP32 PROM source
 *   - Linux arch/mips/sgi-ip32/
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "cpu.h"
#include "elf.h"
#include "hw/char/serial-mm.h"
#include "hw/char/serial.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/sgi_gbe.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_arcs.h"
#include "hw/misc/sgi_crime.h"
#include "hw/misc/sgi_crime_re.h"
#include "hw/misc/sgi_mace.h"
#include "hw/misc/unimp.h"
#include "hw/pci/pci.h"
#include "hw/scsi/sgi_aic7880.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"

/* O2 memory map addresses */
#define O2_RAM_BASE 0x00000000ULL
#define O2_CRIME_BASE 0x14000000ULL
#define O2_CRIME_RE_BASE 0x15000000ULL
#define O2_GBE_BASE 0x16000000ULL
#define O2_MACE_BASE 0x1F000000ULL
#define O2_PROM_BASE 0x1FC00000ULL

#define O2_PROM_SIZE (512 * KiB)
/* Real O2: 8 CRIME banks × 128 MB = 1 GB max. */
#define O2_RAM_MAX (1ULL * GiB)

/*
 * High-RAM alias base: physical 0x40000000.
 *
 * RAM above 256MB is mapped here as a flat alias so the IP54 PROM and
 * IRIX kernel can access it via XKPHYS (>4GB) or TLB (256MB–4GB).
 * PVMEM reports this address as high_base so the PROM passes correct ARCS
 * FreeMemory descriptors to the kernel.
 *
 * Note: the real O2 PROM's SizeMEM() expected CRIME bank mirrors here.
 * IP54 uses PVMEM instead and does not require that mirroring scheme.
 */
#define O2_HIGH_RAM_BASE 0x40000000ULL

static void main_cpu_reset(void *opaque) {
  MIPSCPU *cpu = opaque;
  cpu_reset(CPU(cpu));
}

/*
 * Fix a flash segment's body checksum after patching a word in its body.
 *
 * The O2 PROM uses a flash segment format: 64-byte header with 'SHDR' magic
 * at offset +8 and segLen at +12, followed by body data. The sloader validates
 * that the 32-bit big-endian word sum of the body equals zero before running
 * post1. When we NOP out an instruction, we break this checksum.
 *
 * Fix: add the original instruction value to the last body word. This restores
 * the sum to zero because: new_sum = (old_sum - old_val + 0) + old_val = 0.
 */
static void sgi_o2_fix_segment_body_checksum(uint8_t *rom, int bios_size,
                                             int patch_off, uint32_t old_val) {
  int seg_off;

  /* Scan flash segments (page-aligned at 256-byte intervals) */
  for (seg_off = 0; seg_off < bios_size - 64; seg_off += 256) {
    uint32_t magic = ldl_be_p(rom + seg_off + 8);
    uint32_t seg_len, last_word;
    int body_end, last_off;

    if (magic != 0x53484452) { /* 'SHDR' */
      continue;
    }

    seg_len = ldl_be_p(rom + seg_off + 12);
    if (seg_len < 64 || (int)seg_len > bios_size - seg_off) {
      continue;
    }

    /* Check if patched offset falls within this segment's body */
    body_end = seg_off + (int)seg_len;
    if (patch_off < seg_off + 64 || patch_off >= body_end) {
      /* Advance past this segment to avoid re-scanning its data */
      seg_off += (((int)seg_len + 255) & ~255) - 256;
      continue;
    }

    /* Found enclosing segment — adjust last body word */
    last_off = ((body_end + 3) & ~3) - 4;
    if (last_off >= 0 && last_off + 4 <= bios_size) {
      last_word = ldl_be_p(rom + last_off);
      stl_be_p(rom + last_off, last_word + old_val);
    }
    return;
  }
}

/*
 * Strip the PROM container header if present.
 *
 * IP32 PROM images come in two formats:
 *   1. Raw flash dumps: SHDR segments start at offset 0, file is 512KB.
 *      The CPU reset vector (0xBFC00000) hits the sloader immediately.
 *   2. PROM container: a 256-byte header starting with magic 'PROM'
 *      (0x50524f4d), followed by the flash data at offset 0x100.
 *      These are typically 418-432KB.
 *
 * If we detect the container format, shift the flash data to offset 0
 * so the sloader is at the reset vector. Returns the adjusted bios_size.
 */
#define PROM_CONTAINER_MAGIC 0x50524f4d /* 'PROM' */
#define PROM_CONTAINER_OFFSET 0x100     /* Flash data starts here */

static int sgi_o2_strip_prom_container(int bios_size) {
  uint8_t *rom;
  uint32_t magic;
  int flash_size;

  rom = rom_ptr(O2_PROM_BASE, bios_size);
  if (!rom || bios_size <= PROM_CONTAINER_OFFSET) {
    return bios_size;
  }

  magic = ldl_be_p(rom);
  if (magic != PROM_CONTAINER_MAGIC) {
    return bios_size;
  }

  /* Shift flash data to offset 0, overwriting the container header */
  flash_size = bios_size - PROM_CONTAINER_OFFSET;
  memmove(rom, rom + PROM_CONTAINER_OFFSET, flash_size);
  memset(rom + flash_size, 0, PROM_CONTAINER_OFFSET);

  return flash_size;
}

/*
 * Patch PROM SimpleMEMtst to skip the destructive memory test.
 *
 * The PROM's DupSLStack() saves registers on the stack via kseg0 (cached
 * at 0x80000Fxx), then calls SimpleMEMtst() → simple_memtst() which writes
 * test patterns through kseg1 (uncached at 0xA0000000-0xA0001000), covering
 * the same physical RAM as the stack.
 *
 * On real hardware, the L1 data cache protects the saved register values
 * from the uncached writes — kseg0 reads hit the cache and return the
 * original data. QEMU doesn't emulate the L1 data cache, so both kseg0
 * and kseg1 accesses hit the same physical RAM directly. The memory test
 * patterns overwrite the saved registers, causing an AdEL exception when
 * SimpleMEMtst tries to restore $ra from the corrupted stack.
 *
 * Fix: NOP out the `jal simple_memtst` instruction inside SimpleMEMtst().
 * The register save/restore frame is preserved; only the destructive test
 * loop is skipped. Emulated RAM is always perfect, so the test is moot.
 *
 * SimpleMEMtst prologue signature (MIPS big-endian):
 *   27bdff80  addiu sp, sp, -0x80
 *   ffbf0008  sd    ra, 0x08(sp)
 *   ffa40010  sd    a0, 0x10(sp)
 *   ffa50018  sd    a1, 0x18(sp)
 *   ... (10 more register saves)
 *   0cXXXXXX  jal   simple_memtst
 *   00000000  nop
 */
static void sgi_o2_patch_prom_memtest(int bios_size) {
  static const uint32_t prologue[] = {
      0x27bdff80, /* addiu sp, sp, -0x80 */
      0xffbf0008, /* sd    ra, 0x08(sp)  */
      0xffa40010, /* sd    a0, 0x10(sp)  */
      0xffa50018, /* sd    a1, 0x18(sp)  */
  };
  uint8_t *rom;
  int limit, i;

  rom = rom_ptr(O2_PROM_BASE, bios_size);
  if (!rom) {
    return;
  }

  limit = bios_size - ((int)ARRAY_SIZE(prologue) + 14) * 4;
  for (i = 0; i < limit; i += 4) {
    bool match = true;
    int j;

    for (j = 0; j < ARRAY_SIZE(prologue); j++) {
      if (ldl_be_p(rom + i + j * 4) != prologue[j]) {
        match = false;
        break;
      }
    }
    if (!match) {
      continue;
    }

    /* Found prologue — scan forward for the JAL instruction */
    for (j = ARRAY_SIZE(prologue); j < (int)ARRAY_SIZE(prologue) + 14; j++) {
      uint32_t instr = ldl_be_p(rom + i + j * 4);
      if ((instr >> 26) == 0x03) { /* JAL opcode */
        int patch_off = i + j * 4;
        stl_be_p(rom + patch_off, 0x00000000); /* NOP */
        sgi_o2_fix_segment_body_checksum(rom, bios_size, patch_off, instr);
        return;
      }
    }
  }
}

/*
 * Find a global symbol's st_value in an ELF32 MSB MIPS kernel image.
 *
 * Stock IRIX kernels (unix.IP32) are stripped, so this usually fails — but
 * hand-built/debug kernels carry a symtab. Used for the eager us_delay()
 * workaround (item 3 in the -kernel doc below): if we can resolve
 * early_delay_flag we pre-set it to 1 from the trampoline, replicating what
 * the real PROM does before Execute. Returns 0 when the symbol is absent.
 * When `file_off` is non-NULL and the symbol is found, it receives the ELF
 * file offset of the first byte backing st_value (0 if no alloc section maps
 * it) — used by sgi_o2_find_delayloop_patch() to scan the function's code.
 *
 * ELF32 header/section/symbol layouts are accessed by explicit byte offsets
 * (all BE 32-bit fields) so this is independent of host endianness/struct
 * padding. Offsets: Ehdr e_machine=18 e_shoff=32 e_shentsize=46 e_shnum=48;
 * Shdr sh_type=+4 sh_offset=+16 sh_size=+20 sh_link=+24 sh_entsize=+36
 * sh_flags=+8 sh_addr=+12; Sym st_name=+0 st_value=+4.
 */
static uint32_t sgi_o2_elf_find_sym(const char *kernel_file, const char *name,
                                    uint32_t *file_off)
{
    gchar *buf = NULL;
    gsize len = 0;
    GError *gerr = NULL;
    uint32_t result = 0;
    uint32_t shoff;
    uint16_t shentsize, shnum;
    int i;

    if (!g_file_get_contents(kernel_file, &buf, &len, &gerr)) {
        warn_report("sgi_o2: cannot read kernel '%s': %s", kernel_file,
                    gerr ? gerr->message : "unknown error");
        g_clear_error(&gerr);
        return 0;
    }

    /* Validate ELF32 MSB MIPS magic/class/data/emachine before touching it. */
    if (len < 52 || memcmp(buf, ELFMAG, SELFMAG) != 0 ||
        buf[EI_CLASS] != ELFCLASS32 || buf[EI_DATA] != ELFDATA2MSB ||
        lduw_be_p((uint8_t *)buf + 18) != EM_MIPS) {
        g_free(buf);
        return 0;
    }

    shoff = ldl_be_p((uint8_t *)buf + 32);
    shentsize = lduw_be_p((uint8_t *)buf + 46);
    shnum = lduw_be_p((uint8_t *)buf + 48);

    /* Search every SHT_SYMTAB/SHT_DYNSYM for the named symbol. */
    for (i = 0; i < shnum; i++) {
        uint32_t off = shoff + (uint32_t)i * shentsize;
        uint32_t sh_type, sh_link, sym_off, sym_size, sym_entsize, n, s;
        uint32_t stab_off, nametab_off, nametab_size;

        if (off + 40 > len) {
            break;
        }
        sh_type = ldl_be_p((uint8_t *)buf + off + 4);
        if (sh_type != SHT_SYMTAB && sh_type != SHT_DYNSYM) {
            continue;
        }
        sh_link = ldl_be_p((uint8_t *)buf + off + 24);   /* strtab section idx */
        sym_off = ldl_be_p((uint8_t *)buf + off + 16);
        sym_size = ldl_be_p((uint8_t *)buf + off + 20);
        sym_entsize = ldl_be_p((uint8_t *)buf + off + 36);
        if (sh_link >= shnum) {
            continue;
        }

        /* The linked string table section. */
        stab_off = shoff + sh_link * shentsize;
        if (stab_off + 40 > len) {
            continue;
        }
        nametab_off = ldl_be_p((uint8_t *)buf + stab_off + 16);
        nametab_size = ldl_be_p((uint8_t *)buf + stab_off + 20);

        n = sym_entsize ? sym_size / sym_entsize : 0;
        for (s = 0; s < n; s++) {
            uint32_t so = sym_off + s * sym_entsize;
            uint32_t st_name;
            const char *sname;

            if (so + 16 > len) {
                break;
            }
            st_name = ldl_be_p((uint8_t *)buf + so + 0);
            if (st_name >= nametab_size || nametab_off + st_name >= len) {
                continue;
            }
            sname = (const char *)buf + nametab_off + st_name;
            if (!memchr(sname, '\0', len - (nametab_off + st_name))) {
                continue;
            }
            if (strcmp(sname, name) == 0) {
                result = ldl_be_p((uint8_t *)buf + so + 4); /* st_value */
                if (file_off) {
                    int k;

                    *file_off = 0;
                    /* Map st_value -> ELF file offset via the SHF_ALLOC
                     * section whose [addr, addr+size) contains it. */
                    for (k = 0; k < shnum; k++) {
                        uint32_t o2 = shoff + (uint32_t)k * shentsize;
                        uint32_t s_flags, s_addr, s_off, s_size;

                        if (o2 + 40 > len) {
                            break;
                        }
                        s_flags = ldl_be_p((uint8_t *)buf + o2 + 8);
                        s_addr = ldl_be_p((uint8_t *)buf + o2 + 12);
                        s_off = ldl_be_p((uint8_t *)buf + o2 + 16);
                        s_size = ldl_be_p((uint8_t *)buf + o2 + 20);
                        if ((s_flags & SHF_ALLOC) && result >= s_addr &&
                            result < s_addr + s_size) {
                            *file_off = s_off + (result - s_addr);
                            break;
                        }
                    }
                }
                qemu_log("sgi_o2: symbol '%s' -> va 0x%08x\n", name, result);
                g_free(buf);
                return result;
            }
        }
    }

    g_free(buf);
    return 0;
}

/*
 * Locate the instruction in delayloop() that the trampoline rewrites and
 * return its PHYSICAL address (kseg0 VA - 0x80000000), or 0 when it can't be
 * found.
 *
 * delayloop() (irix/kern/ml/delayasm.s) does `subu a0,a0,a1` before
 * `bgtz a0,.`, decrementing by the PDA decinsperloop value us_delay loaded
 * into a1. Pre-calibration that field is 0, so the spin never terminates.
 * The stock mxview kernel put the SUBU at phys 0x169a4, but that offset
 * drifts between kernel builds, so resolve delayloop's symbol and scan its
 * first 0x40 bytes for the BE opcode 0x00852023 (subu a0,a0,a1).
 */
#define DELAYLOOP_SUBU_A0_A1_BE 0x00852023U
static uint32_t sgi_o2_find_delayloop_patch(const char *kernel_file)
{
    gchar *buf = NULL;
    gsize len = 0;
    GError *gerr = NULL;
    uint32_t file_off = 0;
    uint32_t va = sgi_o2_elf_find_sym(kernel_file, "delayloop", &file_off);
    uint32_t off;

    if (!va || !file_off) {
        return 0;
    }
    if (va < 0x80000000 || va >= 0xA0000000) {
        qemu_log("sgi_o2: delayloop va 0x%08x outside kseg0 — no patch\n",
                 va);
        return 0;
    }

    if (!g_file_get_contents(kernel_file, &buf, &len, &gerr)) {
        warn_report("sgi_o2: cannot scan kernel '%s': %s", kernel_file,
                    gerr ? gerr->message : "unknown error");
        g_clear_error(&gerr);
        return 0;
    }

    for (off = 0; off < 0x40; off += 4) {
        if (file_off + off + 4 > len) {
            break;
        }
        if (ldl_be_p((uint8_t *)buf + file_off + off) ==
            DELAYLOOP_SUBU_A0_A1_BE) {
            uint32_t phys = (va - 0x80000000) + off;

            qemu_log("sgi_o2: delayloop 'subu a0,a0,a1' at va 0x%08x "
                     "(+0x%x), phys 0x%08x — patch to addiu a0,a0,-7\n",
                     va + off, off, phys);
            g_free(buf);
            return phys;
        }
    }

    qemu_log("sgi_o2: delayloop 'subu a0,a0,a1' not found in first 0x40 "
             "bytes — patch skipped\n");
    g_free(buf);
    return 0;
}

/*
 * Direct kernel boot (-kernel) trampoline.
 *
 * When -kernel is given the PROM is NOT loaded; the ROM region is empty and
 * the CPU's reset vector (0xBFC00000) lands here. Mirrors sgi_indy.c's
 * write_kernel_trampoline(), plus the four IP32-specific workarounds the real
 * PROM performs before handing off to /unix:
 *
 *  1. VPDA TLB: kernel accesses the PDA at virtual 0xFFFFA000 via $zero-
 *     relative addressing (e.g. `lw v1,-24360(zero)` → 0xFFFFA0E8). kseg2 is
 *     TLB-mapped, so without a wired entry this TLB-misses into an infinite
 *     exception loop. Fix: wire TLB[0] VA 0xFFFFA000 → phys 0x7000 (one 4 KB
 *     page, ASID 0).
 *
 *  2. NULL-page TLB: before mh16550_earlyinit() the serial console code
 *     dereferences NULL (dports[].dp_cntrl == 0), so the LSR poll
 *     (dp_cntrl+0x507) and THR write (dp_cntrl+0x7) land at VA 0x507/0x7.
 *     Wire TLB[1] virtual page 0 onto the MACE 16550 serial register page
 *     (phys 0x1F390000) so those pre-init console accesses reach the REAL
 *     UART — pre-console cmn_err/panic output actually prints instead of
 *     dead-looping on a zeroed scratch page.
 *
 *  3. us_delay() hang: decinsperloop in the PDA is 0 before calibration, so
 *     delayloop() never decrements and early us_delay() spins forever. The
 *     real PROM sets early_delay_flag=1 so us_delay() uses a hardcoded loop
 *     count (irix/kern/ml/delay.c, delayasm.s). If the ELF exposes the
 *     early_delay_flag symbol we `sw 1` to it here — the store runs AFTER BSS
 *     zero-fill (ROM→RAM copy at reset) and before the jump, exactly the
 *     "direct RAM poke before jump" the workaround needs. Stripped kernels
 *     (stock unix.IP32) have no symtab, so the poke is skipped and the kernel
 *     hangs where the PROM would have set the flag — see the findings note.
 *
 *  4. Exception vectors at 0x80000000/0x80/0x180 are the kernel's own
 *     responsibility; this trampoline only clears BEV so exceptions vector
 *     there (currently unimplemented — see the findings note).
 */
static void write_o2_kernel_trampoline(uint32_t kernel_entry,
                                       uint32_t early_delay_flag_va,
                                       uint32_t delayloop_subu_phys,
                                       uint64_t ram_size)
{
#define MIPS_MFC0(rt, rd) (0x40000000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_MTC0(rt, rd) (0x40800000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_AND(rd, rs, rt)                                                   \
    (0x00000024 | ((rs) << 21) | ((rt) << 16) | ((rd) << 11))
#define MIPS_LUI(rt, imm) (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, im)                                                   \
    (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((im) & 0xFFFF))
#define MIPS_SW(base, rt, imm)                                                 \
    (0xAC000000 | ((base) << 21) | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_JR(rs) (0x00000008 | ((rs) << 21))
#define MIPS_MOVE(rd, rs) (0x00000025 | ((rs) << 21) | ((rd) << 11))
#define MIPS_NOP 0x00000000
#define MIPS_TLBWI 0x42000002

#define ZERO 0
#define A0 4
#define A1 5
#define A2 6
#define T0 8
#define T1 9
#define T2 10
#define T3 11
#define SP 29
#define CP0_INDEX 0
#define CP0_ENTRYLO0 2
#define CP0_ENTRYLO1 3
#define CP0_ENTRYHI 10
#define CP0_STATUS 12

    uint32_t tramp[64];
    int i = 0;
    uint16_t entry_hi = (kernel_entry >> 16) & 0xFFFF;
    uint16_t entry_lo = kernel_entry & 0xFFFF;
    uint64_t scratch_phys;
    uint32_t scratch_entrylo;

    /*
     * TLB[1] target: the MACE 16550 serial register page (phys 0x1F390000,
     * the page holding THR at +0x7 and LSR at +0x507). See the header comment
     * item 2 — with dp_cntrl==0 the kernel's pre-init console path touches
     * exactly these VA offsets, so mapping the NULL page here makes early
     * panic text visible on the real serial line.
     */
    scratch_phys = 0x1F390000ULL;
    scratch_entrylo = (uint32_t)((scratch_phys >> 12) << 6) | 0x1C; /* uncached */

    /* 1. Clear BEV (22), ERL (2) and EXL (1) in CP0_Status. */
    tramp[i++] = cpu_to_be32(MIPS_MFC0(T0, CP0_STATUS));
    tramp[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFBF));
    tramp[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xFFF9));
    tramp[i++] = cpu_to_be32(MIPS_AND(T0, T0, T1));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_STATUS));

    /* 2. TLB[0]: VA 0xFFFFA000 -> phys 0x7000 (PDA), 4 KB page, PFN 0x7. */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T0, ZERO, 0));       /* Index = 0 */
    tramp[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFFF));        /* EntryHi hi */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xA000));    /* EntryHi lo */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T2, ZERO, 0x01DE));  /* EntryLo0 */
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T1, CP0_ENTRYHI));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T2, CP0_ENTRYLO0));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T2, CP0_ENTRYLO1));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_INDEX));
    tramp[i++] = cpu_to_be32(MIPS_TLBWI);

    /* 3. TLB[1]: VA 0 -> scratch page, 4 KB page. */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T0, ZERO, 1));       /* Index = 1 */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T1, ZERO, 0));       /* EntryHi = 0 */
    tramp[i++] = cpu_to_be32(MIPS_LUI(T2, scratch_entrylo >> 16));
    tramp[i++] = cpu_to_be32(MIPS_ORI(T2, T2, scratch_entrylo & 0xFFFF));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T1, CP0_ENTRYHI));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T2, CP0_ENTRYLO0));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T2, CP0_ENTRYLO1));
    tramp[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_INDEX));
    tramp[i++] = cpu_to_be32(MIPS_TLBWI);

    /* 3b. If resolved, early_delay_flag = 1 (runs after BSS zero-fill). */
    if (early_delay_flag_va) {
        tramp[i++] = cpu_to_be32(MIPS_ORI(T3, ZERO, 1));
        tramp[i++] = cpu_to_be32(MIPS_LUI(T2, early_delay_flag_va >> 16));
        tramp[i++] = cpu_to_be32(MIPS_ORI(T2, T2, early_delay_flag_va & 0xFFFF));
        tramp[i++] = cpu_to_be32(MIPS_SW(T2, T3, 0));
    }

    /*
     * 3c. Seed the PDA decinsperloop field so us_delay()'s uncalibrated
     * fallback path does not spin forever. The IP32 kernel's us_delay reads
     * it with `lw s1,-23992(zero)` = VA 0xFFFFA248 (PDA base 0xFFFFA000 +
     * offset 0x248), which TLB[0] maps to phys 0x7000 + 0x248 = 0x7248.
     * Before calibration this field is 0, so delayloop's decrement is 0 and
     * `bgtz a0,.` never terminates (the CRIME_DOG-write hang). Seed it to a
     * sane pre-calibration value (10; only affects early delay timing).
     */
    tramp[i++] = cpu_to_be32(MIPS_ORI(T3, ZERO, 10));
    tramp[i++] = cpu_to_be32(MIPS_LUI(T2, 0x8000));
    tramp[i++] = cpu_to_be32(MIPS_ORI(T2, T2, 0x7248));
    tramp[i++] = cpu_to_be32(MIPS_SW(T2, T3, 0));

    /*
     * 3d. Patch delayloop's decrement (`subu a0,a0,a1`) into
     * `addiu a0,a0,-7`, so the spin always terminates even while the PDA's
     * decinsperloop field is 0. us_delay's uncalibrated fallback passes
     * a1 = pda.decinsperloop (still 0 pre-calibration, and the kernel later
     * re-maps its own PDA page, so data-seeding it is unreliable). Patching
     * the instruction is page-map-independent (kernel text lives at kseg0);
     * the SUBU's physical address is resolved symbol-relatively in
     * sgi_o2_find_delayloop_patch() and written through its kseg0 VA
     * (phys + 0x80000000).
     */
    if (delayloop_subu_phys) {
        uint32_t patch_va = delayloop_subu_phys + 0x80000000;

        tramp[i++] = cpu_to_be32(MIPS_LUI(T3, 0x2484));
        tramp[i++] = cpu_to_be32(MIPS_ORI(T3, T3, 0xFFF9));
        tramp[i++] = cpu_to_be32(MIPS_LUI(T2, patch_va >> 16));
        tramp[i++] = cpu_to_be32(MIPS_ORI(T2, T2, patch_va & 0xFFFF));
        tramp[i++] = cpu_to_be32(MIPS_SW(T2, T3, 0));
    }

    /*
     * 4. ARCS args: a0=0 (argc), a1=0 (argv), a2=environ pointer array
     * (K0SEG address of ARCS_ENVIRON_PHYS) — mirrors sgi_indy.c. The kernel's
     * getargs() parses environ for "key=value" strings (dbaud, console,
     * cpufreq, ...); without it kopt_find() returns empty strings and the
     * UART baud setup divides by zero / pre-console init panics.
     */
    tramp[i++] = cpu_to_be32(MIPS_MOVE(A0, ZERO));
    tramp[i++] = cpu_to_be32(MIPS_MOVE(A1, ZERO));
    tramp[i++] =
        cpu_to_be32(MIPS_LUI(A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) >> 16));
    tramp[i++] =
        cpu_to_be32(MIPS_ORI(A2, A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) & 0xFFFF));
    tramp[i++] = cpu_to_be32(MIPS_LUI(SP, 0x8080)); /* sp = 0x80800000 (kseg0) */
    tramp[i++] = cpu_to_be32(MIPS_LUI(T0, entry_hi));
    tramp[i++] = cpu_to_be32(MIPS_ORI(T0, T0, entry_lo));
    tramp[i++] = cpu_to_be32(MIPS_JR(T0));
    tramp[i++] = cpu_to_be32(MIPS_NOP);

    rom_add_blob_fixed("o2-kernel-trampoline", tramp, i * 4, O2_PROM_BASE);

    qemu_log("sgi_o2: Wrote kernel trampoline at 0x%08x -> entry 0x%08x, "
             "TLB[0]=PDA(0xFFFFA000->0x7000) TLB[1]=NULL->UART(0->0x%08" PRIx64
             ")%s decinsperloop=10 delayloop=%s\n",
             (unsigned)O2_PROM_BASE, kernel_entry, scratch_phys,
             early_delay_flag_va ? " early_delay_flag=1" : "",
             delayloop_subu_phys ? "patched" : "unpatched");

#undef MIPS_MFC0
#undef MIPS_MTC0
#undef MIPS_AND
#undef MIPS_LUI
#undef MIPS_ORI
#undef MIPS_SW
#undef MIPS_JR
#undef MIPS_MOVE
#undef MIPS_NOP
#undef MIPS_TLBWI
#undef ZERO
#undef A0
#undef A1
#undef A2
#undef T0
#undef T1
#undef T2
#undef T3
#undef SP
#undef CP0_INDEX
#undef CP0_ENTRYLO0
#undef CP0_ENTRYLO1
#undef CP0_ENTRYHI
#undef CP0_STATUS
}

/*
 * Minimal exception vectors for -kernel boot (trampoline workaround #4).
 *
 * The 6.5.5 IP32 kernel executes in 64-bit xkseg (0xffffffff80xxxxxx,
 * identity-mapped to phys 0x0xxxxxxx). Before the kernel installs its own
 * handlers, a first fetch of a never-visited page (e.g. the cmn_err/panic
 * path at 0xffffffff80164690) takes an iTLB miss into these vectors. The
 * uTLB-refill handler below services ANY miss with a generic xkseg-identity
 * refill: EntryHi = BadVAddr & ~0x1FFF (full 64-bit VPN2), EntryLo0/1 =
 * PFN pair ((VA & 0x1FFFFFFF & ~0x1FFF) >> 12) << 6 | G|V|D|C=2, written at
 * TLB index 31 (above the wired PDA/NULL entries), then eret. t0-t6 are
 * saved/restored via a scratch area at phys 0xE00; the general-exception
 * vector stores EPC/Cause to phys 0xF80/0xF84 then spins (diagnostic).
 *
 * The image is written to RAM (address_space_write, not a ROM blob) so the
 * kernel can freely overwrite these vectors with its own handlers later.
 *
 * Source: tmp/o2-qemu/exc_vectors.S (assembled with
 * `mips-linux-gnu-as -mips3 -EB`; regenerate the word table from
 * exc_vectors.bin — layout: 0x000 TLB refill, 0x080 XTLB refill, 0x100 cache
 * error, 0x180 general, 0x200 refill handler, 0x320 total).
 */
static const uint32_t sgi_o2_exc_vectors[] = {
    0x1000007fU, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x1000005fU, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x1000ffffU, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x40096800U, 0x00094882U, 0x3129001fU, 0x2529ffffU, 0x2d290003U, 0x11200003U,
    0x00000000U, 0x10000018U, 0x00000000U, 0x40087000U, 0x40096800U, 0x3c0a8000U,
    0xad480f80U, 0xad490f84U, 0x1000ffffU, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U, 0x3c1a8000U, 0xff480e00U, 0xff490e08U, 0xff4a0e10U,
    0xff4b0e18U, 0xff4c0e20U, 0xff4d0e28U, 0xff4e0e30U, 0xff4f0e38U, 0x40295000U,
    0x312900ffU, 0x40284000U, 0x0008437aU, 0x00084378U, 0x01094025U, 0x40a85000U,
    0x00000000U, 0x42000008U, 0x400e0000U, 0x05c00004U, 0x00000000U, 0x31ce001fU,
    0x10000008U, 0x00000000U, 0x8f4e0ff0U, 0x31ce000fU, 0x35ce0010U, 0x25cf0001U,
    0x31ef000fU, 0x35ef0010U, 0xaf4f0ff0U, 0x0008537aU, 0x15400006U, 0x00000000U,
    0x3c0c007cU, 0x358ce41eU, 0x258d0040U, 0x1000000bU, 0x00000000U, 0x3c091fffU,
    0x3529ffffU, 0x01095024U, 0x3c0bffffU, 0x356be000U, 0x014b5024U, 0x000a533aU,
    0x000a61b8U, 0x358c001eU, 0x258d0040U, 0x408c1000U, 0x408d1800U, 0x408e0000U,
    0x00000000U, 0x00000000U, 0x42000002U, 0x00000000U, 0x00000000U, 0x00000000U,
    0x3c1a8000U, 0xdf480e00U, 0xdf490e08U, 0xdf4a0e10U, 0xdf4b0e18U, 0xdf4c0e20U,
    0xdf4d0e28U, 0xdf4e0e30U, 0xdf4f0e38U, 0x42000018U, 0x00000000U, 0x00000000U,
    0x00000000U, 0x00000000U,
};

static void sgi_o2_install_exc_vectors(void) {
  uint8_t buf[sizeof(sgi_o2_exc_vectors)];
  size_t i;

  for (i = 0; i < ARRAY_SIZE(sgi_o2_exc_vectors); i++) {
    stl_be_p(&buf[i * 4], sgi_o2_exc_vectors[i]);
  }
  address_space_write(&address_space_memory, 0, MEMTXATTRS_UNSPECIFIED,
                     buf, sizeof(buf));
  qemu_log("sgi_o2: exception vectors installed at phys 0x0 "
           "(uTLB refill + general stub)\n");
}

static void sgi_o2_init(MachineState *machine) {
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *prom;
  DeviceState *crime_dev;
  DeviceState *mace_dev;
  DeviceState *gbe_dev;
  MIPSCPU *cpu = NULL;
  Clock *cpuclk;
  char *filename;
  int bios_size;

  /* Validate RAM size */
  if (machine->ram_size > O2_RAM_MAX) {
    error_report("RAM size more than 1GB is not supported");
    exit(EXIT_FAILURE);
  }

  /* Create CPU clock (R5000 @ 180MHz default) */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  clock_set_hz(cpuclk, 180000000);

  /* Create N CPUs */
  int ncpus = machine->smp.cpus;
  MIPSCPU **cpus = g_new0(MIPSCPU *, ncpus);
  for (int i = 0; i < ncpus; i++) {
    cpus[i] = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpus[i]);
    cpu_mips_clock_init(cpus[i]);

    if (i == 0) {
      cpu = cpus[0];
      qemu_register_reset(main_cpu_reset, cpus[i]);
    } else {
      CPUState *cs = CPU(cpus[i]);
      cs->start_powered_off = true;
      qemu_register_reset(main_cpu_reset, cpus[i]);
    }
  }

  /*
   * RAM at physical 0x00000000 (SEG0).
   * IP32 has RAM starting at address 0 (unlike IP24 which starts at
   * 0x08000000). machine->ram is created automatically by QEMU via
   * default_ram_id.
   *
   * CRIME sits at 0x14000000, so SEG0 can hold at most 320MB of RAM.
   * For configurations > 320MB, only the first 320MB is mapped here;
   * additional RAM is accessible exclusively via SEG1 TLB aliases at
   * 0x40000000+, which the kernel manages with page tables.
   */
#define O2_SEG0_MAX (O2_CRIME_BASE - O2_RAM_BASE)
  if (machine->ram_size <= O2_SEG0_MAX) {
    memory_region_add_subregion(system_memory, O2_RAM_BASE, machine->ram);
  } else {
    MemoryRegion *seg0 = g_new(MemoryRegion, 1);
    memory_region_init_alias(seg0, OBJECT(machine), "seg0-ram",
                             machine->ram, 0, O2_SEG0_MAX);
    memory_region_add_subregion(system_memory, O2_RAM_BASE, seg0);
  }

  /*
   * SEG1 CRIME memory-bank windows at 0x40000000+.
   *
   * The real O2 PROM SizeMEM() probes eight 128 MiB bank windows in SEG1.
   * Each CRIME bank holds a 32 MiB SIMM (128 MiB SIMMs on larger machines);
   * with 32 MiB SIMMs the SIMM address-mirrors 4× within its 128 MiB window.
   * Empty banks must read 0 ("no SIMM") so the probe terminates.
   *
   * Map each populated bank's 32 MiB slice as 4 mirrored aliases across its
   * window, and cover empty windows with an unimplemented device. This is the
   * historical "SEG1 RAM Aliases" fix; the IP54-era flat >256 MiB alias that
   * replaced it is removed. TODO (milestone 3): 128 MiB SIMMs / bank interleave
   * for >256 MiB machines.
   */
  for (int b = 0; b < 8; b++) {
    hwaddr base = O2_HIGH_RAM_BASE + (hwaddr)b * 128 * MiB;
    hwaddr slice = (hwaddr)b * 32 * MiB;

    if (slice >= machine->ram_size) {
      create_unimplemented_device("o2-seg1-bank", base, 128 * MiB);
      continue;
    }
    hwaddr slice_len = MIN((uint64_t)32 * MiB, machine->ram_size - slice);
    for (int m = 0; m < 4; m++) {
      MemoryRegion *bank = g_new(MemoryRegion, 1);
      memory_region_init_alias(bank, OBJECT(machine), "o2-seg1-bank-ram",
                               machine->ram, slice, slice_len);
      memory_region_add_subregion(system_memory,
                                  base + (hwaddr)m * 32 * MiB, bank);
    }
  }

  /* PROM at 0x1FC00000 */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_rom(prom, NULL, "sgi-o2.prom", O2_PROM_SIZE, &error_fatal);
  memory_region_add_subregion(system_memory, O2_PROM_BASE, prom);

  /*
   * Load PROM/BIOS. Skipped entirely for -kernel direct boot: the trampoline
   * replaces the PROM at the reset vector (see -kernel handling below).
   */
  if (!machine->kernel_filename) {
    if (machine->firmware) {
      filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
      if (!filename) {
        error_report("Could not find firmware '%s'", machine->firmware);
        exit(EXIT_FAILURE);
      }
    } else {
      warn_report("No firmware specified; use -bios with a real IP32 PROM image");
      filename = NULL;
    }

    if (filename) {
      bios_size =
          load_image_targphys(filename, O2_PROM_BASE, O2_PROM_SIZE, NULL);
      g_free(filename);
      if (bios_size < 0) {
        error_report("Could not load PROM image");
        exit(EXIT_FAILURE);
      }
      bios_size = sgi_o2_strip_prom_container(bios_size);
      sgi_o2_patch_prom_memtest(bios_size);
    }
  }

  /* CRIME at 0x14000000 */
  crime_dev = qdev_new(TYPE_SGI_CRIME);
  /* CRIME supports at most 8 × 128MB = 1GB. Cap here to avoid uint32
   * overflow when IP54 is configured with >1GB. PVMEM handles the rest. */
  {
    uint64_t crime_ram = MIN(machine->ram_size, (uint64_t)(8 * 128 * MiB));
    qdev_prop_set_uint32(crime_dev, "ram-size", (uint32_t)crime_ram);
  }
  sysbus_realize_and_unref(SYS_BUS_DEVICE(crime_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(crime_dev), 0, O2_CRIME_BASE);

  /* Wire CRIME interrupt output to CPU IP2 (IRQ index 2) */
  sysbus_connect_irq(SYS_BUS_DEVICE(crime_dev), 0, cpu->env.irq[2]);

  /* CRIME Rendering Engine at 0x15000000 */
  {
    DeviceState *crime_re_dev = qdev_new(TYPE_SGI_CRIME_RE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(crime_re_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(crime_re_dev), 0, O2_CRIME_RE_BASE);
  }

  /* MACE at 0x1F000000 */
  mace_dev = qdev_new(TYPE_SGI_MACE);
  /* Don't pass chardev to MACE — serial port 0 is handled by serial_mm below */
  sysbus_realize_and_unref(SYS_BUS_DEVICE(mace_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(mace_dev), 0, O2_MACE_BASE);

  /*
   * MACE serial port 0 (console) using QEMU's built-in 16550 device.
   *
   * The MACE serial port is a 16550-compatible UART with 256-byte register
   * spacing (regshift=8). We create a serial_mm device and add its memory
   * region as a subregion of the MACE iomem. Subregions take priority over
   * the parent, so serial port accesses go to serial_mm instead of the
   * MACE handler. This gives us proper TX buffering and chardev interaction
   * that the custom MACE serial code lacks.
   *
   * Serial port 0 is at MACE_BASE + 0x390000 = physical 0x1F390000.
   */
  if (serial_hd(0)) {
    MemoryRegion *mace_mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(mace_dev), 0);
    serial_mm_init(mace_mr, MACE_SER1_OFFSET, 8, NULL, 115200, serial_hd(0),
                   DEVICE_BIG_ENDIAN);
  }

  /*
   * Wire MACE interrupt outputs to CRIME interrupt inputs.
   * MACE has 16 interrupt lines that map to CRIME INTSTAT bits [15:0].
   */
  {
    int i;
    for (i = 0; i < 16; i++) {
      qdev_connect_gpio_out_named(mace_dev, "crime-irq", i,
                                  qdev_get_gpio_in(crime_dev, i));
    }
  }

  /*
   * Onboard SCSI: the O2 motherboard carries two AIC-7880s on the
   * MACE PCI bus at slots 1 and 2 (= SCSI controllers 0 and 1 per
   * the kernel's get_adapter_number(slot-1) and mace_ivec INTA
   * routing to CRIME bits 8/9).  Placeholder devices for now; the
   * strict AIC-7880 is the last milestone.  Drives attach via
   * -drive if=scsi,bus=0,unit=N onto controller 0's SCSI bus.
   */
  {
    int slot;
    for (slot = 1; slot <= 2; slot++) {
      PCIBus *mace_pci = PCI_BUS(qdev_get_child_bus(mace_dev, "mace-pci"));
      PCIDevice *aic;

      assert(mace_pci);
      aic = pci_new(PCI_DEVFN(slot, 0), TYPE_SGI_AIC7880);
      qdev_prop_set_uint32(DEVICE(aic), "scsi-bus-num", slot - 1);
      pci_realize_and_unref(aic, mace_pci, &error_fatal);
    }
  }

  /* GBE at 0x16000000 */
  gbe_dev = qdev_new(TYPE_SGI_GBE);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(gbe_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(gbe_dev), 0, O2_GBE_BASE);

  /*
   * Wire GBE interrupt outputs to CRIME sources 16-19 (GBE0 = vertical
   * retrace -> crimeRetraceHandler, GBE1 = pre-blank ->
   * crimePreblankHandler [contract note §3.3 / sys/IP32.h GBE_INTR]).
   * Plumbing only — GBE does not raise these yet.
   */
  {
    int i;
    for (i = 0; i < 4; i++) {
      qdev_connect_gpio_out_named(gbe_dev, "crime-irq", i,
                                  qdev_get_gpio_in(crime_dev,
                                                   CRM_IRQ_GBE0 + i));
    }
  }

  /*
   * Unimplemented device stubs for memory probing.
   * The PROM probes memory by writing patterns and reading back.
   * Unmapped regions need to return 0 (pattern mismatch) instead
   * of causing bus errors.
   *
   * Cover the gap between installed RAM and the start of CRIME.
   * The SEG1 aliases above handle 0x40000000+.
   */
  {
    uint64_t seg0_filled = MIN(machine->ram_size, O2_SEG0_MAX);
    if (seg0_filled < O2_SEG0_MAX) {
      create_unimplemented_device("mem-probe-high",
                                  O2_RAM_BASE + seg0_filled,
                                  O2_SEG0_MAX - seg0_filled);
    }
  }

  /* Gap between CRIME base regs and CRIME RE */
  create_unimplemented_device("gap-crime-re", 0x14000280, 0x00FFFD80);
  /* Gap between CRIME RE and GBE */
  create_unimplemented_device("gap-re-gbe", 0x15005000, 0x00FFB000);
  /* Gap between GBE and VICE (VICE stubbed) */
  create_unimplemented_device("gap-gbe-vice", 0x16100000, 0x00F00000);
  /* VICE + gap between VICE and MACE (VICE stubbed) */
  create_unimplemented_device("gap-vice-mace", 0x17000000, 0x08000000);

  /*
   * Cover the gap after MACE (0x1F400000-0x1FFFFFFF).
   * The PROM ROM subregion at 0x1FC00000 takes priority, but the
   * kernel scans one word past the end of PROM (0x1FC80000) looking
   * for "SHDR" markers. Without this, that read causes a bus error.
   */
  create_unimplemented_device("gap-post-mace", 0x1F400000, 0x00C00000);

  /*
   * Direct kernel boot via -kernel (placeholder boot path while SCSI is
   * deferred). Loads an IP32 kernel ELF (ELF32 MSB, entry in kseg0/kseg1) and
   * installs the boot trampoline at the reset vector — no PROM, no sash/SCSI
   * chain. Real stock unix.IP32 still needs the remaining workarounds noted in
   * write_o2_kernel_trampoline() and progress_notes/o2_qemu/02-kernel-boot.md.
   */
  if (machine->kernel_filename) {
    uint64_t kernel_entry, kernel_high;
    long kernel_size;
    uint32_t early_delay_flag_va;
    uint32_t delayloop_subu_phys;
    DeviceState *arcs_dev;
    SGIARCSState *arcs;

    kernel_size = load_elf(machine->kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL, &kernel_entry, NULL,
                           &kernel_high, NULL, ELFDATA2MSB, EM_MIPS, 1, 0);
    if (kernel_size < 0) {
      error_report("could not load kernel '%s': %s", machine->kernel_filename,
                   load_elf_strerror(kernel_size));
      exit(1);
    }

    qemu_log("sgi_o2: Loaded kernel '%s' (%ld bytes), entry 0x%08" PRIx64
             ", high 0x%08" PRIx64 "\n",
             machine->kernel_filename, kernel_size, kernel_entry, kernel_high);

    early_delay_flag_va =
        sgi_o2_elf_find_sym(machine->kernel_filename, "early_delay_flag", NULL);
    if (early_delay_flag_va) {
      qemu_log("sgi_o2: us_delay workaround: early_delay_flag at 0x%08x, "
               "pre-setting =1 from trampoline\n",
               early_delay_flag_va);
    } else {
      qemu_log("sgi_o2: early_delay_flag not found (stripped kernel?) — "
               "us_delay BSS patch skipped; a stock kernel will hang in "
               "us_delay() after CRIME setup\n");
    }

    delayloop_subu_phys =
        sgi_o2_find_delayloop_patch(machine->kernel_filename);
    if (delayloop_subu_phys) {
      qemu_log("sgi_o2: delayloop SUBU patch target phys 0x%08x\n",
               delayloop_subu_phys);
    } else {
      qemu_log("sgi_o2: delayloop SUBU not found — us_delay uncalibrated "
               "fallback may hang\n");
    }

    write_o2_kernel_trampoline((uint32_t)kernel_entry, early_delay_flag_va,
                               delayloop_subu_phys, machine->ram_size);

    /* Exception vectors (workaround #4) — uTLB-refill handler + general
     * stub at phys 0x0 so the kernel's pre-installed-handler TLB misses
     * (e.g. first entry into the panic/cmn_err page in xkseg) are serviced
     * instead of dead-ending in zeroed vectors. */
    sgi_o2_install_exc_vectors();

    /*
     * ARCS firmware stubs (mirrors sgi_indy.c's Mode-K wiring) so the stock
     * kernel gets its SPB/FirmwareVector, memory descriptors, and env vars
     * before pre-console init.
     *
     * O2 adaptation: SGI_ARCS_MMIO_BASE (0x1F000100) sits INSIDE the MACE
     * device's 0x1F000000–0x1FFFFFFF region, so map the ARCS iomem as an
     * overlapping subregion with priority 10 so it wins over MACE (same
     * _overlap trick sgi_indy.c uses against its GIO empty-slot region).
     */
    arcs_dev = qdev_new(TYPE_SGI_ARCS);
    qdev_prop_set_uint32(arcs_dev, "ram-size", (uint32_t)machine->ram_size);
    qdev_prop_set_uint32(arcs_dev, "kernel-end",
                         (uint32_t)(kernel_high & 0x1FFFFFFF));
    /* O2 -kernel boots unix directly (no sash payload), and the O2's RAM
     * starts at phys 0 so the kernel's PT_LOAD (0x2000+) would collide with
     * the sash-oriented restart/sash-args blobs at 0x2000/0x2230. */
    qdev_prop_set_bit(arcs_dev, "sash-payload", false);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
    memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                        &SGI_ARCS(arcs_dev)->iomem, 10);

    /* Set up SPB, FirmwareVector, stubs, memory descriptors, env vars. */
    arcs = SGI_ARCS(arcs_dev);
    sgi_arcs_setup_stubs(arcs, &address_space_memory);
  }
}

static void sgi_o2_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI O2 (IP32)";
  mc->init = sgi_o2_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 64 * MiB;
  mc->default_ram_id = "sgi-o2.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R5000");
  mc->max_cpus = 1;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
  mc->no_parallel = 1;
}

static const TypeInfo sgi_o2_type = {
    .name = MACHINE_TYPE_NAME("sgi-o2"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_o2_class_init,
};

static void sgi_o2_machine_init(void) { type_register_static(&sgi_o2_type); }

type_init(sgi_o2_machine_init)
