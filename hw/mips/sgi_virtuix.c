/*
 * QEMU Virtuix (IP55) machine emulation
 *
 * Virtuix is an SGI-Indy-derived, *virtualization-native* SMP workstation. It
 * began life as the Indy/IP24 base (chosen for its excellent emulation
 * coverage), but it is deliberately a SEPARATE machine with its OWN device
 * copies so that virtualization-native changes here never touch the authentic
 * Indy (`-M indy`, hw/mips/sgi_indy.c) and vice-versa:
 *
 *   - sgi_mc_virtuix.c       (TYPE_SGI_MC_VIRTUIX)      -- MC + host-RT counter
 *   - sgi_hpc3_virtuix.c     (TYPE_SGI_HPC3_VIRTUIX)    -- HPC3 + PS/2 (sgi-..-virtuix)
 *   - sgi_newport_virtuix.c  (TYPE_SGI_NEWPORT_VIRTUIX) -- Newport graphics
 *   - sgi_smp.c              (TYPE_SGI_SMP)             -- paravirtual SMP/IPI
 *
 * Leaf-peripheral boundary: the standalone leaf devices instantiated BY-REFERENCE
 * (WD33C93 SCSI, Z85C30 serial, Seeq Ethernet, Dallas RTC) remain SHARED -- they
 * are generic peripherals Virtuix does not diverge on, and copying them would add
 * no isolation value. Only the SGI-custom top-level controllers (above) are
 * Virtuix-private. Revisit if Virtuix ever needs to diverge on a leaf device.
 *
 * Divergence from authentic Indy that lives here (NOT in sgi_indy.c):
 *   - CPU clock defaults to 66.67 MHz (IP55_CPU_HZ-overridable); Indy stays 100 MHz.
 *   - RAM cap is 2 GiB; Indy stays 256 MiB.
 *   - MC exposes MC_REALTIME_CTR64 (host-sourced wall clock); Indy's MC does not.
 *   - SMP (sgi-smp / -accel tcg,thread=multi) is a first-class feature here.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "cpu.h"
#include "elf.h"
#include "exec/cpu-common.h"
#include "exec/cputlb.h"
#include "hw/block/block.h"
#include "hw/char/serial.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/cpu.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/sgi_glaccel.h"
#include "hw/display/sgi_newport_virtuix.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_arcs.h"
#include "hw/misc/sgi_hpc3_virtuix.h"
#include "hw/misc/sgi_mc_virtuix.h"
#include "hw/mips/sgi_xfs.h"
#include "hw/misc/sgi_pvaudio.h"
#include "hw/misc/sgi_pvchan.h"
#include "hw/misc/sgi_smp.h"
#include "hw/misc/unimp.h"
#include "hw/scsi/scsi.h"
#include "monitor/qdev.h"
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

/*
 * Memory map (Guinness/IP24-derived; identical base addresses to Indy, but a
 * private copy so Virtuix can relocate things without disturbing Indy).
 */
#define SGI_RAM_LOW_BASE 0x08000000ULL
#define SGI_RAM_HIGH_BASE 0x20000000ULL
#define SGI_GIO_GFX_BASE 0x1f000000ULL
#define SGI_GIO_EXP0_BASE 0x1f400000ULL
#define SGI_GIO_EXP1_BASE 0x1f600000ULL
#define SGI_MC_BASE 0x1fa00000ULL
#define SGI_HPC3_BASE 0x1fb80000ULL
#define SGI_PROM_BASE 0x1fc00000ULL

/*
 * Paravirtual SMP controller: free MMIO hole between the MC (ends 0x1fa1ffff)
 * and HPC3 (0x1fb80000). MUST match the IP55-MP kernel header
 * (virtuix/kernel/ip55mp_addrs.h) as the single source of truth.
 */
#define SGI_VIRTUIX_SMP_BASE 0x1fa80000ULL

/* Paravirtual host<->guest channel */
#define SGI_VIRTUIX_PVCHAN_BASE 0x1fa90000ULL

/* Paravirtual audio (ring-buffer PCM → host -audiodev).  Free MMIO hole between
 * glaccel (0x1fa20000) and the SMP controller (0x1fa80000). */
#define SGI_VIRTUIX_PVAUDIO_BASE 0x1fa30000ULL

#define SGI_PROM_SIZE (512 * KiB)
/* Virtuix RAM cap: 2 GiB (Indy stays at the authentic 256 MiB). */
#define SGI_RAM_MAX (2048 * MiB)

/* Forward declaration for kernel boot trampoline */
static void write_kernel_trampoline(uint32_t kernel_entry_32);

/*
 * GIO empty-slot stub: an undriven GIO slot reads back as all-ones, telling the
 * PROM "no device present" without modeling bus-timeout exceptions.
 */
typedef struct {
  MemoryRegion iomem;
  const char *name;
} GIOEmptySlotState;

static uint64_t gio_empty_slot_read(void *opaque, hwaddr offset,
                                    unsigned size) {
  GIOEmptySlotState *s = opaque;

  qemu_log_mask(LOG_UNIMP,
                "%s: empty slot read (size %d, offset 0x%06" HWADDR_PRIx
                ") -> 0xffffffff\n",
                s->name, size, offset);
  return 0xffffffffffffffffULL;
}

static void gio_empty_slot_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size) {
  GIOEmptySlotState *s = opaque;

  qemu_log_mask(LOG_UNIMP,
                "%s: empty slot write (size %d, offset 0x%06" HWADDR_PRIx
                ", value 0x%08" PRIx64 ")\n",
                s->name, size, offset, value);
}

static const MemoryRegionOps gio_empty_slot_ops = {
    .read = gio_empty_slot_read,
    .write = gio_empty_slot_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void create_gio_empty_slot(MemoryRegion *sysmem, const char *name,
                                  hwaddr base, hwaddr size) {
  GIOEmptySlotState *s = g_new0(GIOEmptySlotState, 1);

  s->name = name;
  memory_region_init_io(&s->iomem, NULL, &gio_empty_slot_ops, s, name, size);
  memory_region_add_subregion(sysmem, base, &s->iomem);
}

static void main_cpu_reset(void *opaque) {
  MIPSCPU *cpu = opaque;

  cpu_reset(CPU(cpu));
}

/*
 * Write a MIPS boot trampoline to the PROM area (physical 0x1FC00000) for
 * -kernel direct boot: clears BEV/ERL, sets ARCS args, sets sp, jumps to entry.
 */
static void write_kernel_trampoline(uint32_t kernel_entry_32) {
#define MIPS_MFC0(rt, rd) (0x40000000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_MTC0(rt, rd) (0x40800000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_AND(rd, rs, rt)                                                   \
  (0x00000024 | ((rs) << 21) | ((rt) << 16) | ((rd) << 11))
#define MIPS_LUI(rt, imm) (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, im)                                                   \
  (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((im) & 0xFFFF))
#define MIPS_JR(rs) (0x00000008 | ((rs) << 21))
#define MIPS_NOP 0x00000000
#define MIPS_MOVE(rd, rs) (0x00000025 | ((rs) << 21) | ((rd) << 11))
#define T0 8
#define T1 9
#define A0 4
#define A1 5
#define A2 6
#define SP 29
#define CP0_STATUS 12

  uint32_t trampoline[16];
  int i = 0;

  uint16_t entry_hi = (kernel_entry_32 >> 16) & 0xFFFF;
  uint16_t entry_lo = kernel_entry_32 & 0xFFFF;

  /* Clear BEV (bit 22) and ERL (bit 2): mask ~((1<<22)|(1<<2)) = 0xFFBFFFFB */
  trampoline[i++] = cpu_to_be32(MIPS_MFC0(T0, CP0_STATUS));
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFBF));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xFFFB));
  trampoline[i++] = cpu_to_be32(MIPS_AND(T0, T0, T1));
  trampoline[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_STATUS));

  /* a0=0 (argc), a1=0 (argv), a2=environ pointer array */
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A0, 0));
  trampoline[i++] = cpu_to_be32(MIPS_MOVE(A1, 0));
  trampoline[i++] =
      cpu_to_be32(MIPS_LUI(A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) >> 16));
  trampoline[i++] =
      cpu_to_be32(MIPS_ORI(A2, A2, (MIPS_K0BASE + ARCS_ENVIRON_PHYS) & 0xFFFF));

  /* sp = 8MB into kseg0 (0x80800000) */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(SP, 0x8080));

  /* jump to kernel entry */
  trampoline[i++] = cpu_to_be32(MIPS_LUI(T0, entry_hi));
  trampoline[i++] = cpu_to_be32(MIPS_ORI(T0, T0, entry_lo));
  trampoline[i++] = cpu_to_be32(MIPS_JR(T0));
  trampoline[i++] = cpu_to_be32(MIPS_NOP);

  rom_add_blob_fixed("kernel-trampoline", trampoline, i * 4, SGI_PROM_BASE);

  qemu_log("Virtuix: Wrote kernel trampoline at 0x%08x -> entry 0x%08x\n",
           (unsigned)SGI_PROM_BASE, kernel_entry_32);

#undef MIPS_MFC0
#undef MIPS_MTC0
#undef MIPS_AND
#undef MIPS_LUI
#undef MIPS_ORI
#undef MIPS_JR
#undef MIPS_NOP
#undef MIPS_MOVE
#undef T0
#undef T1
#undef A0
#undef A1
#undef A2
#undef SP
#undef CP0_STATUS
}

/*
 * Write a MIPS boot trampoline for Track-C Path A (run the on-disk sash).
 *
 * sash is an ECOFF/OMAGIC binary linked in kuseg at 0x10000000 (entry
 * 0x100208e0), which needs a TLB mapping before it can execute — kseg0 direct
 * mapping cannot reach kuseg. This trampoline (a) clears BEV/ERL, (b) programs
 * 16 TLB entries (16 KB pages) mapping kuseg 0x10000000-0x10080000 to physical
 * 0x09000000-0x09080000, (c) sets ARCS args (argc/argv/envp) + sp at the top of
 * the mapped window, and (d) jumps to the sash entry (kuseg, now TLB-mapped).
 * See progress_notes/ip55/prom_c3_path_a_scoping.md for the sash header facts.
 */
static void write_sash_trampoline(uint32_t sash_entry, uint32_t gp_value) {
#define MIPS_MFC0(rt, rd) (0x40000000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_MTC0(rt, rd) (0x40800000 | ((rt) << 16) | ((rd) << 11))
#define MIPS_AND(rd, rs, rt)                                                   \
  (0x00000024 | ((rs) << 21) | ((rt) << 16) | ((rd) << 11))
#define MIPS_LUI(rt, imm) (0x3C000000 | ((rt) << 16) | ((imm) & 0xFFFF))
#define MIPS_ORI(rt, rs, im)                                                   \
  (0x34000000 | ((rs) << 21) | ((rt) << 16) | ((im) & 0xFFFF))
#define MIPS_ADDIU(rt, rs, im)                                                 \
  (0x24000000 | ((rs) << 21) | ((rt) << 16) | ((im) & 0xFFFF))
#define MIPS_ADDU(rd, rs, rt)                                                  \
  (0x00000021 | ((rs) << 21) | ((rt) << 16) | ((rd) << 11))
#define MIPS_BNE(rs, rt, off)                                                  \
  (0x14000000 | ((rs) << 21) | ((rt) << 16) | ((off) & 0xFFFF))
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
#define T4 12
#define T5 13
#define SP 29
#define GP 28
#define CP0_INDEX 0
#define CP0_ENTRYLO0 2
#define CP0_ENTRYLO1 3
#define CP0_PAGEMASK 5
#define CP0_ENTRYHI 10
#define CP0_STATUS 12

  uint32_t tramp[40];
  int i = 0;
  int loop = -1;

  /* Clear BEV (bit 22) + ERL (bit 2) + EXL (bit 1): mask ~((1<<22)|(1<<2)|(1<<1))
   * = 0xFFBFFFF9. EXL must be clear too, or sash's first exception would be a
   * double-fault (the real PROM clears all three before Execute). */
  tramp[i++] = cpu_to_be32(MIPS_MFC0(T0, CP0_STATUS));
  tramp[i++] = cpu_to_be32(MIPS_LUI(T1, 0xFFBF));
  tramp[i++] = cpu_to_be32(MIPS_ORI(T1, T1, 0xFFF9));
  tramp[i++] = cpu_to_be32(MIPS_AND(T0, T0, T1));
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_STATUS));

  /* TLB: 16 entries x 16 KB pages, kuseg 0x10000000 -> phys 0x09000000. */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T0, ZERO, 0));       /* t0 = index 0 */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T4, ZERO, 16));      /* t4 = 16 (bound) */
  tramp[i++] = cpu_to_be32(MIPS_LUI(T1, 0x1000));        /* t1 = EntryHi 0x10000000 */
  tramp[i++] = cpu_to_be32(MIPS_LUI(T2, 0x0024));        /* t2 = EntryLo0 base */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T2, T2, 0x001E));    /*   = PFN 0x09000 (0x24001E) */
  tramp[i++] = cpu_to_be32(MIPS_LUI(T3, 0x0024));        /* t3 = EntryLo1 base */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T3, T3, 0x011E));    /*   = PFN 0x09004 (0x24011E) */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T5, ZERO, 0x6000));  /* t5 = PageMask 16 KB */
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T5, CP0_PAGEMASK)); /* PageMask = 16 KB */
  tramp[i++] = cpu_to_be32(MIPS_ORI(T5, ZERO, 0x8000));  /* t5 = EntryHi step (+32 KB) */

  loop = i;
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T1, CP0_ENTRYHI));  /* EntryHi */
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T2, CP0_ENTRYLO0)); /* EntryLo0 */
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T3, CP0_ENTRYLO1)); /* EntryLo1 */
  tramp[i++] = cpu_to_be32(MIPS_MTC0(T0, CP0_INDEX));    /* Index */
  tramp[i++] = cpu_to_be32(MIPS_TLBWI);                  /* write entry */
  tramp[i++] = cpu_to_be32(MIPS_ADDIU(T0, T0, 1));       /* index++ */
  tramp[i++] = cpu_to_be32(MIPS_ADDU(T1, T1, T5));       /* EntryHi += 32 KB */
  tramp[i++] = cpu_to_be32(MIPS_ADDIU(T2, T2, 0x0200));  /* EntryLo0 += PFN 8 */
  tramp[i++] = cpu_to_be32(MIPS_ADDIU(T3, T3, 0x0200));  /* EntryLo1 += PFN 8 */
  /* bne t0,t4,loop — offset relative to the delay slot (i), loop is back 10. */
  int bne_off = loop - (i + 1);
  tramp[i++] = cpu_to_be32(MIPS_BNE(T0, T4, bne_off));
  tramp[i++] = cpu_to_be32(MIPS_NOP);

  /* ARCS args: argc=2, argv=["dksc(0,1,0)/sash","OSLoadOptions=auto"], envp. */
  tramp[i++] = cpu_to_be32(MIPS_ORI(A0, ZERO, 2));       /* argc = 2 */
  tramp[i++] = cpu_to_be32(MIPS_LUI(A1,
      (MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_ARGV_OFF) >> 16));
  tramp[i++] = cpu_to_be32(MIPS_ORI(A1, A1,
      (MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_ARGV_OFF) & 0xFFFF));
  tramp[i++] = cpu_to_be32(MIPS_LUI(A2,
      (MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_ENVP_OFF) >> 16));
  tramp[i++] = cpu_to_be32(MIPS_ORI(A2, A2,
      (MIPS_K0BASE + ARCS_SASH_ARGS_PHYS + ARCS_SASH_ARGS_ENVP_OFF) & 0xFFFF));
  tramp[i++] = cpu_to_be32(MIPS_LUI(SP, 0x1008));        /* sp = 0x10080000 */

  /* gp = aouthdr gp_value (sash uses gp-relative BSS/data addressing). */
  tramp[i++] = cpu_to_be32(MIPS_LUI(GP, gp_value >> 16));
  tramp[i++] = cpu_to_be32(MIPS_ORI(GP, GP, gp_value & 0xFFFF));

  /* Jump to sash entry (kuseg). */
  tramp[i++] = cpu_to_be32(MIPS_LUI(T5, sash_entry >> 16));
  tramp[i++] = cpu_to_be32(MIPS_ORI(T5, T5, sash_entry & 0xFFFF));
  tramp[i++] = cpu_to_be32(MIPS_JR(T5));
  tramp[i++] = cpu_to_be32(MIPS_NOP);

  rom_add_blob_fixed("sash-trampoline", tramp, i * 4, SGI_PROM_BASE);

  qemu_log("Virtuix: Wrote sash trampoline at 0x%08x -> entry 0x%08x "
           "(TLB kuseg 0x10000000 -> phys 0x09000000, 16 x 16 KB)\n",
           (unsigned)SGI_PROM_BASE, sash_entry);

#undef MIPS_MFC0
#undef MIPS_MTC0
#undef MIPS_AND
#undef MIPS_LUI
#undef MIPS_ORI
#undef MIPS_ADDIU
#undef MIPS_ADDU
#undef MIPS_BNE
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
#undef T4
#undef T5
#undef SP
#undef GP
#undef CP0_INDEX
#undef CP0_ENTRYLO0
#undef CP0_ENTRYLO1
#undef CP0_PAGEMASK
#undef CP0_ENTRYHI
#undef CP0_STATUS
}

/* ------------------------------------------------------------------ */
/* Mode C — our own IP55 PROM (paravirtual ARCS firmware) bootstrap    */
/* ------------------------------------------------------------------ */
/*
 * Mode C is the endpoint of Track C: boot the disk's /unix with NO -kernel
 * and NO borrowed Indy -bios image, via a host-side paravirtual ARCS PROM.
 * See progress_notes/ip55/prom_c1_c2.md for the full sash<->ARCS contract.
 *
 * This scaffold (env-gated SGI_MODE_C=1 so the Mode K path stays byte-
 * identical) reads the SGI disk volume header host-side via the QEMU block
 * layer, validates + parses it, and reports the boot assets (sash location,
 * XFS root partition, sash ECOFF entry). It is the foundation for the
 * host-side file reader + Execute that complete the C2 gate; it does not yet
 * write guest RAM or transfer control.
 */

/* SGI disk volume header (sys/dvh.h), big-endian on disk. */
#define SGI_VH_MAGIC        0x0be5a941u
#define SGI_VH_SECTOR       512
#define SGI_VH_BOOTFILE_OFF 0x08
#define SGI_VH_VOLDIR_OFF   0x48
#define SGI_VH_NVDIR        15
#define SGI_VH_VDNAMESIZE   8
#define SGI_VH_VD_ENTSZ     16   /* char[8] name + int lbn + int nbytes */
#define SGI_VH_PARTAB_OFF   0x138
#define SGI_VH_NPARTAB      16
#define SGI_VH_PT_ENTSZ     12   /* int nblks + int firstlbn + int type */
#define SGI_VH_PTYPE_XFS    10
#define SGI_VH_PTYPE_EFS    7

static uint32_t sgi_be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t sgi_be16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

/*
 * Parse an ARCS device path ("dksc(c,u,p)" or the canonical
 * "scsi(c)disk(u)rdisk(0)partition(p)") into a SCSI unit + partition index.
 * Defaults: unit 1, partition 0 (the canonical golden boot device).
 */
static void sgi_devpath_parse(const char *dp, int *unit, int *part) {
  int c = 0, u = 1, p = 0;
  const char *q;

  *unit = 1;
  *part = 0;
  if (!dp || !*dp) {
    return;
  }
  if (sscanf(dp, "dksc(%d,%d,%d)", &c, &u, &p) == 3) {
    *unit = u;
    *part = p;
    return;
  }
  if ((q = strstr(dp, "disk(")) != NULL) {
    *unit = atoi(q + 5);
  }
  if ((q = strstr(dp, "partition(")) != NULL) {
    *part = atoi(q + 10);
  } else if ((q = strstr(dp, "part(")) != NULL) {
    *part = atoi(q + 5);
  }
}

/*
 * Load an ELF32-MSB MIPS image from a host memory buffer into guest physical
 * RAM (as reset-time ROM blobs, exactly like the -kernel path's load_elf).
 * Sets *entry (virtual) and *high_phys (end-of-image physical). Returns 0 ok.
 */
static int sgi_load_elf32_be(const uint8_t *img, size_t len, uint32_t *entry,
                             uint32_t *low_phys, uint32_t *high_phys) {
  if (len < 52 || memcmp(img, "\x7f" "ELF", 4) != 0) {
    return -1;
  }
  if (img[4] != 1 /*ELFCLASS32*/ || img[5] != 2 /*ELFDATA2MSB*/) {
    return -2;
  }
  if (sgi_be16(&img[18]) != EM_MIPS) {
    return -3;
  }

  uint32_t e_entry = sgi_be32(&img[24]);
  uint32_t e_phoff = sgi_be32(&img[28]);
  uint16_t e_phentsize = sgi_be16(&img[42]);
  uint16_t e_phnum = sgi_be16(&img[44]);
  uint32_t low = 0xFFFFFFFFu;
  uint32_t high = 0;
  int loaded = 0;

  for (int i = 0; i < e_phnum; i++) {
    const uint8_t *ph = &img[e_phoff + (size_t)i * e_phentsize];
    if (e_phoff + (size_t)(i + 1) * e_phentsize > len) {
      return -4;
    }
    uint32_t p_type = sgi_be32(&ph[0]);
    if (p_type != 1 /*PT_LOAD*/) {
      continue;
    }
    uint32_t p_offset = sgi_be32(&ph[4]);
    uint32_t p_vaddr = sgi_be32(&ph[8]);
    uint32_t p_filesz = sgi_be32(&ph[16]);
    uint32_t p_memsz = sgi_be32(&ph[20]);
    if ((uint64_t)p_offset + p_filesz > len) {
      return -5;
    }
    uint32_t phys = p_vaddr & 0x1FFFFFFF;  /* kseg0/kseg1 -> physical */
    if (p_filesz > 0) {
      char *nm = g_strdup_printf("modec-kernel-seg%d", i);
      rom_add_blob_fixed(nm, img + p_offset, p_filesz, phys);
      g_free(nm);
    }
    uint32_t seg_end = (p_vaddr + p_memsz) & 0x1FFFFFFF;
    if (seg_end > high) {
      high = seg_end;
    }
    if (phys < low) {
      low = phys;
    }
    loaded++;
  }
  if (!loaded) {
    return -6;
  }
  *entry = e_entry;
  *low_phys = low;
  *high_phys = high;
  return 0;
}

/*
 * Execute(path, argc, argv, envp) — the firmware's "load + run" for sash's
 * autoboot (Path A). Reads the named /unix from the boot disk's XFS root
 * host-side, loads it into RAM at runtime, and jumps to its entry. Registered
 * on the ARCS device via sgi_arcs_set_execute_cb().
 */
static int sgi_load_elf32_be_runtime(const uint8_t *img, size_t len,
                                     uint32_t *entry, uint32_t *high_phys);

static void sgi_virtuix_execute(SGIARCSState *arcs, uint32_t path_va)
{
    CPUState *cs = first_cpu;
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;
    DriveInfo *dinfo = drive_get(IF_SCSI, 0, 1);
    BlockBackend *blk;
    uint8_t vh[512];
    uint32_t part_firstlbn = 0;
    char path[128];
    const char *fname;
    char *slash, *paren;
    SGIXfs fs;
    SGIXfsInode kino;
    uint8_t *kbuf;
    uint64_t ksize, got;
    uint32_t kentry, khigh;
    int i, rc;

    if (!dinfo || !(blk = blk_by_legacy_dinfo(dinfo))) {
        error_report("ARCS Execute: no boot disk at scsi bus=0 unit=1");
        return;
    }
    if (cpu_memory_rw_debug(cs, path_va, path, sizeof(path) - 1, 0) < 0) {
        error_report("ARCS Execute: cannot read path at 0x%08x", path_va);
        return;
    }
    path[sizeof(path) - 1] = '\0';
    /* sash's kernel_name() yields "dksc(c,u,p)name" (no slash); take the name
     * after the last '/' or ')'. */
    slash = strrchr(path, '/');
    paren = strrchr(path, ')');
    fname = (slash > paren ? slash : paren);
    fname = fname ? fname + 1 : path;
    if (!*fname) {
        fname = "unix";
    }
    qemu_log("ARCS Execute: path=%s -> file=%s\n", path, fname);

    if (blk_pread(blk, 0, 512, vh, 0) < 0 ||
        sgi_be32(&vh[0]) != 0x0be5a941u) {
        error_report("ARCS Execute: bad volume header");
        return;
    }
    for (i = 0; i < SGI_VH_NPARTAB; i++) {
        const uint8_t *pt = &vh[SGI_VH_PARTAB_OFF + i * SGI_VH_PT_ENTSZ];
        if (sgi_be32(pt + 8) == SGI_VH_PTYPE_XFS) {
            part_firstlbn = sgi_be32(pt + 4);
            break;
        }
    }
    if (!part_firstlbn) {
        error_report("ARCS Execute: no XFS root partition");
        return;
    }
    if (sgi_xfs_mount(blk, part_firstlbn, &fs) < 0) {
        error_report("ARCS Execute: xfs mount failed");
        return;
    }
    char kpath[160];
    snprintf(kpath, sizeof(kpath), "/%s", fname);
    if (sgi_xfs_lookup(&fs, kpath, &kino) < 0) {
        error_report("ARCS Execute: '%s' not found", kpath);
        return;
    }
    ksize = kino.size;
    if (ksize == 0 || ksize > 64 * MiB) {
        error_report("ARCS Execute: implausible size %" PRIu64, ksize);
        sgi_xfs_inode_put(&kino);
        return;
    }
    kbuf = g_malloc(ksize);
    if (sgi_xfs_read(&fs, &kino, 0, ksize, kbuf, &got) < 0 || got != ksize) {
        error_report("ARCS Execute: read failed");
        g_free(kbuf);
        sgi_xfs_inode_put(&kino);
        return;
    }
    sgi_xfs_inode_put(&kino);
    qemu_log("ARCS Execute: read %s (%" PRIu64 " bytes)\n", kpath, ksize);

    rc = sgi_load_elf32_be_runtime(kbuf, ksize, &kentry, &khigh);
    g_free(kbuf);
    if (rc < 0) {
        error_report("ARCS Execute: not a loadable ELF (rc=%d)", rc);
        return;
    }
    qemu_log("ARCS Execute: entry 0x%08x, jumping to kernel\n", kentry);

    /*
     * sash setenv("kernname", <boot path>) right before Execute, but our
     * firmware passes its own static environ (a2) to the kernel — which lacks
     * kernname.  The kernel's getargs()/mload.c then logs "Kernname environment
     * variable not set by sash" and refuses to load the runtime symbol table,
     * so loadable modules (a2_dd audio) never register.  The Execute path IS
     * the kernname value (sash sets them equal), so append it to the kernel
     * environ before handing off.
     */
    sgi_arcs_set_kernel_env(arcs, "kernname", path);

    /*
     * The kernel entry (and the ARCS environ pointer) are 32-bit kseg0
     * addresses.  On a 64-bit MIPS CPU they must be SIGN-EXTENDED into the
     * 64-bit PC/GPR: a zero-extended value (e.g. 0x0000000088003c30) lands in
     * xuseg and raises EXCP_AdEL on the first fetch instead of mapping to the
     * kernel's compat-kseg0 physical address.
     *
     * cpu_loop_exit() longjmps out of the sgi-arcs MMIO write handler here; the
     * sgi-arcs region sets disable_reentrancy_guard so that leaving the handler
     * mid-flight does not leave mem_reentrancy_guard.engaged_in_io stuck (which
     * would reject the kernel's own first ARCS hypercall as "re-entrant IO").
     */
    env->active_tc.PC = (target_ulong)(int32_t)kentry;
    env->active_tc.gpr[4] = 0;                            /* a0 = argc */
    env->active_tc.gpr[5] = 0;                            /* a1 = argv */
    env->active_tc.gpr[6] =
        (target_ulong)(int32_t)(MIPS_K0BASE + ARCS_ENVIRON_PHYS);  /* a2 = environ */
    env->CP0_Status = 0;      /* clear BEV/ERL/EXL, KSU=kernel, KX=0 (32-bit entry) */
    env->CP0_EPC = 0;
    env->CP0_Cause = 0;
    env->active_tc.HI[0] = 0;
    env->active_tc.LO[0] = 0;
    tlb_flush(cs);
    cpu_loop_exit(cs);
}

/* Runtime ELF32-MSB loader (address_space_write, for sash's Execute). */
static int sgi_load_elf32_be_runtime(const uint8_t *img, size_t len,
                                     uint32_t *entry, uint32_t *high_phys)
{
    uint32_t e_entry, e_phoff;
    uint16_t e_phentsize, e_phnum;
    int loaded = 0, i;

    if (len < 52 || memcmp(img, "\x7f" "ELF", 4) != 0) {
        return -1;
    }
    if (img[4] != 1 || img[5] != 2) {
        return -2;
    }
    if (sgi_be16(&img[18]) != EM_MIPS) {
        return -3;
    }
    e_entry = sgi_be32(&img[24]);
    e_phoff = sgi_be32(&img[28]);
    e_phentsize = sgi_be16(&img[42]);
    e_phnum = sgi_be16(&img[44]);
    uint32_t high = 0;

    for (i = 0; i < e_phnum; i++) {
        const uint8_t *ph = &img[e_phoff + (size_t)i * e_phentsize];
        uint32_t p_type, p_offset, p_vaddr, p_filesz, p_memsz;
        if (e_phoff + (size_t)(i + 1) * e_phentsize > len) {
            return -4;
        }
        p_type = sgi_be32(&ph[0]);
        if (p_type != 1 /* PT_LOAD */) {
            continue;
        }
        p_offset = sgi_be32(&ph[4]);
        p_vaddr = sgi_be32(&ph[8]);
        p_filesz = sgi_be32(&ph[16]);
        p_memsz = sgi_be32(&ph[20]);
        if ((uint64_t)p_offset + p_filesz > len) {
            return -5;
        }
        uint32_t phys = p_vaddr & 0x1FFFFFFF;
        if (p_filesz > 0) {
            address_space_write(&address_space_memory, phys,
                                MEMTXATTRS_UNSPECIFIED, img + p_offset,
                                p_filesz);
        }
        if (p_memsz > p_filesz) {
            address_space_set(&address_space_memory, phys + p_filesz, 0,
                              p_memsz - p_filesz, MEMTXATTRS_UNSPECIFIED);
        }
        uint32_t seg_end = (p_vaddr + p_memsz) & 0x1FFFFFFF;
        if (seg_end > high) {
            high = seg_end;
        }
        loaded++;
    }
    if (!loaded) {
        return -6;
    }
    *entry = e_entry;
    *high_phys = high;
    return 0;
}

/*
 * Apply the ECOFF relocations in a loaded sash image (host buffer, in place).
 *
 * The on-disk sash is a RELOCATABLE ECOFF (f_flags lacks F_RELFLG): its
 * .text/.rdata/.data carry reloc records and the code/data has relocation
 * placeholders (e.g. `LA sp,scstack` compiled to `lui sp,0; addiu sp,sp,0`).
 * Since Path A loads the sash at exactly its linked vaddr (kuseg 0x10000000 ->
 * phys 0x09000000), the "displacement" for internal (section) references is 0
 * (no-op), and only EXTERNAL symbol references need the symbol value patched
 * in. Only 4 reloc types occur in the real sash (REFWORD/REFHI/REFLO/JMPADDR)
 * and the only external storage classes are Text/Bss/Data/RData (no gp-relative
 * SData/SBss), so this is the minimal applier, ported from the PROM's
 * stand/arcs/lib/libsk/lib/dload.c (doload_relocate). Returns 0 on success.
 */
static int sgi_sash_relocate(uint8_t *sash, size_t sash_size) {
  uint16_t f_nscns = sgi_be16(&sash[2]);
  uint32_t f_symptr = sgi_be32(&sash[8]);
  uint32_t vaddr[16], scnptr[16], relptr[16];
  uint32_t nreloc[16];
  uint32_t iextMax, cbExtOffset;
  uint32_t *sym_value;
  int n, i;

  if (f_nscns > 16) {
    return -1;
  }
  /* Section headers at 20 (filehdr) + 56 (aouthdr) = 76, 40 bytes each. */
  for (n = 0; n < f_nscns; n++) {
    const uint8_t *sh = &sash[76 + n * 40];
    vaddr[n] = sgi_be32(&sh[12]);
    scnptr[n] = sgi_be32(&sh[20]);
    relptr[n] = sgi_be32(&sh[24]);
    nreloc[n] = sgi_be16(&sh[32]);
  }

  /* Symbolic header (HDRR) -> external symbol table. */
  if (f_symptr == 0 || f_symptr + 96 > sash_size) {
    return 0;  /* stripped / no external symbols — nothing to do */
  }
  iextMax = sgi_be32(&sash[f_symptr + 88]);
  cbExtOffset = sgi_be32(&sash[f_symptr + 92]);
  sym_value = g_malloc0(sizeof(uint32_t) * (iextMax ? iextMax : 1));
  for (i = 0; i < (int)iextMax; i++) {
    uint32_t off = cbExtOffset + (uint32_t)i * 16;
    if (off + 16 > sash_size) {
      g_free(sym_value);
      return -2;
    }
    /* EXTR: word0(4) + asym.iss(4) + asym.value(4) + asym word(4). */
    sym_value[i] = sgi_be32(&sash[off + 8]);
  }

  /* Apply each section's reloc records in place. */
  for (n = 0; n < f_nscns; n++) {
    for (i = 0; i < (int)nreloc[n]; i++) {
      uint32_t roff = relptr[n] + (uint32_t)i * 8;
      uint32_t r_vaddr, word, r_symndx, r_type, r_extern;
      uint32_t disp, file_off;
      uint32_t *wp;
      if (roff + 8 > sash_size) {
        g_free(sym_value);
        return -3;
      }
      r_vaddr = sgi_be32(&sash[roff]);
      word = sgi_be32(&sash[roff + 4]);
      r_symndx = (word >> 8) & 0xFFFFFF;
      r_type = (word >> 1) & 0x1F;
      r_extern = word & 1;

      if (!r_extern) {
        continue;  /* internal (section) ref — loaded at vaddr, disp == 0 */
      }
      if (r_symndx >= iextMax) {
        g_free(sym_value);
        return -4;
      }
      disp = sym_value[r_symndx];
      if (disp == 0) {
        continue;
      }
      file_off = scnptr[n] + (r_vaddr - vaddr[n]);
      if (file_off + 4 > sash_size) {
        g_free(sym_value);
        return -5;
      }
      wp = (uint32_t *)(sash + file_off);

      switch (r_type) {
      case 2: /* R_REFWORD — 32-bit absolute word */
        *wp = cpu_to_be32(sgi_be32((uint8_t *)wp) + disp);
        break;
      case 3: /* R_JMPADDR — 26-bit jump target */
        *wp = cpu_to_be32((sgi_be32((uint8_t *)wp) & 0xFC000000u) |
                          ((disp >> 2) & 0x03FFFFFFu));
        break;
      case 4: { /* R_REFHI — high 16 of a lui (peek the following addiu lo) */
        uint16_t hi = sgi_be16(&sash[file_off + 2]);
        uint16_t lo = sgi_be16(&sash[file_off + 6]);
        uint32_t tw = ((uint32_t)hi << 16) | lo;
        uint16_t res;
        tw += disp;
        res = (uint16_t)(tw >> 16);
        if (tw & 0x8000) {
          res++;
        }
        sash[file_off + 2] = res >> 8;
        sash[file_off + 3] = res & 0xFF;
        break;
      }
      case 5: { /* R_REFLO — low 16 of an addiu */
        uint16_t lo = sgi_be16(&sash[file_off + 2]);
        uint16_t res = (uint16_t)(lo + disp);
        sash[file_off + 2] = res >> 8;
        sash[file_off + 3] = res & 0xFF;
        break;
      }
      default:
        g_free(sym_value);
        return -6;  /* unexpected reloc type */
      }
    }
  }

  g_free(sym_value);
  return 0;
}

/*
 * Path I (install mini-kernel) — C3 first step. Load a standalone kseg1 ECOFF
 * (the install CD's /stand/ide.IP22) from a HOST path (SGI_MODE_C_IDE, the
 * virtualization-native "boot from host path") and jump to its entry.
 *
 * ide.IP22 is a FULLY-LINKED ECOFF: f_nscns=3, no symbol table (f_symptr=0),
 * nreloc=0 on every section, linked in kseg1 at 0xa8400000 (entry 0x885025d0).
 * So — unlike the relocatable disk sash — there are NO relocations and NO kuseg
 * TLB mapping: place each section at PA = vaddr & 0x1fffffff, and reuse the Mode
 * K kernel trampoline (jumps a kseg0 entry with argc=0/envp=ARCS environ).
 * See progress_notes/ip55/prom_c3_install_scoping.md.
 */
static void sgi_virtuix_load_ide_ecoff(const char *path, MachineState *machine,
                                       MemoryRegion *system_memory) {
  gsize ide_len = 0;
  uint8_t *ide = NULL;
  GError *err = NULL;
  uint16_t f_magic, f_nscns, a_magic;
  uint32_t entry;
  uint32_t kernel_start = 0xFFFFFFFFu;
  uint32_t kernel_end = 0;
  int n;

  if (!g_file_get_contents(path, (char **)&ide, &ide_len, &err) || !ide) {
    error_report("Mode C IDE: cannot read %s: %s", path,
                 err ? err->message : "unknown error");
    if (err) {
      g_error_free(err);
    }
    return;
  }
  if (ide_len < 76) {
    error_report("Mode C IDE: %s too small for an ECOFF header", path);
    g_free(ide);
    return;
  }
  f_magic = sgi_be16(&ide[0]);
  f_nscns = sgi_be16(&ide[2]);
  a_magic = sgi_be16(&ide[20]);
  entry = sgi_be32(&ide[36]);
  if (f_magic != 0x0160 || a_magic != 0x0107) {
    error_report("Mode C IDE: %s is not a standalone ECOFF "
                 "(f_magic=0x%04x a_magic=0x%04x)", path, f_magic, a_magic);
    g_free(ide);
    return;
  }
  if (f_nscns > 16) {
    error_report("Mode C IDE: %u sections (too many)", f_nscns);
    g_free(ide);
    return;
  }

  for (n = 0; n < f_nscns; n++) {
    const uint8_t *sh = &ide[76 + n * 40];
    char name[9];
    uint32_t vaddr = sgi_be32(&sh[12]);
    uint32_t size = sgi_be32(&sh[16]);
    uint32_t scnptr = sgi_be32(&sh[20]);
    uint32_t phys = vaddr & 0x1fffffff;
    memcpy(name, sh, 8);
    name[8] = '\0';

    if (scnptr >= ide_len || scnptr + size > ide_len) {
      /* .bss (no file data): zero-fill. */
      void *zero = g_malloc0(size);
      rom_add_blob_fixed("ide-bss", zero, size, phys);
      g_free(zero);
    } else {
      rom_add_blob_fixed("ide-sec", ide + scnptr, size, phys);
    }
    if (phys + size > kernel_end) {
      kernel_end = phys + size;
    }
    if (phys < kernel_start) {
      kernel_start = phys;
    }
    qemu_log("Mode C IDE: section %-8s vaddr=0x%08x -> phys 0x%08x (%u B)\n",
             name, vaddr, phys, size);
  }
  g_free(ide);

  qemu_log("Mode C IDE: loaded %s, entry 0x%08x\n", path, entry);
  write_kernel_trampoline(entry);

  /* ARCS firmware stubs (identical Mode K tail) so the mini-kernel finds the
   * SPB/FV/env/memdesc. */
  DeviceState *arcs_dev = qdev_new(TYPE_SGI_ARCS);
  qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
  qdev_prop_set_uint32(arcs_dev, "kernel-start", kernel_start);
  qdev_prop_set_uint32(arcs_dev, "kernel-end", kernel_end);
  if (serial_hd(1)) {
    qdev_prop_set_chr(arcs_dev, "chardev", serial_hd(1));
  }
  sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
  memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                      &SGI_ARCS(arcs_dev)->iomem, 10);
  sgi_arcs_setup_stubs(SGI_ARCS(arcs_dev), &address_space_memory);
  qemu_log("Mode C IDE: firmware ready; handing off to mini-kernel\n");
}

/*
 * Mode C boot (Path C — director decision 2026-07-07): our paravirtual ARCS
 * PROM reads the disk's /unix host-side from the XFS root partition, loads it,
 * and jumps — no guest sash, no -kernel, no borrowed Indy -bios. Reuses the
 * exact Mode K tail (kernel trampoline + ARCS SPB/FV/env/memdesc stubs), so the
 * kernel sees an identical firmware ABI; only the kernel's *source* differs.
 * The XFS reader (hw/mips/sgi_xfs.c) is the generic substrate the future ARCS
 * Open/Read/Seek/Close services (Path A sash) reuse verbatim.
 */
static void sgi_virtuix_mode_c_boot(MachineState *machine,
                                    MemoryRegion *system_memory) {
  DriveInfo *dinfo;
  BlockBackend *blk;
  uint8_t vh[SGI_VH_SECTOR];
  uint32_t part_firstlbn[SGI_VH_NPARTAB] = {0};
  uint32_t part_type[SGI_VH_NPARTAB] = {0};
  int i;

  qemu_log("Virtuix: Mode C (our IP55 PROM) — booting disk /unix host-side\n");

  /* Path I (install mini-kernel from host path): env-gated; skips the disk
   * boot chain entirely. */
  const char *ide_path = getenv("SGI_MODE_C_IDE");
  if (ide_path && *ide_path) {
    sgi_virtuix_load_ide_ecoff(ide_path, machine, system_memory);
    return;
  }

  /* Boot disk is if=scsi,bus=0,unit=1 (the canonical golden convention). */
  dinfo = drive_get(IF_SCSI, 0, 1);
  if (!dinfo || !(blk = blk_by_legacy_dinfo(dinfo))) {
    error_report("Mode C: no boot disk at scsi bus=0 unit=1");
    return;
  }

  if (blk_pread(blk, 0, SGI_VH_SECTOR, vh, 0) < 0 ||
      sgi_be32(&vh[0]) != SGI_VH_MAGIC) {
    error_report("Mode C: bad/absent SGI volume header on boot disk");
    return;
  }

  /* Parse the partition table. */
  for (i = 0; i < SGI_VH_NPARTAB; i++) {
    const uint8_t *pt = &vh[SGI_VH_PARTAB_OFF + i * SGI_VH_PT_ENTSZ];
    part_firstlbn[i] = sgi_be32(pt + 4);
    part_type[i] = sgi_be32(pt + 8);
  }

  /*
   * Path A: locate the volume-header `sash` (the ECOFF the real PROM chain
   * loads + runs). When SGI_MODE_C_PATH_A=1 we actually load it (ECOFF loader)
   * and transfer control via the kuseg-TLB trampoline; without the env gate this
   * stays a cheap header parse, and Mode C (Path C) proceeds to boot /unix
   * host-side. See prom_c1_c2.md + prom_c3_path_a_scoping.md.
   */
  uint32_t sash_lbn = 0, sash_nbytes = 0;
  bool have_sash = false;
  for (i = 0; i < SGI_VH_NVDIR; i++) {
    const uint8_t *vd = &vh[SGI_VH_VOLDIR_OFF + i * SGI_VH_VD_ENTSZ];
    if (memcmp(vd, "sash\0\0\0\0", SGI_VH_VDNAMESIZE) != 0) {
      continue;
    }
    sash_lbn = sgi_be32(vd + 8);
    sash_nbytes = sgi_be32(vd + 12);
    have_sash = true;
    break;
  }

  if (getenv("SGI_MODE_C_PATH_A") && have_sash) {
    uint8_t *sash = g_malloc(sash_nbytes);
    if (blk_pread(blk, (uint64_t)sash_lbn * SGI_VH_SECTOR, sash_nbytes, sash,
                  0) < 0) {
      error_report("Mode C Path A: could not read sash (%u B at lbn %u)",
                   sash_nbytes, sash_lbn);
      g_free(sash);
      return;
    }

    /* ECOFF filehdr (20 B) + aouthdr (56 B), big-endian. */
    uint16_t f_magic = sgi_be16(&sash[0]);
    uint16_t f_nscns = sgi_be16(&sash[2]);
    uint16_t a_magic = sgi_be16(&sash[20]);
    uint32_t tsize = sgi_be32(&sash[24]);
    uint32_t dsize = sgi_be32(&sash[28]);
    uint32_t bsize = sgi_be32(&sash[32]);
    uint32_t entry = sgi_be32(&sash[36]);
    uint32_t text_start = sgi_be32(&sash[40]);
    uint32_t data_start = sgi_be32(&sash[44]);
    uint32_t bss_start = sgi_be32(&sash[48]);
    uint32_t gp_value = sgi_be32(&sash[72]);  /* aouthdr +52 gp_value */
    if (f_magic != 0x0163 || a_magic != 0x0107) {
      error_report("Mode C Path A: unexpected sash header (f_magic=0x%04x "
                   "a_magic=0x%04x)", f_magic, a_magic);
      g_free(sash);
      return;
    }
    /* N_TXTOFF = round_up(FILHSZ + AOUTHSZ + nscns*SCNHSZ, 16). */
    uint32_t txoff = (20 + 56 + (uint32_t)f_nscns * 40 + 15) & ~15u;
    if ((uint64_t)txoff + tsize + dsize > sash_nbytes) {
      error_report("Mode C Path A: sash sections exceed file size");
      g_free(sash);
      return;
    }

    /* Apply ECOFF relocations (the on-disk sash is relocatable). */
    if (sgi_sash_relocate(sash, sash_nbytes) < 0) {
      error_report("Mode C Path A: sash relocation failed");
      g_free(sash);
      return;
    }

    /* kuseg 0x10000000 -> phys 0x09000000 (PA = VA - 0x07000000). */
    uint32_t phys_text = text_start - 0x07000000u;
    uint32_t phys_data = data_start - 0x07000000u;
    uint32_t phys_bss = bss_start - 0x07000000u;
    rom_add_blob_fixed("sash-text", sash + txoff, tsize, phys_text);
    rom_add_blob_fixed("sash-data", sash + txoff + tsize, dsize, phys_data);
    if (bsize > 0) {
      void *zero = g_malloc0(bsize);
      rom_add_blob_fixed("sash-bss", zero, bsize, phys_bss);
      g_free(zero);
    }
    g_free(sash);
    qemu_log("Mode C Path A: loaded sash text=%u@0x%08x data=%u@0x%08x "
             "bss=%u@0x%08x entry=0x%08x gp=0x%08x\n",
             tsize, phys_text, dsize, phys_data, bsize, phys_bss, entry,
             gp_value);

    write_sash_trampoline(entry, gp_value);

    /* ARCS firmware stubs (identical Mode K tail) so sash finds the SPB/FV. */
    DeviceState *arcs_dev = qdev_new(TYPE_SGI_ARCS);
    qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(arcs_dev, "kernel-start", phys_text);
    qdev_prop_set_uint32(arcs_dev, "kernel-end", phys_bss + bsize);
    /* Firmware console: bind to a SECOND -serial if one was provided (the
     * first is the SCC/kernel console); else sash's console falls back to
     * qemu_log/0-byte reads. */
    if (serial_hd(1)) {
        qdev_prop_set_chr(arcs_dev, "chardev", serial_hd(1));
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
    memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                        &SGI_ARCS(arcs_dev)->iomem, 10);
    sgi_arcs_setup_stubs(SGI_ARCS(arcs_dev), &address_space_memory);
    qemu_log("Mode C Path A: firmware ready; handing off to disk sash\n");
    return;
  }

  /* Resolve the boot device + filename from the ARCS environment (the same
   * OSLoadPartition/OSLoadFilename the stock PROM/sash chain consults).
   * SGI_MODE_C_OSLOAD overrides the kernel filename so a staged pv kernel
   * (e.g. /unix.pv) can be booted without replacing the disk's stock /unix. */
  const char *osload_part = "dksc(0,1,0)";  /* matches arcs_env_vars */
  const char *osload_file = getenv("SGI_MODE_C_OSLOAD");
  if (!osload_file || !*osload_file) {
    osload_file = "unix";
  }
  int unit = 1, part = 0;
  sgi_devpath_parse(osload_part, &unit, &part);
  if (part < 0 || part >= SGI_VH_NPARTAB) {
    part = 0;
  }
  qemu_log("Mode C: OSLoadPartition=\"%s\" -> unit %d partition %d "
           "(firstlbn=%u type=%u), OSLoadFilename=\"%s\"\n",
           osload_part, unit, part, part_firstlbn[part], part_type[part],
           osload_file);

  /* Mount the XFS root partition host-side. */
  SGIXfs fs;
  if (sgi_xfs_mount(blk, part_firstlbn[part], &fs) < 0) {
    error_report("Mode C: could not mount XFS at partition %d (firstlbn %u)",
                 part, part_firstlbn[part]);
    return;
  }

  /* Resolve + read the kernel file. */
  char path[64];
  snprintf(path, sizeof(path), "/%s", osload_file);
  SGIXfsInode kino;
  if (sgi_xfs_lookup(&fs, path, &kino) < 0) {
    error_report("Mode C: '%s' not found on the XFS root partition", path);
    return;
  }

  uint64_t ksize = kino.size;
  if (ksize == 0 || ksize > 64 * MiB) {
    error_report("Mode C: implausible kernel size %" PRIu64, ksize);
    sgi_xfs_inode_put(&kino);
    return;
  }
  uint8_t *kbuf = g_malloc(ksize);
  uint64_t got = 0;
  if (sgi_xfs_read(&fs, &kino, 0, ksize, kbuf, &got) < 0 || got != ksize) {
    error_report("Mode C: read of %s failed (%" PRIu64 "/%" PRIu64 ")", path,
                 got, ksize);
    g_free(kbuf);
    sgi_xfs_inode_put(&kino);
    return;
  }
  sgi_xfs_inode_put(&kino);
  qemu_log("Mode C: read %s (%" PRIu64 " bytes) from XFS root\n", path, ksize);

  /* Load the ELF into guest RAM (Execute). */
  uint32_t kentry = 0, klow_phys = 0, khigh_phys = 0;
  int rc = sgi_load_elf32_be(kbuf, ksize, &kentry, &klow_phys, &khigh_phys);
  g_free(kbuf);
  if (rc < 0) {
    error_report("Mode C: %s is not a loadable ELF32-MSB MIPS kernel (rc=%d)",
                 path, rc);
    return;
  }
  qemu_log("Mode C: loaded kernel, entry 0x%08x, low phys 0x%08x, "
           "high phys 0x%08x\n", kentry, klow_phys, khigh_phys);

  /* Mode K tail: trampoline + ARCS firmware stubs (identical firmware ABI). */
  write_kernel_trampoline(kentry);

  DeviceState *arcs_dev = qdev_new(TYPE_SGI_ARCS);
  qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
  qdev_prop_set_uint32(arcs_dev, "kernel-start", klow_phys);
  qdev_prop_set_uint32(arcs_dev, "kernel-end", khigh_phys);
  if (serial_hd(1)) {
      qdev_prop_set_chr(arcs_dev, "chardev", serial_hd(1));
  }
  sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
  memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                      &SGI_ARCS(arcs_dev)->iomem, 10);
  sgi_arcs_setup_stubs(SGI_ARCS(arcs_dev), &address_space_memory);

  qemu_log("Mode C: firmware ready; handing off to disk /unix\n");
}

static void sgi_virtuix_init(MachineState *machine) {
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *prom;
  DeviceState *mc_dev;
  DeviceState *hpc3_dev;
  MIPSCPU *cpu;
  Clock *cpuclk;
  char *filename;
  int bios_size;

  /* sash's Execute() (Path A) is implemented here (needs XFS + MIPS CPU). */
  sgi_arcs_set_execute_cb(sgi_virtuix_execute);

  /* Validate RAM size (Virtuix cap = 2 GiB) */
  if (machine->ram_size > SGI_RAM_MAX) {
    error_report("RAM size more than 2GB is not supported");
    exit(EXIT_FAILURE);
  }

  /*
   * Virtualization-native time: drive CP0 Count/Compare (the source of the HZ
   * scheduling tick, IRQ7) off the host monotonic clock instead of
   * QEMU_CLOCK_VIRTUAL. Under MTTCG at high -smp counts, virtual time advances
   * in bursts (each vCPU thread progresses its clock in chunks), so the tick
   * fires erratically -> the UI stalls for seconds (measured: window-drag lag,
   * with the guest 100% idle, was bursty/stalled HZ-tick delivery). Routing
   * Count to the realtime clock makes the tick smooth. Respects an explicit
   * user override (overwrite=0). See target/mips/system/cp0_timer.c. Authentic
   * Indy keeps the default VIRTUAL clock (sgi_indy.c sets nothing).
   */
  setenv("QEMU_MIPS_COUNT_REALTIME", "1", 0);

  /*
   * Virtualization-native CPU clock. The modeled frequency sets the CP0 Count
   * rate (Count = cpu_clock / CCRes); 66.67 MHz matches the frequency IRIX
   * believes (hinv "66 MHZ IP22"), so cycle->time conversion tracks real time
   * (~1.0x). The CPU still executes flat-out (TCG ignores the modeled clock).
   * Overridable via IP55_CPU_HZ. (Authentic Indy keeps 100 MHz in sgi_indy.c.)
   */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  {
    const char *e = getenv("IP55_CPU_HZ");
    clock_set_hz(cpuclk, e && *e ? strtoull(e, NULL, 0) : 66666666);
  }

  /*
   * Create N CPUs (IP55 SMP). default_cpus=1 keeps uniprocessor boots
   * byte-identical; -smp >1 creates secondaries that start powered off until
   * the SMP controller's BOOT_GO kicks them. Each CPU gets its own CP0 IRQ
   * lines and Count/Compare timer; EBase is auto-set from cpu_index.
   */
  int ncpus = machine->smp.cpus;
  MIPSCPU **cpus = g_new0(MIPSCPU *, ncpus);
  for (int i = 0; i < ncpus; i++) {
    cpus[i] = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpus[i]);
    cpu_mips_clock_init(cpus[i]);
    qemu_register_reset(main_cpu_reset, cpus[i]);
    if (i != 0) {
      CPU(cpus[i])->start_powered_off = true;
    }
  }
  cpu = cpus[0];

  /*
   * Paravirtual SMP controller. Only instantiated for true SMP boots so that
   * uniprocessor boots remain byte-identical. IPIs are delivered on CP0 IP6
   * (env.irq[6]) -- a hardware interrupt line nothing else drives on the
   * emulated Indy -- avoiding the CP0_Cause cross-thread race that a
   * software-interrupt bit would reintroduce (an IP54 lesson).
   */
  /*
   * Always instantiate the sgi-smp block, even at -smp 1. The IP55 kernel reads
   * the CPU_COUNT register (offset 0) unconditionally during early boot
   * (`lui a0,0xbfa8; lw a0,0(a0)` at kernel 0x8822c86c). If the region is
   * unmapped it takes a bus error that loops forever at that PC (no console
   * output — the classic "-smp 1 won't boot" hang). At ncpus==1 the IPI/BOOT_GO
   * paths are simply never exercised; CPU_COUNT reads 1 and CPU_ID reads 0.
   */
  {
    DeviceState *smp = qdev_new(TYPE_SGI_SMP);
    qdev_prop_set_uint32(smp, "num-cpus", ncpus);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(smp), 0, SGI_VIRTUIX_SMP_BASE);

    SGISMPState *smp_state = SGI_SMP(smp);
    for (int i = 0; i < ncpus; i++) {
      smp_state->cpus[i] = CPU(cpus[i]);
      sysbus_connect_irq(SYS_BUS_DEVICE(smp), i, cpus[i]->env.irq[6]);
    }
  }

  /* PROM at 0x1fc00000 */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_rom(prom, NULL, "sgi.prom", SGI_PROM_SIZE, &error_fatal);
  memory_region_add_subregion(system_memory, SGI_PROM_BASE, prom);

  /*
   * Load PROM (skipped for -kernel direct boot, where ARCS stubs provide the
   * firmware interface). Default PROM name is ip55prom.bin; in practice Virtuix
   * boots via -kernel or an explicit -bios.
   */
  if (machine->kernel_filename) {
    filename = NULL;
  } else if (machine->firmware) {
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename) {
      error_report("Could not find firmware '%s'", machine->firmware);
      exit(EXIT_FAILURE);
    }
  } else {
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip55prom.bin");
    if (!filename) {
      warn_report("No firmware specified, use -bios to specify IP55 PROM");
      filename = NULL;
    }
  }

  if (filename) {
    bios_size =
        load_image_targphys(filename, SGI_PROM_BASE, SGI_PROM_SIZE, NULL);
    g_free(filename);
    if (bios_size < 0) {
      error_report("Could not load PROM image");
      exit(EXIT_FAILURE);
    }
  }

  /* Memory Controller (Virtuix variant, with host-RT counter) at 0x1fa00000 */
  mc_dev = qdev_new(TYPE_SGI_MC_VIRTUIX);
  qdev_prop_set_uint32(mc_dev, "ram-size", machine->ram_size);
  object_property_set_link(OBJECT(mc_dev), "ram", OBJECT(machine->ram),
                           &error_fatal);
  object_property_set_link(OBJECT(mc_dev), "system-memory",
                           OBJECT(system_memory), &error_fatal);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(mc_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(mc_dev), 0, SGI_MC_BASE);

  /* HPC3 + IOC2/INT3 (Virtuix variant) at 0x1fb80000 */
  hpc3_dev = qdev_new(TYPE_SGI_HPC3_VIRTUIX);
  qdev_prop_set_chr(hpc3_dev, "chardev", serial_hd(0));
  qdev_prop_set_uint8(hpc3_dev, "board-type", BOARD_IP24);
  {
    SGIHPC3VirtuixState *hpc3 = SGI_HPC3_VIRTUIX(hpc3_dev);
    if (!hpc3->nvram_filename) {
      qdev_prop_set_string(hpc3_dev, "nvram-file", "sgi_virtuix_nvram.bin");
    }
  }
  {
    NICInfo *nd = qemu_find_nic_info(TYPE_SGI_HPC3_VIRTUIX, true, NULL);
    if (nd) {
      qdev_set_nic_properties(hpc3_dev, nd);
    }
  }
  sysbus_realize_and_unref(SYS_BUS_DEVICE(hpc3_dev), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(hpc3_dev), 0, SGI_HPC3_BASE);

  {
    SGIHPC3VirtuixState *hpc3 = SGI_HPC3_VIRTUIX(hpc3_dev);
    scsi_bus_legacy_handle_cmdline(&hpc3->scsi[0]->bus);
  }

  /*
   * Wire HPC3 interrupts to MIPS CPU IRQ lines:
   *   INT3 Local0 (SCSI, ethernet) -> IP2; Local1 (panel, DMA) -> IP3;
   *   PIT Timer 0 -> IP4; PIT Timer 1 -> IP5.
   */
  qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 0, cpu->env.irq[2]);
  qdev_connect_gpio_out_named(hpc3_dev, "cpu-irq", 1, cpu->env.irq[3]);
  qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 0, cpu->env.irq[4]);
  qdev_connect_gpio_out_named(hpc3_dev, "timer-irq", 1, cpu->env.irq[5]);

  /* MC GIO DMA-complete -> HPC3 LIO_GDMA (Local0 bit 4) -> IP2 */
  sysbus_connect_irq(SYS_BUS_DEVICE(mc_dev), 0,
                     qdev_get_gpio_in_named(hpc3_dev, "mc-dma-irq", 0));

  /*
   * Newport graphics at the GIO graphics slot (0x1f000000); REX3 registers at
   * offset 0x0f0000. Empty-slot stubs cover the rest of the slot.
   */
  {
    DeviceState *newport_dev;

    create_gio_empty_slot(system_memory, "gio-gfx-low", SGI_GIO_GFX_BASE,
                          REX3_REG_OFFSET);

    /* Paravirtual GL accelerator: must realize BEFORE Newport because Newport
     * registers as a desktop renderer with glaccel during its own realize. */
    {
        DeviceState *glaccel_dev = qdev_new(TYPE_SGI_GLACCEL);
        qdev_set_id(glaccel_dev, g_strdup("glaccel"), &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(glaccel_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(glaccel_dev), 0, 0x1fa20000ULL);
    }

    newport_dev = qdev_new(TYPE_SGI_NEWPORT_VIRTUIX);
    object_property_add_child(OBJECT(machine), "newport", OBJECT(newport_dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(newport_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(newport_dev), 0,
                    SGI_GIO_GFX_BASE + REX3_REG_OFFSET);

    /* Newport VRINT -> HPC3 retrace interrupt */
    sysbus_connect_irq(SYS_BUS_DEVICE(newport_dev), 0,
                       qdev_get_gpio_in_named(hpc3_dev, "gio-retrace", 0));

    create_gio_empty_slot(system_memory, "gio-gfx-high",
                          SGI_GIO_GFX_BASE + REX3_REG_OFFSET + REX3_REG_SIZE,
                          4 * MiB - REX3_REG_OFFSET - REX3_REG_SIZE);
  }

  create_gio_empty_slot(system_memory, "gio-exp0", SGI_GIO_EXP0_BASE, 2 * MiB);
  create_gio_empty_slot(system_memory, "gio-exp1", SGI_GIO_EXP1_BASE, 4 * MiB);

  /* Paravirtual host<->guest channel: ring+doorbell device for the resident
   * guest agent, replacing serial-scraping + gdbstub-halt + slirp dependence. */
  {
      DeviceState *pvchan_dev = qdev_new(TYPE_SGI_PVCHAN);
      sysbus_realize_and_unref(SYS_BUS_DEVICE(pvchan_dev), &error_fatal);
      sysbus_mmio_map(SYS_BUS_DEVICE(pvchan_dev), 0, SGI_VIRTUIX_PVCHAN_BASE);
      /* pvchan IRQ → CPU IP1 (unused on virtuix; PROM and IRIX route IP1 only
       * for GIO slot 2, which virtuix doesn't populate). */
      sysbus_connect_irq(SYS_BUS_DEVICE(pvchan_dev), 0, cpu->env.irq[1]);
  }

  /* Paravirtual audio: ring-buffer PCM device drained to the host -audiodev.
   * Always instantiated so the guest pvaudio driver can attach; produces sound
   * only when `-global sgi-pvaudio.audiodev=aud0` explicitly wires a backend
   * (no-audiodev boots stay silent — see sgi_pvaudio_realize).  IRQ kept masked
   * driver-side (userland throttles via BUF_TAIL polling); wire to IP1 like
   * pvchan for hygiene. */
  {
      DeviceState *pvaudio_dev = qdev_new(TYPE_SGI_PVAUDIO);
      sysbus_realize_and_unref(SYS_BUS_DEVICE(pvaudio_dev), &error_fatal);
      sysbus_mmio_map(SYS_BUS_DEVICE(pvaudio_dev), 0, SGI_VIRTUIX_PVAUDIO_BASE);
      sysbus_connect_irq(SYS_BUS_DEVICE(pvaudio_dev), 0, cpu->env.irq[1]);
  }

  /*
   * Memory probe areas. The MC dynamically maps RAM aliases (priority 1) over
   * these unimplemented stubs (priority -1000); unmapped banks show through as 0.
   */
  create_unimplemented_device("low-mem-probe", SGI_RAM_LOW_BASE, 256 * MiB);
  create_unimplemented_device("high-mem-probe", SGI_RAM_HIGH_BASE, 256 * MiB);
  create_unimplemented_device("zero-mem-probe", 0x00000000, 512 * KiB);

  /*
   * BL-39: SEG1 high RAM (0x20000000) is now mapped by the MC as bank 2
   * (sgi_mc_virtuix_reset seeds MEMCFG so the IP22 kernel's szmem() counts it),
   * so there is NO separate native high-ram alias here — that would collide
   * with the MC's bank-2 subregion at 0x20000000.  The MC owns all RAM banks.
   */

  create_unimplemented_device("extended-mem-probe0", 0x18000000, 0x07000000);
  create_unimplemented_device("extended-mem-probe1", 0x30000000, 0x50000000);

  /*
   * Direct kernel boot via -kernel: load the IRIX ELF and set up ARCS firmware
   * stubs (GetMemoryDescriptor, GetEnvironmentVariable, Write, ...).
   */
  if (machine->kernel_filename) {
    uint64_t kernel_entry;
    uint64_t kernel_low = 0xFFFFFFFFULL;
    uint64_t kernel_high = 0;
    long kernel_size;
    DeviceState *arcs_dev;
    SGIARCSState *arcs;

    kernel_size = load_elf(machine->kernel_filename, NULL,
                           cpu_mips_kseg0_to_phys, NULL, &kernel_entry,
                           &kernel_low, &kernel_high, NULL, ELFDATA2MSB,
                           EM_MIPS, 1, 0);
    if (kernel_size < 0) {
      error_report("could not load kernel '%s': %s", machine->kernel_filename,
                   load_elf_strerror(kernel_size));
      exit(1);
    }

    uint64_t kernel_low_phys = kernel_low & 0x1FFFFFFF;
    uint64_t kernel_high_phys = kernel_high & 0x1FFFFFFF;

    qemu_log("Virtuix: Loaded kernel '%s' (%ld bytes)\n",
             machine->kernel_filename, kernel_size);
    qemu_log("Virtuix: Kernel entry: 0x%016" PRIx64 ", lowest addr: 0x%016"
             PRIx64 " (phys 0x%08" PRIx64 "), highest addr: 0x%016"
             PRIx64 " (phys 0x%08" PRIx64 ")\n",
             kernel_entry, kernel_low, kernel_low_phys,
             kernel_high, kernel_high_phys);

    write_kernel_trampoline((uint32_t)kernel_entry);

    arcs_dev = qdev_new(TYPE_SGI_ARCS);
    qdev_prop_set_uint32(arcs_dev, "ram-size", machine->ram_size);
    qdev_prop_set_uint32(arcs_dev, "kernel-start",
                         (uint32_t)kernel_low_phys);
    qdev_prop_set_uint32(arcs_dev, "kernel-end", (uint32_t)kernel_high_phys);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(arcs_dev), &error_fatal);
    memory_region_add_subregion_overlap(system_memory, SGI_ARCS_MMIO_BASE,
                                        &SGI_ARCS(arcs_dev)->iomem, 10);

    arcs = SGI_ARCS(arcs_dev);
    sgi_arcs_setup_stubs(arcs, &address_space_memory);
  } else if (getenv("SGI_MODE_C")) {
    /*
     * Mode C: our own IP55 PROM. Env-gated so the Mode K path above stays
     * byte-identical. Reads the disk's /unix host-side from the XFS root
     * partition, loads + jumps to it (Path C). See prom_c1_c2.md.
     */
    sgi_virtuix_mode_c_boot(machine, system_memory);
  }
}

static void sgi_virtuix_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "Virtuix (IP55) - SGI Indy-derived virtualization-native SMP";
  mc->init = sgi_virtuix_init;
  mc->block_default_type = IF_SCSI;
  mc->default_ram_size = 256 * MiB;
  mc->default_ram_id = "sgi.ram";
  /* R5000 = MIPS IV so IP55 runs the full (mips4) nekoware catalog. Same
   * R4x00/UP-era lineage + software-managed caches as the R4600 we ran SMP on,
   * and the IP22/IP55 kernel already compiles the R5000 errata WARs; the R5000
   * code paths are FP/cache, not SMP coherency. (R10000/R12000 would trip
   * IS_R10000() into L2/speculation paths we don't model.) */
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R5000");
  mc->default_cpus = 1;
  mc->max_cpus = 32; /* IP55 SMP: paravirtual IPI via sgi-smp on env.irq[6] */
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
}

static const TypeInfo sgi_virtuix_type = {
    .name = MACHINE_TYPE_NAME("virtuix"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_virtuix_class_init,
};

static void sgi_virtuix_machine_register(void) {
  type_register_static(&sgi_virtuix_type);
}

type_init(sgi_virtuix_machine_register)
