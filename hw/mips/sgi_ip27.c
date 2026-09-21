/*
 * SGI IP27 (Origin 200 / Origin 2000 / Onyx2, "SN0") — headless hub/NUMA model.
 *
 * IP27 is radically unlike the IP22/IP24/IP32 lineage already modelled here:
 * each node pairs one or two R10000/R12000 CPUs with local memory and a Hub
 * (Bedrock) ASIC; nodes are linked by CrayLink/NUMAlink and the hub owns global
 * addressing, routing and coherence.  Booting goes through the SN0-container
 * ARCS PROM (PROM_library/bins/cpu/ip27/ip27prom.img), which discovers nodes/IO
 * through the hub registers and klconfig.
 *
 * Scope: headless.  No graphics, no audio.  This file models the SN0 address
 * spaces and one (or more, later) node; the hub register file lives in
 * hw/misc/sgi_hub.c.
 *
 * Address facts are from Linux arch/mips/include/asm/sn/sn0/addrs.h and
 * arch/mips/include/asm/mach-ip27/spaces.h; PROM images are SN0 "JFKSWCSM"
 * containers (see analysis_tools/prom/sn_container.py).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_baseio.h"
#include "hw/misc/sgi_hub.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/*
 * SN0 address spaces (XKPHYS).  These are direct-mapped segments; the space
 * is selected by address bits [58:56], which is why the CPU must form physical
 * addresses with a >= 59-bit mask (see the PAMask override in the init).
 */
#define IP27_HSPEC_BASE 0x9000000000000000ULL
#define IP27_IO_BASE 0x9200000000000000ULL
#define IP27_MSPEC_BASE 0x9400000000000000ULL
#define IP27_UNCAC_BASE 0x9600000000000000ULL
#define IP27_CAC_BASE 0xa800000000000000ULL

/* Preserve bits [58:0]; bits [58:56] carry the address-space selector. */
/*
 * Physical-address mask for XKPHYS.  Must retain the XKPHYS region
 * discriminator (bit 60), otherwise HSPEC (0x9...) and CAC (0xa8...) fold to
 * the same physical address and, e.g., the LBOOT flash at HSPEC+0x10000000
 * collides with node RAM at CAC+0x10000000.  With bit 60 retained: CAC -> 0
 * (node RAM), and HSPEC/IO/MSPEC/UNCAC map to distinct phys ranges.  KSEG
 * (reset vector) uses a fixed 0x1fffffff mask and is unaffected.
 */
#define IP27_PAMASK 0x17FFFFFFFFFFFFFFULL

/* M-mode: 4 GB per node, nasid at bits [39:32], 16 MB small window. */
#define IP27_NODE_SIZE 0x100000000ULL
#define IP27_SWIN_SIZE 0x1000000ULL
#define IP27_HUB_WIDGET 1ULL

/* PROM window: file code (0x1fc00000) is mapped here. */
#define IP27_PROM_BASE 0x1fc00000ULL
#define IP27_PROM_SIZE (1 * MiB)

/*
 * LBOOT window: the local hub's boot flash (and other directory-bus devices).
 * LBOOT_BASE = HSPEC_BASE + 0x10000000; direct-mapped XKPHYS so the masked
 * physical base is 0x10000000.  The IP27 flash holds the same SN0 container
 * image (header + code), so the PROM reads its identity from here.
 */
#define IP27_LBOOT_PHYS (IP27_HSPEC_BASE + 0x10000000ULL)
#define IP27_FLASH_SIZE (1 * MiB)

#define IP27_MAX_CPUS 2

/* MD_MEMORY_CONFIG bank size codes (hubmd.h MD_SIZE_*). */
#define IP27_MD_SIZE_8MB 1
#define IP27_MD_SIZE_32MB 3
#define IP27_MD_SIZE_1GB 8

static uint64_t ip27_phys(uint64_t va) { return va & IP27_PAMASK; }

/* Physical base of node `nasid`'s widget-`wid` small window. */
static uint64_t ip27_swin_phys(uint32_t nasid, uint32_t wid) {
  return ip27_phys(IP27_IO_BASE | ((uint64_t)nasid << 32) |
                   ((uint64_t)wid << 24));
}

void sgi_ip27_local_reset(void);

static void main_cpu_reset(void *opaque) {
  MIPSCPU *cpu = opaque;
  cpu_reset(CPU(cpu));
  /*
   * The SN0 PROM is entered from the flash sloader, which leaves gp set to
   * the PROM's data pointer and has already mapped the PROM's 1 MB XKSEG
   * image (vaddr 0xc00000001fc00000 -> phys 0x1fc00000, ASID 1, uncached,
   * read-only) via a wired TLB entry.  Model that handoff so early
   * gp-relative PROM accesses resolve; the PROM later re-wires the same entry
   * itself.
   */
  cpu->env.active_tc.gpr[28] = 0xc00000001fce4fd0ULL;
  cpu->env.active_tc.gpr[29] = 0xa800000000100000ULL; /* PROMDATA stack */
  /* Default console-device descriptor ($f3), as left by the sloader. */
  cpu->env.active_fpu.fpr[3].d = 0xc00000001fc74ce0ULL;
  /*
   * The PROM runs in XKSEG (KSEG3), which is TLB-mapped, and changes ASID as
   * it initialises; the bootstrap mapping must be global (EntryLo G bit 0)
   * so it survives those ASID changes.  flags: V(0x2) | G(0x1) | C=2(0x10).
   */
  mips_cpu_install_mapping(cpu, 0xc00000001fc00000ULL, 0x1fc00000ULL,
                           0x001fe000, 1, 0x13);
}

/*
 * CPU-local reset requested via the hub's NI_PORT_RESET (NPR_LOCALRESET):
 * re-run each CPU from the PROM reset vector with the sloader boot state,
 * leaving the flash/IP27log and the console chardev intact.
 */
void sgi_ip27_local_reset(void) {
  CPUState *c;

  CPU_FOREACH(c) {
    main_cpu_reset(MIPS_CPU(c));
  }
}

/*
 * Encode an amount of RAM into MD_MEMORY_CONFIG bank size codes (8 banks).
 *
 * The hub reports one 3-bit size code per bank; the PROM/kernel derive total
 * memory from these.  Decompose into the largest power-of-two banks, in
 * descending address order as the hardware would.
 */
static uint64_t sgi_ip27_mem_config(uint64_t ram_size) {
  uint64_t cfg = 0;
  int bank = 0;

  /*
   * MD_BANK_SHFT is 29 (512 MB per bank slot).  Pack the node as 128 MB DIMMs
   * into successive bank slots (a 256 MB node is bank0+bank1, code 5 each):
   * each bank's size_back_door() then settles at 128 MB, so memory_init_all()
   * never takes its 256 MB-DIMM "hole" path.  mdir_config() overwrites this
   * register from that probe anyway.
   */
  for (uint64_t sz = 0x8000000ULL; sz >= 0x2000000ULL && bank < 8; sz >>= 1) {
    while (ram_size >= sz && bank < 8) {
      int code = 0;
      uint64_t t = sz >> 22; /* MB / 4: MD_SIZE_MBYTES(code) = 4 << code MB */
      while (t > 1) {
        t >>= 1;
        code++;
      }
      cfg |= (uint64_t)code << (bank * 3);
      ram_size -= sz;
      bank++;
    }
  }
  return cfg;
}

/* Read a big-endian 64-bit field from a buffer. */
static uint64_t sgi_ip27_be64(const uint8_t *p) {
  return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
         ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
         ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
         ((uint64_t)p[6] << 8) | (uint64_t)p[7];
}

/*
 * LBOOT flash backing store.  On real hardware the boot flash code region is
 * read-only and the PROM's log/env/NIC storage lives elsewhere; log writes
 * that hit the code region so we can catch anything corrupting the PROM
 * self-checksum.
 */
static uint8_t ip27_flash_mem[IP27_FLASH_SIZE];
static uint64_t ip27_flash_protect;

/* AMD/Fujitsu flash command state (see libkl/ml/fprom.c do_probe/do_write). */
static int ip27_flash_autoselect;
static int ip27_flash_unlock;     /* 1 = AA@0x5555 seen, 2 = +55@0x2AAA */
static int ip27_flash_program;    /* next write stores data */
static int ip27_flash_erase;      /* 0x80 seen, awaiting 0x10 (chip) / 0x30 (sector) */

static uint8_t ip27_flash_byte(uint64_t off) {
  if (ip27_flash_autoselect) {
    /* AMD29F080-style: manufacturer 0x01, device 0xd5 (accepted pair). */
    switch (off) {
    case 0:
      return 0x01;
    case 1:
      return 0xd5;
    case 2:
      return 0x01;
    case 3:
      return 0xd5;
    default:
      return 0xff;
    }
  }
  if (off < IP27_FLASH_SIZE) {
    return ip27_flash_mem[off];
  }
  return 0xff;
}

static uint64_t ip27_flash_read(void *opaque, hwaddr off, unsigned size) {
  uint64_t v = 0;
  unsigned i;
  for (i = 0; i < size; i++) {
    v = (v << 8) | ip27_flash_byte(off + i);
  }
  return v;
}

static void ip27_flash_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size) {
  /*
   * AMD/Fujitsu command protocol.  Commands are 64-bit stores at
   * base + cmd_addr*8, so the flash command address is off/8 and the command
   * byte is val & 0xff.  Command words are not array data (this is what keeps
   * the PROM code region intact / the self-checksum valid).
   */
  uint64_t faddr = off / 8;
  uint8_t b = val & 0xff;

  if (faddr == 0 && b == 0xf0) {         /* reset to read mode */
    ip27_flash_autoselect = 0;
    ip27_flash_unlock = 0;
    ip27_flash_program = 0;
    ip27_flash_erase = 0;
    return;
  }
  if (faddr == 0x5555 && b == 0xaa) {
    ip27_flash_unlock = 1;
    return;
  }
  if (faddr == 0x2aaa && b == 0x55 && ip27_flash_unlock == 1) {
    ip27_flash_unlock = 2;
    return;
  }
  if (faddr == 0x5555 && ip27_flash_unlock == 2 && b == 0x90) {
    ip27_flash_autoselect = 1;
    ip27_flash_unlock = 0;
    return;
  }
  if (faddr == 0x5555 && ip27_flash_unlock == 2 && b == 0xa0) {
    ip27_flash_program = 1;
    ip27_flash_unlock = 0;
    return;
  }
  if (faddr == 0x5555 && ip27_flash_unlock == 2 && b == 0x80) {
    ip27_flash_erase = 1;
    ip27_flash_unlock = 0;
    return;
  }
  if (ip27_flash_erase) {
    /* Chip erase (0x10@0x5555) or sector erase (0x30@addr): set to 0xff. */
    if (faddr == 0x5555 && b == 0x10) {
      ip27_flash_erase = 0;
      memset(ip27_flash_mem, 0xff, sizeof(ip27_flash_mem));
    } else if (b == 0x30) {
      uint64_t sec = (off / 8) & ~(uint64_t)0xffff; /* 64 KiB sector */
      ip27_flash_erase = 0;
      if (sec + 0x10000 <= sizeof(ip27_flash_mem)) {
        memset(ip27_flash_mem + sec, 0xff, 0x10000);
      }
    }
    return;
  }
  if (ip27_flash_program) {
    /* Program: the hub flash write address is the offset * 8, and only the
     * LSByte is used.  Flash can only clear bits, so AND the data in. */
    uint64_t fidx = off / 8;
    if (fidx < IP27_FLASH_SIZE) {
      ip27_flash_mem[fidx] &= (val & 0xff);
    }
    ip27_flash_program = 0;
    return;
  }
  ip27_flash_unlock = 0;
}

static const MemoryRegionOps ip27_flash_ops = {
  .read = ip27_flash_read,
  .write = ip27_flash_write,
  .endianness = DEVICE_BIG_ENDIAN,
  .valid = { .min_access_size = 1, .max_access_size = 8 },
  .impl = { .min_access_size = 1, .max_access_size = 8 },
};

/*
 * Load an SN0-container PROM (magic "JFKSWCSM" at 0x40).  The header carries
 * the load address (0xA0), code offset (0x98) and code size (0xB0); the code
 * is copied to the IP27 PROM window.  The image is never modified.
 */
static void sgi_ip27_load_prom(const char *filename, MemoryRegion *prom,
                               MemoryRegion *flash) {
  g_autoptr(GError) err = NULL;
  gsize len = 0;
  gchar *data = NULL;
  uint64_t load_addr, code_off, code_size, load_phys;
  uint8_t *dst = memory_region_get_ram_ptr(prom);
  uint8_t *flash_dst = ip27_flash_mem;

  if (!g_file_get_contents(filename, &data, &len, &err)) {
    error_report("sgi-ip27: could not read PROM '%s': %s", filename,
                 err->message);
    exit(EXIT_FAILURE);
  }

  if (len < 0x100) {
    error_report("sgi-ip27: PROM '%s' too small (%zu bytes)", filename, len);
    exit(EXIT_FAILURE);
  }
  if (memcmp(data + 0x40, "JFKSWCSM", 8) != 0) {
    error_report("sgi-ip27: PROM '%s' is not an SN0 (JFKSWCSM) container",
                 filename);
    exit(EXIT_FAILURE);
  }

  load_addr = sgi_ip27_be64((const uint8_t *)data + 0xA0);
  code_off = sgi_ip27_be64((const uint8_t *)data + 0x98);
  code_size = sgi_ip27_be64((const uint8_t *)data + 0xB0);
  load_phys = ip27_phys(load_addr);

  if (code_off + code_size > (uint64_t)len || code_size > IP27_PROM_SIZE) {
    error_report("sgi-ip27: PROM '%s' code range out of bounds "
                 "(off=0x%" PRIx64 " size=0x%" PRIx64 ", file=%zu)",
                 filename, code_off, code_size, len);
    exit(EXIT_FAILURE);
  }

  memcpy(dst, data + code_off, code_size);

  /*
   * The boot flash / LBOOT window (HSPEC+0x10000000) presents the PROM code
   * at offset 0: the PROM reads its checksum source at LBOOT+0 for code_size
   * bytes, so the SN0 container header must not be in the window.
   */
  /* Blank NOR flash reads 0xff, not zero. */
  memset(flash_dst, 0xff, IP27_FLASH_SIZE);
  memcpy(flash_dst, data + code_off, MIN((gsize)IP27_FLASH_SIZE, code_size));
  ip27_flash_protect = MIN((uint64_t)IP27_FLASH_SIZE, code_size);

  /*
   * Synthesize the flash-resident IP27CONFIG record.  It is embedded in the
   * running image (start.s XLEAF(ip27config)) at offset 0x60 and is zero in
   * the built image; the firmware reads it (magic + freq_cpu/freq_hub/
   * freq_rtc) and uses check_sum_adj to keep the image checksum 0 after the
   * overlay.  ip27config_t offsets: time_const@0, r10k_mode@4, magic@8,
   * freq_cpu@16, freq_hub@24, freq_rtc@32, ecc_enable@40, fprom_cyc@44,
   * mach_type@48, check_sum_adj@52 (all from base 0x60).
   */
#define IP27_CONFIG_MAGIC    0x69703237636f6e66ULL /* CONFIG_MAGIC */
#define IP27_CONFIG_FREQ_CPU 200000000ULL
#define IP27_CONFIG_FREQ_HUB 100000000ULL
#define IP27_CONFIG_FREQ_RTC 1250ULL
  {
    uint8_t *c = flash_dst + 0x60;
    uint8_t *adj = flash_dst + 0x60 + 0x34; /* check_sum_adj */
    uint8_t old_adj = *adj;
    uint8_t old_sum = 0, new_sum = 0;
    uint64_t d;
    int i;

    for (i = 0; i < 0x34; i++) {
      old_sum += c[i];
    }
    d = IP27_CONFIG_MAGIC;
    for (i = 0; i < 8; i++) {
      c[0x08 + i] = (d >> ((7 - i) * 8)) & 0xff;
    }
    d = IP27_CONFIG_FREQ_CPU;
    for (i = 0; i < 8; i++) {
      c[0x10 + i] = (d >> ((7 - i) * 8)) & 0xff;
    }
    d = IP27_CONFIG_FREQ_HUB;
    for (i = 0; i < 8; i++) {
      c[0x18 + i] = (d >> ((7 - i) * 8)) & 0xff;
    }
    d = IP27_CONFIG_FREQ_RTC;
    for (i = 0; i < 8; i++) {
      c[0x20 + i] = (d >> ((7 - i) * 8)) & 0xff;
    }
    for (i = 0; i < 0x34; i++) {
      new_sum += c[i];
    }
    /* Keep the image's byte sum unchanged: adj += old_sum - new_sum. */
    *adj = (uint8_t)(old_adj + old_sum - new_sum);
  }

  /*
   * Pre-initialise the PROM log (factory state) in the top two 64 KiB
   * sectors.  The PROM/BASEIO monitor refuses to assign a module id until a
   * valid log exists, and its own initlog path is not run automatically.
   * Format (sys/SN/promlog.h, IP27prom/Promlog/promlog.txt): header at
   * offset 0 of the active sector 14 -- magic 0x504c4f47 ("PLOG") at 0x10,
   * version at 0x14, sequence at 0x18; entries from 0x100, the first being
   * the end-of-log marker (status=valid, type=END=3 => 0xe0).
   */
  {
    uint8_t *lg = flash_dst + 14 * 0x10000;
    lg[0x10] = 0x50; lg[0x11] = 0x4c; lg[0x12] = 0x4f; lg[0x13] = 0x47;
    lg[0x14] = 0; lg[0x15] = 0; lg[0x16] = 0; lg[0x17] = 1;
    lg[0x18] = 0; lg[0x19] = 0; lg[0x1a] = 0; lg[0x1b] = 1;
    /* Entry area left 0xff (erased); promlog appends into erased space. */
  }

  qemu_log_mask(LOG_GUEST_ERROR,
                "sgi-ip27: loaded SN0 PROM '%s': load=0x%" PRIx64
                " -> phys 0x%" PRIx64 ", code off=0x%" PRIx64
                " size=0x%" PRIx64 "\n",
                filename, load_addr, load_phys, code_off, code_size);
}

/*
 * Hub back-door (BDDIR/BDPRT/BDECC) storage.  Real hardware only has
 * back-door storage where memory is populated, so the PROM's bank-size probe
 * (size_back_door -> bd_type/bd_alias) sees "unpopulated" beyond a real bank;
 * that is what makes it size bank 0 as 256 MB and banks 1..7 as EMPTY.
 *
 * Per sys/SN/addrs.h (32-bit node): BDPRT/BDDIR live at
 * HSPEC + NODE_ADDRSPACE_SIZE*3/4 (= HSPEC+0xC0000000), and a physical
 * address `pa` maps to ((pa>>2) & BDDIR_UPPER_MASK) with
 * BDDIR_UPPER_MASK = 0xfffff<<10.  So decode the access back to `pa` and only
 * round-trip where pa is within installed RAM.
 */
#define IP27_BDOOR_PHYS 0x80000000ULL      /* HSPEC + 0x80000000 (BDECC) */
#define IP27_BDDIR_PHYS 0xC0000000ULL      /* BDPRT/BDDIR */
#define IP27_BDDIR_WINSZ 0x40000000ULL     /* 1 GiB: full per-node BDDIR address
                                              range.  BDDIR_UPPER_MASK =
                                              0xfffff<<10 over pa>>2 spans all
                                              4 GiB of pa, so bank1 (pa=512 MiB)
                                              encodes dirraw=0x8000000 which the
                                              old 128 MiB window excluded. */
#define IP27_BDDIR_STORE 0x4000000ULL      /* storage (aliased within a bank) */
#define IP27_BDDIR_UPPER_MASK (0xfffffULL << 10)

static uint8_t *ip27_bdoor_dir;
static uint8_t *ip27_bdecc_dir;
static uint64_t ip27_bdoor_bank0_size;
static uint64_t ip27_bdoor_num_banks;

/*
 * Select the back-door storage for an access and its byte index, or NULL if
 * the location is unpopulated.
 *
 * The back door is three distinct address spaces (sys/SN/addrs.h):
 *   - BDECC  at HSPEC + NODE_ADDRSPACE_SIZE/2 (our BDOOR base, offset 0);
 *            BDECC_ENTRY = pa>>2 & BDECC_UPPER_MASK | pa>>3 & 3
 *   - BDDIR  at HSPEC + NODE_ADDRSPACE_SIZE*3/4, is-dir bit 0x200 set,
 *            LO/HI entries 8 bytes apart
 *   - BDPRT  same base, is-dir bit clear, region in bits [4:3]
 * They must not share storage: the memory test's ECC/enable state lives in
 * BDECC, and folding it into the directory array makes the test miscompare.
 *
 * Within each space the entry index is a function of pa with pa>>2 in
 * BDDIR_UPPER_MASK, i.e. idx == pa>>2 (addrs.h BDPRT_ENTRY/BDPRT_TO_MEM).
 * Only bank 0's real RAM extent is populated, so size_back_door() stops at the
 * true size instead of sizing bank 0 to the whole 512 MB slot.
 */
static uint8_t *ip27_bdoor_sel(hwaddr off, uint64_t *idx) {
  const uint64_t dirbase = IP27_BDDIR_PHYS - IP27_BDOOR_PHYS; /* BDDIR/BDPRT */
  uint64_t mask = (ip27_bdoor_bank0_size >> 2) - 1;

  if (off >= dirbase && off < dirbase + IP27_BDDIR_WINSZ) {
    /*
     * The directory is per-DIMM (per bank slot), so storage is keyed by the
     * bank (pa bits [31:29], i.e. 512 MB slots) and the address within the
     * bank folds with period = the DIMM's directory capacity.  Unpopulated
     * bank slots have no directory and read back as absent, so
     * size_back_door() reports them EMPTY rather than aliasing bank0.
     */
    uint64_t dirraw = off - dirbase;
    uint64_t bank = dirraw / 0x8000000ULL;      /* 512 MB slot >> 2 */
    uint64_t within = dirraw % 0x8000000ULL;
    uint64_t period = ip27_bdoor_bank0_size >> 2;

    if (bank >= ip27_bdoor_num_banks || period == 0) {
      return NULL;
    }
    *idx = bank * period + (within % period);
    return ip27_bdoor_dir;
  }
  if (off < IP27_BDDIR_STORE) {
    *idx = off & mask;
    return ip27_bdecc_dir;
  }
  return NULL;
}

static uint64_t ip27_bdoor_read(void *opaque, hwaddr off, unsigned size) {
  uint8_t *arr;
  uint64_t idx, v = 0;
  unsigned i;

  arr = ip27_bdoor_sel(off, &idx);
  if (arr) {
    for (i = 0; i < size; i++) {
      v = (v << 8) | arr[idx + i];
    }
  }
  return v;
}

static void ip27_bdoor_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size) {
  uint8_t *arr;
  uint64_t idx;
  unsigned i;

  arr = ip27_bdoor_sel(off, &idx);
  if (arr) {
    for (i = 0; i < size; i++) {
      arr[idx + size - 1 - i] = val & 0xff;
      val >>= 8;
    }
  }
}

static const MemoryRegionOps ip27_bdoor_ops = {
  .read = ip27_bdoor_read,
  .write = ip27_bdoor_write,
  .endianness = DEVICE_BIG_ENDIAN,
  .valid = { .min_access_size = 1, .max_access_size = 8 },
  .impl = { .min_access_size = 1, .max_access_size = 8 },
};

#define IP27_BANK_SIZE 0x20000000ULL /* MD_BANK_SHFT=29: 512 MB per bank slot */

/*
 * Map node RAM into an XKPHYS space as one DIMM per 512 MB bank slot.  Unlike
 * a flat alias, bank b's DIMM lives at slot b (b << 29) so the PROM's memory
 * test reaches bank1 at (space + 512 MB) rather than folding it into bank0.
 */
static void ip27_add_ram_banks(MemoryRegion *sysmem, uint64_t space_base,
                               MemoryRegion *ram, uint64_t ram_size,
                               uint64_t banksz, const char *name) {
  MemoryRegion *c = g_new(MemoryRegion, 1);
  uint64_t nbank = ram_size / banksz;
  uint64_t b;

  memory_region_init(c, NULL, name, nbank * IP27_BANK_SIZE);
  for (b = 0; b < nbank; b++) {
    MemoryRegion *a = g_new(MemoryRegion, 1);
    memory_region_init_alias(a, NULL, name, ram, b * banksz, banksz);
    memory_region_add_subregion(c, b << 29, a);
  }
  memory_region_add_subregion(sysmem, ip27_phys(space_base), c);
}

static void sgi_ip27_init(MachineState *machine) {
  Clock *cpuclk;
  MemoryRegion *prom;
  MemoryRegion *flash;
  MemoryRegion *system_memory = get_system_memory();
  MemoryRegion *ram = machine->ram;
  DeviceState *hub;
  DeviceState *baseio;
  SGIHubState *hub_state;
  int ncpus = machine->smp.cpus;
  int i;

  if (ncpus < 1) {
    ncpus = 1;
  }
  if (ncpus > IP27_MAX_CPUS) {
    ncpus = IP27_MAX_CPUS;
  }
  machine->smp.cpus = ncpus;

  /* CPU clock: R10000-era.  Overridable for bring-up. */
  cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
  {
    const char *e = getenv("IP27_CPU_HZ");
    clock_set_hz(cpuclk, e && *e ? strtoull(e, NULL, 0) : 200000000ULL);
  }

  /*
   * Create the node's CPUs.  IP27 relies on XKPHYS bits [58:56] to select its
   * address spaces, so each CPU gets the machine-scoped wide PAMask override
   * (zero on every other machine, which therefore keeps the default PAMask).
   */
  for (i = 0; i < ncpus; i++) {
    CPUMIPSState *env;
    MIPSCPU *c = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);

    env = &c->env;
    env->PAMask_override = IP27_PAMASK;
    env->PAMask = IP27_PAMASK;
    /* Model a cold reset (ErrorEPC clear) for the PROM's reset dispatch. */
    env->cold_erre_clear = true;

    cpu_mips_irq_init_cpu(c);
    cpu_mips_clock_init(c);
    qemu_register_reset(main_cpu_reset, c);
    if (i != 0) {
      CPU(c)->start_powered_off = true;
    }
  }

  /* Node-local memory at physical 0, aliased uncached (UNCAC/MSPEC spaces). */
  {
    uint64_t banksz = MIN(machine->ram_size, (uint64_t)0x8000000);
    memory_region_add_subregion(system_memory, 0, ram);
    ip27_add_ram_banks(system_memory, IP27_UNCAC_BASE, ram,
                       machine->ram_size, banksz, "sgi-ip27.ram.uncac");
    ip27_add_ram_banks(system_memory, IP27_MSPEC_BASE, ram,
                       machine->ram_size, banksz, "sgi-ip27.ram.mspec");
    ip27_add_ram_banks(system_memory, IP27_CAC_BASE, ram,
                       machine->ram_size, banksz, "sgi-ip27.ram.cac");
    /*
     * HSPEC is another access-mode alias of node memory: TO_HSPEC(x) =
     * HSPEC_BASE | (x & TO_PHYS_MASK), so HSPEC+off reads/writes local RAM at
     * off (addrs.h).  The LBOOT flash and the bdoor directory are added later,
     * so they overlay this alias.  Without it the kernel's fill of its BSS/
     * heap via HSPEC (e.g. 0x9000000001bdf818) hits unmapped space -> DBE.
     */
    ip27_add_ram_banks(system_memory, IP27_HSPEC_BASE, ram,
                       machine->ram_size, banksz, "sgi-ip27.ram.hspec");
  }


  /* Hub ASIC in the node's widget-1 small window. */
  hub = qdev_new(TYPE_SGI_HUB);
  qdev_prop_set_uint32(hub, "nasid", 0);
  qdev_prop_set_uint32(hub, "num-cpus", ncpus);
  qdev_prop_set_uint64(hub, "mem-config",
                       sgi_ip27_mem_config(machine->ram_size));
  sysbus_realize_and_unref(SYS_BUS_DEVICE(hub), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(hub), 0, ip27_swin_phys(0, IP27_HUB_WIDGET));

  /*
   * BaseIO board as XIO widget 0 (Bridge part 0xc002 + IOC3).  The PROM
   * probes this widget's ID once the hub link reads up and then targets the
   * node's IO widget (widget 8) for the Bridge registers/config.
   */
  baseio = qdev_new(TYPE_SGI_BASEIO);
  qdev_prop_set_uint32(baseio, "nasid", 0);
  qdev_prop_set_uint32(baseio, "widget", 0);
  sysbus_realize_and_unref(SYS_BUS_DEVICE(baseio), &error_fatal);
  sysbus_mmio_map(SYS_BUS_DEVICE(baseio), 0, ip27_swin_phys(0, 0));

  {
    DeviceState *baseio8 = qdev_new(TYPE_SGI_BASEIO);
    qdev_prop_set_uint32(baseio8, "nasid", 0);
    qdev_prop_set_uint32(baseio8, "widget", 8);
    /* The node's IO board (widget 8) carries the IOC3 Ethernet backend. */
    qemu_configure_nic_device(baseio8, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(baseio8), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(baseio8), 0, ip27_swin_phys(0, 8));
  }

  /*
   * Remaining XIO widget space of the node (widgets 2..7, 9..), so stray
   * probes read 0 instead of raising a data bus error.
   */
  create_unimplemented_device("ip27-xio-low", ip27_phys(IP27_IO_BASE) + 0x2000000ULL,
                              0x6000000ULL);
  create_unimplemented_device("ip27-xio-high", ip27_phys(IP27_IO_BASE) + 0x9000000ULL,
                              0x100000000ULL - 0x9000000ULL);

  /* Hub RBOOT window is aliased onto the boot flash after it is created. */

  /*
   * Hub back-door space (HSPEC + 0x80000000: BDDIR/BDPRT/BDECC directory,
   * protection and ECC byte arrays).  The PROM initialises and tests the
   * memory directory through it; back it with RAM so the pattern writes
   * round-trip.
   */
  {
    MemoryRegion *bdoor = g_new(MemoryRegion, 1);
    {
      uint64_t period;
      uint64_t store;
      ip27_bdoor_bank0_size = MIN(machine->ram_size, (uint64_t)0x8000000);
      period = ip27_bdoor_bank0_size >> 2;
      ip27_bdoor_num_banks =
          (machine->ram_size + ip27_bdoor_bank0_size - 1) / ip27_bdoor_bank0_size;
      store = MAX(IP27_BDDIR_STORE, ip27_bdoor_num_banks * period);
      ip27_bdoor_dir = g_malloc0(store);
      ip27_bdecc_dir = g_malloc0(store);
    }
    memory_region_init_io(bdoor, NULL, &ip27_bdoor_ops, NULL,
                          "sgi-ip27.bdoor", 0x80000000ULL);
    memory_region_add_subregion(system_memory,
                                ip27_phys(IP27_HSPEC_BASE + 0x80000000ULL),
                                bdoor);
  }

  hub_state = SGI_HUB(hub);
  {
    CPUState *c;
    i = 0;
    CPU_FOREACH(c) {
      if (i >= IP27_MAX_CPUS) {
        break;
      }
      hub_state->cpus[i] = c;
      /*
       * Hub interrupt levels map to CP0 lines: INT_PEND0 -> IP2,
       * INT_PEND1 -> IP3, RTC -> IP4, PROF -> IP5, error -> IP6.
       */
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 0,
                         MIPS_CPU(c)->env.irq[2]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 1,
                         MIPS_CPU(c)->env.irq[3]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 2,
                         MIPS_CPU(c)->env.irq[4]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 3,
                         MIPS_CPU(c)->env.irq[5]);
      sysbus_connect_irq(SYS_BUS_DEVICE(hub), i * 5 + 4,
                         MIPS_CPU(c)->env.irq[6]);
      i++;
    }
  }

  /*
   * PROM window.  The IP27 PROM executes from this window and also keeps
   * writable data/BSS there (it is RAM-backed after the flash image is
   * copied in), so it must be writable.
   */
  prom = g_new(MemoryRegion, 1);
  memory_region_init_ram(prom, NULL, "sgi-ip27.prom", IP27_PROM_SIZE,
                         &error_fatal);
  memory_region_add_subregion(system_memory, IP27_PROM_BASE, prom);

  /* Boot flash in the LBOOT window (identity + the container image). */
  flash = g_new(MemoryRegion, 1);
  memory_region_init_io(flash, NULL, &ip27_flash_ops, NULL, "sgi-ip27.flash",
                        8 * IP27_FLASH_SIZE);
  memory_region_add_subregion(system_memory, ip27_phys(IP27_LBOOT_PHYS),
                              flash);

  /*
   * RBOOT is a per-node HSPEC alias of the same boot flash; the firmware
   * reads IP27CONFIG via IP27CONFIG_ADDR_NODE = RBOOT+0x60.  Alias the flash
   * into the RBOOT window so the synthesized config is visible there too.
   */
  {
    MemoryRegion *rboot = g_new(MemoryRegion, 1);
    memory_region_init_alias(rboot, NULL, "sgi-ip27.rboot", flash, 0,
                             8 * IP27_FLASH_SIZE);
    memory_region_add_subregion(system_memory,
                                ip27_phys(IP27_HSPEC_BASE + 0x30000000ULL),
                                rboot);
  }

  if (machine->firmware) {
    g_autofree char *filename =
        qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!filename) {
      error_report("sgi-ip27: could not find firmware '%s'",
                   machine->firmware);
      exit(EXIT_FAILURE);
    }
    sgi_ip27_load_prom(filename, prom, flash);
  } else {
    warn_report("sgi-ip27: no firmware specified; use -bios ip27prom.img");
  }
}

static void sgi_ip27_class_init(ObjectClass *oc, const void *data) {
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "SGI Origin 200/2000 (IP27, headless)";
  mc->init = sgi_ip27_init;
  mc->default_ram_size = 256 * MiB;
  mc->default_ram_id = "sgi-ip27.ram";
  mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R10000");
  mc->default_cpus = 2;
  mc->max_cpus = IP27_MAX_CPUS;
  mc->no_floppy = 1;
  mc->no_cdrom = 1;
  mc->no_parallel = 1;
  mc->block_default_type = IF_SCSI;
}

static const TypeInfo sgi_ip27_type = {
    .name = MACHINE_TYPE_NAME("sgi-ip27"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_ip27_class_init,
};

static void sgi_ip27_machine_init(void) { type_register_static(&sgi_ip27_type); }

type_init(sgi_ip27_machine_init)
