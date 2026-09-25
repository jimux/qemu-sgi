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

#include "trace.h"

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

/*
 * Kernel K2 (XKSEG) entry base.  The kernel links at 0xc000000000000000+ and
 * its entry runs there, so this VPN must be TLB-mapped before the jump.
 */
#define IP27_K2_BASE 0xc000000000000000ULL

/* Highest node/brick tag (in units of 1<<32) aliased back to node-0 RAM (B1). */
#define IP27_NODE_TAG_MAX 0x3ff
/*
 * PageMask for a 16 MB page, and EntryLo flags V|D|G|C=5 -- enough to cover
 * the kernel's load at the start of node RAM.  (Encoding mirrors the PROM
 * mapping flags above.)
 */
#define IP27_K2_PAGEMASK 0x01ffe000ULL
#define IP27_K2_FLAGS 0x2fULL

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
 * Two-node PROM window size.  The mapped IP27 PROM aperture is
 * [0x1fc00000, base + size); node-0 RAM bank slot 1 begins at
 * 0x20000000 (MD_BANK_SHFT = 29, see ip27_add_ram_banks), so the window
 * MUST NOT extend to or past 0x20000000 or it silently shadows the first
 * bytes of bank 1: a BTE migration copy whose destination falls in that
 * shadow lands in the PROM region instead of RAM, and the tagged TLB
 * alias then reads an empty page (the two-node coredump storm).
 *
 * The authentic IP27 map (kern/sys/SN/SN0/addrs.h) is a 1 MB image at
 * 0x1fc00000 (IP27PROM_BASE_MAPPED, IP27PROM_SIZE_MAX = 0x100000) with
 * the PROM's data/PCFG at physical 0x01a00000/0x01b00000 (node RAM), not
 * above 0x1fc00000.  This model instead uses the mapped window as the
 * PROM's code+scratch RAM, and the two-node PROM needs roughly 4 MB of
 * it (a 1 MB window boots single-node but PANICs the two-node kernel;
 * [ASSUMPTION] this model simplification should eventually be replaced
 * by backing the PROM data at its authentic 0x01a00000/0x01b00000
 * addresses).  4 MB is the largest window that ends exactly at
 * 0x20000000 and therefore cannot overlap node RAM.  Single-node keeps
 * its authentic 1 MB window byte-for-byte.
 */
#define IP27_PROM_SIZE_TWO (4 * MiB)

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

/*
 * The machine's single router (NULL in single-node mode), kept file-scope so
 * sgi_ip27_local_reset() can re-initialise it on a guest-requested reset.
 */
static SGIRouterState *ip27_router;

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

  /*
   * The router is machine-owned, not a DeviceState, so it has no QEMU reset.
   * Discovery's fence leaves RR_SCRATCH_REG1 held (0xffff) across a panic, and
   * without this the post-panic recovery boot cannot re-lock the router and
   * falls back to single-node (leg 184/185).  Re-init its volatile state here.
   */
  sgi_router_reset(ip27_router);

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
 * The boot flash is a single NOR device with two distinct views.  The CPU's
 * LBOOT/RBOOT *windows* (HSPEC+0x10000000 / HSPEC+0x30000000) translate the
 * device so the PROM *code* appears at offset 0: the PROM reads its
 * self-checksum source at LBOOT+0 and IP27CONFIG lives at LBOOT+0x60 /
 * RBOOT+0x60.  The raw device view -- reached by a user mmap, whose TO_PHYS
 * physical is 0x10000000 (LBOOT) / 0x30000000 (RBOOT) -- holds the SN0
 * container: the promhdr_t / segment table starts at 0 (magic "JFKSWCSM" at
 * 0x40, image name "ip27prom" at 0x80) and the code begins at the header's
 * code offset.
 *
 * The IRIX `flash` utility's flash_readprom/probe use the RAW view: it reads
 * a promhdr_t at 0 and expects the container magic at 0x40.  Keeping code at
 * offset 0 in the raw view (as we used to) made flash_readprom see code bytes
 * -> "Invalid Header" -> an endless "[r]etry or [i]gnore" prompt loop that
 * wedged inst's exit-commands and left /unix unbuilt.  So keep the code-at-0
 * store for the windows and a separate container store for the raw view.
 */
static uint8_t ip27_flash_code[IP27_FLASH_SIZE];   /* code-at-0 window store */
static uint8_t ip27_rawflash[IP27_FLASH_SIZE];     /* raw SN0 container */

/* AMD/Fujitsu flash command state (see libkl/ml/fprom.c do_probe/do_write). */
typedef struct IP27FlashState {
  uint8_t *mem;
  int autoselect;
  int unlock;      /* 1 = AA@0x5555 seen, 2 = +55@0x2AAA */
  int program;     /* next write stores data */
  int erase;       /* 0x80 seen, awaiting 0x10 (chip) / 0x30 (sector) */
} IP27FlashState;

static IP27FlashState ip27_flash_code_state = { .mem = ip27_flash_code };
static IP27FlashState ip27_rawflash_state = { .mem = ip27_rawflash };
static bool ip27_flash_dbg;

static uint8_t ip27_flash_byte(IP27FlashState *s, uint64_t off) {
  if (s->autoselect) {
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
    return s->mem[off];
  }
  return 0xff;
}

static uint64_t ip27_flash_read(void *opaque, hwaddr off, unsigned size) {
  IP27FlashState *s = opaque;
  uint64_t v = 0;
  unsigned i;
  for (i = 0; i < size; i++) {
    v = (v << 8) | ip27_flash_byte(s, off + i);
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
  IP27FlashState *s = opaque;
  uint64_t faddr = off / 8;
  uint8_t b = val & 0xff;

  if (ip27_flash_dbg) {
    qemu_log_mask(LOG_GUEST_ERROR, "ip27flash[%s] WR off=0x%" PRIx64
                  " size=%u val=0x%" PRIx64 "\n",
                  s == &ip27_rawflash_state ? "raw" : "code", (uint64_t)off,
                  size, val);
  }

  if (faddr == 0 && b == 0xf0) {         /* reset to read mode */
    s->autoselect = 0;
    s->unlock = 0;
    s->program = 0;
    s->erase = 0;
    return;
  }
  if (faddr == 0x5555 && b == 0xaa) {
    s->unlock = 1;
    return;
  }
  if (faddr == 0x2aaa && b == 0x55 && s->unlock == 1) {
    s->unlock = 2;
    return;
  }
  if (faddr == 0x5555 && s->unlock == 2 && b == 0x90) {
    s->autoselect = 1;
    s->erase = 0;
    s->unlock = 0;
    return;
  }
  if (faddr == 0x5555 && s->unlock == 2 && b == 0xa0) {
    s->program = 1;
    s->erase = 0;
    s->unlock = 0;
    return;
  }
  if (faddr == 0x5555 && s->unlock == 2 && b == 0x80) {
    s->erase = 1;
    s->program = 0;
    s->unlock = 0;
    return;
  }
  if (s->erase) {
    /*
     * Chip erase (0x10@0x5555) or sector erase (0x30@addr): set to 0xff.
     * A single 0x80 setup is followed by one 0x30 per sector (the IRIX flasher
     * queues all the code sectors this way), so s->erase stays set across the
     * 0x30 commands and is only cleared by the chip-erase command or a reset.
     */
    if (faddr == 0x5555 && b == 0x10) {
      s->erase = 0;
      memset(s->mem, 0xff, IP27_FLASH_SIZE);
    } else if (b == 0x30) {
      uint64_t sec = faddr & ~(uint64_t)0xffff; /* 64 KiB sector */
      if (sec + 0x10000 <= IP27_FLASH_SIZE) {
        memset(s->mem + sec, 0xff, 0x10000);
      }
    }
    return;
  }
  if (s->program) {
    /* Program: the hub flash write address is the offset * 8, and only the
     * LSByte is used.  Flash can only clear bits, so AND the data in. */
    uint64_t fidx = off / 8;
    if (fidx < IP27_FLASH_SIZE) {
      s->mem[fidx] &= (val & 0xff);
    }
    s->program = 0;
    return;
  }
  s->unlock = 0;
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
  uint8_t *flash_dst = ip27_flash_code;

  ip27_flash_dbg = getenv("IP27_FLASH_DBG") != NULL;

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

  /*
   * Raw device view (the `flash` utility's mmap of the hwgraph flash node)
   * is the SN0 container exactly as shipped: promhdr_t / segment table at 0
   * and the code at its container offset.  flash_readprom checks the magic at
   * 0x40 here, so this view must not be code-at-0.
   */
  memset(ip27_rawflash, 0xff, IP27_FLASH_SIZE);
  memcpy(ip27_rawflash, data, MIN((gsize)IP27_FLASH_SIZE, len));

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
/*
 * All three fields are in Hz, matching ip27config_t (sys/SN/SN0/ip27config.h)
 * and the PROM's own ip27config_table (whose entries pair IP27C_MHZ(200) with
 * IP27C_MHZ(100)): freq_cpu is the CPU core clock and freq_hub the hub clock.
 * freq_rtc is compared by ml/SN/mp.c:allowboot() against
 * IP27C_KHZ(IP27_RTC_FREQ) == 1000 * 1250 == 1250000, and any mismatch is a
 * fatal "RTC frequency incorrect. Please upgrade your proms" panic -- so it
 * must read 1250000, not 1250.
 */
#define IP27_CONFIG_FREQ_CPU 200000000ULL /* R10000 core clock (Hz) */
#define IP27_CONFIG_FREQ_HUB 100000000ULL /* hub clock (Hz) */
#define IP27_CONFIG_FREQ_RTC 1250000ULL   /* Hz = 1000 * 1250 kHz */
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

/*
 * IP27 kernel-load relocation region (default-on; see the init).  Forwards the
 * miniroot/sa loader's seg2 writes up by 0x1000000 so the kernel's RW segment
 * lands at its link physical address; reads pass through to real RAM.
 */
typedef struct IP27Seg2Redir {
  MemoryRegion mr;
  MemoryRegion *ram;
  uint64_t phys;
  int64_t delta; /* relocation offset added to phys (0 = read-through) */
} IP27Seg2Redir;

static uint64_t ip27_seg2redir_read(void *opaque, hwaddr off, unsigned size) {
  IP27Seg2Redir *t = opaque;
  /*
   * Never apply the redirect delta on reads: this region overlays ordinary
   * RAM, and only the loader's seg2 WRITES are relocated.  Reads must see the
   * real contents at this address (otherwise the PROM memory test writes here
   * but reads back from +0x1000000 -> miscompare -> ip27_die -> headless).
   */
  uint8_t *p = (uint8_t *)memory_region_get_ram_ptr(t->ram) + t->phys + off;
  switch (size) {
  case 1: return ldub_p(p);
  case 2: return lduw_be_p(p);
  case 4: return ldl_be_p(p);
  default: return ldq_be_p(p);
  }
}

static void ip27_seg2redir_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size) {
  IP27Seg2Redir *t = opaque;
  int64_t delta = t->delta;
  CPUMIPSState *e = current_cpu ? cpu_env(current_cpu) : NULL;
  uint64_t pc = e ? (uint64_t)e->active_tc.PC : 0;

  /*
   * Redirect mode: only the miniroot/sa flat loader relocates seg2 upward.
   * This physical range (0x38ea58..) is ordinary RAM that the PROM's memory
   * test and the kernel also use, so forward the write ONLY when the store
   * comes from the loader itself (which runs at 0xc000000011c.....); everybody
   * else must see normal memory, or the power-on memtest miscompares here.
   */
  if (delta && !(pc >= 0xc000000011c00000ULL && pc < 0xc000000011d00000ULL)) {
    delta = 0;
  }
  uint8_t *p = (uint8_t *)memory_region_get_ram_ptr(t->ram) + t->phys + delta + off;
  switch (size) {
  case 1: stb_p(p, val); break;
  case 2: stw_be_p(p, val); break;
  case 4: stl_be_p(p, val); break;
  default: stq_be_p(p, val); break;
  }
}

static const MemoryRegionOps ip27_seg2redir_ops = {
  .read = ip27_seg2redir_read,
  .write = ip27_seg2redir_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

/*
 * Diagnostic watch window (IP27_WATCH=1).  Overlays one page of flat RAM with
 * an IO region that forwards every access to real RAM unchanged, but traces
 * each write with the guest PC.  Purpose: identify the PLACER -- the code that
 * writes the kernel's RW bytes at the flat page 0x3f3928 -- by catching its
 * store.  Read-through/write-through means behaviour is identical to plain RAM
 * (important: the kernel image must still land where it always did).
 */
typedef struct IP27Watch {
  MemoryRegion mr;
  MemoryRegion *ram;
  uint64_t phys;
  uint64_t len;
} IP27Watch;

static uint64_t ip27_watch_read(void *opaque, hwaddr off, unsigned size) {
  IP27Watch *t = opaque;
  uint8_t *p = (uint8_t *)memory_region_get_ram_ptr(t->ram) + t->phys + off;

  switch (size) {
  case 1: return ldub_p(p);
  case 2: return lduw_be_p(p);
  case 4: return ldl_be_p(p);
  default: return ldq_be_p(p);
  }
}

static void ip27_watch_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size) {
  IP27Watch *t = opaque;
  CPUMIPSState *e = current_cpu ? cpu_env(current_cpu) : NULL;
  uint64_t pc = e ? (uint64_t)e->active_tc.PC : 0;
  uint8_t *p = (uint8_t *)memory_region_get_ram_ptr(t->ram) + t->phys + off;

  trace_sgi_ip27_watch(t->phys + off, pc, val, size);
  switch (size) {
  case 1: stb_p(p, val); break;
  case 2: stw_be_p(p, val); break;
  case 4: stl_be_p(p, val); break;
  default: stq_be_p(p, val); break;
  }
}

static const MemoryRegionOps ip27_watch_ops = {
  .read = ip27_watch_read,
  .write = ip27_watch_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};


/*
 * Node-remap content comparator (IP27_CMP=1).  Registered as the target's
 * same-VPN remap hook; when a user TLB entry with a valid old mapping is
 * replaced by one mapping the same VPN to a different physical page, it reads
 * the old and new pages straight out of node RAM and reports how many bytes
 * differ.  A COW / NUMA migration must carry the page's content, so a large
 * diff with the destination all-zero proves the copy did not land -- the
 * "losing write".  Also prints rld's pObj_Head word (VA 0x0fbdbbd8, page
 * offset 0x3bd8) before and after, which is the value whose loss makes the
 * runtime linker dereference NULL.  Env-gated and inert by default.
 */
static MemoryRegion *ip27_cmp_ram0;
static MemoryRegion *ip27_cmp_ram1;

static bool ip27_cmp_ptr(uint64_t pa, uint8_t **out) {
  uint64_t off = pa & 0xffffffffULL;
  MemoryRegion *mr = ((pa >> 32) & 1) ? ip27_cmp_ram1 : ip27_cmp_ram0;

  if (!mr) {
    return false;
  }
  if (off >= 0x20000000ULL && off < 0x28000000ULL) {
    off = off - 0x18000000ULL; /* bank 1 -> second 128 MB of the node */
  } else if (off >= 0x8000000ULL) {
    return false; /* hole in the bank-slot layout */
  }
  *out = (uint8_t *)memory_region_get_ram_ptr(mr) + off;
  return true;
}

static void ip27_tlb_node_remap_cmp(uint64_t old_pa, uint64_t new_pa,
                                    uint64_t len, uint32_t asid,
                                    uint64_t page_va) {
  uint8_t *op, *np;
  uint64_t diff = 0, first = len, k;
  /* Large (4 MB / 16 MB) pages would make a full byte scan pathological; the
   * pages the fault cluster lives in are 16 KB, and a content loss covers the
   * whole page, so a 64 KB prefix is decisive. */
  uint64_t slen = len > 0x10000ULL ? 0x10000ULL : len;

  if (!ip27_cmp_ptr(old_pa, &op) || !ip27_cmp_ptr(new_pa, &np)) {
    fprintf(stderr, "IP27_CMP vpn=%016" PRIx64 " asid=%04x UNMAPPED"
            " old=%016" PRIx64 " new=%016" PRIx64 "\n",
            page_va, asid, old_pa, new_pa);
    return;
  }
  for (k = 0; k < slen; k++) {
    if (op[k] != np[k]) {
      if (diff == 0) {
        first = k;
      }
      diff++;
    }
  }
  if ((page_va & ~0x3fffULL) == 0x0fbd8000ULL) {
    uint64_t ov = 0, nv = 0;

    if (0x3bd8 + 8 <= len) {
      memcpy(&ov, op + 0x3bd8, 8);
      memcpy(&nv, np + 0x3bd8, 8);
    }
    fprintf(stderr, "IP27_CMP pObjHead vpn=%016" PRIx64 " asid=%04x"
            " diff=%" PRIu64 " oldPa=%016" PRIx64 " newPa=%016" PRIx64
            " oldVal=%016" PRIx64 " newVal=%016" PRIx64 "\n",
            page_va, asid, diff, old_pa, new_pa, ov, nv);
    return;
  }
  if (diff == 0) {
    return; /* ordinary carry-preserving remap */
  }
  {
    uint64_t onz = 0, nnz = 0, k2;

    for (k2 = 0; k2 < slen; k2++) {
      if (op[k2]) {
        onz++;
      }
      if (np[k2]) {
        nnz++;
      }
    }
    {
      uint64_t as_old = 0, as_new = 0;
      MemoryRegionSection ns = memory_region_find(get_system_memory(), new_pa, 1);

      address_space_read(&address_space_memory, old_pa,
                         MEMTXATTRS_UNSPECIFIED, &as_old, sizeof(as_old));
      address_space_read(&address_space_memory, new_pa,
                         MEMTXATTRS_UNSPECIFIED, &as_new, sizeof(as_new));
      fprintf(stderr, "IP27_NEWMR new=%016" PRIx64 " mr=%s off_in_region=%"
              PRIx64 " off_in_as=%" PRIx64 "\n", new_pa,
              ns.mr ? memory_region_name(ns.mr) : "<none>",
              (uint64_t)ns.offset_within_region,
              (uint64_t)ns.offset_within_address_space);
      fprintf(stderr, "IP27_CMP vpn=%016" PRIx64 " asid=%04x len=%" PRIx64
              " diff=%" PRIu64 " first=%" PRIx64 " oldnz=%" PRIu64
              " newnz=%" PRIu64 " old=%016" PRIx64 " new=%016" PRIx64
              " as_old=%016" PRIx64 " as_new=%016" PRIx64 "\n",
              page_va, asid, len, diff, first, onz, nnz, old_pa, new_pa,
              as_old, as_new);
    }
    /*
     * IP27_CMP_SCAN=1 additionally hunts both nodes' RAM for the source page
     * so the destination of a lost copy can be located.  Expensive; opt-in.
     */
    /*
     * User-ASID only and capped: a full two-node scan per remap is very
     * expensive, and the fault cluster is user memory.  Budget via
     * IP27_CMP_SCAN_MAX (default 25).
     */
    static int cmpscan_used;
    static int cmpscan_max = -1;
    if (cmpscan_max < 0) {
      const char *m = getenv("IP27_CMP_SCAN_MAX");
      cmpscan_max = m ? atoi(m) : 25;
    }
    if (len >= 32 && getenv("IP27_CMP_SCAN") && asid != 0 &&
        cmpscan_used++ < cmpscan_max) {
      int nd;
      const uint8_t *sig = op + (first + 32 <= len ? first : 0);
      for (nd = 0; nd < 2; nd++) {
        MemoryRegion *mr = nd ? ip27_cmp_ram1 : ip27_cmp_ram0;
        uint8_t *base;
        uint64_t i, sz;

        if (!mr || !memory_region_is_ram(mr)) {
          continue;
        }
        base = (uint8_t *)memory_region_get_ram_ptr(mr);
        sz = memory_region_size(mr);
        for (i = 0; i + first + 32 <= sz; i++) {
          if (memcmp(base + i + first, sig, 32) == 0) {
            uint64_t lo = i, hi = i + len;
            uint64_t llo = (lo >= 0x20000000ULL && lo < 0x28000000ULL) ?
                           lo - 0x18000000ULL : lo;
            uint64_t lhi = (hi >= 0x20000000ULL && hi < 0x28000000ULL) ?
                           hi - 0x18000000ULL : hi;
            fprintf(stderr, "IP27_CMP src-copy found node%d host+%012" PRIx64
                    " (local %012" PRIx64 "..%012" PRIx64 ") pagematch=%d\n",
                    nd, i, llo, lhi, memcmp(base + i, op, 32) == 0);
          }
        }
      }
    }
  }
}

/*
 * IP27_CMP fill observer: for the rld .data page (VA 0x0fbd8000, pObj_Head at
 * offset 0x3bd8) print the 32-bit word actually at the mapped physical on each
 * softmmu fill.  This shows whether rld's runtime store to pObj_Head is visible
 * to later loads, and whether a node-0 vs node-1 mapping gives different data.
 */
static void ip27_tlb_fill_value(uint64_t va, uint64_t pa, int acc,
                                uint32_t asid) {
  uint8_t *p;
  uint32_t val;

  if ((va & ~0x3fffULL) != 0x0fbd8000ULL) {
    return;
  }
  if (!ip27_cmp_ptr(pa & ~0x3fffULL, &p)) {
    fprintf(stderr, "IP27_VAL va=%016" PRIx64 " pa=%016" PRIx64
            " asid=%04x acc=%d UNMAPPED\n", va, pa, asid, acc);
    return;
  }
  /* pObj_Head offset 0x3bd8; report the aligned word covering the access. */
  val = (uint32_t)((p[0x3bd8] << 24) | (p[0x3bd9] << 16) |
                   (p[0x3bda] << 8) | p[0x3bdb]);
  fprintf(stderr, "IP27_VAL va=%016" PRIx64 " pa=%016" PRIx64
          " asid=%04x acc=%d pObjHead=%08x\n", va, pa, asid, acc, val);
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
  /*
   * Node count for the two-node Origin 2000 leg.  Default 1 keeps the
   * single-node machine byte-for-byte unchanged (gate c).  IP27_NODES=2 models
   * a second hub reachable through a single router, with its own RAM; only
   * node 0 executes code (node 1 is a memory/CPU brick whose PROM does not
   * run), so -m is split evenly between the two nodes.
   */
  int nnodes = 1;
  uint64_t node_ram = machine->ram_size;
  SGIRouterState *router = NULL;
  SGIHubState *hubs[2] = { NULL, NULL };
  MemoryRegion *ram1 = NULL;
  /*
   * The module's ELSC (entry-level system controller).  Its I2C NVRAM holds
   * the module number the PROM reads over the hub's PCF8584 (libkl/io/elsc.c,
   * ml/i2c.c); one ELSC serves a whole module, so both node boards point at
   * it.  Number 1 is the first module (module 0 means "not yet assigned", and
   * router_search_pcfg() only accepts a module > 0).
   */
  SGIElscState *elsc = g_new0(SGIElscState, 1);

  if (getenv("IP27_NODES")) {
    nnodes = atoi(getenv("IP27_NODES"));
  }
  if (nnodes < 1) {
    nnodes = 1;
  }
  if (nnodes > 2) {
    nnodes = 2;
  }

  sgi_elsc_init(elsc, 1, 0);
  if (nnodes == 2) {
    node_ram = machine->ram_size / 2;
  }

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
    /*
     * IP27 derives its scheduling clock from CP0 COUNT/COMPARE (IP7); lock
     * COUNT to the host wall clock so the interval is honoured instead of the
     * virtual clock racing ahead and livelocking IP7 (see cp0_timer.c).
     */
    env->count_realtime = true;
    /*
     * Model the R10000 on-chip performance unit as idle (CP0 reg 25 reads 0).
     * IRIX's scheduler-clock handler checks it before resetting COUNT, and
     * QEMU's generic "unimplemented = ~0" makes that check falsely report an
     * overflow, so clock() never runs and IP7 livelocks.  See cpu.h perf_zero.
     */
    env->perf_zero = true;

    cpu_mips_irq_init_cpu(c);
    cpu_mips_clock_init(c);
    qemu_register_reset(main_cpu_reset, c);
    if (i != 0) {
      CPU(c)->start_powered_off = true;
    }
  }

  /*
   * Emulation accommodation: on real IP27 the loader leaves the kernel's K2
   * entry (0xc000000000000000) mapped to its load physical address before
   * jumping in.  Our executed path never installs it -- the PROM's K2 routine
   * at 0x1fc058e0 only *flushes* the entry (dmtc0 zero,EntryLo0/1; tlbwi over
   * indices 63..0).  Pin the K2 region, re-asserted whenever the guest writes
   * that invalidating entry; a valid write by the kernel's own
   * mapped_kernel_setup_tlb then supersedes it.  The mapping MUST be bit-24
   * aliased (both 16 MB halves -> the same flat physical page) or the RW
   * globals land at the wrong phys and read zero -- see
   * r4k_repin_if_kernel_entry.  Latent on every other machine (pinned_vpn 0).
   *
   * Still required (measured, leg 139): the trace of every tlbwi/tlbwr for this
   * VPN across a whole boot shows ONLY the PROM's flush loop at 0x1fc058fc
   * (dmtc0 zero,EntryLo0/1) plus our pin -- no guest code installs a valid K2
   * entry.  With the pin disabled (IP27_NOPIN=1) the kernel takes a fetch
   * exception on its first K2 instruction (start, badva=pc=0xc00000000001a7b4,
   * exc=26 access=2) instead of running.  So this is a genuine emulation gap:
   * the executed loader never leaves the K2 mapping the real firmware would.
   */
  mips_cpu_pin_kernel_mapping(IP27_K2_BASE, 0, IP27_K2_PAGEMASK,
                              IP27_K2_FLAGS);

  /*
   * Node-local memory at physical 0, aliased uncached (UNCAC/MSPEC spaces).
   * Single node: node 0 owns all of machine->ram.  Two nodes: -m is split
   * evenly, node 0 uses the low half of machine->ram, node 1 gets its own RAM
   * block, and every XKPHYS space carries node 1's memory at NODE_OFFSET(1) =
   * 1<<32 (NODE_SIZE_BITS = 32 in M-mode) so the master PROM/kernel can write
   * node 1's KLDIR/KLCONFIG and read its memory over the fabric.
   *
   * HSPEC is another access-mode alias of node memory: TO_HSPEC(x) =
   * HSPEC_BASE | (x & TO_PHYS_MASK), so HSPEC+off reads/writes local RAM at off
   * (addrs.h).  The LBOOT flash and the bdoor directory are added later, so
   * they overlay this alias.  Without it the kernel's fill of its BSS/heap via
   * HSPEC (e.g. 0x9000000001bdf818) hits unmapped space -> DBE.
   */
  {
    uint64_t banksz = MIN(node_ram, (uint64_t)0x8000000);
    MemoryRegion *ram0 = ram;

    if (nnodes == 2) {
      ram0 = g_new(MemoryRegion, 1);
      memory_region_init_alias(ram0, NULL, "sgi-ip27.ram.node0", ram, 0,
                               node_ram);
    }
    /*
     * Scratch A/B (IP27_FLAT128=1, two-node only): the default maps node0's
     * full 256 MB flat at 0 *and* bank0 0..128 MB + bank1 512..640 MB (which
     * aliases ram0 128..256 MB), so flat physical 128..256 MB and bank1
     * physical 512..640 MB share host cells.  Limit the flat window to the
     * populated bank0 (128 MB) so 128..512 MB is a real hole; if two-node stops
     * coredumping the flat/bank overhang was the collision.
     */
    if (getenv("IP27_FLAT128") && nnodes == 2) {
      MemoryRegion *flat = g_new(MemoryRegion, 1);

      memory_region_init_alias(flat, NULL, "sgi-ip27.ram.flat128", ram, 0,
                               banksz);
      memory_region_add_subregion(system_memory, 0, flat);
    } else {
      memory_region_add_subregion(system_memory, 0, ram0);
    }
    ip27_add_ram_banks(system_memory, IP27_UNCAC_BASE, ram0, node_ram, banksz,
                       "sgi-ip27.ram.uncac");
    ip27_add_ram_banks(system_memory, IP27_MSPEC_BASE, ram0, node_ram, banksz,
                       "sgi-ip27.ram.mspec");
    ip27_add_ram_banks(system_memory, IP27_CAC_BASE, ram0, node_ram, banksz,
                       "sgi-ip27.ram.cac");
    ip27_add_ram_banks(system_memory, IP27_HSPEC_BASE, ram0, node_ram, banksz,
                       "sgi-ip27.ram.hspec");
    if (nnodes == 2) {
      ram1 = g_new(MemoryRegion, 1);

      memory_region_init_ram(ram1, NULL, "sgi-ip27.ram.node1", node_ram,
                             &error_fatal);
      ip27_add_ram_banks(system_memory, IP27_UNCAC_BASE | (1ULL << 32), ram1,
                         node_ram, banksz, "sgi-ip27.ram1.uncac");
      ip27_add_ram_banks(system_memory, IP27_MSPEC_BASE | (1ULL << 32), ram1,
                         node_ram, banksz, "sgi-ip27.ram1.mspec");
      ip27_add_ram_banks(system_memory, IP27_CAC_BASE | (1ULL << 32), ram1,
                         node_ram, banksz, "sgi-ip27.ram1.cac");
      ip27_add_ram_banks(system_memory, IP27_HSPEC_BASE | (1ULL << 32), ram1,
                         node_ram, banksz, "sgi-ip27.ram1.hspec");
    }

    /*
     * B1: node-tagged physical alias.  On IP27 a physical address carries a
     * node/brick tag above the local RAM offset (NODE_OFFSET(n) = n << 32 in
     * M-mode).  The IRIX kernel builds its wired PDA physical address from
     * fpage (CAC-tagged); by the time it reaches EntryLo it is e.g.
     * 0x1c0004a4000 -- tag 0x1c0 with local offset 0x4a4000, valid node-0
     * RAM.  The node is the low bit of the tag (physical bit 32 = NASID bit 0);
     * the upper tag bits are kernel/directory attributes.  So a node-1 page
     * appears both as tag 1 (0x1_0033c000, the loader's staging form) and as
     * higher odd tags such as 0x141 (0x141_0033c000 -- measured: the two-node
     * kernel maps init's text page 0 with tag 0x141, and the content sits at
     * node-1 offset 0x33c000).  In single-node mode every tag k resolves to
     * node 0; in two-node mode odd tags resolve to node 1 and even tags to
     * node 0.  Getting this wrong made the kernel read node-0 RAM for a
     * node-1-tagged mapping -- init then faulted on a corrupt FILE* (0x1018).
     *
     * Reproduce the memory controller's BANK-SLOT layout (bank b at b << 29),
     * not a flat window: a page in the second DIMM carries tag | 0x20000000
     * (MD_BANK_SHFT = 29).
     */
    for (i = 1; i <= IP27_NODE_TAG_MAX; i++) {
      bool real1 = (nnodes == 2 && (i & 1));
      MemoryRegion *tag_ram = real1 ? ram1 : ram0;
      const char *name = real1 ? "sgi-ip27.ram.nodetag1"
                               : "sgi-ip27.ram.nodetag";

      ip27_add_ram_banks(system_memory, (uint64_t)i << 32, tag_ram, node_ram,
                         banksz, name);
    }
  }

  /*
   * IP27 kernel-load relocation -- OPT-IN ONLY (IP27_SEG2REDIR=1), OFF by
   * default.  History: this forwarded the miniroot/sa loader's seg2 writes up
   * by 0x1000000 on the theory that the RW segment must land at the ELF link
   * address 0x138ea58.  The kernel's own mapped_kernel.h disproves that:
   * MAPPED_KERN_RW_TO_PHYS(x) = (KDM_TO_PHYS(x) & 0xffffff) | node_base, i.e.
   * the RW seg2 phys is bit-24-masked to the low/flat page -- for node 0 that
   * is (0x13c3f78 & 0xffffff) = 0x3c3f78, matching the loader's flat placement
   * at the ELF p_paddr (p_paddr 0xc00000000138ea58 -> 0x38ea58).  Relocating
   * the loader's seg2 to 0x138ea58 therefore put it where the kernel never
   * looks: the store and the later load of the same VA then translated to
   * different physical pages, ra read back as 0, and jr ra went to PC 0.  That
   * null-PC is the redirect's bug, not evidence for it.
   *
   * CORRECTION (measured, one run): the loader IS right to place seg2 flat --
   * a watch on phys 0x3f3928 caught the loader (pc 0xc000000011c7ed7c) writing
   * the bytes there byte-by-byte.  The actual offender was the K2 machine pin
   * (mips_cpu_pin_kernel_mapping), which mapped the odd 16 MB half to phys
   * 0x1000000 instead of aliasing it onto the same flat page; fixed in
   * r4k_repin_if_kernel_entry.  So the flat placement and the comment above
   * are correct; the pin was what disagreed with MAPPED_KERN_RW_TO_PHYS.
   *
   * Kept reachable for A/B only.  The ON path gated the forward on the
   * storing PC being in the loader (see ip27_seg2redir_write) so ordinary RAM
   * users saw normal memory; reads were never relocated.
   */
  if (getenv("IP27_SEG2REDIR")) {
    IP27Seg2Redir *t = g_new0(IP27Seg2Redir, 1);
    t->ram = ram;
    t->phys = 0x38ea58;
    t->delta = 0x1000000;
    memory_region_init_io(&t->mr, NULL, &ip27_seg2redir_ops, t,
                          "sgi-ip27.seg2redir", 0x80000);
    memory_region_add_subregion_overlap(system_memory, t->phys, &t->mr, 11);
  }

  /* IP27_WATCH=1: watch-window over the flat RW page (diagnostic, see above).
   * IP27_WATCH_PHYS / IP27_WATCH_LEN select the window (hex), default the
   * seg2 page. */
  if (getenv("IP27_WATCH")) {
    IP27Watch *w = g_new0(IP27Watch, 1);
    const char *ph = getenv("IP27_WATCH_PHYS");
    const char *ln = getenv("IP27_WATCH_LEN");

    w->ram = ram;
    w->phys = ph ? strtoull(ph, NULL, 16) : 0x3f3000;
    w->len = ln ? strtoull(ln, NULL, 16) : 0x1000;
    memory_region_init_io(&w->mr, NULL, &ip27_watch_ops, w,
                          "sgi-ip27.watch", w->len);
    memory_region_add_subregion_overlap(system_memory, w->phys, &w->mr, 12);
    fprintf(stderr, "sgi-ip27: watch window phys=0x%" PRIx64 " len=0x%" PRIx64
            "\n", (uint64_t)w->phys, (uint64_t)w->len);
  }

  /*
   * IP27_CMP=1: register the node-remap content comparator (diagnostic only).
   */
  if (getenv("IP27_CMP")) {
    ip27_cmp_ram0 = ram;
    ip27_cmp_ram1 = ram1;
    mips_sgi_set_tlb_node_remap_hook(ip27_tlb_node_remap_cmp);
    mips_sgi_set_tlb_fill_hook(ip27_tlb_fill_value);
    fprintf(stderr, "sgi-ip27: node-remap content comparator ON\n");
  }

  /*
   * IP27_POISON1=<hex64>: fill node 1's RAM with a distinctive pattern so a
   * node-1/node-0 address-aliasing bug shows up as the poison value in a
   * node-0 location (vs. genuine disk/kernel content).  Diagnostic only.
   */
  if (nnodes == 2 && ram1 && getenv("IP27_POISON1")) {
    uint64_t pat = strtoull(getenv("IP27_POISON1"), NULL, 16);
    void *rp = memory_region_get_ram_ptr(ram1);
    uint64_t n = node_ram / 8;
    uint64_t *p64 = rp;
    uint64_t k;

    for (k = 0; k < n; k++) {
      p64[k] = pat;
    }
    fprintf(stderr, "sgi-ip27: poisoned node-1 RAM (%" PRIu64 " MB) with "
            "0x%016" PRIx64 "\n", node_ram >> 20, pat);
  }


  /*
   * Hub ASIC in each node's widget-1 small window.  Node 0 runs the CPUs;
   * node 1 (two-node leg) is a CPU-less memory brick whose PROM does not run,
   * so its NI identity is seeded here and it is only discovered/configured
   * through the fabric.
   */
  for (i = 0; i < nnodes; i++) {
    DeviceState *h = qdev_new(TYPE_SGI_HUB);

    qdev_prop_set_uint32(h, "nasid", i);
    qdev_prop_set_uint32(h, "num-cpus", (i == 0) ? ncpus : 0);
    /*
     * The two node boards of a module occupy distinct node slots so their
     * hwgraph paths (module/<m>/slot/n1 and /n2) differ.  Reporting n1 for
     * both made klhwg_connect_hubs() add the same hub->router edge twice
     * (GRAPH_DUP panic).  nodeslot_table[7] = n1, [6] = n2.
     */
    qdev_prop_set_uint32(h, "slot-id", (i == 0) ? 7 : 6);
    qdev_prop_set_uint64(h, "mem-config", sgi_ip27_mem_config(node_ram));
    /*
     * Node 1 is a CPU-dead memory brick with no IO board, so its hub's
     * IIO_LLP_CSR must report the xtalk link DOWN.  The kernel's per-node
     * io_graph_init() -> early_probe_for_widget() then sets
     * XWIDGET_PART_NUM_NONE and returns without probing the widget, instead of
     * reading an unbacked node-1 IO widget (PIO read error -> FRU analysis ->
     * KERNEL FAULT).  Grounded in ml/SN/iograph.c:395-417.
     */
    qdev_prop_set_bit(h, "io-attached", i == 0);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(h), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(h), 0, ip27_swin_phys(i, IP27_HUB_WIDGET));
    hubs[i] = SGI_HUB(h);
    sgi_hub_set_elsc(hubs[i], elsc);
    if (i == 1) {
      /*
       * Node 1's PROM never executes here, so seed the two identity registers
       * its PROM would have written before the master discovers it:
       *   NI_SCRATCH_REG0 = the hub NIC (discovery identity/dedup).
       *   NI_SCRATCH_REG1 = the advertisement.  The peer-completion handshake
       *   (robust_discover, main.c) reads ADVERT_DISCDONE and
       *   ADVERT_OBJECTS_MASK; node 1 would have discovered the same 3 objects
       *   (2 hubs + 1 router), so advertise DISCDONE|3 plus module 1 / node
       *   slot 1 / PROM version 6.  ADVERT_NASID (same low bits) is irrelevant
       *   because the NASID is assigned by the master.
       */
      hubs[i]->ni_scratch[0] = 0x000000000002ab01ULL;
      /*
       * ADVERT_CPUMASK (bits 47:44) = 0xc marks node 1's CPUs unusable, which
       * is the honest state for a node whose PROM never runs: the PROM's
       * global barriers and the kernel's CPU start-up then skip it, while its
       * memory is still configured (a CPU-dead memory brick).
       */
      hubs[i]->ni_scratch[1] = 3ULL | (1ULL << 22) | (0xcULL << 44) |
                               (1ULL << 40) | (1ULL << 56) | (6ULL << 24);
    }
  }
  hub = DEVICE(hubs[0]);
  hub_state = hubs[0];

  if (nnodes == 2) {
    /*
     * One router linking the two hubs, wired the way an Origin 2000 module
     * actually wires its two node boards.  In the IP27 PROM (discover.c
     * discover_module_indexes, nasid.c check_router/router_search_pcfg,
     * main.c router_search_pcfg) a module's router carries its two hubs on
     * ports 4 and 5 -- those are the "node" links; port 6 is PEER_PORT, the
     * daisy link to the next module's router, and ports 1/2/3 are the
     * router-to-router dimension links (the "star" variant puts hubs on
     * 1/2/3/6 instead, which is why the old ports-1/2 wiring tripped
     * star_rtr() and left router_search_pcfg() with an unconnected port 4/5/6
     * -> "ERROR: unconnected router").  Both hubs here are the two node slots
     * of module 0, so ports 4 and 5.  Registers are reached only through the
     * hubs' NI vector engine; there is no latency model.
     *
     * Port order matters because nasid_loop() derives the hub's local NASID
     * from its router port, and the PROM then relocates itself to that node's
     * local memory (tlb_ram_cac_node, tlb.s).  hub[0] is the board whose RAM
     * QEMU backs at node 0 (and carries the CPUs/BaseIO), so it must be the
     * module's n0 board.  With hub[0] on port 4 the PROM gave it NASID 1,
     * tlb_ram_cac_node(1) re-addressed its cached code to node 1's RAM -- which
     * a CPU-dead node 1 never populated -- and the CPU ran off the end of the
     * (zeroed) 2 MB window into an XTLB Refill fetch at 0xc00000001fe00000.
     * hub[0] on port 5 yields NASID 0 and the relocation finds the PROM copy.
     */
    router = g_new0(SGIRouterState, 1);
    /*
     * RSRI_CHIPIN (bits 11:8) is the router's slot strap: bit 3 clear means a
     * META router (discover_router() sets PCFG_ROUTER_META on ~CHIPIN & 8, and
     * nasid_assign() rejects "nodes attached to meta router"), so a normal
     * R-brick has it set.  The same 4-bit field indexes routerslot_table
     * (libkl/ml/slots.c): only straps 14/15 are valid, mapping to r2/r1 -- all
     * other values are SLOTNUM_INVALID_CLASS, which later makes the kernel's
     * klhwg_connect_hubs() panic with "Can't find board: .../slot/Invalid0/
     * router".  A single R-brick in a two-node module is r1, so report 0xf
     * (bit 3 set => not META; routerslot_table[15] = ROUTER_CLASS|1 = r1).
     */
    sgi_router_init(router, 0x00000000c0ffee01ULL, 0xf, 2);
    sgi_router_connect(router, 4, hubs[1]);
    sgi_router_connect(router, 5, hubs[0]);
    sgi_hub_set_router(hubs[0], router);
    sgi_hub_set_router(hubs[1], router);
    ip27_router = router;
  }

  /*
   * BaseIO board as XIO widget 0 (Bridge part 0xc002 + IOC3).  The PROM
   * probes this widget's ID once the hub link reads up and then targets the
   * node's IO widget (widget 8) for the Bridge registers/config.
   */
  baseio = qdev_new(TYPE_SGI_BASEIO);
  qdev_prop_set_uint32(baseio, "nasid", 0);
  qdev_prop_set_uint32(baseio, "widget", 0);
  /*
   * Discovery alias at SWIN widget 0: the kernel reads the Bridge part
   * (WIDGET_ID) and its widget id (BRIDGE_WID_CONTROL/WIDGET_CONTROL) HERE to
   * derive basew_id (ml/SN/iograph.c:849) and then addresses the bridge's PCI
   * devices at that widget.  It must therefore report the REAL board's id (8),
   * or basew_id comes out 0 and scsi_ctlr_nums_add walks an empty bridge --
   * which is why devnamefromarcs degenerated to /hw/target/... and the root
   * device was never found.
   */
  qdev_prop_set_uint32(baseio, "wid-id", 8);
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
    /*
     * The base IO bridge delivers its PCI device interrupts to this node's
     * hub: it latches the vector the kernel programs in b_int_addr[line]
     * into the hub's INT_PEND0/1 (sgi_hub_raise_vector).
     */
    SGI_BASEIO(baseio8)->hub = hub_state;

    /*
     * Big-window alias of widget 0.  The IRIX kernel's NODE_SWIN_BASE(nasid,0)
     * resolves widget 0 through the BIG window (sys/SN/SN0/addrs.h:106:
     * widget 0 -> NODE_BWIN_BASE(nasid, SWIN0_BIGWIN)), NOT the small window,
     * because a directly-connected bridge may sit at any widget.  With
     * IIO_NUM_ITTES=7 (SWIN0_BIGWIN=6) and BWIN_SIZE_BITS=29 that address is
     * IO_BASE + 7*BWIN_SIZE = IO_BASE + 0xE0000000.  The kernel reads WIDGET_ID
     * (0x04) and WIDGET_CONTROL (0x24) there to identify the widget
     * (ml/SN/iograph.c:847-849).  Without this alias those reads fell into the
     * unimplemented xio-high device and returned 0; part_num 0 is
     * XBOW_WIDGET_PART_NUM (io/xbow.h:271), which the kernel reported as the
     * phantom "DOWNREV Crossbow ASIC ... rev unknown (code=0)", and
     * base_io_scsi_ctlr_vhdl[] stayed empty.  Alias the real board window here
     * (same offsets as the widget window).
     */
    {
      MemoryRegion *bwin0 = g_new(MemoryRegion, 1);
      memory_region_init_alias(bwin0, NULL, "sgi-ip27.baseio.bwin0",
                               sysbus_mmio_get_region(SYS_BUS_DEVICE(baseio8),
                                                      0),
                               0, SGI_BASEIO_WINDOW_SIZE);
      memory_region_add_subregion_overlap(system_memory,
          ip27_phys(IP27_IO_BASE) + 7 * (1ULL << 29), bwin0, 1);
    }
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
      ip27_bdoor_bank0_size = MIN(node_ram, (uint64_t)0x8000000);
      period = ip27_bdoor_bank0_size >> 2;
      ip27_bdoor_num_banks =
          (node_ram + ip27_bdoor_bank0_size - 1) / ip27_bdoor_bank0_size;
      store = MAX(IP27_BDDIR_STORE, ip27_bdoor_num_banks * period);
      ip27_bdoor_dir = g_malloc0(store);
      ip27_bdecc_dir = g_malloc0(store);
    }
    memory_region_init_io(bdoor, NULL, &ip27_bdoor_ops, NULL,
                          "sgi-ip27.bdoor", 0x80000000ULL);
    memory_region_add_subregion(system_memory,
                                ip27_phys(IP27_HSPEC_BASE + 0x80000000ULL),
                                bdoor);
    if (nnodes == 2) {
      /*
       * Node 1's back-door window.  Both nodes have the same DIMM geometry, so
       * they share the storage arrays (the PROM only sizes/tests each node's
       * own RAM; the register-level fabric means no cross-node aliasing).
       */
      MemoryRegion *bdoor1 = g_new(MemoryRegion, 1);

      memory_region_init_io(bdoor1, NULL, &ip27_bdoor_ops, NULL,
                            "sgi-ip27.bdoor1", 0x80000000ULL);
      memory_region_add_subregion(
          system_memory,
          ip27_phys(IP27_HSPEC_BASE | (1ULL << 32)) + 0x80000000ULL, bdoor1);
    }
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
  memory_region_init_ram(prom, NULL, "sgi-ip27.prom",
                         (nnodes == 2) ? IP27_PROM_SIZE_TWO : IP27_PROM_SIZE,
                         &error_fatal);
  memory_region_add_subregion(system_memory, IP27_PROM_BASE, prom);

  /* Boot flash in the LBOOT window (identity + the code-at-0 image). */
  flash = g_new(MemoryRegion, 1);
  memory_region_init_io(flash, NULL, &ip27_flash_ops, &ip27_flash_code_state,
                        "sgi-ip27.flash.code", 8 * IP27_FLASH_SIZE);
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
    if (nnodes == 2) {
      /*
       * Node 1's LBOOT/RBOOT windows.  The kernel reads each node's IP27CONFIG
       * via IP27CONFIG_ADDR_NODE(nasid) = RBOOT(nasid)+0x60 and probes its
       * LBOOT flash, so alias node 1's windows onto the same synthesized flash.
       * (Node 1's PROM does not run; the config is common.)
       */
      MemoryRegion *l1 = g_new(MemoryRegion, 1);
      MemoryRegion *r1 = g_new(MemoryRegion, 1);

      memory_region_init_alias(l1, NULL, "sgi-ip27.lboot1", flash, 0,
                               8 * IP27_FLASH_SIZE);
      memory_region_add_subregion(
          system_memory,
          ip27_phys(IP27_HSPEC_BASE | (1ULL << 32)) + 0x10000000ULL, l1);
      memory_region_init_alias(r1, NULL, "sgi-ip27.rboot1", flash, 0,
                               8 * IP27_FLASH_SIZE);
      memory_region_add_subregion(
          system_memory,
          ip27_phys(IP27_HSPEC_BASE | (1ULL << 32)) + 0x30000000ULL, r1);
    }
  }

  /*
   * The guest also reaches the flash by its 40-bit PHYSICAL address.  IRIX's
   * TO_PHYS() masks with 0xffffffffff (R10000, sys/mips_addrspace.h), dropping
   * the XKPHYS space discriminator our IP27_PAMASK retains; a kernel mmap of
   * the hwgraph flash node (e.g. /dev/mmem) then installs a user PTE whose
   * PFN is the bare physical (0x10000000 LBOOT / 0x30000000 RBOOT).  With the
   * flash mapped only at the discriminator-preserving physical, that TLB
   * access was unbacked and raised a user Data Bus Error.  Measured on the
   * installer's exit-command `flash -p`: pc=0x1000c148, size=8 store,
   * phys=0x30000000 (the RBOOT window).  These low physical windows are the
   * RAW device view (see the ops comment): the `flash` utility reads a
   * promhdr_t at offset 0 and needs the SN0 container there, not the code.
   */
  {
    MemoryRegion *raw = g_new(MemoryRegion, 1);
    memory_region_init_io(raw, NULL, &ip27_flash_ops, &ip27_rawflash_state,
                          "sgi-ip27.flash.raw", 8 * IP27_FLASH_SIZE);
    memory_region_add_subregion_overlap(system_memory, 0x10000000ULL, raw, 2);
    raw = g_new(MemoryRegion, 1);
    memory_region_init_io(raw, NULL, &ip27_flash_ops, &ip27_rawflash_state,
                          "sgi-ip27.flash.raw", 8 * IP27_FLASH_SIZE);
    memory_region_add_subregion_overlap(system_memory, 0x30000000ULL, raw, 2);
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
