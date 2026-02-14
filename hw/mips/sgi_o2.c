/*
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
 *   0x17000000-0x170003FF  Virtual boot disk
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

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/mips/mips.h"
#include "hw/misc/sgi_crime.h"
#include "hw/misc/sgi_crime_re.h"
#include "hw/misc/sgi_mace.h"
#include "hw/misc/sgi_bootdisk.h"
#include "hw/display/sgi_gbe.h"
#include "hw/misc/unimp.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/char/serial.h"
#include "hw/char/serial-mm.h"
#include "system/system.h"
#include "system/reset.h"
#include "cpu.h"
#include "qemu/log.h"

/* O2 memory map addresses */
#define O2_RAM_BASE           0x00000000ULL
#define O2_CRIME_BASE         0x14000000ULL
#define O2_CRIME_RE_BASE      0x15000000ULL
#define O2_BOOTDISK_BASE      0x17000000ULL
#define O2_GBE_BASE           0x16000000ULL
#define O2_MACE_BASE          0x1F000000ULL
#define O2_PROM_BASE          0x1FC00000ULL

#define O2_PROM_SIZE          (512 * KiB)
#define O2_RAM_MAX            (256 * MiB)

/*
 * SEG1 address space: physical 0x40000000-0x7FFFFFFF
 *
 * The O2 has dual address mapping — SEG0 (0x00000000) and SEG1 (0x40000000)
 * access the same SDRAM. The PROM's SizeMEM() probes memory via SEG1 using
 * TLB entries mapping virtual addresses to physical 0x40000000+.
 *
 * Each CRIME memory bank occupies a 128MB window in SEG1 space. With 32MB
 * SIMMs, the 32MB of RAM mirrors 4 times within the 128MB window. The
 * probing algorithm detects this aliasing pattern to determine SIMM size.
 */
#define O2_SEG1_BASE          0x40000000ULL
#define O2_BANK_SIZE_32M      (32 * MiB)
#define O2_BANK_WINDOW        (128 * MiB)
#define O2_NUM_BANKS          8

static void main_cpu_reset(void *opaque)
{
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
                                              int patch_off, uint32_t old_val)
{
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
static void sgi_o2_patch_prom_memtest(int bios_size)
{
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
                sgi_o2_fix_segment_body_checksum(rom, bios_size,
                                                  patch_off, instr);
                return;
            }
        }
    }
}

static void sgi_o2_init(MachineState *machine)
{
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *prom;
    DeviceState *crime_dev;
    DeviceState *mace_dev;
    DeviceState *gbe_dev;
    MIPSCPU *cpu;
    Clock *cpuclk;
    char *filename;
    int bios_size;

    /* Validate RAM size */
    if (machine->ram_size > O2_RAM_MAX) {
        error_report("RAM size more than 256MB is not supported");
        exit(EXIT_FAILURE);
    }

    /* Create CPU clock (R5000 @ 180MHz default) */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 180000000);

    /* Create CPU */
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, true);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    qemu_register_reset(main_cpu_reset, cpu);

    /*
     * RAM at physical 0x00000000.
     * IP32 has RAM starting at address 0 (unlike IP24 which starts at 0x08000000).
     * machine->ram is created automatically by QEMU via default_ram_id.
     */
    memory_region_add_subregion(system_memory, O2_RAM_BASE, machine->ram);

    /*
     * SEG1 RAM aliases at physical 0x40000000+.
     *
     * The real PROM's SizeMEM() probes memory through TLB entries that map
     * to SEG1. Each CRIME bank gets a 128MB window. With 32MB SIMMs, the
     * RAM mirrors 4 times within each window — the probing algorithm
     * detects this aliasing to determine SIMM size.
     *
     * Create aliases for populated banks and unimplemented devices for
     * empty banks (returning 0 on reads = "no SIMM").
     */
    {
        int num_banks = machine->ram_size / O2_BANK_SIZE_32M;
        int bank, mirror;
        char name[32];

        for (bank = 0; bank < O2_NUM_BANKS; bank++) {
            uint64_t seg1_base = O2_SEG1_BASE + bank * O2_BANK_WINDOW;

            if (bank < num_banks) {
                /* Populated bank: mirror 32MB RAM 4 times in 128MB window */
                for (mirror = 0; mirror < 4; mirror++) {
                    MemoryRegion *alias = g_new(MemoryRegion, 1);
                    snprintf(name, sizeof(name), "seg1-bank%d-m%d",
                             bank, mirror);
                    memory_region_init_alias(alias, NULL, name,
                        machine->ram, bank * O2_BANK_SIZE_32M,
                        O2_BANK_SIZE_32M);
                    memory_region_add_subregion(system_memory,
                        seg1_base + mirror * O2_BANK_SIZE_32M, alias);
                }
            } else {
                /* Empty bank: return 0 on reads (no SIMM) */
                snprintf(name, sizeof(name), "seg1-empty%d", bank);
                create_unimplemented_device(name, seg1_base, O2_BANK_WINDOW);
            }
        }
    }

    /* PROM at 0x1FC00000 */
    prom = g_new(MemoryRegion, 1);
    memory_region_init_rom(prom, NULL, "sgi-o2.prom", O2_PROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(system_memory, O2_PROM_BASE, prom);

    /* Load PROM/BIOS */
    if (machine->firmware) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        if (!filename) {
            error_report("Could not find firmware '%s'", machine->firmware);
            exit(EXIT_FAILURE);
        }
    } else {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, "ip54.bin");
        if (!filename) {
            warn_report("No firmware specified, use -bios to specify IP32 PROM");
            filename = NULL;
        }
    }

    if (filename) {
        bios_size = load_image_targphys(filename, O2_PROM_BASE,
                                        O2_PROM_SIZE, NULL);
        g_free(filename);
        if (bios_size < 0) {
            error_report("Could not load PROM image");
            exit(EXIT_FAILURE);
        }
        sgi_o2_patch_prom_memtest(bios_size);
    }

    /* CRIME at 0x14000000 */
    crime_dev = qdev_new(TYPE_SGI_CRIME);
    qdev_prop_set_uint32(crime_dev, "ram-size", machine->ram_size);
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
        MemoryRegion *mace_mr = sysbus_mmio_get_region(
            SYS_BUS_DEVICE(mace_dev), 0);
        serial_mm_init(mace_mr, MACE_SER1_OFFSET, 8,
                       NULL, 115200, serial_hd(0), DEVICE_BIG_ENDIAN);
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
     * Virtual Boot Disk at 0x15000000.
     * Simple MMIO block device for PROM to load kernels without SCSI.
     * Attach via: -drive file=disk.img,if=mtd,format=raw
     */
    {
        DeviceState *bootdisk_dev;
        DriveInfo *dinfo;

        bootdisk_dev = qdev_new(TYPE_SGI_BOOTDISK);
        /* Try multiple interfaces: mtd, pflash, then scsi bus=0 unit=1 */
        dinfo = drive_get(IF_MTD, 0, 0);
        if (!dinfo) {
            dinfo = drive_get(IF_PFLASH, 0, 0);
        }
        if (!dinfo) {
            /* MCP tool uses -drive if=scsi,bus=0,unit=1 for first disk */
            dinfo = drive_get(IF_SCSI, 0, 1);
        }
        if (dinfo) {
            qdev_prop_set_drive_err(bootdisk_dev, "drive",
                                    blk_by_legacy_dinfo(dinfo),
                                    &error_fatal);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bootdisk_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(bootdisk_dev), 0, O2_BOOTDISK_BASE);
    }

    /* GBE at 0x16000000 */
    gbe_dev = qdev_new(TYPE_SGI_GBE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gbe_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(gbe_dev), 0, O2_GBE_BASE);

    /*
     * Unimplemented device stubs for memory probing.
     * The PROM probes memory by writing patterns and reading back.
     * Unmapped regions need to return 0 (pattern mismatch) instead
     * of causing bus errors.
     *
     * Cover the gap between installed RAM and the start of CRIME.
     * The SEG1 aliases above handle 0x40000000+.
     */
    if (machine->ram_size < O2_RAM_MAX) {
        create_unimplemented_device("mem-probe-high",
                                    O2_RAM_BASE + machine->ram_size,
                                    O2_CRIME_BASE - (O2_RAM_BASE + machine->ram_size));
    }

    /* Gap between CRIME base regs and CRIME RE */
    create_unimplemented_device("gap-crime-re",
                                0x14000280, 0x00FFFD80);
    /* Gap between CRIME RE and GBE */
    create_unimplemented_device("gap-re-gbe",
                                0x15005000, 0x00FFB000);
    /* Gap between GBE and bootdisk */
    create_unimplemented_device("gap-gbe-bootdisk",
                                0x16100000, 0x00F00000);
    /* Gap between bootdisk and MACE */
    create_unimplemented_device("gap-bootdisk-mace",
                                0x17000400, 0x07FFFC00);

    /*
     * Cover the gap after MACE (0x1F400000-0x1FFFFFFF).
     * The PROM ROM subregion at 0x1FC00000 takes priority, but the
     * kernel scans one word past the end of PROM (0x1FC80000) looking
     * for "SHDR" markers. Without this, that read causes a bus error.
     */
    create_unimplemented_device("gap-post-mace",
                                0x1F400000, 0x00C00000);
}

static void sgi_o2_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SGI O2 (IP32)";
    mc->init = sgi_o2_init;
    mc->block_default_type = IF_MTD;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "sgi-o2.ram";
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("R5000");
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
}

static const TypeInfo sgi_o2_type = {
    .name = MACHINE_TYPE_NAME("sgi-o2"),
    .parent = TYPE_MACHINE,
    .class_init = sgi_o2_class_init,
};

static void sgi_o2_machine_init(void)
{
    type_register_static(&sgi_o2_type);
}

type_init(sgi_o2_machine_init)
