/*
 * SGI VICE (Video, Imaging and Compression Engine) — tier-i skeleton
 *
 * Models the O2 (IP32) VICE host PIO window at 0x17000000 (1 MB): the chip
 * register/descriptor block, MSP/BSP IRAM, BSP table and FIFOs, MSP DRAM,
 * and the 128-entry DMA TLB. It answers the kernel's boot probe
 * (PROBE_AND_INVENTORY_VICE): a write to MSP_SW_INT latches VICE_INT bit 2
 * and, when enabled, raises the CRIME VICE interrupt line, so IRIX records
 * INV_COMPRESSION / INV_VICE.
 *
 * Deliberately NOT modelled here (tier-ii): MSP/BSP microcode interpretation,
 * DMA data movement, and host-codec offload. DMA "GO" writes are logged and
 * the descriptor/register state is kept as storage only.
 *
 * @@SEMANTICS@@ scope: this file is the separable tier-i seam. Chip/RAM/TLB
 * state is stored faithfully, identification and the interrupt handshake are
 * functional, and every deliberately-unimplemented path is tagged below so
 * the tier-ii host-offload work can attach without reworking this layer.
 *
 * Register access lanes [vice_drv.h]:
 *   32-bit register: big-endian high lane, offset + 4
 *   16-bit register: offset + 6
 *   IRAM/DRAM/table/TLB/FIFO: bare offset
 * The chip register block is therefore stored as big-endian bytes indexed by
 * the raw (lane-inclusive) offset, which makes both lanes land in the right
 * place; register identity for the few semantic registers is recovered with
 * (offset & ~7).
 *
 * References:
 *   - VICE Design Specification 099-0123-003 (April 17, 1997)
 *   - IRIX sys/vice/{vice_drv.h,vice_regs_host.h,vice_dma.h}
 *   - IRIX kern/ml/MOOSEHEAD/{IP32init.c,vice_copy.c}
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgi_vice.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#include <stdlib.h>
#include <unistd.h>

/*
 * VICE internal clock is the CRIME master clock (66.67 MHz, ~15 ns/tick);
 * VICE_COUNT is a free-running counter off that clock. We derive it from the
 * virtual clock rather than inventing a constant.
 */
#define VICE_NS_PER_TICK 15

static uint64_t sgi_vice_read_chip(SGIViceState *s, hwaddr off, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | s->chip_regs[off + i];
    }
    return v;
}

static void sgi_vice_write_chip(SGIViceState *s, hwaddr off, uint64_t value,
                                unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        s->chip_regs[off + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
}

static uint64_t sgi_vice_read_bytes(const uint8_t *buf, hwaddr off,
                                    unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | buf[off + i];
    }
    return v;
}

static void sgi_vice_write_bytes(uint8_t *buf, hwaddr off, uint64_t value,
                                 unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        buf[off + i] = (value >> (8 * (size - 1 - i))) & 0xff;
    }
}

/*
 * Microcode fingerprint database.
 *
 * @@SEMANTICS@@ tier-ii identification: VICE is a programmable DSP+BSP, but
 * every codec the guest ever runs is shipped as a fixed file under
 * /var/arch/vicetre/ and uploaded into VICE_MSP_IRAM verbatim (the vice_exec
 * copy length is the .mex body length; the tail of IRAM is left zero). So the
 * operation can be identified by *content* without any MSP/BSP interpreter.
 * These are FNV-1a 64 hashes of the 4096-byte IRAM image (body then zeros),
 * computed offline from the .mex files in mxview/var/arch/vicetre/; see
 * tmp/o2-qemu/vice5/fingerprint.py and REPORT.md. Unknown images are reported
 * as "?" and are NOT completed (fail loud, never guess).
 */
static const struct {
    const char *name;
    uint64_t fnv1a;
} sgi_vice_codecs[] = {
    { "cjpeg.mex",         0xc59dfde671182066ULL },
    { "cjpeg_luma.mex",    0x0a61cb6cc9a99ec4ULL },
    { "cjfif.mex",         0x27ad4e032000d701ULL },
    { "djpeg.mex",         0x160f55a6f9008c6eULL },
    { "djfif.mex",         0xc9fd4fd42d7ab025ULL },
    { "dfjpeg.mex",        0x8fc6a05beb4ab09fULL },
    { "dvcntsc.mex",       0xcfb32f66da2c343dULL },
    { "dvcpal411.mex",     0xef8a3decb6f0755fULL },
    { "dvcpal420.mex",     0xfc7ab4ccd205ebd9ULL },
    { "dvencodentsc.mex",  0x981fe13f270f8d07ULL },
    { "dvencodepal411.mex", 0xbded7c1ad7f49c43ULL },
    { "dvencodepal420.mex", 0xce532b04b6bcf11dULL },
    { "mpeg1dec.mex",      0x9c9fed37af28f2cdULL },
    { "mpeg2dec.mex",      0xd7cf41e83f154ab3ULL },
    { "rs.mex",            0x0f939ddfca6a5b41ULL },
};

static void sgi_vice_update_irq(SGIViceState *s);

static uint64_t sgi_vice_fnv1a(const uint8_t *p, size_t n)
{
    uint64_t h = 0xcbf29ce484222325ULL;
    size_t i;

    for (i = 0; i < n; i++) {
        h ^= p[i];
        h *= 0x100000001b3ULL;
    }
    return h;
}

static const char *sgi_vice_fingerprint(const SGIViceState *s)
{
    uint64_t h = sgi_vice_fnv1a(s->msp_iram, sizeof(s->msp_iram));
    size_t i;

    for (i = 0; i < ARRAY_SIZE(sgi_vice_codecs); i++) {
        if (h == sgi_vice_codecs[i].fnv1a) {
            return sgi_vice_codecs[i].name;
        }
    }
    return NULL;
}

static uint32_t sgi_vice_tlb_phys(const SGIViceState *s, unsigned entry)
{
    uint32_t v;

    if (entry >= VICE_NTLBENTRIES) {
        return 0;
    }
    v = sgi_vice_read_bytes(s->tlb, entry * VICE_TLB_STRIDE + 4, 4);
    return (v & 0x1) ? (v & 0xffff0000u) : 0;
}

/*
 * @@SEMANTICS@@ tier-ii completion.  A codec job is started when the driver
 * takes the MSP out of reset and asserts GO: MSP_CTL_STAT bit0=1, bit1=1
 * (value 0x3) [spec 099-0123-003 Table 14; vice_chip.c:vice_exec_msp writes
 * exactly 0x3].  The driver then waits for VICE_INT_MSP_INTR (bit 2), which is
 * enabled by the VICE_INT_EN=0x7c it wrote just before [vice_exec].  The real
 * MSP would raise it from the DSP; we identify the uploaded program and raise
 * the same interrupt.  The ISR (viceintr) reads the MSP Data RAM status words
 * at 0x8000/0x8008 into the atom, then calls the atom completion callback
 * (vice_dms_jintr), which enqueues the DMS output block.
 */
#define VICE_TILE_SIZE 0x10000      /* 64 KB DMS TLB page */

/*
 * @@SEMANTICS@@ tier-ii host-codec offload seam: the output handoff.
 *
 * Traced contract (tmp/o2-qemu/vice6/REPORT.md): the produced codec bytes are
 * read from the DMS output tile (TLB entry VICE_DMS_OUT, 64 KB page, physical
 * page from the TLB) starting at offset 0, and the produced BYTE COUNT is read
 * from MSP Data RAM word 0 (VICE offset 0x8000, i.e. vice_request.vr_stat[0]).
 * viceintr copies DRAM 0x8000/0x8008 into the driver atom; vice_dms_jintr
 * hands vr_stat[0..3] to the DMS consumer, whose wrapper treats a count of 0 as
 * the empty output and a non-zero vr_stat[2] as the 704-byte-header variant.
 * The device must therefore (a) leave the produced bytes in the OUT tile and
 * (b) write the count into MSP DRAM word 0 -- the old completion zeroed DRAM
 * 0..15, which erased the count and is why the round-trip stayed empty.
 *
 * QEMU never interprets MSP/BSP microcode.  The bytes come from the external
 * host-codec helper named by the VICE_HOST_CODEC environment variable (the
 * vice6_encoder.py contract: <helper> <intile.bin> <out.jpg> <w> <h> <q>, the
 * DMS input tile dumped to intile.bin).  If no helper is configured we do NOT
 * fabricate an output: the count stays 0 (the driver's empty-output case) and
 * an UNIMP is logged.  Change this seam to a chardev transport if QEMU must
 * stay free of any exec().
 */
static void sgi_vice_host_offload(SGIViceState *s, const char *codec)
{
    const char *helper = getenv("VICE_HOST_CODEC");
    uint32_t in_phys = sgi_vice_tlb_phys(s, VICE_DMS_IN);
    uint32_t out_phys = sgi_vice_tlb_phys(s, VICE_DMS_OUT);
    const unsigned w = 128, h = 128;
    const gsize tile = (gsize)w * h * 4;   /* dmedia packed 32-bit pixels */
    gchar *in_tmp = NULL, *out_tmp = NULL, *cmd = NULL;
    gchar *out_data = NULL;
    GError *err = NULL;
    gsize out_len = 0;
    gint status = 0;
    gchar *in_buf;
    int fd;

    if (!helper || !in_phys || !out_phys) {
        return;
    }
    in_buf = g_malloc(tile);
    address_space_read(&address_space_memory, in_phys,
                       MEMTXATTRS_UNSPECIFIED, in_buf, tile);

    fd = g_file_open_tmp("vice6-in-XXXXXX", &in_tmp, &err);
    if (fd < 0) {
        g_clear_error(&err);
        g_free(in_buf);
        return;
    }
    if (write(fd, in_buf, tile) != (ssize_t)tile) {
        close(fd);
        qemu_log_mask(LOG_UNIMP, "sgi_vice: host-codec input write failed\n");
        goto out;
    }
    close(fd);
    out_tmp = g_strdup_printf("%s.jpg", in_tmp);
    cmd = g_strdup_printf("%s %s %s %u %u 75", helper, in_tmp, out_tmp, w, h);
    if (!g_spawn_command_line_sync(cmd, NULL, NULL, &status, &err) ||
        status != 0 || !g_file_get_contents(out_tmp, &out_data, &out_len, &err)) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi_vice: host-codec helper failed: %s\n",
                      err && err->message ? err->message : "?");
        goto out;
    }
    if (out_len == 0 || out_len > VICE_TILE_SIZE) {
        qemu_log_mask(LOG_UNIMP,
                      "sgi_vice: host-codec produced %zu bytes (tile %u)\n",
                      out_len, VICE_TILE_SIZE);
        goto out;
    }
    /* Place the produced bytes in the OUT tile ... */
    address_space_write(&address_space_memory, out_phys,
                        MEMTXATTRS_UNSPECIFIED, out_data, out_len);
    /* ... and report the produced count in MSP Data RAM word 0 (vr_stat[0]). */
    s->msp_dram[0] = (out_len >> 24) & 0xff;
    s->msp_dram[1] = (out_len >> 16) & 0xff;
    s->msp_dram[2] = (out_len >> 8) & 0xff;
    s->msp_dram[3] = (out_len >> 0) & 0xff;
    trace_sgi_vice_offload(codec, out_len, out_phys);
out:
    if (in_tmp) {
        unlink(in_tmp);
    }
    if (out_tmp) {
        unlink(out_tmp);
        g_free(out_tmp);
    }
    g_free(in_tmp);
    g_free(in_buf);
    g_free(out_data);
    g_free(cmd);
    g_clear_error(&err);
}

static void sgi_vice_msp_go(SGIViceState *s, uint32_t ctl)
{
    const char *codec = sgi_vice_fingerprint(s);

    trace_sgi_vice_msp_go(ctl, codec ? codec : "?");
    if (!codec) {
        /* Fail loud rather than fabricate a completion for unknown ucode. */
        qemu_log_mask(LOG_UNIMP,
                      "sgi_vice: MSP GO with unrecognised IRAM; no completion\n");
        return;
    }

    trace_sgi_vice_job(codec, sgi_vice_tlb_phys(s, VICE_DMS_IN),
                       sgi_vice_tlb_phys(s, VICE_DMS_OUT),
                       sgi_vice_tlb_phys(s, VICE_DMS_AUX));
    /*
     * @@SEMANTICS@@ the MSP writes its 4-word result (vr_stat) back to the
     * first 16 bytes of Data RAM; viceintr copies DRAM 0x8000/0x8008 into the
     * atom.  Start from a zero (empty-output) result, then let the host-codec
     * seam fill in the produced count and bytes.
     */
    memset(s->msp_dram, 0, 16);
    if (strstr(codec, "jpeg")) {
        sgi_vice_host_offload(s, codec);
    }
    s->int_status |= VICE_INT_MSP_INTR;
    sgi_vice_update_irq(s);
}

static void sgi_vice_update_irq(SGIViceState *s)
{
    int level = (s->int_status & s->int_enable) != 0;

    trace_sgi_vice_int(s->int_status, s->int_enable, level);
    qemu_set_irq(s->crime_irq[0], level);
}

static uint64_t sgi_vice_read(void *opaque, hwaddr off, unsigned size)
{
    SGIViceState *s = opaque;
    hwaddr reg = off & ~7ULL;
    uint64_t v;

    trace_sgi_vice_reg_read(off, size);

    if (off >= VICE_CFG && off < VICE_CFG + 0x20) {
        switch (reg) {
        case VICE_CFG:
            return s->cfg;
        case VICE_INT_EN:
            return s->int_enable;
        default:
            return 0;
        }
    }

    if (off >= VICE_DEBUG_BASE && off < VICE_DEBUG_END) {
        /* MSP/BSP debug + exception-stamp block: storage only (tier-ii). */
        return sgi_vice_read_bytes(s->debug_regs, off - VICE_DEBUG_BASE, size);
    }

    if (off < VICE_CHIP_REGS_SIZE) {
        switch (reg) {
        case VICE_ID:
            return VICE_ID_VALUE;
        case MSP_CTL_STAT:
            /*
             * @@SEMANTICS@@ tier-i: no MSP interpreter exists yet, so the
             * only truthful state is "halted" (bit0 GO/HALT reads 0).
             */
            return 0;
        case VICE_COUNT:
            v = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / VICE_NS_PER_TICK;
            return (uint32_t)v;
        case BSP_CTL_STAT:
            return s->bsp_ctl_stat;
        case VICE_INT:
            return s->int_status;
        default:
            return sgi_vice_read_chip(s, off, size);
        }
    }

    if (off >= 0x2000 && off < 0x2000 + VICE_MSP_IRAM_SIZE) {
        return sgi_vice_read_bytes(s->msp_iram, off - 0x2000, size);
    }
    if (off >= 0x4000 && off < 0x4000 + VICE_BSP_IRAM_SIZE) {
        return sgi_vice_read_bytes(s->bsp_iram, off - 0x4000, size);
    }
    if (off >= 0x5000 && off < 0x5000 + VICE_BSP_TABLE_SIZE) {
        return sgi_vice_read_bytes(s->bsp_table, off - 0x5000, size);
    }
    if (off >= 0x7000 && off < 0x7000 + VICE_BSP_FIFO_SIZE) {
        return sgi_vice_read_bytes(s->bsp_out_fifo, off - 0x7000, size);
    }
    if (off >= 0x7800 && off < 0x7800 + VICE_BSP_FIFO_SIZE) {
        return sgi_vice_read_bytes(s->bsp_in_fifo, off - 0x7800, size);
    }
    if (off >= 0x8000 && off < 0x8000 + VICE_MSP_DRAM_SIZE) {
        return sgi_vice_read_bytes(s->msp_dram, off - 0x8000, size);
    }
    if (off >= 0xf000 && off < 0xf000 + sizeof(s->tlb)) {
        return sgi_vice_read_bytes(s->tlb, off - 0xf000, size);
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_vice: unimplemented read off=0x%" HWADDR_PRIx
                  " size=%u\n", off, size);
    return 0;
}

static void sgi_vice_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    SGIViceState *s = opaque;
    hwaddr reg = off & ~7ULL;

    trace_sgi_vice_reg_write(off, value, size);

    if (off >= VICE_CFG && off < VICE_CFG + 0x20) {
        switch (reg) {
        case VICE_CFG:
            s->cfg = (uint32_t)value;
            break;
        case VICE_INT_RESET:
            /* Write-1-to-clear the corresponding VICE_INT bits. */
            s->int_status &= ~(uint32_t)value;
            sgi_vice_update_irq(s);
            break;
        case VICE_INT_EN:
            s->int_enable = (uint32_t)value & VICE_INT_ALL;
            sgi_vice_update_irq(s);
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "sgi_vice: unimplemented cfg write off=0x%"
                          HWADDR_PRIx "\n", off);
            break;
        }
        return;
    }

    if (off >= VICE_DEBUG_BASE && off < VICE_DEBUG_END) {
        sgi_vice_write_bytes(s->debug_regs, off - VICE_DEBUG_BASE, value, size);
        return;
    }

    if (off < VICE_CHIP_REGS_SIZE) {
        switch (reg) {
        case MSP_SW_INT:
            /*
             * @@SEMANTICS@@ interrupt: write-only, any write latches
             * VICE_INT bit 2 regardless of VICE_INT_EN (spec 2.9.7); the
             * enable bit only gates the CRIME output pin.
             */
            s->int_status |= VICE_INT_MSP_INTR;
            trace_sgi_vice_sw_int("MSP");
            sgi_vice_update_irq(s);
            return;
        case BSP_SW_INT:
            s->int_status |= VICE_INT_BSP_INTR;
            trace_sgi_vice_sw_int("BSP");
            sgi_vice_update_irq(s);
            return;
        case BSP_HALT_RESET:
            if (value & VICEBSPCS_RESET) {
                s->bsp_ctl_stat = VICEBSPCS_HALT | VICEBSPCS_HALT_ACK;
            }
            break;
        case VICE_INT:
            return; /* read-only */
        case MSP_CTL_STAT:
            if (value & 0x1) {
                /*
                 * @@SEMANTICS@@ bit0 GO (spec Table 14): the driver has
                 * uploaded the program and is starting the MSP. Identify the
                 * codec and raise the documented completion interrupt.
                 */
                sgi_vice_msp_go(s, (uint32_t)value);
            }
            return;
        case VICEDMA_CTL_CH1:
        case VICEDMA_CTL_CH2:
            if (value & VICEDMA_CTL_GO) {
                /*
                 * @@SEMANTICS@@ tier-ii seam: DMA descriptors are stored,
                 * but no transfer is performed and no DMA_DONE interrupt is
                 * raised. Code that actually waits on DMA completion lives
                 * in the next milestone.
                 */
                trace_sgi_vice_dma_go(reg == VICEDMA_CTL_CH1 ? 1 : 2,
                                      (uint32_t)value);
                qemu_log_mask(LOG_UNIMP,
                              "sgi_vice: DMA ch%d transfer not implemented\n",
                              reg == VICEDMA_CTL_CH1 ? 1 : 2);
            }
            break;
        default:
            break;
        }
        sgi_vice_write_chip(s, off, value, size);
        return;
    }

    if (off >= 0x2000 && off < 0x2000 + VICE_MSP_IRAM_SIZE) {
        sgi_vice_write_bytes(s->msp_iram, off - 0x2000, value, size);
        return;
    }
    if (off >= 0x4000 && off < 0x4000 + VICE_BSP_IRAM_SIZE) {
        sgi_vice_write_bytes(s->bsp_iram, off - 0x4000, value, size);
        return;
    }
    if (off >= 0x5000 && off < 0x5000 + VICE_BSP_TABLE_SIZE) {
        sgi_vice_write_bytes(s->bsp_table, off - 0x5000, value, size);
        return;
    }
    if (off >= 0x7000 && off < 0x7000 + VICE_BSP_FIFO_SIZE) {
        sgi_vice_write_bytes(s->bsp_out_fifo, off - 0x7000, value, size);
        return;
    }
    if (off >= 0x7800 && off < 0x7800 + VICE_BSP_FIFO_SIZE) {
        sgi_vice_write_bytes(s->bsp_in_fifo, off - 0x7800, value, size);
        return;
    }
    if (off >= 0x8000 && off < 0x8000 + VICE_MSP_DRAM_SIZE) {
        sgi_vice_write_bytes(s->msp_dram, off - 0x8000, value, size);
        return;
    }
    if (off >= 0xf000 && off < 0xf000 + sizeof(s->tlb)) {
        sgi_vice_write_bytes(s->tlb, off - 0xf000, value, size);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "sgi_vice: unimplemented write off=0x%" HWADDR_PRIx
                  " size=%u val=0x%" PRIx64 "\n", off, size, value);
}

static const MemoryRegionOps sgi_vice_ops = {
    .read = sgi_vice_read,
    .write = sgi_vice_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void sgi_vice_reset(DeviceState *dev)
{
    SGIViceState *s = SGI_VICE(dev);

    memset(s->chip_regs, 0, sizeof(s->chip_regs));
    memset(s->msp_iram, 0, sizeof(s->msp_iram));
    memset(s->bsp_iram, 0, sizeof(s->bsp_iram));
    memset(s->bsp_table, 0, sizeof(s->bsp_table));
    memset(s->bsp_out_fifo, 0, sizeof(s->bsp_out_fifo));
    memset(s->bsp_in_fifo, 0, sizeof(s->bsp_in_fifo));
    memset(s->msp_dram, 0, sizeof(s->msp_dram));
    memset(s->tlb, 0, sizeof(s->tlb));
    memset(s->debug_regs, 0, sizeof(s->debug_regs));

    s->cfg = 0;
    s->int_enable = 0;
    s->int_status = 0;
    /*
     * @@SEMANTICS@@ tier-i: with no BSP interpreter the only honest state
     * is "halted"; the driver's BSP_HALT_RESET sequence (assert RESET ->
     * HALT|HALT_ACK, spec 2.6.1) is modelled on top of this.
     */
    s->bsp_ctl_stat = VICEBSPCS_HALT | VICEBSPCS_HALT_ACK;
}

static void sgi_vice_realize(DeviceState *dev, Error **errp)
{
    SGIViceState *s = SGI_VICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &sgi_vice_ops, s,
                          "sgi-vice", VICE_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Single interrupt output -> CRIME line 31 (CRM_INT_VICE). */
    qdev_init_gpio_out_named(dev, s->crime_irq, "crime-irq", 1);
}

static const VMStateDescription vmstate_sgi_vice = {
    .name = "sgi-vice",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(chip_regs, SGIViceState, VICE_CHIP_REGS_SIZE),
        VMSTATE_UINT8_ARRAY(msp_iram, SGIViceState, VICE_MSP_IRAM_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_iram, SGIViceState, VICE_BSP_IRAM_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_table, SGIViceState, VICE_BSP_TABLE_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_out_fifo, SGIViceState, VICE_BSP_FIFO_SIZE),
        VMSTATE_UINT8_ARRAY(bsp_in_fifo, SGIViceState, VICE_BSP_FIFO_SIZE),
        VMSTATE_UINT8_ARRAY(msp_dram, SGIViceState, VICE_MSP_DRAM_SIZE),
        VMSTATE_UINT8_ARRAY(tlb, SGIViceState, VICE_TLB_SIZE),
        VMSTATE_UINT8_ARRAY(debug_regs, SGIViceState, VICE_DEBUG_END -
                            VICE_DEBUG_BASE),
        VMSTATE_UINT32(cfg, SGIViceState),
        VMSTATE_UINT32(int_enable, SGIViceState),
        VMSTATE_UINT32(int_status, SGIViceState),
        VMSTATE_UINT32(bsp_ctl_stat, SGIViceState),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_vice_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_vice_realize;
    device_class_set_legacy_reset(dc, sgi_vice_reset);
    dc->vmsd = &vmstate_sgi_vice;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo sgi_vice_info = {
    .name = TYPE_SGI_VICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIViceState),
    .class_init = sgi_vice_class_init,
};

static void sgi_vice_register_types(void)
{
    type_register_static(&sgi_vice_info);
}

type_init(sgi_vice_register_types)
