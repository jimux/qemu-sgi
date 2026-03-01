/*
 * SGI HPC3 (High Performance Peripheral Controller) emulation
 *
 * The HPC3 is the central I/O controller for SGI Indy (IP24) workstations.
 * It provides:
 *   - SCSI interface (via WD33C93B)
 *   - Ethernet interface (via SEEQ 8003)
 *   - Serial ports (Z85C30 compatible)
 *   - Parallel port
 *   - DMA for all peripherals
 *   - INT3 interrupt controller (on IP24/Indy)
 *
 * References:
 *   - MAME src/mame/sgi/hpc3.cpp
 *   - IRIX kernel sys/hpc3.h
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/sgi_hpc3.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "qemu/guest-random.h"
#include "hw/core/cpu.h"
#include "ui/input.h"
#include "trace.h"

/*
 * DS1386 RTC register offsets in BBRAM
 * The first 16 bytes of BBRAM are the RTC registers.
 * (Based on DS1386 datasheet and MAME implementation)
 */
#define RTC_HUNDREDTHS    0x00
#define RTC_SECONDS       0x01
#define RTC_MINUTES       0x02
#define RTC_MINUTES_ALARM 0x03
#define RTC_HOURS         0x04
#define RTC_HOURS_ALARM   0x05
#define RTC_DAY_OF_WEEK   0x06
#define RTC_DAY_ALARM     0x07
#define RTC_DATE          0x08
#define RTC_MONTH         0x09
#define RTC_YEAR          0x0a
#define RTC_COMMAND       0x0b
#define RTC_WATCHDOG_HUND 0x0c
#define RTC_WATCHDOG_SEC  0x0d
#define RTC_USER1         0x0e
#define RTC_USER2         0x0f

/* Convert binary to BCD */
static inline uint8_t bin_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

/* Convert BCD to binary */
static inline uint8_t bcd_to_bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0f);
}

/*
 * INT3 mappable interrupt bits for timer outputs
 * Timer 0 → mappable bit 2, Timer 1 → mappable bit 3
 */
#define INT3_MAP_TIMER0  0x04
#define INT3_MAP_TIMER1  0x08

/* 8254 PIT clock frequency: 1 MHz */
#define PIT_CLOCK_HZ     1000000
#define PIT_NS_PER_TICK  1000  /* 1 MHz = 1000 ns per tick */

/*
 * HPC3 SCSI DMA control register bits (from MAME hpc3.h)
 */
#define HPC3_DMACTRL_IRQ    0x01  /* DMA interrupt pending (read); clear IRQ (write) */
#define HPC3_DMACTRL_ENDIAN 0x02  /* 0=little-endian, 1=big-endian */
#define HPC3_DMACTRL_DIR    0x04  /* 0=device→memory, 1=memory→device */
#define HPC3_DMACTRL_FLUSH  0x08  /* Flush FIFO */
#define HPC3_DMACTRL_ENABLE 0x10  /* DMA active */
#define HPC3_DMACTRL_WRMASK 0x20  /* Update flags only, preserve enable */

/* DMA descriptor BC field bits */
#define HPC3_BC_XIE         (1U << 29)  /* Interrupt on descriptor end */
#define HPC3_BC_EOX         (1U << 31)  /* End of chain */
#define HPC3_BC_COUNT_MASK  0x3fff      /* 14-bit byte count */

/* Forward declarations */
static void sgi_hpc3_update_irq(SGIHPC3State *s);
static void sgi_hpc3_scsi_dma_run(SGIHPC3State *s, int ch);
static void sgi_hpc3_enet_reset(SGIHPC3State *s);

/*
 * Fetch next DMA descriptor from guest memory
 *
 * Reads 12-byte descriptor (CBP, BC, NBDP) from the address in scsi_nbdp[ch].
 * Sets scsi_dma_count from the lower 14 bits of BC.
 */
static void sgi_hpc3_scsi_dma_fetch_chain(SGIHPC3State *s, int ch)
{
    uint32_t desc_addr = s->scsi_nbdp[ch];
    uint32_t cbp, bc, nbdp;

    cbp = address_space_ldl_be(&address_space_memory, desc_addr, MEMTXATTRS_UNSPECIFIED, NULL);
    bc = address_space_ldl_be(&address_space_memory, desc_addr + 4, MEMTXATTRS_UNSPECIFIED, NULL);
    nbdp = address_space_ldl_be(&address_space_memory, desc_addr + 8, MEMTXATTRS_UNSPECIFIED, NULL);

    s->scsi_cbp[ch] = cbp;
    s->scsi_bc[ch] = bc;
    s->scsi_nbdp[ch] = nbdp;
    s->scsi_dma_count[ch] = bc & HPC3_BC_COUNT_MASK;

    trace_sgi_hpc3_scsi_dma_chain(ch, desc_addr, nbdp,
                                  (bc & HPC3_BC_EOX) ? 1 : 0);
}

/*
 * Advance the DMA descriptor chain after a descriptor's count reaches zero.
 * Handles interrupt-on-end and end-of-chain.
 */
static void sgi_hpc3_scsi_dma_advance_chain(SGIHPC3State *s, int ch)
{
    /* Check for interrupt-on-end (XIE) */
    if (s->scsi_bc[ch] & HPC3_BC_XIE) {
        trace_sgi_hpc3_scsi_irq(ch, s->scsi_ctrl[ch]);
        s->intstat |= (0x100 << ch);
        s->scsi_ctrl[ch] |= HPC3_DMACTRL_IRQ;
        /* Route HPC DMA completion to INT3 LOCAL1 (MAME: hpc_dma_done_w) */
        s->int3_local1_stat |= INT3_LOCAL1_HPC_DMA;
        sgi_hpc3_update_irq(s);
    }

    /* Check for end-of-chain (EOX) */
    if (s->scsi_bc[ch] & HPC3_BC_EOX) {
        s->scsi_dma_active[ch] = false;
        s->scsi_ctrl[ch] &= ~HPC3_DMACTRL_ENABLE;
        return;
    }

    /* Fetch next descriptor */
    sgi_hpc3_scsi_dma_fetch_chain(s, ch);
}

/*
 * Run the SCSI DMA engine for a channel.
 *
 * Transfers data between the WD33C93's async buffer and guest memory
 * via the DMA descriptor chain. Loops while the WD33C93 has data
 * available (async_len > 0) and DMA is active.
 */
static void sgi_hpc3_scsi_dma_run(SGIHPC3State *s, int ch)
{
    WD33C93State *wdc = s->scsi[ch];
    uint32_t total_transferred = 0;

    if (!wdc || !s->scsi_dma_active[ch]) {
        return;
    }

    while (wdc->async_len > 0 && s->scsi_dma_active[ch]) {
        uint32_t chunk;

        if (s->scsi_dma_count[ch] == 0) {
            /* Current descriptor exhausted, advance chain */
            sgi_hpc3_scsi_dma_advance_chain(s, ch);
            if (!s->scsi_dma_active[ch]) {
                break;
            }
            if (s->scsi_dma_count[ch] == 0) {
                break;
            }
        }

        /* Compute transfer chunk: min of available data, descriptor count, transfer count */
        chunk = MIN(wdc->async_len, s->scsi_dma_count[ch]);
        if (wdc->transfer_count > 0) {
            chunk = MIN(chunk, wdc->transfer_count);
        }
        if (chunk == 0) {
            break;
        }

        trace_sgi_hpc3_scsi_dma(ch, s->scsi_cbp[ch], chunk,
                                s->scsi_dma_to_device[ch] ? 1 : 0);

        if (s->scsi_dma_to_device[ch]) {
            /* Memory → device (write to SCSI) */
            address_space_read(&address_space_memory, s->scsi_cbp[ch],
                               MEMTXATTRS_UNSPECIFIED, wdc->async_buf, chunk);
        } else {
            /* Device → memory (read from SCSI) */
            address_space_write(&address_space_memory, s->scsi_cbp[ch],
                                MEMTXATTRS_UNSPECIFIED, wdc->async_buf, chunk);
        }

        /* Advance pointers */
        s->scsi_cbp[ch] += chunk;
        wdc->async_buf += chunk;
        wdc->async_len -= chunk;
        s->scsi_dma_count[ch] -= chunk;
        total_transferred += chunk;
        if (wdc->transfer_count > 0) {
            wdc->transfer_count -= chunk;
            wd33c93_set_transfer_count(wdc, wdc->transfer_count);
        }

        /* If current descriptor is exhausted, advance chain */
        if (s->scsi_dma_count[ch] == 0) {
            sgi_hpc3_scsi_dma_advance_chain(s, ch);
        }
    }

    /*
     * Drain any remaining zero-count terminal descriptors (e.g., EOX
     * terminators). The DMA loop exits when async_len reaches 0, but the
     * descriptor chain may have a trailing EOX entry with count=0 that
     * still needs to be processed to clear dma_active and ENABLE.
     */
    while (s->scsi_dma_active[ch] && s->scsi_dma_count[ch] == 0) {
        sgi_hpc3_scsi_dma_advance_chain(s, ch);
    }

    /*
     * If WD33C93 buffer is fully consumed, lower DRQ and ask the
     * SCSI subsystem for more data (or signal completion).
     *
     * When the WD33C93 transfer count reaches 0 but there is still
     * pending data (multi-pass DMA), raise an "unexpected phase"
     * interrupt instead of continuing. The IRIX driver will reprogram
     * TC and issue TRANSFER_INFO to resume.
     *
     * Reference: IRIX wd93.c:2936 — ST_UNEX_SDATA/RDATA handler
     */
    if (wdc->current_req &&
        (wdc->async_len == 0 || wdc->transfer_count == 0)) {
        wd33c93_set_drq(wdc, false);
        s->scsi_drq[ch] = false;

        if (wdc->transfer_count == 0 && wdc->pending_len > 0) {
            /*
             * Multi-pass DMA: TC exhausted but SCSI buffer not fully
             * consumed. Raise "unexpected phase" interrupt so the driver
             * can reprogram TC and DMA descriptors for the next chunk.
             */
            wdc->async_len = 0;
            wdc->async_buf = NULL;
            wdc->aux_status &= ~(ASR_DBR | ASR_CIP | ASR_BSY);

            /* MAME: COMMAND_PHASE_TRANSFER_COUNT = 0x46 (IRIX: PH_DATA) */
            wdc->regs[WD_COMMAND_PHASE] = 0x46;

            /*
             * Status codes are named from the WD33C93 chip's perspective:
             *   UNEX_RDATA (0x48) = chip receiving data = host writing (DATA OUT)
             *   UNEX_SDATA (0x49) = chip sending data   = host reading (DATA IN)
             * IRIX wd93.c:2930 confirms: ST_UNEX_RDATA with !SCDMA_IN (write),
             *                            ST_UNEX_SDATA with SCDMA_IN (read).
             */
            wdc->scsi_status = s->scsi_dma_to_device[ch]
                             ? SCSI_STATUS_UNEX_RDATA    /* write: chip receives */
                             : SCSI_STATUS_UNEX_SDATA;   /* read: chip sends  */

            qemu_log_mask(LOG_UNIMP, "wd33c93: DMA TC=0 with %u bytes "
                          "pending — raising unexpected-phase 0x%02x\n",
                          wdc->pending_len, wdc->scsi_status);

            wdc->aux_status |= ASR_INT;
            qemu_irq_raise(wdc->irq);
        } else {
            if (wdc->async_len > 0) {
                /* Discard leftover data the WD33C93 doesn't need */
                wdc->async_len = 0;
                wdc->async_buf = NULL;
            }
            scsi_req_continue(wdc->current_req);
        }
    }
}

/*
 * DRQ GPIO input handler - called when WD33C93 asserts/deasserts DRQ
 */
static void sgi_hpc3_scsi_drq(void *opaque, int n, int level)
{
    SGIHPC3State *s = SGI_HPC3(opaque);

    s->scsi_drq[n] = !!level;

    if (level && s->scsi_dma_active[n]) {
        sgi_hpc3_scsi_dma_run(s, n);
    }
}

/*
 * Handle CTRL register write for SCSI DMA channel
 */
static void sgi_hpc3_scsi_ctrl_write(SGIHPC3State *s, int ch, uint32_t val)
{
    bool was_active = s->scsi_dma_active[ch];

    if (val & HPC3_DMACTRL_WRMASK) {
        /* WRMASK: update direction/endian flags, preserve enable state */
        s->scsi_ctrl[ch] = val & ~(HPC3_DMACTRL_IRQ | HPC3_DMACTRL_ENABLE |
                                   HPC3_DMACTRL_WRMASK);
        if (was_active) {
            s->scsi_ctrl[ch] |= HPC3_DMACTRL_ENABLE;
        }
    } else {
        /* Normal write: update enable state */
        s->scsi_ctrl[ch] = val & ~(HPC3_DMACTRL_IRQ | HPC3_DMACTRL_WRMASK);
        s->scsi_dma_active[ch] = !!(s->scsi_ctrl[ch] & HPC3_DMACTRL_ENABLE);
    }

    /* Extract direction and endian flags */
    s->scsi_dma_to_device[ch] = !!(s->scsi_ctrl[ch] & HPC3_DMACTRL_DIR);
    s->scsi_dma_big_endian[ch] = !!(s->scsi_ctrl[ch] & HPC3_DMACTRL_ENDIAN);

    /*
     * When ENABLE is written, always (re)fetch the descriptor chain.
     * The kernel programs new descriptors before each SCSI command.
     * Without this, a previous short transfer (residual > 0) leaves
     * dma_active=true with stale descriptors, causing the next command's
     * data to go to the wrong address.
     */
    if (s->scsi_dma_active[ch]) {
        sgi_hpc3_scsi_dma_fetch_chain(s, ch);

        /* If DRQ already asserted, start DMA immediately */
        if (s->scsi_drq[ch]) {
            sgi_hpc3_scsi_dma_run(s, ch);
        }
    }

    /* FLUSH bit: drain FIFO and mark DMA complete (MAME: scsi_fifo_flush) */
    if (val & HPC3_DMACTRL_FLUSH) {
        s->scsi_dma_count[ch] = 0;
        s->scsi_dma_active[ch] = false;
        s->scsi_ctrl[ch] &= ~(HPC3_DMACTRL_ENABLE | HPC3_DMACTRL_FLUSH);
    }
}

/*
 * Get current counter value for a PIT channel
 */
static uint16_t sgi_hpc3_pit_get_count(SGIHPC3State *s, int channel)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - s->pit_load_time[channel];
    uint64_t elapsed_ticks = elapsed_ns / PIT_NS_PER_TICK;
    uint32_t count = s->pit_count[channel];

    if (count == 0) {
        count = 0x10000;  /* 0 means 65536 */
    }

    /* Counter counts down from reload value */
    uint32_t current = count - (elapsed_ticks % count);
    if (current == 0 || current > 0xffff) {
        current = count & 0xffff;
    }

    return current & 0xffff;
}

/*
 * Timer callback for PIT channel 0 (fires to CPU IP4 - scheduling clock)
 *
 * MAME Reference (ioc2.cpp:210-226):
 * Timer interrupts go directly to CPU IRQ lines, bypassing INT3 masking.
 * This allows the PROM to use timer delays before configuring INT3 masks.
 */
static void sgi_hpc3_pit_timer0_cb(void *opaque)
{
    SGIHPC3State *s = SGI_HPC3(opaque);

    trace_sgi_hpc3_pit(0, s->pit_count[0]);
    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: Timer0 fired, map_status=0x%02x, "
                  "map_mask0=0x%02x, local0_stat=0x%02x, local0_mask=0x%02x\n",
                  s->int3_map_status, s->int3_map_mask0,
                  s->int3_local0_stat, s->int3_local0_mask);

    /*
     * Set timer pending and update CPU IRQ output.
     *
     * Per MAME ioc2.cpp:210-226, PIT timer interrupts go DIRECTLY to
     * dedicated CPU IRQ lines (Timer0→IP4, Timer1→IP5), bypassing the
     * INT3 Local0/Local1 interrupt cascade entirely.  Do NOT set bits
     * in int3_map_status — that would cause a spurious mapped interrupt
     * cascade through MAPPABLE0/MAPPABLE1 when the kernel enables
     * map_mask0/map_mask1 bits.
     */
    s->timer_pending[0] = true;

    sgi_hpc3_update_irq(s);

    /* Reschedule timer if count is non-zero */
    if (s->pit_count[0] > 0) {
        uint64_t period_ns = (uint64_t)s->pit_count[0] * PIT_NS_PER_TICK;
        timer_mod(s->pit_timer[0],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
    }
}

/*
 * Timer callback for PIT channel 1 (fires to CPU IP5 - profiling clock)
 *
 * MAME Reference (ioc2.cpp:210-226):
 * Timer interrupts go directly to CPU IRQ lines, bypassing INT3 masking.
 */
static void sgi_hpc3_pit_timer1_cb(void *opaque)
{
    SGIHPC3State *s = SGI_HPC3(opaque);

    trace_sgi_hpc3_pit(1, s->pit_count[1]);
    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: Timer1 fired, map_status=0x%02x, "
                  "map_mask1=0x%02x, local1_stat=0x%02x, local1_mask=0x%02x\n",
                  s->int3_map_status, s->int3_map_mask1,
                  s->int3_local1_stat, s->int3_local1_mask);

    /* Direct to CPU IP5, bypass INT3 cascade (see timer0 comment above) */
    s->timer_pending[1] = true;

    sgi_hpc3_update_irq(s);

    /* Reschedule timer if count is non-zero */
    if (s->pit_count[1] > 0) {
        uint64_t period_ns = (uint64_t)s->pit_count[1] * PIT_NS_PER_TICK;
        timer_mod(s->pit_timer[1],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
    }
}

/*
 * Start/restart a PIT timer
 */
static void sgi_hpc3_pit_arm_timer(SGIHPC3State *s, int channel)
{
    if (channel > 1) {
        return;  /* Channel 2 is master clock, doesn't generate interrupts */
    }

    uint32_t count = s->pit_count[channel];
    if (count == 0) {
        count = 0x10000;  /* 0 means 65536 */
    }

    s->pit_load_time[channel] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    uint64_t period_ns = (uint64_t)count * PIT_NS_PER_TICK;

    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: Arming timer%d, count=%d, "
                  "period=%" PRIu64 " ns\n", channel, count, period_ns);

    timer_mod(s->pit_timer[channel],
              s->pit_load_time[channel] + period_ns);
}

/*
 * Read from PIT counter register
 */
static uint8_t sgi_hpc3_pit_read(SGIHPC3State *s, int channel)
{
    uint16_t val;

    if (s->pit_latched[channel]) {
        val = s->pit_latch[channel];
    } else {
        val = sgi_hpc3_pit_get_count(s, channel);
    }

    /* Handle 16-bit read in two 8-bit parts based on control word */
    uint8_t rw_mode = (s->pit_control[channel] >> 4) & 0x03;
    uint8_t result;

    switch (rw_mode) {
    case 1:  /* LSB only */
        result = val & 0xff;
        s->pit_latched[channel] = false;
        break;
    case 2:  /* MSB only */
        result = (val >> 8) & 0xff;
        s->pit_latched[channel] = false;
        break;
    case 3:  /* LSB then MSB */
        if (s->pit_rw_state[channel] == 0) {
            result = val & 0xff;
            s->pit_rw_state[channel] = 1;
        } else {
            result = (val >> 8) & 0xff;
            s->pit_rw_state[channel] = 0;
            s->pit_latched[channel] = false;
        }
        break;
    default:
        result = 0;
        break;
    }

    return result;
}

/*
 * Write to PIT counter register
 */
static void sgi_hpc3_pit_write(SGIHPC3State *s, int channel, uint8_t val)
{
    uint8_t rw_mode = (s->pit_control[channel] >> 4) & 0x03;

    switch (rw_mode) {
    case 1:  /* LSB only */
        s->pit_count[channel] = val;
        sgi_hpc3_pit_arm_timer(s, channel);
        break;
    case 2:  /* MSB only */
        s->pit_count[channel] = val << 8;
        sgi_hpc3_pit_arm_timer(s, channel);
        break;
    case 3:  /* LSB then MSB */
        if (s->pit_rw_state[channel] == 0) {
            s->pit_count[channel] = (s->pit_count[channel] & 0xff00) | val;
            s->pit_rw_state[channel] = 1;
        } else {
            s->pit_count[channel] = (s->pit_count[channel] & 0x00ff) | (val << 8);
            s->pit_rw_state[channel] = 0;
            sgi_hpc3_pit_arm_timer(s, channel);
        }
        break;
    }
}

/*
 * Write to PIT control register
 */
static void sgi_hpc3_pit_control_write(SGIHPC3State *s, uint8_t val)
{
    int channel = (val >> 6) & 0x03;

    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: PIT control write 0x%02x "
                  "(ch=%d rw=%d mode=%d)\n",
                  val, channel, (val >> 4) & 3, (val >> 1) & 7);

    if (channel == 3) {
        /* Read-back command - not implemented */
        return;
    }

    uint8_t rw_mode = (val >> 4) & 0x03;

    if (rw_mode == 0) {
        /* Counter latch command */
        s->pit_latch[channel] = sgi_hpc3_pit_get_count(s, channel);
        s->pit_latched[channel] = true;
        s->pit_rw_state[channel] = 0;
    } else {
        /* Control word */
        s->pit_control[channel] = val;
        s->pit_rw_state[channel] = 0;
        s->pit_latched[channel] = false;
    }
}

/*
 * 8042 PS/2 Keyboard Controller emulation
 *
 * The SGI Indy has an Intel 8042-compatible keyboard controller embedded
 * in the IOC2 ASIC. The PROM diagnostic tests this controller to verify
 * the keyboard subsystem is functional.
 *
 * Status register bits:
 *   Bit 0: Output buffer full (keyboard data available)
 *   Bit 1: Input buffer full (waiting for command)
 *   Bit 2: System flag (POST complete)
 *   Bit 3: Command/Data flag (0=data, 1=command written to 0x64)
 *   Bit 4: Keyboard enabled
 *   Bit 5: Transmit timeout error
 *   Bit 6: Receive timeout error
 *   Bit 7: Parity error
 */

/* 8042 controller commands */
#define KBD_CMD_READ_CTRL      0x20  /* Read controller command byte */
#define KBD_CMD_WRITE_CTRL     0x60  /* Write controller command byte */
#define KBD_CMD_DISABLE_MOUSE  0xA7  /* Disable mouse port */
#define KBD_CMD_ENABLE_MOUSE   0xA8  /* Enable mouse port */
#define KBD_CMD_TEST_MOUSE     0xA9  /* Test mouse port */
#define KBD_CMD_SELF_TEST      0xAA  /* Controller self-test */
#define KBD_CMD_IFACE_TEST     0xAB  /* Interface test */
#define KBD_CMD_DISABLE_KBD    0xAD  /* Disable keyboard */
#define KBD_CMD_ENABLE_KBD     0xAE  /* Enable keyboard */
#define KBD_CMD_READ_INPUT     0xC0  /* Read input port */
#define KBD_CMD_READ_OUTPUT    0xD0  /* Read output port */
#define KBD_CMD_WRITE_OUTPUT   0xD1  /* Write output port */
#define KBD_CMD_WRITE_KBD      0xD2  /* Write to keyboard output buffer */
#define KBD_CMD_WRITE_MOUSE    0xD3  /* Write to mouse output buffer */
#define KBD_CMD_WRITE_MOUSE_P  0xD4  /* Write to mouse */

/* 8042 responses (for controller-generated replies only) */
#define KBD_RESP_SELF_TEST_OK  0x55  /* Self-test passed */
#define KBD_RESP_IFACE_OK      0x00  /* Interface test passed */

/* Status register bits */
#define KBD_STAT_OBF           0x01  /* Output buffer full */
#define KBD_STAT_IBF           0x02  /* Input buffer full */
#define KBD_STAT_SYS           0x04  /* System flag */
#define KBD_STAT_CMD           0x08  /* Command/data flag */
#define KBD_STAT_UNLOCKED      0x10  /* Keyboard unlocked */
#define KBD_STAT_MOUSE_OBF     0x20  /* Mouse output buffer full */
#define KBD_STAT_TIMEOUT       0x40  /* General timeout */
#define KBD_STAT_PARITY        0x80  /* Parity error */

/* Controller command byte bits */
#define KBD_MODE_KBD_INT       0x01  /* Keyboard data generates IRQ */
#define KBD_MODE_MOUSE_INT     0x02  /* Mouse data generates IRQ */
#define KBD_MODE_SYS           0x04  /* System flag */
#define KBD_MODE_NO_KEYLOCK    0x08  /* Override keyboard lock */
#define KBD_MODE_DISABLE_KBD   0x10  /* Disable keyboard interface */
#define KBD_MODE_DISABLE_MOUSE 0x20  /* Disable mouse interface */
#define KBD_MODE_KCC           0x40  /* Scan code conversion */

/* INT3 mapped interrupt bit for keyboard/mouse (from MAME ioc2.h) */
#define INT3_MAP_KBDMS         0x10

/*
 * Queue controller-generated data into the keyboard output path.
 * This is for 8042-level responses (self-test, interface test, etc.)
 * that don't come from the PS/2 devices themselves.
 */
static void sgi_hpc3_kbd_queue_ctrl(SGIHPC3State *s, uint8_t data)
{
    ps2_queue(PS2_DEVICE(&s->ps2kbd), data);
}

/*
 * Build 8042 status register dynamically from PS/2 device queue state.
 *
 * The IRIX pckm interrupt handler checks (status & (SR_MSFULL|SR_OBF)):
 *   0x21 → mouse data pending → read from ps2mouse
 *   0x01 → keyboard data pending → read from ps2kbd
 */
static uint8_t sgi_hpc3_kbd_status(SGIHPC3State *s)
{
    uint8_t status = KBD_STAT_SYS | KBD_STAT_UNLOCKED;

    if (!(s->kbd_cmd_byte & KBD_MODE_DISABLE_KBD)) {
        status |= KBD_STAT_UNLOCKED;
    }

    /*
     * Check PS/2 device queues.  Keyboard data takes priority over mouse
     * data, matching standard 8042 behavior.  However, we track which
     * device has pending data so the read handler returns from the
     * correct queue.
     */
    if (!ps2_queue_empty(PS2_DEVICE(&s->ps2kbd))) {
        status |= KBD_STAT_OBF;
        s->kbd_pending_source = 0;  /* keyboard */
    } else if (!ps2_queue_empty(PS2_DEVICE(&s->ps2mouse))) {
        status |= KBD_STAT_OBF | KBD_STAT_MOUSE_OBF;
        s->kbd_pending_source = 1;  /* mouse */
    }

    trace_sgi_hpc3_kbd_status(status);
    return status;
}

/*
 * Re-evaluate INT3_MAP_KBDMS based on PS/2 IRQ levels AND the 8042
 * command byte interrupt enable bits.  On real hardware, IRQ1/IRQ12
 * are only asserted when both OBF and the enable bit are set.
 *
 * IRIX pckm_reinit_lock() clears these bits during polled init with
 * pckm_mutex held. Without gating, we'd fire an interrupt that causes
 * pckm_intr to deadlock on the mutex.
 */
static void sgi_hpc3_kbd_update_map_irq(SGIHPC3State *s)
{
    bool kbd_active = s->kbd_irq_level &&
                      (s->kbd_cmd_byte & KBD_MODE_KBD_INT);
    bool mouse_active = s->mouse_irq_level &&
                        (s->kbd_cmd_byte & KBD_MODE_MOUSE_INT);

    if (kbd_active || mouse_active) {
        s->int3_map_status |= INT3_MAP_KBDMS;
    } else {
        s->int3_map_status &= ~INT3_MAP_KBDMS;
    }
    trace_sgi_hpc3_kbd_irq(s->kbd_irq_level, s->mouse_irq_level,
                           s->int3_map_status);
    sgi_hpc3_update_irq(s);
}

/*
 * PS/2 keyboard IRQ callback — called when the PS/2 keyboard device
 * has data available in its queue (or when the queue drains).
 */
static void sgi_hpc3_ps2_kbd_irq(void *opaque, int n, int level)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    s->kbd_irq_level = level;
    sgi_hpc3_kbd_update_map_irq(s);
}

/*
 * PS/2 mouse IRQ callback — same as keyboard but for mouse data.
 */
static void sgi_hpc3_ps2_mouse_irq(void *opaque, int n, int level)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    s->mouse_irq_level = level;
    sgi_hpc3_kbd_update_map_irq(s);
}

/*
 * Read data from the 8042 output buffer.
 * Returns data from the appropriate PS/2 device queue based on
 * which device has pending data (keyboard takes priority).
 */
static uint8_t sgi_hpc3_kbd_read_data(SGIHPC3State *s)
{
    uint8_t val;

    if (!ps2_queue_empty(PS2_DEVICE(&s->ps2kbd))) {
        val = ps2_read_data(PS2_DEVICE(&s->ps2kbd));
        trace_sgi_hpc3_kbd_data_read(val, 0);
    } else if (!ps2_queue_empty(PS2_DEVICE(&s->ps2mouse))) {
        val = ps2_read_data(PS2_DEVICE(&s->ps2mouse));
        trace_sgi_hpc3_kbd_data_read(val, 1);
    } else {
        val = 0;
    }
    return val;
}

/*
 * Process 8042 keyboard controller command (written to command port 0x64)
 */
static void sgi_hpc3_kbd_command(SGIHPC3State *s, uint8_t cmd)
{
    trace_sgi_hpc3_kbd_cmd(cmd);
    s->kbd_cmd = cmd;

    switch (cmd) {
    case 0x00:  /* NOP - some PROMs send this */
        break;

    case KBD_CMD_SELF_TEST:  /* 0xAA - Controller self-test */
        sgi_hpc3_kbd_queue_ctrl(s, KBD_RESP_SELF_TEST_OK);
        break;

    case KBD_CMD_IFACE_TEST:  /* 0xAB - Keyboard interface test */
        sgi_hpc3_kbd_queue_ctrl(s, KBD_RESP_IFACE_OK);
        break;

    case KBD_CMD_TEST_MOUSE:  /* 0xA9 - Mouse interface test */
        sgi_hpc3_kbd_queue_ctrl(s, KBD_RESP_IFACE_OK);
        break;

    case KBD_CMD_DISABLE_KBD:  /* 0xAD - Disable keyboard */
        s->kbd_cmd_byte |= KBD_MODE_DISABLE_KBD;
        break;

    case KBD_CMD_ENABLE_KBD:  /* 0xAE - Enable keyboard */
        s->kbd_cmd_byte &= ~KBD_MODE_DISABLE_KBD;
        break;

    case KBD_CMD_DISABLE_MOUSE:  /* 0xA7 - Disable mouse */
        s->kbd_cmd_byte |= KBD_MODE_DISABLE_MOUSE;
        break;

    case KBD_CMD_ENABLE_MOUSE:  /* 0xA8 - Enable mouse */
        s->kbd_cmd_byte &= ~KBD_MODE_DISABLE_MOUSE;
        break;

    case KBD_CMD_READ_CTRL:  /* 0x20 - Read controller command byte */
        sgi_hpc3_kbd_queue_ctrl(s, s->kbd_cmd_byte);
        break;

    case KBD_CMD_WRITE_CTRL:  /* 0x60 - Next data byte is command byte */
        /* Wait for data write */
        break;

    case KBD_CMD_READ_INPUT:  /* 0xC0 - Read input port */
        sgi_hpc3_kbd_queue_ctrl(s, 0x80);
        break;

    case KBD_CMD_READ_OUTPUT:  /* 0xD0 - Read output port */
        sgi_hpc3_kbd_queue_ctrl(s, 0x02);
        break;

    case KBD_CMD_WRITE_OUTPUT:  /* 0xD1 - Write output port */
    case KBD_CMD_WRITE_KBD:     /* 0xD2 - Write to keyboard buffer */
    case KBD_CMD_WRITE_MOUSE:   /* 0xD3 - Write to mouse buffer */
    case KBD_CMD_WRITE_MOUSE_P: /* 0xD4 - Write to mouse */
        /* These expect a data byte to follow */
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "sgi_hpc3: unimplemented 8042 command 0x%02x\n", cmd);
        break;
    }
}

/*
 * Process data write to the 8042 data port (0x60).
 *
 * The meaning depends on the last controller command:
 *   - After 0x60: writing the controller command byte
 *   - After 0xD4: forward to PS/2 mouse device
 *   - After 0xD2: queue into keyboard output buffer
 *   - No pending command: forward to PS/2 keyboard device
 */
static void sgi_hpc3_kbd_data_write(SGIHPC3State *s, uint8_t data)
{
    switch (s->kbd_cmd) {
    case KBD_CMD_WRITE_CTRL:  /* 0x60 - Writing command byte */
        s->kbd_cmd_byte = data;
        ps2_keyboard_set_translation(&s->ps2kbd.parent_obj,
                                     (s->kbd_cmd_byte & KBD_MODE_KCC) != 0);
        s->kbd_cmd = 0;
        sgi_hpc3_kbd_update_map_irq(s);  /* fire pending IRQ if now enabled */
        break;

    case KBD_CMD_WRITE_OUTPUT:  /* 0xD1 - Writing to output port */
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_KBD:  /* 0xD2 - Write to keyboard output buffer */
        ps2_queue(PS2_DEVICE(&s->ps2kbd), data);
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_MOUSE:  /* 0xD3 - Write to mouse output buffer */
        ps2_queue(PS2_DEVICE(&s->ps2mouse), data);
        s->kbd_cmd = 0;
        break;

    case KBD_CMD_WRITE_MOUSE_P:  /* 0xD4 - Write to mouse device */
        ps2_write_mouse(&s->ps2mouse, data);
        s->kbd_cmd = 0;
        break;

    default:
        /* No pending controller command — send directly to keyboard */
        ps2_write_keyboard(&s->ps2kbd.parent_obj, data);
        break;
    }
}

/* Update RTC time registers from current time + offset */
static void sgi_hpc3_rtc_update_time(SGIHPC3State *s)
{
    struct tm tm;

    qemu_get_timedate(&tm, s->rtc_time_offset);

    s->bbram[RTC_SECONDS] = bin_to_bcd(tm.tm_sec);
    s->bbram[RTC_MINUTES] = bin_to_bcd(tm.tm_min);
    s->bbram[RTC_HOURS] = bin_to_bcd(tm.tm_hour);
    s->bbram[RTC_DAY_OF_WEEK] = tm.tm_wday + 1;
    s->bbram[RTC_DATE] = bin_to_bcd(tm.tm_mday);
    s->bbram[RTC_MONTH] = bin_to_bcd(tm.tm_mon + 1);
    s->bbram[RTC_YEAR] = bin_to_bcd(tm.tm_year % 100);
}

/* Read RTC register - returns stored value, updating time if TE set */
static uint8_t sgi_hpc3_rtc_read(SGIHPC3State *s, int reg)
{
    /* Update time registers if TE (Transfer Enable) is set */
    if (s->rtc_command & 0x80) {
        sgi_hpc3_rtc_update_time(s);
    }

    switch (reg) {
    case RTC_HUNDREDTHS:
        /* Use realtime clock for sub-second precision - virtual clock may not advance in tight loops */
        return bin_to_bcd((qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 10000000) % 100);
    case RTC_COMMAND:
        return s->rtc_command | 0x80;  /* Oscillator always running */
    default:
        return s->bbram[reg];
    }
}

/*
 * NVRAM variable table layout (256 bytes at bbram[0x40]-bbram[0x13f])
 *
 * The PROM stores environment variables in a 256-byte table within the
 * DS1386 battery-backed RAM. The table starts at bbram offset 0x40
 * (HPC3 address 0x60100 with 4-byte register spacing).
 *
 * From IP22nvram.h / dallas.c:
 *   Byte 0: Checksum (XOR+rotate, seed 0xa5)
 *   Byte 1: Revision (bits[5:0] = rev: 8 for IP22/IP24, 9 for IP28)
 *   Bytes 2-249: Environment variables (single-byte values or strings)
 *   Bytes 250-255 (NVOFF_ENET): 6-byte Ethernet MAC (write-protected)
 */
#define NVRAM_TABLE_BASE  0x40   /* bbram offset for NVRAM variable table */
#define NVRAM_TABLE_SIZE  256    /* Size of NVRAM variable table */

/* Offsets within the 256-byte NVRAM variable table (from IP22nvram.h) */
#define NVOFF_CHECKSUM    0
#define NVOFF_REVISION    1
#define NVOFF_CONSOLE     2      /* 2 bytes */
#define NVOFF_SYSPART     4      /* 48 bytes */
#define NVOFF_OSLOADER    52     /* 18 bytes */
#define NVOFF_OSFILE      70     /* 28 bytes */
#define NVOFF_OSOPTS      98     /* 12 bytes */
#define NVOFF_LBAUD       116    /* 5 bytes - "dbaud" variable */
#define NVOFF_DISKLESS    121    /* 1 byte */
#define NVOFF_TIMEZONE    122    /* 8 bytes */
#define NVOFF_OSPART      130    /* 48 bytes */
#define NVOFF_AUTOLOAD    178    /* 1 byte */
#define NVOFF_NETADDR     181    /* 4 bytes (binary IP) */
#define NVOFF_NOKBD       185    /* 1 byte */
#define NVOFF_VOLUME      232    /* 3 bytes */
#define NVOFF_SCSIHOSTID  235    /* 1 byte */
#define NVOFF_SGILOGO     236    /* 1 byte */
#define NVOFF_NOGUI       237    /* 1 byte */
#define NVOFF_AUTOPOWER   239    /* 1 byte */
#define NVOFF_MONITOR     240    /* 1 byte */
#define NVOFF_ENET        250    /* 6 bytes (write-protected) */

/*
 * Compute NVRAM checksum per dallas.c:nvchecksum()
 *
 * Algorithm: XOR each byte with running checksum, rotating left after
 * each odd-indexed byte. Seed is 0xa5, byte 0 (checksum itself) is skipped.
 */
static uint8_t sgi_hpc3_nvram_checksum(uint8_t *table, int len)
{
    int8_t checksum = (int8_t)0xa5;

    for (int i = 0; i < len; i++) {
        if (i != 0) {
            checksum ^= (int8_t)table[i];
        }
        if (i & 1) {
            checksum = (checksum << 1) | ((uint8_t)checksum >> 7);
        }
    }
    return (uint8_t)checksum;
}

/*
 * Initialize NVRAM with sensible defaults when no backing file exists.
 * This avoids the PROM's "NVRAM checksum is incorrect: reinitializing"
 * message and the Full House "Could not set NVRAM variable" errors.
 */
static void sgi_hpc3_nvram_init_defaults(SGIHPC3State *s)
{
    uint8_t *table = &s->bbram[NVRAM_TABLE_BASE];
    uint8_t mac_random[3];

    memset(table, 0, NVRAM_TABLE_SIZE);

    /* Revision: 8 for IP22/IP24, 9 for IP26/IP28 (set via nvram-rev property) */
    table[NVOFF_REVISION] = s->nvram_rev;

    /* Generate random bytes for MAC uniqueness across instances */
    qemu_guest_getrandom_nofail(mac_random, sizeof(mac_random));

    /* console = "d" (serial) - since Newport graphics isn't functional yet */
    table[NVOFF_CONSOLE] = 'd';

    /* dbaud = "9600" (serial baud rate) */
    memcpy(&table[NVOFF_LBAUD], "9600", 4);

    /* monitor = "h" (1280x1024@60Hz) - ready for when graphics works */
    table[NVOFF_MONITOR] = 'h';

    /* timezone = "PST8PDT" */
    memcpy(&table[NVOFF_TIMEZONE], "PST8PDT", 7);

    /* diskless = "0" */
    table[NVOFF_DISKLESS] = '0';

    /* volume = "80" */
    table[NVOFF_VOLUME] = '8';
    table[NVOFF_VOLUME + 1] = '0';

    /* sgilogo = "y" */
    table[NVOFF_SGILOGO] = 'y';

    /* autopower = "y" */
    table[NVOFF_AUTOPOWER] = 'y';

    /* scsihostid = "0" */
    table[NVOFF_SCSIHOSTID] = '0';

    /* autoload = controlled by property (default 'Y') */
    table[NVOFF_AUTOLOAD] = s->autoload ? 'Y' : 'N';

    /* Ethernet MAC address: SGI OUI 08:00:69 + 3 random bytes */
    table[NVOFF_ENET + 0] = 0x08;
    table[NVOFF_ENET + 1] = 0x00;
    table[NVOFF_ENET + 2] = 0x69;
    table[NVOFF_ENET + 3] = mac_random[0];
    table[NVOFF_ENET + 4] = mac_random[1];
    table[NVOFF_ENET + 5] = mac_random[2];

    /* Compute and set checksum */
    table[NVOFF_CHECKSUM] = sgi_hpc3_nvram_checksum(table, NVRAM_TABLE_SIZE);

    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: initialized NVRAM defaults "
                  "(rev=%d, autoload=%c, eaddr=%02x:%02x:%02x:%02x:%02x:%02x)\n",
                  table[NVOFF_REVISION],
                  table[NVOFF_AUTOLOAD],
                  table[NVOFF_ENET], table[NVOFF_ENET + 1],
                  table[NVOFF_ENET + 2], table[NVOFF_ENET + 3],
                  table[NVOFF_ENET + 4], table[NVOFF_ENET + 5]);
}

/*
 * Initialize 93CS56 serial EEPROM with NVRAM defaults.
 * Used by Full House (Indigo2) machines which access NVRAM via bit-banged
 * serial EEPROM rather than memory-mapped BBRAM.
 *
 * The EEPROM stores 128 × 16-bit words. NVRAM layout is 256 bytes with
 * big-endian byte order within each word: byte 0 = high byte of word 0.
 */
static void sgi_hpc3_eeprom_init_defaults(SGIHPC3State *s)
{
    uint8_t table[NVRAM_TABLE_SIZE];
    uint16_t *data;
    uint8_t mac_random[3];

    memset(table, 0, NVRAM_TABLE_SIZE);

    /* Revision: 8 for IP22, 9 for IP26/IP28 (set via nvram-rev property) */
    table[NVOFF_REVISION] = s->nvram_rev;

    /* Generate random bytes for MAC uniqueness across instances */
    qemu_guest_getrandom_nofail(mac_random, sizeof(mac_random));

    /* console = "d" (serial) */
    table[NVOFF_CONSOLE] = 'd';

    /* dbaud = "9600" */
    memcpy(&table[NVOFF_LBAUD], "9600", 4);

    /* monitor = "h" (1280x1024@60Hz) */
    table[NVOFF_MONITOR] = 'h';

    /* timezone = "PST8PDT" */
    memcpy(&table[NVOFF_TIMEZONE], "PST8PDT", 7);

    /* diskless = "0" */
    table[NVOFF_DISKLESS] = '0';

    /* volume = "80" */
    table[NVOFF_VOLUME] = '8';
    table[NVOFF_VOLUME + 1] = '0';

    /* sgilogo = "y" */
    table[NVOFF_SGILOGO] = 'y';

    /* autopower = "y" */
    table[NVOFF_AUTOPOWER] = 'y';

    /* scsihostid = "0" */
    table[NVOFF_SCSIHOSTID] = '0';

    /* autoload = controlled by property (default 'Y') */
    table[NVOFF_AUTOLOAD] = s->autoload ? 'Y' : 'N';

    /* Ethernet MAC address: SGI OUI 08:00:69 + 3 random bytes */
    table[NVOFF_ENET + 0] = 0x08;
    table[NVOFF_ENET + 1] = 0x00;
    table[NVOFF_ENET + 2] = 0x69;
    table[NVOFF_ENET + 3] = mac_random[0];
    table[NVOFF_ENET + 4] = mac_random[1];
    table[NVOFF_ENET + 5] = mac_random[2];

    /* Compute and set checksum */
    table[NVOFF_CHECKSUM] = sgi_hpc3_nvram_checksum(table, NVRAM_TABLE_SIZE);

    /* Pack byte table into 16-bit words (big-endian byte order) */
    data = eeprom93xx_data(s->serial_eeprom);
    for (int i = 0; i < 128; i++) {
        data[i] = (table[i * 2] << 8) | table[i * 2 + 1];
    }

    qemu_log_mask(LOG_UNIMP, "sgi_hpc3: initialized serial EEPROM defaults "
                  "(rev=%d, autoload=%c, eaddr=%02x:%02x:%02x:%02x:%02x:%02x)\n",
                  table[NVOFF_REVISION],
                  table[NVOFF_AUTOLOAD],
                  table[NVOFF_ENET], table[NVOFF_ENET + 1],
                  table[NVOFF_ENET + 2], table[NVOFF_ENET + 3],
                  table[NVOFF_ENET + 4], table[NVOFF_ENET + 5]);
}

/*
 * Sync a single byte of BBRAM to the backing file.
 */
static void sgi_hpc3_nvram_sync(SGIHPC3State *s, int reg)
{
    if (!s->nvram_filename) {
        return;
    }

    FILE *f = fopen(s->nvram_filename, "r+b");
    if (!f) {
        f = fopen(s->nvram_filename, "w+b");
    }
    if (f) {
        /* Ensure file is large enough */
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        if (fsize < (long)sizeof(s->bbram)) {
            /* Write entire bbram to extend file */
            fseek(f, 0, SEEK_SET);
            fwrite(s->bbram, sizeof(s->bbram), 1, f);
        } else {
            fseek(f, reg, SEEK_SET);
            fputc(s->bbram[reg], f);
        }
        fflush(f);
        fclose(f);
    }
}

/*
 * Update CPU IRQ lines based on interrupt status and mask registers.
 *
 * Interrupt routing (per MAME ioc2.cpp and IRIX IP22.c c0vec_tbl[]):
 *   INT3 Local0 (SCSI, ethernet, etc.) → IP2 (cpu_irq[0])
 *   INT3 Local1 (panel, DMA, etc.)     → IP3 (cpu_irq[1])
 *   PIT Timer 0 (scheduling clock)     → IP4 (timer_irq[0])
 *   PIT Timer 1 (profiling clock)      → IP5 (timer_irq[1])
 *
 * Timer interrupts bypass INT3 masking and go to dedicated CPU IRQ lines
 * (MAME ioc2.cpp:210-226: timer0_int → input line 2 = IP4,
 *  timer1_int → input line 3 = IP5).
 */
static void sgi_hpc3_update_irq(SGIHPC3State *s)
{
    /*
     * Mapped interrupt cascade (per MAME ioc2.cpp:268-284):
     * If any bit in (map_status & map_mask0) is set, assert MAPPABLE0
     * in local0_stat; otherwise clear it.  Same for map_mask1 → local1.
     *
     * This ensures the cascade is always consistent before we evaluate
     * the local0/local1 pending state.
     */
    if (s->int3_map_status & s->int3_map_mask0) {
        s->int3_local0_stat |= INT3_LOCAL0_MAPPABLE0;
    } else {
        s->int3_local0_stat &= ~INT3_LOCAL0_MAPPABLE0;
    }
    if (s->int3_map_status & s->int3_map_mask1) {
        s->int3_local1_stat |= INT3_LOCAL1_LCL0;  /* MAPPABLE1 cascade */
    } else {
        s->int3_local1_stat &= ~INT3_LOCAL1_LCL0;
    }

    /*
     * INT3 Local0/Local1 → IP2/IP3 (masked interrupts only)
     *
     * Only bits corresponding to emulated hardware sources should be set
     * in local0/local1_stat.  Clear any spurious bits from unimplemented
     * devices to prevent interrupt storms from stray interrupts that the
     * kernel cannot acknowledge.
     *
     * Local0 sources: SCSI0 (0x02), SCSI1 (0x04), ETHERNET (0x08),
     *                  MAPPABLE0 (0x80)
     * Local1 sources: HPC_DMA (0x10), GIO2/Retrace (0x80),
     *                  MAPPABLE1/LCL0 (0x08)
     */
    s->int3_local0_stat &= (INT3_LOCAL0_SCSI0 | INT3_LOCAL0_SCSI1 |
                             INT3_LOCAL0_ETHERNET | INT3_LOCAL0_MAPPABLE0);
    s->int3_local1_stat &= (INT3_LOCAL1_LCL0 | INT3_LOCAL1_HPC_DMA |
                             INT3_LOCAL1_GIO2);
    int local0_pending = (s->int3_local0_stat & s->int3_local0_mask) ? 1 : 0;
    int local1_pending = (s->int3_local1_stat & s->int3_local1_mask) ? 1 : 0;

    trace_sgi_hpc3_int3(0, s->int3_local0_stat, s->int3_local0_mask);
    trace_sgi_hpc3_int3(1, s->int3_local1_stat, s->int3_local1_mask);

    qemu_set_irq(s->cpu_irq[0], local0_pending);
    qemu_set_irq(s->cpu_irq[1], local1_pending);

    /* Timer0 → IP4, Timer1 → IP5 (bypass INT3 masking, per MAME ioc2.cpp:210-226) */
    qemu_set_irq(s->timer_irq[0], s->timer_pending[0] ? 1 : 0);
    qemu_set_irq(s->timer_irq[1], s->timer_pending[1] ? 1 : 0);
}

/*
 * Handle SCSI interrupt from WD33C93 controllers
 * n=0 for SCSI0, n=1 for SCSI1
 */
static void sgi_hpc3_scsi_irq(void *opaque, int n, int level)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    uint8_t mask = (n == 0) ? INT3_LOCAL0_SCSI0 : INT3_LOCAL0_SCSI1;

    if (level) {
        s->int3_local0_stat |= mask;
    } else {
        s->int3_local0_stat &= ~mask;
    }

    sgi_hpc3_update_irq(s);
}

/*
 * Handle GIO retrace (VBLANK) interrupt from Newport graphics.
 * Routes to INT3_LOCAL1 bit 7 (GIO2/Retrace).
 */
static void sgi_hpc3_gio_retrace_irq(void *opaque, int n, int level)
{
    SGIHPC3State *s = SGI_HPC3(opaque);

    if (level) {
        s->int3_local1_stat |= INT3_LOCAL1_GIO2;
    } else {
        s->int3_local1_stat &= ~INT3_LOCAL1_GIO2;
    }

    sgi_hpc3_update_irq(s);
}

static void sgi_hpc3_scc_update_irq(SGIHPC3State *s);


/*
 * TX interrupt delay in nanoseconds.
 *
 * On real Z85C30, TX Buffer Empty fires within the same clock cycle
 * when TX_INT_ENBL is set and the buffer is empty.  The IRIX driver
 * depends on this: mips_du_start_tx enables TX_INT_ENBL, then du_tx
 * writes one byte, and the TX_IP interrupt fires immediately after
 * to drain the rest of the STREAMS queue.
 *
 * With -icount shift=0, virtual time advances 1ns per instruction.
 * A delay of 0 fires at the next timer check (after the current TB).
 * This is close enough to "immediate" for the driver to work correctly.
 *
 * We previously used 100µs (100000 ns) which was far too long — by the
 * time the timer fired, mips_du_stop_tx had already cleared TX_INT_ENBL,
 * causing TX_IP to be suppressed and the STREAMS queue to never drain.
 */
#define SCC_TX_DELAY_NS  0  /* Fire at next timer check */

/*
 * SCC TX timer callback: fires after a brief delay to simulate
 * the TX buffer becoming empty after transmission.
 *
 * On real Z85C30 hardware, the TX Buffer Empty condition is latched
 * in RR3 and persists until explicitly cleared by a WR0 "Reset TX Int
 * Pending" command (command 5).  It is NOT gated by WR1 TX_INT_ENBL.
 *
 * The SCC INT pin IS gated by WR1, but our sgi_hpc3_scc_update_irq()
 * simplifies this: it asserts INT if MIE is set AND any RR3 bit is
 * active, regardless of WR1 enables.  This means setting TX_IP in RR3
 * will cause du_handle_intr to be called, which reads RR3 and dispatches
 * to du_tx() unconditionally.  This is key to making the STREAMS TX
 * path work: du_tx called from interrupt context (strlock=0) may succeed
 * even when the initial call from mips_du_start_tx (strlock=1) failed.
 */
static void sgi_hpc3_scc_tx_timer_cb(void *opaque)
{
    SGIHPC3State *s = SGI_HPC3(opaque);

    /*
     * Determine which channel this timer belongs to, then set TX_IP.
     * Each channel has its own timer; we identify the channel by
     * comparing the timer pointer.
     *
     * On real Z85C30 hardware, TX_IP in RR3 is gated by WR1 TX_INT_ENBL.
     * If TX_INT_ENBL has been cleared (e.g., by du_init finishing SCC
     * configuration before our timer fires), we suppress the TX_IP to
     * avoid spurious interrupts during early boot.
     */
    for (int ch = 0; ch < 2; ch++) {
        /* Only process the channel whose timer actually fired */
        if (timer_pending(s->scc_tx_timer[ch])) {
            continue;
        }

        uint8_t tx_ip_bit = ch ? 0x02 : 0x10;

        /*
         * Set TX_IP if TX_INT_ENBL is still set.  TX_IP may already
         * be set (from the synchronous assertion in the WR1 handler),
         * in which case this is a no-op.  The timer's main role is to
         * trigger sgi_hpc3_scc_update_irq() to deliver the interrupt
         * after the MMIO write that scheduled us has completed.
         */
        if (s->scc_wr1[ch] & 0x02) {
            s->scc_rr3 |= tx_ip_bit;
        }
    }

    sgi_hpc3_scc_update_irq(s);
}

static void sgi_hpc3_serial_write(SGIHPC3State *s, int port, uint8_t data)
{
    /*
     * The SGI Indy uses serial1 (offset 0x0c/0x0d) as the console port
     * in the PROM. Both ports output to the chardev for now.
     */
    trace_sgi_hpc3_scc_tx(port, data);
    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_write_all(&s->serial, &data, 1);
    }

    /*
     * Schedule a TX Buffer Empty interrupt after a short delay.
     *
     * On real Z85C30 hardware, the TX Buffer Empty condition is latched
     * in RR3 after each byte is transmitted, regardless of WR1 state.
     * TX_IP is only cleared by WR0 "Reset TX Int Pending" command.
     *
     * We always schedule the timer after a byte write (not gated by WR1
     * TX_INT_ENBL), because the IRIX driver sequence is:
     *   mips_du_start_tx: enable WR1 TX_INT_ENBL → du_tx writes byte →
     *   du_tx exits early → mips_du_stop_tx: clear WR1 TX_INT_ENBL
     * All of this happens synchronously before our timer fires.
     * When the timer fires, it sets TX_IP in RR3 unconditionally.
     *
     * Only schedule after du_open enables TX interrupts (WR1 bit 1).
     * During early boot and du_init, firing TX_IP causes the IRIX threaded
     * interrupt handler (LCL_CALL_ISR with THD_OK) to permanently mask
     * the DUART in map_mask0 because the ithread never gets scheduled
     * to unmask it.
     *
     * We only fire TX_IP when the driver has explicitly enabled TX
     * interrupts via WR1 TX_INT_ENBL (bit 1), meaning du_open has run
     * and the STREAMS path is active.
     *
     * We also set TX_IP synchronously in RR3 so that if the interrupt
     * handler is already running (du_handle_intr loop), it sees the
     * new TX_IP immediately on its next RR3 read.
     */
    if (s->scc_wr1[port] & 0x02) {
        uint8_t tx_ip_bit = port ? 0x02 : 0x10;
        s->scc_rr3 |= tx_ip_bit;
        sgi_hpc3_scc_update_irq(s);
        timer_mod(s->scc_tx_timer[port],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCC_TX_DELAY_NS);
    }
}

/*
 * Update SCC (Z85C30) interrupt state.
 *
 * The SCC interrupt output connects to INT3's DUART mapped interrupt
 * (bit 5 = 0x20 in map_status). The mapped interrupt then cascades
 * through the map_mask registers to LOCAL0 or LOCAL1.
 *
 * MAME reference: ioc2.cpp uses LIO_DUART for the SCC interrupt.
 */
#define LIO_DUART_BIT  0x20

static void sgi_hpc3_scc_update_irq(SGIHPC3State *s)
{
    /*
     * SCC asserts INT if MIE (Master Interrupt Enable, WR9 bit 3) is set
     * and any IP bit is active in RR3.
     *
     * The SCC INT output connects to INT3's DUART mapped interrupt
     * (bit 5 = 0x20 in map_status).  The cascade from map_status
     * through map_mask0/map_mask1 to MAPPABLE0/MAPPABLE1 is handled
     * centrally in sgi_hpc3_update_irq().
     */
    if ((s->scc_wr9 & 0x08) && s->scc_rr3) {
        s->int3_map_status |= LIO_DUART_BIT;
    } else {
        s->int3_map_status &= ~LIO_DUART_BIT;
    }

    /* Debug: trace SCC interrupt state changes involving TX */
    if (s->scc_rr3 & 0x12) {  /* Any TX_IP bit set */
        qemu_log_mask(LOG_UNIMP,
            "sgi_hpc3: SCC IRQ update: rr3=0x%02x wr9=0x%02x "
            "map_status=0x%02x map_mask0=0x%02x local0_stat=0x%02x "
            "local0_mask=0x%02x\n",
            s->scc_rr3, s->scc_wr9,
            s->int3_map_status, s->int3_map_mask0,
            s->int3_local0_stat, s->int3_local0_mask);
    }

    sgi_hpc3_update_irq(s);
}

/*
 * Serial chardev receive callbacks.
 * Input from the chardev (e.g., a socket or pty) is queued into the
 * RX FIFO for port 1 (the console port).  The PROM polls RR0 bit 0
 * to check for received characters, then reads them from the DATA
 * register (or RR8 via the CMD register).
 */
static int sgi_hpc3_serial_can_receive(void *opaque)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    /* Accept data for port 1 (console) */
    return SCC_RX_FIFO_SIZE - s->serial_rx_fifo_count[1];
}

static void sgi_hpc3_serial_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    int port = 1;  /* Console port */

    for (int i = 0; i < size; i++) {
        if (s->serial_rx_fifo_count[port] >= SCC_RX_FIFO_SIZE) {
            break;  /* FIFO full, drop remaining */
        }
        trace_sgi_hpc3_scc_rx(port, buf[i]);
        s->serial_rx_fifo[port][s->serial_rx_fifo_head[port]] = buf[i];
        s->serial_rx_fifo_head[port] =
            (s->serial_rx_fifo_head[port] + 1) % SCC_RX_FIFO_SIZE;
        s->serial_rx_fifo_count[port]++;
    }

    /*
     * If RX interrupts are enabled, assert RX interrupt.
     * WR1 bits 4:3: RX interrupt mode (00=disabled, 01=first char,
     *               10=all chars, 11=special condition)
     */
    if ((s->scc_wr1[port] & 0x18) && (s->scc_wr9 & 0x08) &&
        s->serial_rx_fifo_count[port] > 0) {
        if (port == 1) {
            s->scc_rr3 |= 0x04;  /* Ch B RX Char Available IP */
        } else {
            s->scc_rr3 |= 0x20;  /* Ch A RX Char Available IP */
        }
        sgi_hpc3_scc_update_irq(s);
    }
}

static void sgi_hpc3_serial_event(void *opaque, QEMUChrEvent event)
{
    /* No special handling needed */
}

/*
 * ========================================================================
 * Ethernet (Seeq 80C03 EDLC via HPC3 DMA)
 * ========================================================================
 */

/*
 * Raise ethernet interrupt: set MISC_INT, local0 ethernet bit, update IRQ.
 * Per MAME: only raise if MISC_INT is not already set.
 */
static void sgi_hpc3_enet_raise_irq(SGIHPC3State *s)
{
    if (!(s->enet_misc & HPC3_ENET_MISC_INT)) {
        s->enet_misc |= HPC3_ENET_MISC_INT;
        s->int3_local0_stat |= INT3_LOCAL0_ETHERNET;
        sgi_hpc3_update_irq(s);
        trace_sgi_hpc3_enet_irq(s->enet_misc, s->enet_rx_ctrl, s->enet_tx_ctrl);
    }
}

/*
 * Reset Seeq and ethernet DMA state.
 * Called on ENET_MISC reset bit or device reset.
 */
static void sgi_hpc3_enet_reset(SGIHPC3State *s)
{
    memset(s->seeq_station_addr, 0, sizeof(s->seeq_station_addr));
    s->seeq_rx_cmd = 0;
    s->seeq_tx_cmd = 0;
    s->seeq_rx_status = SEEQ_RXS_OLD;  /* Start with OLD set (no pending) */
    s->seeq_tx_status = SEEQ_TXS_OLD;

    s->enet_rx_cbp = 0;
    s->enet_rx_nbdp = 0;
    s->enet_rx_bc = 0;
    s->enet_rx_ctrl = 0;
    s->enet_tx_cbp = 0;
    s->enet_tx_nbdp = 0;
    s->enet_tx_bc = 0;
    s->enet_tx_ctrl = 0;
    s->enet_misc = HPC3_ENET_MISC_RESET;  /* Seeq held in reset */
    s->enet_dmacfg = 0;
    s->enet_piocfg = 0;
    s->enet_rx_cbdp = 0;
    s->enet_tx_cpfbdp = 0;
    s->enet_tx_ppfbdp = 0;
}

/*
 * TX DMA: Walk the TX descriptor chain, assemble packet, and send.
 * Called when IRIX writes TXC_CA to HPC3_ENET_TX_CTRL.
 */
static void sgi_hpc3_enet_tx(SGIHPC3State *s)
{
    uint8_t packet_buf[ENET_MAX_PACKET];
    int packet_len = 0;
    uint32_t desc_addr;
    uint32_t cbp, bc, nbdp;
    bool xie = false;

    if (!(s->enet_tx_ctrl & HPC3_ENET_TXC_CA)) {
        return;
    }

    /* Save first buffer descriptor pointer for TX done writeback */
    s->enet_tx_cpfbdp = s->enet_tx_nbdp;
    desc_addr = HPC3_DMA_ADDR(s->enet_tx_nbdp);

    while (desc_addr) {
        /* Fetch 12-byte descriptor: {cbp, bc, nbdp} */
        cbp = address_space_ldl_be(&address_space_memory, desc_addr,
                                   MEMTXATTRS_UNSPECIFIED, NULL);
        bc = address_space_ldl_be(&address_space_memory, desc_addr + 4,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
        nbdp = address_space_ldl_be(&address_space_memory, desc_addr + 8,
                                    MEMTXATTRS_UNSPECIFIED, NULL);

        trace_sgi_hpc3_enet_tx(desc_addr, cbp, bc);

        int count = bc & HPC3_ENET_BC_COUNT;
        if (count > 0 && (packet_len + count) <= ENET_MAX_PACKET) {
            address_space_read(&address_space_memory, HPC3_DMA_ADDR(cbp),
                               MEMTXATTRS_UNSPECIFIED,
                               packet_buf + packet_len, count);
            packet_len += count;
        }

        if (bc & HPC3_ENET_BC_XIE) {
            xie = true;
        }

        /* Update current buffer pointer and count */
        s->enet_tx_cbp = cbp + count;
        s->enet_tx_bc = bc;

        if (bc & HPC3_ENET_BC_EOXP) {
            /* End of packet — send it */
            if (packet_len > 0 && s->nic) {
                trace_sgi_hpc3_enet_tx_packet(packet_len);
                qemu_send_packet(qemu_get_queue(s->nic),
                                 packet_buf, packet_len);
            }

            /* Mark TX done: write TXD bit to first descriptor's BC field */
            uint32_t cpfbdp_phys = HPC3_DMA_ADDR(s->enet_tx_cpfbdp);
            uint32_t first_bc = address_space_ldl_be(
                &address_space_memory, cpfbdp_phys + 4,
                MEMTXATTRS_UNSPECIFIED, NULL);
            first_bc |= HPC3_ENET_BC_TXD;
            address_space_stl_be(&address_space_memory,
                                 cpfbdp_phys + 4, first_bc,
                                 MEMTXATTRS_UNSPECIFIED, NULL);

            /* Set Seeq TX status = success */
            s->seeq_tx_status = SEEQ_TXS_SUCCESS;

            /* Copy status to TX control register low bits */
            s->enet_tx_ctrl = (s->enet_tx_ctrl & ~HPC3_ENET_TXC_ST) |
                              (s->seeq_tx_status & HPC3_ENET_TXC_ST);

            /* Reset for next packet */
            packet_len = 0;

            /* Check for end of chain */
            if (bc & HPC3_ENET_BC_EOX) {
                s->enet_tx_ctrl &= ~HPC3_ENET_TXC_CA;
                s->enet_tx_nbdp = nbdp;
                break;
            }

            /* Save next packet's first descriptor */
            s->enet_tx_ppfbdp = s->enet_tx_cpfbdp;
            s->enet_tx_cpfbdp = nbdp;
        }

        if (bc & HPC3_ENET_BC_EOX) {
            /* End of chain without EOXP (partial packet, just stop) */
            s->enet_tx_ctrl &= ~HPC3_ENET_TXC_CA;
            s->enet_tx_nbdp = nbdp;
            break;
        }

        desc_addr = HPC3_DMA_ADDR(nbdp);
        s->enet_tx_nbdp = nbdp;
    }

    /* Raise interrupt if XIE was set or on completion */
    if (xie) {
        sgi_hpc3_enet_raise_irq(s);
    }
}

/*
 * NIC can_receive callback: check if RX DMA is active and receiver enabled.
 */
static bool sgi_hpc3_enet_can_receive(NetClientState *nc)
{
    SGIHPC3State *s = qemu_get_nic_opaque(nc);

    /* Must have RX DMA active and Seeq receiver not disabled */
    if (!(s->enet_rx_ctrl & HPC3_ENET_RXC_CA)) {
        return false;
    }
    if ((s->seeq_rx_cmd & SEEQ_RXC_MODE_MASK) == SEEQ_RXC_MODE_OFF) {
        return false;
    }
    return true;
}

/*
 * NIC receive callback: called when a packet arrives from QEMU network backend.
 * Writes packet data into guest memory via the RX descriptor chain.
 */
static ssize_t sgi_hpc3_enet_receive(NetClientState *nc,
                                      const uint8_t *buf, size_t size)
{
    SGIHPC3State *s = qemu_get_nic_opaque(nc);
    uint32_t desc_addr, cbp, bc, nbdp;
    uint8_t rx_mode;

    /* Check if we can receive */
    if (!(s->enet_rx_ctrl & HPC3_ENET_RXC_CA)) {
        trace_sgi_hpc3_enet_rx_drop((int)size, "rx_dma_inactive");
        return -1;
    }

    rx_mode = s->seeq_rx_cmd & SEEQ_RXC_MODE_MASK;
    if (rx_mode == SEEQ_RXC_MODE_OFF) {
        trace_sgi_hpc3_enet_rx_drop((int)size, "receiver_off");
        return -1;
    }

    /* Address filtering */
    if (rx_mode != SEEQ_RXC_MODE_PROMISC) {
        bool is_broadcast = (buf[0] == 0xff && buf[1] == 0xff &&
                             buf[2] == 0xff && buf[3] == 0xff &&
                             buf[4] == 0xff && buf[5] == 0xff);
        bool is_multicast = (buf[0] & 0x01);
        bool is_station = (memcmp(buf, s->seeq_station_addr, 6) == 0);

        if (rx_mode == SEEQ_RXC_MODE_NORMAL) {
            if (!is_station && !is_broadcast) {
                return size;  /* Silently drop, not for us */
            }
        } else if (rx_mode == SEEQ_RXC_MODE_MULTI) {
            if (!is_station && !is_broadcast && !is_multicast) {
                return size;  /* Silently drop */
            }
        }
    }

    /* Fetch RX descriptor from enet_rx_nbdp */
    if (!s->enet_rx_nbdp) {
        trace_sgi_hpc3_enet_rx_drop((int)size, "no_rx_descriptor");
        return -1;
    }

    desc_addr = HPC3_DMA_ADDR(s->enet_rx_nbdp);
    trace_sgi_hpc3_enet_rx((int)size, desc_addr);

    cbp = address_space_ldl_be(&address_space_memory, desc_addr,
                               MEMTXATTRS_UNSPECIFIED, NULL);
    bc = address_space_ldl_be(&address_space_memory, desc_addr + 4,
                              MEMTXATTRS_UNSPECIFIED, NULL);
    nbdp = address_space_ldl_be(&address_space_memory, desc_addr + 8,
                                MEMTXATTRS_UNSPECIFIED, NULL);

    /* Save current buffer descriptor pointer */
    s->enet_rx_cbdp = s->enet_rx_nbdp;
    s->enet_rx_cbp = cbp;
    s->enet_rx_bc = bc;

    uint32_t buf_size = bc & HPC3_ENET_BC_COUNT;

    /*
     * Skip 2 alignment bytes at the start of the buffer.
     * IRIX expects 2 bytes of padding so the IP header (at offset 14
     * in the ethernet frame) is 32-bit aligned.
     */
    uint32_t write_ptr = HPC3_DMA_ADDR(cbp) + 2;
    uint32_t remaining = (buf_size > 2) ? buf_size - 2 : 0;

    /* Write packet data */
    uint32_t to_write = (size <= remaining) ? size : remaining;
    if (to_write > 0) {
        address_space_write(&address_space_memory, write_ptr,
                            MEMTXATTRS_UNSPECIFIED, buf, to_write);
        write_ptr += to_write;
        remaining -= to_write;
    }

    /* Append Seeq status byte at the end of the frame data */
    uint8_t seeq_status = SEEQ_RXS_GOOD | SEEQ_RXS_END;
    if (remaining > 0) {
        address_space_write(&address_space_memory, write_ptr,
                            MEMTXATTRS_UNSPECIFIED, &seeq_status, 1);
        remaining--;
        write_ptr++;
    }

    /* Set Seeq RX status (clear OLD bit = new status) */
    s->seeq_rx_status = seeq_status;

    /* Copy status to RX control register low bits */
    s->enet_rx_ctrl = (s->enet_rx_ctrl & ~HPC3_ENET_RXC_ST) |
                      (s->seeq_rx_status & HPC3_ENET_RXC_ST);

    /*
     * Write remaining byte count back to descriptor.
     * Clear r_rown (bit 14) to indicate software now owns this descriptor
     * (data is ready for the driver to process). The driver's interrupt
     * handler loops while (!r_rown), so r_rown=0 means "data ready".
     */
    uint32_t updated_bc = (bc & ~(HPC3_ENET_BC_COUNT | HPC3_ENET_BC_ROWN)) |
                          (remaining & HPC3_ENET_BC_COUNT);
    address_space_stl_be(&address_space_memory, desc_addr + 4, updated_bc,
                         MEMTXATTRS_UNSPECIFIED, NULL);

    /* Advance to next descriptor */
    s->enet_rx_nbdp = nbdp;
    bool xie = (bc & HPC3_ENET_BC_XIE) != 0;

    /* Check for end of chain */
    if (bc & HPC3_ENET_BC_EOX) {
        s->enet_rx_ctrl &= ~HPC3_ENET_RXC_CA;
    }

    /* Check for buffer overflow (more data than buffer space) */
    if (size > to_write) {
        s->enet_rx_ctrl |= HPC3_ENET_RXC_RBO;
    }

    /* Raise interrupt */
    if (xie || (bc & HPC3_ENET_BC_EOX) || (s->enet_rx_ctrl & HPC3_ENET_RXC_RBO)) {
        sgi_hpc3_enet_raise_irq(s);
    }

    return size;
}

static NetClientInfo sgi_hpc3_enet_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = sgi_hpc3_enet_can_receive,
    .receive = sgi_hpc3_enet_receive,
};

static uint64_t sgi_hpc3_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    uint32_t val = 0;
    hwaddr base_addr = addr & ~3;  /* Align to 32-bit boundary */

    switch (base_addr) {
    /* GIO Bus Interface registers */
    case HPC3_INTSTAT:
        val = s->intstat;
        break;
    case HPC3_MISC:
        val = s->misc;
        break;
    case HPC3_EEPROM:
        /* Bit 4 (DO) comes from the serial EEPROM, other bits are stored */
        val = s->eeprom & ~0x10;
        if (eeprom93xx_read(s->serial_eeprom)) {
            val |= 0x10;
        }
        break;
    case HPC3_BUSERR_STAT:
        val = s->buserr_stat;
        break;

    /* SCSI channel 0 */
    case HPC3_SCSI0_CBP:
        val = s->scsi_cbp[0];
        break;
    case HPC3_SCSI0_NBDP:
        val = s->scsi_nbdp[0];
        break;
    case HPC3_SCSI0_BC:
        val = (s->scsi_dma_count[0] & HPC3_BC_COUNT_MASK) |
              (s->scsi_bc[0] & ~HPC3_BC_COUNT_MASK);
        break;
    case HPC3_SCSI0_CTRL:
        val = s->scsi_ctrl[0];
        /* Auto-clear IRQ bit on read (MAME hpc3.cpp:291-294) */
        if (val & HPC3_DMACTRL_IRQ) {
            s->scsi_ctrl[0] &= ~HPC3_DMACTRL_IRQ;
            s->intstat &= ~(0x100 << 0);
            if (s->intstat == 0) {
                s->int3_local1_stat &= ~INT3_LOCAL1_HPC_DMA;
                sgi_hpc3_update_irq(s);
            }
        }
        break;
    case HPC3_SCSI0_DMACFG:
        val = s->scsi_dmacfg[0];
        break;
    case HPC3_SCSI0_PIOCFG:
        val = s->scsi_piocfg[0];
        break;
    case HPC3_SCSI0_GIO_FIFO:
        val = 0;
        break;
    case HPC3_SCSI0_DEV_FIFO:
        val = 0;
        break;

    /* SCSI channel 1 */
    case HPC3_SCSI1_CBP:
        val = s->scsi_cbp[1];
        break;
    case HPC3_SCSI1_NBDP:
        val = s->scsi_nbdp[1];
        break;
    case HPC3_SCSI1_BC:
        val = (s->scsi_dma_count[1] & HPC3_BC_COUNT_MASK) |
              (s->scsi_bc[1] & ~HPC3_BC_COUNT_MASK);
        break;
    case HPC3_SCSI1_CTRL:
        val = s->scsi_ctrl[1];
        /* Auto-clear IRQ bit on read (MAME hpc3.cpp:291-294) */
        if (val & HPC3_DMACTRL_IRQ) {
            s->scsi_ctrl[1] &= ~HPC3_DMACTRL_IRQ;
            s->intstat &= ~(0x100 << 1);
            if (s->intstat == 0) {
                s->int3_local1_stat &= ~INT3_LOCAL1_HPC_DMA;
                sgi_hpc3_update_irq(s);
            }
        }
        break;
    case HPC3_SCSI1_GIO_FIFO:
        val = 0;
        break;
    case HPC3_SCSI1_DEV_FIFO:
        val = 0;
        break;
    case HPC3_SCSI1_DMACFG:
        val = s->scsi_dmacfg[1];
        break;
    case HPC3_SCSI1_PIOCFG:
        val = s->scsi_piocfg[1];
        break;

    /* Ethernet DMA registers */
    case HPC3_ENET_RX_CBP:
        val = s->enet_rx_cbp;
        break;
    case HPC3_ENET_RX_NBDP:
        val = s->enet_rx_nbdp;
        break;
    case HPC3_ENET_RX_BC:
        val = s->enet_rx_bc;
        break;
    case HPC3_ENET_RX_CTRL:
        val = s->enet_rx_ctrl;
        break;
    case HPC3_ENET_RX_GIO:
    case HPC3_ENET_RX_DEV:
        val = 0;
        break;
    case HPC3_ENET_TX_CBP:
        val = s->enet_tx_cbp;
        break;
    case HPC3_ENET_TX_NBDP:
        val = s->enet_tx_nbdp;
        break;
    case HPC3_ENET_TX_BC:
        val = s->enet_tx_bc;
        break;
    case HPC3_ENET_TX_CTRL:
        val = s->enet_tx_ctrl;
        break;
    case HPC3_ENET_TX_GIO:
    case HPC3_ENET_TX_DEV:
        val = 0;
        break;
    case HPC3_ENET_MISC:
        val = s->enet_misc;
        break;
    case HPC3_ENET_DMACFG:
        val = s->enet_dmacfg;
        break;
    case HPC3_ENET_PIOCFG:
        val = s->enet_piocfg;
        break;
    case HPC3_ENET_RX_CBDP:
        val = s->enet_rx_cbdp;
        break;
    case HPC3_ENET_TX_CPFBDP:
        val = s->enet_tx_cpfbdp;
        break;
    case HPC3_ENET_TX_PPFBDP:
        val = s->enet_tx_ppfbdp;
        break;

    /* IOC registers - note: byte access at +3 offset for big-endian */
    case HPC3_PAR_DATA:
        val = s->par_data;
        break;
    case HPC3_PAR_CONTROL:
        val = s->par_control;
        break;
    case HPC3_GEN_CONTROL:
        val = s->gen_control;
        break;
    case HPC3_PANEL:
        /* Power button state, volume buttons - return "all buttons up" */
        val = s->panel | 0xf0;
        break;
    case HPC3_SYS_ID:
        val = s->sysid;
        break;
    case HPC3_READ:
        val = s->read_reg;
        break;
    case HPC3_DMA_SELECT:
        val = s->dma_select;
        break;
    case HPC3_WRITE1:
        val = s->write1;
        break;
    case HPC3_WRITE2:
        val = s->write2;
        break;

    /*
     * Serial ports - Z85C30 style (byte accessed at +3 for BE)
     *
     * The register pointer selects which Read Register (RR) is returned.
     * Writing to the cmd register with bits [3:0] sets the pointer for the
     * next read. After a read, the pointer resets to 0 (RR0).
     *
     * Key registers:
     *   RR0: TX empty (bit 2), RX available (bit 0)
     *   RR1: All Sent (bit 0) - transmitter completely idle
     *   RR3: Interrupt pending bits (channel A only)
     */
    /*
     * Serial port read registers (Z85C30 SCC)
     *
     * Channel B = port 1 (HPC3_SERIAL1, console)
     * Channel A = port 0 (HPC3_SERIAL0)
     *
     * RR3 (interrupt pending) is only valid on channel A.
     */
    case HPC3_SERIAL1_CMD:  /* Channel B reads */
        switch (s->serial_reg_ptr[1]) {
        case 1: /* RR1: Special receive conditions */
            val = 0x01;  /* Bit 0 = All Sent (TX complete/idle) */
            break;
        case 2: /* RR2: Interrupt vector (modified by pending IPs on Ch B) */
            val = 0;
            break;
        case 3: /* RR3: Not valid on channel B, returns 0 */
            val = 0;
            break;
        case 8: /* RR8: Receive data (same as DATA register) */
            if (s->serial_rx_fifo_count[1] > 0) {
                val = s->serial_rx_fifo[1][s->serial_rx_fifo_tail[1]];
                s->serial_rx_fifo_tail[1] =
                    (s->serial_rx_fifo_tail[1] + 1) % SCC_RX_FIFO_SIZE;
                s->serial_rx_fifo_count[1]--;
                qemu_chr_fe_accept_input(&s->serial);
                if (s->serial_rx_fifo_count[1] == 0) {
                    s->scc_rr3 &= ~0x04;  /* Clear Ch B RX IP */
                    sgi_hpc3_scc_update_irq(s);
                }
            } else {
                val = 0;
            }
            break;
        default: /* RR0: Status */
            val = 0x2c;  /* Bit 2 = TX empty, Bit 3 = DCD, Bit 5 = CTS */
            if (s->serial_rx_fifo_count[1] > 0) {
                val |= 0x01;  /* Bit 0 = RX character available */
            }
            break;
        }
        s->serial_reg_ptr[1] = 0;
        break;
    case HPC3_SERIAL1_DATA:
        if (s->serial_rx_fifo_count[1] > 0) {
            val = s->serial_rx_fifo[1][s->serial_rx_fifo_tail[1]];
            s->serial_rx_fifo_tail[1] =
                (s->serial_rx_fifo_tail[1] + 1) % SCC_RX_FIFO_SIZE;
            s->serial_rx_fifo_count[1]--;
            qemu_chr_fe_accept_input(&s->serial);
            if (s->serial_rx_fifo_count[1] == 0) {
                s->scc_rr3 &= ~0x04;  /* Clear Ch B RX IP */
                sgi_hpc3_scc_update_irq(s);
            }
        } else {
            val = 0;
        }
        break;
    case HPC3_SERIAL0_CMD:  /* Channel A reads */
        switch (s->serial_reg_ptr[0]) {
        case 1: /* RR1 */
            val = 0x01;  /* All Sent */
            break;
        case 2: /* RR2: Interrupt vector (unmodified on Ch A) */
            val = 0;
            break;
        case 3: /* RR3: Interrupt pending (ONLY valid on channel A) */
            val = s->scc_rr3;
            if (val) {
                qemu_log_mask(LOG_UNIMP,
                    "sgi_hpc3: ChA RR3 read = 0x%02x\n", val);
            }
            break;
        case 8: /* RR8: Receive data */
            if (s->serial_rx_fifo_count[0] > 0) {
                val = s->serial_rx_fifo[0][s->serial_rx_fifo_tail[0]];
                s->serial_rx_fifo_tail[0] =
                    (s->serial_rx_fifo_tail[0] + 1) % SCC_RX_FIFO_SIZE;
                s->serial_rx_fifo_count[0]--;
                qemu_chr_fe_accept_input(&s->serial);
                if (s->serial_rx_fifo_count[0] == 0) {
                    s->scc_rr3 &= ~0x20;  /* Clear Ch A RX IP */
                    sgi_hpc3_scc_update_irq(s);
                }
            } else {
                val = 0;
            }
            break;
        default: /* RR0 */
            val = 0x2c;  /* TX buffer empty, DCD, CTS */
            if (s->serial_rx_fifo_count[0] > 0) {
                val |= 0x01;
            }
            break;
        }
        s->serial_reg_ptr[0] = 0;
        break;
    case HPC3_SERIAL0_DATA:
        if (s->serial_rx_fifo_count[0] > 0) {
            val = s->serial_rx_fifo[0][s->serial_rx_fifo_tail[0]];
            s->serial_rx_fifo_tail[0] =
                (s->serial_rx_fifo_tail[0] + 1) % SCC_RX_FIFO_SIZE;
            s->serial_rx_fifo_count[0]--;
            qemu_chr_fe_accept_input(&s->serial);
            if (s->serial_rx_fifo_count[0] == 0) {
                s->scc_rr3 &= ~0x20;  /* Clear Ch A RX IP */
                sgi_hpc3_scc_update_irq(s);
            }
        } else {
            val = 0;
        }
        break;

    /* 8042 Keyboard/Mouse controller (embedded in IOC2) */
    case HPC3_KBD_MOUSE0:  /* 0x59840 - Data port */
        val = sgi_hpc3_kbd_read_data(s);
        break;
    case HPC3_KBD_MOUSE1:  /* 0x59844 - Status/Command port */
        val = sgi_hpc3_kbd_status(s);
        break;

    /* INT3 interrupt controller (byte access at +3) */
    case HPC3_INT3_LOCAL0_STAT:
        val = s->int3_local0_stat;
        break;
    case HPC3_INT3_LOCAL0_MASK:
        val = s->int3_local0_mask;
        break;
    case HPC3_INT3_LOCAL1_STAT:
        val = s->int3_local1_stat;
        break;
    case HPC3_INT3_LOCAL1_MASK:
        val = s->int3_local1_mask;
        break;
    case HPC3_INT3_MAP_STATUS:
        val = s->int3_map_status;
        break;
    case HPC3_INT3_MAP_MASK0:
        val = s->int3_map_mask0;
        break;
    case HPC3_INT3_MAP_MASK1:
        val = s->int3_map_mask1;
        break;
    case HPC3_INT3_MAP_POLARITY:
        val = s->int3_map_polarity;
        break;
    case HPC3_INT3_TIMER_CLEAR:
        val = 0;
        break;
    case HPC3_INT3_ERROR_STATUS:
        val = s->int3_error_status;
        break;

    /* 8254 PIT timer registers */
    case HPC3_INT3_PIT_COUNTER0:
        val = sgi_hpc3_pit_read(s, 0);
        break;
    case HPC3_INT3_PIT_COUNTER1:
        val = sgi_hpc3_pit_read(s, 1);
        break;
    case HPC3_INT3_PIT_COUNTER2:
        val = sgi_hpc3_pit_read(s, 2);
        break;
    case HPC3_INT3_PIT_CONTROL:
        /* Control register is write-only, return 0 */
        val = 0;
        break;

    /* Full House (IP22) INT3 at PIO4 base - same state, different offsets */
    case HPC3_FH_INT3_LOCAL0_STAT:
        val = s->int3_local0_stat;
        break;
    case HPC3_FH_INT3_LOCAL0_MASK:
        val = s->int3_local0_mask;
        break;
    case HPC3_FH_INT3_LOCAL1_STAT:
        val = s->int3_local1_stat;
        break;
    case HPC3_FH_INT3_LOCAL1_MASK:
        val = s->int3_local1_mask;
        break;
    case HPC3_FH_INT3_MAP_STATUS:
        val = s->int3_map_status;
        break;
    case HPC3_FH_INT3_MAP_MASK0:
        val = s->int3_map_mask0;
        break;
    case HPC3_FH_INT3_MAP_MASK1:
        val = s->int3_map_mask1;
        break;
    case HPC3_FH_INT3_TIMER_CLEAR:
        val = 0;
        break;
    case HPC3_FH_INT3_PIT_COUNTER0:
        val = sgi_hpc3_pit_read(s, 0);
        break;
    case HPC3_FH_INT3_PIT_COUNTER1:
        val = sgi_hpc3_pit_read(s, 1);
        break;
    case HPC3_FH_INT3_PIT_COUNTER2:
        val = sgi_hpc3_pit_read(s, 2);
        break;
    case HPC3_FH_INT3_PIT_CONTROL:
        val = 0;
        break;

    default:
        /*
         * SCSI0 controller registers (WD33C93 at 0x40000-0x40fff)
         * SGI uses 4-byte spacing: offset 0 = addr reg, offset 4 = data reg
         */
        if (base_addr >= HPC3_SCSI0_REG && base_addr < HPC3_SCSI0_REG + 0x1000) {
            int reg = (addr - HPC3_SCSI0_REG) & 0x7;
            if (s->scsi[0]) {
                if (reg < 4) {
                    val = wd33c93_addr_read(s->scsi[0]);
                } else {
                    val = wd33c93_data_read(s->scsi[0]);
                }
            }
        }
        /* SCSI1 controller registers (WD33C93 at 0x48000-0x48fff) */
        else if (base_addr >= HPC3_SCSI1_REG && base_addr < HPC3_SCSI1_REG + 0x1000) {
            int reg = (addr - HPC3_SCSI1_REG) & 0x7;
            if (s->scsi[1]) {
                if (reg < 4) {
                    val = wd33c93_addr_read(s->scsi[1]);
                } else {
                    val = wd33c93_data_read(s->scsi[1]);
                }
            }
        }
        /*
         * Ethernet controller (SEEQ 80C03 at 0x54000-0x54fff)
         *
         * Registers are at 4-byte spacing. IRIX accesses byte 3 of each
         * 32-bit word (big-endian). With addr & ~7 normalization and
         * (addr - HPC3_ENET_REG) calculation, register index = offset >> 2.
         *
         * Read side:
         *   Reg 0-5: collision counters (we return 0)
         *   Reg 6: RX status (reading sets OLD bit)
         *   Reg 7: TX status (reading sets OLD bit)
         */
        else if (base_addr >= HPC3_ENET_REG && base_addr < HPC3_ENET_REG + 0x1000) {
            int reg = ((addr - HPC3_ENET_REG) >> 2) & 7;
            switch (reg) {
            case 6:  /* RX status */
                val = s->seeq_rx_status;
                s->seeq_rx_status |= SEEQ_RXS_OLD;  /* Set OLD on read */
                trace_sgi_hpc3_enet_seeq_read(reg, val);
                break;
            case 7:  /* TX status */
                val = s->seeq_tx_status;
                s->seeq_tx_status |= SEEQ_TXS_OLD;  /* Set OLD on read */
                trace_sgi_hpc3_enet_seeq_read(reg, val);
                break;
            default:
                val = 0;  /* Collision counters, flags - return 0 */
                break;
            }
        }
        /*
         * PBUS PIO data area (0x58000-0x5bfff)
         * The PROM accesses the keyboard controller via PBUS PIO:
         *   0x58484 = Status register (read)
         *   0x58488 = Data register (read)
         *   0x5848c = Alternate status access
         */
        else if (base_addr >= 0x58000 && base_addr < 0x5c000) {
            hwaddr pio_offset = base_addr - 0x58000;
            if (pio_offset == 0x484) {
                /* Status register */
                val = sgi_hpc3_kbd_status(s);
            } else if (pio_offset == 0x488) {
                /* Data register */
                val = sgi_hpc3_kbd_read_data(s);
            } else if (pio_offset == 0x48c) {
                /* Alternate status access */
                val = sgi_hpc3_kbd_status(s);
            } else if (pio_offset == HAL2_REG_ISR) {
                /* HAL2 ISR: TSTATUS always clear (never busy) */
                val = s->hal2_isr & ~HAL2_ISR_TSTATUS;
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else if (pio_offset == HAL2_REG_REV) {
                /* HAL2 revision register */
                val = HAL2_REV_VALUE;
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else if (pio_offset == HAL2_REG_IAR) {
                val = s->hal2_iar;
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else if (pio_offset >= HAL2_REG_IDR0
                       && pio_offset <= HAL2_REG_IDR3
                       && ((pio_offset & 0x0f) == 0)) {
                int idx = (pio_offset - HAL2_REG_IDR0) >> 4;
                val = s->hal2_idr[idx];
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else if (pio_offset == HAL2_VOLUME_RIGHT) {
                val = s->hal2_volume_right;
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else if (pio_offset == HAL2_VOLUME_LEFT) {
                val = s->hal2_volume_left;
                trace_sgi_hpc3_hal2_read(pio_offset, val);
            } else {
                /* Other PBUS PIO reads return 0 */
                val = 0;
            }
        }
        /* BBRAM (0x60000-0x7ffff) - only 8KB used, mirrored
         * DS1386 RTC registers have 4-byte spacing on SGI hardware.
         * Register index = byte_offset / 4
         */
        else if (base_addr >= 0x60000 && base_addr < 0x80000) {
            int byte_offset = (addr - 0x60000) & 0x1fff;  /* 8KB mask */
            int reg = byte_offset >> 2;  /* 4-byte spacing */
            if (reg < 16) {
                /* First 16 registers are DS1386 RTC */
                val = sgi_hpc3_rtc_read(s, reg);
            } else {
                val = s->bbram[reg];
            }
            trace_sgi_hpc3_nvram(reg, val, 0);
        }
        /* PBUS DMA channel registers (8 channels, stride 0x2000) */
        else if (base_addr < 8 * HPC3_PBUS_STRIDE) {
            int ch = base_addr / HPC3_PBUS_STRIDE;
            hwaddr ch_offset = base_addr % HPC3_PBUS_STRIDE;
            if (ch_offset == HPC3_PBUS_BP_BASE) {
                val = s->pbus_bp[ch];
            } else if (ch_offset == HPC3_PBUS_DP_BASE) {
                val = s->pbus_dp[ch];
            } else if (ch_offset == HPC3_PBUS_CTRL_BASE) {
                val = s->pbus_ctrl[ch];
            }
            /* Other offsets in channel return 0 */
        }
        /* PBUS DMA config */
        else if (base_addr >= HPC3_PBUS_CFGDMA_BASE &&
            base_addr < HPC3_PBUS_CFGDMA_BASE + 8 * HPC3_PBUS_CFG_STRIDE) {
            int idx = (base_addr - HPC3_PBUS_CFGDMA_BASE) / HPC3_PBUS_CFG_STRIDE;
            val = s->pbus_dmacfg[idx];
        }
        /* PBUS PIO config */
        else if (base_addr >= HPC3_PBUS_CFGPIO_BASE &&
                 base_addr < HPC3_PBUS_CFGPIO_BASE + 10 * HPC3_PBUS_CFG_STRIDE) {
            int idx = (base_addr - HPC3_PBUS_CFGPIO_BASE) / HPC3_PBUS_CFG_STRIDE;
            val = s->pbus_piocfg[idx];
        }
        else {
            qemu_log_mask(LOG_UNIMP,
                          "sgi_hpc3: unimplemented read at 0x%05" HWADDR_PRIx
                          "\n", addr);
        }
        break;
    }

    trace_sgi_hpc3_read((uint64_t)addr, (uint64_t)val);
    return val;
}

static void sgi_hpc3_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    SGIHPC3State *s = SGI_HPC3(opaque);
    hwaddr base_addr = addr & ~3;  /* Align to 32-bit boundary */

    trace_sgi_hpc3_write((uint64_t)addr, val);

    switch (base_addr) {
    /* GIO Bus Interface registers */
    case HPC3_INTSTAT:
        s->intstat &= ~val;  /* Write to clear */
        if (s->intstat == 0) {
            s->int3_local1_stat &= ~INT3_LOCAL1_HPC_DMA;
            sgi_hpc3_update_irq(s);
        }
        break;
    case HPC3_MISC:
        s->misc = val;
        break;
    case HPC3_EEPROM:
        s->eeprom = val;
        eeprom93xx_write(s->serial_eeprom,
                         (val >> 1) & 1,    /* CS  = bit 1 */
                         (val >> 2) & 1,    /* CLK = bit 2 */
                         (val >> 3) & 1);   /* DI  = bit 3 */
        break;
    case HPC3_BUSERR_STAT:
        s->buserr_stat &= ~val;  /* Write to clear */
        break;

    /* SCSI channel 0 */
    case HPC3_SCSI0_CBP:
        s->scsi_cbp[0] = val;
        break;
    case HPC3_SCSI0_NBDP:
        s->scsi_nbdp[0] = val;
        break;
    case HPC3_SCSI0_BC:
        s->scsi_bc[0] = val;
        break;
    case HPC3_SCSI0_CTRL:
        sgi_hpc3_scsi_ctrl_write(s, 0, val);
        break;
    case HPC3_SCSI0_DMACFG:
        s->scsi_dmacfg[0] = val;
        break;
    case HPC3_SCSI0_PIOCFG:
        s->scsi_piocfg[0] = val;
        break;
    case HPC3_SCSI0_GIO_FIFO:
        break;
    case HPC3_SCSI0_DEV_FIFO:
        break;

    /* SCSI channel 1 */
    case HPC3_SCSI1_CBP:
        s->scsi_cbp[1] = val;
        break;
    case HPC3_SCSI1_NBDP:
        s->scsi_nbdp[1] = val;
        break;
    case HPC3_SCSI1_BC:
        s->scsi_bc[1] = val;
        break;
    case HPC3_SCSI1_CTRL:
        sgi_hpc3_scsi_ctrl_write(s, 1, val);
        break;
    case HPC3_SCSI1_GIO_FIFO:
        break;
    case HPC3_SCSI1_DEV_FIFO:
        break;
    case HPC3_SCSI1_DMACFG:
        s->scsi_dmacfg[1] = val;
        break;
    case HPC3_SCSI1_PIOCFG:
        s->scsi_piocfg[1] = val;
        break;

    /* Ethernet DMA registers */
    case HPC3_ENET_RX_CBP:
        s->enet_rx_cbp = val;
        break;
    case HPC3_ENET_RX_NBDP:
        s->enet_rx_nbdp = val;
        break;
    case HPC3_ENET_RX_BC:
        s->enet_rx_bc = val;
        break;
    case HPC3_ENET_RX_CTRL:
        /*
         * RX control register write with MAME masking semantics:
         * Status bits (low byte from Seeq) and LC are read-only.
         */
        if (s->enet_rx_ctrl & HPC3_ENET_RXC_CAM) {
            /* CAM set: cannot modify CA bit */
            s->enet_rx_ctrl = (s->enet_rx_ctrl & (HPC3_ENET_RXC_LC | HPC3_ENET_RXC_ST)) |
                              (val & ~(HPC3_ENET_RXC_CA | HPC3_ENET_RXC_LC | HPC3_ENET_RXC_ST));
        } else {
            s->enet_rx_ctrl = (s->enet_rx_ctrl & (HPC3_ENET_RXC_LC | HPC3_ENET_RXC_ST)) |
                              (val & ~(HPC3_ENET_RXC_LC | HPC3_ENET_RXC_ST));
        }
        /* If RX DMA just became active, flush queued packets */
        if ((s->enet_rx_ctrl & HPC3_ENET_RXC_CA) && s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case HPC3_ENET_RX_GIO:
    case HPC3_ENET_RX_DEV:
        break;  /* FIFO pointers, read-only */
    case HPC3_ENET_TX_CBP:
        s->enet_tx_cbp = val;
        break;
    case HPC3_ENET_TX_NBDP:
        s->enet_tx_nbdp = val;
        break;
    case HPC3_ENET_TX_BC:
        s->enet_tx_bc = val;
        break;
    case HPC3_ENET_TX_CTRL:
        /*
         * TX control register write with MAME masking semantics.
         * When TXC_CA is set, start TX DMA.
         */
        if (s->enet_tx_ctrl & HPC3_ENET_TXC_CAM) {
            s->enet_tx_ctrl = (s->enet_tx_ctrl & (HPC3_ENET_TXC_LC | HPC3_ENET_TXC_ST)) |
                              (val & ~(HPC3_ENET_TXC_CA | HPC3_ENET_TXC_LC | HPC3_ENET_TXC_ST));
        } else {
            s->enet_tx_ctrl = (s->enet_tx_ctrl & (HPC3_ENET_TXC_LC | HPC3_ENET_TXC_ST)) |
                              (val & ~(HPC3_ENET_TXC_LC | HPC3_ENET_TXC_ST));
        }
        if (s->enet_tx_ctrl & HPC3_ENET_TXC_CA) {
            sgi_hpc3_enet_tx(s);
        }
        break;
    case HPC3_ENET_TX_GIO:
    case HPC3_ENET_TX_DEV:
        break;  /* FIFO pointers, read-only */
    case HPC3_ENET_MISC:
        /*
         * ENET_MISC register:
         *   Bit 0 (RESET): 1 = assert reset, 0 = normal operation
         *   Bit 1 (INT): write 1 to clear interrupt
         *   Bit 2 (LOOPBACK): loopback mode
         */
        /*
         * ERST (bit 0) is toggled by the IRIX driver during every interrupt
         * acknowledge (write 0x03, then 0x00). This is NOT a full device
         * reset — it's part of the HPC3 interrupt handshake. Full reset
         * only happens at device power-on via sgi_hpc3_enet_reset().
         */
        if (val & HPC3_ENET_MISC_INT) {
            /* Clear interrupt */
            s->int3_local0_stat &= ~INT3_LOCAL0_ETHERNET;
            sgi_hpc3_update_irq(s);
        }
        /* Store value, but INT bit is write-1-to-clear */
        s->enet_misc = val & ~HPC3_ENET_MISC_INT;
        break;
    case HPC3_ENET_DMACFG:
        s->enet_dmacfg = val;
        break;
    case HPC3_ENET_PIOCFG:
        s->enet_piocfg = val;
        break;
    case HPC3_ENET_RX_CBDP:
        s->enet_rx_cbdp = val;
        break;
    case HPC3_ENET_TX_CPFBDP:
        s->enet_tx_cpfbdp = val;
        break;
    case HPC3_ENET_TX_PPFBDP:
        s->enet_tx_ppfbdp = val;
        break;

    /* IOC registers (byte access at +3 for BE) */
    case HPC3_PAR_DATA:
        s->par_data = val;
        break;
    case HPC3_PAR_CONTROL:
        s->par_control = val;
        break;
    case HPC3_GEN_CONTROL:
        s->gen_control = val;
        break;
    case HPC3_PANEL:
        s->panel = val;
        break;
    case HPC3_DMA_SELECT:
        s->dma_select = val;
        break;
    case HPC3_WRITE1:
        s->write1 = val;
        /* Bit 0x10 = LED green off, 0x20 = LED amber off */
        break;
    case HPC3_WRITE2:
        s->write2 = val;
        break;

    /*
     * Serial ports - Z85C30 (byte accessed at +3 for BE)
     *
     * The SCC uses register-indirect addressing:
     * - First write to CMD sets the register pointer (low 4 bits)
     * - Second write to CMD writes data to that register
     * - Register 8 (WR8) is the TX buffer
     * - After any data write, register pointer resets to 0
     */
    /*
     * Serial ports - Z85C30 SCC write registers
     *
     * Register access protocol:
     *   1. Write register number to CMD (sets pointer, executes WR0 commands)
     *   2. Write data to CMD (writes to selected WR[n])
     *   3. After data write, pointer resets to 0
     *
     * WR0 (bits 5:3) commands executed on every pointer-set write:
     *   000 = Null, 001 = Point High (+8), 010 = Reset Ext/Status Int
     *   101 = Reset TX Int Pending, 110 = Error Reset
     *
     * Port 1 = Channel B (console), Port 0 = Channel A
     */
    case HPC3_SERIAL1_CMD:
        if (s->serial_reg_ptr[1] == 0) {
            /* Writing to WR0: command + register pointer */
            uint8_t cmd = (val >> 3) & 0x07;
            switch (cmd) {
            case 0: /* Null */
                break;
            case 1: /* Point High */
                break;
            case 2: /* Reset External/Status Interrupts */
                s->scc_rr3 &= ~0x01;  /* Clear Ch B Ext/Status IP */
                sgi_hpc3_scc_update_irq(s);
                break;
            case 5: /* Reset TX Interrupt Pending */
                qemu_log_mask(LOG_UNIMP,
                    "sgi_hpc3: ChB WR0 cmd5 Reset TX IP (rr3 0x%02x→0x%02x)\n",
                    s->scc_rr3, s->scc_rr3 & ~0x02);
                s->scc_rr3 &= ~0x02;  /* Clear Ch B TX IP */
                sgi_hpc3_scc_update_irq(s);
                break;
            case 6: /* Error Reset */
                break;
            }
            /*
             * Z85C30 register pointer: bits 2:0 select register 0-7.
             * Command 1 (Point High) adds 8 for registers 8-15.
             * Previous code used val & 0x0f which included bit 3 (part of
             * the command field), causing WR0_RST_TX_INT (0x28) to set the
             * pointer to 8 instead of 0.  This corrupted all subsequent
             * register accesses — the register number was sent as TX data
             * and the register value was interpreted as a WR0 command.
             */
            {
                uint8_t reg_ptr = val & 0x07;
                if (cmd == 1) {  /* Point High: add 8 */
                    reg_ptr |= 0x08;
                }
                s->serial_reg_ptr[1] = reg_ptr;
            }
        } else {
            /* Writing to WR[n] */
            switch (s->serial_reg_ptr[1]) {
            case 1: /* WR1: Interrupt enables */
                {
                    uint8_t old_wr1 = s->scc_wr1[1];
                    s->scc_wr1[1] = val;
                    qemu_log_mask(LOG_UNIMP,
                        "sgi_hpc3: ChB WR1 0x%02x→0x%02x (TX_INT %s→%s)\n",
                        old_wr1, (uint8_t)val,
                        (old_wr1 & 0x02) ? "ON" : "OFF",
                        (val & 0x02) ? "ON" : "OFF");
                    /* Debug: log WR1 TX_INT transitions (remove after debugging) */
                    /*
                     * Per Z85C30 datasheet: "when the WR1 D1 bit [Tx Int
                     * Enable] is first set after initialization, the Tx IP
                     * bit is set."  The TX buffer is always empty in our
                     * emulation, so TX_IP fires immediately on the 0→1
                     * transition.
                     *
                     * We set TX_IP synchronously and schedule a timer to
                     * deliver the interrupt after the current instruction
                     * completes.  This ensures the IRIX STREAMS driver
                     * sees TX_IP in RR3 when du_handle_intr runs.
                     *
                     * Previous implementation used a 100µs deferred timer,
                     * but mips_du_stop_tx cleared TX_INT_ENBL before the
                     * timer fired, preventing STREAMS TX from ever working.
                     */
                    if ((val & 0x02) && !(old_wr1 & 0x02)) {
                        /* Set TX_IP immediately — latched in RR3 */
                        s->scc_rr3 |= 0x02;  /* Ch B TX IP */
                        /* Cascade interrupt through INT3 immediately */
                        sgi_hpc3_scc_update_irq(s);
                    }
                }
                break;
            case 5: /* WR5: TX controls */
                s->scc_wr5[1] = val;
                break;
            case 8: /* WR8: TX data buffer */
                sgi_hpc3_serial_write(s, 1, val);
                break;
            case 9: /* WR9: Master interrupt control (shared) */
                s->scc_wr9 = val;
                /* Bit 7: Hardware reset channel A */
                /* Bit 6: Hardware reset channel B */
                if (val & 0xc0) {
                    /* Channel reset clears interrupt state */
                    if (val & 0x80) {
                        s->scc_rr3 &= ~0x38;  /* Clear Ch A bits */
                        s->scc_wr1[0] = 0;
                    }
                    if (val & 0x40) {
                        s->scc_rr3 &= ~0x07;  /* Clear Ch B bits */
                        s->scc_wr1[1] = 0;
                    }
                }
                sgi_hpc3_scc_update_irq(s);
                break;
            }
            s->serial_reg_ptr[1] = 0;
        }
        break;
    case HPC3_SERIAL1_DATA:
        sgi_hpc3_serial_write(s, 1, val);
        break;
    case HPC3_SERIAL0_CMD:
        if (s->serial_reg_ptr[0] == 0) {
            /* Writing to WR0: command + register pointer */
            uint8_t cmd = (val >> 3) & 0x07;
            switch (cmd) {
            case 0:
                break;
            case 1: /* Point High */
                break;
            case 2: /* Reset External/Status Interrupts */
                s->scc_rr3 &= ~0x08;  /* Clear Ch A Ext/Status IP */
                sgi_hpc3_scc_update_irq(s);
                break;
            case 5: /* Reset TX Interrupt Pending */
                s->scc_rr3 &= ~0x10;  /* Clear Ch A TX IP */
                sgi_hpc3_scc_update_irq(s);
                break;
            case 6: /* Error Reset */
                break;
            }
            /* Register pointer: bits 2:0, Point High adds 8 */
            {
                uint8_t reg_ptr = val & 0x07;
                if (cmd == 1) {
                    reg_ptr |= 0x08;
                }
                s->serial_reg_ptr[0] = reg_ptr;
            }
        } else {
            switch (s->serial_reg_ptr[0]) {
            case 1: /* WR1: Interrupt enables */
                {
                    uint8_t old_wr1 = s->scc_wr1[0];
                    s->scc_wr1[0] = val;
                    /* TX_IP fires synchronously on TX_INT_ENBL 0→1 */
                    if ((val & 0x02) && !(old_wr1 & 0x02)) {
                        s->scc_rr3 |= 0x10;  /* Ch A TX IP */
                        sgi_hpc3_scc_update_irq(s);
                    }
                }
                break;
            case 5:
                s->scc_wr5[0] = val;
                break;
            case 8:
                sgi_hpc3_serial_write(s, 0, val);
                break;
            case 9:
                s->scc_wr9 = val;
                if (val & 0xc0) {
                    if (val & 0x80) {
                        s->scc_rr3 &= ~0x38;
                        s->scc_wr1[0] = 0;
                    }
                    if (val & 0x40) {
                        s->scc_rr3 &= ~0x07;
                        s->scc_wr1[1] = 0;
                    }
                }
                sgi_hpc3_scc_update_irq(s);
                break;
            }
            s->serial_reg_ptr[0] = 0;
        }
        break;
    case HPC3_SERIAL0_DATA:
        sgi_hpc3_serial_write(s, 0, val);
        break;

    /* 8042 Keyboard/Mouse controller (embedded in IOC2) */
    case HPC3_KBD_MOUSE0:  /* 0x59840 - Data port */
        sgi_hpc3_kbd_data_write(s, val);
        break;
    case HPC3_KBD_MOUSE1:  /* 0x59844 - Command port */
        sgi_hpc3_kbd_command(s, val);
        break;

    /* INT3 interrupt controller (byte access at +3) */
    case HPC3_INT3_LOCAL0_MASK:
        s->int3_local0_mask = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_INT3_LOCAL1_MASK:
        s->int3_local1_mask = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_INT3_MAP_MASK0:
        s->int3_map_mask0 = val;
        /* Re-evaluate mapped interrupt cascade (per MAME set_map_int_mask) */
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_INT3_MAP_MASK1:
        s->int3_map_mask1 = val;
        /* Re-evaluate mapped interrupt cascade (per MAME set_map_int_mask) */
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_INT3_MAP_POLARITY:
        s->int3_map_polarity = val;
        break;
    case HPC3_INT3_TIMER_CLEAR:
        /*
         * Clear timer interrupts (like MAME ioc2.cpp:623-635)
         * Bit 0 clears timer 0, bit 1 clears timer 1.
         * Timer interrupts go directly to CPU via timer_irq[] lines,
         * not through INT3 cascade.  MAPPABLE0/MAPPABLE1 are handled
         * centrally by sgi_hpc3_update_irq().
         */
        if (val & 0x01) {
            s->timer_pending[0] = false;
        }
        if (val & 0x02) {
            s->timer_pending[1] = false;
        }
        sgi_hpc3_update_irq(s);
        break;

    /* 8254 PIT timer registers */
    case HPC3_INT3_PIT_COUNTER0:
        sgi_hpc3_pit_write(s, 0, val);
        break;
    case HPC3_INT3_PIT_COUNTER1:
        sgi_hpc3_pit_write(s, 1, val);
        break;
    case HPC3_INT3_PIT_COUNTER2:
        sgi_hpc3_pit_write(s, 2, val);
        break;
    case HPC3_INT3_PIT_CONTROL:
        sgi_hpc3_pit_control_write(s, val);
        break;

    /* Full House (IP22) INT3 at PIO4 base - same state, different offsets */
    case HPC3_FH_INT3_LOCAL0_MASK:
        s->int3_local0_mask = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_FH_INT3_LOCAL1_MASK:
        s->int3_local1_mask = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_FH_INT3_MAP_MASK0:
        s->int3_map_mask0 = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_FH_INT3_MAP_MASK1:
        s->int3_map_mask1 = val;
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_FH_INT3_TIMER_CLEAR:
        if (val & 0x01) {
            s->timer_pending[0] = false;
        }
        if (val & 0x02) {
            s->timer_pending[1] = false;
        }
        sgi_hpc3_update_irq(s);
        break;
    case HPC3_FH_INT3_PIT_COUNTER0:
        sgi_hpc3_pit_write(s, 0, val);
        break;
    case HPC3_FH_INT3_PIT_COUNTER1:
        sgi_hpc3_pit_write(s, 1, val);
        break;
    case HPC3_FH_INT3_PIT_COUNTER2:
        sgi_hpc3_pit_write(s, 2, val);
        break;
    case HPC3_FH_INT3_PIT_CONTROL:
        sgi_hpc3_pit_control_write(s, val);
        break;

    default:
        /*
         * SCSI0 controller registers (WD33C93 at 0x40000-0x40fff)
         * SGI uses 4-byte spacing: offset 0 = addr reg, offset 4 = data reg
         */
        if (base_addr >= HPC3_SCSI0_REG && base_addr < HPC3_SCSI0_REG + 0x1000) {
            int reg = (addr - HPC3_SCSI0_REG) & 0x7;
            if (s->scsi[0]) {
                if (reg < 4) {
                    wd33c93_addr_write(s->scsi[0], val);
                } else {
                    wd33c93_data_write(s->scsi[0], val);
                }
            }
        }
        /* SCSI1 controller registers (WD33C93 at 0x48000-0x48fff) */
        else if (base_addr >= HPC3_SCSI1_REG && base_addr < HPC3_SCSI1_REG + 0x1000) {
            int reg = (addr - HPC3_SCSI1_REG) & 0x7;
            if (s->scsi[1]) {
                if (reg < 4) {
                    wd33c93_addr_write(s->scsi[1], val);
                } else {
                    wd33c93_data_write(s->scsi[1], val);
                }
            }
        }
        /*
         * Ethernet controller (SEEQ 80C03 at 0x54000-0x54fff)
         *
         * Write side (bank 0 only for now):
         *   Reg 0-5: Station Address bytes 0-5
         *   Reg 6: RX Command (mode + interrupt enables)
         *   Reg 7: TX Command (bank select + interrupt enables)
         */
        else if (base_addr >= HPC3_ENET_REG && base_addr < HPC3_ENET_REG + 0x1000) {
            int reg = ((addr - HPC3_ENET_REG) >> 2) & 7;
            uint8_t bank = s->seeq_tx_cmd & SEEQ_TXC_BANK_MASK;
            trace_sgi_hpc3_enet_seeq_write(reg, (uint32_t)val);

            /*
             * Seeq 80C03 bank-selected register writes.
             * TX command bits [6:5] select which register set regs 0-5 map to:
             *   Bank 0x00: Station address (regs 0-5)
             *   Bank 0x20: Multicast filter low (regs 0-5)
             *   Bank 0x40: Multicast filter high (0-1), control (3), config (4)
             *   Bank 0x60: Reserved (all NOP except reg 6-7)
             * Regs 6 (RX cmd) and 7 (TX cmd) are always accessible.
             */
            switch (reg) {
            case 0: case 1: case 2: case 3: case 4: case 5:
                if (bank == SEEQ_TXC_BANK_STATION) {
                    s->seeq_station_addr[reg] = val;
                }
                /* Banks 0x20, 0x40, 0x60: multicast/control — ignore for now */
                break;
            case 6:  /* RX command (always accessible) */
                s->seeq_rx_cmd = val;
                /* If receiver enabled, flush queued packets */
                if ((val & SEEQ_RXC_MODE_MASK) != SEEQ_RXC_MODE_OFF && s->nic) {
                    qemu_flush_queued_packets(qemu_get_queue(s->nic));
                }
                break;
            case 7:  /* TX command (always accessible) */
                s->seeq_tx_cmd = val;
                break;
            }
        }
        /*
         * PBUS PIO data area (0x58000-0x5bfff)
         * The PROM accesses the keyboard controller via PBUS PIO:
         *   0x58484 = Command register (write)
         *   0x58488 = Data register (write)
         */
        else if (base_addr >= 0x58000 && base_addr < 0x5c000) {
            hwaddr pio_offset = base_addr - 0x58000;
            if (pio_offset == 0x484) {
                /* Command register */
                sgi_hpc3_kbd_command(s, val);
            } else if (pio_offset == 0x488) {
                /* Data register */
                sgi_hpc3_kbd_data_write(s, val);
            } else if (pio_offset == HAL2_REG_ISR) {
                /* HAL2 ISR: PROM writes reset bits here */
                s->hal2_isr = val;
                trace_sgi_hpc3_hal2_write(pio_offset, val);
            } else if (pio_offset == HAL2_REG_IAR) {
                s->hal2_iar = val;
                trace_sgi_hpc3_hal2_iar(val);
                trace_sgi_hpc3_hal2_write(pio_offset, val);
            } else if (pio_offset >= HAL2_REG_IDR0
                       && pio_offset <= HAL2_REG_IDR3
                       && ((pio_offset & 0x0f) == 0)) {
                int idx = (pio_offset - HAL2_REG_IDR0) >> 4;
                s->hal2_idr[idx] = val;
                trace_sgi_hpc3_hal2_write(pio_offset, val);
            } else if (pio_offset == HAL2_VOLUME_RIGHT) {
                s->hal2_volume_right = val;
                trace_sgi_hpc3_hal2_volume("right", val);
            } else if (pio_offset == HAL2_VOLUME_LEFT) {
                s->hal2_volume_left = val;
                trace_sgi_hpc3_hal2_volume("left", val);
            }
            /* Other PBUS PIO writes (keyboard, etc.) are silently ignored */
        }
        /* BBRAM (0x60000-0x7ffff) - only 8KB used, mirrored
         * DS1386 RTC registers have 4-byte spacing on SGI hardware.
         * Register index = byte_offset / 4
         */
        else if (base_addr >= 0x60000 && base_addr < 0x80000) {
            int byte_offset = (addr - 0x60000) & 0x1fff;  /* 8KB mask */
            int reg = byte_offset >> 2;  /* 4-byte spacing */
            trace_sgi_hpc3_nvram(reg, (uint8_t)val, 1);
            if (reg == RTC_COMMAND) {
                /* RTC command register */
                s->rtc_command = val;
            } else {
                s->bbram[reg] = val;
                sgi_hpc3_nvram_sync(s, reg);
                /* Recalculate time offset when time registers are written */
                if (reg >= RTC_SECONDS && reg <= RTC_YEAR) {
                    struct tm tm;
                    qemu_get_timedate(&tm, 0);

                    tm.tm_sec = bcd_to_bin(s->bbram[RTC_SECONDS] & 0x7f);
                    tm.tm_min = bcd_to_bin(s->bbram[RTC_MINUTES] & 0x7f);
                    tm.tm_hour = bcd_to_bin(s->bbram[RTC_HOURS] & 0x3f);
                    tm.tm_mday = bcd_to_bin(s->bbram[RTC_DATE] & 0x3f);
                    tm.tm_mon = bcd_to_bin(s->bbram[RTC_MONTH] & 0x1f) - 1;
                    tm.tm_year = bcd_to_bin(s->bbram[RTC_YEAR]) + 100;

                    s->rtc_time_offset = qemu_timedate_diff(&tm);
                }
            }
        }
        /* PBUS DMA channel registers (8 channels, stride 0x2000) */
        else if (base_addr < 8 * HPC3_PBUS_STRIDE) {
            int ch = base_addr / HPC3_PBUS_STRIDE;
            hwaddr ch_offset = base_addr % HPC3_PBUS_STRIDE;
            if (ch_offset == HPC3_PBUS_BP_BASE) {
                s->pbus_bp[ch] = val;
            } else if (ch_offset == HPC3_PBUS_DP_BASE) {
                s->pbus_dp[ch] = val;
            } else if (ch_offset == HPC3_PBUS_CTRL_BASE) {
                s->pbus_ctrl[ch] = val;
            }
            /* Other offsets in channel are silently ignored */
        }
        /* PBUS DMA config */
        else if (base_addr >= HPC3_PBUS_CFGDMA_BASE &&
            base_addr < HPC3_PBUS_CFGDMA_BASE + 8 * HPC3_PBUS_CFG_STRIDE) {
            int idx = (base_addr - HPC3_PBUS_CFGDMA_BASE) / HPC3_PBUS_CFG_STRIDE;
            s->pbus_dmacfg[idx] = val;
        }
        /* PBUS PIO config */
        else if (base_addr >= HPC3_PBUS_CFGPIO_BASE &&
                 base_addr < HPC3_PBUS_CFGPIO_BASE + 10 * HPC3_PBUS_CFG_STRIDE) {
            int idx = (base_addr - HPC3_PBUS_CFGPIO_BASE) / HPC3_PBUS_CFG_STRIDE;
            s->pbus_piocfg[idx] = val;
        }
        else {
            qemu_log_mask(LOG_UNIMP,
                          "sgi_hpc3: unimplemented write at 0x%05" HWADDR_PRIx
                          " value 0x%08" PRIx64 "\n", addr, val);
        }
        break;
    }
}

static const MemoryRegionOps sgi_hpc3_ops = {
    .read = sgi_hpc3_read,
    .write = sgi_hpc3_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sgi_hpc3_reset(DeviceState *dev)
{
    SGIHPC3State *s = SGI_HPC3(dev);

    s->intstat = 0;
    s->misc = 0;
    s->eeprom = 0;
    s->buserr_stat = 0;

    for (int i = 0; i < 2; i++) {
        s->scsi_cbp[i] = 0;
        s->scsi_nbdp[i] = 0;
        s->scsi_bc[i] = 0;
        s->scsi_ctrl[i] = 0;
        s->scsi_dmacfg[i] = 0;
        s->scsi_piocfg[i] = 0;
        s->scsi_dma_count[i] = 0;
        s->scsi_dma_active[i] = false;
        s->scsi_dma_to_device[i] = false;
        s->scsi_dma_big_endian[i] = false;
        s->scsi_drq[i] = false;
    }

    sgi_hpc3_enet_reset(s);

    /* SCSI controllers are reset by their own device reset */

    for (int i = 0; i < 8; i++) {
        s->pbus_dmacfg[i] = 0;
        s->pbus_bp[i] = 0;
        s->pbus_dp[i] = 0;
        s->pbus_ctrl[i] = 0;
    }
    for (int i = 0; i < 10; i++) {
        s->pbus_piocfg[i] = 0;
    }

    s->par_data = 0;
    s->par_control = 0;
    s->gen_control = 0;
    s->panel = 0x03;  /* Power on, supply enabled */
    /*
     * System ID register (HPC3_SYS_ID at 0x1fbd9858):
     *   Bits [7:5] = CHIP_REV (0x20 = IOC1)
     *   Bits [4:1] = BOARD_REV
     *   Bit  [0]   = BOARD_ID (1=Fullhouse/IP22, 0=Guinness/IP24)
     *
     * MAME reference values:
     *   Guinness (Indy/IP24): 0x26 = IOC1, board_rev=3, guinness
     *   Full House (Indigo2):  0x11 = no IOC1, board_rev=8, fullhouse
     */
    if (s->board_type == BOARD_IP22) {
        s->sysid = 0x11;  /* Full House: no IOC, board rev 8 */
    } else {
        s->sysid = 0x26;  /* Guinness: IOC1, board rev 3 */
    }
    s->read_reg = 0;
    s->dma_select = 0;
    s->write1 = 0x30;  /* LEDs off */
    s->write2 = 0x30;  /* RS232 mode */

    s->int3_local0_stat = 0;
    s->int3_local0_mask = 0;
    s->int3_local1_stat = 0;
    s->int3_local1_mask = 0;
    s->int3_map_status = 0;
    s->int3_map_mask0 = 0;
    s->int3_map_mask1 = 0;
    s->int3_map_polarity = 0;
    s->int3_error_status = 0;

    s->serial_cmd[0] = 0;
    s->serial_cmd[1] = 0;
    s->serial_data[0] = 0;
    s->serial_data[1] = 0;
    s->serial_reg_ptr[0] = 0;
    s->serial_reg_ptr[1] = 0;

    /* Reset serial RX FIFOs */
    for (int i = 0; i < 2; i++) {
        s->serial_rx_fifo_head[i] = 0;
        s->serial_rx_fifo_tail[i] = 0;
        s->serial_rx_fifo_count[i] = 0;
    }

    /* Reset 8254 PIT timers */
    for (int i = 0; i < 3; i++) {
        s->pit_count[i] = 0;
        s->pit_latch[i] = 0;
        s->pit_control[i] = 0;
        s->pit_rw_state[i] = 0;
        s->pit_latched[i] = false;
        s->pit_load_time[i] = 0;
    }
    if (s->pit_timer[0]) {
        timer_del(s->pit_timer[0]);
    }
    if (s->pit_timer[1]) {
        timer_del(s->pit_timer[1]);
    }
    /* Clear direct timer IRQ pending state */
    s->timer_pending[0] = false;
    s->timer_pending[1] = false;

    /*
     * Initialize 8042 keyboard controller.
     * The PS/2 devices are reset by QOM's reset mechanism automatically.
     * We only need to reset our 8042 controller-level state.
     * Default command byte: 0x47 = SYS | NO_KEYLOCK | KCC | KBD_INT
     */
    s->kbd_cmd = 0;
    s->kbd_cmd_byte = KBD_MODE_SYS | KBD_MODE_NO_KEYLOCK
                    | KBD_MODE_KCC | KBD_MODE_KBD_INT;
    s->kbd_write_to_mouse = false;
    s->kbd_irq_level = false;
    s->mouse_irq_level = false;
    s->kbd_pending_source = 0;

    /* Always set RTC registers from host clock */
    struct tm tm;
    qemu_get_timedate(&tm, 0);
    s->bbram[RTC_HUNDREDTHS] = 0;
    s->bbram[RTC_SECONDS] = bin_to_bcd(tm.tm_sec);
    s->bbram[RTC_MINUTES] = bin_to_bcd(tm.tm_min);
    s->bbram[RTC_HOURS] = bin_to_bcd(tm.tm_hour);
    s->bbram[RTC_DAY_OF_WEEK] = tm.tm_wday + 1;
    s->bbram[RTC_DATE] = bin_to_bcd(tm.tm_mday);
    s->bbram[RTC_MONTH] = bin_to_bcd(tm.tm_mon + 1);
    s->bbram[RTC_YEAR] = bin_to_bcd(tm.tm_year % 100);
    s->bbram[RTC_COMMAND] = 0x80;  /* TE=1, oscillator running */
    s->rtc_command = 0x80;
    s->rtc_time_offset = 0;

    /* If NVRAM wasn't loaded from file, initialize with defaults */
    if (!s->nvram_loaded) {
        sgi_hpc3_nvram_init_defaults(s);
    } else if (!s->autoload) {
        /*
         * Even with NVRAM loaded from file, override AutoLoad when
         * the property requests it.  Recompute checksum so the PROM
         * accepts the change.
         */
        uint8_t *table = &s->bbram[NVRAM_TABLE_BASE];
        table[NVOFF_AUTOLOAD] = 'N';
        table[NVOFF_CHECKSUM] = sgi_hpc3_nvram_checksum(table,
                                                         NVRAM_TABLE_SIZE);
    }

    /* Initialize serial EEPROM with NVRAM defaults (Full House path) */
    sgi_hpc3_eeprom_init_defaults(s);
}

static void sgi_hpc3_realize(DeviceState *dev, Error **errp)
{
    SGIHPC3State *s = SGI_HPC3(dev);

    /* Realize PS/2 keyboard and mouse devices */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2kbd), errp)) {
        return;
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ps2mouse), errp)) {
        return;
    }

    /* Connect PS/2 device IRQ outputs to our GPIO input handlers */
    qdev_connect_gpio_out(DEVICE(&s->ps2kbd), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in_named(dev, "ps2-kbd-irq", 0));
    qdev_connect_gpio_out(DEVICE(&s->ps2mouse), PS2_DEVICE_IRQ,
                          qdev_get_gpio_in_named(dev, "ps2-mouse-irq", 0));

    /* Create 93CS56 serial EEPROM (128 × 16-bit words) for Full House NVRAM */
    s->serial_eeprom = eeprom93xx_new(dev, 128);

    /*
     * Create WD33C93 SCSI controllers
     * SCSI channel 0 is always present. Channel 1 exists on Indigo2
     * (Full House) but not on Indy (Guinness). Register access code
     * null-checks s->scsi[1] so leaving it NULL is safe.
     */
    s->scsi[0] = WD33C93(qdev_new(TYPE_WD33C93));
    qdev_realize(DEVICE(s->scsi[0]), NULL, &error_fatal);

    if (s->board_type == BOARD_IP22) {
        s->scsi[1] = WD33C93(qdev_new(TYPE_WD33C93));
        qdev_realize(DEVICE(s->scsi[1]), NULL, &error_fatal);
    } else {
        s->scsi[1] = NULL;
    }

    /*
     * Wire WD33C93 IRQ outputs to HPC3 SCSI IRQ inputs
     * The HPC3 will aggregate these into INT3 and route to CPU
     */
    qdev_connect_gpio_out_named(DEVICE(s->scsi[0]), "irq", 0,
                                qdev_get_gpio_in_named(dev, "scsi-irq", 0));
    if (s->scsi[1]) {
        qdev_connect_gpio_out_named(DEVICE(s->scsi[1]), "irq", 0,
                                    qdev_get_gpio_in_named(dev, "scsi-irq", 1));
    }

    /*
     * Wire WD33C93 DRQ outputs to HPC3 SCSI DRQ inputs
     * DRQ signals trigger the HPC3 DMA engine to move data
     */
    qdev_connect_gpio_out_named(DEVICE(s->scsi[0]), "drq", 0,
                                qdev_get_gpio_in_named(dev, "scsi-drq", 0));
    if (s->scsi[1]) {
        qdev_connect_gpio_out_named(DEVICE(s->scsi[1]), "drq", 0,
                                    qdev_get_gpio_in_named(dev, "scsi-drq", 1));
    }

    /*
     * Create 8254 PIT timers for channels 0 and 1
     * Channel 2 is the master clock and doesn't generate interrupts
     */
    s->pit_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    sgi_hpc3_pit_timer0_cb, s);
    s->pit_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    sgi_hpc3_pit_timer1_cb, s);

    /* SCC TX completion timers (one per channel) */
    s->scc_tx_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       sgi_hpc3_scc_tx_timer_cb, s);
    s->scc_tx_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       sgi_hpc3_scc_tx_timer_cb, s);

    /* Create ethernet NIC (Seeq 80C03 via HPC3 DMA) */
    qemu_macaddr_default_if_unset(&s->enet_conf.macaddr);
    s->nic = qemu_new_nic(&sgi_hpc3_enet_net_info, &s->enet_conf,
                           object_get_typename(OBJECT(dev)), dev->id,
                           &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic),
                              s->enet_conf.macaddr.a);

    /* Register serial chardev receive handlers (for serial console input) */
    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_set_handlers(&s->serial,
                                  sgi_hpc3_serial_can_receive,
                                  sgi_hpc3_serial_receive,
                                  sgi_hpc3_serial_event,
                                  NULL, s, NULL, true);
    }

    /* NVRAM persistence: load from backing file if it exists */
    if (s->nvram_filename) {
        FILE *f = fopen(s->nvram_filename, "rb");
        if (f) {
            if (fread(s->bbram, sizeof(s->bbram), 1, f) == 1) {
                s->nvram_loaded = true;
                qemu_log_mask(LOG_UNIMP,
                              "sgi_hpc3: loaded NVRAM from %s\n",
                              s->nvram_filename);
            }
            fclose(f);
        }
    }
}

static void sgi_hpc3_init(Object *obj)
{
    SGIHPC3State *s = SGI_HPC3(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &sgi_hpc3_ops, s,
                          "sgi-hpc3", HPC3_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* Create GPIO inputs for SCSI interrupts (from WD33C93 controllers) */
    qdev_init_gpio_in_named(DEVICE(s), sgi_hpc3_scsi_irq, "scsi-irq", 2);

    /* Create GPIO inputs for SCSI DRQ (from WD33C93 controllers) */
    qdev_init_gpio_in_named(DEVICE(s), sgi_hpc3_scsi_drq, "scsi-drq", 2);

    /* Create GPIO input for GIO retrace/VBLANK interrupt (from Newport) */
    qdev_init_gpio_in_named(DEVICE(s), sgi_hpc3_gio_retrace_irq,
                            "gio-retrace", 1);

    /* Initialize PS/2 keyboard and mouse child devices */
    object_initialize_child(obj, "ps2kbd", &s->ps2kbd, TYPE_SGI_PS2_KBD);
    object_initialize_child(obj, "ps2mouse", &s->ps2mouse,
                            TYPE_PS2_MOUSE_DEVICE);

    /* GPIO inputs for PS/2 device IRQ callbacks */
    qdev_init_gpio_in_named(DEVICE(s), sgi_hpc3_ps2_kbd_irq,
                            "ps2-kbd-irq", 1);
    qdev_init_gpio_in_named(DEVICE(s), sgi_hpc3_ps2_mouse_irq,
                            "ps2-mouse-irq", 1);

    /* Create GPIO outputs for CPU interrupts (to MIPS CPU) */
    qdev_init_gpio_out_named(DEVICE(s), s->cpu_irq, "cpu-irq", 2);

    /* Create GPIO outputs for PIT timer interrupts (Timer0 → IP4, Timer1 → IP5) */
    qdev_init_gpio_out_named(DEVICE(s), s->timer_irq, "timer-irq", 2);
}

static const Property sgi_hpc3_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIHPC3State, serial),
    DEFINE_PROP_UINT8("board-type", SGIHPC3State, board_type, BOARD_IP24),
    DEFINE_PROP_UINT8("nvram-rev", SGIHPC3State, nvram_rev, 8),
    DEFINE_NIC_PROPERTIES(SGIHPC3State, enet_conf),
    DEFINE_PROP_STRING("nvram-file", SGIHPC3State, nvram_filename),
    DEFINE_PROP_BOOL("autoload", SGIHPC3State, autoload, true),
};

static const VMStateDescription vmstate_sgi_hpc3 = {
    .name = "sgi-hpc3",
    .version_id = 6,
    .minimum_version_id = 6,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(bbram, SGIHPC3State, 8192),
        VMSTATE_UINT32(intstat, SGIHPC3State),
        VMSTATE_UINT32(misc, SGIHPC3State),
        VMSTATE_UINT32(eeprom, SGIHPC3State),
        VMSTATE_UINT32(buserr_stat, SGIHPC3State),
        VMSTATE_UINT32_ARRAY(scsi_cbp, SGIHPC3State, 2),
        VMSTATE_UINT32_ARRAY(scsi_nbdp, SGIHPC3State, 2),
        VMSTATE_UINT32_ARRAY(scsi_bc, SGIHPC3State, 2),
        VMSTATE_UINT32_ARRAY(scsi_ctrl, SGIHPC3State, 2),
        VMSTATE_UINT32_ARRAY(scsi_dmacfg, SGIHPC3State, 2),
        VMSTATE_UINT32_ARRAY(scsi_piocfg, SGIHPC3State, 2),
        VMSTATE_UINT16_ARRAY(scsi_dma_count, SGIHPC3State, 2),
        VMSTATE_BOOL_ARRAY(scsi_dma_active, SGIHPC3State, 2),
        VMSTATE_BOOL_ARRAY(scsi_dma_to_device, SGIHPC3State, 2),
        VMSTATE_BOOL_ARRAY(scsi_dma_big_endian, SGIHPC3State, 2),
        VMSTATE_BOOL_ARRAY(scsi_drq, SGIHPC3State, 2),
        VMSTATE_UINT32(enet_rx_cbp, SGIHPC3State),
        VMSTATE_UINT32(enet_rx_nbdp, SGIHPC3State),
        VMSTATE_UINT32(enet_rx_bc, SGIHPC3State),
        VMSTATE_UINT32(enet_rx_ctrl, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_cbp, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_nbdp, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_bc, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_ctrl, SGIHPC3State),
        VMSTATE_UINT32(enet_misc, SGIHPC3State),
        VMSTATE_UINT32(enet_dmacfg, SGIHPC3State),
        VMSTATE_UINT32(enet_piocfg, SGIHPC3State),
        VMSTATE_UINT32(enet_rx_cbdp, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_cpfbdp, SGIHPC3State),
        VMSTATE_UINT32(enet_tx_ppfbdp, SGIHPC3State),
        VMSTATE_UINT8_ARRAY(seeq_station_addr, SGIHPC3State, 6),
        VMSTATE_UINT8(seeq_rx_cmd, SGIHPC3State),
        VMSTATE_UINT8(seeq_tx_cmd, SGIHPC3State),
        VMSTATE_UINT8(seeq_rx_status, SGIHPC3State),
        VMSTATE_UINT8(seeq_tx_status, SGIHPC3State),
        VMSTATE_UINT32_ARRAY(pbus_dmacfg, SGIHPC3State, 8),
        VMSTATE_UINT32_ARRAY(pbus_piocfg, SGIHPC3State, 10),
        VMSTATE_UINT32_ARRAY(pbus_bp, SGIHPC3State, 8),
        VMSTATE_UINT32_ARRAY(pbus_dp, SGIHPC3State, 8),
        VMSTATE_UINT32_ARRAY(pbus_ctrl, SGIHPC3State, 8),
        VMSTATE_UINT32(par_data, SGIHPC3State),
        VMSTATE_UINT32(par_control, SGIHPC3State),
        VMSTATE_UINT32(gen_control, SGIHPC3State),
        VMSTATE_UINT32(panel, SGIHPC3State),
        VMSTATE_UINT32(sysid, SGIHPC3State),
        VMSTATE_UINT8(board_type, SGIHPC3State),
        VMSTATE_UINT32(read_reg, SGIHPC3State),
        VMSTATE_UINT32(dma_select, SGIHPC3State),
        VMSTATE_UINT32(write1, SGIHPC3State),
        VMSTATE_UINT32(write2, SGIHPC3State),
        VMSTATE_UINT32(int3_local0_stat, SGIHPC3State),
        VMSTATE_UINT32(int3_local0_mask, SGIHPC3State),
        VMSTATE_UINT32(int3_local1_stat, SGIHPC3State),
        VMSTATE_UINT32(int3_local1_mask, SGIHPC3State),
        VMSTATE_UINT32(int3_map_status, SGIHPC3State),
        VMSTATE_UINT32(int3_map_mask0, SGIHPC3State),
        VMSTATE_UINT32(int3_map_mask1, SGIHPC3State),
        VMSTATE_UINT32(int3_map_polarity, SGIHPC3State),
        VMSTATE_UINT32(int3_error_status, SGIHPC3State),
        VMSTATE_UINT8_ARRAY(serial_cmd, SGIHPC3State, 2),
        VMSTATE_UINT8_ARRAY(serial_data, SGIHPC3State, 2),
        VMSTATE_INT64(rtc_time_offset, SGIHPC3State),
        VMSTATE_UINT16_ARRAY(pit_count, SGIHPC3State, 3),
        VMSTATE_UINT16_ARRAY(pit_latch, SGIHPC3State, 3),
        VMSTATE_UINT8_ARRAY(pit_control, SGIHPC3State, 3),
        VMSTATE_UINT8_ARRAY(pit_rw_state, SGIHPC3State, 3),
        VMSTATE_INT64_ARRAY(pit_load_time, SGIHPC3State, 3),
        VMSTATE_UINT8(kbd_cmd, SGIHPC3State),
        VMSTATE_UINT8(kbd_cmd_byte, SGIHPC3State),
        VMSTATE_BOOL(kbd_write_to_mouse, SGIHPC3State),
        VMSTATE_BOOL(kbd_irq_level, SGIHPC3State),
        VMSTATE_BOOL(mouse_irq_level, SGIHPC3State),
        VMSTATE_UINT8(kbd_pending_source, SGIHPC3State),
        VMSTATE_BOOL_ARRAY(timer_pending, SGIHPC3State, 2),
        VMSTATE_UINT32(hal2_isr, SGIHPC3State),
        VMSTATE_UINT32(hal2_iar, SGIHPC3State),
        VMSTATE_UINT32_ARRAY(hal2_idr, SGIHPC3State, 4),
        VMSTATE_UINT8(hal2_volume_left, SGIHPC3State),
        VMSTATE_UINT8(hal2_volume_right, SGIHPC3State),
        VMSTATE_END_OF_LIST()
    }
};

static void sgi_hpc3_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_hpc3_realize;
    device_class_set_legacy_reset(dc, sgi_hpc3_reset);
    dc->vmsd = &vmstate_sgi_hpc3;
    device_class_set_props(dc, sgi_hpc3_properties);
}

static const TypeInfo sgi_hpc3_info = {
    .name          = TYPE_SGI_HPC3,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHPC3State),
    .instance_init = sgi_hpc3_init,
    .class_init    = sgi_hpc3_class_init,
};

/* -----------------------------------------------------------------------
 * SGI PS/2 keyboard subtype — hardware typematic repeat
 * -----------------------------------------------------------------------
 *
 * The standard QEMU ps2_keyboard_event() forwards ALL host key events
 * (including host-OS autorepeat) to the PS/2 scancode queue.  Under
 * -icount shift=0,sleep=off the guest virtual clock races far ahead of
 * real time, causing the X11 server's autorepeat timer to fire many
 * times per physical key press and producing unwanted repeated chars.
 *
 * This SGI-specific subtype of TYPE_PS2_KBD_DEVICE replaces the input
 * event handler to implement PS/2 typematic behaviour driven by
 * QEMU_CLOCK_REALTIME:
 *   - Host OS repeat events are suppressed (typematic_qcode already set).
 *   - QEMU fires its own REALTIME-timed repeat after typematic_delay_ms,
 *     then at typematic_period_ms intervals (~10.9 Hz, 500 ms delay).
 *   - On key-up the timer is cancelled and typematic_qcode cleared.
 *
 * Default rates: 500 ms initial delay, 91 ms period (~10.9 Hz, per
 * PS/2 spec default 0x2B encoding).
 */

/* Modifier bit flags — mirror of the file-scope values in ps2.c */
#define SGI_PS2_MOD_CTRL_L  (1u << 0)
#define SGI_PS2_MOD_SHIFT_L (1u << 1)
#define SGI_PS2_MOD_ALT_L   (1u << 2)
#define SGI_PS2_MOD_CTRL_R  (1u << 3)
#define SGI_PS2_MOD_SHIFT_R (1u << 4)
#define SGI_PS2_MOD_ALT_R   (1u << 5)

/*
 * AT set-2 → XT set-1 translation table.
 * Mirrors translate_table[] from qemu/hw/input/ps2.c.
 */
static const uint8_t sgi_ps2_translate[256] = {
    0xff, 0x43, 0x41, 0x3f, 0x3d, 0x3b, 0x3c, 0x58,
    0x64, 0x44, 0x42, 0x40, 0x3e, 0x0f, 0x29, 0x59,
    0x65, 0x38, 0x2a, 0x70, 0x1d, 0x10, 0x02, 0x5a,
    0x66, 0x71, 0x2c, 0x1f, 0x1e, 0x11, 0x03, 0x5b,
    0x67, 0x2e, 0x2d, 0x20, 0x12, 0x05, 0x04, 0x5c,
    0x68, 0x39, 0x2f, 0x21, 0x14, 0x13, 0x06, 0x5d,
    0x69, 0x31, 0x30, 0x23, 0x22, 0x15, 0x07, 0x5e,
    0x6a, 0x72, 0x32, 0x24, 0x16, 0x08, 0x09, 0x5f,
    0x6b, 0x33, 0x25, 0x17, 0x18, 0x0b, 0x0a, 0x60,
    0x6c, 0x34, 0x35, 0x26, 0x27, 0x19, 0x0c, 0x61,
    0x6d, 0x73, 0x28, 0x74, 0x1a, 0x0d, 0x62, 0x6e,
    0x3a, 0x36, 0x1c, 0x1b, 0x75, 0x2b, 0x63, 0x76,
    0x55, 0x56, 0x77, 0x78, 0x79, 0x7a, 0x0e, 0x7b,
    0x7c, 0x4f, 0x7d, 0x4b, 0x47, 0x7e, 0x7f, 0x6f,
    0x52, 0x53, 0x50, 0x4c, 0x4d, 0x48, 0x01, 0x45,
    0x57, 0x4e, 0x51, 0x4a, 0x37, 0x49, 0x46, 0x54,
    0x80, 0x81, 0x82, 0x41, 0x54, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
    0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
    0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
    0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7,
    0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
    0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7,
    0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
    0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
    0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
    0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
    0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
};

/* Return the modifier bitmask for a given qcode (0 for non-modifiers). */
static unsigned int sgi_ps2_modifier_bit(QKeyCode key)
{
    switch (key) {
    case Q_KEY_CODE_CTRL:    return SGI_PS2_MOD_CTRL_L;
    case Q_KEY_CODE_CTRL_R:  return SGI_PS2_MOD_CTRL_R;
    case Q_KEY_CODE_SHIFT:   return SGI_PS2_MOD_SHIFT_L;
    case Q_KEY_CODE_SHIFT_R: return SGI_PS2_MOD_SHIFT_R;
    case Q_KEY_CODE_ALT:     return SGI_PS2_MOD_ALT_L;
    case Q_KEY_CODE_ALT_R:   return SGI_PS2_MOD_ALT_R;
    default:                 return 0;
    }
}

/*
 * Queue one raw PS/2 byte, applying XT translation if translate mode is on.
 * Mirrors ps2_put_keycode() from ps2.c.
 */
static void sgi_ps2_put_keycode(PS2KbdState *ps2, int keycode)
{
    PS2State *ps = PS2_DEVICE(ps2);

    if (ps2->translate) {
        if (keycode == 0xf0) {
            ps2->need_high_bit = true;
        } else if (ps2->need_high_bit) {
            ps2_queue(ps, sgi_ps2_translate[keycode] | 0x80);
            ps2->need_high_bit = false;
        } else {
            ps2_queue(ps, sgi_ps2_translate[keycode]);
        }
    } else {
        ps2_queue(ps, keycode);
    }
}

/*
 * Emit a PS/2 make code for qcode (used by the typematic timer callback).
 * PAUSE and PRINT are skipped — they do not repeat on real hardware.
 */
static void sgi_ps2_emit_make(SGIPs2KbdState *s, int qcode)
{
    PS2KbdState *ps2 = &s->parent_obj;
    uint16_t keycode = 0;

    if (qcode == Q_KEY_CODE_PAUSE || qcode == Q_KEY_CODE_PRINT) {
        return;
    }

    if (ps2->scancode_set == 1) {
        if (qcode < (int)qemu_input_map_qcode_to_atset1_len) {
            keycode = qemu_input_map_qcode_to_atset1[qcode];
        }
        if (keycode) {
            if (keycode & 0xff00) {
                sgi_ps2_put_keycode(ps2, keycode >> 8);
            }
            sgi_ps2_put_keycode(ps2, keycode & 0xff);
        }
    } else if (ps2->scancode_set == 2) {
        if (qcode < (int)qemu_input_map_qcode_to_atset2_len) {
            keycode = qemu_input_map_qcode_to_atset2[qcode];
        }
        if (keycode) {
            if (keycode & 0xff00) {
                sgi_ps2_put_keycode(ps2, keycode >> 8);
            }
            sgi_ps2_put_keycode(ps2, keycode & 0xff);
        }
    } else if (ps2->scancode_set == 3) {
        if (qcode < (int)qemu_input_map_qcode_to_atset3_len) {
            keycode = qemu_input_map_qcode_to_atset3[qcode];
        }
        if (keycode) {
            ps2_queue(PS2_DEVICE(ps2), keycode);
        }
    }
}

/* Typematic repeat timer callback — fires on QEMU_CLOCK_REALTIME. */
static void sgi_ps2_kbd_typematic(void *opaque)
{
    SGIPs2KbdState *s = opaque;

    if (s->typematic_qcode < 0) {
        return;
    }
    sgi_ps2_emit_make(s, s->typematic_qcode);
    timer_mod(s->typematic_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
              (int64_t)s->typematic_period_ms * SCALE_MS);
}

/*
 * SGI PS/2 keyboard event handler with typematic support.
 *
 * This replaces the upstream ps2_keyboard_event() and implements the
 * full scancode emission for all three scancode sets, plus:
 *   - Host OS repeat events are suppressed.
 *   - A REALTIME timer drives repeat at the programmed typematic rate.
 */
static void sgi_ps2_keyboard_event(DeviceState *dev, QemuConsole *src,
                                   InputEvent *evt)
{
    SGIPs2KbdState *s = (SGIPs2KbdState *)dev;
    PS2KbdState *ps2 = &s->parent_obj;
    InputKeyEvent *key = evt->u.key.data;
    int qcode;
    uint16_t keycode = 0;
    unsigned int mod;

    if (!ps2->scan_enabled) {
        return;
    }

    qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    assert(evt->type == INPUT_EVENT_KIND_KEY);
    qcode = qemu_input_key_value_to_qcode(key->key);
    mod = sgi_ps2_modifier_bit(qcode);

    if (key->down) {
        ps2->modifiers |= mod;

        /*
         * Suppress host OS repeat events: if this qcode is already the
         * held key, our REALTIME timer is already generating repeats.
         */
        if (s->typematic_qcode == qcode) {
            return;
        }

        /* New key: cancel any existing typematic and record the new key. */
        timer_del(s->typematic_timer);
        s->typematic_qcode = qcode;

        /* Emit initial make code (full logic for all three scancode sets). */
        if (ps2->scancode_set == 1) {
            if (qcode == Q_KEY_CODE_PAUSE) {
                if (ps2->modifiers & (SGI_PS2_MOD_CTRL_L | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x46);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xc6);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe1);
                    sgi_ps2_put_keycode(ps2, 0x1d);
                    sgi_ps2_put_keycode(ps2, 0x45);
                    sgi_ps2_put_keycode(ps2, 0xe1);
                    sgi_ps2_put_keycode(ps2, 0x9d);
                    sgi_ps2_put_keycode(ps2, 0xc5);
                }
            } else if (qcode == Q_KEY_CODE_PRINT) {
                if (ps2->modifiers & SGI_PS2_MOD_ALT_L) {
                    sgi_ps2_put_keycode(ps2, 0xb8);
                    sgi_ps2_put_keycode(ps2, 0x38);
                    sgi_ps2_put_keycode(ps2, 0x54);
                } else if (ps2->modifiers & SGI_PS2_MOD_ALT_R) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xb8);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x38);
                    sgi_ps2_put_keycode(ps2, 0x54);
                } else if (ps2->modifiers & (SGI_PS2_MOD_SHIFT_L | SGI_PS2_MOD_CTRL_L |
                                             SGI_PS2_MOD_SHIFT_R | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x37);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x2a);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x37);
                }
            } else if (qcode == Q_KEY_CODE_LANG1 || qcode == Q_KEY_CODE_LANG2) {
                /* make only, no break emitted on key-up */
                if (qcode < (int)qemu_input_map_qcode_to_atset1_len) {
                    keycode = qemu_input_map_qcode_to_atset1[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                }
            } else {
                if (qcode < (int)qemu_input_map_qcode_to_atset1_len) {
                    keycode = qemu_input_map_qcode_to_atset1[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                } else {
                    qemu_log_mask(LOG_UNIMP,
                                  "sgi-ps2: ignoring key with qcode %d\n", qcode);
                }
            }
        } else if (ps2->scancode_set == 2) {
            if (qcode == Q_KEY_CODE_PAUSE) {
                if (ps2->modifiers & (SGI_PS2_MOD_CTRL_L | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x7e);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x7e);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe1);
                    sgi_ps2_put_keycode(ps2, 0x14);
                    sgi_ps2_put_keycode(ps2, 0x77);
                    sgi_ps2_put_keycode(ps2, 0xe1);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x14);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x77);
                }
            } else if (qcode == Q_KEY_CODE_PRINT) {
                if (ps2->modifiers & SGI_PS2_MOD_ALT_L) {
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0x84);
                } else if (ps2->modifiers & SGI_PS2_MOD_ALT_R) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0x84);
                } else if (ps2->modifiers & (SGI_PS2_MOD_SHIFT_L | SGI_PS2_MOD_CTRL_L |
                                             SGI_PS2_MOD_SHIFT_R | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x7c);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x12);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x7c);
                }
            } else if (qcode == Q_KEY_CODE_LANG1 || qcode == Q_KEY_CODE_LANG2) {
                /* make only */
                if (qcode < (int)qemu_input_map_qcode_to_atset2_len) {
                    keycode = qemu_input_map_qcode_to_atset2[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                }
            } else {
                if (qcode < (int)qemu_input_map_qcode_to_atset2_len) {
                    keycode = qemu_input_map_qcode_to_atset2[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                } else {
                    qemu_log_mask(LOG_UNIMP,
                                  "sgi-ps2: ignoring key with qcode %d\n", qcode);
                }
            }
        } else if (ps2->scancode_set == 3) {
            if (qcode < (int)qemu_input_map_qcode_to_atset3_len) {
                keycode = qemu_input_map_qcode_to_atset3[qcode];
            }
            if (keycode) {
                ps2_queue(PS2_DEVICE(ps2), keycode);
            } else {
                qemu_log_mask(LOG_UNIMP,
                              "sgi-ps2: ignoring key with qcode %d\n", qcode);
            }
        }

        /* Start typematic delay timer (PAUSE and PRINT do not repeat). */
        if (qcode != Q_KEY_CODE_PAUSE && qcode != Q_KEY_CODE_PRINT) {
            timer_mod(s->typematic_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                      (int64_t)s->typematic_delay_ms * SCALE_MS);
        }
    } else {
        /* Key up: cancel typematic and emit break code. */
        ps2->modifiers &= ~mod;

        if (s->typematic_qcode == qcode) {
            timer_del(s->typematic_timer);
            s->typematic_qcode = -1;
        }

        if (ps2->scancode_set == 1) {
            if (qcode == Q_KEY_CODE_PAUSE) {
                /* No break code for PAUSE in set 1 */
            } else if (qcode == Q_KEY_CODE_PRINT) {
                if (ps2->modifiers & SGI_PS2_MOD_ALT_L) {
                    sgi_ps2_put_keycode(ps2, 0xd4);
                    sgi_ps2_put_keycode(ps2, 0xb8);
                    sgi_ps2_put_keycode(ps2, 0x38);
                } else if (ps2->modifiers & SGI_PS2_MOD_ALT_R) {
                    sgi_ps2_put_keycode(ps2, 0xd4);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xb8);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x38);
                } else if (ps2->modifiers & (SGI_PS2_MOD_SHIFT_L | SGI_PS2_MOD_CTRL_L |
                                             SGI_PS2_MOD_SHIFT_R | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xb7);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xb7);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xaa);
                }
            } else if (qcode == Q_KEY_CODE_LANG1 || qcode == Q_KEY_CODE_LANG2) {
                /* Ignore release for these keys */
            } else {
                if (qcode < (int)qemu_input_map_qcode_to_atset1_len) {
                    keycode = qemu_input_map_qcode_to_atset1[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    keycode |= 0x80;
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                } else {
                    qemu_log_mask(LOG_UNIMP,
                                  "sgi-ps2: ignoring key with qcode %d\n", qcode);
                }
            }
        } else if (ps2->scancode_set == 2) {
            if (qcode == Q_KEY_CODE_PAUSE) {
                /* No break code for PAUSE in set 2 */
            } else if (qcode == Q_KEY_CODE_PRINT) {
                if (ps2->modifiers & SGI_PS2_MOD_ALT_L) {
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x84);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0x11);
                } else if (ps2->modifiers & SGI_PS2_MOD_ALT_R) {
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x84);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0x11);
                } else if (ps2->modifiers & (SGI_PS2_MOD_SHIFT_L | SGI_PS2_MOD_CTRL_L |
                                             SGI_PS2_MOD_SHIFT_R | SGI_PS2_MOD_CTRL_R)) {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x7c);
                } else {
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x7c);
                    sgi_ps2_put_keycode(ps2, 0xe0);
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, 0x12);
                }
            } else if (qcode == Q_KEY_CODE_LANG1 || qcode == Q_KEY_CODE_LANG2) {
                /* Ignore release for these keys */
            } else {
                if (qcode < (int)qemu_input_map_qcode_to_atset2_len) {
                    keycode = qemu_input_map_qcode_to_atset2[qcode];
                }
                if (keycode) {
                    if (keycode & 0xff00) {
                        sgi_ps2_put_keycode(ps2, keycode >> 8);
                    }
                    sgi_ps2_put_keycode(ps2, 0xf0);
                    sgi_ps2_put_keycode(ps2, keycode & 0xff);
                } else {
                    qemu_log_mask(LOG_UNIMP,
                                  "sgi-ps2: ignoring key with qcode %d\n", qcode);
                }
            }
        } else if (ps2->scancode_set == 3) {
            if (qcode < (int)qemu_input_map_qcode_to_atset3_len) {
                keycode = qemu_input_map_qcode_to_atset3[qcode];
            }
            if (keycode) {
                ps2_queue(PS2_DEVICE(ps2), 0xf0);
                ps2_queue(PS2_DEVICE(ps2), keycode);
            } else {
                qemu_log_mask(LOG_UNIMP,
                              "sgi-ps2: ignoring key with qcode %d\n", qcode);
            }
        }
    }
}

static const QemuInputHandler sgi_ps2_keyboard_handler = {
    .name  = "SGI PS/2 Keyboard (typematic)",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = sgi_ps2_keyboard_event,
};

/*
 * Static capture of parent hold function pointer. We cannot use
 * PS2_DEVICE_GET_CLASS(obj)->parent_phases because that macro always returns
 * the leaf class object; when the parent (ps2_kbd_reset_hold) then calls
 * PS2_DEVICE_GET_CLASS(obj)->parent_phases.hold it retrieves the same pointer
 * again — infinite recursion. Instead we capture rc->phases.hold once at
 * class_init time (before we override it) and call it directly here.
 */
static ResettableHoldPhase sgi_ps2_parent_hold_fn;

/* Reset hold: call upstream ps2_kbd_reset_hold, then reset typematic state. */
static void sgi_ps2_kbd_reset_hold(Object *obj, ResetType type)
{
    SGIPs2KbdState *s = (SGIPs2KbdState *)obj;

    if (sgi_ps2_parent_hold_fn) {
        sgi_ps2_parent_hold_fn(obj, type);
    }
    timer_del(s->typematic_timer);
    s->typematic_qcode    = -1;
    s->typematic_delay_ms  = 500;
    s->typematic_period_ms = 91;
}

static void sgi_ps2_kbd_realize(DeviceState *dev, Error **errp)
{
    SGIPs2KbdState *s = (SGIPs2KbdState *)dev;

    s->typematic_qcode    = -1;
    s->typematic_delay_ms  = 500;
    s->typematic_period_ms = 91;
    s->typematic_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                      sgi_ps2_kbd_typematic, s);
    /* Register our typematic-aware handler instead of the parent's. */
    qemu_input_handler_register(dev, &sgi_ps2_keyboard_handler);
}

static void sgi_ps2_kbd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = sgi_ps2_kbd_realize;
    /* Capture the parent hold (ps2_kbd_reset_hold) before overriding.
     * We do NOT use resettable_class_set_parent_phases because it stores the
     * saved pointer in ps2dc->parent_phases, which ps2_kbd_reset_hold then
     * looks up via PS2_DEVICE_GET_CLASS(obj) — always the leaf class — causing
     * infinite recursion. A file-static capture breaks the cycle cleanly. */
    sgi_ps2_parent_hold_fn = rc->phases.hold;
    rc->phases.hold = sgi_ps2_kbd_reset_hold;
}

static const TypeInfo sgi_ps2_kbd_info = {
    .name          = TYPE_SGI_PS2_KBD,
    .parent        = TYPE_PS2_KBD_DEVICE,
    .instance_size = sizeof(SGIPs2KbdState),
    .class_init    = sgi_ps2_kbd_class_init,
};

static void sgi_hpc3_register_types(void)
{
    type_register_static(&sgi_ps2_kbd_info);
    type_register_static(&sgi_hpc3_info);
}

type_init(sgi_hpc3_register_types)
