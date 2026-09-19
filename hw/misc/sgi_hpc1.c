/*
 * SGI HPC1.5 (High Performance Peripheral Controller) for IP20 (Indigo)
 *
 * Models the HPC1.5 register window at 0x1fb80000: the core DMA/status
 * registers, the SEEQ 8003 ethernet register file, the WD33C93 SCSI
 * indirect interface, the INT2 interrupt multiplexor with its 8254 PIT,
 * three Z85C30 DUARTs (DUART2 channel A is the system console) and the
 * 93C56 serial EEPROM / NVRAM accessed through the aux bit-bang register.
 *
 * References:
 *   - MAME src/mame/sgi/ip20.cpp, hpc1.cpp, int2.cpp
 *   - IP20 PROM source: stand/arcs/IP20prom, stand/arcs/lib/libsk/ml
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */

#include "hw/misc/sgi_hpc1.h"
#include "chardev/char-fe.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define HPC1_MMIO_SIZE 0x10000

/* INT2 register indices (32-bit registers, index = byte offset >> 2) */
#define INT2_LIO0_STATUS  0
#define INT2_LIO0_MASK    1
#define INT2_LIO1_STATUS   2
#define INT2_LIO1_MASK     3
#define INT2_VME_STATUS    4
#define INT2_VME0_MASK     5
#define INT2_VME1_MASK     6
#define INT2_CONFIG        7
#define INT2_TIMER_ACK     8
#define INT2_PIT_BASE      0xc

/* INT2 LIO0 input bit assignments (MAME int2.h) */
#define LIO0_SCSI      0x04
#define LIO0_ETHERNET  0x08
#define LIO0_DUART     0x20

/* PIT clock: 10 MHz crystal / 10 = 1 MHz (MAME int2.cpp set_clk) */
#define PIT_CLOCK_HZ   10000000

/* ------------------------------------------------------------------ */
/* Z85C30 DUART                                                        */
/* ------------------------------------------------------------------ */

#define SCC_RX_IP      0x04  /* channel B / generic RX char available */
#define SCC_RX_IP_A    0x20
#define SCC_TX_IP      0x02
#define SCC_TX_IP_A    0x10

static int scc_console_channel(const SGIHPC1State *s, int d, int c)
{
    /* DUART2 channel A is the console; DUART0 channel A carries the
     * PROM's early low-level output. */
    return (d == 2 && c == 0) || (d == 0 && c == 0);
}

static void scc_update_irq(SGIHPC1State *s);

static void scc_tx(SGIHPC1State *s, int d, int c, uint8_t data)
{
    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_write_all(&s->serial, &data, 1);
    }
    (void)scc_console_channel(s, d, c);
}

static void scc_ctrl_write(SGIHPC1State *s, int d, int c, uint8_t val)
{
    SGIHPC1Uart *u = &s->uart[d][c];

    if (u->reg_ptr == 0) {
        uint8_t cmd = (val >> 3) & 0x07;
        uint8_t ptr = val & 0x07;

        switch (cmd) {
        case 1: /* Point High */
            ptr |= 0x08;
            break;
        case 2: /* Reset External/Status interrupts */
            u->rr3 &= ~(c == 0 ? 0x08 : 0x01);
            scc_update_irq(s);
            break;
        case 5: /* Reset TX interrupt pending */
            u->rr3 &= ~(c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
            scc_update_irq(s);
            break;
        default:
            break;
        }
        u->reg_ptr = ptr;
        return;
    }

    switch (u->reg_ptr) {
    case 1: /* WR1: interrupt enables */
        u->wr[1] = val;
        if (val & 0x02) {
            u->rr3 |= (c == 0 ? SCC_TX_IP_A : SCC_TX_IP);
            scc_update_irq(s);
        }
        break;
    case 5: /* WR5: TX parameters */
        u->wr[5] = val;
        break;
    case 8: /* WR8: transmit buffer */
        scc_tx(s, d, c, val);
        break;
    case 9: /* WR9: master interrupt control (shared across the DUART) */
        s->uart[d][0].wr[9] = val;
        s->uart[d][1].wr[9] = val;
        if (val & 0xc0) {
            if (val & 0x80) {
                s->uart[d][0].rr3 = 0;
                s->uart[d][0].wr[1] = 0;
            }
            if (val & 0x40) {
                s->uart[d][1].rr3 = 0;
                s->uart[d][1].wr[1] = 0;
            }
        }
        scc_update_irq(s);
        break;
    default:
        u->wr[u->reg_ptr] = val;
        break;
    }
    u->reg_ptr = 0;
}

static uint8_t scc_ctrl_read(SGIHPC1State *s, int d, int c)
{
    SGIHPC1Uart *u = &s->uart[d][c];
    uint8_t val;

    switch (u->reg_ptr) {
    case 1: /* RR1 */
        val = 0x01; /* all sent */
        break;
    case 2: /* RR2 interrupt vector */
        val = 0;
        break;
    case 3: /* RR3 only valid on channel A */
        val = (c == 0) ? u->rr3 : 0;
        break;
    default: /* RR0 status */
        val = 0x2c; /* TX empty, DCD, CTS */
        if (u->rx_count > 0) {
            val |= 0x01;
        }
        break;
    }
    u->reg_ptr = 0;
    return val;
}

static uint8_t scc_data_read(SGIHPC1State *s, int d, int c)
{
    SGIHPC1Uart *u = &s->uart[d][c];
    uint8_t val = 0;

    if (u->rx_count > 0) {
        val = u->rx_fifo[u->rx_tail];
        u->rx_tail = (u->rx_tail + 1) % HPC1_RX_FIFO_SIZE;
        u->rx_count--;
        qemu_chr_fe_accept_input(&s->serial);
        if (u->rx_count == 0) {
            u->rr3 &= ~(c == 0 ? SCC_RX_IP_A : SCC_RX_IP);
            scc_update_irq(s);
        }
    }
    return val;
}

static void scc_data_write(SGIHPC1State *s, int d, int c, uint8_t val)
{
    scc_tx(s, d, c, val);
}

/* ------------------------------------------------------------------ */
/* Interrupt aggregation                                               */
/* ------------------------------------------------------------------ */

static void int2_update(SGIHPC1State *s)
{
    qemu_set_irq(s->cpu_irq[0], (s->lio_status[0] & s->lio_mask[0]) != 0);
    qemu_set_irq(s->cpu_irq[1], (s->lio_status[1] & s->lio_mask[1]) != 0);
}

static void scc_update_irq(SGIHPC1State *s)
{
    bool pending = false;
    int d, c;

    for (d = 0; d < HPC1_NUM_DUARTS; d++) {
        for (c = 0; c < HPC1_DUART_CH; c++) {
            SGIHPC1Uart *u = &s->uart[d][c];
            if (u->rr3 && (s->uart[d][0].wr[9] & 0x08)) {
                pending = true;
            }
        }
    }

    if (pending) {
        s->lio_status[0] |= LIO0_DUART;
    } else {
        s->lio_status[0] &= ~LIO0_DUART;
    }
    int2_update(s);
}

static void hpc1_scsi_irq(void *opaque, int n, int level)
{
    SGIHPC1State *s = SGI_HPC1(opaque);

    if (level) {
        s->lio_status[0] |= LIO0_SCSI;
    } else {
        s->lio_status[0] &= ~LIO0_SCSI;
    }
    int2_update(s);
}

/* ------------------------------------------------------------------ */
/* 8254 PIT inside INT2                                                */
/* ------------------------------------------------------------------ */

static void hpc1_pit_timer_cb(void *opaque)
{
    SGIHPC1State *s = opaque;
    int ch;
    int64_t period_ns;

    for (ch = 0; ch < 2; ch++) {
        if (!timer_pending(s->pit_timer[ch])) {
            continue;
        }
        /*
         * Assert the corresponding CPU interrupt via the dedicated
         * timer lines (timer0 -> IP4, timer1 -> IP5).
         */
        if (s->pit_programmed[ch] && s->pit_count[ch] > 0) {
            qemu_irq_pulse(s->timer_irq[ch]);
            period_ns = (int64_t)s->pit_count[ch] * (1000000000LL / PIT_CLOCK_HZ);
            if (period_ns < 1000) {
                period_ns = 1000;
            }
            timer_mod(s->pit_timer[ch],
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
        }
    }
}

static uint16_t hpc1_pit_remaining(SGIHPC1State *s, int ch)
{
    uint16_t reload = s->pit_count[ch];
    int64_t elapsed_ns;
    uint64_t ticks;
    unsigned mode = (s->pit_control[ch] >> 1) & 0x7;

    if (!s->pit_programmed[ch] || reload == 0) {
        return reload;
    }

    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->pit_load_ns[ch];
    if (elapsed_ns < 0) {
        elapsed_ns = 0;
    }
    ticks = elapsed_ns / (1000000000ULL / PIT_CLOCK_HZ);

    /*
     * TCG executes the PROM's 1024-instruction calibration loop in well
     * under one 1 MHz PIT tick, so a strict real-time counter can read the
     * full reload and make the PROM's speed calibration compute "0 ticks".
     * The real part always sees at least one tick over that loop, so floor
     * the elapsed time at one tick once the counter has been started.
     */
    if (ticks == 0) {
        ticks = 1;
    }

    if (mode == 2 || mode == 3) {
        /* Rate generator: wrap around. */
        ticks %= reload;
        return reload - ticks;
    }
    /* One-shot modes: stop at zero. */
    if (ticks >= reload) {
        return 0;
    }
    return reload - ticks;
}

static void hpc1_pit_write(SGIHPC1State *s, int reg, uint8_t val)
{
    int ch, rw;

    if (reg == 3) {
        ch = (val >> 6) & 0x03;
        if (ch < 3) {
            s->pit_control[ch] = val;
            s->pit_rw_state[ch] = 0;
            s->pit_read_state[ch] = 0;
        }
        return;
    }

    if (reg >= 3) {
        return;
    }

    ch = reg;
    rw = (s->pit_control[ch] >> 4) & 0x3;
    switch (rw) {
    case 1: /* LSB only */
        s->pit_count[ch] = val;
        break;
    case 2: /* MSB only */
        s->pit_count[ch] = (uint16_t)val << 8;
        break;
    case 3: /* LSB then MSB */
        if (s->pit_rw_state[ch] == 0) {
            s->pit_low[ch] = val;
            s->pit_rw_state[ch] = 1;
            return;
        }
        s->pit_count[ch] = s->pit_low[ch] | ((uint16_t)val << 8);
        s->pit_rw_state[ch] = 0;
        break;
    default:
        return;
    }
    s->pit_programmed[ch] = true;
    s->pit_load_ns[ch] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint8_t hpc1_pit_read(SGIHPC1State *s, int reg)
{
    int ch, rw;
    uint16_t rem;

    if (reg == 3) {
        return s->pit_control[0];
    }
    if (reg >= 3) {
        return 0;
    }

    ch = reg;
    rw = (s->pit_control[ch] >> 4) & 0x3;
    rem = hpc1_pit_remaining(s, ch);

    switch (rw) {
    case 1: /* LSB only */
        return rem & 0xff;
    case 2: /* MSB only */
        return (rem >> 8) & 0xff;
    default: /* LSB then MSB */
        if (s->pit_read_state[ch] == 0) {
            s->pit_read_state[ch] = 1;
            return rem & 0xff;
        }
        s->pit_read_state[ch] = 0;
        return (rem >> 8) & 0xff;
    }
}

/* ------------------------------------------------------------------ */
/* EEPROM bit-bang via the aux register                                */
/* ------------------------------------------------------------------ */

static uint8_t hpc1_aux_read(SGIHPC1State *s)
{
    uint8_t val = s->aux & ~0x10;
    if (s->eeprom && (eeprom93xx_read(s->eeprom) & 1)) {
        val |= 0x10;
    }
    return val;
}

static void hpc1_aux_write(SGIHPC1State *s, uint8_t val)
{
    if (s->eeprom) {
        /* bit1 = CS, bit2 = CLK, bit3 = DI (MAME ip20.cpp) */
        eeprom93xx_write(s->eeprom, (val >> 1) & 1, (val >> 2) & 1,
                         (val >> 3) & 1);
    }
    s->aux = val;
}

/* ------------------------------------------------------------------ */
/* MMIO dispatch                                                       */
/* ------------------------------------------------------------------ */

static uint64_t sgi_hpc1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIHPC1State *s = opaque;
    uint8_t val8 = 0;

    if (addr >= HPC1_DUART_BASE && addr < HPC1_DUART_BASE + 0x40) {
        unsigned off = addr - HPC1_DUART_BASE;
        int d = off >> 4;
        int grp = (off >> 2) & 0x3;
        int c = (grp & 0x2) ? 0 : 1;   /* grp bit1: 1 = channel A */
        bool is_data = grp & 0x1;
        if (d >= HPC1_NUM_DUARTS) {
            return 0; /* headphone MDACs */
        }
        val8 = is_data ? scc_data_read(s, d, c) : scc_ctrl_read(s, d, c);
        return val8;
    }

    if (addr >= HPC1_RTC_BASE && addr < HPC1_RTC_BASE + 0x80) {
        return s->rtc[addr - HPC1_RTC_BASE];
    }

    if (addr >= HPC1_INT2_BASE && addr < HPC1_INT2_BASE + 0x40) {
        int idx = ((addr - HPC1_INT2_BASE) >> 2) & 0xf;
        switch (idx) {
        case INT2_LIO0_STATUS:
            return s->lio_status[0];
        case INT2_LIO0_MASK:
            return s->lio_mask[0];
        case INT2_LIO1_STATUS:
            return s->lio_status[1];
        case INT2_LIO1_MASK:
            return s->lio_mask[1];
        case INT2_VME_STATUS:
            return s->vme_status;
        case INT2_VME0_MASK:
            return s->vme_mask[0];
        case INT2_VME1_MASK:
            return s->vme_mask[1];
        case INT2_CONFIG:
            return s->int2_config;
        default:
            if (idx >= INT2_PIT_BASE && idx <= INT2_PIT_BASE + 3) {
                return hpc1_pit_read(s, idx - INT2_PIT_BASE);
            }
            return 0;
        }
    }

    if (addr >= HPC1_SCSI_BASE && addr < HPC1_SCSI_BASE + 8) {
        unsigned off = addr - HPC1_SCSI_BASE;
        if (!s->scsi) {
            return 0;
        }
        return (off & 4) ? wd33c93_data_read(s->scsi)
                         : wd33c93_addr_read(s->scsi);
    }

    if (addr >= HPC1_ENET_BASE && addr < HPC1_ENET_BASE + 0x20) {
        int reg = ((addr - HPC1_ENET_BASE) >> 2) & 7;
        switch (reg) {
        case 6:
            val8 = s->seeq_rx_status | 0x80;
            break;
        case 7:
            val8 = s->seeq_tx_status | 0x80;
            break;
        default:
            val8 = 0;
            break;
        }
        return val8;
    }

    switch (addr) {
    case HPC1_MISCSR:
        return s->miscsr;
    case HPC1_AUX:
        return hpc1_aux_read(s);
    default:
        break;
    }

    if (addr + size <= sizeof(s->core_scratch)) {
        uint32_t v = 0;
        unsigned i;
        for (i = 0; i < size; i++) {
            v = (v << 8) | s->core_scratch[addr + i];
        }
        return v;
    }

    qemu_log_mask(LOG_UNIMP, "sgi-hpc1: read addr 0x%" HWADDR_PRIx
                  " size %u\n", addr, size);
    return 0;
}

static void sgi_hpc1_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    SGIHPC1State *s = opaque;
    uint8_t val8 = value & 0xff;

    if (addr >= HPC1_DUART_BASE && addr < HPC1_DUART_BASE + 0x40) {
        unsigned off = addr - HPC1_DUART_BASE;
        int d = off >> 4;
        int grp = (off >> 2) & 0x3;
        int c = (grp & 0x2) ? 0 : 1;
        bool is_data = grp & 0x1;
        if (d >= HPC1_NUM_DUARTS) {
            return;
        }
        if (is_data) {
            scc_data_write(s, d, c, val8);
        } else {
            scc_ctrl_write(s, d, c, val8);
        }
        return;
    }

    if (addr >= HPC1_RTC_BASE && addr < HPC1_RTC_BASE + 0x80) {
        s->rtc[addr - HPC1_RTC_BASE] = val8;
        return;
    }

    if (addr >= HPC1_INT2_BASE && addr < HPC1_INT2_BASE + 0x40) {
        int idx = ((addr - HPC1_INT2_BASE) >> 2) & 0xf;
        switch (idx) {
        case INT2_LIO0_MASK:
            s->lio_mask[0] = val8;
            int2_update(s);
            break;
        case INT2_LIO1_MASK:
            s->lio_mask[1] = val8;
            int2_update(s);
            break;
        case INT2_VME0_MASK:
            s->vme_mask[0] = val8;
            break;
        case INT2_VME1_MASK:
            s->vme_mask[1] = val8;
            break;
        case INT2_CONFIG:
            s->int2_config = val8 & 0x1f;
            break;
        case INT2_TIMER_ACK:
            if (val8 & 0x1) {
                s->timer_pending[0] = false;
            }
            if (val8 & 0x2) {
                s->timer_pending[1] = false;
            }
            break;
        default:
            if (idx >= INT2_PIT_BASE && idx <= INT2_PIT_BASE + 3) {
                hpc1_pit_write(s, idx - INT2_PIT_BASE, val8);
            }
            break;
        }
        return;
    }

    if (addr >= HPC1_SCSI_BASE && addr < HPC1_SCSI_BASE + 8) {
        unsigned off = addr - HPC1_SCSI_BASE;
        if (!s->scsi) {
            return;
        }
        if (off & 4) {
            wd33c93_data_write(s->scsi, val8);
        } else {
            wd33c93_addr_write(s->scsi, val8);
        }
        return;
    }

    if (addr >= HPC1_ENET_BASE && addr < HPC1_ENET_BASE + 0x20) {
        int reg = ((addr - HPC1_ENET_BASE) >> 2) & 7;
        switch (reg) {
        case 6:
            s->seeq_rx_cmd = val8;
            break;
        case 7:
            s->seeq_tx_cmd = val8;
            break;
        default:
            if (reg < 6) {
                s->seeq_station_addr[reg] = val8;
            }
            break;
        }
        return;
    }

    switch (addr) {
    case HPC1_MISCSR:
        s->miscsr = value;
        break;
    case HPC1_AUX:
        hpc1_aux_write(s, val8);
        break;
    case HPC1_SCSI_CTRL:
        s->scsi_ctrl = value;
        break;
    case 0x188:
        s->dsp_bc = value;
        break;
    default:
        if (addr + size <= sizeof(s->core_scratch)) {
            unsigned i;
            for (i = 0; i < size; i++) {
                s->core_scratch[addr + i] =
                    (value >> (8 * (size - 1 - i))) & 0xff;
            }
        } else {
            qemu_log_mask(LOG_UNIMP, "sgi-hpc1: write addr 0x%" HWADDR_PRIx
                          " = 0x%" PRIx64 " size %u\n", addr, value, size);
        }
        break;
    }
}

static const MemoryRegionOps sgi_hpc1_ops = {
    .read = sgi_hpc1_read,
    .write = sgi_hpc1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------ */
/* Chardev receive path                                                */
/* ------------------------------------------------------------------ */

static int sgi_hpc1_serial_can_receive(void *opaque)
{
    SGIHPC1State *s = opaque;
    return HPC1_RX_FIFO_SIZE - s->uart[2][0].rx_count;
}

static void sgi_hpc1_serial_receive(void *opaque, const uint8_t *buf, int size)
{
    SGIHPC1State *s = opaque;
    int i, d, c;
    const int targets[][2] = { { 2, 0 }, { 0, 0 } };

    for (i = 0; i < size; i++) {
        int t;
        for (t = 0; t < 2; t++) {
            SGIHPC1Uart *u;
            d = targets[t][0];
            c = targets[t][1];
            u = &s->uart[d][c];
            if (u->rx_count >= HPC1_RX_FIFO_SIZE) {
                continue;
            }
            u->rx_fifo[u->rx_head] = buf[i];
            u->rx_head = (u->rx_head + 1) % HPC1_RX_FIFO_SIZE;
            u->rx_count++;
            if (u->wr[1] & 0x18) {
                u->rr3 |= (c == 0 ? SCC_RX_IP_A : SCC_RX_IP);
            }
        }
    }
    scc_update_irq(s);
}

static void sgi_hpc1_serial_event(void *opaque, QEMUChrEvent event)
{
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

static void sgi_hpc1_reset(DeviceState *dev)
{
    SGIHPC1State *s = SGI_HPC1(dev);
    int d, c, i;

    memset(s->core_scratch, 0, sizeof(s->core_scratch));
    memset(s->rtc, 0, sizeof(s->rtc));
    s->miscsr = 0;
    s->scsi_ctrl = 0;
    s->scsi_bc = 0;
    s->scsi_cbp = 0;
    s->scsi_nbdp = 0;
    s->dsp_bc = 0;
    s->seeq_rx_cmd = 0;
    s->seeq_tx_cmd = 0;
    s->seeq_rx_status = 0;
    s->seeq_tx_status = 0;
    s->aux = 0;
    for (i = 0; i < 6; i++) {
        s->seeq_station_addr[i] = 0;
    }
    s->lio_status[0] = s->lio_status[1] = 0;
    s->lio_mask[0] = s->lio_mask[1] = 0;
    s->vme_status = 0;
    s->vme_mask[0] = s->vme_mask[1] = 0;
    s->int2_config = 0;
    for (i = 0; i < 3; i++) {
        s->pit_control[i] = 0;
        s->pit_count[i] = 0;
        s->pit_low[i] = 0;
        s->pit_rw_state[i] = 0;
        s->pit_read_state[i] = 0;
        s->pit_load_ns[i] = 0;
        s->pit_programmed[i] = false;
    }
    s->timer_pending[0] = s->timer_pending[1] = false;

    for (d = 0; d < HPC1_NUM_DUARTS; d++) {
        for (c = 0; c < HPC1_DUART_CH; c++) {
            SGIHPC1Uart *u = &s->uart[d][c];
            memset(u, 0, sizeof(*u));
        }
    }
}

static void sgi_hpc1_realize(DeviceState *dev, Error **errp)
{
    SGIHPC1State *s = SGI_HPC1(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sgi_hpc1_ops, s,
                          "sgi-hpc1", HPC1_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    qdev_init_gpio_out_named(dev, s->cpu_irq, "cpu-irq", 2);
    qdev_init_gpio_out_named(dev, s->timer_irq, "timer-irq", 2);
    qdev_init_gpio_in_named(dev, hpc1_scsi_irq, "scsi-irq", 1);

    /* WD33C93 SCSI controller */
    s->scsi = WD33C93(qdev_new(TYPE_WD33C93));
    qdev_realize(DEVICE(s->scsi), NULL, &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(s->scsi), "irq", 0,
                                qdev_get_gpio_in_named(dev, "scsi-irq", 0));

    /* 93C56 NVRAM (128 x 16-bit words) */
    s->eeprom = eeprom93xx_new(dev, 128);

    /* PIT interrupt timers (timer0 -> IP4, timer1 -> IP5) */
    s->pit_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, hpc1_pit_timer_cb, s);
    s->pit_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, hpc1_pit_timer_cb, s);

    if (qemu_chr_fe_backend_connected(&s->serial)) {
        qemu_chr_fe_set_handlers(&s->serial,
                                 sgi_hpc1_serial_can_receive,
                                 sgi_hpc1_serial_receive,
                                 sgi_hpc1_serial_event,
                                 NULL, s, NULL, true);
    }
}

static const Property sgi_hpc1_properties[] = {
    DEFINE_PROP_CHR("chardev", SGIHPC1State, serial),
    DEFINE_PROP_STRING("nvram", SGIHPC1State, nvram_filename),
};

static void sgi_hpc1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sgi_hpc1_realize;
    device_class_set_props(dc, sgi_hpc1_properties);
    device_class_set_legacy_reset(dc, sgi_hpc1_reset);
}

static const TypeInfo sgi_hpc1_info = {
    .name = TYPE_SGI_HPC1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIHPC1State),
    .class_init = sgi_hpc1_class_init,
};

static void sgi_hpc1_register_types(void)
{
    type_register_static(&sgi_hpc1_info);
}

type_init(sgi_hpc1_register_types)
