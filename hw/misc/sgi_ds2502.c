/*
 * Dallas/Maxim DS250x 1-wire EEPROM model, shared by SGI machines.
 * See include/hw/misc/sgi_ds2502.h for provenance and the bus protocol notes.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_ds2502.h"

/* --- One DS250x part (from origin's sgi_hub.c af842ce7a2) -------------- */

enum {
    SGI_DS_IDLE = 0,
    SGI_DS_CMD,        /* shifting in an 8-bit ROM command */
    SGI_DS_READROM,    /* shifting out the 64-bit ROM id */
    SGI_DS_MATCHROM,   /* shifting in a 64-bit match-ROM sequence */
    SGI_DS_SEARCH,     /* ROM search: read bit, complement, write choice */
    SGI_DS_F0_PENDING, /* 0xf0 seen: next op decides search vs read-memory */
    SGI_DS_RMEM_ADDR,  /* shifting in the 16-bit memory address */
    SGI_DS_RMEM_DATA,  /* shifting out memory bytes */
};

/* Dallas/Maxim 1-wire CRC-8 (poly 0x8C, reflected). */
static uint8_t sgi_ds_crc8(const uint8_t *p, int n)
{
    uint8_t crc = 0;
    int i, j;

    for (i = 0; i < n; i++) {
        uint8_t c = p[i];
        for (j = 0; j < 8; j++) {
            if ((crc ^ c) & 1) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
            c >>= 1;
        }
    }
    return crc;
}

/* NIC 16-bit CRC (poly 0xC001), from libsk/ml/nic.c. */
static const int sgi_ds_oddparity[16] = {0, 1, 1, 0, 1, 0, 0, 1,
                                         1, 0, 0, 1, 0, 1, 1, 0};

static uint16_t sgi_ds_crc16_step(uint16_t crc, uint8_t in)
{
    uint16_t data = in;

    data = (data ^ (crc & 0xff)) & 0xff;
    crc >>= 8;
    if (sgi_ds_oddparity[data & 0xf] ^ sgi_ds_oddparity[data >> 4]) {
        crc ^= 0xc001;
    }
    data <<= 6;
    crc ^= data;
    data <<= 1;
    crc ^= data;
    return crc;
}

static uint16_t sgi_ds_crc16(const uint8_t *p, int n)
{
    uint16_t crc = 0;
    int i;

    for (i = 0; i < n; i++) {
        crc = sgi_ds_crc16_step(crc, p[i]);
    }
    return crc;
}

static void sgi_ds_put(uint8_t *dst, const char *s, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        dst[i] = s[i] ? (uint8_t)s[i] : ' ';
    }
}

void sgi_ds2502_build_board(SGIDS2502 *ds, const char *serial,
                            const char *part, const char *name,
                            const uint8_t rom_serial[6])
{
    int a, b;

    memset(ds->mem, 0xff, sizeof(ds->mem));

    ds->mem[0] = 0x01;
    sgi_ds_put(&ds->mem[1], serial, 10);
    sgi_ds_put(&ds->mem[11], part, 19);

    sgi_ds_put(&ds->mem[32 + 0], "", 6);
    sgi_ds_put(&ds->mem[32 + 6], "0001", 4);
    ds->mem[32 + 10] = 0x00;
    memset(&ds->mem[32 + 11], 0x00, 4);
    ds->mem[32 + 15] = 0x00;
    sgi_ds_put(&ds->mem[32 + 16], name, 14);

    for (int page = 0; page < 2; page++) {
        uint8_t *pg = &ds->mem[page * 32];

        pg[30] = 0;
        pg[31] = 0;
        for (a = 0; a < 256; a++) {
            for (b = 0; b < 256; b++) {
                pg[30] = a;
                pg[31] = b;
                if (sgi_ds_crc16(pg, 32) == 0xb001) {
                    goto done;
                }
            }
        }
    done:;
    }

    ds->rom[0] = 0x09;
    memcpy(&ds->rom[1], rom_serial, 6);
    ds->rom[7] = sgi_ds_crc8(ds->rom, 7);
    ds->extra_bits = 8; /* DS2502 clocks a status byte before the data */
}

void sgi_ds2502_build_mac(SGIDS2502 *ds, const uint8_t mac[6],
                          const uint8_t rom_serial[6])
{
    int a, b;

    memset(ds->mem, 0xff, sizeof(ds->mem));
    ds->mem[0] = 0x8d;
    ds->mem[1] = 0x0a;
    ds->mem[2] = ds->mem[3] = ds->mem[4] = ds->mem[5] = 0x00;
    /* eaddr[i] = byte[11-i], so store MSB first at [6]. */
    for (int i = 0; i < 6; i++) {
        ds->mem[6 + i] = mac[5 - i];
    }
    ds->mem[12] = 0;
    ds->mem[13] = 0;
    for (a = 0; a < 256; a++) {
        for (b = 0; b < 256; b++) {
            ds->mem[12] = a;
            ds->mem[13] = b;
            if (sgi_ds_crc16(&ds->mem[1], 13) == 0xb001) {
                goto done;
            }
        }
    }
done:;

    ds->rom[0] = 0x09;
    memcpy(&ds->rom[1], rom_serial, 6);
    ds->rom[7] = sgi_ds_crc8(ds->rom, 7);
    ds->extra_bits = 0; /* nic_eaddr reads the record from byte 0 */
}

void sgi_ds2502_reset(SGIDS2502 *ds)
{
    ds->state = SGI_DS_CMD;
    ds->cmd = 0;
    ds->cmd_bits = 0;
    ds->in = 0;
    ds->in_bits = 0;
    ds->out_index = 0;
    ds->search_phase = 0;
    ds->addr = 0;
    ds->extra = 0;
    ds->data_bit = 0;
}

static void sgi_ds2502_decode(SGIDS2502 *ds)
{
    switch (ds->cmd) {
    case 0x33:
        ds->state = SGI_DS_READROM;
        ds->out_index = 0;
        break;
    case 0x55:
        ds->state = SGI_DS_MATCHROM;
        ds->in = 0;
        ds->in_bits = 0;
        break;
    case 0xcc:
        ds->state = SGI_DS_CMD;
        break;
    case 0xf0:
        ds->state = SGI_DS_F0_PENDING;
        break;
    default:
        ds->state = SGI_DS_CMD;
        break;
    }
    ds->cmd = 0;
    ds->cmd_bits = 0;
}

void sgi_ds2502_write_bit(SGIDS2502 *ds, int bit)
{
    switch (ds->state) {
    case SGI_DS_CMD:
        ds->cmd |= (bit & 1) << ds->cmd_bits;
        if (++ds->cmd_bits == 8) {
            sgi_ds2502_decode(ds);
        }
        break;
    case SGI_DS_MATCHROM:
        ds->in |= (bit & 1) << ds->in_bits;
        if (++ds->in_bits == 64) {
            ds->state = SGI_DS_CMD;
        }
        break;
    case SGI_DS_SEARCH:
        if (ds->search_phase == 2) {
            ds->search_phase = 0;
            if (++ds->out_index == 64) {
                ds->state = SGI_DS_CMD;
            }
        }
        break;
    case SGI_DS_F0_PENDING:
        ds->state = SGI_DS_RMEM_ADDR;
        ds->addr = 0;
        ds->in_bits = 0;
        /* fall through */
    case SGI_DS_RMEM_ADDR:
        ds->addr |= (bit & 1) << ds->in_bits;
        if (++ds->in_bits == 16) {
            ds->state = SGI_DS_RMEM_DATA;
            ds->out_index = 0;
            ds->extra = ds->extra_bits;
        }
        break;
    default:
        break;
    }
}

int sgi_ds2502_read_bit(SGIDS2502 *ds)
{
    int bit;

    switch (ds->state) {
    case SGI_DS_READROM:
        bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
        if (++ds->out_index == 64) {
            ds->state = SGI_DS_CMD;
        }
        return bit;
    case SGI_DS_SEARCH:
        bit = (ds->rom[ds->out_index / 8] >> (ds->out_index % 8)) & 1;
        if (ds->search_phase == 0) {
            ds->search_phase = 1;
            return bit;
        }
        ds->search_phase = 2;
        return bit ^ 1;
    case SGI_DS_F0_PENDING:
        ds->state = SGI_DS_SEARCH;
        ds->out_index = 0;
        ds->search_phase = 0;
        return sgi_ds2502_read_bit(ds);
    case SGI_DS_RMEM_DATA:
        if (ds->extra > 0) {
            bit = (0xff >> (8 - ds->extra)) & 1;
            ds->extra--;
            return bit;
        }
        bit = (ds->mem[ds->addr + ds->out_index / 8] >>
               (ds->out_index % 8)) & 1;
        if (++ds->out_index == 32 * 8) {
            ds->state = SGI_DS_CMD;
        }
        return bit;
    default:
        return 1;
    }
}

int sgi_ds2502_mcr(SGIDS2502 *ds, uint64_t val)
{
    unsigned pulse = (val >> 10) & 0x3ff;
    unsigned sample = (val >> 2) & 0xff;

    if (pulse >= 480) {
        sgi_ds2502_reset(ds);
    } else if (sample == 30) {
        sgi_ds2502_write_bit(ds, 0);
        ds->data_bit = 0;
    } else if (sample == 110) {
        sgi_ds2502_write_bit(ds, 1);
        ds->data_bit = 0;
    } else if (sample == 13) {
        ds->data_bit = sgi_ds2502_read_bit(ds);
    } else {
        ds->data_bit = 0;
    }
    return ds->data_bit;
}

/* --- Multi-device MicroLAN bus ---------------------------------------- */

static int sgi_ds_bus_dev_bit(const SGIDS2502 *dev, int i)
{
    return (dev->rom[i / 8] >> (i % 8)) & 1;
}

void sgi_ds2502_bus_init(SGIDS2502BUS *bus, const char *name)
{
    (void)name;
    memset(bus, 0, sizeof(*bus));
    bus->sel = -1;
}

int sgi_ds2502_bus_add(SGIDS2502BUS *bus, const char *serial, const char *part,
                       const char *name, const uint8_t rom_serial[6])
{
    int idx;

    if (bus->ndev >= SGIDS2502_MAX) {
        return -1;
    }
    idx = bus->ndev++;
    sgi_ds2502_build_board(&bus->dev[idx], serial, part, name, rom_serial);
    return idx;
}

void sgi_ds2502_bus_reset(SGIDS2502BUS *bus)
{
    int i;

    for (i = 0; i < bus->ndev; i++) {
        sgi_ds2502_reset(&bus->dev[i]);
    }
    bus->state = SGI_DS_CMD;
    bus->cmd = 0;
    bus->cmd_bits = 0;
    bus->in_bits = 0;
    bus->sel = -1;
    bus->s_bit = 0;
    bus->s_phase = 0;
    bus->s_last_disc = 0;
    bus->s_disc_marker = 0;
    bus->s_done = false;
    bus->addr = 0;
    bus->out_index = 0;
    bus->extra = 0;
    bus->data_bit = 0;
}

static void sgi_ds_bus_decode(SGIDS2502BUS *bus)
{
    switch (bus->cmd) {
    case 0x33: /* READ ROM (single-drop) */
        bus->state = SGI_DS_READROM;
        bus->out_index = 0;
        bus->sel = bus->ndev > 0 ? 0 : -1;
        break;
    case 0x55: /* MATCH ROM */
        bus->state = SGI_DS_MATCHROM;
        bus->in_bits = 0;
        memset(bus->in, 0, sizeof(bus->in));
        bus->sel = -1;
        break;
    case 0xcc: /* SKIP ROM: broadcast; memory read uses device 0 */
        bus->state = SGI_DS_CMD;
        bus->sel = bus->ndev > 0 ? 0 : -1;
        break;
    case 0xf0: /* SEARCH ROM (read) or READ MEMORY (write) */
        bus->state = SGI_DS_F0_PENDING;
        break;
    default:
        bus->state = SGI_DS_CMD;
        break;
    }
    bus->cmd = 0;
    bus->cmd_bits = 0;
}

void sgi_ds2502_bus_write_bit(SGIDS2502BUS *bus, int bit)
{
    switch (bus->state) {
    case SGI_DS_CMD:
        bus->cmd |= (bit & 1) << bus->cmd_bits;
        if (++bus->cmd_bits == 8) {
            sgi_ds_bus_decode(bus);
        }
        break;
    case SGI_DS_MATCHROM:
        bus->in[bus->in_bits / 8] |= (bit & 1) << (bus->in_bits % 8);
        if (++bus->in_bits == 64) {
            int i;

            bus->sel = -1;
            for (i = 0; i < bus->ndev; i++) {
                if (memcmp(bus->dev[i].rom, bus->in, 8) == 0) {
                    bus->sel = i;
                    break;
                }
            }
            bus->state = SGI_DS_CMD;
        }
        break;
    case SGI_DS_SEARCH:
        /* Master's chosen direction bit for the current position. */
        if (bus->s_bit < 64) {
            int i;

            bus->s_path[bus->s_bit / 8] =
                (bus->s_path[bus->s_bit / 8] & ~(1 << (bus->s_bit % 8))) |
                ((bit & 1) << (bus->s_bit % 8));
            for (i = 0; i < bus->ndev; i++) {
                if (bus->s_active[i] &&
                    sgi_ds_bus_dev_bit(&bus->dev[i], bus->s_bit) != (bit & 1)) {
                    bus->s_active[i] = false;
                }
            }
            bus->s_bit++;
            if (bus->s_bit == 64) {
                bus->state = SGI_DS_CMD;
            }
        }
        break;
    case SGI_DS_F0_PENDING:
        bus->state = SGI_DS_RMEM_ADDR;
        bus->addr = 0;
        bus->in_bits = 0;
        /* fall through */
    case SGI_DS_RMEM_ADDR:
        bus->addr |= (bit & 1) << bus->in_bits;
        if (++bus->in_bits == 16) {
            bus->state = SGI_DS_RMEM_DATA;
            bus->out_index = 0;
            bus->extra = (bus->sel >= 0) ? bus->dev[bus->sel].extra_bits : 0;
        }
        break;
    default:
        break;
    }
}

int sgi_ds2502_bus_read_bit(SGIDS2502BUS *bus)
{
    int bit;

    switch (bus->state) {
    case SGI_DS_READROM:
        bit = (bus->dev[bus->sel].rom[bus->out_index / 8] >>
               (bus->out_index % 8)) & 1;
        if (++bus->out_index == 64) {
            bus->state = SGI_DS_CMD;
        }
        return bit;
    case SGI_DS_SEARCH:
        {
            int i;
            int a = 1, b = 1;

            /*
             * Wired-AND over the parts still on the search path: a is the
             * common data bit, b the common complement.  a==0 && b==0 means
             * a discrepancy (the master picks a direction and drops the
             * mismatching parts); a==1 && b==1 means no part answered (end of
             * enumeration).  Two reads per bit: first the bit, then its
             * complement.
             */
            for (i = 0; i < bus->ndev; i++) {
                if (!bus->s_active[i]) {
                    continue;
                }
                a &= sgi_ds_bus_dev_bit(&bus->dev[i], bus->s_bit);
                b &= !sgi_ds_bus_dev_bit(&bus->dev[i], bus->s_bit);
            }
            if (bus->s_phase == 0) {
                bus->s_phase = 1;
                return a;
            }
            bus->s_phase = 0;
            return b;
        }
    case SGI_DS_F0_PENDING:
        /* A read (not a write) after 0xf0 starts a ROM search. */
        bus->state = SGI_DS_SEARCH;
        bus->s_bit = 0;
        bus->s_phase = 0;
        bus->s_last_disc = 0;
        bus->s_disc_marker = 0;
        bus->in_bits = 0;
        memset(bus->s_path, 0, sizeof(bus->s_path));
        for (int i = 0; i < bus->ndev; i++) {
            bus->s_active[i] = true;
        }
        return sgi_ds2502_bus_read_bit(bus);
    case SGI_DS_RMEM_DATA:
        if (bus->sel < 0) {
            return 1;
        }
        if (bus->extra > 0) {
            bit = (0xff >> (8 - bus->extra)) & 1;
            bus->extra--;
            return bit;
        }
        bit = (bus->dev[bus->sel].mem[bus->addr + bus->out_index / 8] >>
               (bus->out_index % 8)) & 1;
        if (++bus->out_index == 32 * 8) {
            bus->state = SGI_DS_CMD;
        }
        return bit;
    default:
        return 1;
    }
}

int sgi_ds2502_bus_mcr(SGIDS2502BUS *bus, uint64_t val)
{
    unsigned pulse = (val >> 10) & 0x3ff;
    unsigned sample = (val >> 2) & 0xff;

    if (pulse >= 480) {
        sgi_ds2502_bus_reset(bus);
    } else if (sample == 30) {
        sgi_ds2502_bus_write_bit(bus, 0);
        bus->data_bit = 0;
    } else if (sample == 110) {
        sgi_ds2502_bus_write_bit(bus, 1);
        bus->data_bit = 0;
    } else if (sample == 13) {
        bus->data_bit = sgi_ds2502_bus_read_bit(bus);
    } else {
        bus->data_bit = 0;
    }
    return bus->data_bit;
}
