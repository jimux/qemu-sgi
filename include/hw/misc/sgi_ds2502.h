/*
 * Dallas/Maxim DS250x 1-wire EEPROM model, shared by SGI machines.
 *
 * Factored out of origin's sgi_hub.c (commit af842ce7a2) so IP30 (Octane) can
 * use the same part, and extended with a multi-device MicroLAN bus whose SEARCH
 * ROM responder implements the standard 1-wire bit/complement/direction
 * handshake across several devices (see nic_next_scan() in
 * references/stand/arcs/lib/libsk/ml/nic.c).  A single-device board EEPROM uses
 * the SGIDS2502 API directly; a bus that must enumerate several FRU ID chips
 * (e.g. the IP30 power-on NIC diagnostics) uses SGIDS2502BUS.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_DS2502_H
#define HW_MISC_SGI_DS2502_H

#include <stdbool.h>
#include <stdint.h>

/* --- One DS250x part -------------------------------------------------- */

typedef struct SGIDS2502 {
    uint8_t rom[8];      /* family + 48-bit serial + Dallas CRC-8 */
    uint8_t mem[64];     /* two 32-byte pages, each with a trailing CRC-16 */
    int state;
    uint8_t cmd;
    int cmd_bits;
    uint8_t in;
    int in_bits;
    int out_index;
    int search_phase;
    int addr;
    int extra;
    int data_bit;        /* latched line state (MCR_DATA) */
} SGIDS2502;

/* Program a board record (page 0 serial/part, page 1 rev/name) and ROM id. */
void sgi_ds2502_build_board(SGIDS2502 *ds, const char *serial,
                            const char *part, const char *name,
                            const uint8_t rom_serial[6]);

/* Reset the part to the command state (a 1-wire reset pulse). */
void sgi_ds2502_reset(SGIDS2502 *ds);
/* Clock one master bit into the part. */
void sgi_ds2502_write_bit(SGIDS2502 *ds, int bit);
/* Clock one bit out of the part. */
int sgi_ds2502_read_bit(SGIDS2502 *ds);
/*
 * Decode one MCR_PACK(pulse, sample) access and return the latched line data
 * bit.  pulse >= 480 is reset/presence; sample 30/110 is write 0/1; sample 13
 * is a read.  The caller supplies MCR_DONE and the MCR_DATA bit.
 */
int sgi_ds2502_mcr(SGIDS2502 *ds, uint64_t val);

/* --- A MicroLAN with several parts ------------------------------------ */

#define SGIDS2502_MAX 8

typedef struct SGIDS2502BUS {
    SGIDS2502 dev[SGIDS2502_MAX];
    int ndev;
    uint8_t data_bit;        /* latched line state (MCR_DATA) */

    /* Command decode (bus level; the parts are pure data holders). */
    int state;
    uint8_t cmd;
    int cmd_bits;

    /* MATCH ROM / SKIP ROM selection */
    uint8_t in[8];
    int in_bits;
    int sel;                 /* selected part index, -1 = none */

    /* ROM search (multi-drop) */
    uint8_t s_path[8];
    int s_bit;
    int s_phase;             /* 0 = bit, 1 = complement */
    int s_last_disc;
    int s_disc_marker;
    bool s_done;
    bool s_active[SGIDS2502_MAX];

    /* Memory read state (uses dev[sel], or dev[0] for SKIP ROM) */
    int addr;
    int out_index;
    int extra;
} SGIDS2502BUS;

void sgi_ds2502_bus_init(SGIDS2502BUS *bus, const char *name);
/* Add a part; program it with build_board(). Returns the index, or -1. */
int sgi_ds2502_bus_add(SGIDS2502BUS *bus, const char *serial, const char *part,
                       const char *name, const uint8_t rom_serial[6]);
void sgi_ds2502_bus_reset(SGIDS2502BUS *bus);
void sgi_ds2502_bus_write_bit(SGIDS2502BUS *bus, int bit);
int sgi_ds2502_bus_read_bit(SGIDS2502BUS *bus);
int sgi_ds2502_bus_mcr(SGIDS2502BUS *bus, uint64_t val);

#endif /* HW_MISC_SGI_DS2502_H */
