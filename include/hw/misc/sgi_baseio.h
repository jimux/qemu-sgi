/*
 * SGI IP27 BaseIO board model.
 *
 * A standalone Origin 200 attaches a BaseIO (Bridge + IOC3 + SCSI) as an XIO
 * widget.  The IP27 PROM discovers it via the bridge widget-ID word (part
 * 0xc002), then reads the board's 1-wire configuration EEPROM: the bridge MCR
 * register at +0xb4 is the MicroLAN line controller, bit-banged by the PROM's
 * LIBKL NIC driver (nic.c: nic_presence/nic_read/nic_write via access_mcr32).
 *
 * We model the MCR line operations and a DS2502 (family 0x09) whose memory
 * carries the BaseIO manufacturing record (part "030-0734-...", name "BASEIO")
 * that the PROM's board-config parser expects.  Protocol and record layout are
 * taken from the IRIX 6.5.7m ARCS source (libsk/ml/nic.c, libkl/ml/
 * bridge_nic.c) -- authoritative hardware behaviour.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_BASEIO_H
#define HW_MISC_SGI_BASEIO_H

#include "hw/char/serial.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_SGI_BASEIO "sgi-baseio"
OBJECT_DECLARE_SIMPLE_TYPE(SGIBaseIOState, SGI_BASEIO)

/* XIO widget window modelled by the BaseIO. */
#define SGI_BASEIO_WINDOW_SIZE 0x1000000ULL

/* Widget ID word (read at offset 4): part 0xc002 (Bridge). */
#define SGI_BASEIO_WIDGET_PART 0xc002

/* IOC3 on the BaseIO: register block at bridge+0x200000, UART A at +0x220178. */
#define SGI_BASEIO_IOC3_BASE 0x200000ULL
#define SGI_BASEIO_IOC3_SIO_CR 0x200028ULL
#define SGI_BASEIO_IOC3_MCR 0x200030ULL
#define SGI_BASEIO_IOC3_UART 0x220178ULL
#define SGI_BASEIO_IOC3_UART_SIZE 8

/* DS2502 (family 0x09) example ROM id and 1 Kbit memory. */
#define SGI_BASEIO_DS_ROM_SIZE 8
#define SGI_BASEIO_DS_MEM_SIZE 128

/* NIC MicroLAN line state machine. */
typedef enum {
  SGI_DS_IDLE = 0,
  SGI_DS_CMD,         /* shifting in an 8-bit ROM command */
  SGI_DS_READROM,     /* shifting out the 64-bit ROM id */
  SGI_DS_MATCHROM,    /* shifting in a 64-bit match-ROM sequence */
  SGI_DS_SEARCH,      /* ROM search: read bit, complement, write choice */
  SGI_DS_F0_PENDING,  /* 0xf0 seen: next op decides search vs read-memory */
  SGI_DS_RMEM_ADDR,   /* shifting in the 16-bit memory address */
  SGI_DS_RMEM_DATA,   /* shifting out memory bytes */
} SGIDSState;

struct SGIBaseIOState {
  SysBusDevice parent_obj;

  MemoryRegion iomem;

  /* IOC3 SuperIO 16550-compatible UART A (serial console). */
  SerialState ioc3_uart;
  MemoryRegion ioc3_uart_mr;

  uint32_t nasid;
  uint32_t widget;

  /* DS2502 1-wire device. */
  uint8_t ds_rom[SGI_BASEIO_DS_ROM_SIZE];
  uint8_t ds_mem[SGI_BASEIO_DS_MEM_SIZE];
  SGIDSState ds_state;
  uint8_t ds_cmd;        /* command shift register */
  int ds_cmd_bits;
  uint8_t ds_in;         /* generic bit shift register */
  int ds_in_bits;
  int ds_out_index;      /* index for READROM/SEARCH/RMEM_DATA */
  int ds_search_phase;   /* search: 0=read bit, 1=read complement, 2=choice */
  int ds_addr;
  int ds_extra;          /* DS1982 extra status byte pending */
  int ds_data_bit;       /* latched line state for the next read */
};

#endif /* HW_MISC_SGI_BASEIO_H */
