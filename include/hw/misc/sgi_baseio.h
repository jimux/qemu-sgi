/*
 * SGI IP27 BaseIO board model.
 *
 * A standalone Origin 200 attaches a BaseIO (Bridge + IOC3 + SCSI) as an XIO
 * widget.  The IP27 PROM discovers it via the bridge widget-ID word (part
 * 0xc002), then reads the board's 1-wire configuration EEPROM: the bridge MCR
 * register at +0xb4 is the MicroLAN line controller, bit-banged by the PROM's
 * LIBKL NIC driver (nic.c: nic_presence/nic_read/nic_write via access_mcr32).
 *
 * We model the MCR line operations and two DS2502 (family 0x09) devices whose
 * memories carry the BaseIO manufacturing record (part "030-0734-...", name
 * "BASEIO") and the IOC3 Ethernet MAC address.  Protocol and record layout are
 * taken from the IRIX 6.5.7m ARCS source (libsk/ml/nic.c, libkl/ml/
 * bridge_nic.c) -- authoritative hardware behaviour.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_BASEIO_H
#define HW_MISC_SGI_BASEIO_H

#include "hw/char/serial.h"
#include "hw/core/sysbus.h"
#include "hw/scsi/sgi_qlisp.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_SGI_BASEIO "sgi-baseio"
OBJECT_DECLARE_SIMPLE_TYPE(SGIBaseIOState, SGI_BASEIO)

/* XIO widget window modelled by the BaseIO. */
#define SGI_BASEIO_WINDOW_SIZE 0x1000000ULL

/* Widget ID word (read at offset 4): part 0xc002 (Bridge). */
#define SGI_BASEIO_WIDGET_PART 0xc002
/*
 * SGI manufacturer number in WIDGET_ID (bits [11:1]).  The xwidget driver
 * match (XWIDGET_HARDWARE_ID_MATCH) requires the mfg number to match the
 * driver's registration, so the bridge driver pcibr (registered with
 * BRIDGE_WIDGET_MFGR_NUM = 0x036) only attaches when we report it; with mfg 0
 * the board never bound and no "<widget>/pci" vertex was created.
 */
#define SGI_BASEIO_WIDGET_MFGR 0x036

/* IOC3 on the BaseIO: register block at bridge+0x200000, UART A at +0x220178. */
#define SGI_BASEIO_IOC3_BASE 0x200000ULL
#define SGI_BASEIO_IOC3_SIO_CR 0x200028ULL
#define SGI_BASEIO_IOC3_MCR 0x200030ULL
#define SGI_BASEIO_IOC3_UART 0x220178ULL
#define SGI_BASEIO_IOC3_UART_SIZE 8

/*
 * BaseIO on-board QLogic ISP1020 SCSI channels (PCI slots 1 and 2).  Register
 * (DevIO) windows at mem_base 0x08400000 / 0x08600000 for the node's IO widget
 * (widget 8), i.e. offsets 0x400000 / 0x600000 within the 16 MB widget window
 * (see ip27_swin_phys + pci_mem_base in the ARCS bridge code).
 */
#define SGI_BASEIO_QLISP0_OFF 0x400000ULL
#define SGI_BASEIO_QLISP1_OFF 0x600000ULL

/*
 * IOC3 SSRAM (256 KB address space, may not be fully populated), accessed for
 * diagnostics at IOC3 offset 0x40000 (IOC3_RAM_OFF).  Words are 16 data bits +
 * a parity bit (bit 16); a read returns the parity-error bit (bit 17) set when
 * (parity(data) ^ parity_bit) is odd.  See IRIX sys/PCI/ioc3.h IOC3_SSRAM_DM/PM
 * and ARCS diag_enet.c enet_ssram.
 */
#define SGI_BASEIO_IOC3_SSRAM_OFF (SGI_BASEIO_IOC3_BASE + 0x40000ULL)
#define SGI_BASEIO_IOC3_SSRAM_LEN 0x40000ULL
#define SGI_BASEIO_IOC3_SSRAM_WORDS (SGI_BASEIO_IOC3_SSRAM_LEN / 4)

/* DS2502 (family 0x09) example ROM id and 1 Kbit memory. */
#define SGI_BASEIO_DS_ROM_SIZE 8
#define SGI_BASEIO_DS_MEM_SIZE 128

/*
 * IOC3 Ethernet MAC register block, IOC3 offset 0x0F0 (bridge+0x2000F0).
 * 25 words: EMCR..MIDR_W. See IRIX sys/PCI/ioc3.h ioc3_eregs_t and the ARCS
 * standalone ef driver references/stand/arcs/lib/libsk/net/if_ef.c.
 */
#define SGI_BASEIO_ETH_OFF 0x2000F0ULL
#define SGI_BASEIO_ETH_NREGS 25
#define SGI_BASEIO_ETH_SIZE (SGI_BASEIO_ETH_NREGS * 4)

/*
 * Bridge external SSRAM window at DEVIO0 (0x200000) + 0x80000 = bridge+0x280000,
 * up to 512 KB (BRIDGE_EXT_SSRAM in sys/PCI/bridge.h).  ARCS sizes it and runs a
 * pattern test over it; backed with storage (see SGIBaseIOState.br_ssram).
 */
#define SGI_BASEIO_BR_SSRAM_OFF 0x280000ULL
#define SGI_BASEIO_BR_SSRAM_SIZE 0x80000

/* Register word indices (from IOC3 offset 0x0F0). */
#define SGI_IOC3_EMCR 0
#define SGI_IOC3_EISR 1
#define SGI_IOC3_EIER 2
#define SGI_IOC3_ERCSR 3
#define SGI_IOC3_ERBR_H 4
#define SGI_IOC3_ERBR_L 5
#define SGI_IOC3_ERBAR 6
#define SGI_IOC3_ERCIR 7
#define SGI_IOC3_ERPIR 8
#define SGI_IOC3_ERTR 9
#define SGI_IOC3_ETCSR 10
#define SGI_IOC3_ERSR 11
#define SGI_IOC3_ETCDC 12
#define SGI_IOC3_EBIR 13
#define SGI_IOC3_ETBR_H 14
#define SGI_IOC3_ETBR_L 15
#define SGI_IOC3_ETCIR 16
#define SGI_IOC3_ETPIR 17
#define SGI_IOC3_EMAR_H 18
#define SGI_IOC3_EMAR_L 19
#define SGI_IOC3_EHAR_H 20
#define SGI_IOC3_EHAR_L 21
#define SGI_IOC3_MICR 22
#define SGI_IOC3_MIDR_R 23
#define SGI_IOC3_MIDR_W 24

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

/* One DS2502 1-wire device. */
typedef struct SGIBaseIODS {
  uint8_t rom[SGI_BASEIO_DS_ROM_SIZE];
  uint8_t mem[SGI_BASEIO_DS_MEM_SIZE];
  SGIDSState state;
  uint8_t cmd;          /* command shift register */
  int cmd_bits;
  uint8_t in;           /* generic bit shift register */
  int in_bits;
  int out_index;        /* index for READROM/SEARCH/RMEM_DATA */
  int search_phase;     /* search: 0=read bit, 1=read complement, 2=choice */
  int addr;
  int extra;            /* DS1982 extra status byte pending */
  int extra_bits;       /* status bits returned before read-memory data */
  int data_bit;         /* latched line state for the next read */
} SGIBaseIODS;

struct SGIBaseIOState {
  SysBusDevice parent_obj;

  MemoryRegion iomem;

  /* IOC3 SuperIO 16550-compatible UART A (serial console). */
  SerialState ioc3_uart;
  MemoryRegion ioc3_uart_mr;

  /* On-board QLogic ISP1020 SCSI channels (PCI slots 1,2); widget-8 only. */
  SGIQLispState isp[2];

  uint32_t nasid;
  uint32_t widget;

  /*
   * XIO widget id reported in WIDGET_CONTROL (bridge+0x24) -- the id the
   * board is strapped with.  UINT32_MAX means "use @widget".  The IP27 machine
   * maps the base IO board at the real IO widget and adds a discovery alias at
   * SWIN widget 0; the alias must report the REAL board's id so the kernel's
   * iograph (ml/SN/iograph.c:849, reads BRIDGE_WID_CONTROL at SWIN 0) derives
   * basew_id = the widget the board actually lives at.
   */
  uint32_t wid_id;

  /* IOC3 SuperIO interrupt-enable register (IES 0x20 sets, IEC 0x24 clears,
   * both read back this mask).  SIO_IR (0x1c) reports TX-empty (we drain
   * synchronously).  See io/sio_ioc3.c and sys/PCI/ioc3.h. */
  uint32_t sio_ienb;

  /* Bridge free-running counter/timer (offset 0x100): bit0 = enable. */
  uint32_t timer_en;

  /* DS2502 devices: board manufacturing record (0xb4) and MAC (0x200030). */
  SGIBaseIODS ds_board;
  SGIBaseIODS ds_mac;

  /* IOC3 Ethernet MAC register file, RX/TX DMA ring cursors. */
  uint32_t eth_regs[SGI_BASEIO_ETH_NREGS];
  uint32_t eth_rxprod; /* hardware RX produce, byte offset into the ring */
  uint32_t eth_txcons; /* hardware TX consume, byte offset into the ring */
  uint16_t phy_regs[32];
  uint32_t phy_write_data;
  uint32_t phy_read_data;

  /* IOC3 GenericPIO block (GPCR set 0x34 / clear 0x38, GPDR data 0x3c). */
  uint32_t ioc3_gpcr;
  uint32_t ioc3_gpdr;

  /* PCI config-space writes to slot 0 (IOC3), via the bridge config window. */
  uint32_t pci_cfg0[0x40];

  /*
   * Bridge external SSRAM (BRIDGE_EXT_SSRAM) backing storage, at DEVIO0+0x80000
   * = bridge+0x280000.  On IP27 (non-RPROM) ARCS size_bridge_ssram() writes a
   * size marker and reads it back to pick the fitted size, then exercises the
   * array with alternating a5a5/5a5a patterns; without storage those reads
   * return 0, the bridge is mistreated as having no SSRAM, and the kernel loops
   * forever.  Same contract octane models for IP30 (hw/misc/sgi_bridge.c).
   */
  uint8_t br_ssram[SGI_BASEIO_BR_SSRAM_SIZE];

  /*
   * IOC3 byte-bus time-of-day chip (Dallas DS1386) at bridge+0x280000
   * (IOC3_BYTEBUS_DEV0).  IRIX ml/SN/klclock.c rtodc()/wtodc() read and write
   * it, and ml/clksupport.c warns if the calendar does not advance, so we
   * present a ticking BCD calendar.  Only the Dallas register map is modelled;
   * the autodetect in rtodc() writes 0xff to the day register and treats a
   * non-0xff read-back as Dallas, which our live calendar always yields.
   */
  int64_t tod_epoch_ns;  /* QEMU_CLOCK_VIRTUAL ns when the epoch was last set */
  int64_t tod_epoch_sec; /* Unix seconds corresponding to tod_epoch_ns */
  uint8_t tod_control;   /* RTC_DAL_CONTROL (0xb): update enable/disable */
  uint8_t tod_user;      /* RTC_DAL_USER (0xe) */

  /* IOC3 SSRAM diagnostic region (16-bit data + parity bit). */
  MemoryRegion ssram_mr;
  uint32_t ssram[SGI_BASEIO_IOC3_SSRAM_WORDS];

  NICConf nic_conf;
  NICState *nic;
};

#endif /* HW_MISC_SGI_BASEIO_H */
