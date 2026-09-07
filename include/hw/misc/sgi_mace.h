/*
 * SGI MACE (Multimedia, Audio, and Communications Engine) emulation
 *
 * MACE is the peripheral controller in the SGI O2 (IP32). It contains:
 * - PCI bridge (for SCSI, additional cards)
 * - Video input/output interfaces
 * - Ethernet MAC (MAC110)
 * - Audio controller
 * - ISA peripherals (flash, LED, NIC, interrupts)
 * - Keyboard/mouse interface
 * - I2C bus
 * - UST/MSC timers
 * - Serial ports (16550-compatible, 256-byte register spacing)
 * - DS17287 RTC/NVRAM
 *
 * Physical base: 0x1F000000
 * Reference: IRIX sys/mace.h, MAME src/mame/sgi/mace.cpp
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_MACE_H
#define HW_MISC_SGI_MACE_H

#include "chardev/char-fe.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_SGI_MACE "sgi-mace"
OBJECT_DECLARE_SIMPLE_TYPE(SGIMACEState, SGI_MACE)

/* Physical base of the whole MACE register file */
#define MACE_BASE               0x1F000000ULL

/*
 * MACE sub-block offsets from MACE_BASE (0x1F000000)
 */
#define MACE_PCI_OFFSET         0x080000
#define MACE_VIN1_OFFSET        0x100000
#define MACE_VIN2_OFFSET        0x180000
#define MACE_VOUT_OFFSET        0x200000
#define MACE_ENET_OFFSET        0x280000
#define MACE_PERIF_OFFSET       0x300000
#define MACE_ISA_EXT_OFFSET     0x380000

/* Peripheral sub-blocks (relative to MACE_BASE) */
#define MACE_AUDIO_OFFSET       (MACE_PERIF_OFFSET + 0x00000)
#define MACE_ISA_OFFSET         (MACE_PERIF_OFFSET + 0x10000)
#define MACE_KBDMS_OFFSET       (MACE_PERIF_OFFSET + 0x20000)
#define MACE_I2C_OFFSET         (MACE_PERIF_OFFSET + 0x30000)
#define MACE_UST_MSC_OFFSET     (MACE_PERIF_OFFSET + 0x40000)

/* ISA extended sub-blocks (relative to MACE_BASE) */
#define MACE_EPP_OFFSET         (MACE_ISA_EXT_OFFSET + 0x00000)
#define MACE_ECP_OFFSET         (MACE_ISA_EXT_OFFSET + 0x08000)
#define MACE_SER1_OFFSET        (MACE_ISA_EXT_OFFSET + 0x10000)
#define MACE_SER2_OFFSET        (MACE_ISA_EXT_OFFSET + 0x18000)
#define MACE_RTC_OFFSET         (MACE_ISA_EXT_OFFSET + 0x20000)
#define MACE_GAME_OFFSET        (MACE_ISA_EXT_OFFSET + 0x30000)

/* ISA registers (relative to MACE_ISA_OFFSET) */
#define ISA_RINGBASE_REG        0x0000
#define ISA_FLASH_NIC_REG       0x0008
#define ISA_INT_STS_REG         0x0010
#define ISA_INT_MSK_REG         0x0018

/* ISA interrupt bits (spec §5.1.3; kernel sys/uart16550.h) */
#define ISA_INT_RTC_IRQ         0x00000100

/*
 * ISA interrupt groups -> MACE vector / CRIME INTSTAT bit
 * (kernel sys/mace.h: MACE_PERIPH_AUDIO 6, MACE_PERIPH_MISC 5,
 *  MACE_PERIPH_SERIAL 4 == MACE_PERIPH_PARALLEL)
 */
#define ISA_INT_AUDIO_MASK      0x000000ffULL /* bits 0..7   -> CRIME bit 6 */
#define ISA_INT_MISC_MASK       0x0000ff00ULL /* bits 8..15  -> CRIME bit 5 */
#define ISA_INT_PERIPH_MASK     0xffff0000ULL /* bits 16..31 -> CRIME bit 4 */

/*
 * ISA serial port interrupt status/mask bits (spec §5.1.3 table;
 * kernel sys/uart16550.h ISA_SERIAL0/1_*).  Six bits per port.
 */
#define ISA_INT_S0_DIR          0x00100000ULL /* serial #1 device int req   */
#define ISA_INT_S0_TX_THIR      0x00200000ULL /* serial #1 Tx DMA threshold */
#define ISA_INT_S0_TX_PREQ      0x00400000ULL /* serial #1 Tx DMA pair req */
#define ISA_INT_S0_TX_MERR      0x00800000ULL /* serial #1 Tx DMA mem err  */
#define ISA_INT_S0_RX_THIR      0x01000000ULL /* serial #1 Rx DMA threshold */
#define ISA_INT_S0_RX_OVR       0x02000000ULL /* serial #1 Rx DMA overrun   */
#define ISA_INT_S1_DIR          0x04000000ULL /* serial #2 device int req   */
#define ISA_INT_S1_TX_THIR      0x08000000ULL /* serial #2 Tx DMA threshold */
#define ISA_INT_S1_TX_PREQ      0x10000000ULL /* serial #2 Tx DMA pair req */
#define ISA_INT_S1_TX_MERR      0x20000000ULL /* serial #2 Tx DMA mem err  */
#define ISA_INT_S1_RX_THIR      0x40000000ULL /* serial #2 Rx DMA threshold */
#define ISA_INT_S1_RX_OVR       0x80000000ULL /* serial #2 Rx DMA overrun   */

/*
 * ISA serial DMA channel pages (spec §5.1 TABLE 52: the ISA control
 * register space is 4 pages of 16KB; page 2 = serial #1, page 3 =
 * serial #2).  Relative to MACE_ISA_OFFSET.
 */
#define ISA_SER0_DMA_PAGE       0x8000
#define ISA_SER1_DMA_PAGE       0xC000

/* Serial DMA channel registers, 64-bit, within a page (spec §5.1.5) */
#define ISA_DMA_TX_CTRL         0x00    /* W: channel control             */
#define ISA_DMA_TX_RPTR         0x08    /* RO: ring read ptr (hw-owned)   */
#define ISA_DMA_TX_WPTR         0x10    /* RW: ring write ptr (sw-owned)  */
#define ISA_DMA_TX_DEPTH        0x18    /* RO: current ring depth         */
#define ISA_DMA_RX_CTRL         0x20    /* W: channel control             */
#define ISA_DMA_RX_RPTR         0x28    /* RW: ring read ptr (sw-owned)   */
#define ISA_DMA_RX_WPTR         0x30    /* RO: ring write ptr (hw-owned)  */
#define ISA_DMA_RX_DEPTH        0x38    /* RO: current ring depth         */

/* Channel control register bits (spec §5.1.5 TABLE 58) */
#define ISA_DMA_CTRL_RESET      0x400   /* bit 10: channel reset          */
#define ISA_DMA_CTRL_ENABLE     0x200   /* bit 9: DMA enable              */
#define ISA_DMA_CTRL_THR_MASK   0x0e0   /* bits 7:5: interrupt threshold   */
#define ISA_DMA_CTRL_THR_SHIFT  5
#define ISA_DMA_THR_NONE        0       /* interrupt disabled             */
#define ISA_DMA_THR_25          1       /* in: >=25% full, out: <25%       */
#define ISA_DMA_THR_50          2       /* in: >=50% full, out: <50%       */
#define ISA_DMA_THR_75          3       /* in: >=75% full, out: <75%       */
#define ISA_DMA_THR_EMPTY       4       /* ring empty                     */
#define ISA_DMA_THR_NEMPTY      5       /* ring not empty                 */
#define ISA_DMA_THR_FULL        6       /* ring full                      */
#define ISA_DMA_THR_NFULL       7       /* ring not full                  */

/*
 * Ring geometry (spec §5.1.6 / §1.2): all rings are 4KB, accessed in
 * 32-byte blocks; each block holds 16 ctrl/data pairs (4 pairs per
 * 64-bit word, big endian, ctrl bytes in the high half — spec §5.2.2).
 */
#define ISA_RING_SIZE           4096
#define ISA_RING_BLOCK_SIZE     32
#define ISA_RING_BLOCKS         (ISA_RING_SIZE / ISA_RING_BLOCK_SIZE)

/* Ring IDs (spec §5.1.6 TABLE 60) */
#define ISA_RING_ID_S0_TX       4
#define ISA_RING_ID_S0_RX       5
#define ISA_RING_ID_S1_TX       6
#define ISA_RING_ID_S1_RX       7

/* DMA pair control-byte opcodes (spec §5.2.6; kernel uart16550.h) */
#define ISA_DMA_OC_INVAL        0x00    /* invalid pair, skip              */
#define ISA_DMA_OC_TxINTR       0x20    /* post Tx pair-request interrupt  */
#define ISA_DMA_OC_WTHR         0x40    /* write data byte to THR          */
#define ISA_DMA_OC_WMCR         0x80    /* write data byte to MCR          */
#define ISA_DMA_OC_DELAY        0xc0    /* idle before next pair           */
#define ISA_DMA_IC_VALID        0x80    /* input pair is valid             */

/* Poll period for the RX DMA engine (chardev -> UART -> ring) */
#define ISA_DMA_RX_POLL_NS      1000000 /* 1 ms */

/* Serial port register spacing */
#define MACE_SERIAL_REG_SHIFT   8  /* 256-byte spacing between 16550 regs */
#define MACE_SERIAL_SIZE        0x2000  /* 8 registers * 256 bytes each */

/*
 * 16550 register indices (multiplied by 256 for actual offset)
 */
#define UART_REG_THR    0   /* TX Hold / RX Buffer (data) */
#define UART_REG_IER    1   /* Interrupt Enable */
#define UART_REG_IIR    2   /* Interrupt Ident (read) / FIFO Ctrl (write) */
#define UART_REG_LCR    3   /* Line Control */
#define UART_REG_MCR    4   /* Modem Control */
#define UART_REG_LSR    5   /* Line Status */
#define UART_REG_MSR    6   /* Modem Status */
#define UART_REG_SCR    7   /* Scratch */

/* LSR bits */
#define LSR_DR          0x01    /* Data Ready */
#define LSR_THRE        0x20    /* TX Holding Register Empty */
#define LSR_TEMT        0x40    /* Transmitter Empty */

/* LCR bits */
#define LCR_DLAB        0x80    /* Divisor Latch Access Bit */

/* IIR bits */
#define IIR_NO_INT      0x01    /* No interrupt pending */
#define IIR_FIFO_ENA    0xC0    /* FIFOs enabled */

/* MCR bits */
#define MCR_DTR         0x01
#define MCR_RTS         0x02

/* UST/MSC timer registers (relative to MACE_UST_MSC_OFFSET) */
#define UST_MSC_REG     0x00
#define UST_COMPARE1    0x08
#define UST_COMPARE2    0x10
#define UST_COMPARE3    0x18

/* UST period in nanoseconds */
#define MACE_UST_PERIOD_NS  960

/*
 * PCI host bridge registers (relative to MACE_PCI_OFFSET).
 * Spec: MACE ASIC spec §9.4 TABLE 82 (only six registers exist in the
 * 512KB region; unlisted reads return 0xFF, writes are discarded).
 */
#define MACE_PCI_ERROR_ADDR      0x000   /* R/O: PCI address of last error   */
#define MACE_PCI_ERROR_FLAGS     0x004   /* R/W: error flags (w0-clears)     */
#define MACE_PCI_CONTROL         0x008   /* R/W: int enables, arbiter, ...  */
#define MACE_PCI_FLUSH_W         0x00C   /* W:   invalidate read buffers    */
#define MACE_PCI_REV_INFO_R      0x00C   /* R:   host bridge revision       */
#define MACE_PCI_CONFIG_ADDR     0xCF8   /* W/O: type-1 config address      */
#define MACE_PCI_CONFIG_DATA     0xCFC   /* R/W: config cycle data window    */

/* PCI_ERROR_FLAGS bits (sys/mace.h PERR_*; spec TABLE 84) */
#define MACE_PERR_MASTER_ABORT      0x80000000  /* bit31: received MA       */
#define MACE_PERR_TARGET_ABORT      0x40000000  /* bit30                   */
#define MACE_PERR_DATA_PARITY_ERR   0x20000000  /* bit29                   */
#define MACE_PERR_RETRY_ERR         0x10000000  /* bit28                   */
#define MACE_PERR_ILLEGAL_CMD       0x08000000  /* bit27                   */
#define MACE_PERR_SYSTEM_ERR        0x04000000  /* bit26                   */
#define MACE_PERR_INTERRUPT_TEST    0x02000000  /* bit25                   */
#define MACE_PERR_PARITY_ERR        0x01000000  /* bit24                   */
#define MACE_PERR_OVERRUN           0x00800000  /* bit23                   */
#define MACE_PERR_MEMORY_ADDR       0x00200000  /* bit21: addr -> mem space */
#define MACE_PERR_CONFIG_ADDR       0x00100000  /* bit20: addr -> cfg space */
#define MACE_PERR_MASTER_ABORT_AV   0x00080000  /* bit19: err addr valid   */
#define MACE_PERR_TARGET_ABORT_AV   0x00040000  /* bit18                   */
#define MACE_PERR_DATA_PARITY_AV    0x00020000  /* bit17                   */
#define MACE_PERR_RETRY_AV          0x00010000  /* bit16                   */

/* MACE_PCI_CONTROL bits (spec TABLE 85) */
#define MACE_PCI_CONTROL_SCSI0_INT   0x00000001  /* bit0: PCI int #0 enable */
#define MACE_PCI_CONTROL_SCSI1_INT   0x00000002  /* bit1: PCI int #1 enable */
#define MACE_PCI_CONTROL_SLOT0_INT   0x00000004  /* bit2: slot 0 INTA#      */
#define MACE_PCI_CONTROL_SLOT1_INT   0x00000008  /* bit3: slot 1 INTA#      */
#define MACE_PCI_CONTROL_SLOT2_INT   0x00000010  /* bit4: slot 2 INTA#      */
#define MACE_PCI_CONTROL_SHARED0_INT 0x00000020  /* bit5: shared int #5     */
#define MACE_PCI_CONTROL_SHARED1_INT 0x00000040  /* bit6: shared int #6     */
#define MACE_PCI_CONTROL_SHARED2_INT 0x00000080  /* bit7: shared int #7     */
#define MACE_PCI_CONTROL_SERR_EN     0x00000100  /* bit8: SERR_N enable     */
#define MACE_PCI_CONTROL_PARITY_EN   0x00000400  /* bit10: parity response  */
#define MACE_PCI_CONTROL_MRMRA_EN    0x00000800  /* bit11: read ahead       */
#define MACE_PCI_CONTROL_ERR_INT_MASK 0xFE000000 /* bits31..24: error ints  */

/*
 * CPU-side windows onto the PCI bus (spec §9.3 figure; kernel mace.h
 * PCI_LOW_IO/PCI_LOW_MEMORY and pcimh_piotrans_addr).
 */
#define MACE_PCI_LOW_IO_BASE     0x18000000ULL  /* CPU -> PCI I/O, 32 MB    */
#define MACE_PCI_LOW_MEM_BASE    0x1A000000ULL  /* CPU -> PCI mem, 32 MB    */
#define MACE_PCI_LOW_WINDOW_SIZE 0x02000000ULL

/*
 * PCI memory-space layout inside the MACE pci_mem container (the flat
 * space where device BARs live).  The kernel allocates device memory
 * BARs from 0x80000000 upward (pcimh.c INITIAL_MSPACE) and the CPU
 * window at 0x1A000000 reaches exactly that region.  Host RAM is
 * mirrored for DMA at 0x40000000 (native view) and 0 (swapped view)
 * per the spec; QEMU RAM aliases cannot byte-swap, so the swapped view
 * is a plain alias and byte-order handling lives in the DMA devices
 * (documented in progress_notes/o2_qemu/03-mace-pci-scsi.md).
 */
#define MACE_PCI_DEV_MEM_BASE    0x80000000      /* kernel BAR alloc base   */
#define MACE_PCI_HOST_RAM_BASE   0x40000000      /* native view of host RAM */
#define MACE_PCI_HOST_RAM_SIZE   0x40000000      /* 1 GB                   */

/* Number of external PCI interrupt inputs (spec §9.6) */
#define MACE_PCI_NUM_INTS       8

/* Total MACE region size */
#define MACE_REG_SIZE   0x400000

/* Number of serial ports */
#define MACE_NUM_SERIAL 2

/* Serial RX FIFO */
#define MACE_SERIAL_FIFO_SIZE 16

struct SGIMACEState {
    /*
     * The MACE is the PCI host bridge of the O2 (TYPE_PCI_HOST_BRIDGE,
     * like bonito for MIPS); PCIHostState embeds a SysBusDevice so all
     * the sysbus wiring in sgi_o2.c keeps working unchanged.
     */
    PCIHostState parent_obj;

    MemoryRegion iomem;
    CharFrontend serial;  /* Console serial port chardev */

    /* MACE interrupt output to CRIME */
    qemu_irq crime_irq[16];

    /* ISA registers */
    uint64_t isa_ringbase;
    uint64_t isa_flash_nic;
    uint64_t isa_int_status;
    uint64_t isa_int_mask;

    /*
     * ISA serial DMA channels (spec §5.1.5), [port][dir] with dir
     * 0 = TX (output ring) and 1 = RX (input ring).  The channel
     * registers live in the ISA pages at MACE_ISA_OFFSET + 0x8000
     * (serial #1) / + 0xC000 (serial #2); the rings themselves are in
     * guest RAM at (isa_ringbase & 0xffff8000) + ring_id * 4KB.
     * Pointers are byte offsets into the 4KB ring (block << 5).
     */
    uint64_t isa_dma_ctrl[MACE_NUM_SERIAL][2];
    uint32_t isa_dma_rptr[MACE_NUM_SERIAL][2];
    uint32_t isa_dma_wptr[MACE_NUM_SERIAL][2];
    bool isa_dma_preq[MACE_NUM_SERIAL];   /* TxINTR pair seen (sticky) */

    /*
     * RX DMA engine poll timer.  The MACE DMA engine physically
     * watches the UART rx-FIFO "ready" request; here the UART is the
     * serial_mm device overlaid at MACE_SER1_OFFSET, so the engine
     * polls the UART LSR/RBR through the memory bus while the
     * console port's RX channel is enabled.
     */
    QEMUTimer *isa_rx_timer;

    /* UST/MSC timer */
    uint64_t ust_compare[3];

    /* Serial ports */
    struct {
        uint8_t thr;        /* TX holding register */
        uint8_t ier;        /* Interrupt enable */
        uint8_t iir;        /* Interrupt identification */
        uint8_t fcr;        /* FIFO control (write-only) */
        uint8_t lcr;        /* Line control */
        uint8_t mcr;        /* Modem control */
        uint8_t lsr;        /* Line status */
        uint8_t msr;        /* Modem status */
        uint8_t scr;        /* Scratch */
        uint8_t dll;        /* Divisor latch low */
        uint8_t dlh;        /* Divisor latch high */

        /* RX FIFO */
        uint8_t rx_fifo[MACE_SERIAL_FIFO_SIZE];
        int rx_fifo_head;
        int rx_fifo_tail;
        int rx_fifo_count;
    } serial_port[MACE_NUM_SERIAL];

    /*
     * DS17287 RTC — direct-mapped with 256-byte stride (IP32).
     * Register N is accessed at MACE_RTC_OFFSET + N*256 (+7 for BE byte lane).
     * 128 registers: 0-13 = time/status, 14-127 = NVRAM/extended.
     */
    uint8_t rtc_regs[128];

    /*
     * PCI host bridge (permanent, real): a QEMU PCI root bus plus the
     * six MACE host-bridge registers that forward config cycles to it.
     */
    PCIBus *pci_bus;            /* root bus, child name "mace-pci"        */
    MemoryRegion pci_mem;       /* container: device BARs + host RAM view */
    MemoryRegion pci_io;        /* container: PCI I/O space               */
    MemoryRegion pci_mem_alias; /* CPU 0x1A000000 window into pci_mem     */
    MemoryRegion pci_io_alias;  /* CPU 0x18000000 window into pci_io      */

    uint32_t pci_error_addr;    /* PCI address of the last error          */
    uint32_t pci_error_flags;   /* PERR_* status (w0-clears)               */
    uint32_t pci_control;       /* int enables / arbiter / prefetch        */
    uint32_t pci_config_addr;   /* CONFIG_ADDRESS (type-1, bit31 = enable) */
    uint32_t pci_rev_info;      /* read value of the flush/rev register    */
    uint8_t pci_int_level[MACE_PCI_NUM_INTS]; /* latched PCI INTx inputs  */
};

#endif /* HW_MISC_SGI_MACE_H */
