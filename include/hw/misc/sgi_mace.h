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
#include "hw/input/ps2.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "net/net.h"
#include "qemu/audio.h"
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
 * ISA_INT_STS/MSK bits 9 and 11: PS/2 keyboard / mouse interrupt
 * requests (spec §5.1.3 interrupt map; kernel sys/pckm.h
 * MACE_KEYBD_INTR 0x200 / MACE_MOUSE_INTR 0x800 and io/mhpckm.c
 * PCKM_MACEMASK 0xA00).  Both live in the "misc" group (bits 8..15)
 * -> CRIME bit 5 (MACE_PERIPH_MISC, MACE_INTR(5)).
 */
#define ISA_INT_KEYBOARD        0x00000200ULL /* bit 9:  PS/2 keyboard */
#define ISA_INT_MOUSE           0x00000800ULL /* bit 11: PS/2 mouse    */
#define ISA_INT_PCKM_MASK       (ISA_INT_KEYBOARD | ISA_INT_MOUSE)

/*
 * MACE PS/2 keyboard & mouse interface (spec §6 "PS/2 Keyboard & Mouse
 * Interface"; kernel sys/pckm.h struct ps2if).  Two identical ports at
 * MACE_KBDMS_OFFSET (keyboard, PIO A[5]=0) and +0x20 (mouse, A[5]=1);
 * four 64-bit registers each.  The kernel PROM/IRIX driver addresses
 * them with 64-bit PIO at byte lane +7 (PHYS_TO_K1(MACE_KBDMS)+off).
 */
#define MACE_PS2_TX_BUF         0x00    /* WO:  transmit shift buffer     */
#define MACE_PS2_RX_BUF         0x08    /* RO:  receive buffer (data low) */
#define MACE_PS2_CONTROL        0x10    /* RW:  command & control        */
#define MACE_PS2_STATUS         0x18    /* RO:  tx/rx status & error      */

/* PS/2 control register bits (spec §6.2.3 TABLE 73; sys/pckm.h PS2_CMD_*) */
#define PS2_CTRL_CLKINH         0x01    /* inhibit clock after xmission */
#define PS2_CTRL_TX_EN          0x02    /* transmit enable              */
#define PS2_CTRL_TX_IEN          0x04    /* transmit interrupt enable    */
#define PS2_CTRL_RX_IEN          0x08    /* receive interrupt enable     */
#define PS2_CTRL_CLKASS         0x10    /* assert clock (Clken)         */
#define PS2_CTRL_RESET          0x20    /* internal state machine reset */
#define PS2_CTRL_IMPLEMENTED    0x3f

/* PS/2 status register bits (spec §6.2.4 TABLE 74; sys/pckm.h PS2_SR_*) */
#define PS2_STATUS_CLKSIG       0x01    /* external clock signal        */
#define PS2_STATUS_CLKINH       0x02    /* Clken output asserted        */
#define PS2_STATUS_TIP          0x04    /* transmission in progress     */
#define PS2_STATUS_TBE          0x08    /* transmit buffer empty         */
#define PS2_STATUS_RBF          0x10    /* receive buffer full           */
#define PS2_STATUS_RIP          0x20    /* reception in progress         */
#define PS2_STATUS_PARITY       0x40    /* parity error on last byte     */
#define PS2_STATUS_FRAMING      0x80    /* framing error on last byte    */
#define PS2_STATUS_RESET_VAL    0x02    /* Clken asserted, all else 0    */

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

/*
 * Per-channel audio MSC/UST registers (kernel sys/mace.h
 * MACE_AIN_MSC_UST 0x20 / MACE_AOUT1_MSC_UST 0x28 / MACE_AOUT2_MSC_UST
 * 0x30; spec §3.4 "Stereo DMA MSC/UST Registers").  64-bit: low word =
 * sample-pair counter (MSC), high word = UST snapshot.  Writable so the
 * driver can zero the counter (a3_start_dma writes 0 before enabling).
 */
#define UST_AIN_MSCUST  0x20
#define UST_AOUT1_MSCUST 0x28
#define UST_AOUT2_MSCUST 0x30

/* UST period in nanoseconds */
#define MACE_UST_PERIOD_NS  960

/*
 * ============================================================
 * MACE audio codec interface + AD1843 codec (spec §3, TABLE 26-35)
 *
 * Register window at MACE_AUDIO_OFFSET (0x300000; kernel MACE_AUDIO
 * 0xBF300000).  16 64-bit registers at 8-byte stride:
 *   0x00 Control & Status (codec reset, codec-present, ring-ptr aliases)
 *   0x08 Codec status address/control (addr[23:17], R/W bar[16], data[15:0])
 *   0x10 Codec status input mask (interrupt mask on the status word)
 *   0x18 Codec status input (last register read value, RO)
 *   0x20..0x38  Ch1 in  ring: control/read-ptr/write-ptr/depth
 *   0x40..0x58  Ch2 out ring
 *   0x60..0x78  Ch3 out ring
 *
 * Rings: three 4KB buffers in guest RAM at
 *   (isa_ringbase & ~0x7fff) + ring_id * 4KB, ring ids 0/1/2
 *   (spec §3.5.3 TABLE 35).  64-bit stereo sample pairs, 8 bytes each.
 * Channel control register (spec TABLE 33):
 *   bit 10 reset, bit 9 DMA enable, bits 7:5 interrupt threshold.
 * Pointers (spec TABLE 31/32): byte offsets into the 4KB ring, 8-byte
 * (sample-pair) granularity — the PROM hello_tune and the kernel
 * a3_dd/kdsp drivers both traffic in byte offsets (the kdsp shadow
 * units are 4 bytes, the write pointer register value = shadow << 2
 * = bytes; observed session fill = 0x358 = 856 bytes).  The DMA
 * engine bursts four 64-bit samples (32 bytes) per transaction, so
 * output drains advance the hardware read pointer in 32-byte steps.
 * Output channels: hw owns the READ pointer (visible in cntrl_stat
 * aliases); input channel: hw owns the WRITE pointer.
 */
#define AUD_CNTRL_STAT_REG     0x00
#define AUD_CODEC_REG_REG      0x08
#define AUD_CODEC_INTR_MASK_REG 0x10
#define AUD_CODEC_READ_REG     0x18

#define AUD_CHAN_NUM           3
#define AUD_CHAN_REGS          4
#define AUD_CH_CNTRL           0
#define AUD_CH_READ            1
#define AUD_CH_WRITE           2
#define AUD_CH_DEPTH           3

/* CNTRL_STAT bits (spec TABLE 27; kernel ad1843.h) */
#define AUD_CODEC_RESET        (1 << 0)
#define AUD_CODEC_PRESENT      (1 << 1)
/* ring pointer aliases (kernel ad1843.h GET_CH*_RING_*_ALIAS) */
#define AUD_CH1_WRITE_ALIAS_SHIFT 2    /* bits 8:2  = ch1 wptr >> 5 */
#define AUD_CH1_WRITE_ALIAS_MASK  0xfc
#define AUD_CH2_READ_ALIAS_SHIFT  4    /* bits 15:9 = ch2 rptr >> 5 */
#define AUD_CH2_READ_ALIAS_MASK  0x7f00
#define AUD_CH3_READ_ALIAS_SHIFT 11    /* bits 22:16 = ch3 rptr >> 5 */
#define AUD_CH3_READ_ALIAS_MASK  0x3f8000
#define AUD_VOLUME_UP           (1 << 23)
#define AUD_VOLUME_DOWN         (1 << 24)

/* channel control bits (spec TABLE 33; kernel CHAN_*) */
#define AUD_CHAN_THR_MASK      0xe0    /* bits 7:5 interrupt threshold */
#define AUD_CHAN_THR_SHIFT     5
#define AUD_CHAN_THR_OFF       0
#define AUD_CHAN_THR_25        1
#define AUD_CHAN_THR_50        2
#define AUD_CHAN_THR_75        3
#define AUD_CHAN_THR_EMPTY     4
#define AUD_CHAN_THR_NEMPTY    5
#define AUD_CHAN_THR_FULL      6
#define AUD_CHAN_THR_NFULL     7
#define AUD_CHAN_DMA_ENABLE    (1 << 9)
#define AUD_CHAN_RESET         (1 << 10)

/* codec status address/control register (spec TABLE 28) */
#define AUD_CODEC_ADDR_SHIFT   17     /* bits 23:17: codec register addr */
#define AUD_CODEC_ADDR_MASK    0x7f
#define AUD_CODEC_READ_OP      (1 << 16)  /* 1 = read, 0 = write */

/* audio ring geometry (spec §3.5.3): 4KB rings, 32-byte blocks */
#define AUD_RING_SIZE          4096
#define AUD_RING_BLOCK         32
#define AUD_RING_ID_ADC        0      /* ring id 0: audio input  */
#define AUD_RING_ID_DAC1       1      /* ring id 1: audio out #1 */
#define AUD_RING_ID_DAC2       2      /* ring id 2: audio out #2 */

/*
 * ISA interrupt bits for audio (spec §5.1.3 table): bits 0..7, all in
 * the "audio" group (CRIME slot 6, kernel MACE_PERIPH_AUDIO).
 */
#define ISA_INT_AUD_CODEC_STATUS  0x00000001ULL  /* bit 0: status word */
#define ISA_INT_AUD_VOLUME        0x00000002ULL  /* bit 1: volume buttons */
#define ISA_INT_AUD_CH1_THIR      0x00000004ULL  /* bit 2: in #1 threshold */
#define ISA_INT_AUD_CH1_OVF       0x00000008ULL  /* bit 3: in #1 overflow */
#define ISA_INT_AUD_CH2_THIR      0x00000010ULL  /* bit 4: out #2 threshold */
#define ISA_INT_AUD_CH2_MERR       0x00000020ULL  /* bit 5: out #2 mem err */
#define ISA_INT_AUD_CH3_THIR      0x00000040ULL  /* bit 6: out #3 threshold */
#define ISA_INT_AUD_CH3_MERR      0x00000080ULL  /* bit 7: out #3 mem err */

/*
 * AD1843 codec register file (kernel sys/ad1843.h; reset defaults from
 * the PROM hello_tune.c codec_reset_default table — the kernel a3_dd
 * driver bcopy's the same table at probe).
 */
#define AD1843_NUM_REGS         32
#define AD1843_REG_STAT_REV     0
#define AD1843_REG_CH_STAT      1
#define AD1843_REG_CLK_SRC      15
#define AD1843_REG_CG1_MODE     16
#define AD1843_REG_CG1_RATE     17
#define AD1843_REG_CG2_MODE     19
#define AD1843_REG_CG2_RATE     20
#define AD1843_REG_CG3_MODE     22
#define AD1843_REG_CG3_RATE     23
#define AD1843_REG_SERIAL       26
#define AD1843_REG_CH_POWERDOWN 27
#define AD1843_REG_CONFIG       28

/* STAT_REV bits (kernel ad1843.h) */
#define AD1843_INIT             (1 << 15)  /* clock init flag */
#define AD1843_PDNO             (1 << 14)  /* conversion pwr down */
#define AD1843_REV_MASK         0xf       /* revision nibble */

/*
 * Default sample rate of the codec model when the guest never programs
 * a clock generator (the driver does program CG1; this is only a
 * fallback for the host voice).
 */
#define AUD_DEFAULT_RATE       48000

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

/* ============================================================
 * MAC110 fast ethernet (spec §4, TABLE 37..51; kernel sys/if_me.h
 * struct mac110).  The ethernet registers are 32-bit words at
 * 64-bit slots inside MACE_ENET_OFFSET (0x280000); per the spec's
 * big-endian byte lanes the 32-bit registers sit at slot+4 (the
 * drivers' PIO helpers do 32-bit accesses at the exact offsets
 * below, derived from struct mac110: __pad + reg pairs).
 */
#define MAC_REG_MAC_CONTROL     0x04    /* RW: MAC control (TABLE 38)  */
#define MAC_REG_INT_STATUS      0x0c    /* W1C: interrupt status       */
#define MAC_REG_DMA_CONTROL     0x14    /* RW: DMA control (TABLE 40)  */
#define MAC_REG_TIMER           0x1c    /* RW: RX int delay            */
#define MAC_REG_TX_ALIAS        0x24    /* WO: TX int enable alias     */
#define MAC_REG_RX_ALIAS        0x2c    /* WO: RX int enable alias     */
#define MAC_REG_TX_RING         0x34    /* RW: TX ring r/w ptrs        */
#define MAC_REG_RX_FIFO_INFO    0x44    /* RO: RX mcl FIFO w/r/d       */
#define MAC_REG_INT_REQUEST     0x58    /* WO: "generate intr" (diag)   */
#define MAC_REG_LAST_TX_VECTOR  0x58    /* RO: last TX status vector   */
#define MAC_REG_PHY_DATAIO      0x64    /* RW: PHY data (busy bit 16)  */
#define MAC_REG_PHY_ADDRESS     0x6c    /* RW: PHY dev+reg address     */
#define MAC_REG_PHY_READ_START  0x74    /* WO: initiate PHY read       */
#define MAC_REG_BACKOFF         0x7c    /* WO: backoff LFSR seed       */
#define MAC_REG_PHYSADDR        0xa0    /* RW: physical station addr   */
#define MAC_REG_SECPHYSADDR     0xa8    /* RW: secondary address       */
#define MAC_REG_MLAF            0xb0    /* RW: multicast filter hash  */
#define MAC_REG_TX_RING_BASE    0xb8    /* RW: TX ring base [31:13]    */
#define MAC_REG_RX_FIFO_DATA    0x104   /* RW: RX mcl FIFO data port   */
#define MAC_REG_RX_FIFO_ALIAS_S 0x104   /* 32 aliases of the data port */
#define MAC_REG_RX_FIFO_ALIAS_E 0x1f8

/* MAC control bits (TABLE 38; if_me.h MAC_*) */
#define MAC_CTRL_RESET          0x0001  /* core reset (sticky till 0)   */
#define MAC_CTRL_REV_SHIFT      29      /* impl revision [31:29], RO   */
#define MAC_CTRL_REV1           (1u << MAC_CTRL_REV_SHIFT) /* 1st rev */

/* Interrupt status bits (TABLE 39; if_me.h INTR_*) */
#define MAC_INTR_TX_DMA_REQ     0x01    /* TX ring empty                */
#define MAC_INTR_TX_PKT_REQ     0x02    /* TX user interrupt request   */
#define MAC_INTR_TX_LINK_FAIL   0x04
#define MAC_INTR_TX_MEMORY_ERR  0x08
#define MAC_INTR_TX_ABORTED     0x10
#define MAC_INTR_RX_DMA_REQ     0x20    /* RX threshold condition      */
#define MAC_INTR_RX_UNDERFLOW   0x40    /* mcl FIFO empty, packet lost */
#define MAC_INTR_RX_OVERFLOW    0x80
#define MAC_INTR_W1C_MASK       0x000000ffUL

/* DMA control bits (TABLE 40; if_me.h DMA_*) */
#define DMA_CTRL_TX_INTR_EN     0x0001
#define DMA_CTRL_TX_DMA_EN      0x0002
#define DMA_CTRL_TX_RING_MASK   0x000c  /* ring size: 8/16/32/64 KB    */
#define DMA_CTRL_RX_THRESH_MASK 0x01f0  /* mcl FIFO threshold [8:4]    */
#define DMA_CTRL_RX_THRESH_SHIFT 4
#define DMA_CTRL_RX_INTR_EN     0x0200
#define DMA_CTRL_RX_RUNTS_EN    0x0400
#define DMA_CTRL_RX_GATHER_EN   0x0800
#define DMA_CTRL_RX_OFFSET_MASK 0x7000  /* starting dword [14:12]      */
#define DMA_CTRL_RX_OFFSET_SHIFT 12
#define DMA_CTRL_RX_DMA_EN      0x8000

/* MDIO (spec §4.2.9 TABLE 47..49) */
#define MDIO_BUSY               0x10000

/* RX mcl FIFO info register packing (TABLE 45) */
#define RXFIFO_DEPTH_SHIFT      0
#define RXFIFO_RPTR_SHIFT       8
#define RXFIFO_GEN1_SHIFT       12
#define RXFIFO_WPTR_SHIFT       16
#define RXFIFO_GEN2_SHIFT        20

/* TX ring geometry: 128-byte descriptors, pointer = entry index */
#define MAC_TX_DESC_SIZE        128
#define MAC_TX_RING_ENTRIES     512     /* max (64KB ring)             */
#define MAC_TX_PTR_MASK         0x1ff
#define MAC_TX_WPTR_SHIFT       0       /* reg bits [8:0]              */
#define MAC_TX_RPTR_SHIFT       16      /* reg bits [24:16]            */

/* RX mcl FIFO: 16 entries of 4KB cluster base addresses.  The FIFO
 * pointers are 5-bit free-running counters (index [3:0] +
 * generation bit [4]) so empty and full are distinguishable. */
#define MAC_RX_MCL_ENTRIES      16
#define MAC_RX_MCL_IDX_MASK     15
#define MAC_RX_MCL_CNT_MASK     0x1f
#define MAC_RX_MCL_SIZE         4096

/* TX command header (spec §4.4.2) */
#define TX_CMD_LENGTH_MASK      0x00007fffULL  /* length-1             */
#define TX_CMD_OFFSET_MASK      0x007f0000ULL  /* ring data start byte */
#define TX_CMD_OFFSET_SHIFT     16
#define TX_CMD_TERM_DMA         0x00800000ULL
#define TX_CMD_SENT_INT_EN      0x01000000ULL
#define TX_CMD_CONCAT_SHIFT     25    /* bits 27:25: concat ptr valid  */

/* TX status vector (spec §4.4.4) */
#define TX_VEC_COMPLETED        0x00800000ULL
#define TX_VEC_FINISHED          0x8000000000000000ULL

/* RX status vector (spec §4.5.5, TABLE 51; if_me.h RX_VEC_*) */
#define RX_VEC_LENGTH_MASK      0x0000000000007fffULL  /* total length  */
#define RX_VEC_MULTICAST        0x0000000000080000ULL
#define RX_VEC_BROADCAST        0x0000000000100000ULL
#define RX_VEC_BAD_PACKET       0x0000000000800000ULL
#define RX_VEC_MULTICAST_MATCH  0x0000000002000000ULL
#define RX_VEC_PHYSICAL_MATCH   0x0000000004000000ULL
#define RX_VEC_SEQNUM_SHIFT     27
#define RX_VEC_CKSUM_SHIFT      32
#define RX_VALID_PACKET         0x8000000000000000ULL  /* bit 63: valid */

#define MAC_MAX_FRAME           1600   /* 1518 + margin                */
#define MAC_RX_MBUF_SIZE        2048   /* driver reads this many bytes  */

/*
 * PHY identity: the IRIX driver probe (if_me.c mace_ether_mdio_probe)
 * accepts QS6612 / ICS1889 / ICS1890 / National DP83840.  The O2
 * shipped a National DP83840-class PHY (master.d/if_me carries the
 * DP83840 rev-0 "link disconnect" errata workaround — the part SGI
 * expected in the field); model that at MDIO device address 1.
 *   reg 2 = 0x2000 (National OUI MSB), reg 3 = 0x5C0n
 *   -> identity (p2<<12)|(p3>>4) = 0x20005C0 = PHY_DP83840.
 */
#define MAC_PHY_ADDR            1
#define MAC_PHY_REG2_OUI        0x2000
#define MAC_PHY_REG3_ID         0x5c01  /* model 0x5c0, rev 1           */

/* Number of serial ports */
#define MACE_NUM_SERIAL 2

/* Serial RX FIFO */
#define MACE_SERIAL_FIFO_SIZE 16

/* Per-port PS/2 transport state (spec §6; the data plane is the core) */
typedef struct MACEPS2PortState {
    uint8_t control;            /* PS2_CTRL_* bits                     */
    uint8_t tx_byte;             /* latched tx_buf write (launched by   */
                                 /* the TxEN control write — outb())    */
    bool tx_pending;             /* tx_byte not yet shifted out          */
    int irq_level;               /* last ps2-core irq gpio level        */
} MACEPS2PortState;

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

    /*
     * PS/2 keyboard & mouse (spec §6).  Two ports: port 0 = keyboard
     * at MACE_KBDMS_OFFSET, port 1 = mouse at +0x20.  The data plane
     * is QEMU's PS/2 core (hw/input/ps2.c) embedded below — its
     * queue holds the scancode/packet stream and implements the full
     * device command set (ACK 0xFA, BAT 0xAA, keyboard ID 0xAB 0x83,
     * mouse reset etc. that the PROM mh_kbd.c and kernel mhpckm.c
     * drivers poll for).  This model implements only the MACE
     * transport: the control register, the status register (RBF from
     * the queue state, TBE always ready), and the ISA_INT_STS bits
     * 9/11 that gate into CRIME bit 5.
     */
    PS2KbdState ps2kbd;        /* port 0: keyboard core (input handler) */
    PS2MouseState ps2mouse;    /* port 1: mouse core (input handler)   */
    MACEPS2PortState ps2_port[2];

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
     * Audio codec interface + AD1843 codec (spec §3).  The codec is a
     * register file whose DAC outputs feed a QEMU audio voice; the three
     * DMA ring engines move stereo sample pairs between the rings in
     * guest RAM and the codec/voice.  Only the two output channels
     * (DAC1/DAC2, MACE "Ch2 out"/"Ch3 out") produce sound; the input
     * channel ring is serviced with silence (no host capture wired).
     */
    AudioBackend *audio_be;             /* -audiodev backend (may be NULL) */
    SWVoiceOut *audio_voice;            /* host playback voice           */
    QEMUTimer *audio_dma_timer;         /* DMA engine tick                */
    int64_t audio_tick_ns;              /* ns per 32-byte ring block      */

    uint64_t audio_cntrl_stat;          /* 0x00 CNTRL_STAT               */
    uint64_t audio_codec_reg;           /* 0x08 codec addr/control latch */
    uint16_t audio_codec_intr_mask;     /* 0x10 status-word int mask     */
    uint16_t audio_codec_read;          /* 0x18 last codec read value    */
    uint16_t ad1843_reg[AD1843_NUM_REGS];  /* codec register file         */

    /*
     * Per-channel DMA state.  ch[0] = stereo input (ring 0), ch[1] and
     * ch[2] = stereo outputs (rings 1, 2).  ctrl/read/write follow the
     * spec's register layout; mscust holds the channel's 64-bit
     * MSC/UST pair (low 32 = sample-pair counter, high 32 = UST).
     */
    uint64_t audio_ch_ctrl[AUD_CHAN_NUM];
    uint32_t audio_ch_rptr[AUD_CHAN_NUM];   /* byte offset, 32-aligned */
    uint32_t audio_ch_wptr[AUD_CHAN_NUM];
    uint64_t audio_ch_mscust[AUD_CHAN_NUM];

    /*
     * MAC110 fast ethernet (spec §4).  Register window at
     * MACE_ENET_OFFSET (0x280000), interrupt straight to CRIME bit 3
     * (MACE_ETHERNET, kernel sys/mace.h; the ethernet does NOT fan
     * through the ISA_INT map).
     */
    NICState *nic;                 /* QEMU net frontend (slirp &c)     */
    NICConf nic_conf;              /* macaddr + netdev link            */
    bool nic_present;              /* a NIC backend was instantiated   */
    QEMUTimer *ec_rx_timer;        /* wire-delay RX delivery timer     */
    uint8_t ec_rx_pending[MAC_MAX_FRAME];
    int ec_rx_pending_len;        /* -1 = no packet pending            */

    uint32_t ec_mac_control;       /* MAC_CONTROL + RO rev bits        */
    bool ec_force_off;              /* 0x58 write parked the CRIME line */
    uint32_t ec_int_status;        /* latched interrupt events         */
    uint32_t ec_dma_control;       /* DMA_CONTROL                      */
    uint32_t ec_timer;             /* interrupt delay ticks            */
    uint32_t ec_tx_ring;           /* TX ring rptr[24:16]/wptr[8:0]    */
    uint64_t ec_tx_ring_base;      /* TX ring base in guest RAM        */

    /*
     * TX ring [31:13] base is programmed by the driver; entries are
     * 128-byte descriptors fetched over DMA.
     */
    uint32_t ec_rx_fifo[MAC_RX_MCL_ENTRIES];  /* mcl cluster addrs     */
    uint32_t ec_rx_wptr;            /* mcl FIFO write index             */
    uint32_t ec_rx_rptr;            /* mcl FIFO read index (hw pops)    */
    uint32_t ec_rx_seq;             /* RX packet sequence stamp         */

    uint16_t ec_phy_reg[32];        /* emulated DP83840 PHY registers   */
    uint32_t ec_phy_addr;           /* latched MDIO dev+reg address     */
    bool ec_phy_busy;               /* MDIO transfer in progress        */
    uint64_t ec_physaddr;           /* physical station address (BE)   */
    uint64_t ec_secphysaddr;        /* secondary station address        */
    uint64_t ec_mlaf;               /* multicast hash filter            */
    uint64_t ec_last_tx_vector;     /* last TX status vector (diag)     */

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
