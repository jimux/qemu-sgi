/*
 * SGI O2 onboard AIC-7880 PCI SCSI controller — STRICT sequencer.
 * See include/hw/scsi/sgi_aic7880.h for the scope statement,
 * progress_notes/o2_qemu/35-aic-isa.md for the ISA contract and
 * progress_notes/o2_qemu/34-scsi-scope.md for the gate plan.
 *
 * M12a: the sequencer is an INTERPRETER over the driver-downloaded
 * Seq_01 microcode (4-byte instructions, opcode in byte 3).  The
 * previous placeholder faked the sequencer's driver-visible effects
 * with a timer; this file executes the actual program:
 *
 *   - fetch at seq_pc<<2 from seqram, decode imm/src/dst/op
 *   - 21 opcodes per the ISA table (10 PROVEN / 7 INFERRED / 4
 *     UNCERTAIN settled: 0x04 XOR, 0x15 JMP, 0x1b JNE-mask,
 *     0x1f JNZ — see note 35 §3 and 36-aic-sequencer.md)
 *   - the ALU register file = the 256-byte register file itself
 *     (SINDIR/DINDIR are plain indirection through SINDEX/DINDEX;
 *     on-chip SCB array at 0xa0-0xbf; ZERO/CARRY flags from ALU ops)
 *   - the INTSTAT pause contract: MVI INTSTAT with bit0 set freezes
 *     the interpreter, latches SEQINT+INTCODE, raises the PCI
 *     interrupt; the driver (kernel interrupt mode or PROM polled
 *     mode — same mechanism, different reader) services it, writes
 *     SEQADDR/unpauses, and execution resumes after the pause word
 *   - the phase-engine hooks: SCSISIG phase writes, DFCNTRL DMA
 *     enables, DFDAT FIFO, and the QINFIFO read-dequeue drive the
 *     QEMU SCSI bus through the existing request engine
 *
 * Register model: the AIC-7880 is a little-endian PCI device; the
 * IRIX driver (MIPS_BE) reaches register N at bus offset
 * conv(N) = (N & ~3) | (3 - (N & 3)) — applied per BAR access.
 *
 * Copyright (c) 2026 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_regs.h"
#include "hw/scsi/sgi_aic7880.h"
#include "hw/scsi/scsi.h"
#include "scsi/constants.h"
#include "trace.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"

/* =====================================================================
 * AIC-7880 chip registers (chip offsets, LE device view)
 * Source: kern/io/adphim/him_equ.h (LE column) — matches the standard
 * AIC-78xx register map.
 */

/* SCSI block */
#define AIC_SCSISEQ     0x00    /* scsi sequence control       (r/w)  */
#define AIC_SXFRCTL0    0x01    /* scsi transfer control 0     (r/w)  */
#define AIC_SXFRCTL1    0x02    /* scsi transfer control 1     (r/w)  */
#define AIC_SCSISIG     0x03    /* scsi bus signals            (r/w)  */
#define AIC_SCSIRATE    0x04    /* scsi rate control           (r/w)  */
#define AIC_SCSIID      0x05    /* our scsi id                (r/w)  */
#define AIC_SCSIDATL    0x06    /* scsi latched data, lo      (r/w)  */
#define AIC_SCSIDATH    0x07    /* scsi latched data, hi      (r/w)  */
#define AIC_STCNT0      0x08    /* scsi transfer count, lsb    (r/w)  */
#define AIC_STCNT1      0x09    /* scsi transfer count, mid    (r/w)  */
#define AIC_STCNT2      0x0a    /* scsi transfer count, msb    (r/w)  */
#define AIC_CLRSINT0    0x0b    /* clear scsi interrupts 0     (w)    */
#define AIC_SSTAT0      0x0b    /* scsi status 0               (r)    */
#define AIC_CLRSINT1    0x0c    /* clear scsi interrupts 1     (w)    */
#define AIC_SSTAT1      0x0c    /* scsi status 1               (r)    */
#define AIC_SSTAT2      0x0d    /* scsi status 2               (r)    */
#define AIC_SSTAT3      0x0e    /* scsi status 3               (r)    */
#define AIC_SCSITEST    0x0f    /* scsi test control           (r/w)  */
#define AIC_SIMODE0     0x10    /* scsi interrupt mask 0       (r/w)  */
#define AIC_SIMODE1     0x11    /* scsi interrupt mask 1       (r/w)  */
#define AIC_SCSIBUSL    0x12    /* scsi data bus, lo direct    (r)    */
#define AIC_SCSIBUSH    0x13    /* scsi data bus, hi direct    (r)    */
#define AIC_SHADDR0     0x14    /* scsi/host address           (r)    */
#define AIC_SELID       0x1a    /* selection/reselection id    (r)    */
#define AIC_BRDCTL      0x1d    /* board control               (r/w)  */
#define AIC_SEEPROM     0x1e    /* serial EEPROM               (r/w)  */
#define AIC_SBLKCTL     0x1f    /* scsi block control          (r/w)  */

/* SCSISEQ bits */
#define AIC_ENSELO      0x40    /* enable selection out                */
#define AIC_ENAUTOATNO  0x08    /* enable auto attention out           */
#define AIC_SCSIRSTO    0x01    /* scsi reset out                      */

/* SCSISIG bits (SCSI bus phase: CDO|IOO|MSGO) */
#define AIC_BUSPHASE    0xe0    /* phase mask: CD/IO/MSG               */
#define AIC_DOPHASE     0x00    /* data out                            */
#define AIC_DIPHASE     0x40    /* data in                             */
#define AIC_CMDPHASE    0x80    /* command                             */
#define AIC_MIPHASE     0xe0    /* message in                          */
#define AIC_MOPHASE     0xa0    /* message out                         */
#define AIC_STPHASE     0xc0    /* status                              */

/* SSTAT0 bits */
#define AIC_SELDO       0x40    /* selection out completed              */
#define AIC_SELDI       0x20    /* have been reselected                 */

/* SSTAT1 bits */
#define AIC_SELTO       0x80    /* selection timeout                  */
#define AIC_SCSIRSTI    0x20    /* scsi reset in                       */
#define AIC_BUSFREE     0x08    /* bus free detected                  */
#define AIC_SCSIPERR    0x04    /* scsi parity error                   */
#define AIC_REQINIT     0x01    /* latched req                        */
/* CLRSINT1 mask of the SSTAT1 bits it clears */
#define AIC_CLRSINT1_MASK  0xaf

/* Scratch RAM (0x20-0x5f); OPTIMA driver layout (him_equ.h + himdopt.c) */
#define AIC_SCRATCH     0x20
#define AIC_XFER_OPTION    0x20    /* transfer option map               */
#define AIC_FAST20_LOW     0x30
#define AIC_FAST20_HIGH    0x31
#define AIC_DISCON_OPTION  0x32    /* disconnect map (lo + hi byte)     */
#define AIC_WAITING_SCB    0x3a    /* SCB waiting for selection         */
#define AIC_ACTIVE_SCB     0x3b    /* SCB being executed                */
#define AIC_PASS_TO_DRIVER 0x3c    /* status byte to the driver          */
#define AIC_SCB_PTR_ARRAY  0x3d    /* dword: host SCB ptr array (LE)    */
#define AIC_QIN_CNT        0x41
#define AIC_QIN_PTR_ARRAY  0x42    /* dword: queue-in ptr array (LE)    */
#define AIC_NEXT_SCB_ARRAY 0x46    /* 16 bytes of next-scb entries      */
#define AIC_QOUT_PTR_ARRAY 0x56    /* dword: host QOUT ptr array (LE)   */
#define AIC_BUSY_PTR_ARRAY 0x5a    /* dword: busy ptr array (LE)        */

/* Sequencer registers */
#define AIC_SEQCTL      0x60    /* sequencer control           (r/w)  */
#define AIC_SEQRAM       0x61    /* sequencer ram data (auto-inc) (r/w) */
#define AIC_SEQADDR0     0x62    /* sequencer address, lo       (r/w)  */
#define AIC_SEQADDR1     0x63    /* sequencer address, hi       (r/w)  */
#define AIC_BRKADDR0     0x64    /* break address, lo           (r/w)  */
#define AIC_ACCUM        0x64    /* accumulator (alt map)       (r/w)  */
#define AIC_DINDEX       0x65    /* destination index           (r/w)  */
#define AIC_SINDEX       0x66    /* source index                (r/w)  */
#define AIC_ALLZEROS     0x6a    /* reads 0x00 / writes = NONE         */
#define AIC_ALLONES      0x69    /* reads 0xff                          */
#define AIC_FLAGS        0x6b    /* ZERO/CARRY flags             (r)    */
#define AIC_SINDIR       0x6c    /* source index, indirect      (r/w)  */
#define AIC_DINDIR       0x6d    /* destination index, indirect (r/w)  */
#define AIC_FUNCTION1    0x6e    /* function bits 6-4           (r/w)  */
#define AIC_STACK        0x6f    /* subroutine stack            (r)    */

/* SEQCTL bits */
#define AIC_PERRORDIS   0x80    /* parity error disable                */
#define AIC_FAILDIS     0x20    /* illegal opcode/address int disable  */
#define AIC_FASTMODE    0x10    /* sequencer clock select              */
#define AIC_BRKINTEN    0x08    /* breakpoint interrupt enable          */
#define AIC_SEQRESET    0x02    /* clear sequencer program counter     */
#define AIC_LOADRAM     0x01    /* sequencer ram loading mode           */

/* FLAGS bits (him_equ.h) */
#define AIC_FLAG_CARRY  0x01
#define AIC_FLAG_ZERO   0x02

/* Host/PCI block */
#define AIC_VENDID0     0x80    /* PCI vendor id byte 0 (r/o: 0x04)   */
#define AIC_VENDID1     0x81    /*                 byte 1 (0x90)      */
#define AIC_DEVID0      0x82    /* PCI device id byte 0 (0x78)         */
#define AIC_DEVID1      0x83    /*                byte 1 (0x80)      */
#define AIC_COMMAND     0x84    /* PCI command shadow          (r/w)  */
#define AIC_LATTIME     0x85    /* latency timer               (r/w)  */
#define AIC_PCISTATUS   0x86    /* PCI status shadow            (r/w)  */
#define AIC_HCNTRL      0x87    /* host control                (r/w)  */

/* HCNTRL bits */
#define AIC_POWRDN      0x40    /* power down                         */
#define AIC_BANKSEL     0x20    /* scratch bank select                */
#define AIC_SWINT       0x10    /* force interrupt                     */
#define AIC_IRQMS       0x08    /* 0 = high true edge, 1 = low true level */
#define AIC_PAUSE       0x04    /* pause sequencer (write)              */
#define AIC_PAUSEACK    0x04    /* sequencer paused (read)              */
#define AIC_INTEN       0x02    /* enable hardware interrupt           */
#define AIC_CHIPRESET   0x01    /* device hard reset (write)           */

#define AIC_HADDR0      0x88    /* host DMA address 0..3       (r/w)  */
#define AIC_HCNT0       0x8c    /* host DMA count 0..2         (r/w)  */

/* Queue/SCB block */
#define AIC_SCBPTR       0x90    /* SCB pointer (bank select)   (r/w)  */
#define AIC_INTSTAT      0x91    /* interrupt status            (r)    */
#define AIC_CLRINT       0x92    /* clear interrupt status      (w)    */
#define AIC_ERROR        0x92    /* hard error                  (r)    */
#define AIC_DFCNTRL      0x93    /* data fifo control           (r/w)  */
#define AIC_DFSTATUS     0x94    /* data fifo status            (r)    */
#define AIC_DFWADDR0     0x95
#define AIC_DFDAT        0x99
#define AIC_SCBCNT       0x9a
#define AIC_QINFIFO      0x9b    /* queue in fifo               (r/w)  */
#define AIC_QINCNT       0x9c    /* queue in count              (r)    */
#define AIC_QOUTFIFO     0x9d    /* queue out fifo              (r/w)  */
#define AIC_QOUTCNT      0x9e    /* queue out count             (r)    */
#define AIC_SFUNCT       0x9f
#define AIC_SCB00        0xa0    /* on-chip SCB array 00..31    (r/w)  */

/* INTSTAT bits */
#define AIC_SEQINT      0x01    /* sequencer paused itself              */
#define AIC_CMDCMPLT    0x02    /* SCB done, no error                   */
#define AIC_SCSIINT     0x04    /* scsi event                           */
#define AIC_BRKINT      0x08    /* breakpoint                           */
#define AIC_INTCODE     0xf0    /* sequencer interrupt code (with SEQINT) */
#define AIC_ANYPAUSE    (AIC_BRKINT | AIC_SCSIINT | AIC_SEQINT)

/* CLRINT bits */
#define AIC_CLRSEQINT  0x01
#define AIC_CLRCMDINT  0x02
#define AIC_CLRSCSINT  0x04
#define AIC_CLRBRKINT  0x08

/* DFCNTRL bits */
#define AIC_SCSIEN      0x20    /* enable xfer: scsi <-> sfifo  (write) */
#define AIC_SDMAEN      0x10    /* enable xfer: sfifo <-> dfifo  (write) */
#define AIC_HDMAEN      0x08    /* enable xfer: dfifo <-> host   (write) */
#define AIC_DIRECTION   0x04    /* transfer direction: 1 = to host       */
#define AIC_FIFOFLUSH   0x02    /* flush data fifo to host              */
#define AIC_FIFORESET   0x01    /* reset data fifo                      */

/* DFSTATUS bits (read) */
#define AIC_DFTHRSH     0x04
#define AIC_HDONE       0x08
#define AIC_MREQPEND    0x10
#define AIC_FIFOFULL    0x02
#define AIC_FIFOEMP     0x01

/* Sequencer SCB layout (32 bytes, driver writes BE dwords) */
#define AIC_SCB_DWORD_TARLUN   0   /* Tarlun | flags | CDBLen | SegCnt */
#define AIC_SCB_DWORD_SEG      1   /* SG list ptr | PCI_NATIVE_VIEW     */
#define AIC_SCB_DWORD_CDB      2   /* CDB ptr (swapped view)             */
#define AIC_SCB_MAX            32

/* Sequencer microcode opcode numbers (see note 35 §3) */
#define SEQ_OP_MOV     0x00
#define SEQ_OP_OR      0x01
#define SEQ_OP_AND     0x02
#define SEQ_OP_ANDN    0x03
#define SEQ_OP_XOR     0x04
#define SEQ_OP_ADD     0x06
#define SEQ_OP_ADD2    0x07
#define SEQ_OP_ADC     0x08
#define SEQ_OP_ADD3    0x0a
#define SEQ_OP_JMP     0x10
#define SEQ_OP_CALL    0x11
#define SEQ_OP_JMP2    0x15
#define SEQ_OP_JNZF    0x17
#define SEQ_OP_JEQM    0x18
#define SEQ_OP_JNEM    0x19
#define SEQ_OP_JNZ     0x1a
#define SEQ_OP_JNEM2   0x1b
#define SEQ_OP_JE      0x1c
#define SEQ_OP_JNE     0x1d
#define SEQ_OP_JZ      0x1e
#define SEQ_OP_JNZ2    0x1f

/* PCI IDs (adp78 registers 0x9004:0x8078 and 0x8178) */
#define ADAPTEC_VENDOR_ID   0x9004
#define ADAPTEC_DEVICE_ID   0x8078

/* Verbose debug logging — set to 1 to trace the interpreter. */
#define DEBUG_SGI_AIC7880 0

#if DEBUG_SGI_AIC7880
#define AIC_DPRINTF(fmt, ...) \
    fprintf(stderr, "AIC7880[%02d]: " fmt, s->scsi_bus_num, ##__VA_ARGS__)
#else
#define AIC_DPRINTF(fmt, ...) do {} while (0)
#endif

/* =====================================================================
 * Helpers
 */

/* bus offset -> chip register offset (driver's osm_conv_byteptr) */
static inline uint32_t aic_conv(uint32_t off)
{
    return (off & ~3u) | (3u - (off & 3u));
}

static inline uint32_t aic_scratch_le32(SGIAIC7880State *s, uint32_t chip)
{
    return s->regs[chip] | (s->regs[chip + 1] << 8) |
        (s->regs[chip + 2] << 16) | ((uint32_t)s->regs[chip + 3] << 24);
}

/* Host-memory dword in driver (big-endian) order */
static inline uint32_t aic_host_ldl_be(SGIAIC7880State *s, uint32_t addr)
{
    AddressSpace *as = &address_space_memory;

    return address_space_ldl_be(as, addr, MEMTXATTRS_UNSPECIFIED, NULL);
}

static inline void aic_host_st8(SGIAIC7880State *s, uint32_t addr, uint8_t v)
{
    address_space_stb(&address_space_memory, addr, v,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static inline uint8_t aic_host_ldub(SGIAIC7880State *s, uint32_t addr)
{
    return address_space_ldub(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * Read one QOUT ring byte lane the driver would see for a given
 * driver-side index value (index -> conv lane -> host byte).
 */
static uint8_t aic7880_qout_lane(SGIAIC7880State *s, uint8_t index)
{
    if (!s->qout_base) {
        return 0xff;
    }
    return aic_host_ldub(s, s->qout_base + aic_conv(index));
}

/* INTA level: any pending INTSTAT event (or SWINT) gated by INTEN */
static void aic7880_update_irq(SGIAIC7880State *s)
{
    int level = (s->regs[AIC_HCNTRL] & AIC_INTEN) &&
        ((s->intstat & 0x0f) || (s->regs[AIC_HCNTRL] & AIC_SWINT));

    pci_set_irq(PCI_DEVICE(s), level);
}

/* =====================================================================
 * FIFO + host DMA engine
 */

static void aic7880_fifo_reset(SGIAIC7880State *s)
{
    s->fifo_head = s->fifo_tail = s->fifo_count = 0;
}

static void aic7880_fifo_push(SGIAIC7880State *s, uint8_t v)
{
    if (s->fifo_count < AIC7880_FIFO_SIZE) {
        s->fifo[s->fifo_tail] = v;
        s->fifo_tail = (s->fifo_tail + 1) % AIC7880_FIFO_SIZE;
        s->fifo_count++;
    }
}

static uint8_t aic7880_fifo_pop(SGIAIC7880State *s)
{
    uint8_t v = 0;

    if (s->fifo_count) {
        v = s->fifo[s->fifo_head];
        s->fifo_head = (s->fifo_head + 1) % AIC7880_FIFO_SIZE;
        s->fifo_count--;
    }
    return v;
}

/* Host address composed from HADDR0-3 */
static uint32_t aic7880_haddr(SGIAIC7880State *s)
{
    return s->regs[AIC_HADDR0] | (s->regs[AIC_HADDR0 + 1] << 8) |
        (s->regs[AIC_HADDR0 + 2] << 16) |
        ((uint32_t)s->regs[AIC_HADDR0 + 3] << 24);
}

/* Transfer count composed from HCNT0-2 */
static uint32_t aic7880_hcnt(SGIAIC7880State *s)
{
    return s->regs[AIC_HCNT0] | (s->regs[AIC_HCNT0 + 1] << 8) |
        ((uint32_t)s->regs[AIC_HCNT0 + 2] << 16);
}

/*
 * Service a host DMA enable (DFCNTRL HDMAEN rising or FIFOFLUSH).
 * Direction model (the DFCNTRL DIRECTION bit describes the SCSI
 * phase, not the host bus; the host direction is inferred by the
 * engine state, which both Seq_01 revisions drive identically):
 *   - FIFOFLUSH, or HDMAEN with a non-empty FIFO => FIFO -> host
 *     (the QOUT completion write: DFDAT staged + FIFOFLUSH+HDMAEN)
 *   - HDMAEN with an empty FIFO => host -> FIFO (the SCB download:
 *     DFCNTRL 0x0d after a FIFORESET; also data-in staging)
 * Updates HADDR/HCNT and sets HDONE when the count drains.
 */
static void aic7880_hdma_run(SGIAIC7880State *s)
{
    uint32_t addr = aic7880_haddr(s);
    uint32_t cnt = aic7880_hcnt(s);
    bool to_host = (s->regs[AIC_DFCNTRL] & AIC_FIFOFLUSH) != 0 ||
        s->fifo_count != 0;
    uint32_t n;

    if (to_host) {
        n = MIN(cnt, s->fifo_count);
        if (n == 0) {
            s->hdma_done = true;
            return;
        }
    } else {
        if (cnt == 0) {
            s->hdma_done = true;
            return;
        }
        n = MIN(cnt, AIC7880_FIFO_SIZE - s->fifo_count);
    }
    for (uint32_t i = 0; i < n; i++) {
        if (to_host) {
            aic_host_st8(s, addr + i, aic7880_fifo_pop(s));
        } else {
            aic7880_fifo_push(s, aic_host_ldub(s, addr + i));
        }
    }
    addr += n;
    cnt -= n;
    s->regs[AIC_HADDR0] = addr & 0xff;
    s->regs[AIC_HADDR0 + 1] = (addr >> 8) & 0xff;
    s->regs[AIC_HADDR0 + 2] = (addr >> 16) & 0xff;
    s->regs[AIC_HADDR0 + 3] = (addr >> 24) & 0xff;
    s->regs[AIC_HCNT0] = cnt & 0xff;
    s->regs[AIC_HCNT0 + 1] = (cnt >> 8) & 0xff;
    s->regs[AIC_HCNT0 + 2] = (cnt >> 16) & 0xff;
    if (cnt == 0) {
        s->hdma_done = true;
    }
    trace_sgi_aic7880_hdma(s->scsi_bus_num, to_host, n,
                           aic7880_haddr(s), aic7880_hcnt(s));
}

/* =====================================================================
 * SCSI bus side: SG-list cursor (kept from the placeholder — the bus
 * data path is host-side and orthogonal to the sequencer decode)
 */

static void aic7880_kick(SGIAIC7880State *s);

/* Advance the SG cursor to the next element with bytes remaining */
static void aic7880_sg_next(SGIAIC7880State *s)
{
    while (s->sg_index < s->sg_count) {
        uint32_t e = s->sg_list + 8u * s->sg_index;
        uint32_t addr = aic_host_ldl_be(s, e) & 0x3fffffffu;
        uint32_t len = aic_host_ldl_be(s, e + 4) & 0x7fffffffu;

        if (len == 0) {
            s->sg_index++;
            continue;
        }
        s->sg_elem_addr = addr;
        s->sg_elem_left = len;
        return;
    }
    s->sg_elem_left = 0;
}

/*
 * Move one data chunk between the target's buffer and the guest SG
 * list.  Byte streams both ways: the driver pre-swizzles structures
 * and the MACE byte-swapped DMA view preserves FIFO byte order, so
 * plain ordered byte copies are exactly what lands in host memory.
 */
static void aic7880_sg_move(SGIAIC7880State *s, uint8_t *buf,
                            uint32_t len, bool to_host)
{
    while (len) {
        uint32_t n;

        if (s->sg_elem_left == 0) {
            if (s->sg_index >= s->sg_count) {
                return; /* guest SG exhausted (underrun) */
            }
            aic7880_sg_next(s);
            if (s->sg_elem_left == 0) {
                return;
            }
            s->sg_index++;
        }
        n = MIN(len, s->sg_elem_left);
        if (to_host) {
            address_space_write(&address_space_memory, s->sg_elem_addr,
                                MEMTXATTRS_UNSPECIFIED, buf, n);
        } else {
            address_space_read(&address_space_memory, s->sg_elem_addr,
                               MEMTXATTRS_UNSPECIFIED, buf, n);
        }
        s->sg_elem_addr += n;
        s->sg_elem_left -= n;
        buf += n;
        len -= n;
    }
}

/* =====================================================================
 * SCSI request engine (bus side, driven by the microcode's phase
 * sequencing via SCSISIG writes + the selection setup in the SCB)
 */

/* Fetch + execute one SCB from host memory into the on-chip array */
static bool aic7880_fetch_scb(SGIAIC7880State *s, uint8_t scb_num)
{
    uint32_t arr, entry, scb_host;
    uint8_t bank = scb_num & 0x7f;
    int i;

    arr = aic_scratch_le32(s, AIC_SCB_PTR_ARRAY) & 0x3fffffffu;
    if (!arr) {
        return false;
    }
    entry = aic_host_ldl_be(s, arr + 4u * scb_num);
    scb_host = entry & 0x3fffffffu;
    for (i = 0; i < 32; i += 4) {
        uint32_t d = aic_host_ldl_be(s, scb_host + i);

        /* driver writes the seq SCB in BE dwords; on-chip layout is
         * byte-linear in the same order (SCB_DWORD_* indexing in the
         * placeholder's decode) */
        s->scb[bank][i] = d & 0xff;
        s->scb[bank][i + 1] = (d >> 8) & 0xff;
        s->scb[bank][i + 2] = (d >> 16) & 0xff;
        s->scb[bank][i + 3] = (d >> 24) & 0xff;
    }
    trace_sgi_aic7880_scb_fetch(s->scsi_bus_num, scb_num, scb_host,
                                s->scb[bank][0]);
    return true;
}

/* Kick the bus request for the current SCB (selection semantics) */
static void aic7880_start_selection(SGIAIC7880State *s, uint8_t scb_num)
{
    uint8_t bank = scb_num & 0x7f;
    uint32_t d0, d1, d2;
    uint32_t tarlun, cdb_ptr, cdb_len, seg_cnt, seg_ptr;
    uint8_t cdb[16];
    SCSIDevice *sdev;
    int32_t datalen;
    int i;

    d0 = s->scb[bank][0] | (s->scb[bank][1] << 8) |
        (s->scb[bank][2] << 16) | ((uint32_t)s->scb[bank][3] << 24);
    d1 = s->scb[bank][4] | (s->scb[bank][5] << 8) |
        (s->scb[bank][6] << 16) | ((uint32_t)s->scb[bank][7] << 24);
    d2 = s->scb[bank][8] | (s->scb[bank][9] << 8) |
        (s->scb[bank][10] << 16) | ((uint32_t)s->scb[bank][11] << 24);
    /*
     * dword0 is the driver's bitfield word: MIPSpro allocates bitfields
     * LSB-first, so the BE dword in memory is
     *   [31:24] SegCnt, [23:16] CDBLen, [15:8] tag/discon flags,
     *   [7:0] Tarlun (target<<4 | lun).
     */
    tarlun = d0 & 0xff;
    cdb_len = (d0 >> 16) & 0xff;
    seg_cnt = d0 >> 24;
    seg_ptr = d1 & 0x3fffffffu;
    cdb_ptr = d2 & 0x3fffffffu;

    if (cdb_len == 0 || cdb_len > sizeof(cdb)) {
        cdb_len = sizeof(cdb);
    }
    for (i = 0; i < (int)cdb_len; i++) {
        cdb[i] = aic_host_ldub(s, cdb_ptr + i);
    }

    s->cur_scb = scb_num;
    s->sg_list = seg_ptr;
    s->sg_count = seg_cnt ? seg_cnt : 1;
    s->sg_index = 0;
    s->sg_elem_left = 0;

    sdev = scsi_device_find(&s->bus, 0, tarlun >> 4, tarlun & 0xf);
    trace_sgi_aic7880_sel_start(s->scsi_bus_num, scb_num, tarlun,
                                sdev ? 1 : 0);
    if (!sdev) {
        /* Selection timeout: SELTO + WAITING_SCB + SCSIINT (the
         * placeholder contract, now surfaced as SSTAT1 for the
         * microcode's SELTO polling).  The selection observed-bit
         * (SSTAT0 SELDO) is NOT left set: the driver's Ph_IntSelto
         * services the timeout without clearing it (him.c:1589 writes
         * only CLRSINT1), and a stale SELDO diverts the idle loop at
         * word 8 into the retired SCB's phase wait — the M12a2
         * stale-SELDO lesson, same root cause as the completion path.
         * The sequencer returns to idle with the bus free. */
        trace_sgi_aic7880_selto(s->scsi_bus_num, scb_num, tarlun >> 4);
        s->regs[AIC_WAITING_SCB] = scb_num;
        s->regs[AIC_SSTAT1] |= AIC_SELTO;
        s->intstat |= AIC_SCSIINT;
        s->bus_selected = false;
        /* the selection is retired: same SCB number may retry */
        s->cur_scb = 0xff;
        /*
         * Return the sequencer to the idle loop.  The pop fired the
         * selection from inside word 15's ALU write; the natural
         * successor (word 16's fast route for SCBPTR==0 -> words
         * 154-161) walks the retired SCB's data-phase prep, whose
         * DFSTATUS MREQPEND polling (word 158's JNZ -> 158 self-
         * jump) spins forever against our always-set MREQPEND
         * synthesis.  A bare seq_pc store here would be clobbered
         * by the interpreter's PC commit after the write hook
         * returns (M12a2) — use the redirect the commit honors.
         */
        s->seq_redirect = 0;
        s->last_fetched_scb = 0xff;
        s->regs[AIC_SCBPTR] = 0xff;
        aic7880_update_irq(s);
        return;
    }
    s->bus_selected = true;
    s->regs[AIC_SSTAT0] |= AIC_SELDO;  /* selection completed */
    s->cur_phase = AIC_MOPHASE;     /* selection with ATN -> MSG-OUT */

    /*
     * MODE SENSE(6) page 0 to a CD-ROM: QEMU's scsi-cd cannot serve
     * it (scsi-disk.c mode_sense_page() rejects the page for
     * TYPE_ROM -> ASC 0x24); IRIX mount(1) queries exactly this.
     * Synthesize what a real SCSI-2 CD-ROM answers.  This stays
     * until 12d decides the policy (note 34 §4).
     */
    if (cdb[0] == MODE_SENSE && (cdb[2] & 0x3f) == MODE_PAGE_VENDOR_SPECIFIC
        && sdev->type == TYPE_ROM && ((cdb[2] >> 6) & 0x3) != 3) {
        uint8_t resp[12];
        uint64_t nblocks = sdev->max_lba + 1;
        bool dbd = (cdb[1] & 0x8) != 0;
        uint32_t len;

        memset(resp, 0, sizeof(resp));
        resp[2] = 0x80;                    /* SW: write-protected */
        if (dbd) {
            resp[0] = 5;
            resp[4] = MODE_PAGE_VENDOR_SPECIFIC;
            resp[5] = 0;
            len = 6;
        } else {
            if (nblocks > 0xffffff) {
                nblocks = 0;
            }
            resp[0] = 11;
            resp[3] = 8;
            resp[5] = (nblocks >> 16) & 0xff;
            resp[6] = (nblocks >> 8) & 0xff;
            resp[7] = nblocks & 0xff;
            resp[9] = (sdev->blocksize >> 16) & 0xff;
            resp[10] = (sdev->blocksize >> 8) & 0xff;
            resp[11] = sdev->blocksize & 0xff;
            len = 12;
        }
        len = MIN(len, cdb[4]);
        aic7880_sg_move(s, resp, len, true);
        trace_sgi_aic7880_mode_sense_page0_synth(
            s->scsi_bus_num, tarlun >> 4, tarlun & 0xf, dbd,
            (cdb[2] >> 6) & 0x3, cdb[4], sdev->blocksize, nblocks);
        /*
         * Surface as a GOOD completion through the SAME deferred-post
         * machinery as a real request (the M12a2 CD-boot stall: this
         * path used to return after stuffing the data with no
         * completion scheduled — no .complete ever fired, the QOUT
         * byte never posted, and the 60 s adp timeout + reset killed
         * every later command on the controller).
         */
        s->cur_scb = scb_num;
        s->done_status = GOOD;
        s->done_scb = scb_num;
        if (!s->done_pending) {
            s->done_pending = true;
            timer_mod(s->seq_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
        }
        return;
    }

    s->cur_req = scsi_req_new(sdev, 0, tarlun & 0xf, cdb, cdb_len, s);
    datalen = scsi_req_enqueue(s->cur_req);
    if (datalen != 0) {
        scsi_req_continue(s->cur_req);
    }
}

/* SCSIBusInfo: data phase chunk from the target */
static void aic7880_transfer_data(SCSIRequest *req, uint32_t len)
{
    SGIAIC7880State *s = req->hba_private;
    uint8_t *buf = scsi_req_get_buf(req);

    if (!s->cur_req || s->cur_req != req) {
        return;
    }
    aic7880_sg_move(s, buf, len, req->cmd.mode == SCSI_XFER_FROM_DEV);
    scsi_req_continue(req);
    aic7880_kick(s);
}

/* SCSIBusInfo: command done */
/*
 * Deferred completion posting: the request engine's .complete callback
 * records the result; the driver-visible posting (QOUT byte, INTSTAT,
 * IRQ) happens from the sequencer timer a moment later — matching the
 * real chip, where the target's STATUS/MESSAGE phases take bus time
 * and the completion lands while the host is in its wait loop, not
 * synchronously inside its own enqueue MMIO store.  Posting inline
 * raced the driver's paused-window register dance (observed: the PROM
 * poll loop and the kernel ISR both missed CMDCMPLT).
 */
static void aic7880_post_complete(SGIAIC7880State *s)
{
    int status = s->done_status;
    int done_scb = s->done_scb;
    uint32_t qout;

    s->done_pending = false;

    /*
     * Completion posting (strict-registers tier): a real chip's
     * sequencer walks the target's STATUS phase, stores the byte,
     * advances the QOUT slot byte (the r56 arithmetic, words 220-227)
     * and raises INTSTAT CMDCMPLT.  The wire-level phase/message
     * engine that drives those words is the follow-on milestone; the
     * observable driver contract is reproduced here: the SCB byte
     * lands in the host QOUT ring at the slot Ph_checkqout reads
     * (descending by 4 from the pointer low byte — the driver
     * 256-aligns the array so the pointer's LSB doubles as the slot
     * index), the NEXT slot is pre-invalidated (the 0x8027c604
     * phantom-completion lesson), PASS_TO_DRIVER carries the status
     * byte for the CHECK_CONDX dispatch, and CMDCMPLT (or
     * SEQINT+CHECK_CONDX for a non-GOOD status) raises the IRQ.
     */

    s->regs[AIC_SCSIDATL] = status;
    s->regs[AIC_PASS_TO_DRIVER] = status;
    s->regs[AIC_ACTIVE_SCB] = done_scb;
    s->cur_phase = AIC_STPHASE;
    s->bus_selected = false;
    /*
     * Phase-machine exit state (the QOUT-desync root cause, M12a2):
     * on real silicon the completion walk (words 220-230) ends with
     * the target in BUS FREE and the selection-observed bits
     * CONSUMED — the idle loop's word-91/92 walk clears SSTAT0
     * SELDO (MVI CLRSINT0, 0x40) before word 230's JMP 0.  Our
     * C-side completion bypasses those words, so it must clear the
     * bits itself: leaving SELDO set diverts the idle loop at word 8
     * (JNZ SSTAT0 & 0x40 -> 85) into the stale SCB's data-phase wait
     * (words 101-118, the loop the e5b PC histogram caught spinning
     * 567k times), where it never reaches word 14's QINCNT gate —
     * the next queued command is never popped and times out (the
     * "one completion per reset cycle" signature: every reset's
     * Ph_set_scratchmem/Ph_ResetChannel pair clears SSTAT via the
     * host CLRSINT0 0xff write, which is exactly why the first
     * command after each reset worked and every second one died).
     */
    s->regs[AIC_SSTAT0] &= ~(AIC_SELDO | AIC_SELDI);    /* consumed */
    s->regs[AIC_SSTAT1] &= ~(AIC_SELTO | AIC_REQINIT);   /* bus free */
    s->regs[AIC_SSTAT1] |= AIC_BUSFREE;

    qout = s->qout_base;
    if (qout) {
        uint32_t slot = s->qout_slot & 0xff;

        aic_host_st8(s, qout + aic_conv(slot), done_scb);
        aic_host_st8(s, qout + aic_conv((slot - 4) & 0xff), 0xff);
        trace_sgi_aic7880_qout_post(s->scsi_bus_num, done_scb, slot,
            aic_conv(slot),
            aic_host_ldub(s, qout + aic_conv(4)),
            aic_host_ldub(s, qout + aic_conv((slot - 4) & 0xff)));
        s->qout_slot = (slot - 4) & 0xff;
        /*
         * Sequencer-faithful r56 mirror: the microcode's words 224-226
         * advance the scratch byte at 0x56 as the slot counter.  The
         * write address above uses qout_base, so this can no longer
         * corrupt the ring pointer (the M12a2 pointer-corruption bug:
         * writing the slot into the pointer dword's LSB moved the
         * device's ring base +0xfc after the second post, stranding
         * every later completion at an address the driver never
         * reads — the "3 completions then timeout" signature).
         */
        s->regs[AIC_QOUT_PTR_ARRAY] = s->qout_slot;
    }
    if (status == GOOD) {
        s->intstat |= AIC_CMDCMPLT;
    } else {
        s->intstat |= AIC_SEQINT | 0x50;   /* INTCODE CHECK_CONDX */
    }
    /*
     * Retire the SCB and return the sequencer to the idle loop (the
     * real sequencer's word-230 'JMP 0' after posting the QOUT byte).
     * Without this the still-running microcode walks the stale bank
     * into the word-169 invalid-SCB test and pauses NO_ID_MSG (pc 73),
     * which the driver treats as an abort path.
     */
    trace_sgi_aic7880_bus_complete(s->scsi_bus_num, done_scb, status);
    s->last_fetched_scb = 0xff;
    s->cur_scb = 0xff;
    s->regs[AIC_SCBPTR] = 0xff;   /* no loaded SCB: re-arm selects nothing */
    s->seq_pc = 0;                /* idle loop */
    s->seq_int_pause = false;
    aic7880_update_irq(s);
    aic7880_kick(s);
}

/* SCSIBusInfo: command done */
static void aic7880_command_complete(SCSIRequest *req, size_t residual)
{
    SGIAIC7880State *s = req->hba_private;
    int status = req->status;

    if (s->cur_req != req) {
        return;
    }
    s->cur_req = NULL;
    scsi_req_unref(req);

    /*
     * Deferred posting: the driver-visible completion (QOUT byte,
     * INTSTAT CMDCMPLT, IRQ) lands from the sequencer timer a moment
     * later — the real chip walks the target's STATUS/MESSAGE phases
     * asynchronously, and posting inline inside the guest's own
     * enqueue MMIO store raced the driver's paused-window register
     * dance (observed: both the PROM poll loop and the kernel ISR
     * missed CMDCMPLT posted synchronously).
     */
    s->done_status = status;
    s->done_scb = s->cur_scb;
    if (!s->done_pending) {
        s->done_pending = true;
        timer_mod(s->seq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
}

/* SCSIBusInfo: request cancelled (driver abort path) */
static void aic7880_request_cancelled(SCSIRequest *req)
{
    SGIAIC7880State *s = req->hba_private;

    qemu_log_mask(LOG_UNIMP, "sgi_aic7880: request cancelled (scb %d)\n",
                  s->cur_scb);
    if (s->cur_req == req) {
        s->cur_req = NULL;
        scsi_req_unref(req);
    }
}

static const SCSIBusInfo aic7880_scsi_info = {
    .tcq = false,
    .max_target = 8,
    .max_lun = 8,

    .transfer_data = aic7880_transfer_data,
    .complete = aic7880_command_complete,
    .cancel = aic7880_request_cancelled,
};

/*
 * Clear pending unit attention on every device on our bus.  The QEMU
 * SCSI layer reports a power-on UA after VM start; the real O2 boot
 * flow asserts SCSI reset from the driver before probing (adp_reset).
 */
static void aic7880_clear_ua(SGIAIC7880State *s)
{
    SCSISense none = { .key = 0, .asc = 0, .ascq = 0 };
    int id, lun;

    s->bus.unit_attention = none;
    for (id = 0; id < 8; id++) {
        for (lun = 0; lun < 8; lun++) {
            SCSIDevice *d = scsi_device_find(&s->bus, 0, id, lun);

            if (d) {
                d->unit_attention = none;
            }
        }
    }
}

/* =====================================================================
 * The sequencer interpreter
 */

/*
 * QINFIFO read-dequeue: reading QINFIFO pops the next queued SCB
 * number (real-chip behavior the idle loop depends on, word 15).
 */
static uint8_t aic7880_qinfifo_read(SGIAIC7880State *s)
{
    uint8_t v;

    if (s->qin_count == 0) {
        /*
         * Empty queue reads as INVALID_SCB_INDEX (0xff): word 14's
         * QINCNT gate normally prevents this read, but the abort-path
         * rotations and any race read must not look like SCB 0 — the
         * microcode's word-16 reg-zero test routes 0 into the
         * selection prep, so a phantom 0 here selects a stale bank.
         */
        return 0xff;
    }
    v = s->qin[s->qin_head];
    s->qin_head = (s->qin_head + 1) & 0xff;
    s->qin_count--;
    trace_sgi_aic7880_qin_pop(s->scsi_bus_num, v, s->qin_count);
    return v;
}


/* Sequencer-visible register read (chip offsets) */
static uint8_t aic7880_seq_reg_read(SGIAIC7880State *s, uint8_t chip)
{
    switch (chip) {
    case AIC_ALLZEROS:
        return 0x00;
    case AIC_ALLONES:
        return 0xff;
    case AIC_INTSTAT:
        return s->intstat;
    case AIC_DFSTATUS: {
        uint8_t v = AIC_DFTHRSH | AIC_MREQPEND;

        if (s->fifo_count == 0) {
            v |= AIC_FIFOEMP;
        }
        if (s->fifo_count >= AIC7880_FIFO_SIZE) {
            v |= AIC_FIFOFULL;
        }
        if (s->hdma_done) {
            v |= AIC_HDONE;
        }
        return v;
    }
    case AIC_QINCNT:
        return s->qin_count;
    case AIC_QOUTCNT:
        return 0;
    case AIC_QINFIFO:
        /* sequencer read of QINFIFO = dequeue the next SCB number */
        return aic7880_qinfifo_read(s);
    case AIC_STACK:
        return s->seq_sp ? (s->seq_stack[s->seq_sp - 1] & 0xff) : 0;
    case AIC_SCSIBUSL:
        /* direct bus read: current phase + data latch (target status
         * byte visible during STATUS phase) */
        return s->regs[AIC_SCSIDATL];
    case AIC_ERROR:
        return 0x00;
    default:
        break;
    }
    if (chip >= AIC_SCB00 && chip < AIC_SCB00 + 32) {
        return s->scb[s->regs[AIC_SCBPTR] & 0x7f][chip - AIC_SCB00];
    }
    return s->regs[chip];
}

/*
 * Sequencer-visible register write with side effects.  This is the
 * phase-engine hook: the microcode's writes here drive the device.
 * Returns true when the write raised a sequencer pause (INTSTAT
 * SEQINT pattern).
 */
static bool aic7880_seq_reg_write_ex(SGIAIC7880State *s, uint8_t chip,
                                     uint8_t val, bool from_pop)
{
    switch (chip) {
    case AIC_INTSTAT:
        /*
         * The pause contract: bits 3-0 are OR'd into INTSTAT (pending
         * '1's preserved per him_equ.h); bit0 (SEQINT) with an INTCODE
         * nibble freezes the sequencer here — the host services it and
         * resumes after this word.
         */
        s->intstat |= (val & 0x0f) | (val & 0xf0);
        if (val & AIC_SEQINT) {
            trace_sgi_aic7880_seq_pause(s->scsi_bus_num, s->seq_pc,
                                        (val & 0xf0) >> 4);
            return true;    /* pause: stop executing */
        }
        return false;
    case AIC_DFCNTRL: {
        uint8_t old = s->regs[AIC_DFCNTRL];

        s->regs[AIC_DFCNTRL] = val;
        if (val & AIC_FIFORESET) {
            aic7880_fifo_reset(s);
            s->hdma_done = false;
            s->regs[AIC_DFCNTRL] = val & ~AIC_FIFORESET;
        }
        if ((val & AIC_FIFOFLUSH) && (s->fifo_count || !s->hdma_done)) {
            s->hdma_done = false;
            aic7880_hdma_run(s);
            s->regs[AIC_DFCNTRL] = val & ~AIC_FIFOFLUSH;
        }
        if ((val & AIC_HDMAEN) && !(old & AIC_HDMAEN)) {
            /* rising host-DMA enable: run the transfer now (the chip
             * completes it asynchronously fast; completing inline
             * keeps the microcode's DFSTATUS polling loops short) */
            s->hdma_done = false;
            aic7880_hdma_run(s);
        }
        return false;
    }
    case AIC_SCSISIG:
        /* phase change from the microcode */
        s->regs[AIC_SCSISIG] = val;
        s->cur_phase = val & AIC_BUSPHASE;
        trace_sgi_aic7880_phase(s->scsi_bus_num, s->cur_phase,
                               s->regs[AIC_ACTIVE_SCB]);
        return false;
    case AIC_CLRSINT0:
        s->regs[AIC_SSTAT0] &= ~(val & 0x7f);
        return false;
    case AIC_CLRSINT1:
        s->regs[AIC_SSTAT1] &= ~(val & AIC_CLRSINT1_MASK);
        return false;
    case AIC_SCSISEQ:
        s->regs[AIC_SCSISEQ] = val;
        if (val & AIC_SCSIRSTO) {
            aic7880_clear_ua(s);
        }
        return false;
    case AIC_SCBPTR:
        s->regs[AIC_SCBPTR] = val;
        /*
         * When the microcode points SCBPTR at a fresh SCB (the idle
         * loop's QINFIFO pop path words 15-17), pull the 32-byte
         * sequencer SCB from host memory into the on-chip bank.  The
         * real chip DMAs it via the FIFO; fetching here keeps the
         * visible effect identical for every SINDIR/DINDIR access the
         * program then makes to 0xa0-0xbf.
         */
        if (val != 0xff && s->last_fetched_scb != val &&
            (aic_scratch_le32(s, AIC_SCB_PTR_ARRAY) & 0x3fffffffu)) {
            aic7880_fetch_scb(s, val);
            /*
             * Chip-hardware behavior the microcode depends on: when a
             * SCB is loaded (the QINFIFO pop path), the chip marks it
             * ACTIVE.  Words 75-84 restore SCBPTR from ACTIVE_SCB
             * (word 76) — without this latch the 6.5.5f pop of SCB 0
             * (word 16 jumps on SCBPTR==0, bypassing the software
             * ACTIVE_SCB save at word 17) leaves ACTIVE_SCB at the
             * driver's 0xff and the prep path loads a garbage bank.
             */
            s->regs[AIC_ACTIVE_SCB] = val;
            s->last_fetched_scb = val;
        }
        /*
         * Selection: ONLY on a genuine QINFIFO pop (the interpreter's
         * ALU word wrote SCBPTR with the value just read from QINFIFO).
         * On real silicon the chip selects the loaded SCB's target once
         * ENSELO is armed (idle word 5's constant 0x5a); in QEMU the
         * selection IS the request.  Restricting to the pop prevents
         * the word-76 SCBPTR restore (ACTIVE_SCB latch) from
         * re-selecting a completed SCB.
         */
        if (from_pop && val != 0xff) {
            if (!s->cur_req && !s->bus_selected) {
                aic7880_start_selection(s, val);
            } else {
                trace_sgi_aic7880_sel_drop(s->scsi_bus_num, val,
                                           !!s->cur_req, s->bus_selected);
            }
        }
        return false;
    case AIC_ALLZEROS:
        return false;    /* NONE: discard */
    default:
        if (chip >= AIC_SCB00 && chip < AIC_SCB00 + 32) {
            s->scb[s->regs[AIC_SCBPTR] & 0x7f][chip - AIC_SCB00] = val;
            return false;
        }
        s->regs[chip] = val;
        return false;
    }
}


/*
 * Execute the sequencer program.  Runs a bounded instruction budget,
 * stopping on pause (host PAUSE, INTSTAT SEQINT pause, or budget
 * exhaustion -> re-arm).  Returns when the chip is idle at the idle
 * loop with nothing to do (also re-arms).
 */
static void aic7880_seq_run(SGIAIC7880State *s)
{
    int budget = AIC7880_SEQ_BUDGET;

    if (!s->seq_running || s->seq_host_pause || s->seq_int_pause) {
        return;
    }

    while (budget-- > 0) {
        uint32_t pc = s->seq_pc;
        uint32_t off = (pc << 2) & (AIC7880_SEQRAM_SIZE - 1);
        uint8_t imm = s->seqram[off];
        uint8_t src = s->seqram[off + 1];
        uint8_t dst = s->seqram[off + 2];
        uint8_t op = s->seqram[off + 3];
        uint8_t sv, dv;
        bool pause = false;
        uint32_t next = pc + 1;

        /*
         * Bounded-rate fetch trace: 1 event per 64 fetches, and only
         * outside the idle loop (words 0-26: the trampoline table +
         * the poll body, identical in both Seq revisions) — the idle
         * loop spins ~10 instr/iteration forever and would flood the
         * trace file otherwise.
         */
        if (pc > 26 && (s->seq_fetch_count & 63) == 0) {
            trace_sgi_aic7880_seq_fetch(s->scsi_bus_num, pc, op);
        }
        /* M12a2 diagnostic: the idle loop's queue gate — 1-in-1024
         * sampling of words 8-15 so the pop decision point stays
         * visible without flooding (the idle loop is hot: an
         * unconditional word-14 event produced 58M lines in 45s). */
        if (pc >= 8 && pc <= 15 && (s->seq_fetch_count & 1023) == 0) {
            trace_sgi_aic7880_seq_idle_gate(s->scsi_bus_num, pc,
                aic7880_seq_reg_read(s, AIC_SSTAT0),
                aic7880_seq_reg_read(s, AIC_SCSISEQ),
                aic7880_seq_reg_read(s, AIC_QINCNT));
        }
        s->seq_fetch_count++;

        switch (op) {
        case SEQ_OP_MOV:
        case SEQ_OP_OR:
        case SEQ_OP_AND:
        case SEQ_OP_ANDN:
        case SEQ_OP_XOR:
        case SEQ_OP_ADD:
        case SEQ_OP_ADD2:
        case SEQ_OP_ADC:
        case SEQ_OP_ADD3: {
            bool carry_in = false;
            unsigned result;

            sv = aic7880_seq_reg_read(s, src);
            if (op == SEQ_OP_MOV) {
                /* MOV/MVI: dst = src | imm (MVI when src=ALLZEROS) */
                result = (unsigned)sv | imm;
            } else if (op == SEQ_OP_OR) {
                result = (unsigned)sv | imm;
            } else if (op == SEQ_OP_AND) {
                result = (unsigned)sv & imm;
            } else if (op == SEQ_OP_ANDN) {
                result = (unsigned)sv & (uint8_t)~imm;
            } else if (op == SEQ_OP_XOR) {
                result = (unsigned)sv ^ imm;
            } else {
                /* ADD family: ADD / ADC (carry-in) */
                if (op == SEQ_OP_ADC) {
                    carry_in = (s->seq_flags & AIC_FLAG_CARRY) != 0;
                }
                result = (unsigned)sv + imm + (carry_in ? 1 : 0);
            }
            /* flags */
            if (op >= SEQ_OP_ADD) {
                if (result > 0xff) {
                    s->seq_flags |= AIC_FLAG_CARRY;
                } else {
                    s->seq_flags &= ~AIC_FLAG_CARRY;
                }
            }
            dv = result & 0xff;
            if (dv == 0) {
                s->seq_flags |= AIC_FLAG_ZERO;
            } else {
                s->seq_flags &= ~AIC_FLAG_ZERO;
            }
            /* write-back (dst=ALLZEROS/NONE = non-destructive TEST) */
            if (dst != AIC_ALLZEROS) {
                pause = aic7880_seq_reg_write_ex(s, dst, dv,
                                                  src == AIC_QINFIFO);
            }
            break;
        }
        case SEQ_OP_JMP:
        case SEQ_OP_JMP2:
            next = dst;
            break;
        case SEQ_OP_CALL:
            if (s->seq_sp < AIC7880_STACK_DEPTH) {
                s->seq_stack[s->seq_sp++] = pc + 1;
            }
            next = dst;
            break;
        case SEQ_OP_JNZF:
            /*
             * 0x17 = JUMP if the SRC REGISTER reads NON-zero (imm is
             * a don't-care; the disassembler's flag-based reading was
             * wrong — settled by live trace + both-blob simulation).
             * Decisive evidence: word 170 'Jcc src=ALLZEROS' follows
             * the valid-SCB intake (word 21 'Jcc src=SCB00' jumps for
             * a fetched tarlun); reading ALLZEROS as zero must NOT
             * take the word-73 NO_ID_MSG pause for a valid SCB —
             * only a jump-if-NONzero polarity does that, and the
             * 'MVI DINDEX, const; Jcc' pairs decode as unconditional
             * sequencing (src=ALLZEROS reads 0 = never jump), with
             * the MVI pre-loading DINDEX for the following copy
             * loop. The idle loop's pop path: SCB 0 (adp78 numbers
             * its first SCB 0 — kmem_zalloc'd adapter, adp78.c:807)
             * falls through word 16 into the word-21 intake; nonzero
             * SCBs take the word-16 fast route to the data path.
             */
            sv = aic7880_seq_reg_read(s, src);
            if (sv != 0) {
                next = dst;
            }
            break;
        case SEQ_OP_JEQM:
            sv = aic7880_seq_reg_read(s, src);
            if ((sv & imm) == imm) {
                next = dst;
                if (pc > 26 && (s->seq_fetch_count & 7) == 0) {
                    trace_sgi_aic7880_seq_jump(s->scsi_bus_num, pc, dst);
                }
            }
            break;
        case SEQ_OP_JNEM:
        case SEQ_OP_JNEM2:
            sv = aic7880_seq_reg_read(s, src);
            if ((sv & imm) != imm) {
                next = dst;
            }
            break;
        case SEQ_OP_JNZ:
        case SEQ_OP_JNZ2:
            sv = aic7880_seq_reg_read(s, src);
            if ((sv & imm) != 0) {
                next = dst;
            }
            break;
        case SEQ_OP_JE:
            sv = aic7880_seq_reg_read(s, src);
            if (sv == imm) {
                next = dst;
            }
            break;
        case SEQ_OP_JNE:
            sv = aic7880_seq_reg_read(s, src);
            if (sv != imm) {
                next = dst;
            }
            break;
        case SEQ_OP_JZ:
            sv = aic7880_seq_reg_read(s, src);
            if ((sv & imm) == 0) {
                next = dst;
            }
            break;
        default:
            /* Illegal opcode: FAILDIS not set -> latch and pause */
            qemu_log_mask(LOG_UNIMP, "sgi_aic7880: illegal seq opcode "
                          "0x%02x @ pc %u\n", op, pc);
            s->intstat |= AIC_SEQINT;
            s->seq_int_pause = true;
            return;
        }

        /*
         * QINFIFO-as-source read = dequeue.  The real chip pops the
         * queue on every read; the idle loop's 'AND SCBPTR, QINFIFO'
         * (word 15) both pops the next SCB number AND tests it.
         * aic7880_seq_reg_read() implements the pop.
         */

        /* commit PC — a redirect set by a device-side action inside
         * the write hook (the SELTO path returns the sequencer to
         * the idle loop) wins over the natural successor, which
         * would otherwise resume the retired SCB's phase walk. */
        if (s->seq_redirect >= 0) {
            s->seq_pc = (uint32_t)s->seq_redirect &
                ((AIC7880_SEQRAM_SIZE / 4) - 1);
            s->seq_redirect = -1;
        } else {
            s->seq_pc = next & ((AIC7880_SEQRAM_SIZE / 4) - 1);
        }

        if (pause) {
            s->seq_int_pause = true;
            aic7880_update_irq(s);
            return;
        }

        /* Busy-wait escape valve: a jump-to-self (wait loop on
         * DFSTATUS/SSTAT1) would spin the budget; if we detect the PC
         * repeating with no state change possible, break out and
         * re-arm via the timer so the guest can observe progress. */
        if (next == pc && budget < AIC7880_SEQ_BUDGET - 8) {
            break;
        }
    }

    /*
     * Re-arm unless paused: wait loops (DFSTATUS HDONE spins) exit
     * via the self-jump valve and must retry after the async SCSI
     * completion lands; the idle loop polls at the same cadence the
     * real chip's sequencer clock would.  The kick paths (QINFIFO
     * write, CLRINT/CLRSINT, HCNTRL unpause, SEQADDR steer) call
     * seq_run directly for immediate response.
     */
    if (s->seq_running && !s->seq_host_pause && !s->seq_int_pause) {
        timer_mod(s->seq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000);
    }
}

/* Scheduler kick: the interpreter is cooperative; the timer re-arms
 * it whenever there is pending work (budget exhaustion or a wait
 * loop that external state must unblock). */
static void aic7880_seq_timer_cb(void *opaque)
{
    SGIAIC7880State *s = SGI_AIC7880(opaque);

    if (s->done_pending) {
        aic7880_post_complete(s);
        return;
    }
    aic7880_seq_run(s);
}

/* External kick: queue activity / interrupt clear / unpause */
static void aic7880_kick(SGIAIC7880State *s)
{
    if (s->seq_running && !s->seq_host_pause && !s->seq_int_pause) {
        aic7880_seq_run(s);
    }
}

/* =====================================================================
 * Host-visible register file (conv() decoded)
 */

static uint64_t aic7880_reg_read(SGIAIC7880State *s, uint32_t chip)
{
    switch (chip) {
    case AIC_VENDID0:
        return ADAPTEC_VENDOR_ID & 0xff;          /* 0x04 */
    case AIC_VENDID1:
        return (ADAPTEC_VENDOR_ID >> 8) & 0xff;    /* 0x90 */
    case AIC_DEVID0:
        return ADAPTEC_DEVICE_ID & 0xff;           /* 0x78 */
    case AIC_DEVID1:
        return (ADAPTEC_DEVICE_ID >> 8) & 0xff;    /* 0x80 */
    case AIC_HCNTRL: {
        uint8_t v = s->regs[AIC_HCNTRL];

        if (s->seq_host_pause || s->seq_int_pause) {
            v |= AIC_PAUSEACK;
        } else {
            v &= ~AIC_PAUSEACK;
        }
        return v;
    }
    case AIC_INTSTAT:
        trace_sgi_aic7880_intstat_read_ring(
            s->scsi_bus_num, s->intstat, s->qout_slot,
            aic7880_qout_lane(s, 4), aic7880_qout_lane(s, 0),
            aic7880_qout_lane(s, 252), aic7880_qout_lane(s, 248));
        return s->intstat;
    case AIC_ERROR:
        return 0x00;
    case AIC_DFSTATUS: {
        uint8_t v = aic7880_seq_reg_read(s, AIC_DFSTATUS);

        return v;
    }
    case AIC_QINCNT:
        return s->qin_count;
    case AIC_QOUTCNT:
        return 0x00;
    case AIC_SEQRAM: {
        uint32_t a = s->seqaddr & (AIC7880_SEQRAM_SIZE - 1);
        uint8_t v = s->seqram[a];

        s->seqaddr = (s->seqaddr + 1) & (AIC7880_SEQRAM_SIZE - 1);
        return v;
    }
    case AIC_QINFIFO:
        /* host read of QINFIFO: abort path rotates the queue — the
         * same pop the sequencer does */
        return aic7880_qinfifo_read(s);
    case AIC_SCSIBUSL:
        return s->regs[AIC_SCSIDATL];
    default:
        break;
    }

    if (chip >= AIC_SCB00 && chip < AIC_SCB00 + 32) {
        return s->scb[s->regs[AIC_SCBPTR] & 0x7f][chip - AIC_SCB00];
    }
    return s->regs[chip];
}

static void aic7880_chip_reset(SGIAIC7880State *s)
{
    /*
     * CHIPRESET clears the register file and on-chip SCB banks but
     * NOT the downloaded sequencer RAM — the real chip keeps SEQRAM
     * across a host reset (the driver reloads it only when
     * CFP_InitNeeded; adp_reset's reset dance relies on the image
     * persisting).
     */
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->scb, 0, sizeof(s->scb));
    s->qin_head = 0;
    s->qin_count = 0;
    s->intstat = 0;
    s->seq_pc = 0;
    s->seq_flags = 0;
    s->seq_sp = 0;
    s->seq_int_pause = false;
    s->seq_running = false;
    s->seq_host_pause = false;
    aic7880_fifo_reset(s);
    s->cur_scb = 0xff;
    s->cur_req = NULL;
    s->bus_selected = false;
    s->last_fetched_scb = 0xff;
    s->seq_redirect = -1;
    s->qout_slot = 4;
    s->qout_base = 0;    /* register file cleared: re-arm via MovPtrToScratch */
    aic7880_update_irq(s);
}

static void aic7880_reg_write(SGIAIC7880State *s, uint32_t chip, uint8_t val)
{
    switch (chip) {
    case AIC_HCNTRL:
        trace_sgi_aic7880_hcntrl_write(s->scsi_bus_num, val,
                                       s->seq_running, s->seq_host_pause,
                                       s->seq_int_pause);
        s->regs[AIC_HCNTRL] = val;
        if (val & AIC_CHIPRESET) {
            aic7880_chip_reset(s);
        }
        /* host PAUSE request drives the interpreter's host-pause */
        s->seq_host_pause = (val & AIC_PAUSE) != 0;
        aic7880_update_irq(s);
        if (!s->seq_host_pause) {
            aic7880_kick(s);
        }
        break;
    case AIC_INTSTAT:
        /* read-only */
        break;
    case AIC_CLRINT:
        trace_sgi_aic7880_clrint(s->scsi_bus_num, val, s->intstat,
                                 s->qout_slot);
        if (val & AIC_CLRSEQINT) {
            s->intstat &= ~(AIC_SEQINT | AIC_INTCODE);
            s->seq_int_pause = false;   /* resume after the pause word */
        }
        s->intstat &= ~(val & 0x0e);
        aic7880_update_irq(s);
        aic7880_kick(s);
        break;
    case AIC_CLRSINT0:
        s->regs[AIC_SSTAT0] &= ~(val & 0x7f);
        aic7880_kick(s);
        break;
    case AIC_CLRSINT1:
        s->regs[AIC_SSTAT1] &= ~(val & AIC_CLRSINT1_MASK);
        aic7880_kick(s);
        break;
    case AIC_SCSISEQ:
        s->regs[AIC_SCSISEQ] = val;
        if (val & AIC_SCSIRSTO) {
            /* driver-initiated SCSI bus reset: quiet the QEMU bus */
            aic7880_clear_ua(s);
        }
        aic7880_kick(s);
        break;
    case AIC_SEQCTL:
        s->regs[AIC_SEQCTL] = val;
        if (val & AIC_SEQRESET) {
            s->seq_pc = 0;
        }
        /*
         * LOADRAM clear = sequencer may run.  Do not auto-kick: the
         * load/verify path leaves SEQADDR at the image tail; the
         * driver steers SEQADDR to an entry (or unpauses) before it
         * expects execution.
         */
        s->seq_running = (val & AIC_LOADRAM) == 0;
        break;
    case AIC_SEQRAM:
        s->seqram[s->seqaddr & (AIC7880_SEQRAM_SIZE - 1)] = val;
        s->seqaddr = (s->seqaddr + 1) & (AIC7880_SEQRAM_SIZE - 1);
        break;
    case AIC_SEQADDR0:
        s->regs[AIC_SEQADDR0] = val;
        s->seqaddr = ((s->regs[AIC_SEQADDR1] << 8) | val) &
            (AIC7880_SEQRAM_SIZE - 1);
        /*
         * The driver sets SEQADDR to redirect execution (resume-after-
         * pause, restart at an entry).  If the sequencer is running
         * and not loading RAM, this write steers the interpreter PC.
         */
        s->seq_pc = (s->seqaddr >> 2) & ((AIC7880_SEQRAM_SIZE / 4) - 1);
        break;
    case AIC_SEQADDR1:
        s->regs[AIC_SEQADDR1] = val;
        s->seqaddr = ((val << 8) | s->regs[AIC_SEQADDR0]) &
            (AIC7880_SEQRAM_SIZE - 1);
        s->seq_pc = (s->seqaddr >> 2) & ((AIC7880_SEQRAM_SIZE / 4) - 1);
        aic7880_kick(s);
        break;
    case AIC_QINFIFO:
        /*
         * host -> sequencer command queue.  0xff (INVALID_SCB_INDEX)
         * entries are queue-rotation poison (the driver's abort path
         * re-pushes what it read; a race-pop of an empty FIFO reads
         * 0xff and would push it back) — the sequencer would pop it
         * as an SCB and pause NO_ID_MSG.  Drop them.
         */
        if (val != 0xff && s->qin_count < 256) {
            uint16_t tail = (s->qin_head + s->qin_count) & 0xff;
            s->qin[tail] = val;
            s->qin_count++;
        }
        trace_sgi_aic7880_qin_write(s->scsi_bus_num, val, s->qin_count);
        aic7880_kick(s);
        break;
    case AIC_QINCNT:
    case AIC_QOUTCNT:
        /* read-only */
        break;
    default:
        if (chip >= AIC_SCB00 && chip < AIC_SCB00 + 32) {
            s->scb[s->regs[AIC_SCBPTR] & 0x7f][chip - AIC_SCB00] = val;
            return;
        }
        if (chip >= AIC_QOUT_PTR_ARRAY && chip <= AIC_QOUT_PTR_ARRAY + 3) {
            trace_sgi_aic7880_qout_scratch_write(s->scsi_bus_num, chip,
                                                 val, s->qout_slot);
        }
        if (chip == AIC_QOUT_PTR_ARRAY + 3) {
            /*
             * QOUT pointer (re)programmed by Ph_MovPtrToScratch (the
             * last byte of the MIPS_BE dance lands here): latch the
             * ring base for the completion path's host writes and
             * resync the r56 slot mirror to the driver's qout_index
             * order (Ph_set_scratchmem resets it to 4, descending
             * by 4).  Note the LSB lane of a 256-aligned pointer is
             * 0 — the driver's own array address is 256-aligned
             * (himd.c:467) — so the byte here (0x40 = the MSB) is
             * NOT the base; reassemble the dword after the store.
             */
            s->regs[chip] = val;
            s->qout_base = aic_scratch_le32(s, AIC_QOUT_PTR_ARRAY) &
                0x3fffffffu;
            s->qout_slot = 4;
            s->regs[AIC_QOUT_PTR_ARRAY] = s->qout_slot;
            return;
        }
        s->regs[chip] = val;
        break;
    }
}

/* =====================================================================
 * BAR MMIO (bus offsets -> conv() -> chip registers)
 */

static uint64_t aic7880_mmio_read(void *opaque, hwaddr offset, unsigned size)
{
    SGIAIC7880State *s = SGI_AIC7880(opaque);
    uint32_t chip;

    if (offset >= 256) {
        qemu_log_mask(LOG_UNIMP, "sgi_aic7880: read past reg file 0x%03"
                      HWADDR_PRIx "\n", offset);
        return 0xff;
    }
    chip = aic_conv(offset);
    AIC_DPRINTF("read bus 0x%03x chip 0x%02x\n", (uint32_t)offset, chip);
    return aic7880_reg_read(s, chip);
}

static void aic7880_mmio_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    SGIAIC7880State *s = SGI_AIC7880(opaque);
    uint32_t chip;

    if (offset >= 256) {
        qemu_log_mask(LOG_UNIMP, "sgi_aic7880: write past reg file 0x%03"
                      HWADDR_PRIx "\n", offset);
        return;
    }
    chip = aic_conv(offset);
    aic7880_reg_write(s, chip, value & 0xff);
}

static const MemoryRegionOps aic7880_ops = {
    .read = aic7880_mmio_read,
    .write = aic7880_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/* =====================================================================
 * PCI device
 */

static void aic7880_realize(PCIDevice *pci_dev, Error **errp)
{
    SGIAIC7880State *s = SGI_AIC7880(pci_dev);

    /*
     * BAR0: 256-byte register window — the PROM's AIC driver maps the
     * whole register file (SCSI/sequencer block through the host
     * block at 0x80-0xBF: HCNTRL, INTSTAT, QINFIFO, SCB array)
     * through BAR0, while the IRIX adp78 driver maps BAR1
     * (PCIIO_SPACE_WIN(1), 1 KB).  The kernel's BAR-sizing scan stops
     * at the first dead BAR, so BAR0 must exist for BAR1 to be
     * programmed either way.
     */
    memory_region_init_io(&s->mmio, OBJECT(s), &aic7880_ops, s,
                          "sgi-aic7880", 1024);
    memory_region_init_alias(&s->bar0, OBJECT(s), "sgi-aic7880-bar0",
                             &s->mmio, 0, 256);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);

    pci_config_set_interrupt_pin(pci_dev->config, 1);  /* INTA */

    scsi_bus_init(&s->bus, sizeof(s->bus), DEVICE(pci_dev),
                  &aic7880_scsi_info);
    /*
     * Legacy -drive if=scsi,bus=N,unit=M attaches on the bus whose
     * busnr we set here (controller 0 = bus 0, controller 1 = bus 1).
     */
    s->bus.busnr = s->scsi_bus_num;
    scsi_bus_legacy_handle_cmdline(&s->bus);
    /*
     * SGI install CDs are 512-byte-sector EFS images, not 2048-byte
     * ISO9660 media.  The IRIX dksc driver addresses every partition
     * in 512-byte basic blocks — dkscstart() stuffs the absolute
     * 512-bb number straight into the READ CDB's LBA field (confirmed
     * by disassembling the 6.5.5f miniroot kernel's dkscstart), and
     * dk_getblksz() ends up with the 512 fallback because QEMU's
     * scsi-cd mode-sense reply carries no block descriptor.  A
     * 2048-block medium is therefore unaddressable: every read lands
     * ~4x past its target (the EFS superblock read went to LBA 47777
     * = byte 47777*2048 instead of the superblock at byte 47777*512),
     * and each 1-block CDB transfers 4x the driver's buffer.  Present
     * CD-ROM targets with 512-byte sectors — the same doctrine as the
     * wd33c93/virtuix path (see scsi_bus_force_cd_sector_size).
     * (Authentic: SGI's drives really were 512-byte-block devices —
     * do NOT "fix" until 12d decides the MODE_SENSE policy.)
     */
    scsi_bus_force_cd_sector_size(&s->bus, 512);

    s->seq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic7880_seq_timer_cb, s);
}

static void aic7880_reset(DeviceState *dev)
{
    SGIAIC7880State *s = SGI_AIC7880(dev);

    memset(s->seqram, 0, sizeof(s->seqram));
    s->seqaddr = 0;
    s->cur_scb = 0xff;
    s->cur_req = NULL;
    s->last_fetched_scb = 0xff;
    s->seq_redirect = -1;
    s->qout_slot = 4;
    s->qout_base = 0;
    aic7880_chip_reset(s);
}

static void aic7880_exit(PCIDevice *pci_dev)
{
    SGIAIC7880State *s = SGI_AIC7880(pci_dev);

    timer_del(s->seq_timer);
    if (s->cur_req) {
        scsi_req_cancel(s->cur_req);
    }
}

static const VMStateDescription vmstate_sgi_aic7880 = {
    .name = "sgi-aic7880",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, SGIAIC7880State),
        VMSTATE_UINT8_ARRAY(regs, SGIAIC7880State, 256),
        VMSTATE_UINT8_ARRAY(seqram, SGIAIC7880State, AIC7880_SEQRAM_SIZE),
        VMSTATE_UINT32(seqaddr, SGIAIC7880State),
        VMSTATE_UINT8_2DARRAY(scb, SGIAIC7880State, AIC7880_NUM_SCB_BANKS,
                              AIC7880_SCB_BANK_SIZE),
        VMSTATE_UINT16(qin_head, SGIAIC7880State),
        VMSTATE_UINT16(qin_count, SGIAIC7880State),
        VMSTATE_UINT8_ARRAY(qin, SGIAIC7880State, 256),
        VMSTATE_UINT8(intstat, SGIAIC7880State),
        VMSTATE_UINT32(seq_pc, SGIAIC7880State),
        VMSTATE_BOOL(seq_running, SGIAIC7880State),
        VMSTATE_BOOL(seq_host_pause, SGIAIC7880State),
        VMSTATE_BOOL(seq_int_pause, SGIAIC7880State),
        VMSTATE_UINT8(seq_flags, SGIAIC7880State),
        VMSTATE_UINT16_ARRAY(seq_stack, SGIAIC7880State, AIC7880_STACK_DEPTH),
        VMSTATE_UINT8(seq_sp, SGIAIC7880State),
        VMSTATE_UINT16(fifo_head, SGIAIC7880State),
        VMSTATE_UINT16(fifo_tail, SGIAIC7880State),
        VMSTATE_UINT16(fifo_count, SGIAIC7880State),
        VMSTATE_UINT8_ARRAY(fifo, SGIAIC7880State, AIC7880_FIFO_SIZE),
        VMSTATE_BOOL(hdma_done, SGIAIC7880State),
        VMSTATE_UINT8(cur_phase, SGIAIC7880State),
        VMSTATE_BOOL(bus_selected, SGIAIC7880State),
        VMSTATE_INT32(cur_scb, SGIAIC7880State),
        VMSTATE_UINT32(sg_list, SGIAIC7880State),
        VMSTATE_UINT8(sg_count, SGIAIC7880State),
        VMSTATE_UINT8(sg_index, SGIAIC7880State),
        VMSTATE_UINT32(sg_elem_addr, SGIAIC7880State),
        VMSTATE_UINT32(sg_elem_left, SGIAIC7880State),
        VMSTATE_UINT32(seq_fetch_count, SGIAIC7880State),
    VMSTATE_UINT8(last_fetched_scb, SGIAIC7880State),
    VMSTATE_UINT8(qout_slot, SGIAIC7880State),
    VMSTATE_UINT32(qout_base, SGIAIC7880State),
    VMSTATE_INT32(seq_redirect, SGIAIC7880State),
    VMSTATE_BOOL(done_pending, SGIAIC7880State),
    VMSTATE_INT32(done_status, SGIAIC7880State),
    VMSTATE_INT32(done_scb, SGIAIC7880State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property aic7880_properties[] = {
    DEFINE_PROP_UINT32("scsi-bus-num", SGIAIC7880State, scsi_bus_num, 0),
};

static void aic7880_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = aic7880_realize;
    k->exit = aic7880_exit;
    k->vendor_id = ADAPTEC_VENDOR_ID;
    k->device_id = ADAPTEC_DEVICE_ID;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_STORAGE_SCSI;
    dc->desc = "SGI O2 onboard AIC-7880 SCSI controller";
    device_class_set_legacy_reset(dc, aic7880_reset);
    dc->vmsd = &vmstate_sgi_aic7880;
    device_class_set_props(dc, aic7880_properties);
}

static const TypeInfo aic7880_info = {
    .name = TYPE_SGI_AIC7880,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(SGIAIC7880State),
    .class_init = aic7880_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void aic7880_register_types(void)
{
    type_register_static(&aic7880_info);
}

type_init(aic7880_register_types)
