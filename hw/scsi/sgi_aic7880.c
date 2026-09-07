/*
 * SGI O2 onboard AIC-7880 PCI SCSI controller — works-in-place
 * placeholder.  See include/hw/scsi/sgi_aic7880.h for the scope
 * statement and progress_notes/o2_qemu/03-mace-pci-scsi.md for the
 * full contract tables (spec / kernel driver / this device).
 *
 * Register model: the AIC-7880 is a little-endian PCI device with a
 * 256-byte register file.  The IRIX driver (MIPS_BE build) reaches
 * register N at bus offset conv(N) = (N & ~3) | (3 - (N & 3)) — the
 * MACE presents the big-endian SGI bus to the CPU and the driver
 * pre-swaps every byte offset (him_equ.h MIPS_BE values, osm.c
 * osm_conv_byteptr).  This device applies the same conversion on
 * every BAR access and implements the register file at chip offsets.
 *
 * The sequencer SCRIPTS the driver downloads (Seq_01, OPTIMA mode)
 * are stored and verified but never executed.  Instead the device
 * emulates what the driver can observe: QINFIFO consumption, the
 * host-memory SCB fetch/DMA, SCSI execution against the QEMU SCSI
 * bus, and completion via the host QOUT_PTR_ARRAY + INTSTAT.
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
#define AIC_SCSIDATH    0x07    /* scsi latched data, hi       (r/w)  */
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

/* SSTAT1 bits */
#define AIC_SELTO       0x80    /* selection timeout                  */
#define AIC_SCSIRSTI    0x20    /* scsi reset in                       */
#define AIC_BUSFREE     0x08    /* bus free detected                  */
#define AIC_SCSIPERR    0x04    /* scsi parity error                   */
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
#define AIC_DINDEX       0x65    /* destination index           (r/w)  */
#define AIC_SINDEX       0x66    /* source index                (r/w)  */
#define AIC_ACCUM        0x67    /* accumulator                  (r/w)  */
#define AIC_ALLZEROS     0x68    /* reads 0x00                          */
#define AIC_ALLONES      0x69    /* reads 0xff                          */
#define AIC_BRKADDR1     0x6b    /* break address, hi           (r/w)  */
#define AIC_SINDIR       0x6c    /* source index, indirect      (r/w)  */
#define AIC_DINDIR       0x6d    /* destination index, indirect (r/w)  */
#define AIC_FUNCTION1    0x6e    /* function bits 6-4           (r/w)  */
#define AIC_STACK        0x6f    /* subroutine stack            (r)    */

/* SEQCTL bits */
#define AIC_PERRORDIS   0x80    /* parity error disable                */
#define AIC_FAILDIS     0x20    /* illegal opcode/address int disable  */
#define AIC_FASTMODE    0x10    /* sequencer clock select              */
#define AIC_BRKINTEN    0x08    /* breakpoint interrupt enable         */
#define AIC_SEQRESET    0x02    /* clear sequencer program counter     */
#define AIC_LOADRAM     0x01    /* sequencer ram loading mode           */
#define AIC_BRKDIS      0x80    /* BRKADDR1: breakpoint disable        */

/* Host/PCI block */
#define AIC_VENDID0     0x80    /* PCI vendor id byte 0 (r/o: 0x04)   */
#define AIC_VENDID1     0x81    /*                 byte 1 (0x90)      */
#define AIC_DEVID0      0x82    /* PCI device id byte 0 (0x78)         */
#define AIC_DEVID1      0x83    /*                byte 1 (0x80)       */
#define AIC_COMMAND     0x84    /* PCI command shadow          (r/w)  */
#define AIC_LATTIME     0x85    /* latency timer               (r/w)  */
#define AIC_PCISTATUS   0x86    /* PCI status shadow           (r/w)  */
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
#define AIC_HCNT0        0x8c    /* host DMA count 0..2         (r/w)  */

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

/* INTCODE values (him_equ.h) */
#define AIC_INTCHECK_CONDX    0x50    /* check condition from target     */

/* CLRINT bits */
#define AIC_CLRSEQINT  0x01
#define AIC_CLRCMDINT  0x02
#define AIC_CLRSCSINT  0x04
#define AIC_CLRBRKINT  0x08

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

/* Fake sequencer timing: one SCB per round, ~command overhead */
#define AIC_SEQ_DELAY_NS    200000ULL

/* PCI IDs (adp78 registers 0x9004:0x8078 and 0x8178) */
#define ADAPTEC_VENDOR_ID   0x9004
#define ADAPTEC_DEVICE_ID   0x8078

/* Verbose debug logging — set to 1 to trace the fake sequencer. */
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

/* INTA level: any pending INTSTAT event (or SWINT) gated by INTEN */
static void aic7880_update_irq(SGIAIC7880State *s)
{
    int level = (s->regs[AIC_HCNTRL] & AIC_INTEN) &&
        ((s->intstat & 0x0f) || (s->regs[AIC_HCNTRL] & AIC_SWINT));

    pci_set_irq(PCI_DEVICE(s), level);
}

/*
 * Arm the fake sequencer for the next queued SCB.  It runs when the
 * sequencer is unpaused and the previous completion has been acked
 * (the driver clears INTSTAT via CLRINT), mirroring the real chip's
 * one-SCB-at-a-time flow that the single-entry WAITING_SCB /
 * ACTIVE_SCB / PASS_TO_DRIVER registers require.
 */
static void aic7880_kick(SGIAIC7880State *s)
{
    if (s->qin_count == 0 || s->cur_req || (s->intstat & 0x0f) ||
        (s->regs[AIC_HCNTRL] & AIC_PAUSE) || timer_pending(s->seq_timer)) {
        return;
    }
    timer_mod(s->seq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              AIC_SEQ_DELAY_NS);
}

/* =====================================================================
 * Fake sequencer: SCB execution against the QEMU SCSI bus
 */

/*
 * Post a GOOD completion for scb_num: the SCB number into the host
 * QOUT_PTR_ARRAY slot the driver reads next (qout_index order: 4, 0,
 * 252, ..., descending by 4 — himdopt.c Ph_checkqout), then raise
 * CMDCMPLT.
 */
static void aic7880_complete_good(SGIAIC7880State *s, uint8_t scb_num)
{
    uint32_t qout = aic_scratch_le32(s, AIC_QOUT_PTR_ARRAY) & 0x3fffffffu;

    if (qout) {
        aic_host_st8(s, qout + aic_conv(s->qout_slot), scb_num);
        /*
         * Pre-invalidate the NEXT slot (the one the driver reads
         * after processing this completion).  The ring is 64
         * entries; without this, a stale value from the previous
         * cycle would be re-processed as a phantom completion
         * when the driver polls QOUT between our writes (e.g. on
         * a selection-timeout interrupt), desyncing qout_index.
         */
        aic_host_st8(s, qout + aic_conv((uint8_t)(s->qout_slot - 4)),
                     0xFF);
        s->qout_slot -= 4;  /* wraps like the driver's qout_index */
    }
    s->intstat |= AIC_CMDCMPLT;
    AIC_DPRINTF("complete scb %d GOOD (qout@0x%x slot %d)\n", scb_num,
                qout, s->qout_slot + 4);
    aic7880_update_irq(s);
    aic7880_kick(s);
}

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
}

/* SCSIBusInfo: command done */
static void aic7880_command_complete(SCSIRequest *req, size_t residual)
{
    SGIAIC7880State *s = req->hba_private;
    int status = req->status;
    uint8_t scb_num;

    if (s->cur_req != req) {
        return;
    }
    scb_num = s->cur_scb;
    s->cur_req = NULL;
    scsi_req_unref(req);

    if (status == GOOD) {
        aic7880_complete_good(s, scb_num);
    } else {
        /* CHECK_CONDITION etc: status byte to the driver + SEQINT */
        s->regs[AIC_PASS_TO_DRIVER] = status;
        s->regs[AIC_ACTIVE_SCB] = scb_num;
        s->intstat |= AIC_SEQINT | AIC_INTCHECK_CONDX;
        AIC_DPRINTF("complete scb %d status 0x%02x (CHECK_CONDX)\n",
                    scb_num, status);
        aic7880_update_irq(s);
        aic7880_kick(s);
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
 * flow asserts SCSI reset from the driver before probing (adp_reset),
 * after which IRIX expects a quiet bus.  Called on SCSIRSTO.
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

/*
 * MODE SENSE(6) page 0 (vendor-specific) for CD-ROM targets — synthesized.
 *
 * QEMU's scsi-cd rejects this page: scsi-disk.c's mode_sense_valid[]
 * lists MODE_PAGE_VENDOR_SPECIFIC for TYPE_ROM, but mode_sense_page()'s
 * switch only implements it for TYPE_DISK, so the -1 falls through to
 * the "illegal_request" label -> CHECK CONDITION 0x24 "Illegal field
 * in CDB" (observed live: "scsi-disk: MODE_SENSE unsupported page
 * 0x00 ... dev_type=5").  IRIX queries exactly this — mount(1)'s
 * DIOCSENSE drive-parameters read (page 0, alloc 12 = mode-sense
 * header + one block descriptor) — and dksc aborts the open path on
 * the resulting alert.
 *
 * Serve what a real SCSI-2 CD-ROM answers: the 4-byte header plus
 * (when DBD=0) one 8-byte block descriptor carrying the block count
 * and block length.  The write-protect bit (SW) is set — the medium
 * is a CD — so mount(1) takes the read-only path and the EFS mount
 * skips its read-write superblock write-back (efs_mountfs only calls
 * efs_sbupdate when !(vfs_flag & VFS_RDONLY)).
 */
static void aic7880_mode_sense_page0_synth(SGIAIC7880State *s, uint8_t scb_num,
                                           SCSIDevice *sdev,
                                           const uint8_t *cdb,
                                           uint8_t tarlun)
{
    uint8_t resp[12];
    uint64_t nblocks = sdev->max_lba + 1;
    uint8_t page_control = (cdb[2] >> 6) & 0x3;
    bool dbd = (cdb[1] & 0x8) != 0;
    uint32_t len;

    memset(resp, 0, sizeof(resp));
    resp[2] = 0x80;                        /* SW: write-protected */
    if (dbd) {
        /* header + empty vendor-specific page (no block descriptor) */
        resp[0] = 5;                       /* mode data length */
        resp[4] = MODE_PAGE_VENDOR_SPECIFIC;
        resp[5] = 0;                       /* page length */
        len = 6;
    } else {
        /* header + one block descriptor, what alloc-length 12 asks for */
        if (nblocks > 0xffffff) {
            nblocks = 0;                    /* like QEMU's disk path */
        }
        resp[0] = 11;                       /* mode data length */
        resp[3] = 8;                        /* block descriptor length */
        resp[4] = 0;                        /* density code */
        resp[5] = (nblocks >> 16) & 0xff;
        resp[6] = (nblocks >> 8) & 0xff;
        resp[7] = nblocks & 0xff;
        resp[9] = (sdev->blocksize >> 16) & 0xff;
        resp[10] = (sdev->blocksize >> 8) & 0xff;
        resp[11] = sdev->blocksize & 0xff;
        len = 12;
    }
    len = MIN(len, cdb[4]);                 /* allocation length */
    aic7880_sg_move(s, resp, len, true);
    trace_sgi_aic7880_mode_sense_page0_synth(s->scsi_bus_num, tarlun >> 4,
                                             tarlun & 0xf, dbd, page_control,
                                             cdb[4], sdev->blocksize, nblocks);
    AIC_DPRINTF("synth mode sense page 0 scb %d: targ %d len %u blksz %d\n",
                scb_num, tarlun >> 4, len, sdev->blocksize);
    aic7880_complete_good(s, scb_num);
}

/* Execute one SCB fetched from host memory (the "sequencer run") */
static void aic7880_process_scb(SGIAIC7880State *s, uint8_t scb_num)
{
    uint32_t arr, entry, scb_host, d[3];
    uint32_t tarlun, cdb_ptr, cdb_len, seg_cnt, seg_ptr;
    uint8_t cdb[16];
    SCSIDevice *sdev;
    int32_t datalen;
    int i;

    /* Host SCB pointer array (scratch dword, driver wrote | 0x40000000) */
    arr = aic_scratch_le32(s, AIC_SCB_PTR_ARRAY) & 0x3fffffffu;
    if (!arr) {
        AIC_DPRINTF("process scb %d: no SCB_PTR_ARRAY yet\n", scb_num);
        return;
    }
    entry = aic_host_ldl_be(s, arr + 4u * scb_num);
    scb_host = entry & 0x3fffffffu;

#if DEBUG_SGI_AIC7880
    {
        int j;
        fprintf(stderr, "AIC7880[%02d]: scb entry 0x%08x host 0x%08x:",
                s->scsi_bus_num, entry, scb_host);
        for (j = 0; j < 32; j++) {
            fprintf(stderr, " %02x", aic_host_ldub(s, scb_host + j));
        }
        fprintf(stderr, "\n");
    }
#endif
    for (i = 0; i < 3; i++) {
        d[i] = aic_host_ldl_be(s, scb_host + 4u * i);
    }
    /*
     * dword0 is the driver's bitfield word: MIPSpro allocates bitfields
     * LSB-first, so the BE dword in memory is
     *   [31:24] SegCnt, [23:16] CDBLen, [15:8] tag/discon flags,
     *   [7:0] Tarlun (target<<4 | lun).
     * SegPtr (dword1) and CDBPtr (dword2) are plain BE dwords.
     */
    tarlun = d[AIC_SCB_DWORD_TARLUN] & 0xff;
    cdb_len = (d[AIC_SCB_DWORD_TARLUN] >> 16) & 0xff;
    seg_cnt = d[AIC_SCB_DWORD_TARLUN] >> 24;
    seg_ptr = d[AIC_SCB_DWORD_SEG] & 0x3fffffffu;
    cdb_ptr = d[AIC_SCB_DWORD_CDB] & 0x3fffffffu;

    if (cdb_len == 0 || cdb_len > sizeof(cdb)) {
        cdb_len = sizeof(cdb);
    }
    for (i = 0; i < (int)cdb_len; i++) {
        cdb[i] = aic_host_ldub(s, cdb_ptr + i);
    }

    AIC_DPRINTF("process scb %d: targ %d lun %d cdb_len %d seg_cnt %d "
                "cdb[0]=0x%02x scb@0x%x seg@0x%x\n", scb_num, tarlun >> 4,
                tarlun & 0xf, cdb_len, seg_cnt, cdb[0], scb_host, seg_ptr);

    s->cur_scb = scb_num;
    s->sg_list = seg_ptr;
    s->sg_count = seg_cnt ? seg_cnt : 1;
    s->sg_index = 0;
    s->sg_elem_left = 0;

    sdev = scsi_device_find(&s->bus, 0, tarlun >> 4, tarlun & 0xf);
    if (!sdev) {
        /* Selection timeout: SELTO + WAITING_SCB + SCSIINT */
        AIC_DPRINTF("process scb %d: SELTO target %d\n", scb_num,
                    tarlun >> 4);
        s->regs[AIC_WAITING_SCB] = scb_num;
        s->regs[AIC_SSTAT1] |= AIC_SELTO;
        s->intstat |= AIC_SCSIINT;
        aic7880_update_irq(s);
        return;
    }

    /*
     * MODE SENSE(6) page 0 to a CD-ROM: QEMU's scsi-cd cannot serve
     * it (see aic7880_mode_sense_page0_synth).  Intercept before the
     * request reaches the SCSI bus.  Saved values (page_control 3)
     * still fall through to QEMU, which answers "saving parameters
     * not supported" like a real drive.
     */
    if (cdb[0] == MODE_SENSE && (cdb[2] & 0x3f) == MODE_PAGE_VENDOR_SPECIFIC
        && sdev->type == TYPE_ROM && ((cdb[2] >> 6) & 0x3) != 3) {
        aic7880_mode_sense_page0_synth(s, scb_num, sdev, cdb, tarlun);
        return;
    }

    s->cur_req = scsi_req_new(sdev, 0, tarlun & 0xf, cdb, cdb_len, s);
    datalen = scsi_req_enqueue(s->cur_req);
    if (datalen != 0) {
        /* kick the data phase (emulated commands return it inline) */
        scsi_req_continue(s->cur_req);
    }
    /*
     * scsi-hd may have already completed (or chunked data) inline;
     * anything async completes via .complete from the block layer.
     */
}

static void aic7880_seq_timer_cb(void *opaque)
{
    SGIAIC7880State *s = SGI_AIC7880(opaque);
    uint8_t scb_num;

    if (s->cur_req || (s->regs[AIC_HCNTRL] & AIC_PAUSE) ||
        (s->intstat & 0x0f)) {
        /* in flight, paused, or previous completion unacked */
        return;
    }
    if (s->qin_count == 0) {
        return;
    }
    scb_num = s->qin[s->qin_head];
    s->qin_head = (s->qin_head + 1) & 0xff;
    s->qin_count--;
    aic7880_process_scb(s, scb_num);
}

/* =====================================================================
 * Register file
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
    case AIC_ALLZEROS:
        return 0x00;
    case AIC_ALLONES:
        return 0xff;
    case AIC_INTSTAT:
        return s->intstat;
    case AIC_ERROR:
        return 0x00;
    case AIC_DFSTATUS:
        /* idle: fifo empty, host done, no master request pending */
        return AIC_FIFOEMP | AIC_HDONE | AIC_DFTHRSH | AIC_MREQPEND;
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
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->scb, 0, sizeof(s->scb));
    s->qin_head = 0;
    s->qin_count = 0;
    s->intstat = 0;
    aic7880_update_irq(s);
}

static void aic7880_reg_write(SGIAIC7880State *s, uint32_t chip, uint8_t val)
{
    switch (chip) {
    case AIC_HCNTRL:
        s->regs[AIC_HCNTRL] = val;
        if (val & AIC_CHIPRESET) {
            aic7880_chip_reset(s);
        }
        aic7880_update_irq(s);
        aic7880_kick(s);
        break;
    case AIC_INTSTAT:
        /* read-only */
        break;
    case AIC_CLRINT:
        if (val & AIC_CLRSEQINT) {
            s->intstat &= ~(AIC_SEQINT | AIC_INTCODE);
        }
        s->intstat &= ~(val & 0x0e);
        AIC_DPRINTF("clrint 0x%02x -> intstat 0x%02x\n", val, s->intstat);
        aic7880_update_irq(s);
        aic7880_kick(s);
        break;
    case AIC_CLRSINT0:
        s->regs[AIC_SSTAT0] &= ~(val & 0x7f);
        break;
    case AIC_CLRSINT1:
        s->regs[AIC_SSTAT1] &= ~(val & AIC_CLRSINT1_MASK);
        break;
    case AIC_SCSISEQ:
        s->regs[AIC_SCSISEQ] = val;
        if (val & AIC_SCSIRSTO) {
            /* driver-initiated SCSI bus reset: quiet the QEMU bus */
            aic7880_clear_ua(s);
        }
        break;
    case AIC_SEQRAM:
        s->seqram[s->seqaddr & (AIC7880_SEQRAM_SIZE - 1)] = val;
        s->seqaddr = (s->seqaddr + 1) & (AIC7880_SEQRAM_SIZE - 1);
        break;
    case AIC_SEQADDR0:
        s->regs[AIC_SEQADDR0] = val;
        s->seqaddr = ((s->regs[AIC_SEQADDR1] << 8) | val) &
            (AIC7880_SEQRAM_SIZE - 1);
        break;
    case AIC_SEQADDR1:
        s->regs[AIC_SEQADDR1] = val;
        s->seqaddr = ((val << 8) | s->regs[AIC_SEQADDR0]) &
            (AIC7880_SEQRAM_SIZE - 1);
        break;
    case AIC_QINFIFO:
        /* host -> sequencer command queue (what the real chip does) */
        if (s->qin_count < 256) {
            uint16_t tail = (s->qin_head + s->qin_count) & 0xff;
            s->qin[tail] = val;
            s->qin_count++;
        }
        AIC_DPRINTF("qinfifo <- scb %d (count %d)\n", val, s->qin_count);
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
        if (chip == AIC_SCB_PTR_ARRAY + 3) {
            /*
             * Ph_MovPtrToScratch finishes a pointer dword with its
             * MSB; nothing to do (pointers are read lazily), kept
             * as a documented hook.
             */
        }
        if (chip == AIC_QOUT_PTR_ARRAY + 3) {
            /* QOUT pointer (re)programmed: resync the slot order */
            s->qout_slot = 4;
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
    AIC_DPRINTF("write bus 0x%03x chip 0x%02x <- 0x%02x\n",
                (uint32_t)offset, chip, (int)value & 0xff);
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
     */
    scsi_bus_force_cd_sector_size(&s->bus, 512);

    s->seq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic7880_seq_timer_cb, s);
}

static void aic7880_reset(DeviceState *dev)
{
    SGIAIC7880State *s = SGI_AIC7880(dev);

    memset(s->seqram, 0, sizeof(s->seqram));
    s->seqaddr = 0;
    s->qout_slot = 4;
    s->cur_scb = 0xff;
    s->cur_req = NULL;
    s->scb_sram = 0;
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
    .version_id = 1,
    .minimum_version_id = 1,
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
        VMSTATE_UINT8(qout_slot, SGIAIC7880State),
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
    dc->desc = "SGI O2 onboard AIC-7880 SCSI controller (placeholder)";
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
