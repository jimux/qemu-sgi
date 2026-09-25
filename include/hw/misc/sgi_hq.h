/*
 * Shared SGI graphics host-queue + microcode helpers.
 *
 * SGI's graphics ASICs (GR2/HQ2+GE7 on Indy, and MGRAS/GE11 on Octane) are all
 * "a host command queue feeding one or more geometry engines running
 * downloadable microcode".  The plumbing around that is the same problem on
 * every part; only the command set, the register addresses and the microcode
 * ISA differ.  This header carries the machine-independent plumbing so each
 * device model can reuse it instead of re-deriving it.
 *
 * See progress_notes/indy/xz-gr2/146-interface-note-hq-microcode-reuse-mgras.md.
 *
 * Behaviour note: the microcode store keeps values VERBATIM.  The driver does a
 * download-then-verify pass (select a PC, write the words, re-select and read
 * them back) and masks them itself, so nothing here may tidy a value.
 */

#ifndef HW_MISC_SGI_HQ_H
#define HW_MISC_SGI_HQ_H

#include <stdbool.h>
#include <stdint.h>

/* PC space the driver walks when downloading/verifying microcode. */
#define SGI_UCODE_PCS   0x10000
/* Words stored per PC: the instruction window plus the load-data register. */
#define SGI_UCODE_WORDS 5

/* Per-engine microcode store, addressed by a PC register the driver writes
 * before each transfer.  Values are stored and returned exactly as written. */
typedef struct SgiUcode {
    uint32_t pc;                                       /* current PC / load addr */
    uint32_t words[SGI_UCODE_PCS][SGI_UCODE_WORDS];    /* per-PC storage          */
    bool ready;                                        /* "ucode ready" latch     */
} SgiUcode;

/* Zero the store and clear the ready latch. */
void sgi_ucode_reset(SgiUcode *u);

/* Select the PC subsequent transfers are addressed by (the driver's GEPC). */
void sgi_ucode_set_pc(SgiUcode *u, uint32_t pc);
uint32_t sgi_ucode_pc(const SgiUcode *u);

/* Store (write) or return (read) one word of the selected PC's slot.  The PC
 * is masked into SGI_UCODE_PCS, as the hardware does. */
void sgi_ucode_write(SgiUcode *u, unsigned idx, uint32_t val);
uint32_t sgi_ucode_read(const SgiUcode *u, unsigned idx);

/* Host-queue register block.  `magic`, `version` and the FIFO levels are what
 * the driver reads to identify and configure the queue; the status word the
 * driver polls is assembled by sgi_hq_fifostat(). */
typedef struct SgiHQState {
    uint32_t magic;        /* presence magic the driver probes            */
    uint32_t version;      /* revision, read by the driver as ver >> 16   */
    uint32_t numge;        /* geometry engines present                    */
    uint32_t fifo_full_t;  /* full-timeout the driver writes              */
    uint32_t fifo_empty_t; /* empty-timeout                               */
    uint32_t fifo_full;    /* occupancy at which the queue is "full"      */
    uint32_t fifo_empty;   /* occupancy at which the queue is "empty"     */
    bool     idle;         /* queue idle/ready                            */
    SgiUcode ucode;
} SgiHQState;

/* Initialise the block; `magic` is the presence value the driver probes. */
void sgi_hq_reset(SgiHQState *hq, uint32_t magic, uint32_t version,
                  uint32_t numge);

/* The FIFO status word.  Layout is shared across SGI parts: occupancy in bits
 * 6..12 (mask 0x1fc0), error in bit 5, idle/ready in bit 0, ucode-ready in bit
 * 1.  The caller passes its device's two bit positions so a part that moves them
 * still gets a consistent word.  Occupancy is reported empty (0): the model
 * drains the queue synchronously. */
uint32_t sgi_hq_fifostat(const SgiHQState *hq, uint32_t idle_bit,
                         uint32_t ready_bit);

#endif /* HW_MISC_SGI_HQ_H */
