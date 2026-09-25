/*
 * Shared SGI graphics host-queue + microcode helpers.  See
 * include/hw/misc/sgi_hq.h and progress note 146.
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_hq.h"

void sgi_ucode_reset(SgiUcode *u)
{
    memset(u, 0, sizeof(*u));
}

void sgi_ucode_set_pc(SgiUcode *u, uint32_t pc)
{
    u->pc = pc;
}

uint32_t sgi_ucode_pc(const SgiUcode *u)
{
    return u->pc;
}

void sgi_ucode_write(SgiUcode *u, unsigned idx, uint32_t val)
{
    if (idx >= SGI_UCODE_WORDS) {
        return;
    }
    u->words[u->pc & (SGI_UCODE_PCS - 1)][idx] = val;
}

uint32_t sgi_ucode_read(const SgiUcode *u, unsigned idx)
{
    if (idx >= SGI_UCODE_WORDS) {
        return 0;
    }
    return u->words[u->pc & (SGI_UCODE_PCS - 1)][idx];
}

void sgi_hq_reset(SgiHQState *hq, uint32_t magic, uint32_t version,
                  uint32_t numge)
{
    memset(hq, 0, sizeof(*hq));
    hq->magic = magic;
    hq->version = version;
    hq->numge = numge;
    sgi_ucode_reset(&hq->ucode);
}

uint32_t sgi_hq_fifostat(const SgiHQState *hq, uint32_t idle_bit,
                         uint32_t ready_bit)
{
    uint32_t status = 0;

    /* Occupancy (bits 6..12) and error (bit 5) stay 0: the model drains the
     * queue synchronously, so it is always empty and fault-free. */
    if (hq->idle) {
        status |= idle_bit;
    }
    if (hq->ucode.ready) {
        status |= ready_bit;
    }
    return status;
}
