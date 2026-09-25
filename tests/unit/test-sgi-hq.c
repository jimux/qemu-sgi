/*
 * Unit tests for the shared SGI host-queue + microcode helpers
 * (hw/misc/sgi_hq_common.c).
 *
 * The GR2 model depends on two of these behaviours and MGRAS/GE11 will too, so
 * they are pinned here:
 *
 *  - the microcode store is addressed by a PC register and returns values
 *    VERBATIM (the drivers download, re-select each PC and read the words back,
 *    masking them themselves);
 *  - the FIFO status word carries the idle and ucode-ready bits at the
 *    positions the caller passes, and reports the queue empty.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/sgi_hq.h"

static void test_ucode_download_verify(void)
{
    SgiUcode u;
    static const uint32_t words[SGI_UCODE_WORDS] = {
        0x3dfffff, 0x00000000, 0xdeadbeef, 0x80000001, 0x7fffffff,
    };
    unsigned i;

    sgi_ucode_reset(&u);
    g_assert_false(u.ready);
    g_assert_cmpuint(sgi_ucode_pc(&u), ==, 0);

    /* Download a record for PC 0x1234. */
    sgi_ucode_set_pc(&u, 0x1234);
    for (i = 0; i < SGI_UCODE_WORDS; i++) {
        sgi_ucode_write(&u, i, words[i]);
    }

    /* Verify pass: the words come back verbatim, in order. */
    for (i = 0; i < SGI_UCODE_WORDS; i++) {
        g_assert_cmpuint(sgi_ucode_read(&u, i), ==, words[i]);
    }

    /* Selecting the PC does not disturb it, and another PC is independent. */
    g_assert_cmpuint(sgi_ucode_pc(&u), ==, 0x1234);
    sgi_ucode_set_pc(&u, 0x2000);
    for (i = 0; i < SGI_UCODE_WORDS; i++) {
        g_assert_cmpuint(sgi_ucode_read(&u, i), ==, 0);
    }
    sgi_ucode_set_pc(&u, 0x1234);
    for (i = 0; i < SGI_UCODE_WORDS; i++) {
        g_assert_cmpuint(sgi_ucode_read(&u, i), ==, words[i]);
    }
}

static void test_ucode_pc_is_masked(void)
{
    SgiUcode u;

    sgi_ucode_reset(&u);
    /* The store is indexed by pc & (SGI_UCODE_PCS - 1). */
    sgi_ucode_set_pc(&u, SGI_UCODE_PCS + 7);
    sgi_ucode_write(&u, 0, 0xabc);
    g_assert_cmpuint(sgi_ucode_read(&u, 0), ==, 0xabc);

    sgi_ucode_set_pc(&u, SGI_UCODE_PCS * 5 + 7);
    g_assert_cmpuint(sgi_ucode_read(&u, 0), ==, 0xabc);
}

static void test_ucode_out_of_range(void)
{
    SgiUcode u;

    sgi_ucode_reset(&u);
    sgi_ucode_set_pc(&u, 3);
    /* Out-of-range indices must not corrupt anything (and read as 0). */
    sgi_ucode_write(&u, SGI_UCODE_WORDS, 0xffffffff);
    sgi_ucode_write(&u, 99, 0xffffffff);
    g_assert_cmpuint(sgi_ucode_read(&u, SGI_UCODE_WORDS), ==, 0);
    g_assert_cmpuint(sgi_ucode_read(&u, 99), ==, 0);
    /* The real words are untouched. */
    sgi_ucode_write(&u, 0, 0x11);
    g_assert_cmpuint(sgi_ucode_read(&u, 0), ==, 0x11);
}

static void test_ucode_reset(void)
{
    SgiUcode u;

    sgi_ucode_reset(&u);
    sgi_ucode_set_pc(&u, 0x100);
    sgi_ucode_write(&u, 1, 0x12345678);
    u.ready = true;

    sgi_ucode_reset(&u);
    g_assert_cmpuint(sgi_ucode_pc(&u), ==, 0);
    g_assert_false(u.ready);
    g_assert_cmpuint(sgi_ucode_read(&u, 1), ==, 0);
}

static void test_hq_reset_and_identity(void)
{
    SgiHQState hq;

    sgi_hq_reset(&hq, 0xdeadbeef, 0x1234, 4);
    g_assert_cmpuint(hq.magic, ==, 0xdeadbeef);
    g_assert_cmpuint(hq.version, ==, 0x1234);
    g_assert_cmpuint(hq.numge, ==, 4);
    g_assert_false(hq.idle);
    g_assert_false(hq.ucode.ready);
}

static void test_hq_fifostat(void)
{
    SgiHQState hq;
    const uint32_t idle_bit = 0x1, ready_bit = 0x2;

    sgi_hq_reset(&hq, 0, 0, 1);

    /* Idle and ucode-ready are the block's flags, at the caller's positions. */
    g_assert_cmpuint(sgi_hq_fifostat(&hq, idle_bit, ready_bit), ==, 0);

    hq.idle = true;
    g_assert_cmpuint(sgi_hq_fifostat(&hq, idle_bit, ready_bit), ==, idle_bit);

    hq.ucode.ready = true;
    g_assert_cmpuint(sgi_hq_fifostat(&hq, idle_bit, ready_bit),
                     ==, idle_bit | ready_bit);

    /* Occupancy (bits 6..12) and the error bit (5) are always clear. */
    g_assert_cmpuint(sgi_hq_fifostat(&hq, idle_bit, ready_bit) & 0x1fe0, ==, 0);

    /* A part that moves the bits still gets a consistent word. */
    g_assert_cmpuint(sgi_hq_fifostat(&hq, 0x8, 0x4), ==, 0x8 | 0x4);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/sgi-hq/ucode/download-verify", test_ucode_download_verify);
    g_test_add_func("/sgi-hq/ucode/pc-masked", test_ucode_pc_is_masked);
    g_test_add_func("/sgi-hq/ucode/out-of-range", test_ucode_out_of_range);
    g_test_add_func("/sgi-hq/ucode/reset", test_ucode_reset);
    g_test_add_func("/sgi-hq/block/reset-identity", test_hq_reset_and_identity);
    g_test_add_func("/sgi-hq/block/fifostat", test_hq_fifostat);

    return g_test_run();
}
