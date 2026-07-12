/*
 * SGI VIRTUIX/IP55 -- guest-physical-address translation for device DMA.
 *
 * Copyright (c) 2024 the QEMU project
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SGI_VIRTUIX_GPA_H
#define HW_MISC_SGI_VIRTUIX_GPA_H

/*
 * Translate an address a virtuix device received from the guest (a DMA
 * descriptor pointer, a ring base, a scanout base, ...) into a guest physical
 * address suitable for &address_space_memory.
 *
 * The IP22/IP24 memory controller (hw/misc/sgi_mc_virtuix.c) maps guest RAM in
 * two physical segments: SEG0 @ 0x08000000 (banks 0-1, up to 256MB) and
 * SEG1 @ 0x20000000 (bank 2, the next 128MB) -- i.e. a >256MB guest has RAM
 * whose physical address has bit 29 (0x20000000) set.
 *
 * IRIX drivers hand these devices addresses in one of two forms:
 *   - raw *physical* addresses (bit 31 clear), from kvtophys() on a KSEG0 buffer
 *     or from a DMA descriptor the guest built with physical pointers; and
 *   - KSEG0/KSEG1 *virtual* addresses (bit 31 set), which are a 512MB window
 *     onto physical 0..0x1fffffff and therefore never name SEG1 at all.
 *
 * The historical `& 0x1fffffff` (29-bit) mask is correct for the *virtual*
 * form (it strips the KSEG tag) but silently drops bit 29 for the *physical*
 * form, redirecting every SEG1 buffer to a wrong low page -- the BL-54 bug
 * class (see progress_notes/ip55/bl54_docker_slirp.md and
 * progress_notes/ip55/seg1_mask_sweep.md).  It is a harmless no-op at <=256MB
 * (SEG0 only) which is why it hid on the daily -m 256M driver and bit only the
 * -m 512M Docker/appliance configs.
 *
 * So: strip KSEG for virtual addresses (bit 31 set) but preserve bit 29 for raw
 * physical addresses (mask to 30 bits, comfortably above the 0x28000000 SEG1
 * ceiling of a maxed 384MB guest, below any device MMIO).
 */
#define SGI_VIRTUIX_GPA(a)                                          \
    (((uint32_t)(a) & 0x80000000U) ? ((uint32_t)(a) & 0x1fffffffU)  \
                                   : ((uint32_t)(a) & 0x3fffffffU))

#endif /* HW_MISC_SGI_VIRTUIX_GPA_H */
