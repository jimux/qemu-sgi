/*
 *  IRIX network-interface ioctl marshalling for linux-user
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * ---------------------------------------------------------------------------
 *
 * The command numbers, the sweep that chose them, and the two struct sizes are
 * in linux-user/irix/termbits.h. What is left for this file is the part that
 * is *not* just a number: three of the sixteen commands carry data laid out
 * differently from any host ioctl, and one whole field needs value translation.
 *
 * WHAT WE GET FOR FREE
 * --------------------
 * Two happy accidents make most of the group a table entry rather than code.
 *
 * First, **IRIX's struct sockaddr is Linux-shaped, not BSD-shaped.** This was
 * the one thing worth checking before writing any of this, because a `sa_len`
 * byte would have poisoned every address-carrying command. It is not there:
 * f/irix/kern/bsd/sys/socket.h:193-210 defines the struct with an
 * `#undef _HAVE_SA_LEN` on the line immediately *before* the `#ifdef` that
 * would have selected the BSD form, so the 4.3BSD branch is unconditionally
 * dead and the compiled struct is `sa_family_t sa_family; char sa_data[14]`
 * with `sa_family_t` = `unsigned short` (socket.h:181). netinet/in.h:262-273
 * does the same for sockaddr_in with `_HAVE_SIN_LEN`. So IRIX's sockaddr is
 * byte-for-byte Linux's, and linux-user's existing STRUCT_sockaddr thunk is
 * already correct for this target.
 *
 * Second, linux-user's `STRUCT_sockaddr_ifreq` (16-byte name + 16-byte
 * sockaddr) is 32 bytes on n32, exactly `sizeof(struct ifreq)` on IRIX, and
 * `STRUCT_ifconf` is 8, exactly IRIX's. That means the generic
 * `do_ioctl_ifconf()` handler -- which reads the target ifconf, sizes the host
 * buffer from the target ifreq stride, and converts each returned entry back --
 * works for IRIX's SIOCGIFCONF unmodified. It is reused as-is.
 *
 * WHAT NEEDS CODE
 * ---------------
 * 1. **ifr_flags is 32 bits wide on IRIX** (`__uint32_t ifru_flags`,
 *    f/irix/kern/bsd/net/if.h:520) where Linux's is a `short`. Using
 *    linux-user's STRUCT_short_ifreq would put the flags in the wrong two
 *    bytes of a big-endian word -- i.e. it would look like it worked and
 *    return zero.
 *
 * 2. **The IFF_ bit values diverge above 0x200.** Both start
 *    UP/BROADCAST/DEBUG/LOOPBACK/POINTOPOINT/NOTRAILERS/RUNNING/NOARP/
 *    PROMISC/ALLMULTI at 0x1..0x200, and then part company
 *    (if.h:205-233 vs the host's <net/if.h>):
 *
 *        bit      IRIX                       Linux
 *        0x400    IFF_FILTMULTI/INTELLIGENT   IFF_MASTER
 *        0x800    IFF_MULTICAST               IFF_SLAVE
 *        0x1000   IFF_CKSUM                   IFF_MULTICAST
 *        0x2000   IFF_ALLCAST                 IFF_PORTSEL
 *        0x4000   IFF_DRVRLOCK                IFF_AUTOMEDIA
 *        0x8000   IFF_PRIVATE                 IFF_DYNAMIC
 *        0x10000  IFF_LINK0                   IFF_LOWER_UP
 *        0x20000  IFF_LINK1                   IFF_DORMANT
 *        0x40000  IFF_LINK2                   IFF_ECHO
 *
 *    The one that matters is IFF_MULTICAST, which moves. Passed through
 *    untranslated, a multicast-capable interface would come back to the guest
 *    flagged IFF_CKSUM and *not* IFF_MULTICAST -- and libc's `_get_myaddress`
 *    (the RPC/NIS local-address helper, which is why SIOCGIFFLAGS is in
 *    libc.so.1 at all) plus `chooser`'s broadcast scan both read these flags to
 *    decide what an interface can do. Everything above 0x800 has no shared
 *    meaning at all, so it is dropped rather than mistranslated.
 *
 * 3. **SIOCGENADDR returns six raw bytes, not a sockaddr.** IRIX puts the
 *    hardware address in `char ifru_enaddr[6]` (if.h:524, the union member
 *    the comment marks "MIPS ABI"); the host's nearest equivalent,
 *    SIOCGIFHWADDR, returns a `struct sockaddr` whose family says which kind
 *    of link it is. So the six address bytes have to be lifted out of
 *    sa_data[].
 *
 * NOT MAPPED, DELIBERATELY
 * ------------------------
 * SIOCGIFSTATS (osview, gr_osview) hands back a `struct ifstats` inside the
 * ifreq union (if.h:471-477: ipackets, opackets, ierrors, oerrors, collisions
 * as u_int/u_short). The host has no single ioctl that produces those counters
 * -- they live in /proc/net/dev or an rtnetlink dump -- so mapping it would
 * mean building the answer from another source, which is a different job from
 * translating an ioctl. SIOCSIFHEAD (libsaNetwork) has no host equivalent
 * either. Both are left to fall through to the ordinary "Unsupported ioctl"
 * ENOTTY, and both are ledgered rather than silently faked.
 */
#ifndef IRIX_TARGET_IFREQ_H
#define IRIX_TARGET_IFREQ_H

/*
 * ifr_flags. Only the ten bits both systems agree on are carried across; see
 * the divergence table above for why the rest are dropped.
 * IRIX values: f/irix/kern/bsd/net/if.h:205-215 and :218.
 */
#define TARGET_IRIX_IFF_UP              0x0001
#define TARGET_IRIX_IFF_BROADCAST       0x0002
#define TARGET_IRIX_IFF_DEBUG           0x0004
#define TARGET_IRIX_IFF_LOOPBACK        0x0008
#define TARGET_IRIX_IFF_POINTOPOINT     0x0010
#define TARGET_IRIX_IFF_NOTRAILERS      0x0020
#define TARGET_IRIX_IFF_RUNNING         0x0040
#define TARGET_IRIX_IFF_NOARP           0x0080
#define TARGET_IRIX_IFF_PROMISC         0x0100
#define TARGET_IRIX_IFF_ALLMULTI        0x0200
#define TARGET_IRIX_IFF_MULTICAST       0x0800  /* if.h:218 -- Linux has 0x1000 */

static const bitmask_transtbl irix_ifflags_tbl[] = {
    { TARGET_IRIX_IFF_UP,          TARGET_IRIX_IFF_UP,          IFF_UP,          IFF_UP },
    { TARGET_IRIX_IFF_BROADCAST,   TARGET_IRIX_IFF_BROADCAST,   IFF_BROADCAST,   IFF_BROADCAST },
    { TARGET_IRIX_IFF_DEBUG,       TARGET_IRIX_IFF_DEBUG,       IFF_DEBUG,       IFF_DEBUG },
    { TARGET_IRIX_IFF_LOOPBACK,    TARGET_IRIX_IFF_LOOPBACK,    IFF_LOOPBACK,    IFF_LOOPBACK },
    { TARGET_IRIX_IFF_POINTOPOINT, TARGET_IRIX_IFF_POINTOPOINT, IFF_POINTOPOINT, IFF_POINTOPOINT },
    { TARGET_IRIX_IFF_NOTRAILERS,  TARGET_IRIX_IFF_NOTRAILERS,  IFF_NOTRAILERS,  IFF_NOTRAILERS },
    { TARGET_IRIX_IFF_RUNNING,     TARGET_IRIX_IFF_RUNNING,     IFF_RUNNING,     IFF_RUNNING },
    { TARGET_IRIX_IFF_NOARP,       TARGET_IRIX_IFF_NOARP,       IFF_NOARP,       IFF_NOARP },
    { TARGET_IRIX_IFF_PROMISC,     TARGET_IRIX_IFF_PROMISC,     IFF_PROMISC,     IFF_PROMISC },
    { TARGET_IRIX_IFF_ALLMULTI,    TARGET_IRIX_IFF_ALLMULTI,    IFF_ALLMULTI,    IFF_ALLMULTI },
    { TARGET_IRIX_IFF_MULTICAST,   TARGET_IRIX_IFF_MULTICAST,   IFF_MULTICAST,   IFF_MULTICAST },
    { 0, 0, 0, 0 }
};

/* Offset of the ifr_ifru union inside struct ifreq: IFNAMSIZ (if.h:514). */
#define TARGET_IRIX_IFREQ_UNION_OFF     16

/*
 * Copy the interface name out of the guest's ifreq and prime a host one.
 * Returns 0, or a negative target errno.
 */
static abi_long irix_ifreq_get_name(abi_long arg, struct ifreq *ifr)
{
    void *p = lock_user(VERIFY_READ, arg, TARGET_IRIX_IFREQ_SIZE, 1);

    if (!p) {
        return -TARGET_EFAULT;
    }
    memset(ifr, 0, sizeof(*ifr));
    memcpy(ifr->ifr_name, p, IFNAMSIZ);
    ifr->ifr_name[IFNAMSIZ - 1] = '\0';
    unlock_user(p, arg, 0);
    return 0;
}

/*
 * SIOCGIFFLAGS / SIOCSIFFLAGS, and the legacy 16-bit OSIOC* pair.
 *
 * The "old" variants exist because the flags word grew: they exchange only a
 * short, in the *first two bytes* of the 32-bit ifr_flags. That is not a guess
 * -- it is what the kernel does on each side:
 *
 *   f/irix/kern/bsd/net/if.c:1617-1619  case OSIOCGIFFLAGS:
 *                                         *((short*)(&ifr->ifr_flags)) =
 *                                                 (short)ifp->if_flags;
 *   f/irix/kern/bsd/net/if.c:1655-1661  case OSIOCSIFFLAGS:
 *                                         ifr->ifr_flags >>= 16;
 *                                         ifr->ifr_flags &= 0xffff;
 *
 * i.e. on a big-endian target the value lives in the high half of the word,
 * which is the same two bytes a `short` store lands on. ifconfig, arrayd,
 * bootpc and appletalk/atconfig all still issue them.
 */
static abi_long do_ioctl_irix_ifflags(const IOCTLEntry *ie, uint8_t *buf_temp,
                                      int fd, int cmd, abi_long arg)
{
    struct ifreq ifr;
    abi_long ret;
    uint32_t flags;
    bool is_set = (cmd == TARGET_IRIX_SIOCSIFFLAGS ||
                   cmd == TARGET_IRIX_OSIOCSIFFLAGS);
    bool is_old = (cmd == TARGET_IRIX_OSIOCGIFFLAGS ||
                   cmd == TARGET_IRIX_OSIOCSIFFLAGS);
    char *u;
    void *p;

    ret = irix_ifreq_get_name(arg, &ifr);
    if (ret < 0) {
        return ret;
    }

    if (is_set) {
        p = lock_user(VERIFY_READ, arg, TARGET_IRIX_IFREQ_SIZE, 1);
        if (!p) {
            return -TARGET_EFAULT;
        }
        u = (char *)p + TARGET_IRIX_IFREQ_UNION_OFF;
        flags = is_old ? tswap16(*(uint16_t *)u) : tswap32(*(uint32_t *)u);
        unlock_user(p, arg, 0);
        ifr.ifr_flags = target_to_host_bitmask(flags, irix_ifflags_tbl);
        return get_errno(safe_ioctl(fd, SIOCSIFFLAGS, &ifr));
    }

    ret = get_errno(safe_ioctl(fd, SIOCGIFFLAGS, &ifr));
    if (is_error(ret)) {
        return ret;
    }
    /* Linux's ifr_flags is a signed short; take it unsigned before mapping. */
    flags = host_to_target_bitmask((unsigned short)ifr.ifr_flags,
                                   irix_ifflags_tbl);
    p = lock_user(VERIFY_WRITE, arg, TARGET_IRIX_IFREQ_SIZE, 0);
    if (!p) {
        return -TARGET_EFAULT;
    }
    u = (char *)p + TARGET_IRIX_IFREQ_UNION_OFF;
    if (is_old) {
        *(uint16_t *)u = tswap16((uint16_t)flags);
    } else {
        *(uint32_t *)u = tswap32(flags);
    }
    unlock_user(p, arg, TARGET_IRIX_IFREQ_SIZE);
    return ret;
}

/* SIOCGENADDR: six raw bytes of hardware address in ifr_enaddr. */
static abi_long do_ioctl_irix_genaddr(const IOCTLEntry *ie, uint8_t *buf_temp,
                                      int fd, int cmd, abi_long arg)
{
    struct ifreq ifr;
    abi_long ret;
    void *p;

    ret = irix_ifreq_get_name(arg, &ifr);
    if (ret < 0) {
        return ret;
    }
    ret = get_errno(safe_ioctl(fd, SIOCGIFHWADDR, &ifr));
    if (is_error(ret)) {
        return ret;
    }
    p = lock_user(VERIFY_WRITE, arg, TARGET_IRIX_IFREQ_SIZE, 0);
    if (!p) {
        return -TARGET_EFAULT;
    }
    memcpy((char *)p + TARGET_IRIX_IFREQ_UNION_OFF, ifr.ifr_hwaddr.sa_data, 6);
    unlock_user(p, arg, TARGET_IRIX_IFREQ_SIZE);
    return ret;
}

#endif /* IRIX_TARGET_IFREQ_H */
