/*
 * IRIX/SVR4 target errno definitions.
 *
 * Ported from qemu-irix (Kai-Uwe Bloem <derkub@gmail.com>;
 * n64decomp/qemu-irix), GPLv2. In qemu-irix this was the
 * #ifdef TARGET_ABI_IRIX block in linux-user/errno_defs.h; in QEMU 10.x the
 * per-ABI errno table lives in target_errno_defs.h.
 */
#ifndef IRIX_TARGET_ERRNO_DEFS_H
#define IRIX_TARGET_ERRNO_DEFS_H

#define TARGET_EPERM	1	/* Operation not permitted		*/
#define TARGET_ENOENT	2	/* No such file or directory		*/
#define TARGET_ESRCH	3	/* No such process			*/
#define TARGET_EINTR	4	/* Interrupted function call		*/
#define TARGET_EIO	5	/* I/O error				*/
#define TARGET_ENXIO	6	/* No such device or address		*/
#define TARGET_E2BIG	7	/* Arg list too long			*/
#define TARGET_ENOEXEC	8	/* Exec format error			*/
#define TARGET_EBADF	9	/* Bad file number			*/
#define TARGET_ECHILD	10	/* No child processes			*/
#define TARGET_EAGAIN	11	/* Resource temporarily unavailable	*/
#define TARGET_ENOMEM	12	/* Not enough space			*/
#define TARGET_EACCES	13	/* Permission denied			*/
#define TARGET_EFAULT	14	/* Bad address				*/
#define TARGET_ENOTBLK	15	/* Block device required		*/
#define TARGET_EBUSY	16	/* Resource busy			*/
#define TARGET_EEXIST	17	/* File exists				*/
#define TARGET_EXDEV	18	/* Improper link			*/
#define TARGET_ENODEV	19	/* No such device			*/
#define TARGET_ENOTDIR	20	/* Not a directory			*/
#define TARGET_EISDIR	21	/* Is a directory			*/
#define TARGET_EINVAL	22	/* Invalid argument			*/
#define TARGET_ENFILE	23	/* File table overflow			*/
#define TARGET_EMFILE	24	/* Too many open files			*/
#define TARGET_ENOTTY	25	/* Inappropriate I/O control operation	*/
#define TARGET_ETXTBSY	26	/* Text file busy			*/
#define TARGET_EFBIG	27	/* File too large			*/
#define TARGET_ENOSPC	28	/* No space left on device		*/
#define TARGET_ESPIPE	29	/* Illegal seek				*/
#define TARGET_EROFS	30	/* Read only file system		*/
#define TARGET_EMLINK	31	/* Too many links			*/
#define TARGET_EPIPE	32	/* Broken pipe				*/
#define TARGET_EDOM	33	/* Domain error				*/
#define TARGET_ERANGE	34	/* Result too large			*/
#define TARGET_ENOMSG	35	/* No message of desired type		*/
#define TARGET_EIDRM	36	/* Identifier removed			*/
#define TARGET_ECHRNG	37	/* Channel number out of range		*/
#define TARGET_EL2NSYNC 38	/* Level 2 not synchronized		*/
#define TARGET_EL3HLT	39	/* Level 3 halted			*/
#define TARGET_EL3RST	40	/* Level 3 reset			*/
#define TARGET_ELNRNG	41	/* Link number out of range		*/
#define TARGET_EUNATCH 42	/* Protocol driver not attached		*/
#define TARGET_ENOCSI	43	/* No CSI structure available		*/
#define TARGET_EL2HLT	44	/* Level 2 halted			*/
#define TARGET_EDEADLK	45	/* Resource deadlock avoided		*/
#define TARGET_ENOLCK	46	/* No locks available			*/
#define TARGET_ECKPT	47	/* POSIX checkpoint/restart error	*/

/* Convergent Error Returns */
#define TARGET_EBADE	50	/* invalid exchange			*/
#define TARGET_EBADR	51	/* invalid request descriptor		*/
#define TARGET_EXFULL	52	/* exchange full			*/
#define TARGET_ENOANO	53	/* no anode				*/
#define TARGET_EBADRQC	54	/* invalid request code			*/
#define TARGET_EBADSLT	55	/* invalid slot				*/
#define TARGET_EDEADLOCK 56	/* file locking deadlock error		*/

#define TARGET_EBFONT	57	/* bad font file fmt			*/

/* stream problems */
#define TARGET_ENOSTR	60	/* Device not a stream			*/
#define TARGET_ENODATA	61	/* no data (for no delay io)		*/
#define TARGET_ETIME	62	/* timer expired			*/
#define TARGET_ENOSR	63	/* out of streams resources		*/

#define TARGET_ENONET	64	/* Machine is not on the network	*/
#define TARGET_ENOPKG	65	/* Package not installed                */
#define TARGET_EREMOTE	66	/* The object is remote			*/
#define TARGET_ENOLINK	67	/* the link has been severed */
#define TARGET_EADV	68	/* advertise error */
#define TARGET_ESRMNT	69	/* srmount error */

#define TARGET_ECOMM	70	/* Communication error on send		*/
#define TARGET_EPROTO	71	/* Protocol error			*/
#define TARGET_EMULTIHOP 74	/* multihop attempted */
#define TARGET_EBADMSG 77	/* Bad message				*/
#define TARGET_ENAMETOOLONG 78	/* Filename too long */
#define TARGET_EOVERFLOW 79	/* value too large to be stored in data type */
#define TARGET_ENOTUNIQ 80	/* given log. name not unique */
#define TARGET_EBADFD	 81	/* f.d. invalid for this operation */
#define TARGET_EREMCHG	 82	/* Remote address changed */

/* shared library problems */
#define TARGET_ELIBACC	83	/* Can't access a needed shared lib.	*/
#define TARGET_ELIBBAD	84	/* Accessing a corrupted shared lib.	*/
#define TARGET_ELIBSCN	85	/* .lib section in a.out corrupted.	*/
#define TARGET_ELIBMAX	86	/* Attempting to link in too many libs.	*/
#define TARGET_ELIBEXEC 87	/* Attempting to exec a shared library.	*/
#define TARGET_EILSEQ	88	/* Illegal byte sequence. */
#define TARGET_ENOSYS	89	/* Function not implemented		*/
#define TARGET_ELOOP	90	/* Symbolic link loop */
#define TARGET_ERESTART 91	/* Restartable system call */
#define TARGET_ESTRPIPE 92	/* if pipe/FIFO, don't sleep in stream head */

#define TARGET_ENOTEMPTY 93	/* Directory not empty */

#define TARGET_EUSERS	94	/* Too many users (for UFS) */

/* BSD Networking Software */
	/* argument errors */
#define TARGET_ENOTSOCK	95		/* Socket operation on non-socket */
#define TARGET_EDESTADDRREQ	96		/* Destination address required */
#define TARGET_EMSGSIZE	97		/* Inappropriate message buffer length */
#define TARGET_EPROTOTYPE	98		/* Protocol wrong type for socket */
#define TARGET_ENOPROTOOPT	99		/* Protocol not available */
#define TARGET_EPROTONOSUPPORT	120		/* Protocol not supported */
#define TARGET_ESOCKTNOSUPPORT	121		/* Socket type not supported */
#define TARGET_EOPNOTSUPP	122		/* Operation not supported on socket */
#define TARGET_EPFNOSUPPORT	123		/* Protocol family not supported */
#define TARGET_EAFNOSUPPORT	124		/* Address family not supported by
					   protocol family */
#define TARGET_EADDRINUSE	125		/* Address already in use */
#define TARGET_EADDRNOTAVAIL	126		/* Can't assign requested address */

/* operational errors */
#define TARGET_ENETDOWN	127		/* Network is down */
#define TARGET_ENETUNREACH	128		/* Network is unreachable */
#define TARGET_ENETRESET	129		/* Network dropped connection because
					   of reset */
#define TARGET_ECONNABORTED	130		/* Software caused connection abort */
#define TARGET_ECONNRESET	131		/* Connection reset by peer */
#define TARGET_ENOBUFS		132	       	/* No buffer space available */
#define TARGET_EISCONN		133		/* Socket is already connected */
#define TARGET_ENOTCONN	134		/* Socket is not connected */

/* XENIX has 135 - 142 */
#define TARGET_ESHUTDOWN	143		/* Can't send after socket shutdown */
#define TARGET_ETOOMANYREFS	144		/* Too many references: can't splice */
#define TARGET_ETIMEDOUT	145		/* Connection timed out */
#define TARGET_ECONNREFUSED	146		/* Connection refused */
#define TARGET_EHOSTDOWN	147		/* Host is down */
#define TARGET_EHOSTUNREACH	148		/* No route to host */

#define TARGET_EWOULDBLOCK	TARGET_EAGAIN

#define TARGET_EALREADY	149		/* operation already in progress */
#define TARGET_EINPROGRESS	150		/* operation now in progress */
/* SUN Network File System */
#define TARGET_ESTALE		151		/* Stale NFS file handle */

/* XENIX error numbers */
#define TARGET_EUCLEAN 	135	/* Structure needs cleaning */
#define TARGET_ENOTNAM		137	/* Not a XENIX named type file */
#define TARGET_ENAVAIL		138	/* No XENIX semaphores available */
#define TARGET_EISNAM		139	/* Is a named type file */
#define TARGET_EREMOTEIO	140	/* Remote I/O error */
#define TARGET_EINIT		141	/* Reserved for future */
#define TARGET_EREMDEV		142	/* Error 142 */
#define TARGET_ECANCELED	158	/* AIO operation canceled */

/*
 * Linux-only errnos referenced by the shared errno conversion table
 * (errnos.c.inc). IRIX/SVR4 has no equivalents; assign distinct out-of-band
 * values so host<->target conversion has somewhere to map them. IRIX guest
 * binaries never see these.
 */
#define TARGET_EDQUOT		1133	/* Disc quota exceeded (SVR4 value) */
#define TARGET_EDOTDOT		1160
#define TARGET_EHWPOISON	1161
#define TARGET_EKEYEXPIRED	1162
#define TARGET_EKEYREJECTED	1163
#define TARGET_EKEYREVOKED	1164
#define TARGET_EMEDIUMTYPE	1165
#define TARGET_ENOKEY		1166
#define TARGET_ENOMEDIUM	1167
#define TARGET_ENOTRECOVERABLE	1168
#define TARGET_EOWNERDEAD	1169
#define TARGET_ERFKILL		1170

#endif /* IRIX_TARGET_ERRNO_DEFS_H */
