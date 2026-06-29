/*
 * glbridge.h — IRIS GL → host OpenGL passthrough for IRIX userland emulation.
 *
 * The guest libgl.so shim marshals IRIS GL calls into a command buffer and flushes it
 * via a custom syscall (GLSYS_NR). cpu_loop.c routes that syscall here; this glue reads
 * the guest buffer and hands it to an external renderer .so (dlopen'd) that drives real
 * host OpenGL. See progress_notes/irisgl_re/userland_atlantis.md.
 *
 * Part of the IRIX virtualization-native graphics path (translate GL calls, not DGL).
 */
#ifndef IRIX_GLBRIDGE_H
#define IRIX_GLBRIDGE_H

/* Custom IRIX syscall number for the GL passthrough. MUST match glproto.h (guest). */
#define GLSYS_NR 1987

/* Process a flushed guest GL command buffer at guest address `gbuf` (`glen` bytes).
 * Returns the value for a trailing query op (lands in the guest's v0). */
long irix_gl_submit(abi_ulong gbuf, abi_ulong glen);

#endif /* IRIX_GLBRIDGE_H */
