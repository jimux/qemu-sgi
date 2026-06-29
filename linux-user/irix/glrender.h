/* glrender.h — interface between the qemu glue (linux-user/irix/glbridge.c) and the
 * external host renderer .so (glrender.c, built normally with -lGL -lX11).
 *
 * qemu reads the guest GL command buffer, hands the raw bytes to glr_submit(), and on
 * return applies any writeback (a query that returns data into guest memory) and uses
 * retval as the syscall result. This keeps all GL/X11 host code OUT of qemu's build
 * (qemu only dlopen's this .so), and lets the renderer be rebuilt independently.
 */
#ifndef GLRENDER_H
#define GLRENDER_H

struct glr_result {
    long          retval;       /* syscall return value (for a trailing query op) */
    unsigned int  wb_addr;      /* guest address to write back to (0 = none) */
    int           wb_len;       /* bytes of wb_data to write to wb_addr */
    unsigned char wb_data[64];  /* writeback payload (getmatrix=64B, gversion=12B, qread=2B) */
    unsigned int  wb2_addr;     /* 2nd writeback (getorigin/getsize write two pointers) */
    int           wb2_len;
    unsigned char wb2_data[16];
};

/* Process one flushed command buffer (glproto.h encoding). Returns 0 on success.
 * `res` is zeroed by the caller. */
int glr_submit(const unsigned char *buf, int len, struct glr_result *res);

#endif /* GLRENDER_H */
