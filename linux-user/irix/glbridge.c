/*
 * glbridge.c — IRIS GL → host OpenGL passthrough glue for IRIX userland emulation.
 *
 * Thin layer: read the guest's flushed GL command buffer and hand the raw bytes to an
 * external renderer .so (glrender.c, built normally with -lGL -lX11), which decodes the
 * protocol (glproto.h) and drives real host OpenGL in a window. Keeping the GL/X11 host
 * code in a dlopen'd .so keeps it entirely out of qemu's target build. The renderer path
 * is $GLRENDER_SO (default: libglrender.so on the loader path).
 *
 * Translate-the-GL-calls approach (NOT DGL). See memory irisgl_translate_not_dgl.
 */
#include "qemu/osdep.h"
#include "qemu.h"
#include "user-internals.h"
#include "glbridge.h"
#include "glrender.h"

#include <dlfcn.h>

typedef int (*glr_submit_fn)(const unsigned char *, int, struct glr_result *);
static glr_submit_fn g_submit;
static int g_tried;

static void load_renderer(void)
{
    const char *path;
    void *h;

    g_tried = 1;
    path = getenv("GLRENDER_SO");
    if (!path || !*path) {
        path = "libglrender.so";
    }
    h = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
    if (!h) {
        fprintf(stderr, "glbridge: dlopen(%s) failed: %s\n", path, dlerror());
        return;
    }
    g_submit = (glr_submit_fn)dlsym(h, "glr_submit");
    if (!g_submit) {
        fprintf(stderr, "glbridge: glr_submit not found in %s\n", path);
    }
}

long irix_gl_submit(abi_ulong gbuf, abi_ulong glen)
{
    void *buf;
    struct glr_result res;

    if (!g_tried) {
        load_renderer();
    }
    if (!g_submit || glen == 0) {
        return 0;
    }
    {
        static int once;
        if (!once) { once = 1;
            fprintf(stderr, "glbridge: GL passthrough active (first submit %u bytes)\n",
                    (unsigned)glen); }
    }

    buf = lock_user(VERIFY_READ, gbuf, glen, 1);
    if (!buf) {
        return 0;
    }
    memset(&res, 0, sizeof(res));
    g_submit(buf, (int)glen, &res);
    unlock_user(buf, gbuf, 0);

    /* a query op may write back into guest memory (getmatrix/gversion/qread/getsize…) */
    if (res.wb_len > 0 && res.wb_addr) {
        int n = res.wb_len > (int)sizeof(res.wb_data) ? (int)sizeof(res.wb_data)
                                                      : res.wb_len;
        void *w = lock_user(VERIFY_WRITE, res.wb_addr, n, 0);
        if (w) {
            memcpy(w, res.wb_data, n);
            unlock_user(w, res.wb_addr, n);
        }
    }
    if (res.wb2_len > 0 && res.wb2_addr) {       /* 2nd writeback (getorigin/getsize) */
        int n = res.wb2_len > (int)sizeof(res.wb2_data) ? (int)sizeof(res.wb2_data)
                                                        : res.wb2_len;
        void *w = lock_user(VERIFY_WRITE, res.wb2_addr, n, 0);
        if (w) {
            memcpy(w, res.wb2_data, n);
            unlock_user(w, res.wb2_addr, n);
        }
    }
    return res.retval;
}
