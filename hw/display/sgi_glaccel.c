/*
 * SGI Paravirtual GL Accelerator — Dumb Framebuffer Mode
 *
 * This device provides a simple DMA-based framebuffer for the IP30/IP54
 * machine. The guest writes pixel data into a physically contiguous
 * buffer and programs FB_BASE/WIDTH/HEIGHT/FORMAT. The device DMAs
 * the guest framebuffer into the QEMU console surface at 60Hz.
 *
 * A future phase will add OpenGL command passthrough.
 *
 * Copyright (c) 2024 the QEMU project
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* clang-format off */
#include "qemu/osdep.h"
/* clang-format on */
#include "hw/display/sgi_glaccel.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/sockets.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "ui/dmabuf.h"
#include "ui/pixel_ops.h"
#include "framebuffer.h"
#include <poll.h>
#include <dlfcn.h>

#define SGI_GLACCEL_MMIO_SIZE 0x100

/* ============================================================================
 * IN-PROCESS host-GPU GL — the virtio-gpu/virgl-shaped END-GOAL (NOT scaffolding).
 *
 * CLAUDE.md "Host-GPU GL push": the GL destination is to call glr_submit() IN-PROCESS here,
 * exactly as linux-user/irix/glbridge.c already does for the user-mode path — dlopen the
 * renderer .so ($GLRENDER_SO) and drive host OpenGL directly, deleting the external-glserver
 * socket hop. Keeping GL/X11 in a dlopen'd .so keeps it entirely OUT of qemu's target build
 * (qemu only dlopen's it, never links -lGL/-lX11). The GLX context lives on the device-I/O
 * thread under the BQL — the same single-threaded model as hw/display/virtio-gpu-virgl.c —
 * created lazily on the first in-process submit.
 *
 * Mirrors struct glr_result / the glr_* ABI from sgi_glremote/userland/glrender.h (we don't
 * include that header to avoid pulling GL types into the target build; the layout is fixed).
 * ============================================================================ */
struct glr_result_abi {
    long          retval;
    unsigned int  wb_addr;
    int           wb_len;
    unsigned char wb_data[64];
    unsigned int  wb2_addr;
    int           wb2_len;
    unsigned char wb2_data[16];
};
typedef int  (*glr_submit_fn)(const unsigned char *, int, struct glr_result_abi *);
typedef void (*glr_set_ctxid_fn)(int);
typedef int  (*glr_get_last_frame_fn)(int, const unsigned char **, int *, int *);
typedef void (*glr_free_ctx_fn)(int);
typedef int  (*glr_get_dmabuf_fn)(int ctxid, int *w, int *h, int *stride,
                                  int *offset, int *fourcc, uint64_t *modifier);

static glr_submit_fn          gi_submit;
static glr_set_ctxid_fn        gi_set_ctxid;
static glr_get_last_frame_fn   gi_get_last_frame;
static glr_get_dmabuf_fn       gi_get_dmabuf;
static glr_free_ctx_fn         gi_free_ctx;      /* BL-73: optional per-ctx teardown */
static int                     gi_tried;

/* dlopen the renderer once (mirror glbridge.c::load_renderer). Returns true if usable. */
static bool glaccel_inproc_load(void)
{
    const char *path;
    void *h;
    if (gi_tried) {
        return gi_submit != NULL;
    }
    gi_tried = 1;
    path = getenv("GLRENDER_SO");
    if (!path || !*path) {
        path = "libglrender.so";
    }
    h = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
    if (!h) {
        fprintf(stderr, "sgi-glaccel: in-process dlopen(%s) failed: %s\n", path, dlerror());
        return false;
    }
    gi_submit         = (glr_submit_fn)dlsym(h, "glr_submit");
    gi_set_ctxid      = (glr_set_ctxid_fn)dlsym(h, "glr_set_ctxid");
    gi_get_last_frame = (glr_get_last_frame_fn)dlsym(h, "glr_get_last_frame");
    gi_get_dmabuf     = (glr_get_dmabuf_fn)dlsym(h, "glr_get_dmabuf");
    gi_free_ctx       = (glr_free_ctx_fn)dlsym(h, "glr_free_ctx");   /* BL-73; NULL on old .so */
    if (!gi_submit || !gi_get_last_frame) {
        fprintf(stderr, "sgi-glaccel: in-process renderer missing glr_submit/glr_get_last_frame\n");
        gi_submit = NULL;
        return false;
    }
    fprintf(stderr, "sgi-glaccel: IN-PROCESS GL path active (renderer %s)\n", path);
    return true;
}

/*
 * Return bytes per pixel for the current format.
 */
static int glaccel_bpp(SGIGLAccelState *s)
{
    switch (s->format) {
    case GLACCEL_FMT_RGB565:
        return 2;
    case GLACCEL_FMT_RGBA8888:
    default:
        return 4;
    }
}

/*
 * Return the stride (bytes per scanline). If the STRIDE register is 0,
 * compute from width * bpp.
 */
static uint32_t glaccel_stride(SGIGLAccelState *s)
{
    if (s->stride != 0) {
        return s->stride;
    }
    return s->width * glaccel_bpp(s);
}

/*
 * Convert one scanline from guest format to host PIXMAN_x8r8g8b8.
 */
static void glaccel_draw_line_rgba8888(void *opaque, uint8_t *dst,
                                        const uint8_t *src, int width,
                                        int pitch)
{
    /* Guest RGBA8888 (R,G,B,A bytes) → host xRGB (PIXMAN_x8r8g8b8) */
    for (int x = 0; x < width; x++) {
        uint8_t r = src[0];
        uint8_t g = src[1];
        uint8_t b = src[2];
        /* A byte ignored */
        *(uint32_t *)dst = (r << 16) | (g << 8) | b;
        src += 4;
        dst += 4;
    }
}

static void glaccel_draw_line_rgb565(void *opaque, uint8_t *dst,
                                      const uint8_t *src, int width,
                                      int pitch)
{
    /* Guest RGB565 big-endian → host xRGB */
    for (int x = 0; x < width; x++) {
        uint16_t pixel = (src[0] << 8) | src[1];
        uint8_t r = ((pixel >> 11) & 0x1f) << 3;
        uint8_t g = ((pixel >> 5) & 0x3f) << 2;
        uint8_t b = (pixel & 0x1f) << 3;
        *(uint32_t *)dst = (r << 16) | (g << 8) | b;
        src += 2;
        dst += 4;
    }
}

/* ============================================================
 * Live GL frame channel (socket): the host renderer (glserver) connects and streams
 * PVGL frames; the device composites the latest into its console. This is the paravirtual
 * GL "compositing" path — the guest GL is rendered on the host GPU and presented here in
 * the guest's framebuffer.  Protocol (LE): "PVGL"(4) + i32 x,y,w,h ; then w*h*4 RGBA.
 * ============================================================ */
/* BL-76 recomposite-on-delivery: force the compositor to run + present now.
 * Scheduled from glaccel_apply_frame (the single frame-receipt chokepoint) so a
 * frame that lands while the desktop is idle (or under -display none) still
 * reaches the display surface without waiting for a passive gfx_update tick or
 * an external screendump/xrefresh.  Runs on the main loop; graphic_hw_update
 * invokes sgi_glaccel_update which composites and blits into the surface.  The
 * frame readback already happened on the doorbell thread, so this callback does
 * no GL work (pure CPU composite) and is safe off the render thread. */
static void glaccel_present_bh(void *opaque)
{
    SGIGLAccelState *s = opaque;
    if (s->con) {
        graphic_hw_update(s->con);
    }
}

/*
 * ============================================================
 * Note 29 — content-only dirty: the last blanket whole-desktop invalidator
 * ============================================================
 *
 * Notes 27/28 bounded the geometry repaint and the VC2/CMAP/RAMDAC invalidators,
 * and then MEASURED what was still driving 76 % of desktop renders onto the full
 * 1280x1024 path.  The answer (note 28 §3) was `force = s->invalidate` — set
 * unconditionally by the pvgpu content ops:
 *
 *   - a delivered GL frame (glaccel_apply_frame, BL-76 recomposite-on-delivery),
 *   - PVGPU_OP_GL (the submit that produces it, in-process),
 *   - the legacy 2D ops CLEAR / FILL / COPY / BLIT.
 *
 * None of those can alter a DESKTOP pixel:
 *
 *   - a GL frame changes pixels only inside its window's rect, which the
 *     composite draws over s->desk every frame and the bounded blit copies; the
 *     composite signature folds c->frame_serial, so a new frame already defeats
 *     the idle-skip with no invalidate at all.  This is exactly the argument note
 *     28 used to drop the PVGPU_OP_PRESENT invalidate, applied one op earlier.
 *   - the 2D ops write s->fb, and s->fb is scanned out ONLY by the legacy
 *     no-desktop-layer path at the bottom of sgi_glaccel_update().  With a
 *     desktop layer registered (i.e. always, on virtuix) they cannot change a
 *     displayed pixel at all, so the bounded request they owe is the empty set.
 *     (No guest userland emits them today: the shim's ops are OP_GL/WM_*.  They
 *     survive as the device's own 2D console path.)
 *
 * GEOMETRY changes are a different matter and are NOT touched here: they go
 * through BL-81's bounded desk_repaint (or full, when unnameable) in the update
 * path, and a context whose frame vanishes is caught by the geometry set-diff.
 *
 * Fail-safe direction: with no desktop layer registered, or under
 * PVDISPLAY_CONTENT_FULL=1 (the one-binary A/B control), every site keeps the
 * old blanket invalidate.  The socket (non-inproc) frame path also keeps it,
 * because only the in-process path advances c->frame_serial — see the
 * serial_tracked argument.
 */
static bool glaccel_content_full(void)
{
    static int cf = -1;
    if (cf < 0) {
        cf = getenv("PVDISPLAY_CONTENT_FULL") ? 1 : 0;
    }
    return cf != 0;
}

typedef enum {
    GLACCEL_CD_FRAME,   /* a delivered GL frame */
    GLACCEL_CD_GLOP,    /* PVGPU_OP_GL submit */
    GLACCEL_CD_2D       /* CLEAR / FILL / COPY / BLIT into s->fb */
} GlaccelCdSite;

static void glaccel_content_dirty(SGIGLAccelState *s, GlaccelCdSite site,
                                  bool serial_tracked)
{
    switch (site) {
    case GLACCEL_CD_FRAME: s->n_inv_frame++; break;
    case GLACCEL_CD_GLOP:  s->n_inv_glop++;  break;
    default:               s->n_inv_2d++;    break;
    }
    if (!s->desk_render || glaccel_content_full() ||
        (site == GLACCEL_CD_FRAME && !serial_tracked)) {
        s->invalidate = true;
        return;
    }
    s->n_inv_dropped++;
}

static void glaccel_apply_frame(PVGPUCtx *c, int x, int y, int w, int h,
                                const uint8_t *rgba, bool serial_tracked)
{
    size_t npix = (size_t)w * h, i;
    uint32_t *b;
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096) {
        return;
    }
    b = g_new0(uint32_t, npix);
    for (i = 0; i < npix; i++) {
        b[i] = ((uint32_t)rgba[i * 4] << 16) |
               ((uint32_t)rgba[i * 4 + 1] << 8) | rgba[i * 4 + 2];
    }
    g_free(c->frame);
    c->frame = b;
    c->x = x; c->y = y; c->w = w; c->h = h;
    c->active = true;
    c->idle_warned = false;                /* fresh frame — re-arm the idle diagnostic */
    c->last_us = g_get_monotonic_time();   /* for the exit/idle staleness diagnostic */
    if (c->dev) {
        /* Note 29: a frame changes pixels only inside this window's rect; the
         * composite + bounded blit cover it and frame_serial defeats the
         * idle-skip.  Only the socket path (serial_tracked=false) still needs the
         * whole-desktop invalidate. */
        glaccel_content_dirty(c->dev, GLACCEL_CD_FRAME, serial_tracked);
        /* BL-76: delivery drives the composite — force a present on the main loop
         * so an idle desktop / -display none still shows this frame.  Coalesced. */
        if (c->dev->present_bh) qemu_bh_schedule(c->dev->present_bh);
    }

    /* verification hook: dump the composited frame to a PPM. Default overwrites each frame;
     * with SGI_GLACCEL_DUMP_FRAME=N it is a one-shot at the Nth composited frame, giving a
     * deterministic STEADY frame (avoids capturing a partial teardown frame at app exit). */
    {
        const char *dp = getenv("SGI_GLACCEL_DUMP");
        const char *dfp = getenv("SGI_GLACCEL_DUMP_FRAME");
        static int dumpn;
        bool do_dump = dp && (!dfp || dumpn == atoi(dfp));
        dumpn++;
        if (do_dump) {
            FILE *f = fopen(dp, "wb");
            if (f) {
                fprintf(f, "P6\n%d %d\n255\n", w, h);
                for (i = 0; i < npix; i++) {
                    uint8_t px[3] = { (b[i] >> 16) & 0xff, (b[i] >> 8) & 0xff, b[i] & 0xff };
                    fwrite(px, 1, 3, f);
                }
                fclose(f);
            }
        }
    }
}

static void glaccel_gl_parse(PVGPUCtx *c)
{
    GByteArray *rx = c->rxbuf;
    while (rx->len >= 20) {
        const uint8_t *p = rx->data;
        int32_t x, y, w, h;
        size_t payload, total;
        if (memcmp(p, "PVGL", 4) != 0) {
            g_byte_array_remove_range(rx, 0, 1);   /* resync */
            continue;
        }
        x = ldl_le_p(p + 4); y = ldl_le_p(p + 8);
        w = ldl_le_p(p + 12); h = ldl_le_p(p + 16);
        if (w == 0) {                              /* clear */
            c->active = false; if (c->dev) c->dev->invalidate = true;
            g_byte_array_remove_range(rx, 0, 20);
            continue;
        }
        payload = (size_t)w * h * 4;
        total = 20 + payload;
        if (rx->len < total) {
            break;                                 /* wait for the rest */
        }
        /* socket path: nothing advances c->frame_serial here, so this delivery
         * still needs the whole-desktop invalidate (debug fallback path only). */
        glaccel_apply_frame(c, x, y, w, h, rx->data + 20, false);
        g_byte_array_remove_range(rx, 0, total);
    }
}

static void glaccel_gl_read(void *opaque)
{
    PVGPUCtx *c = opaque;
    uint8_t buf[65536];
    ssize_t n = recv(c->conn_fd, (void *)buf, sizeof(buf), 0);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        }
        qemu_set_fd_handler(c->conn_fd, NULL, NULL, NULL);
        close(c->conn_fd);
        c->conn_fd = -1;
        return;
    }
    g_byte_array_append(c->rxbuf, buf, n);
    glaccel_gl_parse(c);
}

/* A renderer connected to the one gl-listen socket; the 4-byte context-id handshake it sends
 * first routes the connection to its window's PVGPUCtx (multiple windows share the socket). */
static void glaccel_gl_accept(void *opaque)
{
    SGIGLAccelState *s = opaque;
    int fd = accept(s->gl_listen_fd, NULL, NULL);
    uint8_t hs[4]; uint32_t cid; PVGPUCtx *c;
    struct timeval tv = { 2, 0 };
    if (fd < 0) {
        return;
    }
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    if (recv(fd, hs, 4, MSG_WAITALL) != 4) { close(fd); return; }
    cid = ((uint32_t)hs[0] << 24) | ((uint32_t)hs[1] << 16) | ((uint32_t)hs[2] << 8) | hs[3];
    if (cid >= PVGPU_MAXCTX) { close(fd); return; }
    c = &s->ctx[cid];
    if (c->conn_fd >= 0) {                          /* one renderer per window */
        qemu_set_fd_handler(c->conn_fd, NULL, NULL, NULL);
        close(c->conn_fd);
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    c->conn_fd = fd;
    if (c->rxbuf == NULL) {
        c->rxbuf = g_byte_array_new();
    } else {
        g_byte_array_set_size(c->rxbuf, 0);
    }
    qemu_set_fd_handler(fd, glaccel_gl_read, NULL, c);
}

static void glaccel_gl_listen(SGIGLAccelState *s, int port, Error **errp)
{
    int fd, on = 1;
    struct sockaddr_in addr;
    fd = qemu_socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        error_setg_errno(errp, errno, "glaccel gl-listen: socket");
        return;
    }
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(fd, 1) < 0) {
        error_setg_errno(errp, errno, "glaccel gl-listen: bind/listen %d", port);
        close(fd);
        return;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    s->gl_listen_fd = fd;
    qemu_set_fd_handler(fd, glaccel_gl_accept, NULL, s);
    qemu_log("sgi-glaccel: GL frame channel listening on 127.0.0.1:%d\n", port);
}

/* ============================================================
 * Paravirtual GPU command ring: the guest submits a buffer of 2D/3D commands (DMA'd from
 * guest RAM on the EXEC doorbell); the device executes them into its internal framebuffer.
 * 2D ops are host memcpy/memset (no per-pixel REX3 emulation); GL ops are forwarded to the
 * host renderer (glserver) whose frame arrives via the gl-listen channel and composites in.
 * ============================================================ */
static uint32_t pv_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void pvgpu_ensure_fb(SGIGLAccelState *s, int w, int h) {
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096) {
        return;
    }
    if (!s->fb || s->fb_w != w || s->fb_h != h) {
        g_free(s->fb);
        s->fb = g_new0(uint32_t, (size_t)w * h);
        s->fb_w = w; s->fb_h = h;
    }
}
/* clip a rect to the framebuffer */
static bool pv_clip(SGIGLAccelState *s, int *x, int *y, int *w, int *h) {
    if (*x < 0) { *w += *x; *x = 0; }
    if (*y < 0) { *h += *y; *y = 0; }
    if (*x + *w > s->fb_w) { *w = s->fb_w - *x; }
    if (*y + *h > s->fb_h) { *h = s->fb_h - *y; }
    return (*w > 0 && *h > 0);
}
/* Fixed-format GL response the device DMAs back into the ring after an OP_GL, for the guest
 * libgl shim to read (big-endian = guest native). Written at cmd_base + align16(cmd_len).
 * Mirrors glserver's reply: a return value plus up to two pointer writebacks (getmatrix,
 * getsize, qread, ...). The shim applies the writebacks to its own pointers. */
#define PVGPU_RESP_MAGIC 0x50565247u            /* "PVRG" */
#define PVGPU_WB_MAX  64
#define PVGPU_WB2_MAX 16
#define PVGPU_RESP_BYTES (4 + 8 + 4 + 4 + PVGPU_WB_MAX + 4 + 4 + PVGPU_WB2_MAX)  /* 108 */

/* BL-71 defense-in-depth: allocate every DMA'd command buffer with a zeroed slack
 * tail. The in-process renderer executes pixel-upload ops (glTexImage2D/DrawPixels)
 * that read w*h*bpp bytes of source data straight out of this buffer; a guest that
 * leaves a stale GL_UNPACK_ROW_LENGTH set makes the driver read a larger strided
 * span than the payload. The renderer now forces tight unpack (the real fix), but
 * a zeroed slack margin turns any residual/future over-read of a few KB into a
 * harmless read of zeros instead of a SEGV that wedges the whole VM. */
#define PVGPU_CMD_SLACK 65536u
typedef struct {
    int64_t  retval;
    uint32_t wb_addr, wb_len;   uint8_t wb_data[PVGPU_WB_MAX];
    uint32_t wb2_addr, wb2_len; uint8_t wb2_data[PVGPU_WB2_MAX];
    bool     ok;
} PvgpuResp;

static void pv_wbe32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }

/* Drain the gl-listen frame channel ONCE without blocking: accept a pending renderer
 * connection and read+parse any available frame bytes. Normally driven by the main loop,
 * but pvgpu_gl_submit also calls it directly because it blocks the BQL-holding vCPU while
 * waiting for glserver's reply — and glserver streams the (large) frame to gl-listen
 * *before* sending that reply, so we must keep gl-listen drained or the two deadlock. */
static void pvgpu_drain_frames(SGIGLAccelState *s, PVGPUCtx *c) {
    if (s->gl_listen_fd >= 0) glaccel_gl_accept(s);   /* route any pending renderer by ctx-id */
    if (c->conn_fd >= 0) glaccel_gl_read(c);          /* one recv + parse for this window */
}

/* send n bytes on the (non-blocking) forward fd, draining this window's frames if it blocks */
static int pvgpu_send_all(SGIGLAccelState *s, PVGPUCtx *c, const uint8_t *p, int n) {
    int put = 0;
    while (put < n) {
        int r = send(c->fwd_fd, p + put, n - put, 0);
        if (r > 0) { put += r; continue; }
        if (r < 0 && errno == EINTR) continue;
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pf[2]; int np = 0, ci = -1;
            pf[np].fd = c->fwd_fd; pf[np].events = POLLOUT; np++;
            if (c->conn_fd >= 0) { pf[np].fd = c->conn_fd; pf[np].events = POLLIN; ci = np; np++; }
            if (poll(pf, np, 1000) > 0 && ci >= 0 && (pf[ci].revents & POLLIN))
                glaccel_gl_read(c);
            continue;
        }
        return -1;
    }
    return 0;
}

/* ⚠️ SCAFFOLDING (CLAUDE.md "Host-GPU GL push — roadmap"): this whole forward/gl-listen
 * socket plumbing to the EXTERNAL glserver process is the incremental transport. The GL
 * end-goal is to call glr_submit() IN-PROCESS here (virtio-gpu/virgl-shaped, like
 * linux-user/irix/glbridge.c does for user-mode) and DMA the reply to guest RAM — deleting
 * this socket hop. Kept as the validated stepping stone; the PVGPUCtx routing/compositing
 * carries over.
 *
 * lazy-connect this window's forward channel to glserver, sending the 4-byte context-id
 * handshake so glserver tags the frames it streams back (the device routes them by ctx). */
static int pvgpu_gl_connect(SGIGLAccelState *s, PVGPUCtx *c) {
    if (!s->gl_forward || !s->gl_forward[0]) return -1;
    if (c->fwd_fd < 0) {
        struct sockaddr_in a; uint8_t hs[4]; int fd, cid = (int)(c - s->ctx);
        fd = qemu_socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        memset(&a, 0, sizeof a); a.sin_family = AF_INET;
        a.sin_port = htons(atoi(s->gl_forward));
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
        hs[0] = cid >> 24; hs[1] = cid >> 16; hs[2] = cid >> 8; hs[3] = cid;
        if (send(fd, hs, 4, 0) != 4) { close(fd); return -1; }   /* ctx-id handshake */
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
        c->fwd_fd = fd; c->pending = 0;
    }
    return c->fwd_fd;
}

/* ASYNC send: forward a GL buffer to glserver fire-and-forget. glserver still replies to
 * every buffer, so the reply count accrues (c->pending) to be drained by the next query —
 * this is the key to speed: void flushes (geometry, swapbuffers) never block the guest CPU. */
static int pvgpu_gl_send(SGIGLAccelState *s, PVGPUCtx *c, const uint8_t *gl, uint32_t gllen) {
    uint8_t hdr[4];
    if (pvgpu_gl_connect(s, c) < 0) return -1;
    hdr[0] = gllen >> 24; hdr[1] = gllen >> 16; hdr[2] = gllen >> 8; hdr[3] = gllen;
    if (pvgpu_send_all(s, c, hdr, 4) < 0) goto drop;
    if (gllen && pvgpu_send_all(s, c, gl, gllen) < 0) goto drop;
    c->pending++;
    return 0;
drop:
    if (c->fwd_fd >= 0) { close(c->fwd_fd); c->fwd_fd = -1; }
    c->pending = 0;
    return -1;
}

/* Drain this window's accrued replies (discarding all but the last, which belongs to the
 * just-sent query), keeping its frame channel drained throughout so frames flow + no deadlock. */
static void pvgpu_gl_drain(SGIGLAccelState *s, PVGPUCtx *c, PvgpuResp *r) {
    uint8_t rb[4096];   /* holds many accrued replies; each is <=108 bytes, parsed in order */
    int rlen = 0;
    time_t deadline = time(NULL) + 15;
    memset(r, 0, sizeof *r);
    if (c->fwd_fd < 0 || c->pending <= 0) { c->pending = 0; return; }
    for (;;) {
        struct pollfd pf[3]; int np = 0, fi, ci = -1, li = -1;
        pf[np].fd = c->fwd_fd; pf[np].events = POLLIN; fi = np++;
        if (c->conn_fd >= 0) { pf[np].fd = c->conn_fd; pf[np].events = POLLIN; ci = np++; }
        else if (s->gl_listen_fd >= 0) { pf[np].fd = s->gl_listen_fd; pf[np].events = POLLIN; li = np++; }
        if (poll(pf, np, 200) < 0) { if (errno == EINTR) continue; goto drop; }
        if (li >= 0 && (pf[li].revents & POLLIN)) pvgpu_drain_frames(s, c);
        else if (ci >= 0 && (pf[ci].revents & POLLIN)) glaccel_gl_read(c);
        if (pf[fi].revents & (POLLIN | POLLHUP)) {
            int got = recv(c->fwd_fd, rb + rlen, (int)sizeof(rb) - rlen, 0);
            if (got > 0) rlen += got;
            else if (got == 0) goto drop;
            else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) goto drop;
        }
        while (rlen >= 16) {                            /* parse complete replies in order */
            uint32_t wbl = pv_be32(rb + 12);
            uint32_t off, wb2l, total;
            if (wbl > PVGPU_WB_MAX) goto drop;
            if (rlen < (int)(16 + wbl + 8)) break;
            off = 16 + wbl; wb2l = pv_be32(rb + off + 4);
            if (wb2l > PVGPU_WB2_MAX) goto drop;
            total = off + 8 + wb2l;
            if (rlen < (int)total) break;
            r->retval = (int64_t)(((uint64_t)pv_be32(rb) << 32) | pv_be32(rb + 4));
            r->wb_addr = pv_be32(rb + 8); r->wb_len = wbl; memcpy(r->wb_data, rb + 16, wbl);
            r->wb2_addr = pv_be32(rb + off); r->wb2_len = wb2l; memcpy(r->wb2_data, rb + off + 8, wb2l);
            r->ok = true;
            memmove(rb, rb + total, rlen - total); rlen -= total;
            if (--c->pending <= 0) { c->pending = 0; return; }
        }
        if (rlen > (int)sizeof(rb) - 256) goto drop;   /* desync guard */
        if (time(NULL) > deadline) {
            fprintf(stderr, "pvgpu: GL drain timeout (pending=%d)\n", c->pending);
            c->pending = 0; return;
        }
    }
drop:
    if (c->fwd_fd >= 0) { close(c->fwd_fd); c->fwd_fd = -1; }
    c->pending = 0;
}

/* serialize a GL reply into the guest-native (big-endian) wire layout the shim reads */
static void pvgpu_pack_resp(uint8_t *o, const PvgpuResp *r) {
    pv_wbe32(o, r->ok ? PVGPU_RESP_MAGIC : 0);
    pv_wbe32(o + 4, (uint32_t)((uint64_t)r->retval >> 32));
    pv_wbe32(o + 8, (uint32_t)r->retval);
    pv_wbe32(o + 12, r->wb_addr); pv_wbe32(o + 16, r->wb_len);
    memcpy(o + 20, r->wb_data, PVGPU_WB_MAX);
    pv_wbe32(o + 84, r->wb2_addr); pv_wbe32(o + 88, r->wb2_len);
    memcpy(o + 92, r->wb2_data, PVGPU_WB2_MAX);
}

/* IN-PROCESS GL submit (the virtio-gpu/virgl-shaped END-GOAL, NOT scaffolding): hand this
 * window's GL command buffer straight to the dlopen'd renderer's glr_submit() — no glserver
 * socket — then pull the just-rendered frame via glr_get_last_frame() and composite it through
 * the SAME glaccel_apply_frame() the gl-listen socket path uses (PVGPUCtx routing, console +
 * desktop-overlay compositing all carry over unchanged). Fills *resp from any query writeback
 * so the doorbell path can DMA the reply back to the guest exactly like the socket path does.
 * Runs on the device-I/O thread under the BQL (single-threaded, like virtio-gpu-virgl). */
static void pvgpu_gl_submit_inproc(SGIGLAccelState *s, PVGPUCtx *c,
                                   const uint8_t *gl, uint32_t gllen,
                                   bool need_reply, PvgpuResp *resp)
{
    struct glr_result_abi r;
    const unsigned char *rgb = NULL;
    int fw = 0, fh = 0, cid = (int)(c - s->ctx), serial;

    /* Thread-migration detection: log if submits arrive on different host threads.
     * glXMakeContextCurrent is thread-affine; under MTTCG a doorbell on a different
     * vCPU thread would silently no-op.  This counter tells us if migration happens
     * in practice before we commit to a dedicated render thread. */
    {
        static pthread_t last_thread;
        static int warned;
        pthread_t cur = pthread_self();
        if (last_thread && !pthread_equal(cur, last_thread) && !warned) {
            fprintf(stderr, "pvgpu: THREAD MIGRATION — GL submit on ctx %d moved from "
                    "thread %lu to %lu (GLX context may be stale on old thread)\n",
                    cid, (unsigned long)last_thread, (unsigned long)cur);
            warned = 1;  /* log once per session — it either happens or it doesn't */
        }
        last_thread = cur;
    }

    if (resp) {
        memset(resp, 0, sizeof *resp);
    }
    if (!glaccel_inproc_load()) {
        return;
    }
    if (gi_set_ctxid) {
        gi_set_ctxid(cid);          /* route any PVGL side-effects + per-ctx frame to this window */
    }
    memset(&r, 0, sizeof r);
    if (gllen) {
        gi_submit(gl, (int)gllen, &r);
    }

    /* ---- GPU scanout (steps ③-④): try DMABUF zero-copy path first ----
     * If the renderer exported a DMABUF, use QEMU's dpy_gl_scanout_dmabuf to
     * display it directly from the GPU — no CPU readback, no RGB→RGBA conversion.
     * Falls back to the CPU readback path when DMABUF is unavailable.            */
    serial = gi_get_last_frame(cid, &rgb, &fw, &fh);  /* always get serial for gating */
    if (serial && serial != c->frame_serial) {
        int dmabuf_fd = -1;
        if (gi_get_dmabuf) {
            int dmabuf_w, dmabuf_h, dmabuf_stride, dmabuf_offset, dmabuf_fourcc;
            uint64_t dmabuf_modifier;
            dmabuf_fd = gi_get_dmabuf(cid, &dmabuf_w, &dmabuf_h,
                        &dmabuf_stride, &dmabuf_offset,
                        &dmabuf_fourcc, &dmabuf_modifier);
            if (dmabuf_fd >= 0) {
                /* Got a DMABUF fd — hand it to QEMU's display for GPU scanout.
                 * The display listener owns the fd after this call (closes it). */
                QemuDmaBuf *dmabuf = qemu_dmabuf_new(
                    dmabuf_w, dmabuf_h,
                    (const uint32_t[]){dmabuf_offset},
                    (const uint32_t[]){dmabuf_stride},
                    0, 0, dmabuf_w, dmabuf_h,
                    dmabuf_fourcc, dmabuf_modifier,
                    (const int32_t[]){dmabuf_fd}, 1,
                    false, true);
                if (dmabuf) {
                    /* scan out to every registered display listener */
                    QemuConsole *con = s->con;
                    if (con) {
                        dpy_gl_scanout_dmabuf(con, dmabuf);
                    }
                    qemu_dmabuf_free(dmabuf);
                } else {
                    close(dmabuf_fd);  /* qemu_dmabuf_new failed — close our ref */
                }
                c->frame_serial = serial;
                goto dmabuf_done;
            }
        }
        /* CPU fallback: readback + RGB→RGBA conversion + software composite */
        if (rgb && fw > 0 && fh > 0) {
            c->frame_serial = serial;
            uint8_t *rgba = g_malloc((size_t)fw * fh * 4);
            size_t i, n = (size_t)fw * fh;
            for (i = 0; i < n; i++) {
                rgba[i * 4 + 0] = rgb[i * 3 + 0];
                rgba[i * 4 + 1] = rgb[i * 3 + 1];
                rgba[i * 4 + 2] = rgb[i * 3 + 2];
                rgba[i * 4 + 3] = 0xff;
            }
            /* in-process path: c->frame_serial was advanced just above, so the
             * composite signature sees this frame without a full invalidate. */
            glaccel_apply_frame(c, c->x, c->y, fw, fh, rgba, true);
            g_free(rgba);
        }
    }
    dmabuf_done:

    /* mirror the socket path's query writeback so getmatrix/getsize/qread DMA back correctly */
    if (need_reply && resp) {
        resp->ok = true;
        resp->retval = r.retval;
        resp->wb_addr = r.wb_addr;
        resp->wb_len  = (r.wb_len > 0 && r.wb_len <= PVGPU_WB_MAX) ? r.wb_len : 0;
        if (resp->wb_len) {
            memcpy(resp->wb_data, r.wb_data, resp->wb_len);
        }
        resp->wb2_addr = r.wb2_addr;
        resp->wb2_len  = (r.wb2_len > 0 && r.wb2_len <= PVGPU_WB2_MAX) ? r.wb2_len : 0;
        if (resp->wb2_len) {
            memcpy(resp->wb2_data, r.wb2_data, resp->wb2_len);
        }
    }
}

/* ---- window-model commit batching ---------------------------------------------
 * The ON arm of the interaction A/B cost +17.7 points of host CPU and 3x resize
 * latency (24-…md §3).  THREE causes, not the two the note named:
 *   (a) every WM_END scheduled a full desktop re-composite, and wm_gen was folded
 *       into the composite signature so the idle-skip could never suppress it
 *       => present only when a window some context is BOUND to actually changed,
 *          and fold wm_sig (bumped only then) instead of wm_gen;
 *   (b) one present per publish, and 4Dwm publishes per validate/move/resize
 *       => coalesce to at most one model-driven present per display refresh;
 *   (c) [found while landing a+b, and the BIGGER one] sgi_glaccel_update()'s
 *       geometry-change test built its rects from the SUBMIT-time origin while
 *       prev_win_rects[] came from the composite's MODEL positions, so
 *       geom_changed — and therefore a full desktop re-render plus a full-screen
 *       blit — was true on EVERY composite while a model was live.  See the
 *       comment at that site.
 * None of the three touches the resolution rules: the model STATE is still
 * committed synchronously and wholesale on every good batch, so the composite
 * that eventually runs always reads the FINAL state.
 *
 * BL-81 (the fourth, and what blocked default-on): with a GL context bound to an
 * on-screen window the remaining cost was not present COUNT — the coalescer had
 * that at one per refresh already — but per-present WORK.  Both a model commit
 * (`s->invalidate = true` at WM_END) and the geometry-change test forced
 * desk_render(force_full) => a 1280x1024 Newport re-walk plus a 5 MB full-screen
 * memcpy + dpy_gfx_update_full, on every present of a move/resize storm: +7.5
 * points of host CPU on resize, +6.6 on move (26-…md §2).  Both now name the
 * rects the change can have altered — the union of each changed window's old and
 * new rect — and hand them to the desktop layer's dirty-rect machinery
 * (desk_repaint), so the render AND the blit are bounded to the window.  Every
 * "cannot name it" case (first model, publisher death, bound-window retire, rect
 * list overflow, no desk_repaint implementation) falls back to the old full path.
 * ------------------------------------------------------------------------------ */

/* One display refresh (the pvgpu console runs at 60Hz). */
#define GLACCEL_WM_PRESENT_PERIOD_NS (NANOSECONDS_PER_SECOND / 60)

static const PVGPUWinRec *glaccel_wm_lookup(const PVGPUWinRec *v, int n, uint32_t xid)
{
    int i;
    for (i = 0; i < n; i++) {
        if (v[i].xid == xid) return &v[i];
    }
    return NULL;
}

/* Does this record differ in any way the composite READS?  Fields the composite
 * ignores (PVGPU_WMF_OVERRIDE, HASCLIP) are deliberately excluded; everything
 * glaccel_composite_overlays() consumes — origin, size, stacking key, MAPPED,
 * and the clip region itself — is compared.  A NULL on exactly one side is a
 * liveness change (the window appeared or was retired) and counts as differing. */
static bool glaccel_wm_rec_differs(const PVGPUWinRec *a, const PVGPUWinRec *b)
{
    int i;
    if (!a || !b) return a != b;
    if (a->x != b->x || a->y != b->y || a->w != b->w || a->h != b->h) return true;
    if (a->stack != b->stack) return true;
    if ((a->flags & PVGPU_WMF_MAPPED) != (b->flags & PVGPU_WMF_MAPPED)) return true;
    if (a->n_clip != b->n_clip) return true;
    for (i = 0; i < a->n_clip && i < PVGPU_MAX_WM_CLIP; i++) {
        if (a->clip[i][0] != b->clip[i][0] || a->clip[i][1] != b->clip[i][1] ||
            a->clip[i][2] != b->clip[i][2] || a->clip[i][3] != b->clip[i][3]) return true;
    }
    return false;
}

static void glaccel_clamp_rect(PVDeskRect *out, int x, int y, int wpx, int hpx,
                               int dw, int dh);
static const PVGPUWinRec *glaccel_wm_find(SGIGLAccelState *s, uint32_t xid);

/* BL-81 — accumulate one rect into the pending bounded-repaint request.
 * Clamped to the desktop, exact-duplicate-suppressed, and saturating to
 * "cannot be bounded" when the fixed list fills.  Consumed (and cleared) by the
 * next sgi_glaccel_update(); it must accumulate ACROSS commits because the
 * coalescer folds many WM_ENDs into one present. */
static void glaccel_wm_add_rrect(SGIGLAccelState *s, int x, int y, int w, int h)
{
    PVDeskRect r;
    int i;
    if (w <= 0 || h <= 0 || s->desk_w <= 0 || s->desk_h <= 0) {
        return;
    }
    glaccel_clamp_rect(&r, x, y, w, h, s->desk_w, s->desk_h);
    if (r.w <= 0 || r.h <= 0) return;
    for (i = 0; i < s->wm_n_rrects; i++) {
        if (s->wm_rrects[i].x == r.x && s->wm_rrects[i].y == r.y &&
            s->wm_rrects[i].w == r.w && s->wm_rrects[i].h == r.h) {
            return;
        }
    }
    if (s->wm_n_rrects >= (int)ARRAY_SIZE(s->wm_rrects)) {
        s->wm_repaint_full = true;   /* out of slots — name nothing, repaint all */
        return;
    }
    s->wm_rrects[s->wm_n_rrects++] = r;
}

/* BL-81 — the region one changed window record can have altered, at one
 * generation: the window rect the server published UNION the rect the frame
 * actually occupies (the ctx frame may be larger or smaller than the record's
 * w/h, and the composite uses the ctx size with the model origin). */
static void glaccel_wm_add_win_rrect(SGIGLAccelState *s, int ox, int oy,
                                     int rw, int rh, int cw, int ch)
{
    int x1 = ox + (rw > cw ? rw : cw);
    int y1 = oy + (rh > ch ? rh : ch);
    glaccel_wm_add_rrect(s, ox, oy, x1 - ox, y1 - oy);
}

/* Would committing wm_shadow[] change a pixel?  Only through a context's bound_xid:
 * glaccel_wm_find() is keyed on it, so a window no context is bound to is never read
 * at composite time and its record moving cannot alter the output.  4Dwm republishes
 * its whole tree on every PostValidateTree, so the overwhelmingly common batch is one
 * where every BOUND window is bit-identical — that batch must be free.
 *
 * Deliberately conservative in the "unknown" directions (risk 1 discipline: degrade to
 * no-model, never to a stale guess): the first model after "none" always counts as a
 * change, and any single differing field short-circuits to true. */
static bool glaccel_wm_bound_changed(SGIGLAccelState *s)
{
    int k;
    bool changed = false;
    if (s->wm_gen == 0) {
        s->wm_repaint_full = true;      /* first model: nothing to bound against */
        return true;
    }
    for (k = 0; k < PVGPU_MAXCTX; k++) {
        uint32_t xid = s->ctx[k].bound_xid;
        const PVGPUWinRec *ow, *nw;
        if (xid == 0) continue;
        ow = glaccel_wm_lookup(s->wm, s->wm_n, xid);
        nw = glaccel_wm_lookup(s->wm_shadow, s->wm_shadow_n, xid);
        if (!glaccel_wm_rec_differs(ow, nw)) continue;
        changed = true;
        /*
         * BL-81.  Every field glaccel_wm_rec_differs() compares — origin, size,
         * stacking key, MAPPED, clip region — can only alter pixels inside the
         * window's OLD rect (which the change may uncover, restack under, or
         * newly clip away) or its NEW rect (where the frame now composites).
         * Outside their union the model change is provably invisible, so record
         * exactly that union as the bounded repaint request instead of the
         * whole-screen `s->invalidate = true` this used to imply.
         *
         * A NULL on one side is an appear/retire: the surviving rect is the whole
         * requirement (a window that did not exist has no old pixels to restore,
         * a retired one has no new ones), so neither needs the full path.
         */
        if (ow) {
            glaccel_wm_add_win_rrect(s, ow->x, ow->y, ow->w, ow->h,
                                     s->ctx[k].w, s->ctx[k].h);
        }
        if (nw) {
            glaccel_wm_add_win_rrect(s, nw->x, nw->y, nw->w, nw->h,
                                     s->ctx[k].w, s->ctx[k].h);
        }
    }
    return changed;
}

static void glaccel_wm_stats(SGIGLAccelState *s)
{
    static int on = -1;
    static int every = 64;
    if (on < 0) {
        const char *e = getenv("SGI_GLACCEL_WM_STATS");
        on = e ? 1 : 0;
        /* SGI_GLACCEL_WM_STATS=all reports EVERY commit.  The default 1-in-64 is
         * too coarse to attribute a single window op (BL-81's correctness probe
         * needs to see the geombounded/geomfull counters move for one move). */
        if (e && !strcmp(e, "all")) every = 1;
    }
    if (!on || (s->wm_commits % every) != 0) return;
    fprintf(stderr, "pvgpu: WMSTATS commits=%" PRIu64 " nop=%" PRIu64
            " presents=%" PRIu64 " coalesced=%" PRIu64 " gen=%u sig=%u wm_n=%d"
            " geombounded=%" PRIu64 " geomfull=%" PRIu64
            " vfy=%" PRIu64 " vfybad=%" PRIu64 "/%" PRIu64
            " upd=%" PRIu64 " t_render=%" PRIu64 " t_comp=%" PRIu64
            " t_blit=%" PRIu64 " t_wmops=%" PRIu64 " (ms)"
            " rfull=%" PRIu64 " rrects=%" PRIu64 " ridle=%" PRIu64 "\n",
            s->wm_commits, s->wm_commits_nop, s->wm_presents,
            s->wm_presents_coalesced, s->wm_gen, s->wm_sig, s->wm_n,
            s->geom_bounded_repaints, s->geom_full_repaints,
            s->verify_frames, s->verify_bad_frames, s->verify_bad_pixels,
            s->n_updates, s->t_desk_render / 1000000, s->t_composite / 1000000,
            s->t_blit / 1000000, s->t_wm_ops / 1000000,
            s->n_render_full, s->n_render_rects, s->n_render_idle);
}

static void glaccel_wm_present_timer_cb(void *opaque)
{
    SGIGLAccelState *s = opaque;
    s->wm_present_pending = false;
    s->wm_last_present_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    s->wm_presents++;
    if (s->present_bh) qemu_bh_schedule(s->present_bh);
}

/* Fix (b): coalesce.  If the last model-driven present was within one refresh, arm
 * (once) a timer for the end of that refresh window instead of presenting now.  The
 * commit already happened, so whenever the deferred present runs it composites the
 * newest model — a burst of N commits inside one refresh yields ONE composite of the
 * final state.  Nothing is dropped: the only thing rate-limited is the repaint. */
/* PVDISPLAY_TIME_BREAKDOWN=1 — per-phase host-time attribution (BL-81 §5). */
static bool glaccel_timing_on(void)
{
    static int on = -1;
    if (on < 0) on = getenv("PVDISPLAY_TIME_BREAKDOWN") ? 1 : 0;
    return on != 0;
}

static uint64_t glaccel_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

#define GLACCEL_TIME(s, field, body) do {                                     \
        if (glaccel_timing_on()) {                                            \
            uint64_t _t0 = glaccel_now_ns();                                  \
            body;                                                             \
            (s)->field += glaccel_now_ns() - _t0;                             \
        } else {                                                              \
            body;                                                             \
        }                                                                     \
    } while (0)

static int64_t glaccel_wm_present_period_ns(void)
{
    /* SGI_GLACCEL_WM_PRESENT_HZ=<n> overrides the one-per-refresh coalescing cap.
     * BL-81 attribution knob: if the model's residual host cost is per-PRESENT it
     * must fall roughly in proportion when this is lowered; if it is per-COMMIT
     * (ring parse + model commit + bound-changed compare, which happen 9600 times
     * either way) lowering it changes nothing.  Debug only. */
    static int64_t period = -1;
    if (period < 0) {
        const char *e = getenv("SGI_GLACCEL_WM_PRESENT_HZ");
        int hz = e ? atoi(e) : 0;
        period = (hz > 0) ? NANOSECONDS_PER_SECOND / hz
                          : GLACCEL_WM_PRESENT_PERIOD_NS;
    }
    return period;
}

static void glaccel_wm_request_present(SGIGLAccelState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    int64_t period = glaccel_wm_present_period_ns();
    if (now - s->wm_last_present_ns >= period || !s->wm_present_timer) {
        s->wm_last_present_ns = now;
        s->wm_presents++;
        if (s->present_bh) qemu_bh_schedule(s->present_bh);
        return;
    }
    s->wm_presents_coalesced++;
    if (!s->wm_present_pending) {
        s->wm_present_pending = true;
        timer_mod_ns(s->wm_present_timer, s->wm_last_present_ns + period);
    }
}

/* execute a command buffer into the internal framebuffer. resp_base != 0 => DMA each OP_GL's
 * reply back to guest RAM at resp_base + align16(len) (the doorbell path; 0 for cmd-file). */
static void pvgpu_exec(SGIGLAccelState *s, const uint8_t *buf, uint32_t len, uint32_t resp_base) {
    uint32_t p = 0;
    PVGPUCtx *wc = &s->ctx[s->cur_ctx < PVGPU_MAXCTX ? s->cur_ctx : 0];   /* this window */
    /* One stderr write per submitted command buffer on the hottest device path —
     * gated like its sibling in the doorbell handler (commit 1896ffea3c). */
    {
        static int exec_dbg = -1;
        if (exec_dbg < 0) {
            exec_dbg = getenv("SGI_GLACCEL_EXEC_DBG") ? 1 : 0;
        }
        if (exec_dbg) {
            fprintf(stderr, "pvgpu: exec ctx=%d %u bytes first_op=%u\n",
                    (int)(wc - s->ctx), len, len >= 4 ? pv_be32(buf) : 0);
        }
    }
    while (p + 4 <= len) {
        uint32_t op = pv_be32(buf + p); p += 4;
        /* BL-81 attribution: charge the WM_* ops (10..15) their own host time, so
         * "per-commit model work" can be separated from "per-present repaint work".
         * Error paths inside the switch `return` and are simply not charged. */
        uint64_t wm_t0 = (op >= PVGPU_OP_WM_BEGIN && op <= PVGPU_OP_WM_BIND
                          && glaccel_timing_on()) ? glaccel_now_ns() : 0;
        switch (op) {
        case PVGPU_OP_CLEAR: {
            uint32_t c; size_t i;
            if (p + 4 > len) return;
            c = pv_be32(buf + p); p += 4;
            if (!s->fb) pvgpu_ensure_fb(s, s->width ? s->width : 800, s->height ? s->height : 600);
            if (s->fb) for (i = 0; i < (size_t)s->fb_w * s->fb_h; i++)
                s->fb[i] = ((c >> 24 & 0xff) << 16) | ((c >> 16 & 0xff) << 8) | (c >> 8 & 0xff);
            s->fb_active = true;
            glaccel_content_dirty(s, GLACCEL_CD_2D, false); break;
        }
        case PVGPU_OP_FILL: {
            int x, y, w, h, yy, xx; uint32_t c, px;
            if (p + 20 > len) return;
            x = pv_be32(buf+p); y = pv_be32(buf+p+4); w = pv_be32(buf+p+8);
            h = pv_be32(buf+p+12); c = pv_be32(buf+p+16); p += 20;
            if (!s->fb) break;
            px = ((c >> 24 & 0xff) << 16) | ((c >> 16 & 0xff) << 8) | (c >> 8 & 0xff);
            if (pv_clip(s, &x, &y, &w, &h))
                for (yy = y; yy < y + h; yy++) for (xx = x; xx < x + w; xx++)
                    s->fb[(size_t)yy * s->fb_w + xx] = px;
            s->fb_active = true;
            glaccel_content_dirty(s, GLACCEL_CD_2D, false); break;
        }
        case PVGPU_OP_COPY: {
            int sx, sy, dx, dy, w, h, r;
            if (p + 24 > len) return;
            sx = pv_be32(buf+p); sy = pv_be32(buf+p+4); dx = pv_be32(buf+p+8);
            dy = pv_be32(buf+p+12); w = pv_be32(buf+p+16); h = pv_be32(buf+p+20); p += 24;
            if (!s->fb || w <= 0 || h <= 0) break;
            if (sx < 0 || sy < 0 || dx < 0 || dy < 0 ||
                sx + w > s->fb_w || sy + h > s->fb_h ||
                dx + w > s->fb_w || dy + h > s->fb_h) break;
            for (r = (dy > sy) ? h - 1 : 0; (dy > sy) ? r >= 0 : r < h; r += (dy > sy) ? -1 : 1)
                memmove(s->fb + (size_t)(dy + r) * s->fb_w + dx,
                        s->fb + (size_t)(sy + r) * s->fb_w + sx, (size_t)w * 4);
            s->fb_active = true;
            glaccel_content_dirty(s, GLACCEL_CD_2D, false); break;
        }
        case PVGPU_OP_BLIT: {
            int x, y, w, h, yy, xx; const uint8_t *img;
            if (p + 16 > len) return;
            x = pv_be32(buf+p); y = pv_be32(buf+p+4); w = pv_be32(buf+p+8); h = pv_be32(buf+p+12);
            p += 16; img = buf + p;
            if (w <= 0 || h <= 0 || p + (uint64_t)w * h * 4 > len) return;
            p += (uint32_t)w * h * 4;
            if (!s->fb) break;
            for (yy = 0; yy < h; yy++) for (xx = 0; xx < w; xx++) {
                int fx = x + xx, fy = y + yy;
                const uint8_t *q = img + ((size_t)yy * w + xx) * 4;
                if (fx >= 0 && fy >= 0 && fx < s->fb_w && fy < s->fb_h)
                    s->fb[(size_t)fy * s->fb_w + fx] = (q[0] << 16) | (q[1] << 8) | q[2];
            }
            s->fb_active = true;
            glaccel_content_dirty(s, GLACCEL_CD_2D, false); break;
        }
        case PVGPU_OP_GL: {
            int x, y; uint32_t need_reply, gllen, adv;
            if (p + 20 > len) return;
            x = pv_be32(buf+p); y = pv_be32(buf+p+4);   /* buf+8 = need_reply flag (was w hint) */
            need_reply = pv_be32(buf+p+8);
            gllen = pv_be32(buf+p+16); p += 20;
            wc->x = x; wc->y = y;                /* composite position for the returned frame */
            if (p + gllen > len) return;
            if (s->inproc) {
                /* IN-PROCESS END-GOAL path (gated on by sgi-glaccel.inproc=on): no socket. */
                PvgpuResp r;
                pvgpu_gl_submit_inproc(s, wc, buf + p, gllen, need_reply, &r);
                if (need_reply && resp_base) {
                    uint8_t o[PVGPU_RESP_BYTES];
                    pvgpu_pack_resp(o, &r);
                    dma_memory_write(&address_space_memory,
                                     resp_base + ((len + 15) & ~15u), o, sizeof o,
                                     MEMTXATTRS_UNSPECIFIED);
                }
            } else {
            pvgpu_gl_send(s, wc, buf + p, gllen); /* async: never block the guest on a void flush */
            if (need_reply) {                    /* query flush: drain replies + hand the last back */
                PvgpuResp r;
                pvgpu_gl_drain(s, wc, &r);
                if (resp_base) {
                    uint8_t o[PVGPU_RESP_BYTES];
                    pvgpu_pack_resp(o, &r);
                    dma_memory_write(&address_space_memory,
                                     resp_base + ((len + 15) & ~15u), o, sizeof o,
                                     MEMTXATTRS_UNSPECIFIED);
                }
            }
            }
            adv = (gllen + 3) & ~3u; p += adv;
            s->fb_active = true;
            glaccel_content_dirty(s, GLACCEL_CD_GLOP, false); break;
        }
        case PVGPU_OP_WINCLIP: {
            uint32_t n, i;
            int old_occ[PVGPU_MAX_OCC][4];
            int old_n = wc->n_occ;
            bool occ_changed;
            if (p + 4 > len) return;
            memcpy(old_occ, wc->occ, sizeof old_occ);
            n = pv_be32(buf + p); p += 4;
            if (n > PVGPU_MAX_OCC) n = PVGPU_MAX_OCC;
            for (i = 0; i < n; i++) {
                if (p + 16 > len) { n = i; break; }
                wc->occ[i][0] = (int)pv_be32(buf+p);   wc->occ[i][1] = (int)pv_be32(buf+p+4);
                wc->occ[i][2] = (int)pv_be32(buf+p+8);  wc->occ[i][3] = (int)pv_be32(buf+p+12);
                p += 16;
            }
            wc->n_occ = n;
            /*
             * Note 29 — the CLIENT-side clip change needs a bounded desktop
             * repaint of its own.  This op only ever set wc->occ; the composite
             * signature folds the occluder set, so the frame was recomposited —
             * but s->desk still holds the pixels the PREVIOUS clip composited,
             * and a clip that SHRINKS leaves them behind inside the window rect.
             * That was harmless only because the very next PVGPU_OP_GL blanket-
             * invalidated the whole desktop and healed it; with the content
             * invalidate bounded, the requirement has to be stated.  It is
             * inside the window rect by construction (a clip cannot alter a pixel
             * the window does not cover), so name that rect.  Independent of the
             * window model: with the model ON the same change also arrives as a
             * clip diff at WM_END, and the rect list de-duplicates.
             */
            occ_changed = (old_n != (int)n) ||
                          memcmp(old_occ, wc->occ, (size_t)n * sizeof old_occ[0]) != 0;
            if (occ_changed && wc->frame && wc->w > 0 && wc->h > 0) {
                const PVGPUWinRec *m = glaccel_wm_find(s, wc->bound_xid);
                glaccel_wm_add_rrect(s, wc->x, wc->y, wc->w, wc->h);
                if (m) {
                    glaccel_wm_add_rrect(s, m->x, m->y, wc->w, wc->h);
                }
                s->wm_repaint = true;
            }
            break;
        }
        case PVGPU_OP_PRESENT:
            /* A GL frame present changes pixels only INSIDE the presenting
             * window's rect.  The composite + bounded blit below already handle
             * that (the composite signature notices the new frame serial), so the
             * whole-desktop re-render this used to request was pure waste — and
             * once the VC2/palette invalidators were bounded it became the LAST
             * saturator, holding 80 % of frames on the full path with an animating
             * GL client on screen.  Geometry changes still take the bounded
             * desk_repaint request (or full, when unnameable) a few hundred lines
             * below; this is the content-only case.
             * PVDISPLAY_PRESENT_FULL=1 restores the old whole-desktop invalidate. */
            {
                static int pf = -1;
                if (pf < 0) pf = getenv("PVDISPLAY_PRESENT_FULL") ? 1 : 0;
                if (pf) s->invalidate = true;
            }
            break;
        /* ---- server-published window model (memo 21 §3) --------------------
         * These six ops are ALWAYS parsed (so the ring stream stays in sync and
         * an unmodified device would not choke), but only acted on when the
         * winmodel property is on.  With it off they are consumed and discarded
         * and the composite path is byte-identical to the pre-model device. */
        case PVGPU_OP_WM_BEGIN: {
            uint32_t gen;
            if (p + 4 > len) return;
            gen = pv_be32(buf + p); p += 4;
            if (!s->winmodel) break;
            s->wm_shadow_n = 0;
            s->wm_batch_gen = gen;
            s->wm_batch_open = true;
            s->wm_batch_bad = false;
            /* the publisher is whichever ring context opened the batch; its
             * CTX_FREE invalidates the model wholesale (xid reuse across an
             * Xsgi restart must never be resolved against a stale model). */
            s->wm_pub_ctx = (int)(wc - s->ctx);
            break;
        }
        case PVGPU_OP_WM_WINDOW: {
            uint32_t xid, flags, stack; int x, y, w, h;
            if (p + 28 > len) return;
            xid   = pv_be32(buf + p);
            x     = (int)pv_be32(buf + p + 4);  y = (int)pv_be32(buf + p + 8);
            w     = (int)pv_be32(buf + p + 12); h = (int)pv_be32(buf + p + 16);
            flags = pv_be32(buf + p + 20);
            stack = pv_be32(buf + p + 24);
            p += 28;
            if (!s->winmodel || !s->wm_batch_open) break;
            if (s->wm_shadow_n >= PVGPU_MAX_WM_WINDOWS || xid == 0) {
                s->wm_batch_bad = true; break;   /* reject wholesale, never truncate */
            }
            {
                PVGPUWinRec *r = &s->wm_shadow[s->wm_shadow_n++];
                memset(r, 0, sizeof *r);
                r->xid = xid; r->x = x; r->y = y; r->w = w; r->h = h;
                r->flags = flags; r->stack = stack;
            }
            break;
        }
        case PVGPU_OP_WM_CLIP: {
            uint32_t xid, n, i;
            if (p + 8 > len) return;
            xid = pv_be32(buf + p); n = pv_be32(buf + p + 4); p += 8;
            if (n > 0xffff || p + (uint64_t)n * 16 > len) return;   /* malformed ring */
            {
                PVGPUWinRec *r = NULL;
                int k;
                if (s->winmodel && s->wm_batch_open) {
                    for (k = s->wm_shadow_n - 1; k >= 0; k--) {
                        if (s->wm_shadow[k].xid == xid) { r = &s->wm_shadow[k]; break; }
                    }
                    if (!r || n > PVGPU_MAX_WM_CLIP) s->wm_batch_bad = true;
                }
                for (i = 0; i < n; i++) {
                    if (r && i < PVGPU_MAX_WM_CLIP) {
                        r->clip[i][0] = (int)pv_be32(buf + p);
                        r->clip[i][1] = (int)pv_be32(buf + p + 4);
                        r->clip[i][2] = (int)pv_be32(buf + p + 8);
                        r->clip[i][3] = (int)pv_be32(buf + p + 12);
                    }
                    p += 16;
                }
                if (r && n <= PVGPU_MAX_WM_CLIP) r->n_clip = (int)n;
            }
            break;
        }
        case PVGPU_OP_WM_GONE: {
            uint32_t xid; int k;
            if (p + 4 > len) return;
            xid = pv_be32(buf + p); p += 4;
            if (!s->winmodel || xid == 0) break;
            /* retire the window from the batch under construction AND from the
             * live model, and drop any context binding to it — a live frame must
             * never be composited into a retired window's rectangle. */
            for (k = s->wm_shadow_n - 1; k >= 0; k--) {
                if (s->wm_shadow[k].xid == xid) {
                    s->wm_shadow[k] = s->wm_shadow[--s->wm_shadow_n];
                }
            }
            for (k = s->wm_n - 1; k >= 0; k--) {
                if (s->wm[k].xid == xid) { s->wm[k] = s->wm[--s->wm_n]; }
            }
            {
                bool was_bound = false;
                for (k = 0; k < PVGPU_MAXCTX; k++) {
                    if (s->ctx[k].bound_xid == xid) {
                        s->ctx[k].bound_xid = 0; was_bound = true;
                    }
                }
                /* Only a retire that dropped a LIVE binding changes the output (the
                 * desktop under that window must be restored).  Retiring a window no
                 * context renders into cannot alter a pixel, and 4Dwm retires
                 * gadgets constantly — that case must not force a repaint. */
                if (was_bound) {
                    s->wm_sig++;
                    /* BL-81 leaves this one on the full path deliberately: a
                     * retire that dropped a live binding happens once per window
                     * close (not per op of a move/resize storm), and the record
                     * has already been removed from wm[] here, so no rect is in
                     * hand to bound it with. */
                    s->wm_repaint = true;
                    s->wm_repaint_full = true;
                    glaccel_wm_request_present(s);
                }
            }
            break;
        }
        case PVGPU_OP_WM_END: {
            uint32_t gen;
            if (p + 4 > len) return;
            gen = pv_be32(buf + p); p += 4;
            if (!s->winmodel) break;
            if (s->wm_batch_open && !s->wm_batch_bad && gen == s->wm_batch_gen) {
                /* Fix (a): decide BEFORE the swap, while wm[] still holds the
                 * previous generation, whether any bound window actually moved. */
                bool changed = glaccel_wm_bound_changed(s);
                memcpy(s->wm, s->wm_shadow,
                       (size_t)s->wm_shadow_n * sizeof s->wm[0]);
                s->wm_n = s->wm_shadow_n;
                s->wm_gen = gen ? gen : 1;   /* 0 is reserved for "no model" */
                s->wm_commits++;
                if (changed) {
                    /* wm_sig — not wm_gen — is what the composite signature folds, so
                     * an unchanged republish leaves the idle-skip free to suppress the
                     * repaint.  A model commit re-composites with no GL activity at all. */
                    s->wm_sig++;
                    /* BL-81: this used to be `s->invalidate = true`, i.e. a
                     * whole-desktop re-walk plus a 5 MB full-screen blit for EVERY
                     * commit that touched a bound window — the dominant term in the
                     * +7.5-point GL-bound cost of note 26 §2, and the one the note's
                     * §5 mechanism (geom_changed) only half explained.  The commit
                     * now names the rects it can have altered
                     * (glaccel_wm_bound_changed) and the update path repaints just
                     * those; wm_repaint_full is the fail-safe back to the old
                     * behaviour whenever they cannot be named. */
                    s->wm_repaint = true;
                    glaccel_wm_request_present(s);
                } else {
                    s->wm_commits_nop++;
                }
                glaccel_wm_stats(s);
            } else if (s->wm_batch_open) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "pvgpu: window-model batch discarded (bad=%d gen=%u/%u n=%d)\n",
                              s->wm_batch_bad, gen, s->wm_batch_gen, s->wm_shadow_n);
            }
            s->wm_batch_open = false;
            s->wm_batch_bad = false;
            break;
        }
        case PVGPU_OP_WM_BIND: {
            uint32_t xid;
            if (p + 4 > len) return;
            xid = pv_be32(buf + p); p += 4;
            wc->bound_xid = xid;
            break;
        }
        case PVGPU_OP_SCANOUT_SET: {
            /* Stage 2 Phase 2a: register a shadow framebuffer in guest RAM.
             * [op:u32][base_lo:u32][base_hi:u32][w:u32][h:u32][stride:u32][fmt:u32]
             * The engine only echoes the params; the desktop renderer (Newport)
             * validates the (hostile) guest values and owns the DID-walk sourcing. */
            uint32_t lo, hi;
            if (p + 24 > len) return;
            lo = pv_be32(buf + p); hi = pv_be32(buf + p + 4);
            /* 2a shortcut (no driver MAP_FB ioctl yet): if base_hi bit 31 is set,
             * base_lo is a byte OFFSET from THIS context's ring physical base, so
             * a synthetic guest test can place the shadowfb inside its own mmap'd
             * ring without knowing the ring's guest-physical address.  Phase 2b's
             * real DDX registers an absolute phys base (bit 31 clear) via a proper
             * PVGPU_MAP_FB ioctl that pins+translates a dedicated fb region. */
            if (hi & 0x80000000u) {
                s->shadow_base = (uint64_t)wc->cmd_base + lo;
            } else {
                s->shadow_base = ((uint64_t)hi << 32) | lo;
            }
            s->shadow_w      = pv_be32(buf + p + 8);
            s->shadow_h      = pv_be32(buf + p + 12);
            s->shadow_stride = pv_be32(buf + p + 16);
            s->shadow_format = pv_be32(buf + p + 20);
            p += 24;
            s->shadow_active = (s->shadow_base != 0 && s->shadow_w > 0
                                && s->shadow_h > 0);
            if (s->desk_scanout) {
                s->desk_scanout(s->desk_opaque, s->shadow_base, s->shadow_w,
                                s->shadow_h, s->shadow_stride, s->shadow_format,
                                s->shadow_active);
            }
            break;
        }
        case PVGPU_OP_DAMAGE: {
            /* Stage 2 Phase 2a: shadowfb region changed — feed the desktop
             * renderer's dirty-rect machinery (bounds-checked + coalesced there).
             * [op:u32][x:u32][y:u32][w:u32][h:u32] */
            uint32_t dx, dy, dw, dh;
            if (p + 16 > len) return;
            dx = pv_be32(buf + p);      dy = pv_be32(buf + p + 4);
            dw = pv_be32(buf + p + 8);  dh = pv_be32(buf + p + 12);
            p += 16;
            if (s->desk_damage) {
                s->desk_damage(s->desk_opaque, (int)dx, (int)dy,
                               (int)dw, (int)dh);
            }
            break;
        }
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "pvgpu: bad op %u at %u\n", op, p - 4);
            return;
        }
        if (wm_t0) s->t_wm_ops += glaccel_now_ns() - wm_t0;
    }
}

/*
 * gfx_update callback — called at ~60Hz by the QEMU display subsystem.
 * DMA the guest framebuffer and blit to the console surface.
 */
static int glaccel_composite_overlays(SGIGLAccelState *s, uint32_t *desk,
                                      int dw, int dh,
                                      PVDeskRect *out_rects, int max_out);
/* glaccel_clamp_rect and glaccel_wm_find are forward-declared above, at the
 * BL-81 rect helpers (note 29's WINCLIP repaint request needs wm_find earlier). */

/*
 * Cheap signature of the GL overlay set (which windows exist, their latest
 * frame serials, placement and occluders). If it is unchanged from the last
 * blitted frame AND the desktop render reported no change AND nothing forced a
 * repaint, the composited output is byte-identical to what the surface already
 * holds, so the whole composite/blit/dpy_gfx_update can be skipped (idle-frame
 * regression fix — restores the old "if (!display_dirty) return" early-out).
 */
static uint64_t glaccel_composite_signature(SGIGLAccelState *s)
{
    uint64_t sig = 1469598103934665603ULL; /* FNV-ish seed */
    int i, k;
    /* Window model: fold wm_sig, NOT wm_gen.  wm_sig advances on (and only on) a
     * commit that changed something a BOUND context resolves through, so a
     * placement/clip/stacking change still repaints with no GL frame — while the
     * constant no-op republishing of an unchanged 4Dwm tree leaves the signature
     * alone and stays suppressible by the idle-skip below (24-…md §3: folding
     * wm_gen made every republish a mandatory full-screen repaint).  Zero when no
     * model has ever committed => unchanged signature. */
    sig = (sig ^ s->wm_sig) * 1099511628211ULL;
    for (i = 0; i < PVGPU_MAXCTX; i++) {
        PVGPUCtx *c = &s->ctx[i];
        if (!c->active || !c->frame || c->w <= 0 || c->h <= 0) {
            continue;
        }
        sig = (sig ^ (uint32_t)i) * 1099511628211ULL;
        sig = (sig ^ c->bound_xid) * 1099511628211ULL;
        sig = (sig ^ c->frame_serial) * 1099511628211ULL;
        sig = (sig ^ (uint32_t)c->x) * 1099511628211ULL;
        sig = (sig ^ (uint32_t)c->y) * 1099511628211ULL;
        sig = (sig ^ (uint32_t)c->w) * 1099511628211ULL;
        sig = (sig ^ (uint32_t)c->h) * 1099511628211ULL;
        sig = (sig ^ (uint32_t)c->n_occ) * 1099511628211ULL;
        for (k = 0; k < c->n_occ && k < PVGPU_MAX_OCC; k++) {
            sig = (sig ^ (uint32_t)c->occ[k][0]) * 1099511628211ULL;
            sig = (sig ^ (uint32_t)c->occ[k][1]) * 1099511628211ULL;
            sig = (sig ^ (uint32_t)c->occ[k][2]) * 1099511628211ULL;
            sig = (sig ^ (uint32_t)c->occ[k][3]) * 1099511628211ULL;
        }
    }
    return sig;
}

/* Phase D: blit one rect of the engine desktop buffer to the display surface
 * and issue a bounded dpy_gfx_update for it (replaces the full-screen memcpy +
 * dpy_gfx_update_full on incremental frames).  r is in desk/surface space. */
static void glaccel_blit_rect(SGIGLAccelState *s, uint8_t *dst, int dst_stride,
                              const PVDeskRect *r)
{
    int y;
    if (r->w <= 0 || r->h <= 0) return;
    for (y = r->y; y < r->y + r->h && y < s->desk_h; y++) {
        memcpy(dst + (size_t)y * dst_stride + (size_t)r->x * 4,
               s->desk + (size_t)y * s->desk_w + r->x,
               (size_t)r->w * 4);
    }
    dpy_gfx_update(s->con, r->x, r->y, r->w, r->h);
}

static void sgi_glaccel_update(void *opaque)
{
    SGIGLAccelState *s = opaque;
    DisplaySurface *surface;
    int width, height, src_stride;

    /* ---- unified display path (Phase A) — runs when a desktop layer is registered ---- */
    if (s->desk_render) {
        int y, dst_stride, n_wins, i;
        uint8_t *dst;
        uint64_t sig;
        PVDeskRect desk_rects[PVDESK_MAX_RECTS];
        PVDeskRect win_rects[PVGPU_MAXCTX];
        int n_desk_rects = 0, n_win_rects = 0;
        bool desk_full, desk_changed, force, geom_changed = false;

        /* pin the console to desktop resolution */
        surface = qemu_console_surface(s->con);
        if (!surface || surface_width(surface) != s->desk_w ||
            surface_height(surface) != s->desk_h) {
            qemu_console_resize(s->con, s->desk_w, s->desk_h);
            surface = qemu_console_surface(s->con);
            s->invalidate = true;
        }
        if (!surface) return;

        /* Stage 2 Phase 2a note: the shadow framebuffer is NOT scanned out here.
         * SCANOUT_SET/DAMAGE are forwarded to the desktop renderer (Newport),
         * which sources the base layer's CI8 rows from guest RAM inside its
         * existing DID/XMAP/CMAP walk.  The engine's unified path below is
         * therefore identical whether or not a shadowfb is active — Phase D's
         * dirty-rect + bounded-blit machinery covers both by construction. */

        /*
         * Phase D force decision.  A full desktop repaint (and full blit) is
         * required when:
         *  - a backend/console invalidate is pending (s->invalidate);
         *  - all GL windows just vanished (one-frame restore of the desktop
         *    underneath — the old logic);
         *  - the GL window GEOMETRY changed vs. last frame (a window moved,
         *    resized, appeared, or vanished) AND the desktop layer cannot take a
         *    bounded repaint request: the desktop under the previous positions
         *    must be restored, which the plain incremental path does not do.
         *
         * BL-81: when the desktop layer DOES implement desk_repaint, a geometry
         * change is expressed as a bounded request instead — the union of every
         * old and new GL-window rect involved in the change.  That is exactly the
         * region whose desktop pixels can differ: inside the new rects the window
         * covers the desktop (and is composited over it below), inside the old
         * rects the desktop is newly uncovered, and outside both nothing moved.
         * The renderer folds those rects into its dirty list, repaints them (or
         * more — coalescing/saturating to full is always legal), returns them in
         * desk_rects[], and the bounded blit below copies exactly them.  This
         * replaces a 1280x1024 re-walk + 5 MB memcpy per present of a move/resize
         * storm with work proportional to the window (note 26 §5).
         *
         * Content-only changes (same geometry, new frame_serial) stay on the
         * cheap bounded path exactly as before.
         */
        force = s->invalidate;
        {
            int n_now = 0, j;
            /* BL-81 union set: old ∪ new rects of the windows that changed.
             * Snapshotted here because win_rects[] is overwritten later by
             * glaccel_composite_overlays(). */
            PVDeskRect geom_rects[2 * PVGPU_MAXCTX];
            int n_geom_rects = 0;
            for (i = 0; i < PVGPU_MAXCTX; i++) {
                PVGPUCtx *c = &s->ctx[i];
                const PVGPUWinRec *m;
                if (!(c->active && c->frame && c->w > 0 && c->h > 0)) continue;
                /* Resolve the rect the SAME way the composite will (window model
                 * first, submit-time origin otherwise).  Getting this wrong is not
                 * cosmetic: prev_win_rects[] is filled from glaccel_composite_
                 * overlays()'s out_rects, which are MODEL positions, so comparing
                 * them against submit-time origins made geom_changed — and hence
                 * force, a full desktop re-render plus a full-screen blit — true on
                 * EVERY composite for as long as a model placed any window away
                 * from its submit origin.  That was a second, larger source of the
                 * ON-arm host-CPU cost than the WM_END scheduling itself. */
                m = glaccel_wm_find(s, c->bound_xid);
                if (m) {
                    /* an unmapped/fully-clipped modelled window composites nothing,
                     * so it must not appear in the geometry set either */
                    if (!(m->flags & PVGPU_WMF_MAPPED) || m->n_clip <= 0) continue;
                    glaccel_clamp_rect(&win_rects[n_now], m->x, m->y, c->w, c->h,
                                       s->desk_w, s->desk_h);
                } else {
                    glaccel_clamp_rect(&win_rects[n_now], c->x, c->y, c->w, c->h,
                                       s->desk_w, s->desk_h);
                }
                n_now++;
            }
            if (!force && s->prev_n_windows > 0 && n_now == 0) {
                force = true;
            }
            /* Compare as an unordered SET: the composite emits out_rects in
             * stacking order (model stack when modelled), which need not match
             * this ctx-index walk, and a pure reordering is not a geometry
             * change.  n <= PVGPU_MAXCTX (8), so O(n^2) is free.
             *
             * BL-81 walks the difference in BOTH directions and records the
             * offending rects: a rect present now but not before is a new/moved-to
             * position (must be re-walked), a rect present before but not now is a
             * vacated position (the desktop under it is newly uncovered).  Their
             * union is the complete set of pixels a geometry change can alter. */
            if (n_now != s->prev_n_win_rects) {
                geom_changed = true;
            }
            for (i = 0; i < n_now; i++) {
                bool found = false;
                for (j = 0; j < s->prev_n_win_rects; j++) {
                    if (win_rects[i].x == s->prev_win_rects[j].x &&
                        win_rects[i].y == s->prev_win_rects[j].y &&
                        win_rects[i].w == s->prev_win_rects[j].w &&
                        win_rects[i].h == s->prev_win_rects[j].h) {
                        found = true; break;
                    }
                }
                if (!found) {
                    geom_changed = true;
                    geom_rects[n_geom_rects++] = win_rects[i];
                }
            }
            for (j = 0; j < s->prev_n_win_rects; j++) {
                bool found = false;
                for (i = 0; i < n_now; i++) {
                    if (win_rects[i].x == s->prev_win_rects[j].x &&
                        win_rects[i].y == s->prev_win_rects[j].y &&
                        win_rects[i].w == s->prev_win_rects[j].w &&
                        win_rects[i].h == s->prev_win_rects[j].h) {
                        found = true; break;
                    }
                }
                if (!found) {
                    geom_changed = true;
                    geom_rects[n_geom_rects++] = s->prev_win_rects[j];
                }
            }
            /* Fold in the rects the model commits themselves named (BL-81, set by
             * glaccel_wm_bound_changed at WM_END): a clip/stacking/MAPPED change
             * alters pixels inside a window rect without changing any rect, so the
             * geometry diff above cannot see it. */
            if (s->wm_repaint) {
                for (i = 0; i < s->wm_n_rrects &&
                            n_geom_rects < (int)ARRAY_SIZE(geom_rects); i++) {
                    geom_rects[n_geom_rects++] = s->wm_rrects[i];
                }
                if (i < s->wm_n_rrects) s->wm_repaint_full = true;
            }

            if (geom_changed || s->wm_repaint) {
                /*
                 * BL-81 bounded repaint.  Conditions to take it — every one of
                 * them fails SAFE (to the pre-BL-81 full repaint):
                 *  - a desktop layer that implements desk_repaint;
                 *  - a non-empty rect set.  An empty one means we cannot NAME the
                 *    changed region (e.g. the window count changed with no rect
                 *    differing — two windows sharing a rect, one closing), so
                 *    repaint everything;
                 *  - nothing set wm_repaint_full (first model ever, a bound window
                 *    retired, or the rect list overflowed);
                 *  - not already forced for another reason (console/backend
                 *    invalidate, or all GL windows vanishing — a once-per-close
                 *    event left on the full path deliberately);
                 *  - PVDISPLAY_GEOM_FULL=1 not set (the A/B oracle: restores the
                 *    exact pre-BL-81 behaviour in one binary).
                 */
                static int geom_full_oracle = -1;
                if (geom_full_oracle < 0)
                    geom_full_oracle = getenv("PVDISPLAY_GEOM_FULL") ? 1 : 0;

                if (!force && !geom_full_oracle && !s->wm_repaint_full &&
                    s->desk_repaint && n_geom_rects > 0) {
                    /* PVDISPLAY_GEOM_SHRINK=N: POSITIVE control for the bounded
                     * repaint's correctness probe — deliberately request N pixels
                     * LESS on every side than the change requires, so a probe that
                     * cannot see an under-repaint (stale desktop pixels left under
                     * a window's old position) is proven toothless.  Debug only. */
                    static int shrink = -1;
                    if (shrink < 0) {
                        const char *e = getenv("PVDISPLAY_GEOM_SHRINK");
                        shrink = e ? atoi(e) : 0;
                    }
                    if (shrink > 0) {
                        for (i = 0; i < n_geom_rects; i++) {
                            PVDeskRect *r = &geom_rects[i];
                            int sx = r->w > 2 * shrink ? shrink : 0;
                            int sy = r->h > 2 * shrink ? shrink : 0;
                            r->x += sx; r->w -= 2 * sx;
                            r->y += sy; r->h -= 2 * sy;
                        }
                    }
                    s->desk_repaint(s->desk_opaque, geom_rects, n_geom_rects);
                    s->geom_bounded_repaints++;
                } else {
                    force = true;
                    s->geom_full_repaints++;
                }
            }
            s->wm_repaint = false;
            s->wm_repaint_full = false;
            s->wm_n_rrects = 0;
        }

        /* render the desktop into the engine buffer */
        {
            int rc = 0;
            s->n_updates++;
            GLACCEL_TIME(s, t_desk_render,
                         rc = s->desk_render(s->desk_opaque, s->desk, s->desk_w,
                                             s->desk_h, force, desk_rects,
                                             PVDESK_MAX_RECTS));
            s->invalidate = false;
            desk_full = (rc == PVDESK_FULL);
            desk_changed = (rc != 0);
            n_desk_rects = desk_full ? 0 : rc;
            if (desk_full)   s->n_render_full++;
            else if (rc > 0) s->n_render_rects++;
            else             s->n_render_idle++;
            /* The timing block has to be reportable with the model OFF too — the
             * whole point of the attribution is "how much does the model ADD to the
             * device-side cost", and glaccel_wm_stats() only ever fires on a model
             * commit.  Print independently, every 256 updates. */
            if (glaccel_timing_on() && (s->n_updates % 256) == 0) {
                fprintf(stderr, "pvgpu: TIMING upd=%" PRIu64 " t_render=%" PRIu64
                        " t_comp=%" PRIu64 " t_blit=%" PRIu64 " t_wmops=%" PRIu64
                        " (ms) rfull=%" PRIu64 " rrects=%" PRIu64 " ridle=%" PRIu64
                        " presents=%" PRIu64 " geombounded=%" PRIu64
                        " geomfull=%" PRIu64
                        " inv(frame=%" PRIu64 " glop=%" PRIu64 " 2d=%" PRIu64
                        " dropped=%" PRIu64 ")\n",
                        s->n_updates, s->t_desk_render / 1000000,
                        s->t_composite / 1000000, s->t_blit / 1000000,
                        s->t_wm_ops / 1000000, s->n_render_full,
                        s->n_render_rects, s->n_render_idle, s->wm_presents,
                        s->geom_bounded_repaints, s->geom_full_repaints,
                        s->n_inv_frame, s->n_inv_glop, s->n_inv_2d,
                        s->n_inv_dropped);
            }
        }

        /* Idle-frame skip: the desktop reported no change AND the GL overlay set
         * is identical to the last blitted frame — the surface already holds the
         * composited output, so skip everything. */
        sig = glaccel_composite_signature(s);
        {
            /* BL-92 A/B decode instrumentation: one line per gfx_update while the
             * env is set, reporting exactly what the composite decision path
             * saw — desk_render rc, the signature vs last, and the per-ctx
             * frame serials — so a frozen display can be attributed to the
             * skip vs a stale serial vs a dead render.  Pure print; no
             * behavior change.  SGI_GLACCEL_DECISION_DBG=<n> prints only every
             * n-th update (1 = all). */
            static int dd_dbg = -1;
            static int dd_every = 1;
            if (dd_dbg < 0) {
                const char *e = getenv("SGI_GLACCEL_DECISION_DBG");
                dd_dbg = e ? 1 : 0;
                if (e) dd_every = atoi(e) > 0 ? atoi(e) : 1;
            }
            if (dd_dbg && (s->n_updates % dd_every) == 0) {
                /* one entry can exceed 16 bytes by far (the format is
                 * ~55-96 chars); sized per PVGPU_MAXCTX with a clamp
                 * after each snprintf — snprintf returns the WOULD-BE
                 * length, so an unclamped off would write the trailing
                 * NUL past the buffer end (review fix). */
                char ctxs[PVGPU_MAXCTX * 96 + 1];
                int off = 0, ci;
                for (ci = 0; ci < PVGPU_MAXCTX && off < (int)sizeof(ctxs) - 96; ci++) {
                    PVGPUCtx *c = &s->ctx[ci];
                    if (!c->active) continue;
                    /* BL-92 decode: fold a cheap FNV hash of the delivered
                     * frame CONTENT — serials advancing while this hash is
                     * constant means the renderer is re-delivering identical
                     * pixels (frozen content), not a device-side skip. */
                    uint64_t fh = 1469598103934665603ULL;
                    if (c->frame && c->w > 0 && c->h > 0) {
                        const uint32_t *px = c->frame;
                        size_t np = (size_t)c->w * c->h, k;
                        for (k = 0; k < np; k++) {
                            fh = (fh ^ px[k]) * 1099511628211ULL;
                        }
                    } else {
                        fh = 0;
                    }
                    off += snprintf(ctxs + off, sizeof(ctxs) - off,
                                    " c%d[%d %dx%d@%d,%d ser=%u fh=%llx xid=%u]", ci,
                                    c->frame ? 1 : 0, c->w, c->h, c->x, c->y,
                                    (unsigned)c->frame_serial,
                                    (unsigned long long)fh, c->bound_xid);
                    if (off > (int)sizeof(ctxs) - 1) {
                        off = (int)sizeof(ctxs) - 1;
                    }
                }
                ctxs[off] = 0;
                fprintf(stderr, "pvgpu: DECISION upd=%" PRIu64 " rrc=%d "
                        "desk_changed=%d force=%d desk_full=%d geom=%d "
                        "sig=%llx last=%llx have=%d skip=%d%s\n",
                        s->n_updates, n_desk_rects ? n_desk_rects : 0,
                        desk_changed, force, desk_full, geom_changed,
                        (unsigned long long)sig,
                        (unsigned long long)s->last_composite_sig,
                        s->have_composite_sig,
                        (!desk_changed && s->have_composite_sig &&
                         sig == s->last_composite_sig),
                        ctxs);
            }
        }
        if (!desk_changed && s->have_composite_sig &&
            sig == s->last_composite_sig) {
            return;   /* nothing changed; leave the surface as-is */
        }
        s->last_composite_sig = sig;
        s->have_composite_sig = true;

        /* composite GL overlays onto the desktop buffer; capture composited
         * (clamped) window rects for a bounded blit */
        GLACCEL_TIME(s, t_composite,
                     n_wins = glaccel_composite_overlays(s, s->desk, s->desk_w,
                                                         s->desk_h, win_rects,
                                                         PVGPU_MAXCTX));
        s->prev_n_windows = n_wins;
        n_win_rects = n_wins < PVGPU_MAXCTX ? n_wins : PVGPU_MAXCTX;

        /*
         * PVDISPLAY_BOUND_VERIFY=1 — device-side differential oracle for BL-81.
         *
         * Screendump-based probes of the bounded repaint are hard to give teeth:
         * transient staleness HEALS (any later whole-desktop repaint rebuilds the
         * surface from VRAM), so a deliberately-broken control can pass.  This
         * checks the actual invariant, per frame, inside the device: after a
         * BOUNDED frame, s->desk must be pixel-identical to what a from-scratch
         * full render + composite of the SAME state would have produced.  The
         * comparison uses the same VRAM and the same GL frames within one update()
         * call, so it is immune to guest timing and to boot-to-boot drift.
         *
         * Mismatches are reported and then HEALED (the verify buffer is the correct
         * image), so the mode is a pure detector: it cannot mask a defect by
         * leaving the screen wrong, and it cannot be mistaken for the shipping path
         * (it forces a full blit and costs a whole extra render per frame).
         */
        {
            static int bound_verify = -1;
            if (bound_verify < 0)
                bound_verify = getenv("PVDISPLAY_BOUND_VERIFY") ? 1 : 0;
            if (bound_verify && !force) {
                size_t npix = (size_t)s->desk_w * s->desk_h, k, bad = 0, fx = 0, fy = 0;
                if (!s->verify_buf) {
                    s->verify_buf = g_new0(uint32_t, npix);
                }
                s->desk_render(s->desk_opaque, s->verify_buf, s->desk_w, s->desk_h,
                               true, NULL, 0);
                glaccel_composite_overlays(s, s->verify_buf, s->desk_w, s->desk_h,
                                           NULL, 0);
                for (k = 0; k < npix; k++) {
                    if ((s->desk[k] & 0xffffffu) != (s->verify_buf[k] & 0xffffffu)) {
                        if (!bad) { fx = k % s->desk_w; fy = k / s->desk_w; }
                        bad++;
                    }
                }
                s->verify_frames++;
                if (bad) {
                    s->verify_bad_frames++;
                    s->verify_bad_pixels += bad;
                    fprintf(stderr, "pvgpu: BOUNDVERIFY MISMATCH frame=%" PRIu64
                            " pixels=%zu first=%zu,%zu bounded=%" PRIu64
                            " full=%" PRIu64 "\n", s->verify_frames, bad, fx, fy,
                            s->geom_bounded_repaints, s->geom_full_repaints);
                }
                memcpy(s->desk, s->verify_buf, npix * 4);   /* heal */
                desk_full = true;                           /* and blit it all */
            }
        }

        dst_stride = surface_stride(surface);
        dst = surface_data(surface);

        /* PVDISPLAY_FULL_BLIT=1: measurement oracle — force the pre-Phase-D
         * full-screen blit + dpy_gfx_update_full every frame, so the Phase D
         * bounded-blit CPU win can be measured before/after with one binary. */
        {
            static int fb_oracle = -1;
            if (fb_oracle < 0)
                fb_oracle = getenv("PVDISPLAY_FULL_BLIT") ? 1 : 0;
            if (fb_oracle) desk_full = true;
        }

        GLACCEL_TIME(s, t_blit, {
            if (desk_full) {
                /* full blit engine buffer → display surface */
                for (y = 0; y < s->desk_h; y++)
                    memcpy(dst + (size_t)y * dst_stride,
                           s->desk + (size_t)y * s->desk_w,
                           (size_t)s->desk_w * 4);
                dpy_gfx_update_full(s->con);
            } else {
                /* bounded blit: desktop dirty rects + composited GL window rects.
                 * Both copies are bounded here (VRAM→desk happened per-rect in the
                 * render callback; desk→surface + dpy_gfx_update are bounded now). */
                for (i = 0; i < n_desk_rects; i++) {
                    glaccel_blit_rect(s, dst, dst_stride, &desk_rects[i]);
                }
                for (i = 0; i < n_win_rects; i++) {
                    glaccel_blit_rect(s, dst, dst_stride, &win_rects[i]);
                }
            }
        });

        /* remember this frame's window geometry for next frame's change test */
        s->prev_n_win_rects = n_win_rects;
        for (i = 0; i < n_win_rects; i++) {
            s->prev_win_rects[i] = win_rects[i];
        }
        return;   /* unified path — never fall through to legacy paths */
    }

    /* ---- legacy paths — only reachable when no desktop layer is registered ---- */

    /* Paravirtual-GPU path: present the internal framebuffer (2D command output), with any
     * host-rendered GL frame composited in at its position. */
    if (s->fb_active && s->fb && s->fb_w > 0 && s->fb_h > 0) {
        int y, dst_stride;
        uint8_t *dst;
        /* composite the primary window's latest GL frame into the fb (device console only) */
        PVGPUCtx *c0 = &s->ctx[0];
        if (c0->active && c0->frame && c0->w > 0 && c0->h > 0) {
            int gy, gx;
            for (gy = 0; gy < c0->h; gy++) {
                int fy = c0->y + gy;
                if (fy < 0 || fy >= s->fb_h) continue;
                for (gx = 0; gx < c0->w; gx++) {
                    int fx = c0->x + gx;
                    if (fx < 0 || fx >= s->fb_w) continue;
                    s->fb[(size_t)fy * s->fb_w + fx] = c0->frame[(size_t)gy * c0->w + gx];
                }
            }
        }
        surface = qemu_console_surface(s->con);
        if (!surface || surface_width(surface) != s->fb_w ||
            surface_height(surface) != s->fb_h) {
            qemu_console_resize(s->con, s->fb_w, s->fb_h);
            surface = qemu_console_surface(s->con);
        }
        if (!surface) return;
        dst_stride = surface_stride(surface);
        dst = surface_data(surface);
        for (y = 0; y < s->fb_h; y++)
            memcpy(dst + (size_t)y * dst_stride, s->fb + (size_t)y * s->fb_w, (size_t)s->fb_w * 4);
        dpy_gfx_update_full(s->con);
        return;
    }

    /* Live GL frame path: present the primary window's latest host-rendered frame. */
    {
    PVGPUCtx *c0 = &s->ctx[0];
    if (c0->active && c0->frame && c0->w > 0 && c0->h > 0) {
        int y, dst_stride;
        uint8_t *dst;
        surface = qemu_console_surface(s->con);
        if (!surface || surface_width(surface) != c0->w ||
            surface_height(surface) != c0->h) {
            qemu_console_resize(s->con, c0->w, c0->h);
            surface = qemu_console_surface(s->con);
        }
        if (!surface) {
            return;
        }
        dst_stride = surface_stride(surface);
        dst = surface_data(surface);
        for (y = 0; y < c0->h; y++) {
            memcpy(dst + (size_t)y * dst_stride,
                   c0->frame + (size_t)y * c0->w,
                   (size_t)c0->w * 4);
        }
        dpy_gfx_update_full(s->con);
        return;
    }
    }

    if (s->fb_base == 0 || s->width == 0 || s->height == 0) {
        return;
    }

    width = s->width;
    height = s->height;
    src_stride = glaccel_stride(s);

    surface = qemu_console_surface(s->con);

    /* Resize console if needed */
    if (!surface ||
        surface_width(surface) != width ||
        surface_height(surface) != height) {
        qemu_console_resize(s->con, width, height);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }

    if (!surface) {
        return;
    }

    /*
     * DMA the entire guest framebuffer and convert scanline by scanline.
     * For a dumb framebuffer this is simple and correct. A future
     * optimization could use dirty-page tracking.
     */
    int dst_stride = surface_stride(surface);
    uint8_t *dst = surface_data(surface);
    uint8_t *src_line = g_malloc(src_stride);

    for (int y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        s->fb_base + (uint64_t)y * src_stride,
                        src_line, src_stride, MEMTXATTRS_UNSPECIFIED);

        if (s->format == GLACCEL_FMT_RGB565) {
            glaccel_draw_line_rgb565(s, dst, src_line, width, src_stride);
        } else {
            glaccel_draw_line_rgba8888(s, dst, src_line, width, src_stride);
        }
        dst += dst_stride;
    }

    g_free(src_line);
    dpy_gfx_update_full(s->con);
}

static void sgi_glaccel_invalidate(void *opaque)
{
    SGIGLAccelState *s = opaque;
    s->invalidate = true;
    /* forward to the desktop layer so its cached state (dirty flag, mode cache) is invalidated */
    if (s->desk_invalidate) {
        s->desk_invalidate(s->desk_opaque);
    }
}

static uint64_t sgi_glaccel_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIGLAccelState *s = opaque;

    switch (addr) {
    case SGI_GLACCEL_STATUS:
        return s->status;
    case SGI_GLACCEL_WIDTH:
        return s->width;
    case SGI_GLACCEL_HEIGHT:
        return s->height;
    case SGI_GLACCEL_CMD_BASE:
        return s->cmd_base;
    case SGI_GLACCEL_CMD_LEN:
        return s->cmd_len;
    case SGI_GLACCEL_FB_BASE:
        return s->fb_base;
    case SGI_GLACCEL_FORMAT:
        return s->format;
    case SGI_GLACCEL_STRIDE:
        return s->stride;
    case SGI_GLACCEL_CONTEXT:
        return s->cur_ctx;
    case SGI_GLACCEL_WINMODEL:
        /* generation of the live server-published window model; 0 = nobody is
         * publishing (or the device is not consuming), i.e. "keep guessing". */
        return s->winmodel ? s->wm_gen : 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }
}

static void sgi_glaccel_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    SGIGLAccelState *s = opaque;

    switch (addr) {
    case SGI_GLACCEL_STATUS:
        s->status &= ~val; /* W1C */
        if (s->status == 0) {
            qemu_irq_lower(s->irq);
        }
        break;
    case SGI_GLACCEL_WIDTH:
        s->width = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_HEIGHT:
        s->height = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_CMD_BASE:
        s->cmd_base = val;
        s->ctx[s->cur_ctx].cmd_base = val;   /* latch this context's ring base */
        break;
    case SGI_GLACCEL_CMD_LEN:
        s->cmd_len = val;
        break;
    case SGI_GLACCEL_FB_BASE:
        s->fb_base = val;
        s->invalidate = true;
        break;
    case SGI_GLACCEL_FORMAT:
        if (val <= GLACCEL_FMT_RGB565) {
            s->format = val;
            s->invalidate = true;
        }
        break;
    case SGI_GLACCEL_STRIDE:
        s->stride = val;
        break;
    case SGI_GLACCEL_CONTEXT:
        s->cur_ctx = (val < PVGPU_MAXCTX) ? val : 0;   /* select the window for the next EXEC */
        break;
    case SGI_GLACCEL_EXEC:
        if (val & GLACCEL_CMD_RESET) {
            s->status = 0;
            s->width = 0;
            s->height = 0;
            s->cmd_base = 0;
            s->cmd_len = 0;
            s->fb_base = 0;
            s->format = GLACCEL_FMT_RGBA8888;
            s->stride = 0;
            s->invalidate = true;
            qemu_irq_lower(s->irq);
        }
        {
            /* BL-71-era debug: fires once per doorbell (very noisy). Gate it
             * behind SGI_GLACCEL_EXEC_DBG like the other opt-in device/renderer
             * knobs (GLRENDER_SO/SGI_GLACCEL_DUMP/...). Cached — read once. */
            static int exec_dbg = -1;
            if (exec_dbg < 0) {
                exec_dbg = getenv("SGI_GLACCEL_EXEC_DBG") ? 1 : 0;
            }
            if (exec_dbg) {
                fprintf(stderr, "pvgpu: EXEC val=0x%x cmd_base=0x%x cmd_len=%u\n",
                         (unsigned)val, s->cmd_base, s->cmd_len);
            }
        }
        if (val & GLACCEL_CMD_PROCESS) {
            /* Legacy EXEC path: process the current context's latched ring (used by the 2D
             * test + cmd-file). Don't raise IRQ — the driver polls STATUS_DONE. */
            uint32_t base = s->ctx[s->cur_ctx].cmd_base;
            if (base && s->cmd_len && s->cmd_len <= 64 * 1024 * 1024) {
                uint8_t *cmd = g_malloc(s->cmd_len + PVGPU_CMD_SLACK);
                memset(cmd + s->cmd_len, 0, PVGPU_CMD_SLACK);
                if (dma_memory_read(&address_space_memory, base, cmd, s->cmd_len,
                                    MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                    pvgpu_exec(s, cmd, s->cmd_len, base);
                }
                g_free(cmd);
            } else if (s->fb_base && s->width && s->height) {
                sgi_glaccel_update(s);   /* legacy dumb-framebuffer path */
            }
            s->status |= GLACCEL_STATUS_DONE;
        }
        break;
    case SGI_GLACCEL_DOORBELL: {
        /* Atomic per-window submit (race-free vs other windows): the single write carries
         * the context id and byte length; the ring base was latched per context. */
        int c = (val >> 24) & 0xff;
        uint32_t glen = val & 0x00FFFFFFu, base;
        if (c >= PVGPU_MAXCTX) break;
        s->cur_ctx = c;
        base = s->ctx[c].cmd_base;
        if (base && glen && glen <= 64 * 1024 * 1024) {
            uint8_t *cmd = g_malloc(glen + PVGPU_CMD_SLACK);
            memset(cmd + glen, 0, PVGPU_CMD_SLACK);
            if (dma_memory_read(&address_space_memory, base, cmd, glen,
                                MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                pvgpu_exec(s, cmd, glen, base);
            }
            g_free(cmd);
        }
        s->status |= GLACCEL_STATUS_DONE;
        break;
    }
    case SGI_GLACCEL_CTX_FREE: {
        /* Phase B: guest unmaps ring -> explicit context teardown.
         * Fired on munmap, process exit, AND kill -9 (ddmap.h last-reference). */
        int c = (int)val;
        if (c < 0 || c >= PVGPU_MAXCTX) break;
        {
            PVGPUCtx *ctx = &s->ctx[c];
            if (ctx->active || ctx->frame) {
                qemu_log_mask(LOG_UNIMP, "pvgpu: CTX_FREE ctx %d (was active=%d w=%d h=%d)\n",
                              c, ctx->active, ctx->w, ctx->h);
            }
            ctx->active = false;
            ctx->idle_warned = false;
            g_free(ctx->frame);  ctx->frame = NULL;
            ctx->w = ctx->h = ctx->x = ctx->y = 0;
            ctx->n_occ = 0;
            ctx->pending = 0;
            ctx->frame_serial = 0;
            ctx->cmd_base = 0;
            ctx->bound_xid = 0;
            /* Window model lifetime: if the PUBLISHER's ring context went away the
             * server itself is gone (Xsgi exit/respawn).  X window ids are recycled
             * across a server restart, so a surviving model could resolve a NEW
             * client's binding against a DEAD window's rectangle — the worst failure
             * this design can produce (memo 21 §6 risk 1).  Invalidate wholesale. */
            if (s->wm_pub_ctx == c) {
                qemu_log_mask(LOG_UNIMP, "pvgpu: window-model publisher ctx %d freed "
                              "-> model invalidated (gen %u -> 0)\n", c, s->wm_gen);
                s->wm_n = 0;
                s->wm_shadow_n = 0;
                s->wm_gen = 0;
                s->wm_batch_open = false;
                s->wm_pub_ctx = -1;
                for (int k = 0; k < PVGPU_MAXCTX; k++) s->ctx[k].bound_xid = 0;
                /* a wholesale invalidation always changes what the composite reads,
                 * and BL-81 cannot bound it: every modelled window's placement is
                 * being discarded at once.  Stale rects would be worse than useless,
                 * so drop them and keep the whole-desktop repaint. */
                s->wm_sig++;
                s->wm_n_rrects = 0;
                s->wm_repaint = false;
                s->wm_repaint_full = false;
                s->invalidate = true;
                glaccel_wm_request_present(s);
            }
            if (ctx->fwd_fd >= 0) { close(ctx->fwd_fd); ctx->fwd_fd = -1; }
            if (ctx->conn_fd >= 0) { close(ctx->conn_fd); ctx->conn_fd = -1; }
            if (ctx->rxbuf) { g_byte_array_unref(ctx->rxbuf); ctx->rxbuf = NULL; }
            /* BL-73: tear down the HOST renderer's per-context frame too.  The
             * in-process renderer keeps ONE global GL context + `last_frame`; if we
             * only free the device-side PVGPUCtx, a context id reused by the NEXT
             * guest GL client inherits this dead client's final frame (the device
             * composites the stale readback at the new window's position — Cycles'
             * splash re-appearing over roam/sphere_demo).  Invalidate it at the
             * source so the reused context starts frame-less until it renders its
             * own.  Optional symbol: an older libglrender without it simply keeps
             * the pre-fix behaviour. */
            if (glaccel_inproc_load() && gi_free_ctx) {
                gi_free_ctx(c);
            }
            s->invalidate = true;
        }
        break;
    }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad register offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps sgi_glaccel_ops = {
    .read = sgi_glaccel_read,
    .write = sgi_glaccel_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const GraphicHwOps sgi_glaccel_hw_ops = {
    .gfx_update = sgi_glaccel_update,
    .invalidate = sgi_glaccel_invalidate,
};

/* singleton for the Newport desktop overlay (one paravirtual GPU per machine) */
static SGIGLAccelState *g_glaccel_overlay;

/* Overlay-idle diagnostic threshold.  History: an idle *reaper* once cleared an overlay
 * that stopped submitting frames, on the theory the device couldn't see a guest GL process
 * die.  BL-76 removed the reaping: CTX_FREE (guest driver pvgpuunmap on the ring's ddmap
 * last-reference — munmap, exit, AND kill -9) now tears the context down explicitly and
 * reliably, so the idle timeout is redundant.  Worse, it was actively wrong for a
 * *single-buffered* SoXt/Inventor viewer, which legitimately renders ONE frame (a single
 * glFlush, no glXSwapBuffers) and then sits idle in its event loop, alive — the 5 s reaper
 * made its composited window vanish (revert to bare Motif orange) the instant anything else
 * repainted the desktop.  A frame must persist until CTX_FREE, exactly like an animating
 * double-buffered one.  The threshold is now diagnostic ONLY: it logs a one-shot warning
 * (a firing backstop signals a broken CTX_FREE path) but never clears the overlay. */
#define GLACCEL_OVERLAY_IDLE_US (5000 * 1000)

int sgi_glaccel_get_windows(PVGPUWindow *out, int max)
{
    SGIGLAccelState *s = g_glaccel_overlay;
    int64_t now;
    int n = 0, i, k;
    if (!s) return 0;
    now = g_get_monotonic_time();
    for (i = 0; i < PVGPU_MAXCTX && n < max; i++) {
        PVGPUCtx *c = &s->ctx[i];
        if (!c->active || !c->frame || c->w <= 0 || c->h <= 0) continue;
        if (!c->idle_warned && now - c->last_us > GLACCEL_OVERLAY_IDLE_US) {
            /* BL-76: diagnostic only — the overlay persists (single-buffered viewers are
             * legitimately idle after one frame); a firing warning means CTX_FREE broke. */
            qemu_log_mask(LOG_UNIMP, "pvgpu: ctx %d idle >%.1fs (single-buffered overlay; "
                          "retained until CTX_FREE)\n", i, (now - c->last_us) / 1e6);
            c->idle_warned = true;
        }
        out[n].frame = c->frame; out[n].x = c->x; out[n].y = c->y;
        out[n].w = c->w; out[n].h = c->h;
        out[n].n_occ = c->n_occ < PVGPU_MAX_OCC ? c->n_occ : PVGPU_MAX_OCC;
        for (k = 0; k < out[n].n_occ; k++) {
            out[n].occ[k][0] = c->occ[k][0]; out[n].occ[k][1] = c->occ[k][1];
            out[n].occ[k][2] = c->occ[k][2]; out[n].occ[k][3] = c->occ[k][3];
        }
        n++;
    }
    return n;
}

/* ---- unified display engine (Phase A) ---- */

void sgi_glaccel_register_desktop(PVDeskRenderFn render, PVDeskInvalidateFn invalidate,
                                  PVDeskScanoutFn scanout, PVDeskDamageFn damage,
                                  PVDeskRepaintFn repaint,
                                  void *opaque, int w, int h)
{
    SGIGLAccelState *s = g_glaccel_overlay;
    assert(s);                       /* glaccel must be realized first */
    assert(!s->desk_render);         /* only one desktop layer */
    assert(w > 0 && h > 0);
    s->desk_render = render;
    s->desk_invalidate = invalidate;
    s->desk_scanout = scanout;
    s->desk_damage = damage;
    s->desk_repaint = repaint;       /* BL-81; NULL => pre-BL-81 force_full path */
    s->desk_opaque = opaque;
    s->desk_w = w;  s->desk_h = h;
    s->desk = g_new0(uint32_t, (size_t)w * h);
    s->invalidate = true;            /* force full first render */
}

QemuConsole *sgi_glaccel_get_console(void)
{
    SGIGLAccelState *s = g_glaccel_overlay;
    assert(s && s->con);
    return s->con;
}

/* Per-frame GL-overlay composite: copy active frames + occluder rects from per-context
 * state into the on-stack window array, then composite each over *desk.  Returns the
 * number of visible windows composited. */
/* Clamp a window rect [x,y,w,h] to the screen and store it in *out (Phase D:
 * so the caller can bound its blit/dpy_gfx_update to composited window rects). */
static void glaccel_clamp_rect(PVDeskRect *out, int x, int y, int wpx, int hpx,
                               int dw, int dh)
{
    int x0 = x, y0 = y, x1 = x + wpx, y1 = y + hpx;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > dw) x1 = dw;
    if (y1 > dh) y1 = dh;
    out->x = x0; out->y = y0;
    out->w = (x1 > x0) ? x1 - x0 : 0;
    out->h = (y1 > y0) ? y1 - y0 : 0;
}

/* Window model lookup.  Returns NULL — meaning "no model for this context, use the
 * shim-published origin + occluders" — unless a live model is being consumed AND the
 * context carries a binding AND that binding resolves.  An unresolvable or suspect
 * binding must NEVER fall back to a best guess: compositing a LIVE frame into another
 * window's rectangle is strictly worse than today's stale-but-self-consistent position
 * (memo 21 §6 risk 1). */
static const PVGPUWinRec *glaccel_wm_find(SGIGLAccelState *s, uint32_t xid)
{
    int i;
    if (!s->winmodel || s->wm_gen == 0 || xid == 0) return NULL;
    for (i = 0; i < s->wm_n; i++) {
        if (s->wm[i].xid == xid) return &s->wm[i];
    }
    return NULL;
}

static int glaccel_composite_overlays(SGIGLAccelState *s, uint32_t *desk,
                                      int dw, int dh,
                                      PVDeskRect *out_rects, int max_out)
{
    PVGPUWindow wins[PVGPU_MAXCTX];
    int n_wins, wi, gy, gx, oi;
    int64_t now = g_get_monotonic_time();

    /* gather active windows, applying the idle timeout */
    n_wins = 0;
    for (int i = 0; i < PVGPU_MAXCTX && n_wins < PVGPU_MAXCTX; i++) {
        PVGPUCtx *c = &s->ctx[i];
        if (!c->active || !c->frame || c->w <= 0 || c->h <= 0) continue;
        if (!c->idle_warned && now - c->last_us > GLACCEL_OVERLAY_IDLE_US) {
            /* BL-76: diagnostic only — do NOT reap (see GLACCEL_OVERLAY_IDLE_US note). */
            qemu_log_mask(LOG_UNIMP, "pvgpu: ctx %d idle >%.1fs in composite "
                          "(single-buffered overlay retained until CTX_FREE)\n",
                          i, (now - c->last_us) / 1e6);
            c->idle_warned = true;
        }
        {
            const PVGPUWinRec *m = glaccel_wm_find(s, c->bound_xid);
            PVGPUWindow *W = &wins[n_wins];
            W->frame = c->frame; W->w = c->w; W->h = c->h;
            W->n_clip = 0; W->n_occ = 0; W->stack = i;
            if (m) {
                /* The model is authoritative: placement and visibility resolved HERE,
                 * at composite time, not at submit time.  The frame is content only.
                 * A window the server reports as not visible composites nothing. */
                if (!(m->flags & PVGPU_WMF_MAPPED) || m->n_clip <= 0) continue;
                W->x = m->x; W->y = m->y;
                W->stack = (int)m->stack;
                W->n_clip = m->n_clip < PVGPU_MAX_WM_CLIP ? m->n_clip : PVGPU_MAX_WM_CLIP;
                for (int k = 0; k < W->n_clip; k++) {
                    W->clip[k][0] = m->clip[k][0]; W->clip[k][1] = m->clip[k][1];
                    W->clip[k][2] = m->clip[k][2]; W->clip[k][3] = m->clip[k][3];
                }
            } else {
                W->x = c->x; W->y = c->y;
                W->n_occ = c->n_occ < PVGPU_MAX_OCC ? c->n_occ : PVGPU_MAX_OCC;
                for (int k = 0; k < W->n_occ; k++) {
                    W->occ[k][0] = c->occ[k][0]; W->occ[k][1] = c->occ[k][1];
                    W->occ[k][2] = c->occ[k][2]; W->occ[k][3] = c->occ[k][3];
                }
            }
            n_wins++;
        }
    }

    /* Composite order.  Without a model this is ctx-allocation order (the historical
     * behaviour, and the ctx-index-stacking defect of memo 21 §2.4 item 2); with one it
     * is the server's own bottom-to-top stacking index.  Insertion sort on the key is
     * stable, so a mixed desktop (some contexts modelled, some not) keeps the unmodelled
     * ones in their old relative order. */
    for (wi = 1; wi < n_wins; wi++) {
        PVGPUWindow tmp = wins[wi];
        int j = wi - 1;
        while (j >= 0 && wins[j].stack > tmp.stack) { wins[j + 1] = wins[j]; j--; }
        wins[j + 1] = tmp;
    }

    /* composite each window over the desktop */
    for (wi = 0; wi < n_wins; wi++) {
        PVGPUWindow *w = &wins[wi];
        for (gy = 0; gy < w->h; gy++) {
            int fy = w->y + gy;
            if (fy < 0 || fy >= dh) continue;
            for (gx = 0; gx < w->w; gx++) {
                int fx = w->x + gx;
                bool vis;
                if (fx < 0 || fx >= dw) continue;
                if (w->n_clip > 0) {
                    /* modelled: the pixel is visible iff it lies in the server's
                     * clip region (which already accounts for ancestors, siblings
                     * above, and a frame larger than its window). */
                    vis = false;
                    for (oi = 0; oi < w->n_clip; oi++) {
                        if (fx >= w->clip[oi][0] && fx < w->clip[oi][0] + w->clip[oi][2] &&
                            fy >= w->clip[oi][1] && fy < w->clip[oi][1] + w->clip[oi][3]) {
                            vis = true; break;
                        }
                    }
                } else {
                    /* occluder check: skip if pixel is inside any occluding rect */
                    vis = true;
                    for (oi = 0; oi < w->n_occ; oi++) {
                        if (fx >= w->occ[oi][0] && fx < w->occ[oi][0] + w->occ[oi][2] &&
                            fy >= w->occ[oi][1] && fy < w->occ[oi][1] + w->occ[oi][3]) {
                            vis = false; break;
                        }
                    }
                }
                if (vis) {
                    desk[(size_t)fy * dw + fx] = w->frame[(size_t)gy * w->w + gx];
                }
            }
        }
        if (out_rects && wi < max_out) {
            glaccel_clamp_rect(&out_rects[wi], w->x, w->y, w->w, w->h, dw, dh);
        }
    }
    return n_wins;
}

static void sgi_glaccel_realize(DeviceState *dev, Error **errp)
{
    SGIGLAccelState *s = SGI_GLACCEL(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &sgi_glaccel_ops, s,
                          "sgi-glaccel", SGI_GLACCEL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);

    g_glaccel_overlay = s;
    s->con = graphic_console_init(dev, 0, &sgi_glaccel_hw_ops, s);
    s->format = GLACCEL_FMT_RGBA8888;
    s->invalidate = true;
    /* BL-76 recomposite-on-delivery: present a freshly-applied GL frame from the
     * main loop (see glaccel_present_bh / glaccel_apply_frame). */
    s->present_bh = qemu_bh_new(glaccel_present_bh, s);

    /* live GL frame channel — one accept socket, per-context connections */
    s->gl_listen_fd = -1;
    s->cur_ctx = 0;
    s->wm_pub_ctx = -1;   /* no window-model publisher yet */
    /* window-model present coalescer: at most one model-driven composite per
     * display refresh (the model STATE is always committed synchronously). */
    s->wm_present_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                       glaccel_wm_present_timer_cb, s);
    s->wm_last_present_ns = 0;
    s->wm_present_pending = false;
    {
        int i;
        for (i = 0; i < PVGPU_MAXCTX; i++) {
            s->ctx[i].dev = s;
            s->ctx[i].fwd_fd = -1;
            s->ctx[i].conn_fd = -1;
            s->ctx[i].rxbuf = NULL;
        }
    }
    if (s->gl_listen && s->gl_listen[0] && strcmp(s->gl_listen, "off") != 0) {
        int port = atoi(s->gl_listen);
        if (port > 0 && port < 65536) {
            glaccel_gl_listen(s, port, errp);
        }
    }
}

/* test hook: qom-set "cmd-file" to a file of pvgpu commands; executes them immediately so
 * the device's 2D/3D path can be validated host-side before the guest kernel driver exists. */
static void glaccel_set_cmd_file(Object *obj, const char *value, Error **errp)
{
    SGIGLAccelState *s = SGI_GLACCEL(obj);
    GError *gerr = NULL;
    gchar *data = NULL; gsize len = 0;
    if (!value || !value[0]) {
        return;
    }
    if (!g_file_get_contents(value, &data, &len, &gerr)) {
        error_setg(errp, "cmd-file: %s", gerr ? gerr->message : "read failed");
        if (gerr) g_error_free(gerr);
        return;
    }
    pvgpu_exec(s, (const uint8_t *)data, (uint32_t)len, 0);
    g_free(data);
}

static const Property sgi_glaccel_props[] = {
    DEFINE_PROP_STRING("gl-listen", SGIGLAccelState, gl_listen),
    DEFINE_PROP_STRING("gl-forward", SGIGLAccelState, gl_forward),
    /* IN-PROCESS GL end-goal gate (default OFF): only the launch that sets this uses the
     * in-process glr_submit path; every stock app keeps the proven socket/Newport path. */
    DEFINE_PROP_BOOL("inproc", SGIGLAccelState, inproc, true),
    /* Server-published window model (memo 21).  Default ON since the bounded
     * invalidation chain landed (notes 27/28/29): the WM_* ring ops are consumed,
     * so a GL window's placement, stacking, MAPPED state and clip come from the X
     * server's own view instead of the client's submit-time origin — which is what
     * makes a frame follow its window (the M3 defect) instead of staying where it
     * was drawn.
     *
     * It defaulted OFF for three legs because the model's correctness came at a
     * measured cost: it roughly doubles the number of desktop presents, and while
     * every present was a whole-desktop re-walk that was +7.5 points of host CPU on
     * a resize storm (26-…md §2).  With the repaint and the invalidation both
     * bounded, a present is now proportional to the window: on the same GL-bound
     * A/B (medians of 3, 600 ops/phase, an animating client bound to two on-screen
     * windows) ON vs publisher-only is resize +2.1 points, move -2.0 and
     * whole-bench +1.1, against a ±4-point arm-to-arm drift bound and a flat
     * ctl_prop floor (+0.7) — where the same rig measured +8.9 / +4.3 / +6.0 before
     * this chain.
     *
     * -global sgi-glaccel.winmodel=off restores the pre-model composite (submit-time
     * origin + occluder rects) exactly; it stays the device-side kill-switch of
     * memo 21 §4 and the control arm of every A/B above. */
    DEFINE_PROP_BOOL("winmodel", SGIGLAccelState, winmodel, true),
};

static void sgi_glaccel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = sgi_glaccel_realize;
    device_class_set_props(dc, sgi_glaccel_props);
    object_class_property_add_str(klass, "cmd-file", NULL, glaccel_set_cmd_file);
}

static const TypeInfo sgi_glaccel_info = {
    .name = TYPE_SGI_GLACCEL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGLAccelState),
    .class_init = sgi_glaccel_class_init,
};

static void sgi_glaccel_register_types(void)
{
    type_register_static(&sgi_glaccel_info);
}

type_init(sgi_glaccel_register_types)
