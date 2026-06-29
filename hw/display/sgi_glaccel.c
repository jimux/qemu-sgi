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

static glr_submit_fn          gi_submit;
static glr_set_ctxid_fn        gi_set_ctxid;
static glr_get_last_frame_fn   gi_get_last_frame;
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
static void glaccel_apply_frame(PVGPUCtx *c, int x, int y, int w, int h,
                                const uint8_t *rgba)
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
    c->last_us = g_get_monotonic_time();   /* for the exit/idle staleness timeout */
    if (c->dev) c->dev->invalidate = true;

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
        glaccel_apply_frame(c, x, y, w, h, rx->data + 20);
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

    /* pull the just-rendered frame (top-down RGB) and present it via the shared compositor —
     * but only when the renderer's frame serial advanced (one completed frame per SWAPBUFFERS),
     * so we match the socket path's cadence and never composite a stale mid-frame readback. */
    serial = gi_get_last_frame(cid, &rgb, &fw, &fh);
    if (serial && serial != c->frame_serial && rgb && fw > 0 && fh > 0) {
        c->frame_serial = serial;
        uint8_t *rgba = g_malloc((size_t)fw * fh * 4);
        size_t i, n = (size_t)fw * fh;
        for (i = 0; i < n; i++) {
            rgba[i * 4 + 0] = rgb[i * 3 + 0];
            rgba[i * 4 + 1] = rgb[i * 3 + 1];
            rgba[i * 4 + 2] = rgb[i * 3 + 2];
            rgba[i * 4 + 3] = 0xff;
        }
        glaccel_apply_frame(c, c->x, c->y, fw, fh, rgba);   /* same path as gl-listen frames */
        g_free(rgba);
    }

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

/* execute a command buffer into the internal framebuffer. resp_base != 0 => DMA each OP_GL's
 * reply back to guest RAM at resp_base + align16(len) (the doorbell path; 0 for cmd-file). */
static void pvgpu_exec(SGIGLAccelState *s, const uint8_t *buf, uint32_t len, uint32_t resp_base) {
    uint32_t p = 0;
    PVGPUCtx *wc = &s->ctx[s->cur_ctx < PVGPU_MAXCTX ? s->cur_ctx : 0];   /* this window */
    fprintf(stderr, "pvgpu: exec ctx=%d %u bytes first_op=%u\n",
            (int)(wc - s->ctx), len, len >= 4 ? pv_be32(buf) : 0);
    while (p + 4 <= len) {
        uint32_t op = pv_be32(buf + p); p += 4;
        switch (op) {
        case PVGPU_OP_CLEAR: {
            uint32_t c; size_t i;
            if (p + 4 > len) return;
            c = pv_be32(buf + p); p += 4;
            if (!s->fb) pvgpu_ensure_fb(s, s->width ? s->width : 800, s->height ? s->height : 600);
            if (s->fb) for (i = 0; i < (size_t)s->fb_w * s->fb_h; i++)
                s->fb[i] = ((c >> 24 & 0xff) << 16) | ((c >> 16 & 0xff) << 8) | (c >> 8 & 0xff);
            s->fb_active = true; s->invalidate = true; break;
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
            s->fb_active = true; s->invalidate = true; break;
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
            s->fb_active = true; s->invalidate = true; break;
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
            s->fb_active = true; s->invalidate = true; break;
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
            s->fb_active = true; s->invalidate = true; break;
        }
        case PVGPU_OP_WINCLIP: {
            uint32_t n, i;
            if (p + 4 > len) return;
            n = pv_be32(buf + p); p += 4;
            if (n > PVGPU_MAX_OCC) n = PVGPU_MAX_OCC;
            for (i = 0; i < n; i++) {
                if (p + 16 > len) { n = i; break; }
                wc->occ[i][0] = (int)pv_be32(buf+p);   wc->occ[i][1] = (int)pv_be32(buf+p+4);
                wc->occ[i][2] = (int)pv_be32(buf+p+8);  wc->occ[i][3] = (int)pv_be32(buf+p+12);
                p += 16;
            }
            wc->n_occ = n;
            break;
        }
        case PVGPU_OP_PRESENT:
            s->invalidate = true; break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "pvgpu: bad op %u at %u\n", op, p - 4);
            return;
        }
    }
}

/*
 * gfx_update callback — called at ~60Hz by the QEMU display subsystem.
 * DMA the guest framebuffer and blit to the console surface.
 */
static void sgi_glaccel_update(void *opaque)
{
    SGIGLAccelState *s = opaque;
    DisplaySurface *surface;
    int width, height, src_stride;

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
        fprintf(stderr, "pvgpu: EXEC val=0x%x cmd_base=0x%x cmd_len=%u\n",
                 (unsigned)val, s->cmd_base, s->cmd_len);
        if (val & GLACCEL_CMD_PROCESS) {
            /* Legacy EXEC path: process the current context's latched ring (used by the 2D
             * test + cmd-file). Don't raise IRQ — the driver polls STATUS_DONE. */
            uint32_t base = s->ctx[s->cur_ctx].cmd_base;
            if (base && s->cmd_len && s->cmd_len <= 64 * 1024 * 1024) {
                uint8_t *cmd = g_malloc(s->cmd_len);
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
            uint8_t *cmd = g_malloc(glen);
            if (dma_memory_read(&address_space_memory, base, cmd, glen,
                                MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
                pvgpu_exec(s, cmd, glen, base);
            }
            g_free(cmd);
        }
        s->status |= GLACCEL_STATUS_DONE;
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

/* A GL app that exits/is killed simply stops submitting frames (the device can't see the
 * guest process die). Treat the overlay as gone once frames stop for a short window, so the
 * desktop is restored instead of freezing on the last frame. (~1s: atlantis-class demos
 * animate continuously; a 1s gap means the window is gone or stalled.) */
#define GLACCEL_OVERLAY_IDLE_US (1200 * 1000)

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
        if (now - c->last_us > GLACCEL_OVERLAY_IDLE_US) {   /* app gone/stalled -> drop */
            c->active = false;
            continue;
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

    /* live GL frame channel — one accept socket, per-context connections */
    s->gl_listen_fd = -1;
    s->cur_ctx = 0;
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
