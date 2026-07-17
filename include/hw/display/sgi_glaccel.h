#ifndef SGI_GLACCEL_H
#define SGI_GLACCEL_H

#include "exec/cpu-common.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_SGI_GLACCEL "sgi-glaccel"
OBJECT_DECLARE_SIMPLE_TYPE(SGIGLAccelState, SGI_GLACCEL)

#define SGI_GLACCEL_BASE_ADDR 0x1f480300
#define SGI_GLACCEL_IRQ 7

/* Register Offsets (all 4-byte aligned, big-endian) */
#define SGI_GLACCEL_STATUS   0x00
#define SGI_GLACCEL_WIDTH    0x04
#define SGI_GLACCEL_HEIGHT   0x08
#define SGI_GLACCEL_CMD_BASE 0x0C
#define SGI_GLACCEL_CMD_LEN  0x10
#define SGI_GLACCEL_FB_BASE  0x14
#define SGI_GLACCEL_EXEC     0x18
#define SGI_GLACCEL_FORMAT   0x1C   /* Pixel format: 0=RGBA8888, 1=RGB565 */
#define SGI_GLACCEL_STRIDE   0x20   /* Bytes per scanline (0 = width*bpp) */
#define SGI_GLACCEL_CONTEXT  0x24   /* GL context (window) id; a CMD_BASE write latches per ctx */
#define SGI_GLACCEL_DOORBELL 0x28   /* atomic submit: write (ctx<<24)|len -> process ctx's ring */
#define SGI_GLACCEL_CTX_FREE 0x2C   /* Phase B: write ctx id -> free context (unmap lifecycle) */

/* Execution Commands */
#define GLACCEL_CMD_RESET   (1 << 0)
#define GLACCEL_CMD_PROCESS (1 << 1)

/* ---- paravirtual GPU command-ring opcodes (2D + 3D acceleration) ----
 * The guest writes a command buffer (sequence of these, big-endian = guest native) to a
 * DMA region, programs CMD_BASE/CMD_LEN, then writes EXEC=GLACCEL_CMD_PROCESS. The device
 * executes them into its internal framebuffer: 2D ops are host memcpy/memset (no per-pixel
 * REX3 emulation); GL ops are forwarded to the host renderer and composited. */
#define PVGPU_OP_CLEAR   1   /* [op][rgba] : fill whole fb */
#define PVGPU_OP_FILL    2   /* [op][x][y][w][h][rgba] : fill rect */
#define PVGPU_OP_COPY    3   /* [op][sx][sy][dx][dy][w][h] : copy rect within fb */
#define PVGPU_OP_BLIT    4   /* [op][x][y][w][h] then w*h u32 rgba : blit image */
#define PVGPU_OP_GL      5   /* [op][x][y][w][h][gllen] then gllen bytes glproto (pad 4) */
#define PVGPU_OP_PRESENT 6   /* [op] : present the framebuffer to the console */
#define PVGPU_OP_WINCLIP 7   /* [op][n] then n*[x][y][w][h] : screen rects of windows stacked
                              * ABOVE the GL window that occlude it (desktop overlay clipping) */
#define PVGPU_OP_SCANOUT_SET 8  /* Stage 2: [op][base_lo][base_hi][w][h][stride][format]
                                 * register shadow framebuffer in guest RAM */
#define PVGPU_OP_DAMAGE      9  /* Stage 2: [op][x][y][w][h] — shadowfb region changed */

#define PVGPU_MAX_OCC 16     /* max occluder rects tracked for desktop overlay clipping */
#define PVGPU_MAXCTX  8      /* max concurrent GL windows (contexts) */

/* ---- desktop layer types (used by SGIGLAccelState, must precede it) ---- */
typedef struct { int x, y, w, h; } PVDeskRect;

/* Max damage rects the desktop render reports back per frame (Phase D). */
#define PVDESK_MAX_RECTS 16
/* Render callback return sentinel: the whole screen changed (repaint everything). */
#define PVDESK_FULL      (-1)

/* Desktop render callback (Phase D dirty-rect scanout): converts the changed
 * regions of the desktop into *dst (w*h xRGB32).  When force_full is true the
 * desktop MUST repaint every pixel (the caller discarded the previous buffer).
 * Otherwise it repaints only the regions dirtied since the last call and reports
 * them in rects[0..max_rects) (post-window-offset screen space, == dst space).
 * Return value:
 *   PVDESK_FULL (-1) — the whole screen was repainted (rects[] undefined);
 *   0               — nothing changed (dst untouched, rects[] undefined);
 *   n > 0           — n dirty rects written to rects[], only those were repainted. */
typedef int (*PVDeskRenderFn)(void *opaque, uint32_t *dst, int w, int h,
                              bool force_full, PVDeskRect *rects, int max_rects);
/* Desktop invalidate: called when the engine needs the desktop to invalidate its cache
 * (surface reallocation, display-backend switch, etc.). */
typedef void (*PVDeskInvalidateFn)(void *opaque);

/* Stage 2 Phase 2a — paravirtual shadowfb scanout callbacks.
 * The pvgpu ring ops SCANOUT_SET/DAMAGE arrive on the glaccel device, but the
 * base layer's DID/XMAP/CMAP walk lives in the desktop renderer (Newport).  So
 * the engine forwards them straight to the desktop layer: it registers a shadow
 * framebuffer in guest RAM (SCANOUT_SET) whose CI8 rows the renderer sources
 * INSTEAD of VRAM's rgbci plane — keeping per-window DID palettes, popup/overlay
 * cidaux compositing and colormap semantics intact by construction (per the
 * director's Phase 2 design review: keep the DID walk, swap only the pixel
 * source; there is deliberately NO global CI8->xRGB LUT). */
typedef void (*PVDeskScanoutFn)(void *opaque, uint64_t base, uint32_t w,
                                uint32_t h, uint32_t stride, uint32_t fmt,
                                bool active);
typedef void (*PVDeskDamageFn)(void *opaque, int x, int y, int w, int h);

/* Per-GL-window (per-context) device state: its own glserver forward connection, its own
 * gl-listen frame connection, its latest frame, screen placement, and occluder rects. */
typedef struct PVGPUCtx {
    SGIGLAccelState *dev;        /* back-pointer (fd-handler opaque); idx = this - dev->ctx */
    uint32_t    cmd_base;        /* this context's command-ring phys base (latched once) */
    int         fwd_fd;          /* forward connection to glserver (this context) */
    int         pending;         /* fire-and-forget replies not yet drained (async) */
    int         conn_fd;         /* gl-listen frame connection from this context's renderer */
    GByteArray *rxbuf;           /* frame reassembly buffer for conn_fd */
    uint32_t   *frame;           /* latest frame, host xRGB */
    int         w, h;            /* frame dimensions */
    int         x, y;            /* tracked screen placement (window origin) */
    bool        active;          /* a frame is present */
    int64_t     last_us;         /* monotonic time of last frame (overlay idle diagnostic) */
    bool        idle_warned;     /* BL-76: one-shot "idle backstop" diagnostic already logged */
    int         occ[PVGPU_MAX_OCC][4]; /* screen rects occluding this window */
    int         n_occ;
    int         frame_serial;    /* in-process path: last glr_get_last_frame serial composited */
} PVGPUCtx;

/* One active GL window for the Newport desktop multi-composite. */
typedef struct PVGPUWindow {
    uint32_t *frame;
    int x, y, w, h;
    int occ[PVGPU_MAX_OCC][4];
    int n_occ;
} PVGPUWindow;

/* Pixel formats */
#define GLACCEL_FMT_RGBA8888 0
#define GLACCEL_FMT_RGB565   1

/* Status bits */
#define GLACCEL_STATUS_DONE  (1 << 0)

struct SGIGLAccelState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    QemuConsole *con;

    /* Registers */
    uint32_t status;
    uint32_t width;
    uint32_t height;
    uint32_t cmd_base;
    uint32_t cmd_len;
    uint32_t fb_base;
    uint32_t format;
    uint32_t stride;

    /* Internal state */
    bool invalidate;

    /* --- live GL frame channel (host renderer -> device -> console) ---
     * The host renderer (glserver) connects and streams PVGL frames; the device
     * composites the latest one. Protocol (little-endian):
     *   "PVGL"(4) + int32 x,y,w,h ; then w*h*4 RGBA bytes.  w==0 clears.  */
    char       *gl_listen;        /* qom prop: "<port>" to listen on 127.0.0.1:port */
    int         gl_listen_fd;     /* one accept socket; the ctx-id handshake routes per window */

    /* Per-context (per GL window) state — multiple GL apps run concurrently, each with its
     * own command ring (driver), glserver connection, frame, placement and occluders. */
    PVGPUCtx    ctx[PVGPU_MAXCTX];
    int         cur_ctx;          /* selected by the CONTEXT register before EXEC */

    /* --- paravirtual GPU: internal framebuffer + command ring --- *
     * 2D ops draw here via host memcpy; GL frames composite here; gfx_update presents it. */
    uint32_t   *fb;               /* internal framebuffer, host xRGB, fb_w*fb_h */
    int         fb_w, fb_h;       /* framebuffer dimensions */
    bool        fb_active;        /* fb has been drawn (present it instead of legacy paths) */
    char       *gl_forward;       /* qom prop: "<port>" of host glserver to forward GL ops to */
    char       *cmd_file;         /* qom prop (test): a file of commands to execute now */

    /* IN-PROCESS host-GPU GL (CLAUDE.md "Host-GPU GL push" END-GOAL, NOT scaffolding).
     * PVGPU_OP_GL calls glr_submit() directly via a dlopen'd renderer .so (no glserver
     * socket) — the virtio-gpu/virgl-shaped destination. Default ON (all GL apps use the
     * in-process path); set sgi-glaccel.inproc=off to fall back to the socket/glserver
     * path for debugging or if the renderer .so is unavailable. */
    bool        inproc;           /* qom prop: in-process glr_submit instead of the socket */

    /* ---- unified display engine (Phase A) ---- */
    uint32_t   *desk;             /* engine-owned desktop buffer, xRGB, desk_w*desk_h */
    int         desk_w, desk_h;   /* desktop resolution (e.g. 1280x1024) */
    PVDeskRenderFn      desk_render;      /* desktop render callback */
    PVDeskInvalidateFn  desk_invalidate;  /* desktop invalidate callback */
    PVDeskScanoutFn     desk_scanout;     /* Stage 2: register shadow fb (SCANOUT_SET) */
    PVDeskDamageFn      desk_damage;      /* Stage 2: shadow fb damage (DAMAGE) */
    void               *desk_opaque;      /* opaque for the desktop callbacks */
    int         prev_n_windows;   /* one-frame GL-overlay restore: count of windows last frame */
    uint64_t    last_composite_sig; /* signature of last frame's GL overlay set (idle-skip) */
    bool        have_composite_sig; /* last_composite_sig is valid (a frame was blitted) */
    /* Phase D bounded scanout: previous frame's GL window rects (for detecting
     * geometry changes that need a full desktop restore vs. content-only frames
     * that only reblit the window rect). */
    PVDeskRect  prev_win_rects[PVGPU_MAXCTX];
    int         prev_n_win_rects;

    /* ---- Stage 2 Phase 2a shadow framebuffer (Variant B) ----
     * The engine only latches the SCANOUT_SET parameters for debug/echo; the
     * authoritative shadowfb state and the DID-walk pixel sourcing live in the
     * desktop renderer (Newport), reached via desk_scanout/desk_damage.  There
     * is deliberately NO shadow_cmap LUT here (see the callback typedefs). */
    uint64_t    shadow_base;       /* guest physical address of shadow fb */
    uint32_t    shadow_w, shadow_h, shadow_stride;
    uint32_t    shadow_format;     /* 0=CI8, 1=xRGB */
    bool        shadow_active;     /* shadowfb registered */
};

/* Desktop overlay: fill out[] with every active host-rendered GL window (frame + tracked
 * screen placement + occluder rects) so the Newport desktop device can composite them into
 * the live framebuffer — making GL apps (atlantis etc.) appear as windows on the 4Dwm desktop
 * rather than in the glaccel device's own console. Returns the count of active windows.
 * frame pixels are host xRGB (0x00RRGGBB). */
int sgi_glaccel_get_windows(PVGPUWindow *out, int max);

/* Register a desktop renderer with the unified display engine.  The engine owns the console
 * after registration; Newport (or any future desktop device) demotes itself to a renderer.
 * Only one desktop layer may be registered.  w/h must be the native desktop resolution. */
void sgi_glaccel_register_desktop(PVDeskRenderFn render, PVDeskInvalidateFn invalidate,
                                  PVDeskScanoutFn scanout, PVDeskDamageFn damage,
                                  void *opaque, int w, int h);

/* Phase E: return the unified engine's console for hardware-cursor setup.
 * Only valid after register_desktop (asserts otherwise). */
QemuConsole *sgi_glaccel_get_console(void);

#endif /* SGI_GLACCEL_H */
