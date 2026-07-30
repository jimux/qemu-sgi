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
#define SGI_GLACCEL_WINMODEL 0x30   /* R: server-published window-model generation (0 = none) */

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

/* ---- server-published window model (memo 21 / pvdisplay 24) ----------------
 * The X server (parasite DDX) publishes an AUTHORITATIVE window model on its own
 * ring context; a GL client publishes only the binding (its ctx <-> its X window
 * id).  Neither publisher guesses.  When a model is live the composite resolves
 * placement + clip + stacking from it AT COMPOSITE TIME instead of from the
 * submit-time origin the frame carries.  Gated by the "winmodel" QOM property;
 * with no model (generation 0) every path is byte-identical to the pre-model
 * device.  See progress_notes/ip55/pvdisplay/24-winmodel-implementation.md. */
#define PVGPU_OP_WM_BEGIN   10  /* [op][gen] : open an atomic model update */
#define PVGPU_OP_WM_WINDOW  11  /* [op][xid][x][y][w][h][flags][stack] : one window record */
#define PVGPU_OP_WM_CLIP    12  /* [op][xid][n] then n*[x][y][w][h] : visible region, screen */
#define PVGPU_OP_WM_GONE    13  /* [op][xid] : window retired (unmapped/destroyed) */
#define PVGPU_OP_WM_END     14  /* [op][gen] : commit — swap the shadow model in */
#define PVGPU_OP_WM_BIND    15  /* [op][xid] : CLIENT ring — bind this context to that window */

#define PVGPU_WMF_MAPPED    (1u << 0)  /* window is viewable (has a non-empty clip) */
#define PVGPU_WMF_HASCLIP   (1u << 1)  /* a WM_CLIP for this xid follows in this batch */
#define PVGPU_WMF_OVERRIDE  (1u << 2)  /* override-redirect (informational) */

#define PVGPU_MAX_WM_WINDOWS 256 /* model capacity (a bigger batch is rejected wholesale).
                                  * memo 21 §3.1 proposed 64; a live 4Dwm desktop publishes
                                  * the whole tree INCLUDING every frame gadget (M1 logged 14
                                  * windows for ONE client) — measured batches on the baked
                                  * demo-runner desktop reach 5908 bytes, i.e. up to ~211
                                  * records.  64 (or 128) would mean "no model, ever" on a
                                  * real desktop; the publisher discards rather than truncates,
                                  * so the cap must have real headroom. */
#define PVGPU_MAX_WM_CLIP    32  /* per-window clip rects (publisher degrades to extents) */

#define PVGPU_MAX_OCC 16     /* max occluder rects tracked for desktop overlay clipping */
#define PVGPU_MAXCTX  8      /* max concurrent GL windows (contexts) */

/* One published window: screen rect, stacking index (0 = bottom) and visible region. */
typedef struct PVGPUWinRec {
    uint32_t xid;
    int      x, y, w, h;
    uint32_t flags;
    uint32_t stack;
    int      clip[PVGPU_MAX_WM_CLIP][4];
    int      n_clip;
} PVGPUWinRec;

/* ---- desktop layer types (used by SGIGLAccelState, must precede it) ---- */
typedef struct { int x, y, w, h; } PVDeskRect;

/* Max damage rects the desktop render reports back per frame (Phase D). */
/* Kept EQUAL to NEWPORT_DIRTY_MAX: newport_render_desktop() renders every dirty
 * rect but can only REPORT max_rects of them, and the engine blits only what is
 * reported — a smaller cap here would leave rendered-but-unblitted regions stale.
 * Raised 16 -> 64 with the bounded VC2/palette invalidation: once the direct
 * whole-screen invalidators stopped firing, the 16-slot list became the binding
 * saturator (81 % of frames still went FULL with dfull_total already down to 337).
 * Note 27 6d's "list size is irrelevant" refutation was true only while
 * newport_dirty_full() was assigning the saturated value on every table word. */
#define PVDESK_MAX_RECTS 64
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

/* BL-81 — bounded geometry repaint.  When a server-published window model moves
 * or resizes a GL window, the desktop UNDER the window's old rect must be
 * repainted (and the new rect re-walked), but nothing else needs to change.
 * Before BL-81 the engine expressed that as force_full=true on the render
 * callback, i.e. a whole-desktop re-walk plus a full-screen blit on EVERY
 * present of a resize/move storm (+7 points of host CPU, note 26 §5).
 *
 * This callback lets the engine express the same requirement as a bounded
 * request: "repaint at least these rects next render".  The desktop layer folds
 * them into whatever dirty-rect machinery it already has, so the next
 * render_desktop() returns them in rects[] and the engine bounded-blits them.
 *
 * Contract, in the safe direction: the desktop layer may repaint MORE than
 * asked (up to the whole screen — coalescing/saturation is always legal), never
 * less.  If a desktop layer does not implement this callback the engine keeps
 * the pre-BL-81 force_full path, so absence degrades to correct-but-slow. */
typedef void (*PVDeskRepaintFn)(void *opaque, const PVDeskRect *rects, int n);

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
    uint32_t    bound_xid;       /* window model: X window id this context renders into (0 = none) */
} PVGPUCtx;

/* One active GL window for the Newport desktop multi-composite. */
typedef struct PVGPUWindow {
    uint32_t *frame;
    int x, y, w, h;
    int occ[PVGPU_MAX_OCC][4];
    int n_occ;
    /* window model (n_clip > 0 => clip[] replaces the occluder test entirely) */
    int clip[PVGPU_MAX_WM_CLIP][4];
    int n_clip;
    int stack;                   /* composite order key (model stack, else ctx index) */
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

    /* BL-76 recomposite-on-delivery: a GL frame is read back / applied on the
     * doorbell (device-I/O) thread, out of band from any display refresh tick.
     * With -display none (batch grading) or an idle 4Dwm desktop nothing ticks
     * the gfx_update compositor, so a single-buffered one-shot frame (the static
     * Inventor viewers) lands but never reaches the surface until external
     * activity (screendump/xrefresh) forces graphic_hw_update.  This bottom-half
     * runs graphic_hw_update(con) on the main loop whenever a frame is applied,
     * so delivery itself drives the composite.  Idempotent scheduling coalesces
     * bursts (animating demos) into one present per main-loop iteration. */
    QEMUBH     *present_bh;

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

    /* ---- server-published window model (kill-switch: sgi-glaccel.winmodel) ----
     * wm[] is the LIVE model, wm_shadow[] the batch under construction.  A batch is
     * only ever swapped in whole (WM_END with a matching generation); a truncated,
     * over-capacity or mismatched batch is discarded and the previous model stays
     * live — the device never presents a half-state.  wm_gen == 0 means "no
     * authoritative model": every context falls back to its own submit-time origin
     * plus occluder rects, i.e. exactly the pre-model behaviour. */
    bool        winmodel;         /* qom prop: consume the model (default off) */
    PVGPUWinRec wm[PVGPU_MAX_WM_WINDOWS];
    int         wm_n;
    uint32_t    wm_gen;           /* live model generation (0 = none) */
    PVGPUWinRec wm_shadow[PVGPU_MAX_WM_WINDOWS];
    int         wm_shadow_n;
    uint32_t    wm_batch_gen;
    bool        wm_batch_open;
    bool        wm_batch_bad;
    int         wm_pub_ctx;       /* ring context the publisher owns (-1 = none) */

    /* ---- commit batching (24-…md §3 "where the batching should go") ----
     * 4Dwm revalidates its whole window tree constantly, so most committed batches
     * are bit-identical for every window a GL context is actually BOUND to.  Such a
     * republish cannot change a pixel and must therefore cost nothing: it bumps
     * neither wm_sig (so the composite idle-skip can still suppress the repaint) nor
     * schedules a present.  wm_sig — NOT wm_gen — is what the composite signature
     * folds: it advances only on a commit that changed a bound window's rect, clip,
     * stacking or liveness, or on a wholesale model invalidation.
     *
     * The present itself is coalesced to at most one per display refresh.  The model
     * STATE is always committed synchronously; only the present is deferred, and the
     * deferred composite reads the final committed state — so a burst of N publishes
     * costs one composite of the last one, never a dropped commit. */
    uint32_t    wm_sig;             /* commit serial: bumped only on a bound-visible change */
    QEMUTimer  *wm_present_timer;   /* drains a coalesced present at the next refresh */
    int64_t     wm_last_present_ns; /* REALTIME ns of the last model-driven present */
    bool        wm_present_pending; /* wm_present_timer is armed */
    /* counters (SGI_GLACCEL_WM_STATS=1 logs them periodically) */
    uint64_t    wm_commits;         /* committed batches */
    uint64_t    wm_commits_nop;     /* of which: no bound window changed => free */
    uint64_t    wm_presents;        /* model-driven presents actually issued */
    uint64_t    wm_presents_coalesced; /* presents folded into a pending one */

    /* ---- unified display engine (Phase A) ---- */
    uint32_t   *desk;             /* engine-owned desktop buffer, xRGB, desk_w*desk_h */
    int         desk_w, desk_h;   /* desktop resolution (e.g. 1280x1024) */
    PVDeskRenderFn      desk_render;      /* desktop render callback */
    PVDeskInvalidateFn  desk_invalidate;  /* desktop invalidate callback */
    PVDeskScanoutFn     desk_scanout;     /* Stage 2: register shadow fb (SCANOUT_SET) */
    PVDeskDamageFn      desk_damage;      /* Stage 2: shadow fb damage (DAMAGE) */
    PVDeskRepaintFn     desk_repaint;     /* BL-81: bounded geometry repaint request */
    void               *desk_opaque;      /* opaque for the desktop callbacks */
    int         prev_n_windows;   /* one-frame GL-overlay restore: count of windows last frame */
    uint64_t    last_composite_sig; /* signature of last frame's GL overlay set (idle-skip) */
    bool        have_composite_sig; /* last_composite_sig is valid (a frame was blitted) */
    /* Phase D bounded scanout: previous frame's GL window rects (for detecting
     * geometry changes that need a full desktop restore vs. content-only frames
     * that only reblit the window rect). */
    PVDeskRect  prev_win_rects[PVGPU_MAXCTX];
    int         prev_n_win_rects;
    /* BL-81 pending bounded repaint.  A model commit that changed a bound window
     * used to set s->invalidate (= repaint the whole desktop); it now records the
     * rects it can have altered here.  Accumulates across commits because the
     * present coalescer folds many WM_ENDs into one present; consumed and cleared
     * by the next sgi_glaccel_update().  wm_repaint_full is the fail-safe: set
     * whenever the changed region cannot be named, and it restores the old
     * whole-desktop behaviour for that frame. */
    PVDeskRect  wm_rrects[2 * PVGPU_MAXCTX];
    int         wm_n_rrects;
    bool        wm_repaint;
    bool        wm_repaint_full;
    /* BL-81 counters (SGI_GLACCEL_WM_STATS): how many changed frames took the
     * bounded repaint vs. fell back to the whole-desktop repaint. */
    uint64_t    geom_bounded_repaints;
    uint64_t    geom_full_repaints;
    /* PVDISPLAY_BOUND_VERIFY=1 differential oracle (debug only): scratch buffer for
     * the from-scratch reference render, and how many bounded frames disagreed with
     * it.  verify_bad_frames must be 0; anything else is a bounded-repaint miss. */
    uint32_t   *verify_buf;
    uint64_t    verify_frames;
    uint64_t    verify_bad_frames;
    uint64_t    verify_bad_pixels;
    /* PVDISPLAY_TIME_BREAKDOWN=1 (debug only): where the host CPU of a model-driven
     * present actually goes.  BL-81's A/B showed that bounding the desktop repaint
     * recovers only ~2 of the ~7.5 points, so the residual has to be attributed by
     * measurement rather than assumed.  All in ns. */
    uint64_t    t_desk_render;    /* the desktop layer's render callback */
    uint64_t    t_composite;      /* glaccel_composite_overlays (per-pixel + clip) */
    uint64_t    t_blit;           /* desk -> surface copy + dpy_gfx_update */
    uint64_t    t_wm_ops;         /* WM_* ring-op parse + commit + bound-changed */
    uint64_t    n_updates;        /* sgi_glaccel_update calls that did work */
    /* What the desktop layer ACTUALLY did, as opposed to what the engine asked for.
     * BL-81's attribution run found t_desk_render identical whether the engine
     * requested a bounded repaint or a full one, so the question "did the renderer
     * honour the bound, or saturate to full-screen anyway?" needs its own counter. */
    uint64_t    n_render_full;    /* render returned PVDESK_FULL */
    uint64_t    n_render_rects;   /* render returned n > 0 bounded rects */
    uint64_t    n_render_idle;    /* render returned 0 (nothing changed) */
    /* Note 29 — attribution of the blanket `s->invalidate` (= force_full) that
     * note 28's why_full(force=766) counted as the largest remaining cause of
     * full desktop renders.  Which content op asked, and how many of those asks
     * are now dropped as provably invisible to the desktop (see
     * glaccel_content_dirty).  Reported by the PVDISPLAY_TIME_BREAKDOWN line. */
    uint64_t    n_inv_frame;      /* a delivered GL frame */
    uint64_t    n_inv_glop;       /* PVGPU_OP_GL submit */
    uint64_t    n_inv_2d;         /* CLEAR / FILL / COPY / BLIT into s->fb */
    uint64_t    n_inv_dropped;    /* … of the above, suppressed (content-only) */

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
                                  PVDeskRepaintFn repaint,
                                  void *opaque, int w, int h);

/* Phase E: return the unified engine's console for hardware-cursor setup.
 * Only valid after register_desktop (asserts otherwise). */
QemuConsole *sgi_glaccel_get_console(void);

#endif /* SGI_GLACCEL_H */
