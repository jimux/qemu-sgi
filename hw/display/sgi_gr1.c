/*
 * SGI GR1 ("Eclipse") graphics for the Personal IRIS (IP6)
 *
 * The board is a GE5 geometry engine (HQ1 microcode sequencer + Weitek
 * WTL3132 floating point data path + microcode/data RAM + host FIFO), an RE2
 * raster engine with its video RAM, five XMAP2 display multiplexers, two
 * Bt431 cursor generators and three Bt457 RAMDACs.
 *
 * Why the microcode is executed rather than its effects imitated: the host
 * talks to the board as a FIFO of indexed register writes, and the meaning of
 * each write is decided by the microcode the host has downloaded (a FIFO
 * word's aperture offset is the microcode dispatch address, its data is an
 * operand).  The same selector carries different meanings on different
 * occasions (measured by the personal-iris lane on the PROM's own graphics
 * test), so the pixels the guest sees are a function of that program and
 * cannot be produced without running it.
 *
 * This is a port of MAME, the reference implementation that runs this
 * machine (license BSD-3-Clause, copyright Patrick Mackinlay):
 *   src/mame/sgi/sgi_gr1.cpp     board map, display registers, FIFO
 *   src/mame/sgi/sgi_ge5.cpp     HQ1 sequencer
 *   src/devices/machine/wtl3132.cpp   FPU data path
 *   src/mame/sgi/sgi_re2.cpp     raster engine and screen update
 *   src/mame/sgi/sgi_xmap2.cpp   XMAP2
 *   src/devices/video/bt45x.cpp, bt431.cpp
 * Structure, names and behaviour follow those sources; deviations are marked
 * "DEVIATION" with the reason.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/sgi_gr1.h"
#include "fpu/softfloat.h"
#include "ui/console.h"
#include "trace.h"

#define GR1_SCREEN_W    1280
#define GR1_SCREEN_H    1024
#define GR1_VRAM_WORDS  (1280 * 1024)
/* MAME screen set_raw(): 1680 x 1065 total, active from (246, 39) */
#define GR1_HBEND       246
#define GR1_VBEND       39
#define GR1_VTOTAL      1065
#define GR1_VBSTART     (39 + 1024)

#define GE5_CODE_WORDS  0x4000  /* code_map 0x0000-0x3fff (64-bit words) */
#define GE5_DATA_WORDS  0x2000  /* data_map 0x0000-0x1fff (32-bit words) */
#define GR1_FIFO_DEPTH  512

/* GE runs synchronously with the host access that unblocks it; this bounds
 * one burst (a full-screen clear is a few million HQ1 cycles). */
#define GE5_BURST       (1u << 22)

/* ---- WTL3132 ---------------------------------------------------------- */

#define WTL_M_ENCN   0x000000003ULL
#define WTL_M_MBIN   0x000000004ULL
#define WTL_M_ADST   0x000000018ULL
#define WTL_M_ABIN   0x0000000e0ULL
#define WTL_M_DADD   0x000001f00ULL
#define WTL_M_IOCT   0x000006000ULL
#define WTL_M_CWEN   0x000008000ULL
#define WTL_M_CADD   0x0001f0000ULL
#define WTL_M_BADD   0x003e00000ULL
#define WTL_M_AADD   0x07c000000ULL
#define WTL_M_F      0x380000000ULL
#define WTL_M_CANCEL 0x8000000000000000ULL

#define WTL_S_ENCN   0
#define WTL_S_MBIN   2
#define WTL_S_ADST   3
#define WTL_S_ABIN   5
#define WTL_S_DADD   8
#define WTL_S_IOCT   13
#define WTL_S_CWEN   15
#define WTL_S_CADD   16
#define WTL_S_BADD   21
#define WTL_S_AADD   26
#define WTL_S_F      31

#define OPF(c, x) ((unsigned)(((c) & WTL_M_##x) >> WTL_S_##x))

enum { F_MISC = 0, F_FSUBR = 1, F_FSUB = 2, F_FADD = 3,
       F_FMNA = 5, F_FMNS = 6, F_FMAC = 7 };
enum { MF_FCLSR = 0, MF_FSTSR = 1, MF_FMODE = 3, MF_FABS = 4,
       MF_FLOAT = 5, MF_FIX = 6, MF_FLUT = 7 };

#define MODE_IBA 0x0001
#define MODE_RTN 0x0002
#define MODE_IBP 0x0008
#define MODE_OBP 0x0010
#define MODE_FXO 0x0020
#define MODE_FXH 0x0100
#define MODE_IBB 0x0800

typedef struct WTL3132 {
    float_status fs;

    bool fpcn_state, fpex_state, zero_state;

    uint64_t c_port;
    float32 x_port, x_in, x_out;

    int abort, neut, stall;

    uint64_t slot[4];
    unsigned head;

    uint16_t mode;
    float32 f[32];
    float32 t1, t2, t3;
    bool cr, sr, zr;

    float32 c_bus_data;
    unsigned c_bus_addr;
    bool c_bus_cwen;

    float32 ma_in, mb_in, m_out;
    float32 aa_in[2], ab_in[2], a_out;
} WTL3132;

/* ---- GE5 (HQ1) -------------------------------------------------------- */

enum { GE_STALL, GE_DECODE, GE_READ, GE_CONTROL, GE_WRITE, GE_COMPLETE };

typedef struct GE5Decode {
    unsigned source;
    bool inc_reptr;
    bool secondary;
    bool inc_memptr;
    unsigned destination;
    unsigned control;
    uint64_t fpu;
    uint8_t operation;
    uint16_t immediate;
} GE5Decode;

/* ---- RE2 -------------------------------------------------------------- */

enum {
    REG_ENABRGB = 0x04, REG_BIGENDIAN, REG_FUNC, REG_HADDR, REG_NOPUP,
    REG_XYFRAC, REG_RGB, REG_YX, REG_PUPDATA, REG_PATL, REG_PATH, REG_DZI,
    REG_DZF, REG_DR, REG_DG, REG_DB, REG_Z, REG_R, REG_G, REG_B, REG_STIP,
    REG_STIPCOUNT, REG_DX, REG_DY, REG_NUMPIX, REG_X, REG_Y, REG_IR,
    REG_RWDATA = 0x20, REG_PIXMASK, REG_AUXMASK, REG_WIDDATA, REG_UAUXDATA,
    REG_RWMODE, REG_READBUF, REG_PIXTYPE, REG_ASELECT, REG_ALIGNPAT,
    REG_ENABPAT, REG_ENABSTIP, REG_ENABDITH, REG_ENABWID, REG_CURWID,
    REG_DEPTHFN, REG_REPSTIP, REG_ENABLWID, REG_FBOPTION, REG_TOPSCAN,
    REG_TESTMODE, REG_TESTDATA, REG_ZBOPTION, REG_XZOOM, REG_UPACMODE,
    REG_YMIN, REG_YMAX, REG_XMIN, REG_XMAX, REG_COLORCMP, REG_MEGOPTION,
};

enum { RWMODE_FB = 0, RWMODE_PUP, RWMODE_UAUX, RWMODE_ZB, RWMODE_WID,
       RWMODE_FB_P = 6, RWMODE_ZB_P = 7 };

enum { IR_SHADED = 1, IR_FLAT = 2, IR_FLAT4 = 3, IR_TOPLINE = 4,
       IR_BOTLINE = 5, IR_READBUF = 6, IR_WRITEBUF = 7 };

enum { RE_IDLE, RE_EXECUTE, RE_DMA_R, RE_DMA_W };

static const uint32_t re2_regmask[64] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00000001, 0x0000000f, 0x00000003,
    0x00000001, 0x0000000f, 0x07ffffff, 0x003fffff, 0x00000003, 0x0000ffff, 0x0000ffff, 0x00ffffff,
    0x00003fff, 0x00ffffff, 0x000fffff, 0x000fffff, 0x00ffffff, 0x007fffff, 0x0007ffff, 0x0007ffff,
    0x0000ffff, 0x000000ff, 0x0000ffff, 0x0000ffff, 0x000007ff, 0x0000ffff, 0x000007ff, 0x00000007,

    0xffffffff, 0x00ffffff, 0x000001ff, 0x0000000f, 0x0000000f, 0x00000007, 0x00000001, 0x00000003,
    0x0000003f, 0x00000001, 0x00000001, 0x00000001, 0x00000001, 0x00000001, 0x0000000f, 0x0000000f,
    0x000000ff, 0x00000001, 0x00000003, 0x0003ffff, 0x00000001, 0x00003fff, 0x00000001, 0x000000ff,
    0x00000003, 0x000007ff, 0x000007ff, 0x00000fff, 0x00000fff, 0x00000001, 0x00000001, 0x00000000,
};

typedef struct GR1Rect {
    int min_x, max_x, min_y, max_y;
} GR1Rect;

static inline bool rect_contains(const GR1Rect *r, int x, int y)
{
    return x >= r->min_x && x <= r->max_x && y >= r->min_y && y <= r->max_y;
}

static inline void rect_set(GR1Rect *r, int x0, int x1, int y0, int y1)
{
    r->min_x = x0;
    r->max_x = x1;
    r->min_y = y0;
    r->max_y = y1;
}

/* ---- XMAP2 / Bt457 / Bt431 ------------------------------------------- */

#define XMAP_MODE_MC 0x3c00
#define XMAP_MODE_ME 0x0200
#define XMAP_MODE_UE 0x0100
#define XMAP_MODE_OE 0x00f0
#define XMAP_MODE_BS 0x0008
#define XMAP_MODE_DM 0x0007

typedef struct XMAP2 {
    uint16_t addr;
    uint32_t color[8192];   /* 0x00RRGGBB */
    uint32_t overlay[16];
    uint16_t mode[16];
    bool wid_aux;
    bool map_select;
    uint8_t options;
} XMAP2;

typedef struct Bt457 {
    uint8_t address;
    uint8_t address_rgb;
    uint8_t read_mask, blink_mask, command, control;
    uint8_t color_ram[256 + 4];
} Bt457;

typedef struct Bt431 {
    uint16_t address;
    uint8_t command;
    uint16_t cursor_x, cursor_y, window_x, window_y, window_w, window_h;
    uint8_t ram[512];
    GR1Rect bm_window, ch_v, ch_h;
} Bt431;

/* ---- board ------------------------------------------------------------ */

#define DR0_WM  0xf7
#define DR1_WM  0xe7
#define DR2_WM  0xe7
#define DR3_WM  0xe7
#define DR4_WM  0xe7
#define DR4_RM  0x9f
#define DR0_GRF1EN    0x01
#define DR0_SMALLMON0 0x08
#define DR1_TURBO     0x08
#define DR3_FIFOEMPTY 0x08
#define DR3_FIFOFULL  0x10
#define DR4_MEGOPT    0x08
#define DR4_GESTALL   0x10
#define DR4_MS        0x80

/* MAME "options" input port defaults: no turbo (0x08), Z buffer present. */
#define GR1_OPTIONS   0x08

struct SGIGR1State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    QemuConsole *con;
    qemu_irq irq[3];
    QEMUTimer *ge_timer;
    QEMUTimer *vbl_timer;
    bool vblank;

    /* properties */
    bool re_nowrite;    /* negative control: RE2 drops every pixel write */
    bool cwen_from_dr1;

    /* sgi_gr1 */
    unsigned bank;
    uint8_t dr0, dr1, dr2, dr3, dr4;
    uint64_t fifo[GR1_FIFO_DEPTH];
    unsigned fifo_head, fifo_count;
    bool reset;

    /* sgi_ge5 */
    uint64_t *code;
    uint32_t *data;
    int icount;
    unsigned state;
    bool int_state, cwen, fpu_c, re_rdy, re_drq;
    uint8_t mar;
    uint16_t pc;
    unsigned sp;
    uint16_t stack[8];
    uint8_t reptr;
    uint16_t memptr, memptr_temp;
    uint16_t dma_count;
    uint32_t finish[2];
    GE5Decode decode;
    uint64_t bus;
    uint32_t fpu_data;
    bool fpu_c_latch;
    WTL3132 fpu;

    /* sgi_re2 */
    unsigned re_state;
    bool ir_pending;
    uint32_t reg[64];
    bool enabrgb, bigendian;
    uint32_t func[4];
    bool nopup;
    unsigned pupdata;
    uint32_t pat;
    int32_t dz, dr, dg, db;
    int64_t z;
    uint32_t r, g, b;
    uint16_t stip;
    uint8_t stipcount;
    int32_t dx, dy;
    unsigned numpix;
    uint32_t x, y;
    unsigned ir;
    GR1Rect clip;
    uint32_t *vram;
    uint32_t vram_mask;

    XMAP2 xmap[5];
    Bt431 cursor[2];
    Bt457 ramdac[3];

    bool dirty;
    uint64_t stat_pixels;    /* pixels written to VRAM, for the trace */
};

static void ge5_run(SGIGR1State *s);

/* ==================== WTL3132 (wtl3132.cpp) ============================ */

static inline float32 i2f(int32_t v, float_status *fs)
{
    return int32_to_float32(v, fs);
}

static void wtl_reset(WTL3132 *w)
{
    w->abort = 1;
    w->neut = 1;
    w->stall = 1;
    w->fpcn_state = 1;
    w->fpex_state = 1;
    w->zero_state = 1;
    /* initialize pipeline with fnop */
    for (int i = 0; i < 4; i++) {
        w->slot[i] = ((uint64_t)F_FSUB << WTL_S_F) | WTL_M_CWEN | WTL_M_ABIN |
                     WTL_M_ADST;
    }
    w->head = 0;
    w->mode = 0;
}

static void wtl_stage1(SGIGR1State *s, unsigned index)
{
    WTL3132 *w = &s->fpu;
    uint64_t code = w->slot[index];
    float32 a_bus = w->f[OPF(code, AADD)];
    float32 b_bus = w->f[OPF(code, BADD)];

    /* C-to-A / C-to-B internal bypass */
    if ((w->mode & MODE_IBA) && !w->c_bus_cwen &&
        OPF(code, AADD) == w->c_bus_addr) {
        a_bus = w->c_bus_data;
    }
    if ((w->mode & MODE_IBB) && !w->c_bus_cwen &&
        OPF(code, BADD) == w->c_bus_addr) {
        b_bus = w->c_bus_data;
    }

    switch (OPF(code, IOCT)) {
    case 1: /* floadrc */
        w->f[OPF(code, DADD)] = w->x_port;
        w->x_in = w->x_port;
        break;
    case 2: /* fstore */
        if ((w->mode & MODE_OBP) && !w->c_bus_cwen &&
            OPF(code, DADD) == w->c_bus_addr) {
            w->x_out = w->c_bus_data;
        } else {
            w->x_out = w->f[OPF(code, DADD)];
        }
        break;
    case 3: /* fload */
        w->f[OPF(code, DADD)] = w->x_port;
        if ((w->mode & MODE_IBP) && OPF(code, AADD) == OPF(code, DADD)) {
            a_bus = w->x_port;
        }
        break;
    }

    if (OPF(code, F) == F_MISC) {
        switch (OPF(code, BADD)) {
        case MF_FCLSR:
            w->sr = false;
            break;
        case MF_FSTSR:
            w->x_port = make_float32(w->sr);
            break;
        case MF_FMODE: {
            bool polarity = !!(w->mode & MODE_FXH) ^
                            !!((OPF(code, CADD) << 5) & MODE_FXH);

            w->mode = (OPF(code, ABIN) << 10) | (OPF(code, CADD) << 5) |
                      OPF(code, AADD);
            if (!(w->mode & MODE_FXO)) {
                w->sr = false;
            }
            if (polarity) {
                w->fpex_state = !w->sr;
            }
            break;
        }
        }
    }

    w->ma_in = a_bus;
    w->mb_in = OPF(code, MBIN) ? w->x_in : b_bus;
    w->aa_in[0] = a_bus;
    w->ab_in[0] = b_bus;
}

static void wtl_stage2(SGIGR1State *s, unsigned index)
{
    WTL3132 *w = &s->fpu;
    uint64_t code = w->slot[index];

    w->aa_in[1] = (OPF(code, F) & 4) ? w->m_out : w->aa_in[0];

    switch (OPF(code, ABIN)) {
    case 0: w->ab_in[1] = w->x_in; break;
    case 1: w->ab_in[1] = w->ab_in[0]; break;
    case 2: w->ab_in[1] = w->t2; break;
    case 3: w->ab_in[1] = w->t1; break;
    case 4: w->ab_in[1] = w->t3; break;
    case 6: w->ab_in[1] = i2f(2, &w->fs); break;
    case 7: w->ab_in[1] = i2f(0, &w->fs); break;
    }

    switch (OPF(code, F)) {
    case F_FMNA:
    case F_FMNS:
        w->m_out = float32_mul(float32_mul(w->ma_in, w->mb_in, &w->fs),
                               i2f(-1, &w->fs), &w->fs);
        break;
    case F_FMAC:
        w->m_out = float32_mul(w->ma_in, w->mb_in, &w->fs);
        break;
    }

    /* complete fstore: the X port drives the GE's fpu data latch */
    if (OPF(code, IOCT) == 2) {
        s->fpu_data = float32_val(w->x_out);
    }
}

static void wtl_stage3(SGIGR1State *s, unsigned index)
{
    WTL3132 *w = &s->fpu;
    uint64_t code = w->slot[index];
    float32 zero = i2f(0, &w->fs);

    set_float_exception_flags(0, &w->fs);

    switch (OPF(code, F)) {
    case F_MISC:
        switch (OPF(code, BADD)) {
        case MF_FABS:
            if (float32_lt(w->aa_in[1], zero, &w->fs)) {
                w->a_out = float32_mul(w->aa_in[1], i2f(-1, &w->fs), &w->fs);
            } else {
                w->a_out = w->aa_in[1];
            }
            if (w->mode & MODE_RTN) {
                w->cr = float32_eq_quiet(w->a_out, zero, &w->fs);
            }
            break;
        case MF_FLOAT: {
            uint32_t v = float32_val(w->aa_in[1]);

            if ((w->mode & MODE_RTN) && OPF(code, ENCN) == 1) {
                w->cr = (v & 0xff000000) && (~v & 0xff000000);
            }
            w->a_out = i2f((int32_t)v, &w->fs);
            /* MAME's HACK, verbatim: GR1 DMA microcode needs cr on ENCN=2 */
            if (OPF(code, ENCN) == 2) {
                w->cr = float32_lt(w->a_out, zero, &w->fs);
            }
            break;
        }
        case MF_FIX: {
            FloatRoundMode save = get_float_rounding_mode(&w->fs);
            int32_t v;

            if ((w->mode & MODE_RTN) && OPF(code, ENCN) == 1) {
                w->cr = float32_lt(w->aa_in[1], i2f(-4194304, &w->fs), &w->fs) ||
                        float32_lt(i2f(4194304, &w->fs), w->aa_in[1], &w->fs);
            }
            set_float_rounding_mode((w->mode & MODE_RTN) ?
                                    float_round_nearest_even : float_round_down,
                                    &w->fs);
            v = float32_to_int32(w->aa_in[1], &w->fs);
            set_float_rounding_mode(save, &w->fs);
            w->a_out = make_float32((uint32_t)sextract32(v, 0, 24));
            break;
        }
        case MF_FLUT:
            /* MAME TODO: lookup table; reciprocal */
            w->a_out = float32_div(i2f(1, &w->fs), w->aa_in[1], &w->fs);
            break;
        }
        break;
    case F_FSUBR:
        w->a_out = float32_sub(w->ab_in[1], w->aa_in[1], &w->fs);
        break;
    case F_FSUB:
        w->a_out = float32_sub(w->aa_in[1], w->ab_in[1], &w->fs);
        break;
    case F_FADD:
        w->a_out = float32_add(w->aa_in[1], w->ab_in[1], &w->fs);
        break;
    case F_FMNA:
        w->a_out = float32_add(w->m_out, w->ab_in[1], &w->fs);
        break;
    case F_FMNS:
        w->a_out = float32_sub(w->m_out, w->ab_in[1], &w->fs);
        break;
    case F_FMAC:
        w->a_out = float32_add(w->m_out, w->ab_in[1], &w->fs);
        break;
    }

    if (OPF(code, F)) {
        switch (OPF(code, ENCN)) {
        case 1:
            w->cr = float32_le(w->a_out, zero, &w->fs);
            w->zr = float32_eq_quiet(w->a_out, zero, &w->fs);
            break;
        case 2:
            w->cr = float32_lt(w->a_out, zero, &w->fs);
            w->zr = float32_eq_quiet(w->a_out, zero, &w->fs);
            break;
        case 3:
            w->cr = float32_eq_quiet(w->a_out, zero, &w->fs);
            w->zr = float32_eq_quiet(w->a_out, zero, &w->fs);
            break;
        }
        if ((w->mode & MODE_FXH) || !w->sr) {
            w->sr = !!(get_float_exception_flags(&w->fs) & float_flag_overflow);
        }
    }
}

static void wtl_stage4(SGIGR1State *s, unsigned index)
{
    WTL3132 *w = &s->fpu;
    uint64_t code = w->slot[index];

    w->c_bus_data = w->a_out;
    w->c_bus_addr = OPF(code, CADD);
    w->c_bus_cwen = OPF(code, CWEN);

    switch (OPF(code, ADST)) {
    case 0: w->t3 = w->a_out; break;
    case 1: w->t2 = w->a_out; break;
    case 2: w->t1 = w->a_out; break;
    }

    /* output lines: only fpcn is wired (to the GE's branch condition) */
    if (w->cr ^ w->fpcn_state) {
        w->fpcn_state = w->cr;
        s->fpu_c = w->fpcn_state;
    }
    if (w->sr ^ w->fpex_state) {
        w->fpex_state = w->sr;
    }
    if (w->zr ^ w->zero_state) {
        w->zero_state = w->zr;
    }
}

static void wtl_clk(SGIGR1State *s)
{
    WTL3132 *w = &s->fpu;
    unsigned index0 = (w->head + 4) & 3;
    unsigned index1 = (w->head + 3) & 3;
    unsigned index2 = (w->head + 2) & 3;
    unsigned index3 = (w->head + 1) & 3;

    w->slot[index0] = w->c_port;

    if (!w->stall || !w->abort) {
        w->slot[index0] |= WTL_M_CANCEL;
    }

    /* retire pipeline stages in fifo order */
    if (!(w->slot[index3] & WTL_M_CANCEL)) {
        wtl_stage4(s, index3);
    }
    if (!(w->slot[index2] & WTL_M_CANCEL)) {
        wtl_stage3(s, index2);
    }
    if (!(w->slot[index1] & WTL_M_CANCEL)) {
        wtl_stage2(s, index1);
    }
    if (!(w->slot[index0] & WTL_M_CANCEL)) {
        wtl_stage1(s, index0);
    }

    /* register file write after stage 1: effective 4 cycle delay */
    if (!w->c_bus_cwen && !(w->slot[index3] & WTL_M_CANCEL)) {
        w->f[w->c_bus_addr] = w->c_bus_data;
    }

    /* NEUT- and ABORT- prevent writeback of the previous instruction */
    if (!w->neut || !w->abort) {
        w->slot[index1] |= WTL_M_CANCEL;
    }

    w->head = index3;
}

/* ==================== RE2 (sgi_re2.cpp) ================================ */

static inline void re2_set_rdy(SGIGR1State *s, bool state)
{
    s->re_rdy = state;
}

static inline void re2_set_drq(SGIGR1State *s, bool state)
{
    s->re_drq = state;
}

static inline void re2_vram_w(SGIGR1State *s, uint32_t offset, uint32_t data,
                              uint32_t mem_mask)
{
    /* DEVIATION: bounds check (MAME indexes past the array on a bad y) */
    if (offset >= GR1_VRAM_WORDS || s->re_nowrite) {
        return;
    }
    s->vram[offset] = (s->vram[offset] & ~mem_mask) |
                      (data & mem_mask & s->vram_mask);
    s->dirty = true;
    s->stat_pixels++;
}

static inline uint32_t re2_vram_r(SGIGR1State *s, uint32_t offset)
{
    return offset < GR1_VRAM_WORDS ? s->vram[offset] : 0;
}

static void re2_reset(SGIGR1State *s)
{
    s->vram_mask = 0xffffffffu;
    s->re_state = RE_IDLE;
    s->ir_pending = false;
    re2_set_rdy(s, true);
    re2_set_drq(s, false);
    memset(s->reg, 0, sizeof(s->reg));
    /* reset register values indicate presence of RE2 */
    s->reg[REG_DZF] = ~0u;
    rect_set(&s->clip, 0, 1279, 0, 1023);
}

static void re2_increment(SGIGR1State *s)
{
    s->x += s->dx;
    s->y += s->dy;
    s->z += s->dz;
    s->r += s->dr;
    s->g += s->dg;
    s->b += s->db;
}

static bool re2_wid(SGIGR1State *s, unsigned ir, uint32_t offset)
{
    unsigned wid;

    if (!s->reg[REG_ENABWID]) {
        return true;
    }
    if ((ir == IR_TOPLINE || ir == IR_BOTLINE) && !s->reg[REG_ENABLWID]) {
        return true;
    }
    wid = re2_vram_r(s, offset) >> 28;
    if (s->reg[REG_FBOPTION] & 1) {
        if (s->reg[REG_DEPTHFN] & 8) {
            return (wid & 0xe) == (s->reg[REG_CURWID] & 0xe);
        }
        return (wid & 0xf) == (s->reg[REG_CURWID] & 0xf);
    }
    return (wid & 0x3) == (s->reg[REG_CURWID] & 0x3);
}

static bool re2_pattern(SGIGR1State *s, unsigned x, unsigned n)
{
    unsigned index;

    if (!s->reg[REG_ENABPAT]) {
        return true;
    }
    index = (s->reg[REG_ALIGNPAT] ? x : n) % 32;
    return (s->pat >> (31 - index)) & 1;
}

static uint32_t re2_unpack(uint32_t data, unsigned n, uint32_t mode)
{
    switch (mode) {
    case 1: data = (uint16_t)(data >> (16 * (1 - n))); break;
    case 3: data = (uint8_t)(data >> (8 * (3 - n))); break;
    }
    return data;
}

static inline uint32_t re2_aux(SGIGR1State *s)
{
    return s->nopup ?
        (s->reg[REG_WIDDATA] << 28) | (s->reg[REG_UAUXDATA] << 24) :
        (s->reg[REG_WIDDATA] << 28) | ((s->reg[REG_UAUXDATA] & 0x3) << 26) |
        (s->pupdata << 24);
}

static void re2_draw_shaded_span(SGIGR1State *s)
{
    uint32_t mask = (s->reg[REG_AUXMASK] << 24) | s->reg[REG_PIXMASK];
    uint32_t aux = re2_aux(s);

    for (unsigned n = 0; s->numpix--; n++) {
        if (rect_contains(&s->clip, s->x >> 14, s->y >> 14)) {
            if (re2_pattern(s, s->x >> 14, n)) {
                uint32_t offset = (s->y >> 14) * 0x500 + (s->x >> 14);

                if (re2_wid(s, IR_SHADED, offset)) {
                    uint32_t color = (s->r >> 11) << 0 | (s->g >> 11) << 8 |
                                     (s->b >> 11) << 16;

                    re2_vram_w(s, offset, aux | color, mask);
                }
            }
        }
        re2_increment(s);
    }
    s->re_state = RE_IDLE;
}

static void re2_draw_flat_span(SGIGR1State *s, unsigned n)
{
    uint32_t mask = (s->reg[REG_AUXMASK] << 24) | s->reg[REG_PIXMASK];
    uint32_t aux = re2_aux(s);
    uint32_t offset = (s->y >> 14) * 0x500 + (s->x >> 14);

    for (unsigned i = 0; i < s->numpix; i++) {
        if (rect_contains(&s->clip, (s->x >> 14) + i, s->y >> 14)) {
            uint32_t color = (s->r >> 11) << 0 | (s->g >> 11) << 8 |
                             (s->b >> 11) << 16;

            re2_vram_w(s, offset + i, aux | color, mask);
        }
        if ((i % n) == 0) {
            re2_increment(s);
        }
    }
    s->re_state = RE_IDLE;
}

static void re2_read_buffer(SGIGR1State *s)
{
    if (s->numpix > 0) {
        switch (s->reg[REG_RWMODE]) {
        case RWMODE_FB_P:
            s->reg[REG_RWDATA] =
                re2_vram_r(s, (s->y >> 14) * 0x500 + (s->x >> 14));
            break;
        }
        trace_sgi_gr1_re_read(s->x >> 14, s->y >> 14, s->reg[REG_RWMODE],
                              s->reg[REG_RWDATA]);
        re2_increment(s);
        s->numpix--;
        s->re_state = RE_DMA_R;
    } else {
        s->re_state = RE_IDLE;
    }
    re2_set_drq(s, s->re_state == RE_DMA_R);
}

static void re2_write_buffer(SGIGR1State *s)
{
    if (s->re_state == RE_DMA_W) {
        for (unsigned i = 0; i <= s->reg[REG_UPACMODE]; i++) {
            if (rect_contains(&s->clip, s->x >> 14, s->y >> 14)) {
                uint32_t offset = (s->y >> 14) * 0x500 + (s->x >> 14);

                if (re2_wid(s, IR_WRITEBUF, offset)) {
                    uint32_t data = re2_unpack(s->reg[REG_RWDATA], i,
                                               s->reg[REG_UPACMODE]);

                    switch (s->reg[REG_RWMODE]) {
                    case RWMODE_UAUX:
                        re2_vram_w(s, offset, data << 24,
                                   (s->reg[REG_AUXMASK] &
                                    (s->nopup ? 0xf : 0xc)) << 24);
                        break;
                    }
                }
            }
            re2_increment(s);
            s->numpix--;
        }
    }
    s->re_state = s->numpix > 0 ? RE_DMA_W : RE_IDLE;
    re2_set_drq(s, s->re_state == RE_DMA_W);
}

static void re2_execute(SGIGR1State *s)
{
    uint32_t *reg = s->reg;

    s->enabrgb = !!reg[REG_ENABRGB];
    s->bigendian = !!reg[REG_BIGENDIAN];
    for (int i = 0; i < 4; i++) {
        s->func[i] = (reg[REG_FUNC] >> i) & 1 ? ~0u : 0;
    }
    s->nopup = !!reg[REG_NOPUP];
    s->pupdata = reg[REG_PUPDATA];
    s->pat = (reg[REG_PATH] << 16) | reg[REG_PATL];
    s->dz = (int32_t)(((int64_t)((uint64_t)reg[REG_DZI] << 40) >> 26) |
                      reg[REG_DZF]);
    s->dr = sextract32(reg[REG_DR], 0, 24);
    s->dg = sextract32(reg[REG_DG], 0, 20);
    s->db = sextract32(reg[REG_DB], 0, 20);
    s->z = (int64_t)((uint64_t)reg[REG_Z] << 40) >> 26;
    s->r = reg[REG_R];
    s->g = reg[REG_G];
    s->b = reg[REG_B];
    s->stip = reg[REG_STIP];
    s->stipcount = reg[REG_STIPCOUNT];
    s->dx = (int32_t)(int16_t)reg[REG_DX];
    s->dy = (int32_t)(int16_t)reg[REG_DY];
    s->numpix = reg[REG_NUMPIX];
    s->x = ((reg[REG_X] >> 3) * 5 + (reg[REG_X] & 0x7)) << 14;
    s->y = reg[REG_Y] << 14;
    s->ir = reg[REG_IR];

    trace_sgi_gr1_re_ir(s->ir, reg[REG_X], reg[REG_Y], s->numpix,
                        reg[REG_RWMODE], s->r, s->g, s->b);

    switch (s->ir) {
    case IR_SHADED:
        re2_draw_shaded_span(s);
        break;
    case IR_FLAT:
        re2_draw_flat_span(s, 5);
        break;
    case IR_FLAT4:
        re2_draw_flat_span(s, 20);
        break;
    case IR_TOPLINE:
    case IR_BOTLINE:
        s->re_state = RE_IDLE;
        break;
    case IR_READBUF:
        re2_read_buffer(s);
        break;
    case IR_WRITEBUF:
        re2_write_buffer(s);
        break;
    default:
        /* DEVIATION: MAME leaves the state at EXECUTE for ir 0 */
        s->re_state = RE_IDLE;
        break;
    }
}

/*
 * MAME schedules EXECUTE on a zero-delay timer; here it runs in line, which
 * leaves rdy asserted by the time the GE's next write arrives (the GE's
 * rdy/drq stalls then only ever see a completed RE).
 */
static void re2_step(SGIGR1State *s)
{
    switch (s->re_state) {
    case RE_IDLE:
        re2_set_rdy(s, !s->ir_pending);
        if (s->ir_pending) {
            s->ir_pending = false;
            s->re_state = RE_EXECUTE;
            re2_step(s);
        }
        break;
    case RE_EXECUTE:
        re2_execute(s);
        if (s->re_state == RE_IDLE) {
            re2_step(s);
        }
        break;
    default:
        break;
    }
}

static uint32_t re2_reg_r(SGIGR1State *s, unsigned offset)
{
    uint32_t data = 0xffffffffu & re2_regmask[offset];

    switch (offset) {
    case REG_RWDATA:
        if (s->re_state == RE_DMA_R) {
            data = s->reg[REG_RWDATA];
            re2_read_buffer(s);
            re2_step(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "sgi-gr1: re2 rwdata read when empty\n");
        }
        break;
    case REG_DZI:
    case REG_DZF:
    case REG_STIP:
    case REG_STIPCOUNT:
        data = s->reg[offset];
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sgi-gr1: re2 read of register 0x%02x\n",
                      offset);
        break;
    }
    return data;
}

static void re2_reg_w(SGIGR1State *s, unsigned offset, uint32_t data)
{
    if (!re2_regmask[offset]) {
        qemu_log_mask(LOG_UNIMP, "sgi-gr1: re2 write of register 0x%02x\n",
                      offset);
        return;
    }
    s->reg[offset] = data & re2_regmask[offset];

    switch (offset) {
    case REG_RGB:
        s->reg[REG_B] = (data & 0x000000ff) << 11;
        s->reg[REG_G] = (data & 0x0000ff00) << 3;
        s->reg[REG_R] = (data & 0x0fff0000) >> 5;
        break;
    case REG_YX:
        s->reg[REG_X] = (data & 0x00000fff) >> 0;
        s->reg[REG_Y] = (data & 0x007ff000) >> 12;
        break;
    case REG_IR:
        s->ir_pending = true;
        if (s->re_state == RE_IDLE) {
            re2_step(s);
        }
        break;
    case REG_RWDATA:
        if (s->re_state == RE_DMA_W) {
            re2_write_buffer(s);
            re2_step(s);
        }
        break;
    case REG_YMIN:
    case REG_YMAX:
    case REG_XMIN:
    case REG_XMAX:
        rect_set(&s->clip,
                 (s->reg[REG_XMIN] >> 3) * 5 + (s->reg[REG_XMIN] & 0x7),
                 (s->reg[REG_XMAX] >> 3) * 5 + (s->reg[REG_XMAX] & 0x7),
                 s->reg[REG_YMIN], s->reg[REG_YMAX]);
        break;
    }
}

/* ==================== GE5 (sgi_ge5.cpp) ================================ */

static void ge5_set_int(SGIGR1State *s, bool state)
{
    if (state != s->int_state) {
        s->int_state = state;
        trace_sgi_gr1_ge_int(state, s->pc);
        qemu_set_irq(s->irq[SGI_GR1_IRQ_GE], state);
    }
}

static void ge5_reset(SGIGR1State *s)
{
    s->pc = 0;
    s->sp = 0;
    s->reptr = 0;
    s->memptr = 0;
    s->memptr_temp = 0;
    ge5_set_int(s, false);
    s->state = GE_STALL;
    wtl_reset(&s->fpu);
}

static inline uint64_t ge5_code(SGIGR1State *s, uint16_t pc)
{
    pc &= 0x7fff;
    return pc < GE5_CODE_WORDS ? s->code[pc] : 0;
}

static inline uint32_t ge5_data_r(SGIGR1State *s, uint16_t a)
{
    return s->data[a & (GE5_DATA_WORDS - 1)];
}

static inline void ge5_data_w(SGIGR1State *s, uint16_t a, uint32_t v,
                              uint32_t mask)
{
    uint32_t *p = &s->data[a & (GE5_DATA_WORDS - 1)];

    *p = (*p & ~mask) | (v & mask);
}

static void ge5_decode(SGIGR1State *s)
{
    uint64_t primary = ge5_code(s, s->pc);
    GE5Decode *d = &s->decode;

    d->source = (primary >> 38) & 3;
    d->inc_reptr = (primary >> 37) & 1;
    d->secondary = (primary >> 36) & 1;
    d->inc_memptr = (primary >> 35) & 1;
    d->destination = (primary >> 33) & 3;
    d->control = (primary >> 29) & 0xf;

    d->fpu = ((primary & 0x1ffff800ULL) << 5) | ((primary & 0x7ffULL) << 2) |
             (2ULL << WTL_S_ENCN);
    if (s->cwen) {
        d->fpu |= WTL_M_CWEN;
    }

    if (d->secondary) {
        uint64_t secondary = ge5_code(s, s->pc + 1);

        d->operation = (secondary >> 32) & 0xfe;
        d->immediate = (secondary >> 19) & 0x3fff;
    }
}

static void ge5_secondary(SGIGR1State *s)
{
    GE5Decode *d = &s->decode;

    switch (d->operation) {
    case 0x3c: /* store register */
        switch (d->immediate) {
        case 0: /* TODO: store pcsave? */
            break;
        case 1:
            s->bus = s->memptr;
            break;
        case 2:
            s->bus = s->reptr;
            break;
        }
        break;
    case 0x8c: /* load reptr */
        s->reptr = s->bus & 0x3f;
        break;
    case 0x90: /* load memptr; set reptr */
        s->memptr = s->bus & 0x7fff;
        s->reptr = d->immediate & 0x3f;
        break;
    case 0x9c: /* set reptr */
        s->reptr = d->immediate & 0x3f;
        break;
    case 0xb0: /* load memptr */
        s->memptr = s->bus & 0x7fff;
        break;
    case 0xb4: /* set memptr */
        s->memptr = d->immediate & 0x7fff;
        break;
    case 0xb6: /* set memptr; set finish flag */
        s->memptr = d->immediate & 0x7fff;
        s->finish[d->immediate & 1] = 1;
        break;
    case 0xb8: /* set memptr_temp */
        s->memptr_temp = s->memptr | 0x8000;
        s->memptr = d->immediate & 0x7fff;
        break;
    case 0xbc: /* nop? */
        break;
    case 0xfc:
        if (d->immediate) {
            ge5_set_int(s, true);   /* assert interrupt */
        }   /* else TODO: assert dma ready */
        break;
    case 0xfe:
        if (d->immediate) {
            s->dma_count = s->bus;  /* load dma count */
        }   /* else TODO: reset dma? */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sgi-gr1: ge5 unknown secondary op 0x%02x\n",
                      d->operation);
        break;
    }
}

static uint64_t gr1_fifo_r(SGIGR1State *s);

/* MAME execute_run(): runs until icount is spent or the engine blocks */
static void ge5_execute(SGIGR1State *s)
{
    GE5Decode *d = &s->decode;

    while (s->icount > 0) {
        switch (s->state) {
        case GE_STALL:
            s->icount = 0;
            break;

        case GE_DECODE:
            ge5_decode(s);
            if (d->secondary) {
                ge5_secondary(s);
            }
            if (d->inc_memptr) {
                s->memptr = (s->memptr + 1) & 0x7fff;
            }
            if (d->inc_reptr) {
                s->reptr = (s->reptr + 1) & 0x3f;
            }
            s->pc += d->secondary ? 2 : 1;
            s->state = GE_READ;
            break;

        case GE_READ:
            s->state = GE_CONTROL;
            switch (d->source) {
            case 0: /* reptr */
                if (s->reptr == 0x20 && !s->re_drq) {
                    s->state = GE_READ;     /* re read stall */
                    s->icount = 0;
                } else {
                    s->bus = re2_reg_r(s, s->reptr);
                }
                break;
            case 1: /* fifo */
                if (s->fifo_count == 0) {
                    s->state = GE_READ;     /* fifo read stall */
                    s->icount = 0;
                } else {
                    s->bus = gr1_fifo_r(s);
                }
                break;
            case 2: /* memptr */
                s->bus = ge5_data_r(s, s->memptr);
                break;
            case 3: /* fpu */
                d->fpu |= (2ULL << WTL_S_IOCT);
                s->bus = s->fpu_data;
                break;
            }
            break;

        case GE_CONTROL:
            s->state = GE_WRITE;
            switch (d->control) {
            case 0x0:
                break;
            case 0x1:
                s->pc = d->immediate;
                break;
            case 0x2:
                if (s->fpu_c_latch) {
                    s->pc = d->immediate;
                }
                break;
            case 0x3:
                if (!s->fpu_c_latch) {
                    s->pc = d->immediate;
                }
                break;
            case 0x4:
                s->stack[s->sp] = s->pc;
                s->sp = (s->sp + 1) & 7;
                s->pc = d->immediate;
                break;
            case 0x5:
                if (s->fpu_c_latch) {
                    s->stack[s->sp] = s->pc;
                    s->sp = (s->sp + 1) & 7;
                    s->pc = d->immediate;
                }
                break;
            case 0x6:
                if (!s->fpu_c_latch) {
                    s->stack[s->sp] = s->pc;
                    s->sp = (s->sp + 1) & 7;
                    s->pc = d->immediate;
                }
                break;
            case 0x7: /* return */
                s->sp = (s->sp + 7) & 7;
                s->pc = s->stack[s->sp];
                break;
            case 0x8: /* fetch */
                s->pc = (s->bus >> 31) & 0x1fe;
                trace_sgi_gr1_ge_fetch((uint32_t)(s->bus >> 32),
                                       (uint32_t)s->bus);
                /* neutralize previous instruction writeback */
                s->fpu.neut = 0;
                break;
            case 0x9: /* branch indirect (MAME: TODO verify value) */
                s->pc = s->bus;
                break;
            case 0xa:
                if ((s->bus >> 31) & 1) {
                    s->pc = d->immediate;
                }
                break;
            case 0xb:
                if (!((s->bus >> 31) & 1)) {
                    s->pc = d->immediate;
                }
                break;
            case 0xc: /* stall */
                s->state = GE_STALL;
                s->icount = 0;
                break;
            case 0xd:
                if ((s->bus >> 31) & 1) {
                    s->stack[s->sp] = s->pc;
                    s->sp = (s->sp + 1) & 7;
                    s->pc = d->immediate;
                }
                break;
            case 0xe:
                if (!((s->bus >> 31) & 1)) {
                    s->stack[s->sp] = s->pc;
                    s->sp = (s->sp + 1) & 7;
                    s->pc = d->immediate;
                }
                break;
            case 0xf: /* dma cycle */
                if (--s->dma_count) {
                    s->pc -= d->secondary ? 2 : 1;
                }
                break;
            }
            break;

        case GE_WRITE:
            s->state = GE_COMPLETE;
            switch (d->destination) {
            case 0: /* reptr */
                if (s->reptr == 0x20 && !s->re_rdy && !s->re_drq) {
                    s->state = GE_WRITE;    /* re write stall */
                    s->icount = 0;
                } else if (s->reptr > 0x20 && !s->re_rdy) {
                    s->state = GE_WRITE;    /* unbuffered register stall */
                    s->icount = 0;
                } else {
                    re2_reg_w(s, s->reptr, (uint32_t)s->bus);
                }
                break;
            case 1: /* TODO: bus? */
                break;
            case 2: /* memptr */
                ge5_data_w(s, s->memptr, (uint32_t)s->bus, 0xffffffffu);
                break;
            case 3: /* fpu */
                s->fpu.x_port = make_float32((uint32_t)s->bus);
                d->fpu |= (3ULL << WTL_S_IOCT);
                break;
            }
            break;

        case GE_COMPLETE:
            s->state = GE_DECODE;
            if (s->memptr_temp & 0x8000) {
                s->memptr = s->memptr_temp & 0x7fff;
                s->memptr_temp = 0;
            }
            /* FIXME (MAME): fpu condition has additional 1 cycle latency */
            s->fpu_c_latch = s->fpu_c;
            s->fpu.c_port = d->fpu;
            wtl_clk(s);
            s->fpu.neut = 1;
            s->icount--;
            break;
        }
    }
}

static bool ge5_runnable(SGIGR1State *s)
{
    switch (s->state) {
    case GE_STALL:
        return false;
    case GE_READ:
        if (s->decode.source == 1 && s->fifo_count == 0) {
            return false;
        }
        if (s->decode.source == 0 && s->reptr == 0x20 && !s->re_drq) {
            return false;
        }
        return true;
    case GE_WRITE:
        if (s->decode.destination == 0 &&
            ((s->reptr == 0x20 && !s->re_rdy && !s->re_drq) ||
             (s->reptr > 0x20 && !s->re_rdy))) {
            return false;
        }
        return true;
    default:
        return true;
    }
}

/*
 * Run the engine until it blocks (stall, empty FIFO, RE handshake) or the
 * burst budget runs out; a spent budget resumes from a timer so a long
 * draw cannot hold the vCPU indefinitely.
 */
static void ge5_run(SGIGR1State *s)
{
    int budget = GE5_BURST;

    while (budget > 0 && ge5_runnable(s)) {
        s->icount = MIN(budget, 4096);
        budget -= s->icount;
        ge5_execute(s);
        budget += s->icount;    /* unspent cycles of a blocked pass */
        if (s->icount > 0) {
            break;
        }
    }
    if (ge5_runnable(s)) {
        timer_mod(s->ge_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * SCALE_US);
    }
}

static void ge5_timer_cb(void *opaque)
{
    ge5_run(opaque);
}

static void ge5_command_w(SGIGR1State *s, unsigned offset)
{
    switch (offset) {
    case 0x00: /* clear stall */
        s->state = GE_DECODE;
        break;
    case 0x10: /* set single step */
    case 0x20: /* clear single step */
    case 0x30: /* execute single step */
        break;
    case 0x50: /* clear interrupt */
        ge5_set_int(s, false);
        break;
    }
}

/* ==================== XMAP2 (sgi_xmap2.cpp) ============================ */

static inline uint8_t rgb_r(uint32_t c) { return c >> 16; }
static inline uint8_t rgb_g(uint32_t c) { return c >> 8; }
static inline uint8_t rgb_b(uint32_t c) { return c; }

static inline void rgb_set(uint32_t *c, unsigned shift, uint8_t v)
{
    *c = (*c & ~(0xffu << shift)) | ((uint32_t)v << shift);
}

static uint8_t xmap_reg_r(XMAP2 *x, unsigned offset)
{
    unsigned idx = x->map_select ? x->addr : (x->addr & 0xfff);

    switch (offset) {
    case 1: case 2: case 3: {
        uint32_t c;

        if (x->addr & 0x1000) {
            c = x->color[idx];
        } else if (x->addr < 0x10) {
            c = x->overlay[x->addr];
        } else {
            break;
        }
        return offset == 1 ? rgb_b(c) : offset == 2 ? rgb_g(c) : rgb_r(c);
    }
    case 4: /* increment address (MAME: read does not increment) */
        break;
    case 5:
        if (x->addr < 0x20) {
            uint16_t mode = x->mode[(x->addr >> 1) & 0xf];

            return (x->addr & 1) ? (mode >> 8) : (uint8_t)mode;
        } else if (x->addr == 0x20) {
            return x->wid_aux;
        } else if (x->addr == 0x21) {
            return x->options;
        }
        break;
    case 6:
        return x->addr >> 8;
    case 7:
        return (uint8_t)x->addr;
    }
    return 0;
}

static void xmap_reg_w(XMAP2 *x, unsigned offset, uint8_t data)
{
    unsigned idx = x->map_select ? x->addr : (x->addr & 0xfff);
    unsigned shift = offset == 1 ? 0 : offset == 2 ? 8 : 16;

    switch (offset) {
    case 1: case 2: case 3:
        if (x->addr & 0x1000) {
            rgb_set(&x->color[idx], shift, data);
        } else if (x->addr < 0x10) {
            rgb_set(&x->overlay[x->addr], shift, data);
        }
        break;
    case 4:
        x->addr = (x->addr + 1) & 0x1fff;
        break;
    case 5:
        if (x->addr < 0x20) {
            uint16_t *mode = &x->mode[(x->addr >> 1) & 0xf];

            if (x->addr & 1) {
                *mode = ((uint16_t)(data & 0x3f) << 8) | (*mode & 0x00ff);
            } else {
                *mode = (*mode & 0x3f00) | data;
            }
        } else if (x->addr == 0x20) {
            x->wid_aux = data & 1;
        }
        break;
    case 6:
        x->addr = (uint16_t)((data & 0x1f) << 8) | (x->addr & 0x00ff);
        break;
    case 7:
        x->addr = (x->addr & 0x1f00) | data;
        break;
    }
}

/* the colour a display pixel of index 'index' shows (pen_color) */
static inline uint32_t xmap_pen(XMAP2 *x, unsigned index)
{
    /* MAME's palette holds 4096 colours, the half picked by map_select */
    return x->color[(x->map_select ? 4096 : 0) + (index & 0xfff)];
}

static inline uint32_t xmap_overlay_r(XMAP2 *x, uint8_t data)
{
    return x->wid_aux ? x->overlay[data & 0x3] : x->overlay[data & 0xf];
}

/* ==================== Bt457 / Bt431 ==================================== */

static void bt457_increment(Bt457 *b)
{
    if (b->control & 0x07) {
        b->address_rgb = (b->address_rgb + 1) % 3;
        if (b->address_rgb == 0) {
            b->address++;
        }
    } else {
        b->address++;
    }
}

static inline bool bt457_selected(Bt457 *b)
{
    return !(b->control & 0x07) ||
           (b->control & 0x07) == (1 << b->address_rgb);
}

static uint8_t bt457_r(Bt457 *b, unsigned reg)
{
    uint8_t data = 0;   /* space.unmap() */

    switch (reg) {
    case 0:
        b->address_rgb = 0;
        return b->address;
    case 1:
        if (bt457_selected(b)) {
            data = b->color_ram[b->address];
        }
        bt457_increment(b);
        return data;
    case 2:
        switch (b->address) {
        case 4: return b->read_mask;
        case 5: return b->blink_mask;
        case 6: return b->command;
        case 7: return b->control;
        }
        return 0;
    case 3:
        if (b->address < 4 && bt457_selected(b)) {
            data = b->color_ram[256 + b->address];
        }
        bt457_increment(b);
        return data;
    }
    return 0;
}

static void bt457_w(Bt457 *b, unsigned reg, uint8_t data)
{
    switch (reg) {
    case 0:
        b->address_rgb = 0;
        b->address = data;
        break;
    case 1:
        if (bt457_selected(b)) {
            b->color_ram[b->address] = data;
        }
        bt457_increment(b);
        break;
    case 2:
        switch (b->address) {
        case 4: b->read_mask = data; break;
        case 5: b->blink_mask = data; break;
        case 6: b->command = data; break;
        case 7: b->control = data & 0xf; break;
        }
        break;
    case 3:
        if (b->address < 4 && bt457_selected(b)) {
            b->color_ram[256 + b->address] = data;
        }
        bt457_increment(b);
        break;
    }
}

static inline uint8_t bt457_lookup(Bt457 *b, uint8_t pixel, uint8_t overlay)
{
    if (overlay & 3) {
        return b->color_ram[256 + (overlay & (b->command & 3))];
    }
    return (b->command & 0x40) ? b->color_ram[pixel & b->read_mask] :
                                 b->color_ram[256];
}

static void bt431_update(Bt431 *c)
{
    unsigned mux = c->command & 0x0c;
    int p = mux == 0x00 ? 37 : mux == 0x04 ? 52 : mux == 0x08 ? 57 : 0;
    int cursor_x = c->cursor_x + p;
    int cursor_y = (c->cursor_y < 0xfc0 ? c->cursor_y : c->cursor_y - 0x1000) + 32;
    int thickness = c->command & 0x03;

    rect_set(&c->bm_window, cursor_x - 31, cursor_x + 32, cursor_y - 31,
             cursor_y + 32);
    if (c->window_x == 0 && c->window_y == 0 && c->window_w == 0x0fff &&
        c->window_h == 0x0fff) {
        rect_set(&c->ch_v, cursor_x - thickness, cursor_x + thickness,
                 c->window_y, c->window_y + c->window_h - 1);
        rect_set(&c->ch_h, c->window_x, c->window_x + c->window_w - 1,
                 cursor_y - thickness, cursor_y + thickness);
    } else {
        int wx = c->window_x + (mux == 0x00 ? 5 : mux == 0x04 ? 20 :
                                mux == 0x08 ? 25 : 0);
        int ww = c->window_w + (mux == 0x00 ? 2 : mux == 0x04 ? 8 :
                                mux == 0x08 ? 10 : 0);
        int wh = c->window_h + 2;

        rect_set(&c->ch_v, cursor_x - thickness, cursor_x + thickness,
                 c->window_y + 1, c->window_y + wh - 2);
        rect_set(&c->ch_h, wx + 1, wx + ww - 2, cursor_y - thickness,
                 cursor_y + thickness);
    }
}

static uint16_t *bt431_reg(Bt431 *c, unsigned n, bool *hi)
{
    static const unsigned char map[13] = { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6 };
    uint16_t *regs[7] = { NULL, &c->cursor_x, &c->cursor_y, &c->window_x,
                          &c->window_y, &c->window_w, &c->window_h };

    if (n == 0 || n > 12) {
        return NULL;
    }
    *hi = !(n & 1);
    return regs[map[n]];
}

static uint8_t bt431_r(Bt431 *c, unsigned reg)
{
    uint8_t data = 0;

    switch (reg) {
    case 0: return c->address & 0xff;
    case 1: return c->address >> 8;
    case 2:
        data = c->ram[c->address & 0x1ff];
        c->address = (c->address + 1) & 0x1ff;
        return data;
    case 3: {
        unsigned n = c->address & 0xf;
        bool hi;
        uint16_t *r = bt431_reg(c, n, &hi);

        if (n == 0) {
            data = c->command;
        } else if (r) {
            data = hi ? (*r >> 8) : (*r & 0xff);
        }
        c->address = (c->address + 1) & 0x1ff;
        return data;
    }
    }
    return 0;
}

static void bt431_w(Bt431 *c, unsigned reg, uint8_t data)
{
    switch (reg) {
    case 0:
        c->address = ((c->address & 0xff00) | data) & 0x1ff;
        break;
    case 1:
        c->address = ((c->address & 0x00ff) | ((uint16_t)data << 8)) & 0x1ff;
        break;
    case 2:
        c->ram[c->address & 0x1ff] = data;
        c->address = (c->address + 1) & 0x1ff;
        break;
    case 3: {
        unsigned n = c->address & 0xf;
        bool hi;
        uint16_t *r = bt431_reg(c, n, &hi);

        if (n == 0) {
            c->command = data & 0x7f;
        } else if (r) {
            *r = hi ? (uint16_t)(((data & 0xf) << 8) | (*r & 0xff)) :
                      (uint16_t)((*r & 0x0f00) | data);
        }
        c->address = (c->address + 1) & 0x1ff;
        bt431_update(c);
        break;
    }
    }
}

static bool bt431_cur(Bt431 *c, int x, int y)
{
    bool data = false;

    if ((c->command & 0x20) &&
        (rect_contains(&c->ch_h, x, y) || rect_contains(&c->ch_v, x, y))) {
        data = true;
    }
    if ((c->command & 0x40) && rect_contains(&c->bm_window, x, y)) {
        int bx = x - c->bm_window.min_x;
        int by = y - c->bm_window.min_y;
        bool bit = (c->ram[by * 8 + bx / 8] >> (7 - bx % 8)) & 1;

        if (c->command & 0x10) {
            data |= bit;
        } else {
            data ^= bit;
        }
    }
    return data;
}

/* ==================== board (sgi_gr1.cpp) ============================== */

static uint64_t gr1_fifo_r(SGIGR1State *s)
{
    uint64_t data = s->fifo[s->fifo_head];

    s->fifo_head = (s->fifo_head + 1) % GR1_FIFO_DEPTH;
    s->fifo_count--;

    /*
     * DEVIATION: MAME writes "m_dr3 &= DR3_FIFOEMPTY" here, which keeps only
     * the empty bit instead of clearing it; the bit is active low (set =
     * not empty), so the intent is to clear it.
     */
    if (s->fifo_count == 0) {
        s->dr3 &= ~DR3_FIFOEMPTY;
    }
    if (!(s->dr3 & DR3_FIFOFULL) && s->fifo_count <= 256) {
        s->dr3 |= DR3_FIFOFULL;
        qemu_set_irq(s->irq[SGI_GR1_IRQ_FIFO], 0);
    }
    return data;
}

static void gr1_fifo_w(SGIGR1State *s, unsigned offset, uint32_t data)
{
    trace_sgi_gr1_fifo_w(offset, data, s->fifo_count);

    if (s->fifo_count == GR1_FIFO_DEPTH) {
        qemu_log_mask(LOG_GUEST_ERROR, "sgi-gr1: fifo overflow\n");
        return;
    }
    s->fifo[(s->fifo_head + s->fifo_count) % GR1_FIFO_DEPTH] =
        ((uint64_t)offset << 32) | data;
    s->fifo_count++;

    s->dr3 |= DR3_FIFOEMPTY;
    if ((s->dr3 & DR3_FIFOFULL) && s->fifo_count > 256) {
        s->dr3 &= ~DR3_FIFOFULL;
        qemu_set_irq(s->irq[SGI_GR1_IRQ_FIFO], 1);
    }
    ge5_run(s);
}

/*
 * Byte-lane helpers.  The bus is big-endian 32-bit; MAME's umask32 names the
 * lane a narrow device sits on.  An access is widened to its 32-bit word
 * with a byte mask so every device sees MAME's (offset, data, mem_mask).
 */
static inline uint32_t lane_mask(hwaddr addr, unsigned size)
{
    unsigned shift = (4 - size - (addr & 3)) * 8;

    return (size == 4 ? 0xffffffffu : ((1u << (size * 8)) - 1)) << shift;
}

/*
 * DEVIATION (display registers): MAME puts dr0..dr4 on umask32(0xff000000),
 * but this PROM writes them as "andi 0xff; sw" (e.g. bfc036c4/bfc036cc for
 * dr0, bfc03b94..bfc03b9c for dr4), i.e. on the low byte, the same lane as
 * the XMAP2s.  Follow the firmware's lane.
 */
#define DR_LANE 0x000000ffu

static uint32_t gr1_read_word(SGIGR1State *s, uint32_t eff, uint32_t mask)
{
    uint32_t x = eff & 0x7fff;

    /* banked, lower half (bank 0) */
    if (eff < 0x0400) {
        uint16_t pc = (eff >> 2) | ((uint16_t)(s->mar & 0x7f) << 8);

        s->pc = pc;
        return (uint32_t)ge5_code(s, pc);
    }
    if (eff >= 0x1400 && eff < 0x1800) {
        s->memptr = ((eff - 0x1400) >> 2) | ((uint16_t)(s->mar & 0x3f) << 8);
        return ge5_data_r(s, s->memptr);
    }
    /* mirrored in both banks */
    if (x >= 0x0800 && x < 0x0c00) {
        return (uint32_t)s->bus;            /* buffer_r */
    }
    if (x >= 0x2000 && x < 0x2008) {
        return s->finish[(x - 0x2000) >> 2];
    }
    /* banked, upper half (bank 1) */
    if (eff >= 0x8000 && eff < 0x8400) {
        uint16_t pc = ((eff - 0x8000) >> 2) | ((uint16_t)(s->mar & 0x7f) << 8);

        s->pc = pc;
        return (uint32_t)(ge5_code(s, pc) >> 32);
    }
    if (eff >= 0x8400 && eff < 0x84a0) {
        return xmap_reg_r(&s->xmap[(eff - 0x8400) >> 5], (eff & 0x1f) >> 2);
    }
    switch (eff) {
    case 0x84c0: return s->dr1;
    case 0x84e0: return s->dr0;
    case 0x85a0: return (s->dr4 | DR4_GESTALL) & DR4_RM;
    case 0x85c0: return s->dr3;
    case 0x85e0: return s->dr2;
    case 0x8740: return (uint32_t)s->pc << 16;
    }
    if (eff >= 0x8500 && eff < 0x8550 && (eff & 0x1f) < 0x10) {
        return bt457_r(&s->ramdac[(eff - 0x8500) >> 5], (eff & 0xf) >> 2);
    }
    if (eff >= 0x8560 && eff < 0x85a0 && (eff & 0x1f) < 0x10) {
        return bt431_r(&s->cursor[(eff - 0x8560) >> 5], (eff & 0xf) >> 2);
    }
    qemu_log_mask(LOG_UNIMP, "sgi-gr1: unmapped read 0x%04x\n", eff);
    return 0;
}

static void gr1_write_word(SGIGR1State *s, uint32_t eff, uint32_t data,
                           uint32_t mask)
{
    uint32_t x = eff & 0x7fff;

    if (eff < 0x0400) {
        uint16_t pc = (eff >> 2) | ((uint16_t)(s->mar & 0x7f) << 8);

        s->pc = pc;
        if (pc < GE5_CODE_WORDS) {
            s->code[pc] = (s->code[pc] & ~(uint64_t)mask) | (data & mask);
        }
        return;
    }
    if (eff >= 0x1400 && eff < 0x1800) {
        s->memptr = ((eff - 0x1400) >> 2) | ((uint16_t)(s->mar & 0x3f) << 8);
        ge5_data_w(s, s->memptr, data, mask);
        return;
    }
    if (x >= 0x0800 && x < 0x0c00) {
        gr1_fifo_w(s, (x - 0x0800) >> 2, data);
        return;
    }
    if (x >= 0x0c00 && x < 0x0e00) {
        s->mar = ((x - 0x0c00) >> 2) & 0x7f;
        return;
    }
    if (x >= 0x0e00 && x < 0x0e08) {
        s->bank = (x - 0x0e00) >> 2;       /* mar_msb */
        return;
    }
    if (x >= 0x2000 && x < 0x2008) {
        s->finish[(x - 0x2000) >> 2] = data;
        return;
    }
    if (eff >= 0x8000 && eff < 0x8400) {
        uint16_t pc = ((eff - 0x8000) >> 2) | ((uint16_t)(s->mar & 0x7f) << 8);
        uint64_t m = (uint64_t)(mask & 0xff) << 32;

        s->pc = pc;
        /* MAME's FIXME, verbatim: force the secondary instruction bit */
        if ((data & 0x100) && !(data & 0x10)) {
            data |= 0x10;
        }
        if (pc < GE5_CODE_WORDS) {
            s->code[pc] = (s->code[pc] & ~m) | (((uint64_t)data << 32) & m);
        }
        return;
    }
    if (eff >= 0x8400 && eff < 0x84c0) {
        if (!(mask & 0xff)) {
            return;
        }
        if (eff < 0x84a0) {
            xmap_reg_w(&s->xmap[(eff - 0x8400) >> 5], (eff & 0x1f) >> 2, data);
        } else {
            for (int i = 0; i < 5; i++) {
                xmap_reg_w(&s->xmap[i], (eff & 0x1f) >> 2, data);
            }
        }
        s->dirty = true;
        return;
    }
    if (eff >= 0x8640 && eff < 0x8784) {
        if (eff != 0x8740) {
            ge5_command_w(s, (eff - 0x8640) >> 2);
            ge5_run(s);
        }
        return;
    }
    if (eff >= 0x8500 && eff < 0x8550 && (eff & 0x1f) < 0x10) {
        if (mask & 0xff) {
            bt457_w(&s->ramdac[(eff - 0x8500) >> 5], (eff & 0xf) >> 2, data);
            s->dirty = true;
        }
        return;
    }
    if (eff >= 0x8560 && eff < 0x85a0 && (eff & 0x1f) < 0x10) {
        if (mask & 0xff) {
            bt431_w(&s->cursor[(eff - 0x8560) >> 5], (eff & 0xf) >> 2, data);
            s->dirty = true;
        }
        return;
    }
    if (mask & DR_LANE) {
        uint8_t v = data;

        switch (eff) {
        case 0x84c0:
            s->dr1 = (s->dr1 & ~DR1_WM) | (v & DR1_WM);
            /*
             * MAME: m_ge->cwen_w(BIT(data, 1)).  With MAME's lane the PROM's
             * "sw" delivers 0 here, so MAME's engine always runs with CWEN
             * clear; keep that unless asked (bit 1 doubles as monitor type).
             */
            if (s->cwen_from_dr1) {
                s->cwen = (v >> 1) & 1;
            }
            return;
        case 0x84e0:
            s->dr0 = (s->dr0 & ~DR0_WM) | (v & DR0_WM);
            return;
        case 0x85a0:
            s->dr4 = (s->dr4 & ~DR4_WM) | (v & DR4_WM);
            for (int i = 0; i < 5; i++) {
                s->xmap[i].map_select = !!(s->dr4 & DR4_MS);
            }
            s->dirty = true;
            return;
        case 0x85c0:
            s->dr3 = (s->dr3 & ~DR3_WM) | (v & DR3_WM);
            return;
        case 0x85e0:
            s->dr2 = (s->dr2 & ~DR2_WM) | (v & DR2_WM);
            return;
        }
    }
    qemu_log_mask(LOG_UNIMP, "sgi-gr1: unmapped write 0x%04x <- 0x%x\n",
                  eff, data);
}

static uint64_t gr1_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIGR1State *s = opaque;
    uint32_t eff = (s->bank ? 0x8000 : 0) | (addr & 0x7ffc);
    uint32_t mask = lane_mask(addr, size);
    uint32_t v = gr1_read_word(s, eff, mask);
    unsigned shift = (4 - size - (addr & 3)) * 8;

    trace_sgi_gr1_read(eff | (addr & 3), size, v);
    return (v & mask) >> shift;
}

static void gr1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SGIGR1State *s = opaque;
    uint32_t eff = (s->bank ? 0x8000 : 0) | (addr & 0x7ffc);
    uint32_t mask = lane_mask(addr, size);
    unsigned shift = (4 - size - (addr & 3)) * 8;
    uint32_t data = (uint32_t)val << shift;

    trace_sgi_gr1_write(eff | (addr & 3), size, (uint32_t)val);
    gr1_write_word(s, eff, data, mask);
}

static const MemoryRegionOps gr1_ops = {
    .read = gr1_read,
    .write = gr1_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- reset line and vertical blank ------------------------------------ */

static void gr1_reset_line(void *opaque, int n, int level)
{
    SGIGR1State *s = opaque;

    /* MAME reset_w(): a 1 -> 0 transition of cpuauxctl bit 7 resets GE+RE */
    if (!s->reset && !level) {
        trace_sgi_gr1_board_reset();
        ge5_reset(s);
        re2_reset(s);
    }
    s->reset = !level;
}

static void gr1_vbl_cb(void *opaque)
{
    SGIGR1State *s = opaque;
    int64_t line_ns = NANOSECONDS_PER_SECOND / (60 * GR1_VTOTAL);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->vblank = !s->vblank;
    qemu_set_irq(s->irq[SGI_GR1_IRQ_VBLANK], s->vblank);
    timer_mod(s->vbl_timer, now + line_ns *
              (s->vblank ? (GR1_VTOTAL - GR1_VBSTART + GR1_VBEND)
                         : (GR1_VBSTART - GR1_VBEND)));
}

/* ---- display (sgi_re2.cpp screen_update) ------------------------------ */

static void gr1_update_display(void *opaque)
{
    SGIGR1State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t *dest;

    if (!surface || !s->dirty) {
        return;
    }
    s->dirty = false;
    dest = (uint32_t *)surface_data(surface);

    for (int sy = 0; sy < GR1_SCREEN_H; sy++) {
        unsigned mem_y = 1023 - sy;    /* TODO (MAME): variable topscan */

        for (int sx = 0; sx < GR1_SCREEN_W; sx++) {
            unsigned mem_x = sx;
            XMAP2 *xm = &s->xmap[mem_x % 5];
            uint32_t data = s->vram[mem_y * 0x500 + mem_x];
            uint16_t mode = xm->mode[data >> 28];
            uint32_t color = ((data & 0xff) << 16) | (data & 0xff00) |
                             ((data >> 16) & 0xff);  /* 24 bit rgb default */
            uint8_t cursor;

            if (((data >> 20) & mode & XMAP_MODE_OE) ||
                ((mode & XMAP_MODE_UE) && !(data & 0x00ffffffu))) {
                color = xmap_overlay_r(xm, data >> 24);
            } else {
                switch (mode & XMAP_MODE_DM) {
                case 0: {
                    uint16_t index = (mode & XMAP_MODE_ME) ?
                        ((mode & XMAP_MODE_MC) >> 2) | (uint8_t)data :
                        (uint8_t)data;

                    color = xmap_pen(xm, index);
                    break;
                }
                case 1: {
                    uint8_t buffer = (mode & XMAP_MODE_BS) ?
                        (uint8_t)data >> 4 : data & 0x0f;
                    uint16_t index = (mode & XMAP_MODE_ME) ?
                        ((mode & XMAP_MODE_MC) >> 2) | buffer : buffer;

                    color = xmap_pen(xm, index);
                    break;
                }
                case 2: {
                    uint16_t buffer = (uint16_t)((mode & XMAP_MODE_BS) ?
                                                 data >> 12 : data) & 0x0fff;
                    uint16_t index = (mode & XMAP_MODE_ME) ?
                        ((mode & XMAP_MODE_MC) >> 2) | (buffer & 0xff) : buffer;

                    color = xmap_pen(xm, index);
                    break;
                }
                case 5: {
                    uint8_t r, g, b;

                    if (mode & XMAP_MODE_BS) {
                        r = ((data >> 0x00) & 0xf0) | ((data >> 0x04) & 0x0f);
                        g = ((data >> 0x08) & 0xf0) | ((data >> 0x0c) & 0x0f);
                        b = ((data >> 0x10) & 0xf0) | ((data >> 0x14) & 0x0f);
                    } else {
                        r = ((data << 0x04) & 0xf0) | ((data >> 0x00) & 0x0f);
                        g = ((data >> 0x04) & 0xf0) | ((data >> 0x08) & 0x0f);
                        b = ((data >> 0x0c) & 0xf0) | ((data >> 0x10) & 0x0f);
                    }
                    color = (r << 16) | (g << 8) | b;
                    break;
                }
                }
            }

            cursor = (bt431_cur(&s->cursor[0], sx + GR1_HBEND, sy + GR1_VBEND) ? 1 : 0) |
                     (bt431_cur(&s->cursor[1], sx + GR1_HBEND, sy + GR1_VBEND) ? 2 : 0);

            dest[sy * GR1_SCREEN_W + sx] =
                (bt457_lookup(&s->ramdac[0], rgb_r(color), cursor) << 16) |
                (bt457_lookup(&s->ramdac[1], rgb_g(color), cursor) << 8) |
                bt457_lookup(&s->ramdac[2], rgb_b(color), cursor);
        }
    }
    dpy_gfx_update(s->con, 0, 0, GR1_SCREEN_W, GR1_SCREEN_H);
}

static void gr1_invalidate(void *opaque)
{
    SGIGR1State *s = opaque;

    s->dirty = true;
}

static const GraphicHwOps gr1_gfx_ops = {
    .invalidate = gr1_invalidate,
    .gfx_update = gr1_update_display,
};

/* ---- QOM -------------------------------------------------------------- */

static void gr1_reset_hold(Object *obj, ResetType type)
{
    SGIGR1State *s = SGI_GR1(obj);

    s->bank = 0;
    s->dr0 = DR0_GRF1EN | DR0_SMALLMON0;
    s->dr1 = DR1_TURBO;
    s->dr2 = 0;
    s->dr3 = 0;
    s->dr4 = DR4_MEGOPT;
    s->fifo_head = 0;
    s->fifo_count = 0;
    qemu_set_irq(s->irq[SGI_GR1_IRQ_FIFO], 0);
    s->reset = true;
    s->cwen = false;

    ge5_reset(s);
    re2_reset(s);
    for (int i = 0; i < 5; i++) {
        s->xmap[i].options = GR1_OPTIONS;
    }
    for (int i = 0; i < 2; i++) {
        s->cursor[i].address = 0;
        s->cursor[i].command = 0;
        bt431_update(&s->cursor[i]);
    }
    s->dirty = true;

    s->vblank = false;
    timer_mod(s->vbl_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / 60);
}

static void gr1_realize(DeviceState *dev, Error **errp)
{
    SGIGR1State *s = SGI_GR1(dev);
    WTL3132 *w = &s->fpu;

    s->code = g_new0(uint64_t, GE5_CODE_WORDS);
    s->data = g_new0(uint32_t, GE5_DATA_WORDS);
    s->vram = g_new0(uint32_t, GR1_VRAM_WORDS);

    /* softfloat3 defaults: round to nearest even, no flush to zero */
    set_float_rounding_mode(float_round_nearest_even, &w->fs);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, &w->fs);
    set_float_3nan_prop_rule(float_3nan_prop_s_cab, &w->fs);
    set_float_infzeronan_rule(float_infzeronan_dnan_never, &w->fs);
    set_float_default_nan_pattern(0b01000000, &w->fs);

    s->ge_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ge5_timer_cb, s);
    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gr1_vbl_cb, s);

    s->con = graphic_console_init(dev, 0, &gr1_gfx_ops, s);
    qemu_console_resize(s->con, GR1_SCREEN_W, GR1_SCREEN_H);
}

static void gr1_init(Object *obj)
{
    SGIGR1State *s = SGI_GR1(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &gr1_ops, s, "sgi-gr1", 0x8000);
    sysbus_init_mmio(sbd, &s->mmio);
    for (int i = 0; i < 3; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    qdev_init_gpio_in(DEVICE(obj), gr1_reset_line, 1);
}

static const Property gr1_properties[] = {
    DEFINE_PROP_BOOL("re-nowrite", SGIGR1State, re_nowrite, false),
    DEFINE_PROP_BOOL("cwen-from-dr1", SGIGR1State, cwen_from_dr1, false),
};

static void gr1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "SGI GR1 (Eclipse) graphics";
    dc->realize = gr1_realize;
    rc->phases.hold = gr1_reset_hold;
    device_class_set_props(dc, gr1_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo gr1_info = {
    .name = TYPE_SGI_GR1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGR1State),
    .instance_init = gr1_init,
    .class_init = gr1_class_init,
};

static void gr1_register_types(void)
{
    type_register_static(&gr1_info);
}

type_init(gr1_register_types)
