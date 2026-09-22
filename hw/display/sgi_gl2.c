/*
 * Silicon Graphics IRIS 3130 GL2 graphics: UC4 + DC4 + BP3.
 *
 * High-level emulation of the update controller (UC4), display controller
 * (DC4) and bitplane cards (BP3) that together drive the 3130's monitor.
 * The PROM's own graphics code (stand/lib/gl: gl2_probe, gl2_scrinit,
 * gl2_putat, gl2_fill) is the behavioural contract; the board firmware PROMs
 * and the UC4 schematic are the hardware reference.
 *
 * The guest never addresses the bitplanes directly: it issues UC4 commands
 * (FILLRECT, DRAWCHAR, DRAWPIXEL...) and reads pixels back through
 * UC_READPIXELAB, so this model stores a per-pixel colour INDEX rather than
 * the physical planes.  The DC4 colormap maps index -> RGB for scanout.
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/sgi_gl2.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "qapi/error.h"

/* Screen geometry: GL2 default (stand/lib/gl/gl2gl.c). */
#define GL2_XDIM 1024
#define GL2_YDIM 768

/* The graphics MMIO window: MBIO + 0x2000 .. MBIO + 0x4fff. */
#define GL2_MMIO_BASE 0x2000
#define GL2_MMIO_SIZE 0x3000

/* Offsets within this region. */
#define R_FBC         0x0000
#define R_FBC_PIXEL   (R_FBC + 0x000)
#define R_FBC_FLAGS   (R_FBC + 0x400)
#define R_FBC_DATA    (R_FBC + 0x800)
#define R_GE_FLAGS    (R_FBC + 0xC00)
#define R_UC          0x1000
#define R_DC          0x2000

/* UC4 buffers (gl2uc4.h). */
#define UC_EDB    0x01
#define UC_ECB    0x02
#define UC_XSB    0x03
#define UC_XEB    0x04
#define UC_YSB    0x05
#define UC_YEB    0x06
#define UC_FMAB   0x07
#define UC_MDB    0x10
#define UC_RPB    0x11
#define UC_CFB    0x12
#define UC_NBUF   0x20

/* UC4 commands (gl2uc4.h). */
#define UC_READFONT     0x00
#define UC_WRITEFONT    0x01
#define UC_READREPEAT   0x02
#define UC_SETADDRS     0x03
#define UC_SAVEWORD     0x04
#define UC_DRAWWORD     0x05
#define UC_READLSTIP    0x06
#define UC_NOOP         0x07
#define UC_DRAWCHAR     0x09
#define UC_FILLRECT     0x0a
#define UC_FILLTRAP     0x0b
#define UC_SETSCRMASKX  0x10
#define UC_SETSCRMASKY  0x11
#define UC_SETCOLORCD   0x14
#define UC_SETCOLORAB   0x15
#define UC_SETWECD      0x16
#define UC_SETWEAB      0x17
#define UC_READPIXELCD  0x18
#define UC_READPIXELAB  0x19
#define UC_DRAWPIXELCD  0x1a
#define UC_DRAWPIXELAB  0x1b

/* DC4 flag bits (gl2dcdev.h). */
#define DC_BUSOP    0x10
#define DC_MULTIMAP 0x20
#define DC_RGBMODE  0x40

#define GL2_NMAP    16
#define GL2_NCOLOR  256
/* Font/pattern RAM: FONTBASEADDR (0xf400) + 256 chars * 16 words, plus slack. */
#define GL2_FONT_WORDS 0x14000

struct SGIGL2State {
    SysBusDevice parent;

    MemoryRegion mmio;
    qemu_irq irq;
    QemuConsole *con;

    /*
     * Bitplane value per pixel.  The UC4 writes planes A/B (colourAB/wrtenAB)
     * and C/D (colourCD/wrtenCD); the DC4 looks the resulting plane code up
     * in the colormap.  Storing the code (not the physical planes) is
     * invisible to the guest, which only reads back through READPIXEL.
     */
    uint8_t *fb;
    uint8_t cmap[GL2_NMAP][GL2_NCOLOR][3];
    uint16_t dc_flags;            /* DC4 flag register */
    uint8_t cur_map;              /* DC4 map selected for scanout */

    uint16_t buf[UC_NBUF];        /* UC4 buffer file */
    uint16_t color_ab, color_cd;
    uint16_t we_ab, we_cd;
    uint16_t ucr;
    uint16_t scrmaskx, scrmasky;
    uint16_t fmaddr;              /* current FM (font/pattern) address */

    uint16_t font[GL2_FONT_WORDS];
    uint16_t fbc_data;            /* FBC data latch */

    /*
     * GF2 FBC microstore and host interface.  The kernel uploads 4096 states
     * x 4 slices (64-bit microword) through FBDATA, with the slice in GEflags
     * bits 13-14 and the state's high 3 bits (block) in GEflags bits 10-12;
     * the low 9 bits come from the FBDATA word address.
     */
    uint16_t microram[4][4096];
    uint8_t micro_slice;
    uint8_t micro_block;
    bool micro_access;            /* FBC in WRITEMICRO/READMICRO mode */
    uint16_t fbc_flags;           /* last FBCflags command */
    uint16_t fbc_out;             /* FBC output register (reset/version) */
    uint16_t ge_flags;

    /*
     * GE command pipe (GEPORT, write-only).  Every write is a big-endian
     * stream of 16-bit words: a passthru header (low byte GEpassthru 0x08,
     * count-1 in bits 8-14), then the FBC command word, then count-1 operand
     * words.  Parsed here and executed against the framebuffer.
     */
    MemoryRegion ge;
    uint16_t ge_cmd;
    uint16_t ge_args[64];
    unsigned ge_nargs;            /* operands collected for the current cmd */
    unsigned ge_need;             /* operands the current cmd still wants */
    bool ge_in_cmd;

    bool testpattern;
    bool trace;
    bool dirty;
};

/*
 * Write the planes selected by a 2-bit write-enable *we* with the matching
 * bits of *color*, leaving the other planes untouched.  *shift* is 0 for the
 * A/B group and 2 for the C/D group.  This is what makes an erase that sets
 * only WE plane B (as the PROM's cursor/erase steps do) leave plane A -- where
 * the character was drawn -- intact.
 */
static uint8_t gl2_blend(uint8_t old, uint16_t color, uint16_t we, int shift)
{
    uint16_t mask = (we & 3) << shift;

    return (old & ~mask) | (((color & 3) << shift) & mask);
}

/* The plane code a FILLRECT/DRAWCHAR/DRAWPIXEL would write for this pixel. */
static uint8_t gl2_planes(SGIGL2State *s, uint8_t old)
{
    old = gl2_blend(old, s->color_ab, s->we_ab, 0);
    old = gl2_blend(old, s->color_cd, s->we_cd, 2);
    return old;
}

static void gl2_set_pixel(SGIGL2State *s, int x, int y, uint8_t val)
{
    if ((unsigned)x >= GL2_XDIM || (unsigned)y >= GL2_YDIM) {
        return;
    }
    s->fb[y * GL2_XDIM + x] = val;
}

static void gl2_fill_rect(SGIGL2State *s, int x0, int y0, int x1, int y1)
{
    int x, y;

    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            if ((unsigned)x < GL2_XDIM && (unsigned)y < GL2_YDIM) {
                gl2_set_pixel(s, x, y,
                              gl2_planes(s, s->fb[y * GL2_XDIM + x]));
            }
        }
    }
}

/*
 * Draw the character whose glyph lives in font RAM at address *addr*.  The
 * PROM loads each font word as (glyph_byte << 8) | 0xff, so bit 7 of the high
 * byte is the leftmost pixel.  Glyph cells are 8 wide by 16 tall
 * (CHARWIDTH/CHARHEIGHT).
 */
static void gl2_drawchar(SGIGL2State *s)
{
    /*
     * The PROM loads the glyph's font address straight into the FMAB buffer
     * (gl2_putat: LDFMADDR(CharToAddr(c)); DRAWCHAR) and only uses SETADDRS
     * when streaming the font in with WRITEFONT.
     */
    int addr = s->buf[UC_FMAB] % GL2_FONT_WORDS;
    int gx, gy, bit;

    for (gy = 0; gy < 16; gy++) {
        for (gx = 0; gx < 8; gx++) {
            uint16_t w = s->font[(addr + gy) % GL2_FONT_WORDS];
            int px = (int16_t)s->buf[UC_XSB] + gx;
            int py = (int16_t)s->buf[UC_YSB] + gy;

            bit = (w >> (15 - gx)) & 1;
            if (bit && (unsigned)px < GL2_XDIM && (unsigned)py < GL2_YDIM) {
                gl2_set_pixel(s, px, py,
                              gl2_planes(s, s->fb[py * GL2_XDIM + px]));
            }
        }
    }
}

static void gl2_exec(SGIGL2State *s, unsigned cmd, uint16_t val)
{
    if (s->trace) {
        fprintf(stderr, "gl2: cmd=0x%02x val=0x%04x ab=0x%04x we=0x%04x "
                "box=(%d,%d)-(%d,%d) fmaddr=0x%x fmab=0x%x\n", cmd, val,
                s->color_ab, s->we_ab, (int16_t)s->buf[UC_XSB],
                (int16_t)s->buf[UC_YSB], (int16_t)s->buf[UC_XEB],
                (int16_t)s->buf[UC_YEB], s->fmaddr, s->buf[UC_FMAB]);
    }

    switch (cmd) {
    case UC_SETCOLORAB:
        s->color_ab = val;
        break;
    case UC_SETCOLORCD:
        s->color_cd = val;
        break;
    case UC_SETWEAB:
        s->we_ab = val;
        break;
    case UC_SETWECD:
        s->we_cd = val;
        break;
    case UC_SETSCRMASKX:
        s->scrmaskx = val;
        break;
    case UC_SETSCRMASKY:
        s->scrmasky = val;
        break;
    case UC_SETADDRS:
        s->fmaddr = s->buf[UC_FMAB] % GL2_FONT_WORDS;
        break;
    case UC_WRITEFONT:
        s->font[s->fmaddr++ % GL2_FONT_WORDS] = val;
        break;
    case UC_FILLRECT:
        gl2_fill_rect(s, (int16_t)s->buf[UC_XSB], (int16_t)s->buf[UC_YSB],
                      (int16_t)s->buf[UC_XEB], (int16_t)s->buf[UC_YEB]);
        break;
    case UC_DRAWCHAR:
        gl2_drawchar(s);
        break;
    case UC_DRAWPIXELAB: {
        int x = (int16_t)s->buf[UC_XSB], y = (int16_t)s->buf[UC_YSB];

        if ((unsigned)x < GL2_XDIM && (unsigned)y < GL2_YDIM) {
            uint8_t old = s->fb[y * GL2_XDIM + x];

            old = gl2_blend(old, s->color_ab, s->we_ab, 0);
            gl2_set_pixel(s, x, y, old);
        }
        break;
    }
    case UC_DRAWPIXELCD: {
        int x = (int16_t)s->buf[UC_XSB], y = (int16_t)s->buf[UC_YSB];

        if ((unsigned)x < GL2_XDIM && (unsigned)y < GL2_YDIM) {
            uint8_t old = s->fb[y * GL2_XDIM + x];

            old = gl2_blend(old, s->color_cd, s->we_cd, 2);
            gl2_set_pixel(s, x, y, old);
        }
        break;
    }
    case UC_READFONT:
    case UC_READREPEAT:
    case UC_SAVEWORD:
    case UC_DRAWWORD:
    case UC_READLSTIP:
    case UC_NOOP:
    case UC_FILLTRAP:
    default:
        break;
    }
}

static uint64_t gl2_read(void *opaque, hwaddr addr, unsigned size)
{
    SGIGL2State *s = opaque;
    unsigned off = addr;

    if (off < R_UC) {
        if (s->trace) {
            fprintf(stderr, "gl2: FBC RD off=0x%x\n", off);
        }
        switch (off & ~0x3ff) {
        case R_FBC_PIXEL & ~0x3ff:
            return 0;
        case R_FBC_FLAGS & ~0x3ff:
            return 0;
        case R_FBC_DATA & ~0x3ff: {
            unsigned low = (off & 0x3ff) >> 1;
            unsigned idx = s->micro_block * 512 + low;

            if (s->micro_access) {
                return idx < 4096 ? s->microram[s->micro_slice][idx] : 0;
            }
            return s->fbc_out;
        }
        default:
            return 0;           /* GEflags reads as 0 when idle */
        }
    }
    if (off < R_DC) {
        unsigned o = off - R_UC;
        if (o == 0x180) {
            /* UCR: report not-busy, no vertical interval. */
            return s->ucr & ~0x8000;
        }
        if (o >= 0x200) {
            unsigned cmd = (o - 0x200) >> 1;
            if (cmd == UC_READPIXELAB || cmd == UC_READPIXELCD) {
                int x = (int16_t)s->buf[UC_XSB];
                int y = (int16_t)s->buf[UC_YSB];
                if ((unsigned)x < GL2_XDIM && (unsigned)y < GL2_YDIM) {
                    return s->fb[y * GL2_XDIM + x];
                }
            }
            if (cmd == UC_READFONT) {
                return s->font[s->fmaddr % GL2_FONT_WORDS];
            }
            return 0;
        }
        return 0;
    }
    /* DC4: reading the colormap. */
    {
        unsigned o = off - R_DC;
        if (o >= 0x200 && o < 0x800) {
            unsigned chan = (o >> 9) & 3;       /* 1=red 2=green 3=blue */
            unsigned idx = (o >> 1) & 0xff;
            if (chan >= 1 && chan <= 3) {
                return s->cmap[s->cur_map][idx][chan - 1];
            }
        }
        return s->dc_flags;
    }
}

static void gl2_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SGIGL2State *s = opaque;
    unsigned off = addr;
    uint16_t v = val & 0xffff;

    if (off < R_UC) {
        if (s->trace) {
            fprintf(stderr, "gl2: FBC WR off=0x%x val=0x%04x\n", off, v);
        }
        switch (off & ~0x3ff) {
        case R_FBC_PIXEL & ~0x3ff:
            break;              /* clear interrupt / misc */
        case R_FBC_FLAGS & ~0x3ff:
            s->fbc_flags = v;
            s->micro_access = (v == 0xfe || v == 0xff); /* WRITE/READMICRO */
            break;
        case R_FBC_DATA & ~0x3ff: {
            unsigned low = (off & 0x3ff) >> 1;
            unsigned idx = s->micro_block * 512 + low;

            if (s->micro_access) {
                if (idx < 4096) {
                    s->microram[s->micro_slice][idx] = v;
                }
            } else {
                /*
                 * Output register: the reset/version queries the kernel
                 * writes (8 = scratch size, 7 = microcode version).
                 */
                switch (v) {
                case 8:
                    s->fbc_out = 0xfff;
                    break;
                case 7:
                    s->fbc_out = 0x0200;
                    break;
                default:
                    s->fbc_out = v;
                    break;
                }
            }
            break;
        }
        case R_GE_FLAGS & ~0x3ff:
            s->ge_flags = v;
            s->micro_slice = (v >> 13) & 3;
            s->micro_block = (v >> 10) & 7;
            break;
        default:
            break;
        }
        return;
    }
    if (off < R_DC) {
        unsigned o = off - R_UC;
        if (o == 0x180) {
            s->ucr = v;
            return;
        }
        if (o >= 0x80 && o < 0x80 + UC_NBUF * 2) {
            unsigned buf = (o - 0x80) >> 1;
            s->buf[buf] = v;
            return;
        }
        if (o >= 0x200) {
            unsigned cmd = (o - 0x200) >> 1;
            gl2_exec(s, cmd, v);
            return;
        }
        return;
    }
    {
        unsigned o = off - R_DC;
        if (o < 2) {
            if (s->trace) {
                fprintf(stderr, "gl2: DCflags=0x%04x\n", v);
            }
            s->dc_flags = v;
            s->cur_map = v & 0xf;
            return;
        }
        if (o >= 0x200 && o < 0x800) {
            unsigned chan = (o >> 9) & 3;
            unsigned idx = (o >> 1) & 0xff;
            if (chan >= 1 && chan <= 3) {
                if (s->trace) {
                    fprintf(stderr, "gl2: cmap[%d][%d][%d]=0x%04x\n",
                            s->dc_flags & 0xf, idx, chan - 1, v);
                }
                s->cmap[s->dc_flags & 0xf][idx][chan - 1] = v & 0xff;
            }
            return;
        }
    }
}

static void gl2_gfx_update(void *opaque)
{
    SGIGL2State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t *dest;
    int x, y;

    if (!surface) {
        return;
    }
    dest = (uint32_t *)surface_data(surface);
    for (y = 0; y < GL2_YDIM; y++) {
        for (x = 0; x < GL2_XDIM; x++) {
            uint8_t c = s->fb[y * GL2_XDIM + x];
            uint8_t *rgb = s->cmap[s->cur_map][c];
            dest[y * GL2_XDIM + x] = rgb_to_pixel32(rgb[0], rgb[1], rgb[2]);
        }
    }
    dpy_gfx_update(s->con, 0, 0, GL2_XDIM, GL2_YDIM);
}

static const GraphicHwOps gl2_gfx_ops = {
    .gfx_update = gl2_gfx_update,
};

static const MemoryRegionOps gl2_ops = {
    .read = gl2_read,
    .write = gl2_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* Plot one pixel with the current FBC colour and write-enable state. */
static void gl2_draw_pixel(SGIGL2State *s, unsigned x, unsigned y)
{
    uint8_t *p;

    if (x >= GL2_XDIM || y >= GL2_YDIM) {
        return;
    }
    p = &s->fb[y * GL2_XDIM + x];
    *p = gl2_blend(*p, s->color_ab, s->we_ab, 0);
    *p = gl2_blend(*p, s->color_cd, s->we_cd, 2);
    s->dirty = true;
}

/* Execute one FBC command delivered through the GE passthru pipe. */
static void gl2_ge_exec(SGIGL2State *s, uint16_t cmd,
                        const uint16_t *args, unsigned nargs)
{
    unsigned i;

    if (s->trace) {
        fprintf(stderr, "gl2: GE cmd=0x%02x nargs=%u", cmd, nargs);
        for (i = 0; i < nargs; i++) {
            fprintf(stderr, " %04x", args[i]);
        }
        fprintf(stderr, "\n");
    }

    switch (cmd) {
    case 0x04:                          /* FBCrgbcolor */
    case 0x05:                          /* FBCrgbwrten */
        if (nargs >= 2) {
            /* w0: plane enables for A/B (bits 0-1) and C/D (bits 2-3);
             * w1: matching colour bits. */
            if (cmd == 0x04) {
                s->color_ab = args[0];
                s->color_cd = args[0];
            } else {
                s->we_ab = args[0];
                s->we_cd = args[0];
            }
        }
        break;

    case 0x12:                          /* FBCpoint: x, y */
        if (nargs >= 2) {
            gl2_draw_pixel(s, args[0], args[1]);
        }
        break;

    case 0x2f:                          /* FBCpixelsetup */
        /*
         * A pixel-readback setup carries a sub-command word naming the read
         * operation.  The kernel issues this in gr_init as a self-test and
         * checks FBCdata for the matching interrupt code; model the
         * 32-plane readback code (_INTPIXEL32) it expects.
         */
        for (i = 0; i + 1 < nargs; i++) {
            if (args[i] == 0x000e) {
                s->fbc_out = 10;        /* _INTPIXEL32 */
                break;
            }
        }
        break;

    default:
        break;
    }
}

/* Assemble the 16-bit GE port stream and dispatch whole commands. */
static void gl2_ge_word(SGIGL2State *s, uint16_t w)
{
    if (!s->ge_in_cmd) {
        if ((w & 0xff) != 0x08) {   /* not a GEpassthru header */
            return;
        }
        s->ge_need = ((w >> 8) & 0x7f) + 1;     /* words incl. command */
        s->ge_cmd = 0;
        s->ge_nargs = 0;
        s->ge_in_cmd = true;
        return;
    }

    if (s->ge_nargs == 0) {
        s->ge_cmd = w;
    } else if (s->ge_nargs - 1 < ARRAY_SIZE(s->ge_args)) {
        s->ge_args[s->ge_nargs - 1] = w;
    }
    s->ge_nargs++;
    s->ge_need--;

    if (s->ge_need == 0) {
        gl2_ge_exec(s, s->ge_cmd, s->ge_args, s->ge_nargs - 1);
        s->ge_in_cmd = false;
    }
}

static uint64_t gl2_ge_read(void *opaque, hwaddr addr, unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)size;
    return 0;
}

static void gl2_ge_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SGIGL2State *s = opaque;

    if (s->trace) {
        fprintf(stderr, "gl2: GE WR off=0x%x val=0x%llx size=%u\n",
                (unsigned)addr, (unsigned long long)val, size);
    }
    if (addr < 0x1000) {
        return;                     /* GETOKEN: token/port control */
    }
    if (size == 4) {
        gl2_ge_word(s, (val >> 16) & 0xffff);
        gl2_ge_word(s, val & 0xffff);
    } else {
        gl2_ge_word(s, val & 0xffff);
    }
}

static const MemoryRegionOps gl2_ge_ops = {
    .read = gl2_ge_read,
    .write = gl2_ge_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void gl2_cmap_default(SGIGL2State *s)
{
    int i;

    for (i = 0; i < GL2_NCOLOR; i++) {
        s->cmap[0][i][0] = i;
        s->cmap[0][i][1] = (i * 3) & 0xff;
        s->cmap[0][i][2] = (i * 7) & 0xff;
    }
}

/* A deterministic pattern the model itself can draw, to prove the display
 * path with no guest involvement (P5 gate G0). */
static void gl2_test_pattern(SGIGL2State *s)
{
    int x, y;

    for (y = 0; y < GL2_YDIM; y++) {
        for (x = 0; x < GL2_XDIM; x++) {
            uint8_t c;
            if (y < GL2_YDIM / 4) {
                c = (x / 64) & 0x0f;                 /* vertical bars   */
            } else if (y < GL2_YDIM / 2) {
                c = (y * 255) / (GL2_YDIM / 2);       /* vertical ramp   */
            } else if (y < (GL2_YDIM * 3) / 4) {
                c = ((x / 16 + y / 16) & 1) ? 255 : 0; /* checkerboard   */
            } else {
                c = (x * 255) / GL2_XDIM;             /* horizontal ramp */
            }
            s->fb[y * GL2_XDIM + x] = c;
        }
    }
    /* white border */
    for (x = 0; x < GL2_XDIM; x++) {
        s->fb[x] = 255;
        s->fb[(GL2_YDIM - 1) * GL2_XDIM + x] = 255;
    }
    for (y = 0; y < GL2_YDIM; y++) {
        s->fb[y * GL2_XDIM] = 255;
        s->fb[y * GL2_XDIM + GL2_XDIM - 1] = 255;
    }
}

static void gl2_reset(DeviceState *dev)
{
    SGIGL2State *s = SGI_GL2(dev);

    memset(s->buf, 0, sizeof(s->buf));
    s->color_ab = s->color_cd = 0;
    s->we_ab = s->we_cd = 0;
    s->ucr = 0;
    s->scrmaskx = s->scrmasky = 0;
    s->fmaddr = 0;
    s->fbc_data = 0;
    s->micro_access = false;
    s->micro_slice = s->micro_block = 0;
    s->fbc_flags = 0;
    s->fbc_out = 0;
    s->ge_flags = 0;
    s->ge_in_cmd = false;
    s->ge_need = s->ge_nargs = 0;
    s->ge_cmd = 0;
    memset(s->ge_args, 0, sizeof(s->ge_args));
    memset(s->microram, 0, sizeof(s->microram));
    s->cur_map = 0;
    if (s->fb) {
        memset(s->fb, 0, (size_t)GL2_XDIM * GL2_YDIM * sizeof(uint8_t));
    }
    memset(s->font, 0, sizeof(s->font));
    gl2_cmap_default(s);
    if (s->testpattern && s->fb) {
        gl2_test_pattern(s);
    }
    s->dirty = true;
}

static void gl2_realize(DeviceState *dev, Error **errp)
{
    SGIGL2State *s = SGI_GL2(dev);

    s->con = graphic_console_init(dev, 0, &gl2_gfx_ops, s);
    qemu_console_resize(s->con, GL2_XDIM, GL2_YDIM);
}

static void gl2_init(Object *obj)
{
    SGIGL2State *s = SGI_GL2(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->fb = g_malloc0((size_t)GL2_XDIM * GL2_YDIM * sizeof(uint8_t));
    memory_region_init_io(&s->mmio, obj, &gl2_ops, s, "sgi-gl2",
                          GL2_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    memory_region_init_io(&s->ge, obj, &gl2_ge_ops, s, "sgi-gl2-ge",
                          0x2000);
    sysbus_init_mmio(sbd, &s->ge);
    sysbus_init_irq(sbd, &s->irq);
}

MemoryRegion *sgi_gl2_mmio_region(DeviceState *dev)
{
    return &SGI_GL2(dev)->mmio;
}

MemoryRegion *sgi_gl2_ge_region(DeviceState *dev)
{
    return &SGI_GL2(dev)->ge;
}

static const Property gl2_properties[] = {
    DEFINE_PROP_BOOL("testpattern", SGIGL2State, testpattern, false),
    DEFINE_PROP_BOOL("trace", SGIGL2State, trace, false),
};

static void gl2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gl2_realize;
    device_class_set_legacy_reset(dc, gl2_reset);
    device_class_set_props(dc, gl2_properties);
}

static const TypeInfo gl2_info = {
    .name          = TYPE_SGI_GL2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SGIGL2State),
    .instance_init = gl2_init,
    .class_init    = gl2_class_init,
};

static void gl2_register_types(void)
{
    type_register_static(&gl2_info);
}

type_init(gl2_register_types)
