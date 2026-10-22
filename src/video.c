/* Neo Geo video per MAME 2025 hardware-accurate layout:
 *  fix layer:    vram word 0x7000 + ty + 0x20*tx, entry = code(12b) | pal(4b)<<12
 *                gfx byte = fixed[(code<<5|y)+off], off {0x10,0x18,0x00,0x08} = px pairs
 *  sprites:      per-scanline active lists; tables at 0x8000(zoom) 0x8200(y/rows/chain)
 *                0x8400(x); tile code+attr at (n<<6)|(tile<<1); gfx via predecoded 8bpp
 *  CRAM:         0x8000 blank | B:5 G:5 R:5
 *
 * MAME parity (neogeo_spr.cpp sprite_line_timer_callback): the screen is drawn
 * ONE SCANLINE at a time, interleaved with the CPUs — at the start of each line
 * MAME draws the previous line (update_partial) and re-parses the sprite lists
 * (parse_sprites). Mid-frame VRAM/CRAM writes therefore become visible. The
 * emulator drives this via ng_render_begin + ng_draw_line from ng_run_frame's
 * chunk loop; ng_render() keeps the whole-frame path for headless dumps.
 * (Coordinate system kept self-consistent: fb row == visible line 0..223,
 *  sprite y in the same space, list parity = row&1.)
 */
#include "linoragx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static int envf_vdbg(void){ static int v=-1; if(v<0) v=getenv("VDBG")?1:0; return v; }
static int envf_nofix(void){ static int v=-1; if(v<0) v=getenv("NOFIX")?1:0; return v; }
static int envf_nosp(void){ static int v=-1; if(v<0) v=getenv("NOSP")?1:0; return v; }


uint16_t *ng_vram(neogeo *ng);
uint16_t *ng_cram(neogeo *ng);
uint8_t  *ng_fixed(neogeo *ng, uint32_t *sz);
int       ng_fixed_bank_type(neogeo *ng);
uint8_t  *ng_sprite8(neogeo *ng, uint32_t *mask);
uint8_t  *ng_zoomy(neogeo *ng);
int       ng_auto_counter(neogeo *ng);

#define MAX_SPRITES_PER_SCREEN 381
#define MAX_SPRITES_PER_LINE   110

static const uint16_t zoom_x_tables[16] =
{ 0x0080,0x0880,0x0888,0x2888,0x288a,0x2a8a,0x2aaa,0xaaaa,
  0xaaea,0xbaea,0xbaeb,0xbbeb,0xbbef,0xfbef,0xfbff,0xffff };

/* MAME neogeo_v.cpp palette model.  CRAM word layout (NOT contiguous 5-bit
   fields): bit15=dark, and each channel's LSB is scattered in the high bits:
       R = {b11,b10,b9,b8,b14}  G = {b7,b6,b5,b4,b13}  B = {b3,b2,b1,b0,b12}
   Each 5-bit channel drives an R-2R resistor DAC (3900/2200/1000/470/220 ohm);
   the dark bit adds an 8200 ohm pulldown, the screen-shadow latch a 150 ohm
   one.  Weights are computed exactly like MAME's compute_resistor_weights
   (src/lib/video/resnet.cpp) and combined per set bit (combine_weights). */
static uint8_t cram_lut[32][4];   /* [value][dark + 2*shadow] */
static int cram_lut_ready = 0;

static void cram_lut_init(void)
{
    static const double res[5] = { 3900, 2200, 1000, 470, 220 };
    double weights[4][5], scaler = 0;
    /* MAME neogeo_v.cpp: dark+shadow pulldown is the parallel of 8200 and 150
       ohm, kept as a double (147.26), not truncated. */
    const double pd[4] = { 0, 8200, 150, 1.0 / ((1.0 / 8200) + (1.0 / 150)) };
    for (int k = 0; k < 4; k++) {
        double w[5], sum = 0;
        for (int n = 0; n < 5; n++) {
            /* MAME resnet.cpp: an absent pulldown/pullup is an OPEN net, i.e.
               conductance 1.0/1e12 = 1e-12, NOT 1e12 (that would be a short). */
            double g0 = (pd[k] == 0) ? 1.0 / 1e12 : 1.0 / pd[k];  /* pulldown conductance */
            double g1 = 1.0 / 1e12;                               /* no pullup: open */
            for (int j = 0; j < 5; j++) {
                if (j == n) g1 += 1.0 / res[j];  /* this bit high: resistor to Vcc */
                else        g0 += 1.0 / res[j];  /* this bit low: resistor to gnd */
            }
            double R0 = 1.0 / g0, R1 = 1.0 / g1;
            w[n] = 255.0 * R0 / (R1 + R0);
        }
        if (k == 0) {                          /* normal net defines the shared scaler */
            for (int n = 0; n < 5; n++) sum += w[n];
            scaler = 255.0 / sum;
        }
        for (int n = 0; n < 5; n++) weights[k][n] = w[n] * scaler;
    }
    for (int i = 0; i < 32; i++)
        for (int k = 0; k < 4; k++) {
            double s = 0;
            for (int b = 0; b < 5; b++) if ((i >> b) & 1) s += weights[k][b];
            int v = (int)(s + 0.5);
            cram_lut[i][k] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
        }
    cram_lut_ready = 1;
    if (getenv("CRAMLUT")) {
        for (int i = 0; i < 32; i++)
            fprintf(stderr, "LUT %2d: %3d %3d %3d %3d\n", i,
                    cram_lut[i][0], cram_lut[i][1], cram_lut[i][2], cram_lut[i][3]);
    }
}

static inline uint32_t cram24(uint16_t c, int shadow)
{
    if (!cram_lut_ready) cram_lut_init();
    uint32_t dark = (c >> 15) & 1;
    uint32_t r = ((c >> 14) & 0x1) | ((c >> 7) & 0x1e);
    uint32_t g = ((c >> 13) & 0x1) | ((c >> 3) & 0x1e);
    uint32_t b = ((c >> 12) & 0x1) | ((c << 1) & 0x1e);
    int k = (int)(dark + (shadow ? 2 : 0));
    return 0xFF000000 | (cram_lut[r][k] << 16) | (cram_lut[g][k] << 8) | cram_lut[b][k];
}

/* MAME parse_sprites: the priority lists are built by the emulator each line
   (games never write them) — objects in table order, max 110 per line. */
/* PIXDBG=x,y pixel probe state (file scope so fix-layer path can query it) */
static int g_pdx = -2, g_pdy = -1;
int ng_pixdbg(int x, int y)
{
    if (g_pdx == -2) { const char *e = getenv("PIXDBG");
        if (e) sscanf(e, "%d,%d", &g_pdx, &g_pdy); else g_pdx = -1; }
    return g_pdx >= 0 && y == g_pdy && x == g_pdx;
}

static void parse_sprites(uint16_t *vram, int scanline)
{
    uint16_t *list = &vram[(scanline & 1) ? 0x8680 : 0x8600];
    uint16_t *dst = list;
    int y = 0, rows = 0, cnt = 0;
    for (int n = 0; n < MAX_SPRITES_PER_SCREEN; n++) {
        uint16_t yc = vram[0x8200 | n];
        if (!(yc & 0x40)) {                 /* not chained: fresh y/rows */
            y = 0x200 - (yc >> 7);
            rows = yc & 0x3F;
        }
        /* MAME parse_sprites skips rows==0 BEFORE sprite_on_scanline: the
           per-line list never contains 0-row sprites (the rows==0 arm of
           sprite_on_scanline only matters for the regular draw path). */
        if (rows == 0) continue;
        if (!(rows >= 0x20 || ((scanline - y) & 0x1FF) < rows * 0x10)) continue;
        *dst++ = n;
        if (++cnt == MAX_SPRITES_PER_LINE) break;
    }
    memset(dst, 0, (MAX_SPRITES_PER_LINE - cnt + 1) * 2);
}

/* frame start: MAME bitmap.fill(bg_pen) — background colour is CRAM[bank|0xFFF].
   MAME renders the whole frame once at vblank, so the screen-shadow latch is
   sampled ONCE per frame here; the BIOS toggles ch0 dozens of times per frame
   (fade effects) and reading it per scanline would darken part of the frame
   ("translucent black flicker on the upper half"). */
static int render_shadow;

void ng_render_begin(neogeo *ng, uint32_t *fb)
{
    uint16_t *cram = ng_cram(ng);
    uint32_t bank = ng_cram_bank_render(ng);
    uint32_t sh = ng_screen_shadow(ng);
    render_shadow = (int)sh;
    { extern int ng_pixdbg(int,int);
      static uint64_t lastf = ~0ull; extern uint64_t ng_frame_no(neogeo*);
      if (ng_pixdbg(300,140) && ng_frame_no(ng) != lastf && ng_frame_no(ng) >= 19990) { lastf = ng_frame_no(ng);
          fprintf(stderr, "PIX begin f=%llu sh=%u\n", (unsigned long long)lastf, sh); } }
    uint32_t bg = cram24(cram[bank | 0xFFF], sh);
    for (int i = 0; i < NG_W*NG_H; i++) fb[i] = bg;

    if (envf_vdbg()) {
        uint16_t *vram = ng_vram(ng);
        uint32_t fix_sz; uint8_t *fixed = ng_fixed(ng, &fix_sz);
        uint32_t fix_mask = fix_sz - 1;
        uint16_t e0 = vram[0x7000]; int c0=e0&0xFFF, p0=e0>>12;
        const uint16_t *ps = &cram[(p0<<4)|bank];
        fprintf(stderr, "VDBG e0=%04x code=%x pal=%x bank=%x pens[3]=%04x c24=%08x base=%x g10=%02x\n",
            e0, c0, p0, bank, ps[3], (unsigned)cram24(ps[3], 0), (c0<<5)&fix_mask, fixed?fixed[((c0<<5)&fix_mask)+0x10]:0);
        fprintf(stderr, "VDBG fb0=%08x active?\n", (unsigned)fb[0]);
    }
}

/* one visible scanline (fb row 0..223), MAME order: sprites first, then the
   fix layer overwrites them (fix has priority over sprites). */
void ng_draw_line(neogeo *ng, uint32_t *fb, int line)
{
    uint16_t *vram = ng_vram(ng);
    uint16_t *cram = ng_cram(ng);
    uint32_t bank = ng_cram_bank_render(ng);
    uint32_t fix_sz, spr_mask;
    int sh = render_shadow;   /* frame-start snapshot (MAME vblank semantics) */
    uint8_t *fixed = ng_fixed(ng, &fix_sz);
    uint8_t *spr8  = ng_sprite8(ng, &spr_mask);
    uint8_t *zoomy = ng_zoomy(ng);
    uint32_t fix_mask = fix_sz - 1;
    int ac = ng_auto_counter(ng);

    /* vertical content shift: user-reported 16-line downward offset (bottom
       of the picture cut off, black bar on top). Row `line` of the fb shows
       content row line+VOFF. Override with VOFF env (0 restores old map). */
    static int voff = -1;
    if (voff < 0) voff = getenv("VOFF") ? atoi(getenv("VOFF")) : 16;
    int dline = line;
    line += voff;

    /* ---- sprites for this line ---- */
    if (spr8 && !envf_nosp()) {
        static int nolist = -2;
        if (nolist == -2) nolist = getenv("NOPLIST") ? 1 : 0;
        if (!nolist) parse_sprites(vram, line);
        uint16_t *list = &vram[(line & 1) ? 0x8680 : 0x8600];  /* MAME: odd scanline -> 0x8680 */
        int maxsi = MAX_SPRITES_PER_LINE - 1;
        while (maxsi >= 0 && list[maxsi] == 0) maxsi--;
        if (maxsi != MAX_SPRITES_PER_LINE - 1) maxsi++;
        int x = 0, zoom_x = 0, y = 0, rows = 0, zoom_y = 0;

        /* draw_sprites: iterate the priority list; chain state machine must
           see every entry (even off-screen ones) exactly like MAME */
        for (int si = 0; si <= maxsi; si++) {
            int n = list[si] & 0x1FF;
            uint16_t yc = vram[0x8200 | n];
            uint16_t zc = vram[0x8000 | n];
            if (yc & 0x40) {                        /* chained */
                x = (x + zoom_x + 1) & 0x1FF;
                zoom_x = (zc >> 8) & 0x0F;
            } else {
                y = 0x200 - (yc >> 7);
                x = vram[0x8400 | n] >> 7;
                zoom_y = zc & 0xFF;
                zoom_x = (zc >> 8) & 0x0F;
                rows = yc & 0x3F;
            }
            if (x >= 0x140 && x <= 0x1F0) continue;
            if (!(rows >= 0x20 || ((line - y) & 0x1FF) < rows * 16)) continue;   /* list never holds rows==0 (parse_sprites); MAME double-check is equivalent here */

            int sprite_line = (line - y) & 0x1FF;
            int zoom_line = sprite_line & 0xFF;
            int invert = (sprite_line >> 8) & 1;
            if (invert) zoom_line ^= 0xFF;          /* MAME: vertical flip of zoom line */
            if (rows > 0x20) {
                zoom_line = zoom_line % ((zoom_y + 1) << 1);
                if (zoom_line > zoom_y) {
                    zoom_line = ((zoom_y + 1) << 1) - 1 - zoom_line;
                    invert = !invert;
                }
            }
            uint8_t syt = zoomy[(zoom_y << 8) | zoom_line];
            int sprite_y = syt & 0x0F;
            int tile = syt >> 4;
            if (invert) { sprite_y ^= 0x0F; tile ^= 0x1F; }

            int ao = (n << 6) | (tile << 1);
            uint16_t attr = vram[ao + 1];
            uint32_t code = (((uint32_t)attr << 12) & 0xF0000) | vram[ao];
            if (attr & 0x08) code = (code & ~7u) | (ac & 7);
            else if (attr & 0x04) code = (code & ~3u) | (ac & 3);
            if (attr & 0x02) sprite_y ^= 0x0F;      /* vflip */

            const uint16_t *pens = &cram[((attr >> 8) << 4) | bank];   /* MAME: m_pens base = palette + bank */
            uint32_t gfx = ((code << 8) | (sprite_y << 4)) & spr_mask;
            {   static int sprmax = -2; static uint32_t rawmax = 0;
                if (sprmax == -2) { sprmax = getenv("SPRMAX") ? 1 : 0; }
                if (sprmax) {
                    uint32_t raw = ((code << 8) | (sprite_y << 4));
                    if (raw > rawmax) { rawmax = raw; }
                    static long sc = 0;
                    if ((sc++ % 600000) == 0)
                        fprintf(stderr, "[sprmax] rawmax=%x mask=%x\n", rawmax, spr_mask);
                }
            }
            int x_inc = 1;
            if (attr & 0x01) { gfx += 0x0F; x_inc = -1; }   /* hflip */

            static int sprdbg_line = -2;
            if (sprdbg_line == -2) { const char *e = getenv("SPRDBG"); sprdbg_line = e ? atoi(e) : -1; }
            {   /* SPRROW=<line>: dump every sprite entry touching that line */
                static int sprrow = -2;
                if (sprrow == -2) { const char *e = getenv("SPRROW"); sprrow = e ? atoi(e) : -1; }
                if (line == sprrow) {
                    const uint8_t *gd = spr8 + gfx;
                    extern uint64_t ng_frame_no(neogeo *ng);
                    fprintf(stderr, "SPRROW f=%llu si=%d n=%d x=%d y=%d tile=%d sy=%d code=%x attr=%x gfx=%x "
                        "bytes=%02x %02x %02x %02x %02x %02x %02x %02x pens=%04x %04x %04x %04x\n",
                        (unsigned long long)ng_frame_no(ng), si, n, x, y, tile, sprite_y, (unsigned)code, attr,
                        (unsigned)gfx, gd[0],gd[1],gd[2],gd[3],gd[4],gd[5],gd[6],gd[7],
                        pens[0],pens[1],pens[2],pens[3]);
                }
            }
            if (line == sprdbg_line && si < 3) {
                const uint8_t *gd = spr8 + gfx;
                extern uint64_t ng_frame_no(neogeo *ng);
                fprintf(stderr, "SPRDBG f=%llu l=%d si=%d x=%x n=%d tile=%d sy=%d code=%x attr=%x gfx=%x "
                    "bytes=%02x %02x %02x %02x %02x %02x %02x %02x pens=%04x %04x %04x\n",
                    (unsigned long long)ng_frame_no(ng), line, si, x, n, tile, sprite_y, (unsigned)code, attr, (unsigned)gfx,
                    gd[0],gd[1],gd[2],gd[3],gd[4],gd[5],gd[6],gd[7],
                    pens[1], pens[2], pens[3]);
            }

            uint16_t zt = zoom_x_tables[zoom_x];
            const uint8_t *g = spr8 + gfx;
            /* PIXDBG=x,y : trace every sprite write to one screen pixel */
            #define PIXHIT(dx) ng_pixdbg(dx, dline)
            if (x <= 0x1F0) {                       /* MAME: no wrap-around */
                int dx = x;
                for (int px = 0; px < 16; px++) {
                    if (zt & 0x8000) {
                        if (dx < NG_W) {
                            uint8_t p = g[0];
                            if (PIXHIT(dx)) fprintf(stderr, "PIX spr si=%d n=%d x=%d y=%d rows=%d zx=%d zy=%d tile=%d code=%x attr=%x gfx=%x p=%d pens=%04x\n", si, n, x, y, rows, zoom_x, zoom_y, tile, (unsigned)code, attr, (unsigned)gfx, p, pens[p]);
                            if (p) fb[dline*NG_W + dx] = cram24(pens[p], sh);
                        }
                        dx++;
                    }
                    zt <<= 1;
                    if (!zt) break;
                    g += x_inc;
                }
            } else {                                /* MAME: wrap-around */
                int xw = x;
                int dx = 0;
                for (int px = 0; px < 16; px++) {
                    if (zt & 0x8000) {
                        if (xw >= 0x200) {
                            if (dx < NG_W) {
                                uint8_t p = g[0];
                                if (PIXHIT(dx)) fprintf(stderr, "PIX sprW si=%d n=%d x=%d y=%d rows=%d zx=%d zy=%d tile=%d code=%x attr=%x gfx=%x p=%d pens=%04x\n", si, n, x, y, rows, zoom_x, zoom_y, tile, (unsigned)code, attr, (unsigned)gfx, p, pens[p]);
                                if (p) fb[dline*NG_W + dx] = cram24(pens[p], sh);
                            }
                            dx++;
                        }
                        xw++;
                    }
                    zt <<= 1;
                    if (!zt) break;
                    g += x_inc;
                }
            }
        }
    }

    /* ---- fixed layer for this line (drawn LAST per MAME screen_update:
       non-pen0 pixels overwrite sprites; COLUMN-major table at WORD 0x7000,
       entry(row,col)=vram[0x7000+col*0x20+row], 40 cols x 32 rows, no scroll) ---- */
    if (fixed && !envf_nofix()) {
        static const uint8_t pbyte[8] = {0x10,0x10,0x18,0x18,0x00,0x00,0x08,0x08};
        int fbtype = ng_fixed_bank_type(ng);
        int fix_banked = (fbtype && fix_sz > 0x20000);
        int garouoffsets[32];
        if (fix_banked && fbtype == 1) {
            int garoubank = 0, k = 0, gy = 0;
            while (gy < 32) {
                if (vram[0x7500 + k] == 0x0200 && (vram[0x7580 + k] & 0xFF00) == 0xFF00) {
                    garoubank = vram[0x7580 + k] & 3;
                    garouoffsets[gy++] = garoubank;
                }
                garouoffsets[gy++] = garoubank;
                k += 2;
            }
        }
        int row = line >> 3;
        const uint16_t *vd = &vram[0x7000 + row];
        uint32_t *drow = &fb[dline*NG_W];
        for (int x = 0; x < NG_W; x++) {
            uint16_t e = vd[(x >> 3)*0x20];
            uint32_t code = e & 0xFFF;
            if (fix_banked) {   /* MAME FIX_BANKTYPE_GAROU / _KOF2000 */
                if (fbtype == 1) code += 0x1000 * (garouoffsets[(row - 2) & 31] ^ 3);
                else if (fbtype == 2) code += 0x1000 * (((vram[0x7500 + ((row - 1) & 31) + 32*(x/6)] >> (5 - (x%6))*2) & 3) ^ 3);
            }
            uint8_t d = fixed[( ((code << 5) | (line & 7)) & fix_mask ) | pbyte[x & 7]];
            uint8_t p = (x & 1) ? (d >> 4) : (d & 0x0F);
            { extern int ng_pixdbg(int,int);
              if (ng_pixdbg(x, line)) fprintf(stderr, "PIX fix code=%x e=%04x p=%d\n", (unsigned)code, e, p); }
            if (p) drow[x] = cram24(cram[((e >> 12) << 4) | p | bank], render_shadow);
        }
    }
}

/* whole-frame path (headless dumps, ctl dump): begin + all lines */
void ng_render(neogeo *ng, uint32_t *fb)
{
    ng_render_begin(ng, fb);
    for (int line = 0; line < NG_H; line++) ng_draw_line(ng, fb, line);
}
