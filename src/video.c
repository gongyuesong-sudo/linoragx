/* Neo Geo video per MAME 2025 hardware-accurate layout:
 *  fix layer:    vram word 0x7000 + ty + 0x20*tx, entry = code(12b) | pal(4b)<<12
 *                gfx byte = fixed[(code<<5|y)+off], off {0x10,0x18,0x00,0x08} = px pairs
 *  sprites:      per-scanline active lists; tables at 0x8000(zoom) 0x8200(y/rows/chain)
 *                0x8400(x); tile code+attr at (n<<6)|(tile<<1); gfx via predecoded 8bpp
 *  CRAM:         0x8000 blank | B:5 G:5 R:5
 */
#include "linoragx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static int envf_vdbg(void){ static int v=-1; if(v<0) v=getenv("VDBG")?1:0; return v; }
static int envf_notile(void){ static int v=-1; if(v<0) v=getenv("NOTILE")?1:0; return v; }
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
#define MAX_SPRITES_PER_LINE   96

static const uint16_t zoom_x_tables[16] =
{ 0x0080,0x0880,0x0888,0x2888,0x288a,0x2a8a,0x2aaa,0xaaaa,
  0xaaea,0xbaea,0xbaeb,0xbbeb,0xbbef,0xfbef,0xfbff,0xffff };

static inline uint32_t cram24(uint16_t c)
{
    /* Neo Geo CRAM word: XRRRRRGG GGGBBBBB (MAME: R=bits10-14, G=5-9, B=0-4) */
    uint32_t dark = (c >> 15) & 1;
    uint32_t r = (c >> 10) & 0x1F, g = (c >> 5) & 0x1F, b = c & 0x1F;
    r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
    if (dark) { r >>= 1; g >>= 1; b >>= 1; }
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

/* MAME parse_sprites: the priority lists are built by the emulator each frame
   (games never write them) — objects in table order, max 110 per line. */
#define MAX_SPRITES_PER_SCREEN 381
#define MAX_SPRITES_PER_LINE   110
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
        if (rows == 0) continue;
        if (!(rows >= 0x20 || ((scanline - y) & 0x1FF) < rows * 0x10)) continue;
        *dst++ = n;
        if (++cnt == MAX_SPRITES_PER_LINE) break;
    }
    memset(dst, 0, (MAX_SPRITES_PER_LINE - cnt + 1) * 2);
}

void ng_render(neogeo *ng, uint32_t *fb)
{
    uint16_t *vram = ng_vram(ng);
    uint16_t *cram = ng_cram(ng);
    uint32_t bank = ng_cram_bank_render(ng);
    uint32_t fix_sz, spr_mask;
    uint8_t *fixed = ng_fixed(ng, &fix_sz);
    uint8_t *spr8  = ng_sprite8(ng, &spr_mask);
    uint8_t *zoomy = ng_zoomy(ng);
    uint32_t fix_mask = fix_sz - 1;
    int ac = ng_auto_counter(ng);

    /* MAME fixed-layer banking (Garou/KOF2000 carts with >128K fix region) */
    int fbtype = ng_fixed_bank_type(ng);
    int fix_banked = (fbtype && fixed && fix_sz > 0x20000);
    int garouoffsets[32];
    if (fix_banked && fbtype == 1) {
        int garoubank = 0, k = 0, y = 0;
        while (y < 32) {
            if (vram[0x7500 + k] == 0x0200 && (vram[0x7580 + k] & 0xFF00) == 0xFF00) {
                garoubank = vram[0x7580 + k] & 3;
                garouoffsets[y++] = garoubank;
            }
            garouoffsets[y++] = garoubank;
            k += 2;
        }
    }

    /* MAME: bitmap.fill(bg_pen) — background colour is CRAM[bank|0xFFF] */
    {
        uint32_t bg = cram24(cram[bank | 0xFFF]);
        for (int i = 0; i < NG_W*NG_H; i++) fb[i] = bg;
    }

    /* ---- tile (background) layer: vram 0x9000-0x9fff, 32x32 grid of 8x8
       tiles sharing the s1 pattern region, scroll regs at vram[0x84]/[0x85];
       MAME draws it first (tilemap->draw), pen0 transparent ---- */
    if (fixed && !envf_notile()) {
        static const uint8_t pbyte[8] = {0x10,0x10,0x18,0x18,0x00,0x00,0x08,0x08};
        int scrollx = vram[0x84] & 0x1FF;
        int scrolly = vram[0x85] & 0x1FF;
        for (int y = 0; y < NG_H; y++) {
            int cy = (y + scrolly) & 0xFF;
            int row = (cy >> 3)*32, ry = cy & 7;
            uint32_t *drow = &fb[y*NG_W];
            for (int x = 0; x < NG_W; x++) {
                int cx = (x + scrollx) & 0xFF;
                uint16_t e = vram[0x4800 + row + (cx >> 3)];   /* byte 0x9000 = word 0x4800 (sprite tables live at word 0x8000+) */
                uint8_t d = fixed[( (((e & 0xFFF) << 5) | ry) & fix_mask ) | pbyte[cx & 7]];
                uint8_t p = (cx & 1) ? (d >> 4) : (d & 0x0F);
                if (p) drow[x] = cram24(cram[((e >> 12) << 4) | p | bank]);
            }
        }
    }

    if (envf_vdbg()) {
        uint16_t e0 = vram[0x7000]; int c0=e0&0xFFF, p0=e0>>12;
        const uint16_t *ps = &cram[(p0<<4)|bank];
        fprintf(stderr, "VDBG e0=%04x code=%x pal=%x bank=%x pens[3]=%04x c24=%08x base=%x g10=%02x\n",
            e0, c0, p0, bank, ps[3], (unsigned)cram24(ps[3]), (c0<<5)&fix_mask, fixed[((c0<<5)&fix_mask)+0x10]);
        fprintf(stderr, "VDBG fb0=%08x active?\n", (unsigned)fb[0]);
    }

    /* ---- sprites, per scanline (MAME draws sprites BEFORE the fix layer:
       the fix layer's non-pen0 pixels overwrite sprites) ---- */
    if (spr8 && !envf_nosp())
    for (int line = 0; line < NG_H; line++) {
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
            if (!(rows == 0 || rows >= 0x20 || ((line - y) & 0x1FF) < rows * 16)) continue;

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

            const uint16_t *pens = &cram[((attr >> 8) << 4) | bank];
            uint32_t gfx = ((code << 8) | (sprite_y << 4)) & spr_mask;
            int x_inc = 1;
            if (attr & 0x01) { gfx += 0x0F; x_inc = -1; }   /* hflip */

            static int sprdbg_line = -2;
            if (sprdbg_line == -2) { const char *e = getenv("SPRDBG"); sprdbg_line = e ? atoi(e) : -1; }
            if (line == sprdbg_line && si < 3) {
                const uint8_t *gd = spr8 + gfx;
                const uint8_t *raw = ng_spr_raw(ng);
                fprintf(stderr, "SPRDBG l=%d si=%d n=%d tile=%d sy=%d code=%x attr=%x gfx=%x "
                    "bytes=%02x %02x %02x %02x %02x %02x %02x %02x raw=%02x %02x %02x %02x %02x %02x %02x %02x pens=%04x %04x %04x\n",
                    line, si, n, tile, sprite_y, (unsigned)code, attr, (unsigned)gfx,
                    gd[0],gd[1],gd[2],gd[3],gd[4],gd[5],gd[6],gd[7],
                    raw[(gfx>>1)&0x1FFFFFF],raw[(gfx>>1)+1&0x1FFFFFF],raw[(gfx>>1)+2&0x1FFFFFF],raw[(gfx>>1)+3&0x1FFFFFF],
                    raw[(gfx>>1)+4&0x1FFFFFF],raw[(gfx>>1)+5&0x1FFFFFF],raw[(gfx>>1)+6&0x1FFFFFF],raw[(gfx>>1)+7&0x1FFFFFF],
                    pens[1], pens[2], pens[3]);
            }

            uint16_t zt = zoom_x_tables[zoom_x];
            const uint8_t *g = spr8 + gfx;
            if (x <= 0x1F0) {                       /* MAME: no wrap-around */
                int dx = x;
                for (int px = 0; px < 16; px++) {
                    if (zt & 0x8000) {
                        if (dx < NG_W) {
                            uint8_t p = g[0];
                            if (p) fb[line*NG_W + dx] = cram24(pens[p]);
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
                                if (p) fb[line*NG_W + dx] = cram24(pens[p]);
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

    /* ---- fix layer, drawn LAST: its non-pen0 pixels overwrite sprites
       (MAME screen_update: draw_sprites then draw_fixed_layer) ---- */
    if (!envf_nofix())
    for (int ty = 0; ty < 28; ty++) {
        for (int tx = 0; tx < 40; tx++) {
            uint16_t e = vram[0x7000 + ty + 0x20*tx];
            int code = e & 0xFFF, pal = e >> 12;
            if (fix_banked) {
                if (fbtype == 1)
                    code += 0x1000 * (garouoffsets[(ty - 2) & 31] ^ 3);
                else
                    code += 0x1000 * (((vram[0x7500 + ((ty - 1) & 31) + 32*(tx/6)] >> (5 - (tx % 6))*2) & 3) ^ 3);
            }
            const uint16_t *pens = &cram[(pal<<4) | bank];
            for (int y = 0; y < 8; y++) {
                int dy = ty*8 + y;
                if (dy >= NG_H) continue;
                uint32_t base = ((code<<5) | y) & fix_mask;
                const uint8_t *g = fixed + base;
                uint32_t *row = &fb[dy*NG_W + tx*8];
                uint8_t d;
                d = g[0x10]; if (d&0x0F) row[0]=cram24(pens[d&0x0F]); if (d&0xF0) row[1]=cram24(pens[d>>4]);
                d = g[0x18]; if (d&0x0F) row[2]=cram24(pens[d&0x0F]); if (d&0xF0) row[3]=cram24(pens[d>>4]);
                d = g[0x00]; if (d&0x0F) row[4]=cram24(pens[d&0x0F]); if (d&0xF0) row[5]=cram24(pens[d>>4]);
                d = g[0x08]; if (d&0x0F) row[6]=cram24(pens[d&0x0F]); if (d&0xF0) row[7]=cram24(pens[d>>4]);
            }
        }
    }
}
