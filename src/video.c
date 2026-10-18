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
    /* bit15 = brightness-reduction ("dark") flag, NOT blank */
    uint32_t dark = (c >> 15) & 1;
    uint32_t r = c & 0x1F, g = (c >> 5) & 0x1F, b = (c >> 10) & 0x1F;
    r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
    if (dark) { r >>= 1; g >>= 1; b >>= 1; }
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

void ng_render(neogeo *ng, uint32_t *fb)
{
    uint16_t *vram = ng_vram(ng);
    uint16_t *cram = ng_cram(ng);
    uint32_t bank = ng_cram_bank(ng);
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

    memset(fb, 0, NG_W*NG_H*4);

    /* ---- fix layer ---- */
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

    if (envf_vdbg()) {
        uint16_t e0 = vram[0x7000]; int c0=e0&0xFFF, p0=e0>>12;
        const uint16_t *ps = &cram[(p0<<4)|bank];
        fprintf(stderr, "VDBG e0=%04x code=%x pal=%x bank=%x pens[3]=%04x c24=%08x base=%x g10=%02x\n",
            e0, c0, p0, bank, ps[3], (unsigned)cram24(ps[3]), (c0<<5)&fix_mask, fixed[((c0<<5)&fix_mask)+0x10]);
        fprintf(stderr, "VDBG fb0=%08x active?\n", (unsigned)fb[0]);
    }
    if (!spr8) return;

    /* ---- sprites, per scanline ---- */
    for (int line = 0; line < NG_H; line++) {
        uint16_t *list = &vram[0x8600 | (line & 1) ? 0 : 0x80];  /* parity lists */
        (void)list;
        int x = 0, zoom_x = 0, y = 0, rows = 0;
        int count = 0;
        uint16_t active[MAX_SPRITES_PER_LINE];

        /* parse_sprites */
        for (int n = 0; n < MAX_SPRITES_PER_SCREEN && count < MAX_SPRITES_PER_LINE; n++) {
            uint16_t yc = vram[0x8200 | n];
            if (!(yc & 0x40)) {                     /* new block */
                y = 0x200 - (yc >> 7);
                rows = yc & 0x3F;
            }
            if (rows == 0) continue;
            int sl = (line - y) & 0x1FF;
            if (sl >= rows * 16) continue;          /* sprite_on_scanline */
            active[count++] = n;
        }
        if (envf_vdbg() && line == 0) {
            fprintf(stderr, "VDBG line0 active=%d ytab[0..3]=%04x %04x %04x %04x\n", count,
                vram[0x8200], vram[0x8201], vram[0x8202], vram[0x8203]);
            for (int z=0; z<count && z<6; z++) {
                int n=active[z];
                fprintf(stderr, "  act[%d] n=%d yc=%04x zc=%04x xc=%04x\n", z, n,
                    vram[0x8200|n], vram[0x8000|n], vram[0x8400|n]);
            }
        }

        /* draw_sprites */
        x = 0; zoom_x = 0;
        for (int i = 0; i < count; i++) {
            int n = active[i];
            uint16_t yc = vram[0x8200 | n];
            uint16_t zc = vram[0x8000 | n];
            if (yc & 0x40) {                        /* chained */
                x = (x + zoom_x + 1) & 0x1FF;
                zoom_x = (zc >> 8) & 0x0F;
            } else {
                y = 0x200 - (yc >> 7);
                x = vram[0x8400 | n] >> 7;
                zoom_x = (zc >> 8) & 0x0F;
            }
            int zoom_y = zc & 0xFF;
            rows = yc & 0x3F;
            if (x >= 0x140 && x <= 0x1F0) continue;

            int sprite_line = (line - y) & 0x1FF;
            int zoom_line = sprite_line & 0xFF;
            int invert = (sprite_line >> 8) & 1;
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

            uint16_t zt = zoom_x_tables[zoom_x];
            int dx = x;
            const uint8_t *g = spr8 + gfx;
            for (int px = 0; px < 16; px++) {
                if (zt & 0x8000) {
                    if (dx >= 0x200) dx -= 0x200;    /* wrap */
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
        }
    }
}
