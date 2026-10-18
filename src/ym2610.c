/* YM2610 (YM2151+YM3012 combo) — compact FM approximation.
 * 6 FM channels x 4 operators, phase/inc/envelope; SSG 3ch square.
 * Not cycle-accurate; aims at recognizable sound. Rate: 49716 Hz. */
#include "linoragx.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FM_RATE 49716.0

typedef struct { uint8_t *regs; } chip;

struct ym2610 {
    uint8_t r26[2][0x100];     /* 2610 A/B banks */
    uint8_t r21[0x100];        /* 2151 (SSG) */
    double  phase[2][24][4];
    double  env[2][24][4];
    int     sl[2][24][4];
    double  acc;
    double  ssg_phase[3];
    int     ssg_out[3];
    int16_t last[2];
    /* YM2610 timers (driver-init handshakes poll these) */
    int      busy[2];                   /* YM D7 one-shot: set on write, cleared by status read */
    int      tcnt[2], tload[2], ten[2], tflag[2], tfrac;
    int      tcntb[2], tloadb[2], tfracb[2], irqen[2];
};

static const double mul_tab[16] = {0.5,1,2,3,4,5,6,7,8,9,10,10,12,12,15,15};
static const int tl2amp[128] = { /* linearized attenuation -> amplitude 0..8192 */
    8192,7318,6538,5837,5206,4638,4130,3677,3274,2917,2599,2316,2064,1839,1638,1459,
    1300,1157,1031,919,819,729,650,579,516,460,410,365,325,290,258,230,
    205,183,163,145,129,115,103,91,81,73,65,58,52,46,41,36,
    33,29,26,23,21,18,16,15,13,11,10,9,8,7,6,5,
    5,4,4,3,3,3,2,2,2,2,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0 };

ym2610 *ym_create(void) { return calloc(1, sizeof(ym2610)); }
void ym_destroy(ym2610 *y) { free(y); }
void ym_reset(ym2610 *y) { memset(y, 0, sizeof *y); }

void ym_write(ym2610 *y, int chip, uint8_t reg, uint8_t val)
{
    if (chip == 2) { /* YM2151 SSG: real chip ignores writes to reg > 0x7F */
        if (reg <= 0x7F) y->r21[reg] = val;
        return;
    }
    int c = chip ? 1 : 0;
    y->r26[c][reg] = val;
    y->busy[c] = 1;                     /* real YM2610 raises D7 after accepting a write */
    if (reg == 0x24) { y->tload[c] = (y->tload[c] & 0xFF00) | val; y->tloadb[c] = (y->tloadb[c] & 0x300) | val; }
    if (reg == 0x25) { y->tload[c] = (y->tload[c] & 0x00FF) | ((val & 0x03) << 8); y->tloadb[c] = (y->tloadb[c] & 0x0FF) | ((val & 0x03) << 8); }
    if (reg == 0x27) {  /* timer/irq control: bit0=/LOAD C, bit1=/LOAD B, bit2=/LOAD A (low=load), bit6=IRQB en, bit7=IRQA en */
        if (!(val & 0x02)) { y->tcntb[c] = y->tloadb[c] ? y->tloadb[c] : 1024; }
        if (!(val & 0x04)) { y->tcnt[c] = y->tload[c] ? y->tload[c] : 256; }
        y->ten[c] = ((val >> 1) & 3);   /* bit1=timer B en, bit2=timer A en */
        y->irqen[c] = val & 0xC0;
        if (!(val & 0x02)) y->tflag[c] &= ~2;
        if (!(val & 0x04)) y->tflag[c] &= ~1;
    }
    if (reg == 0x28 || reg == 0x29) { /* key on/off */
        int ch = val & 3;
        for (int op = 0; op < 4; op++) {
            if (val & 0x10) y->env[c][ch][op] = 0;          /* key on: attack */
            if (!(val & 0x10)) y->env[c][ch][op] = 1e9;     /* key off -> silent */
        }
    }
}

void ym_advance(ym2610 *y, int zcycles)
{
    /* YM2610 timer A: 1 step / 192 clocks; timer B: 1 step / 80*80/6? -> 6 steps/80 clocks */
    y->tfrac += zcycles;
    while (y->tfrac >= 192) {
        y->tfrac -= 192;
        for (int c = 0; c < 2; c++) {
            if (!(y->ten[c] & 2)) continue;              /* timer A enabled? */
            if (--y->tcnt[c] <= 0) {
                y->tflag[c] |= 1;
                y->tcnt[c] = y->tload[c] ? y->tload[c] : 256;
            }
        }
    }
    for (int c = 0; c < 2; c++) {
        if (!(y->ten[c] & 1)) continue;                  /* timer B enabled? */
        y->tfracb[c] += zcycles;
        while (y->tfracb[c] >= 80) {
            y->tfracb[c] -= 80;
            if (--y->tcntb[c] <= 0) {
                y->tflag[c] |= 2;
                y->tcntb[c] = y->tloadb[c] ? y->tloadb[c] : 1024;
            }
        }
    }
}

int ym_irq_pending(ym2610 *y)
{
    if ((y->tflag[0] & 1) && (y->irqen[0] & 0x80)) return 1;   /* A IRQA */
    if ((y->tflag[1] & 1) && (y->irqen[1] & 0x80)) return 1;   /* B IRQA */
    if ((y->tflag[0] & 2) && (y->irqen[0] & 0x40)) return 1;   /* A IRQB */
    if ((y->tflag[1] & 2) && (y->irqen[1] & 0x40)) return 1;   /* B IRQB */
    return 0;
}

uint8_t ym_read(ym2610 *y, int chip)
{
    int c = chip ? 1 : 0;
    uint8_t st = (uint8_t)(y->tflag[c] & 3);
    if (y->busy[c]) { st |= 0x80; y->busy[c] = 0; }   /* one-shot D7 handshake */
    y->tflag[c] = 0;                                  /* status read clears flags */
    return st;
}

/* produce n samples of chip c mixed into buf (already accumulated) */
static void fm_stream(ym2610 *y, int c, int16_t *buf, int n)
{
    uint8_t *R = y->r26[c];
    for (int i = 0; i < n; i++) {
        int mix = 0;
        for (int ch = 0; ch < 6; ch++) {
            int fb = (R[0xB6 + ch] >> 4) & 7;
            int alg = R[0xB0 + ch] & 7;
            double mod = 0;
            for (int op = 3; op >= 0; op--) {
                int slot = ch*4 + op;
                int fnum = ((R[0xA4 + ch] & 3) << 8) | R[0xA0 + ch];
                int blk  = (R[0xA4 + ch] >> 2) & 7;
                int reg  = R[0x60 + slot];           /* multiple + TL hi */
                double mul = mul_tab[reg >> 4];
                int tl = ((reg & 0x7C) << 1) | (R[0x80 + slot] >> 6);
                if (tl > 127) tl = 127;
                double freq = FM_RATE/144.0 * (fnum/1024.0) * pow(2.0, blk-1) * mul * (1+0.06155*(R[0xB0+ch]&1));
                y->phase[c][ch][op] += freq / FM_RATE;
                if (y->phase[c][ch][op] >= 1.0) y->phase[c][ch][op] -= 1.0;
                /* envelope: simple attack/decay toward TL */
                double ar = (R[0x80 + slot] >> 2) & 0x1F;
                double target = 1.0 - tl/127.0;
                if (y->env[c][ch][op] > 1e8) { /* off */ }
                else y->env[c][ch][op] += (target - y->env[c][ch][op]) * (0.0001 + ar*0.0006);
                double e = (y->env[c][ch][op] > 1e8) ? 0 : y->env[c][ch][op];
                double p = y->phase[c][ch][op] + mod * (1.0/6.28);
                double s = sin(6.283185 * p) * e * 8192.0;
                mod = (op == 0) ? 0 : s;
                if (op == 0 && fb) mod = s * (fb/8.0);
                if (alg == 7 || (alg & 1) == 0) { /* rough carrier sum */
                    if (op == 0 || op == 3) mix += (int)s;
                } else mix += (op == 0) ? (int)s : 0;
            }
        }
        /* SSG (YM2151) */
        for (int ch = 0; ch < 3; ch++) {
            int fn = ((y->r21[0x20 + ch*8] & 0x0F) << 6) | (y->r21[0x18 + ch*8] & 0x3F);
            if (!fn) continue;
            double f = 2048.0 * FM_RATE / (4096*32) * fn / 8.0;
            y->ssg_phase[ch] += f / FM_RATE;
            if (y->ssg_phase[ch] >= 1.0) { y->ssg_phase[ch] -= 1.0; y->ssg_out[ch] ^= 1; }
            int vol = y->r21[0x0E + ch*8] & 0x1F;
            if (y->ssg_out[ch]) mix += (8192 - vol*256) / 6;
        }
        /* noise channel of 2610 (ch5 special) — light white noise when enabled */
        if (R[0xB6] & 0x04) mix += (rand() & 255) - 128;
        int v = mix >> 4;
        if (v > 32767) v = 32767; if (v < -32768) v = -32768;
        buf[i*2]   += (int16_t)v;
        buf[i*2+1] += (int16_t)v;
    }
}

void ym_stream(ym2610 *y, int16_t *stereo, int frames)
{
    memset(stereo, 0, frames*2*sizeof(int16_t));
    fm_stream(y, 0, stereo, frames);
    fm_stream(y, 1, stereo, frames);
}
