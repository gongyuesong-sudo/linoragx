/* YM2610 FM + ADPCM-A/B via ymfm (MAME's FM synth core, fm.c successor).
 * YM2151 SSG stays on the compact square-wave model: ymfm has no YM2151
 * (its ssg_engine is the 16-register YM2149 map, YM2151 uses 128 regs).
 * External C interface unchanged; timer/IRQ/status handshake semantics are
 * preserved because the sm1 sound driver polls them. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include "ymfm_opn.h"

extern "C" {
typedef struct ym2610 ym2610;
ym2610 *ym_create(void);
void ym_destroy(ym2610 *y);
void ym_reset(ym2610 *y);
void ym_write(ym2610 *y, int chip, uint8_t reg, uint8_t val);
uint8_t ym_read(ym2610 *y, int chip);
int ym_irq_pending(ym2610 *y);
void ym_stream(ym2610 *y, int16_t *stereo, int frames);
void ym_advance(ym2610 *y, int zcycles);
void ym_set_rom(ym2610 *y, const uint8_t *rom, uint32_t size);
}

struct ym2610 {
    struct LinIntf : public ymfm::ymfm_interface {
        ym2610 *y = nullptr;
        virtual uint8_t ymfm_external_read(ymfm::access_class type, uint32_t address) override;
        virtual void ymfm_update_irq(bool asserted) override { (void)asserted; }
        void fire_timer(uint32_t tnum) { if (m_engine) m_engine->engine_timer_expired(tnum); }
    };

    LinIntf intf;
    ymfm::ym2610 chip;                       /* OPM FM (YM2151) + SSG + ADPCM-A/B */

    /* YM2610 timers (driver-init handshakes poll these) */
    int busy[2];
    unsigned long clk;             /* YM tick clock (for busy auto-clear) */
    unsigned long busy_at[2];      /* tick at which D7 busy self-clears (~25us) */
    int tcnt[2], tload[2], ten[2], tflag[2], tfrac;
    int tcntb[2], tloadb[2], tfracb[2], irqen[2];

    const uint8_t *rom = nullptr;            /* cart audio ROM (ADPCM samples) */
    uint32_t rom_size = 0;

    std::vector<ymfm::ym2610::output_data> obuf;
    std::vector<ymfm::ym2610::output_data> pend;   /* native samples not yet consumed */
    double frac = 0.0;                             /* fractional read position in pend */

    /* anti-alias filter: 4th-order Butterworth (2 cascaded biquads) LP @18k
     * in the native 223.7k domain, applied before decimation */
    struct Biquad {
        double b0=1,b1=0,b2=0,a1=0,a2=0, x1=0,x2=0,y1=0,y2=0;
        void design(double fs, double fc) {
            double w = 2.0*3.14159265358979*fc/fs, cw = cos(w), sw = sin(w), q = 0.707106781;
            double alpha = sw/(2.0*q), a0 = 1.0+alpha;
            b0=(1.0-cw)/2.0/a0; b1=(1.0-cw)/a0; b2=b0; a1=-2.0*cw/a0; a2=(1.0-alpha)/a0;
            x1=x2=y1=y2=0;
        }
        double step(double x) {
            double y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            x2=x1; x1=x; y2=y1; y1=y; return y;
        }
    };
    struct LP4 { Biquad s1, s2;
        void design(double fs, double fc) {
            /* Butterworth 4th order: two stages with Q 0.5412 / 1.3066 */
            s1.design(fs, fc); s2.design(fs, fc);
            double w = 2.0*3.14159265358979*fc/fs, cw = cos(w), sw = sin(w);
            double a0;
            double al1 = sw/(2.0*0.54119610), a = 1.0+al1;
            s1.b0=(1.0-cw)/2.0/a; s1.b1=(1.0-cw)/a; s1.b2=s1.b0; s1.a1=-2.0*cw/a; s1.a2=(1.0-al1)/a;
            double al2 = sw/(2.0*1.30656296); a = 1.0+al2;
            s2.b0=(1.0-cw)/2.0/a; s2.b1=(1.0-cw)/a; s2.b2=s2.b0; s2.a1=-2.0*cw/a; s2.a2=(1.0-al2)/a;
            (void)a0;
        }
        double step(double x) { return s2.step(s1.step(x)); }
    };
    LP4 bqL, bqR;

    ym2610() : intf(), chip(intf, 0x7f) { intf.y = this; bqL.design(3579545.0/16.0, 18000.0); bqR = bqL; }
};

uint8_t ym2610::LinIntf::ymfm_external_read(ymfm::access_class type, uint32_t address)
{
    (void)type;                              /* ADPCM-A and -B share the audio ROM */
    if (!y->rom || address >= y->rom_size) return 0;   /* MAME: no wrap, OOB reads 0 */
    return y->rom[address];
}

/* ---- lifecycle ---- */

ym2610 *ym_create(void) { return new ym2610(); }
void ym_destroy(ym2610 *y) { delete y; }

void ym_reset(ym2610 *y)
{
    y->chip.reset();
    y->pend.clear();
    y->frac = 0.0;
    memset(&y->busy, 0, (char *)(y->irqen + 2) - (char *)&y->busy);
}

void ym_set_rom(ym2610 *y, const uint8_t *rom, uint32_t size)
{
    y->rom = rom;
    y->rom_size = size;
}

/* ---- register writes ---- */

void ym_write(ym2610 *y, int chipno, uint8_t reg, uint8_t val)
{
    static int ymdbg = -1;
    if (ymdbg < 0) ymdbg = getenv("YMBDBG") ? 1 : 0;
    if (ymdbg) fprintf(stderr, "YM%c %02x=%02x\n", 'A'+chipno, reg, val);
    if (chipno == 2) return;                 /* legacy YM2151 mirror: bank A IS the OPM FM, ymfm handles it */
    int c = chipno ? 1 : 0;
    y->chip.write(c ? 2 : 0, reg);           /* bank A: offset 0/1, bank B: 2/3 */
    y->chip.write(c ? 3 : 1, val);
    y->busy[c] = 1;                          /* real YM2610 raises D7 after a write */
    y->busy_at[c] = y->clk + 200;            /* ...and clears it ~25us later on its own */

    if (reg == 0x24) { y->tload[c] = (y->tload[c] & 0xFF00) | val; y->tloadb[c] = (y->tloadb[c] & 0x300) | val; }
    if (reg == 0x25) { y->tload[c] = (y->tload[c] & 0x00FF) | ((val & 0x03) << 8); y->tloadb[c] = (y->tloadb[c] & 0x0FF) | ((val & 0x03) << 8); }
    if (reg == 0x27) {  /* timer/irq control: bit1=/LOAD B, bit2=/LOAD A, bit6=IRQB en, bit7=IRQA en */
        if (!(val & 0x02)) { y->tcntb[c] = y->tloadb[c] ? y->tloadb[c] : 1024; }
        if (!(val & 0x04)) { y->tcnt[c] = y->tload[c] ? y->tload[c] : 256; }
        y->ten[c] = ((val >> 1) & 3);
        y->irqen[c] = val & 0xC0;
        if (!(val & 0x02)) y->tflag[c] &= ~2;
        if (!(val & 0x04)) y->tflag[c] &= ~1;
    }
}

/* ---- timers (host-owned: ymfm leaves timing to the caller) ---- */

void ym_advance(ym2610 *y, int zcycles)
{
    /* timer A: 1 step / 192 clocks; timer B: 1 step / 80 clocks */
    y->clk += (unsigned long)zcycles;        /* busy D7 auto-clear clock */
    y->tfrac += zcycles;
    while (y->tfrac >= 192) {
        y->tfrac -= 192;
        for (int c = 0; c < 2; c++) {
            if (!(y->ten[c] & 2)) continue;
            if (--y->tcnt[c] <= 0) {
                y->tflag[c] |= 1;
                y->tcnt[c] = y->tload[c] ? y->tload[c] : 256;
                if (c == 0) y->intf.fire_timer(0);
            }
        }
    }
    for (int c = 0; c < 2; c++) {
        if (!(y->ten[c] & 1)) continue;
        y->tfracb[c] += zcycles;
        while (y->tfracb[c] >= 80) {
            y->tfracb[c] -= 80;
            if (--y->tcntb[c] <= 0) {
                y->tflag[c] |= 2;
                y->tcntb[c] = y->tloadb[c] ? y->tloadb[c] : 1024;
                if (c == 0) y->intf.fire_timer(1);
            }
        }
    }
}

int ym_irq_pending(ym2610 *y)
{
    /* YM2610 reg 0x27: IRQA/IRQB enable bits are ACTIVE LOW (0 = enable).
     * Timer A -> IRQA (bit6 of 0x27), Timer B -> IRQB (bit7). Getting this
     * inverted starves the sm1 music sequencer: it is timer-IRQ driven. */
    if ((y->tflag[0] & 1) && !(y->irqen[0] & 0x40)) return 1;
    if ((y->tflag[1] & 1) && !(y->irqen[1] & 0x40)) return 1;
    if ((y->tflag[0] & 2) && !(y->irqen[0] & 0x80)) return 1;
    if ((y->tflag[1] & 2) && !(y->irqen[1] & 0x80)) return 1;
    return 0;
}

uint8_t ym_read(ym2610 *y, int chipno)
{
    int c = chipno ? 1 : 0;
    uint8_t st = (uint8_t)(y->tflag[c] & 3);
    if (y->busy[c] && y->clk >= y->busy_at[c]) y->busy[c] = 0;  /* D7 self-clears with time */
    if (y->busy[c]) { st |= 0x80; y->busy[c] = 0; }
    y->tflag[c] = 0;
    return st;
}

/* ---- sound generation ---- */

/* ym2610 at OPN_FIDELITY_MAX generates at clock/16; output is linearly
 * resampled to 44.1k. All sound sources (OPM FM / SSG / ADPCM-A/B) live
 * inside the ymfm chip — no external generators. */
static const double YM_CLOCK = 3579545.0;
static const double NAT_RATE = YM_CLOCK / 16.0;
static const double OUT_RATE = 44100.0;

void ym_stream(ym2610 *y, int16_t *stereo, int frames)
{
    memset(stereo, 0, frames * 2 * sizeof(int16_t));
    if (frames <= 0) return;

    /* Continuous resampling: the fractional read position carries across
     * frames (a per-frame reset makes a waveform discontinuity every frame
     * = audible 59Hz clicking). pend holds unconsumed native samples. */
    const double STEP = NAT_RATE / OUT_RATE;
    int need = (int)(y->frac + frames * STEP) + 2 - (int)y->pend.size();
    if (need < 0) need = 0;
    y->obuf.resize(need);
    y->chip.generate(y->obuf.data(), need);
    for (int i = 0; i < need; i++) {        /* anti-alias before decimation */
        y->obuf[i].data[0] = (int32_t)y->bqL.step(y->obuf[i].data[0]);
        y->obuf[i].data[1] = (int32_t)y->bqR.step(y->obuf[i].data[1]);
        y->pend.push_back(y->obuf[i]);
    }

    { static int ysd = -1; static long ysc = 0;
      if (ysd < 0) ysd = getenv("YMSTREAM") ? 1 : 0;
      if (ysd && (ysc++ % 300) == 0)
          fprintf(stderr, "[yms] need=%d gen0=%d,%d pend=%d\n",
                  need, need ? (int)y->obuf[0].data[0] : 0, need ? (int)y->obuf[0].data[1] : 0,
                  (int)y->pend.size()); }

    double pos = y->frac;
    for (int i = 0; i < frames; i++, pos += STEP) {
        int j = (int)pos;
        if (j + 1 >= (int)y->pend.size()) j = (int)y->pend.size() - 2;
        double f = pos - j;
        int32_t l0 = y->pend[j].data[0];
        int32_t r0 = y->pend[j].data[1];
        int32_t l1 = y->pend[j+1].data[0];
        int32_t r1 = y->pend[j+1].data[1];
        int32_t L = (int32_t)(l0 * (1.0 - f) + l1 * f);
        int32_t R = (int32_t)(r0 * (1.0 - f) + r1 * f);
        if (L > 32767) L = 32767; if (L < -32768) L = -32768;
        if (R > 32767) R = 32767; if (R < -32768) R = -32768;
        stereo[i*2]   = (int16_t)L;
        stereo[i*2+1] = (int16_t)R;
    }
    int cons = (int)pos;
    y->pend.erase(y->pend.begin(), y->pend.begin() + cons);
    y->frac = pos - cons;
}
