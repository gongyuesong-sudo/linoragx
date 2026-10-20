/* YM2610 FM + ADPCM-A/B via ymfm (MAME's FM synth core, fm.c successor).
 * YM2151 SSG stays on the compact square-wave model: ymfm has no YM2151
 * (its ssg_engine is the 16-register YM2149 map, YM2151 uses 128 regs).
 * External C interface unchanged; timer/IRQ/status handshake semantics are
 * preserved because the sm1 sound driver polls them. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
    ymfm::ym2610 chip;                       /* 7 FM channels + ADPCM-A/B */

    /* YM2151 SSG (compact model, bipolar square) */
    uint8_t r21[0x100];
    double  ssg_phase[3];
    int     ssg_out[3];

    /* YM2610 timers (driver-init handshakes poll these) */
    int busy[2];
    int tcnt[2], tload[2], ten[2], tflag[2], tfrac;
    int tcntb[2], tloadb[2], tfracb[2], irqen[2];

    const uint8_t *rom = nullptr;            /* cart audio ROM (ADPCM samples) */
    uint32_t rom_size = 0;

    std::vector<ymfm::ym2610::output_data> obuf;

    ym2610() : intf(), chip(intf, 0x7f) { intf.y = this; }
};

uint8_t ym2610::LinIntf::ymfm_external_read(ymfm::access_class type, uint32_t address)
{
    (void)type;                              /* ADPCM-A and -B share the audio ROM */
    if (!y->rom || !y->rom_size) return 0;
    return y->rom[address % y->rom_size];
}

/* ---- lifecycle ---- */

ym2610 *ym_create(void) { return new ym2610(); }
void ym_destroy(ym2610 *y) { delete y; }

void ym_reset(ym2610 *y)
{
    y->chip.reset();
    memset(y->r21, 0, sizeof y->r21);
    memset(y->ssg_phase, 0, sizeof y->ssg_phase);
    memset(y->ssg_out, 0, sizeof y->ssg_out);
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
    if (chipno == 2) {                       /* YM2151 SSG: real chip ignores reg > 0x7F */
        if (reg <= 0x7F) y->r21[reg] = val;
        return;
    }
    int c = chipno ? 1 : 0;
    y->chip.write(c ? 2 : 0, reg);           /* bank A: offset 0/1, bank B: 2/3 */
    y->chip.write(c ? 3 : 1, val);
    y->busy[c] = 1;                          /* real YM2610 raises D7 after a write */

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
    if ((y->tflag[0] & 1) && (y->irqen[0] & 0x80)) return 1;
    if ((y->tflag[1] & 1) && (y->irqen[1] & 0x80)) return 1;
    if ((y->tflag[0] & 2) && (y->irqen[0] & 0x40)) return 1;
    if ((y->tflag[1] & 2) && (y->irqen[1] & 0x40)) return 1;
    return 0;
}

uint8_t ym_read(ym2610 *y, int chipno)
{
    int c = chipno ? 1 : 0;
    uint8_t st = (uint8_t)(y->tflag[c] & 3);
    if (y->busy[c]) { st |= 0x80; y->busy[c] = 0; }
    y->tflag[c] = 0;
    return st;
}

/* ---- sound generation ---- */

/* ym2610 at OPN_FIDELITY_MAX generates at clock/16; output is linearly
 * resampled to 44.1k. YM2151 SSG is generated directly at the output rate. */
static const double YM_CLOCK = 3579545.0;
static const double NAT_RATE = YM_CLOCK / 16.0;
static const double OUT_RATE = 44100.0;

static void ssg_stream(ym2610 *y, int32_t *buf, int n)
{
    for (int i = 0; i < n; i++) {
        int mix = 0;
        for (int ch = 0; ch < 3; ch++) {
            int fn = ((y->r21[0x20 + ch*8] & 0x0F) << 6) | (y->r21[0x18 + ch*8] & 0x3F);
            if (!fn) continue;
            double f = 2048.0 * OUT_RATE / (4096*32) * fn / 8.0;
            y->ssg_phase[ch] += f / OUT_RATE;
            if (y->ssg_phase[ch] >= 1.0) { y->ssg_phase[ch] -= 1.0; y->ssg_out[ch] ^= 1; }
            int vol = y->r21[0x0E + ch*8] & 0x1F;
            int amp = (8192 - vol*256) / 6;
            mix += y->ssg_out[ch] ? amp : -amp;   /* bipolar square: unipolar = DC = harsh buzz */
        }
        buf[i] += mix;
    }
}

void ym_stream(ym2610 *y, int16_t *stereo, int frames)
{
    memset(stereo, 0, frames * 2 * sizeof(int16_t));
    if (frames <= 0) return;

    int n = (int)(frames * (NAT_RATE / OUT_RATE)) + 2;
    y->obuf.resize(n);
    y->chip.generate(y->obuf.data(), n);

    std::vector<int32_t> mix(frames, 0);
    ssg_stream(y, mix.data(), frames);

    double step = (double)n / frames;
    double pos = 0.0;
    for (int i = 0; i < frames; i++, pos += step) {
        int j = (int)pos;
        if (j >= n - 1) j = n - 2;
        double f = pos - j;
        int32_t l0 = y->obuf[j].data[0] + mix[i];
        int32_t r0 = y->obuf[j].data[1] + mix[i];
        int32_t l1 = y->obuf[j+1].data[0] + mix[i];
        int32_t r1 = y->obuf[j+1].data[1] + mix[i];
        int32_t L = (int32_t)(l0 * (1.0 - f) + l1 * f);
        int32_t R = (int32_t)(r0 * (1.0 - f) + r1 * f);
        if (L > 32767) L = 32767; if (L < -32768) L = -32768;
        if (R > 32767) R = 32767; if (R < -32768) R = -32768;
        stereo[i*2]   = (int16_t)L;
        stereo[i*2+1] = (int16_t)R;
    }
}
