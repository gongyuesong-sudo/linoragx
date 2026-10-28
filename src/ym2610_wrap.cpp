#include <time.h>
/* YM2610 FM + ADPCM-A/B via ymfm (MAME's FM synth core, fm.c successor).
 * YM2151 SSG stays inside ymfm's OPM: ymfm has no YM2151 (its ssg_engine is
 * the 16-register YM2149 map, YM2151 uses 128 regs).
 * External C interface unchanged; timer/IRQ/status handshake semantics are
 * preserved because the sm1 sound driver polls them.
 *
 * MAME-parity audio rework: the chip runs at the real NEOGEO_YM2610_CLOCK
 * (master/3 = 8 MHz -> native rate clock/16 = 500 kHz), its three output
 * streams are generated separately and mixed by audio_engine.cpp with
 * MAME's speaker routing/volumes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include "ymfm_opn.h"

/* MAME-style stream mixer (src/audio_engine.cpp) */
extern "C" {
typedef struct audio_engine audio_engine;
audio_engine *ae_create(double in_rate, double out_rate);
void ae_destroy(audio_engine *);
void ae_reset(audio_engine *);
void ae_feed(audio_engine *, const int32_t *fmL, const int32_t *fmR,
             const int32_t *adpcmA, const int32_t *adpcmB, int n);
int  ae_avail(audio_engine *);
int  ae_pull(audio_engine *, int16_t *stereo, int frames);
}

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
    int tpend[2];   /* clk of last timer overflow: keeps the IRQ latched so a
                       driver that status-polls faster than our chunk-granular
                       pending check cannot swallow the pulse (mslug m1 heartbeat
                       polls bank-A status ~10x/frame and cleared tflag before
                       the Z80 INT was ever raised). Cleared by the status read
                       (ack) or once the window (~1 frame) expires. */
    int tcntb[2], tloadb[2], tfracb[2], irqen[2];

    const uint8_t *rom = nullptr;            /* cart audio ROM (ADPCM-A samples) */
    uint32_t rom_size = 0;
    const uint8_t *b_rom = nullptr;          /* m1 program ROM (ADPCM-B samples, Z80 space) */
    uint32_t b_rom_size = 0;

    audio_engine *ae;                        /* MAME-style stream mixer */
    std::vector<ymfm::ym2610::output_data> obuf_fm, obuf_a, obuf_b;
    std::vector<int32_t> tL, tR, tA, tB;     /* deinterleave staging */

    ym2610() : intf(), chip(intf, 0x7f) {
        intf.y = this;
        ae = ae_create(500000.0, 48000.0);   /* clock/16 native -> 48 kHz out */
    }
};

uint8_t ym2610::LinIntf::ymfm_external_read(ymfm::access_class type, uint32_t address)
{
    static int adb = -1;
    if (adb < 0) adb = getenv("ADPCMDBG") ? 1 : 0;
    if (adb) {
        extern int g_ym_frame;
        static uint32_t maxaddr[2] = {0,0}; static long oob[2] = {0,0}, tot[2] = {0,0};
        static long lastprint = 0;
        int t = (type == ymfm::ACCESS_ADPCM_B) ? 1 : 0;
        tot[t]++;
        uint32_t rsz = (t == 1) ? y->b_rom_size : y->rom_size;
        if (address >= rsz) oob[t]++;
        if (address > maxaddr[t]) maxaddr[t] = address;
        if (g_ym_frame != lastprint) {   /* once per frame batch */
            lastprint = g_ym_frame;
            static long fno = 0;
            if ((fno++ % 60) == 0)
                fprintf(stderr, "[adpcm] f=%d A: max=%06x oob=%ld/%ld | B: max=%06x oob=%ld/%ld | romA=%06x romB=%06x\n",
                        g_ym_frame, maxaddr[0], oob[0], tot[0], maxaddr[1], oob[1], tot[1], y->rom_size, y->b_rom_size);
        }
    }
    /* MAME parity (ymopn.cpp ym2610_device_base): ADPCM-A and ADPCM-B read the
     * SAME cart sample region (ymsnd:adpcma = v1..v4 linear) unless the set
     * defines a separate adpcmb region; uninstalled/OOB addresses read 0, no
     * wrap. The BIOS defines no sample region at all -> BIOS ADPCM reads 0. */
    if (type != ymfm::ACCESS_ADPCM_A && type != ymfm::ACCESS_ADPCM_B) return 0;
    /* LINORAGX FIX: sets without a dedicated ADPCM-B region (no ym_set_rom_b
     * call -> b_rom_size==0) must share the ADPCM-A sample region, per the
     * MAME-parity note above; previously every B read was OOB -> silent.
     * ADBSHARE=0 disables the fallback (SMA self-decrypt carts like mslug3
     * sample their audiocpu program space through ADPCM-B reads and expect
     * the old zero-fill). */
    static int adbs = -1;
    if (adbs < 0) adbs = getenv("ADBSHARE") ? atoi(getenv("ADBSHARE")) : 1;
    /* Priority: game sample region (y->rom, installed at m1 boot) wins; the
     * BIOS sm1+sp1v1 concat (b_rom) serves only while no game samples are
     * installed. Reversed priority starved SMA self-decrypt carts (mslug3)
     * whose audiocpu program reads its own sample space via ADPCM-B. */
    const uint8_t *src = y->rom;
    uint32_t sz = y->rom_size;
    if ((!src || !sz) && type == ymfm::ACCESS_ADPCM_B && adbs && y->b_rom && y->b_rom_size) {
        src = y->b_rom; sz = y->b_rom_size;
    }
    if (!src || address >= sz) return 0;   /* MAME: no wrap, OOB reads 0 */
    return src[address];
}

/* ---- lifecycle ---- */

ym2610 *ym_create(void) { return new ym2610(); }
void ym_destroy(ym2610 *y) { ae_destroy(y->ae); delete y; }

void ym_reset(ym2610 *y)
{
    y->chip.reset();
    ae_reset(y->ae);
    memset(&y->busy, 0, (char *)(y->irqen + 2) - (char *)&y->busy);
}

void ym_set_rom(ym2610 *y, const uint8_t *rom, uint32_t size)
{
    y->rom = rom;
    y->rom_size = size;
}

extern "C" void ym_set_rom_b(ym2610 *y, const uint8_t *rom, uint32_t size)
{
    y->b_rom = rom;
    y->b_rom_size = size;
}

/* ---- register writes ---- */

void ym_write(ym2610 *y, int chipno, uint8_t reg, uint8_t val)
{
    static int ymdbg = -1;
    extern int g_ym_frame;
    if (ymdbg < 0) ymdbg = getenv("YMBDBG") ? 1 : 0;
    if (ymdbg) fprintf(stderr, "YM%c f=%d %02x=%02x\n", 'A'+chipno, g_ym_frame, reg, val);
    if (chipno == 2) return;                 /* legacy YM2151 mirror: bank A IS the OPM FM, ymfm handles it */
    int c = chipno ? 1 : 0;
    /* MAME ym2610 bank semantics (ymopn.cpp): bank A (0x2A0000/1) is the pure
       OPM FM register file; bank B (0x2A0002/3) is SSG(00-0F)/ADPCM-B(10-1B)/
       EOS(1C)/ADPCM-A(20-3F)/test+IRQ-mask(80+).  ymfm's lower port mixes
       SSG+ADPCM-B+FM in one address space, so route explicitly: FM regs only
       reach ymfm via the lower port at >=0x1D; bank B SSG/ADPCM-B also use the
       lower port (their 00-1C aliases), ADPCM-A the upper port (0x100+ alias).
       Previous direct bank->port mapping sent bank A 00-1C to SSG/ADPCM-B and
       bank B 00-1C to ADPCM-A: the BIOS SSG test beeps (00/01 tone writes)
       never reached the SSG engine (FM+SSG stream identically zero). */
    if (chipno == 0) {
        if (reg >= 0x1d) { y->chip.write(0, reg); y->chip.write(1, val); }
        else if (reg <= 0x0d) {
            /* ymfm's lower port maps 00-0D to the SSG engine.  Bank A low
               registers are the SSG tone/volume/enable file; forward them so
               the SSG is reachable (BIOS sound test, SSG-using titles).  The
               0E-1C gap is left dropped: those alias ADPCM-B START and the
               OPM timer loads the host timer model owns. */
            y->chip.write(0, reg); y->chip.write(1, val);
        }
        /* reg 0E-1C: OPM timer/noise — host-owned timers below.  Forwarding
           these to ymfm's lower port aliased them as ADPCM-B START writes
           (BIOS OPM timer loads polluted them). */
    } else if (reg <= 0x2f) {
        /* ADPCM-A register file (dump/total/test 00-02, levels 08-0D,
           start 10-1D, end 20-2D) — game m1 drivers write it here. */
        y->chip.write(2, reg); y->chip.write(3, val);
    } else if (reg >= 0x40 && reg <= 0x4d) {
        /* YM2610 lower port decodes only A0-A4: bank-B writes 0x40-0x4D alias
           down to 0x00-0x0D = the SSG register file.  The sm1/BIOS SSG
           self-test writes 41-4E=7F through this alias; the old code dropped
           them (after an earlier attempt routed them to the FM upper port and
           muted FM).  Route the alias to ymfm's lower port SSG range instead. */
        uint8_t s = reg & 0x0f;
        if (s <= 0x0d) { y->chip.write(0, s); y->chip.write(1, val); }
    } else if (reg == 0xb5) {
        y->irqen[0] = (y->irqen[0] & 0x80) | (val & 0x40);   /* IRQA mask (0=enable) */
    } else if (reg == 0xb6) {
        y->irqen[1] = (y->irqen[1] & 0x40) | (val & 0x80);   /* IRQB mask */
    }
    /* bank B 40-7F are YM2610 TEST registers: the old mapping aliased them
       into ymfm's upper port at 0x140+ which routed straight into FM slot
       registers 40-7F (BIOS writes 41-4E=7F every self-test -> FM muted,
       the identically-zero FM stream).  Drop them. */
    y->busy[c] = 1;                          /* real YM2610 raises D7 after a write */
    y->busy_at[c] = y->clk + 200;            /* ...and clears it ~25us later on its own */

    if (c == 0) { /* timers belong to bank A only; bank B 24/25/27 are ADPCM-A regs */
    if (reg == 0x24) { y->tload[c] = (y->tload[c] & 0xFF00) | val; y->tloadb[c] = (y->tloadb[c] & 0x300) | val; }
    if (reg == 0x25) { y->tload[c] = (y->tload[c] & 0x00FF) | ((val & 0x03) << 8); y->tloadb[c] = (y->tloadb[c] & 0x0FF) | ((val & 0x03) << 8); }
    if (reg == 0x27) {  /* timer/irq control (LEGACY mapping - sm1 BIOS driver depends on it):
                          bit1=/LOAD B, bit2=/LOAD A-ish, bit6/7=IRQ enables.
                          Datasheet-strict remap (bit0/1=/LOAD, bit5/6=enable) was tried
                          and killed the sm1 one-shot tick (BIOS BGM silent) - the BIOS
                          driver toggles 0x1f/0x2f and relies on timer B running in BOTH
                          states. kof2000 m1 arms 0x35 (enable B + /LOAD B=0) which no
                          interpretation runs - m1 BGM tick remains an open question. */
        { static int r27 = -1;
          if (r27 < 0) r27 = getenv("R27DBG") ? 1 : 0;
          if (r27) { extern int g_ym_frame;
                     fprintf(stderr, "R27 bank%c f=%d val=%02x\n", 'A'+c, g_ym_frame, val); } }
        if (!(val & 0x02)) { y->tcntb[c] = y->tloadb[c] ? y->tloadb[c] : 1024; }
        if (!(val & 0x04)) { y->tcnt[c] = y->tload[c] ? y->tload[c] : 256; }
        y->ten[c] = ((val >> 1) & 3);
        y->irqen[c] = val & 0xC0;
        if (!(val & 0x02)) y->tflag[c] &= ~2;
        if (!(val & 0x04)) y->tflag[c] &= ~1;
    }
    }
}

/* ---- timers (host-owned: ymfm leaves timing to the caller) ---- */

void ym_advance(ym2610 *y, int zcycles)
{
    /* timer A: 1 step / 192 clocks; timer B: 1 step / 80 clocks */
    y->clk += (unsigned long)zcycles;        /* busy D7 auto-clear clock */
    { static int tp = -2;
      if (tp == -2) tp = getenv("TIMPROBE") ? 1 : 0;
      if (tp) { static unsigned long n = 0;
                if ((n++ % 12000) == 0)
                    fprintf(stderr, "TIM ten=%d tload=%d tcnt=%d tflag=%d irqen=%02x clk=%lu\n",
                            y->ten[0], y->tload[0], y->tcnt[0], y->tflag[0], y->irqen[0], y->clk); } }
    y->tfrac += zcycles;
    while (y->tfrac >= 192) {
        y->tfrac -= 192;
        for (int c = 0; c < 2; c++) {
            if (!(y->ten[c] & 2)) continue;
            if (--y->tcnt[c] <= 0) {
                y->tflag[c] |= 1;
                extern long g_tflagA_sets; g_tflagA_sets++;
                y->tpend[c] = y->clk ? (int)y->clk : 1;
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
                y->tpend[c] = y->clk ? (int)y->clk : 1;
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
     * inverted starves the sm1 music sequencer: it is timer-IRQ driven.
     * tpend[] latches the pulse for ~1 frame so a fast status-polling driver
     * cannot clear tflag before the chunk-granular check ever raises INT. */
    const int window = 30000;    /* ~3.75 YM ms: long enough to bridge one
                                    chunk gap, short enough not to storm */
    int l0 = (y->tflag[0] & 1) || (y->tpend[0] && (int)y->clk - y->tpend[0] < window);
    int l0b = (y->tflag[0] & 2) || (y->tpend[0] && (int)y->clk - y->tpend[0] < window);
    int l1 = (y->tflag[1] & 1) || (y->tpend[1] && (int)y->clk - y->tpend[1] < window);
    int l1b = (y->tflag[1] & 2) || (y->tpend[1] && (int)y->clk - y->tpend[1] < window);
    if (l0 && !(y->irqen[0] & 0x40)) return 1;
    if (l1 && !(y->irqen[1] & 0x40)) return 1;
    if (l0b && !(y->irqen[0] & 0x80)) return 1;
    if (l1b && !(y->irqen[1] & 0x80)) return 1;
    return 0;
}

uint8_t ym_read(ym2610 *y, int chipno)
{
    int c = chipno ? 1 : 0;
    uint8_t st = (uint8_t)(y->tflag[c] & 3);
    if (y->busy[c] && y->clk >= y->busy_at[c]) y->busy[c] = 0;  /* D7 self-clears with time */
    if (y->busy[c]) st |= 0x80;   /* NOT read-cleared: BUSY is a hardware timing
                                     window, not a flag. mslug-family m1 heartbeats
                                     poll status constantly; consuming D7 on read
                                     starved the rst-8 BUSY wait in the IRQ handler. */
    /* YM2610 status D6 = ADPCM-B sample-finished flag (EOS), matching ymfm's
       own ym2608 read_status (live_eos). The BGM pump polls for D6=1 to know
       the current segment ended; a PLAYING-semantics bit would be inverted and
       make the driver punch each segment short (logo sound played too fast). */
    if (chipno == 1 && (y->chip.adpcm_b_status() & ymfm::adpcm_b_channel::STATUS_EOS)) { st |= 0x40; y->chip.adpcm_b_clear_eos(); }
    y->tflag[c] = 0;
    /* Status read ACKs the timer IRQ: clear the tpend latch together with
       tflag. With instruction-level delivery (linoragx_z80_step) the INT is
       raised at the next Z80 instruction after tflag sets, so a heartbeat
       poll can no longer swallow it — leaving tpend alive past the ACK just
       re-storms the line (handler re-entered with no flag to service). */
    y->tpend[c] = 0;
    return st;
}

/* ---- sound generation ---- */

/* ym2610 at OPN_FIDELITY_MAX generates at clock/16; the three split streams
 * are handed to the MAME-style mixer which band-limits and resamples each
 * stream to the output rate and applies the NeoGeo speaker routing.
 * All sound sources (OPM FM / SSG / ADPCM-A/B) live inside the ymfm chip. */
static const double YM_CLOCK = 8000000.0;   /* MAME: NEOGEO_YM2610_CLOCK = master/3 */
static const double NAT_RATE = YM_CLOCK / 16.0;   /* 500 kHz native */
static const double OUT_RATE = 48000.0;           /* MAME default output rate */

/* global frame counter for debug prints (advanced by ym_stream) */
int g_ym_frame = 0;
long g_tflagA_sets = 0;   /* diagnostic: count of timer-A flag assertions */

void ym_stream(ym2610 *y, int16_t *stereo, int frames)
{
    g_ym_frame++;
    static int ytm = -2; static long ytc = 0; static double ytm_acc = 0;
    struct timespec yts0, yts1;
    if (ytm == -2) ytm = getenv("AUDDBG") ? 1 : 0;
    if (ytm) clock_gettime(CLOCK_MONOTONIC, &yts0);
    memset(stereo, 0, frames * 2 * sizeof(int16_t));
    if (frames <= 0) return;

    /* generate exactly what this frame needs (plus kernel overlap) */
    const double STEP = NAT_RATE / OUT_RATE;
    int need = (int)(frames * STEP) + 96 - ae_avail(y->ae);
    if (need < 0) need = 0;
    y->obuf_fm.resize(need); y->obuf_a.resize(need); y->obuf_b.resize(need);
    y->chip.generate_split(y->obuf_fm.data(), y->obuf_a.data(), y->obuf_b.data(), need);

    y->tL.resize(need); y->tR.resize(need); y->tA.resize(need); y->tB.resize(need);
    for (int i = 0; i < need; i++) {
        y->tL[i] = y->obuf_fm[i].data[0];
        y->tR[i] = y->obuf_fm[i].data[1];
        y->tA[i] = y->obuf_a[i].data[0];      /* ADPCM streams are mono */
        y->tB[i] = y->obuf_b[i].data[1];
    }
    ae_feed(y->ae, y->tL.data(), y->tR.data(), y->tA.data(), y->tB.data(), need);
    ae_pull(y->ae, stereo, frames);

    { static int ysd = -1; static long ysc = 0;
      if (ysd < 0) ysd = getenv("YMSTREAM") ? 1 : 0;
      if (ysd && (ysc++ % 60) == 0) {
          int mx = 0;
          for (int i = 0; i < need; i++) { int v = y->tL[i] < 0 ? -y->tL[i] : y->tL[i]; if (v > mx) mx = v; }
          int ma = 0, mb = 0;
          for (int i = 0; i < need; i++) { int v = y->tA[i] < 0 ? -y->tA[i] : y->tA[i]; if (v > ma) ma = v;
                                           v = y->tB[i] < 0 ? -y->tB[i] : y->tB[i]; if (v > mb) mb = v; }
          fprintf(stderr, "[yms] need=%d frames=%d fm=%d aa=%d ab=%d\n", need, frames, mx, ma, mb); } }
    if (ytm) { clock_gettime(CLOCK_MONOTONIC, &yts1);
        ytm_acc += (yts1.tv_sec - yts0.tv_sec) * 1e6 + (yts1.tv_nsec - yts0.tv_nsec) / 1e3;
        if ((ytc++ % 300) == 0) { fprintf(stderr, "[ymtm] avg=%.0fus last=%.0fus frames=%d\n", ytm_acc / 300.0,
            (yts1.tv_sec - yts0.tv_sec) * 1e6 + (yts1.tv_nsec - yts0.tv_nsec) / 1e3, frames); ytm_acc = 0; } }
}
