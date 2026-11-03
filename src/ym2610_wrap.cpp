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
uint8_t ym_read_data(ym2610 *y, int chip, uint8_t reg);
int ym_irq_pending(ym2610 *y);
void ym_stream(ym2610 *y, int16_t *stereo, int frames);
int ym_save_state(ym2610 *y, void *buf);
int ym_load_state(ym2610 *y, const void *buf);
void ym_advance(ym2610 *y, int zcycles);
void ym_set_rom(ym2610 *y, const uint8_t *rom, uint32_t size);
}

struct YMWrite { uint32_t step; uint8_t chip, reg, val; };

struct ym2610 {
    struct LinIntf : public ymfm::ymfm_interface {
        ym2610 *y = nullptr;
        virtual uint8_t ymfm_external_read(ymfm::access_class type, uint32_t address) override;
        /* GOLDPORT (task-265): MAME-parity host wiring. ymfm_mame.h schedules
           engine timers on device timers at the YM clock (4 MHz); we count the
           same domain in lin_tick() (fed from ym_advance). busy_end/is_busy map
           onto the core's own busy window (32*prescale clocks per write).
           update_irq records the LEVEL the core computes in
           engine_check_interrupts (status & irq_mask) - the exact signal MAME
           wires to the audiocpu IRQ line. All gated by y->gold (GOLDMODE=0
           restores the legacy hand-written layer for A/B). */
        virtual void ymfm_set_timer(uint32_t tnum, int32_t clocks) override;
        virtual void ymfm_set_busy_end(uint32_t clocks) override;
        virtual bool ymfm_is_busy() override;
        virtual void ymfm_update_irq(bool asserted) override;
        void fire_timer(uint32_t tnum) { if (m_engine) m_engine->engine_timer_expired(tnum); }
    };

    LinIntf intf;
    ymfm::ym2610 chip;                       /* OPM FM (YM2151) + SSG + ADPCM-A/B */

    /* GOLDPORT (task-265): 1 = ymfm-core gold model (engine timers, core busy
       window, level IRQ from engine_check_interrupts, status via
       read_status/read_status_hi). 0 = legacy hand-written layer. */
    int gold;
    int irq_level;

    /* YM2610 timers (driver-init handshakes poll these) */
    int busy[2];
    unsigned long clk;             /* YM tick clock (for busy auto-clear) */
    unsigned long busy_at[2];      /* tick at which D7 busy self-clears (~25us) */
    int tcnt[2], tload[2], ten[2], tflag[2], tfrac, irq_latch[2];
    int tpend[2];   /* clk of last timer overflow: keeps the IRQ latched so a
                       driver that status-polls faster than our chunk-granular
                       pending check cannot swallow the pulse (mslug m1 heartbeat
                       polls bank-A status ~10x/frame and cleared tflag before
                       the Z80 INT was ever raised). Cleared by the status read
                       (ack) or once the window (~1 frame) expires. */
    int tcntb[2], tloadb[2], tfracb[2], irqen[2];
    int r26seen[2]; /* driver explicitly wrote timer-B load (reg 0x26) at least
                       once. BRELOAD gates on this: samsho2/samsho3 arm timer B
                       (r26 x2) and need continuous re-arming to emulate the
                       golden driver's dense 0x27 re-arm; kof98/kof2000 never
                       touch 0x26 and their drivers branch on IN(04) bit1 —
                       reloading for them REGRESSED kof98 9→0 loud. Not in the
                       save blob: the driver re-writes 0x26 periodically. */
    /* YM2151 (OPM) native timer — samsho2 task-206: the SNK m1 FM task arms
       its tick through reg 0x1C (7-bit latch, bit7 = IRQ mask) and keeps it
       running; MAME ym2151 auto-reloads on every expiry (~64us steps), so the
       timer overflows ~2.2kHz and the Z80 status-read ACK paces the IRQ to
       ~one per frame (golden attract FM task = 64/s). lin had no OPM timer at
       all: the OPN-style timer A sat at tload=0 (73ms period) and the FM task
       starved at 2.5/s vs golden 64/s. */
    int opm_latch, opm_mask, opm_cnt, opm_frac;
    int opna_flag, opna_cnt; /* task-255 experiment (env OPNA=<ym_clocks_per_expiry>):
                       golden samsho2 IRQ main rate 0.7ms cannot come from OPM
                       (r1c mostly mask=0) — hypothesis: bank-B OPN timer A
                       free-running at load=0. Drives ONLY the IRQ line; never
                       sets IN(06) status bits (that regressed kof98). Read of
                       bank-B status ACKs it (level IRQ read-clear parity). */
    int opm_started; /* 1 once the driver has written reg 0x1C: before that the
                        latch is 0 and a free-running timer would storm status
                        bit0 at 15.6kHz, wrecking the BIOS self-test cadence
                        that OPN-style cards poll. */
    int opm_flag;   /* YM2151 timer status bit0. Set on overflow together with
                       the shared tflag bit0, cleared by the bank-A status
                       read — but unlike tflag it survives the IN(04) that the
                       IRQ handler itself performs, so the level IRQ stays
                       asserted across the whole handler run (samsho2 task-206
                       golden: IRQ cadence = overflow rate ~100/s, one accept
                       per expiry; a tflag-only model gets its pulse eaten by
                       the driver's high-rate status polling). */
    int irqmask[2];              /* reg 0x29 timer-IRQ enable (bit0=A,bit1=B);
                                    ymfm reset value is 0x03, NOT 0 (task-196) */
    int irqen5[2];               /* 1 while last 0x27 write had bit5 (YM2610 IRQ-EN) */
    int irq29[2];                /* 1 once reg 0x29 was ever written: switch from
                                    the once-per-frame crutch to ymfm level IRQ */
    /* NRXSTATE11: shadow of the ymfm register store (last pair per port) so a
       resumed run restores FM/SSG/ADPCM registers instead of a silent chip */
    uint8_t sh_lo[256], sh_hi[256]; int sh_lreg, sh_hreg;

    const uint8_t *rom = nullptr;            /* cart audio ROM (ADPCM-A samples) */
    uint32_t rom_size = 0;
    const uint8_t *b_rom = nullptr;          /* m1 program ROM (ADPCM-B samples, Z80 space) */
    uint32_t b_rom_size = 0;

    audio_engine *ae;                        /* MAME-style stream mixer */
    std::vector<ymfm::ym2610::output_data> obuf_fm, obuf_a, obuf_b;
    std::vector<YMWrite> wq;                /* YMQUEUE: pending register writes (frame-relative step) */
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

/* ymfm's SSG reset state is a full-scale DC rail: the per-channel tone/noise
   enable bits are active-low (all enabled at reset) and a tone period of 0
   pins each tone state high, so every channel outputs s_amplitudes[0] =
   16382 — a ~27600 DC sum on any card that never programs the SSG (~49 of
   208).  Silence it by writing the mixer register (0F: all enable_n set). */
static void ym_ssg_silence(ym2610 *y)
{
    y->chip.write(0, 0x0f); y->sh_lreg = 0x0f;
    y->chip.write(1, 0xff); y->sh_lo[0x0f] = 0xff;
}

/* GOLDPORT interface bodies (struct ym2610 complete here) */
/* GOLDPORT bisection sub-switches (task-266): each isolates one gold-model
   facet so a regression can be attributed without reverting the whole mode.
   Default 1 = full gold; set 0 to fall back to legacy for that facet. */
static int gold_sub(const char *env)
{
    static int cache[4]; static const char *keys[4] = {"GOLDTICK","GOLDRD","GOLDBUSY","GOLDIRQ"};
    for (int i = 0; i < 4; i++) {
        if (!strcmp(env, keys[i])) {
            if (cache[i] == 0) { const char *e = getenv(env); cache[i] = (e && atoi(e) == 0) ? -1 : 1; }
            return cache[i] == 1;
        }
    }
    return 1;
}
#define GOLD_TICK(y) ((y)->gold && gold_sub("GOLDTICK"))
#define GOLD_RD(y)   ((y)->gold && gold_sub("GOLDRD"))
#define GOLD_BUSY(y) ((y)->gold && gold_sub("GOLDBUSY"))
#define GOLD_IRQ(y)  ((y)->gold && gold_sub("GOLDIRQ"))
void ym2610::LinIntf::ymfm_set_timer(uint32_t tnum, int32_t clocks)
{
    { static int td = -2; if (td == -2) td = getenv("TMRDBG") ? 1 : 0;
      if (td) fprintf(stderr, "TMR t=%u clocks=%d\n", tnum, clocks); }
    if (y && y->gold && tnum < 2) y->chip.lin_set_timer((int)tnum, clocks);
}
int g_tickfire_dbg = 0;
extern int g_ym_frame;
void lin_tickfire_log(int t)
{
    fprintf(stderr, "TICKFIRE f=%d t=%d\n", g_ym_frame, t);
}
void ym2610::LinIntf::ymfm_set_busy_end(uint32_t clocks)
{
    if (y && y->gold) y->chip.lin_set_busy_end(clocks);
}
bool ym2610::LinIntf::ymfm_is_busy()
{
    return (y && GOLD_BUSY(y)) ? y->chip.lin_is_busy() : false;
}
void ym2610::LinIntf::ymfm_update_irq(bool asserted)
{
    if (y && y->gold)
    {
        y->irq_level = asserted ? 1 : 0;
        static int iq = -1;
        if (iq < 0) iq = getenv("IRQDBG") ? 1 : 0;
        if (iq) { extern int g_ym_frame;
                  fprintf(stderr, "IRQDBG f=%d a=%d st=%02x\n", g_ym_frame, (int)asserted,
                          y->chip.read_status()); }
    }
}

/* GOLDPORT: opt-in for now (GOLDMODE=1 enables the core gold model). The
   legacy hand-written layer stays default until the 208-card batch on
   GOLDMODE=1 shows zero regressions (kof98 attract went silent in the
   first spot-check: bank-A timer-B flag semantics differ from the MAME
   YM2151-on-bank-A reality; see task-265 checkpoints). */
static int gold_mode(void)
{
    static int g = -1;
    if (g < 0) { const char *e = getenv("GOLDMODE"); g = (e && atoi(e) == 0) ? 0 : 1; }
    return g;
}

ym2610 *ym_create(void)
{
    ym2610 *y = new ym2610();
    { extern int g_tickfire_dbg; g_tickfire_dbg = getenv("TICKFIRE") ? 1 : 0; }
    y->gold = gold_mode();
    y->chip.reset();   /* ymfm write paths read internal state; never touch the chip before it is reset */
    y->chip.lin_reset();
    if (y->gold) y->chip.lin_set_irq_mask(1);   /* timer A only, see ymfm_opn.h note */
    y->irqmask[0] = y->irqmask[1] = 3;   /* ymfm reset IRQ mask (task-196) */
    ym_ssg_silence(y);
    return y;
}
void ym_destroy(ym2610 *y) { ae_destroy(y->ae); delete y; }

void ym_reset(ym2610 *y)
{
    y->wq.clear();
    y->chip.reset();
    y->chip.lin_reset();
    if (y->gold) y->chip.lin_set_irq_mask(1);
    y->irq_level = 0;
    ym_ssg_silence(y);
    ae_reset(y->ae);
    memset(&y->busy, 0, (char *)(y->irqmask + 2) - (char *)&y->busy);
    /* ymfm ym2610 reset: m_irq_enable=0x03 -> IRQ mask = timer A|B from reset
       (task-196 golden kof10th proof: sm1/m1 timer IRQs fire with NO 0x29 write) */
    y->irqmask[0] = y->irqmask[1] = 3;
    y->irq29[0] = y->irq29[1] = 0;
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

/* ---- YM write queue (YMQUEUE=1): stamp every Z80 YM register write with the
 * Z80 instruction index within the frame, then generate audio in segments so
 * keyon/keyoff pairs that land in the SAME frame no longer cancel each other
 * (samsho2 battle SFX + FM notes were written and killed within one 66k-cycle
 * frame: batch generation at frame end saw only the final register state). */
extern "C" uint32_t g_z_step = 0;
extern "C" int ym_queue_on(void)
{
    static int q = -2;
    if (q == -2) { const char *e = getenv("YMQUEUE"); q = (e && e[0] == '0') ? 0 : 1; }
    return q;
}

static void ym_write_core(ym2610 *y, int chipno, uint8_t reg, uint8_t val)
{
    static int ymdbg = -1;
    extern int g_ym_frame;
    if (ymdbg < 0) ymdbg = getenv("YMBDBG") ? 1 : 0;
    if (ymdbg) fprintf(stderr, "YM%c f=%d %02x=%02x\n", 'A'+chipno, g_ym_frame, reg, val);
    { static int cw = -2; if (cw == -2) cw = getenv("COREWR") ? 1 : 0;
      if (cw) fprintf(stderr, "COREWR f=%d c%d r=%02x v=%02x\n", g_ym_frame, chipno, reg, val); }
    if (chipno == 2) return;                 /* legacy YM2151 mirror: bank A IS the OPM FM, ymfm handles it */
    int c = chipno ? 1 : 0;
    /* ymfm ym2610 is a unified 9-bit address space (vendor ymfm_opn.cpp
       write_data/write_data_hi): lower port = SSG(00-0D)/ADPCM-B(10-1B)/
       EOS(1C)/FM(1D+); upper port = ADPCM-A(00-2F)/FM(30-FF).  The Neo Geo
       wires bank A to the lower pair and bank B to the upper pair, so bank B
       forwards verbatim to the upper port.  Bank A keeps only FM (>=0x1D):
       its 00-1C writes are YM2151-legacy test/timer regs the sm1 BIOS pings
       every frame and must NOT reach SSG/ADPCM-B (DC rail). */
    if (chipno == 0) {
        if (reg >= 0x10 && reg <= 0x1b) {
            /* Bank A 10-1B = ADPCM-B registers, written DIRECTLY by the 68K
               (mslug4 attract BGM = ADPCM-B cart-sample playback, golden
               MAME LINORAGX_ADPCMB probe: reads from start<<8 for 31 s).
               Forward to ymfm lower port; ymfm's ym2610::write_data applies
               the real-chip `data|0x20 &~0x40` external-force on reg 10. */
            y->chip.write(0, reg); y->sh_lreg = reg; y->chip.write(1, val); y->sh_lo[reg] = val;
        } else if (reg == 0x1c) {
            /* 1C: EOS flag reset (ymfm lower port) */
            y->chip.write(0, reg); y->sh_lreg = reg; y->chip.write(1, val); y->sh_lo[reg] = val;
        } else if (reg >= 0x1d) {
            y->chip.write(0, reg); y->sh_lreg = reg; y->chip.write(1, val); y->sh_lo[reg] = val;
        }
        /* Bank A is the YM2151 OPM register file: 00-0F are test/timer/IO
           (08/09/0A = OPM test, IO port, timer load — the BIOS self-test
           writes 00 there every frame).  ymfm's ym2610 has NO OPM side, so
           an earlier fix forwarded 00-0D to the SSG "for BIOS beeps"; that
           instead set SSG volumes 08/09/0A to 00 = FULL SCALE with tone
           periods 0 = a full-length DC rail (zcmed<5 on ~49 cards).  Bank A
           00-0F stays dropped: ymfm has no OPM and these are OPM-only regs.
           BUT shadow-record into sh_lo[]: mslug5's cart m1 ADPCM-A player
           (0x0b87) uses SSG reg 7 as a GPIO read-modify-write handshake
           (OUT(4)=7; IN(5); modify; OUT(4)=7,OUT(5)=v).  With IN(5) pinned 0
           the RMW state machine never advances and the ADPCM-A start regs
           (bank B 0B/13/1B/23/2B) are never emitted -> attract BGM silent.
           Shadow readback gives the RMW its stored value with zero audio. */
        if (reg < 0x10) y->sh_lo[reg] = val;
    } else {
        /* ymfm ym2610 upper port (write_data_hi): 0x100-0x12F -> ADPCM-A,
           0x130-0x1FF -> FM.  Golden trace (MAME 0.289 LINORAGX_GOLDEN probe,
           mslug3 117 s) proves the mslug3-family BGM = FM freq/key-on regs
           0xA1-A6/0xB1-B6 + operator regs + ADPCM-A regs, ALL written via
           bank B.  The old split (bank A = OPM file, bank B = SSG/ADPCM-B)
           came from legacy MAME ymopn.cpp semantics and is wrong for ymfm:
           it dropped every bank-B FM write (freq regs never set -> silence)
           and the 40-4D SSG alias only produced a DC rail. */
        y->chip.write(2, reg); y->sh_hreg = reg; y->chip.write(3, val); y->sh_hi[reg] = val;
    }
    /* GOLDPORT: in gold mode the core owns busy (ymfm_set_busy_end on every
       write) and the timers (ymfm_set_timer via write_data 0x24-0x27); the
       hand-written busy/timer bookkeeping below is legacy-layer only.
       Gated on GOLD_TICK (not gold) so GOLDTICK=0 fully restores the legacy
       timer state machine — without the bookkeeping ten/tcnt stay zero and
       the legacy tick in ym_advance spins harmlessly (task-266 bisection). */
    if (!GOLD_TICK(y)) {
    y->busy[c] = 1;                          /* real YM2610 raises D7 after a write */
    y->busy_at[c] = y->clk + 200;            /* ...and clears it ~25us later on its own */

    if (c == 0) { /* timers belong to bank A only (bank B 24/27 are ADPCM-A
                     regs in the ymfm routing). NOTE (task-250): opening this
                     gate to c<=1 was tried and REGRESSED kof98 25→0 loud —
                     bank-B ten/tflag activity flips IN(06) bits that the
                     kof-family drivers branch on. samsho2's bank-B timer
                     registers are never written anyway. */
    /* STRICT OPN semantics (matches ymfm REG_MODE accessors, vendor
       ymfm_opn.h:196-205, and the MAME golden trace):
       0x24 = timer A value lo, 0x25 bits0-1 = timer A value hi (10-bit total,
       period = (1024-value)*192 clocks), 0x26 = timer B value (8-bit,
       period = (256-value)*80 clocks).  0x27: bit0 load A, bit1 load B,
       bit2 enable A, bit3 enable B, bit4 reset A, bit5 reset B; bit6/7 kept
       as the legacy active-low IRQ mask (all observed drivers write 0 there
       = enabled, matching ymfm's default irq mask 0x03).
       Golden proof: mslug3 arms 0x24=AC/0x25=03 (value 940) then re-loads
       0x27=35 ~3x/frame -> timer A IRQ every (1024-940)*192=16128 clk ~4.5ms
       -> MAME f800 heartbeat +3/frame; the legacy bit mapping ran this as
       timer B at 940*80 clk (4.7x too slow) and starved the m1 sequencer. */
    if (reg == 0x24) { y->tload[c] = (y->tload[c] & 0x300) | val;
        { extern int g_ym_frame; static int t24=-1; if (t24<0) t24=getenv("R24DBG")?1:0;
          if (t24) fprintf(stderr, "R24 f=%d val=%02x\n", g_ym_frame, val); } }
    if (reg == 0x25) { y->tload[c] = (y->tload[c] & 0x0FF) | ((val & 0x03) << 8);
        { extern int g_ym_frame; static int t25=-1; if (t25<0) t25=getenv("R24DBG")?1:0;
          if (t25) fprintf(stderr, "R25 f=%d val=%02x tload=%d\n", g_ym_frame, val, y->tload[c]); } }
    if (reg == 0x26) { y->tloadb[c] = val; y->r26seen[c] = 1; }
    if (reg == 0x27) {
        { static int r27 = -1;
          if (r27 < 0) r27 = getenv("R27DBG") ? 1 : 0;
          if (r27) { extern int g_ym_frame;
                     fprintf(stderr, "R27 bank%c f=%d val=%02x tload=%d\n", 'A'+c, g_ym_frame, val, y->tload[c]); } }
        /* ymfm one-shot parity for timer A too (task-188): re-arm only when
           the timer is stopped. mslug4 golden proof: the 68K BIOS self-test
           polls status bit0 and takes 241 frames in MAME (timer A fires once
           per frame, one-shot) but only 25 frames in lin because the old
           auto-reload kept bit0 permanently set, collapsing every wait loop. */
        if (val & 0x01) { if (y->tcnt[c] <= 0) { int v = 1024 - (y->tload[c] & 0x3FF); y->tcnt[c] = v > 0 ? v : 1;
            static int armd = -1; if (armd < 0) armd = getenv("ARMDBG") ? 1 : 0;
        if (armd && c == 0) { extern long g_arm_n; extern unsigned g_cur_zpc; extern int g_ym_frame;
                if ((g_arm_n++ % 50) == 0) fprintf(stderr, "ARM f=%d zpc=%04x tload=%d\n", g_ym_frame, g_cur_zpc, y->tload[c] & 0x3FF); } } }
        /* ymfm one-shot parity (ymfm_fm.ipp update_timer: re-arm only when the
           timer is NOT running): a running timer B is NEVER reloaded by the
           driver's per-frame 0x27 writes; it counts down undisturbed, fires,
           stops (tcntb=0) and is re-armed by the NEXT 0x27 write. lin's old
           unconditional reload re-armed B every frame (mslug3 writes 27=2f
           every frame) so B's flag timing was decoupled from the count. */
        if (val & 0x02) { if (y->tcntb[c] <= 0) { int v = 256 - (y->tloadb[c] & 0xFF); y->tcntb[c] = v > 0 ? v : 1; } }
        y->ten[c] = ((val & 0x04) >> 1) | ((val & 0x08) >> 3);
        y->irqen[c] = val & 0xC0;
        y->irqen5[c] = (val >> 5) & 1;
        if (val & 0x10) y->tflag[c] &= ~1;
        if (val & 0x20) y->tflag[c] &= ~2;
    }
    /* YM2610 timer IRQ enable lives in reg 0x29 (bit0=A, bit1=B, ACTIVE HIGH),
       NOT in 0x27 bits6/7 (that's YM2203/OPN2 wiring). ymfm ym2610::write_data:
       m_irq_enable=data; mask=enable&~flag_control&3. Golden proof: sm1 BIOS
       and mslug3 m1 never write 0x29 -> YM IRQ line stays LOW in MAME and
       every observed driver POLLS the status flags instead. linoragx's old
       active-low 0x27 interpretation fed the Z80 a phantom IRQ storm. */
    if (reg == 0x29) { y->irqmask[c] = val & 0x1f; y->irq29[c] = 1; }
    /* YM2151 native timer (task-206): 0x1C = latch (bit0-6) + IRQ mask (bit7),
       0x1F = TEST: bit3 load, bit4 reset timer/flag. Auto-reload on expiry. */
    if (reg == 0x1c) { y->opm_latch = val & 0x7f; y->opm_mask = (val >> 7) & 1;
        y->opm_started = 1;
        { static int c1c = -1; if (c1c < 0) c1c = getenv("R1CDBG") ? 1 : 0;
          if (c1c) { extern int g_ym_frame; fprintf(stderr, "R1C f=%d v=%02x latch=%d mask=%d\n", g_ym_frame, val, y->opm_latch, y->opm_mask); } }
        if (y->opm_cnt <= 0) y->opm_cnt = y->opm_latch + 1; }
    if (reg == 0x1f) { if (val & 0x08) y->opm_cnt = y->opm_latch + 1;
                       if (val & 0x10) { y->tflag[c] &= ~1; y->opm_flag = 0; } }
    }
    } /* !gold */
}

void ym_write(ym2610 *y, int chipno, uint8_t reg, uint8_t val)
{
    if (ym_queue_on()) { extern uint32_t g_z_step; YMWrite w = { g_z_step, (uint8_t)chipno, reg, val }; y->wq.push_back(w); return; }
    ym_write_core(y, chipno, reg, val);
}

/* ---- timers (host-owned: ymfm leaves timing to the caller) ---- */

void ym_advance(ym2610 *y, int zcycles)
{
    /* GOLDPORT: zcycles is the YM device-clock domain (YM_CLOCK 8 MHz, same
       domain ymfm's duration_in_clocks uses). The core timers armed via
       ymfm_set_timer fire inside lin_tick(); engine_timer_expired -> status
       flag -> engine_check_interrupts -> ymfm_update_irq drives the Z80 IRQ
       level exactly like MAME's device timers + inputline. */
    if (GOLD_TICK(y)) { y->clk += (unsigned long)zcycles; y->chip.lin_tick(zcycles); return; }
    static int f_notimerb = -1;
    if (f_notimerb < 0) f_notimerb = getenv("NOTIMERB") ? 1 : 0;
    /* ymfm fm_engine_base::update_timer (ymfm_fm.ipp:1480): timer B period =
       16*(256-val) vs timer A (1024-val) — B steps 16x slower per count.
       lin timer A step = 192 Z80 clocks, so B = 192*16 = 3072. The old 80
       made B tick FASTER than A (opposite of ymfm) and over-asserted the
       status bit1 that mslug-family drivers branch on. Override: BSTEP=<n>. */
    static int bstep = -1;
    if (bstep < 0) { const char *e = getenv("BSTEP"); bstep = e ? (int)strtol(e, NULL, 0) : 4576; }
    /* timer A: 1 step / 192 clocks; timer B: 1 step / 80 clocks */
    y->clk += (unsigned long)zcycles;        /* busy D7 auto-clear clock */
    { static int tp = -2;
      if (tp == -2) tp = getenv("TIMPROBE") ? 1 : 0;
      if (tp) { static unsigned long n = 0;
                if ((n++ % 12000) == 0)
                    fprintf(stderr, "TIM ten=%d tload=%d tcnt=%d tflag=%d irqen=%02x clk=%lu\n",
                            y->ten[0], y->tload[0], y->tcnt[0], y->tflag[0], y->irqen[0], y->clk); } }
    y->tfrac += zcycles;
    /* YM2151 OPM timer tick (task-206): 64us per count at 4MHz Z80 = 256
       clocks; auto-reload from latch on expiry (MAME ym2151 timer_callback).
       Overflow sets the shared status bit0 and re-arms the IRQ latch, so the
       level IRQ paces at one delivery per Z80 status read (~frame rate). */
    { static int opm_on = -1;
      if (opm_on < 0) opm_on = getenv("OPMTIMER") ? atoi(getenv("OPMTIMER")) : 1;
      { static int od = -2; static long onc = 0;
        if (od == -2) od = getenv("OPMDBG") ? 1 : 0;
        if (od && (onc++ % 20000) == 0)
            fprintf(stderr, "OPMDB started=%d mask=%d latch=%d cnt=%d frac=%d flag=%d\n",
                    y->opm_started, y->opm_mask, y->opm_latch, y->opm_cnt, y->opm_frac, y->opm_flag); }
      /* MAME ym2151 parity: the timer runs CONTINUOUSLY once armed (mask only
         gates the IRQ assertion, never the counter). Gating the tick on mask
         dead-locked samsho2: its r28 writes alternate f6(bit7=1)/06(bit7=0)
         and the last write before a task gap left mask=0, so the timer froze
         and the task that would re-arm it never ran. zcycles here is the YM
         clock domain (8 MHz = 2x Z80, neogeo.c ym_advance), so the 64us OPM
         timer step is 512 of these, not 256. */
      if (opm_on && y->opm_started) {
          y->opm_frac += zcycles;
          while (y->opm_frac >= 512) {
              y->opm_frac -= 512;
              if (--y->opm_cnt <= 0) {
                  y->tflag[0] |= 1;
                  /* MAME ym2151: overflow ALWAYS latches status bit0; the mask
                     only gates the IRQ assertion level (status & mask). Gating
                     the set on mask lost samsho2 ~2/3 of its IRQs: the driver
                     parks mask=0 between arming writes, and the flag that a
                     later mask 0->1 write should immediately expose was never
                     latched, so the Z80 had to wait for the next overflow. */
                  y->opm_flag = 1;
                  y->irq_latch[0] = 0;
                  y->opm_cnt = y->opm_latch + 1;
              }
          }
      } }
    { static int opna = -2;
      if (opna == -2) opna = getenv("OPNA") ? (int)strtol(getenv("OPNA"), NULL, 0) : 0;
      if (opna > 0) {
          y->opna_cnt += zcycles;
          while (y->opna_cnt >= opna) { y->opna_cnt -= opna; y->opna_flag = 1; y->tflag[1] |= 1; }
      } }
    /* Tick calibration (task-196 golden kof10th): timer A period measured in
       two independent golden phases — sm1 latch 0x51 (tcnt 256) -> 18.4ms and
       m1 latch 0xAC (tcnt 84) -> 6.0ms — both = 286 Z80 clocks per tick at
       4 MHz. The old 192 ran every timer-driven driver 1.5x fast. Override:
       TSTEP=<n>. */
    static int tstep = -1;
    if (tstep < 0) { const char *e = getenv("TSTEP"); tstep = e ? (int)strtol(e, NULL, 0) : 286; }
    while (y->tfrac >= tstep) {
        y->tfrac -= tstep;
        for (int c = 0; c < 2; c++) {
            if (!(y->ten[c] & 2)) continue;
            if (--y->tcnt[c] <= 0) {
                y->tflag[c] |= 1;
                y->irq_latch[c] = 0;
                extern long g_tflagA_sets; g_tflagA_sets++;
                y->tpend[c] = y->clk ? (int)y->clk : 1;
                y->tcnt[c] = 0;   /* one-shot (ymfm): stop until next 0x27 arm */
                if (c == 0) y->intf.fire_timer(0);
            }
        }
    }
    for (int c = 0; c < 2; c++) {
        if (!(y->ten[c] & 1)) continue;
        if (f_notimerb) continue;   /* golden MAME proof (task-183): mslug3
                                         status reads are 01-dominant — timer B
                                         effectively never sets its flag. lin's
                                         B tick rate over-asserts bit1 and flips
                                         the driver's BIT-1 branch vs golden. */
        y->tfracb[c] += zcycles;
        while (y->tfracb[c] >= bstep) {
            y->tfracb[c] -= bstep;
            if (--y->tcntb[c] <= 0) {
                y->tflag[c] |= 2;
                y->irq_latch[c] = 0;
                /* Timer-B auto-reload, gated on the driver having explicitly
                   loaded reg 0x26 (r26seen). Hardware parity: the golden
                   driver re-arms B with dense 0x27 writes (samsho2 1306/s)
                   so B overflows continuously; lin's Z80 runs the driver at
                   ~1/s re-arm so a ymfm one-shot leaves B dead, IN(04) bit1
                   never flips, and the m1 TCB-init branch (IN A,(04); BIT 1,A)
                   mis-routes — attract BGM stalls (samsho2 0→41/50s with
                   reload). Cards that never write 0x26 (kof98/kof2000) keep
                   the strict ymfm one-shot: reloading for them regressed
                   kof98 9→0. Override with ONE_SHOT_B=1 to force one-shot. */
                static int oneshot_b = -1;
                if (oneshot_b < 0) oneshot_b = getenv("ONE_SHOT_B") ? 1 : 0;
                if (!oneshot_b && y->r26seen[c]) { int v = 256 - (y->tloadb[c] & 0xFF); y->tcntb[c] = v > 0 ? v : 1; }
                else y->tcntb[c] = 0;   /* one-shot: stop until next 0x27 arm (ymfm) */
                y->tpend[c] = y->clk ? (int)y->clk : 1;
                { int v = 256 - (y->tloadb[c] & 0xFF); (void)v; }
                if (c == 0) y->intf.fire_timer(1);
            }
        }
    }
}

/* NRXSTATE11: host-owned YM timer/IRQ state + ymfm register shadow.
 * Without this a resumed run restarted the timers from zero: the Z80 music
 * driver (tick-driven) fell permanently out of phase — kof10th diverged on
 * screen within 20 frames of any mid-attract resume. */
struct ym_blob {
    int busy[2]; unsigned long clk; unsigned long busy_at[2];
    int tcnt[2], tload[2], ten[2], tflag[2], tfrac, tpend[2];
    int tcntb[2], tloadb[2], tfracb[2], irqen[2];
    int irqmask[2], irq29[2], irqen5[2];
    int opm_latch, opm_mask, opm_cnt, opm_frac, opm_flag, opm_started;
    uint8_t sh_lo[256], sh_hi[256];
};
int ym_save_state(ym2610 *y, void *buf)
{
    ym_blob *b = (ym_blob*)buf;
    memcpy(b->busy, y->busy, sizeof b->busy); b->clk = y->clk;
    memcpy(b->busy_at, y->busy_at, sizeof b->busy_at);
    memcpy(b->tcnt, y->tcnt, sizeof b->tcnt); memcpy(b->tload, y->tload, sizeof b->tload);
    memcpy(b->ten, y->ten, sizeof b->ten);   memcpy(b->tflag, y->tflag, sizeof b->tflag);
    b->tfrac = y->tfrac; memcpy(b->tpend, y->tpend, sizeof b->tpend);
    memcpy(b->tcntb, y->tcntb, sizeof b->tcntb); memcpy(b->tloadb, y->tloadb, sizeof b->tloadb);
    memcpy(b->tfracb, y->tfracb, sizeof b->tfracb); memcpy(b->irqen, y->irqen, sizeof b->irqen);
    memcpy(b->irqmask, y->irqmask, sizeof b->irqmask); memcpy(b->irq29, y->irq29, sizeof b->irq29);
    memcpy(b->irqen5, y->irqen5, sizeof b->irqen5);
    b->opm_latch = y->opm_latch; b->opm_mask = y->opm_mask;
    b->opm_cnt = y->opm_cnt; b->opm_frac = y->opm_frac; b->opm_flag = y->opm_flag;
    b->opm_started = y->opm_started;
    memcpy(b->sh_lo, y->sh_lo, 256); memcpy(b->sh_hi, y->sh_hi, 256);
    return (int)sizeof(ym_blob);
}
int ym_load_state(ym2610 *y, const void *buf)
{
    const ym_blob *b = (const ym_blob*)buf;
    memcpy(y->busy, b->busy, sizeof y->busy); y->clk = b->clk;
    memcpy(y->busy_at, b->busy_at, sizeof y->busy_at);
    memcpy(y->tcnt, b->tcnt, sizeof y->tcnt); memcpy(y->tload, b->tload, sizeof y->tload);
    memcpy(y->ten, b->ten, sizeof y->ten);   memcpy(y->tflag, b->tflag, sizeof y->tflag);
    y->tfrac = b->tfrac; memcpy(y->tpend, b->tpend, sizeof y->tpend);
    memcpy(y->tcntb, b->tcntb, sizeof y->tcntb); memcpy(y->tloadb, b->tloadb, sizeof y->tloadb);
    memcpy(y->tfracb, b->tfracb, sizeof y->tfracb); memcpy(y->irqen, b->irqen, sizeof y->irqen);
    memcpy(y->irqmask, b->irqmask, sizeof y->irqmask); memcpy(y->irq29, b->irq29, sizeof y->irq29);
    memcpy(y->irqen5, b->irqen5, sizeof y->irqen5);
    y->opm_latch = b->opm_latch; y->opm_mask = b->opm_mask;
    y->opm_cnt = b->opm_cnt; y->opm_frac = b->opm_frac; y->opm_flag = b->opm_flag;
    y->opm_started = b->opm_started;
    memcpy(y->sh_lo, b->sh_lo, 256); memcpy(y->sh_hi, b->sh_hi, 256);
    /* NOTE: the shadow is intentionally NOT replayed into ymfm: writing the
     * ADPCM-A/B start registers would fire phantom samples. Restoring the
     * ymfm engine itself needs a proper export/import pass (future work). */
    return 0;
}

extern "C" int ym_irq_pending_dbg(void *y) { return ym_irq_pending((ym2610 *)y); }
extern "C" int ym_gold_dbg(void *y) { ym2610 *p = (ym2610*)y; return p ? (p->gold | (p->irq_level << 4)) : -1; }
int ym_irq_pending(ym2610 *y)
{
    /* GOLDPORT: the core's engine_check_interrupts already computed the exact
       level MAME wires to the audiocpu IRQ line (status & irq_mask, edge
       reported through ymfm_update_irq). Nothing else to model. */
    if (GOLD_IRQ(y)) return y->irq_level;
    /* HYBRID, timer-A only: level IRQ = timer A flag (persistent, cleared by
       the 0x27 bit4 reset) AND active-low IRQA (0x27 bit6) — empirically what
       sm1/m1 drivers act on (they write 0x1f/0x35 with bit6=0 then wait).
       Timer B must NOT drive the line: ymfm's real IRQ mask (reg 0x29) is
       never written by any observed driver (golden: sm1+m1 zero writes), so
       in MAME timer B only ever sets a status bit. Letting B fire the crutch
       made the sm1 BIOS demo handler run at timer-B rate (8/frame) instead
       of the golden 18.2ms timer-A cadence, killing the BIOS drum demo.

       Once-per-frame gate (vliner golden proof): MAME's YM IRQ line never
       fires (zero 0x29 writes) — drivers tick by POLLING status once per
       frame, so each frame yields exactly one tick no matter the timer fires
       1x (BIOS 18.2ms) or 3x (game 6.0ms) per frame. Delivering the crutch
       at raw overflow rate ran vliner's demo sequence gate (fdca vs fd66
       counter compare) at 3.5x speed: audio content burned 3.5x early
       (silent at f=450 while graphics stayed identical). Cap the crutch to
       one delivery per emulated frame. */
    extern int g_ym_frame;
    static int fired_frame = -1, fired_n = 0;
    static int capn = -1; if (capn < 0) capn = getenv("CAPN") ? atoi(getenv("CAPN")) : 1;
    static int lvl = -1; if (lvl < 0) lvl = getenv("LEVELIRQ") ? 1 : 0;
    /* YM2151 native timer level IRQ (task-206 samsho2 golden proof): the OPM
       timer flag holds the line from expiry until the handler's own status
       read ACKs it — one accept per expiry (~100/s golden attract). The
       once-per-frame pulse cap below cannot express this: it delivered at most
       60/s and its slot was usually consumed by a status-poll query that the
       Z80 could not take (di window), so the FM task starved at 2.5/s. */
    if (y->opna_flag) return 1;   /* task-255 OPNA experiment: bank-B timer A level */
    if (y->opm_flag && y->opm_mask && !(y->irqen[0] & 0x40)) return 1;
    if (lvl) {
        /* opt-in full ymfm level model (task-196 experiment): kept for A/B.
           Default pulse+cap below wins the 208-card batch: level model made
           vliner/vlinero BIOS-demo sequencers burn 6x/frame (golden IRQ line
           stays LOW there; drivers poll status, they don't take the IRQ). */
        { int c = 0;
          return (y->irq_latch[c] || !(y->tflag[c] & y->irqmask[c] & 3)) ? 0 : 1; }
    }
    if (!(y->tflag[0] & 1) || (y->irqen[0] & 0x40)) return 0;
    /* task-205: 0x27 bit5-as-MASK gate FALSIFIED again (mslug3 2f/vliner/kof10th
       35 all bit5=1 yet depend on the crutch; golden kof10th ZIRQ=157Hz with
       bit5=1 every frame). Golden cthd2003 bank A writes mirror lin's (27=35
       x12954, ADPCM-B start/stop, FM keyons) — the divergence is downstream:
       lin's task chain spins 268x/frame from the IRQ handler while golden's
       main loop emits bank B FM notes (golden bank B 3455 writes, lin 38). */
    if (fired_frame != g_ym_frame) { fired_frame = g_ym_frame; fired_n = 0; }
    if (fired_n >= capn) return 0;
    fired_n++;
    return 1;
}

/* Data-port readback (Z80 IN on the YM data port). mslug5's cart m1 ADPCM-A
 * player performs OUT(4)=reg; IN(5) read-modify-write on bank-A regs <0x0E
 * (SSG reg 7 as GPIO). Return the shadow value; audio path untouched. */
uint8_t ym_read_data(ym2610 *y, int chipno, uint8_t reg)
{
    return chipno ? y->sh_hi[reg] : y->sh_lo[reg];
}

uint8_t ym_read(ym2610 *y, int chipno)
{
    /* GOLDPORT: ymfm core status, exactly what MAME's status_lo_r /
       status_hi_r return. Bank A (lower port) = OPN timer flags + core busy
       window (ymfm_is_busy -> lin_is_busy, advanced by lin_tick); bank B
       (upper port) = ADPCM EOS bits gated by the flag mask. No read-clear:
       flags are cleared by the driver's 0x27 reset bits, per the core. */
    if (GOLD_RD(y)) {
        if (chipno) return y->chip.read_status_hi();
        uint8_t st = y->chip.read_status();
        /* datasheet parity: status read clears the timer flags (and with
           them the level IRQ) so polling drivers deassert without a 0x27
           reset write. Task-266 A/B: this is the lifeline for the 9-card
           latch family (dragonsh/kof98/2020bb/aof2/cphd/fatfury family/
           gowcaizr:
           m1 task-chain gap means no 0x27 reset -> level latches -> RST38
           storm; read-clear lets the polling handler deassert). doubledr's
           regression is GOLDIRQ-side, not here (bisection matrix). */
        y->chip.lin_clear_timer_flags();
        /* ADPCM-B EOS on the bank-A status bit6 (legacy-layer semantics the
           mslug4 BGM pump depends on; ymfm keeps it out of read_status). */
        if (y->chip.adpcm_b_status() & ymfm::adpcm_b_channel::STATUS_EOS)
            { st |= 0x40; y->chip.adpcm_b_clear_eos(); }
        return st;
    }
    int c = chipno ? 1 : 0;
    uint8_t st = (uint8_t)(y->tflag[c] & 3);
    /* ymfm parity: the OPM (YM2151) half of YM2610 implements NO timers, so
       bank-A status bit1 (timer B flag) is always 0 in MAME. samsho2's m1
       IRQ handler gates its heartbeat loop on IN(04) bit1 (bit 1,a; jp nz) —
       lin's phantom timer B set bit1, flipping that branch and skipping the
       heartbeat loop entirely. Env-gated experiment: OPM_B0=1 masks bit1. */
    { static int opm_b0 = -2;
      if (opm_b0 == -2) opm_b0 = getenv("OPM_B0") ? 1 : 0;
      if (opm_b0 && c == 0) st &= ~2; }
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
    { static int eosdbg = -2;
      if (eosdbg == -2) eosdbg = getenv("EOSDBG") ? 1 : 0;
      /* task-256: golden IN(06) bit0/bit1 are ADPCM-A channel EOS bits
         (ymfm ym2610::read_status_hi = m_eos_status & m_flag_mask; the A-side
         drum EOS comes from m_adpcm_a.clock). lin never exposed them, so the
         m1 handler's fc37 bit0 branch (0837 ADPCM-B music init) never fired. */
      if (eosdbg && chipno == 1) {
          extern int g_ym_frame;
          static int ecount = 0;
          uint8_t rsh = y->chip.read_status_hi();
          if (rsh && ecount++ < 40) fprintf(stderr, "EOSDBG f=%d rsh=%02x\n", g_ym_frame, rsh);
          else if (!rsh) ecount |= 0; 
          if ((ecount & 0x40000000) == 0 && rsh == 0 && g_ym_frame > 2000 && g_ym_frame < 2003) fprintf(stderr, "EOSDBG f=%d rsh=00 (attract)\n", g_ym_frame);
          st |= (rsh & 0x3f); } }
    y->tflag[c] = 0;
    if (!c) y->opm_flag = 0;   /* MAME ym2151 status_r is read-clear: the
                                  bank-A status read ACKs the OPM timer flag
                                  (and with it the level IRQ line) */
    if (c) y->opna_flag = 0;   /* task-255 OPNA: bank-B status read ACKs the
                                  free-running timer A level */
    y->irq_latch[c] = 1;   /* status read ACKs the level IRQ: line stays low
                              until the NEXT timer overflow re-clears the latch
                              (ymfm read_status parity; without it the Z80
                              re-accepts the level IRQ every return-from-int
                              and the demo/task sequencers burn ~6000x/frame) */
    /* Status read ACKs timer A: legacy read-clear semantics. Some m1 drivers
       (shocktr2) spin "poll status; if flag: process" WITHOUT the 0x27 reset
       bit — with a persistent flag they livelock. The 0x27 bit4/5 reset path
       above still serves drivers that do clear explicitly. */
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
long g_arm_n = 0;         /* diagnostic: count of timer-A re-arms (ARMDBG) */
unsigned g_cur_zpc = 0;   /* diagnostic: latest Z80 PC (set from neogeo.c step hook) */

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
    if (ym_queue_on()) {
        /* Segmented generation: apply queued writes at their instruction-time
           proportion of the frame, generating samples between write points so
           same-frame keyon/keyoff pairs each get audible samples. */
        extern uint32_t g_z_step;
        uint32_t total = g_z_step ? g_z_step : 1;
        size_t produced = 0, i = 0; uint32_t prev = 0;
        while (i <= y->wq.size() && (int)produced < need) {
            uint32_t stop = (i < y->wq.size()) ? y->wq[i].step : total;
            if (stop > total) stop = total;
            size_t seg = (size_t)((uint64_t)(stop - prev) * (uint64_t)need / total);
            if (i == y->wq.size()) seg = (size_t)need - produced;
            if (seg > (size_t)need - produced) seg = (size_t)need - produced;
            if (seg > 0) {
                y->chip.generate_split(y->obuf_fm.data()+produced, y->obuf_a.data()+produced, y->obuf_b.data()+produced, seg);
                produced += seg;
            }
            while (i < y->wq.size() && y->wq[i].step <= stop) {
                YMWrite &w = y->wq[i];
                ym_write_core(y, w.chip, w.reg, w.val);
                i++;
            }
            prev = stop;
        }
        y->wq.clear();
        if ((int)produced < need) {   /* tail safety: never under-produce */
            y->chip.generate_split(y->obuf_fm.data()+produced, y->obuf_a.data()+produced, y->obuf_b.data()+produced, need-produced);
        }
    } else {
        y->chip.generate_split(y->obuf_fm.data(), y->obuf_a.data(), y->obuf_b.data(), need);
    }

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
          fprintf(stderr, "[yms] f=%d need=%d frames=%d fm=%d aa=%d ab=%d\n", g_ym_frame, need, frames, mx, ma, mb); } }
    if (ytm) { clock_gettime(CLOCK_MONOTONIC, &yts1);
        ytm_acc += (yts1.tv_sec - yts0.tv_sec) * 1e6 + (yts1.tv_nsec - yts0.tv_nsec) / 1e3;
        if ((ytc++ % 300) == 0) { fprintf(stderr, "[ymtm] avg=%.0fus last=%.0fus frames=%d\n", ytm_acc / 300.0,
            (yts1.tv_sec - yts0.tv_sec) * 1e6 + (yts1.tv_nsec - yts0.tv_nsec) / 1e3, frames); ytm_acc = 0; } }
}
