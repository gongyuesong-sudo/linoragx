/* Neo Geo machine (MVS): memory map per MAME snk/neogeo.cpp (2025 hardware-accurate).
 * File roles: p1/p2=68k prog (word-swap), s1=fixed gfx, m1=z80 prog, c1-c4=sprites, v1-v4=ADPCM.
 * VRAM via VLOAD port at 0x3C0000; CRAM direct at 0x400000.
 * 68k core: Musashi. Z80 core: cpu_z80 (MIT) via z80_wrap. */
#include "linoragx.h"
/* ---- tiny parallel-for for bulk decrypt passes (32-core host) ---- */
#include <pthread.h>
typedef void (*pf_fn)(void *arg, uint32_t lo, uint32_t hi);
typedef struct { pf_fn fn; void *arg; uint32_t lo, hi, next, gran; } pf_job;
static void *pf_worker(void *p)
{
    pf_job *j = (pf_job *)p;
    for (;;) {
        /* step MUST be a multiple of the worker's block granularity: workers
           start their loop AT lo, so a misaligned chunk start (e.g. ds_run's
           0x80 sprite blocks starting at lo+1) decodes garbage and scribbles
           the predecoded plane from the 2nd chunk onward (mslug5 snow). */
        uint32_t step = (j->hi - j->lo) / 32 + 1;
        step = (step + j->gran - 1) & ~(uint32_t)(j->gran - 1);
        uint32_t lo = __atomic_add_fetch(&j->next, step, __ATOMIC_RELAXED) - step;
        if (lo >= j->hi) break;
        uint32_t hi = lo + step < j->hi ? lo + step : j->hi;
        j->fn(j->arg, lo, hi);
    }
    return NULL;
}
static void parallel_for_g(pf_fn fn, void *arg, uint32_t lo, uint32_t hi, int nthreads, uint32_t gran)
{
    if (hi <= lo + 0x10000 || nthreads < 2) { fn(arg, lo, hi); return; }
    pthread_t th[16];
    if (nthreads > 16) nthreads = 16;
    pf_job j = { fn, arg, lo, hi, lo, gran };
    for (int i = 0; i < nthreads - 1; i++) pthread_create(&th[i], NULL, pf_worker, &j);
    pf_worker(&j);
    for (int i = 0; i < nthreads - 1; i++) pthread_join(th[i], NULL);
}
static void parallel_for(pf_fn fn, void *arg, uint32_t lo, uint32_t hi, int nthreads)
{
    parallel_for_g(fn, arg, lo, hi, nthreads, 1);
}

#include "m68k.h"
#include <stdlib.h>
#include "m68k.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define BIT(x,b) (((x)>>(b))&1)
#define CYCLES_PER_FRAME  202752  /* MAME: HTOTAL 384 px * VTOTAL 264 * 2 (68k=12MHz, pixel=6MHz) */
#define LINES_PER_FRAME   264     /* MAME NEOGEO_VTOTAL 0x108 */
#define CYCLES_PER_LINE   768     /* 384 pixel clocks * 2 */

struct neogeo {
    /* ROM images */
    uint8_t *rom;   uint32_t rom_size;    /* concatenated PRG p1+p2+p3 (word-swapped), up to 8MB */
    uint32_t prg_bank_base;               /* MAME write_banksel: (bank+1)*0x100000 window @0x200000 */
    uint8_t *fixed; uint32_t fixed_size;  /* s1: packed 4bpp fix tiles */
    uint8_t *snd;   uint32_t snd_size;    /* m1: z80 program */
    uint8_t *acrypt; uint32_t acrypt_size; /* encrypted m1 (cmc50 carts) */
    int fixed_bank_type;                   /* MAME FIX_BANKTYPE: 1=GAROU(mslug3) 2=KOF2000 */
    uint16_t jgram[0x1000];               /* NEOGEO_JOCKEYGP/VLINER cart RAM @200000-201fff */
    uint8_t *spr;   uint32_t spr_size;    /* c1-c4 byte-interleaved sprite planes */
    uint8_t *sprite8; uint32_t sprite8_mask;  /* predecoded 8bpp sprite gfx */
    uint8_t *sm1;                         /* sound bios (sm1.sm1, 128K) */
    int ym_rom_set;                       /* audio ROM handed to ymfm once */
    uint8_t *bios;  uint32_t bios_size;   /* 68k bios (NEO-GEO.ROM, 128K) */
    uint8_t *sfix;                        /* fixed bios (NG-SFIX.ROM / sfix.sfix, 128K) */
    uint8_t  zoomy[0x20000];              /* vertical zoom table (real or synthesized) */
    int      have_zoom;

    /* RAM */
    uint16_t vram[0x10000];
    uint16_t cram[0x10000];
    uint16_t workram[0x8000];
    uint8_t  sndram[0x800];
    uint8_t  sram[0x10000];
    uint8_t  mc_data[0x800];
    int      mc_lock1, mc_unlock2, mc_regsel;
    int      sram_unlocked;            /* systemlatch ch6: SRAM CE */
    int      cart_prot;                /* 1=kof98 2=kof99 3=garou 4=mslug3 5=kof2000 6=PVC (SMA/PVC PRG decrypted) */
    int      kof98_state;
    uint16_t kof98_trig;
    uint16_t kof98_default[2];
    uint16_t sma_rng;                  /* SMA LFSR (init 0x2345) */
    int znmi_t;              /* post-NMI trace countdown */
    int m1_view;               /* 1 = cart m1 visible in Z80 space (during ch5 pulses or after boot) */
    int m1_booted;             /* 1 = game-phase handoff reset done */
    int ch5_bios_pend;         /* ch5 went 0->1 while 68K still in BIOS: candidate handoff */
    unsigned long ch5_on_frame;/* frame of that 0->1 (sustained >=3 frames = real handoff, pulse = transient) */
    uint16_t k3bl_overlay;         /* kof2k3bl bootleg overlay register */
    uint8_t k3bl_px;               /* 1=pl_px_decrypt applied, 2=upl variant */
    uint16_t pvc_ram[0x1000];          /* PVC protection RAM @ $2FE000-$2FFFFF */
    uint32_t cram_bank;
    uint32_t screen_shadow;   /* systemlatch ch0: MAME set_screen_shadow (150R pulldown) */
    uint32_t cram_bank_render;  /* bank at first CRAM write of the frame (MAME line-render parity) */
    uint64_t bank_snapf;
    int dbg_vw; int dbg_z80; int sl1_trace;

    /* VLOAD / video ctrl */
    uint16_t vram_offset, vram_modulo, vram_readbuf;
    uint8_t  auto_speed; int auto_disabled; uint8_t auto_counter;
    int irq_enable; uint32_t display_counter; int raster_pending, raster_armed; unsigned long vload_count, nmi_cnt, p0_cnt, sl1_cnt;
    int irq_line;
    int vblank_enable;

    /* system ctrl */
    int vector_bank;
    int cramw;
    int sramw;
    int screen_enabled;
    uint8_t soundlatch1;      /* 68k -> z80 command */
    uint8_t soundlatch2;      /* z80 -> 68k status */
    int z80_nmi_en, z80_nmi_pend, z80_nmi_asserted;
    int m1_handoff;          /* 0=arm pending, 1=armed (ch5 seen high), 2=done */
    uint64_t wd_last;        /* mc_cycles at last watchdog feed ($300001 odd write) */
    int wd_armed;
    int ym_irq_prev;
    int irq_sound;            /* IRQ_SOUND: set by Z80 soundlatch2 write, acked on AUDIO_COIN read */
    unsigned long hit_a, hit_b, hit_c;
    int cart_audio;   /* HC259 ch5: 0=sm1 BIOS sound, 1=cart m1 */
    uint8_t zbank[4];         /* f000,e000,c000,8000 entries */

    /* buttons: bit0 UP,1 DN,2 LF,3 RT,4 C,5 D,6 A,7 B,8 start,9 select,10 coin */
    uint32_t buttons[2];
    /* UPD1990A memory card (minimal) */
    int      mc_dummy;
    uint64_t mc_cycles;
    /* UPD4990A serial state */
    int      mc_cki, mc_cs, mc_di;
    int      mc_bitcnt, mc_cmd, mc_readphase;
    uint8_t  mc_resp;          /* response byte, shifted MSB first */
    int      mc_dobit;         /* current DO bit */
    int      mc_tp;
    uint8_t  adpcm_a[4];
    uint8_t  adpcm_b[4];
    uint8_t  ym_reg[2];          /* YM2610 A/B register latch (two-port protocol) */
    int      pcprof;
    unsigned long pcprof_tot;
    /* ctrlsel */
    uint8_t  ctrlsel;
    uint8_t  fc;
    uint64_t frame;
    int      scanline;
    int      perline;     /* 1 = draw lines inside chunk loop; 0 = whole frame at frame end (default) */
    uint32_t *fb;                 /* window framebuffer: when set, ng_run_frame draws
                                     each scanline as it completes (MAME update_partial) */
    int      draw_next;           /* next fb row to draw in the current frame */
    char     gamename[64];
    void    *z;
    ym2610  *ym;
};

/* ---- zoom table synthesis (000-lo.lo replacement) ---- */
static void synth_zoomy(neogeo *ng)
{
    for (int zy = 0; zy < 0x100; zy++)
        for (int l = 0; l < 0x100; l++) {
            int s = (l * 256) / (zy + 1);   /* fallback only; real 000-lo.lo is loaded when present */
            if (s > 0xFF) s = 0xFF;
            ng->zoomy[(zy<<8)|l] = (uint8_t)s;
        }
}

/* ---- sprite gfx predecode: c-planes -> 8bpp, 0x80 rom bytes -> 256 bytes ---- */
static uint32_t region_mask_pow2(uint32_t len)
{
    uint32_t mask = 0xFFFFFFFFu;
    uint32_t dbl = len*2 - 1;
    for (uint32_t bit = 0x80000000u; bit; bit >>= 1) {
        if (dbl & bit) break;
        mask >>= 1;
    }
    return mask;
}

typedef struct { const uint8_t *spr; uint8_t *dst; uint32_t sz; } ds_ctx;
static void ds_run(void *a, uint32_t lo, uint32_t hi)
{
    ds_ctx *c = (ds_ctx *)a;
    for (uint32_t i = lo; i < hi; i += 0x80) {
        const uint8_t *src = c->spr + i;
        uint8_t *dest = c->dst + (i << 1);   /* 4bpp src 0x80 -> 8bpp out 0x100 */
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 8; x++)
                *(dest++) = (BIT(src[0x43|(y<<2)],x)<<3) | (BIT(src[0x41|(y<<2)],x)<<2)
                          | (BIT(src[0x42|(y<<2)],x)<<1) | (BIT(src[0x40|(y<<2)],x)<<0);
            for (int x = 0; x < 8; x++)
                *(dest++) = (BIT(src[0x03|(y<<2)],x)<<3) | (BIT(src[0x01|(y<<2)],x)<<2)
                          | (BIT(src[0x02|(y<<2)],x)<<1) | (BIT(src[0x00|(y<<2)],x)<<0);
        }
    }
}

static void decode_sprites(neogeo *ng)
{
    if (!ng->spr || !ng->spr_size) { ng->sprite8_mask = 0; return; }
    /* MAME get_region_mask(rgn, len): mask = smallest pow2-1 covering 2*len-1,
       using the RAW region length (e.g. mslug5 0x4000001 -> 0x0FFFFFFF).
       Rounding len down to 0x80 first drops a mask bit (0x07FFFFFF) and folds
       high CMC sprite codes (code_ext 0x80000 -> bit 27) to wrong gfx planes
       = full-screen sprite garbling. ds_run writes 2*(len&~0x7f) <= mask+1. */
    uint32_t mask = region_mask_pow2(ng->spr_size);
    ng->sprite8 = calloc(mask+1, 1);
    ng->sprite8_mask = mask;
    ds_ctx c = { ng->spr, ng->sprite8, ng->spr_size };
    parallel_for_g(ds_run, &c, 0, ng->spr_size & ~0x7fu, 12, 0x80);   /* ds_run blocks at 0x80 granularity */
}

/* ---- Z80 side ---- */
extern uint16_t z80w_pc(void*);
/* cached env-flag accessors: getenv on every hot-path call was ~56% of CPU */
#define DBGFLAG(name, var) static int var(void) { static int v = -1; if (v < 0) v = getenv(name) ? 1 : 0; return v; }
DBGFLAG("NOIRQ2", f_noirq2)
DBGFLAG("ZTRACE",  f_ztrace)
static int ztf0(void){ static int v=-1; if(v<0){const char*e=getenv("ZTF0"); v=e?atoi(e):3;} return v; }
static int ztf1(void){ static int v=-1; if(v<0){const char*e=getenv("ZTF1"); v=e?atoi(e):12;} return v; }
DBGFLAG("PCW",     f_pcw)
DBGFLAG("ROMRD",   f_romrd)
DBGFLAG("ROMRD2",  f_romrd2)
DBGFLAG("WP0",     f_wp0)
DBGFLAG("BANKDBG", f_bankdbg)
DBGFLAG("SRAMRD",  f_sramrd)
DBGFLAG("PROTDBG", f_protdbg)
DBGFLAG("SMAPROT", f_smaprot)
DBGFLAG("RAMWA",   f_ramwa)
DBGFLAG("RAMWAF",  f_ramwaf)
DBGFLAG("SCRW",    f_scrw)
DBGFLAG("LATCH_TR",f_latch_tr)
DBGFLAG("WDTR",    f_wdtr)
DBGFLAG("NOCARTAUDIO", f_nocartaudio)
DBGFLAG("M1BOOT1", f_m1boot1)
DBGFLAG("ZTRAIL",  f_ztrail)
DBGFLAG("ZPCHIST", f_zpchist)
DBGFLAG("ZBC",     f_zbc)
DBGFLAG("ZSP",     f_zsp)
DBGFLAG("ZSEQ",    f_zseq)
DBGFLAG("ZRET",    f_zret)
DBGFLAG("Z18",     f_z18)
DBGFLAG("Z390",    f_z390)
DBGFLAG("ZNMI",    f_znmi)
DBGFLAG("ZNT",     f_znt)
DBGFLAG("ZNTD",    f_zntd)
DBGFLAG("CMCTR",   f_cmctr)
DBGFLAG("SWDBG",   f_swdbg)
DBGFLAG("DUMPFIX", f_dumpfix)
DBGFLAG("FD98W", f_fd98w)
DBGFLAG("LINORAGX_TRACE_VLOAD", f_vload)
DBGFLAG("VWDIST", f_vwdist)
DBGFLAG("LINORAGX_TRACE_VC", f_vc)
DBGFLAG("SRAMW", f_sramw)
DBGFLAG("ZNMIT", f_znmit)
DBGFLAG("NORASTER", f_noraster)
DBGFLAG("WARNW", f_warnw)
DBGFLAG("FD82W", f_fd82w)

static uint32_t zbank_base(neogeo *ng, int region, int entry, uint32_t len)
{
    if (len <= 0x10000) return 0;           /* small roms: no banking */
    uint32_t mask = (len - 0x10000 - 1) & 0x3FFFF;
    return 0x10000 + (((uint32_t)entry << (11 + region)) & mask);
}
static uint8_t z_read(uint32_t a, void *c)
{
    neogeo *ng = (neogeo*)c;
    a &= 0xFFFF;
    if (getenv("VIEWDBG") && a <= 0x0012) {
        static long vd = 0;
        if ((vd++ % 20) == 0) fprintf(stderr, "VIEW a=%04x cv=%d m1v=%d ca=%d zpc=%04x f=%llu\n",
            a, ng->snd ? (ng->cart_audio || ng->m1_view) : 0, ng->m1_view, ng->cart_audio,
            z80w_pc(ng->z), (unsigned long long)ng->frame);
    }
    if (getenv("BLKRD") && a >= 0xfc30 && a < 0xff00) {
        static long brd = 0;
        if ((brd++ % 100) == 0) fprintf(stderr, "BLKRD a=%04x pc=%04x f=%llu\n", a, z80w_pc(ng->z), (unsigned long long)ng->frame);
    }
    if (f_ztrace() && a >= 0xFD25 && a <= 0xFD68 && ng->frame > 40 && ng->frame < 60)
        fprintf(stderr, "ZR f=%d a=%04x pc=%04x\n", ng->frame, a, z80w_pc(ng->z));
    /* MAME audio_map parity: 0x0000-0x7FFF = bank_audio_main (entry1=cart
       audiocpu at INIT — set_entry(1) in init_audio — entry0=sm1 when HC259
       ch5=0); 0x8000-0xF7FF = cart banks ALWAYS (never sm1). Z80 never
       resets on ch5. Carts without audio keep entry0=sm1 for everything. */
    /* Empirical MAME parity: Z80 boots sm1 (BIOS driver) and KEEPS executing
       it across ch5 toggles — the sm1 idle loop survives the bank switch;
       ch5=1 only exposes cart m1 in 0x0000-0x7FFF for games that pulse the
       audio reset line. Resetting the Z80 on ch5 (old behavior) deadlocks
       pnyaa-class carts whose m1 defers dispatch on the first NMI. */
    int cart_visible = ng->snd ? (ng->cart_audio || ng->m1_view) : 0;   /* visible during ch5 pulses (transient) and permanently after the post-pulse m1 boot */
    const uint8_t *r = cart_visible ? ng->snd : (ng->sm1 ? ng->sm1 : ng->snd);
    uint32_t sz = cart_visible ? ng->snd_size : (ng->sm1 ? 0x20000 : ng->snd_size);
    if (!r) { r = ng->snd; sz = ng->snd_size; }
    if (a < 0x8000) {
        return (r && a < sz) ? r[a] : 0xFF;
    }
    if (!cart_visible && ng->sm1 && a < 0xF800)
        return ng->sm1[a & 0x1FFFF];           /* BIOS phase: sm1 flat-mapped, NO banking (zbank regs may hold cart-m1 garbage; sm1 is 128K flat) */
    if (r && sz > 0x10000 && cart_visible) {   /* upper banks use the SAME view (r/sz) as the lower 32K: BIOS pulse keeps everything on sm1 (sm1 code spans upper banks); cart view only post-handoff */
        uint32_t b;
        if (a < 0xC000) b = zbank_base(ng,3,ng->zbank[3],sz) + (a-0x8000);
        else if (a < 0xE000) b = zbank_base(ng,2,ng->zbank[2],sz) + (a-0xC000);
        else if (a < 0xF000) b = zbank_base(ng,1,ng->zbank[1],sz) + (a-0xE000);
        else b = zbank_base(ng,0,ng->zbank[0],sz) + (a-0xF000);
        return (b < sz) ? r[b] : 0xFF;
    }
    if (sz <= 0x10000) {                    /* unbanked: direct */
        uint32_t pa = a;
        return (r && pa < sz) ? r[pa] : 0xFF;
    }
    if (sz <= 0x10000) {                    /* unbanked: direct */
        uint32_t pa = a;
        return (r && pa < sz) ? r[pa] : 0xFF;
    }
    if (a < 0xC000) { uint32_t b = zbank_base(ng,3,ng->zbank[3],sz) + (a-0x8000);
        return (r && b < sz) ? r[b] : 0xFF; }
    if (a < 0xE000) { uint32_t b = zbank_base(ng,2,ng->zbank[2],sz) + (a-0xC000);
        return (r && b < sz) ? r[b] : 0xFF; }
    if (a < 0xF000) { uint32_t b = zbank_base(ng,1,ng->zbank[1],sz) + (a-0xE000);
        return (r && b < sz) ? r[b] : 0xFF; }
    if (a < 0xF800) { uint32_t b = zbank_base(ng,0,ng->zbank[0],sz) + (a-0xF000);
        return (r && b < sz) ? r[b] : 0xFF; }
    return ng->sndram[a & 0x7FF];
}
static void z_write(uint32_t a, uint8_t v, void *c)
{
    neogeo *ng = (neogeo*)c;
    a &= 0xFFFF;
    if (getenv("BLKRD") && a >= 0xfc30 && a < 0xff00) {
        static long bwr = 0;
        if ((bwr++ % 50) == 0) fprintf(stderr, "BLKWR a=%04x v=%02x pc=%04x f=%llu\n", a, v, z80w_pc(ng->z), (unsigned long long)ng->frame);
    }
    /* Z80 RAM is 0xF800-0xFFFF (2K). The stack starts at 0xFFFC (SP=0xFFFF at
       reset, first push writes 0xFFFB/0xFFFC): the high bytes are still RAM —
       dropping them corrupts NMI return addresses (crash to 0xFFFD). */
    if (a >= 0xF800) {
        if (f_ztrace() && (a >= 0xF800))
            fprintf(stderr, "ZW f=%d a=%04x v=%02x pc=%04x\n", ng->frame, a, v, z80w_pc(ng->z));
        ng->sndram[a & 0x7FF] = v;
    }
    /* 0x8000-0xF7FF is banked ADPCM ROM: writes ignored (MAME parity) */
}
static uint16_t z_in(uint32_t p, void *c)
{
    neogeo *ng = (neogeo*)c;
    uint32_t port = p & 0xFF;
    if (port >= 0x08 && port <= 0x0B) {
        /* Neo Geo sound bank latch: in a,(08..0B) latches bank[port-8] from
           the A-bus value the program just loaded (ld a,N; in a,(08)).
           The old port-high-byte form never fired (Z80 in ports are 8-bit),
           leaving zbank stuck at 0: samsho2 m1 switches four banks
           (2e/16/0a/04) to window its song table and read garbage instead,
           so its track blocks never initialized and BGM stayed silent. */
        extern uint8_t z80w_a(void*);
        ng->zbank[port - 0x08] = z80w_a(ng->z);
        { static long zbn = 0; if ((zbn++ % 50) == 0) fprintf(stderr, "ZBANK %d <= %02x zpc=%04x f=%llu\n", port - 8, ng->zbank[port - 8], z80w_pc(ng->z), (unsigned long long)ng->frame); }
        return 0;
    }
    switch (port) {
    case 0x00: { uint8_t v = ng->soundlatch1;      /* generic_latch: read returns data,
                 clear_w drops data_pending (NMI line edge), data itself is kept */
                 if (ng->dbg_z80==3) fprintf(stderr, "P0R %02x zpc=%04x f=%d\n", v, z80w_pc(ng->z), ng->frame);
                 ng->z80_nmi_pend = 0; ng->z80_nmi_asserted = 0; ng->p0_cnt++; return v; }
    case 0x04: { uint8_t v = ym_read(ng->ym, 0);
                 { static int st_rd = -2; if (st_rd==-2) st_rd = getenv("STATRD")?1:0;
                   if (st_rd) { static long c4=0; if ((c4++ % 200)==0) fprintf(stderr,"STATRD A=%02x f=%d\n", v, ng->frame); } }
                 return v; }   /* YM2610 A status (timer flags) */
    case 0x06: { uint8_t v = ym_read(ng->ym, 1);
                 { static int st_rd = -2; if (st_rd==-2) st_rd = getenv("STATRD")?1:0;
                   if (st_rd) { static long c6=0; if ((c6++ % 200)==0) fprintf(stderr,"STATRD B=%02x f=%d\n", v, ng->frame); } }
                 return v; }   /* YM2610 B status */
    case 0x05: case 0x07: return 0x00;      /* data read: not modelled */
    case 0xA0: case 0xA1: case 0xA2: case 0xA3:
        return 0x00;                       /* ADPCM-B: always ready (bit0=0) */
    }
    return 0xFF;
}
static void z_out(uint32_t p, uint8_t v, void *c)
{
    neogeo *ng = (neogeo*)c;
    { static int zio = -1; if (zio < 0) zio = getenv("Z80IO") ? 1 : 0;
      if (zio && (p & 0xFF) != 0x0C) fprintf(stderr, "ZIO %02x=%02x\n", p & 0xFF, v); }
    switch (p & 0xFF) {
    case 0x04: ng->ym_reg[0] = v; break;                    /* YM2610 A: register latch (YM2151 shares bus) */
    case 0x05: ym_write(ng->ym, 0, ng->ym_reg[0], v); break;/* YM2610 A: data write (bank A = OPM/YM2151 FM, handled inside ymfm) */
    case 0x06: ng->ym_reg[1] = v; break;                    /* YM2610 B: register latch */
    case 0x07: ym_write(ng->ym, 1, ng->ym_reg[1], v); break;/* YM2610 B: data write */
    case 0x0C: ng->soundlatch2 = v; if (!f_noirq2()) { ng->irq_sound = 1; m68k_set_virq(2, 1); }   /* MAME soundlatch2_w: sets IRQ_SOUND -> 68K level 2 */
              if (ng->dbg_z80==3) { extern uint16_t z80w_pc(void*); uint16_t zp = z80w_pc(ng->z);
                  fprintf(stderr, "S2W %02x zpc=%04x f=%d at=%02x %02x\n", v, zp, ng->frame,
                      z_read((uint32_t)(zp-2), ng), z_read((uint32_t)(zp-1), ng)); } break;
    case 0x08: ng->z80_nmi_en = 1; break;
    case 0x18: ng->z80_nmi_en = 0; break;
    case 0x80: case 0x81: case 0x82: case 0x83:   /* ADPCM-A bank/vol: stored, no playback yet */
        ng->adpcm_a[p & 3] = v; break;
    case 0xA0: case 0xA1: case 0xA2: case 0xA3:   /* ADPCM-B ctrl/bank/start/delta */
        ng->adpcm_b[p & 3] = v; break;
    default: break;
    }
}

/* UPD4990A: 4-bit commands; status/calendar reads return plausible bytes */
static uint8_t mc4990_response(neogeo *ng, int cmd)
{
    (void)ng;
    switch (cmd & 0x0F) {
    case 0x07: return 0x3D;   /* status: OK (BIOS accepts 0x39..0x3F) */
    case 0x08: return 0x3D;   /* status: OK */
    case 0x03: return 0x25;   /* calendar year-ish BCD */

    default:   return 0x00;
    }
}

/* ---- 68k side ---- */
static neogeo *g_ng;
static uint32_t sma_bank(int type, uint16_t sel);
static uint32_t pvc_bank_base(neogeo *ng);
static void pvc_unpack_color(neogeo *ng);
static void pvc_pack_color(neogeo *ng);

static uint16_t r16(neogeo *ng, uint32_t a)
{
    a &= 0xFFFFFF;
    if (ng->dbg_z80 == 5) {
        static unsigned long c1 = 0, c2 = 0, c3 = 0;
        if (a == 0x1012E0) c1++;
        if (a == 0x10A62C) c2++;
        if (a == 0x1041D2) c3++;
        if (a == 0x10A62C && ng->frame > 5000 && (c2 % 200) == 7)
            fprintf(stderr, "A62CRD f=%d pc=%08x\n", ng->frame, m68k_get_reg(NULL,M68K_REG_PC));
        ng->hit_a = c1; ng->hit_b = c2; ng->hit_c = c3;
    }
    if (a < 0x80) {                         /* vectors (mirror of bank base) */
        if (ng->dbg_z80==4 && (a==0x64||a==0x68)) fprintf(stderr, "VFETCH%02x vbank=%d pc=%08x\n", a, ng->vector_bank, m68k_get_reg(NULL,M68K_REG_PC));
        const uint8_t *v = ng->vector_bank ? ng->rom : ng->bios;
        uint32_t sz = ng->vector_bank ? ng->rom_size : 0x20000;
        if (!v || a+2 > sz) return 0xFFFF;
        return (v[a]<<8)|v[a+1];
    }
    if (ng->cart_prot == 1 && a >= 0x100 && a <= 0x103) {   /* kof98 protection port */
        int off = (a >> 1) & 1;
        if (ng->kof98_state == 1) return off ? 0x00fd : 0x00c2;
        if (ng->kof98_state == 2) return off ? 0x4f2d : 0x4e45;
        return ng->kof98_default[off];
    }
    if (a < 0x100000) {
        if (f_pcw() && (a == 0x100 || a == 0x182) && ng->frame > 4100 && ng->frame < 4200) {
            static int r100 = 0;
            if (r100 < 8) { r100++;
                fprintf(stderr, "HDRRD f=%d a=%06x val=%04x romsz=%d pc=%08x\n", ng->frame, a,
                    (ng->rom && a+2 <= ng->rom_size) ? (unsigned)((ng->rom[a]<<8)|ng->rom[a+1]) : 0xFFFFu,
                    ng->rom_size, m68k_get_reg(NULL,M68K_REG_PC));
            }
        }
        if (f_pcw() && a == 0x01820184) {
            static int rw_done = 0;
            if (!rw_done) { rw_done = 1;
                fprintf(stderr, "CMPREAD f=%d a=%06x romsz=%d val=%04x tbl=%04x pc=%08x\n",
                    ng->frame, a, ng->rom_size,
                    (ng->rom && a+2 <= ng->rom_size) ? (unsigned)((ng->rom[a]<<8)|ng->rom[a+1]) : 0xFFFFu,
                    m68k_read_memory_16(0xC13674), m68k_get_reg(NULL,M68K_REG_PC));
            }
        }
        if (!ng->rom || a+2 > ng->rom_size) return 0xFFFF;
        if (f_romrd() && a >= 0x108 && a <= 0x12F && ng->frame < 260) {
            static int rr = 0;
            if (rr < 24) { rr++;
                fprintf(stderr, "ROMRD f=%d a=%06x val=%04x pc=%08x\n", ng->frame, a,
                    (unsigned)((ng->rom[a]<<8)|ng->rom[a+1]), m68k_get_reg(NULL,M68K_REG_PC));
            }
        }
        if (f_romrd2() && ng->frame < 70) {
            static FILE *rf; 
            if (!rf) rf = fopen("/tmp/romrd.log", "w");
            if (rf) fprintf(rf, "f=%d a=%06x v=%04x pc=%08x\n", ng->frame, a,
                (unsigned)((ng->rom[a]<<8)|ng->rom[a+1]), m68k_get_reg(NULL,M68K_REG_PC));
        }
        return (ng->rom[a]<<8)|ng->rom[a+1];
    }
    if (a < 0x200000) {
        if (f_pcw() && (a & 0xFFFFFFFE) == 0x100182) {
            fprintf(stderr, "RD182 f=%d a=%06x val=%04x pc=%08x\n", ng->frame, a,
                ng->workram[(a & 0xFFFF)>>1], m68k_get_reg(NULL,M68K_REG_PC));
        }
        { uint32_t off = a & 0xFFFF, o = off >> 1;
          if (f_wp0() && o <= 1) { uint32_t rpc = m68k_get_reg(NULL,M68K_REG_PC);
              if (rpc >= 0xBE00 && rpc <= 0xBE40)
                  fprintf(stderr, "WPRD f=%d a=%06x w0=%04x w1=%04x pc=%08x\n",
                  ng->frame, a, ng->workram[0], ng->workram[1], rpc); }
          if (off & 1) return ((ng->workram[o] & 0xFF) << 8) | ((o + 1 < 0x8000) ? (ng->workram[o+1] >> 8) : 0);
          return ng->workram[o]; }
    }
    if (ng->cart_prot == 12 && (a & 0xFFF000) == 0x280000) return 0xFFFF;  /* vliner IN5/IN6 */
    if (a < 0x300000) {   /* cart PRG bank window (MAME install_read_bank 0x200000-0x2fffff) */
        if (ng->cart_prot == 12 && a <= 0x201FFF)          /* JOCKEYGP cart RAM */
            return ng->jgram[(a >> 1) & 0xFFF];
        if (ng->cart_prot >= 2 && ng->cart_prot <= 7) {      /* SMA protection ports */
            int rndr = 0;                                    /* random_r port per cart type (MAME neogeo.cpp) */
            switch (ng->cart_prot) {
                case 2: rndr = (a >= 0x2FFFF8 && a <= 0x2FFFFB); break;              /* kof99 */
                case 3: case 4: rndr = (a >= 0x2FFFCC && a <= 0x2FFFF1); break;      /* garou/garouh: 2fffcc-cd + 2ffff0-f1 */
                case 7: rndr = (a >= 0x2FFFD8 && a <= 0x2FFFDB); break;              /* kof2000 */
                case 5: case 6: rndr = (a >= 0x2FFFF8 && a <= 0x2FFFFB); break;   /* mslug3/3a: MAME comments it out, but game reads it */
            }
            if (rndr) {                                        /* SMA random_r LFSR */
                uint16_t old = ng->sma_rng;
                uint16_t nb = (uint16_t)(((ng->sma_rng>>2)^(ng->sma_rng>>3)^(ng->sma_rng>>5)^(ng->sma_rng>>6)
                    ^(ng->sma_rng>>7)^(ng->sma_rng>>11)^(ng->sma_rng>>12)^(ng->sma_rng>>15)) & 1);
                ng->sma_rng = (uint16_t)((ng->sma_rng << 1) | nb);
                return old;
            }
            if (a >= 0x2FE446 && a <= 0x2FE447) return 0x9a37;   /* prot_9a37_r */
        }
        if ((ng->cart_prot == 8 || ng->cart_prot == 13) && a >= 0x2FE000)  /* PVC / kof2k3bl prot RAM */
            return ng->pvc_ram[(a >> 1) & 0xFFF];
        uint32_t o = ng->prg_bank_base + (a - 0x200000);
        if (f_romrd() && o >= 0x100 && o <= 0x12F && ng->frame < 260) {
            static int rm = 0;
            if (rm < 24) { rm++;
                fprintf(stderr, "ROMMIR f=%d a=%06x o=%06x pc=%08x\n", ng->frame, a, o,
                    m68k_get_reg(NULL,M68K_REG_PC));
            }
        }
        if (f_bankdbg() && a == 0x200000 && ng->frame > 4400 && ng->frame < 4700)
            fprintf(stderr, "BANKR f=%d o=%08x v=%04x pc=%08x\n", ng->frame, o,
                (ng->rom && o + 2 <= ng->rom_size) ? ((ng->rom[o]<<8)|ng->rom[o+1]) : 0xFFFF,
                m68k_get_reg(NULL, M68K_REG_PC));
        if (!ng->rom || o + 2 > ng->rom_size) return 0xFFFF;
        return (ng->rom[o]<<8)|ng->rom[o+1];
    }
    if (a < 0x320000) {                     /* P1 control @0x300000 (mirror 0x01FF7E
                                               excludes TEST 0x300080-81):
                                               upper byte = edge(P1 buttons), lower = DSW */
        if ((a & 0x81) == 0x80) return 0xFFC0;                     /* TEST port + mirror */
        { uint8_t hi = (uint8_t)~ng->buttons[0];                   /* P1 */
          uint8_t lo = (uint8_t)~ng->buttons[1];                   /* P2 */
          return (hi<<8) | lo; }
    }
    if (a < 0x320002) {                     /* AUDIO_COIN */
        if (getenv("COINDBG") && (ng->buttons[0] & 0x400))
            { static int cd = 0; if (cd++ < 20) fprintf(stderr, "[coindbg] f=%d coin-active read a=%06x\n", ng->frame, a); }
        if (ng->dbg_vw == 3 && ng->frame > 7900) { fprintf(stderr, "COINRD f=%d a=%06x pc=%08x sl2=%02x\n", ng->frame, a, m68k_get_reg(NULL,M68K_REG_PC), ng->soundlatch2); if (++ng->sl1_trace > 40) ng->dbg_vw=0; }
        uint16_t lo = 0x3F;                 /* coins/service inactive */
        if (ng->buttons[0] & 0x400) lo &= ~0x01;
        if (ng->buttons[1] & 0x400) lo &= ~0x02;
        /* UPD4990A: TP toggles with serial clock; idle = slow RTC square
           wave.  Frame-based: the BIOS validates the TP period against Timer
           C counts (calendar check) and only passes with ~30-frame halves. */
        if (ng->mc_bitcnt || ng->mc_readphase) { if (ng->mc_tp) lo |= 0x40; }
        else if ((ng->frame / 30) & 1) lo |= 0x40;
        lo |= 0x80;
        /* reading AUDIO_COIN acks IRQ_SOUND (MAME audio_coin_r) */
        if (ng->irq_sound) { ng->irq_sound = 0; if (!ng->raster_pending) m68k_set_virq(2, 0); }
        return (ng->soundlatch2<<8) | lo;
    }
    if (a < 0x340002) {                     /* P2 */
        uint8_t b = (uint8_t)~ng->buttons[1];
        return (b<<8) | 0xFF;
    }
    if (a < 0x380002) {                     /* SYSTEM */
        uint16_t hi = 0x8F;
        if (ng->buttons[0] & 0x100) hi &= ~0x01;
        if (ng->buttons[0] & 0x200) hi &= ~0x02;
        if (ng->buttons[1] & 0x100) hi &= ~0x04;
        if (ng->buttons[1] & 0x200) hi &= ~0x08;
        return (hi<<8) | 0xFF;
    }
    if (a < 0x3E0000) {                     /* video registers R (mirror 0x1FFF8) */
        switch ((a & 0x6)>>1) {
        case 0: case 1: return ng->vram_readbuf;
        case 2: return ng->vram_modulo;
        default: {
            int vc = ng->scanline + 0x100;
            if (vc >= 0x200) vc -= LINES_PER_FRAME;
            return (uint16_t)((vc<<7) | (ng->auto_counter & 7));
        }}
    }
    if (a >= 0x3C8000 && a < 0x3D0000) {    /* zoom table read port */
        uint16_t w = ng->zoomy[(a - 0x3C8000) >> 1];
        return (w<<8)|w;
    }
    if (a < 0x400000) return 0xFFFF;
    if (a < 0x420000) return ng->cram[((a>>1)&0xFFF) | ng->cram_bank];
    if (a < 0xC00000)                       /* JEIDA memcard: 0xFF00|data when unlocked */
        return (!ng->mc_lock1 && ng->mc_unlock2 && ng->mc_regsel)
             ? (0xFF00 | ng->mc_data[(a >> 1) & 0x7FF]) : 0xFFFF;
    if (a < 0xD00000) {
        uint32_t o = a & 0x1FFFF;
        if (!ng->bios || o+2 > ng->bios_size) return 0xFFFF;
        if (ng->dbg_z80==4 && o >= 0x1F800 && o < 0x1F80C)
            fprintf(stderr, "TMPLRD a=%06x o=%05x -> %02x%02x pc=%08x\n", a, o, ng->bios[o], ng->bios[o+1], m68k_get_reg(NULL,M68K_REG_PC));
        return (ng->bios[o]<<8)|ng->bios[o+1];
    }
    { uint32_t o = a & 0xFFFF;
      /* kof97-family boot check reads the save marker ($D00100) while SRAM is
         CE-locked; deselected SRAM floats low. Scoped to the marker word so
         BIOS header/counter reads (done unlocked) are unaffected. */
      if (!ng->sram_unlocked && o >= 0x100 && o <= 0x103) return 0x0000;
      if (f_sramrd() && (o == 0x46 || o == 0x124 || o == 0x128))
          fprintf(stderr, "SRAMRD f=%d o=%04x v=%04x pc=%08x\n", ng->frame, o,
              (unsigned)((ng->sram[o]<<8)|ng->sram[o+1]), m68k_get_reg(NULL,M68K_REG_PC));
      return (ng->sram[o]<<8)|ng->sram[o+1]; }
}

static void w16(neogeo *ng, uint32_t a, uint16_t d, uint16_t mem_mask)
{
    a &= 0xFFFFFF;
    if (ng->cart_prot == 1 && (a & ~1u) == 0x20AAAA) {   /* kof98 protection trigger */
        uint16_t w = (mem_mask == 0xFFFF) ? d : (uint16_t)(((mem_mask & 0xFF00) ? (d & 0xFF00) : (ng->kof98_trig & 0xFF00)) |
                                                           ((mem_mask & 0x00FF) ? (d & 0x00FF) : (ng->kof98_trig & 0x00FF)));
        ng->kof98_trig = w;
        if (w == 0x0090) ng->kof98_state = 1;
        else if (w == 0x00f0) ng->kof98_state = 2;
        if (f_protdbg()) fprintf(stderr, "PROTW f=%d d=%04x m=%04x st=%d pc=%08x\n",
            ng->frame, w, mem_mask, ng->kof98_state, m68k_get_reg(NULL, M68K_REG_PC));
        return;
    }
    if (ng->cart_prot >= 2 && ng->cart_prot <= 7) {          /* SMA write_bankprot, port per cart type */
        int bs = 0;
        if (f_smaprot() && a >= 0x2FE000 && a <= 0x2FFFFF)
            fprintf(stderr, "SMAPW f=%d a=%06x d=%04x m=%04x pc=%08x\n",
                ng->frame, a, d, mem_mask, m68k_get_reg(NULL, M68K_REG_PC));
        switch (ng->cart_prot) {
            case 2: bs = (a >= 0x2FFFF0 && a <= 0x2FFFF1); break;   /* kof99 */
            case 3: case 4: bs = (a >= 0x2FFFC0 && a <= 0x2FFFC1); break;  /* garou/garouh */
            case 5: case 6: bs = (a >= 0x2FFFE4 && a <= 0x2FFFE5); break;  /* mslug3/3a */
            case 7: bs = (a >= 0x2FFFEC && a <= 0x2FFFED); break;   /* kof2000 */
            default: break;
        }
        if (bs) {
            ng->prg_bank_base = sma_bank(ng->cart_prot, d);
            if (f_bankdbg()) fprintf(stderr, "SMABANK f=%d d=%04x -> base=%08x pc=%08x\n",
                ng->frame, d, ng->prg_bank_base, m68k_get_reg(NULL, M68K_REG_PC));
            return;
        }
    }
    if (ng->cart_prot == 9 && a >= 0x2FFFF0 && a <= 0x2FFFFF) {   /* CTHD2K3 write_bankprot */
        static const int cb[8] = { 1,0,1,0,1,0,3,2 };
        ng->prg_bank_base = (uint32_t)(cb[d & 7] + 1) * 0x100000;
        if (f_bankdbg()) fprintf(stderr, "CTHDBANK f=%d d=%04x -> base=%08x pc=%08x\n",
            ng->frame, d, ng->prg_bank_base, m68k_get_reg(NULL, M68K_REG_PC));
        return;
    }
    if (ng->cart_prot == 12 && a >= 0x200000 && a <= 0x201FFF) { /* JOCKEYGP cart RAM */
        uint32_t o = (a >> 1) & 0xFFF;
        ng->jgram[o] = (uint16_t)((ng->jgram[o] & ~mem_mask) | (d & mem_mask));
        return;
    }
    if (ng->cart_prot == 8 && a >= 0x2FE000 && a <= 0x2FFFFF) {   /* PVC protection_w + bankprot */
        uint32_t off = (a >> 1) & 0xFFF;
        ng->pvc_ram[off] = (uint16_t)((ng->pvc_ram[off] & ~mem_mask) | (d & mem_mask));
        if (off == 0xff0) pvc_unpack_color(ng);
        else if (off >= 0xff4 && off <= 0xff5) pvc_pack_color(ng);
        if (off >= 0xff8) ng->prg_bank_base = pvc_bank_base(ng);
        return;
    }
    if (ng->cart_prot == 13 && a >= 0x2FE000 && a <= 0x2FFFFF) {  /* kof2k3bl_prot kof2003_w */
        uint32_t off = (a >> 1) & 0xFFF;
        uint16_t *pr = &ng->pvc_ram[off];
        if (mem_mask & 0xFF00) *pr = (uint16_t)((*pr & 0x00FF) | (d & 0xFF00));
        if (mem_mask & 0x00FF) *pr = (uint16_t)((*pr & 0xFF00) | (d & 0x00FF));
        if (off == 0xFF0/2 || off == 0xFF2/2) {
            uint8_t *cr = (uint8_t *)ng->pvc_ram;
            uint8_t prt = cr[0x1FF2 ^ 1];
            ng->prg_bank_base = 0x100000 + ((uint32_t)(cr[0x1FF3 ^ 1] << 16) | (cr[0x1FF2 ^ 1] << 8) | cr[0x1FF1 ^ 1]);
            cr[0x1FF0 ^ 1] = 0xa0;
            cr[0x1FF1 ^ 1] &= 0xfe;
            cr[0x1FF3 ^ 1] &= 0x7f;
            ng->k3bl_overlay = (uint16_t)((prt & 0x00FF) | (ng->k3bl_overlay & 0xFF00));
        }
        return;
    }
    if (a >= 0x2FFFF0 && a <= 0x2FFFFF) {   /* MAME write_banksel (PRO-C/LOROM) */
        int bank = (mem_mask & 0xFF00) && (mem_mask & 0x00FF) ? (d & 7)
                 : (mem_mask & 0xFF00) ? ((d >> 8) & 7) : (d & 7);
        if ((uint32_t)(bank + 1) * 0x100000 >= ng->rom_size) bank = 0;
        ng->prg_bank_base = (uint32_t)(bank + 1) * 0x100000;
        if (f_bankdbg()) fprintf(stderr, "BANKW f=%d a=%06x d=%04x m=%04x -> base=%08x pc=%08x\n",
            ng->frame, a, d, mem_mask, ng->prg_bank_base, m68k_get_reg(NULL, M68K_REG_PC));
        return;
    }
    if (a < 0x100000) return;
    if (a < 0x200000) {
        { static uint32_t tgt = 0xFFFFFFFF; static int ini = 0;
          if (!ini) { const char *e = getenv("RAMWA"); tgt = e ? (uint32_t)strtoul(e, 0, 16) : 0xFFFFFFFF; ini = 1; }
          if ((ng->dbg_z80 == 5 || f_ramwa()) && (a & 0xFFFFu) == (tgt & 0xFFFFu) &&
              (ng->frame > 3000 || f_ramwaf())) {
              fprintf(stderr, "RAMWA f=%d a=%06x d=%04x m=%04x pc=%08x\n", ng->frame, a, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC));
              if (++ng->cramw > 60) ng->dbg_z80 = 0; } }
        uint32_t o = (a & 0xFFFF)>>1;
        if (f_scrw() && (a & 0xFFFF) >= 0x0100 && (a & 0xFFFF) <= 0x01FE && ng->frame >= 3600 && ng->frame < 4300)
        { static int done98=0; if(!done98){done98=1; fprintf(stderr,"AT4101 182=%08x 100=%08x w108=%04x w114=%04x w10a=%04x\n", m68k_read_memory_32(0x100182), m68k_read_memory_32(0x100100), m68k_read_memory_16(0x100108), m68k_read_memory_16(0x100114), m68k_read_memory_16(0x10010A)); }
            fprintf(stderr, "SCRW a=%06x d=%04x m=%04x pc=%08x f=%d\n", a, d, mem_mask,
                    m68k_get_reg(NULL,M68K_REG_PC), ng->frame); }
        if (f_scrw() && a == 0x10FFFE && ng->frame > 4100 && ng->frame < 4300) {
            unsigned sp = m68k_get_reg(NULL, M68K_REG_SP);
            fprintf(stderr, "CLRSTACK sp=%08x ret:", sp);
            for (int z=0; z<8; z++) fprintf(stderr, " %08x", m68k_read_memory_32(sp + z*4));
            fprintf(stderr, "\n");
        }
        if (f_fd98w() && (a & 0xFFFF) >= 0xFD98 && (a & 0xFFFF) <= 0xFD99)
            fprintf(stderr, "FD98W a=%06x d=%04x m=%04x pc=%08x f=%d\n", a, d, mem_mask,
                    m68k_get_reg(NULL,M68K_REG_PC), ng->frame);
        if (ng->dbg_vw && (a & 0xFFFFFFFE) == 0x100E34) {
            unsigned sp = m68k_get_reg(NULL, M68K_REG_SP);
            fprintf(stderr, "E35W a=%06x d=%04x mask=%04x pc=%08x ret:", a, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC));
            for (int z=0; z<6; z++) fprintf(stderr, " %08x", m68k_read_memory_32(sp + z*4));
            fprintf(stderr, "\n");
        }
        uint32_t off = a & 0xFFFF;
        if (off & 1) {   /* odd word access spans two RAM words (MAME splits to bytes) */
            if (mem_mask & 0xFF00) ng->workram[o] = (ng->workram[o]&0xFF00)|(d>>8);
            if (mem_mask & 0x00FF && o + 1 < 0x8000) ng->workram[o+1] = (ng->workram[o+1]&0x00FF)|(d<<8);
        } else {
            if (mem_mask & 0xFF00) ng->workram[o] = (ng->workram[o]&0x00FF)|(d&0xFF00);
            if (mem_mask & 0x00FF) ng->workram[o] = (ng->workram[o]&0xFF00)|(d&0x00FF);
        }
        if (f_wp0() && o <= 1 && (ng->workram[0] == 0x0064 || ng->workram[1] == 0x00b4)) {
            static int wpc = 0;
            if (!wpc) { wpc = 1;
                fprintf(stderr, "WP0 f=%d a=%06x d=%04x m=%04x pc=%08x w0=%04x w1=%04x\n", ng->frame, a, d,
                    mem_mask, m68k_get_reg(NULL,M68K_REG_PC), ng->workram[0], ng->workram[1]); } }
        return;
    }
    if (a < 0x340000) {                     /* audio command: MAME map is
        0x320000.w mirror 0x01fffe — EVEN addresses only, low byte only.
        Odd writes (e.g. BIOS coin $300001) are watchdog/no-op, NOT latches:
        latching them storms the Z80 NMI handler and overflows its stack. */
        if (a & 1) {
            /* watchdog feed: MAME map(0x300001,0x300001).mirror(0x01fffe) */
            ng->wd_last = ng->mc_cycles; ng->wd_armed = 1;
            if (f_wdtr()) fprintf(stderr, "WDFEED f=%d a=%06x pc=%08x\n", ng->frame, a, m68k_get_reg(NULL,M68K_REG_PC));
        }
        if ((a & 1) == 0 && (mem_mask & 0xFF00)) {
            /* even-address byte lane = the command byte (Musashi: even byte
               write presents data<<8 with mask 0xFF00; word write d>>8;
               odd-byte writes (mask 0x00FF: watchdog $300001 etc.) must NOT
               latch — they carry port data, not sound commands). */
            static int crtr = -2;
            if (crtr == -2) crtr = getenv("CRAMTRACE") ? 1 : 0;
            if (crtr && a >= 0x200000 && a < 0x210000 && ng->frame < 400)
                fprintf(stderr, "SLABSORB f=%d a=%06x d=%04x m=%04x pc=%08x\n",
                    ng->frame, a, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC));
            uint8_t v = (uint8_t)(d >> 8);
            ng->soundlatch1 = v;
            /* MAME generic_latch_8: data always overwritten, but data_pending
               (and hence the NMI line) only rises when idle. */
            if (!ng->z80_nmi_pend) ng->z80_nmi_pend = 1;
            /* Assert the NMI the instant the command lands (level semantics).
               The frame/chunk-end poll alone loses commands whenever the Z80
               sits in its di heartbeat: the heartbeat's in(00) clears
               data_pending before the poll ever fires, so the command sits
               in the latch with no NMI ever taken (samsho2 cmd 2d: game BGM
               start command silently swallowed). mz80int_dispatch keeps the
               NMI line pending until the core actually takes it, so an
               immediate assert here is race-free even mid-heartbeat. */
            if (ng->z80_nmi_en) {
                /* One soundlatch write == one NMI pulse (real HC259/latch
                   hardware: every write re-edges the line regardless of
                   whether the Z80 handler ever read the latch back).
                   The old !z80_nmi_asserted level guard dead-locked the
                   line high after the first unread command (samsho2
                   cmd 07/2d at f=765/779 never fired). */
                z80w_nmi(ng->z);
                ng->nmi_cnt++;
                ng->z80_nmi_asserted = 1;
            }
            if (ng->dbg_z80==3) fprintf(stderr, "SL1W f=%llu %02x a=%06x d=%04x m=%04x pc=%08x zpc=%04x pend=%d en=%d asrt=%d\n", (unsigned long long)ng->frame, v, a, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC), z80w_pc(ng->z), ng->z80_nmi_pend, ng->z80_nmi_en, ng->z80_nmi_asserted);
            ng->sl1_cnt++;
        }
        return;
    }
    if (a < 0x380002) { ng->ctrlsel = d & 0x3F; return; }
    if (a >= 0x380050 && a < 0x380052) {    /* 4990A serial in: bit0 DI, bit1 CKI, bit2 CS */
        int v = d & 0xFF;
        int cki = v & 1, cs = (v >> 2) & 1, di = (v >> 1) & 1;   /* NOTE: BIOS uses bit1=CKI? see below */
        cki = (v >> 1) & 1; di = v & 1; cs = (v >> 2) & 1;
        if (cki && !ng->mc_cki) {           /* rising edge */
            ng->mc_tp ^= 1;
            if (!ng->mc_readphase) {
                if (ng->mc_bitcnt < 4) ng->mc_cmd = (ng->mc_cmd >> 1) | (di << 3);
                ng->mc_bitcnt++;
            } else {
                ng->mc_resp <<= 1;          /* next bit out MSB-first */
                ng->mc_dobit = (ng->mc_resp >> 7) & 1;
            }
        }
        if (cs && !ng->mc_cs) {             /* CS rising: latch command, enter read phase */
            ng->mc_resp = mc4990_response(ng, ng->mc_cmd);
            ng->mc_dobit = (ng->mc_resp >> 7) & 1;
            ng->mc_readphase = 1;
            ng->mc_bitcnt = 0; ng->mc_cmd = 0;
        } else if (!cs && ng->mc_cs) {
            ng->mc_readphase = 0;
        }
        ng->mc_cki = cki; ng->mc_cs = cs;
        return;
    }
    if (a < 0x3C0000) {                     /* systemlatch = HC259 write_a3:
                                               channel = addr bits 1-3, value = addr bit 4 */
        int off = (a >> 1) & 0xF;
        int ch  = off & 7, v = (off >> 3) & 1;
        switch (ch) {
        case 0:
            if (getenv("SHDWDBG")) fprintf(stderr, "SHDW v=%d f=%llu pc=%08x\n", v, (unsigned long long)ng->frame, m68k_get_reg(NULL,M68K_REG_PC));
            ng->screen_shadow = v; break;   /* MAME: q_out_cb<0> = set_screen_shadow */
        case 1: if (f_latch_tr()) fprintf(stderr, "LATCH ch1 v=%d pc=%08x f=%d\n", v, m68k_get_reg(NULL,M68K_REG_PC), ng->frame); ng->vector_bank = v; break;
        case 2: ng->mc_lock1   = v; break;
        case 3: ng->mc_unlock2 = v; break;
        case 4: ng->mc_regsel  = v; break;
        case 5: {
            int want = f_nocartaudio() ? 0 : v;
            if (f_latch_tr()) fprintf(stderr, "LATCH ch5 v=%d pc=%08x f=%d\n", v, m68k_get_reg(NULL,M68K_REG_PC), ng->frame);
            if (want != ng->cart_audio) {
                ng->cart_audio = want;
                if (!want) ng->ch5_bios_pend = 0;
                if (!want && !ng->m1_booted && !ng->m1_view && ng->snd && ng->sm1
                    && f_m1boot1()) {
                    /* BIOS self-test pulse just ended (0->1->0 within BIOS): boot cart m1
                       NOW so its table init completes BEFORE the test command arrives;
                       the pulse itself was answered by sm1 (no reset at 0->1 in BIOS). */
                    z80w_reset(ng->z);
                    ng->z80_nmi_asserted = 0;
                    ng->z80_nmi_pend = 0;   /* discard the BIOS pulse command: m1 tables not
                                               ready yet; BIOS resends after ack timeout and
                                               m1 (fully booted by then) answers the resend */
                    ng->m1_view = 1;
                    if (f_latch_tr()) fprintf(stderr, "M1BOOT f=%d\n", ng->frame);
                }
                /* Asymmetric reset (pnyaa/rotd evidence):
                   - 0->1 (cart m1 becomes visible): reset the Z80 so m1
                     boots from its own vectors with clean RAM. Do NOT clear
                     z80_nmi_pend — the HC259 latch line survives; the 68K
                     may have written the command BEFORE enabling (mslug4).
                   - 1->0 (back to sm1 view): NO reset. pnyaa toggles
                     1->0 one instruction after enabling; resetting there
                     strands the Z80 in sm1 BIOS whose stack usage overflows
                     into 0xFFFD (m1 SP assumptions) and the handshake dies
                     at echo 0x01 forever. */
                if (want) {
                    /* MAME parity: the Z80 is NEVER reset by the BIOS-phase
                       ch5 pulse (uni-bios self-test toggles 0->1->0 within
                       c11eda..c11ee4 expecting SM1 to answer; resetting
                       there hands the test to cart m1 whose echo-only first
                       NMI deadlocks the BIOS: pnyaa/kof2003/kf2k3pl).
                       BIOS-phase enable now ALSO keeps the SM1 view: cart m1
                       must not see any BIOS-phase command or its track
                       state machine loads garbage (samsho2: cmd residue
                       fe3c=0x0d deadlocks the m1 main loop at 0x24b0 and the
                       game's own cmd 2d is never dispatched). Handoff to a
                       freshly reset cart m1 happens when the 68K PC leaves
                       the BIOS windows — i.e. the game has actually started.
                       Game-side enables (68K already outside BIOS, mslug3
                       f234@0x10fa6) reset+boot immediately as before. */
                    unsigned long pc = (unsigned long)m68k_get_reg(NULL, M68K_REG_PC);
                    int in_bios = (pc >= 0xC00000UL && pc < 0xD00000UL) || pc >= 0xFF8000UL;
                    if (!in_bios) { ng->cart_audio = 1;
                        z80w_reset(ng->z); ng->z80_nmi_asserted = 0; ng->m1_booted = 1; ng->m1_view = 1;
                    } else { ng->cart_audio = 0;   /* keep SM1 visible through BIOS phase */
                        ng->ch5_bios_pend = 1; ng->ch5_on_frame = ng->frame; }
                }
            }
            if (f_latch_tr()) fprintf(stderr, "LATCH ch5 now=%d f=%d\n", ng->cart_audio, ng->frame);
            break; }
        case 6: ng->sram_unlocked = v; break;  /* SRAM unlock (CE gate) */
        case 7: { /* palette bank DELIBERATELY IGNORED: MAME honors ch7 per-scanline,
             but our full-frame render picks the wrong bank for raster-bank games
             (bstars2 writes anim palettes to bank1 yet renders bank0: honoring
             bank regressed it 0.06->0.45). par48 semantics: all CRAM writes land
             in 0-0xFFF, last writer wins. LATCH7 env traces the real latch state. */
            static int l7=-2; if (l7==-2) l7 = getenv("LATCH7") ? 1 : 0;
            if (l7) fprintf(stderr, "LATCH7 v=%d f=%d\n", v, ng->frame); } break;
        default: break;
        }
        return;
    }
    if (a < 0x3E0000) {
        if (mem_mask == 0x00FF) return;
        if (mem_mask == 0xFF00) d = (d & 0xFF00) | (d >> 8);
        switch ((a & 0xE)>>1) {      /* MAME: map(0x3c0000,0x3c000f).mirror(0x1fff0) —
                                        bits 4+ mirror away, so $3C0010 (OUTPUT_CTRL on
                                        some docs) is actually a VLOAD mirror write. */
        case 0:
            { static int q=0, tot=0; tot++; if (q<8 && (tot%997)==0) fprintf(stderr, "VOFFS tot=%d last d=%04x pc=%08x\n", tot, d, m68k_get_reg(NULL,M68K_REG_PC)); q++; }
            ng->vram_offset = d & 0xFFFF;   /* MAME: full 16-bit word offset into 0x10000 words */
            ng->vram_readbuf = ng->vram[ng->vram_offset];   /* read happens right away */
            break;
        case 1:
            if (1) { static int k=0; if (ng->vram_offset>=0xC000 && ng->vram_offset<0xC800 && ++k<=20) fprintf(stderr, "VW off=%04x val=%04x\n", ng->vram_offset, d); }
            if (f_vwdist()) { static unsigned long hist[64]; static int done=0;
                hist[(ng->vram_offset>>10)&63]++;
                if (done++ == 5000) { for (int q=0;q<64;q++) if (hist[q]) fprintf(stderr, "VWDIST %04x-%04x: %lu\n", q<<10, (q<<10)|0x3FF, hist[q]); done=-1; } }
            ng->vram[ng->vram_offset] = d;
            if (f_vload()) ng->vload_count++;
            ng->vram_offset = (ng->vram_offset + ng->vram_modulo) & 0xFFFF;
            ng->vram_readbuf = ng->vram[ng->vram_offset];   /* prefetch */
            break;
        case 2: ng->vram_modulo = d; break;
        case 3:
            if (f_vc()) fprintf(stderr, "VC w=%04x pc=%08x\n", d, m68k_get_reg(NULL,M68K_REG_PC));
            ng->auto_speed = d >> 8;
            ng->auto_disabled = (d >> 3) & 1;
            ng->irq_enable = (d >> 4) & 0xF;   /* IRQ2CTRL bits4-7 */
            ng->vblank_enable = d & 1;
            break;
        case 4: ng->display_counter = (ng->display_counter & 0x0000FFFF) | ((uint32_t)(d & 0xFF) << 16); break;
        case 5:
            ng->display_counter = (ng->display_counter & 0xFFFF0000) | d;
            ng->raster_armed = 1;              /* MAME: any lsb write arms timer */
            break;
        case 6:
            if (d & 2) { ng->raster_pending = 0; ng->irq_line = 0; m68k_set_virq(2, 0); }
            if (d & 4) { m68k_set_virq(1, 0); }                          /* vblank */
            if (d & 1) { }                                                /* irq3 (cart) */
            break;
        case 7: break;
        }
        return;
    }
    if (a < 0x420000) {                     /* CRAM */
        uint32_t o = ((a>>1)&0xFFF) | ng->cram_bank;   /* MAME: offset + palette bank */
        /* MAME renders each scanline with the bank set during vblank (games
           write bank0 palette, render, then flip to bank1 at frame end for
           the NEXT frame's palette prep).  Full-frame render must snapshot
           the bank at the frame's first CRAM write, not read it at frame end. */
        if (ng->bank_snapf != ng->frame) {
            ng->bank_snapf = ng->frame;
            ng->cram_bank_render = ng->cram_bank;
            { static int sb=-2; if (sb==-2) sb = getenv("SNAPDBG") ? 1 : 0;
              if (sb && ng->frame > 11990) fprintf(stderr, "SNAP f=%llu bank=%x\n", (unsigned long long)ng->frame, ng->cram_bank_render); }
        }
        static int crtr2 = -2;
        if (crtr2 == -2) crtr2 = getenv("CRAMTRACE") ? 1 : 0;
        if (crtr2 && ng->frame < 2000 && (m68k_get_reg(NULL,M68K_REG_PC) < 0xC00000 || getenv("CRAMT2")))
            fprintf(stderr, "CRAMT f=%d a=%06x o=%04x d=%04x m=%04x pc=%08x\n",
                ng->frame, a, o, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC));
        if (ng->dbg_z80==4 && ng->cramw<4000 && m68k_get_reg(NULL,M68K_REG_PC) < 0xC00000) { ng->cramw++;
            fprintf(stderr, "CRAMW a=%06x o=%04x d=%04x m=%04x pc=%08x\n", a, o, d, mem_mask, m68k_get_reg(NULL,M68K_REG_PC)); }
        if (mem_mask & 0xFF00) ng->cram[o] = (ng->cram[o]&0x00FF)|(d&0xFF00);
        if (mem_mask & 0x00FF) ng->cram[o] = (ng->cram[o]&0xFF00)|(d&0x00FF);
        return;
    }
    if (a < 0xC00000) {                     /* JEIDA memcard write */
        if (!ng->mc_lock1 && ng->mc_unlock2)
            ng->mc_data[(a >> 1) & 0x7FF] = d & 0xFF;
        return;
    }
    if (a < 0xC40000) {                     /* VRAM direct window (MAME: map(0xC00000,0xC3FFFF)
                                               writeonly16 -> vram bytes, word index = (a&0x1FFFF)>>1) */
        uint32_t o = (a & 0x1FFFF) >> 1;
        if (o < 0x10000) {
            if (mem_mask & 0xFF00) ng->vram[o] = (ng->vram[o] & 0x00FF) | (d & 0xFF00);
            if (mem_mask & 0x00FF) ng->vram[o] = (ng->vram[o] & 0xFF00) | (d & 0x00FF);
        }
        return;
    }
    if (a < 0x420000) {
        uint32_t o = (a & 0x1FFFF)>>1;
        if (mem_mask & 0xFF00) ng->cram[o] = (ng->cram[o]&0x00FF)|(d&0xFF00);
        if (mem_mask & 0x00FF) ng->cram[o] = (ng->cram[o]&0xFF00)|(d&0x00FF);
        return;
    }
    if (a < 0xD00000) return;
    { uint32_t o = a & 0xFFFF;
      if (f_sramw() && ((o >= 0xFE && o <= 0x130) || o == 0x44 || o == 0x46 || o == 0x124))
          fprintf(stderr, "SRAMW f=%d o=%04x d=%04x m=%04x pc=%08x\n", ng->frame, o, d, mem_mask,
              m68k_get_reg(NULL, M68K_REG_PC));
      if (mem_mask & 0xFF00) ng->sram[o] = d>>8;
      if (mem_mask & 0x00FF) ng->sram[o+1] = d&0xFF; }
}

unsigned int m68k_read_memory_8(unsigned int a)
{
    uint16_t w = r16(g_ng, a & ~1u);
    uint8_t v = (a & 1) ? (w & 0xFF) : (w >> 8);
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFF) == 0xE2FE && a >= 0x100000 && a < 0x200000)
        fprintf(stderr, "FLAGRD a=%06x %02x pc=%08x\n", a, v, m68k_get_reg(NULL,M68K_REG_PC));
    return v;
}
unsigned int m68k_read_memory_16(unsigned int a)
{
    uint16_t w = r16(g_ng, a);
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFFFF) >= 0x10E2F8 && (a & 0xFFFFFF) <= 0x10E300)
        fprintf(stderr, "W16RD a=%06x %04x pc=%08x\n", a, w, m68k_get_reg(NULL,M68K_REG_PC));
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFFFF) == 0x10E2FE)
        fprintf(stderr, "FLAGRD16 %04x pc=%08x\n", w, m68k_get_reg(NULL,M68K_REG_PC));
    return w;
}
unsigned int m68k_read_memory_32(unsigned int a)
{ return (r16(g_ng,a)<<16) | r16(g_ng,a+2); }
void m68k_write_memory_8(unsigned int a, unsigned int v)
{
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFFFF) == 0x10E2FE && v==0)
        fprintf(stderr, "FLAGCLR pc=%08x sp=%08x\n", m68k_get_reg(NULL,M68K_REG_PC), m68k_get_reg(NULL,M68K_REG_A7));
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFFFF) == 0x10E36)
        fprintf(stderr, "E36W %02x pc=%08x\n", v, m68k_get_reg(NULL,M68K_REG_PC));
    if (g_ng && g_ng->dbg_z80==4 && (a & 0xFFFFFF) == 0x10E2FE)
        fprintf(stderr, "FLAGW %02x pc=%08x f=%d\n", v, m68k_get_reg(NULL,M68K_REG_PC), g_ng->frame);
    uint32_t wa = a & ~1u;
    uint16_t old = r16(g_ng, wa);
    uint16_t nd = (a & 1) ? (old & 0xFF00)|(v&0xFF) : (old & 0x00FF)|((v&0xFF)<<8);
    w16(g_ng, wa, nd, (a & 1) ? 0x00FF : 0xFF00);
}
void m68k_write_memory_16(unsigned int a, unsigned int v) { w16(g_ng, a, (uint16_t)v, 0xFFFF); }
void m68k_write_memory_32(unsigned int a, unsigned int v)
{ w16(g_ng, a, (uint16_t)(v>>16), 0xFFFF); w16(g_ng, a+2, (uint16_t)v, 0xFFFF); }
unsigned int m68k_read_disassembler_8(unsigned int a)  { return m68k_read_memory_8(a); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return m68k_read_memory_16(a); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return m68k_read_memory_32(a); }

/* ---- rom loading ---- */
static void wordswap(uint8_t *d, uint32_t n)
{
    for (uint32_t i = 0; i + 1 < n; i += 2) { uint8_t t = d[i]; d[i] = d[i+1]; d[i+1] = t; }
}

/* ---- MAME neogeo_swlist.xml driven exact region loading ---- */
typedef struct { char fname[128]; uint32_t offset, size; int cont, swap, byte, l32, fill; uint8_t fval; } rg_ent;

static const char *sw_attr(const char *p, const char *end, const char *attr, char *out, size_t osz)
{
    char pat[32]; snprintf(pat, sizeof pat, "%s=\"", attr);
    const char *a = p - 1;
    while ((a = strstr(a + 1, pat)) != NULL && a < end) {
        if (a == p || a[-1] == ' ' || a[-1] == '\t') break;
    }
    if (!a || a >= end) return NULL;
    a += strlen(pat);
    const char *b = strchr(a, '"');
    if (!b || b >= end) return NULL;
    size_t n = (size_t)(b - a); if (n >= osz) n = osz - 1;
    memcpy(out, a, n); out[n] = 0;
    return out;
}

static const char *sw_find(const char *xml, const char *sw)
{
    char pat[160]; snprintf(pat, sizeof pat, "<software name=\"%s\"", sw);
    const char *p = xml;
    size_t pl = strlen(pat);
    while ((p = strstr(p, pat)) != NULL) {
        char c = p[pl];
        if (c == ' ' || c == '>' || c == '\t' || c == '\n' || c == '/') return p;
        p += pl;
    }
    return NULL;
}

/* bounded substring search within [a,end) */
static int sw_has(const char *a, const char *end, const char *pat)
{
    size_t pl = strlen(pat);
    for (const char *p = a; p + pl <= end; p++)
        if (!memcmp(p, pat, pl)) return 1;
    return 0;
}

static int sw_area(const char *swxml, const char *area, rg_ent *ents, int maxn, int *outn, uint32_t *areasz)
{
    char pat[64]; snprintf(pat, sizeof pat, "<dataarea name=\"%s\"", area);
    const char *a = strstr(swxml, pat);
    if (!a) return -1;
    const char *ae = strstr(a, "</dataarea>");
    if (!ae) return -1;
    *outn = 0; *areasz = 0;
    {   char tmp[64]; const char *tag0 = strstr(a, ">");
        if (tag0 && tag0 < ae && sw_attr(a, tag0, "size", tmp, sizeof tmp))
            *areasz = (uint32_t)strtoul(tmp, NULL, 16);
    }
    const char *p = a;
    while (*outn < maxn && (p = strstr(p, "<rom ")) != NULL && p < ae) {
        p += 5;
        const char *tagend = strstr(p, "/>");
        if (!tagend || tagend > ae) break;
        rg_ent *e = &ents[*outn]; memset(e, 0, sizeof *e);
        char tmp[64];
        sw_attr(p, tagend, "name", e->fname, sizeof e->fname);
        if (sw_attr(p, tagend, "offset", tmp, sizeof tmp)) e->offset = (uint32_t)strtoul(tmp, NULL, 16);
        if (sw_attr(p, tagend, "size", tmp, sizeof tmp))   e->size   = (uint32_t)strtoul(tmp, NULL, 16);
        if (sw_attr(p, tagend, "loadflag", tmp, sizeof tmp)) {
            e->cont = strstr(tmp, "continue") ? 1 : 0;
            e->swap = strstr(tmp, "word_swap") ? 1 : 0;
            e->byte = strstr(tmp, "load16_byte") ? 1 : 0;
            e->l32  = strstr(tmp, "load32_word") ? 1 : 0;
        }
        if (!e->fname[0] && sw_attr(p, tagend, "value", tmp, sizeof tmp)) {
            e->fill = 1; e->fval = (uint8_t)strtoul(tmp, NULL, 0);
        }
        (*outn)++;
    }
    return *outn ? 0 : -1;
}

static uint8_t *sw_build(const char *zip, const rg_ent *e, int n, uint32_t cap, uint32_t areasz, uint32_t *outsize)
{
    uint32_t max = areasz;
    for (int i = 0; i < n; i++) {
        uint32_t span = (e[i].byte || e[i].l32) ? 2 * e[i].size : e[i].size;
        if (e[i].offset + span > max) max = e[i].offset + span;
    }
    if (!max || max > cap) return NULL;
    /* slack 1MB: bootleg PX decrypt memcpy rounds read past the last 1MB chunk */
    uint8_t *buf = calloc(1, max + 0x100000);
    if (!buf) return NULL;
    uint8_t *file = 0; uint32_t fsz = 0, pos = 0; int fswap = 0, fbyte = 0, f32 = 0;
    for (int i = 0; i < n; i++) {
        if (e[i].fill) {   /* <rom value="0xff" .../> fill entry */
            if (e[i].offset + e[i].size <= max) memset(buf + e[i].offset, e[i].fval, e[i].size);
            free(file); file = 0;
            continue;
        }
        if (e[i].fname[0]) {
            free(file); file = 0;
            if (rom_load_zip(zip, e[i].fname, &file, &fsz) != 0) {
                /* bootleg-tolerant: some sets ship the same ROM with a board
                   prefix (pnyaa "267-m1.m1" vs swlist "m1.m1") — retry by
                   name suffix before giving up */
                if (rom_load_zip_suffix(zip, e[i].fname, &file, &fsz) != 0) { file = 0; continue; }
            }
            pos = 0; fswap = e[i].swap; fbyte = e[i].byte; f32 = e[i].l32;
        }
        if (!file) continue;
        uint32_t len = e[i].size ? e[i].size : (fsz - pos);
        if (pos + len > fsz) len = fsz - pos;
        if (fbyte) {
            for (uint32_t k = 0; k < len && e[i].offset + 2*k < max; k++)
                buf[e[i].offset + 2*k] = file[pos + k];
        } else if (f32) {   /* load32_word[_swap]: file word w -> bus addr off+4w (word swapped) */
            for (uint32_t w = 0; 2*w + 1 < len; w++) {
                uint32_t dst = e[i].offset + 4*w;
                if (dst + 1 >= max) break;
                if (fswap) { buf[dst] = file[pos+2*w+1]; buf[dst+1] = file[pos+2*w]; }
                else       { buf[dst] = file[pos+2*w];   buf[dst+1] = file[pos+2*w+1]; }
            }
        } else if (e[i].offset + len <= max) {
            memcpy(buf + e[i].offset, file + pos, len);
            if (fswap) wordswap(buf + e[i].offset, len);
        }
        pos += len;
    }
    free(file);
    *outsize = max;
    return buf;
}

static int load_bios_pref(neogeo *ng, const char *dir, const char *pref)
{
    char p[1024];
    static const char *cands[] = { "uni-bios_3_2.rom", "asia-s3.rom", "sp-s3.sp1", "vs-bios.rom",
                                   "usa_2slt.bin", "NEO-GEO.ROM", NULL };
    const char *force = getenv("LINORAGX_BIOS");
    if (force && force[0]) snprintf(p, sizeof p, "%s", force);
    FILE *f = NULL; long n = 0;
    if (p[0]) { f = fopen(p, "rb"); if (f) { fseek(f,0,SEEK_END); n = ftell(f); fseek(f,0,SEEK_SET); fprintf(stderr, "[bios] %s\n", p); } }
    if (pref && !f) {
        snprintf(p, sizeof p, "%s/%s", dir, pref);
        f = fopen(p, "rb");
        if (f) { fseek(f,0,SEEK_END); n = ftell(f); fseek(f,0,SEEK_SET); fprintf(stderr, "[bios] %s (cart-pref)\n", p); }
    }
    for (int i = 0; cands[i] && !f; i++) {
        snprintf(p, sizeof p, "%s/%s", dir, cands[i]);
        f = fopen(p, "rb");
        if (f) { fseek(f,0,SEEK_END); n = ftell(f); fseek(f,0,SEEK_SET); fprintf(stderr, "[bios] %s\n", p); }
    }
    if (!f) return -1;
    if (n > 0x100000) n = 0x100000;
    ng->bios = calloc(1, n); ng->bios_size = n;
    if (fread(ng->bios,1,n,f) != (size_t)n) { fclose(f); return -2; }
    fclose(f);
    wordswap(ng->bios, n);                  /* MAME: ROM_LOAD16_WORD_SWAP_BIOS */
    static const char *sf[] = { "sfix.sfix", "NG-SFIX.ROM", NULL };
    for (int i = 0; sf[i]; i++) {
        snprintf(p, sizeof p, "%s/%s", dir, sf[i]);
        if ((f = fopen(p, "rb"))) {
            ng->sfix = calloc(1, 0x20000);
            fseek(f,0,SEEK_END); long m = ftell(f); fseek(f,0,SEEK_SET);
            if (m > 0x20000) m = 0x20000;
            if (fread(ng->sfix,1,m,f) != (size_t)m) {}
            fclose(f);
            break;
        }
    }
    {   static const char *zl[] = { "000-lo.lo", "NG-LO.ROM", NULL };
        for (int i = 0; zl[i]; i++) {
            snprintf(p, sizeof p, "%s/%s", dir, zl[i]);
            if ((f = fopen(p, "rb"))) {
                fseek(f,0,SEEK_END); long m = ftell(f); fseek(f,0,SEEK_SET);
                if (m > 0x20000) m = 0x20000;
                if (fread(ng->zoomy,1,m,f) != (size_t)m) {}
                fclose(f);
                ng->have_zoom = 1;
                break;
            }
        }
    }
    snprintf(p, sizeof p, "%s/sm1.sm1", dir);
    if ((f = fopen(p, "rb"))) {
        ng->sm1 = calloc(1, 0x20000);
        fseek(f,0,SEEK_END); long m = ftell(f); fseek(f,0,SEEK_SET);
        if (m > 0x20000) m = 0x20000;
        if (fread(ng->sm1,1,m,f) != (size_t)m) {}
        fclose(f);
    }
    return 0;
}
static int load_bios(neogeo *ng, const char *dir) { return load_bios_pref(ng, dir, NULL); }

/* KOF98 custom protection (MAME prot_kof98.cpp): decrypt first 1MB of PRG,
 * shift p2 down to 0x100000, capture header words for the protection port. */
static void kof98_decrypt(neogeo *ng)
{
    if (ng->rom_size < 0x600000) return;
    uint8_t *src = ng->rom;
    uint8_t *dst = calloc(1, 0x200000);
    if (!dst) return;
    static const uint32_t sec[8] = {0,0x100000,4,0x100004,0x10000a,0xa,0x10000e,0xe};
    static const uint32_t pos[4] = {0x000,0x004,0x00a,0x00e};
    memcpy(dst, src, 0x200000);
    for (uint32_t i = 0x800; i < 0x100000; i += 0x200) {
        for (int j = 0; j < 0x100; j += 0x10) {
            for (int k = 0; k < 16; k += 2) {
                memcpy(src+i+j+k,       dst+i+j+sec[k/2]+0x100, 2);
                memcpy(src+i+j+k+0x100, dst+i+j+sec[k/2],       2);
            }
            if (i >= 0x80000 && i < 0xc0000) {
                for (int k = 0; k < 4; k++) {
                    memcpy(src+i+j+pos[k],       dst+i+j+pos[k],       2);
                    memcpy(src+i+j+pos[k]+0x100, dst+i+j+pos[k]+0x100, 2);
                }
            } else if (i >= 0xc0000) {
                for (int k = 0; k < 4; k++) {
                    memcpy(src+i+j+pos[k],       dst+i+j+pos[k]+0x100, 2);
                    memcpy(src+i+j+pos[k]+0x100, dst+i+j+pos[k],       2);
                }
            }
        }
        memcpy(src+i+0x000, dst+i+0x000000, 2);
        memcpy(src+i+0x002, dst+i+0x100000, 2);
        memcpy(src+i+0x100, dst+i+0x000100, 2);
        memcpy(src+i+0x102, dst+i+0x100100, 2);
    }
    memmove(src + 0x100000, src + 0x200000, 0x400000);
    ng->kof98_default[0] = (uint16_t)((src[0x100] << 8) | src[0x101]);
    ng->kof98_default[1] = (uint16_t)((src[0x102] << 8) | src[0x103]);
    ng->cart_prot = 1;
    fprintf(stderr, "KOF98PROT def=%04x,%04x vec=%08x\n", ng->kof98_default[0], ng->kof98_default[1],
        (unsigned)((src[0]<<16)|src[2]));
    free(dst);
}

/* ---- SMA / PVC protection (ported from MAME prot_sma.cpp / prot_pvc.cpp) ----
 * Our region byte array is bus byte order (address A <-> byte A).  MAME's
 * 16-bit big-endian region stores native words, so MAME region byte r <->
 * bus byte r^1 and rom[BYTE_XOR_LE(x)] accesses bus byte x.  Translation
 * rules used below:
 *   MAME rom[BYTE_XOR_LE(x)] -> bus[x]
 *   MAME raw rom[x]          -> bus[x^1] (word-aligned block moves unchanged)
 *   MAME uint16_t* word view -> bus word via rdw/wrw
 * cart_prot: 2=kof99 3=garou 4=garouh 5=mslug3 6=mslug3a 7=kof2000 8=PVC
 */
static uint32_t bswp(uint32_t v, int n, const int *p)
{
    uint32_t r = 0;
    for (int k = 0; k < n; k++)
        if (v & (1u << p[k])) r |= 1u << (n - 1 - k);
    return r;
}
static uint16_t rdw(const uint8_t *b, uint32_t wi) { return (uint16_t)((b[2*wi]<<8)|b[2*wi+1]); }
static void wrw(uint8_t *b, uint32_t wi, uint16_t v) { b[2*wi]=(uint8_t)(v>>8); b[2*wi+1]=(uint8_t)v; }
static uint8_t *prot_buf(void) { static uint8_t *p; if (!p) p = calloc(1, 0x900000); return p; }

static void sma_dec_kof99(uint8_t *R)
{
    static const int dp[16]={13,7,3,0,9,4,5,6,1,12,8,14,10,11,2,15};
    static const int bp[10]={6,2,4,9,8,3,1,7,0,5};
    static const int fp[19]={18,11,6,14,17,16,5,8,10,12,0,4,3,2,7,9,15,13,1};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    static uint16_t buf[0x400];
    for (uint32_t i = 0; i < 0x300000; i += 0x400) {
        for (int j = 0; j < 0x400; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x400; j++) wrw(P, i+j, buf[bswp(j,10,bp)]);
    }
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x380000 + bswp(i,19,fp)));
}

static void sma_dec_garou(uint8_t *R)
{
    static const int dp[16]={13,12,14,10,8,2,3,1,5,9,11,4,15,0,6,7};
    static const int fp[19]={18,4,5,16,14,7,9,6,13,17,15,3,1,2,12,11,8,10,0};
    static const int bp[14]={9,4,8,3,13,6,2,7,0,12,1,11,10,5};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x388000 + bswp(i,19,fp)));
    static uint16_t buf[0x4000];
    for (uint32_t i = 0; i < 0x400000; i += 0x4000) {
        for (int j = 0; j < 0x4000; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x4000; j++) wrw(P, i+j, buf[bswp(j,14,bp)]);
    }
}

static void sma_dec_garouh(uint8_t *R)
{
    static const int dp[16]={14,5,1,11,7,4,10,15,3,12,8,13,0,2,9,6};
    static const int fp[19]={18,5,16,11,2,6,7,17,3,12,8,14,4,0,9,1,10,15,13};
    static const int bp[14]={12,8,1,7,11,3,13,10,6,9,5,4,0,2};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x3FC000 + bswp(i,19,fp)));
    static uint16_t buf[0x4000];
    for (uint32_t i = 0; i < 0x400000; i += 0x4000) {
        for (int j = 0; j < 0x4000; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x4000; j++) wrw(P, i+j, buf[bswp(j,14,bp)]);
    }
}

static void sma_dec_mslug3(uint8_t *R)
{
    static const int dp[16]={4,11,14,3,1,13,0,7,2,8,12,15,10,9,5,6};
    static const int fp[19]={18,15,2,1,13,3,0,9,6,16,4,11,5,7,12,17,14,10,8};
    static const int bp[15]={2,11,0,14,6,4,13,8,9,3,10,7,5,12,1};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x2E8000 + bswp(i,19,fp)));
    static uint16_t buf[0x8000];
    for (uint32_t i = 0; i < 0x400000; i += 0x8000) {
        for (int j = 0; j < 0x8000; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x8000; j++) wrw(P, i+j, buf[bswp(j,15,bp)]);
    }
}

static void sma_dec_mslug3a(uint8_t *R)
{
    static const int dp[16]={2,11,12,14,9,3,1,4,13,7,6,8,10,15,0,5};
    static const int fp[19]={18,1,16,14,7,17,5,8,4,15,6,3,2,0,13,10,12,9,11};
    static const int bp[15]={12,0,11,3,4,13,6,8,14,7,5,2,10,9,1};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x2E8000 + bswp(i,19,fp)));
    static uint16_t buf[0x8000];
    for (uint32_t i = 0; i < 0x400000; i += 0x8000) {
        for (int j = 0; j < 0x8000; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x8000; j++) wrw(P, i+j, buf[bswp(j,15,bp)]);
    }
}

static void sma_dec_kof2000(uint8_t *R)
{
    static const int dp[16]={12,8,11,3,15,14,7,0,10,13,6,5,9,2,1,4};
    static const int bp[10]={4,1,3,8,6,2,7,0,9,5};
    static const int fp[19]={18,8,4,15,13,3,14,16,2,6,17,7,12,10,0,5,11,1,9};
    uint8_t *P = R + 0x100000;
    for (uint32_t i = 0; i < 0x400000; i++) wrw(P, i, (uint16_t)bswp(rdw(P,i),16,dp));
    static uint16_t buf[0x400];
    for (uint32_t i = 0; i < 0x31D000; i += 0x400) {
        for (int j = 0; j < 0x400; j++) buf[j] = rdw(P, i+j);
        for (int j = 0; j < 0x400; j++) wrw(P, i+j, buf[bswp(j,10,bp)]);
    }
    for (uint32_t i = 0; i < 0x60000; i++)
        wrw(R, i, rdw(R, 0x39D000 + bswp(i,19,fp)));
}

/* SMA bank base tables; sel = raw 16-bit banksel write */
static uint32_t sma_bank(int type, uint16_t sel)
{
    static const int t_kof99[64] = {
        0x000000,0x100000,0x200000,0x300000,0x3cc000,0x4cc000,0x3f2000,0x4f2000,
        0x407800,0x507800,0x40d000,0x50d000,0x417800,0x517800,0x420800,0x520800,
        0x424800,0x524800,0x429000,0x529000,0x42e800,0x52e800,0x431800,0x531800,
        0x54d000,0x551000,0x567000,0x592800,0x588800,0x581800,0x599800,0x594800,
        0x598000 };
    static const int t_garou[64] = {
        0x000000,0x100000,0x200000,0x300000,0x280000,0x380000,0x2d0000,0x3d0000,
        0x2f0000,0x3f0000,0x400000,0x500000,0x420000,0x520000,0x440000,0x540000,
        0x498000,0x598000,0x4a0000,0x5a0000,0x4a8000,0x5a8000,0x4b0000,0x5b0000,
        0x4b8000,0x5b8000,0x4c0000,0x5c0000,0x4c8000,0x5c8000,0x4d0000,0x5d0000,
        0x458000,0x558000,0x460000,0x560000,0x468000,0x568000,0x470000,0x570000,
        0x478000,0x578000,0x480000,0x580000,0x488000,0x588000,0x490000,0x590000,
        0x5d0000,0x5d8000,0x5e0000,0x5e8000,0x5f0000,0x5f8000,0x600000 };
    static const int t_garouh[64] = {
        0x000000,0x100000,0x200000,0x300000,0x280000,0x380000,0x2d0000,0x3d0000,
        0x2c8000,0x3c8000,0x400000,0x500000,0x420000,0x520000,0x440000,0x540000,
        0x598000,0x698000,0x5a0000,0x6a0000,0x5a8000,0x6a8000,0x5b0000,0x6b0000,
        0x5b8000,0x6b8000,0x5c0000,0x6c0000,0x5c8000,0x6c8000,0x5d0000,0x6d0000,
        0x458000,0x558000,0x460000,0x560000,0x468000,0x568000,0x470000,0x570000,
        0x478000,0x578000,0x480000,0x580000,0x488000,0x588000,0x490000,0x590000,
        0x5d8000,0x6d8000,0x5e0000,0x6e0000,0x5e8000,0x6e8000,0x6e8000 };
    static const int t_mslug3[64] = {
        0x000000,0x020000,0x040000,0x060000,0x070000,0x090000,0x0b0000,0x0d0000,
        0x0e0000,0x0f0000,0x120000,0x130000,0x140000,0x150000,0x180000,0x190000,
        0x1a0000,0x1b0000,0x1e0000,0x1f0000,0x200000,0x210000,0x240000,0x250000,
        0x260000,0x270000,0x2a0000,0x2b0000,0x2c0000,0x2d0000,0x300000,0x310000,
        0x320000,0x330000,0x360000,0x370000,0x380000,0x390000,0x3c0000,0x3d0000,
        0x400000,0x410000,0x440000,0x450000,0x460000,0x470000,0x4a0000,0x4b0000,
        0x4c0000 };
    static const int t_mslug3a[64] = {
        0x000000,0x030000,0x040000,0x070000,0x080000,0x0a0000,0x0c0000,0x0e0000,
        0x0f0000,0x100000,0x130000,0x140000,0x150000,0x160000,0x190000,0x1a0000,
        0x1b0000,0x1c0000,0x1f0000,0x200000,0x210000,0x220000,0x250000,0x260000,
        0x270000,0x280000,0x2b0000,0x2c0000,0x2d0000,0x2e0000,0x310000,0x320000,
        0x330000,0x340000,0x370000,0x380000,0x390000,0x3a0000,0x3d0000,0x3e0000,
        0x400000,0x410000,0x440000,0x450000,0x460000,0x470000,0x4a0000,0x4b0000,
        0x4c0000 };
    static const int t_kof2000[64] = {
        0x000000,0x100000,0x200000,0x300000,0x3f7800,0x4f7800,0x3ff800,0x4ff800,
        0x407800,0x507800,0x40f800,0x50f800,0x416800,0x516800,0x41d800,0x51d800,
        0x424000,0x524000,0x523800,0x623800,0x526000,0x626000,0x528000,0x628000,
        0x52a000,0x62a000,0x52b800,0x62b800,0x52d000,0x62d000,0x52e800,0x62e800,
        0x618000,0x619000,0x61a000,0x61a800 };
    static const int s_kof99[6]  = {5,12,10,8,6,14};
    static const int s_garou[6]  = {12,14,6,7,9,5};
    static const int s_garouh[6] = {13,11,2,14,8,4};
    static const int s_mslug3[6] = {9,3,6,15,12,14};
    static const int s_mslug3a[6]= {11,12,6,1,3,15};
    static const int s_kof2000[6]= {5,10,3,7,14,15};
    const int *tab = 0, *sp = 0;
    switch (type) {
        case 2: tab = t_kof99;   sp = s_kof99;   break;
        case 3: tab = t_garou;   sp = s_garou;   break;
        case 4: tab = t_garouh;  sp = s_garouh;  break;
        case 5: tab = t_mslug3;  sp = s_mslug3;  break;
        case 6: tab = t_mslug3a; sp = s_mslug3a; break;
        case 7: tab = t_kof2000; sp = s_kof2000; break;
        default: return 0x100000;
    }
    int data = (int)bswp(sel, 6, sp);
    return (uint32_t)(0x100000 + tab[data]);
}

/* ---- PVC PRG decrypt ----
 * MAME operates on the region's NATIVE byte layout (LE host: region byte r
 * holds bus byte r^1).  Our ng->rom array IS bus byte order; translating
 * MAME's accesses into bus space gives:
 *   M[i] ^= x[(i^1)%0x20]      -> B[b] ^= x[b%0x20]
 *   v = M[i]|M[i+3]<<8         -> v = B[i+1]|B[i+2]<<8   (i%4==0)
 *   M[0x800000+i]^=M[0x100002|i] -> B[0x800000+j]^=B[0x100002|j]
 * Word-aligned block moves are byte-order agnostic.
 */
static void pvc_xor(uint8_t *R, uint32_t a, uint32_t b, const uint8_t *x)
{ for (uint32_t i = a; i < b; i++) R[i] ^= x[i % 0x20]; }
static void pvc_scr(uint8_t *R, uint32_t a, uint32_t b, const int *sp)
{
    for (uint32_t i = a; i < b; i += 4) {
        uint32_t v = bswp(R[i+1] | (R[i+2] << 8), 16, sp);
        R[i+1] = (uint8_t)v; R[i+2] = (uint8_t)(v >> 8);
    }
}
static void pvc_rawxor(uint8_t *R)   /* MAME: rom[0x800000+i] ^= rom[0x100002|i] */
{ for (uint32_t i = 0; i < 0x100000; i++) R[0x800000+i] ^= R[0x100002|i]; }

static void pvc_dec_mslug5(uint8_t *R)
{
    static const uint8_t x1[0x20]={0xc2,0x4b,0x74,0xfd,0x0b,0x34,0xeb,0xd7,0x10,0x6d,0xf9,0xce,0x5d,0xd5,0x61,0x29,0xf5,0xbe,0x0d,0x82,0x72,0x45,0x0f,0x24,0xb3,0x34,0x1b,0x99,0xea,0x09,0xf3,0x03};
    static const uint8_t x2[0x20]={0x36,0x09,0xb0,0x64,0x95,0x0f,0x90,0x42,0x6e,0x0f,0x30,0xf6,0xe5,0x08,0x30,0x64,0x08,0x04,0x00,0x2f,0x72,0x09,0xa0,0x13,0xc9,0x0b,0xa0,0x3e,0xc2,0x00,0x40,0x2b};
    static const int sp[16]={15,14,13,12,10,11,8,9,6,7,4,5,3,2,1,0};
    static const int m1[8]={7,6,5,4,1,0,3,2};
    static const int m2[8]={5,4,7,6,1,0,3,2};
    uint8_t *B = prot_buf();
    pvc_xor(R,0,0x100000,x1); pvc_xor(R,0x100000,0x800000,x2); pvc_scr(R,0x100000,0x800000,sp);
    memcpy(B,R,0x800000);
    for (int i = 0; i < 16; i++) {
        uint32_t o = (i & 0xf0) + bswp(i & 0x0f, 8, m1);
        memcpy(R + i*0x10000, B + o*0x10000, 0x10000);
    }
    for (uint32_t i = 0x100000; i < 0x800000; i += 0x100) {
        uint32_t o = (i & 0xf000ff) + ((i & 0x000f00) ^ 0x00700) + (bswp((i & 0x0ff000) >> 12, 8, m2) << 12);
        memcpy(R + i, B + o, 0x100);
    }
    memcpy(B,R,0x800000);
    memcpy(R+0x100000, B+0x700000, 0x100000);
    memcpy(R+0x200000, B+0x100000, 0x600000);
}

static void pvc_dec_svc(uint8_t *R)
{
    static const uint8_t x1[0x20]={0x3b,0x6a,0xf7,0xb7,0xe8,0xa9,0x20,0x99,0x9f,0x39,0x34,0x0c,0xc3,0x9a,0xa5,0xc8,0xb8,0x18,0xce,0x56,0x94,0x44,0xe3,0x7a,0xf7,0xdd,0x42,0xf0,0x18,0x60,0x92,0x9f};
    static const uint8_t x2[0x20]={0x69,0x0b,0x60,0xd6,0x4f,0x01,0x40,0x1a,0x9f,0x0b,0xf0,0x75,0x58,0x0e,0x60,0xb4,0x14,0x04,0x20,0xe4,0xb9,0x0d,0x10,0x89,0xeb,0x07,0x30,0x90,0x50,0x0e,0x20,0x26};
    static const int sp[16]={15,14,13,12,10,11,8,9,6,7,4,5,3,2,1,0};
    static const int m1[8]={7,6,5,4,2,3,0,1};
    static const int m2[8]={4,5,6,7,1,0,3,2};
    uint8_t *B = prot_buf();
    pvc_xor(R,0,0x100000,x1); pvc_xor(R,0x100000,0x800000,x2); pvc_scr(R,0x100000,0x800000,sp);
    memcpy(B,R,0x800000);
    for (int i = 0; i < 16; i++) {
        uint32_t o = (i & 0xf0) + bswp(i & 0x0f, 8, m1);
        memcpy(R + i*0x10000, B + o*0x10000, 0x10000);
    }
    for (uint32_t i = 0x100000; i < 0x800000; i += 0x100) {
        uint32_t o = (i & 0xf000ff) + ((i & 0x000f00) ^ 0x00a00) + (bswp((i & 0x0ff000) >> 12, 8, m2) << 12);
        memcpy(R + i, B + o, 0x100);
    }
    memcpy(B,R,0x800000);
    memcpy(R+0x100000, B+0x700000, 0x100000);
    memcpy(R+0x200000, B+0x100000, 0x600000);
}

static void pvc_dec_kof2003(uint8_t *R)
{
    static const uint8_t x1[0x20]={0x3b,0x6a,0xf7,0xb7,0xe8,0xa9,0x20,0x99,0x9f,0x39,0x34,0x0c,0xc3,0x9a,0xa5,0xc8,0xb8,0x18,0xce,0x56,0x94,0x44,0xe3,0x7a,0xf7,0xdd,0x42,0xf0,0x18,0x60,0x92,0x9f};
    static const uint8_t x2[0x20]={0x2f,0x02,0x60,0xbb,0x77,0x01,0x30,0x08,0xd8,0x01,0xa0,0xdf,0x37,0x0a,0xf0,0x65,0x28,0x03,0xd0,0x23,0xd3,0x03,0x70,0x42,0xbb,0x06,0xf0,0x28,0xba,0x0f,0xf0,0x7a};
    static const int sp[16]={15,14,13,12,5,4,7,6,9,8,11,10,3,2,1,0};
    static const int m1[8]={7,6,5,4,0,1,2,3};
    static const int m2[8]={4,5,6,7,1,0,3,2};
    uint8_t *B = prot_buf();
    pvc_rawxor(R);
    pvc_xor(R,0,0x100000,x1); pvc_xor(R,0x100000,0x800000,x2); pvc_scr(R,0x100000,0x800000,sp);
    for (int i = 0; i < 16; i++) {
        uint32_t o = (i & 0xf0) + bswp(i & 0x0f, 8, m1);
        memcpy(B + i*0x10000, R + o*0x10000, 0x10000);
    }
    for (uint32_t i = 0x100000; i < 0x900000; i += 0x100) {
        uint32_t o = (i & 0xf000ff) + ((i & 0x000f00) ^ 0x00800) + (bswp((i & 0x0ff000) >> 12, 8, m2) << 12);
        memcpy(B + i, R + o, 0x100);
    }
    memcpy(R+0x000000, B+0x000000, 0x100000);
    memcpy(R+0x100000, B+0x800000, 0x100000);
    memcpy(R+0x200000, B+0x100000, 0x700000);
}

static void pvc_dec_kof2003h(uint8_t *R)
{
    static const uint8_t x1[0x20]={0xc2,0x4b,0x74,0xfd,0x0b,0x34,0xeb,0xd7,0x10,0x6d,0xf9,0xce,0x5d,0xd5,0x61,0x29,0xf5,0xbe,0x0d,0x82,0x72,0x45,0x0f,0x24,0xb3,0x34,0x1b,0x99,0xea,0x09,0xf3,0x03};
    static const uint8_t x2[0x20]={0x2b,0x09,0xd0,0x7f,0x51,0x0b,0x10,0x4c,0x5b,0x07,0x70,0x9d,0x3e,0x0b,0xb0,0xb6,0x54,0x09,0xe0,0xcc,0x3d,0x0d,0x80,0x99,0x87,0x03,0x90,0x82,0xfe,0x04,0x20,0x18};
    static const int sp[16]={15,14,13,12,10,11,8,9,6,7,4,5,3,2,1,0};
    static const int m1[8]={7,6,5,4,1,0,3,2};
    static const int m2[8]={6,7,4,5,0,1,2,3};
    uint8_t *B = prot_buf();
    pvc_rawxor(R);
    pvc_xor(R,0,0x100000,x1); pvc_xor(R,0x100000,0x800000,x2); pvc_scr(R,0x100000,0x800000,sp);
    for (int i = 0; i < 16; i++) {
        uint32_t o = (i & 0xf0) + bswp(i & 0x0f, 8, m1);
        memcpy(B + i*0x10000, R + o*0x10000, 0x10000);
    }
    for (uint32_t i = 0x100000; i < 0x900000; i += 0x100) {
        uint32_t o = (i & 0xf000ff) + ((i & 0x000f00) ^ 0x00400) + (bswp((i & 0x0ff000) >> 12, 8, m2) << 12);
        memcpy(B + i, R + o, 0x100);
    }
    memcpy(R+0x000000, B+0x000000, 0x100000);
    memcpy(R+0x100000, B+0x800000, 0x100000);
    memcpy(R+0x200000, B+0x100000, 0x700000);
}

static void pvc_dec_kf2k3pcb(uint8_t *R)
{
    static const uint8_t x2[0x20]={0xb4,0x0f,0x40,0x6c,0x38,0x07,0xd0,0x3f,0x53,0x08,0x80,0xaa,0xbe,0x07,0xc0,0xfa,0xd0,0x08,0x10,0xd2,0xf1,0x03,0x70,0x7e,0x87,0x0b,0x40,0xf6,0x2a,0x0a,0xe0,0xf9};
    static const int sp[16]={15,14,13,12,4,5,6,7,8,9,10,11,3,2,1,0};
    static const int m1[8]={7,6,5,4,1,0,3,2};
    static const int m2[8]={4,5,6,7,1,0,3,2};
    uint8_t *B = prot_buf();
    pvc_rawxor(R);
    pvc_xor(R,0x100000,0x800000,x2); pvc_scr(R,0x100000,0x800000,sp);
    for (int i = 0; i < 16; i++) {
        uint32_t o = (i & 0xf0) + bswp(i & 0x0f, 8, m1);
        memcpy(B + i*0x10000, R + o*0x10000, 0x10000);
    }
    for (uint32_t i = 0x100000; i < 0x900000; i += 0x100) {
        uint32_t o = (i & 0xf000ff) + ((i & 0x000f00) ^ 0x00300) + (bswp((i & 0x0ff000) >> 12, 8, m2) << 12);
        memcpy(B + i, R + o, 0x100);
    }
    memcpy(R+0x000000, B+0x000000, 0x100000);
    memcpy(R+0x100000, B+0x800000, 0x100000);
    memcpy(R+0x200000, B+0x100000, 0x700000);
}

/* ---- CMC42/CMC50 graphics descrambler (ported from MAME prot_cmc.cpp) ----
 * Operates in MAME region byte space (bus array word-swapped); callers wrap
 * with wordswap() like pvc_run().  Tables auto-extracted, do not hand-edit. */
static const uint8_t cmc_kof99_type0_t03[256] = {
    0xfb,0x86,0x9d,0xf1,0xbf,0x80,0xd5,0x43,0xab,0xb3,0x9f,0x6a,0x33,0xd9,0xdb,0xb6,
    0x66,0x08,0x69,0x88,0xcc,0xb7,0xde,0x49,0x97,0x64,0x1f,0xa6,0xc0,0x2f,0x52,0x42,
    0x44,0x5a,0xf2,0x28,0x98,0x87,0x96,0x8a,0x83,0x0b,0x03,0x61,0x71,0x99,0x6b,0xb5,
    0x1a,0x8e,0xfe,0x04,0xe1,0xf7,0x7d,0xdd,0xed,0xca,0x37,0xfc,0xef,0x39,0x72,0xda,
    0xb8,0xbe,0xee,0x7f,0xe5,0x31,0x78,0xf3,0x91,0x9a,0xd2,0x11,0x19,0xb9,0x09,0x4c,
    0xfd,0x6d,0x2a,0x4d,0x65,0xa1,0x89,0xc7,0x75,0x50,0x21,0xfa,0x16,0x00,0xe9,0x12,
    0x74,0x2b,0x1e,0x4f,0x14,0x01,0x70,0x3a,0x4e,0x3f,0xf5,0xf4,0x1d,0x3d,0x15,0x27,
    0xa7,0xff,0x45,0xe0,0x6e,0xf9,0x54,0xc8,0x48,0xad,0xa5,0x0a,0xf6,0x2d,0x2c,0xe2,
    0x68,0x67,0xd6,0x85,0xb4,0xc3,0x34,0xbc,0x62,0xd3,0x5f,0x84,0x06,0x5b,0x0d,0x95,
    0xea,0x5e,0x9e,0xd4,0xeb,0x90,0x7a,0x05,0x81,0x57,0xe8,0x60,0x2e,0x20,0x25,0x7c,
    0x46,0x0c,0x93,0xcb,0xbd,0x17,0x7e,0xec,0x79,0xb2,0xc2,0x22,0x41,0xb1,0x10,0xac,
    0xa8,0xbb,0x9b,0x82,0x4b,0x9c,0x8b,0x07,0x47,0x35,0x24,0x56,0x8d,0xaf,0xe6,0x26,
    0x40,0x38,0xc4,0x5d,0x1b,0xc5,0xd1,0x0f,0x6c,0x7b,0xb0,0xe3,0xa3,0x23,0x6f,0x58,
    0xc1,0xba,0xcf,0xd7,0xa2,0xe7,0xd0,0x63,0x5c,0xf8,0x73,0xa0,0x13,0xdc,0x29,0xcd,
    0xc9,0x76,0xae,0x8f,0xe4,0x59,0x30,0xaa,0x94,0x1c,0x3c,0x0e,0x55,0x92,0x77,0x32,
    0xc6,0xce,0x18,0x36,0xdf,0xa9,0x8c,0xd8,0xa4,0xf0,0x3b,0x51,0x4a,0x02,0x3e,0x53,
};
static const uint8_t cmc_kof99_type0_t12[256] = {
    0x1f,0xac,0x4d,0xcd,0xca,0x70,0x02,0x6b,0x18,0x40,0x62,0xb2,0x3f,0x9b,0x5b,0xef,
    0x69,0x68,0x71,0x3b,0xcb,0xd4,0x30,0xbc,0x47,0x72,0x74,0x5e,0x84,0x4c,0x1b,0xdb,
    0x6a,0x35,0x1d,0xf5,0xa1,0xb3,0x87,0x5d,0x57,0x28,0x2f,0xc4,0xfd,0x24,0x26,0x36,
    0xad,0xbe,0x61,0x63,0x73,0xaa,0x82,0xee,0x29,0xd0,0xdf,0x8c,0x15,0xb5,0x96,0xf3,
    0xdd,0x7e,0x3a,0x37,0x58,0x7f,0x0c,0xfc,0x0b,0x07,0xe8,0xf7,0xf4,0x14,0xb8,0x81,
    0xb6,0xd7,0x1e,0xc8,0x85,0xe6,0x9d,0x33,0x60,0xc5,0x95,0xd5,0x55,0x00,0xa3,0xb7,
    0x7d,0x50,0x0d,0xd2,0xc1,0x12,0xe5,0xed,0xd8,0xa4,0x9c,0x8f,0x2a,0x4f,0xa8,0x01,
    0x52,0x83,0x65,0xea,0x9a,0x6c,0x44,0x4a,0xe2,0xa5,0x2b,0x46,0xe1,0x34,0x25,0xf8,
    0xc3,0xda,0xc7,0x6e,0x48,0x38,0x7c,0x78,0x06,0x53,0x64,0x16,0x98,0x3c,0x91,0x42,
    0x39,0xcc,0xb0,0xf1,0xeb,0x13,0xbb,0x05,0x32,0x86,0x0e,0xa2,0x0a,0x9e,0xfa,0x66,
    0x54,0x8e,0xd3,0xe7,0x19,0x20,0x77,0xec,0xff,0xbd,0x6d,0x43,0x23,0x03,0xab,0x75,
    0x3d,0xcf,0xd1,0xde,0x92,0x31,0xa7,0x45,0x4b,0xc2,0x97,0xf9,0x7a,0x88,0xd9,0x1c,
    0xe9,0xe4,0x10,0xc9,0x22,0x2d,0x90,0x76,0x17,0x79,0x04,0x51,0x1a,0x5a,0x5f,0x2c,
    0x21,0x6f,0x3e,0xe0,0xf0,0xbf,0xd6,0x94,0x0f,0x80,0x11,0xa0,0x5c,0xa9,0x49,0x2e,
    0xce,0xaf,0xa6,0x9f,0x7b,0x99,0xb9,0xb4,0xe3,0xfb,0xf6,0x27,0xf2,0x93,0xfe,0x08,
    0x67,0xae,0x09,0x89,0xdc,0x4e,0xc6,0xc0,0x8a,0xb1,0x59,0x8b,0x41,0x56,0x8d,0xba,
};
static const uint8_t cmc_kof99_type1_t03[256] = {
    0xa9,0x17,0xaf,0x0d,0x34,0x6e,0x53,0xb6,0x7f,0x58,0xe9,0x14,0x5f,0x55,0xdb,0xd4,
    0x42,0x80,0x99,0x59,0xa8,0x3a,0x57,0x5d,0xd5,0x6f,0x4c,0x68,0x35,0x46,0xa6,0xe7,
    0x7b,0x71,0xe0,0x93,0xa2,0x1f,0x64,0x21,0xe3,0xb1,0x98,0x26,0xab,0xad,0xee,0xe5,
    0xbb,0xd9,0x1e,0x2e,0x95,0x36,0xef,0x23,0x79,0x45,0x04,0xed,0x13,0x1d,0xf4,0x85,
    0x96,0xec,0xc2,0x32,0xaa,0x7c,0x15,0xd8,0xda,0x92,0x90,0x9d,0xb7,0x56,0x6a,0x66,
    0x41,0xfc,0x00,0xf6,0x50,0x24,0xcf,0xfb,0x11,0xfe,0x82,0x48,0x9b,0x27,0x1b,0x67,
    0x4e,0x84,0x69,0x97,0x6d,0x8c,0xd2,0xba,0x74,0xf9,0x8f,0xa5,0x54,0x5c,0xcd,0x73,
    0x07,0xd1,0x01,0x09,0xf1,0x19,0x3b,0x5e,0x87,0x30,0x76,0xcc,0xc0,0x5a,0xa7,0x49,
    0x22,0xfa,0x16,0x02,0xdf,0xa4,0xff,0xb3,0x75,0x33,0xbd,0x88,0x2f,0xcb,0x2a,0x44,
    0xb8,0xbf,0x1c,0x0f,0x81,0x10,0x43,0xb4,0xc8,0x7e,0x9a,0x25,0xea,0x83,0x4b,0x38,
    0x7a,0xd7,0x3d,0x1a,0x4f,0x62,0x51,0xc9,0x47,0x0e,0xce,0x3f,0xc7,0x4d,0x2c,0xa1,
    0x86,0xb9,0xc5,0xca,0xdd,0x6b,0x70,0x6c,0x91,0x9c,0xbe,0x0a,0x9f,0xf5,0x94,0xbc,
    0x18,0x2b,0x60,0x20,0x29,0xf7,0xf2,0x28,0xc4,0xa0,0x0b,0x65,0xde,0x8d,0x78,0x12,
    0x3e,0xd0,0x77,0x08,0x8b,0xae,0x05,0x31,0x3c,0xd6,0xa3,0x89,0x06,0xdc,0x52,0x72,
    0xb0,0xb5,0x37,0xd3,0xc3,0x8a,0xc6,0xf0,0xc1,0x61,0xfd,0x4a,0x5b,0x7d,0x9e,0xf3,
    0x63,0x40,0x2d,0xe8,0xb2,0xe6,0x39,0x03,0xeb,0x8e,0xe1,0x0c,0xe4,0xe2,0xf8,0xac,
};
static const uint8_t cmc_kof99_type1_t12[256] = {
    0xea,0xe6,0x5e,0xa7,0x8e,0xac,0x34,0x03,0x30,0x97,0x52,0x53,0x76,0xf2,0x62,0x0b,
    0x0a,0xfc,0x94,0xb8,0x67,0x36,0x11,0xbc,0xae,0xca,0xfa,0x15,0x04,0x2b,0x17,0xc4,
    0x3e,0x5b,0x59,0x01,0x57,0xe2,0xba,0xb7,0xd1,0x3f,0xf0,0x6a,0x9c,0x2a,0xcb,0xa9,
    0xe3,0x2c,0xc0,0x0f,0x46,0x91,0x8a,0xd0,0x98,0xc5,0xa6,0x1b,0x96,0x29,0x12,0x09,
    0x63,0xed,0xe0,0xa2,0x86,0x77,0xbe,0xe5,0x65,0xdb,0xbd,0x50,0xb3,0x9d,0x1a,0x4e,
    0x79,0x0c,0x00,0x43,0xdf,0x3d,0x54,0x33,0x8f,0x89,0xa8,0x7b,0xf9,0xd5,0x27,0x82,
    0xbb,0xc2,0x8c,0x47,0x88,0x6b,0xb4,0xc3,0xf8,0xaa,0x06,0x1e,0x83,0x7d,0x05,0x78,
    0x85,0xf6,0x6e,0x2e,0xec,0x5a,0x31,0x45,0x38,0x14,0x16,0x8b,0x02,0xe4,0x4f,0xb0,
    0xbf,0xab,0xa4,0x9e,0x48,0x60,0x19,0x35,0x08,0xde,0xdd,0x66,0x90,0x51,0xcc,0xa3,
    0xaf,0x70,0x9b,0x75,0x95,0x49,0x6c,0x64,0x72,0x7e,0x44,0xa0,0x73,0x25,0x68,0x55,
    0x1f,0x40,0x7a,0x74,0x0e,0x8d,0xdc,0x1c,0x71,0xc8,0xcf,0xd7,0xe8,0xce,0xeb,0x32,
    0x3a,0xee,0x07,0x61,0x4d,0xfe,0x5c,0x7c,0x56,0x2f,0x2d,0x5f,0x6f,0x9f,0x81,0x22,
    0x58,0x4b,0xad,0xda,0xb9,0x10,0x18,0x23,0xe1,0xf3,0x6d,0xe7,0xe9,0x28,0xd6,0xd8,
    0xf4,0x4c,0x39,0x21,0xb2,0x84,0xc1,0x24,0x26,0xf1,0x93,0x37,0xc6,0x4a,0xcd,0x20,
    0xc9,0xd9,0xc7,0xb1,0xff,0x99,0xd4,0x5d,0xb5,0xa1,0x87,0x0d,0x69,0x92,0x13,0x80,
    0xd2,0xd3,0xfd,0x1d,0xf5,0x3b,0xa5,0x7f,0xef,0x9a,0xb6,0x42,0xfb,0x3c,0xf7,0x41,
};
static const uint8_t cmc_kof99_address_8_15_xor1[256] = {
    0x00,0xb1,0x1e,0xc5,0x3d,0x40,0x45,0x5e,0xf2,0xf8,0x04,0x63,0x36,0x87,0x88,0xbf,
    0xab,0xcc,0x78,0x08,0xdd,0x20,0xd4,0x35,0x09,0x8e,0x44,0xae,0x33,0xa9,0x9e,0xcd,
    0xb3,0xe5,0xad,0x41,0xda,0xbe,0xf4,0x16,0x57,0x2e,0x53,0x67,0xaf,0xdb,0x8a,0xd8,
    0x34,0x17,0x3c,0x01,0x55,0x73,0xcf,0xe3,0xe8,0xc7,0x0d,0xe9,0xa3,0x13,0x0c,0xf6,
    0x90,0x4e,0xfb,0x97,0x6d,0x5f,0xa8,0x71,0x11,0xfc,0xd1,0x95,0x81,0xba,0x8c,0x1b,
    0x39,0xfe,0xa2,0x15,0xa6,0x52,0x4d,0x5b,0x59,0xa5,0xe0,0x96,0xd9,0x8f,0x7b,0xed,
    0x29,0xd3,0x1f,0x0e,0xec,0x23,0x0f,0xb8,0x6c,0x6f,0x7d,0x18,0x46,0xd6,0xe4,0xb5,
    0x9a,0x79,0x02,0xf5,0x03,0xc0,0x60,0x66,0x5c,0x2f,0x76,0x85,0x9d,0x54,0x1a,0x6a,
    0x28,0xce,0x7f,0x7c,0x91,0x99,0x4c,0x83,0x3e,0xb4,0x1d,0x05,0xc1,0xc3,0xd7,0x47,
    0xde,0xbc,0x62,0x6e,0x86,0x14,0x80,0x77,0xeb,0xf3,0x07,0x31,0x56,0xd2,0xc2,0xc6,
    0x6b,0xdc,0xfd,0x22,0x92,0xf0,0x06,0x51,0x2d,0x38,0xe6,0xa0,0x25,0xdf,0xd5,0x2c,
    0x1c,0x94,0x12,0x9c,0xb0,0x9b,0xc4,0x0b,0xc8,0xd0,0xf7,0x30,0xcb,0x27,0xfa,0x7a,
    0x10,0x61,0xaa,0xa4,0x70,0xb7,0x2a,0x5a,0xc9,0xf1,0x0a,0x49,0x65,0xee,0x69,0x4b,
    0x3a,0x8d,0x32,0x5d,0x68,0xb9,0x9f,0x75,0x19,0x3f,0xac,0x37,0x4f,0xe7,0x93,0x89,
    0x7e,0x4a,0x3b,0xea,0x74,0x72,0x43,0xbd,0x24,0xef,0xb6,0xff,0x64,0x58,0x84,0x8b,
    0xa7,0xbb,0xb2,0xe1,0x26,0x2b,0x50,0xca,0x21,0xf9,0x98,0xa1,0xe2,0x42,0x82,0x48,
};
static const uint8_t cmc_kof99_address_8_15_xor2[256] = {
    0x9b,0x9d,0xc1,0x3d,0xa9,0xb8,0xf4,0x6f,0xf6,0x25,0xc7,0x47,0xd5,0x97,0xdf,0x6b,
    0xeb,0x90,0xa4,0xb2,0x5d,0xf5,0x66,0xb0,0xb9,0x8b,0x93,0x64,0xec,0x7b,0x65,0x8c,
    0xf1,0x43,0x42,0x6e,0x45,0x9f,0xb3,0x35,0x06,0x71,0x96,0xdb,0xa0,0xfb,0x0b,0x3a,
    0x1f,0xf8,0x8e,0x69,0xcd,0x26,0xab,0x86,0xa2,0x0c,0xbd,0x63,0xa5,0x7a,0xe7,0x6a,
    0x5f,0x18,0x9e,0xbf,0xad,0x55,0xb1,0x1c,0x5c,0x03,0x30,0xc6,0x37,0x20,0xe3,0xc9,
    0x52,0xe8,0xee,0x4f,0x01,0x70,0xc4,0x77,0x29,0x2a,0xba,0x53,0x12,0x04,0x7d,0xaf,
    0x33,0x8f,0xa8,0x4d,0xaa,0x5b,0xb4,0x0f,0x92,0xbb,0xed,0xe1,0x2f,0x50,0x6c,0xd2,
    0x2c,0x95,0xd9,0xf9,0x98,0xc3,0x76,0x4c,0xf2,0xe4,0xe5,0x2b,0xef,0x9c,0x49,0xb6,
    0x31,0x3b,0xbc,0xa1,0xca,0xde,0x62,0x74,0xea,0x81,0x00,0xdd,0xa6,0x46,0x88,0x3f,
    0x39,0xd6,0x23,0x54,0x24,0x4a,0xd8,0xdc,0xd7,0xd1,0xcc,0xbe,0x57,0x7c,0xda,0x44,
    0x61,0xce,0xd3,0xd4,0xe9,0x28,0x80,0xe0,0x56,0x8a,0x09,0x05,0x9a,0x89,0x1b,0xf7,
    0xf3,0x99,0x6d,0x5e,0x48,0x91,0xc0,0xd0,0xc5,0x79,0x78,0x41,0x59,0x21,0x2e,0xff,
    0xc2,0x4b,0x38,0x83,0x32,0xe6,0xe2,0x7f,0x1e,0x17,0x58,0x1d,0x1a,0xfa,0x85,0x82,
    0x94,0xc8,0x72,0x7e,0xb7,0xac,0x0e,0xfc,0xfd,0x16,0x27,0x75,0x8d,0xcb,0x08,0xfe,
    0x0a,0x02,0x0d,0x36,0x11,0x22,0x84,0x40,0x34,0x3e,0x2d,0x68,0x5a,0xa7,0x67,0xae,
    0x87,0x07,0x10,0x60,0x14,0x73,0x3c,0x51,0x19,0xa3,0xb5,0xcf,0x13,0xf0,0x15,0x4e,
};
static const uint8_t cmc_kof99_address_16_23_xor1[256] = {
    0x00,0x5f,0x03,0x52,0xce,0xe3,0x7d,0x8f,0x6b,0xf8,0x20,0xde,0x7b,0x7e,0x39,0xbe,
    0xf5,0x94,0x18,0x78,0x80,0xc9,0x7f,0x7a,0x3e,0x63,0xf2,0xe0,0x4e,0xf7,0x87,0x27,
    0x69,0x6c,0xa4,0x1d,0x85,0x5b,0xe6,0x44,0x25,0x0c,0x98,0xc7,0x01,0x02,0xa3,0x26,
    0x09,0x38,0xdb,0xc3,0x1e,0xcf,0x23,0x45,0x68,0x76,0xd6,0x22,0x5d,0x5a,0xae,0x16,
    0x9f,0xa2,0xb5,0xcd,0x81,0xea,0x5e,0xb8,0xb9,0x9d,0x9c,0x1a,0x0f,0xff,0xe1,0xe7,
    0x74,0xaa,0xd4,0xaf,0xfc,0xc6,0x33,0x29,0x5c,0xab,0x95,0xf0,0x19,0x47,0x59,0x67,
    0xf3,0x96,0x60,0x1f,0x62,0x92,0xbd,0x89,0xee,0x28,0x13,0x06,0xfe,0xfa,0x32,0x6d,
    0x57,0x3c,0x54,0x50,0x2c,0x58,0x49,0xfb,0x17,0xcc,0xef,0xb2,0xb4,0xf9,0x07,0x70,
    0xc5,0xa9,0xdf,0xd5,0x3b,0x86,0x2b,0x0d,0x6e,0x4d,0x0a,0x90,0x43,0x31,0xc1,0xf6,
    0x88,0x0b,0xda,0x53,0x14,0xdc,0x75,0x8e,0xb0,0xeb,0x99,0x46,0xa1,0x15,0x71,0xc8,
    0xe9,0x3f,0x4a,0xd9,0x73,0xe5,0x7c,0x30,0x77,0xd3,0xb3,0x4b,0x37,0x72,0xc2,0x04,
    0x97,0x08,0x36,0xb1,0x3a,0x61,0xec,0xe2,0x1c,0x9a,0x8b,0xd1,0x1b,0x2e,0x9e,0x8a,
    0xd8,0x41,0xe4,0xc4,0x40,0x2f,0xad,0xc0,0xb6,0x84,0x51,0x66,0xbb,0x12,0xe8,0xdd,
    0xcb,0xbc,0x6f,0xd0,0x11,0x83,0x56,0x4c,0xca,0xbf,0x05,0x10,0xd7,0xba,0xfd,0xed,
    0x8c,0x0e,0x4f,0x3d,0x35,0x91,0xb7,0xac,0x34,0x64,0x2a,0xf1,0x79,0x6a,0x9b,0x2d,
    0x65,0xf4,0x42,0xa0,0x8d,0xa7,0x48,0x55,0x21,0x93,0x24,0xd2,0xa6,0xa5,0xa8,0x82,
};
static const uint8_t cmc_kof99_address_16_23_xor2[256] = {
    0x29,0x97,0x1a,0x2c,0x0b,0x94,0x3e,0x75,0x01,0x0d,0x1b,0xe1,0x4d,0x38,0x39,0x8f,
    0xe7,0xd0,0x60,0x90,0xb2,0x0f,0xbb,0x70,0x1f,0xe6,0x5b,0x87,0xb4,0x43,0xfd,0xf5,
    0xf6,0xf9,0xad,0xc0,0x98,0x17,0x9f,0x91,0x15,0x51,0x55,0x64,0x6c,0x18,0x61,0x0e,
    0xd9,0x93,0xab,0xd6,0x24,0x2f,0x6a,0x3a,0x22,0xb1,0x4f,0xaa,0x23,0x48,0xed,0xb9,
    0x88,0x8b,0xa3,0x6b,0x26,0x4c,0xe8,0x2d,0x1c,0x99,0xbd,0x5c,0x58,0x08,0x50,0xf2,
    0x2a,0x62,0xc1,0x72,0x66,0x04,0x10,0x37,0x6e,0xfc,0x44,0xa9,0xdf,0xd4,0x20,0xdd,
    0xee,0x41,0xdb,0x73,0xde,0x54,0xec,0xc9,0xf3,0x4b,0x2e,0xae,0x5a,0x4a,0x5e,0x47,
    0x07,0x2b,0x76,0xa4,0xe3,0x28,0xfe,0xb0,0xf0,0x02,0x06,0xd1,0xaf,0x42,0xc2,0xa5,
    0xe0,0x67,0xbf,0x16,0x8e,0x35,0xce,0x8a,0xe5,0x3d,0x7b,0x96,0xd7,0x79,0x52,0x1e,
    0xa1,0xfb,0x9b,0xbe,0x21,0x9c,0xe9,0x56,0x14,0x7f,0xa0,0xe4,0xc3,0xc4,0x46,0xea,
    0xf7,0xd2,0x1d,0x31,0x0a,0x5f,0xeb,0xa2,0x68,0x8d,0xb5,0xc5,0x74,0x0c,0xdc,0x82,
    0x80,0x09,0x19,0x95,0x71,0x9a,0x11,0x57,0x77,0x4e,0xc6,0xff,0x12,0x03,0xa7,0xc7,
    0xf4,0xc8,0xb6,0x7a,0x59,0x36,0x3c,0x53,0xe2,0x69,0x8c,0x25,0x05,0x45,0x63,0xf8,
    0x34,0x89,0x33,0x3f,0x85,0x27,0xbc,0x65,0xfa,0xa8,0x6d,0x84,0x5d,0xba,0x40,0x32,
    0x30,0xef,0x83,0x13,0xa6,0x78,0xcc,0x81,0x9e,0xda,0xca,0xd3,0x7e,0x9d,0x6f,0xcd,
    0xb7,0xb3,0xd8,0xcf,0x3b,0x00,0x92,0xb8,0x86,0xac,0x49,0x7c,0xf1,0xd5,0xcb,0x7d,
};
static const uint8_t cmc_kof99_address_0_7_xor[256] = {
    0x74,0xad,0x5d,0x1d,0x9e,0xc3,0xfa,0x4e,0xf7,0xdb,0xca,0xa2,0x64,0x36,0x56,0x0c,
    0x4f,0xcf,0x43,0x66,0x1e,0x91,0xe3,0xa5,0x58,0xc2,0xc1,0xd4,0xb9,0xdd,0x76,0x16,
    0xce,0x61,0x75,0x01,0x2b,0x22,0x38,0x55,0x50,0xef,0x6c,0x99,0x05,0xe9,0xe8,0xe0,
    0x2d,0xa4,0x4b,0x4a,0x42,0xae,0xba,0x8c,0x6f,0x93,0x14,0xbd,0x71,0x21,0xb0,0x02,
    0x15,0xc4,0xe6,0x60,0xd7,0x44,0xfd,0x85,0x7e,0x78,0x8f,0x00,0x81,0xf1,0xa7,0x3b,
    0xa0,0x10,0xf4,0x9f,0x39,0x88,0x35,0x62,0xcb,0x19,0x31,0x11,0x51,0xfb,0x2a,0x20,
    0x45,0xd3,0x7d,0x92,0x1b,0xf2,0x09,0x0d,0x97,0xa9,0xb5,0x3c,0xee,0x5c,0xaf,0x7b,
    0xd2,0x3a,0x49,0x8e,0xb6,0xcd,0xd9,0xde,0x8a,0x29,0x6e,0xd8,0x0b,0xe1,0x69,0x87,
    0x1a,0x96,0x18,0xcc,0xdf,0xe7,0xc5,0xc7,0xf8,0x52,0xc9,0xf0,0xb7,0xe5,0x33,0xda,
    0x67,0x9d,0xa3,0x03,0x0e,0x72,0x26,0x79,0xe2,0xb8,0xfc,0xaa,0xfe,0xb4,0x86,0xc8,
    0xd1,0xbc,0x12,0x08,0x77,0xeb,0x40,0x8d,0x04,0x25,0x4d,0x5a,0x6a,0x7a,0x2e,0x41,
    0x65,0x1c,0x13,0x94,0xb2,0x63,0x28,0x59,0x5e,0x9a,0x30,0x07,0xc6,0xbf,0x17,0xf5,
    0x0f,0x89,0xf3,0x1f,0xea,0x6d,0xb3,0xc0,0x70,0x47,0xf9,0x53,0xf6,0xd6,0x54,0xed,
    0x6b,0x4c,0xe4,0x8b,0x83,0x24,0x90,0xb1,0x7c,0xbb,0x73,0xab,0xd5,0x2f,0x5f,0xec,
    0x9c,0x2c,0xa8,0x34,0x46,0x37,0x27,0xa1,0x0a,0x06,0x80,0x68,0x82,0x32,0x84,0xff,
    0x48,0xac,0x7f,0x3f,0x95,0xdc,0x98,0x9b,0xbe,0x23,0x57,0x3e,0x5b,0xd0,0x3d,0xa6,
};
static const uint8_t cmc_kof2000_type0_t03[256] = {
    0x10,0x61,0xf1,0x78,0x85,0x52,0x68,0xe3,0x12,0x0d,0xfa,0xf0,0xc9,0x36,0x5e,0x3d,
    0xf9,0xa6,0x01,0x2e,0xc7,0x84,0xea,0x2b,0x6d,0x14,0x38,0x4f,0x55,0x1c,0x9d,0xa7,
    0x7a,0xc6,0xf8,0x9a,0xe6,0x42,0xb5,0xed,0x7d,0x3a,0xb1,0x05,0x43,0x4a,0x22,0xfd,
    0xac,0xa4,0x31,0xc3,0x32,0x76,0x95,0x9e,0x7e,0x88,0x8e,0xa2,0x97,0x18,0xbe,0x2a,
    0xf5,0xd6,0xca,0xcc,0x72,0x3b,0x87,0x6c,0xde,0x75,0xd7,0x21,0xcb,0x0b,0xdd,0xe7,
    0xe1,0x65,0xaa,0xb9,0x44,0xfb,0x66,0x15,0x1a,0x3c,0x98,0xcf,0x8a,0xdf,0x37,0xa5,
    0x2f,0x67,0xd2,0x83,0xb6,0x6b,0xfc,0xe0,0xb4,0x7c,0x08,0xdc,0x93,0x30,0xab,0xe4,
    0x19,0xc2,0x8b,0xeb,0xa0,0x0a,0xc8,0x03,0xc0,0x4b,0x64,0x71,0x86,0x9c,0x9b,0x16,
    0x79,0xff,0x70,0x09,0x8c,0xd0,0xf6,0x53,0x07,0x73,0xd4,0x89,0xb3,0x00,0xe9,0xfe,
    0xec,0x8f,0xbc,0xb2,0x1e,0x5d,0x11,0x35,0xa9,0x06,0x59,0x9f,0xc1,0xd3,0x7b,0xf2,
    0xc5,0x77,0x4e,0x39,0x20,0xd5,0x6a,0x82,0xda,0x45,0xf3,0x33,0x81,0x23,0xba,0xe2,
    0x1d,0x5f,0x5c,0x51,0x49,0xae,0x8d,0xc4,0xa8,0xf7,0x1f,0x0f,0x34,0x28,0xa1,0xd9,
    0x27,0xd8,0x4c,0x2c,0xbf,0x91,0x3e,0x69,0x57,0x41,0x25,0x0c,0x5a,0x90,0x92,0xb0,
    0x63,0x6f,0x40,0xaf,0x74,0xb8,0x2d,0x80,0xbb,0x46,0x94,0xe5,0x29,0xee,0xb7,0x1b,
    0x96,0xad,0x13,0x0e,0x58,0x99,0x60,0x4d,0x17,0x26,0xce,0xe8,0xdb,0xef,0x24,0xa3,
    0x6e,0x7f,0x54,0x3f,0x02,0xd1,0x5b,0x50,0x56,0x48,0xf4,0xbd,0x62,0x47,0x04,0xcd,
};
static const uint8_t cmc_kof2000_type0_t12[256] = {
    0xf4,0x28,0xb4,0x8f,0xfa,0xeb,0x8e,0x54,0x2b,0x49,0xd1,0x76,0x71,0x47,0x8b,0x57,
    0x92,0x85,0x7c,0xb8,0x5c,0x22,0xf9,0x26,0xbc,0x5b,0x6d,0x67,0xae,0x5f,0x6f,0xf5,
    0x9f,0x48,0x66,0x40,0x0d,0x11,0x4e,0xb2,0x6b,0x35,0x15,0x0f,0x18,0x25,0x1d,0xba,
    0xd3,0x69,0x79,0xec,0xa8,0x8c,0xc9,0x7f,0x4b,0xdb,0x51,0xaf,0xca,0xe2,0xb3,0x81,
    0x12,0x5e,0x7e,0x38,0xc8,0x95,0x01,0xff,0xfd,0xfb,0xf2,0x74,0x62,0x14,0xa5,0x98,
    0xa6,0xda,0x80,0x53,0xe8,0x56,0xac,0x1b,0x52,0xd0,0xf1,0x45,0x42,0xb6,0x1a,0x4a,
    0x3a,0x99,0xfc,0xd2,0x9c,0xcf,0x31,0x2d,0xdd,0x86,0x2f,0x29,0xe1,0x03,0x19,0xa2,
    0x41,0x33,0x83,0x90,0xc1,0xbf,0x0b,0x08,0x3d,0xd8,0x8d,0x6c,0x39,0xa0,0xe3,0x55,
    0x02,0x50,0x46,0xe6,0xc3,0x82,0x36,0x13,0x75,0xab,0x27,0xd7,0x1f,0x0a,0xd4,0x89,
    0x59,0x4f,0xc0,0x5d,0xc6,0xf7,0x88,0xbd,0x3c,0x00,0xef,0xcd,0x05,0x1c,0xaa,0x9b,
    0xed,0x7a,0x61,0x17,0x93,0xfe,0x23,0xb9,0xf3,0x68,0x78,0xf6,0x5a,0x7b,0xe0,0xe4,
    0xa3,0xee,0x16,0x72,0xc7,0x3b,0x8a,0x37,0x2a,0x70,0xa9,0x2c,0x21,0xf8,0x24,0x09,
    0xce,0x20,0x9e,0x06,0x87,0xc5,0x04,0x64,0x43,0x7d,0x4d,0x10,0xd6,0xa4,0x94,0x4c,
    0x60,0xde,0xdf,0x58,0xb1,0x44,0x3f,0xb0,0xd9,0xe5,0xcb,0xbb,0xbe,0xea,0x07,0x34,
    0x73,0x6a,0x77,0xf0,0x9d,0x0c,0x2e,0x0e,0x91,0x9a,0xcc,0xc2,0xb7,0x63,0x97,0xd5,
    0xdc,0xc4,0x32,0xe7,0x84,0x3e,0x30,0xa1,0x1e,0xb5,0x6e,0x65,0xe9,0xad,0xa7,0x96,
};
static const uint8_t cmc_kof2000_type1_t03[256] = {
    0x9a,0x2f,0xcc,0x4e,0x40,0x69,0xac,0xca,0xa5,0x7b,0x0a,0x61,0x91,0x0d,0x55,0x74,
    0xcd,0x8b,0x0b,0x80,0x09,0x5e,0x38,0xc7,0xda,0xbf,0xf5,0x37,0x23,0x31,0x33,0xe9,
    0xae,0x87,0xe5,0xfa,0x6e,0x5c,0xad,0xf4,0x76,0x62,0x9f,0x2e,0x01,0xe2,0xf6,0x47,
    0x8c,0x7c,0xaa,0x98,0xb5,0x92,0x51,0xec,0x5f,0x07,0x5d,0x6f,0x16,0xa1,0x1d,0xa9,
    0x48,0x45,0xf0,0x6a,0x9c,0x1e,0x11,0xa0,0x06,0x46,0xd5,0xf1,0x73,0xed,0x94,0xf7,
    0xc3,0x57,0x1b,0xe0,0x97,0xb1,0xa4,0xa7,0x24,0xe7,0x2b,0x05,0x5b,0x34,0x0c,0xb8,
    0x0f,0x9b,0xc8,0x4d,0x5a,0xa6,0x86,0x3e,0x14,0x29,0x84,0x58,0x90,0xdb,0x2d,0x54,
    0x9d,0x82,0xd4,0x7d,0xc6,0x67,0x41,0x89,0xc1,0x13,0xb0,0x9e,0x81,0x6d,0xa8,0x59,
    0xbd,0x39,0x8e,0xe6,0x25,0x8f,0xd9,0xa2,0xe4,0x53,0xc5,0x72,0x7e,0x36,0x4a,0x4f,
    0x52,0xc2,0x22,0x2a,0xce,0x3c,0x21,0x2c,0x00,0xd7,0x75,0x8a,0x27,0xee,0x43,0xfe,
    0xcb,0x6b,0xb9,0xa3,0x78,0xb7,0x85,0x02,0x20,0xd0,0x83,0xc4,0x12,0xf9,0xfd,0xd8,
    0x79,0x64,0x3a,0x49,0x03,0xb4,0xc0,0xf2,0xdf,0x15,0x93,0x08,0x35,0xff,0x70,0xdd,
    0x28,0x6c,0x0e,0x04,0xde,0x7a,0x65,0xd2,0xab,0x42,0x95,0xe1,0x3f,0x3b,0x7f,0x66,
    0xd1,0x8d,0xe3,0xbb,0x1c,0xfc,0x77,0x1a,0x88,0x18,0x19,0x68,0x1f,0x56,0xd6,0xe8,
    0xb6,0xbc,0xd3,0xea,0x3d,0x26,0xb3,0xc9,0x44,0xdc,0xf3,0x32,0x30,0xef,0x96,0x4c,
    0xaf,0x17,0xf8,0xfb,0x60,0x50,0xeb,0x4b,0x99,0x63,0xba,0xb2,0x71,0xcf,0x10,0xbe,
};
static const uint8_t cmc_kof2000_type1_t12[256] = {
    0xda,0xa7,0xd6,0x6e,0x2f,0x5e,0xf0,0x3f,0xa4,0xce,0xd3,0xfd,0x46,0x2a,0xac,0xc9,
    0xbe,0xeb,0x9f,0xd5,0x3c,0x61,0x96,0x11,0xd0,0x38,0xca,0x06,0xed,0x1b,0x65,0xe7,
    0x23,0xdd,0xd9,0x05,0xbf,0x5b,0x5d,0xa5,0x95,0x00,0xec,0xf1,0x01,0xa9,0xa6,0xfc,
    0xbb,0x54,0xe3,0x2e,0x92,0x58,0x0a,0x7b,0xb6,0xcc,0xb1,0x5f,0x14,0x35,0x72,0xff,
    0xe6,0x52,0xd7,0x8c,0xf3,0x43,0xaf,0x9c,0xc0,0x4f,0x0c,0x42,0x8e,0xef,0x80,0xcd,
    0x1d,0x7e,0x88,0x3b,0x98,0xa1,0xad,0xe4,0x9d,0x8d,0x2b,0x56,0xb5,0x50,0xdf,0x66,
    0x6d,0xd4,0x60,0x09,0xe1,0xee,0x4a,0x47,0xf9,0xfe,0x73,0x07,0x89,0xa8,0x39,0xea,
    0x82,0x9e,0xcf,0x26,0xb2,0x4e,0xc3,0x59,0xf2,0x3d,0x9a,0xb0,0x69,0xf7,0xbc,0x34,
    0xe5,0x36,0x22,0xfb,0x57,0x71,0x99,0x6c,0x83,0x30,0x55,0xc2,0xbd,0xf4,0x77,0xe9,
    0x76,0x97,0xa0,0xe0,0xb9,0x86,0x6b,0xa3,0x84,0x67,0x1a,0x70,0x02,0x5a,0x41,0x5c,
    0x25,0x81,0xaa,0x28,0x78,0x4b,0xc6,0x64,0x53,0x16,0x4d,0x8b,0x20,0x93,0xae,0x0f,
    0x94,0x2c,0x3a,0xc7,0x62,0xe8,0xc4,0xdb,0x04,0xc5,0xfa,0x29,0x48,0xd1,0x08,0x24,
    0x0d,0xe2,0xd8,0x10,0xb4,0x91,0x8a,0x13,0x0e,0xdc,0xd2,0x79,0xb8,0xf8,0xba,0x2d,
    0xcb,0xf5,0x7d,0x37,0x51,0x40,0x31,0xa2,0x0b,0x18,0x63,0x7f,0xb3,0xab,0x9b,0x87,
    0xf6,0x90,0xde,0xc8,0x27,0x45,0x7c,0x1c,0x85,0x68,0x33,0x19,0x03,0x75,0x15,0x7a,
    0x1f,0x49,0x8f,0x4c,0xc1,0x44,0x17,0x12,0x6f,0x32,0xb7,0x3e,0x74,0x1e,0x21,0x6a,
};
static const uint8_t cmc_kof2000_address_8_15_xor1[256] = {
    0xfc,0x9b,0x1c,0x35,0x72,0x53,0xd6,0x7d,0x84,0xa4,0xc5,0x93,0x7b,0xe7,0x47,0xd5,
    0x24,0xa2,0xfa,0x19,0x0c,0xb1,0x8c,0xb9,0x9d,0xd8,0x59,0x4f,0x3c,0xb2,0x78,0x4a,
    0x2a,0x96,0x9a,0xf1,0x1f,0x22,0xa8,0x5b,0x67,0xa3,0x0f,0x00,0xfb,0xdf,0xeb,0x0a,
    0x57,0xb8,0x25,0xd7,0xf0,0x6b,0x0b,0x31,0x95,0x23,0x2d,0x5c,0x27,0xc7,0xf4,0x55,
    0x1a,0xf7,0x74,0xbe,0xd3,0xac,0x3d,0xc1,0x7f,0xbd,0x28,0x01,0x10,0xe5,0x09,0x37,
    0x1e,0x58,0xaf,0x17,0xf2,0x16,0x30,0x92,0x36,0x68,0xe6,0xd4,0xea,0xb7,0x75,0x54,
    0x77,0x41,0xb4,0x8d,0xe0,0xf3,0x51,0x03,0xa9,0xe8,0x66,0xab,0x29,0xa5,0xed,0xcb,
    0xd1,0xaa,0xf5,0xdb,0x4c,0x42,0x97,0x8a,0xae,0xc9,0x6e,0x04,0x33,0x85,0xdd,0x2b,
    0x6f,0xef,0x12,0x21,0x7a,0xa1,0x5a,0x91,0xc8,0xcc,0xc0,0xa7,0x60,0x3e,0x56,0x2f,
    0xe4,0x71,0x99,0xc2,0xa0,0x45,0x80,0x65,0xbb,0x87,0x69,0x81,0x73,0xca,0xf6,0x46,
    0x43,0xda,0x26,0x7e,0x8f,0xe1,0x8b,0xfd,0x50,0x79,0xba,0xc6,0x63,0x4b,0xb3,0x8e,
    0x34,0xe2,0x48,0x14,0xcd,0xe3,0xc4,0x05,0x13,0x40,0x06,0x6c,0x88,0xb0,0xe9,0x1b,
    0x4d,0xf8,0x76,0x02,0x44,0x94,0xcf,0x32,0xfe,0xce,0x3b,0x5d,0x2c,0x89,0x5f,0xdc,
    0xd2,0x9c,0x6a,0xec,0x18,0x6d,0x0e,0x86,0xff,0x5e,0x9e,0xee,0x11,0xd0,0x49,0x52,
    0x4e,0x61,0x90,0x0d,0xc3,0x39,0x15,0x83,0xb5,0x62,0x3f,0x70,0x7c,0xad,0x20,0xbf,
    0x2e,0x08,0x1d,0xf9,0xb6,0xa6,0x64,0x07,0x82,0x38,0x98,0x3a,0x9f,0xde,0xbc,0xd9,
};
static const uint8_t cmc_kof2000_address_8_15_xor2[256] = {
    0x00,0xbe,0x06,0x5a,0xfa,0x42,0x15,0xf2,0x3f,0x0a,0x84,0x93,0x4e,0x78,0x3b,0x89,
    0x32,0x98,0xa2,0x87,0x73,0xdd,0x26,0xe5,0x05,0x71,0x08,0x6e,0x9b,0xe0,0xdf,0x9e,
    0xfc,0x83,0x81,0xef,0xb2,0xc0,0xc3,0xbf,0xa7,0x6d,0x1b,0x95,0xed,0xb9,0x3e,0x13,
    0xb0,0x47,0x9c,0x7a,0x24,0x41,0x68,0xd0,0x36,0x0b,0xb5,0xc2,0x67,0xf7,0x54,0x92,
    0x1e,0x44,0x86,0x2b,0x94,0xcc,0xba,0x23,0x0d,0xca,0x6b,0x4c,0x2a,0x9a,0x2d,0x8b,
    0xe3,0x52,0x29,0xf0,0x21,0xbd,0xbb,0x1f,0xa3,0xab,0xf8,0x46,0xb7,0x45,0x82,0x5e,
    0xdb,0x07,0x5d,0xe9,0x9d,0x1a,0x48,0xce,0x91,0x12,0xd4,0xee,0xa9,0x39,0xf1,0x18,
    0x2c,0x22,0x8a,0x7e,0x34,0x4a,0x8c,0xc1,0x14,0xf3,0x20,0x35,0xd9,0x96,0x33,0x77,
    0x9f,0x76,0x7c,0x90,0xc6,0xd5,0xa1,0x5b,0xac,0x75,0xc7,0x0c,0xb3,0x17,0xd6,0x99,
    0x56,0xa6,0x3d,0x1d,0xb1,0x2e,0xd8,0xbc,0x2f,0xde,0x60,0x55,0x6c,0x40,0xcd,0x43,
    0xff,0xad,0x38,0x79,0x51,0xc8,0x0e,0x5f,0xc4,0x66,0xcb,0xa8,0x7d,0xa4,0x3a,0xea,
    0x27,0x7b,0x70,0x8e,0x5c,0x19,0x0f,0x80,0x6f,0x8f,0x10,0xf9,0x49,0x85,0x69,0x7f,
    0xeb,0x1c,0x01,0x65,0x37,0xa5,0x28,0xe4,0x6a,0x03,0x04,0xd1,0x31,0x11,0x30,0xfb,
    0x88,0x97,0xd3,0xf6,0xc5,0x4d,0xf5,0x3c,0xe8,0x61,0xdc,0xd2,0xb4,0xb8,0xa0,0xae,
    0x16,0x25,0x02,0x09,0xfe,0xcf,0x53,0x63,0xaf,0x59,0xf4,0xe1,0xec,0xd7,0xe7,0x50,
    0xe2,0xc9,0xaa,0x4b,0x8d,0x4f,0xe6,0x64,0xda,0x74,0xb6,0x72,0x57,0x62,0xfd,0x58,
};
static const uint8_t cmc_kof2000_address_16_23_xor1[256] = {
    0x45,0x9f,0x6e,0x2f,0x28,0xbc,0x5e,0x6d,0xda,0xb5,0x0d,0xb8,0xc0,0x8e,0xa2,0x32,
    0xee,0xcd,0x8d,0x48,0x8c,0x27,0x14,0xeb,0x65,0xd7,0xf2,0x93,0x99,0x90,0x91,0xfc,
    0x5f,0xcb,0xfa,0x75,0x3f,0x26,0xde,0x72,0x33,0x39,0xc7,0x1f,0x88,0x79,0x73,0xab,
    0x4e,0x36,0x5d,0x44,0xd2,0x41,0xa0,0x7e,0xa7,0x8b,0xa6,0xbf,0x03,0xd8,0x86,0xdc,
    0x2c,0xaa,0x70,0x3d,0x46,0x07,0x80,0x58,0x0b,0x2b,0xe2,0xf0,0xb1,0xfe,0x42,0xf3,
    0xe9,0xa3,0x85,0x78,0xc3,0xd0,0x5a,0xdb,0x1a,0xfb,0x9d,0x8a,0xa5,0x12,0x0e,0x54,
    0x8f,0xc5,0x6c,0xae,0x25,0x5b,0x4b,0x17,0x02,0x9c,0x4a,0x24,0x40,0xe5,0x9e,0x22,
    0xc6,0x49,0x62,0xb6,0x6b,0xbb,0xa8,0xcc,0xe8,0x81,0x50,0x47,0xc8,0xbe,0x5c,0xa4,
    0xd6,0x94,0x4f,0x7b,0x9a,0xcf,0xe4,0x59,0x7a,0xa1,0xea,0x31,0x37,0x13,0x2d,0xaf,
    0x21,0x69,0x19,0x1d,0x6f,0x16,0x98,0x1e,0x08,0xe3,0xb2,0x4d,0x9b,0x7f,0xa9,0x77,
    0xed,0xbd,0xd4,0xd9,0x34,0xd3,0xca,0x09,0x18,0x60,0xc9,0x6a,0x01,0xf4,0xf6,0x64,
    0xb4,0x3a,0x15,0xac,0x89,0x52,0x68,0x71,0xe7,0x82,0xc1,0x0c,0x92,0xf7,0x30,0xe6,
    0x1c,0x3e,0x0f,0x0a,0x67,0x35,0xba,0x61,0xdd,0x29,0xc2,0xf8,0x97,0x95,0xb7,0x3b,
    0xe0,0xce,0xf9,0xd5,0x06,0x76,0xb3,0x05,0x4c,0x04,0x84,0x3c,0x87,0x23,0x63,0x7c,
    0x53,0x56,0xe1,0x7d,0x96,0x1b,0xd1,0xec,0x2a,0x66,0xf1,0x11,0x10,0xff,0x43,0x2e,
    0xdf,0x83,0x74,0xf5,0x38,0x20,0xfd,0xad,0xc4,0xb9,0x55,0x51,0xb0,0xef,0x00,0x57,
};
static const uint8_t cmc_kof2000_address_16_23_xor2[256] = {
    0x00,0xb8,0xf0,0x34,0xca,0x21,0x3c,0xf9,0x01,0x8e,0x75,0x70,0xec,0x13,0x27,0x96,
    0xf4,0x5b,0x88,0x1f,0xeb,0x4a,0x7d,0x9d,0xbe,0x02,0x14,0xaf,0xa2,0x06,0xc6,0xdb,
    0x35,0x6b,0x74,0x45,0x7b,0x29,0xd2,0xfe,0xb6,0x15,0xd0,0x8a,0xa9,0x2d,0x19,0xf6,
    0x5e,0x5a,0x90,0xe9,0x11,0x33,0xc2,0x47,0x37,0x4c,0x4f,0x59,0xc3,0x04,0x57,0x1d,
    0xf2,0x63,0x6d,0x6e,0x31,0x95,0xcb,0x3e,0x67,0xb2,0xe3,0x98,0xed,0x8d,0xe6,0xfb,
    0xf8,0xba,0x5d,0xd4,0x2a,0xf5,0x3b,0x82,0x05,0x16,0x44,0xef,0x4d,0xe7,0x93,0xda,
    0x9f,0xbb,0x61,0xc9,0x53,0xbd,0x76,0x78,0x52,0x36,0x0c,0x66,0xc1,0x10,0xdd,0x7a,
    0x84,0x69,0xcd,0xfd,0x58,0x0d,0x6c,0x89,0x68,0xad,0x3a,0xb0,0x4b,0x46,0xc5,0x03,
    0xb4,0xf7,0x30,0x8c,0x4e,0x60,0x73,0xa1,0x8b,0xb1,0x62,0xcc,0xd1,0x08,0xfc,0x77,
    0x7e,0xcf,0x56,0x51,0x07,0xa6,0x80,0x92,0xdc,0x0b,0xa4,0xc7,0xe8,0xe1,0xb5,0x71,
    0xea,0xb3,0x2f,0x94,0x18,0xe2,0x3d,0x49,0x65,0xaa,0xf1,0x91,0xc8,0x99,0x55,0x79,
    0x86,0xa7,0x26,0xa0,0xac,0x5f,0xce,0x6a,0x5c,0xf3,0x87,0x8f,0x12,0x1c,0xd8,0xe4,
    0x9b,0x64,0x2e,0x1e,0xd7,0xc0,0x17,0xbc,0xa3,0xa8,0x9a,0x0e,0x25,0x40,0x41,0x50,
    0xb9,0xbf,0x28,0xdf,0x32,0x54,0x9e,0x48,0xd5,0x2b,0x42,0xfa,0x9c,0x7f,0xd3,0x85,
    0x43,0xde,0x81,0x0f,0x24,0xc4,0x38,0xae,0x83,0x1b,0x6f,0x7c,0xe5,0xff,0x1a,0xd9,
    0x3f,0xb7,0x22,0x97,0x09,0xe0,0xa5,0x20,0x23,0x2c,0x72,0xd6,0x39,0xab,0x0a,0xee,
};
static const uint8_t cmc_kof2000_address_0_7_xor[256] = {
    0x26,0x48,0x06,0x9b,0x21,0xa9,0x1b,0x76,0xc9,0xf8,0xb4,0x67,0xe4,0xff,0x99,0xf7,
    0x15,0x9e,0x62,0x00,0x72,0x4d,0xa0,0x4f,0x02,0xf1,0xea,0xef,0x0b,0xf3,0xeb,0xa6,
    0x93,0x78,0x6f,0x7c,0xda,0xd4,0x7b,0x05,0xe9,0xc6,0xd6,0xdb,0x50,0xce,0xd2,0x01,
    0xb5,0xe8,0xe0,0x2a,0x08,0x1a,0xb8,0xe3,0xf9,0xb1,0xf4,0x8b,0x39,0x2d,0x85,0x9c,
    0x55,0x73,0x63,0x40,0x38,0x96,0xdc,0xa3,0xa2,0xa1,0x25,0x66,0x6d,0x56,0x8e,0x10,
    0x0f,0x31,0x1c,0xf5,0x28,0x77,0x0a,0xd1,0x75,0x34,0xa4,0xfe,0x7d,0x07,0x51,0x79,
    0x41,0x90,0x22,0x35,0x12,0xbb,0xc4,0xca,0xb2,0x1f,0xcb,0xc8,0xac,0xdd,0xd0,0x0d,
    0xfc,0xc5,0x9d,0x14,0xbc,0x83,0xd9,0x58,0xc2,0x30,0x9a,0x6a,0xc0,0x0c,0xad,0xf6,
    0x5d,0x74,0x7f,0x2f,0xbd,0x1d,0x47,0xd5,0xe6,0x89,0xcf,0xb7,0xd3,0x59,0x36,0x98,
    0xf0,0xfb,0x3c,0xf2,0x3f,0xa7,0x18,0x82,0x42,0x5c,0xab,0xba,0xde,0x52,0x09,0x91,
    0xaa,0x61,0xec,0xd7,0x95,0x23,0xcd,0x80,0xa5,0x68,0x60,0x27,0x71,0xe1,0x2c,0x2e,
    0x8d,0x2b,0x57,0x65,0xbf,0xc1,0x19,0xc7,0x49,0x64,0x88,0x4a,0xcc,0x20,0x4e,0xd8,
    0x3b,0x4c,0x13,0x5f,0x9f,0xbe,0x5e,0x6e,0xfd,0xe2,0xfa,0x54,0x37,0x0e,0x16,0x7a,
    0x6c,0x33,0xb3,0x70,0x84,0x7e,0xc3,0x04,0xb0,0xae,0xb9,0x81,0x03,0x29,0xdf,0x46,
    0xe5,0x69,0xe7,0x24,0x92,0x5a,0x4b,0x5b,0x94,0x11,0x3a,0x3d,0x87,0xed,0x97,0xb6,
    0x32,0x3e,0x45,0xaf,0x1e,0x43,0x44,0x8c,0x53,0x86,0x6b,0xee,0xa8,0x8a,0x8f,0x17,
};
static const uint8_t cmc_m1_address_8_15_xor[256] = {
    0x0a,0x72,0xb7,0xaf,0x67,0xde,0x1d,0xb1,0x78,0xc4,0x4f,0xb5,0x4b,0x18,0x76,0xdd,
    0x11,0xe2,0x36,0xa1,0x82,0x03,0x98,0xa0,0x10,0x5f,0x3f,0xd6,0x1f,0x90,0x6a,0x0b,
    0x70,0xe0,0x64,0xcb,0x9f,0x38,0x8b,0x53,0x04,0xca,0xf8,0xd0,0x07,0x68,0x56,0x32,
    0xae,0x1c,0x2e,0x48,0x63,0x92,0x9a,0x9c,0x44,0x85,0x41,0x40,0x09,0xc0,0xc8,0xbf,
    0xea,0xbb,0xf7,0x2d,0x99,0x21,0xf6,0xba,0x15,0xce,0xab,0xb0,0x2a,0x60,0xbc,0xf1,
    0xf0,0x9e,0xd5,0x97,0xd8,0x4e,0x14,0x9d,0x42,0x4d,0x2c,0x5c,0x2b,0xa6,0xe1,0xa7,
    0xef,0x25,0x33,0x7a,0xeb,0xe7,0x1b,0x6d,0x4c,0x52,0x26,0x62,0xb6,0x35,0xbe,0x80,
    0x01,0xbd,0xfd,0x37,0xf9,0x47,0x55,0x71,0xb4,0xf2,0xff,0x27,0xfa,0x23,0xc9,0x83,
    0x17,0x39,0x13,0x0d,0xc7,0x86,0x16,0xec,0x49,0x6f,0xfe,0x34,0x05,0x8f,0x00,0xe6,
    0xa4,0xda,0x7b,0xc1,0xf3,0xf4,0xd9,0x75,0x28,0x66,0x87,0xa8,0x45,0x6c,0x20,0xe9,
    0x77,0x93,0x7e,0x3c,0x1e,0x74,0xf5,0x8c,0x3e,0x94,0xd4,0xc2,0x5a,0x06,0x0e,0xe8,
    0x3d,0xa9,0xb2,0xe3,0xe4,0x22,0xcf,0x24,0x8e,0x6b,0x8a,0x8d,0x84,0x4a,0xd2,0x91,
    0x88,0x79,0x57,0xa5,0x0f,0xcd,0xb9,0xac,0x3b,0xaa,0xb3,0xd1,0xee,0x31,0x81,0x7c,
    0xd7,0x89,0xd3,0x96,0x43,0xc5,0xc6,0xc3,0x69,0x7f,0x46,0xdf,0x30,0x5b,0x6e,0xe5,
    0x08,0x95,0x9b,0xfb,0xb8,0x58,0x0c,0x61,0x50,0x5d,0x3a,0xa2,0x29,0x12,0xfc,0x51,
    0x7d,0x1a,0x02,0x65,0x54,0x5e,0x19,0xcc,0xdc,0xdb,0x73,0xed,0xad,0x59,0x2f,0xa3,
};
static const uint8_t cmc_m1_address_0_7_xor[256] = {
    0xf4,0xbc,0x02,0xf7,0x2c,0x3d,0xe8,0xd9,0x50,0x62,0xec,0xbd,0x53,0x73,0x79,0x61,
    0x00,0x34,0xcf,0xa2,0x63,0x28,0x90,0xaf,0x44,0x3b,0xc5,0x8d,0x3a,0x46,0x07,0x70,
    0x66,0xbe,0xd8,0x8b,0xe9,0xa0,0x4b,0x98,0xdc,0xdf,0xe2,0x16,0x74,0xf1,0x37,0xf5,
    0xb7,0x21,0x81,0x01,0x1c,0x1b,0x94,0x36,0x09,0xa1,0x4a,0x91,0x30,0x92,0x9b,0x9a,
    0x29,0xb1,0x38,0x4d,0x55,0xf2,0x56,0x18,0x24,0x47,0x9d,0x3f,0x80,0x1f,0x22,0xa4,
    0x11,0x54,0x84,0x0d,0x25,0x48,0xee,0xc6,0x59,0x15,0x03,0x7a,0xfd,0x6c,0xc3,0x33,
    0x5b,0xc4,0x7b,0x5a,0x05,0x7f,0xa6,0x40,0xa9,0x5d,0x41,0x8a,0x96,0x52,0xd3,0xf0,
    0xab,0x72,0x10,0x88,0x6f,0x95,0x7c,0xa8,0xcd,0x9c,0x5f,0x32,0xae,0x85,0x39,0xac,
    0xe5,0xd7,0xfb,0xd4,0x08,0x23,0x19,0x65,0x6b,0xa7,0x93,0xbb,0x2b,0xbf,0xb8,0x35,
    0xd0,0x06,0x26,0x68,0x3e,0xdd,0xb9,0x69,0x2a,0xb2,0xde,0x87,0x45,0x58,0xff,0x3c,
    0x9e,0x7d,0xda,0xed,0x49,0x8c,0x14,0x8e,0x75,0x2f,0xe0,0x6e,0x78,0x6d,0x20,0xd2,
    0xfa,0x2d,0x51,0xcc,0xc7,0xe7,0x1d,0x27,0x97,0xfc,0x31,0xdb,0xf8,0x42,0xe3,0x99,
    0x5e,0x83,0x0e,0xb4,0x2e,0xf6,0xc0,0x0c,0x4c,0x57,0xb6,0x64,0x0a,0x17,0xa3,0xc1,
    0x77,0x12,0xfe,0xe6,0x8f,0x13,0x71,0xe4,0xf9,0xad,0x9f,0xce,0xd5,0x89,0x7e,0x0f,
    0xc2,0x86,0xf3,0x67,0xba,0x60,0x43,0xc9,0x04,0xb3,0xb0,0x1e,0xb5,0xc8,0xeb,0xa5,
    0x76,0xea,0x5c,0x82,0x1a,0x4f,0xaa,0xca,0xe1,0x0b,0x4e,0xcb,0x6a,0xef,0xd1,0xd6,
};

typedef struct {
    const uint8_t *t03, *t12, *u03, *u12;
    const uint8_t *a815_1, *a815_2, *a1623_1, *a1623_2, *a07;
} cmc_keys;
static const cmc_keys cmc42_keys = {
    cmc_kof99_type0_t03, cmc_kof99_type0_t12, cmc_kof99_type1_t03, cmc_kof99_type1_t12,
    cmc_kof99_address_8_15_xor1, cmc_kof99_address_8_15_xor2,
    cmc_kof99_address_16_23_xor1, cmc_kof99_address_16_23_xor2, cmc_kof99_address_0_7_xor };
static const cmc_keys cmc50_keys = {
    cmc_kof2000_type0_t03, cmc_kof2000_type0_t12, cmc_kof2000_type1_t03, cmc_kof2000_type1_t12,
    cmc_kof2000_address_8_15_xor1, cmc_kof2000_address_8_15_xor2,
    cmc_kof2000_address_16_23_xor1, cmc_kof2000_address_16_23_xor2, cmc_kof2000_address_0_7_xor };



static void cmc_dec1(uint8_t *r0, uint8_t *r1, uint8_t c0, uint8_t c1,
                     const uint8_t *t0hi, const uint8_t *t0lo, const uint8_t *t1,
                     const uint8_t *a07, uint32_t base, int invert)
{
    uint8_t tmp = t1[(base & 0xff) ^ a07[(base >> 8) & 0xff]];
    uint8_t xor0 = (uint8_t)((t0hi[(base >> 8) & 0xff] & 0xfe) | (tmp & 1));
    uint8_t xor1 = (uint8_t)((tmp & 0xfe) | (t0lo[(base >> 8) & 0xff] & 1));
    if (invert) { *r0 = c1 ^ xor0; *r1 = c0 ^ xor1; }
    else        { *r0 = c0 ^ xor0; *r1 = c1 ^ xor1; }
}


typedef struct { uint8_t *rom, *buf; const cmc_keys *K; } cmc_p1_ctx;
static void cmc_p1(void *a, uint32_t lo, uint32_t hi)
{
    cmc_p1_ctx *c = (cmc_p1_ctx *)a;
    uint8_t *rom = c->rom, *buf = c->buf; const cmc_keys *K = c->K;
    for (uint32_t rp = lo; rp < hi; rp++) {
        cmc_dec1(&buf[4*rp+0], &buf[4*rp+3], rom[4*rp+0], rom[4*rp+3],
                 K->t03, K->t12, K->u03, K->a07, rp, (rp >> 8) & 1);
        cmc_dec1(&buf[4*rp+1], &buf[4*rp+2], rom[4*rp+1], rom[4*rp+2],
                 K->t12, K->t03, K->u12, K->a07, rp,
                 (int)(((rp >> 16) ^ K->a1623_2[(rp >> 8) & 0xff]) & 1));
    }
}
typedef struct { uint8_t *rom, *buf; const cmc_keys *K; uint32_t n; int extra_xor; uint32_t rs; } cmc_p2_ctx;
static void cmc_p2(void *a, uint32_t lo, uint32_t hi)
{
    cmc_p2_ctx *c = (cmc_p2_ctx *)a;
    uint8_t *rom = c->rom, *buf = c->buf; const cmc_keys *K = c->K;
    uint32_t n = c->n; uint32_t rom_size = c->rs; int extra_xor = c->extra_xor;
    for (uint32_t rp = lo; rp < hi; rp++) {
        uint32_t b = rp ^ (uint32_t)extra_xor;
        b ^= (uint32_t)K->a815_1[(b >> 16) & 0xff] << 8;
        b ^= (uint32_t)K->a815_2[b & 0xff] << 8;
        b ^= (uint32_t)K->a1623_1[b & 0xff] << 16;
        b ^= (uint32_t)K->a1623_2[(b >> 8) & 0xff] << 16;
        b ^= K->a07[(b >> 8) & 0xff];
        if (rom_size == 0x3000000) {
            if (rp < 0x2000000/4) b &= (0x2000000/4)-1;
            else b = 0x2000000/4 + (b & ((0x1000000/4)-1));
        } else if (rom_size == 0x6000000) {
            if (rp < 0x4000000/4) b &= (0x4000000/4)-1;
            else b = 0x4000000/4 + (b & ((0x1000000/4)-1));
        } else b &= n - 1;
        rom[4*rp+0] = buf[4*b+0]; rom[4*rp+1] = buf[4*b+1];
        rom[4*rp+2] = buf[4*b+2]; rom[4*rp+3] = buf[4*b+3];
    }
}

static void cmc_gfx_decrypt(uint8_t *rom, uint32_t rom_size, int extra_xor, const cmc_keys *K)
{
    uint32_t n = rom_size / 4;
    uint8_t *buf = calloc(1, rom_size);
    if (!buf) return;
    { cmc_p1_ctx c1 = { rom, buf, K }; parallel_for(cmc_p1, &c1, 0, n, 12); }
    { cmc_p2_ctx c2 = { rom, buf, K, n, extra_xor, rom_size }; parallel_for(cmc_p2, &c2, 0, n, 12); }
    free(buf);
}

static void cmc_sfix_decrypt(uint8_t *rom, uint32_t rom_size, uint8_t *fixed, uint32_t fsz)
{
    uint8_t *src = rom + rom_size - fsz;
    for (uint32_t i = 0; i < fsz; i++)
        fixed[i] = src[(i & ~0x1fu) + ((i & 7) << 2) + ((~i & 8u) >> 2) + ((i & 0x10) >> 4)];
}

static int cmc_m1_scramble(int address, uint16_t key)
{
    static const int p1[8][16] = {
        {15,14,10,7,1,2,3,8,0,12,11,13,6,9,5,4},
        {7,1,8,11,15,9,2,3,5,13,4,14,10,0,6,12},
        {8,6,14,3,10,7,15,1,4,0,2,5,13,11,12,9},
        {2,8,15,9,3,4,11,7,13,6,0,10,1,12,14,5},
        {1,13,6,15,14,3,8,10,9,4,7,12,5,2,0,11},
        {11,15,3,4,7,0,9,2,6,14,12,1,8,5,10,13},
        {10,5,13,8,6,15,1,14,11,9,3,0,12,7,4,2},
        {9,3,7,0,2,12,4,11,14,10,5,8,15,13,1,6},
    };
    int block = (address >> 16) & 7;
    int aux = address & 0xffff;
    static const int kp[16] = {12,0,2,4,8,15,7,13,10,1,3,6,11,9,14,5};
    aux ^= (int)bswp(key, 16, kp);
    int pat[16];
    for (int i = 0; i < 16; i++) pat[i] = p1[block][15 - i];
    aux = (int)bswp((uint32_t)aux, 16, pat);
    aux ^= cmc_m1_address_0_7_xor[(aux >> 8) & 0xff];
    aux ^= (int)cmc_m1_address_8_15_xor[aux & 0xff] << 8;
    static const int fp[16] = {7,15,14,6,5,13,12,4,11,3,10,2,9,1,8,0};
    aux = (int)bswp((uint32_t)aux, 16, fp);
    return (block << 16) | aux;
}

static void cmc50_m1_decrypt(uint8_t *crypt, uint32_t crypt_size, uint8_t *snd)
{
    uint16_t key = 0;
    for (int i = 0; i < 0x10000; i++) key = (uint16_t)(key + crypt[i]);   /* byte sum (verified: word sum breaks kof2000 m1) */
    uint8_t *buf = calloc(1, 0x80000);
    if (!buf) return;
    /* audiocrypt region may be 128K/256K (rotd/pnyaa-style ENCRYPTED_128K):
       MAME reads the 512K scramble through the smaller region with wrap */
    uint32_t cmask = (crypt_size >= 0x10000) ? crypt_size - 1 : 0x7FFFF;
    for (int i = 0; i < 0x80000; i++) buf[i] = crypt[cmc_m1_scramble(i, key) & cmask];
    memcpy(snd, buf, 0x10000);
    memcpy(snd + 0x10000, buf, 0x80000);
    free(buf);
}

/* Apply CMC gfx (+optional sfix from decrypted C tail) to the sprite region.
 * sw_build's sprites buf (load16_byte) == MAME region byte space, so decrypt
 * in place; fixed buf likewise matches MAME fixed region (raw s1 / fill). */
/* svcpcb_s1data_decrypt (MAME neopcb.cpp): s1[i] = bitswap<8>(s1[i]^0xd2, 4,0,7,2,5,1,6,3) */
static uint8_t svc_s1_bitswap(uint8_t x)
{
    static const int p[8] = {3,6,1,5,2,7,0,4};  /* result bit i = src bit p[i] */
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) if (x & (1u << p[i])) r |= 1u << i;
    return r;
}

/* ---- MAME-faithful svcpcb chain (neopcb.cpp init_mslug5pcb/init_svcpcb):
 * svcpcb_gfx_decrypt -> cmc50_gfx_decrypt(kof2000 tables) -> sfix -> s1data.
 * Tables auto-extracted from MAME prot_cmc.cpp; logic verbatim. ---- */
#include "cmc_mame_tables.h"

/* MAME bitswap<N>(x, n-1..0): result bit (n-1-i) = x bit p[i] */
static uint32_t mame_bitswap(uint32_t x, const int8_t *p, int n)
{
    uint32_t r = 0;
    for (int i = 0; i < n; i++)
        if (x & (1u << p[i])) r |= 1u << (n - 1 - i);
    return r;
}

static void svcpcb_gfx_do(uint8_t *rom, uint32_t rom_size)
{
    static const uint8_t xorval[4] = { 0x34, 0x21, 0xc4, 0xe9 };
    static const int8_t b32[32] = { 0x09,0x0d,0x13,0x00,0x17,0x0f,0x03,0x05,
        0x04,0x0c,0x11,0x1e,0x12,0x15,0x0b,0x06,0x1b,0x0a,0x1a,0x1c,0x14,0x02,
        0x0e,0x1d,0x18,0x08,0x01,0x10,0x19,0x1f,0x07,0x16 };
    static const int8_t b24[24] = { 0x17,0x16,0x15,0x04,0x0b,0x0e,0x08,0x0c,
        0x10,0x00,0x0a,0x13,0x03,0x06,0x02,0x07,0x0d,0x01,0x11,0x09,0x14,0x0f,
        0x12,0x05 };
    uint8_t *buf = malloc(rom_size);
    if (!buf) return;
    for (uint32_t i = 0; i < rom_size; i++)
        rom[i] ^= xorval[i & 3];
    for (uint32_t i = 0; i + 4 <= rom_size; i += 4) {
        uint32_t v = rom[i] | rom[i+1] << 8 | rom[i+2] << 16 | (uint32_t)rom[i+3] << 24;
        v = mame_bitswap(v, b32, 32);
        buf[i] = v & 0xff; buf[i+1] = (v >> 8) & 0xff;
        buf[i+2] = (v >> 16) & 0xff; buf[i+3] = (v >> 24) & 0xff;
    }
    for (uint32_t i = 0; i < rom_size / 4; i++) {
        uint32_t ofst = mame_bitswap(i & 0x1fffff, b24, 24);
        ofst ^= 0x0c8923;
        ofst += i & 0xffe00000;
        memcpy(rom + i * 4, buf + ofst * 4, 4);
    }
    free(buf);
}

/* MAME cmc_prot_device::decrypt() */
static void mame_cmc_decrypt(uint8_t *r0, uint8_t *r1, uint8_t c0, uint8_t c1,
                             const uint8_t *t0hi, const uint8_t *t0lo,
                             const uint8_t *t1, int base, int invert)
{
    uint8_t tmp, xor0, xor1;
    tmp = t1[(base & 0xff) ^ mame_kof2000_address_0_7_xor[(base >> 8) & 0xff]];
    xor0 = (t0hi[(base >> 8) & 0xff] & 0xfe) | (tmp & 0x01);
    xor1 = (tmp & 0xfe) | (t0lo[(base >> 8) & 0xff] & 0x01);
    if (invert) { *r0 = c1 ^ xor0; *r1 = c0 ^ xor1; }
    else        { *r0 = c0 ^ xor0; *r1 = c1 ^ xor1; }
}

/* MAME cmc_prot_device::gfx_decrypt() with kof2000 table set (= cmc50) */
static void mame_cmc50_gfx_decrypt(uint8_t *rom, uint32_t rom_size, int extra_xor)
{
    uint8_t *buf = malloc(rom_size);
    if (!buf) return;
    for (uint32_t rpos = 0; rpos + 4 <= rom_size; rpos += 4) {
        mame_cmc_decrypt(buf+rpos+0, buf+rpos+3, rom[rpos+0], rom[rpos+3],
            mame_kof2000_type0_t03, mame_kof2000_type0_t12, mame_kof2000_type1_t03,
            rpos, (rpos >> 8) & 1);
        mame_cmc_decrypt(buf+rpos+1, buf+rpos+2, rom[rpos+1], rom[rpos+2],
            mame_kof2000_type0_t12, mame_kof2000_type0_t03, mame_kof2000_type1_t12,
            rpos, ((rpos >> 16) ^ mame_kof2000_address_16_23_xor2[(rpos >> 8) & 0xff]) & 1);
    }
    uint32_t q = rom_size / 4;
    for (uint32_t rpos = 0; rpos < q; rpos++) {
        int32_t baser = (int32_t)rpos;
        baser ^= extra_xor;
        baser ^= (int32_t)mame_kof2000_address_8_15_xor1[(baser >> 16) & 0xff] << 8;
        baser ^= (int32_t)mame_kof2000_address_8_15_xor2[baser & 0xff] << 8;
        baser ^= (int32_t)mame_kof2000_address_16_23_xor1[baser & 0xff] << 16;
        baser ^= (int32_t)mame_kof2000_address_16_23_xor2[(baser >> 8) & 0xff] << 16;
        baser ^= (int32_t)mame_kof2000_address_0_7_xor[(baser >> 8) & 0xff];
        uint32_t b = (uint32_t)baser & (q - 1);   /* pow2 rom sizes here */
        rom[4*rpos+0] = buf[4*b+0]; rom[4*rpos+1] = buf[4*b+1];
        rom[4*rpos+2] = buf[4*b+2]; rom[4*rpos+3] = buf[4*b+3];
    }
    free(buf);
}

static void cmc_gfx_run(neogeo *ng, int is50, int key, int sfix, int s1data)
{
    uint32_t sz = ng->spr_size & ~3u;   /* odd +1 from load16_byte stride spans */
    if (!ng->spr || sz < 0x80000) return;
    if (s1data) {
        /* svcpcb family full chain (MAME init_mslug5pcb/init_svcpcb).
         * Our spr buffer is bus-order; MAME chain works in region byte
         * space — wordswap in, decrypt, wordswap back (same trick as pvc_run). */
        wordswap(ng->spr, sz);
        svcpcb_gfx_do(ng->spr, sz);
        mame_cmc50_gfx_decrypt(ng->spr, sz, key);
        uint32_t fsz = ng->fixed_size ? ng->fixed_size : 0x20000;
        if (fsz > 0x80000) fsz = 0x80000;
        if (fsz >= sz) fsz = 0x20000;
        if (!ng->fixed) { ng->fixed = calloc(fsz, 1); ng->fixed_size = fsz; }
        uint8_t *fxm = calloc(1, fsz);
        if (!fxm) { wordswap(ng->spr, sz); return; }
        cmc_sfix_decrypt(ng->spr, sz, fxm, fsz);
        for (uint32_t i = 0; i < fsz; i++) fxm[i] = svc_s1_bitswap(fxm[i] ^ 0xd2);
        wordswap(ng->spr, sz);
        wordswap(fxm, fsz);
        for (uint32_t i = 0; i < fsz && i < ng->fixed_size; i++) ng->fixed[i] |= fxm[i];
        free(fxm);
        return;
    }
    cmc_gfx_decrypt(ng->spr, sz, key, is50 ? &cmc50_keys : &cmc42_keys);
    if (sfix) {
        /* MAME: sfix_decrypt(spr, spr_size, fix, FIX_REGION_SIZE) — tail of C */
        uint32_t fsz = ng->fixed_size ? ng->fixed_size : 0x20000;
        if (fsz > 0x80000) fsz = 0x80000;
        if (fsz >= sz) fsz = 0x20000;
        if (!ng->fixed) { ng->fixed = calloc(fsz, 1); ng->fixed_size = fsz; }
        uint8_t *fxm = calloc(1, fsz);
        if (!fxm) return;
        cmc_sfix_decrypt(ng->spr, sz, fxm, fsz);
        if (s1data)
            for (uint32_t i = 0; i < fsz; i++) fxm[i] = svc_s1_bitswap(fxm[i] ^ 0xd2);
        for (uint32_t i = 0; i < fsz && i < ng->fixed_size; i++) ng->fixed[i] |= fxm[i];
        free(fxm);
    }
}


/* PVC decrypt wrapper: sw_build's bus-order array is word-swapped relative
 * to MAME's region byte layout.  Empirically verified on real kof2003 ROM:
 * wordswap -> decrypt (bus-transcribed rules) -> wordswap == MAME result
 * (vec 0010f300, header "NE", region code non-zero). */
static void pvc_run(uint8_t *R, uint32_t sz, void (*dec)(uint8_t *))
{
    wordswap(R, sz);
    dec(R);
    wordswap(R, sz);
}

static uint32_t pvc_bank_base(neogeo *ng)
{
    uint32_t ba = (ng->pvc_ram[0xff8] >> 8) | (ng->pvc_ram[0xff9] << 8);
    ng->pvc_ram[0xff8] = (uint16_t)((ng->pvc_ram[0xff8] & 0xfe00) | 0x00a0);
    ng->pvc_ram[0xff9] &= 0x7fff;
    return ba + 0x100000;
}

/* ---- CTHD2K3 bootleg family (MAME prot_cthd.cpp): cthd2003 / matrim ---- */
static void cthd_fix_do(uint8_t *spr, uint32_t start, uint32_t end,
                        int b3, int b2, int b1, int b0)
{
    uint8_t buf[16 * 128];
    uint8_t *real = spr + start * 128;
    for (uint32_t i = 0; i < (end - start) * 128; i += 16 * 128) {
        for (int j = 0; j < 16; j++) {
            int off = ((j & 1) ? (1 << b0) : 0) + ((j & 2) ? (1 << b1) : 0) +
                      ((j & 4) ? (1 << b2) : 0) + ((j & 8) ? (1 << b3) : 0);
            memcpy(buf + j * 128, real + i + off * 128, 128);
        }
        memcpy(real + i, buf, 16 * 128);
    }
}
static void cthd_gfx_fix(uint8_t *spr, uint32_t start, uint32_t end)
{
    cthd_fix_do(spr, start + 512 * 0, end + 512 * 0, 0, 3, 2, 1);
    cthd_fix_do(spr, start + 512 * 1, end + 512 * 1, 1, 0, 3, 2);
    cthd_fix_do(spr, start + 512 * 2, end + 512 * 2, 2, 1, 0, 3);
    cthd_fix_do(spr, start + 512 * 5, end + 512 * 5, 0, 1, 2, 3);
    cthd_fix_do(spr, start + 512 * 6, end + 512 * 6, 0, 1, 2, 3);
    cthd_fix_do(spr, start + 512 * 7, end + 512 * 7, 0, 2, 3, 1);
}

typedef struct { uint8_t *spr; uint32_t sz; } cthd_c_ctx;
static void cthd_c_run(void *a, uint32_t lo, uint32_t hi)
{
    cthd_c_ctx *c = (cthd_c_ctx *)a;
    for (uint32_t k = lo; k < hi; k++) {
        int i;
        if (k < 125) i = (int)(k / 25) * 200 + (int)(k % 25) * 8;
        else i = 1000 + (int)(k - 125) * 8;
        uint32_t s = (uint32_t)i * 512, e = s + 512;
        if (e * 128 <= c->sz) cthd_gfx_fix(c->spr, s, e);
    }
}

static void cthd2003_c(uint8_t *spr, uint32_t sz)
{
    /* start/end are TILE indices (fix_do multiplies by 128); max = 1016*512*128 < 64MB */
    struct { uint8_t *spr; uint32_t sz; } cc = { spr, sz };
    parallel_for(cthd_c_run, &cc, 0, 128, 12);   /* 5*25 + 3 tail blocks */
}
static void cthd_swap_mid(neogeo *ng, uint8_t *p)  /* MAME tmp: swap [0x8000,0x10000)<->[0x10000,0x18000) */
{
    uint8_t tmp[0x8000];
    (void)ng;
    memcpy(tmp, p + 0x8000, 0x8000);
    memcpy(p + 0x8000, p + 0x10000, 0x8000);
    memcpy(p + 0x10000, tmp, 0x8000);
}
static void cthd2003_decrypt(neogeo *ng)
{
    if (ng->fixed && ng->fixed_size >= 0x20000) cthd_swap_mid(ng, ng->fixed);
    /* MAME's audio swap reads past the 0x20000 audiocpu region (m1 here is a
     * complete 128K image; the swap+copy is OOB UB in MAME) — skip it. */
    if (ng->spr && ng->spr_size >= 0x800000) cthd2003_c(ng->spr, ng->spr_size & ~3u);
}
/* razoola CPU patches; ng->rom is bus-order (words byte-swapped vs MAME region) */
static void cthd_patch(neogeo *ng)
{
    uint8_t *r = ng->rom; uint32_t sz = ng->rom_size;
    if (!r || sz < 0x200000) return;
    uint16_t v;
    v = 0xf94e; memcpy(r + 0xf415a, &v, 2);           /* bswap(4ef9) */
    v = 0x0f00; memcpy(r + 0xf415c, &v, 2);           /* bswap(000f) */
    v = 0xf24c; memcpy(r + 0xf415e, &v, 2);           /* bswap(4cf2) */
    memset(r + 0x1ae290, 0, 0x1ae8d0 - 0x1ae290);
    for (uint32_t i = 0x1f8ef0; i + 4 <= 0x1fa1f0; i += 4) {
        uint16_t a, b;
        memcpy(&a, r + i, 2); memcpy(&b, r + i + 2, 2);
        a = (uint16_t)(a - 0x0077);                   /* bswap(0x7000) */
        b = (uint16_t)(b - 0x1000);                   /* bswap(0x0010) */
        memcpy(r + i, &a, 2); memcpy(r + i + 2, &b, 2);
    }
    memset(r + 0xac500, 0xff, 0xac520 - 0xac500);
    { uint16_t d = 0x03dd;                            /* bswap(dd03) */
      memcpy(r + 0x991d0, &d, 2); memcpy(r + 0x99306, &d, 2);
      memcpy(r + 0x99354, &d, 2); memcpy(r + 0x9943e, &d, 2); }
}

/* kof2k2 family CPU sector shuffles (MAME prot_kof2k2.cpp); sector tables are
 * region-byte offsets; our bus-order rom buffer shares byte offsets for whole
 * 0x80000 blocks so the shuffle is order-independent. */
static void k2k2_shuffle(neogeo *ng, const int *sec, int nsec, uint32_t src_off)
{
    uint32_t span = (uint32_t)nsec * 0x80000;
    if (!ng->rom || ng->rom_size < src_off + span) return;
    uint8_t *buf = calloc(1, span);
    if (!buf) return;
    memcpy(buf, ng->rom + src_off, span);
    for (int i = 0; i < nsec; i++)
        memcpy(ng->rom + src_off + (uint32_t)i * 0x80000, buf + sec[i], 0x80000);
    free(buf);
}
static void matrim_dec68k(neogeo *ng)
{
    static const int sec[8] = {0x100000,0x280000,0x300000,0x180000,0x000000,0x380000,0x200000,0x080000};
    k2k2_shuffle(ng, sec, 8, 0x100000);   /* sec[] are offsets from cpurom+0x100000 */
}
static void samsho5_dec68k(neogeo *ng)
{
    static const int sec[16] = {0x000000,0x080000,0x700000,0x680000,0x500000,0x180000,0x200000,0x480000,
                                0x300000,0x780000,0x600000,0x280000,0x100000,0x580000,0x400000,0x380000};
    k2k2_shuffle(ng, sec, 16, 0);
}

static void pvc_unpack_color(neogeo *ng)
{
    uint16_t pen = ng->pvc_ram[0xff0];
    uint8_t b = (uint8_t)(((pen & 0x000f) << 1) | ((pen & 0x1000) >> 12));
    uint8_t g = (uint8_t)(((pen & 0x00f0) >> 3) | ((pen & 0x2000) >> 13));
    uint8_t r = (uint8_t)(((pen & 0x0f00) >> 7) | ((pen & 0x4000) >> 14));
    uint8_t s = (uint8_t)((pen & 0x8000) >> 15);
    ng->pvc_ram[0xff1] = (uint16_t)((g << 8) | b);
    ng->pvc_ram[0xff2] = (uint16_t)((s << 8) | r);
}
static void pvc_pack_color(neogeo *ng)
{
    uint16_t gb = ng->pvc_ram[0xff4];
    uint16_t sr = ng->pvc_ram[0xff5];
    ng->pvc_ram[0xff6] = (uint16_t)(((gb & 0x001e) >> 1) | ((gb & 0x1e00) >> 5) |
        ((sr & 0x001e) << 7) | ((gb & 0x0001) << 12) | ((gb & 0x0100) << 5) |
        ((sr & 0x0001) << 14) | ((sr & 0x0100) << 7));
}

/* PCFLOW=lo-hi instruction-level PC flow probe (env-gated; near-free when off) */
static struct neogeo *g_pcflow_ng;
static int g_pcflow_lo = -1, g_pcflow_hi = -1;
void linoragx_pc_hook(unsigned int pc)
{
    if (g_pcflow_lo < 0) return;
    int f = g_pcflow_ng ? g_pcflow_ng->frame : -1;
    if (f >= g_pcflow_lo && f <= g_pcflow_hi)
        fprintf(stderr, "F %d %08x\n", f, pc);
}

/* Z80FLOW=lo-hi: instruction-level Z80 PC flow (env-gated) */
static int g_z80flow_lo = -1, g_z80flow_hi = -1;
void linoragx_z80_step(unsigned int pc)
{
    if (g_z80flow_lo < 0) return;
    int f = g_ng ? g_ng->frame : -1;
    if (f >= g_z80flow_lo && f <= g_z80flow_hi) {
        static long zc = 0;
        extern void z80w_force_ei(void*);
        if (pc == 0x24cb && (zc++ % 400) == 0 && g_ng) {
            extern uint8_t z80w_peek(void*, uint32_t);
            extern int z80w_iff1(void*), z80w_im(void*);
            extern void z80w_force_ei(void*);
            fprintf(stderr, "Z24CB fe3c=%02x iff1=%d im=%d\n", z80w_peek(g_ng->z, 0xfe3c),
                    z80w_iff1(g_ng->z), z80w_im(g_ng->z));
        }
        /* HACKTEST: force-enable interrupts while wedged in the 24b0 wait loop */
        if (pc == 0x24b0 && g_ng && getenv("FORCEEI")) z80w_force_ei(g_ng->z);
        if (pc >= 0x1140 && pc <= 0x1148 && g_ng && getenv("M1CHK")) {
            extern uint8_t z80w_peek(void*, uint32_t);
            fprintf(stderr, "M1CHK pc=%04x read=%02x snd=%02x sm1=%02x cv=%d m1v=%d sndsz=%u\n",
                    pc, z80w_peek(g_ng->z, pc),
                    g_ng->snd ? g_ng->snd[pc] : 0xFF,
                    g_ng->sm1 ? g_ng->sm1[pc] : 0xFF,
                    g_ng->snd ? (g_ng->cart_audio || g_ng->m1_view) : 0,
                    g_ng->m1_view, g_ng->snd_size);
        }
        if (pc == 0x0010 && g_ng && getenv("RST10DBG")) {
            extern uint8_t z80w_peek(void*, uint32_t);
            unsigned cmd = z80w_peek(g_ng->z, 0xfe3d);
            unsigned tgt = z_read(0x0cad + 3u*cmd + 1, g_ng) | (z_read(0x0cad + 3u*cmd + 2, g_ng) << 8);
            fprintf(stderr, "RST10 cmd=%02x -> %04x f=%llu\n", cmd, tgt, (unsigned long long)g_ng->frame);
        }
        fprintf(stderr, "Z %d %04x\n", f, pc);
    }
}

neogeo *ng_create(const char *biosdir, const char *gamezip, char *err, size_t errsz)
{
    neogeo *ng = calloc(1, sizeof *ng);
    err[0] = 0;
    if (load_bios(ng, biosdir) != 0) {
        snprintf(err, errsz, "NEO-GEO.ROM not found in %s", biosdir);
        free(ng); return NULL;
    }
    if (!ng->have_zoom) synth_zoomy(ng);

    if (gamezip && gamezip[0]) {
        uint8_t *d; uint32_t n;
        {   const char *gb = strrchr(gamezip,'/'); gb = gb?gb+1:gamezip;
            snprintf(ng->gamename, sizeof ng->gamename, "%s", gb);
            char *dot = strrchr(ng->gamename,'.'); if (dot) *dot = 0;
        }
        {   /* MAME swlist exact layout first (offset/continue/word_swap) */
            char xp[1200]; const char *xe = getenv("LINORAGX_SWLIST");
            if (xe) snprintf(xp, sizeof xp, "%s", xe);
            else snprintf(xp, sizeof xp, "%s/neogeo_swlist.xml", biosdir);
            uint8_t *xb = 0; uint32_t xs = 0;
            if (rom_load_file(xp, &xb, &xs) == 0) {
                char *xml = malloc((size_t)xs + 1);
                if (xml) {
                    memcpy(xml, xb, xs); xml[xs] = 0;
                    const char *sw = sw_find(xml, ng->gamename);
                    if (sw) {
                        rg_ent ents[96]; int ne; uint32_t asz;
                        int sno = getenv("SWNO") ? atoi(getenv("SWNO")) : 0;
                        int asz_spr = 0;
                        if (!(sno & 1) && !ng->rom && sw_area(sw, "maincpu", ents, 96, &ne, &asz) == 0)
                            ng->rom = sw_build(gamezip, ents, ne, 0x1000000, asz, &ng->rom_size); /* cap 16MB */
                        if (!(sno & 2) && !ng->spr && sw_area(sw, "sprites", ents, 96, &ne, &asz) == 0) {
                            asz_spr = (int)asz;
                            ng->spr = sw_build(gamezip, ents, ne, 0x8000000, asz, &ng->spr_size);
                        }
                        if (!(sno & 4) && !ng->fixed && sw_area(sw, "fixed", ents, 96, &ne, &asz) == 0)
                            ng->fixed = sw_build(gamezip, ents, ne, 0x100000, asz, &ng->fixed_size);
                        if (!(sno & 8) && !ng->snd && sw_area(sw, "audiocpu", ents, 96, &ne, &asz) == 0)
                            ng->snd = sw_build(gamezip, ents, ne, 0x80000, asz, &ng->snd_size);
                        if (!(sno & 16) && !ng->acrypt && sw_area(sw, "audiocrypt", ents, 96, &ne, &asz) == 0)
                            ng->acrypt = sw_build(gamezip, ents, ne, 0x80000, asz, &ng->acrypt_size);
                        if (ng->rom && !getenv("NOPROT")) {
                            int sz = ng->rom_size;
                            const char *swend = strstr(sw, "</software>");
                            if (!swend) swend = sw + strlen(sw);
                            if      (sw_has(sw, swend, "rom_kof98\""))   kof98_decrypt(ng);
                            else if (sw_has(sw, swend, "sma_kof99\"")   && sz >= 0x900000) { sma_dec_kof99(ng->rom);   ng->cart_prot = 2; }
                            else if (sw_has(sw, swend, "sma_garouh\"")  && sz >= 0x900000) { sma_dec_garouh(ng->rom);  ng->cart_prot = 4; }
                            else if (sw_has(sw, swend, "sma_garou\"")   && sz >= 0x900000) { sma_dec_garou(ng->rom);   ng->cart_prot = 3; }
                            else if (sw_has(sw, swend, "sma_mslug3a\"") && sz >= 0x900000) { sma_dec_mslug3a(ng->rom); ng->cart_prot = 6; }
                            else if (sw_has(sw, swend, "sma_mslug3\"")  && sz >= 0x900000) { sma_dec_mslug3(ng->rom);  ng->cart_prot = 5; }
                            else if (sw_has(sw, swend, "sma_kof2k\"")   && sz >= 0x900000) { sma_dec_kof2000(ng->rom); ng->cart_prot = 7; }
                            else if (sw_has(sw, swend, "pvc_mslug5\"")  && sz >= 0x800000) { pvc_run(ng->rom, ng->rom_size, pvc_dec_mslug5);  ng->cart_prot = 8; }
                            else if (sw_has(sw, swend, "pvc_svc\"")     && sz >= 0x800000) { pvc_run(ng->rom, ng->rom_size, pvc_dec_svc);     ng->cart_prot = 8; }
                            else if (sw_has(sw, swend, "pvc_kf2k3h\"")  && sz >= 0x900000) { pvc_run(ng->rom, ng->rom_size, pvc_dec_kof2003h);ng->cart_prot = 8; }
                            else if (sw_has(sw, swend, "pvc_kof2k\"")   && sz >= 0x900000) { pvc_run(ng->rom, ng->rom_size, pvc_dec_kof2003); ng->cart_prot = 8; }
                            else if (sw_has(sw, swend, "boot_cthd2k3\"")) { cthd2003_decrypt(ng); cthd_patch(ng); ng->cart_prot = 9; }
                            else if (sw_has(sw, swend, "k2k2_matrim\"")) { matrim_dec68k(ng);  ng->cart_prot = 10; ng->fixed_bank_type = 2; }
                            else if (sw_has(sw, swend, "k2k2_samsh5\"")) { samsho5_dec68k(ng); ng->cart_prot = 11; ng->fixed_bank_type = 1; }
                            if (sw_has(sw, swend, "cmc50_jockeygp\"") || sw_has(sw, swend, "rom_vliner\""))
                                ng->cart_prot = 12;         /* cart RAM window @200000 */
                            if (sw_has(sw, swend, "boot_kf2k3pl\"") || sw_has(sw, swend, "boot_kf2k3upl\""))
                                ng->cart_prot = 13;         /* kof2k3bl bootleg prot window @2FE000 */
                            if (ng->cart_prot >= 2 && ng->cart_prot <= 6) ng->fixed_bank_type = 1; /* MAME FIX_BANKTYPE_GAROU */
                            else if (ng->cart_prot == 7) ng->fixed_bank_type = 2;                /* FIX_BANKTYPE_KOF2000 */
                            if (!getenv("LINORAGX_BIOS") && ng->cart_prot == 5 &&
                                (sw_has(sw, swend, "sma_mslug3\"") || sw_has(sw, swend, "sma_mslug3a\""))) {
                                /* mslug3 crashes under uni-bios (boot-loops into BIOS RAM test); stock s3 works */
                                free(ng->bios); ng->bios = NULL;
                                load_bios_pref(ng, biosdir, "asia-s3.rom");
                            }
                            {   /* CMC42/50 graphics descramble per slot feature (MAME cart devices) */
                                static const struct { const char *f; uint8_t chip, key, sf, m1, s1; } ctab[] = {
                                    {"sma_kof99\"",42,0x00,1,0},
                                    {"sma_garouh\"",42,0x06,1,0},{"sma_garou\"",42,0x06,1,0},
                                    {"sma_mslug3a\"",42,0xad,1,0},{"sma_mslug3\"",42,0xad,1,0},{"sma_kof2k\"",50,0x00,1,1},
                                    {"pvc_mslug5\"",50,0x19,1,1,0},{"pvc_svc\"",50,0x57,1,1,0},
                                    {"pvc_kf2k3h\"",50,0x9d,1,1},{"pvc_kf2k3\"",50,0x9d,1,1},
                                    {"cmc42_zupapa\"",42,0xbd,0,0},{"cmc42_mslug3h\"",42,0xad,0,0},{"cmc42_ganryu\"",42,0x07,0,0},
                                    {"cmc42_s1945p\"",42,0x05,0,0},{"cmc42_preisle2\"",42,0x9f,0,0},{"cmc42_bangbead\"",42,0xf8,0,0},
                                    {"cmc42_nitd\"",42,0xff,0,0},{"cmc42_sengoku3\"",42,0xfe,0,0},{"cmc42_kof99k\"",42,0x00,0,0},
                                    {"cmc50_kof2001\"",50,0x1e,1,1},{"cmc50_kof2000n\"",50,0x00,1,1},{"cmc50_jockeygp\"",50,0xac,1,1},
                                    {"pcm2_mslug4\"",50,0x31,1,1},{"pcm2_ms4p\"",50,0x31,0,1},{"pcm2_rotd\"",50,0x3f,1,1},
                                    {"pcm2_pnyaa\"",50,0x2e,1,1},{"k2k2_matrim\"",50,0x6a,1,1},{"k2k2_samsh5\"",50,0x0f,1,1},
                                    {"pvc_kf2k3\"",50,0,1,0},{"pvc_kf2k3h\"",50,0,1,0},
                                    {"boot_kf2k3pl\"",50,0x9d,1,0},{"boot_kf2k3upl\"",50,0x9d,1,0},
                                };
                                for (unsigned ci = 0; ci < sizeof ctab / sizeof ctab[0]; ci++) {
                                    if (!sw_has(sw, swend, ctab[ci].f)) continue;
                                    if (ctab[ci].m1 && ng->acrypt && ng->acrypt_size >= 0x10000) {
                                        if (ng->snd) free(ng->snd);
                                        ng->snd = calloc(0x90000, 1); ng->snd_size = 0x90000;
                                        if (ng->snd) cmc50_m1_decrypt(ng->acrypt, ng->acrypt_size, ng->snd);
                                        if (f_cmctr()) fprintf(stderr, "M1DEC sw=%s acsz=%x snd[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x\n", ctab[ci].f, ng->acrypt_size, ng->snd[0],ng->snd[1],ng->snd[2],ng->snd[3],ng->snd[4],ng->snd[5],ng->snd[6],ng->snd[7]);
                                    }
                                    if (!getenv("LINORAGX_NOCMC"))
                                        cmc_gfx_run(ng, ctab[ci].chip == 50, ctab[ci].key, ctab[ci].sf, ctab[ci].s1);
                                    if (!strcmp(ctab[ci].f, "boot_kf2k3pl\"")) ng->k3bl_px = 1;
                                    else if (!strcmp(ctab[ci].f, "boot_kf2k3upl\"")) ng->k3bl_px = 2;
                                    fprintf(stderr, "CMC %s chip=%d key=%02x sfix=%d m1=%d spr=%x\n",
                                        ng->gamename, ctab[ci].chip, ctab[ci].key, ctab[ci].sf, ctab[ci].m1, ng->spr_size);
                                    break;
                                }
                            }
                            if (ng->k3bl_px && ng->rom && ng->rom_size >= 0x700000) {
                                /* kof2k3bl bootleg static PX decrypt (MAME prot_kof2k3bl.cpp) */
                                uint16_t *r16 = (uint16_t*)ng->rom;
                                if (ng->k3bl_px == 1) {
                                    static uint16_t tmp[0x100000];   /* WORDS: memcpy copies 0x100000 BYTES */
                                    for (int base = 0; base < 0x700000/2; base += 0x100000/2) {
                                        memcpy(tmp, &r16[base], 0x100000);
                                        for (int j = 0; j < 0x100000/2; j++) {
                                            /* MAME bitswap<24>(j,23,22,21,20,19,0,1,...,17):
                                               result bits 23..19 = src 23..19; bits 18..1 = src 0..17; bit0 = 0 */
                                            r16[base+j] = tmp[((j & 0xF80000) | ((j & 0x7FFFF) << 1))];
                                        }
                                    }
                                    r16[0xf38ac/2] = 0x4e75;   /* Altera patch: rte */
                                } else if (ng->rom_size >= 0x800000) {
                                    uint8_t *src = ng->rom;
                                    memmove(src + 0x100000, src, 0x600000);
                                    memmove(src, src + 0x700000, 0x100000);
                                    uint8_t *rom = src + 0xfe000;
                                    uint8_t *b2 = src + 0xd0610;
                                    for (int i = 0; i < 0x2000/2; i++) {
                                        int ofst = (i & 0xff00) + ((((i&0xff)&0x80)?0x80:0)|(((i&0xff)&0x40)?0x40:0)|(((i&0xff)&0x01)?0x20:0)|(((i&0xff)&0x10)?0x10:0)|(((i&0xff)&0x08)?0x08:0)|(((i&0xff)&0x04)?0x04:0)|(((i&0xff)&0x02)?0x02:0)|(((i&0xff)&0x20)?0x01:0));
                                        memcpy(&rom[i*2], &b2[ofst*2], 2);
                                    }
                                }
                                ng->k3bl_overlay = r16[0x58196/2];
                                fprintf(stderr, "K3BL px=%d overlay=%04x\n", ng->k3bl_px, ng->k3bl_overlay);
                            }
                            if (ng->cart_prot >= 2) {
                                ng->prg_bank_base = 0x100000;
                                ng->sma_rng = 0x2345;
                                fprintf(stderr, "PROT %s type=%d vec=%08x reg120=%04x/%04x hdr100=%04x\n", ng->gamename, ng->cart_prot,
                                    ((unsigned)rdw(ng->rom,0)<<16)|rdw(ng->rom,1), rdw(ng->rom,0x90), rdw(ng->rom,0x91), rdw(ng->rom,0x80));
                                if (f_dumpfix()) {
                                    FILE *df = fopen("/tmp/fix_dump.bin", "wb");
                                    if (df) { fwrite(ng->rom, 1, ng->rom_size, df); fclose(df); }
                                }
                            }
                        }
                    }
                    free(xml);
                }
                free(xb);
            }
        }
        if (f_swdbg()) fprintf(stderr, "SWDBG %s rom=%p/%x spr=%p/%x fix=%p/%x snd=%p/%x\n",
            ng->gamename, (void*)ng->rom, ng->rom_size, (void*)ng->spr, ng->spr_size,
            (void*)ng->fixed, ng->fixed_size, (void*)ng->snd, ng->snd_size);
        if (!ng->rom && (rom_load_zip_suffix(gamezip, ".p1", &d, &n) == 0 ||
                         rom_load_zip_suffix(gamezip, "-p1.bin", &d, &n) == 0)) {
            wordswap(d, n);
            ng->rom = calloc(1, 0x800000); ng->rom_size = n < 0x100000 ? 0x100000 : n;
            memcpy(ng->rom, d, n < 0x800000 ? n : 0x800000);
            uint32_t off = n > 0x800000 ? 0x800000 : n;   /* PRG = p1 then p2 then p3 */
            if (n == 0x200000) {
                /* MAME swlist "offset 0x100000 + continue@0": halves swapped —
                   region[0:1MB]=file[1MB:2MB] (vectors), region[1MB:]=file[0:1MB] */
                memmove(ng->rom + 0x100000, d, 0x100000);
                memcpy(ng->rom, d + 0x100000, 0x100000);
                off = 0x200000;
            }
            free(d);
            if (rom_load_zip_suffix(gamezip, ".p2", &d, &n) == 0 ||
                rom_load_zip_suffix(gamezip, ".sp2", &d, &n) == 0 ||
                rom_load_zip_suffix(gamezip, "-p2.bin", &d, &n) == 0) {
                wordswap(d, n);
                if (off + n <= 0x800000) memcpy(ng->rom + off, d, n);
                ng->rom_size = off + n > ng->rom_size ? off + n : ng->rom_size;
                free(d); off += n;
            }
            if (rom_load_zip_suffix(gamezip, ".p3", &d, &n) == 0 ||
                rom_load_zip_suffix(gamezip, "-p3.bin", &d, &n) == 0) {
                wordswap(d, n);
                if (off + n <= 0x800000) memcpy(ng->rom + off, d, n);
                ng->rom_size = off + n > ng->rom_size ? off + n : ng->rom_size;
                free(d);
            }
            /* MAME init_cpu: len > 1MB -> initial bank base = 0x100000, else 0 */
            ng->prg_bank_base = ng->rom_size > 0x100000 ? 0x100000 : 0;
        }
        if (!ng->fixed && (rom_load_zip_suffix(gamezip, ".s1", &d, &n) == 0 ||
                           rom_load_zip_suffix(gamezip, "-s1.bin", &d, &n) == 0)) { ng->fixed = d; ng->fixed_size = n; }
        if (!ng->snd && (rom_load_zip_suffix(gamezip, ".m1", &d, &n) == 0 ||
                         rom_load_zip_suffix(gamezip, "-m1.bin", &d, &n) == 0 ||
                         rom_load_zip_suffix(gamezip, "-m1.rom", &d, &n) == 0)) { ng->snd = d; ng->snd_size = n; }
        if (!ng->spr) {
            uint8_t *c[8] = {0}; uint32_t n[8] = {0};
            static const char cx[8][5] = {"c1","c2","c3","c4","c5","c6","c7","c8"};
            for (int k = 0; k < 8; k++) {
                char s1[8], s2[8], s3[10];
                snprintf(s1, sizeof s1, "-%s.%s", cx[k], cx[k]);
                snprintf(s2, sizeof s2, ".%s", cx[k]);
                snprintf(s3, sizeof s3, "-%s.bin", cx[k]);
                c[k] = 0;
                if (rom_load_zip_suffix(gamezip, s1, &c[k], &n[k]) != 0) {
                    c[k] = 0;
                    if (rom_load_zip_suffix(gamezip, s2, &c[k], &n[k]) != 0) {
                        c[k] = 0;
                        if (rom_load_zip_suffix(gamezip, s3, &c[k], &n[k]) != 0) c[k] = 0;
                    }
                }
            }
            if (c[0] && c[1]) {
                /* MAME layout: pairs interleaved, placed sequentially:
                   [c1c2][c3c4][c5c6][c7c8], pair size = 2*max(pair) */
                uint32_t off = 0, base[8];
                for (int p = 0; p < 4; p++) {
                    uint32_t lo = c[2*p] ? n[2*p] : 0;
                    if (c[2*p+1] && n[2*p+1] > lo) lo = n[2*p+1];
                    base[2*p] = off; base[2*p+1] = off + 1;
                    off += 2 * lo;
                }
                uint32_t sz = off;
                ng->spr = calloc(sz, 1); ng->spr_size = sz;
                for (int k = 0; k < 8; k++)
                    if (c[k])
                        for (uint32_t i = 0; i < n[k] && base[k] + 2*i < sz; i++)
                            ng->spr[base[k] + 2*i] = c[k][i];
            }
            for (int k = 0; k < 8; k++) free(c[k]);
        }
        if (!ng->rom) snprintf(err, errsz, "no .p1 program rom in %s", gamezip);
        ng->prg_bank_base = ng->rom_size > 0x100000 ? 0x100000 : 0;
    }
    if (ng->rom && ng->cart_prot == 0 &&
        (!strncmp(ng->gamename, "vliner", 6) || !strncmp(ng->gamename, "jinng", 5)))
        ng->cart_prot = 12;                          /* rom_vliner: plain ROM + 4K cart RAM */
    if (!ng->rom) {
        ng->rom = calloc(1, 0x100000); ng->rom_size = 0x100000;
        snprintf(ng->gamename, sizeof ng->gamename, "neo-geo bios");
    }
    if (getenv("ROMDUMP")) {
        FILE *df = fopen("/tmp/fix_dump.bin", "wb");
        if (df) { fwrite(ng->rom, 1, ng->rom_size, df); fclose(df); }
    }
    {   /* fixed region = cart s1 if cart has one, else BIOS sfix (MAME: m_fixed_layer_source ? cart : bios — never OR'd) */
        uint32_t fsz = 0x20000;
        while (fsz < ng->fixed_size) fsz <<= 1;
        uint8_t *fx = calloc(fsz, 1);
        int cart_has_fix = 0;
        for (uint32_t i = 0; i < ng->fixed_size && i < fsz; i++) if (ng->fixed[i]) { cart_has_fix = 1; break; }
        if (ng->sfix && !cart_has_fix) for (uint32_t i = 0; i < 0x20000; i++) fx[i] = ng->sfix[i];
        for (uint32_t i = 0; i < ng->fixed_size && i < fsz; i++) fx[i] |= ng->fixed[i];
        free(ng->fixed); ng->fixed = fx; ng->fixed_size = fsz;
        if (getenv("FIXDUMP")) {
            FILE *df = fopen("/tmp/fixed_plane.bin", "wb");
            if (df) { fwrite(ng->fixed, 1, fsz, df); fclose(df); }
            if (ng->spr && getenv("CRDUMP")) {
                FILE *cf = fopen("/tmp/cdec_head.bin", "wb");
                if (cf) { fwrite(ng->spr, 1, 0x40000, cf); fclose(cf); }
                uint64_t ssz = ng->spr_size & ~3ull;
                if (ssz > 0x40000) {
                    FILE *ct = fopen("/tmp/cdec_tail.bin", "wb");
                    if (ct) { fwrite(ng->spr + (ssz - 0x40000), 1, 0x40000, ct); fclose(ct); }
                }
            }
        }
    }
    decode_sprites(ng);

    ng->ym = ym_create();
    /* PCFLOW=lo-hi: instruction-level 68K PC flow dump (debug hook) */
    { const char *e = getenv("PCFLOW"); const char *d = e ? strchr(e, '-') : 0;
      if (e) { g_pcflow_lo = atoi(e); g_pcflow_hi = d ? atoi(d + 1) : 999999; g_pcflow_ng = ng; } }
    { const char *e = getenv("Z80FLOW"); const char *d = e ? strchr(e, '-') : 0;
      if (e) { g_z80flow_lo = atoi(e); g_z80flow_hi = d ? atoi(d + 1) : 999999; } }
    ym_reset(ng->ym);        /* ymfm requires an explicit reset; new() leaves chip state uninitialized */
    { const char *zt = getenv("Z80TRACE"); if (zt) ng->dbg_z80 = atoi(zt); }
    g_ng = ng;
    ng->z = z80w_create(ng, z_read, z_write, z_in, z_out);
    { extern void z80w_set_step_hook(void*, void (*)(unsigned));
      extern void linoragx_z80_step(unsigned);
      z80w_set_step_hook(ng->z, linoragx_z80_step); }
    m68k_set_cpu_type(M68K_CPU_TYPE_68000);
    m68k_init();
    m68k_set_instr_hook_callback(linoragx_pc_hook);  /* must follow m68k_init (it clears callbacks) */
    ng->screen_enabled = 1;
    ng->mc_lock1 = 1; ng->mc_unlock2 = 1; ng->mc_regsel = 1;

    ng->zbank[0]=0x1E; ng->zbank[1]=0x0E; ng->zbank[2]=0x06; ng->zbank[3]=0x02;
    m68k_pulse_reset();
    z80w_reset(ng->z);
        return ng;
}

void ng_destroy(neogeo *ng)
{
    if (!ng) return;
    free(ng->rom); free(ng->fixed); free(ng->snd); free(ng->spr); free(ng->sprite8);
    free(ng->bios); free(ng->sm1); free(ng->sfix);
    if (ng->z) z80w_destroy(ng->z);
    if (ng->ym) ym_destroy(ng->ym);
    free(ng);
}

/* ---- frame ---- */
extern uint16_t z80w_pc(void*);
void ng_run_frame(neogeo *ng)
{

    /* MAME watchdog: 3,244,030 master cycles (~16 frames) without a feed
       resets the whole system (CPU + audio CPU + latch state). Games use it
       as protection: pnyaa crashes its Z80 during the BIOS ch5 1->0 bank
       switch on purpose and rides the watchdog reset to boot cart m1 clean
       with ch5=1. Without it the 68K spins in the BIOS error handler. */
    /* OBSERVE-ONLY: uni-bios feeds $300001 during its own boot and during
       the cart-driver wait; a real reset here livelocks the BIOS forever
       (pnyaa: Z80 reboot every ~10 frames, boot never reaches EI). Games
       use the watchdog as a protection self-reset AFTER the BIOS hands
       over — enabling it post-boot needs a handoff detector, parked. */
    if (f_wdtr() && ng->wd_armed && ng->mc_cycles - ng->wd_last > 3244030ull) {
        ng->wd_armed = 0;
        fprintf(stderr, "WDTO f=%d\n", ng->frame);
    }
    if (!ng->auto_disabled) {
        if (ng->fc == 0) { ng->fc = ng->auto_speed ? ng->auto_speed : 1; ng->auto_counter++; }
        else ng->fc--;
    }
    /* vertical interrupt: level 1 (vector 0x64), fires every frame (LSPC auto) */
    m68k_set_virq(1, 1);
    /* MAME per-line rendering: arm the framebuffer for this frame; lines are
       drawn at their start inside the chunk loop (mid-frame VRAM/CRAM writes
       become visible, like sprite_line_timer + update_partial in MAME).
       Default OFF: MAME's Neo Geo renders the whole frame once at vblank;
       per-line interleaving exposed BIOS mid-frame writes as flickering
       black blocks in window mode (user report 2026-10-20). PERLINE=1 restores. */
    static int perline = -1;
    if (perline < 0) perline = getenv("PERLINE") ? 1 : 0;
    ng->perline = perline;
    if (ng->fb && perline) { ng_render_begin(ng, ng->fb); ng->draw_next = 0; }
    if (f_pcw() && ng->frame < 3) {
        static int z1 = 0;
        if (!z1) { z1 = 1;
            fprintf(stderr, "ROMCHK f=%d r00=%02x%02x%02x%02x%02x%02x%02x%02x r100=%02x%02x%02x%02x r182=%02x%02x sz=%d\n", ng->frame, ng->rom?ng->rom[0]:0xEE,ng->rom?ng->rom[1]:0xEE,ng->rom?ng->rom[2]:0xEE,ng->rom?ng->rom[3]:0xEE,ng->rom?ng->rom[4]:0xEE,ng->rom?ng->rom[5]:0xEE,ng->rom?ng->rom[6]:0xEE,ng->rom?ng->rom[7]:0xEE,
                ng->rom?ng->rom[0x100]:0xEE, ng->rom?ng->rom[0x101]:0xEE,
                ng->rom?ng->rom[0x182]:0xEE, ng->rom?ng->rom[0x183]:0xEE, ng->rom_size);
        }
    }
    int64_t left = CYCLES_PER_FRAME;
    static int chunk_max = -1;
    if (chunk_max < 0) { const char *e = getenv("NG_CHUNK"); chunk_max = e ? atoi(e) : 16; }
    while (left > 0) {
        int chunk = left > chunk_max ? chunk_max : (int)left;
        m68k_execute(chunk);
        { static int lo = -2, hi = -2;
          if (lo == -2) { const char *e = getenv("PCTRACE"); const char *d = e ? strchr(e, '-') : 0;
              lo = e ? atoi(e) : -1; hi = d ? atoi(d + 1) : -1; }
          if (lo >= 0 && ng->frame >= lo && ng->frame <= hi)
              fprintf(stderr, "T %d %08x\n", ng->frame, m68k_get_reg(NULL, M68K_REG_PC)); }
        { static int pcstop_init = -1; static uint32_t pcstop_tgt;
          if (pcstop_init < 0) { const char *pe = getenv("PCSTOP"); pcstop_tgt = pe ? (uint32_t)strtoul(pe, 0, 16) : 0; pcstop_init = pe ? 1 : 0; }
          if (pcstop_init) { static int ps = 0;
            uint32_t tgt = pcstop_tgt;
            static int pmin = -2;
            if (pmin == -2) { const char *pf = getenv("PCSTOPF"); pmin = pf ? atoi(pf) : 0; }
            uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
            if (!ps && pc == tgt && ng->frame >= pmin) {
                ps = 1;
                uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5);
                uint32_t a4 = m68k_get_reg(NULL, M68K_REG_A4);
                uint32_t a0 = m68k_get_reg(NULL, M68K_REG_A0);
                uint32_t sp = m68k_get_reg(NULL, M68K_REG_SP);
                uint32_t base = (a5 - 0x8000) & 0xFFFFFF;
                fprintf(stderr, "PCSTOP f=%d pc=%08x a5=%08x a4=%08x a0=%08x sp=%08x\n",
                    ng->frame, pc, a5, a4, a0, sp);
                fprintf(stderr, "PCSTOP [%06x]=%04x:%04x\n", base,
                    ng->workram[(base & 0xFFFF) >> 1], ng->workram[((base + 2) & 0xFFFF) >> 1]);
                for (int z = 0; z < 8; z++)
                    fprintf(stderr, "PCSTOP sp+%d=%04x\n", 2*z, ng->workram[((sp + 2*z) & 0xFFFF) >> 1]);
            } }
        if (f_warnw()) { static int wdone = 0;
            uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
            if (!wdone && pc >= 0xAE76 && pc < 0xAE9C) {
                wdone = 1;
                unsigned sp = m68k_get_reg(NULL, M68K_REG_SP);
                fprintf(stderr, "WARNW f=%d pc=%08x sp=%08x ret:", ng->frame, pc, sp);
                for (int z=0; z<10; z++) fprintf(stderr, " %08x", m68k_read_memory_32((sp + 2*z*4) & 0xFFFFFF));
                fprintf(stderr, " fd80=%08x d100=%04x fdAE=%02x\n", m68k_read_memory_32(0x10FD80),
                    m68k_read_memory_16(0xD00100), m68k_read_memory_16(0x10FDAE));
            } }
        if (f_fd82w()) { static int fdone = 0;
            uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
            int site = pc == 0xC160E8 ? 0 : pc == 0xC17BC6 ? 1 : pc == 0xC1A3F8 ? 2 : -1;
            if (site >= 0 && !(fdone & (1 << site))) {
                fdone |= 1 << site;
                unsigned sp = m68k_get_reg(NULL, M68K_REG_SP);
                fprintf(stderr, "FD82W f=%d pc=%08x ret: %08x %08x %08x\n", ng->frame, pc,
                    m68k_read_memory_32(sp), m68k_read_memory_32(sp+4), m68k_read_memory_32(sp+8));
            } }
        if (f_pcw() && m68k_get_reg(NULL, M68K_REG_PC) == 0xC11FFC && ng->frame > 4100) {
            fprintf(stderr, "CKSUM f=%d d7=%08x d0=%08x a0=%08x a1=%08x p182=%08x\n", ng->frame,
                m68k_get_reg(NULL,M68K_REG_D7), m68k_get_reg(NULL,M68K_REG_D0),
                m68k_get_reg(NULL,M68K_REG_A0), m68k_get_reg(NULL,M68K_REG_A1),
                m68k_read_memory_32(0x100182));
          }
        }
        if (f_pcw() && m68k_get_reg(NULL, M68K_REG_PC) == 0xC15222) {
            unsigned sp = m68k_get_reg(NULL, M68K_REG_SP);
            fprintf(stderr, "FATAL f=%d sp=%08x ret:", ng->frame, sp);
            for (int z=0; z<6; z++) fprintf(stderr, " %08x", m68k_read_memory_32(sp + z*4));
            fprintf(stderr, " a0=%08x a1=%08x a2=%08x d0=%08x d1=%08x d2=%08x d5=%08x d6=%08x\n",
                m68k_get_reg(NULL,M68K_REG_A0), m68k_get_reg(NULL,M68K_REG_A1), m68k_get_reg(NULL,M68K_REG_A2),
                m68k_get_reg(NULL,M68K_REG_D0), m68k_get_reg(NULL,M68K_REG_D1), m68k_get_reg(NULL,M68K_REG_D2),
                m68k_get_reg(NULL,M68K_REG_D5), m68k_get_reg(NULL,M68K_REG_D6));
        }
        if (f_pcw() && m68k_get_reg(NULL, M68K_REG_PC) == 0xC10F70) {
            static int pcw_done = 0;
            if (!pcw_done) { pcw_done = 1;
                fprintf(stderr, "CHK f=%d p182=%08x rom=%04x tbl=%04x d7=%08x\n", ng->frame,
                    m68k_read_memory_32(0x100182),
                    m68k_read_memory_16(0x01820184), m68k_read_memory_16(0xC13674),
                    m68k_get_reg(NULL, M68K_REG_D7));
            }
        }
        if (ng->pcprof) {
            static unsigned hist[4096]; static unsigned long tot;
            unsigned pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xFFFFF;
            hist[(pc>>6)&4095]++; tot++;
            ng->pcprof_tot = tot;
            static int dump_at = -1;
            if (dump_at >= 0 && (long)ng->frame == dump_at) {
                for (int k=0;k<4096;k++) if (hist[k] > (int)(tot/200))
                    fprintf(stderr, "PCP %05x %lu\n", k<<6, (unsigned long)hist[k]);
                dump_at = -2;
            }
            if (dump_at == -1) dump_at = (int)ng->frame + 100;
        }
        ng->mc_cycles += chunk;
        left -= chunk;
        ng->scanline = (int)((CYCLES_PER_FRAME - left) / CYCLES_PER_LINE) % LINES_PER_FRAME;
        /* per-line draw: fb row r = scanline r+40 (frame start = MAME line 240,
           visible lines start at s=40); a line is complete once the beam passed it */
        if (ng->fb && ng->perline) {
            int done = ng->scanline - 40;
            while (ng->draw_next < done) { ng_draw_line(ng, ng->fb, ng->draw_next); ng->draw_next++; }
        }
        { extern uint16_t z80w_sp(void*);
          uint16_t zpc0 = z80w_pc(ng->z);
          z80w_exec(ng->z, chunk/3);   /* MAME: Z80 = master/6 = 4MHz, 68K = master/2 = 12MHz -> 1:3 */
          uint16_t zpc1 = z80w_pc(ng->z);
          if (f_znmi() && ng->frame >= 61 && ng->frame <= 63 && zpc0 != zpc1)
              fprintf(stderr, "ZNMI f=%d %04x->%04x\n", ng->frame, zpc0, zpc1);
          if (f_z18() && zpc0 >= 0x1800 && zpc0 <= 0x1810)
              fprintf(stderr, "Z18 f=%d %04x->%04x sp=%04x\n", ng->frame, zpc0, zpc1, z80w_sp(ng->z));
          if (f_z390() && zpc0 >= 0x0380 && zpc0 <= 0x03a0)
              fprintf(stderr, "Z390 f=%d %04x->%04x sp=%04x\n", ng->frame, zpc0, zpc1, z80w_sp(ng->z));
          if (f_zret() && zpc0 == 0x1807)
              fprintf(stderr, "ZRET f=%d sp=%04x ram=%02x%02x\n", ng->frame, z80w_sp(ng->z), ng->sndram[z80w_sp(ng->z)&0x7ff], ng->sndram[(z80w_sp(ng->z)+1)&0x7ff]);
          if (f_zseq() && zpc0 >= 0x2b18 && zpc0 <= 0x2b22)
              fprintf(stderr, "ZSEQ f=%d %04x->%04x sp=%04x\n", ng->frame, zpc0, zpc1, z80w_sp(ng->z));
          if (f_zsp() && (zpc0 == 0x2b1c || zpc0 == 0x2b1f || zpc0 == 0x1807 || zpc0 == 0x180e)) {
              extern unsigned long z80w_bc(void*);
              fprintf(stderr, "ZSP f=%d pc=%04x sp=%04x bc=%lx\n", ng->frame, zpc0, z80w_sp(ng->z), z80w_bc(ng->z));
          }
          if (f_zbc() && zpc0 == 0x00c5) {
              extern unsigned long z80w_bc(void*);
              fprintf(stderr, "ZBC f=%d bc=%lx sp=%04x\n", ng->frame, z80w_bc(ng->z), z80w_sp(ng->z));
          }
          if (ng->znmi_t > 0) { fprintf(stderr, "ZNT f=%d %04x\n", ng->frame, zpc0); ng->znmi_t--; }
          if (zpc0 >= 0x00baf && zpc0 <= 0x00bc0 && f_zntd() && ng->frame >= 60 && ng->znmi_t == 0) {
              extern unsigned long z80w_bc(void*);
              static int zntd_once = 0;
              if (!zntd_once) { zntd_once = 1;
                  fprintf(stderr, "ZNTD ae7=%02x%02x aeb=%02x%02x b3=%02x sp=%04x hl=%04x\n",
                      ng->sndram[0xae7], ng->sndram[0xae8], ng->sndram[0xaeb], ng->sndram[0xaec],
                      ng->sndram[0xb3], z80w_sp(ng->z), (unsigned)(z80w_bc(ng->z) & 0xFFFF));
                  ng->znmi_t = -1;
              } }
          if (f_ztrail() && ng->frame >= ztf0() && ng->frame <= ztf1()) {
              static uint16_t tprev;
              if (zpc0 != tprev) { fprintf(stderr, "ZTR f=%d %04x\n", ng->frame, zpc0); tprev = zpc0; }
          }
          if (f_zpchist() && ng->frame >= 100 && ng->frame < 280) {
              static unsigned long hist[0x10000];
              hist[zpc0]++;
              static int dumped = 0;
              if (ng->frame == 279 && !dumped) {
                  dumped = 1;
                  fprintf(stderr, "ZPCHIST top:\n");
                  for (int rep = 0; rep < 12; rep++) {
                      unsigned long best = 0; int bi = -1;
                      for (int i = 0; i < 0x10000; i++) if (hist[i] > best) { best = hist[i]; bi = i; }
                      if (bi < 0) break;
                      fprintf(stderr, "  %04x: %lu\n", bi, best);
                      hist[bi] = 0;
                  }
              }
          }
          if (ng->dbg_z80 == 2) {
              extern uint16_t z80w_sp(void*);
              static uint16_t lastpc; static uint16_t lastsp;
              uint16_t sp1 = z80w_sp(ng->z);
              if (zpc1 != lastpc || sp1 != lastsp) {
                  fprintf(stderr, "ZT %04x/%04x -> %04x/%04x\n", lastpc, lastsp, zpc1, sp1);
                  lastpc = zpc1; lastsp = sp1;
                  if (--ng->dbg_z80 <= 0) ng->dbg_z80 = 0;
              }
          }
          if (ng->dbg_z80 == 1 && zpc1 >= 0xF000 && zpc0 < 0xF000) {
              fprintf(stderr, "ZCRASH %04x->%04x sp=%04x sl1=%02x pend=%d en=%d nmi=%lu\n",
                  zpc0, zpc1, z80w_sp(ng->z), ng->soundlatch1, ng->z80_nmi_pend,
                  ng->z80_nmi_en, ng->nmi_cnt);
              if (++ng->dbg_z80 > 40) ng->dbg_z80 = 0;
          } }
        ym_advance(ng->ym, chunk*2/3);  /* YM2610 = master/3 = 8MHz = 2x Z80 rate */
        /* YM2610 timer IRQ -> Z80 INT must be evaluated at chunk granularity:
           m1's heartbeat polls in(04) several times per frame and the status
           read ACKs (clears) the timer flags — a frame-end-only check lets
           the heartbeat swallow every IRQ before it is ever delivered
           (samsho2 m1 track sequencer is timer-driven and deadlocks without it). */
        { int p = ym_irq_pending(ng->ym);
          if (p) z80w_irq(ng->z, 0xff);   /* level semantics: re-assert every chunk */
          ng->ym_irq_prev = p; }
        { static int zram_mask = 0;
          extern uint8_t z80w_peek(void*, uint32_t);
          extern uint16_t z80w_sp(void*);
          /* ZLO: catch PC entering the zero-page crash zone with stack snapshot */
          if (f_ztrace()) {
              static int zlo_armed = 1;
              uint16_t zpc_now = z80w_pc(ng->z);
              if (zlo_armed && ng->frame > 50 && zpc_now < 0x0100 && zpc_now != 0x0038 && zpc_now != 0x0066) {
                  zlo_armed = 0;
                  uint16_t spn = z80w_sp(ng->z);
                  fprintf(stderr, "ZLO f=%d pc=%04x sp=%04x stk=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                      ng->frame, zpc_now, spn,
                      z80w_peek(ng->z,spn), z80w_peek(ng->z,spn+1), z80w_peek(ng->z,spn+2), z80w_peek(ng->z,spn+3),
                      z80w_peek(ng->z,spn+4), z80w_peek(ng->z,spn+5), z80w_peek(ng->z,spn+6), z80w_peek(ng->z,spn+7));
              }
          }
          if (f_ztrace() && ng->frame >= 59 && ng->frame <= 70) {
              int bit = 1 << (ng->frame - 59);
              if (!(zram_mask & bit)) { zram_mask |= bit;
                  fprintf(stderr, "ZRAM f=%d ffd=%02x ffe=%02x fff=%02x fe38=%02x fe18=%02x fc33=%02x pc=%04x sp=%04x ca=%d zr=%02x sm=%02x snd=%p\n",
                      ng->frame, z80w_peek(ng->z,0xfffd), z80w_peek(ng->z,0xfffe),
                      z80w_peek(ng->z,0xffff), z80w_peek(ng->z,0xfe38),
                      z80w_peek(ng->z,0xfe18), z80w_peek(ng->z,0xfc33),
                      z80w_pc(ng->z), z80w_sp(ng->z),
                      ng->cart_audio, z_read(0x2106, ng), ng->sm1 ? ng->sm1[0x2106] : 0xEE, (void*)ng->snd); } } }
        /* (YM timer IRQ now delivered at chunk granularity above) */
        { int a = ng->z80_nmi_pend && ng->z80_nmi_en;
          if (a && !ng->z80_nmi_asserted) { z80w_nmi(ng->z); ng->nmi_cnt++; if (f_znmit()) ng->znmi_t = 40; }
          ng->z80_nmi_asserted = a; }
        /* Timer C: level 1 (vector 0x64), once per frame mid-screen */
        if (!f_noraster() && (ng->irq_enable & 1) && ng->raster_armed && !ng->raster_pending) {
            int64_t cyc = (int64_t)((CYCLES_PER_FRAME - left)) ;   /* cycles since frame start */
            if (cyc * 2 > (int64_t)ng->display_counter) {          /* pixel ticks */
                ng->raster_armed = 0; ng->raster_pending = 1;
                m68k_set_virq(2, 1);
            }
        }
    }
    /* MAME LS74 semantics: the vblank line stays asserted until the 68K acks
       via 0x3C000C bit2 (done in the BIOS/game ISR itself). Clearing it at
       frame end would swallow the interrupt whenever the CPU sits in a
       IRQ-disabled loop across the whole frame (Musashi only samples IRQs at
       m68k_execute entry and set_sr, so a dropped line is lost until the next
       frame — the BIOS coin screen then never reaches its ISR). */
    if (getenv("VB_CLEAR")) m68k_set_virq(1, 0);    /* A/B: old frame-end clear */
    if (ng->irq_enable & 5) ng->raster_armed = 1;   /* AUTOLOAD_VBLANK or REPEAT */
    if (ng->fb) {   /* frame-end whole-frame draw (MAME vblank semantics) unless per-line */
        if (!ng->perline) { ng_render_begin(ng, ng->fb); ng->draw_next = 0; }
        while (ng->draw_next < NG_H) { ng_draw_line(ng, ng->fb, ng->draw_next); ng->draw_next++; }
    }
    ng->scanline = 0;
    if (ng->scanline == 0 && getenv("ZRAMDUMP")) {
        static int zrd = -2;
        if (zrd == -2) { const char *e = getenv("ZRAMDUMP"); zrd = e ? atoi(e) : 0; }
        if (zrd && ng->frame == zrd) {
            extern uint8_t z80w_peek(void*, uint32_t);
            fprintf(stderr, "ZRD f=%d zpc=%04x\n", ng->frame, z80w_pc(ng->z));
            fprintf(stderr, "ZRD fe10:");
            for (int q = 0; q < 0x40; q++) fprintf(stderr, " %02x", z80w_peek(ng->z, 0xfe10 + q));
            fprintf(stderr, "\nZRD feb0:");
            for (int q = 0; q < 0x20; q++) fprintf(stderr, " %02x", z80w_peek(ng->z, 0xfeb0 + q));
            fprintf(stderr, "\nZRD fec0:");
            for (int q = 0; q < 0x40; q++) fprintf(stderr, " %02x", z80w_peek(ng->z, 0xfec0 + q));
            fprintf(stderr, "\nZRD fd90:");
            for (int q = 0; q < 0x40; q++) fprintf(stderr, " %02x", z80w_peek(ng->z, 0xfd90 + q));
            fprintf(stderr, "\nZRD fc30:");
            for (int q = 0; q < 0x10; q++) fprintf(stderr, " %02x", z80w_peek(ng->z, 0xfc30 + q));
            fprintf(stderr, "\n");
        }
    }
    /* BIOS-phase ch5 handoff: real HC259 q5 drives the Z80 RESET line — a
       sustained ch5=1 left by the BIOS at the end of its audio init is the
       handoff to cart m1 (the self-test pulse returns to 0 within the same
       frame). Without this the Z80 stays on sm1, which does not understand
       the game's own sound protocol (samsho2 family cmd 2d handshake), and
       game BGM never starts. */
    /* BIOS-phase handoff: the BIOS left ch5=1 (cart audio requested) but the
       Z80 must stay on SM1 until the BIOS is fully done. The real switch
       point is the 68K PC leaving the BIOS alias windows — the game has
       started and its own sound protocol begins; reset into a clean cart m1
       so its RAM/track state never saw a BIOS-phase command (samsho2 cmd
       residue deadlocked m1's main loop otherwise). */
    if (ng->ch5_bios_pend && ng->snd && !ng->m1_booted) {
        unsigned long pc = (unsigned long)m68k_get_reg(NULL, M68K_REG_PC);
        int in_bios = (pc >= 0xC00000UL && pc < 0xD00000UL) || pc >= 0xFF8000UL
                   || (pc >= 0x100000UL && pc < 0x200000UL);   /* BIOS workram */
        if (!in_bios) {
            if (ng->frame - ng->ch5_on_frame >= 2) {   /* debounce bank-switch transients */
                ng->ch5_bios_pend = 0;
                ng->cart_audio = 1;
                z80w_reset(ng->z);
                ng->z80_nmi_asserted = 0;   /* keep z80_nmi_pend: the HC259 line survives (mslug4) */
                ng->m1_booted = 1; ng->m1_view = 1;
                if (f_latch_tr()) fprintf(stderr, "PCHANDOFF f=%llu pc=%08lx\n",
                                          (unsigned long long)ng->frame, pc);
            }
        } else ng->ch5_on_frame = ng->frame;   /* still in BIOS: keep waiting */
    }
    { extern unsigned long long z80w_dbg_frame; z80w_dbg_frame = ng->frame; }
    ng->frame++;
    { static int k68 = -2;
      if (k68 == -2) k68 = getenv("K68PC") ? 1 : 0;
      if (k68) fprintf(stderr, "68KPC f=%llu pc=%08x\n",
                       (unsigned long long)ng->frame, m68k_get_reg(NULL, M68K_REG_PC)); }
}

/* arm per-line rendering into fb (window mode); NULL disables (headless) */
void ng_set_fb(neogeo *ng, uint32_t *fb) { ng->fb = fb; }

/* netplay accessors: raw player button masks + game identity for handshake */
uint32_t ng_get_buttons(neogeo *ng, int p) { return (p == 0 || p == 1) ? ng->buttons[p] : 0; }
void ng_set_buttons(neogeo *ng, int p, uint32_t mask) { if (p == 0 || p == 1) ng->buttons[p] = mask; }
uint32_t ng_game_crc(neogeo *ng) { return rom_crc32((const uint8_t *)ng->gamename, (uint32_t)strlen(ng->gamename)); }

/* ---- accessors for video.c ---- */
uint16_t *ng_vram(neogeo *ng) { return ng->vram; }
uint16_t *ng_workram(neogeo *ng) { return ng->workram; }
uint16_t *ng_cram(neogeo *ng) { return ng->cram; }
uint8_t  *ng_fixed(neogeo *ng, uint32_t *sz) { *sz = ng->fixed_size; return ng->fixed; }
int       ng_fixed_bank_type(neogeo *ng) { return ng->fixed_bank_type; }
uint8_t  *ng_sprite8(neogeo *ng, uint32_t *mask) { *mask = ng->sprite8_mask; return ng->sprite8; }
uint8_t  *ng_spr_raw(neogeo *ng) { return ng->spr; }
uint8_t  *ng_spr(neogeo *ng, uint32_t *sz) { *sz = ng->spr_size; return ng->spr; }
uint8_t  *ng_zoomy(neogeo *ng) { return ng->zoomy; }
int       ng_auto_counter(neogeo *ng) { return ng->auto_counter & 7; }
void      ng_set_button(neogeo *ng, int p, int bit, int down)
{
    if (p < 0 || p > 1 || bit < 0 || bit > 10) return;
    if (down) ng->buttons[p] |= 1u << bit; else ng->buttons[p] &= ~(1u << bit);
}
uint32_t ng_cram_bank(neogeo *ng) { return ng->cram_bank; }
uint32_t ng_screen_shadow(neogeo *ng) { return ng->screen_shadow; }
uint32_t ng_cram_bank_render(neogeo *ng) { return ng->cram_bank_render; }
void ng_dbg_vw(neogeo *ng, int on) { ng->dbg_vw = on; }
void ng_dbg_z80(neogeo *ng, int on) { ng->dbg_z80 = on; }
void ng_dbg_hits(neogeo *ng, unsigned long *a, unsigned long *b, unsigned long *c)
{ *a = ng->hit_a; *b = ng->hit_b; *c = ng->hit_c; }
uint8_t *ng_dbg_m1(neogeo *ng, uint32_t *sz) {
    fprintf(stderr, "DBG_M1 sz=%x ptr=%p [0..7]=%02x %02x %02x %02x\n", ng->snd_size, (void*)ng->snd, ng->snd?ng->snd[0]:0xEE,ng->snd?ng->snd[1]:0xEE,ng->snd?ng->snd[2]:0xEE,ng->snd?ng->snd[3]:0xEE);
    *sz = ng->snd_size; return ng->snd; }
void ng_dbg_state(neogeo *ng, unsigned *pc68, unsigned *pcz, unsigned *sl2)
{
    *pc68 = (unsigned)m68k_get_reg(NULL, M68K_REG_PC);
    *pcz  = ng->z ? (unsigned)z80w_pc(ng->z) : 0;
    *sl2  = ng->soundlatch2;
}
void ng_dbg_nmi(neogeo *ng, int *pend, int *en, int *asserted, int *cart, unsigned *latch1)
{
    *pend = ng->z80_nmi_pend; *en = ng->z80_nmi_en; *asserted = ng->z80_nmi_asserted;
    *cart = ng->cart_audio; *latch1 = ng->soundlatch1;
}
int ng_dbg_vbank(neogeo *ng) { return (int)ng->vector_bank; }
void ng_dbg_btn(neogeo *ng, int p, int mask) { ng->buttons[p] |= mask; }
void ng_dbg_pcprof(neogeo *ng, int on) { ng->pcprof = on; }
uint8_t *ng_rom(neogeo *ng, uint32_t *sz) { *sz = ng->rom_size; return ng->rom; }
extern uint16_t z80w_pc(void*);
int ng_z80_pc(neogeo *ng) { return z80w_pc(ng->z); }
int ng_dbg_mc(void *v, int idx) {
    neogeo *n = (neogeo*)v;
    switch (idx) {
    case 0: return n->mc_lock1; case 1: return n->mc_unlock2; case 2: return n->mc_regsel;
    default: return n->mc_data[idx-3];
    }
}
unsigned long ng_dbg_vloads(void *v) { return ((neogeo*)v)->vload_count; }
unsigned long ng_dbg_hsk(void *v, int i) {
    neogeo *n=(neogeo*)v; return i==0? n->nmi_cnt : i==1? n->p0_cnt : i==2? n->sl1_cnt : n->soundlatch2; }
int ng_dbg_irqen(void *v) { return ((neogeo*)v)->irq_enable; }
int ng_dbg_dcnt(void *v)  { return ((neogeo*)v)->display_counter; }
void ng_audio(neogeo *ng, int16_t *stereo, int frames) {
    if (ng->ym && ng->snd && !ng->ym_rom_set) { ym_set_rom(ng->ym, ng->snd, ng->snd_size); ng->ym_rom_set = 1; }
    ym_stream(ng->ym, stereo, frames);
}
uint64_t ng_frame_no(neogeo *ng) { return ng->frame; }
const char *ng_game_name(neogeo *ng) { return ng->gamename; }

/* ---- save state (NRXSTATE07 magic) ---- */
#pragma pack(push,1)
struct hdr {
    char magic[11];             /* NRXSTATE09\0 */
    uint64_t frame; uint8_t fc;
    uint32_t sr, ssp, usp, vbr, pc;
    uint32_t d[8], a[8];
    uint8_t zstate[64];
    uint8_t zbank[4]; uint8_t vecbank;
    uint16_t voff, vmod, vrb, dcnt, ictl;
    uint32_t crambank;          /* NRXSTATE09: systemlatch ch7 palette bank */
};
#pragma pack(pop)

int ng_save_state(neogeo *ng, const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    struct hdr h; memset(&h, 0, sizeof h);
    memcpy(h.magic, "NRXSTATE09", 11);
    h.crambank = ng->cram_bank | ((ng->screen_shadow & 1) << 1);  /* bit1 = shadow (NRXSTATE09-compatible) */
    h.frame = ng->frame; h.fc = ng->fc;
    h.sr = m68k_get_reg(NULL, M68K_REG_SR);
    h.ssp = m68k_get_reg(NULL, M68K_REG_A7);
    h.usp = m68k_get_reg(NULL, M68K_REG_USP);
    h.vbr = m68k_get_reg(NULL, M68K_REG_VBR);
    h.pc = m68k_get_reg(NULL, M68K_REG_PC);
    for (int i=0;i<8;i++) h.d[i] = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_D0+i));
    for (int i=0;i<8;i++) h.a[i] = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0+i));
    z80w_save(ng->z, h.zstate);
    memcpy(h.zbank, ng->zbank, 4);
    h.vecbank = ng->vector_bank;
    h.voff = ng->vram_offset; h.vmod = ng->vram_modulo; h.vrb = ng->vram_readbuf;
    h.dcnt = ng->display_counter; h.ictl = ng->irq_enable;
    fwrite(&h, sizeof h, 1, f);
    fwrite(ng->vram, sizeof ng->vram, 1, f);
    fwrite(ng->cram, sizeof ng->cram, 1, f);
    fwrite(ng->workram, sizeof ng->workram, 1, f);
    fwrite(ng->sndram, sizeof ng->sndram, 1, f);
    fwrite(ng->sram, sizeof ng->sram, 1, f);
    fclose(f);
    return 0;
}

int ng_load_state(neogeo *ng, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    struct hdr h;
    if (fread(&h, sizeof h, 1, f) != 1 || memcmp(h.magic, "NRXSTATE09", 11)) { fclose(f); return -2; }
    ng->frame = h.frame; ng->fc = h.fc;
    ng->vector_bank = h.vecbank;
    m68k_set_reg(M68K_REG_SR, h.sr);
    m68k_set_reg(M68K_REG_A7, h.ssp);
    m68k_set_reg(M68K_REG_USP, h.usp);
    m68k_set_reg(M68K_REG_VBR, h.vbr);
    for (int i=0;i<8;i++) m68k_set_reg((m68k_register_t)(M68K_REG_D0+i), h.d[i]);
    for (int i=0;i<8;i++) m68k_set_reg((m68k_register_t)(M68K_REG_A0+i), h.a[i]);
    m68k_set_reg(M68K_REG_PC, h.pc);
    z80w_load(ng->z, h.zstate);
    memcpy(ng->zbank, h.zbank, 4);
    ng->vram_offset = h.voff; ng->vram_modulo = h.vmod; ng->vram_readbuf = h.vrb;
    ng->display_counter = h.dcnt; ng->irq_enable = h.ictl;
    ng->cram_bank = h.crambank & 0x1000;
    ng->screen_shadow = (h.crambank >> 1) & 1;
    if (fread(ng->vram, sizeof ng->vram, 1, f) != 1 ||
        fread(ng->cram, sizeof ng->cram, 1, f) != 1 ||
        fread(ng->workram, sizeof ng->workram, 1, f) != 1 ||
        fread(ng->sndram, sizeof ng->sndram, 1, f) != 1 ||
        fread(ng->sram, sizeof ng->sram, 1, f) != 1) { fclose(f); return -3; }
    fclose(f);
    return 0;
}
