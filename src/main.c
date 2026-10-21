#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
/* linoragx — NeoRageX-style Neo Geo emulator rewrite for Linux/Wayland/Vulkan.
 * CLI: linoragx [--bios DIR] [--game FILE.zip] [--ini FILE] [--scale N]
 *            [--headless] [--frames N] [--dump out.bmp] [--state FILE]
 *            [--save-state FILE] [--no-audio] [--scanlines] */
#include "linoragx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* from wl_input.c */
typedef struct wl_input wl_input;
wl_input *wl_input_create(const char *disp);
void wl_input_destroy(wl_input *in);
int  wl_input_next(wl_input *in, uint32_t *sym, int *down);

/* keysym -> (player, bit); bits: 0UP 1DN 2LF 3RT 4C 5D 6A 7B 8start 10coin */
struct kmap { uint32_t sym; int p, bit; };
static const struct kmap kmap[] = {
    {0xff52,0,0},{0xff54,0,1},{0xff51,0,2},{0xff53,0,3},   /* arrows */
    {'z',0,6},{'x',0,7},{'c',0,4},{'v',0,5},{'b',0,9},      /* A B C D select */
    {' ',0,8},{'\n',0,8},                                   /* start */
    {'5',0,10},{'6',1,10},                                  /* coins */
    {0xff9c,1,0},{0xff99,1,1},{0xff96,1,2},{0xff98,1,3},   /* kp8/2/4/6 */
    {'q',1,6},{'w',1,7},{'e',1,4},{'r',1,5},
};

static void handle_key(neogeo *ng, uint32_t sym, int down)
{
    for (unsigned i = 0; i < sizeof kmap/sizeof kmap[0]; i++)
        if (kmap[i].sym == sym) { ng_set_button(ng, kmap[i].p, kmap[i].bit, down); return; }
}

static double now_s(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec/1e9;
}

/* ---- built-in CPU profiler (PROF=1): SIGPROF sampling + dladdr symbols ---- */
#include <signal.h>
#include <sys/time.h>
#include <ucontext.h>
#include <dlfcn.h>
#define PROF_SLOTS 8192
static struct { const void *pc; unsigned long n; } g_prof[PROF_SLOTS];
static void prof_handler(int sig, siginfo_t *si, void *uc) {
    (void)sig; (void)si;
    ucontext_t *u = (ucontext_t*)uc;
    const void *pc = (const void*)u->uc_mcontext.gregs[REG_RIP];
    const void *ra = (const void*)u->uc_mcontext.gregs[REG_RBP]; (void)ra;
    /* if inside libc getenv, sample the return address instead */
    Dl_info di;
    Dl_info dm; int in_main = dladdr((void*)&prof_handler, &dm) && dladdr((void*)pc, &di) && di.dli_fbase == dm.dli_fbase;
    if (!in_main && dladdr((void*)pc, &di) && di.dli_sname && !strcmp(di.dli_sname, "getenv")
        && di.dli_fname && strstr(di.dli_fname, "libc")) {
        const void *ret = (const void*)*(unsigned long*)u->uc_mcontext.gregs[REG_RSP];
        if (ret) pc = ret;
    }
    for (int i = 0; i < PROF_SLOTS; i++) {
        if (g_prof[i].pc == pc) { g_prof[i].n++; return; }
        if (!g_prof[i].pc) { g_prof[i].pc = pc; g_prof[i].n = 1;
            if (getenv("PROFDBG")) { Dl_info dd; if (dladdr((void*)pc,&dd)&&dd.dli_fname)
                fprintf(stderr,"[P] %p in %s +0x%lx sym=%s\n", pc, dd.dli_fname,
                    (unsigned long)((char*)pc-(char*)dd.dli_fbase), dd.dli_sname?dd.dli_sname:"?"); }
            return; }
    }
}
static int envf_dbg(void){ static int v=-1; if(v<0) v=getenv("LINORAGX_DEBUG")?1:0; return v; }
static int envf_tp(void){ static int v=-1; if(v<0) v=getenv("LINORAGX_TESTPATTERN")?1:0; return v; }
static int envf_prof(void){ static int v=-1; if(v<0) v=getenv("PROF")?1:0; return v; }
static void prof_start(void) {
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = prof_handler; sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigaction(SIGPROF, &sa, NULL);
    struct itimerval t = { {0,10000},{0,10000} };
    setitimer(ITIMER_PROF, &t, NULL);
}
static void prof_dump(void) {
    struct itimerval z = { {0,0},{0,0} }; setitimer(ITIMER_PROF, &z, NULL);
    /* top-N selection sort over slots */
    unsigned long total = 0; static int idx[PROF_SLOTS]; int nn = 0;
    for (int i = 0; i < PROF_SLOTS; i++) if (g_prof[i].pc) { idx[nn++]=i; total+=g_prof[i].n; }
    for (int a = 0; a < nn && a < 18; a++) { int b=a; for (int c=a+1;c<nn;c++) if (g_prof[idx[c]].n>g_prof[idx[b]].n) b=c;
        int t0=idx[a]; idx[a]=idx[b]; idx[b]=t0; }
    { Dl_info d0; if (dladdr((void*)&prof_dump, &d0) && d0.dli_fbase)
        fprintf(stderr, "== PROF total=%lu base=%p ==\n", total, d0.dli_fbase);
      else fprintf(stderr, "== PROF total=%lu ==\n", total); }
    for (int a = 0; a < nn && a < 18; a++) {
        Dl_info di; const char *sym = "?";
        if (dladdr((void*)g_prof[idx[a]].pc, &di) && di.dli_sname) sym = di.dli_sname;
        fprintf(stderr, "  %6.2f%% %8lu  %s  @%p\n", 100.0*g_prof[idx[a]].n/(total?total:1), g_prof[idx[a]].n, sym, g_prof[idx[a]].pc);
    }
}
int main(int argc, char **argv)
{
    nconfig cfg;
    char defbios[512];
    /* default bios: dir of exe, else /home/gill/neoragex */
    snprintf(defbios, sizeof defbios, "/home/gill/neoragex");
    cfg_default(&cfg, defbios);

    const char *bios = NULL, *game = NULL, *ini = NULL;
    const char *dump = NULL, *state = NULL, *save_state = NULL;
    const char *backend_opt = NULL, *dump_audio = NULL;
    int16_t *wavbuf = NULL; size_t wavn = 0, wavcap = 0;
    int frames = 0, headless = 0, no_audio = 0;
    int scale = -1, scanl = -1, fullscreen = 0;
    int netmode = 0, netport = 55300; const char *netaddr = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i],"--bios") && i+1<argc) bios = argv[++i];
        else if ((!strcmp(argv[i],"--game") || !strcmp(argv[i],"--rom")) && i+1<argc) game = argv[++i];
        else if (!strcmp(argv[i],"--ini") && i+1<argc) ini = argv[++i];
        else if (!strcmp(argv[i],"--scale") && i+1<argc) scale = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--backend") && i+1<argc) backend_opt = argv[++i];
        else if (!strcmp(argv[i],"--frames") && i+1<argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--dump")) dump = i+1<argc ? argv[++i] : "frame.bmp";
        else if (!strcmp(argv[i],"--state") && i+1<argc) state = argv[++i];
        else if (!strcmp(argv[i],"--save-state") && i+1<argc) save_state = argv[++i];
        else if (!strcmp(argv[i],"--headless")) headless = 1;
        else if (!strcmp(argv[i],"--no-audio")) no_audio = 1;
        else if (!strcmp(argv[i],"--dump-audio") && i+1<argc) dump_audio = argv[++i];
        else if (!strcmp(argv[i],"--scanlines")) scanl = 1;
        else if (!strcmp(argv[i],"--fullscreen")) fullscreen = 1;
        else if (!strcmp(argv[i],"--net") && i+1<argc) {
            const char *m = argv[++i];
            if (!strcmp(m,"host")) { netmode = 1; if (i+1<argc && atoi(argv[i+1])>0) netport = atoi(argv[++i]); }
            else if (!strcmp(m,"client") && i+1<argc) { netmode = 2; netaddr = argv[++i]; }
        }
        else if (!strcmp(argv[i],"--help")) {
            printf("linoragx — Neo Geo emulator (NeoRageX rewrite), Wayland/X11+Vulkan\n"
                   "  --bios DIR     BIOS dir (default /home/gill/neoragex)\n"
                   "  --game F.zip   game zip with .p1/.c1/.s1 members (alias: --rom)\n"
                   "  --ini FILE     neorage.ini-style config\n"
                   "  --scale N      window scale (default 2)\n"
                   "  --backend NAME window backend: wayland | x11 | headless (auto-detect)\n"
                   "  --scanlines    scanline effect\n"
                   "  --fullscreen   start fullscreen (toggle: F11 or Alt+Enter)\n"
                   "  --net host [port]    LAN netplay: host P1, wait for client (55300)\n"
                   "  --net client IP[:P]  LAN netplay: join host as P2\n"
                   "  --headless     no window; use with --frames/--dump\n"
                   "  --frames N     run N frames then exit\n"
                   "  --dump F.bmp   render last frame offscreen to BMP\n"
                   "  --state F      load NRXSTATE07 save state\n"
                   "  --save-state F save state at exit\n"
                   "  --no-audio     mute\n"
                   "  --dump-audio F write mixed PCM of all frames to wav F\n"
                   "keys: arrows/ZXCVBN=P1  space=start  5/6=coins\n");
            return 0;
        }
    }
    if (ini) cfg_load(&cfg, ini);
    if (bios) snprintf(cfg.rompath, sizeof cfg.rompath, "%s", bios);
    if (game) snprintf(cfg.gamezip, sizeof cfg.gamezip, "%s", game);
    if (scale >= 0) cfg.scale = scale;
    if (scanl >= 0) cfg.scanlines = scanl;
    if (no_audio) cfg.audio = 0;
    if (dump_audio) cfg.audio = 1;   /* audio dump forces generation even headless */
    if (!cfg.rompath[0]) snprintf(cfg.rompath, sizeof cfg.rompath, "%s", defbios);

    char err[512];
    fprintf(stderr, "[paths] rompath=%s gamezip=%s\n", cfg.rompath, cfg.gamezip);
    neogeo *ng = ng_create(cfg.rompath, cfg.gamezip, err, sizeof err);
    if (ng && getenv("PCPROF")) ng_dbg_pcprof(ng, 1);
    if (err[0]) fprintf(stderr, "[ng] %s\n", err);
    if (!ng) return 1;
    if (getenv("Z80DBG")) ng_dbg_z80(ng, atoi(getenv("Z80DBG")));

    /* LAN netplay: lockstep input exchange (host = local P1, client = local P2) */
    int netfd = -1, lplayer = 0;
    if (netmode) {
        uint32_t gcrc = ng_game_crc(ng);
        if (netmode == 1) {
            fprintf(stderr, "[net] hosting on :%d, waiting for client...\n", netport);
            netfd = net_host(netport, gcrc);
            lplayer = 0;
        } else {
            fprintf(stderr, "[net] connecting to %s...\n", netaddr);
            netfd = net_connect(netaddr, gcrc);
            lplayer = 1;
        }
        if (netfd < 0) { fprintf(stderr, "[net] link failed (game mismatch? firewall? port busy?)\n"); ng_destroy(ng); return 1; }
        fprintf(stderr, "[net] linked, local player = P%d\n", lplayer + 1);
    }

    if (state) {
        int r = ng_load_state(ng, state);
        if (r) fprintf(stderr, "[state] load failed (%d)\n", r);
        else fprintf(stderr, "[state] loaded %s\n", state);
    }

    /* audio */
    int16_t *sndbuf = malloc(48000*2*2);
    double samp_acc = 0;
    if (cfg.audio && !headless && !getenv("LINORAGX_NOALSA")) audio_init(48000);

    /* renderer: pick window backend (wayland > x11 > headless) */
    const char *wdisp = getenv("WAYLAND_DISPLAY");
    const char *backend = NULL;
    if (!headless) {
        if (backend_opt) backend = (!strcmp(backend_opt,"headless")) ? NULL : backend_opt;
        else if (wdisp) backend = "wayland";
        else if (getenv("DISPLAY")) backend = "x11";
    }
    vkrend *vk = NULL;
    vk = vk_init(ng, backend, cfg.scale, cfg.scanlines, err, sizeof err);
    if (!vk && backend) {
        fprintf(stderr, "[vk] %s window init failed (%s), trying headless\n", backend, err);
        backend = NULL;
        vk = vk_init(ng, NULL, cfg.scale, cfg.scanlines, err, sizeof err);
    } else if (!vk) {
        fprintf(stderr, "[vk] headless init: %s\n", err);
    }
    if (vk && backend) fprintf(stderr, "[vk] window backend: %s\n", backend);
    if (vk && fullscreen) vk_set_fullscreen(vk, 1);
    wl_input *in = (backend && !strcmp(backend,"wayland")) ? wl_input_create(wdisp) : NULL;

    uint32_t *fb = malloc(NG_W*NG_H*4);
    if (vk) ng_set_fb(ng, fb);   /* window mode: ng_run_frame draws per scanline (MAME) */
    if (getenv("LINORAGX_CTL")) {
        int cp = atoi(getenv("LINORAGX_CTL"));
        if (cp > 0 && ctl_open(cp) != 0) fprintf(stderr, "[ctl] open failed on port %d\n", cp);
    }
    int n = 0;
    if (envf_prof()) prof_start();
    double t0 = now_s();
    while (!frames || n < frames) {
        if (vk && vk_pump(vk)) break;
        if (in || vk) {
            uint32_t sym; int down; static int mod_alt = 0, mod_ctl = 0;
            while ((in ? wl_input_next(in, &sym, &down) : vk_next_key(vk, &sym, &down))) {
                if (sym == 0xffe9 || sym == 0xffea) mod_alt = down;
                else if (sym == 0xffe3 || sym == 0xffe4) mod_ctl = down;
                else if (down && ((mod_alt && sym == 0xffc1) || (mod_ctl && sym == 'q'))) { n = frames; break; }
                else if (down && vk && (sym == 0xffbe || (mod_alt && sym == 0xff0d)))
                    vk_toggle_fullscreen(vk);   /* F11 / Alt+Enter */
                handle_key(ng, sym, down);
            }
            if (in && vk) { while (vk_next_key(vk, &sym, &down)) handle_key(ng, sym, down); }
        }
        if (ctl_active()) {
            int cr = ctl_poll(ng, fb);
            if (cr & 1) break;
            if (cr & 2) {   /* paused: keep window alive, no emulation */
                if (vk) vk_present(vk, fb);
                struct timespec ts = {0, 10*1000*1000}; nanosleep(&ts, NULL);
                continue;
            }
        }
        /* scripted input: LINORAGX_INPUT="frame:bit:hold,..." (P1) or
           "frame:player:bit:hold,..." (4-field form, for netplay P2 tests) */
        {
            struct injev { int f, p, b, down; };
            static struct injev evs[128]; static int nevs = -1, ei = 0;
            if (nevs == -1) {
                nevs = 0; const char *s = getenv("LINORAGX_INPUT");
                while (s && *s && nevs + 1 < 128) { int a,b,c,d; int p = 0, bit, hold;
                    if (sscanf(s,"%d:%d:%d:%d",&a,&b,&c,&d)==4) { p=b; bit=c; hold=d; }
                    else if (sscanf(s,"%d:%d:%d",&a,&b,&c)==3) { bit=b; hold=c; }
                    else { bit=-1; hold=0; }
                    if (bit >= 0) {
                        evs[nevs++]=(struct injev){a,p,bit,1}; evs[nevs++]=(struct injev){a+hold,p,bit,0}; }
                    s = strchr(s, ','); if (s) s++; }
                for (int i=0;i<nevs;i++) for (int j=i+1;j<nevs;j++)
                    if (evs[j].f < evs[i].f) { struct injev t=evs[i]; evs[i]=evs[j]; evs[j]=t; }
            }
            while (ei < nevs && evs[ei].f <= n) { ng_set_button(ng, evs[ei].p, evs[ei].b, evs[ei].down); ei++;
                if (getenv("INPDBG")) fprintf(stderr, "[inp] f=%d p%d bit=%d down=%d\n", evs[ei-1].f, evs[ei-1].p, evs[ei-1].b, evs[ei-1].down); }
        }
        if (netfd >= 0) {
            uint32_t remote;
            if (net_step(netfd, n, ng_get_buttons(ng, lplayer), &remote) < 0) {
                fprintf(stderr, "[net] peer lost at frame %d\n", n);
                break;
            }
            ng_set_buttons(ng, 1 - lplayer, remote);
        }
        ng_run_frame(ng);
        if (!vk) ng_render(ng, fb);   /* window mode already drew per line in ng_run_frame */
        {   /* LINDUMPF=N: at frame N dump vram+cram+zoomy to /tmp/lindump.bin and fb to /tmp/lindump.bmp */
            static int lindf = -2;
            if (lindf == -2) { const char *e = getenv("LINDUMPF"); lindf = e ? atoi(e) : -1; }
            if (lindf >= 0 && (int)n == lindf) {
                extern int dump_bmp(const uint32_t *fb, const char *path);
                uint16_t *vr = ng_vram(ng), *cr = ng_cram(ng);
                uint8_t *zy = ng_zoomy(ng);
                FILE *f = fopen("/tmp/lindump.bin", "wb");
                if (f) {
                    for (int i = 0; i < 0x8800; i++) { uint8_t b[2]={(uint8_t)(vr[i]>>8),(uint8_t)vr[i]}; fwrite(b,1,2,f); }
                    for (int i = 0; i < 0x1000; i++) { uint8_t b[2]={(uint8_t)(cr[i]>>8),(uint8_t)cr[i]}; fwrite(b,1,2,f); }
                    fwrite(zy, 1, 0x10000, f);
                    fclose(f);
                }
                dump_bmp(fb, "/tmp/lindump.bmp");
                fprintf(stderr, "LINDUMP at frame %lld bank=%x\n", (long long)n, ng_cram_bank_render(ng));
            }
        }
        if (envf_dbg() && (n % 30 == 29)) {
            #include "m68k.h"
            uint16_t *cr = ng_cram(ng), *vr = ng_vram(ng);
            int cn=0, vn=0;
            for (int i=0;i<0x10000;i++) if (cr[i]&0x7FFF) cn++;
            for (int i=0;i<0x8800;i++) if (vr[i]) vn++;
            fprintf(stderr, "[dbg] n=%d cram_nz=%d vram_nz=%d pc=%08x fee4=%02x fe8c=%02x fee3=%02x fd80=%02x fd81=%02x fdbe=%08x ff00=%08x%08x\n", n, cn, vn,
                    m68k_get_reg(NULL, M68K_REG_PC),
                    m68k_read_memory_8(0x10FEE4), m68k_read_memory_8(0x10FE8C), m68k_read_memory_8(0x10FEE3),
                    m68k_read_memory_8(0x10FD80), m68k_read_memory_8(0x10FD81), m68k_read_memory_32(0x10FDBE),
                    m68k_read_memory_32(0x10FF00), m68k_read_memory_32(0x10FF04));
        }
        if (envf_tp())          /* render self-test */
            for (int y = 0; y < NG_H; y++)
                for (int x = 0; x < NG_W; x++)
                    fb[y*NG_W+x] = 0xFF000000u | ((y*8)<<16) | ((x*8)<<8) | 0x40;
        if (vk) vk_present(vk, fb);
        if (cfg.audio) {
            int ns = audio_level_adj(811);       /* 48000/59.185, rate-corrected */
            ng_audio(ng, sndbuf, ns);
            audio_submit(sndbuf, ns);
            if (dump_audio) {
                size_t need = wavn + ns*2;
                if (need > wavcap) {
                    wavcap = need < 1u<<20 ? 1u<<20 : need*2;
                    wavbuf = realloc(wavbuf, wavcap*2);
                }
                memcpy(wavbuf+wavn, sndbuf, ns*2*2);
                wavn = need;
            }
        }
        n++;
        if (!headless && (n % 60 == 0)) fprintf(stderr, "[fps] n=%d %.1f fps\n", n, n/(now_s()-t0));
        if (frames && n >= frames) {
            if (dump) {
                /* write the emulated framebuffer directly: the Vulkan
                   offscreen readback is viewport-flip sensitive */
                if (dump_bmp(fb, dump) == 0)
                    fprintf(stderr, "[dump] wrote %s\n", dump);
            }
            break;
        }
        if (!headless) {
            double want = t0 + n/59.185;
            double now = now_s();
            if (want > now) { struct timespec ts = {(long)want, (long)((want-(long)want)*1e9)};
                /* absolute pacing via nanosleep */
                struct timespec cur; clock_gettime(CLOCK_MONOTONIC,&cur);
                double d = want - (cur.tv_sec + cur.tv_nsec/1e9);
                if (d > 0 && d < 0.05) { ts.tv_sec=(long)d; ts.tv_nsec=(long)((d-(long)d)*1e9); nanosleep(&ts,NULL);} }
        }
    }
    (void)samp_acc;
    if (dump_audio) {
        FILE *f = fopen(dump_audio, "wb");
        if (f) {
            unsigned ds = (unsigned)(wavn*2), rs = 48000*4, cs = 36+ds;
            unsigned v;
            fwrite("RIFF",1,4,f); v=cs; fwrite(&v,4,1,f); fwrite("WAVE",1,4,f);
            fwrite("fmt ",1,4,f); v=16; fwrite(&v,4,1,f);
            v=1; fwrite(&v,2,1,f); v=2; fwrite(&v,2,1,f);        /* PCM, stereo */
            v=48000; fwrite(&v,4,1,f); v=rs; fwrite(&v,4,1,f);
            v=4; fwrite(&v,2,1,f); v=16; fwrite(&v,2,1,f);
            fwrite("data",1,4,f); v=ds; fwrite(&v,4,1,f);
            if (wavn) fwrite(wavbuf,2,wavn,f);
            fclose(f);
            fprintf(stderr, "[dump-audio] wrote %s (%u samples)\n", dump_audio, ds/2);
        } else fprintf(stderr, "[dump-audio] cannot open %s\n", dump_audio);
    }
    free(wavbuf);
    if (envf_prof()) prof_dump();
    fprintf(stderr, "[linoragx] ran %d frames (%.1f fps)\n", n, n/(now_s()-t0+1e-9));
    if (save_state) {
        int r = ng_save_state(ng, save_state);
        fprintf(stderr, "[state] save %s (%d)\n", save_state, r);
    }
    if (in) wl_input_destroy(in);
    if (vk) vk_destroy(vk);
    audio_close();
    ng_destroy(ng);
    free(fb); free(sndbuf);
    return 0;
}
