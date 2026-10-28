/* extern "C" wrapper around the MIT-licensed cpu_z80 class (AAE core).
 * Handler tables use region-relative offsets + sentinel entries. */
#include "cpu_z80.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

typedef uint8_t  (*rd_fn)(uint32_t, void*);
typedef void     (*wr_fn)(uint32_t, uint8_t, void*);
typedef uint16_t (*in_fn)(uint32_t, void*);
typedef void     (*out_fn)(uint32_t, uint8_t, void*);

struct z80w {
    cpu_z80 *cpu;
    MemoryReadByte  rd_tab[2];
    MemoryWriteByte wr_tab[2];
    z80PortRead     prd_tab[2];
    z80PortWrite    pwr_tab[2];
    uint8_t ram[0x10000];
    rd_fn  rd;  wr_fn wr;  in_fn in;  out_fn out;
    void *ctx;
};

static UINT8 tr_rd(UINT32 off, MemoryReadByte *h)
{ if (getenv("BLKRD") && off >= 0xFEB0 && off < 0xFF00) {
    static long brd = 0; z80w *w0 = (z80w*)h->pUserArea;
    if (off < 0xFED0 || (brd++ % 100) == 0) fprintf(stderr, "BLKRD a=%04x pc=%04x\n", off, w0->cpu->get_state().PC); }
  z80w *w = (z80w*)h->pUserArea;
  if (getenv("ZTR") && off < 0xF800) { static long zt = 0;
    if (zt++ < 4000) fprintf(stderr, "ZTR rd=%04x pc=%04x\n", off, w->cpu->get_state().PC); }
  if (getenv("ZRAM") && off >= 0xFF00) { static long zr = 0;
    if (zr++ < 30) fprintf(stderr, "ZRAM rd=%04x pc=%04x\n", off, w->cpu->get_state().PC); }
  if (getenv("ZCRASH")) { static uint16_t ring[4096]; static int rp = 0; static int fired = 0;
    ring[rp++ & 4095] = w->cpu->get_state().PC;
    if (!fired && off == 0xFFFD) { fired = 1; fprintf(stderr, "ZCRASH ring (oldest->newest):\n");
      for (int qi = 0; qi < 4096; qi++) fprintf(stderr, "  %04x\n", ring[(rp + qi) & 4095]); } }
  /* THE FIX: the core's Push/Pop walk m_rgbStack as a POINTER into
     m_rgbMemory and read back via the internal array (mz80GetMemory's
     sentinel fallthrough). Routing 0xF800-0xFFFF to the callback RAM
     (ng->sndram) made Push write sndram while Pop read the untouched
     internal array -> every RET popped 0x0000: m1 boot-looped to
     0x0000 forever (pnyaa/rotd/mslug4 class). Serve the Z80 RAM window
     from the core's own array so stack traffic is coherent. */
  if (off >= 0xF800) return w->cpu->ram_array()[off];
  return w->rd(off + h->lowAddr, w->ctx); }
static void  tr_wr(UINT32 off, UINT8 v, MemoryWriteByte *h)
{ if (getenv("BLKRD") && ((off >= 0xFEB0 && off < 0xFF00) || (off >= 0xFE4A && off < 0xFE80))) {
    static long bwr = 0; z80w *w0 = (z80w*)h->pUserArea;
    if (off < 0xFEC0 || (off >= 0xFE4A && off < 0xFE80) || (off >= 0xFC30 && off < 0xFC60) || (bwr++ % 50) == 0) fprintf(stderr, "BLKWR a=%04x v=%02x pc=%04x\n", off, v, w0->cpu->get_state().PC); }
  z80w *w = (z80w*)h->pUserArea;
  if (off >= 0xF800) { w->cpu->ram_array()[off] = v;
    { static int wal = -2, wah = -1;
      if (wal == -2) { const char *e = getenv("Z80RAMWA");
        if (e) { wal = (int)strtol(e, 0, 16); const char *d = strchr(e, '-');
          wah = d ? (int)strtol(d + 1, 0, 16) : wal; } }
      if (wal >= 0 && (int)off >= wal && (int)off <= wah) {
        extern int g_ym_frame;
        fprintf(stderr, "ZRW f=%d a=%04x v=%02x pc=%04x\n", g_ym_frame, off, v,
                w->cpu->get_state().PC); } }
    if (getenv("FDWR") && (off == 0xFD67 || off == 0xFD66)) { static long fw = 0;
      if (fw++ < 40) fprintf(stderr, "FDWR a=%04x v=%02x zpc=%04x\n", off, v, w->cpu->get_state().PC);
      if (fw == 40) fprintf(stderr, "FDWR ...silenced\n"); }
    return; }
  w->wr(off + h->lowAddr, v, w->ctx); }
static UINT16 tr_in(UINT16 p, z80PortRead *h)
{ z80w *w = (z80w*)h->pUserArea; return w->in(p, w->ctx); }
static void  tr_out(UINT16 p, UINT8 v, z80PortWrite *h)
{ z80w *w = (z80w*)h->pUserArea; w->out(p, v, w->ctx); }

extern "C" {

void *z80w_create(void *ctx, rd_fn rd, wr_fn wr, in_fn in, out_fn out)
{
    z80w *w = new z80w();
    memset(w, 0, sizeof(*w));
    w->rd = rd; w->wr = wr; w->in = in; w->out = out; w->ctx = ctx;

    w->rd_tab[0] = {0, 0xFFFF, tr_rd, w};
    w->rd_tab[1] = {0xFFFFFFFF, 0, nullptr, nullptr};
    w->wr_tab[0] = {0, 0xFFFF, tr_wr, w};
    w->wr_tab[1] = {0xFFFFFFFF, 0, nullptr, nullptr};
    w->prd_tab[0] = {0, 0xFFFF, tr_in, w};
    w->prd_tab[1] = {0xFFFF, 0, nullptr, nullptr};
    w->pwr_tab[0] = {0, 0xFFFF, tr_out, w};
    w->pwr_tab[1] = {0xFFFF, 0, nullptr, nullptr};

    w->cpu = new cpu_z80(w->ram, w->rd_tab, w->wr_tab, w->prd_tab, w->pwr_tab, 0x38, 0);
    return w;
}
void z80w_destroy(void *v)
{ z80w *w = (z80w*)v; delete w->cpu; delete w; }

void z80w_reset(void *v) {
    ((z80w*)v)->cpu->mz80reset();
    /* core reset leaves SP=0; real Z80 SP floats until firmware sets it, but
       sm1/m1 rely on a sane stack before their first SP load — place it at the
       top of the 2K audio RAM (0xF800-0xFFFF). */
    cpu_z80::State st = ((z80w*)v)->cpu->get_state();
    st.SP = 0xFFF6;
    ((z80w*)v)->cpu->set_state(st);
}
unsigned long z80w_exec(void *v, unsigned long t) { return ((z80w*)v)->cpu->mz80exec(t); }
uint8_t z80w_peek(void *v, uint32_t off) { return ((z80w*)v)->cpu->ram_array()[off & 0xFFFF]; }
void z80w_poke(void *v, uint32_t off, uint8_t val) { ((z80w*)v)->cpu->ram_array()[off & 0xFFFF] = val; }
int z80w_iff1(void *v) { return ((z80w*)v)->cpu->get_state().IFF1; }
int z80w_im(void *v) { return ((z80w*)v)->cpu->get_state().NIM; }
void z80w_force_ei(void *v) { cpu_z80::State st = ((z80w*)v)->cpu->get_state();
                              if (!st.IFF1) { st.IFF1 = 1; st.IFF2 = 1; ((z80w*)v)->cpu->set_state(st); } }
void z80w_force_jmp(void *v, uint16_t pc) { cpu_z80::State st = ((z80w*)v)->cpu->get_state();
                              st.PC = pc; ((z80w*)v)->cpu->set_state(st); }
uint8_t z80w_a(void *v) { return (uint8_t)(((z80w*)v)->cpu->get_state().AF >> 8); }
uint16_t z80w_hl(void *v) { return (uint16_t)(((z80w*)v)->cpu->get_state().HL); }
uint16_t z80w_af(void *v) { return (uint16_t)(((z80w*)v)->cpu->get_state().AF); }
uint16_t z80w_de(void *v) { return (uint16_t)(((z80w*)v)->cpu->get_state().DE); }
unsigned long long z80w_dbg_frame = 0;
void z80w_set_step_hook(void *v, void (*h)(unsigned pc)) { ((z80w*)v)->cpu->set_step_hook(h); }
void z80w_irq(void *v, uint8_t vector) { ((z80w*)v)->cpu->mz80int(vector); }
void z80w_nmi(void *v) { ((z80w*)v)->cpu->mz80nmi(); }
uint16_t z80w_pc(void *v) { return ((z80w*)v)->cpu->GetPC(); }
void z80w_statdump(void *v) {
  z80w *w = (z80w*)v; cpu_z80::State s = w->cpu->get_state();
  fprintf(stderr, "ZSTAT pc=%04x sp=%04x af=%04x bc=%04x de=%04x hl=%04x halt=%d iff1=%d iff2=%d im=%d i=%02x ram[fffc..ffff]=%02x %02x %02x %02x\n",
          s.PC, s.SP, s.AF, s.BC, s.DE, s.HL, s.FHalt, s.IFF1, s.IFF2, s.NIM, s.I,
          w->ram[0xfffc], w->ram[0xfffd], w->ram[0xfffe], w->ram[0xffff]);
}
unsigned long z80w_bc(void *v) { cpu_z80::State st = ((z80w*)v)->cpu->get_state(); return st.BC; }
uint16_t z80w_sp(void *v) { cpu_z80::State s = ((z80w*)v)->cpu->get_state(); return s.SP; }
void z80w_regdump(void *v, unsigned *out6) {
  cpu_z80::State s = ((z80w*)v)->cpu->get_state();
  out6[0]=s.PC; out6[1]=s.SP; out6[2]=s.AF; out6[3]=s.BC; out6[4]=s.DE; out6[5]=s.HL;
}

int z80w_save(void *v, void *buf)   /* buf = cpu_z80::State */
{ *(cpu_z80::State*)buf = ((z80w*)v)->cpu->get_state(); return (int)sizeof(cpu_z80::State); }
int z80w_load(void *v, const void *buf)
{ ((z80w*)v)->cpu->set_state(*(const cpu_z80::State*)buf); return 0; }
/* NRXSTATE11: private Z80 RAM (0xF800-0xFFFF) was never serialized; the sound
 * driver's task blocks live there, so any resume silently wiped them. */
int z80w_save_ram(void *v, void *buf)
{ memcpy(buf, ((z80w*)v)->cpu->ram_array() + 0xF800, 0x800); return 0; }
int z80w_load_ram(void *v, const void *buf)
{ memcpy(((z80w*)v)->cpu->ram_array() + 0xF800, buf, 0x800); return 0; }

}
