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
{ if (getenv("BLKRD") && off >= 0xFC30 && off < 0xFF00) {
    static long brd = 0; z80w *w0 = (z80w*)h->pUserArea;
    if ((brd++ % 100) == 0) fprintf(stderr, "BLKRD a=%04x pc=%04x\n", off, w0->cpu->get_state().PC); }
  z80w *w = (z80w*)h->pUserArea;
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
{ if (getenv("BLKRD") && off >= 0xFC30 && off < 0xFF00) {
    static long bwr = 0; z80w *w0 = (z80w*)h->pUserArea;
    if ((bwr++ % 50) == 0) fprintf(stderr, "BLKWR a=%04x v=%02x pc=%04x\n", off, v, w0->cpu->get_state().PC); }
  z80w *w = (z80w*)h->pUserArea;
  if (off >= 0xF800) { w->cpu->ram_array()[off] = v; return; }
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
int z80w_iff1(void *v) { return ((z80w*)v)->cpu->get_state().IFF1; }
int z80w_im(void *v) { return ((z80w*)v)->cpu->get_state().NIM; }
void z80w_force_ei(void *v) { cpu_z80::State st = ((z80w*)v)->cpu->get_state();
                              if (!st.IFF1) { st.IFF1 = 1; st.IFF2 = 1; ((z80w*)v)->cpu->set_state(st); } }
uint8_t z80w_a(void *v) { return (uint8_t)(((z80w*)v)->cpu->get_state().AF >> 8); }
unsigned long long z80w_dbg_frame = 0;
void z80w_set_step_hook(void *v, void (*h)(unsigned pc)) { ((z80w*)v)->cpu->set_step_hook(h); }
void z80w_irq(void *v, uint8_t vector) { ((z80w*)v)->cpu->mz80int(vector); }
void z80w_nmi(void *v) { ((z80w*)v)->cpu->mz80nmi(); }
uint16_t z80w_pc(void *v) { return ((z80w*)v)->cpu->GetPC(); }
unsigned long z80w_bc(void *v) { cpu_z80::State st = ((z80w*)v)->cpu->get_state(); return st.BC; }
uint16_t z80w_sp(void *v) { cpu_z80::State s = ((z80w*)v)->cpu->get_state(); return s.SP; }

int z80w_save(void *v, void *buf)   /* buf = cpu_z80::State */
{ *(cpu_z80::State*)buf = ((z80w*)v)->cpu->get_state(); return (int)sizeof(cpu_z80::State); }
int z80w_load(void *v, const void *buf)
{ ((z80w*)v)->cpu->set_state(*(const cpu_z80::State*)buf); return 0; }

}
