// 单元测试：验证 cpu_z80 核心对 DD 前缀指令的 PC 推进
#include <cstdio>
#include <cstring>
#include <cstdint>
extern "C" {
void *z80w_create(void*, unsigned char(*)(unsigned, void*), void(*)(unsigned, unsigned char, void*),
                  unsigned(*)(unsigned, void*), void(*)(unsigned, unsigned char, void*));
void z80w_set_step_hook(void*, void(*)(unsigned));
void z80w_reset(void*);
unsigned long z80w_exec(void*, unsigned long);
uint16_t z80w_pc(void*);
}
static unsigned char mem[0x10000];
static unsigned char rd(unsigned a, void*) { return mem[a]; }
static void wr(unsigned a, unsigned char v, void*) { mem[a] = v; }
static unsigned in_(unsigned, void*) { return 0xFF; }
static void out_(unsigned, unsigned char, void*) {}
static int n = 0;
static void hook(unsigned pc) { if (n < 12) printf("step%02d PC=%04x\n", n++, pc); }
int main() {
    memset(mem, 0, sizeof mem);
    // ld (ix+7),0 / nop / ld e,88 / call 2b7c / ret
    unsigned char prog[] = {0xDD,0x36,0x07,0x00, 0x00, 0x1E,0x88, 0xCD,0x7C,0x2B, 0xC9};
    memcpy(mem, prog, sizeof prog);
    void *z = z80w_create(0, rd, wr, in_, out_);
    z80w_reset(z);
    z80w_set_step_hook(z, hook);
    z80w_exec(z, 200);
    printf("final PC=%04x\n", z80w_pc(z));
    return 0;
}
