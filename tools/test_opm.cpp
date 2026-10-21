/* Unit test v3: ymfm ym2610 FM bank uses the OPN (YM2612) core, NOT OPM.
 * FM registers must go through the LOW bus (write(0)=reg, write(1)=data)
 * with address >= 0x1D (lower addresses are intercepted for SSG/ADPCM-B).
 * Programs ch1 (in mask 0x36) FM-mode 4-op voice, checks FM peak. */
#include <cstdio>
#include <cstdint>
#include <vector>
#include "ymfm_opn.h"

struct TestIntf : public ymfm::ymfm_interface {
    virtual uint8_t ymfm_external_read(ymfm::access_class, uint32_t) override { return 0; }
};

int main() {
    TestIntf intf;
    ymfm::ym2610 chip(intf);          /* default mask 0x36 = ch 1,2,4,5 */
    chip.reset();
    auto W = [&](int r, int d) { chip.write(0, r); chip.write(1, d); };

    W(0x20, 0x00);                    /* test off, normal */
    int ch = 1;
    int ops[4] = {ch, ch+4, ch+8, ch+12};   /* op1,op3,op2,op4 offsets */
    for (int o : ops) {
        W(0x30 + o, 0x07);            /* MULT=7, DET=0 */
        W(0x38 + o, 0x00);            /* TL=0 (max volume) */
        W(0x50 + o, 0xF8);            /* RS=3, AR=15 (instant attack) */
        W(0x60 + o, 0x00);            /* AM=0, MUL=0? (AM off) */
        W(0x70 + o, 0x05);            /* DR=5 (slow decay) */
        W(0x80 + o, 0x00);            /* SR=0 (no release decay) */
        W(0x90 + o, 0xF0);            /* SSG-EN=1, DR=0 (sustain) */
    }
    W(0xA0 + ch, 0x00);               /* FNUM low */
    W(0xA4 + ch, 0x24);               /* BLOCK=2, FNUM high=4 */
    W(0xB0 + ch, 0x01);               /* FB=0, FN=1 (FM mode) */
    W(0x28, 0xF0 | 0x01);             /* KEYON: all slots, CH1 (ymfm OPNA: bit1-0=ch, bit2=+3 bank) */

    const int N = 8000;
    std::vector<ymfm::ym2610::output_data> fm(N), aa(N), ab(N);
    chip.generate_split(fm.data(), aa.data(), ab.data(), N);
    int peak_fm = 0;
    for (int i = 0; i < N; i++) {
        int a = fm[i].data[0] < 0 ? -fm[i].data[0] : fm[i].data[0];
        int b = fm[i].data[1] < 0 ? -fm[i].data[1] : fm[i].data[1];
        if (a > peak_fm) peak_fm = a;
        if (b > peak_fm) peak_fm = b;
    }
    printf("OPN unit: fm peak = %d  %s\n", peak_fm, peak_fm > 0 ? "SOUND OK" : "STILL SILENT");

    /* Control: same program on a plain ym2612 (pure OPN) to isolate whether
       the register program itself is valid. */
    {
        TestIntf i2;
        ymfm::ym2612 chip2(i2);
        chip2.reset();
        auto W2 = [&](int r, int d) { chip2.write(0, r); chip2.write(1, d); };
        W2(0x20, 0x00);
        for (int o : ops) {
            W2(0x30 + o, 0x07); W2(0x38 + o, 0x00); W2(0x50 + o, 0xF8);
            W2(0x60 + o, 0x00); W2(0x70 + o, 0x05); W2(0x80 + o, 0x00); W2(0x90 + o, 0xF0);
        }
        W2(0xA0 + ch, 0x00); W2(0xA4 + ch, 0x24); W2(0xB0 + ch, 0x01);
        W2(0x28, 0xF0 | 0x04);
        std::vector<ymfm::ym2612::output_data> o2(N);
        chip2.generate(o2.data(), N);
        int pk = 0;
        for (int i = 0; i < N; i++) {
            int a = o2[i].data[0] < 0 ? -o2[i].data[0] : o2[i].data[0];
            int b = o2[i].data[1] < 0 ? -o2[i].data[1] : o2[i].data[1];
            if (a > pk) pk = a; if (b > pk) pk = b;
        }
        printf("ym2612 control: peak = %d  %s\n", pk, pk > 0 ? "SOUND OK" : "SILENT");
    }
    return peak_fm > 0 ? 0 : 1;
}
