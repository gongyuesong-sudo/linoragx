/* Unit test: does ymfm ym2610 FM (OPM bank A) actually produce output
 * through generate_split? Programs a simple OPM voice (CH1 freq, carrier
 * envelope, KEYON) and checks the fm stream peak. */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include "ymfm_opn.h"

struct TestIntf : public ymfm::ymfm_interface {
    virtual uint8_t ymfm_external_read(ymfm::access_class, uint32_t) override { return 0x55; }
};

int main() {
    TestIntf intf;
    ymfm::ym2610 chip(intf, 0x7f);
    chip.reset();
    chip.write(0, 0x01); chip.write(1, 0x00);   /* TEST off */

    /* OPM CH1: freq coarse/fine */
    chip.write(0, 0x10); chip.write(1, 0x10);   /* CH1 freq H */
    chip.write(0, 0x20); chip.write(1, 0x00);   /* CH1 freq L */
    chip.write(0, 0x38); chip.write(1, 0x00);   /* AM depth off? (0x38=ch8? use 0x30) */
    /* operator settings for CH1 slot4 (carrier): 0xC0+0x1C? OPM slot addr = reg base + (slot<<2 | ch)
       slot4 = 0xE0..0xE7 region: 0x60+0x1C=0x7C? Simpler: set ALL operator regs to a
       simple one-stage envelope with max output. */
    for (int slot = 0; slot < 4; slot++) {
        int base = 0x60 + slot * 0x20;          /* 0x60,0x80,0xA0,0xC0 */
        for (int ch = 0; ch < 8; ch++) {
            chip.write(0, base + 0x10 + ch); chip.write(1, 0x1F);   /* total level 0 (max) */
            chip.write(0, base + 0x20 + ch); chip.write(1, 0xF8);   /* attack fast, decay1 slow */
            chip.write(0, base + 0x30 + ch); chip.write(1, 0x00);   /* sustain max, release slow */
            chip.write(0, base + 0x00 + ch); chip.write(1, 0x01);   /* multi x1, detune 0 */
        }
    }
    chip.write(0, 0x0F); chip.write(1, 0x00);   /* IO/normal */
    chip.write(0, 0x28); chip.write(1, 0x31);   /* KEYON CH1 (BNH=0, KEYON bits 0x30|ch0) */

    /* generate via generate_split */
    const int N = 4000;
    std::vector<ymfm::ym2610::output_data> fm(N), aa(N), ab(N);
    chip.generate_split(fm.data(), aa.data(), ab.data(), N);
    int peak_fm = 0, peak_aa = 0;
    for (int i = 0; i < N; i++) {
        int a = fm[i].data[0] < 0 ? -fm[i].data[0] : fm[i].data[0];
        int b = fm[i].data[1] < 0 ? -fm[i].data[1] : fm[i].data[1];
        if (a > peak_fm) peak_fm = a;
        if (b > peak_fm) peak_fm = b;
        int c = aa[i].data[0] < 0 ? -aa[i].data[0] : aa[i].data[0];
        if (c > peak_aa) peak_aa = c;
    }
    printf("generate_split: fm peak=%d  adpcm_a peak=%d\n", peak_fm, peak_aa);

    /* control: same program through generate() (summed path) */
    ymfm::ym2610 chip2(intf, 0x7f);
    chip2.reset();
    chip2.write(0, 0x01); chip2.write(1, 0x00);
    chip2.write(0, 0x10); chip2.write(1, 0x10);
    chip2.write(0, 0x20); chip2.write(1, 0x00);
    for (int slot = 0; slot < 4; slot++) {
        int base = 0x60 + slot * 0x20;
        for (int ch = 0; ch < 8; ch++) {
            chip2.write(0, base + 0x10 + ch); chip2.write(1, 0x1F);
            chip2.write(0, base + 0x20 + ch); chip2.write(1, 0xF8);
            chip2.write(0, base + 0x30 + ch); chip2.write(1, 0x00);
            chip2.write(0, base + 0x00 + ch); chip2.write(1, 0x01);
        }
    }
    chip2.write(0, 0x28); chip2.write(1, 0x31);
    std::vector<ymfm::ym2610::output_data> sum(N);
    chip2.generate(sum.data(), N);
    int peak_sum = 0;
    for (int i = 0; i < N; i++) {
        int a = sum[i].data[0] < 0 ? -sum[i].data[0] : sum[i].data[0];
        if (a > peak_sum) peak_sum = a;
    }
    printf("generate(sum): peak=%d\n", peak_sum);
    return 0;
}
