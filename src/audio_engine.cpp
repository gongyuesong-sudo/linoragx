/* MAME-style audio mixer (src/emu/sound + speaker equivalent).
 *
 * Each YM2610 sub-engine is an independent stream generated at the chip's
 * native rate (clock/16 = 500 kHz at the MAME-correct NEOGEO_YM2610_CLOCK
 * of master/3 = 8 MHz).  Streams are band-limited resampled with a
 * ratio-aware windowed-sinc interpolator (polyphase table, Blackman,
 * cutoff = out/2 — the same principle as MAME's speex resampler) and mixed
 * with MAME's NeoGeo speaker routing (neogeo.cpp):
 *
 *     add_route(0, speaker, 0.84, 0/1)   FM+SSG  -> L and R  x0.84
 *     add_route(1, speaker, 0.98, 0)     ADPCM-A -> L        x0.98
 *     add_route(2, speaker, 0.98, 1)     ADPCM-B -> R        x0.98
 *
 * Replaces the previous single-mixed-stream path that ran the chip at a
 * wrong 3.58 MHz clock (2.24x slow) with Butterworth + linear decimation.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

struct Stream {
    std::vector<float> buf;   /* native-rate samples, oldest first */
    double pos;               /* fractional read cursor (index into buf) */
};

struct audio_engine {
    double in_rate, out_rate, step;
    /* polyphase kernel: PHASES sub-filters of TAPS taps each */
    static const int TAPS = 32, PHASES = 256;
    std::vector<float> kern;          /* [PHASES*TAPS] */
    Stream fmL, fmR, adpcmA, adpcmB;
    /* MAME NeoGeo speaker volumes */
    static constexpr float VOL_FM = 0.84f, VOL_ADPCM = 0.98f;
};

/* Blackman window, symmetric over (-L, L) */
static double blackman(double x) {           /* x in (-1,1) */
    return 0.42 + 0.50 * cos(2.0 * M_PI * x / 2.0)
               + 0.08 * cos(2.0 * M_PI * x);
}

static void build_kernel(audio_engine *e) {
    /* cutoff normalized to the input rate; for downsampling fc = out/(2*in) */
    double fc = e->out_rate < e->in_rate ? e->out_rate / (2.0 * e->in_rate) : 0.5;
    int L = e->TAPS / 2;
    e->kern.resize((size_t)e->PHASES * e->TAPS);
    for (int ph = 0; ph < e->PHASES; ph++) {
        double frac = (double)ph / e->PHASES;      /* distance into the next input sample */
        float *k = &e->kern[(size_t)ph * e->TAPS];
        double sum = 0.0;
        for (int t = 0; t < e->TAPS; t++) {
            double d = (t - (L - 1)) - frac;       /* tap offset from read point */
            double xd = d / (L + 0.5);
            double w;
            if (fabs(xd) >= 1.0) w = 0.0;
            else {
                double s = sin(2.0 * M_PI * fc * d);
                double c = (fabs(d) < 1e-9) ? 1.0 : (M_PI * fc * d);
                w = 2.0 * fc * (s / c) * blackman(xd);
            }
            k[t] = (float)w;
            sum += w;
        }
        if (sum != 0.0) for (int t = 0; t < e->TAPS; t++) k[t] = (float)(k[t] / sum);
    }
}

extern "C" {

audio_engine *ae_create(double in_rate, double out_rate) {
    audio_engine *e = new audio_engine();
    e->in_rate = in_rate; e->out_rate = out_rate;
    e->step = in_rate / out_rate;
    build_kernel(e);
    e->fmL.pos = e->fmR.pos = e->adpcmA.pos = e->adpcmB.pos = 0.0;
    return e;
}

void ae_destroy(audio_engine *e) { delete e; }

void ae_reset(audio_engine *e) {
    e->fmL.buf.clear(); e->fmR.buf.clear();
    e->adpcmA.buf.clear(); e->adpcmB.buf.clear();
    e->fmL.pos = e->fmR.pos = e->adpcmA.pos = e->adpcmB.pos = 0.0;
}

/* push native-rate samples from the split ymfm streams */
void ae_feed(audio_engine *e, const int32_t *fmL, const int32_t *fmR,
             const int32_t *adpcmA, const int32_t *adpcmB, int n) {
    Stream *ss[4] = { &e->fmL, &e->fmR, &e->adpcmA, &e->adpcmB };
    const int32_t *src[4] = { fmL, fmR, adpcmA, adpcmB };
    for (int s = 0; s < 4; s++)
        for (int i = 0; i < n; i++)
            ss[s]->buf.push_back((float)src[s][i]);
}

/* unconsumed native samples available (min across streams) */
int ae_avail(audio_engine *e) {
    Stream *ss[4] = { &e->fmL, &e->fmR, &e->adpcmA, &e->adpcmB };
    long avail = 1L << 30;
    for (int s = 0; s < 4; s++) {
        long a = (long)ss[s]->buf.size() - (long)ss[s]->pos;
        if (a < avail) avail = a;
    }
    return avail < 0 ? 0 : (int)avail;
}

/* pull frames of stereo output; returns frames produced */
int ae_pull(audio_engine *e, int16_t *out, int frames) {
    int L = audio_engine::TAPS / 2;
    const float *kern = e->kern.data();
    Stream *ss[4] = { &e->fmL, &e->fmR, &e->adpcmA, &e->adpcmB };
    double step = e->step;
    for (int i = 0; i < frames; i++) {
        float v[4];
        for (int s = 0; s < 4; s++) {
            Stream &st = *ss[s];
            if (st.buf.empty()) { v[s] = 0.0f; continue; }
            size_t base = (size_t)st.pos;
            if (base >= st.buf.size()) base = st.buf.size() - 1;
            double d = st.pos - base;
            if (d < 0.0) d = 0.0;
            if (d > 0.999999) d = 0.999999;
            int ph = (int)(d * audio_engine::PHASES);
            if (ph >= audio_engine::PHASES) ph = audio_engine::PHASES - 1;
            const float *k = kern + (size_t)ph * audio_engine::TAPS;
            const float *b = st.buf.data();
            size_t first = base + 1 >= (size_t)L ? base - (L - 1) : 0;
            size_t last  = base + (size_t)L;
            if (last >= st.buf.size()) last = st.buf.size() - 1;
            float acc = 0.0f;
            for (size_t j = first; j <= last; j++)
                acc += b[j] * k[(int)(j - first) + (int)(first - (base - (L - 1)))];
            v[s] = acc;
            st.pos += step;
        }
        float l = v[0] * audio_engine::VOL_FM + v[2] * audio_engine::VOL_ADPCM;
        float r = v[1] * audio_engine::VOL_FM + v[3] * audio_engine::VOL_ADPCM;
        int il = (int)l, ir = (int)r;
        if (il > 32767) il = 32767; if (il < -32768) il = -32768;
        if (ir > 32767) ir = 32767; if (ir < -32768) ir = -32768;
        out[i*2] = (int16_t)il; out[i*2+1] = (int16_t)ir;
    }
    /* retire consumed history (keep TAPS of lookback before the cursor) */
    for (int s = 0; s < 4; s++) {
        Stream &st = *ss[s];
        size_t drop = (size_t)st.pos;
        if (drop > 0) {
            size_t keep_from = drop > (size_t)L ? drop - (size_t)L : 0;
            st.buf.erase(st.buf.begin(), st.buf.begin() + keep_from);
            st.pos -= (double)keep_from;
        }
    }
    return frames;
}

} /* extern "C" */
