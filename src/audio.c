/* ALSA output via dlopen (no dev headers needed); silent fallback. */
#include "linoragx.h"
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void *hnd;
static void *pcm;
static int (*p_open)(void**,const char*,int,int);
static int (*p_close)(void*);
static int (*p_set_hw_params_all)(void*,unsigned,int,unsigned,int,int,int);
static int (*p_writei)(void*,const void*,unsigned);
static int (*p_prepare)(void*);
static int (*p_drop)(void*);

static int16_t *ring;            /* stereo frames, capacity rate_g*2 */
static int ring_cap, ring_head, ring_count;
static int rate_g = 44100;
static short *conv;

int audio_init(int rate)
{
    hnd = dlopen("libasound.so.2", RTLD_NOW);
    if (!hnd) { fprintf(stderr, "[audio] no libasound, silent\n"); return -1; }
    p_open  = dlsym(hnd, "snd_pcm_open");
    p_close = dlsym(hnd, "snd_pcm_close");
    p_set_hw_params_all = dlsym(hnd, "snd_pcm_set_params");
    p_writei = dlsym(hnd, "snd_pcm_writei");
    p_prepare = dlsym(hnd, "snd_pcm_prepare");
    p_drop = dlsym(hnd, "snd_pcm_drop");
    if (!p_open || !p_set_hw_params_all || !p_writei) { dlclose(hnd); hnd=0; return -1; }
    int err = p_open(&pcm, "default", 0 /*SND_PCM_STREAM_PLAYBACK*/, 1 /*NONBLOCK: a full buffer must never stall the video frame loop*/);
    if (err < 0) { fprintf(stderr, "[audio] no device (%s), silent\n", strerror(-err)); hnd=0; return -1; }
    /* snd_pcm_set_params(pcm, format, access, channels, rate, soft_resample, latency) */
    err = p_set_hw_params_all(pcm, 2 /*S16_LE*/, 3 /*RW_INTERLEAVED*/, 2, rate, 1, 500000);
    if (err < 0) { p_close(pcm); pcm=0; hnd=0; return -1; }
    rate_g = rate;
    conv = malloc(rate*2*4);
    ring_cap = rate*2; ring_head = ring_count = 0;
    ring = malloc(ring_cap * 2 * sizeof(int16_t));
    fprintf(stderr, "[audio] ALSA default @%dHz\n", rate);
    return 0;
}

void audio_close(void)
{
    if (pcm) { p_drop(pcm); p_close(pcm); pcm = 0; }
    if (hnd) dlclose(hnd);
    hnd = 0;
    free(conv); conv = 0;
    free(ring); ring = 0; ring_cap = ring_head = ring_count = 0;
}

/* submit 44100 stereo s16; resample to rate_g by nearest.
 * A NONBLOCK device returns -EAGAIN when its buffer is momentarily full:
 * the leftover must be queued and flushed next call — dropping whole chunks
 * (the old behaviour) punches 16ms holes in the waveform = audible clicking. */
static void ring_push(const int16_t *src, int n)
{
    if (n > ring_cap) { src += (n - ring_cap) * 2; n = ring_cap; }  /* chronic lag: keep newest */
    for (int i = 0; i < n; i++) {
        int t = (ring_head + ring_count) % ring_cap;
        ring[t*2] = src[i*2]; ring[t*2+1] = src[i*2+1];
        ring_count++;
        if (ring_count == ring_cap) ring_head = (ring_head + 1) % ring_cap;  /* overwrite oldest */
    }
}

void audio_submit(const int16_t *stereo, int frames)
{
    if (!pcm) return;
    int outn = (int)((long)frames * rate_g / 44100);
    if (outn > rate_g) outn = rate_g;
    for (int i = 0; i < outn; i++) {
        int s = (int)((long)i * 44100 / rate_g);
        if (s >= frames) s = frames-1;
        conv[i*2]   = stereo[s*2];
        conv[i*2+1] = stereo[s*2+1];
    }
    ring_push(conv, outn);

    int guard = 8;
    while (ring_count > 0 && guard-- > 0) {
        int tail = ring_cap - ring_head;
        int first = ring_count < tail ? ring_count : tail;
        int w = p_writei(pcm, ring + ring_head*2, first);
        if (w > 0) {
            ring_head = (ring_head + w) % ring_cap;
            ring_count -= w;
            if (w < first) break;                 /* buffer filled up; retry next frame */
        } else if (w == -11 /*-EAGAIN*/) {
            break;                                /* full: keep queue, never stall video */
        } else if (w == -32 /*-EPIPE: xrun*/) {
            p_prepare(pcm);                       /* recover and retry */
        } else {
            break;
        }
    }
    static int audbg = -2; static long acnt = 0;
    if (audbg == -2) audbg = getenv("AUDDBG") ? 1 : 0;
    if (audbg && (acnt++ % 300) == 0)
        fprintf(stderr, "[aud] in_frames=%d out=%d queued=%d\n", frames, outn, ring_count);
}
