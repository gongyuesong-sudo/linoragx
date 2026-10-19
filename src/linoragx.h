#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define NG_W 320
#define NG_H 224

typedef struct neogeo neogeo;

/* machine */
neogeo *ng_create(const char *biosdir, const char *gamezip, char *err, size_t errsz);
void    ng_destroy(neogeo *ng);
void    ng_run_frame(neogeo *ng);
void    ng_render(neogeo *ng, uint32_t *fb /* NG_W*NG_H RGBA8888 */);
void    ng_audio(neogeo *ng, int16_t *stereo, int frames);
void    ng_set_button(neogeo *ng, int player, int bit, int down);
void    ng_press_key(neogeo *ng, int keysym);      /* map xkb keysym -> buttons */
uint64_t ng_frame_no(neogeo *ng);
const char *ng_game_name(neogeo *ng);

/* save state (NRXSTATE07 compatible magic) */
int ng_save_state(neogeo *ng, const char *path);
int ng_load_state(neogeo *ng, const char *path);

/* rom helpers */
int  rom_load_file(const char *path, uint8_t **out, uint32_t *outsz);
int  rom_load_zip(const char *zip, const char *member, uint8_t **out, uint32_t *outsz);
int  rom_load_zip_suffix(const char *zip, const char *suffix, uint8_t **out, uint32_t *outsz);
uint16_t *ng_vram(neogeo *ng);
uint16_t *ng_workram(neogeo *ng);
uint16_t *ng_cram(neogeo *ng);
uint32_t ng_cram_bank(neogeo *ng);
uint32_t ng_cram_bank_render(neogeo *ng);
int ng_dbg_vbank(neogeo *ng);
void ng_dbg_z80(neogeo *ng, int on);
void ng_dbg_btn(neogeo *ng, int p, int mask);
void ng_dbg_pcprof(neogeo *ng, int on);
uint8_t  *ng_rom(neogeo *ng, uint32_t *sz);
uint8_t  *ng_fixed(neogeo *ng, uint32_t *sz);
uint8_t  *ng_sprite8(neogeo *ng, uint32_t *mask);
uint8_t  *ng_spr_raw(neogeo *ng);
uint8_t  *ng_spr(neogeo *ng, uint32_t *sz);
uint8_t  *ng_zoomy(neogeo *ng);
int       ng_auto_counter(neogeo *ng);
uint32_t rom_crc32(const uint8_t *d, uint32_t n);

/* config (neorage.ini subset) */
typedef struct {
    char rompath[512];
    char gamezip[512];
    int  scanlines;        /* 0 off,1 on,2 50% */
    int  vsync;
    int  audio;
    int  scale;
} nconfig;
void cfg_default(nconfig *c, const char *biosdir);
int  cfg_load(nconfig *c, const char *inipath);
int  cfg_save(nconfig *c, const char *inipath);

/* ym2610 */
typedef struct ym2610 ym2610;
ym2610 *ym_create(void);
void ym_destroy(ym2610 *y);
void ym_reset(ym2610 *y);
void ym_write(ym2610 *y, int chip, uint8_t reg, uint8_t val);
uint8_t ym_read(ym2610 *y, int chip);
int ym_irq_pending(ym2610 *y);
void ym_stream(ym2610 *y, int16_t *stereo, int frames);
void ym_advance(ym2610 *y, int zcycles);

/* audio out (ALSA via dlopen, silent fallback) */
int  audio_init(int rate);
void audio_close(void);
void audio_submit(const int16_t *stereo, int frames);

/* z80 wrapper (z80_wrap.cpp) */
void      *z80w_create(void *ctx, uint8_t (*rd)(uint32_t,void*), void (*wr)(uint32_t,uint8_t,void*),
                       uint16_t (*in)(uint32_t,void*), void (*out)(uint32_t,uint8_t,void*));
void       z80w_destroy(void *v);
void       z80w_reset(void *v);
unsigned long z80w_exec(void *v, unsigned long t);
void       z80w_irq(void *v, uint8_t vector);
void       z80w_nmi(void *v);
int        z80w_save(void *v, void *buf);
int        z80w_load(void *v, const void *buf);

/* ctl channel (LINORAGX_CTL=<port>, localhost TCP line protocol) */
int  ctl_open(int port);
int  ctl_active(void);
int  ctl_poll(neogeo *ng, const uint32_t *fb);   /* bit0 quit, bit1 skip-frame */
int  dump_bmp(const uint32_t *fb, const char *path);

/* vulkan renderer */
typedef struct vkrend vkrend;
/* backend: "wayland" | "x11" | NULL (headless) */
vkrend *vk_init(neogeo *ng, const char *backend, int scale, int scanlines, char *err, size_t errsz);
void   vk_destroy(vkrend *v);
int    vk_pump(vkrend *v);                    /* handle events; 0=ok, 1=close */
int    vk_next_key(vkrend *v, uint32_t *sym, int *down);  /* X11 key events; 1=popped */
void   vk_present(vkrend *v, const uint32_t *fb);
int    vk_offscreen_dump(vkrend *v, const uint32_t *fb, const char *bmp_path);
