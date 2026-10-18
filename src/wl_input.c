/* Wayland input: seat + keyboard; xkbcommon via dlopen (runtime-only lib). */
#include <wayland-client.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

typedef struct xkb_context xkb_context;
typedef struct xkb_keymap xkb_keymap;
typedef struct xkb_state xkb_state;
enum { XKB_CONTEXT_NO_FLAGS = 0, XKB_KEYMAP_FORMAT_TEXT_V1 = 1, XKB_KEY_DOWN = 1, XKB_KEY_UP = 0 };

typedef xkb_context* (*fn_ctx_new)(int);
typedef xkb_keymap* (*fn_km_from_string)(xkb_context*, const char*, int);
typedef xkb_state* (*fn_state_new)(xkb_keymap*);
typedef uint32_t (*fn_key_sym)(xkb_state*, uint32_t);
typedef void (*fn_state_upd)(xkb_state*, uint32_t, int);

struct wl_input {
    struct wl_display *dpy;
    struct wl_registry *reg;
    struct wl_seat *seat;
    struct wl_keyboard *kbd;
    xkb_context *xctx;
    xkb_keymap *kmap;
    xkb_state *kstate;
    fn_key_sym key_sym;
    fn_state_upd state_upd;
    uint32_t ev[256]; int ev_w, ev_r;
};

static void seat_cap(void *d, struct wl_seat *s, uint32_t cap)
{
    struct wl_input *in = d;
    if ((cap & WL_SEAT_CAPABILITY_KEYBOARD) && !in->kbd) {
        in->kbd = wl_seat_get_keyboard(s);
        /* listener attached below via extern struct */
        extern const struct wl_keyboard_listener linoragx_kb_listener;
        wl_keyboard_add_listener(in->kbd, &linoragx_kb_listener, in);
    }
}
static void seat_name(void *d, struct wl_seat *s, const char *n) { (void)d;(void)s;(void)n; }
static const struct wl_seat_listener seat_listener = { seat_cap, seat_name };

static void kb_keymap(void *d, struct wl_keyboard *k, uint32_t fmt, int fd, uint32_t len);
static void kb_enter(void *d, struct wl_keyboard *k, uint32_t s, struct wl_surface *sv, struct wl_array *keys)
{ (void)d;(void)k;(void)s;(void)sv;(void)keys; }
static void kb_leave(void *d, struct wl_keyboard *k, uint32_t s, struct wl_surface *sv)
{ (void)d;(void)k;(void)s;(void)sv; }
static void kb_key(void *d, struct wl_keyboard *k, uint32_t serial, uint32_t time, uint32_t key, uint32_t st);
static void kb_mod(void *d, struct wl_keyboard *k, uint32_t s, uint32_t m, uint32_t g, uint32_t l, uint32_t o)
{ (void)d;(void)k;(void)s;(void)m;(void)g;(void)l;(void)o; }
static void kb_rep(void *d, struct wl_keyboard *k, int32_t rate, int32_t delay)
{ (void)d;(void)k;(void)rate;(void)delay; }

const struct wl_keyboard_listener linoragx_kb_listener = {
    kb_keymap, kb_enter, kb_leave, kb_key, kb_mod, kb_rep,
};

static void registry_global(void *d, struct wl_registry *r, uint32_t name, const char *iface, uint32_t ver)
{
    struct wl_input *in = d;
    if (!strcmp(iface, "wl_seat")) {
        in->seat = wl_registry_bind(r, name, &wl_seat_interface, ver < 5 ? ver : 5);
        wl_seat_add_listener(in->seat, &seat_listener, in);
    }
}
static void registry_rm(void *d, struct wl_registry *r, uint32_t n) { (void)d;(void)r;(void)n; }
static const struct wl_registry_listener reg_listener = { registry_global, registry_rm };

static void kb_keymap(void *dd, struct wl_keyboard *k, uint32_t fmt, int fd, uint32_t len)
{
    struct wl_input *in = dd;
    (void)k;
    if (fmt != 1) { close(fd); return; }
    char *buf = malloc(len+1);
    ssize_t got = read(fd, buf, len);
    close(fd);
    if (got <= 0) { free(buf); return; }
    buf[got] = 0;
    if (!in->xctx) { free(buf); return; }
    fn_km_from_string km = (fn_km_from_string)dlsym(RTLD_DEFAULT, "xkb_keymap_new_from_string");
    if (!km) km = (fn_km_from_string)dlsym(NULL, "xkb_keymap_new_from_string");
    if (!km) { free(buf); return; }
    in->kmap = km(in->xctx, buf, XKB_KEYMAP_FORMAT_TEXT_V1);
    free(buf);
    if (in->kmap) {
        xkb_state* (*sn)(xkb_keymap*) = (xkb_state*(*)(xkb_keymap*))dlsym(RTLD_DEFAULT, "xkb_state_new");
        if (sn) in->kstate = sn(in->kmap);
    }
}

static void kb_key(void *dd, struct wl_keyboard *k, uint32_t serial, uint32_t time, uint32_t key, uint32_t st)
{
    struct wl_input *in = dd;
    (void)k;(void)serial;(void)time;
    if (!in->kstate || !in->key_sym) return;
    uint32_t sym = in->key_sym(in->kstate, key + 8);
    if (in->state_upd) in->state_upd(in->kstate, key + 8, (int)st);
    in->ev[in->ev_w] = (sym & 0xFFFFFF) | ((st == XKB_KEY_DOWN) ? 0x1000000u : 0);
    in->ev_w = (in->ev_w + 1) & 255;
}

struct wl_input *wl_input_create(const char *disp)
{
    struct wl_input *in = calloc(1, sizeof *in);
    in->dpy = wl_display_connect(disp);
    if (!in->dpy) { free(in); return NULL; }
    void *h = dlopen("libxkbcommon.so.0", RTLD_NOW | RTLD_GLOBAL);
    if (h) {
        fn_ctx_new cn = (fn_ctx_new)dlsym(h, "xkb_context_new");
        in->key_sym = (fn_key_sym)dlsym(h, "xkb_state_key_get_one_sym");
        in->state_upd = (fn_state_upd)dlsym(h, "xkb_state_update_key");
        if (cn) in->xctx = cn(XKB_CONTEXT_NO_FLAGS);
    }
    in->reg = wl_display_get_registry(in->dpy);
    wl_registry_add_listener(in->reg, &reg_listener, in);
    wl_display_roundtrip(in->dpy);
    return in;
}

void wl_input_destroy(struct wl_input *in)
{
    if (!in) return;
    if (in->kbd) wl_keyboard_destroy(in->kbd);
    if (in->seat) wl_seat_destroy(in->seat);
    if (in->reg) wl_registry_destroy(in->reg);
    if (in->dpy) wl_display_disconnect(in->dpy);
    free(in);
}

int wl_input_next(struct wl_input *in, uint32_t *sym, int *down)
{
    if (!in || in->ev_r == in->ev_w) return 0;
    uint32_t e = in->ev[in->ev_r];
    in->ev_r = (in->ev_r + 1) & 255;
    *down = (e >> 24) & 1;
    *sym = e & 0xFFFFFF;
    return 1;
}
