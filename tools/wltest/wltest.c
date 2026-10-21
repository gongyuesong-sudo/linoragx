/* Minimal headless Wayland test compositor for linoragx verification.
 * Implements wl_compositor, xdg_wm_base, wl_seat+keyboard, dmabuf.
 * No scanout: proves bind + Vulkan surface + present loop + keys.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/memfd.h>
static int memfd_create_(const char *n, unsigned f){ return (int)syscall(SYS_memfd_create, n, f); }
#define memfd_create memfd_create_
#include <wayland-server.h>
#include <xkbcommon/xkbcommon.h>
#include "xdg-shell-server-protocol.h"
#include "dmabuf-server-protocol.h"

static struct wl_display *disp;
static uint32_t sc_ctr = 1;
static int g_commits = 0, g_buffers = 0, g_keyev = 0;
static struct wl_resource *g_kbd = NULL;
static struct wl_resource *g_top_wl_surf = NULL;
static struct xkb_context *xkbctx;
static void send_enter(void);

/* ---------- generic no-ops ---------- */
static void s_destroy(struct wl_client *c, struct wl_resource *r){ wl_resource_destroy(r); }
static void s_nop(struct wl_client *c, struct wl_resource *r){ (void)c;(void)r; }
static void s_nop_r(struct wl_client *c, struct wl_resource *r, struct wl_resource *a){ (void)c;(void)r;(void)a; }
static void s_nop_s(struct wl_client *c, struct wl_resource *r, const char *a){ (void)c;(void)r;(void)a; }
static void s_i32(struct wl_client *c, struct wl_resource *r, int32_t a){ (void)c;(void)r;(void)a; }
static void s_2i(struct wl_client *c, struct wl_resource *r, int32_t a, int32_t b){ (void)c;(void)r;(void)a;(void)b; }
static void s_4i(struct wl_client *c, struct wl_resource *r, int32_t a, int32_t b, int32_t w, int32_t h){ (void)c;(void)r;(void)a;(void)b;(void)w;(void)h; }
static void s_u(struct wl_client *c, struct wl_resource *r, uint32_t a){ (void)c;(void)r;(void)a; }
static void s_seat_ser(struct wl_client *c, struct wl_resource *r, struct wl_resource *s, uint32_t ser){ (void)c;(void)r;(void)s;(void)ser; }
static void s_resize(struct wl_client *c, struct wl_resource *r, struct wl_resource *s, uint32_t ser, uint32_t w, uint32_t h){ (void)c;(void)r;(void)s;(void)ser;(void)w;(void)h; }
static void s_out(struct wl_client *c, struct wl_resource *r, struct wl_output *o){ (void)c;(void)r;(void)o; }

/* ---------- wl_surface / compositor ---------- */
struct surf_ctx { int is_top; };
static void s_attach(struct wl_client *c, struct wl_resource *r, struct wl_resource *b, int32_t x, int32_t y){
    (void)c;(void)r;(void)x;(void)y;
    if (b) { g_buffers++; wl_buffer_send_release(b); }  /* immediate recycle for WSI */
}
static void s_frame(struct wl_client *c, struct wl_resource *r, uint32_t cid){
    struct wl_resource *cb = wl_resource_create(c, &wl_callback_interface, 1, cid);
    wl_callback_send_done(cb, 0);
    wl_resource_destroy(cb);
}
static void s_commit(struct wl_client *c, struct wl_resource *r);
static const struct wl_surface_interface surf_impl = {
    s_destroy, s_attach, s_4i /*damage*/, s_frame, s_nop_r /*opaque*/,
    s_nop_r /*input*/, s_commit, s_i32 /*transform*/, s_i32 /*scale*/,
    s_4i /*damage_buffer*/, s_2i /*offset*/,
};
static void free_sc(struct wl_resource *r){ free(wl_resource_get_user_data(r)); }
static void comp_surface(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *s = wl_resource_create(c, &wl_surface_interface,
        wl_resource_get_version(r), id);
    struct surf_ctx *sc = calloc(1, sizeof *sc);
    wl_resource_set_implementation(s, &surf_impl, sc, free_sc);
}
static const struct wl_region_interface reg_impl = { s_destroy, s_4i, s_4i, s_4i, s_2i };
static void comp_region(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *s = wl_resource_create(c, &wl_region_interface, 1, id);
    wl_resource_set_implementation(s, &reg_impl, NULL, NULL);
}
static const struct wl_compositor_interface comp_impl = { comp_surface, comp_region };

/* ---------- xdg ---------- */
static void xdm_destroy(struct wl_client *c, struct wl_resource *r){ wl_resource_destroy(r); }
static void xdm_pong(struct wl_client *c, struct wl_resource *r, uint32_t s){ (void)c;(void)r;(void)s; }
static void pos_size(struct wl_client *c, struct wl_resource *r, int32_t w, int32_t h){ (void)c;(void)r;(void)w;(void)h; }
static const struct xdg_positioner_interface pos_impl = {
    s_destroy, pos_size, s_4i, s_u, s_u, s_u, pos_size, s_nop, pos_size, s_u,
};
static void xdm_pos(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *p = wl_resource_create(c, &xdg_positioner_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(p, &pos_impl, NULL, NULL);
}
static void s_showmenu(struct wl_client *c, struct wl_resource *r, struct wl_resource *s, uint32_t ser, int32_t x, int32_t y){ (void)c;(void)r;(void)s;(void)ser;(void)x;(void)y; }
static void s_resize3(struct wl_client *c, struct wl_resource *r, struct wl_resource *s, uint32_t ser, uint32_t e){ (void)c;(void)r;(void)s;(void)ser;(void)e; }
static const struct xdg_toplevel_interface tl_impl = {
    s_destroy, s_nop_r /*set_parent*/, s_nop_s /*set_title*/, s_nop_s /*set_app_id*/,
    s_showmenu /*show_window_menu*/, s_seat_ser /*move*/, s_resize3 /*resize*/,
    s_2i /*set_max_size*/, s_2i /*set_min_size*/, s_nop /*set_maximized*/,
    s_nop /*unset_maximized*/, s_nop_r /*set_fullscreen*/, s_nop /*unset_fullscreen*/,
    s_nop /*set_minimized*/,
};
static void xs_destroy(struct wl_client *c, struct wl_resource *r){ wl_resource_destroy(r); }
static void xs_get_top(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *wl_s = wl_resource_get_user_data(r);
    struct wl_resource *t = wl_resource_create(c, &xdg_toplevel_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(t, &tl_impl, NULL, NULL);
    struct wl_array arr; wl_array_init(&arr);
    xdg_toplevel_send_configure(t, 0, 0, &arr);
    wl_array_release(&arr);
    xdg_surface_send_configure(r, sc_ctr++);
    ((struct surf_ctx*)wl_resource_get_user_data(wl_s))->is_top = 1;
}
static void xs_get_popup(struct wl_client *c, struct wl_resource *r, uint32_t id,
    struct wl_resource *p, struct wl_resource *po){ (void)c;(void)r;(void)id;(void)p;(void)po; }
static void xs_ack(struct wl_client *c, struct wl_resource *r, uint32_t s){ (void)c;(void)r;(void)s; }
static const struct xdg_surface_interface xs_impl = { xs_destroy, xs_get_top, xs_get_popup, s_4i, xs_ack };
static void xdm_get_xdg(struct wl_client *c, struct wl_resource *r, uint32_t id, struct wl_resource *s){
    struct wl_resource *x = wl_resource_create(c, &xdg_surface_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(x, &xs_impl, s, NULL);
    wl_resource_set_user_data(x, s);
}
static const struct xdg_wm_base_interface xdm_impl = { xdm_destroy, xdm_pos, xdm_get_xdg, xdm_pong };

static void s_commit(struct wl_client *c, struct wl_resource *r){
    (void)c;
    struct surf_ctx *sc = wl_resource_get_user_data(r);
    if (sc && sc->is_top && !g_top_wl_surf) {
        g_top_wl_surf = r;
        fprintf(stderr, "[wltest] toplevel mapped\n");
        send_enter();
    }
    g_commits++;
}

/* ---------- seat / keyboard ---------- */
static const struct wl_keyboard_interface kb_impl = { s_destroy /*release*/ };
static void send_enter(void){
    if (!g_kbd || !g_top_wl_surf) return;
    struct wl_array arr; wl_array_init(&arr);
    wl_keyboard_send_enter(g_kbd, sc_ctr++, g_top_wl_surf, &arr);
    wl_array_release(&arr);
}
static void seat_get_kb(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *k = wl_resource_create(c, &wl_keyboard_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(k, &kb_impl, NULL, NULL);
    g_kbd = k;
    struct xkb_keymap *km = xkb_keymap_new_from_names(xkbctx, NULL, XKB_KEYMAP_COMPILE_NO_FLAGS);
    char *ks = xkb_keymap_get_as_string(km, XKB_KEYMAP_FORMAT_TEXT_V1);
    size_t len = strlen(ks);
    int fd = memfd_create("keymap", MFD_CLOEXEC);
    ssize_t off = 0;
    while (off < (ssize_t)len) {
        ssize_t w = write(fd, ks + off, len - off);
        if (w <= 0) break;
        off += w;
    }
    wl_keyboard_send_keymap(k, WL_KEYBOARD_KEYMAP_FORMAT_XKB_V1, fd, len);
    close(fd);
    free(ks);
    xkb_keymap_unref(km);
    wl_keyboard_send_modifiers(k, sc_ctr++, 0, 0, 0, 0);
    send_enter();
}
static void ptr_set(struct wl_client *c, struct wl_resource *r, uint32_t s, struct wl_resource *sf, int32_t x, int32_t y){ (void)c;(void)r;(void)s;(void)sf;(void)x;(void)y; }
static const struct wl_pointer_interface ptr_impl = { ptr_set, s_destroy };
static void seat_get_ptr(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *p = wl_resource_create(c, &wl_pointer_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(p, &ptr_impl, NULL, NULL);
}
static void seat_get_touch(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *p = wl_resource_create(c, &wl_touch_interface,
        wl_resource_get_version(r), id);
    static const struct wl_touch_interface timpl = { s_destroy };
    wl_resource_set_implementation(p, &timpl, NULL, NULL);
}
static const struct wl_seat_interface seat_impl = { seat_get_ptr, seat_get_kb, seat_get_touch, s_destroy };

/* ---------- linux_dmabuf ---------- */
static void buf_destroy(struct wl_client *c, struct wl_resource *r){ wl_resource_destroy(r); }
static const struct wl_buffer_interface buf_impl = { buf_destroy };
static void dp_add(struct wl_client *c, struct wl_resource *r, int32_t fd, uint32_t pi,
    uint32_t off, uint32_t st, uint32_t mhi, uint32_t mlo){
    (void)c;(void)r;(void)pi;(void)off;(void)st;(void)mhi;(void)mlo;
    close(fd);
}
static void dp_create(struct wl_client *c, struct wl_resource *r,
    int32_t w, int32_t h, uint32_t fmt, uint32_t flags){
    (void)c;(void)r;(void)w;(void)h;(void)fmt;(void)flags;
    g_buffers++;
}
static void dp_create_immed(struct wl_client *c, struct wl_resource *r, uint32_t id,
    int32_t w, int32_t h, uint32_t fmt, uint32_t flags){
    (void)c;(void)r;(void)w;(void)h;(void)fmt;(void)flags;
    struct wl_resource *b = wl_resource_create(c, &wl_buffer_interface, 1, id);
    wl_resource_set_implementation(b, &buf_impl, NULL, NULL);
    g_buffers++;
}
static const struct zwp_linux_buffer_params_v1_interface dp_impl = {
    s_destroy, dp_add, dp_create, dp_create_immed,
};
static void dmb_params(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *p = wl_resource_create(c, &zwp_linux_buffer_params_v1_interface,
        wl_resource_get_version(r), id);
    wl_resource_set_implementation(p, &dp_impl, NULL, NULL);
}
static void dmb_fb(struct wl_client *c, struct wl_resource *r, uint32_t id){
    struct wl_resource *p = wl_resource_create(c, &zwp_linux_dmabuf_feedback_v1_interface,
        wl_resource_get_version(r), id);
    static const struct zwp_linux_dmabuf_feedback_v1_interface fimpl = { s_destroy };
    wl_resource_set_implementation(p, &fimpl, NULL, NULL);
}
static void dmb_sfb(struct wl_client *c, struct wl_resource *r, uint32_t id, struct wl_resource *sf){
    (void)sf; dmb_fb(c, r, id);
}
static const struct zwp_linux_dmabuf_v1_interface dmb_impl = { s_destroy, dmb_params, dmb_fb, dmb_sfb };

/* ---------- bind ---------- */
static void bind(struct wl_client *c, void *ud, uint32_t ver, uint32_t name){
    uintptr_t t = (uintptr_t)ud;
    if (t == 1) {
        struct wl_resource *r = wl_resource_create(c, &wl_compositor_interface, ver > 4 ? 4 : ver, name);
        wl_resource_set_implementation(r, &comp_impl, NULL, NULL);
    } else if (t == 2) {
        struct wl_resource *r = wl_resource_create(c, &xdg_wm_base_interface, ver > 3 ? 3 : ver, name);
        wl_resource_set_implementation(r, &xdm_impl, NULL, NULL);
    } else if (t == 3) {
        struct wl_resource *r = wl_resource_create(c, &wl_seat_interface, ver > 4 ? 4 : ver, name);
        wl_resource_set_implementation(r, &seat_impl, NULL, NULL);
        wl_seat_send_capabilities(r, WL_SEAT_CAPABILITY_KEYBOARD);
    } else if (t == 4) {
        struct wl_resource *r = wl_resource_create(c, &zwp_linux_dmabuf_v1_interface, ver > 4 ? 4 : ver, name);
        wl_resource_set_implementation(r, &dmb_impl, NULL, NULL);
        if (wl_resource_get_version(r) >= ZWP_LINUX_DMABUF_V1_MODIFIER_SINCE_VERSION) {
            zwp_linux_dmabuf_v1_send_modifier(r, 0x34324241, 0, 0);
            zwp_linux_dmabuf_v1_send_modifier(r, 0x34324241, 0, 1);
            zwp_linux_dmabuf_v1_send_modifier(r, 0x34325258, 0, 0);
            zwp_linux_dmabuf_v1_send_modifier(r, 0x34325258, 0, 1);
        } else {
            zwp_linux_dmabuf_v1_send_format(r, 0x34324241);
            zwp_linux_dmabuf_v1_send_format(r, 0x34325258);
        }
    }
}

int main(int argc, char **argv){
    const char *sock = argc > 1 ? argv[1] : "wayland-test";
    xkbctx = xkb_context_new(XKB_CONTEXT_NO_FLAGS);
    disp = wl_display_create();
    if (wl_display_add_socket_auto(disp) < 0) {
        fprintf(stderr, "[wltest] cannot create socket\n");
        return 1;
    }
    fprintf(stderr, "[wltest] listening on %s\n", getenv("WAYLAND_DISPLAY"));
    wl_display_add_global(disp, &wl_compositor_interface, (void*)(uintptr_t)1, bind);
    wl_display_add_global(disp, &xdg_wm_base_interface, (void*)(uintptr_t)2, bind);
    wl_display_add_global(disp, &wl_seat_interface, (void*)(uintptr_t)3, bind);
    wl_display_add_global(disp, &zwp_linux_dmabuf_v1_interface, (void*)(uintptr_t)4, bind);
    wl_display_run(disp);
    fprintf(stderr, "[wltest] done commits=%d buffers=%d keys=%d\n", g_commits, g_buffers, g_keyev);
    wl_display_destroy_clients(disp);
    wl_display_destroy(disp);
    return 0;
}
