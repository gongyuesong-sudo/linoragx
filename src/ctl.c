/* ctl.c — localhost TCP control channel for MCP integration.
 * Enabled by env LINORAGX_CTL=<port>. Line protocol, one command per \n,
 * one JSON line per command. Never blocks the emu loop (non-blocking). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "linoragx.h"
#include "m68k.h"

#define CTL_MAX_PRESS 16
typedef struct { int p, bit; long long release_at; } ctl_press;

static int lfd = -1, cfd = -1;
static int ctl_paused = 0, ctl_quit = 0;
static long long ctl_steps = 0;
static ctl_press presses[CTL_MAX_PRESS];
static int npress = 0;
static char rbuf[4096];
static int rlen = 0;
static long long ctl_target = 0;   /* absolute frame at which 'frames N' fires, 0=none */

static void set_nb(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK); }

int ctl_open(int port)
{
    lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) return -1;
    int on = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons((uint16_t)port);
    if (bind(lfd, (struct sockaddr*)&a, sizeof a) < 0 || listen(lfd, 2) < 0) return -1;
    set_nb(lfd);
    fprintf(stderr, "[ctl] listening on 127.0.0.1:%d\n", port);
    return 0;
}

int ctl_active(void) { return lfd >= 0; }
int ctl_want_quit(void) { return ctl_quit; }
int ctl_is_paused(void) { return ctl_paused; }

static void send_line(const char *s)
{
    if (cfd < 0) return;
    char out[4096];
    int n = snprintf(out, sizeof out, "%s\n", s);
    ssize_t off = 0;
    while (off < n) { ssize_t w = send(cfd, out+off, n-off, MSG_NOSIGNAL); if (w <= 0) break; off += w; }
}

static void write_u24(unsigned char *p, unsigned v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; }
static void write_u32(unsigned char *p, unsigned v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

/* 24-bit BMP writer from RGBA8888 fb (bottom-up rows) */
int dump_bmp(const uint32_t *fb, const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    int w = NG_W, h = NG_H, rowsz = (w*3+3)/4*4;
    unsigned img = rowsz*h;
    unsigned char hdr[54]; memset(hdr, 0, 54);
    hdr[0]='B'; hdr[1]='M';
    write_u32(hdr+2, 54+img); write_u32(hdr+10, 54);
    write_u32(hdr+14, 40);
    *(int*)(hdr+18)=w; *(int*)(hdr+22)=h;
    hdr[26]=1; hdr[28]=24;
    write_u32(hdr+34, img);
    fwrite(hdr, 1, 54, f);
    unsigned char *row = malloc(rowsz);
    for (int y = h-1; y >= 0; y--) {
        for (int x = 0; x < w; x++) {
            uint32_t px = fb[y*w+x];
            row[x*3+2] = px & 0xFF;          /* R (stored BGR) */
            row[x*3+1] = (px>>8) & 0xFF;     /* G */
            row[x*3+0] = (px>>16) & 0xFF;    /* B */
        }
        fwrite(row, 1, rowsz, f);
    }
    free(row); fclose(f);
    return 0;
}

static int bit_of(const char *s)
{
    static const char *names[] = {"up","down","left","right","b","a","c","z","l","r","start","select",NULL};
    /* Neo Geo bit order: 0 up,1 down,2 left,3 right,4 B,5 A,6 C,7 Z,8 L,9 R,10 start,11 select */
    for (int i = 0; names[i]; i++) if (!strcasecmp(s, names[i])) return i;
    return atoi(s);
}

static void handle_cmd(neogeo *ng, const uint32_t *fb, char *cmd)
{
    char resp[2048];
    char op[64] = "";
    sscanf(cmd, "%63s", op);
    long long fr = ng_frame_no(ng);
    if (!strcmp(op, "state")) {
        snprintf(resp, sizeof resp,
            "{\"frame\":%lld,\"pc\":\"0x%08x\",\"paused\":%d,\"game\":\"%s\",\"steps\":%lld}",
            fr, m68k_get_reg(NULL, M68K_REG_PC), ctl_paused, ng_game_name(ng), ctl_steps);
    } else if (!strcmp(op, "pause")) {
        ctl_paused = 1; snprintf(resp, sizeof resp, "{\"ok\":true,\"paused\":true}");
    } else if (!strcmp(op, "resume")) {
        ctl_paused = 0; ctl_steps = 0; snprintf(resp, sizeof resp, "{\"ok\":true,\"paused\":false}");
    } else if (!strcmp(op, "step")) {
        long long n = 1; sscanf(cmd, "%*s %lld", &n);
        ctl_paused = 1; ctl_steps = n;
        snprintf(resp, sizeof resp, "{\"ok\":true,\"stepping\":%lld}", n);
    } else if (!strcmp(op, "press")) {
        int p = 0, hold = 3; char b[16] = "start";
        sscanf(cmd, "%*s %d %15s %d", &p, b, &hold);
        int bit = bit_of(b);
        ng_set_button(ng, p, bit, 1);
        if (npress < CTL_MAX_PRESS) presses[npress++] = (ctl_press){p, bit, fr + hold};
        snprintf(resp, sizeof resp, "{\"ok\":true,\"press\":{\"p\":%d,\"bit\":%d,\"hold\":%d}}", p, bit, hold);
    } else if (!strcmp(op, "btn")) {
        int p = 0, mask = 0;
        sscanf(cmd, "%*s %d %i", &p, &mask);
        for (int b = 0; b < 12; b++) ng_set_button(ng, p, b, (mask>>b)&1);
        snprintf(resp, sizeof resp, "{\"ok\":true,\"mask\":%d}", mask);
    } else if (!strcmp(op, "release")) {
        int p = 0, mask = 0;
        sscanf(cmd, "%*s %d %i", &p, &mask);
        for (int b = 0; b < 12; b++) if ((mask>>b)&1) ng_set_button(ng, p, b, 0);
        snprintf(resp, sizeof resp, "{\"ok\":true}");
    } else if (!strcmp(op, "dump")) {
        char path[512] = "/tmp/linoragx_ctl.bmp";
        sscanf(cmd, "%*s %511s", path);
        int r = fb ? dump_bmp(fb, path) : -1;
        snprintf(resp, sizeof resp, "{\"ok\":%s,\"path\":\"%s\"}", r==0?"true":"false", path);
    } else if (!strcmp(op, "save")) {
        char path[512] = "/tmp/linoragx.rom";
        sscanf(cmd, "%*s %511s", path);
        int r = ng_save_state(ng, path);
        snprintf(resp, sizeof resp, "{\"ok\":%s,\"code\":%d,\"path\":\"%s\"}", r==0?"true":"false", r, path);
    } else if (!strcmp(op, "load")) {
        char path[512] = "";
        sscanf(cmd, "%*s %511s", path);
        int r = ng_load_state(ng, path);
        snprintf(resp, sizeof resp, "{\"ok\":%s,\"code\":%d}", r==0?"true":"false", r);
    } else if (!strcmp(op, "sprdump")) {
        unsigned t0 = 0; int cnt = 1; char path[512] = "/tmp/spr.raw";
        sscanf(cmd, "%*s %x %d %511s", &t0, &cnt, path);
        if (cnt > 4096) cnt = 4096;
        uint32_t mask = 0; uint8_t *s8 = ng_sprite8(ng, &mask);
        FILE *f = fopen(path, "wb");
        int ok = f != NULL;
        for (int i = 0; ok && i < cnt; i++) {
            uint32_t off = ((uint32_t)(t0 + i) * 0x100u) & mask;
            fwrite(s8 + off, 1, 256, f);
        }
        if (f) fclose(f);
        snprintf(resp, sizeof resp, "{\"ok\":%s,\"tiles\":%d,\"path\":\"%s\"}", ok?"true":"false", cnt, path);
    } else if (!strcmp(op, "gfxpeek")) {
        unsigned off = 0; int n = 16;
        sscanf(cmd, "%*s %x %d", &off, &n);
        if (n > 64) n = 64;
        uint32_t mask = 0; uint8_t *s8 = ng_sprite8(ng, &mask);
        char hex[64*3+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+3*i, "%02x ", s8 ? s8[(off+i)&mask] : 0xEE);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"hex\":\"%s\",\"mask\":\"0x%x\"}", off, hex, mask);
    } else if (!strcmp(op, "zoompeek")) {
        unsigned off = 0; int n = 16;
        sscanf(cmd, "%*s %x %d", &off, &n);
        if (n > 64) n = 64;
        uint8_t *zy = ng_zoomy(ng);
        char hex[64*3+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+3*i, "%02x ", zy ? zy[(off+i)&0xFFFF] : 0xEE);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"hex\":\"%s\"}", off, hex);
    } else if (!strcmp(op, "sprdraw")) {
        /* replicate ng_render's per-scanline sprite walk at a given line */
        unsigned line = 0;
        sscanf(cmd, "%*s %x", &line);
        uint16_t *vram = ng_vram(ng);
        uint16_t *cram = ng_cram(ng);
        uint32_t bank = ng_cram_bank_render(ng);
        uint32_t spr_mask; uint8_t *spr8 = ng_sprite8(ng, &spr_mask);
        uint8_t *zoomy = ng_zoomy(ng);
        int ac = ng_auto_counter(ng);
        uint16_t *list = &vram[(line & 1) ? 0x8680 : 0x8600];
        int maxsi = 95;
        while (maxsi >= 0 && list[maxsi] == 0) maxsi--;
        if (maxsi != 95) maxsi++;
        int x = 0, zoom_x = 0, y = 0, rows = 0, zoom_y = 0;
        char out[2048] = "";
        int shown = 0;
        for (int si = 0; si <= maxsi && shown < 6; si++) {
            int n = list[si] & 0x1FF;
            uint16_t yc = vram[0x8200 | n], zc = vram[0x8000 | n];
            if (yc & 0x40) { x = (x + zoom_x + 1) & 0x1FF; zoom_x = (zc >> 8) & 0x0F; }
            else { y = 0x200 - (yc >> 7); x = vram[0x8400 | n] >> 7; zoom_y = zc & 0xFF; zoom_x = (zc >> 8) & 0x0F; rows = yc & 0x3F; }
            if (x >= 0x140 && x <= 0x1F0) continue;
            if (!(rows == 0 || rows >= 0x20 || ((int)(line - y) & 0x1FF) < rows * 16)) continue;
            int sprite_line = ((int)line - y) & 0x1FF;
            int zoom_line = sprite_line & 0xFF; int invert = (sprite_line >> 8) & 1;
            if (invert) zoom_line ^= 0xFF;
            if (rows > 0x20) {
                zoom_line = zoom_line % ((zoom_y + 1) << 1);
                if (zoom_line > zoom_y) { zoom_line = ((zoom_y + 1) << 1) - 1 - zoom_line; invert = !invert; }
            }
            uint8_t syt = zoomy[(zoom_y << 8) | zoom_line];
            int sprite_y = syt & 0x0F; int tile = syt >> 4;
            if (invert) { sprite_y ^= 0x0F; tile ^= 0x1F; }
            int ao = (n << 6) | (tile << 1);
            uint16_t attr = vram[ao + 1];
            uint32_t code = (((uint32_t)attr << 12) & 0xF0000) | vram[ao];
            if (attr & 0x08) code = (code & ~7u) | (ac & 7);
            else if (attr & 0x04) code = (code & ~3u) | (ac & 3);
            if (attr & 0x02) sprite_y ^= 0x0F;
            uint32_t gfx = ((code << 8) | (sprite_y << 4)) & spr_mask;
            const uint16_t *pens = &cram[((attr >> 8) << 4) | bank];
            int off = snprintf(out + strlen(out), sizeof(out) - strlen(out),
                "si=%d n=%d y=%x x=%x rows=%x zy=%x zx=%x sl=%d zl=%d inv=%d syt=%02x tile=%x sy=%d code=%x attr=%x gfx=%x p0=%x p1=%x p2=%x\n",
                si, n, y, x, rows, zoom_y, zoom_x, sprite_line, zoom_line, invert, syt, tile, sprite_y,
                (unsigned)code, attr, (unsigned)gfx, pens[0], pens[1], pens[2]);
            if (off < 0) break;
            shown++;
        }
        snprintf(resp, sizeof resp, "{\"line\":%u,\"maxsi\":%d,\"walk\":\"%s\"}", line, maxsi, out);
    } else if (!strcmp(op, "gfxdec")) {
        /* dump raw CROM bytes at gfx offset + our predecoded bytes, plus a
           manual 4bpp decode of the first 8 pixels for cross-check */
        unsigned off = 0;
        sscanf(cmd, "%*s %x", &off);
        uint32_t mask = 0; uint8_t *s8 = ng_sprite8(ng, &mask);
        uint32_t rsz = 0; uint8_t *raw = ng_spr(ng, &rsz);
        uint32_t ro = (off & mask) >> 1;
        char hex[64*3+1] = "", dec[64] = "";
        for (int i = 0; i < 16; i++) sprintf(hex+3*i, "%02x ", (raw && ro+i < rsz) ? raw[ro+i] : 0xEE);
        for (int i = 0; i < 8; i++) {
            uint8_t b = (raw && ro+i < rsz) ? raw[ro+i] : 0;
            sprintf(dec+2*i, "%x%x", b>>4, b&0xF);
        }
        char h2[64*3+1] = "";
        for (int i = 0; i < 16; i++) sprintf(h2+3*i, "%02x ", s8 ? s8[(off+i)&mask] : 0xEE);
        snprintf(resp, sizeof resp, "{\"gfx\":\"0x%x\",\"raw\":\"%s\",\"dec\":\"%s\",\"pre\":[%s]}",
            off, hex, dec, h2);
    } else if (!strcmp(op, "sprstats")) {
        uint32_t sz = 0; uint8_t *sp = ng_spr(ng, &sz);
        long nz = 0; uint32_t first = 0xFFFFFFFF;
        for (uint32_t i = 0; i < sz; i++) if (sp[i]) { nz++; if (first == 0xFFFFFFFF) first = i; }
        snprintf(resp, sizeof resp, "{\"size\":%u,\"nonzero\":%ld,\"first_nonzero\":\"0x%x\"}", sz, nz, first);
    } else if (!strcmp(op, "sprpeek")) {
        unsigned a = 0; int n = 16;
        sscanf(cmd, "%*s %x %d", &a, &n);
        if (n > 64) n = 64;
        uint32_t sz = 0; uint8_t *sp = ng_spr(ng, &sz);
        char hex[64*2+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+2*i, "%02x", a+i < sz ? sp[a+i] : 0xEE);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"hex\":\"%s\"}", a, hex);
    } else if (!strcmp(op, "crampeek")) {
        unsigned a = 0; int n = 8;
        sscanf(cmd, "%*s %x %d", &a, &n);
        if (n > 32) n = 32;
        uint16_t *cr = ng_cram(ng);
        char hex[32*5+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+5*i, "%04x ", cr[(a+i)&0x1FFF]);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"words\":\"%s\",\"bank\":\"0x%x\"}", a, hex, ng_cram_bank(ng));
    } else if (!strcmp(op, "regs")) {
        snprintf(resp, sizeof resp, "{\"sr\":\"0x%04x\",\"vbr\":\"0x%08x\",\"usp\":\"0x%08x\"}",
            m68k_get_reg(NULL, M68K_REG_SR), m68k_get_reg(NULL, M68K_REG_VBR),
            m68k_get_reg(NULL, M68K_REG_USP));
    } else if (!strcmp(op, "vdump")) {
        /* whole vram+cram+zoomy to a binary file (big-endian), reply path */
        uint16_t *vr = ng_vram(ng);
        uint16_t *cr = ng_cram(ng);
        uint8_t *zy = ng_zoomy(ng);
        const char *path = "/tmp/linoragx_vdump.bin";
        FILE *f = fopen(path, "wb");
        if (!f) { snprintf(resp, sizeof resp, "{\"err\":\"fopen\"}"); }
        else {
            for (int i = 0; i < 0x8800; i++) { uint8_t b[2] = {(uint8_t)(vr[i]>>8), (uint8_t)vr[i]}; fwrite(b,1,2,f); }
            for (int i = 0; i < 0x1000; i++) { uint8_t b[2] = {(uint8_t)(cr[i]>>8), (uint8_t)cr[i]}; fwrite(b,1,2,f); }
            fwrite(zy, 1, 0x10000, f);
            fclose(f);
            snprintf(resp, sizeof resp, "{\"path\":\"%s\",\"size\":%d}", path, 0x8800*2 + 0x1000*2 + 0x10000);
        }
    } else if (!strcmp(op, "vpeek")) {
        unsigned a = 0; int n = 8;
        sscanf(cmd, "%*s %x %d", &a, &n);
        if (n > 32) n = 32;
        uint16_t *vr = ng_vram(ng);
        char hex[32*5+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+5*i, "%04x ", a+i < 0x8800 ? vr[a+i] : 0xFFFF);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"words\":\"%s\"}", a, hex);
    } else if (!strcmp(op, "peek")) {
        unsigned a = 0; int n = 4;
        sscanf(cmd, "%*s %x %d", &a, &n);
        if (n > 64) n = 64;
        char hex[64*2+1] = "";
        for (int i = 0; i < n; i++) sprintf(hex+2*i, "%02x", m68k_read_memory_8(a+i) & 0xFF);
        snprintf(resp, sizeof resp, "{\"addr\":\"0x%x\",\"hex\":\"%s\"}", a, hex);
    } else if (!strcmp(op, "poke")) {
        unsigned a = 0, v = 0;
        sscanf(cmd, "%*s %x %x", &a, &v);
        m68k_write_memory_8(a, v);
        snprintf(resp, sizeof resp, "{\"ok\":true}");
    } else if (!strcmp(op, "frames")) {
        long long t = 0; sscanf(cmd, "%*s %lld", &t);
        ctl_target = fr + t;
        snprintf(resp, sizeof resp, "{\"ok\":true,\"target\":%lld}", ctl_target);
    } else if (!strcmp(op, "quit")) {
        ctl_quit = 1;
        snprintf(resp, sizeof resp, "{\"ok\":true,\"bye\":true}");
    } else {
        snprintf(resp, sizeof resp, "{\"error\":\"unknown cmd: %s\"}", op);
    }
    send_line(resp);
}

/* called once per frame from main loop.
 * returns bit0=quit requested, bit1=skip this frame (paused, no step budget) */
int ctl_poll(neogeo *ng, const uint32_t *fb)
{
    int ret = 0;
    if (lfd < 0) return 0;
    if (cfd < 0) {
        int fd = accept(lfd, NULL, NULL);
        if (fd >= 0) { cfd = fd; set_nb(fd); rlen = 0; }
    }
    if (cfd >= 0) {
        for (;;) {
            char tmp[1024];
            ssize_t n = recv(cfd, tmp, sizeof tmp, 0);
            if (n > 0) {
                for (ssize_t i = 0; i < n; i++) {
                    if (tmp[i] == '\n') {
                        rbuf[rlen] = 0;
                        if (rlen) handle_cmd(ng, fb, rbuf);
                        rlen = 0;
                    } else if (rlen < (int)sizeof rbuf - 1) {
                        rbuf[rlen++] = tmp[i];
                    }
                }
            } else if (n == 0) {
                close(cfd); cfd = -1; rlen = 0; break;
            } else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) { close(cfd); cfd = -1; rlen = 0; }
                break;
            }
        }
    }
    /* auto-release held presses */
    long long fr = ng_frame_no(ng);
    for (int i = 0; i < npress; ) {
        if (fr >= presses[i].release_at) {
            ng_set_button(ng, presses[i].p, presses[i].bit, 0);
            presses[i] = presses[--npress];
        } else i++;
    }
    if (ctl_quit) ret |= 1;
    if (ctl_paused) {
        if (ctl_steps > 0) ctl_steps--;
        else ret |= 2;
    }
    if (ctl_target && fr >= ctl_target) ret |= 1;
    return ret;
}
