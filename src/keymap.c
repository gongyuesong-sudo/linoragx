/* keymap.c — configurable key bindings with per-game override.
 *
 * Three layers, later wins:
 *   1. built-in defaults (arrows/ZXCVBN P1, QWER P2, space start, 5/6 coins)
 *   2. [default] section of keys.ini          — user global defaults
 *   3. [<gameid>] section of keys.ini         — per-game overrides
 * A game section that names an action REPLACES that action's bindings;
 * actions it does not name inherit the global default.
 *
 * keys.ini format (ASCII keysyms, ':' separated, case-insensitive):
 *   [default]
 *   p1a:j,k
 *   p1start:space,return
 *   [kof2000]
 *   p1a:u
 * Key names: single printable char ('z','5'), named keys below, or 0xNNNN.
 * "none" unbinds an action.
 */
#include "linoragx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>

struct kb { uint32_t sym; int p, bit; };
#define KB_MAX 512
static struct kb g_kb[KB_MAX];
static int g_nkb;

/* action name -> (player, bit); bits: 0UP 1DN 2LF 3RT 4C 5D 6A 7B 8start 9select 10coin */
static const struct { const char *name; int p, bit; } g_act[] = {
    {"p1up",0,0},{"p1dn",0,1},{"p1lf",0,2},{"p1rt",0,3},
    {"p1a",0,6},{"p1b",0,7},{"p1c",0,4},{"p1d",0,5},
    {"p1select",0,9},{"p1start",0,8},{"p1coin",0,10},
    {"p2up",1,0},{"p2dn",1,1},{"p2lf",1,2},{"p2rt",1,3},
    {"p2a",1,6},{"p2b",1,7},{"p2c",1,4},{"p2d",1,5},
    {"p2select",1,9},{"p2start",1,8},{"p2coin",1,10},
};
#define NACT ((int)(sizeof g_act / sizeof g_act[0]))

/* named keysyms (X11 keysym values) */
static const struct { const char *name; uint32_t sym; } g_named[] = {
    {"up",0xff52},{"down",0xff54},{"left",0xff51},{"right",0xff53},
    {"space",' '},{"return",0xff0d},{"enter",0xff0d},{"kpenter",0xff8d},
    {"tab",0xff09},{"esc",0xff1b},{"insert",0xff63},{"delete",0xffff},
    {"home",0xff50},{"end",0xff57},{"pgup",0xff55},{"pgdn",0xff56},
    {"f1",0xffbe},{"f2",0xffbf},{"f3",0xffc0},{"f4",0xffc1},{"f5",0xffc2},
    {"f6",0xffc3},{"f7",0xffc4},{"f8",0xffc5},{"f9",0xffc6},{"f10",0xffc7},
    {"f11",0xffc8},{"f12",0xffc9},
    {"kp0",0xffb0},{"kp1",0xffb1},{"kp2",0xffb2},{"kp3",0xffb3},{"kp4",0xffb4},
    {"kp5",0xffb5},{"kp6",0xffb6},{"kp7",0xffb7},{"kp8",0xffb8},{"kp9",0xffb9},
    {"kpadd",0xffab},{"kpsub",0xffad},{"kpmul",0xffaa},{"kpdiv",0xffaf},
    {"shift",0xffe1},{"ctrl",0xffe3},{"alt",0xffe9},
};

static int key_parse(const char *s, uint32_t *out)
{
    while (*s == ' ') s++;
    if (!*s) return 0;
    if (!strcasecmp(s, "none")) { *out = 0; return 2; }          /* unbind */
    if (!strncmp(s, "0x", 2) || !strncmp(s, "0X", 2)) { *out = (uint32_t)strtoul(s, 0, 16); return 1; }
    for (unsigned i = 0; i < sizeof g_named/sizeof g_named[0]; i++)
        if (!strcasecmp(s, g_named[i].name)) { *out = g_named[i].sym; return 1; }
    char *e; long v = strtol(s, &e, 10);                          /* bare decimal keysym (2+ digits) */
    if (*s && *e == 0 && strlen(s) >= 2 && v > 0 && v <= 0x10FFFF) { *out = (uint32_t)v; return 1; }
    if (!s[1]) { *out = (uint32_t)(unsigned char)s[0];            /* single printable char */
        if (*out >= 'A' && *out <= 'Z') *out += 32;               /* lowercase, matches X11 path */
        return 1; }
    return 0;
}

/* remove all bindings of (p,bit) that came from layer >= `from` */
static int g_owner[KB_MAX];   /* 0=builtin 1=default-section 2=game-section */
static void act_clear(int p, int bit, int min_owner)
{
    int w = 0;
    for (int i = 0; i < g_nkb; i++)
        if (!(g_kb[i].p == p && g_kb[i].bit == bit && g_owner[i] >= min_owner)) {
            g_kb[w] = g_kb[i]; g_owner[w] = g_owner[i]; w++;
        }
    g_nkb = w;
}

static void act_bind(int p, int bit, uint32_t sym, int owner)
{
    if (g_nkb < KB_MAX) { g_kb[g_nkb] = (struct kb){sym, p, bit}; g_owner[g_nkb] = owner; g_nkb++; }
}

static void act_apply(int ai, const char *vallist, int owner)
{
    int p = g_act[ai].p, bit = g_act[ai].bit;
    act_clear(p, bit, 0);                         /* naming an action replaces its prior bindings */
    char buf[256]; snprintf(buf, sizeof buf, "%s", vallist);
    char *tok = buf;
    while (tok) {
        char *comma = strchr(tok, ',');
        if (comma) *comma++ = 0;
        uint32_t sym; int r = key_parse(tok, &sym);
        if (r == 1) act_bind(p, bit, sym, owner);
        else if (r == 0) fprintf(stderr, "[keys] unknown key '%s' for %s\n", tok, g_act[ai].name);
        tok = comma;
    }
}

void keymap_init_default(void)
{
    /* built-in defaults (owner 0): arrows/ZXCVBN P1, kp8246/QWER P2 */
    static const struct { const char *act; const char *keys; } defs[] = {
        {"p1up","up"},{"p1dn","down"},{"p1lf","left"},{"p1rt","right"},
        {"p1a","z"},{"p1b","x"},{"p1c","c"},{"p1d","v"},{"p1select","b"},
        {"p1start","space,return"},{"p1coin","5"},
        {"p2up","kp8"},{"p2dn","kp2"},{"p2lf","kp4"},{"p2rt","kp6"},
        {"p2a","q"},{"p2b","w"},{"p2c","e"},{"p2d","r"},{"p2coin","6"},
    };
    for (unsigned i = 0; i < sizeof defs/sizeof defs[0]; i++) {
        int ai = -1;
        for (int j = 0; j < NACT; j++) if (!strcmp(g_act[j].name, defs[i].act)) { ai = j; break; }
        if (ai >= 0) act_apply(ai, defs[i].keys, 0);
    }
}

int keymap_load_file(const char *path, const char *gameid)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[512];
    /* two passes so [default] always applies before [gameid], whatever the file order */
    for (int pass = 0; pass < 2; pass++) {
        int section = -1;                     /* 0 = [default], 2 = [gameid] */
        rewind(f);
        while (fgets(line, sizeof line, f)) {
            char *s = line;
            while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
            char *e = s + strlen(s);
            while (e > s && (e[-1]==' '||e[-1]=='\t'||e[-1]=='\r'||e[-1]=='\n')) *--e = 0;
            if (!*s || *s == '#' || *s == ';' || *s == '/') continue;   /* comment/blank */
            if (*s == '[') {
                char *close = strchr(s, ']');
                if (!close) continue;
                *close = 0;
                const char *sec = s + 1;
                section = (!strcasecmp(sec, "default")) ? 0
                    : (gameid && !strcasecmp(sec, gameid)) ? 2 : -1;
                continue;
            }
            char *colon = strchr(s, ':');
            if (!colon || section != pass * 2) continue;
            *colon = 0;
            char *k = s, *v = colon + 1;
            while (*v == ' ') v++;
            int ai = -1;
            for (int j = 0; j < NACT; j++) if (!strcasecmp(g_act[j].name, k)) { ai = j; break; }
            if (ai < 0) { if (pass == 0) fprintf(stderr, "[keys] unknown action '%s'\n", k); continue; }
            act_apply(ai, v, section);
        }
    }
    fclose(f);
    return 0;
}

int keymap_lookup(uint32_t sym, int *p, int *bit)
{
    if (sym >= 'A' && sym <= 'Z') sym += 32;
    for (int i = 0; i < g_nkb; i++)
        if (g_kb[i].sym == sym) { *p = g_kb[i].p; *bit = g_kb[i].bit; return 1; }
    return 0;
}

void keymap_dump(const char *gameid)
{
    fprintf(stderr, "[keys] effective bindings (game=%s):\n", gameid ? gameid : "-");
    for (int a = 0; a < NACT; a++) {
        char line[256]; line[0] = 0;
        for (int i = 0; i < g_nkb; i++)
            if (g_kb[i].p == g_act[a].p && g_kb[i].bit == g_act[a].bit) {
                char nm[16];
                uint32_t s = g_kb[i].sym;
                const char *ext = NULL;
                for (unsigned j = 0; j < sizeof g_named/sizeof g_named[0]; j++)
                    if (g_named[j].sym == s) { ext = g_named[j].name; break; }
                if (ext) snprintf(nm, sizeof nm, "%s", ext);
                else if (s >= 0x21 && s <= 0x7e) snprintf(nm, sizeof nm, "%c", (char)s);
                else snprintf(nm, sizeof nm, "0x%x", s);
                if (line[0]) strncat(line, ",", sizeof line - strlen(line) - 1);
                strncat(line, nm, sizeof line - strlen(line) - 1);
            }
        fprintf(stderr, "  %-9s = %s\n", g_act[a].name, line[0] ? line : "(none)");
    }
}
