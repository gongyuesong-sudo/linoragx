/* neorage.ini-compatible config subset */
#include "linoragx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

void cfg_default(nconfig *c, const char *biosdir)
{
    snprintf(c->rompath, sizeof c->rompath, "%s", biosdir);
    c->gamezip[0] = 0;
    c->scanlines = 0; c->vsync = 1; c->audio = 1; c->scale = 2;
}

static char *trim(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

int cfg_load(nconfig *c, const char *inipath)
{
    FILE *f = fopen(inipath, "r");
    if (!f) return -1;
    char line[1024];
    while (fgets(line, sizeof line, f)) {
        char *s = trim(line);
        if (*s == '/' || *s == '[' || !*s) continue;
        char *colon = strchr(s, ':');
        if (!colon) continue;
        *colon = 0;
        char *k = trim(s), *v = trim(colon+1);
        if (!strcasecmp(k, "ROMS"))      snprintf(c->rompath, sizeof c->rompath, "%s", v);
        else if (!strcasecmp(k, "GAME")) snprintf(c->gamezip, sizeof c->gamezip, "%s", v);
        else if (!strcasecmp(k, "SCANLINES")) c->scanlines = !strcasecmp(v,"YES")?1:!strcasecmp(v,"50%")?2:0;
        else if (!strcasecmp(k, "SOUND")) c->audio = !strcasecmp(v, "YES");
    }
    fclose(f);
    /* D:\ style windows path -> ignore */
    if (c->rompath[1] == ':' || c->rompath[0] == '\\') c->rompath[0] = 0;
    return 0;
}

int cfg_save(nconfig *c, const char *inipath)
{
    FILE *f = fopen(inipath, "w");
    if (!f) return -1;
    fprintf(f, "/*PATHS*/\r\nROMS:%s\r\nGAME:%s\r\n\r\n", c->rompath, c->gamezip);
    fprintf(f, "/*VIDEO*/\r\nSCANLINES:%s\r\nSCALE:%d\r\nVSYNC:%s\r\n\r\n",
            c->scanlines==2?"50%":c->scanlines?"YES":"NO", c->scale, c->vsync?"YES":"NO");
    fprintf(f, "/*AUDIO*/\r\nSOUND:%s\r\n", c->audio?"YES":"NO");
    fclose(f);
    return 0;
}
