/* ROM loading: plain files + minimal ZIP reader (stored/deflate via zlib). */
#include "linoragx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

int rom_load_file(const char *path, uint8_t **out, uint32_t *outsz)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n <= 0 || n > 256*1024*1024) { fclose(f); return -2; }
    uint8_t *d = malloc(n);
    if (fread(d, 1, n, f) != (size_t)n) { free(d); fclose(f); return -3; }
    fclose(f);
    *out = d; *outsz = (uint32_t)n;
    return 0;
}

uint32_t rom_crc32(const uint8_t *d, uint32_t n)
{ return (uint32_t)crc32(0, d, n); }

/* ---- minimal zip ----
 * find EOCD, walk central dir, extract member by name (or largest if member==NULL) */
static int zip_extract(const uint8_t *z, uint32_t zs, const char *name,
                       uint8_t **out, uint32_t *outsz)
{
    /* EOCD signature 0x06054b50 */
    long i = (long)zs - 22;
    if (i < 0) return -1;
    for (; i >= 0 && (long)zs - i < 66000; i--)
        if (*(uint32_t*)(z+i) == 0x06054b50) break;
    if (i < 0) return -2;
    uint16_t nent = *(uint16_t*)(z+i+10);
    uint32_t cdofs = *(uint32_t*)(z+i+16);
    uint32_t best_sz = 0; const uint8_t *best_cdh = NULL;
    uint32_t p = cdofs;
    for (int k = 0; k < nent && p + 46 <= zs; k++) {
        if (*(uint32_t*)(z+p) != 0x02014b50) break;
        uint32_t csize = *(uint32_t*)(z+p+20);
        uint32_t usize = *(uint32_t*)(z+p+24);
        uint16_t nlen = *(uint16_t*)(z+p+28);
        uint16_t elen = *(uint16_t*)(z+p+30);
        uint16_t clen = *(uint16_t*)(z+p+32);
        uint32_t lho  = *(uint32_t*)(z+p+42);
        const char *nm = (const char*)(z+p+46);
        uint32_t nl = 0; while (nl < nlen && nm[nl]) nl++;   /* name may not be NUL-terminated */
        int match;
        if (name) {
            match = (nl == strlen(name)) && !strncasecmp(nm, name, nl);
        } else {
            char safe[128]; uint32_t cp = nl < 127 ? nl : 127;
            memcpy(safe, nm, cp); safe[cp] = 0;
            match = usize > best_sz && usize <= 8*1024*1024;
            if (strstr(safe, ".gl") || strstr(safe, ".GLY") || strstr(safe, ".txt")) match = 0;
        }
        if (match) {
            if (name || usize > best_sz) { best_sz = usize; best_cdh = z+p; (void)lho; }
        }
        p += 46 + nlen + elen + clen;
    }
    if (!best_cdh) return -3;
    uint16_t method = *(uint16_t*)(best_cdh+10);
    uint32_t csize = *(uint16_t*)(best_cdh+18) | (*(uint32_t*)(best_cdh+20)&0xFFFF0000);
    csize = *(uint32_t*)(best_cdh+20);
    uint32_t usize = *(uint32_t*)(best_cdh+24);
    uint16_t nlen = *(uint16_t*)(best_cdh+28);
    uint16_t elen = *(uint16_t*)(best_cdh+30);
    uint16_t clen = *(uint16_t*)(best_cdh+32);
    uint32_t lho  = *(uint32_t*)(best_cdh+42);
    if (lho + 30 > zs) return -4;
    uint16_t lnlen = *(uint16_t*)(z+lho+26);
    uint16_t lelen = *(uint16_t*)(z+lho+28);
    const uint8_t *data = z + lho + 30 + lnlen + lelen;
    if (data + csize > z + zs) return -5;
    uint8_t *d = malloc(usize ? usize : 1);
    if (method == 0) {
        if (usize != csize) { free(d); return -6; }
        memcpy(d, data, csize);
    } else if (method == 8) {
        z_stream s; memset(&s, 0, sizeof s);
        s.next_in = (Bytef*)data; s.avail_in = csize;
        if (inflateInit2(&s, -15) != Z_OK) { free(d); return -7; }
        s.next_out = d; s.avail_out = usize;
        int r = inflate(&s, Z_FINISH);
        inflateEnd(&s);
        if (r != Z_STREAM_END && !(r == Z_OK && s.total_out == usize)) { free(d); return -8; }
        usize = (uint32_t)s.total_out;
    } else { free(d); return -9; }
    (void)nlen;(void)elen;(void)clen;
    *out = d; *outsz = usize;
    return 0;
}

/* single-entry zip buffer cache: sw_build re-reads the same 40-90MB zip per
 * member; caching turns O(members*size) file I/O into one read. */
static char  zc_path[1024];
static uint8_t *zc_buf;
static uint32_t zc_size;
static int zip_buf(const char *zip, uint8_t **z, uint32_t *zs)
{
    if (zc_buf && !strcmp(zc_path, zip)) { *z = zc_buf; *zs = zc_size; return 0; }
    uint8_t *b; uint32_t n;
    if (rom_load_file(zip, &b, &n)) return -1;
    free(zc_buf);
    snprintf(zc_path, sizeof zc_path, "%s", zip);
    zc_buf = b; zc_size = n;
    *z = b; *zs = n;
    return 0;
}

int rom_load_zip(const char *zip, const char *member, uint8_t **out, uint32_t *outsz)
{
    uint8_t *z; uint32_t zs;
    if (zip_buf(zip, &z, &zs)) return -1;
    return zip_extract(z, zs, member, out, outsz);
}

/* extract zip member whose name ends with `suffix` (case-insensitive), e.g. "-c1.c1" or ".p1" */
int rom_load_zip_suffix(const char *zip, const char *suffix, uint8_t **out, uint32_t *outsz)
{
    uint8_t *z; uint32_t zs;
    if (zip_buf(zip, &z, &zs)) return -1;
    /* find EOCD */
    long i = (long)zs - 22; int found = -1;
    for (; i >= 0 && (long)zs - i < 66000; i--)
        if (*(uint32_t*)(z+i) == 0x06054b50) break;
    int r = -1;
    if (i >= 0) {
        uint16_t nent = *(uint16_t*)(z+i+10);
        uint32_t cdofs = *(uint32_t*)(z+i+16);
        uint32_t p = cdofs;
        size_t sl = strlen(suffix);
        for (int k = 0; k < nent && p + 46 <= zs; k++) {
            if (*(uint32_t*)(z+p) != 0x02014b50) break;
            uint16_t nlen = *(uint16_t*)(z+p+28);
            uint16_t elen = *(uint16_t*)(z+p+30);
            uint16_t clen = *(uint16_t*)(z+p+32);
            const char *nm = (const char*)(z+p+46);
            uint32_t nl = 0; while (nl < nlen && nm[nl]) nl++;
            if (nl >= sl) {
                const char *tail = nm + nl - sl;
                if (!strncasecmp(tail, suffix, sl)) {
                    char full[128]; uint32_t cp = nl < 127 ? nl : 127;
                    memcpy(full, nm, cp); full[cp] = 0;
                    r = zip_extract(z, zs, full, out, outsz);
                    if (r == 0) { found = k; break; }
                }
            }
            p += 46 + nlen + elen + clen;
        }
    }
    (void)found;
    return r;
}

/* true if zip contains a member with exactly this name (case-insensitive) */
int rom_zip_contains(const char *zip, const char *name)
{
    uint8_t *z; uint32_t zs;
    if (rom_load_file(zip, &z, &zs)) return 0;
    long i = (long)zs - 22; int found = 0;
    for (; i >= 0 && (long)zs - i < 66000; i--)
        if (*(uint32_t*)(z+i) == 0x06054b50) break;
    if (i >= 0 && *(uint32_t*)(z+i) == 0x06054b50) {
        uint16_t n = *(uint16_t*)(z+i+10);
        uint32_t off = *(uint32_t*)(z+i+16);
        uint32_t p = off;
        for (uint16_t k = 0; k < n && p + 46 <= zs; k++) {
            if (*(uint32_t*)(z+p) != 0x02014b50) break;
            uint16_t nlen = *(uint16_t*)(z+p+28), elen = *(uint16_t*)(z+p+30), clen = *(uint16_t*)(z+p+32);
            const uint8_t *nm = z + p + 46;
            if (p + 46 + nlen <= zs) {
                uint32_t nl = 0; while (nl < nlen && nm[nl]) nl++;
                const char *s = name; uint32_t mi = 0;
                for (; s[mi] && mi < nl; mi++) {
                    char a = nm[mi], b = s[mi];
                    if (a >= 'A' && a <= 'Z') a += 32;
                    if (b >= 'A' && b <= 'Z') b += 32;
                    if (a != b) break;
                }
                if (mi == nl && !s[mi]) { found = 1; break; }
            }
            p += 46 + nlen + elen + clen;
        }
    }
    free(z);
    return found;
}
