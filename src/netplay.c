/* LAN netplay: lockstep input sync over TCP (MAME netplay style).
 *
 * Both machines run the same deterministic emulation; each emulated frame the
 * two sides exchange one 8-byte input record (frame, P1 mask, P2 mask) and
 * only advance when both inputs for that frame are known. No video/audio
 * streaming: each side renders locally (Vulkan). On LAN (RTT 1-5 ms) the
 * per-frame lockstep is imperceptible; TCP_NODELAY avoids Nagle stalls.
 *
 * host = local P1, client = local P2. Handshake exchanges magic + game CRC so
 * mismatched carts fail immediately instead of desyncing silently.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#include "linoragx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#define NET_MAGIC 0x4C4E5831u   /* "LNX1" */
#define NET_PORT  55300

static int io_all(int fd, void *p, int n, int do_send)
{
    char *b = p;
    while (n > 0) {
        ssize_t r = do_send ? send(fd, b, n, 0) : recv(fd, b, n, 0);
        if (r <= 0) return -1;    /* closed or SO_RCVTIMEO timeout */
        b += r; n -= (int)r;
    }
    return 0;
}

static int net_handshake(int fd, uint32_t game_crc)
{
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    struct timeval tv = { 10, 0 };                 /* 10 s stall = peer lost */
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    uint32_t out[2] = { htonl(NET_MAGIC), htonl(game_crc) };
    uint32_t in[2];
    if (io_all(fd, out, 8, 1) < 0 || io_all(fd, in, 8, 0) < 0) return -1;
    if (ntohl(in[0]) != NET_MAGIC) return -1;
    if (ntohl(in[1]) != game_crc) return -2;       /* different game loaded */
    return 0;
}

/* block until a peer connects; returns conn fd or -1 (port in use etc.) */
int net_host(int port, uint32_t game_crc)
{
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons((uint16_t)port);
    if (bind(s, (struct sockaddr *)&a, sizeof a) < 0 || listen(s, 1) < 0) { close(s); return -1; }
    int fd = accept(s, NULL, NULL);
    close(s);
    if (fd < 0) return -1;
    if (net_handshake(fd, game_crc) < 0) { close(fd); return -1; }
    return fd;
}

/* hostport = "ip" or "ip:port" */
int net_connect(const char *hostport, uint32_t game_crc)
{
    char buf[128];
    snprintf(buf, sizeof buf, "%s", hostport);
    int port = NET_PORT;
    char *colon = strrchr(buf, ':');
    if (colon) { *colon = 0; port = atoi(colon + 1); }
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, buf, &a.sin_addr) != 1) { close(fd); return -1; }
    if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
    if (net_handshake(fd, game_crc) < 0) { close(fd); return -1; }
    return fd;
}

/* exchange this frame's inputs; returns 0 ok, -1 peer lost, -2 desync */
int net_step(int fd, uint64_t frame, uint32_t local_mask, uint32_t *remote_mask)
{
    uint8_t out[8], in[8];
    uint32_t f = htonl((uint32_t)frame);
    uint16_t m = htons((uint16_t)(local_mask & 0xFFFF));
    memcpy(out, &f, 4); memcpy(out + 4, &m, 2); out[6] = out[7] = 0;
    if (io_all(fd, out, 8, 1) < 0) return -1;
    if (io_all(fd, in, 8, 0) < 0) return -1;
    uint32_t rf; uint16_t rm;
    memcpy(&rf, in, 4); memcpy(&rm, in + 4, 2);
    if (ntohl(rf) != (uint32_t)frame) return -2;
    *remote_mask = ntohs(rm);
    return 0;
}

void net_close(int fd) { if (fd >= 0) close(fd); }
