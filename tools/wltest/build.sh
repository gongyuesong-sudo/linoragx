#!/bin/sh
# Build the headless Wayland test compositor (no root needed).
# Prereqs (one-time, all unpacked under ~/opt/sysroot-wl):
#   apt-get download libwayland-dev libxkbcommon-dev wayland-protocols
#   dpkg-deb -x each into ~/opt/sysroot-wl
# Scanner: ~/opt/ogsdeps/bin/wayland-scanner (or system wayland-scanner)
set -e
cd "$(dirname "$0")"
SYS=${SYSROOT:-$HOME/opt/sysroot-wl}
SC=${SCANNER:-$HOME/opt/ogsdeps/bin/wayland-scanner}
XML=$SYS/usr/share/wayland-protocols
$SC private-code $XML/stable/linux-dmabuf/linux-dmabuf-v1.xml dmabuf-protocol.c
$SC server-header $XML/stable/linux-dmabuf/linux-dmabuf-v1.xml dmabuf-server-protocol.h
$SC server-header $XML/stable/xdg-shell/xdg-shell.xml xdg-shell-server-protocol.h
gcc -O1 -I "$SYS/usr/include" wltest.c dmabuf-protocol.c \
    ../../src/xdg-shell-protocol.c \
    -L "$SYS/usr/lib/x86_64-linux-gnu" -lwayland-server -lxkbcommon -o wltest
echo "built: ./wltest  (run it, then WAYLAND_DISPLAY=<printed socket> linoragx --backend wayland ...)"
