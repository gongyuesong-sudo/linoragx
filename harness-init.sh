#!/bin/bash
# harness health check (idempotent): build + headless smoke
set -e
cd /home/gill/linoragx/reawork/rewrite
make -j"$(nproc)" > /tmp/harness_build.log 2>&1 || { echo "BUILD FAIL"; tail -20 /tmp/harness_build.log; exit 1; }
cd /home/gill/linoragx
out=$(reawork/rewrite/build/linoragx --rom roms/kof2000.zip --bios /home/gill/linoragx \
  --frames 300 --headless --dump /tmp/harness_smoke.bmp 2>&1 | tail -2)
echo "$out"
[ -s /tmp/harness_smoke.bmp ] && echo "Environment health check complete" || { echo "SMOKE FAIL"; exit 1; }
