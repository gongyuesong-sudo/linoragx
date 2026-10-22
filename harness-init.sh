#!/bin/bash
set -e
test -x /home/gill/linoragx/reawork/rewrite/build/linoragx || { echo "FAIL: build/linoragx missing"; exit 1; }
test -f /home/gill/linoragx/uni-bios_3_2.rom || { echo "FAIL: BIOS missing"; exit 1; }
test -d /home/gill/linoragx/roms || { echo "FAIL: roms dir missing"; exit 1; }
cd /home/gill/linoragx/reawork/rewrite && make -j$(nproc) >/dev/null 2>&1 || { echo "FAIL: build broken"; exit 1; }
echo "Environment health check complete"
