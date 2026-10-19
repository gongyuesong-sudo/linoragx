#!/bin/bash
# harness validation: windowed Vulkan present path stays up 25s on x11 backend
cd /home/gill/linoragx || exit 1
out=$(DISPLAY=:0.0 timeout -s KILL 25 env LD_LIBRARY_PATH=$HOME/opt/lib/emulink \
  reawork/rewrite/build/linoragx --rom roms/kof2000.zip --bios /home/gill/linoragx 2>&1)
code=$?
echo "exit=$code"
if [ "$code" -eq 124 ] && echo "$out" | grep -q "window backend"; then
  echo "WINDOW_OK"; exit 0
fi
echo "$out" | tail -5
exit 1
