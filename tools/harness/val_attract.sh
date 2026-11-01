#!/bin/bash
# val_attract.sh <rom> <min_colors> [frames...] — 投币+start，多帧数转储取最大色数 >= min 则通过
# 用于 attract 场景内容判定（单帧采样会踩转场/淡入淡出相位）
BIN=/home/gill/linoragx/reawork/rewrite/build/linoragx
PY=$HOME/opt/miniforge/bin/python
ROM="$1"; MIN="$2"; shift 2
FRAMES="${*:-18000 20000 22000 24000}"
max=0
for f in $FRAMES; do
  BMP=/tmp/va_${ROM}_$f.bmp
  LINORAGX_INPUT="5000:10:30,5100:10:30,5200:10:30,5400:8:30" MCLOCK=12345 timeout 200 "$BIN" \
    --rom /home/gill/linoragx/roms/$ROM.zip --bios /home/gill/linoragx \
    --frames $f --headless --no-audio --dump "$BMP" >/dev/null 2>&1
  n=$($PY -c "from PIL import Image;print(len(Image.open('$BMP').convert('RGB').getcolors(1000000)))" 2>/dev/null)
  n=${n:-0}; echo "$ROM f=$f colors=$n"
  [ "$n" -gt "$max" ] && max=$n
done
echo "$ROM max=$max min=$MIN"
[ "$max" -ge "$MIN" ]
