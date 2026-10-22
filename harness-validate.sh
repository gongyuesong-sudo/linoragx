#!/bin/bash
# harness 验证器：harness_validate.sh <rom> <min_colors> | final
# rom 模式：投币+start 跑 12000 帧，色数>=min 则 exit 0
# final 模式：复测结果(>=68条)与正片终表(final_table.json, 208条)齐备则 exit 0
BIN=/home/gill/linoragx/reawork/rewrite/build/linoragx
PY=$HOME/opt/miniforge/bin/python
ROM="$1"; MIN="$2"
if [ "$ROM" = "final" ]; then
  $PY - <<'EOF'
import json, os, sys
rt = os.path.expanduser('~/.hermes/cache/scratch/retest_multi/result.json')
ft = os.path.expanduser('~/.hermes/cache/scratch/final_table.json')
ok = True
try:
    r = json.load(open(rt))
    print('retest entries:', len(r))
    ok &= len(r) >= 68
except Exception as e:
    print('retest missing:', e); ok = False
try:
    f = json.load(open(ft))
    print('final_table entries:', len(f))
    ok &= len(f) >= 208
except Exception as e:
    print('final_table missing:', e); ok = False
sys.exit(0 if ok else 1)
EOF
  exit $?
fi
BMP=/tmp/hv_$ROM.bmp
LINORAGX_INPUT="5000:10:30,5100:10:30,5200:10:30,5400:8:30" timeout 170 "$BIN" --rom /home/gill/linoragx/roms/$ROM.zip --bios /home/gill/linoragx --frames 12000 --headless --no-audio --dump "$BMP" >/dev/null 2>&1
n=$($PY -c "from PIL import Image;print(len(Image.open('$BMP').convert('RGB').getcolors(1000000)))" 2>/dev/null)
echo "$ROM colors=${n:-0} min=$MIN"
[ "${n:-0}" -ge "$MIN" ]
