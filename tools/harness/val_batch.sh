#!/bin/bash
# harness validation: full 207-card regression, pass if OK>=60 and GLITCH<=45
set -e
DIR=~/.hermes/cache/scratch/batch25
mkdir -p "$DIR"
sed -e 's#batch2[0-9]#batch25#g' ~/.hermes/cache/scratch/batch_test24.py > "$DIR/run.py"
python3 "$DIR/run.py" > "$DIR/log.txt" 2>&1 || true
python3 - "$DIR/result.csv" <<'EOF'
import csv,sys,collections
c=collections.Counter(r['class'] for r in csv.DictReader(open(sys.argv[1])))
print(dict(c))
ok=c.get('OK',0); gl=c.get('GLITCH',0)
print(f"OK={ok} need>=60 GLITCH={gl} need<=45")
sys.exit(0 if ok>=60 and gl<=45 else 1)
EOF
