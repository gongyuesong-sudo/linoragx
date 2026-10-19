#!/usr/bin/env python3
"""Harness validation helpers for linoragx.

usage:
  harness_val.py glitch <c1,c2,...> <min_ok> [threshold=0.35] [frames=12000]
      run each card headless, count cards with noise < threshold.
      BLACK (<=1 color) counts as fail. pass if count >= min_ok.
  harness_val.py speed [frames=3000] [min_fps=600]
      headless fps check on kof2000.
  harness_val.py input [card=kof2000]
      LINORAGX_INPUT coin+start must change the screen vs no-input run.
  harness_val.py audio [card=kof2000] [rms_min=0.001] [rms_max=0.5]
      requires --dump-audio <wav>; check PCM rms in band.
"""
import os, struct, subprocess, sys, time, wave

ROOT = "/home/gill/linoragx"
BIN = os.environ.get("LINORAGX_BIN", f"{ROOT}/reawork/rewrite/build/linoragx")
BIOS = ROOT
OUT = os.environ.get("HARNESS_VAL_OUT", os.path.expanduser("~/.hermes/cache/scratch/harness_val"))


def bmp_stats(p):
    with open(p, "rb") as f:
        d = f.read()
    off = struct.unpack_from("<I", d, 10)[0]
    w = struct.unpack_from("<i", d, 18)[0]
    h = abs(struct.unpack_from("<i", d, 22)[0])
    rowsz = (w * 3 + 3) // 4 * 4
    diff = tot = 0
    s = set()
    for y in range(h):
        row = d[off + y * rowsz: off + y * rowsz + w * 3]
        for x in range(0, (w - 1) * 3, 3):
            if row[x:x + 3] != row[x + 3:x + 6]:
                diff += 1
            tot += 1
        for x in range(0, w * 3, 3):
            s.add(row[x:x + 3])
    return diff / tot, len(s)


def run_dump(card, frames, extra_env=None, dump="d.bmp", extra_args=None):
    os.makedirs(OUT, exist_ok=True)
    dp = os.path.join(OUT, dump)
    if os.path.exists(dp):
        os.remove(dp)
    env = dict(os.environ)
    if extra_env:
        env.update(extra_env)
    cmd = [BIN, "--rom", f"{ROOT}/roms/{card}.zip", "--bios", BIOS,
           "--frames", str(frames), "--headless", "--dump", dp]
    if extra_args:
        cmd += extra_args
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=900)
    return dp, r


def cmd_glitch(args):
    cards = args[0].split(",")
    min_ok = int(args[1])
    thr = float(args[2]) if len(args) > 2 else 0.35
    frames = int(args[3]) if len(args) > 3 else 12000
    ok = 0
    for c in cards:
        dp, r = run_dump(c, frames, dump=f"{c}.bmp")
        if not os.path.exists(dp):
            print(f"{c:12s} NO_DUMP rc={r.returncode}")
            continue
        n, col = bmp_stats(dp)
        good = col > 1 and n < thr
        ok += good
        print(f"{c:12s} noise={n:.3f} colors={col} {'OK' if good else 'FAIL'}")
    print(f"glitch_ok={ok}/{len(cards)} need>={min_ok}")
    return 0 if ok >= min_ok else 1


def cmd_speed(args):
    frames = int(args[0]) if args else 3000
    min_fps = float(args[1]) if len(args) > 1 else 600
    t0 = time.time()
    dp, r = run_dump("kof2000", frames, dump="speed.bmp")
    dt = time.time() - t0
    fps = frames / dt
    print(f"frames={frames} dt={dt:.1f}s fps={fps:.0f} need>={min_fps:.0f}")
    return 0 if fps >= min_fps else 1


def cmd_input(args):
    card = args[0] if args else "kof2000"
    dp0, _ = run_dump(card, 1200, dump="in_plain.bmp")
    dp1, _ = run_dump(card, 1200, dump="in_inject.bmp",
                      extra_env={"LINORAGX_INPUT": "120:10:15,300:8:15,600:8:15"})
    if not (os.path.exists(dp0) and os.path.exists(dp1)):
        print("missing dump")
        return 1
    a = open(dp0, "rb").read()
    b = open(dp1, "rb").read()
    n1, c1 = bmp_stats(dp1)
    changed = a != b
    print(f"screen_changed={changed} inject_colors={c1}")
    return 0 if changed and c1 > 16 else 1


def cmd_audio(args):
    card = args[0] if args else "kof2000"
    rms_min = float(args[1]) if len(args) > 1 else 0.001
    rms_max = float(args[2]) if len(args) > 2 else 0.5
    os.makedirs(OUT, exist_ok=True)
    wav = os.path.join(OUT, "audio.wav")
    if os.path.exists(wav):
        os.remove(wav)
    cmd = [BIN, "--rom", f"{ROOT}/roms/{card}.zip", "--bios", BIOS,
           "--frames", "600", "--headless", "--dump-audio", wav]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if not os.path.exists(wav):
        print(f"no audio dump produced rc={r.returncode} {r.stderr[-300:]}")
        return 1
    with wave.open(wav, "rb") as w:
        n = w.getnframes()
        data = w.readframes(n)
    import array
    a = array.array("h", data)
    if not a:
        print("empty wav")
        return 1
    rms = (sum(x * x for x in a) / len(a)) ** 0.5 / 32768.0
    print(f"frames={n} rms={rms:.4f} band=[{rms_min},{rms_max}]")
    return 0 if rms_min <= rms <= rms_max else 1


if __name__ == "__main__":
    dispatch = {"glitch": cmd_glitch, "speed": cmd_speed,
                "input": cmd_input, "audio": cmd_audio}
    if len(sys.argv) < 2 or sys.argv[1] not in dispatch:
        print(__doc__)
        sys.exit(2)
    sys.exit(dispatch[sys.argv[1]](sys.argv[2:]))
