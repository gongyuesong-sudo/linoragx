#!/usr/bin/env python3
"""mcp_linoragx.py — MCP (Model Context Protocol) server for the linoragx Neo Geo emulator.

Speaks JSON-RPC 2.0 over stdio (LSP Content-Length framing).
Spawns the emulator with LINORAGX_CTL=<port> and drives it over the localhost
control channel. Register in any MCP client as:

  command: python3
  args: ["/home/gill/linoragx/reawork/rewrite/tools/mcp_linoragx.py"]
  env:   LINORAGX_BIN=/home/gill/linoragx/reawork/bin/linoragx
         LINORAGX_BIOS=/home/gill/linoragx
"""
import base64
import json
import os
import select
import socket
import subprocess
import sys
import time

BIN = os.environ.get("LINORAGX_BIN", "/home/gill/linoragx/reawork/bin/linoragx")
BIOS = os.environ.get("LINORAGX_BIOS", "/home/gill/linoragx")
ROMS = os.environ.get("LINORAGX_ROMS", os.path.join(BIOS, "roms"))

class Emu:
    def __init__(self):
        self.proc = None
        self.sock = None
        self.port = 0
        self.log = []

    def launch(self, rom, headless=True, bios=None, extra=None):
        if self.proc and self.proc.poll() is None:
            raise RuntimeError("emulator already running; call quit first")
        self.port = 27600 + (os.getpid() % 400)
        env = dict(os.environ, LINORAGX_CTL=str(self.port))
        # LINORAGX_BIOS is the emulator's own force-BIOS hook; we pass --bios
        # instead, so never leak the server-side env var into the child.
        env.pop("LINORAGX_BIOS", None)
        env.setdefault("LD_LIBRARY_PATH", os.path.expanduser("~/opt/lib/emulink"))
        cmd = [BIN, "--rom", rom, "--bios", bios or BIOS]
        if not os.path.exists(rom):
            cand = os.path.join(ROMS, rom if not rom.endswith(".zip") else rom)
            if os.path.exists(cand):
                cmd[cmd.index(rom)] = cand
        if headless:
            cmd += ["--headless", "--no-audio"]
        if extra:
            cmd += extra
        self.proc = subprocess.Popen(cmd, env=env, stderr=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL)
        deadline = time.time() + 20
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("emulator exited early (code %s)" % self.proc.returncode)
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=1)
                break
            except OSError:
                time.sleep(0.2)
        else:
            raise RuntimeError("control channel never came up on port %d" % self.port)
        self.sock.settimeout(5)
        return self.cmd("state")

    def cmd(self, line):
        if not self.sock:
            raise RuntimeError("emulator not running")
        self.sock.sendall((line + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("control channel closed")
            buf += chunk
        return json.loads(buf.split(b"\n", 1)[0])

    def wait_frames(self, target, timeout=120.0):
        end = time.time() + timeout
        while time.time() < end:
            st = self.cmd("state")
            if st.get("frame", 0) >= target:
                return st
            time.sleep(0.05)
        raise TimeoutError("frame %d not reached" % target)

    def quit(self):
        try:
            if self.sock:
                self.sock.sendall(b"quit\n")
                self.sock.close()
        except OSError:
            pass
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.sock = None
        self.proc = None

EMU = Emu()

TOOLS = [
    ("launch", "Launch a Neo Geo ROM in the emulator (headless by default).",
     {"rom": "path to rom zip (bare name resolves under %s)" % ROMS,
      "bios": "BIOS dir (optional)", "headless": "bool, default true"}, ["rom"]),
    ("status", "Get emulator status: frame, PC, paused, game.", {}, []),
    ("screenshot", "Capture current video frame as BMP (returned as image).",
     {"path": "optional file path to also save the BMP"}, []),
    ("press", "Press a button for N frames then auto-release.",
     {"button": "up|down|left|right|b|a|c|z|l|r|start|select", "player": "0 or 1",
      "hold_frames": "int, default 3"}, ["button"]),
    ("set_buttons", "Set the full button bitmask of a player (bit0 up,1 down,2 left,3 right,4 B,5 A,6 C,7 Z,8 L,9 R,10 start,11 select).",
     {"mask": "int bitmask", "player": "0 or 1"}, ["mask"]),
    ("release_buttons", "Release buttons given by bitmask.", {"mask": "int", "player": "0 or 1"}, ["mask"]),
    ("pause", "Pause emulation.", {}, []),
    ("resume", "Resume emulation.", {}, []),
    ("step", "Run exactly N frames while paused.", {"frames": "int, default 1"}, []),
    ("wait_frames", "Block until the emulator reaches the given frame number.",
     {"frame": "int target frame", "timeout": "seconds, default 120"}, ["frame"]),
    ("save_state", "Save emulator state to a file.", {"path": "file path"}, ["path"]),
    ("load_state", "Load emulator state from a file.", {"path": "file path"}, ["path"]),
    ("peek", "Read 68K memory bytes (hex string).", {"addr": "hex address e.g. 0x10FE00", "n": "byte count, max 64"}, ["addr"]),
    ("list_games", "List available ROM zips.", {"dir": "optional dir, default %s" % ROMS}, []),
    ("quit", "Quit the emulator.", {}, []),
]

def call_tool(name, args):
    args = args or {}
    if name == "launch":
        rom = args["rom"]
        if not os.path.sep in rom and not rom.endswith(".zip"):
            rom = os.path.join(ROMS, rom + ".zip")
        if not os.path.sep in rom:
            rom = os.path.join(ROMS, rom)
        r = EMU.launch(rom, headless=args.get("headless", True), bios=args.get("bios"))
        return {"text": json.dumps({"launched": rom, "state": r})}
    if name == "status":
        return {"text": json.dumps(EMU.cmd("state"))}
    if name == "screenshot":
        path = args.get("path") or "/tmp/linoragx_mcp.bmp"
        r = EMU.cmd("dump " + path)
        out = {"dumped": r}
        if os.path.exists(path):
            with open(path, "rb") as f:
                out["image_b64"] = base64.b64encode(f.read()).decode()
            out["mime"] = "image/bmp"
        return out
    if name == "press":
        return {"text": json.dumps(EMU.cmd("press %d %s %d" % (
            int(args.get("player", 0)), args["button"], int(args.get("hold_frames", 3)))))}
    if name == "set_buttons":
        return {"text": json.dumps(EMU.cmd("btn %d %d" % (int(args.get("player", 0)), int(args["mask"]))))}
    if name == "release_buttons":
        return {"text": json.dumps(EMU.cmd("release %d %d" % (int(args.get("player", 0)), int(args["mask"]))))}
    if name == "pause":
        return {"text": json.dumps(EMU.cmd("pause"))}
    if name == "resume":
        return {"text": json.dumps(EMU.cmd("resume"))}
    if name == "step":
        return {"text": json.dumps(EMU.cmd("step %d" % int(args.get("frames", 1))))}
    if name == "wait_frames":
        st = EMU.wait_frames(int(args["frame"]), float(args.get("timeout", 120)))
        return {"text": json.dumps(st)}
    if name == "save_state":
        return {"text": json.dumps(EMU.cmd("save " + args["path"]))}
    if name == "load_state":
        return {"text": json.dumps(EMU.cmd("load " + args["path"]))}
    if name == "peek":
        return {"text": json.dumps(EMU.cmd("peek %s %d" % (args["addr"], int(args.get("n", 4)))))}
    if name == "list_games":
        d = args.get("dir") or ROMS
        games = sorted(f[:-4] for f in os.listdir(d) if f.endswith(".zip"))
        return {"text": json.dumps({"dir": d, "count": len(games), "games": games})}
    if name == "quit":
        EMU.quit()
        return {"text": json.dumps({"ok": True})}
    raise ValueError("unknown tool: " + name)

def tool_result(name, out):
    if "image_b64" in out:
        return {"content": [
            {"type": "text", "text": json.dumps({k: v for k, v in out.items() if k != "image_b64"})},
            {"type": "image", "data": out["image_b64"], "mimeType": out["mime"]}]}
    return {"content": [{"type": "text", "text": out.get("text", "")}]}

def read_msg():
    headers = {}
    while True:
        line = sys.stdin.buffer.readline()
        if not line:
            return None
        line = line.strip()
        if not line:
            break
        if b":" in line:
            k, v = line.split(b":", 1)
            headers[k.strip().lower()] = v.strip()
    n = int(headers.get(b"content-length", 0))
    if not n:
        return None
    data = sys.stdin.buffer.read(n)
    return json.loads(data)

def send_msg(obj):
    data = json.dumps(obj).encode()
    sys.stdout.buffer.write(b"Content-Length: %d\r\n\r\n" % len(data) + data)
    sys.stdout.buffer.flush()

def serve():
    while True:
        msg = read_msg()
        if msg is None:
            break
        mid = msg.get("id")
        method = msg.get("method", "")
        try:
            if method == "initialize":
                send_msg({"jsonrpc": "2.0", "id": mid, "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "linoragx", "version": "1.0"}}})
            elif method in ("notifications/initialized", "initialized"):
                pass
            elif method == "ping":
                send_msg({"jsonrpc": "2.0", "id": mid, "result": {}})
            elif method == "tools/list":
                send_msg({"jsonrpc": "2.0", "id": mid, "result": {"tools": [
                    {"name": n, "description": d,
                     "inputSchema": {"type": "object", "properties": {
                         k: {"type": "string", "description": v} for k, v in props.items()},
                         "required": req}}
                    for n, d, props, req in TOOLS]}})
            elif method == "tools/call":
                params = msg.get("params", {})
                try:
                    res = tool_result(params.get("name"),
                                      call_tool(params.get("name"), params.get("arguments")))
                except Exception as e:
                    res = {"content": [{"type": "text", "text": "error: %s" % e}],
                           "isError": True}
                send_msg({"jsonrpc": "2.0", "id": mid, "result": res})
            elif mid is not None:
                send_msg({"jsonrpc": "2.0", "id": mid,
                          "error": {"code": -32601, "message": "method not found: " + method}})
        except Exception as e:
            if mid is not None:
                send_msg({"jsonrpc": "2.0", "id": mid,
                          "error": {"code": -32603, "message": str(e)}})
    EMU.quit()

if __name__ == "__main__":
    serve()
