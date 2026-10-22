"""PC side: decode video, send it to the ZedBoard over UDP, collect per-frame results.

    python host/sender.py --clip clips/syn_busy.yuv --verify      synthetic clip, boxes checked
                                                                  against the golden model
    python host/sender.py --rtsp                                   camera (URL from .env)
    python host/sender.py --rtsp --seconds 60 --log run.csv

Protocol (little endian):
  video  -> board:5000  header {u16 'VF', u16 frame, u16 line, u16 nlines, u32 t_send_us} + lines
  result <- board:5002  see result_t in sw/app/src/main.c
Latency reported here = result arrival - first packet of that frame left the PC
(PC -> board -> PL processing -> board -> PC). The HDMI path adds at most one
display frame (16.7 ms at 60 Hz) on top; see docs/MEASUREMENTS.md.
"""
import argparse
import csv
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "model"))

W, H = 640, 360
LINE = W * 2
FRAME = LINE * H
BOARD = ("192.168.1.10", 5000)
RES_FMT = "<HHIIIIIIHH" + "H" * 32 + "I" * 8 + "IIIII"
RES_SIZE = struct.calcsize(RES_FMT)


def us():
    return int(time.perf_counter() * 1e6) & 0xFFFFFFFF


def load_env():
    env = {}
    p = ROOT / ".env"
    if p.exists():
        for line in p.read_text().splitlines():
            if "=" in line and not line.strip().startswith("#"):
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip()
    return env


def ffmpeg_bin():
    return os.environ.get("FFMPEG", "ffmpeg")


def rtsp_frames(url, low_latency=True):
    cmd = [ffmpeg_bin(), "-loglevel", "error", "-rtsp_transport", "tcp"]
    if low_latency:
        cmd += ["-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32", "-analyzeduration", "0"]
    cmd += ["-i", url, "-vf", f"scale={W}:{H}", "-pix_fmt", "yuyv422", "-f", "rawvideo", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=FRAME * 2)
    try:
        while True:
            buf = proc.stdout.read(FRAME)
            if len(buf) < FRAME:
                return
            yield np.frombuffer(buf, dtype="<u2").reshape(H, W)
    finally:
        proc.kill()


def clip_frames(path, count):
    from yuv import read_frames
    for f in read_frames(path, count=count):
        yield f


class Receiver(threading.Thread):
    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock = sock
        self.results = {}
        self.t_recv = {}
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            t = us()
            if len(data) < RES_SIZE:
                continue
            v = struct.unpack(RES_FMT, data[:RES_SIZE])
            if v[0] != 0x5652:
                continue
            r = dict(frame=v[1], pl_cycles=v[2], t_send=v[3], t_first=v[4], t_last=v[5],
                     t_kick=v[6], t_done=v[7], lines=v[8], n_boxes=v[9],
                     boxes=[tuple(v[10 + 4 * i: 14 + 4 * i]) for i in range(8)],
                     area=list(v[42:50]), rx_frames=v[50], proc_frames=v[51],
                     dropped=v[52], lost_lines=v[53], err=v[54])
            r["latency_us"] = (t - r["t_send"]) & 0xFFFFFFFF
            self.results[r["frame"]] = r


def send_frame(sock, frame_id, frame, pace_us):
    raw = frame.astype("<u2").tobytes()
    t0 = us()
    for line in range(H):
        hdr = struct.pack("<HHHHI", 0x5646, frame_id & 0xFFFF, line, 1, t0)
        sock.sendto(hdr + raw[line * LINE:(line + 1) * LINE], BOARD)
        if pace_us and line % 8 == 7:
            end = time.perf_counter() + pace_us * 8e-6
            while time.perf_counter() < end:
                pass
    return t0


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--clip")
    src.add_argument("--rtsp", nargs="?", const="env")
    ap.add_argument("--frames", type=int, default=250)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument("--pace-us", type=float, default=20.0, help="gap per packet")
    ap.add_argument("--verify", action="store_true", help="compare boxes with the golden model")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", 5002))
    rx.settimeout(0.2)
    recv = Receiver(rx)
    recv.start()

    if a.clip:
        frames = clip_frames(a.clip, a.frames)
    else:
        url = load_env().get("CAM_URL") if a.rtsp == "env" else a.rtsp
        if not url:
            sys.exit("CAM_URL missing in .env")
        frames = rtsp_frames(url)

    sent = []
    period = 1.0 / a.fps if a.clip else 0.0     # camera frames arrive at camera rate
    t_start = time.perf_counter()
    for i, f in enumerate(frames):
        if a.seconds and time.perf_counter() - t_start > a.seconds:
            break
        t_next = t_start + i * period
        while a.clip and time.perf_counter() < t_next:
            time.sleep(0.0005)
        send_frame(tx, i, f, a.pace_us)
        sent.append(f if a.verify else None)
        if i % 25 == 0:
            r = recv.results.get(i - 5)
            msg = f"sent {i + 1}"
            if r:
                msg += (f"  frame {r['frame']}: lat {r['latency_us'] / 1000:.1f} ms, pl {r['pl_cycles'] / 148.5e3:.2f} ms, "
                        f"boxes {r['n_boxes']}, dropped {r['dropped']}, lost lines {r['lost_lines']}")
            print(msg, flush=True)
    time.sleep(0.5)
    recv.stop = True
    elapsed = time.perf_counter() - t_start

    res = recv.results
    n = len(sent)
    lat = np.array([res[k]["latency_us"] for k in sorted(res)]) / 1000.0
    print(f"\nframes sent {n}, results {len(res)} ({len(res) / max(n, 1) * 100:.1f}%), {n / elapsed:.1f} fps offered")
    if len(lat):
        print(f"latency ms: mean {lat.mean():.2f}  p50 {np.percentile(lat, 50):.2f}  "
              f"p95 {np.percentile(lat, 95):.2f}  max {lat.max():.2f}")
        last = res[max(res)]
        print(f"board: rx {last['rx_frames']} proc {last['proc_frames']} dropped {last['dropped']} "
              f"lost lines {last['lost_lines']} err {last['err']:#x}")

    if a.log:
        with open(a.log, "w", newline="") as fh:
            keys = ["frame", "latency_us", "pl_cycles", "t_first", "t_last", "t_kick", "t_done",
                    "lines", "n_boxes", "rx_frames", "proc_frames", "dropped", "lost_lines", "err"]
            w = csv.writer(fh)
            w.writerow(keys)
            for k in sorted(res):
                w.writerow([res[k][x] for x in keys])
        print("log:", a.log)

    if a.verify:
        import golden as G
        pipe = G.Pipeline(G.Regs())
        ok = bad = missing = 0
        for i, f in enumerate(sent):
            exp = pipe.step(f).boxes
            r = res.get(i)
            if r is None:
                missing += 1
                continue
            got = [(b[0], b[1], b[2], b[3], r["area"][j]) for j, b in enumerate(r["boxes"][:r["n_boxes"]])]
            if got == list(exp):
                ok += 1
            else:
                bad += 1
                if bad <= 3:
                    print(f"frame {i}: board {got}\n          model {list(exp)}")
        print(f"hardware vs golden model boxes: {ok} match, {bad} differ, {missing} missing")
        if missing:
            print("note: a missing/dropped frame changes the background state, later frames may differ")


if __name__ == "__main__":
    main()
