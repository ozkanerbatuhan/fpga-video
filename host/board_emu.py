"""Software stand-in for the board: same UDP protocol, golden model instead of the PL.

Used to test host/sender.py and the protocol without hardware:
    python host/board_emu.py &
    python host/sender.py --clip clips/syn_day.yuv --board 127.0.0.1 --frames 50 --verify
"""
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))
import golden as G  # noqa: E402

W, H = 640, 360
LINE = W * 2


def now_us():
    return int(time.perf_counter() * 1e6) & 0xFFFFFFFF


def main(port=5000):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    s.bind(("127.0.0.1", port))
    pipe = G.Pipeline(G.Regs())
    buf = np.zeros(H * LINE, np.uint8)
    cur, lines, t_send, t_first = None, 0, 0, 0
    stats = dict(rx=0, proc=0, dropped=0, lost=0)
    print("board emulator on 127.0.0.1:5000", flush=True)

    def finish(peer):
        nonlocal lines
        stats["rx"] += 1
        stats["lost"] += H - lines
        frame = buf.view("<u2").reshape(H, W)
        t_kick = now_us()
        res = pipe.step(frame)
        stats["proc"] += 1
        boxes = [(b[0], b[1], b[2], b[3]) for b in res.boxes] + [(0, 0, 0, 0)] * (8 - len(res.boxes))
        areas = [b[4] for b in res.boxes] + [0] * (8 - len(res.boxes))
        pkt = struct.pack("<HHIIIIIIHH", 0x5652, cur, 0, t_send, t_first, now_us(), t_kick, now_us(),
                          lines, len(res.boxes))
        pkt += struct.pack("<32H", *[v for b in boxes for v in b])
        pkt += struct.pack("<8I", *areas)
        pkt += struct.pack("<5I", stats["rx"], stats["proc"], stats["dropped"], stats["lost"], 0)
        s.sendto(pkt, (peer[0], 5002))

    peer = None
    while True:
        data, peer = s.recvfrom(4096)
        magic, frame, line, n, ts = struct.unpack("<HHHHI", data[:12])
        if magic != 0x5646:
            continue
        if cur is not None and frame != cur:
            finish(peer)
            cur = None
        if cur is None:
            cur, lines, t_send, t_first = frame, 0, ts, now_us()
        buf[line * LINE:(line + n) * LINE] = np.frombuffer(data[12:12 + n * LINE], np.uint8)
        lines += n
        if lines >= H:
            finish(peer)
            cur = None


if __name__ == "__main__":
    main()
