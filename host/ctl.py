"""Run-time control of the board pipeline.

    python host/ctl.py mode 3          0 orig, 1 gray, 2 gauss, 3 sobel, 4 raw mask, 5 mask
    python host/ctl.py overlay 0|1
    python host/ctl.py thresh 25
    python host/ctl.py alpha 5         background EMA alpha = 2^-5
    python host/ctl.py sobel 2         sobel magnitude shift
    python host/ctl.py minarea 60
    python host/ctl.py bgreset
    python host/ctl.py swapyc 0|1      HDMI byte lane swap (if colours are wrong)
    python host/ctl.py text 0|1        automatic frame/object text
    python host/ctl.py bars            colour bars on the current display buffer
"""
import socket
import struct
import sys

CMDS = dict(mode=1, overlay=2, thresh=3, alpha=4, sobel=5, minarea=6, bgreset=7,
            swapyc=8, text=9, bars=10)


def send(cmd, val=0, board="192.168.1.10"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)
    s.sendto(struct.pack("<HHI", 0x5643, CMDS[cmd], val), (board, 5001))
    try:
        s.recvfrom(64)
        return True
    except socket.timeout:
        return False


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in CMDS:
        sys.exit(__doc__)
    ok = send(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    print("ack" if ok else "no answer from board")
