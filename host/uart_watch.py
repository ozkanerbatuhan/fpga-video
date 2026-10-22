"""Wait for the ZedBoard USB-UART, print the firmware console, exit on a marker.

    python host/uart_watch.py                         wait for the port, print until "listening on"
    python host/uart_watch.py --until never --seconds 60
"""
import argparse
import sys
import time

import serial
import serial.tools.list_ports as lp


def find_port():
    for p in lp.comports():
        d = (p.description or "") + " " + (p.manufacturer or "")
        if any(k in d for k in ("Silicon Labs", "CP210", "USB Serial", "UART")):
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--until", default="listening on")
    ap.add_argument("--seconds", type=float, default=600)
    a = ap.parse_args()

    t0 = time.time()
    port = a.port
    while not port:
        port = find_port()
        if time.time() - t0 > a.seconds:
            sys.exit("no UART port found")
        if not port:
            time.sleep(1)
    print("UART", port, flush=True)
    s = serial.Serial(port, 115200, timeout=0.5)
    buf = b""
    while time.time() - t0 < a.seconds:
        buf += s.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").rstrip()
            print(text, flush=True)
            if a.until != "never" and a.until in text:
                return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
