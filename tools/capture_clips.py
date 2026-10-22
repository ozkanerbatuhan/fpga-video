"""Record test clips from the IP camera in the pipeline's format (640x360 YUYV raw).

    python tools/capture_clips.py day_moving --seconds 30
    python tools/capture_clips.py night --seconds 30 --main     (use the main stream)

Camera URL comes from .env (never committed):
    CAM_URL=rtsp://user:password@192.168.1.64:554/Streaming/Channels/102
Hikvision: channel 101 = main stream, 102 = sub stream.
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from sender import load_env  # noqa: E402

W, H, FPS = 640, 360, 25


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--seconds", type=int, default=30)
    ap.add_argument("--main", action="store_true", help="main stream (101) instead of sub stream (102)")
    a = ap.parse_args()

    url = load_env().get("CAM_URL")
    if not url:
        sys.exit("CAM_URL missing in .env (see .env.example)")
    if a.main:
        url = url.replace("/102", "/101")
    out = ROOT / "clips" / f"{a.name}.yuv"
    ff = os.environ.get("FFMPEG", "ffmpeg")
    cmd = [ff, "-y", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", url, "-t", str(a.seconds),
           "-vf", f"scale={W}:{H},fps={FPS}", "-pix_fmt", "yuyv422", "-f", "rawvideo", str(out)]
    subprocess.run(cmd, check=True)
    n = out.stat().st_size // (W * H * 2)
    meta = {"width": W, "height": H, "fps": FPS, "frames": n, "format": "yuyv422",
            "source": "camera:" + ("main" if a.main else "sub")}
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    # small mp4 preview next to it
    subprocess.run([ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "yuyv422", "-s", f"{W}x{H}",
                    "-r", str(FPS), "-i", str(out), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    str(out.with_suffix(".mp4"))], check=True)
    print(f"{out}: {n} frames")


if __name__ == "__main__":
    main()
