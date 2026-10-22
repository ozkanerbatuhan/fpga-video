"""Run the golden model over a clip and render a preview video plus before/after frames.

    python model/run_model.py clips/syn_day.yuv [--frames 250] [--out docs/media]

The preview is a 2x2 mosaic: input | sobel, motion mask | boxes on input.
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "model"))

from golden import Pipeline, Regs, MODE_ORIG  # noqa: E402
from yuv import read_frames, yuyv_to_rgb, gray_to_yuyv  # noqa: E402


def ffmpeg_bin():
    return os.environ.get("FFMPEG", "ffmpeg")


def mosaic(frame, res):
    a = yuyv_to_rgb(frame)
    b = yuyv_to_rgb(gray_to_yuyv(res.sobel))
    c = yuyv_to_rgb(gray_to_yuyv(res.mask * 255))
    d = yuyv_to_rgb(res.out)
    return np.vstack([np.hstack([a, b]), np.hstack([c, d])])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--frames", type=int, default=None)
    ap.add_argument("--out", default=str(ROOT / "docs" / "media"))
    a = ap.parse_args()

    frames = read_frames(a.clip, count=a.frames)
    name = Path(a.clip).stem
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    regs = Regs(width=frames.shape[2], height=frames.shape[1], mode=MODE_ORIG, overlay=True)
    pipe = Pipeline(regs)

    h, w = frames.shape[1] * 2, frames.shape[2] * 2
    enc = subprocess.Popen([ffmpeg_bin(), "-y", "-loglevel", "error", "-f", "rawvideo",
                            "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "25", "-i", "-",
                            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
                            str(out / f"{name}_model.mp4")], stdin=subprocess.PIPE)
    counts = []
    for i, f in enumerate(frames):
        regs.text = f"F{i:04d} OBJ {len(pipe.prev_boxes)}"
        res = pipe.step(f)
        counts.append((len(res.boxes), res.n_components))
        m = mosaic(f, res)
        enc.stdin.write(m.tobytes())
        if i == min(len(frames) - 1, 100):
            from PIL import Image
            Image.fromarray(m).save(out / f"{name}_f{i}.png")
    enc.stdin.close()
    enc.wait()
    boxes = np.array(counts)
    print(f"{name}: {len(frames)} frames, boxes/frame mean {boxes[:, 0].mean():.2f}, "
          f"components max {boxes[:, 1].max()}")


if __name__ == "__main__":
    main()
