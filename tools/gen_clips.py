"""Synthetic test clips, used until real camera clips are recorded (tools/capture_clips.py).

Each clip is a fixed outdoor-like scene (gradient sky, textured ground, static
blocks), per-frame sensor noise and a slow global brightness drift, with moving
textured objects. Deterministic (seeded), so tests are reproducible.

    python tools/gen_clips.py            -> clips/syn_day.yuv, syn_busy.yuv, syn_night.yuv
"""
import argparse
from pathlib import Path

import numpy as np
from scipy import ndimage

from yuv import write_frames

W, H, FPS = 640, 360, 25


def scene(rng, dark):
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    sky = 170 - 60 * yy / H
    tex = ndimage.gaussian_filter(rng.normal(0, 1, (H, W)), 3) * 60
    ground = np.where(yy > H * 0.55, 95 + tex, sky + tex * 0.2)
    for _ in range(6):
        x0, y0 = rng.integers(0, W - 80), rng.integers(int(H * 0.25), int(H * 0.5))
        w, h = rng.integers(30, 90), rng.integers(40, 110)
        ground[y0:y0 + h, x0:x0 + w] = rng.integers(60, 200)
    cb = 128 + ndimage.gaussian_filter(rng.normal(0, 1, (H, W)), 8) * 40
    cr = 128 + ndimage.gaussian_filter(rng.normal(0, 1, (H, W)), 8) * 40
    if dark:
        ground = ground * 0.25 + 10
        cb = 128 + (cb - 128) * 0.3
        cr = 128 + (cr - 128) * 0.3
    return ground, cb, cr


def make_objects(rng, n, dark):
    objs = []
    for i in range(n):
        objs.append(dict(
            x=rng.uniform(-60, W), y=rng.uniform(H * 0.45, H * 0.85),
            vx=rng.choice([-1, 1]) * rng.uniform(1.5, 5.0), vy=rng.uniform(-0.6, 0.6),
            w=rng.integers(20, 70), h=rng.integers(25, 60),
            lum=rng.integers(20, 60) if i % 2 else rng.integers(190, 240),
            cb=rng.integers(60, 200), cr=rng.integers(60, 200),
            ellipse=bool(i % 2)))
        if dark:
            objs[-1]["lum"] = objs[-1]["lum"] * 0.35 + 25
    return objs


def render(name, n_obj, dark, noise, seconds, seed):
    rng = np.random.default_rng(seed)
    base_y, base_cb, base_cr = scene(rng, dark)
    objs = make_objects(rng, n_obj, dark)
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    frames = []
    for t in range(seconds * FPS):
        drift = 4 * np.sin(2 * np.pi * t / (FPS * 20))
        y = base_y + drift
        cb, cr = base_cb.copy(), base_cr.copy()
        for o in objs:
            o["x"] += o["vx"]
            o["y"] += o["vy"]
            if o["x"] > W + 40:
                o["x"] = -o["w"] - 40
            if o["x"] < -o["w"] - 40:
                o["x"] = W + 40
            o["y"] = float(np.clip(o["y"], H * 0.3, H - o["h"]))
            if o["ellipse"]:
                m = ((xx - o["x"] - o["w"] / 2) / (o["w"] / 2)) ** 2 + \
                    ((yy - o["y"] - o["h"] / 2) / (o["h"] / 2)) ** 2 <= 1
            else:
                m = (xx >= o["x"]) & (xx < o["x"] + o["w"]) & (yy >= o["y"]) & (yy < o["y"] + o["h"])
            stripes = 12 * np.sin((xx - o["x"]) / 4.0)
            y = np.where(m, o["lum"] + stripes, y)
            cb = np.where(m, o["cb"], cb)
            cr = np.where(m, o["cr"], cr)
        y = y + rng.normal(0, noise, (H, W))
        y = np.clip(np.round(y), 16, 235).astype(np.uint16)
        cb2 = np.clip(np.round((cb[:, 0::2] + cb[:, 1::2]) / 2), 16, 240).astype(np.uint16)
        cr2 = np.clip(np.round((cr[:, 0::2] + cr[:, 1::2]) / 2), 16, 240).astype(np.uint16)
        c = np.empty((H, W), np.uint16)
        c[:, 0::2], c[:, 1::2] = cb2, cr2
        frames.append((c << 8) | y)
    return np.stack(frames)


CLIPS = {
    "syn_day":   dict(n_obj=2, dark=False, noise=2.0, seed=1),
    "syn_busy":  dict(n_obj=5, dark=False, noise=3.0, seed=2),
    "syn_night": dict(n_obj=2, dark=True,  noise=5.0, seed=3),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "clips"))
    ap.add_argument("--seconds", type=int, default=30)
    a = ap.parse_args()
    Path(a.out).mkdir(exist_ok=True)
    for name, kw in CLIPS.items():
        f = render(name, seconds=a.seconds, **kw)
        write_frames(Path(a.out) / f"{name}.yuv", f, FPS, "synthetic:" + name)
        print(name, f.shape)


if __name__ == "__main__":
    main()
