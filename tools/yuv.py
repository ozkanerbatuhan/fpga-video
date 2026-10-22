"""Raw YUYV clip I/O.

A clip is a headerless file of frames in ffmpeg's yuyv422 layout: bytes Y0 U0 Y1 V0 ...
Read as little-endian 16-bit words this is exactly the pipeline's pixel word,
(C << 8) | Y, so no repacking is needed between ffmpeg, the model and the RTL.
"""
import json
from pathlib import Path

import numpy as np


def clip_meta(path):
    meta = Path(path).with_suffix(".json")
    return json.loads(meta.read_text())


def read_frames(path, width=None, height=None, start=0, count=None):
    if width is None:
        m = clip_meta(path)
        width, height = m["width"], m["height"]
    fsz = width * height
    data = np.memmap(path, dtype="<u2", mode="r")
    n = data.size // fsz
    stop = n if count is None else min(n, start + count)
    return np.array(data[start * fsz:stop * fsz]).reshape(-1, height, width)


def write_frames(path, frames, fps, source):
    frames = np.asarray(frames, dtype="<u2")
    frames.tofile(path)
    meta = {"width": frames.shape[2], "height": frames.shape[1], "fps": fps,
            "frames": frames.shape[0], "format": "yuyv422", "source": source}
    Path(path).with_suffix(".json").write_text(json.dumps(meta, indent=2))


def yuyv_to_rgb(frame):
    """For previews only (BT.601 studio range, float)."""
    y = (frame & 0xFF).astype(np.float32)
    c = (frame >> 8).astype(np.float32)
    cb = np.repeat(c[:, 0::2], 2, axis=1)
    cr = np.repeat(c[:, 1::2], 2, axis=1)
    yy = 1.164 * (y - 16)
    r = yy + 1.596 * (cr - 128)
    g = yy - 0.392 * (cb - 128) - 0.813 * (cr - 128)
    b = yy + 2.017 * (cb - 128)
    return np.clip(np.stack([r, g, b], -1), 0, 255).astype(np.uint8)


def gray_to_yuyv(y):
    return ((128 << 8) | np.asarray(y, dtype=np.uint16)).astype(np.uint16)
