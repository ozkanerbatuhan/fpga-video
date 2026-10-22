"""Bit-exact golden model of the video pipeline.

Written from the specification in docs/SPEC.md, not from the RTL. It works on
whole frames with NumPy (padding, vectorised kernels, scipy labelling), so it
shares no structure with the streaming hardware: a bug in the RTL's line
buffers, border handling or label merging cannot be reproduced here by
construction.

Frame format everywhere: YUYV 4:2:2, one pixel per 16-bit word,
word = (C << 8) | Y, C = Cb on even x and Cr on odd x.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from font8x8 import FONT8X8

# Output modes (CTRL.mode)
MODE_ORIG, MODE_GRAY, MODE_GAUSS, MODE_SOBEL, MODE_MASK_RAW, MODE_MASK = range(6)

K_BOXES = 8
TEXT_LEN = 16


@dataclass
class Regs:
    width: int = 640
    height: int = 360
    mode: int = MODE_ORIG
    overlay: bool = True          # draw boxes and text on top of the mode output
    thresh: int = 20              # motion threshold on |g - bg|
    alpha_shift: int = 4          # background EMA, alpha = 2^-k
    sobel_shift: int = 1          # sobel magnitude >> shift, saturate at 255
    min_area: int = 40            # smallest component that gets a box
    box_y: int = 145              # box colour (YCbCr), default green
    box_cb: int = 54
    box_cr: int = 34
    text: str = ""
    text_x: int = 8
    text_y: int = 8


@dataclass
class FrameResult:
    gray: np.ndarray
    gauss: np.ndarray
    sobel: np.ndarray
    mask_raw: np.ndarray
    mask: np.ndarray
    boxes: list                    # boxes found in THIS frame, drawn on the next
    out: np.ndarray                # YUYV words, uint16
    n_components: int = 0


def split_yuyv(frame):
    f = frame.astype(np.uint16)
    return (f & 0xFF).astype(np.int32), (f >> 8).astype(np.int32)


def join_yuyv(y, c):
    return ((c.astype(np.uint16) & 0xFF) << 8 | (y.astype(np.uint16) & 0xFF)).astype(np.uint16)


def rgb_to_y(rgb):
    """BT.601 studio-range luma, integer: Y = ((66R + 129G + 25B + 128) >> 8) + 16."""
    r, g, b = (rgb[..., i].astype(np.int32) for i in range(3))
    return ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16


def _windows(img):
    """All 3x3 neighbourhoods with edge replication: returns array [3,3,H,W]."""
    p = np.pad(img, 1, mode="edge")
    h, w = img.shape
    return np.stack([np.stack([p[dy:dy + h, dx:dx + w] for dx in range(3)]) for dy in range(3)])


def gauss3(img):
    k = np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]], dtype=np.int32)
    win = _windows(img.astype(np.int32))
    s = np.einsum("ij,ijhw->hw", k, win)
    return (s + 8) >> 4


def sobel(img, shift):
    kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.int32)
    win = _windows(img.astype(np.int32))
    gx = np.einsum("ij,ijhw->hw", kx, win)
    gy = np.einsum("ij,ijhw->hw", kx.T, win)
    return np.minimum(255, (np.abs(gx) + np.abs(gy)) >> shift)


def erode(mask):
    return _windows(mask.astype(np.uint8)).min(axis=(0, 1))


def dilate(mask):
    return _windows(mask.astype(np.uint8)).max(axis=(0, 1))


def motion_step(g, bg, thresh, alpha_shift):
    """One frame of background subtraction. bg is Q8.4 (None = initialise from g).

    Returns (mask_raw, new_bg).
    """
    g = g.astype(np.int64)
    if bg is None:
        return np.zeros(g.shape, dtype=np.uint8), g << 4
    diff = np.abs(g - (bg >> 4))
    mask_raw = (diff > thresh).astype(np.uint8)
    new_bg = bg + np.floor_divide((g << 4) - bg, 1 << alpha_shift)
    assert new_bg.min() >= 0 and new_bg.max() <= 4095
    return mask_raw, new_bg


def morph(mask_raw):
    """Opening (erode, dilate) followed by one more dilation to close small gaps."""
    return dilate(dilate(erode(mask_raw)))


def components(mask, min_area):
    """8-connected components; returns top-K boxes (x0, y0, x1, y1, area) and component count.

    Order: area descending, then y0, x0, y1, x1 ascending.
    """
    lab, n = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    if n == 0:
        return [], 0
    areas = np.bincount(lab.ravel())[1:]
    boxes = []
    for i, sl in enumerate(ndimage.find_objects(lab)):
        a = int(areas[i])
        if a >= min_area:
            boxes.append((sl[1].start, sl[0].start, sl[1].stop - 1, sl[0].stop - 1, a))
    boxes.sort(key=lambda b: (-b[4], b[1], b[0], b[3], b[2]))
    return boxes[:K_BOXES], n


def draw(y, c, boxes, regs):
    """Box outlines (2 px thick, inside the box) and the text string, in place."""
    h, w = y.shape
    xs = np.arange(w)[None, :]
    ys = np.arange(h)[:, None]
    hit = np.zeros((h, w), dtype=bool)
    for (x0, y0, x1, y1, _a) in boxes:
        inside = (xs >= x0) & (xs <= x1) & (ys >= y0) & (ys <= y1)
        core = (xs >= x0 + 2) & (xs <= x1 - 2) & (ys >= y0 + 2) & (ys <= y1 - 2)
        hit |= inside & ~core
    cbox = np.where(xs % 2 == 0, regs.box_cb, regs.box_cr)
    y[hit] = regs.box_y
    c[hit] = np.broadcast_to(cbox, (h, w))[hit]

    text = regs.text[:TEXT_LEN].ljust(TEXT_LEN, "\0")
    for i, ch in enumerate(text):
        code = ord(ch)
        if code < 32 or code > 127:
            continue
        glyph = FONT8X8[code - 32]
        for gy in range(8):
            row = glyph[gy]
            for gx in range(8):
                if (row >> gx) & 1:      # bit 0 is the leftmost pixel
                    px, py = regs.text_x + 8 * i + gx, regs.text_y + gy
                    if px < w and py < h:
                        y[py, px] = 235
                        c[py, px] = 128


class Pipeline:
    """Holds the state that survives between frames: background model and last boxes."""

    def __init__(self, regs: Regs):
        self.regs = regs
        self.bg = None            # Q8.4, 12 bit
        self.prev_boxes = []

    def step(self, frame):
        r = self.regs
        assert frame.shape == (r.height, r.width)
        y, c = split_yuyv(frame)
        gray = y
        g = gauss3(gray)
        sb = sobel(g, r.sobel_shift)

        mask_raw, self.bg = motion_step(g, self.bg, r.thresh, r.alpha_shift)

        mask = morph(mask_raw)
        boxes, n = components(mask, r.min_area)

        if r.mode == MODE_ORIG:
            oy, oc = y.copy(), c.copy()
        else:
            src = {MODE_GRAY: gray, MODE_GAUSS: g, MODE_SOBEL: sb,
                   MODE_MASK_RAW: mask_raw * 255, MODE_MASK: mask * 255}[r.mode]
            oy, oc = src.astype(np.int32).copy(), np.full_like(y, 128)
        if r.overlay:
            draw(oy, oc, self.prev_boxes, r)
        out = join_yuyv(oy, oc)

        res = FrameResult(gray=gray.astype(np.uint8), gauss=g.astype(np.uint8),
                          sobel=sb.astype(np.uint8), mask_raw=mask_raw, mask=mask,
                          boxes=boxes, out=out, n_components=n)
        self.prev_boxes = boxes
        return res
