"""Block and system simulations (Vivado xsim) with bit-exact comparison against the golden model.

    python sim/run_sim.py              all tests
    python sim/run_sim.py gauss ccl    tests whose name starts with one of the words
    python sim/run_sim.py --list

Every test writes its stimulus to sim/build/<test>/in.txt, runs the testbench,
and compares sim/build/<test>/out.txt (and boxes.txt) beat by beat with the
model, including tuser/tlast positions. Most tests run with random source
gaps and random sink back-pressure.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "tools"))

import golden as G  # noqa: E402
from yuv import read_frames  # noqa: E402

VIVADO_BIN = Path(os.environ.get("VIVADO_BIN", r"D:\AMDDesignTools\2025.2\Vivado\bin"))
BUILD = ROOT / "sim" / "build"
RTL = ["axis_skid.sv", "win3x3_filter.sv", "motion_detect.sv", "ccl_bbox.sv", "overlay.sv",
       "luma.sv", "stream_adapt.sv", "vproc_regs.sv", "vproc_top.sv", "upscale2x.sv"]
CLIP = ROOT / "clips" / "syn_busy.yuv"
# output tdata width per DUT (the TB masks the rest of its 64-bit bus)
OBITS = dict(gauss=24, sobel=24, erode=17, dilate=17, motion=25, ccl=17, overlay=16,
             luma_yuyv=24, luma_rgb=32, upscale=16, top=16)


def tool(name):
    ext = ".bat" if os.name == "nt" else ""
    return str(VIVADO_BIN / (name + ext))


# ---------------------------------------------------------------- stream files
def video_beats(frames, data_fn=lambda f: f):
    """Frames [N,H,W] -> (data, user, last) arrays in raster order."""
    n, h, w = frames.shape
    data = np.concatenate([np.asarray(data_fn(f), dtype=np.uint64).ravel() for f in frames])
    user = np.zeros((n, h, w), np.uint8)
    user[:, 0, 0] = 1
    last = np.zeros((n, h, w), np.uint8)
    last[:, :, -1] = 1
    return data, user.ravel(), last.ravel()


def write_stream(path, data, user, last):
    with open(path, "w") as f:
        f.write("\n".join(f"{d:x} {u:x} {l:x}" for d, u, l in zip(data.tolist(), user.tolist(), last.tolist())))
        f.write("\n")


def read_stream(path):
    a = np.loadtxt(path, dtype=str, ndmin=2)
    if a.size == 0:
        return np.zeros(0, np.uint64), np.zeros(0, np.uint8), np.zeros(0, np.uint8)
    # unused upper bits of the wide TB bus are undriven (leading x); any other x is an error
    data = np.array([int(x.lstrip("xX") or "0", 16) for x in a[:, 0]], dtype=np.uint64)
    return data, a[:, 1].astype(np.uint8), a[:, 2].astype(np.uint8)


def compare(name, got, exp, mask=None):
    gd, gu, gl = got
    ed, eu, el = exp
    if mask is not None:
        gd = gd & np.uint64(mask)
        ed = ed & np.uint64(mask)
    if len(gd) != len(ed):
        return f"{name}: beat count {len(gd)} != expected {len(ed)}"
    for label, g, e in (("tdata", gd, ed), ("tuser", gu, eu), ("tlast", gl, el)):
        bad = np.nonzero(g != e)[0]
        if bad.size:
            i = bad[0]
            return f"{name}: {label} mismatch at beat {i} ({bad.size} total): got {int(g[i]):x} expected {int(e[i]):x}"
    return None


# ---------------------------------------------------------------- simulator
_compiled = set()


def compile_dut(dut):
    snap = f"snap_{dut}"
    wd = BUILD / "_xsim"
    wd.mkdir(parents=True, exist_ok=True)
    if dut in _compiled:
        return wd, snap
    if "_xvlog" not in _compiled:
        srcs = [str(ROOT / "rtl" / f) for f in RTL] + [str(ROOT / "tb" / "tb_block.sv")]
        run([tool("xvlog"), "--sv", "--nolog"] + srcs, wd)
        _compiled.add("_xvlog")
    # options go through a file: the .bat wrappers split arguments at '='
    (wd / f"elab_{dut}.opt").write_text(
        f'tb_block -generic_top "DUTN={list(OBITS).index(dut)}" -s {snap} --O2 --nolog -timescale 1ns/1ps\n')
    run([tool("xelab"), "-f", f"elab_{dut}.opt"], wd)
    _compiled.add(dut)
    return wd, snap


def run(cmd, cwd):
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0 or "ERROR" in r.stdout:
        print(r.stdout[-4000:], r.stderr[-2000:])
        raise RuntimeError(f"command failed: {' '.join(cmd[:3])}")
    return r.stdout


def simulate(test, dut, data, user, last, **plus):
    wd, snap = compile_dut(dut)
    td = BUILD / test
    td.mkdir(parents=True, exist_ok=True)
    shutil.copy(ROOT / "rtl" / "font8x8.mem", wd / "font8x8.mem")
    write_stream(td / "in.txt", data, user, last)
    args = [f"IN={(td / 'in.txt').as_posix()}", f"OUT={(td / 'out.txt').as_posix()}"]
    if plus.pop("boxes", False):
        args.append(f"BOX={(td / 'boxes.txt').as_posix()}")
    plus.setdefault("OBITS", OBITS[dut])
    args += [f"{k}={v}" for k, v in plus.items()]
    opt = td / "xsim.opt"
    opt.write_text(f"{snap} -R --nolog " + " ".join(f'-testplusarg "{a}"' for a in args) + "\n")
    cmd = [tool("xsim"), "-f", str(opt)]
    t0 = time.time()
    out = run(cmd, wd)
    log = [l for l in out.splitlines() if l.startswith(("TB_DONE", "ERR=", "FRAMES=", "CYCLES=", "VERSION="))]
    return td, log, time.time() - t0


# ---------------------------------------------------------------- stimulus helpers
RNG = np.random.default_rng(12345)


def clip_frames(n, start=40):
    return read_frames(CLIP, start=start, count=n)


def rand_blobs(h, w, n, rng, density=0.5):
    m = np.zeros((h, w), np.uint8)
    for _ in range(n):
        cy, cx = rng.integers(0, h), rng.integers(0, w)
        ry, rx = rng.integers(1, max(2, h // 6)), rng.integers(1, max(2, w // 6))
        yy, xx = np.ogrid[:h, :w]
        m |= (((yy - cy) / ry) ** 2 + ((xx - cx) / rx) ** 2 <= 1).astype(np.uint8)
    noise = rng.random((h, w)) < density * 0.05
    return m | noise.astype(np.uint8)


def boxes_from_file(path):
    rows = []
    for line in Path(path).read_text().split("\n"):
        if not line.strip():
            continue
        v = [int(x) for x in line.split()]
        n, nl, ovf = v[:3]
        bx = [tuple(v[3 + 5 * i: 8 + 5 * i]) for i in range(n)]
        rows.append((n, nl, ovf, bx))
    return rows


# ---------------------------------------------------------------- tests
STRESS = dict(PV=70, PR=60)
TESTS = {}


def test(name):
    def deco(fn):
        TESTS[name] = fn
        return fn
    return deco


@test("luma_yuyv")
def t_luma_yuyv():
    f = RNG.integers(0, 65536, (2, 24, 32), dtype=np.uint64)
    d, u, l = video_beats(f)
    td, log, _ = simulate("luma_yuyv", "luma_yuyv", d, u, l, W=32, H=24, **STRESS)
    exp = ((d << np.uint64(8)) | (d & np.uint64(0xFF)), u, l)
    return compare("luma_yuyv", read_stream(td / "out.txt"), exp)


@test("luma_rgb")
def t_luma_rgb():
    rgb = RNG.integers(0, 256, (2, 24, 32, 3), dtype=np.int64)
    rgb[0, 0, 0] = (255, 255, 255)
    rgb[0, 0, 1] = (0, 0, 0)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    word = ((r << 16) | (b << 8) | g).astype(np.uint64)          # Xilinx order {R, B, G}
    d, u, l = video_beats(word)
    td, log, _ = simulate("luma_rgb", "luma_rgb", d, u, l, W=32, H=24, **STRESS)
    y = G.rgb_to_y(rgb).astype(np.uint64).ravel()
    exp = ((d << np.uint64(8)) | y, u, l)
    return compare("luma_rgb", read_stream(td / "out.txt"), exp)


def window_test(name, dut, frames, side, fn, w, h, mask=None, **plus):
    """frames: pixel planes [N,H,W]; side: [N,H,W] side data; fn: model per plane."""
    pw = 1 if dut in ("erode", "dilate") else 8
    word = (side.astype(np.uint64) << np.uint64(pw)) | frames.astype(np.uint64)
    d, u, l = video_beats(word)
    td, log, _ = simulate(name, dut, d, u, l, W=w, H=h, **plus)
    exp_planes = np.stack([fn(f) for f in frames]).astype(np.uint64)
    ed = ((side.astype(np.uint64) << np.uint64(pw)) | exp_planes).ravel()
    return compare(name, read_stream(td / "out.txt"), (ed, u, l), mask)


def gauss_case(name, w, h, n, src, **plus):
    if src == "rand":
        f = RNG.integers(0, 256, (n, h, w))
    else:
        f = (clip_frames(n) & 0xFF)[:, :h, :w]
    side = RNG.integers(0, 65536, (n, h, w))
    return window_test(name, "gauss", f, side, G.gauss3, w, h, **plus)


@test("gauss_rand")
def _():
    return gauss_case("gauss_rand", 37, 23, 3, "rand", **STRESS)


@test("gauss_tiny")
def _():
    return gauss_case("gauss_tiny", 2, 2, 3, "rand", **STRESS) or gauss_case("gauss_3x2", 3, 2, 2, "rand", **STRESS)


@test("gauss_clip")
def _():
    return gauss_case("gauss_clip", 640, 360, 2, "clip", **STRESS)


@test("gauss_noback")
def _():
    return gauss_case("gauss_noback", 640, 360, 1, "clip", PV=100, PR=100)


@test("sobel_rand")
def _():
    errs = []
    for ssh in range(4):
        f = RNG.integers(0, 256, (2, 21, 33))
        f[0, :, 16:] = 255
        f[0, :, :16] = 0                              # strong edge: saturation
        side = RNG.integers(0, 65536, f.shape)
        e = window_test(f"sobel_rand_s{ssh}", "sobel", f, side, lambda p: G.sobel(p, ssh), 33, 21, SSH=ssh, **STRESS)
        if e:
            errs.append(e)
    return "; ".join(errs) or None


@test("sobel_clip")
def _():
    f = G.gauss3(clip_frames(1)[0] & 0xFF)[None]
    side = RNG.integers(0, 65536, f.shape)
    return window_test("sobel_clip", "sobel", f, side, lambda p: G.sobel(p, 1), 640, 360, SSH=1, **STRESS)


@test("morph_rand")
def _():
    errs = []
    for dut, fn in (("erode", G.erode), ("dilate", G.dilate)):
        f = np.stack([rand_blobs(30, 41, 6, RNG), (RNG.random((30, 41)) < 0.5).astype(np.uint8),
                      np.ones((30, 41), np.uint8)])
        side = RNG.integers(0, 65536, f.shape)
        e = window_test(f"{dut}_rand", dut, f, side, fn, 41, 30, **STRESS)
        if e:
            errs.append(e)
    return "; ".join(errs) or None


@test("motion")
def _():
    n, h, w = 6, 360, 640
    g = np.stack([G.gauss3(f & 0xFF) for f in clip_frames(n)])
    side = RNG.integers(0, 65536, g.shape)
    word = (side.astype(np.uint64) << np.uint64(8)) | g.astype(np.uint64)
    d, u, l = video_beats(word)
    td, log, _ = simulate("motion", "motion", d, u, l, W=w, H=h, THR=20, ALPHA=4, BGRESET=4, **STRESS)
    bg, masks = None, []
    for i in range(n):
        if i == 4:
            bg = None
        m, bg = G.motion_step(g[i], bg, 20, 4)
        masks.append(m)
    ed = ((word.ravel() << np.uint64(1)) | np.stack(masks).astype(np.uint64).ravel())
    return compare("motion", read_stream(td / "out.txt"), (ed, u, l))


def ccl_case(name, masks, w, h, mina=1, expect_overflow=False, **plus):
    side = RNG.integers(0, 65536, masks.shape)
    word = (side.astype(np.uint64) << np.uint64(1)) | masks.astype(np.uint64)
    d, u, l = video_beats(word)
    td, log, _ = simulate(name, "ccl", d, u, l, W=w, H=h, MINA=mina, boxes=True, **plus)
    e = compare(name, read_stream(td / "out.txt"), (word.ravel(), u, l))
    if e:
        return e
    rows = boxes_from_file(td / "boxes.txt")
    if len(rows) != len(masks):
        return f"{name}: {len(rows)} published frames, expected {len(masks)}"
    for i, (m, (n, nl, ovf, bx)) in enumerate(zip(masks, rows)):
        if expect_overflow:
            if not ovf:
                return f"{name}: frame {i} overflow flag not set"
            continue
        if ovf:
            return f"{name}: frame {i} unexpected overflow ({nl} labels)"
        exp, ncomp = G.components(m, mina)
        if list(bx) != list(exp):
            return f"{name}: frame {i} boxes\n  got {bx}\n  exp {exp}"
    return None


@test("ccl_rand")
def _():
    masks = np.stack([rand_blobs(60, 80, k, RNG) for k in (3, 8, 15, 25)] + [np.zeros((60, 80), np.uint8)])
    return ccl_case("ccl_rand", masks, 80, 60, mina=1, **STRESS)


@test("ccl_shapes")
def _():
    h, w = 48, 64
    m = np.zeros((h, w), np.uint8)
    m[5:30, 5] = m[5:30, 15] = m[29, 5:16] = 1          # U: merges at the bottom
    m[5:20, 20:40] = 1
    m[8:17, 24:36] = 0                                  # ring
    m[25:45, 45:47] = 1
    for i in range(14):                                 # staircase diagonal (8-connectivity only)
        m[30 + i, 22 + i] = 1
    m[0, :] = 1                                         # full top row
    m[:, w - 1] = 1                                     # full right column
    zig = np.zeros((h, w), np.uint8)                    # many merges in one row
    zig[10:40, 2:62:2] = 1
    zig[40, 2:61] = 1
    spiral = np.zeros((h, w), np.uint8)
    x0, y0, x1, y1 = 2, 2, w - 3, h - 3
    while x0 < x1 and y0 < y1:
        spiral[y0, x0:x1 + 1] = 1
        spiral[y0:y1 + 1, x1] = 1
        spiral[y1, x0:x1 + 1] = 1
        spiral[y0 + 2:y1 + 1, x0] = 1
        x0, y0, x1, y1 = x0 + 2, y0 + 2, x1 - 2, y1 - 2
        if x0 < x1:
            spiral[y0, x0 - 2:x0 + 1] = 1
    full = np.ones((h, w), np.uint8)
    checker = (np.indices((h, w)).sum(0) % 2).astype(np.uint8)   # one 8-connected component
    return ccl_case("ccl_shapes", np.stack([m, zig, spiral, full, checker]), w, h, mina=1, **STRESS)


@test("ccl_clip")
def _():
    p = G.Pipeline(G.Regs())
    masks = []
    for f in clip_frames(4):
        masks.append(p.step(f).mask)
    return ccl_case("ccl_clip", np.stack(masks[1:]), 640, 360, mina=40, **STRESS)


@test("ccl_overflow")
def _():
    m = np.zeros((100, 100), np.uint8)
    m[::2, ::2] = 1                                     # 2500 isolated pixels > 2047 labels
    return ccl_case("ccl_overflow", m[None], 100, 100, mina=1, expect_overflow=True)


@test("overlay")
def _():
    errs = []
    h, w = 40, 150
    n = 2
    data = RNG.integers(0, 2 ** 34, (n, h, w), dtype=np.uint64)
    boxes = [(3, 4, 30, 20, 0), (40, 0, 41, 39, 0), (100, 10, 149, 12, 0), (60, 30, 61, 31, 0)]
    bf = BUILD / "overlay_boxes.txt"
    BUILD.mkdir(parents=True, exist_ok=True)
    bf.write_text(f"{len(boxes)}\n" + "\n".join(f"{b[0]} {b[1]} {b[2]} {b[3]}" for b in boxes) + "\n")
    text = "Hi_OBJ:3_az09.Z"
    for mode in range(6):
        for ovl in (0, 1):
            name = f"overlay_m{mode}_o{ovl}"
            d, u, l = video_beats(data)
            td, log, _ = simulate(name, "overlay", d, u, l, W=w, H=h, MODE=mode, OVL=ovl,
                                  TX=5, TY=3, TEXT=text, OVB=bf.as_posix(), **STRESS)
            regs = G.Regs(width=w, height=h, mode=mode, overlay=bool(ovl), text=text.replace("_", " "),
                          text_x=5, text_y=3, box_y=0x91, box_cb=0x36, box_cr=0x22)
            exp = []
            for f in data:
                orig = (f >> np.uint64(18)) & np.uint64(0xFFFF)
                sob = ((f >> np.uint64(10)) & np.uint64(0xFF)).astype(np.int32)
                gau = ((f >> np.uint64(2)) & np.uint64(0xFF)).astype(np.int32)
                mraw = ((f >> np.uint64(1)) & np.uint64(1)).astype(np.int32)
                msk = (f & np.uint64(1)).astype(np.int32)
                y, c = G.split_yuyv(orig)
                if mode == 0:
                    oy, oc = y.copy(), c.copy()
                else:
                    src = {1: y, 2: gau, 3: sob, 4: mraw * 255, 5: msk * 255}[mode]
                    oy, oc = src.copy(), np.full_like(y, 128)
                if ovl:
                    G.draw(oy, oc, boxes, regs)
                exp.append(G.join_yuyv(oy, oc))
            e = compare(name, read_stream(td / "out.txt"), video_beats(np.stack(exp)))
            if e:
                errs.append(e)
    return "; ".join(errs) or None


@test("upscale")
def _():
    f = RNG.integers(0, 65536, (2, 4, 640), dtype=np.uint64)
    d, u, l = video_beats(f)
    td, log, _ = simulate("upscale", "upscale", d, u, l, **STRESS)
    exp = []
    for fr in f:
        out = np.zeros((8, 1280), np.uint64)
        for y in range(4):
            for j in range(1280):
                x = j >> 1
                yv = fr[y, x] & np.uint64(0xFF)
                cb = fr[y, x & ~1] >> np.uint64(8)
                cr = fr[y, x | 1] >> np.uint64(8)
                out[2 * y, j] = out[2 * y + 1, j] = ((cr if j & 1 else cb) << np.uint64(8)) | yv
        exp.append(out)
    return compare("upscale", read_stream(td / "out.txt"), video_beats(np.stack(exp)))


def top_case(name, n, mode, ovl, **plus):
    frames = clip_frames(n, start=60)
    h, w = frames.shape[1:]
    data = frames.astype(np.uint64).ravel()
    user = np.zeros(data.size, np.uint8)
    last = np.zeros((n, h * w), np.uint8)
    last[:, -1] = 1                                     # DMA packet: tlast at end of frame only
    text = "F_OBJ_TEST"
    td, log, secs = simulate(name, "top", data, user, last.ravel(), W=w, H=h, MODE=mode, OVL=ovl,
                             THR=20, ALPHA=4, SSH=1, MINA=40, TX=8, TY=8, TEXT=text, boxes=True, **plus)
    regs = G.Regs(width=w, height=h, mode=mode, overlay=bool(ovl), text=text.replace("_", " "))
    pipe = G.Pipeline(regs)
    res = [pipe.step(f) for f in frames]
    exp_last = np.zeros((n, h * w), np.uint8)
    exp_last[:, -1] = 1
    exp = (np.stack([r.out for r in res]).astype(np.uint64).ravel(), np.zeros(n * h * w, np.uint8), exp_last.ravel())
    e = compare(name, read_stream(td / "out.txt"), exp)
    if e:
        return e
    rows = boxes_from_file(td / "boxes.txt")
    for i, (r, (nb, nl, ovf, bx)) in enumerate(zip(res, rows)):
        if ovf or list(bx) != list(r.boxes):
            return f"{name}: frame {i} boxes got {bx} exp {r.boxes}"
    info = dict(x.split("=", 1) for x in log if not x.startswith("TB_DONE"))
    if int(info.get("ERR", "1"), 16) != 0:
        return f"{name}: error register {info.get('ERR')}"
    cyc = int(info.get("CYCLES", 0))
    print(f"    {name}: {n} frames, sim {secs:.0f} s, last frame {cyc} {cyc / 148.5e3:.2f} ms @148.5 MHz (in SOF -> out EOF)")
    return None


@test("top_boxes")
def _():
    return top_case("top_boxes", 6, 0, 1, **STRESS)


@test("top_modes")
def _():
    for mode in (3, 5):
        e = top_case(f"top_mode{mode}", 3, mode, 1, PV=100, PR=100)
        if e:
            return e
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("only", nargs="*")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        print("\n".join(TESTS))
        return 0
    subprocess.run([sys.executable, str(ROOT / "tools" / "gen_mem.py")], check=True)
    names = [t for t in TESTS if not a.only or any(t.startswith(o) for o in a.only)]
    fails = 0
    for t in names:
        t0 = time.time()
        try:
            err = TESTS[t]()
        except Exception as ex:  # noqa: BLE001
            err = f"{t}: exception {ex}"
        status = "PASS" if err is None else "FAIL"
        print(f"[{status}] {t} ({time.time() - t0:.0f} s)" + ("" if err is None else f"\n    {err}"))
        fails += err is not None
    print(f"{len(names) - fails}/{len(names)} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
