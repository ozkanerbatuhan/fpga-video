# Pipeline specification

The golden model (`model/golden.py`) is written from this page; the RTL is
tested against the model.

## Frame and stream format

* 640 x 360 (run-time `SIZE` register, up to MAX_W x MAX_H = 640 x 360), YUYV 4:2:2.
* One pixel per 16-bit word: `word = (C << 8) | Y`, C = Cb on even x, Cr on odd x.
  This is ffmpeg's `yuyv422` byte order read as little-endian 16-bit words.
* Inside the pipeline: AXI4-Stream video, `tuser` = start of frame, `tlast` = end of line.
* At the DMA boundary: `tlast` = end of frame (one DMA packet per frame).

## Processing

| Stage | Definition |
|---|---|
| luma | `Y = word[7:0]` (RGB variant: `((66R + 129G + 25B + 128) >> 8) + 16`) |
| gauss | 3x3 `[1 2 1; 2 4 2; 1 2 1]`, `(sum + 8) >> 4`, edge replication |
| sobel | on gauss, `min(255, (abs(Gx) + abs(Gy)) >> sobel_shift)`, edge replication |
| motion | on gauss, background `bg` unsigned Q8.4; `mask_raw = abs(g - (bg >> 4)) > thresh`; `bg' = bg + floor(((g << 4) - bg) / 2^alpha_shift)`; init frame: `bg' = g << 4`, mask 0 |
| morph | erode 3x3 -> dilate 3x3 -> dilate 3x3 on mask_raw, edge replication |
| labelling | 8-connected components of the morph mask; boxes with area >= min_area; top 8 by (area desc, y0, x0, y1, x1 asc) |
| overlay | base image by mode; box outline 2 px inside the box, colour `BOX_COLOR` (C = Cb on even x, Cr on odd x); 16-char text, 8x8 font, Y=235 C=128; text over box over image; boxes of frame n-1 drawn on frame n |

Modes: 0 original, 1 gray (Y, C=128), 2 gauss, 3 sobel, 4 mask_raw x 255, 5 mask x 255.

## Register map (AXI4-Lite, base 0x43C0_0000)

| Offset | Name | Access | Fields (reset) |
|---|---|---|---|
| 0x00 | CTRL | RW | [2:0] mode (0), [4] overlay (1), [8] bg_reset, write 1 (self-clearing) |
| 0x04 | SIZE | RW | [10:0] width (640), [26:16] height (360) |
| 0x08 | PARAM | RW | [7:0] thresh (20), [11:8] alpha_shift (4), [17:16] sobel_shift (1) |
| 0x0C | MIN_AREA | RW | [17:0] (40) |
| 0x10 | BOX_COLOR | RW | [7:0] Y (145), [15:8] Cb (54), [23:16] Cr (34) |
| 0x14 | TEXT_POS | RW | [10:0] x (8), [26:16] y (8) |
| 0x18 | VERSION | RO | 0x5650_0100 |
| 0x20-0x2C | TEXT0..3 | RW | 16 characters, byte 0 of TEXT0 first; codes outside 32..127 are blank |
| 0x30 | STATUS | RO | [3:0] boxes, [26:16] labels used, [31] label overflow |
| 0x34 | FRAMES | RO | frames completed by the labeller |
| 0x38 | ERR | RW1C | [0] DMA packet short, [1] DMA packet long, [2] window sync, [3] tuser mid-frame, [4] tlast misplaced, [5] background overflow, [6] labeller tuser mid-frame |
| 0x3C | CYCLES | RO | first input beat to last output beat of the last frame (clock cycles) |
| 0x40+8i | BOX i (0..7) | RO | +0: [10:0] x0, [26:16] y0; +4: [10:0] x1, [26:16] y1 |
| 0x80+4i | AREA i | RO | [17:0] |

## Network protocol (UDP, little endian)

| Port | Direction | Content |
|---|---|---|
| 5000 | PC -> board | `{u16 0x5646, u16 frame, u16 line, u16 nlines, u32 t_send_us}` + nlines x 1280 bytes |
| 5001 | PC -> board | `{u16 0x5643, u16 cmd, u32 value}`, echoed back as acknowledgement (host/ctl.py) |
| 5002 | board -> PC | per processed frame: boxes, areas, PL cycles, board timestamps, counters (`result_t` in sw/app/src/main.c) |

Board address 192.168.1.10/24.
