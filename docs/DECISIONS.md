# Design decisions — with "what's your guess?" questions

Each block below starts with the question to answer **before** opening the
answer. Answers give the number that was actually measured or chosen, and why.
Mark your guess next to each question, then compare.

---

## 0. System split

**Q0.1** Why not decode the camera's H.264 on the Zynq's ARM cores?
<details><summary>Answer</summary>
Two Cortex-A9 at 667 MHz decode 640x360 H.264 in software, but not 1080p at
25 fps, and the decode would eat the CPU needed for the network stack. The
PC decodes (ffmpeg, sub-stream) and sends raw YUYV; the board does what an
FPGA is good at (fixed per-pixel work, streaming). In a real gimbal the
sensor delivers raw pixels, so the FPGA side of this design is the
representative part.
</details>

**Q0.2** Raw 640x360 YUYV at 25 fps — how many Mbit/s on the wire?
<details><summary>Answer</summary>
640 x 360 x 2 B x 25 = 11.5 MB/s = **92 Mbit/s** of payload (+ 1 % headers).
Fast Ethernet (100 M) is not enough; the board must link at 1 Gbit/s.
Gray only would be 46 Mbit/s, but the overlay draws on the colour image.
</details>

**Q0.3** Where is the clock-domain crossing in the whole design?
<details><summary>Answer</summary>
Exactly one data CDC: inside Xilinx `v_axi4s_vid_out` (async FIFO between
the 148.5 MHz processing clock and the 74.25 MHz pixel clock). Everything
else (DMA, VDMA, pipeline, upscaler) runs on one clock. The GPIO control bits
going into the pixel domain are static and double-synchronised.
</details>

---

## 1. Window generator (shared by Gauss, Sobel, erode, dilate)

**Q1.1** How many line buffers does a 3x3 window need, and what is the
output latency in lines?
<details><summary>Answer</summary>
**Two** line buffers (rows r-1 and r-2; row r is the incoming pixel). The
output for row y is produced while row y+1 arrives: latency = 1 line + 1
pixel (+ 3 pipeline registers).
</details>

**Q1.2** 640 pixels x 8 bit per line buffer. How many BRAM18 for the Gauss
stage? (Hint: the side data travels in the buffer too.)
<details><summary>Answer</summary>
Gauss carries 16 bit of side data (the original YUYV pixel) with the centre
pixel: lb0 is 640 x 24 bit, lb1 is 640 x 8 bit. A BRAM18 holds 1K x 18 or
2K x 9: lb0 = 2 BRAM18 (24 bit wide > 18), lb1 = 1 BRAM18. Check the
hierarchical utilisation report in hw/reports/utilization_hier.txt.
</details>

**Q1.3** Borders: the frame has no row -1. What does the hardware do, and
how does it finish the last row without waiting for the next frame?
<details><summary>Answer</summary>
Replicate (clamp-to-edge), same as `np.pad(mode="edge")` in the model. The
frame is walked as (H+1) x (W+1) steps; the extra column and row are flush
steps that need no input pixel. Cost: W+H+1 idle cycles per frame (< 0.5 %).
</details>

**Q1.4** Pipeline depth of the window filter, and what happens on
back-pressure?
<details><summary>Answer</summary>
Issue (BRAM read) -> column/shift -> kernel -> output register -> skid
buffer. One enable freezes every stage, including the BRAM read registers,
so a stall never loses or duplicates data. The skid buffer makes tready a
register, so tready paths do not chain combinationally through the 9 blocks.
</details>

---

## 2. Gauss 3x3

**Q2.1** Kernel [1 2 1; 2 4 2; 1 2 1]/16 on 8-bit input: accumulator width?
Rounding?
<details><summary>Answer</summary>
Sum max = 16 x 255 = 4080 -> **12 bit**. Output = (sum + 8) >> 4, round half
up. No multipliers: shifts and adds (0 DSP).
</details>

## 3. Sobel

**Q3.1** |Gx| + |Gy| maximum on 8-bit input? How is it brought to 8 bit?
<details><summary>Answer</summary>
|Gx| <= 4 x 255 = 1020, so |Gx|+|Gy| <= 2040 (11 bit). Output =
min(255, (|Gx|+|Gy|) >> sobel_shift), shift set at run time (default 1).
L1 norm instead of sqrt(Gx^2+Gy^2): no multipliers, standard on FPGAs.
</details>

## 4. Motion detection

**Q4.1** Background model at 640x360 in Q8.4 — on-chip BRAM or DDR? How many
BRAM36 does it cost?
<details><summary>Answer</summary>
On chip. 230 400 px x 12 bit = 2.76 Mbit; the XC7Z020 has 140 BRAM36
(4.9 Mbit). Vivado maps it at roughly 4.5 bit per... see the measured count in
`utilization_hier.txt` (u_motion). DDR would need a second DMA stream that
reads and writes the background in lock-step with the video, plus 2x the
memory traffic; BRAM costs nothing in bandwidth and is deterministic.
</details>

**Q4.2** Why Q8.4 and not 8-bit integer background?
<details><summary>Answer</summary>
With alpha = 1/16 and an 8-bit background, bg += (g - bg) >> 4 stops moving
when |g - bg| < 16: a 15-level dead zone, bigger than the threshold.
4 fractional bits shrink the dead zone to < 1 level.
</details>

**Q4.3** Why is the background update `>>>` (arithmetic shift, floor) and
not division?
<details><summary>Answer</summary>
Free in hardware, and the model reproduces it exactly with
`np.floor_divide`. It biases the background down by < 1/16 level per frame;
irrelevant against a threshold of 20.
</details>

## 5. Morphology and labelling

**Q5.1** Why erode -> dilate -> dilate and not just open?
<details><summary>Answer</summary>
Erode removes isolated noise pixels; the first dilate restores object size
(opening); the second dilate closes 1-pixel gaps so an object split by the
threshold becomes one component. Chosen on the clips with the golden model.
</details>

**Q5.2** 8-connected labelling looks at 4 neighbours. At most how many
*different* components can meet at one pixel?
<details><summary>Answer</summary>
**Two.** Left, up-left and up-right all touch "up"; if up is set, everything
is already one component. If up is clear, up-right may differ from
left/up-left. So one union per pixel at most.
</details>

**Q5.3** How many provisional labels are needed, and what happens if they run
out?
<details><summary>Answer</summary>
2047 (11-bit labels). The worst case on the clips was ~30 per frame after
morphology. On overflow new pixels get no label and the `overflow` flag is set
(tested: 2500 isolated pixels).
</details>

**Q5.4** The labeller takes several cycles per foreground pixel. Is that a
throughput problem?
<details><summary>Answer</summary>
No at this frame rate: measured 246 k cycles for a 230 k pixel frame
(1.07 cycles/pixel, 1.66 ms at 148.5 MHz), budget at 25 fps is 40 ms. It
back-pressures upstream; worst case (all foreground) is about 8 cycles/pixel
= 12 ms.
</details>

## 6. Overlay

**Q6.1** The boxes drawn on frame n come from which frame?
<details><summary>Answer</summary>
Frame n-1. Boxes are known only after the last pixel of a frame; drawing them
on the same frame would need a full frame buffer. One frame (40 ms) of box lag.
</details>

## 7. Clocks and timing

**Q7.1** Why 148.5 MHz for the processing clock?
<details><summary>Answer</summary>
It is the 1080p60 pixel clock: the window blocks run at 1 pixel/clock, so the
same pipeline could take 1080p60 directly from a sensor. Both 148.5 and
74.25 MHz come from one MMCM (VCO 742.5 MHz = 100 MHz x 37.125 / 5).
</details>
