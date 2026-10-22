/* Register maps of the PL blocks used by the firmware.
 * Base addresses come from the block design (hw/build.tcl, assign_bd_address);
 * they are checked against the .xsa by sw/check_addr.py during the build. */
#ifndef VIDEO_HW_H
#define VIDEO_HW_H

#include "xil_io.h"
#include "xil_types.h"

#define VPROC_BASE   0x43C00000U
#define DMA_BASE     0x40400000U
#define VDMA_BASE    0x43000000U
#define IIC_BASE     0x41600000U
#define GPIO_BASE    0x41200000U

/* frame geometry (pipeline maximum, fixed by the protocol) */
#define FRAME_W      640
#define FRAME_H      360
#define LINE_BYTES   (FRAME_W * 2)
#define FRAME_BYTES  (LINE_BYTES * FRAME_H)

/* ---- vproc (docs/SPEC.md) ---- */
#define VP_CTRL      0x00   /* [2:0] mode, [4] overlay, [8] bg_reset (W1) */
#define VP_SIZE      0x04
#define VP_PARAM     0x08   /* [7:0] thresh, [11:8] alpha_shift, [17:16] sobel_shift */
#define VP_MIN_AREA  0x0C
#define VP_COLOR     0x10
#define VP_TPOS      0x14
#define VP_VERSION   0x18
#define VP_TEXT0     0x20
#define VP_STATUS    0x30   /* [3:0] n_boxes, [26:16] labels, [31] overflow */
#define VP_FRAMES    0x34
#define VP_ERR       0x38
#define VP_CYCLES    0x3C
#define VP_BOX(i)    (0x40 + 8 * (i))
#define VP_AREA(i)   (0x80 + 4 * (i))

/* ---- AXI DMA, simple mode ---- */
#define DMA_MM2S_CR  0x00
#define DMA_MM2S_SR  0x04
#define DMA_MM2S_SA  0x18
#define DMA_MM2S_LEN 0x28
#define DMA_S2MM_CR  0x30
#define DMA_S2MM_SR  0x34
#define DMA_S2MM_DA  0x48
#define DMA_S2MM_LEN 0x58
#define DMA_SR_HALTED 0x1
#define DMA_SR_IDLE   0x2
#define DMA_SR_ERRS   0x770

/* ---- AXI VDMA, MM2S ---- */
#define VDMA_MM2S_CR     0x00
#define VDMA_MM2S_SR     0x04
#define VDMA_PARK_PTR    0x28
#define VDMA_MM2S_VSIZE  0x50
#define VDMA_MM2S_HSIZE  0x54
#define VDMA_MM2S_STRIDE 0x58
#define VDMA_MM2S_ADDR(i) (0x5C + 4 * (i))

/* ---- AXI GPIO ---- */
#define GPIO_DATA    0x00
#define GPIO_TRI     0x04

static inline void wr(u32 base, u32 off, u32 v) { Xil_Out32(base + off, v); }
static inline u32  rd(u32 base, u32 off)        { return Xil_In32(base + off); }

#endif
