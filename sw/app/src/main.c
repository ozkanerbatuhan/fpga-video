/* Bare-metal firmware for the ZedBoard video pipeline.
 *
 *   PC --UDP 5000--> lwIP --> input frame buffer (DDR)
 *                         --> AXI DMA MM2S --> PL pipeline --> AXI DMA S2MM
 *                         --> display buffer --> VDMA --> upscaler --> HDMI
 *   board --UDP 5002--> PC : boxes, timestamps, statistics per frame
 *   PC --UDP 5001--> board : run-time control (mode, thresholds, ...)
 *
 * Three input buffers (filling / ready / in the DMA) and three display
 * buffers (VDMA park pointer flips to the newest finished one).
 */
#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xiic_l.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/etharp.h"

#include "platform.h"
#include "platform_config.h"
#include "video_hw.h"

/* ------------------------------------------------------------------ config */
#define PORT_VIDEO   5000
#define PORT_CTRL    5001
#define PORT_RESULT  5002

#define MAGIC_VIDEO  0x5646   /* "VF" */
#define MAGIC_CTRL   0x5643   /* "VC" */
#define MAGIC_RESULT 0x5652   /* "VR" */

/* defined and set by the timer callback in the template's platform_zynq.c */
extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;

static struct netif netif_s;
struct netif *echo_netif = &netif_s;     /* platform_zynq.c checks the link on it */
static unsigned char mac_addr[6] = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x02};

/* ------------------------------------------------------------------ time */
#define GT_BASE  0xF8F00200U
#define GT_HZ    (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2U)
static u64 gt_now(void)
{
    u32 hi, lo;
    do {
        hi = Xil_In32(GT_BASE + 4);
        lo = Xil_In32(GT_BASE + 0);
    } while (Xil_In32(GT_BASE + 4) != hi);
    return ((u64)hi << 32) | lo;
}
static u32 now_us(void) { return (u32)(gt_now() / (GT_HZ / 1000000U)); }

/* ------------------------------------------------------------------ buffers */
static u8 in_buf[3][FRAME_BYTES]  __attribute__((aligned(64)));
static u8 out_buf[3][FRAME_BYTES] __attribute__((aligned(64)));

typedef struct {
    u16 frame;
    u16 lines;
    u32 t_send;            /* PC timestamp of the first packet (echoed back) */
    u32 t_first, t_last;   /* board receive times */
    int state;             /* 0 free, 1 filling, 2 ready, 3 in DMA */
} inframe_t;
static inframe_t inf[3];
static int fill_idx = -1, ready_idx = -1, dma_idx = -1;
static int disp_idx = 0, next_out = 1;
static int dma_busy = 0;
static u32 t_kick;

/* statistics */
static u32 st_rx_frames, st_proc_frames, st_dropped, st_lost_lines, st_pkts;

/* result destination: whoever sent video last */
static ip_addr_t peer_ip;
static int peer_valid = 0;
static struct udp_pcb *pcb_result;

/* control state */
static u32 ctrl_mode = 0, ctrl_overlay = 1, ctrl_auto_text = 1;
static u32 gpio_val = 0;

/* ------------------------------------------------------------------ ADV7511 */
#define ADV_ADDR 0x39

static int adv_write(u8 reg, u8 val)
{
    u8 b[2] = {reg, val};
    return XIic_Send(IIC_BASE, ADV_ADDR, b, 2, XIIC_STOP) == 2 ? 0 : -1;
}

static int adv_read(u8 reg, u8 *val)
{
    if (XIic_Send(IIC_BASE, ADV_ADDR, &reg, 1, XIIC_REPEATED_START) != 1) return -1;
    return XIic_Recv(IIC_BASE, ADV_ADDR, val, 1, XIIC_STOP) == 1 ? 0 : -1;
}

/* Settings follow the ZedBoard ADV7511 configuration used by Analog Devices
 * (Linux DT zynq-zed-adv7511.dtsi: yuv422, 8 bit, 1x clock, style 1, right
 * justified) and the register writes of the Linux adv7511 driver. */
static int adv7511_init(void)
{
    static const u8 fixed[][2] = {
        {0x98, 0x03}, {0x9A, 0xE0}, {0x9C, 0x30}, {0x9D, 0x61},
        {0xA2, 0xA4}, {0xA3, 0xA4}, {0xE0, 0xD0}, {0xF9, 0x00}, {0x55, 0x02},
    };
    /* YCbCr -> RGB, scaling x4 */
    static const u16 csc[12] = {
        0x0734, 0x04ad, 0x0000, 0x1c1b,
        0x1ddc, 0x04ad, 0x1f24, 0x0135,
        0x0000, 0x04ad, 0x087c, 0x1b77,
    };
    u8 id_hi = 0, id_lo = 0, v;
    int i, err = 0;

    if (adv_read(0xF5, &id_hi) || adv_read(0xF6, &id_lo)) {
        xil_printf("ADV7511: no I2C answer\r\n");
        return -1;
    }
    xil_printf("ADV7511: chip id %02x%02x\r\n", id_hi, id_lo);

    err |= adv_write(0xD6, 0xC0);          /* HPD always high (no hot-plug reset) */
    err |= adv_write(0x41, 0x10);          /* power up */
    for (i = 0; i < (int)(sizeof fixed / sizeof fixed[0]); i++)
        err |= adv_write(fixed[i][0], fixed[i][1]);
    err |= adv_write(0x15, 0x01);          /* input ID 1: YCbCr 4:2:2, separate syncs */
    err |= adv_write(0x16, 0x38);          /* out 4:4:4, 8 bit, style 1, out RGB */
    err |= adv_write(0x17, 0x02);          /* 16:9, sync polarity as input (high) */
    err |= adv_write(0x48, 0x08);          /* right justified */
    err |= adv_write(0xBA, 0x60);          /* clock delay 0 */
    err |= adv_write(0xD0, 0x0C);          /* no sync pulse generation */

    err |= adv_write(0x1A, 0x20);          /* CSC update mode on */
    for (i = 0; i < 12; i++) {
        u8 hi = (u8)(csc[i] >> 8) & 0x1F;
        if (i == 0) hi |= 0x80 | (2 << 5); /* enable, scaling 4 */
        if (i == 1) hi |= 0x20;            /* keep update mode */
        err |= adv_write(0x18 + 2 * i, hi);
        err |= adv_write(0x19 + 2 * i, (u8)csc[i]);
    }
    err |= adv_write(0x1A, (u8)(csc[1] >> 8) & 0x1F);   /* update mode off */

    if (!adv_read(0xAF, &v)) err |= adv_write(0xAF, v | 0x02);   /* HDMI mode */
    if (!adv_read(0x42, &v)) xil_printf("ADV7511: status 0x42=%02x (bit6 HPD, bit5 monitor sense)\r\n", v);
    return err;
}

/* ------------------------------------------------------------------ display */
static void fill_bars(u8 *buf)
{
    /* 8 colour bars, YCbCr 75%: white yellow cyan green magenta red blue black */
    static const u8 bars[8][3] = {
        {180, 128, 128}, {162, 44, 142}, {131, 156, 44}, {112, 72, 58},
        {84, 184, 198},  {65, 100, 212}, {35, 212, 114}, {16, 128, 128}};
    int x, y;
    for (y = 0; y < FRAME_H; y++)
        for (x = 0; x < FRAME_W; x++) {
            const u8 *b = bars[x * 8 / FRAME_W];
            u8 *p = buf + y * LINE_BYTES + 2 * x;
            p[0] = b[0];
            p[1] = (x & 1) ? b[2] : b[1];
        }
    Xil_DCacheFlushRange((UINTPTR)buf, FRAME_BYTES);
}

static void vdma_init(void)
{
    int i;
    wr(VDMA_BASE, VDMA_MM2S_CR, 0x4);                 /* reset */
    while (rd(VDMA_BASE, VDMA_MM2S_CR) & 0x4) ;
    wr(VDMA_BASE, VDMA_MM2S_CR, 0x1);                 /* run, park mode */
    for (i = 0; i < 3; i++)
        wr(VDMA_BASE, VDMA_MM2S_ADDR(i), (u32)(UINTPTR)out_buf[i]);
    wr(VDMA_BASE, VDMA_PARK_PTR, 0);
    wr(VDMA_BASE, VDMA_MM2S_STRIDE, LINE_BYTES);
    wr(VDMA_BASE, VDMA_MM2S_HSIZE, LINE_BYTES);
    wr(VDMA_BASE, VDMA_MM2S_VSIZE, FRAME_H);          /* starts the channel */
}

static void vdma_show(int idx)
{
    wr(VDMA_BASE, VDMA_PARK_PTR, (u32)idx);
}

/* ------------------------------------------------------------------ pipeline */
static void vproc_text(u16 frame, u32 nboxes)
{
    char t[17];
    u32 w[4];
    int i;
    memset(t, 0, sizeof t);
    snprintf(t, sizeof t, "F%05u OBJ %u", (unsigned)frame, (unsigned)nboxes);
    for (i = 0; i < 4; i++)
        w[i] = (u32)(u8)t[4 * i] | (u32)(u8)t[4 * i + 1] << 8 |
               (u32)(u8)t[4 * i + 2] << 16 | (u32)(u8)t[4 * i + 3] << 24;
    for (i = 0; i < 4; i++)
        wr(VPROC_BASE, VP_TEXT0 + 4 * i, w[i]);
}

static void dma_reset(void)
{
    wr(DMA_BASE, DMA_MM2S_CR, 0x4);
    wr(DMA_BASE, DMA_S2MM_CR, 0x4);
    while (rd(DMA_BASE, DMA_MM2S_CR) & 0x4) ;
    while (rd(DMA_BASE, DMA_S2MM_CR) & 0x4) ;
    wr(DMA_BASE, DMA_MM2S_CR, 0x1);
    wr(DMA_BASE, DMA_S2MM_CR, 0x1);
}

static void kick(int idx)
{
    u8 *src = in_buf[idx], *dst = out_buf[next_out];
    static u32 last_boxes = 0;

    if (ctrl_auto_text)
        vproc_text(inf[idx].frame, last_boxes);
    Xil_DCacheFlushRange((UINTPTR)src, FRAME_BYTES);
    Xil_DCacheInvalidateRange((UINTPTR)dst, FRAME_BYTES);

    wr(DMA_BASE, DMA_S2MM_DA, (u32)(UINTPTR)dst);
    wr(DMA_BASE, DMA_S2MM_LEN, FRAME_BYTES);
    wr(DMA_BASE, DMA_MM2S_SA, (u32)(UINTPTR)src);
    wr(DMA_BASE, DMA_MM2S_LEN, FRAME_BYTES);
    inf[idx].state = 3;
    dma_idx = idx;
    dma_busy = 1;
    t_kick = now_us();
    last_boxes = rd(VPROC_BASE, VP_STATUS) & 0xF;
}

typedef struct __attribute__((packed)) {
    u16 magic, frame;
    u32 pl_cycles;
    u32 t_send;              /* PC clock, echoed */
    u32 t_first, t_last;     /* board clock (us) */
    u32 t_kick, t_done;
    u16 lines, n_boxes;
    u16 box[8][4];
    u32 area[8];
    u32 rx_frames, proc_frames, dropped, lost_lines;
    u32 err;
} result_t;

static void finish(void)
{
    result_t r;
    struct pbuf *p;
    int i;
    u32 st = rd(VPROC_BASE, VP_STATUS);

    memset(&r, 0, sizeof r);
    r.magic = MAGIC_RESULT;
    r.frame = inf[dma_idx].frame;
    r.pl_cycles = rd(VPROC_BASE, VP_CYCLES);
    r.t_send = inf[dma_idx].t_send;
    r.t_first = inf[dma_idx].t_first;
    r.t_last = inf[dma_idx].t_last;
    r.t_kick = t_kick;
    r.t_done = now_us();
    r.lines = inf[dma_idx].lines;
    r.n_boxes = st & 0xF;
    for (i = 0; i < 8; i++) {
        u32 a = rd(VPROC_BASE, VP_BOX(i)), b = rd(VPROC_BASE, VP_BOX(i) + 4);
        r.box[i][0] = a & 0x7FF; r.box[i][1] = a >> 16;
        r.box[i][2] = b & 0x7FF; r.box[i][3] = b >> 16;
        r.area[i] = rd(VPROC_BASE, VP_AREA(i));
    }
    r.rx_frames = st_rx_frames;
    r.proc_frames = ++st_proc_frames;
    r.dropped = st_dropped;
    r.lost_lines = st_lost_lines;
    r.err = rd(VPROC_BASE, VP_ERR);

    /* page flip: show the frame just written */
    disp_idx = next_out;
    vdma_show(disp_idx);
    next_out = (next_out + 1) % 3;

    inf[dma_idx].state = 0;
    dma_idx = -1;
    dma_busy = 0;
    gpio_val ^= 1 << 3;                       /* LD2 toggles per processed frame */
    wr(GPIO_BASE, GPIO_DATA, gpio_val);

    if (peer_valid && (p = pbuf_alloc(PBUF_TRANSPORT, sizeof r, PBUF_RAM)) != NULL) {
        memcpy(p->payload, &r, sizeof r);
        udp_sendto(pcb_result, p, &peer_ip, PORT_RESULT);
        pbuf_free(p);
    }
}

static void complete_fill(void)
{
    if (fill_idx < 0) return;
    st_rx_frames++;
    st_lost_lines += FRAME_H - inf[fill_idx].lines;
    if (ready_idx >= 0) {                     /* newer frame replaces an unprocessed one */
        inf[ready_idx].state = 0;
        st_dropped++;
    }
    inf[fill_idx].state = 2;
    ready_idx = fill_idx;
    fill_idx = -1;
    gpio_val ^= 1 << 2;                       /* LD1 toggles per received frame */
}

static int free_buffer(void)
{
    int i;
    for (i = 0; i < 3; i++)
        if (inf[i].state == 0) return i;
    return -1;
}

/* ------------------------------------------------------------------ UDP */
typedef struct __attribute__((packed)) {
    u16 magic, frame, line, nlines;
    u32 t_send;
} vhdr_t;

static void on_video(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                     const ip_addr_t *addr, u16_t port)
{
    vhdr_t h;
    u32 bytes;
    (void)arg; (void)pcb; (void)port;

    st_pkts++;
    if (p->tot_len < sizeof h) goto out;
    pbuf_copy_partial(p, &h, sizeof h, 0);
    bytes = (u32)h.nlines * LINE_BYTES;
    if (h.magic != MAGIC_VIDEO || h.line + h.nlines > FRAME_H || p->tot_len < sizeof h + bytes)
        goto out;

    ip_addr_copy(peer_ip, *addr);
    peer_valid = 1;

    if (fill_idx >= 0 && inf[fill_idx].frame != h.frame)
        complete_fill();                      /* previous frame ended with lines missing */
    if (fill_idx < 0) {
        fill_idx = free_buffer();
        if (fill_idx < 0) { st_dropped++; goto out; }
        inf[fill_idx].frame = h.frame;
        inf[fill_idx].lines = 0;
        inf[fill_idx].t_send = h.t_send;
        inf[fill_idx].t_first = now_us();
        inf[fill_idx].state = 1;
    }
    pbuf_copy_partial(p, in_buf[fill_idx] + (u32)h.line * LINE_BYTES, bytes, sizeof h);
    inf[fill_idx].lines += h.nlines;
    inf[fill_idx].t_last = now_us();
    if (inf[fill_idx].lines >= FRAME_H)
        complete_fill();
out:
    pbuf_free(p);
}

typedef struct __attribute__((packed)) {
    u16 magic, cmd;
    u32 val;
} ctrl_t;

static void apply_ctrl(u32 cmd, u32 val)
{
    u32 v;
    switch (cmd) {
    case 1: ctrl_mode = val & 7; break;
    case 2: ctrl_overlay = val & 1; break;
    case 3: v = rd(VPROC_BASE, VP_PARAM); wr(VPROC_BASE, VP_PARAM, (v & ~0xFFu) | (val & 0xFF)); break;
    case 4: v = rd(VPROC_BASE, VP_PARAM); wr(VPROC_BASE, VP_PARAM, (v & ~0xF00u) | ((val & 0xF) << 8)); break;
    case 5: v = rd(VPROC_BASE, VP_PARAM); wr(VPROC_BASE, VP_PARAM, (v & ~0x30000u) | ((val & 3) << 16)); break;
    case 6: wr(VPROC_BASE, VP_MIN_AREA, val); break;
    case 7: wr(VPROC_BASE, VP_CTRL, ctrl_mode | ctrl_overlay << 4 | 1u << 8); return;
    case 8: gpio_val = (gpio_val & ~1u) | (val & 1); wr(GPIO_BASE, GPIO_DATA, gpio_val); return;
    case 9: ctrl_auto_text = val & 1; if (!ctrl_auto_text) vproc_text(0, 0); return;
    case 10: fill_bars(out_buf[disp_idx]); return;
    default: return;
    }
    wr(VPROC_BASE, VP_CTRL, ctrl_mode | ctrl_overlay << 4);
}

static void on_ctrl(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port)
{
    ctrl_t c;
    (void)arg;
    if (p->tot_len >= sizeof c) {
        pbuf_copy_partial(p, &c, sizeof c, 0);
        if (c.magic == MAGIC_CTRL) {
            apply_ctrl(c.cmd, c.val);
            xil_printf("ctrl %u = %u\r\n", (unsigned)c.cmd, (unsigned)c.val);
        }
        udp_sendto(pcb, p, addr, port);       /* echo as acknowledgement */
    }
    pbuf_free(p);
}

/* ------------------------------------------------------------------ main */
int main(void)
{
    ip_addr_t ip, mask, gw;
    struct udp_pcb *pv, *pc;
    u32 t_stat, slow_ticks = 0, v;
    int i, have_hdmi;

    init_platform();
    Xil_Out32(GT_BASE + 8, Xil_In32(GT_BASE + 8) | 1);   /* global timer on */
    xil_printf("\r\n== fpga-video firmware ==\r\n");

    wr(GPIO_BASE, GPIO_TRI, 0);
    v = rd(VPROC_BASE, VP_VERSION);
    xil_printf("vproc version %08x\r\n", (unsigned)v);
    wr(VPROC_BASE, VP_SIZE, (FRAME_H << 16) | FRAME_W);
    wr(VPROC_BASE, VP_CTRL, ctrl_mode | ctrl_overlay << 4);

    for (i = 0; i < 3; i++) {
        fill_bars(out_buf[i]);
        inf[i].state = 0;
    }
    vdma_init();
    dma_reset();
    have_hdmi = adv7511_init() == 0;
    gpio_val = have_hdmi ? (1u << 4) : 0;     /* LD3: HDMI transmitter configured */
    wr(GPIO_BASE, GPIO_DATA, gpio_val);

    lwip_init();
    IP4_ADDR(&ip, 192, 168, 1, 10);
    IP4_ADDR(&mask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
    if (!xemac_add(&netif_s, &ip, &mask, &gw, mac_addr, PLATFORM_EMAC_BASEADDR)) {
        xil_printf("xemac_add failed\r\n");
        return -1;
    }
    netif_set_default(&netif_s);
    netif_set_up(&netif_s);

    pv = udp_new(); udp_bind(pv, IP_ADDR_ANY, PORT_VIDEO); udp_recv(pv, on_video, NULL);
    pc = udp_new(); udp_bind(pc, IP_ADDR_ANY, PORT_CTRL);  udp_recv(pc, on_ctrl, NULL);
    pcb_result = udp_new();
    xil_printf("listening on 192.168.1.10 udp %d (video) %d (ctrl), results to %d\r\n",
               PORT_VIDEO, PORT_CTRL, PORT_RESULT);

    t_stat = now_us();
    while (1) {
        xemacif_input(&netif_s);

        if (dma_busy) {
            u32 s2 = rd(DMA_BASE, DMA_S2MM_SR), m2 = rd(DMA_BASE, DMA_MM2S_SR);
            if ((s2 | m2) & DMA_SR_ERRS) {
                xil_printf("DMA error mm2s %08x s2mm %08x\r\n", (unsigned)m2, (unsigned)s2);
                inf[dma_idx].state = 0; dma_idx = -1; dma_busy = 0;
                dma_reset();
            } else if ((s2 & DMA_SR_IDLE) && (m2 & DMA_SR_IDLE)) {
                finish();
            }
        }
        if (!dma_busy && ready_idx >= 0) {
            int idx = ready_idx;
            ready_idx = -1;
            kick(idx);
        }

        if (TcpFastTmrFlag) { tcp_fasttmr(); TcpFastTmrFlag = 0; }
        if (TcpSlowTmrFlag) {
            tcp_slowtmr();
            TcpSlowTmrFlag = 0;
            if (++slow_ticks % 10 == 0) etharp_tmr();
        }
        if (now_us() - t_stat >= 1000000U) {
            t_stat = now_us();
            gpio_val ^= 1u << 1;              /* LD0 heartbeat */
            wr(GPIO_BASE, GPIO_DATA, gpio_val);
            xil_printf("rx %u proc %u drop %u lost_lines %u pkts %u err %x cyc %u\r\n",
                       (unsigned)st_rx_frames, (unsigned)st_proc_frames, (unsigned)st_dropped,
                       (unsigned)st_lost_lines, (unsigned)st_pkts,
                       (unsigned)rd(VPROC_BASE, VP_ERR), (unsigned)rd(VPROC_BASE, VP_CYCLES));
        }
    }
    cleanup_platform();
    return 0;
}
