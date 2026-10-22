// Video processing pipeline, AXI DMA in / AXI DMA out, AXI4-Lite control.
//
//   DMA MM2S -> framer -> luma -> gauss -> sobel -> motion -> erode -> dilate
//            -> dilate -> ccl (boxes) -> overlay -> eof -> DMA S2MM
//
// Side data accumulates along the chain so every stage sees the pixels it
// needs at the same frame position without separate delay lines:
//   luma    {orig16, y}
//   gauss   {orig16, g}
//   sobel   {orig16, g, sb}
//   motion  {orig16, sb, g, mraw}
//   morph   {orig16, sb, g, mraw, m}
module vproc_top #(
    parameter int MAX_W = 640,
    parameter int MAX_H = 360,
    parameter     FONT_FILE = "font8x8.mem"
) (
    input  logic        clk,
    input  logic        rst,

    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    input  logic [15:0] s_axis_tdata,
    input  logic        s_axis_tlast,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,

    output logic [15:0] m_axis_tdata,
    output logic        m_axis_tlast,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready
);
    localparam int K = 8;

    logic [2:0]   mode;
    logic         overlay_en, bg_reset;
    logic [10:0]  width, height, text_x, text_y;
    logic [7:0]   thresh;
    logic [3:0]   alpha_shift;
    logic [1:0]   sobel_shift;
    logic [17:0]  min_area;
    logic [23:0]  box_color;
    logic [127:0] text;
    logic [K*11-1:0] box_x0, box_y0, box_x1, box_y1;
    logic [K*18-1:0] box_area;
    logic [3:0]   n_boxes;
    logic [10:0]  n_labels;
    logic         overflow, ccl_done;

    // stream wires: {tdata, tuser, tlast, tvalid, tready}
    logic [15:0] f_d;  logic f_u, f_l, f_v, f_r;
    logic [23:0] l_d;  logic l_u, l_l, l_v, l_r;
    logic [23:0] g_d;  logic g_u, g_l, g_v, g_r;
    logic [31:0] s_d;  logic s_u, s_l, s_v, s_r;
    logic [32:0] m_d;  logic m_u, m_l, m_v, m_r;
    logic [33:0] e_d;  logic e_u, e_l, e_v, e_r;
    logic [33:0] d1_d; logic d1_u, d1_l, d1_v, d1_r;
    logic [33:0] d2_d; logic d2_u, d2_l, d2_v, d2_r;
    logic [33:0] c_d;  logic c_u, c_l, c_v, c_r;
    logic [15:0] o_d;  logic o_u, o_l, o_v, o_r;

    logic err_short, err_long, motion_ovf, ccl_sof;
    logic [2:0] win_sync, win_sof, win_eol;   // gauss, sobel, morph (OR of the three)
    logic [4:0] es, ef, ee;

    axis_framer u_framer (
        .clk, .rst, .width, .height,
        .s_tdata(s_axis_tdata), .s_tlast(s_axis_tlast), .s_tvalid(s_axis_tvalid), .s_tready(s_axis_tready),
        .m_tdata(f_d), .m_tuser(f_u), .m_tlast(f_l), .m_tvalid(f_v), .m_tready(f_r),
        .err_short, .err_long);

    luma #(.FMT("YUYV")) u_luma (
        .clk, .rst,
        .s_tdata(f_d), .s_tuser(f_u), .s_tlast(f_l), .s_tvalid(f_v), .s_tready(f_r),
        .m_tdata(l_d), .m_tuser(l_u), .m_tlast(l_l), .m_tvalid(l_v), .m_tready(l_r));

    win3x3_filter #(.DW(8), .SW(16), .OW(8), .KERNEL("GAUSS"), .MAX_W(MAX_W)) u_gauss (
        .clk, .rst, .width, .height, .sobel_shift,
        .s_tdata(l_d), .s_tuser(l_u), .s_tlast(l_l), .s_tvalid(l_v), .s_tready(l_r),
        .m_tdata(g_d), .m_tuser(g_u), .m_tlast(g_l), .m_tvalid(g_v), .m_tready(g_r),
        .err_sync(es[0]), .err_sof(ef[0]), .err_eol(ee[0]));

    win3x3_filter #(.DW(8), .SW(24), .OW(8), .KERNEL("SOBEL"), .MAX_W(MAX_W)) u_sobel (
        .clk, .rst, .width, .height, .sobel_shift,
        .s_tdata({g_d, g_d[7:0]}), .s_tuser(g_u), .s_tlast(g_l), .s_tvalid(g_v), .s_tready(g_r),
        .m_tdata(s_d), .m_tuser(s_u), .m_tlast(s_l), .m_tvalid(s_v), .m_tready(s_r),
        .err_sync(es[1]), .err_sof(ef[1]), .err_eol(ee[1]));

    // s_d = {orig16, g8, sb8} -> motion wants {side, g}: side = {orig16, sb8}
    motion_detect #(.SW(24), .DEPTH(MAX_W * MAX_H)) u_motion (
        .clk, .rst, .thresh, .alpha_shift, .bg_reset,
        .s_tdata({s_d[31:16], s_d[7:0], s_d[15:8]}), .s_tuser(s_u), .s_tlast(s_l), .s_tvalid(s_v), .s_tready(s_r),
        .m_tdata(m_d), .m_tuser(m_u), .m_tlast(m_l), .m_tvalid(m_v), .m_tready(m_r),
        .err_overflow(motion_ovf));

    win3x3_filter #(.DW(1), .SW(33), .OW(1), .KERNEL("ERODE"), .MAX_W(MAX_W)) u_erode (
        .clk, .rst, .width, .height, .sobel_shift,
        .s_tdata({m_d, m_d[0]}), .s_tuser(m_u), .s_tlast(m_l), .s_tvalid(m_v), .s_tready(m_r),
        .m_tdata(e_d), .m_tuser(e_u), .m_tlast(e_l), .m_tvalid(e_v), .m_tready(e_r),
        .err_sync(es[2]), .err_sof(ef[2]), .err_eol(ee[2]));

    win3x3_filter #(.DW(1), .SW(33), .OW(1), .KERNEL("DILATE"), .MAX_W(MAX_W)) u_dilate1 (
        .clk, .rst, .width, .height, .sobel_shift,
        .s_tdata({e_d[33:1], e_d[0]}), .s_tuser(e_u), .s_tlast(e_l), .s_tvalid(e_v), .s_tready(e_r),
        .m_tdata(d1_d), .m_tuser(d1_u), .m_tlast(d1_l), .m_tvalid(d1_v), .m_tready(d1_r),
        .err_sync(es[3]), .err_sof(ef[3]), .err_eol(ee[3]));

    win3x3_filter #(.DW(1), .SW(33), .OW(1), .KERNEL("DILATE"), .MAX_W(MAX_W)) u_dilate2 (
        .clk, .rst, .width, .height, .sobel_shift,
        .s_tdata({d1_d[33:1], d1_d[0]}), .s_tuser(d1_u), .s_tlast(d1_l), .s_tvalid(d1_v), .s_tready(d1_r),
        .m_tdata(d2_d), .m_tuser(d2_u), .m_tlast(d2_l), .m_tvalid(d2_v), .m_tready(d2_r),
        .err_sync(es[4]), .err_sof(ef[4]), .err_eol(ee[4]));

    ccl_bbox #(.SW(33), .MAX_W(MAX_W), .LW(11), .K(K)) u_ccl (
        .clk, .rst, .width, .height, .min_area,
        .s_tdata(d2_d), .s_tuser(d2_u), .s_tlast(d2_l), .s_tvalid(d2_v), .s_tready(d2_r),
        .m_tdata(c_d), .m_tuser(c_u), .m_tlast(c_l), .m_tvalid(c_v), .m_tready(c_r),
        .box_x0, .box_y0, .box_x1, .box_y1, .box_area, .n_boxes, .n_labels, .overflow,
        .done(ccl_done), .err_sof(ccl_sof));

    overlay #(.K(K), .FONT_FILE(FONT_FILE)) u_overlay (
        .clk, .rst, .mode, .overlay_en, .box_color, .text, .text_x, .text_y,
        .box_x0, .box_y0, .box_x1, .box_y1, .n_boxes,
        .s_tdata(c_d), .s_tuser(c_u), .s_tlast(c_l), .s_tvalid(c_v), .s_tready(c_r),
        .m_tdata(o_d), .m_tuser(o_u), .m_tlast(o_l), .m_tvalid(o_v), .m_tready(o_r));

    axis_eof u_eof (
        .clk, .rst, .height,
        .s_tdata(o_d), .s_tuser(o_u), .s_tlast(o_l), .s_tvalid(o_v), .s_tready(o_r),
        .m_tdata(m_axis_tdata), .m_tlast(m_axis_tlast), .m_tvalid(m_axis_tvalid), .m_tready(m_axis_tready));

    // Latency in cycles: first input beat of a frame to last output beat of the
    // same frame. The next frame can enter before the current one leaves, so
    // start times wait in a 2-entry queue.
    logic [31:0] now, t_q0, t_q1, cyc_last;
    logic [1:0]  t_n;
    wire in_sof  = f_v && f_r && f_u;
    wire out_eof = m_axis_tvalid && m_axis_tready && m_axis_tlast;
    always_ff @(posedge clk) begin
        if (rst) begin
            now <= '0; t_n <= '0; cyc_last <= '0;
        end else begin
            now <= now + 1'b1;
            if (out_eof && t_n != 0)
                cyc_last <= now - t_q0 + 1'b1;
            case ({in_sof, out_eof && t_n != 0})
                2'b10: begin
                    if (t_n == 0) t_q0 <= now; else t_q1 <= now;
                    if (t_n != 2) t_n <= t_n + 1'b1;
                end
                2'b01: begin t_q0 <= t_q1; t_n <= t_n - 1'b1; end
                2'b11: begin
                    if (t_n == 1) t_q0 <= now; else begin t_q0 <= t_q1; t_q1 <= now; end
                end
                default: ;
            endcase
        end
    end

    vproc_regs #(.K(K)) u_regs (
        .clk, .rst,
        .s_axi_awaddr, .s_axi_awvalid, .s_axi_awready, .s_axi_wdata, .s_axi_wstrb,
        .s_axi_wvalid, .s_axi_wready, .s_axi_bresp, .s_axi_bvalid, .s_axi_bready,
        .s_axi_araddr, .s_axi_arvalid, .s_axi_arready, .s_axi_rdata, .s_axi_rresp,
        .s_axi_rvalid, .s_axi_rready,
        .mode, .overlay_en, .bg_reset, .width, .height, .thresh, .alpha_shift, .sobel_shift,
        .min_area, .box_color, .text_x, .text_y, .text,
        .box_x0, .box_y0, .box_x1, .box_y1, .box_area, .n_boxes, .n_labels, .overflow,
        .frame_done(ccl_done),
        .err_pulse({ccl_sof, motion_ovf, |ee, |ef, |es, err_long, err_short}),
        .cycles(cyc_last));
endmodule
