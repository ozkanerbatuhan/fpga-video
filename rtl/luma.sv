// Luma extraction. Output carries the original pixel as side data.
//   FMT "YUYV": in {C, Y}            -> Y = in[7:0]
//   FMT "RGB" : in {R, B, G} (Xilinx AXI4-Stream video order)
//               Y = ((66R + 129G + 25B + 128) >> 8) + 16   (BT.601, studio range)
// tdata out = {in, Y}
module luma #(
    parameter string FMT = "YUYV",
    parameter int    IW  = (FMT == "RGB") ? 24 : 16
) (
    input  logic          clk,
    input  logic          rst,
    input  logic [IW-1:0] s_tdata,
    input  logic          s_tuser,
    input  logic          s_tlast,
    input  logic          s_tvalid,
    output logic          s_tready,
    output logic [IW+7:0] m_tdata,
    output logic          m_tuser,
    output logic          m_tlast,
    output logic          m_tvalid,
    input  logic          m_tready
);
    logic [7:0] yv;
    generate
        if (FMT == "RGB") begin : g_rgb
            wire [7:0]  g = s_tdata[7:0];
            wire [7:0]  b = s_tdata[15:8];
            wire [7:0]  r = s_tdata[23:16];
            wire [16:0] acc = 66 * r + 129 * g + 25 * b + 128;
            assign yv = acc[15:8] + 8'd16;
        end else begin : g_yuyv
            assign yv = s_tdata[7:0];
        end
    endgenerate

    axis_skid #(.W(IW+10)) u_skid (
        .clk, .rst,
        .s_data ({s_tuser, s_tlast, s_tdata, yv}), .s_valid(s_tvalid), .s_ready(s_tready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
