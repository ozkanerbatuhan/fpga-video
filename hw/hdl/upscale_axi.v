// Module-reference wrapper for the 2x display upscaler.
module upscale_axi (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [15:0] s_axis_tdata,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    output wire [15:0] m_axis_tdata,
    output wire        m_axis_tuser,
    output wire        m_axis_tlast,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);
    reg rst = 1'b1;
    always @(posedge aclk) rst <= ~aresetn;

    upscale2x #(.IN_W(640)) u (
        .clk(aclk), .rst(rst),
        .s_tdata(s_axis_tdata), .s_tuser(s_axis_tuser), .s_tlast(s_axis_tlast),
        .s_tvalid(s_axis_tvalid), .s_tready(s_axis_tready),
        .m_tdata(m_axis_tdata), .m_tuser(m_axis_tuser), .m_tlast(m_axis_tlast),
        .m_tvalid(m_axis_tvalid), .m_tready(m_axis_tready));
endmodule
