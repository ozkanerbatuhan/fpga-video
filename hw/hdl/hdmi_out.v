// Parallel video to the ZedBoard ADV7511 (16-bit YCbCr 4:2:2, separate syncs).
// Outputs are registered in the IOBs on the pixel clock; the pixel clock is
// forwarded inverted through an ODDR so the ADV7511 samples mid-eye.
//
// ctrl comes from an AXI GPIO (static, written by software):
//   ctrl[0]   swap the two bytes of the data bus (Y/C lane order)
//   ctrl[4:1] user LEDs LD0..LD3
module hdmi_out (
    input  wire        pix_clk,
    input  wire [15:0] vid_data,      // {C, Y}
    input  wire        vid_hsync,
    input  wire        vid_vsync,
    input  wire        vid_active_video,
    input  wire [4:0]  ctrl,
    (* X_INTERFACE_IGNORE = "true" *)
    output wire        hd_clk,
    output reg  [15:0] hd_d,
    output reg         hd_hsync,
    output reg         hd_vsync,
    output reg         hd_de,
    output wire [3:0]  led
);
    (* ASYNC_REG = "TRUE" *) reg [4:0] ctrl_q1, ctrl_q2;
    always @(posedge pix_clk) begin
        ctrl_q1 <= ctrl;
        ctrl_q2 <= ctrl_q1;
    end
    assign led = ctrl_q2[4:1];

    always @(posedge pix_clk) begin
        hd_d     <= ctrl_q2[0] ? {vid_data[7:0], vid_data[15:8]} : vid_data;
        hd_hsync <= vid_hsync;
        hd_vsync <= vid_vsync;
        hd_de    <= vid_active_video;
    end

    ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_clk (
        .Q(hd_clk), .C(pix_clk), .CE(1'b1), .D1(1'b0), .D2(1'b1), .R(1'b0), .S(1'b0));
endmodule
