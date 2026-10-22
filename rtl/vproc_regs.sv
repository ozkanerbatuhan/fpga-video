// AXI4-Lite register file of the video pipeline. Map: docs/SPEC.md.
module vproc_regs #(
    parameter int K = 8
) (
    input  logic          clk,
    input  logic          rst,
    // AXI4-Lite slave
    input  logic [7:0]    s_axi_awaddr,
    input  logic          s_axi_awvalid,
    output logic          s_axi_awready,
    input  logic [31:0]   s_axi_wdata,
    input  logic [3:0]    s_axi_wstrb,
    input  logic          s_axi_wvalid,
    output logic          s_axi_wready,
    output logic [1:0]    s_axi_bresp,
    output logic          s_axi_bvalid,
    input  logic          s_axi_bready,
    input  logic [7:0]    s_axi_araddr,
    input  logic          s_axi_arvalid,
    output logic          s_axi_arready,
    output logic [31:0]   s_axi_rdata,
    output logic [1:0]    s_axi_rresp,
    output logic          s_axi_rvalid,
    input  logic          s_axi_rready,
    // control
    output logic [2:0]    mode,
    output logic          overlay_en,
    output logic          bg_reset,      // 1-cycle pulse
    output logic [10:0]   width,
    output logic [10:0]   height,
    output logic [7:0]    thresh,
    output logic [3:0]    alpha_shift,
    output logic [1:0]    sobel_shift,
    output logic [17:0]   min_area,
    output logic [23:0]   box_color,
    output logic [10:0]   text_x,
    output logic [10:0]   text_y,
    output logic [127:0]  text,
    // status
    input  logic [K*11-1:0] box_x0, box_y0, box_x1, box_y1,
    input  logic [K*18-1:0] box_area,
    input  logic [3:0]    n_boxes,
    input  logic [10:0]   n_labels,
    input  logic          overflow,
    input  logic          frame_done,
    input  logic [6:0]    err_pulse,
    input  logic [31:0]   cycles
);
    localparam logic [31:0] VERSION = 32'h5650_0100;

    logic [31:0] frames;
    logic [6:0]  err;

    // ---------------- write channel ----------------
    logic [7:0]  waddr;
    logic        aw_ok, w_ok;
    logic [31:0] wdata;
    logic [3:0]  wstrb;

    assign s_axi_awready = !aw_ok && !s_axi_bvalid;
    assign s_axi_wready  = !w_ok  && !s_axi_bvalid;
    assign s_axi_bresp   = 2'b00;

    function automatic logic [31:0] merge(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
        for (int i = 0; i < 4; i++) if (s[i]) old[i*8 +: 8] = d[i*8 +: 8];
        return old;
    endfunction

    logic [31:0] r_ctrl, r_size, r_param, r_area, r_color, r_tpos;
    logic [31:0] r_text [4];

    assign mode        = r_ctrl[2:0];
    assign overlay_en  = r_ctrl[4];
    assign width       = r_size[10:0];
    assign height      = r_size[26:16];
    assign thresh      = r_param[7:0];
    assign alpha_shift = r_param[11:8];
    assign sobel_shift = r_param[17:16];
    assign min_area    = r_area[17:0];
    assign box_color   = r_color[23:0];
    assign text_x      = r_tpos[10:0];
    assign text_y      = r_tpos[26:16];
    assign text        = {r_text[3], r_text[2], r_text[1], r_text[0]};

    always_ff @(posedge clk) begin
        if (rst) begin
            aw_ok <= 1'b0; w_ok <= 1'b0; s_axi_bvalid <= 1'b0;
            r_ctrl  <= 32'h0000_0010;               // mode 0, overlay on
            r_size  <= {5'd0, 11'd360, 5'd0, 11'd640};
            r_param <= {14'd0, 2'd1, 4'd0, 4'd4, 8'd20};
            r_area  <= 32'd40;
            r_color <= {8'd0, 8'd34, 8'd54, 8'd145};
            r_tpos  <= {5'd0, 11'd8, 5'd0, 11'd8};
            for (int i = 0; i < 4; i++) r_text[i] <= '0;
            bg_reset <= 1'b0;
            err <= '0;
            frames <= '0;
        end else begin
            bg_reset <= 1'b0;
            err <= err | err_pulse;
            if (frame_done) frames <= frames + 1'b1;

            if (s_axi_awvalid && s_axi_awready) begin
                aw_ok <= 1'b1; waddr <= s_axi_awaddr;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                w_ok <= 1'b1; wdata <= s_axi_wdata; wstrb <= s_axi_wstrb;
            end
            if (aw_ok && w_ok) begin
                aw_ok <= 1'b0; w_ok <= 1'b0; s_axi_bvalid <= 1'b1;
                case (waddr[7:2])
                    6'h00: begin
                        r_ctrl <= merge(r_ctrl, wdata, wstrb) & 32'h0000_0017;
                        if (wstrb[1] && wdata[8]) bg_reset <= 1'b1;
                    end
                    6'h01: r_size  <= merge(r_size,  wdata, wstrb);
                    6'h02: r_param <= merge(r_param, wdata, wstrb);
                    6'h03: r_area  <= merge(r_area,  wdata, wstrb);
                    6'h04: r_color <= merge(r_color, wdata, wstrb);
                    6'h05: r_tpos  <= merge(r_tpos,  wdata, wstrb);
                    6'h08: r_text[0] <= merge(r_text[0], wdata, wstrb);
                    6'h09: r_text[1] <= merge(r_text[1], wdata, wstrb);
                    6'h0A: r_text[2] <= merge(r_text[2], wdata, wstrb);
                    6'h0B: r_text[3] <= merge(r_text[3], wdata, wstrb);
                    6'h0E: err <= (err & ~wdata[6:0]) | err_pulse;     // write 1 to clear
                    default: ;
                endcase
            end
            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;
        end
    end

    // ---------------- read channel ----------------
    assign s_axi_arready = !s_axi_rvalid;
    assign s_axi_rresp   = 2'b00;

    logic [31:0] rd;
    wire  [2:0]  bi = s_axi_araddr[5:3];
    wire  [2:0]  ai = s_axi_araddr[4:2];
    always_comb begin
        rd = '0;
        case (s_axi_araddr[7:2])
            6'h00: rd = r_ctrl;
            6'h01: rd = r_size;
            6'h02: rd = r_param;
            6'h03: rd = r_area;
            6'h04: rd = r_color;
            6'h05: rd = r_tpos;
            6'h06: rd = VERSION;
            6'h08: rd = r_text[0];
            6'h09: rd = r_text[1];
            6'h0A: rd = r_text[2];
            6'h0B: rd = r_text[3];
            6'h0C: rd = {overflow, 4'd0, n_labels, 12'd0, n_boxes};
            6'h0D: rd = frames;
            6'h0E: rd = {25'd0, err};
            6'h0F: rd = cycles;
            default: begin
                if (s_axi_araddr[7:6] == 2'b01)                  // 0x40..0x7F boxes
                    rd = s_axi_araddr[2] ? {5'd0, box_y1[bi*11 +: 11], 5'd0, box_x1[bi*11 +: 11]}
                                         : {5'd0, box_y0[bi*11 +: 11], 5'd0, box_x0[bi*11 +: 11]};
                else if (s_axi_araddr[7:5] == 3'b100)            // 0x80..0x9F areas
                    rd = {14'd0, box_area[ai*18 +: 18]};
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst)
            s_axi_rvalid <= 1'b0;
        else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rdata  <= rd;
            end else if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;
        end
    end
endmodule
