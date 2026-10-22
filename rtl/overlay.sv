// Output stage: mode select, box outlines and a text line on top.
//
// tdata in  = {orig[15:0], sobel[7:0], gauss[7:0], mask_raw, mask}   (34 bit)
// tdata out = YUYV word {C, Y}
//
// Boxes are latched at start of frame, so the boxes drawn on frame n are the
// ones published by the labelling stage at the end of frame n-1.
// Box outline is 2 px thick, inside the box. Text: 16 characters, 8x8 font,
// white; text has priority over boxes, boxes over the image.
module overlay #(
    parameter int K = 8,
    parameter     FONT_FILE = "font8x8.mem"
) (
    input  logic             clk,
    input  logic             rst,
    input  logic [2:0]       mode,
    input  logic             overlay_en,
    input  logic [23:0]      box_color,     // {Cr, Cb, Y}
    input  logic [127:0]     text,          // byte i = character i
    input  logic [10:0]      text_x,
    input  logic [10:0]      text_y,
    input  logic [K*11-1:0]  box_x0, box_y0, box_x1, box_y1,
    input  logic [3:0]       n_boxes,

    input  logic [33:0]      s_tdata,
    input  logic             s_tuser,
    input  logic             s_tlast,
    input  logic             s_tvalid,
    output logic             s_tready,

    output logic [15:0]      m_tdata,
    output logic             m_tuser,
    output logic             m_tlast,
    output logic             m_tvalid,
    input  logic             m_tready
);
    logic en, skid_ready;
    assign s_tready = en;
    wire take = s_tvalid && en;

    // ---------------- stage 0: position, text lookup ----------------
    logic [10:0] x, y;
    wire  [10:0] x_now = s_tuser ? 11'd0 : x;
    wire  [10:0] y_now = s_tuser ? 11'd0 : y;

    always_ff @(posedge clk) begin
        if (rst) begin
            x <= '0; y <= '0;
        end else if (take) begin
            x <= s_tlast ? 11'd0 : x_now + 1'b1;
            y <= s_tlast ? y_now + 1'b1 : y_now;
        end
    end

    wire signed [11:0] tx = $signed({1'b0, x_now}) - $signed({1'b0, text_x});
    wire signed [11:0] ty = $signed({1'b0, y_now}) - $signed({1'b0, text_y});
    wire        in_text   = (tx >= 0) && (tx < 128) && (ty >= 0) && (ty < 8);
    wire [3:0]  ci        = tx[6:3];
    wire [7:0]  code      = text[ci*8 +: 8];
    wire        printable = (code >= 8'd32) && (code <= 8'd127);
    wire [6:0]  gi        = 7'(code - 8'd32);
    wire [9:0]  rom_a     = {gi, ty[2:0]};

    logic [7:0] font [0:767];
    initial $readmemh(FONT_FILE, font);
    logic [7:0] rom_q;
    always_ff @(posedge clk)
        if (en) rom_q <= font[rom_a];

    // boxes latched at start of frame
    logic [10:0] bx0 [K], by0 [K], bx1 [K], by1 [K];
    logic        bv  [K];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < K; i++) bv[i] <= 1'b0;
        end else if (take && s_tuser) begin
            for (int i = 0; i < K; i++) begin
                bx0[i] <= box_x0[i*11 +: 11];
                by0[i] <= box_y0[i*11 +: 11];
                bx1[i] <= box_x1[i*11 +: 11];
                by1[i] <= box_y1[i*11 +: 11];
                bv[i]  <= (i < n_boxes);
            end
        end
    end

    logic        p1_valid, p1_user, p1_last, p1_text;
    logic [2:0]  p1_col;
    logic [10:0] p1_x, p1_y;
    logic [33:0] p1_data;
    always_ff @(posedge clk) begin
        if (rst)
            p1_valid <= 1'b0;
        else if (en) begin
            p1_valid <= take;
            p1_user  <= s_tuser;
            p1_last  <= s_tlast;
            p1_data  <= s_tdata;
            p1_x     <= x_now;
            p1_y     <= y_now;
            p1_text  <= in_text && printable;
            p1_col   <= tx[2:0];
        end
    end

    // ---------------- stage 1: compose ----------------
    wire [15:0] orig  = p1_data[33:18];
    wire [7:0]  sob   = p1_data[17:10];
    wire [7:0]  gau   = p1_data[9:2];
    wire        mraw  = p1_data[1];
    wire        msk   = p1_data[0];

    logic [7:0] by, bc;
    always_comb begin
        bc = 8'd128;
        case (mode)
            3'd0:    begin by = orig[7:0]; bc = orig[15:8]; end
            3'd1:    by = orig[7:0];
            3'd2:    by = gau;
            3'd3:    by = sob;
            3'd4:    by = {8{mraw}};
            3'd5:    by = {8{msk}};
            default: begin by = orig[7:0]; bc = orig[15:8]; end
        endcase
    end

    logic hit;
    always_comb begin
        hit = 1'b0;
        for (int i = 0; i < K; i++) begin
            if (bv[i] &&
                p1_x >= bx0[i] && p1_x <= bx1[i] && p1_y >= by0[i] && p1_y <= by1[i] &&
                !(p1_x >= bx0[i] + 2 && p1_x + 2 <= bx1[i] && p1_y >= by0[i] + 2 && p1_y + 2 <= by1[i]))
                hit = 1'b1;
        end
    end

    wire glyph = p1_text && rom_q[p1_col];
    logic [7:0] oy, oc;
    always_comb begin
        oy = by; oc = bc;
        if (overlay_en && glyph) begin
            oy = 8'd235; oc = 8'd128;
        end else if (overlay_en && hit) begin
            oy = box_color[7:0];
            oc = p1_x[0] ? box_color[23:16] : box_color[15:8];
        end
    end

    logic        o_valid, o_user, o_last;
    logic [15:0] o_data;
    assign en = !o_valid || skid_ready;
    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= p1_valid;
            o_user  <= p1_user;
            o_last  <= p1_last;
            o_data  <= {oc, oy};
        end
    end

    axis_skid #(.W(18)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
