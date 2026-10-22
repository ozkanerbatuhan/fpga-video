// 3x3 neighbourhood filter on an AXI4-Stream video frame.
//
// Window generator with two line buffers plus a selectable kernel. Borders are
// replicated (clamp-to-edge), so the output frame has the same size as the input.
//
// The frame is walked as (H+1) x (W+1) "steps": steps with r < H and c < W
// consume one input pixel; steps with r >= 1 and c >= 1 produce output pixel
// (r-1, c-1). The extra column and row are flush steps that need no input,
// which is how the right and bottom borders are produced without waiting for
// the next frame.
//
// tdata in  = {side[SW-1:0], pix[DW-1:0]}     side travels with the centre pixel
// tdata out = {side[SW-1:0], result[OW-1:0]}
// tuser = start of frame, tlast = end of line (AXI4-Stream video convention).
//
// Pipeline: issue (line-buffer read) -> column/shift -> kernel -> skid.
// One enable moves the whole pipeline, so a stall freezes every stage, the
// BRAM read registers included.
module win3x3_filter #(
    parameter int    DW     = 8,
    parameter int    SW     = 16,
    parameter int    OW     = 8,
    parameter string KERNEL = "GAUSS",   // GAUSS, SOBEL, ERODE, DILATE
    parameter int    MAX_W  = 640
) (
    input  logic             clk,
    input  logic             rst,
    input  logic [10:0]      width,
    input  logic [10:0]      height,
    input  logic [1:0]       sobel_shift,

    input  logic [SW+DW-1:0] s_tdata,
    input  logic             s_tuser,
    input  logic             s_tlast,
    input  logic             s_tvalid,
    output logic             s_tready,

    output logic [SW+OW-1:0] m_tdata,
    output logic             m_tuser,
    output logic             m_tlast,
    output logic             m_tvalid,
    input  logic             m_tready,

    output logic             err_sync,    // beat dropped while waiting for start of frame
    output logic             err_sof,     // tuser in the middle of a frame
    output logic             err_eol      // tlast not at the end of a line
);
    localparam int AW = $clog2(MAX_W);

    // ---------------- step counter / issue stage ----------------
    logic        busy;
    logic [10:0] rs, cs, fw, fh;
    logic        en;
    logic        skid_ready;

    wire  need_in = !busy || (rs < fh && cs < fw);
    wire  hunt    = !busy;                 // waiting for tuser
    assign s_tready = en && need_in;
    wire  take    = s_tvalid && s_tready;
    wire  fire    = busy ? (need_in ? take : en) : (take && s_tuser);

    wire [10:0] cur_w = busy ? fw : width;
    wire [10:0] cur_h = busy ? fh : height;

    // ---------------- line buffers ----------------
    (* ram_style = "block" *) logic [SW+DW-1:0] lb0 [MAX_W];   // row r-1 (with side)
    (* ram_style = "block" *) logic [DW-1:0]    lb1 [MAX_W];   // row r-2
    logic [SW+DW-1:0] lb0_q;
    logic [DW-1:0]    lb1_q;

    // stage 1 registers
    logic             s1_valid, s1_needin;
    logic [10:0]      s1_rs, s1_cs;
    logic [DW-1:0]    s1_pix;
    logic [SW-1:0]    s1_side;

    always_ff @(posedge clk) begin
        if (fire && (busy ? cs < fw : 1'b1))
            begin
                lb0_q <= lb0[busy ? cs[AW-1:0] : '0];
                lb1_q <= lb1[busy ? cs[AW-1:0] : '0];
            end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            rs <= '0; cs <= '0;
            err_sync <= 1'b0; err_sof <= 1'b0; err_eol <= 1'b0;
        end else begin
            err_sync <= take && hunt && !s_tuser;
            err_sof  <= take && busy && s_tuser;
            err_eol  <= take && (busy ? (s_tlast != (cs == fw - 1)) : (s_tuser && s_tlast != (width == 1)));
            if (fire) begin
                if (!busy) begin
                    busy <= 1'b1;
                    fw <= width;
                    fh <= height;
                end
                if (cs == cur_w) begin
                    cs <= '0;
                    if (rs == cur_h) begin
                        rs   <= '0;
                        busy <= 1'b0;
                    end else
                        rs <= rs + 1'b1;
                end else
                    cs <= cs + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            s1_valid <= 1'b0;
        else if (en) begin
            s1_valid  <= fire;
            s1_rs     <= busy ? rs : '0;
            s1_cs     <= busy ? cs : '0;
            s1_needin <= need_in;
            s1_pix    <= s_tdata[DW-1:0];
            s1_side   <= s_tdata[SW+DW-1:DW];
        end
    end

    // ---------------- stage 1: column vector, line-buffer write, window shift ----------------
    wire          s1_rcol = (s1_cs == fw);            // right flush column
    wire [DW-1:0] mid     = lb0_q[DW-1:0];
    wire [SW-1:0] mid_sd  = lb0_q[SW+DW-1:DW];
    wire [DW-1:0] top     = (s1_rs == 1) ? mid : lb1_q;
    wire [DW-1:0] bot     = (s1_rs == fh) ? mid : s1_pix;

    logic [DW-1:0] w [3][3];          // [row][col], col 2 newest
    logic [SW-1:0] sd1, sd2;          // side of window columns 1 and 2
    logic          s2_valid, s2_user, s2_last, s2_lclamp;

    always_ff @(posedge clk) begin
        if (en && s1_valid && s1_needin && !s1_rcol) begin
            lb0[s1_cs[AW-1:0]] <= {s1_side, s1_pix};
            lb1[s1_cs[AW-1:0]] <= mid;
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            s2_valid <= 1'b0;
        else if (en) begin
            s2_valid <= s1_valid && s1_rs != 0 && s1_cs != 0;
            if (s1_valid) begin
                for (int r = 0; r < 3; r++) begin
                    w[r][0] <= w[r][1];
                    w[r][1] <= w[r][2];
                end
                sd1 <= sd2;
                if (!s1_rcol) begin
                    w[0][2] <= top;
                    w[1][2] <= mid;
                    w[2][2] <= bot;
                    sd2     <= mid_sd;
                end
                s2_user   <= (s1_rs == 1) && (s1_cs == 1);
                s2_last   <= s1_rcol;
                s2_lclamp <= (s1_cs == 1);
            end
        end
    end

    // ---------------- stage A: clamp the left column, register the window ----------------
    logic [DW-1:0] p [3][3];
    logic [SW-1:0] sda, sdb;
    logic          a_valid, a_user, a_last, b_valid, b_user, b_last;
    always_ff @(posedge clk) begin
        if (rst) begin
            a_valid <= 1'b0;
            b_valid <= 1'b0;
        end else if (en) begin
            for (int r = 0; r < 3; r++) begin
                p[r][0] <= s2_lclamp ? w[r][1] : w[r][0];
                p[r][1] <= w[r][1];
                p[r][2] <= w[r][2];
            end
            sda <= sd1;
            a_valid <= s2_valid; a_user <= s2_user; a_last <= s2_last;
            sdb <= sda;
            b_valid <= a_valid;  b_user <= a_user;  b_last <= a_last;
        end
    end

    // ---------------- stage B (k1, registered) and output stage (k2) ----------------
    // Widths are explicit: every operand is zero-extended to 12 bit so no
    // expression is widened to 32 bit by an integer constant.
    logic [OW-1:0] result;
    generate
        if (KERNEL == "GAUSS") begin : g_gauss
            localparam int SWD = DW + 4;
            logic [SWD-1:0] r0, r1, r2;           // weighted row sums, k1
            always_ff @(posedge clk)
                if (en) begin
                    r0 <= SWD'(p[0][0]) + (SWD'(p[0][1]) << 1) + SWD'(p[0][2]);
                    r1 <= SWD'(p[1][0]) + (SWD'(p[1][1]) << 1) + SWD'(p[1][2]);
                    r2 <= SWD'(p[2][0]) + (SWD'(p[2][1]) << 1) + SWD'(p[2][2]);
                end
            wire [SWD-1:0] s = r0 + (r1 << 1) + r2 + SWD'(8);
            assign result = OW'(s >> 4);
        end else if (KERNEL == "SOBEL") begin : g_sobel
            localparam int GW = DW + 4;           // signed, |G| <= 4 * (2^DW - 1)
            logic signed [GW-1:0] gx, gy;         // k1
            always_ff @(posedge clk)
                if (en) begin
                    gx <= $signed(GW'(p[0][2]) + (GW'(p[1][2]) << 1) + GW'(p[2][2]))
                        - $signed(GW'(p[0][0]) + (GW'(p[1][0]) << 1) + GW'(p[2][0]));
                    gy <= $signed(GW'(p[2][0]) + (GW'(p[2][1]) << 1) + GW'(p[2][2]))
                        - $signed(GW'(p[0][0]) + (GW'(p[0][1]) << 1) + GW'(p[0][2]));
                end
            wire [GW-1:0] ax  = gx[GW-1] ? GW'(-gx) : GW'(gx);
            wire [GW-1:0] ay  = gy[GW-1] ? GW'(-gy) : GW'(gy);
            wire [GW:0]   mag = {1'b0, ax} + {1'b0, ay};
            wire [GW:0]   sh  = mag >> sobel_shift;
            assign result = (|sh[GW:OW]) ? {OW{1'b1}} : sh[OW-1:0];
        end else if (KERNEL == "ERODE") begin : g_erode
            logic [OW-1:0] rr [3];
            always_ff @(posedge clk)
                if (en)
                    for (int r = 0; r < 3; r++)
                        rr[r] <= OW'(p[r][0]) & OW'(p[r][1]) & OW'(p[r][2]);
            assign result = rr[0] & rr[1] & rr[2];
        end else begin : g_dilate
            logic [OW-1:0] rr [3];
            always_ff @(posedge clk)
                if (en)
                    for (int r = 0; r < 3; r++)
                        rr[r] <= OW'(p[r][0]) | OW'(p[r][1]) | OW'(p[r][2]);
            assign result = rr[0] | rr[1] | rr[2];
        end
    endgenerate

    // ---------------- output register + skid ----------------
    logic              o_valid, o_user, o_last;
    logic [SW+OW-1:0]  o_data;

    assign en = !o_valid || skid_ready;

    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= b_valid;
            o_data  <= {sdb, result};
            o_user  <= b_user;
            o_last  <= b_last;
        end
    end

    axis_skid #(.W(SW+OW+2)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
