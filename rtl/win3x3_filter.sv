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

    // ---------------- stage 2: kernel ----------------
    logic [DW-1:0] p [3][3];
    always_comb begin
        for (int r = 0; r < 3; r++) begin
            p[r][0] = s2_lclamp ? w[r][1] : w[r][0];
            p[r][1] = w[r][1];
            p[r][2] = w[r][2];
        end
    end

    logic [OW-1:0] result;
    generate
        if (KERNEL == "GAUSS") begin : g_gauss
            logic [DW+3:0] s;
            always_comb begin
                s = p[0][0] + 2*p[0][1] + p[0][2]
                  + 2*p[1][0] + 4*p[1][1] + 2*p[1][2]
                  + p[2][0] + 2*p[2][1] + p[2][2];
                result = OW'((s + 8) >> 4);
            end
        end else if (KERNEL == "SOBEL") begin : g_sobel
            logic signed [DW+3:0] gx, gy;
            logic        [DW+3:0] ax, ay;
            logic        [DW+4:0] mag, sh;
            always_comb begin
                gx = $signed({3'b0, p[0][2]}) + 2*$signed({3'b0, p[1][2]}) + $signed({3'b0, p[2][2]})
                   - $signed({3'b0, p[0][0]}) - 2*$signed({3'b0, p[1][0]}) - $signed({3'b0, p[2][0]});
                gy = $signed({3'b0, p[2][0]}) + 2*$signed({3'b0, p[2][1]}) + $signed({3'b0, p[2][2]})
                   - $signed({3'b0, p[0][0]}) - 2*$signed({3'b0, p[0][1]}) - $signed({3'b0, p[0][2]});
                ax  = gx[DW+3] ? -gx : gx;
                ay  = gy[DW+3] ? -gy : gy;
                mag = ax + ay;
                sh  = mag >> sobel_shift;
                result = (sh > (2**OW - 1)) ? {OW{1'b1}} : OW'(sh);
            end
        end else if (KERNEL == "ERODE") begin : g_erode
            always_comb begin
                result = '1;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        result = result & OW'(p[r][c]);
            end
        end else begin : g_dilate
            always_comb begin
                result = '0;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        result = result | OW'(p[r][c]);
            end
        end
    endgenerate

    // ---------------- stage 3: output register + skid ----------------
    logic              o_valid, o_user, o_last;
    logic [SW+OW-1:0]  o_data;

    assign en = !o_valid || skid_ready;

    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= s2_valid;
            o_data  <= {sd1, result};
            o_user  <= s2_user;
            o_last  <= s2_last;
        end
    end

    axis_skid #(.W(SW+OW+2)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
