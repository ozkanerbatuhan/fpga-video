// Streaming connected-component labelling (8-connected) with bounding boxes.
//
// Single pass over the binary mask, raster order. Each foreground pixel looks at
// four already-labelled neighbours (left, up-left, up, up-right). Because of
// 8-adjacency at most two of them can belong to different components:
//   up != 0  -> join up               (left, up-left and up-right all touch up)
//   up == 0  -> join up-right and (left or up-left), merge the two if different
// Labels live in a union-find parent table (LUTRAM; the larger root is pointed
// at the smaller one). Per-label box and area live in a BRAM and are merged into
// the surviving root on every union. At end of frame the roots are scanned and
// the K largest boxes with area >= min_area are published.
//
// The block passes the stream through unchanged. It is a multi-cycle state
// machine (1 cycle per background pixel, about 4-8 per foreground pixel), so it
// back-pressures upstream; at 25 fps of 640x360 the budget is ~25 cycles/pixel.
// The next frame's first beat is not accepted until the current frame's boxes
// are published, so downstream sees frame n's boxes before frame n+1 starts.
//
// Box order: area descending, then y0, x0, y1, x1 ascending.
module ccl_bbox #(
    parameter int SW    = 1,       // side data passed through with the mask bit
    parameter int MAX_W = 640,
    parameter int LW    = 11,      // label width -> 2^LW - 1 provisional labels
    parameter int K     = 8
) (
    input  logic             clk,
    input  logic             rst,
    input  logic [10:0]      width,
    input  logic [10:0]      height,
    input  logic [17:0]      min_area,

    input  logic [SW:0]      s_tdata,      // {side, mask}
    input  logic             s_tuser,
    input  logic             s_tlast,
    input  logic             s_tvalid,
    output logic             s_tready,

    output logic [SW:0]      m_tdata,
    output logic             m_tuser,
    output logic             m_tlast,
    output logic             m_tvalid,
    input  logic             m_tready,

    // results of the last completed frame
    output logic [K*11-1:0]  box_x0, box_y0, box_x1, box_y1,
    output logic [K*18-1:0]  box_area,
    output logic [3:0]       n_boxes,
    output logic [LW-1:0]    n_labels,
    output logic             overflow,
    output logic             done,          // 1-cycle pulse when results update
    output logic             err_sof
);
    localparam int MAXL = 2**LW - 1;
    localparam int BW   = 11*4 + 18;
    localparam int XW   = $clog2(MAX_W);

    typedef enum logic [3:0] {
        S_PIX, S_FIND_A, S_FIND_B, S_RD, S_UPD, S_MRG_HI, S_MRG_LO, S_MRG_UPD,
        S_SCAN0, S_SCAN_RD, S_SCAN_CHK, S_PUB
    } state_t;
    state_t st;

    // ---------------- tables (one write port each) ----------------
    (* ram_style = "distributed" *) logic [LW-1:0] lb  [MAX_W];   // previous-row labels
    (* ram_style = "distributed" *) logic [LW-1:0] par [2**LW];   // union-find parent
    (* ram_style = "block" *)       logic [BW-1:0] bbt [2**LW];   // {x0,y0,x1,y1,area}

    logic          lb_we,  par_we,  bb_we, bb_re;
    logic [XW-1:0] lb_wa;
    logic [LW-1:0] lb_wd, par_wa, par_wd, bb_wa, bb_ra;
    logic [BW-1:0] bb_wd, bb_q;

    always_ff @(posedge clk) begin
        if (lb_we)  lb[lb_wa]   <= lb_wd;
        if (par_we) par[par_wa] <= par_wd;
        if (bb_we)  bbt[bb_wa]  <= bb_wd;
        if (bb_re)  bb_q        <= bbt[bb_ra];
    end

    // ---------------- frame position and neighbours ----------------
    // x is 0 at every start of frame (it wraps on the last pixel), so the
    // line-buffer addresses come straight from registers (x, x_p1); only the
    // row/column flags look at tuser.
    logic [10:0]   x, x_p1, y, fw, fh;
    logic          x_last, y_last, y_zero;
    logic [LW-1:0] left_r, saved_u;
    logic [LW:0]   next_l, scan_l;
    logic          in_frame;

    wire [10:0] x_now  = x;
    wire [10:0] y_now  = s_tuser ? 11'd0 : y;
    wire        row0   = s_tuser || y_zero;
    wire        col0   = (x == 0);
    wire        eol    = s_tuser ? (width == 1) : x_last;
    wire        eof    = eol && (s_tuser ? (height == 1) : y_last);

    wire [LW-1:0] lb_u  = lb[x[XW-1:0]];
    wire [LW-1:0] lb_ur = lb[x_p1[XW-1:0]];
    wire [LW-1:0] n_u  = !row0 ? lb_u : '0;
    wire [LW-1:0] n_ur = (!row0 && !eol) ? lb_ur : '0;
    wire [LW-1:0] n_ul = (!row0 && !col0) ? saved_u : '0;
    wire [LW-1:0] n_l  = !col0 ? left_r : '0;

    wire [LW-1:0] lft = (n_l != 0) ? n_l : n_ul;
    wire [LW-1:0] a0  = (n_u != 0) ? n_u : ((n_ur != 0) ? n_ur : lft);
    wire [LW-1:0] b0  = (n_u != 0) ? '0  : ((n_ur != 0) ? lft  : '0);

    // ---------------- pass-through ----------------
    logic skid_ready;
    assign s_tready = (st == S_PIX) && skid_ready;
    wire take  = s_tvalid && s_tready;
    wire label = take && (s_tuser || in_frame);   // beats outside a frame are only passed

    axis_skid #(.W(SW+3)) u_skid (
        .clk, .rst,
        .s_data ({s_tuser, s_tlast, s_tdata}), .s_valid(take), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );

    // ---------------- current foreground pixel ----------------
    logic [10:0]   px, py;
    logic [LW-1:0] ca, cb, ra, rf, u_cur, root;
    logic          last_px;
    logic [BW-1:0] bb_hi;

    function automatic logic [BW-1:0] bb_union(input logic [BW-1:0] a, input logic [BW-1:0] b);
        logic [10:0] ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
        logic [17:0] aa, ba;
        {ax0, ay0, ax1, ay1, aa} = a;
        {bx0, by0, bx1, by1, ba} = b;
        return {(ax0 < bx0) ? ax0 : bx0, (ay0 < by0) ? ay0 : by0,
                (ax1 > bx1) ? ax1 : bx1, (ay1 > by1) ? ay1 : by1, 18'(aa + ba)};
    endfunction

    wire [BW-1:0] pix_bb  = {px, py, px, py, 18'd1};
    wire          new_lab = (ca == 0);
    wire          can_new = (next_l <= MAXL);
    wire [LW-1:0] lo      = (ra < rf) ? ra : rf;
    wire [LW-1:0] hi      = (ra < rf) ? rf : ra;
    logic [LW-1:0] par_q;
    logic          pv;
    always_ff @(posedge clk) par_q <= par[rf];
    wire [LW-1:0] fin_lab = (st == S_UPD && new_lab) ? (can_new ? next_l[LW-1:0] : '0) : root;
    wire          fin     = (st == S_UPD) || (st == S_MRG_UPD);

    // ---------------- top-K ----------------
    logic [10:0]   k_x0 [K], k_y0 [K], k_x1 [K], k_y1 [K];
    logic [17:0]   k_a  [K];
    logic          k_v  [K];
    logic          scan_root;

    wire [10:0] c_x0 = bb_q[BW-1  -: 11];
    wire [10:0] c_y0 = bb_q[BW-12 -: 11];
    wire [10:0] c_x1 = bb_q[BW-23 -: 11];
    wire [10:0] c_y1 = bb_q[BW-34 -: 11];
    wire [17:0] c_a  = bb_q[17:0];

    logic better [K];
    logic [3:0] n_valid;
    always_comb begin
        n_valid = '0;
        for (int i = 0; i < K; i++) begin
            better[i] = !k_v[i] ||
                (c_a > k_a[i]) || (c_a == k_a[i] && (
                (c_y0 < k_y0[i]) || (c_y0 == k_y0[i] && (
                (c_x0 < k_x0[i]) || (c_x0 == k_x0[i] && (
                (c_y1 < k_y1[i]) || (c_y1 == k_y1[i] && c_x1 < k_x1[i])))))));
            n_valid = n_valid + k_v[i];
        end
    end

    // ---------------- memory ports ----------------
    always_comb begin
        lb_we = 1'b0;  lb_wa = px[XW-1:0];  lb_wd = fin_lab;
        par_we = 1'b0; par_wa = hi;         par_wd = lo;
        bb_we = 1'b0;  bb_wa = root;        bb_wd = bb_union(bb_q, pix_bb);
        bb_re = 1'b0;  bb_ra = root;
        case (st)
            S_PIX: if (label && !s_tdata[0]) begin
                lb_we = 1'b1; lb_wa = x_now[XW-1:0]; lb_wd = '0;
            end
            S_FIND_B: if (pv && par_q == rf && rf != ra) par_we = 1'b1;
            S_RD:      bb_re = 1'b1;
            S_UPD: begin
                lb_we = 1'b1;
                if (!new_lab) bb_we = 1'b1;
                else if (can_new) begin
                    par_we = 1'b1; par_wa = next_l[LW-1:0]; par_wd = next_l[LW-1:0];
                    bb_we = 1'b1;  bb_wa = next_l[LW-1:0];  bb_wd = pix_bb;
                end
            end
            S_MRG_HI:  begin bb_re = 1'b1; bb_ra = hi; end
            S_MRG_LO:  bb_re = 1'b1;
            S_MRG_UPD: begin
                lb_we = 1'b1;
                bb_we = 1'b1; bb_wd = bb_union(bb_union(bb_q, bb_hi), pix_bb);
            end
            S_SCAN_RD: begin bb_re = 1'b1; bb_ra = scan_l[LW-1:0]; end
            default: ;
        endcase
    end

    // ---------------- control ----------------
    always_ff @(posedge clk) begin
        if (rst) begin
            st <= S_PIX;
            x <= '0; x_p1 <= 11'd1; y <= '0;
            x_last <= 1'b0; y_last <= 1'b0; y_zero <= 1'b1;
            in_frame <= 1'b0;
            next_l <= 1;
            scan_l <= '0;
            n_boxes <= '0; n_labels <= '0; overflow <= 1'b0;
            done <= 1'b0; err_sof <= 1'b0;
            for (int i = 0; i < K; i++) k_v[i] <= 1'b0;
            box_x0 <= '0; box_y0 <= '0; box_x1 <= '0; box_y1 <= '0; box_area <= '0;
        end else begin
            done <= 1'b0;
            err_sof <= 1'b0;
            if (fin) begin
                left_r  <= fin_lab;
                saved_u <= u_cur;
                st      <= last_px ? S_SCAN0 : S_PIX;
            end
            case (st)
            S_PIX: if (label) begin
                if (s_tuser) begin
                    fw <= width; fh <= height;
                    next_l <= 1;
                    overflow <= 1'b0;
                    if (in_frame) err_sof <= 1'b1;
                end
                in_frame <= 1'b1;
                px <= x_now; py <= y_now;
                u_cur <= n_u;
                last_px <= eof;
                // next position (x wraps to 0 after the last pixel of a line)
                if (eol) begin
                    x <= 11'd0; x_p1 <= 11'd1;
                    x_last <= ((s_tuser ? width : fw) == 1);
                    y <= y_now + 1'b1;
                    y_zero <= 1'b0;
                    y_last <= (y_now + 1'b1 == (s_tuser ? height : fh) - 1);
                end else begin
                    x <= x_p1; x_p1 <= x_p1 + 1'b1;
                    x_last <= (x_p1 == (s_tuser ? width : fw) - 1);
                    y <= y_now;
                    y_zero <= (y_now == 0);
                    y_last <= (y_now == (s_tuser ? height : fh) - 1);
                end
                if (!s_tdata[0]) begin
                    left_r  <= '0;
                    saved_u <= n_u;
                    if (eof) st <= S_SCAN0;
                end else begin
                    ca <= a0;
                    cb <= b0;
                    rf <= a0;
                    pv <= 1'b0;
                    st <= (a0 == 0) ? S_UPD : S_FIND_A;
                end
            end

            // par_q follows par[rf] one cycle late: pv marks it valid
            S_FIND_A:                       // rf walks up the tree from ca
                if (!pv)
                    pv <= 1'b1;
                else if (par_q == rf) begin
                    ra <= rf;
                    if (cb == 0) begin
                        root <= rf;
                        st <= S_RD;
                    end else begin
                        rf <= cb;
                        pv <= 1'b0;
                        st <= S_FIND_B;
                    end
                end else begin
                    rf <= par_q;
                    pv <= 1'b0;
                end

            S_FIND_B:                       // rf walks up the tree from cb
                if (!pv)
                    pv <= 1'b1;
                else if (par_q == rf) begin
                    root <= lo;
                    st <= (rf == ra) ? S_RD : S_MRG_HI;   // par[hi] <= lo written by the port logic
                end else begin
                    rf <= par_q;
                    pv <= 1'b0;
                end

            S_RD:     st <= S_UPD;          // bb_q <= bbt[root]
            S_UPD:    if (new_lab) begin
                          if (can_new) next_l <= next_l + 1'b1;
                          else overflow <= 1'b1;
                      end
            S_MRG_HI: st <= S_MRG_LO;       // bb_q <= bbt[hi]
            S_MRG_LO: begin                 // bb_q <= bbt[lo]
                bb_hi <= bb_q;
                st <= S_MRG_UPD;
            end
            S_MRG_UPD: ;

            // ---------------- end of frame ----------------
            S_SCAN0: begin
                for (int i = 0; i < K; i++) k_v[i] <= 1'b0;
                scan_l <= 1;
                st <= (next_l == 1) ? S_PUB : S_SCAN_RD;
            end
            S_SCAN_RD: begin                // bb_q <= bbt[scan_l]
                scan_root <= (par[scan_l[LW-1:0]] == scan_l[LW-1:0]);
                st <= S_SCAN_CHK;
            end
            S_SCAN_CHK: begin
                if (scan_root && c_a >= min_area) begin
                    for (int i = 0; i < K; i++) begin
                        if (better[i]) begin
                            if (i == 0 || !better[(i == 0) ? 0 : i-1]) begin
                                k_x0[i] <= c_x0; k_y0[i] <= c_y0; k_x1[i] <= c_x1; k_y1[i] <= c_y1;
                                k_a[i]  <= c_a;  k_v[i]  <= 1'b1;
                            end else begin
                                k_x0[i] <= k_x0[(i == 0) ? 0 : i-1]; k_y0[i] <= k_y0[(i == 0) ? 0 : i-1];
                                k_x1[i] <= k_x1[(i == 0) ? 0 : i-1]; k_y1[i] <= k_y1[(i == 0) ? 0 : i-1];
                                k_a[i]  <= k_a[(i == 0) ? 0 : i-1];  k_v[i]  <= k_v[(i == 0) ? 0 : i-1];
                            end
                        end
                    end
                end
                scan_l <= scan_l + 1'b1;
                st <= (scan_l + 1'b1 >= next_l) ? S_PUB : S_SCAN_RD;
            end
            S_PUB: begin
                for (int i = 0; i < K; i++) begin
                    box_x0[i*11 +: 11]   <= k_v[i] ? k_x0[i] : 11'd0;
                    box_y0[i*11 +: 11]   <= k_v[i] ? k_y0[i] : 11'd0;
                    box_x1[i*11 +: 11]   <= k_v[i] ? k_x1[i] : 11'd0;
                    box_y1[i*11 +: 11]   <= k_v[i] ? k_y1[i] : 11'd0;
                    box_area[i*18 +: 18] <= k_v[i] ? k_a[i]  : 18'd0;
                end
                n_boxes  <= n_valid;
                n_labels <= LW'(next_l - 1'b1);
                in_frame <= 1'b0;
                done <= 1'b1;
                st <= S_PIX;
            end
            default: st <= S_PIX;
            endcase
        end
    end
endmodule
