// Background subtraction with an exponential moving average background.
//
// Background bg is stored per pixel in on-chip BRAM as unsigned Q8.4 (12 bit).
//   mask   = |g - (bg >> 4)| > thresh
//   bg'    = bg + (((g << 4) - bg) >>> alpha_shift)       (arithmetic shift = floor)
// On an "init" frame (after reset or a bg_reset request) bg' = g << 4 and mask = 0.
//
// tdata in  = {side, g[7:0]}
// tdata out = {side, g[7:0], mask}
// Read latency of the background RAM is 2 (BRAM output register), so the
// pipeline is: accept/read -> RAM reg -> compute/write-back -> out reg -> skid.
module motion_detect #(
    parameter int SW    = 24,
    parameter int DEPTH = 640 * 360
) (
    input  logic             clk,
    input  logic             rst,
    input  logic [7:0]       thresh,
    input  logic [3:0]       alpha_shift,
    input  logic             bg_reset,      // pulse: next frame re-initialises the background

    input  logic [SW+7:0]    s_tdata,
    input  logic             s_tuser,
    input  logic             s_tlast,
    input  logic             s_tvalid,
    output logic             s_tready,

    output logic [SW+8:0]    m_tdata,
    output logic             m_tuser,
    output logic             m_tlast,
    output logic             m_tvalid,
    input  logic             m_tready,

    output logic             err_overflow   // frame larger than DEPTH
);
    localparam int AW = $clog2(DEPTH);

    logic en, skid_ready;
    assign s_tready = en;
    wire take = s_tvalid && en;

    logic [AW-1:0] addr;
    logic          init_pending, frame_init;
    logic          full;

    wire [AW-1:0] a_now    = s_tuser ? '0 : addr;
    wire          init_now = s_tuser ? init_pending : frame_init;

    always_ff @(posedge clk) begin
        if (rst) begin
            addr <= '0;
            init_pending <= 1'b1;
            frame_init <= 1'b1;
            full <= 1'b0;
            err_overflow <= 1'b0;
        end else begin
            err_overflow <= 1'b0;
            if (bg_reset)
                init_pending <= 1'b1;
            if (take) begin
                if (s_tuser) begin
                    frame_init <= init_pending || bg_reset;
                    init_pending <= 1'b0;
                    full <= 1'b0;
                end
                if (a_now == AW'(DEPTH - 1)) begin
                    full <= 1'b1;
                    addr <= a_now;
                end else
                    addr <= a_now + 1'b1;
                if (!s_tuser && full)
                    err_overflow <= 1'b1;
            end
        end
    end

    // ---------------- background RAM ----------------
    (* ram_style = "block" *) logic [11:0] mem [DEPTH];
    logic [11:0] q1, q2;

    logic          p1_valid, p2_valid;
    logic [SW+7:0] p1_data, p2_data;
    logic          p1_user, p2_user, p1_last, p2_last, p1_init, p2_init;
    logic [AW-1:0] p1_addr, p2_addr;

    always_ff @(posedge clk) begin
        if (en) begin
            q1 <= mem[a_now];
            q2 <= q1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;
        end else if (en) begin
            p1_valid <= take;
            p1_data  <= s_tdata;  p1_user <= s_tuser; p1_last <= s_tlast;
            p1_init  <= init_now; p1_addr <= a_now;
            p2_valid <= p1_valid;
            p2_data  <= p1_data;  p2_user <= p1_user; p2_last <= p1_last;
            p2_init  <= p1_init;  p2_addr <= p1_addr;
        end
    end

    // ---------------- compute ----------------
    wire [7:0]          g    = p2_data[7:0];
    wire [8:0]          bgi  = {1'b0, q2[11:4]};
    wire signed [9:0]   dif  = $signed({2'b0, g}) - $signed({1'b0, bgi});
    wire [8:0]          adif = dif[9] ? 9'(-dif) : dif[8:0];
    wire                mask = !p2_init && (adif > {1'b0, thresh});
    wire signed [13:0]  d    = $signed({2'b0, g, 4'b0}) - $signed({2'b0, q2});
    wire signed [13:0]  step = d >>> alpha_shift;
    wire [11:0]         bgn  = p2_init ? {g, 4'b0} : 12'($signed({2'b0, q2}) + step);

    always_ff @(posedge clk) begin
        if (en && p2_valid)
            mem[p2_addr] <= bgn;
    end

    // ---------------- output ----------------
    logic          o_valid, o_user, o_last;
    logic [SW+8:0] o_data;
    assign en = !o_valid || skid_ready;

    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= p2_valid;
            o_data  <= {p2_data, mask};
            o_user  <= p2_user;
            o_last  <= p2_last;
        end
    end

    axis_skid #(.W(SW+11)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
