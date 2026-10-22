// Background subtraction with an exponential moving average background.
//
// Background bg is stored per pixel in on-chip BRAM as unsigned Q8.4 (12 bit).
//   mask   = |g - (bg >> 4)| > thresh
//   bg'    = bg + (((g << 4) - bg) >>> alpha_shift)       (arithmetic shift = floor)
// On an "init" frame (after reset or a bg_reset request) bg' = g << 4 and mask = 0.
//
// tdata in  = {side, g[7:0]}
// tdata out = {side, g[7:0], mask}
// The background RAM (bram_banked) has a read latency of RL cycles; pixel data
// waits in a matching delay line, then bg' is written back to the same address.
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
    localparam int RL = 4;                  // bram_banked read latency

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

    // ---------------- delay line matching the RAM latency ----------------
    logic [11:0]   q;
    logic          p_valid [RL];
    logic [SW+7:0] p_data  [RL];
    logic          p_user  [RL], p_last [RL], p_init [RL];
    logic [AW-1:0] p_addr  [RL];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < RL; i++) p_valid[i] <= 1'b0;
        end else if (en) begin
            p_valid[0] <= take;
            p_data[0]  <= s_tdata;  p_user[0] <= s_tuser; p_last[0] <= s_tlast;
            p_init[0]  <= init_now; p_addr[0] <= a_now;
            for (int i = 1; i < RL; i++) begin
                p_valid[i] <= p_valid[i-1];
                p_data[i]  <= p_data[i-1]; p_user[i] <= p_user[i-1]; p_last[i] <= p_last[i-1];
                p_init[i]  <= p_init[i-1]; p_addr[i] <= p_addr[i-1];
            end
        end
    end

    // ---------------- compute ----------------
    localparam int L = RL - 1;
    wire [7:0]          g    = p_data[L][7:0];
    wire [8:0]          bgi  = {1'b0, q[11:4]};
    wire signed [9:0]   dif  = $signed({2'b0, g}) - $signed({1'b0, bgi});
    wire [8:0]          adif = dif[9] ? 9'(-dif) : dif[8:0];
    wire                mask = !p_init[L] && (adif > {1'b0, thresh});
    wire signed [13:0]  d    = $signed({2'b0, g, 4'b0}) - $signed({2'b0, q});
    wire signed [13:0]  step = d >>> alpha_shift;
    wire [11:0]         bgn  = p_init[L] ? {g, 4'b0} : 12'($signed({2'b0, q}) + step);

    bram_banked #(.DEPTH(DEPTH), .W(12)) u_bg (
        .clk, .en,
        .raddr(a_now), .rdata(q),
        .we(en && p_valid[L]), .waddr(p_addr[L]), .wdata(bgn));

    // ---------------- output ----------------
    logic          o_valid, o_user, o_last;
    logic [SW+8:0] o_data;
    assign en = !o_valid || skid_ready;

    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= p_valid[L];
            o_data  <= {p_data[L], mask};
            o_user  <= p_user[L];
            o_last  <= p_last[L];
        end
    end

    axis_skid #(.W(SW+11)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
