// Adapters between AXI DMA (packet = whole frame, tlast at the end) and
// AXI4-Stream video (tuser = start of frame, tlast = end of line).
// Both are combinational on the handshake; the next block registers.

// DMA MM2S -> video. Generates tuser/tlast from width/height counters and
// checks that the DMA's tlast lands on the last pixel of the frame.
module axis_framer (
    input  logic        clk,
    input  logic        rst,
    input  logic [10:0] width,
    input  logic [10:0] height,
    input  logic [15:0] s_tdata,
    input  logic        s_tlast,         // DMA end of packet
    input  logic        s_tvalid,
    output logic        s_tready,
    output logic [15:0] m_tdata,
    output logic        m_tuser,
    output logic        m_tlast,
    output logic        m_tvalid,
    input  logic        m_tready,
    output logic        err_short,       // packet ended before the frame
    output logic        err_long         // frame ended without end of packet
);
    logic [10:0] x, y;
    wire eol = (x == width - 1);
    wire eof = eol && (y == height - 1);

    assign m_tdata  = s_tdata;
    assign m_tvalid = s_tvalid;
    assign s_tready = m_tready;
    assign m_tuser  = (x == 0) && (y == 0);
    assign m_tlast  = eol || s_tlast;      // an early end of packet also ends the line

    always_ff @(posedge clk) begin
        if (rst) begin
            x <= '0; y <= '0;
            err_short <= 1'b0; err_long <= 1'b0;
        end else begin
            err_short <= 1'b0; err_long <= 1'b0;
            if (s_tvalid && m_tready) begin
                if (eof || s_tlast) begin
                    x <= '0; y <= '0;
                    err_short <= s_tlast && !eof;
                    err_long  <= eof && !s_tlast;
                end else if (eol) begin
                    x <= '0; y <= y + 1'b1;
                end else
                    x <= x + 1'b1;
            end
        end
    end
endmodule

// Video -> DMA S2MM: tlast only on the last pixel of the frame.
module axis_eof (
    input  logic        clk,
    input  logic        rst,
    input  logic [10:0] height,
    input  logic [15:0] s_tdata,
    input  logic        s_tuser,
    input  logic        s_tlast,         // end of line
    input  logic        s_tvalid,
    output logic        s_tready,
    output logic [15:0] m_tdata,
    output logic        m_tlast,         // end of frame
    output logic        m_tvalid,
    input  logic        m_tready
);
    logic [10:0] y;
    wire [10:0] y_now = s_tuser ? 11'd0 : y;

    assign m_tdata  = s_tdata;
    assign m_tvalid = s_tvalid;
    assign s_tready = m_tready;
    assign m_tlast  = s_tlast && (y_now == height - 1);

    always_ff @(posedge clk) begin
        if (rst)
            y <= '0;
        else if (s_tvalid && m_tready && s_tlast)
            y <= m_tlast ? 11'd0 : y_now + 1'b1;
        else if (s_tvalid && m_tready && s_tuser)
            y <= y_now;
    end
endmodule
