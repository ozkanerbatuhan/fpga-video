// Small synchronous FIFO (distributed RAM) with a registered "almost empty
// enough" flag for credit-based flow control.
module axis_fifo #(
    parameter int W     = 8,
    parameter int DEPTH = 16,       // power of two
    parameter int SPACE = 8         // room flag: at least SPACE free entries
) (
    input  logic         clk,
    input  logic         rst,
    input  logic [W-1:0] s_data,
    input  logic         s_valid,   // caller guarantees room (credit)
    output logic         room,
    output logic [W-1:0] m_data,
    output logic         m_valid,
    input  logic         m_ready
);
    localparam int AW = $clog2(DEPTH);
    (* ram_style = "distributed" *) logic [W-1:0] mem [DEPTH];
    logic [AW-1:0] wp, rp;
    logic [AW:0]   cnt;

    wire rd = m_valid && m_ready;
    assign m_valid = (cnt != 0);
    assign m_data  = mem[rp];

    always_ff @(posedge clk) begin
        if (s_valid) mem[wp] <= s_data;
        if (rst) begin
            wp <= '0; rp <= '0; cnt <= '0; room <= 1'b1;
        end else begin
            if (s_valid) wp <= wp + 1'b1;
            if (rd)      rp <= rp + 1'b1;
            cnt  <= cnt + s_valid - rd;
            room <= (DEPTH - (cnt + s_valid - rd)) >= SPACE;
        end
    end
endmodule
