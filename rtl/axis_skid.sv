// Two-entry skid buffer (AXI4-Stream register slice).
// Breaks the combinational tready path: s_ready is a register.
module axis_skid #(
    parameter int W = 8
) (
    input  logic         clk,
    input  logic         rst,
    input  logic [W-1:0] s_data,
    input  logic         s_valid,
    output logic         s_ready,
    output logic [W-1:0] m_data,
    output logic         m_valid,
    input  logic         m_ready
);
    logic [W-1:0] skid;
    logic         skid_valid;

    assign s_ready = ~skid_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            m_valid    <= 1'b0;
            skid_valid <= 1'b0;
        end else begin
            if (m_ready || !m_valid) begin
                // Output register is free (or being drained this cycle).
                if (skid_valid) begin
                    m_data     <= skid;
                    m_valid    <= 1'b1;
                    skid_valid <= 1'b0;
                end else begin
                    m_data  <= s_data;
                    m_valid <= s_valid;
                end
            end else if (s_valid && s_ready) begin
                // Output stalled: park the incoming beat.
                skid       <= s_data;
                skid_valid <= 1'b1;
            end
        end
    end
endmodule
