// Deep single-clock RAM built from 4K-deep banks with a pipelined read mux.
//
// A 230 400 x 12 bit memory inferred as one array becomes 12 columns of
// 32K x 1 cascades (96 BRAM36) with a LUT mux behind un-registered BRAM
// outputs. Splitting it into 4K banks lets each bank use the BRAM output
// register and the bank select is a registered 8:1 + 8:1 tree.
//
// Read latency = 4 enabled cycles: BRAM read, BRAM output register, mux
// level 1, mux level 2. All read-side registers advance on en only.
module bram_banked #(
    parameter int DEPTH = 640 * 360,
    parameter int W     = 12,
    parameter int BANK  = 4096
) (
    input  logic                     clk,
    input  logic                     en,
    input  logic [$clog2(DEPTH)-1:0] raddr,
    output logic [W-1:0]             rdata,
    input  logic                     we,
    input  logic [$clog2(DEPTH)-1:0] waddr,
    input  logic [W-1:0]             wdata
);
    localparam int AW  = $clog2(DEPTH);
    localparam int BW  = $clog2(BANK);
    localparam int NB  = (DEPTH + BANK - 1) / BANK;
    localparam int NG  = (NB + 7) / 8;           // groups of 8 banks
    localparam int SW  = AW - BW;                // bank select width

    logic [W-1:0] q   [NB];
    logic [W-1:0] qr  [NB];

    genvar b;
    generate
        for (b = 0; b < NB; b++) begin : g_bank
            (* ram_style = "block" *) logic [W-1:0] mem [BANK];
            always_ff @(posedge clk) begin
                if (we && waddr[AW-1:BW] == SW'(b))
                    mem[waddr[BW-1:0]] <= wdata;
                if (en) begin
                    q[b]  <= mem[raddr[BW-1:0]];
                    qr[b] <= q[b];
                end
            end
        end
    endgenerate

    // bank select travels with the data
    logic [SW-1:0] s1, s2, s3;
    always_ff @(posedge clk)
        if (en) begin
            s1 <= raddr[AW-1:BW];
            s2 <= s1;
            s3 <= s2;
        end

    logic [W-1:0] g [NG];
    always_ff @(posedge clk)
        if (en)
            for (int i = 0; i < NG; i++) begin
                g[i] <= '0;
                for (int j = 0; j < 8; j++)
                    if (i * 8 + j < NB && s2[2:0] == 3'(j))
                        g[i] <= qr[(i * 8 + j < NB) ? i * 8 + j : 0];
            end

    always_ff @(posedge clk)
        if (en)
            rdata <= g[s3 >> 3];
endmodule
