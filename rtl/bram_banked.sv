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

    // Write port: registered twice, the second time once per group of 8 banks,
    // so no single register drives all ~57 BRAMs spread over the die. A write
    // lands 2 cycles after it is presented; callers must not read an address
    // within 2 cycles of writing it.
    logic          we1;
    logic [AW-1:0] wa1;
    logic [W-1:0]  wd1;
    always_ff @(posedge clk) begin
        we1 <= we; wa1 <= waddr; wd1 <= wdata;
    end
    (* equivalent_register_removal = "no" *) logic          we2 [NG];
    (* equivalent_register_removal = "no" *) logic [AW-1:0] wa2 [NG];
    (* equivalent_register_removal = "no" *) logic [W-1:0]  wd2 [NG];
    always_ff @(posedge clk)
        for (int i = 0; i < NG; i++) begin
            we2[i] <= we1 && (wa1[AW-1:BW+3] == (AW-BW-3)'(i));
            wa2[i] <= wa1;
            wd2[i] <= wd1;
        end

    genvar b;
    generate
        for (b = 0; b < NB; b++) begin : g_bank
            (* ram_style = "block" *) logic [W-1:0] mem [BANK];
            always_ff @(posedge clk) begin
                if (we2[b / 8] && wa2[b / 8][BW+2:BW] == 3'(b % 8))
                    mem[wa2[b / 8][BW-1:0]] <= wd2[b / 8];
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
