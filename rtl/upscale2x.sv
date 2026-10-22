// 2x nearest-neighbour upscaler for YUYV video (display path, 640x360 -> 1280x720).
//
// A line is stored as pixel pairs {Y1, Cr, Y0, Cb}; it is then emitted twice,
// each source pixel twice. Chroma stays 4:2:2-correct: output pixel j uses
// Y[j/2] and Cb of the pair on even j, Cr on odd j.
// Two line buffers: the next line is written while the current one is emitted.
module upscale2x #(
    parameter int IN_W = 640
) (
    input  logic        clk,
    input  logic        rst,
    input  logic [15:0] s_tdata,
    input  logic        s_tuser,
    input  logic        s_tlast,
    input  logic        s_tvalid,
    output logic        s_tready,
    output logic [15:0] m_tdata,
    output logic        m_tuser,
    output logic        m_tlast,
    output logic        m_tvalid,
    input  logic        m_tready
);
    localparam int PW = $clog2(IN_W / 2);
    localparam int OW = $clog2(IN_W * 2);

    (* ram_style = "block" *) logic [31:0] buf0 [IN_W/2];
    (* ram_style = "block" *) logic [31:0] buf1 [IN_W/2];

    // ---------------- write side ----------------
    logic        wsel;
    logic [1:0]  full;
    logic [1:0]  sof;              // line in buffer is the first of a frame
    logic [10:0] wx;
    logic [15:0] even_px;
    logic        line_sof;
    logic        rsel;
    logic        rd_done;          // read side releases buffer rsel

    assign s_tready = !full[wsel];
    wire take = s_tvalid && s_tready;

    always_ff @(posedge clk) begin
        if (take && wx[0]) begin
            if (wsel) buf1[wx[PW:1]] <= {s_tdata, even_px};
            else      buf0[wx[PW:1]] <= {s_tdata, even_px};
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            wsel <= 1'b0; full <= 2'b00; wx <= '0; line_sof <= 1'b0;
        end else begin
            if (rd_done) full[rsel] <= 1'b0;
            if (take) begin
                if (!wx[0]) even_px <= s_tdata;
                if (s_tuser) line_sof <= 1'b1;
                if (s_tlast) begin
                    full[wsel] <= 1'b1;
                    sof[wsel]  <= line_sof || s_tuser;
                    line_sof   <= 1'b0;
                    wsel <= ~wsel;
                    wx   <= '0;
                end else
                    wx <= wx + 1'b1;
            end
        end
    end

    // ---------------- read side ----------------
    logic en, skid_ready;
    logic [OW-1:0] j;
    logic          pass;           // 0: first copy of the line, 1: second
    wire           active = full[rsel];
    wire           last_j = (j == OW'(IN_W * 2 - 1));
    wire           issue  = en && active;
    assign rd_done = issue && last_j && pass;

    logic [31:0] q;
    always_ff @(posedge clk)
        if (issue) q <= rsel ? buf1[j[OW-1:2]] : buf0[j[OW-1:2]];

    always_ff @(posedge clk) begin
        if (rst) begin
            rsel <= 1'b0; j <= '0; pass <= 1'b0;
        end else if (issue) begin
            if (last_j) begin
                j <= '0;
                pass <= ~pass;
                if (pass) rsel <= ~rsel;
            end else
                j <= j + 1'b1;
        end
    end

    logic p1_valid, p1_user, p1_last, p1_odd, p1_hi;
    always_ff @(posedge clk) begin
        if (rst)
            p1_valid <= 1'b0;
        else if (en) begin
            p1_valid <= issue;
            p1_user  <= sof[rsel] && !pass && j == 0;
            p1_last  <= last_j;
            p1_odd   <= j[0];
            p1_hi    <= j[1];
        end
    end

    // q = {C1(Cr), Y1, C0(Cb), Y0}
    wire [7:0] oy = p1_hi  ? q[23:16] : q[7:0];
    wire [7:0] oc = p1_odd ? q[31:24] : q[15:8];

    logic        o_valid, o_user, o_last;
    logic [15:0] o_data;
    assign en = !o_valid || skid_ready;
    always_ff @(posedge clk) begin
        if (rst)
            o_valid <= 1'b0;
        else if (en) begin
            o_valid <= p1_valid;
            o_user  <= p1_user;
            o_last  <= p1_last;
            o_data  <= {oc, oy};
        end
    end

    axis_skid #(.W(18)) u_skid (
        .clk, .rst,
        .s_data ({o_user, o_last, o_data}), .s_valid(o_valid), .s_ready(skid_ready),
        .m_data ({m_tuser, m_tlast, m_tdata}), .m_valid(m_tvalid), .m_ready(m_tready)
    );
endmodule
