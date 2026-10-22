// Generic stream testbench for every block of the pipeline.
//
// Input  file (+IN=...):  one beat per line, "DATA USER LAST" in hex.
// Output file (+OUT=...): same format, one line per accepted output beat.
// Boxes  file (+BOX=...): for ccl/top, one line per published frame:
//         "n_boxes n_labels overflow x0 y0 x1 y1 area ..." (K boxes, decimal).
//
// Stress: the source drops tvalid between beats with probability (100-PV)%,
// the sink drops tready with probability (100-PR)%, both from $urandom(SEED).
// Once tvalid is high the beat is held until accepted (AXI4-Stream rule).
`timescale 1ns/1ps
module tb_block;
    // DUT is chosen by number: xelab's -generic_top mangles some string values
    parameter int    DUTN = 0;
    localparam string DUT = DUTN == 0 ? "gauss"  : DUTN == 1 ? "sobel"     : DUTN == 2 ? "erode"    :
                            DUTN == 3 ? "dilate" : DUTN == 4 ? "motion"    : DUTN == 5 ? "ccl"      :
                            DUTN == 6 ? "overlay": DUTN == 7 ? "luma_yuyv" : DUTN == 8 ? "luma_rgb" :
                            DUTN == 9 ? "upscale": "top";
    parameter        FONT = "font8x8.mem";

    logic clk = 0, rst = 1;
    always #3.367 clk = ~clk;         // 148.5 MHz

    // ---------------- configuration ----------------
    int W = 64, H = 48, THR = 20, ALPHA = 4, SSH = 1, MINA = 40, MODE = 0, OVL = 1;
    int PV = 100, PR = 100, SEED = 1, NOUT = -1, TIMEOUT = 2000000;
    int TX = 8, TY = 8, BGRESET_AT = -1, rc, OBITS = 64;
    logic [63:0] omask;
    string fin, fout, fbox, text_s;
    logic [127:0] text_v;

    // ---------------- DUT ports (wide; each DUT uses a slice) ----------------
    logic [63:0] s_tdata, m_tdata;
    logic s_tuser, s_tlast, s_tvalid, s_tready;
    logic m_tuser, m_tlast, m_tvalid, m_tready;

    localparam int K = 8;
    logic [K*11-1:0] box_x0, box_y0, box_x1, box_y1;
    logic [K*18-1:0] box_area;
    logic [3:0]  n_boxes;
    logic [10:0] n_labels;
    logic        overflow, done;
    logic        bg_reset = 0;
    logic [K*11-1:0] ov_x0 = 0, ov_y0 = 0, ov_x1 = 0, ov_y1 = 0;   // overlay test boxes
    logic [3:0]      ov_n = 0;
    logic [31:0]     rv;
    string           fovb;
    int              fd_ovb, ovn, bx0, by0, bx1, by1;

    // AXI-Lite (top only)
    logic [7:0]  awaddr = 0, araddr = 0;
    logic        awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
    logic [31:0] wdata = 0, rdata;
    logic [3:0]  wstrb = 4'hF;
    logic        awready, wready, bvalid, arready, rvalid;
    logic [1:0]  bresp, rresp;

    generate
        if (DUT == "gauss") begin : d
            win3x3_filter #(.DW(8), .SW(16), .OW(8), .KERNEL("GAUSS")) u (
                .clk, .rst, .width(11'(W)), .height(11'(H)), .sobel_shift(2'(SSH)),
                .s_tdata(s_tdata[23:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[23:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .err_sync(), .err_sof(), .err_eol());
        end else if (DUT == "sobel") begin : d
            win3x3_filter #(.DW(8), .SW(16), .OW(8), .KERNEL("SOBEL")) u (
                .clk, .rst, .width(11'(W)), .height(11'(H)), .sobel_shift(2'(SSH)),
                .s_tdata(s_tdata[23:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[23:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .err_sync(), .err_sof(), .err_eol());
        end else if (DUT == "erode") begin : d
            win3x3_filter #(.DW(1), .SW(16), .OW(1), .KERNEL("ERODE")) u (
                .clk, .rst, .width(11'(W)), .height(11'(H)), .sobel_shift(2'(SSH)),
                .s_tdata(s_tdata[16:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[16:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .err_sync(), .err_sof(), .err_eol());
        end else if (DUT == "dilate") begin : d
            win3x3_filter #(.DW(1), .SW(16), .OW(1), .KERNEL("DILATE")) u (
                .clk, .rst, .width(11'(W)), .height(11'(H)), .sobel_shift(2'(SSH)),
                .s_tdata(s_tdata[16:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[16:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .err_sync(), .err_sof(), .err_eol());
        end else if (DUT == "motion") begin : d
            motion_detect #(.SW(16), .DEPTH(640 * 360)) u (
                .clk, .rst, .thresh(8'(THR)), .alpha_shift(4'(ALPHA)), .bg_reset,
                .s_tdata(s_tdata[23:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[24:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .err_overflow());
        end else if (DUT == "ccl") begin : d
            ccl_bbox #(.SW(16), .K(K)) u (
                .clk, .rst, .width(11'(W)), .height(11'(H)), .min_area(18'(MINA)),
                .s_tdata(s_tdata[16:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[16:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready,
                .box_x0, .box_y0, .box_x1, .box_y1, .box_area, .n_boxes, .n_labels,
                .overflow, .done, .err_sof());
        end else if (DUT == "overlay") begin : d
            overlay #(.K(K), .FONT_FILE(FONT)) u (
                .clk, .rst, .mode(3'(MODE)), .overlay_en(OVL[0]), .box_color(24'h22_36_91),
                .text(text_v), .text_x(11'(TX)), .text_y(11'(TY)),
                .box_x0(ov_x0), .box_y0(ov_y0), .box_x1(ov_x1), .box_y1(ov_y1), .n_boxes(ov_n),
                .s_tdata(s_tdata[33:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[15:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready);
        end else if (DUT == "luma_yuyv") begin : d
            luma #(.FMT("YUYV")) u (
                .clk, .rst, .s_tdata(s_tdata[15:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[23:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready);
        end else if (DUT == "luma_rgb") begin : d
            luma #(.FMT("RGB")) u (
                .clk, .rst, .s_tdata(s_tdata[23:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[31:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready);
        end else if (DUT == "upscale") begin : d
            upscale2x #(.IN_W(640)) u (
                .clk, .rst, .s_tdata(s_tdata[15:0]), .s_tuser, .s_tlast, .s_tvalid, .s_tready,
                .m_tdata(m_tdata[15:0]), .m_tuser, .m_tlast, .m_tvalid, .m_tready);
        end else if (DUT == "top") begin : d
            // DMA-style stream: tlast only at end of frame, no tuser
            vproc_top #(.FONT_FILE(FONT)) u (
                .clk, .rst,
                .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
                .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
                .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
                .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
                .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
                .s_axis_tdata(s_tdata[15:0]), .s_axis_tlast(s_tlast), .s_axis_tvalid(s_tvalid),
                .s_axis_tready(s_tready),
                .m_axis_tdata(m_tdata[15:0]), .m_axis_tlast(m_tlast), .m_axis_tvalid(m_tvalid),
                .m_axis_tready(m_tready));
            assign m_tuser = 1'b0;
            assign done = u.ccl_done;
            assign box_x0 = u.box_x0; assign box_y0 = u.box_y0;
            assign box_x1 = u.box_x1; assign box_y1 = u.box_y1;
            assign box_area = u.box_area; assign n_boxes = u.n_boxes;
            assign n_labels = u.n_labels; assign overflow = u.overflow;
        end
    endgenerate

    // ---------------- AXI-Lite master (top) ----------------
    task automatic axil_write(input logic [7:0] a, input logic [31:0] d);
        @(posedge clk);
        awaddr <= a; awvalid <= 1; wdata <= d; wvalid <= 1;
        fork
            begin do @(posedge clk); while (!awready); awvalid <= 0; end
            begin do @(posedge clk); while (!wready);  wvalid  <= 0; end
        join
        do @(posedge clk); while (!bvalid);
    endtask

    task automatic axil_read(input logic [7:0] a, output logic [31:0] d);
        @(posedge clk);
        araddr <= a; arvalid <= 1;
        do @(posedge clk); while (!arready);
        arvalid <= 0;
        do @(posedge clk); while (!rvalid);
        d = rdata;
    endtask

    // ---------------- source ----------------
    int fd_in, fd_out, fd_box, n_in = 0, n_out = 0, n_frames = 0;
    logic [63:0] d_in; logic u_in, l_in;
    logic        src_done = 0;

    initial begin
        if (!$value$plusargs("IN=%s", fin)) $fatal(1, "+IN missing");
        if (!$value$plusargs("OUT=%s", fout)) $fatal(1, "+OUT missing");
        rc = $value$plusargs("BOX=%s", fbox);
        rc = $value$plusargs("W=%d", W);        rc = $value$plusargs("H=%d", H);
        rc = $value$plusargs("THR=%d", THR);    rc = $value$plusargs("ALPHA=%d", ALPHA);
        rc = $value$plusargs("SSH=%d", SSH);    rc = $value$plusargs("MINA=%d", MINA);
        rc = $value$plusargs("MODE=%d", MODE);  rc = $value$plusargs("OVL=%d", OVL);
        rc = $value$plusargs("PV=%d", PV);      rc = $value$plusargs("PR=%d", PR);
        rc = $value$plusargs("SEED=%d", SEED);  rc = $value$plusargs("NOUT=%d", NOUT);
        rc = $value$plusargs("TX=%d", TX);      rc = $value$plusargs("TY=%d", TY);
        rc = $value$plusargs("TIMEOUT=%d", TIMEOUT);
        rc = $value$plusargs("OBITS=%d", OBITS);
        omask = (OBITS >= 64) ? {64{1'b1}} : ((64'd1 << OBITS) - 1);
        rc = $value$plusargs("BGRESET=%d", BGRESET_AT);
        text_s = "";
        rc = $value$plusargs("TEXT=%s", text_s);
        text_v = '0;
        for (int i = 0; i < text_s.len() && i < 16; i++)
            text_v[i*8 +: 8] = (text_s[i] == "_") ? 8'h20 : text_s[i];
        rc = $urandom(SEED);
        if ($value$plusargs("OVB=%s", fovb)) begin
            fd_ovb = $fopen(fovb, "r");
            rc = $fscanf(fd_ovb, "%d", ovn);
            ov_n = 4'(ovn);
            for (int i = 0; i < ovn; i++) begin
                rc = $fscanf(fd_ovb, "%d %d %d %d", bx0, by0, bx1, by1);
                ov_x0[i*11 +: 11] = 11'(bx0); ov_y0[i*11 +: 11] = 11'(by0);
                ov_x1[i*11 +: 11] = 11'(bx1); ov_y1[i*11 +: 11] = 11'(by1);
            end
            $fclose(fd_ovb);
        end

        fd_in  = $fopen(fin, "r");
        fd_out = $fopen(fout, "w");
        if (fd_in == 0 || fd_out == 0) $fatal(1, "cannot open files");
        if (fbox != "") fd_box = $fopen(fbox, "w");

        s_tvalid = 0; s_tdata = 0; s_tuser = 0; s_tlast = 0;
        repeat (10) @(posedge clk);
        rst <= 0;
        repeat (5) @(posedge clk);

        if (DUT == "top") begin
            axil_write(8'h04, {5'd0, 11'(H), 5'd0, 11'(W)});
            axil_write(8'h08, {14'd0, 2'(SSH), 4'd0, 4'(ALPHA), 8'(THR)});
            axil_write(8'h0C, 32'(MINA));
            axil_write(8'h10, 32'h0022_3691);
            axil_write(8'h14, {5'd0, 11'(TY), 5'd0, 11'(TX)});
            axil_write(8'h20, text_v[31:0]);  axil_write(8'h24, text_v[63:32]);
            axil_write(8'h28, text_v[95:64]); axil_write(8'h2C, text_v[127:96]);
            axil_write(8'h00, {27'd0, OVL[0], 1'b0, 3'(MODE)});
        end

        while (!$feof(fd_in)) begin
            if ($fscanf(fd_in, "%h %h %h\n", d_in, u_in, l_in) != 3) break;
            // frame index for an optional background reset request
            if (u_in && BGRESET_AT >= 0 && n_frames == BGRESET_AT) begin
                if (DUT == "top") axil_write(8'h00, {23'd0, 1'b1, 3'd0, OVL[0], 1'b0, 3'(MODE)});
                else begin @(posedge clk); bg_reset <= 1; @(posedge clk); bg_reset <= 0; end
            end
            if (u_in) n_frames++;
            while (($urandom % 100) >= PV) @(posedge clk);
            s_tdata <= d_in; s_tuser <= u_in; s_tlast <= l_in; s_tvalid <= 1;
            do @(posedge clk); while (!s_tready);
            s_tvalid <= 0;
            n_in++;
        end
        src_done = 1;
    end

    // ---------------- sink ----------------
    always @(posedge clk) begin
        m_tready <= rst ? 1'b0 : (($urandom % 100) < PR);
        if (!rst && m_tvalid && m_tready) begin
            $fwrite(fd_out, "%h %h %h\n", m_tdata & omask, m_tuser, m_tlast);
            n_out++;
        end
    end

    // ---------------- boxes ----------------
    always @(posedge clk) begin
        if (!rst && done && fbox != "") begin
            $fwrite(fd_box, "%0d %0d %0d", n_boxes, n_labels, overflow);
            for (int i = 0; i < K; i++)
                $fwrite(fd_box, " %0d %0d %0d %0d %0d", box_x0[i*11 +: 11], box_y0[i*11 +: 11],
                        box_x1[i*11 +: 11], box_y1[i*11 +: 11], box_area[i*18 +: 18]);
            $fwrite(fd_box, "\n");
        end
    end

    // ---------------- end ----------------
    int idle = 0;
    always @(posedge clk) begin
        if (m_tvalid && m_tready) idle <= 0; else idle <= idle + 1;
        if ((NOUT >= 0 && n_out >= NOUT) || (src_done && idle > 20000) || idle > TIMEOUT) begin
            repeat (50) @(posedge clk);
            if (DUT == "top") begin
                axil_read(8'h38, rv); $display("ERR=%h", rv);
                axil_read(8'h34, rv); $display("FRAMES=%0d", rv);
                axil_read(8'h3C, rv); $display("CYCLES=%0d", rv);
                axil_read(8'h18, rv); $display("VERSION=%h", rv);
            end
            $display("TB_DONE in=%0d out=%0d", n_in, n_out);
            $fclose(fd_out);
            if (fbox != "") $fclose(fd_box);
            $finish;
        end
    end
endmodule
