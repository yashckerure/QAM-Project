
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.05.2026 21:03:20
// Design Name: 
// Module Name: tb_qam_slicer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// tb_qam_slicer.v
// Integration TB: bit_source -> symbol_packer -> qam_mapper -> qam_slicer.
// With no channel between mapper and slicer, the sliced symbols must match
// the packed symbols exactly. This is the deterministic zero-BER check.
//
// Dumps slicer output to:
//   sliced_qpsk.txt   (64 hex lines)
//   sliced_16qam.txt  (64 hex lines)
//
// These files should be IDENTICAL to syms_qpsk.txt / syms_16qam.txt that
// the symbol_packer TB produced.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_qam_slicer;

    localparam integer SPS        = 4;
    localparam integer NUM_SYMS   = 64;
    localparam integer MAX_BPS    = 8;
    localparam integer DATA_W     = 16;

    reg                       clk;
    reg                       rst_n;
    reg                       sym_en;
    reg  [2:0]                qam_mode;

    wire                      bit_out;
    wire                      bit_valid;
    wire                      bit_ready;

    wire [MAX_BPS-1:0]        sym_bits;
    wire [3:0]                bits_used;
    wire                      sym_valid;

    wire signed [DATA_W-1:0]  i_map;
    wire signed [DATA_W-1:0]  q_map;
    wire                      iq_valid;

    wire [MAX_BPS-1:0]        sliced_bits;
    wire [3:0]                sliced_bits_used;
    wire                      sliced_valid;

    bit_source #(.LFSR_W(23), .SEED(23'h5A3C7E)) u_src (
        .clk       (clk),
        .rst_n     (rst_n),
        .bit_ready (bit_ready),
        .bit_out   (bit_out),
        .bit_valid (bit_valid)
    );

    symbol_packer #(.MAX_BPS(MAX_BPS)) u_pack (
        .clk       (clk),
        .rst_n     (rst_n),
        .qam_mode  (qam_mode),
        .sym_en    (sym_en),
        .bit_in    (bit_out),
        .bit_valid (bit_valid),
        .bit_ready (bit_ready),
        .sym_bits  (sym_bits),
        .bits_used (bits_used),
        .sym_valid (sym_valid)
    );

    qam_mapper #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_map (
        .clk       (clk),
        .rst_n     (rst_n),
        .qam_mode  (qam_mode),
        .sym_bits  (sym_bits),
        .bits_used (bits_used),
        .sym_valid (sym_valid),
        .i_out     (i_map),
        .q_out     (q_map),
        .iq_valid  (iq_valid)
    );

    qam_slicer #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_slice (
        .clk       (clk),
        .rst_n     (rst_n),
        .qam_mode  (qam_mode),
        .i_in      (i_map),
        .q_in      (q_map),
        .iq_valid  (iq_valid),
        .sym_bits  (sliced_bits),
        .bits_used (sliced_bits_used),
        .sym_valid (sliced_valid)
    );

    // 100 MHz clock
    initial clk = 0;
    always #5 clk = ~clk;

    // sym_en generator
    integer clk_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 0;
            sym_en  <= 1'b0;
        end else begin
            if (clk_cnt == SPS-1) begin
                clk_cnt <= 0;
                sym_en  <= 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1;
                sym_en  <= 1'b0;
            end
        end
    end

    // Capture
    integer fd;
    integer sym_cnt;

    always @(posedge clk) begin
        if (rst_n && sliced_valid) begin
            $display("[%0t] mode=%0d sym %0d : sliced = %h",
                     $time, qam_mode, sym_cnt,
                     sliced_bits & ((1 << sliced_bits_used) - 1));
            case (sliced_bits_used)
                4'd2:  $fwrite(fd, "%01h\n", sliced_bits[1:0]);
                4'd4:  $fwrite(fd, "%01h\n", sliced_bits[3:0]);
                default: ;
            endcase
            sym_cnt <= sym_cnt + 1;
        end
    end

    // Mode sweep
    initial begin
        // QPSK
        rst_n    = 1'b0;
        qam_mode = 3'd0;
        sym_cnt  = 0;
        fd       = $fopen("sliced_qpsk.txt", "w");
        #50;
        rst_n = 1'b1;
        wait (sym_cnt == NUM_SYMS);
        #20;
        $fclose(fd);
        $display("DONE mode 0 (QPSK)");

        // 16-QAM
        rst_n    = 1'b0;
        sym_cnt  = 0;
        qam_mode = 3'd1;
        fd       = $fopen("sliced_16qam.txt", "w");
        #50;
        rst_n = 1'b1;
        wait (sym_cnt == NUM_SYMS);
        #20;
        $fclose(fd);
        $display("DONE mode 1 (16-QAM)");

        $finish;
    end

    initial begin
        #1000000;
        $display("WARN: safety timeout");
        $finish;
    end

endmodule
