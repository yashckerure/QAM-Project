//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.05.2026 21:10:52
// Design Name: 
// Module Name: tb_ber_counter
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
// tb_ber_counter.v
// Full bit-pipeline TB: zero-BER loopback (no channel, no filter).
//
// Chain: bit_source -> symbol_packer -> qam_mapper -> qam_slicer -> ber_counter
//
// Sweeps QPSK then 16-QAM, target 4096 bits compared per mode.
// Pass criterion: bit_errors == 0 for both modes.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_ber_counter;

    localparam integer SPS              = 4;
    localparam integer MAX_BPS          = 8;
    localparam integer DATA_W           = 16;
    localparam integer NUM_BITS_TARGET  = 32'd4096;

    reg                       clk;
    reg                       rst_n;
    reg                       sym_en;
    reg  [2:0]                qam_mode;
    reg                       ber_enable;

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

    wire [31:0]               bit_errors;
    wire [31:0]               bits_compared;
    wire                      status_done;

    bit_source #(.LFSR_W(23), .SEED(23'h5A3C7E)) u_src (
        .clk(clk), .rst_n(rst_n), .bit_ready(bit_ready),
        .bit_out(bit_out), .bit_valid(bit_valid)
    );

    symbol_packer #(.MAX_BPS(MAX_BPS)) u_pack (
        .clk(clk), .rst_n(rst_n), .qam_mode(qam_mode), .sym_en(sym_en),
        .bit_in(bit_out), .bit_valid(bit_valid), .bit_ready(bit_ready),
        .sym_bits(sym_bits), .bits_used(bits_used), .sym_valid(sym_valid)
    );

    qam_mapper #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_map (
        .clk(clk), .rst_n(rst_n), .qam_mode(qam_mode),
        .sym_bits(sym_bits), .bits_used(bits_used), .sym_valid(sym_valid),
        .i_out(i_map), .q_out(q_map), .iq_valid(iq_valid)
    );

    qam_slicer #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_slice (
        .clk(clk), .rst_n(rst_n), .qam_mode(qam_mode),
        .i_in(i_map), .q_in(q_map), .iq_valid(iq_valid),
        .sym_bits(sliced_bits), .bits_used(sliced_bits_used),
        .sym_valid(sliced_valid)
    );

    ber_counter #(
        .LFSR_W          (23),
        .SEED            (23'h5A3C7E),
        .MAX_BPS         (MAX_BPS),
        .NUM_BITS_TARGET (NUM_BITS_TARGET)
    ) u_ber (
        .clk             (clk),
        .rst_n           (rst_n),
        .enable          (ber_enable),
        .sliced_bits     (sliced_bits),
        .sliced_bits_used(sliced_bits_used),
        .sliced_valid    (sliced_valid),
        .bit_errors      (bit_errors),
        .bits_compared   (bits_compared),
        .status_done     (status_done)
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

    integer fd;

    initial begin
        fd         = $fopen("ber_report.txt", "w");
        ber_enable = 1'b0;

        // ----- QPSK -----
        rst_n      = 1'b0;
        qam_mode   = 3'd0;
        #50;
        rst_n      = 1'b1;
        ber_enable = 1'b1;
        wait (status_done);
        #20;
        ber_enable = 1'b0;
        $display("QPSK    : errors=%0d bits=%0d", bit_errors, bits_compared);
        $fwrite(fd, "QPSK    : errors=%0d bits=%0d\n", bit_errors, bits_compared);

        // ----- 16-QAM -----
        rst_n      = 1'b0;
        qam_mode   = 3'd1;
        #50;
        rst_n      = 1'b1;
        ber_enable = 1'b1;
        wait (status_done);
        #20;
        ber_enable = 1'b0;
        $display("16-QAM  : errors=%0d bits=%0d", bit_errors, bits_compared);
        $fwrite(fd, "16-QAM  : errors=%0d bits=%0d\n", bit_errors, bits_compared);

        $fclose(fd);
        $finish;
    end

    // Safety timeout
    initial begin
        #2000000;
        $display("WARN: safety timeout");
        $finish;
    end

endmodule