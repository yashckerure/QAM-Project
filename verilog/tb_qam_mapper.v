`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.05.2026 19:38:42
// Design Name: 
// Module Name: tb_qam_mapper
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
// tb_qam_mapper.v
// Chain: bit_source -> qam_mapper, sweeping all 5 modes.
// Dumps "I_hex Q_hex" per symbol to per-mode files.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// tb_qam_mapper.v
// Integration TB: bit_source -> symbol_packer -> qam_mapper.
// Tests QPSK and 16-QAM end-to-end.
//
// Outputs:
//   iq_qpsk.txt    (64 lines, "I Q" hex per line)
//   iq_16qam.txt   (64 lines)
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_qam_mapper;

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
        if (rst_n && iq_valid) begin
            $display("[%0t] mode=%0d sym %0d : I=%6d (%h)  Q=%6d (%h)",
                     $time, qam_mode, sym_cnt, i_map, i_map[15:0],
                     q_map, q_map[15:0]);
            $fwrite(fd, "%h %h\n", i_map[15:0], q_map[15:0]);
            sym_cnt <= sym_cnt + 1;
        end
    end

    // Mode sweep with reset between modes
    initial begin
        // ----- Mode 0: QPSK -----
        rst_n    = 1'b0;
        qam_mode = 3'd0;
        sym_cnt  = 0;
        fd       = $fopen("iq_qpsk.txt", "w");
        #50;
        rst_n = 1'b1;

        wait (sym_cnt == NUM_SYMS);
        #20;
        $fclose(fd);
        $display("DONE mode 0 (QPSK)");

        // ----- Reset between modes -----
        rst_n    = 1'b0;
        sym_cnt  = 0;
        qam_mode = 3'd1;
        fd       = $fopen("iq_16qam.txt", "w");
        #50;
        rst_n = 1'b1;

        wait (sym_cnt == NUM_SYMS);
        #20;
        $fclose(fd);
        $display("DONE mode 1 (16-QAM)");

        $finish;
    end

    // Safety timeout
    initial begin
        #1000000;
        $display("WARN: safety timeout");
        $finish;
    end

endmodule
