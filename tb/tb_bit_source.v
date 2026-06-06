//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.05.2026 19:11:40
// Design Name: 
// Module Name: tb_bit_source
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
// tb_bit_source.v
// Streams 4096 PRBS bits from bit_source, dumps to bits.txt one bit per line.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// tb_bit_source.v
// Streams 4096 PRBS bits from bit_source, dumps to bits.txt one bit per line.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_bit_source;

    localparam integer NUM_BITS = 4096;

    reg  clk;
    reg  rst_n;
    reg  bit_ready;
    wire bit_out;
    wire bit_valid;

    bit_source #(.LFSR_W(23), .SEED(23'h5A3C7E)) dut (
        .clk(clk), .rst_n(rst_n),
        .bit_ready(bit_ready),
        .bit_out(bit_out), .bit_valid(bit_valid)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer fd;
    integer bit_cnt;

    initial begin
        rst_n     = 1'b0;
        bit_ready = 1'b1;             // free-running for now
        bit_cnt   = 0;
        fd = $fopen("bits.txt", "w");
        #50;
        rst_n = 1'b1;
    end

    always @(posedge clk) begin
        if (rst_n && bit_valid) begin
            $fwrite(fd, "%0d\n", bit_out);
            bit_cnt <= bit_cnt + 1;
            if (bit_cnt == NUM_BITS-1) begin
                $fclose(fd);
                $display("DONE: %0d bits dumped to bits.txt", NUM_BITS);
                $finish;
            end
        end
    end

    initial begin
        #(NUM_BITS * 10 + 1000);
        $display("WARN: safety timeout");
        $finish;
    end

endmodule