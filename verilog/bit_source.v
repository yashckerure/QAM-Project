//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.05.2026 19:08:58
// Design Name: 
// Module Name: bit_source
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
// bit_source.v
// 1-bit-per-clock PRBS bit source. Mode-agnostic.
// Default polynomial PRBS-23: x^23 + x^18 + 1 (taps on bits 22 and 17).
// Selectable PRBS-15: x^15 + x^14 + 1 (taps on bits 14 and 13).
// Output is one fresh bit per clock when bit_ready=1.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// bit_source.v
// 1-bit-per-clock PRBS source. Mode-agnostic.
// Output is COMBINATIONAL from the LFSR MSB. The LFSR advances only when
// the consumer asserts bit_ready at the clock edge. This is the standard
// AXI-Stream "ready/valid" producer pattern and avoids the lost-bit race
// that the previous registered-output version had.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module bit_source #(
    parameter integer LFSR_W = 23,
    parameter [LFSR_W-1:0] SEED = 23'h5A3C7E
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                bit_ready,
    output wire                bit_out,
    output wire                bit_valid
);

    reg  [LFSR_W-1:0] lfsr;

    wire fb_23 = lfsr[22] ^ lfsr[17];
    wire fb_15 = lfsr[14] ^ lfsr[13];
    wire fb    = (LFSR_W == 23) ? fb_23 :
                 (LFSR_W == 15) ? fb_15 : 1'b0;

    // Combinational outputs
    assign bit_out   = lfsr[LFSR_W-1];
    assign bit_valid = rst_n;

    // LFSR advances ONLY on consumption (bit_ready high at the edge)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr <= SEED;
        end else if (bit_ready) begin
            lfsr <= {lfsr[LFSR_W-2:0], fb};
        end
    end

endmodule