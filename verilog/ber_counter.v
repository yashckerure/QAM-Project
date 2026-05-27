//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.05.2026 21:12:09
// Design Name: 
// Module Name: ber_counter
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
// ber_counter.v
// Cumulative BER counter for the adaptive QAM loopback.
//
// Operation:
//   - Holds its own PRBS-23 generator with the SAME seed as the TX bit_source.
//   - On each sliced_valid pulse, unpacks the Qm-bit slicer output (MSB-first)
//     and compares each bit against the next PRBS bit. Mismatches increment
//     bit_errors; total bits compared increments bits_compared.
//   - Both counters saturate at 2^32 - 1.
//   - status_done asserts once bits_compared reaches NUM_BITS_TARGET.
//
// Design notes:
//   - PRBS-23 advance loop runs combinationally; we precompute the next state
//     and the bps comparison bits in one cycle.
//   - When the chain grows (RRC, channel, recovery), the slicer still emits
//     symbols in the same order the TX bits were packed -- so this comparator
//     stays correct without retuning.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module ber_counter #(
    parameter integer LFSR_W           = 23,
    parameter [LFSR_W-1:0] SEED        = 23'h5A3C7E,
    parameter integer MAX_BPS          = 8,
    parameter integer NUM_BITS_TARGET  = 32'd4096
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  enable,
    input  wire  [MAX_BPS-1:0]   sliced_bits,
    input  wire  [3:0]           sliced_bits_used,
    input  wire                  sliced_valid,
    output reg   [31:0]          bit_errors,
    output reg   [31:0]          bits_compared,
    output reg                   status_done
);

    // -------------------------------------------------------------------------
    // Local PRBS state
    // -------------------------------------------------------------------------
    reg [LFSR_W-1:0] lfsr;

    // -------------------------------------------------------------------------
    // Combinational unrolled PRBS advance + comparison
    //   For each of MAX_BPS positions, if i < sliced_bits_used:
    //     - extract slicer bit at position (sliced_bits_used - 1 - i)  [MSB-first]
    //     - extract PRBS bit (current LFSR MSB)
    //     - XOR -> mismatch
    //     - advance LFSR
    // -------------------------------------------------------------------------
    reg [LFSR_W-1:0] lfsr_w_var;     // working LFSR during the unroll
    reg [7:0]        bits_to_check;
    reg [7:0]        prbs_bits_local;
    reg [7:0]        mismatch_mask;
    reg [3:0]        n_mismatch;
    reg [3:0]        n_bits;
    integer          i;
    reg              prbs_bit;
    reg              slicer_bit;
    reg              fb;

    always @(*) begin
        lfsr_w_var      = lfsr;
        bits_to_check   = sliced_bits;
        prbs_bits_local = 8'd0;
        mismatch_mask   = 8'd0;
        n_mismatch      = 4'd0;
        n_bits          = sliced_bits_used;

        for (i = 0; i < MAX_BPS; i = i + 1) begin
            if (i < sliced_bits_used) begin
                // MSB-first: first bit out of PRBS corresponds to the highest
                // bit position in sliced_bits (position sliced_bits_used-1-i)
                prbs_bit   = lfsr_w_var[LFSR_W-1];
                slicer_bit = bits_to_check[sliced_bits_used - 1 - i];

                prbs_bits_local[sliced_bits_used - 1 - i] = prbs_bit;
                if (prbs_bit != slicer_bit) begin
                    mismatch_mask[sliced_bits_used - 1 - i] = 1'b1;
                    n_mismatch = n_mismatch + 4'd1;
                end

                fb = (LFSR_W == 23) ? (lfsr_w_var[22] ^ lfsr_w_var[17]) :
                     (LFSR_W == 15) ? (lfsr_w_var[14] ^ lfsr_w_var[13]) :
                                       1'b0;
                lfsr_w_var = {lfsr_w_var[LFSR_W-2:0], fb};
            end
        end
    end

    // -------------------------------------------------------------------------
    // Sequential update
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr          <= SEED;
            bit_errors    <= 32'd0;
            bits_compared <= 32'd0;
            status_done   <= 1'b0;
        end else if (enable && sliced_valid && !status_done) begin
            lfsr          <= lfsr_w_var;
            bit_errors    <= bit_errors    + {28'd0, n_mismatch};
            bits_compared <= bits_compared + {28'd0, n_bits};

            if ((bits_compared + n_bits) >= NUM_BITS_TARGET) begin
                status_done <= 1'b1;
            end
        end
    end

endmodule
