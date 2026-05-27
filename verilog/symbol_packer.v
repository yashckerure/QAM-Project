`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.05.2026 20:07:09
// Design Name: 
// Module Name: symbol_packer
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
// symbol_packer.v
// Accumulate 1-bit-per-clock stream into Qm-bit symbols.
// Qm set by qam_mode: 0=QPSK(2), 1=16QAM(4), 2=64QAM(6), 3=256QAM(8).
// Bits are packed MSB-first: first bit in becomes the MSB of sym_bits.
//
// Output sym_bits is emitted with sym_valid pulse, aligned to sym_en pulse
// from top-level (which marks symbol boundaries at SPS=4 cadence).
//
// Backpressure: bit_ready is deasserted when the accumulator already has
// Qm bits and is waiting for sym_en to release them. This is the standard
// AXI-Stream valid/ready handshake. When FEC chain is inserted upstream,
// bit_ready connects to the new upstream block; semantics unchanged.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module symbol_packer #(
    parameter integer MAX_BPS = 8                   // max bits per symbol (256-QAM)
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire  [2:0]           qam_mode,          // 0..3 -> QPSK..256-QAM
    input  wire                  sym_en,            // 1-clock pulse, symbol boundary
    // upstream interface (from bit_source or, later, from FEC chain)
    input  wire                  bit_in,
    input  wire                  bit_valid,
    output reg                   bit_ready,
    // downstream interface (to qam_mapper)
    output reg   [MAX_BPS-1:0]   sym_bits,
    output reg   [3:0]           bits_used,
    output reg                   sym_valid
);

    // -------------------------------------------------------------------------
    // Decode mode -> bits per symbol
    // -------------------------------------------------------------------------
    reg [3:0] bps;
    always @(*) begin
        case (qam_mode)
            3'd0:    bps = 4'd2;     // QPSK
            3'd1:    bps = 4'd4;     // 16-QAM
            3'd2:    bps = 4'd6;     // 64-QAM
            3'd3:    bps = 4'd8;     // 256-QAM
            default: bps = 4'd4;     // safe default
        endcase
    end

    // -------------------------------------------------------------------------
    // Accumulator state
    //   acc       : holds bits as they arrive, MSB-first
    //   acc_count : how many bits currently held (0..bps)
    //   full      : 1 when acc_count == bps (ready to emit)
    // -------------------------------------------------------------------------
    reg [MAX_BPS-1:0] acc;
    reg [3:0]         acc_count;
    wire              full = (acc_count == bps);

    // -------------------------------------------------------------------------
    // bit_ready: accept a new bit unless we are full and waiting for sym_en
    // -------------------------------------------------------------------------
    always @(*) begin
        bit_ready = !full;
    end

    // -------------------------------------------------------------------------
    // Main logic
    //   - On every clock where bit_valid && bit_ready, shift bit_in into acc
    //     at position (bps-1-acc_count). First bit goes to MSB, second to
    //     next-MSB, and so on.
    //   - On sym_en, if full, emit sym_bits and reset accumulator.
    //   - If sym_en arrives while not full (mode change or starvation), emit
    //     zeros with sym_valid low - mapper will see invalid and skip.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc        <= {MAX_BPS{1'b0}};
            acc_count  <= 4'd0;
            sym_bits   <= {MAX_BPS{1'b0}};
            bits_used  <= 4'd0;
            sym_valid  <= 1'b0;
        end else begin
            // Default: sym_valid is a 1-clock pulse, clear it each cycle
            sym_valid <= 1'b0;

            // Accept incoming bit if room
            if (bit_valid && bit_ready) begin
                acc[bps - 1 - acc_count] <= bit_in;
                acc_count                <= acc_count + 4'd1;
            end

            // Emit on symbol boundary if we have a full symbol
            if (sym_en) begin
                if (full) begin
                    sym_bits  <= acc;
                    bits_used <= bps;
                    sym_valid <= 1'b1;
                    // Clear for next symbol
                    acc       <= {MAX_BPS{1'b0}};
                    acc_count <= 4'd0;
                end
                // If not full at sym_en, hold accumulator; sym_valid stays 0
                // This is the starvation case at Qm > 4 without FEC
            end
        end
    end

endmodule
