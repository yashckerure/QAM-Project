//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.05.2026 19:38:56
// Design Name: 
// Module Name: qam_mapper
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
// qam_mapper.v
// Square-QAM Gray-coded mapper for adaptive modem.
//   - Supports QPSK / 16 / 64 / 256 / 1024-QAM
//   - Arithmetic Gray-to-PAM-level conversion (no large ROM)
//   - Output format: signed 16-bit Q5.10
//   - Zero-stuffing upsampling at 4 SPS (zero output between symbols)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// qam_mapper.v
// Square-QAM mapper using arithmetic Gray-PAM.
// Modes: 0=QPSK, 1=16QAM, 2=64QAM, 3=256QAM (bps_axis = 1, 2, 3, 4).
//
// Bit layout of sym_bits[7:0] (MSB-first):
//   For Qm-bit symbol with bps_axis = Qm/2:
//     I-bits occupy the upper bps_axis positions of the valid field
//     Q-bits occupy the lower bps_axis positions of the valid field
//   Example, 16-QAM (bps_axis=2, bits_used=4):
//     sym_bits[3] = I-MSB (I-gray bit 1)
//     sym_bits[2] = I-LSB (I-gray bit 0)
//     sym_bits[1] = Q-MSB (Q-gray bit 1)
//     sym_bits[0] = Q-LSB (Q-gray bit 0)
//
// Output: i_out, q_out in Q5.10 signed 16-bit. Registered, with iq_valid
// pulse one clock after sym_valid.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module qam_mapper #(
    parameter integer DATA_W  = 16,
    parameter integer FRAC_W  = 10,
    parameter integer MAX_BPS = 8
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire  [2:0]                qam_mode,
    input  wire  [MAX_BPS-1:0]        sym_bits,
    input  wire  [3:0]                bits_used,
    input  wire                       sym_valid,
    output reg   signed [DATA_W-1:0]  i_out,
    output reg   signed [DATA_W-1:0]  q_out,
    output reg                        iq_valid
);

    // -------------------------------------------------------------------------
    // Decode mode -> bits per axis
    // -------------------------------------------------------------------------
    reg [2:0] bps_axis;
    always @(*) begin
        case (qam_mode)
            3'd0:    bps_axis = 3'd1;     // QPSK
            3'd1:    bps_axis = 3'd2;     // 16-QAM
            3'd2:    bps_axis = 3'd3;     // 64-QAM
            3'd3:    bps_axis = 3'd4;     // 256-QAM
            default: bps_axis = 3'd2;
        endcase
    end

    // -------------------------------------------------------------------------
    // Split sym_bits into i_gray and q_gray (both right-justified)
    //
    //   I-half occupies positions [bits_used-1 : bps_axis]
    //   Q-half occupies positions [bps_axis-1 : 0]
    //
    // For each axis we collect bps_axis bits MSB-first into the LSBs of
    // i_gray / q_gray (so i_gray[bps_axis-1] is the I-MSB).
    // -------------------------------------------------------------------------
    reg [3:0] i_gray, q_gray;
    integer k;
    always @(*) begin
        i_gray = 4'd0;
        q_gray = 4'd0;
        for (k = 0; k < 4; k = k + 1) begin
            if (k < bps_axis) begin
                i_gray[k] = sym_bits[bps_axis + k];   // upper half, indexed from low
                q_gray[k] = sym_bits[k];              // lower half
            end
        end
    end

    // -------------------------------------------------------------------------
    // Gray-to-binary per axis (MSB unchanged, ripple XOR downward).
    // Process all 4 bit positions unconditionally; masking is implicit because
    // unused upper i_gray/q_gray bits are zero.
    // -------------------------------------------------------------------------
    reg [3:0] i_bin, q_bin;
    always @(*) begin
        i_bin[3] = i_gray[3];
        i_bin[2] = i_bin[3] ^ i_gray[2];
        i_bin[1] = i_bin[2] ^ i_gray[1];
        i_bin[0] = i_bin[1] ^ i_gray[0];

        q_bin[3] = q_gray[3];
        q_bin[2] = q_bin[3] ^ q_gray[2];
        q_bin[1] = q_bin[2] ^ q_gray[1];
        q_bin[0] = q_bin[1] ^ q_gray[0];
    end

    // -------------------------------------------------------------------------
    // Extract the valid binary bits per mode, compute PAM level, scale to Q5.10
    //   level = 2*binary - (2^N - 1)
    //   scaled = level << FRAC_W
    // -------------------------------------------------------------------------
    reg signed [5:0] i_level, q_level;   // 6 bits is enough for +-15
    reg signed [5:0] offset;
    reg [3:0]        i_bin_eff, q_bin_eff;
    always @(*) begin
        case (bps_axis)
            3'd1: begin
                i_bin_eff = {3'd0, i_bin[0]};
                q_bin_eff = {3'd0, q_bin[0]};
                offset    = 6'sd1;
            end
            3'd2: begin
                i_bin_eff = {2'd0, i_bin[1:0]};
                q_bin_eff = {2'd0, q_bin[1:0]};
                offset    = 6'sd3;
            end
            3'd3: begin
                i_bin_eff = {1'd0, i_bin[2:0]};
                q_bin_eff = {1'd0, q_bin[2:0]};
                offset    = 6'sd7;
            end
            3'd4: begin
                i_bin_eff = i_bin;
                q_bin_eff = q_bin;
                offset    = 6'sd15;
            end
            default: begin
                i_bin_eff = 4'd0;
                q_bin_eff = 4'd0;
                offset    = 6'sd3;
            end
        endcase
        i_level = $signed({1'b0, i_bin_eff, 1'b0}) - offset;
        q_level = $signed({1'b0, q_bin_eff, 1'b0}) - offset;
    end

    wire signed [DATA_W-1:0] i_q510 = $signed(i_level) <<< FRAC_W;
    wire signed [DATA_W-1:0] q_q510 = $signed(q_level) <<< FRAC_W;

    // -------------------------------------------------------------------------
    // Register outputs, one-cycle pipeline behind sym_valid
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_out    <= {DATA_W{1'b0}};
            q_out    <= {DATA_W{1'b0}};
            iq_valid <= 1'b0;
        end else if (sym_valid) begin
            i_out    <= i_q510;
            q_out    <= q_q510;
            iq_valid <= 1'b1;
        end else begin
            iq_valid <= 1'b0;
        end
    end

endmodule
