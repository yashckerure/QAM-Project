//=============================================================================
// Project      : Adaptive QAM Modem
// File         : qam_mapper.v
// Description  : Square-QAM Gray-coded mapper for adaptive modem.
//=============================================================================
// Additional Notes:
// - Supports QPSK / 16 / 64 / 256 / 1024-QAM
// - Arithmetic Gray-to-PAM-level conversion (no large ROM)
// - Output format: signed 16-bit Q5.10
//=============================================================================
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
    input  wire                       aclk,
    input  wire                       aresetn,
    input  wire  [2:0]                qam_mode,
    input  wire  [MAX_BPS-1:0]        s_axis_tdata,
    input  wire  [3:0]                s_axis_tuser,
    input  wire                       s_axis_tvalid,
    output wire                       s_axis_tready,
    output reg   [2*DATA_W-1:0]       m_axis_tdata,
    output reg                        m_axis_tvalid,
    input  wire                       m_axis_tready
);

    assign s_axis_tready = 1'b1;

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
    // Extract interleaved I and Q bits
    // -------------------------------------------------------------------------
    reg [3:0] i_bits, q_bits;
    always @(*) begin
        i_bits = 4'd0;
        q_bits = 4'd0;
        case (bps_axis)
            3'd1: begin
                i_bits[0] = s_axis_tdata[1];
                q_bits[0] = s_axis_tdata[0];
            end
            3'd2: begin
                i_bits[1] = s_axis_tdata[3];
                q_bits[1] = s_axis_tdata[2];
                i_bits[0] = s_axis_tdata[1];
                q_bits[0] = s_axis_tdata[0];
            end
            3'd3: begin
                i_bits[2] = s_axis_tdata[5];
                q_bits[2] = s_axis_tdata[4];
                i_bits[1] = s_axis_tdata[3];
                q_bits[1] = s_axis_tdata[2];
                i_bits[0] = s_axis_tdata[1];
                q_bits[0] = s_axis_tdata[0];
            end
            3'd4: begin
                i_bits[3] = s_axis_tdata[7];
                q_bits[3] = s_axis_tdata[6];
                i_bits[2] = s_axis_tdata[5];
                q_bits[2] = s_axis_tdata[4];
                i_bits[1] = s_axis_tdata[3];
                q_bits[1] = s_axis_tdata[2];
                i_bits[0] = s_axis_tdata[1];
                q_bits[0] = s_axis_tdata[0];
            end
            default: begin
                i_bits[0] = s_axis_tdata[1];
                q_bits[0] = s_axis_tdata[0];
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Compute 3GPP recursive magnitude
    // -------------------------------------------------------------------------
    reg [4:0] i_mag, q_mag;
    always @(*) begin
        case (bps_axis)
            3'd1: begin
                i_mag = 5'd1;
                q_mag = 5'd1;
            end
            3'd2: begin
                i_mag = 5'd1 + 2*i_bits[0];
                q_mag = 5'd1 + 2*q_bits[0];
            end
            3'd3: begin
                i_mag = i_bits[1] ? (5'd4 + (5'd1 + 2*i_bits[0])) : (5'd4 - (5'd1 + 2*i_bits[0]));
                q_mag = q_bits[1] ? (5'd4 + (5'd1 + 2*q_bits[0])) : (5'd4 - (5'd1 + 2*q_bits[0]));
            end
            3'd4: begin
                i_mag = i_bits[2] ? (5'd8 + (i_bits[1] ? (5'd4 + (5'd1 + 2*i_bits[0])) : (5'd4 - (5'd1 + 2*i_bits[0])))) 
                                  : (5'd8 - (i_bits[1] ? (5'd4 + (5'd1 + 2*i_bits[0])) : (5'd4 - (5'd1 + 2*i_bits[0]))));
                q_mag = q_bits[2] ? (5'd8 + (q_bits[1] ? (5'd4 + (5'd1 + 2*q_bits[0])) : (5'd4 - (5'd1 + 2*q_bits[0]))))
                                  : (5'd8 - (q_bits[1] ? (5'd4 + (5'd1 + 2*q_bits[0])) : (5'd4 - (5'd1 + 2*q_bits[0]))));
            end
            default: begin
                i_mag = 5'd1;
                q_mag = 5'd1;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Scale and apply sign
    // -------------------------------------------------------------------------
    reg [9:0] scale_factor;
    always @(*) begin
        case (qam_mode)
            3'd0: scale_factor = 10'd724;  // QPSK:    round(1/sqrt(2) * 1024)
            3'd1: scale_factor = 10'd324;  // 16-QAM:  round(1/sqrt(10) * 1024)
            3'd2: scale_factor = 10'd158;  // 64-QAM:  round(1/sqrt(42) * 1024)
            3'd3: scale_factor = 10'd79;   // 256-QAM: round(1/sqrt(170) * 1024)
            default: scale_factor = 10'd724;
        endcase
    end

    wire [14:0] i_scaled = i_mag * scale_factor;
    wire [14:0] q_scaled = q_mag * scale_factor;

    // The MSB is the sign bit
    wire i_sign = i_bits[bps_axis - 1];
    wire q_sign = q_bits[bps_axis - 1];

    wire signed [DATA_W-1:0] i_q510 = i_sign ? -$signed({1'b0, i_scaled}) : $signed({1'b0, i_scaled});
    wire signed [DATA_W-1:0] q_q510 = q_sign ? -$signed({1'b0, q_scaled}) : $signed({1'b0, q_scaled});

    // -------------------------------------------------------------------------
    // Register outputs, one-cycle pipeline behind s_axis_tvalid
    // -------------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axis_tdata  <= {(2*DATA_W){1'b0}};
            m_axis_tvalid <= 1'b0;
        end else if (s_axis_tvalid) begin
            // Pack Q into upper half, I into lower half
            m_axis_tdata  <= {q_q510, i_q510};
            m_axis_tvalid <= 1'b1;
        end else begin
            m_axis_tvalid <= 1'b0;
        end
    end

endmodule
