`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : qam_slicer.v
// Description  : Hard-decision inverse QAM mapper (slicer).
//=============================================================================
// Additional Notes:
// - Inputs:  Q5.10 signed I, Q from the mapper (or, later, the matched filter)
// - Outputs: Qm-bit symbol, MSB-first, ready to feed back to BER counter.
//=============================================================================
//
// This is the exact arithmetic inverse of qam_mapper.v. In the absence of a
// channel or filter, slicer output should equal mapper input bit-for-bit.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module qam_slicer #(
    parameter integer DATA_W  = 16,
    parameter integer FRAC_W  = 10,
    parameter integer MAX_BPS = 8
)(
    input  wire                       aclk,
    input  wire                       aresetn,
    input  wire  [2:0]                qam_mode,
    input  wire  [2*DATA_W-1:0]       s_axis_tdata,
    input  wire                       s_axis_tvalid,
    output wire                       s_axis_tready,
    output reg   [MAX_BPS-1:0]        m_axis_tdata,
    output reg   [3:0]                m_axis_tuser,
    output reg                        m_axis_tvalid,
    input  wire                       m_axis_tready
);

    assign s_axis_tready = 1'b1;

    wire signed [DATA_W-1:0] i_in = s_axis_tdata[DATA_W-1:0];
    wire signed [DATA_W-1:0] q_in = s_axis_tdata[2*DATA_W-1:DATA_W];

    // -------------------------------------------------------------------------
    // Decode mode
    // -------------------------------------------------------------------------
    reg [2:0] bps_axis;
    reg [3:0] bps_total;
    reg signed [5:0] max_level;     // +(2^N - 1): 1, 3, 7, 15
    always @(*) begin
        case (qam_mode)
            3'd0:    begin bps_axis = 3'd1; bps_total = 4'd2; max_level = 6'sd1;  end
            3'd1:    begin bps_axis = 3'd2; bps_total = 4'd4; max_level = 6'sd3;  end
            3'd2:    begin bps_axis = 3'd3; bps_total = 4'd6; max_level = 6'sd7;  end
            3'd3:    begin bps_axis = 3'd4; bps_total = 4'd8; max_level = 6'sd15; end
            default: begin bps_axis = 3'd2; bps_total = 4'd4; max_level = 6'sd3;  end
        endcase
    end

    // -------------------------------------------------------------------------
    // Step 1: round to nearest odd integer level
    //
    // For sample s (Q5.10), the half-step index is:
    //   k = floor( (s + 2^FRAC_W) / 2^(FRAC_W+1) )       symmetric rounding
    // Then level = 2*k - 1.
    //
    // Effectively: add (1 << FRAC_W), arithmetic-shift right by (FRAC_W+1),
    //              then convert to odd level by (2*k - 1).
    //
    // We use the SAR of the signed input directly; the shift count is
    // FRAC_W+1 = 11 for FRAC_W=10.
    // -------------------------------------------------------------------------
    wire signed [DATA_W-1:0] i_biased = i_in + (1 <<< FRAC_W);
    wire signed [DATA_W-1:0] q_biased = q_in + (1 <<< FRAC_W);

    wire signed [DATA_W-1:0] i_kfull  = i_biased >>> (FRAC_W + 1);   // half-step index
    wire signed [DATA_W-1:0] q_kfull  = q_biased >>> (FRAC_W + 1);

    // Convert half-step index -> odd level:  level = 2*k - 1
    wire signed [DATA_W:0] i_level_raw = (i_kfull <<< 1) - 1;
    wire signed [DATA_W:0] q_level_raw = (q_kfull <<< 1) - 1;

    // -------------------------------------------------------------------------
    // Step 2: clip to legal range [-max_level, +max_level]
    // -------------------------------------------------------------------------
    reg signed [5:0] i_level, q_level;
    always @(*) begin
        // I
        if      (i_level_raw >  $signed({{(DATA_W+1-6){max_level[5]}}, max_level})) i_level =  max_level;
        else if (i_level_raw < -$signed({{(DATA_W+1-6){max_level[5]}}, max_level})) i_level = -max_level;
        else                                                                       i_level =  i_level_raw[5:0];

        // Q
        if      (q_level_raw >  $signed({{(DATA_W+1-6){max_level[5]}}, max_level})) q_level =  max_level;
        else if (q_level_raw < -$signed({{(DATA_W+1-6){max_level[5]}}, max_level})) q_level = -max_level;
        else                                                                       q_level =  q_level_raw[5:0];
    end

    // -------------------------------------------------------------------------
    // Step 3: level -> binary index
    //   binary = (level + max_level) / 2,  in range [0, 2^N - 1]
    // -------------------------------------------------------------------------
    wire signed [6:0] i_bin_signed = (i_level + max_level) >>> 1;
    wire signed [6:0] q_bin_signed = (q_level + max_level) >>> 1;
    wire [3:0] i_bin = i_bin_signed[3:0];
    wire [3:0] q_bin = q_bin_signed[3:0];

    // -------------------------------------------------------------------------
    // Step 4: binary -> Gray
    //   gray[N-1] = binary[N-1]
    //   gray[i]   = binary[i+1] XOR binary[i]
    // (Process all 4 bit positions; unused upper bits are zero.)
    // -------------------------------------------------------------------------
    reg [3:0] i_gray, q_gray;
    always @(*) begin
        i_gray[3] = i_bin[3];
        i_gray[2] = i_bin[3] ^ i_bin[2];
        i_gray[1] = i_bin[2] ^ i_bin[1];
        i_gray[0] = i_bin[1] ^ i_bin[0];

        q_gray[3] = q_bin[3];
        q_gray[2] = q_bin[3] ^ q_bin[2];
        q_gray[1] = q_bin[2] ^ q_bin[1];
        q_gray[0] = q_bin[1] ^ q_bin[0];
    end

    // -------------------------------------------------------------------------
    // Step 5: reassemble symbol with I in upper, Q in lower (MSB-first)
    //   For bps_axis = N, the Qm-bit symbol is:
    //     sym[2N-1 : N] = i_gray[N-1 : 0]
    //     sym[N-1   : 0] = q_gray[N-1 : 0]
    // -------------------------------------------------------------------------
    reg [MAX_BPS-1:0] sym_pack;
    integer k;
    always @(*) begin
        sym_pack = {MAX_BPS{1'b0}};
        for (k = 0; k < 4; k = k + 1) begin
            if (k < bps_axis) begin
                sym_pack[bps_axis + k] = i_gray[k];   // upper half
                sym_pack[k]            = q_gray[k];   // lower half
            end
        end
    end

    // -------------------------------------------------------------------------
    // Register outputs, one-cycle pipeline behind s_axis_tvalid
    // -------------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axis_tdata  <= {MAX_BPS{1'b0}};
            m_axis_tuser  <= 4'd0;
            m_axis_tvalid <= 1'b0;
        end else if (s_axis_tvalid) begin
            m_axis_tdata  <= sym_pack;
            m_axis_tuser  <= bps_total;
            m_axis_tvalid <= 1'b1;
        end else begin
            m_axis_tvalid <= 1'b0;
        end
    end

endmodule
