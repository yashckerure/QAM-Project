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
    // Extract absolute value
    // -------------------------------------------------------------------------
    wire i_sign = i_in[DATA_W-1];
    wire q_sign = q_in[DATA_W-1];
    
    wire [DATA_W-1:0] i_abs = i_sign ? -i_in : i_in;
    wire [DATA_W-1:0] q_abs = q_sign ? -q_in : q_in;

    // -------------------------------------------------------------------------
    // Multiply by inverse scale factor
    // -------------------------------------------------------------------------
    reg [16:0] inv_scale;
    always @(*) begin
        case (qam_mode)
            3'd0: inv_scale = 17'd5793;  // QPSK:    round(sqrt(2) * 4096)
            3'd1: inv_scale = 17'd12953; // 16-QAM:  round(sqrt(10) * 4096)
            3'd2: inv_scale = 17'd26545; // 64-QAM:  round(sqrt(42) * 4096)
            3'd3: inv_scale = 17'd53406; // 256-QAM: round(sqrt(170) * 4096)
            default: inv_scale = 17'd5793;
        endcase
    end

    // Product is Q5.10 * QX.12 = Q(5+X).22
    wire [31:0] i_prod = i_abs * inv_scale;
    wire [31:0] q_prod = q_abs * inv_scale;

    // Add 1<<21 for rounding, then shift right by 22
    wire [4:0] i_mag_est = (i_prod + (1 << 21)) >> 22;
    wire [4:0] q_mag_est = (q_prod + (1 << 21)) >> 22;

    // -------------------------------------------------------------------------
    // Clip to max magnitude
    // -------------------------------------------------------------------------
    reg [4:0] max_mag;
    always @(*) begin
        case (bps_axis)
            3'd1: max_mag = 5'd1;
            3'd2: max_mag = 5'd3;
            3'd3: max_mag = 5'd7;
            3'd4: max_mag = 5'd15;
            default: max_mag = 5'd3;
        endcase
    end

    wire [4:0] i_clip = (i_mag_est > max_mag) ? max_mag : i_mag_est;
    wire [4:0] q_clip = (q_mag_est > max_mag) ? max_mag : q_mag_est;

    wire [3:0] i_val = i_clip >> 1;
    wire [3:0] q_val = q_clip >> 1;

    // -------------------------------------------------------------------------
    // Decode 3GPP magnitude to bits
    // -------------------------------------------------------------------------
    reg [3:0] i_bits, q_bits;
    always @(*) begin
        i_bits = 4'd0;
        q_bits = 4'd0;
        
        // Sign bits
        i_bits[bps_axis-1] = i_sign;
        q_bits[bps_axis-1] = q_sign;

        if (bps_axis > 1) begin
            i_bits[bps_axis-2] = i_val[bps_axis-2];
            q_bits[bps_axis-2] = q_val[bps_axis-2];
        end
        if (bps_axis > 2) begin
            i_bits[bps_axis-3] = i_val[bps_axis-3] ^ ~i_val[bps_axis-2];
            q_bits[bps_axis-3] = q_val[bps_axis-3] ^ ~q_val[bps_axis-2];
        end
        if (bps_axis > 3) begin
            i_bits[bps_axis-4] = i_val[bps_axis-4] ^ ~i_val[bps_axis-3];
            q_bits[bps_axis-4] = q_val[bps_axis-4] ^ ~q_val[bps_axis-3];
        end
    end

    // -------------------------------------------------------------------------
    // Interleave I and Q back to symbol
    // -------------------------------------------------------------------------
    reg [MAX_BPS-1:0] sym_pack;
    always @(*) begin
        sym_pack = {MAX_BPS{1'b0}};
        case (bps_axis)
            3'd1: begin
                sym_pack[1] = i_bits[0];
                sym_pack[0] = q_bits[0];
            end
            3'd2: begin
                sym_pack[3] = i_bits[1];
                sym_pack[2] = q_bits[1];
                sym_pack[1] = i_bits[0];
                sym_pack[0] = q_bits[0];
            end
            3'd3: begin
                sym_pack[5] = i_bits[2];
                sym_pack[4] = q_bits[2];
                sym_pack[3] = i_bits[1];
                sym_pack[2] = q_bits[1];
                sym_pack[1] = i_bits[0];
                sym_pack[0] = q_bits[0];
            end
            3'd4: begin
                sym_pack[7] = i_bits[3];
                sym_pack[6] = q_bits[3];
                sym_pack[5] = i_bits[2];
                sym_pack[4] = q_bits[2];
                sym_pack[3] = i_bits[1];
                sym_pack[2] = q_bits[1];
                sym_pack[1] = i_bits[0];
                sym_pack[0] = q_bits[0];
            end
            default: begin
                sym_pack[1] = i_bits[0];
                sym_pack[0] = q_bits[0];
            end
        endcase
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
