`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : llr_descrambler.v
// Description  : Soft (LLR-domain) PDSCH descrambler. Inverse of the bit
//                scrambler (TS 38.211 Sec 7.3.1.1) applied to LLRs on the RX
//                soft path (soft_demapper -> ... -> LDPC decoder).
//=============================================================================
// Why this exists:
//   The TX scrambler XORs coded bits with the length-31 Gold sequence c(n).
//   On RX we have LLRs, not bits, and decode softly, so descrambling must act
//   on LLRs: a bit XORed with c=1 has its hard value flipped, which corresponds
//   to FLIPPING THE SIGN of its LLR. Thus:
//        descrambled_llr(n) = c(n) ? -llr(n) : llr(n)
//   (LLR sign convention: positive => bit 0, negative => bit 1.)
//
// Gold sequence: IDENTICAL generation to scrambler/descrambler so it inverts
//   them exactly. Same C_INIT, same Nc=1600 warmup, same taps/shift.
//
// Negation saturates: -(MIN) would overflow signed LLR_W, so -(-2^(W-1)) is
//   clamped to +(2^(W-1)-1). For LLR_W=4: -(-8) -> +7.
//
// Registered AXI-Stream outputs. Single packet per reset session.
//=============================================================================
module llr_descrambler #(
    parameter [30:0]   C_INIT = 31'h00008000,   // must match TX scrambler
    parameter integer  LLR_W  = 4
)(
    input  wire                    aclk,
    input  wire                    aresetn,

    input  wire signed [LLR_W-1:0] s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output wire                    s_axis_tready,
    input  wire                    s_axis_tlast,

    output reg  signed [LLR_W-1:0] m_axis_tdata,
    output reg                     m_axis_tvalid,
    input  wire                    m_axis_tready,
    output reg                     m_axis_tlast
);
    localparam integer NC = 1600;
    localparam signed [LLR_W-1:0] LLR_MIN = {1'b1, {(LLR_W-1){1'b0}}};  // -2^(W-1)
    localparam signed [LLR_W-1:0] LLR_MAX = {1'b0, {(LLR_W-1){1'b1}}};  // +2^(W-1)-1

    reg [30:0] x1;
    reg [30:0] x2;
    wire x1_fb  = x1[3] ^ x1[0];
    wire x2_fb  = x2[3] ^ x2[2] ^ x2[1] ^ x2[0];
    wire gold_b = x1[0] ^ x2[0];

    // saturating negate
    function signed [LLR_W-1:0] sat_neg(input signed [LLR_W-1:0] v);
        begin
            if (v == LLR_MIN) sat_neg = LLR_MAX;
            else              sat_neg = -v;
        end
    endfunction

    localparam S_WARMUP  = 1'b0;
    localparam S_RUNNING = 1'b1;
    reg        state;
    reg [10:0] warmup_count;

    assign s_axis_tready = (state == S_RUNNING) &&
                           (!m_axis_tvalid || m_axis_tready);
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            x1            <= 31'h00000001;
            x2            <= C_INIT;
            warmup_count  <= 11'd0;
            state         <= S_WARMUP;
            m_axis_tdata  <= {LLR_W{1'b0}};
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end
            case (state)
                S_WARMUP: begin
                    x1 <= {x1_fb, x1[30:1]};
                    x2 <= {x2_fb, x2[30:1]};
                    if (warmup_count == NC - 1) state <= S_RUNNING;
                    else warmup_count <= warmup_count + 11'd1;
                end
                S_RUNNING: begin
                    if (input_fire) begin
                        m_axis_tdata  <= gold_b ? sat_neg(s_axis_tdata) : s_axis_tdata;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= s_axis_tlast;
                        x1 <= {x1_fb, x1[30:1]};
                        x2 <= {x2_fb, x2[30:1]};
                    end
                end
                default: state <= S_WARMUP;
            endcase
        end
    end
endmodule
