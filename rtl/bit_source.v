//=============================================================================
// Project      : Adaptive QAM Modem
// File         : bit_source.v
// Description  : 1-bit-per-clock PRBS bit source. Mode-agnostic.
//=============================================================================
// Additional Notes:
// - Default polynomial PRBS-23: x^23 + x^18 + 1 (taps on bits 22 and 17).
// - Selectable PRBS-15: x^15 + x^14 + 1 (taps on bits 14 and 13).
//=============================================================================
//
// Output is COMBINATIONAL from the LFSR MSB. The LFSR advances only when
// the consumer asserts m_axis_tready at the clock edge. This is the standard
// AXI4-Stream "ready/valid" producer pattern and avoids the lost-bit race
// that the previous registered-output version had.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module bit_source #(
    parameter integer LFSR_W = 23,
    parameter [LFSR_W-1:0] SEED = 23'h5A3C7E
)(
    input  wire                aclk,
    input  wire                aresetn,
    input  wire                m_axis_tready,
    output wire                m_axis_tdata,
    output wire                m_axis_tvalid
);

    reg  [LFSR_W-1:0] lfsr;

    wire fb_23 = lfsr[22] ^ lfsr[17];
    wire fb_15 = lfsr[14] ^ lfsr[13];
    wire fb    = (LFSR_W == 23) ? fb_23 :
                 (LFSR_W == 15) ? fb_15 : 1'b0;

    // Combinational outputs
    assign m_axis_tdata  = lfsr[LFSR_W-1];
    assign m_axis_tvalid = aresetn;

    // LFSR advances ONLY on consumption (m_axis_tready high at the edge)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            lfsr <= SEED;
        end else if (m_axis_tready) begin
            lfsr <= {lfsr[LFSR_W-2:0], fb};
        end
    end

endmodule