`timescale 1ns / 1ps
//=============================================================================
// File        : code_block_de_concat.v
// Description : RX inverse of code block concatenation (TS 38.212 5.5 inverse).
//               Splits the received soft (LLR) stream back into the C per-code-
//               block LLR streams. Sits early on RX (after descrambling), before
//               the LLR deinterleaver / rate dematch.
//=============================================================================
// LLR-DOMAIN: carries 4-bit signed LLRs (the RX works in LLRs from the soft
// demapper down to the decoder). For this project C = 1 (single code block),
// so de-concatenation is a PASS-THROUGH of the single LLR stream. This is the
// standards-mandated behaviour for one code block.
//
// SCOPE: C>1 (de-interleaving the stream into multiple per-CB streams) is not
// exercised; the chain is single-CB by design (fixed Zc=52/K=520).
//=============================================================================
module code_block_de_concat #(
    parameter integer LLR_W = 4
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
    wire out_ready = !m_axis_tvalid || m_axis_tready;
    assign s_axis_tready = out_ready;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            m_axis_tdata<={LLR_W{1'b0}}; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else if(out_ready) begin
            m_axis_tvalid <= s_axis_tvalid;
            if(s_axis_tvalid) begin
                m_axis_tdata <= s_axis_tdata;   // C=1: pass through
                m_axis_tlast <= s_axis_tlast;
            end
        end
    end
endmodule
