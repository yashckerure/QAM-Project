`timescale 1ns / 1ps
//=============================================================================
// File        : code_block_concat.v
// Description : 5G NR code block concatenation (TS 38.212 5.5). Sits after the
//               per-CB processing (rate matching) on the TX path, before the
//               scrambler/symbol mapping.
//=============================================================================
// 5.5 concatenates the C rate-matched code blocks f^(0),...,f^(C-1) end to end
// into one output bit stream g. For this project C = 1 (see code_block_seg),
// so concatenation is a PASS-THROUGH of the single rate-matched block (E bits).
// This is the standards-mandated behaviour for a single code block.
//
// SCOPE: C>1 (multiple blocks back-to-back) is not exercised because the chain
// is single-CB by design (fixed Zc=52/K=520). For C>1 this block would stream
// the C blocks in order; structurally still a concatenation of bit streams.
//=============================================================================
module code_block_concat (
    input  wire aclk,
    input  wire aresetn,
    input  wire s_axis_tdata,
    input  wire s_axis_tvalid,
    output wire s_axis_tready,
    input  wire s_axis_tlast,
    output reg  m_axis_tdata,
    output reg  m_axis_tvalid,
    input  wire m_axis_tready,
    output reg  m_axis_tlast
);
    wire out_ready = !m_axis_tvalid || m_axis_tready;
    assign s_axis_tready = out_ready;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            m_axis_tdata<=1'b0; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else if(out_ready) begin
            m_axis_tvalid <= s_axis_tvalid;
            if(s_axis_tvalid) begin
                m_axis_tdata <= s_axis_tdata;   // C=1: pass through
                m_axis_tlast <= s_axis_tlast;
            end
        end
    end
endmodule
