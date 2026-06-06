`timescale 1ns / 1ps
//=============================================================================
// File        : code_block_deseg.v
// Description : RX inverse of code block segmentation (TS 38.212 5.2.2 inverse).
//               Reassembles the decoded code block(s) into the transport block.
//               Sits AFTER the LDPC decoder (hard bits), before crc24a_check.
//=============================================================================
// BIT-DOMAIN: the decoder outputs hard bits, so this is a 1-bit block. For this
// project B = 520 <= Kcb = 3840 (BG2) => C = 1, L = 0, no filler => PASS-THROUGH
// of the 520 decoded bits. Standards-mandated behaviour for a single code block.
//
// SCOPE: for C>1 this block would remove the per-CB filler bits and the type-24B
// CRC and (optionally) check it, then concatenate the C blocks. Not exercised
// here: single-CB by design (fixed Zc=52/K=520, no filler, no CRC-24B).
//=============================================================================
module code_block_deseg #(
    parameter integer B   = 520,
    parameter integer KCB = 3840
)(
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
    // synthesis translate_off
    initial if (B > KCB) $display("WARNING: code_block_deseg: B=%0d > KCB=%0d, C>1 not supported", B, KCB);
    // synthesis translate_on
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
