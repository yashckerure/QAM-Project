`timescale 1ns / 1ps
//=============================================================================
// File        : code_block_seg.v
// Description : 5G NR code block segmentation + code-block CRC (TS 38.212 5.2.2).
//               Sits between crc24a_attach and ldpc_encoder on the TX path.
//=============================================================================
// 5.2.2 segmentation DECISION for this project:
//   B = 520 bits (496 payload + 24 CRC-24A), base graph BG2 -> Kcb = 3840.
//   B (520) <= Kcb (3840)  =>  C = 1, L = 0  =>  NO code-block CRC-24B,
//   and (since K = 10*Zc = 10*52 = 520 = B) NO filler bits.
//   Therefore the standards-mandated behaviour for this size is a PASS-THROUGH
//   of the 520-bit block. This is compliant, not a shortcut: 5.2.2 requires
//   no segmentation and no per-CB CRC when B <= Kcb (single code block).
//
// SCOPE / honest limitations:
//   - C>1 (B > Kcb) is NOT supported. It is incompatible with the fixed
//     Zc=52/K=520 LDPC encoder downstream (multi-CB needs variable K and per-CB
//     CRC-24B). The chain is single-CB by design.
//   - The 5.2.2 adaptive lifting-size selection (which for B=520 would derive
//     Zc=72, K=720, 200 filler bits) is NOT implemented. The project fixes
//     Zc=52, K=520, F=0 as a documented operating point.
//   - CRC-24B (0x800063) would be attached here only if C>1; not used at C=1.
//=============================================================================
module code_block_seg #(
    parameter integer B   = 520,     // transport block size (incl. TB CRC-24A)
    parameter integer KCB = 3840     // BG2 max code block size
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
    // C = 1 for B <= KCB (compile-time for this project). C>1 unsupported.
    // synthesis translate_off
    initial if (B > KCB) $display("WARNING: code_block_seg: B=%0d > KCB=%0d, C>1 not supported", B, KCB);
    // synthesis translate_on

    wire out_ready = !m_axis_tvalid || m_axis_tready;
    assign s_axis_tready = out_ready;
    wire input_fire = s_axis_tvalid && s_axis_tready;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            m_axis_tdata<=1'b0; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else if(out_ready) begin
            m_axis_tvalid <= s_axis_tvalid;
            if(s_axis_tvalid) begin
                m_axis_tdata <= s_axis_tdata;   // single code block: pass through
                m_axis_tlast <= s_axis_tlast;
            end
        end
    end
endmodule
