`timescale 1ns / 1ps
//=============================================================================
// File        : ber_counter.v   (payload-level redesign)
// Description : Post-decode BER + BLER counter for the coded adaptive chain.
//               Taps the RECOVERED PAYLOAD after crc24a_check (1-bit stream),
//               NOT the slicer. Regenerates the TX PRBS-23 and compares.
//=============================================================================
// Why this replaces the original: the old counter compared the slicer symbol
// bits to PRBS, which only held for the uncoded loopback. With LDPC + rate
// matching + interleaving + scrambling in the chain, those bits are coded/
// scrambled, not the payload. The original PRBS payload reappears only at the
// crc24a_check output, so BER is measured there.
//
// Connect to crc24a_check:
//   s_axis_tdata/tvalid/tlast <- m_axis_* (recovered 496-bit payload)
//   crc_ok / crc_valid        <- crc_ok / crc_valid
//
// Metrics:
//   bit_errors / bits_compared : post-decode BER (PRBS regen, 1 advance / bit)
//   packet_errors / packets    : BLER (counted on crc_valid; error if !crc_ok)
// PRBS-23 matches bit_source exactly: out=lfsr[22], fb=lfsr[22]^lfsr[17],
//   continuous across packets (no per-packet reset). Counters saturate at 2^32-1.
//=============================================================================
module ber_counter #(
    parameter integer       LFSR_W = 23,
    parameter [LFSR_W-1:0]  SEED   = 23'h5A3C7E
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        enable,
    input  wire [31:0] num_bits_target,

    // recovered payload stream (from crc24a_check)
    input  wire        s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    // block status (from crc24a_check)
    input  wire        crc_ok,
    input  wire        crc_valid,

    output reg  [31:0] bit_errors,
    output reg  [31:0] bits_compared,
    output reg  [31:0] packet_errors,
    output reg  [31:0] packets,
    output reg         status_done
);
    assign s_axis_tready = 1'b1;       // passive monitor, always ready

    reg [LFSR_W-1:0] lfsr;
    wire prbs_bit = lfsr[LFSR_W-1];
    wire fb = (LFSR_W==23) ? (lfsr[22]^lfsr[17]) :
              (LFSR_W==15) ? (lfsr[14]^lfsr[13]) : 1'b0;

    wire bit_fire = enable && s_axis_tvalid && !status_done;
    wire mismatch = bit_fire && (prbs_bit != s_axis_tdata);

    function [31:0] sat_inc;             // saturating +inc
        input [31:0] v; input [31:0] inc;
        begin sat_inc = (v + inc < v) ? 32'hFFFFFFFF : v + inc; end
    endfunction

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            lfsr<=SEED; bit_errors<=0; bits_compared<=0;
            packet_errors<=0; packets<=0; status_done<=0;
        end else begin
            // BER path
            if(bit_fire) begin
                lfsr          <= {lfsr[LFSR_W-2:0], fb};
                if(prbs_bit != s_axis_tdata) bit_errors <= sat_inc(bit_errors,1);
                bits_compared <= sat_inc(bits_compared,1);
                if(sat_inc(bits_compared,1) >= num_bits_target) status_done <= 1'b1;
            end
            // BLER path (independent of status_done so blocks keep counting)
            if(enable && crc_valid) begin
                packets <= sat_inc(packets,1);
                if(!crc_ok) packet_errors <= sat_inc(packet_errors,1);
            end
        end
    end
endmodule
