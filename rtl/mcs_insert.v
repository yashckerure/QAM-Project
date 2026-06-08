`timescale 1ns / 1ps
//=============================================================================
// File        : mcs_insert.v
// Description : Prepends a fixed-QPSK MCS header to each packet's symbol stream.
//               Sits on TX after qam_mapper. Lets the RX learn the payload's MCS
//               before it must demodulate (the role PDCCH plays in real 5G).
//=============================================================================
// Complex symbol {Q[15:0], I[15:0]} signed Q5.10. Header = N_HDR=9 QPSK symbols:
//   3-bit MCS, each bit carried on 3 symbols' I-axis (majority-of-3 on RX),
//   Q-axis held at +1 (724) as a pilot. Robust + mode-independent (always QPSK).
//   bit b -> I = b ? -724 : +724. Packets delimited by tlast.
//=============================================================================
module mcs_insert #(
    parameter integer N_HDR = 9,
    parameter signed [15:0] QPSK = 16'sd724
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [2:0]  mcs,                 // from controller (this packet's MCS)
    input  wire [31:0] s_axis_tdata,        // payload symbols
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);
    localparam S_HDR=1'b0, S_PAY=1'b1;
    reg state;
    reg [3:0] hdr_cnt;
    reg [2:0] mcs_lat;

    wire m_fire = m_axis_tvalid && m_axis_tready;

    // header symbol for index i using latched mcs
    function [31:0] hdr_sym;
        input [3:0] i; input [2:0] m;
        reg b;
        begin
            b = m[2 - (i/3)];                 // i 0-2->m[2], 3-5->m[1], 6-8->m[0]
            hdr_sym = { QPSK, (b ? -QPSK : QPSK) };   // Q=pilot, I=bit
        end
    endfunction

    // effective mcs for the header: latch at first header symbol, hold through
    wire [2:0] eff_mcs = (hdr_cnt==0) ? mcs : mcs_lat;

    // accept payload only in PAY state and when output can take it
    assign s_axis_tready = (state==S_PAY) && (!m_axis_tvalid || m_axis_tready);

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=S_HDR; hdr_cnt<=0; mcs_lat<=mcs;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(m_fire) begin m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0; end
            case(state)
            S_HDR: begin
                // start header only when a payload is waiting; latch its MCS
                if(s_axis_tvalid && (!m_axis_tvalid || m_axis_tready)) begin
                    if(hdr_cnt==0) mcs_lat <= mcs;
                    m_axis_tdata  <= hdr_sym(hdr_cnt, eff_mcs);
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= 1'b0;
                    if(hdr_cnt==N_HDR-1) begin hdr_cnt<=0; state<=S_PAY; end
                    else hdr_cnt<=hdr_cnt+1'b1;
                end
            end
            S_PAY: begin
                if(s_axis_tvalid && s_axis_tready) begin
                    m_axis_tdata  <= s_axis_tdata;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= s_axis_tlast;
                    if(s_axis_tlast) begin
                        state<=S_HDR; mcs_lat<=mcs;   // latch MCS for next packet
                    end
                end
            end
            endcase
        end
    end
endmodule
