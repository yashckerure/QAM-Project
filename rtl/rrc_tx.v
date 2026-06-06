//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.06.2026 19:12:10
// Design Name: 
// Module Name: rrc_tx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//=============================================================================
// File        : rrc_tx.v  (seamless / gap-free version)
// Description : Polyphase interpolating RRC TX filter. beta=0.5, span=8, SPS=4
//               -> 33 taps (padded 36 = 4 phases x 9). 1 complex symbol (Q5.10)
//               in -> 4 complex samples (Q5.10) out, CONTINUOUS (no bubble):
//               a new symbol is consumed on the same beat phase 3 is emitted.
//=============================================================================
// Coeffs Q1.14. Phase p tap k = h[p+4k], multiplies x[n-k]. acc(40b)=sum coeff*
// data; >>>14; saturate Q5.10. Group delay = (33-1)/2 = 16 samples (4 symbols).
//=============================================================================
module rrc_tx #(parameter integer DW=16)(
    input  wire             aclk,
    input  wire             aresetn,
    input  wire [2*DW-1:0]  s_axis_tdata,
    input  wire             s_axis_tvalid,
    output wire             s_axis_tready,
    output reg  [2*DW-1:0]  m_axis_tdata,
    output reg              m_axis_tvalid,
    input  wire             m_axis_tready
);
    localparam signed [15:0] H0_0 = -16'sd83;
    localparam signed [15:0] H0_1 = 16'sd25;
    localparam signed [15:0] H0_2 = 16'sd348;
    localparam signed [15:0] H0_3 = -16'sd869;
    localparam signed [15:0] H0_4 = 16'sd9312;
    localparam signed [15:0] H0_5 = -16'sd869;
    localparam signed [15:0] H0_6 = 16'sd348;
    localparam signed [15:0] H0_7 = 16'sd25;
    localparam signed [15:0] H0_8 = -16'sd83;
    localparam signed [15:0] H1_0 = -16'sd31;
    localparam signed [15:0] H1_1 = -16'sd135;
    localparam signed [15:0] H1_2 = 16'sd127;
    localparam signed [15:0] H1_3 = 16'sd1285;
    localparam signed [15:0] H1_4 = 16'sd7984;
    localparam signed [15:0] H1_5 = -16'sd1285;
    localparam signed [15:0] H1_6 = 16'sd127;
    localparam signed [15:0] H1_7 = 16'sd135;
    localparam signed [15:0] H1_8 = 16'sd0;
    localparam signed [15:0] H2_0 = 16'sd88;
    localparam signed [15:0] H2_1 = -16'sd123;
    localparam signed [15:0] H2_2 = -16'sd615;
    localparam signed [15:0] H2_3 = 16'sd4740;
    localparam signed [15:0] H2_4 = 16'sd4740;
    localparam signed [15:0] H2_5 = -16'sd615;
    localparam signed [15:0] H2_6 = -16'sd123;
    localparam signed [15:0] H2_7 = 16'sd88;
    localparam signed [15:0] H2_8 = 16'sd0;
    localparam signed [15:0] H3_0 = 16'sd135;
    localparam signed [15:0] H3_1 = 16'sd127;
    localparam signed [15:0] H3_2 = -16'sd1285;
    localparam signed [15:0] H3_3 = 16'sd7984;
    localparam signed [15:0] H3_4 = 16'sd1285;
    localparam signed [15:0] H3_5 = 16'sd127;
    localparam signed [15:0] H3_6 = -16'sd135;
    localparam signed [15:0] H3_7 = -16'sd31;
    localparam signed [15:0] H3_8 = 16'sd0;

    wire signed [DW-1:0] in_i = s_axis_tdata[DW-1:0];
    wire signed [DW-1:0] in_q = s_axis_tdata[2*DW-1:DW];

    reg signed [DW-1:0] hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7;
    reg signed [DW-1:0] hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7;
    reg signed [DW-1:0] obi0,obi1,obi2,obi3, obq0,obq1,obq2,obq3;
    reg [1:0] phase;

    localparam S_EMPTY=1'b0, S_RUN=1'b1;
    reg state;
    // seamless: ready when empty, or when emitting phase 3 with downstream ready
    assign s_axis_tready = (state==S_EMPTY) || (state==S_RUN && phase==2'd3 && m_axis_tready);
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    function signed [DW-1:0] sat(input signed [39:0] a);
        reg signed [39:0] t; begin
            t=a>>>14;
            if(t>40'sd32767) sat=16'sd32767;
            else if(t<-40'sd32768) sat=-16'sd32768;
            else sat=t[DW-1:0];
        end
    endfunction
    function signed [DW-1:0] macp(input integer p,
        input signed [DW-1:0] x0,x1,x2,x3,x4,x5,x6,x7,x8);
        reg signed [39:0] acc; begin
            case(p)
              0: acc=H0_0*x0+H0_1*x1+H0_2*x2+H0_3*x3+H0_4*x4+H0_5*x5+H0_6*x6+H0_7*x7+H0_8*x8;
              1: acc=H1_0*x0+H1_1*x1+H1_2*x2+H1_3*x3+H1_4*x4+H1_5*x5+H1_6*x6+H1_7*x7+H1_8*x8;
              2: acc=H2_0*x0+H2_1*x1+H2_2*x2+H2_3*x3+H2_4*x4+H2_5*x5+H2_6*x6+H2_7*x7+H2_8*x8;
              default: acc=H3_0*x0+H3_1*x1+H3_2*x2+H3_3*x3+H3_4*x4+H3_5*x5+H3_6*x6+H3_7*x7+H3_8*x8;
            endcase
            macp=sat(acc);
        end
    endfunction

    // consume current input: compute 4 phase outs from {in,hist}, shift hist,
    // preload phase 0, set phase=0, valid=1
    task accept_symbol; begin
        obi0<=macp(0,in_i,hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7);
        obi1<=macp(1,in_i,hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7);
        obi2<=macp(2,in_i,hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7);
        obi3<=macp(3,in_i,hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7);
        obq0<=macp(0,in_q,hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7);
        obq1<=macp(1,in_q,hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7);
        obq2<=macp(2,in_q,hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7);
        obq3<=macp(3,in_q,hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7);
        hi7<=hi6;hi6<=hi5;hi5<=hi4;hi4<=hi3;hi3<=hi2;hi2<=hi1;hi1<=hi0;hi0<=in_i;
        hq7<=hq6;hq6<=hq5;hq5<=hq4;hq4<=hq3;hq3<=hq2;hq2<=hq1;hq1<=hq0;hq0<=in_q;
        phase<=2'd0; m_axis_tvalid<=1'b1;
        m_axis_tdata<={macp(0,in_q,hq0,hq1,hq2,hq3,hq4,hq5,hq6,hq7),
                       macp(0,in_i,hi0,hi1,hi2,hi3,hi4,hi5,hi6,hi7)};
    end endtask

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=S_EMPTY; phase<=2'd0; m_axis_tvalid<=1'b0; m_axis_tdata<=0;
            hi0<=0;hi1<=0;hi2<=0;hi3<=0;hi4<=0;hi5<=0;hi6<=0;hi7<=0;
            hq0<=0;hq1<=0;hq2<=0;hq3<=0;hq4<=0;hq5<=0;hq6<=0;hq7<=0;
        end else begin
            case(state)
            S_EMPTY: if(input_fire) begin accept_symbol; state<=S_RUN; end
            S_RUN: if(output_fire) begin
                if(phase==2'd3) begin
                    if(s_axis_tvalid) begin
                        accept_symbol;           // seamless reload, stay in RUN
                    end else begin
                        m_axis_tvalid<=1'b0; state<=S_EMPTY;
                    end
                end else begin
                    phase<=phase+2'd1;
                    case(phase)
                      2'd0: m_axis_tdata<={obq1,obi1};
                      2'd1: m_axis_tdata<={obq2,obi2};
                      default: m_axis_tdata<={obq3,obi3};
                    endcase
                end
            end
            endcase
        end
    end
endmodule