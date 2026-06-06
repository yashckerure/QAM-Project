`timescale 1ns / 1ps
//=============================================================================
// File        : rate_dematch.v
// Description : LLR-valued inverse of rate_match (TS 38.212 5.4.2.1 selection).
//               Takes E received 4-bit signed LLRs, scatters each to the
//               codeword position it was selected from (circular buffer, start
//               k0(RV)), zero-fills punctured positions, and streams the full
//               N=2600-LLR vector to the LDPC decoder.
//=============================================================================
// - E (e_in) and RV (rv_in) are runtime inputs, same convention as rate_match.
//   k0 table (BG2, Ncb=N=2600): rv 0/1/2/3 -> 0/676/1300/2236.
// - Punctured / unsent positions are filled with LLR=0 ("no information").
// - Repetition (E>N): multiple LLRs map to one position -> soft-combine (add,
//   saturate to 4-bit signed). For E<=N (puncturing) each position is written
//   at most once. FILLER-BIT handling not implemented (K=520 -> no filler).
// - Slave = 4-bit signed LLR (E beats). Master = 4-bit signed LLR (2600 beats).
// - Three phases: CLEAR buffer (N) -> COLLECT+accumulate (E) -> STREAM (N).
//=============================================================================
module rate_dematch #(
    parameter integer N      = 2600,
    parameter integer ADDR_W = 13,
    parameter integer LLR_W  = 4
)(
    input  wire                    aclk,
    input  wire                    aresetn,
    input  wire [1:0]              rv_in,
    input  wire [ADDR_W-1:0]       e_in,

    input  wire signed [LLR_W-1:0] s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output reg                     s_axis_tready,
    input  wire                    s_axis_tlast,

    output reg  signed [LLR_W-1:0] m_axis_tdata,
    output reg                     m_axis_tvalid,
    input  wire                    m_axis_tready,
    output reg                     m_axis_tlast
);
    reg signed [LLR_W-1:0] buf_mem [0:N-1];

    reg [ADDR_W-1:0] clr_cnt;
    reg [ADDR_W-1:0] coll_cnt;     // 0..E-1
    reg [ADDR_W-1:0] e_total;
    reg [ADDR_W-1:0] wr_ptr;       // circular write pointer
    reg [ADDR_W-1:0] rd_cnt;       // 0..N-1 output

    function [ADDR_W-1:0] k0_of(input [1:0] rv);
        begin case(rv)
            2'd0: k0_of=13'd0; 2'd1: k0_of=13'd676;
            2'd2: k0_of=13'd1300; default: k0_of=13'd2236;
        endcase end
    endfunction

    // saturating 4-bit signed add (soft combining)
    function signed [LLR_W-1:0] sat_add(input signed [LLR_W-1:0] a,
                                        input signed [LLR_W-1:0] b);
        reg signed [LLR_W+1:0] s; begin
            s = a + b;
            if (s >  $signed({1'b0,{(LLR_W-1){1'b1}}}))  sat_add =  {1'b0,{(LLR_W-1){1'b1}}}; // +7
            else if (s < $signed({1'b1,{(LLR_W-1){1'b0}}})) sat_add = {1'b1,{(LLR_W-1){1'b0}}}; // -8
            else sat_add = s[LLR_W-1:0];
        end
    endfunction

    localparam [1:0] S_CLEAR=2'd0, S_COLLECT=2'd1, S_STREAM=2'd2;
    reg [1:0] state;
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    always @(*) s_axis_tready = (state==S_COLLECT);

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=S_CLEAR; clr_cnt<=0; coll_cnt<=0; e_total<=0; wr_ptr<=0; rd_cnt<=0;
            m_axis_tdata<=0; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else begin
            if(output_fire) begin m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0; end
            case(state)
            S_CLEAR: begin
                buf_mem[clr_cnt] <= {LLR_W{1'b0}};
                if(clr_cnt==N-1) begin
                    clr_cnt <= 0; coll_cnt <= 0;
                    wr_ptr  <= k0_of(rv_in);   // latch start pointer BEFORE first write
                    e_total <= e_in;           // latch E
                    state   <= S_COLLECT;
                end else clr_cnt<=clr_cnt+1'b1;
            end
            S_COLLECT: begin
                if(input_fire) begin
                    buf_mem[wr_ptr] <= sat_add(buf_mem[wr_ptr], s_axis_tdata);
                    if(wr_ptr==N-1) wr_ptr<=0; else wr_ptr<=wr_ptr+1'b1;
                    if(coll_cnt == e_total-1'b1) begin
                        coll_cnt<=0; rd_cnt<=0; state<=S_STREAM;
                    end else coll_cnt<=coll_cnt+1'b1;
                end
            end
            S_STREAM: begin
                if(!m_axis_tvalid || m_axis_tready) begin
                    m_axis_tdata  <= buf_mem[rd_cnt];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (rd_cnt==N-1);
                    if(rd_cnt==N-1) begin
                        rd_cnt<=0; clr_cnt<=0; state<=S_CLEAR;
                    end else rd_cnt<=rd_cnt+1'b1;
                end
            end
            endcase
        end
    end
endmodule
