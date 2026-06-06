`timescale 1ns / 1ps
//=============================================================================
// File        : rate_match.v
// Description : 5G NR LDPC rate matching, bit selection only (TS 38.212 5.4.2.1).
//               Collect the N=2600-bit BG2/Zc=52 codeword, then stream E bits
//               read from a circular buffer starting at k0(RV). The 5.4.2.2 bit
//               interleaver is a SEPARATE downstream block (bit_interleaver).
//=============================================================================
// - E is a runtime input (e_in); RV is a runtime input (rv_in) selecting k0.
//   k0 table for BG2, Ncb=N=2600: rv 0/1/2/3 -> 0/676/1300/2236.
// - For RV=0, E<=N, no filler: output = first E bits (puncturing). Circular
//   wraparound supports E>N (repetition). FILLER-BIT SKIP IS NOT IMPLEMENTED:
//   K=520 (=10*Zc) produces no NULL/filler bits. If code-block segmentation
//   ever yields filler bits, add filler-skip here.
// - 1-bit in (2600) / 1-bit out (E). Registered AXI-Stream outputs.
//=============================================================================
module rate_match #(
    parameter integer N      = 2600,
    parameter integer ADDR_W = 13
)(
    input  wire                aclk,
    input  wire                aresetn,
    input  wire [1:0]          rv_in,
    input  wire [ADDR_W-1:0]   e_in,

    input  wire                s_axis_tdata,
    input  wire                s_axis_tvalid,
    output reg                 s_axis_tready,
    input  wire                s_axis_tlast,

    output reg                 m_axis_tdata,
    output reg                 m_axis_tvalid,
    input  wire                m_axis_tready,
    output reg                 m_axis_tlast
);
    reg buf_mem [0:N-1];

    reg [ADDR_W-1:0] coll_cnt;     // 0..N-1
    reg [ADDR_W-1:0] e_total;      // latched E
    reg [ADDR_W-1:0] rd_ptr;       // circular read pointer, wraps at N
    reg [ADDR_W-1:0] out_cnt;      // 0..E-1

    // k0 from RV (BG2, Ncb=N)
    function [ADDR_W-1:0] k0_of;
        input [1:0] rv;
        begin
            case(rv)
              2'd0: k0_of = 13'd0;
              2'd1: k0_of = 13'd676;
              2'd2: k0_of = 13'd1300;
              default: k0_of = 13'd2236;
            endcase
        end
    endfunction

    localparam S_COLLECT=1'b0, S_STREAM=1'b1;
    reg state;
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    always @(*) s_axis_tready = (state==S_COLLECT);

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=S_COLLECT; coll_cnt<=0; e_total<=0; rd_ptr<=0; out_cnt<=0;
            m_axis_tdata<=1'b0; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else begin
            if(output_fire) begin m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0; end
            case(state)
            S_COLLECT: begin
                if(input_fire) begin
                    buf_mem[coll_cnt] <= s_axis_tdata;
                    if(coll_cnt==0) e_total <= e_in;            // latch E on first beat
                    if(coll_cnt==N-1) begin
                        rd_ptr   <= k0_of(rv_in);               // start of circular read
                        out_cnt  <= 0;
                        coll_cnt <= 0;
                        state    <= S_STREAM;
                    end else begin
                        coll_cnt <= coll_cnt + 1'b1;
                    end
                end
            end
            S_STREAM: begin
                if(!m_axis_tvalid || m_axis_tready) begin
                    m_axis_tdata  <= buf_mem[rd_ptr];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (out_cnt == e_total-1'b1);
                    // advance circular pointer
                    if(rd_ptr==N-1) rd_ptr<=0; else rd_ptr<=rd_ptr+1'b1;
                    if(out_cnt == e_total-1'b1) begin
                        state<=S_COLLECT; out_cnt<=0;
                    end else begin
                        out_cnt<=out_cnt+1'b1;
                    end
                end
            end
            endcase
        end
    end
endmodule
