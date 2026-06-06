//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 19:51:08
// Design Name: 
// Module Name: tb_bit_deinterleaver
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
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : tb_bit_deinterleaver.v
// Description  : Standalone TB for bit_deinterleaver.
//                Drives the interleaved version of 128 PRBS-23 bits
//                (16-QAM, Qm=4, E=128, N=32) into the DUT.
//                Expects output == original PRBS-23 stream.
//=============================================================================
`timescale 1ns / 1ps
//=============================================================================
// Standalone TB for LLR-width bit_deinterleaver.
// Round-trip: take a known LLR stream, apply the interleaver permutation, feed
// into the deinterleaver, expect the original LLR stream back. 16-QAM Qm=4,
// E=1040, N=260. LLR values span the full 4-bit signed range to catch width bugs.
//=============================================================================
module tb_bit_deinterleaver;
    localparam integer LLR_W = 4;
    localparam integer E      = 1040;
    localparam [3:0]   QM     = 4'd4;
    localparam integer N      = 260;

    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg  [3:0]  qm_in; reg [12:0] n_in;
    reg  signed [LLR_W-1:0] s_td; reg s_tv; wire s_tr; reg s_tl;
    wire signed [LLR_W-1:0] m_td; wire m_tv; reg m_tr; wire m_tl;

    bit_deinterleaver #(.LLR_W(LLR_W),.E_MAX(4096),.ADDR_W(13)) dut(
        .aclk(aclk),.aresetn(aresetn),.qm_in(qm_in),.n_in(n_in),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl));

    reg signed [LLR_W-1:0] data        [0:E-1];   // original LLRs
    reg signed [LLR_W-1:0] interleaved [0:E-1];   // after interleaver permutation
    integer k,r,c,idx,i,out_count,errors;

    initial begin
        // known LLR pattern spanning -8..7
        for(k=0;k<E;k=k+1) data[k] = (k % 16) - 8;
        // interleaver: interleaved[c*Qm+r] = data[r*N+c]
        idx=0;
        for(c=0;c<N;c=c+1) for(r=0;r<QM;r=r+1) begin
            interleaved[idx]=data[r*N+c]; idx=idx+1;
        end
    end

    // checker: deinterleaver output must equal original data
    initial begin
        out_count=0; errors=0;
    end
    always @(negedge aclk) begin
        if(aresetn && m_tv && m_tr) begin
            if(m_td !== data[out_count]) begin errors=errors+1;
                if(errors<=12) $display("MISMATCH idx %0d got %0d exp %0d",out_count,m_td,data[out_count]); end
            if(out_count==E-1 && !m_tl) begin errors=errors+1; $display("tlast missing"); end
            out_count=out_count+1;
            if(out_count==E) begin
                $display("DONE: %0d LLRs, ERRORS=%0d",out_count,errors);
                $display(errors==0?"PASS: LLR deinterleaver round-trip exact":"FAIL");
                $finish;
            end
        end
    end

    initial begin
        aresetn=0; qm_in=0; n_in=0; s_td=0; s_tv=0; s_tl=0; m_tr=0;
        #50; @(posedge aclk); aresetn=1; @(posedge aclk);
        qm_in=QM; n_in=N; m_tr=1;
        for(i=0;i<E;i=i+1) begin
            @(negedge aclk); s_td=interleaved[i]; s_tv=1; s_tl=(i==E-1);
            @(posedge aclk); while(!s_tr) @(posedge aclk);
        end
        @(negedge aclk); s_tv=0; s_tl=0; wait(0);
    end
    initial begin #2000000; $display("TIMEOUT out_count=%0d",out_count); $finish; end
endmodule