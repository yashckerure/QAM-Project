`timescale 1ns/1ps
module tb_rate_dematch;
    localparam integer N=2600, ADDR_W=13, LLR_W=4, NCFG=5;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [1:0] rv_in; reg [ADDR_W-1:0] e_in;
    reg signed [LLR_W-1:0] s_td; reg s_tv,s_tl; wire s_tr;
    wire signed [LLR_W-1:0] m_td; wire m_tv,m_tl; reg m_tr;
    rate_dematch #(.N(N),.ADDR_W(ADDR_W),.LLR_W(LLR_W)) dut(.aclk(aclk),.aresetn(aresetn),
        .rv_in(rv_in),.e_in(e_in),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl));

    integer rvs [0:NCFG-1]; integer es [0:NCFG-1];
    integer signed expbuf [0:N-1];
    integer ci, i, j, k0, addr, c, errors, total, s, llrv;

    function integer k0_of(input integer rv);
        begin case(rv) 0:k0_of=0; 1:k0_of=676; 2:k0_of=1300; default:k0_of=2236; endcase end
    endfunction
    function integer satadd(input integer a, input integer b);
        begin s=a+b; if(s>7) satadd=7; else if(s<-8) satadd=-8; else satadd=s; end
    endfunction
    function integer llr_of(input integer idx);     // deterministic input LLR pattern
        begin llr_of=(idx % 15) - 7; end            // ranges -7..7
    endfunction

    initial begin
        rvs[0]=0; es[0]=1040;  rvs[1]=0; es[1]=624;
        rvs[2]=1; es[2]=1040;  rvs[3]=3; es[3]=624;
        rvs[4]=0; es[4]=2700;                          // repetition (E>N): tests soft-combine
    end

    initial begin
        errors=0; total=0;
        aresetn=0; rv_in=0; e_in=0; s_td=0; s_tv=0; s_tl=0; m_tr=1;
        #50; @(posedge aclk); aresetn=1; @(posedge aclk);
        for(ci=0; ci<NCFG; ci=ci+1) begin
            // build expected buffer
            k0=k0_of(rvs[ci]);
            for(i=0;i<N;i=i+1) expbuf[i]=0;
            for(j=0;j<es[ci];j=j+1) begin addr=(k0+j)%N; expbuf[addr]=satadd(expbuf[addr], llr_of(j)); end
            // drive E LLRs
            rv_in=rvs[ci][1:0]; e_in=es[ci][ADDR_W-1:0];
            for(j=0;j<es[ci];j=j+1) begin
                @(negedge aclk); s_td=llr_of(j); s_tv=1; s_tl=(j==es[ci]-1);
                @(posedge aclk); while(!s_tr) @(posedge aclk);
            end
            @(negedge aclk); s_tv=0; s_tl=0;
            // read N outputs, compare
            for(i=0;i<N;i=i+1) begin
                @(posedge aclk); while(!m_tv) @(posedge aclk);
                if(m_td !== expbuf[i][LLR_W-1:0]) begin errors=errors+1;
                    if(errors<=15) $display("cfg%0d(rv%0d,E%0d) pos%0d got %0d exp %0d",ci,rvs[ci],es[ci],i,m_td,expbuf[i]); end
                if(i==N-1 && !m_tl) begin errors=errors+1; $display("cfg%0d tlast missing",ci); end
                total=total+1;
                @(negedge aclk);
            end
        end
        $display("TOTAL=%0d ERRORS=%0d",total,errors);
        if(errors==0) $display("PASS: rate_dematch scatter/zero-fill/soft-combine matches reference");
        else          $display("FAIL");
        $finish;
    end
    initial begin #60000000; $display("TIMEOUT total=%0d",total); $finish; end
endmodule
