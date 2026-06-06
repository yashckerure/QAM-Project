`timescale 1ns/1ps
// Verifies llr_descrambler against the proven bit descrambler used as oracle:
//  - descrambler fed all-zero bits -> its output bit at beat n IS c(n).
//  - both blocks share C_INIT and the 1600-cycle warmup, so they run in lockstep.
//  - expected llr_out(n) = c(n) ? sat_neg(llr_in(n)) : llr_in(n).
// Also cross-checks c(n) against a TB-internal LFSR replica.
module tb_llr_descrambler;
    localparam integer N=520, LLR_W=4;
    localparam [30:0] C_INIT = 31'h00008000;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;

    // common handshake driven to both DUT and oracle
    reg  s_tv, s_tl;
    // DUT (llr)
    reg  signed [LLR_W-1:0] dut_in;
    wire signed [LLR_W-1:0] dut_out; wire dut_tr, dut_ov, dut_ol;
    // oracle (bit descrambler), input tied to 0 -> output = c(n)
    wire orc_out, orc_tr, orc_ov, orc_ol;

    llr_descrambler #(.C_INIT(C_INIT),.LLR_W(LLR_W)) dut(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(dut_in),.s_axis_tvalid(s_tv),.s_axis_tready(dut_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(dut_out),.m_axis_tvalid(dut_ov),.m_axis_tready(1'b1),.m_axis_tlast(dut_ol));
    descrambler #(.C_INIT(C_INIT)) orc(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(1'b0),.s_axis_tvalid(s_tv),.s_axis_tready(orc_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(orc_out),.m_axis_tvalid(orc_ov),.m_axis_tready(1'b1),.m_axis_tlast(orc_ol));

    // TB-internal Gold-sequence replica
    reg [30:0] x1, x2; reg tb_c [0:N-1];
    integer k;
    initial begin
        x1=31'h1; x2=C_INIT;
        for(k=0;k<1600;k=k+1) begin
            x1={ (x1[3]^x1[0]), x1[30:1]};
            x2={ (x2[3]^x2[2]^x2[1]^x2[0]), x2[30:1]};
        end
        for(k=0;k<N;k=k+1) begin
            tb_c[k]=x1[0]^x2[0];
            x1={ (x1[3]^x1[0]), x1[30:1]};
            x2={ (x2[3]^x2[2]^x2[1]^x2[0]), x2[30:1]};
        end
    end

    function integer llr_in_of(input integer n); begin llr_in_of=(n%16)-8; end endfunction // -8..7
    function integer sat_neg(input integer v); begin if(v==-8) sat_neg=7; else sat_neg=-v; end endfunction

    integer i, oc, errors;
    reg signed [LLR_W-1:0] exp_llr, drv;

    // checker (negedge sampled, lockstep beats)
    initial begin
        oc=0; errors=0;
        wait(aresetn);
        forever begin
            @(negedge aclk);
            if(dut_ov) begin
                // oracle output should equal c(n)
                if(orc_out !== tb_c[oc]) begin errors=errors+1;
                    if(errors<=10) $display("c-mismatch n%0d oracle %0d replica %0d",oc,orc_out,tb_c[oc]); end
                exp_llr = tb_c[oc] ? sat_neg(llr_in_of(oc)) : llr_in_of(oc);
                if(dut_out !== exp_llr) begin errors=errors+1;
                    if(errors<=10) $display("llr n%0d c%0d in %0d got %0d exp %0d",oc,tb_c[oc],llr_in_of(oc),dut_out,exp_llr); end
                if(oc==N-1 && !dut_ol) begin errors=errors+1; $display("tlast missing"); end
                oc=oc+1;
                if(oc==N) begin
                    $display("DONE: %0d LLRs, ERRORS=%0d",oc,errors);
                    if(errors==0) $display("PASS: llr_descrambler matches Gold seq + saturating negate");
                    else          $display("FAIL");
                    $finish;
                end
            end
        end
    end

    // driver
    initial begin
        aresetn=0; s_tv=0; s_tl=0; dut_in=0;
        #50; @(posedge aclk); aresetn=1; @(posedge aclk);
        wait(dut_tr);   // warmup done (oracle finishes same cycle)
        for(i=0;i<N;i=i+1) begin
            @(negedge aclk); drv=llr_in_of(i); dut_in=drv; s_tv=1; s_tl=(i==N-1);
            @(posedge aclk); while(!dut_tr) @(posedge aclk);
        end
        @(negedge aclk); s_tv=0; s_tl=0;
    end
    initial begin #2_000_000; $display("TIMEOUT oc=%0d",oc); $finish; end
endmodule
