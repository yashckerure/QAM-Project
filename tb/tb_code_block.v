`timescale 1ns/1ps
// Series test: data -> code_block_seg -> code_block_concat -> compare to data.
module tb_code_block;
    localparam integer B=520;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg  s_td, s_tv, s_tl; wire s_tr;
    wire mid_td, mid_tv, mid_tl; wire mid_tr;
    wire o_td, o_tv, o_tl; reg o_tr;

    code_block_seg #(.B(B),.KCB(3840)) u_seg(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(mid_td),.m_axis_tvalid(mid_tv),.m_axis_tready(mid_tr),.m_axis_tlast(mid_tl));
    code_block_concat u_concat(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(mid_td),.s_axis_tvalid(mid_tv),.s_axis_tready(mid_tr),.s_axis_tlast(mid_tl),
        .m_axis_tdata(o_td),.m_axis_tvalid(o_tv),.m_axis_tready(o_tr),.m_axis_tlast(o_tl));

    reg data [0:B-1];
    integer i, oc, errors, seed, bp;
    initial begin seed=44; for(i=0;i<B;i=i+1) data[i]=$random(seed)&1; end

    // checker: sample at negedge (stable), consume when valid && ready
    initial begin
        oc=0; errors=0; o_tr=1; bp=1;
        @(posedge aclk); wait(aresetn);
        forever begin
            @(negedge aclk);
            if(o_tv && o_tr) begin
                if(o_td !== data[oc]) begin errors=errors+1;
                    if(errors<=12) $display("MISMATCH idx %0d got %0d exp %0d",oc,o_td,data[oc]); end
                if(oc==B-1 && !o_tl) begin errors=errors+1; $display("tlast missing"); end
                if(oc<B-1 && o_tl) begin errors=errors+1; $display("early tlast at %0d",oc); end
                oc=oc+1;
                if(oc==B) begin
                    $display("DONE: %0d bits, ERRORS=%0d",oc,errors);
                    if(errors==0) $display("PASS: seg+concat identity (C=1) with tlast");
                    else          $display("FAIL");
                    $finish;
                end
            end
            // toggle backpressure on the NEGEDGE so it's stable across the next posedge
            bp = bp + 1;
            o_tr = (bp % 5 != 0);   // periodic, deterministic backpressure
        end
    end

    initial begin
        aresetn=0; s_td=0; s_tv=0; s_tl=0;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        for(i=0;i<B;i=i+1) begin
            @(negedge aclk); s_td=data[i]; s_tv=1; s_tl=(i==B-1);
            @(posedge aclk); while(!s_tr) @(posedge aclk);
        end
        @(negedge aclk); s_tv=0; s_tl=0;
    end
    initial begin #500000; $display("TIMEOUT oc=%0d",oc); $finish; end
endmodule
