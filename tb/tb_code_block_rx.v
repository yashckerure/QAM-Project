`timescale 1ns/1ps
// Verifies RX inverses as C=1 identity pass-throughs with tlast + backpressure.
//  Phase 1: code_block_de_concat (4-bit signed LLR)
//  Phase 2: code_block_deseg     (1-bit)
module tb_code_block_rx;
    localparam integer E=624, B=520, LLR_W=4;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;

    // ---- de_concat (LLR) ----
    reg  signed [LLR_W-1:0] dc_s_td; reg dc_s_tv, dc_s_tl; wire dc_s_tr;
    wire signed [LLR_W-1:0] dc_o_td; wire dc_o_tv, dc_o_tl; reg dc_o_tr;
    code_block_de_concat #(.LLR_W(LLR_W)) u_dc(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(dc_s_td),.s_axis_tvalid(dc_s_tv),.s_axis_tready(dc_s_tr),.s_axis_tlast(dc_s_tl),
        .m_axis_tdata(dc_o_td),.m_axis_tvalid(dc_o_tv),.m_axis_tready(dc_o_tr),.m_axis_tlast(dc_o_tl));

    // ---- deseg (bit) ----
    reg  ds_s_td; reg ds_s_tv, ds_s_tl; wire ds_s_tr;
    wire ds_o_td; wire ds_o_tv, ds_o_tl; reg ds_o_tr;
    code_block_deseg #(.B(B),.KCB(3840)) u_ds(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(ds_s_td),.s_axis_tvalid(ds_s_tv),.s_axis_tready(ds_s_tr),.s_axis_tlast(ds_s_tl),
        .m_axis_tdata(ds_o_td),.m_axis_tvalid(ds_o_tv),.m_axis_tready(ds_o_tr),.m_axis_tlast(ds_o_tl));

    integer i, oc, errors, bp;
    reg signed [LLR_W-1:0] exp_llr; reg exp_bit;
    function integer llr_of(input integer k); begin llr_of=(k%15)-7; end endfunction
    function integer bit_of(input integer k); begin bit_of=((k*7+3)%2); end endfunction

    // ---------- Phase 1: de_concat ----------
    initial begin
        errors=0;
        // de_concat checker
        oc=0; dc_o_tr=1; bp=1;
        aresetn=0; dc_s_td=0; dc_s_tv=0; dc_s_tl=0;
        ds_s_td=0; ds_s_tv=0; ds_s_tl=0; ds_o_tr=1;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);

        // drive E LLRs into de_concat
        fork
            begin // driver
                for(i=0;i<E;i=i+1) begin
                    @(negedge aclk); dc_s_td=llr_of(i); dc_s_tv=1; dc_s_tl=(i==E-1);
                    @(posedge aclk); while(!dc_s_tr) @(posedge aclk);
                end
                @(negedge aclk); dc_s_tv=0; dc_s_tl=0;
            end
            begin // checker
                while(oc<E) begin
                    @(negedge aclk);
                    if(dc_o_tv && dc_o_tr) begin
                        exp_llr=llr_of(oc);
                    if(dc_o_td !== exp_llr) begin errors=errors+1;
                            if(errors<=10) $display("DECONCAT idx %0d got %0d exp %0d",oc,dc_o_td,llr_of(oc)); end
                        if(oc==E-1 && !dc_o_tl) begin errors=errors+1; $display("deconcat tlast missing"); end
                        if(oc<E-1 && dc_o_tl) begin errors=errors+1; $display("deconcat early tlast %0d",oc); end
                        oc=oc+1;
                    end
                    bp=bp+1; dc_o_tr=(bp%5!=0);
                end
            end
        join
        $display("PHASE1 de_concat: %0d LLRs, errors so far=%0d",oc,errors);

        // ---------- Phase 2: deseg ----------
        oc=0; ds_o_tr=1; bp=2;
        @(posedge aclk);
        fork
            begin
                for(i=0;i<B;i=i+1) begin
                    @(negedge aclk); ds_s_td=bit_of(i); ds_s_tv=1; ds_s_tl=(i==B-1);
                    @(posedge aclk); while(!ds_s_tr) @(posedge aclk);
                end
                @(negedge aclk); ds_s_tv=0; ds_s_tl=0;
            end
            begin
                while(oc<B) begin
                    @(negedge aclk);
                    if(ds_o_tv && ds_o_tr) begin
                        exp_bit=bit_of(oc);
                    if(ds_o_td !== exp_bit) begin errors=errors+1;
                            if(errors<=10) $display("DESEG idx %0d got %0d exp %0d",oc,ds_o_td,bit_of(oc)); end
                        if(oc==B-1 && !ds_o_tl) begin errors=errors+1; $display("deseg tlast missing"); end
                        if(oc<B-1 && ds_o_tl) begin errors=errors+1; $display("deseg early tlast %0d",oc); end
                        oc=oc+1;
                    end
                    bp=bp+1; ds_o_tr=(bp%4!=0);
                end
            end
        join
        $display("PHASE2 deseg: %0d bits, TOTAL ERRORS=%0d",oc,errors);
        if(errors==0) $display("PASS: de_concat (LLR) + deseg (bit) identity (C=1) with tlast");
        else          $display("FAIL");
        $finish;
    end
    initial begin #1000000; $display("TIMEOUT"); $finish; end
endmodule
