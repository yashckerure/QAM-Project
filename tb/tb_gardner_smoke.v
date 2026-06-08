`timescale 1ns/1ps
module tb_gardner_smoke;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] m_td; wire m_tv, m_tl; reg m_tr;
    wire signed [15:0] mu_step_dbg;

    gardner dut(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl),
        .mu_step_dbg(mu_step_dbg));

    integer i, nin, nout, seed, ph, smin, smax;
    reg signed [15:0] sre, sim;

    initial begin
        seed=5; aresetn=0; s_td=0; s_tv=0; s_tl=0; m_tr=1;
        nin=0; nout=0; ph=0; smin=99999; smax=-99999;
        sre=724; sim=724;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);

        // feed 8000 input samples = rectangular-hold QPSK at 4 sps (zero offset)
        for(i=0;i<8000;i=i+1) begin
            @(negedge aclk);
            if(ph==0) begin   // new symbol every 4 samples
                sre = ($random(seed)&1) ? 16'sd724 : -16'sd724;
                sim = ($random(seed)&1) ? 16'sd724 : -16'sd724;
            end
            s_td = {sim, sre}; s_tv = 1; s_tl = 0;
            ph = (ph==3) ? 0 : ph+1;
            @(posedge aclk);
            if(s_tr) nin=nin+1;
            if(m_tv && m_tr) begin
                nout=nout+1;
                if(mu_step_dbg<smin) smin=mu_step_dbg;
                if(mu_step_dbg>smax) smax=mu_step_dbg;
            end
        end
        s_tv=0;
        repeat(20) @(posedge aclk);

        $display("inputs accepted=%0d  symbols out=%0d  (expect ~%0d)", nin, nout, nin/4);
        $display("mu_step range [%0d, %0d]  (nominal 4096, clamp +/-512)", smin, smax);
        if(nout >= (nin/4)-3 && nout <= (nin/4)+3)
             $display("RATE OK: ~1 symbol per 4 input samples");
        else $display("RATE CHECK: unexpected symbol count");
        if(smin>=4096-512 && smax<=4096+512)
             $display("LOOP STABLE: mu_step stayed within clamp, no runaway");
        else $display("LOOP CHECK: mu_step outside expected band");
        $finish;
    end
    initial begin #2000000; $display("TIMEOUT"); $finish; end
endmodule
