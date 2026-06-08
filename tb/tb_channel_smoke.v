`timescale 1ns/1ps
module tb_channel_smoke;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [15:0] noise_std;
    reg tap_we; reg [2:0] tap_idx; reg signed [15:0] tap_re, tap_im;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] m_td; wire m_tv, m_tl; reg m_tr;

    channel #(.NUM_TAPS(8),.IDX_W(12)) dut(
        .aclk(aclk),.aresetn(aresetn),.noise_std(noise_std),
        .tap_we(tap_we),.tap_idx(tap_idx),.tap_re(tap_re),.tap_im(tap_im),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl));

    localparam signed [15:0] SIG = 16'sd724;   // QPSK component, Q5.10 ~0.707
    integer i, n, errs;
    real sum, sumsq, d, mn, sd;
    integer ni, nq;

    initial begin
        aresetn=0; noise_std=0; tap_we=0; tap_idx=0; tap_re=0; tap_im=0;
        s_td={SIG,SIG}; s_tv=0; s_tl=0; m_tr=1;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);

        // ---- Phase 1: unit tap, no noise -> pass-through (after 4-cyc latency) ----
        s_tv=1;
        repeat(10) @(posedge aclk);
        errs=0;
        for(i=0;i<50;i=i+1) begin
            @(negedge aclk);
            if(m_tv && (m_td !== {SIG,SIG})) begin errs=errs+1;
                if(errs<=4) $display("passthru mismatch: got %h exp %h",m_td,{SIG,SIG}); end
        end
        if(errs==0) $display("PHASE1 PASS: unit-tap + noise_std=0 is pass-through");
        else        $display("PHASE1 FAIL: errs=%0d",errs);

        // ---- Phase 2: add noise, measure std of I component ----
        noise_std=16'd102;   // 0.0996 in Q5.10 -> expect noise std ~102
        @(posedge aclk);
        sum=0.0; sumsq=0.0; n=0;
        for(i=0;i<40000;i=i+1) begin
            @(negedge aclk);
            if(m_tv) begin
                ni = $signed(m_td[15:0]) - SIG;     // noise on I = out - signal
                sum  = sum + ni;
                sumsq= sumsq + (ni*1.0)*(ni*1.0);
                n = n + 1;
            end
        end
        mn = sum/n;
        sd = $sqrt(sumsq/n - mn*mn);
        $display("PHASE2: N=%0d  noise mean=%.2f (exp ~0)  noise std=%.2f (exp ~102)", n, mn, sd);
        if(sd>85.0 && sd<120.0 && mn>-8.0 && mn<8.0)
             $display("PHASE2 PASS: Box-Muller noise std within tolerance");
        else $display("PHASE2 CHECK: std/mean outside loose tolerance (inspect)");

        // ---- Phase 3: multipath echo (tap0=1.0, tap2=0.5) sanity, no noise ----
        noise_std=0;
        @(posedge aclk);
        tap_we=1; tap_idx=2; tap_re=16'sd8192; tap_im=16'sd0; @(posedge aclk); // 0.5 @ Q1.14
        tap_we=0; @(posedge aclk);
        // with constant input SIG, steady-state out = SIG*(1.0+0.5)=1086
        repeat(12) @(posedge aclk);
        @(negedge aclk);
        if(m_tv) $display("PHASE3: echo out I=%0d (exp ~1086 = 724*1.5)", $signed(m_td[15:0]));
        $finish;
    end
    initial begin #6000000; $display("TIMEOUT"); $finish; end
endmodule
