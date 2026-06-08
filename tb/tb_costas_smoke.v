`timescale 1ns/1ps
module tb_costas_smoke;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] m_td; wire m_tv, m_tl; reg m_tr;
    wire signed [31:0] freq_dbg;

    costas #(.KP_SHL(14),.KI_SHL(4)) dut(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl),
        .freq_dbg(freq_dbg));

    real c, s;                 // 30 deg rotation applied to input
    integer i, seed, n;
    real si, sq, ri, rq, sumI, sumQ;
    integer yi, yq, ri_i, rq_i;

    initial begin
        c=0.8660254; s=0.5;    // 30 degrees
        seed=11; aresetn=0; s_td=0; s_tv=0; s_tl=0; m_tr=1;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);

        // converge: feed rotated QPSK
        for(i=0;i<4000;i=i+1) begin
            @(negedge aclk);
            si = ($random(seed)&1)? 724.0 : -724.0;
            sq = ($random(seed)&1)? 724.0 : -724.0;
            ri = si*c - sq*s;  rq = si*s + sq*c;
            ri_i = $rtoi(ri); rq_i = $rtoi(rq);
            s_td = {rq_i[15:0], ri_i[15:0]}; s_tv=1;
        end
        // measure: after lock, |yI| and |yQ| should be ~724 (on QPSK grid)
        sumI=0.0; sumQ=0.0; n=0;
        for(i=0;i<500;i=i+1) begin
            @(negedge aclk);
            si = ($random(seed)&1)? 724.0 : -724.0;
            sq = ($random(seed)&1)? 724.0 : -724.0;
            ri = si*c - sq*s;  rq = si*s + sq*c;
            ri_i = $rtoi(ri); rq_i = $rtoi(rq);
            s_td = {rq_i[15:0], ri_i[15:0]}; s_tv=1;
            if(m_tv) begin
                yi=$signed(m_td[15:0]); yq=$signed(m_td[31:16]);
                sumI = sumI + (yi<0? -yi : yi);
                sumQ = sumQ + (yq<0? -yq : yq);
                n=n+1;
            end
        end
        $display("after lock: mean|yI|=%.1f mean|yQ|=%.1f (on-grid target 724)  freq_dbg=%0d",
                 sumI/n, sumQ/n, freq_dbg);
        if(sumI/n>670.0 && sumI/n<780.0 && sumQ/n>670.0 && sumQ/n<780.0)
             $display("PASS: Costas removed the 30deg offset, constellation on QPSK grid");
        else $display("CHECK: constellation not on grid (loop gain/sign may need tuning)");
        $finish;
    end
    initial begin #2000000; $display("TIMEOUT"); $finish; end
endmodule
