`timescale 1ns/1ps
module tb_agc_smoke;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] m_td; wire m_tv, m_tl; reg m_tr;
    wire [15:0] gain;

    agc #(.AVG_SH(6),.LOOP_SH(16)) dut(
        .aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl),
        .gain(gain));

    integer i, n, seed; real pw, sumP; integer yi, yq;
    reg signed [15:0] amp;
    task run_at_amp(input signed [15:0] A, input [200:0] label);
        begin
            amp=A; sumP=0.0; n=0;
            // converge
            for(i=0;i<120000;i=i+1) begin
                @(negedge aclk);
                s_td = {($random(seed)&1)?amp:-amp, ($random(seed)&1)?amp:-amp};
                s_tv=1; s_tl=0;
            end
            // measure last 20000
            for(i=0;i<20000;i=i+1) begin
                @(negedge aclk);
                s_td = {($random(seed)&1)?amp:-amp, ($random(seed)&1)?amp:-amp};
                s_tv=1;
                if(m_tv) begin
                    yi=$signed(m_td[15:0]); yq=$signed(m_td[31:16]);
                    pw=(yi/1024.0)*(yi/1024.0)+(yq/1024.0)*(yq/1024.0);
                    sumP=sumP+pw; n=n+1;
                end
            end
            $display("%0s in_amp=%0d  converged gain=%0d (%.3f)  out_power=%.3f (target 1.0)",
                     label, A, gain, gain/1024.0, sumP/n);
        end
    endtask

    initial begin
        seed=3; aresetn=0; s_td=0; s_tv=0; s_tl=0; m_tr=1;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        run_at_amp(16'sd256,  "ATTENUATED (amp 0.25):");   // power 0.125 -> gain ~2.83
        // reset between cases
        aresetn=0; @(posedge aclk); @(posedge aclk); aresetn=1; @(posedge aclk);
        run_at_amp(16'sd2048, "AMPLIFIED  (amp 2.0): ");    // power 8.0  -> gain ~0.354
        $finish;
    end
    initial begin #40000000; $display("TIMEOUT"); $finish; end
endmodule
