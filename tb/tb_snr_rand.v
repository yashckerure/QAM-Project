`timescale 1ns/1ps
module tb_snr_rand;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [2:0] qam_mode;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] err_pow; wire err_valid;
    snr_estimator #(.AVG_SH(8)) dut(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .err_pow(err_pow),.err_valid(err_valid));
    integer i, seed, ni, nq, cnt; real true_pow; reg signed [15:0] sre, sim_;
    initial begin
        seed=77; aresetn=0; qam_mode=0; s_td=0; s_tv=0; s_tl=0;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        true_pow=0.0; cnt=0;
        for(i=0;i<8000;i=i+1) begin
            @(negedge aclk);
            sre = ($random(seed)&1)? 16'sd724 : -16'sd724;
            sim_= ($random(seed)&1)? 16'sd724 : -16'sd724;
            ni = $random(seed) % 61;   // [-60,60], well inside QPSK decision region
            nq = $random(seed) % 61;
            s_td = {sim_+nq[15:0], sre+ni[15:0]}; s_tv=1;
            if(i>=3000) begin true_pow = true_pow + (ni*ni+nq*nq); cnt=cnt+1; end
        end
        $display("err_pow=%0d  true_noise_pow=%.0f  ratio=%.3f",
                 err_pow, true_pow/cnt, err_pow/(true_pow/cnt));
        if(err_pow > (true_pow/cnt)*0.85 && err_pow < (true_pow/cnt)*1.15)
             $display("PASS: snr_estimator tracks true error power");
        else $display("CHECK mismatch");
        $finish;
    end
    initial begin #2000000; $finish; end
endmodule
