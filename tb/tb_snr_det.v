`timescale 1ns/1ps
module tb_snr_det;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [2:0] qam_mode;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] err_pow; wire err_valid;
    snr_estimator #(.AVG_SH(8)) dut(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .err_pow(err_pow),.err_valid(err_valid));
    integer i; reg signed [15:0] sre, sim_;
    initial begin
        aresetn=0; qam_mode=0; s_td=0; s_tv=0; s_tl=0;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        // deterministic noise +50 on each axis: expect err_pow -> 50^2+50^2 = 5000
        for(i=0;i<5000;i=i+1) begin
            @(negedge aclk);
            sre = (i&1)? 16'sd724 : -16'sd724;
            sim_= (i&2)? 16'sd724 : -16'sd724;
            s_td = {sim_+16'sd50, sre+16'sd50}; s_tv=1;
        end
        $display("deterministic +50/+50: err_pow=%0d (expect 5000)", err_pow);
        if(err_pow>4500 && err_pow<5500) $display("PASS estimator scaling correct");
        else $display("FAIL estimator scaling: ratio=%.2f", err_pow/5000.0);
        $finish;
    end
    initial begin #1000000; $finish; end
endmodule
