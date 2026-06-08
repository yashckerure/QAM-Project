`timescale 1ns/1ps
module tb_equalizer_train;
    localparam integer N=10000, CENTER=3;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [2:0] qam_mode; reg train_en; reg [31:0] d_train;
    reg [31:0] s_td; reg s_tv, s_tl; wire s_tr;
    wire [31:0] m_td; wire m_tv, m_tl; reg m_tr;

    equalizer #(.NTAPS(7),.CENTER(CENTER),.MU_SH(6)) dut(
        .aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .train_en(train_en),.d_train(d_train),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr),.m_axis_tlast(m_tl));

    integer sre [0:N-1];
    integer sim_ [0:N-1];
    integer i, seed, oc, ei, eq_;
    real err_first, err_last; integer nf, nl;
    integer exp_re, exp_im, yr, yq;
    reg [15:0] dtr_re, dtr_im;

    initial begin
        seed=21;
        for(i=0;i<N;i=i+1) begin
            sre[i] = ($random(seed)&1)? 724 : -724;
            sim_[i] = ($random(seed)&1)? 724 : -724;
        end
    end

    // checker: output oc should converge to s[oc-CENTER]
    initial begin
        oc=0; err_first=0.0; err_last=0.0; nf=0; nl=0;
        @(posedge aclk); wait(aresetn);
        forever begin
            @(negedge aclk);
            if(m_tv && m_tr) begin
                exp_re = (oc-CENTER>=0)? sre[oc-CENTER] : 0;
                exp_im = (oc-CENTER>=0)? sim_[oc-CENTER] : 0;
                yr=$signed(m_td[15:0]); yq=$signed(m_td[31:16]);
                ei = (yr-exp_re); if(ei<0) ei=-ei;
                eq_= (yq-exp_im); if(eq_<0) eq_=-eq_;
                if(oc>=20 && oc<220) begin err_first=err_first+ei+eq_; nf=nf+1; end
                if(oc>=N-1200 && oc<N-200) begin err_last=err_last+ei+eq_; nl=nl+1; end
                oc=oc+1;
                if(oc==N) begin
                    $display("LMS train: avg|err| first200=%.1f  last1000=%.1f (Q5.10 units, /axis-pair)",
                             err_first/nf, err_last/nl);
                    if(err_last/nl < err_first/nf*0.3 && err_last/nl < 120.0)
                         $display("PASS: equalizer converged - ISI error collapsed");
                    else $display("CHECK: insufficient convergence (tune MU_SH)");
                    $finish;
                end
            end
        end
    end

    // driver: rx[k] = s[k] + 0.4*s[k-1]; train with d_train = s[k-CENTER]
    initial begin
        aresetn=0; qam_mode=0; train_en=1; d_train=0; s_td=0; s_tv=0; s_tl=0; m_tr=1;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        for(i=0;i<N;i=i+1) begin
            @(negedge aclk);
            // ISI channel (0.4 = 410/1024)
            ei = sre[i] + ((i>0)? ((sre[i-1]*410)>>>10) : 0);
            eq_= sim_[i] + ((i>0)? ((sim_[i-1]*410)>>>10) : 0);
            s_td = {eq_[15:0], ei[15:0]};
            dtr_re = (i-CENTER>=0)? sre[i-CENTER][15:0] : 16'd0;
            dtr_im = (i-CENTER>=0)? sim_[i-CENTER][15:0] : 16'd0;
            d_train = {dtr_im, dtr_re};
            s_tv=1; s_tl=0;
            @(posedge aclk); while(!s_tr) @(posedge aclk);
        end
        s_tv=0;
    end
    initial begin #4000000; $display("TIMEOUT oc=%0d",oc); $finish; end
endmodule
