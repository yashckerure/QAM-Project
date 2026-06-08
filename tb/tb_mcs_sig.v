`timescale 1ns/1ps
// Loopback: mcs_insert -> (optional header corruption) -> mcs_recover.
// Checks recovered MCS, derived qam_mode/E/n, and payload pass-through, for
// several MCS values; plus robustness to one corrupted header symbol.
module tb_mcs_sig;
    localparam integer PLEN=8;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn;
    reg [2:0] mcs_in;
    reg [31:0] ins_s_td; reg ins_s_tv, ins_s_tl; wire ins_s_tr;
    wire [31:0] ins_m_td; wire ins_m_tv, ins_m_tl; wire ins_m_tr;
    // corruption hook
    reg corrupt_en; integer corrupt_idx; integer link_cnt;
    wire [31:0] link_td = (corrupt_en && link_cnt==corrupt_idx) ? {ins_m_td[31:16], -ins_m_td[15:0]} : ins_m_td;

    wire [31:0] rec_m_td; wire rec_m_tv, rec_m_tl; reg rec_m_tr;
    wire [2:0] rmcs, rqam; wire [12:0] re, rn; wire rmcs_v;

    mcs_insert u_ins(.aclk(aclk),.aresetn(aresetn),.mcs(mcs_in),
        .s_axis_tdata(ins_s_td),.s_axis_tvalid(ins_s_tv),.s_axis_tready(ins_s_tr),.s_axis_tlast(ins_s_tl),
        .m_axis_tdata(ins_m_td),.m_axis_tvalid(ins_m_tv),.m_axis_tready(ins_m_tr),.m_axis_tlast(ins_m_tl));

    mcs_recover u_rec(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(link_td),.s_axis_tvalid(ins_m_tv),.s_axis_tready(ins_m_tr),.s_axis_tlast(ins_m_tl),
        .m_axis_tdata(rec_m_td),.m_axis_tvalid(rec_m_tv),.m_axis_tready(rec_m_tr),.m_axis_tlast(rec_m_tl),
        .mcs(rmcs),.qam_mode(rqam),.e_out(re),.n_out(rn),.mcs_valid(rmcs_v));

    integer i, errors, seed, oc;
    reg [31:0] pay [0:PLEN-1];
    integer exp_qam, exp_e, exp_n;

    task run_packet(input [2:0] m, input do_corrupt, input integer cidx, input [200:0] tag);
        begin
            corrupt_en=do_corrupt; corrupt_idx=cidx; link_cnt=0;
            mcs_in=m;
            for(i=0;i<PLEN;i=i+1) pay[i] = { (($random(seed)&1)?16'sd724:-16'sd724),
                                             (($random(seed)&1)?16'sd724:-16'sd724) };
            // expected mapping
            case(m)
              0:begin exp_qam=0;exp_e=1040;exp_n=520;end
              1:begin exp_qam=1;exp_e=1040;exp_n=260;end
              2:begin exp_qam=1;exp_e=624; exp_n=156;end
              3:begin exp_qam=2;exp_e=624; exp_n=104;end
              default:begin exp_qam=3;exp_e=624;exp_n=78;end
            endcase
            oc=0;
            fork
                begin // drive payload into insert
                    for(i=0;i<PLEN;i=i+1) begin
                        @(negedge aclk); ins_s_td=pay[i]; ins_s_tv=1; ins_s_tl=(i==PLEN-1);
                        @(posedge aclk); while(!ins_s_tr) @(posedge aclk);
                    end
                    @(negedge aclk); ins_s_tv=0; ins_s_tl=0;
                end
                begin // count link symbols (for corruption index)
                    while(oc<PLEN) begin
                        @(posedge aclk);
                        if(ins_m_tv && ins_m_tr) link_cnt=link_cnt+1;
                        if(rec_m_tv && rec_m_tr) begin
                            if(rec_m_td !== pay[oc]) begin errors=errors+1;
                                $display("%0s payload[%0d] mismatch got %h exp %h",tag,oc,rec_m_td,pay[oc]); end
                            oc=oc+1;
                        end
                    end
                end
            join
            if(rmcs!==m) begin errors=errors+1; $display("%0s MCS got %0d exp %0d",tag,rmcs,m); end
            if(rqam!==exp_qam||re!==exp_e||rn!==exp_n) begin errors=errors+1;
                $display("%0s map got qam%0d e%0d n%0d exp qam%0d e%0d n%0d",tag,rqam,re,rn,exp_qam,exp_e,exp_n); end
            $display("%0s mcs=%0d->qam%0d/E%0d/n%0d  payload %0d/%0d ok", tag, rmcs, rqam, re, rn, oc, PLEN);
            repeat(4) @(posedge aclk);
        end
    endtask

    initial begin
        errors=0; seed=9; aresetn=0; mcs_in=0; ins_s_td=0; ins_s_tv=0; ins_s_tl=0;
        rec_m_tr=1; corrupt_en=0; corrupt_idx=0; link_cnt=0;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        run_packet(3'd0, 0, 0, "MCS0      ");
        run_packet(3'd2, 0, 0, "MCS2      ");
        run_packet(3'd4, 0, 0, "MCS4      ");
        run_packet(3'd3, 1, 1, "MCS3+err  ");   // corrupt header symbol index 1 (mcs[2] group)
        if(errors==0) $display("PASS: MCS signalling insert/recover (mapping + payload + 1-err robust)");
        else          $display("FAIL: errors=%0d", errors);
        $finish;
    end
    initial begin #1000000; $display("TIMEOUT"); $finish; end
endmodule
