`timescale 1ns/1ps
module tb_rrc_rx;
    localparam DW=16;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [2*DW-1:0] s_td; reg s_tv; wire s_tr;
    wire [2*DW-1:0] m_td; wire m_tv; reg m_tr;
    rrc_rx dut(.aclk(aclk),.aresetn(aresetn),.s_axis_tdata(s_td),.s_axis_tvalid(s_tv),
        .s_axis_tready(s_tr),.m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr));
    integer fin,fexp,c,Ih,Qh,ei,eq,errors,total,NS;
    reg [15:0] ai[0:1023], aq[0:1023];
    reg signed [DW-1:0] gi,gq;
    integer fed,i;
    initial begin
        NS=0; fin=$fopen("rx_in.txt","r");
        while(!$feof(fin)) begin c=$fscanf(fin,"%h %h\n",Ih,Qh);
            if(c==2) begin ai[NS]=Ih[15:0]; aq[NS]=Qh[15:0]; NS=NS+1; end end
        $fclose(fin);
    end
    initial begin   // continuous input
        aresetn=0; s_tv=0; s_td=0; m_tr=1; fed=0;
        #40; @(posedge aclk); aresetn=1; @(negedge aclk);
        while(fed<NS) begin s_td={aq[fed],ai[fed]}; s_tv=1;
            @(posedge aclk); if(s_tr) fed=fed+1; @(negedge aclk); end
        s_tv=0;
    end
    initial begin
        errors=0; total=0; fexp=$fopen("rx_exp.txt","r");
        @(posedge aclk); wait(aresetn);
        for(i=0;i<NS+60;i=i+1) begin
            @(posedge aclk);
            if(m_tv && total<NS) begin
                gi=m_td[DW-1:0]; gq=m_td[2*DW-1:DW];
                c=$fscanf(fexp,"%d %d",ei,eq);
                if(gi!==ei[15:0]||gq!==eq[15:0]) begin errors=errors+1;
                    if(errors<=12) $display("MISMATCH #%0d got(%0d,%0d) exp(%0d,%0d)",total,gi,gq,ei,eq); end
                total=total+1;
            end
        end
        $display("TOTAL=%0d (expect %0d) ERRORS=%0d",total,NS,errors);
        $display(errors==0&&total==NS?"PASS: rrc_rx matches matched-filter golden":"FAIL");
        $fclose(fexp); $finish;
    end
    initial begin #4000000; $display("TIMEOUT"); $finish; end
endmodule
