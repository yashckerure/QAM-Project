//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.06.2026 19:13:18
// Design Name: 
// Module Name: tb_rrc_tx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
module tb_rrc_tx;
    localparam DW=16, SPS=4;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [2*DW-1:0] s_td; reg s_tv; wire s_tr;
    wire [2*DW-1:0] m_td; wire m_tv; reg m_tr;
    rrc_tx dut(.aclk(aclk),.aresetn(aresetn),.s_axis_tdata(s_td),.s_axis_tvalid(s_tv),
        .s_axis_tready(s_tr),.m_axis_tdata(m_td),.m_axis_tvalid(m_tv),.m_axis_tready(m_tr));

    integer NS, i, c, ei, eq, errors, total, gaps, started;
    reg [15:0] si_arr [0:1023]; reg [15:0] sq_arr [0:1023];
    reg signed [DW-1:0] gi,gq;
    integer fin,fexp,Ih,Qh,fed;

    initial begin
        NS=0; fin=$fopen("tx_in.txt","r");
        while(!$feof(fin)) begin c=$fscanf(fin,"%h %h\n",Ih,Qh);
            if(c==2) begin si_arr[NS]=Ih[15:0]; sq_arr[NS]=Qh[15:0]; NS=NS+1; end end
        $fclose(fin);
    end

    initial begin
        aresetn=0; s_tv=0; s_td=0; m_tr=1; fed=0;
        #40; @(posedge aclk); aresetn=1;
        @(negedge aclk);
        while(fed<NS) begin
            s_td={sq_arr[fed],si_arr[fed]}; s_tv=1;
            @(posedge aclk);
            if(s_tr) fed=fed+1;
            @(negedge aclk);
        end
        s_tv=0;
    end

    initial begin
        errors=0; total=0; gaps=0; started=0;
        fexp=$fopen("tx_exp.txt","r");
        @(posedge aclk); wait(aresetn);
        for(i=0;i<NS*SPS + 50;i=i+1) begin
            @(posedge aclk);
            if(m_tv) begin
                started=1;
                if(total < NS*SPS) begin
                    gi=m_td[DW-1:0]; gq=m_td[2*DW-1:DW];
                    c=$fscanf(fexp,"%d %d",ei,eq);
                    if(gi!==ei[15:0]||gq!==eq[15:0]) begin errors=errors+1;
                        if(errors<=12) $display("VAL MISMATCH #%0d got(%0d,%0d) exp(%0d,%0d)",total,gi,gq,ei,eq); end
                    total=total+1;
                end
            end else if(started && total<NS*SPS) begin
                gaps=gaps+1;
            end
        end
        $display("TOTAL=%0d (expect %0d)  VAL-ERRORS=%0d  GAPS=%0d",total,NS*SPS,errors,gaps);
        if(errors==0 && gaps==0 && total==NS*SPS)
             $display("PASS: values correct AND gap-free (seamless)");
        else $display("FAIL");
        $fclose(fexp); $finish;
    end
    initial begin #6000000; $display("TIMEOUT"); $finish; end
endmodule
