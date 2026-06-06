`timescale 1ns / 1ps
module tb_soft_demapper;
    localparam DATA_W=16;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [2:0] qam_mode; reg [2*DATA_W-1:0] s_tdata;
    reg s_tvalid; wire s_tready;
    wire signed [3:0] m_tdata; wire m_tvalid, m_tlast; reg m_tready;

    soft_demapper dut(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .s_axis_tdata(s_tdata),.s_axis_tvalid(s_tvalid),.s_axis_tready(s_tready),
        .m_axis_tdata(m_tdata),.m_axis_tvalid(m_tvalid),.m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast));

    integer fin, fexp, r, mode, Qm, i, errors, total, code;
    integer Ihex, Qhex;
    reg signed [3:0] exp [0:7];
    reg signed [3:0] got [0:7];

    task get_symbol; begin
        code=$fscanf(fin,"%d %h %h %d\n",mode,Ihex,Qhex,Qm);
        // read expected line
        for(i=0;i<Qm;i=i+1) r=$fscanf(fexp,"%d",exp[i]);
    end endtask

    initial begin
        errors=0; total=0;
        aresetn=0; s_tvalid=0; m_tready=1; qam_mode=0; s_tdata=0;
        fin=$fopen("vec_in.txt","r"); fexp=$fopen("vec_exp.txt","r");
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);
        while(!$feof(fin)) begin
            get_symbol;
            if(code==4) begin
                @(negedge aclk);
                qam_mode = mode[2:0];
                s_tdata  = {Qhex[15:0], Ihex[15:0]};
                s_tvalid = 1;
                @(posedge aclk); while(!s_tready) @(posedge aclk);
                @(negedge aclk); s_tvalid=0;
                // collect Qm LLRs
                for(i=0;i<Qm;i=i+1) begin
                    @(posedge aclk); while(!m_tvalid) @(posedge aclk);
                    got[i]=m_tdata;
                    if(i==Qm-1 && !m_tlast) begin errors=errors+1; $display("tlast missing mode=%0d",mode); end
                    @(negedge aclk);
                end
                for(i=0;i<Qm;i=i+1) begin
                    total=total+1;
                    if(got[i]!==exp[i]) begin errors=errors+1;
                        if(errors<=20) $display("MISMATCH mode=%0d I=%0h Q=%0h slot=%0d got=%0d exp=%0d",mode,Ihex,Qhex,i,got[i],exp[i]);
                    end
                end
            end
        end
        $display("TOTAL LLRs=%0d  ERRORS=%0d", total, errors);
        if(errors==0) $display("PASS: soft_demapper matches max-log reference");
        else          $display("FAIL");
        $fclose(fin); $fclose(fexp); $finish;
    end
    initial begin #5000000; $display("TIMEOUT total=%0d",total); $finish; end
endmodule
