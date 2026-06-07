`timescale 1ns / 1ps
module tb_ldpc_decoder;
    localparam NIN=2600, NOUT=520;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg signed [3:0] s_tdata; reg s_tvalid, s_tlast; wire s_tready;
    wire m_tdata, m_tvalid, m_tlast; reg m_tready;
    ldpc_decoder dut(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(s_tdata),.s_axis_tvalid(s_tvalid),.s_axis_tready(s_tready),.s_axis_tlast(s_tlast),
        .m_axis_tdata(m_tdata),.m_axis_tvalid(m_tvalid),.m_axis_tready(m_tready),.m_axis_tlast(m_tlast));
    reg [3:0] llr [0:NIN-1];
    integer fd, oc, i;
    initial begin oc=0; fd=$fopen("dec_out.txt","w"); end
    always @(negedge aclk) if(aresetn && m_tvalid && m_tready) begin
        $fwrite(fd,"%0d\n",m_tdata); oc=oc+1;
        if(oc==NOUT) begin $fclose(fd);
            $display("DONE: %0d bits", oc);
            $display(m_tlast?"PASS-CHECK: tlast on final bit":"WARN: no tlast"); $finish; end
    end
    initial begin
        aresetn=0; s_tdata=0; s_tvalid=0; s_tlast=0; m_tready=0;
        $readmemb("dec_llr_in.txt", llr);
        #50; @(posedge aclk); aresetn=1; @(posedge aclk); m_tready=1;
        wait(s_tready);
        for(i=0;i<NIN;i=i+1) begin
            @(negedge aclk); s_tdata=$signed(llr[i]); s_tvalid=1; s_tlast=(i==NIN-1);
            @(posedge aclk); while(!s_tready) @(posedge aclk);
        end
        @(negedge aclk); s_tvalid=0; s_tlast=0; wait(0);
    end
    initial begin #3_000_000; $display("WARN timeout oc=%0d",oc); $fclose(fd); $finish; end
endmodule
