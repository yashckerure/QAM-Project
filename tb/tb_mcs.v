`timescale 1ns/1ps
module tb_mcs;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn; reg [31:0] metric; reg metric_valid;
    reg force_en; reg [2:0] force_mcs;
    wire [2:0] mcs, qam_mode; wire [12:0] e_out, n_out; wire [1:0] rv_out;

    mcs_controller #(.DWELL(8)) dut(.aclk(aclk),.aresetn(aresetn),
        .metric(metric),.metric_valid(metric_valid),
        .force_en(force_en),.force_mcs(force_mcs),
        .mcs(mcs),.qam_mode(qam_mode),.e_out(e_out),.n_out(n_out),.rv_out(rv_out));

    integer errors;
    task expect_map(input [2:0] m, input [2:0] qm, input [12:0] e, input [12:0] n);
        begin
            if(mcs!==m||qam_mode!==qm||e_out!==e||n_out!==n) begin errors=errors+1;
              $display("MAP ERR at mcs=%0d: qm=%0d e=%0d n=%0d (exp qm=%0d e=%0d n=%0d)",
                       mcs,qam_mode,e_out,n_out,qm,e,n); end
        end
    endtask
    task settle(input [31:0] mval, input [2:0] expect_mcs, input [200:0] tag);
        integer k;
        begin
            metric=mval;
            for(k=0;k<200;k=k+1) @(posedge aclk);   // allow dwell stepping
            $display("%0s metric=%0d -> mcs=%0d (expect %0d)", tag, mval, mcs, expect_mcs);
            if(mcs!==expect_mcs) errors=errors+1;
        end
    endtask

    initial begin
        errors=0; aresetn=0; metric=32'd50000; metric_valid=1; force_en=0; force_mcs=0;
        #40; @(posedge aclk); aresetn=1; @(posedge aclk);

        // climb: decreasing metric (improving channel) 0->4
        settle(32'd50000, 3'd0, "bad      ");
        settle(32'd15000, 3'd1, "ok       "); expect_map(3'd1,3'd1,13'd1040,13'd260);
        settle(32'd6000,  3'd2, "good     "); expect_map(3'd2,3'd1,13'd624, 13'd156);
        settle(32'd2500,  3'd3, "better   "); expect_map(3'd3,3'd2,13'd624, 13'd104);
        settle(32'd800,   3'd4, "best     "); expect_map(3'd4,3'd3,13'd624, 13'd78);
        // map at MCS0 too
        // fall: increasing metric (degrading) 4->0
        settle(32'd50000, 3'd0, "degraded ");  expect_map(3'd0,3'd0,13'd1040,13'd520);

        // force override
        force_en=1; force_mcs=3'd3; repeat(5) @(posedge aclk);
        $display("force MCS3 -> mcs=%0d qm=%0d e=%0d", mcs,qam_mode,e_out);
        if(mcs!==3'd3) errors=errors+1;
        force_en=0;

        if(errors==0) $display("PASS: MCS adaptation (climb/fall + table mapping + force)");
        else          $display("FAIL: errors=%0d", errors);
        $finish;
    end
    initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule
