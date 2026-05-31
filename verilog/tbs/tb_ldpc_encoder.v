`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : tb_ldpc_encoder.v
// Description  : Standalone TB for ldpc_encoder (BG2, Zc=52, K=520 -> N=2600).
//                Drives the 520-bit golden input (ldpc_in.txt) through the
//                AXI-Stream slave, dumps the 2600-bit master output to
//                ldpc_enc_hdl_out.txt, then diff against ldpc_ref.txt.
//=============================================================================
// Files required in the simulation working directory:
//   ldpc_in.txt   (520 lines, one bit each) - stimulus
//   ldpc_ref.txt  (2600 lines)              - golden output, for the diff
// Pass criterion: diff ldpc_enc_hdl_out.txt ldpc_ref.txt  -> no differences.
//=============================================================================

module tb_ldpc_encoder;

    localparam integer NUM_INPUT_BITS  = 520;
    localparam integer NUM_OUTPUT_BITS = 2600;

    reg aclk;
    reg aresetn;

    // DUT slave (driven by TB)
    reg  s_tdata;
    reg  s_tvalid;
    wire s_tready;
    reg  s_tlast;

    // DUT master (consumed by TB)
    wire m_tdata;
    wire m_tvalid;
    reg  m_tready;
    wire m_tlast;

    // --------------------------------------------------------------
    // DUT
    // --------------------------------------------------------------
    ldpc_encoder u_dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(s_tdata),
        .s_axis_tvalid(s_tvalid),
        .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast),
        .m_axis_tdata(m_tdata),
        .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast)
    );

    // --------------------------------------------------------------
    // 100 MHz clock
    // --------------------------------------------------------------
    initial aclk = 0;
    always #5 aclk = ~aclk;

    // --------------------------------------------------------------
    // Stimulus memory: 520 golden input bits
    // --------------------------------------------------------------
    reg stim_mem [0:NUM_INPUT_BITS-1];

    // --------------------------------------------------------------
    // Dump master output
    // --------------------------------------------------------------
    integer fd;
    integer out_count;

    initial begin
        out_count = 0;
        fd = $fopen("ldpc_enc_hdl_out.txt", "w");
    end

    always @(negedge aclk) begin
        if (aresetn && m_tvalid && m_tready) begin
            $fwrite(fd, "%0d\n", m_tdata);
            out_count = out_count + 1;
            if (out_count == NUM_OUTPUT_BITS) begin
                $fclose(fd);
                $display("DONE: %0d output bits dumped", out_count);
                if (m_tlast)
                    $display("PASS-CHECK: tlast asserted on final bit");
                else
                    $display("WARN: tlast NOT asserted on final bit");
                $finish;
            end
        end
    end

    // --------------------------------------------------------------
    // Main stimulus
    // --------------------------------------------------------------
    integer i;
    initial begin
        aresetn  = 1'b0;
        s_tdata  = 1'b0;
        s_tvalid = 1'b0;
        s_tlast  = 1'b0;
        m_tready = 1'b0;

        $readmemb("ldpc_in.txt", stim_mem);

        #50;
        @(posedge aclk);
        aresetn  = 1'b1;
        @(posedge aclk);
        m_tready = 1'b1;          // free-running consumer

        // Wait for the encoder to be ready to collect
        wait (s_tready);
        $display("Encoder ready to collect at time %0t ns", $time);

        for (i = 0; i < NUM_INPUT_BITS; i = i + 1) begin
            @(negedge aclk);
            s_tdata  = stim_mem[i];
            s_tvalid = 1'b1;
            s_tlast  = (i == NUM_INPUT_BITS - 1);
            @(posedge aclk);
            while (!s_tready) @(posedge aclk);
        end

        @(negedge aclk);
        s_tvalid = 1'b0;
        s_tlast  = 1'b0;

        wait (0);
    end

    // --------------------------------------------------------------
    // Safety timeout: collect 520 + core latency (<2500) + capture 50
    //                 + drain 2600 ~ under 60us. Allow 2ms.
    // --------------------------------------------------------------
    initial begin
        #2_000_000;
        $display("WARN: safety timeout, out_count=%0d", out_count);
        $fclose(fd);
        $finish;
    end

endmodule
