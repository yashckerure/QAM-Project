`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 00:08:08
// Design Name: 
// Module Name: tb_crc24a_attach
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
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : tb_crc24a_attach.v
// Description  : Standalone TB for crc24a_attach.
//                Drives 128 PRBS bits with tlast on the last bit, dumps every
//                output bit (info + 24 CRC) to crc_out.txt.
//=============================================================================
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : tb_crc24a_attach.v
// Description  : Standalone TB for crc24a_attach.
//                Uses procedural single-process stimulus to eliminate races.
//=============================================================================
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : crc24a_attach.v
// Description  : CRC-24A attach per TS 38.212 Section 5.1.
//                Registered AXI outputs to avoid combinational output races.
//=============================================================================
`timescale 1ns / 1ps

module tb_crc24a_attach;

    localparam integer NUM_INPUT_BITS = 496;
    localparam integer NUM_OUTPUT_BITS = NUM_INPUT_BITS + 24;

    reg  aclk;
    reg  aresetn;

    reg  s_in_tdata;
    reg  s_in_tvalid;
    wire s_in_tready;
    reg  s_in_tlast;

    wire m_out_tdata;
    wire m_out_tvalid;
    reg  m_out_tready;
    wire m_out_tlast;

    crc24a_attach dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(s_in_tdata),
        .s_axis_tvalid(s_in_tvalid),
        .s_axis_tready(s_in_tready),
        .s_axis_tlast(s_in_tlast),
        .m_axis_tdata(m_out_tdata),
        .m_axis_tvalid(m_out_tvalid),
        .m_axis_tready(m_out_tready),
        .m_axis_tlast(m_out_tlast)
    );

    initial aclk = 0;
    always #5 aclk = ~aclk;

    // -------------------------------------------------------------------------
    // PRBS-23 state, advanced procedurally by the stimulus driver
    // -------------------------------------------------------------------------
    reg [22:0] lfsr;

    function automatic [0:0] prbs_step_fn;
        input dummy;
        begin
            prbs_step_fn = lfsr[22];
            lfsr = {lfsr[21:0], lfsr[22] ^ lfsr[17]};
        end
    endfunction

    // -------------------------------------------------------------------------
    // Output dump - sampled on negedge, after DUT signals have settled
    // -------------------------------------------------------------------------
    integer fd;
    integer out_count;

    initial begin
        out_count = 0;
        fd = $fopen("crc_out.txt", "w");
    end

    always @(negedge aclk) begin
        if (aresetn && m_out_tvalid && m_out_tready) begin
            $fwrite(fd, "%0d\n", m_out_tdata);
            out_count = out_count + 1;
            if (out_count == NUM_OUTPUT_BITS) begin
                $fclose(fd);
                $display("DONE: %0d bits dumped", out_count);
                $finish;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Main stimulus driver
    // -------------------------------------------------------------------------
    integer i;
    initial begin
        // Reset state
        aresetn      = 1'b0;
        s_in_tdata   = 1'b0;
        s_in_tvalid  = 1'b0;
        s_in_tlast   = 1'b0;
        m_out_tready = 1'b0;
        lfsr         = 23'h5A3C7E;

        // Hold reset for 50 ns
        #50;
        @(posedge aclk);
        aresetn = 1'b1;

        // One settling clock before any data
        @(posedge aclk);

        // Now arm the consumer
        m_out_tready = 1'b1;

        // Drive 128 PRBS bits, one per clock, with tlast on the last bit
        for (i = 0; i < NUM_INPUT_BITS; i = i + 1) begin
            @(negedge aclk);                       // drive on negedge so DUT samples cleanly
            s_in_tdata  = prbs_step_fn(1'b0);
            s_in_tvalid = 1'b1;
            s_in_tlast  = (i == NUM_INPUT_BITS-1);
            @(posedge aclk);
            // Wait for DUT to accept; for this DUT, tready=1 in S_IDLE/S_PASSING
            // so the bit is consumed on this same posedge.
            while (!s_in_tready) @(posedge aclk);
        end

        // Deassert valid after last input bit
        @(negedge aclk);
        s_in_tvalid = 1'b0;
        s_in_tlast  = 1'b0;

        // Wait for the dump to complete (it will $finish)
        wait (0);
    end

    // Safety timeout
    initial begin
        #100000;
        $display("WARN: safety timeout, out_count=%0d", out_count);
        $fclose(fd);
        $finish;
    end

endmodule
