`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 20:10:45
// Design Name: 
// Module Name: tb_crc24a_check
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
// File         : tb_crc24a_check.v
// Description  : Loopback TB: bit_source -> crc24a_attach -> crc24a_check.
//                Expects crc24a_check output == original PRBS-23 stream and
//                crc_ok = 1.
//=============================================================================
`timescale 1ns / 1ps

module tb_crc24a_check;

    localparam integer NUM_INPUT_BITS = 496;

    reg  aclk;
    reg  aresetn;

    // Stage 1: PRBS bit source (TB-local)
    reg  prbs_tdata;
    reg  prbs_tvalid;
    wire prbs_tready;
    reg  prbs_tlast;

    // Stage 2: crc24a_attach outputs
    wire attach_tdata;
    wire attach_tvalid;
    wire attach_tready;
    wire attach_tlast;

    // Stage 3: crc24a_check outputs (DUT)
    wire chk_tdata;
    wire chk_tvalid;
    reg  chk_tready;
    wire chk_tlast;
    wire chk_crc_ok;
    wire chk_crc_valid;

    // --------------------------------------------------------------
    // TX-side: CRC attach
    // --------------------------------------------------------------
    crc24a_attach u_attach (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(prbs_tdata),
        .s_axis_tvalid(prbs_tvalid),
        .s_axis_tready(prbs_tready),
        .s_axis_tlast(prbs_tlast),
        .m_axis_tdata(attach_tdata),
        .m_axis_tvalid(attach_tvalid),
        .m_axis_tready(attach_tready),
        .m_axis_tlast(attach_tlast)
    );

    // --------------------------------------------------------------
    // RX-side: CRC check (DUT)
    // --------------------------------------------------------------
    crc24a_check dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(attach_tdata),
        .s_axis_tvalid(attach_tvalid),
        .s_axis_tready(attach_tready),
        .s_axis_tlast(attach_tlast),
        .m_axis_tdata(chk_tdata),
        .m_axis_tvalid(chk_tvalid),
        .m_axis_tready(chk_tready),
        .m_axis_tlast(chk_tlast),
        .crc_ok(chk_crc_ok),
        .crc_valid(chk_crc_valid)
    );

    initial aclk = 0;
    always #5 aclk = ~aclk;

    // PRBS-23 driver
    reg [22:0] lfsr;

    function automatic [0:0] prbs_step_fn;
        input dummy;
        begin
            prbs_step_fn = lfsr[22];
            lfsr = {lfsr[21:0], lfsr[22] ^ lfsr[17]};
        end
    endfunction

    // Dump output info bits
    integer fd;
    integer out_count;
    reg     crc_ok_captured;

    initial begin
        out_count       = 0;
        crc_ok_captured = 1'b0;
        fd              = $fopen("crc_check_out.txt", "w");
    end

    always @(negedge aclk) begin
        if (aresetn && chk_tvalid && chk_tready) begin
            $fwrite(fd, "%0d\n", chk_tdata);
            out_count = out_count + 1;
        end
        if (aresetn && chk_crc_valid) begin
            crc_ok_captured = chk_crc_ok;
            $display("CRC status: crc_ok=%0d at time %0t ns",
                     chk_crc_ok, $time);
        end
        if (out_count == NUM_INPUT_BITS) begin
            #20;
            $fclose(fd);
            $display("DONE: %0d bits dumped, crc_ok=%0d",
                     out_count, crc_ok_captured);
            $finish;
        end
    end

    integer i;
    initial begin
        aresetn     = 1'b0;
        prbs_tdata  = 1'b0;
        prbs_tvalid = 1'b0;
        prbs_tlast  = 1'b0;
        chk_tready  = 1'b0;
        lfsr        = 23'h5A3C7E;

        #50;
        @(posedge aclk);
        aresetn = 1'b1;
        @(posedge aclk);
        chk_tready = 1'b1;

        for (i = 0; i < NUM_INPUT_BITS; i = i + 1) begin
            @(negedge aclk);
            prbs_tdata  = prbs_step_fn(1'b0);
            prbs_tvalid = 1'b1;
            prbs_tlast  = (i == NUM_INPUT_BITS - 1);
            @(posedge aclk);
            while (!prbs_tready) @(posedge aclk);
        end

        @(negedge aclk);
        prbs_tvalid = 1'b0;
        prbs_tlast  = 1'b0;

        wait (0);
    end

    initial begin
        #200_000;
        $display("WARN: safety timeout, out_count=%0d", out_count);
        $fclose(fd);
        $finish;
    end

endmodule
