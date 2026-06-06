`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 20:04:56
// Design Name: 
// Module Name: tb_descrambler
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
// File         : tb_descrambler.v
// Description  : Loopback TB: bit_source -> scrambler -> descrambler.
//                Expects descrambler output == original PRBS-23 stream.
//                128 bits, c_init = 0x00008000.
//=============================================================================
`timescale 1ns / 1ps

module tb_descrambler;

    localparam integer NUM_INPUT_BITS = 520;
    localparam [30:0]  C_INIT         = 31'h00008000;

    reg  aclk;
    reg  aresetn;

    // Stage 1: PRBS bit generator (TB-local)
    reg  prbs_tdata;
    reg  prbs_tvalid;
    wire prbs_tready;
    reg  prbs_tlast;

    // Stage 2: scrambler outputs
    wire scr_tdata;
    wire scr_tvalid;
    wire scr_tready;
    wire scr_tlast;

    // Stage 3: descrambler outputs
    wire dsc_tdata;
    wire dsc_tvalid;
    reg  dsc_tready;
    wire dsc_tlast;

    // --------------------------------------------------------------
    // Scrambler instance
    // --------------------------------------------------------------
    scrambler #(.C_INIT(C_INIT)) u_scr (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(prbs_tdata),
        .s_axis_tvalid(prbs_tvalid),
        .s_axis_tready(prbs_tready),
        .s_axis_tlast(prbs_tlast),
        .m_axis_tdata(scr_tdata),
        .m_axis_tvalid(scr_tvalid),
        .m_axis_tready(scr_tready),
        .m_axis_tlast(scr_tlast)
    );

    // --------------------------------------------------------------
    // Descrambler instance (DUT)
    // --------------------------------------------------------------
    descrambler #(.C_INIT(C_INIT)) u_dsc (
        .aclk(aclk), .aresetn(aresetn),
        .s_axis_tdata(scr_tdata),
        .s_axis_tvalid(scr_tvalid),
        .s_axis_tready(scr_tready),
        .s_axis_tlast(scr_tlast),
        .m_axis_tdata(dsc_tdata),
        .m_axis_tvalid(dsc_tvalid),
        .m_axis_tready(dsc_tready),
        .m_axis_tlast(dsc_tlast)
    );

    // --------------------------------------------------------------
    // 100 MHz clock
    // --------------------------------------------------------------
    initial aclk = 0;
    always #5 aclk = ~aclk;

    // --------------------------------------------------------------
    // PRBS-23 driver (procedural, same pattern as prior blocks)
    // --------------------------------------------------------------
    reg [22:0] lfsr;

    function automatic [0:0] prbs_step_fn;
        input dummy;
        begin
            prbs_step_fn = lfsr[22];
            lfsr = {lfsr[21:0], lfsr[22] ^ lfsr[17]};
        end
    endfunction

    // --------------------------------------------------------------
    // Dump descrambler output
    // --------------------------------------------------------------
    integer fd;
    integer out_count;

    initial begin
        out_count = 0;
        fd = $fopen("descrambler_out.txt", "w");
    end

    always @(negedge aclk) begin
        if (aresetn && dsc_tvalid && dsc_tready) begin
            $fwrite(fd, "%0d\n", dsc_tdata);
            out_count = out_count + 1;
            if (out_count == NUM_INPUT_BITS) begin
                $fclose(fd);
                $display("DONE: %0d bits dumped", out_count);
                $finish;
            end
        end
    end

    // --------------------------------------------------------------
    // Main stimulus: wait for scrambler warmup, then drive PRBS bits
    // --------------------------------------------------------------
    integer i;
    initial begin
        aresetn     = 1'b0;
        prbs_tdata  = 1'b0;
        prbs_tvalid = 1'b0;
        prbs_tlast  = 1'b0;
        dsc_tready  = 1'b0;
        lfsr        = 23'h5A3C7E;

        #50;
        @(posedge aclk);
        aresetn = 1'b1;
        @(posedge aclk);
        dsc_tready = 1'b1;

        // Wait for scrambler warmup to complete
        wait (prbs_tready);
        $display("Scrambler warmup complete at time %0t ns", $time);

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

    // --------------------------------------------------------------
    // Safety timeout: scrambler warmup ~16us + descrambler warmup ~16us
    //                 + 128 bit transfer ~1.3us. Allow 2ms.
    // --------------------------------------------------------------
    initial begin
        #2_000_000;
        $display("WARN: safety timeout, out_count=%0d", out_count);
        $fclose(fd);
        $finish;
    end

endmodule
