//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 19:43:41
// Design Name: 
// Module Name: tb_bit_interleaver
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
// File         : tb_bit_interleaver.v
// Description  : Standalone TB for bit_interleaver.
//                Test case: Qm=4 (16-QAM), E=128, PRBS-23 seed 0x5A3C7E.
//                N = E/Qm = 32.
//=============================================================================
`timescale 1ns / 1ps

module tb_bit_interleaver;

    localparam integer NUM_INPUT_BITS = 1040;
    localparam [3:0]   QM_TEST        = 4'd4;     // 16-QAM
    localparam integer N_TEST         = 260;       // 128 / 4

    reg         aclk;
    reg         aresetn;

    reg  [3:0]  qm_in;
    reg  [12:0] n_in;

    reg         s_in_tdata;
    reg         s_in_tvalid;
    wire        s_in_tready;
    reg         s_in_tlast;

    wire        m_out_tdata;
    wire        m_out_tvalid;
    reg         m_out_tready;
    wire        m_out_tlast;

    bit_interleaver #(.E_MAX(4096), .ADDR_W(13)) dut (
        .aclk(aclk), .aresetn(aresetn),
        .qm_in(qm_in),
        .n_in(n_in),
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

    // PRBS-23 state
    reg [22:0] lfsr;

    function automatic [0:0] prbs_step_fn;
        input dummy;
        begin
            prbs_step_fn = lfsr[22];
            lfsr = {lfsr[21:0], lfsr[22] ^ lfsr[17]};
        end
    endfunction

    integer fd;
    integer out_count;

    initial begin
        out_count = 0;
        fd = $fopen("interleaver_out.txt", "w");
    end

    always @(negedge aclk) begin
        if (aresetn && m_out_tvalid && m_out_tready) begin
            $fwrite(fd, "%0d\n", m_out_tdata);
            out_count = out_count + 1;
            if (out_count == NUM_INPUT_BITS) begin
                $fclose(fd);
                $display("DONE: %0d bits dumped", out_count);
                $finish;
            end
        end
    end

    integer i;
    initial begin
        aresetn      = 1'b0;
        qm_in        = 4'd0;
        n_in         = 11'd0;
        s_in_tdata   = 1'b0;
        s_in_tvalid  = 1'b0;
        s_in_tlast   = 1'b0;
        m_out_tready = 1'b0;
        lfsr         = 23'h5A3C7E;

        #50;
        @(posedge aclk);
        aresetn = 1'b1;
        @(posedge aclk);

        // Set per-packet parameters
        qm_in = QM_TEST;
        n_in  = N_TEST;

        // Allow consumer
        m_out_tready = 1'b1;

        // Drive 128 PRBS bits with tlast on last
        for (i = 0; i < NUM_INPUT_BITS; i = i + 1) begin
            @(negedge aclk);
            s_in_tdata  = prbs_step_fn(1'b0);
            s_in_tvalid = 1'b1;
            s_in_tlast  = (i == NUM_INPUT_BITS - 1);
            @(posedge aclk);
            while (!s_in_tready) @(posedge aclk);
        end

        @(negedge aclk);
        s_in_tvalid = 1'b0;
        s_in_tlast  = 1'b0;

        wait (0);
    end

    initial begin
        #1_000_000;
        $display("WARN: safety timeout, out_count=%0d", out_count);
        $fclose(fd);
        $finish;
    end

endmodule