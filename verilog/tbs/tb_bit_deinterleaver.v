`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 19:51:08
// Design Name: 
// Module Name: tb_bit_deinterleaver
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
// File         : tb_bit_deinterleaver.v
// Description  : Standalone TB for bit_deinterleaver.
//                Drives the interleaved version of 128 PRBS-23 bits
//                (16-QAM, Qm=4, E=128, N=32) into the DUT.
//                Expects output == original PRBS-23 stream.
//=============================================================================
`timescale 1ns / 1ps

module tb_bit_deinterleaver;

    localparam integer NUM_INPUT_BITS = 1040;
    localparam [3:0]   QM_TEST        = 4'd4;
    localparam integer N_TEST         = 260;

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

    bit_deinterleaver #(.E_MAX(4096), .ADDR_W(13)) dut (
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

    // ------------------------------------------------------------
    // Pre-compute the test data: PRBS-23 stream, then interleave
    // ------------------------------------------------------------
    reg [22:0] lfsr;
    reg        prbs_bits      [0:NUM_INPUT_BITS-1];
    reg        interleaved    [0:NUM_INPUT_BITS-1];

    integer    k, r, c, idx;

    initial begin
        // 1. Generate 128 PRBS-23 bits
        lfsr = 23'h5A3C7E;
        for (k = 0; k < NUM_INPUT_BITS; k = k + 1) begin
            prbs_bits[k] = lfsr[22];
            lfsr = {lfsr[21:0], lfsr[22] ^ lfsr[17]};
        end

        // 2. Interleave them: write row-by-row, read col-by-col
        //    matrix is QM_TEST rows by N_TEST cols
        //    Output index = c*Qm + r  for matrix[r][c] = input[r*N + c]
        idx = 0;
        for (c = 0; c < N_TEST; c = c + 1) begin
            for (r = 0; r < QM_TEST; r = r + 1) begin
                interleaved[idx] = prbs_bits[r * N_TEST + c];
                idx = idx + 1;
            end
        end
    end

    // ------------------------------------------------------------
    // Dump output bits
    // ------------------------------------------------------------
    integer fd;
    integer out_count;

    initial begin
        out_count = 0;
        fd = $fopen("deinterleaver_out.txt", "w");
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

    // ------------------------------------------------------------
    // Drive the interleaved sequence into the DUT
    // ------------------------------------------------------------
    integer i;
    initial begin
        aresetn      = 1'b0;
        qm_in        = 4'd0;
        n_in         = 13'd0;
        s_in_tdata   = 1'b0;
        s_in_tvalid  = 1'b0;
        s_in_tlast   = 1'b0;
        m_out_tready = 1'b0;

        #50;
        @(posedge aclk);
        aresetn = 1'b1;
        @(posedge aclk);

        qm_in = QM_TEST;
        n_in  = N_TEST;
        m_out_tready = 1'b1;

        for (i = 0; i < NUM_INPUT_BITS; i = i + 1) begin
            @(negedge aclk);
            s_in_tdata  = interleaved[i];
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