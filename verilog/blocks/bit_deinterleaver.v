/////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 19:49:53
// Design Name: 
// Module Name: bit_deinterleaver
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
// File         : bit_deinterleaver.v
// Description  : Inverse of bit_interleaver (TS 38.212 Section 5.4.2.2).
//                Matrix has Qm rows and N=E/Qm columns.
//                Write column-by-column, read row-by-row.
//=============================================================================
// Additional Notes:
// - Per-packet inputs (latched on first beat):
//     qm_in : modulation order (2, 4, 6, 8)
//     n_in  : N = E/Qm
// - Phase 1 COLLECT: write input bits into matrix column-by-column.
//   First Qm bits fill column 0 (rows 0..Qm-1), next Qm bits fill column 1, etc.
//   Equivalent linear address for input bit k: (k mod Qm) * N + (k div Qm).
//   Maintained with two counters (wr_row, wr_col), running offset for k div Qm.
// - Phase 2 STREAM: read out row-by-row at addresses 0, 1, 2, ..., E-1.
// - Registered AXI-Stream outputs.
// - E_MAX = 1024 bits buffer.
//=============================================================================
`timescale 1ns / 1ps

module bit_deinterleaver #(
    parameter integer E_MAX  = 4096,
    parameter integer ADDR_W = 13
)(
    input  wire                  aclk,
    input  wire                  aresetn,

    input  wire [3:0]            qm_in,
    input  wire [ADDR_W-1:0]     n_in,

    input  wire                  s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output reg                   s_axis_tready,
    input  wire                  s_axis_tlast,

    output reg                   m_axis_tdata,
    output reg                   m_axis_tvalid,
    input  wire                  m_axis_tready,
    output reg                   m_axis_tlast
);

    // ------------------------------------------------------------
    // Buffer
    // ------------------------------------------------------------
    reg buf_mem [0:E_MAX-1];

    // Latched per-packet parameters
    reg [3:0]        qm;
    reg [ADDR_W-1:0] n_total;

    // Write counters: column-by-column filling
    reg [3:0]        wr_row;       // 0..Qm-1
    reg [ADDR_W-1:0] wr_col;       // 0..N-1, also = k div Qm

    wire [ADDR_W-1:0] wr_addr = ({{(ADDR_W-4){1'b0}}, wr_row}) * n_total + wr_col;
    //  Hardware: small multiply (4-bit by ADDR_W-bit). Vivado infers a small DSP.

    // Read counters: sequential 0..E-1
    reg [ADDR_W-1:0] rd_count;
    reg [ADDR_W-1:0] e_total;      // = qm * n_total (computed once)

    // FSM
    localparam S_COLLECT = 1'b0;
    localparam S_STREAM  = 1'b1;
    reg state;

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // ------------------------------------------------------------
    // Handshake
    // ------------------------------------------------------------
    always @(*) begin
        s_axis_tready = (state == S_COLLECT);
    end

    // ------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= S_COLLECT;
            qm            <= 4'd2;
            n_total       <= {ADDR_W{1'b0}};
            e_total       <= {ADDR_W{1'b0}};
            wr_row        <= 4'd0;
            wr_col        <= {ADDR_W{1'b0}};
            rd_count      <= {ADDR_W{1'b0}};
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            // Clear valid when downstream consumes
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            case (state)
                // -------------------------------------------------
                // COLLECT: write input bits at permuted addresses
                // -------------------------------------------------
                S_COLLECT: begin
                    if (input_fire) begin
                        buf_mem[wr_addr] <= s_axis_tdata;

                        // Latch parameters on first beat
                        if ((wr_row == 4'd0) && (wr_col == {ADDR_W{1'b0}})) begin
                            qm      <= qm_in;
                            n_total <= n_in;
                            // e_total = qm_in * n_in. Small mult.
                            e_total <= ({{(ADDR_W-4){1'b0}}, qm_in}) * n_in;
                        end

                        // Advance (wr_row, wr_col): row first (column-major fill)
                        if (wr_row == (qm - 4'd1)) begin
                            wr_row <= 4'd0;
                            wr_col <= wr_col + {{(ADDR_W-1){1'b0}}, 1'b1};
                        end else begin
                            wr_row <= wr_row + 4'd1;
                        end

                        if (s_axis_tlast) begin
                            state    <= S_STREAM;
                            wr_row   <= 4'd0;
                            wr_col   <= {ADDR_W{1'b0}};
                            rd_count <= {ADDR_W{1'b0}};
                        end
                    end
                end

                // -------------------------------------------------
                // STREAM: sequential read
                // -------------------------------------------------
                S_STREAM: begin
                    if (!m_axis_tvalid || m_axis_tready) begin
                        m_axis_tdata  <= buf_mem[rd_count];
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (rd_count == (e_total -
                                          {{(ADDR_W-1){1'b0}}, 1'b1}));

                        if (rd_count == (e_total -
                                         {{(ADDR_W-1){1'b0}}, 1'b1})) begin
                            state    <= S_COLLECT;
                            rd_count <= {ADDR_W{1'b0}};
                        end else begin
                            rd_count <= rd_count +
                                        {{(ADDR_W-1){1'b0}}, 1'b1};
                        end
                    end
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule
