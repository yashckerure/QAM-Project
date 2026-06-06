`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 19:43:03
// Design Name: 
// Module Name: bit_interleaver
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
// File         : bit_interleaver.v
// Description  : PDSCH bit interleaver per TS 38.212 Section 5.4.2.2.
//                Matrix interleaver, Qm rows by N=E/Qm columns.
//=============================================================================
// Additional Notes:
// - Per-packet inputs (latched on first beat after reset or after prior packet
//   completes):
//     qm_in : modulation order (2, 4, 6, 8)
//     n_in  : N = E/Qm (host-provided; avoids on-chip division)
// - Phase 1 COLLECT: bits stored sequentially in buffer 0..E-1.
//   s_axis_tready asserted; m_axis idle.
// - Phase 2 STREAM: bits emitted in column-major order.
//   Output bit at index m = buf[(m mod Qm) * N + (m div Qm)].
//   Implemented with incremental (rd_row, rd_col, rd_offset) counters.
// - Registered AXI-Stream outputs.
// - E_MAX = 1024 bits buffer.
//=============================================================================
`timescale 1ns / 1ps

module bit_interleaver #(
    parameter integer E_MAX  = 4096,
    parameter integer ADDR_W = 13        // log2(E_MAX) + 1 to hold counts up to E_MAX
)(
    input  wire                  aclk,
    input  wire                  aresetn,

    input  wire [3:0]            qm_in,    // 2, 4, 6, or 8
    input  wire [ADDR_W-1:0]     n_in,     // N = E/Qm

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
    // Buffer (1 bit per address). Inferred as LUTRAM/BRAM by Vivado.
    // ------------------------------------------------------------
    reg buf_mem [0:E_MAX-1];

    // Latched per-packet parameters
    reg [3:0]        qm;
    reg [ADDR_W-1:0] n_total;

    // Write/read counters
    reg [ADDR_W-1:0] wr_count;
    reg [3:0]        rd_row;
    reg [ADDR_W-1:0] rd_col;
    reg [ADDR_W-1:0] rd_offset;     // rd_row * n_total, maintained incrementally

    wire [ADDR_W-1:0] rd_addr = rd_offset + rd_col;

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
            wr_count      <= {ADDR_W{1'b0}};
            qm            <= 4'd2;
            n_total       <= {ADDR_W{1'b0}};
            rd_row        <= 4'd0;
            rd_col        <= {ADDR_W{1'b0}};
            rd_offset     <= {ADDR_W{1'b0}};
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
                // COLLECT: write input bits to buffer
                // -------------------------------------------------
                S_COLLECT: begin
                    if (input_fire) begin
                        buf_mem[wr_count] <= s_axis_tdata;

                        // Latch parameters on first beat
                        if (wr_count == {ADDR_W{1'b0}}) begin
                            qm      <= qm_in;
                            n_total <= n_in;
                        end

                        wr_count <= wr_count + {{(ADDR_W-1){1'b0}}, 1'b1};

                        if (s_axis_tlast) begin
                            state     <= S_STREAM;
                            wr_count  <= {ADDR_W{1'b0}};
                            rd_row    <= 4'd0;
                            rd_col    <= {ADDR_W{1'b0}};
                            rd_offset <= {ADDR_W{1'b0}};
                        end
                    end
                end

                // -------------------------------------------------
                // STREAM: read out in column-major order
                // -------------------------------------------------
                S_STREAM: begin
                    if (!m_axis_tvalid || m_axis_tready) begin
                        // Emit bit at rd_addr
                        m_axis_tdata  <= buf_mem[rd_addr];
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (rd_row == (qm - 4'd1)) &&
                                         (rd_col == (n_total - {{(ADDR_W-1){1'b0}}, 1'b1}));

                        // Advance counters
                        if (rd_row == (qm - 4'd1)) begin
                            rd_row    <= 4'd0;
                            rd_offset <= {ADDR_W{1'b0}};
                            if (rd_col == (n_total - {{(ADDR_W-1){1'b0}}, 1'b1})) begin
                                state  <= S_COLLECT;
                                rd_col <= {ADDR_W{1'b0}};
                            end else begin
                                rd_col <= rd_col + {{(ADDR_W-1){1'b0}}, 1'b1};
                            end
                        end else begin
                            rd_row    <= rd_row + 4'd1;
                            rd_offset <= rd_offset + n_total;
                        end
                    end
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule
