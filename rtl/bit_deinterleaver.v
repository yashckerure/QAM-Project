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
//                LLR-WIDTH version for the soft RX path: carries LLR_W-bit
//                signed LLRs instead of single bits. Matrix has Qm rows and
//                N=E/Qm columns; write column-by-column, read row-by-row.
//=============================================================================
// Additional Notes:
// - Identical permutation/FSM to the verified 1-bit deinterleaver; only the
//   datapath element width changed (1 bit -> LLR_W-bit signed). Set LLR_W=4 to
//   match the LDPC decoder's sfix4 LLR format.
// - Per-packet inputs latched on first beat: qm_in (2/4/6/8), n_in (= E/Qm).
// - Input element k written at addr (k mod Qm)*N + (k div Qm); read 0..E-1.
// - E_MAX = 4096 elements buffer. Registered AXI-Stream outputs.
//=============================================================================
`timescale 1ns / 1ps

module bit_deinterleaver #(
    parameter integer LLR_W  = 4,
    parameter integer E_MAX  = 4096,
    parameter integer ADDR_W = 13
)(
    input  wire                    aclk,
    input  wire                    aresetn,

    input  wire [3:0]              qm_in,
    input  wire [ADDR_W-1:0]       n_in,

    input  wire signed [LLR_W-1:0] s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output reg                     s_axis_tready,
    input  wire                    s_axis_tlast,

    output reg  signed [LLR_W-1:0] m_axis_tdata,
    output reg                     m_axis_tvalid,
    input  wire                    m_axis_tready,
    output reg                     m_axis_tlast
);

    reg signed [LLR_W-1:0] buf_mem [0:E_MAX-1];

    reg [3:0]        qm;
    reg [ADDR_W-1:0] n_total;

    reg [3:0]        wr_row;       // 0..Qm-1
    reg [ADDR_W-1:0] wr_col;       // 0..N-1 (= k div Qm)

    wire [ADDR_W-1:0] wr_addr = ({{(ADDR_W-4){1'b0}}, wr_row}) * n_total + wr_col;

    reg [ADDR_W-1:0] rd_count;
    reg [ADDR_W-1:0] e_total;      // = qm * n_total

    localparam S_COLLECT = 1'b0;
    localparam S_STREAM  = 1'b1;
    reg state;

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    always @(*) begin
        s_axis_tready = (state == S_COLLECT);
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= S_COLLECT;
            qm            <= 4'd2;
            n_total       <= {ADDR_W{1'b0}};
            e_total       <= {ADDR_W{1'b0}};
            wr_row        <= 4'd0;
            wr_col        <= {ADDR_W{1'b0}};
            rd_count      <= {ADDR_W{1'b0}};
            m_axis_tdata  <= {LLR_W{1'b0}};
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            case (state)
                S_COLLECT: begin
                    if (input_fire) begin
                        buf_mem[wr_addr] <= s_axis_tdata;

                        if ((wr_row == 4'd0) && (wr_col == {ADDR_W{1'b0}})) begin
                            qm      <= qm_in;
                            n_total <= n_in;
                            e_total <= ({{(ADDR_W-4){1'b0}}, qm_in}) * n_in;
                        end

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
