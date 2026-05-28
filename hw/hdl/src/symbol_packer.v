`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : symbol_packer.v
// Description  : Accumulates 1-bit-per-clock stream into Qm-bit symbols.
//=============================================================================
// Additional Notes:
// - Qm set by qam_mode: 0=QPSK(2), 1=16QAM(4), 2=64QAM(6), 3=256QAM(8).
// - Bits are packed MSB-first: first bit in becomes the MSB of sym_bits.
//=============================================================================
//
// Output m_axis_tdata is emitted with m_axis_tvalid pulse, aligned to m_axis_tready pulse
// from downstream (which marks symbol boundaries at SPS=4 cadence).
//
// Backpressure: s_axis_tready is deasserted when the accumulator already has
// Qm bits and is waiting for m_axis_tready to release them. This is the standard
// AXI4-Stream valid/ready handshake. When FEC chain is inserted upstream,
// s_axis_tready connects to the new upstream block; semantics unchanged.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module symbol_packer #(
    parameter integer MAX_BPS = 8                   // max bits per symbol (256-QAM)
)(
    input  wire                  aclk,
    input  wire                  aresetn,
    input  wire  [2:0]           qam_mode,          // 0..3 -> QPSK..256-QAM
    input  wire                  sym_en,            // 1-clock pulse, symbol boundary strobe
    // upstream interface (from bit_source or, later, from FEC chain)
    input  wire                  s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output reg                   s_axis_tready,
    // downstream interface (to qam_mapper)
    output reg   [MAX_BPS-1:0]   m_axis_tdata,
    output reg   [3:0]           m_axis_tuser,
    output reg                   m_axis_tvalid,
    input  wire                  m_axis_tready      // downstream flow control ready
);

    // -------------------------------------------------------------------------
    // Decode mode -> bits per symbol
    // -------------------------------------------------------------------------
    reg [3:0] bps;
    always @(*) begin
        case (qam_mode)
            3'd0:    bps = 4'd2;     // QPSK
            3'd1:    bps = 4'd4;     // 16-QAM
            3'd2:    bps = 4'd6;     // 64-QAM
            3'd3:    bps = 4'd8;     // 256-QAM
            default: bps = 4'd4;     // safe default
        endcase
    end

    // -------------------------------------------------------------------------
    // Accumulator state
    //   acc       : holds bits as they arrive, MSB-first
    //   acc_count : how many bits currently held (0..bps)
    //   full      : 1 when acc_count == bps (ready to emit)
    // -------------------------------------------------------------------------
    reg [MAX_BPS-1:0] acc;
    reg [3:0]         acc_count;
    wire              full = (acc_count == bps);

    // -------------------------------------------------------------------------
    // s_axis_tready: accept a new bit unless we are full and waiting to emit
    // -------------------------------------------------------------------------
    wire out_fire = m_axis_tvalid && m_axis_tready;
    wire out_ready = !m_axis_tvalid || m_axis_tready;
    wire move_to_out = full && sym_en && out_ready;

    always @(*) begin
        s_axis_tready = !full || move_to_out;
    end

    // -------------------------------------------------------------------------
    // Main logic
    // -------------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            acc          <= {MAX_BPS{1'b0}};
            acc_count    <= 4'd0;
            m_axis_tdata <= {MAX_BPS{1'b0}};
            m_axis_tuser <= 4'd0;
            m_axis_tvalid<= 1'b0;
        end else begin
            // Output valid logic
            if (move_to_out) begin
                m_axis_tdata  <= acc;
                m_axis_tuser  <= bps;
                m_axis_tvalid <= 1'b1;
            end else if (out_fire) begin
                m_axis_tvalid <= 1'b0;
            end

            // Input accumulator logic
            if (s_axis_tvalid && s_axis_tready) begin
                if (move_to_out) begin
                    // acc is being transferred to out, so start a new acc
                    acc <= {MAX_BPS{1'b0}};
                    acc[bps - 1] <= s_axis_tdata;
                    acc_count <= 4'd1;
                end else begin
                    acc[bps - 1 - acc_count] <= s_axis_tdata;
                    acc_count <= acc_count + 4'd1;
                end
            end else if (move_to_out) begin
                acc <= {MAX_BPS{1'b0}};
                acc_count <= 4'd0;
            end
        end
    end

endmodule
