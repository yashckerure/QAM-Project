`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : tb_ber_counter.v
// Description  : Full bit-pipeline wrapper: zero-BER loopback (no channel/filter).
//=============================================================================
// Additional Notes:
// - Chain: bit_source -> symbol_packer -> qam_mapper -> qam_slicer -> ber_counter
//=============================================================================
// Sweeps QPSK then 16-QAM, target 4096 bits compared per mode.
// Pass criterion: bit_errors == 0 for both modes.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_ber_counter #(
    parameter integer SPS              = 4,
    parameter integer MAX_BPS          = 8,
    parameter integer DATA_W           = 16,
    parameter integer NUM_BITS_TARGET  = 4096
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sym_en,
    input  wire  [2:0] qam_mode,
    input  wire        ber_enable,

    output wire [31:0] bit_errors,
    output wire [31:0] bits_compared,
    output wire        status_done
);

    wire                      bit_tdata;
    wire                      bit_tvalid;
    wire                      bit_tready;

    wire [MAX_BPS-1:0]        sym_tdata;
    wire [3:0]                sym_tuser;
    wire                      sym_tvalid;
    wire                      sym_tready;

    wire [2*DATA_W-1:0]       iq_tdata;
    wire                      iq_tvalid;
    wire                      iq_tready;

    wire [MAX_BPS-1:0]        sliced_tdata;
    wire [3:0]                sliced_tuser;
    wire                      sliced_tvalid;
    wire                      sliced_tready;

    bit_source #(.LFSR_W(23), .SEED(23'h5A3C7E)) u_src (
        .aclk(clk), .aresetn(rst_n), 
        .m_axis_tready(bit_tready),
        .m_axis_tdata(bit_tdata), 
        .m_axis_tvalid(bit_tvalid)
    );

    symbol_packer #(.MAX_BPS(MAX_BPS)) u_pack (
        .aclk(clk), .aresetn(rst_n), .qam_mode(qam_mode), .m_axis_tready(sym_en),
        .s_axis_tdata(bit_tdata), .s_axis_tvalid(bit_tvalid), .s_axis_tready(bit_tready),
        .m_axis_tdata(sym_tdata), .m_axis_tuser(sym_tuser), .m_axis_tvalid(sym_tvalid)
    );

    qam_mapper #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_map (
        .aclk(clk), .aresetn(rst_n), .qam_mode(qam_mode),
        .s_axis_tdata(sym_tdata), .s_axis_tuser(sym_tuser), .s_axis_tvalid(sym_tvalid), .s_axis_tready(sym_tready),
        .m_axis_tdata(iq_tdata), .m_axis_tvalid(iq_tvalid), .m_axis_tready(iq_tready)
    );

    qam_slicer #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_slice (
        .aclk(clk), .aresetn(rst_n), .qam_mode(qam_mode),
        .s_axis_tdata(iq_tdata), .s_axis_tvalid(iq_tvalid), .s_axis_tready(iq_tready),
        .m_axis_tdata(sliced_tdata), .m_axis_tuser(sliced_tuser),
        .m_axis_tvalid(sliced_tvalid), .m_axis_tready(sliced_tready)
    );

    ber_counter #(
        .LFSR_W          (23),
        .SEED            (23'h5A3C7E),
        .MAX_BPS         (MAX_BPS),
        .NUM_BITS_TARGET (NUM_BITS_TARGET)
    ) u_ber (
        .aclk            (clk),
        .aresetn         (rst_n),
        .enable          (ber_enable),
        .s_axis_tdata    (sliced_tdata),
        .s_axis_tuser    (sliced_tuser),
        .s_axis_tvalid   (sliced_tvalid),
        .s_axis_tready   (sliced_tready),
        .bit_errors      (bit_errors),
        .bits_compared   (bits_compared),
        .status_done     (status_done)
    );

endmodule