//=============================================================================
// Project      : Adaptive QAM Modem
// File         : qam_loopback_axi.v
// Description  : AXI-Lite wrapper for the Milestone 1 QAM loopback chain.
//=============================================================================
// Additional Notes:
// - Exposes control/status registers to the Zynq PS over AXI-Lite.
// - Instantiates bit_source -> symbol_packer -> qam_mapper -> 
//                qam_slicer -> ber_counter.
//=============================================================================
`timescale 1ns / 1ps

module qam_loopback_axi #(
    parameter integer MAX_BPS = 8,
    parameter integer DATA_W  = 16
)(
    // AXI-Lite Slave Interface
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    
    input  wire [4:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    
    input  wire [4:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready
);

    //-------------------------------------------------------------------------
    // AXI-Lite Slave Logic
    //-------------------------------------------------------------------------
    reg        axi_awready;
    reg        axi_wready;
    reg [1:0]  axi_bresp;
    reg        axi_bvalid;
    reg        axi_arready;
    reg [31:0] axi_rdata;
    reg [1:0]  axi_rresp;
    reg        axi_rvalid;

    assign s_axi_awready = axi_awready;
    assign s_axi_wready  = axi_wready;
    assign s_axi_bresp   = axi_bresp;
    assign s_axi_bvalid  = axi_bvalid;
    assign s_axi_arready = axi_arready;
    assign s_axi_rdata   = axi_rdata;
    assign s_axi_rresp   = axi_rresp;
    assign s_axi_rvalid  = axi_rvalid;

    // Registers
    reg [31:0] slv_reg0_ctrl;
    reg [31:0] slv_reg4_cfg_sps;
    reg [31:0] slv_reg5_target;

    // Status inputs from modem
    wire [31:0] stat_bits_compared;
    wire [31:0] stat_bit_errors;
    wire        stat_status_done;

    wire slv_reg_wren = axi_wready && s_axi_wvalid && axi_awready && s_axi_awvalid;
    wire slv_reg_rden = axi_arready && s_axi_arvalid && ~axi_rvalid;

    // Write Logic
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            axi_bvalid  <= 1'b0;
            axi_bresp   <= 2'b0;
            slv_reg0_ctrl    <= 32'd0;
            slv_reg4_cfg_sps <= 32'd4;
            slv_reg5_target  <= 32'd4096;
        end else begin
            // awready / wready handshaking
            if (~axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                axi_awready <= 1'b1;
                axi_wready  <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
                axi_wready  <= 1'b0;
            end

            // Register Write
            if (slv_reg_wren) begin
                case (s_axi_awaddr[4:2])
                    3'h0: begin // 0x00 CTRL
                        if (s_axi_wstrb[0]) slv_reg0_ctrl[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) slv_reg0_ctrl[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) slv_reg0_ctrl[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) slv_reg0_ctrl[31:24] <= s_axi_wdata[31:24];
                    end
                    3'h4: begin // 0x10 CFG_SPS
                        if (s_axi_wstrb[0]) slv_reg4_cfg_sps[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) slv_reg4_cfg_sps[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) slv_reg4_cfg_sps[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) slv_reg4_cfg_sps[31:24] <= s_axi_wdata[31:24];
                    end
                    3'h5: begin // 0x14 TARGET_BITS
                        if (s_axi_wstrb[0]) slv_reg5_target[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) slv_reg5_target[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) slv_reg5_target[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) slv_reg5_target[31:24] <= s_axi_wdata[31:24];
                    end
                endcase
            end

            // bvalid generation
            if (axi_awready && s_axi_awvalid && ~axi_bvalid && axi_wready && s_axi_wvalid) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0; // OKAY
            end else if (s_axi_bready && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    // Read Logic
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_arready <= 1'b0;
            axi_rvalid  <= 1'b0;
            axi_rresp   <= 2'b0;
            axi_rdata   <= 32'd0;
        end else begin
            // arready handshaking
            if (~axi_arready && s_axi_arvalid) begin
                axi_arready <= 1'b1;
            end else begin
                axi_arready <= 1'b0;
            end

            // rvalid and rdata generation
            if (slv_reg_rden) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b0; // OKAY
                case (s_axi_araddr[4:2])
                    3'h0: axi_rdata <= slv_reg0_ctrl;                   // 0x00 CTRL
                    3'h1: axi_rdata <= {31'd0, stat_status_done};       // 0x04 STATUS
                    3'h2: axi_rdata <= stat_bits_compared;              // 0x08 BIT_COUNT
                    3'h3: axi_rdata <= stat_bit_errors;                 // 0x0C ERR_COUNT
                    3'h4: axi_rdata <= slv_reg4_cfg_sps;                // 0x10 CFG_SPS
                    3'h5: axi_rdata <= slv_reg5_target;                 // 0x14 TARGET_BITS
                    3'h6: axi_rdata <= 32'h00010000;                    // 0x18 VERSION (v1.0.0)
                    default: axi_rdata <= 32'd0;
                endcase
            end else if (axi_rvalid && s_axi_rready) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // Modem Chain Instantiation
    //-------------------------------------------------------------------------
    wire [2:0]  qam_mode    = slv_reg0_ctrl[2:0];
    wire        ber_enable  = slv_reg0_ctrl[4];
    wire        sw_reset    = slv_reg0_ctrl[5];
    
    // Core reset is asserted if AXI reset is low OR software reset is high
    wire core_resetn = s_axi_aresetn & ~sw_reset;
    wire [31:0] cfg_sps = slv_reg4_cfg_sps;
    
    // Symbol Strobe Generator
    reg [31:0] sps_counter;
    reg        sym_en;
    
    always @(posedge s_axi_aclk or negedge core_resetn) begin
        if (!core_resetn) begin
            sps_counter <= 32'd0;
            sym_en      <= 1'b0;
        end else begin
            if (sps_counter >= (cfg_sps - 1)) begin
                sps_counter <= 32'd0;
                sym_en      <= 1'b1;
            end else begin
                sps_counter <= sps_counter + 32'd1;
                sym_en      <= 1'b0;
            end
        end
    end

    // AXI-Stream wiring
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
        .aclk(s_axi_aclk), .aresetn(core_resetn), 
        .m_axis_tready(bit_tready),
        .m_axis_tdata(bit_tdata), 
        .m_axis_tvalid(bit_tvalid)
    );

    symbol_packer #(.MAX_BPS(MAX_BPS)) u_pack (
        .aclk(s_axi_aclk), .aresetn(core_resetn), .qam_mode(qam_mode), .sym_en(sym_en),
        .s_axis_tdata(bit_tdata), .s_axis_tvalid(bit_tvalid), .s_axis_tready(bit_tready),
        .m_axis_tdata(sym_tdata), .m_axis_tuser(sym_tuser), .m_axis_tvalid(sym_tvalid), .m_axis_tready(sym_tready)
    );

    qam_mapper #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_map (
        .aclk(s_axi_aclk), .aresetn(core_resetn), .qam_mode(qam_mode),
        .s_axis_tdata(sym_tdata), .s_axis_tuser(sym_tuser), .s_axis_tvalid(sym_tvalid), .s_axis_tready(sym_tready),
        .m_axis_tdata(iq_tdata), .m_axis_tvalid(iq_tvalid), .m_axis_tready(iq_tready)
    );

    qam_slicer #(.DATA_W(DATA_W), .FRAC_W(10), .MAX_BPS(MAX_BPS)) u_slice (
        .aclk(s_axi_aclk), .aresetn(core_resetn), .qam_mode(qam_mode),
        .s_axis_tdata(iq_tdata), .s_axis_tvalid(iq_tvalid), .s_axis_tready(iq_tready),
        .m_axis_tdata(sliced_tdata), .m_axis_tuser(sliced_tuser),
        .m_axis_tvalid(sliced_tvalid), .m_axis_tready(sliced_tready)
    );

    ber_counter #(
        .LFSR_W          (23),
        .SEED            (23'h5A3C7E),
        .MAX_BPS         (MAX_BPS)
    ) u_ber (
        .aclk            (s_axi_aclk),
        .aresetn         (core_resetn),
        .enable          (ber_enable),
        .num_bits_target (slv_reg5_target),
        .s_axis_tdata    (sliced_tdata),
        .s_axis_tuser    (sliced_tuser),
        .s_axis_tvalid   (sliced_tvalid),
        .s_axis_tready   (sliced_tready),
        .bit_errors      (stat_bit_errors),
        .bits_compared   (stat_bits_compared),
        .status_done     (stat_status_done)
    );

endmodule
