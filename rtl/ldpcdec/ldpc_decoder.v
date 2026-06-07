`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : ldpc_decoder.v
// Description  : AXI-Stream wrapper around the HDL Coder generated NR LDPC
//                Decoder core (BG2, Zc=52, N=2600 LLRs -> K=520 info bits).
//=============================================================================
// Additional Notes:
// - Mirror of ldpc_encoder: gearboxes the project's serial AXI-Stream to the
//   core's 52-wide parallel frame interface, then serializes the result back.
// - SLAVE side carries 4-bit signed LLRs (s_axis_tdata = signed [3:0]),
//   matching the core's sfix4 dataIn. MASTER side is 1-bit hard decoded bits.
// - Core config tied to constants: bgn=1 (=> base graph 2), liftingSize=52.
// - Core reset async active-low -> .reset connects directly to aresetn.
//   clk_enable tied high; ce_out follows clk_enable.
// - Flow: COLLECT 2600 LLRs -> wait nextFrame -> FEED 50 cols (52 LLRs/col)
//   -> CAPTURE 10 cols (52 bits/col) -> DRAIN 520 bits. One code block per pass.
// - Bit/LLR order matches reshape(.,Zc,.) column-major, same convention proven
//   against nrLDPCEncode/nrLDPCDecode: codeword LLR m -> col m/52, lane m%52;
//   output info bit k*52+i <- dataOut lane i, output col k.
//=============================================================================

module ldpc_decoder (
    input  wire               aclk,
    input  wire               aresetn,

    input  wire signed [3:0]  s_axis_tdata,   // 4-bit signed LLR
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,
    input  wire               s_axis_tlast,

    output reg                m_axis_tdata,   // 1-bit decoded info bit
    output reg                m_axis_tvalid,
    input  wire               m_axis_tready,
    output reg                m_axis_tlast
);

    // -----------------------------------------------------------------------
    // Frame geometry (BG2, Zc=52)
    // -----------------------------------------------------------------------
    localparam integer ZC         = 52;
    localparam integer LLR_W      = 4;
    localparam integer N_LLR      = 2600;   // coded LLRs in  (= 50 * Zc)
    localparam integer K_BITS     = 520;    // info bits out  (= 10 * Zc)
    localparam integer IN_CYCLES  = 50;     // N_LLR / Zc
    localparam integer OUT_CYCLES = 10;     // K_BITS / Zc
    localparam integer SLICE_W    = ZC*LLR_W; // 208 bits per feed column

    // -----------------------------------------------------------------------
    // FSM
    // -----------------------------------------------------------------------
    localparam [2:0] S_COLLECT = 3'd0,
                     S_WAIT    = 3'd1,
                     S_FEED    = 3'd2,
                     S_CAPTURE = 3'd3,
                     S_DRAIN   = 3'd4;

    reg [2:0] state;

    // Buffers (no reset: written before read)
    reg [N_LLR*LLR_W-1:0] inbuf;     // 2600 * 4 = 10400 bits
    reg [K_BITS-1:0]      outbuf;    // 520 bits

    reg [11:0] collect_count;   // 0..2599
    reg [5:0]  feed_col;        // 0..49
    reg [3:0]  cap_col;         // 0..9
    reg [9:0]  drain_count;     // 0..519

    // -----------------------------------------------------------------------
    // Core interface wires
    // -----------------------------------------------------------------------
    wire [255:0] core_din_bus;     // 64 lanes * 4 bit
    wire [63:0]  core_dout_bus;    // 64 lanes * 1 bit
    wire         core_ce_out;
    wire         core_ctrlOut_start;
    wire         core_ctrlOut_end;
    wire         core_ctrlOut_valid;
    wire         core_nextFrame;

    wire feeding = (state == S_FEED);

    // One 52-LLR column per feed cycle on lanes 0..51 (208 bits); lanes 52..63 = 0
    wire [13:0]      feed_base  = feed_col * SLICE_W;     // 0..10192
    wire [SLICE_W-1:0] feed_slice = feeding ? inbuf[feed_base +: SLICE_W] : {SLICE_W{1'b0}};
    assign core_din_bus = {{(256-SLICE_W){1'b0}}, feed_slice};

    wire core_ctrlIn_valid = feeding;
    wire core_ctrlIn_start = feeding && (feed_col == 6'd0);
    wire core_ctrlIn_end   = feeding && (feed_col == IN_CYCLES-1);

    wire out_sample = core_ce_out && core_ctrlOut_valid;

    assign s_axis_tready = (state == S_COLLECT);

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // -----------------------------------------------------------------------
    // Generated NR LDPC Decoder core
    // -----------------------------------------------------------------------
    dec_HDL_Algorithm u_core (
        .clk(aclk),
        .reset(aresetn),          // active-low async, per generation settings
        .clk_enable(1'b1),
        .dataIn_0(core_din_bus[3:0]),
        .dataIn_1(core_din_bus[7:4]),
        .dataIn_2(core_din_bus[11:8]),
        .dataIn_3(core_din_bus[15:12]),
        .dataIn_4(core_din_bus[19:16]),
        .dataIn_5(core_din_bus[23:20]),
        .dataIn_6(core_din_bus[27:24]),
        .dataIn_7(core_din_bus[31:28]),
        .dataIn_8(core_din_bus[35:32]),
        .dataIn_9(core_din_bus[39:36]),
        .dataIn_10(core_din_bus[43:40]),
        .dataIn_11(core_din_bus[47:44]),
        .dataIn_12(core_din_bus[51:48]),
        .dataIn_13(core_din_bus[55:52]),
        .dataIn_14(core_din_bus[59:56]),
        .dataIn_15(core_din_bus[63:60]),
        .dataIn_16(core_din_bus[67:64]),
        .dataIn_17(core_din_bus[71:68]),
        .dataIn_18(core_din_bus[75:72]),
        .dataIn_19(core_din_bus[79:76]),
        .dataIn_20(core_din_bus[83:80]),
        .dataIn_21(core_din_bus[87:84]),
        .dataIn_22(core_din_bus[91:88]),
        .dataIn_23(core_din_bus[95:92]),
        .dataIn_24(core_din_bus[99:96]),
        .dataIn_25(core_din_bus[103:100]),
        .dataIn_26(core_din_bus[107:104]),
        .dataIn_27(core_din_bus[111:108]),
        .dataIn_28(core_din_bus[115:112]),
        .dataIn_29(core_din_bus[119:116]),
        .dataIn_30(core_din_bus[123:120]),
        .dataIn_31(core_din_bus[127:124]),
        .dataIn_32(core_din_bus[131:128]),
        .dataIn_33(core_din_bus[135:132]),
        .dataIn_34(core_din_bus[139:136]),
        .dataIn_35(core_din_bus[143:140]),
        .dataIn_36(core_din_bus[147:144]),
        .dataIn_37(core_din_bus[151:148]),
        .dataIn_38(core_din_bus[155:152]),
        .dataIn_39(core_din_bus[159:156]),
        .dataIn_40(core_din_bus[163:160]),
        .dataIn_41(core_din_bus[167:164]),
        .dataIn_42(core_din_bus[171:168]),
        .dataIn_43(core_din_bus[175:172]),
        .dataIn_44(core_din_bus[179:176]),
        .dataIn_45(core_din_bus[183:180]),
        .dataIn_46(core_din_bus[187:184]),
        .dataIn_47(core_din_bus[191:188]),
        .dataIn_48(core_din_bus[195:192]),
        .dataIn_49(core_din_bus[199:196]),
        .dataIn_50(core_din_bus[203:200]),
        .dataIn_51(core_din_bus[207:204]),
        .dataIn_52(core_din_bus[211:208]),
        .dataIn_53(core_din_bus[215:212]),
        .dataIn_54(core_din_bus[219:216]),
        .dataIn_55(core_din_bus[223:220]),
        .dataIn_56(core_din_bus[227:224]),
        .dataIn_57(core_din_bus[231:228]),
        .dataIn_58(core_din_bus[235:232]),
        .dataIn_59(core_din_bus[239:236]),
        .dataIn_60(core_din_bus[243:240]),
        .dataIn_61(core_din_bus[247:244]),
        .dataIn_62(core_din_bus[251:248]),
        .dataIn_63(core_din_bus[255:252]),
        .ctrlIn_start(core_ctrlIn_start),
        .ctrlIn_end(core_ctrlIn_end),
        .ctrlIn_valid(core_ctrlIn_valid),
        .bgn(1'b1),               // base graph 2
        .liftingSizeIn(16'd52),   // Zc = 52
        .ce_out(core_ce_out),
        .dataOut_0(core_dout_bus[0]),
        .dataOut_1(core_dout_bus[1]),
        .dataOut_2(core_dout_bus[2]),
        .dataOut_3(core_dout_bus[3]),
        .dataOut_4(core_dout_bus[4]),
        .dataOut_5(core_dout_bus[5]),
        .dataOut_6(core_dout_bus[6]),
        .dataOut_7(core_dout_bus[7]),
        .dataOut_8(core_dout_bus[8]),
        .dataOut_9(core_dout_bus[9]),
        .dataOut_10(core_dout_bus[10]),
        .dataOut_11(core_dout_bus[11]),
        .dataOut_12(core_dout_bus[12]),
        .dataOut_13(core_dout_bus[13]),
        .dataOut_14(core_dout_bus[14]),
        .dataOut_15(core_dout_bus[15]),
        .dataOut_16(core_dout_bus[16]),
        .dataOut_17(core_dout_bus[17]),
        .dataOut_18(core_dout_bus[18]),
        .dataOut_19(core_dout_bus[19]),
        .dataOut_20(core_dout_bus[20]),
        .dataOut_21(core_dout_bus[21]),
        .dataOut_22(core_dout_bus[22]),
        .dataOut_23(core_dout_bus[23]),
        .dataOut_24(core_dout_bus[24]),
        .dataOut_25(core_dout_bus[25]),
        .dataOut_26(core_dout_bus[26]),
        .dataOut_27(core_dout_bus[27]),
        .dataOut_28(core_dout_bus[28]),
        .dataOut_29(core_dout_bus[29]),
        .dataOut_30(core_dout_bus[30]),
        .dataOut_31(core_dout_bus[31]),
        .dataOut_32(core_dout_bus[32]),
        .dataOut_33(core_dout_bus[33]),
        .dataOut_34(core_dout_bus[34]),
        .dataOut_35(core_dout_bus[35]),
        .dataOut_36(core_dout_bus[36]),
        .dataOut_37(core_dout_bus[37]),
        .dataOut_38(core_dout_bus[38]),
        .dataOut_39(core_dout_bus[39]),
        .dataOut_40(core_dout_bus[40]),
        .dataOut_41(core_dout_bus[41]),
        .dataOut_42(core_dout_bus[42]),
        .dataOut_43(core_dout_bus[43]),
        .dataOut_44(core_dout_bus[44]),
        .dataOut_45(core_dout_bus[45]),
        .dataOut_46(core_dout_bus[46]),
        .dataOut_47(core_dout_bus[47]),
        .dataOut_48(core_dout_bus[48]),
        .dataOut_49(core_dout_bus[49]),
        .dataOut_50(core_dout_bus[50]),
        .dataOut_51(core_dout_bus[51]),
        .dataOut_52(core_dout_bus[52]),
        .dataOut_53(core_dout_bus[53]),
        .dataOut_54(core_dout_bus[54]),
        .dataOut_55(core_dout_bus[55]),
        .dataOut_56(core_dout_bus[56]),
        .dataOut_57(core_dout_bus[57]),
        .dataOut_58(core_dout_bus[58]),
        .dataOut_59(core_dout_bus[59]),
        .dataOut_60(core_dout_bus[60]),
        .dataOut_61(core_dout_bus[61]),
        .dataOut_62(core_dout_bus[62]),
        .dataOut_63(core_dout_bus[63]),
        .ctrlOut_start(core_ctrlOut_start),
        .ctrlOut_end(core_ctrlOut_end),
        .ctrlOut_valid(core_ctrlOut_valid),
        .liftingSizeOut(),        // unused
        .nextFrame(core_nextFrame)
    );

    // -----------------------------------------------------------------------
    // Sequential control
    // -----------------------------------------------------------------------
    wire [9:0] cap_base = cap_col * ZC;   // 0..468

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= S_COLLECT;
            collect_count <= 12'd0;
            feed_col      <= 6'd0;
            cap_col       <= 4'd0;
            drain_count   <= 10'd0;
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            case (state)
                // Collect 2600 serial LLRs into inbuf
                S_COLLECT: begin
                    if (input_fire) begin
                        inbuf[collect_count*LLR_W +: LLR_W] <= s_axis_tdata;
                        if (collect_count == N_LLR-1) begin
                            collect_count <= 12'd0;
                            state         <= S_WAIT;
                        end else begin
                            collect_count <= collect_count + 12'd1;
                        end
                    end
                end

                // Wait until the core is ready for a new frame
                S_WAIT: begin
                    if (core_nextFrame) begin
                        feed_col <= 6'd0;
                        state    <= S_FEED;
                    end
                end

                // Feed 50 columns of 52 LLRs (ctrlIn_* driven combinationally)
                S_FEED: begin
                    if (feed_col == IN_CYCLES-1) begin
                        cap_col <= 4'd0;
                        state   <= S_CAPTURE;
                    end else begin
                        feed_col <= feed_col + 6'd1;
                    end
                end

                // Capture 10 output columns (52 bits each) into outbuf
                S_CAPTURE: begin
                    if (out_sample) begin
                        outbuf[cap_base +: ZC] <= core_dout_bus[ZC-1:0];
                        if (cap_col == OUT_CYCLES-1) begin
                            drain_count   <= 10'd0;
                            m_axis_tdata  <= outbuf[0];
                            m_axis_tvalid <= 1'b1;
                            m_axis_tlast  <= (K_BITS == 1);
                            state         <= S_DRAIN;
                        end else begin
                            cap_col <= cap_col + 4'd1;
                        end
                    end
                end

                // Drain 520 bits, one per beat, honoring backpressure
                S_DRAIN: begin
                    if (output_fire) begin
                        if (drain_count == K_BITS-1) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            state         <= S_COLLECT;
                        end else begin
                            drain_count  <= drain_count + 10'd1;
                            m_axis_tdata <= outbuf[drain_count + 10'd1];
                            m_axis_tlast <= (drain_count + 10'd1 == K_BITS-1);
                        end
                    end
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule
