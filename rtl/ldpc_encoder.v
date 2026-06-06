`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : ldpc_encoder.v
// Description  : AXI-Stream wrapper around the HDL Coder generated NR LDPC
//                Encoder core (BG2, Zc=52, K=520 -> N=2600).
//=============================================================================
// Additional Notes:
// - Exposes the project's 1-bit-per-beat AXI-Stream interface and aclk/aresetn.
//   Internally instantiates HDL_Algorithm (the generated core) and gearboxes
//   between the serial stream and the core's 52-wide parallel frame interface.
// - Core config tied to constants: bgn=1 (=> base graph 2), liftingSize=52.
// - Core reset is async active-low (generated with ResetType=async,
//   ResetAssertedLevel=active-low), so .reset connects directly to aresetn.
//   Core clk_enable tied high; ce_out follows clk_enable.
// - Flow: COLLECT 520 bits -> wait nextFrame -> FEED 10 cols (52 bits/col)
//   -> CAPTURE 50 cols -> DRAIN 2600 bits. One code block per pass; the block
//   self-sequences back to COLLECT for the next block.
// - Backpressure: s_axis_tready high only while collecting; m_axis honors
//   m_axis_tready while draining. The core itself is never stalled.
// - Registered AXI-Stream outputs (Pynq/synthesis-friendly).
// - Bit order matches MATLAB reshape(msg,Zc,K) column-major: info bit n goes to
//   core column n/52, lane n%52; output column k, lane i -> codeword bit k*52+i.
//   This is the exact mapping verified bit-exact against nrLDPCEncode.
//=============================================================================

module ldpc_encoder (
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    output reg         m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);

    // -----------------------------------------------------------------------
    // Frame geometry (BG2, Zc=52)
    // -----------------------------------------------------------------------
    localparam integer ZC         = 52;
    localparam integer K_BITS     = 520;    // info bits  = 10 * Zc
    localparam integer N_BITS     = 2600;   // coded bits = 50 * Zc
    localparam integer IN_CYCLES  = 10;     // K_BITS / Zc
    localparam integer OUT_CYCLES = 50;     // N_BITS / Zc

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
    reg [K_BITS-1:0] inbuf;
    reg [N_BITS-1:0] outbuf;

    reg [9:0]  collect_count;   // 0..519
    reg [3:0]  feed_col;        // 0..9
    reg [5:0]  cap_col;         // 0..49
    reg [11:0] drain_count;     // 0..2599

    // -----------------------------------------------------------------------
    // Core interface wires
    // -----------------------------------------------------------------------
    wire [63:0] core_din;
    wire [63:0] core_dout;
    wire        core_ce_out;
    wire        core_ctrlOut_start;
    wire        core_ctrlOut_end;
    wire        core_ctrlOut_valid;
    wire        core_nextFrame;

    wire feeding = (state == S_FEED);

    // Present one 52-bit column per feed cycle on lanes 0..51; lanes 52..63 = 0
    wire [9:0]  feed_base  = feed_col * ZC;            // 0..468
    wire [51:0] feed_slice = feeding ? inbuf[feed_base +: ZC] : 52'b0;
    assign core_din = {12'b0, feed_slice};

    wire core_ctrlIn_valid = feeding;
    wire core_ctrlIn_start = feeding && (feed_col == 4'd0);
    wire core_ctrlIn_end   = feeding && (feed_col == IN_CYCLES-1);

    // Valid output sample from the core (ce_out follows clk_enable=1)
    wire out_sample = core_ce_out && core_ctrlOut_valid;

    // Accept input only while collecting
    assign s_axis_tready = (state == S_COLLECT);

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // -----------------------------------------------------------------------
    // Generated NR LDPC Encoder core
    // -----------------------------------------------------------------------
    HDL_Algorithm u_core (
        .clk(aclk),
        .reset(aresetn),          // active-low async, per generation settings
        .clk_enable(1'b1),
        .dataIn_0(core_din[0]),
        .dataIn_1(core_din[1]),
        .dataIn_2(core_din[2]),
        .dataIn_3(core_din[3]),
        .dataIn_4(core_din[4]),
        .dataIn_5(core_din[5]),
        .dataIn_6(core_din[6]),
        .dataIn_7(core_din[7]),
        .dataIn_8(core_din[8]),
        .dataIn_9(core_din[9]),
        .dataIn_10(core_din[10]),
        .dataIn_11(core_din[11]),
        .dataIn_12(core_din[12]),
        .dataIn_13(core_din[13]),
        .dataIn_14(core_din[14]),
        .dataIn_15(core_din[15]),
        .dataIn_16(core_din[16]),
        .dataIn_17(core_din[17]),
        .dataIn_18(core_din[18]),
        .dataIn_19(core_din[19]),
        .dataIn_20(core_din[20]),
        .dataIn_21(core_din[21]),
        .dataIn_22(core_din[22]),
        .dataIn_23(core_din[23]),
        .dataIn_24(core_din[24]),
        .dataIn_25(core_din[25]),
        .dataIn_26(core_din[26]),
        .dataIn_27(core_din[27]),
        .dataIn_28(core_din[28]),
        .dataIn_29(core_din[29]),
        .dataIn_30(core_din[30]),
        .dataIn_31(core_din[31]),
        .dataIn_32(core_din[32]),
        .dataIn_33(core_din[33]),
        .dataIn_34(core_din[34]),
        .dataIn_35(core_din[35]),
        .dataIn_36(core_din[36]),
        .dataIn_37(core_din[37]),
        .dataIn_38(core_din[38]),
        .dataIn_39(core_din[39]),
        .dataIn_40(core_din[40]),
        .dataIn_41(core_din[41]),
        .dataIn_42(core_din[42]),
        .dataIn_43(core_din[43]),
        .dataIn_44(core_din[44]),
        .dataIn_45(core_din[45]),
        .dataIn_46(core_din[46]),
        .dataIn_47(core_din[47]),
        .dataIn_48(core_din[48]),
        .dataIn_49(core_din[49]),
        .dataIn_50(core_din[50]),
        .dataIn_51(core_din[51]),
        .dataIn_52(core_din[52]),
        .dataIn_53(core_din[53]),
        .dataIn_54(core_din[54]),
        .dataIn_55(core_din[55]),
        .dataIn_56(core_din[56]),
        .dataIn_57(core_din[57]),
        .dataIn_58(core_din[58]),
        .dataIn_59(core_din[59]),
        .dataIn_60(core_din[60]),
        .dataIn_61(core_din[61]),
        .dataIn_62(core_din[62]),
        .dataIn_63(core_din[63]),
        .ctrlIn_start(core_ctrlIn_start),
        .ctrlIn_end(core_ctrlIn_end),
        .ctrlIn_valid(core_ctrlIn_valid),
        .bgn(1'b1),               // base graph 2
        .liftingSizeIn(16'd52),   // Zc = 52
        .ce_out(core_ce_out),
        .dataOut_0(core_dout[0]),
        .dataOut_1(core_dout[1]),
        .dataOut_2(core_dout[2]),
        .dataOut_3(core_dout[3]),
        .dataOut_4(core_dout[4]),
        .dataOut_5(core_dout[5]),
        .dataOut_6(core_dout[6]),
        .dataOut_7(core_dout[7]),
        .dataOut_8(core_dout[8]),
        .dataOut_9(core_dout[9]),
        .dataOut_10(core_dout[10]),
        .dataOut_11(core_dout[11]),
        .dataOut_12(core_dout[12]),
        .dataOut_13(core_dout[13]),
        .dataOut_14(core_dout[14]),
        .dataOut_15(core_dout[15]),
        .dataOut_16(core_dout[16]),
        .dataOut_17(core_dout[17]),
        .dataOut_18(core_dout[18]),
        .dataOut_19(core_dout[19]),
        .dataOut_20(core_dout[20]),
        .dataOut_21(core_dout[21]),
        .dataOut_22(core_dout[22]),
        .dataOut_23(core_dout[23]),
        .dataOut_24(core_dout[24]),
        .dataOut_25(core_dout[25]),
        .dataOut_26(core_dout[26]),
        .dataOut_27(core_dout[27]),
        .dataOut_28(core_dout[28]),
        .dataOut_29(core_dout[29]),
        .dataOut_30(core_dout[30]),
        .dataOut_31(core_dout[31]),
        .dataOut_32(core_dout[32]),
        .dataOut_33(core_dout[33]),
        .dataOut_34(core_dout[34]),
        .dataOut_35(core_dout[35]),
        .dataOut_36(core_dout[36]),
        .dataOut_37(core_dout[37]),
        .dataOut_38(core_dout[38]),
        .dataOut_39(core_dout[39]),
        .dataOut_40(core_dout[40]),
        .dataOut_41(core_dout[41]),
        .dataOut_42(core_dout[42]),
        .dataOut_43(core_dout[43]),
        .dataOut_44(core_dout[44]),
        .dataOut_45(core_dout[45]),
        .dataOut_46(core_dout[46]),
        .dataOut_47(core_dout[47]),
        .dataOut_48(core_dout[48]),
        .dataOut_49(core_dout[49]),
        .dataOut_50(core_dout[50]),
        .dataOut_51(core_dout[51]),
        .dataOut_52(core_dout[52]),
        .dataOut_53(core_dout[53]),
        .dataOut_54(core_dout[54]),
        .dataOut_55(core_dout[55]),
        .dataOut_56(core_dout[56]),
        .dataOut_57(core_dout[57]),
        .dataOut_58(core_dout[58]),
        .dataOut_59(core_dout[59]),
        .dataOut_60(core_dout[60]),
        .dataOut_61(core_dout[61]),
        .dataOut_62(core_dout[62]),
        .dataOut_63(core_dout[63]),
        .ctrlOut_start(core_ctrlOut_start),
        .ctrlOut_end(core_ctrlOut_end),
        .ctrlOut_valid(core_ctrlOut_valid),
        .liftingSizeOut(),        // unused
        .nextFrame(core_nextFrame)
    );

    // -----------------------------------------------------------------------
    // Sequential control
    // -----------------------------------------------------------------------
    wire [11:0] cap_base = cap_col * ZC;   // 0..2548

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= S_COLLECT;
            collect_count <= 10'd0;
            feed_col      <= 4'd0;
            cap_col       <= 6'd0;
            drain_count   <= 12'd0;
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            case (state)
                // ---------------------------------------------------------
                // Collect 520 serial info bits into inbuf[0..519]
                // ---------------------------------------------------------
                S_COLLECT: begin
                    if (input_fire) begin
                        inbuf[collect_count] <= s_axis_tdata;
                        if (collect_count == K_BITS-1) begin
                            collect_count <= 10'd0;
                            state         <= S_WAIT;
                        end else begin
                            collect_count <= collect_count + 10'd1;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Wait until the core is ready to accept a new frame
                // ---------------------------------------------------------
                S_WAIT: begin
                    if (core_nextFrame) begin
                        feed_col <= 4'd0;
                        state    <= S_FEED;
                    end
                end

                // ---------------------------------------------------------
                // Feed 10 columns of 52 bits (ctrlIn_* driven combinationally)
                // ---------------------------------------------------------
                S_FEED: begin
                    if (feed_col == IN_CYCLES-1) begin
                        cap_col <= 6'd0;
                        state   <= S_CAPTURE;
                    end else begin
                        feed_col <= feed_col + 4'd1;
                    end
                end

                // ---------------------------------------------------------
                // Capture 50 output columns into outbuf[0..2599]
                // ---------------------------------------------------------
                S_CAPTURE: begin
                    if (out_sample) begin
                        outbuf[cap_base +: ZC] <= core_dout[ZC-1:0];
                        if (cap_col == OUT_CYCLES-1) begin
                            // Last column captured; preload first drain bit.
                            // outbuf[0] was written 49 cycles ago and is stable.
                            drain_count   <= 12'd0;
                            m_axis_tdata  <= outbuf[0];
                            m_axis_tvalid <= 1'b1;
                            m_axis_tlast  <= (N_BITS == 1);
                            state         <= S_DRAIN;
                        end else begin
                            cap_col <= cap_col + 6'd1;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Drain 2600 bits, one per beat, honoring backpressure
                // ---------------------------------------------------------
                S_DRAIN: begin
                    if (output_fire) begin
                        if (drain_count == N_BITS-1) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            state         <= S_COLLECT;
                        end else begin
                            drain_count  <= drain_count + 12'd1;
                            m_axis_tdata <= outbuf[drain_count + 12'd1];
                            m_axis_tlast <= (drain_count + 12'd1 == N_BITS-1);
                        end
                    end
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule
