`timescale 1ns / 1ps
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : soft_demapper.v
// Description  : Soft-decision QAM demapper (max-log LLR) for QPSK/16/64/256.
//                Inverse of qam_mapper.v. One symbol in -> Qm 4-bit signed LLRs
//                out (one per beat, MSB-first, matching the packer/mapper order).
//=============================================================================
// Additional Notes:
// - Input  s_axis_tdata = {Q[15:0], I[15:0]} signed Q5.10 (qam_mapper output fmt)
// - Output m_axis_tdata = signed [3:0] LLR; positive => bit 0, negative => bit 1
// - Max-log LLR via per-axis nested-abs (Gray PAM): no mult, no ROM
//     k=Qm/2-1 (MSB): r ; lower bits fold with thresholds 2^k * scale
// - scale per mode = 724/324/158/79 (= mapper). LLR scaled to 4-bit by >>SH+sat.
//   SH per mode {7,6,5,4}; absolute LLR scale is a documented tunable (decoder
//   is robust to it; final value set at BER integration).
// - Registered outputs; AXI-Stream backpressure honored.
//=============================================================================

module soft_demapper #(
    parameter integer DATA_W = 16
)(
    input  wire                  aclk,
    input  wire                  aresetn,
    input  wire  [2:0]           qam_mode,
    input  wire  [2*DATA_W-1:0]  s_axis_tdata,   // {Q, I} Q5.10
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    output reg   signed [3:0]    m_axis_tdata,    // one 4-bit signed LLR
    output reg                   m_axis_tvalid,
    input  wire                  m_axis_tready,
    output reg                   m_axis_tlast
);
    // ---- mode -> bits/axis, Qm, scale, shift -------------------------------
    reg [2:0] m;      // bps_axis
    reg [9:0] s;      // scale
    reg [2:0] sh;     // output right-shift
    always @(*) begin
        case (qam_mode)
            3'd0: begin m=3'd1; s=10'd724; sh=3'd7; end
            3'd1: begin m=3'd2; s=10'd324; sh=3'd6; end
            3'd2: begin m=3'd3; s=10'd158; sh=3'd5; end
            3'd3: begin m=3'd4; s=10'd79;  sh=3'd4; end
            default: begin m=3'd2; s=10'd324; sh=3'd6; end
        endcase
    end
    wire [3:0] Qm = {m,1'b0};   // 2*m

    wire signed [DATA_W-1:0] rI = s_axis_tdata[DATA_W-1:0];
    wire signed [DATA_W-1:0] rQ = s_axis_tdata[2*DATA_W-1:DATA_W];

    // ---- per-axis nested-abs max-log LLRs (combinational) ------------------
    // returns lvl[k], k=0..m-1 (k=m-1 is MSB)
    function signed [17:0] absv(input signed [17:0] x);
        absv = x[17] ? -x : x;
    endfunction

    // compute the 4 possible LLR slots for one axis value r
    task axis_llrs(input signed [DATA_W-1:0] r,
                   output signed [17:0] l0, output signed [17:0] l1,
                   output signed [17:0] l2, output signed [17:0] l3);
        reg signed [17:0] a, rr, s2, s4, s8;
        begin
            rr = r; a = absv(rr);
            s2 = 2*s; s4 = 4*s; s8 = 8*s;
            l3 = 18'sd0; l2 = 18'sd0; l1 = 18'sd0; l0 = 18'sd0;
            case (m)
                3'd1: begin l0 = rr; end                                   // k0=MSB
                3'd2: begin l1 = rr; l0 = s2 - a; end                      // k1=MSB,k0=LSB
                3'd3: begin l2 = rr; l1 = s4 - a; l0 = s2 - absv(s4 - a); end
                3'd4: begin l3 = rr; l2 = s8 - a;
                            l1 = s4 - absv(s8 - a);
                            l0 = s2 - absv(s4 - absv(s8 - a)); end
                default: begin l1 = rr; l0 = s2 - a; end
            endcase
        end
    endtask

    function signed [3:0] q4(input signed [17:0] x, input [2:0] shift);
        reg signed [17:0] t;
        begin
            t = x >>> shift;
            if (t > 18'sd7)       q4 = 4'sd7;
            else if (t < -18'sd8) q4 = -4'sd8;
            else                  q4 = t[3:0];
        end
    endfunction

    // emission buffer: slot index 0..Qm-1, MSB-first symbol order
    //   slot even -> I axis, odd -> Q axis ; gray-bit k = m-1-(slot>>1)
    reg signed [3:0] llrbuf [0:7];
    integer si; reg [2:0] kk; reg axisI;
    reg signed [17:0] iL0,iL1,iL2,iL3, qL0,qL1,qL2,qL3;
    reg signed [17:0] selI, selQ;

    // ---- output FSM --------------------------------------------------------
    localparam S_IDLE=1'b0, S_DRAIN=1'b1;
    reg state;
    reg [3:0] cnt;     // drain index 0..Qm-1

    assign s_axis_tready = (state==S_IDLE);
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state<=S_IDLE; cnt<=4'd0;
            m_axis_tdata<=4'sd0; m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (input_fire) begin
                        axis_llrs(rI, iL0,iL1,iL2,iL3);
                        axis_llrs(rQ, qL0,qL1,qL2,qL3);
                        // fill buffer slots 0..Qm-1 (MSB-first symbol order):
                        //   even slot -> I axis, odd -> Q ; gray-bit k = m-1-(slot>>1)
                        for (si=0; si<8; si=si+1) begin
                            kk    = (m-1) - (si>>1);
                            axisI = ~si[0];
                            selI = (kk==3)?iL3 : (kk==2)?iL2 : (kk==1)?iL1 : iL0;
                            selQ = (kk==3)?qL3 : (kk==2)?qL2 : (kk==1)?qL1 : qL0;
                            llrbuf[si] = q4(axisI?selI:selQ, sh);   // blocking: ready same cycle
                        end
                        cnt          <= 4'd0;
                        m_axis_tdata <= llrbuf[0];                  // preload first LLR
                        m_axis_tvalid<= 1'b1;
                        m_axis_tlast <= (Qm==4'd1);
                        state        <= S_DRAIN;
                    end
                end
                S_DRAIN: begin
                    if (output_fire) begin
                        if (cnt==Qm-4'd1) begin
                            m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0; state<=S_IDLE;
                        end else begin
                            cnt<=cnt+4'd1;
                            m_axis_tdata<=llrbuf[cnt+4'd1];
                            m_axis_tlast<=((cnt+4'd1)==Qm-4'd1);
                        end
                    end
                end
            endcase
        end
    end
endmodule
