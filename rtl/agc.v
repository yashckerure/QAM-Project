`timescale 1ns / 1ps
//=============================================================================
// File        : agc.v
// Description : Automatic Gain Control. Normalizes received complex sample power
//               to unity. Sits on RX after the channel, before rrc_rx.
//=============================================================================
// Complex sample {Q[15:0], I[15:0]}, each signed Q5.10. 1 sample/clk, latency 1.
//
// Loop (all QAM constellations are unit-average-power, so the target is the same
// for every modulation -> AGC is modulation-independent):
//   y      = saturate( gain * x >>> 10 )          gain is Q6.10
//   p      = yI^2 + yQ^2                           instantaneous power (Q10.20)
//   avg_p += (p - avg_p) >>> AVG_SH               IIR power estimate
//   gain  += (REF_P - avg_p) >>> LOOP_SH          integrator (clamped)
// Negative feedback: low power -> gain rises -> power rises toward REF_P.
//
// REF_P default = 1.0 in Q10.20 (unit power). Loop only adapts on valid samples.
//=============================================================================
module agc #(
    parameter integer AVG_SH  = 6,                 // power-average time constant
    parameter integer LOOP_SH = 18,                // loop speed (larger = slower)
    parameter [39:0]  REF_P   = 40'd1048576,       // target power = 1.0 (Q10.20)
    parameter [15:0]  GAIN_INIT = 16'd1024,        // 1.0 in Q6.10
    parameter [15:0]  GMIN    = 16'd16,            // ~0.0156
    parameter [15:0]  GMAX    = 16'd32767          // ~31.99
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg  [15:0] gain                         // current gain (Q6.10) - observe/debug
);
    wire ce = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = ce;
    wire sample_fire = s_axis_tvalid && ce;

    wire signed [15:0] x_re = s_axis_tdata[15:0];
    wire signed [15:0] x_im = s_axis_tdata[31:16];

    // y = gain * x >>> 10, saturate to Q5.10
    function signed [15:0] sat16;
        input signed [32:0] v;
        begin
            if (v >  33'sd32767)      sat16 =  16'sd32767;
            else if (v < -33'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    wire signed [32:0] prod_re = $signed({1'b0,gain}) * x_re;   // Q6.10 * Q5.10 = Q11.20
    wire signed [32:0] prod_im = $signed({1'b0,gain}) * x_im;
    wire signed [15:0] y_re = sat16(prod_re >>> 10);
    wire signed [15:0] y_im = sat16(prod_im >>> 10);

    // instantaneous power yI^2 + yQ^2  (Q10.20)
    wire signed [39:0] p_inst = (y_re*y_re) + (y_im*y_im);

    reg signed [39:0] avg_p;
    wire signed [39:0] e = $signed(REF_P) - avg_p;

    reg signed [31:0] gnext;          // signed next-gain for clamping
    reg signed [39:0] e_step;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            gain          <= GAIN_INIT;
            avg_p         <= REF_P;        // start at target to avoid startup kick
            m_axis_tdata  <= 32'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else if(ce) begin
            // output (latency 1)
            m_axis_tdata  <= {y_im, y_re};
            m_axis_tvalid <= s_axis_tvalid;
            m_axis_tlast  <= s_axis_tlast;
            // adapt only on real samples
            if(sample_fire) begin
                avg_p <= avg_p + ((p_inst - avg_p) >>> AVG_SH);
                // integrator (explicit signed arithmetic) with clamp
                e_step = e >>> LOOP_SH;
                gnext  = $signed({16'd0, gain}) + e_step[31:0];
                if      (gnext < $signed({16'd0, GMIN})) gain <= GMIN;
                else if (gnext > $signed({16'd0, GMAX})) gain <= GMAX;
                else                                     gain <= gnext[15:0];
            end
        end
    end
endmodule
