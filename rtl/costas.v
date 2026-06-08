`timescale 1ns / 1ps
//=============================================================================
// File        : costas.v
// Description : Costas carrier phase/frequency recovery loop. Operates at symbol
//               rate. De-rotates complex symbols to remove residual carrier
//               phase and frequency offset.
//=============================================================================
// Complex symbol {Q[15:0], I[15:0]}, signed Q5.10. 1 symbol in / 1 out.
//
//   theta     = phase_acc[31:20]                          (NCO -> 4096-pt LUT)
//   y = x * e^{-j theta} :
//       y_I =  x_I*cos + x_Q*sin   (>>>14)
//       y_Q = -x_I*sin + x_Q*cos   (>>>14)
//   phase error (classic Costas, mode-independent, hard-limited decisions):
//       e = sign(y_I)*y_Q - sign(y_Q)*y_I        (~ |y| sin(phase error))
//   PI loop / NCO:
//       freq_word += e <<< KI_SHL                 (integral -> tracks freq offset)
//       phase_acc += freq_word + (e <<< KP_SHL)   (proportional -> fast phase)
//   Output = de-rotated symbol y.
//
// CALIBRATION (verify polarity / tune at integration): KP_SHL, KI_SHL loop gains,
//   and the e sign. freq_word is clamped to prevent runaway.
// LUTs costas_cos.mem / costas_sin.mem (4096 x Q1.14) must be added to project.
//=============================================================================
module costas #(
    parameter integer KP_SHL  = 14,                 // proportional gain (left shift)
    parameter integer KI_SHL  = 4,                  // integral gain (left shift)
    parameter signed [31:0] FREQ_LIM = 32'sd268435456, // +/-2^28 clamp on freq_word
    parameter         COS_MEM = "costas_cos.mem",
    parameter         SIN_MEM = "costas_sin.mem"
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
    output reg  signed [31:0] freq_dbg               // observe acquired freq
);
    wire ce = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = ce;
    wire fire = s_axis_tvalid && ce;

    wire signed [15:0] x_re = s_axis_tdata[15:0];
    wire signed [15:0] x_im = s_axis_tdata[31:16];

    (* rom_style="block" *) reg signed [15:0] cos_lut [0:4095];
    (* rom_style="block" *) reg signed [15:0] sin_lut [0:4095];
    initial begin $readmemh(COS_MEM, cos_lut); $readmemh(SIN_MEM, sin_lut); end

    reg [31:0] phase_acc;
    reg signed [31:0] freq_word;

    wire [11:0] theta = phase_acc[31:20];
    wire signed [15:0] c = cos_lut[theta];
    wire signed [15:0] s = sin_lut[theta];

    function signed [15:0] sat16;
        input signed [33:0] v;
        begin
            if (v >  34'sd32767)      sat16 =  16'sd32767;
            else if (v < -34'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    // de-rotate by theta : y = x * e^{-j theta}
    wire signed [33:0] yi_acc = (x_re * c) + (x_im * s);   // Q6.24
    wire signed [33:0] yq_acc = (x_im * c) - (x_re * s);
    wire signed [15:0] y_re = sat16(yi_acc >>> 14);
    wire signed [15:0] y_im = sat16(yq_acc >>> 14);

    // classic Costas phase error: sign(yI)*yQ - sign(yQ)*yI
    wire signed [16:0] term0 = y_re[15] ? -y_im : y_im;   // sign(yI)*yQ
    wire signed [16:0] term1 = y_im[15] ? -y_re : y_re;   // sign(yQ)*yI
    wire signed [17:0] e = term0 - term1;

    reg signed [31:0] freq_nxt;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            phase_acc<=32'd0; freq_word<=32'sd0; freq_dbg<=32'sd0;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(m_axis_tvalid && m_axis_tready) m_axis_tvalid<=1'b0;
            if(fire) begin
                // integral path (signed) with clamp
                freq_nxt = freq_word + ($signed(e) <<< KI_SHL);
                if      (freq_nxt >  FREQ_LIM) freq_word <=  FREQ_LIM;
                else if (freq_nxt < -FREQ_LIM) freq_word <= -FREQ_LIM;
                else                           freq_word <= freq_nxt;
                // NCO: freq + proportional
                phase_acc <= phase_acc + freq_word + ($signed(e) <<< KP_SHL);
                freq_dbg  <= freq_word;
                // output de-rotated symbol
                m_axis_tdata  <= {y_im, y_re};
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;
            end
        end
    end
endmodule
