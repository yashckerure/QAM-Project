`timescale 1ns / 1ps
//=============================================================================
// File        : equalizer.v
// Description : Adaptive LMS linear equalizer (complex T-spaced FIR). Removes
//               residual ISI / multipath. Sits on RX after Costas, before the
//               soft_demapper. Learns the inverse of the channel multipath FIR.
//=============================================================================
// Complex symbol {Q[15:0], I[15:0]}, signed Q5.10. 1 symbol in / 1 out.
// Taps wr/wi : Q3.12 (range +/-8). Center tap init = 1.0 -> initial pass-through.
//
//   y[k]   = sum_i w[i] * x[k-i]                    (complex FIR, current taps)
//   error  = train_en ? (d_train - y) : (slice(y) - y)
//   w[i]  += mu * error * conj(x[k-i])              (complex LMS, mu = 2^-MU_SH)
//
// TRAINING mode (train_en=1): error vs known pilot d_train (aligned to center
//   tap, i.e. d_train = transmitted symbol delayed by CENTER). Use for blind
//   acquisition on a pilot/preamble. DECISION-DIRECTED mode (train_en=0): error
//   vs the hard decision from the per-qam_mode slicer; use after acquisition.
//
// qam_mode encoding (must match qam_mapper): 0=QPSK 1=16QAM 2=64QAM 3=256QAM.
// CALIBRATION: MU_SH (step size) - larger = slower/stabler. Verify at integration.
//=============================================================================
module equalizer #(
    parameter integer NTAPS  = 7,
    parameter integer CENTER = 3,          // center (reference) tap index
    parameter integer MU_SH  = 6           // LMS step = 2^-MU_SH (after >>>8)
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [2:0]  qam_mode,
    input  wire        train_en,
    input  wire [31:0] d_train,            // training symbol {Q,I} Q5.10
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);
    wire ce = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = ce;
    wire fire = s_axis_tvalid && ce;

    wire signed [15:0] in_re = s_axis_tdata[15:0];
    wire signed [15:0] in_im = s_axis_tdata[31:16];

    // delay line (history); current sample combined in xr_c/xi_c below
    reg signed [15:0] dr [0:NTAPS-2];
    reg signed [15:0] di [0:NTAPS-2];
    // taps Q3.12
    reg signed [15:0] wr [0:NTAPS-1];
    reg signed [15:0] wi [0:NTAPS-1];

    integer i;

    // current tap inputs x[k..k-N+1] : index0 = new sample, rest = history
    function signed [15:0] xr_c; input integer idx;
        begin xr_c = (idx==0) ? in_re : dr[idx-1]; end endfunction
    function signed [15:0] xi_c; input integer idx;
        begin xi_c = (idx==0) ? in_im : di[idx-1]; end endfunction

    function signed [15:0] sat16;
        input signed [39:0] v;
        begin
            if (v >  40'sd32767)      sat16 =  16'sd32767;
            else if (v < -40'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    // ---- FIR output y = sum w[i]*x[i]  (Q3.12 * Q5.10 = Q8.22, >>>12 -> Q5.10)
    reg signed [39:0] acc_re, acc_im;
    always @(*) begin
        acc_re = 40'sd0; acc_im = 40'sd0;
        for(i=0;i<NTAPS;i=i+1) begin
            acc_re = acc_re + (wr[i]*xr_c(i)) - (wi[i]*xi_c(i));
            acc_im = acc_im + (wr[i]*xi_c(i)) + (wi[i]*xr_c(i));
        end
    end
    wire signed [15:0] y_re = sat16(acc_re >>> 12);
    wire signed [15:0] y_im = sat16(acc_im >>> 12);

    // ---- per-mode nearest-level slicer (decision-directed mode)
    function signed [15:0] slice_axis;
        input signed [15:0] v;
        input [2:0] mode;
        reg signed [15:0] av; reg [3:0] cnt; reg [15:0] mag;
        begin
            av = v[15] ? -v : v;
            case(mode)
              3'd0: begin mag = 16'd724;  end                       // QPSK
              3'd1: begin cnt = (av>648); mag = (2*cnt+1)*324; end  // 16QAM
              3'd2: begin cnt = (av>316)+(av>632)+(av>948);         // 64QAM
                          mag = (2*cnt+1)*158; end
              default: begin                                        // 256QAM
                          cnt = (av>158)+(av>316)+(av>474)+(av>632)
                              +(av>790)+(av>948)+(av>1106);
                          mag = (2*cnt+1)*79; end
            endcase
            slice_axis = v[15] ? -$signed(mag) : $signed(mag);
        end
    endfunction

    wire signed [15:0] dec_re = train_en ? $signed(d_train[15:0])  : slice_axis(y_re, qam_mode);
    wire signed [15:0] dec_im = train_en ? $signed(d_train[31:16]) : slice_axis(y_im, qam_mode);
    wire signed [16:0] e_re = dec_re - y_re;      // Q5.10
    wire signed [16:0] e_im = dec_im - y_im;

    // ---- LMS tap update : w += (e * conj(x)) >>> (8+MU_SH)
    function signed [15:0] sat_tap;       // saturate to Q3.12 16-bit
        input signed [39:0] v;
        begin
            if (v >  40'sd32767)      sat_tap =  16'sd32767;
            else if (v < -40'sd32768) sat_tap = -16'sd32768;
            else                      sat_tap = v[15:0];
        end
    endfunction

    reg signed [39:0] ecx_re, ecx_im;     // e*conj(x) terms, Q10.20

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            for(i=0;i<NTAPS;i=i+1) begin
                wr[i] <= (i==CENTER) ? 16'sd4096 : 16'sd0;   // 1.0 Q3.12 at center
                wi[i] <= 16'sd0;
            end
            for(i=0;i<NTAPS-1;i=i+1) begin dr[i]<=0; di[i]<=0; end
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(m_axis_tvalid && m_axis_tready) m_axis_tvalid<=1'b0;
            if(fire) begin
                // LMS update (signed throughout): e*conj(x) = e*(xr - j xi)
                for(i=0;i<NTAPS;i=i+1) begin
                    ecx_re = ($signed(e_re)*xr_c(i)) + ($signed(e_im)*xi_c(i));
                    ecx_im = ($signed(e_im)*xr_c(i)) - ($signed(e_re)*xi_c(i));
                    wr[i] <= sat_tap($signed({{24{wr[i][15]}},wr[i]}) + (ecx_re >>> (8+MU_SH)));
                    wi[i] <= sat_tap($signed({{24{wi[i][15]}},wi[i]}) + (ecx_im >>> (8+MU_SH)));
                end
                // shift delay line (history gets current sample)
                for(i=NTAPS-2;i>0;i=i-1) begin dr[i]<=dr[i-1]; di[i]<=di[i-1]; end
                dr[0]<=in_re; di[0]<=in_im;
                // output equalized symbol
                m_axis_tdata  <= {y_im, y_re};
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;
            end
        end
    end
endmodule
