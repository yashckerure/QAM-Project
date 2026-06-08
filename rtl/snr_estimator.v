`timescale 1ns / 1ps
//=============================================================================
// File        : snr_estimator.v
// Description : Decision-directed channel-quality estimator. Measures the
//               averaged decision error power (EVM^2 numerator) of equalized
//               symbols. Lower err_pow = better channel. Feeds mcs_controller.
//=============================================================================
// Complex symbol {Q[15:0], I[15:0]} signed Q5.10, post-equalizer. qam_mode
// selects the slicer constellation. Constellations are unit-average-power, so
// with reference power = 1.0, err_pow ~= EVM^2 and SNR_linear ~= 1/err_pow.
//
//   yhat   = slice(y, qam_mode)            nearest constellation point
//   err    = |yhat - y|^2  = e_re^2 + e_im^2          (Q10.20)
//   err_pow += (err - err_pow) >>> AVG_SH             IIR average (Q10.20)
//
// Output err_pow (Q10.20, 32-bit) is the quality metric. MCS thresholds are set
// in these units (calibrated from BER curves). qam_mode encoding must match
// qam_mapper: 0=QPSK 1=16QAM 2=64QAM 3=256QAM.
//=============================================================================
module snr_estimator #(
    parameter integer AVG_SH = 8
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [2:0]  qam_mode,
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [31:0] err_pow,            // Q10.20 averaged error power
    output reg         err_valid           // high once first sample processed
);
    assign s_axis_tready = 1'b1;           // always accepts (passive monitor)
    wire fire = s_axis_tvalid;

    wire signed [15:0] y_re = s_axis_tdata[15:0];
    wire signed [15:0] y_im = s_axis_tdata[31:16];

    function signed [15:0] slice_axis;
        input signed [15:0] v;
        input [2:0] mode;
        reg signed [15:0] av; reg [3:0] cnt; reg [15:0] mag;
        begin
            av = v[15] ? -v : v;
            case(mode)
              3'd0: mag = 16'd724;
              3'd1: begin cnt = (av>648); mag = (2*cnt+1)*324; end
              3'd2: begin cnt = (av>316)+(av>632)+(av>948); mag = (2*cnt+1)*158; end
              default: begin
                  cnt = (av>158)+(av>316)+(av>474)+(av>632)+(av>790)+(av>948)+(av>1106);
                  mag = (2*cnt+1)*79; end
            endcase
            slice_axis = v[15] ? -$signed(mag) : $signed(mag);
        end
    endfunction

    wire signed [15:0] dhat_re = slice_axis(y_re, qam_mode);
    wire signed [15:0] dhat_im = slice_axis(y_im, qam_mode);
    wire signed [16:0] e_re = dhat_re - y_re;
    wire signed [16:0] e_im = dhat_im - y_im;
    wire signed [39:0] err  = (e_re*e_re) + (e_im*e_im);   // Q10.20

    reg signed [39:0] errpow_s, delta;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            err_pow <= 32'd0; err_valid <= 1'b0;
        end else if(fire) begin
            errpow_s = $signed({8'd0, err_pow});
            delta    = (err - errpow_s) >>> AVG_SH;     // signed IIR step
            err_pow  <= (errpow_s + delta) >= 0 ? (errpow_s + delta) : 40'sd0;
            err_valid <= 1'b1;
        end
    end
endmodule
