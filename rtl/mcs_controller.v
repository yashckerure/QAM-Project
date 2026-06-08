`timescale 1ns / 1ps
//=============================================================================
// File        : mcs_controller.v
// Description : Link-adaptation controller. Maps the channel-quality metric
//               (err_pow from snr_estimator; lower = better) to an MCS index and
//               drives the adaptive blocks (qam_mode, E, n=E/Qm, RV).
//=============================================================================
// MCS table (curated monotonic-efficiency ladder; RV=0):
//   idx mod      qm_mode  E     n=E/Qm   (R)
//    0  QPSK        0     1040   520     1/2
//    1  16QAM       1     1040   260     1/2
//    2  16QAM       1      624   156     5/6
//    3  64QAM       2      624   104     5/6
//    4  256QAM      3      624    78     5/6
//
// Thresholds TH1>TH2>TH3>TH4 (err_pow units, Q10.20): metric below TH_k unlocks
// MCS k. Placeholders - calibrate from measured BER-curve crossovers.
// Hysteresis: active MCS moves one step toward the target only after the target
// has differed for DWELL consecutive valid samples (anti-flap). force_en pins a
// fixed MCS (for BER sweeps).
//=============================================================================
module mcs_controller #(
    parameter [31:0] TH1 = 32'd20000,   // unlock MCS1
    parameter [31:0] TH2 = 32'd9000,    // unlock MCS2
    parameter [31:0] TH3 = 32'd3500,    // unlock MCS3
    parameter [31:0] TH4 = 32'd1400,    // unlock MCS4
    parameter integer DWELL = 256
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [31:0] metric,          // err_pow from snr_estimator
    input  wire        metric_valid,
    input  wire        force_en,        // 1 = pin force_mcs
    input  wire [2:0]  force_mcs,
    output reg  [2:0]  mcs,
    output reg  [2:0]  qam_mode,
    output reg  [12:0] e_out,           // rate-match output length E
    output reg  [12:0] n_out,           // symbols = E/Qm (bit_interleaver n_in)
    output reg  [1:0]  rv_out
);
    // target MCS from thresholds (lower metric -> higher MCS)
    reg [2:0] target;
    always @(*) begin
        target = 3'd0;
        if (metric < TH1) target = 3'd1;
        if (metric < TH2) target = 3'd2;
        if (metric < TH3) target = 3'd3;
        if (metric < TH4) target = 3'd4;
    end

    reg [2:0] active;
    reg [15:0] dwell_cnt;

    // MCS table lookup -> outputs
    always @(*) begin
        case(active)
          3'd0: begin qam_mode=3'd0; e_out=13'd1040; n_out=13'd520; end
          3'd1: begin qam_mode=3'd1; e_out=13'd1040; n_out=13'd260; end
          3'd2: begin qam_mode=3'd1; e_out=13'd624;  n_out=13'd156; end
          3'd3: begin qam_mode=3'd2; e_out=13'd624;  n_out=13'd104; end
          default: begin qam_mode=3'd3; e_out=13'd624; n_out=13'd78; end
        endcase
        rv_out = 2'd0;
        mcs    = active;
    end

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            active <= 3'd0; dwell_cnt <= 16'd0;
        end else if(force_en) begin
            active <= (force_mcs > 3'd4) ? 3'd4 : force_mcs;
            dwell_cnt <= 16'd0;
        end else if(metric_valid) begin
            if(target != active) begin
                if(dwell_cnt >= DWELL-1) begin
                    active    <= (target > active) ? active + 3'd1 : active - 3'd1;
                    dwell_cnt <= 16'd0;
                end else dwell_cnt <= dwell_cnt + 16'd1;
            end else dwell_cnt <= 16'd0;
        end
    end
endmodule
