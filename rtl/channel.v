`timescale 1ns / 1ps
//=============================================================================
// File        : channel.v
// Description : Synthesizable baseband channel model. Sits AFTER rrc_tx, at the
//               sample level (4 samples/symbol, 100 Msps @ 100 MHz = 1 samp/clk).
//               Physically-correct structure:
//                   x -> [ complex multipath/fading FIR ] -> [ + AWGN ] -> y
//               Both stages are in this one block (noise is added LAST, at the
//               RX front-end, after the signal has propagated through the paths).
//=============================================================================
// DATA FORMAT : complex sample {Q[15:0], I[15:0]}, each signed Q5.10 (val/1024).
//               1 sample per beat, AXI-Stream. Expects gap-free input (as rrc_tx
//               provides). Internal bubbles inject a zero sample (= no signal),
//               which is physically correct.
//
// STAGE 1 - MULTIPATH / FADING FIR
//   y = sum_k h[k] * x[n-k]   (complex), NUM_TAPS taps at 1-sample spacing.
//   Tap coeffs h[k] = {hi[k], hr[k]}, signed Q1.14, in a writable register file
//   (ports tap_we/tap_idx/tap_re/tap_im). DEFAULT = unit tap (h[0]=1+j0, rest 0)
//   => pass-through (AWGN-only). FADING = drive the taps over time from an
//   external process (SW on the Pynq, or a future fading-generator block):
//   writing new tap values each coherence interval realises Rayleigh/Rician
//   fading. Static non-unit taps realise fixed multipath.
//
// STAGE 2 - AWGN (Box-Muller)
//   Two uniform LFSRs -> ROM lookups f=sqrt(-2 ln U1), cos/sin(2*pi U2) ->
//   z0=f*cos, z1=f*sin are i.i.d. standard normals (Q5.10, std=1.0=1024).
//   noise_I = z0*noise_std, noise_Q = z1*noise_std  (noise_std = per-component
//   std-dev, Q5.10 unsigned, runtime). noise_std=0 disables noise.
//   ROMs loaded from bm_fmag.mem / bm_cos.mem / bm_sin.mem (add to project).
//
// PIPELINE : 4-cycle latency, 1 sample/clk throughput, global clock-enable stall
//   on downstream backpressure. MAC: 40-bit acc, >>>14, saturate (project conv).
//=============================================================================
module channel #(
    parameter integer NUM_TAPS = 8,
    parameter integer IDX_W    = 12,        // Box-Muller LUT index width (4096)
    parameter         FMAG_MEM = "bm_fmag.mem",
    parameter         COS_MEM  = "bm_cos.mem",
    parameter         SIN_MEM  = "bm_sin.mem",
    parameter [31:0]  SEED1    = 32'hACE12345,
    parameter [31:0]  SEED2    = 32'h1F2E3D4C
)(
    input  wire        aclk,
    input  wire        aresetn,

    // runtime control
    input  wire [15:0] noise_std,           // AWGN per-component std, Q5.10 unsigned

    // multipath/fading tap write port (write anytime; not gated by sample CE)
    input  wire                       tap_we,
    input  wire [$clog2(NUM_TAPS)-1:0] tap_idx,
    input  wire signed [15:0]         tap_re, // Q1.14
    input  wire signed [15:0]         tap_im, // Q1.14

    // AXI-Stream sample in/out : {Q,I} each signed Q5.10
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);
    //----------------------------------------------------------------------
    // clock enable / handshake (linear pipeline, freeze on downstream stall)
    //----------------------------------------------------------------------
    wire ce = m_axis_tready || !m_axis_tvalid;
    assign s_axis_tready = ce;

    wire signed [15:0] x_re = s_axis_tdata[15:0];
    wire signed [15:0] x_im = s_axis_tdata[31:16];

    //----------------------------------------------------------------------
    // STAGE 1 : complex multipath/fading FIR
    //----------------------------------------------------------------------
    reg signed [15:0] hr [0:NUM_TAPS-1];     // Q1.14
    reg signed [15:0] hi [0:NUM_TAPS-1];
    reg signed [15:0] dr [0:NUM_TAPS-1];     // delay line, Q5.10
    reg signed [15:0] di [0:NUM_TAPS-1];

    integer t;
    // tap write + reset to unit tap
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            for(t=0;t<NUM_TAPS;t=t+1) begin
                hr[t] <= (t==0) ? 16'sd16384 : 16'sd0;   // 1.0 in Q1.14 at tap 0
                hi[t] <= 16'sd0;
            end
        end else if(tap_we) begin
            hr[tap_idx] <= tap_re;
            hi[tap_idx] <= tap_im;
        end
    end

    // delay line shift (zero-inject on invalid = no signal)
    integer k;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            for(k=0;k<NUM_TAPS;k=k+1) begin dr[k]<=16'sd0; di[k]<=16'sd0; end
        end else if(ce) begin
            for(k=NUM_TAPS-1;k>0;k=k-1) begin dr[k]<=dr[k-1]; di[k]<=di[k-1]; end
            dr[0] <= s_axis_tvalid ? x_re : 16'sd0;
            di[0] <= s_axis_tvalid ? x_im : 16'sd0;
        end
    end

    // complex MAC : 40-bit acc, >>>14, saturate to Q5.10
    function signed [15:0] sat16;
        input signed [39:0] v;
        begin
            if (v >  40'sd32767)      sat16 =  16'sd32767;
            else if (v < -40'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    reg signed [39:0] acc_re, acc_im;
    integer m;
    always @(*) begin
        acc_re = 40'sd0; acc_im = 40'sd0;
        for(m=0;m<NUM_TAPS;m=m+1) begin
            acc_re = acc_re + (hr[m]*dr[m]) - (hi[m]*di[m]);
            acc_im = acc_im + (hr[m]*di[m]) + (hi[m]*dr[m]);
        end
    end

    reg signed [15:0] fir_re1, fir_im1, fir_re2, fir_im2, fir_re3, fir_im3;

    //----------------------------------------------------------------------
    // STAGE 2 : Box-Muller AWGN
    //----------------------------------------------------------------------
    (* rom_style="block" *) reg [15:0] f_lut   [0:(1<<IDX_W)-1];
    (* rom_style="block" *) reg [15:0] cos_lut [0:(1<<IDX_W)-1];
    (* rom_style="block" *) reg [15:0] sin_lut [0:(1<<IDX_W)-1];
    initial begin
        $readmemh(FMAG_MEM, f_lut);
        $readmemh(COS_MEM,  cos_lut);
        $readmemh(SIN_MEM,  sin_lut);
    end

    // two Fibonacci LFSRs (different maximal polys) -> uniforms
    reg [31:0] lfsr1, lfsr2;
    wire fb1 = lfsr1[31]^lfsr1[21]^lfsr1[1]^lfsr1[0];     // taps 32,22,2,1
    wire fb2 = lfsr2[31]^lfsr2[29]^lfsr2[25]^lfsr2[24];   // taps 32,30,26,25
    wire [IDX_W-1:0] idx1 = lfsr1[31:32-IDX_W];
    wire [IDX_W-1:0] idx2 = lfsr2[31:32-IDX_W];

    reg [IDX_W-1:0] idx1_r, idx2_r;
    reg [15:0] f_r, c_r, s_r;          // f_r Q3.13 unsigned ; c_r,s_r Q1.14 signed
    reg signed [15:0] z0_r, z1_r;      // standard normals, Q5.10
    reg signed [15:0] nI, nQ;          // scaled noise, Q5.10

    // f*trig -> Q5.10 : Q3.13 * Q1.14 = Q4.27, >>>17
    function signed [15:0] sat_shift17;
        input signed [31:0] v;          // Q4.27
        reg signed [31:0] sh;
        begin
            sh = v >>> 17;
            if (sh >  32'sd32767)      sat_shift17 =  16'sd32767;
            else if (sh < -32'sd32768) sat_shift17 = -16'sd32768;
            else                       sat_shift17 = sh[15:0];
        end
    endfunction
    // z*sigma -> Q5.10 : Q5.10 * Q5.10 = Q10.20, >>>10
    function signed [15:0] sat_shift10;
        input signed [31:0] v;          // Q10.20
        reg signed [31:0] sh;
        begin
            sh = v >>> 10;
            if (sh >  32'sd32767)      sat_shift10 =  16'sd32767;
            else if (sh < -32'sd32768) sat_shift10 = -16'sd32768;
            else                       sat_shift10 = sh[15:0];
        end
    endfunction

    function signed [15:0] sat_add;
        input signed [16:0] v;
        begin
            if (v >  17'sd32767)      sat_add =  16'sd32767;
            else if (v < -17'sd32768) sat_add = -16'sd32768;
            else                      sat_add = v[15:0];
        end
    endfunction

    //----------------------------------------------------------------------
    // valid / last pipeline (4-cycle latency)
    //----------------------------------------------------------------------
    reg vld0,vld1,vld2,vld3, lst0,lst1,lst2,lst3;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            lfsr1<=SEED1; lfsr2<=SEED2;
            idx1_r<=0; idx2_r<=0; f_r<=0; c_r<=0; s_r<=0;
            z0_r<=0; z1_r<=0; nI<=0; nQ<=0;
            fir_re1<=0; fir_im1<=0; fir_re2<=0; fir_im2<=0; fir_re3<=0; fir_im3<=0;
            vld0<=0; vld1<=0; vld2<=0; vld3<=0;
            lst0<=0; lst1<=0; lst2<=0; lst3<=0;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else if(ce) begin
            // --- t0 : advance RNG, capture indices ; FIR delay line shifts above
            lfsr1 <= {lfsr1[30:0], fb1};
            lfsr2 <= {lfsr2[30:0], fb2};
            idx1_r <= idx1; idx2_r <= idx2;
            vld0 <= s_axis_tvalid; lst0 <= s_axis_tlast;

            // --- t1 : ROM reads ; FIR MAC registered
            f_r <= f_lut[idx1_r];
            c_r <= cos_lut[idx2_r];
            s_r <= sin_lut[idx2_r];
            fir_re1 <= sat16(acc_re >>> 14);
            fir_im1 <= sat16(acc_im >>> 14);
            vld1 <= vld0; lst1 <= lst0;

            // --- t2 : standard normals ; FIR delay
            z0_r <= sat_shift17($signed({16'd0,f_r}) * $signed(c_r));
            z1_r <= sat_shift17($signed({16'd0,f_r}) * $signed(s_r));
            fir_re2 <= fir_re1; fir_im2 <= fir_im1;
            vld2 <= vld1; lst2 <= lst1;

            // --- t3 : scale by noise_std ; FIR delay
            nI <= sat_shift10($signed(z0_r) * $signed({1'b0,noise_std}));
            nQ <= sat_shift10($signed(z1_r) * $signed({1'b0,noise_std}));
            fir_re3 <= fir_re2; fir_im3 <= fir_im2;
            vld3 <= vld2; lst3 <= lst2;

            // --- t4 : add noise to faded signal -> output
            m_axis_tdata[15:0]  <= sat_add(fir_re3 + nI);
            m_axis_tdata[31:16] <= sat_add(fir_im3 + nQ);
            m_axis_tvalid <= vld3;
            m_axis_tlast  <= lst3;
        end
    end
endmodule
