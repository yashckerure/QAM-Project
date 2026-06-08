`timescale 1ns / 1ps
//=============================================================================
// File        : gardner.v
// Description : Gardner symbol-timing recovery + 4->1 downsampler. Sits on RX
//               after Costas (or after rrc_rx), input at SPS=4, output 1 symbol
//               per symbol at the recovered timing instant.
//=============================================================================
// Complex samples {Q[15:0], I[15:0]}, signed Q5.10. Input 4 samples/symbol.
//
// Loop (standard digital timing PLL):
//   - Linear (Farrow deg-1) interpolator: y(mu) = a + mu*(b-a).
//   - Mod-1 decrementing NCO: each input sample nco -= mu_step (nominal 1/SPS).
//     On underflow -> STROBE (one per symbol); mu = nco_old/mu_step (~nco_old*SPS).
//   - SPS=4 => the Gardner MID-POINT sample is exactly 2 input samples before the
//     on-time sample, so both come from one delay line at the same mu.
//   - Gardner TED (decision-free):
//       e = (yon_I - yonPrev_I)*ymid_I + (yon_Q - yonPrev_Q)*ymid_Q
//   - PI loop filter: integ += e>>>KI_SH ; ctrl = (e>>>KP_SH)+integ ;
//     mu_step = NOM_STEP + TED_SIGN*ctrl  (clamped near nominal).
//   - Output = on-time interpolant yon (the recovered symbol).
//
// CALIBRATION (cannot be tuned without a timing-offset/full-chain lock test):
//   KP_SH, KI_SH (loop gains) and TED_SIGN (feedback polarity). Defaults are
//   conservative; adjust at integration so the loop is stable negative feedback.
//=============================================================================
module gardner #(
    parameter integer SPS      = 4,
    parameter signed [15:0] NOM_STEP = 16'sd4096,   // 1/SPS = 0.25 in Q2.14
    parameter integer KP_SH    = 20,                // proportional gain shift
    parameter integer KI_SH    = 26,                // integral gain shift
    parameter integer TED_SIGN = 1,                 // +1 or -1 (feedback polarity)
    parameter signed [15:0] STEP_LIM = 16'sd512     // mu_step clamp around nominal
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
    output reg  signed [15:0] mu_step_dbg          // observe loop (Q2.14)
);
    // input handshake: stall only if an unconsumed symbol is pending
    wire ce = !(m_axis_tvalid && !m_axis_tready);
    assign s_axis_tready = ce;
    wire fire = s_axis_tvalid && ce;

    wire signed [15:0] x_re = s_axis_tdata[15:0];
    wire signed [15:0] x_im = s_axis_tdata[31:16];

    // delay line (need >=5 deep: on-time uses d[1],d[2]; mid uses d[3],d[4])
    localparam integer L = 6;
    reg signed [15:0] dr [0:L-1];
    reg signed [15:0] di [0:L-1];
    integer i;

    // NCO + loop-filter state (all SIGNED arithmetic)
    reg signed [15:0] nco;          // Q2.14
    reg signed [15:0] mu_step;      // Q2.14
    reg signed [39:0] integ;        // loop-filter integrator (wide)
    reg signed [15:0] yon_re_prev, yon_im_prev;

    // saturating helpers
    function signed [15:0] sat16;
        input signed [39:0] v;
        begin
            if (v >  40'sd32767)      sat16 =  16'sd32767;
            else if (v < -40'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction
    // linear interp: a + mu*(b-a), mu Q2.14 in [0,1), a/b Q5.10
    function signed [15:0] interp;
        input signed [15:0] a, b, mu;
        reg signed [16:0] diff;
        reg signed [33:0] term;
        begin
            diff = b - a;
            term = ($signed({1'b0,mu}) * diff) >>> 14;   // Q5.10
            interp = sat16({{6{term[33]}},term} + a);
        end
    endfunction

    reg strobe;
    reg signed [15:0] mu;
    reg signed [15:0] yon_re, yon_im, ymid_re, ymid_im;
    reg signed [39:0] ted_e;
    reg signed [39:0] ctrl;
    reg signed [16:0] nco_dec;
    reg signed [16:0] step_calc;

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            for(i=0;i<L;i=i+1) begin dr[i]<=0; di[i]<=0; end
            nco<=16'sd0; mu_step<=NOM_STEP; integ<=0;
            yon_re_prev<=0; yon_im_prev<=0; mu_step_dbg<=NOM_STEP;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(m_axis_tvalid && m_axis_tready) m_axis_tvalid<=1'b0;
            if(fire) begin
                // shift delay line
                for(i=L-1;i>0;i=i-1) begin dr[i]<=dr[i-1]; di[i]<=di[i-1]; end
                dr[0]<=x_re; di[0]<=x_im;

                // NCO decrement / underflow detect
                nco_dec = nco - mu_step;
                if(nco_dec < 0) begin
                    strobe = 1'b1;
                    mu     = (nco <<< 2);            // ~ nco_old/mu_step (SPS=4)
                    if(mu > 16'sd16383) mu = 16'sd16383;
                    if(mu < 0)          mu = 16'sd0;
                    nco   <= nco_dec + 16'sd16384;   // + 1.0 (Q2.14)
                end else begin
                    strobe = 1'b0;
                    nco   <= nco_dec[15:0];
                end

                if(strobe) begin
                    // interpolate on-time (d[1],d[2]) and mid-point (d[3],d[4])
                    yon_re  = interp(dr[2], dr[1], mu);
                    yon_im  = interp(di[2], di[1], mu);
                    ymid_re = interp(dr[4], dr[3], mu);
                    ymid_im = interp(di[4], di[3], mu);

                    // Gardner TED (decision-free)
                    ted_e = ((yon_re - yon_re_prev) * ymid_re)
                          + ((yon_im - yon_im_prev) * ymid_im);

                    // PI loop filter (signed throughout)
                    integ <= integ + (ted_e >>> KI_SH);
                    ctrl  = (ted_e >>> KP_SH) + integ;
                    // mu_step = nominal +/- ctrl, clamped near nominal
                    if (TED_SIGN > 0) step_calc = NOM_STEP + ctrl[15:0];
                    else              step_calc = NOM_STEP - ctrl[15:0];
                    if (step_calc > NOM_STEP + STEP_LIM) mu_step <= NOM_STEP + STEP_LIM;
                    else if (step_calc < NOM_STEP - STEP_LIM) mu_step <= NOM_STEP - STEP_LIM;
                    else mu_step <= step_calc[15:0];
                    mu_step_dbg <= mu_step;

                    yon_re_prev <= yon_re; yon_im_prev <= yon_im;

                    // emit recovered symbol
                    m_axis_tdata  <= {yon_im, yon_re};
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= s_axis_tlast;
                end
            end
        end
    end
endmodule
