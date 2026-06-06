`timescale 1ns / 1ps
//=============================================================================
// File        : rrc_rx.v
// Description : Matched root-raised-cosine RX filter. Full-rate 33-tap complex
//               FIR (same taps as rrc_tx). 1 complex sample (Q5.10) in -> 1
//               complex sample (Q5.10) out. Downsample by SPS=4 is performed
//               downstream by timing recovery (Gardner), not here.
//=============================================================================
// Matched filter: real symmetric RRC -> taps == TX taps. acc(40b)=sum coef*data;
// >>>14; saturate Q5.10. Group delay = (33-1)/2 = 16 samples. The TX(x)RX cascade
// is a full raised cosine -> Nyquist zero-ISI at symbol instants (verified).
//=============================================================================
module rrc_rx #(parameter integer DW=16)(
    input  wire             aclk,
    input  wire             aresetn,
    input  wire [2*DW-1:0]  s_axis_tdata,    // {Q,I} Q5.10 sample
    input  wire             s_axis_tvalid,
    output wire             s_axis_tready,
    output reg  [2*DW-1:0]  m_axis_tdata,    // {Q,I} Q5.10 filtered sample
    output reg              m_axis_tvalid,
    input  wire             m_axis_tready
);
    localparam integer NTAP = 33;

    function signed [DW-1:0] coef(input integer k);
        begin
            case(k)
              0: coef=-16'sd83;
              1: coef=-16'sd31;
              2: coef=16'sd88;
              3: coef=16'sd135;
              4: coef=16'sd25;
              5: coef=-16'sd135;
              6: coef=-16'sd123;
              7: coef=16'sd127;
              8: coef=16'sd348;
              9: coef=16'sd127;
              10: coef=-16'sd615;
              11: coef=-16'sd1285;
              12: coef=-16'sd869;
              13: coef=16'sd1285;
              14: coef=16'sd4740;
              15: coef=16'sd7984;
              16: coef=16'sd9312;
              17: coef=16'sd7984;
              18: coef=16'sd4740;
              19: coef=16'sd1285;
              20: coef=-16'sd869;
              21: coef=-16'sd1285;
              22: coef=-16'sd615;
              23: coef=16'sd127;
              24: coef=16'sd348;
              25: coef=16'sd127;
              26: coef=-16'sd123;
              27: coef=-16'sd135;
              28: coef=16'sd25;
              29: coef=16'sd135;
              30: coef=16'sd88;
              31: coef=-16'sd31;
              32: coef=-16'sd83;
              default: coef=16'sd0;
            endcase
        end
    endfunction

    wire signed [DW-1:0] in_i = s_axis_tdata[DW-1:0];
    wire signed [DW-1:0] in_q = s_axis_tdata[2*DW-1:DW];

    // sample history x[n-1 .. n-32]
    reg signed [DW-1:0] sri [0:NTAP-2];
    reg signed [DW-1:0] srq [0:NTAP-2];

    function signed [DW-1:0] sat(input signed [39:0] a);
        reg signed [39:0] t; begin
            t=a>>>14;
            if(t>40'sd32767) sat=16'sd32767;
            else if(t<-40'sd32768) sat=-16'sd32768;
            else sat=t[DW-1:0];
        end
    endfunction

    // combinational FIR over {in, history}
    integer k;
    reg signed [39:0] acci, accq;
    always @(*) begin
        acci = coef(0)*in_i;
        accq = coef(0)*in_q;
        for(k=1;k<NTAP;k=k+1) begin
            acci = acci + coef(k)*sri[k-1];
            accq = accq + coef(k)*srq[k-1];
        end
    end

    wire out_ready = !m_axis_tvalid || m_axis_tready;
    assign s_axis_tready = out_ready;
    wire input_fire = s_axis_tvalid && s_axis_tready;

    integer j;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            m_axis_tdata<=0; m_axis_tvalid<=1'b0;
            for(j=0;j<NTAP-1;j=j+1) begin sri[j]<=0; srq[j]<=0; end
        end else begin
            if(out_ready) begin
                m_axis_tvalid <= s_axis_tvalid;
                if(s_axis_tvalid) begin
                    m_axis_tdata <= {sat(accq), sat(acci)};
                    // shift history: newest = in
                    for(j=NTAP-2;j>0;j=j-1) begin sri[j]<=sri[j-1]; srq[j]<=srq[j-1]; end
                    sri[0]<=in_i; srq[0]<=in_q;
                end
            end
        end
    end
endmodule
