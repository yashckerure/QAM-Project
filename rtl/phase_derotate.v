`timescale 1ns/1ps
//=============================================================================
// phase_derotate : resolves the QPSK Costas 4-fold (90 deg) ambiguity.
//   Sits AFTER costas. On 'sop' it correlates the next PRE_LEN recovered
//   symbols against a known constant QPSK preamble (+KNI,+KNQ), decides the
//   rotation k in {0,1,2,3} (x90 deg), consumes the preamble (not output),
//   then de-rotates every payload symbol by e^{-j k 90} (sign/swap only).
//   32-bit {Q,I} Q5.10 AXIS. A real preamble would be a PN sequence; the
//   correlation generalizes (multiply by known signs) - logic is identical.
//=============================================================================
module phase_derotate #(parameter integer PRE_LEN=64,
                        parameter signed [15:0] KNI=16'sd724, parameter signed [15:0] KNQ=16'sd724)(
    input  wire aclk, input wire aresetn, input wire sop,
    input  wire [31:0] s_axis_tdata, input wire s_axis_tvalid, output wire s_axis_tready, input wire s_axis_tlast,
    output reg  [31:0] m_axis_tdata, output reg m_axis_tvalid, input wire m_axis_tready, output reg m_axis_tlast,
    output reg [1:0] k_dbg
);
    localparam IDLE=2'd0, PRE=2'd1, PAY=2'd2;
    reg [1:0] state;
    reg [15:0] cnt;
    reg signed [31:0] acc_re, acc_im;     // correlation accumulators
    reg [1:0] k;
    wire signed [15:0] rI = s_axis_tdata[15:0];
    wire signed [15:0] rQ = s_axis_tdata[31:16];
    // constant preamble (KNI=KNQ>0): D_re ~ sum(rI+rQ), D_im ~ sum(rQ-rI)
    wire signed [31:0] dre = rI + rQ;
    wire signed [31:0] dim = rQ - rI;
    wire out_fire = m_axis_tvalid && m_axis_tready;
    assign s_axis_tready = (state==PRE) ? 1'b1 : (!m_axis_tvalid || m_axis_tready);
    wire in_fire = s_axis_tvalid && s_axis_tready;

    // de-rotate payload by k
    reg signed [15:0] oI, oQ;
    always @(*) begin
        case(k)
            2'd0: begin oI= rI; oQ= rQ; end          // x1
            2'd1: begin oI= rQ; oQ=-rI; end          // x(-j)  undo +90
            2'd2: begin oI=-rI; oQ=-rQ; end          // x(-1)
            default: begin oI=-rQ; oQ= rI; end        // x(+j)  undo +270
        endcase
    end

    function [1:0] decide; input signed [31:0] re; input signed [31:0] im;
        reg signed [31:0] are, aim; begin
            are = (re<0)?-re:re; aim = (im<0)?-im:im;
            if(are>=aim) decide = (re>=0)?2'd0:2'd2;
            else         decide = (im>=0)?2'd1:2'd3;
        end
    endfunction

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=IDLE; cnt<=0; acc_re<=0; acc_im<=0; k<=0; k_dbg<=0;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(out_fire) begin m_axis_tvalid<=0; m_axis_tlast<=0; end
            if(in_fire) begin
                if(sop) begin                       // first preamble symbol (accumulate it)
                    acc_re <= dre; acc_im <= dim; cnt <= 16'd1; state <= PRE;
                end else case(state)
                    PRE: begin
                        acc_re <= acc_re + dre; acc_im <= acc_im + dim;
                        if(cnt==PRE_LEN-1) begin
                            k     <= decide(acc_re+dre, acc_im+dim);
                            k_dbg <= decide(acc_re+dre, acc_im+dim);
                            state <= PAY;
                        end else cnt <= cnt+1'b1;
                    end
                    PAY: begin
                        m_axis_tdata <= {oQ,oI}; m_axis_tvalid<=1'b1; m_axis_tlast<=s_axis_tlast;
                    end
                    default: ;
                endcase
            end else if(sop) begin                  // sop with no symbol: arm PRE
                state<=PRE; cnt<=0; acc_re<=0; acc_im<=0;
            end
        end
    end
endmodule
