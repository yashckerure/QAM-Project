`timescale 1ns/1ps
// Tier-2 stage-2a ideal timing recovery: replaces gardner for the zero-impairment
// sanity run. Decimates the RRC sample stream 4->1 at sample PHASE, and drops the
// first DROP_SYM symbols (RRC TX+RX cascade group delay) so the first emitted
// symbol is the first real data symbol. 32-bit {Q,I} AXIS. No tlast (rebuilt later).
module ideal_decimator #(parameter integer SPS=4, parameter integer PHASE=0, parameter integer DROP_SYM=8)(
    input  wire        aclk, input wire aresetn,
    input  wire [31:0] s_axis_tdata, input wire s_axis_tvalid, output wire s_axis_tready, input wire s_axis_tlast,
    output reg  [31:0] m_axis_tdata, output reg m_axis_tvalid, input wire m_axis_tready, output reg m_axis_tlast
);
    reg [1:0]  ph;
    reg [15:0] dropcnt;
    wire keep    = (ph==PHASE[1:0]);
    wire is_emit = keep && (dropcnt>=DROP_SYM);
    assign s_axis_tready = is_emit ? (!m_axis_tvalid || m_axis_tready) : 1'b1;
    wire in_fire  = s_axis_tvalid && s_axis_tready;
    wire out_fire = m_axis_tvalid && m_axis_tready;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin ph<=0; dropcnt<=0; m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0; end
        else begin
            if(out_fire) begin m_axis_tvalid<=0; m_axis_tlast<=0; end
            if(in_fire) begin
                if(keep) begin
                    if(dropcnt<DROP_SYM) dropcnt<=dropcnt+1'b1;
                    else begin m_axis_tdata<=s_axis_tdata; m_axis_tvalid<=1'b1; end
                end
                ph <= (ph==SPS-1) ? 2'd0 : ph+2'd1;
            end
        end
    end
endmodule
