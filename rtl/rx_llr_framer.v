`timescale 1ns/1ps
// Integration block REQUIRED by the architecture: the symbol domain
// (symbol_packer/qam_mapper) drops packet framing, and soft_demapper only
// re-creates a PER-SYMBOL tlast. The bit_deinterleaver terminates its COLLECT
// on tlast, so it needs a PACKET-level tlast. This framer ignores the incoming
// per-symbol tlast and asserts tlast on the E_in-th LLR. Signed-LLR AXIS.
module rx_llr_framer #(parameter integer LLR_W=4, parameter integer ADDR_W=13)(
    input  wire                    aclk, input wire aresetn,
    input  wire [ADDR_W-1:0]       e_in,            // LLRs per packet (= E)
    input  wire signed [LLR_W-1:0] s_axis_tdata, input wire s_axis_tvalid,
    output wire                    s_axis_tready,    input wire s_axis_tlast, // ignored
    output reg  signed [LLR_W-1:0] m_axis_tdata, output reg m_axis_tvalid,
    input  wire                    m_axis_tready,    output reg m_axis_tlast
);
    reg [ADDR_W-1:0] cnt;
    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;
    wire in_fire  = s_axis_tvalid && s_axis_tready;
    wire out_fire = m_axis_tvalid && m_axis_tready;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin cnt<=0; m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0; end
        else begin
            if(out_fire) begin m_axis_tvalid<=0; m_axis_tlast<=0; end
            if(in_fire) begin
                m_axis_tdata<=s_axis_tdata; m_axis_tvalid<=1'b1;
                if(cnt==e_in-1'b1) begin m_axis_tlast<=1'b1; cnt<=0; end
                else begin m_axis_tlast<=1'b0; cnt<=cnt+1'b1; end
            end
        end
    end
endmodule
