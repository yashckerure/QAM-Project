`timescale 1ns/1ps
// Integration helper: stands in for the MAC delivering transport blocks.
// Counts INFO_LEN accepted bits from bit_source and asserts tlast on the last,
// giving crc24a_attach its packet boundary. 1-bit AXIS, single-register stage.
module tx_payload_framer #(parameter integer INFO_LEN=496, parameter integer CW=13)(
    input  wire aclk, input wire aresetn,
    input  wire s_axis_tdata, input wire s_axis_tvalid, output wire s_axis_tready,
    output reg  m_axis_tdata, output reg m_axis_tvalid, input wire m_axis_tready, output reg m_axis_tlast
);
    reg [CW-1:0] cnt;
    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;
    wire in_fire  = s_axis_tvalid && s_axis_tready;
    wire out_fire = m_axis_tvalid && m_axis_tready;
    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin cnt<=0; m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0; end
        else begin
            if(out_fire) begin m_axis_tvalid<=0; m_axis_tlast<=0; end
            if(in_fire) begin
                m_axis_tdata<=s_axis_tdata; m_axis_tvalid<=1'b1;
                if(cnt==INFO_LEN-1) begin m_axis_tlast<=1'b1; cnt<=0; end
                else begin m_axis_tlast<=1'b0; cnt<=cnt+1'b1; end
            end
        end
    end
endmodule
