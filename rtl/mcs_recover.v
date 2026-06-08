`timescale 1ns / 1ps
//=============================================================================
// File        : mcs_recover.v
// Description : Reads the fixed-QPSK MCS header at the start of each packet,
//               recovers the MCS (majority-of-3), strips the header, and passes
//               the payload symbols through. Sits on RX after the equalizer,
//               before soft_demapper. Outputs the per-packet demod config.
//=============================================================================
// Mirrors mcs_insert: N_HDR=9 header symbols, bit b = (I<0), majority-of-3 per
// MCS bit. Outputs mcs + derived qam_mode/E/n (E/Qm) so the RX demod/decode
// chain is configured for THIS packet. Framing via tlast (real link: add sync).
//=============================================================================
module mcs_recover #(
    parameter integer N_HDR = 9
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [31:0] m_axis_tdata,        // payload symbols only
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg  [2:0]  mcs,
    output reg  [2:0]  qam_mode,
    output reg  [12:0] e_out,
    output reg  [12:0] n_out,
    output reg         mcs_valid            // pulses when a new MCS is decoded
);
    localparam S_HDR=1'b0, S_PAY=1'b1;
    reg state;
    reg [3:0] hdr_cnt;
    reg [1:0] v2, v1, v0;                   // 1-vote counts per mcs bit (0..3)

    wire s_fire = s_axis_tvalid && s_axis_tready;
    wire hdr_bit = s_axis_tdata[15];        // I<0 -> bit 1
    wire m_fire = m_axis_tvalid && m_axis_tready;

    // accept input in HDR always (consumed), in PAY when output free
    assign s_axis_tready = (state==S_HDR) ? 1'b1 : (!m_axis_tvalid || m_axis_tready);

    reg [2:0] mcs_d;
    reg [1:0] v2n, v1n, v0n;
    always @(*) begin
        // include the current symbol's vote so the last bit gets all 3 votes
        v2n = v2; v1n = v1; v0n = v0;
        case(hdr_cnt/3)
          0: if(hdr_bit) v2n = v2 + 2'd1;
          1: if(hdr_bit) v1n = v1 + 2'd1;
          default: if(hdr_bit) v0n = v0 + 2'd1;
        endcase
        mcs_d[2] = (v2n >= 2'd2);
        mcs_d[1] = (v1n >= 2'd2);
        mcs_d[0] = (v0n >= 2'd2);
    end

    task map_mcs(input [2:0] m);
        begin
            case(m)
              3'd0: begin qam_mode<=3'd0; e_out<=13'd1040; n_out<=13'd520; end
              3'd1: begin qam_mode<=3'd1; e_out<=13'd1040; n_out<=13'd260; end
              3'd2: begin qam_mode<=3'd1; e_out<=13'd624;  n_out<=13'd156; end
              3'd3: begin qam_mode<=3'd2; e_out<=13'd624;  n_out<=13'd104; end
              default: begin qam_mode<=3'd3; e_out<=13'd624; n_out<=13'd78; end
            endcase
        end
    endtask

    always @(posedge aclk or negedge aresetn) begin
        if(!aresetn) begin
            state<=S_HDR; hdr_cnt<=0; v2<=0; v1<=0; v0<=0;
            mcs<=0; qam_mode<=0; e_out<=13'd1040; n_out<=13'd520; mcs_valid<=0;
            m_axis_tdata<=0; m_axis_tvalid<=0; m_axis_tlast<=0;
        end else begin
            if(m_fire) begin m_axis_tvalid<=1'b0; m_axis_tlast<=1'b0; end
            mcs_valid<=1'b0;
            case(state)
            S_HDR: begin
                if(s_fire) begin
                    // accumulate vote for the bit this symbol carries
                    case(hdr_cnt/3)
                      0: if(hdr_bit) v2<=v2+1'b1;
                      1: if(hdr_bit) v1<=v1+1'b1;
                      default: if(hdr_bit) v0<=v0+1'b1;
                    endcase
                    if(hdr_cnt==N_HDR-1) begin
                        mcs <= mcs_d;          // majority result (v0 of last sym not yet added)
                        map_mcs(mcs_d);
                        mcs_valid <= 1'b1;
                        hdr_cnt<=0; state<=S_PAY;
                    end else hdr_cnt<=hdr_cnt+1'b1;
                end
            end
            S_PAY: begin
                if(s_fire) begin
                    m_axis_tdata  <= s_axis_tdata;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= s_axis_tlast;
                    if(s_axis_tlast) begin
                        state<=S_HDR; v2<=0; v1<=0; v0<=0;
                    end
                end
            end
            endcase
        end
    end
endmodule
