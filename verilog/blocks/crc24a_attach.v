`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 00:04:50
// Design Name: 
// Module Name: crc24a_attach
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//=============================================================================
// Project      : Adaptive QAM Modem
// File         : crc24a_attach.v
// Description  : CRC-24A attach per TS 38.212 Section 5.1.
//=============================================================================
// Additional Notes:
// - Polynomial g_CRC24A(D) = D^24 + D^23 + D^18 + D^17 + D^14 + D^11 + D^10
//                          + D^7 + D^6 + D^5 + D^4 + D^3 + D + 1
// - 24-bit polynomial (D^24 implicit): 0x864CFB
// - Computes CRC serially as input bits stream in.
// - After s_axis_tlast on the final input bit, holds s_axis_tready low for
//   24 clocks while streaming out the CRC (MSB-first). m_axis_tlast asserts
//   on the 24th CRC bit.
//=============================================================================
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : crc24a_attach.v
// Description  : CRC-24A attach per TS 38.212 Section 5.1.
//                Registered AXI outputs to avoid combinational output races.
//=============================================================================
`timescale 1ns / 1ps

module crc24a_attach (
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    output reg         m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);

    localparam [23:0] CRC_POLY = 24'h864CFB;

    localparam [1:0] S_PASSING = 2'd0;
    localparam [1:0] S_APPEND  = 2'd1;

    reg [1:0]  state;
    reg [23:0] crc_reg;
    reg [4:0]  crc_count;

    // Input side: accept whenever output side has room or is in PASSING
    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // CRC step
    wire feedback = crc_reg[23] ^ s_axis_tdata;
    wire [23:0] crc_next = feedback ? ((crc_reg << 1) ^ CRC_POLY) : (crc_reg << 1);

    // -------------------------------------------------------------------------
    // Backpressure: ready only when downstream can accept and we're in PASSING
    // -------------------------------------------------------------------------
    assign s_axis_tready = (state == S_PASSING) && (!m_axis_tvalid || m_axis_tready);

    // -------------------------------------------------------------------------
    // Main FSM and output registers
    // -------------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state         <= S_PASSING;
            crc_reg       <= 24'd0;
            crc_count     <= 5'd0;
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin

            // Default: clear valid when consumed
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            case (state)
                // -----------------------------------------------------------
                // PASSING: register each input bit to output, compute CRC
                // -----------------------------------------------------------
                S_PASSING: begin
                    if (input_fire) begin
                        m_axis_tdata  <= s_axis_tdata;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b0;
                        crc_reg       <= crc_next;
                        if (s_axis_tlast) begin
                            state     <= S_APPEND;
                            crc_count <= 5'd0;
                        end
                    end
                end

                // -----------------------------------------------------------
                // APPEND: emit 24 CRC bits, MSB-first
                // -----------------------------------------------------------
                S_APPEND: begin
                    if (!m_axis_tvalid || m_axis_tready) begin
                        m_axis_tdata  <= crc_reg[23];
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (crc_count == 5'd23);
                        crc_reg       <= crc_reg << 1;
                        if (crc_count == 5'd23) begin
                            state     <= S_PASSING;
                            crc_count <= 5'd0;
                            crc_reg   <= 24'd0;
                        end else begin
                            crc_count <= crc_count + 5'd1;
                        end
                    end
                end

                default: state <= S_PASSING;
            endcase
        end
    end

endmodule