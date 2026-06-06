`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 20:10:05
// Design Name: 
// Module Name: crc24a_check
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
// File         : crc24a_check.v
// Description  : CRC-24A check per TS 38.212 Section 5.1 (RX-side, inverse
//                of crc24a_attach).
//=============================================================================
// Additional Notes:
// - Polynomial g_CRC24A(D) = D^24 + D^23 + D^18 + D^17 + D^14 + D^11 + D^10
//                          + D^7 + D^6 + D^5 + D^4 + D^3 + D + 1
// - 24-bit polynomial (D^24 implicit): 0x864CFB
// - Input stream contains B info bits followed by 24 CRC bits.
// - CRC is computed over ALL input bits (info + CRC). If clean, the resulting
//   CRC register is zero.
// - Output stream contains only the B info bits (CRC bits stripped).
// - Implementation: 24-bit shift-register delay line. Each input bit goes into
//   the delay line; the bit that exits the delay line is emitted on the output.
//   When the input packet ends (tlast on the 24th CRC bit), the delay line
//   still holds the last 24 bits = the CRC bits, which are discarded.
// - crc_ok and crc_valid pulse for 1 clock after the last info bit is emitted.
// - Registered AXI-Stream outputs.
//=============================================================================
`timescale 1ns / 1ps

module crc24a_check (
    input  wire        aclk,
    input  wire        aresetn,

    input  wire        s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,

    output reg         m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,

    output reg         crc_ok,
    output reg         crc_valid
);

    localparam [23:0] CRC_POLY = 24'h864CFB;

    // -----------------------------------------------------------
    // 24-bit delay line (shift register).
    // Bit at delay_line[23] is the oldest, exits next.
    // -----------------------------------------------------------
    reg [23:0] delay_line;
    reg [4:0]  fill_count;       // Counts up to 24 (delay line filling)

    // CRC register (accumulates over every input bit)
    reg [23:0] crc_reg;

    wire feedback = crc_reg[23] ^ s_axis_tdata;
    wire [23:0] crc_next = feedback ? ((crc_reg << 1) ^ CRC_POLY)
                                    : (crc_reg << 1);

    // -----------------------------------------------------------
    // Handshake. Accept input whenever the output side can keep up.
    // -----------------------------------------------------------
    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // -----------------------------------------------------------
    // Sequential
    // -----------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            delay_line    <= 24'd0;
            fill_count    <= 5'd0;
            crc_reg       <= 24'd0;
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            crc_ok        <= 1'b0;
            crc_valid     <= 1'b0;
        end else begin
            // Default: 1-cycle strobe
            crc_valid <= 1'b0;

            // Clear valid when downstream consumes
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            if (input_fire) begin
                // Update CRC over EVERY input bit (info + CRC bits)
                crc_reg <= crc_next;

                // Push new bit into delay line at MSB end; oldest bit at
                // delay_line[0] is what gets emitted (when line is full)
                delay_line <= {s_axis_tdata, delay_line[23:1]};

                if (fill_count < 5'd24) begin
                    // Delay line still filling; do not emit
                    fill_count <= fill_count + 5'd1;
                end else begin
                    // Delay line full: emit the oldest bit (delay_line[0])
                    m_axis_tdata  <= delay_line[0];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= 1'b0;     // not the last info bit yet
                end

                if (s_axis_tlast) begin
                    // This is the 24th CRC bit. After this clock, the CRC
                    // register has accumulated over info + CRC, which equals
                    // zero if no errors.
                    //
                    // The bit just emitted on this clock IS the last info bit
                    // (it was delay_line[0]). Mark tlast on it.
                    m_axis_tlast <= 1'b1;

                    // Pulse the status flag using the NEXT crc_reg value
                    crc_ok    <= (crc_next == 24'd0);
                    crc_valid <= 1'b1;

                    // Reset for next packet
                    delay_line <= 24'd0;
                    fill_count <= 5'd0;
                    crc_reg    <= 24'd0;
                end
            end
        end
    end

endmodule
