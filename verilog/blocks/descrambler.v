`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2026 20:03:05
// Design Name: 
// Module Name: descrambler
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
// File         : descrambler.v
// Description  : PDSCH descrambler (inverse of scrambler, TS 38.211 Sec 7.3.1.1).
//                Structurally identical to the scrambler since XOR is self-
//                inverse. Uses the same length-31 Gold sequence and C_INIT.
//=============================================================================
// Additional Notes:
// - Gold seq: c(n) = (x1(n+Nc) + x2(n+Nc)) mod 2, Nc = 1600
// - x1(n+31) = (x1(n+3) + x1(n)) mod 2; x1(0)=1, x1(1..30)=0
// - x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2
// - x2 initialized from C_INIT (parameter): x2(n) = (C_INIT >> n) & 1
// - For correct end-to-end operation, the RX-side C_INIT must match TX-side.
// - Operation: descrambled_bit = scrambled_bit XOR c(n).
// - At reset, both LFSRs initialize and warmup runs for Nc=1600 cycles
//   before s_axis_tready is asserted.
// - Registered AXI-Stream outputs.
// - Single-packet per reset session.
//=============================================================================
`timescale 1ns / 1ps

module descrambler #(
    parameter [30:0] C_INIT = 31'h00008000     // must match TX scrambler
)(
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

    localparam integer NC = 1600;

    // -----------------------------------------------------------
    // LFSR state and feedback
    // -----------------------------------------------------------
    reg [30:0] x1;
    reg [30:0] x2;

    wire x1_fb   = x1[3] ^ x1[0];
    wire x2_fb   = x2[3] ^ x2[2] ^ x2[1] ^ x2[0];
    wire gold_b  = x1[0] ^ x2[0];

    // -----------------------------------------------------------
    // FSM
    // -----------------------------------------------------------
    localparam S_WARMUP  = 1'b0;
    localparam S_RUNNING = 1'b1;

    reg         state;
    reg  [10:0] warmup_count;

    assign s_axis_tready = (state == S_RUNNING) &&
                           (!m_axis_tvalid || m_axis_tready);

    wire input_fire  = s_axis_tvalid && s_axis_tready;
    wire output_fire = m_axis_tvalid && m_axis_tready;

    // -----------------------------------------------------------
    // Sequential
    // -----------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            x1            <= 31'h00000001;
            x2            <= C_INIT;
            warmup_count  <= 11'd0;
            state         <= S_WARMUP;
            m_axis_tdata  <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            if (output_fire) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            case (state)
                S_WARMUP: begin
                    x1 <= {x1_fb, x1[30:1]};
                    x2 <= {x2_fb, x2[30:1]};
                    if (warmup_count == NC - 1)
                        state <= S_RUNNING;
                    else
                        warmup_count <= warmup_count + 11'd1;
                end

                S_RUNNING: begin
                    if (input_fire) begin
                        m_axis_tdata  <= s_axis_tdata ^ gold_b;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= s_axis_tlast;
                        x1 <= {x1_fb, x1[30:1]};
                        x2 <= {x2_fb, x2[30:1]};
                    end
                end

                default: state <= S_WARMUP;
            endcase
        end
    end

endmodule
