`timescale 1ns/1ps
//=============================================================================
// tb_tier2_2a : XSim sanity run for stage 2a (zero noise, fixed MCS0).
//   Expected: BER=0, BLER=0  -> confirms real LDPC + rate-match + RRC + framing.
//   MCS0: QPSK, R=1/2, K=520, N=2600, E=1040, n=520 symbols/pkt, RV=0.
//   NOTE: runs slowly in XSim due to iterative LDPC decode; give it time.
//   If it deadlocks or BER~0.5, see the diagnosis order in the handoff notes.
//=============================================================================
module tb_tier2_2a;
    localparam integer NPKT=4, INFO=496;
    reg aclk=0; always #5 aclk=~aclk;           // 100 MHz
    reg aresetn, enable;
    wire [31:0] be,bc,pe,pk; wire done;

    tier2_top_2a #(.INFO_LEN(INFO), .DEC_PHASE(0), .DEC_DROP(8)) dut(
        .aclk(aclk), .aresetn(aresetn),
        .qam_mode(3'd0), .qm(4'd2), .n_sym(13'd520), .e_len(13'd1040), .rv(2'd0),
        .noise_std(16'd0),                       // zero noise
        .enable(enable), .num_bits_target(INFO*NPKT),
        .bit_errors(be), .bits_compared(bc), .packet_errors(pe), .packets(pk), .status_done(done));

    integer cyc;
    initial begin
        aresetn=0; enable=0;
        repeat(8) @(posedge aclk);
        @(negedge aclk); aresetn=1; enable=1;     // clean negedge deassert (no PRBS race)
        cyc=0;
        while(!done && cyc<5000000) begin
            @(posedge aclk); cyc=cyc+1;
            if((cyc % 100000)==0) $display("[cyc %0d] bits=%0d errs=%0d pkts=%0d", cyc, bc, be, pk);
        end
        @(posedge aclk);
        $display("=== Tier-2 2a DONE @cyc %0d ===", cyc);
        $display("bits_compared=%0d (target %0d)", bc, INFO*NPKT);
        $display("bit_errors=%0d  packets=%0d  packet_errors=%0d", be, pk, pe);
        if(be==0 && bc>=INFO*NPKT && pe==0 && pk>=NPKT)
             $display("PASS: full real chain lossless at zero noise");
        else $display("CHECK: see diagnosis order (phase ambiguity n/a here; suspect FEC framing/latency or decimation)");
        $finish;
    end
    initial begin #50000000; $display("WALL TIMEOUT"); $finish; end
endmodule
