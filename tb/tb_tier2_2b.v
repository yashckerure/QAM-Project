`timescale 1ns/1ps
//=============================================================================
// tb_tier2_2b : AWGN BER-vs-Eb/N0 sweep, fixed MCS0 (QPSK, R=1/2).
//   Reuses tier2_top_2a (front-end idealized). Resets per SNR point.
//   Calibration (verified): Es/N0 = 524288/noise_std^2 ; Eb/N0 = Es/N0/(Qm*R).
//   MCS0: Qm*R = 1, so Eb/N0(dB) = 10*log10(524288/noise_std^2).
//   For other MCS: noise_std = round(sqrt(524288/(10^(EbN0dB/10)*Qm*R))).
//   *** Use the regenerated bm_*.mem LUTs. ***
//   Runs slowly (iterative LDPC). Start with few points / NPKT, then extend.
//=============================================================================
module tb_tier2_2b;
    localparam integer NPKT=20, INFO=496;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn, enable; reg [15:0] nstd;
    wire [31:0] be,bc,pe,pk; wire done;

    tier2_top_2a #(.INFO_LEN(INFO), .DEC_PHASE(0), .DEC_DROP(8)) dut(
        .aclk(aclk), .aresetn(aresetn),
        .qam_mode(3'd0), .qm(4'd2), .n_sym(13'd520), .e_len(13'd1040), .rv(2'd0),
        .noise_std(nstd), .enable(enable), .num_bits_target(INFO*NPKT),
        .bit_errors(be), .bits_compared(bc), .packet_errors(pe), .packets(pk), .status_done(done));

    integer cyc; real ber, bler, ebn0;
    // noise_std for Eb/N0 = 0,1,2,3,4,5 dB (MCS0). Add/trim as needed.
    integer NPTS; reg [15:0] tbl [0:5];
    initial begin tbl[0]=724; tbl[1]=645; tbl[2]=575; tbl[3]=513; tbl[4]=457; tbl[5]=407; NPTS=6; end

    integer p;
    task run_point(input [15:0] ns);
    begin
        nstd=ns; aresetn=0; enable=0;
        repeat(8) @(posedge aclk); @(negedge aclk); aresetn=1; enable=1;
        cyc=0;
        while(!done && cyc<8000000) begin @(posedge aclk); cyc=cyc+1; end
        @(posedge aclk);
        ber  = (bc>0) ? be*1.0/bc : 0.0;
        bler = (pk>0) ? pe*1.0/pk : 0.0;
        ebn0 = 10.0*$log10(524288.0/(1.0*ns*ns));   // MCS0: Qm*R=1
        $display("noise_std=%0d  Eb/N0=%.2f dB | bits=%0d errs=%0d BER=%e | pkts=%0d perr=%0d BLER=%.3f",
                 ns, ebn0, bc, be, ber, pk, pe, bler);
    end endtask

    initial begin
        $display("=== Tier-2 2b AWGN sweep (MCS0 QPSK R=1/2, %0d pkts/pt) ===", NPKT);
        for(p=0;p<NPTS;p=p+1) run_point(tbl[p]);
        $display("=== sweep done ===");
        $finish;
    end
    initial begin #400000000; $display("WALL TIMEOUT"); $finish; end
endmodule
