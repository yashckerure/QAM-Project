`timescale 1ns/1ps
module tb_ber;
    localparam integer PKT=496, NPKT=5, NTOT=PKT*NPKT;
    reg aclk=0; always #5 aclk=~aclk;
    reg aresetn, enable; reg [31:0] num_bits_target;
    reg s_td, s_tv, s_tl, crc_ok, crc_valid; wire s_tr;
    wire [31:0] be, bc, pe, pk; wire done;

    ber_counter dut(.aclk(aclk),.aresetn(aresetn),.enable(enable),
        .num_bits_target(num_bits_target),
        .s_axis_tdata(s_td),.s_axis_tvalid(s_tv),.s_axis_tready(s_tr),.s_axis_tlast(s_tl),
        .crc_ok(crc_ok),.crc_valid(crc_valid),
        .bit_errors(be),.bits_compared(bc),.packet_errors(pe),.packets(pk),.status_done(done));

    reg [22:0] lf; wire pbit = lf[22]; wire fb = lf[22]^lf[17];
    integer g, inj_be, inj_pe; reg is_last;

    initial begin
        lf=23'h5A3C7E; inj_be=0; inj_pe=0;
        aresetn=0; enable=0; num_bits_target=32'hFFFFFFFF;
        s_td=0; s_tv=0; s_tl=0; crc_ok=1; crc_valid=0;
        #40; @(posedge aclk); aresetn=1; enable=1; @(posedge aclk);

        for(g=0; g<NTOT; g=g+1) begin
            @(negedge aclk);
            is_last = ((g % PKT)==PKT-1);
            // 3 injected bit errors at global positions 10, 600, 1500
            if(g==10||g==600||g==1500) begin s_td=~pbit; inj_be=inj_be+1; end
            else s_td=pbit;
            s_tv=1; s_tl=is_last;
            if(is_last) begin
                crc_valid=1; crc_ok=((g/PKT)!=2);          // packet index 2 fails CRC
                if((g/PKT)==2) inj_pe=inj_pe+1;
            end else crc_valid=0;
            @(posedge aclk);
            lf = {lf[21:0], fb};                            // advance PRBS, mirrors DUT
        end
        @(negedge aclk); s_tv=0; s_tl=0; crc_valid=0;
        @(posedge aclk);

        $display("bits_compared=%0d (expect %0d)", bc, NTOT);
        $display("bit_errors=%0d (expect %0d)", be, inj_be);
        $display("packets=%0d (expect %0d)  packet_errors=%0d (expect %0d)", pk, NPKT, pe, inj_pe);
        if(bc==NTOT && be==inj_be && pk==NPKT && pe==inj_pe)
             $display("PASS: ber_counter BER + BLER correct (PRBS aligned)");
        else $display("FAIL");
        $finish;
    end
    initial begin #2000000; $display("TIMEOUT"); $finish; end
endmodule
