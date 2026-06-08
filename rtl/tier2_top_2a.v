`timescale 1ns/1ps
//=============================================================================
// tier2_top_2a : full real chain, zero-impairment sanity (stage 2a).
//   - Real LDPC encoder/decoder, rate_match/dematch, RRC TX/RX, channel.
//   - Front-end loops (AGC/Costas/Gardner/equalizer) and MCS header OMITTED;
//     timing handled by ideal_decimator; channel noise_std=0, unit tap.
//   - Fixed MCS via top-level config inputs.
// Expected result at noise_std=0 : BER=0, BLER=0 (confirms FEC+RRC+framing).
//=============================================================================
module tier2_top_2a #(
    parameter integer INFO_LEN = 496,
    parameter integer DEC_PHASE = 0,    // RRC decimation phase (0..3) - calibrate
    parameter integer DEC_DROP  = 8     // RRC cascade group delay in symbols - calibrate
)(
    input  wire        aclk, input wire aresetn,
    input  wire [2:0]  qam_mode,        // fixed for 2a
    input  wire [3:0]  qm,              // bits/sym
    input  wire [12:0] n_sym,           // symbols/packet = E/Qm  (interleaver n_in)
    input  wire [12:0] e_len,           // E (rate-match out; framer/dematch)
    input  wire [1:0]  rv,
    input  wire [15:0] noise_std,       // 0 for 2a
    input  wire        enable,
    input  wire [31:0] num_bits_target,
    output wire [31:0] bit_errors, output wire [31:0] bits_compared,
    output wire [31:0] packet_errors, output wire [31:0] packets, output wire status_done
);
    // ---------- TX bit domain ----------
    wire bs_td,bs_tv,bs_tr;
    bit_source u_src(.aclk(aclk),.aresetn(aresetn),.m_axis_tready(bs_tr),.m_axis_tdata(bs_td),.m_axis_tvalid(bs_tv));
    wire pf_td,pf_tv,pf_tl,pf_tr;
    tx_payload_framer #(.INFO_LEN(INFO_LEN)) u_pf(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(bs_td),.s_axis_tvalid(bs_tv),.s_axis_tready(bs_tr),
        .m_axis_tdata(pf_td),.m_axis_tvalid(pf_tv),.m_axis_tready(pf_tr),.m_axis_tlast(pf_tl));
    wire ca_td,ca_tv,ca_tl,ca_tr;
    crc24a_attach u_crc(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(pf_td),.s_axis_tvalid(pf_tv),.s_axis_tready(pf_tr),.s_axis_tlast(pf_tl),
        .m_axis_tdata(ca_td),.m_axis_tvalid(ca_tv),.m_axis_tready(ca_tr),.m_axis_tlast(ca_tl));
    wire sg_td,sg_tv,sg_tl,sg_tr;
    code_block_seg #(.B(520)) u_seg(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(ca_td),.s_axis_tvalid(ca_tv),.s_axis_tready(ca_tr),.s_axis_tlast(ca_tl),
        .m_axis_tdata(sg_td),.m_axis_tvalid(sg_tv),.m_axis_tready(sg_tr),.m_axis_tlast(sg_tl));
    // real LDPC encoder (K=520 -> N=2600)
    wire en_td,en_tv,en_tl,en_tr;
    ldpc_encoder u_enc(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(sg_td),.s_axis_tvalid(sg_tv),.s_axis_tready(sg_tr),.s_axis_tlast(sg_tl),
        .m_axis_tdata(en_td),.m_axis_tvalid(en_tv),.m_axis_tready(en_tr),.m_axis_tlast(en_tl));
    wire rm_td,rm_tv,rm_tl,rm_tr;
    rate_match u_rm(.aclk(aclk),.aresetn(aresetn),.rv_in(rv),.e_in(e_len),
        .s_axis_tdata(en_td),.s_axis_tvalid(en_tv),.s_axis_tready(en_tr),.s_axis_tlast(en_tl),
        .m_axis_tdata(rm_td),.m_axis_tvalid(rm_tv),.m_axis_tready(rm_tr),.m_axis_tlast(rm_tl));
    wire il_td,il_tv,il_tl,il_tr;
    bit_interleaver u_il(.aclk(aclk),.aresetn(aresetn),.qm_in(qm),.n_in(n_sym),
        .s_axis_tdata(rm_td),.s_axis_tvalid(rm_tv),.s_axis_tready(rm_tr),.s_axis_tlast(rm_tl),
        .m_axis_tdata(il_td),.m_axis_tvalid(il_tv),.m_axis_tready(il_tr),.m_axis_tlast(il_tl));
    wire cc_td,cc_tv,cc_tl,cc_tr;
    code_block_concat u_cc(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(il_td),.s_axis_tvalid(il_tv),.s_axis_tready(il_tr),.s_axis_tlast(il_tl),
        .m_axis_tdata(cc_td),.m_axis_tvalid(cc_tv),.m_axis_tready(cc_tr),.m_axis_tlast(cc_tl));
    wire sc_td,sc_tv,sc_tl,sc_tr;
    scrambler u_scr(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(cc_td),.s_axis_tvalid(cc_tv),.s_axis_tready(cc_tr),.s_axis_tlast(cc_tl),
        .m_axis_tdata(sc_td),.m_axis_tvalid(sc_tv),.m_axis_tready(sc_tr),.m_axis_tlast(sc_tl));
    // ---------- symbol domain ----------
    wire [7:0] sp_td; wire [3:0] sp_tu; wire sp_tv,sp_tr;
    symbol_packer u_pk(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),.sym_en(1'b1),
        .s_axis_tdata(sc_td),.s_axis_tvalid(sc_tv),.s_axis_tready(sc_tr),
        .m_axis_tdata(sp_td),.m_axis_tuser(sp_tu),.m_axis_tvalid(sp_tv),.m_axis_tready(sp_tr));
    wire [31:0] mp_td; wire mp_tv,mp_tr;
    qam_mapper u_map(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .s_axis_tdata(sp_td),.s_axis_tuser(sp_tu),.s_axis_tvalid(sp_tv),.s_axis_tready(sp_tr),
        .m_axis_tdata(mp_td),.m_axis_tvalid(mp_tv),.m_axis_tready(mp_tr));
    // ---------- pulse shaping + channel (sample domain) ----------
    wire [31:0] rt_td; wire rt_tv,rt_tr;
    rrc_tx u_rt(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(mp_td),.s_axis_tvalid(mp_tv),.s_axis_tready(mp_tr),
        .m_axis_tdata(rt_td),.m_axis_tvalid(rt_tv),.m_axis_tready(rt_tr));
    wire [31:0] ch_td; wire ch_tv,ch_tr,ch_tl;
    channel u_ch(.aclk(aclk),.aresetn(aresetn),.noise_std(noise_std),
        .tap_we(1'b0),.tap_idx(3'd0),.tap_re(16'sd0),.tap_im(16'sd0),
        .s_axis_tdata(rt_td),.s_axis_tvalid(rt_tv),.s_axis_tready(rt_tr),.s_axis_tlast(1'b0),
        .m_axis_tdata(ch_td),.m_axis_tvalid(ch_tv),.m_axis_tready(ch_tr),.m_axis_tlast(ch_tl));
    wire [31:0] rr_td; wire rr_tv,rr_tr;
    rrc_rx u_rr(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(ch_td),.s_axis_tvalid(ch_tv),.s_axis_tready(ch_tr),
        .m_axis_tdata(rr_td),.m_axis_tvalid(rr_tv),.m_axis_tready(rr_tr));
    wire [31:0] dc_td; wire dc_tv,dc_tr;
    ideal_decimator #(.PHASE(DEC_PHASE),.DROP_SYM(DEC_DROP)) u_dc(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(rr_td),.s_axis_tvalid(rr_tv),.s_axis_tready(rr_tr),.s_axis_tlast(1'b0),
        .m_axis_tdata(dc_td),.m_axis_tvalid(dc_tv),.m_axis_tready(dc_tr),.m_axis_tlast());
    // ---------- RX: demap + framing + de-FEC ----------
    wire signed [3:0] dm_td; wire dm_tv,dm_tr,dm_tl;
    soft_demapper u_dem(.aclk(aclk),.aresetn(aresetn),.qam_mode(qam_mode),
        .s_axis_tdata(dc_td),.s_axis_tvalid(dc_tv),.s_axis_tready(dc_tr),
        .m_axis_tdata(dm_td),.m_axis_tvalid(dm_tv),.m_axis_tready(dm_tr),.m_axis_tlast(dm_tl));
    wire signed [3:0] fr_td; wire fr_tv,fr_tr,fr_tl;
    rx_llr_framer u_fr(.aclk(aclk),.aresetn(aresetn),.e_in(e_len),
        .s_axis_tdata(dm_td),.s_axis_tvalid(dm_tv),.s_axis_tready(dm_tr),.s_axis_tlast(dm_tl),
        .m_axis_tdata(fr_td),.m_axis_tvalid(fr_tv),.m_axis_tready(fr_tr),.m_axis_tlast(fr_tl));
    wire signed [3:0] ds_td; wire ds_tv,ds_tr,ds_tl;
    llr_descrambler u_des(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(fr_td),.s_axis_tvalid(fr_tv),.s_axis_tready(fr_tr),.s_axis_tlast(fr_tl),
        .m_axis_tdata(ds_td),.m_axis_tvalid(ds_tv),.m_axis_tready(ds_tr),.m_axis_tlast(ds_tl));
    wire signed [3:0] dcc_td; wire dcc_tv,dcc_tr,dcc_tl;
    code_block_de_concat u_dcc(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(ds_td),.s_axis_tvalid(ds_tv),.s_axis_tready(ds_tr),.s_axis_tlast(ds_tl),
        .m_axis_tdata(dcc_td),.m_axis_tvalid(dcc_tv),.m_axis_tready(dcc_tr),.m_axis_tlast(dcc_tl));
    wire signed [3:0] di_td; wire di_tv,di_tr,di_tl;
    bit_deinterleaver #(.LLR_W(4)) u_di(.aclk(aclk),.aresetn(aresetn),.qm_in(qm),.n_in(n_sym),
        .s_axis_tdata(dcc_td),.s_axis_tvalid(dcc_tv),.s_axis_tready(dcc_tr),.s_axis_tlast(dcc_tl),
        .m_axis_tdata(di_td),.m_axis_tvalid(di_tv),.m_axis_tready(di_tr),.m_axis_tlast(di_tl));
    wire signed [3:0] rd_td; wire rd_tv,rd_tr,rd_tl;
    rate_dematch u_rd(.aclk(aclk),.aresetn(aresetn),.rv_in(rv),.e_in(e_len),
        .s_axis_tdata(di_td),.s_axis_tvalid(di_tv),.s_axis_tready(di_tr),.s_axis_tlast(di_tl),
        .m_axis_tdata(rd_td),.m_axis_tvalid(rd_tv),.m_axis_tready(rd_tr),.m_axis_tlast(rd_tl));
    wire dec_td,dec_tv,dec_tr,dec_tl;
    ldpc_decoder u_dec(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(rd_td),.s_axis_tvalid(rd_tv),.s_axis_tready(rd_tr),.s_axis_tlast(rd_tl),
        .m_axis_tdata(dec_td),.m_axis_tvalid(dec_tv),.m_axis_tready(dec_tr),.m_axis_tlast(dec_tl));
    wire dseg_td,dseg_tv,dseg_tr,dseg_tl;
    code_block_deseg #(.B(520)) u_dseg(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(dec_td),.s_axis_tvalid(dec_tv),.s_axis_tready(dec_tr),.s_axis_tlast(dec_tl),
        .m_axis_tdata(dseg_td),.m_axis_tvalid(dseg_tv),.m_axis_tready(dseg_tr),.m_axis_tlast(dseg_tl));
    wire ck_td,ck_tv,ck_tr,ck_tl,crc_ok,crc_valid;
    crc24a_check u_ck(.aclk(aclk),.aresetn(aresetn),
        .s_axis_tdata(dseg_td),.s_axis_tvalid(dseg_tv),.s_axis_tready(dseg_tr),.s_axis_tlast(dseg_tl),
        .m_axis_tdata(ck_td),.m_axis_tvalid(ck_tv),.m_axis_tready(ck_tr),.m_axis_tlast(ck_tl),
        .crc_ok(crc_ok),.crc_valid(crc_valid));
    ber_counter u_ber(.aclk(aclk),.aresetn(aresetn),.enable(enable),.num_bits_target(num_bits_target),
        .s_axis_tdata(ck_td),.s_axis_tvalid(ck_tv),.s_axis_tready(ck_tr),.s_axis_tlast(ck_tl),
        .crc_ok(crc_ok),.crc_valid(crc_valid),
        .bit_errors(bit_errors),.bits_compared(bits_compared),
        .packet_errors(packet_errors),.packets(packets),.status_done(status_done));
endmodule
