# Adaptive QAM Modem on Pynq Z2 — Project Handoff (Team Reference)

> Authoritative project-state and integration document. Anyone picking up a block
> should read Sections 1–6 once (conventions + interfaces), then go to their block's
> spec in Section 8. This supersedes all earlier handoffs.

---

## 1. PROJECT OVERVIEW

A **5G NR-aligned adaptive M-QAM modem in Verilog** for the **Pynq Z2 board
(Xilinx Zynq XC7Z020)**. "Adaptive" = both **modulation order** and **code rate**
change at runtime based on channel quality. The design is a single-carrier baseband
loopback implementing the PDSCH bit/coding chain, pulse shaping, a synchronization
RX front-end, and a bounded channel.

**In scope:** single-carrier baseband; PDSCH bit/coding chain; QPSK/16/64/256-QAM;
adaptive modulation + code rate; LDPC (BG2 only); CRC; scrambling; rate matching;
code-block segmentation + concatenation (+ RX inverses); RRC pulse shaping;
synchronization front-end (AGC, carrier recovery, timing recovery, equalizer);
a bounded AWGN + multipath/fading/interference channel.

**Out of scope (do NOT build):** OFDM; MIMO; HARQ; PDCCH/PBCH/PUCCH (PDSCH only);
1024-QAM; LDPC BG1; OTFS (noted only as a known 6G research direction).

**Resource fit:** Pynq Z2 PL resource-fit constraints are **set aside for now** by
project direction — build and verify functionally first. The LDPC decoder alone
overflows the device (Section 9). This is a known, accepted, open item.

## 2. ENGINEERING CONVENTIONS (apply to every block)

1. **Verify, do not assume.** For any 5G spec value, MATLAB function/block name,
   toolbox capability, or FPGA fact: confirm against the actual spec/docs before
   coding. Several prior errors came from guessing toolbox behavior from memory.
2. **No test-rigging.** If a testbench fails, find the real bug. No calibration
   offsets, strobe nudges, or magic constants that pass a test but break on hardware.
3. **Golden-reference verification.** Every block is verified by dumping its output
   and comparing (diff / Windows `fc`, or in-TB compare) against an independent
   golden reference (MATLAB, Python, or a known-good block used as oracle). A block
   is not "done" until it matches bit/value-exact (or within a stated tolerance for
   DSP blocks).
4. **Complete files.** Deliver full `.v` files or clearly-marked single-section edits.
5. **Stable interfaces.** The locked parameters (Section 4) and AXI-Stream
   conventions (Section 3) are fixed. Flag explicitly before changing one.
6. **Toolchain:** Windows 11, Vivado (synthesis + XSim), MATLAB R2024b (5G Toolbox +
   Wireless HDL Toolbox + HDL Coder). Verilog-2001, ASCII only. Not Linux, not cocotb.

## 3. AXI-STREAM + DATAPATH CONVENTIONS

- **Handshake:** transfer on a clock edge only when `tvalid` AND `tready` are both
  high. Outputs are **registered**. `tlast` marks the last beat of a packet.
- **Clock/reset:** single `aclk` (100 MHz). `aresetn` async active-low.
  Idiom: `always @(posedge aclk or negedge aresetn)`.
- **Naming:** `s_axis_*` slave (input), `m_axis_*` master (output).
- **Data formats:**
  - **Bit-domain** blocks carry **1 bit** per beat on `tdata`.
  - **LLR-domain** (soft RX) blocks carry **4-bit signed** (`signed [3:0]`) per beat.
  - **Symbol/sample-domain** blocks carry complex `{Q[15:0], I[15:0]}` (32-bit),
    each 16-bit **signed Q5.10** (value = stored/1024).
  - Coefficients **Q1.14** signed 16-bit (value = stored/16384).
  - MAC accumulators **40-bit signed**; product `>>>14`; saturate to Q5.10.
- **One element per beat.** Gearbox blocks (LDPC en/decoder, rate_match/dematch)
  internally convert between the serial stream and a wider buffered representation.

## 4. LOCKED ARCHITECTURE PARAMETERS

```
Clock                100 MHz, single domain
Reset                async active-low (aresetn)
SPS                  4 samples per symbol
Symbol rate          25 Msym/s
Data path            Q5.10 signed 16-bit I/Q
Coefficient path     Q1.14 signed 16-bit
Accumulator          40-bit signed; product >>>14; saturate to Q5.10
Modulation set       QPSK / 16 / 64 / 256-QAM   (Qm = 2/4/6/8; bits-per-axis = Qm/2)
LDPC base graph      BG2 only
LDPC lifting Zc      52   (FIXED project parameter -- see Section 8 compliance note)
Info block K         520 bits (= 10*Zc)
Mother codeword N    2600 bits (full lifted 52*Zc = 2704; first 2*Zc=104 punctured)
Code rates           1/2 (E=1040) and 5/6 (E=624).  E = K / R.   (see MCS table, Sec 9)
Redundancy version   RV = 0 (no HARQ)
LLR format           4-bit SIGNED (sfix4) -- system-wide lock (Section 5)
PRBS                 PRBS-23, poly x^23 + x^18 + 1, seed 0x5A3C7E
CRC-24A              0x864CFB  (transport-block CRC; init 0; no reflect; no final XOR; MSB-first)
CRC-24B              0x800063  (code-block CRC; attached ONLY when C>1; unused here, C=1)
Scrambler c_init     0x00008000; length-31 Gold sequence; Nc=1600 warmup
                     x1 init 0x1, x2 init c_init; x1_fb=x1[3]^x1[0];
                     x2_fb=x2[3]^x2[2]^x2[1]^x2[0]; gold=x1[0]^x2[0]; right-shift, fb->MSB
QAM scaling          724 / 324 / 158 / 79 = round(1/sqrt(2,10,42,170) * 1024)
RRC filter           root-raised-cosine, beta=0.5, span=8 symbols, 33 taps (Q1.14)
```

## 5. LLR FORMAT LOCK (read before building any soft RX block)

The LDPC **decoder core** was generated with **4-bit signed (sfix4)** LLR inputs —
a **system-wide constant**. Every soft RX block — `soft_demapper`, `llr_descrambler`,
`code_block_de_concat`, `bit_deinterleaver` (LLR version), `rate_dematch` — produces/
carries **4-bit signed** LLRs in this format. **Convention: positive LLR => bit 0,
negative => bit 1.** Changing the width later requires regenerating + re-verifying
the decoder. Where soft values must be negated (llr_descrambler, soft-combining in
rate_dematch), negation **saturates**: -(-8) clamps to +7.

## 6. MATLAB -> HDL WORKFLOW (for HDL-Coder-generated blocks: LDPC en/decoder)

- **Open the example:** `openExample('whdl/NRLDPCEncodeAndDecodeHDLExample')`.
- **bgn convention is INVERTED between function and block:** functions
  `nrLDPCEncode`/`nrLDPCDecode` use **bgn=2** for BG2; the Simulink BLOCK uses
  **bgn=1** for BG2 (0 = BG1). The wrappers tie the block's `bgn=1`. liftingSize=52.
- **Match reset:** generate with `'ResetType','async'`,
  `'ResetAssertedLevel','active-low'`.
- **Module-name collisions -> use `ModulePrefix`.** Each core re-emits `HDL_Algorithm`,
  `NR_LDPC_*`, `SimpleDualPortRAM_*` with the same names. Encoder core UNPREFIXED;
  decoder core regenerated with `'ModulePrefix','dec_'`. Any further generated core
  needs a unique prefix.
- **makehdltb stale-context fix:** one clean session — `bdclose all;` then
  `run('<setup>');` then `makehdl(...)` then `makehdltb(...)`.
- **XSim from Windows cmd** (in the folder with the `.v` + `.txt`):
  `dir /b *.v > files.prj` -> `xvlog -f files.prj` -> `xelab <tb_top> -s <snap>` ->
  `xsim <snap> -runall` -> `fc <out>.txt <ref>.txt`.
- **VERIFIED: there is NO MathWorks HDL rate-match block.** Wireless HDL Toolbox ships
  HDL blocks for LDPC encoder/decoder, CRC, Polar — but not rate matching.
  `nrRateMatchLDPC` is a behavioral 5G-Toolbox function (GPU-capable, no HDL path).
  So `rate_match`/`rate_dematch` are HAND-WRITTEN; golden reference = `nrRateMatchLDPC`
  / `nrRateRecoverLDPC`.

## 7. BLOCK INTERFACE REFERENCE (completed blocks)

All blocks share `aclk`, `aresetn`, standard `s_axis_*`/`m_axis_*` handshake. Only
data widths and extra signals are listed; exact port names are in each file header.

| Block | Slave in | Master out | Extra signals / notes |
|---|---|---|---|
| bit_source | (none) | 1-bit | PRBS-23 source |
| crc24a_attach | 1-bit (496) | 1-bit (520) | appends 24 CRC bits; tlast on last |
| code_block_seg | 1-bit (520) | 1-bit (520) | TS 38.212 5.2.2; C=1 pass-through (see compliance note) |
| ldpc_encoder | 1-bit (520) | 1-bit (2600) | wrapper + UNPREFIXED core; bgn=1, Zc=52 |
| rate_match | 1-bit (2600) | 1-bit (E) | 5.4.2.1 bit SELECTION only; runtime `rv_in[1:0]`,`e_in[12:0]`; k0={0,676,1300,2236} |
| bit_interleaver | 1-bit (E) | 1-bit (E) | 5.4.2.2 bit interleaving; runtime `qm_in`,`n_in`(=E/Qm) |
| code_block_concat | 1-bit (E) | 1-bit (E) | TS 38.212 5.5; C=1 pass-through |
| scrambler / descrambler | 1-bit | 1-bit | Gold seq; c_init=0x8000; 1600 warmup; XOR self-inverse (HARD-path) |
| symbol_packer | 1-bit | `[MAX_BPS-1:0]` | runtime `qam_mode[2:0]`; `sym_en` strobe; `tuser`=bps |
| qam_mapper | `[MAX_BPS-1:0]` | 32-bit `{Q,I}` Q5.10 | runtime `qam_mode`; Gray; scale per Sec 4 |
| qam_slicer | 32-bit `{Q,I}` | 1-bit | hard decision; PROVISIONAL (soft path uses soft_demapper) |
| rrc_tx | 32-bit `{Q,I}` symbol | 32-bit `{Q,I}` sample | polyphase 1->4; seamless; group delay 16 samples |
| rrc_rx | 32-bit `{Q,I}` sample | 32-bit `{Q,I}` sample | matched 33-tap FIR, full rate; group delay 16 samples |
| soft_demapper | 32-bit `{Q,I}` Q5.10 | 4-bit signed LLR | runtime `qam_mode`; max-log; one LLR/beat MSB-first; tlast/symbol |
| llr_descrambler | 4-bit signed LLR | 4-bit signed LLR | SOFT-path descramble; sign-flip where Gold=1; c_init,1600 warmup; sat-negate |
| code_block_de_concat | 4-bit signed LLR | 4-bit signed LLR | 5.5 inverse; C=1 pass-through (LLR domain) |
| bit_deinterleaver (LLR) | 4-bit signed LLR (E) | 4-bit signed LLR (E) | 5.4.2.2 inverse; runtime `qm_in`,`n_in`; LLR_W=4 |
| rate_dematch | 4-bit signed LLR (E) | 4-bit signed LLR (2600) | 5.4.2.1 inverse; scatter to k0(RV), zero-fill punctured, sat soft-combine; runtime `rv_in`,`e_in` |
| ldpc_decoder | 4-bit signed LLR (2600) | 1-bit (520) | wrapper + `dec_`-PREFIXED core; bgn=1, Zc=52 |
| code_block_deseg | 1-bit (520) | 1-bit (520) | 5.2.2 inverse; C=1 pass-through (bit domain) |
| crc24a_check | 1-bit (520) | 1-bit (496) | outputs `crc_ok`; strips CRC |
| ber_counter | 1-bit | (count) | regenerates PRBS-23, counts errors |

**Chain order — TX** (TS 38.212 §5 then §7.3 scramble/modulate):
bit_source -> crc24a_attach (CRC-24A) -> code_block_seg (5.2.2) -> ldpc_encoder (5.3.2)
-> rate_match (5.4.2.1 selection) -> bit_interleaver (5.4.2.2) -> code_block_concat (5.5)
-> scrambler (7.3.1) -> symbol_packer -> qam_mapper -> rrc_tx -> channel.

**Chain order — RX** (exact mirror):
channel -> AGC -> rrc_rx -> Costas -> Gardner (1-of-4 downsample) -> equalizer ->
soft_demapper -> llr_descrambler -> code_block_de_concat -> bit_deinterleaver (LLR)
-> rate_dematch -> ldpc_decoder -> code_block_deseg -> crc24a_check -> ber_counter.

> ORDER CORRECTIONS folded in this revision: (a) NR LDPC has **no sub-block
> interleaver** — rate_match is bit-selection ONLY; the §5.4.2.2 bit interleaver is
> the separate `bit_interleaver`. (b) `code_block_concat` comes **before** the
> scrambler (concat is §5.5, scrambling is §7.3.1). For C=1 all of these are identity,
> so behavior is unchanged, but the documented order is now correct.

## 8. BLOCK STATUS + SPECS FOR UNBUILT BLOCKS

### DONE — verified
bit_source; crc24a_attach; crc24a_check; code_block_seg; code_block_concat;
ldpc_encoder; ldpc_decoder; rate_match; rate_dematch; bit_interleaver;
bit_deinterleaver (LLR); scrambler; descrambler; llr_descrambler;
code_block_de_concat; code_block_deseg; symbol_packer; qam_mapper;
qam_slicer (provisional); soft_demapper; rrc_tx (seamless); rrc_rx (matched);
ber_counter.

Verification highlights: LDPC en/decoder match `nrLDPCEncode/Decode` (0 diff in XSim).
soft_demapper matches a brute-force max-log oracle (0 sign mismatches ~100k pts).
rrc_tx/rrc_rx match fixed-point FIR golden; cascade Nyquist zero-ISI confirmed.
rate_match: 4368 selected bits across RV 0/1/2/3 x E{1040,624}, 0 errors.
rate_dematch: 13000 LLRs across 5 configs incl E=2700 repetition (soft-combine), 0 err.
seg+concat: 520-bit series identity with backpressure, 0 err. de_concat+deseg: 624 LLRs
+ 520 bits identity, 0 err. llr_descrambler: 520 LLRs vs descrambler-oracle Gold seq +
saturating negate (incl -8 corner), 0 err. LLR deinterleaver: 1040 LLRs round-trip exact.

### CODE-BLOCK COMPLIANCE NOTE (seg / concat / deseg / de_concat)
For B=520 ≤ Kcb=3840 (BG2), TS 38.212 §5.2.2 mandates **C=1, L=0, no CRC-24B**, and
because the project fixes **K=520 (=10·Zc, Zc=52) with zero filler**, the standards-
mandated behavior IS a pass-through. These four blocks implement the §5.2.2 decision
and §5.5 concatenation correctly for a single code block. **Honest scope:**
(a) **C>1 is intentionally unsupported** — incompatible with the fixed-K=520 encoder;
(b) the §5.2.2 **adaptive lifting-size selection is not implemented** — strict §5.2.2
for a 520-bit TB would derive Zc=72/K=720/F=200 filler; the project instead fixes
Zc=52/K=520/F=0 as a self-chosen operating point. **Do NOT switch to Zc=72** — it would
invalidate all verified LDPC work for no benefit in a fixed-TBS demonstration modem.

### NOTE on the two descramblers
`descrambler` (1-bit, XOR) is the inverse for any HARD-bit path (uncoded loopback /
qam_slicer route). The LDPC **soft** path uses `llr_descrambler` (sign-flip on LLRs),
which sits after `soft_demapper`. Both share c_init/warmup and produce the same Gold
sequence.

### TO BUILD — adaptation
- **mcs_controller** — hand-written. Input: SNR/CQI. Output: MCS index -> (Qm, E).
  Broadcasts `qam_mode` to packer/mapper and (E, RV) to rate_match + (Qm, n=E/Qm) to
  bit_interleaver; on RX drives soft_demapper/bit_deinterleaver/rate_dematch. Logic =
  threshold lookup from the MCS table (Section 9). RV=0 fixed for now.
- **MCS signalling insert (TX) / recover (RX)** — tag each packet with the chosen MCS
  in a side-band field (`tuser`); recover on RX to configure the adaptive RX blocks.
  `tuser` format NOT finalized.

### TO BUILD — RX front-end (co-design with channel, Section 10)
- **AGC** — amplitude normalization.
- **Costas loop** — carrier phase/frequency recovery.
- **Gardner timing recovery** — symbol timing; performs the 1-of-4 downsample of the
  matched-filter output.
- **equalizer (+ channel estimation)** — undo bounded multipath.
- **SNR / channel-quality estimator** — feeds mcs_controller (closes the adaptive loop).

### TO BUILD — channel
- **channel** — AWGN core + bounded multipath/fading/interference, configurable Eb/N0.
  See Section 10 for build order and the open synthesizable-vs-testbench decision.

### TO BUILD — integration / deployment
- Full-chain integration TB (coding loop first, then full modem; build in the
  8-symbol RRC cascade-delay offset — Section 11).
- `tb_bit_chain` update to current packet sizes (496/520/2600/E).
- `qam_loopback_axi` — AXI-Lite wrapper for Pynq.
- Pynq Python driver.
- BER-vs-SNR sweep harness + MCS threshold refinement.

## 9. MCS TABLE (decided — curated monotonic-efficiency ladder)

5-point ladder. Two distinct E values {1040, 624} (keeps rate_match simple). E must be
a multiple of Qm so n=E/Qm is integer. SNR thresholds are PLACEHOLDERS — calibrate from
measured BER-curve crossovers; the controller takes thresholds as parameters.

| MCS | Modulation | Qm | Code rate R | E=520/R | n=E/Qm | Efficiency (Qm·R) | SNR threshold* |
|---|---|---|---|---|---|---|---|
| 0 | QPSK   | 2 | 1/2 | 1040 | 520 | 1.00 | ~0–2 dB   |
| 1 | 16-QAM | 4 | 1/2 | 1040 | 260 | 2.00 | ~6–8 dB   |
| 2 | 16-QAM | 4 | 5/6 |  624 | 156 | 3.33 | ~11–13 dB |
| 3 | 64-QAM | 6 | 5/6 |  624 | 104 | 5.00 | ~17–19 dB |
| 4 | 256-QAM| 8 | 5/6 |  624 |  78 | 6.67 | ~23–26 dB |

\*Placeholder. Note non-monotonicity if other points are added (e.g. 64-QAM 1/2 = 3.0
b/s sits below 16-QAM 5/6 = 3.33 b/s) — the ladder is curated to stay monotonic.

## 9b. OPEN DECISIONS / KNOWN CHALLENGES
1. **Pynq Z2 resource fit (accepted-open).** Decoder OOC on XC7Z020: ~117,548 LUTs
   (221%, overflow), BRAM 139.5/140, FF 91,591 (86%), DSP 0. Decoder alone doesn't fit;
   MathWorks NR LDPC Decoder is fixed high-parallelism, no area knob. Proceed
   functionally. Encoder real synth numbers NOT yet captured (TODO: OOC synth).
2. **MCS SNR thresholds** — placeholders; calibrate from BER curves.
3. **MCS signalling (`tuser`) format** — not finalized.
4. **Channel impairment budget** — delay taps / fading rate / interference level not
   finalized; must equal what the RX front-end is designed to resolve.
5. **Channel form** — synthesizable RTL noise gen vs testbench model vs MATLAB-imported
   samples (Section 10).

## 10. CHANNEL — BUILD GUIDANCE
The channel and RX front-end are a matched pair (the channel exists to be undone).
- **Build AWGN first** — self-contained, testable now on the existing chain (the LDPC
  decoder fights it; no front-end block needed to undo it). Enables early BER-vs-SNR.
- **Add other impairments alongside their correctors:** gain - AGC; carrier offset -
  Costas; timing offset - Gardner; multipath/fading - equalizer + channel estimation.
  Building impairments before their corrector gives nothing to test against.
- **Open: how the channel exists** — (a) synthesizable RTL (LFSR Gaussian via
  Box-Muller or sum-of-uniforms/CLT — non-trivial in HW), (b) testbench-only model, or
  (c) MATLAB-generated noise samples from a file. Decide before building.

## 11. VERIFICATION METHODOLOGY + TESTBENCH NOTES
- **Per-block TBs:** drive stimulus on `negedge aclk`, sample/dump on `negedge`, gate to
  exact packet length, compare against a golden file / in-TB expectation / a proven
  block used as oracle (e.g. llr_descrambler checked against descrambler).
- **Backpressure tests:** toggle `m_axis_tready` on the **negedge** so it is stable
  across the sampling posedge — toggling and sampling on the same edge creates a race
  that looks like a DUT bug but is a TB artifact (seen and fixed on the seg/concat TB).
- **Generated-block flow:** MATLAB -> `makehdl`/`makehdltb` -> XSim self-check, then
  hand-write the AXI wrapper and verify it with its own golden TB.
- **TESTBENCH GOTCHA — string ternary in `$display`.** Do NOT write
  `$display(cond ? "PASS" : "FAIL")` — non-portable (Icarus prints garbage, XSim
  prints nothing). Use `if (cond) $display(...); else $display(...)`. Substantive pass
  = `ERRORS=0` plus expected element count.
- **Verilog-2005 gotcha:** cannot bit-select a function-call result
  (`f(x)[3:0]`) — assign to a temp reg first.
- **Coefficients are hardcoded** as `localparam` in filter `.v` files — no `.mem` needed.
- **Filter group delay (alignment).** Each RRC delays 16 samples; TX+RX cascade = 32
  samples = 8 symbols. The full-chain BER comparison MUST offset by 8 symbols
  ("received symbol n aligns to transmitted symbol n-8") or BER reads ~0.5 with a
  perfect channel. Build the offset into the integration TB from the start.
- **rate_dematch latency:** CLEAR (~2600) + COLLECT (E) + STREAM (2600) per frame —
  expect real latency on the RX coded path (correctness, not throughput, is the goal).

## 12. FILE LAYOUT
```
rtl/
  bit_source.v  crc24a_attach.v  crc24a_check.v
  code_block_seg.v  code_block_concat.v  code_block_deseg.v  code_block_de_concat.v
  scrambler.v  descrambler.v  llr_descrambler.v
  rate_match.v  rate_dematch.v
  bit_interleaver.v  bit_deinterleaver.v        (deinterleaver = LLR/4-bit)
  symbol_packer.v  qam_mapper.v  qam_slicer.v
  soft_demapper.v
  rrc_tx.v  rrc_rx.v
  ber_counter.v
  ldpc_encoder.v   + encoder core: HDL_Algorithm.v, NR_LDPC_Encoder.v, SimpleDualPortRAM_generic*.v
  ldpc_decoder.v   + decoder core: dec_HDL_Algorithm.v, dec_NR_LDPC_Decoder.v, dec_SimpleDualPortRAM_generic*.v
tb/
  tb_*.v for each block
  (rate_match: tb_rate_match.v + rm_d.txt ; others compute expected in-TB)
matlab/  (golden references + scripts)
  gen_ldpc_golden.m  run_ldpc_*_check.m  ldpc_in.txt (520)  ldpc_ref.txt (2600)
  nrRateMatchLDPC / nrRateRecoverLDPC cascade checks (rate_match->bit_interleaver)
ref/    (non-RTL design/reference, do NOT add to Vivado)
  taps.py (RRC coeff gen + zero-ISI proof)  taps_q14.txt
  ref.py  (soft_demapper max-log oracle)  vec_in/vec_exp, tx_in/tx_exp, rx_in/rx_exp
```
Vivado: add all `rtl/*.v` (both LDPC core sets — encoder unprefixed, decoder
`dec_`-prefixed) as DESIGN sources; TBs as SIMULATION sources; ensure no old
unprefixed decoder core files remain.

## 13. SUGGESTED OWNERSHIP / NEXT ACTIONS
The TX and RX **bit/coding paths are functionally complete and individually verified**
(bit_source -> qam_mapper on TX; soft_demapper -> crc24a_check on RX). Remaining work,
parallelizable:
- **RX front-end + channel (co-design):** AWGN channel first, then AGC / Costas /
  Gardner / equalizer / SNR estimator, each with its matching impairment.
- **Adaptation:** mcs_controller + MCS signalling insert/recover (MCS table is decided;
  drives the already-adaptive mapper/packer/interleaver/rate_match + RX mirrors).
- **Integration:** full-chain TB (with the 8-symbol cascade-delay offset) -> qam_loopback_axi
  + Pynq driver -> BER sweep + MCS threshold calibration.

When starting a new chat for a block, re-upload this document and state which block is
being built; the assistant should read Sections 2–8 and continue without re-deriving
settled facts, verifying any MATLAB/spec claim before stating it.

### Cross-check still owed (do once, at integration)
Confirm the `rate_match -> bit_interleaver` cascade equals `nrRateMatchLDPC`:
```matlab
d = nrLDPCEncode(double(msg520), 2);          % 2600-bit BG2 codeword
golden = nrRateMatchLDPC(d, E, 0, '16QAM', 1);% selection + 5.4.2.2 interleave, RV0, Qm4
% hardware: d -> rate_match(rv=0,e_in=E) -> bit_interleaver(qm=4,n=E/4); compare to golden
```
If they match, the rate_match / bit_interleaver split is proven standards-correct.
