# Adaptive QAM Modem on Pynq Z2 — Project Handoff (Team Reference)

> Authoritative project-state and integration document. Anyone picking up a block
> should read Sections 1–6 once (conventions + interfaces), then go to their block's
> spec in Section 8, and Section 14 for integration status/results. Supersedes all
> earlier handoffs.

---

## 0. CURRENT STATUS (newest first — full detail in Section 14)

**All blocks built and individually verified. Chain integrated and largely proven.**

- **Tier-1 integration (coding loop, LDPC bypassed, ideal symbol loopback):** PASS,
  BER=0 / BLER=0 across QPSK / 16-QAM / 256-QAM. Shook out handshake, framing, MCS
  fan-out, PRBS alignment.
- **Tier-2 stage 2a (full real chain incl. LDPC + RRC + channel, front-end idealized,
  zero noise):** PASS in XSim, BER=0 — confirms real FEC + pulse-shaping + framing.
- **Tier-2 stage 2b (AWGN BER-vs-Eb/N0 sweep, MCS0):** PASS — clean LDPC waterfall
  (BER ≈ 0.20 / 0.094 / 1.1e-3 / <1e-4 at 0/1/2/3 dB; knee ~1–2 dB). Calibration
  derived and verified (Section 14).
- **Tier-2 stage 2c-1 (carrier recovery + 90° ambiguity):** DONE. Costas locks a
  carrier offset; new `phase_derotate` block resolves the 4-fold ambiguity (0 errors,
  all 4 rotations).
- **Tier-2 stage 2c-2 (Gardner timing recovery): OPEN — Gardner does NOT lock.** Loop
  runs away to the `mu_step` clamp for both feedback polarities and all gains tried.
  Needs TED S-curve characterization + loop fix. This is the #1 remaining work item.
  Everything else works with ideal/assisted timing.

**Three NEW required design blocks** (add to Vivado; not in earlier handoffs):
- `ber_counter` (REDESIGNED) — payload-level post-decode BER **and** BLER. Replaces the
  old slicer-level counter, which was only valid for the uncoded loopback.
- `rx_llr_framer` — re-imposes packet `tlast` after `soft_demapper` (the symbol domain
  drops packet framing; the demapper only re-creates per-symbol `tlast`, but
  `bit_deinterleaver` needs packet-level `tlast`). Without it the RX deinterleaver
  closes a block after one symbol.
- `phase_derotate` — resolves the QPSK Costas 90° ambiguity using a known preamble.

**Key tuning / discipline findings:**
- **Costas `KI_SHL`: 4 → ~7.** Cuts carrier acquisition ~8× (≈30k → ≈4k symbols), no
  EVM penalty. Acquisition still ≫ the 9-symbol MCS header → carrier needs a dedicated
  lead-in / acquisition preamble, separate from per-packet MCS signalling.
- **Reset deassertion:** deassert `aresetn` on a **negedge** (or via a reset
  synchronizer). Releasing async reset on a clock edge races the PRBS phase and
  produces a phantom ~0.5 BER on a perfectly correct datapath (cost us a debug cycle).
- **Box-Muller noise LUTs (`bm_*.mem`) were mis-scaled** (~3.2× too little noise ≈ 10 dB
  error). Regenerated + verified. Use the corrected `bm_fmag/cos/sin.mem` (and
  regenerated `costas_cos/sin.mem`) — see Section 14.

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
| crc24a_check | 1-bit (520) | 1-bit (496) | outputs `crc_ok`,`crc_valid` (pulse on last); strips CRC |
| rx_llr_framer | 4-bit signed LLR | 4-bit signed LLR | NEW. re-imposes PACKET `tlast` on E-th LLR (`e_in`); ignores per-symbol tlast. Place right after soft_demapper |
| ber_counter | 1-bit (payload) + crc_ok/crc_valid | (counts) | REDESIGNED. taps post-CRC payload; regenerates PRBS-23 → post-decode BER; counts crc_ok → BLER; saturating |
| channel | 32-bit `{Q,I}` sample | 32-bit `{Q,I}` sample | `noise_std[15:0]` (0=off), writable 8-tap multipath (`tap_we/idx/re/im`); Box-Muller AWGN; needs `bm_*.mem` |
| agc | 32-bit `{Q,I}` | 32-bit `{Q,I}` | power-normalize to unit; modulation-independent; `gain` out; SIGNED integrator |
| gardner | 32-bit `{Q,I}` (4 sps) | 32-bit `{Q,I}` (1 sps) | timing recovery + 4→1 downsample; `mu_step_dbg`; **NOT LOCKING — see Sec 14** |
| costas | 32-bit `{Q,I}` | 32-bit `{Q,I}` | carrier recovery; `freq_dbg`; `KI_SHL` → set ~7; 90° ambiguity (resolve w/ phase_derotate) |
| phase_derotate | 32-bit `{Q,I}` | 32-bit `{Q,I}` | NEW. resolves Costas 90° ambiguity via known preamble; `sop` in, `k_dbg` out; sign/swap de-rotate |
| equalizer | 32-bit `{Q,I}` | 32-bit `{Q,I}` | LMS FIR (7-tap, center init 1.0); `qam_mode`,`train_en`,`d_train`; DD/trained |
| snr_estimator | 32-bit `{Q,I}` | (metric) | DD-EVM (err_pow, lower=better ≈ EVM²); passive monitor → mcs_controller |
| mcs_controller | (metric) | config | metric→MCS via TH1–4 (placeholders); dwell-hysteresis; `force_en/force_mcs`; outputs qam_mode/e/n/rv |
| mcs_insert | 32-bit `{Q,I}` + mcs | 32-bit `{Q,I}` | TX after qam_mapper; prepends 9-sym fixed-QPSK header (maj-of-3) |
| mcs_recover | 32-bit `{Q,I}` | 32-bit `{Q,I}` + mcs | RX after equalizer; decodes header (maj-of-3), strips it, outputs mcs + qam_mode/e/n |

**Chain order — TX** (TS 38.212 §5 then §7.3 scramble/modulate):
bit_source -> crc24a_attach (CRC-24A) -> code_block_seg (5.2.2) -> ldpc_encoder (5.3.2)
-> rate_match (5.4.2.1 selection) -> bit_interleaver (5.4.2.2) -> code_block_concat (5.5)
-> scrambler (7.3.1) -> symbol_packer -> qam_mapper -> rrc_tx -> channel.

**Chain order — RX** (exact mirror; NEW blocks marked *):
channel -> agc -> rrc_rx -> gardner (1-of-4 downsample) -> costas -> *phase_derotate* ->
equalizer -> mcs_recover -> soft_demapper -> *rx_llr_framer* -> llr_descrambler ->
code_block_de_concat -> bit_deinterleaver (LLR) -> rate_dematch -> ldpc_decoder ->
code_block_deseg -> crc24a_check -> ber_counter.

> Front-end framing notes: the symbol domain (symbol_packer/qam_mapper/rrc) carries
> NO packet `tlast`; `soft_demapper` re-creates only a PER-SYMBOL `tlast`. `rx_llr_framer`
> rebuilds the PACKET `tlast` (on the E-th LLR) that the deinterleaver needs. The carrier
> path needs a lead-in/preamble (Costas acquisition ≈4k symbols even tuned); `phase_derotate`
> uses a known preamble (its `sop`) to kill the residual 90° ambiguity before the demapper.
> `equalizer` sits upstream of `mcs_recover`, so for adaptive mode it cannot know the
> current packet's payload `qam_mode` while equalizing it — train on the QPSK header then
> freeze taps, or feed back `mcs_recover.qam_mode` delayed one packet. (`soft_demapper` is
> downstream of `mcs_recover`, so it gets `qam_mode` correctly with no feedback.)

> ORDER CORRECTIONS folded in this revision: (a) NR LDPC has **no sub-block
> interleaver** — rate_match is bit-selection ONLY; the §5.4.2.2 bit interleaver is
> the separate `bit_interleaver`. (b) `code_block_concat` comes **before** the
> scrambler (concat is §5.5, scrambling is §7.3.1). For C=1 all of these are identity,
> so behavior is unchanged, but the documented order is now correct.

## 8. BLOCK STATUS + SPECS (all blocks now built; see Section 14 for integration)

### DONE — verified
bit_source; crc24a_attach; crc24a_check; code_block_seg; code_block_concat;
ldpc_encoder; ldpc_decoder; rate_match; rate_dematch; bit_interleaver;
bit_deinterleaver (LLR); scrambler; descrambler; llr_descrambler;
code_block_de_concat; code_block_deseg; symbol_packer; qam_mapper;
qam_slicer (provisional); soft_demapper; rrc_tx (seamless); rrc_rx (matched);
ber_counter (REDESIGNED — payload BER+BLER); rx_llr_framer (NEW); phase_derotate (NEW);
channel (AWGN + multipath); agc; costas; equalizer; snr_estimator; mcs_controller;
mcs_insert; mcs_recover. **gardner: built but NOT locking (open — Section 14).**

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

### BUILT — adaptation (verified individually; integration tuning ongoing)
- **mcs_controller** — metric→MCS via 4 descending thresholds TH1–4 (PLACEHOLDERS,
  calibrate from BER curves); dwell-hysteresis (DWELL=256), step-toward-target;
  `force_en`/`force_mcs` override. Outputs qam_mode/e/n/rv. Verified climb/fall/force.
- **MCS signalling** — `mcs_insert` (TX, after qam_mapper: prepends 9-symbol fixed-QPSK
  header, majority-of-3, Q-axis pilot) + `mcs_recover` (RX, after equalizer: maj-of-3
  decode, strips header, outputs mcs + table-derived qam_mode/e/n). Verified loopback
  incl. 1-error robustness. NOTE: header doubles as nothing else — carrier acquisition
  needs a SEPARATE longer preamble (Section 14).

### BUILT — RX front-end
- **agc** — verified converges from 64× power range to ~unit (SIGNED integrator — a
  required fix; unsigned makes the whole add unsigned and wraps on negative correction).
- **costas** — verified LOCKS carrier offset (EVM≈3); set `KI_SHL≈7` (Section 14);
  90° ambiguity resolved by `phase_derotate`.
- **gardner** — **OPEN: does not achieve timing lock** (loop runs to clamp, both
  polarities). Section 14 has the diagnosis + next step. The chain runs with an ideal
  decimator stand-in (`ideal_decimator.v`) meanwhile.
- **equalizer** — LMS, verified error collapses 4× when trained on injected ISI. `d_train`
  must be delayed by CENTER. For AWGN-only runs it can stay at the unit-center passthrough.
- **snr_estimator** — DD-EVM, verified clean→0 / tracks true error power. Passive.

### BUILT — channel
- **channel** — sample-domain, 1 samp/clk, 4-cyc latency. Stage1 writable 8-tap complex
  multipath FIR (default unit tap = passthrough). Stage2 Box-Muller AWGN (`noise_std`
  Q5.10; 0 disables). Needs `bm_fmag/cos/sin.mem` (4096-entry LUTs) — **use the
  REGENERATED ones** (originals mis-scaled, Section 14). Verified passthrough exact,
  noise std calibration (Section 14), multipath echo exact.

### BUILT — integration / test infrastructure (Section 14)
- `tier2_top_2a` (full real chain, front-end idealized), `ideal_decimator`,
  `tx_payload_framer`, `fec_bypass_rx`, `carrier_inject`, `timing_offset`,
  and TBs `tb_tier1[_multi]`, `tb_tier2_2a`, `tb_tier2_2b`, `tb_derot`, `tb_2c1`, `tb_2c2`.
- Still TODO: `qam_loopback_axi` (AXI-Lite Pynq wrapper) + Pynq Python driver;
  Gardner fix; full-chain XSim confirmation with real front-end; MCS-threshold calibration.

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
1. **Gardner timing recovery does not lock (TOP open item).** Loop runs to the `mu_step`
   clamp for both `TED_SIGN` polarities and all gains tried; output EVM ~32000 (garbage).
   Next step = open-loop TED S-curve characterization (Section 14). Chain runs with
   `ideal_decimator` meanwhile.
2. **Pynq Z2 resource fit (accepted-open).** Decoder OOC on XC7Z020: ~117,548 LUTs
   (221%, overflow), BRAM 139.5/140, FF 91,591 (86%), DSP 0. MathWorks NR LDPC Decoder
   is fixed high-parallelism, no area knob. Proceed functionally. Encoder OOC not yet captured.
3. **MCS SNR thresholds (TH1–4)** — placeholders; calibrate from the 2b BER curves
   (extend the 2b sweep to each MCS).
4. **Carrier acquisition preamble** — Costas needs ~4k symbols even tuned, ≫ the 9-symbol
   MCS header. Decide the dedicated acquisition-preamble / continuous-lead-in structure
   ([acq preamble] → [de-rotation preamble for phase_derotate] → [MCS header] → [payload]).
5. **Adaptive-mode equalizer `qam_mode`** — equalizer is upstream of mcs_recover; train on
   header + freeze, or feed back qam_mode delayed one packet (Section 7 note).
6. **Channel impairment budget** — multipath delay taps / fading rate / interference level
   not finalized; must equal what the front-end is designed to resolve.

### RESOLVED since earlier handoffs
- **Channel form:** synthesizable RTL, Box-Muller AWGN + writable multipath FIR (built).
- **MCS signalling:** done via `mcs_insert`/`mcs_recover` (9-symbol QPSK header, maj-of-3)
  — NOT a `tuser` side-band; it's an in-band header.
- **noise_std ↔ Eb/N0 calibration:** derived + verified (Section 14).

## 10. CHANNEL — BUILD GUIDANCE (channel now BUILT; kept as design rationale)
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
- **RESET DEASSERTION (critical, learned in integration).** Deassert `aresetn` on a
  **negedge** (or through a reset synchronizer), never coincident with a posedge.
  Releasing async reset on a clock edge races `bit_source`'s PRBS by one bit, which the
  ber_counter then mis-aligns → a phantom **~0.5 BER on a perfectly correct datapath**.
  Tell-tale: BER ≈ 0.5 but every CRC passes (`packet_errors=0`) → datapath fine,
  reference mis-aligned. Applies to XSim TBs too.
- **PRBS alignment:** the redesigned `ber_counter` regenerates a CONTINUOUS PRBS-23
  (one advance per recovered payload bit, no per-packet reset) to match `bit_source`.
  `crc_valid` pulses CONCURRENT with the last payload bit (`tlast`), not a cycle later.

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

The full TX and RX paths are built and verified, and the chain is integrated and proven
end-to-end except for Gardner timing (Sections 0 and 14). Remaining work, in priority order:

1. **Fix Gardner timing recovery** (TOP). Open-loop TED S-curve characterization first
   (Section 14), then loop sign/gain — same approach that fixed Costas.
2. **Full-chain XSim confirmation with the REAL front-end** (replace `ideal_decimator`
   with the fixed Gardner; carrier offset + Costas KI=7 + phase_derotate; then multipath
   + equalizer). Decoder is too large for iverilog at length — runs in XSim.
3. **Calibrate MCS thresholds** — extend the 2b AWGN sweep to each MCS; set TH1–4 from
   the BER/BLER crossovers; then enable adaptive mode (un-force mcs_controller) and test
   live MCS switching + the equalizer `qam_mode` feedback timing.
4. **Decide + build the acquisition preamble** (Section 9b item 4) and wire `phase_derotate`'s
   `sop` from the framing.
5. **Deployment:** `qam_loopback_axi` (AXI-Lite Pynq wrapper) + Pynq Python driver; encoder
   OOC synth numbers.

When starting a new chat, re-upload this document and state the task; read Sections 0, 2–8,
and 14, and verify any MATLAB/spec claim before stating it.

### Cross-check still owed (do once, at integration)
Confirm the `rate_match -> bit_interleaver` cascade equals `nrRateMatchLDPC`:
```matlab
d = nrLDPCEncode(double(msg520), 2);          % 2600-bit BG2 codeword
golden = nrRateMatchLDPC(d, E, 0, '16QAM', 1);% selection + 5.4.2.2 interleave, RV0, Qm4
% hardware: d -> rate_match(rv=0,e_in=E) -> bit_interleaver(qm=4,n=E/4); compare to golden
```
If they match, the rate_match / bit_interleaver split is proven standards-correct.

## 14. INTEGRATION & BRING-UP RESULTS (newest section)

Staged bring-up. Tier 1 runs in iverilog/XSim (LDPC bypassed); Tier 2 needs XSim for
the real decoder (too large for iverilog at length). All RTL files in `rtl/`; the
integration tops/TBs and stand-ins below are simulation infrastructure.

### Tier 1 — coding loop, LDPC bypassed, ideal symbol loopback  [PASS]
`tier1_top` wires bit_source→…→qam_mapper =loopback= soft_demapper→…→ber_counter, with
the LDPC encoder+rate_match bypassed (seg→interleaver direct) and decoder+rate_dematch
bypassed (`fec_bypass_rx`, an LLR→hard slice). Result: **BER=0 / BLER=0 across QPSK,
16-QAM, 256-QAM** (`tb_tier1`, `tb_tier1_multi`). (64-QAM excluded only because the
bypass E=520 isn't a multiple of 6; the real chain uses E=624, which is.)
Three integration findings, all resolved:
- **RX packet-framing gap** → fixed by the new `rx_llr_framer` (see Section 0/7).
- **Wrong deinterleaver in soft path** → must use the **LLR (4-bit)** `bit_deinterleaver`,
  not the 1-bit hard one (hard-slicing before deinterleave destroys soft info).
- **Reset-deassertion race** → negedge deassert (Section 11).

### Tier 2 stage 2a — full real chain, front-end idealized, zero noise  [PASS]
`tier2_top_2a` = real ldpc_encoder + rate_match + RRC + channel(noise=0) + real
rate_dematch + ldpc_decoder, with front-end loops omitted and timing via
`ideal_decimator`, fixed MCS0. **BER=0 in XSim** → confirms FEC + RRC + framing line up.
- Verified: encoder emits exactly **2600** codeword bits for 520 info bits (matches
  rate_match N).
- RRC TX+RX cascade group delay = **8 symbols**; decimation phase **0**. `ideal_decimator`
  uses `DEC_PHASE=0, DEC_DROP=8` (validated lossless, 0/300 errors, by sweeping phase).

### Tier 2 stage 2b — AWGN BER-vs-Eb/N0 sweep (MCS0)  [PASS]
`tb_tier2_2b` resets per SNR point and prints BER/BLER. Measured (20 pkts/pt):
0 dB → BER 0.199 BLER 1.0; 1 dB → 0.094 / 0.90; 2 dB → 1.1e-3 / 0.05; ≥3 dB → 0 errors
in 9920 bits (i.e. <1e-4). Clean LDPC waterfall, knee ~1–2 dB, monotonic. (Below
threshold at 0 dB the decoder amplifies errors — expected.) For a real floor number,
rerun ≥3 dB points with large `NPKT`.

**Noise calibration (verified to ~0.1 dB):**
- Symbol energy `Es ≈ 2^20 = 1,048,576` (raw), all modulations (unit-energy normalized).
- Per-component noise std (raw) ≈ `noise_std` (measured 1012 at noise_std=1024).
- **Es/N0 = 524288 / noise_std²**;  **Eb/N0 = Es/N0 / (Qm·R)**.
- Set a target: `noise_std = round( sqrt( 524288 / (10^(EbN0_dB/10) · Qm · R) ) )`.
- MCS0 table (Qm·R=1): 0 dB→724, 1→645, 2→575, 3→513, 4→457, 5→407, 6→363.
- **The original `bm_*.mem` were mis-scaled (~3.2× low ≈ 10 dB).** Regenerated + verified
  (`bm_fmag/cos/sin.mem`); `costas_cos/sin.mem` regenerated too. Use the corrected files.

### Tier 2 stage 2c-1 — carrier recovery + 90° ambiguity  [DONE]
- Costas **locks** a carrier offset (`tb_2c1`, `carrier_inject` injects e^{jΔ}): freq_dbg
  → 4·DPHI, post-lock **EVM≈3** (clean constellation).
- **Costas `KI_SHL`: 4 → ~7** cuts acquisition ~8× (≈30k→≈4k symbols), no EVM penalty.
- `phase_derotate` resolves the QPSK 4-fold ambiguity: correlate a known preamble vs the
  post-Costas symbols → detect k·90° → de-rotate payload (sign/swap). **0/300 errors for
  all 4 rotations** (`tb_derot`). Needs a `sop` strobe at the preamble (framing).

### Tier 2 stage 2c-2 — Gardner timing recovery  [OPEN — does not lock]
`tb_2c2` (+ `timing_offset` injector) drives Gardner in place of `ideal_decimator`.
**The loop runs away to the `mu_step` clamp (±512) for BOTH `TED_SIGN` polarities and all
gains tried; output EVM ~32000 (garbage), BER ≈ 0.5.** mu_step near nominal only means it
hasn't moved yet; with stronger gains it pegs immediately. This is unstable feedback /
no valid discriminant, not slow acquisition — consistent with the original note that
Gardner's timing-lock was deferred (only rate + boundedness were ever checked).
**Next step (definitive):** open-loop TED S-curve — force `mu_step=NOM_STEP`, expose
`ted_e`, sweep a static timing offset (`timing_offset` MU), plot mean `ted_e` vs offset.
A clean S-curve through zero → detector OK, fix is loop sign/gain (contained). No clean
discriminant → the on-time/mid-point sample indexing in the TED is structurally wrong.

### Integration file inventory (simulation infrastructure — keep separate from synth RTL)
- Design blocks to ADD to the build: `ber_counter` (redesigned), `rx_llr_framer`,
  `phase_derotate`. Corrected LUTs: `bm_fmag/cos/sin.mem`, `costas_cos/sin.mem`.
- Test-only (do NOT synthesize): `tier2_top_2a.v`, `ideal_decimator.v` (Gardner stand-in),
  `tx_payload_framer.v` (stands in for the MAC giving crc24a_attach its 496-bit boundary —
  the real system still needs an equivalent framer), `fec_bypass_rx.v` (Tier-1 decoder
  stand-in), `carrier_inject.v`, `timing_offset.v`, and TBs `tb_tier1.v`, `tb_tier1_multi.v`,
  `tb_tier2_2a.v`, `tb_tier2_2b.v`, `tb_derot.v`, `tb_2c1.v`, `tb_2c2.v`.
