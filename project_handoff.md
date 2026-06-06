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
   (`$fwrite`) and comparing (diff / Windows `fc`, or in-TB compare) against an
   independent golden reference (MATLAB, Python, or a known-good model). A block is
   not "done" until it matches its golden reference bit/value-exact (or within a
   stated tolerance for DSP blocks).
4. **Complete files.** Deliver full `.v` files or clearly-marked single-section
   edits, not loose snippets.
5. **Stable interfaces.** The locked parameters (Section 4) and AXI-Stream
   conventions (Section 3) are fixed. Flag explicitly before changing one.
6. **Toolchain:** Windows 11, Vivado (synthesis + XSim simulation), MATLAB R2024b
   (5G Toolbox + Wireless HDL Toolbox + HDL Coder). Verilog-2001, ASCII only.
   Not Linux, not cocotb.

## 3. AXI-STREAM + DATAPATH CONVENTIONS

Every block uses the same handshake and reset idiom so blocks chain cleanly:

- **Handshake:** data transfers on a clock edge only when `tvalid` AND `tready` are
  both high. Outputs are **registered**. `tlast` marks the last beat of a packet.
- **Clock/reset:** single `aclk` (100 MHz). `aresetn` is **async, active-low**.
  Idiom: `always @(posedge aclk or negedge aresetn)`.
- **Naming:** `s_axis_*` for slave (input) side, `m_axis_*` for master (output) side.
- **Data formats:**
  - **Bit-domain** blocks carry **1 bit** per beat on `tdata`.
  - **LLR-domain** (soft RX) blocks carry **4-bit signed** (`signed [3:0]`) per beat.
  - **Symbol/sample-domain** blocks carry a complex value as a **32-bit** word
    `{Q[15:0], I[15:0]}`, each 16-bit **signed Q5.10** (value = stored/1024).
  - Coefficients are **Q1.14** signed 16-bit (value = stored/16384).
  - MAC accumulators are **40-bit signed**; after a Q1.14*Q5.10 product, shift
    right (`>>>`) by 14 and saturate back to 16-bit Q5.10.
- **One element per beat.** All streaming blocks move one element (bit / LLR /
  complex sample) per beat. Gearbox blocks (LDPC en/decoder) internally convert
  between this serial stream and a parallel core interface.

## 4. LOCKED ARCHITECTURE PARAMETERS

```
Clock                100 MHz, single domain
Reset                async active-low (aresetn)
SPS                  4 samples per symbol
Symbol rate          25 Msym/s
Data path            Q5.10 signed 16-bit I/Q  (range +/-32, step 1/1024)
Coefficient path     Q1.14 signed 16-bit
Accumulator          40-bit signed; product >>>14; saturate to Q5.10
Modulation set       QPSK / 16 / 64 / 256-QAM   (Qm = 2/4/6/8; bits-per-axis = Qm/2)
LDPC base graph      BG2 only
LDPC lifting Zc      52
Info block K         520 bits (= 10*Zc)
Mother codeword N    2600 bits (full lifted 52*Zc = 2704; first 2*Zc=104 punctured)
Code rates           1/2 baseline (E=1040); adaptive set also 5/6 (E=624). E = K / R.
Redundancy version   RV = 0 (no HARQ)
LLR format           4-bit SIGNED (sfix4) -- system-wide lock (Section 5)
PRBS                 PRBS-23, poly x^23 + x^18 + 1, seed 0x5A3C7E
CRC-24A              0x864CFB  (transport-block CRC; init 0; no reflect; no final XOR; MSB-first)
CRC-24B              0x800063  (code-block CRC; attached only when C>1 -- verify 38.212 5.2.2 when building seg)
Scrambler c_init     0x00008000 (n_RNTI=1, q=0, n_ID=0); Gold sequence; Nc=1600 warmup
QAM scaling          724 / 324 / 158 / 79 = round(1/sqrt(2), 1/sqrt(10), 1/sqrt(42), 1/sqrt(170) * 1024)
RRC filter           root-raised-cosine, beta=0.5, span=8 symbols, 33 taps (Q1.14)
```

## 5. LLR FORMAT LOCK (read before building any soft RX block)

The LDPC **decoder core** was generated with **4-bit signed (sfix4)** LLR inputs.
This is a **system-wide constant**. Every soft RX block — `soft_demapper`,
`bit_deinterleaver` (LLR version), `rate_dematch` — must produce/carry **4-bit
signed** LLRs in this exact format. **Convention: positive LLR => bit 0, negative =>
bit 1.** Changing the LLR width later requires **regenerating the decoder** at the
new width and re-verifying.

## 6. MATLAB -> HDL WORKFLOW (for LDPC and any other HDL-Coder-generated block)

The LDPC encoder/decoder (and a rate-match block if one exists in HDL form) come
from the MathWorks Wireless HDL Toolbox via HDL Coder. Use this exact recipe — these
are corrections to known traps:

- **Open the example:** `openExample('whdl/NRLDPCEncodeAndDecodeHDLExample')`.
- **bgn convention is INVERTED between function and block:** functions
  `nrLDPCEncode`/`nrLDPCDecode` use **bgn=2** for base graph 2; the Simulink BLOCK
  uses **bgn=1** for base graph 2 (0 = BG1). The wrappers tie the block's `bgn=1`.
- **liftingSize = 52** passed directly.
- **Match our reset:** generate with `'ResetType','async'`,
  `'ResetAssertedLevel','active-low'` so the core reset connects to `aresetn`.
- **Module-name collisions -> use `ModulePrefix`.** Each generated core re-emits
  `HDL_Algorithm`, `NR_LDPC_*`, and its own `SimpleDualPortRAM_*` set with the SAME
  names. The **encoder** core is UNPREFIXED; the **decoder** core was regenerated
  with `'ModulePrefix','dec_'`. Any further generated core MUST use a unique prefix
  to avoid duplicate-module errors when all are in one Vivado project.
- **makehdltb stale-context fix:** run in one clean session —
  `bdclose all;` then `run('<setup/check script>');` then `makehdl(...)` then
  `makehdltb(...)`. (Avoids the `hdlmdlgenlib` "valid DUT" error.)
- **XSim from Windows cmd** (Vivado `settings64.bat` on PATH), run in the folder
  holding the `.v` and `.txt` files (relative-path reads):
  `dir /b *.v > files.prj` -> `xvlog -f files.prj` -> `xelab <tb_top> -s <snap>` ->
  `xsim <snap> -runall` -> `fc <out>.txt <ref>.txt`. (cmd does not expand `*.v`.)
- **`hdlsrc\<MODEL>\` is regeneratable byproduct** — delete it before regenerating
  so prefixed/unprefixed files never mix; copy only the source `.v` into `rtl\`.

## 7. BLOCK INTERFACE REFERENCE (completed blocks)

All blocks share `aclk`, `aresetn`, and standard `s_axis_*`/`m_axis_*` handshake.
Only the data widths and any extra control signals are listed. Exact port names are
in each file header.

| Block | Slave in | Master out | Extra signals / notes |
|---|---|---|---|
| bit_source | (none) | 1-bit | PRBS-23 source; combinational data, advances on tready |
| crc24a_attach | 1-bit (496) | 1-bit (520) | appends 24 CRC bits; tlast on last out bit |
| crc24a_check | 1-bit (520) | 1-bit (496) | outputs `crc_ok`; strips CRC |
| ldpc_encoder | 1-bit (520) | 1-bit (2600) | wrapper + UNPREFIXED core; bgn=1, Zc=52 |
| scrambler / descrambler | 1-bit | 1-bit | Gold seq; c_init=0x8000; 1600-cycle warmup; XOR self-inverse; one packet per reset |
| bit_interleaver | 1-bit (E) | 1-bit (E) | runtime `qm_in[3:0]`, `n_in[ADDR_W-1:0]` (= E/Qm); latched per packet |
| symbol_packer | 1-bit | `[MAX_BPS-1:0]` symbol | runtime `qam_mode[2:0]`; emits on `sym_en` strobe; `tuser`=bps |
| qam_mapper | `[MAX_BPS-1:0]` symbol | 32-bit `{Q,I}` Q5.10 | runtime `qam_mode`; Gray; scale per Section 4 |
| qam_slicer | 32-bit `{Q,I}` | 1-bit | hard decision; PROVISIONAL (soft_demapper replaces on coded path) |
| soft_demapper | 32-bit `{Q,I}` Q5.10 | 4-bit signed LLR | runtime `qam_mode`; max-log; one LLR/beat MSB-first; `tlast` per symbol |
| bit_deinterleaver (LLR) | 4-bit signed LLR (E) | 4-bit signed LLR (E) | runtime `qm_in`, `n_in`; `LLR_W=4` param; inverse of bit_interleaver |
| ldpc_decoder | 4-bit signed LLR (2600) | 1-bit (520) | wrapper + `dec_`-PREFIXED core; bgn=1, Zc=52 |
| rrc_tx | 32-bit `{Q,I}` symbol | 32-bit `{Q,I}` sample | polyphase interpolator: 1 symbol in -> 4 samples out; seamless (no gaps); group delay 16 samples |
| rrc_rx | 32-bit `{Q,I}` sample | 32-bit `{Q,I}` sample | matched 33-tap FIR, full rate (1 in/1 out); group delay 16 samples; downsample is downstream |
| ber_counter | 1-bit | (count) | regenerates PRBS-23, counts errors vs original |

**Chain order (TX):** bit_source -> crc24a_attach -> [code_block_seg] -> ldpc_encoder
-> rate_match -> bit_interleaver -> scrambler -> [code_block_concat] -> symbol_packer
-> qam_mapper -> rrc_tx -> channel.

**Chain order (RX):** channel -> AGC -> rrc_rx (matched) -> Costas -> Gardner (1-of-4
downsample) -> equalizer -> soft_demapper -> descrambler -> bit_deinterleaver(LLR) ->
rate_dematch -> [code_block_deseg] -> ldpc_decoder -> [code_block_de-concat] ->
crc24a_check -> ber_counter.
(Exact placement of scramble vs interleave and seg/concat follows TS 38.211/38.212;
confirm ordering against the spec when wiring the full chain.)

## 8. BLOCK STATUS + SPECS FOR UNBUILT BLOCKS

### DONE — verified + integrated
bit_source; crc24a_attach; crc24a_check; ldpc_encoder; ldpc_decoder;
scrambler; descrambler; bit_interleaver; bit_deinterleaver (LLR, 4-bit);
symbol_packer; qam_mapper; qam_slicer (provisional); soft_demapper;
rrc_tx (seamless); rrc_rx (matched); ber_counter.

Verification highlights: LDPC en/decoder match `nrLDPCEncode/Decode` and recover
`ldpc_in.txt` in XSim (0 diff). soft_demapper matches a brute-force max-log oracle
over the project's own constellation (0 mismatches, ~100k points; 10,016 quantized
LLRs exact). rrc_tx/rrc_rx match a fixed-point FIR golden; the TX->RX cascade
recovers symbols at gain ~1.0 with <=3.4 LSB deviation (Nyquist zero-ISI confirmed
in fixed point). LLR deinterleaver round-trips 1040 LLRs exact.

### TO BUILD — TX side
- **code_block_seg** — TS 38.212 5.2.2. Segmentation + per-code-block CRC-24B.
  For K=520 the regime is C=1 (single code block); verify 5.2.2: when C=1 no
  CRC-24B is attached. Must be a standards-compliant block implementing the general
  rule, even though it is near-pass-through at current sizes.
- **rate_match** — TS 38.212 5.4.2: sub-block interleaver + circular-buffer bit
  selection (RV=0). Output length **E is a runtime input** (E in {1040, 624, ...}
  per the MCS table). Decision: the 5.4.2.2 bit interleaver stays a SEPARATE
  downstream block (existing `bit_interleaver`), so rate_match = sub-block
  interleave + selection only. Build path: first verify whether a MathWorks HDL
  rate-match block exists (and exposes E/RV as inputs) -> decide generate vs
  hand-write. Verify against `nrRateMatchLDPC`.
- **code_block_concat** — TS 38.212 5.5: concatenate rate-matched code blocks.
  C=1 -> pass-through, but a real standards-compliant block.
- **mcs_controller** — hand-written. Input: SNR/CQI. Output: MCS index -> (Qm, E).
  Broadcasts `qam_mode` to packer/mapper and E (+ N=E/Qm) to rate_match/interleaver;
  tags the packet with the MCS index. Core logic = threshold lookup from the MCS
  table (Section 9, not finalized).
- **MCS signalling insert** — tag each packet with the chosen MCS in a side-band
  field (`tuser`), since there is no PDCCH.

### TO BUILD — RX side
- **AGC** — automatic gain control (amplitude normalization).
- **Costas loop** — carrier phase/frequency recovery.
- **Gardner timing recovery** — symbol-timing recovery; performs the 1-of-4
  downsample of the matched-filter output (selects the correct sample per symbol).
- **equalizer (+ channel estimation)** — undo bounded multipath. Scope per Section 1.
- **SNR / channel-quality estimator** — measures channel quality; feeds mcs_controller
  (closes the adaptive loop).
- **soft_demapper** — DONE (listed above).
- **bit_deinterleaver (LLR)** — DONE.
- **rate_dematch** — inverse of rate_match; **LLR-valued (4-bit signed)**; rebuilds
  the 2600-LLR vector from E received LLRs, zero-filling punctured/unsent positions
  ("no information"); feeds the decoder. Design as rate_match's pair.
- **code_block_deseg** — inverse of code_block_seg.
- **code_block_de-concat** — inverse of code_block_concat.
- **MCS signalling recover** — read MCS from the recovered `tuser`; configure RX
  adaptive blocks (soft_demapper, deinterleaver, rate_dematch) with the matching
  Qm/E.

### TO BUILD — channel
- **channel** — AWGN core + bounded multipath / fading / interference, configurable
  Eb/N0. See Section 10 for the recommended build order and the open
  synthesizable-vs-testbench decision.

### TO BUILD — integration / deployment
- Full-chain integration testbench (coding loop first, then full modem).
- `tb_bit_chain` update to current packet sizes (496/520/2600/E).
- `qam_loopback_axi` — AXI-Lite wrapper for Pynq.
- Pynq Python driver.
- BER-vs-SNR sweep harness + MCS threshold refinement.

## 9. OPEN DECISIONS / KNOWN CHALLENGES

1. **Pynq Z2 resource fit (accepted-open).** Decoder OOC synthesis on XC7Z020:
   ~117,548 LUTs (221% — overflow), BRAM 139.5/140 (99.6%), FFs 91,591 (86%), DSP 0.
   The decoder alone does not fit. MathWorks NR LDPC Decoder is fixed
   high-parallelism with no area knob. Proceed functionally; revisit architecture
   later. Encoder real synthesis numbers: NOT yet captured (TODO: OOC synth).
2. **MCS table — NOT finalized.** Gates rate_match E values, mcs_controller, and
   signalling. Working assumption: QPSK/16/64/256 x {1/2, 5/6}; E in {1040, 624};
   thresholds TBD from measured BER curves. Pending: full 8-point grid vs a curated
   monotonic-efficiency ladder. (Note non-monotonicity: 16-QAM 5/6 ~3.33 b/s beats
   64-QAM 1/2 = 3.0 b/s.)
3. **MCS signalling (`tuser`) format — NOT finalized.**
4. **Channel impairment budget — NOT finalized** (delay taps, fading/Doppler rate,
   interference level). Must equal what the RX front-end is designed to resolve.
5. **Channel form — NOT decided:** synthesizable RTL noise generator vs
   testbench/simulation model vs MATLAB-imported noise samples (Section 10).

## 10. CHANNEL — BUILD GUIDANCE

The channel exists to be undone by the RX front-end, so the two are a matched pair.
Recommended approach:

- **Build AWGN first.** It is self-contained and testable now (the LDPC decoder is
  the thing that fights it — no front-end block needed to "undo" it). Enables early
  BER-vs-SNR work on the already-built chain.
- **Add other impairments alongside their correctors**, not up front:
  gain variation - AGC; carrier offset - Costas; timing offset - Gardner;
  multipath/fading - equalizer + channel estimation. Building these before their
  corrector exists gives nothing to test against and risks rework.
- **Open decision — how the channel exists:** on real hardware the "channel" is the
  RF path, not synthesized logic. Decide whether the channel is (a) a synthesizable
  RTL block (LFSR-based Gaussian noise via Box-Muller or sum-of-uniforms/CLT — note
  hardware Gaussian generation is non-trivial), (b) a testbench-only model, or
  (c) MATLAB-generated noise samples fed from a file. This changes what gets built.

## 11. VERIFICATION METHODOLOGY + TESTBENCH NOTES

- **Per-block TBs:** drive stimulus on `negedge aclk`, sample/dump on `negedge`,
  gate to the exact packet length, compare against a golden reference file or an
  in-TB computed expectation. Use continuous-input + gap-checking TBs for
  streaming/rate-changing blocks (e.g., rrc_tx is verified gap-free).
- **Generated-block flow:** generate via MATLAB -> `makehdl`/`makehdltb` -> XSim
  self-check, then hand-write the AXI-Stream wrapper and verify the wrapper with its
  own golden-reference TB.
- **TESTBENCH GOTCHA — string ternary in `$display`.** Do NOT write
  `$display(cond ? "PASS" : "FAIL");` — a string-valued ternary in `$display` is
  non-portable (Icarus prints garbage; XSim prints nothing). Always write:
  `if (cond) $display("PASS..."); else $display("FAIL...");`. The substantive pass
  condition is `ERRORS=0` plus the expected element count, regardless of the
  PASS/FAIL line.
- **Coefficients are hardcoded** as `localparam` in the filter `.v` files — no
  `.mem`/`$readmem` coefficient file is needed at simulation time. The only runtime
  files are stimulus/expected `.txt` vectors, which must sit in the simulator's run
  directory.
- **Filter group delay (alignment).** Each RRC filter delays by (33-1)/2 = 16
  samples; the TX+RX cascade delays by 32 samples = 8 symbols. The full-chain
  BER comparison MUST offset by this fixed delay ("received symbol n aligns to
  transmitted symbol n-8") or BER will read ~0.5 with a perfect channel. This is
  bookkeeping, not a bug; build the offset into the integration TB from the start.

## 12. FILE LAYOUT

```
rtl/
  bit_source.v  crc24a_attach.v  crc24a_check.v
  scrambler.v   descrambler.v
  bit_interleaver.v  bit_deinterleaver.v        (deinterleaver = LLR/4-bit version)
  symbol_packer.v  qam_mapper.v  qam_slicer.v
  soft_demapper.v
  rrc_tx.v  rrc_rx.v
  ber_counter.v
  ldpc_encoder.v   + encoder core: HDL_Algorithm.v, NR_LDPC_Encoder.v, SimpleDualPortRAM_generic*.v
  ldpc_decoder.v   + decoder core: dec_HDL_Algorithm.v, dec_NR_LDPC_Decoder.v, dec_SimpleDualPortRAM_generic*.v
tb/
  tb_*.v for each block
matlab/  (golden references + scripts)
  gen_ldpc_golden.m  run_ldpc_encoder_check.m  run_ldpc_decoder_check.m
  ldpc_in.txt (520)  ldpc_ref.txt (2600)  dec_llr_in.txt (2600 x 4-bit)
ref/    (non-RTL design/reference, do NOT add to Vivado)
  taps.py (RRC coefficient generator + zero-ISI proof)  taps_q14.txt
  ref.py  (soft_demapper max-log oracle)  vec_in/vec_exp, tx_in/tx_exp, rx_in/rx_exp
```

Vivado: add all `rtl/*.v` (including both LDPC core sets — encoder unprefixed,
decoder `dec_`-prefixed) as DESIGN sources; add TBs as SIMULATION sources; ensure no
old unprefixed decoder core files remain in `rtl/`. Hierarchy should show each
wrapper elaborating cleanly with no duplicate-module collisions.

## 13. SUGGESTED OWNERSHIP / NEXT ACTIONS

Independent chunks that can proceed in parallel (none blocked by the resource-fit or
MCS-table decisions except where noted):

- **TX coding:** rate_match (runtime E) + code_block_seg + code_block_concat.
  (rate_match's E set depends on the MCS table — use provisional {1040, 624} and
  refine.)
- **RX coding:** rate_dematch (pairs with rate_match) + code_block_deseg +
  code_block_de-concat. LLR-valued, 4-bit format.
- **RX front-end + channel (co-design):** AWGN channel first, then AGC / Costas /
  Gardner / equalizer / SNR estimator each with their matching impairment.
- **Adaptation:** mcs_controller + MCS signalling insert/recover — needs the MCS
  table decided first; drives the already-adaptive mapper/packer/interleaver/
  rate_match.
- **Integration:** full-chain TB (with the 8-symbol cascade-delay offset), then
  qam_loopback_axi + Pynq driver, then BER sweep + MCS threshold refinement.

When starting a new chat for a block, re-upload this document and state which block
is being built; the assistant should read Sections 2–8 and continue without
re-deriving settled facts, verifying any MATLAB/spec claim before stating it.
