# Adaptive QAM Modem — 5G NR Compliant

## Overview

This project is a fully synthesizable, adaptive digital QAM modem designed for FPGA deployment on the **PYNQ-Z2 (Zynq-7020)**. It is written in strict **Verilog-2001** and uses the **AXI4-Stream** protocol for all internal data routing.

The modulation and demodulation logic is compliant with **3GPP TS 38.211 V19.3.0 (2025-03)**, *"NR; Physical channels and modulation"*, which defines the constellation mapping for the 5G New Radio physical layer.

The modem adaptively switches between **QPSK, 16-QAM, 64-QAM, and 256-QAM** while maintaining a strict cycle-accurate pipelined architecture.

---

## 5G NR Standards Compliance

### Reference Document

> **3GPP TS 38.211 V19.3.0 (2025-03)**
> *"NR; Physical channels and modulation"*
> **Section 5.1**: Modulation mapper
> **Tables 5.1-1 through 5.1-4**: Constellation definitions for QPSK, 16-QAM, 64-QAM, 256-QAM

### 3GPP Modulation Mapper Formula (TS 38.211 §5.1)

The 3GPP standard defines the complex-valued modulation symbol as:

```
d(i) = (1/√K) × [(1 - 2·b_I(0)) × { ... recursive magnitude ... }]
     + j·(1/√K) × [(1 - 2·b_Q(0)) × { ... recursive magnitude ... }]
```

Where:
- `b(0), b(1), ..., b(Qm-1)` are the input bits for a single symbol
- `Qm` is the modulation order in bits (2, 4, 6, or 8)
- `K` is the average power normalization constant
- Bits are **interleaved** between I and Q axes: `b(0)→I, b(1)→Q, b(2)→I, b(3)→Q, ...`
- `b(0)` is the **MSB** (transmitted first)

### Bit-to-Axis Assignment (Interleaved)

The 3GPP standard interleaves bits between the I and Q axes. For a symbol with `Qm` bits:

| Modulation | Qm | I-axis bits | Q-axis bits |
|---|---|---|---|
| QPSK | 2 | b(0) | b(1) |
| 16-QAM | 4 | b(0), b(2) | b(1), b(3) |
| 64-QAM | 6 | b(0), b(2), b(4) | b(1), b(3), b(5) |
| 256-QAM | 8 | b(0), b(2), b(4), b(6) | b(1), b(3), b(5), b(7) |

This is **different** from the textbook convention of splitting the symbol in half (upper bits → I, lower bits → Q). Our implementation follows the 3GPP interleaved ordering exactly.

### Recursive Magnitude Generation

Per TS 38.211 Tables 5.1-1 through 5.1-4, the magnitude for each axis is computed recursively from the **inner bits** (all bits except the sign bit):

| Modulation | Magnitude formula (per axis) |
|---|---|
| QPSK | `mag = 1` (always) |
| 16-QAM | `mag = 1 + 2·b_lsb` → produces {1, 3} |
| 64-QAM | `mag = (4 ± (1 + 2·b_lsb))` → produces {1, 3, 5, 7} |
| 256-QAM | `mag = (8 ± (4 ± (1 + 2·b_lsb)))` → produces {1, 3, 5, ..., 15} |

The `±` sign at each level is determined by the corresponding inner bit: `+` if the bit is `1`, `−` if `0`.

The **sign bit** (MSB of each axis, i.e., `b(0)` for I and `b(1)` for Q) determines the final polarity:

```
level = (-1)^b_sign × magnitude
```

### Average Power Normalization Constants

The 3GPP standard specifies that constellation points must be normalized so the average power equals 1.0. This is achieved by dividing by `√K`:

| Modulation | K (avg. power) | Scale = 1/√K | Fixed-Point (Q0.10) |
|---|---|---|---|
| QPSK | 2 | 1/√2 ≈ 0.7071 | `round(0.7071 × 1024) = 724` |
| 16-QAM | 10 | 1/√10 ≈ 0.3162 | `round(0.3162 × 1024) = 324` |
| 64-QAM | 42 | 1/√42 ≈ 0.1543 | `round(0.1543 × 1024) = 158` |
| 256-QAM | 170 | 1/√170 ≈ 0.0767 | `round(0.0767 × 1024) = 79` |

Where `K = (2/3)(M − 1)` for an M-QAM constellation, matching the values in TS 38.211.

The output I and Q values are in **signed 16-bit Q5.10** fixed-point format:
```
I_out = sign × magnitude × scale_factor    (in Q5.10)
Q_out = sign × magnitude × scale_factor    (in Q5.10)
```

### Slicer Inverse Scaling

The `qam_slicer` performs the exact inverse of the mapper. It recovers the integer magnitude from the Q5.10 sample by multiplying with the inverse scale factor `√K`:

| Modulation | Inverse Scale = √K | Fixed-Point (Q5.12) |
|---|---|---|
| QPSK | √2 ≈ 1.4142 | `round(1.4142 × 4096) = 5793` |
| 16-QAM | √10 ≈ 3.1623 | `round(3.1623 × 4096) = 12953` |
| 64-QAM | √42 ≈ 6.4807 | `round(6.4807 × 4096) = 26545` |
| 256-QAM | √170 ≈ 13.0384 | `round(13.0384 × 4096) = 53406` |

The recovered magnitude is then decoded back to bits using the inverse of the recursive structure, and interleaved back into the Qm-bit symbol.

---

## Repository Structure

- **`hw/hdl/src/`**: All synthesizable Verilog RTL modules.
- **`tb/tests/`**: Python-based testbenches using the Cocotb framework (`cocotb_tools.runner`, no Makefiles).
- **`tb/wrappers/`**: Verilog wrappers used exclusively for multi-module testbench port mapping.
- **`models/python/`**: Bit-exact Python golden reference models. All verification is done in-simulation via `assert` — no `.txt` reference files.
- **`docs/`**: 3GPP specification documents and reference material.

---

## Current Architecture & Data Path

Every block in the pipeline adheres to the **AXI4-Stream** protocol standard with `tdata`, `tvalid`, `tready` handshaking.

```
┌────────────┐   ┌────────────────┐   ┌─────────────┐   ┌─────────────┐   ┌──────────────┐
│ bit_source │──→│ symbol_packer  │──→│ qam_mapper  │──→│ qam_slicer  │──→│ ber_counter  │
│ (PRBS-23)  │   │ (serial→Qm)   │   │ (3GPP §5.1) │   │ (inverse)   │   │ (error count)│
└────────────┘   └────────────────┘   └─────────────┘   └─────────────┘   └──────────────┘
     1 bit/clk        Qm bits/sym        {Q,I} Q5.10       Qm bits/sym        BER report
```

### Block Descriptions

| # | Module | Function | 3GPP Reference |
|---|--------|----------|----------------|
| 1 | `bit_source.v` | PRBS-23 generator, 1 bit/clock, ITU-T O.150 compliant | — |
| 2 | `symbol_packer.v` | Serial-to-parallel: packs Qm serial bits into one symbol | — |
| 3 | `qam_mapper.v` | 3GPP modulation mapper with interleaved bits, recursive magnitude, and power normalization | TS 38.211 §5.1, Tables 5.1-1 to 5.1-4 |
| 4 | `qam_slicer.v` | Hard-decision inverse mapper: inverse scaling, magnitude-to-bit decoding, interleaved reassembly | Inverse of TS 38.211 §5.1 |
| 5 | `ber_counter.v` | Regenerated-PRBS bit-error comparator | — |

### Design Patterns

- **AXI4-Stream everywhere**: Guarantees compatibility with Xilinx IP cores (FIR Compiler, DMA) and future Zynq deployment.
- **No ROM lookup tables**: The 3GPP recursive magnitude formulas are implemented as combinational logic — no BRAM consumed.
- **Q5.10 fixed-point**: 5 integer bits provide headroom for 256-QAM levels (±15) after normalization, 10 fractional bits retain precision.
- **Single-cycle throughput**: Both mapper and slicer produce one output per clock cycle.

---

## Verification Status

**Milestone 1 is COMPLETE: Zero-BER deterministic loopback verified across all 4 QAM modes.**

All verification uses the `cocotb` + `cocotb_tools.runner` infrastructure. Golden models in `adaptive_qam.py` generate bit-exact reference streams. Hardware outputs are compared cycle-by-cycle via in-simulation `assert` statements.

| Block | Module | Status | Verification |
|-------|--------|--------|--------------|
| 1 | `bit_source.v` | ✅ Done | PRBS-23 sequences verified cycle-by-cycle against golden model |
| 2 | `symbol_packer.v` | ✅ Done | QPSK, 16-QAM, 64-QAM, 256-QAM packing verified |
| 3 | `qam_mapper.v` | ✅ Done | 3GPP-compliant mapping with normalization verified across all 4 modes |
| 4 | `qam_slicer.v` | ✅ Done | Inverse 3GPP decoding verified — perfect round-trip across all 4 modes |
| 5 | `ber_counter.v` | ✅ Done | End-to-end loopback: 0 BER over 4096+ bits for QPSK and 16-QAM |

### Running the Test Suite

```bash
cd tb/tests/
python test_bit_source.py
python test_symbol_packer.py
python test_qam_mapper.py
python test_qam_slicer.py
python test_ber_counter.py
```

Default simulator is **Icarus Verilog**, overridable via `SIM` environment variable.

---

## 33-Step Master Roadmap

The project is driven by the following block-by-block master plan. Every subsequent block is additive — improving the modem without risking the foundational milestones.

### Milestone 1: Zero-BER deterministic loopback ✅ COMPLETE
*Prove the bit path works. No noise yet, no shaping yet, just wiring.*
1. **`bit_source`** ✅ — PRBS-23 info source, 1 bit/clk
2. **`symbol_packer`** ✅ — Qm bits per symbol
3. **`qam_mapper`** ✅ — 3GPP TS 38.211 §5.1 compliant: bits → (I, Q) with interleaved ordering, recursive magnitude, and power normalization
4. **`qam_slicer`** ✅ — hard-decision inverse mapper (I, Q) → bits with inverse scaling and 3GPP bit recovery
5. **`ber_counter`** ✅ — closes the bit loop, count mismatches

### Milestone 2: BER curves vs SNR for QPSK, 16-QAM
*First demoable thing. Working uncoded QAM modem.*
6. **`rrc_fir`** (TX + RX) — pulse shaping + matched filter
7. **`awgn_channel`** — additive noise
8. **`sym_strobe_gen`** — symbol-rate sampler ("cheat" timing recovery)

### Milestone 3: Hardware Deployment
*This is the foundational milestone. Working hardware modem on real silicon.*
9. **Pynq deployment** — AXI plumbing, UART loopback, live on board

### Milestone 4: Realistic channel runs through the chain
*Introducing real-world RF impairments.*
10. **`multipath_fir`** — 3-5 tap FIR for ISI simulation
11. **`cfo_block`** — complex multiply by exp(j·φ) for carrier offset
12. **`phase_noise`** — slow random walk on φ for real-world impairment
13. **`sco_farrow`** — fractional resampler for sample clock mismatch
14. **`tone_interferer`** — NCO + adder to simulate a jammer

### Milestone 5: Locked, equalized signal, soft bits ready for FEC
15. **`agc`** — gain loop
16. **`costas_loop`** — carrier recovery
17. **`gardner_ted`** — timing recovery, replaces block 8
18. **`lms_equalizer`** — adaptive FIR
19. **`soft_demapper`** — replaces block 4 slicer, produces LLR output

### Milestone 6: Full 5G PDSCH bit chain working
*64-QAM and 256-QAM unlock here.*
20. **`crc24a`** — TB-level CRC (TS 38.212 §6.2.1)
21. **`cb_segment`** — code block split + CRC-24B (TS 38.212 §6.2.3)
22. **`ldpc_encoder`** — BG1/BG2 (TS 38.212 §6.2.4)
23. **`rate_match`** — circular buffer + bit selection (TS 38.212 §6.2.5)
24. **`cb_concat`** — concatenate code blocks (TS 38.212 §6.2.6)
25. **`scrambler`** — 31-bit Gold sequence (TS 38.211 §7.3.1.1)
26. **`bit_interleaver`** — Qm × E/Qm matrix (TS 38.211 §5.3.3)
*RX-side counterparts of 20-26 in reverse order:*
27. **`ldpc_decoder`** — NMS, soft-input
28. **`cb_desegment`** — check CRC-24B per CB
29. **`crc_check`** — check CRC-24A on full TB

### Milestone 7: Closed-loop adaptive 5G PDSCH modem
*This is the headline deliverable.*
30. **`snr_estimator`** — slicer error variance
31. **`cqi_quantizer`** — map SNR → CQI (TS 38.214 Table 5.2.2.1-3)
32. **`mcs_selector_fsm`** — closes the adaptation loop (TS 38.214 Table 5.1.3.1-2)
33. **`preamble_framer`** — mode signaling
