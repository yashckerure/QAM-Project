# Adaptive QAM Modem — Project Conventions & Agent Context

> **Purpose:** This file is the single source of truth for all coding conventions,
> architectural decisions, and workflow rules followed in this project.
> Any AI agent or human contributor working on this codebase MUST read this
> file before making changes.

---

## 1. Project Structure

```
QAM-Project/
├── hw/
│   └── hdl/
│       └── src/              # All synthesizable Verilog RTL modules
│           ├── bit_source.v
│           ├── symbol_packer.v
│           ├── qam_mapper.v
│           ├── qam_slicer.v
│           └── ber_counter.v
├── models/
│   └── python/
│       └── adaptive_qam.py   # Bit-exact Python golden reference model
├── tb/
│   ├── tests/                # Python/Cocotb testbenches (one per module)
│   │   ├── test_bit_source.py
│   │   ├── test_symbol_packer.py
│   │   ├── test_qam_mapper.py
│   │   ├── test_qam_slicer.py
│   │   └── test_ber_counter.py
│   └── wrappers/             # Verilog wrappers for multi-module testbenches
│       └── tb_ber_counter.v
├── sim/                      # Generated simulation artifacts (git-ignored)
│   ├── sim_build/
│   └── waves/
├── docs/                     # Documentation and reference material
├── README.md                 # Architecture overview and 33-step roadmap
├── AGENTS.md                 # THIS FILE — conventions and rules
└── .gitignore
```

### Placement Rules
- **RTL source files** go in `hw/hdl/src/`. One module per file. Filename matches the module name.
- **Python golden models** go in `models/python/`. The single file `adaptive_qam.py` contains bit-exact functional models for every RTL block.
- **Cocotb testbenches** go in `tb/tests/`. One test file per RTL module. Naming convention: `test_<module_name>.py`.
- **Verilog wrappers** (for integration tests that instantiate multiple RTL modules) go in `tb/wrappers/`.
- **Simulation build artifacts** are generated under `sim/` and are git-ignored. Never commit them.
- **Never** place project source files in `/tmp`, `~/.gemini`, the Desktop, or anywhere outside the project root.

---

## 2. Verilog RTL Rules

### Language Standard
- **Strictly Verilog-2001.** No SystemVerilog constructs (`logic`, `always_ff`, `interface`, etc.).
- All code must be **fully synthesizable**. No `$display`, `$fwrite`, `$dumpfile`, `initial`, or other simulation-only constructs in RTL source files. These are permitted only in testbench wrappers under `tb/wrappers/`.

### Timescale
- Every `.v` file must include `` `timescale 1ns / 1ps `` before the module declaration.

### Reset Convention
- All modules use **asynchronous active-low reset** named `aresetn`.
- Reset is coded as: `always @(posedge aclk or negedge aresetn)`.

### Clock Convention
- The system clock is named `aclk` (following AXI naming).

### AXI4-Stream Interface Naming
All module-to-module data interfaces use the AXI4-Stream protocol. Signal names follow the ARM AMBA standard:

| Signal | Direction (Master) | Direction (Slave) | Purpose |
|--------|-------------------|-------------------|---------|
| `aclk` | input | input | Clock |
| `aresetn` | input | input | Active-low async reset |
| `m_axis_tdata` | output | — | Outgoing payload data |
| `m_axis_tvalid` | output | — | Master asserts when data is valid |
| `m_axis_tready` | input | — | Downstream slave signals it can accept |
| `m_axis_tuser` | output | — | Sideband metadata (e.g., bits-per-symbol) |
| `s_axis_tdata` | — | input | Incoming payload data |
| `s_axis_tvalid` | — | input | Upstream master asserts when data is valid |
| `s_axis_tready` | — | output | This module signals it can accept data |
| `s_axis_tuser` | — | input | Incoming sideband metadata |

**Key rules:**
- A data transfer occurs on the rising edge of `aclk` when BOTH `tvalid` AND `tready` are HIGH.
- Source-only modules (e.g., `bit_source`) have only `m_axis_*` ports.
- Sink-only modules (e.g., `ber_counter`) have only `s_axis_*` ports.
- Processing modules in the middle of the chain have both `s_axis_*` (input) and `m_axis_*` (output) ports.
- The `qam_mode` control signal is currently a standalone wire. In the future, it will migrate to in-band `tuser` metadata.

### Fixed-Point Format
- All internal I/Q DSP values use **signed 16-bit Q5.10** format (5 integer bits, 10 fractional bits).
- The fractional width is parameterized as `FRAC_W = 10`.
- The data width is parameterized as `DATA_W = 16`.
- The `qam_mapper` packs I and Q into a single 32-bit `m_axis_tdata` word as `{Q[15:0], I[15:0]}`.

### Commenting Style
- Use `// -----` separator bars to visually delineate logical sections within a module.
- Each logical block (decode logic, accumulator state, main FSM) gets a short section header comment.
- Comments must reference AXI4-Stream signal names (e.g., `s_axis_tready`, `m_axis_tvalid`), NOT legacy names (e.g., `bit_ready`, `sym_valid`).

---

## 3. File Header Format

### Verilog Files (`.v`)
```verilog
//=============================================================================
// Project      : Adaptive QAM Modem
// File         : <filename>.v
// Description  : Brief 1-2 sentence description.
//=============================================================================
// Additional Notes:
// - Specific architectural notes, AXI behavior, latency, etc.
//=============================================================================
`timescale 1ns / 1ps
```

### Python Files (`.py`)
```python
# =============================================================================
# Project      : Adaptive QAM Modem
# File         : <filename>.py
# Description  : Brief description of what the test suite verifies.
# =============================================================================
```

### Rules
- Do NOT use the auto-generated Vivado header template (the one with `Company:`, `Engineer:`, `Create Date:`, etc.). Strip it if it exists.
- Do NOT duplicate headers. Each file gets exactly one header block at the top.

---

## 4. Cocotb Testbench Rules

### Runner Infrastructure
- **No Makefiles.** All testbenches use the `cocotb_tools.runner` API and are executed directly as Python scripts: `python3 tb/tests/test_<module>.py`.
- The default simulator is **Icarus Verilog** (`icarus`), overridable via the `SIM` environment variable.
- Waveform dumps go to `sim/waves/` in FST format.
- Simulation builds go to `sim/sim_build/`.

### Testbench Template
Every cocotb test file follows this exact structure:

```python
# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_<module>.py
# Description  : <what it tests>
# =============================================================================
import cocotb
import os
import sys
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge

# Add python models to path
proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(proj_root, "models", "python"))
import adaptive_qam as grm

@cocotb.test()
async def test_<module>(dut):
    """Docstring describing the test."""

    cocotb.start_soon(Clock(dut.aclk, 10, unit="ns").start())

    # ... test body ...

if __name__ == "__main__":
    from cocotb_tools.runner import get_runner

    sim = os.getenv("SIM", "icarus")

    proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sim_dir = os.path.join(proj_path, "sim")
    build_dir = os.path.join(sim_dir, "sim_build")
    waves_dir = os.path.join(sim_dir, "waves")
    wave_file = os.path.join(waves_dir, "tb_<module>.fst")

    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [os.path.join(proj_path, "hw", "hdl", "src", "<module>.v")]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="<module>",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="<module>",
        test_module="test_<module>",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
```

### Verification Methodology
- **Golden model comparison:** Every test generates expected values from `adaptive_qam.py` and compares them cycle-by-cycle against the DUT output using `assert`.
- **Cycle-by-cycle logging:** Every test prints the captured output on each valid cycle using `dut._log.info(...)`. This is mandatory for debugging and traceability.
- **Mode sweep:** Every test must exercise ALL supported QAM modes: `cases = [(0, 2, "QPSK"), (1, 4, "16QAM"), (2, 6, "64QAM"), (3, 8, "256QAM")]`.
- **PRBS seed:** The canonical PRBS-23 seed used across all tests and the golden model is `0x5A3C7E`.
- **No file-based verification.** Do NOT write `.txt` reference files and diff them. All checking is done in-simulation via `assert`.

### AXI Signal Driving
- Drive AXI inputs on the **falling edge** of `aclk` (`await FallingEdge(dut.aclk)`).
- Sample AXI outputs on the **falling edge** or via `ReadOnly()` to avoid race conditions.
- Always initialize all input signals to 0 before releasing reset.
- Reset sequence: assert `aresetn = 0`, wait one falling edge, then deassert `aresetn = 1`.

---

## 5. Python Golden Model Rules

- The golden model lives in `models/python/adaptive_qam.py`.
- It must be **bit-exact** with respect to the Verilog RTL. No floating-point approximations — use integer arithmetic only.
- Each RTL block has a corresponding Python function (e.g., `prbs_stream()`, `pack_symbols()`, `map_symbol()`, `slice_iq()`, `ber_count()`).
- When adding a new RTL block, the corresponding golden model function MUST be added to `adaptive_qam.py` first, before writing the Verilog.
- The golden model is imported in cocotb tests as `import adaptive_qam as grm`.

---

## 6. Git Workflow

- **Branch naming:** Use descriptive kebab-case branch names (e.g., `axi-stream-refactor`, `rrc-fir-integration`).
- **Commit messages:** Use a short imperative summary line followed by a blank line and bullet-point details. Example:
  ```
  Refactor all modules to AXI4-Stream interfaces

  - Renamed clk/rst_n to aclk/aresetn
  - Added s_axis_*/m_axis_* port naming
  - Updated all cocotb testbenches to match
  ```
- **Do NOT commit to `main` directly** for non-trivial changes. Create a feature branch and merge after verification.
- **Always run the full test suite** before committing: all 5 testbenches must pass.

---

## 7. Design Decisions & Rationale

These decisions have been made and should NOT be revisited without explicit discussion:

| Decision | Rationale |
|----------|-----------|
| PRBS-23 (not PRBS-7 or PRBS-15) | Industry standard per ITU-T O.150. Long repeat period (2^23 - 1) generates worst-case bit patterns for testing. |
| Q5.10 fixed-point (not Q3.12) | 5 integer bits provide enough headroom for 256-QAM constellation levels (±15) without overflow. |
| 3GPP recursive magnitude (not ROM LUT) | Implements TS 38.211 §5.1 exactly. Interleaved bit ordering, sign-magnitude mapping, and `1/√K` power normalization are all 5G NR compliant. No BRAM consumed. |
| Cocotb + runner (not Makefiles) | Dynamic Python runners are simpler, avoid Makefile debugging, and integrate naturally with `pytest`. |
| AXI4-Stream (not ad-hoc valid/ready) | Industry standard. Ensures compatibility with Xilinx IP cores (FIR Compiler, DMA, etc.) and future Pynq/Zynq deployment. |
| 1-bit-per-clock serial bit source | Decouples the PRBS generator from the modulation order. The symbol_packer handles the serial-to-parallel conversion. |

---

## 8. Adding a New Block — Checklist

When implementing a new block (e.g., block 6 `rrc_fir`), follow this exact sequence:

1. **Golden model first.** Add the Python function to `adaptive_qam.py` and verify it standalone.
2. **Write the RTL module** in `hw/hdl/src/<module>.v` using the standard header and AXI4-Stream ports.
3. **Write the cocotb testbench** in `tb/tests/test_<module>.py` using the template above.
4. **Test all QAM modes.** The `cases` list must include QPSK, 16-QAM, 64-QAM, and 256-QAM.
5. **Add cycle-by-cycle logging.** Every valid output cycle must log via `dut._log.info(...)`.
6. **Run the full suite.** All existing tests must still pass. The new module must not break any downstream module.
7. **Commit** on a feature branch with a descriptive message.

---

## 9. Key Constants & Parameters

| Constant | Value | Used In |
|----------|-------|---------|
| PRBS Seed | `0x5A3C7E` | `bit_source.v`, `ber_counter.v`, all testbenches |
| LFSR Width | 23 | `bit_source.v`, `ber_counter.v` |
| PRBS Polynomial | x^23 + x^18 + 1 | Tap positions: bits 22 and 17 |
| DATA_W | 16 | All I/Q data paths |
| FRAC_W | 10 | Q5.10 fixed-point fractional bits |
| MAX_BPS | 8 | Maximum bits per symbol (256-QAM) |
| Clock Period | 10 ns | All cocotb testbenches |
| Simulator | Icarus Verilog | Default, overridable via `SIM` env var |

---

## 10. Current Status & What Comes Next

Refer to `README.md` for the full 33-step master roadmap and milestone definitions.

**Milestone 1 is COMPLETE.** All 5 core blocks are implemented, verified across all 4 QAM modes, and committed. The next block to implement is **block 6: `rrc_fir`** (Root Raised Cosine pulse-shaping filter).
