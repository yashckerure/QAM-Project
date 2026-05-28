# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_symbol_packer.py
# Description  : Validates bit-to-symbol packing across all QAM modes.
# =============================================================================
import cocotb
import os
import sys
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(proj_root, "models", "python"))
import adaptive_qam as grm

@cocotb.test()
async def test_symbol_packer(dut):
    """Test symbol_packer across multiple QAM modes."""
    
    cocotb.start_soon(Clock(dut.aclk, 10, unit="ns").start())

    cases = [(0, 2, "QPSK"), (1, 4, "16QAM"), (2, 6, "64QAM"), (3, 8, "256QAM")]
    NUM_SYMS = 64
    SEED = 0x5A3C7E
    
    dut.aresetn.value = 0
    dut.m_axis_tready.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    await FallingEdge(dut.aclk)
    dut.aresetn.value = 1
    
    for mode_idx, bps, name in cases:
        dut._log.info(f"Testing mode {name} (bps={bps})")
        dut.qam_mode.value = mode_idx
        
        ideal_bits, _ = grm.prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        ideal_symbols = grm.pack_symbols(ideal_bits, bps)
        
        expected_symbols = []
        bit_idx = 0
        
        # We need to drive bits and check symbols
        while len(expected_symbols) < NUM_SYMS:
            await FallingEdge(dut.aclk)
            
            # Read output if valid
            if dut.m_axis_tvalid.value == 1:
                sym_val = int(dut.m_axis_tdata.value)
                sym_idx = len(expected_symbols)
                
                assert sym_val == ideal_symbols[sym_idx], f"[{name}] Match failed! HW: {sym_val:x}, Ideal: {ideal_symbols[sym_idx]:x}"
                dut._log.info(f"[{name}] Cycle {sym_idx:2d}: captured symbol = {sym_val:x} (binary: {sym_val:0{bps}b})")
                expected_symbols.append(sym_val)
            
            # Drive input if ready
            if dut.s_axis_tready.value == 1 and bit_idx < len(ideal_bits):
                dut.s_axis_tvalid.value = 1
                dut.s_axis_tdata.value = ideal_bits[bit_idx]
                bit_idx += 1
            else:
                dut.s_axis_tvalid.value = 0
                
            # Strobe sym_en (m_axis_tready) to simulate symbol consumption
            dut.m_axis_tready.value = 1 # Free running consumption
            
        dut._log.info(f"Passed {name}!")

if __name__ == "__main__":
    from cocotb_tools.runner import get_runner
    
    sim = os.getenv("SIM", "icarus")
    
    proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sim_dir = os.path.join(proj_path, "sim")
    build_dir = os.path.join(sim_dir, "sim_build")
    waves_dir = os.path.join(sim_dir, "waves")
    wave_file = os.path.join(waves_dir, "tb_symbol_packer.fst")
    
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [os.path.join(proj_path, "hw", "hdl", "src", "symbol_packer.v")]
    
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="symbol_packer",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="symbol_packer",
        test_module="test_symbol_packer",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
