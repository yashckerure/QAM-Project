# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_qam_mapper.py
# Description  : Validates arithmetic Gray-coding QAM symbol mapping.
# =============================================================================
import cocotb
import os
import sys
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(proj_root, "models", "python"))
import adaptive_qam as grm

def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

@cocotb.test()
async def test_qam_mapper(dut):
    """Test qam_mapper across multiple QAM modes."""
    
    cocotb.start_soon(Clock(dut.aclk, 10, unit="ns").start())

    cases = [(0, 2, "QPSK"), (1, 4, "16QAM"), (2, 6, "64QAM"), (3, 8, "256QAM")]
    NUM_SYMS = 64
    SEED = 0x5A3C7E
    
    dut.aresetn.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tuser.value = 0
    dut.m_axis_tready.value = 1
    await FallingEdge(dut.aclk)
    dut.aresetn.value = 1
    
    for mode_idx, bps, name in cases:
        dut._log.info(f"Testing mode {name} (bps={bps})")
        dut.qam_mode.value = mode_idx
        
        ideal_bits, _ = grm.prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        ideal_symbols = grm.pack_symbols(ideal_bits, bps)
        ideal_iq      = [grm.map_symbol(s, mode_idx, frac_w=10) for s in ideal_symbols]
        
        # Drive and Check
        for sym_idx, (ideal_i, ideal_q) in enumerate(ideal_iq):
            dut.s_axis_tvalid.value = 1
            dut.s_axis_tdata.value = ideal_symbols[sym_idx]
            dut.s_axis_tuser.value = bps
            
            await FallingEdge(dut.aclk)
            
            if dut.m_axis_tvalid.value == 1:
                # m_axis_tdata is {Q, I} packed 32-bit.
                packed = int(dut.m_axis_tdata.value)
                hw_q = twos_comp((packed >> 16) & 0xFFFF, 16)
                hw_i = twos_comp(packed & 0xFFFF, 16)
                
                assert hw_i == ideal_i, f"[{name}] I mismatch at {sym_idx}! HW: {hw_i}, Ideal: {ideal_i}"
                assert hw_q == ideal_q, f"[{name}] Q mismatch at {sym_idx}! HW: {hw_q}, Ideal: {ideal_q}"
                dut._log.info(f"[{name}] Cycle {sym_idx:2d}: I = {hw_i:6d}, Q = {hw_q:6d}")
                
        dut.s_axis_tvalid.value = 0
        await FallingEdge(dut.aclk)
        dut._log.info(f"Passed {name}!")

if __name__ == "__main__":
    from cocotb_tools.runner import get_runner
    
    sim = os.getenv("SIM", "icarus")
    
    proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sim_dir = os.path.join(proj_path, "sim")
    build_dir = os.path.join(sim_dir, "sim_build")
    waves_dir = os.path.join(sim_dir, "waves")
    wave_file = os.path.join(waves_dir, "tb_qam_mapper.fst")
    
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [os.path.join(proj_path, "hw", "hdl", "src", "qam_mapper.v")]
    
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="qam_mapper",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="qam_mapper",
        test_module="test_qam_mapper",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
