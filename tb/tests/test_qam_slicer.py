# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_qam_slicer.py
# Description  : Validates inverse arithmetic slicing (I/Q to Symbol).
# =============================================================================
import cocotb
import os
import sys
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge

proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(proj_root, "models", "python"))
import adaptive_qam as grm

@cocotb.test()
async def test_qam_slicer(dut):
    """Test qam_slicer across multiple QAM modes."""
    
    cocotb.start_soon(Clock(dut.aclk, 10, unit="ns").start())

    cases = [(0, 2, "QPSK"), (1, 4, "16QAM"), (2, 6, "64QAM"), (3, 8, "256QAM")]
    NUM_SYMS = 64
    SEED = 0x5A3C7E
    
    dut.aresetn.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.m_axis_tready.value = 1
    await FallingEdge(dut.aclk)
    dut.aresetn.value = 1
    
    for mode_idx, bps, name in cases:
        dut._log.info(f"Testing mode {name} (bps={bps})")
        dut.qam_mode.value = mode_idx
        
        ideal_bits, _ = grm.prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        ideal_symbols = grm.pack_symbols(ideal_bits, bps)
        ideal_iq      = [grm.map_symbol(s, mode_idx, frac_w=10) for s in ideal_symbols]
        ideal_sliced  = [grm.slice_iq(I, Q, mode_idx, frac_w=10) for (I, Q) in ideal_iq]
        
        # Drive and Check
        for sym_idx, (i_val, q_val) in enumerate(ideal_iq):
            dut.s_axis_tvalid.value = 1
            
            # Python negative to two's complement for cocotb
            if i_val < 0: i_val += (1 << 16)
            if q_val < 0: q_val += (1 << 16)
                
            dut.s_axis_tdata.value = (q_val << 16) | i_val
            
            await FallingEdge(dut.aclk)
            
            if dut.m_axis_tvalid.value == 1:
                hw_sym = int(dut.m_axis_tdata.value)
                ideal_sym = ideal_sliced[sym_idx]
                
                assert hw_sym == ideal_sym, f"[{name}] Mismatch at {sym_idx}! HW: {hw_sym:x}, Ideal: {ideal_sym:x}"
                dut._log.info(f"[{name}] Cycle {sym_idx:2d}: sliced symbol = {hw_sym:x} (binary: {hw_sym:0{bps}b})")
                
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
    wave_file = os.path.join(waves_dir, "tb_qam_slicer.fst")
    
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [os.path.join(proj_path, "hw", "hdl", "src", "qam_slicer.v")]
    
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="qam_slicer",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="qam_slicer",
        test_module="test_qam_slicer",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
