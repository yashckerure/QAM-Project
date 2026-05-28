# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_bit_source.py
# Description  : Validates the PRBS-23 bit generator.
# =============================================================================
import cocotb
import os
import sys
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge

# Add python models to path
proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(proj_root, "models", "python"))
import adaptive_qam as grm

@cocotb.test()
async def test_bit_source(dut):
    """Test that the PRBS-23 bit source generates valid bits predictably."""
    
    NUM_BITS = 128
    
    # 1. Start a 10ns clock
    cocotb.start_soon(Clock(dut.aclk, 10, unit="ns").start())

    NUM_BITS = 128
    
    # 2. Start with ready low to prove it holds state
    dut.m_axis_tready.value = 0
    
    # 2. Reset the module safely
    dut.m_axis_tready.value = 0
    dut.aresetn.value = 0
    await FallingEdge(dut.aclk)
    dut._log.info(f"After reset low: bit_out={dut.m_axis_tdata.value}")
    dut.aresetn.value = 1
    dut._log.info(f"After reset high: bit_out={dut.m_axis_tdata.value}")
    
    dut._log.info("Starting PRBS generation...")
    
    # GENERATE EXACT EXPECTED VALUES
    ideal_bits, _ = grm.prbs_stream(NUM_BITS, lfsr_w=23, seed=0x5A3C7E)
    expected_bits = []
    
    from cocotb.triggers import ReadOnly, RisingEdge
    
    dut.m_axis_tready.value = 1
    
    # Capture bits synchronously
    while len(expected_bits) < NUM_BITS:
        await ReadOnly()
        if dut.m_axis_tvalid.value == 1 and dut.m_axis_tready.value == 1:
            val = int(dut.m_axis_tdata.value)
            idx = len(expected_bits)
            
            # ASSERT AGAINST GOLDEN MODEL!
            assert val == ideal_bits[idx], f"Match failed at idx {idx}! HW: {val}, Ideal: {ideal_bits[idx]}"
            
            dut._log.info(f"Cycle {idx:3d}: captured bit = {val}")
            expected_bits.append(val)
            
        await RisingEdge(dut.aclk)

    assert len(expected_bits) == NUM_BITS, f"Failed to capture required number of bits"

if __name__ == "__main__":
    from cocotb_tools.runner import get_runner
    
    sim = os.getenv("SIM", "icarus")
    
    proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sim_dir = os.path.join(proj_path, "sim")
    build_dir = os.path.join(sim_dir, "sim_build")
    waves_dir = os.path.join(sim_dir, "waves")
    wave_file = os.path.join(waves_dir, "tb_bit_source.fst")
    
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [os.path.join(proj_path, "hw", "hdl", "src", "bit_source.v")]
    
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="bit_source",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="bit_source",
        test_module="test_bit_source",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
