# =============================================================================
# Project      : Adaptive QAM Modem
# File         : test_ber_counter.py
# Description  : Full pipeline integration and zero-BER loopback test.
# =============================================================================
import cocotb
import os
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge

@cocotb.test()
async def test_ber_counter(dut):
    """Test full loopback chain: bit_source -> packer -> mapper -> slicer -> ber_counter."""
    
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    cases = [(0, 2, "QPSK"), (1, 4, "16QAM"), (2, 6, "64QAM"), (3, 8, "256QAM")]
    SPS = 4
    
    dut.rst_n.value = 0
    dut.sym_en.value = 0
    dut.qam_mode.value = 0
    dut.ber_enable.value = 0
    
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    
    for mode_idx, bps, name in cases:
        dut._log.info(f"Testing loopback mode {name} (bps={bps})")
        
        dut.rst_n.value = 0
        dut.qam_mode.value = mode_idx
        dut.ber_enable.value = 0
        
        for _ in range(5):
            await FallingEdge(dut.clk)
            
        dut.rst_n.value = 1
        dut.ber_enable.value = 1
        
        clk_cnt = 0
        
        while dut.status_done.value == 0:
            await FallingEdge(dut.clk)
            
            # sym_en strobe generator
            if clk_cnt == SPS - 1:
                clk_cnt = 0
                dut.sym_en.value = 1
                if int(dut.bits_compared.value) > 0:
                    dut._log.info(f"[{name}] Compared: {int(dut.bits_compared.value):4d} bits, Errors: {int(dut.bit_errors.value)}")
            else:
                clk_cnt += 1
                dut.sym_en.value = 0
                
        dut.ber_enable.value = 0
        
        errs = int(dut.bit_errors.value)
        cnt = int(dut.bits_compared.value)
        
        dut._log.info(f"[{name}] Done! errors={errs}, bits={cnt}")
        assert errs == 0, f"[{name}] Loopback failed! Expected 0 errors, got {errs}"

if __name__ == "__main__":
    from cocotb_tools.runner import get_runner
    
    sim = os.getenv("SIM", "icarus")
    
    proj_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sim_dir = os.path.join(proj_path, "sim")
    build_dir = os.path.join(sim_dir, "sim_build")
    waves_dir = os.path.join(sim_dir, "waves")
    wave_file = os.path.join(waves_dir, "tb_ber_counter.fst")
    
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(waves_dir, exist_ok=True)

    sources = [
        os.path.join(proj_path, "hw", "hdl", "src", "bit_source.v"),
        os.path.join(proj_path, "hw", "hdl", "src", "symbol_packer.v"),
        os.path.join(proj_path, "hw", "hdl", "src", "qam_mapper.v"),
        os.path.join(proj_path, "hw", "hdl", "src", "qam_slicer.v"),
        os.path.join(proj_path, "hw", "hdl", "src", "ber_counter.v"),
        os.path.join(proj_path, "tb", "wrappers", "tb_ber_counter.v")
    ]
    
    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel="tb_ber_counter",
        always=True,
        waves=True,
        build_dir=build_dir
    )
    runner.test(
        hdl_toplevel="tb_ber_counter",
        test_module="test_ber_counter",
        waves=True,
        plusargs=[f"+{sim}.dumpfile={wave_file}"],
        test_dir=waves_dir
    )
