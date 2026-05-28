# =============================================================================
# Project      : Adaptive QAM Modem
# File         : qam_modem.py
# Description  : Python driver for the Adaptive QAM Modem PYNQ Overlay
# =============================================================================

import time
import os
try:
    from pynq import Overlay
except ImportError:
    print("WARNING: pynq module not found. This script must be run on a PYNQ board.")
    Overlay = object # Mock

class AdaptiveQAMModem:
    # Register Map Offsets
    REG_CTRL        = 0x00
    REG_STATUS      = 0x04
    REG_BIT_COUNT   = 0x08
    REG_ERR_COUNT   = 0x0C
    REG_CFG_SPS     = 0x10
    REG_TARGET_BITS = 0x14
    
    QAM_MODES = {
        "QPSK":   0,
        "16-QAM": 1,
        "64-QAM": 2,
        "256-QAM": 3
    }

    def __init__(self, bitfile_path="qam_modem.bit"):
        if not os.path.exists(bitfile_path):
            raise FileNotFoundError(f"Bitstream file not found: {bitfile_path}")
            
        print(f"Loading overlay from {bitfile_path}...")
        self.overlay = Overlay(bitfile_path)
        
        # The IP block name in Vivado is 'qam_0'
        if hasattr(self.overlay, 'qam_0'):
            self.mmio = self.overlay.qam_0
        else:
            raise RuntimeError("IP block 'qam_0' not found in the overlay hierarchy.")
            
        self.reset_modem()
        
    def reset_modem(self):
        """Toggle the software reset bit"""
        # Set bit 5 (reset) high
        ctrl = self.mmio.read(self.REG_CTRL)
        self.mmio.write(self.REG_CTRL, ctrl | (1 << 5))
        time.sleep(0.01)
        # Clear reset
        self.mmio.write(self.REG_CTRL, ctrl & ~(1 << 5))
        
    def set_qam_mode(self, mode_name):
        if mode_name not in self.QAM_MODES:
            raise ValueError(f"Invalid mode. Choose from: {list(self.QAM_MODES.keys())}")
            
        val = self.QAM_MODES[mode_name]
        ctrl = self.mmio.read(self.REG_CTRL)
        # Clear bits 2:0 and set new mode
        ctrl = (ctrl & ~0x07) | (val & 0x07)
        self.mmio.write(self.REG_CTRL, ctrl)
        
    def set_sps(self, sps):
        """Set Samples Per Symbol (throttles the modem speed)"""
        self.mmio.write(self.REG_CFG_SPS, int(sps))
        
    def set_target_bits(self, target):
        """Set the number of bits to compare before stopping"""
        self.mmio.write(self.REG_TARGET_BITS, int(target))
        
    def start_ber_test(self):
        """Enable the BER counter and kick off the test"""
        self.reset_modem()
        ctrl = self.mmio.read(self.REG_CTRL)
        # Set bit 4 (ber_enable)
        self.mmio.write(self.REG_CTRL, ctrl | (1 << 4))
        
    def is_done(self):
        """Check if the BER test has finished"""
        status = self.mmio.read(self.REG_STATUS)
        return (status & 0x01) == 1
        
    def wait_for_done(self, timeout=10.0):
        """Block until the test finishes or times out"""
        start = time.time()
        while not self.is_done():
            if (time.time() - start) > timeout:
                raise TimeoutError("BER test timed out!")
            time.sleep(0.05)
            
    def get_bits_compared(self):
        return self.mmio.read(self.REG_BIT_COUNT)
        
    def get_bit_errors(self):
        return self.mmio.read(self.REG_ERR_COUNT)

if __name__ == "__main__":
    pass # See the deployment guide for test scripts
