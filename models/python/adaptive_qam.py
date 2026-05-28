"""
adaptive_qam.py - Python golden reference for adaptive QAM modem.
Bit-exact functional models for each Verilog block.

Blocks:
  1. bit_source     (PRBS-23, 1 bit per clock, mode-agnostic)
  2. symbol_packer  (1-bit serial -> Qm-bit parallel, MSB-first)
  3. qam_mapper     (Qm-bit symbol -> (I, Q) in Q5.10, arithmetic Gray-PAM)
  4. qam_slicer     (hard-decision inverse of qam_mapper)
  5. ber_counter    (regenerated-PRBS comparator, bit error counter)
"""
import numpy as np


# =============================================================================
# Block 1: PRBS bit source
# =============================================================================

def prbs_step(lfsr, lfsr_w):
    if lfsr_w == 23:
        tap_a, tap_b = 22, 17
    elif lfsr_w == 15:
        tap_a, tap_b = 14, 13
    else:
        raise ValueError(f"Unsupported LFSR width: {lfsr_w}")
    mask    = (1 << lfsr_w) - 1
    out_bit = (lfsr >> (lfsr_w - 1)) & 1
    fb      = ((lfsr >> tap_a) ^ (lfsr >> tap_b)) & 1
    lfsr    = ((lfsr << 1) | fb) & mask
    return out_bit, lfsr


def prbs_stream(n_bits, lfsr_state=None, lfsr_w=23, seed=0x5A3C7E):
    if lfsr_state is None:
        lfsr = seed & ((1 << lfsr_w) - 1)
    else:
        lfsr = lfsr_state & ((1 << lfsr_w) - 1)
    bits = []
    for _ in range(n_bits):
        b, lfsr = prbs_step(lfsr, lfsr_w)
        bits.append(b)
    return bits, lfsr


# =============================================================================
# Block 2: Symbol packer
# =============================================================================

def pack_symbols(bits, bps):
    n_full = len(bits) // bps
    symbols = []
    for s in range(n_full):
        sym = 0
        for j in range(bps):
            sym = (sym << 1) | bits[s * bps + j]
        symbols.append(sym)
    return symbols


# =============================================================================
# Block 3: QAM mapper
# =============================================================================

def get_scale_factor(qam_mode):
    if qam_mode == 0: return 724
    if qam_mode == 1: return 324
    if qam_mode == 2: return 158
    if qam_mode == 3: return 79
    return 724

def extract_iq_bits(sym_bits, bps_axis):
    bps_total = bps_axis * 2
    i_bits = 0
    q_bits = 0
    for k in range(bps_axis):
        i_bit = (sym_bits >> (bps_total - 1 - 2*k)) & 1
        q_bit = (sym_bits >> (bps_total - 2 - 2*k)) & 1
        i_bits = (i_bits << 1) | i_bit
        q_bits = (q_bits << 1) | q_bit
    return i_bits, q_bits

def bits_to_mag(bits, bps_axis):
    if bps_axis == 1:
        return 1
    elif bps_axis == 2:
        return 1 + 2 * (bits & 1)
    elif bps_axis == 3:
        val0 = 1 + 2 * (bits & 1)
        b1 = (bits >> 1) & 1
        return (4 + val0) if b1 else (4 - val0)
    elif bps_axis == 4:
        val0 = 1 + 2 * (bits & 1)
        b2 = (bits >> 1) & 1
        val1 = (4 + val0) if b2 else (4 - val0)
        b1 = (bits >> 2) & 1
        return (8 + val1) if b1 else (8 - val1)
    return 1

def map_symbol(sym_bits, qam_mode, frac_w=10):
    bps_axis_table = [1, 2, 3, 4]
    bps_axis = bps_axis_table[qam_mode]
    i_bits, q_bits = extract_iq_bits(sym_bits, bps_axis)
    
    i_mag = bits_to_mag(i_bits, bps_axis)
    q_mag = bits_to_mag(q_bits, bps_axis)
    
    scale_factor = get_scale_factor(qam_mode)
    
    i_scaled = i_mag * scale_factor
    q_scaled = q_mag * scale_factor
    
    i_sign = (i_bits >> (bps_axis - 1)) & 1
    q_sign = (q_bits >> (bps_axis - 1)) & 1
    
    i_level = -i_scaled if i_sign else i_scaled
    q_level = -q_scaled if q_sign else q_scaled
    
    return i_level, q_level


def twos16(v):
    return int(v) & 0xFFFF


# =============================================================================
# Block 4: QAM slicer
# =============================================================================

def get_inv_scale_factor(qam_mode):
    if qam_mode == 0: return 5793
    if qam_mode == 1: return 12953
    if qam_mode == 2: return 26545
    if qam_mode == 3: return 53406
    return 5793

def mag_to_bits(mag_est, bps_axis, sign_bit):
    max_mag = (1 << bps_axis) - 1
    if mag_est > max_mag: mag_est = max_mag
    val = mag_est >> 1
    bits = sign_bit << (bps_axis - 1)
    if bps_axis > 1:
        b1 = (val >> (bps_axis - 2)) & 1
        bits |= (b1 << (bps_axis - 2))
    if bps_axis > 2:
        v_curr = (val >> (bps_axis - 3)) & 1
        v_prev = (val >> (bps_axis - 2)) & 1
        b2 = v_curr ^ (1 - v_prev)
        bits |= (b2 << (bps_axis - 3))
    if bps_axis > 3:
        v_curr = (val >> (bps_axis - 4)) & 1
        v_prev = (val >> (bps_axis - 3)) & 1
        b3 = v_curr ^ (1 - v_prev)
        bits |= (b3 << (bps_axis - 4))
    return bits

def interleave_iq_bits(i_bits, q_bits, bps_axis):
    sym_bits = 0
    for k in range(bps_axis):
        i_bit = (i_bits >> (bps_axis - 1 - k)) & 1
        q_bit = (q_bits >> (bps_axis - 1 - k)) & 1
        sym_bits = (sym_bits << 1) | i_bit
        sym_bits = (sym_bits << 1) | q_bit
    return sym_bits

def slice_iq(i_sample, q_sample, qam_mode, frac_w=10):
    bps_axis_table = [1, 2, 3, 4]
    bps_axis  = bps_axis_table[qam_mode]
    inv_scale = get_inv_scale_factor(qam_mode)

    def slice_axis(sample):
        sign_bit = 1 if sample < 0 else 0
        abs_val = -sample if sample < 0 else sample
        prod = abs_val * inv_scale
        mag_est = (prod + (1 << 21)) >> 22
        return mag_to_bits(mag_est, bps_axis, sign_bit)

    i_bits = slice_axis(i_sample)
    q_bits = slice_axis(q_sample)
    return interleave_iq_bits(i_bits, q_bits, bps_axis)


# =============================================================================
# Block 5: BER counter
# =============================================================================

def unpack_symbol(sym, bps):
    """MSB-first bit list of a bps-bit symbol."""
    return [(sym >> (bps - 1 - j)) & 1 for j in range(bps)]


def ber_count(sliced_symbols, qam_mode, seed=0x5A3C7E, lfsr_w=23):
    """
    Regenerate PRBS-23 from seed, unpack each sliced symbol MSB-first, compare.
    Returns (bit_errors, bits_compared).
    """
    bps_table = [2, 4, 6, 8]
    bps = bps_table[qam_mode]
    total_bits = len(sliced_symbols) * bps
    ref_bits, _ = prbs_stream(total_bits, lfsr_w=lfsr_w, seed=seed)

    errs = 0
    cnt  = 0
    for s_idx, sym in enumerate(sliced_symbols):
        rx_bits = unpack_symbol(sym, bps)
        for j in range(bps):
            if rx_bits[j] != ref_bits[s_idx * bps + j]:
                errs += 1
            cnt += 1
    return errs, cnt


# =============================================================================
if __name__ == "__main__":
    pass