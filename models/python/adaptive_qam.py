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


def verify_bit_source():
    NUM_BITS = 4096
    bits, final_state = prbs_stream(NUM_BITS, lfsr_w=23, seed=0x5A3C7E)
    with open("bits_ref.txt", "w") as f:
        for b in bits:
            f.write(f"{b}\n")
    print(f"[bit_source] Wrote {NUM_BITS} reference bits to bits_ref.txt")


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


def write_symbols_file(filename, symbols, bps):
    hex_width = (bps + 3) // 4
    with open(filename, "w") as f:
        for s in symbols:
            f.write(f"{s:0{hex_width}x}\n")


def verify_symbol_packer():
    NUM_SYMS = 64
    SEED     = 0x5A3C7E
    cases = [(0, 2, "qpsk"), (1, 4, "16qam")]
    for mode_idx, bps, name in cases:
        bits, _ = prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        symbols = pack_symbols(bits, bps)
        outfile = f"syms_{name}_ref.txt"
        write_symbols_file(outfile, symbols, bps)
        print(f"[symbol_packer] Mode {name:6s}: wrote {NUM_SYMS} -> {outfile}")


# =============================================================================
# Block 3: QAM mapper
# =============================================================================

def gray_to_binary(g, n):
    b = (g >> (n - 1)) & 1
    out = b << (n - 1)
    for i in range(n - 2, -1, -1):
        b = b ^ ((g >> i) & 1)
        out |= b << i
    return out


def map_symbol(sym_bits, qam_mode, frac_w=10):
    bps_axis_table = [1, 2, 3, 4]
    bps_axis = bps_axis_table[qam_mode]
    i_gray = (sym_bits >> bps_axis) & ((1 << bps_axis) - 1)
    q_gray = sym_bits & ((1 << bps_axis) - 1)
    i_bin = gray_to_binary(i_gray, bps_axis)
    q_bin = gray_to_binary(q_gray, bps_axis)
    offset  = (1 << bps_axis) - 1
    i_level = 2 * i_bin - offset
    q_level = 2 * q_bin - offset
    return i_level << frac_w, q_level << frac_w


def twos16(v):
    return int(v) & 0xFFFF


def write_iq_file(filename, iq_pairs):
    with open(filename, "w") as f:
        for I, Q in iq_pairs:
            f.write(f"{twos16(I):04x} {twos16(Q):04x}\n")


def verify_qam_mapper():
    NUM_SYMS = 64
    SEED     = 0x5A3C7E
    cases = [(0, 2, "qpsk"), (1, 4, "16qam")]
    for mode_idx, bps, name in cases:
        bits, _  = prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        symbols  = pack_symbols(bits, bps)
        iq_pairs = [map_symbol(s, mode_idx, frac_w=10) for s in symbols]
        outfile = f"iq_{name}_ref.txt"
        write_iq_file(outfile, iq_pairs)
        print(f"[qam_mapper] Mode {name:6s}: wrote {NUM_SYMS} -> {outfile}")


# =============================================================================
# Block 4: QAM slicer
# =============================================================================

def binary_to_gray(b, n):
    g = (b >> (n - 1)) & 1
    out = g << (n - 1)
    for i in range(n - 2, -1, -1):
        g = ((b >> (i + 1)) & 1) ^ ((b >> i) & 1)
        out |= g << i
    return out


def slice_iq(i_sample, q_sample, qam_mode, frac_w=10):
    bps_axis_table = [1, 2, 3, 4]
    bps_axis  = bps_axis_table[qam_mode]
    max_level = (1 << bps_axis) - 1

    def slice_axis(sample):
        biased = sample + (1 << frac_w)
        k = biased >> (frac_w + 1)
        level = 2 * k - 1
        if level >  max_level: level =  max_level
        if level < -max_level: level = -max_level
        binary = (level + max_level) >> 1
        return binary_to_gray(binary, bps_axis)

    i_gray = slice_axis(i_sample)
    q_gray = slice_axis(q_sample)
    return (i_gray << bps_axis) | q_gray


def verify_qam_slicer():
    NUM_SYMS = 64
    SEED     = 0x5A3C7E
    cases = [(0, 2, "qpsk"), (1, 4, "16qam")]
    for mode_idx, bps, name in cases:
        bits, _  = prbs_stream(NUM_SYMS * bps, lfsr_w=23, seed=SEED)
        symbols  = pack_symbols(bits, bps)
        iq_pairs = [map_symbol(s, mode_idx, frac_w=10) for s in symbols]
        sliced   = [slice_iq(I, Q, mode_idx, frac_w=10) for (I, Q) in iq_pairs]
        outfile = f"sliced_{name}_ref.txt"
        write_symbols_file(outfile, sliced, bps)
        mismatches = sum(1 for a, b in zip(symbols, sliced) if a != b)
        print(f"[qam_slicer] Mode {name:6s}: wrote {NUM_SYMS} -> {outfile}  "
              f"(self-check: {mismatches} mismatches)")


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


def verify_ber_counter():
    """
    Full Python loopback: bits -> pack -> map -> slice -> ber count.
    Expected: zero errors for both modes.
    Writes ber_report_ref.txt.
    """
    SEED          = 0x5A3C7E
    NUM_BITS_TGT  = 4096
    cases = [(0, 2, "QPSK   "), (1, 4, "16-QAM ")]

    with open("ber_report_ref.txt", "w") as f:
        for mode_idx, bps, name in cases:
            n_syms      = NUM_BITS_TGT // bps
            bits, _     = prbs_stream(n_syms * bps, lfsr_w=23, seed=SEED)
            symbols     = pack_symbols(bits, bps)
            iq_pairs    = [map_symbol(s, mode_idx) for s in symbols]
            sliced      = [slice_iq(I, Q, mode_idx) for (I, Q) in iq_pairs]
            errs, cnt   = ber_count(sliced, mode_idx, seed=SEED)
            line = f"{name.strip():8s}: errors={errs} bits={cnt}"
            print(f"[ber_counter] {line}")
            f.write(line + "\n")


# =============================================================================
if __name__ == "__main__":
    verify_bit_source()
    print()
    verify_symbol_packer()
    print()
    verify_qam_mapper()
    print()
    verify_qam_slicer()
    print()
    verify_ber_counter()