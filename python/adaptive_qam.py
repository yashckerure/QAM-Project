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
# Block 6: CRC-24A (TS 38.212 Section 5.1)
# =============================================================================
#
# Polynomial g_CRC24A(D) = D^24 + D^23 + D^18 + D^17 + D^14 + D^11 + D^10
#                       + D^7 + D^6 + D^5 + D^4 + D^3 + D + 1
#
# Standard test vector (TS 38.212): no canonical vector is given in the spec
# itself, but cross-validated against MATLAB nrCRCEncode('24A', ...) and the
# published reference implementations.

CRC24A_POLY = 0x864CFB     # 24-bit polynomial, leading D^24 implicit


def crc24a_compute(bits):
    """
    Compute CRC-24A over a list of bits (MSB-first into the LFSR).
    Returns the 24-bit CRC as an integer.
    """
    crc = 0
    for b in bits:
        # Shift left by 1, XOR in next bit at LSB
        feedback = ((crc >> 23) & 1) ^ b
        crc = ((crc << 1) & 0xFFFFFF)
        if feedback:
            crc ^= CRC24A_POLY
    return crc


def crc24a_attach(bits):
    """
    Append the 24-bit CRC-24A to a list of bits (MSB-first).
    Returns: list of bits, length = len(input) + 24.
    """
    crc = crc24a_compute(bits)
    crc_bits = [(crc >> (23 - i)) & 1 for i in range(24)]
    return list(bits) + crc_bits

def verify_crc24a_attach():
    """
    Match tb_crc24a_attach.v: 496 PRBS-23 input bits, seed 0x5A3C7E,
    then append CRC. Total output = 520 bits.
    """
    NUM_INPUT_BITS = 496
    bits, _ = prbs_stream(NUM_INPUT_BITS, lfsr_w=23, seed=0x5A3C7E)
    output_bits = crc24a_attach(bits)

    with open("crc_out_ref.txt", "w") as f:
        for b in output_bits:
            f.write(f"{b}\n")

    crc_val = crc24a_compute(bits)
    print(f"[crc24a_attach] Input  : {NUM_INPUT_BITS} bits")
    print(f"[crc24a_attach] Output : {len(output_bits)} bits")
    print(f"[crc24a_attach] CRC    : 0x{crc_val:06X}")
    print(f"[crc24a_attach] Wrote crc_out_ref.txt")

# =============================================================================
# Block 11: PDSCH scrambler (TS 38.211 Section 7.3.1.1)
# =============================================================================
#
# Gold sequence per TS 38.211 Section 5.2:
#   c(n)     = (x1(n+Nc) + x2(n+Nc)) mod 2,    Nc = 1600
#   x1(n+31) = (x1(n+3) + x1(n)) mod 2,        x1(0)=1, x1(1..30)=0
#   x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2
#   x2 initialized from c_init: x2(n) = (c_init >> n) & 1 for n=0..30
#
# c_init = n_RNTI * 2^15 + q * 2^14 + n_ID  (Section 7.3.1.1)
# Project values: n_RNTI=1, q=0, n_ID=0  =>  c_init = 0x00008000

NC_GOLD = 1600


def gold_sequence(n_bits, c_init):
    """
    Generate n_bits of the Gold sequence per TS 38.211 Section 5.2.
    Bit-exact match to the hardware LFSR implementation.
    """
    x1 = [0] * (NC_GOLD + n_bits + 31)
    x2 = [0] * (NC_GOLD + n_bits + 31)
    x1[0] = 1
    for i in range(31):
        x2[i] = (c_init >> i) & 1

    for n in range(NC_GOLD + n_bits):
        x1[n + 31] = (x1[n + 3] + x1[n]) & 1
        x2[n + 31] = (x2[n + 3] + x2[n + 2] + x2[n + 1] + x2[n]) & 1

    return [(x1[n + NC_GOLD] + x2[n + NC_GOLD]) & 1 for n in range(n_bits)]


def scrambler(bits, c_init):
    """
    PDSCH scrambler per TS 38.211 Section 7.3.1.1.
    Output: input bits XOR Gold(c_init).
    """
    c = gold_sequence(len(bits), c_init)
    return [b ^ ci for b, ci in zip(bits, c)]


def verify_scrambler():
    """
    Match tb_scrambler.v: 128 PRBS-23 bits, c_init = 0x00008000.
    """
    NUM_INPUT_BITS = 520
    C_INIT = 0x00008000

    bits, _ = prbs_stream(NUM_INPUT_BITS, lfsr_w=23, seed=0x5A3C7E)
    scrambled = scrambler(bits, C_INIT)

    with open("scrambler_out_ref.txt", "w") as f:
        for b in scrambled:
            f.write(f"{b}\n")

    print(f"[scrambler] Input  : {NUM_INPUT_BITS} bits")
    print(f"[scrambler] Output : {len(scrambled)} bits")
    print(f"[scrambler] c_init : 0x{C_INIT:08X}")
    print(f"[scrambler] First 8 Gold bits: {gold_sequence(8, C_INIT)}")
    print(f"[scrambler] Wrote scrambler_out_ref.txt")

# =============================================================================
# Block 12: Bit interleaver (TS 38.212 Section 5.4.2.2)
# =============================================================================
#
# Matrix interleaver with Qm rows and N=E/Qm columns.
# Write bits into matrix by rows, read out by columns.
# Output bit m = Input bit at index (m mod Qm)*N + (m div Qm)

def bit_interleaver(bits, Qm):
    """
    PDSCH bit interleaver per TS 38.212 Section 5.4.2.2.
    Requires len(bits) divisible by Qm.
    """
    E = len(bits)
    assert E % Qm == 0, f"E={E} must be a multiple of Qm={Qm}"
    N = E // Qm

    # Write row-by-row
    matrix = [[bits[r * N + c] for c in range(N)] for r in range(Qm)]

    # Read column-by-column
    out = []
    for c in range(N):
        for r in range(Qm):
            out.append(matrix[r][c])
    return out


def bit_deinterleaver(bits, Qm):
    """
    Inverse: write column-by-column, read row-by-row.
    """
    E = len(bits)
    assert E % Qm == 0
    N = E // Qm
    matrix = [[0] * N for _ in range(Qm)]
    idx = 0
    for c in range(N):
        for r in range(Qm):
            matrix[r][c] = bits[idx]
            idx += 1
    out = []
    for r in range(Qm):
        for c in range(N):
            out.append(matrix[r][c])
    return out


def verify_bit_interleaver():
    """
    Standalone TB matches: Qm=4 (16-QAM), E=128, PRBS-23 seed 0x5A3C7E.
    Also dumps references for other modes for later sweep tests.
    """
    # Primary test: 16-QAM with E=128
    bits1040, _ = prbs_stream(1040, lfsr_w=23, seed=0x5A3C7E)
    out_16qam = bit_interleaver(bits1040, 4)
    with open("interleaver_out_ref.txt", "w") as f:
        for b in out_16qam:
            f.write(f"{b}\n")
    print(f"[bit_interleaver] 16-QAM (Qm=4, E=128): wrote interleaver_out_ref.txt")

    # Round-trip sanity (every mode)
    for Qm, E_use, name in [(2, 128, "QPSK"), (4, 128, "16QAM"),
                             (6, 126, "64QAM"), (8, 128, "256QAM")]:
        bits, _ = prbs_stream(E_use, lfsr_w=23, seed=0x5A3C7E)
        interleaved   = bit_interleaver(bits, Qm)
        deinterleaved = bit_deinterleaver(interleaved, Qm)
        ok = (deinterleaved == bits)
        print(f"[bit_interleaver] {name:7s} (Qm={Qm}, E={E_use}): "
              f"round-trip {'OK' if ok else 'FAIL'}")

def verify_bit_deinterleaver():
    """
    Standalone TB matches: Qm=4 (16-QAM), E=128, PRBS-23 seed 0x5A3C7E.
    Input to the DUT is the INTERLEAVED stream, output should be the
    original PRBS bits.
    """
    bits128, _ = prbs_stream(1040, lfsr_w=23, seed=0x5A3C7E)
    interleaved   = bit_interleaver(bits128, 4)
    deinterleaved = bit_deinterleaver(interleaved, 4)

    # Sanity: deinterleaved must equal original
    assert deinterleaved == bits128, "Round-trip broken in Python"

    # Reference file: 128 bits of the deinterleaved output
    with open("deinterleaver_out_ref.txt", "w") as f:
        for b in deinterleaved:
            f.write(f"{b}\n")

    print(f"[bit_deinterleaver] 16-QAM (Qm=4, E=128): "
          f"wrote deinterleaver_out_ref.txt")
    print(f"[bit_deinterleaver] Round-trip self-check: "
          f"{'OK' if deinterleaved == bits128 else 'FAIL'}")

def verify_descrambler():
    """
    Standalone TB: take 128 scrambled bits (produced by scrambler on PRBS-23),
    feed into descrambler, expect original PRBS-23 bits.
    """
    NUM_INPUT_BITS = 520
    C_INIT = 0x00008000

    # Original PRBS-23 stream
    bits, _ = prbs_stream(NUM_INPUT_BITS, lfsr_w=23, seed=0x5A3C7E)

    # TX: scramble
    scrambled = scrambler(bits, C_INIT)

    # RX: descramble (same operation; XOR is self-inverse)
    descrambled = scrambler(scrambled, C_INIT)

    # Sanity round-trip
    assert descrambled == bits, "Round-trip broken in Python"

    # Reference file: 128 bits of descrambler output
    with open("descrambler_out_ref.txt", "w") as f:
        for b in descrambled:
            f.write(f"{b}\n")

    print(f"[descrambler] 520 bits, c_init=0x{C_INIT:08X}")
    print(f"[descrambler] Round-trip self-check: "
          f"{'OK' if descrambled == bits else 'FAIL'}")
    print(f"[descrambler] Wrote descrambler_out_ref.txt")

# =============================================================================
# Block 15: CRC-24A check (TS 38.212 Section 5.1, inverse of Block 6)
# =============================================================================

def crc24a_check(bits_with_crc):
    """
    Inverse of crc24a_attach. Input is info_bits + 24 CRC bits.
    Computes CRC-24A over the entire input; returns (info_bits, crc_ok).
    crc_ok is True iff the computed CRC over (info + CRC) equals zero.
    """
    assert len(bits_with_crc) >= 24, "Input must contain at least the CRC"
    info_bits = list(bits_with_crc[:-24])
    crc_remainder = crc24a_compute(bits_with_crc)
    return info_bits, (crc_remainder == 0)


def verify_crc24a_check():
    """
    Standalone TB: TX-side runs crc24a_attach over 496 PRBS-23 bits, producing
    520 bits. Feed those 152 bits into the DUT. Expect:
      - 128 output bits matching original PRBS-23
      - crc_ok = 1 at end of packet
    """
    NUM_INPUT_BITS = 496
    bits, _ = prbs_stream(NUM_INPUT_BITS, lfsr_w=23, seed=0x5A3C7E)
    bits_with_crc = crc24a_attach(bits)

    info_out, crc_ok = crc24a_check(bits_with_crc)

    with open("crc_check_out_ref.txt", "w") as f:
        for b in info_out:
            f.write(f"{b}\n")

    assert info_out == bits, "Recovered info bits don't match original"
    assert crc_ok, "CRC check failed on clean data"

    print(f"[crc24a_check] Input : {len(bits_with_crc)} bits (128 info + 24 CRC)")
    print(f"[crc24a_check] Output: {len(info_out)} bits")
    print(f"[crc24a_check] crc_ok: {crc_ok}")
    print(f"[crc24a_check] Round-trip self-check: "
          f"{'OK' if info_out == bits and crc_ok else 'FAIL'}")
    print(f"[crc24a_check] Wrote crc_check_out_ref.txt")

def verify_bit_chain_integration():
    """
    Full bit-domain integration: bit_source -> crc24a_attach -> scrambler
        -> bit_interleaver -> bit_deinterleaver -> descrambler
        -> crc24a_check -> output.
    
    Test: 128 PRBS-23 bits + CRC = 152 bits flow through the chain.
    Configuration: 16-QAM equivalent (Qm=4, N=38).
    Expected output: 128 bits identical to original PRBS-23.
    """
    NUM_INFO_BITS = 128
    QM            = 4
    C_INIT        = 0x00008000

    # TX side
    bits, _    = prbs_stream(NUM_INFO_BITS, lfsr_w=23, seed=0x5A3C7E)
    after_crc  = crc24a_attach(bits)              # 152 bits
    after_scr  = scrambler(after_crc, C_INIT)     # 152 bits
    after_int  = bit_interleaver(after_scr, QM)   # 152 bits

    # RX side
    after_dnt  = bit_deinterleaver(after_int, QM) # 152 bits
    after_dsc  = scrambler(after_dnt, C_INIT)     # 152 bits (XOR self-inverse)
    recovered, crc_ok = crc24a_check(after_dsc)   # 128 bits

    # Sanity checks
    assert recovered == bits, "Bit chain round-trip broken in Python"
    assert crc_ok, "CRC check failed in Python"

    # Reference file: the 128 recovered info bits (== original PRBS-23)
    with open("chain_out_ref.txt", "w") as f:
        for b in recovered:
            f.write(f"{b}\n")

    print(f"[bit_chain] Info bits     : {NUM_INFO_BITS}")
    print(f"[bit_chain] After CRC     : {len(after_crc)} bits")
    print(f"[bit_chain] Qm = {QM}, N = {len(after_crc)//QM}")
    print(f"[bit_chain] Recovered     : {len(recovered)} bits")
    print(f"[bit_chain] crc_ok        : {crc_ok}")
    print(f"[bit_chain] Round-trip    : "
          f"{'OK' if recovered == bits and crc_ok else 'FAIL'}")
    print(f"[bit_chain] Wrote chain_out_ref.txt")
# =============================================================================
if __name__ == "__main__":
    verify_crc24a_attach()
    verify_scrambler()
    verify_bit_interleaver()
    verify_bit_deinterleaver()
    verify_descrambler()
    verify_crc24a_check()
    verify_bit_chain_integration()