import argparse
import struct
import numpy as np
import matplotlib.pyplot as plt

SYNC = 0xA5
PKT_LEN = 10
FMT = "<BBHHI"  # sync, status, seq, raw14, t_us  (little-endian)

import numpy as np

def iir_lowpass(x, fc_hz, fs_hz):
    x = np.asarray(x, dtype=float)
    dt = 1.0 / fs_hz
    rc = 1.0 / (2.0 * np.pi * fc_hz)
    alpha = dt / (rc + dt)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i-1] + alpha * (x[i] - y[i-1])
    return y

def iir_highpass(x, fc_hz, fs_hz):
    x = np.asarray(x, dtype=float)
    dt = 1.0 / fs_hz
    rc = 1.0 / (2.0 * np.pi * fc_hz)
    alpha = rc / (rc + dt)
    y = np.empty_like(x)
    y[0] = 0.0
    for i in range(1, len(x)):
        y[i] = alpha * (y[i-1] + x[i] - x[i-1])
    return y


def median_filter_1d(x, k=5):
    """
    1D median filter with odd window size k (NumPy only).
    Edge handling: pads with edge values.
    """
    x = np.asarray(x)
    if k % 2 == 0 or k < 1:
        raise ValueError("k must be an odd positive integer")
    if k == 1 or x.size == 0:
        return x.copy()

    r = k // 2
    xpad = np.pad(x, (r, r), mode="edge")

    if hasattr(np.lib.stride_tricks, "sliding_window_view"):
        w = np.lib.stride_tricks.sliding_window_view(xpad, k)  # (len(x), k)
        y = np.median(w, axis=1)
    else:
        # Fallback (slower) for older NumPy
        y = np.empty(len(x), dtype=float)
        for i in range(len(x)):
            y[i] = np.median(xpad[i:i + k])

    # Keep integer type if input was integer
    if np.issubdtype(x.dtype, np.integer):
        y = np.rint(y).astype(x.dtype)
    return y

def parse_capture(path: str):
    data = open(path, "rb").read()
    n = len(data)

    packets = []
    i = 0
    while i + PKT_LEN <= n:
        if data[i] != SYNC:
            j = data.find(bytes([SYNC]), i + 1)
            if j == -1:
                break
            i = j
            continue

        pkt = data[i:i + PKT_LEN]
        s, status, seq, raw14, t_us = struct.unpack(FMT, pkt)
        if s == SYNC:
            packets.append((status, seq, raw14, t_us))
            i += PKT_LEN
        else:
            i += 1

    if not packets:
        raise RuntimeError("No valid packets found (check file/format).")

    arr = np.array(packets, dtype=np.uint32)
    status = arr[:, 0].astype(np.uint8)
    seq    = arr[:, 1].astype(np.uint16)
    raw14  = arr[:, 2].astype(np.uint16)
    t_us   = arr[:, 3].astype(np.uint32)

    # Unwrap micros() (uint32 wrap)
    t_unwrapped = t_us.astype(np.uint64)
    wraps = 0
    for k in range(1, len(t_unwrapped)):
        if t_us[k] < t_us[k - 1]:
            wraps += 1
        t_unwrapped[k] += wraps * (1 << 32)

    t_s = (t_unwrapped - t_unwrapped[0]) / 1e6
    return status, seq, raw14, t_s

def counts_to_pressure_mbar(raw_counts, pmin_mbar, pmax_mbar, cal_min_frac, cal_max_frac, full_counts=16384):
    """
    Honeywell transfer function (Eq. 2): Pressure = (Output-Outputmin)/(Outputmax-Outputmin)*(Pmax-Pmin)+Pmin
    raw_counts are your 14-bit values (0..16383).
    """
    out_min = cal_min_frac * full_counts
    out_max = cal_max_frac * full_counts
    return (raw_counts - out_min) * (pmax_mbar - pmin_mbar) / (out_max - out_min) + pmin_mbar

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="capture.bin")

    # Your sensor range:
    ap.add_argument("--pmin", type=float, default=-100.0, help="Pressure min (mbar)")
    ap.add_argument("--pmax", type=float, default=100.0,  help="Pressure max (mbar)")

    # Calibration span (common is 10%..90% for Honeywell digital output sensors)
    ap.add_argument("--cal-min", type=float, default=0.10, help="Outputmin as fraction of full scale (e.g. 0.10)")
    ap.add_argument("--cal-max", type=float, default=0.90, help="Outputmax as fraction of full scale (e.g. 0.90)")

    # Median filter
    ap.add_argument("--k", type=int, default=5, help="Median filter window (odd). Use 1 to disable.")

    # Plot speed for huge files
    ap.add_argument("--max-points", type=int, default=0,
                    help="Optional decimation target (e.g. 200000). 0 = no decimation.")

    args = ap.parse_args()

    status, seq, raw14, t_s = parse_capture(args.file)

    # (Optional) If you ever log nonzero status, keep only valid
    mask = (status == 0)
    if np.any(~mask):
        t_s = t_s[mask]
        raw14 = raw14[mask]
        seq = seq[mask]

    # Optional decimation (do before filtering for speed)
    if args.max_points and len(raw14) > args.max_points:
        step = int(np.ceil(len(raw14) / args.max_points))
        t_s = t_s[::step]
        raw14 = raw14[::step]
        seq = seq[::step]

    # Median filter counts (linear mapping => same idea as filtering pressure)
    raw14_f = median_filter_1d(raw14, k=args.k)

    # Convert to mbar using transfer function
    p_mbar = counts_to_pressure_mbar(
        raw14_f.astype(np.float64),
        pmin_mbar=args.pmin,
        pmax_mbar=args.pmax,
        cal_min_frac=args.cal_min,
        cal_max_frac=args.cal_max,
        full_counts=16384
    )


    # Estimate sample rate from timestamps
    fs = 1.0 / float(np.median(np.diff(t_s)))

    # 1) Kill single-sample spikes
    p1 = median_filter_1d(p_mbar, k=3)

    # 2) Band-limit for compressor cycles
    p_ac = iir_highpass(p1, fc_hz=5.0, fs_hz=fs)     # remove drift/DC
    p_f  = iir_lowpass(p_ac,  fc_hz=250.0, fs_hz=fs) # keep 60 Hz + harmonics

    plt.figure()
    plt.plot(t_s, p_f)
    plt.xlabel("Time (s)")
    plt.ylabel("Intake vacuum (mbar, filtered)")
    plt.title("Intake vacuum cycles (~60 Hz)")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
