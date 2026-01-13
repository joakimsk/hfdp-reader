import argparse
import struct
import numpy as np
import matplotlib.pyplot as plt

SYNC = 0xA5
PKT_LEN = 10
FMT = "<BBHHI"  # sync, status, seq, raw14, t_us  (little-endian)

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

    # sliding_window_view is available in NumPy >= 1.20
    if hasattr(np.lib.stride_tricks, "sliding_window_view"):
        w = np.lib.stride_tricks.sliding_window_view(xpad, k)  # (len(x), k)
        y = np.median(w, axis=1)
    else:
        # Fallback (slower) if sliding_window_view isn't available
        y = np.empty_like(xpad[r:-r], dtype=float)
        for i in range(len(y)):
            y[i] = np.median(xpad[i:i+k])

    # Keep type similar to input
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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="capture.bin")
    ap.add_argument("--k", type=int, default=5, help="Median filter window (odd), e.g. 3 or 5")
    ap.add_argument("--max-points", type=int, default=0,
                    help="Optional decimation target (e.g. 200000). 0 = no decimation.")
    args = ap.parse_args()

    status, seq, raw14, t_s = parse_capture(args.file)

    # Optional decimation (helps plotting huge files). Decimate before filtering for speed.
    if args.max_points and len(raw14) > args.max_points:
        step = int(np.ceil(len(raw14) / args.max_points))
        t_s = t_s[::step]
        raw14 = raw14[::step]
        seq = seq[::step]
        status = status[::step]

    raw14_med = median_filter_1d(raw14, k=args.k)

    # Rate estimate
    dt = np.diff(t_s)
    est_hz = (1.0 / float(np.median(dt))) if len(dt) else float("nan")
    print(f"Packets: {len(raw14)}")
    print(f"Duration: {t_s[-1]:.6f} s")
    print(f"Estimated rate (median): {est_hz:.2f} Hz")

    plt.figure()
    plt.plot(t_s, raw14, label="raw14")
    plt.plot(t_s, raw14_med, label=f"median k={args.k}")
    plt.xlabel("Time (s)")
    plt.ylabel("raw14 (counts)")
    plt.title("capture.bin: raw14 vs time (with median filter)")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
