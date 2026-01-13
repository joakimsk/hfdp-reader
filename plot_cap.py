import argparse
import struct
import numpy as np
import matplotlib.pyplot as plt

SYNC = 0xA5
PKT_LEN = 10
FMT = "<BBHHI"  # sync, status, seq, raw14, t_us

def parse_capture(path: str):
    data = open(path, "rb").read()
    n = len(data)

    sync = SYNC
    packets = []

    i = 0
    while i + PKT_LEN <= n:
        if data[i] != sync:
            # resync: find next 0xA5
            j = data.find(bytes([sync]), i + 1)
            if j == -1:
                break
            i = j
            continue

        pkt = data[i:i + PKT_LEN]
        s, status, seq, raw14, t_us = struct.unpack(FMT, pkt)
        if s == sync:
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

    # unwrap micros() (uint32 wrap)
    t_unwrapped = t_us.astype(np.uint64)
    wraps = 0
    for k in range(1, len(t_unwrapped)):
        if t_us[k] < t_us[k - 1]:
            wraps += 1
        t_unwrapped[k] += wraps * (1 << 32)

    # convert to seconds from start
    t_s = (t_unwrapped - t_unwrapped[0]) / 1e6

    return status, seq, raw14, t_s

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="capture.bin")
    ap.add_argument("--max-points", type=int, default=0,
                    help="Optional decimation target (e.g. 200000). 0 = no decimation.")
    args = ap.parse_args()

    status, seq, raw14, t_s = parse_capture(args.file)

    # Optional simple decimation (keeps shape, faster plots for huge files)
    if args.max_points and len(raw14) > args.max_points:
        step = int(np.ceil(len(raw14) / args.max_points))
        t_s = t_s[::step]
        raw14 = raw14[::step]
        seq = seq[::step]
        status = status[::step]

    # Basic stats
    dt = np.diff(t_s)
    if len(dt) > 0:
        med_dt = float(np.median(dt))
        est_hz = 1.0 / med_dt if med_dt > 0 else float("nan")
    else:
        est_hz = float("nan")

    print(f"Packets: {len(raw14)}")
    print(f"Duration: {t_s[-1]:.6f} s")
    print(f"Estimated rate (median): {est_hz:.2f} Hz")
    unique_status = np.unique(status)
    print(f"Status values present: {unique_status}")

    plt.figure()
    plt.plot(t_s, raw14)
    plt.xlabel("Time (s)")
    plt.ylabel("raw14 (counts)")
    plt.title("capture.bin: raw14 vs time")
    plt.grid(True)

    # Optional: quick check for missing seqs (wrap-safe for uint16 is harder; this is a hint only)
    if len(seq) > 1:
        dseq = (seq.astype(np.int32)[1:] - seq.astype(np.int32)[:-1])
        misses = np.sum(dseq != 1)
        if misses:
            print(f"Seq discontinuities (not equal to +1): {misses}")

    plt.show()

if __name__ == "__main__":
    main()
