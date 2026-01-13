import time
import threading
import struct
import numpy as np
import serial
from collections import deque

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

# ===== Serial + packet format =====
PORT = "COM5"
BAUD = 460800

SYNC = 0xA5
FMT  = "<BBHHI"  # sync, status, seq, raw14, t_us
SIZE = struct.calcsize(FMT)

# ===== Convert raw14 -> mbar (transfer function) =====
P_MIN = -100.0
P_MAX =  100.0
OUT_MIN = 1638.0
OUT_MAX = 14745.0

def raw_to_mbar(raw14: int) -> float:
    return ((raw14 - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN)) + P_MIN

# ===== Filtering (generic) =====
ENABLE_FILTERS = True

MED_K = 3          # short median for spike removal (odd: 1 disables)
HP_HZ = 5.0        # high-pass cutoff (Hz) for drift/DC removal (0 disables)
LP_HZ = 250.0      # low-pass cutoff (Hz) for smoothing / band-limit (0 disables)

SHOW_RAW = False   # set True to overlay raw mbar
SHOW_FILTERED = True

# ===== Ring buffer settings =====
FS_EST = 2000                    # expected sample rate
WINDOW_SEC = 5                   # show last 5 seconds
BUF_LEN = FS_EST * WINDOW_SEC * 2  # buffer bigger than window

# Shared ring buffer
buf_t = np.zeros(BUF_LEN, dtype=np.float64)    # seconds
buf_p_raw = np.zeros(BUF_LEN, dtype=np.float32)  # mbar
buf_p_filt = np.zeros(BUF_LEN, dtype=np.float32) # mbar (filtered)
buf_seq = np.zeros(BUF_LEN, dtype=np.uint16)

lock = threading.Lock()
write_idx = 0
have = 0

# Stats
rx_count = 0
drop_count = 0
last_seq = None

# Timestamp unwrapping for uint32 micros()
last_tus = None
t_base = 0.0

# ===== Filter state (streaming) =====
med_buf = deque(maxlen=max(1, MED_K))

hp_y = 0.0
hp_x_prev = 0.0

lp_y = 0.0
lp_initialized = False

def _rc(fc_hz: float) -> float:
    return 1.0 / (2.0 * np.pi * fc_hz)

def apply_filters_streaming(x: float, dt: float) -> float:
    """
    Streaming: trailing median (MED_K), then HP (HP_HZ), then LP (LP_HZ).
    dt is seconds between samples (from tus).
    """
    global hp_y, hp_x_prev, lp_y, lp_initialized

    # Median (trailing)
    if MED_K <= 1:
        x_med = x
    else:
        med_buf.append(x)
        x_med = float(np.median(np.fromiter(med_buf, dtype=np.float64)))

    if not ENABLE_FILTERS:
        return x_med

    # High-pass (1st order)
    x_hp = x_med
    if HP_HZ and HP_HZ > 0.0:
        rc = _rc(HP_HZ)
        a = rc / (rc + dt) if dt > 0 else 0.0
        # y[n] = a*(y[n-1] + x[n] - x[n-1])
        hp_y = a * (hp_y + x_med - hp_x_prev)
        hp_x_prev = x_med
        x_hp = hp_y

    # Low-pass (1st order)
    x_lp = x_hp
    if LP_HZ and LP_HZ > 0.0:
        rc = _rc(LP_HZ)
        alpha = dt / (rc + dt) if dt > 0 else 1.0
        if not lp_initialized:
            lp_y = x_hp
            lp_initialized = True
        else:
            lp_y = lp_y + alpha * (x_hp - lp_y)
        x_lp = lp_y

    return x_lp

def append_sample(seq: int, raw14: int, tus: int):
    global write_idx, have, last_tus, t_base, rx_count, drop_count, last_seq

    # Drop detection (ignore first packet)
    if last_seq is not None:
        expected = (last_seq + 1) & 0xFFFF
        if seq != expected:
            diff = (seq - expected) & 0xFFFF
            if diff < 1000:  # ignore insane jumps from rare desync
                drop_count += diff
    last_seq = seq

    # Unwrap micros() (uint32 wrap ~71 minutes)
    if last_tus is None:
        last_tus = tus
        t_base = 0.0
        dt = 1.0 / FS_EST
        t_sec = 0.0
    else:
        dt_us = (tus - last_tus) & 0xFFFFFFFF
        last_tus = tus
        dt = dt_us * 1e-6
        t_base += dt
        t_sec = t_base

    p_raw = raw_to_mbar(raw14)
    p_filt = apply_filters_streaming(p_raw, dt)

    with lock:
        buf_t[write_idx] = t_sec
        buf_p_raw[write_idx] = p_raw
        buf_p_filt[write_idx] = p_filt
        buf_seq[write_idx] = seq
        write_idx = (write_idx + 1) % BUF_LEN
        have = min(have + 1, BUF_LEN)

    rx_count += 1

def serial_reader():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()

    buf = bytearray()

    while True:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk

        # Parse as many packets as possible
        while True:
            i = buf.find(bytes([SYNC]))
            if i < 0:
                # keep a little tail in case sync spans reads
                if len(buf) > SIZE:
                    buf = buf[-(SIZE - 1):]
                break

            if len(buf) - i < SIZE:
                buf = buf[i:]
                break

            pkt = buf[i:i+SIZE]
            buf = buf[i+SIZE:]

            sync, status, seq, raw14, tus = struct.unpack(FMT, pkt)
            if sync != SYNC:
                continue

            # Only plot valid samples (status==0)
            if status == 0:
                append_sample(seq, raw14, tus)

def get_latest_window(window_sec: float):
    """Return arrays (t, p_raw, p_filt) for the latest window."""
    with lock:
        if have == 0:
            return None, None, None

        n = have
        end = write_idx
        start = (end - n) % BUF_LEN

        if start < end:
            t_all = buf_t[start:end].copy()
            p_raw_all = buf_p_raw[start:end].copy()
            p_filt_all = buf_p_filt[start:end].copy()
        else:
            t_all = np.concatenate((buf_t[start:].copy(), buf_t[:end].copy()))
            p_raw_all = np.concatenate((buf_p_raw[start:].copy(), buf_p_raw[:end].copy()))
            p_filt_all = np.concatenate((buf_p_filt[start:].copy(), buf_p_filt[:end].copy()))

    # Slice last window_sec
    t_max = t_all[-1]
    t_min = t_max - window_sec
    j = np.searchsorted(t_all, t_min, side="left")

    t = t_all[j:] - t_all[j]
    return t, p_raw_all[j:], p_filt_all[j:]

class LivePlot(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Intake Vacuum Live Plot (mbar)")

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("left", "Pressure", units="mbar")
        self.plot.setLabel("bottom", "Time", units="s")

        self.curve_raw = self.plot.plot([], [])       # optional raw
        self.curve_filt = self.plot.plot([], [])      # filtered

        self.stats = QtWidgets.QLabel("Starting…")
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.addWidget(self.plot)
        layout.addWidget(self.stats)
        self.setCentralWidget(w)

        self.last_stats_t = time.time()
        self.last_stats_rx = 0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)  # ~30 FPS

    def update_plot(self):
        t, p_raw, p_filt = get_latest_window(WINDOW_SEC)
        if t is None:
            return

        if SHOW_RAW:
            self.curve_raw.setData(t, p_raw)
        else:
            self.curve_raw.setData([], [])

        if SHOW_FILTERED:
            self.curve_filt.setData(t, p_filt)
        else:
            self.curve_filt.setData([], [])

        # Update stats ~2x/sec
        now = time.time()
        if now - self.last_stats_t >= 0.5:
            dt = now - self.last_stats_t
            rx = rx_count
            hz = (rx - self.last_stats_rx) / dt

            filt_txt = "OFF"
            if ENABLE_FILTERS:
                filt_txt = f"MED{k_to_str(MED_K)}  HP={HP_HZ:g}Hz  LP={LP_HZ:g}Hz"

            self.stats.setText(
                f"RX ~{hz:.1f} samples/s   total={rx_count}   drops={drop_count}   filters: {filt_txt}"
            )
            self.last_stats_t = now
            self.last_stats_rx = rx

def k_to_str(k):
    return "" if k <= 1 else f" k={k}"

def main():
    # start reader thread
    th = threading.Thread(target=serial_reader, daemon=True)
    th.start()

    app = QtWidgets.QApplication([])
    win = LivePlot()
    win.resize(1000, 600)
    win.show()
    app.exec_()

if __name__ == "__main__":
    main()
