import time
import threading
import struct
import numpy as np
import serial

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

# ===== Serial + packet format =====
PORT = "COM5"
BAUD = 460800

SYNC = 0xA5
FMT  = "<BBHHI"  # sync, status, seq, raw14, t_us
SIZE = struct.calcsize(FMT)

# ===== Convert raw14 -> mbar (same transfer function you used on MCU) =====
P_MIN = -100.0
P_MAX =  100.0
OUT_MIN = 1638.0
OUT_MAX = 14745.0

def raw_to_mbar(raw14: int) -> float:
    return ((raw14 - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN)) + P_MIN

# ===== Ring buffer settings =====
FS_EST = 2000                    # expected sample rate
WINDOW_SEC = 5                   # show last 5 seconds
BUF_LEN = FS_EST * WINDOW_SEC * 2  # buffer bigger than window

# Shared ring buffer
buf_t = np.zeros(BUF_LEN, dtype=np.float64)   # seconds
buf_p = np.zeros(BUF_LEN, dtype=np.float32)   # mbar
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

def append_sample(seq: int, raw14: int, tus: int):
    global write_idx, have, last_tus, t_base, rx_count, drop_count, last_seq

    # Drop detection (ignore first packet)
    if last_seq is not None:
        expected = (last_seq + 1) & 0xFFFF
        if seq != expected:
            diff = (seq - expected) & 0xFFFF
            # ignore insane jumps from rare desync
            if diff < 1000:
                drop_count += diff
    last_seq = seq

    # Unwrap micros() (uint32 wrap ~71 minutes)
    if last_tus is None:
        last_tus = tus
        t_base = 0.0
        t_sec = 0.0
    else:
        dt = (tus - last_tus) & 0xFFFFFFFF
        last_tus = tus
        t_base += dt * 1e-6
        t_sec = t_base

    p = raw_to_mbar(raw14)

    with lock:
        buf_t[write_idx] = t_sec
        buf_p[write_idx] = p
        buf_seq[write_idx] = seq
        write_idx = (write_idx + 1) % BUF_LEN
        have = min(have + 1, BUF_LEN)

    rx_count += 1

def serial_reader():
    global rx_count
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
    """Return arrays (t, p) for the latest window."""
    with lock:
        if have == 0:
            return None, None

        # Reconstruct chronological view from ring buffer
        n = have
        end = write_idx
        start = (end - n) % BUF_LEN

        if start < end:
            t_all = buf_t[start:end].copy()
            p_all = buf_p[start:end].copy()
        else:
            t_all = np.concatenate((buf_t[start:].copy(), buf_t[:end].copy()))
            p_all = np.concatenate((buf_p[start:].copy(), buf_p[:end].copy()))

    # Slice last window_sec
    t_max = t_all[-1]
    t_min = t_max - window_sec
    j = np.searchsorted(t_all, t_min, side="left")
    return t_all[j:] - t_all[j], p_all[j:]

class LivePlot(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Diff Pressure Live Plot (mbar)")

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("left", "Pressure", units="mbar")
        self.plot.setLabel("bottom", "Time", units="s")
        self.curve = self.plot.plot([], [])

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
        t, p = get_latest_window(WINDOW_SEC)
        if t is None:
            return

        # Plot the last window
        self.curve.setData(t, p)

        # Update stats ~2x/sec
        now = time.time()
        if now - self.last_stats_t >= 0.5:
            dt = now - self.last_stats_t
            rx = rx_count
            hz = (rx - self.last_stats_rx) / dt

            self.stats.setText(
                f"RX ~{hz:.1f} samples/s   total={rx_count}   drops={drop_count}"
            )
            self.last_stats_t = now
            self.last_stats_rx = rx

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
