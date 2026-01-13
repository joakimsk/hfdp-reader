import serial
import time

PORT = "COM5"          # e.g. "COM6" on Windows or "/dev/ttyUSB0" on Linux/macOS
BAUD = 460800
OUTFILE = "capture.bin"

SYNC = 0xA5
PKT_LEN = 10  # 1+1+2+2+4

def read_exact(ser: serial.Serial, n: int) -> bytes:
    """Read exactly n bytes or raise EOFError on timeout/close."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise EOFError("Serial timeout/no data")
        buf.extend(chunk)
    return bytes(buf)

def next_packet_bytes(ser: serial.Serial) -> bytes:
    """Resync on SYNC and return one full packet's raw bytes (10 bytes)."""
    while True:
        b = ser.read(1)
        if not b:
            raise EOFError("Serial timeout/no data while seeking sync")
        if b[0] == SYNC:
            rest = read_exact(ser, PKT_LEN - 1)
            return bytes([SYNC]) + rest

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser, open(OUTFILE, "wb") as f:
        # Optional: let the board reboot / settle
        time.sleep(0.2)
        ser.reset_input_buffer()

        count = 0
        try:
            while True:
                pkt = next_packet_bytes(ser)
                f.write(pkt)          # write raw bytes
                count += 1
                if count % 1000 == 0:
                    f.flush()
                    print(f"Wrote {count} packets ({count * PKT_LEN} bytes)")
        except KeyboardInterrupt:
            print("\nStopped.")
        except EOFError as e:
            print(f"\nStopped: {e}")
        finally:
            f.flush()
            print(f"Final: {count} packets, {count * PKT_LEN} bytes -> {OUTFILE}")

if __name__ == "__main__":
    main()
