"""Record raw serial data with timestamps to a file for playback."""
import time
import serial

SERIAL_PORT = "COM4"
BAUD_RATE = 460800
DURATION = 20  # seconds
OUTPUT = "playback.txt"

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
print(f"Recording {DURATION}s from {SERIAL_PORT}...")

t0 = time.perf_counter()
lines = 0
buf = b""

with open(OUTPUT, "w") as f:
    while time.perf_counter() - t0 < DURATION:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf += chunk
            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode(errors="ignore").strip()
                parts = line.split(",")
                if len(parts) not in (9, 11):
                    continue
                try:
                    [int(p) for p in parts]
                except ValueError:
                    continue
                t = time.perf_counter() - t0
                f.write(f"{t:.6f},{line}\n")
                lines += 1
        else:
            time.sleep(0.001)

ser.close()
elapsed = time.perf_counter() - t0
print(f"Done: {lines} frames in {elapsed:.1f}s ({lines/elapsed:.1f} Hz)")
print(f"Saved to {OUTPUT}")
