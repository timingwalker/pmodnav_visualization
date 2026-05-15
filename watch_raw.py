"""Print raw ADC in real-time, marking duplicates with *DUP*."""
import serial, time

ser = serial.Serial("COM4", 460800, timeout=0)
buf = b""
last = None
cnt = 0

print(f"{'#':>6}  {'gx':>6} {'gy':>6} {'gz':>6}  {'ax':>6} {'ay':>6} {'az':>6}  {'mx':>6} {'my':>6} {'mz':>6}")
print("-" * 80)

try:
    while True:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="ignore").strip()
                parts = line.split(",")
                if len(parts) != 9:
                    continue
                try:
                    vals = tuple(int(p) for p in parts)
                except ValueError:
                    continue
                cnt += 1
                dup = " *DUP*" if vals == last else ""
                print(f"{cnt:>6}  {vals[0]:>6} {vals[1]:>6} {vals[2]:>6}  "
                      f"{vals[3]:>6} {vals[4]:>6} {vals[5]:>6}  "
                      f"{vals[6]:>6} {vals[7]:>6} {vals[8]:>6}{dup}")
                last = vals
        else:
            time.sleep(0.001)
except KeyboardInterrupt:
    print(f"\nDone. {cnt} frames.")
    ser.close()
