"""Print every gyro frame to catch cross-axis spikes during pitch tilt."""
import serial

ser = serial.Serial("COM4", 460800, timeout=0.01)
buf = b""
print("Keep sensor FLAT for 3s, then do a PURE PITCH tilt (no yaw rotation)")
print(f"{'gx':>6} {'gy':>6} {'gz':>6}")
print("-" * 40)

while True:
    raw = ser.readline().decode(errors="ignore").strip()
    parts = raw.split(",")
    if len(parts) != 9:
        continue
    try:
        gx, gy, gz = int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError:
        continue
    flag = " *** GZ SPIKE ***" if abs(gz) > 800 else ""
    print(f"{gx:6d} {gy:6d} {gz:6d}{flag}")
