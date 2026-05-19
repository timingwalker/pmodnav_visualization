"""Generate synthetic IMU data: pure pitch, zero cross-axis, no noise."""
import math, time

DURATION = 10.0          # seconds
RATE = 450               # Hz
OUTPUT = "playback.txt"

n = int(DURATION * RATE)
with open(OUTPUT, "w") as f:
    for i in range(n):
        t = i / RATE

        # Pitch: sinusoidal oscillation, 0° -> +60° -> -60° -> 0°
        pitch_deg = 60.0 * math.sin(2.0 * math.pi * 0.25 * t)  # 0.25 Hz oscillation
        pitch_rad = math.radians(pitch_deg)

        # Pitch rate (gyro Y, deg/s) — derivative of pitch
        pitch_rate_dps = 60.0 * 2.0 * math.pi * 0.25 * math.cos(2.0 * math.pi * 0.25 * t)
        pitch_rate_rads = math.radians(pitch_rate_dps)

        # Gyro: only Y axis rotates (pure pitch). Convert rad/s back to ADC counts
        gy = pitch_rate_rads / (245.0 / 32768.0 * math.pi / 180.0)
        # Small noise so consecutive values aren't identical
        gx = 0
        gz = 0

        # Accelerometer: gravity rotates with pitch
        ay = 0
        ax_g = math.sin(pitch_rad)  # g
        az_g = math.cos(pitch_rad)  # g
        ax = ax_g / (2.0 / 32768.0)
        az = az_g / (2.0 / 32768.0)

        # Magnetometer: constant (pointing north, horizontal)
        mx = 2000
        my = 0
        mz = -2000

        f.write(f"{t:.6f},{int(gx)},{int(gy)},{int(gz)},"
                f"{int(ax)},{int(ay)},{int(az)},"
                f"{mx},{my},{mz}\n")

print(f"Generated {n} frames ({DURATION}s @ {RATE}Hz) -> {OUTPUT}")
print("Pure pitch 0°->+60°->-60°->0°, zero yaw drift, zero cross-axis")
