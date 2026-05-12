"""Step-by-step diagnostic: raw ADC → SI conversion → coordinate mapping → AHRS attitude. Press Ctrl+C to advance."""
import time, json, os
import serial
import numpy as np
import imufusion

SERIAL_PORT = "COM4"
BAUD_RATE = 115200
GYRO_SCALE = 245.0 / 32768.0 * np.pi / 180.0
ACC_SCALE = 2.0 / 32768.0
MAG_SCALE = 4.0 / 32768.0

CALIB_FILE = "calib.json"
gyro_bias = np.zeros(3)
mag_offset = np.zeros(3)
if os.path.exists(CALIB_FILE):
    d = json.load(open(CALIB_FILE))
    gyro_bias = np.array(d["gyro_bias"])
    mag_offset = np.array(d["mag_offset"])
    print(f"Calibration loaded: gyro_bias(deg/s)={gyro_bias*180/np.pi}")
    print(f"mag_offset(counts)={mag_offset}")
else:
    print("Warning: no calibration file")

settings = imufusion.Settings(imufusion.CONVENTION_NWU, 0.005, 245.0, 1.0, 2.0, 3)
ahrs = imufusion.Ahrs(settings)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

# AHRS init
print("AHRS initializing (keep still)...")
n = 0
while ahrs.flags.initialising and n < 500:
    raw = ser.readline().decode(errors="ignore").strip()
    parts = raw.split(",")
    if len(parts) != 9: continue
    try: v = [int(x) for x in parts]
    except: continue
    gx=v[0]*GYRO_SCALE-gyro_bias[0]; gy=v[1]*GYRO_SCALE-gyro_bias[1]; gz=v[2]*GYRO_SCALE-gyro_bias[2]
    ax=v[3]*ACC_SCALE; ay=v[4]*ACC_SCALE; az=v[5]*ACC_SCALE
    mx=(v[6]-mag_offset[0])*MAG_SCALE; my=(v[7]-mag_offset[1])*MAG_SCALE; mz=(v[8]-mag_offset[2])*MAG_SCALE
    ahrs.update(np.array([gx,-gy,gz],dtype=np.float64), np.array([ax,-ay,az],dtype=np.float64), np.array([mx,my,mz],dtype=np.float64), 1/85)
    n += 1
print(f"Init done: {n} frames\n")

def run_step(title, header, fmt_func):
    """Print live data every 10 frames. Press Ctrl+C to advance."""
    print("=" * 80)
    print(title)
    print(f"  {header}")
    cnt = 0
    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            parts = raw.split(",")
            if len(parts) != 9: continue
            try: v = [int(x) for x in parts]
            except: continue
            if cnt % 10 == 0:
                print(f"  {fmt_func(v)}")
            cnt += 1
    except KeyboardInterrupt:
        print()

# Step 1: Raw ADC
print("""
[Instructions]
  (a) Keep sensor flat on desk, check az ~15700-16000, ax/ay ~0
  (b) Tilt forward (X up), check ax increases, az decreases
  (c) Tilt right (Y up), check ay increases, az decreases
  Press Ctrl+C when done""")
run_step(
    "Step 1: Raw ADC (print every 10 frames, Ctrl+C to advance)",
    f"{'gx':>7} {'gy':>7} {'gz':>7} {'ax':>7} {'ay':>7} {'az':>7} {'mx':>6} {'my':>6} {'mz':>7}",
    lambda v: f"{v[0]:7d} {v[1]:7d} {v[2]:7d} {v[3]:7d} {v[4]:7d} {v[5]:7d} {v[6]:6d} {v[7]:6d} {v[8]:7d}"
)

# Step 2: SI conversion
print("""
[Instructions]
  (a) Keep flat, check az ~+1.0g, ax/ay ~0g, gyro values all ~0 deg/s
  (b) Tilt forward, check ax and az direction change
  (c) Tilt right, check ay and az direction change
  Press Ctrl+C when done""")
run_step(
    "Step 2: SI conversion (bias removal + scaling, print every 10 frames, Ctrl+C to advance)",
    f"{'gx_dps':>8} {'gy_dps':>8} {'gz_dps':>8} {'ax_g':>8} {'ay_g':>8} {'az_g':>8} {'mx_G':>8} {'my_G':>8} {'mz_G':>8}",
    lambda v: f"{v[0]*GYRO_SCALE*180/np.pi-gyro_bias[0]*180/np.pi:8.2f} {v[1]*GYRO_SCALE*180/np.pi-gyro_bias[1]*180/np.pi:8.2f} {v[2]*GYRO_SCALE*180/np.pi-gyro_bias[2]*180/np.pi:8.2f} {v[3]*ACC_SCALE:8.3f} {v[4]*ACC_SCALE:8.3f} {v[5]*ACC_SCALE:8.3f} {(v[6]-mag_offset[0])*MAG_SCALE:8.3f} {(v[7]-mag_offset[1])*MAG_SCALE:8.3f} {(v[8]-mag_offset[2])*MAG_SCALE:8.3f}"
)

# Step 3: Coordinate mapping → NWU ([gx,-gy,gz], [ax,-ay,az], [mx,my,mz])
print("""
[Instructions]
  Mapping: gyro_m=[gx, -gy, gz], acc_m=[ax, -ay, az], mag_m=[mx, my, mz]
  (a) Keep flat, acc_m should be ~[0, 0, +1.0g]
  (b) Tilt forward (rotate around Y), check gyro_m[1] and acc_m[1] change
  (c) Tilt right (rotate around X), check gyro_m[0] and acc_m[0] change
  Press Ctrl+C when done""")
run_step(
    "Step 3: Coordinate mapping -> NWU (print every 10 frames, Ctrl+C to advance)",
    f"{'gx_m':>8} {'gy_m':>8} {'gz_m':>8} {'ax_m':>8} {'ay_m':>8} {'az_m':>8} {'mx_m':>8} {'my_m':>8} {'mz_m':>8}",
    lambda v: (
        lambda gx,gy,gz,ax,ay,az,mx,my,mz:
        f"{gx*180/np.pi:8.2f} {-gy*180/np.pi:8.2f} {gz*180/np.pi:8.2f} {ax:8.3f} {-ay:8.3f} {az:8.3f} {mx:8.3f} {my:8.3f} {mz:8.3f}"
    )(
        v[0]*GYRO_SCALE-gyro_bias[0], v[1]*GYRO_SCALE-gyro_bias[1], v[2]*GYRO_SCALE-gyro_bias[2],
        v[3]*ACC_SCALE, v[4]*ACC_SCALE, v[5]*ACC_SCALE,
        (v[6]-mag_offset[0])*MAG_SCALE, (v[7]-mag_offset[1])*MAG_SCALE, (v[8]-mag_offset[2])*MAG_SCALE
    )
)

# Step 4: AHRS attitude
print("=" * 80)
print("""
[Instructions]
  (a) Flat on desk -> Pitch and Roll should be ~0 deg
  (b) Tilt forward (around Y) -> Pitch should change continuously
  (c) Tilt right (around X) -> Roll should change continuously
  (d) Flat, rotate CW around Z -> Yaw should decrease monotonically through 360 deg
  (e) Flat, rotate CCW around Z -> Yaw should increase monotonically through 360 deg
  Ctrl+C to exit""")
print("Step 4: AHRS attitude angles (print every 10 frames, Ctrl+C to exit)")
print(f"  {'Roll':>8} {'Pitch':>8} {'Yaw':>8}")
cnt = 0
try:
    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        parts = raw.split(",")
        if len(parts) != 9: continue
        try: v = [int(x) for x in parts]
        except: continue
        gx=v[0]*GYRO_SCALE-gyro_bias[0]; gy=v[1]*GYRO_SCALE-gyro_bias[1]; gz=v[2]*GYRO_SCALE-gyro_bias[2]
        ax=v[3]*ACC_SCALE; ay=v[4]*ACC_SCALE; az=v[5]*ACC_SCALE
        mx=(v[6]-mag_offset[0])*MAG_SCALE; my=(v[7]-mag_offset[1])*MAG_SCALE; mz=(v[8]-mag_offset[2])*MAG_SCALE
        ahrs.update(np.array([gx,-gy,gz],dtype=np.float64), np.array([ax,-ay,az],dtype=np.float64), np.array([mx,my,mz],dtype=np.float64), 1/85)
        if cnt % 10 == 0:
            r, p, y = ahrs.quaternion.to_euler()
            print(f"  {r:8.1f} {p:8.1f} {y:8.1f}")
        cnt += 1
except KeyboardInterrupt:
    print()

ser.close()
print("\nDone.")
