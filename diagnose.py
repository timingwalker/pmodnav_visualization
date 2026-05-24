"""Step-by-step diagnostic: raw ADC -> SI conversion -> coordinate mapping -> AHRS attitude. Press Ctrl+C to advance."""
import sys, time, json, os
import serial
import numpy as np
import imufusion

SERIAL_PORT = "COM4"
BAUD_RATE = 460800
GYRO_SCALE = 245.0 / 32768.0 * np.pi / 180.0
ACC_SCALE = 4.0 / 32768.0
MAG_SCALE = 4.0 / 32768.0

CALIB_FILE = "calib.json"
gyro_bias = np.zeros(3)
mag_offset = np.zeros(3)

def _arg_str(flag, default):
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
    return default

def _arg_int(flag, default):
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return int(sys.argv[idx + 1])
    return default

def _arg_float(flag, default):
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return float(sys.argv[idx + 1])
    return default

SERIAL_PORT = _arg_str("--port", SERIAL_PORT)
BAUD_RATE = _arg_int("--baud", BAUD_RATE)

def _parse_parts(parts):
    """Return 9 sensor values from 9 or 11 field format. Returns None on error."""
    try:
        if len(parts) == 11:
            return [int(x) for x in parts[2:]]
        elif len(parts) == 9:
            return [int(x) for x in parts]
    except ValueError:
        pass
    return None
if os.path.exists(CALIB_FILE):
    d = json.load(open(CALIB_FILE))
    gyro_bias = np.array(d["gyro_bias"])
    mag_offset = np.array(d["mag_offset"])
    print(f"Calibration loaded: gyro_bias(deg/s)={gyro_bias*180/np.pi}")
    print(f"mag_offset(counts)={mag_offset}")
else:
    print("Warning: no calibration file")

def _wrap_360(deg):
    deg = deg % 360.0
    return deg + 360.0 if deg < 0 else deg

def run_mag_heading():
    """Check whether magnetometer heading should use +Y or -Y."""
    print(f"Opening serial {SERIAL_PORT} at {BAUD_RATE}...", flush=True)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print(f"Serial {SERIAL_PORT} opened at {BAUD_RATE}", flush=True)
    print("\n[Mag heading check]")
    print("  Keep the sensor flat, then slowly rotate it 360 deg around Z.")
    print("  Watch which heading column moves continuously through a full circle.")
    print("  h(+Y)=atan2(my_cal, mx_cal); h(-Y)=atan2(-my_cal, mx_cal)")
    print("  Press Ctrl+C to exit.\n")
    print(f"  {'mx_cal':>9} {'my_cal':>9} {'mz_cal':>9} {'h(+Y)':>9} {'h(-Y)':>9} {'raw mx':>8} {'raw my':>8}", flush=True)
    cnt = 0
    invalid = 0
    last_status = time.perf_counter()
    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            now = time.perf_counter()
            if not raw:
                if now - last_status >= 1.0:
                    print("  waiting for serial lines...", flush=True)
                    last_status = now
                continue
            parts = raw.split(",")
            v = _parse_parts(parts)
            if v is None:
                invalid += 1
                if invalid <= 5 or now - last_status >= 1.0:
                    print(f"  skipped non-sensor line: {raw}", flush=True)
                    last_status = now
                continue
            mx_raw, my_raw, mz_raw = v[6], v[7], v[8]
            mx = (mx_raw - mag_offset[0]) * MAG_SCALE
            my = (my_raw - mag_offset[1]) * MAG_SCALE
            mz = (mz_raw - mag_offset[2]) * MAG_SCALE
            heading_pos_y = _wrap_360(np.degrees(np.arctan2(my, mx)))
            heading_neg_y = _wrap_360(np.degrees(np.arctan2(-my, mx)))
            if cnt % 10 == 0:
                print(f"  {mx:9.4f} {my:9.4f} {mz:9.4f} {heading_pos_y:9.1f} {heading_neg_y:9.1f} {mx_raw:8d} {my_raw:8d}", flush=True)
            cnt += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nDone.")

if "--mag-heading" in sys.argv:
    run_mag_heading()
    raise SystemExit(0)

def run_raw_watch():
    """Print parsed raw sensor frames without AHRS or calibration."""
    print(f"Opening serial {SERIAL_PORT} at {BAUD_RATE}...", flush=True)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print(f"Serial {SERIAL_PORT} opened at {BAUD_RATE}", flush=True)
    print("\n[Raw 9-axis watch]")
    print("  Move/tilt/rotate the sensor and check which fields change.")
    print("  Press Ctrl+C to exit.\n")
    print(f"  {'gx':>7} {'gy':>7} {'gz':>7} {'ax':>7} {'ay':>7} {'az':>7} {'mx':>7} {'my':>7} {'mz':>7}", flush=True)
    cnt = 0
    invalid = 0
    last_status = time.perf_counter()
    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            now = time.perf_counter()
            if not raw:
                if now - last_status >= 1.0:
                    print("  waiting for serial lines...", flush=True)
                    last_status = now
                continue
            v = _parse_parts(raw.split(","))
            if v is None:
                invalid += 1
                if invalid <= 5 or now - last_status >= 1.0:
                    print(f"  skipped non-sensor line: {raw}", flush=True)
                    last_status = now
                continue
            if cnt % 10 == 0:
                print(
                    f"  {v[0]:7d} {v[1]:7d} {v[2]:7d} "
                    f"{v[3]:7d} {v[4]:7d} {v[5]:7d} "
                    f"{v[6]:7d} {v[7]:7d} {v[8]:7d}",
                    flush=True,
                )
            cnt += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nDone.")

if "--raw-watch" in sys.argv:
    run_raw_watch()
    raise SystemExit(0)

def run_gyro_integral():
    """Integrate one mapped gyro axis as a scalar angle for ODR/scale checks."""
    axis_name = _arg_str("--axis", "y").lower()
    axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis_name, 1)
    show_all = axis_name == "all"
    rate_seconds = _arg_float("--rate-seconds", 5.0)
    print_hz = _arg_float("--print-hz", 2.0)
    manual_hz = _arg_float("--sample-hz", 0.0)

    print(f"Opening serial {SERIAL_PORT} at {BAUD_RATE}...", flush=True)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print(f"Serial {SERIAL_PORT} opened at {BAUD_RATE}", flush=True)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    measured_hz = manual_hz
    if measured_hz <= 0.0:
        print(f"Measuring valid line rate for {rate_seconds:.1f}s...", flush=True)
        count = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < rate_seconds:
            raw = ser.readline().decode(errors="ignore").strip()
            if _parse_parts(raw.split(",")) is not None:
                count += 1
        elapsed = time.perf_counter() - t0
        measured_hz = count / elapsed if elapsed > 0 and count else 449.0

    dt = 1.0 / measured_hz
    print(f"Using {measured_hz:.2f} Hz, dt={dt*1000:.3f} ms", flush=True)
    print("Keep still until you are ready, then rotate. Press Ctrl+C to stop.", flush=True)
    if show_all:
        print(f"{'gx':>8} {'gy':>8} {'gz':>8} {'ang_x':>9} {'ang_y':>9} {'ang_z':>9}", flush=True)
    else:
        print(f"{'gx':>8} {'gy':>8} {'gz':>8} {'angle_deg':>10}", flush=True)

    angles = np.zeros(3)
    last_print_t = 0.0
    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            parsed = _parse_parts(raw.split(","))
            if parsed is None:
                continue
            gx = parsed[0] * GYRO_SCALE - gyro_bias[0]
            gy = parsed[1] * GYRO_SCALE - gyro_bias[1]
            gz = parsed[2] * GYRO_SCALE - gyro_bias[2]
            gyro = np.array([gx, -gy, gz], dtype=np.float64)
            angles += gyro * dt

            now = time.perf_counter()
            if now - last_print_t >= 1.0 / max(print_hz, 0.1):
                last_print_t = now
                if show_all:
                    print(
                        f"{gyro[0]*180/np.pi:8.2f} {gyro[1]*180/np.pi:8.2f} {gyro[2]*180/np.pi:8.2f} "
                        f"{angles[0]*180/np.pi:9.1f} {angles[1]*180/np.pi:9.1f} {angles[2]*180/np.pi:9.1f}",
                        flush=True,
                    )
                else:
                    print(
                        f"{gyro[0]*180/np.pi:8.2f} {gyro[1]*180/np.pi:8.2f} {gyro[2]*180/np.pi:8.2f} "
                        f"{angles[axis_idx]*180/np.pi:10.1f}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        if show_all:
            print(
                f"\nFinal integrated angles: "
                f"x={angles[0]*180/np.pi:.1f} deg "
                f"y={angles[1]*180/np.pi:.1f} deg "
                f"z={angles[2]*180/np.pi:.1f} deg"
            )
        else:
            print(f"\nFinal integrated {axis_name}-axis angle: {angles[axis_idx]*180/np.pi:.1f} deg")
    finally:
        ser.close()

if "--gyro-integral" in sys.argv:
    run_gyro_integral()
    raise SystemExit(0)

def _quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y],
    ])

def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def _gyro_integrate(q, gyro, dt):
    angle = np.linalg.norm(gyro) * dt
    if angle < 1e-12:
        return q
    axis = gyro / np.linalg.norm(gyro)
    half = angle / 2.0
    dq = np.array([np.cos(half), *(axis * np.sin(half))])
    q = _quat_mul(q, dq)
    return q / np.linalg.norm(q)

def _quat_to_euler(q):
    w, x, y, z = q
    sr_cp = 2*(w*x + y*z); cr_cp = 1 - 2*(x*x + y*y)
    roll = np.arctan2(sr_cp, cr_cp)
    sp = 2*(w*y - z*x)
    sp = max(-1.0, min(1.0, sp))
    pitch = np.arcsin(sp)
    sy_cp = 2*(w*z + x*y); cy_cp = 1 - 2*(y*y + z*z)
    yaw = np.arctan2(sy_cp, cy_cp)
    return np.degrees([roll, pitch, yaw])

def run_ahrs_watch():
    """Print mapped sensor data, AHRS attitude, and model nose vector."""
    sensor_mode = _arg_str("--sensors", "gam")
    gain = float(_arg_str("--gain", "0.8"))
    acc_rej = float(_arg_str("--acc-rej", "5.0"))
    mag_rej = float(_arg_str("--mag-rej", "10.0"))
    rate_seconds = _arg_float("--rate-seconds", 2.0)
    print_hz = _arg_float("--print-hz", 4.0)
    manual_hz = _arg_float("--sample-hz", 0.0)
    print(f"Opening serial {SERIAL_PORT} at {BAUD_RATE}...", flush=True)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    settings = imufusion.Settings(imufusion.CONVENTION_NWU, gain, 245.0, acc_rej, mag_rej, 3)
    ahrs = imufusion.Ahrs(settings)
    q_gyro = np.array([1.0, 0.0, 0.0, 0.0])
    print(f"Serial {SERIAL_PORT} opened at {BAUD_RATE}", flush=True)
    print(f"Mode={sensor_mode} gain={gain} acc_rej={acc_rej} mag_rej={mag_rej}", flush=True)

    measured_hz = manual_hz
    if measured_hz <= 0.0:
        print(f"Measuring valid frame rate for {rate_seconds:.1f}s...", flush=True)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        rate_count = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < rate_seconds:
            raw = ser.readline().decode(errors="ignore").strip()
            if _parse_parts(raw.split(",")) is not None:
                rate_count += 1
        elapsed = time.perf_counter() - t0
        measured_hz = rate_count / elapsed if elapsed > 0 and rate_count else 449.0
    sample_period = 1.0 / measured_hz
    print(f"Using {measured_hz:.2f} Hz, dt={sample_period*1000:.3f} ms", flush=True)

    def read_sample():
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            parsed = _parse_parts(raw.split(","))
            if parsed is None:
                continue
            gx, gy, gz, ax, ay, az, mx, my, mz = parsed
            gx = gx * GYRO_SCALE - gyro_bias[0]
            gy = gy * GYRO_SCALE - gyro_bias[1]
            gz = gz * GYRO_SCALE - gyro_bias[2]
            ax *= ACC_SCALE; ay *= ACC_SCALE; az *= ACC_SCALE
            mx = (mx - mag_offset[0]) * MAG_SCALE
            my = (my - mag_offset[1]) * MAG_SCALE
            mz = (mz - mag_offset[2]) * MAG_SCALE
            return (
                np.array([gx, -gy, gz], dtype=np.float64),
                np.array([ax, -ay, az], dtype=np.float64),
                np.array([mx, -my, mz], dtype=np.float64),
            )

    if sensor_mode != "g":
        print("Initialising AHRS. Keep the sensor still...", flush=True)
        init_frames = 0
        while ahrs.flags.initialising and init_frames < 500:
            gyro, acc, mag = read_sample()
            if sensor_mode == "ga":
                ahrs.update_no_magnetometer(gyro, acc, sample_period)
            else:
                ahrs.update(gyro, acc, mag, sample_period)
            init_frames += 1
        q_ref = np.array(ahrs.quaternion.wxyz)
        print(f"Reference captured after {init_frames} init frames.", flush=True)
    else:
        q_ref = q_gyro.copy()
        print("Reference captured for gyro-only mode.", flush=True)

    R_ref = _quat_to_matrix(q_ref)
    print("Rotate around sensor Y. Dominant mapped gyro should be gy_m.", flush=True)
    print("rel_nose=(R_current * R_ref.T * [1,0,0]); for pure pitch around Y, rel_nose_y should stay near 0.", flush=True)
    print("Press Ctrl+C to exit.\n", flush=True)
    print(
        f"  {'gx':>7} {'gy':>7} {'gz':>7} "
        f"{'ax':>6} {'ay':>6} {'az':>6} "
        f"{'hz':>6} "
        f"{'roll':>7} {'pitch':>7} {'yaw':>7} "
        f"{'rel_x':>7} {'rel_y':>7} {'rel_z':>7}",
        flush=True,
    )
    cnt = 0
    last_print_t = 0.0
    try:
        while True:
            gyro, acc, mag = read_sample()
            dt = sample_period

            if sensor_mode == "g":
                q_gyro = _gyro_integrate(q_gyro, gyro, dt)
                q = q_gyro
                e = _quat_to_euler(q)
            elif sensor_mode == "ga":
                ahrs.update_no_magnetometer(gyro, acc, dt)
                q = ahrs.quaternion.wxyz
                e = ahrs.quaternion.to_euler()
            else:
                ahrs.update(gyro, acc, mag, dt)
                q = ahrs.quaternion.wxyz
                e = ahrs.quaternion.to_euler()

            now = time.perf_counter()
            if now - last_print_t >= 1.0 / max(print_hz, 0.1):
                last_print_t = now
                R_rel = _quat_to_matrix(q) @ R_ref.T
                nose = R_rel @ np.array([1.0, 0.0, 0.0])
                print(
                    f"  {gyro[0]*180/np.pi:7.1f} {gyro[1]*180/np.pi:7.1f} {gyro[2]*180/np.pi:7.1f} "
                    f"{acc[0]:6.2f} {acc[1]:6.2f} {acc[2]:6.2f} "
                    f"{measured_hz:6.1f} "
                    f"{e[0]:7.1f} {e[1]:7.1f} {e[2]:7.1f} "
                    f"{nose[0]:7.3f} {nose[1]:7.3f} {nose[2]:7.3f}",
                    flush=True,
                )
            cnt += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nDone.")

if "--ahrs-watch" in sys.argv:
    run_ahrs_watch()
    raise SystemExit(0)

settings = imufusion.Settings(imufusion.CONVENTION_NWU, 0.5, 245.0, 1.0, 2.0, 3)
ahrs = imufusion.Ahrs(settings)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

# AHRS init
print("AHRS initializing (keep still)...")
n = 0
while ahrs.flags.initialising and n < 500:
    raw = ser.readline().decode(errors="ignore").strip()
    parts = raw.split(",")
    v = _parse_parts(parts)
    if v is None: continue
    gx=v[0]*GYRO_SCALE-gyro_bias[0]; gy=v[1]*GYRO_SCALE-gyro_bias[1]; gz=v[2]*GYRO_SCALE-gyro_bias[2]
    ax=v[3]*ACC_SCALE; ay=v[4]*ACC_SCALE; az=v[5]*ACC_SCALE
    mx=(v[6]-mag_offset[0])*MAG_SCALE; my=(v[7]-mag_offset[1])*MAG_SCALE; mz=(v[8]-mag_offset[2])*MAG_SCALE
    ahrs.update(np.array([gx,-gy,gz],dtype=np.float64), np.array([ax,-ay,az],dtype=np.float64), np.array([mx,-my,mz],dtype=np.float64), 1/449)
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
            v = _parse_parts(parts)
            if v is None: continue
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

# Step 3: Coordinate mapping -> NWU ([gx,-gy,gz], [ax,-ay,az], [mx,-my,mz])
print("""
[Instructions]
  Mapping: gyro_m=[gx, -gy, gz], acc_m=[ax, -ay, az], mag_m=[mx, -my, mz]
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
        v = _parse_parts(parts)
        if v is None: continue
        gx=v[0]*GYRO_SCALE-gyro_bias[0]; gy=v[1]*GYRO_SCALE-gyro_bias[1]; gz=v[2]*GYRO_SCALE-gyro_bias[2]
        ax=v[3]*ACC_SCALE; ay=v[4]*ACC_SCALE; az=v[5]*ACC_SCALE
        mx=(v[6]-mag_offset[0])*MAG_SCALE; my=(v[7]-mag_offset[1])*MAG_SCALE; mz=(v[8]-mag_offset[2])*MAG_SCALE
        ahrs.update(np.array([gx,-gy,gz],dtype=np.float64), np.array([ax,-ay,az],dtype=np.float64), np.array([mx,-my,mz],dtype=np.float64), 1/449)
        if cnt % 10 == 0:
            r, p, y = ahrs.quaternion.to_euler()
            print(f"  {r:8.1f} {p:8.1f} {y:8.1f}")
        cnt += 1
except KeyboardInterrupt:
    print()

ser.close()
print("\nDone.")
