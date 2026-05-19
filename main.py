import sys
import os
import json
import time
import ctypes
import gc
import threading
import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter
import imufusion

# Raise Windows timer resolution to 1ms
ctypes.windll.winmm.timeBeginPeriod(1)
# Disable automatic GC to prevent stop-the-world pauses
gc.disable()

# ============================================================
# Configuration
# ============================================================
SERIAL_PORT = "COM4"
BAUD_RATE = 460800
SAMPLE_PERIOD = 1 / 449

GYRO_SCALE = 245.0 / 32768.0 * np.pi / 180.0
ACC_SCALE = 4.0 / 32768.0
MAG_SCALE = 4.0 / 32768.0

GAIN = 0.8             # Lower = trust gyro more (responsive); higher = trust accel/mag more
GYRO_RANGE = 245.0
ACC_REJECTION = 5.0     # Higher = tolerate more non-gravity acceleration
MAG_REJECTION = 10.0     # Higher = trust magnetometer more (counter gyro cross-axis)
RECOVERY_PERIOD = 3
CALIBRATION_FRAMES = 100
USE_MAGNETOMETER = True

# ============================================================
# Arguments
# ============================================================
PLAYBACK_FILE = None
GAIN_OVERRIDE = None
ACC_REJ_OVERRIDE = None
MAG_REJ_OVERRIDE = None
SENSOR_MODE = "gam"  # g=gyro, a=accel, m=mag
SYNC_60HZ = False

def _arg_val(flag):
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return float(sys.argv[idx + 1])
    return None

def _arg_str(flag, default):
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
    return default

if "--playback" in sys.argv:
    idx = sys.argv.index("--playback")
    if idx + 1 < len(sys.argv):
        PLAYBACK_FILE = sys.argv[idx + 1]

GAIN_OVERRIDE = _arg_val("--gain")
ACC_REJ_OVERRIDE = _arg_val("--acc-rej")
MAG_REJ_OVERRIDE = _arg_val("--mag-rej")
SENSOR_MODE = _arg_str("--sensors", "gam")
if "--sync-60hz" in sys.argv:
    SYNC_60HZ = True

if GAIN_OVERRIDE is not None:
    GAIN = GAIN_OVERRIDE
if ACC_REJ_OVERRIDE is not None:
    ACC_REJECTION = ACC_REJ_OVERRIDE
if MAG_REJ_OVERRIDE is not None:
    MAG_REJECTION = MAG_REJ_OVERRIDE

print(f"Sensor mode: {SENSOR_MODE}  GAIN={GAIN}  ACC_REJ={ACC_REJECTION}  MAG_REJ={MAG_REJECTION}")

# ============================================================
# Serial port
# ============================================================
ser = None
if PLAYBACK_FILE is None:
    try:
        import serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Serial {SERIAL_PORT} opened")
    except Exception as e:
        print(f"Cannot open serial {SERIAL_PORT}: {e}")

# ============================================================
# Calibration (load existing or re-acquire)
# ============================================================
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib.json")
gyro_bias = np.zeros(3)
mag_offset = np.zeros(3)
gz_deadband = 0.0  # rad/s, below this threshold gz is forced to 0

def parse_line(line):
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) != 9:
        return None
    try:
        return [int(p) for p in parts]
    except ValueError:
        return None

if ser is not None:
    if os.path.exists(CALIB_FILE) and "--recal" not in sys.argv:
        d = json.load(open(CALIB_FILE))
        gyro_bias = np.array(d["gyro_bias"])
        mag_offset = np.array(d["mag_offset"])
        gz_deadband = d.get("gz_deadband", 0.0)
        print(f"Calibration loaded ({CALIB_FILE})")
        print(f"  Gyro bias: X={gyro_bias[0]*180/np.pi:.2f} Y={gyro_bias[1]*180/np.pi:.2f} Z={gyro_bias[2]*180/np.pi:.2f} deg/s")
        print(f"  GZ deadband: {gz_deadband*180/np.pi:.2f} deg/s")
        print(f"  Hard-iron: X={mag_offset[0]:.0f} Y={mag_offset[1]:.0f} Z={mag_offset[2]:.0f}")
    else:
        if "--recal" in sys.argv:
            print("Force recalibrating...")
        # Gyro bias + Z deadband
        print(f"Calibrating gyro bias (keep sensor still, {CALIBRATION_FRAMES} frames)...")
        samples = 0
        gz_samples = []
        while samples < CALIBRATION_FRAMES:
            raw = ser.readline().decode(errors="ignore")
            vals = parse_line(raw)
            if vals is None:
                continue
            gx, gy, gz = vals[0], vals[1], vals[2]
            gyro_bias[0] += gx * GYRO_SCALE
            gyro_bias[1] += gy * GYRO_SCALE
            gyro_bias[2] += gz * GYRO_SCALE
            gz_samples.append(gz * GYRO_SCALE)  # in rad/s
            samples += 1
        gyro_bias /= CALIBRATION_FRAMES
        gz_std = float(np.std(gz_samples))
        gz_deadband = 5.0 * gz_std  # 5σ threshold
        bias_dps = gyro_bias * 180 / np.pi
        print(f"  Gyro bias: X={bias_dps[0]:.2f} Y={bias_dps[1]:.2f} Z={bias_dps[2]:.2f} deg/s")
        print(f"  GZ noise std: {gz_std*180/np.pi:.3f} deg/s, deadband: {gz_deadband*180/np.pi:.2f} deg/s")

        # Magnetometer hard-iron calibration (guided multi-step)
        mag_min = np.full(3, 1e9)
        mag_max = np.full(3, -1e9)

        def mag_collect(duration, prompt):
            print(f"\n  {prompt}")
            print(f"    ({duration}s, press Ctrl+C to skip this step)")
            t0 = time.perf_counter()
            try:
                while time.perf_counter() - t0 < duration:
                    raw = ser.readline().decode(errors="ignore")
                    vals = parse_line(raw)
                    if vals is None:
                        continue
                    mx, my, mz = vals[6], vals[7], vals[8]
                    mag_min[0] = min(mag_min[0], mx)
                    mag_min[1] = min(mag_min[1], my)
                    mag_min[2] = min(mag_min[2], mz)
                    mag_max[0] = max(mag_max[0], mx)
                    mag_max[1] = max(mag_max[1], my)
                    mag_max[2] = max(mag_max[2], mz)
            except KeyboardInterrupt:
                pass
            print(f"    Range X=[{mag_min[0]}, {mag_max[0]}] Y=[{mag_min[1]}, {mag_max[1]}] Z=[{mag_min[2]}, {mag_max[2]}]")

        print("\n  --- Magnetometer Calibration ---")
        print("  Collect data in 4 poses to cover all axes.")
        input("  Ready? Press Enter to start...")

        mag_collect(15, "1/4: Sensor FLAT on desk, slowly spin 360° around Z")
        mag_collect(15, "2/4: Pitch sensor UP ~60°, slowly spin around Z")
        mag_collect(15, "3/4: Roll sensor LEFT ~60°, slowly spin around Z")
        mag_collect(15, "4/4: Random motion — tilt and rotate in all directions")

        mag_offset = (mag_max + mag_min) / 2.0
        print(f"\n  Hard-iron offset: X={mag_offset[0]:.0f} Y={mag_offset[1]:.0f} Z={mag_offset[2]:.0f}")

        # Save to file
        json.dump({
            "gyro_bias": gyro_bias.tolist(),
            "mag_offset": mag_offset.tolist(),
            "gz_deadband": gz_deadband,
        }, open(CALIB_FILE, "w"))
        print(f"  Calibration saved to {CALIB_FILE}")

# ============================================================
# AHRS
# ============================================================
settings = imufusion.Settings(
    imufusion.CONVENTION_NWU, GAIN, GYRO_RANGE,
    ACC_REJECTION, MAG_REJECTION, RECOVERY_PERIOD,
)
ahrs = imufusion.Ahrs(settings)

# ============================================================
# AHRS init (wait for initialising flag to clear before opening window)
# ============================================================
if ser is not None and SENSOR_MODE != "g":
    print("AHRS initializing (keep sensor still)...")
    init_frames = 0
    max_init = 500 if USE_MAGNETOMETER else 200
    while ahrs.flags.initialising and init_frames < max_init:
        raw = ser.readline().decode(errors="ignore")
        vals = parse_line(raw)
        if vals is None:
            continue
        gx, gy, gz = vals[0], vals[1], vals[2]
        ax, ay, az = vals[3], vals[4], vals[5]
        mx, my, mz = vals[6], vals[7], vals[8]
        gx = gx * GYRO_SCALE - gyro_bias[0]
        gy = gy * GYRO_SCALE - gyro_bias[1]
        gz = gz * GYRO_SCALE - gyro_bias[2]
        ax *= ACC_SCALE; ay *= ACC_SCALE; az *= ACC_SCALE
        mx = (mx - mag_offset[0]) * MAG_SCALE
        my = (my - mag_offset[1]) * MAG_SCALE
        mz = (mz - mag_offset[2]) * MAG_SCALE
        if USE_MAGNETOMETER:
            ahrs.update(
                np.array([gx, -gy, gz], dtype=np.float64),
                np.array([ax, -ay, az], dtype=np.float64),
                np.array([mx, my, mz], dtype=np.float64),
                SAMPLE_PERIOD,
            )
        else:
            ahrs.update_no_magnetometer(
                np.array([gx, -gy, gz], dtype=np.float64),
                np.array([ax, -ay, az], dtype=np.float64),
                SAMPLE_PERIOD,
            )
        init_frames += 1
        if init_frames % 100 == 0:
            print(f"  Waited {init_frames} frames...")
    print(f"  AHRS initialized ({init_frames} frames)")

# Thread-safe shared attitude data
orientation_lock = threading.Lock()
shared_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
shared_euler = np.zeros(3)
shared_frame_count = 0
shared_feed_interval = 0.0  # ms, average data feed interval

# Quaternion helpers for gyro-only mode
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
    axis = gyro * dt / angle
    half = angle / 2.0
    s = np.sin(half)
    dq = np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s])
    return _quat_mul(q, dq)

def _quat_to_euler(q):
    w, x, y, z = q
    sr_cp = 2*(w*x + y*z); cr_cp = 1 - 2*(x*x + y*y)
    roll = np.arctan2(sr_cp, cr_cp)
    sp = 2*(w*y - z*x)
    sp = max(-1.0, min(1.0, sp))
    pitch = np.arcsin(sp)
    sy_cp = 2*(w*z + x*y); cy_cp = 1 - 2*(y*y + z*z)
    yaw = np.arctan2(sy_cp, cy_cp)
    return np.array([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

# ============================================================
# Serial read thread
# ============================================================
def serial_thread():
    global shared_quaternion, shared_euler, shared_frame_count, shared_feed_interval
    buf = b""
    last_t = time.perf_counter()
    feed_times = []
    q_gyro = np.array([1.0, 0.0, 0.0, 0.0])  # for gyro-only mode
    # 60Hz sync accumulators
    _acc_gyro = [0.0, 0.0, 0.0]
    _acc_count = 0
    _latest_acc = np.zeros(3)
    _latest_mag = np.zeros(3)
    _last_sync_t = time.perf_counter()

    while True:
        try:
            # Non-blocking read: only read available bytes
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
            else:
                time.sleep(0.001)
                continue
            buf += chunk

            # Extract complete lines from buffer (delimited by \n)
            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode(errors="ignore")
                vals = parse_line(line)
                if vals is None:
                    continue

                gx, gy, gz, ax, ay, az, mx, my, mz = vals
                gx = gx * GYRO_SCALE - gyro_bias[0]
                gy = gy * GYRO_SCALE - gyro_bias[1]
                gz = gz * GYRO_SCALE - gyro_bias[2]
                # Apply Z-axis deadband to suppress cross-axis leakage
                if gz_deadband > 0 and abs(gz) < gz_deadband:
                    gz = 0.0
                ax *= ACC_SCALE
                ay *= ACC_SCALE
                az *= ACC_SCALE
                mx = (mx - mag_offset[0]) * MAG_SCALE
                my = (my - mag_offset[1]) * MAG_SCALE
                mz = (mz - mag_offset[2]) * MAG_SCALE

                gyro = np.array([gx, -gy, gz], dtype=np.float64)
                acc  = np.array([ax, -ay, az], dtype=np.float64)
                mag  = np.array([mx, my, mz], dtype=np.float64)

                now = time.perf_counter()
                dt = now - last_t
                last_t = now
                if dt > 0.1:
                    dt = SAMPLE_PERIOD

                # --- 60Hz sync: accumulate gyro, batch-update AHRS ---
                if SYNC_60HZ and SENSOR_MODE == "gam":
                    # Accumulate gyro rotation (radians) + track latest accel/mag
                    _acc_gyro[0] += gyro[0] * dt
                    _acc_gyro[1] += gyro[1] * dt
                    _acc_gyro[2] += gyro[2] * dt
                    _acc_count += 1
                    _latest_acc = acc
                    _latest_mag = mag

                    if now - _last_sync_t >= 1.0 / 60.0:
                        sync_dt = now - _last_sync_t
                        # Effective gyro (rad/s) over the sync period
                        eff_gyro = np.array([
                            _acc_gyro[0] / sync_dt,
                            _acc_gyro[1] / sync_dt,
                            _acc_gyro[2] / sync_dt,
                        ])
                        if USE_MAGNETOMETER:
                            ahrs.update(eff_gyro, _latest_acc, _latest_mag, sync_dt)
                        else:
                            ahrs.update_no_magnetometer(eff_gyro, _latest_acc, sync_dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                        _acc_gyro = [0.0, 0.0, 0.0]
                        _acc_count = 0
                        _last_sync_t = now
                    else:
                        continue  # skip shared update, wait for next frame
                else:
                    # --- Per-frame sensor mode dispatch ---
                    if SENSOR_MODE == "g":
                        q_gyro = _gyro_integrate(q_gyro, gyro, dt)
                        q_out = q_gyro.copy()
                        e_out = _quat_to_euler(q_out)
                    elif SENSOR_MODE == "ga":
                        ahrs.update_no_magnetometer(gyro, acc, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                    elif SENSOR_MODE == "gm":
                        ahrs.update(gyro, np.zeros(3), mag, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                    elif SENSOR_MODE == "am":
                        ahrs.update(np.zeros(3), acc, mag, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                    elif SENSOR_MODE == "a":
                        ahrs.update_no_magnetometer(np.zeros(3), acc, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                    elif SENSOR_MODE == "m":
                        ahrs.update(np.zeros(3), np.zeros(3), mag, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()
                    else:  # "gam" default
                        if USE_MAGNETOMETER:
                            ahrs.update(gyro, acc, mag, dt)
                        else:
                            ahrs.update_no_magnetometer(gyro, acc, dt)
                        q_out = ahrs.quaternion.wxyz
                        e_out = ahrs.quaternion.to_euler()

                feed_now = time.perf_counter()
                feed_times.append(feed_now)
                if len(feed_times) > 50:
                    feed_times.pop(0)
                if len(feed_times) >= 2:
                    feed_avg = (feed_now - feed_times[0]) / (len(feed_times) - 1) * 1000
                    shared_feed_interval = feed_avg

                with orientation_lock:
                    shared_quaternion = np.array(q_out)
                    shared_euler = np.array(e_out)
                    shared_frame_count += 1

        except Exception:
            time.sleep(0.001)


def playback_thread():
    """Read pre-recorded serial data from file and feed through AHRS."""
    global shared_quaternion, shared_euler, shared_frame_count, shared_feed_interval
    q_gyro = np.array([1.0, 0.0, 0.0, 0.0])
    lines_data = []
    with open(PLAYBACK_FILE) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",", 1)
            if len(parts) != 2:
                continue
            t_rel = float(parts[0])
            raw = parts[1]
            lines_data.append((t_rel, raw))
    if not lines_data:
        return
    total_duration = lines_data[-1][0]
    print(f"Playback: {len(lines_data)} frames over {total_duration:.1f}s "
          f"({len(lines_data)/total_duration:.1f} Hz)")

    feed_times = []  # sliding window for feed rate display
    last_t = time.perf_counter()
    _acc_gyro = [0.0, 0.0, 0.0]
    _acc_count = 0
    _latest_acc = np.zeros(3)
    _latest_mag = np.zeros(3)
    _last_sync_t = time.perf_counter()
    while True:
        t_start = time.perf_counter()
        idx = 0
        while idx < len(lines_data):
            t_target = t_start + lines_data[idx][0]
            wait = t_target - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            vals = parse_line(lines_data[idx][1])
            idx += 1
            if vals is None:
                continue

            gx, gy, gz, ax, ay, az, mx, my, mz = vals
            gx = gx * GYRO_SCALE - gyro_bias[0]
            gy = gy * GYRO_SCALE - gyro_bias[1]
            gz = gz * GYRO_SCALE - gyro_bias[2]
            if gz_deadband > 0 and abs(gz) < gz_deadband:
                gz = 0.0
            ax *= ACC_SCALE; ay *= ACC_SCALE; az *= ACC_SCALE
            mx = (mx - mag_offset[0]) * MAG_SCALE
            my = (my - mag_offset[1]) * MAG_SCALE
            mz = (mz - mag_offset[2]) * MAG_SCALE

            gyro = np.array([gx, -gy, gz], dtype=np.float64)
            acc = np.array([ax, -ay, az], dtype=np.float64)
            mag = np.array([mx, my, mz], dtype=np.float64)

            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            if dt > 0.1:
                dt = SAMPLE_PERIOD

            # --- 60Hz sync: accumulate gyro, batch-update AHRS ---
            if SYNC_60HZ and SENSOR_MODE == "gam":
                _acc_gyro[0] += gyro[0] * dt
                _acc_gyro[1] += gyro[1] * dt
                _acc_gyro[2] += gyro[2] * dt
                _acc_count += 1
                _latest_acc = acc
                _latest_mag = mag

                if now - _last_sync_t >= 1.0 / 60.0:
                    sync_dt = now - _last_sync_t
                    eff_gyro = np.array([
                        _acc_gyro[0] / sync_dt,
                        _acc_gyro[1] / sync_dt,
                        _acc_gyro[2] / sync_dt,
                    ])
                    if USE_MAGNETOMETER:
                        ahrs.update(eff_gyro, _latest_acc, _latest_mag, sync_dt)
                    else:
                        ahrs.update_no_magnetometer(eff_gyro, _latest_acc, sync_dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                    _acc_gyro = [0.0, 0.0, 0.0]
                    _acc_count = 0
                    _last_sync_t = now
                else:
                    continue
            else:
                # --- Per-frame sensor mode dispatch ---
                if SENSOR_MODE == "g":
                    q_gyro = _gyro_integrate(q_gyro, gyro, dt)
                    q_out = q_gyro.copy()
                    e_out = _quat_to_euler(q_out)
                elif SENSOR_MODE == "ga":
                    ahrs.update_no_magnetometer(gyro, acc, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                elif SENSOR_MODE == "gm":
                    ahrs.update(gyro, np.zeros(3), mag, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                elif SENSOR_MODE == "am":
                    ahrs.update(np.zeros(3), acc, mag, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                elif SENSOR_MODE == "a":
                    ahrs.update_no_magnetometer(np.zeros(3), acc, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                elif SENSOR_MODE == "m":
                    ahrs.update(np.zeros(3), np.zeros(3), mag, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()
                else:  # "gam" default
                    if USE_MAGNETOMETER:
                        ahrs.update(gyro, acc, mag, dt)
                    else:
                        ahrs.update_no_magnetometer(gyro, acc, dt)
                    q_out = ahrs.quaternion.wxyz
                    e_out = ahrs.quaternion.to_euler()

            # Track feed interval for diagnostics
            feed_now = time.perf_counter()
            feed_times.append(feed_now)
            if len(feed_times) > 50:
                feed_times.pop(0)
            if len(feed_times) >= 2:
                feed_avg = (feed_now - feed_times[0]) / (len(feed_times) - 1) * 1000
                shared_feed_interval = feed_avg

            with orientation_lock:
                shared_quaternion = np.array(q_out)
                shared_euler = np.array(e_out)
                shared_frame_count += 1

        # Loop playback
        t_start += total_duration
        last_t = time.perf_counter()
        q_gyro = np.array([1.0, 0.0, 0.0, 0.0])
        _acc_gyro = [0.0, 0.0, 0.0]
        _last_sync_t = time.perf_counter()

# ============================================================
# Airplane model
# ============================================================
def create_airplane():
    fuselage = pv.Cylinder(center=(0, 0, 0), direction=(1, 0, 0),
                           radius=0.15, height=3.0, resolution=16)
    nose = pv.Cone(center=(1.8, 0, 0), direction=(1, 0, 0),
                   radius=0.15, height=0.8, resolution=16)
    wing_left = pv.Cube(center=(-0.3, -1.2, 0),
                        x_length=1.0, y_length=0.1, z_length=2.4)
    wing_right = pv.Cube(center=(-0.3, 1.2, 0),
                         x_length=1.0, y_length=0.1, z_length=2.4)
    tail_vert = pv.Cube(center=(-1.2, 0, 0.5),
                        x_length=0.6, y_length=0.08, z_length=1.0)
    tail_horiz = pv.Cube(center=(-1.2, 0, 0),
                         x_length=0.6, y_length=1.6, z_length=0.08)
    return fuselage + nose + wing_left + wing_right + tail_vert + tail_horiz

# ============================================================
# Visualization
# ============================================================
plotter = BackgroundPlotter(window_size=(900, 650), title="IMU Attitude Visualization")
airplane = create_airplane()
airplane_actor = plotter.add_mesh(
    airplane, color="steelblue", specular=0.0, smooth_shading=False)
plotter.add_axes(xlabel="X(fwd)", ylabel="Y(right)", zlabel="Z(up)",
                 viewport=(0.75, 0.0, 0.95, 0.2))  # upper-right corner
plotter.show_grid(xtitle="X", ytitle="Y", ztitle="Z")
hud = plotter.add_text("", position="lower_left",
                        font_size=11, color="red", font="courier")
plotter.camera.position = (8, -6, 4)
plotter.camera.focal_point = (0, 0, 0)
plotter.camera.roll = -90

last_quat = None
last_text = ""
_print_cnt = 0
_render_times = []  # last N render timestamps for FPS calc

def quat_to_matrix(q):
    """Convert quaternion [w,x,y,z] to 4x4 rotation matrix."""
    w, x, y, z = q
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T

# ============================================================
# Render callback (update model only, no serial I/O)
# ============================================================
def render_update():
    global last_quat, last_text, _print_cnt, _render_times

    with orientation_lock:
        q_wxyz = shared_quaternion.copy()
        euler = shared_euler.copy()
        fc = shared_frame_count

    # Always update model matrix
    airplane_actor.user_matrix = quat_to_matrix(q_wxyz)

    # Render interval tracking
    t_now = time.perf_counter()
    _render_times.append(t_now)
    if len(_render_times) > 50:
        _render_times.pop(0)
    render_fps = 0
    render_jitter = 0
    render_cost = 0
    if len(_render_times) >= 2:
        dts = np.diff(_render_times) * 1000  # ms between calls
        dt_avg = (t_now - _render_times[0]) / (len(_render_times) - 1)
        render_fps = 1.0 / dt_avg if dt_avg > 0 else 0
        render_jitter = np.std(dts)
        render_cost = dts[-1]

    # to_euler() returns degrees
    roll_deg = euler[0]
    pitch_deg = euler[1]
    yaw_deg = euler[2]
    global _print_cnt
    _print_cnt += 1
    if _print_cnt % 60 == 0:
        print(f"[{fc:6d}] Roll={roll_deg:7.1f} Pitch={pitch_deg:7.1f} Yaw={yaw_deg:7.1f}")
    text = (f"Roll : {roll_deg: 7.2f} deg\n"
            f"Pitch: {pitch_deg: 7.2f} deg\n"
            f"Yaw  : {yaw_deg: 7.2f} deg\n"
            f"FPS: {render_fps:.0f} jit:{render_jitter:.1f}ms cost:{render_cost:.2f}ms feed:{shared_feed_interval:.1f}ms")
    if text != last_text:
        hud.set_text(0, text)
        last_text = text

# ============================================================
# Start
# ============================================================
if PLAYBACK_FILE:
    t = threading.Thread(target=playback_thread, daemon=True)
    t.start()
    print(f"Playback started from {PLAYBACK_FILE}")
elif ser is not None:
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    # Boost serial thread priority to reduce GIL starvation by Qt
    THREAD_SET_INFORMATION = 0x0020
    handle = ctypes.windll.kernel32.OpenThread(THREAD_SET_INFORMATION, False, t.native_id)
    if handle:
        ctypes.windll.kernel32.SetThreadPriority(handle, 1)  # ABOVE_NORMAL
        ctypes.windll.kernel32.CloseHandle(handle)
    print("IMU data acquisition started (background thread), 3D view ready")
else:
    hud.set_text(0, f"Waiting for serial {SERIAL_PORT} ...\n(static model when no hardware)")

# Hook matrix update into VTK render loop for VSync synchronization
plotter.render_window.AddObserver('StartEvent', lambda *_: render_update())
# Keep a QTimer to trigger VTK render at display rate
plotter.add_callback(lambda: plotter.render(), interval=1)
plotter.app.exec()
