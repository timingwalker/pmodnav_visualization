import sys
import os
import json
import time
import ctypes
import threading
import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter
import imufusion

# Raise Windows timer resolution to 1ms
ctypes.windll.winmm.timeBeginPeriod(1)

# ============================================================
# Configuration
# ============================================================
SERIAL_PORT = "COM4"
BAUD_RATE = 115200
SAMPLE_PERIOD = 1 / 85

GYRO_SCALE = 245.0 / 32768.0 * np.pi / 180.0
ACC_SCALE = 2.0 / 32768.0
MAG_SCALE = 4.0 / 32768.0

GAIN = 0.1             # Lower = trust gyro more (responsive); higher = trust accel/mag more
GYRO_RANGE = 245.0
ACC_REJECTION = 1.0
MAG_REJECTION = 2.0
RECOVERY_PERIOD = 3
CALIBRATION_FRAMES = 100
USE_MAGNETOMETER = True

# ============================================================
# Serial port
# ============================================================
try:
    import serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"Serial {SERIAL_PORT} opened")
except Exception as e:
    print(f"Cannot open serial {SERIAL_PORT}: {e}")
    ser = None

# ============================================================
# Calibration (load existing or re-acquire)
# ============================================================
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib.json")
gyro_bias = np.zeros(3)
mag_offset = np.zeros(3)

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
        print(f"Calibration loaded ({CALIB_FILE})")
        print(f"  Gyro bias: X={gyro_bias[0]*180/np.pi:.2f} Y={gyro_bias[1]*180/np.pi:.2f} Z={gyro_bias[2]*180/np.pi:.2f} deg/s")
        print(f"  Hard-iron: X={mag_offset[0]:.0f} Y={mag_offset[1]:.0f} Z={mag_offset[2]:.0f}")
    else:
        if "--recal" in sys.argv:
            print("Force recalibrating...")
        # Gyro bias
        print(f"Calibrating gyro bias (keep sensor still, {CALIBRATION_FRAMES} frames)...")
        samples = 0
        while samples < CALIBRATION_FRAMES:
            raw = ser.readline().decode(errors="ignore")
            vals = parse_line(raw)
            if vals is None:
                continue
            gx, gy, gz = vals[0], vals[1], vals[2]
            gyro_bias[0] += gx * GYRO_SCALE
            gyro_bias[1] += gy * GYRO_SCALE
            gyro_bias[2] += gz * GYRO_SCALE
            samples += 1
        gyro_bias /= CALIBRATION_FRAMES
        bias_dps = gyro_bias * 180 / np.pi
        print(f"  Gyro bias: X={bias_dps[0]:.2f} Y={bias_dps[1]:.2f} Z={bias_dps[2]:.2f} deg/s")

        # Magnetometer hard-iron calibration
        print("Magnetometer calibration: rotate sensor slowly (20 sec)...")
        mag_min = np.full(3, 1e9)
        mag_max = np.full(3, -1e9)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 20.0:
            raw = ser.readline().decode(errors="ignore")
            vals = parse_line(raw)
            if vals is None:
                continue
            mx, my, mz = vals[6], vals[7], vals[8]
            mag_min = np.minimum(mag_min, [mx, my, mz])
            mag_max = np.maximum(mag_max, [mx, my, mz])
        mag_offset = (mag_max + mag_min) / 2.0
        print(f"  Hard-iron offset: X={mag_offset[0]:.0f} Y={mag_offset[1]:.0f} Z={mag_offset[2]:.0f}")

        # Save to file
        json.dump({
            "gyro_bias": gyro_bias.tolist(),
            "mag_offset": mag_offset.tolist(),
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
if ser is not None:
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

# ============================================================
# Serial read thread
# ============================================================
def serial_thread():
    global shared_quaternion, shared_euler, shared_frame_count
    buf = b""

    while True:
        try:
            # Read all available bytes
            chunk = ser.read(max(1, ser.in_waiting or 1))
            if not chunk:
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
                ax *= ACC_SCALE
                ay *= ACC_SCALE
                az *= ACC_SCALE
                mx = (mx - mag_offset[0]) * MAG_SCALE
                my = (my - mag_offset[1]) * MAG_SCALE
                mz = (mz - mag_offset[2]) * MAG_SCALE

                gyro = np.array([gx, -gy, gz], dtype=np.float64)
                acc = np.array([ax, -ay, az], dtype=np.float64)
                mag = np.array([mx, my, mz], dtype=np.float64)

                if USE_MAGNETOMETER:
                    ahrs.update(gyro, acc, mag, SAMPLE_PERIOD)
                else:
                    ahrs.update_no_magnetometer(gyro, acc, SAMPLE_PERIOD)

                # to_euler() returns degrees
                q = ahrs.quaternion.wxyz
                e = ahrs.quaternion.to_euler()
                with orientation_lock:
                    shared_quaternion = np.array(q)
                    shared_euler = np.array(e)
                    shared_frame_count += 1

        except Exception:
            time.sleep(0.001)

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
    airplane, color="steelblue", specular=0.4, smooth_shading=True)
plotter.add_axes(xlabel="X(fwd)", ylabel="Y(right)", zlabel="Z(up)")
plotter.show_grid(xtitle="X", ytitle="Y", ztitle="Z")
hud = plotter.add_text("Waiting for data...", position="upper_left",
                        font_size=13, color="white", font="courier")
plotter.camera.position = (8, -6, 4)
plotter.camera.focal_point = (0, 0, 0)
plotter.camera.roll = -90

last_quat = None
last_text = ""
_print_cnt = 0

# ============================================================
# Render callback (update model only, no serial I/O)
# ============================================================
def render_update():
    global last_quat, last_text, _print_cnt

    with orientation_lock:
        q_wxyz = shared_quaternion.copy()
        euler = shared_euler.copy()
        fc = shared_frame_count

    # Update model (only when attitude changes)
    if last_quat is None or not np.allclose(q_wxyz, last_quat, atol=1e-7):
        last_quat = q_wxyz
        w, x, y, z = q_wxyz
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        airplane_actor.user_matrix = T

    # to_euler() returns degrees
    roll_deg = euler[0]
    pitch_deg = euler[1]
    yaw_deg = euler[2]
    global _print_cnt
    _print_cnt += 1
    if _print_cnt % 30 == 0:
        print(f"[{fc:6d}] qz={q_wxyz[3]:.4f} | Roll={roll_deg:7.1f} Pitch={pitch_deg:7.1f} Yaw={yaw_deg:7.1f}")
    text = (f"Roll : {roll_deg: 7.2f} deg\n"
            f"Pitch: {pitch_deg: 7.2f} deg\n"
            f"Yaw  : {yaw_deg: 7.2f} deg\n"
            f"Frames: {fc}")
    if text != last_text:
        hud.set_text(0, text)
        last_text = text

# ============================================================
# Start
# ============================================================
if ser is not None:
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    plotter.add_callback(render_update, interval=10)
    print("IMU data acquisition started (background thread), 3D view ready")
else:
    hud.set_text(0, f"Waiting for serial {SERIAL_PORT} ...\n(static model when no hardware)")

plotter.app.exec()
