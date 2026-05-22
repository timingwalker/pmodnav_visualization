"""End-to-end performance measurement."""
import time
import ctypes
import gc
import threading
import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter
import imufusion
import serial

ctypes.windll.winmm.timeBeginPeriod(1)
gc.disable()

SERIAL_PORT = "COM4"
BAUD_RATE = 460800
SAMPLE_PERIOD = 1 / 449
GYRO_SCALE = 245.0 / 32768.0 * np.pi / 180.0
ACC_SCALE = 4.0 / 32768.0
MAG_SCALE = 4.0 / 32768.0
CALIB = 100

# ── Serial ──
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
print(f"Serial {SERIAL_PORT} opened")

# ── Calibration ──
gyro_bias = np.zeros(3)
print(f"Calibrating ({CALIB} frames)...")
samples = 0
while samples < CALIB:
    raw = ser.readline().decode(errors="ignore").strip()
    parts = raw.split(",")
    if len(parts) == 11:
        try:
            vals = [int(p) for p in parts[2:]]
        except ValueError:
            continue
    elif len(parts) == 9:
        try:
            vals = [int(p) for p in parts]
        except ValueError:
            continue
    else:
        continue
    gyro_bias[0] += vals[0] * GYRO_SCALE
    gyro_bias[1] += vals[1] * GYRO_SCALE
    gyro_bias[2] += vals[2] * GYRO_SCALE
    samples += 1
gyro_bias /= CALIB
print(f"Gyro bias: {np.linalg.norm(gyro_bias)*180/np.pi:.2f} deg/s")

# ── AHRS ──
settings_obj = imufusion.Settings(imufusion.CONVENTION_NWU, 0.5, 245.0, 1.0, 2.0, 3)
ahrs = imufusion.Ahrs(settings_obj)

# ── AHRS Init ──
print("AHRS initializing (keep sensor still)...")
init_frames = 0
while ahrs.flags.initialising:
    raw = ser.readline().decode(errors="ignore").strip()
    parts = raw.split(",")
    if len(parts) == 11:
        try:
            vals = [int(p) for p in parts[2:]]
        except ValueError:
            continue
    elif len(parts) == 9:
        try:
            vals = [int(p) for p in parts]
        except ValueError:
            continue
    else:
        continue
    gx, gy, gz = vals[0], vals[1], vals[2]
    ax, ay, az = vals[3], vals[4], vals[5]
    mx, my, mz = vals[6], vals[7], vals[8]
    gx = gx * GYRO_SCALE - gyro_bias[0]
    gy = gy * GYRO_SCALE - gyro_bias[1]
    gz = gz * GYRO_SCALE - gyro_bias[2]
    ax *= ACC_SCALE; ay *= ACC_SCALE; az *= ACC_SCALE
    mx *= MAG_SCALE; my *= MAG_SCALE; mz *= MAG_SCALE
    ahrs.update(
        np.array([gx, -gy, gz], dtype=np.float64),
        np.array([ax, -ay, az], dtype=np.float64),
        np.array([mx, -my, mz], dtype=np.float64),
        SAMPLE_PERIOD,
    )
    init_frames += 1
    if init_frames % 100 == 0:
        print(f"  Waited {init_frames} frames...")
print(f"  AHRS initialized ({init_frames} frames)")

# ── Shared data ──
lock = threading.Lock()
stats = {
    "q_list": [],
    "serial_count": 0,
    "serial_intervals": [],
    "serial_last_ts": time.perf_counter(),
    "render_intervals": [],
    "ahrs_us": [],
}
render_times = [time.perf_counter()]
running = True

# ── Serial thread ──
def serial_loop():
    buf = b""
    global running
    last_ts = None  # previous frame hardware timestamp (microseconds)
    while running:
        try:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                buf += chunk

            if b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode(errors="ignore").strip()
                parts = line.split(",")
                if len(parts) == 11:
                    try:
                        ts_hi = int(parts[0]) & 0xFFFFFFFF
                        ts_lo = int(parts[1]) & 0xFFFFFFFF
                        ts_us = ((ts_hi << 32) | ts_lo) / 25.0
                        vals = [int(p) for p in parts[2:]]
                    except ValueError:
                        continue
                elif len(parts) == 9:
                    try:
                        ts_us = 0
                        vals = [int(p) for p in parts]
                    except ValueError:
                        continue
                else:
                    continue

                gx, gy, gz = vals[0:3]
                ax, ay, az = vals[3:6]
                mx, my, mz = vals[6:9]

                gx = gx * GYRO_SCALE - gyro_bias[0]
                gy = gy * GYRO_SCALE - gyro_bias[1]
                gz = gz * GYRO_SCALE - gyro_bias[2]
                ax *= ACC_SCALE; ay *= ACC_SCALE; az *= ACC_SCALE
                mx *= MAG_SCALE; my *= MAG_SCALE; mz *= MAG_SCALE

                if ts_us > 0 and last_ts is not None:
                    dt = (ts_us - last_ts) / 1_000_000.0
                    if dt > 0.1 or dt <= 0:
                        dt = SAMPLE_PERIOD
                else:
                    dt = SAMPLE_PERIOD
                last_ts = ts_us
                t1 = time.perf_counter()
                ahrs.update(
                    np.array([gx, -gy, gz], dtype=np.float64),
                    np.array([ax, -ay, az], dtype=np.float64),
                    np.array([mx, -my, mz], dtype=np.float64),
                    dt,
                )
                t2 = time.perf_counter()

                with lock:
                    now = time.perf_counter()
                    si = (now - stats["serial_last_ts"]) * 1000
                    stats["serial_last_ts"] = now
                    stats["serial_count"] += 1
                    stats["serial_intervals"].append(si)
                    stats["ahrs_us"].append((t2 - t1) * 1e6)
                    q = ahrs.quaternion.wxyz
                    stats["q_list"].append(np.array(q))
                    if len(stats["q_list"]) > 200:
                        stats["q_list"].pop(0)
            else:
                time.sleep(0.001)

        except Exception:
            time.sleep(0.001)

    print("Serial thread exited")

# ── Visualization ──
plotter = BackgroundPlotter(window_size=(900, 650), title="Profile")
airplane = (
    pv.Cylinder(center=(0, 0, 0), direction=(1, 0, 0), radius=0.15, height=3.0) +
    pv.Cone(center=(1.8, 0, 0), direction=(1, 0, 0), radius=0.15, height=0.8) +
    pv.Cube(center=(-0.3, -1.2, 0), x_length=1.0, y_length=0.1, z_length=2.4) +
    pv.Cube(center=(-0.3, 1.2, 0), x_length=1.0, y_length=0.1, z_length=2.4) +
    pv.Cube(center=(-1.2, 0, 0.5), x_length=0.6, y_length=0.08, z_length=1.0) +
    pv.Cube(center=(-1.2, 0, 0), x_length=0.6, y_length=1.6, z_length=0.08)
)
actor = plotter.add_mesh(airplane, color="steelblue", specular=0.4, smooth_shading=True)
plotter.add_axes(xlabel="X", ylabel="Y", zlabel="Z")
plotter.show_grid(xtitle="X", ytitle="Y", ztitle="Z")
hud = plotter.add_text("Starting...", position="upper_left", font_size=13, color="white")
plotter.camera.position = (8, -6, 4)
plotter.camera.focal_point = (0, 0, 0)
plotter.camera.roll = -90

last_q = None
render_count = 0

def render_cb():
    global last_q, render_count
    now = time.perf_counter()
    dt = (now - render_times[0]) * 1000
    render_times[0] = now
    render_count += 1

    with lock:
        stats["render_intervals"].append(dt)
        sc = stats["serial_count"]
        qlist = stats["q_list"]
        ahrs_data = stats["ahrs_us"][-50:] if stats["ahrs_us"] else []

    # Update model
    if qlist and (last_q is None or not np.allclose(qlist[-1], last_q, atol=1e-6)):
        last_q = qlist[-1]
        w, x, y, z = last_q
        R = np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
            [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
            [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        actor.user_matrix = T

    # HUD
    if len(stats["render_intervals"]) > 10:
        ri = stats["render_intervals"][-100:]
        avg_ms = sum(ri) / len(ri)
        fps = 1000 / avg_ms if avg_ms > 0 else 0
        ahrs_avg = sum(ahrs_data) / len(ahrs_data) if ahrs_data else 0
        text = (f"Render: {avg_ms:5.1f}ms / {fps:5.1f} FPS\n"
                f"Serial frames: {sc}\n"
                f"AHRS: {ahrs_avg:.1f} us")
        hud.set_text(0, text)

# ── Start ──
t = threading.Thread(target=serial_loop, daemon=True)
t.start()
# Boost serial thread priority to reduce GIL starvation by Qt
THREAD_SET_INFORMATION = 0x0020
handle = ctypes.windll.kernel32.OpenThread(THREAD_SET_INFORMATION, False, t.native_id)
if handle:
    ctypes.windll.kernel32.SetThreadPriority(handle, 1)  # ABOVE_NORMAL
    ctypes.windll.kernel32.CloseHandle(handle)
time.sleep(0.5)
print(f"Received {stats['serial_count']} serial frames after startup")

plotter.add_callback(render_cb, interval=5)
print("Window open, close it to see report...\n")
plotter.app.exec()

# ── Report ──
running = False
time.sleep(0.2)
ser.close()

with lock:
    sc = stats["serial_count"]
    ri = stats["render_intervals"]
    ahrs_us = stats["ahrs_us"]
    q_list = stats["q_list"]

print(f"\n{'='*60}")
print(f"                Performance Report")
print(f"{'='*60}")

if ri:
    ri_stable = ri[10:]
    avg = sum(ri_stable) / len(ri_stable)
    print(f"\n[1] Render callback ({render_count} calls)")
    print(f"    Avg interval: {avg:.1f} ms  ->  {1000/avg:.1f} FPS")
    print(f"    Min: {min(ri_stable):.1f} ms  Max: {max(ri_stable):.1f} ms")
    print(f"    Jitter: {max(ri_stable)-min(ri_stable):.1f} ms")
    print(f"\n    All intervals (ms):")
    for i, v in enumerate(ri_stable):
        marker = " <-- spike" if v > avg * 3 else ""
        print(f"      [{i+1:4d}] {v:6.1f}{marker}")
else:
    print("\n[1] Render callback: no data!")

si = stats["serial_intervals"]
if si:
    si_stable = si[10:] if len(si) > 10 else si
    avg_si = sum(si_stable) / len(si_stable)
    print(f"\n[2] Serial data")
    print(f"    Total frames: {sc}")
    print(f"    Avg interval: {avg_si:.1f} ms  ->  {1000/avg_si:.1f} Hz")
    print(f"    Min interval: {min(si_stable):.1f} ms")
    print(f"    Max interval: {max(si_stable):.1f} ms")
    print(f"\n    All intervals (ms):")
    for i, v in enumerate(si_stable):
        marker = " <-- spike" if v > avg_si * 3 else ""
        print(f"      [{i+1:4d}] {v:6.1f}{marker}")
else:
    print(f"\n[2] Serial data: total={sc}, no interval data")

if ahrs_us:
    print(f"\n[3] AHRS.update() timing")
    print(f"    Avg: {sum(ahrs_us)/len(ahrs_us):.1f} us")
    print(f"    Max: {max(ahrs_us):.1f} us")

if len(q_list) > 10:
    diffs = [np.linalg.norm(np.array(q_list[i]) - np.array(q_list[i-1]))
             for i in range(1, len(q_list))]
    print(f"\n[4] Quaternion frame delta")
    print(f"    Avg: {sum(diffs)/len(diffs):.6f}")
    print(f"    Max: {max(diffs):.6f}")

print()
