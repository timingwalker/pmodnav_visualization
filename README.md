# PmodNAV IMU 3D Attitude Visualization

Real-time 3D airplane visualization for raw LSM9DS1/PmodNAV serial data.

The current stable path is intentionally simple: **gyro-only orientation display** with a manually supplied effective sample rate. This gives responsive motion and correct 360 degree rotations. Accelerometer and magnetometer fusion remain available for diagnosis, but they are not the default because they caused slow response, pitch fold-back, and yaw/heading pull during large rotations in the current hardware/firmware setup.

## Files

| File | Purpose |
|---|---|
| `main.py` | Main 3D visualization app |
| `diagnose.py` | Serial, gyro integral, magnetometer heading, and AHRS diagnostic tools |
| `profile.py` | Optional performance profiling window |
| `record.py` | Raw serial recorder for playback/debugging |
| `calib.json` | Generated calibration data |
| `doc/REPORT.md` | Current debugging notes and operating decisions |

## Hardware And Serial Data

The firmware sends CSV frames over USB serial:

```text
gx,gy,gz,ax,ay,az,mx,my,mz
```

The parser also accepts an 11-field format with two timestamp fields before the 9 sensor values:

```text
ts_hi,ts_lo,gx,gy,gz,ax,ay,az,mx,my,mz
```

Current defaults:

| Setting | Value |
|---|---|
| Baud rate | `460800` |
| Gyro scale | `245 / 32768` dps/count |
| Accelerometer scale | `4 / 32768` g/count |
| Magnetometer scale | `4 / 32768` gauss/count |
| Effective sample rate for current old firmware | `381 Hz` |

The serial line rate can be higher than the effective IMU update rate. For the currently connected firmware, 381 Hz was validated by integrating a physical 360 degree Y-axis rotation to about 355 degrees.

## Coordinate System

The physical sensor orientation is:

```text
X: forward
Y: right
Z: up
```

For imuFusion's `NWU` convention, Y is negated when using `ga` or `gam` modes:

```python
gyro = [gx, -gy, gz]
acc  = [ax, -ay, az]
mag  = [mx, -my, mz]
```

The main app displays orientation relative to the first received frame, so place the sensor in the desired neutral airplane pose before starting or before the first data frame arrives.

## Install

```bash
python -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run

Recommended command for the current board/firmware:

```bash
.venv/bin/python main.py --port /dev/cu.usbserial-210319AB5A421 --sample-hz 381
```

On Windows, replace the port with something like `COM4`.

The default sensor mode is:

```text
--sensors g
```

This is gyro-only orientation display. It is the current stable mode because it is responsive and supports full 360 degree rotations.

## Useful Options

| Option | Meaning |
|---|---|
| `--port PORT` | Serial port path/name |
| `--baud N` | Serial baud rate, default `460800` |
| `--sample-hz N` | Override effective sample rate; use `381` for the current old firmware |
| `--recal` | Recreate `calib.json` |
| `--sensors g` | Gyro-only display, default and recommended |
| `--sensors ga` | Gyro + accelerometer fusion, useful for drift experiments but not full flips |
| `--sensors gam --mag` | Gyro + accelerometer + magnetometer fusion, diagnostic only for now |

Advanced options kept for diagnostics:

| Option | Meaning |
|---|---|
| `--playback FILE` | Replay a recorded raw-data file instead of opening serial |
| `--gain N` | Override AHRS fusion gain for `ga`/`gam` experiments |
| `--acc-rej N` | Override accelerometer rejection threshold |
| `--mag-rej N` | Override magnetometer rejection threshold |
| `--mag` / `--no-mag` | Enable or disable magnetometer fusion when using `gam`; default is off |

## Calibration

Run calibration when changing boards or after a bad `calib.json`:

```bash
.venv/bin/python main.py --port /dev/cu.usbserial-210319AB5A421 --sample-hz 381 --recal
```

`calib.json` currently stores:

| Field | Meaning |
|---|---|
| `gyro_bias` | Gyroscope zero-rate bias |
| `mag_offset` | Magnetometer hard-iron offset |
| `gz_deadband` | Small Z gyro deadband derived from calibration noise |

For default gyro-only display, the gyro bias is the most important part.

## Diagnostics

Raw serial watch:

```bash
.venv/bin/python diagnose.py --raw-watch --port /dev/cu.usbserial-210319AB5A421
```

Validate gyro integration for a physical 360 degree Y-axis rotation:

```bash
.venv/bin/python diagnose.py --gyro-integral --port /dev/cu.usbserial-210319AB5A421 --axis y --sample-hz 381 --print-hz 2
```

Show all three integrated gyro axes for a single motion:

```bash
.venv/bin/python diagnose.py --gyro-integral --port /dev/cu.usbserial-210319AB5A421 --axis all --sample-hz 381 --print-hz 2
```

AHRS watch:

```bash
.venv/bin/python diagnose.py --ahrs-watch --port /dev/cu.usbserial-210319AB5A421 --sensors ga --sample-hz 381 --print-hz 2
```

## Current Findings

- Pure gyro mode with `--sample-hz 381` matches the desired behavior best.
- `ga` and `gam` fusion can make pitch rotations fold back instead of completing 360 degrees.
- Magnetometer fusion currently pulls yaw during pitch motion, causing the airplane to move along a wrong diagonal path.
- The magnetometer should remain off in normal use until its calibration and coordinate behavior are revalidated.

## Notes

- Pure gyro display will slowly drift over time because any residual gyro bias is integrated. Recalibrate or restart in the neutral pose when needed.
- Accelerometer fusion can correct long-term roll/pitch drift, but it is not suitable for arbitrary full 360 degree rotations without careful handling.
- Magnetometer fusion is useful for heading correction only after calibration and axis mapping are proven stable.
