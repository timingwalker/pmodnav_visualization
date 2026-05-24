# PMODNAV Visualization Current Report

Last updated: 2026-05-24

## Current Stable Configuration

Run the visualization with:

```bash
.venv/bin/python main.py --port /dev/cu.usbserial-210319AB5A421 --sample-hz 381
```

This uses:

| Item | Current value |
|---|---|
| Serial baud | `460800` |
| Display mode | Startup-relative orientation |
| Sensor mode | `g`, gyro-only |
| Effective sample rate | `381 Hz` for the current old firmware |
| Magnetometer fusion | Off by default |

This configuration gives the expected behavior for responsive 360 degree rotations.

## Why Gyro-Only Is Default

The project originally attempted full `gyro + accelerometer + magnetometer` fusion. Live tests showed that fusion made the visual behavior worse for this use case:

- `ga` can use gravity to pull pitch/roll back toward the nearest gravity-consistent orientation, so full 360 degree flips can fold back.
- `gam` additionally let magnetometer heading pull yaw during a pure pitch motion, making the airplane move along a diagonal path.
- Pure gyro integration with the corrected effective sample rate gave the expected full-turn behavior.

Pure gyro mode can drift over time because residual gyro bias is integrated. For this visualization, that tradeoff is currently better than incorrect fused motion.

## Sample Rate Finding

Counting serial CSV lines showed a line rate around 440-460 Hz, but physical integration showed the effective IMU update rate was lower.

Test:

```bash
.venv/bin/python diagnose.py --gyro-integral --port /dev/cu.usbserial-210319AB5A421 --axis y --sample-hz 381 --print-hz 2
```

Result for a physical 360 degree Y-axis rotation:

```text
Final integrated y-axis angle: about -355 deg
```

Therefore `--sample-hz 381` is used for the current firmware. New firmware should be revalidated with the same `--gyro-integral` test.

## Coordinate System

Physical sensor axes:

```text
X: forward
Y: right
Z: up
```

When using imuFusion's `NWU` convention in `ga`/`gam` modes, map sensor data as:

```python
gyro = [gx, -gy, gz]
acc  = [ax, -ay, az]
mag  = [mx, -my, mz]
```

The main gyro-only path uses the same mapped gyro axes.

## Magnetometer Status

Magnetometer hard-iron calibration is stored in `calib.json`, but magnetometer fusion is not considered stable yet.

Observed issue:

- During a pure pitch-up motion, gyro and accelerometer data matched the expected axis.
- With `gam`, yaw changed substantially and `rel_nose_y` became large.
- This made the airplane appear to climb diagonally even though the physical motion was nearly pure pitch.

Recommendation:

- Keep normal visualization in gyro-only mode.
- Use `--sensors gam --mag` only for diagnostics.
- Revalidate magnetometer calibration and tilt compensation before using it for heading correction.

## Useful Commands

Raw data:

```bash
.venv/bin/python diagnose.py --raw-watch --port /dev/cu.usbserial-210319AB5A421
```

Y-axis gyro integral:

```bash
.venv/bin/python diagnose.py --gyro-integral --port /dev/cu.usbserial-210319AB5A421 --axis y --sample-hz 381 --print-hz 2
```

All-axis gyro integral:

```bash
.venv/bin/python diagnose.py --gyro-integral --port /dev/cu.usbserial-210319AB5A421 --axis all --sample-hz 381 --print-hz 2
```

AHRS watch without magnetometer:

```bash
.venv/bin/python diagnose.py --ahrs-watch --port /dev/cu.usbserial-210319AB5A421 --sensors ga --sample-hz 381 --print-hz 2
```

AHRS watch with magnetometer:

```bash
.venv/bin/python diagnose.py --ahrs-watch --port /dev/cu.usbserial-210319AB5A421 --sensors gam --sample-hz 381 --print-hz 2
```

## Code Notes

- `main.py` is the primary app.
- `diagnose.py` keeps focused tools for raw serial validation and motion analysis.
- `profile.py` is optional and intended for performance checks, not for validating motion correctness.
- `record.py` can capture raw serial streams for later playback/debugging.
- `main.py` keeps `--playback`, `--gain`, `--acc-rej`, `--mag-rej`, `--mag`, and `--no-mag` only as diagnostic options. The normal path is still gyro-only with `--sample-hz 381`.
