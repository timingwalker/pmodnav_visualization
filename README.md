# PmodNAV IMU 3D Attitude Visualization

[English](#english) | [中文](#chinese)

> This project was developed with the assistance of **DeepSeek V4-Pro** (via Claude Code), totaling ~1.53 billion tokens. See [doc/REPORT.md](doc/REPORT.md) for the full development journey.

<a id="english"></a>
Real-time 3D visualization of the LSM9DS1 on a [PmodNAV](https://digilent.com/reference/pmod/pmodnav) module using sensor fusion and pyvista.

Reads 9-axis IMU data (gyro + accelerometer + magnetometer) from the PmodNAV via serial port, fuses them with [imuFusion](https://github.com/xioTechnologies/Fusion), and displays a real-time airplane model in 3D.

## Directory Structure

```
├── main.py              Main program
├── diagnose.py          Step-by-step diagnostic tool for debugging
├── profile.py           Performance measurement (frame rate, latency, AHRS timing)
├── calib.json           Calibration data (auto-generated)
├── requirements.txt     Python dependencies
├── doc/REPORT.md            Full development report (Chinese)
```

## Hardware Setup

- **Sensor**: LSM9DS1 on a [PmodNAV](https://digilent.com/reference/pmod/pmodnav) module (SPI), connected to a microcontroller (STM32 or similar)
- **Connection**: USB serial (COM port)
- **Baud rate**: 115200
- **Output rate**: ~85 Hz (configurable in firmware)

## Serial Data Format

The firmware reads LSM9DS1 registers and outputs one line per sample:

```
gx,gy,gz,ax,ay,az,mx,my,mz\r\n
```

| Field | Description | Range |
|---|---|---|
| `gx, gy, gz` | Gyroscope raw ADC (int16) | ±245 dps |
| `ax, ay, az` | Accelerometer raw ADC (int16) | ±2 g |
| `mx, my, mz` | Magnetometer raw ADC (int16) | ±4 Gauss |

### Sensor Register Configuration

| Register | Value | Meaning |
|---|---|---|
| `CTRL_REG1_G` (0x10) | 0x60 | Gyro ±245 dps, 119 Hz |
| `CTRL_REG6_XL` (0x1F) | 0x38 | Accel ±2g, 119 Hz |
| `CTRL_REG1_M` (0x20) | 0x7E | Mag 80 Hz |
| `CTRL_REG2_M` (0x21) | 0x00 | Mag ±4 Gauss |

### Scale Factors

```python
GYRO_SCALE = 245.0 / 32768.0 * pi / 180.0   # dps → rad/s
ACC_SCALE  = 2.0 / 32768.0                    # g
MAG_SCALE  = 4.0 / 32768.0                    # Gauss
```

## Coordinate System

The sensor is placed on a desk with the user facing the monitor:

```
Sensor X → Forward (toward monitor) = North
Sensor Y → Right hand side           = East
Sensor Z → Upward                    = Up
```

We use the **NWU** (North-West-Up) convention from imuFusion. Since the sensor's Y is East (not West), only Y is negated:

```python
gyro: [ gx, -gy,  gz]     # Y negated (East → West)
acc:  [ ax, -ay,  az]     # Y negated
mag:  [ mx,  my,  mz]     # NOT negated (heading via atan2)
```

## Quick Start

### 1. Set COM port

Open [main.py](main.py) and change `SERIAL_PORT` to match your device's COM port (e.g., `COM3`, `COM5`, etc. on Windows; `/dev/ttyUSB0` on Linux):

```python
SERIAL_PORT = "COM4"   # ← change this to your actual port
```

> On Windows, find the COM number in **Device Manager → Ports (COM & LPT)**. On Linux, check `ls /dev/tty*`.

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

### 3. Calibrate (first run only)

Connect the sensor and keep it still, then run:

```bash
python main.py --recal
```

Follow the prompts:
- Keep the sensor **still** for gyro bias calibration (~1.2s)
- Slowly rotate the sensor **in all directions** for magnetometer calibration (20s)
- Keep the sensor **still** until AHRS initialization completes

Calibration data is saved to `calib.json` and reused on subsequent runs.

### 4. Run

```bash
python main.py
```

The 3D window opens once AHRS initialization is complete. Move the sensor and watch the airplane follow.

To force recalibration at any time:

```bash
python main.py --recal
```

## AHRS Parameters

| Parameter | Value | Notes |
|---|---|---|
| Convention | NWU | North-West-Up |
| Gain | 0.1 | Lower = faster response (gyro trust > accel/mag) |
| Gyro range | 245 dps | Matches sensor hardware |

These can be adjusted in `main.py` (lines 26–31).

## Diagnostics

If something seems wrong, use the step-by-step diagnostic tool:

```bash
python diagnose.py
```

It walks through 4 steps:
1. **Raw ADC** — verify serial data is arriving
2. **SI Conversion** — verify scale factors and bias calibration
3. **Coordinate Mapping** — verify NWU axis mapping
4. **AHRS Attitude** — verify orientation angles

Each step prints live data. Press `Ctrl+C` to advance to the next step.

## Performance Profiling

```bash
python profile.py
```

Measures render frame rate, serial data rate, AHRS computation time, and quaternion change rate. Displays real-time FPS in the 3D window and prints a summary report when closed.

## Notes

- If you change environments (different desk/location), run `python main.py --recal` to recalibrate the magnetometer.
- `imuFusion.quaternion.to_euler()` returns **degrees** directly (not radians). Do not apply `np.degrees()`.
- pyvista 0.48 requires PyQt6 and pyvistaqt; older API calls like `add_timer_callback` or `SetOrientation` no longer work.

---

<a id="chinese"></a>
## 中文概述

> 本项目由 **DeepSeek V4-Pro**（通过 Claude Code）辅助开发，总计消耗约 1.53 亿 token。完整开发过程见 [doc/REPORT.md](doc/REPORT.md)。

基于 [PmodNAV](https://digilent.com/reference/pmod/pmodnav) 模块上的 LSM9DS1 九轴传感器，通过串口读取数据，使用 [imuFusion](https://github.com/xioTechnologies/Fusion) 进行姿态融合，pyvista 实时 3D 渲染飞机模型。

### 目录结构

```
├── main.py              主程序
├── diagnose.py           分步诊断工具（原始数据 → 姿态角）
├── profile.py            全链路性能测量
├── calib.json            校准数据（自动生成，可复用）
├── requirements.txt      Python 依赖
├── doc/REPORT.md             完整开发报告（中文）
```

### 串口数据格式

固件以 115200 波特率、约 85 Hz 频率输出，每行 9 个逗号分隔的 int16：

```
gx,gy,gz,ax,ay,az,mx,my,mz\r\n
```

| 字段 | 含义 | 量程 |
|---|---|---|
| `gx, gy, gz` | 陀螺仪原始 ADC | ±245 dps |
| `ax, ay, az` | 加速度计原始 ADC | ±2 g |
| `mx, my, mz` | 磁力计原始 ADC | ±4 Gauss |

### 坐标系

传感器平放桌面、正对显示器：

```
X → 前（北）    Y → 右（东）    Z → 上
```

采用 imuFusion 的 **NWU**（北-西-上）约定，传感器 Y 为东而非西，需取反：

```python
gyro: [ gx, -gy,  gz]    # Y 取反（东 → 西）
acc:  [ ax, -ay,  az]    # Y 取反
mag:  [ mx,  my,  mz]    # 不取反（atan2 航向计算）
```

### 快速开始

**1. 修改串口号**：打开 [main.py](main.py)，将 `SERIAL_PORT` 改为实际使用的 COM 编号（Windows 在设备管理器的"端口"中查看；Linux 查看 `/dev/tty*`）：

```python
SERIAL_PORT = "COM4"   # ← 改为实际端口号
```

**2. 安装依赖**：

```bash
pip install -r requirements.txt
```

**3. 首次校准**：

```bash
python main.py --recal
```

**4. 正常运行**：

```bash
python main.py
```

校准过程：① 静止 100 帧测陀螺零偏 → ② 各方向慢旋 20 秒测磁力计硬铁 → ③ 静止等 AHRS 初始化 → 窗口打开，可以移动传感器。

### 诊断与性能分析

```bash
python diagnose.py    # 逐步排查：原始ADC → SI转换 → 坐标映射 → 姿态角
python profile.py     # 全链路性能：渲染帧率、串口速率、AHRS 耗时
```

### 注意事项

- 更换使用环境后建议 `python main.py --recal` 重新校准磁力计
- `imuFusion.quaternion.to_euler()` 返回值**已经是度数**，不要再调 `np.degrees()`
- pyvista 0.48 依赖 PyQt6 + pyvistaqt，旧版 API（`add_timer_callback`、`SetOrientation` 等）已变更
- 互补滤波 gain 越小陀螺权重越大、响应越快
