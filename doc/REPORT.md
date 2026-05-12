# IMU 3D 姿态可视化 — 完整开发报告

---

## 项目概述

**路径**：`E:\02_Project-TBD\IMU\deepseek\`

**功能**：串口读取 LSM9DS1 九轴数据 → `imuFusion` 传感器融合 → `pyvista` 3D 实时显示飞机模型

**最终文件**：

| 文件 | 功能 |
|---|---|
| `main.py` | 主程序入口 |
| `diagnose.py` | 4 步诊断：原始 ADC → SI 转换 → 坐标映射 → AHRS 姿态角 |
| `profile.py` | 全链路性能测量（渲染帧率 / 串口速率 / AHRS 耗时） |
| `calib.json` | 持久化校准数据 |
| `requirements.txt` | Python 依赖清单 |

---

## 技术方案

| 项目 | 选择 |
|---|---|
| Python | 3.14.4 |
| 传感器融合 | `imuFusion` 1.2.11（AHRS 互补滤波） |
| 3D 渲染 | `pyvista` 0.48.1 + `pyvistaqt` BackgroundPlotter |
| Qt 后端 | `PyQt6` 6.11.0 |
| 串口 | `pyserial` 3.5，115200 baud |
| 数据率 | ~85 Hz（固件 `usleep(8000)` 实测） |
| 刻度因子 | 陀螺 ±245dps、加计 ±2g、磁力计 ±4Gauss（均从寄存器反推验证） |

### 坐标系映射

传感器实际方向（正对显示器）：

```
X → 前（北）    Y → 右（东）    Z → 上
```

imuFusion 三种内置约定（ENU / NED / NWU）都不直接匹配。最接近的是 **NWU**（X=北、Y=西、Z=上），只需把 Y 从东翻到西：

```
         传感器实际         NWU 约定         映射
X:        前(北)     →     X=北             gx →  gx   不变
Y:        右(东)     →     Y=西             gy → -gy   取反（东→西）
Z:        上         →     Z=上             gz →  gz   不变

gyro: [ gx, -gy,  gz]    ← 仅 Y 取反
acc:  [ ax, -ay,  az]    ← 仅 Y 取反
mag:  [ mx,  my,  mz]    ← 不取反（实践中验证：atan2 航向计算，取反会反转方向）
```

### AHRS 参数

| 参数 | 值 | 说明 |
|---|---|---|
| gain | 0.1 | 越小越信陀螺（跟手），越大越信加计/磁力计（稳但慢） |
| gyroscope_range | 245 dps | 匹配传感器硬件量程 |
| acceleration_rejection | 1.0 s | 加速度纠正恢复时间 |
| magnetic_rejection | 2.0 s | 磁力计纠正恢复时间 |
| recovery_trigger_period | 3 samples | 恢复触发采样数 |

### 数据流

```
[1] 串口原始 ADC（gx,gy,gz,ax,ay,az,mx,my,mz\r\n — 9 个 int16）
        ↓
[2] 刻度转换 → SI 单位
    gyro: raw × (245/32768) × (π/180) → rad/s，减陀螺零偏
    acc:  raw × (2/32768) → g
    mag:  (raw − 硬铁偏移) × (4/32768) → Gauss
        ↓
[3] 坐标映射 → NWU（仅 Y 取反）
        ↓
[4] imuFusion.Ahrs.update() → 四元数 quaternion.wxyz
        ↓
[5] to_matrix() → R → airplane_actor.user_matrix = R
        ↓
[6] BackgroundPlotter + Qt 事件循环渲染
```

串口读取放在独立线程，AHRS 更新在线程内完成，渲染回调只从共享变量读取最新姿态更新模型，互不阻塞。

### 架构

```
main.py
├── 配置区（串口、刻度因子、AHRS 参数）
├── 校准模块（陀螺零偏 + 磁力计硬铁，持久化到 calib.json）
├── AHRS 初始化（等待 initialising 标志清除后再开窗）
├── 串口读取线程（持续读取 → 映射 → AHRS 更新 → 写入共享变量）
├── 飞机模型（pyvista 几何体：机身圆柱 + 机头圆锥 + 机翼 + 尾翼）
├── 渲染回调（读共享变量 → 更新 user_matrix + HUD 显示 Roll/Pitch/Yaw）
└── BackgroundPlotter + Qt 事件循环
```

---

## 踩坑记录

### 1. `to_euler()` 单位混淆 —— 最隐蔽的 bug

**现象**：平放传感器时 AHRS 姿态角显示 Pitch=244°、Roll=103°，而不是接近 0°。尝试了十几种坐标映射组合，结果都不对。

**定位过程**：用纯合成数据（acc=[0,0,1], gyro=[0,0,0]）喂给 AHRS，手动从 quaternion 计算 Euler 角与 `to_euler()` 对比。同一个四元数 `[0.9994, 0, 0, 0]`（近乎恒等变换），手算得 Pitch≈0°，`to_euler()` 返回 ~230°。

进一步测试：给 AHRS 输入 10° 前倾的加速度数据（acc=[sin10°, 0, cos10°]），AHRS 收敛后 quaternion 的 y 分量为 -0.087。手算 Pitch = -asin(2w·y) = -10.0°。`to_euler()` 原始返回值 = **-10.0**。结论：函数返回的就是度数 -10°（而不是弧度 -0.1745 rad）。代码中错误地又调了 `np.degrees(-10.0)` = -573°。573 / 10 = 57.3 = 180/π。

**修复**：`to_euler()` 返回值直接当度数使用，删掉多余的 `np.degrees()` 调用。

**教训**：不要假设第三方库的返回值单位，用合成数据验证。

---

### 2. 坐标映射试错

传感器 X=前(北)、Y=右(东)、Z=上。imuFusion 三种内置约定都不直接匹配。

**试过但不行的组合**：

| 约定 | 映射 | 问题 |
|---|---|---|
| NED + Z取反 | `[gx, gy, -gz]` | AHRS 认为传感器倒立，Pitch 错 |
| ENU + X↔Y | `[gy, gx, gz]` | Pitch/Roll 互换（上抬变右倾） |
| NWU + 复杂 | `[-gy, gx, -gz]` | 航向反转 |
| NWU + 裸数据 | `[gx, gy, gz]` | Y=东≠西，Pitch 错 |

**最终正确**：NWU + `[gx, -gy, gz]`。且发现 mag 的 Y 不需取反（与 gyro/acc 不同）——因为磁力计中 Y 分量通过 atan2 参与航向计算，取反会反转航向方向。

**教训**：逐传感器分别验证，不要假设三者需要同样的映射。

---

### 3. 磁力计硬铁偏移

**现象**：传感器平放绕 Z 轴旋转 360°，yaw 走几十度就反向，无法完成完整旋转。

**根因**：原始 mag 数据 mx 范围 200~1700、my 范围 650~1750——全部在正象限。磁场矢量 `(mx, my)` 困在第一象限，`atan2(my, mx)` 只能扫过约 60°，永远覆盖不了 360°。硬铁偏移来自 PCB 元件、USB 连接器、焊锡等材料的残余磁化，产生约 (540, 1634) counts 的固定偏置。

**解决**：校准阶段让用户朝各个方向旋转传感器 20 秒，采集每个轴的最大最小值，硬铁偏移 = (max + min) / 2。每帧 `mag_raw − mag_offset` 得到去偏置的真实地磁场。偏移量持久化到 `calib.json`。

**教训**：任何磁力计应用的必备步骤——即使周围没有明显的磁铁也需要做。

---

### 4. 互补滤波 gain 方向与直觉相反

**现象**：gain 从 0.05 改到 0.5 后，传感器旋转响应反而变慢了。

**公式**：`姿态 = (1-gain) × 陀螺积分 + gain × 加计/磁力计估算`

- gain=0.5 → 陀螺权重 50%、等加计慢慢修正 → 反应迟钝
- gain=0.005 → 陀螺权重 99.5%、加计只修漂移 → 极跟手
- gain=0.1 → 折中，跟手且漂移小

直觉上 gain 像 PID 增益越大越快，但互补滤波恰好相反。

**教训**：调参前再看一眼公式。

---

### 5. pyvista 0.48 API 大量变更

| 旧 API | 新 API（0.48.1） |
|---|---|
| `Plotter.add_timer_callback(ms, cb)` | `BackgroundPlotter.add_callback(cb, interval=ms)` |
| `mesh.SetOrientation(9 floats)` | `actor.user_matrix = 4×4 np.array` |
| `show_grid(xlabel=, ylabel=)` | `show_grid(xtitle=, ytitle=)` |
| `Plotter.show()` | `BackgroundPlotter.app.exec()` |

此外，pyvista 0.48 在没有 Qt 后端时交互式渲染循环不工作，需额外安装 `PyQt6` + `pyvistaqt`。

---

### 6. AHRS 初始化必须在窗口打开前完成

**现象**：窗口一打开用户就开始旋转，但飞机要等好几秒才跟手。

**根因**：AHRS `flags.initialising` 需要约 200-300 帧（~3 秒）才清除。初始化期间在估计陀螺零偏、确定初始姿态，输出的 quaternion 不可靠。

**解决**：创建窗口之前先跑 `while ahrs.flags.initialising` 循环，等初始化完成后再开 3D 窗口。用户看到飞机时姿态已是正确的。

---

## 目录结构

```
E:\02_Project-TBD\IMU\deepseek\
├── main.py          主程序入口
├── diagnose.py       4 步诊断工具
├── profile.py        全链路性能测量
├── calib.json        持久化校准数据
├── requirements.txt  Python 依赖清单
├── REPORT.md         本报告
```

---

## Token 用量与费用

开发周期 2026-05-09 ~ 2026-05-12，使用 **DeepSeek V4-Pro**（通过 VSCode Claude Code 插件接入）。

### Token 统计

| 日期 | 请求数 | 输出 token | 输入 cache hit | 输入 cache miss |
|---|---|---|---|---|
| 05-09 | 152 | 92,309 | 12,532,480 | 198,332 |
| 05-11 | 255 | 137,988 | 63,094,656 | 419,506 |
| 05-12 | 176 | 101,977 | 76,484,480 | 102,525 |
| **合计** | **583** | **332,274** | **152,111,616** | **720,363** |

> 注：05-09 另有 DeepSeek V4-Flash 少量调用（1,154 输出 token，1 次请求），忽略不计。

### 费用统计

| 日期 | 费用 (CNY) |
|---|---|
| 05-09 | ¥1.46 |
| 05-11 | ¥3.66 |
| 05-12 | ¥2.83 |
| **合计** | **¥7.96** |

总 token 量 = 输出 token（332,274）+ cache hit（152,111,616）+ cache miss（720,363）= 1.53 亿。**总费用约 ¥8**。Claude Code 的 prompt caching 机制使大量上下文命中缓存（cache hit 占输入 token 的 99.5%），显著降低输入成本。
