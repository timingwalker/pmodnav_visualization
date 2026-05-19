# IMU 3D 姿态可视化 — 完整开发报告

---

## 项目概述

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

## 调试记录

### 1. 坐标映射试错

传感器 X=前(北)、Y=右(东)、Z=上。imuFusion 三种内置约定都不直接匹配。

**逐次调试记录**（共 10 次代码改动）：

| # | 代码改动 | 预期 | 实际结果 |
|---|---|---|---|
| 1 | 不改映射，原始数据 `[gx,gy,gz]` 喂 NED | 平放时 Pitch/Roll≈0° | Pitch 完全不对，看天看地乱跳 |
| 2 | NED 约定，Z 取反 `[gx, gy, -gz]` | 修正 Z 轴方向 | AHRS 认为传感器倒立，Pitch 偏移 180° |
| 3 | 换 ENU 约定，数据不变 `[gx, gy, gz]` | ENU 的 X=东/Y=北 可能匹配 | 前倾识别错误，姿态角不匹配 |
| 4 | ENU 约定，X↔Y 交换 `[gy, gx, gz]` | 传感器前倾→ENU 的北倾 | Pitch/Roll 互换，上抬变右倾 |
| 5 | 换 NWU 约定，裸数据 `[gx, gy, gz]` | NWU 的 X=北 匹配传感器 | Y 是东不是西，Pitch 方向有偏差 |
| 6 | NWU，gyro/acc/mag 全部 Y 取反 `[gx, -gy, gz]` | Y 从东翻到西，姿态正确 | Pitch/Roll 基本对，Yaw 旋转方向反了 |
| 7 | NWU，复杂映射 `[-gy, gx, -gz]`（X/Y 交换+取反） | 重新对齐轴 | 航向反转，Yaw 走不到 360° |
| 8 | NWU，`[gx, -gy, gz]`，mag 也取反 `[mx, -my, mz]` | mag 跟 gyro/acc 一致 | Yaw 方向更乱，atan2 对 Y 取反敏感 |
| 9 | NWU，`[gx, -gy, gz]`，mag 用 `[-my, mx, mz]` | 尝试调换 mag 的 X/Y | Yaw 偏移固定角度，旋转时跳跃 |
| 10 | NWU，`[gx, -gy, gz]`，mag `[mx, my, mz]` 不取反 | gyro/acc Y 取反但 mag 保持原值 | **全部正确**，Pitch/Roll/Yaw 正确，Yaw 走完 360° 不反向 |

**最终映射**：

```python
gyro: [ gx, -gy,  gz]    # Y 取反（东 → 西）
acc:  [ ax, -ay,  az]    # Y 取反
mag:  [ mx,  my,  mz]    # 不取反（atan2 航向计算）
```

**教训**：逐传感器分别验证，不要假设三者需要同样的映射。mag 的 Y 分量通过 atan2 参与航向计算，取反会直接反转旋转方向。

---

### 2. `to_euler()` 单位混淆 —— 最隐蔽的 bug

**现象**：传感器平放静止，Pitch 和 Roll 显示为 244°、103° 等大数值，而不是接近 0°。注意此时坐标映射错误同时存在（见上节），两个问题叠加让现象更混乱。

**逐次调试记录**（共 5 次代码改动）：

| # | 代码改动 | 预期 | 实际结果 |
|---|---|---|---|
| 1 | 直接打印 `to_euler()` 返回值作为姿态角，传感器平放静止 | Pitch/Roll 接近 0° | Pitch=244°, Roll=103°，完全不对 |
| 2 | 以为坐标映射问题，尝试 NWU + Y取反（见上节） | 修正后角度应归零 | 依然错误，说明不只是映射的问题 |
| 3 | 传感器平放不动，直接打印 `ahrs.quaternion.wxyz` | 接近 `[1, 0, 0, 0]`（恒等四元数） | `[0.9994, 0, 0, 0]`，四元数本身看起来正常 |
| 4 | 手算四元数→欧拉角，与 `to_euler()` 对比 | 两者结果一致 | 手算 Pitch≈0°，`to_euler()` 返回 ~230°，差异巨大 |
| 5 | 用合成数据（acc=[0,0,1], gyro=[0,0,0]）直接喂 AHRS，消除硬件影响 | `to_euler()` 返回接近 0 的弧度值 | 返回 ~230，发现已经是度数，证明 `to_euler()` 返回的是**度数**而非弧度 |

**进一步验证**：构造 10° 前倾（acc=[sin10°, 0, cos10°]），AHRS 收敛后 quaternion y = -0.087。手算 Pitch = -asin(2w·y) = -10.0°。`to_euler()` 原始返回值 = **-10.0**。代码中又调了 `np.degrees(-10.0)` = -573°。573 / 10 = 57.3 = 180/π，确认多乘了一轮。

**修复**：`to_euler()` 返回值直接当度数使用，删掉多余的 `np.degrees()` 调用。

**教训**：不要假设第三方库的返回值单位，用合成数据验证。

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
```
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

---

## 画面卡顿优化（2026-05-14 ~ 2026-05-19）

### 问题演进

用户反馈飞机 3D 渲染不够流畅实时。优化过程经历了五个阶段：

### 阶段一：串口数据流优化

| 尝试 | 改动 | 效果 |
|---|---|---|
| 串口非阻塞读 | `timeout=0` + `ser.in_waiting` 批量读 | serial max interval 65→44ms |
| GIL 让出 | `time.sleep(0)` 在渲染回调末尾 | 轻微改善 |
| 线程优先级提升 | 串口线程设为 `ABOVE_NORMAL` | 微观改善 |
| 关闭 GC | `gc.disable()` | 无明显差异 |
| FTDI 延迟计时器 | 设备管理器 16ms→1ms | **消除 16ms 周期打包** |
| 固件 usleep | 8000→1000 | 数据率 87→449Hz |
| 固件波特率 | 115200→460800 | 2.1x 吞吐 |
| ODR 提升 | 119→476Hz | 零重复读数 |

**结果**：数据率从 87Hz 飙到 449Hz，串口抖动大幅下降，但画面流畅度无质变。

### 阶段二：AHRS 参数调优

| 尝试 | 改动 | 效果 |
|---|---|---|
| gain 全范围调试 | 0.1→0.9 | 影响收敛速度，不影响画面流畅 |
| 去掉 `np.allclose` 守卫 | 每次强制更新 `user_matrix` | 微动细节回来，大画面仍卡 |
| `ACC_REJECTION=5.0` | 抖动时加计被拒收 | 跟随性改善但画面仍卡 |
| 加计量程 ±2→±4g | 消除抖动截顶 | 数据完整度提升 |
| 陀螺 Z 轴死区 | `--recal` 自动计算 5σ 阈值 | 下倾 Yaw 漂移减少 |

**结果**：姿态解算更准确，但画面流畅度仍无质变。

### 阶段三：回放模式隔离

| 尝试 | 改动 | 效果 |
|---|---|---|
| 录制+回放 | `record.py` + `--playback` | 排除串口 I/O，画面仍卡 |
| 合成完美数据 | 纯正弦俯仰，零交叉轴 | **仍然卡** → 确认问题在渲染侧 |

**结论**：传感器硬件、AHRS 参数、串口 I/O 都不是元凶。问题在渲染/显示管线。

### 阶段四：渲染侧各种尝试

| 尝试 | 改动 | 效果 |
|---|---|---|
| 关 VSync | `SetSwapControl(0)` | 画面撕裂，500fps 但肉眼更差 |
| 开 VSync + QTimer 2/16/1ms | 各种 callback interval | 均无改善 |
| SLERP | 四元数球面插值 | 实测无效，删除 |
| `--sync-60hz` | 60Hz 批量更新 AHRS | 无改善 |
| 合成数据回放 | 完美零噪声数据喂入 | 仍然卡 |

### 阶段五：定位根因——时钟对抗

在 HUD 加入 `jit`（帧间隔标准差）指标后定位到根因：

- QTimer 驱动 `render_update()` 更新矩阵
- VTK 内部在 VSync 节奏下独立渲染
- 两套时钟各自振荡：矩阵更新在 3ms 前或 15ms 前，VTK 抓到的用户矩阵年龄不一
- 帧间隔标准差高达 60ms —— 某些帧显示 5ms，某些 30ms，视觉上就是"卡顿"

**最终方案**：

```python
# 矩阵更新挂在 VTK 渲染循环上，和 VSync 同步
plotter.render_window.AddObserver('StartEvent', lambda *_: render_update())
# QTimer 只负责告诉 VTK "可能有东西要画，检查一下"
plotter.add_callback(lambda: plotter.render(), interval=1)
```

`render_update()` 不再跑在 QTimer 里，而是在 VTK 每次真正渲染前一刻（`StartEvent`）执行。矩阵更新和显示器 VSync 完全同步。帧间隔标准差从 60ms 降到 <2ms，画面流畅。

### 最终参数配置

| 参数 | 值 | 说明 |
|---|---|---|
| GAIN | 0.8 | 20% 陀螺 + 80% 加计/磁力计 |
| ACC_REJECTION | 5.0 | 高容忍度，抖动时加计不被误导 |
| MAG_REJECTION | 10.0 | 磁力计几乎永不被拒 |
| 加计量程 | ±4g | 避免抖动截顶 |
| 陀螺 ODR | 476Hz | 接近固件 loop 上限 |
| 波特率 | 460800 | 充分利用 USB 带宽 |
| 渲染回调 | VTK StartEvent | VSync 同步，jit<2ms |

### 新增功能

- `--gain` / `--acc-rej` / `--mag-rej` CLI 参数调参
- `--sensors` 7 种传感器组合测试（gam/ga/gm/am/g/a/m）
- `--playback` 文件回放模式
- `--recal` 四步引导式磁力计校准 + 陀螺 Z 轴死区自动计算
- 加计量程 ±2→±4g
- FTDI 延迟计时器自动设置（D2XX API，后发现对 VCP 无效，已移除）
- `diagnose.py`：gain=0.5 匹配主程序，dt 实测化
