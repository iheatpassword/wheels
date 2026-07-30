# Wheels — 基于 MSPM0G3507 的两轮差速循迹小车

> 技术交接文档 · 用于理解项目结构与代码审查参考

## 1. 项目概述

本项目是一个基于 TI MSPM0G3507 微控制器的两轮差速循迹小车，采用 **双闭环控制架构**（转向环 + 速度环），通过 4 路红外传感器实现循迹跟随，并支持 UART 串口实时调参。

**核心能力：**
- 循迹跟随：4 路红外传感器加权偏差 → 差速转向
- 速度闭环：编码器反馈 → PID → PWM 稳速
- 实时调参：UART 命令在线调整 PID 参数、目标速度、传感器状态查看

**硬件清单：**

| 模块 | 型号/规格 | 用途 |
|------|-----------|------|
| MCU | MSPM0G3507 (TI, Cortex-M0+) | 主控 |
| 电机驱动 | TB6612FNG | 双路 H 桥驱动左右电机 |
| 编码器 | 双路正交编码器 | 速度反馈（中断解码） |
| IMU | MPU6050 + DMP | 姿态解算（已初始化，转向环暂未使用） |
| 循迹传感器 | 4 路红外反射 | 循迹偏差检测 |
| 显示 | OLED 0.96" (硬件 I2C) | 状态显示 |
| 通信 | UART0 | 调试命令收发 |
| 其他 | LED / 按键 / 蜂鸣器 | 指示与交互 |

**关键参数：**
- PWM 频率：10kHz，周期 400，最大占空比 399（见 [motor.h:9-10](file:///d:/document/mspm0Project/wheels/Drivers/motor.h#L9-L10)）
- 控制周期：10ms（TIMER_0 中断触发）
- 速度单位：脉冲/秒（counts/s）

---

## 2. 目录结构

```
wheels/
├── wheels.c                  # 主程序入口（main + 主循环）
├── wheels.syscfg             # SysConfig 硬件引脚配置
├── README.md                 # 本文档
├── CODE_REVIEW_REPORT.md     # 代码审查报告（⚠️ 部分内容已过时）
│
├── App/                      # 应用层
│   ├── pid.c / pid.h         # PID 控制器：速度环 + 转向环
│   └── patrol.c / patrol.h   # 循迹：传感器读取 + 加权偏差计算
│
├── Drivers/                  # 驱动层
│   ├── motor.c / motor.h     # 电机驱动（TB6612，PWM 调速 + 方向控制）
│   ├── encoder.c / encoder.h # 编码器驱动（中断正交解码 + 速度计算）
│   ├── gFunc.c / gFunc.h     # 通用函数：UART 命令解析 + TIMER_0 中断 + millis()
│   ├── uart.c / uart.h       # UART 收发驱动
│   ├── key.c / key.h         # 按键驱动（含 LED 模式）
│   ├── MSPM0/                # MCU 底层：时钟配置 + 中断初始化
│   ├── MPU6050/              # MPU6050 + DMP 姿态解算（I2C）
│   └── OLED_Hardware_I2C/    # OLED 显示驱动
│
└── Debug/
    ├── ti_msp_dl_config.c    # SysConfig 自动生成（勿手动编辑）
    └── ti_msp_dl_config.h    # 引脚/外设宏定义
```

---

## 3. 软件架构

### 3.1 控制流

主循环以 10ms 为控制周期（TIMER_0 中断置位 `read_patrol` 标志，主循环检测并清除）：

```
┌─────────────────────────────────────────────────────────┐
│  TIMER_0 中断（每 10ms）                                 │
│    └─ read_patrol = 1                                    │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│  主循环 if (read_patrol)                                 │
│                                                          │
│  ① patrol_get_error()  → position_error + status        │
│     └─ 4 路传感器加权偏差 [-3, +3]                       │
│                                                          │
│  ② if (PATROL_OK):                                      │
│       steer_pid_update(error)   ← 转向环                 │
│         └─ PID → turn_output → 差速合成                  │
│            target_left  = base + turn                    │
│            target_right = base - turn                    │
│     else (LOST / JUNCTION):                             │
│       停车（目标速度置 0）                                │
│                                                          │
│  ③ pid_app_update(10)            ← 速度环                │
│     ├─ speed_control_update(left)                        │
│     │   └─ 编码器反馈 → PID → motor_set_speed(PWM)       │
│     └─ speed_control_update(right)                       │
│                                                          │
│  ④ 每 100ms 调试输出（速度 / 传感器 / 偏差 / turn）      │
└─────────────────────────────────────────────────────────┘
```

入口代码见 [wheels.c:138-178](file:///d:/document/mspm0Project/wheels/wheels.c#L138-L178)。

### 3.2 双闭环说明

#### 转向环（循迹偏差控制）

- **输入**：4 路红外传感器的加权偏差（见 [patrol.c](file:///d:/document/mspm0Project/wheels/App/patrol.c)）
- **权重**（从左到右物理顺序 `r2 r1 l1 l2`）：

| 传感器 | 位置 | 权重 |
|--------|------|------|
| r2 | 左外侧 | -3 |
| r1 | 左内侧 | -1 |
| l1 | 右内侧 | +1 |
| l2 | 右外侧 | +3 |

- **公式**：`偏差 = Σ(传感器值 × 权重) / 触发传感器数`，范围 [-3, +3]
- **方向约定**：偏差正值 = 车偏左（线在右侧）→ 需右转；负值 = 车偏右 → 需左转
- **PID 输出**：`turn_output`（差速补偿，脉冲/秒），限幅 ±2000
- **差速合成**：`target_left = base + turn`，`target_right = base - turn`
  - turn 正 → 左轮快、右轮慢 → 右转 ✓
- 实现：[pid.c:398-457](file:///d:/document/mspm0Project/wheels/App/pid.c#L398-L457)

#### 速度环（编码器反馈控制）

- **反馈**：编码器累计计数的增量，经 `polarity` 极性对齐后计算速度
- **极性约定**（解决编码器方向与电机方向不一致）：

| 车轮 | motor_set_speed 向前 | encoder 增量方向 | polarity | 效果 |
|------|----------------------|------------------|----------|------|
| 左轮 | 负值 | 负 | **-1.0** | speed = delta × (-1) = 正 |
| 右轮 | 正值 | 正 | **+1.0** | speed = delta × (+1) = 正 |

- **结论**：左右轮向前时 `setpoint` 均为正值，PID 正常负反馈
- 实现：[pid.c:87-90](file:///d:/document/mspm0Project/wheels/App/pid.c#L87-L90)（速度计算）、[pid.c:283-285](file:///d:/document/mspm0Project/wheels/App/pid.c#L283-L285)（极性配置）

### 3.3 边界处理

| 情况 | 判定 | 行为 |
|------|------|------|
| 丢线 | 4 路全白（sum=0） | `PATROL_LOST` → 停车 |
| 路口/十字 | 4 路全黑（sum=4） | `PATROL_JUNCTION` → 停车 |
| 正常 | 1~3 路触发 | `PATROL_OK` → 计算偏差并控制 |

见 [patrol.c:57-83](file:///d:/document/mspm0Project/wheels/App/patrol.c#L57-L83)。

---

## 4. 模块说明

### 4.1 motor — 电机驱动

TB6612 双路 H 桥驱动，通过 IN1/IN2 控制方向、PWM 控制速度。

| 函数 | 说明 |
|------|------|
| `motor_set_speed(channel, speed)` | 设置单轮速度（-399~399，负=反转） |
| `motor_set_speed_both(left, right)` | 同时设置双轮 |
| `motor_stop_both(mode)` | 停车（COAST 滑行 / BRAKE 制动） |

文件：[motor.h](file:///d:/document/mspm0Project/wheels/Drivers/motor.h)、[motor.c](file:///d:/document/mspm0Project/wheels/Drivers/motor.c)

### 4.2 encoder — 编码器驱动

中断方式实现正交解码，维护累计计数 `encoder_left_count` / `encoder_right_count`（`volatile int32_t`）。

| 函数 | 说明 |
|------|------|
| `encoder_read_left/right()` | 读取并清零累计计数（注意：会清零） |
| `encoder_get_speed(&l, &r)` | 计算速度（counts/s，调试用） |
| `encoder_reset()` | 清零计数 |

> ⚠️ 速度环 PID 内部直接读取累计计数（不清零），与 `encoder_get_speed` 独立。

文件：[encoder.h](file:///d:/document/mspm0Project/wheels/Drivers/encoder.h)、[encoder.c](file:///d:/document/mspm0Project/wheels/Drivers/encoder.c)

### 4.3 patrol — 循迹模块

精简设计：仅负责传感器读取与加权偏差计算，不直接操控电机。

| 类型/函数 | 说明 |
|-----------|------|
| `PatrolData_t` | 4 路传感器位域（r2/r1/l1/l2） |
| `PatrolStatus_t` | 状态枚举（OK / LOST / JUNCTION） |
| `patrol_read()` | 读取 4 路传感器 |
| `patrol_get_error(&err)` | 计算加权偏差，返回状态 |
| `patrol_get_raw(...)` | 获取原始传感器值（调试用） |

文件：[patrol.h](file:///d:/document/mspm0Project/wheels/App/patrol.h)、[patrol.c](file:///d:/document/mspm0Project/wheels/App/patrol.c)

### 4.4 pid — PID 控制器

包含基础 PID、速度环、转向环三部分。

| 结构体 | 说明 |
|--------|------|
| `PID_Controller_t` | 基础 PID（kp/ki/kd/integral/限幅） |
| `Speed_Control_t` | 速度环（含 encoder 指针、polarity 极性） |
| `Steer_Control_t` | 转向环（含 base_speed、turn_output、max_turn） |

| 函数 | 说明 |
|------|------|
| `pid_app_init()` | 初始化速度环 + 转向环（默认参数） |
| `pid_app_update(dt_ms)` | 速度环执行（驱动 PWM） |
| `steer_pid_update(error, dt_ms)` | 转向环执行（计算差速 + 设置目标） |
| `speed_control_set(sc, target)` | 设置单轮目标速度 |
| `speed_pid_get_raw_speed(&l, &r)` | 读取 PID 内部速度（调参用） |

文件：[pid.h](file:///d:/document/mspm0Project/wheels/App/pid.h)、[pid.c](file:///d:/document/mspm0Project/wheels/App/pid.c)

### 4.5 gFunc — 通用函数与命令解析

| 内容 | 说明 |
|------|------|
| `uart_cmd_process()` | 主循环中调用，解析并执行串口命令 |
| `read_patrol` | TIMER_0 中断置位标志（volatile） |
| `millis()` | 毫秒级时间戳 |

文件：[gFunc.h](file:///d:/document/mspm0Project/wheels/Drivers/gFunc.h)、[gFunc.c](file:///d:/document/mspm0Project/wheels/Drivers/gFunc.c)

### 4.6 其他模块

| 模块 | 说明 |
|------|------|
| uart | UART0 收发，含 `uart_printf` 格式化输出 |
| key | 按键消抖 + LED 模式控制 |
| clock / interrupt | MSPM0 时钟配置与中断初始化 |
| MPU6050 | MPU6050 + DMP 姿态解算（已初始化，转向环暂未使用 yaw） |
| OLED | 0.96" OLED 硬件 I2C 显示 |

---

## 5. UART 调试命令参考

通过 UART0 发送命令（`\r` 或 `\r\n` 结尾），波特率见 SysConfig 配置。

### 电机控制

| 命令 | 说明 | 示例 |
|------|------|------|
| `m <left> <right>` | 设置电机速度（-399~399） | `m -200 200` |
| `mstop` | 停止电机 | `mstop` |

### 速度环 PID

| 命令 | 说明 | 示例 |
|------|------|------|
| `spid <ch> <kp> <ki> <kd>` | 设置全部参数 | `spid b 0.5 0.1 0.05` |
| `skp <ch> <kp>` | 单独设 kp | `skp l 0.5` |
| `ski <ch> <ki>` | 单独设 ki | `ski r 0.1` |
| `skd <ch> <kd>` | 单独设 kd | `skd b 0.05` |
| `gpid [ch]` | 查询参数 | `gpid l` |
| `starget <ch> <speed>` | 设置目标速度（counts/s） | `starget b 2000` |
| `gspeed [ch]` | 查询当前速度 | `gspeed` |

### 转向环 PID（循迹偏差环）

| 命令 | 说明 | 示例 |
|------|------|------|
| `stpid <kp> <ki> <kd>` | 设置全部参数 | `stpid 400 0 0` |
| `stkp <kp>` | 单独设 kp | `stkp 500` |
| `stki <ki>` | 单独设 ki | `stki 50` |
| `stkd <kd>` | 单独设 kd | `stkd 30` |
| `gtpid` | 查询参数 + base_speed | `gtpid` |
| `gpatrol` | 查看传感器状态 + 偏差 | `gpatrol` |
| `sbase <cnt/s>` | 设置基础前进速度（0=停） | `sbase 2000` |
| `sstop` | 停止转向 + 速度环 | `sstop` |

> `ch`：`l`=左轮，`r`=右轮，`b`=两轮（默认）

### 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 速度环 kp | 0.346 | 每单位速度误差的 PWM 输出 |
| 速度环 ki / kd | 0 / 0 | 待调 |
| 转向环 kp | 400 | 每单位偏差的差速输出（脉冲/秒） |
| 转向环 ki / kd | 0 / 0 | 待调 |
| 转向环 max_turn | 2000 | 最大差速限幅 |

---

## 6. 编译与烧录

### 工具链
- **IDE**：Code Composer Studio (CCS)
- **配置工具**：SysConfig（生成 `Debug/ti_msp_dl_config.c/h`）
- **SDK**：TI MSPM0 SDK
- **目标芯片**：MSPM0G3507

### 烧录步骤
1. 用 CCS 打开项目目录
2. 确认 `wheels.syscfg` 中引脚配置与硬件一致
3. 编译生成 `.out` 文件
4. 通过调试器（XDS110）下载到芯片

> ⚠️ CCS 删除项目时默认会"同时删除磁盘文件"且不可恢复，操作前务必备份。

---

## 7. 关键设计决策（代码审查参考）

### 7.1 polarity 极性字段

**问题**：左轮电机正转时编码器反向计数，导致 PID 正反馈（速度失控满转）。

**方案**：在 `Speed_Control_t` 中增加 `polarity` 字段，速度计算时乘以极性系数：
```c
sc->speed = (float)delta_count / dt * sc->polarity;
```
- 左轮 `polarity = -1.0`，右轮 `polarity = +1.0`
- 效果：左右轮向前时 `setpoint` 与 `speed` 均为正值，PID 正常负反馈

见 [pid.c:87-90](file:///d:/document/mspm0Project/wheels/App/pid.c#L87-L90)、[pid.c:283-285](file:///d:/document/mspm0Project/wheels/App/pid.c#L283-L285)。

### 7.2 加权偏差归一化

偏差计算除以"触发传感器数"，避免多传感器同时触发时偏差过大：
```c
偏差 = Σ(传感器 × 权重) / 触发数
```
例如 r1+l1 同时触发 → `(-1+1)/2 = 0`（居中）。

见 [patrol.c:73-83](file:///d:/document/mspm0Project/wheels/App/patrol.c#L73-L83)。

### 7.3 转向环与速度环调用顺序

转向环**先**设置左右轮目标速度（`speed_control_set`），速度环**后**执行 PWM 驱动（`pid_app_update`）。两者不冲突：转向环只写 setpoint，速度环读 setpoint 并驱动 PWM。

见 [wheels.c:148-162](file:///d:/document/mspm0Project/wheels/wheels.c#L148-L162)。

### 7.4 速度单位

全程使用**脉冲/秒（counts/s）**作为速度单位，不依赖编码器线数与轮径等物理参数，调参时关注相对值即可。

### 7.5 调试输出与 PID 反馈一致性

调试输出通过 `speed_pid_get_raw_speed()` 直接读取 PID 内部 `sc->speed`，确保观察值与 PID 实际使用的反馈值一致，调参才有意义。

见 [pid.c:365-370](file:///d:/document/mspm0Project/wheels/App/pid.c#L365-L370)。

---

## 8. 已知限制与后续方向

| 项 | 说明 |
|----|------|
| MPU6050 未接入转向环 | 已初始化 DMP，但转向环改用循迹偏差驱动，yaw 暂未使用 |
| patrol_test 未调用 | [wheels.c](file:///d:/document/mspm0Project/wheels/wheels.c) 中 `patrol_test` 函数保留但未启用 |
| 速度限幅保守 | 差速合成上限 8000 counts/s，需根据实测满转速度调整 |
| CODE_REVIEW_REPORT.md 过时 | 描述的是旧 patrol 状态机架构，现已精简为加权偏差模式，以本 README 为准 |
| 传感器极性假设 | 假设 1=检测到黑线，若硬件相反需在 `patrol_read` 中取反 |
