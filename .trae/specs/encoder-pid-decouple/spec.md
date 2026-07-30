# Encoder & PID 解耦重构 - Product Requirement Document

## Overview
- **Summary**: 重构编码器和 PID 相关代码，实现**职责分离**——编码器层只负责读取增量并清零，PID 层只负责纯控制算法计算。通过消除耦合和花哨技巧，使速度闭环调参变得简单可靠、易于定位问题。
- **Purpose**: 当前系统 PID 调参困难、逻辑复杂，根本原因是编码器读取与 PID 控制深度耦合（如 `encoder_get_speed()` 内嵌极性反转、`speed_update()` 直接读累加计数器做差分、两套"读+算速度"实现并存）。重构后每层职责清晰，问题可快速定位。
- **Target Users**: 小车控制系统开发者，需要快速调参和稳定的循迹/速度控制。

## Goals
- **G1**: 编码器层实现"读取增量+清零"的极简接口，与 PID 完全解耦
- **G2**: PID 层实现最朴素的 P+I+D 算法，去掉所有花哨技巧（滤波、条件抗饱和、自动积分限幅等）
- **G3**: 速度环更新接口改为接受编码器 delta 和 dt 参数，不再依赖指针读累加值
- **G4**: 保留所有 UART 命令接口兼容性，现有 Python 脚本无需修改
- **G5**: 代码量精简，每个模块行数可控

## Non-Goals (Out of Scope)
- 不修改电机驱动层（motor.h/c）
- 不修改循迹传感器层（patrol.h/c）
- 不修改 UART 通信协议
- 不改变中断触发频率（10ms 速度环、20ms 方向环）
- 不引入任何滤波算法或预测算法
- 不修改 Python 调参脚本

## Background & Context
- 当前系统有两套"读编码器并算速度"的实现：`encoder_get_speed()` 在 encoder.c 中自行管理时间戳和极性反转；`speed_update()` 在 pid.c 中通过指针读累加计数器自行差分
- `encoder_get_speed()` 中左轮有 `*left_speed = -last_left_speed` 硬编码极性反转，右轮没有，导致方向约定混乱
- `pid_step()` 使用条件积分抗饱和（block_pos/block_neg），逻辑复杂且调参时难以直觉理解
- `speed_set_ki()` 中根据 ki 自动估算 integral_max，改变参数时行为不可预测
- `MotorSpeed_t` 存储 `volatile int32_t *enc` 指针和 `last_enc`，使 PID 与编码器耦合

## Functional Requirements

### FR-1: 编码器极简采样接口
- 提供 `int32_t encoder_sample_left(void)` — 读取左轮编码器增量（counts since last call），清零计数器
- 提供 `int32_t encoder_sample_right(void)` — 读取右轮编码器增量，清零计数器
- 提供 `void encoder_reset_all(void)` — 一次性清零左右轮计数器
- 保留 ISR 中的正交解码逻辑（硬件层不动）
- **删除 `encoder_get_speed()`** — 该函数逻辑复杂且与 PID 层重复，不再需要

### FR-2: 纯 PID 控制器
- 通用 PID 结构体只包含：`kp, ki, kd, setpoint, integral, last_error, out_min, out_max, integral_max`
- `pid_step()` 实现最朴素 P+I+D：
  ```
  error = setpoint - measure
  integral += error * dt, 限制在 [-integral_max, +integral_max]
  derivative = (error - last_error) / dt
  output = kp*error + ki*integral + kd*derivative
  output = clamp(output, out_min, out_max)
  last_error = error
  ```
- 不使用条件积分抗饱和（条件太复杂，调试困难）
- 不使用 dt<=0 的特殊处理（调用方保证）

### FR-3: 速度环解耦设计
- `MotorSpeed_t` 不再包含编码器指针和 last_enc 字段
- `speed_update()` 签名改为：`speed_update(MotorSpeed_t *s, int32_t encoder_delta, float dt_ms)`
- 速度计算直接在 `speed_update` 内完成：`speed = delta / (dt_ms / 1000)`
- `speed_set_kp/ki/kd` 只改参数并 `pid_reset()`，不再自动计算 integral_max
- integral_max 初始化时固定设为 `out_max * 0.5f`

### FR-4: 方向环保持简化
- 方向环逻辑不变（循迹误差 → PID → 差速补偿 → 写入速度环 setpoint）
- 去掉 `steer_set_ki` 中的自动 integral_max 重算

### FR-5: 主循环调用方式
- wheels.c 中速度环分支：先 `encoder_sample_left()` 取 delta，再调 `speed_update(delta, 10.0f)`
- 方向环分支恢复启用，调用 `steer_step()`
- 调试输出格式保持 `D: %5.1f, %5.1f, %d, %10d` 不变

### FR-6: UART 命令兼容
- 所有命令字符串/响应格式保持不变
- 命令处理改为调用新接口函数

## Non-Functional Requirements

### NFR-1: 代码简洁性
- encoder.c：≤ 80 行（不含 ISR）
- pid.c：≤ 200 行
- 每个函数 ≤ 30 行

### NFR-2: 可维护性
- 编码极性约定必须在 encoder.c 中明确注释：正增量 = 前进方向
- PID 极性验证必须提供调试手段（D 串中输出 delta/速度/PWM）

### NFR-3: 响应格式兼容
- 所有 UART 命令响应字符串必须与旧版一致
- `auto_tune.py` 无需任何修改即可使用

## Constraints
- **技术**: MSPM0 微控制器，MSPM0 SDK
- **引脚**: 编码器引脚已配置为正交解码中断，不修改
- **PWM**: PWM 周期 400，最大占空比 399，10kHz
- **控制周期**: 速度环 10ms，方向环 20ms，调试输出 100ms
- **编译器**: TI CCS，不使用 GCC 特性

## Assumptions
- A1: 编码器正增量对应前进方向（若实际相反，应在 encoder ISR 层面修正，见 FR-1 中说明）
- A2: 10ms 控制周期下 encoder delta 足够稳定（不会因 dt 太小导致速度计算除零）
- A3: 用户能接受简单积分限幅代替条件抗饱和（调参时只需调小 Ki 即可避免积分饱和）
- A4: 当前无编码器滤波需求（MSPM0 编码器噪声在低速下可接受）

## Acceptance Criteria

### AC-1: 编码器接口解耦
- **Given**: 系统正常运行，编码器 ISR 在更新计数器
- **When**: 调用 `encoder_sample_left()` 两次
- **Then**: 第二次调用返回的增量是两次调用之间的实际编码器脉冲数，与 PID 状态无关
- **Verification**: `programmatic`
- **Notes**: 通过读取返回值与已知移动距离对比验证

### AC-2: 编码器清零语义
- **Given**: 编码器计数器已有累积值
- **When**: 调用 `encoder_sample_left()` 和 `encoder_sample_right()`
- **Then**: 两个计数器被清零，后续读取从 0 开始累积
- **Verification**: `programmatic`
- **Notes**: 连续快速调用两次，第二次应返回接近 0

### AC-3: PID 纯算法实现
- **Given**: PID 控制器已初始化
- **When**: 以 setpoint=100, measure=50, dt=0.01 调用 `pid_step()`
- **Then**: 返回值 = kp*50 + ki*integral + kd*500，积分按规则累加并限幅
- **Verification**: `programmatic`
- **Notes**: 可手动计算验证

### AC-4: 速度环解耦
- **Given**: 速度环已初始化，setpoint=500
- **When**: 调用 `speed_update(&g_spd_left, 5, 10.0f)`（delta=5 counts, dt=10ms）
- **Then**: 内部 speed = 500 counts/s，PID 以 speed=500 反馈计算 PWM 输出
- **Verification**: `programmatic`

### AC-5: 极性正确
- **Given**: 小车静止，手动向前转动左轮
- **When**: 观察 D 串输出
- **Then**: 编码器 delta 为正值，速度为正值，PWM 为正值（前进方向）
- **Verification**: `human-judgment`
- **Notes**: 手动转动车轮观察调试串

### AC-6: UART 命令兼容
- **Given**: 系统已编译烧录
- **When**: 通过串口发送 `spid b 0.05 0 0`、`starget l 500`、`gspeed l`
- **Then**: 响应格式与旧版一致，能正确设置参数和读取速度
- **Verification**: `programmatic`

### AC-7: 无编译错误
- **Given**: 代码修改完成
- **When**: 在 CCS 中编译
- **Then**: 零 error、零 warning
- **Verification**: `programmatic`

### AC-8: 代码量达标
- **Given**: encoder.c 和 pid.c 重写完成
- **When**: 统计非空行数
- **Then**: encoder.c ≤ 80 行（不含 ISR），pid.c ≤ 200 行
- **Verification**: `human-judgment`
- **Notes**: 手动计数或使用 wc -l 工具

## Open Questions
- [ ] 编码器极性方向是否已验证？（如果前进方向是负增量，需要在 ISR 中反转 LUT 或修改极性约定）
- [ ] 是否需要保留 `encoder_read_left/right()` 的兼容性？（这些函数也会清零计数器，与 `encoder_sample_*` 功能重复）
