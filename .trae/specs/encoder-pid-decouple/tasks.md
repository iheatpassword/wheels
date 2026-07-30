# Encoder & PID 解耦重构 - Implementation Plan

## [x] Task 1: 重构 encoder.h/c — 极简采样接口
- **Priority**: high
- **Depends On**: None
- **Description**: 
  - 重写 `encoder.h`：删除 `encoder_get_speed()` 声明，新增 `encoder_sample_left()`、`encoder_sample_right()`、`encoder_reset_all()` 三个接口
  - 重写 `encoder.c`：实现三个新函数，每个函数只读计数器→保存临时值→清零→返回；删除 `encoder_get_speed()` 及其中复杂逻辑（时间戳管理、极性反转、滤波）
  - 保留 ISR 正交解码逻辑不动（硬件层无需修改）
  - 保留 `encoder_init()` 和 `encoder_reset()` 用于初始化
- **Acceptance Criteria Addressed**: AC-1, AC-2
- **Test Requirements**:
  - `programmatic` TR-1.1: `encoder_sample_left()` 返回值等于自上次调用以来的增量，计数器归零
  - `programmatic` TR-1.2: 连续两次调用 `encoder_sample_left()`，第二次返回 0（假设没有新脉冲）
  - `programmatic` TR-1.3: `encoder_sample_right()` 行为与左轮对称
  - `human-judgement` TR-1.4: 代码行数 ≤ 80 行（不含 ISR），逻辑清晰无冗余

## [x] Task 2: 重写 pid.h/c — 纯 PID 控制器
- **Priority**: high
- **Depends On**: Task 1
- **Description**:
  - 重写 `PID_t` 结构体：保留 `kp, ki, kd, setpoint, integral, last_error, out_min, out_max, integral_max`
  - 重写 `MotorSpeed_t`：删除 `enc` 指针和 `last_enc` 字段，只保留 `pid, ch, speed`
  - `MotorSpeed_t` 新增 `last_delta` 字段用于 D 项计算（delta 是 count，需除以 dt 得速度）
  - 重写 `pid_step()`：最朴素 P+I+D，积分直接累加+限幅，不使用条件抗饱和
  - 重写 `speed_update()`：签名改为 `speed_update(MotorSpeed_t *s, int32_t delta, float dt_ms)`，内部计算速度=delta/(dt_ms/1000)，然后 PID
  - `speed_set_kp/ki/kd` 改为直接赋值+pid_reset()，不再自动算 integral_max
  - `pid_app_init()` 中初始化 integral_max 为 out_max * 0.5f
  - 删除 `encoder.h` 的 #include，改为只依赖 `motor.h`
- **Acceptance Criteria Addressed**: AC-3, AC-4
- **Test Requirements**:
  - `programmatic` TR-2.1: `pid_step()` 在已知输入下产生正确输出（手动计算对比）
  - `programmatic` TR-2.2: `speed_update(&s, 10, 10.0f)` 内部 speed = 1000 counts/s（10 counts / 0.01s）
  - `programmatic` TR-2.3: integral 值限制在 [-integral_max, +integral_max]
  - `human-judgement` TR-2.4: 代码行数 ≤ 200 行，每个函数 ≤ 30 行

## [x] Task 3: 修改 wheels.c — 新主循环调用
- **Priority**: high
- **Depends On**: Task 1, Task 2
- **Description**:
  - 恢复 `steer_flag` 分支（当前被注释掉）
  - 速度环分支改为：
    ```
    int32_t dl = encoder_sample_left();
    int32_t dr = encoder_sample_right();
    speed_update(&g_spd_left,  dl,  10.0f);
    speed_update(&g_spd_right, dr, 10.0f);
    ```
  - 调试输出改为在 wheels.c 中直接算 PID 输出值，不再依赖 `g_spd_left.pid.output`（新 PID_t 不存 output 字段）
  - D 串格式保持 `D: %5.1f, %5.1f, %d, %10d`，字段含义不变
- **Acceptance Criteria Addressed**: AC-4, AC-6
- **Test Requirements**:
  - `programmatic` TR-3.1: 速度环分支编译无错误
  - `programmatic` TR-3.2: steer_flag 分支编译无错误
  - `human-judgement` TR-3.3: D 串调试输出格式与旧版一致

## [x] Task 4: 修改 gFunc.c — UART 命令适配
- **Priority**: high
- **Depends On**: Task 2
- **Description**:
  - 所有 PID 命令处理函数（spid, skp, ski, skd, gpid, starget, gspeed）改为调用新接口
  - 方向环命令（stpid, stkp, stki, stkd, gtpid, sbase, sstop）改为调用新接口
  - 命令字符串解析保持不变
  - 所有响应字符串格式保持不变
  - `debug_speed_off` 中调用 `pid_reset(&g_spd_left.pid)` 和 `pid_reset(&g_spd_right.pid)`
  - 删除 `encoder.h` 的 #include（如果存在）
- **Acceptance Criteria Addressed**: AC-6
- **Test Requirements**:
  - `programmatic` TR-4.1: 所有 UART 命令处理函数编译无错误
  - `human-judgement` TR-4.2: 所有响应字符串与旧版一致

## [x] Task 5: 静态验证与编译检查
- **Priority**: medium
- **Depends On**: Task 1, Task 2, Task 3, Task 4
- **Description**:
  - 使用 GetDiagnostics 检查所有修改文件的语法错误
  - 检查是否有残留的旧类型/旧函数引用
  - 验证文件间依赖关系正确
  - 检查 wheels.c 中不再直接访问 encoder_count 累计值（除调试串中的读取）
- **Acceptance Criteria Addressed**: AC-7
- **Test Requirements**:
  - `programmatic` TR-5.1: 无残留的 `encoder_get_speed` 引用
  - `programmatic` TR-5.2: 无残留的旧类型/旧函数引用（Speed_Control_t, Steer_Control_t, speed_control_*, steer_pid_*, PID_Controller_t 等）
  - `programmatic` TR-5.3: gFunc.c 不再 #include "encoder.h"
  - `human-judgement` TR-5.4: 代码整体可读性良好
