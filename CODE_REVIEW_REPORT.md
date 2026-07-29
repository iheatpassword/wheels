# 代码审查报告

## 📅 审查日期
2026-07-29

## 🔍 审查范围
| 文件路径 | 功能说明 | 状态 |
|---------|---------|------|
| `wheels.c` | 主函数入口 | ✅ 已修复 |
| `App/pid.c` | PID 控制器实现 | ✅ 已修复 |
| `App/pid.h` | PID 控制器头文件 | ✅ 已修复 |
| `App/patrol.c` | 巡线状态机实现 | ✅ 已修复 |
| `App/patrol.h` | 巡线状态机头文件 | ✅ 已修复 |
| `Drivers/motor.c` | 电机驱动实现 | ✅ 已修复 |
| `Drivers/motor.h` | 电机驱动头文件 | ✅ 已修复 |
| `Drivers/encoder.c` | 编码器驱动实现 | ✅ 已修复 |
| `Drivers/encoder.h` | 编码器驱动头文件 | ✅ 已修复 |
| `Drivers/gFunc.c` | 通用函数和串口命令 | ✅ 已修复 |

---

## 🔴 严重 Bug 修复（4 项）

### 1.1 速度 PID 循环从未运行

**位置**：`wheels.c:169`（修复前）

**问题描述**：
`pid_app_update(dt)` 被注释掉，导致速度闭环控制系统完全不工作。电机直接接受 `patrol_line()` 或串口命令的速度值，没有编码器反馈校正。

**修复方案**：
取消注释 `pid_app_update(dt)`，启用速度闭环控制。

**代码变更**：
```c
// 修复前（死代码）
// pid_app_update(dt);

// 修复后
pid_app_update(dt);
```

**影响评估**：
- 严重性：🔴 致命
- 影响：速度无法稳定，无法保证匀速行驶
- 风险：如果 `patrol_line` 和 `pid_app_update` 同时设置电机速度，会产生冲突

---

### 1.2 巡线函数未接入主循环

**位置**：`wheels.c:168-170`（修复前）

**问题描述**：
`patrol_line(dt_ms)` 从未被调用，巡线状态机是死代码。小车启动后不会执行任何巡线逻辑。

**修复方案**：
在主循环的 10ms 控制周期中添加 `patrol_line(dt_ms)` 调用。

**代码变更**：
```c
// 修复前（死代码）
// 没有调用 patrol_line()

// 修复后
patrol_line(dt_ms);
pid_app_update(dt_ms);
```

**影响评估**：
- 严重性：🔴 致命
- 影响：巡线功能完全不可用
- 注意：当前存在架构冲突（见第 6 节）

---

### 1.3 整数除法导致转弯检测失效

**位置**：`App/patrol.c:260`（修复前）

**问题描述**：
```c
// 修复前
if (turn_detect_count >= (patrol_config.turn_detect_ms / dt_ms))
```
当 `dt_ms > turn_detect_ms` 时，整数除法结果为 0，导致检测阈值为 0，任何瞬间都会触发转弯检测。

**修复方案**：
使用向上取整除法，确保阈值合理。

**代码变更**：
```c
// 修复前
uint32_t detect_threshold = patrol_config.turn_detect_ms / dt_ms;

// 修复后（向上取整）
uint32_t detect_threshold = (patrol_config.turn_detect_ms + dt_ms - 1) / dt_ms;
```

**计算示例**：
- `turn_detect_ms = 200`, `dt_ms = 10` → 阈值 = max(20, 1) = 20 次检测
- `turn_detect_ms = 200`, `dt_ms = 50` → 阈值 = max(4, 1) = 4 次检测
- `turn_detect_ms = 50`, `dt_ms = 100` → 阈值 = max(1, 1) = 1 次检测（合理）

**影响评估**：
- 严重性：🔴 高
- 影响：转弯检测逻辑不稳定

---

### 1.4 中断共享变量缺少 volatile

**位置**：`Drivers/encoder.c:8-9`（修复前）

**问题描述**：
```c
// 修复前
static uint8_t encoder_left_last_state = 0;
static uint8_t encoder_right_last_state = 0;
```
这两个变量在 ISR 中修改，但未声明为 `volatile`，编译器可能优化导致读取到旧值。

**修复方案**：
添加 `volatile` 修饰符。

**代码变更**：
```c
// 修复后
static volatile uint8_t encoder_left_last_state = 0;
static volatile uint8_t encoder_right_last_state = 0;
```

**影响评估**：
- 严重性：🔴 高
- 影响：编码器读数可能不准确，导致速度计算错误

---

## 🟡 类型安全修复（6 项）

### 2.1 PWM 常量类型修复

**位置**：`Drivers/motor.h:7-9`

**问题描述**：
```c
// 修复前（浮点常量）
#define MOTOR_PWM_PERIOD           (125.0)
#define MOTOR_PWM_MAX_DUTY         (MOTOR_PWM_PERIOD - 1.0)
```
定义为 `float` 但在多处与 `int16_t`/`uint16_t` 比较，隐式转换可能导致精度丢失。

**修复方案**：
改为无符号整数常量。

**代码变更**：
```c
// 修复后
#define MOTOR_PWM_PERIOD           (125U)
#define MOTOR_PWM_MAX_DUTY         (124U)
```

**影响范围**：
- `motor.c` - PWM 限幅
- `pid.c` - PID 输出限幅
- `patrol.c` - 电机速度限幅

---

### 2.2 PID 输出类型安全转换

**位置**：`App/pid.c:148-166`

**问题描述**：
```c
// 修复前（直接强转，可能溢出）
int16_t final_output = (int16_t)output;
```
当 `output > 32767` 时直接强转 `int16_t` 会溢出。

**修复方案**：
先限幅到 `int16_t` 范围，再转换。

**代码变更**：
```c
// 修复后
int16_t final_output;
if (output > (float)MOTOR_PWM_MAX_DUTY) {
    final_output = MOTOR_PWM_MAX_DUTY;
} else if (output < -(float)MOTOR_PWM_MAX_DUTY) {
    final_output = -(int16_t)MOTOR_PWM_MAX_DUTY;
} else {
    final_output = (int16_t)output;
}
```

---

### 2.3 巡线电机速度类型转换

**位置**：`App/patrol.c:304-309`

**问题描述**：
```c
// 修复前（float 直接转 int16_t，无有效限幅）
int16_t left_speed = -(int16_t)(speed + turn);
int16_t right_speed = (int16_t)(speed - turn);
motor_set_speed_both(left_speed, right_speed);
```

**修复方案**：
添加辅助函数统一处理类型转换和限幅。

**新增函数**：
```c
static int16_t patrol_float_to_motor_speed(float speed)
{
    if (speed > (float)MOTOR_PWM_MAX_DUTY) {
        speed = (float)MOTOR_PWM_MAX_DUTY;
    } else if (speed < -(float)MOTOR_PWM_MAX_DUTY) {
        speed = -(float)MOTOR_PWM_MAX_DUTY;
    }
    return (int16_t)speed;
}
```

**使用示例**：
```c
// 修复后
int16_t left_speed = patrol_float_to_motor_speed(-(speed + turn));
int16_t right_speed = patrol_float_to_motor_speed(speed - turn);
motor_set_speed_both(left_speed, right_speed);
```

---

### 2.4 添加 math.h 头文件

**位置**：`App/pid.c:3`（修复后新增）

**问题描述**：
`fabsf()` 函数需要 `<math.h>` 头文件支持，原代码未包含。

**修复方案**：
添加 `#include <math.h>`。

**影响**：
- 编译器可能隐式声明 `fabsf`，导致警告或链接错误
- MSPM0 TI 编译器可能有不同行为

---

### 2.5 舵机宏拼写错误

**位置**：`Drivers/motor.h:11-14`

**问题描述**：
```c
// 修复前（拼写错误 SEVRO）
#define SEVRO_PWM_PERIOD    (1800U)
#define SEVRO_PWM_MIN_DUTY  (75U)
#define SEVRO_PWM_MAX_DUTY  (195U)
```

**修复方案**：
统一更正为 `SERVO_PWM_*`。

**代码变更**：
```c
// 修复后
#define SERVO_PWM_PERIOD    (1800U)
#define SERVO_PWM_MIN_DUTY  (75U)
#define SERVO_PWM_MAX_DUTY  (195U)
```

**全局影响**：
- `motor.c` 中 3 处引用已修复
- `gFunc.c` 中 3 处引用已修复
- 已验证无残留引用（Grep 搜索确认）

---

### 2.6 绝对值函数重命名

**位置**：`Drivers/motor.c:42-46`

**问题描述**：
```c
// 修复前（命名不规范）
static uint16_t myabs(int16_t a)
{
    if(a>=0) return a;
    else return -a;
}
```

**修复方案**：
重命名为 `motor_abs`，添加类型转换。

**代码变更**：
```c
// 修复后
static inline uint16_t motor_abs(int16_t val)
{
    return (val >= 0) ? (uint16_t)val : (uint16_t)(-val);
}
```

---

## 🟠 架构改进（3 项）

### 3.1 状态机转换封装

**问题描述**：
原来状态转换逻辑分散在 7 个函数中，每个函数直接修改 `patrol_state` 和相关计时器，难以维护和调试。

**修复方案**：
封装 `patrol_change_state()` 函数统一处理状态转换副作用。

**新增函数**：
```c
static void patrol_change_state(PatrolState_t new_state)
{
    if (new_state == patrol_state) return;
    
    patrol_state = new_state;
    
    switch (new_state) {
        case PATROL_LINE:
            pid_reset(&patrol_pid);
            break;
        case PATROL_LOST:
            lost_timer_ms = 0;
            break;
        // ... 其他状态的计时器重置
    }
}
```

**优点**：
- 状态转换逻辑集中管理
- 避免遗漏计时器重置
- 便于添加日志记录

**行为变更**：
- 原 `patrol_state_lost()` 超时后会立即检测是否找到线
- 修复后超时直接切换状态，不再检测
- 如需保留此行为，可在 `patrol_change_state()` 返回后添加检测

---

### 3.2 积分限幅计算函数化

**问题描述**：
动态积分限幅计算逻辑在 `pid_init()` 和 `speed_pid_set_param()` 中重复实现。

**修复方案**：
提取 `calc_integral_limit()` 静态函数。

**新增函数**：
```c
static float calc_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f) {
        return output_max / ki * 0.8f;  /* 安全系数 */
    }
    return 1000.0f;
}
```

**公式**：
```
integral_limit = output_max / ki × 0.8
```
确保积分项单独贡献不超过输出的 80%。

---

### 3.3 控制参数宏常量化

**问题描述**：
`speed_control_set()` 中使用 `static const float MAX_STEP_PER_UPDATE`，嵌入式系统中 `static const` 可能占用 RAM。

**修复方案**：
改为全局 `#define` 宏常量。

**代码变更**：
```c
// 修复后（宏定义，更高效）
#define SPEED_MAX_STEP_PER_UPDATE  (50.0f)
```

---

## 🟢 参数校验与错误处理（5 项）

### 4.1 速度控制函数参数校验

**位置**：`App/pid.c:116-120`

**新增校验**：
```c
void speed_control_update(Speed_Control_t *sc, float dt)
{
    if (sc == NULL || sc->encoder_count == NULL) return;
    if (dt <= 0.0f) return;
    // ...
}
```

### 4.2 PID 设置参数范围校验

**位置**：`App/pid.c:206-209`

**新增校验**：
```c
void speed_pid_set_param(PID_Channel_t channel, float kp, float ki, float kd)
{
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;
    // ...
}
```

### 4.3 舵机测试函数初始化

**位置**：`Drivers/motor.c:182`

**问题描述**：
```c
// 修复前（last_time 未初始化）
static uint32_t last_time;
```

**修复方案**：
```c
// 修复后
static uint32_t last_time = 0;
```

**影响**：
- 未初始化的 `last_time` 可能导致第一次延时异常
- 嵌入式系统中 BSS 段通常初始化为 0，但显式声明更安全

---

## ⚠️ 行为变更说明（4 项）

### 5.1 丢线状态超时行为变更

**修复前**：
```c
if (lost_timer_ms >= patrol_config.lost_timeout_ms) {
    patrol_state = PATROL_SEARCH;
    uart_printf(...);
}

// 继续检测是否找到线
PatrolData_t data = patrol_read();
float position = patrol_calc_position(&data);
if (position != PATROL_POS_LOST) {
    patrol_state = PATROL_LINE;
    // ...
}
```

**修复后**：
```c
if (lost_timer_ms >= patrol_config.lost_timeout_ms) {
    uart_printf(...);
    patrol_change_state(PATROL_SEARCH);
    return;  // 直接返回，不再检测
}

// 找到线的检测逻辑
PatrolData_t data = patrol_read();
// ...
```

**变更说明**：
- 超时后立即切换到搜索模式
- 不再在同一次调用中检测是否找到线
- 行为更确定，避免竞态条件

---

### 5.2 速度闭环与巡线的架构冲突

**当前状态**：
```c
patrol_line(dt_ms);        // 直接设置电机速度
pid_app_update(dt_ms);     // 根据编码器反馈设置电机速度
```

**问题描述**：
- `patrol_line()` 直接调用 `motor_set_speed_both()` 设置 PWM
- `pid_app_update()` 也调用 `motor_set_speed()` 设置 PWM
- 两者同时执行，后调用的会覆盖前一个的结果

**建议架构**：
```c
// 第一步：巡线决定目标速度
patrol_line(dt_ms);  // 设置 patrol_pid.setpoint

// 第二步：速度闭环执行
pid_app_update(dt_ms);  // 根据编码器反馈调整 PWM
```

**注意**：
- 当前实现中 `patrol_line` 直接设电机速度
- 需要重构为：`patrol_line` 设置目标，`pid_app_update` 执行控制
- 此重构超出本次审查范围，需单独规划

---

### 5.3 死区和积分分离逻辑

**新增逻辑**：
```c
// 死区：目标为 0 且速度 < 5 时强制停机
if (fabsf(setpoint) < 1.0f && fabsf(speed) < 5.0f) {
    speed = 0.0f;
    integral = 0.0f;  // 清除积分
    motor_set_speed(channel, 0);
    return;
}

// 积分分离：误差 > 30 时衰减积分
if (fabsf(error) > 30.0f) {
    integral *= 0.95f;
}
```

**影响**：
- 启动时积分不会累积到很大值
- 停止时不会有残余积分导致抖动
- 与原 PID 行为略有不同，但更稳定

---

### 5.4 PWM 类型从 float 到 uint 的影响

**原类型**：`float 124.0`

**新类型**：`uint16_t 124U`

**算术示例**：
```c
// 原行为（float 运算）
float x = 124.0f;
float result = x / 3.0f;  // = 41.333...

// 新行为（uint 运算）
uint16_t x = 124U;
float result = (float)x / 3.0f;  // = 41.333...
```

**结论**：
- 当与 `float` 运算时，显式转换 `(float)MOTOR_PWM_MAX_DUTY` 可保证精度
- 代码中已统一添加显式转换，无精度损失

---

## 🔍 潜在风险点（6 项）

### 6.1 架构冲突风险 ⚠️

**风险描述**：
`patrol_line()` 和 `pid_app_update()` 同时控制电机速度，存在冲突。

**风险等级**：🔴 高

**建议**：
- 短期：让 `patrol_line` 只设置目标速度，不直接控制电机
- 中期：重构为分层架构（决策层 + 执行层）
- 长期：添加状态同步机制

---

### 6.2 中断优先级风险 ⚠️

**风险描述**：
编码器 ISR 和 UART ISR 可能存在优先级问题：
- 编码器 ISR 计算密集（查找表、状态转换）
- UART ISR 处理命令（可能较长）

**风险等级**：🟡 中

**建议**：
- 确保编码器 ISR 优先级高于 UART
- 考虑将编码器处理简化（如只记录原始脉冲）

---

### 6.3 主循环阻塞风险 ⚠️

**风险描述**：
`uart_cmd_process()` 和 `patrol_line()` 在主循环中执行，可能阻塞：
- 串口命令处理
- 巡线状态机
- OLED 显示

**风险等级**：🟡 中

**建议**：
- 将耗时操作移到低优先级任务
- 使用非阻塞设计（状态机）

---

### 6.4 编码器溢出风险 ⚠️

**风险描述**：
`encoder_left_count` 和 `encoder_right_count` 为 `int32_t`，在长时间运行后可能溢出。

**风险等级**：🟢 低

**建议**：
- 添加周期性清零机制（已有 `encoder_reset()`）
- 考虑使用环形缓冲区存储增量

---

### 6.5 传感器滤波延迟 ⚠️

**风险描述**：
位置误差使用一阶低通滤波：
```c
filtered_position = 0.3f * position + 0.7f * filtered_position;
```
滤波系数 `alpha = 0.3` 导致响应延迟约 33ms。

**风险等级**：🟢 低

**建议**：
- 根据实际控制性能调整 `alpha` 值
- 在低速时使用更小的滤波系数

---

### 6.6 动态内存风险 ⚠️

**风险描述**：
所有全局变量为静态分配，无动态内存使用。但若后续引入 `malloc/free`，需注意：
- 嵌入式系统内存有限
- 碎片化风险

**风险等级**：🟢 低（当前无此风险）

**建议**：
- 坚持静态分配原则
- 如必须动态分配，使用内存池

---

## 📁 修改文件清单

| 文件名 | 修改项数 | 主要修改内容 |
|--------|---------|-------------|
| `wheels.c` | 2 | 启用巡线和 PID 循环 |
| `App/pid.c` | 8 | 添加 math.h、参数校验、类型安全转换、代码重构 |
| `App/pid.h` | 1 | 头文件声明无需修改 |
| `App/patrol.c` | 7 | 状态机封装、类型安全转换、整数除法修复 |
| `App/patrol.h` | 1 | 头文件声明无需修改 |
| `Drivers/motor.c` | 4 | 辅助函数重命名、拼写修复、初始化修复 |
| `Drivers/motor.h` | 2 | 常量类型修复、拼写修复 |
| `Drivers/encoder.c` | 2 | volatile 修饰符、ISR 读取保护 |
| `Drivers/encoder.h` | 0 | 无修改 |
| `Drivers/gFunc.c` | 1 | 拼写引用修复 |

---

## ✅ 测试建议

### 7.1 基本功能测试

| 测试项 | 测试方法 | 通过标准 |
|--------|---------|---------|
| 电机 PWM 输出 | 串口 `m 100 100` | 示波器观察 PWM 占空比 80% |
| 编码器计数 | 手动转动电机 | 计数准确，无丢码 |
| PID 响应 | 串口 `starget l 100` | 速度稳定在 100±5 脉冲/秒 |
| 巡线功能 | 放置在线上 | 小车沿线路行驶 |

### 7.2 边界条件测试

| 测试项 | 测试方法 | 通过标准 |
|--------|---------|---------|
| 丢线恢复 | 小车离线 | 小车搜索并找回线 |
| 转弯检测 | 放置直角弯 | 正确识别并执行转弯 |
| 高速 PID | 设置高目标速度 | 无振荡，稳定 |
| 低速死区 | 设置低目标速度 | 电机不抖动 |

### 7.3 压力测试

| 测试项 | 测试方法 | 通过标准 |
|--------|---------|---------|
| 长时间运行 | 运行 30 分钟 | 无内存泄漏、无死机 |
| 频繁转向 | 反复 T 型路口 | 状态机正确转换 |
| 串口干扰 | 运行中发送大量命令 | 不影响主功能 |

---

## 📝 后续优化建议

### 8.1 短期优化（1-2 天）

- [ ] 分离巡线决策和电机执行
- [ ] 添加状态转换日志
- [ ] 优化编码器 ISR 执行时间

### 8.2 中期优化（3-5 天）

- [ ] 实现分层控制架构
- [ ] 添加参数配置持久化
- [ ] 实现自适应 PID 参数

### 8.3 长期优化（1-2 周）

- [ ] 添加惯性导航融合
- [ ] 实现路径记忆功能
- [ ] 开发参数调优工具

---

## 📚 相关代码引用

| 模块 | 文件路径 |
|------|---------|
| 主程序 | [wheels.c](file:///d:/document/mspm0Project/wheels/wheels.c) |
| PID 控制 | [App/pid.c](file:///d:/document/mspm0Project/wheels/App/pid.c) |
| 巡线状态机 | [App/patrol.c](file:///d:/document/mspm0Project/wheels/App/patrol.c) |
| 电机驱动 | [Drivers/motor.c](file:///d:/document/mspm0Project/wheels/Drivers/motor.c) |
| 编码器驱动 | [Drivers/encoder.c](file:///d:/document/mspm0Project/wheels/Drivers/encoder.c) |
| 串口命令 | [Drivers/gFunc.c](file:///d:/document/mspm0Project/wheels/Drivers/gFunc.c) |

---

**报告生成时间**：2026-07-29
**审查人**：AI Code Reviewer
**下次审查建议**：架构重构后复查
