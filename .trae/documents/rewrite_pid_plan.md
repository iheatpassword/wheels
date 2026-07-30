# PID 重写计划：以最稳定可行的方式重写速度环和方向环

## 1. 现状分析

### 当前代码主要问题
1. **结构重复冗余**：`Speed_Control_t` 内嵌 `PID_Controller_t`，又有 `Steer_Control_t` 另包一层，加上 `pid_init / speed_control_init / steer_pid_init` 三级初始化，调用链过长。
2. **多余逻辑**：
   - `first_update` 标志（启动时跳过积分）—— 实际上启动时积分本来就是 0，真正要防的是启动瞬间的巨大误差导致的饱和，应使用 **抗饱和 (clamping)** 而非额外标志。
   - `speed_alpha` 低通滤波—— 编码器噪声本就不大，用滤波会引入相位滞后，反而使稳定性变差。
   - `raw_speed` 与 `speed` 双字段—— 冗余。
   - 超速保护 `speed_abs > target_abs * 2.5 + 500`—— 这是为修复极性问题打的补丁，极性正确后应删除。
   - `polarity` 字段—— 把方向反转放到结构体里，使问题更难排查。应由 `encoder` 增量的正负号约定来统一。
   - `calc_integral_limit / calc_steer_integral_limit` 两套重复函数。
3. **API 层次过深**：`uart_cmd_spid → speed_pid_set_param → 直接改 kp/ki/kd + pid_reset`，用户调一个参数要走 3 层。
4. **方向环 `steer_pid_update`** 里自己算 `base ± turn`，这意味着方向环同时承担了 "循迹纠偏" 和 "速度分配" 两件事，耦合度高。

---

## 2. 设计原则

- **单一职责**：PID 就是 PID，只关心 `error → output`。
- **最少字段**：每个结构体只保留必要字段。
- **去掉滤波**：相信传感器，用 PID 的 D 项/输出限幅保证稳定。
- **统一极性**：所有方向约定在一处明确（正向前进 = 正 encoder delta = 正 PWM）。
- **保留 UART 命令兼容性**：`spid / stpid / starget / gspeed / sbase / sstop` 等常用命令保持接口不变。
- **去掉启动补丁**：不使用 `first_update`，改使用 **条件积分 (conditional integration)** + **输出饱和抗积分饱和**。

---

## 3. 新结构体设计

### 3.1 通用 `PID_t`（所有环共用）
```c
typedef struct {
    float kp, ki, kd;
    float integral;
    float last_error;
    float out_min, out_max;    /* 输出硬限幅 */
    float integral_max;        /* 积分最大允许值（软限幅） */
} PID_t;
```

**更新规则（稳定版 PID）**：
```
error    = setpoint - measure
integral += error * dt
如果 output 饱和且 error 继续推向饱和侧 → 停止累积（抗饱和）
derivative = (error - last_error) / dt
output   = kp*error + ki*integral + kd*derivative
output   = clamp(output, out_min, out_max)
last_error = error
```

### 3.2 速度环：`MotorSpeed_t`
```c
typedef struct {
    PID_t pid;
    Motor_Channel_t ch;
    volatile int32_t *enc;  /* encoder count 累计值 */
    int32_t last_enc;
    float speed;            /* 当前实测速度 (counts/s) */
} MotorSpeed_t;
```

### 3.3 方向环：`Steer_t`
```c
typedef struct {
    PID_t pid;
    float base_speed;        /* 基础前进速度 (counts/s) */
    float max_turn;          /* 最大差速补偿 (counts/s) */
} Steer_t;
```

### 3.4 全局实例（统一命名）
```c
MotorSpeed_t g_spd_left, g_spd_right;
Steer_t      g_steer;
```

---

## 4. 核心接口

### pid.h（精简后）
```c
/* 通用 PID */
void pid_begin(PID_t *p, float kp, float ki, float kd, float out_max, float integral_max);
void pid_reset(PID_t *p);
float pid_step(PID_t *p, float setpoint, float measure, float dt);  /* 返回 output */

/* 速度环 */
void speed_init(MotorSpeed_t *s, Motor_Channel_t ch, volatile int32_t *enc, float kp, float ki, float kd);
void speed_set_target(MotorSpeed_t *s, float target);
void speed_update(MotorSpeed_t *s, float dt_ms);   /* 内部读编码器、算速度、写 PWM */
void speed_stop(MotorSpeed_t *s);

/* 方向环 */
void steer_init(float kp, float ki, float kd, float max_turn);
void steer_set_base(float base);
void steer_stop(void);
void steer_step(float error, float dt_ms);   /* 读 patrol error, 输出到两电机目标 */

/* 应用层 */
void pid_app_init(void);
void pid_app_update(void);
```

### 保持兼容的 UART 接口（在 gFunc.c 中实现，语义保持不变）
- `spid / skp / ski / skd / gpid` → 内部调用新的 `speed_*` 函数
- `starget / gspeed` → 同上
- `stpid / stkp / stki / stkd / gtpid` → 同上
- `sbase / sstop` → 同上
- `debug_speed_only / debug_speed_off` → 保留（调试用）

---

## 5. 关键实现要点

### 5.1 稳定版 PID 算法
```c
float pid_step(PID_t *p, float setpoint, float measure, float dt)
{
    float error   = setpoint - measure;
    float derr    = (error - p->last_error) / dt;
    float raw_out = p->kp * error + p->ki * p->integral + p->kd * derr;

    /* 抗饱和：仅在输出不饱和时才积分 */
    bool saturated_pos = (raw_out >= p->out_max) && (error > 0);
    bool saturated_neg = (raw_out <= p->out_min) && (error < 0);
    if (!saturated_pos && !saturated_neg) {
        p->integral += error * dt;
        /* 积分软限幅 */
        if (p->integral >  p->integral_max) p->integral =  p->integral_max;
        if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    }

    float out = p->kp * error + p->ki * p->integral + p->kd * derr;
    if (out >  p->out_max) out =  p->out_max;
    if (out <  p->out_min) out =  p->out_min;

    p->last_error = error;
    return out;
}
```

> 注意：上式里 `raw_out` 只是用来判定饱和的，最终 `out` 用新 `integral` 重新计算一次再 clamp。这比"先积分后 clamp"的写法更稳定。

### 5.2 速度环（10ms 调用一次）
```c
void speed_update(MotorSpeed_t *s, float dt_ms)
{
    float dt = dt_ms / 1000.0f;
    int32_t cur = *s->enc;
    float vel = (float)(cur - s->last_enc) / dt;
    s->speed = vel;
    s->last_enc = cur;

    /* pid.setpoint 由 speed_set_target 设定 */
    float out = pid_step(&s->pid, s->pid.setpoint, vel, dt);
    motor_set_speed(s->ch, (int16_t)out);
}
```

**去掉的东西**：
- `speed_alpha` 低通滤波 → 删除
- `polarity` → 删除（统一正向约定：前进 = encoder 增量为正）
- 超速保护 → 删除（极性正确后不需要）
- `first_update` → 用抗饱和代替

> ⚠️ **极性统一约定**：重新烧录前必须确认 `encoder_left_count` 和 `encoder_right_count` 在正转（前进）时都是 **正增量**。如果实际相反，应在 `encoder` 层面解决（改极性/接线），而不是在 PID 层补偿。

### 5.3 方向环（与速度环独立，50Hz）
```c
void steer_step(float error, float dt_ms)
{
    float dt = dt_ms / 1000.0f;
    /* steer PID 输出: turn (counts/s 差速补偿) */
    float turn = pid_step(&g_steer.pid, 0.0f, error, dt);

    float base = g_steer.base_speed;
    float tl = base + turn;
    float tr = base - turn;

    /* 限幅到最大允许速度 (例如 7999) */
    if (tl >  7999) tl =  7999;
    if (tl < -7999) tl = -7999;
    if (tr >  7999) tr =  7999;
    if (tr < -7999) tr = -7999;

    /* 直接写入速度环 setpoint */
    speed_set_target(&g_spd_left,  tl);
    speed_set_target(&g_spd_right, tr);
}
```

### 5.4 `pid_app_update()` — 唯一主循环入口
```c
void pid_app_update(void)
{
    /* 方向环 50Hz：在 wheels.c 的 steer_flag 分支里调 */
    /* 速度环 100Hz：在 wheels.c 的 speed_flag 分支里调 */
    /* 把两套 update 拆开，由主循环按 flag 分别触发 */
}
```

---

## 6. 文件变更清单

| 文件 | 动作 | 说明 |
|---|---|---|
| `App/pid.h` | **重写** | 精简结构体与接口 |
| `App/pid.c` | **重写** | 新 PID + 新速度环 + 新方向环 |
| `wheels.c`  | **修改** | 调用新接口；恢复 steer_flag 分支；调试输出保留 |
| `Drivers/gFunc.c` | **小改** | 所有 UART 命令改为调用新接口函数，命令字符串/格式保持不变 |
| `Drivers/gFunc.h` | 不改 | |
| `App/patrol.h / patrol.c` | 不改 | 继续提供 `patrol_get_error()` |

---

## 7. 风险与应对

| 风险 | 应对 |
|---|---|
| 去掉低通滤波后，速度环抖动 | 启动时用很小的 Kp（0.01~0.02）即可；调试阶段保留 `debug_speed_only` 命令 |
| 极性方向不一致导致正反馈 | **在改 PID 前必须先验证** `encoder` 正负方向；在 `pid_app_init` 里加诊断输出（手动转动车轮，打印 delta） |
| `integral_max` 取值不当导致响应过慢/过冲 | 默认设为 `output_max * 0.5`，调参时通过 `stpid/skp` 等动态修改后自动重算 |
| 方向环/速度环循环调用 `speed_set_target` 的 overhead | 无开销，它只是写一个 `float setpoint` |
| 原有 Python 脚本 `auto_tune.py` 解析 `D:` 调试串 | **保持兼容**：保留 `uart_printf(UART0, "D: %5.1f, %5.1f, %d, %10d\r\n", ...)` 的格式，只替换内部字段 |

---

## 8. 验证步骤（烧录后）

1. **手动转动车轮** → 观察 `D:` 串，确认前进方向 `encoder_left_count` 和 `encoder_right_count` 都是**正增量**。如为负，需要在 `encoder.c` 层面修正极性（而非 PID 层）。
2. **开环测试**：`m 200 200` → 小车前进；`m 200 -200` → 原地旋转。
3. **速度环测试**：`debug_speed_only` → `starget b 500` → 观察 `gspeed` 能否稳定到 500 ± 50。
4. **调 Kp/Ki/Kd**：使用 `spid / skp / ski / skd`。
5. **方向环测试**：`debug_speed_off` → `sbase 300` → 手动给循迹偏差，车应自动修正。
6. **边界测试**：`gpatrol` 确认全黑/全白的处理。

---

## 9. 里程碑（完成标志）

- [ ] `pid.h` / `pid.c` 重写完成，代码行数 **< 200 行**（不含注释）。
- [ ] `wheels.c` 调用新接口，主循环 10ms / 20ms 两个 flag 分支完整。
- [ ] `gFunc.c` 所有命令字符串/响应格式与旧版一致（`auto_tune.py` 无需修改）。
- [ ] 编译零 error、零 warning。
- [ ] 手动测试步骤 1–6 全部通过。
