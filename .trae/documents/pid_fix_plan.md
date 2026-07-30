# PID 逻辑修复方案

## 问题根源分析

经过对代码的完整审阅，已定位到 4 个独立的 bug，共同导致了所有异常现象：

### Bug 1（核心问题）：编码器极性配置错误 — 正反馈振荡
**位置**：`App/pid.c` 第 300-301 行 `pid_app_init()`
**现象**：
- 电机正转时，编码器计数 `encoder_count` **递减**（变为负数）
- 代码中 `polarity = +1.0f`，导致：
  ```
  raw_speed = (delta_count / dt) * (+1.0)  → 负数
  error    = setpoint - feedback = (+1000) - (负数)  → 超大正数
  output   = Kp * error  → 超大正数  → 电机加速
  ```
- 电机加速→编码器更负→error 更大→输出更大 → **正反馈失控**！

**验证**：用户手工转动编码器正方向读数为负，证实了此判断。

### Bug 2（恶化问题）：积分限幅过大导致"停不下来"
**位置**：`App/pid.c` 第 132-137 行 `calc_integral_limit()`
```c
static float calc_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f)
        return output_max / ki * 0.8f;  // 当 ki 很小时，此值巨大！
    return 1000.0f;  // ← 即使 ki=0，也返回 1000，远大于 output_max(399)
}
```
- 当 Ki=0 时，积分限幅为 1000，远超 PWM 最大值 399
- 正反馈运行时，积分累积到 1000（饱和）
- 用户将所有参数置 0 后，积分仍保留在 ±399（被 output_max 限幅），电机继续满速

### Bug 3（持续性问题）：设置目标速度时不清积分
**位置**：`App/pid.c` 第 84-88 行 `speed_control_set()`
```c
void speed_control_set(Speed_Control_t *sc, float target_speed)
{
    sc->pid.setpoint = target_speed;  // ← 仅更新设定值，不重置 PID
}
```
- 切换目标速度（如从 +1000 转为 -1000）时，积分项残留
- 旧的积分会与新目标方向对抗，导致电机短暂卡死不动
- 这直接解释了"反转调试电机完全不动"的现象

### Bug 4（安全性缺失）：没有超速保护
- 当正反馈导致电机失控时，没有机制能自动止损
- 用户必须手动将所有 PID 参数置 0 才能刹车

---

## 修复方案

### 修改文件列表
| 文件 | 修改内容 |
|------|----------|
| `App/pid.c` | 修正极性、修复积分限幅、添加目标切换清积分、添加超速保护 |
| `Drivers/gFunc.c` | 扩展调试输出，增加更多诊断信息 |

### 修改步骤

#### Step 1: 修正编码器极性
在 `pid_app_init()` 中，将左右轮极性同时改为 `-1.0f`：
```c
// 修改前
speed_left.polarity  = +1.0f;
speed_right.polarity = +1.0f;

// 修改后
speed_left.polarity  = -1.0f;
speed_right.polarity = -1.0f;
```
**原理**：极性 -1.0 将反转编码器读数，使正转产生正反馈，形成正确的负反馈闭环。

#### Step 2: 修复积分限幅计算
修改 `calc_integral_limit()` 确保返回值永不超过 `output_max`：
```c
static float calc_integral_limit(float ki, float output_max)
{
    float limit;
    if (ki > 0.001f)
        limit = output_max / ki * 0.8f;
    else
        limit = output_max * 0.5f;  // ← Ki=0 时保守限幅
    // 保证积分限幅永不超过输出最大值
    if (limit > output_max) limit = output_max;
    return limit;
}
```

#### Step 3: 目标切换时清积分
修改 `speed_control_set()` 在设置新目标时清零积分项：
```c
void speed_control_set(Speed_Control_t *sc, float target_speed)
{
    if (sc == NULL) return;
    // 目标改变时清零积分，防止积分残留导致起步冲击
    if (sc->pid.setpoint != target_speed) {
        sc->pid.integral = 0.0f;
        sc->pid.last_error = 0.0f;
        sc->pid.first_update = 1;
    }
    sc->pid.setpoint = target_speed;
}
```

#### Step 4: 添加超速保护机制
在 `speed_control_update()` 中增加速度异常保护：
```c
void speed_control_update(Speed_Control_t *sc, float dt)
{
    // ... 现有代码 ...
    
    /* 超速保护：实际速度超过目标太多时强制停机 */
    float speed_abs = fabsf(sc->speed);
    float target_abs = fabsf(sc->pid.setpoint);
    if (target_abs > 0.0f && speed_abs > target_abs * 2.5f + 500.0f) {
        // 异常反馈：停车并输出警告
        sc->pid.output = 0.0f;
        sc->pid.integral = 0.0f;
        motor_set_speed(sc->motor_channel, 0);
        return;  // 跳过本轮正常输出
    }
    
    // ... 继续正常 PID 计算 ...
}
```

#### Step 5: 增强调试输出
在 `wheels.c` 的调试输出中增加 raw_speed 观察：
```c
uart_printf(UART0, "D: %5.1f spd=%5.1f raw=%5.1f out=%d enc=%ld\r\n",
            speed_left.pid.setpoint, speed_left.speed,
            speed_left.raw_speed,
            (int)speed_left.pid.output, (long)encoder_left_count);
```

---

## 验证步骤（烧录后执行）

### 测试 1: 极性验证（最低风险）
```
skp l 0      ← 关闭 P 项
starget l 500 ← 设定小目标
gspeed l     ← 读取速度
```
**预期**：`speed` 为正值（+50 左右），表示方向正确

### 测试 2: 正转调试
```
spid l 0.05 0 0  ← 保守起步
starget l 1000
gspeed l          ← 逐步调整 Kp
```
**预期**：速度稳定在 1000 附近，无振荡

### 测试 3: 反转调试
```
starget l -1000
gspeed l
```
**预期**：速度稳定在 -1000 附近，电机平稳反转

### 测试 4: 双向切换
```
starget l 1000   ← 等稳定
starget l -1000  ← 立即反转
gspeed l         ← 观察是否平滑过渡
```
**预期**：电机平滑加减速，无卡死

### 测试 5: 失控保护
```
skp l 0.5        ← 故意设大 Kp
starget l 1000
```
**预期**：超速保护触发（如有警告日志），电机不会一直满速

---

## 风险评估

| 风险 | 应对措施 |
|------|----------|
| 极性反了之后，某些边界情况可能仍不稳定 | 超速保护机制兜底 |
| 积分清零可能导致切换目标时短暂冲击 | 仅在目标**改变时**清零，不影响同目标稳态 |
| 修改后编码器读数符号变化，调试脚本需更新 | 调试输出已同时打印 `raw_speed`，便于核查 |

## 回滚方案
若修复后出现新问题，可通过 UART 命令临时恢复：
```
spid b 0 0 0       ← 紧急关闭所有 PID
m 0 0              ← 直接指令刹车
```
