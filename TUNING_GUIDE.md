# Wheels 小车调参系统使用说明

## 固件协议检查结果

### 当前固件支持的命令

| 命令 | 格式 | 返回示例 | Python 脚本支持 |
|------|------|----------|-----------------|
| `skp <ch> <kp>` | 设置 kp | `OK L kp=0.300` | ✅ |
| `starget <ch> <speed>` | 设置目标速度 | `OK L target=-500.0` | ✅ |
| `gspeed <ch>` | 读取当前速度 | `Speed L: 2500.0 (target: -500.0)` | ✅ |
| `gpid <ch>` | 读取 PID 参数 | `PID L: kp=0.346 ki=0.000 kd=0.000` | ✅ |
| `help` | 显示帮助 | 命令列表 | ✅ |

### ⚠️ 需要烧录的新命令

| 命令 | 功能 | 状态 |
|------|------|------|
| `debug_speed_only` | 启用调试模式（跳过循迹保护） | 代码已编写，需烧录 |
| `debug_speed_off` | 恢复正常模式 | 代码已编写，需烧录 |

---

## Python 调参脚本说明

### 脚本列表

| 脚本 | 功能 | 适用场景 |
|------|------|----------|
| `serial_helper.py` | 基础串口通信 | 手动发送命令 |
| `auto_tune_v2.py` | 智能调参系统 | **推荐使用** |
| `tune_quick.py` | 快速调参（轮询模式） | 当前固件可用 |

### auto_tune_v2.py 功能详解

#### 1. 实时监控模式
```bash
python auto_tune_v2.py monitor l           # 无限监控左轮
python auto_tune_v2.py monitor l 10        # 监控 10 秒
```

#### 2. 采样模式（持续采样并统计）
```bash
python auto_tune_v2.py sample l 5 -500     # 采样 5 秒，目标速度 -500
```
输出包含：
- 平均速度、中位速度
- 最小/最大速度
- 标准差（衡量稳定性）
- 平均误差、最大误差

#### 3. 单步测试
```bash
python auto_tune_v2.py step l -500 0.3     # 测试 kp=0.3
```

#### 4. 扫描调参
```bash
python auto_tune_v2.py scan l -500        # 扫描左轮 kp 参数
```

#### 5. 数据记录（保存到 CSV）
```bash
python auto_tune_v2.py record l -500 0.3 10  # 记录 10 秒数据
```

---

## 调参流程（推荐）

### 步骤 1：烧录新固件
确保烧录了支持 `debug_speed_only` 命令的版本。

### 步骤 2：启用调试模式
```bash
python serial_helper.py "debug_speed_only"
```

### 步骤 3：设置 kp 并测试
```bash
# 先测试一个保守的 kp 值
python auto_tune_v2.py step l -500 0.1

# 如果稳定，逐步增加 kp
python auto_tune_v2.py step l -500 0.2
python auto_tune_v2.py step l -500 0.3
```

### 步骤 4：自动扫描
```bash
python auto_tune_v2.py scan l -500
```

### 步骤 5：精细调整并记录
```bash
# 找到最佳 kp 后，记录详细数据
python auto_tune_v2.py record l -500 <best_kp> 10
```

### 步骤 6：恢复正常模式
```bash
python serial_helper.py "debug_speed_off"
```

---

## 关于极性的说明

### 当前状态
- 左轮前进需要**负 PWM**（用户确认）
- 左轮编码器在前进时读数为**负**（用户确认）

### 期望映射
```
starget l -500 (负目标 = 前进)
  → PID 输出负
  → motor_set_speed(负)
  → 左轮前进 ✅
  → 编码器负向计数
  → speed = 负 (未取反)
  → error = -500 - (-500) = 0 ✅
```

### 如果速度方向相反
如果发现 `starget l -500` 后速度显示为**正值**，说明编码器极性需要调整。此时需要：
1. 在 [pid.c](file:///d:/document/mspm0Project/wheels/App/pid.c) 中修改 `speed_left.polarity`
2. 如果当前是 `-1.0f`，改为 `+1.0f`
3. 重新烧录测试

---

## 常用命令速查

### 串口基础
```bash
python serial_helper.py "help"                  # 查看所有命令
python serial_helper.py "debug_speed_only"      # 启用调试模式
python serial_helper.py "debug_speed_off"       # 恢复正常模式
```

### 速度环调参
```bash
python serial_helper.py "skp l 0.3"             # 设置左轮 kp
python serial_helper.py "starget l -500"        # 设置左轮目标速度
python serial_helper.py "gspeed l"              # 读取左轮速度
python serial_helper.py "gpid l"                # 读取左轮 PID 参数
```

### 转向环调参
```bash
python serial_helper.py "stkp 100"              # 设置转向环 kp
python serial_helper.py "gtpid"                 # 读取转向环参数
python serial_helper.py "gpatrol"               # 查看传感器状态
python serial_helper.py "sbase 2000"            # 设置前进速度
```

---

## 故障排查

### 问题：速度始终为 0
**原因**：固件未烧录或 LOST 保护生效
**解决**：
1. 确认已烧录最新固件
2. 发送 `debug_speed_only` 启用调试模式
3. 观察是否返回 `OK debug speed-only mode enabled`

### 问题：速度方向相反
**原因**：编码器极性不匹配
**解决**：检查并修改 `speed_left.polarity` 或 `speed_right.polarity`

### 问题：速度不稳定
**原因**：kp 过大或系统振荡
**解决**：减小 kp 值

### 问题：串口连接失败
**原因**：端口被占用
**解决**：关闭 CCS 调试会话和 Serial Terminal

---

## 文件路径参考

- 基础脚本：[serial_helper.py](file:///d:/document/mspm0Project/wheels/serial_helper.py)
- 智能调参：[auto_tune_v2.py](file:///d:/document/mspm0Project/wheels/auto_tune_v2.py)
- 快速调参：[tune_quick.py](file:///d:/document/mspm0Project/wheels/tune_quick.py)
- 固件命令处理：[gFunc.c](file:///d:/document/mspm0Project/wheels/Drivers/gFunc.c)
- 速度环控制：[pid.c](file:///d:/document/mspm0Project/wheels/App/pid.c)
