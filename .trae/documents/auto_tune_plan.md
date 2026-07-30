# 自动速度环调参计划

## 一、现状审查

### 现有脚本问题

| 问题 | 说明 |
|------|------|
| **串口每次开闭** | `serial_helper.send_command()` 每次打开/关闭串口，开销巨大（500ms+/次） |
| **响应解析脆弱** | `gspeed` 命令的响应被 `enc:` 调试输出（每100ms一行）干扰，regex 解析不可靠 |
| **无流式读取** | 没有持久连接来持续读取调试流中的速度数据 |
| **版本混乱** | `auto_tune.py` / `auto_tune_v2.py` / `auto_tune_v3.py` 功能重叠 |
| **无真正自动化** | `scan()` 只报告统计结果，不会自动寻找最优参数 |

### 固件已具备的条件

- `wheels.c` 每 100ms 输出一行 `enc: L=xxx.x R=xxx.x`
- 支持 `skp/ski/skd` 设置单个 PID 参数
- 支持 `starget` 设置目标速度，速度环持续运行
- 调试模式：`debug_speed_only` 跳过循迹保护

**结论：固件不需要修改，问题全在 Python 脚本。**

---

## 二、简化方案

### 核心思路

**跳过 `gspeed` 命令，直接解析固件已有的 `enc:` 调试输出流。**

固件每 100ms 输出一行速度数据。只要建立持久串口连接，就能实时获取速度，无需额外命令。

### 架构设计

```
┌─────────────────────────────────────────────┐
│              auto_tune.py (单一脚本)          │
├─────────────────────────────────────────────┤
│                                             │
│  SerialStream (持久连接)                     │
│  ├── 后台线程持续读取串口                    │
│  ├── 解析 "enc: L=xxx R=xxx" 行 → 缓存速度  │
│  ├── send_cmd() 发送命令（skp/starget 等）   │
│  └── get_speed() 从缓存获取最新速度          │
│                                             │
│  StepResponseAnalyzer                       │
│  ├── 捕获阶跃响应（速度 vs 时间）            │
│  ├── 计算：稳态误差、超调量、稳定时间        │
│  └── 输出评分（越小越好）                   │
│                                             │
│  AutoTuner                                  │
│  ├── tune_kp()   自动搜索最优 Kp            │
│  ├── tune_ki()   在最优 Kp 基础上搜 Ki      │
│  ├── tune_kd()   在最优 Kp+Ki 基础上搜 Kd   │
│  └── tune_all()  一键全自动调参              │
│                                             │
└─────────────────────────────────────────────┘
```

### 调参流程（3 步）

```
Step 1: 停电机 → 设置 Kp → 设目标速度 → 记录阶跃响应 → 评分 → 停电机
Step 2: 遍历 Kp 列表，重复 Step 1，选出得分最高的 Kp
Step 3: 固定最优 Kp，遍历 Ki/Kd，同上
```

---

## 三、实现步骤

### 步骤 1：创建 `auto_tune.py`（替代所有旧版本）

**文件**: `d:\document\mspm0Project\wheels\auto_tune.py`

#### 1.1 SerialStream 类

```python
class SerialStream:
    """持久串口连接 + 后台读取线程"""
    
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)
        self.speed_cache = {'l': None, 'r': None}  # 最新速度
        self.speed_lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.start()
    
    def _read_loop(self):
        """后台线程：持续读取并解析 enc: 行"""
        while self.running:
            line = self.ser.readline().decode(errors='replace')
            # 解析: "enc: L= 2500.0 R= 2480.0"
            m = re.match(r'enc:\s*L=([+-]?\d+\.?\d*)\s+R=([+-]?\d+\.?\d*)', line)
            if m:
                with self.speed_lock:
                    self.speed_cache['l'] = float(m.group(1))
                    self.speed_cache['r'] = float(m.group(2))
    
    def get_speed(self, ch):
        """获取最新速度（线程安全）"""
        with self.speed_lock:
            return self.speed_cache.get(ch)
    
    def send_cmd(self, cmd, wait=0.5):
        """发送命令并返回响应"""
        # 跳过 enc: 行，只取命令响应
        ...
    
    def close(self):
        self.running = False
        self.thread.join()
```

#### 1.2 StepResponseAnalyzer 类

```python
class StepResponseAnalyzer:
    """阶跃响应分析"""
    
    def capture(self, stream, channel, target, duration=5.0):
        """
        捕获阶跃响应
        
        Returns:
            {
                'times': [...],      # 时间点
                'speeds': [...],     # 速度值
                'steady_speed': ..., # 稳态速度
                'steady_error': ..., # 稳态误差(%)
                'overshoot': ...,    # 超调量(%)
                'settle_time': ...,  # 稳定时间(s)
                'score': ...         # 综合评分
            }
        """
    
    def score(self, result):
        """
        评分：稳态误差 + 超调 + 稳定时间
        分数越低越好
        """
```

#### 1.3 AutoTuner 类

```python
class AutoTuner:
    """自动调参"""
    
    def tune_kp(self, channel, target, kp_range):
        """扫描 Kp，返回最优值"""
    
    def tune_ki(self, channel, target, best_kp, ki_range):
        """固定 Kp，扫描 Ki"""
    
    def tune_kd(self, channel, target, best_kp, best_ki, kd_range):
        """固定 Kp+Ki，扫描 Kd"""
    
    def tune_all(self, channel, target):
        """一键全自动：Kp → Ki → Kd"""
```

#### 1.4 CLI 接口

```
python auto_tune.py tune l -500         # 全自动调参（推荐）
python auto_tune.py kp_scan l -500      # 仅扫描 Kp
python auto_tune.py step l -500 0.3     # 手动单步测试
python auto_tune.py monitor l           # 实时监控
```

### 步骤 2：删除旧脚本

删除以下冗余文件（功能已合并到 auto_tune.py）：
- `auto_tune.py` (旧版)
- `auto_tune_v2.py`
- `auto_tune_v3.py`
- `tune_quick.py`
- `tune_speed_loop.py`
- `tune_helper.py`

保留：
- `serial_helper.py`（基础串口工具，手动调试用）

### 步骤 3：测试流程

1. 烧录当前固件（已就绪）
2. `python auto_tune.py monitor l` — 验证能否正确读取速度
3. `python auto_tune.py step l -500 0.3` — 验证单步测试
4. `python auto_tune.py tune l -500` — 执行全自动调参

---

## 四、关键设计决策

| 决策 | 理由 |
|------|------|
| 不用 `gspeed` 命令 | 固件每 100ms 已输出速度，直接解析更可靠 |
| 持久串口连接 | 避免每次开闭的开销，支持实时流式读取 |
| 后台读取线程 | 不阻塞主线程，命令发送和速度读取可并行 |
| 先扫 Kp 再扫 Ki/Kd | 按影响从大到小，效率最高 |
| 评分函数：误差+超调+稳定时间 | 综合评估，避免选到有震荡的参数 |

## 五、风险处理

| 风险 | 处理 |
|------|------|
| 固件输出格式变化 | 解析函数加容错，记录原始行 |
| 串口偶尔阻塞 | 设置超时，超时后重连 |
| 电机响应过慢/过快 | 扫描范围可配置，`duration` 参数自适应 |
| 旧脚本用户习惯 | 保留 `serial_helper.py` 供手动使用 |