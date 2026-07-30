"""
极性验证 - 直接用 m 命令，绕过 PID

核心思路：发送 m 命令后立即读取编码器速度
"""
import sys, time
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== 极性验证 (绕过 PID) ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 停电机
print("1. 停电机...")
resp = stream.send_cmd("m 0 0", wait=0.3)
time.sleep(0.5)

# 2. 发送 m 200 命令（直接设置电机，PID 会覆盖但会有短暂转动）
print("\n2. 发送 m 200 0...")
print("   【请观察电机是前进还是后退】")
resp = stream.send_cmd("m 200 0", wait=0.3)

# 3. 快速读取编码器数据
print("\n3. 监控 enc: 输出（5秒）...")
print(f"   {'Time':>8} | {'PID L':>10} | {'RAW L':>10}")
print("   " + "-" * 45)

count = 0
last_pid = None
last_raw = None
start = time.time()

while time.time() - start < 5.0:
    sp_l, sp_r = stream.get_both_speeds()
    raw_l, raw_r = stream.get_raw_speeds()
    
    if sp_l is not None or raw_l is not None:
        last_pid = sp_l
        last_raw = raw_l
        count += 1
        if count <= 15:
            print(f"   {time.time()-start:8.2f} | {sp_l:10.1f} | {raw_l:10d}")
    
    time.sleep(0.05)

# 4. 停电机
print("\n4. 停电机...")
resp = stream.send_cmd("m 0 0", wait=0.3)
time.sleep(0.5)

# 5. 打印原始 raw_lines
print("\n5. 原始串口数据（最近10行）:")
for line in list(stream.raw_lines)[-10:]:
    print(f"   {line}")

# 6. 判断
print(f"\n{'='*60}")
print("结果：")
print(f"  最后 PID 速度: {last_pid}")
print(f"  最后 RAW 编码器: {last_raw}")
print(f"\n请判断极性：")
print(f"  - 如果电机【前进】且 RAW 为【正】 → 用 polarity = +1")
print(f"  - 如果电机【前进】且 RAW 为【负】 → 用 polarity = -1")
print(f"  - 如果电机【后退】且 RAW 为【正】 → 用 polarity = -1")
print(f"  - 如果电机【后退】且 RAW 为【负】 → 用 polarity = +1")
print(f"{'='*60}")

stream.close()