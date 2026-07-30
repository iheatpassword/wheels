"""诊断脚本：检查 PID 内部状态"""
import sys, time, re
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== PID 诊断 ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 启用调试模式
print("1. 启用 debug_speed_only...")
resp = stream.send_cmd("debug_speed_only", wait=0.5)
print(f"   {resp}")

# 2. 停电机
print("\n2. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.5)
print(f"   {resp}")
time.sleep(0.5)

# 3. 检查当前 PID 参数
print("\n3. 检查 PID 参数...")
resp = stream.send_cmd("gpid l", wait=0.5)
print(f"   {resp}")

# 4. 检查当前速度
print("\n4. 检查当前速度...")
resp = stream.send_cmd("gspeed l", wait=0.5)
print(f"   {resp}")

# 5. 设置较大的 Kp=0.1 测试
print("\n5. 设置 Kp=0.1 (较大值)...")
resp = stream.send_cmd("skp l 0.1", wait=0.5)
print(f"   {resp}")

# 6. 设置目标速度
print("\n6. 设置目标 l=+500...")
resp = stream.send_cmd("starget l 500", wait=0.5)
print(f"   {resp}")

# 7. 等 1 秒后检查
time.sleep(1.0)
print("\n7. 检查速度...")
resp = stream.send_cmd("gspeed l", wait=0.5)
print(f"   {resp}")

# 8. 监控 enc: 输出
print("\n8. 监控 enc: 输出（3秒）...")
print(f"   {'Time':>8} | {'L':>10} | {'R':>10}")
print("   " + "-" * 38)
start = time.time()
count = 0
last_l = None
while time.time() - start < 3.0:
    sp_l, sp_r = stream.get_both_speeds()
    if sp_l is not None:
        last_l = sp_l
        count += 1
        if count % 3 == 1:
            print(f"   {time.time()-start:8.1f} | {sp_l:10.1f} | {sp_r:10.1f}")
    time.sleep(0.05)

print(f"\n   收到 {count} 条数据，最后 L={last_l}")

# 9. 用 gspeed 再查一次
print("\n9. 再查 gspeed...")
resp = stream.send_cmd("gspeed l", wait=0.5)
print(f"   {resp}")

# 10. 停电机
print("\n10. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.5)
print(f"   {resp}")

stream.close()
print("\n诊断完成")