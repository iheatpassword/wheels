"""手动 PID 验证：直接观察电机速度响应"""
import sys, time, re
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== 手动 PID 验证 ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 启用调试模式
print("1. 启用 debug_speed_only...")
resp = stream.send_cmd("debug_speed_only", wait=0.5)
print(f"   {resp}")

# 2. 设置 Kp=0.02
print("\n2. 设置 Kp=0.02...")
resp = stream.send_cmd("skp l 0.02", wait=0.5)
print(f"   {resp}")

# 3. 停电机
print("\n3. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.5)
print(f"   {resp}")
time.sleep(1.0)

# 4. 检查当前速度
sp_l, sp_r = stream.get_both_speeds()
print(f"\n4. 当前速度: L={sp_l}, R={sp_r}")

# 5. 设置目标速度 +500
print("\n5. 设置目标 l=+500...")
resp = stream.send_cmd("starget l 500", wait=0.5)
print(f"   {resp}")

# 6. 监控速度响应（5秒）
print("\n6. 监控速度响应（5秒）...")
print(f"   {'Time':>8} | {'L':>10} | 说明")
print("   " + "-" * 45)
start = time.time()
last_speed = None
samples = []
while time.time() - start < 5.0:
    sp_l, sp_r = stream.get_both_speeds()
    if sp_l is not None:
        last_speed = sp_l
        samples.append(sp_l)
        elapsed = time.time() - start
        
        note = ""
        if abs(sp_l) > 5000:
            note = "⚠️ 高速"
        elif abs(sp_l) > 1000:
            note = "⚠️ 中速"
        elif abs(sp_l) > 100:
            note = "✅ 低速"
        elif sp_l == 0:
            note = "⏸️ 静止"
        
        if len(samples) % 5 == 1 or len(samples) == 1:
            print(f"   {elapsed:8.1f} | {sp_l:10.1f} | {note}")
    
    time.sleep(0.05)

# 7. 停电机
print("\n7. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.5)
print(f"   {resp}")
time.sleep(0.5)

# 8. 结果总结
if last_speed:
    print(f"\n{'='*50}")
    print(f"结果:")
    print(f"  最终速度: {last_speed:.1f} counts/s")
    print(f"  目标速度: 500 counts/s")
    print(f"  绝对误差: {abs(last_speed - 500):.1f} counts/s")
    
    if last_speed == 0:
        print(f"\n  ❌ 电机完全不动！可能需要检查：")
        print(f"     - 极性是否正确")
        print(f"     - 电机驱动是否正常")
        print(f"     - PID 输出是否为 0")
    elif abs(last_speed) > 5000:
        print(f"\n  ❌ 仍然失控！Kp 可能太大或极性仍有问题")
    elif 400 < abs(last_speed) < 600:
        print(f"\n  ✅ 接近目标！PID 控制正常")
    else:
        print(f"\n  ⚠️ 偏离目标，需要调整 Kp")

stream.close()
print("\n测试完成")