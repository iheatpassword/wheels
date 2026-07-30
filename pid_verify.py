"""验证 PID 控制 - 使用正确的 polarity=-1"""
import sys, time
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== PID 控制验证 (polarity=-1, Kp=0.02) ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 停电机
print("1. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.3)
time.sleep(0.5)

# 2. 确认当前参数
print("\n2. 检查 PID 参数...")
resp = stream.send_cmd("gpid l", wait=0.3)
print(f"   {resp}")

# 3. 设置目标速度
print("\n3. 设置目标 l=+500...")
resp = stream.send_cmd("starget l 500", wait=0.3)
print(f"   {resp}")

# 4. 监控速度响应（5秒）
print("\n4. 监控速度响应（5秒）...")
print(f"   {'Time':>8} | {'PID L':>10} | {'RAW L':>10} | 说明")
print("   " + "-" * 60)

count = 0
last_pid = None
last_raw = None
peak_pid = 0
start = time.time()

while time.time() - start < 5.0:
    sp_l, sp_r = stream.get_both_speeds()
    raw_l, raw_r = stream.get_raw_speeds()
    
    if sp_l is not None:
        last_pid = sp_l
        last_raw = raw_l
        count += 1
        peak_pid = max(peak_pid, abs(sp_l))
        
        note = ""
        t = time.time() - start
        if t > 0.5:
            if abs(sp_l) > 5000:
                note = "⚠️ 高速"
            elif abs(sp_l) > 1000:
                note = "⚠️ 中速"
            elif abs(sp_l) > 100:
                note = "✅ 低速"
            if 400 < abs(sp_l) < 600:
                note = "🎯 接近目标"
        
        if count <= 20 or count % 3 == 1:
            print(f"   {t:8.2f} | {sp_l:10.1f} | {raw_l if raw_l else 0:10d} | {note}")
    
    time.sleep(0.05)

# 5. 停电机
print("\n5. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.3)
time.sleep(0.5)

# 6. 判断
print(f"\n{'='*60}")
print("结果：")
print(f"  最后 PID 速度: {last_pid}")
print(f"  峰值速度: {peak_pid}")
print(f"  目标: +500")
print(f"\n  判断：")
if last_pid is not None:
    if last_pid == 0:
        print("  ❌ 电机完全不动！")
    elif abs(last_pid) > 5000:
        print("  ❌ 仍然失控！")
    elif 400 < abs(last_pid) < 600:
        print("  ✅ PID 控制正常！稳定在目标附近")
    else:
        print(f"  ⚠️ 偏离目标，需要调整 Kp")
        print(f"     当前 Kp=0.02，实际速度={last_pid:.1f}")
        if last_pid > 500:
            print(f"     需要增大 Kp（速度不够）")
        elif last_pid < 100:
            print(f"     需要减小 Kp（速度过大或方向错误）")

stream.close()