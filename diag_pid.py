"""诊断 PID 内部状态 - 解析 D: 输出"""
import sys, time, re
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== PID 内部状态诊断 ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 停电机
print("1. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.3)
time.sleep(0.5)

# 2. 设置目标速度
print("\n2. 设置目标 l=+500...")
resp = stream.send_cmd("starget l 500", wait=0.3)
print(f"   {resp}")

# 3. 监控诊断输出
print("\n3. 监控诊断输出（3秒）...")
print(f"   解析 D: 行 - setpoint, speed, output, encoder_count")
print(f"   {'Time':>8} | {'sp':>10} | {'spd':>10} | {'out':>8} | {'cnt':>12}")
print("   " + "-" * 65)

start = time.time()
count = 0
last_values = None

while time.time() - start < 3.0:
    # 检查 raw_lines 中的 D: 行
    for line in list(stream.raw_lines):
        m = re.match(r'D:\s*sp=\s*([+-]?\d+\.?\d*)\s+spd=\s*([+-]?\d+\.?\d*)\s+out=\s*([+-]?\d+)\s+cnt=\s*([+-]?\d+)', line)
        if m:
            sp = float(m.group(1))
            spd = float(m.group(2))
            out = int(m.group(3))
            cnt = int(m.group(4))
            last_values = (sp, spd, out, cnt)
            count += 1
            if count <= 20:
                print(f"   {time.time()-start:8.2f} | {sp:10.1f} | {spd:10.1f} | {out:8d} | {cnt:12d}")
    
    time.sleep(0.05)

# 4. 停电机
print("\n4. 停电机...")
resp = stream.send_cmd("starget l 0", wait=0.3)
time.sleep(0.5)

# 5. 打印原始 D: 行
print("\n5. 所有 D: 行（最近20行）:")
d_lines = [line for line in list(stream.raw_lines) if line.startswith('D:')]
for line in d_lines[-20:]:
    print(f"   {line}")

# 6. 分析
print(f"\n{'='*60}")
if last_values:
    sp, spd, out, cnt = last_values
    print("最后一组数据：")
    print(f"  setpoint (sp): {sp}")
    print(f"  speed (spd):   {spd}")
    print(f"  output (out):  {out}")
    print(f"  encoder cnt:   {cnt}")
    print(f"\n分析：")
    if out == 0:
        print("  ❌ output=0，电机不动！")
        print("     可能原因：setpoint=0 或 speed 计算问题")
    elif out > 0:
        print(f"  ✅ output={out}（正PWM），电机应该正转")
    elif out < 0:
        print(f"  ✅ output={out}（负PWM），电机应该反转")
    
    if sp != 0 and spd == 0 and out == 0:
        print("\n  【问题】setpoint != 0，但 speed=0 且 output=0")
        print("  可能原因：")
        print("    1. encoder_count 没有更新（cnt 不变）")
        print("    2. speed 计算公式有误")
        print("    3. PID update 逻辑问题")
else:
    print("  ❌ 没有收到 D: 行！")
    print("     可能固件没有正确编译/烧录")

stream.close()