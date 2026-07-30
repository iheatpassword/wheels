"""速度环分步诊断 - 逐步测试每个环节"""
import sys, time, re
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=" * 60)
print("=== 速度环分步诊断 ===")
print("=" * 60)

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 清空 D: 行缓冲
stream.raw_lines.clear()

# ========== 测试 1: 验证串口通信 ==========
print("\n[测试 1] 验证串口通信...")
resp = stream.send_cmd("gpid l", wait=0.5)
print(f"  响应: {resp.strip()}")

# ========== 测试 2: 设置较大的目标速度，观察编码器 ==========
print("\n[测试 2] 设置目标速度 +2000，观察编码器反馈...")
stream.raw_lines.clear()
stream.send_cmd("starget l 2000", wait=0.3)
time.sleep(0.5)  # 等待电机启动

# 收集数据 2 秒
print("\n  收集 2 秒数据...")
d_data = []
start = time.time()
while time.time() - start < 2.0:
    for line in list(stream.raw_lines):
        if line.startswith('D:'):
            d_data.append(line.strip())
    time.sleep(0.05)

# 分析数据
print(f"  收到 {len(d_data)} 条 D: 数据")
if d_data:
    # 显示前 3 条和后 3 条
    for i, line in enumerate(d_data[:3]):
        print(f"    [{i}] {line}")
    if len(d_data) > 6:
        print(f"    ... ({len(d_data)-6} 条)")
    for i, line in enumerate(d_data[-3:], len(d_data)-3):
        print(f"    [{i}] {line}")
    
    # 解析数据
    enc_counts = []
    for line in d_data:
        m = re.search(r'cnt=\s*([+-]?\d+)', line)
        if m:
            enc_counts.append(int(m.group(1)))
    
    if enc_counts:
        print(f"\n  编码器计数范围: {min(enc_counts)} ~ {max(enc_counts)}")
        if max(enc_counts) == 0 and min(enc_counts) == 0:
            print("\n  ❌ 编码器计数始终为 0！")
            print("     问题：编码器无反馈")
            print("     可能原因：")
            print("       1. 电机没转动（PWM=10 太小无法启动）")
            print("       2. 编码器接线错误")
            print("       3. GPIO 中断配置问题")
        else:
            print(f"  ✅ 编码器有反馈，变化范围: {max(enc_counts) - min(enc_counts)}")
else:
    print("  ❌ 没有收到 D: 数据！")
    print("     可能 UART 通信问题")

# ========== 测试 3: 停止电机 ==========
print("\n[测试 3] 停止电机...")
stream.raw_lines.clear()
stream.send_cmd("starget l 0", wait=0.3)
time.sleep(0.3)

# 打印最终 D: 行
for line in list(stream.raw_lines):
    if line.startswith('D:'):
        print(f"  最终: {line.strip()}")

# ========== 诊断总结 ==========
print("\n" + "=" * 60)
print("诊断总结：")
print("=" * 60)
if not d_data:
    print("  1. UART 通信异常 - 检查串口连接")
elif d_data and all('cnt=0' in l for l in d_data):
    print("  1. 编码器无反馈 (cnt 始终为 0)")
    print("     - 尝试手动转动车轮看 cnt 是否变化")
    print("     - 检查编码器接线: DIO2=PA2, DIO1=PA1 (左轮)")
    print("     - 检查 GPIO 中断是否启用")
    
    # 检查 PWM 输出
    outputs = []
    for line in d_data:
        m = re.search(r'out=\s*([+-]?\d+)', line)
        if m:
            outputs.append(int(m.group(1)))
    if outputs:
        print(f"  2. PID 输出: {set(outputs)}")
        if all(o < 20 for o in outputs):
            print("     - PWM 输出过小 (< 20)，可能无法驱动电机")
            print("     - 建议增大 Kp 或直接测试电机能力")
else:
    print("  编码器有反馈，问题可能在 PID 参数调整")

stream.close()
