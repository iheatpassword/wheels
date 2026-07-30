"""查看固件原始输出"""
import sys, time
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== 查看固件原始输出 ===\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 1. 停电机
print("1. 停电机...")
resp = stream.send_cmd("m 0 0", wait=0.5)
time.sleep(1.0)

# 2. 让电机开环正转
print("\n2. 让电机正转 (m 200 0)...")
resp = stream.send_cmd("m 200 0", wait=0.5)

# 3. 等待并打印所有原始输出
print("\n3. 打印所有原始输出（3秒）...")
time.sleep(1.0)  # 等 enc: 数据

start = time.time()
while time.time() - start < 3.0:
    if stream.raw_lines:
        # 打印最近的行
        lines = list(stream.raw_lines)
        for line in lines[-10:]:  # 只打印最近10行
            print(f"  >>> {line}")
        # 清空已打印的
        stream.raw_lines.clear()
    time.sleep(0.1)

# 4. 停电机
print("\n4. 停电机...")
resp = stream.send_cmd("m 0 0", wait=0.5)

# 5. 打印剩余的
print("\n5. 剩余输出:")
for line in list(stream.raw_lines)[-10:]:
    print(f"  >>> {line}")

stream.close()