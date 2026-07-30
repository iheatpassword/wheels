"""手动编码器测试 - 手动转动车轮观察计数变化"""
import sys, time
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

print("=== 编码器手动测试 ===")
print("请手动转动左轮，观察 cnt 计数是否变化\n")

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)

# 停电机
stream.send_cmd("starget l 0", wait=0.2)
stream.send_cmd("starget r 0", wait=0.2)

print("开始监控（5秒），请在此期间手动转动车轮...\n")
print(f"  {'Time':>8} | {'sp':>8} | {'spd':>8} | {'cnt':>10}")
print("  " + "-" * 50)

start = time.time()
last_cnt = None
changes = 0

while time.time() - start < 5.0:
    for line in list(stream.raw_lines):
        if line.startswith('D:'):
            # 解析 D: 行
            parts = line.split()
            vals = {}
            for i, p in enumerate(parts):
                if p in ('sp=', 'spd=', 'out=', 'cnt='):
                    key = p.rstrip('=')
                    if i+1 < len(parts):
                        vals[key] = parts[i+1]
            
            cnt_str = vals.get('cnt', '0').lstrip('+')
            try:
                cnt = int(cnt_str)
            except:
                cnt = 0
            
            if last_cnt is None:
                last_cnt = cnt
            
            if cnt != last_cnt:
                changes += 1
                print(f"  {time.time()-start:8.2f} | {'':>8} | {'':>8} | {cnt:>10}  *** 变化! ***")
                last_cnt = cnt
    
    time.sleep(0.05)

print(f"\n检测到 {changes} 次计数变化")

if changes == 0:
    print("\n❌ 编码器计数没有变化！")
    print("可能原因：")
    print("  1. 编码器接线错误")
    print("  2. GPIO 中断未正确配置")
    print("  3. 编码器脉冲信号未到达")
else:
    print("\n✅ 编码器工作正常")

stream.close()
