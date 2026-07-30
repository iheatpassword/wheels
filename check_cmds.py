"""检查可用命令"""
import sys, time
sys.path.insert(0, '.')
from auto_tune import SerialStream, DEFAULT_PORT, DEFAULT_BAUDRATE

stream = SerialStream(DEFAULT_PORT, DEFAULT_BAUDRATE)
resp = stream.send_cmd("help", wait=1.0)
print("可用命令:")
print(resp)
stream.close()