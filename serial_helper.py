import serial
import sys
import time
import argparse

DEFAULT_PORT = "COM7"
DEFAULT_BAUDRATE = 9600   # UART0 实际配置见 Debug/ti_msp_dl_config.h
DEFAULT_TIMEOUT = 3.0

def send_command(command, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE,
                 timeout=DEFAULT_TIMEOUT, post_sleep=0.5, debug=False,
                 eol="\r\n", ignore_initial_logs=True):
    """
    发送命令到串口并接收响应。
    
    Args:
        command: 要发送的命令字符串（不含结束符）
        port, baudrate, timeout: 串口参数
        post_sleep: 发送命令后等待响应的秒数（长命令需要更长）
        debug: 是否打印调试信息（时间戳+字节数）
        eol: 命令结束符（默认 \r\n）
        ignore_initial_logs: 打开串口后先清空可能累积的日志
    
    Returns:
        str: 设备响应（含 echo + 输出，已去首尾空白）
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1,
                            inter_byte_timeout=0.1,
                            exclusive=True)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 丢弃初始日志：给一点时间让缓冲里的日志被读掉
        if ignore_initial_logs:
            t0 = time.time()
            while time.time() - t0 < 0.3:
                if ser.in_waiting:
                    junk = ser.read(ser.in_waiting)
                    if debug:
                        sys.stderr.write(f"[丢弃 {len(junk)} 字节初始日志]\n")
                time.sleep(0.02)
            ser.reset_input_buffer()

        # 发送命令
        cmd_bytes = (command + eol).encode('utf-8')
        if debug:
            sys.stderr.write(f"[发送 {len(cmd_bytes)} 字节]: {repr(command + eol)}\n")
        ser.write(cmd_bytes)
        ser.flush()

        # 等待响应：在 post_sleep 秒内尽量读取
        start = time.time()
        buf = bytearray()
        last_data_time = start  # 记录最后一次收到数据的时间
        while time.time() - start < post_sleep:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buf.extend(data)
                last_data_time = time.time()  # 更新最后收到数据的时间
            else:
                # 只有当已经收到数据，并且空闲超过 0.8 秒才认为结束
                # 这样可以避免在调试日志和命令响应之间过早退出
                if len(buf) > 0 and time.time() - last_data_time > 0.8:
                    break
                time.sleep(0.02)

        ser.close()
        return buf.decode('utf-8', errors='replace').strip()

    except serial.SerialException as e:
        return f"[串口错误] {e}"
    except PermissionError as e:
        return f"[权限错误] {e}。请确认 CCS 终端已关闭且串口未被占用。"
    except Exception as e:
        return f"[未知错误] {type(e).__name__}: {e}"


def read_logs(port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE, duration=5.0,
              filter_lines=None, debug=False):
    """
    持续读取日志 duration 秒，按行返回（可选过滤）。
    
    Args:
        port, baudrate: 串口参数
        duration: 读取时长（秒）
        filter_lines: 字符串列表，包含这些字符串的行会被过滤掉
        debug: 是否打印调试信息
    
    Returns:
        list[str]: 过滤后的日志行列表
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        lines = []
        start = time.time()
        while time.time() - start < duration:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                # 按行拆分
                for line in data.split('\n'):
                    line = line.strip()
                    if line:
                        if filter_lines is None:
                            lines.append(line)
                        else:
                            # 只有不包含过滤关键字的行才保留
                            if not any(f in line for f in filter_lines):
                                lines.append(line)
            time.sleep(0.05)
        ser.close()
        return lines
    except Exception as e:
        return [f"[错误] {e}"]


def interactive(port, baudrate):
    """简易交互模式：一行一条命令"""
    print(f"交互模式 {port}@{baudrate}，输入 q 退出")
    while True:
        try:
            cmd = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if cmd.lower() in ('q', 'quit', 'exit'):
            break
        if not cmd:
            continue
        print(send_command(cmd, port, baudrate, debug=False))
        print()


def main():
    p = argparse.ArgumentParser(description="Wheels 小车 UART 调试助手")
    p.add_argument("command", nargs="?", help="要发送的命令（留空进入交互模式）")
    p.add_argument("-p", "--port", default=DEFAULT_PORT, help=f"串口 (默认 {DEFAULT_PORT})")
    p.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUDRATE,
                   help=f"波特率 (默认 {DEFAULT_BAUDRATE})")
    p.add_argument("-w", "--wait", type=float, default=0.8,
                   help="发送后等待响应秒数 (默认 0.8)")
    p.add_argument("-v", "--verbose", action="store_true", help="显示调试信息")
    args = p.parse_args()

    if args.command is None:
        interactive(args.port, args.baud)
        return

    print(f"命令: {args.command}  端口: {args.port}@{args.baud}  等待: {args.wait}s")
    print("-" * 50)
    resp = send_command(args.command, args.port, args.baud,
                        post_sleep=args.wait, debug=args.verbose)
    print(resp)


if __name__ == "__main__":
    main()
