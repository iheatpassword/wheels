"""
Wheels 小车自动速度环调参系统

核心思路：直接解析固件已有的 enc: 调试输出流（每 100ms 一行），
建立持久串口连接 + 后台读取线程，实现真正的自动化调参。

极性约定：
  左轮: polarity=+1（编码器方向与电机一致）
  右轮: polarity=+1（编码器方向与电机一致）
  正向目标 (setpoint>0) = 前进方向

使用方法:
  python auto_tune.py monitor l              # 实时监控速度
  python auto_tune.py step l 500 0.02        # 手动单步测试 (Kp=0.02, 小保守起步)
  python auto_tune.py kp_scan l 500          # 扫描 Kp 参数
  python auto_tune.py tune l 500             # 一键全自动调参
"""

import sys
import time
import re
import statistics
import threading
import argparse
from collections import deque

try:
    import serial
except ImportError:
    print("请安装 pyserial: pip install pyserial")
    sys.exit(1)

DEFAULT_PORT = "COM7"
DEFAULT_BAUDRATE = 9600


class SerialStream:
    """
    持久串口连接 + 后台读取线程
    
    设计原则：只有后台线程能读串口，主线程通过队列获取响应。
    - _read_loop: 唯一串口读取者，解析 enc: 更新速度缓存，
                   其他行放入 response_queue 供 send_cmd 读取
    - send_cmd:   发送命令后从 response_queue 收集响应
    """

    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.speed_cache = {'l': None, 'r': None}
        self.raw_cache = {'l': None, 'r': None}
        self.speed_lock = threading.Lock()
        self.raw_lines = deque(maxlen=500)
        self.response_queue = deque()
        self.running = False
        self.ser = None
        self._open()
        self._start_reader()

    def _open(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate,
                timeout=0.2,
                exclusive=True
            )
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as e:
            print(f"❌ 串口打开失败: {e}")
            print("   请确认 CCS 调试会话已关闭")
            sys.exit(1)

    def _start_reader(self):
        """启动后台读取线程"""
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        """
        后台读取循环（唯一串口读者）
        
        解析规则：
        - enc: 行 → 更新 speed_cache
        - 其他行 → 放入 response_queue（供 send_cmd 读取）
        - 所有行 → 放入 raw_lines（供调试）
        """
        buf = bytearray()
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buf.extend(data)
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        line_str = line.decode('utf-8', errors='replace').strip()
                        if line_str:
                            self._parse_line(line_str)
                else:
                    time.sleep(0.01)
            except Exception:
                if self.running:
                    time.sleep(0.1)

        if buf:
            line_str = buf.decode('utf-8', errors='replace').strip()
            if line_str:
                self._parse_line(line_str)

    def _parse_line(self, line):
        """解析一行数据，路由到对应缓存"""
        self.raw_lines.append(line)

        # 解析 PID 速度和原始编码器速度
        # 格式: enc: L=xxx.x R=xxx.x | raw: L=xxx R=xxx
        m = re.match(r'enc:\s*L=\s*([+-]?\d+\.?\d*)\s+R=\s*([+-]?\d+\.?\d*)', line)
        if m:
            with self.speed_lock:
                self.speed_cache['l'] = float(m.group(1))
                self.speed_cache['r'] = float(m.group(2))
            
            # 尝试解析原始编码器速度
            raw_m = re.search(r'raw:\s*L=\s*([+-]?\d+)\s+R=\s*([+-]?\d+)', line)
            if raw_m:
                with self.speed_lock:
                    self.raw_cache['l'] = int(raw_m.group(1))
                    self.raw_cache['r'] = int(raw_m.group(2))
        else:
            self.response_queue.append(line)

    def get_speed(self, ch):
        """获取最新速度（线程安全）"""
        with self.speed_lock:
            if ch == 'l':
                return self.speed_cache['l']
            elif ch == 'r':
                return self.speed_cache['r']
            else:
                return None

    def get_both_speeds(self):
        """获取双通道最新速度"""
        with self.speed_lock:
            return self.speed_cache['l'], self.speed_cache['r']

    def get_raw_speeds(self):
        """获取双通道原始编码器速度"""
        with self.speed_lock:
            return self.raw_cache['l'], self.raw_cache['r']

    def wait_speed(self, ch, timeout=3.0):
        """等待速度数据就绪"""
        start = time.time()
        while time.time() - start < timeout:
            if self.get_speed(ch) is not None:
                return True
            time.sleep(0.05)
        return False

    def send_cmd(self, cmd, wait=0.5):
        """
        发送命令并返回响应（从 response_queue 收集）
        
        由于只有后台线程读串口，这里通过队列安全获取响应。
        """
        try:
            self.response_queue.clear()

            cmd_bytes = (cmd + "\r\n").encode('utf-8')
            self.ser.write(cmd_bytes)
            self.ser.flush()

            start = time.time()
            response_lines = []
            last_data = start

            while time.time() - start < wait:
                while self.response_queue:
                    line = self.response_queue.popleft()
                    response_lines.append(line)
                    last_data = time.time()

                if response_lines and (time.time() - last_data) > 0.5:
                    break

                time.sleep(0.02)

            return '\n'.join(response_lines)

        except Exception as e:
            return f"[错误] {e}"

    def close(self):
        """关闭连接"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()


class StepResponseAnalyzer:
    """
    阶跃响应分析器
    
    捕获速度从 0 跳到目标值的过程，分析：
    - 稳态速度 / 稳态误差
    - 超调量
    - 稳定时间
    - 综合评分
    """

    def __init__(self):
        self.settle_threshold = 0.05  # 5% 误差视为稳定
        self.overshoot_threshold = 1.5  # 超过目标 150% 视为严重超调

    def capture(self, stream, channel, target, duration=5.0, stop_after=True):
        """
        捕获阶跃响应
        
        Args:
            stream: SerialStream 实例
            channel: 'l' 或 'r'
            target: 目标速度
            duration: 捕获时长（秒）
            stop_after: 测试后是否停电机
        
        Returns:
            dict: 分析结果
        """
        abs_target = abs(target)

        stream.send_cmd(f"skp {channel} 0")
        time.sleep(0.3)

        stream.send_cmd(f"starget {channel} 0")
        time.sleep(0.5)

        speed_history = []
        t0 = time.time()
        while time.time() - t0 < 1.0:
            sp = stream.get_speed(channel)
            if sp is not None:
                speed_history.append(sp)
            time.sleep(0.05)

        stream.send_cmd(f"starget {channel} {target}")
        start_time = time.time()

        samples = []
        speed_history.clear()
        max_speed = None

        while time.time() - start_time < duration:
            sp = stream.get_speed(channel)
            if sp is not None:
                elapsed = time.time() - start_time
                samples.append({
                    'time': elapsed,
                    'speed': sp,
                    'abs_speed': abs(sp)
                })
                speed_history.append(sp)

                if max_speed is None or abs(sp) > max_speed:
                    max_speed = abs(sp)

            time.sleep(0.05)

        if stop_after:
            stream.send_cmd(f"starget {channel} 0")
            time.sleep(0.3)

        if len(samples) < 3:
            return {'error': '采样数据不足', 'samples': samples}

        return self._analyze(samples, abs_target, max_speed)

    def _analyze(self, samples, abs_target, max_speed):
        """分析阶跃响应数据"""
        speeds = [s['speed'] for s in samples]
        abs_speeds = [s['abs_speed'] for s in samples]
        times = [s['time'] for s in samples]

        n = len(speeds)
        steady_start = n // 2
        steady_speeds = abs_speeds[steady_start:]

        if len(steady_speeds) < 3:
            steady_speeds = abs_speeds[-3:]

        steady_speed = statistics.mean(steady_speeds)
        steady_error = abs(steady_speed - abs_target) / abs_target * 100 if abs_target > 0 else 100.0

        overshoot = 0.0
        if max_speed and abs_target > 0:
            overshoot = (max_speed - abs_target) / abs_target * 100

        settle_time = times[-1]
        settled = False
        for i in range(n - 1, -1, -1):
            if abs(abs_speeds[i] - steady_speed) > steady_speed * self.settle_threshold:
                settle_time = times[min(i + 1, n - 1)]
                settled = True
                break
        if not settled:
            settle_time = 0.0

        score = self._calc_score(steady_error, overshoot, settle_time)

        return {
            'samples': len(samples),
            'target': abs_target,
            'steady_speed': steady_speed,
            'steady_error': steady_error,
            'max_speed': max_speed or 0,
            'overshoot': overshoot,
            'settle_time': settle_time,
            'score': score,
            'stable': steady_error < 10.0 and overshoot < 30.0,
            'speeds': speeds,
            'abs_speeds': abs_speeds,
            'times': times,
        }

    def _calc_score(self, steady_error, overshoot, settle_time):
        """
        综合评分（越低越好）
        
        score = error_weight * error% + overshoot_weight * overshoot% + time_weight * settle_time
        """
        return steady_error * 1.0 + overshoot * 0.5 + settle_time * 2.0

    def print_result(self, result, kp=None, ki=None, kd=None):
        """打印测试结果"""
        if 'error' in result:
            print(f"  ⚠️ {result['error']}")
            return

        p = []
        if kp is not None:
            p.append(f"Kp={kp:.3f}")
        if ki is not None:
            p.append(f"Ki={ki:.3f}")
        if kd is not None:
            p.append(f"Kd={kd:.3f}")
        pstr = ", ".join(p) if p else "参数"

        stable_mark = '✅稳定' if result['stable'] else '❌不稳定'

        print(f"  {pstr}:")
        print(f"    稳态速度: {result['steady_speed']:.1f} (误差 {result['steady_error']:.1f}%)")
        print(f"    峰值速度: {result['max_speed']:.1f} (超调 {result['overshoot']:.1f}%)")
        print(f"    稳定时间: {result['settle_time']:.2f}s")
        print(f"    评分: {result['score']:.2f}  {stable_mark}")


class AutoTuner:
    """
    自动调参器
    
    使用方式:
        tuner = AutoTuner(port, baudrate)
        result = tuner.tune('l', -500)  # 一键全自动
        tuner.close()
    """

    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        self.stream = SerialStream(port, baudrate)
        self.analyzer = StepResponseAnalyzer()
        self._verify_connection()

    def _verify_connection(self):
        """验证连接是否正常"""
        print("🔌 正在验证串口连接...")
        resp = self.stream.send_cmd("help", wait=0.5)
        if "Unknown command" in resp or "UART Debug" in resp:
            print("  ✅ 已连接到设备")
        else:
            print(f"  ⚠️ 响应异常: {resp[:50]}")

        self.stream.send_cmd("debug_speed_only", wait=0.3)
        print("  🛠️  已启用调试模式（跳过循迹保护）")

        self.stream.wait_speed('l', timeout=2.0)
        sp_l, sp_r = self.stream.get_both_speeds()
        print(f"  📡 初始速度: L={sp_l}, R={sp_r}")
        print()

    def _step_test(self, channel, target, kp, ki=0.0, kd=0.0, settle_wait=1.0):
        """
        单步测试：设置参数 → 捕获阶跃响应 → 返回分析结果
        """
        self.stream.send_cmd(f"skp {channel} {kp}")
        if ki > 0:
            self.stream.send_cmd(f"ski {channel} {ki}")
        if kd > 0:
            self.stream.send_cmd(f"skd {channel} {kd}")

        self.stream.send_cmd(f"starget {channel} 0")
        time.sleep(settle_wait)

        self.stream.send_cmd(f"starget {channel} {target}")

        start = time.time()
        last_speed = None
        while time.time() - start < 3.0:
            sp = self.stream.get_speed(channel)
            if sp is not None:
                last_speed = sp
                if abs(sp) > abs(target) * 0.8:
                    break
            time.sleep(0.05)

        duration = 5.0
        if abs(target) >= 3000:
            duration = 4.0

        result = self.analyzer.capture(
            self.stream, channel, target,
            duration=duration, stop_after=True
        )

        return result

    def tune_kp(self, channel, target, kp_list=None):
        """
        扫描 Kp 参数
        
        Args:
            channel: 'l' 或 'r'
            target: 目标速度（counts/s）
            kp_list: Kp 值列表
        
        Returns:
            dict: {'best_kp': float, 'best_score': float, 'results': list}
        """
        if kp_list is None:
            kp_list = [0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50]

        print(f'\n{"#" * 60}')
        print(f'# Kp 扫描: ch={channel}, target={target}')
        print(f'{"#" * 60}')
        print(f'共 {len(kp_list)} 个值，每个约 8-10 秒\n')

        results = []
        best = None

        for i, kp in enumerate(kp_list):
            print(f'[{i+1}/{len(kp_list)}] Kp={kp:.3f}', end='')
            result = self._step_test(channel, target, kp)

            if 'error' in result:
                print(f'  ⚠️ {result["error"]}')
                continue

            result['kp'] = kp
            results.append(result)

            self.analyzer.print_result(result, kp=kp)

            if result['stable']:
                if best is None or result['score'] < best['score']:
                    best = result

            time.sleep(0.5)

        self._print_summary('Kp', results, best)

        return {
            'best_kp': best['kp'] if best else None,
            'best_score': best['score'] if best else None,
            'results': results
        }

    def tune_ki(self, channel, target, best_kp, ki_list=None):
        """
        固定 Kp，扫描 Ki 参数
        """
        if ki_list is None:
            ki_list = [0.0, 0.2, 0.5, 1.0, 2.0, 5.0]

        print(f'\n{"#" * 60}')
        print(f'# Ki 扫描: ch={channel}, target={target}, 固定 Kp={best_kp}')
        print(f'{"#" * 60}')

        results = []
        best = None

        for i, ki in enumerate(ki_list):
            print(f'[{i+1}/{len(ki_list)}] Ki={ki:.3f}', end='')
            result = self._step_test(channel, target, best_kp, ki=ki)

            if 'error' in result:
                print(f'  ⚠️ {result["error"]}')
                continue

            result['ki'] = ki
            results.append(result)

            self.analyzer.print_result(result, kp=best_kp, ki=ki)

            if result['stable']:
                if best is None or result['score'] < best['score']:
                    best = result

            time.sleep(0.5)

        self._print_summary('Ki', results, best, fixed_kp=best_kp)

        return {
            'best_ki': best['ki'] if best else None,
            'best_score': best['score'] if best else None,
            'results': results
        }

    def tune_kd(self, channel, target, best_kp, best_ki, kd_list=None):
        """
        固定 Kp+Ki，扫描 Kd 参数
        """
        if kd_list is None:
            kd_list = [0.0, 0.01, 0.02, 0.05, 0.1]

        print(f'\n{"#" * 60}')
        print(f'# Kd 扫描: ch={channel}, Kp={best_kp}, Ki={best_ki}')
        print(f'{"#" * 60}')

        results = []
        best = None

        for i, kd in enumerate(kd_list):
            print(f'[{i+1}/{len(kd_list)}] Kd={kd:.3f}', end='')
            result = self._step_test(channel, target, best_kp, ki=best_ki, kd=kd)

            if 'error' in result:
                print(f'  ⚠️ {result["error"]}')
                continue

            result['kd'] = kd
            results.append(result)

            self.analyzer.print_result(result, kp=best_kp, ki=best_ki, kd=kd)

            if result['stable']:
                if best is None or result['score'] < best['score']:
                    best = result

            time.sleep(0.5)

        self._print_summary('Kd', results, best, fixed_kp=best_kp, fixed_ki=best_ki)

        return {
            'best_kd': best['kd'] if best else None,
            'best_score': best['score'] if best else None,
            'results': results
        }

    def tune_all(self, channel, target):
        """
        一键全自动调参：Kp → Ki → Kd
        
        Returns:
            dict: 最优参数
        """
        print(f'\n{"=" * 60}')
        print(f'🚀 全自动调参: ch={channel}, target={target}')
        print(f'{"=" * 60}')

        kp_result = self.tune_kp(channel, target)

        if kp_result['best_kp'] is None:
            print('\n⚠️ 未找到稳定的 Kp，调参终止')
            return kp_result

        best_kp = kp_result['best_kp']
        print(f'\n✨ 最优 Kp = {best_kp:.3f}')

        ki_result = self.tune_ki(channel, target, best_kp)
        best_ki = ki_result.get('best_ki', 0.0)
        if best_ki is None:
            best_ki = 0.0

        print(f'\n✨ 最优 Ki = {best_ki:.3f}')

        kd_result = self.tune_kd(channel, target, best_kp, best_ki)
        best_kd = kd_result.get('best_kd', 0.0)
        if best_kd is None:
            best_kd = 0.0

        print(f'\n✨ 最优 Kd = {best_kd:.3f}')

        self.stream.send_cmd(f"spid {channel} {best_kp} {best_ki} {best_kd}")
        self.stream.send_cmd(f"starget {channel} 0")

        print(f'\n{"=" * 60}')
        print(f'🎯 最终参数:')
        print(f'   Kp = {best_kp:.3f}')
        print(f'   Ki = {best_ki:.3f}')
        print(f'   Kd = {best_kd:.3f}')
        print(f'{"=" * 60}')

        return {
            'kp': best_kp,
            'ki': best_ki,
            'kd': best_kd,
            'kp_results': kp_result,
            'ki_results': ki_result,
            'kd_results': kd_result
        }

    def tune_channel(self, channel, target):
        """
        对单个通道进行全自动调参
        
        channel: 'l' 或 'r' 或 'b'
        """
        if channel == 'b':
            print('\n⚠️ 建议分别调参 L 和 R 通道，而不是同时调')
            print('   请分别使用: tune l 和 tune r')
            return None

        return self.tune_all(channel, target)

    def _print_summary(self, param_name, results, best, fixed_kp=None, fixed_ki=None):
        """打印扫描结果汇总"""
        print(f'\n{"=" * 60}')
        print(f'📊 {param_name} 扫描汇总')
        print(f'{"=" * 60}')

        headers = []
        if fixed_kp is not None:
            headers.append(f'Kp={fixed_kp:.2f}')
        if fixed_ki is not None:
            headers.append(f'Ki={fixed_ki:.2f}')
        headers.append(f'{param_name}')
        headers.extend(['误差%', '超调%', '稳定时间', '评分', '稳定'])

        print(' | '.join(f'{h:>8}' for h in headers))
        print('-' * (9 * len(headers) + 3 * (len(headers) - 1)))

        for r in results:
            p_val = r.get(param_name.lower(), 0)
            mark = '✅' if r.get('stable') else '❌'
            row = []
            if fixed_kp is not None:
                row.append(f'{fixed_kp:8.3f}')
            if fixed_ki is not None:
                row.append(f'{fixed_ki:8.3f}')
            row.extend([
                f'{p_val:8.3f}',
                f'{r["steady_error"]:8.1f}',
                f'{r["overshoot"]:8.1f}',
                f'{r["settle_time"]:8.2f}',
                f'{r["score"]:8.2f}',
                f'{mark:>8}'
            ])
            print(' | '.join(row))

        if best:
            print(f'\n🎯 推荐: {param_name}={best[param_name.lower()]:.3f} (评分 {best["score"]:.2f})')
        else:
            print(f'\n⚠️ 没有找到稳定的 {param_name}')

    def close(self):
        """关闭连接"""
        self.stream.send_cmd("starget b 0", wait=0.3)
        self.stream.close()


def main():
    parser = argparse.ArgumentParser(
        description='Wheels 小车自动速度环调参',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python auto_tune.py monitor l               实时监控左轮速度
  python auto_tune.py step l -500 0.3         手动测试 kp=0.3
  python auto_tune.py kp_scan l -500          扫描 Kp 参数
  python auto_tune.py tune l -500             一键全自动调参
  python auto_tune.py tune b -500             调两个通道
        """
    )

    parser.add_argument('mode', choices=['monitor', 'step', 'kp_scan', 'tune'],
                        help='运行模式')
    parser.add_argument('channel', choices=['l', 'r', 'b'],
                        help='通道: l=左, r=右, b=双')
    parser.add_argument('target', nargs='?', type=float, default=500,
                        help='目标速度 (counts/s)，正值=前进')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'串口 (默认 {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE,
                        help=f'波特率 (默认 {DEFAULT_BAUDRATE})')
    parser.add_argument('--kp', type=float, default=0.02,
                        help='step 模式的 Kp 值 (默认 0.02)')
    parser.add_argument('--ki', type=float, default=0.0,
                        help='step 模式的 Ki 值 (默认 0)')
    parser.add_argument('--kd', type=float, default=0.0,
                        help='step 模式的 Kd 值 (默认 0)')

    if len(sys.argv) < 2:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()

    tuner = AutoTuner(args.port, args.baud)

    try:
        if args.mode == 'monitor':
            print(f'\n📡 实时监控 {args.channel} (Ctrl+C 退出)\n')
            print(f'{"时间":>8} | {"L速度":>10} | {"R速度":>10}')
            print('-' * 35)
            start = time.time()
            history_l = deque(maxlen=20)
            history_r = deque(maxlen=20)

            while True:
                sp_l, sp_r = tuner.stream.get_both_speeds()
                elapsed = time.time() - start
                history_l.append(sp_l if sp_l else 0)
                history_r.append(sp_r if sp_r else 0)

                avg_l = statistics.mean([abs(x) for x in history_l]) if history_l else 0
                avg_r = statistics.mean([abs(x) for x in history_r]) if history_r else 0

                print(f'{elapsed:8.1f} | {sp_l if sp_l else 0:10.1f} | {sp_r if sp_r else 0:10.1f}  '
                      f'avg_L={avg_l:.0f} avg_R={avg_r:.0f}')

                time.sleep(0.3)

        elif args.mode == 'step':
            print(f'\n🔍 单步测试: ch={args.channel}, target={args.target}, kp={args.kp}')
            result = tuner._step_test(args.channel, args.target, args.kp, args.ki, args.kd)
            tuner.analyzer.print_result(result, kp=args.kp, ki=args.ki, kd=args.kd)

        elif args.mode == 'kp_scan':
            kp_list = [0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50]
            tuner.tune_kp(args.channel, args.target, kp_list)

        elif args.mode == 'tune':
            if args.channel == 'b':
                print('\n' + '=' * 60)
                print('🔄 双通道全自动调参')
                print('=' * 60)
                l_result = tuner.tune_all('l', args.target)
                r_result = tuner.tune_all('r', args.target)
                print('\n' + '=' * 60)
                print('📋 双通道调参结果汇总:')
                print('=' * 60)
                if l_result:
                    print(f'  左轮: Kp={l_result["kp"]:.3f} Ki={l_result["ki"]:.3f} Kd={l_result["kd"]:.3f}')
                if r_result:
                    print(f'  右轮: Kp={r_result["kp"]:.3f} Ki={r_result["ki"]:.3f} Kd={r_result["kd"]:.3f}')
            else:
                tuner.tune_all(args.channel, args.target)

    except KeyboardInterrupt:
        print('\n\n⏹️  已停止')
    finally:
        tuner.close()


if __name__ == '__main__':
    main()