# Encoder & PID 解耦重构 - Verification Checklist

## 编码器层验证
- [x] Checkpoint 1: `encoder_sample_left()` 读取增量后清零计数器，返回值为正增量（前进方向）
- [x] Checkpoint 2: `encoder_sample_right()` 行为与左轮对称
- [x] Checkpoint 3: `encoder_reset_all()` 同时清零左右计数器
- [x] Checkpoint 4: `encoder_get_speed()` 已从 .h 和 .c 中删除
- [x] Checkpoint 5: ISR 正交解码逻辑保持不变
- [x] Checkpoint 6: encoder.c 代码行数 ≤ 80 行（不含 ISR 函数体）

## PID 层验证
- [x] Checkpoint 7: `PID_t` 结构体字段仅包含 kp/ki/kd/setpoint/integral/last_error/out_min/out_max/integral_max
- [x] Checkpoint 8: `MotorSpeed_t` 不再包含 `volatile int32_t *enc` 指针和 `last_enc` 字段
- [x] Checkpoint 9: `pid_step()` 实现朴素 P+I+D，无条件积分抗饱和逻辑
- [x] Checkpoint 10: `speed_update()` 接受 delta 和 dt_ms 参数，不读编码器指针
- [x] Checkpoint 11: `speed_set_kp/ki/kd` 不再自动计算 integral_max
- [x] Checkpoint 12: pid.c 代码行数 ≤ 200 行，每个函数 ≤ 30 行
- [x] Checkpoint 13: pid.c 不再 #include "encoder.h"

## 主循环验证
- [x] Checkpoint 14: wheels.c 速度环分支先调 `encoder_sample_left/right()`，再调 `speed_update(delta, dt)`
- [x] Checkpoint 15: wheels.c 方向环分支（steer_flag）已恢复启用
- [x] Checkpoint 16: 调试输出 D 串格式保持 `D: %5.1f, %5.1f, %d, %10d`
- [x] Checkpoint 17: wheels.c 不再直接依赖 encoder 累计计数器做差分计算

## UART 命令兼容验证
- [x] Checkpoint 18: `spid <ch> <kp> <ki> <kd>` 响应格式与旧版一致
- [x] Checkpoint 19: `skp/ski/skd <ch> <val>` 响应格式与旧版一致
- [x] Checkpoint 20: `gpid [ch]` 响应格式与旧版一致
- [x] Checkpoint 21: `starget <ch> <speed>` 响应格式与旧版一致
- [x] Checkpoint 22: `gspeed [ch]` 响应格式与旧版一致
- [x] Checkpoint 23: `stpid/stkp/stki/stkd/gtpid` 响应格式与旧版一致
- [x] Checkpoint 24: `sbase/sstop` 响应格式与旧版一致
- [x] Checkpoint 25: `debug_speed_only/debug_speed_off` 响应格式与旧版一致

## 编译验证
- [x] Checkpoint 26: CCS 编译零 error（VSCode 解析错误为 SDK 路径配置问题，非代码问题）
- [x] Checkpoint 27: CCS 编译零 warning
- [x] Checkpoint 28: 无残留旧符号引用（Speed_Control_t, Steer_Control_t, speed_control_*, steer_pid_*, PID_Controller_t, PID_Channel_t, encoder_get_speed）

## 功能验证（烧录后）
- [ ] Checkpoint 29: 手动向前转动左轮 → D 串中 encoder delta 为正，speed 为正
- [ ] Checkpoint 30: `starget l 500` → 小车左轮前进，速度收敛到 500 附近
- [ ] Checkpoint 31: `starget r -500` → 小车右轮后退
- [ ] Checkpoint 32: 方向环启用后（debug_speed_off + sbase），小车能自动循迹
