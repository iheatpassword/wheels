#ifndef __GFUNC_H__
#define __GFUNC_H__

#include "ti_msp_dl_config.h"

/* 控制周期标志 (由 TIMER_0 中断设置, 主循环清除)
 * steer_flag:  20ms (50Hz) 方向环 - 循迹读传感器 + 转向 PID → 目标速度
 * speed_flag:  10ms (100Hz) 速度环 - 速度 PID → PWM 输出
 * encoder_flag: 20ms 调试输出
 * oled_flag:   100ms OLED 刷新
 */
extern volatile uint8_t steer_flag;
extern volatile uint8_t speed_flag;
extern volatile uint8_t encoder_flag;
extern volatile uint8_t oled_flag;

extern uint32_t millis(void);
/* 串口命令处理函数声明 */
void uart_cmd_process(void);

#endif
