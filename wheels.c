/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "Drivers/key.h"
#include "Drivers/encoder.h"
#include "Drivers/uart.h"
#include "Drivers/gFunc.h"
#include "Drivers/motor.h"
#include "App/patrol.h"
#include "App/pid.h"
#include "App/timer.h"

#include "clock.h"
#include "interrupt.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"

/* 外部变量：调试模式标志（定义在 gFunc.c） */
extern volatile uint8_t debug_speed_only;

/* 简单巡线模式开关（定义在 patrol.c，由 simpletest 命令控制）
 *   0 = PID 方向环 steer_step（默认，调参后正式运行）
 *   1 = if-else 查表法 patrol_simple_run（Bring-Up 阶段验证极性/接线/安装，无需调参） */
extern volatile uint8_t patrol_simple_mode;

/* 外部变量：控制周期标志（定义在 gFunc.c）
 * steer_flag:   20ms (50Hz) 方向环 - 循迹读传感器 + 转向 PID
 * speed_flag:   10ms (100Hz) 速度环 - 速度 PID → PWM 输出
 * encoder_flag: 100ms 调试输出
 * oled_flag:   100ms OLED 刷新
 */

// expect phenomenon: if four-way patrol not detect black, led0 on
void patrol_test(void)
{
    uint8_t r2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R2_PIN) > 0);
    uint8_t r1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R1_PIN) > 0);
    uint8_t l2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L2_PIN) > 0);
    uint8_t l1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L1_PIN) > 0);

    if((r2 == 0) && (r1 == 0) && (l2 == 0) && (l1 == 0))
    {
        DL_GPIO_setPins(LED_PORT, LED_led0_PIN);
    }
    else
    {
        DL_GPIO_clearPins(LED_PORT, LED_led0_PIN);
    }
}
// expect phenomenon: led0 blink at 1 sec intervals
void led_test(void)
{
    const uint16_t delayTime=1000;
    static uint32_t last_time=0;
    if(millis()-last_time>delayTime)
    {
       DL_GPIO_togglePins(LED_PORT, LED_led0_PIN);
       last_time=millis();
    }
}
//expect phenomenon: beeper sounds at 1 sec intervals
void beeper_test(void)
{
    const uint16_t delayTime=1000;
    static uint32_t last_time=0;
    if(millis()-last_time>delayTime)
    {
        // DL_GPIO_togglePins(Beeper_PORT, Beeper_PIN_0_PIN);
       DL_GPIO_setPins(Beeper_PORT, Beeper_PIN_0_PIN);
       DL_Common_delayCycles(CPUCLK_FREQ/500);
       DL_GPIO_clearPins(Beeper_PORT, Beeper_PIN_0_PIN);
       last_time=millis();
    }
}
void beeper_blink(void)
{
    DL_GPIO_setPins(Beeper_PORT, Beeper_PIN_0_PIN);
    DL_Common_delayCycles(CPUCLK_FREQ/500);
    DL_GPIO_clearPins(Beeper_PORT, Beeper_PIN_0_PIN);
}
void wait_for_mpu6050(void)
{
    DL_Common_delayCycles(CPUCLK_FREQ*3);
    beeper_blink();
}



int main(void)
{    
    SYSCFG_DL_init();
    SysTick_Init();

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    DL_TimerG_startCounter(TIMER_0_INST);

    motor_init();
    encoder_init();
    // MPU6050_Init();
    // OLED_Init();

    Interrupt_Init();
    // OLED_ShowString(0,0,(uint8_t *)"initing...",8);    
    // wait_for_mpu6050();

    // /* 初始化速度闭环 PID 控制（左右电机） */
    pid_app_init();
    // /* 初始化循迹模块（加权偏差模式） */
    patrol_init();
    // OLED_Clear();
    // /* 初始化计时器（OLED 显示计时） - 必须在 OLED_Clear 之后 */
    // timer_init();
    

    while (1) {
        led_test();
        /* 处理串口命令 */
        uart_cmd_process();

        /* 100ms 周期：OLED 计时器刷新 */
        // if (oled_flag)
        // {
        //     oled_flag = 0;
        //     timer_update();
        // }

        /* 20ms 周期 (50Hz): 方向环
         * 读取循迹传感器 → 计算转向 PID → 设置左右轮目标速度
         * 方向环独立于速度环运行，提供目标速度给速度环跟踪
         *
         * 两种模式由 patrol_simple_mode（simpletest 命令）二选一：
         *   mode=0（默认）：patrol_get_error 加权偏差 → steer_step(PID方向环)
         *   mode=1（简单） ：patrol_simple_run 直接查表给左右轮目标（无需调参，先验证极性）
         */
        if (steer_flag)
        {
            steer_flag = 0;

            if (!debug_speed_only)
            {
                if (patrol_simple_mode) {
                    /* ================ 简单巡线模式（if-else 查表，无PID） ================
                     * 不经过 patrol_get_error / steer_step，
                     * patrol_simple_run 内部自己读传感器、按 nibble 查表分配速度、
                     * 调用 speed_set_target，同时同步 steer.dbg 快照和状态。
                     * 丢线/路口直接在 patrol_simple_run 内处理为停车 (0,0)。
                     * ================================================================ */
                    float base = g_steer.base_speed;
                    if (base < 1.0f) base = 1500.0f;   /* 忘记 sbase 时给个默认起步速度 */
                    patrol_simple_run(base);
                } else {
                    /* ================ PID 方向环模式（默认） ================ */
                    float error;
                    uint8_t r2, r1, l1, l2;
                    PatrolStatus_t st = patrol_get_error(&error);

                    /* 把传感器原始状态送入方向环调试快照（便于 gsteer / S: 调试串查看） */
                    patrol_get_raw(&r2, &r1, &l1, &l2);
                    steer_update_sensors(r2, r1, l1, l2);

                    /* 边界保护：丢线/路口停车 */
                    if (st == PATROL_LOST || st == PATROL_JUNCTION)
                    {
                        if (st == PATROL_LOST)     steer_set_state(STEER_LOST);
                        else                       steer_set_state(STEER_JUNCTION);
                        steer_stop();
                    }
                    else
                    {
                        steer_step(error, 20);
                    }
                }
            }
        }

        /* 10ms 周期 (100Hz): 速度环 + 调试输出
         * 采样编码器增量（打时间戳）→ 真实 dt 算速度 → PID → PWM → 调试串口
         * 波特率 115200，100Hz 下 D 串输出约 2.6ms，可接受
         * dt=0 表示首次采样，无基准，跳过本次 PID
         */
        if (speed_flag)
        {
            speed_flag = 0;
            float dtl = 0.0f, dtr = 0.0f;
            int32_t dl = encoder_sample_left(&dtl);
            int32_t dr = encoder_sample_right(&dtr);
            
            /* dt=0 表示首次采样，无基准，跳过本次 PID */
            if (dtl > 0.0f) {
                speed_update(&g_spd_left,  dl, dtl);
            }
            if (dtr > 0.0f) {
                speed_update(&g_spd_right, dr, dtr);
            }

            /* 文本调试输出：auto_tune.py 解析 "D: %5.1f, %5.1f, %d, %10d" 格式
             * 字段：[0]目标速度 [1]实测速度 [2]PID输出(PWM) [3]编码器增量
             * DL 左轮，DR 右轮；保留原 "D:" 前缀兼容旧脚本（内容与 DL 相同） */
            // uart_printf(UART0, "DL: %5.1f, %5.1f, %d, %10d\r\n",
            //             g_spd_left.pid.setpoint, g_spd_left.speed,
            //             (int)g_spd_left.last_out, (int)g_spd_left.last_delta);
            // uart_printf(UART0, "DR: %5.1f, %5.1f, %d, %10d\r\n",
            //             g_spd_right.pid.setpoint, g_spd_right.speed,
            //             (int)g_spd_right.last_out, (int)g_spd_right.last_delta);
        }
    }
}
