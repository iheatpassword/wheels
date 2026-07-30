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
    MPU6050_Init();
    OLED_Init();

    Interrupt_Init();
    OLED_ShowString(0,0,(uint8_t *)"initing...",8);    
    wait_for_mpu6050();

    /* 初始化速度闭环 PID 控制（左右电机） */
    pid_app_init();
    /* 初始化循迹模块（加权偏差模式） */
    patrol_init();
    OLED_Clear();
    /* 初始化计时器（OLED 显示计时） - 必须在 OLED_Clear 之后 */
    timer_init();
    
    while (1) {
        led_test();

        /* 处理串口命令 */
        uart_cmd_process();

        /* 100ms 周期：OLED 计时器刷新 */
        if (oled_flag)
        {
            oled_flag = 0;
            timer_update();
        }

        /* 20ms 周期 (50Hz): 方向环
         * 读取循迹传感器 → 计算转向 PID → 设置左右轮目标速度
         * 方向环独立于速度环运行，提供目标速度给速度环跟踪
         */
        // if (steer_flag)
        // {
        //     steer_flag = 0;

        //     if (!debug_speed_only)
        //     {
        //         float error;
        //         PatrolStatus_t st = patrol_get_error(&error);

        //         /* 边界保护：丢线/路口停车 */
        //         if (st == PATROL_LOST || st == PATROL_JUNCTION)
        //         {
        //             steer_stop();
        //         }
        //         else
        //         {
        //             steer_step(error, 20);
        //         }
        //     }
        // }

        /* 10ms 周期 (100Hz): 速度环
         * 采样编码器增量 → 速度 PID → PWM 输出
         */
        if (speed_flag)
        {
            speed_flag = 0;
            int32_t dl = encoder_sample_left();
            int32_t dr = encoder_sample_right();
            speed_update(&g_spd_left,  dl,  10.0f);
            speed_update(&g_spd_right, dr, 10.0f);
        }

        /* 100ms 周期：调试输出
         * 注意：auto_tune.py 解析 "D: %5.1f, %5.1f, %d, %10d" 格式
         *   字段含义：[0] 左目标速度, [1] 左实测速度, [2] 左 PID 输出(PWM), [3] 左编码器增量 */
        if (encoder_flag)
        {
            encoder_flag = 0;
            float spd = g_spd_left.speed;
            float err = g_spd_left.pid.setpoint - spd;
            float derr = (err - g_spd_left.pid.last_error) / 0.01f;
            float out = g_spd_left.pid.kp * err
                      + g_spd_left.pid.ki * g_spd_left.pid.integral
                      + g_spd_left.pid.kd * derr;
            if (out >  (float)MOTOR_PWM_MAX_DUTY) out =  MOTOR_PWM_MAX_DUTY;
            if (out < -(float)MOTOR_PWM_MAX_DUTY) out = -MOTOR_PWM_MAX_DUTY;

            uart_printf(UART0, "D: %5.1f, %5.1f, %d, %10d\r\n",
                        g_spd_left.pid.setpoint, spd,
                        (int)out, (int)g_spd_left.last_delta);
        }
    }
}
