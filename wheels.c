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

#include "clock.h"
#include "interrupt.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"

void timer_test(void)
{
    static uint8_t i=0;
    //static uint32_t last_time=0;
    if(millis()%1000==0)//print every 1s
    {
        i++;
        uart_printf(UART0, "i=%d\r\n", i);
        uart_printf(UART0, "millis=%d\r\n",millis());
        //last_time=millis();
    }
}
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
    OLED_Clear();

    // motor_set_speed_both(-20, 90);

    uint32_t encoder_print_count = 0;
    
    while (1) {
        led_test();
        // motor_test();

        /* 处理串口命令 */
        uart_cmd_process();
        
        /* 控制周期：10ms 中断触发 */
        if (read_patrol)
        {
            read_patrol = 0;
            
            /* 读取编码器速度 (counts/s) */
            int32_t l_speed, r_speed;
            encoder_get_speed(&l_speed, &r_speed);
            
            /* 调试输出：每 100ms 输出一次 */
            encoder_print_count++;
            if (encoder_print_count >= 10)
            {
                uart_printf(UART0, "enc: %6d, %6d\n",l_speed, r_speed);
                encoder_print_count=0;
            }

        }

        // uart_printf(UART0, "pitch=%5.1f\r\n",pitch);
        // uart_printf(UART0, "roll=%5.1f\r\n",roll);
        // uart_printf(UART0, "yaw=%5.1f\r\n",yaw);


    }
}
