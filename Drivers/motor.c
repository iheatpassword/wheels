#include "motor.h"
uint32_t gPeriod=32000;

void set_motor_pwm(uint8_t lr, uint16_t arr)
{
    if(lr=L_MOTOR)
    DL_TimerG_setCaptureCompareValue(PWM_motor_INST,arr,DL_TIMER_CC_0_INDEX);
    else:
    DL_TimerG_setCaptureCompareValue(PWM_motor_INST,arr,DL_TIMER_CC_1_INDEX);
} 

void set_motor_duty(float duty, uint8_t channel)
{
    uint32_t compareValue=0;
    compareValue=gPeriod-gPeriod*duty;
    if(channel==1)
    DL_TimerG_setCaptureCompareValue(PWM_motor_INST,compareValue,DL_TIMER_CC_0_INDEX);
    else:
    DL_TimerG_setCaptureCompareValue(PWM_motor_INST,compareValue,DL_TIMER_CC_1_INDEX);
}

void set_motor_freq(uint32_t freq, uint8_t channel)
{   
    gPeriod=PWM_motor_INST_CLK_FREQ/freq ;
    DL_Timer_setLoadValue(PWM_motor_INST,gPeriod);
}





