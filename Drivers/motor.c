#include "motor.h"

/* ========== 内部辅助函数 ========== */

static void motor_set_pwm(uint8_t channel, uint16_t compareValue)
{
    if (compareValue > MOTOR_PWM_MAX_DUTY) {
        compareValue = MOTOR_PWM_MAX_DUTY;
    }

    if (channel == MOTOR_LEFT) {
        DL_TimerG_setCaptureCompareValue(
            PWM_motor_INST, compareValue, DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(
            PWM_motor_INST, compareValue, DL_TIMER_CC_1_INDEX);
    }
}

static void motor_set_dir_forward(uint8_t channel)
{
    if (channel == MOTOR_LEFT) {
        DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
        DL_GPIO_clearPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    } else {
        DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
        DL_GPIO_clearPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
    }
}

static void motor_set_dir_backward(uint8_t channel)
{
    if (channel == MOTOR_LEFT) {
        DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
        DL_GPIO_setPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
        DL_GPIO_setPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
    }
}

static uint16_t myabs(int16_t a)
{
    if(a>=0) return a;
    else return -a;
}

/* ========== 对外接口 ========== */

void motor_init(void)
{
    /* 方向引脚初始化为低电平（滑行停止状态） */
    DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
    DL_GPIO_clearPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
    DL_GPIO_clearPins(Motor_BIN2_PORT, Motor_BIN2_PIN);

    /* PWM 初始占空比为 0 */
    DL_TimerG_setCaptureCompareValue(
        PWM_motor_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(
        PWM_motor_INST, 0U, DL_TIMER_CC_1_INDEX);

    /* 启动 PWM 定时器 */
    DL_TimerG_startCounter(PWM_motor_INST);

    /* 退出待机模式 */
    DL_GPIO_setPins(Motor_STBY_PORT, Motor_STBY_PIN);
}

void motor_set_speed(Motor_Channel_t channel, int16_t speed)
{
    uint16_t absSpeed;

    if (speed >= 0) {
        absSpeed = (uint16_t)speed;
        motor_set_dir_forward(channel);
    } else {
        absSpeed = (uint16_t)(-speed);
        motor_set_dir_backward(channel);
    }

    if (absSpeed > MOTOR_PWM_MAX_DUTY) {
        absSpeed = MOTOR_PWM_MAX_DUTY;
    }

    motor_set_pwm(channel, absSpeed);
}

void motor_set_speed_both(int16_t leftSpeed, int16_t rightSpeed)
{
    motor_set_speed(MOTOR_LEFT, leftSpeed);
    motor_set_speed(MOTOR_RIGHT, rightSpeed);
}

void motor_stop(Motor_Channel_t channel, Motor_StopMode_t mode)
{
    if (mode == MOTOR_STOP_COAST) {
        /* 滑行停止：IN1=0, IN2=0 */
        if (channel == MOTOR_LEFT) {
            DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
            DL_GPIO_clearPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
        } else {
            DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
            DL_GPIO_clearPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
        }
        motor_set_pwm(channel, 0U);
    } else {
        /* 刹车停止：IN1=1, IN2=1（短路制动） */
        if (channel == MOTOR_LEFT) {
            DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
            DL_GPIO_setPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
        } else {
            DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
            DL_GPIO_setPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
        }
        motor_set_pwm(channel, MOTOR_PWM_MAX_DUTY);
    }
}

void motor_stop_both(Motor_StopMode_t mode)
{
    motor_stop(MOTOR_LEFT, mode);
    motor_stop(MOTOR_RIGHT, mode);
}

void motor_brake(Motor_Channel_t channel)
{
    motor_stop(channel, MOTOR_STOP_BRAKE);
}

void motor_brake_both(void)
{
    motor_stop_both(MOTOR_STOP_BRAKE);
}

void motor_standby(bool enable)
{
    if (enable) {
        /* 进入待机模式：STBY=0 */
        DL_GPIO_clearPins(Motor_STBY_PORT, Motor_STBY_PIN);
    } else {
        /* 退出待机模式：STBY=1 */
        DL_GPIO_setPins(Motor_STBY_PORT, Motor_STBY_PIN);
    }
}

void motor_test(void)
{
    static int16_t l_speed=0, r_speed=0;
    static uint32_t last_time=0;
    static int8_t step=100;
    motor_set_speed_both(l_speed,r_speed);
    if(millis()-last_time>=2000)
    {
        l_speed+=step;
        r_speed-=step;
        if(myabs(l_speed)>=MOTOR_PWM_MAX_DUTY||myabs(r_speed)>=MOTOR_PWM_MAX_DUTY)step=-step;
        last_time=millis();
    }

}