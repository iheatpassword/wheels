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

/* 辅助函数：计算绝对值（安全类型转换） */
static inline uint16_t motor_abs(int16_t val)
{
    return (val >= 0) ? (uint16_t)val : (uint16_t)(-val);
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
    static int16_t l_speed = 0, r_speed = 0;
    static uint32_t last_time = 0;
    static int8_t step = 20;
    
    motor_set_speed_both(l_speed, r_speed);
    if (millis() - last_time >= 1000) {
        l_speed += step;
        r_speed -= step;
        if (motor_abs(l_speed) >= MOTOR_PWM_MAX_DUTY || 
            motor_abs(r_speed) >= MOTOR_PWM_MAX_DUTY) {
            step = -step;
        }
        last_time = millis();
    }
}

/* ==================== 方向反转测试 ==================== */
#include "App/pid.h"
#include "clock.h"

static volatile uint8_t dir_test_active = 0;
static volatile float   dir_test_target = 0;
static volatile uint32_t dir_test_last_switch = 0;
static volatile uint8_t dir_test_polarity = 1;  /* 1=正向, -1=反向 */

/* 启动反转测试 */
void motor_dir_test_start(float speed)
{
    dir_test_target = speed;
    dir_test_polarity = 1;
    dir_test_last_switch = tick_ms;
    dir_test_active = 1;
    
    /* 设置初始目标速度 */
    speed_set_target(&g_spd_left,  speed);
    speed_set_target(&g_spd_right, speed);
    
    /* 打印测试开始信息 */
    uart_printf(UART0, "\r\n=== Motor Direction Test ===\r\n");
    uart_printf(UART0, "Target speed: %.0f counts/s\r\n", speed);
    uart_printf(UART0, "Switch interval: 2000ms\r\n");
    uart_printf(UART0, "Press 'mdtest_stop' to stop\r\n\r\n");
}

/* 停止反转测试 */
void motor_dir_test_stop(void)
{
    dir_test_active = 0;
    dir_test_target = 0;
    dir_test_polarity = 1;
    
    /* 停止电机并清除 PID */
    speed_stop(&g_spd_left);
    speed_stop(&g_spd_right);
    
    uart_printf(UART0, "\r\n=== Direction Test Stopped ===\r\n\r\n");
}

/* 更新反转测试（在速度环中调用） */
void motor_dir_test_update(void)
{
    if (!dir_test_active) return;
    
    uint32_t now = tick_ms;
    if (now - dir_test_last_switch >= 2000) {
        dir_test_polarity = -dir_test_polarity;
        dir_test_last_switch = now;
        
        float new_target = dir_test_target * dir_test_polarity;
        speed_set_target(&g_spd_left,  new_target);
        speed_set_target(&g_spd_right, new_target);
        
        /* 打印反转瞬间的信息 */
        uart_printf(UART0, "SWITCH: polarity=%d new_target=%+.0f speed=%.0f delta=%ld pwm=%.0f\r\n",
                    dir_test_polarity,
                    new_target,
                    g_spd_left.speed,
                    (long)g_spd_left.last_delta,
                    g_spd_left.last_out);
    }
}