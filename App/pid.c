#include "pid.h"
#include "uart.h"
#include <math.h>

/* 全局速度控制器实例 */
Speed_Control_t speed_left;
Speed_Control_t speed_right;

/* ================ PID 基础函数 ================ */

void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float output_min, float output_max, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    /* 动态积分限幅：确保积分项单独不会超过输出范围 */
    /* integral_limit = output_max / ki * 0.8（安全系数） */
    if (ki > 0.001f) {
        pid->integral_limit = output_max / ki * 0.8f;
    } else {
        pid->integral_limit = integral_limit;  /* ki 很小时使用用户指定值 */
    }
}

void pid_reset(PID_Controller_t *pid)
{
    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float pid_update(PID_Controller_t *pid, float feedback, float dt)
{
    float error = pid->setpoint - feedback;
    float derivative;

    /* 积分计算（带限幅） */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    /* 微分计算（带防抖动） */
    if (dt > 0.0f) {
        derivative = (error - pid->last_error) / dt;
    } else {
        derivative = 0.0f;
    }

    /* PID 输出计算 */
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    /* 输出限幅 */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }

    /* 更新上次误差 */
    pid->last_error = error;

    return pid->output;
}

/* ================ 速度闭环控制 ================ */

void speed_control_init(Speed_Control_t *sc, Motor_Channel_t channel, 
                        volatile int32_t *encoder, 
                        float kp, float ki, float kd)
{
    sc->motor_channel = channel;
    sc->encoder_count = encoder;
    sc->last_count = *encoder;
    sc->speed = 0.0f;

    /* 初始化 PID：输出范围对应 PWM 占空比 -399 ~ 399 */
    pid_init(&sc->pid, kp, ki, kd, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY, 1000.0f);
}

/* 目标速度平滑过渡：每次更新最大变化量，防止突变引起积分饱和 */
#define SPEED_MAX_STEP_PER_UPDATE  (50.0f)

void speed_control_set(Speed_Control_t *sc, float target_speed)
{
    /* 参数校验 */
    if (sc == NULL) return;
    
    float delta = target_speed - sc->pid.setpoint;
    if (delta > SPEED_MAX_STEP_PER_UPDATE) {
        sc->pid.setpoint += SPEED_MAX_STEP_PER_UPDATE;
    } else if (delta < -SPEED_MAX_STEP_PER_UPDATE) {
        sc->pid.setpoint -= SPEED_MAX_STEP_PER_UPDATE;
    } else {
        sc->pid.setpoint = target_speed;
    }
}

/* 死区速度阈值：低于此速度视为停止 */
#define SPEED_DEADZONE_THRESHOLD  (5.0f)
/* 积分分离阈值：误差大于此值时衰减积分 */
#define SPEED_INTEGRAL_SEPARATION  (30.0f)
/* 极性保护阈值：目标速度低于此值时检查极性 */
#define SPEED_POLARITY_PROTECT     (1.0f)

void speed_control_update(Speed_Control_t *sc, float dt)
{
    /* 参数校验 */
    if (sc == NULL || sc->encoder_count == NULL) return;
    if (dt <= 0.0f) return;

    int32_t current_count = *sc->encoder_count;
    int32_t delta_count = current_count - sc->last_count;

    /* 计算当前速度（脉冲/秒） */
    sc->speed = (float)delta_count / dt;

    /* 死区处理：目标速度为 0 且当前速度接近 0 时，停止电机避免抖动 */
    if (fabsf(sc->pid.setpoint) < SPEED_POLARITY_PROTECT && 
        fabsf(sc->speed) < SPEED_DEADZONE_THRESHOLD) {
        sc->speed = 0.0f;
        sc->pid.integral = 0.0f;  /* 清除积分避免启动冲击 */
        motor_set_speed(sc->motor_channel, 0);
        sc->last_count = current_count;
        return;
    }

    /* 积分分离：当误差过大时衰减积分，防止启动/突变时积分饱和 */
    float error = sc->pid.setpoint - sc->speed;
    if (fabsf(error) > SPEED_INTEGRAL_SEPARATION) {
        /* 误差过大时，逐步衰减积分项（每次衰减 5%） */
        sc->pid.integral *= 0.95f;
    }

    /* 更新 PID 控制器 */
    float output = pid_update(&sc->pid, sc->speed, dt);

    /* 最终输出安全检查：类型安全转换到 int16_t，并确保不超过 PWM 最大值 */
    int16_t final_output;
    
    /* 极性反转保护：当目标速度接近 0 且输出极性与目标相反时，强制归零 */
    if (fabsf(sc->pid.setpoint) < SPEED_POLARITY_PROTECT && 
        ((sc->pid.setpoint >= 0 && output < 0) || 
         (sc->pid.setpoint <= 0 && output > 0))) {
        final_output = 0;
        pid_reset(&sc->pid);
    } else {
        /* 安全类型转换：限幅后再转换，防止溢出 */
        if (output > (float)MOTOR_PWM_MAX_DUTY) {
            final_output = MOTOR_PWM_MAX_DUTY;
        } else if (output < -(float)MOTOR_PWM_MAX_DUTY) {
            final_output = -(int16_t)MOTOR_PWM_MAX_DUTY;
        } else {
            final_output = (int16_t)output;
        }
    }

    /* 设置电机速度 */
    motor_set_speed(sc->motor_channel, final_output);

    /* 更新上次计数值 */
    sc->last_count = current_count;
}

void speed_control_stop(Speed_Control_t *sc)
{
    /* 参数校验 */
    if (sc == NULL) return;
    
    pid_reset(&sc->pid);
    sc->speed = 0.0f;
    motor_set_speed(sc->motor_channel, 0);
}

/* ================ 串口调参接口 ================ */

/**
 * @brief 设置速度闭环 PID 参数
 * @param channel 电机通道 (PID_CH_LEFT / PID_CH_RIGHT / PID_CH_BOTH)
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
/* 计算动态积分限幅：确保积分项单独不会超过输出范围 */
/* integral_limit = output_max / ki * 0.8（安全系数） */
static float calc_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f) {
        return output_max / ki * 0.8f;
    }
    return 1000.0f;  /* ki 很小时使用默认值 */
}

void speed_pid_set_param(PID_Channel_t channel, float kp, float ki, float kd)
{
    /* 参数范围校验 */
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;
    
    float output_max = (float)MOTOR_PWM_MAX_DUTY;
    float ilim = calc_integral_limit(ki, output_max);
    
    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_left.pid.kp = kp;
        speed_left.pid.ki = ki;
        speed_left.pid.kd = kd;
        speed_left.pid.integral_limit = ilim;
        pid_reset(&speed_left.pid);
        uart_printf(UART0, "PID L: kp=%.3f ki=%.3f kd=%.3f ilim=%.1f\r\n", 
                    kp, ki, kd, ilim);
    }
    
    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_right.pid.kp = kp;
        speed_right.pid.ki = ki;
        speed_right.pid.kd = kd;
        speed_right.pid.integral_limit = ilim;
        pid_reset(&speed_right.pid);
        uart_printf(UART0, "PID R: kp=%.3f ki=%.3f kd=%.3f ilim=%.1f\r\n", 
                    kp, ki, kd, ilim);
    }
}

/**
 * @brief 获取速度闭环 PID 参数
 * @param channel 电机通道
 * @param kp 输出比例系数
 * @param ki 输出积分系数
 * @param kd 输出微分系数
 */
void speed_pid_get_param(PID_Channel_t channel, float *kp, float *ki, float *kd)
{
    if (channel == PID_CH_LEFT) {
        *kp = speed_left.pid.kp;
        *ki = speed_left.pid.ki;
        *kd = speed_left.pid.kd;
        uart_printf(UART0, "PID L: kp=%.3f ki=%.3f kd=%.3f\r\n", *kp, *ki, *kd);
    } else if (channel == PID_CH_RIGHT) {
        *kp = speed_right.pid.kp;
        *ki = speed_right.pid.ki;
        *kd = speed_right.pid.kd;
        uart_printf(UART0, "PID R: kp=%.3f ki=%.3f kd=%.3f\r\n", *kp, *ki, *kd);
    } else {
        /* PID_CH_BOTH: 同时打印左右参数 */
        uart_printf(UART0, "PID L: kp=%.3f ki=%.3f kd=%.3f\r\n", 
                    speed_left.pid.kp, speed_left.pid.ki, speed_left.pid.kd);
        uart_printf(UART0, "PID R: kp=%.3f ki=%.3f kd=%.3f\r\n", 
                    speed_right.pid.kp, speed_right.pid.ki, speed_right.pid.kd);
        if (kp) *kp = speed_left.pid.kp;
        if (ki) *ki = speed_left.pid.ki;
        if (kd) *kd = speed_left.pid.kd;
    }
}

/**
 * @brief 设置速度闭环目标速度
 * @param channel 电机通道
 * @param target_speed 目标速度（脉冲/秒）
 */
void speed_pid_set_target(PID_Channel_t channel, float target_speed)
{
    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_control_set(&speed_left, target_speed);
        uart_printf(UART0, "Target L: %.1f\r\n", target_speed);
    }
    
    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_control_set(&speed_right, target_speed);
        uart_printf(UART0, "Target R: %.1f\r\n", target_speed);
    }
}

/**
 * @brief 获取当前电机速度
 * @param channel 电机通道
 * @return float 当前速度（脉冲/秒）
 */
float speed_pid_get_speed(PID_Channel_t channel)
{
    float speed = 0.0f;
    
    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed = speed_left.speed;
        uart_printf(UART0, "Speed L: %.1f (target: %.1f)\r\n", 
                    speed, speed_left.pid.setpoint);
    }
    
    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed = speed_right.speed;
        uart_printf(UART0, "Speed R: %.1f (target: %.1f)\r\n", 
                    speed, speed_right.pid.setpoint);
    }
    
    return speed;
}

/* ================ 应用层初始化和更新 ================ */

void pid_app_init(void)
{
    /* 初始化速度控制器 */
    /* 参数需根据实际硬件调试：
     * kp: 比例系数，影响响应速度
     * ki: 积分系数，消除稳态误差
     * kd: 微分系数，抑制震荡
     */
    speed_control_init(&speed_left, MOTOR_LEFT, &encoder_left_count, 
                       0.5f, 0.0f, 0.0f);
    speed_control_init(&speed_right, MOTOR_RIGHT, &encoder_right_count, 
                       0.5f, 0.0f, 0.0f);
    
    uart_printf(UART0, "Speed PID init: L(kp=0.5 ki=0 kd=0) R(kp=0.5 ki=0 kd=0)\r\n");
}

void pid_app_update(uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;

    /* 更新速度控制 */
    speed_control_update(&speed_left, dt);
    speed_control_update(&speed_right, dt);
}
