#include "pid.h"

/* 全局速度控制器实例 */
Speed_Control_t speed_left;
Speed_Control_t speed_right;

/* 全局舵机控制器实例 */
Servo_Control_t servo_pitch;
Servo_Control_t servo_roll;

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
    pid->integral_limit = integral_limit;
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

/* ================ PD 基础函数（用于舵机） ================ */

void pd_init(PD_Controller_t *pd, float kp, float kd, 
             float output_min, float output_max)
{
    pd->kp = kp;
    pd->kd = kd;
    pd->setpoint = 0.0f;
    pd->output = 0.0f;
    pd->last_error = 0.0f;
    pd->output_min = output_min;
    pd->output_max = output_max;
}

void pd_reset(PD_Controller_t *pd)
{
    pd->output = 0.0f;
    pd->last_error = 0.0f;
}

float pd_update(PD_Controller_t *pd, float feedback, float dt)
{
    float error = pd->setpoint - feedback;
    float derivative;

    /* 微分计算 */
    if (dt > 0.0f) {
        derivative = (error - pd->last_error) / dt;
    } else {
        derivative = 0.0f;
    }

    /* PD 输出计算 */
    pd->output = pd->kp * error + pd->kd * derivative;

    /* 输出限幅 */
    if (pd->output > pd->output_max) {
        pd->output = pd->output_max;
    } else if (pd->output < pd->output_min) {
        pd->output = pd->output_min;
    }

    /* 更新上次误差 */
    pd->last_error = error;

    return pd->output;
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

    /* 初始化 PID：输出范围对应 PWM 占空比 -124 ~ 124 */
    pid_init(&sc->pid, kp, ki, kd, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY, 1000.0f);
}

void speed_control_set(Speed_Control_t *sc, float target_speed)
{
    sc->pid.setpoint = target_speed;
}

void speed_control_update(Speed_Control_t *sc, float dt)
{
    int32_t current_count = *sc->encoder_count;
    int32_t delta_count = current_count - sc->last_count;

    /* 计算当前速度（脉冲/秒） */
    if (dt > 0.0f) {
        sc->speed = (float)delta_count / dt;
    }

    /* 更新 PID 控制器 */
    float output = pid_update(&sc->pid, sc->speed, dt);

    /* 设置电机速度 */
    motor_set_speed(sc->motor_channel, (int16_t)output);

    /* 更新上次计数值 */
    sc->last_count = current_count;
}

void speed_control_stop(Speed_Control_t *sc)
{
    pid_reset(&sc->pid);
    sc->speed = 0.0f;
    motor_set_speed(sc->motor_channel, 0);
}

/* ================ 舵机角度 PD 控制 ================ */

void servo_angle_init(Servo_Control_t *sc, float *angle, 
                      float kp, float kd)
{
    sc->angle = angle;

    /* 初始化 PD：输出范围作为修正值，限制在 ±30 CCR 范围内 */
    /* 过大的修正值可能导致舵机剧烈抖动 */
    pd_init(&sc->pd, kp, kd, -30.0f, 30.0f);
}

void servo_angle_set(Servo_Control_t *sc, float target_angle)
{
    sc->pd.setpoint = target_angle;
}

void servo_angle_update(Servo_Control_t *sc, float dt)
{
    /* 获取当前角度 */
    float current_angle = *sc->angle;

    /* 更新 PD 控制器 */
    float pd_output = pd_update(&sc->pd, current_angle, dt);

    /* 计算基准 CCR：将设定角度映射到舵机 CCR 值 */
    /* 角度 0° → CCR 45，角度 180° → CCR 225 */
    float base_ccr = SEVRO_PWM_MIN_DUTY + (sc->pd.setpoint / 180.0f) * (SEVRO_PWM_MAX_DUTY - SEVRO_PWM_MIN_DUTY);

    /* 最终输出 = 基准 CCR + PD 修正 */
    float output = base_ccr + pd_output;

    /* 输出限幅 */
    if (output > SEVRO_PWM_MAX_DUTY) {
        output = SEVRO_PWM_MAX_DUTY;
    } else if (output < SEVRO_PWM_MIN_DUTY) {
        output = SEVRO_PWM_MIN_DUTY;
    }

    /* 设置舵机角度 */
    servo_setting((uint16_t)output);
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
                       0.5f, 0.1f, 0.05f);
    speed_control_init(&speed_right, MOTOR_RIGHT, &encoder_right_count, 
                       0.5f, 0.1f, 0.05f);

    /* 初始化舵机角度控制器 */
    /* 参数需根据实际硬件调试：
     * kp: 比例系数，影响响应速度
     * kd: 微分系数，抑制震荡
     */
    servo_angle_init(&servo_pitch, &pitch, 1.0f, 0.1f);
    servo_angle_init(&servo_roll, &roll, 1.0f, 0.1f);
}

void pid_app_update(uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;

    /* 更新速度控制 */
    speed_control_update(&speed_left, dt);
    speed_control_update(&speed_right, dt);

    /* 更新舵机角度控制 */
    servo_angle_update(&servo_pitch, dt);
    servo_angle_update(&servo_roll, dt);
}
