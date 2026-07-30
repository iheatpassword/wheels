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

    /* 积分（带限幅） */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    /* 微分 */
    float derivative = (error - pid->last_error) / dt;

    /* PID 输出 */
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

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
    sc->polarity = +1.0f;  /* 默认不取反，由调用方按需设置 */

    /* 基础 PID：输出范围 -399 ~ 399，积分限幅 399 */
    pid_init(&sc->pid, kp, ki, kd, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
}

void speed_control_set(Speed_Control_t *sc, float target_speed)
{
    if (sc == NULL) return;
    sc->pid.setpoint = target_speed;
}

void speed_control_update(Speed_Control_t *sc, float dt)
{
    if (sc == NULL || sc->encoder_count == NULL) return;
    if (dt <= 0.0f) return;

    /* 读取编码器累计计数，计算速度（应用极性对齐编码器与电机方向） */
    int32_t current_count = *sc->encoder_count;
    int32_t delta_count = current_count - sc->last_count;
    sc->speed = (float)delta_count / dt * sc->polarity;

    /* 基础 PID 计算 */
    float output = pid_update(&sc->pid, sc->speed, dt);

    /* 输出到电机 */
    int16_t final_output;
    if (output > (float)MOTOR_PWM_MAX_DUTY)
        final_output = MOTOR_PWM_MAX_DUTY;
    else if (output < -(float)MOTOR_PWM_MAX_DUTY)
        final_output = -(int16_t)MOTOR_PWM_MAX_DUTY;
    else
        final_output = (int16_t)output;

    motor_set_speed(sc->motor_channel, final_output);
    sc->last_count = current_count;
}

void speed_control_stop(Speed_Control_t *sc)
{
    if (sc == NULL) return;
    pid_reset(&sc->pid);
    sc->speed = 0.0f;
    motor_set_speed(sc->motor_channel, 0);
}

/* ================ 串口调参接口 ================ */

static float calc_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f)
        return output_max / ki * 0.8f;
    return 1000.0f;
}

void speed_pid_set_param(PID_Channel_t channel, float kp, float ki, float kd)
{
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;

    float ilim = calc_integral_limit(ki, (float)MOTOR_PWM_MAX_DUTY);

    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_left.pid.kp = kp;
        speed_left.pid.ki = ki;
        speed_left.pid.kd = kd;
        speed_left.pid.integral_limit = ilim;
        pid_reset(&speed_left.pid);
        uart_printf(UART0, "OK L: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_right.pid.kp = kp;
        speed_right.pid.ki = ki;
        speed_right.pid.kd = kd;
        speed_right.pid.integral_limit = ilim;
        pid_reset(&speed_right.pid);
        uart_printf(UART0, "OK R: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }
}

void speed_pid_set_kp(PID_Channel_t channel, float kp)
{
    if (kp < 0.0f) kp = 0.0f;

    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_left.pid.kp = kp;
        pid_reset(&speed_left.pid);
        uart_printf(UART0, "OK L kp=%5.3f\r\n", kp);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_right.pid.kp = kp;
        pid_reset(&speed_right.pid);
        uart_printf(UART0, "OK R kp=%5.3f\r\n", kp);
    }
}

void speed_pid_set_ki(PID_Channel_t channel, float ki)
{
    if (ki < 0.0f) ki = 0.0f;

    float ilim = calc_integral_limit(ki, (float)MOTOR_PWM_MAX_DUTY);

    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_left.pid.ki = ki;
        speed_left.pid.integral_limit = ilim;
        pid_reset(&speed_left.pid);
        uart_printf(UART0, "OK L ki=%5.3f\r\n", ki);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_right.pid.ki = ki;
        speed_right.pid.integral_limit = ilim;
        pid_reset(&speed_right.pid);
        uart_printf(UART0, "OK R ki=%5.3f\r\n", ki);
    }
}

void speed_pid_set_kd(PID_Channel_t channel, float kd)
{
    if (kd < 0.0f) kd = 0.0f;

    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_left.pid.kd = kd;
        pid_reset(&speed_left.pid);
        uart_printf(UART0, "OK L kd=%5.3f\r\n", kd);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_right.pid.kd = kd;
        pid_reset(&speed_right.pid);
        uart_printf(UART0, "OK R kd=%5.3f\r\n", kd);
    }
}

void speed_pid_get_param(PID_Channel_t channel, float *kp, float *ki, float *kd)
{
    if (channel == PID_CH_LEFT) {
        *kp = speed_left.pid.kp;
        *ki = speed_left.pid.ki;
        *kd = speed_left.pid.kd;
        uart_printf(UART0, "PID L: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", *kp, *ki, *kd);
    } else if (channel == PID_CH_RIGHT) {
        *kp = speed_right.pid.kp;
        *ki = speed_right.pid.ki;
        *kd = speed_right.pid.kd;
        uart_printf(UART0, "PID R: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", *kp, *ki, *kd);
    } else {
        uart_printf(UART0, "PID L: kp=%5.3f ki=%5.3f kd=%5.3f\r\n",
                    speed_left.pid.kp, speed_left.pid.ki, speed_left.pid.kd);
        uart_printf(UART0, "PID R: kp=%5.3f ki=%5.3f kd=%5.3f\r\n",
                    speed_right.pid.kp, speed_right.pid.ki, speed_right.pid.kd);
        if (kp) *kp = speed_left.pid.kp;
        if (ki) *ki = speed_left.pid.ki;
        if (kd) *kd = speed_left.pid.kd;
    }
}

void speed_pid_set_target(PID_Channel_t channel, float target_speed)
{
    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed_control_set(&speed_left, target_speed);
        uart_printf(UART0, "OK L target=%5.1f\r\n", target_speed);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed_control_set(&speed_right, target_speed);
        uart_printf(UART0, "OK R target=%5.1f\r\n", target_speed);
    }
}

float speed_pid_get_speed(PID_Channel_t channel)
{
    float speed = 0.0f;

    if (channel == PID_CH_LEFT || channel == PID_CH_BOTH) {
        speed = speed_left.speed;
        uart_printf(UART0, "Speed L: %5.1f (target: %5.1f)\r\n",
                    speed, speed_left.pid.setpoint);
    }

    if (channel == PID_CH_RIGHT || channel == PID_CH_BOTH) {
        speed = speed_right.speed;
        uart_printf(UART0, "Speed R: %5.1f (target: %5.1f)\r\n",
                    speed, speed_right.pid.setpoint);
    }

    return speed;
}

void speed_pid_get_raw_speed(float *left_speed, float *right_speed)
{
    if (left_speed)  *left_speed  = speed_left.speed;
    if (right_speed) *right_speed = speed_right.speed;
}

/* ================ 应用层 ================ */

void pid_app_init(void)
{
    /* 速度环初始化
     * PWM/速度比例约 1:20，Kp=0.02 起步较保守
     * 正向目标 +500 → error*Kp = 500*0.02 = 10 PWM → 约 200 counts/s */
    speed_control_init(&speed_left, MOTOR_LEFT, &encoder_left_count,
                       0.02f, 0.0f, 0.0f);
    speed_control_init(&speed_right, MOTOR_RIGHT, &encoder_right_count,
                       0.02f, 0.0f, 0.0f);

    /* 极性配置：
     * 诊断确认：电机前进时 encoder_delta 为正（计数递增），
     * 因此左右轮都使用 polarity=+1.0，使 encoder 反馈与 setpoint 方向一致
     * 
     * PWM/速度比例：约 1:20（20 PWM ≈ 400 counts/s）
     * 因此 Kp 需很小（~0.02），避免 error*Kp 立即饱和输出 */
    speed_left.polarity  = +1.0f;
    speed_right.polarity = +1.0f;

    /* 转向环初始化（循迹偏差驱动，保守参数安全起步）
     * 偏差范围 [-3, +3]（加权平均后）
     * kp=400: 每单位偏差输出 400 脉冲/秒差速
     *         例：偏差 +3（车偏左）→ 差速 +1200 → 左轮快右轮慢 → 右转修正
     * ki=0, kd=0: 先只用 P 项，防止震荡
     * max_turn=2000: 最大差速 2000 脉冲/秒 */
    steer_pid_init(400.0f, 0.0f, 0.0f, 2000.0f);

    uart_printf(UART0, "Speed PID init: L(kp=0.020 ki=0 kd=0) R(kp=0.020 ki=0 kd=0)\r\n");
    uart_printf(UART0, "Steer PID init: kp=400 ki=0 kd=0  max_turn=2000 (patrol error mode)\r\n");
}

void pid_app_update(uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;
    speed_control_update(&speed_left, dt);
    speed_control_update(&speed_right, dt);
}

/* ================ 转向环（循迹偏差控制） ================ */

Steer_Control_t steer_control;

static float calc_steer_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f)
        return output_max / ki * 0.8f;
    return 1000.0f;
}

void steer_pid_init(float kp, float ki, float kd, float max_turn)
{
    steer_control.turn_output = 0.0f;
    steer_control.base_speed = 0.0f;
    steer_control.max_turn_output = max_turn;

    /* 转向 PID：输出 = 差速补偿量（脉冲/秒），限幅 ±max_turn
     * setpoint=0（目标是循迹偏差为0，即居中） */
    float output_max = max_turn;
    float ilim = calc_steer_integral_limit(ki, output_max);
    pid_init(&steer_control.pid, kp, ki, kd, -output_max, output_max, ilim);
    steer_control.pid.setpoint = 0.0f;
}

void steer_pid_set_base_speed(float base_speed_counts)
{
    steer_control.base_speed = base_speed_counts;
}

float steer_pid_get_base_speed(void)
{
    return steer_control.base_speed;
}

void steer_pid_stop(void)
{
    pid_reset(&steer_control.pid);
    steer_control.turn_output = 0.0f;
    steer_control.base_speed = 0.0f;
    speed_control_set(&speed_left, 0.0f);
    speed_control_set(&speed_right, 0.0f);
}

/* 转向环调参接口 */
void steer_pid_set_kp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    steer_control.pid.kp = kp;
    pid_reset(&steer_control.pid);
    uart_printf(UART0, "OK steer kp=%5.3f\r\n", kp);
}

void steer_pid_set_ki(float ki)
{
    if (ki < 0.0f) ki = 0.0f;
    steer_control.pid.ki = ki;
    steer_control.pid.integral_limit = calc_steer_integral_limit(ki, steer_control.max_turn_output);
    pid_reset(&steer_control.pid);
    uart_printf(UART0, "OK steer ki=%5.3f\r\n", ki);
}

void steer_pid_set_kd(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    steer_control.pid.kd = kd;
    pid_reset(&steer_control.pid);
    uart_printf(UART0, "OK steer kd=%5.3f\r\n", kd);
}

void steer_pid_set_param(float kp, float ki, float kd)
{
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;

    float ilim = calc_steer_integral_limit(ki, steer_control.max_turn_output);
    steer_control.pid.kp = kp;
    steer_control.pid.ki = ki;
    steer_control.pid.kd = kd;
    steer_control.pid.integral_limit = ilim;
    pid_reset(&steer_control.pid);
    uart_printf(UART0, "OK steer: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
}

void steer_pid_get_param(float *kp, float *ki, float *kd)
{
    if (kp) *kp = steer_control.pid.kp;
    if (ki) *ki = steer_control.pid.ki;
    if (kd) *kd = steer_control.pid.kd;
    uart_printf(UART0, "Steer PID: kp=%5.3f ki=%5.3f kd=%5.3f  base=%5.1f  max_turn=%5.1f\r\n",
                steer_control.pid.kp, steer_control.pid.ki, steer_control.pid.kd,
                steer_control.base_speed, steer_control.max_turn_output);
}

/* 核心：差速合成
 * 输入 position_error 来自循迹加权偏差：
 *   正值 = 车偏左（线在右侧）→ 需右转
 *   负值 = 车偏右（线在左侧）→ 需左转
 *
 * PID 输出 turn_output（脉冲/秒差速补偿）：
 *   error 正 → turn_output 正 → 左轮快、右轮慢 → 车辆右转 ✓
 *   error 负 → turn_output 负 → 左轮慢、右轮快 → 车辆左转 ✓
 *
 * 差速合成：
 *   target_left  = base_speed + turn_output
 *   target_right = base_speed - turn_output
 * （注意：左右轮向前时 setpoint 都为正值） */
void steer_pid_update(float position_error, uint32_t dt_ms)
{
    /* PID 计算：偏差直接作为误差（setpoint=0，目标是居中） */
    float error = -position_error;
    float dt = (float)dt_ms / 1000.0f;

    PID_Controller_t *pid = &steer_control.pid;
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    float derivative;
    if (dt > 0.0f)
        derivative = (error - pid->last_error) / dt;
    else
        derivative = 0.0f;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;
    pid->last_error = error;
    pid->output = output;

    steer_control.turn_output = output;

    /* 差速合成 → 左右轮目标速度 */
    float base = steer_control.base_speed;
    float turn = steer_control.turn_output;
    float target_left  = base + turn;
    float target_right = base - turn;

    /* 饱和限幅（保守上限 8000 脉冲/秒） */
    float speed_limit = 8000.0f;
    if (target_left >  speed_limit) target_left =  speed_limit;
    if (target_left < -speed_limit) target_left = -speed_limit;
    if (target_right >  speed_limit) target_right =  speed_limit;
    if (target_right < -speed_limit) target_right = -speed_limit;

    /* 输出到速度环 */
    speed_control_set(&speed_left, target_left);
    speed_control_set(&speed_right, target_right);
}

/* ================ 应用层初始化（更新：加入 steer_pid_init） ================ */
