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
    /* 速度环初始化 */
    speed_control_init(&speed_left, MOTOR_LEFT, &encoder_left_count,
                       0.346f, 0.0f, 0.0f);
    speed_control_init(&speed_right, MOTOR_RIGHT, &encoder_right_count,
                       0.346f, 0.0f, 0.0f);

    /* 极性配置（已确认）：
     * 左轮：motor_set_speed(负值) → 向前；向前时 encoder_delta < 0（读数负）
     *       polarity = -1：speed = delta_count * (-1) = 正值
     *       这样 setpoint=+正值 对应向前
     * 右轮：motor_set_speed(正值) → 向前；向前时 encoder_delta > 0（读数正）
     *       polarity = +1：speed = delta_count * (+1) = 正值
     *       这样 setpoint=+正值 对应向前
     * 结论：左右轮向前时 setpoint 都为正值 */
    speed_left.polarity  = -1.0f;
    speed_right.polarity = +1.0f;

    /* 转向环初始化（保守参数，安全起步）
     * kp=20: 每度误差输出 20 脉冲/秒差速（例：误差 10° → 差速 200）
     * ki=0, kd=0: 先只用 P 项，防止震荡
     * max_turn=2000: 最大差速 2000 脉冲/秒（约满转的 25%） */
    steer_pid_init(20.0f, 0.0f, 0.0f, 2000.0f);

    uart_printf(UART0, "Speed PID init: L(kp=0.346 ki=0 kd=0) R(kp=0.346 ki=0 kd=0)\r\n");
    uart_printf(UART0, "Steer PID init: kp=20 ki=0 kd=0  max_turn=2000\r\n");
}

void pid_app_update(uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;
    speed_control_update(&speed_left, dt);
    speed_control_update(&speed_right, dt);
}

/* ================ 转向环（航向控制） ================ */

Steer_Control_t steer_control;

static float calc_steer_integral_limit(float ki, float output_max)
{
    if (ki > 0.001f)
        return output_max / ki * 0.8f;
    return 1000.0f;
}

/* 角度归一化：将误差限制在 (-180, 180]，避免走 350° 到 10° 绕大圈 */
static float normalize_angle_error(float error_deg)
{
    while (error_deg > 180.0f)  error_deg -= 360.0f;
    while (error_deg <= -180.0f) error_deg += 360.0f;
    return error_deg;
}

void steer_pid_init(float kp, float ki, float kd, float max_turn)
{
    steer_control.target_yaw = 0.0f;
    steer_control.current_yaw = 0.0f;
    steer_control.turn_output = 0.0f;
    steer_control.base_speed = 0.0f;
    steer_control.max_turn_output = max_turn;

    /* 航向 PID：输出 = 差速补偿量（脉冲/秒），限幅 ±max_turn */
    float output_max = max_turn;
    float ilim = calc_steer_integral_limit(ki, output_max);
    pid_init(&steer_control.pid, kp, ki, kd, -output_max, output_max, ilim);
}

void steer_pid_set_target_yaw(float yaw_deg)
{
    steer_control.target_yaw = yaw_deg;
}

void steer_pid_set_base_speed(float base_speed_counts)
{
    steer_control.base_speed = base_speed_counts;
}

void steer_pid_reset_yaw_zero(float current_yaw_deg)
{
    steer_control.current_yaw = current_yaw_deg;
    steer_control.target_yaw = current_yaw_deg;
    pid_reset(&steer_control.pid);
}

void steer_pid_adjust_yaw(float delta_deg)
{
    steer_control.target_yaw += delta_deg;
    /* 目标角度也做归一化，防止累积溢出 */
    while (steer_control.target_yaw > 180.0f)  steer_control.target_yaw -= 360.0f;
    while (steer_control.target_yaw <= -180.0f) steer_control.target_yaw += 360.0f;
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
    uart_printf(UART0, "Steer PID: kp=%5.3f ki=%5.3f kd=%5.3f  target=%5.1fdeg  base=%5.1f\r\n",
                steer_control.pid.kp, steer_control.pid.ki, steer_control.pid.kd,
                steer_control.target_yaw, steer_control.base_speed);
}

/* 核心：差速合成
 * 规则：右转 → yaw 增加 → error 正 → turn_output 正
 *       turn_output 正 → 左轮更快，右轮更慢 → 车辆右转
 * 所以：
 *   target_left  = base_speed + turn_output
 *   target_right = base_speed - turn_output
 * （注意：左右轮向前时 setpoint 都为正值） */
void steer_pid_update(float current_yaw_deg, uint32_t dt_ms)
{
    steer_control.current_yaw = current_yaw_deg;

    /* 角度误差归一化（最短路径转向） */
    float raw_error = steer_control.target_yaw - current_yaw_deg;
    float error = normalize_angle_error(raw_error);

    /* PID 计算：输出 = 差速补偿（脉冲/秒） */
    float dt = (float)dt_ms / 1000.0f;
    /* 直接复用 pid_update 内部公式，将 error 先写入 setpoint 以便 pid_update 计算
     * 但 pid_update 以 feedback 为输入，我们直接传入 error 的负值不合适。
     * 更干净的办法：把 current_yaw 映射为 0-centered。这里手动计算。*/
    /* --- 手动展开最小 PID（避免改 global setpoint 污染其它逻辑） --- */
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

    /* 饱和处理：确保单轮不超过 ±MOTOR_PWM_MAX_DUTY 等效的目标速度
     * 注意：目标速度 counts/s 上限取决于电机满转的编码器读数，保守用 8000 限制 */
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
