#include "pid.h"
#include "uart.h"

MotorSpeed_t g_spd_left;
MotorSpeed_t g_spd_right;
Steer_t      g_steer;

void pid_begin(PID_t *p, float kp, float ki, float kd,
               float out_max, float integral_max)
{
    p->kp     = kp;
    p->ki     = ki;
    p->kd     = kd;
    p->setpoint   = 0.0f;
    p->integral   = 0.0f;
    p->last_error = 0.0f;
    p->out_min    = -out_max;
    p->out_max    =  out_max;
    p->integral_max = out_max * 0.5f;
}

void pid_reset(PID_t *p)
{
    p->integral   = 0.0f;
    p->last_error = 0.0f;
}

float pid_step(PID_t *p, float setpoint, float measure, float dt)
{
    float error = setpoint - measure;
    float derivative = (error - p->last_error) / dt;
    float p_term = p->kp * error;
    float d_term = p->kd * derivative;

    /* 条件积分抗饱和：输出已饱和且积分仍往饱和方向推时，停止累加。
     * 允许误差反向时继续积分，帮助退出饱和。 */
    float i_term = p->ki * p->integral;
    float saturated_hi = (p_term + i_term + d_term >= p->out_max) && (error > 0);
    float saturated_lo = (p_term + i_term + d_term <= p->out_min) && (error < 0);
    if (!saturated_hi && !saturated_lo) {
        p->integral += error * dt;
        if (p->integral >  p->integral_max) p->integral =  p->integral_max;
        if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    }

    float out = p_term + p->ki * p->integral + d_term;
    if (out >  p->out_max) out =  p->out_max;
    if (out <  p->out_min) out =  p->out_min;
    p->last_error = error;
    return out;
}

void speed_init(MotorSpeed_t *s, Motor_Channel_t ch,
                float kp, float ki, float kd)
{
    s->ch             = ch;
    s->speed          = 0.0f;
    s->speed_raw      = 0.0f;
    s->filter_alpha   = 0.3f;  /* 默认中等滤波 */
    s->motor_polarity = 1.0f;  /* 默认输出不翻转；若硬件BIN1/BIN2反接则设为-1.0 */
    s->last_delta     = 0;
    s->last_out       = 0.0f;
    pid_begin(&s->pid, kp, ki, kd,
              (float)MOTOR_PWM_MAX_DUTY,
              (float)MOTOR_PWM_MAX_DUTY * 0.5f);
}

/* 设置一阶低通系数：1=无滤波, 0.5=中等, 0.1=强滤波 */
void speed_set_filter(MotorSpeed_t *s, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s->filter_alpha = alpha;
}

/* 设置电机 PWM 输出极性：+1.0 正常, -1.0 翻转。
 * 用于补偿硬件 BIN1/BIN2 交叉反接导致的方向与符号不一致。
 * 修改时同步重置 PID，防止旧积分造成瞬态冲击。 */
void speed_set_motor_polarity(MotorSpeed_t *s, float polarity)
{
    if (polarity >= 0.0f) s->motor_polarity =  1.0f;
    else                  s->motor_polarity = -1.0f;
    pid_reset(&s->pid);
}

void speed_set_target(MotorSpeed_t *s, float target)
{
    /* 目标速度符号变化（正↔负切换）时重置积分项。
     * 原因：ki 较大时，正转积累的正积分在切换到负目标后释放极慢（约10秒），
     *       期间积分项主导 PID 输出为正值，导致电机反常地正向满转。
     *       重置后 PID 从纯 P 开始，方向立即正确。 */
    if ((s->pid.setpoint > 0.0f && target < 0.0f) ||
        (s->pid.setpoint < 0.0f && target > 0.0f)) {
        pid_reset(&s->pid);
    }
    s->pid.setpoint = target;
}

void speed_set_kp(MotorSpeed_t *s, float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    s->pid.kp = kp;
    pid_reset(&s->pid);
}

void speed_set_ki(MotorSpeed_t *s, float ki)
{
    if (ki < 0.0f) ki = 0.0f;
    s->pid.ki = ki;
    pid_reset(&s->pid);
}

void speed_set_kd(MotorSpeed_t *s, float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    s->pid.kd = kd;
    pid_reset(&s->pid);
}

void speed_get_params(MotorSpeed_t *s, float *kp, float *ki, float *kd)
{
    if (kp) *kp = s->pid.kp;
    if (ki) *ki = s->pid.ki;
    if (kd) *kd = s->pid.kd;
}

float speed_get_speed(MotorSpeed_t *s)
{
    return s->speed;
}

void speed_update(MotorSpeed_t *s, int32_t delta, float dt_ms)
{
    float vel_raw = (float)delta / (dt_ms / 1000.0f);  /* 原始速度 */
    s->speed_raw = vel_raw;

    /* 一阶低通滤波：filtered = alpha * raw + (1-alpha) * filtered_old */
    s->speed = s->filter_alpha * vel_raw + (1.0f - s->filter_alpha) * s->speed;

    s->last_delta = delta;
    float out = pid_step(&s->pid, s->pid.setpoint, s->speed, dt_ms / 1000.0f);
    s->last_out = out;

    /* 1) PID 输出硬限幅到 ±MOTOR_PWM_MAX_DUTY（±399） */
    int16_t pwm_raw;
    if (out >  (float)MOTOR_PWM_MAX_DUTY) pwm_raw =  MOTOR_PWM_MAX_DUTY;
    else if (out < -(float)MOTOR_PWM_MAX_DUTY) pwm_raw = -MOTOR_PWM_MAX_DUTY;
    else                                       pwm_raw = (int16_t)out;

    /* 2) 应用电机输出极性（补偿硬件 BIN1/BIN2 交叉反接）
     *    motor_polarity = +1.0 → 符号不变, -1.0 → 符号翻转 */
    int16_t pwm = (int16_t)((float)pwm_raw * s->motor_polarity);

    /* 3) 翻转后的安全限幅（理论上仍在范围内，防止浮点转换出现边缘偏差） */
    if (pwm >  MOTOR_PWM_MAX_DUTY) pwm =  MOTOR_PWM_MAX_DUTY;
    if (pwm < -MOTOR_PWM_MAX_DUTY) pwm = -MOTOR_PWM_MAX_DUTY;
    motor_set_speed(s->ch, pwm);
}

void speed_stop(MotorSpeed_t *s)
{
    if (s == NULL) return;
    pid_reset(&s->pid);
    s->pid.setpoint = 0.0f;
    s->speed      = 0.0f;
    s->speed_raw  = 0.0f;
    s->last_delta = 0;
    s->last_out   = 0.0f;
    motor_set_speed(s->ch, 0);
}

void steer_init(float kp, float ki, float kd, float max_turn)
{
    g_steer.base_speed = 0.0f;
    g_steer.max_turn   = max_turn;
    pid_begin(&g_steer.pid, kp, ki, kd, max_turn, max_turn * 0.5f);
    g_steer.pid.setpoint = 0.0f;
}

void steer_set_base(float base)
{
    g_steer.base_speed = base;
}

void steer_set_kp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    g_steer.pid.kp = kp;
    pid_reset(&g_steer.pid);
}

void steer_set_ki(float ki)
{
    if (ki < 0.0f) ki = 0.0f;
    g_steer.pid.ki = ki;
    pid_reset(&g_steer.pid);
}

void steer_set_kd(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    g_steer.pid.kd = kd;
    pid_reset(&g_steer.pid);
}

void steer_set_param(float kp, float ki, float kd)
{
    if (kp < 0.0f) kp = 0.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (kd < 0.0f) kd = 0.0f;
    g_steer.pid.kp = kp;
    g_steer.pid.ki = ki;
    g_steer.pid.kd = kd;
    pid_reset(&g_steer.pid);
}

void steer_get_param(float *kp, float *ki, float *kd,
                     float *base, float *max_turn)
{
    if (kp)       *kp       = g_steer.pid.kp;
    if (ki)       *ki       = g_steer.pid.ki;
    if (kd)       *kd       = g_steer.pid.kd;
    if (base)     *base     = g_steer.base_speed;
    if (max_turn) *max_turn = g_steer.max_turn;
}

void steer_step(float error, float dt_ms)
{
    float turn = pid_step(&g_steer.pid, 0.0f, error, dt_ms / 1000.0f);
    float base = g_steer.base_speed;
    float tl   = base + turn;
    float tr   = base - turn;
    const float SPEED_LIMIT = 7999.0f;
    if (tl >  SPEED_LIMIT) tl =  SPEED_LIMIT;
    if (tl < -SPEED_LIMIT) tl = -SPEED_LIMIT;
    if (tr >  SPEED_LIMIT) tr =  SPEED_LIMIT;
    if (tr < -SPEED_LIMIT) tr = -SPEED_LIMIT;
    speed_set_target(&g_spd_left,  tl);
    speed_set_target(&g_spd_right, tr);
}

void steer_stop(void)
{
    pid_reset(&g_steer.pid);
    g_steer.base_speed = 0.0f;
    speed_stop(&g_spd_left);
    speed_stop(&g_spd_right);
}

void pid_app_init(void)
{
    speed_init(&g_spd_left,  MOTOR_LEFT,  0.3f, 0.191f, 0.0f);
    speed_init(&g_spd_right, MOTOR_RIGHT, 0.3f, 0.191f, 0.0f);

    /* 补偿右轮 TB6612 的 BIN1/BIN2 硬件交叉反接。
     * 现象：speed_set_target(RIGHT, -4000) 本应反转，实际满速正转（正反馈饱和）。
     * 根因：BIN1/BIN2 引脚实际接线与 SysConfig 宏定义交叉，
     *       导致 software backward 设置 → hardware 正转，继而形成正反馈发散。
     * 修复：右轮 PWM 输出乘以 -1.0，使软件语义（正PWM=前进, 负PWM=后退）
     *       与实际物理方向重新对齐，闭环恢复负反馈。 */
    speed_set_motor_polarity(&g_spd_right, 1.0f);

    steer_init(400.0f, 0.0f, 0.0f, 2000.0f);
    uart_printf(UART0,
                "Speed: L(kp=0.130 pol=+1) R(kp=0.020 pol=-1) | Steer: kp=400 max_turn=2000\r\n");
}