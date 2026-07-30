#ifndef _PID_H_
#define _PID_H_

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"

/* ==========================================================================
 * 通用 PID 控制器 — 极简纯算法版
 *  - 输出硬限幅 [out_min, out_max]
 *  - 积分软限幅：integral_max
 *  - 无抗饱和、无低通滤波、无极性补偿
 * ========================================================================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float last_error;
    float out_min;
    float out_max;
    float integral_max;
} PID_t;

/* ==========================================================================
 * 速度闭环：每 10ms 调用一次 speed_update()
 * 外部传入编码器增量 (delta) → 实测速度 → 一阶低通滤波 → PID → PWM 输出
 * ========================================================================== */
typedef struct {
    PID_t  pid;
    Motor_Channel_t ch;
    float  speed;          /* 滤波后的速度（用于 PID 反馈） */
    float  speed_raw;      /* 原始未经滤波的速度（用于调试） */
    float  filter_alpha;   /* 低通系数：1=无滤波, 0.5=中等, 0.1=强滤波 */
    int32_t last_delta;
    float  last_out;       /* speed_update 最近一次实际输出（限幅后，下发 PWM 前） */
} MotorSpeed_t;

/* ==========================================================================
 * 方向环：每 20ms 调用一次 steer_step()
 * 循迹误差 ∈ [-3, +3] → PID → 差速补偿 → 叠加到左右轮基础速度
 * ========================================================================== */
typedef struct {
    PID_t  pid;
    float  base_speed;
    float  max_turn;
} Steer_t;

/* 全局实例 */
extern MotorSpeed_t g_spd_left;
extern MotorSpeed_t g_spd_right;
extern Steer_t      g_steer;

/* ---------------- 通用 PID ---------------- */
void  pid_begin(PID_t *p, float kp, float ki, float kd,
                float out_max, float integral_max);
void  pid_reset(PID_t *p);
float pid_step(PID_t *p, float setpoint, float measure, float dt);

/* ---------------- 速度环 ---------------- */
void  speed_init(MotorSpeed_t *s, Motor_Channel_t ch,
                 float kp, float ki, float kd);
void  speed_set_target(MotorSpeed_t *s, float target);
void  speed_set_kp(MotorSpeed_t *s, float kp);
void  speed_set_ki(MotorSpeed_t *s, float ki);
void  speed_set_kd(MotorSpeed_t *s, float kd);
void  speed_set_filter(MotorSpeed_t *s, float alpha);
void  speed_get_params(MotorSpeed_t *s, float *kp, float *ki, float *kd);
float speed_get_speed(MotorSpeed_t *s);
void  speed_update(MotorSpeed_t *s, int32_t delta, float dt_ms);
void  speed_stop(MotorSpeed_t *s);

/* ---------------- 方向环 ---------------- */
void  steer_init(float kp, float ki, float kd, float max_turn);
void  steer_set_base(float base);
void  steer_set_kp(float kp);
void  steer_set_ki(float ki);
void  steer_set_kd(float kd);
void  steer_set_param(float kp, float ki, float kd);
void  steer_get_param(float *kp, float *ki, float *kd, float *base, float *max_turn);
void  steer_step(float error, float dt_ms);
void  steer_stop(void);

/* ---------------- 应用层 ---------------- */
void  pid_app_init(void);

#endif /* _PID_H_ */