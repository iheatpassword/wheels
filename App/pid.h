#ifndef _PID_H_
#define _PID_H_

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"

/* 电机通道扩展（支持两边同时控制） */
typedef enum {
    PID_CH_LEFT  = 0,   /* 左电机 */
    PID_CH_RIGHT = 1,   /* 右电机 */
    PID_CH_BOTH  = 2    /* 两边 */
} PID_Channel_t;

/* PID 控制器结构体 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float output;
    float integral;
    float last_error;
    float output_min;
    float output_max;
    float integral_limit;  /* 积分限幅 */
} PID_Controller_t;

/* 速度控制结构体 */
typedef struct {
    PID_Controller_t pid;
    Motor_Channel_t motor_channel;
    volatile int32_t *encoder_count;
    int32_t last_count;
    float speed;       /* 当前速度（脉冲/秒） */
    float polarity;    /* 编码器极性：+1.0 或 -1.0，用于对齐编码器与电机方向 */
} Speed_Control_t;

/* 全局速度控制器实例 */
extern Speed_Control_t speed_left;
extern Speed_Control_t speed_right;

/* PID 基础函数 */
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float output_min, float output_max, float integral_limit);
void pid_reset(PID_Controller_t *pid);
float pid_update(PID_Controller_t *pid, float feedback, float dt);

/* 速度控制函数 */
void speed_control_init(Speed_Control_t *sc, Motor_Channel_t channel, 
                        volatile int32_t *encoder, 
                        float kp, float ki, float kd);
void speed_control_set(Speed_Control_t *sc, float target_speed);
void speed_control_update(Speed_Control_t *sc, float dt);
void speed_control_stop(Speed_Control_t *sc);

/* 串口调参接口 */
void speed_pid_set_param(PID_Channel_t channel, float kp, float ki, float kd);
void speed_pid_get_param(PID_Channel_t channel, float *kp, float *ki, float *kd);
void speed_pid_set_target(PID_Channel_t channel, float target_speed);
float speed_pid_get_speed(PID_Channel_t channel);

/* 单独调整参数接口 */
void speed_pid_set_kp(PID_Channel_t channel, float kp);
void speed_pid_set_ki(PID_Channel_t channel, float ki);
void speed_pid_set_kd(PID_Channel_t channel, float kd);

/* 获取 PID 内部原始速度（调参用，确保与 PID 反馈一致） */
void speed_pid_get_raw_speed(float *left_speed, float *right_speed);

/* ================ 转向环（循迹偏差控制） ================ */

/* 转向控制结构体：以循迹加权偏差为输入，差速输出到左右轮 */
typedef struct {
    PID_Controller_t pid;       /* 转向 PID（setpoint=0，目标是偏差为0即居中） */
    float turn_output;          /* 转向输出量（脉冲/秒，差速补偿） */
    float base_speed;           /* 基础前进速度（脉冲/秒，0=原地转向） */
    float max_turn_output;      /* 最大转向输出限幅（脉冲/秒） */
} Steer_Control_t;

extern Steer_Control_t steer_control;

/* 初始化转向环
 * kp, ki, kd: 转向 PID 参数（kp 单位：脉冲/秒 每单位偏差）
 * max_turn: 最大转向差速（脉冲/秒） */
void steer_pid_init(float kp, float ki, float kd, float max_turn);

/* 更新转向环（10ms 调用），内部计算差速并设置左右轮目标速度
 * position_error: 循迹加权偏差（正值=车偏左需右转，负值=车偏右需左转）
 * dt_ms: 控制周期（毫秒）
 * 差速合成：target_left = base + turn, target_right = base - turn
 *           turn 正 → 左轮快右轮慢 → 右转 */
void steer_pid_update(float position_error, uint32_t dt_ms);

/* 设置基础前进速度（脉冲/秒，0=原地转向） */
void steer_pid_set_base_speed(float base_speed_counts);
float steer_pid_get_base_speed(void);

/* 停止转向控制，左右轮目标速度置 0 */
void steer_pid_stop(void);

/* 调参接口：单独设置转向环参数 */
void steer_pid_set_kp(float kp);
void steer_pid_set_ki(float ki);
void steer_pid_set_kd(float kd);
void steer_pid_set_param(float kp, float ki, float kd);
void steer_pid_get_param(float *kp, float *ki, float *kd);

/* 应用层初始化 */
void pid_app_init(void);

/* 应用层更新 */
void pid_app_update(uint32_t dt_ms);

#endif
