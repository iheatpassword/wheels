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
    float speed;  /* 当前速度（脉冲/秒） */
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

/* 应用层初始化 */
void pid_app_init(void);

/* 应用层更新 */
void pid_app_update(uint32_t dt_ms);

#endif
