#ifndef _PID_H_
#define _PID_H_

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"

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

/* PD 控制器结构体（用于舵机，无积分） */
typedef struct {
    float kp;
    float kd;
    float setpoint;
    float output;
    float last_error;
    float output_min;
    float output_max;
} PD_Controller_t;

/* 速度控制结构体 */
typedef struct {
    PID_Controller_t pid;
    Motor_Channel_t motor_channel;
    volatile int32_t *encoder_count;
    int32_t last_count;
    float speed;  /* 当前速度（脉冲/秒） */
} Speed_Control_t;

/* 舵机角度控制结构体 */
typedef struct {
    PD_Controller_t pd;
    float *angle;  /* 当前角度指针 */
} Servo_Control_t;

/* 全局速度控制器实例 */
extern Speed_Control_t speed_left;
extern Speed_Control_t speed_right;

/* 全局舵机控制器实例 */
extern Servo_Control_t servo_pitch;
extern Servo_Control_t servo_roll;

/* PID 函数 */
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float output_min, float output_max, float integral_limit);
void pid_reset(PID_Controller_t *pid);
float pid_update(PID_Controller_t *pid, float feedback, float dt);

/* PD 函数 */
void pd_init(PD_Controller_t *pd, float kp, float kd, 
             float output_min, float output_max);
void pd_reset(PD_Controller_t *pd);
float pd_update(PD_Controller_t *pd, float feedback, float dt);

/* 速度控制函数 */
void speed_control_init(Speed_Control_t *sc, Motor_Channel_t channel, 
                        volatile int32_t *encoder, 
                        float kp, float ki, float kd);
void speed_control_set(Speed_Control_t *sc, float target_speed);
void speed_control_update(Speed_Control_t *sc, float dt);
void speed_control_stop(Speed_Control_t *sc);

/* 舵机角度控制函数 */
void servo_angle_init(Servo_Control_t *sc, float *angle, 
                      float kp, float kd);
void servo_angle_set(Servo_Control_t *sc, float target_angle);
void servo_angle_update(Servo_Control_t *sc, float dt);

/* 应用层初始化 */
void pid_app_init(void);

/* 应用层更新（建议在定时器中断中调用，周期建议 10ms） */
/* dt_ms: 上次调用到本次的时间间隔（毫秒） */
void pid_app_update(uint32_t dt_ms);

#endif
