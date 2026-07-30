#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "gFunc.h"

/* PWM 配置：周期 400，最大占空比 399（10kHz 控制频率）
 * MOTOR_PWM_MAX_DUTY 不带 U 后缀，避免取反时无符号回绕导致隐式转换警告 */
#define MOTOR_PWM_PERIOD           (400U)
#define MOTOR_PWM_MAX_DUTY         (399)
//注意!我将电机控制频率改为了10Khz, 400的周期!以匹配10Khz控制频率

typedef enum {
    MOTOR_LEFT  = 0U,
    MOTOR_RIGHT = 1U
} Motor_Channel_t;

typedef enum {
    MOTOR_STOP_COAST = 0U,
    MOTOR_STOP_BRAKE = 1U
} Motor_StopMode_t;

void motor_init(void);
void motor_set_speed(Motor_Channel_t channel, int16_t speed);
void motor_set_speed_both(int16_t leftSpeed, int16_t rightSpeed);
void motor_stop(Motor_Channel_t channel, Motor_StopMode_t mode);
void motor_stop_both(Motor_StopMode_t mode);
void motor_brake(Motor_Channel_t channel);
void motor_brake_both(void);
void motor_standby(bool enable);
void motor_test(void);

/* 方向反转测试：每 2 秒翻转目标速度，用于验证编码器极性和 PID 响应 */
void motor_dir_test_start(float speed);
void motor_dir_test_stop(void);
void motor_dir_test_update(void);

#endif
