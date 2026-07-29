#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "gFunc.h"

/* PWM 配置：周期 125，最大占空比 124（整数常量，避免浮点转换） */
#define MOTOR_PWM_PERIOD           (125U)
#define MOTOR_PWM_MAX_DUTY         (124U)

/* 舵机 PWM 配置：周期 1800，对应角度 0~180° 的 CCR 范围 */
#define SERVO_PWM_PERIOD            (1800U)
#define SERVO_PWM_MIN_DUTY          (75U)   /* 0° */
#define SERVO_PWM_MAX_DUTY          (195U)  /* 180° */

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
void servo_setting(uint16_t ccr);
void servo_test(void);

#endif
