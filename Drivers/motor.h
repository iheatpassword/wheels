#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "gFunc.h"

#define MOTOR_PWM_PERIOD           (125U)
#define MOTOR_PWM_MAX_DUTY         (MOTOR_PWM_PERIOD - 1U)
#define SEVRO_PWM_PERIOD            (1800U)
#define SEVRO_PWM_MIN_DUTY          (75U)//(45U)//angle = 0
#define SEVRO_PWM_MAX_DUTY          (195U)//(225U)//angle = 180

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
