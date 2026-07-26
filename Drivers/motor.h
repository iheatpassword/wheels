#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"

#define L_MOTOR     0
#define R_MOTOR     1

void set_motor_duty(float duty, uint8_t channel);
void set_motor_freq(uint32_t freq, uint8_t channel);


#endif
