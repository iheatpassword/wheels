#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "ti_msp_dl_config.h"

extern volatile int32_t encoder_left_count;
extern volatile int32_t encoder_right_count;

void encoder_init(void);
int32_t encoder_read_left(void);
int32_t encoder_read_right(void);
void encoder_reset(void);
void encoder_get_speed(int32_t *left_speed, int32_t *right_speed);

void encoder_leftA_isr(void);
void encoder_leftB_isr(void);
void encoder_rightA_isr(void);
void encoder_rightB_isr(void);

#endif
