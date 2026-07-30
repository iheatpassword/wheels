#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "ti_msp_dl_config.h"

extern volatile int32_t encoder_left_count;
extern volatile int32_t encoder_right_count;

void    encoder_init(void);

/* 采样增量并打时间戳。dt_ms 输出距上次采样的真实间隔（ms），首次返回 0。 */
int32_t encoder_sample_left(float *dt_ms);
int32_t encoder_sample_right(float *dt_ms);
void    encoder_reset(void);
void    encoder_reset_all(void);

void encoder_leftA_isr(void);
void encoder_leftB_isr(void);
void encoder_rightA_isr(void);
void encoder_rightB_isr(void);

#endif
