#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "ti_msp_dl_config.h"

void encoder_init(void);
uint16_t encoder_read(void);

#endif