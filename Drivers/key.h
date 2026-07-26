#ifndef _key_H
#define _key_H

#include "ti_msp_dl_config.h"

extern uint32_t millis(void);
uint8_t key_pressed(GPIO_Regs* key_port, uint32_t key_pin);
void key_test(void);

#endif
