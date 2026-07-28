#ifndef __GFUNC_H__
#define __GFUNC_H__

#include "ti_msp_dl_config.h"

extern volatile uint8_t read_patrol;

extern uint32_t millis(void);
/* 串口命令处理函数声明 */
void uart_cmd_process(void);

#endif
