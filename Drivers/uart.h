#ifndef __UART_H__
#define __UART_H__

#include "ti_msp_dl_config.h"

void uart_send_string(UART_Regs* uart, char* str);
void uart_printf(UART_Regs* uart, char* fmt, ...);

/* JustFloat 协议：发送 float 数组 + 帧尾 {0x00, 0x00, 0x80, 0x7f}
 * 用于上位机波形显示。data: float 数组, count: 通道数 */
void uart_justfloat(UART_Regs* uart, float *data, uint8_t count);

#endif
