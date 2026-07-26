#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"

// 定义枚举类型 LED_MODE
typedef enum
{
    LED_MODE_OFF,    // 0
    LED_MODE_ON,     // 1
    LED_MODE_TIMER_3S, //3
    LED_MODE_BLINK   // 2
} LED_MODE;

#define LED_MODE_OFF    0
#define LED_MODE_ON     1
#define LED_MODE_TIMER_3S   3
#define LED_MODE_BLINK      2
#define KEY_DEBOUNCE_MS     15
#define TIMER_3S_MS         3000
#define BLINK_INTERVAL_MS   1000

extern uint32_t millis(void);
uint8_t key_pressed(GPIO_Regs* key_port, uint32_t key_pin);
void key_test(void);

#endif
