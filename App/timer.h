#ifndef __TIMER_H__
#define __TIMER_H__

#include "ti_msp_dl_config.h"

/* 计时器状态 */
typedef enum {
    TIMER_STOPPED = 0,    /* 已停止 */
    TIMER_RUNNING = 1     /* 运行中 */
} TimerState_t;

/* 计时器结构体 */
typedef struct {
    TimerState_t state;       /* 当前状态 */
    uint32_t start_ms;        /* 开始时间戳 (millis) */
    uint32_t elapsed_ms;      /* 累计已用时间 (ms) */
    uint32_t last_update_ms;  /* 上次更新显示的时间戳 */
} Timer_t;

/* 计时器接口 */
void timer_init(void);
void timer_start(void);
void timer_stop(void);
void timer_reset(void);
void timer_update(void);           /* 在主循环中调用，更新 OLED 显示 */
uint32_t timer_get_elapsed_ms(void);
TimerState_t timer_get_state(void);

#endif
