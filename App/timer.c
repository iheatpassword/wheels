#include "timer.h"
#include "oled_hardware_i2c.h"
#include "gFunc.h"

/* 全局计时器实例 */
static Timer_t timer;

/* OLED 显示位置定义 */
#define TIMER_OLED_X       0     /* 起始 X 坐标 */
#define TIMER_OLED_Y       0     /* 起始 Y 坐标（第 0 行） */
#define TIMER_DISPLAY_INTERVAL_MS 100  /* 显示刷新间隔 */

/* 辅助函数：格式化时间字符串 "MM:SS.mmm" */
static void format_time_string(uint32_t ms, char *buf)
{
    uint32_t minutes, seconds, millis_part;
    
    minutes = ms / 60000;
    seconds = (ms % 60000) / 1000;
    millis_part = ms % 1000;
    
    /* 使用 OLED 字符显示（每个字符 6px 宽，8x16 字体） */
    /* 格式：MM:SS:mmm 共 12 个字符 */
    buf[0]  = '0' + (minutes / 10);
    buf[1]  = '0' + (minutes % 10);
    buf[2]  = ':';
    buf[3]  = '0' + (seconds / 10);
    buf[4]  = '0' + (seconds % 10);
    buf[5]  = '.';
    buf[6]  = '0' + (millis_part / 100);
    buf[7]  = '0' + ((millis_part / 10) % 10);
    buf[8]  = '0' + (millis_part % 10);
    buf[9]  = '\0';
}

/* 初始化计时器 */
void timer_init(void)
{
    timer.state = TIMER_STOPPED;
    timer.start_ms = 0;
    timer.elapsed_ms = 0;
    timer.last_update_ms = 0;
    
    /* 初始显示计时器状态（不清除屏幕，保留其他内容） */
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y, (uint8_t *)"Timer: OFF  ", 16);
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y + 2, (uint8_t *)"00:00.000", 16);
}

/* 开始计时 */
void timer_start(void)
{
    if (timer.state == TIMER_RUNNING) {
        return;  /* 已在运行中，忽略 */
    }
    
    timer.start_ms = millis();
    timer.state = TIMER_RUNNING;
    timer.last_update_ms = 0;  /* 强制下次更新显示 */
    
    /* 更新 OLED 状态显示 */
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y, (uint8_t *)"Timer: RUN  ", 16);
}

/* 停止计时 */
void timer_stop(void)
{
    if (timer.state != TIMER_RUNNING) {
        return;  /* 已停止，忽略 */
    }
    
    /* 保存当前累计时间 */
    uint32_t current_ms = millis();
    timer.elapsed_ms += (current_ms - timer.start_ms);
    timer.state = TIMER_STOPPED;
    timer.last_update_ms = 0;  /* 强制下次更新显示 */
    
    /* 更新 OLED 状态显示 */
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y, (uint8_t *)"Timer: STOP ", 16);
    
    /* 显示最终时间 */
    char time_str[16];
    format_time_string(timer.elapsed_ms, time_str);
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y + 2, (uint8_t *)time_str, 16);
}

/* 重置计时器 */
void timer_reset(void)
{
    timer.state = TIMER_STOPPED;
    timer.start_ms = 0;
    timer.elapsed_ms = 0;
    timer.last_update_ms = 0;
    
    /* 更新 OLED 显示 */
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y, (uint8_t *)"Timer: OFF  ", 16);
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y + 2, (uint8_t *)"00:00.000", 16);
}

/* 定时器更新（在主循环中调用） */
void timer_update(void)
{
    if (timer.state != TIMER_RUNNING) {
        return;  /* 非运行状态不更新 */
    }
    
    uint32_t current_ms = millis();
    
    /* 限制显示更新频率，避免频繁刷屏 */
    if ((current_ms - timer.last_update_ms) < TIMER_DISPLAY_INTERVAL_MS) {
        return;
    }
    
    timer.last_update_ms = current_ms;
    
    /* 计算当前累计时间 */
    uint32_t current_elapsed = timer.elapsed_ms + (current_ms - timer.start_ms);
    
    /* 格式化并显示 */
    char time_str[16];
    format_time_string(current_elapsed, time_str);
    OLED_ShowString(TIMER_OLED_X, TIMER_OLED_Y + 2, (uint8_t *)time_str, 16);
}

/* 获取当前累计时间（ms） */
uint32_t timer_get_elapsed_ms(void)
{
    if (timer.state == TIMER_RUNNING) {
        uint32_t current_ms = millis();
        return timer.elapsed_ms + (current_ms - timer.start_ms);
    }
    return timer.elapsed_ms;
}

/* 获取当前状态 */
TimerState_t timer_get_state(void)
{
    return timer.state;
}
