/*
 * 编码器驱动（极简采样接口）
 * 极性约定：正增量 = 电机前进方向
 * 采样时清零计数器并打时间戳，dt 由调用方传入计算速度
 */

#include "encoder.h"
#include "interrupt.h"
#include "clock.h"

volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;

static volatile uint8_t encoder_left_last_state = 0;
static volatile uint8_t encoder_right_last_state = 0;

static const int8_t encoder_lut[4][4] = {
    { 0,  1, -1,  0},
    {-1,  0,  0,  1},
    { 1,  0,  0, -1},
    { 0, -1,  1,  0},
};

void encoder_init(void)
{
    uint8_t a, b;
    a = (DL_GPIO_readPins(Encoder_leftA_PORT, Encoder_leftA_PIN) & Encoder_leftA_PIN) ? 0x02 : 0x00;
    b = (DL_GPIO_readPins(Encoder_leftB_PORT, Encoder_leftB_PIN) & Encoder_leftB_PIN) ? 0x01 : 0x00;
    encoder_left_last_state = a | b;

    a = (DL_GPIO_readPins(Encoder_rightA_PORT, Encoder_rightA_PIN) & Encoder_rightA_PIN) ? 0x02 : 0x00;
    b = (DL_GPIO_readPins(Encoder_rightB_PORT, Encoder_rightB_PIN) & Encoder_rightB_PIN) ? 0x01 : 0x00;
    encoder_right_last_state = a | b;

    enable_group1_irq = 1;
}

/* 采样左轮增量并打时间戳。dt_ms 输出距上次采样的真实间隔（ms）。
 * 首次调用 dt_ms 返回 0，调用方应跳过本次 PID 计算。 */
int32_t encoder_sample_left(float *dt_ms)
{
    int32_t temp = encoder_left_count;
    encoder_left_count = 0;

    static unsigned long last_tick = 0;
    static uint8_t first = 1;
    unsigned long now = tick_ms;
    float dt = first ? 0.0f : (float)(now - last_tick);
    first = 0;
    last_tick = now;
    if (dt_ms) *dt_ms = dt;
    return temp;
}

/* 采样右轮增量并打时间戳。dt_ms 输出距上次采样的真实间隔（ms）。
 * 首次调用 dt_ms 返回 0，调用方应跳过本次 PID 计算。 */
int32_t encoder_sample_right(float *dt_ms)
{
    int32_t temp = encoder_right_count;
    encoder_right_count = 0;

    static unsigned long last_tick = 0;
    static uint8_t first = 1;
    unsigned long now = tick_ms;
    float dt = first ? 0.0f : (float)(now - last_tick);
    first = 0;
    last_tick = now;
    if (dt_ms) *dt_ms = dt;
    return -temp;
}

void encoder_reset(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

void encoder_reset_all(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

static void encoder_update(volatile int32_t *count, volatile uint8_t *last_state,
                           GPIO_Regs *a_port, uint32_t a_pin,
                           GPIO_Regs *b_port, uint32_t b_pin)
{
    uint8_t a = (DL_GPIO_readPins(a_port, a_pin) & a_pin) ? 0x02 : 0x00;
    uint8_t b = (DL_GPIO_readPins(b_port, b_pin) & b_pin) ? 0x01 : 0x00;
    uint8_t current_state = a | b;

    uint8_t prev_state = *last_state;
    int8_t delta = encoder_lut[prev_state][current_state];
    if (delta != 0) {
        *count += delta;
    }

    *last_state = current_state;
}

void encoder_leftA_isr(void)
{
    encoder_update(&encoder_left_count, &encoder_left_last_state,
                   Encoder_leftA_PORT, Encoder_leftA_PIN,
                   Encoder_leftB_PORT, Encoder_leftB_PIN);
}

void encoder_leftB_isr(void)
{
    encoder_update(&encoder_left_count, &encoder_left_last_state,
                   Encoder_leftA_PORT, Encoder_leftA_PIN,
                   Encoder_leftB_PORT, Encoder_leftB_PIN);
}

void encoder_rightA_isr(void)
{
    encoder_update(&encoder_right_count, &encoder_right_last_state,
                   Encoder_rightA_PORT, Encoder_rightA_PIN,
                   Encoder_rightB_PORT, Encoder_rightB_PIN);
}

void encoder_rightB_isr(void)
{
    encoder_update(&encoder_right_count, &encoder_right_last_state,
                   Encoder_rightA_PORT, Encoder_rightA_PIN,
                   Encoder_rightB_PORT, Encoder_rightB_PIN);
}
