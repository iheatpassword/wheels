#include "encoder.h"
#include "interrupt.h"

/* 编码器计数值 */
volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;

/* 编码器状态（用于正交解码，ISR 中使用需 volatile） */
static volatile uint8_t encoder_left_last_state = 0;
static volatile uint8_t encoder_right_last_state = 0;

/* 正交编码器状态转换表 */
/* 行：last_state，列：current_state */
/* 值：+1=正转, -1=反转, 0=无效/抖动 */
static const int8_t encoder_lut[4][4] = {
    { 0,  1, -1,  0},  // 00 -> 00,01,10,11
    {-1,  0,  0,  1},  // 01 -> 00,01,10,11
    { 1,  0,  0, -1},  // 10 -> 00,01,10,11
    { 0, -1,  1,  0},  // 11 -> 00,01,10,11
};

void encoder_init(void)
{
    /* 读取初始状态（使用按位与确保正确判断） */
    uint8_t a = (DL_GPIO_readPins(Encoder_leftA_PORT, Encoder_leftA_PIN) & Encoder_leftA_PIN) ? 0x02 : 0x00;
    uint8_t b = (DL_GPIO_readPins(Encoder_leftB_PORT, Encoder_leftB_PIN) & Encoder_leftB_PIN) ? 0x01 : 0x00;
    encoder_left_last_state = a | b;

    a = (DL_GPIO_readPins(Encoder_rightA_PORT, Encoder_rightA_PIN) & Encoder_rightA_PIN) ? 0x02 : 0x00;
    b = (DL_GPIO_readPins(Encoder_rightB_PORT, Encoder_rightB_PIN) & Encoder_rightB_PIN) ? 0x01 : 0x00;
    encoder_right_last_state = a | b;

    /* 启用 GROUP1 中断 */
    enable_group1_irq = 1;
}

int32_t encoder_read_left(void)
{
    return encoder_left_count;
}

int32_t encoder_read_right(void)
{
    return encoder_right_count;
}

void encoder_reset(void)
{
    encoder_left_count = 0;
    encoder_right_count = 0;
}

static void encoder_update(volatile int32_t *count, volatile uint8_t *last_state, 
                           GPIO_Regs *a_port, uint32_t a_pin, 
                           GPIO_Regs *b_port, uint32_t b_pin)
{
    /* 使用按位与正确读取引脚状态 */
    uint8_t a = (DL_GPIO_readPins(a_port, a_pin) & a_pin) ? 0x02 : 0x00;
    uint8_t b = (DL_GPIO_readPins(b_port, b_pin) & b_pin) ? 0x01 : 0x00;
    uint8_t current_state = a | b;

    /* 通过查找表判断方向（确保 last_state 不在变化中） */
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
