#include "encoder.h"
#include "interrupt.h"

/* 编码器计数值 */
volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;

/* 编码器状态（用于正交解码） */
static uint8_t encoder_left_last_state = 0;
static uint8_t encoder_right_last_state = 0;

void encoder_init(void)
{
    /* 读取初始状态 */
    uint8_t a = DL_GPIO_readPins(Encoder_leftA_PORT, Encoder_leftA_PIN) ? 0x02 : 0x00;
    uint8_t b = DL_GPIO_readPins(Encoder_leftB_PORT, Encoder_leftB_PIN) ? 0x01 : 0x00;
    encoder_left_last_state = a | b;

    a = DL_GPIO_readPins(Encoder_rightA_PORT, Encoder_rightA_PIN) ? 0x02 : 0x00;
    b = DL_GPIO_readPins(Encoder_rightB_PORT, Encoder_rightB_PIN) ? 0x01 : 0x00;
    encoder_right_last_state = a | b;

    /* 启用 GROUP1 中断（GPIOA 中断） */
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

/* 正交编码器方向判断：基于格雷码状态机 */
/* 正转：00 -> 01 -> 11 -> 10 -> 00 */
/* 反转：00 -> 10 -> 11 -> 01 -> 00 */
static void encoder_update(volatile int32_t *count, uint8_t *last_state, GPIO_Regs * a_port, uint32_t a_pin, GPIO_Regs * b_port, uint32_t b_pin)
{
    uint8_t a = DL_GPIO_readPins(a_port, a_pin) ? 0x02 : 0x00;
    uint8_t b = DL_GPIO_readPins(b_port, b_pin) ? 0x01 : 0x00;
    uint8_t current_state = a | b;

    /* 根据状态变化判断方向 */
    switch (*last_state) {
        case 0x00: /* 00 */
            if (current_state == 0x01) *count++;  /* 00->01: 正转 */
            if (current_state == 0x02) *count--;  /* 00->10: 反转 */
            break;
        case 0x01: /* 01 */
            if (current_state == 0x03) *count++;  /* 01->11: 正转 */
            if (current_state == 0x00) *count--;  /* 01->00: 反转 */
            break;
        case 0x03: /* 11 */
            if (current_state == 0x02) *count++;  /* 11->10: 正转 */
            if (current_state == 0x01) *count--;  /* 11->01: 反转 */
            break;
        case 0x02: /* 10 */
            if (current_state == 0x00) *count++;  /* 10->00: 正转 */
            if (current_state == 0x03) *count--;  /* 10->11: 反转 */
            break;
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
