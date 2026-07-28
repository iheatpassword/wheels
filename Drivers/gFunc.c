#include "gFunc.h"
#include "motor.h"
#include "uart.h"
#include "clock.h"

/* 10ms 中断标志，由 TIMER_0 中断设置，主循环清除 */
volatile uint8_t read_patrol = 0;

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            /* 10ms 周期触发，仅设置标志，实际处理在主循环 */
            read_patrol = 1;
            break;
        default:
            break;    
    }
}

extern inline uint32_t millis(void)
{
    return tick_ms;
}

/* ================ 串口命令处理 ================ */

#define UART_CMD_BUFFER_SIZE    64

static char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
static uint8_t uart_cmd_buffer_len = 0;
static uint8_t uart_cmd_ready = 0;

/* 字符串转整数 */
static int32_t uart_atoi(const char *str)
{
    int32_t result = 0;
    int8_t sign = 1;
    
    while (*str == ' ' || *str == '\t') str++;
    
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }
    
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }
    
    return result * sign;
}

/* 查找下一个参数（不修改原字符串） */
static const char *uart_find_next_arg(const char *str)
{
    while (*str != '\0' && *str != ' ' && *str != '\t') str++;
    while (*str == ' ' || *str == '\t') str++;
    return str;
}

/* 处理电机速度命令 */
static void uart_cmd_motor(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: m <left_speed> <right_speed>\r\n");
        return;
    }
    
    int16_t left_speed = (int16_t)uart_atoi(p);
    
    p = uart_find_next_arg(p);
    if (*p == '\0') {
        uart_printf(UART0, "Usage: m <left_speed> <right_speed>\r\n");
        return;
    }
    
    int16_t right_speed = (int16_t)uart_atoi(p);
    
    motor_set_speed_both(left_speed, right_speed);
    uart_printf(UART0, "Motor: left=%d, right=%d\r\n", left_speed, right_speed);
}

/* 处理电机停止命令 */
static void uart_cmd_motor_stop(void)
{
    motor_stop_both(MOTOR_STOP_COAST);
    uart_printf(UART0, "Motor stopped\r\n");
}

/* 处理舵机角度命令 */
static void uart_cmd_servo(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: servo <angle> (0-180)\r\n");
        return;
    }
    
    int32_t angle = uart_atoi(p);
    
    /* 角度范围限制 */
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    /* 将角度转换为 CCR 值 */
    /* 0° → 45, 180° → 225 */
    uint16_t ccr = SEVRO_PWM_MIN_DUTY + (uint16_t)((angle / 180.0f) * (SEVRO_PWM_MAX_DUTY - SEVRO_PWM_MIN_DUTY));
    
    servo_setting(ccr);
    uart_printf(UART0, "Servo: angle=%d, ccr=%d\r\n", angle, ccr);
}

/* 处理帮助命令 */
static void uart_cmd_help(void)
{
    uart_printf(UART0, "=== UART Debug Commands ===\r\n");
    uart_printf(UART0, "m <left> <right>    - Set motor speed (-124~124)\r\n");
    uart_printf(UART0, "mstop               - Stop motors\r\n");
    uart_printf(UART0, "servo <angle>       - Set servo angle (0~180)\r\n");
    uart_printf(UART0, "help                - Show this help\r\n");
    uart_printf(UART0, "============================\r\n");
}

/* 命令解析主函数 */
void uart_cmd_process(void)
{
    if (!uart_cmd_ready) return;
    
    /* 立即清除标志，防止 ISR 覆盖 */
    uart_cmd_ready = 0;
    
    /* 命令缓冲区现在是安全的，可以读取 */
    const char *cmd = uart_cmd_buffer;
    
    /* 跳过前导空白 */
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    /* 匹配命令 */
    if (cmd[0] == 'm' && (cmd[1] == ' ' || cmd[1] == '\t' || cmd[1] == '\0')) {
        if (cmd[1] == '\0') {
            uart_cmd_motor_stop();
        } else {
            uart_cmd_motor(cmd);
        }
    } else if (cmd[0] == 'm' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'o' && cmd[4] == 'p' && 
              (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        uart_cmd_motor_stop();
    } else if (cmd[0] == 's' && cmd[1] == 'e' && cmd[2] == 'r' && cmd[3] == 'v' && cmd[4] == 'o' && 
              (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        uart_cmd_servo(cmd);
    } else if (cmd[0] == 'h' && cmd[1] == 'e' && cmd[2] == 'l' && cmd[3] == 'p' && 
              (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        uart_cmd_help();
    } else {
        uart_printf(UART0, "Unknown command: %s\r\n", cmd);
    }
}

/* UART 中断处理 */
void UART_0_INST_IRQHandler(void)
{
    uint8_t data;
    
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            DL_GPIO_togglePins(LED_PORT, LED_led0_PIN);
            
            data = DL_UART_Main_receiveData(UART_0_INST);
            
            /* 回显 */
            DL_UART_Main_transmitData(UART_0_INST, data);
            
            /* 如果命令尚未处理，丢弃新字符 */
            if (uart_cmd_ready) {
                break;
            }
            
            /* 处理命令 */
            if (data == '\r' || data == '\n') {
                /* 命令结束 */
                if (uart_cmd_buffer_len > 0) {
                    uart_cmd_buffer[uart_cmd_buffer_len] = '\0';
                    uart_cmd_ready = 1;
                }
                uart_cmd_buffer_len = 0;
            } else if (data == '\b' || data == 0x7F) {
                /* 退格 */
                if (uart_cmd_buffer_len > 0) {
                    uart_cmd_buffer_len--;
                }
            } else if (uart_cmd_buffer_len < UART_CMD_BUFFER_SIZE - 1) {
                /* 添加到缓冲区 */
                uart_cmd_buffer[uart_cmd_buffer_len++] = (char)data;
            }
            break;
            
        default:
            break;    
    }
}
