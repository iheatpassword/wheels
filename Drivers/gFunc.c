#include "gFunc.h"
#include "motor.h"
#include "uart.h"
#include "clock.h"
#include "App/pid.h"

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

/* 字符串转浮点数（支持小数） */
static float uart_atof(const char *str)
{
    float result = 0.0f;
    float fraction = 0.1f;
    int8_t sign = 1;
    uint8_t has_dot = 0;
    
    while (*str == ' ' || *str == '\t') str++;
    
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }
    
    while (*str >= '0' && *str <= '9') {
        result = result * 10.0f + (*str - '0');
        str++;
    }
    
    if (*str == '.') {
        has_dot = 1;
        str++;
        while (*str >= '0' && *str <= '9') {
            result += (*str - '0') * fraction;
            fraction *= 0.1f;
            str++;
        }
    }
    
    return result * sign;
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
    
    /* 将角度转换为 CCR 值（线性映射 0~180° 到 SERVO_PWM_MIN~SERVO_PWM_MAX） */
    uint16_t ccr = SERVO_PWM_MIN_DUTY + (uint16_t)((angle / 180.0f) * (SERVO_PWM_MAX_DUTY - SERVO_PWM_MIN_DUTY));
    
    servo_setting(ccr);
    uart_printf(UART0, "Servo: angle=%d, ccr=%d\r\n", angle, ccr);
}

/* 解析通道字符为 PID_Channel_t */
static PID_Channel_t uart_parse_channel(char ch)
{
    switch (ch) {
        case 'l': case 'L': return PID_CH_LEFT;
        case 'r': case 'R': return PID_CH_RIGHT;
        case 'b': case 'B': return PID_CH_BOTH;
        default: return PID_CH_BOTH;  /* 默认两边 */
    }
}

/* 处理 PID 参数设置命令：spid <ch> <kp> <ki> <kd> */
static void uart_cmd_spid(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid <l|r|b> <kp> <ki> <kd>\r\n");
        return;
    }
    
    PID_Channel_t ch = uart_parse_channel(*p);
    p = uart_find_next_arg(p);
    
    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid <l|r|b> <kp> <ki> <kd>\r\n");
        return;
    }
    float kp = uart_atof(p);
    p = uart_find_next_arg(p);
    
    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid <l|r|b> <kp> <ki> <kd>\r\n");
        return;
    }
    float ki = uart_atof(p);
    p = uart_find_next_arg(p);
    
    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid <l|r|b> <kp> <ki> <kd>\r\n");
        return;
    }
    float kd = uart_atof(p);
    
    speed_pid_set_param(ch, kp, ki, kd);
}

/* 处理 PID 参数获取命令：gpid [ch] */
static void uart_cmd_gpid(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    PID_Channel_t ch = PID_CH_BOTH;  /* 默认两边 */
    
    if (*p != '\0') {
        ch = uart_parse_channel(*p);
    }
    
    float kp, ki, kd;
    speed_pid_get_param(ch, &kp, &ki, &kd);
}

/* 处理目标速度设置命令：starget <ch> <speed> */
static void uart_cmd_starget(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: starget <l|r|b> <speed>\r\n");
        return;
    }
    
    PID_Channel_t ch = uart_parse_channel(*p);
    p = uart_find_next_arg(p);
    
    if (*p == '\0') {
        uart_printf(UART0, "Usage: starget <l|r|b> <speed>\r\n");
        return;
    }
    float target = uart_atof(p);
    
    speed_pid_set_target(ch, target);
}

/* 处理当前速度获取命令：gspeed [ch] */
static void uart_cmd_gspeed(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    PID_Channel_t ch = PID_CH_BOTH;  /* 默认两边 */
    
    if (*p != '\0') {
        ch = uart_parse_channel(*p);
    }
    
    speed_pid_get_speed(ch);
}

/* 处理帮助命令 */
static void uart_cmd_help(void)
{
    uart_printf(UART0, "=== UART Debug Commands ===\r\n");
    uart_printf(UART0, "Motor Control:\r\n");
    uart_printf(UART0, "  m <left> <right>   - Set motor speed (-124~124)\r\n");
    uart_printf(UART0, "  mstop              - Stop motors\r\n");
    uart_printf(UART0, "  servo <angle>      - Set servo angle (0~180)\r\n");
    uart_printf(UART0, "\r\n");
    uart_printf(UART0, "Speed Loop PID:\r\n");
    uart_printf(UART0, "  spid <ch> <kp> <ki> <kd>  - Set speed PID params\r\n");
    uart_printf(UART0, "  gpid [ch]                 - Get speed PID params\r\n");
    uart_printf(UART0, "  starget <ch> <speed>      - Set target speed (cnt/s)\r\n");
    uart_printf(UART0, "  gspeed [ch]               - Get current speed\r\n");
    uart_printf(UART0, "\r\n");
    uart_printf(UART0, "  ch: l=left, r=right, b=both (default)\r\n");
    uart_printf(UART0, "  help                 - Show this help\r\n");
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
    if (cmd[0] == 's' && cmd[1] == 'p' && cmd[2] == 'i' && cmd[3] == 'd' &&
        (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* spid <ch> <kp> <ki> <kd> */
        uart_cmd_spid(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 'p' && cmd[2] == 'i' && cmd[3] == 'd' &&
               (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* gpid [ch] */
        uart_cmd_gpid(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'a' && cmd[3] == 'r' && cmd[4] == 'g' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* starget <ch> <speed> */
        uart_cmd_starget(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 's' && cmd[2] == 'p' && cmd[3] == 'e' && cmd[4] == 'e' && cmd[5] == 'd' &&
               (cmd[6] == ' ' || cmd[6] == '\t' || cmd[6] == '\0')) {
        /* gspeed [ch] */
        uart_cmd_gspeed(cmd);
    } else if (cmd[0] == 'm' && (cmd[1] == ' ' || cmd[1] == '\t' || cmd[1] == '\0')) {
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
        uart_printf(UART0, "Unknown command: %s (type 'help')\r\n", cmd);
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
