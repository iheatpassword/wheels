#include "gFunc.h"
#include "motor.h"
#include "uart.h"
#include "clock.h"
#include "App/pid.h"
#include "App/patrol.h"
#include "App/timer.h"

/* 控制周期标志 (由 TIMER_0 中断设置, 主循环清除) */
volatile uint8_t steer_flag = 0;   /* 20ms (50Hz) 方向环 */
volatile uint8_t speed_flag = 0;   /* 10ms (100Hz) 速度环 */
volatile uint8_t encoder_flag = 0; /* 20ms 调试输出 */
volatile uint8_t oled_flag = 0;    /* 100ms OLED 刷新 */

/* 调试模式：1=仅速度环（禁用循迹保护），0=正常模式 */
volatile uint8_t debug_speed_only = 0;

void TIMER_0_INST_IRQHandler(void)
{
    static uint32_t counter=0;
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            /* 10ms 基本周期触发，仅设置标志，实际处理在主循环 */
            counter++;

            /* 10ms: 速度环 (100Hz) */
            speed_flag = 1;

            /* 20ms: 方向环 + 调试输出 (50Hz) */
            if((counter % 2) == 0) {
                steer_flag = 1;
                
            }

            /* 100ms: OLED 刷新 (10Hz) */
            if((counter % 10) == 0) {
                oled_flag = 1;
                encoder_flag = 1;
            }

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


/* 解析通道字符：
 *  l/L → left
 *  r/R → right
 *  其他/默认 → both */
static uint8_t uart_parse_channel(char ch)
{
    switch (ch) {
        case 'l': case 'L': return 0;  /* left only */
        case 'r': case 'R': return 1;  /* right only */
        default:            return 2;  /* both */
    }
}

/* 智能解析通道（可省略，默认 both），并移动指针 *pp 到下一 token 开始。
 * 识别规则：
 *   若当前 token 是单字符 (l/L/r/R/b/B) 后跟分隔符或结尾 → 按通道消费该 token；
 *   若当前 token 以数字/'-'/'.' 等数值字符开头 → 视为省略了通道（默认 both），
 *     不消费该 token，*pp 保持指向数值首字符，后续 uart_atof/uart_atoi 正常解析负数。
 * 这样输入 "starget -4000" 与 "starget r -4000" 都能正确解析。 */
static void uart_try_parse_channel(const char **pp, uint8_t *ch_out)
{
    const char *p = *pp;
    /* 跳过前导空白（调用方通常已跳过，防御性处理） */
    while (*p == ' ' || *p == '\t') p++;

    char c = *p;
    /* 合法单通道：c ∈ {l,r,b} 且 下一个字符是空白或 '\0' */
    if ((c == 'l' || c == 'L' || c == 'r' || c == 'R' ||
         c == 'b' || c == 'B') &&
        (p[1] == ' ' || p[1] == '\t' || p[1] == '\0')) {
        uint8_t ch;
        switch (c) {
            case 'l': case 'L': ch = 0; break;
            case 'r': case 'R': ch = 1; break;
            default:            ch = 2; break;
        }
        *ch_out = ch;
        p++;                                   /* 消费通道字符 */
        while (*p == ' ' || *p == '\t') p++;   /* 消费空白，指向下一 token */
        *pp = p;
        return;
    }
    /* 否则默认 both，且不移动指针（保留数值 token） */
    *ch_out = 2;
}

/* 处理 PID 参数设置命令：spid [ch] <kp> <ki> <kd>  (ch可省略，默认both) */
static void uart_cmd_spid(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid [l|r|b] <kp> <ki> <kd>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid [l|r|b] <kp> <ki> <kd>\r\n");
        return;
    }
    float kp = uart_atof(p);
    p = uart_find_next_arg(p);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid [l|r|b] <kp> <ki> <kd>\r\n");
        return;
    }
    float ki = uart_atof(p);
    p = uart_find_next_arg(p);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: spid [l|r|b] <kp> <ki> <kd>\r\n");
        return;
    }
    float kd = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_kp(&g_spd_left,  kp);
        speed_set_ki(&g_spd_left,  ki);
        speed_set_kd(&g_spd_left,  kd);
        uart_printf(UART0, "OK L: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }
    if (ch == 1 || ch == 2) {
        speed_set_kp(&g_spd_right, kp);
        speed_set_ki(&g_spd_right, ki);
        speed_set_kd(&g_spd_right, kd);
        uart_printf(UART0, "OK R: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }
}

/* 处理单独设置 kp 命令：skp [ch] <value>  (ch可省略，默认both) */
static void uart_cmd_skp(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Usage: skp [l|r|b] <kp>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: skp [l|r|b] <kp>\r\n");
        return;
    }
    float kp = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_kp(&g_spd_left, kp);
        uart_printf(UART0, "OK L kp=%5.3f\r\n", kp);
    }
    if (ch == 1 || ch == 2) {
        speed_set_kp(&g_spd_right, kp);
        uart_printf(UART0, "OK R kp=%5.3f\r\n", kp);
    }
}

/* 处理单独设置 ki 命令：ski [ch] <value>  (ch可省略，默认both) */
static void uart_cmd_ski(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Usage: ski [l|r|b] <ki>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: ski [l|r|b] <ki>\r\n");
        return;
    }
    float ki = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_ki(&g_spd_left, ki);
        uart_printf(UART0, "OK L ki=%5.3f\r\n", ki);
    }
    if (ch == 1 || ch == 2) {
        speed_set_ki(&g_spd_right, ki);
        uart_printf(UART0, "OK R ki=%5.3f\r\n", ki);
    }
}

/* 处理单独设置 kd 命令：skd [ch] <value>  (ch可省略，默认both) */
static void uart_cmd_skd(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Usage: skd [l|r|b] <kd>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: skd [l|r|b] <kd>\r\n");
        return;
    }
    float kd = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_kd(&g_spd_left, kd);
        uart_printf(UART0, "OK L kd=%5.3f\r\n", kd);
    }
    if (ch == 1 || ch == 2) {
        speed_set_kd(&g_spd_right, kd);
        uart_printf(UART0, "OK R kd=%5.3f\r\n", kd);
    }
}

/* 设置速度环滤波系数：sfilter [ch] <alpha>  (ch可省略，默认both)
 * alpha: 1=无滤波, 0.5=中等, 0.1=强滤波 */
static void uart_cmd_sfilter(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Usage: sfilter [l|r|b] <alpha>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: sfilter [l|r|b] <alpha>\r\n");
        return;
    }
    float alpha = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_filter(&g_spd_left, alpha);
        uart_printf(UART0, "OK L filter=%5.3f\r\n", alpha);
    }
    if (ch == 1 || ch == 2) {
        speed_set_filter(&g_spd_right, alpha);
        uart_printf(UART0, "OK R filter=%5.3f\r\n", alpha);
    }
}

/* 处理 PID 参数获取命令：gpid [ch]  (ch可省略，默认both) */
static void uart_cmd_gpid(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    uint8_t ch = 2;  /* 默认两边 */

    if (*p != '\0') {
        uart_try_parse_channel(&p, &ch);
    }

    float kp, ki, kd;
    if (ch == 0 || ch == 2) {
        speed_get_params(&g_spd_left, &kp, &ki, &kd);
        uart_printf(UART0, "PID L: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }
    if (ch == 1 || ch == 2) {
        speed_get_params(&g_spd_right, &kp, &ki, &kd);
        uart_printf(UART0, "PID R: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
    }
}

/* 处理目标速度设置命令：starget [ch] <speed>  (ch可省略，默认both) */
static void uart_cmd_starget(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    if (*p == '\0') {
        uart_printf(UART0, "Usage: starget [l|r|b] <speed>\r\n");
        return;
    }

    uint8_t ch;
    uart_try_parse_channel(&p, &ch);

    if (*p == '\0') {
        uart_printf(UART0, "Usage: starget [l|r|b] <speed>\r\n");
        return;
    }
    float target = uart_atof(p);

    if (ch == 0 || ch == 2) {
        speed_set_target(&g_spd_left, target);
        uart_printf(UART0, "OK L target=%5.1f (setpoint updated)\r\n", target);
    }
    if (ch == 1 || ch == 2) {
        speed_set_target(&g_spd_right, target);
        uart_printf(UART0, "OK R target=%5.1f (setpoint updated)\r\n", target);
    }
}

/* 处理当前速度获取命令：gspeed [ch]  (ch可省略，默认both) */
static void uart_cmd_gspeed(const char *args)
{
    const char *p = uart_find_next_arg(args);  /* 跳过命令名 */
    uint8_t ch = 2;  /* 默认两边 */

    if (*p != '\0') {
        uart_try_parse_channel(&p, &ch);
    }

    if (ch == 0 || ch == 2) {
        uart_printf(UART0, "Speed L: %5.1f (target: %5.1f)\r\n",
                    g_spd_left.speed, g_spd_left.pid.setpoint);
    }
    if (ch == 1 || ch == 2) {
        uart_printf(UART0, "Speed R: %5.1f (target: %5.1f)\r\n",
                    g_spd_right.speed, g_spd_right.pid.setpoint);
    }
}

/* ==================== 转向环命令 ==================== */

/* 设置转向环全部参数：stpid <kp> <ki> <kd> */
static void uart_cmd_stpid(const char *args)
{
    const char *p = uart_find_next_arg(args);
    float kp = uart_atof(p);   p = uart_find_next_arg(p);
    float ki = uart_atof(p);   p = uart_find_next_arg(p);
    float kd = uart_atof(p);

    if (kp == 0.0f && ki == 0.0f && kd == 0.0f) {
        const char *q = uart_find_next_arg(args);
        if (*q == '\0') {
            uart_printf(UART0, "Usage: stpid <kp> <ki> <kd>\r\n");
            return;
        }
    }
    steer_set_param(kp, ki, kd);
    uart_printf(UART0, "OK steer: kp=%5.3f ki=%5.3f kd=%5.3f\r\n", kp, ki, kd);
}

/* 单独设置转向环参数：stkp / stki / stkd <value> */
static void uart_cmd_stkp(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') { uart_printf(UART0, "Usage: stkp <kp>\r\n"); return; }
    float kp = uart_atof(p);
    steer_set_kp(kp);
    uart_printf(UART0, "OK steer kp=%5.3f\r\n", kp);
}

static void uart_cmd_stki(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') { uart_printf(UART0, "Usage: stki <ki>\r\n"); return; }
    float ki = uart_atof(p);
    steer_set_ki(ki);
    uart_printf(UART0, "OK steer ki=%5.3f\r\n", ki);
}

static void uart_cmd_stkd(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') { uart_printf(UART0, "Usage: stkd <kd>\r\n"); return; }
    float kd = uart_atof(p);
    steer_set_kd(kd);
    uart_printf(UART0, "OK steer kd=%5.3f\r\n", kd);
}

/* 获取转向环参数：gtpid */
static void uart_cmd_gtpid(const char *args)
{
    (void)args;
    float kp, ki, kd, base, mt;
    steer_get_param(&kp, &ki, &kd, &base, &mt);
    uart_printf(UART0,
                "Steer PID: kp=%5.3f ki=%5.3f kd=%5.3f  base=%5.1f  max_turn=%5.1f\r\n",
                kp, ki, kd, base, mt);
}

/* 查看循迹传感器实时状态：gpatrol */
static void uart_cmd_gpatrol(const char *args)
{
    (void)args;
    uint8_t r2, r1, l1, l2;
    patrol_get_raw(&r2, &r1, &l1, &l2);
    float err = 0.0f;
    PatrolStatus_t st = patrol_get_error(&err);
    const char *sname = (st == PATROL_OK) ? "OK"
                      : (st == PATROL_LOST) ? "LOST" : "JUNCTION";
    uart_printf(UART0, "pat[%d%d%d%d] %s err=%+4.2f  (r2=-3 r1=-1 l1=+1 l2=+3)\r\n",
                r2, r1, l1, l2, sname, err);
}

/* 设置基础前进速度：sbase <counts/s>（0=原地/停车） */
static void uart_cmd_sbase(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') { uart_printf(UART0, "Usage: sbase <counts_per_sec>\r\n"); return; }
    float spd = uart_atof(p);
    steer_set_base(spd);
    uart_printf(UART0, "OK steer base_speed=%5.1f cnt/s\r\n", spd);
}

/* 停止转向控制：sstop（同时停止转向环和速度环） */
static void uart_cmd_sstop(const char *args)
{
    (void)args;
    steer_stop();
    uart_printf(UART0, "OK steer + speed stopped\r\n");
}

/* 启用调试模式：仅速度环（禁用循迹保护） */
static void uart_cmd_debug_speed_only(const char *args)
{
    (void)args;
    debug_speed_only = 1;
    uart_printf(UART0, "OK debug speed-only mode enabled\r\n");
}

/* 恢复正常模式：启用循迹保护 */
static void uart_cmd_debug_speed_off(const char *args)
{
    (void)args;
    debug_speed_only = 0;
    /* 复位速度环 PID，防止调试模式下积累的积分带入正常模式 */
    pid_reset(&g_spd_left.pid);
    pid_reset(&g_spd_right.pid);
    uart_printf(UART0, "OK normal mode (patrol protection on)\r\n");
}

/* 方向环完整状态查询：gsteer */
static void uart_cmd_gsteer(const char *args)
{
    (void)args;
    steer_print_status();
}

/* 方向环每拍调试输出开关：sdebug <0|1> */
static void uart_cmd_sdebug(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Steer debug: %s (sdebug 0=off, 1=on, per 20ms step)\r\n",
                    steer_get_debug() ? "ON" : "OFF");
        return;
    }
    int on = uart_atoi(p);
    steer_set_debug(on ? 1 : 0);
    uart_printf(UART0, "OK steer debug: %s\r\n", steer_get_debug() ? "ON" : "OFF");
}

/* 方向环状态复位：steer_reset（PID清零 + 停车态，不清debug开关） */
static void uart_cmd_steer_reset(const char *args)
{
    (void)args;
    steer_reset();
    uart_printf(UART0, "OK steer reset (PID + state cleared, debug flag kept)\r\n");
}

/* 简单查表法巡线开关：simpletest <0|1>
 *   simpletest 1 → 走 patrol_simple_run（无PID，if-else查表，适合 Bring-Up 验证极性）
 *   simpletest 0 → 走原 PID 方向环 steer_step（默认）
 *   simpletest   → 查询当前状态 */
static void uart_cmd_simpletest(const char *args)
{
    const char *p = uart_find_next_arg(args);
    if (*p == '\0') {
        uart_printf(UART0, "Simple patrol: %s  (simpletest 0=PID steer, 1=if-else table)\r\n",
                    patrol_get_simple_mode() ? "ON (table)" : "OFF (PID steer)");
        return;
    }
    int on = uart_atoi(p);
    patrol_set_simple_mode(on ? 1 : 0);
    /* 切换模式时清理方向环 PID 和状态，避免旧参数/状态干扰 */
    steer_reset();
    uart_printf(UART0, "OK Simple patrol: %s\r\n",
                patrol_get_simple_mode() ? "ON (table, no PID)" : "OFF (PID steer)");
}

/* ==================== 计时器命令 ==================== */

/* 处理计时器开始命令：tstart */
static void uart_cmd_tstart(const char *args)
{
    (void)args;
    timer_start();
    uart_printf(UART0, "Timer START\r\n");
}

/* 处理计时器停止命令：tstop */
static void uart_cmd_tstop(const char *args)
{
    (void)args;
    timer_stop();
    uint32_t elapsed = timer_get_elapsed_ms();
    uart_printf(UART0, "Timer STOP: %lu ms elapsed\r\n", elapsed);
}

/* 处理计时器重置命令：treset */
static void uart_cmd_treset(const char *args)
{
    (void)args;
    timer_reset();
    uart_printf(UART0, "Timer RESET\r\n");
}

/* 获取计时器状态：gtimer */
static void uart_cmd_gtimer(const char *args)
{
    (void)args;
    TimerState_t state = timer_get_state();
    uint32_t elapsed = timer_get_elapsed_ms();
    const char *state_str = (state == TIMER_RUNNING) ? "RUNNING" : "STOPPED";
    uart_printf(UART0, "Timer: %s, elapsed=%lu ms\r\n", state_str, elapsed);
}

/* 处理帮助命令 */
static void uart_cmd_help(void)
{
    uart_printf(UART0, "=== UART Debug Commands ===\r\n");
    uart_printf(UART0, "Motor Control:\r\n");
    uart_printf(UART0, "  m <left> <right>   - Set motor speed (-399~399)\r\n");
    uart_printf(UART0, "  mstop              - Stop motors\r\n");
    uart_printf(UART0, "\r\n");
    uart_printf(UART0, "Speed Loop PID:\r\n");
    uart_printf(UART0, "  spid [ch] <kp> <ki> <kd>  - Set all PID params (ch default both)\r\n");
    uart_printf(UART0, "  skp [ch] <kp>             - Set kp only\r\n");
    uart_printf(UART0, "  ski [ch] <ki>             - Set ki only\r\n");
    uart_printf(UART0, "  skd [ch] <kd>             - Set kd only\r\n");
    uart_printf(UART0, "  sfilter [ch] <alpha>      - Set speed filter (0=strong, 1=none)\r\n");
    uart_printf(UART0, "  gpid [ch]                 - Get speed PID params\r\n");
    uart_printf(UART0, "  starget [ch] <speed>      - Set target speed (cnt/s, negative=reverse)\r\n");
    uart_printf(UART0, "  gspeed [ch]               - Get current speed\r\n");
    uart_printf(UART0, "\r\n");
    uart_printf(UART0, "Steer PID (Patrol Error Loop):\r\n");
    uart_printf(UART0, "  stpid <kp> <ki> <kd>      - Set steer PID params\r\n");
    uart_printf(UART0, "  stkp <kp>                 - Set steer kp\r\n");
    uart_printf(UART0, "  stki <ki>                 - Set steer ki\r\n");
    uart_printf(UART0, "  stkd <kd>                 - Set steer kd\r\n");
    uart_printf(UART0, "  gtpid                     - Get steer PID params\r\n");
    uart_printf(UART0, "  gpatrol                   - Show patrol sensors + error\r\n");
    uart_printf(UART0, "  sbase <cnt/s>             - Set base forward speed (0=stop)\r\n");
    uart_printf(UART0, "  sstop                     - Stop steer + speed loop\r\n");
    uart_printf(UART0, "  gsteer                    - Full steer status (state/sensors/PID/targets)\r\n");
    uart_printf(UART0, "  sdebug <0|1>              - Per-step steer trace (S: ... line, 50Hz)\r\n");
    uart_printf(UART0, "  steer_reset               - Reset steer PID+state (keeps debug flag)\r\n");
    uart_printf(UART0, "  simpletest <0|1>          - Simple patrol: 0=PID steer (def), 1=if-else table\r\n");
    uart_printf(UART0, "\r\n");
    uart_printf(UART0, "Timer (OLED Display):\r\n");
    uart_printf(UART0, "  tstart                    - Start timer\r\n");
    uart_printf(UART0, "  tstop                     - Stop timer\r\n");
    uart_printf(UART0, "  treset                    - Reset timer\r\n");
    uart_printf(UART0, "  gtimer                    - Get timer status\r\n");
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
    
    /* 重置缓冲区长度，准备接收下一条命令 */
    uart_cmd_buffer_len = 0;
    
    /* 跳过前导空白 */
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    /* 匹配命令 */
    if (cmd[0] == 's' && cmd[1] == 'p' && cmd[2] == 'i' && cmd[3] == 'd' &&
        (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* spid <ch> <kp> <ki> <kd> */
        uart_cmd_spid(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'k' && cmd[2] == 'p' &&
               (cmd[3] == ' ' || cmd[3] == '\t' || cmd[3] == '\0')) {
        /* skp <ch> <kp> */
        uart_cmd_skp(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'k' && cmd[2] == 'i' &&
               (cmd[3] == ' ' || cmd[3] == '\t' || cmd[3] == '\0')) {
        /* ski <ch> <ki> */
        uart_cmd_ski(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'k' && cmd[2] == 'd' &&
               (cmd[3] == ' ' || cmd[3] == '\t' || cmd[3] == '\0')) {
        /* skd <ch> <kd> */
        uart_cmd_skd(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'f' && cmd[2] == 'i' && cmd[3] == 'l' && cmd[4] == 't' && cmd[5] == 'e' && cmd[6] == 'r' &&
               (cmd[7] == ' ' || cmd[7] == '\t' || cmd[7] == '\0')) {
        /* sfilter <ch> <alpha> */
        uart_cmd_sfilter(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 'p' && cmd[2] == 'i' && cmd[3] == 'd' &&
               (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* gpid [ch] */
        uart_cmd_gpid(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'a' && cmd[3] == 'r' && 
               cmd[4] == 'g' && cmd[5] == 'e' && cmd[6] == 't' &&
               (cmd[7] == ' ' || cmd[7] == '\t' || cmd[7] == '\0')) {
        /* starget <ch> <speed> */
        uart_cmd_starget(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 's' && cmd[2] == 'p' && cmd[3] == 'e' && cmd[4] == 'e' && cmd[5] == 'd' &&
               (cmd[6] == ' ' || cmd[6] == '\t' || cmd[6] == '\0')) {
        /* gspeed [ch] */
        uart_cmd_gspeed(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'p' && cmd[3] == 'i' && cmd[4] == 'd' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* stpid <kp> <ki> <kd> */
        uart_cmd_stpid(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'k' && cmd[3] == 'p' &&
               (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* stkp <kp> */
        uart_cmd_stkp(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'k' && cmd[3] == 'i' &&
               (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* stki <ki> */
        uart_cmd_stki(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'k' && cmd[3] == 'd' &&
               (cmd[4] == ' ' || cmd[4] == '\t' || cmd[4] == '\0')) {
        /* stkd <kd> */
        uart_cmd_stkd(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 't' && cmd[2] == 'p' && cmd[3] == 'i' && cmd[4] == 'd' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* gtpid */
        uart_cmd_gtpid(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 'p' && cmd[2] == 'a' && cmd[3] == 't' && cmd[4] == 'r' &&
               cmd[5] == 'o' && cmd[6] == 'l' &&
               (cmd[7] == ' ' || cmd[7] == '\t' || cmd[7] == '\0')) {
        /* gpatrol */
        uart_cmd_gpatrol(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'b' && cmd[2] == 'a' && cmd[3] == 's' && cmd[4] == 'e' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* sbase <cnt/s> */
        uart_cmd_sbase(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'o' && cmd[4] == 'p' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* sstop */
        uart_cmd_sstop(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'e' &&
               cmd[4] == 'e' && cmd[5] == 'r' &&
               (cmd[6] == ' ' || cmd[6] == '\t' || cmd[6] == '\0')) {
        /* gsteer */
        uart_cmd_gsteer(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'd' && cmd[2] == 'e' && cmd[3] == 'b' &&
               cmd[4] == 'u' && cmd[5] == 'g' &&
               (cmd[6] == ' ' || cmd[6] == '\t' || cmd[6] == '\0')) {
        /* sdebug <0|1> */
        uart_cmd_sdebug(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'e' && cmd[3] == 'e' &&
               cmd[4] == 'r' && cmd[5] == '_' && cmd[6] == 'r' && cmd[7] == 'e' &&
               cmd[8] == 's' && cmd[9] == 'e' && cmd[10] == 't' &&
               (cmd[11] == ' ' || cmd[11] == '\t' || cmd[11] == '\0')) {
        /* steer_reset */
        uart_cmd_steer_reset(cmd);
    } else if (cmd[0] == 's' && cmd[1] == 'i' && cmd[2] == 'm' && cmd[3] == 'p' &&
               cmd[4] == 'l' && cmd[5] == 'e' && cmd[6] == 't' && cmd[7] == 'e' &&
               cmd[8] == 's' && cmd[9] == 't' &&
               (cmd[10] == ' ' || cmd[10] == '\t' || cmd[10] == '\0')) {
        /* simpletest <0|1> */
        uart_cmd_simpletest(cmd);
    } else if (cmd[0] == 'd' && cmd[1] == 'e' && cmd[2] == 'b' && cmd[3] == 'u' && 
               cmd[4] == 'g' && cmd[5] == '_' && cmd[6] == 's' && cmd[7] == 'p' &&
               cmd[8] == 'e' && cmd[9] == 'e' && cmd[10] == 'd' && cmd[11] == '_' &&
               cmd[12] == 'o' && cmd[13] == 'n' && cmd[14] == 'l' && cmd[15] == 'y' &&
               (cmd[16] == ' ' || cmd[16] == '\t' || cmd[16] == '\0')) {
        /* debug_speed_only */
        uart_cmd_debug_speed_only(cmd);
    } else if (cmd[0] == 'd' && cmd[1] == 'e' && cmd[2] == 'b' && cmd[3] == 'u' && 
               cmd[4] == 'g' && cmd[5] == '_' && cmd[6] == 's' && cmd[7] == 'p' &&
               cmd[8] == 'e' && cmd[9] == 'e' && cmd[10] == 'd' && cmd[11] == '_' &&
               cmd[12] == 'o' && cmd[13] == 'f' && cmd[14] == 'f' &&
               (cmd[15] == ' ' || cmd[15] == '\t' || cmd[15] == '\0')) {
        /* debug_speed_off */
        uart_cmd_debug_speed_off(cmd);
    } else if (cmd[0] == 'm' && (cmd[1] == ' ' || cmd[1] == '\t' || cmd[1] == '\0')) {
        if (cmd[1] == '\0') {
            uart_cmd_motor_stop();
        } else {
            uart_cmd_motor(cmd);
        }
    } else if (cmd[0] == 'm' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'o' && cmd[4] == 'p' && 
              (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        uart_cmd_motor_stop();
    } else if (cmd[0] == 't' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'a' && cmd[4] == 'r' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* tstart */
        uart_cmd_tstart(cmd);
    } else if (cmd[0] == 't' && cmd[1] == 's' && cmd[2] == 't' && cmd[3] == 'o' && cmd[4] == 'p' &&
               (cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* tstop */
        uart_cmd_tstop(cmd);
    } else if (cmd[0] == 't' && cmd[1] == 'r' && cmd[2] == 'e' && cmd[3] == 's' && cmd[4] == 'e' &&
               (cmd[5] == 't' || cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* treset */
        uart_cmd_treset(cmd);
    } else if (cmd[0] == 'g' && cmd[1] == 't' && cmd[2] == 'i' && cmd[3] == 'm' && cmd[4] == 'e' &&
               (cmd[5] == 'r' || cmd[5] == ' ' || cmd[5] == '\t' || cmd[5] == '\0')) {
        /* gtimer */
        uart_cmd_gtimer(cmd);
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
            
            // /* 回显 */
            // DL_UART_Main_transmitData(UART_0_INST, data);
            
            /* 如果命令尚未处理，丢弃新字符 */
            if (uart_cmd_ready) {
                break;
            }
            
            /* 处理命令 */
            if (data == '\r' || data == '\n') {
                /* 命令结束：仅当缓冲区有内容时才触发命令就绪 */
                if (uart_cmd_buffer_len > 0) {
                    uart_cmd_buffer[uart_cmd_buffer_len] = '\0';
                    uart_cmd_ready = 1;
                    /* 保留 uart_cmd_buffer_len 不变，主循环处理后再清空 */
                }
                /* 空行（连续 \r\n）直接忽略，不设置就绪标志 */
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
