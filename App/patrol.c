#include "patrol.h"
#include "motor.h"
#include "uart.h"
#include "gFunc.h"
#include <math.h>

/* ==================== 全局变量 ==================== */
PatrolData_t patrol_data = {0};
PatrolState_t patrol_state = PATROL_LINE;
PatrolConfig_t patrol_config = {0};
PID_Controller_t patrol_pid = {0};

/* 内部状态变量 */
static uint16_t lost_timer_ms = 0;           /* 丢线计时 */
static uint16_t turn_timer_ms = 0;           /* 转弯计时 */
static float last_position = 0;              /* 上次位置误差 */
static float filtered_position = 0;          /* 滤波后位置 */
static uint8_t consecutive_lost_count = 0;   /* 连续丢线次数 */
static uint8_t turn_detect_count = 0;        /* 转弯检测计数 */
static uint16_t tjunction_stop_timer = 0;    /* T型路口停止计时 */
static uint16_t search_timer = 0;            /* 搜索模式计时 */
static bool search_direction = true;         /* 搜索方向（true:左转, false:右转） */

/* 惯性导航数据（预留） */
static PatrolInertialData_t inertial_data = {0};
static bool inertial_enabled = false;

/* ==================== 状态机辅助函数 ==================== */

/**
 * @brief 安全的电机速度设置：将 float 转换为 int16_t 并限幅
 */
static int16_t patrol_float_to_motor_speed(float speed)
{
    /* 限幅到 PWM 范围 */
    if (speed > (float)MOTOR_PWM_MAX_DUTY) {
        speed = (float)MOTOR_PWM_MAX_DUTY;
    } else if (speed < -(float)MOTOR_PWM_MAX_DUTY) {
        speed = -(float)MOTOR_PWM_MAX_DUTY;
    }
    return (int16_t)speed;
}

/**
 * @brief 状态切换：封装状态转换的副作用处理
 */
static void patrol_change_state(PatrolState_t new_state)
{
    /* 仅在状态变化时执行切换逻辑 */
    if (new_state == patrol_state) return;
    
    patrol_state = new_state;
    
    /* 根据新状态重置相关计时器 */
    switch (new_state) {
        case PATROL_LINE:
            pid_reset(&patrol_pid);
            filtered_position = last_position;  /* 保持上次位置作为滤波初值 */
            break;
            
        case PATROL_LOST:
            lost_timer_ms = 0;
            consecutive_lost_count = 0;
            break;
            
        case PATROL_TURN_L:
        case PATROL_TURN_R:
            turn_timer_ms = 0;
            turn_detect_count = 0;
            break;
            
        case PATROL_TJUNCTION:
            tjunction_stop_timer = 0;
            break;
            
        case PATROL_SEARCH:
            search_timer = 0;
            search_direction = true;
            break;
            
        case PATROL_STOP:
            motor_stop_both(MOTOR_STOP_COAST);
            break;
            
        default:
            break;
    }
}

/* ==================== 默认配置 ==================== */
#define PATROL_DEFAULT_KP            0.8f
#define PATROL_DEFAULT_KI            0.1f
#define PATROL_DEFAULT_KD            0.05f
#define PATROL_DEFAULT_BASE_SPEED    40.0f
#define PATROL_DEFAULT_MIN_SPEED     15.0f
#define PATROL_DEFAULT_SPEED_RATIO   0.5f
#define PATROL_DEFAULT_LOST_TIMEOUT  500U
#define PATROL_DEFAULT_TURN_DETECT   200U
#define PATROL_DEFAULT_TURN_INNER    0
#define PATROL_DEFAULT_TURN_OUTER    50

/* 传感器权重配置 */
/* 实际物理顺序（从左到右）：r2 ---- r1 ---中线--- l1 ---- l2 */
/* r2: 左外侧，r1: 左内侧，l1: 右内侧，l2: 右外侧 */
#define PATROL_DEFAULT_WEIGHT_R2     3.0f    /* 左外侧权重 */
#define PATROL_DEFAULT_WEIGHT_R1     1.0f    /* 左内侧权重 */
#define PATROL_DEFAULT_WEIGHT_L1    -1.0f    /* 右内侧权重 */
#define PATROL_DEFAULT_WEIGHT_L2    -3.0f    /* 右外侧权重 */

/* ==================== 辅助函数 ==================== */

/**
 * @brief 计算位置误差（加权平均法）
 * 
 * 实际物理顺序（面向前方，从左到右）：r2 ---- r1 ---中线--- l1 ---- l2
 * 
 * r2/r1 是左侧传感器（r2=左外侧，r1=左内侧）
 * l1/l2 是右侧传感器（l1=右内侧，l2=右外侧）
 * 
 * 由于r1和l1距离中线更近，权重较小（1.0）
 * r2和l2距离中线更远，权重较大（3.0）
 * 
 * @param data 传感器数据
 * @return float 位置误差（-100 ~ 100，0表示在线中间，负值偏右，正值偏左）
 *              PATROL_POS_LOST（-999）表示丢线
 */
#define PATROL_POS_LOST  (-999.0f)
static float patrol_calc_position(PatrolData_t *data)
{
    float numerator = 0;
    float denominator = 0;
    
    /* 根据传感器状态累加权重 */
    /* r2/r1 左侧传感器：正值权重 */
    /* l1/l2 右侧传感器：负值权重 */
    if (data->r2) { numerator += patrol_config.weight_r2; denominator += 1.0f; }
    if (data->r1) { numerator += patrol_config.weight_r1; denominator += 1.0f; }
    if (data->l1) { numerator += patrol_config.weight_l1; denominator += 1.0f; }
    if (data->l2) { numerator += patrol_config.weight_l2; denominator += 1.0f; }
    
    /* 全部传感器丢线 */
    if (denominator == 0.0f) {
        return PATROL_POS_LOST;
    }
    
    /* 归一化到 -100 ~ 100 */
    /* 最大权重和为 3+1=4，最小为 -3-1=-4 */
    /* 归一化系数 = 100 / 4 = 25 */
    float position = (numerator / denominator) * 25.0f;
    
    /* 一阶滞后滤波，减少传感器抖动 */
    const float alpha = 0.3f;  /* 滤波系数，越小越平滑但响应越慢 */
    filtered_position = alpha * position + (1.0f - alpha) * filtered_position;
    
    return filtered_position;
}

/**
 * @brief 判断是否为T型路口
 * 
 * T型路口特征：左右内侧传感器同时检测到线（r1 & l1），
 * 且左右外侧传感器至少有一个也检测到线（r2 | l2）
 * 
 * @param data 传感器数据
 * @return true T型路口
 * @return false 不是T型路口
 */
static bool patrol_is_tjunction(PatrolData_t *data)
{
    /* T型路口：左右内侧都在线上，且至少有一侧外侧也在线上 */
    return (data->r1 && data->l1) && (data->r2 || data->l2);
}

/**
 * @brief 判断是否为左直角弯
 * 
 * 左直角弯特征：左内侧和左外侧都在线上（r1 & r2），
 * 右侧传感器全部丢线（!l1 && !l2）
 * 
 * 说明：当线路向左转弯时，小车会越过线，导致左侧传感器在线，右侧离线
 * 
 * @param data 传感器数据
 * @return true 左直角弯
 * @return false 不是左直角弯
 */
static bool patrol_is_left_turn(PatrolData_t *data)
{
    /* 左直角弯：左侧两个传感器在线，右侧两个传感器离线 */
    return (data->r1 && data->r2) && (!data->l1 && !data->l2);
}

/**
 * @brief 判断是否为右直角弯
 * 
 * 右直角弯特征：右内侧和右外侧都在线上（l1 & l2），
 * 左侧传感器全部丢线（!r1 && !r2）
 * 
 * 说明：当线路向右转弯时，小车会越过线，导致右侧传感器在线，左侧离线
 * 
 * @param data 传感器数据
 * @return true 右直角弯
 * @return false 不是右直角弯
 */
static bool patrol_is_right_turn(PatrolData_t *data)
{
    /* 右直角弯：右侧两个传感器在线，左侧两个传感器离线 */
    return (data->l1 && data->l2) && (!data->r1 && !data->r2);
}

/**
 * @brief 动态计算速度（根据位置误差调整）
 * 
 * 误差越大，速度越低，防止弯道甩尾
 * 
 * @param position 位置误差（-100 ~ 100）
 * @return float 当前速度
 */
static float patrol_calc_speed(float position)
{
    float abs_error = (position >= 0) ? position : -position;
    
    /* 速度随误差线性衰减 */
    /* abs_error = 0 时，速度 = base_speed */
    /* abs_error = 100 时，速度 = base_speed * (1 - speed_reduce_ratio) */
    float speed = patrol_config.base_speed * 
                  (1.0f - abs_error / 100.0f * patrol_config.speed_reduce_ratio);
    
    /* 速度下限保护 */
    if (speed < patrol_config.min_speed) {
        speed = patrol_config.min_speed;
    }
    
    return speed;
}

/* ==================== 状态机处理函数 ==================== */

/**
 * @brief 正常巡线状态处理
 */
static void patrol_state_line(uint32_t dt_ms)
{
    PatrolData_t data = patrol_read();
    float position = patrol_calc_position(&data);
    
    /* 丢线检测 */
    if (position == PATROL_POS_LOST) {
        consecutive_lost_count++;
        if (consecutive_lost_count > 3) {
            /* 连续丢线超过3次，进入丢线恢复状态 */
            uart_printf(UART0, "Patrol: Lost line (pos=%.1f)\r\n", last_position);
            patrol_change_state(PATROL_LOST);
        }
        return;
    }
    
    consecutive_lost_count = 0;
    
    /* 计算转弯检测阈值（向上取整，防止 dt_ms 过大时检测立即触发） */
    uint32_t detect_threshold = (patrol_config.turn_detect_ms + dt_ms - 1) / dt_ms;
    
    /* 直角弯检测 */
    if (patrol_is_left_turn(&data)) {
        turn_detect_count++;
        if (turn_detect_count >= detect_threshold) {
            uart_printf(UART0, "Patrol: Left turn detected (r2=%d,r1=%d,l1=%d,l2=%d)\r\n",
                        data.r2, data.r1, data.l1, data.l2);
            patrol_change_state(PATROL_TURN_L);
        }
        return;
    }
    
    if (patrol_is_right_turn(&data)) {
        turn_detect_count++;
        if (turn_detect_count >= detect_threshold) {
            uart_printf(UART0, "Patrol: Right turn detected (r2=%d,r1=%d,l1=%d,l2=%d)\r\n",
                        data.r2, data.r1, data.l1, data.l2);
            patrol_change_state(PATROL_TURN_R);
        }
        return;
    }
    
    turn_detect_count = 0;
    
    /* T型路口检测 */
    if (patrol_is_tjunction(&data)) {
        uart_printf(UART0, "Patrol: T-junction detected (r2=%d,r1=%d,l1=%d,l2=%d)\r\n",
                    data.r2, data.r1, data.l1, data.l2);
        patrol_change_state(PATROL_TJUNCTION);
        return;
    }
    
    /* PID 控制转向 */
    float dt = (float)dt_ms / 1000.0f;
    float turn = pid_update(&patrol_pid, position, dt);
    
    /* 计算速度 */
    float speed = patrol_calc_speed(position);
    
    /* 设置电机速度 */
    /* 电机约定：前进时左轮为负，右轮为正 */
    /* 位置 > 0（偏左）：需要右转，左电机减速（更负），右电机加速 */
    /* 位置 < 0（偏右）：需要左转，左电机加速（更接近0），右电机减速 */
    float left_speed_float = -(speed + turn);   /* 左轮取负 */
    float right_speed_float = (speed - turn);   /* 右轮保持正 */
    
    /* 安全类型转换并限幅 */
    int16_t left_speed = patrol_float_to_motor_speed(left_speed_float);
    int16_t right_speed = patrol_float_to_motor_speed(right_speed_float);
    
    motor_set_speed_both(left_speed, right_speed);
    
    last_position = position;
}

/**
 * @brief 丢线恢复状态处理
 */
static void patrol_state_lost(uint32_t dt_ms)
{
    lost_timer_ms += dt_ms;
    
    /* 根据惯性导航数据保持方向（预留） */
    if (inertial_enabled) {
        /* 可以使用陀螺仪数据保持当前方向 */
        /* 这里简单实现：保持上次的转向方向 */
    }
    
    /* 继续沿上次方向移动，尝试找回线 */
    /* last_position > 0：偏左（r2/r1在线），说明线在左侧，向左搜索 */
    /* last_position < 0：偏右（l1/l2在线），说明线在右侧，向右搜索 */
    int16_t search_speed = patrol_float_to_motor_speed(patrol_config.min_speed);
    if (last_position > 0) {
        /* 上次偏左，说明线在左侧，向左转向搜索 */
        /* 左转：左轮正，右轮正（原地向左转） */
        motor_set_speed_both(search_speed, search_speed);
    } else {
        /* 上次偏右，说明线在右侧，向右转向搜索 */
        /* 右转：左轮负，右轮负（原地向右转） */
        motor_set_speed_both(-search_speed, -search_speed);
    }
    
    /* 超时处理 */
    if (lost_timer_ms >= patrol_config.lost_timeout_ms) {
        uart_printf(UART0, "Patrol: Enter search mode (lost %dms)\r\n", lost_timer_ms);
        patrol_change_state(PATROL_SEARCH);
        return;
    }
    
    /* 检测是否重新找到线 */
    PatrolData_t data = patrol_read();
    float position = patrol_calc_position(&data);
    if (position != PATROL_POS_LOST) {
        /* 找到线，回到正常巡线 */
        uart_printf(UART0, "Patrol: Line found, back to normal (pos=%.1f)\r\n", position);
        patrol_change_state(PATROL_LINE);
    }
}

/**
 * @brief 左直角弯状态处理
 */
static void patrol_state_turn_l(uint32_t dt_ms)
{
    turn_timer_ms += dt_ms;
    
    /* 执行左转弯：内侧电机停/慢转，外侧电机快转 */
    /* 电机约定：前进时左轮为负，右轮为正 */
    /* 左转弯：左轮（内侧）慢/停，右轮（外侧）快转 */
    /* 左轮正（反转慢转），右轮正（正转快转）→ 原地向左转 */
    motor_set_speed_both(patrol_config.turn_speed_inner, 
                         patrol_config.turn_speed_outer);
    
    uart_printf(UART0, "Patrol: Left turning (timer=%dms, inner=%d, outer=%d)\r\n",
                turn_timer_ms, patrol_config.turn_speed_inner, patrol_config.turn_speed_outer);
    
    /* 检测是否完成转弯（重新检测到线） */
    PatrolData_t data = patrol_read();
    
    /* 左转弯完成：右侧传感器（l1/l2）重新检测到线 */
    /* 说明：左转弯时，小车向左旋转，右侧传感器会先碰到新的线路 */
    if (data.l1 || data.l2) {
        uart_printf(UART0, "Patrol: Left turn completed (l1=%d,l2=%d)\r\n", data.l1, data.l2);
        patrol_change_state(PATROL_LINE);
        return;
    }
    
    /* 超时保护 */
    if (turn_timer_ms > 2000) {
        /* 2秒未完成转弯，进入搜索模式 */
        uart_printf(UART0, "Patrol: Left turn timeout (%dms)\r\n", turn_timer_ms);
        patrol_change_state(PATROL_SEARCH);
    }
}

/**
 * @brief 右直角弯状态处理
 */
static void patrol_state_turn_r(uint32_t dt_ms)
{
    turn_timer_ms += dt_ms;
    
    /* 执行右转弯：内侧电机停/慢转，外侧电机快转 */
    /* 电机约定：前进时左轮为负，右轮为正 */
    /* 右转弯：右轮（内侧）慢/停，左轮（外侧）快转 */
    /* 左轮负（正转快转），右轮负（反转慢转）→ 原地向右转 */
    motor_set_speed_both(-patrol_config.turn_speed_outer, 
                         -patrol_config.turn_speed_inner);
    
    uart_printf(UART0, "Patrol: Right turning (timer=%dms, inner=%d, outer=%d)\r\n",
                turn_timer_ms, patrol_config.turn_speed_inner, patrol_config.turn_speed_outer);
    
    /* 检测是否完成转弯（重新检测到线） */
    PatrolData_t data = patrol_read();
    
    /* 右转弯完成：左侧传感器（r1/r2）重新检测到线 */
    /* 说明：右转弯时，小车向右旋转，左侧传感器会先碰到新的线路 */
    if (data.r1 || data.r2) {
        uart_printf(UART0, "Patrol: Right turn completed (r1=%d,r2=%d)\r\n", data.r1, data.r2);
        patrol_change_state(PATROL_LINE);
        return;
    }
    
    /* 超时保护 */
    if (turn_timer_ms > 2000) {
        /* 2秒未完成转弯，进入搜索模式 */
        uart_printf(UART0, "Patrol: Right turn timeout (%dms)\r\n", turn_timer_ms);
        patrol_change_state(PATROL_SEARCH);
    }
}

/**
 * @brief T型路口状态处理
 */
static void patrol_state_tjunction(uint32_t dt_ms)
{
    /* T型路口处理策略：
     * 1. 减速停止（默认行为）
     * 2. 根据预设策略选择方向（预留）
     */
    
    /* 减速停止 */
    tjunction_stop_timer += dt_ms;
    
    if (tjunction_stop_timer < 500) {
        /* 前500ms减速前进 */
        /* 电机约定：前进时左轮为负，右轮为正 */
        int16_t decel_speed = patrol_float_to_motor_speed(patrol_config.min_speed);
        motor_set_speed_both(-decel_speed, decel_speed);
        uart_printf(UART0, "Patrol: T-junction decelerating (timer=%dms)\r\n", tjunction_stop_timer);
    } else {
        /* 停止 */
        uart_printf(UART0, "Patrol: T-junction, stopped\r\n");
        patrol_change_state(PATROL_STOP);
    }
    
    /* TODO: 预留接口：根据外部指令选择转向方向 */
    /* 可以通过串口命令或预设策略设置转向方向 */
}

/**
 * @brief 搜索模式处理
 */
static void patrol_state_search(uint32_t dt_ms)
{
    /* 搜索模式：原地旋转寻找线 */
    
    search_timer += dt_ms;
    int16_t search_speed = patrol_float_to_motor_speed(patrol_config.min_speed);
    
    /* 交替左右旋转搜索 */
    /* 电机约定：前进时左轮为负，右轮为正 */
    /* 左转：左轮正，右轮正 */
    /* 右转：左轮负，右轮负 */
    if (search_timer < 1000) {
        if (search_direction) {
            /* 向左搜索 */
            motor_set_speed_both(search_speed, search_speed);
        } else {
            /* 向右搜索 */
            motor_set_speed_both(-search_speed, -search_speed);
        }
    } else {
        search_timer = 0;
        search_direction = !search_direction;
        uart_printf(UART0, "Patrol: Search direction changed (dir=%d)\r\n", search_direction);
    }
    
    /* 检测是否找到线 */
    PatrolData_t data = patrol_read();
    float position = patrol_calc_position(&data);
    if (position != PATROL_POS_LOST) {
        /* 找到线，回到正常巡线 */
        uart_printf(UART0, "Patrol: Line found in search mode (pos=%.1f)\r\n", position);
        patrol_change_state(PATROL_LINE);
    }
}

/* ==================== 对外接口函数 ==================== */

/**
 * @brief 读取传感器数据
 */
PatrolData_t patrol_read(void)
{
    /* 实际物理顺序（从左到右）：r2 ---- r1 ---中线--- l1 ---- l2 */
    /* r2: 左外侧，r1: 左内侧，l1: 右内侧，l2: 右外侧 */
    patrol_data.r2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R2_PIN) & Patrol_R2_PIN) ? 1 : 0;
    patrol_data.r1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R1_PIN) & Patrol_R1_PIN) ? 1 : 0;
    patrol_data.l1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L1_PIN) & Patrol_L1_PIN) ? 1 : 0;
    patrol_data.l2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L2_PIN) & Patrol_L2_PIN) ? 1 : 0;
    return patrol_data;
}

/**
 * @brief 初始化巡线模块
 */
void patrol_init(void)
{
    /* 设置默认配置 */
    patrol_config.pid_kp = PATROL_DEFAULT_KP;
    patrol_config.pid_ki = PATROL_DEFAULT_KI;
    patrol_config.pid_kd = PATROL_DEFAULT_KD;
    patrol_config.base_speed = PATROL_DEFAULT_BASE_SPEED;
    patrol_config.min_speed = PATROL_DEFAULT_MIN_SPEED;
    patrol_config.speed_reduce_ratio = PATROL_DEFAULT_SPEED_RATIO;
    patrol_config.lost_timeout_ms = PATROL_DEFAULT_LOST_TIMEOUT;
    patrol_config.turn_detect_ms = PATROL_DEFAULT_TURN_DETECT;
    patrol_config.turn_speed_inner = PATROL_DEFAULT_TURN_INNER;
    patrol_config.turn_speed_outer = PATROL_DEFAULT_TURN_OUTER;
    patrol_config.weight_r2 = PATROL_DEFAULT_WEIGHT_R2;
    patrol_config.weight_r1 = PATROL_DEFAULT_WEIGHT_R1;
    patrol_config.weight_l1 = PATROL_DEFAULT_WEIGHT_L1;
    patrol_config.weight_l2 = PATROL_DEFAULT_WEIGHT_L2;
    
    /* 初始化 PID 控制器 */
    /* 输出范围：-MOTOR_PWM_MAX_DUTY ~ MOTOR_PWM_MAX_DUTY（转向修正量） */
    pid_init(&patrol_pid, patrol_config.pid_kp, patrol_config.pid_ki, patrol_config.pid_kd,
             -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY, 100.0f);
    
    /* 设置目标值为 0（在线中间） */
    patrol_pid.setpoint = 0.0f;
    
    /* 初始化状态 */
    patrol_state = PATROL_LINE;
    lost_timer_ms = 0;
    turn_timer_ms = 0;
    last_position = 0;
    filtered_position = 0;
    consecutive_lost_count = 0;
    turn_detect_count = 0;
    tjunction_stop_timer = 0;
    search_timer = 0;
    search_direction = true;
    
    uart_printf(UART0, "Patrol: Initialized (r2-r1-l1-l2 order)\r\n");
}

/**
 * @brief 设置巡线配置
 */
void patrol_set_config(PatrolConfig_t *config)
{
    patrol_config = *config;
    
    /* 重新初始化 PID */
    pid_init(&patrol_pid, patrol_config.pid_kp, patrol_config.pid_ki, patrol_config.pid_kd,
             -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY, 100.0f);
    patrol_pid.setpoint = 0.0f;
}

/**
 * @brief 设置基础速度
 */
void patrol_set_speed(float speed)
{
    /* 限幅到有效范围 */
    if (speed > (float)MOTOR_PWM_MAX_DUTY) {
        patrol_config.base_speed = (float)MOTOR_PWM_MAX_DUTY;
    } else if (speed < 0.0f) {
        patrol_config.base_speed = 0.0f;
    } else {
        patrol_config.base_speed = speed;
    }
}

/**
 * @brief 获取当前位置误差
 */
float patrol_get_position(void)
{
    return last_position;
}

/**
 * @brief 获取当前巡线状态
 */
PatrolState_t patrol_get_state(void)
{
    return patrol_state;
}

/**
 * @brief 惯性导航数据更新（预留接口）
 */
void patrol_update_inertial(PatrolInertialData_t *data)
{
    if (data != NULL) {
        inertial_data = *data;
        inertial_enabled = true;
    }
}

/**
 * @brief 巡线主函数
 * 
 * 需定时调用，建议10ms周期
 * 
 * @param dt_ms 时间间隔（毫秒）
 */
void patrol_line(uint32_t dt_ms)
{
    switch (patrol_state) {
        case PATROL_LINE:
            patrol_state_line(dt_ms);
            break;
            
        case PATROL_LOST:
            patrol_state_lost(dt_ms);
            break;
            
        case PATROL_TURN_L:
            patrol_state_turn_l(dt_ms);
            break;
            
        case PATROL_TURN_R:
            patrol_state_turn_r(dt_ms);
            break;
            
        case PATROL_TJUNCTION:
            patrol_state_tjunction(dt_ms);
            break;
            
        case PATROL_SEARCH:
            patrol_state_search(dt_ms);
            break;
            
        case PATROL_STOP:
            /* 停止状态，保持电机停止 */
            motor_stop_both(MOTOR_STOP_COAST);
            break;
            
        default:
            patrol_state = PATROL_LINE;
            break;
    }
}
