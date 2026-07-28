#ifndef __PATROL_H__
#define __PATROL_H__

#include "ti_msp_dl_config.h"
#include "pid.h"

/* ==================== 传感器数据结构 ==================== */
/* 实际物理顺序（面向前方，从左到右）：r2 ---- r1 ---中线--- l1 ---- l2 */
typedef struct
{
    uint8_t r2:1;  /* 左外侧传感器 */
    uint8_t r1:1;  /* 左内侧传感器（距离中线较近） */
    uint8_t l1:1;  /* 右内侧传感器（距离中线较近） */
    uint8_t l2:1;  /* 右外侧传感器 */
} PatrolData_t;

extern PatrolData_t patrol_data;

/* ==================== 巡线状态机 ==================== */
typedef enum {
    PATROL_LINE      = 0,  /* 正常巡线 */
    PATROL_LOST      = 1,  /* 丢线恢复 */
    PATROL_TURN_L    = 2,  /* 左直角弯 */
    PATROL_TURN_R    = 3,  /* 右直角弯 */
    PATROL_TJUNCTION = 4,  /* T型路口 */
    PATROL_SEARCH    = 5,  /* 搜索模式 */
    PATROL_STOP      = 6,  /* 停止 */
} PatrolState_t;

/* ==================== 惯性导航数据结构（预留） ==================== */
typedef struct {
    float pitch;   /* 俯仰角（度） */
    float roll;    /* 横滚角（度） */
    float yaw;     /* 航向角（度） */
    float angular_velocity_z;  /* 角速度Z轴（度/秒） */
} PatrolInertialData_t;

/* ==================== 巡线配置参数 ==================== */
typedef struct {
    /* PID 参数 */
    float pid_kp;           /* 比例系数 */
    float pid_ki;           /* 积分系数 */
    float pid_kd;           /* 微分系数 */
    
    /* 速度参数 */
    float base_speed;       /* 基础速度（直道） */
    float min_speed;        /* 最小速度（弯道） */
    float speed_reduce_ratio; /* 速度随误差衰减比例 */
    
    /* 丢线参数 */
    uint16_t lost_timeout_ms;  /* 丢线超时时间（毫秒） */
    
    /* 转弯参数 */
    uint16_t turn_detect_ms;   /* 转弯检测时间（毫秒） */
    int16_t turn_speed_inner;  /* 转弯时内侧电机速度 */
    int16_t turn_speed_outer;  /* 转弯时外侧电机速度 */
    
    /* 传感器权重（根据实际间距调整） */
    float weight_r2;        /* 左外侧权重 */
    float weight_r1;        /* 左内侧权重（距离近，权重小） */
    float weight_l1;        /* 右内侧权重（距离近，权重小） */
    float weight_l2;        /* 右外侧权重 */
} PatrolConfig_t;

/* ==================== 全局变量 ==================== */
extern PatrolState_t patrol_state;
extern PatrolConfig_t patrol_config;
extern PID_Controller_t patrol_pid;

/* ==================== 函数声明 ==================== */

/* 读取传感器数据 */
PatrolData_t patrol_read(void);

/* 初始化巡线模块 */
void patrol_init(void);

/* 设置巡线配置 */
void patrol_set_config(PatrolConfig_t *config);

/* 设置基础速度 */
void patrol_set_speed(float speed);

/* 获取当前位置误差（-100 ~ 100） */
float patrol_get_position(void);

/* 获取当前巡线状态 */
PatrolState_t patrol_get_state(void);

/* 惯性导航数据更新（预留接口） */
void patrol_update_inertial(PatrolInertialData_t *data);

/* 巡线主函数（需定时调用，建议10ms周期） */
void patrol_line(uint32_t dt_ms);

#endif
