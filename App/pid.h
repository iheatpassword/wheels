#ifndef _PID_H_
#define _PID_H_

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"

/* ==========================================================================
 * 通用 PID 控制器 — 极简纯算法版
 *  - 输出硬限幅 [out_min, out_max]
 *  - 积分软限幅：integral_max
 *  - 无抗饱和、无低通滤波、无极性补偿
 * ========================================================================== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float last_error;
    float out_min;
    float out_max;
    float integral_max;
} PID_t;

/* ==========================================================================
 * 速度闭环：每 10ms 调用一次 speed_update()
 * 外部传入编码器增量 (delta) → 实测速度 → 一阶低通滤波 → PID → PWM 输出
 * ========================================================================== */
typedef struct {
    PID_t  pid;
    Motor_Channel_t ch;
    float  speed;          /* 滤波后的速度（用于 PID 反馈） */
    float  speed_raw;      /* 原始未经滤波的速度（用于调试） */
    float  filter_alpha;   /* 低通系数：1=无滤波, 0.5=中等, 0.1=强滤波 */
    float  motor_polarity; /* PWM 输出极性：+1.0 正常, -1.0 翻转（补偿硬件 BIN1/BIN2 反接） */
    int32_t last_delta;
    float  last_out;       /* speed_update 最近一次实际输出（限幅后，下发 PWM 前） */
} MotorSpeed_t;

/* 方向环运行状态（便于调试/日志/上位机可视化） */
typedef enum {
    STEER_STOPPED   = 0,  /* 已停车 / 未激活 */
    STEER_STRAIGHT  = 1,  /* 直线行驶（|error| 小） */
    STEER_TURN_L    = 2,  /* 左转修正中（error < 0） */
    STEER_TURN_R    = 3,  /* 右转修正中（error > 0） */
    STEER_LOST      = 4,  /* 丢线保护（PATROL_LOST） */
    STEER_JUNCTION  = 5,  /* 路口/十字（PATROL_JUNCTION） */
} SteerState_t;

/* 方向环调试快照（steer_step 每拍更新，供 gsteer / S: 输出读取） */
typedef struct {
    float          error;         /* 最近一次循迹加权误差 [-3, +3] */
    float          turn;          /* 最近一次 PID 差速输出（限幅后） */
    float          target_left;   /* 下发给左轮速度环的目标（counts/s） */
    float          target_right;  /* 下发给右轮速度环的目标（counts/s） */
    uint8_t        sen_r2;        /* 传感器原始位（从 patrol 快照） */
    uint8_t        sen_r1;
    uint8_t        sen_l1;
    uint8_t        sen_l2;
    SteerState_t   state;         /* 当前运行状态 */
    uint8_t        debug_enable;  /* 1 = 每拍打 "S: ..." 调试串 */
    uint16_t       step_cnt;      /* steer_step 调用计数（用于分频输出等） */
} SteerDebug_t;

/* ==========================================================================
 * 方向环：每 20ms 调用一次 steer_step()
 * 循迹误差 ∈ [-3, +3] → PID → 差速补偿 → 叠加到左右轮基础速度
 * ========================================================================== */
typedef struct {
    PID_t        pid;
    float        base_speed;
    float        max_turn;
    SteerDebug_t dbg;            /* 调试快照 + 运行状态 */
} Steer_t;

/* 全局实例 */
extern MotorSpeed_t g_spd_left;
extern MotorSpeed_t g_spd_right;
extern Steer_t      g_steer;

/* ---------------- 通用 PID ---------------- */
void  pid_begin(PID_t *p, float kp, float ki, float kd,
                float out_max, float integral_max);
void  pid_reset(PID_t *p);
float pid_step(PID_t *p, float setpoint, float measure, float dt);

/* ---------------- 速度环 ---------------- */
void  speed_init(MotorSpeed_t *s, Motor_Channel_t ch,
                 float kp, float ki, float kd);
void  speed_set_target(MotorSpeed_t *s, float target);
void  speed_set_kp(MotorSpeed_t *s, float kp);
void  speed_set_ki(MotorSpeed_t *s, float ki);
void  speed_set_kd(MotorSpeed_t *s, float kd);
void  speed_set_filter(MotorSpeed_t *s, float alpha);
void  speed_set_motor_polarity(MotorSpeed_t *s, float polarity);
void  speed_get_params(MotorSpeed_t *s, float *kp, float *ki, float *kd);
float speed_get_speed(MotorSpeed_t *s);
void  speed_update(MotorSpeed_t *s, int32_t delta, float dt_ms);
void  speed_stop(MotorSpeed_t *s);

/* ---------------- 方向环 ---------------- */
void         steer_init(float kp, float ki, float kd, float max_turn);
void         steer_set_base(float base);
void         steer_set_kp(float kp);
void         steer_set_ki(float ki);
void         steer_set_kd(float kd);
void         steer_set_param(float kp, float ki, float kd);
void         steer_get_param(float *kp, float *ki, float *kd, float *base, float *max_turn);
void         steer_step(float error, float dt_ms);
void         steer_stop(void);
/* 方向环调试接口 */
void         steer_set_debug(uint8_t enable);                  /* sdebug 0/1 开关每拍调试输出 */
uint8_t      steer_get_debug(void);                            /* 查询调试开关状态 */
void         steer_reset(void);                                /* 状态复位：PID清零 + state=STOPPED */
void         steer_update_sensors(uint8_t r2, uint8_t r1, uint8_t l1, uint8_t l2); /* wheels.c 中每次读传感器后调用 */
void         steer_set_state(SteerState_t st);                 /* 丢线/路口等外部条件改变状态时调用 */
const char*  steer_state_name(SteerState_t st);                /* 状态名 → "STRAIGHT"/"TURN_L"/... 便于打印 */
void         steer_print_status(void);                         /* 打印完整方向环状态（gsteer 命令用） */

/* ---------------- 应用层 ---------------- */
void  pid_app_init(void);

#endif /* _PID_H_ */