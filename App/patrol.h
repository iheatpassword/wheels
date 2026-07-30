#ifndef __PATROL_H__
#define __PATROL_H__

#include "ti_msp_dl_config.h"

/* ==================== 4路红外循迹传感器 ====================
 * 实际物理顺序（面向前方，从左到右）：r2 ---- r1 ---中线--- l1 ---- l2
 *   r2: 左外侧
 *   r1: 左内侧
 *   l1: 右内侧
 *   l2: 右外侧
 * 值：1 = 检测到黑线，0 = 未检测到（白底）
 * ========================================================== */
typedef struct {
    uint8_t r2:1;  /* 左外侧 */
    uint8_t r1:1;  /* 左内侧 */
    uint8_t l1:1;  /* 右内侧 */
    uint8_t l2:1;  /* 右外侧 */
} PatrolData_t;

/* 循迹状态（边界情况识别） */
typedef enum {
    PATROL_OK        = 0,  /* 正常：1~3 路检测到线，可计算偏差 */
    PATROL_LOST      = 1,  /* 丢线：4 路全白（车脱离轨迹）→ 停车 */
    PATROL_JUNCTION  = 2,  /* 路口/十字：4 路全黑 → 停车 */
} PatrolStatus_t;

extern PatrolData_t patrol_data;

/* ==================== 简单巡线模式（无PID，if-else查表法，调试用） ====================
 * 开关：patrol_simple_mode（0=PID方向环 steer_step，1=简单查表 patrol_simple_run）
 * 优点：不依赖调参，上电即可跑，适合先验证传感器/电机极性和物理安装是否正确。
 * 缺点：无平滑，有抖动，过弯不漂亮，仅用于 Bring-Up 阶段。
 *
 * 调用方式：wheels.c 的 steer_flag 分支里，按模式二选一：
 *   if (patrol_simple_mode) patrol_simple_run(base_speed_cnts);
 *   else                     steer_step(error, 20);
 * ========================================================================== */
extern volatile uint8_t patrol_simple_mode;

/* 获取/设置简单模式（simpletest 命令用） */
uint8_t patrol_get_simple_mode(void);
void    patrol_set_simple_mode(uint8_t on);

/**
 * @brief  简单查表法巡线（不经过 PID 方向环）
 *
 * 直接根据 4 路传感器的触发组合给左右轮分配固定目标速度，
 * 通过串口命令 simpletest <0|1> 和原 PID 方向环互斥切换。
 *
 * 策略（左→右 r2 r1 | l1 l2）：
 *   0000 (全白=丢线)          → 停车 (0, 0)
 *   1111 (全黑=路口)          → 停车 (0, 0)
 *   0010 / 0100 (居中：r1或l1单触发，或都触发) → 直行 (B, B)
 *   0110 (r1+l1 同时)         → 直行 (B, B)
 *   0001 (仅 l2 = 右外=车极度偏左) → 急右转 (B, 0.3B)
 *   0011 (l1+l2 = 右半边)     → 大右转 (B, 0.6B)
 *   0101 (r1+l2 少见组合)     → 按 r1优先(偏右) → 微左转
 *   0111 (r1+l1+l2 = 车偏左严重) → 大右转 (B, 0.5B)
 *   1000 (仅 r2 = 左外=车极度偏右) → 急左转 (0.3B, B)
 *   1100 (r2+r1 = 左半边)     → 大左转 (0.6B, B)
 *   1001 (r2+l2 对角)         → 直行 (安全退化)
 *   1010 (r2+l1)              → 直行
 *   1011 (r2+l1+l2)           → 右转
 *   1101 (r2+r1+l2)           → 左转
 *   1110 (r2+r1+l1)           → 左转
 *
 * @param  base_speed  基础前进速度 (counts/s，与 sbase 同一量级，例如 1500~2000)
 *                     内部自动把差速分配好，直接调用速度环的 speed_set_target。
 */
void patrol_simple_run(float base_speed);

/* ==================== 接口函数 ==================== */

/* 读取 4 路传感器原始数据 */
PatrolData_t patrol_read(void);

/* 计算加权偏差（转向环输入信号）
 *
 * 权重（从左到右）：r2=-3, r1=-1, l1=+1, l2=+3
 * 归一化方式：加权求和 / 检测到线的传感器数量
 * 输出范围：[-3, +3]
 *   正值 = 车偏左（线在右侧）→ 需右转修正
 *   负值 = 车偏右（线在左侧）→ 需左转修正
 *   0    = 居中（r1 与 l1 同时在线，或对称分布）
 *
 * 边界处理：
 *   全白 → 返回 PATROL_LOST，*error 置 0
 *   全黑 → 返回 PATROL_JUNCTION，*error 置 0
 *
 * @param error 输出加权偏差（仅 PATROL_OK 时有效，可为 NULL）
 * @return 当前循迹状态 */
PatrolStatus_t patrol_get_error(float *error);

/* 获取最近一次传感器原始数据（调试用） */
void patrol_get_raw(uint8_t *r2, uint8_t *r1, uint8_t *l1, uint8_t *l2);

/* 初始化循迹模块 */
void patrol_init(void);

#endif
