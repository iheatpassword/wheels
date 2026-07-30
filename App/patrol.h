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
