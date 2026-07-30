#include "patrol.h"
#include "uart.h"

/* ==================== 全局变量 ==================== */
PatrolData_t patrol_data = {0};

/* ==================== 权重配置（从左到右） ====================
 * 物理顺序：r2 ---- r1 ---中线--- l1 ---- l2
 *   r2(左外) = -3   r1(左内) = -1   l1(右内) = +1   l2(右外) = +3
 *
 * 方向约定（与转向环一致，"右转为正"）：
 *   车偏左（线在车右侧）→ 右侧传感器触发 → 偏差为正 → 需右转修正
 *   车偏右（线在车左侧）→ 左侧传感器触发 → 偏差为负 → 需左转修正
 * ========================================================== */
#define PATROL_WEIGHT_R2   (-3.0f)
#define PATROL_WEIGHT_R1   (-1.0f)
#define PATROL_WEIGHT_L1   ( 1.0f)
#define PATROL_WEIGHT_L2   ( 3.0f)

/* ==================== 传感器读取 ==================== */
PatrolData_t patrol_read(void)
{
    /* 实际物理顺序（从左到右）：r2 ---- r1 ---中线--- l1 ---- l2 */
    patrol_data.r2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R2_PIN) & Patrol_R2_PIN) ? 1 : 0;
    patrol_data.r1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R1_PIN) & Patrol_R1_PIN) ? 1 : 0;
    patrol_data.l1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L1_PIN) & Patrol_L1_PIN) ? 1 : 0;
    patrol_data.l2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L2_PIN) & Patrol_L2_PIN) ? 1 : 0;
    return patrol_data;
}

/* ==================== 加权偏差计算 ==================== */
PatrolStatus_t patrol_get_error(float *error)
{
    PatrolData_t d = patrol_read();
    uint8_t sum = (uint8_t)(d.r2 + d.r1 + d.l1 + d.l2);

    /* 边界 1：全白 = 丢线 */
    if (sum == 0) {
        if (error) *error = 0.0f;
        return PATROL_LOST;
    }

    /* 边界 2：全黑 = 路口/十字 */
    if (sum == 4) {
        if (error) *error = 0.0f;
        return PATROL_JUNCTION;
    }

    /* 正常情况：加权平均
     * 例：仅 l2 触发 → (-3*0 + -1*0 + 1*0 + 3*1) / 1 = +3（车偏左，需右转）
     * 例：仅 r2 触发 → (-3*1 + -1*0 + 1*0 + 3*0) / 1 = -3（车偏右，需左转）
     * 例：r1+l1 同时触发 → (-1+1)/2 = 0（居中） */
    float numerator = (float)d.r2 * PATROL_WEIGHT_R2
                    + (float)d.r1 * PATROL_WEIGHT_R1
                    + (float)d.l1 * PATROL_WEIGHT_L1
                    + (float)d.l2 * PATROL_WEIGHT_L2;

    if (error) *error = numerator / (float)sum;
    return PATROL_OK;
}

/* ==================== 调试辅助 ==================== */
void patrol_get_raw(uint8_t *r2, uint8_t *r1, uint8_t *l1, uint8_t *l2)
{
    if (r2) *r2 = patrol_data.r2;
    if (r1) *r1 = patrol_data.r1;
    if (l1) *l1 = patrol_data.l1;
    if (l2) *l2 = patrol_data.l2;
}

/* ==================== 初始化 ==================== */
void patrol_init(void)
{
    patrol_data.r2 = 0;
    patrol_data.r1 = 0;
    patrol_data.l1 = 0;
    patrol_data.l2 = 0;
    uart_printf(UART0, "Patrol init: weighted error mode (r2=-3 r1=-1 l1=+1 l2=+3)\r\n");
}
