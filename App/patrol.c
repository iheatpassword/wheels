#include "patrol.h"
#include "uart.h"
#include "App/pid.h"

/* ==================== 全局变量 ==================== */
PatrolData_t patrol_data = {0};
volatile uint8_t patrol_simple_mode = 0;   /* 0=PID方向环, 1=简单查表法 */

uint8_t patrol_get_simple_mode(void) { return patrol_simple_mode; }
void    patrol_set_simple_mode(uint8_t on) { patrol_simple_mode = on ? 1 : 0; }

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
#define PATROL_WEIGHT_L2   ( 1.0f)
#define PATROL_WEIGHT_L1   ( 3.0f)

/* ==================== 传感器读取 ==================== */
PatrolData_t patrol_read(void)
{
    /* 实际物理顺序（从左到右）：r2 ---- r1 ---中线--- l2 ---- l1 */
    patrol_data.r2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R2_PIN) & Patrol_R2_PIN) ? 0 : 1;
    patrol_data.r1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_R1_PIN) & Patrol_R1_PIN) ? 0 : 1;
    patrol_data.l1 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L1_PIN) & Patrol_L1_PIN) ? 0 : 1;
    patrol_data.l2 = (DL_GPIO_readPins(Patrol_PORT, Patrol_L2_PIN) & Patrol_L2_PIN) ? 0 : 1;    
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

/* ==================== 简单查表法巡线（无 PID） ====================
 * 把 4 路传感器拼成 nibble：bits = [r2 r1 l1 l2]（MSB→LSB，左→右）
 * 然后用 switch 直接给左右轮分配 (目标左, 目标右) = (L*B, R*B) 的系数。
 * 内侧减速的转向策略：
 *   右转 = 左轮以 B 跑，右轮降速到 0.3~0.6B
 *   左转 = 右轮以 B 跑，左轮降速到 0.3~0.6B
 *   直行 = 两轮同速 B
 *   异常 = 停止
 *
 * 转向力度分三档：
 *   微转（单内侧触发，error=±1）  → 内侧 0.80B，1:0.8 差速，柔和
 *   大转（同侧两个，error=±2）   → 内侧 0.55B，1:0.55 差速，中等
 *   急转（仅最外侧，error=±3）   → 内侧 0.30B，1:0.3 差速，强横
 * ========================================================== */
static void patrol_apply(float base, float left_k, float right_k)
{
    const float LIM = 7999.0f;
    float tl = base * left_k;
    float tr = base * right_k;
    if (tl >  LIM) tl =  LIM;
    if (tl < -LIM) tl = -LIM;
    if (tr >  LIM) tr =  LIM;
    if (tr < -LIM) tr = -LIM;
    speed_set_target(&g_spd_left,  tl);
    speed_set_target(&g_spd_right, tr);

    /* 同步把左右目标更新到 steer.dbg，方便 gsteer / S: 串查看
     * （简单模式下 steer_step 不会执行，所以需要自己写快照） */
    g_steer.dbg.target_left  = tl;
    g_steer.dbg.target_right = tr;
}

void patrol_simple_run(float base_speed)
{
    /* 读一次传感器并同步到 steer 调试快照 */
    patrol_read();
    uint8_t r2 = patrol_data.r2;
    uint8_t r1 = patrol_data.r1;
    uint8_t l1 = patrol_data.l1;
    uint8_t l2 = patrol_data.l2;
    steer_update_sensors(r2, r1, l1, l2);

    /* 按 nibble 分发给系数 */
    uint8_t nibble = ((r2 & 1) << 3) | ((r1 & 1) << 2)
                   | ((l1 & 1) << 1) |  (l2 & 1);

    switch (nibble) {
    /* ---- 直行组（error≈0） ---- */
    case 0b0110: /* r1 + l1  居中 */
    case 0b0100: /* 仅 r1    略偏右 */
    case 0b0010: /* 仅 l1    略偏左 */
    case 0b1010: /* r2 + l1  交叉 */
    case 0b1001: /* r2 + l2  对角，退化直行 */
        g_steer.dbg.error = 0.0f;
        g_steer.dbg.turn  = 0.0f;
        steer_set_state(STEER_STRAIGHT);
        patrol_apply(base_speed, 1.0f, 1.0f);
        break;

    /* ---- 右转组（车偏左 → 右轮减速） ---- */
    case 0b0001: /* 仅 l2 = 最右外 → 急右转 error=+3 */
        g_steer.dbg.error =  3.0f;
        g_steer.dbg.turn  =  base_speed * 0.7f;
        steer_set_state(STEER_TURN_R);
        patrol_apply(base_speed, 1.0f, 0.30f); break;
    case 0b0011: /* l1 + l2 = 右半边 → 大右转 error=+2 */
        g_steer.dbg.error =  2.0f;
        g_steer.dbg.turn  =  base_speed * 0.45f;
        steer_set_state(STEER_TURN_R);
        patrol_apply(base_speed, 1.0f, 0.55f); break;
    case 0b1011: /* r2 + l1 + l2 → 右转（右侧占优） */
        g_steer.dbg.error =  1.0f / 3.0f;
        g_steer.dbg.turn  =  base_speed * 0.2f;
        steer_set_state(STEER_TURN_R);
        patrol_apply(base_speed, 1.0f, 0.80f); break;

    /* ---- 左转组（车偏右 → 左轮减速） ---- */
    case 0b1000: /* 仅 r2 = 最左外 → 急左转 error=-3 */
        g_steer.dbg.error = -3.0f;
        g_steer.dbg.turn  = -base_speed * 0.7f;
        steer_set_state(STEER_TURN_L);
        patrol_apply(base_speed, 0.30f, 1.0f); break;
    case 0b1100: /* r2 + r1 = 左半边 → 大左转 error=-2 */
        g_steer.dbg.error = -2.0f;
        g_steer.dbg.turn  = -base_speed * 0.45f;
        steer_set_state(STEER_TURN_L);
        patrol_apply(base_speed, 0.55f, 1.0f); break;
    case 0b1110: /* r2 + r1 + l1 → 左转（左侧占优） */
        g_steer.dbg.error = -1.0f / 3.0f;
        g_steer.dbg.turn  = -base_speed * 0.2f;
        steer_set_state(STEER_TURN_L);
        patrol_apply(base_speed, 0.80f, 1.0f); break;

    /* ---- 混合组：r1 + l2（略偏右但l2也触发，按偏左 → 右转微修） ---- */
    case 0b0101: /* r1 + l2 */
        g_steer.dbg.error =  1.0f;
        g_steer.dbg.turn  =  base_speed * 0.2f;
        steer_set_state(STEER_TURN_R);
        patrol_apply(base_speed, 1.0f, 0.80f); break;
    /* ---- 混合组：r2 + r1 + l2（偏右但l2也触发，左转大一点） ---- */
    case 0b1101: /* r2 + r1 + l2 */
        g_steer.dbg.error = -1.0f / 3.0f;
        g_steer.dbg.turn  = -base_speed * 0.25f;
        steer_set_state(STEER_TURN_L);
        patrol_apply(base_speed, 0.75f, 1.0f); break;
    /* ---- 混合组：r1 + l1 + l2（r1同时触发偏右，但右侧多 → 右转） ---- */
    case 0b0111: /* r1 + l1 + l2 */
        g_steer.dbg.error =  1.0f / 3.0f;
        g_steer.dbg.turn  =  base_speed * 0.25f;
        steer_set_state(STEER_TURN_R);
        patrol_apply(base_speed, 1.0f, 0.75f); break;

    /* ---- 全白 = 丢线 ---- */
    case 0b0000:
        g_steer.dbg.error = 0.0f;
        g_steer.dbg.turn  = 0.0f;
        steer_set_state(STEER_LOST);
        patrol_apply(0.0f, 0.0f, 0.0f);  /* 直接停，简单粗暴；要找线改成 (B,-B) 原地转 */
        break;
    /* ---- 全黑 = 路口 ---- */
    case 0b1111:
        g_steer.dbg.error = 0.0f;
        g_steer.dbg.turn  = 0.0f;
        steer_set_state(STEER_JUNCTION);
        patrol_apply(0.0f, 0.0f, 0.0f);
        break;

    default:
        /* 理论上 4bit 全覆盖不会进；安全起见直行 */
        steer_set_state(STEER_STRAIGHT);
        patrol_apply(base_speed, 1.0f, 1.0f);
        break;
    }

    /* 如果方向环每拍 S: 调试串开着，这里也手动打一条（简单模式下 steer_step 不执行） */
    if (steer_get_debug()) {
        uart_printf(UART0,
                    "S: STEP=%5u STATE=%-10s SEN=%d%d%d%d ERR=%+5.2f TURN=%+7.1f TL=%+7.1f TR=%+7.1f (SIMPLE)\r\n",
                    (unsigned)g_steer.dbg.step_cnt,
                    steer_state_name(g_steer.dbg.state),
                    r2, r1, l1, l2,
                    g_steer.dbg.error,
                    g_steer.dbg.turn,
                    g_steer.dbg.target_left,
                    g_steer.dbg.target_right);
    }
}

/* ==================== 初始化 ==================== */
void patrol_init(void)
{
    patrol_data.r2 = 0;
    patrol_data.r1 = 0;
    patrol_data.l1 = 0;
    patrol_data.l2 = 0;
    patrol_simple_mode = 0;   /* 默认还是 PID 方向环，需要简单模式用 simpletest 1 打开 */
    uart_printf(UART0, "Patrol init: weighted error mode (r2=-3 r1=-1 l1=+1 l2=+3) | simple=%s\r\n",
                patrol_simple_mode ? "ON" : "OFF");
}
