#include "ball_control.h"

#include "bsp_uart.h"

/*
 * H3 钢球控制调参面板
 *
 * 控制结构：
 *   位置环 P  -> 目标速度
 *   速度环 P  -> 舵机脉宽偏移量
 *   补偿机制  -> 位置增益 + 方向增益 + 近目标软区
 *
 * 注意：这里先用纯P控制，不加I和D。
 *       因为球-板系统本身已经包含舵机角度 -> 加速度 -> 速度 -> 位置的积分链。
 */

/* ==================== 1) 外环：位置环P控制 ==================== */
// 作用：位置误差(mm) -> 目标速度(mm/s)
#define BALL_CONTROL_OUTER_KP_NUM                  (3)
#define BALL_CONTROL_OUTER_KP_DEN                  (2)
// 外环比例系数 KP = 3/2 = 1.5
// 含义：球偏离目标 1mm，就命令它以 1.5mm/s 的速度滚回去
// 调大：归位更快，但容易冲过头/震荡
// 调小：归位更慢，但更稳定

#define BALL_CONTROL_TARGET_SPEED_LIMIT_MM_PER_S   (150)
// 目标速度限幅：±150 mm/s
// 调大：远距离移动更快，反应更迅速
// 调小：靠近目标时更温柔，减少冲过头


/* ==================== 2) 内环：速度环P控制 ==================== */
// 作用：速度误差(mm/s) -> 舵机脉宽偏移量(us)
#define BALL_CONTROL_INNER_KP_POS_NUM              (4)
// 正方向（球向右/正方向滚）速度环增益
// 如果球向正方向推不动，增大此值

#define BALL_CONTROL_INNER_KP_NEG_NUM              (5)
// 负方向（球向左/负方向滚）速度环增益
// 如果从 +50mm 回 -50mm 时停在 -30mm 推不动，增大此值

#define BALL_CONTROL_INNER_KP_DEN                  (1)
// 内环比例系数的分母（正反向共用）
// 所以正向增益 = 4/1 = 4，负向增益 = 5/1 = 5

#define BALL_CONTROL_OUTPUT_POS_LIMIT_US           (220)
// 正方向最大舵机偏移量：+220 μs
// 如果正方向冲过头，减小此值

#define BALL_CONTROL_OUTPUT_NEG_LIMIT_US           (230)
// 负方向最大舵机偏移量：-230 μs
// 如果负方向冲过头，减小此值


/* ==================== 3) 最小输出（克服静摩擦） ==================== */
// 作用：当偏差较大时，即使算法算出的输出很小，也强制给一个最小推力
#define BALL_CONTROL_MIN_OUTPUT_ENABLE_ERROR_MM    (6)
// 最小输出启用阈值：|偏差| > 6mm 时才启用
// 调大：在更大的范围内都会给最小推力
// 调小：只在更远的距离才给最小推力

#define BALL_CONTROL_MIN_OUTPUT_POS_US             (0)
// 正方向最小推力：+90 μs
// 作用：当球从 -30mm 向 0mm 移动时，如果推不动就增大此值
// 如果推过头（直接冲过0），就减小此值

#define BALL_CONTROL_MIN_OUTPUT_NEG_US             (0)
// 负方向最小推力：-70 μs
// 作用：当球从 +50mm 向 -50mm 移动时，如果停在 -30mm 推不动就增大此值
// 如果直接冲过 -50mm，就减小此值


/* ==================== 4) 位置增益补偿（机械不对称补偿） ==================== */
// 背景：舵机安装在 +78/+79mm 位置，导致：
//       球在正方向时舵机力矩大（少给力）
//       球在负方向时舵机力矩小（多给力）
#define BALL_CONTROL_POS_STRONG_MM                 (80)
#define BALL_CONTROL_NEG_WEAK_MM                   (-80)
// 正强区边界：+80mm，负弱区边界：-80mm

#define BALL_CONTROL_GAIN_FAR_POS_PERCENT          (100)
// 球在 +80mm 以外（正远端）：舵机力矩最大，输出衰减到 70%
// 如果 +100mm 附近拉不回来，增大此值（70→80）
// 如果 +100mm 附近太猛，减小此值（70→60）

#define BALL_CONTROL_GAIN_MID_POS_PERCENT          (100)
// 球在 0~+80mm（正中段）：舵机力矩较强，输出衰减到 90%
// 如果正半区反应慢，增大此值（90→100）
// 如果正半区来回晃，减小此值（90→80）

#define BALL_CONTROL_GAIN_MID_NEG_PERCENT          (100)
// 球在 -80~0mm（负中段）：舵机力矩较弱，输出增强到 135%
// 如果负半区回调慢、推不动，增大此值（135→145）
// 如果负半区来回晃，减小此值（135→125）

#define BALL_CONTROL_GAIN_FAR_NEG_PERCENT          (100)
// 球在 -80mm 以外（负远端）：舵机力矩最弱，输出增强到 140%
// 如果 -80~-115mm 拉不回来，增大此值（140→150）
// 如果 -80~-115mm 冲太多，减小此值（140→130）


/* ==================== 5) 近目标软控制 ==================== */
// 作用：当球接近目标时，削弱控制，防止震荡，实现平稳收敛
#define BALL_CONTROL_DEAD_ZONE_MM                  (2)
// 死区：|偏差| ≤ 5mm 且速度很慢时，输出强制为0
// 如果目标附近一直小抖，增大此值（5→6/7）
// 如果停得太早（误差偏大），减小此值（5→4/3）

#define BALL_CONTROL_DEAD_ZONE_SPEED_MM_PER_S      (30)
// 死区速度条件：|速度| ≤ 20mm/s 才进入死区
// 如果目标附近还在抖，增大此值（20→30）
// 如果进目标时刹不住，减小此值（20→10）

#define BALL_CONTROL_SOFT_ZONE_MM                  (0)
// 软区范围：|偏差| ≤ 12mm 进入软区
// 调大：更早开始温柔控制（更平滑）
// 调小：更晚开始温柔控制（响应更快）

#define BALL_CONTROL_SOFT_ZONE_GAIN_PERCENT        (100)
// 软区输出比例：输出衰减到 70%
// 如果接近目标但不收敛，增大此值（70→80）
// 如果接近目标还冲来冲去，减小此值（70→60）

#define BALL_CONTROL_SOFT_ZONE_LIMIT_US            (500)
// 软区最大输出：±110 μs
// 如果近目标推不动，增大此值（110→130）
// 如果近目标晃幅大，减小此值（110→90）


/* ==================== 6) 提前刹车（远处猛推，近处提前收） ==================== */
// 作用：解决"力度够但收不住"——球快到目标时先反向刹车，而不是冲过去再拉回来
// 这不是D项，而是用视觉算出的球速做提前收力（老师说的只用PI不冲突）
/* 刹车只在目标已经到 +/-50 后启用，避免 target=0/-5 的过渡段误触发。 */
#define BALL_CONTROL_BRAKE_TARGET_ACTIVE_MM        (50)

/* 第一阶段 0 -> -50：根据每次进入 -30mm 附近的速度动态算刹车。
 * 入口速度 <=60mm/s：基本不反推，只轻微衰减。
 * 入口速度越高：反向目标速度越大、目标速度衰减越强。
 */
#define BALL_CONTROL_BRAKE_NEG_ZONE_MM             (20)
#define BALL_CONTROL_BRAKE_NEG_ENTRY_MIN_MM_PER_S  (90)
#define BALL_CONTROL_BRAKE_NEG_REVERSE_DIV         (8)
#define BALL_CONTROL_BRAKE_NEG_REVERSE_MAX_MM_PER_S (20)
#define BALL_CONTROL_BRAKE_NEG_GAIN_MAX_PERCENT    (98)
#define BALL_CONTROL_BRAKE_NEG_GAIN_MIN_PERCENT    (80)
#define BALL_CONTROL_BRAKE_NEG_GAIN_DIV            (10)

/* 第二阶段 -50 -> +50：10次实测进入 +30mm 附近速度多在 76~125mm/s。
 * 比第一阶段更温和：只拦快冲，避免把球刹回 +40。
 */
#define BALL_CONTROL_BRAKE_POS_ZONE_MM             (0)
#define BALL_CONTROL_BRAKE_POS_ENTRY_MIN_MM_PER_S  (70)
#define BALL_CONTROL_BRAKE_POS_REVERSE_DIV         (5)
#define BALL_CONTROL_BRAKE_POS_REVERSE_MAX_MM_PER_S (35)
#define BALL_CONTROL_BRAKE_POS_GAIN_MAX_PERCENT    (92)
#define BALL_CONTROL_BRAKE_POS_GAIN_MIN_PERCENT    (60)
#define BALL_CONTROL_BRAKE_POS_GAIN_DIV            (6)
#define BALL_CONTROL_BRAKE_PROBE_LOG_ENABLE        (1U)

#define BALL_CONTROL_POS_TARGET_MM                  (50)
#define BALL_CONTROL_POS_CATCH_ZONE_MM              (25)
#define BALL_CONTROL_POS_CATCH_FINE_ZONE_MM         (10)
#define BALL_CONTROL_POS_CATCH_KP_NUM               (2)
#define BALL_CONTROL_POS_CATCH_KP_DEN               (1)
#define BALL_CONTROL_POS_CATCH_SPEED_LIMIT_MM_PER_S (32)
#define BALL_CONTROL_POS_CATCH_FINE_SPEED_LIMIT_MM_PER_S (22)
#define BALL_CONTROL_POS_CATCH_OUTPUT_LIMIT_US      (100)
#define BALL_CONTROL_POS_CATCH_FINE_OUTPUT_LIMIT_US (90)



/* ==================== 7) 舵机输出变化率限制 ==================== */
#define BALL_CONTROL_OUTPUT_STEP_LIMIT_US          (500)
// 每帧（每次视觉数据更新）舵机脉宽最大变化量：±70 μs
// 调小：控制更平滑，但响应更慢（有延迟）
// 调大：响应更快，但更容易抖动/震荡
// 如果舵机突然猛打、球被甩出去，减小此值


/* ==================== 7) 输入安全与方向 ==================== */
#define BALL_CONTROL_MEASURED_SPEED_LIMIT_MM_PER_S (800)
// 速度测量上限：±800 mm/s
// 如果传感器测出的速度超过此值，视为噪声/干扰，直接忽略
// 防止异常数据导致失控

#define BALL_CONTROL_SAMPLE_MIN_MS                 (10U)
// 最小控制周期：10ms
// 如果两次控制间隔 <10ms，说明调用太频繁，不进行速度计算
// 原因：舵机响应有限，太快了来不及动作

#define BALL_CONTROL_SAMPLE_MAX_MS                 (150U)
// 最大控制周期：150ms
// 如果两次控制间隔 >150ms，说明程序卡顿或丢帧
// 强制截断到150ms，防止速度计算数值爆炸

#define BALL_CONTROL_SERVO_DIRECTION               (1)
// 舵机方向系数
// = 1：正向控制（误差越大舵机正向偏转）
// = -1：反向控制（如果舵机装反了，改成 -1 就能修正）
// 如果球往两个方向都远离目标，就把 1 改成 -1


/* ==================== 调参顺序建议 ====================
 *
 * 第一步：调"能不能到目标"
 *   到不了 -50mm：调大 GAIN_MID_NEG_PERCENT 或 MIN_OUTPUT_NEG_US
 *   到不了 +50mm：调大 GAIN_MID_POS_PERCENT 或 MIN_OUTPUT_POS_US
 *
 * 第二步：调"目标附近稳不稳"
 *   接近目标时来回冲：降低 SOFT_ZONE_GAIN_PERCENT (70→60)
 *   还冲：降低 SOFT_ZONE_LIMIT_US (120→100)
 *
 * 第三步：调"极近距离小抖"
 *   小范围抖：增大 DEAD_ZONE_MM (5→6)
 *   停得太早：减小 DEAD_ZONE_MM (5→4)
 *
 * 第四步（最后动这两个）：
 *   BALL_CONTROL_OUTER_KP_NUM/DEN：整体太慢→增大，整体太冲→减小
 *   BALL_CONTROL_TARGET_SPEED_LIMIT：整体响应慢→增大，容易冲过头→减小
 * ==================================================== */



static uint16_t _LimitPulse(int32_t pulseUs);
static int16_t _LimitSigned(int32_t value, int16_t limit);
static int16_t _Abs16(int16_t value);
static int16_t _ScalePercent(int16_t value, int16_t percent);
static int16_t _GetPositionGainPercent(int16_t ballMm);
static int16_t _ApplyMinimumOutput(int16_t outputUs, int16_t errorMm);
static int16_t _ApplyNearTargetControl(int16_t outputUs, int16_t errorMm,
    int16_t ballSpeedMmPerSec);
static int16_t _ApplyBrakeZone(BallControl_t *pControl,
    int16_t targetSpeedMmPerSec, int16_t targetMm, int16_t ballMm,
    int16_t errorMm, int16_t ballSpeedMmPerSec);
static int16_t _ApplyOutputStepLimit(const BallControl_t *pControl,
    int16_t outputUs);
static int16_t _CalcInnerOutputUs(const BallControl_t *pControl,
    int16_t positionErrorMm, int16_t ballMm, int16_t speedErrorMmPerSec);

void BallControl_Init(BallControl_t *pControl)
{
    BallControl_Reset(pControl);
}

void BallControl_Reset(BallControl_t *pControl)
{
    if (pControl == 0) {
        return;
    }

    pControl->lastBallMm = 0;
    pControl->targetSpeedMmPerSec = 0;
    pControl->ballSpeedMmPerSec = 0;
    pControl->lastFrameSeq = 0U;
    pControl->lastSampleMs = 0U;
    pControl->lastPulseUs = BSP_SERVO_PULSE_CENTER_US;
    pControl->hasFrame = 0U;
    pControl->brakeZoneActive = 0U;
    pControl->brakeReverseActive = 0U;
    pControl->brakeEntrySpeedMmPerSec = 0;
    pControl->brakeDynamicReverseMmPerSec = 0;
    pControl->brakeDynamicGainPercent = 100;
}

uint16_t BallControl_Update(BallControl_t *pControl,
    int16_t targetMm, int16_t ballMm, uint32_t frameSeq,
    uint32_t nowMs, uint8_t valid)
{
    int16_t positionErrorMm;
    int16_t speedErrorMmPerSec;
    int16_t pulseOffsetUs;
    uint32_t sampleMs;
    int32_t pulseUs;

    if ((pControl == 0) || (valid == 0U)) {
        if (pControl != 0) {
            BallControl_Reset(pControl);
        }
        return BSP_SERVO_PULSE_CENTER_US;
    }

    if ((pControl->hasFrame != 0U) &&
        (frameSeq == pControl->lastFrameSeq)) {
        return pControl->lastPulseUs;
    }

    positionErrorMm = (int16_t)(targetMm - ballMm);

    /* 先算球速：提前刹车要用它来判断"是不是正在冲向目标" */
    if (pControl->hasFrame != 0U) {
        sampleMs = nowMs - pControl->lastSampleMs;
        if ((sampleMs >= BALL_CONTROL_SAMPLE_MIN_MS) &&
            (sampleMs <= BALL_CONTROL_SAMPLE_MAX_MS)) {
            pControl->ballSpeedMmPerSec = _LimitSigned(
                ((int32_t)(ballMm - pControl->lastBallMm) * 1000L) /
                    (int32_t)sampleMs,
                BALL_CONTROL_MEASURED_SPEED_LIMIT_MM_PER_S);
        } else {
            pControl->ballSpeedMmPerSec = 0;
        }
    } else {
        pControl->ballSpeedMmPerSec = 0;
    }

    /* 位置环：误差 -> 目标速度 */
    pControl->targetSpeedMmPerSec = _LimitSigned(
        ((int32_t)positionErrorMm * BALL_CONTROL_OUTER_KP_NUM) /
            BALL_CONTROL_OUTER_KP_DEN,
        BALL_CONTROL_TARGET_SPEED_LIMIT_MM_PER_S);

    /* 提前刹车：接近目标且冲得太快时，收力甚至反向 */
    pControl->targetSpeedMmPerSec = _ApplyBrakeZone(pControl,
        pControl->targetSpeedMmPerSec, targetMm, ballMm, positionErrorMm,
        pControl->ballSpeedMmPerSec);

    speedErrorMmPerSec = (int16_t)(
        pControl->targetSpeedMmPerSec - pControl->ballSpeedMmPerSec);
    pulseOffsetUs = _CalcInnerOutputUs(pControl, positionErrorMm, ballMm,
        speedErrorMmPerSec);

    pulseUs = (int32_t)BSP_SERVO_PULSE_CENTER_US +
        ((int32_t)BALL_CONTROL_SERVO_DIRECTION * pulseOffsetUs);

    pControl->lastBallMm = ballMm;
    pControl->lastFrameSeq = frameSeq;
    pControl->lastSampleMs = nowMs;
    pControl->lastPulseUs = _LimitPulse(pulseUs);
    pControl->hasFrame = 1U;

    return pControl->lastPulseUs;
}

int16_t BallControl_GetSpeedMmPerSec(const BallControl_t *pControl)
{
    return (pControl == 0) ? 0 : pControl->ballSpeedMmPerSec;
}

static int16_t _CalcInnerOutputUs(const BallControl_t *pControl,
    int16_t positionErrorMm, int16_t ballMm, int16_t speedErrorMmPerSec)
{
    int32_t outputUs;

    if (speedErrorMmPerSec >= 0) {
        outputUs =
            ((int32_t)speedErrorMmPerSec * BALL_CONTROL_INNER_KP_POS_NUM) /
            BALL_CONTROL_INNER_KP_DEN;
        outputUs = _LimitSigned(outputUs, BALL_CONTROL_OUTPUT_POS_LIMIT_US);
    } else {
        outputUs =
            ((int32_t)speedErrorMmPerSec * BALL_CONTROL_INNER_KP_NEG_NUM) /
            BALL_CONTROL_INNER_KP_DEN;
        outputUs = _LimitSigned(outputUs, BALL_CONTROL_OUTPUT_NEG_LIMIT_US);
    }

    outputUs = _ScalePercent((int16_t)outputUs,
        _GetPositionGainPercent(ballMm));
    outputUs = _ApplyMinimumOutput((int16_t)outputUs, positionErrorMm);
    if (outputUs > 0) {
        outputUs = _LimitSigned(outputUs, BALL_CONTROL_OUTPUT_POS_LIMIT_US);
    } else if (outputUs < 0) {
        outputUs = _LimitSigned(outputUs, BALL_CONTROL_OUTPUT_NEG_LIMIT_US);
    }
    outputUs = _ApplyNearTargetControl((int16_t)outputUs, positionErrorMm,
        (pControl == 0) ? 0 : pControl->ballSpeedMmPerSec);

    if (((int16_t)(ballMm + positionErrorMm) >= BALL_CONTROL_POS_TARGET_MM) &&
        (_Abs16(positionErrorMm) <= BALL_CONTROL_POS_CATCH_ZONE_MM)) {
        if (_Abs16(positionErrorMm) <= BALL_CONTROL_POS_CATCH_FINE_ZONE_MM) {
            outputUs = _LimitSigned(outputUs,
                BALL_CONTROL_POS_CATCH_FINE_OUTPUT_LIMIT_US);
        } else {
            outputUs = _LimitSigned(outputUs,
                BALL_CONTROL_POS_CATCH_OUTPUT_LIMIT_US);
        }
    }

    outputUs = _ApplyOutputStepLimit(pControl, (int16_t)outputUs);

    return (int16_t)outputUs;
}

static int16_t _ApplyMinimumOutput(int16_t outputUs, int16_t errorMm)
{
    int16_t absError = _Abs16(errorMm);

    if (absError <= BALL_CONTROL_MIN_OUTPUT_ENABLE_ERROR_MM) {
        return outputUs;
    }

    if ((outputUs > 0) && (outputUs < BALL_CONTROL_MIN_OUTPUT_POS_US)) {
        return BALL_CONTROL_MIN_OUTPUT_POS_US;
    }
    if ((outputUs < 0) && (outputUs > -BALL_CONTROL_MIN_OUTPUT_NEG_US)) {
        return (int16_t)-BALL_CONTROL_MIN_OUTPUT_NEG_US;
    }

    return outputUs;
}

static int16_t _ApplyNearTargetControl(int16_t outputUs, int16_t errorMm,
    int16_t ballSpeedMmPerSec)
{
    int16_t absError = _Abs16(errorMm);
    int16_t absSpeed = _Abs16(ballSpeedMmPerSec);

    if ((absError <= BALL_CONTROL_DEAD_ZONE_MM) &&
        (absSpeed <= BALL_CONTROL_DEAD_ZONE_SPEED_MM_PER_S)) {
        return 0;
    }

    if (absError <= BALL_CONTROL_SOFT_ZONE_MM) {
        outputUs = _ScalePercent(outputUs, BALL_CONTROL_SOFT_ZONE_GAIN_PERCENT);
        outputUs = _LimitSigned(outputUs, BALL_CONTROL_SOFT_ZONE_LIMIT_US);
    }

    return outputUs;
}

static int16_t _ApplyBrakeZone(BallControl_t *pControl,
    int16_t targetSpeedMmPerSec, int16_t targetMm, int16_t ballMm,
    int16_t errorMm, int16_t ballSpeedMmPerSec)
{
    int16_t absError = _Abs16(errorMm);
    int16_t absSpeed = _Abs16(ballSpeedMmPerSec);
    int16_t brakeZoneMm;
	
	/* 第二阶段 +50 专用捕获：进入捕获区后改成低速位置环，避免 60~40 来回振荡 */
if (targetMm >= BALL_CONTROL_POS_TARGET_MM) {
    int16_t distanceMm = _Abs16((int16_t)(targetMm - ballMm));

    if (distanceMm <= BALL_CONTROL_POS_CATCH_ZONE_MM) {
        int16_t catchTargetSpeedMmPerSec;
        int16_t catchSpeedLimitMmPerSec;

        catchTargetSpeedMmPerSec = (int16_t)(
            ((int32_t)(targetMm - ballMm) *
             BALL_CONTROL_POS_CATCH_KP_NUM) /
            BALL_CONTROL_POS_CATCH_KP_DEN);

        if (distanceMm <= BALL_CONTROL_POS_CATCH_FINE_ZONE_MM) {
            catchSpeedLimitMmPerSec =
                BALL_CONTROL_POS_CATCH_FINE_SPEED_LIMIT_MM_PER_S;
        } else {
            catchSpeedLimitMmPerSec =
                BALL_CONTROL_POS_CATCH_SPEED_LIMIT_MM_PER_S;
        }

        if (catchTargetSpeedMmPerSec > catchSpeedLimitMmPerSec) {
            catchTargetSpeedMmPerSec = catchSpeedLimitMmPerSec;
        } else if (catchTargetSpeedMmPerSec < -catchSpeedLimitMmPerSec) {
            catchTargetSpeedMmPerSec = (int16_t)-catchSpeedLimitMmPerSec;
        }

        if (pControl != 0) {
            pControl->brakeReverseActive = 0U;
        }

        return catchTargetSpeedMmPerSec;
    }
}
	
    int16_t brakeReverseMmPerSec;
    int16_t brakeGainPercent;
    int16_t entrySpeedMmPerSec;
    int16_t excessSpeedMmPerSec;

    if (_Abs16(targetMm) < BALL_CONTROL_BRAKE_TARGET_ACTIVE_MM) {
        if (pControl != 0) {
            pControl->brakeZoneActive = 0U;
            pControl->brakeReverseActive = 0U;
        }
        return targetSpeedMmPerSec;
    }

    if (targetMm < 0) {
        brakeZoneMm = BALL_CONTROL_BRAKE_NEG_ZONE_MM;
        brakeReverseMmPerSec = 0;
        brakeGainPercent = BALL_CONTROL_BRAKE_NEG_GAIN_MAX_PERCENT;
    } else if (targetMm > 0) {
        brakeZoneMm = BALL_CONTROL_BRAKE_POS_ZONE_MM;
        brakeReverseMmPerSec = 0;
        brakeGainPercent = BALL_CONTROL_BRAKE_POS_GAIN_MAX_PERCENT;
    } else {
        if (pControl != 0) {
            pControl->brakeZoneActive = 0U;
            pControl->brakeReverseActive = 0U;
        }
        return targetSpeedMmPerSec;
    }

    /* 远离刹车区：保持原来的大力度，不动 */
    if ((brakeZoneMm <= 0) || (absError > brakeZoneMm)) {
        if (pControl != 0) {
            pControl->brakeZoneActive = 0U;
            pControl->brakeReverseActive = 0U;
        }
        return targetSpeedMmPerSec;
    }

    /* 速度被限幅到最大值时，多半是视觉跳点；不要用它计算动态刹车。 */
    if (absSpeed >= BALL_CONTROL_MEASURED_SPEED_LIMIT_MM_PER_S) {
        if (pControl != 0) {
            pControl->brakeReverseActive = 0U;
        }
        return targetSpeedMmPerSec;
    }

    /* 球正在远离目标，不能把这次速度记录成刹车入口速度。 */
    if (((int32_t)errorMm * (int32_t)ballSpeedMmPerSec) <= 0L) {
        if (pControl != 0) {
            pControl->brakeReverseActive = 0U;
        }
        return targetSpeedMmPerSec;
    }

    if ((pControl != 0) && (pControl->brakeZoneActive == 0U)) {
        pControl->brakeZoneActive = 1U;
        pControl->brakeReverseActive = 0U;
        entrySpeedMmPerSec = absSpeed;

        if (targetMm < 0) {
            excessSpeedMmPerSec = (int16_t)(
                entrySpeedMmPerSec - BALL_CONTROL_BRAKE_NEG_ENTRY_MIN_MM_PER_S);
            if (excessSpeedMmPerSec < 0) {
                excessSpeedMmPerSec = 0;
            }

            brakeReverseMmPerSec = (int16_t)(
                excessSpeedMmPerSec / BALL_CONTROL_BRAKE_NEG_REVERSE_DIV);
            if (brakeReverseMmPerSec >
                BALL_CONTROL_BRAKE_NEG_REVERSE_MAX_MM_PER_S) {
                brakeReverseMmPerSec =
                    BALL_CONTROL_BRAKE_NEG_REVERSE_MAX_MM_PER_S;
            }

            brakeGainPercent = (int16_t)(
                BALL_CONTROL_BRAKE_NEG_GAIN_MAX_PERCENT -
                (excessSpeedMmPerSec / BALL_CONTROL_BRAKE_NEG_GAIN_DIV));
            if (brakeGainPercent < BALL_CONTROL_BRAKE_NEG_GAIN_MIN_PERCENT) {
                brakeGainPercent = BALL_CONTROL_BRAKE_NEG_GAIN_MIN_PERCENT;
            }
        } else if (targetMm > 0) {
            excessSpeedMmPerSec = (int16_t)(
                entrySpeedMmPerSec - BALL_CONTROL_BRAKE_POS_ENTRY_MIN_MM_PER_S);
            if (excessSpeedMmPerSec < 0) {
                excessSpeedMmPerSec = 0;
            }

            brakeReverseMmPerSec = (int16_t)(
                excessSpeedMmPerSec / BALL_CONTROL_BRAKE_POS_REVERSE_DIV);
            if (brakeReverseMmPerSec >
                BALL_CONTROL_BRAKE_POS_REVERSE_MAX_MM_PER_S) {
                brakeReverseMmPerSec =
                    BALL_CONTROL_BRAKE_POS_REVERSE_MAX_MM_PER_S;
            }

            brakeGainPercent = (int16_t)(
                BALL_CONTROL_BRAKE_POS_GAIN_MAX_PERCENT -
                (excessSpeedMmPerSec / BALL_CONTROL_BRAKE_POS_GAIN_DIV));
            if (brakeGainPercent < BALL_CONTROL_BRAKE_POS_GAIN_MIN_PERCENT) {
                brakeGainPercent = BALL_CONTROL_BRAKE_POS_GAIN_MIN_PERCENT;
            }
        }

        pControl->brakeEntrySpeedMmPerSec = entrySpeedMmPerSec;
        pControl->brakeDynamicReverseMmPerSec = brakeReverseMmPerSec;
        pControl->brakeDynamicGainPercent = brakeGainPercent;

        if (BALL_CONTROL_BRAKE_PROBE_LOG_ENABLE != 0U) {
            BspUart_Printf("[H3B] zone target=%d ball=%d err=%d speed=%d cmd=%d entry=%d rev=%d gain=%d\n",
                (int)targetMm,
                (int)ballMm,
                (int)errorMm,
                (int)ballSpeedMmPerSec,
                (int)targetSpeedMmPerSec,
                (int)pControl->brakeEntrySpeedMmPerSec,
                (int)pControl->brakeDynamicReverseMmPerSec,
                (int)pControl->brakeDynamicGainPercent);
        }
    }

    if (pControl != 0) {
        brakeReverseMmPerSec = pControl->brakeDynamicReverseMmPerSec;
        brakeGainPercent = pControl->brakeDynamicGainPercent;
    }

    /* 正在冲向目标：按进入刹车区时的速度动态给反向目标速度。 */
    if ((brakeReverseMmPerSec > 0) &&
        (absError > BALL_CONTROL_DEAD_ZONE_MM)) {
        if (pControl != 0) {
            pControl->brakeReverseActive = 1U;
        }
        if (errorMm > 0) {
            return (int16_t)-brakeReverseMmPerSec;
        }
        if (errorMm < 0) {
            return brakeReverseMmPerSec;
        }
    }

    /* 接近目标但速度可控：只收力，不反推 */
    if (pControl != 0) {
        pControl->brakeReverseActive = 0U;
    }
    return _ScalePercent(targetSpeedMmPerSec, brakeGainPercent);
}

static int16_t _ApplyOutputStepLimit(const BallControl_t *pControl,
    int16_t outputUs)
{
    int16_t lastOutputUs;
    int16_t deltaUs;

    if ((pControl == 0) || (pControl->hasFrame == 0U)) {
        return outputUs;
    }

    lastOutputUs = (int16_t)(
        (int32_t)pControl->lastPulseUs - (int32_t)BSP_SERVO_PULSE_CENTER_US);
    if (BALL_CONTROL_SERVO_DIRECTION < 0) {
        lastOutputUs = (int16_t)-lastOutputUs;
    }

    deltaUs = (int16_t)(outputUs - lastOutputUs);
    if (deltaUs > BALL_CONTROL_OUTPUT_STEP_LIMIT_US) {
        return (int16_t)(lastOutputUs + BALL_CONTROL_OUTPUT_STEP_LIMIT_US);
    }
    if (deltaUs < -BALL_CONTROL_OUTPUT_STEP_LIMIT_US) {
        return (int16_t)(lastOutputUs - BALL_CONTROL_OUTPUT_STEP_LIMIT_US);
    }

    return outputUs;
}

static int16_t _GetPositionGainPercent(int16_t ballMm)
{
    if (ballMm >= BALL_CONTROL_POS_STRONG_MM) {
        return BALL_CONTROL_GAIN_FAR_POS_PERCENT;
    }
    if (ballMm >= 0) {
        return BALL_CONTROL_GAIN_MID_POS_PERCENT;
    }
    if (ballMm <= BALL_CONTROL_NEG_WEAK_MM) {
        return BALL_CONTROL_GAIN_FAR_NEG_PERCENT;
    }
    return BALL_CONTROL_GAIN_MID_NEG_PERCENT;
}

static int16_t _ScalePercent(int16_t value, int16_t percent)
{
    return (int16_t)(((int32_t)value * (int32_t)percent) / 100L);
}

static int16_t _Abs16(int16_t value)
{
    if (value < 0) {
        return (int16_t)-value;
    }
    return value;
}

static int16_t _LimitSigned(int32_t value, int16_t limit)
{
    if (value < -(int32_t)limit) {
        return (int16_t)-limit;
    }
    if (value > (int32_t)limit) {
        return limit;
    }
    return (int16_t)value;
}

static uint16_t _LimitPulse(int32_t pulseUs)
{
    if (pulseUs < (int32_t)BSP_SERVO_PULSE_MIN_US) {
        return BSP_SERVO_PULSE_MIN_US;
    }
    if (pulseUs > (int32_t)BSP_SERVO_PULSE_MAX_US) {
        return BSP_SERVO_PULSE_MAX_US;
    }
    return (uint16_t)pulseUs;
}
