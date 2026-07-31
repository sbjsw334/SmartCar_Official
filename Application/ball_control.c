#include "ball_control.h"

/* Outer position P: position error -> target ball speed. */
// 外环：位置环（比例控制）
// 作用：根据球的位置偏差，计算出“球应该以多快的速度滚向目标点”
// 输入：目标位置 - 实际位置（mm）
// 输出：目标速度（mm/s）

#define BALL_CONTROL_OUTER_KP_NUM                  (3)
// 外环比例系数的分子 = 3
// 配合分母组成 KP = 3/2 = 1.5
// 含义：球偏离1mm，就命令它以 1.5mm/s 的速度滚回去
// 调大：归位快但易震荡；调小：反应慢但稳

#define BALL_CONTROL_OUTER_KP_DEN                  (2)
// 外环比例系数的分母 = 2
// 和分子一起构成 KP = 3/2 = 1.5（用分数形式避免浮点误差）

#define BALL_CONTROL_TARGET_SPEED_LIMIT_MM_PER_S   (180)
// 外环输出限幅：目标速度最大 ±180 mm/s
// 防止算出的目标速度太大，导致球瞬间飞出去失控
// 如果球总冲过头，减小这个值；如果反应太慢，增大它


/* Inner speed P: speed error -> servo pulse offset. No I or D. */
// 内环：速度环（比例控制，没有积分I和微分D）
// 作用：根据速度偏差，计算出“舵机应该偏转多少微秒”
// 输入：目标速度 - 实际速度（mm/s）
// 输出：舵机脉宽偏移量（微秒）

#define BALL_CONTROL_INNER_KP_POS_NUM              (4)
// 正向内环比例系数的分子 = 4
// 用于球在负半区、需要往正方向回到目标点时
// 这边实测存在静摩擦/机构死区，所以给更大的P

#define BALL_CONTROL_INNER_KP_NEG_NUM              (2)
// 负向内环比例系数的分子 = 2
// 用于球在正半区、需要往负方向回到目标点时
// 这边已经能及时回调，所以先保持原来的力度

#define BALL_CONTROL_INNER_KP_DEN                  (1)
// 内环比例系数的分母 = 1
// 和正/负方向分子一起构成对应方向的 KP

#define BALL_CONTROL_INNER_OUTPUT_POS_LIMIT_US     (260)
// 正向内环输出限幅：最大 +260 微秒
// 本机构正向回调偏弱，所以允许更大的正向舵机偏移

#define BALL_CONTROL_INNER_OUTPUT_NEG_LIMIT_US     (180)
// 负向内环输出限幅：最大 -180 微秒
// 负向已经能回调，先不继续加大，避免另一边过冲

#define BALL_CONTROL_POSITIVE_MIN_OUTPUT_US        (90)
// 正向最小动作量：当需要往正方向回调时，至少给 +90 微秒
// 用来克服 -30mm 到 0mm 附近调不动的静摩擦/死区
// 如果接近0点开始来回抖，可以把它降到 60~70


#define BALL_CONTROL_MEASURED_SPEED_LIMIT_MM_PER_S (800)
// 速度测量值上限：±800 mm/s
// 如果计算出的球速超过这个范围，就截断为 +800 或 -800 mm/s
// 防止偶尔的跳变数据把舵机带飞


#define BALL_CONTROL_SAMPLE_MIN_MS                 (10U)
// 两个有效视觉帧用于测速的最小间隔：10 毫秒
// 如果间隔小于 10ms，认为这次速度估计不可靠，将实际速度暂时置为0
// 外环和内环仍会继续计算，并不是跳过整次控制


#define BALL_CONTROL_SAMPLE_MAX_MS                 (150U)
// 两个有效视觉帧用于测速的最大间隔：150 毫秒
// 如果间隔超过 150ms，说明视觉曾长时间停顿，旧速度已经没有参考意义
// 此时将实际速度暂时置为0，并不会把采样间隔强制改成150ms


#define BALL_CONTROL_SERVO_DIRECTION               (1)
// 舵机转向方向系数
// = 1：正向控制（误差越大舵机正向偏转）
// = -1：反向控制（如果舵机装反了，改成 -1 就能修正，不用重写代码）
// 调试时如果发现球往反方向跑，把这个 1 改成 -1 就解决了

static uint16_t _LimitPulse(int32_t pulseUs);
static int16_t _LimitSigned(int32_t value, int16_t limit);
static int16_t _CalcInnerOutputUs(int16_t positionErrorMm,
    int16_t speedErrorMmPerSec);

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
    pControl->targetSpeedMmPerSec = _LimitSigned(
        ((int32_t)positionErrorMm * BALL_CONTROL_OUTER_KP_NUM) /
            BALL_CONTROL_OUTER_KP_DEN,
        BALL_CONTROL_TARGET_SPEED_LIMIT_MM_PER_S);

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

    speedErrorMmPerSec = (int16_t)(
        pControl->targetSpeedMmPerSec - pControl->ballSpeedMmPerSec);
    pulseOffsetUs = _CalcInnerOutputUs(positionErrorMm, speedErrorMmPerSec);

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

static int16_t _CalcInnerOutputUs(int16_t positionErrorMm,
    int16_t speedErrorMmPerSec)
{
    int32_t outputUs;

    if (speedErrorMmPerSec >= 0) {
        outputUs =
            ((int32_t)speedErrorMmPerSec * BALL_CONTROL_INNER_KP_POS_NUM) /
            BALL_CONTROL_INNER_KP_DEN;
        outputUs = _LimitSigned(outputUs,
            BALL_CONTROL_INNER_OUTPUT_POS_LIMIT_US);

        if ((positionErrorMm > 3) &&
            (outputUs > 0) &&
            (outputUs < BALL_CONTROL_POSITIVE_MIN_OUTPUT_US)) {
            outputUs = BALL_CONTROL_POSITIVE_MIN_OUTPUT_US;
        }
    } else {
        outputUs =
            ((int32_t)speedErrorMmPerSec * BALL_CONTROL_INNER_KP_NEG_NUM) /
            BALL_CONTROL_INNER_KP_DEN;
        outputUs = _LimitSigned(outputUs,
            BALL_CONTROL_INNER_OUTPUT_NEG_LIMIT_US);
    }

    return (int16_t)outputUs;
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
