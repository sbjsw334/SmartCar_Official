#include "ball_control.h"

#define BALL_CONTROL_POS_TO_SPEED_DIV         (3)//它决定外环目标速度大小。数值越小，目标速度越大 3
#define BALL_CONTROL_SPEED_LIMIT_MM_PER_FRAME (20)
#define BALL_CONTROL_SPEED_KP_US_PER_MM       (5)//它决定舵机跟随速度误差的力度     1
#define BALL_CONTROL_SPEED_KD_US_PER_MM       (0)//它负责刹车，压惯性              2

static uint16_t _LimitPulse(int32_t pulseUs);
static int16_t _LimitSpeed(int32_t speedMm);

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
    pControl->lastSpeedErrorMm = 0;
    pControl->lastFrameSeq = 0U;
    pControl->lastPulseUs = BSP_SERVO_PULSE_CENTER_US;
    pControl->hasFrame = 0U;
}

uint16_t BallControl_Update(BallControl_t *pControl,
    int16_t targetMm, int16_t ballMm, uint32_t frameSeq, uint8_t valid)
{
    int16_t positionErrorMm;
    int16_t targetSpeedMm;
    int16_t ballSpeedMm = 0;
    int16_t speedErrorMm;
    int16_t speedDeltaMm = 0;
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
    targetSpeedMm = _LimitSpeed(
        (int32_t)positionErrorMm / BALL_CONTROL_POS_TO_SPEED_DIV);

    if (pControl->hasFrame != 0U) {
        ballSpeedMm = (int16_t)(ballMm - pControl->lastBallMm);
    }

    speedErrorMm = (int16_t)(targetSpeedMm - ballSpeedMm);
    if (pControl->hasFrame != 0U) {
        speedDeltaMm = (int16_t)(
            speedErrorMm - pControl->lastSpeedErrorMm);
    }

    pulseUs = (int32_t)BSP_SERVO_PULSE_CENTER_US +
        ((int32_t)BALL_CONTROL_SPEED_KP_US_PER_MM * speedErrorMm) +
        ((int32_t)BALL_CONTROL_SPEED_KD_US_PER_MM * speedDeltaMm);

    pControl->lastBallMm = ballMm;
    pControl->lastSpeedErrorMm = speedErrorMm;
    pControl->lastFrameSeq = frameSeq;
    pControl->lastPulseUs = _LimitPulse(pulseUs);
    pControl->hasFrame = 1U;

    return pControl->lastPulseUs;
}

static int16_t _LimitSpeed(int32_t speedMm)
{
    if (speedMm < -BALL_CONTROL_SPEED_LIMIT_MM_PER_FRAME) {
        return (int16_t)-BALL_CONTROL_SPEED_LIMIT_MM_PER_FRAME;
    }
    if (speedMm > BALL_CONTROL_SPEED_LIMIT_MM_PER_FRAME) {
        return BALL_CONTROL_SPEED_LIMIT_MM_PER_FRAME;
    }
    return (int16_t)speedMm;
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
