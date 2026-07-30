#include "ball_control.h"

#define BALL_CONTROL_KP_US_PER_MM (3)
#define BALL_CONTROL_KD_US_PER_MM (4)

static uint16_t _LimitPulse(int32_t pulseUs);

void BallControl_Init(BallControl_t *pControl)
{
    BallControl_Reset(pControl);
}

void BallControl_Reset(BallControl_t *pControl)
{
    if (pControl == 0) {
        return;
    }

    pControl->lastErrorMm = 0;
    pControl->hasLastError = 0U;
}

uint16_t BallControl_Update(BallControl_t *pControl,
    int16_t targetMm, int16_t ballMm, uint8_t valid)
{
    int16_t errorMm;
    int16_t deltaMm = 0;
    int32_t pulseUs;

    if ((pControl == 0) || (valid == 0U)) {
        if (pControl != 0) {
            BallControl_Reset(pControl);
        }
        return BSP_SERVO_PULSE_CENTER_US;
    }

    errorMm = (int16_t)(targetMm - ballMm);
    if (pControl->hasLastError != 0U) {
        deltaMm = (int16_t)(errorMm - pControl->lastErrorMm);
    }

    pControl->lastErrorMm = errorMm;
    pControl->hasLastError = 1U;

    pulseUs = (int32_t)BSP_SERVO_PULSE_CENTER_US +
        ((int32_t)BALL_CONTROL_KP_US_PER_MM * errorMm) +
        ((int32_t)BALL_CONTROL_KD_US_PER_MM * deltaMm);

    return _LimitPulse(pulseUs);
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
