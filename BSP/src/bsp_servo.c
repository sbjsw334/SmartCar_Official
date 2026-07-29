#include "bsp_servo.h"

#include "ti_msp_dl_config.h"

void BspServo_Init(void)
{
    BspServo_Center();
}

void BspServo_SetPulseUs(uint16_t pulseUs)
{
    uint32_t loadValue;
    uint32_t pulseCounts;

    if (pulseUs < BSP_SERVO_PULSE_MIN_US) {
        pulseUs = BSP_SERVO_PULSE_MIN_US;
    } else if (pulseUs > BSP_SERVO_PULSE_MAX_US) {
        pulseUs = BSP_SERVO_PULSE_MAX_US;
    }

    loadValue = DL_TimerG_getLoadValue(PWM_SERVO_INST);
    pulseCounts = (uint32_t)pulseUs * (PWM_SERVO_INST_CLK_FREQ / 1000000U);
    DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST,
        (loadValue > pulseCounts) ? (loadValue - pulseCounts) : 0U,
        DL_TIMER_CC_0_INDEX);
}

void BspServo_Center(void)
{
    BspServo_SetPulseUs(BSP_SERVO_PULSE_CENTER_US);
}
