#include "bsp_motor.h"

#include "ti_msp_dl_config.h"

#define MOTOR_PORT_A       MOTOR_GPIOA_OUT_PORT
#define MOTOR_PORT_B       MOTOR_GPIOB_OUT_PORT
#define MOTOR_LEFT_IN1     MOTOR_GPIOA_OUT_AIN1_PIN
#define MOTOR_LEFT_IN2     MOTOR_GPIOB_OUT_AIN2_PIN
#define MOTOR_RIGHT_IN1    MOTOR_GPIOB_OUT_BIN1_PIN
#define MOTOR_RIGHT_IN2    MOTOR_GPIOB_OUT_BIN2_PIN
#define MOTOR_STANDBY      MOTOR_GPIOA_OUT_STBY_PIN
#define MOTOR_LEFT_TIMER   PWM_MOTOR_A_INST
#define MOTOR_RIGHT_TIMER  PWM_MOTOR_B_INST

static uint16_t _AbsSpeed(int16_t speed);
static uint32_t _SpeedToCompare(GPTIMER_Regs *timer, uint16_t speed);
static void _SetPwm(uint16_t leftSpeed, uint16_t rightSpeed);

void BspMotor_Init(void)
{
    BspMotor_Stop();
}

void BspMotor_SetSignedSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
    DL_GPIO_setPins(MOTOR_PORT_A, MOTOR_STANDBY);

    if (leftSpeed >= 0) {
        DL_GPIO_setPins(MOTOR_PORT_A, MOTOR_LEFT_IN1);
        DL_GPIO_clearPins(MOTOR_PORT_B, MOTOR_LEFT_IN2);
    } else {
        DL_GPIO_clearPins(MOTOR_PORT_A, MOTOR_LEFT_IN1);
        DL_GPIO_setPins(MOTOR_PORT_B, MOTOR_LEFT_IN2);
    }

    if (rightSpeed >= 0) {
        DL_GPIO_setPins(MOTOR_PORT_B, MOTOR_RIGHT_IN1);
        DL_GPIO_clearPins(MOTOR_PORT_B, MOTOR_RIGHT_IN2);
    } else {
        DL_GPIO_clearPins(MOTOR_PORT_B, MOTOR_RIGHT_IN1);
        DL_GPIO_setPins(MOTOR_PORT_B, MOTOR_RIGHT_IN2);
    }

    _SetPwm(_AbsSpeed(leftSpeed), _AbsSpeed(rightSpeed));
}

void BspMotor_Stop(void)
{
    DL_GPIO_clearPins(MOTOR_PORT_A, MOTOR_STANDBY | MOTOR_LEFT_IN1);
    DL_GPIO_clearPins(MOTOR_PORT_B,
        MOTOR_LEFT_IN2 | MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2);
    _SetPwm(0U, 0U);
}

static uint16_t _AbsSpeed(int16_t speed)
{
    uint16_t value = (uint16_t)((speed < 0) ? -speed : speed);

    return (value > BSP_MOTOR_SPEED_MAX) ? BSP_MOTOR_SPEED_MAX : value;
}

static uint32_t _SpeedToCompare(GPTIMER_Regs *timer, uint16_t speed)
{
    uint32_t loadValue = DL_TimerA_getLoadValue(timer);

    return loadValue - ((loadValue * speed) / BSP_MOTOR_SPEED_MAX);
}

static void _SetPwm(uint16_t leftSpeed, uint16_t rightSpeed)
{
    DL_TimerA_setCaptureCompareValue(MOTOR_LEFT_TIMER,
        _SpeedToCompare(MOTOR_LEFT_TIMER, leftSpeed), DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_RIGHT_TIMER,
        _SpeedToCompare(MOTOR_RIGHT_TIMER, rightSpeed), DL_TIMER_CC_1_INDEX);
}
