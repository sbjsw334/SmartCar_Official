#include "bsp_encoder.h"

#include "ti_msp_dl_config.h"

#define BSP_ENCODER_GPIOA_PORT      ENCODER_GPIOA_IN_PORT
#define BSP_ENCODER_GPIOB_PORT      ENCODER_GPIOB_IN_PORT
#define BSP_ENCODER_GPIOA_IRQN      ENCODER_GPIOA_IN_INT_IRQN
#define BSP_ENCODER_GPIOB_IRQN      ENCODER_GPIOB_IN_INT_IRQN

#define BSP_ENCODER_LEFT_A_PIN      ENCODER_GPIOA_IN_LEFT_A_PIN
#define BSP_ENCODER_LEFT_B_PIN      ENCODER_GPIOB_IN_LEFT_B_PIN
#define BSP_ENCODER_RIGHT_A_PIN     ENCODER_GPIOA_IN_RIGHT_A_PIN
#define BSP_ENCODER_RIGHT_B_PIN     ENCODER_GPIOA_IN_RIGHT_B_PIN

#define BSP_ENCODER_LEFT_A_IOMUX    ENCODER_GPIOA_IN_LEFT_A_IOMUX
#define BSP_ENCODER_LEFT_B_IOMUX    ENCODER_GPIOB_IN_LEFT_B_IOMUX
#define BSP_ENCODER_RIGHT_A_IOMUX   ENCODER_GPIOA_IN_RIGHT_A_IOMUX
#define BSP_ENCODER_RIGHT_B_IOMUX   ENCODER_GPIOA_IN_RIGHT_B_IOMUX

#define BSP_ENCODER_GPIOA_MASK      (BSP_ENCODER_LEFT_A_PIN | \
                                     BSP_ENCODER_RIGHT_A_PIN | \
                                     BSP_ENCODER_RIGHT_B_PIN)
#define BSP_ENCODER_GPIOB_MASK      (BSP_ENCODER_LEFT_B_PIN)

/* If one wheel counts backward while the car moves forward, change 1 to -1. */
#define BSP_ENCODER_LEFT_DIR        (1)
#define BSP_ENCODER_RIGHT_DIR       (-1)

static volatile int32_t s_leftCount = 0;
static volatile int32_t s_rightCount = 0;
static int32_t s_lastLeftCount = 0;
static int32_t s_lastRightCount = 0;
static volatile int16_t s_leftPps = 0;
static volatile int16_t s_rightPps = 0;
static volatile uint8_t s_speedMs = 0U;

static uint8_t _ReadLeftState(void);
static uint8_t _ReadRightState(void);
static void _CountLeftByEdge(uint32_t statusA, uint32_t statusB);
static void _CountRightByEdge(uint32_t statusA);
static int16_t _ClampI32ToI16(int32_t value);

void BspEncoder_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(BSP_ENCODER_LEFT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BSP_ENCODER_LEFT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BSP_ENCODER_RIGHT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BSP_ENCODER_RIGHT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* SysConfig owns edge polarity, interrupt enable, and priority. */
    NVIC_EnableIRQ(BSP_ENCODER_GPIOA_IRQN);
    NVIC_EnableIRQ(BSP_ENCODER_GPIOB_IRQN);

    BspEncoder_Reset();
}

void BspEncoder_Task1ms(void)
{
    int32_t leftNow;
    int32_t rightNow;
    int32_t leftDelta;
    int32_t rightDelta;

    s_speedMs++;
    if (s_speedMs < BSP_ENCODER_SPEED_PERIOD_MS) {
        return;
    }
    s_speedMs = 0U;

    __disable_irq();
    leftNow = s_leftCount;
    rightNow = s_rightCount;
    __enable_irq();

    leftDelta = leftNow - s_lastLeftCount;
    rightDelta = rightNow - s_lastRightCount;
    s_lastLeftCount = leftNow;
    s_lastRightCount = rightNow;

    s_leftPps = _ClampI32ToI16((leftDelta * 1000L) / (int32_t)BSP_ENCODER_SPEED_PERIOD_MS);
    s_rightPps = _ClampI32ToI16((rightDelta * 1000L) / (int32_t)BSP_ENCODER_SPEED_PERIOD_MS);
}

void BspEncoder_Reset(void)
{
    __disable_irq();
    s_leftCount = 0;
    s_rightCount = 0;
    s_lastLeftCount = 0;
    s_lastRightCount = 0;
    s_leftPps = 0;
    s_rightPps = 0;
    s_speedMs = 0U;
    __enable_irq();
}

void BspEncoder_GetData(BspEncoderData_t *pData)
{
    if (pData == 0) {
        return;
    }

    __disable_irq();
    pData->leftPps = s_leftPps;
    pData->rightPps = s_rightPps;
    __enable_irq();
}

void GROUP1_IRQHandler(void)
{
    uint32_t statusA;
    uint32_t statusB;

    statusA = DL_GPIO_getEnabledInterruptStatus(
        BSP_ENCODER_GPIOA_PORT, BSP_ENCODER_GPIOA_MASK);
    statusB = DL_GPIO_getEnabledInterruptStatus(
        BSP_ENCODER_GPIOB_PORT, BSP_ENCODER_GPIOB_MASK);

    if (((statusA & BSP_ENCODER_LEFT_A_PIN) != 0U) ||
        ((statusB & BSP_ENCODER_LEFT_B_PIN) != 0U)) {
        _CountLeftByEdge(statusA, statusB);
    }

    if ((statusA & (BSP_ENCODER_RIGHT_A_PIN | BSP_ENCODER_RIGHT_B_PIN)) != 0U) {
        _CountRightByEdge(statusA);
    }

    DL_GPIO_clearInterruptStatus(BSP_ENCODER_GPIOA_PORT, statusA);
    DL_GPIO_clearInterruptStatus(BSP_ENCODER_GPIOB_PORT, statusB);
}

static uint8_t _ReadLeftState(void)
{
    uint8_t state = 0U;

    if (DL_GPIO_readPins(BSP_ENCODER_GPIOA_PORT, BSP_ENCODER_LEFT_A_PIN) != 0U) {
        state |= 0x02U;
    }
    if (DL_GPIO_readPins(BSP_ENCODER_GPIOB_PORT, BSP_ENCODER_LEFT_B_PIN) != 0U) {
        state |= 0x01U;
    }

    return state;
}

static uint8_t _ReadRightState(void)
{
    uint8_t state = 0U;

    if (DL_GPIO_readPins(BSP_ENCODER_GPIOA_PORT, BSP_ENCODER_RIGHT_A_PIN) != 0U) {
        state |= 0x02U;
    }
    if (DL_GPIO_readPins(BSP_ENCODER_GPIOA_PORT, BSP_ENCODER_RIGHT_B_PIN) != 0U) {
        state |= 0x01U;
    }

    return state;
}

static void _CountLeftByEdge(uint32_t statusA, uint32_t statusB)
{
    uint8_t state = _ReadLeftState();
    uint8_t a = (uint8_t)((state >> 1) & 0x01U);
    uint8_t b = (uint8_t)(state & 0x01U);

    if ((statusA & BSP_ENCODER_LEFT_A_PIN) != 0U) {
        if (a == b) {
            s_leftCount += BSP_ENCODER_LEFT_DIR;
        } else {
            s_leftCount -= BSP_ENCODER_LEFT_DIR;
        }
    }

    if ((statusB & BSP_ENCODER_LEFT_B_PIN) != 0U) {
        if (a == b) {
            s_leftCount -= BSP_ENCODER_LEFT_DIR;
        } else {
            s_leftCount += BSP_ENCODER_LEFT_DIR;
        }
    }
}

static void _CountRightByEdge(uint32_t statusA)
{
    uint8_t state = _ReadRightState();
    uint8_t a = (uint8_t)((state >> 1) & 0x01U);
    uint8_t b = (uint8_t)(state & 0x01U);

    if ((statusA & BSP_ENCODER_RIGHT_A_PIN) != 0U) {
        if (a == b) {
            s_rightCount += BSP_ENCODER_RIGHT_DIR;
        } else {
            s_rightCount -= BSP_ENCODER_RIGHT_DIR;
        }
    }

    if ((statusA & BSP_ENCODER_RIGHT_B_PIN) != 0U) {
        if (a == b) {
            s_rightCount -= BSP_ENCODER_RIGHT_DIR;
        } else {
            s_rightCount += BSP_ENCODER_RIGHT_DIR;
        }
    }
}

static int16_t _ClampI32ToI16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}
