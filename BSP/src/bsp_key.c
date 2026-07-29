#include "bsp_key.h"

#include "ti_msp_dl_config.h"

#define BSP_KEY_PORT            KEY_GPIO_IN_PORT
#define BSP_KEY_START_PIN       KEY_GPIO_IN_START_PIN
#define BSP_KEY_DEBOUNCE_MS     (20U)

static uint8_t s_samplePressed = 0U;
static uint8_t s_stablePressed = 0U;
static uint8_t s_stableMs = 0U;
static volatile uint8_t s_pressEvent = 0U;

static uint8_t _ReadPressed(void);

void BspKey_Init(void)
{
    s_samplePressed = _ReadPressed();
    s_stablePressed = s_samplePressed;
    s_stableMs = 0U;
    s_pressEvent = 0U;
}

void BspKey_Task1ms(void)
{
    uint8_t pressed = _ReadPressed();

    if (pressed != s_samplePressed) {
        s_samplePressed = pressed;
        s_stableMs = 0U;
        return;
    }

    if (s_stableMs < BSP_KEY_DEBOUNCE_MS) {
        s_stableMs++;
    }

    if ((s_stableMs >= BSP_KEY_DEBOUNCE_MS) &&
        (s_stablePressed != s_samplePressed)) {
        s_stablePressed = s_samplePressed;
        if (s_stablePressed != 0U) {
            s_pressEvent = 1U;
        }
    }
}

uint8_t BspKey_GetPressEvent(void)
{
    uint8_t event;

    __disable_irq();
    event = s_pressEvent;
    s_pressEvent = 0U;
    __enable_irq();

    return event;
}

static uint8_t _ReadPressed(void)
{
    return (uint8_t)(DL_GPIO_readPins(BSP_KEY_PORT, BSP_KEY_START_PIN) == 0U);
}
