#include "bsp_key.h"

#include "ti_msp_dl_config.h"

#define BSP_KEY_PORT          KEY_GPIOA_IN_PORT
#define BSP_KEY_START_PIN     KEY_GPIOA_IN_START_PIN
#define BSP_KEY_MODE_PIN      KEY_GPIOA_IN_MODE_PIN
#define BSP_KEY_PLUS_PIN      KEY_GPIOA_IN_PLUS_PIN
#define BSP_KEY_MINUS_PIN     KEY_GPIOA_IN_MINUS_PIN
#define BSP_KEY_DEBOUNCE_MS   (20U)
#define BSP_KEY_COUNT         (4U)

static uint8_t s_samplePressed = 0U;
static volatile uint8_t s_stablePressed = 0U;
static uint8_t s_stableMs[BSP_KEY_COUNT];
static volatile uint8_t s_pressEvents = 0U;

static uint8_t _ReadPressed(void);

void BspKey_Init(void)
{
    uint8_t index;

    s_samplePressed = _ReadPressed();
    s_stablePressed = 0U;
    s_pressEvents = 0U;
    for (index = 0U; index < BSP_KEY_COUNT; index++) {
        s_stableMs[index] = 0U;
    }
}

void BspKey_Task1ms(void)
{
    uint8_t pressed = _ReadPressed();
    uint8_t index;

    for (index = 0U; index < BSP_KEY_COUNT; index++) {
        uint8_t event = (uint8_t)(1U << index);
        uint8_t sample = (uint8_t)(pressed & event);

        if (sample != (s_samplePressed & event)) {
            s_samplePressed ^= event;
            s_stableMs[index] = 0U;
            continue;
        }

        if (s_stableMs[index] < BSP_KEY_DEBOUNCE_MS) {
            s_stableMs[index]++;
        }

        if ((s_stableMs[index] >= BSP_KEY_DEBOUNCE_MS) &&
            (sample != (s_stablePressed & event))) {
            s_stablePressed ^= event;
            if (sample != 0U) {
                s_pressEvents |= event;
            }
        }
    }
}

uint8_t BspKey_GetPressEvents(void)
{
    uint8_t events;

    __disable_irq();
    events = s_pressEvents;
    s_pressEvents = 0U;
    __enable_irq();

    return events;
}

uint8_t BspKey_GetPressed(void)
{
    return s_stablePressed;
}

static uint8_t _ReadPressed(void)
{
    uint32_t pins = DL_GPIO_readPins(BSP_KEY_PORT,
        BSP_KEY_START_PIN | BSP_KEY_MODE_PIN |
        BSP_KEY_PLUS_PIN | BSP_KEY_MINUS_PIN);
    uint8_t pressed = 0U;

    if ((pins & BSP_KEY_START_PIN) == 0U) {
        pressed |= BSP_KEY_EVENT_START;
    }
    if ((pins & BSP_KEY_MODE_PIN) == 0U) {
        pressed |= BSP_KEY_EVENT_MODE;
    }
    if ((pins & BSP_KEY_PLUS_PIN) == 0U) {
        pressed |= BSP_KEY_EVENT_PLUS;
    }
    if ((pins & BSP_KEY_MINUS_PIN) == 0U) {
        pressed |= BSP_KEY_EVENT_MINUS;
    }

    return pressed;
}
