#include "bsp_gray.h"

#include "ti_msp_dl_config.h"

#define GRAY_ACTIVE_LOW (0U)

#define GRAY_PORT   GRAY_GPIOB_IN_PORT
#define GRAY_OUT1   GRAY_GPIOB_IN_OUT1_PIN
#define GRAY_OUT2   GRAY_GPIOB_IN_OUT2_PIN
#define GRAY_OUT3   GRAY_GPIOB_IN_OUT3_PIN
#define GRAY_OUT4   GRAY_GPIOB_IN_OUT4_PIN
#define GRAY_OUT5   GRAY_GPIOB_IN_OUT5_PIN
#define GRAY_OUT6   GRAY_GPIOB_IN_OUT6_PIN
#define GRAY_OUT7   GRAY_GPIOB_IN_OUT7_PIN
#define GRAY_OUT8   GRAY_GPIOB_IN_OUT8_PIN
#define GRAY_MASK \
    (GRAY_OUT1 | GRAY_OUT2 | GRAY_OUT3 | GRAY_OUT4 | \
     GRAY_OUT5 | GRAY_OUT6 | GRAY_OUT7 | GRAY_OUT8)

static uint8_t s_samples[3];
static uint8_t s_sampleIndex = 0U;

static uint8_t _ReadRaw(void);

void BspGray_Init(void)
{
    s_samples[0] = _ReadRaw();
    s_samples[1] = s_samples[0];
    s_samples[2] = s_samples[0];
    s_sampleIndex = 0U;
}

void BspGray_Task1ms(void)
{
    s_samples[s_sampleIndex] = _ReadRaw();
    s_sampleIndex = (uint8_t)((s_sampleIndex + 1U) % 3U);
}

uint8_t BspGray_GetFiltered(void)
{
    return (uint8_t)((s_samples[0] & s_samples[1]) |
        (s_samples[0] & s_samples[2]) |
        (s_samples[1] & s_samples[2]));
}

static uint8_t _ReadRaw(void)
{
    uint32_t pins = DL_GPIO_readPins(GRAY_PORT, GRAY_MASK);
    uint8_t value = 0U;

    if ((pins & GRAY_OUT1) != 0U) { value |= 0x80U; }
    if ((pins & GRAY_OUT2) != 0U) { value |= 0x40U; }
    if ((pins & GRAY_OUT3) != 0U) { value |= 0x20U; }
    if ((pins & GRAY_OUT4) != 0U) { value |= 0x10U; }
    if ((pins & GRAY_OUT5) != 0U) { value |= 0x08U; }
    if ((pins & GRAY_OUT6) != 0U) { value |= 0x04U; }
    if ((pins & GRAY_OUT7) != 0U) { value |= 0x02U; }
    if ((pins & GRAY_OUT8) != 0U) { value |= 0x01U; }

#if (GRAY_ACTIVE_LOW != 0U)
    value = (uint8_t)(~value);
#endif

    return value;
}
