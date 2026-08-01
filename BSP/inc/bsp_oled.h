#ifndef _BSP_OLED_H_
#define _BSP_OLED_H_

#include <stdint.h>

#define BSP_OLED_H2_WAIT_OK   (0U)
#define BSP_OLED_H2_WAIT_ENC  (1U)
#define BSP_OLED_H2_WAIT_IMU  (2U)
#define BSP_OLED_H2_WAIT_LINE (3U)

typedef struct {
    uint8_t mode;
    uint8_t fatherState;
    uint32_t elapsedMs;
    uint32_t routePulses;
    int16_t ballTargetMm;
    int16_t ballOffsetMm;
    uint8_t ballValid;
    uint8_t gray;
    uint16_t h2RoutePercent;
    uint16_t h2YawDeg;
    uint16_t h2ImuError;
    uint16_t h2ImuRx;
    uint16_t h2ImuAngle;
    uint8_t h2ImuValid;
    uint8_t h2ImuOnline;
    uint8_t h2FinishLine;
    uint8_t h2WaitCode;
} BspOledStatusView_t;

void BspOled_Init(void);
void BspOled_Clear(void);
void BspOled_ShowText(uint8_t row, uint8_t col, const char *text);
void BspOled_ShowStatus(const BspOledStatusView_t *pView);

#endif /* _BSP_OLED_H_ */
