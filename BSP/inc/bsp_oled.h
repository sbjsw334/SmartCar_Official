#ifndef _BSP_OLED_H_
#define _BSP_OLED_H_

#include <stdint.h>

typedef struct {
    uint8_t mode;
    uint8_t fatherState;
    uint8_t routeState;
    uint8_t ballState;
    uint32_t elapsedMs;
    int16_t ballTargetMm;
    int16_t ballOffsetPx;
    uint8_t ballValid;
    uint8_t gray;
} BspOledStatusView_t;

void BspOled_Init(void);
void BspOled_Clear(void);
void BspOled_ShowText(uint8_t row, uint8_t col, const char *text);
void BspOled_ShowStatus(const BspOledStatusView_t *pView);

#endif /* _BSP_OLED_H_ */
