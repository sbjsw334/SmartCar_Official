#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include <stdint.h>

typedef enum {
    BSP_KEY_EVENT_NONE  = 0U,
    BSP_KEY_EVENT_START = (1U << 0),
    BSP_KEY_EVENT_MODE  = (1U << 1),
    BSP_KEY_EVENT_PLUS  = (1U << 2),
    BSP_KEY_EVENT_MINUS = (1U << 3),
} BspKeyEvent_t;

void BspKey_Init(void);
void BspKey_Task1ms(void);
uint8_t BspKey_GetPressEvents(void);
uint8_t BspKey_GetPressed(void);

#endif /* _BSP_KEY_H_ */
