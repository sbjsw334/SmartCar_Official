#ifndef _BSP_K230_H_
#define _BSP_K230_H_

#include <stdint.h>

#define BSP_K230_OFFSET_MIN_MM  (-125)
#define BSP_K230_OFFSET_MAX_MM  (125)
#define BSP_K230_TIMEOUT_MS     (100U)

typedef struct {
    int16_t offsetMm;
    uint8_t valid;
} BspK230Ball_t;

void BspK230_Init(void);
void BspK230_Task(void);
void BspK230_Task1ms(void);
void BspK230_GetBall(BspK230Ball_t *pBall);

#endif /* _BSP_K230_H_ */
