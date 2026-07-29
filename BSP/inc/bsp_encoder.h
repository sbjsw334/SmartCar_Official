#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include <stdint.h>

#define BSP_ENCODER_SPEED_PERIOD_MS (10U)

typedef struct {
    int16_t leftPps;
    int16_t rightPps;
} BspEncoderData_t;

void BspEncoder_Init(void);
void BspEncoder_Task1ms(void);
void BspEncoder_Reset(void);
void BspEncoder_GetData(BspEncoderData_t *pData);

#endif /* _BSP_ENCODER_H_ */
