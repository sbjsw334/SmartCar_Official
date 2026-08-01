#ifndef _BSP_JY61P_H_
#define _BSP_JY61P_H_

#include <stdint.h>

typedef struct {
    int16_t rollCd;       /* roll, 0.01 deg */
    int16_t pitchCd;      /* pitch, 0.01 deg */
    int16_t yawCd;        /* yaw, 0.01 deg */
    int16_t accXmg;       /* X acceleration, mg */
    int16_t accYmg;       /* Y acceleration, mg */
    int16_t accZmg;       /* Z acceleration, mg */
    uint32_t sampleSeq;   /* valid angle frame sequence */
    uint16_t rxCount;     /* raw UART byte count, debug */
    uint16_t angleCount;  /* parsed angle frame count, debug */
    uint16_t errorCount;  /* timeout/checksum/drop count, debug */
    uint8_t online;       /* has received a valid angle frame */
    uint8_t valid;        /* latest data is valid */
} BspJy61pData_t;

void BspJy61p_Init(void);
void BspJy61p_Task1ms(void);
void BspJy61p_Task(void);
uint8_t BspJy61p_GetData(BspJy61pData_t *pData);
uint8_t BspJy61p_ZeroYaw(void);

#endif /* _BSP_JY61P_H_ */
