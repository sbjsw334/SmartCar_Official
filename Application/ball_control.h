#ifndef _BALL_CONTROL_H_
#define _BALL_CONTROL_H_

#include <stdint.h>

#include "bsp_servo.h"

typedef struct {
    int16_t lastBallMm;
    int16_t lastSpeedErrorMm;
    uint32_t lastFrameSeq;
    uint16_t lastPulseUs;
    uint8_t hasFrame;
} BallControl_t;

void BallControl_Init(BallControl_t *pControl);
void BallControl_Reset(BallControl_t *pControl);
uint16_t BallControl_Update(BallControl_t *pControl,
    int16_t targetMm, int16_t ballMm, uint32_t frameSeq, uint8_t valid);

#endif /* _BALL_CONTROL_H_ */
