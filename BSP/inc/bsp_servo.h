#ifndef _BSP_SERVO_H_
#define _BSP_SERVO_H_

#include <stdint.h>

#define BSP_SERVO_PULSE_MIN_US    (1000U)
#define BSP_SERVO_PULSE_CENTER_US (1500U)
#define BSP_SERVO_PULSE_MAX_US    (2000U)

void BspServo_Init(void);
void BspServo_SetPulseUs(uint16_t pulseUs);
void BspServo_Center(void);

#endif /* _BSP_SERVO_H_ */
