#ifndef _BSP_SERVO_H_
#define _BSP_SERVO_H_

#include <stdint.h>

#define BSP_SERVO_PULSE_MIN_US    (850U)//实测安全下限
#define BSP_SERVO_PULSE_CENTER_US (1150U)
#define BSP_SERVO_PULSE_MAX_US    (1450U)//实测安全上限

void BspServo_Init(void);
void BspServo_SetPulseUs(uint16_t pulseUs);
void BspServo_Center(void);

#endif /* _BSP_SERVO_H_ */
