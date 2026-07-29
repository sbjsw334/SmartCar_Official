#ifndef _BSP_MOTOR_H_
#define _BSP_MOTOR_H_

#include <stdint.h>

#define BSP_MOTOR_SPEED_MAX (100U)

void BspMotor_Init(void);
void BspMotor_SetSignedSpeed(int16_t leftSpeed, int16_t rightSpeed);
void BspMotor_Stop(void);

#endif /* _BSP_MOTOR_H_ */
