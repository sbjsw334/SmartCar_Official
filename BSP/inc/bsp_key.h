#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include <stdint.h>

void BspKey_Init(void);
void BspKey_Task1ms(void);
uint8_t BspKey_GetPressEvent(void);

#endif /* _BSP_KEY_H_ */
