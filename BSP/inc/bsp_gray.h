#ifndef _BSP_GRAY_H_
#define _BSP_GRAY_H_

#include <stdint.h>

#define BSP_GRAY_ALL_WHITE (0x00U)
#define BSP_GRAY_ALL_BLACK (0xFFU)

void BspGray_Init(void);
void BspGray_Task1ms(void);
uint8_t BspGray_GetFiltered(void);

#endif /* _BSP_GRAY_H_ */
