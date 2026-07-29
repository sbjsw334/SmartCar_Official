#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>

void BspUart_Init(void);
int BspUart_Printf(const char *format, ...);
uint8_t BspUart_ReadCommand(char *cmd);

#endif /* _BSP_UART_H_ */
