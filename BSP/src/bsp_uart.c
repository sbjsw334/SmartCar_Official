#include "bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

#define UART_RX_BUFFER_SIZE (16U)
#define UART_TX_BUFFER_SIZE (160U)

static volatile char s_rxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;

static void _SendChar(char ch);
static void _PushRx(char ch);

void BspUart_Init(void)
{
    DL_UART_Main_clearInterruptStatus(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

int BspUart_Printf(const char *format, ...)
{
    char buffer[UART_TX_BUFFER_SIZE];
    va_list args;
    int length;
    int count;
    int i;

    if (format == 0) {
        return 0;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0) {
        return length;
    }

    count = length;
    if (count >= (int)sizeof(buffer)) {
        count = (int)sizeof(buffer) - 1;
    }
    for (i = 0; i < count; i++) {
        _SendChar(buffer[i]);
    }

    return length;
}

uint8_t BspUart_ReadCommand(char *cmd)
{
    if ((cmd == 0) || (s_rxHead == s_rxTail)) {
        return 0U;
    }

    *cmd = s_rxBuffer[s_rxTail];
    s_rxTail = (uint8_t)((s_rxTail + 1U) % UART_RX_BUFFER_SIZE);
    return 1U;
}

void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_0_INST) == DL_UART_MAIN_IIDX_RX) {
        _PushRx((char)DL_UART_Main_receiveData(UART_0_INST));
    }
}

static void _SendChar(char ch)
{
    if (ch == '\n') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)'\r');
    }
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)ch);
}

static void _PushRx(char ch)
{
    uint8_t nextHead = (uint8_t)((s_rxHead + 1U) % UART_RX_BUFFER_SIZE);

    if (nextHead != s_rxTail) {
        s_rxBuffer[s_rxHead] = ch;
        s_rxHead = nextHead;
    }
}
