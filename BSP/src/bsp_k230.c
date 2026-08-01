#include "bsp_k230.h"

#include "ti_msp_dl_config.h"

#define BSP_K230_RX_BUFFER_SIZE   (64U)
#define BSP_K230_LINE_SIZE        (24U)
#define BSP_K230_SPIKE_REJECT_MM  (80)

static volatile uint8_t s_rxBuffer[BSP_K230_RX_BUFFER_SIZE];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;

static char s_line[BSP_K230_LINE_SIZE];
static uint8_t s_lineLength = 0U;
static volatile BspK230Ball_t s_ball;
static volatile BspK230Debug_t s_debug;
static volatile uint16_t s_ageMs = BSP_K230_TIMEOUT_MS;

static void _PushRx(uint8_t byte);
static uint8_t _PopRx(uint8_t *pByte);
static void _ProcessByte(uint8_t byte);
static uint8_t _ParseBallLine(const char *line, int16_t *pPosition, uint8_t *pValid);
static uint8_t _IsSpikePosition(int16_t position);
static int16_t _ClampPosition(int32_t position);

void BspK230_Init(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_lineLength = 0U;
    s_ball.frameSeq = 0U;
    s_ball.offsetMm = 0;
    s_ball.valid = 0U;
    s_debug.rxBytes = 0U;
    s_debug.pollBytes = 0U;
    s_debug.lines = 0U;
    s_debug.parsed = 0U;
    s_ageMs = BSP_K230_TIMEOUT_MS;

    DL_UART_Main_clearInterruptStatus(UART_K230_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

void BspK230_Task(void)
{
    uint8_t byte;

    while (DL_UART_Main_receiveDataCheck(UART_K230_INST, &byte)) {
        s_debug.pollBytes++;
        _PushRx(byte);
    }

    while (_PopRx(&byte) != 0U) {
        _ProcessByte(byte);
    }
}

void BspK230_Task1ms(void)
{
    if (s_ageMs < 0xFFFFU) {
        s_ageMs++;
    }
    if (s_ageMs >= BSP_K230_TIMEOUT_MS) {
        s_ball.valid = 0U;
    }
}

void BspK230_GetBall(BspK230Ball_t *pBall)
{
    if (pBall == 0) {
        return;
    }

    __disable_irq();
    *pBall = s_ball;
    __enable_irq();
}

void BspK230_GetDebug(BspK230Debug_t *pDebug)
{
    if (pDebug == 0) {
        return;
    }

    __disable_irq();
    *pDebug = s_debug;
    __enable_irq();
}

void UART_K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_K230_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            s_debug.rxBytes++;
            _PushRx((uint8_t)DL_UART_Main_receiveData(UART_K230_INST));
            break;

        default:
            break;
    }
}

static void _PushRx(uint8_t byte)
{
    uint8_t nextHead = (uint8_t)((s_rxHead + 1U) % BSP_K230_RX_BUFFER_SIZE);

    if (nextHead == s_rxTail) {
        return;
    }

    s_rxBuffer[s_rxHead] = byte;
    s_rxHead = nextHead;
}

static uint8_t _PopRx(uint8_t *pByte)
{
    if ((pByte == 0) || (s_rxHead == s_rxTail)) {
        return 0U;
    }

    *pByte = s_rxBuffer[s_rxTail];
    s_rxTail = (uint8_t)((s_rxTail + 1U) % BSP_K230_RX_BUFFER_SIZE);
    return 1U;
}

static void _ProcessByte(uint8_t byte)
{
    int16_t position;
    uint8_t valid;

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        s_debug.lines++;
        s_line[s_lineLength] = '\0';
        if (_ParseBallLine(s_line, &position, &valid) != 0U) {
            __disable_irq();
            if ((valid != 0U) && (_IsSpikePosition(position) == 0U)) {
                s_ball.frameSeq++;
                s_ball.offsetMm = position;
                s_ageMs = 0U;
                s_ball.valid = 1U;
            }
            s_debug.parsed++;
            __enable_irq();
        }
        s_lineLength = 0U;
        return;
    }

    if (s_lineLength < (BSP_K230_LINE_SIZE - 1U)) {
        s_line[s_lineLength++] = (char)byte;
    } else {
        s_lineLength = 0U;
    }
}

static uint8_t _ParseBallLine(const char *line, int16_t *pPosition, uint8_t *pValid)
{
    uint8_t index = 0U;
    uint8_t negative = 0U;
    uint8_t digitCount = 0U;
    int32_t value = 0L;

    if ((line == 0) || (pPosition == 0) || (pValid == 0)) {
        return 0U;
    }
    if ((line[index++] != 'B') || (line[index++] != ',')) {
        return 0U;
    }

    if ((line[index] == '+') || (line[index] == '-')) {
        negative = (uint8_t)(line[index] == '-');
        index++;
    }

    while ((line[index] >= '0') && (line[index] <= '9')) {
        if (digitCount >= 6U) {
            return 0U;
        }
        value = (value * 10L) + (int32_t)(line[index] - '0');
        digitCount++;
        index++;
    }

    if ((digitCount == 0U) || (line[index++] != ',')) {
        return 0U;
    }
    if (((line[index] != '0') && (line[index] != '1')) ||
        (line[index + 1U] != '\0')) {
        return 0U;
    }

    if (negative != 0U) {
        value = -value;
    }
    *pPosition = _ClampPosition(value);
    *pValid = (uint8_t)(line[index] == '1');
    return 1U;
}

static uint8_t _IsSpikePosition(int16_t position)
{
    int32_t diff;

    if (s_ball.valid == 0U) {
        return 0U;
    }

    diff = (int32_t)position - (int32_t)s_ball.offsetMm;
    if (diff < 0) {
        diff = -diff;
    }

    return (uint8_t)(diff > BSP_K230_SPIKE_REJECT_MM);
}

static int16_t _ClampPosition(int32_t position)
{
    if (position < BSP_K230_OFFSET_MIN_MM) {
        return BSP_K230_OFFSET_MIN_MM;
    }
    if (position > BSP_K230_OFFSET_MAX_MM) {
        return BSP_K230_OFFSET_MAX_MM;
    }
    return (int16_t)position;
}
