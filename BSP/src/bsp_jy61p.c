#include "bsp_jy61p.h"

#include "ti_msp_dl_config.h"

#define BSP_JY61P_RX_BUFFER_SIZE     (96U)
#define BSP_JY61P_FRAME_SIZE         (11U)
#define BSP_JY61P_FRAME_HEAD         (0x55U)
#define BSP_JY61P_FRAME_ACC          (0x51U)
#define BSP_JY61P_FRAME_GYRO         (0x52U)
#define BSP_JY61P_FRAME_ANGLE        (0x53U)
#define BSP_JY61P_TIMEOUT_MS         (200U)

static volatile uint8_t s_rxBuffer[BSP_JY61P_RX_BUFFER_SIZE];
static volatile uint8_t s_rxHead;
static volatile uint8_t s_rxTail;
static volatile uint16_t s_ageMs;
static BspJy61pData_t s_data;
static int16_t s_yawZeroCd;
static uint8_t s_hasYawZero;

static void _PushRx(uint8_t byte);
static uint8_t _PopRx(uint8_t *pByte);
static void _ProcessByte(uint8_t byte);
static void _ParseFrame(const uint8_t *pFrame);
static uint8_t _ChecksumOk(const uint8_t *pFrame);
static int16_t _Le16(const uint8_t *pData);
static int16_t _NormalizeAngleCd(int32_t angleCd);
static int16_t _RawToAngleCd(int16_t raw);
static int16_t _RawToAccMg(int16_t raw);

void BspJy61p_Init(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_ageMs = BSP_JY61P_TIMEOUT_MS;
    s_yawZeroCd = 0;
    s_hasYawZero = 0U;

    s_data.rollCd = 0;
    s_data.pitchCd = 0;
    s_data.yawCd = 0;
    s_data.accXmg = 0;
    s_data.accYmg = 0;
    s_data.accZmg = 0;
    s_data.sampleSeq = 0U;
    s_data.rxCount = 0U;
    s_data.angleCount = 0U;
    s_data.errorCount = 0U;
    s_data.online = 0U;
    s_data.valid = 0U;

    DL_UART_Main_clearInterruptStatus(UART_JY61P_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);
}

void BspJy61p_Task1ms(void)
{
    if (s_ageMs < 0xFFFFU) {
        s_ageMs++;
    }

    if (s_ageMs >= BSP_JY61P_TIMEOUT_MS) {
        s_data.valid = 0U;
        if (((s_ageMs % 100U) == 0U) && (s_data.errorCount < 0xFFFFU)) {
            s_data.errorCount++;
        }
    }
}

void BspJy61p_Task(void)
{
    uint8_t byte;

    while (DL_UART_Main_receiveDataCheck(UART_JY61P_INST, &byte)) {
        _PushRx(byte);
    }

    while (_PopRx(&byte) != 0U) {
        _ProcessByte(byte);
    }
}

uint8_t BspJy61p_GetData(BspJy61pData_t *pData)
{
    uint32_t interruptState;

    if (pData == 0) {
        return 0U;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    *pData = s_data;
    __set_PRIMASK(interruptState);

    return pData->valid;
}

uint8_t BspJy61p_ZeroYaw(void)
{
    if (s_data.valid == 0U) {
        return 0U;
    }

    s_yawZeroCd = s_data.yawCd;
    s_hasYawZero = 1U;
    return 1U;
}

void UART_JY61P_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST) ==
        DL_UART_MAIN_IIDX_RX) {
        _PushRx((uint8_t)DL_UART_Main_receiveData(UART_JY61P_INST));
    }
}

static void _PushRx(uint8_t byte)
{
    uint8_t nextHead = (uint8_t)((s_rxHead + 1U) % BSP_JY61P_RX_BUFFER_SIZE);

    if (nextHead == s_rxTail) {
        if (s_data.errorCount < 0xFFFFU) {
            s_data.errorCount++;
        }
        return;
    }

    s_rxBuffer[s_rxHead] = byte;
    s_rxHead = nextHead;
    if (s_data.rxCount < 0xFFFFU) {
        s_data.rxCount++;
    }
}

static uint8_t _PopRx(uint8_t *pByte)
{
    if ((pByte == 0) || (s_rxHead == s_rxTail)) {
        return 0U;
    }

    *pByte = s_rxBuffer[s_rxTail];
    s_rxTail = (uint8_t)((s_rxTail + 1U) % BSP_JY61P_RX_BUFFER_SIZE);
    return 1U;
}

static void _ProcessByte(uint8_t byte)
{
    static uint8_t frame[BSP_JY61P_FRAME_SIZE];
    static uint8_t index = 0U;

    if (index == 0U) {
        if (byte != BSP_JY61P_FRAME_HEAD) {
            return;
        }
        frame[index++] = byte;
        return;
    }

    if ((index == 1U) &&
        (byte != BSP_JY61P_FRAME_ACC) &&
        (byte != BSP_JY61P_FRAME_GYRO) &&
        (byte != BSP_JY61P_FRAME_ANGLE)) {
        index = 0U;
        if (byte == BSP_JY61P_FRAME_HEAD) {
            frame[index++] = byte;
        }
        return;
    }

    frame[index++] = byte;
    if (index >= BSP_JY61P_FRAME_SIZE) {
        if (_ChecksumOk(frame) != 0U) {
            _ParseFrame(frame);
        } else if (s_data.errorCount < 0xFFFFU) {
            s_data.errorCount++;
        }
        index = 0U;
    }
}

static void _ParseFrame(const uint8_t *pFrame)
{
    int16_t yawCd;

    if (pFrame[1] == BSP_JY61P_FRAME_ACC) {
        s_data.accXmg = _RawToAccMg(_Le16(&pFrame[2]));
        s_data.accYmg = _RawToAccMg(_Le16(&pFrame[4]));
        s_data.accZmg = _RawToAccMg(_Le16(&pFrame[6]));
        return;
    }

    if (pFrame[1] != BSP_JY61P_FRAME_ANGLE) {
        return;
    }

    s_data.rollCd = _RawToAngleCd(_Le16(&pFrame[2]));
    s_data.pitchCd = _RawToAngleCd(_Le16(&pFrame[4]));
    yawCd = _RawToAngleCd(_Le16(&pFrame[6]));
    if (s_hasYawZero != 0U) {
        yawCd = _NormalizeAngleCd((int32_t)yawCd - (int32_t)s_yawZeroCd);
    }
    s_data.yawCd = yawCd;
    s_data.sampleSeq++;
    if (s_data.angleCount < 0xFFFFU) {
        s_data.angleCount++;
    }
    s_data.online = 1U;
    s_data.valid = 1U;
    s_ageMs = 0U;
}

static uint8_t _ChecksumOk(const uint8_t *pFrame)
{
    uint8_t sum = 0U;
    uint8_t index;

    for (index = 0U; index < (BSP_JY61P_FRAME_SIZE - 1U); index++) {
        sum = (uint8_t)(sum + pFrame[index]);
    }

    return (uint8_t)(sum == pFrame[BSP_JY61P_FRAME_SIZE - 1U]);
}

static int16_t _Le16(const uint8_t *pData)
{
    return (int16_t)((uint16_t)pData[0] | ((uint16_t)pData[1] << 8));
}

static int16_t _NormalizeAngleCd(int32_t angleCd)
{
    while (angleCd > 18000L) {
        angleCd -= 36000L;
    }
    while (angleCd < -18000L) {
        angleCd += 36000L;
    }
    return (int16_t)angleCd;
}

static int16_t _RawToAngleCd(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 18000L) / 32768L);
}

static int16_t _RawToAccMg(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 16000L) / 32768L);
}
