#include <stdint.h>

#include "app_car.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_jy61p.h"
#include "bsp_k230.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "msg_map.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t s_controlMs = 0U;
static volatile uint8_t s_telemetryMs = 0U;

static void _ProcessDebugCommands(void);
static void _ProcessKeyEvents(void);
static void _UpdateOled(void);
static uint16_t _GetH2RoutePercent(void);
static uint16_t _GetH2AbsYawDeg(void);
static uint8_t _IsH2FinishLine(uint8_t gray);
static uint8_t _GetH2WaitCode(void);
static int32_t _AbsI32(int32_t value);

/* H2 OLED debug parameters. Tune app_car.c first, then keep these values aligned while debugging. */
#define H2_DEBUG_ENCODER_COUNTS_PER_REV   (13U * 28U * 4U)
#define H2_DEBUG_WHEEL_CIRCUMFERENCE_MM   (204U)
#define H2_DEBUG_LAP_LENGTH_MM            (6142U)
#define H2_DEBUG_LAP_EXPECTED_PULSES      \
    ((H2_DEBUG_LAP_LENGTH_MM * H2_DEBUG_ENCODER_COUNTS_PER_REV) / \
     H2_DEBUG_WHEEL_CIRCUMFERENCE_MM)
#define H2_DEBUG_LAP_GATE_PERCENT         (80U)
#define H2_DEBUG_IMU_ALLOW_FALLBACK       (1U)
#define H2_DEBUG_IMU_MIN_ABS_YAW_CD       (30000L)
#define H2_DEBUG_IMU_MAX_ABS_YAW_CD       (43000L)
#define H2_DEBUG_FINISH_SENSOR_MIN        (4U)

int main(void)
{
    MsgId_t msg;

    __disable_irq();

    SYSCFG_DL_init();
    MsgMap_Init();
    BspMotor_Init();
    BspGray_Init();
    BspEncoder_Init();
    BspJy61p_Init();
    BspKey_Init();
    BspServo_Init();
    BspUart_Init();
    BspOled_Init();
    BspK230_Init();
    appCarCon.init(&appCarMain);
    _UpdateOled();

    __enable_irq();
//    (void)MsgMap_Post(MSG_KEY_START);

    while (1) {
        BspJy61p_Task();
        BspK230_Task();
        _ProcessDebugCommands();

        _ProcessKeyEvents();

        while (MsgMap_Get(&msg) != 0U) {
            appCarCon.run(&appCarMain, msg);
            if (msg == MSG_TELEMETRY_200MS) {
                _UpdateOled();
            }
        }
    }
}

void SysTick_Handler(void)
{
    AppCar_Tick1ms(&appCarMain);
    BspKey_Task1ms();
    BspGray_Task1ms();
    BspEncoder_Task1ms();
    BspJy61p_Task1ms();
    BspK230_Task1ms();

    s_controlMs++;
    if (s_controlMs >= APP_CAR_CONTROL_PERIOD_MS) {
        s_controlMs = 0U;
        (void)MsgMap_Post(MSG_CONTROL_TICK);
    }

    s_telemetryMs++;
    if (s_telemetryMs >= 200U) {
        s_telemetryMs = 0U;
        (void)MsgMap_Post(MSG_TELEMETRY_200MS);
    }
}

static void _ProcessDebugCommands(void)
{
    char command;

    while (BspUart_ReadCommand(&command) != 0U) {
        switch (command) {
            case '2':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_TRACE_ONLY);
                break;

            case '3':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALL_STATIC);
                break;

            case '4':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALANCE_AB);
                break;

            case '5':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALANCE_LAP_CENTER);
                break;

            case 's':
            case 'S':
                (void)MsgMap_Post(MSG_KEY_START);
                break;

            case 'x':
            case 'X':
                (void)MsgMap_Post(MSG_STOP);
                break;

            default:
                break;
        }
    }
}

static void _ProcessKeyEvents(void)
{
    uint8_t events = BspKey_GetPressEvents();

    if ((events & BSP_KEY_EVENT_START) != 0U) {
        (void)MsgMap_Post(MSG_KEY_START);
    }

    if ((events & BSP_KEY_EVENT_MODE) != 0U) {
        AppCarMode_t mode = (AppCarMode_t)(appCarMain.mode + 1U);

        if (mode > APP_CAR_MODE_BALANCE_LAP_CENTER) {
            mode = APP_CAR_MODE_TRACE_ONLY;
        }
        appCarCon.setMode(&appCarMain, mode);
    }

    if ((events & BSP_KEY_EVENT_PLUS) != 0U) {
        if (appCarMain.mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
            appCarCon.adjustH5HoldBiasMm(&appCarMain, 2);
        } else {
            appCarCon.setBallTargetMm(&appCarMain,
                (int16_t)(appCarMain.ballTargetMm + 10));
        }
    }

    if ((events & BSP_KEY_EVENT_MINUS) != 0U) {
        if (appCarMain.mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
            appCarCon.adjustH5HoldBiasMm(&appCarMain, -2);
        } else {
            appCarCon.setBallTargetMm(&appCarMain,
                (int16_t)(appCarMain.ballTargetMm - 10));
        }
    }
}

static void _UpdateOled(void)
{
    BspOledStatusView_t view;
    BspJy61pData_t imu;

    (void)BspJy61p_GetData(&imu);

    view.mode = (uint8_t)appCarMain.mode;
    view.fatherState = (uint8_t)appCarMain.fatherState;
    view.elapsedMs = appCarMain.elapsedMs;
    view.routePulses = appCarMain.routePulses;
    view.ballTargetMm = appCarMain.ballTargetMm;
    view.ballOffsetMm = appCarMain.ballOffsetMm;
    view.ballValid = appCarMain.ballValid;
    view.gray = appCarMain.gray;
    view.h2RoutePercent = _GetH2RoutePercent();
    view.h2YawDeg = _GetH2AbsYawDeg();
    view.h2ImuError = imu.errorCount;
    view.h2ImuRx = imu.rxCount;
    view.h2ImuAngle = imu.angleCount;
    view.h2ImuValid = appCarMain.imuValid;
    view.h2ImuOnline = appCarMain.imuOnline;
    view.h2FinishLine = _IsH2FinishLine(appCarMain.gray);
    view.h2WaitCode = _GetH2WaitCode();
    BspOled_ShowStatus(&view);
}

static uint16_t _GetH2RoutePercent(void)
{
    uint32_t percent;

    if (H2_DEBUG_LAP_EXPECTED_PULSES == 0U) {
        return 0U;
    }

    percent = (appCarMain.routePulses * 100U) / H2_DEBUG_LAP_EXPECTED_PULSES;
    if (percent > 999U) {
        percent = 999U;
    }

    return (uint16_t)percent;
}

static uint16_t _GetH2AbsYawDeg(void)
{
    int32_t yawCd = _AbsI32(appCarMain.imuLapYawCd);

    yawCd /= 100L;
    if (yawCd > 999L) {
        yawCd = 999L;
    }

    return (uint16_t)yawCd;
}

static uint8_t _IsH2FinishLine(uint8_t gray)
{
    uint8_t blackCount = 0U;

    while (gray != 0U) {
        blackCount += (uint8_t)(gray & 0x01U);
        gray >>= 1;
    }

    return (uint8_t)(blackCount >= H2_DEBUG_FINISH_SENSOR_MIN);
}

static uint8_t _GetH2WaitCode(void)
{
    if (appCarMain.fatherState == APP_CAR_FATHER_FINISHED) {
        return BSP_OLED_H2_WAIT_OK;
    }

    if (_GetH2RoutePercent() < H2_DEBUG_LAP_GATE_PERCENT) {
        return BSP_OLED_H2_WAIT_ENC;
    }

    if (_IsH2FinishLine(appCarMain.gray) == 0U) {
        return BSP_OLED_H2_WAIT_LINE;
    }

    return BSP_OLED_H2_WAIT_OK;
}

static int32_t _AbsI32(int32_t value)
{
    if (value >= 0) {
        return value;
    }

    return -value;
}
