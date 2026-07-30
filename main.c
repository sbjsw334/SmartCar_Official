#include <stdint.h>

#include "app_car.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
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

int main(void)
{
    MsgId_t msg;

    __disable_irq();

    SYSCFG_DL_init();
    MsgMap_Init();
    BspMotor_Init();
    BspGray_Init();
    BspEncoder_Init();
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

            case '6':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALANCE_LAP_TARGET);
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

        if (mode > APP_CAR_MODE_BALANCE_LAP_TARGET) {
            mode = APP_CAR_MODE_TRACE_ONLY;
        }
        appCarCon.setMode(&appCarMain, mode);
    }

    if ((events & BSP_KEY_EVENT_PLUS) != 0U) {
        appCarCon.setBallTargetMm(&appCarMain,
            (int16_t)(appCarMain.ballTargetMm + 10));
    }

    if ((events & BSP_KEY_EVENT_MINUS) != 0U) {
        appCarCon.setBallTargetMm(&appCarMain,
            (int16_t)(appCarMain.ballTargetMm - 10));
    }
}

static void _UpdateOled(void)
{
    BspOledStatusView_t view;

    view.mode = (uint8_t)appCarMain.mode;
    view.fatherState = (uint8_t)appCarMain.fatherState;
    view.elapsedMs = appCarMain.elapsedMs;
    view.routePulses = appCarMain.routePulses;
    view.ballTargetMm = appCarMain.ballTargetMm;
    view.ballOffsetMm = appCarMain.ballOffsetMm;
    view.ballValid = appCarMain.ballValid;
    view.gray = appCarMain.gray;
    BspOled_ShowStatus(&view);
}
