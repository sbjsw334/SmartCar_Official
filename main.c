#include <stdint.h>

#include "app_car.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_k230.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "msg_map.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t s_controlMs = 0U;
static volatile uint8_t s_telemetryMs = 0U;

static void _ProcessDebugCommands(void);

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
    BspK230_Init();
    appCarCon.init(&appCarMain);

    __enable_irq();

    while (1) {
        BspK230_Task();
        _ProcessDebugCommands();

        if (BspKey_GetPressEvent() != 0U) {
            (void)MsgMap_Post(MSG_KEY_START);
        }

        while (MsgMap_Get(&msg) != 0U) {
            appCarCon.run(&appCarMain, msg);
        }
    }
}

void SysTick_Handler(void)
{
    BspKey_Task1ms();
    BspGray_Task1ms();
    BspEncoder_Task1ms();
    BspK230_Task1ms();

    s_controlMs++;
    if (s_controlMs >= 10U) {
        s_controlMs = 0U;
        (void)MsgMap_Post(MSG_CONTROL_10MS);
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
            case '1':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_TRACE_ONLY);
                break;

            case '2':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALL_STATIC);
                break;

            case '3':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALANCE_AB);
                break;

            case '4':
                appCarCon.setMode(&appCarMain, APP_CAR_MODE_BALANCE_LAP_CENTER);
                break;

            case '5':
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
