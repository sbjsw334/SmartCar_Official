#include "app_car.h"

#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_k230.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

#define APP_CAR_CONTROL_PERIOD_MS (10U)
#define APP_CAR_LEAVE_SPEED       (20)

static void _Init(AppCarDef *pCar);
static void _Run(AppCarDef *pCar, MsgId_t msg);
static void _SetMode(AppCarDef *pCar, AppCarMode_t mode);
static AppCarFatherState_t _GetFatherState(const AppCarDef *pCar);
static AppCarRouteState_t _GetRouteState(const AppCarDef *pCar);
static AppCarBallState_t _GetBallState(const AppCarDef *pCar);

static void _FatherStopped(AppCarDef *pCar, MsgId_t msg);
static void _FatherRunning(AppCarDef *pCar, MsgId_t msg);
static void _FatherFinished(AppCarDef *pCar, MsgId_t msg);
static void _FatherFault(AppCarDef *pCar, MsgId_t msg);

static void _RouteDisabled(AppCarDef *pCar);
static void _RouteLeaveStart(AppCarDef *pCar);
static void _RouteTracking(AppCarDef *pCar);
static void _RouteFinishAction(AppCarDef *pCar);
static void _RouteComplete(AppCarDef *pCar);

static void _BallDisabled(AppCarDef *pCar);
static void _BallWaitVision(AppCarDef *pCar);
static void _BallMovePositive(AppCarDef *pCar);
static void _BallMoveNegative(AppCarDef *pCar);
static void _BallHoldTarget(AppCarDef *pCar);

static void _EnterStopped(AppCarDef *pCar);
static void _EnterRunning(AppCarDef *pCar);
static void _EnterFinished(AppCarDef *pCar);
static void _EnterFault(AppCarDef *pCar);
static void _SetRouteState(AppCarDef *pCar, AppCarRouteState_t state);
static void _SetBallState(AppCarDef *pCar, AppCarBallState_t state);
static void _ConfigureChildren(AppCarDef *pCar);
static void _RunChildren(AppCarDef *pCar);
static void _SampleInputs(AppCarDef *pCar);
static void _RunTraceControl(AppCarDef *pCar);
static void _PrintTelemetry(const AppCarDef *pCar);

AppCarDef appCarMain;

AppCarConDef appCarCon = {
    .init = _Init,
    .run = _Run,
    .setMode = _SetMode,
    .getFatherState = _GetFatherState,
    .getRouteState = _GetRouteState,
    .getBallState = _GetBallState,
};

static void _Init(AppCarDef *pCar)
{
    if (pCar == 0) {
        return;
    }

    pCar->mode = APP_CAR_MODE_TRACE_ONLY;
    pCar->elapsedMs = 0U;
    pCar->routeStateMs = 0U;
    pCar->ballStateMs = 0U;
    pCar->ballOffsetPx = 0;
    pCar->leftSpeed = 0;
    pCar->rightSpeed = 0;
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->traceTurn = 0;
    pCar->traceState = (uint8_t)TRACE_STATE_SEARCHING;
    pCar->ballValid = 0U;
    pCar->gray = BSP_GRAY_ALL_WHITE;

    TraceControl_Init(&pCar->trace);
    _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
    _SetBallState(pCar, APP_CAR_BALL_DISABLED);
    _EnterStopped(pCar);
    _SampleInputs(pCar);

    BspUart_Printf("\n[H] state-machine foundation ready\n");
    BspUart_Printf("[H] 1..5=mode, s=start, x=stop\n");
    BspUart_Printf("[K230] B,<offset_px>,<valid>\\n\n");
    _PrintTelemetry(pCar);
}

static void _Run(AppCarDef *pCar, MsgId_t msg)
{
    if ((pCar == 0) || (pCar->pFatherState == 0)) {
        return;
    }

    pCar->pFatherState(pCar, msg);
}

static void _SetMode(AppCarDef *pCar, AppCarMode_t mode)
{
    if ((pCar == 0) ||
        (mode > APP_CAR_MODE_BALANCE_LAP_TARGET) ||
        (pCar->fatherState == APP_CAR_FATHER_RUNNING)) {
        return;
    }

    pCar->mode = mode;
    BspUart_Printf("[MODE] %u\n", (unsigned)mode);
}

static AppCarFatherState_t _GetFatherState(const AppCarDef *pCar)
{
    return (pCar == 0) ? APP_CAR_FATHER_FAULT : pCar->fatherState;
}

static AppCarRouteState_t _GetRouteState(const AppCarDef *pCar)
{
    return (pCar == 0) ? APP_CAR_ROUTE_DISABLED : pCar->routeState;
}

static AppCarBallState_t _GetBallState(const AppCarDef *pCar)
{
    return (pCar == 0) ? APP_CAR_BALL_DISABLED : pCar->ballState;
}

static void _FatherStopped(AppCarDef *pCar, MsgId_t msg)
{
    if (msg == MSG_KEY_START) {
        _EnterRunning(pCar);
    } else if (msg == MSG_FAULT) {
        _EnterFault(pCar);
    } else if (msg == MSG_CONTROL_10MS) {
        _SampleInputs(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        _PrintTelemetry(pCar);
    }
}

static void _FatherRunning(AppCarDef *pCar, MsgId_t msg)
{
    if ((msg == MSG_KEY_START) || (msg == MSG_STOP)) {
        _EnterStopped(pCar);
    } else if (msg == MSG_FAULT) {
        _EnterFault(pCar);
    } else if (msg == MSG_CONTROL_10MS) {
        _SampleInputs(pCar);
        pCar->elapsedMs += APP_CAR_CONTROL_PERIOD_MS;
        pCar->routeStateMs += APP_CAR_CONTROL_PERIOD_MS;
        pCar->ballStateMs += APP_CAR_CONTROL_PERIOD_MS;
        _RunChildren(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        _PrintTelemetry(pCar);
    }
}

static void _FatherFinished(AppCarDef *pCar, MsgId_t msg)
{
    if (msg == MSG_KEY_START) {
        _EnterRunning(pCar);
    } else if (msg == MSG_STOP) {
        _EnterStopped(pCar);
    } else if (msg == MSG_FAULT) {
        _EnterFault(pCar);
    } else if (msg == MSG_CONTROL_10MS) {
        _SampleInputs(pCar);
        pCar->ballStateMs += APP_CAR_CONTROL_PERIOD_MS;
        pCar->pBallState(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        _PrintTelemetry(pCar);
    }
}

static void _FatherFault(AppCarDef *pCar, MsgId_t msg)
{
    if ((msg == MSG_STOP) || (msg == MSG_KEY_START)) {
        _EnterStopped(pCar);
    } else if (msg == MSG_CONTROL_10MS) {
        _SampleInputs(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        _PrintTelemetry(pCar);
    }
}

static void _RouteDisabled(AppCarDef *pCar)
{
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
}

static void _RouteLeaveStart(AppCarDef *pCar)
{
    if (pCar->gray == BSP_GRAY_ALL_BLACK) {
        pCar->leftCommand = APP_CAR_LEAVE_SPEED;
        pCar->rightCommand = APP_CAR_LEAVE_SPEED;
        BspMotor_SetSignedSpeed(APP_CAR_LEAVE_SPEED, APP_CAR_LEAVE_SPEED);
        return;
    }

    _SetRouteState(pCar, APP_CAR_ROUTE_TRACKING);
    _RunTraceControl(pCar);
}

static void _RouteTracking(AppCarDef *pCar)
{
    if ((pCar->gray == BSP_GRAY_ALL_BLACK) &&
        (pCar->mode != APP_CAR_MODE_BALANCE_AB)) {
        _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
        pCar->pRouteState(pCar);
        return;
    }

    _RunTraceControl(pCar);
}

static void _RouteFinishAction(AppCarDef *pCar)
{
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
}

static void _RouteComplete(AppCarDef *pCar)
{
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    _EnterFinished(pCar);
}

static void _BallDisabled(AppCarDef *pCar)
{
    (void)pCar;
    BspServo_Center();
}

static void _BallWaitVision(AppCarDef *pCar)
{
    BspServo_Center();

    if (pCar->ballValid == 0U) {
        return;
    }

    if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
        _SetBallState(pCar, APP_CAR_BALL_MOVE_POSITIVE);
    } else {
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
    }
}

static void _BallMovePositive(AppCarDef *pCar)
{
    /* Firmware owner: run ball controller toward +50 mm, then switch state. */
    (void)pCar;
    BspServo_Center();
}

static void _BallMoveNegative(AppCarDef *pCar)
{
    /* Firmware owner: run ball controller toward -50 mm, then switch state. */
    (void)pCar;
    BspServo_Center();
}

static void _BallHoldTarget(AppCarDef *pCar)
{
    /* Firmware owner: keep running ball position PD at the selected target. */
    (void)pCar;
    BspServo_Center();
}

static void _EnterStopped(AppCarDef *pCar)
{
    BspMotor_Stop();
    BspServo_Center();
    TraceControl_Init(&pCar->trace);
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->traceTurn = 0;
    pCar->traceState = (uint8_t)TRACE_STATE_SEARCHING;
    pCar->fatherState = APP_CAR_FATHER_STOPPED;
    pCar->pFatherState = _FatherStopped;
    _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
    _SetBallState(pCar, APP_CAR_BALL_DISABLED);
}

static void _EnterRunning(AppCarDef *pCar)
{
    pCar->elapsedMs = 0U;
    BspEncoder_Reset();
    BspMotor_Stop();
    BspServo_Center();
    TraceControl_Init(&pCar->trace);
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->fatherState = APP_CAR_FATHER_RUNNING;
    pCar->pFatherState = _FatherRunning;
    _ConfigureChildren(pCar);
    BspUart_Printf("[RUN] start mode=%u\n", (unsigned)pCar->mode);
}

static void _EnterFinished(AppCarDef *pCar)
{
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->fatherState = APP_CAR_FATHER_FINISHED;
    pCar->pFatherState = _FatherFinished;
    BspUart_Printf("[RUN] finished time=%lu ms\n",
        (unsigned long)pCar->elapsedMs);
}

static void _EnterFault(AppCarDef *pCar)
{
    BspMotor_Stop();
    BspServo_Center();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->fatherState = APP_CAR_FATHER_FAULT;
    pCar->pFatherState = _FatherFault;
    BspUart_Printf("[FAULT] stopped\n");
}

static void _SetRouteState(AppCarDef *pCar, AppCarRouteState_t state)
{
    pCar->routeState = state;
    pCar->routeStateMs = 0U;

    switch (state) {
        case APP_CAR_ROUTE_LEAVE_START:
            pCar->pRouteState = _RouteLeaveStart;
            break;
        case APP_CAR_ROUTE_TRACKING:
            pCar->pRouteState = _RouteTracking;
            break;
        case APP_CAR_ROUTE_FINISH_ACTION:
            pCar->pRouteState = _RouteFinishAction;
            break;
        case APP_CAR_ROUTE_COMPLETE:
            pCar->pRouteState = _RouteComplete;
            break;
        case APP_CAR_ROUTE_DISABLED:
        default:
            pCar->pRouteState = _RouteDisabled;
            break;
    }
}

static void _SetBallState(AppCarDef *pCar, AppCarBallState_t state)
{
    pCar->ballState = state;
    pCar->ballStateMs = 0U;

    switch (state) {
        case APP_CAR_BALL_WAIT_VISION:
            pCar->pBallState = _BallWaitVision;
            break;
        case APP_CAR_BALL_MOVE_POSITIVE:
            pCar->pBallState = _BallMovePositive;
            break;
        case APP_CAR_BALL_MOVE_NEGATIVE:
            pCar->pBallState = _BallMoveNegative;
            break;
        case APP_CAR_BALL_HOLD_TARGET:
            pCar->pBallState = _BallHoldTarget;
            break;
        case APP_CAR_BALL_DISABLED:
        default:
            pCar->pBallState = _BallDisabled;
            break;
    }
}

static void _ConfigureChildren(AppCarDef *pCar)
{
    if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
        _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
        _SetBallState(pCar, APP_CAR_BALL_WAIT_VISION);
    } else if (pCar->mode == APP_CAR_MODE_TRACE_ONLY) {
        _SetRouteState(pCar, APP_CAR_ROUTE_LEAVE_START);
        _SetBallState(pCar, APP_CAR_BALL_DISABLED);
    } else {
        _SetRouteState(pCar, APP_CAR_ROUTE_LEAVE_START);
        _SetBallState(pCar, APP_CAR_BALL_WAIT_VISION);
    }
}

static void _RunChildren(AppCarDef *pCar)
{
    if ((pCar->pRouteState == 0) || (pCar->pBallState == 0)) {
        _EnterFault(pCar);
        return;
    }

    pCar->pRouteState(pCar);
    if (pCar->fatherState == APP_CAR_FATHER_RUNNING) {
        pCar->pBallState(pCar);
    }
}

static void _SampleInputs(AppCarDef *pCar)
{
    BspEncoderData_t encoder;
    BspK230Ball_t ball;

    BspEncoder_GetData(&encoder);
    BspK230_GetBall(&ball);

    pCar->gray = BspGray_GetFiltered();
    pCar->leftSpeed = encoder.leftPps;
    pCar->rightSpeed = encoder.rightPps;
    pCar->ballOffsetPx = ball.offsetPx;
    pCar->ballValid = ball.valid;
}

static void _RunTraceControl(AppCarDef *pCar)
{
    TraceControl_Update(&pCar->trace, pCar->gray);

    pCar->leftCommand = pCar->trace.leftCommand;
    pCar->rightCommand = pCar->trace.rightCommand;
    pCar->traceTurn = pCar->trace.lastTurn;
    pCar->traceState = (uint8_t)pCar->trace.state;

    BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
}

static void _PrintTelemetry(const AppCarDef *pCar)
{
    BspUart_Printf(
        "[T] f=%u r=%u b=%u mode=%u time=%lu ball=%d/%u gray=%02X enc=%d,%d cmd=%d,%d tr=%d/%u q=%u\n",
        (unsigned)pCar->fatherState,
        (unsigned)pCar->routeState,
        (unsigned)pCar->ballState,
        (unsigned)pCar->mode,
        (unsigned long)pCar->elapsedMs,
        (int)pCar->ballOffsetPx,
        (unsigned)pCar->ballValid,
        (unsigned)pCar->gray,
        (int)pCar->leftSpeed,
        (int)pCar->rightSpeed,
        (int)pCar->leftCommand,
        (int)pCar->rightCommand,
        (int)pCar->traceTurn,
        (unsigned)pCar->traceState,
        (unsigned)MsgMap_GetOverflowCount());
}
