#include "app_car.h"

#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_k230.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

/* Track speeds: tune these four values at the field. */
#define APP_CAR_TRACE_SPEED_H2           (26)       // 第二问速度
#define APP_CAR_TRACE_SPEED_H4           (32)       // 第四问速度
#define APP_CAR_TRACE_SPEED_H5           (28)       // 第五问速度
#define APP_CAR_TRACE_SPEED_H6           (26)       // 第六问速度

/* H2/H5/H6 finish protection. 13 PPR * 28 gear ratio * 4 edges. */
#define APP_CAR_ENCODER_COUNTS_PER_REV   (13U * 28U * 4U)
#define APP_CAR_WHEEL_CIRCUMFERENCE_MM   (204U)
#define APP_CAR_LAP_LENGTH_MM            (6142U)
#define APP_CAR_LAP_GATE_PERCENT         (80U)     // 第二问 里程放行比例，当前80%  当走了总路程的80%后 才允许停车
#define APP_CAR_LAP_EXPECTED_PULSES      \
    ((APP_CAR_LAP_LENGTH_MM * APP_CAR_ENCODER_COUNTS_PER_REV) / \
     APP_CAR_WHEEL_CIRCUMFERENCE_MM)
#define APP_CAR_LAP_GATE_PULSES          \
    ((APP_CAR_LAP_EXPECTED_PULSES * APP_CAR_LAP_GATE_PERCENT) / 100U)
#define APP_CAR_FINISH_CONFIRM_MS         (30U)
#define APP_CAR_FINISH_FOLLOW_MS          (500U)     // 第二问 过线后继续循迹时间
#define APP_CAR_FINISH_SENSOR_MIN         (4U)

#define APP_CAR_TARGET_MIN_MM             (-100)
#define APP_CAR_TARGET_MAX_MM             (100)
#define APP_CAR_BALL_CENTER_MM            (0)
#define APP_CAR_BALL_POSITIVE_MM          (50)
#define APP_CAR_BALL_NEGATIVE_MM          (-50)
#define APP_CAR_BALL_STABLE_ERROR_MM      (10)
#define APP_CAR_BALL_STABLE_MS            (250U)

static void _Init(AppCarDef *pCar);
static void _Run(AppCarDef *pCar, MsgId_t msg);
static void _SetMode(AppCarDef *pCar, AppCarMode_t mode);
static void _SetBallTargetMm(AppCarDef *pCar, int16_t targetMm);
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
static void _RunBallControl(AppCarDef *pCar, int16_t targetMm);
static uint8_t _IsBallStable(AppCarDef *pCar, int16_t targetMm);
static uint8_t _IsFinishLine(uint8_t gray);
static uint32_t _AbsCount(int32_t count);
static void _PrintTelemetry(const AppCarDef *pCar);
static int16_t _GetTraceSpeed(AppCarMode_t mode);

AppCarDef appCarMain;

AppCarConDef appCarCon = {
    .init = _Init,
    .run = _Run,
    .setMode = _SetMode,
    .setBallTargetMm = _SetBallTargetMm,
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
    pCar->routePulses = 0U;
    pCar->finishLineMs = 0U;
    pCar->ballStableMs = 0U;
    pCar->ballTargetMm = 0;
    pCar->ballOffsetMm = 0;
    pCar->leftSpeed = 0;
    pCar->rightSpeed = 0;
    pCar->leftCount = 0;
    pCar->rightCount = 0;
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->traceTurn = 0;
    pCar->traceState = (uint8_t)TRACE_STATE_SEARCHING;
    pCar->ballValid = 0U;
    pCar->gray = BSP_GRAY_ALL_WHITE;
    pCar->timerRunning = 0U;

    TraceControl_Init(&pCar->trace);
    BallControl_Init(&pCar->ballControl);
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(pCar->mode));
    _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
    _SetBallState(pCar, APP_CAR_BALL_DISABLED);
    _EnterStopped(pCar);
    _SampleInputs(pCar);

    BspUart_Printf("\n[H] state-machine foundation ready\n");
    BspUart_Printf("[H] 2..6=mode, s=start, x=stop\n");
    BspUart_Printf("[K230] B,<offset_mm>,<valid>\\n\n");
    _PrintTelemetry(pCar);
}

static void _Run(AppCarDef *pCar, MsgId_t msg)
{
    if ((pCar == 0) || (pCar->pFatherState == 0)) {
        return;
    }

    pCar->pFatherState(pCar, msg);
}

void AppCar_Tick1ms(AppCarDef *pCar)
{
    if ((pCar != 0) && (pCar->timerRunning != 0U)) {
        pCar->elapsedMs++;
    }
}

static void _SetMode(AppCarDef *pCar, AppCarMode_t mode)
{
    if ((pCar == 0) ||
        (mode > APP_CAR_MODE_BALANCE_LAP_TARGET) ||
        (pCar->fatherState == APP_CAR_FATHER_RUNNING)) {
        return;
    }

    pCar->mode = mode;
    if (mode == APP_CAR_MODE_BALL_STATIC) {
        pCar->ballTargetMm = APP_CAR_BALL_CENTER_MM;
        BallControl_Reset(&pCar->ballControl);
    } else {
        BspServo_Center();
    }
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(mode));
    BspUart_Printf("[MODE] %u\n", (unsigned)mode);
}

static void _SetBallTargetMm(AppCarDef *pCar, int16_t targetMm)
{
    if ((pCar == 0) || (pCar->fatherState == APP_CAR_FATHER_RUNNING)) {
        return;
    }

    if (targetMm < APP_CAR_TARGET_MIN_MM) {
        targetMm = APP_CAR_TARGET_MIN_MM;
    } else if (targetMm > APP_CAR_TARGET_MAX_MM) {
        targetMm = APP_CAR_TARGET_MAX_MM;
    }

    pCar->ballTargetMm = targetMm;
    BspUart_Printf("[BALL] target=%d mm\n", (int)targetMm);
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
    } else if (msg == MSG_CONTROL_TICK) {
        _SampleInputs(pCar);
        if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
            pCar->ballTargetMm = APP_CAR_BALL_CENTER_MM;
            _RunBallControl(pCar, APP_CAR_BALL_CENTER_MM);
        }
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
    } else if (msg == MSG_CONTROL_TICK) {
        _SampleInputs(pCar);
        pCar->routeStateMs += APP_CAR_CONTROL_PERIOD_MS;
        pCar->ballStateMs += APP_CAR_CONTROL_PERIOD_MS;
        _RunChildren(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        /* Do not block the 2 ms control loop with UART while driving. */
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
    } else if (msg == MSG_CONTROL_TICK) {
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
    } else if (msg == MSG_CONTROL_TICK) {
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
    if (_IsFinishLine(pCar->gray) != 0U) {
        pCar->leftCommand = pCar->trace.baseSpeed;
        pCar->rightCommand = pCar->trace.baseSpeed;
        BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
        return;
    }

    _SetRouteState(pCar, APP_CAR_ROUTE_TRACKING);
    _RunTraceControl(pCar);
}

static void _RouteTracking(AppCarDef *pCar)
{
    if ((pCar->mode != APP_CAR_MODE_BALANCE_AB) &&
        (pCar->routePulses >= APP_CAR_LAP_GATE_PULSES) &&
        (_IsFinishLine(pCar->gray) != 0U)) {
        if (pCar->finishLineMs < APP_CAR_FINISH_CONFIRM_MS) {
            pCar->finishLineMs += APP_CAR_CONTROL_PERIOD_MS;
        }
        if (pCar->finishLineMs >= APP_CAR_FINISH_CONFIRM_MS) {
            _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
            pCar->pRouteState(pCar);
            return;
        }
    } else {
        pCar->finishLineMs = 0U;
    }

    _RunTraceControl(pCar);
}

static void _RouteFinishAction(AppCarDef *pCar)
{
    if (pCar->routeStateMs < APP_CAR_FINISH_FOLLOW_MS) {
        _RunTraceControl(pCar);
        return;
    }

    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->timerRunning = 0U;
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
        pCar->ballTargetMm = APP_CAR_BALL_POSITIVE_MM;
        _SetBallState(pCar, APP_CAR_BALL_MOVE_POSITIVE);
    } else {
        _SetRouteState(pCar, APP_CAR_ROUTE_LEAVE_START);
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
    }
}

static void _BallMovePositive(AppCarDef *pCar)
{
    pCar->ballTargetMm = APP_CAR_BALL_POSITIVE_MM;
    _RunBallControl(pCar, APP_CAR_BALL_POSITIVE_MM);

    if (_IsBallStable(pCar, APP_CAR_BALL_POSITIVE_MM) != 0U) {
        pCar->ballTargetMm = APP_CAR_BALL_NEGATIVE_MM;
        _SetBallState(pCar, APP_CAR_BALL_MOVE_NEGATIVE);
    }
}

static void _BallMoveNegative(AppCarDef *pCar)
{
    pCar->ballTargetMm = APP_CAR_BALL_NEGATIVE_MM;
    _RunBallControl(pCar, APP_CAR_BALL_NEGATIVE_MM);

    if (_IsBallStable(pCar, APP_CAR_BALL_NEGATIVE_MM) != 0U) {
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
        _EnterFinished(pCar);
    }
}

static void _BallHoldTarget(AppCarDef *pCar)
{
    _RunBallControl(pCar, pCar->ballTargetMm);
}

static void _EnterStopped(AppCarDef *pCar)
{
    BspMotor_Stop();
    BspServo_Center();
    TraceControl_Init(&pCar->trace);
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(pCar->mode));
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->traceTurn = 0;
    pCar->traceState = (uint8_t)TRACE_STATE_SEARCHING;
    pCar->ballStableMs = 0U;
    pCar->timerRunning = 0U;
    BallControl_Reset(&pCar->ballControl);
    pCar->fatherState = APP_CAR_FATHER_STOPPED;
    pCar->pFatherState = _FatherStopped;
    _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
    _SetBallState(pCar, APP_CAR_BALL_DISABLED);
}

static void _EnterRunning(AppCarDef *pCar)
{
    pCar->elapsedMs = 0U;
    pCar->routePulses = 0U;
    pCar->leftCount = 0;
    pCar->rightCount = 0;
    pCar->timerRunning = 1U;
    BspEncoder_Reset();
    BspMotor_Stop();
    BspServo_Center();
    TraceControl_Init(&pCar->trace);
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(pCar->mode));
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->ballStableMs = 0U;
    pCar->fatherState = APP_CAR_FATHER_RUNNING;
    pCar->pFatherState = _FatherRunning;
    BallControl_Reset(&pCar->ballControl);
    _ConfigureChildren(pCar);
    BspUart_Printf("[RUN] start mode=%u\n", (unsigned)pCar->mode);
}

static void _EnterFinished(AppCarDef *pCar)
{
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->timerRunning = 0U;
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
    pCar->timerRunning = 0U;
    pCar->fatherState = APP_CAR_FATHER_FAULT;
    pCar->pFatherState = _FatherFault;
    BspUart_Printf("[FAULT] stopped\n");
}

static void _SetRouteState(AppCarDef *pCar, AppCarRouteState_t state)
{
    pCar->routeState = state;
    pCar->routeStateMs = 0U;
    pCar->finishLineMs = 0U;

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
    pCar->ballStableMs = 0U;
    BallControl_Reset(&pCar->ballControl);

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
        if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) ||
            (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER)) {
            pCar->ballTargetMm = 0;
        }
        _SetRouteState(pCar, APP_CAR_ROUTE_DISABLED);
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
    pCar->leftCount = encoder.leftCount;
    pCar->rightCount = encoder.rightCount;
    pCar->routePulses =
        (_AbsCount(encoder.leftCount) + _AbsCount(encoder.rightCount)) / 2U;
    pCar->ballOffsetMm = ball.offsetMm;
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

static void _RunBallControl(AppCarDef *pCar, int16_t targetMm)
{
    uint16_t pulseUs = BallControl_Update(&pCar->ballControl,
        targetMm, pCar->ballOffsetMm, pCar->ballValid);

    BspServo_SetPulseUs(pulseUs);
}

static uint8_t _IsBallStable(AppCarDef *pCar, int16_t targetMm)
{
    int16_t errorMm;

    if (pCar->ballValid == 0U) {
        pCar->ballStableMs = 0U;
        return 0U;
    }

    errorMm = (int16_t)(targetMm - pCar->ballOffsetMm);
    if (errorMm < 0) {
        errorMm = (int16_t)-errorMm;
    }

    if (errorMm <= APP_CAR_BALL_STABLE_ERROR_MM) {
        if (pCar->ballStableMs < APP_CAR_BALL_STABLE_MS) {
            pCar->ballStableMs += APP_CAR_CONTROL_PERIOD_MS;
        }
    } else {
        pCar->ballStableMs = 0U;
    }

    return (uint8_t)(pCar->ballStableMs >= APP_CAR_BALL_STABLE_MS);
}

static uint8_t _IsFinishLine(uint8_t gray)
{
    uint8_t blackCount = 0U;

    while (gray != 0U) {
        blackCount += (uint8_t)(gray & 0x01U);
        gray >>= 1;
    }

    return (uint8_t)(blackCount >= APP_CAR_FINISH_SENSOR_MIN);
}

static uint32_t _AbsCount(int32_t count)
{
    if (count >= 0) {
        return (uint32_t)count;
    }

    return (uint32_t)(-(count + 1)) + 1U;
}

static void _PrintTelemetry(const AppCarDef *pCar)
{
    BspK230Debug_t k230Debug;

    BspK230_GetDebug(&k230Debug);
    BspUart_Printf(
        "[T] f=%u r=%u b=%u mode=%u time=%lu target=%dmm ball=%d/%u gray=%02X enc=%d,%d count=%ld,%ld lap=%lu cmd=%d,%d tr=%d/%u q=%u k=%lu,%lu,%lu,%lu\n",
        (unsigned)pCar->fatherState,
        (unsigned)pCar->routeState,
        (unsigned)pCar->ballState,
        (unsigned)pCar->mode,
        (unsigned long)pCar->elapsedMs,
        (int)pCar->ballTargetMm,
        (int)pCar->ballOffsetMm,
        (unsigned)pCar->ballValid,
        (unsigned)pCar->gray,
        (int)pCar->leftSpeed,
        (int)pCar->rightSpeed,
        (long)pCar->leftCount,
        (long)pCar->rightCount,
        (unsigned long)pCar->routePulses,
        (int)pCar->leftCommand,
        (int)pCar->rightCommand,
        (int)pCar->traceTurn,
        (unsigned)pCar->traceState,
        (unsigned)MsgMap_GetOverflowCount(),
        (unsigned long)k230Debug.rxBytes,
        (unsigned long)k230Debug.pollBytes,
        (unsigned long)k230Debug.lines,
        (unsigned long)k230Debug.parsed);
}

static int16_t _GetTraceSpeed(AppCarMode_t mode)
{
    switch (mode) {
        case APP_CAR_MODE_BALANCE_AB:
            return APP_CAR_TRACE_SPEED_H4;
        case APP_CAR_MODE_BALANCE_LAP_CENTER:
            return APP_CAR_TRACE_SPEED_H5;
        case APP_CAR_MODE_BALANCE_LAP_TARGET:
            return APP_CAR_TRACE_SPEED_H6;
        case APP_CAR_MODE_TRACE_ONLY:
        case APP_CAR_MODE_BALL_STATIC:
        default:
            return APP_CAR_TRACE_SPEED_H2;
    }
}
