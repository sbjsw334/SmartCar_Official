#include "app_car.h"

#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_jy61p.h"
#include "bsp_k230.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

/* Track speeds: tune these four values at the field. */
#define APP_CAR_TRACE_SPEED_H2           (36)       /* H2 normal 8-sensor trace speed */
#define APP_CAR_TRACE_SPEED_H4           (32)       /* H4 speed */
#define APP_CAR_TRACE_SPEED_H5           (28)       /* H5 speed */
#define APP_CAR_TRACE_SPEED_H6           (26)       /* H6 speed */

/* H2/H5/H6 lap protection. 13 PPR * 28 gear ratio * 4 edges. */
#define APP_CAR_ENCODER_COUNTS_PER_REV   (13U * 28U * 4U)
#define APP_CAR_WHEEL_CIRCUMFERENCE_MM   (204U)
#define APP_CAR_LAP_LENGTH_MM            (6142U)
#define APP_CAR_LAP_GATE_PERCENT         (80U)      /* H2: allow finish line after 80% lap */
#define APP_CAR_LAP_EXPECTED_PULSES      \
    ((APP_CAR_LAP_LENGTH_MM * APP_CAR_ENCODER_COUNTS_PER_REV) / \
     APP_CAR_WHEEL_CIRCUMFERENCE_MM)
#define APP_CAR_LAP_GATE_PULSES          \
    ((APP_CAR_LAP_EXPECTED_PULSES * APP_CAR_LAP_GATE_PERCENT) / 100U)
#define APP_CAR_FINISH_CONFIRM_MS        (30U)      /* H2: finish line must last 30 ms */
#define APP_CAR_FINISH_SENSOR_MIN        (4U)       /* H2: 4+ black sensors means cross line */
#define APP_CAR_FINISH_FOLLOW_MS         (400U)     /* H5/H6: keep old finish follow logic */

/* H2 simple finish: 8-sensor trace -> finish line confirm -> follow a little -> JY61P align.
 * 第二题只在 START 和最终回正时使用陀螺仪；正常循迹过程完全不依赖陀螺仪。
 */
#define APP_CAR_H2_FINISH_FOLLOW_MS      (400U)     /* 第二题：识别回A线后继续循迹时间，太短会早停，太长会冲过 */
#define APP_CAR_H2_FINISH_BRAKE_MS       (80U)      /* H2: reverse brake time */
#define APP_CAR_H2_FINISH_BRAKE_SPEED    (20)       /* H2: reverse brake strength */
#define APP_CAR_H2_FINISH_ALIGN_ENABLE   (1U)       /* 第二题：停车后是否用JY61P回到起始角度，0=关闭 */
#define APP_CAR_H2_FINISH_ALIGN_ERROR_CD (300L)     /* 第二题：回正允许误差，300=3度 */
#define APP_CAR_H2_FINISH_ALIGN_STABLE_MS (100U)    /* 第二题：回正到位后保持时间 */
#define APP_CAR_H2_FINISH_ALIGN_TIMEOUT_MS (1000U)  /* 第二题：回正最长时间，超时直接结束 */
#define APP_CAR_H2_FINISH_ALIGN_SPEED    (12)       /* 第二题：原地回正速度，太大易过冲，太小可能转不动 */
#define APP_CAR_H2_FINISH_ALIGN_DIRECTION (1)       /* 第二题：回正方向补偿，若越转越偏改成-1 */

/* H4 parameters: A -> B with ball held near O.
 * H4调参只优先改下面这些值：
 * 1) H4_RAMP_MS：起步缓启动时间，越大越稳但越慢。
 * 2) H4_START_FF_MM：起步球位前馈，默认0；如果起步球总往同一边甩，再改正负和大小。
 * 3) H4_AB_LENGTH_MM / H4_AB_GATE_PERCENT：B点横线放行距离，防止刚出发误识别横线。
 * 4) H4_FINISH_FOLLOW_MS：过B停表后继续循迹时间，只为停车更柔，不计入H4时间。
 */
#define APP_CAR_H4_AB_LENGTH_MM           (1500U)
#define APP_CAR_H4_AB_EXPECTED_PULSES     \
    ((APP_CAR_H4_AB_LENGTH_MM * APP_CAR_ENCODER_COUNTS_PER_REV) / \
     APP_CAR_WHEEL_CIRCUMFERENCE_MM)
#define APP_CAR_H4_AB_GATE_PERCENT        (70U)
#define APP_CAR_H4_AB_GATE_PULSES         \
    ((APP_CAR_H4_AB_EXPECTED_PULSES * APP_CAR_H4_AB_GATE_PERCENT) / 100U)
#define APP_CAR_H4_RAMP_START_SPEED       (18)
#define APP_CAR_H4_RAMP_MS                (900U)
#define APP_CAR_H4_START_READY_ERROR_MM   (10)
#define APP_CAR_H4_START_READY_MS         (200U)
#define APP_CAR_H4_START_FF_MM            (0)
#define APP_CAR_H4_START_FF_HOLD_MS       (500U)
#define APP_CAR_H4_START_FF_FADE_MS       (700U)
#define APP_CAR_H4_FINISH_FOLLOW_MS       (300U)

#define APP_CAR_TARGET_MIN_MM             (-100)
#define APP_CAR_TARGET_MAX_MM             (100)
#define APP_CAR_BALL_CENTER_MM            (0)
#define APP_CAR_BALL_POSITIVE_MM          (50)
#define APP_CAR_BALL_NEGATIVE_MM          (-50)
#define APP_CAR_BALL_POSITIVE_CONTROL_MM  (53)
#define APP_CAR_BALL_NEGATIVE_CONTROL_MM  (-58)
#define APP_CAR_BALL_TARGET_POS_STEP_MM   (3)
#define APP_CAR_BALL_TARGET_POS_PERIOD_MS (20U)
#define APP_CAR_BALL_TARGET_NEG_STEP_MM   (3)
#define APP_CAR_BALL_TARGET_NEG_PERIOD_MS (20U)
#define APP_CAR_BALL_REACH_ERROR_MM       (8)
#define APP_CAR_BALL_REACH_SPEED_MM_PER_S (45)
#define APP_CAR_BALL_FINAL_ERROR_MM       (8)
#define APP_CAR_BALL_FINAL_SPEED_MM_PER_S (30)
#define APP_CAR_BALL_FINAL_STABLE_MS      (180U)
#define APP_CAR_BALL_FRAME_MS             (33U)

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
static void _RouteFinishBrake(AppCarDef *pCar);
static void _RouteFinishAlign(AppCarDef *pCar);
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
static int16_t _GetCurrentTraceSpeed(const AppCarDef *pCar);
static int16_t _GetBallHoldTargetMm(const AppCarDef *pCar);
static int16_t _ClampBallTargetMm(int16_t targetMm);
static uint16_t _GetFinishFollowMs(AppCarMode_t mode);
static int16_t _StepBallTargetMm(AppCarDef *pCar, int16_t finalTargetMm);
static uint8_t _IsBallReached(const AppCarDef *pCar, int16_t targetMm);
static uint8_t _IsBallFinalStable(AppCarDef *pCar, int16_t targetMm);
static void _PrintBallStageResult(const AppCarDef *pCar, const char *name,
    int16_t targetMm);
static uint8_t _IsFinishLine(uint8_t gray);
static uint32_t _AbsCount(int32_t count);
static int32_t _AbsI32(int32_t value);
static int16_t _AngleDeltaCd(int16_t nowCd, int16_t lastCd);
static int32_t _H2HeadingErrorCd(const AppCarDef *pCar);
static void _ResetImuLap(AppCarDef *pCar);
static void _UpdateImuLap(AppCarDef *pCar, const BspJy61pData_t *pImu);
static uint8_t _CanFinishByRouteGate(const AppCarDef *pCar);
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
    pCar->uptimeMs = 0U;
    pCar->elapsedMs = 0U;
    pCar->routeStateMs = 0U;
    pCar->ballStateMs = 0U;
    pCar->routePulses = 0U;
    pCar->finishLineMs = 0U;
    pCar->finishAlignStableMs = 0U;
    pCar->imuLapYawCd = 0;
    pCar->imuYawCd = 0;
    pCar->imuLastYawCd = 0;
    pCar->imuSampleSeq = 0U;
    pCar->imuValid = 0U;
    pCar->imuOnline = 0U;
    pCar->imuHasLastYaw = 0U;
    pCar->ballStableMs = 0U;
    pCar->ballStableFrameSeq = 0U;
    pCar->ballTargetMm = 0;
    pCar->lastBallTargetMm = 0;
    pCar->ballOffsetMm = 0;
    pCar->ballFrameSeq = 0U;
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
    if (pCar != 0) {
        pCar->uptimeMs++;
        if (pCar->timerRunning != 0U) {
            pCar->elapsedMs++;
        }
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
        if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
            _PrintTelemetry(pCar);
        }
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
    int16_t speed;

    if (_IsFinishLine(pCar->gray) != 0U) {
        speed = _GetCurrentTraceSpeed(pCar);
        pCar->leftCommand = speed;
        pCar->rightCommand = speed;
        BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
        return;
    }

    _SetRouteState(pCar, APP_CAR_ROUTE_TRACKING);
    _RunTraceControl(pCar);
}

static void _RouteTracking(AppCarDef *pCar)
{
    if (_CanFinishByRouteGate(pCar) &&
        (_IsFinishLine(pCar->gray) != 0U)) {
        if (pCar->finishLineMs < APP_CAR_FINISH_CONFIRM_MS) {
            pCar->finishLineMs += APP_CAR_CONTROL_PERIOD_MS;
        }
        if (pCar->finishLineMs >= APP_CAR_FINISH_CONFIRM_MS) {
            if (pCar->mode == APP_CAR_MODE_TRACE_ONLY) {
                _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
            } else {
                if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
                    pCar->timerRunning = 0U;
                }
                _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
            }
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
    if (pCar->routeStateMs < _GetFinishFollowMs(pCar->mode)) {
        _RunTraceControl(pCar);
        return;
    }
    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->timerRunning = 0U;
    if (pCar->mode == APP_CAR_MODE_TRACE_ONLY) {
        _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_BRAKE);
    } else {
        _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
    }
    pCar->pRouteState(pCar);
}

static void _RouteFinishBrake(AppCarDef *pCar)
{
    if (pCar->routeStateMs < APP_CAR_H2_FINISH_BRAKE_MS) {
        pCar->leftCommand = -APP_CAR_H2_FINISH_BRAKE_SPEED;
        pCar->rightCommand = -APP_CAR_H2_FINISH_BRAKE_SPEED;
        BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
        return;
    }

    BspMotor_Stop();
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    if ((pCar->mode == APP_CAR_MODE_TRACE_ONLY) &&
        (APP_CAR_H2_FINISH_ALIGN_ENABLE != 0U) &&
        (pCar->imuOnline != 0U) &&
        (pCar->imuHasLastYaw != 0U)) {
        _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ALIGN);
    } else {
        _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
    }
}

static void _RouteFinishAlign(AppCarDef *pCar)
{
    int32_t errorCd = _H2HeadingErrorCd(pCar);
    int16_t turnSpeed;

    if ((pCar->imuOnline == 0U) || (pCar->imuHasLastYaw == 0U) ||
        (pCar->routeStateMs >= APP_CAR_H2_FINISH_ALIGN_TIMEOUT_MS)) {
        BspMotor_Stop();
        pCar->leftCommand = 0;
        pCar->rightCommand = 0;
        _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
        return;
    }

    if (_AbsI32(errorCd) <= APP_CAR_H2_FINISH_ALIGN_ERROR_CD) {
        BspMotor_Stop();
        pCar->leftCommand = 0;
        pCar->rightCommand = 0;
        if (pCar->finishAlignStableMs < APP_CAR_H2_FINISH_ALIGN_STABLE_MS) {
            pCar->finishAlignStableMs += APP_CAR_CONTROL_PERIOD_MS;
        }
        if (pCar->finishAlignStableMs >= APP_CAR_H2_FINISH_ALIGN_STABLE_MS) {
            _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
        }
        return;
    }

    pCar->finishAlignStableMs = 0U;
    turnSpeed = APP_CAR_H2_FINISH_ALIGN_SPEED;
    if (((errorCd > 0) && (APP_CAR_H2_FINISH_ALIGN_DIRECTION > 0)) ||
        ((errorCd < 0) && (APP_CAR_H2_FINISH_ALIGN_DIRECTION < 0))) {
        turnSpeed = (int16_t)-turnSpeed;
    }

    pCar->leftCommand = (int16_t)-turnSpeed;
    pCar->rightCommand = turnSpeed;
    BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
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
    int32_t ballErrorMm;

    if (pCar->mode != APP_CAR_MODE_BALANCE_AB) {
        BspServo_Center();
    }

    if (pCar->ballValid == 0U) {
        pCar->ballStableMs = 0U;
        return;
    }

    if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
        pCar->ballTargetMm = pCar->ballOffsetMm;
        _SetBallState(pCar, APP_CAR_BALL_MOVE_POSITIVE);
    } else {
        if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
            pCar->ballTargetMm = APP_CAR_BALL_CENTER_MM;
            _RunBallControl(pCar, APP_CAR_BALL_CENTER_MM);
            ballErrorMm = _AbsI32((int32_t)pCar->ballOffsetMm);
            if (ballErrorMm > APP_CAR_H4_START_READY_ERROR_MM) {
                pCar->ballStableMs = 0U;
                return;
            }
            if (pCar->ballStableMs < APP_CAR_H4_START_READY_MS) {
                pCar->ballStableMs += APP_CAR_CONTROL_PERIOD_MS;
                return;
            }
        }
        _SetRouteState(pCar, APP_CAR_ROUTE_LEAVE_START);
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
    }
}

static void _BallMovePositive(AppCarDef *pCar)
{
    int16_t targetMm = _StepBallTargetMm(pCar, APP_CAR_BALL_POSITIVE_CONTROL_MM);
    _RunBallControl(pCar, targetMm);

    if ((targetMm == APP_CAR_BALL_POSITIVE_CONTROL_MM) &&
        (_IsBallReached(pCar, APP_CAR_BALL_POSITIVE_MM) != 0U)) {
        _PrintBallStageResult(pCar, "+50", APP_CAR_BALL_POSITIVE_MM);
        _SetBallState(pCar, APP_CAR_BALL_MOVE_NEGATIVE);
    }
}

static void _BallMoveNegative(AppCarDef *pCar)
{
    int16_t targetMm = _StepBallTargetMm(pCar, APP_CAR_BALL_NEGATIVE_CONTROL_MM);
    _RunBallControl(pCar, targetMm);

    if ((targetMm == APP_CAR_BALL_NEGATIVE_CONTROL_MM) &&
        (_IsBallFinalStable(pCar, APP_CAR_BALL_NEGATIVE_MM) != 0U)) {
        _PrintBallStageResult(pCar, "-50", APP_CAR_BALL_NEGATIVE_MM);
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
        _EnterFinished(pCar);
    }
}

static void _BallHoldTarget(AppCarDef *pCar)
{
    _RunBallControl(pCar, _GetBallHoldTargetMm(pCar));
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
    _ResetImuLap(pCar);
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
    pCar->finishAlignStableMs = 0U;

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
        case APP_CAR_ROUTE_FINISH_BRAKE:
            pCar->pRouteState = _RouteFinishBrake;
            break;
        case APP_CAR_ROUTE_FINISH_ALIGN:
            pCar->pRouteState = _RouteFinishAlign;
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
    pCar->ballStableFrameSeq = pCar->ballFrameSeq;

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
    BspJy61pData_t imu;

    BspEncoder_GetData(&encoder);
    BspK230_GetBall(&ball);
    BspJy61p_GetData(&imu);

    pCar->gray = BspGray_GetFiltered();
    pCar->leftSpeed = encoder.leftPps;
    pCar->rightSpeed = encoder.rightPps;
    pCar->leftCount = encoder.leftCount;
    pCar->rightCount = encoder.rightCount;
    pCar->routePulses =
        (_AbsCount(encoder.leftCount) + _AbsCount(encoder.rightCount)) / 2U;
    pCar->ballOffsetMm = ball.offsetMm;
    pCar->ballFrameSeq = ball.frameSeq;
    pCar->ballValid = ball.valid;
    pCar->imuYawCd = imu.yawCd;
    pCar->imuValid = imu.valid;
    pCar->imuOnline = imu.online;
    _UpdateImuLap(pCar, &imu);
}

static void _RunTraceControl(AppCarDef *pCar)
{
    TraceControl_SetBaseSpeed(&pCar->trace, _GetCurrentTraceSpeed(pCar));
    TraceControl_Update(&pCar->trace, pCar->gray);

    pCar->leftCommand = pCar->trace.leftCommand;
    pCar->rightCommand = pCar->trace.rightCommand;
    pCar->traceTurn = pCar->trace.lastTurn;
    pCar->traceState = (uint8_t)pCar->trace.state;

    BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
}

static void _RunBallControl(AppCarDef *pCar, int16_t targetMm)
{
    uint16_t pulseUs;

    int16_t ffMmPerSec = 0;

    /* 目标与上一帧相同 = 爬坡已结束，允许刹车；
     * 爬坡途中误差小是"跟得住"，不是"要到站"，此时刹车会拖慢跟随 */
    if (targetMm == pCar->lastBallTargetMm) {
        BallControl_SetTargetSettled(&pCar->ballControl, 1U);
    } else {
        BallControl_SetTargetSettled(&pCar->ballControl, 0U);
        /* 爬坡中：把斜坡速度前馈给控制器。
         * 斜坡是 STEP_MM 每 PERIOD_MS，方向由目标变化的符号决定 */
        ffMmPerSec = (int16_t)(((int32_t)APP_CAR_BALL_TARGET_POS_STEP_MM *
            1000L) / (int32_t)APP_CAR_BALL_TARGET_POS_PERIOD_MS);
        if (targetMm < pCar->lastBallTargetMm) {
            ffMmPerSec = (int16_t)-ffMmPerSec;
        }
    }
    BallControl_SetFeedforward(&pCar->ballControl, ffMmPerSec);
    pCar->lastBallTargetMm = targetMm;

    pulseUs = BallControl_Update(&pCar->ballControl,
        targetMm, pCar->ballOffsetMm, pCar->ballFrameSeq,
        pCar->uptimeMs, pCar->ballValid);

    BspServo_SetPulseUs(pulseUs);
}

static int16_t _GetCurrentTraceSpeed(const AppCarDef *pCar)
{
    int16_t targetSpeed;
    int16_t startSpeed;
    uint32_t rampMs;
    int32_t speed;

    targetSpeed = _GetTraceSpeed(pCar->mode);
    if ((pCar->mode != APP_CAR_MODE_BALANCE_AB) ||
        (pCar->fatherState != APP_CAR_FATHER_RUNNING)) {
        return targetSpeed;
    }

    startSpeed = APP_CAR_H4_RAMP_START_SPEED;
    if (startSpeed > targetSpeed) {
        startSpeed = targetSpeed;
    }

    rampMs = APP_CAR_H4_RAMP_MS;
    if ((rampMs == 0U) || (pCar->routeStateMs >= rampMs)) {
        return targetSpeed;
    }

    speed = (int32_t)startSpeed +
        (((int32_t)(targetSpeed - startSpeed) * (int32_t)pCar->routeStateMs) /
         (int32_t)rampMs);
    return (int16_t)speed;
}

static int16_t _GetBallHoldTargetMm(const AppCarDef *pCar)
{
    int32_t targetMm = pCar->ballTargetMm;
    int32_t feedforwardMm = APP_CAR_H4_START_FF_MM;
    uint32_t tMs;
    uint32_t fadeMs;
    uint32_t holdMs;

    if ((pCar->mode != APP_CAR_MODE_BALANCE_AB) ||
        (pCar->fatherState != APP_CAR_FATHER_RUNNING)) {
        return _ClampBallTargetMm((int16_t)targetMm);
    }

    if ((pCar->routeState != APP_CAR_ROUTE_LEAVE_START) &&
        (pCar->routeState != APP_CAR_ROUTE_TRACKING)) {
        return _ClampBallTargetMm((int16_t)targetMm);
    }

    holdMs = APP_CAR_H4_START_FF_HOLD_MS;
    fadeMs = APP_CAR_H4_START_FF_FADE_MS;
    tMs = pCar->routeStateMs;

    if (tMs < holdMs) {
        targetMm += feedforwardMm;
    } else if ((fadeMs != 0U) && (tMs < (holdMs + fadeMs))) {
        targetMm += (feedforwardMm * (int32_t)((holdMs + fadeMs) - tMs)) /
            (int32_t)fadeMs;
    }

    return _ClampBallTargetMm((int16_t)targetMm);
}

static int16_t _ClampBallTargetMm(int16_t targetMm)
{
    if (targetMm < APP_CAR_TARGET_MIN_MM) {
        return APP_CAR_TARGET_MIN_MM;
    }
    if (targetMm > APP_CAR_TARGET_MAX_MM) {
        return APP_CAR_TARGET_MAX_MM;
    }
    return targetMm;
}

static uint16_t _GetFinishFollowMs(AppCarMode_t mode)
{
    if (mode == APP_CAR_MODE_BALANCE_AB) {
        return APP_CAR_H4_FINISH_FOLLOW_MS;
    }

    return APP_CAR_FINISH_FOLLOW_MS;
}

static int16_t _StepBallTargetMm(AppCarDef *pCar, int16_t finalTargetMm)
{
    int16_t targetMm = pCar->ballTargetMm;
    int16_t stepMm;
    uint16_t periodMs;

    if (targetMm == finalTargetMm) {
        return targetMm;
    }

    if (targetMm < finalTargetMm) {
        stepMm = APP_CAR_BALL_TARGET_POS_STEP_MM;
        periodMs = APP_CAR_BALL_TARGET_POS_PERIOD_MS;
    } else {
        stepMm = APP_CAR_BALL_TARGET_NEG_STEP_MM;
        periodMs = APP_CAR_BALL_TARGET_NEG_PERIOD_MS;
    }

    if ((pCar->ballStateMs % periodMs) != 0U) {
        return targetMm;
    }

    if (targetMm < finalTargetMm) {
        targetMm = (int16_t)(targetMm + stepMm);
        if (targetMm > finalTargetMm) {
            targetMm = finalTargetMm;
        }
    } else if (targetMm > finalTargetMm) {
        targetMm = (int16_t)(targetMm - stepMm);
        if (targetMm < finalTargetMm) {
            targetMm = finalTargetMm;
        }
    }

    pCar->ballTargetMm = targetMm;
    return targetMm;
}

static uint8_t _IsBallReached(const AppCarDef *pCar, int16_t targetMm)
{
    int16_t errorMm;
    int16_t speedMmPerSec;

    if ((pCar == 0) || (pCar->ballValid == 0U)) {
        return 0U;
    }

    errorMm = (int16_t)(targetMm - pCar->ballOffsetMm);
    if (errorMm < 0) {
        errorMm = (int16_t)-errorMm;
    }

    speedMmPerSec = BallControl_GetSpeedMmPerSec(&pCar->ballControl);
    if (speedMmPerSec < 0) {
        speedMmPerSec = (int16_t)-speedMmPerSec;
    }

    return (uint8_t)((errorMm <= APP_CAR_BALL_REACH_ERROR_MM) &&
        (speedMmPerSec <= APP_CAR_BALL_REACH_SPEED_MM_PER_S));
}

static uint8_t _IsBallFinalStable(AppCarDef *pCar, int16_t targetMm)
{
    int16_t errorMm;
    int16_t speedMmPerSec;

    if ((pCar == 0) || (pCar->ballValid == 0U)) {
        pCar->ballStableMs = 0U;
        return 0U;
    }

    if (pCar->ballFrameSeq == pCar->ballStableFrameSeq) {
        return (uint8_t)(pCar->ballStableMs >= APP_CAR_BALL_FINAL_STABLE_MS);
    }
    pCar->ballStableFrameSeq = pCar->ballFrameSeq;

    errorMm = (int16_t)(targetMm - pCar->ballOffsetMm);
    if (errorMm < 0) {
        errorMm = (int16_t)-errorMm;
    }

    speedMmPerSec = BallControl_GetSpeedMmPerSec(&pCar->ballControl);
    if (speedMmPerSec < 0) {
        speedMmPerSec = (int16_t)-speedMmPerSec;
    }

    if ((errorMm <= APP_CAR_BALL_FINAL_ERROR_MM) &&
        (speedMmPerSec <= APP_CAR_BALL_FINAL_SPEED_MM_PER_S)) {
        if (pCar->ballStableMs < APP_CAR_BALL_FINAL_STABLE_MS) {
            pCar->ballStableMs += APP_CAR_BALL_FRAME_MS;
        }
    } else {
        pCar->ballStableMs = 0U;
    }

    return (uint8_t)(pCar->ballStableMs >= APP_CAR_BALL_FINAL_STABLE_MS);
}

static void _PrintBallStageResult(const AppCarDef *pCar, const char *name,
    int16_t targetMm)
{
    int16_t errorMm = (int16_t)(targetMm - pCar->ballOffsetMm);
    int16_t speedMmPerSec = BallControl_GetSpeedMmPerSec(&pCar->ballControl);

    BspUart_Printf("[H3] %s ok t=%lu ms stage=%lu ms target=%d ball=%d err=%d speed=%d\n",
        name,
        (unsigned long)pCar->elapsedMs,
        (unsigned long)pCar->ballStateMs,
        (int)targetMm,
        (int)pCar->ballOffsetMm,
        (int)errorMm,
        (int)speedMmPerSec);
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

static int32_t _AbsI32(int32_t value)
{
    if (value >= 0) {
        return value;
    }

    return -value;
}

static int16_t _AngleDeltaCd(int16_t nowCd, int16_t lastCd)
{
    int32_t delta = (int32_t)nowCd - (int32_t)lastCd;

    if (delta > 18000L) {
        delta -= 36000L;
    } else if (delta < -18000L) {
        delta += 36000L;
    }

    return (int16_t)delta;
}

static int32_t _H2HeadingErrorCd(const AppCarDef *pCar)
{
    int32_t errorCd = pCar->imuLapYawCd;

    while (errorCd > 18000L) {
        errorCd -= 36000L;
    }
    while (errorCd < -18000L) {
        errorCd += 36000L;
    }

    return errorCd;
}

static void _ResetImuLap(AppCarDef *pCar)
{
    pCar->imuLapYawCd = 0;
    pCar->imuYawCd = 0;
    pCar->imuLastYawCd = 0;
    pCar->imuSampleSeq = 0U;
    pCar->imuValid = 0U;
    pCar->imuOnline = 0U;
    pCar->imuHasLastYaw = 0U;
}

static void _UpdateImuLap(AppCarDef *pCar, const BspJy61pData_t *pImu)
{
    int16_t deltaCd;

    if ((pImu == 0) || (pImu->valid == 0U)) {
        return;
    }

    if (pImu->sampleSeq == pCar->imuSampleSeq) {
        return;
    }
    pCar->imuSampleSeq = pImu->sampleSeq;

    if (pCar->imuHasLastYaw == 0U) {
        pCar->imuLastYawCd = pImu->yawCd;
        pCar->imuHasLastYaw = 1U;
        return;
    }

    deltaCd = _AngleDeltaCd(pImu->yawCd, pCar->imuLastYawCd);
    pCar->imuLastYawCd = pImu->yawCd;
    pCar->imuLapYawCd += deltaCd;
}

static uint8_t _CanFinishByRouteGate(const AppCarDef *pCar)
{
    if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
        return 0U;
    }

    if (pCar->routePulses < APP_CAR_LAP_GATE_PULSES) {
        return 0U;
    }

    return 1U;
}

static void _PrintTelemetry(const AppCarDef *pCar)
{
    BspK230Debug_t k230Debug;

    BspK230_GetDebug(&k230Debug);
    BspUart_Printf(
        "[T] f=%u r=%u b=%u mode=%u time=%lu target=%dmm ball=%d/%u gray=%02X enc=%d,%d count=%ld,%ld lap=%lu iy=%ld/%d/%u/%u cmd=%d,%d tr=%d/%u q=%u k=%lu,%lu,%lu,%lu\n",
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
        (long)pCar->imuLapYawCd,
        (int)pCar->imuYawCd,
        (unsigned)pCar->imuValid,
        (unsigned)pCar->imuOnline,
        (int)pCar->leftCommand,
        (int)pCar->rightCommand,
        (int)pCar->traceTurn,
        (unsigned)pCar->traceState,
        (unsigned)MsgMap_GetOverflowCount(),
        (unsigned long)k230Debug.rxBytes,
        (unsigned long)k230Debug.pollBytes,
        (unsigned long)k230Debug.lines,
        (unsigned long)k230Debug.parsed);

    /* H3 控制器诊断：dt 是视觉帧真实间隔，speed 死了刹车就是摆设 */
    if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
        BspUart_Printf(
            "[C] dt=%u spd=%d tspd=%d out=%d i=%d set=%u rej=%u err=%d\n",
            (unsigned)BallControl_GetDtMs(&pCar->ballControl),
            (int)BallControl_GetSpeedMmPerSec(&pCar->ballControl),
            (int)pCar->ballControl.targetSpeedMmPerSec,
            (int)BallControl_GetOutputUs(&pCar->ballControl),
            (int)BallControl_GetIntegralUs(&pCar->ballControl),
            (unsigned)pCar->ballControl.targetSettled,
            (unsigned)BallControl_GetRejectCount(&pCar->ballControl),
            (int)(pCar->ballTargetMm - pCar->ballOffsetMm));
    }
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
