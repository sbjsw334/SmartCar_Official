#include "app_car.h"

#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_jy61p.h"
#include "bsp_k230.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

/* Track speeds: tune these values at the field. */
#define APP_CAR_TRACE_SPEED_H2           (36)       /* H2 normal 8-sensor trace speed */
#define APP_CAR_TRACE_SPEED_H4           (24)       /* H4 A->B稳球速度：先用24，车稳后再试26 */
#define APP_CAR_TRACE_SPEED_H5           (28)       /* H5 speed */

/* H2/H5 lap protection. 13 PPR * 28 gear ratio * 4 edges. */
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

/* H2 finish: trace -> finish line confirm -> gyro-straight -> brake -> JY61P align.
 * Normal H2 tracing does not depend on the gyro. After detecting A again,
 * lock the current gyro heading and drive straight before aligning back to start.
 */
#define APP_CAR_H2_FINISH_FOLLOW_MS      (500U)     /* H2: after finish line, lock current gyro yaw and drive straight */
#define APP_CAR_H2_FINISH_BRAKE_MS       (80U)      /* H2: reverse brake time */
#define APP_CAR_H2_FINISH_BRAKE_SPEED    (20)       /* H2: reverse brake strength */
#define APP_CAR_H2_FINISH_ALIGN_ENABLE   (1U)       /* 第二题：停车后是否用JY61P回到起始角度，0=关闭 */
#define APP_CAR_H2_FINISH_ALIGN_ERROR_CD (100L)     /* H2: final align tolerance, 100 = 1 degree */
#define APP_CAR_H2_FINISH_ALIGN_STABLE_MS (100U)    /* 第二题：回正到位后保持时间 */
#define APP_CAR_H2_FINISH_ALIGN_TIMEOUT_MS (1000U)   /* H2: final align timeout */
#define APP_CAR_H2_FINISH_ALIGN_SPEED    (13)       /* H2: base rotate speed */
#define APP_CAR_H2_FINISH_ALIGN_EXTRA_SPEED (1)     /* H2: dither 16/17 to get about 16.5 */
#define APP_CAR_H2_FINISH_ALIGN_DITHER_MS (20U)     /* H2: align speed dither period */
#define APP_CAR_H2_FINISH_ALIGN_DIRECTION (1)       /* 第二题：回正方向补偿，若越转越偏改成-1 */
#define APP_CAR_H2_FINISH_FORWARD_SPEED  (APP_CAR_TRACE_SPEED_H2)
#define APP_CAR_H2_FINISH_FORWARD_KP_NUM (1)
#define APP_CAR_H2_FINISH_FORWARD_KP_DEN (50)
#define APP_CAR_H2_FINISH_FORWARD_LIMIT  (12)
#define APP_CAR_H2_FINISH_FORWARD_DIRECTION (1)

/* H4 parameters: A -> B with ball held near O.
 * H4调参只优先改下面这些值：
 * 1) APP_CAR_TRACE_SPEED_H4：A到B循迹速度，先用24求稳；球稳后可试26。
 * 2) H4_RAMP_START_SPEED / H4_RAMP_MS：起步速度和缓启动时间，RAMP_MS越大越稳但越慢。
 * 3) H4_START_READY_ERROR_MM / H4_START_READY_MS：START后球在中心附近稳定多久才允许发车。
 * 4) H4_START_FF_MM：起步球位前馈，默认0；若起步球总往同一边甩，再在-15~+15mm内小步改。
 * 5) A线忽略：H4刚起步遇到4路以上黑线连续30ms，只认为通过A线，继续循迹，不停表。
 * 6) H4_AB_LENGTH_MM：题目AB直线长度；H4仅用编码器判断通过B，B点没有横向黑线。
 * 7) H4_FINISH_CRUISE_MS：通过B并停表后继续正常循迹时间，不计入H4时间。
 * 8) H4_FINISH_DECEL_MS：随后保持循迹方向，将左右电机输出平滑降到0，减小钢球惯性。
 */
#define APP_CAR_H4_AB_LENGTH_MM           (1500U)
#define APP_CAR_H4_AB_EXPECTED_PULSES     \
    ((APP_CAR_H4_AB_LENGTH_MM * APP_CAR_ENCODER_COUNTS_PER_REV) / \
     APP_CAR_WHEEL_CIRCUMFERENCE_MM)
#define APP_CAR_H4_RAMP_START_SPEED       (10)
#define APP_CAR_H4_RAMP_MS                (2200U)
#define APP_CAR_H4_START_READY_ERROR_MM   (10)
#define APP_CAR_H4_START_READY_MS         (200U)
#define APP_CAR_H4_START_FF_MM            (0)
#define APP_CAR_H4_START_FF_HOLD_MS       (500U)
#define APP_CAR_H4_START_FF_FADE_MS       (700U)
#define APP_CAR_H4_FINISH_CRUISE_MS       (2000U)
#define APP_CAR_H4_FINISH_DECEL_MS        (2200U)

/* H4 JY61P Y轴加速度前馈：视觉仍为主闭环，只提前补偿起步/转弯/减速惯性。 */
#define APP_CAR_H4_ACC_FF_ENABLE           (1U)
#define APP_CAR_H4_ACC_FF_DIRECTION        (1)
#define APP_CAR_H4_ACC_FF_FILTER_DEN       (4)
#define APP_CAR_H4_ACC_FF_DEAD_MG          (12)
#define APP_CAR_H4_ACC_FF_GAIN_NUM         (1)
#define APP_CAR_H4_ACC_FF_GAIN_DEN         (5)
#define APP_CAR_H4_ACC_FF_LIMIT_MM         (6)
#define APP_CAR_H4_ACC_FF_REJECT_MG        (250)
#define APP_CAR_H4_ACC_FF_STEP_MM          (1)

/* H5 parameters: one clockwise lap with ball held at O.
 * 第五题只优先调下面这些值：
 * 1) APP_CAR_TRACE_SPEED_H5：整圈循迹速度，先用28；若球晃先降到24~26，稳定后再提速。
 * 2) H5_RAMP_START_SPEED / H5_RAMP_MS：起步缓启动，起步球甩就增大RAMP_MS。
 * 3) H5_START_READY_ERROR_MM / H5_START_READY_MS：START后球在中心附近稳定多久才发车。
 * 4) H5_HOLD_BIAS_MM：H5行驶中的球位目标偏置；球总往负方向偏，就把它调成正值。
 * 5) H5_START_FF_MM：起步前馈；只在起步前1.2s左右叠加，抵消起步瞬间偏移。
 * 6) H5_ACC_FF_*：用编码器速度变化估算车体加减速，只小幅修正球目标。
 * 7) H5_LAP_GATE_PERCENT：一圈到80%以后才允许A横线作为终点，避免刚起步误判A线。
 * 8) H5_FINISH_FOLLOW_MS：识别A线后停表，再继续循迹一点用于过线/柔停，不计入成绩时间。
 */
#define APP_CAR_H5_RAMP_START_SPEED       (18)
#define APP_CAR_H5_RAMP_MS                (1200U)
#define APP_CAR_H5_START_READY_ERROR_MM   (10)
#define APP_CAR_H5_START_READY_MS         (200U)
#define APP_CAR_H5_HOLD_BIAS_MM           (6)
#define APP_CAR_H5_HOLD_BIAS_MIN_MM       (-30)
#define APP_CAR_H5_HOLD_BIAS_MAX_MM       (30)
#define APP_CAR_H5_START_FF_MM            (8)
#define APP_CAR_H5_START_FF_HOLD_MS       (500U)
#define APP_CAR_H5_START_FF_FADE_MS       (700U)
#define APP_CAR_H5_ACC_FF_ENABLE          (1U)
#define APP_CAR_H5_ACC_FF_DIRECTION       (1)
#define APP_CAR_H5_ACC_FF_LIMIT_MM        (10)
#define APP_CAR_H5_ACC_FF_DEAD_PPS        (2)
#define APP_CAR_H5_ACC_FF_GAIN_NUM        (1)
#define APP_CAR_H5_ACC_FF_GAIN_DEN        (4)
#define APP_CAR_H5_SPEED_FILTER_DEN       (8)
#define APP_CAR_H5_LAP_GATE_PERCENT       (80U)
#define APP_CAR_H5_LAP_GATE_PULSES        \
    ((APP_CAR_LAP_EXPECTED_PULSES * APP_CAR_H5_LAP_GATE_PERCENT) / 100U)
#define APP_CAR_H5_FINISH_FOLLOW_MS       (400U)

#define APP_CAR_TARGET_MIN_MM             (-100)
#define APP_CAR_TARGET_MAX_MM             (100)
#define APP_CAR_BALL_CENTER_MM            (0)
#define APP_CAR_BALL_POSITIVE_MM          (50)
#define APP_CAR_BALL_POSITIVE_HOLD_BIAS_MM (6)
#define APP_CAR_BALL_POSITIVE_HOLD_MM     (APP_CAR_BALL_POSITIVE_MM + APP_CAR_BALL_POSITIVE_HOLD_BIAS_MM)
#define APP_CAR_BALL_NEGATIVE_MM          (-50)
#define APP_CAR_BALL_POSITIVE_CONTROL_MM  (50)
#define APP_CAR_BALL_NEGATIVE_CONTROL_MM  (-50)
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
#define APP_CAR_H3_TIME_LIMIT_MS          (5000U)
#define APP_CAR_H3_FINISH_ERROR_MM        (10)

static void _Init(AppCarDef *pCar);
static void _Run(AppCarDef *pCar, MsgId_t msg);
static void _SetMode(AppCarDef *pCar, AppCarMode_t mode);
static void _SetBallTargetMm(AppCarDef *pCar, int16_t targetMm);
static void _AdjustH5HoldBiasMm(AppCarDef *pCar, int16_t deltaMm);
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
static void _ResetH4AccelFeedforward(AppCarDef *pCar);
static void _UpdateH4AccelFeedforward(AppCarDef *pCar,
    const BspJy61pData_t *pImu);
static void _UpdateH5AccelFeedforward(AppCarDef *pCar);
static void _RunTraceControl(AppCarDef *pCar);
static void _RunH4FinishDecel(AppCarDef *pCar, uint32_t decelElapsedMs);
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
static void _LockH2FinishForwardYaw(AppCarDef *pCar);
static void _RunH2FinishForward(AppCarDef *pCar);
static int32_t _H2FinishForwardErrorCd(const AppCarDef *pCar);
static int32_t _H2HeadingErrorCd(const AppCarDef *pCar);
static void _ResetImuLap(AppCarDef *pCar);
static void _UpdateImuLap(AppCarDef *pCar, const BspJy61pData_t *pImu);
static uint8_t _CanFinishByRouteGate(const AppCarDef *pCar);
static void _PrintTelemetry(const AppCarDef *pCar);
static void _PrintH4Telemetry(const AppCarDef *pCar);
static int16_t _GetTraceSpeed(AppCarMode_t mode);

AppCarDef appCarMain;

AppCarConDef appCarCon = {
    .init = _Init,
    .run = _Run,
    .setMode = _SetMode,
    .setBallTargetMm = _SetBallTargetMm,
    .adjustH5HoldBiasMm = _AdjustH5HoldBiasMm,
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
    pCar->h2FinishForwardYawCd = 0;
    pCar->imuYawCd = 0;
    pCar->imuLastYawCd = 0;
    pCar->imuSampleSeq = 0U;
    pCar->h4AccSampleSeq = 0U;
    pCar->h4AccBiasMg = 0;
    pCar->h4AccFiltMg = 0;
    pCar->h4AccFfMm = 0;
    pCar->h4AccRejectCount = 0U;
    pCar->imuValid = 0U;
    pCar->imuOnline = 0U;
    pCar->imuHasLastYaw = 0U;
    pCar->h4AccBiasValid = 0U;
    pCar->h2FinishForwardYawValid = 0U;
    pCar->ballStableMs = 0U;
    pCar->ballStableFrameSeq = 0U;
    pCar->ballTargetMm = 0;
    pCar->ballOffsetMm = 0;
    pCar->ballFrameSeq = 0U;
    pCar->leftSpeed = 0;
    pCar->rightSpeed = 0;
    pCar->h5HoldBiasMm = APP_CAR_H5_HOLD_BIAS_MM;
    pCar->h5SpeedFiltPps = 0;
    pCar->h5LastSpeedFiltPps = 0;
    pCar->h5AccelFfMm = 0;
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
    BspUart_Printf("[H] 2..5=mode, s=start, x=stop\n");
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
        (mode > APP_CAR_MODE_BALANCE_LAP_CENTER) ||
        (pCar->fatherState == APP_CAR_FATHER_RUNNING)) {
        return;
    }

    pCar->mode = mode;
    if (mode == APP_CAR_MODE_BALL_STATIC) {
        pCar->ballTargetMm = APP_CAR_BALL_CENTER_MM;
        BallControl_Reset(&pCar->ballControl);
    } else if (mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
        pCar->ballTargetMm = pCar->h5HoldBiasMm;
        BspServo_Center();
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

static void _AdjustH5HoldBiasMm(AppCarDef *pCar, int16_t deltaMm)
{
    int16_t biasMm;

    if ((pCar == 0) || (pCar->mode != APP_CAR_MODE_BALANCE_LAP_CENTER)) {
        return;
    }

    biasMm = (int16_t)(pCar->h5HoldBiasMm + deltaMm);
    if (biasMm < APP_CAR_H5_HOLD_BIAS_MIN_MM) {
        biasMm = APP_CAR_H5_HOLD_BIAS_MIN_MM;
    } else if (biasMm > APP_CAR_H5_HOLD_BIAS_MAX_MM) {
        biasMm = APP_CAR_H5_HOLD_BIAS_MAX_MM;
    }

    pCar->h5HoldBiasMm = biasMm;
    pCar->ballTargetMm = biasMm;
    BspUart_Printf("[H5T] bias=%d ball=%d af=%d\n",
        (int)pCar->h5HoldBiasMm,
        (int)pCar->ballOffsetMm,
        (int)pCar->h5AccelFfMm);
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
    } else if (msg == MSG_TELEMETRY_200MS) {
        if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
            _PrintTelemetry(pCar);
        }
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
        } else if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
            _PrintH4Telemetry(pCar);
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
        if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
            _PrintTelemetry(pCar);
        }
    }
}

static void _FatherFault(AppCarDef *pCar, MsgId_t msg)
{
    if ((msg == MSG_STOP) || (msg == MSG_KEY_START)) {
        _EnterStopped(pCar);
    } else if (msg == MSG_CONTROL_TICK) {
        _SampleInputs(pCar);
    } else if (msg == MSG_TELEMETRY_200MS) {
        if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
            _PrintTelemetry(pCar);
        }
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
        if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) ||
            (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER)) {
            if (pCar->finishLineMs < APP_CAR_FINISH_CONFIRM_MS) {
                pCar->finishLineMs += APP_CAR_CONTROL_PERIOD_MS;
            }
            if (pCar->finishLineMs >= APP_CAR_FINISH_CONFIRM_MS) {
                _SetRouteState(pCar, APP_CAR_ROUTE_TRACKING);
                _RunTraceControl(pCar);
                return;
            }
        }
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
    if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) &&
        (pCar->routePulses >= APP_CAR_H4_AB_EXPECTED_PULSES)) {
        /* 题目B点没有横向黑线：编码器达到1.5m即视为通过B并立即停表。 */
        pCar->timerRunning = 0U;
        _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
        pCar->pRouteState(pCar);
        return;
    }

    if (_CanFinishByRouteGate(pCar) &&
        (_IsFinishLine(pCar->gray) != 0U)) {
        if (pCar->finishLineMs < APP_CAR_FINISH_CONFIRM_MS) {
            pCar->finishLineMs += APP_CAR_CONTROL_PERIOD_MS;
        }
        if (pCar->finishLineMs >= APP_CAR_FINISH_CONFIRM_MS) {
            if (pCar->mode == APP_CAR_MODE_TRACE_ONLY) {
                _LockH2FinishForwardYaw(pCar);
                _SetRouteState(pCar, APP_CAR_ROUTE_FINISH_ACTION);
            } else {
                if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) ||
                    (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER)) {
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
        if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) &&
            (pCar->routeStateMs >= APP_CAR_H4_FINISH_CRUISE_MS)) {
            _RunH4FinishDecel(pCar,
                pCar->routeStateMs - APP_CAR_H4_FINISH_CRUISE_MS);
        } else if (pCar->mode == APP_CAR_MODE_TRACE_ONLY) {
            _RunH2FinishForward(pCar);
        } else {
            _RunTraceControl(pCar);
        }
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
        (APP_CAR_H2_FINISH_ALIGN_ENABLE != 0U)) {
        BspUart_Printf("[H2] align start iy=%ld online=%u has=%u\n",
            (long)pCar->imuLapYawCd,
            (unsigned)pCar->imuOnline,
            (unsigned)pCar->imuHasLastYaw);
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
        BspUart_Printf("[H2] align skip/timeout iy=%ld online=%u has=%u\n",
            (long)pCar->imuLapYawCd,
            (unsigned)pCar->imuOnline,
            (unsigned)pCar->imuHasLastYaw);
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
            BspUart_Printf("[H2] align ok err=%ld iy=%ld\n",
                (long)errorCd,
                (long)pCar->imuLapYawCd);
            _SetRouteState(pCar, APP_CAR_ROUTE_COMPLETE);
        }
        return;
    }

    pCar->finishAlignStableMs = 0U;
    turnSpeed = APP_CAR_H2_FINISH_ALIGN_SPEED;
    if ((APP_CAR_H2_FINISH_ALIGN_EXTRA_SPEED > 0) &&
        (APP_CAR_H2_FINISH_ALIGN_DITHER_MS != 0U) &&
        ((((pCar->routeStateMs / APP_CAR_H2_FINISH_ALIGN_DITHER_MS) & 0x01U) != 0U))) {
        turnSpeed = (int16_t)(turnSpeed + APP_CAR_H2_FINISH_ALIGN_EXTRA_SPEED);
    }
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
    int32_t startReadyErrorMm;
    uint16_t startReadyMs;
    uint16_t stableStepMs;
    int16_t holdTargetMm;

    if ((pCar->mode != APP_CAR_MODE_BALANCE_AB) &&
        (pCar->mode != APP_CAR_MODE_BALANCE_LAP_CENTER)) {
        BspServo_Center();
    }

    if (pCar->ballValid == 0U) {
        pCar->ballStableMs = 0U;
        if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
            pCar->ballStableFrameSeq = pCar->ballFrameSeq;
        }
        return;
    }

    if (pCar->mode == APP_CAR_MODE_BALL_STATIC) {
        pCar->ballTargetMm = (int16_t)-APP_CAR_BALL_TARGET_NEG_STEP_MM;
        /* H3 tuning flow: skip 0 hold, first run to -50, then measure -50 -> +50. */
        _SetBallState(pCar, APP_CAR_BALL_MOVE_NEGATIVE);
    } else {
        if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) ||
            (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER)) {
            if (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
                startReadyErrorMm = APP_CAR_H5_START_READY_ERROR_MM;
                startReadyMs = APP_CAR_H5_START_READY_MS;
                holdTargetMm = pCar->h5HoldBiasMm;
            } else {
                startReadyErrorMm = APP_CAR_H4_START_READY_ERROR_MM;
                startReadyMs = APP_CAR_H4_START_READY_MS;
                holdTargetMm = APP_CAR_BALL_CENTER_MM;
            }
            pCar->ballTargetMm = holdTargetMm;
            _RunBallControl(pCar, holdTargetMm);

            if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
                /* H4只按K230新帧累计稳定时间，避免同一帧被2ms控制周期重复计算。 */
                if (pCar->ballFrameSeq == pCar->ballStableFrameSeq) {
                    return;
                }
                pCar->ballStableFrameSeq = pCar->ballFrameSeq;
                stableStepMs = APP_CAR_BALL_FRAME_MS;
            } else {
                stableStepMs = APP_CAR_CONTROL_PERIOD_MS;
            }

            ballErrorMm = _AbsI32((int32_t)pCar->ballOffsetMm);
            if (ballErrorMm > startReadyErrorMm) {
                pCar->ballStableMs = 0U;
                return;
            }
            if (pCar->ballStableMs < startReadyMs) {
                pCar->ballStableMs += stableStepMs;
                if (pCar->ballStableMs < startReadyMs) {
                    return;
                }
            }
        }
        _SetRouteState(pCar, APP_CAR_ROUTE_LEAVE_START);
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
    }
}

static void _BallMovePositive(AppCarDef *pCar)
{
    int16_t targetMm = _StepBallTargetMm(pCar, APP_CAR_BALL_POSITIVE_CONTROL_MM);
    int16_t finishErrorMm;

    _RunBallControl(pCar, targetMm);

    if (targetMm != APP_CAR_BALL_POSITIVE_CONTROL_MM) {
        return;
    }

    if (pCar->ballValid == 0U) {
        return;
    }

    finishErrorMm = (int16_t)(APP_CAR_BALL_POSITIVE_MM - pCar->ballOffsetMm);
    if (finishErrorMm < 0) {
        finishErrorMm = (int16_t)-finishErrorMm;
    }

    if ((pCar->mode == APP_CAR_MODE_BALL_STATIC) &&
        (pCar->timerRunning != 0U) &&
        (finishErrorMm <= APP_CAR_H3_FINISH_ERROR_MM)) {
        pCar->timerRunning = 0U;
        BspUart_Printf("[H3] competition finish t=%lu ms limit=%lu ms %s\n",
            (unsigned long)pCar->elapsedMs,
            (unsigned long)APP_CAR_H3_TIME_LIMIT_MS,
            (pCar->elapsedMs <= APP_CAR_H3_TIME_LIMIT_MS) ? "OK" : "OVER");
    }

    if (_IsBallFinalStable(pCar, APP_CAR_BALL_POSITIVE_MM) != 0U) {
        _PrintBallStageResult(pCar, "+50", APP_CAR_BALL_POSITIVE_MM);
        pCar->ballTargetMm = APP_CAR_BALL_POSITIVE_HOLD_MM;
        _SetBallState(pCar, APP_CAR_BALL_HOLD_TARGET);
        _EnterFinished(pCar);
    }
}

static void _BallMoveNegative(AppCarDef *pCar)
{
    int16_t targetMm = _StepBallTargetMm(pCar, APP_CAR_BALL_NEGATIVE_CONTROL_MM);
    int16_t errorMm;

    _RunBallControl(pCar, targetMm);

    if (targetMm != APP_CAR_BALL_NEGATIVE_CONTROL_MM) {
        return;
    }

    if (pCar->ballValid == 0U) {
        return;
    }

    errorMm = (int16_t)(APP_CAR_BALL_NEGATIVE_MM - pCar->ballOffsetMm);
    if (errorMm < 0) {
        errorMm = (int16_t)-errorMm;
    }

    if (errorMm <= APP_CAR_BALL_REACH_ERROR_MM) {
        _PrintBallStageResult(pCar, "-50", APP_CAR_BALL_NEGATIVE_MM);
        pCar->ballControl.brakeZoneActive = 0U;
        pCar->ballControl.brakeReverseActive = 0U;
        pCar->ballControl.brakeEntrySpeedMmPerSec = 0;
        pCar->ballControl.brakeDynamicReverseMmPerSec = 0;
        pCar->ballControl.brakeDynamicGainPercent = 100;
        _SetBallState(pCar, APP_CAR_BALL_MOVE_POSITIVE);
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
    pCar->h5SpeedFiltPps = 0;
    pCar->h5LastSpeedFiltPps = 0;
    pCar->h5AccelFfMm = 0;
    pCar->h4AccFiltMg = 0;
    pCar->h4AccFfMm = 0;
    pCar->h4AccRejectCount = 0U;
    pCar->h4AccBiasValid = 0U;
    pCar->h2FinishForwardYawValid = 0U;
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
    pCar->h5SpeedFiltPps = 0;
    pCar->h5LastSpeedFiltPps = 0;
    pCar->h5AccelFfMm = 0;
    pCar->timerRunning = 1U;
    BspEncoder_Reset();
    BspMotor_Stop();
    BspServo_Center();
    TraceControl_Init(&pCar->trace);
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(pCar->mode));
    pCar->leftCommand = 0;
    pCar->rightCommand = 0;
    pCar->ballStableMs = 0U;
    pCar->h2FinishForwardYawValid = 0U;
    _ResetH4AccelFeedforward(pCar);
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
            if (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
                pCar->ballTargetMm = pCar->h5HoldBiasMm;
            } else {
                pCar->ballTargetMm = APP_CAR_BALL_CENTER_MM;
            }
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
    _UpdateH4AccelFeedforward(pCar, &imu);
    _UpdateH5AccelFeedforward(pCar);
}

static void _ResetH4AccelFeedforward(AppCarDef *pCar)
{
    BspJy61pData_t imu;

    if (pCar == 0) {
        return;
    }

    pCar->h4AccSampleSeq = 0U;
    pCar->h4AccBiasMg = 0;
    pCar->h4AccFiltMg = 0;
    pCar->h4AccFfMm = 0;
    pCar->h4AccRejectCount = 0U;
    pCar->h4AccBiasValid = 0U;

    if ((pCar->mode == APP_CAR_MODE_BALANCE_AB) &&
        (BspJy61p_GetData(&imu) != 0U)) {
        pCar->h4AccSampleSeq = imu.sampleSeq;
        pCar->h4AccBiasMg = imu.accYmg;
        pCar->h4AccBiasValid = 1U;
    }
}

static void _UpdateH4AccelFeedforward(AppCarDef *pCar,
    const BspJy61pData_t *pImu)
{
    int32_t dynamicAccMg;
    int32_t filteredAccMg;
    int32_t ffMm;
    int32_t currentFfMm;

    if ((pCar == 0) || (pImu == 0) ||
        (pCar->mode != APP_CAR_MODE_BALANCE_AB) ||
        (APP_CAR_H4_ACC_FF_ENABLE == 0U) || (pImu->valid == 0U)) {
        if (pCar != 0) {
            pCar->h4AccFfMm = 0;
        }
        return;
    }

    if (pImu->sampleSeq == pCar->h4AccSampleSeq) {
        return;
    }
    pCar->h4AccSampleSeq = pImu->sampleSeq;

    if (pCar->h4AccBiasValid == 0U) {
        pCar->h4AccBiasMg = pImu->accYmg;
        pCar->h4AccFiltMg = 0;
        pCar->h4AccFfMm = 0;
        pCar->h4AccBiasValid = 1U;
        return;
    }

    dynamicAccMg = (int32_t)pImu->accYmg - (int32_t)pCar->h4AccBiasMg;
    if (_AbsI32(dynamicAccMg) > APP_CAR_H4_ACC_FF_REJECT_MG) {
        if (pCar->h4AccRejectCount < 0xFFFFU) {
            pCar->h4AccRejectCount++;
        }
        return;
    }

    filteredAccMg =
        (((int32_t)pCar->h4AccFiltMg *
          (int32_t)(APP_CAR_H4_ACC_FF_FILTER_DEN - 1U)) + dynamicAccMg) /
        (int32_t)APP_CAR_H4_ACC_FF_FILTER_DEN;
    pCar->h4AccFiltMg = (int16_t)filteredAccMg;

    if (_AbsI32(filteredAccMg) <= APP_CAR_H4_ACC_FF_DEAD_MG) {
        ffMm = 0;
    } else {
        ffMm = (filteredAccMg * (int32_t)APP_CAR_H4_ACC_FF_GAIN_NUM) /
            (int32_t)APP_CAR_H4_ACC_FF_GAIN_DEN;
        ffMm *= (int32_t)APP_CAR_H4_ACC_FF_DIRECTION;
        if (ffMm > APP_CAR_H4_ACC_FF_LIMIT_MM) {
            ffMm = APP_CAR_H4_ACC_FF_LIMIT_MM;
        } else if (ffMm < -APP_CAR_H4_ACC_FF_LIMIT_MM) {
            ffMm = -APP_CAR_H4_ACC_FF_LIMIT_MM;
        }
    }

    currentFfMm = pCar->h4AccFfMm;
    if (ffMm > (currentFfMm + APP_CAR_H4_ACC_FF_STEP_MM)) {
        ffMm = currentFfMm + APP_CAR_H4_ACC_FF_STEP_MM;
    } else if (ffMm < (currentFfMm - APP_CAR_H4_ACC_FF_STEP_MM)) {
        ffMm = currentFfMm - APP_CAR_H4_ACC_FF_STEP_MM;
    }
    pCar->h4AccFfMm = (int16_t)ffMm;
}

static void _UpdateH5AccelFeedforward(AppCarDef *pCar)
{
    int32_t avgSpeedPps;
    int32_t speedFiltPps;
    int32_t accelPps;
    int32_t ffMm;

    if ((APP_CAR_H5_ACC_FF_ENABLE == 0U) ||
        (pCar->mode != APP_CAR_MODE_BALANCE_LAP_CENTER) ||
        (pCar->fatherState != APP_CAR_FATHER_RUNNING) ||
        ((pCar->routeState != APP_CAR_ROUTE_LEAVE_START) &&
         (pCar->routeState != APP_CAR_ROUTE_TRACKING))) {
        pCar->h5AccelFfMm = 0;
        return;
    }

    avgSpeedPps = ((int32_t)pCar->leftSpeed + (int32_t)pCar->rightSpeed) / 2L;
    speedFiltPps =
        (((int32_t)pCar->h5SpeedFiltPps *
          (int32_t)(APP_CAR_H5_SPEED_FILTER_DEN - 1U)) + avgSpeedPps) /
        (int32_t)APP_CAR_H5_SPEED_FILTER_DEN;
    accelPps = speedFiltPps - (int32_t)pCar->h5LastSpeedFiltPps;

    pCar->h5SpeedFiltPps = (int16_t)speedFiltPps;
    pCar->h5LastSpeedFiltPps = (int16_t)speedFiltPps;

    if (_AbsI32(accelPps) <= APP_CAR_H5_ACC_FF_DEAD_PPS) {
        pCar->h5AccelFfMm = 0;
        return;
    }

    ffMm = (accelPps * (int32_t)APP_CAR_H5_ACC_FF_GAIN_NUM) /
        (int32_t)APP_CAR_H5_ACC_FF_GAIN_DEN;
    ffMm *= (int32_t)APP_CAR_H5_ACC_FF_DIRECTION;

    if (ffMm > APP_CAR_H5_ACC_FF_LIMIT_MM) {
        ffMm = APP_CAR_H5_ACC_FF_LIMIT_MM;
    } else if (ffMm < -APP_CAR_H5_ACC_FF_LIMIT_MM) {
        ffMm = -APP_CAR_H5_ACC_FF_LIMIT_MM;
    }

    pCar->h5AccelFfMm = (int16_t)ffMm;
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

static void _RunH4FinishDecel(AppCarDef *pCar, uint32_t decelElapsedMs)
{
    uint32_t remainingMs;

    if ((pCar == 0) || (APP_CAR_H4_FINISH_DECEL_MS == 0U) ||
        (decelElapsedMs >= APP_CAR_H4_FINISH_DECEL_MS)) {
        BspMotor_Stop();
        if (pCar != 0) {
            pCar->leftCommand = 0;
            pCar->rightCommand = 0;
        }
        return;
    }

    /* 先计算正常循迹方向，再等比例缩小两轮输出，直到平滑降为0。 */
    TraceControl_SetBaseSpeed(&pCar->trace, _GetTraceSpeed(pCar->mode));
    TraceControl_Update(&pCar->trace, pCar->gray);
    remainingMs = APP_CAR_H4_FINISH_DECEL_MS - decelElapsedMs;

    pCar->leftCommand = (int16_t)(
        ((int32_t)pCar->trace.leftCommand * (int32_t)remainingMs) /
        (int32_t)APP_CAR_H4_FINISH_DECEL_MS);
    pCar->rightCommand = (int16_t)(
        ((int32_t)pCar->trace.rightCommand * (int32_t)remainingMs) /
        (int32_t)APP_CAR_H4_FINISH_DECEL_MS);
    pCar->traceTurn = pCar->trace.lastTurn;
    pCar->traceState = (uint8_t)pCar->trace.state;

    BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
}

static void _RunBallControl(AppCarDef *pCar, int16_t targetMm)
{
    uint16_t pulseUs = BallControl_Update(&pCar->ballControl,
        targetMm, pCar->ballOffsetMm, pCar->ballFrameSeq,
        pCar->uptimeMs, pCar->ballValid);

    BspServo_SetPulseUs(pulseUs);
}

static int16_t _GetCurrentTraceSpeed(const AppCarDef *pCar)
{
    int16_t targetSpeed;
    int16_t startSpeed;
    uint32_t rampMs;
    uint32_t rampElapsedMs;
    int32_t speed;

    targetSpeed = _GetTraceSpeed(pCar->mode);
    if (pCar->fatherState != APP_CAR_FATHER_RUNNING) {
        return targetSpeed;
    }

    if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
        startSpeed = APP_CAR_H4_RAMP_START_SPEED;
        rampMs = APP_CAR_H4_RAMP_MS;
        /* H4用球控HOLD状态的连续时间，经过A线切换路线状态时不重置缓启动。 */
        rampElapsedMs = pCar->ballStateMs;
    } else if (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
        startSpeed = APP_CAR_H5_RAMP_START_SPEED;
        rampMs = APP_CAR_H5_RAMP_MS;
        rampElapsedMs = pCar->routeStateMs;
    } else {
        return targetSpeed;
    }

    if (startSpeed > targetSpeed) {
        startSpeed = targetSpeed;
    }

    if ((rampMs == 0U) || (rampElapsedMs >= rampMs)) {
        return targetSpeed;
    }

    speed = (int32_t)startSpeed +
        (((int32_t)(targetSpeed - startSpeed) * (int32_t)rampElapsedMs) /
         (int32_t)rampMs);
    return (int16_t)speed;
}

static int16_t _GetBallHoldTargetMm(const AppCarDef *pCar)
{
    int32_t targetMm = pCar->ballTargetMm;
    int32_t feedforwardMm = 0;
    int32_t accelFfMm = 0;
    uint32_t tMs;
    uint32_t fadeMs = 0U;
    uint32_t holdMs = 0U;

    if (pCar->fatherState != APP_CAR_FATHER_RUNNING) {
        return _ClampBallTargetMm((int16_t)targetMm);
    }

    if (pCar->mode == APP_CAR_MODE_BALANCE_AB) {
        targetMm += pCar->h4AccFfMm;
        feedforwardMm = APP_CAR_H4_START_FF_MM;
        holdMs = APP_CAR_H4_START_FF_HOLD_MS;
        fadeMs = APP_CAR_H4_START_FF_FADE_MS;
    } else if (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
        feedforwardMm = APP_CAR_H5_START_FF_MM;
        holdMs = APP_CAR_H5_START_FF_HOLD_MS;
        fadeMs = APP_CAR_H5_START_FF_FADE_MS;
        accelFfMm = pCar->h5AccelFfMm;
    } else {
        return _ClampBallTargetMm((int16_t)targetMm);
    }

    if ((pCar->routeState != APP_CAR_ROUTE_LEAVE_START) &&
        (pCar->routeState != APP_CAR_ROUTE_TRACKING)) {
        return _ClampBallTargetMm((int16_t)targetMm);
    }

    tMs = pCar->routeStateMs;

    if (tMs < holdMs) {
        targetMm += feedforwardMm;
    } else if ((fadeMs != 0U) && (tMs < (holdMs + fadeMs))) {
        targetMm += (feedforwardMm * (int32_t)((holdMs + fadeMs) - tMs)) /
            (int32_t)fadeMs;
    }

    targetMm += accelFfMm;

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
        return (uint16_t)(APP_CAR_H4_FINISH_CRUISE_MS +
            APP_CAR_H4_FINISH_DECEL_MS);
    }
    if (mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
        return APP_CAR_H5_FINISH_FOLLOW_MS;
    }

    return APP_CAR_H2_FINISH_FOLLOW_MS;
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

static void _LockH2FinishForwardYaw(AppCarDef *pCar)
{
    if ((pCar->imuOnline != 0U) && (pCar->imuHasLastYaw != 0U)) {
        pCar->h2FinishForwardYawCd = pCar->imuLapYawCd;
        pCar->h2FinishForwardYawValid = 1U;
    } else {
        pCar->h2FinishForwardYawValid = 0U;
    }
}

static void _RunH2FinishForward(AppCarDef *pCar)
{
    int16_t baseSpeed = APP_CAR_H2_FINISH_FORWARD_SPEED;
    int16_t turnSpeed = 0;
    int32_t errorCd;
    int32_t turnAbs;

    if ((pCar->h2FinishForwardYawValid != 0U) &&
        (pCar->imuOnline != 0U) &&
        (pCar->imuHasLastYaw != 0U)) {
        errorCd = _H2FinishForwardErrorCd(pCar);
        turnAbs = (_AbsI32(errorCd) * APP_CAR_H2_FINISH_FORWARD_KP_NUM) /
            APP_CAR_H2_FINISH_FORWARD_KP_DEN;
        if (turnAbs > APP_CAR_H2_FINISH_FORWARD_LIMIT) {
            turnAbs = APP_CAR_H2_FINISH_FORWARD_LIMIT;
        }
        turnSpeed = (int16_t)turnAbs;
        if (((errorCd > 0) && (APP_CAR_H2_FINISH_FORWARD_DIRECTION > 0)) ||
            ((errorCd < 0) && (APP_CAR_H2_FINISH_FORWARD_DIRECTION < 0))) {
            turnSpeed = (int16_t)-turnSpeed;
        }
    }

    pCar->leftCommand = (int16_t)(baseSpeed - turnSpeed);
    pCar->rightCommand = (int16_t)(baseSpeed + turnSpeed);
    BspMotor_SetSignedSpeed(pCar->leftCommand, pCar->rightCommand);
}

static int32_t _H2FinishForwardErrorCd(const AppCarDef *pCar)
{
    int32_t errorCd = pCar->imuLapYawCd - pCar->h2FinishForwardYawCd;

    while (errorCd > 18000L) {
        errorCd -= 36000L;
    }
    while (errorCd < -18000L) {
        errorCd += 36000L;
    }

    return errorCd;
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
    pCar->h2FinishForwardYawCd = 0;
    pCar->imuYawCd = 0;
    pCar->imuLastYawCd = 0;
    pCar->imuSampleSeq = 0U;
    pCar->imuValid = 0U;
    pCar->imuOnline = 0U;
    pCar->imuHasLastYaw = 0U;
    pCar->h2FinishForwardYawValid = 0U;
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
        /* H4通过B由1.5m编码器距离单独判断，不使用横线终点逻辑。 */
        return 0U;
    }
    if (pCar->mode == APP_CAR_MODE_BALANCE_LAP_CENTER) {
        return (uint8_t)(pCar->routePulses >= APP_CAR_H5_LAP_GATE_PULSES);
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
        "[T] f=%u r=%u b=%u mode=%u time=%lu target=%dmm ball=%d/%u h5b=%d h5af=%d gray=%02X enc=%d,%d count=%ld,%ld lap=%lu iy=%ld/%d/%u/%u cmd=%d,%d tr=%d/%u q=%u k=%lu,%lu,%lu,%lu\n",
        (unsigned)pCar->fatherState,
        (unsigned)pCar->routeState,
        (unsigned)pCar->ballState,
        (unsigned)pCar->mode,
        (unsigned long)pCar->elapsedMs,
        (int)pCar->ballTargetMm,
        (int)pCar->ballOffsetMm,
        (unsigned)pCar->ballValid,
        (int)pCar->h5HoldBiasMm,
        (int)pCar->h5AccelFfMm,
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
}

static void _PrintH4Telemetry(const AppCarDef *pCar)
{
    BspJy61pData_t imu;
    int16_t targetMm;

    (void)BspJy61p_GetData(&imu);
    targetMm = _GetBallHoldTargetMm(pCar);
    BspUart_Printf(
        "[H4D] t=%lu s=%lu r=%u p=%lu b=%d/%u ay=%d iv=%u bias=%d ayf=%d ff=%d tg=%d rej=%u c=%d,%d d=%d\n",
        (unsigned long)pCar->ballStateMs,
        (unsigned long)pCar->elapsedMs,
        (unsigned)pCar->routeState,
        (unsigned long)pCar->routePulses,
        (int)pCar->ballOffsetMm,
        (unsigned)pCar->ballValid,
        (int)imu.accYmg,
        (unsigned)imu.valid,
        (int)pCar->h4AccBiasMg,
        (int)pCar->h4AccFiltMg,
        (int)pCar->h4AccFfMm,
        (int)targetMm,
        (unsigned)pCar->h4AccRejectCount,
        (int)pCar->leftCommand,
        (int)pCar->rightCommand,
        (int)pCar->traceTurn);
}

static int16_t _GetTraceSpeed(AppCarMode_t mode)
{
    switch (mode) {
        case APP_CAR_MODE_BALANCE_AB:
            return APP_CAR_TRACE_SPEED_H4;
        case APP_CAR_MODE_BALANCE_LAP_CENTER:
            return APP_CAR_TRACE_SPEED_H5;
        case APP_CAR_MODE_TRACE_ONLY:
        case APP_CAR_MODE_BALL_STATIC:
        default:
            return APP_CAR_TRACE_SPEED_H2;
    }
}
