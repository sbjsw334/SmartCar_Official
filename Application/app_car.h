#ifndef _APP_CAR_H_
#define _APP_CAR_H_

#include <stdint.h>

#include "msg_map.h"
#include "ball_control.h"
#include "trace_control.h"

#define APP_CAR_CONTROL_PERIOD_MS (2U)

typedef struct _AppCarDef AppCarDef;

typedef enum {
    APP_CAR_FATHER_STOPPED = 0,
    APP_CAR_FATHER_RUNNING,
    APP_CAR_FATHER_FINISHED,
    APP_CAR_FATHER_FAULT,
} AppCarFatherState_t;

typedef enum {
    APP_CAR_ROUTE_DISABLED = 0,
    APP_CAR_ROUTE_LEAVE_START,
    APP_CAR_ROUTE_TRACKING,
    APP_CAR_ROUTE_FINISH_ACTION,
    APP_CAR_ROUTE_FINISH_BRAKE,
    APP_CAR_ROUTE_FINISH_ALIGN,
    APP_CAR_ROUTE_COMPLETE,
} AppCarRouteState_t;

typedef enum {
    APP_CAR_BALL_DISABLED = 0,
    APP_CAR_BALL_WAIT_VISION,
    APP_CAR_BALL_MOVE_POSITIVE,
    APP_CAR_BALL_MOVE_NEGATIVE,
    APP_CAR_BALL_HOLD_TARGET,
} AppCarBallState_t;

typedef enum {
    APP_CAR_MODE_TRACE_ONLY = 0,
    APP_CAR_MODE_BALL_STATIC,
    APP_CAR_MODE_BALANCE_AB,
    APP_CAR_MODE_BALANCE_LAP_CENTER,
    APP_CAR_MODE_BALANCE_LAP_TARGET,
} AppCarMode_t;

typedef void (*AppCarFatherStateHandler_t)(AppCarDef *pCar, MsgId_t msg);
typedef void (*AppCarChildStateHandler_t)(AppCarDef *pCar);

struct _AppCarDef {
    AppCarFatherStateHandler_t pFatherState;
    AppCarChildStateHandler_t pRouteState;
    AppCarChildStateHandler_t pBallState;

    AppCarFatherState_t fatherState;
    AppCarRouteState_t routeState;
    AppCarBallState_t ballState;
    AppCarMode_t mode;

    volatile uint32_t uptimeMs;
    volatile uint32_t elapsedMs;
    uint32_t routeStateMs;
    uint32_t ballStateMs;
    uint32_t routePulses;
    uint16_t finishLineMs;
    uint16_t finishAlignStableMs;
    int32_t imuLapYawCd;
    int32_t h2FinishForwardYawCd;
    int16_t imuYawCd;
    int16_t imuLastYawCd;
    uint32_t imuSampleSeq;
    uint16_t ballStableMs;
    uint32_t ballStableFrameSeq;
    int16_t ballTargetMm;
    int16_t lastBallTargetMm;   /* 上一帧目标，用于判断爬坡是否结束 */
    int16_t ballOffsetMm;
    uint32_t ballFrameSeq;
    int16_t leftSpeed;
    int16_t rightSpeed;
    int32_t leftCount;
    int32_t rightCount;
    int16_t leftCommand;
    int16_t rightCommand;
    int8_t traceTurn;
    uint8_t traceState;
    uint8_t ballValid;
    uint8_t imuValid;
    uint8_t imuOnline;
    uint8_t imuHasLastYaw;
    uint8_t h2FinishForwardYawValid;
    uint8_t gray;
    volatile uint8_t timerRunning;

    TraceControl_t trace;
    BallControl_t ballControl;
};

typedef struct {
    void (*init)(AppCarDef *pCar);
    void (*run)(AppCarDef *pCar, MsgId_t msg);
    void (*setMode)(AppCarDef *pCar, AppCarMode_t mode);
    void (*setBallTargetMm)(AppCarDef *pCar, int16_t targetMm);
    AppCarFatherState_t (*getFatherState)(const AppCarDef *pCar);
    AppCarRouteState_t (*getRouteState)(const AppCarDef *pCar);
    AppCarBallState_t (*getBallState)(const AppCarDef *pCar);
} AppCarConDef;

extern AppCarConDef appCarCon;
extern AppCarDef appCarMain;

/* Called by the 1 ms SysTick. Competition timing never depends on the queue. */
void AppCar_Tick1ms(AppCarDef *pCar);

#endif /* _APP_CAR_H_ */
