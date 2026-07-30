#include "trace_control.h"

#define TRACE_DEFAULT_SPEED (30)
#define TRACE_SPEED_MIN     (10)
#define TRACE_SPEED_MAX     (80)

static void _UseLastDirection(TraceControl_t *pControl);

void TraceControl_Init(TraceControl_t *pControl)
{
    if (pControl == 0) {
        return;
    }

    pControl->baseSpeed = TRACE_DEFAULT_SPEED;
    pControl->lastTurn = 0;
    pControl->state = TRACE_STATE_SEARCHING;
    pControl->leftCommand = 0;
    pControl->rightCommand = 0;
}

void TraceControl_SetBaseSpeed(TraceControl_t *pControl, int16_t speed)
{
    if (pControl == 0) {
        return;
    }

    if (speed < TRACE_SPEED_MIN) {
        speed = TRACE_SPEED_MIN;
    } else if (speed > TRACE_SPEED_MAX) {
        speed = TRACE_SPEED_MAX;
    }
    pControl->baseSpeed = speed;
}

void TraceControl_Update(TraceControl_t *pControl, uint8_t gray)
{
    uint8_t pattern;
    int16_t speed;

    if (pControl == 0) {
        return;
    }

    /* BspGray uses 1 for black; the proven rule table uses 0 for black. */
    pattern = (uint8_t)~gray;
    speed = pControl->baseSpeed;

    if ((pattern == 0xEFU) || (pattern == 0xE7U) ||
        (pattern == 0xF7U)) {
        pControl->leftCommand = speed;
        pControl->rightCommand = speed;
        pControl->lastTurn = 0;
        pControl->state = TRACE_STATE_CENTERED;
    } else if ((pattern == 0xF3U) || (pattern == 0xFBU)) {
        pControl->leftCommand = speed + 5;
        pControl->rightCommand = speed - 5;
        pControl->lastTurn = 1;
        pControl->state = TRACE_STATE_TRACKING;
    } else if ((pattern == 0xF9U) || (pattern == 0xFDU)) {
        pControl->leftCommand = speed + 8;
        pControl->rightCommand = speed - 8;
        pControl->lastTurn = 2;
        pControl->state = TRACE_STATE_TRACKING;
    } else if ((pattern == 0xFCU) || (pattern == 0xFEU)) {
        pControl->leftCommand = speed + 10;
        pControl->rightCommand = speed - 10;
        pControl->lastTurn = 3;
        pControl->state = TRACE_STATE_TRACKING;
    } else if ((pattern == 0xCFU) || (pattern == 0xDFU)) {
        pControl->leftCommand = speed - 5;
        pControl->rightCommand = speed + 5;
        pControl->lastTurn = -1;
        pControl->state = TRACE_STATE_TRACKING;
    } else if ((pattern == 0x9FU) || (pattern == 0xBFU)) {
        pControl->leftCommand = speed - 8;
        pControl->rightCommand = speed + 8;
        pControl->lastTurn = -2;
        pControl->state = TRACE_STATE_TRACKING;
    } else if ((pattern == 0x3FU) || (pattern == 0x7FU)) {
        pControl->leftCommand = speed - 10;
        pControl->rightCommand = speed + 10;
        pControl->lastTurn = -3;
        pControl->state = TRACE_STATE_TRACKING;
    } else {
        pControl->state = TRACE_STATE_SEARCHING;
        _UseLastDirection(pControl);
    }
}

static void _UseLastDirection(TraceControl_t *pControl)
{
    int16_t speed = pControl->baseSpeed;

    switch (pControl->lastTurn) {
        case 3:
            pControl->leftCommand = speed + 10;
            pControl->rightCommand = -10;
            break;
        case 2:
            pControl->leftCommand = speed + 8;
            pControl->rightCommand = -5;
            break;
        case 1:
            pControl->leftCommand = speed + 5;
            pControl->rightCommand = 0;
            break;
        case -1:
            pControl->leftCommand = 0;
            pControl->rightCommand = speed + 5;
            break;
        case -2:
            pControl->leftCommand = -5;
            pControl->rightCommand = speed + 8;
            break;
        case -3:
            pControl->leftCommand = -10;
            pControl->rightCommand = speed + 10;
            break;
        case 0:
        default:
            pControl->leftCommand = speed;
            pControl->rightCommand = speed;
            break;
    }
}
