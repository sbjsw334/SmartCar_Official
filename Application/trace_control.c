#include "trace_control.h"

#include "bsp_gray.h"

#define TRACE_BASE_SPEED          (20)
#define TRACE_CORRECTION_LIGHT    (5)
#define TRACE_CORRECTION_MEDIUM   (10)
#define TRACE_CORRECTION_STRONG   (20)
#define TRACE_SEARCH_REVERSE      (15)

static void _SetTurn(TraceControl_t *pControl, int8_t turnLevel);
static void _SearchLastDirection(TraceControl_t *pControl);

void TraceControl_Init(TraceControl_t *pControl)
{
    if (pControl == 0) {
        return;
    }

    pControl->lastTurn = 0;
    pControl->state = TRACE_STATE_SEARCHING;
    pControl->leftCommand = 0;
    pControl->rightCommand = 0;
}

void TraceControl_Update(TraceControl_t *pControl, uint8_t gray)
{
    if (pControl == 0) {
        return;
    }

    if (gray == BSP_GRAY_ALL_BLACK) {
        pControl->state = TRACE_STATE_STOP_MARK;
        pControl->leftCommand = 0;
        pControl->rightCommand = 0;
        return;
    }

    switch (gray) {
        case 0x08U:
        case 0x10U:
        case 0x18U:
            pControl->lastTurn = 0;
            pControl->state = TRACE_STATE_CENTERED;
            pControl->leftCommand = TRACE_BASE_SPEED;
            pControl->rightCommand = TRACE_BASE_SPEED;
            break;

        case 0x04U:
        case 0x0CU:
            _SetTurn(pControl, 1);
            break;

        case 0x02U:
        case 0x06U:
            _SetTurn(pControl, 2);
            break;

        case 0x01U:
        case 0x03U:
            _SetTurn(pControl, 3);
            break;

        case 0x20U:
        case 0x30U:
            _SetTurn(pControl, -1);
            break;

        case 0x40U:
        case 0x60U:
            _SetTurn(pControl, -2);
            break;

        case 0x80U:
        case 0xC0U:
            _SetTurn(pControl, -3);
            break;

        /* Wide edge patterns usually appear at a right-angle bend. */
        case 0x07U:
        case 0x0FU:
        case 0x1FU:
        case 0x3FU:
        case 0x7FU:
            pControl->lastTurn = 3;
            _SearchLastDirection(pControl);
            break;

        case 0xE0U:
        case 0xF0U:
        case 0xF8U:
        case 0xFCU:
        case 0xFEU:
            pControl->lastTurn = -3;
            _SearchLastDirection(pControl);
            break;

        default:
            _SearchLastDirection(pControl);
            break;
    }
}

static void _SetTurn(TraceControl_t *pControl, int8_t turnLevel)
{
    int16_t correction;

    if ((turnLevel > 3) || (turnLevel < -3)) {
        return;
    }

    if ((turnLevel == 1) || (turnLevel == -1)) {
        correction = TRACE_CORRECTION_LIGHT;
    } else if ((turnLevel == 2) || (turnLevel == -2)) {
        correction = TRACE_CORRECTION_MEDIUM;
    } else {
        correction = TRACE_CORRECTION_STRONG;
    }

    pControl->lastTurn = turnLevel;
    pControl->state = TRACE_STATE_TRACKING;

    if (turnLevel > 0) {
        pControl->leftCommand = TRACE_BASE_SPEED + correction;
        pControl->rightCommand = TRACE_BASE_SPEED - correction;
    } else {
        pControl->leftCommand = TRACE_BASE_SPEED - correction;
        pControl->rightCommand = TRACE_BASE_SPEED + correction;
    }
}

static void _SearchLastDirection(TraceControl_t *pControl)
{
    int16_t searchSpeed;
    int16_t insideSpeed;
    int8_t level = pControl->lastTurn;

    pControl->state = TRACE_STATE_SEARCHING;

    if (level == 0) {
        pControl->leftCommand = TRACE_BASE_SPEED;
        pControl->rightCommand = TRACE_BASE_SPEED;
        return;
    }

    searchSpeed = TRACE_BASE_SPEED + TRACE_CORRECTION_STRONG;
    insideSpeed = 0;
    if ((level >= 3) || (level <= -3)) {
        insideSpeed = -TRACE_SEARCH_REVERSE;
    } else if ((level == 2) || (level == -2)) {
        insideSpeed = -TRACE_CORRECTION_LIGHT;
    }

    if (level > 0) {
        pControl->leftCommand = searchSpeed;
        pControl->rightCommand = insideSpeed;
    } else {
        pControl->leftCommand = insideSpeed;
        pControl->rightCommand = searchSpeed;
    }
}
