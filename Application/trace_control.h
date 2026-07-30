#ifndef _TRACE_CONTROL_H_
#define _TRACE_CONTROL_H_

#include <stdint.h>

typedef enum {
    TRACE_STATE_CENTERED = 0,
    TRACE_STATE_TRACKING,
    TRACE_STATE_SEARCHING,
} TraceState_t;

typedef struct {
    int16_t baseSpeed;
    int8_t lastTurn;
    TraceState_t state;
    int16_t leftCommand;
    int16_t rightCommand;
} TraceControl_t;

void TraceControl_Init(TraceControl_t *pControl);
void TraceControl_SetBaseSpeed(TraceControl_t *pControl, int16_t speed);
void TraceControl_Update(TraceControl_t *pControl, uint8_t gray);

#endif /* _TRACE_CONTROL_H_ */
