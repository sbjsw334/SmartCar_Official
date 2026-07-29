#ifndef _TRACE_CONTROL_H_
#define _TRACE_CONTROL_H_

#include <stdint.h>

typedef enum {
    TRACE_STATE_STOP_MARK = 0,
    TRACE_STATE_CENTERED,
    TRACE_STATE_TRACKING,
    TRACE_STATE_SEARCHING,
} TraceState_t;

typedef struct {
    int8_t lastTurn;
    TraceState_t state;
    int16_t leftCommand;
    int16_t rightCommand;
} TraceControl_t;

void TraceControl_Init(TraceControl_t *pControl);
void TraceControl_Update(TraceControl_t *pControl, uint8_t gray);

#endif /* _TRACE_CONTROL_H_ */
