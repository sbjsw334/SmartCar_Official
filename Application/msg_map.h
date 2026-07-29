#ifndef _MSG_MAP_H_
#define _MSG_MAP_H_

#include <stdint.h>

typedef enum {
    MSG_NONE = 0,
    MSG_CONTROL_10MS,
    MSG_TELEMETRY_200MS,
    MSG_KEY_START,
    MSG_STOP,
    MSG_FAULT,
} MsgId_t;

#define MSG_MAP_QUEUE_SIZE (16U)

void MsgMap_Init(void);
uint8_t MsgMap_Post(MsgId_t msg);
uint8_t MsgMap_Get(MsgId_t *pMsg);
uint8_t MsgMap_GetOverflowCount(void);

#endif /* _MSG_MAP_H_ */
