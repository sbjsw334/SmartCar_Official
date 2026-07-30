#include "msg_map.h"

#include "ti_msp_dl_config.h"

typedef struct {
    MsgId_t buffer[MSG_MAP_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t overflowCount;
    uint8_t controlTickPending;
    uint8_t telemetryTickPending;
} MsgQueue_t;

static volatile MsgQueue_t s_queue;

void MsgMap_Init(void)
{
    s_queue.head = 0U;
    s_queue.tail = 0U;
    s_queue.count = 0U;
    s_queue.overflowCount = 0U;
    s_queue.controlTickPending = 0U;
    s_queue.telemetryTickPending = 0U;
}

uint8_t MsgMap_Post(MsgId_t msg)
{
    if (msg == MSG_NONE) {
        return 1U;
    }

    __disable_irq();

    if (((msg == MSG_CONTROL_TICK) && (s_queue.controlTickPending != 0U)) ||
        ((msg == MSG_TELEMETRY_200MS) && (s_queue.telemetryTickPending != 0U))) {
        __enable_irq();
        return 1U;
    }

    if (s_queue.count >= MSG_MAP_QUEUE_SIZE) {
        if (s_queue.overflowCount < 0xFFU) {
            s_queue.overflowCount++;
        }
        __enable_irq();
        return 0U;
    }

    s_queue.buffer[s_queue.tail] = msg;
    s_queue.tail = (uint8_t)((s_queue.tail + 1U) % MSG_MAP_QUEUE_SIZE);
    s_queue.count++;

    if (msg == MSG_CONTROL_TICK) {
        s_queue.controlTickPending = 1U;
    } else if (msg == MSG_TELEMETRY_200MS) {
        s_queue.telemetryTickPending = 1U;
    }

    __enable_irq();
    return 1U;
}

uint8_t MsgMap_Get(MsgId_t *pMsg)
{
    if (pMsg == 0) {
        return 0U;
    }

    __disable_irq();

    if (s_queue.count == 0U) {
        *pMsg = MSG_NONE;
        __enable_irq();
        return 0U;
    }

    *pMsg = s_queue.buffer[s_queue.head];
    s_queue.head = (uint8_t)((s_queue.head + 1U) % MSG_MAP_QUEUE_SIZE);
    s_queue.count--;

    if (*pMsg == MSG_CONTROL_TICK) {
        s_queue.controlTickPending = 0U;
    } else if (*pMsg == MSG_TELEMETRY_200MS) {
        s_queue.telemetryTickPending = 0U;
    }

    __enable_irq();
    return 1U;
}

uint8_t MsgMap_GetOverflowCount(void)
{
    uint8_t count;

    __disable_irq();
    count = s_queue.overflowCount;
    __enable_irq();

    return count;
}
