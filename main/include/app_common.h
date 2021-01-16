#ifndef _APP_COMMON_H_
#define _APP_COMMON_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define QUEUE_LENGTH    5

typedef struct {
    uint8_t channel;        // Channel number. Values over CHACHANNEL_NUMBER are for current sensor
    uint8_t state;          // New state in percent
} queue_value_t;

xQueueHandle state_queue;
void app_common_queue_init();
void queue_value(uint8_t channel, uint8_t value);
//void app_common_queue_item(queue_value_t item);

#endif  // _APP_COMMON_H_
