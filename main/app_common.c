#include "app_common.h"
#include "app_board.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG     "APP_COMMON"

const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;

void queue_value(uint8_t channel, uint8_t value){
    queue_value_t item;
    item.channel = channel;
    item.state = value;
    portBASE_TYPE status = xQueueSend(state_queue, &item, xTicksToWait);
    if (status != pdPASS){
        ESP_LOGE(TAG, "Error queuing state");
    }
}

void app_common_queue_init(){
    state_queue = xQueueCreate(QUEUE_LENGTH, sizeof(queue_value_t));
    if(!state_queue){
        ESP_LOGE(TAG, "Error creating queue");
        return;
    }
}

