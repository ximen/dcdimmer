#ifndef _APP_MQTT_H_
#define _APP_MQTT_H_

#include <stdint.h>
#include "app_common.h"

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
char *app_mqtt_get_topic(uint8_t channel);
void app_mqtt_notify(queue_value_t state);

#endif  // _APP_MQTT_H_