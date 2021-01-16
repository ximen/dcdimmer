#include "app_config.h"
#include "app_config_mqtt.h"
#include "app_common.h"
#include "app_mqtt.h"
#include "app_board.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"

#define TAG             "APP_MQTT"
#define AVAIL_TOPIC     "/available"
#define OFFLINE_MSG     "offline"
#define ALLOC_ERR_STR   "Error allocating buffer!"

char *app_mqtt_get_topic(uint8_t channel){
    char *topic;
    char topic_name[CONFIG_APP_CONFIG_SHORT_NAME_LEN];
    sprintf(topic_name, "topic%d_element", channel + 1);
    esp_err_t err = app_config_getValue(topic_name, string, &topic);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Error retrieving topic %s", topic_name);
        return NULL;
    }
    return topic;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id) {
            case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            for(uint8_t i=0; i<CHANNEL_NUMBER; i++){
                char *topic = app_mqtt_get_topic(i);
                char brightness_topic[58] = {0};
                strncat(brightness_topic, topic, 50);
                strncat(brightness_topic, "/brightness", 50);
                if(strlen(topic) > 0){
                    ESP_LOGI(TAG, "Subscribing %s", topic);
                    esp_mqtt_client_subscribe(client, topic, 1);
                    ESP_LOGI(TAG, "Subscribing %s", brightness_topic);
                    esp_mqtt_client_subscribe(client, brightness_topic, 1);
                }
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            for(uint8_t i=0; i < CHANNEL_NUMBER; i++){
                char *topic = app_mqtt_get_topic(i);
                char brightness_topic[58] = {0};
                strncat(brightness_topic, topic, 50);
                strncat(brightness_topic, "/brightness", 50);
                if(strlen(topic) == 0){
                    ESP_LOGI(TAG, "Empty topic %d", i);
                    continue;
                }
                if(strncmp(event->topic, topic, event->topic_len) == 0){
                    if(strncmp(event->data, "ON", event->data_len) == 0){
                        ESP_LOGI(TAG, "Got ON");
                        uint8_t current_value = app_board_get_level(i);
                        ESP_LOGI(TAG, "Current value: %d", current_value);
                        if(current_value == 0){
                            queue_value(i, 100);
                        } 
                    } else if (strncmp(event->data, "OFF", event->data_len) == 0){
                            ESP_LOGI(TAG, "Got OFF");
                        queue_value(i, 0);
                    } else {
                        ESP_LOGW(TAG, "Error parsing payload");
                    }
                    break;
                } else if(strncmp(event->topic, brightness_topic, event->topic_len) == 0){
                    char val_str[4] = {0};
                    strncpy(val_str, event->data, (event->data_len > 3)?(3):(event->data_len));
                    long val = strtol(val_str, NULL, 10);
                    if(val>100){
                        ESP_LOGW(TAG, "Wrong brightness value %d", (int)val);
                        continue;
                    }
                    queue_value(i, (uint8_t)val);
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
        case MQTT_EVENT_ANY:
            ESP_LOGI(TAG, "MQTT_EVENT_ANY");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

void app_mqtt_notify(queue_value_t state){
        if(state.channel < CHANNEL_NUMBER){                 // Channel topic
            char *topic = app_mqtt_get_topic(state.channel);
            char status_topic[58] = {0};
            char brightness_status_topic[58] = {0};
            strncat(status_topic, topic, 50);
            strncat(brightness_status_topic, topic, 50);
            strcat(status_topic, "/status");
            strcat(brightness_status_topic, "/brightness/status");
            if (strlen(topic) > 0){
                ESP_LOGI(TAG, "Publishing MQTT status. Topic %s, value %d", topic, state.state);
                if(state.state) app_config_mqtt_publish(status_topic, "ON", true);
                else app_config_mqtt_publish(status_topic, "OFF", true);
                char brightness_value[4];
                sprintf(brightness_value, "%3d", state.state);
                ESP_LOGI(TAG, "Publishing MQTT brightness status. Topic %s, value %s", brightness_status_topic, brightness_value);
                app_config_mqtt_publish(brightness_status_topic, brightness_value, true);
            }
            else{
                ESP_LOGI(TAG,"Topic not specified");
            }
        } else {        // Current topic
            char *current_topic;
            esp_err_t err = app_config_getValue("current_element", string, &current_topic);
            if ((err == ESP_OK) && (strlen(current_topic) > 0)){
                ESP_LOGI(TAG, "Publishing current value %d on topic %s", state.state, current_topic);
                char current_value[6];
                sprintf(current_value, "%2.2f", (float)state.state*(16.5/4096));
                app_config_mqtt_publish(current_topic, current_value, true);
            }

        }

}

void app_mqtt_get_lwt(char **topic, char **msg){
    for (uint8_t i = 0; i < CHANNEL_NUMBER; i++){
        char element[17] = {0};
        sprintf(element, "topic%d_element", i + 1);
        char *base_path;
        app_config_getValue(element, string, &base_path);
        if(strlen(base_path)){
            char *avail_topic = calloc(strlen(base_path) + sizeof(AVAIL_TOPIC) + 1, sizeof(char));
            if (avail_topic) {
                strncat(avail_topic, base_path, strlen(base_path) + 1);
                strncat(avail_topic, AVAIL_TOPIC, sizeof(AVAIL_TOPIC) + 1);
                *topic = avail_topic;
                *msg = OFFLINE_MSG;
            } else {
                ESP_LOGE(TAG, ALLOC_ERR_STR);
                free(base_path);
            }
        }
    }
}