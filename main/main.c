#include "app_config.h"
#include "app_config_wifi.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include <stdint.h>
#include "app_common.h"
#include "app_board.h"
#include "app_ble.h"
#include "app_config_mqtt_light.h"

#define TAG "MAIN"
#define TASK_STACK_SIZE 4096
#define ADC_PERIOD_MS   100

bool mqtt_enable;
char *avail_topic;

uint16_t adc_values[1000/ADC_PERIOD_MS];
uint16_t adc_counter;
app_config_mqtt_light_t *lights[6];

app_config_cbs_t app_cbs;

void notify(queue_value_t state){
    bool config_mesh_enable;
    bool config_mqtt_enable;
    app_config_getBool("ble_mesh_enable", &config_mesh_enable);
    app_config_getBool("mqtt_enable", &config_mqtt_enable);
    if (config_mesh_enable) {
        ESP_LOGI(TAG, "BLE Mesh enabled, notifying");
        app_ble_notify(state);
    }
    if (config_mqtt_enable && state.state != lights[state.channel]->state.brightness){
        app_config_mqtt_light_set_brightness(state.state, lights[state.channel]);
    }
}

void worker_task( void *pvParameters ){
    for (;;){
        queue_value_t item;
        portBASE_TYPE xStatus = xQueueReceive(state_queue, &item, portMAX_DELAY );
        if( xStatus == pdPASS ){
            ESP_LOGI(TAG, "Received from queue: channel=%d, value=%d", item.channel, item.state);
            app_board_set_level(item.channel, item.state);
            notify(item);
        }
        else {
            ESP_LOGI(TAG, "Queue receive timed out");
        }
        //int val = app_board_get_adc();
        //ESP_LOGI(TAG, "ADC: %d", val);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void adc_task( void *pvParameters ){
    for (;;){
        adc_values[adc_counter++] = app_board_get_adc();
        if(adc_counter == 1000/ADC_PERIOD_MS){
            uint32_t acc = 0;
            for(uint16_t i=0; i<adc_counter; i++){
                acc += adc_values[i];
            }
            adc_counter = 0;
            acc = acc/(1000/ADC_PERIOD_MS);
            ESP_LOGI(TAG, "ADC: %d", acc);
            queue_value_t item;
            item.channel = CHANNEL_NUMBER + 1;
            item.state = acc;
            notify(item);
        }
        vTaskDelay(ADC_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void ip_cb(ip_event_t event, void *event_data){
    char *std_mqtt_prefix;
    app_config_getString("std_mqtt_prefix", &std_mqtt_prefix);
    char *std_mqtt_objid;
    app_config_getString("std_mqtt_objid", &std_mqtt_objid);
    int avail_topic_len = strlen(std_mqtt_prefix) + strlen(std_mqtt_objid) + strlen(CONFIG_APP_CONFIG_MQTT_SWITCH_AVAIL_STR) + 3;
    avail_topic = (char *)malloc(avail_topic_len);
    snprintf(avail_topic, avail_topic_len, "%s/%s%s", std_mqtt_prefix, std_mqtt_objid, CONFIG_APP_CONFIG_MQTT_SWITCH_AVAIL_STR);
    app_config_mqtt_lwt_t lwt = {.topic = avail_topic, .msg = "offline"};
    ESP_LOGI(TAG, "LWT topic: %s", lwt.topic);
    app_config_mqtt_init(&lwt);
}

void mqtt_light_cmd_handler(app_config_light_state_t state, app_config_mqtt_light_t *light){
    ESP_LOGI(TAG, "Setting channel %d to %d", (int)light->user_data, state.onoff);
    if(state.onoff){
        app_board_set_on((int)light->user_data);
    } else {
        app_board_set_off((int)light->user_data);
    }
}

void mqtt_light_brightness_handler(app_config_light_state_t state, app_config_mqtt_light_t *light){
    ESP_LOGD(TAG, "Setting brightness %d to %d", (int)light->user_data, state.brightness);
    queue_value((int)light->user_data, state.brightness);
}

void mqtt_cb(esp_mqtt_event_handle_t event){
    if(event->event_id == MQTT_EVENT_CONNECTED){
        app_config_mqtt_publish(avail_topic, "online", true);
        char *light_prefix;
        app_config_getString("light_prefix", &light_prefix);
        for(int i=0; i<6; i++){
            char *light_name;
            char light_obj[10] = {0};
            char light_iter[15] = {0};
            snprintf(light_iter, 15, "light%d_name", (uint8_t)i+1);
            snprintf(light_obj, 10, "light%d", (uint8_t)i+1);
            app_config_getString("light_prefix", &light_prefix);
            app_config_getString(light_iter, &light_name);
            if(strlen(light_name))
                lights[i] = app_config_mqtt_light_create(light_prefix,
                                                    light_obj,
                                                    light_name,
                                                    mqtt_light_cmd_handler,
                                                    mqtt_light_brightness_handler, true, true, (void *)i);
        }
    } else if(event->event_id == MQTT_EVENT_DISCONNECTED){
        free(avail_topic);
    }
}

void app_main(void){
    app_board_init();

    //app_mqtt_get_lwt(&app_cbs.lwt.topic, &app_cbs.lwt.msg);
    app_cbs.config_srv = app_ble_mesh_config_server_cb;
    app_cbs.generic_srv = app_ble_mesh_generic_server_cb;
    //app_cbs.mqtt = mqtt_event_handler;
    ESP_ERROR_CHECK(app_config_init(&app_cbs));

    ESP_LOGI(TAG, "Creating worker queue");
    app_common_queue_init();

    ESP_LOGI(TAG, "Starting worker task");
    if (xTaskCreate(worker_task, "Worker1", TASK_STACK_SIZE, NULL, 1, NULL ) != pdPASS){
        ESP_LOGE(TAG, "Error creating worker task");
        return;
    }
    
    char *current_topic;
    esp_err_t err = app_config_getValue("current_element", string, &current_topic);
    if ((err == ESP_OK) && (strlen(current_topic) > 0)){
        ESP_LOGI(TAG, "Starting ADC task");
        if (xTaskCreate(adc_task, "ADC1", TASK_STACK_SIZE, NULL, 1, NULL ) != pdPASS){
            ESP_LOGE(TAG, "Error creating worker task");
            return;
        }
    }

    app_config_getBool("mqtt_enable", &mqtt_enable);    
    if(mqtt_enable){
        app_config_ip_register_cb(IP_EVENT_STA_GOT_IP, ip_cb);
        app_config_mqtt_register_cb(MQTT_EVENT_CONNECTED, mqtt_cb);
        app_config_mqtt_register_cb(MQTT_EVENT_DISCONNECTED, mqtt_cb);
    }
}
