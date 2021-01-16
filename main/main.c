#include "app_config.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include <stdint.h>
#include "app_common.h"
#include "app_board.h"
#include "app_mqtt.h"
#include "app_ble.h"

#define TAG "MAIN"
#define TASK_STACK_SIZE 4096
#define ADC_PERIOD_MS   100

uint16_t adc_values[1000/ADC_PERIOD_MS];
uint16_t adc_counter;

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
    if (config_mqtt_enable){
        app_mqtt_notify(state);
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
        int val = app_board_get_adc();
        ESP_LOGI(TAG, "ADC: %d", val);
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

void app_main(void){
    app_board_init();

    app_cbs.config_srv = app_ble_mesh_config_server_cb;
    app_cbs.generic_srv = app_ble_mesh_generic_server_cb;
    app_cbs.mqtt = mqtt_event_handler;
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
}
