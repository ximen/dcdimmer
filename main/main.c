/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "app_config.h"
#include "app_config_ble_mesh.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include <stdint.h>
#include "app_common.h"
#include "app_board.h"
#include "app_mqtt.h"

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
        esp_ble_mesh_elem_t *element = esp_ble_mesh_find_element(esp_ble_mesh_get_primary_element_address() + state.channel + 1);
        if(element){
            esp_ble_mesh_model_t *onoff_model = esp_ble_mesh_find_sig_model(element, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            esp_ble_mesh_model_t *level_model = esp_ble_mesh_find_sig_model(element, ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_SRV);
            esp_ble_mesh_server_state_value_t onoff_value;
            esp_ble_mesh_server_state_value_t level_value;
            if (state.state) onoff_value.gen_onoff.onoff = 1; else onoff_value.gen_onoff.onoff = 0;
            level_value.gen_level.level = state.state;
            ESP_LOGI(TAG, "Updating server value. OnOff %d, Level %d", onoff_value.gen_onoff.onoff, level_value.gen_level.level);
            esp_ble_mesh_server_model_update_state(onoff_model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &onoff_value);
            esp_ble_mesh_server_model_update_state(level_model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &level_value);
        }
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

uint8_t get_channel_number(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx){
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (uint8_t i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                return i - 1;
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            return model->element->element_addr - primary_addr - 1;
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        return model->element->element_addr - primary_addr;
    } else {
        ESP_LOGE(TAG, "Error looking for channel number");
    }
    return 0xFF;
} 

static void app_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param){
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT){
        switch (param->ctx.recv_op) {
            case ESP_BLE_MESH_MODEL_OP_NODE_RESET:
                ESP_LOGW(TAG, "Resetting Ble mesh node!");
                esp_ble_mesh_node_local_reset();
                break;
            default:
                break;
        }
        
    }
}

static void app_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event, esp_ble_mesh_generic_server_cb_param_t *param){
esp_ble_mesh_gen_onoff_srv_t *onoff_srv;
esp_ble_mesh_gen_level_srv_t *level_srv;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            uint8_t channel = get_channel_number(param->model, &param->ctx);
            if (channel < CHANNEL_NUMBER) queue_value(channel, param->value.state_change.onoff_set.onoff*100);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET || ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK){
            uint8_t channel = get_channel_number(param->model, &param->ctx);
            uint8_t val = ((param->value.state_change.level_set.level+32767)*100)/65534;
            ESP_LOGI(TAG, "level %d, ch %d, value %d", param->value.state_change.level_set.level, channel, val);
            if (channel < CHANNEL_NUMBER) queue_value(channel, val);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            onoff_srv = param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", onoff_srv->state.onoff);
            uint8_t channel = get_channel_number(param->model, &param->ctx);
            ESP_LOGI(TAG, "Received get, channel %d", channel);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
            level_srv = param->model->user_data;
            ESP_LOGI(TAG, "level %d on channel %d", level_srv->state.level, get_channel_number(param->model, &param->ctx));
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            if (param->value.set.onoff.op_en) {
                ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                    param->value.set.onoff.trans_time, param->value.set.onoff.delay);
                ESP_LOGI(TAG, "Received set");
            }
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
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
