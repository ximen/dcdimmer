/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "app_config.h"
#include "app_config_mqtt.h"
#include "app_config_ble_mesh.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include <stdint.h>
#include "app_board.h"

#define TAG "MAIN"
#define TASK_STACK_SIZE 4096
#define QUEUE_LENGTH    5
#define ADC_PERIOD_MS   100

uint16_t adc_values[1000/ADC_PERIOD_MS];
uint16_t adc_counter;

const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;

app_config_cbs_t app_cbs;

xQueueHandle state_queue;

typedef struct {
    uint8_t channel;        // Channel number. Values over CHACHANNEL_NUMBER are for current sensor
    uint8_t state;          // New state in percent
} queue_value_t;

void queue_value(uint8_t channel, uint8_t value){
    queue_value_t item;
    item.channel = channel;
    item.state = value;
    portBASE_TYPE status = xQueueSend(state_queue, &item, xTicksToWait);
    if (status != pdPASS){
        ESP_LOGE(TAG, "Error queuing state");
    }
}

char *get_mqtt_topic(uint8_t channel){
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
        if(state.channel < CHANNEL_NUMBER){                 // Channel topic
            char *topic = get_mqtt_topic(state.channel);
            char status_topic[58] = {0};
            char brightness_status_topic[58] = {0};
            strncat(status_topic, topic, 50);
            strncat(brightness_status_topic, topic, 50);
            strcat(status_topic, "/status");
            strcat(brightness_status_topic, "/brightness/status");
            if (strlen(topic) > 0){
                ESP_LOGI(TAG, "Publishing MQTT status. Topic %s, value %d", topic, state.state);
                if(state.state) app_config_mqtt_publish(status_topic, "ON");
                else app_config_mqtt_publish(status_topic, "OFF");
                char brightness_value[4];
                sprintf(brightness_value, "%3d", state.state);
                ESP_LOGI(TAG, "Publishing MQTT brightness status. Topic %s, value %s", brightness_status_topic, brightness_value);
                app_config_mqtt_publish(brightness_status_topic, brightness_value);
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
                app_config_mqtt_publish(current_topic, current_value);
            }

        }
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

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id) {
            case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            for(uint8_t i=0; i<CHANNEL_NUMBER; i++){
                char *topic = get_mqtt_topic(i);
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
                char *topic = get_mqtt_topic(i);
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

static void worker_task( void *pvParameters ){
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

void app_main(void){
    app_board_init();

    app_cbs.config_srv = app_ble_mesh_config_server_cb;
    app_cbs.generic_srv = app_ble_mesh_generic_server_cb;
    app_cbs.mqtt = mqtt_event_handler;
    ESP_ERROR_CHECK(app_config_init(&app_cbs));
    ESP_LOGI(TAG, "Creating worker queue");
    state_queue = xQueueCreate(QUEUE_LENGTH, sizeof(queue_value_t));
    if(!state_queue){
        ESP_LOGE(TAG, "Error creating queue");
        return;
    }
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
