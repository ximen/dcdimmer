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
#include "driver/gpio.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include <driver/adc.h>
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#define TAG "MAIN"
#define BUTTON_PIN 		GPIO_NUM_0
#define ADC_PIN         GPIO_NUM_33
#define SHUTDOWN_PIN    GPIO_NUM_26
#define INA_RESET_PIN   GPIO_NUM_25
#define CHANNEL_NUMBER  6
#define ACTIVE_LEVEL 	1
#define ON_LEVEL		ACTIVE_LEVEL
#define OFF_LEVEL		!ACTIVE_LEVEL
#define TASK_STACK_SIZE 4096
#define QUEUE_LENGTH    5
#define ADC_PERIOD_MS   100

uint16_t adc_values[1000/ADC_PERIOD_MS];
uint16_t adc_counter;

const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;

const gpio_num_t outputs[CHANNEL_NUMBER] = {
		GPIO_NUM_14,
		GPIO_NUM_15,
		GPIO_NUM_16,
		GPIO_NUM_17,
		GPIO_NUM_18,
		GPIO_NUM_19,
};

app_config_cbs_t app_cbs;

xQueueHandle state_queue;

typedef struct {
    uint8_t channel;
    uint8_t state;
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
        esp_ble_mesh_model_t *model = esp_ble_mesh_find_sig_model(element, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
        esp_ble_mesh_server_state_value_t value = {.gen_onoff.onoff = state.state};
        ESP_LOGI(TAG, "Updating server value. Element: %d, Model: %d", element->element_addr, model->model_idx);
        esp_ble_mesh_server_model_update_state(model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &value);
    }
    if (config_mqtt_enable){
        if(state.channel < CHANNEL_NUMBER){                 // Channel topic
            char *topic = get_mqtt_topic(state.channel);
            char status_topic[58] = {0};
            strncat(status_topic, topic, 50);
            strcat(status_topic, "/status");
            if (strlen(topic) > 0){
                ESP_LOGI(TAG, "Publishing MQTT status. Topic %s, value %d", topic, state.state);
                if(state.state) app_config_mqtt_publish(status_topic, "ON");
                else app_config_mqtt_publish(status_topic, "OFF");
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

static void app_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event, esp_ble_mesh_generic_server_cb_param_t *param){
esp_ble_mesh_gen_onoff_srv_t *srv;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            uint8_t channel = get_channel_number(param->model, &param->ctx);
            if (channel < CHANNEL_NUMBER) queue_value(channel, param->value.state_change.onoff_set.onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv = param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv->state.onoff);
            uint8_t channel = get_channel_number(param->model, &param->ctx);
            ESP_LOGI(TAG, "Received get, channel %d", channel);
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
                if(strlen(topic) > 0){
                    ESP_LOGI(TAG, "Subscribing %s", topic);
                    esp_mqtt_client_subscribe(client, topic, 1);
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
                if(strlen(topic) == 0){
                    ESP_LOGI(TAG, "Empty topic %d", i);
                    continue;
                }
                if(strncmp(event->topic, topic, event->topic_len) == 0){
                    if(strncmp(event->data, "ON", event->data_len) == 0){
                        ESP_LOGI(TAG, "Got ON");
                        queue_value(i, 1);
                    } else if (strncmp(event->data, "OFF", event->data_len) == 0){
                            ESP_LOGI(TAG, "Got OFF");
                        queue_value(i, 0);
                    } else {
                        ESP_LOGW(TAG, "Error parsing payload");
                    }
                    break;
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
        adc_values[adc_counter++] = adc1_get_raw(ADC1_CHANNEL_5);
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
            if (item.state == 0){
                gpio_set_level(outputs[item.channel], OFF_LEVEL);
                ESP_LOGI(TAG, "Set %d pin to %d", outputs[item.channel], OFF_LEVEL);
            } else {
                gpio_set_level(outputs[item.channel], ON_LEVEL);
                ESP_LOGI(TAG, "Set %d pin to %d", outputs[item.channel], ON_LEVEL);
            }
                
            notify(item);
        }
        else {
            ESP_LOGI(TAG, "Queue receive timed out");
        }
        int val = adc1_get_raw(ADC1_CHANNEL_5);
        ESP_LOGI(TAG, "ADC: %d", val);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
void app_main(void){
    for (uint8_t i=0; i<CHANNEL_NUMBER; i++){
        gpio_reset_pin(outputs[i]);
        gpio_intr_disable(outputs[i]);
        gpio_set_direction(outputs[i], GPIO_MODE_OUTPUT);
        gpio_pullup_dis(outputs[i]);
        gpio_pulldown_dis(outputs[i]);
        gpio_set_level(outputs[i], OFF_LEVEL);
    }

    gpio_reset_pin(INA_RESET_PIN);
    gpio_reset_pin(ADC_PIN);
    gpio_reset_pin(SHUTDOWN_PIN);
    gpio_set_direction(INA_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SHUTDOWN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADC_PIN, GPIO_MODE_INPUT);
    gpio_pullup_dis(INA_RESET_PIN);
    gpio_pulldown_dis(INA_RESET_PIN);
    gpio_pullup_dis(SHUTDOWN_PIN);
    gpio_pulldown_dis(SHUTDOWN_PIN);
    gpio_pullup_dis(ADC_PIN);
    gpio_pulldown_dis(ADC_PIN);
    gpio_set_level(INA_RESET_PIN, 0);
    gpio_set_level(SHUTDOWN_PIN, 1);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0);

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
