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
#include "driver/ledc.h"
#include <stdint.h>

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
#define FADE_TIME_MS    500

ledc_channel_config_t ledc_channel[CHANNEL_NUMBER];
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

uint8_t get_current_value(uint8_t channel){
    uint32_t duty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, channel);
    return duty*100/128;        // 7-bit resolution
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
                        uint8_t current_value = get_current_value(i);
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
            ledc_set_fade_with_time(ledc_channel[item.channel].speed_mode, ledc_channel[item.channel].channel, item.state*128/100, FADE_TIME_MS);
            ledc_fade_start(ledc_channel[item.channel].speed_mode, ledc_channel[item.channel].channel, LEDC_FADE_WAIT_DONE);
            // if (item.state == 0){
            //     gpio_set_level(outputs[item.channel], OFF_LEVEL);
            //     ESP_LOGI(TAG, "Set %d pin to %d", outputs[item.channel], OFF_LEVEL);
            // } else {
            //     gpio_set_level(outputs[item.channel], ON_LEVEL);
            //     ESP_LOGI(TAG, "Set %d pin to %d", outputs[item.channel], ON_LEVEL);
            // }
                
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

void check_reset_pin(){
    if (gpio_get_level(BUTTON_PIN) == 0){
        app_config_erase();
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
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(INA_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SHUTDOWN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADC_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_dis(BUTTON_PIN);
    gpio_pullup_dis(INA_RESET_PIN);
    gpio_pulldown_dis(INA_RESET_PIN);
    gpio_pullup_dis(SHUTDOWN_PIN);
    gpio_pulldown_dis(SHUTDOWN_PIN);
    gpio_pullup_dis(ADC_PIN);
    gpio_pulldown_dis(ADC_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_level(INA_RESET_PIN, 0);
    gpio_set_level(SHUTDOWN_PIN, 1);

    check_reset_pin();
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0);

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

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_7_BIT,  // resolution of PWM duty
        .freq_hz = 2000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    for (uint8_t i=0; i<CHANNEL_NUMBER; i++){
        ledc_channel[i].channel  = i;
        ledc_channel[i].duty     = 0,
        ledc_channel[i].gpio_num = outputs[i];
        ledc_channel[i].speed_mode = LEDC_HIGH_SPEED_MODE;
        ledc_channel[i].hpoint = 0;
        ledc_channel[i].timer_sel = LEDC_TIMER_0;
        ledc_channel_config(&ledc_channel[i]);
    }
    ledc_fade_func_install(0);
}
