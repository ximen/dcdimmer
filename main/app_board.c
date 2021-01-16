#include "app_board.h"
#include "app_common.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static ledc_channel_config_t ledc_channel[CHANNEL_NUMBER];
static const gpio_num_t outputs[CHANNEL_NUMBER] = {
		GPIO_NUM_14,
		GPIO_NUM_15,
		GPIO_NUM_16,
		GPIO_NUM_17,
		GPIO_NUM_18,
		GPIO_NUM_19,
};
static TimerHandle_t   reset_timer;
uint8_t last_levels[CHANNEL_NUMBER] = {100};

static void IRAM_ATTR gpio_isr_handler(void* arg){
    if (gpio_get_level(BUTTON_PIN == 0)){
        xTimerStartFromISR(reset_timer, 0);
    } else {
        xTimerStopFromISR(reset_timer, 0);
    }
}

static void reset_timer_cb(TimerHandle_t xTimer){
    app_config_erase();
}

void app_board_init(){
    reset_timer = xTimerCreate("reset_timer", RESET_TIME_MS / portTICK_PERIOD_MS, pdFALSE, NULL, reset_timer_cb);

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
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) NULL);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_0);

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

void app_board_set_level(uint8_t channel, uint8_t level){
    uint8_t max_brightness = 100;
    char element[18];
    sprintf(element, "bright%d_element", channel + 1);
    app_config_getValue(element, int8, &max_brightness);
    ledc_set_fade_with_time(ledc_channel[channel].speed_mode, ledc_channel[channel].channel, level*128*max_brightness/(100*100), FADE_TIME_MS);
    ledc_fade_start(ledc_channel[channel].speed_mode, ledc_channel[channel].channel, LEDC_FADE_WAIT_DONE);
    last_levels[channel] = level;
}

uint8_t app_board_get_level(uint8_t channel){
    char element[18];
    sprintf(element, "bright%d_element", channel + 1);
    uint8_t max_brightness = 100;
    app_config_getValue(element, int8, &max_brightness);
    uint32_t duty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, channel);
    return duty*100*100/(128*max_brightness);        // 7-bit resolution
}

uint16_t app_board_get_adc(){
    return adc1_get_raw(ADC1_CHANNEL_5);
}

void app_board_set_on(uint8_t channel){
    char element[16];
    sprintf(element, "save%d_element", channel + 1);
    bool remember;
    app_config_getValue(element, bool, &remember);
    if(remember){
        queue_value(channel, last_levels[channel]);
    } else {
        queue_value(channel, 100);
    }
}

void app_board_set_off(uint8_t channel){
    queue_value(channel, 0);
}