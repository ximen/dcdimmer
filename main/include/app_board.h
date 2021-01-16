#ifndef _APP_BOARD_H_
#define _APP_BOARD_H_

#include <stdint.h>

#define BUTTON_PIN 		GPIO_NUM_0
#define ADC_PIN         GPIO_NUM_33
#define SHUTDOWN_PIN    GPIO_NUM_26
#define INA_RESET_PIN   GPIO_NUM_25
#define CHANNEL_NUMBER  6
#define ACTIVE_LEVEL 	1
#define ON_LEVEL		ACTIVE_LEVEL
#define OFF_LEVEL		!ACTIVE_LEVEL
#define FADE_TIME_MS    500
#define RESET_TIME_MS   3000

void app_board_init();
void app_board_set_level(uint8_t channel, uint8_t level);
void app_board_set_on(uint8_t channel);
void app_board_set_off(uint8_t channel);
uint8_t app_board_get_level(uint8_t channel);
uint16_t app_board_get_adc();

#endif // _APP_BOARD_H_