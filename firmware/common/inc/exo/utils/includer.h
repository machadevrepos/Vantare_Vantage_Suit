/*
 * includer.h - aggregated common include + shared globals for application TUs.
 * Ported from legacy LIBRARY/CUSTOM/INCLUDER.h to the exo/ header layout.
 */

#ifndef EXO_UTILS_INCLUDER_H_
#define EXO_UTILS_INCLUDER_H_

#include "main.h"

#define ALWAYS_PRINT
//#define PRINT_DEBUG

#include <exo/types/enums.h>

volatile LOOP_CONT ota_cont = $CONTINUE;
volatile LOOP_CONT gnss_cont = $CONTINUE;
volatile LOOP_CONT watchdog_cont = $CONTINUE;

#if defined(BLE_ON)
volatile LOOP_CONT ble_cont = $CONTINUE;
#else
volatile LOOP_CONT ble_cont = $BREAK;
#endif

#include <string>
#define String std::string
#include <initializer_list>

#include <exo/utils/string_handler.h>
#include <initializer_list>
#include <exo/utils/dynamic_array.h>
#include <exo/utils/m_usart.h>

#include <exo/utils/cjson.h>

cJSON *esp_doc = NULL;

double analog_value[100];
uint32_t analog_index = 0;
String direction_return = "";

struct Meta {
		double value_double;
		String name = "";
		bool meta_update = 0, data_update = 0;
};

struct Parameter {
		double value_double;
		String value_string;
		String name = "";
};

String config_file_name = "config.json";
#define not_found 0xFFFFFFFF
#define us_s 1000000
#define us_ms 1000
//#define us 1
#define ms_s 1000
#define s_hr 3600

void delay_us(volatile uint32_t del) {
	if (del == 0) {
		return;

	}
	del = (del / 10) * (HAL_RCC_GetHCLKFreq() / 1000000UL);

	for (; del > 0; del--) {

	}
}
void delay_ms(uint32_t ms) {
	delay_us(ms * us_ms);
//	HAL_Delay(ms);
//	vTaskDelay(pdMS_TO_TICKS(ms));
}
void delay_s(uint32_t s) {
	delay_ms(s * ms_s);
}

volatile uint32_t reset_counter = UINT32_MAX;
const uint32_t refresh_value = 300;  // 500 = 1 second

inline void refresh_counter(uint32_t new_value = 0) {
	watchdog_cont = $CONTINUE;
}

#include <exo/types/c_structs.h>

#include <exo/utils/print_control.h>
#include <exo/utils/uart_printer.h>

#include <exo/utils/swo_printer.h>
SWO_PRINTER debug;

//#define PRINT_DEBUG
#if defined(PRINT_DEBUG)
String print_string_1 = " : ";
String print_string_2 = " ";
#define printdebug(a) debug.snprint("%s%s%s%s%s", a.c_str(), print_string_1.c_str(), d_t_s(__LINE__).c_str(), print_string_2.c_str(), "\r\n")
#define printdebugl(a,b) debug.Print(a+print_string_1+d_t_s(b)+print_string_2+"\r\n")
#else
int global_iter = 0;
#define printdebug(a) global_iter++
#define printdebugl(a,b) global_iter++
#endif

#include <exo/utils/json_handler.h>

#include <exo/utils/dig_pin.h>
#include <exo/utils/pwm_pin.h>

DIG_PIN led_b(RGB_B_GPIO_Port, RGB_B_Pin, 0);
DIG_PIN led_r(RGB_R_GPIO_Port, RGB_R_Pin, 0);
DIG_PIN led_g(RGB_G_GPIO_Port, RGB_G_Pin, 0);

#include <exo/utils/rgb_led.h>

RGB_LED rgb(led_r, led_g, led_b, 0);

#include <exo/sensors/analog.h>

#include <exo/utils/softuart.h>

#include <exo/utils/functions.h>

/*	OBJECT DEFINITIONS	*/

#endif /* EXO_UTILS_INCLUDER_H_ */
