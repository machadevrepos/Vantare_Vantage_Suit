/*
 * includer.h
 *
 *  Created on: Apr 19, 2023
 *      Author: dhanveer.singh
 */

#ifndef INC_INCLUDER_H_
#define INC_INCLUDER_H_

#include "main.h"

#define ALWAYS_PRINT
//#define PRINT_DEBUG

#include <ENUMS.h>

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

#include <STRING_HANDLER.h>
#include <initializer_list>
#include <DYNAMIC_ARRAY.h>
#include <M_USART.h>

//#include <../ArduinoJsonSTM/ArduinoJson.h>
#include <cJSON.h>

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
//	if (watchdog_cont == $CONTINUE) {
//		HAL_IWDG_Refresh (&hiwdg);
//	} else {

//	}
	//	if (new_value == 0) {
//		reset_counter = refresh_value;
//	} else {
//		reset_counter = new_value;
//	}
}

#include <C_STRUCTS.h>

#include <PRINT_CONTROL.h>
#include <UART_PRINTER.h>

#include <SWO_PRINTER.h>
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

#include "JSON_HANDLER.h"
//#include "EEPROM_Emul/EEPROM.h"

//#include <FatFs/fatfs.h>
//#include <SD_CARD.h>

//#include "UART_ESP.h"

#include "DIG_PIN.h"
#include "PWM_PIN.h"

DIG_PIN led_b(RGB_B_GPIO_Port, RGB_B_Pin, 0);
DIG_PIN led_r(RGB_R_GPIO_Port, RGB_R_Pin, 0);
DIG_PIN led_g(RGB_G_GPIO_Port, RGB_G_Pin, 0);

#include "RGB_LED.h"

RGB_LED rgb(led_r, led_g, led_b, 0);

#include <ANALOG.h>

//#include <RS485.h>
//DIG_PIN rs485_1_rede(RS485_1_DE_GPIO_Port, RS485_1_DE_Pin, 1);
//DIG_PIN rs485_2_rede(RS485_2_DE_GPIO_Port, RS485_2_DE_Pin, 1);

//RS485 rs485_1(&huart1, rs485_1_rede, USART1_IRQn);
//RS485 rs485_2(&huart2, rs485_2_rede, USART2_IRQn);

//RS485 esp_rs485(&huart3, rs485_2_rede, USART3_IRQn);

//#include <SETTINGS.h>

//#include <CAN.h>

//CAN can1(&hfdcan1);

//#include <MODBUS/Modbus.h>



#include<softuart.h>

//#include <NEOWAY.h>
//NEOWAY neoway(&huart4);

//#include "CARDS.h"



#include"FUNCTIONS.h"

/*	OBJECT DEFINITIONS	*/

#endif /* INC_INCLUDER_H_ */

