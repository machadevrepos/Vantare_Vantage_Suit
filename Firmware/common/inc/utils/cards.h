/*
 * CARDS.h
 *
 *  Created on: Oct 22, 2025
 *      Author: dhanv
 */

#ifndef INC_CUSTOM_CARDS_H_
#define INC_CUSTOM_CARDS_H_

typedef enum
{
	$PIN_MODE_DIGITAL_INPUT,
	$PIN_MODE_DIGITAL_OUTPUT,
	$PIN_MODE_PWM,
	$PIN_ANALOG_INPUT,
	$PIN_MODE_CURRENT_INPUT
} PinMode_t;

typedef struct
{
		uint32_t value;
		PWM_PIN *pwm_pin;
		DIG_PIN *dig_pin;
		ANALOG *analog_pin;
} Pin_value_t;

typedef struct
{
		PinMode_t config;
		Pin_value_t pins[4];
} CardConfig_t;

PWM_PIN C21_D1_PWM( { &htim3, TIM_CHANNEL_4, C21_D1_GPIO_Port, C21_D1_Pin });
PWM_PIN C21_D2_PWM( { &htim3, TIM_CHANNEL_3, C21_D2_GPIO_Port, C21_D2_Pin });
PWM_PIN C21_D3_PWM( { &htim3, TIM_CHANNEL_2, C21_D3_GPIO_Port, C21_D3_Pin });
PWM_PIN C21_D4_PWM( { &htim3, TIM_CHANNEL_1, C21_D4_GPIO_Port, C21_D4_Pin });

PWM_PIN C22_D1_PWM( { &htim4, TIM_CHANNEL_4, C22_D1_GPIO_Port, C22_D1_Pin });
PWM_PIN C22_D2_PWM( { &htim4, TIM_CHANNEL_3, C22_D2_GPIO_Port, C22_D2_Pin });
PWM_PIN C22_D3_PWM( { &htim4, TIM_CHANNEL_2, C22_D3_GPIO_Port, C22_D3_Pin });
PWM_PIN C22_D4_PWM( { &htim4, TIM_CHANNEL_1, C22_D4_GPIO_Port, C22_D4_Pin });

PWM_PIN C23_D1_PWM( { &htim2, TIM_CHANNEL_4, C23_D1_GPIO_Port, C23_D1_Pin });
PWM_PIN C23_D2_PWM( { &htim1, TIM_CHANNEL_4, C23_D2_GPIO_Port, C23_D2_Pin });
PWM_PIN C23_D3_PWM( { &htim1, TIM_CHANNEL_2, C23_D3_GPIO_Port, C23_D3_Pin });
PWM_PIN C23_D4_PWM( { &htim1, TIM_CHANNEL_3, C23_D4_GPIO_Port, C23_D4_Pin });

DIG_PIN C21_D1_DIG(C21_D1_GPIO_Port, C21_D1_Pin);
DIG_PIN C21_D2_DIG(C21_D2_GPIO_Port, C21_D2_Pin);
DIG_PIN C21_D3_DIG(C21_D3_GPIO_Port, C21_D3_Pin);
DIG_PIN C21_D4_DIG(C21_D4_GPIO_Port, C21_D4_Pin);

DIG_PIN C22_D1_DIG(C22_D1_GPIO_Port, C22_D1_Pin);
DIG_PIN C22_D2_DIG(C22_D2_GPIO_Port, C22_D2_Pin);
DIG_PIN C22_D3_DIG(C22_D3_GPIO_Port, C22_D3_Pin);
DIG_PIN C22_D4_DIG(C22_D4_GPIO_Port, C22_D4_Pin);

DIG_PIN C23_D1_DIG(C23_D1_GPIO_Port, C23_D1_Pin);
DIG_PIN C23_D2_DIG(C23_D2_GPIO_Port, C23_D2_Pin);
DIG_PIN C23_D3_DIG(C23_D3_GPIO_Port, C23_D3_Pin);
DIG_PIN C23_D4_DIG(C23_D4_GPIO_Port, C23_D4_Pin);

ANALOG C21_A1(&hadc1, ADC_CHANNEL_1, ADC_REGULAR_RANK_1);
ANALOG C21_A2(&hadc1, ADC_CHANNEL_2, ADC_REGULAR_RANK_2);
ANALOG C21_A3(&hadc1, ADC_CHANNEL_3, ADC_REGULAR_RANK_3);
ANALOG C21_A4(&hadc1, ADC_CHANNEL_4, ADC_REGULAR_RANK_4);

ANALOG C22_A1(&hadc1, ADC_CHANNEL_13, ADC_REGULAR_RANK_5);
ANALOG C22_A2(&hadc1, ADC_CHANNEL_12, ADC_REGULAR_RANK_6);
ANALOG C22_A3(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_7);
ANALOG C22_A4(&hadc1, ADC_CHANNEL_10, ADC_REGULAR_RANK_8);

ANALOG C23_A1(&hadc1, ADC_CHANNEL_17, ADC_REGULAR_RANK_9);
ANALOG C23_A2(&hadc1, ADC_CHANNEL_16, ADC_REGULAR_RANK_10);
ANALOG C23_A3(&hadc1, ADC_CHANNEL_15, ADC_REGULAR_RANK_11);
ANALOG C23_A4(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_12);

ANALOG vrefint_adc(&hadc1, ADC_CHANNEL_VREFINT);

double ang_read[16] = { 0 };
uint8_t data_send[3][17];
uint8_t data_length[3] = { 0 };

ANALOG *sensors[] = {
		&C21_A1,
		&C21_A2,
		&C21_A3,
		&C21_A4,
		&C22_A1,
		&C22_A2,
		&C22_A3,
		&C22_A4,
		&C23_A1,
		&C23_A2,
		&C23_A3,
		&C23_A4,
};

bool digital_status[12] = { 0 };

CardConfig_t pin_configs[3] = {
		{ $PIN_MODE_DIGITAL_INPUT, {
				{ 0, &C21_D1_PWM, &C21_D1_DIG, &C21_A1 },
				{ 0, &C21_D2_PWM, &C21_D2_DIG, &C21_A2 },
				{ 0, &C21_D3_PWM, &C21_D3_DIG, &C21_A3 },
				{ 0, &C21_D4_PWM, &C21_D4_DIG, &C21_A4 }
		} },
		{ $PIN_MODE_PWM, {
				{ 1, &C22_D1_PWM, &C22_D1_DIG, &C22_A1 },
				{ 1, &C22_D2_PWM, &C22_D2_DIG, &C22_A2 },
				{ 1, &C22_D3_PWM, &C22_D3_DIG, &C22_A3 },
				{ 1, &C22_D4_PWM, &C22_D4_DIG, &C22_A4 }
		} },
		{ $PIN_ANALOG_INPUT, {
				{ 0, &C23_D1_PWM, &C23_D1_DIG, &C23_A1 },
				{ 0, &C23_D2_PWM, &C23_D2_DIG, &C23_A2 },
				{ 0, &C23_D3_PWM, &C23_D3_DIG, &C23_A3 },
				{ 0, &C23_D4_PWM, &C23_D4_DIG, &C23_A4 }
		} }
};

enum PIN_NUMBER
{
	$C21_D1_N = 0,
	$C21_D2_N,
	$C21_D3_N,
	$C21_D4_N,
	$C22_D1_N,
	$C22_D2_N,
	$C22_D3_N,
	$C22_D4_N,
	$C23_D1_N,
	$C23_D2_N,
	$C23_D3_N,
	$C23_D4_N
};

void PIN_SET_MODE(uint8_t card_index, uint8_t pin_index, PinMode_t mode)
		{
	if (card_index >= 3 || pin_index >= 4)
		return;

	CardConfig_t *card = &pin_configs[card_index];
	Pin_value_t *pin = &card->pins[pin_index];

	card->config = mode;

	switch (mode)
	{
		case $PIN_MODE_DIGITAL_INPUT:
			pin->dig_pin->CONFIG_GPIO(GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_HIGH, 0);
			break;

		case $PIN_MODE_DIGITAL_OUTPUT:
			pin->dig_pin->CONFIG_GPIO(GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_HIGH, 0);
			break;

		case $PIN_MODE_PWM:
			pin->pwm_pin->CONFIG_GPIO();
			pin->pwm_pin->START();
			break;
		default:
			// No specific configuration needed for analog or current input
			break;
	}
}

bool PIN_DIGITAL_WRITE(uint8_t card_index, uint8_t pin_index, bool state)
		{
	if (card_index >= 3 || pin_index >= 4)
		return 0;

	pin_configs[card_index].pins[pin_index].dig_pin->SET(state);
	return 1;
}

bool PIN_DIGITAL_READ(uint8_t card_index, uint8_t pin_index)
		{
	if (card_index >= 3 || pin_index >= 4)
		return false;

	return pin_configs[card_index].pins[pin_index].dig_pin->READ();
}

bool PIN_PWM_SET_PERCENT(uint8_t card_index, uint8_t pin_index, uint8_t percent)
		{
	if (card_index >= 3 || pin_index >= 4)
		return 0;

	pin_configs[card_index].pins[pin_index].pwm_pin->SET_PERCENT(percent);
	return 1;
}

void PWM_INIT()
{
	HAL_TIM_Base_Start (&htim2);
	HAL_TIM_Base_Start (&htim3);
	HAL_TIM_Base_Start (&htim1);
}

void ANALOG_INIT()
{
	/*
	 HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	 uint32_t vrefint_raw = vrefint_adc.READ(); // Read the raw value from the VREFINT channel
	 uint32_t vdda_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(&hadc1, vrefint_raw, ADC_RESOLUTION_12B);
	 double voltage_reference = ((double) vdda_voltage / 1000.0) / 16384.0; // Convert to volts
	 //	double voltage_reference = 3.3 / 16384.0; // Convert to volts
	 vrefint_adc.set_config((voltage_reference), 0, 0);

	 double vrefint = vrefint_adc.CONVERT();
	 ANALOG::SET_VERF_INT(vdda_voltage);
	 //	 */

	C22_A1.SET_factor(10 / (3.3 * 16384.0));
	C22_A2.SET_factor(10 / (3.3 * 16384.0));
	C22_A3.SET_factor(10 / (3.3 * 16384.0));
	C22_A4.SET_factor(10 / (3.3 * 16384.0));

	ANALOG::INIT_DMA(&hadc1, sensors, 12);
	ANALOG::READ_DMA_MULTI_VOLTAGE_NULL(12);
}

volatile bool reconfig_pins = true;

void PIN_MODE_DEMO()
{
	if (reconfig_pins)
	{
		for (uint8_t card_index = 0; card_index < 3; card_index++)
				{
			for (uint8_t pin_index = 0; pin_index < 4; pin_index++)
					{
				uint8_t global_pin = card_index * 4 + pin_index;

				PIN_SET_MODE(card_index, pin_index, pin_configs[card_index].config);
				bool state = false;
				switch (pin_configs[card_index].config)
				{
					case $PIN_MODE_PWM:
						if (PIN_PWM_SET_PERCENT(card_index, pin_index, pin_configs[card_index].pins[pin_index].value))
								{
							debug.snprint("Pin %d set to PWM mode at %d duty cycle\r\n", global_pin + 1, pin_configs[card_index].pins[pin_index].value);
						}
						break;
					case $PIN_MODE_DIGITAL_OUTPUT:
						if (PIN_DIGITAL_WRITE(card_index, pin_index, ((pin_configs[card_index].pins[pin_index].value > 0) ? 1 : 0)))
								{
							debug.snprint("Pin %d set to Digital Output mode\r\n", global_pin + 1);
						}
						break;
					case $PIN_MODE_DIGITAL_INPUT:
						state = PIN_DIGITAL_READ(card_index, pin_index);
						debug.snprint("Pin %d state: %d\r\n", global_pin + 1, state);
						break;
					case $PIN_ANALOG_INPUT:
						debug.snprint("Pin %d set to Analog Input mode\r\n", global_pin + 1);
						break;
					case $PIN_MODE_CURRENT_INPUT:
						debug.snprint("Pin %d set to Current Input mode\r\n", global_pin + 1);
						break;
					default:
						debug.snprint("Pin %d has unknown mode\r\n", global_pin + 1);
						break;
				}
			}
		}
		reconfig_pins = false;
	}
}

void uint32_t_to_bytes(uint32_t value, uint8_t *bytes)
		{
	bytes[0] = (value >> 24) & 0xFF;
	bytes[1] = (value >> 16) & 0xFF;
	bytes[2] = (value >> 8) & 0xFF;
	bytes[3] = value & 0xFF;
}

void CARD_LOOP(uint8_t data_send[3][17], uint8_t data_length[3])
		{
	if (data_send == NULL)
		return;

	ANALOG::GET_READINGS(12);

	for (uint8_t card_index = 0; card_index < 3; card_index++)
			{
//		volatile uint8_t data_length = 0;

		data_send[card_index][0] = card_index + 1;
		switch (pin_configs[card_index].config)
		{
			case $PIN_MODE_DIGITAL_INPUT:
				data_length[card_index] = 5;
				break;
			case $PIN_MODE_DIGITAL_OUTPUT:
				data_length[card_index] = 5;
				data_send[card_index][1] = 1;
				break;
			case $PIN_MODE_PWM:
				data_length[card_index] = 2;
				data_send[card_index][1] = 1;
				break;
			case $PIN_MODE_CURRENT_INPUT:
				data_length[card_index] = 17;
				break;
			case $PIN_ANALOG_INPUT:
				data_length[card_index] = 17;
				break;
			default:
				data_length = 0;
				break;
		}

		for (uint8_t pin_index = 0; pin_index < 4; pin_index++)
				{
			Pin_value_t *pin = &pin_configs[card_index].pins[pin_index];

			switch (pin_configs[card_index].config)
			{
				case $PIN_ANALOG_INPUT:
					pin->value = (uint32_t) pin->analog_pin->GET_last_raw();
					uint32_t_to_bytes(pin->value, &data_send[card_index][1 + pin_index * 4]);
					break;
				case $PIN_MODE_CURRENT_INPUT:
					pin->value = (uint32_t) pin->analog_pin->GET_last_raw();
					uint32_t_to_bytes(pin->value, &data_send[card_index][1 + pin_index * 4]);
					break;
				case $PIN_MODE_DIGITAL_INPUT:
					pin->value = pin->dig_pin->READ() ? 1 : 0;
					data_send[card_index][1 + pin_index] = (uint8_t) pin->value;
					break;
				case $PIN_MODE_PWM:
					pin->pwm_pin->SET_PERCENT(pin->value);
					break;
				case $PIN_MODE_DIGITAL_OUTPUT:
					pin->dig_pin->SET(pin->value > 0 ? 1 : 0);
					break;
			}

		}
	}
}
/*
 "Cards": [
 {
 Modes
 I : Input
 O : Output
 A : Analog
 P : PWM
 C : Current

 "mode": "I",

 Pin configs
 m : multiplication factor
 a : offset addition
 r : Raw offset
 tu: Threshold upper
 tl: Threshold lower

 "P": [
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 }
 ]
 },
 {
 "mode": "O",
 "P": [
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 }
 ]
 },
 {
 "mode": "A",
 "P": [
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 },
 { "m": 1.0, "a": 0.0, "r": 0.0, "tu": 0.0, "tl": 0.0 }
 ]
 }
 ]*/

void CARD_LOOP_JSON() {

	ANALOG::GET_READINGS(12);

	for (uint8_t card_index = 0; card_index < 3; card_index++)
			{

		for (uint8_t pin_index = 0; pin_index < 4; pin_index++)
				{
			Pin_value_t *pin = &pin_configs[card_index].pins[pin_index];

			switch (pin_configs[card_index].config)
			{
				case $PIN_ANALOG_INPUT:
					pin->value = (uint32_t) pin->analog_pin->GET_last_raw();

					break;
				case $PIN_MODE_CURRENT_INPUT:
					pin->value = (uint32_t) pin->analog_pin->GET_last_raw();
					uint32_t_to_bytes(pin->value, &data_send[card_index][1 + pin_index * 4]);
					break;
				case $PIN_MODE_DIGITAL_INPUT:
					pin->value = pin->dig_pin->READ() ? 1 : 0;
					data_send[card_index][1 + pin_index] = (uint8_t) pin->value;
					break;
				case $PIN_MODE_PWM:
					pin->pwm_pin->SET_PERCENT(pin->value);
					break;
				case $PIN_MODE_DIGITAL_OUTPUT:
					pin->dig_pin->SET(pin->value > 0 ? 1 : 0);
					break;
			}

		}
	}
}

#endif /* INC_CUSTOM_CARDS_H_ */
