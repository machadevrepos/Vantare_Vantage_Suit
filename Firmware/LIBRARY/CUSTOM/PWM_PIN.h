#ifndef PWM_PIN_H
#define PWM_PIN_H

#include "main.h"

typedef struct
{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef *GPIOx;
	uint16_t Pin;
} PWM_PIN_TypeDef;

class PWM_PIN
{
	PWM_PIN_TypeDef pin;
	bool is_started = false;

public:
	PWM_PIN(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *GPIOx, uint16_t Pin) : pin({htim, channel, GPIOx, Pin})
	{
	}

	void CONFIG_GPIO()
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = pin.Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

		if (pin.htim->Instance == TIM1)
		{
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		}

		HAL_GPIO_Init(pin.GPIOx, &GPIO_InitStruct);
	}

	void START()
	{
		HAL_TIM_PWM_Start(pin.htim, pin.channel);
	}

	void SET_PERCENT(uint8_t percent)
	{
		if (percent > 99)
			percent = 100;

		if (percent == 0 && is_started)
		{
			STOP();
			is_started = false;
			return;
		}

		else if (!is_started && percent > 0)
		{
			START();
			is_started = true;
		}

		switch (pin.channel)
		{
		case TIM_CHANNEL_1:
			pin.htim->Instance->CCR1 = (((pin.htim->Instance->ARR + 1) * percent) / 100) - 1;
			break;
		case TIM_CHANNEL_2:
			pin.htim->Instance->CCR2 = (((pin.htim->Instance->ARR + 1) * percent) / 100) - 1;
			break;
		case TIM_CHANNEL_3:
			pin.htim->Instance->CCR3 = (((pin.htim->Instance->ARR + 1) * percent) / 100) - 1;
			break;
		case TIM_CHANNEL_4:
			pin.htim->Instance->CCR4 = (((pin.htim->Instance->ARR + 1) * percent) / 100) - 1;
			break;
		}
	}

	void STOP()
	{
		HAL_TIM_PWM_Stop(pin.htim, pin.channel);
	}

	void SET_DIRECT(uint32_t value)
	{
		switch (pin.channel)
		{
		case TIM_CHANNEL_1:
			pin.htim->Instance->CCR1 = value;
			break;
		case TIM_CHANNEL_2:
			pin.htim->Instance->CCR2 = value;
			break;
		case TIM_CHANNEL_3:
			pin.htim->Instance->CCR3 = value;
			break;
		case TIM_CHANNEL_4:
			pin.htim->Instance->CCR4 = value;
			break;
		}
	}
};

#endif
