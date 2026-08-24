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

	void SET_COMPARE(uint32_t value)
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
		if (percent > 100U)
			percent = 100;

		if (percent == 0U)
		{
			SET_COMPARE(0U);
			if (is_started)
			{
				STOP();
				is_started = false;
			}
			return;
		}

		const uint64_t scaled_ticks =
				(static_cast<uint64_t>(pin.htim->Instance->ARR) + 1ULL) * percent;
		const uint32_t ticks = static_cast<uint32_t>(scaled_ticks / 100ULL);
		SET_COMPARE(ticks == 0U ? 0U : (ticks - 1U));

		/* Program CCR before enabling the channel so a stopped output cannot
		 * briefly restart with a stale compare value. */
		if (!is_started)
		{
			START();
			is_started = true;
		}
	}

	void STOP()
	{
		HAL_TIM_PWM_Stop(pin.htim, pin.channel);
	}

	void SET_DIRECT(uint32_t value)
	{
		SET_COMPARE(value);
	}
};

#endif
