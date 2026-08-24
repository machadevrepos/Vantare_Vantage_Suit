/*
 * PWR_PIN.h
 *
 *  Created on: May 31, 2023
 *      Author: dhanveer.singh
 */

#ifndef INC_PWR_PIN_H_
#define INC_PWR_PIN_H_
class DIG_PIN {

		bool active_state = 1;
		volatile uint32_t active_edge_time = 0;
		volatile bool curr_state = 0;
		volatile uint32_t pulse_time = 0;
		volatile bool is_changed = 0;

	public:
		struct PWRPIN {
				GPIO_TypeDef *PWRPIN_GPIOx;
				uint16_t PWRPIN_GPIO_PINx;
		};

		DIG_PIN(GPIO_TypeDef *PWRPIN_GPI_Ox, uint16_t PWRPIN_GPIO_PI_Nx, bool active_state = 1) :
				active_state(active_state) {
			SET_PWRPIN(PWRPIN_GPI_Ox, PWRPIN_GPIO_PI_Nx);
		}

		PWRPIN pwrpin;

		void SET_PWRPIN(GPIO_TypeDef *PWRPIN_GPI_Ox, uint16_t PWRPIN_GPIO_PI_Nx) {
			pwrpin.PWRPIN_GPIO_PINx = PWRPIN_GPIO_PI_Nx;
			pwrpin.PWRPIN_GPIOx = PWRPIN_GPI_Ox;
		}

		void Toggle(uint32_t del = 0) {
			HAL_GPIO_TogglePin(pwrpin.PWRPIN_GPIOx, pwrpin.PWRPIN_GPIO_PINx);
			if (del > 0)
				delay_ms(del);
		}

		void SET(bool state = 0, uint32_t del = 0) {
			HAL_GPIO_WritePin(pwrpin.PWRPIN_GPIOx, pwrpin.PWRPIN_GPIO_PINx, (active_state == state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			if (del > 0)
				delay_ms(del);
		}

		void PULSE(bool high_or_low, uint32_t del = 0, uint32_t del2 = 0) {
			SET(high_or_low, del);
			high_or_low = (active_state == high_or_low);
			SET(!high_or_low, del2);
		}

		bool READ() {
			bool state = HAL_GPIO_ReadPin(pwrpin.PWRPIN_GPIOx, pwrpin.PWRPIN_GPIO_PINx);
			// SWO_monitor.Print((active_state == state) ? "Read now: 1\r\n" : "Read now: 0\r\n");
			return active_state == state;
		}

		void INTERRUPT_ROUTINE() {
			if (READ()) {
				active_edge_time = HAL_GetTick();
			}
			UPDATE_STATE();
		}

		void UPDATE_STATE() {
			curr_state = READ();
			// SWO_monitor.Print(curr_state ? "Setting State: 1\r\n" : "Setting State: 0\r\n");
		}

		bool GET_LAST_STATE() {
			return curr_state;
		}

		bool IS_CHANGED() {
			static bool last_state = 0;
			if (curr_state != last_state) {
				last_state = curr_state;
				return true;
			}
			return false;
		}

		uint32_t TIMER_ROUTINE() {
			// SWO_monitor.Print(READ() ? "Curr state: 1\r\n" : "Curr state: 0\r\n");
			if (curr_state != active_state) { // IS_CHANGED() &&
				pulse_time = HAL_GetTick() - active_edge_time;
				if (pulse_time > 0) {
					return pulse_time;
				}
			}
			return 0;
		}

		void UPDATE_PULSE_TIME() {
			if (curr_state == active_state)
				pulse_time = HAL_GetTick() - active_edge_time;
		}

		uint32_t GET_PULSE_TIME() {
			return pulse_time;
		}

		void CONFIG_GPIO(uint32_t Mode, uint32_t Pull, uint32_t Speed, uint32_t Alternate)
				{
			GPIO_InitTypeDef GPIO_InitStruct = { 0 };
			GPIO_InitStruct.Pin = pwrpin.PWRPIN_GPIO_PINx;
			GPIO_InitStruct.Mode = Mode;
			GPIO_InitStruct.Pull = Pull;
			GPIO_InitStruct.Speed = Speed;
			GPIO_InitStruct.Alternate = Alternate;
			HAL_GPIO_Init(pwrpin.PWRPIN_GPIOx, &GPIO_InitStruct);
		}

};

#endif /* INC_PWR_PIN_H_ */
