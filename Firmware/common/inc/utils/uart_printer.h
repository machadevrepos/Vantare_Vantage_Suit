/*
 * UART_PRINTER.h
 *
 *  Created on: Aug 21, 2023
 *      Author: dhanveer.singh
 */

#ifndef UART_PRINTER_H_
#define UART_PRINTER_H_

class UART_PRINTER {

		UART_HandleTypeDef *uptr;  //!< A Pointer pointing to UART_HandleTypeDef object.
		String Pass;  //!< A String value representing intended password
	public:
		print_control control;

		UART_PRINTER(UART_HandleTypeDef *uptr_i, String Pass_i) {

			uptr = uptr_i;
			Pass = Pass_i;
		}

		void Print(uint8_t *out, uint32_t out_size, String name) {
			if (control.is_uart_print()) {
				Print("\n\rARR - " + name + " ");
				for (uint32_t var = 0; var < out_size; var++) {
					Print(d_t_h_s(*(out + var)));
					Print(" ");
				}
				Print("\n\r");
			}
		}
		void Print(uint16_t out, uint32_t out_size, String name) {
			if (control.is_uart_print()) {
				Print("\n\rARR - " + name + " ");
				for (uint32_t var = 0; var < out_size; var++) {
					Print((out + var));
					Print(" ");
				}
				Print("\n\r");
			}
		}
		/**
		 * @fn 	String Read()
		 * @brief To take input String from user through UART protocol
		 */
		String Read(uint32_t timeout) {
			//To ask user to enter value to be changed
			String read_string;
			if (control.is_uart_print()) {
				HAL_UART_Init(uptr);
				UART_ReceiveStringToIdle(uptr, read_string, { timeout });
			}
			while (read_string.back() == '\n' || read_string.back() == '\r') {
				read_string.pop_back();
			}
			return read_string;
		}

		/**
		 * @fn 	void Print(double out)
		 * @brief To Print Data into serial monitor with input as a double value.
		 */
		void Print(double out, double decimals = 5) {
			if (control.is_uart_print()) {
				Print(d_t_s(out, decimals));
			}
		}
		/**
		 * @fn 	void Print(String out = NULL)
		 * @brief To Print Data into serial monitor with input as a String value.
		 */
		void Print(String out = "Empty String") {
//			PrintForce(d_t_s(is_uart_print(), 0));
			if (control.is_uart_print()) {
//				HAL_UART_Init(uptr);
				HAL_UART_Transmit(uptr, (uint8_t*) out.c_str(), out.size(), UINT32_MAX);

			}
		}
		void Print_str(char *out, uint32_t size) {
			if (control.is_uart_print()) {
//				HAL_UART_Init(uptr);
				HAL_UART_Transmit(uptr, (uint8_t*) out, size, UINT32_MAX);
			}
		}

		/**
		 * @fn 	void Print(DynamicArray<uint8_t> temp)
		 * @brief To Print Data into serial monitor with input as a Dynamic Array object.
		 */

		void Print(DynamicArray<uint8_t> &temp) {
			if (control.is_uart_print()) {
				if (temp.is_empty()) {
					Print("EMPTY FRAME");
				} else {
					for (uint32_t iterator = 0; iterator < temp.size(); iterator++) {
						double temp_var = *(temp.at(iterator));
						Print(d_t_h_s(temp_var));
						Print(" ");
					};
				}
				Print("\n\r");
			}
		}

		void Raw(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
			uint8_t out[4] = { b1, b2, b3, b4 };
			HAL_UART_Transmit(uptr, (uint8_t*) out, 4, UINT32_MAX);
		}
		void Raw(uint8_t b1, uint8_t b2) {
			uint8_t out[4] = { b1, b2 };
			HAL_UART_Transmit(uptr, (uint8_t*) out, 2, UINT32_MAX);
		}

		void Raw_arr(uint8_t *arr, uint32_t size) {
			HAL_UART_Transmit(uptr, arr, size, UINT32_MAX);

		}

		String get_pass() {
			return Pass;
		}

};

#endif /* UART_PRINTER_H_ */
