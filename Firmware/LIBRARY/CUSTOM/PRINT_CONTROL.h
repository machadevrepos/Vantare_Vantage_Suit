/*
 * PRINT_CONTROL.h
 *
 *  Created on: Sep 29, 2023
 *      Author: dhanveer.singh
 */

#ifndef PRINT_CONTROL_H_
#define PRINT_CONTROL_H_

class print_control {
	public:
#if defined(ALWAYS_PRINT)
volatile bool uart_print = 1;  //!< A bool value representing if Printing is to be done or not as per input password.
volatile bool ble_print = 1;  //!< A bool value representing if Printing is to be done or not as per input password.
#else
		volatile bool uart_print = 0;  //!< A bool value representing if Printing is to be done or not as per input password.
		volatile bool ble_print = 0;  //!< A bool value representing if Printing is to be done or not as per input password.
#endif

		print_control(){

		};

		void set_both_print(bool print_temp) {
			uart_print = print_temp;
			ble_print = print_temp;
		}

		void set_uart_print(bool print_temp) {
			uart_print = print_temp;
		}
		bool is_uart_print() {
			return uart_print;
		}

		void set_ble_print(bool print_temp) {
			ble_print = print_temp;
		}
		bool is_ble_print() {
			return ble_print;
		}

		DynamicArray<bool> ble_print_arr, uart_print_arr;

		void save_ble_print(bool set_now = 0) {
			ble_print_arr.push_back(is_ble_print());
			set_ble_print(set_now);
		}
		void restore_ble_print() {
			set_ble_print(ble_print_arr.pop_back());
		}

		void save_uart_print(bool set_now = 0) {
			uart_print_arr.push_back(is_uart_print());
			set_uart_print(set_now);
		}
		void restore_uart_print() {
			set_uart_print(uart_print_arr.pop_back());
		}

		bool is_any_print() {
			return uart_print || ble_print;
		}
		void save_both_print(bool set_now = 0) {
			save_ble_print(set_now);
			save_uart_print(set_now);
		}
		void restore_both_print() {
			restore_ble_print();
			restore_uart_print();
		}
};
#endif /* PRINT_CONTROL_H_ */
