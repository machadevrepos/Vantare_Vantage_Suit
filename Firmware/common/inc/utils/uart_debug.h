/**
 * @file UART_DEBUG.h
 * @brief for class definition of DEBUG module to be added for debug with password authentication functionality in AWS2.0
 *
 * 	 Dependencies: DYNAMIC_ARRAY.h
 *   Author: Smile.Guleria
 */
#ifndef INC_UART_DEBUG_H_
#define INC_UART_DEBUG_H_
/**
 * for definition of UART_DEBUG class used to define UART_DEBUG module used in AWS 2.0.
 */
#include <INCLUDER.h>
class UART_DEBUG {

//		DynamicArray<VARIABLES*> *VariablePointer;  //!< A Pointer pointing to Dynamic Array object to types VARIABLES pointer.
	UART_PRINTER *debug;
//	NEOWAY *neoway;
	uint32_t try_count = 3;
	LOOP_CONT both_control = $CONTINUE;
	String resp_string = "";
public:

	UART_DEBUG(DynamicArray<VARIABLES*> *temp_i, UART_PRINTER *debug) :
			debug(debug) {
//			VariablePointer = temp_i;

	}

	void Print2(String out) {


		debug->Print(out);
	}

	void Json_print(String json_print, String name_1 = "Json", bool formatted = 1) {

//			serializeJson(json_ret, json_print);
		if (formatted) {
			for (uint32_t var = 0; var < json_print.size(); var++) {
				//				String char_s = json_print.at(var);
				char char_s = json_print.at(var);
				//				if (char_s == "," || char_s == "{" || char_s == "}") {
				if (char_s == ',' || char_s == '{' || char_s == '}') {
					//					json_print.insert(var, "\n\r", 1);
					json_print.replace(var, 1, "\n\r");
				}
				if (char_s == '\"') {
					//					json_print.insert(var, "\n\r", 1);
					json_print.replace(var, 1, " ");
				}
				if (char_s == ':') {
					//					json_print.insert(var, "\n\r", 1);
					json_print.replace(var, 1, "-");
				}
				//				} else {
				//					Print2(char_s);
				//				}
			}
		}
		Print2("\n\r\n\r" + name_1 + " -->");
		Print2(json_print);
		Print2("<--" + name_1 + "\n\r");
	}

	void Print2(DynamicArray<uint8_t> &temp) {
		debug->Print(temp);
	}

	String convert_dy_str(DynamicArray<uint8_t> &temp) {
		String frame;
		if (temp.is_empty()) {
			frame += "EMPTY FRAME";
		} else {
			for (uint32_t iterator = 0; iterator < temp.size(); iterator++) {
				double temp_var = *(temp.at(iterator));
				frame += d_t_h_s(temp_var) + " ";
			};
		}
		frame += "\n\r";
		return frame;
	}
	void Print2(uint8_t *out, uint32_t out_size, String name) {
		debug->Print(out, out_size, name);
	}

	void Print2(double out, double decimals = 5) {

//			neo2.BLE_SEND(d_t_s(out, decimals));

		debug->Print(d_t_s(out, decimals));
	}


	String Ser_Read(uint32_t a) {
//			Print2("s");
		return debug->Read(a);
	}

	/**
	 * @fn 	void void PassAuthen()
	 * @brief timeout = multiple of 1000
	 */
	String Both_Get_pass(uint32_t timeout_in_s = 5, int *ble_ser = NULL) {
		String uart_in = "", ble_in = "", pass = debug->get_pass();
		for (uint16_t it = timeout_in_s * 5; it > 0; it--) {
#if defined(UL1_ON)
				LED_1.Toggle(1);
#endif
			uart_in = Ser_Read(100);
#if defined(UL1_ON)
				LED_1.Toggle(1);
#endif
//			ble_in = BLE_Read(100);
			if (uart_in == pass) {
				if (ble_ser != NULL) {
					*ble_ser = 1;
				}
				return uart_in;
			}
			if (ble_in == pass) {
				if (ble_ser != NULL) {
					*ble_ser = 2;
				}
				return ble_in;
			}
		}
		return "";
	}

	String Both_Read(uint32_t timeout_in_s = 5) {
		String Uart_resp = "", Ble_resp = "";
		for (uint16_t it = (timeout_in_s == 0 ? 5 : timeout_in_s); it > 0; it--) {
//			refresh_counter();
			Uart_resp = Ser_Read(500);
//			Ble_resp = BLE_Read(500);
			if (!Uart_resp.empty()) {
				return Uart_resp;
			}
			if (!Ble_resp.empty()) {
				return Ble_resp;
			}
			if (timeout_in_s == 0) {
				it++;
			}
		}
		return "";
	}

	NEOWAY_RETURN Both_read_check(String print_this = "", uint32_t timeout_in_s = 5, String ex_resp = "", String *resp = NULL) {
		NEOWAY_RETURN return_check_temp = $N_OK;
		String response;
		do {
			if (print_this != "") {
				Print2("\n\r" + print_this + " : ");
			}

			response = Both_Read(timeout_in_s);

//				Print2(response);

			return_check_temp = CHECK_RESPONSE(&response, &ex_resp);

			if (resp != NULL) {
				*resp = response;
			}
			if (return_check_temp == $EXPECTED_RESPONSE) {
				break;
			} else {
				if (response.empty()) {
					Print2("\tNo Response");
				} else {
					Print2("\tWrong response");
				}
			}
		} while (timeout_in_s == 0);
		return return_check_temp;
	}

	NEOWAY_RETURN CHECK_RESPONSE(String *response, String *ex_resp) {
		if (ex_resp == NULL) {
			return $ex_resp_NULL;
		} else if (*response == *ex_resp || *ex_resp == "") {
			return $EXPECTED_RESPONSE;
		} else if (response->find("ERROR") == not_found) {
			return $ERROR_RESPONSE;
		} else if (response->size() == 0) {
			return $NO_RESPONSE;
		} else {
			return $N_ERROR;
		}
		HAL_Delay(1);
	}
};
#endif
//Procedure to be followed
/* 1. Menu's setup is to be done
 * 2. Menu's PassAuthen is to be done-----> after authentication Configuration display and updation is to be done if necessary
 */
