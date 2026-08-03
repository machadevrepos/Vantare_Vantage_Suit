/*
 * SWO_PRINTER.h
 *
 *  Created on: Nov 26, 2024
 *      Author: dhanv
 */

#ifndef SWO_PRINTER_H_
#define SWO_PRINTER_H_
#include <stdarg.h>

String NoName = "NoName";

/* CMSIS ITM_SendChar spins on a full ITM FIFO with no timeout, so a host that stops draining
 * SWO (viewer paused, buffer full, probe detached mid-stream) stalls the entire main loop:
 * BLE events back up, record ACKs go out late, and the training CSV service is starved.
 * Trace is a diagnostic and must never be able to hold up acquisition or transfer, so the
 * wait is bounded here and the character is dropped once the budget is spent. */
#ifndef EXO_SWO_TX_SPIN_LIMIT
#define EXO_SWO_TX_SPIN_LIMIT 256U
#endif

inline bool exo_swo_send_char(uint8_t ch)
{
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0UL) || ((ITM->TER & 1UL) == 0UL)) {
        return false; /* trace not enabled: discard without waiting */
    }
    for (uint32_t spin = 0U; spin < EXO_SWO_TX_SPIN_LIMIT; ++spin) {
        if (ITM->PORT[0U].u32 != 0UL) {
            ITM->PORT[0U].u8 = ch;
            return true;
        }
        __NOP();
    }
    return false; /* FIFO still full: drop this character rather than block */
}

class SWO_PRINTER {

	public:
		print_control control;

		SWO_PRINTER() {

		}

		void Print(uint8_t *out, uint32_t out_size, String name = NoName) {
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

		void PRINT_JSON(const cJSON *json, int indent_level = 0, const String &prefix = "")
		{
			String indent(indent_level * 2, ' '); // 2 spaces per indent level
			if (!json) return;

			if (cJSON_IsObject(json)) {
				if (!prefix.empty()) {
					Print("\r\n" + indent + prefix + " : {");
				}
				cJSON *item = json->child;
				while (item) {
					String key = item->string ? item->string : "";
					if (cJSON_IsObject(item) || cJSON_IsArray(item)) {
						PRINT_JSON(item, indent_level + 1, key);
					} else {
						char *value_str = cJSON_PrintUnformatted(item);
						Print("\r\n" + indent + "  " + key + " : " + (value_str ? value_str : ""));
						if (value_str) cJSON_free(value_str);
					}
					item = item->next;
				}
				if (!prefix.empty()) {
					Print("\r\n" + indent + "}");
				}
			} else if (cJSON_IsArray(json)) {
				if (!prefix.empty()) {
					Print("\r\n" + indent + prefix + " : [");
				}
				int index = 0;
				cJSON *item = json->child;
				while (item) {
					String array_prefix = "[" + d_t_s(index++) + "]";
					PRINT_JSON(item, indent_level + 1, array_prefix);
					item = item->next;
				}
				if (!prefix.empty()) {
					Print("\r\n" + indent + "]");
				}
			} else {
				char *value_str = cJSON_PrintUnformatted(json);
				if (!prefix.empty()) {
					Print("\r\n" + indent + prefix + " : " + (value_str ? value_str : ""));
				} else {
					Print("\r\n" + indent + (value_str ? value_str : ""));
				}
				if (value_str) cJSON_free(value_str);
			}
		}
		void Json_print(String json_print, String name_1 = "Json", bool formatted = 1) {

			//			serializeJson(json_ret, json_print);
			if (formatted) {
				for (uint32_t var = 0; var < json_print.size(); var++) {
					//				string char_s = json_print.at(var);
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
			Print("\n\r\n\r" + name_1 + " -->");
			Print(json_print);
			Print("<--" + name_1 + "\n\r");
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
		void Prints(String &out) {
//			PrintForce(d_t_s(is_uart_print(), 0));
			if (control.is_uart_print()) {
				for (uint32_t i = 0; i < out.size(); i++) {
					(void)exo_swo_send_char((uint8_t)out[i]);
				}
			}
		}
		void Print(String out) {
//			PrintForce(d_t_s(is_uart_print(), 0));
			if (control.is_uart_print()) {
				for (uint32_t i = 0; i < out.size(); i++) {
					(void)exo_swo_send_char((uint8_t)out[i]);
				}
			}
		}
		void snprint(const char *format, ...)
				{
			if (!control.is_uart_print()) {
				return;
			}

			char buffer[160] = { 0 };
			va_list args;
			va_start(args, format);
			int written = vsnprintf(buffer, sizeof(buffer), format, args);
			va_end(args);

			if (written <= 0) {
				return;
			}

			uint32_t len = (written >= (int) sizeof(buffer)) ?
					(uint32_t) (sizeof(buffer) - 1U) : (uint32_t) written;
			Print_cstr(buffer, len);
		}
		void Print_cstr(char *out, uint32_t size) {
			if (control.is_uart_print()) {
				for (uint32_t i = 0; i < size; i++) {
					(void)exo_swo_send_char((uint8_t)out[i]);
//					delay_ms(1);
				}
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

};

#endif /* SWO_PRINTER_H_ */
