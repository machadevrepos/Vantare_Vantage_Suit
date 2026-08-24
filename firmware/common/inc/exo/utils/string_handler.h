/*
 * double_to_string.h
 *
 *  Created on: May 23, 2023
 *      Author: dhanveer.singh
 */

#ifndef INC_DOUBLE_TO_STRING_H_
#define INC_DOUBLE_TO_STRING_H_

bool compare_str(String &str1, String &str2, uint8_t len)
		{
	return str1.substr(0, len) == str2.substr(0, len);
}

String hex_digits = "0123456789ABCDEF";
String d_t_h_s(uint32_t decimal) {
	String hex;
	uint32_t decimal_2 = decimal;
	if (decimal == 0) {
		hex = "0";
	}
	while (decimal > 0) {
		int digit = decimal % 16;
		hex = hex_digits[digit] + hex;
		decimal /= 16;
	}
	if (decimal_2 < 0xF) {
		hex = "0" + hex;
	}
	return hex;
}

String d_t_s(double value, int precision = 0, bool add_0 = 0) {

	String result;

	// Handle negative values
	if (value < 0) {
		result += "-";
		value = -value;
	}

	// Convert integer part
	unsigned long long intPart = static_cast<unsigned long long>(value);

	if (add_0 && intPart < 10) {
		result += "0";
	}
	result += std::__cxx11::to_string(intPart);

	// Convert decimal part
	if (precision > 0) {
		result += ".";
		double decimalPart = value - intPart;
		for (int i = 0; i < precision; ++i) {
			decimalPart *= 10;
			int digit = static_cast<int>(decimalPart);
			result += std::__cxx11::to_string(digit);
			decimalPart -= digit;
		}
	}
	return result;
}

double s_t_d(const String &str) {
	double result = 0.0;
	double factor = 1.0;
	bool hasDecimal = false;
	int decimalPlaces = 0;
	bool isNegative = false;
	bool hasLeadingZeros = true;

	// Handle negative sign
	if (!str.empty() && str[0] == '-') {
		isNegative = true;
	}

	for (char c : str) {
		if (c == '-') {
			continue;  // Skip the negative sign if present
		} else
		if (c == '.') {
			hasDecimal = true;
		} else
		if (c >= '0' && c <= '9') {
			if (hasDecimal) {
				++decimalPlaces;
				factor /= 10.0;
				result += (c - '0') * factor;
			} else {
				result = result * 10.0 + (c - '0');
			}
			if (hasLeadingZeros && c != '0') {
				hasLeadingZeros = false;
			}
		} else {
			// Invalid character, handle error as needed
			return 0.0;
		}
	}

	if (hasLeadingZeros) {
		return 0.0;  // Input String contains only leading zeros, consider it invalid
	}

	if (isNegative) {
		result = -result;
	}

	return result;
}
String d_t_b_s(uint32_t decimal, uint32_t highlight, uint32_t digits = 32) {
	String binary = "0b";
	if (decimal == 0) {
		binary += "0";
	}
	while (digits--) {
		if (highlight == digits) {
			binary += "(";
		}
		binary += d_t_s((decimal & (0x1 << digits)) > 0);
		if (highlight == digits) {
			binary += ")";
		}
	}

	return binary;
}

uint8_t if_in_range(String &data, uint32_t index) {
	if (index < data.length()) {
		return data.at(index);
	} else
		return 0;
}

uint32_t str_to_uint32(String &data, uint32_t index) {
	uint32_t ret2 = 0;
	for (volatile uint32_t var = 0; var < 4; var++) {
		uint32_t byte = if_in_range(data, index + var);
		ret2 = (ret2 << 8) | byte;
	}
	return ret2;
}
String uint32_to_str(uint32_t value) {
	String result = "";
	for (int32_t var = 3; var >= 0; var--) {
		char byte = (char) ((value >> (8 * var)) & 0xFF);
		if (byte == '\0') {
			continue;
		}
		result += byte;
	}

	return result;
}

const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
String generate_random_string(uint32_t size = 16) {
	String rnd_str;
	size_t charset_size = sizeof(charset) - 1;

	// Seed the random number generator
	srand (HAL_GetTick());

for(	size_t i = 0; i < size; i++) {
		int key = rand() % charset_size;
		rnd_str += charset[key];
	}
	return rnd_str;
}

#endif /* INC_DOUBLE_TO_STRING_H_ */
