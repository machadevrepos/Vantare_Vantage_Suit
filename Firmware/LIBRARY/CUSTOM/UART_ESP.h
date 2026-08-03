#ifndef UART_ESP_H
#define UART_ESP_H

enum UART_ESP_Status
{
	$UART_ESP_OK = 0x00,
	$UART_ESP_ERROR = 0x01,
	$UART_ESP_BUSY = 0x02,
	$UART_ESP_TIMEOUT = 0x03
};

enum ESP_COMMANDS
{
	$CARD_1,
	$CARD_2,
	$CARD_3,
	$CARD_CAN_RS_1,
	$CARD_CAN_RS_2,
	$RELAY_1,
	$RELAY_2,
	$FAN_1,
	$RESTART_ESP,
};

class UART_ESP
{

	public:
		UART_ESP(UART_HandleTypeDef *huart)
				{
			this->huart = huart;
		}

		void sendData(const uint8_t *data, uint16_t size)
				{
			HAL_UART_Transmit(huart, (uint8_t*) data, size, HAL_MAX_DELAY);
		}
		uint32_t receiveData(uint8_t *buffer, uint16_t size)
				{
			uint16_t bytes_received = 0;
			HAL_UARTEx_ReceiveToIdle(huart, buffer, size, &bytes_received, 1000);

			return 2;
		}
		void sendStringc(const char *str)
				{
			HAL_UART_Transmit(huart, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
		}
		uint32_t receiveStringc(char *buffer, uint16_t size)
				{
			uint16_t bytes_received = 0;
			HAL_UARTEx_ReceiveToIdle(huart, (uint8_t*) buffer, size, &bytes_received, 100);
			return bytes_received;
		}

		void sendString(const String &str)
				{
			sendStringc(str.c_str());
		}

		void receiveString(String &str)
				{
#define BUFFER_SIZE 1024
			char *buffer = new char[BUFFER_SIZE];
			receiveStringc(buffer, BUFFER_SIZE);
			buffer[BUFFER_SIZE] = '\0'; // Null-terminate the string
			str = String(buffer);
			delete[] buffer;
		}

	private:
		UART_HandleTypeDef *huart;
};

UART_ESP ESP32(&huart3);

#endif // UART_ESP_H
