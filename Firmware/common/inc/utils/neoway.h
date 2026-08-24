/**
 *@file NEOWAY.h
 *@brief for class definition of NEOWAY module used in AWS 2.0 system.
 *  Created on: May 18, 2023
 *      Author: dhanveer.singh
 */

#ifndef INC_NEOWAY_H_
#define INC_NEOWAY_H_

#include <INCLUDER.h>
#include <M_USART.h>
#include <initializer_list>
#include <PRINT_CONTROL.h>
#include <ENUMS.h>
#include <stdint.h>
#include <string>
#define String std::string
#include "stm32wbxx_hal.h"

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

#define NEOWAY_AT_MODDE_ENABLE 0
#if NEOWAY_AT_MODDE_ENABLE
// UART Mode Configuration - Choose one mode
#define UART_BLOCKING_MODE // Default blocking mode (original behavior)
// #define UART_INTERRUPT_MODE // Interrupt-based UART communication
//  #define UART_DMA_MODE          // DMA-based UART communication

// If no mode is defined, default to blocking mode
#if !defined(UART_BLOCKING_MODE) && !defined(UART_INTERRUPT_MODE) && !defined(UART_DMA_MODE)
#define UART_BLOCKING_MODE
#endif

/*
 * ==========================================
 * AVAILABLE FUNCTIONS BY MODE:
 * ==========================================
 *
 * BLOCKING MODE (UART_BLOCKING_MODE):
 * - SEND_RECIEVE() - Original blocking send/receive function
 * - SEND_RECIEVE_POINTER() - Original blocking send/receive with pointer
 * - All existing AWS, BLE, and power control functions
 *
 * INTERRUPT MODE (UART_INTERRUPT_MODE):
 * - All BLOCKING MODE functions plus:
 * - INIT_UART_INTERRUPT() - Initialize interrupt mode
 * - UART_SEND_IT() - Send data using interrupts
 * - SEND_RECEIVE_IT() - Send/receive using interrupts
 * - UART_DATA_AVAILABLE() - Check for new data
 * - UART_GET_RECEIVED_DATA() - Get received data
 * - RESET_UART_INTERRUPT() - Reset interrupt buffers
 * - DEINIT_UART_INTERRUPT() - Deinitialize interrupt mode
 * - RESET_UART_BUFFERS() - Common buffer reset function
 *
 * DMA MODE (UART_DMA_MODE):
 * - All BLOCKING MODE functions plus:
 * - INIT_UART_DMA_INTERRUPT() - Initialize DMA mode
 * - UART_SEND_DMA() - Send data using DMA
 * - SEND_RECEIVE_DMA() - Send/receive using DMA
 * - UART_DATA_AVAILABLE() - Check for new data
 * - UART_GET_RECEIVED_DATA() - Get received data
 * - RESET_UART_DMA() - Reset DMA buffers
 * - DEINIT_UART_DMA() - Deinitialize DMA mode
 * - RESET_UART_BUFFERS() - Common buffer reset function
 */

struct Aws_Config
{
	String host = "a1q2r34gpscepz-ats.iot.us-west-2.amazonaws.com";
	String client_id = "NEOWAY1000";
	String rootca_name = "rooca";
	String clientcert_name = "cltcert";
	String clientkey_name = "cltkey";
	String data_pub_topic = "";
	String meta_pub_topic = "";
} aws_config;

volatile uint8_t at_command_no = 1;
struct AT_Command
{
	String cmd;
	std::initializer_list<uint32_t> timeout = {500};
	uint16_t try_count = 1;
	std::initializer_list<String> ex_resp = {"OK"};
	uint8_t no = at_command_no++;
};

// NEOWAY AT Commands
AT_Command AT_OK_10 = {"AT", {1000}, 10, {"OK"}};
AT_Command AT_OK_1 = {"AT", {1000}, 1, {"OK"}};
AT_Command AT_ATE0 = {"ATE0", {1000}, 10, {"OK"}};
AT_Command AT_CPIN_Q = {"AT+CPIN?", {5000}, 5, {"OK"}};
AT_Command AT_CSQ = {"AT+CSQ", {5000}, 5, {"OK"}};
AT_Command AT_CREG_2 = {"AT+CREG=2", {5000}, 5, {"OK"}};
AT_Command AT_CREG_Q = {"AT+CREG?", {5000}, 5, {"2,"}};
AT_Command AT_CGATT_1 = {"AT+CGATT=1", {5000}, 5, {"OK"}};
AT_Command AT_CGATT_Q = {"AT+CGATT?", {5000}, 5, {"+CGATT: 1"}};
AT_Command AT_XIIC_1 = {"AT+XIIC=1", {5000}, 5, {"OK"}};
AT_Command AT_XIIC_Q = {"AT+XIIC?", {5000}, 5, {"+XIIC:    1"}};

// AWS Configuration Commands
AT_Command AT_AWSTLSCFG_AUTHMODE = {"AT+AWSTLSCFG=authmode,1", {5000}, 2, {"OK"}};
AT_Command AT_AWSTLSCFG_ROOTCA = {"AT+AWSTLSCFG=rootca," + aws_config.rootca_name, {5000}, 2, {"OK"}};
AT_Command AT_AWSTLSCFG_CLIENTCERT = {"AT+AWSTLSCFG=clientcert," + aws_config.clientcert_name, {5000}, 2, {"OK"}};
AT_Command AT_AWSTLSCFG_CLIENTKEY = {"AT+AWSTLSCFG=clientkey," + aws_config.clientkey_name, {5000}, 2, {"OK"}};
AT_Command AT_AWSCONNPARAM = {"AT+AWSCONNPARAM=" + aws_config.host + ":8883,1", {5000}, 2, {"OK"}};
AT_Command AT_AWSTLSCFG_Q = {"AT+AWSTLSCFG?", {5000}, 2, {"OK"}};
AT_Command AT_AWSCONN = {"AT+AWSCONN=60,1,4", {30000}, 2, {"OK"}};

// BLE Commands
AT_Command AT_NWBTBLENAME_Q = {"AT+NWBTBLENAME?", {2000}, 1, {""}};
AT_Command AT_NWBTBLEPWR_1 = {"AT+NWBTBLEPWR=1", {1 * ms_s, 2 * ms_s}, 1, {"OK", ""}};
AT_Command AT_NWBLEDISCON_Q = {"AT+NWBLEDISCON?", {1 * ms_s}, 1, {"NWBLEDISCON:\r\n1"}};
AT_Command AT_NWBTBLEPWR_0 = {"AT+NWBTBLEPWR=0", {5000}, 2, {"OK"}};

extern osSemaphoreId_t neoway_rxHandle;
extern osSemaphoreId_t neoway_txHandle;

/**
 * for definition of NEOWAY class used to define NEOWAY module used in AWS 2.0.
 */
class NEOWAY
{

public:
	UART_HandleTypeDef *NEOUART;										   //!< Address of a UART object being used to communicate with NEOWAY module connected externally to NEOWAY module.
	String data_pub_topic = "";											   //!< a String that stores topic of data to be published.
	String meta_pub_topic = "";											   //!< a String that stores topic of meta_data to be published.
	String /*response = "",*/ sub_resp = "", ble_resp = "", dma_resp = ""; //!< a String that stores response coming from UART communication by NEOWAY module for data+meta data publishing.
	volatile bool ble_init = 0, sending_ble = 0;
	volatile NEOWAY_RETURN return_check; //!< an array of size 10 of type NEOWAY_RETURN that stores NEOWAY responses.
	volatile LOOP_CONT neo_control = $CONTINUE;
	bool aws_connected = false;
	uint8_t dma_char[100];
	DIG_PIN V_3_8;
	DIG_PIN NW_PWR;

	String rootca_name = "rooca";
	String clientcert_name = "cltcert";
	String clientkey_name = "cltkey";

#if defined(UART_INTERRUPT_MODE) || defined(UART_DMA_MODE)
	// UART Interrupt handling variables
	volatile bool uart_error = false;
	static const uint16_t UART_RX_BUFFER_SIZE = 2048;
	static const uint16_t UART_TX_BUFFER_SIZE = 1024;
	uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE + 5];
	uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE + 5];
	volatile uint16_t uart_rx_index = 0;
	volatile uint16_t uart_rx_length = 0;
	String interrupt_response = "";
#ifdef UART_DMA_MODE
	volatile bool uart_tx_complete = false;
#endif
#endif

	print_control control;

	/**
	 * a constructor for NEOWAY class object with input parameters
	 * @param [in] NEOUART A pointer pointing to UART_HanfleTypeDef object.
	 */
	NEOWAY(UART_HandleTypeDef *NEOUART) : V_3_8(N58_EN_GPIO_Port, N58_EN_Pin), NW_PWR(N58_PWR_GPIO_Port, N58_PWR_Pin)
	{
		this->NEOUART = NEOUART;
#if defined(UART_INTERRUPT_MODE) || defined(UART_DMA_MODE)
		// Initialize interrupt variables
		uart_error = false;
		uart_rx_index = 0;
		uart_rx_length = 0;
		memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
		memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);
#ifdef UART_DMA_MODE
		uart_tx_complete = false;
#endif
#endif
#if defined(DMAUART)
		START_DMA();
		PAUSE_DMA();
#endif
	}

	/**
	 * @fn Vvoid RESTART()
	 * @brief GPIO_Pins are rewritten as per pwr_pin structure made by us with different hal delays then during PWR_setup time.
	 */
	void START()
	{
		V_3_8.PULSE(0, 2 * us_s, 1 * us_s);
		NW_PWR.SET(1);

		//			NW_PWR.PULSE(1,1 * us_s, 2 * us_s);
	}

	void RESTART()
	{
		V_3_8.PULSE(0, 2 * us_s, 1 * us_s);
		NW_PWR.PULSE(1, 3 * us_s, 2 * us_s);
	}

	String GET_BLE_RESP()
	{
		String resp_ret = ble_resp;
		ble_resp.clear();
		return resp_ret;
	}

	String GET_SUB_RESP()
	{
		String resp_ret = sub_resp;
		sub_resp.clear();
		return resp_ret;
	}

	void PARSE_RECEIVED_MESSAGES(const String &accu_response)
	{
		uint32_t rec_loc = accu_response.find("+AWSSUBRECV");
		uint32_t ble_rec_loc = accu_response.find("BLEPRECV");

		if (rec_loc != not_found)
		{
			uint32_t end_loc = accu_response.find('\n', rec_loc);
			sub_resp = accu_response.substr(rec_loc, end_loc - rec_loc);
			debug.Print("\r\nSUB_RESP : " + sub_resp);
		}

		if (ble_rec_loc != not_found)
		{
			uint32_t ble_end_loc = accu_response.find('\n', ble_rec_loc);
			String ble_temp = accu_response.substr(ble_rec_loc, ble_end_loc);
			uint32_t ble_data_loc = ble_temp.find_last_of(',');
			if (ble_data_loc != not_found)
			{
				ble_temp = ble_temp.substr(ble_data_loc + 1);
				while (ble_temp.back() == '\n' || ble_temp.back() == '\r')
				{
					ble_temp.pop_back();
				}
				ble_resp = ble_temp;
			}
		}
	}

	/**
	 * @fn String SEND_RECIEVE(String command, initializer_list<uint32_t> timeout = { 0 }, uint16_t try_count = 1, initializer_list<String> ex_resp = { "OK", "", "" })
	 * @brief To send or receive commands to AWS from NEOWAY module.
	 * @details Iterating through each no of trials following instructions are to be executed.
	 *
	 *    -HAL_UART_Transmit of input command to NEOWAY through NEOART Object.
	 *
	 *    -Now we go on iterating until timeout.size() is achieved.
	 *
	 *    -under each iteration HAL_UART_Receive is done to receive the response and then response is checked and accordingly stored in return check array .Once we are did all iterations return check is checked if expected response is achieved or not. if yes then break that complete trial loop and return the final response otherwise next trial is done to retry sending command to neoway module until unless total no of possible input trials are finished.
	 * @param [in] command It accepts AT commands that are to be send to NEOWAY Module to communicate with and setup it.
	 * @param [in] timeout initially initialized to 0.
	 * @param [in] try_count Designates no of times send/receive process have to be initiated if not done last time correctly .Initialized to 1.
	 * @param [in]  ex_resp Initialized as list of 3 items: "OK"," "," "
	 * @return A String designating NEOWAY response.
	 */
	String SEND_RECIEVE(String command, std::initializer_list<uint32_t> timeout = {0}, uint16_t try_count = 1, std::initializer_list<String> ex_resp = {"OK", "", ""})
	{
		AT_Command cmd = {command, timeout, try_count, ex_resp};
		return SEND_RECIEVE(cmd);
	}

	String SEND_RECIEVE(AT_Command &command)
	{
		if (neo_control == $CONTINUE)
		{
			refresh_counter();
			String accu_response(5000, ' '); // accumulated response
			String response(5000, ' ');
			accu_response.clear();
			if (command.cmd.back() != 0x0A)
			{
				command.cmd.push_back(0x0D);
				command.cmd.push_back(0x0A);
			}
			uint16_t try_count = command.try_count;

			while (try_count--)
			{
				debug.Print("\r\n---->\r\n" + command.cmd + "\r\n---------- " + d_t_s(try_count));
				BLE_SEND("\r\n-->\r\n" + command.cmd + "\r\n---------- " + d_t_s(try_count));
				HAL_UART_Init(NEOUART);
				HAL_UART_Transmit(NEOUART, (uint8_t *)command.cmd.c_str(), command.cmd.size(), UINT32_MAX - 1);
				uint32_t i = 0;

				for (; i < command.timeout.size(); i++)
				{
					response.clear();
					refresh_counter();
					UART_ReceiveStringToIdle(NEOUART, response, (uint32_t)*(command.timeout.begin() + i));
					//						uint8_t rec_buff[100] = { 0 };
					//						uint16_t rec_len = 0;
					//						HAL_UARTEx_ReceiveToIdle(NEOUART, rec_buff, 100, &rec_len, 1000);
					refresh_counter();
#if defined(neo_print_resp_size)
					debug.Print("< " + d_t_s(response.size()) + " > ");
#endif

					return_check = CHECK_RESPONSE(&response, i < command.ex_resp.size() ? (String *)(command.ex_resp.begin() + i) : NULL);

					accu_response = accu_response + response;
				}

				if (return_check == $EXPECTED_RESPONSE)
				{
					debug.Print("\t{=}\r\n");
					BLE_SEND("\t{=}\r\n");
				}
				else
				{
					BLE_SEND("\t{!}\r\n");
					debug.Print("\t{!}\r\n");
				}

				debug.Print(accu_response + "\r\n<--\r\n");
				BLE_SEND(accu_response + "\r\n<--\r\n");

				PARSE_RECEIVED_MESSAGES(accu_response);

				refresh_counter();

				bool should_break = false;
				if (EVALUATE_RESPONSE_CONTROL(return_check, try_count, should_break))
				{
					return accu_response;
				}

				if (should_break)
				{
					break;
				}
			}
			return accu_response;
		}
		else
		{
			debug.Print("\r\n\t\tSKIPPING : " + command.cmd);
			return "";
		}
	}

	void SEND_RECIEVE_POINTER(String command, std::initializer_list<uint32_t> timeout = {0}, uint16_t try_count = 1, std::initializer_list<String> ex_resp = {"OK", "", ""}, String &accu_response = (String &)"")
	{
		AT_Command cmd = {command, timeout, try_count, ex_resp};
		SEND_RECIEVE_POINTER(cmd, accu_response);
	}

	void SEND_RECIEVE_POINTER(AT_Command &command, String &accu_response)
	{
		if (neo_control == $CONTINUE)
		{
			refresh_counter();
			String response(5000, ' ');
			accu_response.clear();
			command.cmd.push_back(0x0D);
			command.cmd.push_back(0x0A);

			uint16_t try_count = command.try_count;

			while (try_count--)
			{
				debug.Print("\r\n---->\r\n" + command.cmd + "\r\n---------- " + d_t_s(try_count) + "\r\n");
				BLE_SEND("\r\n-->\r\n" + command.cmd + "\r\n---------- " + d_t_s(try_count) + "\r\n");
				HAL_UART_Init(NEOUART);
				HAL_UART_Transmit(NEOUART, (uint8_t *)command.cmd.c_str(), command.cmd.size(), UINT32_MAX - 1);
				uint32_t i = 0;

				for (; i < command.timeout.size(); i++)
				{
					response.clear();
					refresh_counter();
					UART_ReceiveStringToIdle(NEOUART, response, (uint32_t)*(command.timeout.begin() + i));
					refresh_counter();
#if defined(neo_print_resp_size)
					debug.Print("< " + d_t_s(response.size()) + " > ");
#endif
					return_check = CHECK_RESPONSE(&response, i < command.ex_resp.size() ? (String *)(command.ex_resp.begin() + i) : NULL);
					accu_response = accu_response + response;
				}

				if (return_check == $EXPECTED_RESPONSE)
				{
					debug.Print("\t{=}");
					BLE_SEND("\t{=}");
				}
				else
				{
					BLE_SEND("\t{!}");
					debug.Print("\t{!}");
				}

				debug.Print(accu_response + "\r\n<--\r\n");
				BLE_SEND(accu_response + "\r\n<--\r\n");

				PARSE_RECEIVED_MESSAGES(accu_response);

				refresh_counter();

				bool should_break = false;
				if (EVALUATE_RESPONSE_CONTROL(return_check, try_count, should_break))
				{
					return;
				}
				if (should_break)
				{
					break;
				}
			}
		}
		else
		{
			debug.Print("\r\n\t\tSKIPPING : " + command.cmd);
		}
	}

#ifdef UART_INTERRUPT_MODE

	void INIT_UART_INTERRUPT()
	{
		HAL_UARTEx_ReceiveToIdle_IT(NEOUART, (uint8_t *)uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}

	void UART_TX_IRQ_Handler()
	{
		osSemaphoreRelease(neoway_txHandle);
	}

	void UART_ERROR_IRQ_Handler()
	{
		uart_error = true;
		// Clear error flags
		__HAL_UART_CLEAR_PEFLAG(NEOUART);
		__HAL_UART_CLEAR_FEFLAG(NEOUART);
		__HAL_UART_CLEAR_NEFLAG(NEOUART);
		__HAL_UART_CLEAR_OREFLAG(NEOUART);

		// Restart reception
		uart_rx_index = 0;
		HAL_UARTEx_ReceiveToIdle_IT(NEOUART, (uint8_t *)uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}

	void UART_IDLE_IRQ_Handler(uint16_t Pos)
	{
		// Process received data
		if (Pos > 0)
		{
			uart_rx_buffer[Pos] = '\0';
			interrupt_response += String((char *)uart_rx_buffer);
		}

		HAL_UARTEx_ReceiveToIdle_IT(NEOUART, (uint8_t *)uart_rx_buffer, UART_RX_BUFFER_SIZE);
		debug.Prints(interrupt_response);
		PARSE_RECEIVED_MESSAGES(interrupt_response);

		osSemaphoreRelease(neoway_rxHandle);
	}

	bool UART_SEND_IT(const String &data)
	{
		if (data.length() > UART_TX_BUFFER_SIZE)
		{
			return false;
		}

		memcpy(uart_tx_buffer, data.c_str(), data.length());
		uart_tx_buffer[data.length()] = '\r';	  // Add CR
		uart_tx_buffer[data.length() + 1] = '\n'; // Add LF
		uart_tx_buffer[data.length() + 2] = '\0'; // Null-terminate the String

		HAL_StatusTypeDef status = HAL_UART_Transmit_IT(NEOUART, uart_tx_buffer, data.length() + 2);

		if (status != HAL_OK)
		{
			return false;
		}

		return true;
	}

	String UART_GET_RECEIVED_DATA(bool clear_buffer = true)
	{
		String result = "";
		if (clear_buffer)
		{
			result = interrupt_response;
			interrupt_response = ""; // Clear the buffer after reading
			return result;
		}
		return interrupt_response;
	}

	bool UART_DATA_AVAILABLE()
	{
		return (osSemaphoreGetCount(neoway_rxHandle) > 0) || (interrupt_response.length() > 0);
	}

	void RESET_UART_INTERRUPT()
	{
		osSemaphoreRelease(neoway_rxHandle);
		osSemaphoreRelease(neoway_txHandle);
		uart_error = false;
		uart_rx_index = 0;
		uart_rx_length = 0;
		interrupt_response.clear();
		memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
		memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);
	}

	void DEINIT_UART_INTERRUPT()
	{
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_RXNE);
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_TC);
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_ERR);
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_IDLE);
	}

	String SEND_RECEIVE_IT(AT_Command *command)
	{
		if (neo_control == $CONTINUE)
		{
			refresh_counter();
			String accu_response = "";
			String response = "";

			command->cmd.push_back(0x0D);
			command->cmd.push_back(0x0A);

			while (command->try_count--)
			{
				debug.Print("\r\n---->\r\n" + command->cmd + "\r\n---------- " + d_t_s(command->try_count) + "\r\n");
				BLE_SEND("\r\n-->\r\n" + command->cmd + "\r\n---------- " + d_t_s(command->try_count) + "\r\n");

				HAL_UART_Init(NEOUART);
				if (!UART_SEND_IT(command->cmd))
				{
					continue;
				}

				// Wait for transmission complete
				if (osSemaphoreAcquire(neoway_txHandle, 5000) != osOK)
				{
					continue;
				}

				uint32_t i = 0;
				for (; i < command->timeout.size(); i++)
				{
					response.clear();
					refresh_counter();

					// Wait for response with timeout
					if (osSemaphoreAcquire(neoway_rxHandle, (uint32_t)*(command->timeout.begin() + i)) == osOK)
					{
						response = UART_GET_RECEIVED_DATA();
					}

					refresh_counter();

					return_check = CHECK_RESPONSE(&response, i < command->ex_resp.size() ? (String *)(command->ex_resp.begin() + i) : NULL);
					accu_response = accu_response + response;
				}

				if (return_check == $EXPECTED_RESPONSE)
				{
					debug.Print("\t{=}");
					BLE_SEND("\t{=}");
				}
				else
				{
					BLE_SEND("\t{!}");
					debug.Print("\t{!}");
				}

				debug.Print(accu_response + "\r\n<--\r\n");
				BLE_SEND(accu_response + "\r\n<--\r\n");

				PARSE_RECEIVED_MESSAGES(accu_response);

				refresh_counter();

				bool should_break = false;
				if (EVALUATE_RESPONSE_CONTROL(return_check, command->try_count, should_break))
				{
					return accu_response;
				}

				if (should_break)
				{
					break;
				}
			}
			return accu_response;
		}
		else
		{
			debug.Print("\r\n\t\tSKIPPING : " + command->cmd);
			return "";
		}
	}

#endif // UART_INTERRUPT_MODE

#ifdef UART_DMA_MODE
	void INIT_UART_DMA_INTERRUPT()
	{
		uart_rx_index = 0;
		uart_rx_length = 0;
		interrupt_response.clear();

		__HAL_UART_ENABLE_IT(NEOUART, UART_IT_IDLE);
		__HAL_UART_ENABLE_IT(NEOUART, UART_IT_ERR);

		HAL_UART_Receive_DMA(NEOUART, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}

	void UART_DMA_RX_IRQ_Handler()
	{
		uart_rx_length = UART_RX_BUFFER_SIZE;
		uart_rx_buffer[UART_RX_BUFFER_SIZE - 1] = '\0';
		interrupt_response += String((char *)uart_rx_buffer);

		uart_rx_index = 0;
		memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
		HAL_UART_Receive_DMA(NEOUART, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}

	bool UART_SEND_DMA(const String &data, uint32_t timeout = 5000)
	{
		if (data.length() > UART_TX_BUFFER_SIZE)
		{
			return false;
		}

		memcpy(uart_tx_buffer, data.c_str(), data.length());
		uart_tx_complete = false;

		HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(NEOUART, uart_tx_buffer, data.length());

		if (status != HAL_OK)
		{
			return false;
		}

		uint32_t start_time = HAL_GetTick();
		while (!uart_tx_complete && (HAL_GetTick() - start_time) < timeout)
		{
			osDelay(1);
		}

		return uart_tx_complete;
	}

	void RESET_UART_DMA()
	{
		HAL_UART_DMAStop(NEOUART);
		uart_error = false;
		uart_rx_index = 0;
		uart_rx_length = 0;
		uart_tx_complete = false;
		interrupt_response.clear();
		memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
		memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);
	}

	void DEINIT_UART_DMA()
	{
		HAL_UART_DMAStop(NEOUART);
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_IDLE);
		__HAL_UART_DISABLE_IT(NEOUART, UART_IT_ERR);
	}
#endif // UART_DMA_MODE

#if defined(UART_INTERRUPT_MODE) || defined(UART_DMA_MODE)
	void RESET_UART_BUFFERS()
	{
		uart_error = false;
		uart_rx_index = 0;
		uart_rx_length = 0;
		interrupt_response.clear();
		memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
		memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE);
#ifdef UART_DMA_MODE
		uart_tx_complete = false;
#endif
	}
#endif // UART_INTERRUPT_MODE || UART_DMA_MODE

	NEOWAY_RETURN GET_Return() const
	{
		return return_check;
	}

	void RESET_CONTROL()
	{
		neo_control = $CONTINUE;
	}

	/**
	 * @fn NEOWAY_RETURN CHECK_RESPONSE(String *response, String *ex_resp)
	 * @brief To check incoming response to NEOWAY.
	 * @detail as per ex_response and response being fed to input Possible NEOWAY_RETURN response are being returned.
	 * @param [in] response base address of Response variable which stores String data being received from NEOWAY module through NEOART.
	 * @param [in] ex_resp base address of Ex_Response list which stores expected responses.
	 * @return NEOWAY_RETURN a NEOWAY_RETURN enum designating NEOWAY Return status.
	 */
	NEOWAY_RETURN CHECK_RESPONSE(String *response, String *ex_resp)
	{
		if (ex_resp == NULL)
		{
			return $EXPECTED_RESPONSE;
		}
		else if (response->find(ex_resp->c_str()) < not_found || *ex_resp == "")
		{
			if (response->find(ex_resp->c_str()) < not_found || *ex_resp == "")
			{
				return $EXPECTED_RESPONSE;
			}
			else if (response->find("ERROR") == not_found)
			{
				if (response->find("ERROR") == not_found)
				{
					return $ERROR_RESPONSE;
				}
				else if (response->size() == 0)
				{
					return $NO_RESPONSE;
				}
				else
				{
					return $N_ERROR;
				}
				HAL_Delay(1);
			}
		}
		return $N_ERROR;
	}

	/**
	 * @fn const String& GET_data_pub_topic()
	 * @brief To get String stored in data_pub_topic.
	 * @return data_pub_topic Returns constant String stored in data_pub_topic.
	 */
	const String &GET_data_pub_topic() const
	{
		return data_pub_topic;
	}
	/**
	 * @fn SET_data_pub_topic(const String &data_pub_topic = "")
	 * @brief To set data_pub_topic of corresponding this NEOWAY object as per input String
	 * @param [in] data_pub_topic a constant String Initially initialized to " ".
	 */
	void SET_data_pub_topic(const String &data_pub_topic = "")
	{
		this->data_pub_topic = data_pub_topic;
	}
	/**
	 * @fn const String& GET_meta_pub_topic()()
	 * @brief To get String stored in meta_pub_topic.
	 * @return data_pub_topic Returns constant String stored in meta_pub_topic.
	 */
	const String &GET_meta_pub_topic() const
	{
		return meta_pub_topic;
	}
	/**
	 * @fn SET_meta_pub_topic(const String &meta_pub_topic = "")
	 * @brief To set meta_pub_topic of corresponding this NEOWAY object as per input String
	 * @param [in] meta_pub_topic a constant String Initially initialized to " ".
	 */
	void SET_meta_pub_topic(const String &meta_pub_topic = "")
	{
		this->meta_pub_topic = meta_pub_topic;
	}

	void BLE_NAME(String ws_name)
	{
		String ble_name = "CWS_" + ws_name;
		AT_Command cmd = {"AT+NWBTBLENAME?", {2000}, 1, {ble_name}};
		SEND_RECIEVE(cmd);
		if (neo_control != $CONTINUE)
		{
			neo_control = $CONTINUE;
			AT_Command set_name = {"AT+NWBTBLENAME=\"" + ble_name + "\"", {2000}, 1, {"OK"}};
			SEND_RECIEVE(set_name);
		}
	}

	void BLE_SETUP(String ws_name)
	{
		control.save_ble_print(1);
		BLE_NAME(ws_name);
		SEND_RECIEVE(AT_NWBTBLEPWR_1);
		SEND_RECIEVE(AT_NWBLEDISCON_Q);
		if (neo_control == $CONTINUE)
		{
			ble_init = 1;
			delay_ms(1 * ms_s);
			BLE_SEND("Connected");
		}
		else
		{
			neo_control = $CONTINUE;
			AT_Command empty_cmd = {"", {5 * ms_s}, 1, {""}};
			SEND_RECIEVE(empty_cmd);
			SEND_RECIEVE(AT_NWBLEDISCON_Q);
			if (neo_control == $CONTINUE)
			{
				ble_init = 1;
				delay_ms(1 * ms_s);
				BLE_SEND("Connected");
			}
			else
			{
				neo_control = $CONTINUE;
				SEND_RECIEVE(AT_NWBTBLEPWR_0);
			}
		}
		control.restore_ble_print();
		neo_control = $CONTINUE;
	}

	String BLE_READ(uint32_t timeout = 5000)
	{
		if (ble_init == 1 && ble_cont == $CONTINUE && control.is_ble_print())
		{
			LOOP_CONT neo_control_temp = neo_control;
			control.save_both_print(0);
			NEOWAY_RETURN return_check_temp = return_check;
			neo_control = $CONTINUE;

			AT_Command cmd = {"", {timeout}, 1, {""}};
			SEND_RECIEVE(cmd);

			neo_control = neo_control_temp;
			return_check = return_check_temp;
			control.restore_both_print();
		}
		return GET_BLE_RESP();
	}

	void BLE_SEND(String send_String)
	{
		if (ble_init == 1 && sending_ble == 0 && ble_cont == $CONTINUE && control.is_ble_print())
		{
			LOOP_CONT neo_control_temp = neo_control;
			sending_ble = 1;
			control.save_both_print(0);
			neo_control = $CONTINUE;
			NEOWAY_RETURN return_check_temp = return_check;

			AT_Command cmd1 = {"AT+NWBLEPSEND=0,0,0,1," + d_t_s(send_String.size(), 0), {500}, 1, {">"}};
			SEND_RECIEVE(cmd1);

			AT_Command cmd2 = {send_String, {100, 100}, 1, {"OK", "NWBLEPSEND"}};
			SEND_RECIEVE(cmd2);

			neo_control = neo_control_temp;
			return_check = return_check_temp;
			sending_ble = 0;
			control.restore_both_print();
		}
	}

	void POWER_ON()
	{
		SEND_RECIEVE(AT_OK_1);
		if (neo_control != $CONTINUE)
		{
			neo_control = $CONTINUE;
			START();
		}
		SEND_RECIEVE(AT_OK_10);
		SEND_RECIEVE(AT_ATE0);
	}

	void POWER_OFF()
	{
		NW_PWR.PULSE(1, 3 * us_s, 3 * us_s);
		V_3_8.SET(0, 1 * us_s);
	}

	void INIT()
	{
		SEND_RECIEVE(AT_CPIN_Q);
		SEND_RECIEVE(AT_CSQ);
		SEND_RECIEVE(AT_CREG_2);
		HAL_Delay(1000);
		SEND_RECIEVE(AT_CREG_Q);
		SEND_RECIEVE(AT_CGATT_1);
		SEND_RECIEVE(AT_CGATT_Q);
		SEND_RECIEVE(AT_XIIC_1);
		SEND_RECIEVE(AT_XIIC_Q);
		delay_ms(2 * ms_s);
	}

	/**
	 * @brief GPS power control + optional status query.
	 * @details Uses AT$MYGPSPWR to enable/disable GPS and AT$MYGPSSTATE to confirm status.
	 * @param enable true to start GPS service, false to stop.
	 * @param query_state true to query $MYGPSSTATE after power toggle.
	 * @param startup_delay_ms Optional delay after enabling GPS to allow warm-up.
	 */
	void GPS_INIT(bool enable = true, bool query_state = true, uint32_t startup_delay_ms = 2000)
	{
		uint8_t en = enable ? 1U : 0U;
		AT_Command pwr_cmd = {"AT$MYGPSPWR=" + d_t_s(en), {5000}, 2, {"OK"}};
		SEND_RECIEVE(pwr_cmd);

		if (query_state)
		{
			AT_Command state_cmd = {"AT$MYGPSSTATE", {2000}, 1, {"$MYGPSSTATE"}};
			SEND_RECIEVE(state_cmd);
		}

		if (enable && startup_delay_ms > 0)
		{
			delay_ms(startup_delay_ms);
		}
	}

	/**
	 * @brief Read GPS NMEA data via AT$MYGPSPOS.
	 * @param type NMEA type: 0 GPGGA, 1 GPGSA, 2 GPGSV, 3 GPRMC, 4 GPVTG, 5 GPGLL, 6 all.
	 * @param mode Output mode: 0 once, 1 periodic, 2 disable periodic.
	 * @param timeout First response timeout in ms (NMEA data).
	 * @return Accumulated response containing NMEA payload and OK.
	 */
	String GPS_READ(uint8_t type = 0, uint8_t mode = 0, uint32_t timeout = 5000)
	{
		AT_Command pos_cmd = {"AT$MYGPSPOS=" + d_t_s(type) + "," + d_t_s(mode), {timeout, 1000}, 1, {"$MYGPSPOS", "OK"}};
		return SEND_RECIEVE(pos_cmd);
	}

	/**
	 * @brief neoway initialisation only
	 * @retval none
	 */
	void AWS_CON()
	{
		SEND_RECIEVE(AT_AWSTLSCFG_AUTHMODE);
		SEND_RECIEVE(AT_AWSTLSCFG_ROOTCA);
		SEND_RECIEVE(AT_AWSTLSCFG_CLIENTCERT);
		SEND_RECIEVE(AT_AWSTLSCFG_CLIENTKEY);
		SEND_RECIEVE(AT_AWSCONNPARAM);
		SEND_RECIEVE(AT_AWSTLSCFG_Q);
		SEND_RECIEVE(AT_AWSCONN);
		if (neo_control == $CONTINUE)
		{
			aws_connected = true;
		}
	}

private:
	bool EVALUATE_RESPONSE_CONTROL(NEOWAY_RETURN return_check, uint16_t &try_count, bool &should_break)
	{
		if (return_check == $EXPECTED_RESPONSE || return_check == $ex_resp_NULL)
		{
			neo_control = $CONTINUE;
#if defined(neo_cont_print)
			debug.Print("\r\nneo_control = CONTINUE");
#endif
			return true; // Should return from calling function
		}
		else if (return_check != $NO_RESPONSE || return_check != $ERROR_RESPONSE)
		{
			if (try_count > 1)
			{
				neo_control = $REPEAT;
#if defined(neo_cont_print)
				debug.Print("\r\nneo_control = REPEAT - NO/ERROR RESPONSE");
#endif
			}
			else
			{
				neo_control = $BREAK;
#if defined(neo_cont_print)
				debug.Print("\r\nneo_control = BREAK1");
#endif
			}
		}
		else
		{
			neo_control = $BREAK;
#if defined(neo_cont_print)
			debug.Print("\r\nneo_control = BREAK2");
#endif
			should_break = true; // External break flag
		}

		return false; // Continue loop
	}
};

#ifdef UART_INTERRUPT_MODE

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Pos)
{
	if (huart->Instance == USART2)
	{ // Adjust USART instance as needed
		if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
		{
			// Handle IDLE event
			extern NEOWAY neoway;
			neoway.UART_IDLE_IRQ_Handler(Pos);
		}
	}
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{ // Adjust USART instance as needed
		extern NEOWAY neoway;
		neoway.UART_TX_IRQ_Handler();
	}
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{ // Adjust USART instance as needed
		extern NEOWAY neoway;
		neoway.UART_ERROR_IRQ_Handler();
	}
}

void INIT_UART_CALLBACKS()
{
	extern NEOWAY neoway;
	neoway.INIT_UART_INTERRUPT();
	debug.Print("\r\nNEOWAY UART callbacks initialized\r\n");
}

#endif // UART_INTERRUPT_MODE

#ifdef UART_DMA_MODE

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{ // Adjust USART instance as needed
		extern NEOWAY neoway;
		neoway.UART_DMA_RX_IRQ_Handler();
	}
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{ // Adjust USART instance as needed
		extern NEOWAY neoway;
		neoway.uart_tx_complete = true;
	}
}

#endif // UART_DMA_MODE

/*
 * UART MODES CONFIGURATION AND INTEGRATION GUIDE:
 *
 * ==========================================
 * MODE SELECTION:
 * ==========================================
 *
 * Choose one of the following modes by uncommenting the appropriate #define:
 *
 * 1. BLOCKING MODE (Default):
 *    #define UART_BLOCKING_MODE
 *    - Uses original HAL_UART_Transmit() and HAL_UART_Receive() functions
 *    - Simple but blocks execution during communication
 *    - No interrupt handling required
 *
 * 2. INTERRUPT MODE:
 *    #define UART_INTERRUPT_MODE
 *    - Uses HAL_UART_Transmit_IT() and HAL_UART_Receive_IT() functions
 *    - Non-blocking communication with interrupt handling
 *    - Suitable for moderate data rates
 *
 * 3. DMA MODE:
 *    #define UART_DMA_MODE
 *    - Uses HAL_UART_Transmit_DMA() and HAL_UART_Receive_DMA() functions
 *    - Most efficient for high-speed data transfer
 *    - Requires DMA configuration in CubeMX
 *
 * ==========================================
 * INTEGRATION STEPS:
 * ==========================================
 *
 * FOR INTERRUPT MODE:
 * 1. Call INIT_UART_CALLBACKS() after initializing the NEOWAY object
 * 2. Ensure neoway_rxHandle and neoway_txHandle semaphores are created in FreeRTOS
 * 3. Adjust USART instance in callback functions as needed
 *
 * FOR DMA MODE:
 * 1. Configure DMA in CubeMX for UART TX and RX
 * 2. Call neoway.INIT_UART_DMA_INTERRUPT() after initializing the NEOWAY object
 * 3. Adjust USART instance in callback functions as needed
 *
 * ==========================================
 * USAGE EXAMPLES:
 * ==========================================
 *
 * BLOCKING MODE (Default):
 * NEOWAY neoway(&huart2);
 * neoway.POWER_ON();
 * neoway.INIT();
 * neoway.AWS_CON();
 *
 * INTERRUPT MODE:
 * NEOWAY neoway(&huart2);
 * INIT_UART_CALLBACKS();
 * neoway.POWER_ON();
 * neoway.INIT();
 * neoway.AWS_CON();
 *
 * DMA MODE:
 * NEOWAY neoway(&huart2);
 * neoway.INIT_UART_DMA_INTERRUPT();
 * neoway.POWER_ON();
 * neoway.INIT();
 * neoway.AWS_CON();
 */
#endif /* NEOWAY_AT_MODDE_ENABLE */

#define NEOWAY_SELF_MODE_ENABLE 1
#if NEOWAY_SELF_MODE_ENABLE == 1

extern bool STM32_RS485_BridgeHandleReverseRxFrame(const uint8_t *data, uint16_t size);

/* Status framing is generated by the on-module CLI (see code/comm/nwy_cli_status.*).
 Frames: [id][len][payload...][crc16/ccitt]. */
#define NWY_STATUS_MAX_PAYLOAD_LEN 255U
#define NWY_STATUS_RX_MAX_PAYLOAD_LEN 64
#define NWY_MODBUS_A6_HEADER 0xA6U
#define NWY_MODBUS_FLAG_TOKEN_PRESENT 0x01U
#define NWY_MODBUS_FLAG_IP_PRESENT 0x02U
#define NWY_MODBUS_FLAG_MSG_ID_PRESENT 0x04U
#define NWY_MODBUS_FLAG_WRITE_RESULT 0x40U
#define NWY_MODBUS_FLAG_WRITE_REQ 0x80U
#define NWY_MODBUS_MAX_FRAME_LEN 600U
#define NWY_RX_WATCHDOG_IDLE_TIMEOUT_MS 55000U
#define NWY_RX_WATCHDOG_CONNECTED_TIMEOUT_MS 60000U
#define NWY_MQTT_STATUS_CHECK_TIMEOUT_MS 5000U
#define NWY_MQTT_STATUS_CHECK_QUERY 0xFFU

typedef enum
{
	NWY_STATUS_UART_READY = 0xC1,
	NWY_STATUS_SIM = 0xC2,
	NWY_STATUS_SIGNAL = 0xC3,
	NWY_STATUS_INTERNET = 0xC4,
	NWY_STATUS_SD_CARD = 0xC5,
	NWY_STATUS_MQTT = 0xC6,
	NWY_STATUS_DATA_PUSH = 0xC7,
	NWY_STATUS_TIME = 0xC8,
	NWY_STATUS_DATE = 0xC9
} nwy_status_frame_id_t;

typedef enum
{
	NWY_SIM_OK = 0x00,
	NWY_SIM_NO_SIM = 0x01,
	NWY_SIM_ERROR = 0x02
} nwy_sim_status_t;

typedef enum
{
	NWY_INTERNET_CONNECTED = 0x00,
	NWY_INTERNET_ERROR = 0x01
} nwy_internet_status_t;

typedef enum
{
	NWY_SD_INIT_SUCCESS = 0x00,
	NWY_SD_INIT_ERROR = 0x01,
	NWY_SD_NO_CARD = 0x02
} nwy_sd_status_t;

typedef enum
{
	NWY_MQTT_CONNECTED = 0x00,
	NWY_MQTT_CONNECTION_START = 0x01,
	NWY_MQTT_SSL_ERROR = 0x02,
	NWY_MQTT_DISCONNECTED = 0x03,
	NWY_MQTT_TIMEOUT = 0x04
} nwy_mqtt_status_t;

typedef enum
{
	NWY_PUBLISH_SUCCESS = 0x00,
	NWY_PUBLISH_FAIL = 0x01
} nwy_publish_status_t;

typedef struct
{
		bool valid;
		uint16_t hour;
		uint16_t min;
		uint16_t sec;
} nwy_time_fields_t;

typedef struct
{
		bool valid;
		uint16_t day;
		uint16_t month;
		uint32_t year;
} nwy_date_fields_t;

typedef struct
{
		bool uart_ready;
		nwy_sim_status_t sim_status;
		uint8_t signal_strength;
		nwy_internet_status_t internet_status;
		nwy_sd_status_t sd_status;
		nwy_mqtt_status_t mqtt_status;
		nwy_publish_status_t publish_status;
		nwy_time_fields_t time_fields;
		nwy_date_fields_t date_fields;
} nwy_status_snapshot_t;

class NEOWAY
{

	public:
		UART_HandleTypeDef *NEOUART;										   //!< Address of a UART object being used to communicate with NEOWAY module connected externally to NEOWAY module.
		String data_pub_topic = "";											   //!< a String that stores topic of data to be published.
		String meta_pub_topic = "";											   //!< a String that stores topic of meta_data to be published.
		String /*response = "",*/sub_resp = "", ble_resp = "", dma_resp = ""; //!< a String that stores response coming from UART communication by NEOWAY module for data+meta data publishing.
		volatile bool ble_init = 0, sending_ble = 0;
		volatile NEOWAY_RETURN return_check; //!< an array of size 10 of type NEOWAY_RETURN that stores NEOWAY responses.
		volatile LOOP_CONT neo_control = $CONTINUE;
		bool aws_connected = false;
		uint8_t dma_char[100];
		DIG_PIN V_3_8;
		DIG_PIN NW_PWR;
		DIG_PIN N58_RST;

		using status_frame_cb = void (*)(uint8_t frame_id, const uint8_t *payload, uint16_t length);

		NEOWAY(UART_HandleTypeDef *NEOUART) :
				V_3_8(N58_EN_GPIO_Port, N58_EN_Pin), NW_PWR(N58_PWR_GPIO_Port, N58_PWR_Pin), N58_RST(
						N58_RST_GPIO_Port, N58_RST_Pin)
		{
			this->NEOUART = NEOUART;
			reset_parser();
			reset_status();
			reset_rx_watchdog();
		}

		String GET_BLE_RESP()
		{
			String resp_ret = ble_resp;
			ble_resp.clear();
			return resp_ret;
		}

		String GET_SUB_RESP()
		{
			String resp_ret = sub_resp;
			sub_resp.clear();
			return resp_ret;
		}

		void BLE_SEND(String /*send_String*/)
				{
			/* Self mode does not forward BLE debug. */
		}

		void RESTART()
		{
			reset_parser();
			reset_status();
			reset_rx_watchdog();
			N58_RST.PULSE(1, 1 * ms_s, 1 * ms_s);
		}

		void POWER_ON()
		{
			V_3_8.PULSE(0, 2 * ms_s, 1 * ms_s);
			NW_PWR.SET(1);
		}

		void POWER_OFF()
		{
			NW_PWR.PULSE(1, 3 * ms_s, 3 * ms_s);
			V_3_8.SET(0, 1 * ms_s);
		}

		void INIT()
		{
			reset_parser();
			reset_status();
			reset_rx_watchdog();
			INIT_STATUS_RX_IT();
		}

		void AWS_CON()
		{
			aws_connected = true;
		}

		/* Arm single-byte interrupt-driven RX to feed status frames without blocking. */
		void INIT_STATUS_RX_IT()
		{
			(void) arm_status_rx(true);
		}

		/* Called from HAL_UART_RxCpltCallback to push the byte and re-arm RX. */
		void STATUS_RX_IRQ_Handler()
		{
			if (!status_rx_armed)
			{
				return;
			}

			uint8_t rx_byte = status_rx_byte;
			stage_raw_rx_byte(rx_byte);
			stage_pending_status_byte(rx_byte);
			status_rx_armed = false;
			(void) arm_status_rx(true);
		}

		void STATUS_TX_IRQ_Handler()
		{
			status_tx_in_progress = false;
		}

		void STATUS_ERROR_IRQ_Handler()
		{
			if (NEOUART == NULL)
			{
				status_rx_armed = false;
				status_tx_in_progress = false;
				return;
			}

			uint32_t error_code = NEOUART->ErrorCode;
			if (error_code == HAL_UART_ERROR_NONE)
			{
				if (!status_rx_armed)
				{
					(void) arm_status_rx(false);
				}
				return;
			}

			status_tx_in_progress = false;
			status_rx_armed = false;

			if ((error_code & HAL_UART_ERROR_PE) != 0U)
			{
				__HAL_UART_CLEAR_PEFLAG(NEOUART);
			}
			if ((error_code & HAL_UART_ERROR_FE) != 0U)
			{
				__HAL_UART_CLEAR_FEFLAG(NEOUART);
			}
			if ((error_code & HAL_UART_ERROR_NE) != 0U)
			{
				__HAL_UART_CLEAR_NEFLAG(NEOUART);
			}
			if ((error_code & HAL_UART_ERROR_ORE) != 0U)
			{
				__HAL_UART_CLEAR_OREFLAG(NEOUART);
			}

			for (uint32_t count = 0; count < 16U; ++count)
			{
				if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_RXNE) != SET)
				{
					break;
				}

				uint8_t rx_byte = (uint8_t) (NEOUART->Instance->RDR & 0xFFU);
				stage_raw_rx_byte(rx_byte);
				stage_pending_status_byte(rx_byte);
			}

			(void) arm_status_rx(false);
		}

		void SET_STATUS_CALLBACK(status_frame_cb cb)
				{
			on_status_frame = cb;
		}

		const nwy_status_snapshot_t& GET_STATUS() const
		{
			return status_snapshot;
		}

		void POLL_WATCHDOG()
		{
			if (NEOUART == NULL)
			{
				return;
			}

			if (!status_rx_armed)
			{
				(void) arm_status_rx(true);
			}

			poll_status_rx_registers();
			flush_pending_status_bytes();
			flush_pending_raw_rx_log();
			flush_pending_status_log();

			uint32_t now = HAL_GetTick();
			if (mqtt_status_check_pending)
			{
				if ((now - mqtt_status_check_sent_tick) >= NWY_MQTT_STATUS_CHECK_TIMEOUT_MS)
				{
					debug.Print("[N4G] MQTT status check timeout; restarting module\r\n");
					RESTART();
				}
				return;
			}

			if ((now - last_valid_rx_tick) < rx_watchdog_timeout_ms)
			{
				return;
			}

			if (send_mqtt_status_check())
			{
				mqtt_status_check_pending = true;
				mqtt_status_check_sent_tick = now;
				debug.Print("[N4G] RX idle; requesting MQTT status\r\n");
			}
		}

		/* Build and send a DATA_PUSH frame carrying an ASCII payload with a 1-byte length header. */
		bool SEND_DATA_PUSH(char *message, size_t len)
				{
			if (len > NWY_STATUS_MAX_PAYLOAD_LEN || NEOUART == NULL)
			{
				return false;
			}

			if (status_tx_in_progress)
			{
				return false;
			}

			uint16_t payload_len = (uint16_t) len;
			status_tx_frame[0] = (uint8_t) NWY_STATUS_DATA_PUSH;
			status_tx_frame[1] = (uint8_t) payload_len;
			if (payload_len > 0)
					{
				memcpy(&status_tx_frame[2], message, payload_len);
			}

			uint16_t crc = crc16_ccitt(status_tx_frame, (uint32_t) (2 + payload_len));
			status_tx_frame[2 + payload_len] = (uint8_t) (crc >> 8);
			status_tx_frame[3 + payload_len] = (uint8_t) (crc & 0xFF);

			size_t total_len = (size_t) payload_len + 4U;
			debug.Print(status_tx_frame, 5," DATA_PUSH Frame Sent: ");

			HAL_StatusTypeDef tx = HAL_UART_Transmit_IT(NEOUART, status_tx_frame, (uint16_t) total_len);
			if (tx == HAL_OK)
			{
				status_tx_in_progress = true;
				return true;
			}

			return false;
		}

		void FEED_STATUS_BYTES(const uint8_t *data, size_t len)
				{
			if (data == NULL || len == 0)
					{
				return;
			}

			for (size_t i = 0; i < len; ++i)
					{
				if (parser.frame_type == NWY_FRAME_NONE)
						{
					if (is_status_frame_id(data[i]))
							{
						begin_frame(NWY_FRAME_STATUS, data[i]);
						continue;
					}

					if (data[i] == NWY_MODBUS_A6_HEADER)
							{
						begin_frame(NWY_FRAME_MODBUS_A6, data[i]);
						continue;
					}

					continue;
				}

				if (parser.received_len >= sizeof(parser.buffer))
						{
					debug.snprint("[N4G][PARSER] overflow type=%u len=%u\r\n",
							(unsigned) parser.frame_type,
							(unsigned) parser.received_len);
					reset_parser();
					continue;
				}

				parser.buffer[parser.received_len++] = data[i];

				if (parser.frame_type == NWY_FRAME_STATUS && parser.received_len >= 2 && parser.expected_len == 0)
						{
					uint16_t payload_len = parser.buffer[1];
					if (payload_len > NWY_STATUS_RX_MAX_PAYLOAD_LEN)
					{
						debug.snprint("[N4G][PARSER] status_len_invalid payload=%u max=%u resync\r\n",
								(unsigned) payload_len,
								(unsigned) NWY_STATUS_RX_MAX_PAYLOAD_LEN);
						reset_parser();
						continue;
					}
					parser.expected_len = (size_t) payload_len + 4U; /* id + 1-byte len + payload + crc16 */
					debug.snprint("[N4G][PARSER] status_expect payload=%u total=%u\r\n",
							(unsigned) payload_len,
							(unsigned) parser.expected_len);
				}

				if (parser.frame_type == NWY_FRAME_MODBUS_A6 && parser.received_len >= 3 && parser.expected_len == 0)
						{
					size_t frame_len = 0U;
					if (!compute_modbus_a6_frame_len(parser.buffer[1], parser.buffer[2], &frame_len))
					{
						debug.snprint("[N4G][PARSER] a6_len_invalid payload=%u flags=0x%02X\r\n",
								(unsigned) parser.buffer[1],
								(unsigned) parser.buffer[2]);
						reset_parser();
						continue;
					}
					parser.expected_len = frame_len;
					debug.snprint("[N4G][PARSER] a6_expect payload=%u flags=0x%02X total=%u\r\n",
							(unsigned) parser.buffer[1],
							(unsigned) parser.buffer[2],
							(unsigned) parser.expected_len);
				}

				if (parser.expected_len > 0U && parser.received_len == parser.expected_len)
						{
					debug.snprint("[N4G][PARSER] complete type=%u len=%u\r\n",
							(unsigned) parser.frame_type,
							(unsigned) parser.expected_len);
					if (parser.frame_type == NWY_FRAME_STATUS)
					{
						process_status_frame(parser.buffer, parser.expected_len);
					}
					else if (parser.frame_type == NWY_FRAME_MODBUS_A6)
					{
						process_modbus_a6_frame(parser.buffer, parser.expected_len);
					}
					reset_parser();
				}
			}
		}

	private:
		typedef enum
		{
				NWY_FRAME_NONE = 0,
				NWY_FRAME_STATUS,
				NWY_FRAME_MODBUS_A6
		} nwy_frame_type_t;

		struct
		{
				nwy_frame_type_t frame_type = NWY_FRAME_NONE;
				size_t expected_len = 0;
				size_t received_len = 0;
				uint8_t buffer[1 + 1 + NWY_STATUS_MAX_PAYLOAD_LEN + 2] = { 0 };
		} parser;

		uint8_t status_rx_byte = 0;
		volatile bool status_rx_armed = false;
		volatile bool raw_rx_log_pending = false;
		volatile bool raw_rx_log_overflow = false;
		uint16_t raw_rx_log_len = 0U;
		uint8_t raw_rx_log_buffer[128] = { 0 };
		volatile bool pending_status_bytes_overflow = false;
		volatile uint16_t pending_status_bytes_len = 0U;
		uint8_t pending_status_bytes[128] = { 0 };
		volatile bool status_log_pending = false;
		uint8_t pending_status_frame_id = 0U;
		uint16_t pending_status_payload_len = 0U;
		uint8_t pending_status_payload[NWY_STATUS_MAX_PAYLOAD_LEN] = { 0 };
		uint8_t status_tx_frame[1 + 1 + NWY_STATUS_MAX_PAYLOAD_LEN + 2] = { 0 };
		volatile bool status_tx_in_progress = false;
		volatile bool mqtt_status_check_pending = false;
		uint32_t last_valid_rx_tick = 0U;
		uint32_t mqtt_status_check_sent_tick = 0U;
		uint32_t rx_watchdog_timeout_ms = NWY_RX_WATCHDOG_IDLE_TIMEOUT_MS;

		nwy_status_snapshot_t status_snapshot = { 0 };
		status_frame_cb on_status_frame = NULL;

		void reset_rx_watchdog()
		{
			status_tx_in_progress = false;
			mqtt_status_check_pending = false;
			last_valid_rx_tick = HAL_GetTick();
			mqtt_status_check_sent_tick = 0U;
			rx_watchdog_timeout_ms = NWY_RX_WATCHDOG_IDLE_TIMEOUT_MS;
		}

		void note_valid_rx_message()
		{
			last_valid_rx_tick = HAL_GetTick();
		}

		void note_mqtt_status_check_connected()
		{
			mqtt_status_check_pending = false;
			mqtt_status_check_sent_tick = 0U;
			last_valid_rx_tick = HAL_GetTick();
			rx_watchdog_timeout_ms = NWY_RX_WATCHDOG_CONNECTED_TIMEOUT_MS;
		}

		void stage_raw_rx_byte(uint8_t byte)
		{
			if (raw_rx_log_len < sizeof(raw_rx_log_buffer))
			{
				raw_rx_log_buffer[raw_rx_log_len++] = byte;
				raw_rx_log_pending = true;
				return;
			}

			raw_rx_log_overflow = true;
		}

		void flush_pending_raw_rx_log()
		{
			if (!raw_rx_log_pending && !raw_rx_log_overflow)
			{
				return;
			}

			if (raw_rx_log_len > 0U)
			{
				debug.Print(raw_rx_log_buffer, raw_rx_log_len, "[N4G][RAW_RX]");
				raw_rx_log_len = 0U;
				raw_rx_log_pending = false;
			}

			if (raw_rx_log_overflow)
			{
				debug.Print("[N4G][RAW_RX] overflow\r\n");
				raw_rx_log_overflow = false;
			}
		}

		void stage_pending_status_byte(uint8_t byte)
		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			if (pending_status_bytes_len < sizeof(pending_status_bytes))
			{
				pending_status_bytes[pending_status_bytes_len++] = byte;
			}
			else
			{
				pending_status_bytes_overflow = true;
			}
			if (!primask)
			{
				__enable_irq();
			}
		}

		void flush_pending_status_bytes()
		{
			uint8_t local_buffer[sizeof(pending_status_bytes)] = { 0 };
			uint16_t local_len = 0U;
			bool overflowed = false;

			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			local_len = pending_status_bytes_len;
			if (local_len > 0U)
			{
				memcpy(local_buffer, pending_status_bytes, local_len);
				pending_status_bytes_len = 0U;
			}
			overflowed = pending_status_bytes_overflow;
			pending_status_bytes_overflow = false;
			if (!primask)
			{
				__enable_irq();
			}

			if (overflowed)
			{
				debug.Print("[N4G][RX] pending_status_overflow\r\n");
				reset_parser();
			}

			if (local_len > 0U)
			{
				FEED_STATUS_BYTES(local_buffer, local_len);
			}
		}

		void stage_status_log(uint8_t frame_id, const uint8_t *payload, uint16_t payload_len)
		{
			pending_status_frame_id = frame_id;
			pending_status_payload_len = payload_len;
			if ((payload != NULL) && (payload_len > 0U))
			{
				memcpy(pending_status_payload, payload, payload_len);
			}
			status_log_pending = true;
		}

		void flush_pending_status_log()
		{
			if (!status_log_pending)
			{
				return;
			}

			status_log_pending = false;
			debug.snprint("[N4G][RX] %s id=0x%02X len=%u\r\n",
					status_frame_name(pending_status_frame_id),
					(unsigned) pending_status_frame_id,
					(unsigned) pending_status_payload_len);
			if (pending_status_payload_len > 0U)
			{
				debug.Print((uint8_t*) pending_status_payload, pending_status_payload_len, "[N4G][RX] payload");
			}

			switch (pending_status_frame_id)
			{
				case NWY_STATUS_UART_READY:
					debug.Print("[N4G][RX] uart_ready=1\r\n");
					break;
				case NWY_STATUS_SIM:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] sim_status=0x%02X\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_SIGNAL:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] signal=%u\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_INTERNET:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] internet_status=0x%02X\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_SD_CARD:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] sd_status=0x%02X\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_MQTT:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] mqtt_status=0x%02X\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_DATA_PUSH:
					if (pending_status_payload_len >= 1U)
					{
						debug.snprint("[N4G][RX] publish_status=0x%02X\r\n", (unsigned) pending_status_payload[0]);
					}
					break;
				case NWY_STATUS_TIME:
					if (status_snapshot.time_fields.valid)
					{
						debug.snprint("[N4G][RX] time=%02u:%02u:%02u\r\n",
								(unsigned) status_snapshot.time_fields.hour,
								(unsigned) status_snapshot.time_fields.min,
								(unsigned) status_snapshot.time_fields.sec);
					}
					break;
				case NWY_STATUS_DATE:
					if (status_snapshot.date_fields.valid)
					{
						debug.snprint("[N4G][RX] date=%02u/%02u/%04lu\r\n",
								(unsigned) status_snapshot.date_fields.day,
								(unsigned) status_snapshot.date_fields.month,
								(unsigned long) status_snapshot.date_fields.year);
					}
					break;
				default:
					break;
			}
		}

		void poll_status_rx_registers()
		{
			if (NEOUART == NULL)
			{
				return;
			}

			if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_ORE) == SET)
			{
				__HAL_UART_CLEAR_OREFLAG(NEOUART);
				debug.Print("[N4G][RAW_RX] overrun\r\n");
			}

			if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_FE) == SET)
			{
				__HAL_UART_CLEAR_FEFLAG(NEOUART);
				debug.Print("[N4G][RAW_RX] framing_error\r\n");
			}

			if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_NE) == SET)
			{
				__HAL_UART_CLEAR_NEFLAG(NEOUART);
				debug.Print("[N4G][RAW_RX] noise_error\r\n");
			}

			if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_PE) == SET)
			{
				__HAL_UART_CLEAR_PEFLAG(NEOUART);
				debug.Print("[N4G][RAW_RX] parity_error\r\n");
			}

			for (uint32_t count = 0; count < 64U; ++count)
			{
				if (__HAL_UART_GET_FLAG(NEOUART, UART_FLAG_RXNE) != SET)
				{
					break;
				}

				uint8_t rx_byte = (uint8_t) (NEOUART->Instance->RDR & 0xFFU);
				stage_raw_rx_byte(rx_byte);
				stage_pending_status_byte(rx_byte);
			}
		}

		bool send_status_frame(uint8_t frame_id, const uint8_t *payload, uint16_t payload_len)
		{
			if ((NEOUART == NULL) || status_tx_in_progress || (payload_len > NWY_STATUS_MAX_PAYLOAD_LEN)
					|| ((payload_len > 0U) && (payload == NULL)))
			{
				return false;
			}

			status_tx_frame[0] = frame_id;
			status_tx_frame[1] = (uint8_t) payload_len;
			if (payload_len > 0U)
			{
				memcpy(&status_tx_frame[2], payload, payload_len);
			}

			uint16_t crc = crc16_ccitt(status_tx_frame, (uint32_t) (2U + payload_len));
			status_tx_frame[2U + payload_len] = (uint8_t) (crc >> 8);
			status_tx_frame[3U + payload_len] = (uint8_t) (crc & 0xFF);

			HAL_StatusTypeDef tx = HAL_UART_Transmit_IT(NEOUART, status_tx_frame,
					(uint16_t) (payload_len + 4U));
			if (tx != HAL_OK)
			{
				return false;
			}

			status_tx_in_progress = true;
			return true;
		}

		bool arm_status_rx(bool log_failure)
		{
			(void) log_failure;

			if (NEOUART == NULL)
			{
				status_rx_armed = false;
				return false;
			}

			HAL_StatusTypeDef rx_status = HAL_UART_Receive_IT(NEOUART, &status_rx_byte, 1);
			status_rx_armed = (rx_status == HAL_OK);
			if (status_rx_armed)
			{
				return true;
			}

			return false;
		}

		bool send_mqtt_status_check()
		{
			const uint8_t query_payload[1] = { NWY_MQTT_STATUS_CHECK_QUERY };
			return send_status_frame((uint8_t) NWY_STATUS_MQTT, query_payload, 1U);
		}

		static uint16_t read_u16_be(const uint8_t *p)
				{
			return (uint16_t) ((uint16_t) p[0] << 8 | (uint16_t) p[1]);
		}

		static uint32_t read_u32_be(const uint8_t *p)
				{
			return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | (uint32_t) p[3];
		}

		static const char* status_frame_name(uint8_t frame_id)
		{
			switch (frame_id)
			{
				case NWY_STATUS_UART_READY:
					return "UART_READY";
				case NWY_STATUS_SIM:
					return "SIM";
				case NWY_STATUS_SIGNAL:
					return "SIGNAL";
				case NWY_STATUS_INTERNET:
					return "INTERNET";
				case NWY_STATUS_SD_CARD:
					return "SD_CARD";
				case NWY_STATUS_MQTT:
					return "MQTT";
				case NWY_STATUS_DATA_PUSH:
					return "DATA_PUSH";
				case NWY_STATUS_TIME:
					return "TIME";
				case NWY_STATUS_DATE:
					return "DATE";
				default:
					return "UNKNOWN";
			}
		}

		static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
				{
			uint16_t crc = 0xFFFF;
			for (uint32_t i = 0; i < len; ++i)
					{
				crc ^= (uint16_t) data[i] << 8;
				for (int bit = 0; bit < 8; ++bit)
						{
					if (crc & 0x8000)
							{
						crc = (uint16_t) ((crc << 1) ^ 0x1021);
					}
					else
					{
						crc = (uint16_t) (crc << 1);
					}
				}
			}
			return crc;
		}

		void reset_parser()
		{
			parser.frame_type = NWY_FRAME_NONE;
			parser.expected_len = 0;
			parser.received_len = 0;
			memset(parser.buffer, 0, sizeof(parser.buffer));
		}

		void begin_frame(nwy_frame_type_t frame_type, uint8_t first_byte)
		{
			reset_parser();
			parser.frame_type = frame_type;
			parser.buffer[0] = first_byte;
			parser.received_len = 1U;
			debug.snprint("[N4G][PARSER] start type=%u first=0x%02X\r\n",
					(unsigned) frame_type,
					(unsigned) first_byte);
		}

		static bool is_status_frame_id(uint8_t frame_id)
		{
			return (frame_id >= (uint8_t) NWY_STATUS_UART_READY)
					&& (frame_id <= (uint8_t) NWY_STATUS_DATE);
		}

		static bool compute_modbus_a6_frame_len(uint8_t payload_len, uint8_t flags,
				size_t *out_frame_len)
		{
			size_t header_len = 3U;
			if ((flags & NWY_MODBUS_FLAG_MSG_ID_PRESENT) != 0U)
			{
				header_len += 4U;
			}
			if ((flags & NWY_MODBUS_FLAG_TOKEN_PRESENT) != 0U)
			{
				header_len += 4U;
			}
			if ((flags & NWY_MODBUS_FLAG_IP_PRESENT) != 0U)
			{
				header_len += 4U;
			}

			size_t frame_len = header_len + (size_t) payload_len;
			if ((frame_len < 3U) || (frame_len > NWY_MODBUS_MAX_FRAME_LEN))
			{
				return false;
			}

			if (out_frame_len != NULL)
			{
				*out_frame_len = frame_len;
			}
			return true;
		}

		void process_modbus_a6_frame(const uint8_t *frame, size_t len)
		{
			if ((frame == NULL) || (len < 3U) || (len > NWY_MODBUS_MAX_FRAME_LEN))
			{
				debug.snprint("[N4G][A6] reject len=%u\r\n", (unsigned) len);
				return;
			}

			note_valid_rx_message();
			debug.snprint("[N4G][A6] parsed len=%u flags=0x%02X payload=%u\r\n",
					(unsigned) len,
					(unsigned) frame[2],
					(unsigned) frame[1]);
			(void) STM32_RS485_BridgeHandleReverseRxFrame(frame, (uint16_t) len);
		}

		void reset_status()
		{
			status_snapshot.uart_ready = false;
			status_snapshot.sim_status = NWY_SIM_ERROR;
			status_snapshot.signal_strength = 0;
			status_snapshot.internet_status = NWY_INTERNET_ERROR;
			status_snapshot.sd_status = NWY_SD_NO_CARD;
			status_snapshot.mqtt_status = NWY_MQTT_DISCONNECTED;
			status_snapshot.publish_status = NWY_PUBLISH_FAIL;
			status_snapshot.time_fields.valid = false;
			status_snapshot.date_fields.valid = false;
		}

		void process_status_frame(const uint8_t *frame, size_t len)
				{
			if (len < 4)
					{
				debug.snprint("[N4G][STATUS] reject_short len=%u\r\n", (unsigned) len);
				return;
			}

			uint8_t frame_id = frame[0];
			uint16_t payload_len = frame[1];
			size_t total_len = (size_t) payload_len + 4U;
			if (total_len != len)
					{
				debug.snprint("[N4G][STATUS] len_mismatch id=0x%02X payload=%u len=%u total=%u\r\n",
						(unsigned) frame_id,
						(unsigned) payload_len,
						(unsigned) len,
						(unsigned) total_len);
				return;
			}

			uint16_t crc_rx = (uint16_t) (((uint16_t) frame[len - 2] << 8) | frame[len - 1]);
			uint16_t crc_calc = crc16_ccitt(frame, (uint32_t) (2U + payload_len));
			if (crc_rx != crc_calc)
					{
				debug.snprint("[N4G][STATUS] crc_mismatch id=0x%02X rx=0x%04X calc=0x%04X len=%u\r\n",
						(unsigned) frame_id,
						(unsigned) crc_rx,
						(unsigned) crc_calc,
						(unsigned) len);
				return;
			}

			debug.snprint("[N4G][STATUS] parsed id=0x%02X payload=%u len=%u\r\n",
					(unsigned) frame_id,
					(unsigned) payload_len,
					(unsigned) len);
			handle_status_frame(frame_id, &frame[2], payload_len);
			(void) STM32_RS485_BridgeHandleReverseRxFrame(frame, (uint16_t) len);
		}

		void handle_status_frame(uint8_t frame_id, const uint8_t *payload, uint16_t payload_len)
				{
			note_valid_rx_message();

			switch (frame_id)
			{
				case NWY_STATUS_UART_READY:
					status_snapshot.uart_ready = true;
					break;
				case NWY_STATUS_SIM:
					if (payload_len >= 1)
							{
						status_snapshot.sim_status = (nwy_sim_status_t) payload[0];
					}
					break;
				case NWY_STATUS_SIGNAL:
					if (payload_len >= 1)
							{
						status_snapshot.signal_strength = payload[0];
					}
					break;
				case NWY_STATUS_INTERNET:
					if (payload_len >= 1)
							{
						status_snapshot.internet_status = (nwy_internet_status_t) payload[0];
					}
					break;
				case NWY_STATUS_SD_CARD:
					if (payload_len >= 1)
							{
						status_snapshot.sd_status = (nwy_sd_status_t) payload[0];
					}
					break;
				case NWY_STATUS_MQTT:
					if (payload_len >= 1)
							{
						status_snapshot.mqtt_status = (nwy_mqtt_status_t) payload[0];
						if (mqtt_status_check_pending)
						{
							if (payload[0] == (uint8_t) NWY_MQTT_CONNECTED)
							{
								note_mqtt_status_check_connected();
							}
							else
							{
								debug.snprint("[N4G] MQTT status check failed: 0x%02X; restarting module\r\n",
										(unsigned) payload[0]);
								RESTART();
								return;
							}
						}
					}
					break;
				case NWY_STATUS_DATA_PUSH:
					if (payload_len >= 1)
							{
						status_snapshot.publish_status = (nwy_publish_status_t) payload[0];
					}
					break;
				case NWY_STATUS_TIME:
					if (payload_len >= 6)
							{
						status_snapshot.time_fields.valid = true;
						status_snapshot.time_fields.hour = read_u16_be(payload);
						status_snapshot.time_fields.min = read_u16_be(payload + 2);
						status_snapshot.time_fields.sec = read_u16_be(payload + 4);
					}
					break;
				case NWY_STATUS_DATE:
					if (payload_len >= 8)
							{
						status_snapshot.date_fields.valid = true;
						status_snapshot.date_fields.day = read_u16_be(payload);
						status_snapshot.date_fields.month = read_u16_be(payload + 2);
						status_snapshot.date_fields.year = read_u32_be(payload + 4);
					}
					break;
				default:
					break;
			}

			stage_status_log(frame_id, payload, payload_len);

			if (on_status_frame != NULL)
			{
				on_status_frame(frame_id, payload, payload_len);
			}
		}
};
/*

 #if NEOWAY_SELF_MODE_ENABLE == 1
 extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
 extern NEOWAY neoway;
 if (huart == neoway.NEOUART)
 {
 neoway.STATUS_RX_IRQ_Handler();
 }
 }
 #endif  NEOWAY_SELF_MODE_ENABLE
 */

#endif /* NEOWAY_SELF_MODE_ENABLE */

#endif /* INC_NEOWAY_H_ */
