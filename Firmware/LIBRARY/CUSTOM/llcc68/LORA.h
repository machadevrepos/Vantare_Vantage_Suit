/*
 * RTOS.h
 *
 *  Created on: Dec 19, 2024
 *      Author: dhanv
 */

#ifndef LORA_H_
#define LORA_H_

#include <CUSTOM/INCLUDER.h>
#include <CUSTOM/llcc68/driver_llcc68.h>
#include <CUSTOM/llcc68/driver_llcc68_interface.h>
#include <CUSTOM/llcc68/driver_llcc68_lora.h>

#include <CUSTOM/llcc68/driver_llcc68_sent_receive_test.h>
#pragma GCC push_options
#pragma GCC optimize("O0")

// #define SLS_RELAY
// #define SLS_HUB
// #define SLS_DALI

static String file_name_3 = "sr_test";
extern String device_id;
static volatile uint8_t gs_rx_done; /**< rx done */

uint32_t lora_period = 1000;

extern osMessageQueueId_t LORA_TX_QHandle;
extern osMessageQueueId_t LORA_RX_QHandle;

struct LORA_TX_STC
{
		char device_id[16];
		uint16_t device_id_size;
		char data[512];
		uint16_t data_size;
};
struct LORA_RX_STC
{
		char device_id[16];
		uint16_t device_id_size;
		char data[512];
		uint16_t data_size;
};

/**
 * @brief     interface receive callback
 * @param[in] type is the receive callback type
 * @param[in] *buf points to a buffer address
 * @param[in] len is the buffer length
 * @note      none
 */
static void a_callback(uint16_t type, uint8_t *buf, uint16_t len)
		{
	//	String lora_rec_data = "";
	switch (type)
	{
		case LLCC68_IRQ_TX_DONE:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq tx done.\r\n");

			break;
		}
		case LLCC68_IRQ_RX_DONE:
			{

			llcc68_bool_t enable;
			uint8_t rssi_pkt_raw;
			uint8_t snr_pkt_raw;
			uint8_t signal_rssi_pkt_raw;
			float rssi_pkt;
			float snr_pkt;
			float signal_rssi_pkt;

			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq rx done.\r\n");

			/* get the status */
			if (llcc68_get_lora_packet_status(&gs_handle, (uint8_t*) &rssi_pkt_raw, (uint8_t*) &snr_pkt_raw, (uint8_t*) &signal_rssi_pkt_raw, (float*) &rssi_pkt, (float*) &snr_pkt, (float*) &signal_rssi_pkt) != 0)
					{
				return;
			}
			printdebug(file_name_3);

			/* check the // error */
			if (llcc68_check_packet_error(&gs_handle, &enable) != 0)
					{
				return;
			}
//			String *send_str = new String();
			LORA_RX_STC rec_data;
			if ((enable == LLCC68_BOOL_FALSE) && len)
					{
//				for (i = 0; i < len; i++)
//						{
//					*send_str += buf[i];
//				}
				memcpy(rec_data.data, buf, len);
				rec_data.data[len] = '\0'; // Null-terminate the string
				rec_data.data_size = len;

//				printdebug(file_name_3);
				llcc68_interface_debug_print("\r\n");

				if (osMessageQueuePut(LORA_RX_QHandle, &rec_data, 0, 0) != osOK)
						{
//					delete send_str;
				}
				else
				{
					gs_rx_done = 1;
				}
			}

			break;
		}
		case LLCC68_IRQ_PREAMBLE_DETECTED:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq preamble detected.\r\n");

			break;
		}
		case LLCC68_IRQ_SYNC_WORD_VALID:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq valid sync word detected.\r\n");

			break;
		}
		case LLCC68_IRQ_HEADER_VALID:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq valid header.\r\n");

			break;
		}
		case LLCC68_IRQ_HEADER_ERR:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq header // error.\r\n");

			break;
		}
		case LLCC68_IRQ_CRC_ERR:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq crc // error.\r\n");

			break;
		}
		case LLCC68_IRQ_CAD_DONE:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq cad done.\r\n");

			break;
		}
		case LLCC68_IRQ_CAD_DETECTED:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq cad detected.\r\n");

			break;
		}
		case LLCC68_IRQ_TIMEOUT:
			{
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: irq timeout.\r\n");

			break;
		}
		default:
			{
			break;
		}
	}
}

class LORA
{

	public:
		LORA()
		{
			gs_rx_done = 0;
			link_interface();
		}
		/**
		 * @brief  llcc68 interrupt test irq
		 * @return status code
		 *         - 0 success
		 *         - 1 run failed
		 * @note   none
		 */
		uint8_t interrupt_handler(void)
				{
			if (llcc68_irq_handler(&gs_handle) != 0)
					{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		bool error_return(uint8_t res, uint32_t line)
				{
			if (res != 0)
					{
				printdebugl(file_name_3, line);
				return true;
			}
			return false;
		}
		uint8_t setup()
		{

			uint8_t res;
			uint32_t reg;
			uint8_t modulation;
			uint8_t config;

			link_interface();
			res = llcc68_init(&gs_handle);
			if (error_return(res, __LINE__))
				return 1;

			/* enter standby */
			res = llcc68_set_standby(&gs_handle, LLCC68_CLOCK_SOURCE_XTAL_32MHZ);
			if (error_return(res, __LINE__))
				return 1;

			/* disable stop timer on preamble */
			res = llcc68_set_stop_timer_on_preamble(&gs_handle, LLCC68_BOOL_FALSE);
			if (error_return(res, __LINE__))
				return 1;

			/* set dc dc ldo */
			res = llcc68_set_regulator_mode(&gs_handle, LLCC68_REGULATOR_MODE_DC_DC_LDO);
			if (error_return(res, __LINE__))
				return 1;

			/* set +22dBm power */
			res = llcc68_set_pa_config(&gs_handle, 0x04, 0x07);
			if (error_return(res, __LINE__))
				return 1;

			/* enter to stdby rc mode */
			res = llcc68_set_rx_tx_fallback_mode(&gs_handle, LLCC68_RX_TX_FALLBACK_MODE_STDBY_XOSC);
			if (error_return(res, __LINE__))
				return 1;

			/* set dio irq */
			res = llcc68_set_dio_irq_params(&gs_handle, 0x03FFU, 0x03FFU, 0x0000, 0x0000);
			if (error_return(res, __LINE__))
				return 1;

			/* clear irq status */
			res = llcc68_clear_irq_status(&gs_handle, 0x03FFU);
			if (error_return(res, __LINE__))
				return 1;

			/* set lora mode */
			res = llcc68_set_packet_type(&gs_handle, LLCC68_PACKET_TYPE_LORA);
			if (error_return(res, __LINE__))
				return 1;

			/* +17dBm */
			res = llcc68_set_tx_params(&gs_handle, 22, LLCC68_RAMP_TIME_40US);
			if (error_return(res, __LINE__))
				return 1;

			llcc68_lora_bandwidth_t lora_bw = LLCC68_LORA_BANDWIDTH_500_KHZ;
			/* sf9, 125khz, cr4/5, disable low data rate optimize */
			res = llcc68_set_lora_modulation_params(&gs_handle, LLCC68_LORA_SF_8, lora_bw, LLCC68_LORA_CR_4_5, LLCC68_BOOL_TRUE);
			if (error_return(res, __LINE__))
				return 1;

			/* convert the frequency */
			res = llcc68_frequency_convert_to_register(&gs_handle, 868000000U, (uint32_t*) &reg);
			if (error_return(res, __LINE__))
				return 1;

			/* set the frequency */
			res = llcc68_set_rf_frequency(&gs_handle, reg);
			if (error_return(res, __LINE__))
				return 1;

			/* set base address */
			res = llcc68_set_buffer_base_address(&gs_handle, 0x00, 0x00);
			if (res != 0)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: set buffer base address failed.\r\n");
				(void) llcc68_deinit(&gs_handle);
				// error++;
				return 1;
			}

			/* 1 lora symb num */
			res = llcc68_set_lora_symb_num_timeout(&gs_handle, 0);
			if (error_return(res, __LINE__))
				return 1;

			/* reset stats */
			res = llcc68_reset_stats(&gs_handle, 0x0000, 0x0000, 0x0000);
			if (error_return(res, __LINE__))
				return 1;

			/* clear device // errors */
			res = llcc68_clear_device_errors(&gs_handle);
			if (error_return(res, __LINE__))
				return 1;

			/* set the lora sync word */
			res = llcc68_set_lora_sync_word(&gs_handle, 0x1424U);
			if (error_return(res, __LINE__))
				return 1;

			/* get tx modulation */
			res = llcc68_get_tx_modulation(&gs_handle, (uint8_t*) &modulation);
			if (error_return(res, __LINE__))
				return 1;

			if (lora_bw == LLCC68_LORA_BANDWIDTH_500_KHZ)
					{
				modulation &= 0xFB;
			}
			else
			{
				modulation |= 0x04;
			}

			/* set the tx modulation */
			res = llcc68_set_tx_modulation(&gs_handle, modulation);
			if (error_return(res, __LINE__))
				return 1;

			/* set the rx gain */
			res = llcc68_set_rx_gain(&gs_handle, 0x94);
			if (error_return(res, __LINE__))
				return 1;

			/* set the ocp */
			res = llcc68_set_ocp(&gs_handle, 0x38);
			if (error_return(res, __LINE__))
				return 1;

			/* get the tx clamp config */
			res = llcc68_get_tx_clamp_config(&gs_handle, (uint8_t*) &config);
			if (error_return(res, __LINE__))
				return 1;

			config |= 0x1E;

			/* set the tx clamp config */
			res = llcc68_set_tx_clamp_config(&gs_handle, config);
			if (error_return(res, __LINE__)) {
				return 1;
			}

			llcc68_interface_debug_print("llcc68: setup done.\r\n");
			return 0;
		}

		/**
		 * @brief  sent test
		 * @return status code
		 *         - 0 success
		 *         - 1 test failed
		 * @note   none
		 */
		uint8_t send(String data)
				{
			uint8_t res;

			/* start sent test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start sent test.\r\n");

			/* init the llcc68 */
			NVIC_DisableIRQ(EXTI9_5_IRQn);

			setup();

			NVIC_EnableIRQ(EXTI9_5_IRQn);

			if (data.length() > 256)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: data too long.\r\n");
				return 1;
			}
			else
			{
				String data2 = (String) "llcc68: Sending: " + data + (String) "\r\n";
				llcc68_interface_debug_print(data2.c_str());
			}
			/* sent the data */
			res = llcc68_lora_transmit(&gs_handle, LLCC68_CLOCK_SOURCE_XTAL_32MHZ, 50, LLCC68_LORA_HEADER_EXPLICIT, LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE, (uint8_t*) data.c_str(), data.size(), 3000 * 1000);

			if (res != 0)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: lora sent failed.\r\n");
				(void) llcc68_deinit(&gs_handle);
				return res;
			}

			/* finish sent test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: finish sent test.\r\n");

			/* deinit */
			//	(void) llcc68_deinit(&gs_handle);
			return 0;
		}

		uint8_t send_cstr(uint8_t *buf, uint16_t len) {
			uint8_t res;

			/* start sent test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start sent test.\r\n");

			/* init the llcc68 */
			NVIC_DisableIRQ(EXTI9_5_IRQn);

			setup();

			NVIC_EnableIRQ(EXTI9_5_IRQn);

			if (len > 256)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: data too long.\r\n");
				return 1;
			}
			else
			{
				char data2[512];
				snprintf(data2, sizeof(data2), "llcc68: Sending: %.*s\r\n", len, buf);

				llcc68_interface_debug_print(data2);
			}
			/* sent the data */
			res = llcc68_lora_transmit(&gs_handle, LLCC68_CLOCK_SOURCE_XTAL_32MHZ, 50, LLCC68_LORA_HEADER_EXPLICIT, LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE, buf, len, 0);

			if (res != 0)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: lora sent failed.\r\n");
				(void) llcc68_deinit(&gs_handle);
				return res;
			}

			/* finish sent test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: finish sent test.\r\n");

			/* deinit */
			//	(void) llcc68_deinit(&gs_handle);
			return 0;
		}

		void link_interface()
		{
			/* link interface function */
			DRIVER_LLCC68_LINK_INIT(&gs_handle, llcc68_handle_t);
			DRIVER_LLCC68_LINK_SPI_INIT(&gs_handle, llcc68_interface_spi_init);
			DRIVER_LLCC68_LINK_SPI_DEINIT(&gs_handle, llcc68_interface_spi_deinit);
			DRIVER_LLCC68_LINK_SPI_WRITE_READ(&gs_handle, llcc68_interface_spi_write_read);
			DRIVER_LLCC68_LINK_RESET_GPIO_INIT(&gs_handle, llcc68_interface_reset_gpio_init);
			DRIVER_LLCC68_LINK_RESET_GPIO_DEINIT(&gs_handle, llcc68_interface_reset_gpio_deinit);
			DRIVER_LLCC68_LINK_RESET_GPIO_WRITE(&gs_handle, llcc68_interface_reset_gpio_write);
			DRIVER_LLCC68_LINK_BUSY_GPIO_INIT(&gs_handle, llcc68_interface_busy_gpio_init);
			DRIVER_LLCC68_LINK_BUSY_GPIO_DEINIT(&gs_handle, llcc68_interface_busy_gpio_deinit);
			DRIVER_LLCC68_LINK_BUSY_GPIO_READ(&gs_handle, llcc68_interface_busy_gpio_read);
			DRIVER_LLCC68_LINK_DELAY_MS(&gs_handle, llcc68_interface_delay_ms);
			DRIVER_LLCC68_LINK_DEBUG_PRINT(&gs_handle, llcc68_interface_debug_print);
			DRIVER_LLCC68_LINK_RECEIVE_CALLBACK(&gs_handle, a_callback);
		}

		/**
		 * @brief     receive test
		 * @param[in] s is the timeout time
		 * @return    status code
		 *            - 0 success
		 *            - 1 test failed
		 * @note      none
		 */
		uint8_t receive(uint32_t ms, String &data)
				{
			uint8_t res;

			uint8_t lora_config;

			/* start receive test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start receive test.\r\n");

			NVIC_DisableIRQ(EXTI9_5_IRQn);

			setup();

			/* set lora packet params */
			res = llcc68_set_lora_packet_params(&gs_handle, 50, LLCC68_LORA_HEADER_EXPLICIT, 255, LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE);
			if (error_return(res, __LINE__))
				return 1;

			/* get iq polarity */
			res = llcc68_get_iq_polarity(&gs_handle, (uint8_t*) &lora_config);
			if (error_return(res, __LINE__))
				return 1;

			lora_config |= 1 << 2;

			/* set the iq polarity */
			res = llcc68_set_iq_polarity(&gs_handle, lora_config);
			if (error_return(res, __LINE__))
				return 1;

			/* start receive */
			res = llcc68_continuous_receive(&gs_handle);
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			if (error_return(res, __LINE__))
				return 1;

			/* start receiving */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start receiving...\r\n");
			gs_rx_done = 0;

			String *rs485_stc_temp = NULL;
			if (osMessageQueueGet(LORA_RX_QHandle, rs485_stc_temp, 0, ms) == osOK)
					{
				if (rs485_stc_temp == NULL)
				{
					printdebug(file_name_3);
					return 1;
				}
				data = *rs485_stc_temp;
				delete rs485_stc_temp;
				return 0;
			}
			return 1;
		}

		/**
		 * @brief     receive test
		 * @param[in] s is the timeout time
		 * @return    status code
		 *            - 0 success
		 *            - 1 test failed
		 * @note      none
		 */
		uint8_t receive_once(uint32_t ms, String &data)
				{
			uint8_t res;

			uint8_t lora_config;

			/* start receive test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start receive test.\r\n");

			NVIC_DisableIRQ(EXTI9_5_IRQn);

			setup();

			/* set lora packet params */
			res = llcc68_set_lora_packet_params(&gs_handle, 50, LLCC68_LORA_HEADER_EXPLICIT, 255, LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE);
			if (error_return(res, __LINE__))
				return 1;

			/* get iq polarity */
			res = llcc68_get_iq_polarity(&gs_handle, (uint8_t*) &lora_config);
			if (error_return(res, __LINE__))
				return 1;

			lora_config |= 1 << 2;

			/* set the iq polarity */
			res = llcc68_set_iq_polarity(&gs_handle, lora_config);
			if (error_return(res, __LINE__))
				return 1;

			/* start receive */
			res = llcc68_single_receive(&gs_handle, ms * us_s);
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			if (res != 0)
					{
				printdebug(file_name_3);
				llcc68_interface_debug_print("llcc68: lora continuous receive failed.\r\n");
				(void) llcc68_deinit(&gs_handle);
				return 1;
			}

			/* start receiving */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start receiving...\r\n");
			gs_rx_done = 0;
			String *rs485_stc_temp = NULL;
			if (osMessageQueueGet(LORA_TX_QHandle, rs485_stc_temp, 0, ms) == osOK)
					{
				if (rs485_stc_temp == NULL)
				{
					printdebug(file_name_3);
					return 1;
				}
				data = *rs485_stc_temp;
				delete rs485_stc_temp;
				return 0;
			}
			return 1;
		}

		/**
		 * @brief     receive test
		 * @param[in] s is the timeout time
		 * @return    status code
		 *            - 0 success
		 *            - 1 test failed
		 * @note      none
		 */
		uint8_t start_receive()
		{
			uint8_t res;

			uint8_t lora_config;

			/* start receive test */
			printdebug(file_name_3);
			llcc68_interface_debug_print("llcc68: start receive test.\r\n");

			NVIC_DisableIRQ(EXTI9_5_IRQn);

			setup();

			/* set lora packet params */
			res = llcc68_set_lora_packet_params(&gs_handle, 50, LLCC68_LORA_HEADER_EXPLICIT, 255, LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE);
			if (error_return(res, __LINE__))
				return 1;

			/* get iq polarity */
			res = llcc68_get_iq_polarity(&gs_handle, (uint8_t*) &lora_config);
			if (error_return(res, __LINE__))
				return 1;

			lora_config |= 1 << 2;

			/* set the iq polarity */
			res = llcc68_set_iq_polarity(&gs_handle, lora_config);
			if (error_return(res, __LINE__))
				return 1;

			/* start receive */
			res = llcc68_continuous_receive(&gs_handle);
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			if (res != 0)
					{
				printdebug(file_name_3);
				(void) llcc68_deinit(&gs_handle);
				return 1;
			}

			return 0;
		}

		/**
		 * @brief     receive test
		 * @param[in] s is the timeout time
		 * @return    status code
		 *            - 0 success
		 *            - 1 test failed
		 * @note      none
		 */
		uint8_t get_data(uint32_t ms, LORA_RX_STC &rec_data)
				{

			/* start receiving */
			printdebug(file_name_3);
			gs_rx_done = 0;

//			LORA_RX_STC rec_data;
			if (osMessageQueueGet(LORA_RX_QHandle, &rec_data, 0, pdMS_TO_TICKS(ms)) == osOK)
					{
				if (rec_data.data_size == 0)
						{
					printdebug(file_name_3);
					return 1;
				}
//				uint32_t i = 0;
//				while (rec_data.data[i] != '\0')
//				{
//					data += (rs485_stc_temp)[i];
//					if (i >= 255)
//							{
//						break;
//					}
//					i++;
//				}
				//				data = rs485_stc_temp;
				//				delete rs485_stc_temp;
				return 0;
			}
			return 1;
		}
};

LORA lora;

#define TEST_LORA
#if defined(TEST_LORA)
void LORA_TASK(void *argument)
		{
	while (1)
	{

		device_id = d_t_h_s(config_uid.as<uint32_t>());

#if defined(SLS_HUB)
				// Red led for sending
				// green for sucessful reception
				// blue for failed reception
				rgb.SET(1, 0, 0);

				for (JsonPair spoke_data : config_spokes.as<JsonObject>())
				{

					volatile uint32_t last_tick = HAL_GetTick();
					String lora_send_data = device_id + "_" + spoke_data.value().as<String>();
					debug.Print("Sending : " + lora_send_data + "\r\n");
					lora.send(lora_send_data);
					rgb.OFF();

//					String lora_data = "";
					LORA_RX_STC lora_data;
					int times_received = 0;
					lora.start_receive();
					//					debug.Print("Lora Data Sent\r\n");

					if (lora.get_data(lora_period + 100, lora_data) == 0)
					{
						debug.snprint("\t\tReceived : %s\r\n", lora_data.data);
						times_received++;
//						lora_data = "";
						memset(lora_data.data, 0, sizeof(lora_data.data));
					}

					while (HAL_GetTick() - last_tick < (lora_period))
					{
						vTaskDelay(pdMS_TO_TICKS(10));
					}
					vTaskDelay(pdMS_TO_TICKS(200));
				}

				vTaskDelay(pdMS_TO_TICKS(200));
#endif

#if defined(SLS_DALI) || defined(SLS_RELAY)

//				String lora_data = "";
				LORA_RX_STC lora_data;
				lora.start_receive();
				debug.Print("llcc68_start_receive\r\n");
				if (lora.get_data(lora_period *2, lora_data) == 0)
							{
								debug.snprint("\t\tReceived : %s\r\n", lora_data.data);

					rgb.PULSE(0, 1, 0, lora_period / 2);

					uint32_t hub_id = 0;
					uint32_t spoke_id = 0;
					sscanf(lora_data.data, "%lX_%lX", &hub_id, &spoke_id);
					debug.Print("Hub ID : " + d_t_h_s(hub_id) + " SpokeID: " + d_t_h_s(spoke_id) + "\r\n");
					if (spoke_id == config_uid.as<uint32_t>())
					{
						debug.Print("My Turn\r\n");

						// Red led for sending
						// green for sucessful reception
						// blue for failed reception
						rgb.SET(1, 0, 0);
						String lora_send_data = device_id + "_" + d_t_h_s(hub_id) + "_" + generate_random_string();
						lora.send(lora_send_data);
						rgb.OFF();
					}
					//					else {
					////						rgb.PULSE(0, 0, 1, lora_period/3);
					//					}
				}

//				lora_data = "";
				memset(lora_data.data, 0, sizeof(lora_data.data));

				//		relay_1.Toggle(100 * us_ms);
				//		relay_2.Toggle(100 * us_ms);

				vTaskDelay(pdMS_TO_TICKS(100));

#endif
			}
		}
#else /* TEST_LORA */
void LORA_TASK(void *argument)
		{
	while (1)
	{

		device_id = config_uid.as<String>();

#if defined(SLS_HUB)
				// Red led for sending
				// green for sucessful reception
				// blue for failed reception
				LORA_TX_STC new_data;
				// waits message from Quectel Task
				if (osMessageQueueGet(LORA_TX_QHandle, &new_data, 0, 10000) != osOK)
				{
					debug.Print("No LORA message\r\n");
					continue;
				}
				rgb.SET(1, 0, 0);

				volatile uint32_t last_tick = HAL_GetTick();
				char lora_send_data[1024] = {0};
				snprintf(lora_send_data, sizeof(lora_send_data), "%s_%s_%s", device_id.c_str(), new_data.device_id, new_data.data);

				bool received = false;
				uint8_t times_tried= 0;
				do {
					debug.snprint("\r\n In Lora Sending : %s\r\n", lora_send_data);
					lora.send( lora_send_data);
					rgb.OFF();

					LORA_RX_STC rec_data;
					int times_received = 0;
					lora.start_receive();

					if (lora.get_data(lora_period + 100, rec_data) == 0)
					{
						debug.snprint("\t\tReceived : %s\r\n", rec_data.data);
						received = true;
						times_received++;
					}

					while (HAL_GetTick() - last_tick < (lora_period))
					{
						vTaskDelay(pdMS_TO_TICKS(10));
					}
				}while(received==false && times_tried++ < 3);

#endif

#if defined(SLS_DALI) || defined(SLS_RELAY)

				LORA_RX_STC rec_data;
				lora.start_receive();
				debug.Print("llcc68_start_receive\r\n");
				if (lora.get_data(lora_period, rec_data) == 0)
				{
					debug.snprint("Received : %s\r\n", rec_data.data);

					rgb.PULSE(0, 1, 0, lora_period / 2);

					uint32_t hub_id = 0;
					uint32_t spoke_id = 0;
					sscanf(rec_data.data, "%lX_%lX", &hub_id, &spoke_id);
					debug.Print("Hub ID : " + d_t_h_s(hub_id) + " SpokeID: " + d_t_h_s(spoke_id) + "\r\n");
					if (spoke_id == config_uid.as<uint32_t>())
					{
						debug.Print("My Turn\r\n");

						// Red led for sending
						// green for sucessful reception
						// blue for failed reception
						rgb.SET(1, 0, 0);
						String lora_send_data = device_id + "_" + d_t_h_s(hub_id) + "_" + generate_random_string();
						lora.send(lora_send_data);
						rgb.OFF();
					}

				}


				vTaskDelay(pdMS_TO_TICKS(100));

#endif
			}
		}
#endif /* TEST_LORA */

#pragma GCC pop_options

#endif /* LLCC68_LORA_H_ */
