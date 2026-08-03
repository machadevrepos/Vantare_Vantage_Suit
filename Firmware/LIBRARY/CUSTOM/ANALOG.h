/*
 * ANALOG.h
 *
 *  Created on: May 3, 2025
 *      Author: dhanv
 */

#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include <stm32wbxx_hal.h>
//#include <linked_list.h>

// #ifdef __cplusplus

// Configuration macro to enable/disable  DMA functionality
//#define USE_ANALOG_DMA

#if defined(USE_ANALOG_DMA)
// Maximum number of channels for DMA operation
#ifndef ANALOG_DMA_MAX_CHANNELS
#define ANALOG_DMA_MAX_CHANNELS 16
#endif

// DMA buffer size per channel
#ifndef ANALOG_DMA_BUFFER_SIZE
#define ANALOG_DMA_BUFFER_SIZE 1
#endif

// Digital scale for 12-bit resolution
#define ANALOG_DIGITAL_SCALE_12BITS 0xFFF
#define ANALOG_VAR_CONVERTED_DATA_INIT_VALUE (ANALOG_DIGITAL_SCALE_12BITS + 1)
#endif

class ANALOG {
		ADC_ChannelConfTypeDef ADC_Channel_conf;
		ADC_HandleTypeDef *adc_handle;
		double factor = 1;  //!< a double value to amplify analog value read by sensor,if necessary.Initially initialized to 1
		double offset = 0;  //!< a double value to set offset to analog value read by sensor,if necessary.Initially initialized to 0,that is no offset.
		uint32_t raw_offset = 0;

		double threshold_high = -1;
		double threshold_low = -1;

		static double vref_int;
		uint8_t rank;

		volatile uint32_t last_raw = 0;
		volatile double last_converted = 0;

#if defined(USE_ANALOG_DMA)
		// DMA related static variables
		static uint32_t dma_buffer[ANALOG_DMA_MAX_CHANNELS * ANALOG_DMA_BUFFER_SIZE];
		//		static volatile uint8_t dma_transfer_status;  // 0: not completed, 1: completed, 2: not started
		static ADC_ChannelConfTypeDef dma_channels[ANALOG_DMA_MAX_CHANNELS];
		static uint8_t dma_channel_count;
		static ANALOG *dma_instances[ANALOG_DMA_MAX_CHANNELS];
		static uint32_t *dma_channel_ranks;
		#endif

//		  sConfig.Channel = ADC_CHANNEL_1;
//		  sConfig.Rank = ADC_REGULAR_RANK_1;
//		  sConfig.SamplingTime = ADC_SAMPLETIME_68CYCLES;
//		  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//		  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//		  sConfig.Offset = 0;

	public:
		ANALOG(ADC_HandleTypeDef *adc_handle, uint32_t channel, uint32_t rank = ADC_REGULAR_RANK_1, uint32_t sampling_time = ADC_SAMPLETIME_6CYCLES_5) :
				adc_handle(adc_handle) {
			ADC_Channel_conf.Channel = channel;
			ADC_Channel_conf.Rank = rank;
			ADC_Channel_conf.SamplingTime = sampling_time;
			ADC_Channel_conf.OffsetNumber = ADC_OFFSET_NONE;
			ADC_Channel_conf.Offset = 0;
		}

		static void SET_VERF_INT(double vref_int) {
			ANALOG::vref_int = vref_int;
		}

		double READ() {
			double analog_value = 0;

			if (HAL_ADC_ConfigChannel(adc_handle, &ADC_Channel_conf) != HAL_OK) {
				Error_Handler();
			}
			HAL_ADC_Start(adc_handle);

			if (HAL_ADC_PollForConversion(adc_handle, 1000) == HAL_OK) {
				uint32_t ang_value_raw = HAL_ADC_GetValue(adc_handle);
				analog_value = ang_value_raw + raw_offset;
			} else {
				analog_value = -1;
			}
			HAL_ADC_Stop(adc_handle);

			return analog_value;
		}

		double AVERAGE(int num_samples = 10) {
			double average = 0;
			for (int i = 0; i < num_samples; i++) {
				average += READ();
				delay_ms(10);
			}
			average /= num_samples;
			return average;
		}

		double READ_VOLTAGE() {
			double analog_value = 0;

			if (HAL_ADC_ConfigChannel(adc_handle, &ADC_Channel_conf) != HAL_OK) {
				Error_Handler();
			}
			HAL_ADC_Start(adc_handle);

			if (HAL_ADC_PollForConversion(adc_handle, 1000) == HAL_OK) {
				uint32_t analog_value_raw = HAL_ADC_GetValue(adc_handle);
				analog_value = __HAL_ADC_CALC_DATA_TO_VOLTAGE( vref_int, analog_value_raw, adc_handle->Init.Resolution);
			} else {
				analog_value = -1;
			}
			HAL_ADC_Stop(adc_handle);

			return analog_value;
		}

		double CONVERT() {
			double analog_value = READ();
			analog_value = (analog_value * factor) + offset;
			if (analog_value > threshold_high && threshold_high != -1) {
				analog_value = threshold_high;
			}
			if (analog_value < threshold_low && threshold_low != -1) {
				analog_value = threshold_low;
			}
			return analog_value;
		}

		double CONVERT_AVERAGE(int num_samples = 10) {
			double average = 0;
			for (int i = 0; i < num_samples; i++) {
				average += CONVERT();
				delay_ms(50);
			}
			average /= num_samples;
			return average;
		}

		void STABALISE() {
			double avg[100];
			for (int i = 0; i < 100; i++) {
				avg[i] = READ();
				delay_ms(10);
				if (avg[i] - avg[i - 1] > 10 || avg[i] - avg[i - 1] < -10) {
//					i--;
				}
			}
		}

		void SET_factor(double factor = 1) {
			this->factor = factor;
		}

		void SET_offset(double offset = 0) {
			this->offset = offset;
		}

		void SET_raw_offset(uint32_t raw_offset = 0) {
			this->raw_offset = raw_offset;
		}

		void set_config(double factor = 1, double offset = 0, uint32_t raw_offset = 0) {
			this->factor = factor;
			this->offset = offset;
			this->raw_offset = raw_offset;
		}
		void SET_threshold(double threshold_low = -1, double threshold_high = -1) {
			this->threshold_high = threshold_high;
			this->threshold_low = threshold_low;
		}

		uint32_t GET_last_raw() {
			return last_raw;
		}
		double GET_last_converted() {
			return last_converted;
		}

#if defined(USE_ANALOG_DMA)
		// DMA methods
		static HAL_StatusTypeDef INIT_DMA(ADC_HandleTypeDef *adc_handle, ANALOG *instances[], uint8_t num_channels);
		static HAL_StatusTypeDef READ_DMA_MULTI(double *results, uint8_t num_channels);
		static HAL_StatusTypeDef READ_DMA_MULTI_NULL(uint8_t num_channels);
		static HAL_StatusTypeDef READ_DMA_MULTI_VOLTAGE(double *results, uint8_t num_channels);
		static HAL_StatusTypeDef READ_DMA_MULTI_VOLTAGE_NULL(uint8_t num_channels);
		static void ADD_CHANNEL_TO_DMA(ANALOG *instance);
		static void CLEAR_DMA_CHANNELS();
		static uint8_t GET_DMA_STATUS();
		static void GET_READINGS(uint8_t num_channels);

		// DMA callback handlers
		static void DMA_ConvCpltCallback(DMA_HandleTypeDef *hadc);
		static void DMA_ConvHalfCpltCallback(DMA_HandleTypeDef *hadc);
		static void DMA_ErrorCallback(DMA_HandleTypeDef *hadc);
#endif
};

double ANALOG::vref_int = 3.3;

#if defined(USE_ANALOG_DMA)
// Static variable definitions for DMA
uint32_t ANALOG::dma_buffer[ANALOG_DMA_MAX_CHANNELS * ANALOG_DMA_BUFFER_SIZE];
volatile uint8_t dma_transfer_status = 2;  // Initially not started
ADC_ChannelConfTypeDef ANALOG::dma_channels[ANALOG_DMA_MAX_CHANNELS];
uint8_t ANALOG::dma_channel_count = 0;
ANALOG *ANALOG::dma_instances[ANALOG_DMA_MAX_CHANNELS] = { nullptr };
uint32_t *ANALOG::dma_channel_ranks = nullptr;
extern DMA_QListTypeDef ADCQueue;

HAL_StatusTypeDef ANALOG::INIT_DMA(ADC_HandleTypeDef *adc_handle, ANALOG *instances[], uint8_t num_channels) {

	MX_ADCQueue_Config();
	__HAL_LINKDMA(&hadc1, DMA_Handle, handle_GPDMA1_Channel0);
	if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &ADCQueue) != HAL_OK)
			{
		Error_Handler();
	}

	for (uint16_t i = 0; i < ANALOG_DMA_MAX_CHANNELS * ANALOG_DMA_BUFFER_SIZE; i++) {
		dma_buffer[i] = ANALOG_VAR_CONVERTED_DATA_INIT_VALUE;
	}

	for (uint8_t i = 0; i < num_channels; i++) {
		dma_instances[i] = instances[i];
		instances[i]->rank = i;

	}

	dma_channel_count = num_channels;
	dma_transfer_status = 2;  // Ready to start

	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel0, HAL_DMA_XFER_CPLT_CB_ID, DMA_ConvCpltCallback);
	HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel0, HAL_DMA_XFER_HALFCPLT_CB_ID, DMA_ConvHalfCpltCallback);

	return HAL_OK;
}

// Multi-channel DMA read method
HAL_StatusTypeDef ANALOG::READ_DMA_MULTI(double *results, uint8_t num_channels) {
	if (num_channels != dma_channel_count || results == nullptr) {
		return HAL_ERROR;
	}

	ADC_HandleTypeDef *adc_handle = dma_instances[0]->adc_handle;

// Start DMA transfer
	dma_transfer_status = 0;  // Transfer in progress

	if (HAL_ADC_Start_DMA(adc_handle, dma_buffer, num_channels) != HAL_OK) {
		return HAL_ERROR;
	}
// Wait for completion (with timeout)
	uint32_t timeout = HAL_GetTick() + 1000;  // 1 second timeout
	while (dma_transfer_status == 0) {
		if (HAL_GetTick() > timeout) {
			HAL_ADC_Stop_DMA(adc_handle);
			return HAL_TIMEOUT;
		}
	}

	if (dma_transfer_status == 1) {  // Transfer completed successfully
		for (uint8_t i = 0; i < num_channels; i++) {
			dma_instances[i]->last_raw = dma_buffer[i] + dma_instances[i]->raw_offset;
			results[i] = (dma_instances[i]->last_raw * dma_instances[i]->factor) + dma_instances[i]->offset;

			if (dma_instances[i]->threshold_high != -1 && results[i] > dma_instances[i]->threshold_high) {
				results[i] = dma_instances[i]->threshold_high;
			}
			if (dma_instances[i]->threshold_low != -1 && results[i] < dma_instances[i]->threshold_low) {
				results[i] = dma_instances[i]->threshold_low;
			}
			dma_instances[i]->last_converted = results[i];
		}
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef ANALOG::READ_DMA_MULTI_NULL(uint8_t num_channels) {
	if (num_channels != dma_channel_count) {
		return HAL_ERROR;
	}
	ADC_HandleTypeDef *adc_handle = dma_instances[0]->adc_handle;

	dma_transfer_status = 0;  // Transfer in progress

	if (HAL_ADC_Start_DMA(adc_handle, dma_buffer, num_channels) != HAL_OK) {
		return HAL_ERROR;
	}
	// Wait for completion (with timeout)
	uint32_t timeout = HAL_GetTick() + 1000;  // 1 second timeout
	while (dma_transfer_status == 0) {
		if (HAL_GetTick() > timeout) {
//			HAL_ADC_Stop_DMA(adc_handle);
			return HAL_TIMEOUT;
		}
	}
	if (dma_transfer_status == 1) {  // Transfer completed successfully
		for (uint8_t i = 0; i < num_channels; i++) {
			dma_instances[i]->last_raw = dma_buffer[i] + dma_instances[i]->raw_offset;
			dma_instances[i]->last_converted = (dma_instances[i]->last_raw * dma_instances[i]->factor) + dma_instances[i]->offset;

			if (dma_instances[i]->threshold_high != -1 && dma_instances[i]->last_converted > dma_instances[i]->threshold_high) {
				dma_instances[i]->last_converted = dma_instances[i]->threshold_high;
			}
			if (dma_instances[i]->threshold_low != -1 && dma_instances[i]->last_converted < dma_instances[i]->threshold_low) {
				dma_instances[i]->last_converted = dma_instances[i]->threshold_low;
			}
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}

void ANALOG::GET_READINGS(uint8_t num_channels) {
	for (uint8_t i = 0; i < num_channels; i++) {
		dma_instances[i]->last_raw = dma_buffer[i] + dma_instances[i]->raw_offset;
		dma_instances[i]->last_converted = (dma_instances[i]->last_raw * dma_instances[i]->factor) + dma_instances[i]->offset;

		if (dma_instances[i]->threshold_high != -1 && dma_instances[i]->last_converted > dma_instances[i]->threshold_high) {
			dma_instances[i]->last_converted = dma_instances[i]->threshold_high;
		}
		if (dma_instances[i]->threshold_low != -1 && dma_instances[i]->last_converted < dma_instances[i]->threshold_low) {
			dma_instances[i]->last_converted = dma_instances[i]->threshold_low;
		}
	}
}

HAL_StatusTypeDef ANALOG::READ_DMA_MULTI_VOLTAGE(double *results, uint8_t num_channels) {
	HAL_StatusTypeDef status = READ_DMA_MULTI_NULL(num_channels);
	if (status == HAL_OK) {
		for (uint8_t i = 0; i < num_channels; i++) {
			results[i] = dma_instances[i]->last_raw * (3.3 / 8190.0);
		}
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef
ANALOG::READ_DMA_MULTI_VOLTAGE_NULL(uint8_t num_channels) {
//	HAL_StatusTypeDef status =
			READ_DMA_MULTI_NULL(num_channels);
//	if (status == HAL_OK) {
	for (uint8_t i = 0; i < num_channels; i++) {
		dma_instances[i]->last_converted = ((dma_instances[i]->last_raw + dma_instances[i]->raw_offset) * dma_instances[i]->factor) + dma_instances[i]->offset;
	}
	return HAL_OK;
//	}

//	return HAL_ERROR;
}

// Add channel to DMA sequence
void ANALOG::ADD_CHANNEL_TO_DMA(ANALOG *instance) {
	if (dma_channel_count < ANALOG_DMA_MAX_CHANNELS) {
		dma_instances[dma_channel_count] = instance;
		dma_channels[dma_channel_count] = instance->ADC_Channel_conf;
		dma_channels[dma_channel_count].Rank = dma_channel_count + 1;
		dma_channel_count++;
	}
}

// Clear DMA channels
void ANALOG::CLEAR_DMA_CHANNELS() {
	dma_channel_count = 0;
	for (uint8_t i = 0; i < ANALOG_DMA_MAX_CHANNELS; i++) {
		dma_instances[i] = nullptr;
	}
}

// Get DMA transfer status
uint8_t
ANALOG::GET_DMA_STATUS()
{
	return dma_transfer_status;
}

// DMA completion callback
void ANALOG::DMA_ConvCpltCallback(DMA_HandleTypeDef *hadc) {
	dma_transfer_status = 1;  // Transfer completed
}

// DMA half completion callback
void ANALOG::DMA_ConvHalfCpltCallback(DMA_HandleTypeDef *hadc) {
// Can be used for continuous mode operations
}

// DMA error callback
void ANALOG::DMA_ErrorCallback(DMA_HandleTypeDef *hadc) {
	dma_transfer_status = 3;  // Error occurred
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ANALOG::DMA_ConvCpltCallback(hadc->DMA_Handle);
}
#endif // USE_ANALOG_DMA

// #endif // __cplusplus

#endif /* INC_ANALOG_H_ */
