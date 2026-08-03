/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      spi.c
 * @brief     spi source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-11-11
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/11  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef C_SPI_H
#define C_SPI_H

#include <spi.h>
//#include "llcc68/c_spi.h"

/**
 * @brief spi mode enumeration definition
 */
typedef enum {
	SPI_MODE_0 = 0x00, /**< mode 0 */
	SPI_MODE_1 = 0x01, /**< mode 1 */
	SPI_MODE_2 = 0x02, /**< mode 2 */
	SPI_MODE_3 = 0x03, /**< mode 3 */
} spi_mode_t;


/**
 * @brief spi var definition
 */
//SPI_HandleTypeDef hspi1; /**< spi handle */

/**
 * @brief  spi cs init
 * @return status code
 *         - 0 success
 * @note   none
 */
static uint8_t a_spi_cs_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* enable cs gpio clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* gpio init */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	return 0;
}

/**
 * @brief     spi bus init
 * @param[in] mode is the spi mode
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      SCLK is PA5, MOSI is PA7 MISO is PA6 and CS is PA4
 */
uint8_t spi_init(spi_mode_t mode) {
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

	/* set the mode */
	if (mode == SPI_MODE_0) {
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	} else if (mode == SPI_MODE_1) {
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	} else if (mode == SPI_MODE_2) {
		hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	} else {
		hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
		hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	}
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.CRCPolynomial = 10;

	/* spi init */
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		return 1;
	}

	return a_spi_cs_init();
}

/**
 * @brief  spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t spi_deinit(void) {
	/* cs deinit */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

	/* spi deinit */
	if (HAL_SPI_DeInit(&hspi1) != HAL_OK) {
		return 1;
	}

	return 0;
}

/**
 * @brief     spi bus write command
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t spi_write_cmd(uint8_t *buf, uint16_t len) {
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* if len > 0 */
	if (len > 0) {
		/* transmit the buffer */
		res = HAL_SPI_Transmit(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief     spi bus write
 * @param[in] addr is the spi register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t spi_write(uint8_t addr, uint8_t *buf, uint16_t len) {
	uint8_t buffer;
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* transmit the addr */
	buffer = addr;
	res = HAL_SPI_Transmit(&hspi1, (uint8_t*) &buffer, 1, 1000);
	if (res != HAL_OK) {
		/* set cs high */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	/* if len > 0 */
	if (len > 0) {
		/* transmit the buffer */
		res = HAL_SPI_Transmit(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief     spi bus write address 16
 * @param[in] addr is the spi register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t spi_write_address16(uint16_t addr, uint8_t *buf, uint16_t len) {
	uint8_t buffer[2];
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* transmit the addr  */
	buffer[0] = (addr >> 8) & 0xFF;
	buffer[1] = addr & 0xFF;
	res = HAL_SPI_Transmit(&hspi1, (uint8_t*) buffer, 2, 1000);
	if (res != HAL_OK) {
		/* set cs high */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	/* if len > 0 */
	if (len > 0) {
		/* transmit the buffer */
		res = HAL_SPI_Transmit(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief      spi bus read command
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t spi_read_cmd(uint8_t *buf, uint16_t len) {
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* if len > 0 */
	if (len > 0) {
		/* receive to the buffer */
		res = HAL_SPI_Receive(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief      spi bus read
 * @param[in]  addr is the spi register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t spi_read(uint8_t addr, uint8_t *buf, uint16_t len) {
	uint8_t buffer;
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* transmit the addr */
	buffer = addr;
	res = HAL_SPI_Transmit(&hspi1, (uint8_t*) &buffer, 1, 1000);
	if (res != HAL_OK) {
		/* set cs high */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	/* if len > 0 */
	if (len > 0) {
		/* receive to the buffer */
		res = HAL_SPI_Receive(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief      spi bus read address 16
 * @param[in]  addr is the spi register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t spi_read_address16(uint16_t addr, uint8_t *buf, uint16_t len) {
	uint8_t buffer[2];
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* transmit the addr  */
	buffer[0] = (addr >> 8) & 0xFF;
	buffer[1] = addr & 0xFF;
	res = HAL_SPI_Transmit(&hspi1, (uint8_t*) buffer, 2, 1000);
	if (res != HAL_OK) {
		/* set cs high */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	/* if len > 0 */
	if (len > 0) {
		/* receive to the buffer */
		res = HAL_SPI_Receive(&hspi1, buf, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief      spi transmit
 * @param[in]  *tx points to a tx buffer
 * @param[out] *rx points to a rx buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 transmit failed
 * @note       none
 */
uint8_t spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len) {
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* if len > 0 */
	if (len > 0) {
		/* transmit */
		res = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

/**
 * @brief      spi bus write read
 * @param[in]  *in_buf points to an input buffer
 * @param[in]  in_len is the input length
 * @param[out] *out_buf points to an output buffer
 * @param[in]  out_len is the output length
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
 */
uint8_t spi_write_read(uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf, uint32_t out_len) {
	uint8_t res;

	/* set cs low */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/* if in_len > 0 */
	if (in_len > 0) {
		/* transmit the input buffer */
		res = HAL_SPI_Transmit(&hspi1, in_buf, in_len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* if out_len > 0 */
	if (out_len > 0) {
		/* transmit to the output buffer */
		res = HAL_SPI_Receive(&hspi1, out_buf, out_len, 1000);
		if (res != HAL_OK) {
			/* set cs high */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			return 1;
		}
	}

	/* set cs high */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}
#endif	//C_SPI_H
