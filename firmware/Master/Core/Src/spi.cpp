/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#define SD_CS_GPIO_PORT GPIOA
#define SD_CS_PIN GPIO_PIN_4
#define SD_SPI_TIMEOUT_MS 100U

static uint32_t s_spi1_sd_prescaler = SPI_BAUDRATEPRESCALER_2;

static HAL_StatusTypeDef SPI1_SD_Reinit(uint32_t prescaler)
{
  hspi1.Init.BaudRatePrescaler = prescaler;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  return HAL_SPI_Init(&hspi1);
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  if (SPI1_SD_Reinit(SPI_BAUDRATEPRESCALER_128) != HAL_OK)
  {
    Error_Handler();
  }
  s_spi1_sd_prescaler = SPI_BAUDRATEPRESCALER_128;
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA12     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA12     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_12);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void SPI1_SD_SetSpeedLow(void)
{
  if (s_spi1_sd_prescaler == SPI_BAUDRATEPRESCALER_128) {
    return;
  }
  (void)SPI1_SD_Reinit(SPI_BAUDRATEPRESCALER_128);
  s_spi1_sd_prescaler = SPI_BAUDRATEPRESCALER_128;
}

void SPI1_SD_SetSpeedHigh(void)
{
  if (s_spi1_sd_prescaler == SPI_BAUDRATEPRESCALER_2) {
    return;
  }
  (void)SPI1_SD_Reinit(SPI_BAUDRATEPRESCALER_2);
  s_spi1_sd_prescaler = SPI_BAUDRATEPRESCALER_2;
}

void SPI1_SD_ChipSelect(uint8_t selected)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

HAL_StatusTypeDef SPI1_SD_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
  if (len == 0U) {
    return HAL_OK;
  }
  if (tx != NULL && rx != NULL) {
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, timeout_ms);
  }
  if (tx != NULL) {
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, timeout_ms);
  }
  if (rx != NULL) {
    uint8_t dummy = 0xFFU;
    for (uint16_t i = 0U; i < len; ++i) {
      if (HAL_SPI_TransmitReceive(&hspi1, &dummy, &rx[i], 1U, timeout_ms) != HAL_OK) {
        return HAL_ERROR;
      }
    }
    return HAL_OK;
  }
  return HAL_ERROR;
}

uint8_t SPI1_SD_TransferByte(uint8_t tx)
{
  uint8_t rx = 0xFFU;
  (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1U, SD_SPI_TIMEOUT_MS);
  return rx;
}

void SPI1_SD_ClockIdleBytes(uint16_t count)
{
  uint8_t dummy = 0xFFU;
  while (count-- > 0U) {
    (void)HAL_SPI_Transmit(&hspi1, &dummy, 1U, SD_SPI_TIMEOUT_MS);
  }
}

/* USER CODE END 1 */

