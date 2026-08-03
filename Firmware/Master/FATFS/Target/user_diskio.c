/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include <string.h>
#include "diskio.h"
#include "ff_gen_drv.h"
#include "dbg_trace.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SD_BLOCK_SIZE 512U
#define SD_INIT_TIMEOUT_MS 1000U
#define SD_CMD_TIMEOUT_MS 200U
#define SD_DATA_TIMEOUT_MS 500U

#define SD_CMD0 (0U)
#define SD_CMD8 (8U)
#define SD_CMD9 (9U)
#define SD_CMD12 (12U)
#define SD_CMD16 (16U)
#define SD_CMD17 (17U)
#define SD_CMD18 (18U)
#define SD_CMD24 (24U)
#define SD_CMD25 (25U)
#define SD_CMD55 (55U)
#define SD_CMD58 (58U)
#define SD_ACMD41 (41U)

#define SD_TOKEN_START_BLOCK 0xFEU
#define SD_TOKEN_MULTI_WRITE 0xFCU
#define SD_TOKEN_STOP_TRAN 0xFDU
#define SD_LOG(...) PRINT_MESG_DBG(__VA_ARGS__)

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t s_card_is_sdhc = 0U;
static uint32_t s_sector_count = 0U;

static void SD_Select(void);
static void SD_Deselect(void);
static void SD_ReadBytes(uint8_t *data, uint16_t len);
static uint8_t SD_WaitToken(uint8_t token, uint32_t timeout_ms);
static uint8_t SD_WaitReady(uint32_t timeout_ms);
static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
static uint8_t SD_WriteDataBlock(const uint8_t *data, uint8_t token);
static uint8_t SD_ReadCsdAndCapacity(void);

static void SD_Select(void)
{
  SPI1_SD_ChipSelect(1U);
}

static void SD_Deselect(void)
{
  SPI1_SD_ChipSelect(0U);
  SPI1_SD_TransferByte(0xFFU);
}

static void SD_ReadBytes(uint8_t *data, uint16_t len)
{
  if (data == NULL) {
    while (len-- > 0U) {
      (void)SPI1_SD_TransferByte(0xFFU);
    }
    return;
  }
  for (uint16_t i = 0U; i < len; ++i) {
    data[i] = SPI1_SD_TransferByte(0xFFU);
  }
}

static uint8_t SD_WaitToken(uint8_t token, uint32_t timeout_ms)
{
  uint8_t rx = 0xFFU;
  uint32_t start = HAL_GetTick();
  do {
    rx = SPI1_SD_TransferByte(0xFFU);
    if (rx == token) {
      return rx;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);
  return rx;
}

static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  do {
    if (SPI1_SD_TransferByte(0xFFU) == 0xFFU) {
      return 1U;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);
  return 0U;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t frame[6];
  uint8_t r1 = 0xFFU;

  if (cmd != SD_CMD12) {
    SD_Deselect();
    SD_Select();
    if (!SD_WaitReady(SD_CMD_TIMEOUT_MS)) {
      SD_Deselect();
      return 0xFFU;
    }
  } else {
    SD_Select();
    (void)SPI1_SD_TransferByte(0xFFU);
  }

  frame[0] = 0x40U | cmd;
  frame[1] = (uint8_t)(arg >> 24);
  frame[2] = (uint8_t)(arg >> 16);
  frame[3] = (uint8_t)(arg >> 8);
  frame[4] = (uint8_t)arg;
  frame[5] = crc;
  for (uint8_t i = 0U; i < sizeof(frame); ++i) {
    (void)SPI1_SD_TransferByte(frame[i]);
  }

  for (uint8_t n = 0U; n < 10U; ++n) {
    r1 = SPI1_SD_TransferByte(0xFFU);
    if ((r1 & 0x80U) == 0U) {
      return r1;
    }
  }
  return r1;
}

static uint8_t SD_WriteDataBlock(const uint8_t *data, uint8_t token)
{
  uint8_t resp = 0xFFU;

  if (!SD_WaitReady(SD_DATA_TIMEOUT_MS)) {
    return 0U;
  }

  SPI1_SD_TransferByte(token);
  for (uint16_t i = 0U; i < SD_BLOCK_SIZE; ++i) {
    SPI1_SD_TransferByte(data[i]);
  }
  SPI1_SD_TransferByte(0xFFU);
  SPI1_SD_TransferByte(0xFFU);

  resp = SPI1_SD_TransferByte(0xFFU);
  if ((resp & 0x1FU) != 0x05U) {
    return 0U;
  }
  return SD_WaitReady(SD_DATA_TIMEOUT_MS);
}

static uint8_t SD_ReadCsdAndCapacity(void)
{
  uint8_t csd[16] = {0};
  uint8_t csd_structure = 0U;

  if (SD_SendCommand(SD_CMD9, 0U, 0x01U) != 0x00U) {
    SD_Deselect();
    return 0U;
  }
  if (SD_WaitToken(SD_TOKEN_START_BLOCK, SD_DATA_TIMEOUT_MS) != SD_TOKEN_START_BLOCK) {
    SD_Deselect();
    return 0U;
  }
  SD_ReadBytes(csd, sizeof(csd));
  SD_ReadBytes(NULL, 2U);
  SD_Deselect();

  csd_structure = (uint8_t)((csd[0] >> 6) & 0x03U);
  if (csd_structure == 1U) {
    uint32_t c_size = (((uint32_t)(csd[7] & 0x3FU)) << 16) | (((uint32_t)csd[8]) << 8) | (uint32_t)csd[9];
    s_sector_count = (c_size + 1U) * 1024U;
    SD_LOG("[SD] CSD v2 detected, sectors=%lu\r\n", (unsigned long)s_sector_count);
    return 1U;
  } else {
    uint32_t c_size = (((uint32_t)(csd[6] & 0x03U)) << 10) | (((uint32_t)csd[7]) << 2) | ((uint32_t)(csd[8] >> 6));
    uint32_t c_size_mult = (((uint32_t)(csd[9] & 0x03U)) << 1) | ((uint32_t)(csd[10] >> 7));
    uint32_t read_bl_len = (uint32_t)(csd[5] & 0x0FU);
    uint32_t blocknr = (c_size + 1U) * (1UL << (c_size_mult + 2U));
    uint32_t blocklen = (1UL << read_bl_len);
    uint32_t capacity = blocknr * blocklen;
    s_sector_count = capacity / SD_BLOCK_SIZE;
    SD_LOG("[SD] CSD v1 detected, sectors=%lu\r\n", (unsigned long)s_sector_count);
    return (s_sector_count != 0U) ? 1U : 0U;
  }
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  uint8_t r1 = 0xFFU;
  uint8_t ocr[4] = {0};
  uint32_t start_ms = HAL_GetTick();

  if (pdrv != 0U) {
    SD_LOG("[SD] init rejected: invalid pdrv=%u\r\n", (unsigned)pdrv);
    return STA_NOINIT;
  }

  SD_LOG("[SD] init start: SPI1 low-speed, CS=PA4\r\n");
  Stat = STA_NOINIT;
  SPI1_SD_SetSpeedLow();
  SPI1_SD_ChipSelect(0U);
  SPI1_SD_ClockIdleBytes(12U);

  /* CMD0: reset to idle */
  do {
    r1 = SD_SendCommand(SD_CMD0, 0U, 0x95U);
    if ((HAL_GetTick() - start_ms) > SD_INIT_TIMEOUT_MS) {
      SD_LOG("[SD] init fail: CMD0 timeout, r1=0x%02X\r\n", (unsigned)r1);
      return Stat;
    }
  } while (r1 != 0x01U);
  SD_LOG("[SD] CMD0 ok, card in idle\r\n");

  /* CMD8: check v2 cards */
  r1 = SD_SendCommand(SD_CMD8, 0x000001AAU, 0x87U);
  SD_LOG("[SD] CMD8 r1=0x%02X\r\n", (unsigned)r1);
  if (r1 == 0x01U) {
    SD_ReadBytes(ocr, sizeof(ocr));
    if (ocr[2] != 0x01U || ocr[3] != 0xAAU) {
      SD_LOG("[SD] init fail: CMD8 echo mismatch ocr=%02X %02X %02X %02X\r\n",
             (unsigned)ocr[0], (unsigned)ocr[1], (unsigned)ocr[2], (unsigned)ocr[3]);
      return Stat;
    }
    SD_LOG("[SD] card type path: SD v2+/HC capable\r\n");

    start_ms = HAL_GetTick();
    do {
      (void)SD_SendCommand(SD_CMD55, 0U, 0x01U);
      r1 = SD_SendCommand(SD_ACMD41, 0x40000000U, 0x01U);
      if ((HAL_GetTick() - start_ms) > SD_INIT_TIMEOUT_MS) {
        SD_LOG("[SD] init fail: ACMD41(HCS) timeout, r1=0x%02X\r\n", (unsigned)r1);
        return Stat;
      }
    } while (r1 != 0x00U);
    SD_LOG("[SD] ACMD41(HCS) ready\r\n");

    r1 = SD_SendCommand(SD_CMD58, 0U, 0x01U);
    if (r1 != 0x00U) {
      SD_LOG("[SD] init fail: CMD58 r1=0x%02X\r\n", (unsigned)r1);
      return Stat;
    }
    SD_ReadBytes(ocr, sizeof(ocr));
    s_card_is_sdhc = ((ocr[0] & 0x40U) != 0U) ? 1U : 0U;
    SD_LOG("[SD] OCR=%02X %02X %02X %02X, sdhc=%u\r\n",
           (unsigned)ocr[0], (unsigned)ocr[1], (unsigned)ocr[2], (unsigned)ocr[3],
           (unsigned)s_card_is_sdhc);
  } else {
    /* legacy SDSC path */
    SD_LOG("[SD] card type path: legacy SDSC\r\n");
    start_ms = HAL_GetTick();
    do {
      (void)SD_SendCommand(SD_CMD55, 0U, 0x01U);
      r1 = SD_SendCommand(SD_ACMD41, 0U, 0x01U);
      if ((HAL_GetTick() - start_ms) > SD_INIT_TIMEOUT_MS) {
        SD_LOG("[SD] init fail: ACMD41(legacy) timeout, r1=0x%02X\r\n", (unsigned)r1);
        return Stat;
      }
    } while (r1 != 0x00U);
    s_card_is_sdhc = 0U;
    r1 = SD_SendCommand(SD_CMD16, SD_BLOCK_SIZE, 0x01U);
    if (r1 != 0x00U) {
      SD_LOG("[SD] init fail: CMD16 r1=0x%02X\r\n", (unsigned)r1);
      return Stat;
    }
    SD_LOG("[SD] CMD16 blocklen=512 set\r\n");
  }

  if (!SD_ReadCsdAndCapacity()) {
    SD_LOG("[SD] init fail: CSD parse/capacity\r\n");
    return Stat;
  }

  SPI1_SD_ChipSelect(0U);
  SPI1_SD_TransferByte(0xFFU);
  SPI1_SD_SetSpeedHigh();
  Stat = 0U;
  SD_LOG("[SD] init done: status=0, sdhc=%u, sectors=%lu, bytes=%lu\r\n",
         (unsigned)s_card_is_sdhc,
         (unsigned long)s_sector_count,
         (unsigned long)(s_sector_count * SD_BLOCK_SIZE));
  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  if (pdrv != 0U) {
    return STA_NOINIT;
  }
  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  DWORD block = sector;
  BYTE token = 0xFFU;

  if (pdrv != 0U || count == 0U) {
    return RES_PARERR;
  }
  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }
  if (s_card_is_sdhc == 0U) {
    block *= SD_BLOCK_SIZE;
  }

  if (count == 1U) {
    if (SD_SendCommand(SD_CMD17, block, 0x01U) != 0x00U) {
      SD_Deselect();
      return RES_ERROR;
    }
    token = SD_WaitToken(SD_TOKEN_START_BLOCK, SD_DATA_TIMEOUT_MS);
    if (token != SD_TOKEN_START_BLOCK) {
      SD_Deselect();
      return RES_ERROR;
    }
    SD_ReadBytes(buff, SD_BLOCK_SIZE);
    SD_ReadBytes(NULL, 2U);
    SD_Deselect();
    return RES_OK;
  }

  if (SD_SendCommand(SD_CMD18, block, 0x01U) != 0x00U) {
    SD_Deselect();
    return RES_ERROR;
  }

  do {
    token = SD_WaitToken(SD_TOKEN_START_BLOCK, SD_DATA_TIMEOUT_MS);
    if (token != SD_TOKEN_START_BLOCK) {
      SD_Deselect();
      return RES_ERROR;
    }
    SD_ReadBytes(buff, SD_BLOCK_SIZE);
    SD_ReadBytes(NULL, 2U);
    buff += SD_BLOCK_SIZE;
  } while (--count);

  (void)SD_SendCommand(SD_CMD12, 0U, 0x01U);
  SD_Deselect();
  return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  DWORD block = sector;
  uint8_t resp = 0xFFU;

  if (pdrv != 0U || count == 0U) {
    return RES_PARERR;
  }
  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }
  if (Stat & STA_PROTECT) {
    return RES_WRPRT;
  }
  if (s_card_is_sdhc == 0U) {
    block *= SD_BLOCK_SIZE;
  }

  if (count == 1U) {
    if (SD_SendCommand(SD_CMD24, block, 0x01U) != 0x00U) {
      SD_Deselect();
      return RES_ERROR;
    }
    if (!SD_WriteDataBlock(buff, SD_TOKEN_START_BLOCK)) {
      SD_Deselect();
      return RES_ERROR;
    }
    SD_Deselect();
    return RES_OK;
  }

  if (SD_SendCommand(SD_CMD25, block, 0x01U) != 0x00U) {
    SD_Deselect();
    return RES_ERROR;
  }

  do {
    if (!SD_WriteDataBlock(buff, SD_TOKEN_MULTI_WRITE)) {
      SD_Deselect();
      return RES_ERROR;
    }
    buff += SD_BLOCK_SIZE;
  } while (--count);

  SPI1_SD_TransferByte(SD_TOKEN_STOP_TRAN);
  resp = SPI1_SD_TransferByte(0xFFU);
  (void)resp;
  if (!SD_WaitReady(SD_DATA_TIMEOUT_MS)) {
    SD_Deselect();
    return RES_ERROR;
  }
  SD_Deselect();
  return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  if (pdrv != 0U) {
    return RES_PARERR;
  }
  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  switch (cmd) {
    case CTRL_SYNC:
      return SD_WaitReady(SD_CMD_TIMEOUT_MS) ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT:
      if (buff == NULL) {
        return RES_PARERR;
      }
      *(DWORD *)buff = s_sector_count;
      return (s_sector_count != 0U) ? RES_OK : RES_ERROR;
    case GET_SECTOR_SIZE:
      if (buff == NULL) {
        return RES_PARERR;
      }
      *(WORD *)buff = SD_BLOCK_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      if (buff == NULL) {
        return RES_PARERR;
      }
      *(DWORD *)buff = 1U;
      return RES_OK;
    default:
      return RES_PARERR;
  }
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

