/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#define FINGERPRINT_OK 0x00				  //!< Command execution is complete
#define FINGERPRINT_PACKETRECIEVEERR 0x01 //!< Error when receiving data package
#define FINGERPRINT_NOFINGER 0x02		  //!< No finger on the sensor
#define FINGERPRINT_IMAGEFAIL 0x03		  //!< Failed to enroll the finger
#define FINGERPRINT_IMAGEMESS \
	0x06 //!< Failed to generate character file due to overly disorderly
		 //!< fingerprint image
#define FINGERPRINT_FEATUREFAIL \
	0x07						  //!< Failed to generate character file due to the lack of character point
								  //!< or small fingerprint image
#define FINGERPRINT_NOMATCH 0x08  //!< Finger doesn't match
#define FINGERPRINT_NOTFOUND 0x09 //!< Failed to find matching finger
#define FINGERPRINT_ENROLLMISMATCH \
	0x0A //!< Failed to combine the character files
#define FINGERPRINT_BADLOCATION \
	0x0B //!< Addressed PageID is beyond the finger library
#define FINGERPRINT_DBRANGEFAIL \
	0x0C								   //!< Error when reading template from library or invalid template
#define FINGERPRINT_UPLOADFEATUREFAIL 0x0D //!< Error when uploading template
#define FINGERPRINT_PACKETRESPONSEFAIL \
	0x0E							 //!< Module failed to receive the following data packages
#define FINGERPRINT_UPLOADFAIL 0x0F	 //!< Error when uploading image
#define FINGERPRINT_DELETEFAIL 0x10	 //!< Failed to delete the template
#define FINGERPRINT_DBCLEARFAIL 0x11 //!< Failed to clear finger library
#define FINGERPRINT_PASSFAIL \
	0x13 //!< Find whether the fingerprint passed or failed
#define FINGERPRINT_INVALIDIMAGE \
	0x15							//!< Failed to generate image because of lac of valid primary image
#define FINGERPRINT_FLASHERR 0x18	//!< Error when writing flash
#define FINGERPRINT_INVALIDREG 0x1A //!< Invalid register number
#define FINGERPRINT_ADDRCODE 0x20	//!< Address code
#define FINGERPRINT_PASSVERIFY 0x21 //!< Verify the fingerprint passed
#define FINGERPRINT_STARTCODE \
	0xEF01 //!< Fixed falue of EF01H; High byte transferred first

#define FINGERPRINT_COMMANDPACKET 0x1 //!< Command packet
#define FINGERPRINT_DATAPACKET \
	0x2								  //!< Data packet, must follow command packet or acknowledge packet
#define FINGERPRINT_ACKPACKET 0x7	  //!< Acknowledge packet
#define FINGERPRINT_ENDDATAPACKET 0x8 //!< End of data packet

#define FINGERPRINT_TIMEOUT 0xFF   //!< Timeout was reached
#define FINGERPRINT_BADPACKET 0xFE //!< Bad packet was sent

#define FINGERPRINT_GETIMAGE 0x01 //!< Collect finger image
#define FINGERPRINT_IMAGE2TZ 0x02 //!< Generate character file from image
#define FINGERPRINT_SEARCH 0x04	  //!< Search for fingerprint in slot
#define FINGERPRINT_REGMODEL \
	0x05 //!< Combine character files and generate template

#define FINGERPRINT_STORE 0x06			//!< Store template
#define FINGERPRINT_LOAD 0x07			//!< Read/load template
#define FINGERPRINT_UPLOAD 0x08			//!< Upload template
#define FINGERPRINT_DELETE 0x0C			//!< Delete templates
#define FINGERPRINT_EMPTY 0x0D			//!< Empty library
#define FINGERPRINT_READSYSPARAM 0x0F	//!< Read system parameters
#define FINGERPRINT_SETPASSWORD 0x12	//!< Sets passwords
#define FINGERPRINT_VERIFYPASSWORD 0x13 //!< Verifies the password
#define FINGERPRINT_HISPEEDSEARCH \
	0x1B							   //!< Asks the sensor to search for a matching fingerprint template to the
									   //!< last model generated
#define FINGERPRINT_TEMPLATECOUNT 0x1D //!< Read finger template numbers
#define FINGERPRINT_AURALEDCONFIG 0x35 //!< Aura LED control
#define FINGERPRINT_LEDON 0x50		   //!< Turn on the onboard LED
#define FINGERPRINT_LEDOFF 0x51		   //!< Turn off the onboard LED

typedef struct
{
	uint8_t TxBuffer[32];
	uint8_t RxBuffer[32];
	uint8_t AnswerBuffer[32];
	uint8_t GotAnswer;
	int16_t Template;

} Fingerprint;

Fingerprint FingerPrint;

// #########################################################################################################################
void VerifyPassword(uint32_t pass)
{
	memset(FingerPrint.TxBuffer, 0, sizeof(FingerPrint.TxBuffer));
	FingerPrint.TxBuffer[0] = FINGERPRINT_STARTCODE >> 8; // header
	FingerPrint.TxBuffer[1] = FINGERPRINT_STARTCODE & 0XFF;
	FingerPrint.TxBuffer[2] = 0xFF; // chip address
	FingerPrint.TxBuffer[3] = 0xFF;
	FingerPrint.TxBuffer[4] = 0xFF;
	FingerPrint.TxBuffer[5] = 0xFF;
	FingerPrint.TxBuffer[6] = FINGERPRINT_COMMANDPACKET; // command packet

	uint8_t data[32] = {FINGERPRINT_VERIFYPASSWORD, pass >> 24, pass >> 16, pass >> 8, pass & 0xFF};
	uint16_t len_data = sizeof(data) + 2; // add the byte for checksum

	uint16_t checksum = ((len_data >> 8) + (len_data & 0xFF)) + FINGERPRINT_COMMANDPACKET;

	FingerPrint.TxBuffer[7] = len_data >> 8; // packet legnth
	FingerPrint.TxBuffer[8] = len_data & 0xFF;

	FingerPrint.TxBuffer[9] = FINGERPRINT_VERIFYPASSWORD; // identifier
	FingerPrint.TxBuffer[10] = pass >> 24;
	FingerPrint.TxBuffer[11] = pass >> 16;
	FingerPrint.TxBuffer[12] = pass >> 8;
	FingerPrint.TxBuffer[13] = pass & 0xFF;

	for (uint8_t i = 0; i < sizeof(data); i++)
	{
		checksum += data[i];
	}

	FingerPrint.TxBuffer[14] = checksum >> 8;	// checksum
	FingerPrint.TxBuffer[15] = checksum & 0xFF; // checksum

	HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, 32, 100);
	HAL_Delay(1000);
}

// #########################################################################################################################

int16_t ReadTemplateNumber(void)
{
	memset(FingerPrint.TxBuffer, 0, sizeof(FingerPrint.TxBuffer));
	FingerPrint.TxBuffer[0] = FINGERPRINT_STARTCODE >> 8;
	FingerPrint.TxBuffer[1] = FINGERPRINT_STARTCODE & 0xFF;
	FingerPrint.TxBuffer[2] = 0xFF;
	FingerPrint.TxBuffer[3] = 0xFF;
	FingerPrint.TxBuffer[4] = 0xFF;
	FingerPrint.TxBuffer[5] = 0xFF;
	FingerPrint.TxBuffer[6] = FINGERPRINT_COMMANDPACKET;

	uint8_t data[32] = {FINGERPRINT_TEMPLATECOUNT};
	uint16_t len_data = sizeof(data) + 2; // add the byte for checksum

	uint16_t checksum = ((len_data >> 8) + (len_data & 0xFF)) + FINGERPRINT_COMMANDPACKET;

	FingerPrint.TxBuffer[7] = len_data >> 8; // packet legnth
	FingerPrint.TxBuffer[8] = len_data & 0xFF;

	FingerPrint.TxBuffer[9] = FINGERPRINT_TEMPLATECOUNT;

	for (uint8_t i = 0; i < sizeof(data); i++)
	{
		checksum += data[i];
	}

	FingerPrint.TxBuffer[10] = checksum >> 8;	// checksum
	FingerPrint.TxBuffer[11] = checksum & 0xFF; // checksum
	do
	{
		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, 12, 100);
	} while (0);
	//--- Read Template

	return -1;
}

// #########################################################################################################################
bool SaveNewFinger(uint16_t Location, uint8_t WaitForFingerInSecond)
{
	uint8_t TimeOut;

	//+++ take Image
	memset(FingerPrint.TxBuffer, 0, sizeof(FingerPrint.TxBuffer));
	FingerPrint.TxBuffer[0] = FINGERPRINT_STARTCODE >> 8; // header
	FingerPrint.TxBuffer[1] = FINGERPRINT_STARTCODE & 0XFF;
	FingerPrint.TxBuffer[2] = 0xFF;
	FingerPrint.TxBuffer[3] = 0xFF;
	FingerPrint.TxBuffer[4] = 0xFF;
	FingerPrint.TxBuffer[5] = 0xFF;
	FingerPrint.TxBuffer[6] = FINGERPRINT_COMMANDPACKET;

	uint8_t data[32] = {FINGERPRINT_GETIMAGE};
	uint16_t len_data = sizeof(data) + 2; // add the byte for checksum

	uint16_t checksum = ((len_data >> 8) + (len_data & 0xFF)) + FINGERPRINT_COMMANDPACKET;

	FingerPrint.TxBuffer[7] = len_data >> 8; // packet legnth
	FingerPrint.TxBuffer[8] = len_data & 0xFF;

	FingerPrint.TxBuffer[9] = FINGERPRINT_GETIMAGE;

	for (uint8_t i = 0; i < sizeof(data); i++)
	{
		checksum += data[i];
	}

	FingerPrint.TxBuffer[10] = checksum >> 8;	// checksum
	FingerPrint.TxBuffer[11] = checksum & 0xFF; // checksum

	TimeOut = 0;
	while (TimeOut < WaitForFingerInSecond)
	{
		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		TimeOut++;
		if (TimeOut == WaitForFingerInSecond)
			goto Faild;
	}
	//--- take Image
	//+++	put image to buffer 1
	do
	{
		FingerPrint.TxBuffer[8] = 0x04;
		FingerPrint.TxBuffer[9] = FINGERPRINT_IMAGE2TZ;
		FingerPrint.TxBuffer[10] = 1;
		FingerPrint.TxBuffer[11] = 0x00;
		FingerPrint.TxBuffer[12] = 0x08;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		else
			goto Faild;
	} while (0);
	//---	put image to buffer 1

	//+++ Wait for put your finger up
	TimeOut = 0;
	while (TimeOut < WaitForFingerInSecond)
	{
		HAL_Delay(1000);
		TimeOut++;
		if (TimeOut == WaitForFingerInSecond)
			goto Faild;
	}
	//--- Wait for put your finger up

	//+++ take Image
	FingerPrint.TxBuffer[8] = 0x03;
	FingerPrint.TxBuffer[9] = FINGERPRINT_GETIMAGE;
	FingerPrint.TxBuffer[10] = 0x00;
	FingerPrint.TxBuffer[11] = 0x05;
	TimeOut = 0;
	while (TimeOut < WaitForFingerInSecond)
	{
		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		TimeOut++;
		if (TimeOut == WaitForFingerInSecond)
			goto Faild;
	}
	//--- take Image

	//+++	put image to buffer 2
	do
	{
		FingerPrint.TxBuffer[8] = 0x04;
		FingerPrint.TxBuffer[9] = FINGERPRINT_IMAGE2TZ;
		FingerPrint.TxBuffer[10] = 2;
		FingerPrint.TxBuffer[11] = 0x00;
		FingerPrint.TxBuffer[12] = 0x09;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		else
			goto Faild;
	} while (0);
	//---	put image to buffer 2

	//+++ Wait for put your finger up
	TimeOut = 0;
	while (TimeOut < WaitForFingerInSecond)
	{
		HAL_Delay(1000);
		TimeOut++;
		if (TimeOut == WaitForFingerInSecond)
			goto Faild;
	}
	//--- Wait for put your finger up

	//+++ Create Register model
	do
	{
		FingerPrint.TxBuffer[8] = 0x03;
		FingerPrint.TxBuffer[9] = FINGERPRINT_REGMODEL;
		FingerPrint.TxBuffer[10] = 0x00;
		FingerPrint.TxBuffer[11] = 0x09;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		else
			goto Faild;

	} while (0);
	//---	Create Register model

	//+++ Store in memory
	do
	{
		FingerPrint.TxBuffer[8] = 0x06;
		FingerPrint.TxBuffer[9] = FINGERPRINT_STORE;
		FingerPrint.TxBuffer[10] = 0x01;
		FingerPrint.TxBuffer[11] = Location >> 8;
		FingerPrint.TxBuffer[12] = Location & 0x00ff;

		uint8_t Checksum = 0;
		for (uint8_t i = 0; i < 9; i++)
			Checksum += FingerPrint.TxBuffer[i + 6];

		FingerPrint.TxBuffer[13] = Checksum >> 8;
		FingerPrint.TxBuffer[14] = Checksum & 0x00FF;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
			break;
		else
			goto Faild;

	} while (0);
	//--- Store in memory

	ReadTemplateNumber();
	return true;

Faild:
	HAL_UART_Transmit(&huart2, (uint8_t *)"Timeout", 10, 100);
	return false;
}

// #########################################################################################################################

int16_t Scan_Fingerprint(void)
{
	uint8_t Timeout;

	FingerPrint.GotAnswer = 0;

	//+++ take Image
	memset(FingerPrint.TxBuffer, 0, sizeof(FingerPrint.TxBuffer));
	FingerPrint.TxBuffer[0] = FINGERPRINT_STARTCODE >> 8; // header
	FingerPrint.TxBuffer[1] = FINGERPRINT_STARTCODE & 0XFF;
	FingerPrint.TxBuffer[2] = 0xFF;
	FingerPrint.TxBuffer[3] = 0xFF;
	FingerPrint.TxBuffer[4] = 0xFF;
	FingerPrint.TxBuffer[5] = 0xFF;
	FingerPrint.TxBuffer[6] = FINGERPRINT_COMMANDPACKET;

	uint8_t data[32] = {FINGERPRINT_GETIMAGE};
	uint16_t len_data = sizeof(data) + 2; // add the byte for checksum

	uint16_t checksum = ((len_data >> 8) + (len_data & 0xFF)) + FINGERPRINT_COMMANDPACKET;

	FingerPrint.TxBuffer[7] = len_data >> 8; // packet legnth
	FingerPrint.TxBuffer[8] = len_data & 0xFF;

	FingerPrint.TxBuffer[9] = FINGERPRINT_GETIMAGE;

	for (uint8_t i = 0; i < sizeof(data); i++)
	{
		checksum += data[i];
	}

	FingerPrint.TxBuffer[10] = checksum >> 8;	// checksum
	FingerPrint.TxBuffer[11] = checksum & 0xFF; // checksum

	do
	{
		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		for (Timeout = 0; Timeout < 20; Timeout++)
		{
			HAL_Delay(100);
			if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
				break;
		}
		if (Timeout > 19)
			goto Faild;
	} while (0);
	//--- take Image

	//+++	put image to buffer 1
	do
	{
		FingerPrint.TxBuffer[8] = 0x04;
		FingerPrint.TxBuffer[9] = FINGERPRINT_IMAGE2TZ;
		FingerPrint.TxBuffer[10] = 1;
		FingerPrint.TxBuffer[11] = 0x00;
		FingerPrint.TxBuffer[12] = 0x08;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		for (Timeout = 0; Timeout < 20; Timeout++)
		{
			HAL_Delay(100);
			if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x03) && (FingerPrint.AnswerBuffer[2] == 0x00) && (FingerPrint.AnswerBuffer[3] == 0x00) && (FingerPrint.AnswerBuffer[4] == 0x0A))
				break;
			if (Timeout > 19)
				goto Faild;
		}
	} while (0);
	//---	put image to buffer 1

	//+++	Searching
	do
	{
		FingerPrint.TxBuffer[8] = 0x08;
		FingerPrint.TxBuffer[9] = FINGERPRINT_SEARCH;
		FingerPrint.TxBuffer[10] = 1;
		FingerPrint.TxBuffer[11] = 0x00;
		FingerPrint.TxBuffer[12] = 0x00;
		FingerPrint.TxBuffer[13] = 0x01;
		FingerPrint.TxBuffer[14] = 0xF4;

		uint16_t Checksum = 0;

		for (uint8_t i = 0; i < 11; i++)

			Checksum += FingerPrint.TxBuffer[i + 6];
		FingerPrint.TxBuffer[15] = Checksum >> 8;
		FingerPrint.TxBuffer[16] = Checksum & 0x00FF;

		memset(FingerPrint.AnswerBuffer, 0, sizeof(FingerPrint.AnswerBuffer));

		HAL_UART_Transmit(&huart2, FingerPrint.TxBuffer, sizeof(FingerPrint.TxBuffer), 100);
		HAL_Delay(1000);
		HAL_UART_Receive(&huart2, FingerPrint.AnswerBuffer, sizeof(FingerPrint.AnswerBuffer), 100);

		for (Timeout = 0; Timeout < 20; Timeout++)
		{
			HAL_Delay(100);
			if ((FingerPrint.AnswerBuffer[0] == 0x00) && (FingerPrint.AnswerBuffer[1] == 0x07) && (FingerPrint.AnswerBuffer[2] == 0x00))
			{
				return FingerPrint.AnswerBuffer[3] * 256 + FingerPrint.AnswerBuffer[4];
			}
			if (Timeout > 19)
				goto Faild;
		}
	} while (0);
	//---	Searching

Faild:
	HAL_UART_Transmit(&huart2, (uint8_t *)"Timeout", 10, 100);

	return -1;
}
// #########################################################################################################################

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	/* NOTE : This function should not be modified, when the callback is needed,
			  the HAL_UART_RxCpltCallback can be implemented in the user file.
	 */
	HAL_UART_Transmit_DMA(&huart2, FingerPrint.RxBuffer, sizeof(FingerPrint.RxBuffer));
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	VerifyPassword(0x0);
	HAL_Delay(1000);
	HAL_UART_Receive_DMA(&huart2, FingerPrint.RxBuffer, sizeof(FingerPrint.RxBuffer));
	HAL_Delay(1000);
	ReadTemplateNumber();
	HAL_UART_Receive_DMA(&huart2, FingerPrint.RxBuffer, sizeof(FingerPrint.RxBuffer));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 57600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
