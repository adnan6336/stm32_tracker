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
#include <stdlib.h>
#include <stdio.h>
#include "gnss_parser.h"
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NMEA_MAX_CHARS 85
#define NMEA_MAX_LINES 4
#define TIMCLOCK   64000000
#define PRESCALAR  640
#define NUMVAL 2
#define MAX_COMMAND_LEN 50 //maximum command
#define rTime 180
#define mainCount 120
#define smsBunch 10
#define msgCount 10
#define LOC_PKT_INTVL 5000 // Location packet interval
#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))
//#define COM_PORT huart1
#define GNSS_PORT huart2
#define AT_PORT huart1
#define RESPONSE_MAX_LINE 6
#define RESPONSE_MAX_CHAR 50
#define De 1
#define MAX_DOMAIN_CHAR 51 //max is 50
#define MAX_PORT_CHAR 6 //max is 5

//uint8_t a=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim3_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
//--------------INPUT CAPTURE RING-------------------
uint16_t Difference = 0;
int Is_First_Captured = 0;
float frequency = 0;
uint16_t usWidth = 0;
uint8_t gotIndication = 0;
uint16_t vals[NUMVAL];
//--------------INPUT CAPTURE RING-------------------

static const uint16_t crctab16[] = { 0X0000, 0X1189, 0X2312, 0X329B, 0X4624,
		0X57AD, 0X6536, 0X74BF, 0X8C48, 0X9DC1, 0XAF5A, 0XBED3, 0XCA6C, 0XDBE5,
		0XE97E, 0XF8F7, 0X1081, 0X0108, 0X3393, 0X221A, 0X56A5, 0X472C, 0X75B7,
		0X643E, 0X9CC9, 0X8D40, 0XBFDB, 0XAE52, 0XDAED, 0XCB64, 0XF9FF, 0XE876,
		0X2102, 0X308B, 0X0210, 0X1399, 0X6726, 0X76AF, 0X4434, 0X55BD, 0XAD4A,
		0XBCC3, 0X8E58, 0X9FD1, 0XEB6E, 0XFAE7, 0XC87C, 0XD9F5, 0X3183, 0X200A,
		0X1291, 0X0318, 0X77A7, 0X662E, 0X54B5, 0X453C, 0XBDCB, 0XAC42, 0X9ED9,
		0X8F50, 0XFBEF, 0XEA66, 0XD8FD, 0XC974, 0X4204, 0X538D, 0X6116, 0X709F,
		0X0420, 0X15A9, 0X2732, 0X36BB, 0XCE4C, 0XDFC5, 0XED5E, 0XFCD7, 0X8868,
		0X99E1, 0XAB7A, 0XBAF3, 0X5285, 0X430C, 0X7197, 0X601E, 0X14A1, 0X0528,
		0X37B3, 0X263A, 0XDECD, 0XCF44, 0XFDDF, 0XEC56, 0X98E9, 0X8960, 0XBBFB,
		0XAA72, 0X6306, 0X728F, 0X4014, 0X519D, 0X2522, 0X34AB, 0X0630, 0X17B9,
		0XEF4E, 0XFEC7, 0XCC5C, 0XDDD5, 0XA96A, 0XB8E3, 0X8A78, 0X9BF1, 0X7387,
		0X620E, 0X5095, 0X411C, 0X35A3, 0X242A, 0X16B1, 0X0738, 0XFFCF, 0XEE46,
		0XDCDD, 0XCD54, 0XB9EB, 0XA862, 0X9AF9, 0X8B70, 0X8408, 0X9581, 0XA71A,
		0XB693, 0XC22C, 0XD3A5, 0XE13E, 0XF0B7, 0X0840, 0X19C9, 0X2B52, 0X3ADB,
		0X4E64, 0X5FED, 0X6D76, 0X7CFF, 0X9489, 0X8500, 0XB79B, 0XA612, 0XD2AD,
		0XC324, 0XF1BF, 0XE036, 0X18C1, 0X0948, 0X3BD3, 0X2A5A, 0X5EE5, 0X4F6C,
		0X7DF7, 0X6C7E, 0XA50A, 0XB483, 0X8618, 0X9791, 0XE32E, 0XF2A7, 0XC03C,
		0XD1B5, 0X2942, 0X38CB, 0X0A50, 0X1BD9, 0X6F66, 0X7EEF, 0X4C74, 0X5DFD,
		0XB58B, 0XA402, 0X9699, 0X8710, 0XF3AF, 0XE226, 0XD0BD, 0XC134, 0X39C3,
		0X284A, 0X1AD1, 0X0B58, 0X7FE7, 0X6E6E, 0X5CF5, 0X4D7C, 0XC60C, 0XD785,
		0XE51E, 0XF497, 0X8028, 0X91A1, 0XA33A, 0XB2B3, 0X4A44, 0X5BCD, 0X6956,
		0X78DF, 0X0C60, 0X1DE9, 0X2F72, 0X3EFB, 0XD68D, 0XC704, 0XF59F, 0XE416,
		0X90A9, 0X8120, 0XB3BB, 0XA232, 0X5AC5, 0X4B4C, 0X79D7, 0X685E, 0X1CE1,
		0X0D68, 0X3FF3, 0X2E7A, 0XE70E, 0XF687, 0XC41C, 0XD595, 0XA12A, 0XB0A3,
		0X8238, 0X93B1, 0X6B46, 0X7ACF, 0X4854, 0X59DD, 0X2D62, 0X3CEB, 0X0E70,
		0X1FF9, 0XF78F, 0XE606, 0XD49D, 0XC514, 0XB1AB, 0XA022, 0X92B9, 0X8330,
		0X7BC7, 0X6A4E, 0X58D5, 0X495C, 0X3DE3, 0X2C6A, 0X1EF1, 0X0F78, };

uint8_t locationDataIntervalA = 5; //Location packet interval when ignition is on
uint8_t locationDataIntervalB = 5; //Location packet interval when ignition is off
uint8_t isSMSActive=0;
uint8_t indicationCounter = 0;
uint8_t crcc;
uint8_t rCrc;
uint8_t isFlash = 0;
//char validSender[11] = "3322336979";//osama
char validSender[11] = "3352093997";//adnan
extern uint8_t gnssCRC;
uint8_t cPin[4];
uint8_t recTimeA = 0; // non-v
uint8_t msgCounter = 0; // non-v
volatile uint8_t isPulse = 0;
uint8_t heartBeatTimer = 0; // non-v
uint8_t rebootCounter = 0; //heartbeat check // non-v
uint8_t COM_BUFFER[1];
uint8_t AT_BUFFER[1];
uint8_t GNSS_BUFFER[1];
uint8_t responseBuffer[RESPONSE_MAX_LINE][RESPONSE_MAX_CHAR] = { 0 };
volatile uint8_t lineCount = 0, charCount = 0;
volatile uint8_t nmeaLC = 0; //line Count
volatile uint8_t nmeaCC = 0; //char Count
volatile uint8_t isStart = 0;
volatile uint8_t isBusy = 0;
volatile uint8_t isDataMode = 0;
volatile uint16_t resTimeout = 0;
uint8_t gps_dumm[18] = { 0x78, 0x78, 0x1f, 0x12, 0x17, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
uint8_t *gps_info = gps_dumm;
uint8_t isGNSSTimStart = 0;
volatile uint8_t commandCase = 0;
volatile uint8_t isResponseOk = 0;
volatile uint8_t recResponse = 0;
uint8_t imei[8];
//uint8_t portAdd[MAX_PORT_CHAR] = "12345";
//uint8_t portAdd[MAX_PORT_CHAR] = "6503";//osama portal
uint8_t portAdd[MAX_PORT_CHAR] = "9000";//tanzeel portal
uint8_t domainAdd[MAX_DOMAIN_CHAR] = "182.180.188.205";
//uint8_t domainAdd[] = "\"103.217.177.163\"";
uint8_t msgcleared = 0;
char tcpCommand[50];
char nmeaResponse[NMEA_MAX_LINES][NMEA_MAX_CHARS];
volatile uint8_t isTcpOpen = 0;
uint8_t isReg = 0;
uint8_t isWhereApiCalled = 0;
volatile uint8_t isLoggedIn = 0;
uint8_t processComplete = 0;
uint8_t processCount = 0;
uint8_t tim6Count = 0;
uint16_t infoSNo = 1;
uint8_t loginPacket[18] = { 0x78, 0x78, 0x0D, 0x01, [16]=0x0D, [17]=0x0 };
uint8_t dataPacket[36] = { 0x78, 0x78, 0x1F, 0x12, [34]=0xD, [35]=0xA };
volatile uint8_t savePacket[32];
uint8_t readPacket[32];
uint8_t bunchdata[21][32];
uint8_t heartbeatPacket[15] = { 0x78, 0x78, 0xA, 0x13, [13]=0xD, [14]=0xA };
volatile uint16_t StartN;
volatile uint16_t EndN;
volatile uint16_t StartSec;
volatile uint16_t EndSec;
volatile uint8_t flashready = 0;
uint8_t stats = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

//char* int2string(int num, char *str);
//void clearit();
//void rebootsystem();
//void send_login_packet();
//void send_data_packet();
//void send_hb_packet();
//uint8_t checkdatasize();
//void save_data_packet();
//void where_api_handler();
//uint8_t read_data_packet();
//uint16_t GetCrc16(const uint8_t *pData, int nLength);
//uint8_t get_elements(char *array, uint8_t size);
//void send_command(char *command, uint16_t timeout, uint8_t caseId,
//		uint8_t retryCount, uint8_t isReset);
//void quectel_init();
//uint8_t estabilish_tcp();
//void incoming_msg_handler();
//char* substring(char *destination, const char *source, uint8_t beg, uint8_t n);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t a = 0;

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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&AT_PORT, AT_BUFFER, 1);
	HAL_UART_Receive_IT(&huart2, GNSS_BUFFER, 1);
	W25qxx_Init();
	//INPUT CAPTURE------
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim17);
	W25qxx_EraseSector(0);
	W25qxx_EraseSector(1);
	W25qxx_ReadByte(&isFlash, 0);
	if (isFlash != 1) {
		HAL_UART_Transmit(&huart4, "first time", sizeof("first time"),
				100);
		W25qxx_EraseSector(0);
		W25qxx_EraseSector(1);
		W25qxx_WriteByte(1, 0);
		//printf("chip erased/n");
		StartN = 0;
		EndN = 0;
		StartSec = 1;
		EndSec = 1;
		flashready = 1;
		W25qxx_WriteByte('1', 9);
		W25qxx_WriteByte('2', 10);
		W25qxx_WriteByte('3', 11);
		W25qxx_WriteByte('4', 12);
		W25qxx_WriteByte(0, 1);
		W25qxx_WriteByte(1, 2);
		W25qxx_WriteByte(0, 3);
		W25qxx_WriteByte(0, 4);
		W25qxx_WriteByte(0, 5);
		W25qxx_WriteByte(1, 6);
		W25qxx_WriteByte(0, 7);
		W25qxx_WriteByte(0, 8);
		cPin[0] = '1';
		cPin[1] = '2';
		cPin[2] = '3';
		cPin[3] = '4';
		for (uint8_t te = 13; te < 23; te++) {
			W25qxx_WriteByte(0, te);
			validSender[te - 13] = 0;
		}
		for (uint8_t te = 23; te < 73; te++) {
			W25qxx_WriteByte(domainAdd[te - 23], te);
		}
		for (uint8_t te = 73; te < 79; te++) {
			W25qxx_WriteByte(portAdd[te - 73], te);
		}
	} else {
		HAL_UART_Transmit(&huart4, "reading from rom",
				sizeof("reading from rom"), 100);

		uint8_t myread[78];
		memset(myread, 0, sizeof(myread));
		//printf("already flashed once/n");
		W25qxx_ReadBytes(myread, 1, 78);
		StartSec = myread[0];
		StartSec = StartSec << 8 | myread[1];

		StartN = myread[2];
		StartN = StartN << 8 | myread[3];

		EndSec = myread[4];
		EndSec = EndSec << 8 | myread[5];

		EndN = myread[6];
		EndN = EndN << 8 | myread[7];

		cPin[0] = myread[8];
		cPin[1] = myread[9];
		cPin[2] = myread[10];
		cPin[3] = myread[11];
		for (uint8_t te = 0; te < 10; te++) {
			validSender[te] = myread[te + 12];
		}
		for (uint8_t te = 0; te < 50; te++) {
			domainAdd[te] = myread[te + 22];
		}
		for (uint8_t te = 0; te < 6; te++) {
			portAdd[te] = myread[te + 72];
		}

		HAL_Delay(100);
		flashready = 1;
	}

	//-------------------check if tracker has registered any mobile number?-------------
	isSMSActive=0;
	for(uint8_t a=0;a<6;a++){
		if(validSender[a]!=0){
			isSMSActive=1;
			break;
		}
	}
	//----------------------------------------------------------------------------------



	HAL_GPIO_WritePin(PWR_KEY_GPIO_Port, PWR_KEY_Pin, 1);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(PWR_KEY_GPIO_Port, PWR_KEY_Pin, 0);
	HAL_Delay(5000);
	quectel_init();

	//INPUT CAPTURE------
	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, vals, NUMVAL);
	//-----------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		while (isTcpOpen == 0 && isLoggedIn == 0) {
			stats = 1;
			while (isReg == 0) {
				stats = 2;
				// HAL_UART_Transmit(&huart4, "at+creg",
				// sizeof("at+creg"), 100);
				send_command("AT+CREG?\r\n", 3, 3, 5, 1);
				if (!isReg) {
					HAL_Delay(10000);
					stats = 3;
					rebootCounter++;
					if (rebootCounter > mainCount) {
						rebootsystem();
					}
				}
			}
			if (estabilish_tcp() == 1) {
				stats = 4;
				// HAL_UART_Transmit(&huart4, "Loginpacket sending",
				// sizeof("loginpacket sending"), 100);
				send_login_packet();
				HAL_Delay(5000);
				if (isLoggedIn == 0) {
					// HAL_UART_Transmit(&huart4, "Loginpacket sending",
					// sizeof("loginpacket sending"), 100);
					send_login_packet();
					HAL_Delay(5000);
					if (isLoggedIn == 0) {
						isTcpOpen = 0;
					}
				}
			}
			else {
				stats = 5;
				recTimeA = 0;
				while (recTimeA < rTime) { //18 to 180
					HAL_Delay(1000);
					recTimeA++;
					where_api_handler();
				}
				//SEND LOCATION VIA SMS
				//--------------------------------------------------------------------------
				send_location_packet_via_sms();
				//--------------------------------------------------------
			}
		}
		while (isTcpOpen == 1 && isLoggedIn == 1 && isDataMode == 1) {
			stats = 7;
			HAL_Delay(locationDataIntervalA*1000);
			heartBeatTimer++;
			if (heartBeatTimer > 36) {
				stats = 8;
				isLoggedIn = 0;
				send_hb_packet();
				HAL_Delay(10000);
				heartBeatTimer = 0;
			}
			if (isDataMode == 1 && isLoggedIn == 1 && isTcpOpen == 1) {
				stats = 8;
				incoming_msg_handler();
				send_data_packet();
			}

		}
		isTcpOpen = 0;
		isLoggedIn = 0;
		isDataMode = 0;
		// HAL_UART_Transmit(&huart4, "all zero",
		// sizeof("all zero"), 100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 640;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7200;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7200;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7200;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 4000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WD_GPIO_Port, WD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CTS_Pin|DCD_Pin|LED_1_Pin|LED_2_Pin
                          |PWR_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUTPUT_1_GPIO_Port, OUTPUT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Q_CTRL_Pin|FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : WD_Pin */
  GPIO_InitStruct.Pin = WD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTS_Pin DTR_Pin */
  GPIO_InitStruct.Pin = RTS_Pin|DTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CTS_Pin DCD_Pin LED_1_Pin LED_2_Pin
                           PWR_KEY_Pin */
  GPIO_InitStruct.Pin = CTS_Pin|DCD_Pin|LED_1_Pin|LED_2_Pin
                          |PWR_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OUTPUT_1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUTPUT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_1_Pin */
  GPIO_InitStruct.Pin = INPUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Q_CTRL_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = Q_CTRL_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		MX_USART2_UART_Init();
		HAL_UART_Receive_IT(&GNSS_PORT, GNSS_BUFFER, 1);
	}
	if (huart->Instance == USART1) {
		MX_USART1_UART_Init();
		HAL_UART_Receive_IT(&AT_PORT, AT_BUFFER, 1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//------------------------------------------------------------------
	if (huart == &GNSS_PORT) {
		// Incoming data from GNSS, receiving single byte
		HAL_UART_Receive_IT(&GNSS_PORT, GNSS_BUFFER, 1);
		if (isGNSSTimStart == 0) {
			isGNSSTimStart = 1;
			HAL_TIM_Base_Start_IT(&htim17);
		}
		TIM17->CNT &= 0x0;
		if(GNSS_BUFFER[0] != NULL){
			nmeaResponse[nmeaLC][nmeaCC] = GNSS_BUFFER[0];
			if (GNSS_BUFFER[0] == '\n') {
				nmeaLC++;
				if (nmeaLC > NMEA_MAX_LINES - 1) {
					nmeaLC = NMEA_MAX_LINES -1;
				}
				nmeaCC = 0;
			} else {
				nmeaCC++;
				if (nmeaCC > NMEA_MAX_CHARS - 1) {
					nmeaCC = 0;
				}
			}

		}
	}
	//------------------------------------------------------------------

	if (huart == &AT_PORT) {
		recResponse = 1;
		if (isStart == 0) {
			isStart = 1;
			FIX_TIMER_TRIGGER(&htim16);
			HAL_TIM_Base_Start_IT(&htim16);
		}
		TIM16->CNT &= 0x0;
		HAL_UART_Receive_IT(&AT_PORT, AT_BUFFER, 1);
		if (AT_BUFFER[0] == '\n') {
			if (lineCount > RESPONSE_MAX_LINE - 2) {
				lineCount = 0;
			} else {
				lineCount++;
			}
			charCount = 0;
		} else {
			responseBuffer[lineCount][charCount] = AT_BUFFER[0];
			charCount++;
			if (lineCount > RESPONSE_MAX_CHAR - 2) {
				charCount = 0;
			}
		}
	}
	//-----------------------------------------------------------------------------

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim14) {
		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
		HAL_GPIO_TogglePin(WD_GPIO_Port, WD_Pin);
	}
	if (htim == &htim16) {
		char tResponse = '0';
		//todo
		//---- server response check
		if (responseBuffer[0][0] == 0x78 && responseBuffer[0][1] == 0x78) {
			// HAL_UART_Transmit(&huart4, "Res rec server",
			// sizeof("Res rec server"), 100);
			uint8_t c = responseBuffer[0][2];
			uint8_t tempCrcData[c - 1];
			uint16_t crcResult = 0;
			for (uint8_t i = 2; i < c + 1; i++) {
				tempCrcData[i - 2] = responseBuffer[0][i];
			}
			uint8_t *tempPtr = tempCrcData;
			crcResult = GetCrc16(tempPtr,
					sizeof(tempCrcData) / sizeof(tempCrcData[0]));
			uint16_t checker = responseBuffer[0][c + 1];
			checker = checker << 8 | responseBuffer[0][c + 2];
			if (crcResult == checker) {
				if (responseBuffer[0][3] == 1 || responseBuffer[0][3] == 0x13) {
					isLoggedIn = 1;
					if (responseBuffer[0][3] == 1) {
						// HAL_UART_Transmit(&huart4, "Login rec",
						// sizeof("Login rec"), 100);
					} else {
						// HAL_UART_Transmit(&huart4, "HB rec", sizeof("HB rec"),
						// 100);
					}
				}
			}
			HAL_TIM_Base_Stop_IT(&htim16);
			memset(responseBuffer, 0, sizeof(responseBuffer));
			lineCount = 0;
			charCount = 0;
			isStart = 0;
		} else {
			uint8_t tLine = 99;
			char *ptr;
			uint8_t tIndex;
			//message handling here------------------------------------------
			for (uint8_t i = 0; i <= RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "+CMT:");
				if (ptr != NULL) {
					tLine = i;
					break;
				}
			}
			if (tLine != 99) {
				//some message is received!!!.
				//---check the sender's number.
				char sender[50];
				memset(sender, 0, sizeof(sender));
				ptr = strchr(responseBuffer[tLine], '\"');
				if (ptr != NULL) {
					tIndex = ptr - (char) responseBuffer[tLine];
					substring(sender, responseBuffer[tLine], tIndex + 1, 13);
					substring(sender, sender, 3, 10);
					uint8_t isOwner = 0;
					isOwner = !strcmp(sender, validSender); //check if sender's number is an owner?
					// HAL_UART_Transmit(&huart4, "Sending Sender", 15, 100);
					// HAL_UART_Transmit(&huart4, sender, 10, 100);
					//printf("message received from %s \n", sender);
					//---check the message content for any command. (*auto# for example)
					uint8_t ind1;
					uint8_t ind2; //command length.
					char *x;
					char sCommand[MAX_COMMAND_LEN];
					x = strchr(responseBuffer[tLine + 1], '*');
					if (x != NULL) {
						char *y;
						y = strchr(responseBuffer[tLine + 1], '#');
						if (y != NULL) {
							//command found!!!
							//---extract the command.
							ind1 = x - (char) responseBuffer[tLine + 1] + 1;
							ind2 = y - (char) responseBuffer[tLine + 1] - 1;
							substring(sCommand, responseBuffer[tLine + 1], ind1,ind2);
							//---check command type
							//--->1)RES0 = reset user PIN and registered NUMBER
							//--->2)RNUM3322336979xxxx = register new owner's number
							//--->3)RPIN = set new pin
							//--->4)WHERE
							//--->5)SERVER,DNS,PORT (DNS<=50,PORT<=5)
							if(sCommand[0] == 'R'
							&& sCommand[1] == 'E'
							&& sCommand[2] == 'S'
							&& sCommand[3] == '0'
							&& ind2 == 4) {
								//--- reset command received.
								cPin[0] = '1';
								cPin[1] = '2';
								cPin[2] = '3';
								cPin[3] = '4';
								//---saving to flash memory
								save_to_flash();
								//printf("Reset Completed\n");
							} else if (sCommand[0] == 'R'
									&& sCommand[1] == 'N'
									&& sCommand[2] == 'U'
									&& sCommand[3] == 'M'
									&& ind2 == 18) {
								//---owner number registration command received
								//printf("number registration command received\n");
								if(sCommand[14] == cPin[0]
								&& sCommand[15] == cPin[1]
								&& sCommand[16] == cPin[2]
								&& sCommand[17] == cPin[3]) {
									//PIN is valid!!!
									//---register new number
									for (uint8_t m = 0; m < 10; m++) {
										validSender[m] = sCommand[m + 4];
									}
									save_to_flash();
									isSMSActive=1;
									// HAL_UART_Transmit(&huart4,
									// "NEW NUM SAVED\n", 14, 100);

									//---send success message(todo)
								} else {
									// incorrect pin, send message (incoorect pin),(todo)
									// *future* stop sending message after 3 fails
								}
							} else if (sCommand[0] == 'R'
									&& sCommand[1] == 'P'
									&& sCommand[2] == 'I'
									&& sCommand[3] == 'N'
									&& ind2 == 12
									&& isOwner == 1) {
								//---SET PIN command received from owner.
								// RPINxxxxNNNN
								//printf("set PIN command received\n");
								if (sCommand[4] == cPin[0]
								&& sCommand[5] == cPin[1]
								&& sCommand[6] == cPin[2]
								&& sCommand[7] == cPin[3]) {
									//old PIN is valid!!!
									//---set new pin
									cPin[0] = sCommand[8];
									cPin[1] = sCommand[9];
									cPin[2] = sCommand[10];
									cPin[3] = sCommand[11];
									//---saving to flash memory
									save_to_flash();
									//printf("NEW PIN set \n");
								}
							} else if (sCommand[0] == 'W'
									&& sCommand[1] == 'H'
									&& sCommand[2] == 'E'
									&& sCommand[3] == 'R'
									&& sCommand[4] == 'E'
									&& isOwner == 1) {
								//WHERE API REQUEST RECEIVED
								isWhereApiCalled = 1;
							} else if (sCommand[0] == 'S'
									&& sCommand[1] == 'E'
									&& sCommand[2] == 'R'
									&& sCommand[3] == 'V'
									&& sCommand[4] == 'E'
									&& sCommand[5] == 'R'
									&& sCommand[6] == ','
									&& isOwner == 1) {
								//SERVER CONFIG COMMAND RECEIVED
								check_command_SERVER(sCommand);///handle the SERVER CONFIG COMMAND

							} else if (sCommand[0] == 'T'
									&& sCommand[1] == 'I'
									&& sCommand[2] == 'M'
									&& sCommand[3] == 'E'
									&& sCommand[4] == 'R'
									&& sCommand[5] == ','
									&& isOwner == 1) {
								//TIMER CONFIG COMMAND RECEIVED
								check_command_TIMER(sCommand);///handle the TIMER CONFIG COMMAND

							}
						}

					}

				}

			}
		}
		if (commandCase == 0) {
			char *ptr;
			char *ptr2;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "CLOSED");
				ptr2 = strstr(responseBuffer[i], "+PDP DEACT");
				if (ptr != NULL) {
					// HAL_UART_Transmit(&huart4, "closed recv",
					// sizeof("closed recv"), 100);

					isLoggedIn = 0;
					isDataMode = 0; //command mode activated
					isTcpOpen = 0;
					break;
				}
				if (ptr2 != NULL) {
					// HAL_UART_Transmit(&huart4, "pdp deact recv",
					// sizeof("pdp deact"), 100);
					isLoggedIn = 0;
					isDataMode = 0; //command mode activated
					isTcpOpen = 0;
					break;
				}
			}
			HAL_TIM_Base_Stop_IT(&htim16);
			memset(responseBuffer, 0, sizeof(responseBuffer));
			lineCount = 0;
			charCount = 0;
			isStart = 0;
		}

		// }
		if (commandCase == 1) {
			uint8_t tLine = 99;
			char *ptr;
			char *ptr2;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "OK");
				ptr2 = strstr(responseBuffer[i], "ERROR");

				if (ptr != NULL) {
					tLine = i;
					tResponse = 'G';
					break;
				}
				if (ptr2 != NULL) {
					tLine = i;
					tResponse = 'B';
					break;
				}
			}
			if (tLine != 99) {
				if (tResponse == 'G') {
					isResponseOk = 1;
					clearit();
					commandCase = 0;

				} else if (tResponse == 'B') {
					isResponseOk = 0;
					clearit();
					commandCase = 0;
				}
			} else {

				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						//printf("TIMEOUT HASH TAG\n");
						rebootsystem();

					}
					clearit();
					isResponseOk = 0;

				}
			}
		} else if (commandCase == 2) {
			//CPIN Case
			uint8_t tLine = 99;
			char *ptr;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "READY");
				if (ptr != NULL) {
					tLine = i;
					break;
				}
			}
			if (tLine != 99) {
				isResponseOk = 1;
				clearit();
			} else {
				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();

					}
					clearit();
					isResponseOk = 0;
				}
			}
		} else if (commandCase == 3) {
			// CREG? / CGREG? case
			uint8_t tLine = 99;
			char *ptr;
			char *ptr2;
			char *ptr3;
			char *ptr4;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "+CREG: 0,1");
				ptr2 = strstr(responseBuffer[i], "+CREG: 0,5");
				ptr3 = strstr(responseBuffer[i], "+CGREG: 0,1");
				ptr4 = strstr(responseBuffer[i], "+CGREG: 0,5");
				if (ptr != NULL || ptr2 != NULL || ptr3 != NULL || ptr4 != NULL) {
					tLine = i;
					break;
				}
			}
			if (tLine != 99) {
				isReg = 1;
				isResponseOk = 1;
				clearit();

			} else {
				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();
					}
					clearit();
					isResponseOk = 0;
				}
			}
		}

		else if (commandCase == 4) {
			//CGSN (IMEI) case
			uint8_t tLine = 99;
			char *ptr;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "OK");
				if (ptr != NULL) {
					tLine = i;
					break;
				}
			}
			if (tLine != 99) {
				isResponseOk = 1;
				//to-do with that line
				char p[10];
				memset(p, 0, sizeof(p));
				char *myt;
				myt = responseBuffer[tLine - 2];
				strncpy(p, myt, 1);
				imei[0] = (int) strtol(p, NULL, 16);
				myt++;
				for (int i = 1; i < 8; i++) {
					memset(p, 0, sizeof(p));
					strncpy(p, myt, 2);
					imei[i] = (int) strtol(p, NULL, 16);
					myt += 2;
				}
				clearit();

			} else {
				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();
					}
					isResponseOk = 0;
					clearit();

				}
			}
		} else if (commandCase == 5) {
			//tcp open case
			uint8_t tLine = 99;
			char *ptr;
			char *ptr2;
			char *ptr3;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "ALREADY CONNECT"); //todo check wether in data mode or not
				ptr2 = strstr(responseBuffer[i], "CONNECT FAIL");
				ptr3 = strstr(responseBuffer[i], "CONNECT");

				if (ptr != NULL) {
					tLine = i;
					tResponse = 'G';
					isDataMode = 0;
					break;
				} else if (ptr2 != NULL) {
					tLine = i;
					tResponse = 'B';
					isDataMode = 0;
					break;
				} else if (ptr3 != NULL) {
					tLine = i;
					tResponse = 'G';
					isDataMode = 1;
					break;
				}
			}
			if (tLine != 99) {
				if (tResponse == 'G') {
					isResponseOk = 1;
					//to-do with that line
					clearit();
					isTcpOpen = 1;
					isDataMode = 1;

				} else if (tResponse == 'B') {
					isResponseOk = 0;
					clearit();
					isTcpOpen = 0;
				}
			} else {
				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();
					}
					clearit();
					isResponseOk = 0;

				}
			}
		}
		if (commandCase == 6) {
			uint8_t tLine = 99;
			char *ptr;
			char *ptr2;
			char *ptr3;

			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "NO CARRIER");
				ptr2 = strstr(responseBuffer[i], "ERROR");
				ptr3 = strstr(responseBuffer[i], "CONNECT");

				if (ptr != NULL) {
					tLine = i;
					tResponse = 'B';
					break;
				}
				if (ptr2 != NULL) {
					tLine = i;
					tResponse = 'B';
					break;
				}
				if (ptr3 != NULL) {
					tLine = i;
					tResponse = 'G';
					break;
				}
			}
			if (tLine != 99) {
				if (tResponse == 'G') {
					isResponseOk = 1;
					//to-do with that line
					clearit();

				} else if (tResponse == 'B') {
					isResponseOk = 0;
					//to-do with that line
					clearit();
				}
			} else {

				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();
					}
					clearit();
					isResponseOk = 0;

				}
			}
		} else if (commandCase == 7) { //cmgs response check
			uint8_t tLine = 99;
			char *ptr;
			char *ptr2;
			for (uint8_t i = 0; i < RESPONSE_MAX_LINE; i++) {
				ptr = strstr(responseBuffer[i], "+CMGS:");
				ptr2 = strstr(responseBuffer[i], "ERROR");
				if (ptr != NULL) {
					tLine = i;
					tResponse = 'G';
					break;
				}
				if (ptr2 != NULL) {
					tLine = i;
					tResponse = 'B';
					break;
				}
			}
			if (tLine != 99) {
				if (tResponse == 'G') {
					isResponseOk = 1;
					clearit();
					//to-do with that line

				} else if (tResponse == 'B') {
					isResponseOk = 1;
					clearit();
					//to-do with that line

				}
			} else {
				resTimeout--;
				if (resTimeout < 1) {
					if (!recResponse) {
						rebootsystem();
					}
					isResponseOk = 0;
					clearit();
				}
			}

		}

	} else if (htim == &htim17) {

		//---------------------GNSS Timer-------------------------------------

		for(uint8_t tLine =0; tLine< nmeaLC; tLine++){
			uint8_t commandSize = 0;
			char *tempSentenceCheck;
			tempSentenceCheck = strstr(nmeaResponse[tLine],"VTG");
			if(tempSentenceCheck != NULL){
				//Disable all nmea sentences except GGA and RMC
				HAL_UART_Transmit(&GNSS_PORT, "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n",51,2000);
				HAL_UART_Transmit(&huart4, "GNS SET\r\n", 9, 500);
			}
			for(uint8_t tChar =0 ;tChar<NMEA_MAX_CHARS;tChar++) {
				if(nmeaResponse[tLine][tChar] != NULL){
					commandSize++;
				}
				else{
					break;
				}
			}
			//SENT NMEA SENTENCES TO DEBUG PORT
			// HAL_UART_Transmit(&huart4, nmeaResponse[tLine],commandSize, 1000);
		}

		indicationCounter++;


		// char mynmea[6][85] = {"$GNRMC,082435.000,A,2451.5794,N,06703.2442,E,0.35,156.75,160223,,,D*70","xxx","$GNGGA,082435.000,2451.5794,N,06703.2442,E,2,9,0.98,35.1,M,-46.6,M,,*51"};
		uint8_t *tempGps;
		tempGps = nmea_parser(nmeaResponse, NMEA_MAX_CHARS,&crcc,&rCrc);
		if (tempGps != NULL) {
			gps_info = tempGps;
		}
		else{
//			HAL_UART_Transmit(&huart4, "WRONG", 5, 100);
		}
		HAL_TIM_Base_Stop_IT(&htim17);
		isGNSSTimStart = 0;
		if (tim6Count > 5) {
			if (isLoggedIn == 0 && isTcpOpen == 0 && flashready == 1) {
				save_data_packet();
			}
			tim6Count = 0;
		} else {
			tim6Count++;
		}
		memset(nmeaResponse, 0, sizeof(nmeaResponse));
		nmeaLC = 0;
		nmeaCC = 0;
		HAL_UART_Receive_IT(&GNSS_PORT, GNSS_BUFFER, 1);
		HAL_UART_Receive_IT(&GNSS_PORT, GNSS_BUFFER, 1);
	}
}

void rebootsystem() {
	save_to_flash();
	// printf("rebooting system \n");
	NVIC_SystemReset();

	// todo save flash info
}
void save_to_flash() {
	W25qxx_EraseSector(0);
	W25qxx_WriteByte(1, 0);
	uint8_t t[2];
	t[0] = StartSec >> 8;
	t[1] = StartSec;
	W25qxx_WriteByte(t[0], 1);
	W25qxx_WriteByte(t[1], 2);
	t[0] = StartN >> 8;
	t[1] = StartN;
	W25qxx_WriteByte(t[0], 3);
	W25qxx_WriteByte(t[1], 4);
	t[0] = EndSec >> 8;
	t[1] = EndSec;
	W25qxx_WriteByte(t[0], 5);
	W25qxx_WriteByte(t[1], 6);
	t[0] = EndN >> 8;
	t[1] = EndN;
	W25qxx_WriteByte(t[0], 7);
	W25qxx_WriteByte(t[1], 8);

	W25qxx_WriteByte(cPin[0], 9);
	W25qxx_WriteByte(cPin[1], 10);
	W25qxx_WriteByte(cPin[2], 11);
	W25qxx_WriteByte(cPin[3], 12);

	for (uint8_t te = 13; te < 23; te++) {
		W25qxx_WriteByte(validSender[te - 13], te);
	}

	for (uint8_t te = 23; te < 73; te++) {
		W25qxx_WriteByte(domainAdd[te - 23], te);
	}

	for (uint8_t te = 73; te < 79; te++) {
		W25qxx_WriteByte(portAdd[te - 73], te);
	}
	HAL_Delay(100);
	// HAL_UART_Transmit(&huart4, "saved to flash",
	// sizeof("saved to flash"), 100);

}

void send_command(char *command, uint16_t timeout, uint8_t caseId,
		uint8_t retryCount, uint8_t isReset) {
	uint8_t processComplete = 0, processCount = 0;
	uint16_t commandSize = 0;
	uint16_t i = 0;
	while (command[i] != NULL) {
		commandSize++;
		i++;
	}
	while (processComplete == 0) {
		while (isBusy)
			;
		isBusy = 1;
		isResponseOk = 0;
		commandCase = caseId;
		isStart = 1;
		recResponse = 0;
		HAL_UART_Transmit(&AT_PORT, command, commandSize, 1000);
		FIX_TIMER_TRIGGER(&htim16);
		HAL_TIM_Base_Start_IT(&htim16);
		resTimeout = timeout; //300 ms
		while (isBusy)
			;
		commandCase = 0;
		if (isResponseOk) {
			processComplete = 1;
		} else {
			//printf("failed\n");
			processCount++;
			if (processCount > retryCount) {
				if (isReset == 1) {
					rebootsystem();
				}
				break;
			}
			HAL_Delay(5000);
		}
	}
}
void quectel_init() {
	// printf("--Sending AT-- \n");
	send_command("AT\r\n", 3, 1, 1, 1);
	// printf("--sending AT+QIURC=1--\n");
	// send_command("AT+QIURC=1\r\n", 3, 1, 1,1);
	// printf("--Sending AT+CPIN-- \n");
	send_command("AT+CPIN?\r\n", 51, 2, 2, 1);
	// printf("--Sending AT+CREG?-- \n");
	// send_command("AT+CREG?\r\n",3,3,5,1);
	// printf("--Sending AT+CGREG?-- \n");
	// send_command("AT+CGREG?\r\n",3,3,3,1);
	// printf("--Sending AT+CMGF=1-- \n");
	send_command("AT+CMGF=1\r\n", 3, 1, 3, 1);
	// printf("--Sending AT+CNMI=2,2-- \n");
	send_command("AT+CNMI=2,2\r\n", 3, 1, 3, 1);
	// printf("--Sending AT+CGSN--\r\n \n");
	send_command("AT+CGSN\r\n", 3, 4, 2, 1);
	send_command("AT+QMGDA=\"DEL ALL\"\r\n", 50, 1, 0, 0);

}

uint8_t estabilish_tcp() {
	// HAL_UART_Transmit(&huart4, "est tcp",
	// sizeof("est tcp"), 100);
	send_command("+++", 10, 1, 0, 0);
	memset(tcpCommand, 0, sizeof(tcpCommand));
	strcat(tcpCommand, "AT+QIOPEN=\"TCP\",\"");
	strcat(tcpCommand, domainAdd);
	strcat(tcpCommand, "\",");
	strcat(tcpCommand, portAdd);
	strcat(tcpCommand, "\r\n");
	// printf("--Sent AT+QIDEACT \n");
	send_command("AT+QIDEACT\r\n", 401, 1, 2, 0);

	if (isResponseOk == 0) {
		return 0;
	}
	send_command("AT+QIMODE=1\r\n", 3, 1, 3, 0);
	if (isResponseOk == 0) {
		return 0;
	}
	send_command("AT+QITCFG=3,2,512,1\r\n", 3, 1, 1, 0);
	if (isResponseOk == 0) {
		return 0;
	}
	send_command("AT+QIREGAPP=\"network\",\"\",\"\"\r\n", 3, 1, 1, 0);
	if (isResponseOk == 0) {
		return 0;
	}
	send_command("AT+QIACT\r\n", 15100, 1, 0, 0);
	if (isResponseOk == 0) {
		return 0;
	}
	// HAL_UART_Transmit(&huart4, "S qiopen", sizeof("S qiopen"), 100);

	send_command(tcpCommand, 7510, 5, 2, 0);
	if (isResponseOk == 1) {
		return 1;
	} else {
		return 0;
	}
}

uint16_t GetCrc16(const uint8_t *pData, int nLength) {
	uint16_t fcs = 0xffff; // initialization
	int a = 0;
	while (nLength > 0) {
		a = (fcs ^ *pData) & 0xff;
		fcs = (fcs >> 8) ^ crctab16[a];
		nLength--;
		pData++;
	}
	return ~fcs; // negated
}

void where_api_handler() {
	if (isWhereApiCalled == 1) {
		// printf("--Sending AT+CREG?-- \n");
		// HAL_UART_Transmit(&huart4, "S creg in api", sizeof("S creg in api"),
		// 100);
		send_command("AT+CREG?\r\n", 3, 3, 3, 1);
		if (isReg == 1) {
			send_current_location_via_sms(); //sending current location
			isWhereApiCalled = 0;
		}
	}
}

void clearit() {
	resTimeout = 3;
	HAL_TIM_Base_Stop_IT(&htim16);
	memset(responseBuffer, 0, sizeof(responseBuffer));
	lineCount = 0;
	charCount = 0;
	isStart = 0;
	isBusy = 0;

}

void incoming_msg_handler() {
	if (isPulse == 1) {
		isPulse = 0;
		//printf("--Sending +++-- \n");
		//HAL_UART_Transmit(&huart4, "S +++ incom", sizeof("S +++ incom"), 100);
		send_command("+++", 10, 1, 0, 0);
		isDataMode = 0;
		indicationCounter = 0;
		while (indicationCounter < 5)
			;
		where_api_handler();
		//HAL_UART_Transmit(&huart4, "S msg del", sizeof("S msg del"), 100);
		send_command("AT+QMGDA=\"DEL ALL\"\r\n", 50, 1, 0, 0);
		msgcleared = 0;
		if (isResponseOk == 1) {
			msgcleared = 1;
		}
		//printf("--Sending ATO-- \n");
		send_command("ATO\r\n", 10, 6, 0, 0);
		if (isResponseOk == 1) {
			isDataMode = 1;
			// HAL_UART_Transmit(&huart4, "conn resum",
			// sizeof("conn resum"), 100);
		} else {
			isLoggedIn = 0;
			isDataMode = 0;
			isTcpOpen = 0;
		}
	}

}

void send_login_packet() {
	if (isTcpOpen == 1 && isDataMode == 1) {
		for (uint8_t i = 0; i < 8; i++) {
			loginPacket[i + 4] = imei[i];
		}
		loginPacket[12] = infoSNo >> 8;
		loginPacket[13] = infoSNo;
		uint8_t tempCrcData[12];
		for (uint8_t i = 0; i < 12; i++) {
			tempCrcData[i] = loginPacket[i + 2];
		}
		uint8_t *tempPtr = tempCrcData;
		uint16_t crcResult = 0;
		crcResult = GetCrc16(tempPtr,
				sizeof(tempCrcData) / sizeof(tempCrcData[0]));
		loginPacket[14] = crcResult >> 8;
		loginPacket[15] = crcResult;
		HAL_UART_Transmit(&AT_PORT, loginPacket, 18, 100);
		//printf("SENT LOGING PACKET SUCCESSFULLY\n");
		infoSNo++;

	} else {
		//printf("TCP SESSION NOT OPENED\n");
	}
}

void send_data_packet() {
	infoSNo++;
	uint8_t sendCounter = 0;
	while (read_data_packet() == 1 && sendCounter < 100 && isLoggedIn == 1
			&& isDataMode == 1 && isTcpOpen == 1) {
		for (uint8_t i = 0; i < 18; i++) {
			dataPacket[i + 4] = readPacket[i];
		}
		dataPacket[30] = infoSNo >> 8;
		dataPacket[31] = infoSNo;
		uint8_t tempCrcData[30];
		for (uint8_t i = 0; i < 29; i++) {
			tempCrcData[i] = dataPacket[i + 2];
		}
		uint8_t *tempPtr = tempCrcData;
		uint16_t crcResult = 0;
		crcResult = GetCrc16(tempPtr,
				sizeof(tempCrcData) / sizeof(tempCrcData[0]));
		dataPacket[32] = crcResult >> 8;
		dataPacket[33] = crcResult;
		HAL_UART_Transmit(&AT_PORT, dataPacket, 36, 100);
		// HAL_UART_Transmit(&huart4, dataPacket, 36, 100);
		sendCounter++;
		HAL_Delay(1000);
	}
	if (read_data_packet() == 0) {
		for (uint8_t i = 0; i < 18; i++) {
			dataPacket[i + 4] = gps_info[i];
		}
		dataPacket[30] = infoSNo >> 8;
		dataPacket[31] = infoSNo;
		uint8_t tempCrcData[30];
		for (uint8_t i = 0; i < 29; i++) {
			tempCrcData[i] = dataPacket[i + 2];
		}
		uint8_t *tempPtr = tempCrcData;
		uint16_t crcResult = 0;
		crcResult = GetCrc16(tempPtr,
				sizeof(tempCrcData) / sizeof(tempCrcData[0]));
		dataPacket[32] = crcResult >> 8;
		dataPacket[33] = crcResult;
		HAL_UART_Transmit(&AT_PORT, dataPacket, 36, 100);

		// HAL_UART_Transmit(&huart4, dataPacket, 36, 100);

	}
}
uint8_t checkdatasize() {
	if (StartSec == EndSec) {
		if ((StartN - EndN) >= 672) {
			return 1;
		} else {
			return 0;
		}
	} else if ((StartSec - EndSec) == 1) {
		if ((4096 - EndN + StartN) >= 672) {
			return 1;
		} else {
			return 0;
		}
	} else {
		return 1;
	}
}

void save_data_packet() {
	memset(savePacket, 0, sizeof(savePacket));
	for (uint8_t i = 0; i < 18; i++) {
		savePacket[i] = gps_info[i];
	}
	W25qxx_WriteSector(savePacket, StartSec, StartN, 32);
	StartN = StartN + 32;
	if (StartN > 4090) {
		StartN = 0;
		StartSec += 1;
		if (StartSec == 1024) {
			StartSec = 1;
		}
		W25qxx_EraseSector(StartSec);
		if (StartSec == EndSec) {
			EndN = 0;
			if (EndSec == 1023) {
				EndSec = 1;
			} else {
				EndSec += 1;
			}
		}
	}
}
uint8_t read_data_packet() {
	memset(readPacket, 0, sizeof(readPacket));
	if ((EndSec == StartSec) && (EndN == StartN)) {
		if (EndN != 0 || EndSec != 1) {
			W25qxx_EraseSector(1);
			StartN = 0;
			EndN = 0;
			StartSec = 1;
			EndSec = 1;
		}
		return 0;
	} else {
		W25qxx_ReadSector(readPacket, EndSec, EndN, 32);
		//reading data//
		EndN = EndN + 32;
		if (EndN > 4090) {
			if (EndSec == 1023) {
				EndSec = 1;
				EndN = 0;
			} else {
				EndSec = EndSec + 1;
				EndN = 0;
			}
		}
		return 1;
	}
}

void send_hb_packet() {
	if (isTcpOpen == 1 && isDataMode == 1) {
		uint8_t TermInfo = 0;
		uint8_t VLvl;
		uint8_t SigStre = 20;
		uint8_t GSMSS;

		int voltage = 4400;
		//if relay is cut
		if (0) {
			TermInfo = TermInfo | 0x80;
		}
		//if gps tracking is on

		if (1) {
			TermInfo = TermInfo | 0x40;
		}
		//if SOS is on

		if (1) {
			TermInfo = TermInfo | 0x20;
		}
		//if Low batt alarm is on

		if (1) {
			TermInfo = TermInfo | 0x18;
		}
		//if Power Cut alarm is on

		if (1) {
			TermInfo = TermInfo | 0x10;
		}
		//if shock alarm is on
		if (1) {
			TermInfo = TermInfo | 0x8;
		}
		// 000 means normal
		//if charge is on
		if (1) {
			TermInfo = TermInfo | 0x4;
		}
		//if ACC is on
		if (1) {
			TermInfo = TermInfo | 0x2;
		}
		//if Activated
		if (1) {
			TermInfo = TermInfo | 0x1;
		}
		if (voltage > 4400) {
			VLvl = 6;
		} else if (voltage > 4100) {
			VLvl = 5;

		} else if (voltage > 4000) {
			VLvl = 4;

		} else if (voltage > 3900) {
			VLvl = 3;

		} else if (voltage > 3800) {
			VLvl = 2;

		} else if (voltage > 3700) {
			VLvl = 1;

		} else {
			VLvl = 0;

		}
		if (SigStre > 19) {
			GSMSS = 4;
		} else if (SigStre > 14) {
			GSMSS = 3;
		} else if (SigStre > 9) {
			GSMSS = 2;
		} else if (SigStre > 1) {
			GSMSS = 1;
		} else {
			GSMSS = 0;
		}

		heartbeatPacket[4] = TermInfo;
		heartbeatPacket[5] = VLvl;
		heartbeatPacket[6] = GSMSS;
		heartbeatPacket[7] = 0;
		heartbeatPacket[8] = 2;
		heartbeatPacket[9] = infoSNo >> 8;
		heartbeatPacket[10] = infoSNo;

		uint8_t tempCrcData[9];
		for (uint8_t i = 0; i < 10; i++) {
			tempCrcData[i] = heartbeatPacket[i + 2];
		}
		uint8_t *tempPtr = tempCrcData;
		uint16_t crcResult = 0;
		crcResult = GetCrc16(tempPtr,
				sizeof(tempCrcData) / sizeof(tempCrcData[0]));
		heartbeatPacket[11] = crcResult >> 8;
		heartbeatPacket[12] = crcResult;
		HAL_UART_Transmit(&AT_PORT, heartbeatPacket, 15, 100);
		infoSNo++;

	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (isDataMode == 1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // if the interrupt source is channel1
			uint16_t IC_Val1, IC_Val2;
			IC_Val1 = vals[0];
			IC_Val2 = vals[1];
			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			} else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}
			float refClock = TIMCLOCK / (PRESCALAR);
			float mFactor = 1000 / refClock;
			usWidth = Difference * mFactor;
			if (usWidth > 100 && usWidth < 200) {
				//printf("Got message indication\n");
				isPulse = 1;
			}
		}
	}
}

char* substring(char *destination, const char *source, uint8_t beg, uint8_t n) {
	// extracts `n` characters from the source string starting from `beg` index
	// and copy them into the destination string
	while (n > 0) {
		*destination = *(source + beg);
		destination++;
		source++;
		n--;
	}

	// null terminate destination string
	*destination = '\0';

	// return the destination string
	return destination;
}

void send_current_location_via_sms() {
	if (isSMSActive == 1) {
		//printf("sending message(current location)\n");
		// send_command("AT+CMGS=\"3352093997\"\n\r", 10, 6, 0, 0);
		char tempMsg[150];
		uint8_t speed = 0;
		uint32_t lat = 0, lon = 0;
		double tempFloat;
		double tempMin;
		uint8_t tempDeg;
		char buf[15];
		float latitude, longitude;
		lat = (gps_info[7] << 24) | (gps_info[8] << 16) | (gps_info[9] << 8)
		| gps_info[10];
		lon = (gps_info[11] << 24) | (gps_info[12] << 16) | (gps_info[13] << 8)
		| gps_info[14];
		speed = gps_info[15];
		tempFloat = lat / 30000.0;
		tempDeg = (int) tempFloat / 60;
		tempMin = (int) tempFloat % 60;
		tempFloat = tempFloat - (int) tempFloat;
		tempMin = tempMin + tempFloat;
		latitude = tempMin / 60.0;
		latitude += tempDeg;
		tempFloat = lon / 30000.0;
		tempDeg = (int) tempFloat / 60;
		tempMin = (int) tempFloat % 60;
		tempFloat = tempFloat - (int) tempFloat;
		tempMin = tempMin + tempFloat;
		longitude = tempMin / 60.0;
		longitude += tempDeg;
		memset(tempMsg, 0, sizeof(tempMsg));
		memset(buf, 0, sizeof(buf));
		gcvt(latitude, 8, buf);
		strcat(tempMsg,"AT+CMGS=\"");
		strcat(tempMsg,validSender);
		strcat(tempMsg,"\"\r");
//		strcat(tempMsg, "AT+CMGS=\"3322336979\"\r");
		strcat(tempMsg, buf);
		memset(buf, 0, sizeof(buf));
		gcvt(longitude, 8, buf);
		strcat(tempMsg, ",");
		strcat(tempMsg, buf);
		strcat(tempMsg, ",");
		memset(buf, 0, sizeof(buf));
		int2string(speed, buf);
		strcat(tempMsg, buf);
		uint8_t tempCount = 0;

		//todo replace while with for loop
		while (tempMsg[tempCount] != NULL) {
			tempCount++;
		}

		tempMsg[tempCount] = 26;
		//printf("--Sending message to mobile \n");

		send_command(tempMsg, 12005, 7, 0, 0);

		// char tecMsg[] = {'A','T','+','C','M','G','S','=','\"','3','3','2','2','3','3','6','9','7','9','\"','\r','h','e','l','l','o',26,0};

		//    send_command(tecMsg, 12005, 7, 0, 0);
	}
}
void send_location_packet_via_sms(){
	if(isSMSActive==1){
		msgCounter = 0;
		uint8_t dataSize = 0;
		dataSize = checkdatasize();
		while (dataSize == 1 && msgCounter < msgCount) {
			stats = 6;
			uint8_t loopCount = 0;
			memset(bunchdata, 0, sizeof(bunchdata));
			//printf("--Sending AT+CREG?-- \n");
			send_command("AT+CREG?\r\n", 3, 3, 3, 1);
			if (isReg == 1) {
				while (read_data_packet() == 1 && loopCount < smsBunch) {
					//printf("readed the data \n");
					for (uint8_t i = 0; i < 18; i++) {
						bunchdata[loopCount][i] = readPacket[i];
					}
					loopCount++;
				}
				char temMsg[1000];
				int n = 21;
				int tempCt = 0;
				memset(temMsg, 0, sizeof(temMsg));
				strcat(temMsg,"AT+CMGS=\"");
				strcat(temMsg,validSender);
				strcat(temMsg,"\"\r");
	//			strcat(temMsg, "AT+CMGS=\"3322336979\"\r");
				// todo send 21 msg packet
				for (uint8_t i = 0; i < loopCount; i++) {
					for (uint8_t y = 0; y < 18; y++) {
						n += sprintf(&temMsg[n], "%d", bunchdata[i][y]);
					}
					while (temMsg[tempCt] != NULL) {
						tempCt++;
					}
					temMsg[tempCt] = ',';
					n++;
				}
				tempCt = 0;
				while (temMsg[tempCt] != NULL) {
					tempCt++;
				}
				temMsg[tempCt] = 26;
				send_command(temMsg, 12005, 7, 0, 0);
				dataSize = checkdatasize();
				msgCounter++;
			} else {
				break;
			}
		}
	}
}
char* int2string(int num, char *str) {
	if (str == NULL) {
		return NULL;
	}
	sprintf(str, "%d", num);
	return str;
}

void check_command_SERVER(char* command){
		    //check for data integrity by counting commas.
		    //there must be 2 commas in total.
		    uint8_t commaPosition[2]={0,0};
		    uint8_t totalCommas=0;
		    for (uint8_t a=0;a<MAX_COMMAND_LEN;a++){
		        if(command[a]==','){
		            if(totalCommas<2){
		                commaPosition[totalCommas]=a;
		            }
		            totalCommas++;
		        }
		    }
		    if(totalCommas ==2 && commaPosition[0] == 6 ){
		        //two commas found, and first one is on 6th index.
		        //data is good.
		    	memset(portAdd,0,sizeof(portAdd));
		    	memset(domainAdd,0,sizeof(domainAdd));

		        //extract dns
	            for(uint8_t a=commaPosition[0]+1;a<commaPosition[1];a++){
	                    domainAdd[a-(commaPosition[0]+1)]=command[a];
	                }
	            //extract port
	    	    for(uint8_t a=commaPosition[1]+1;a<commaPosition[1]+7;a++){
	    	        if(command[a]!=NULL){
	    	            portAdd[a-(commaPosition[1]+1)] = command[a];
	    	        }
	    	    }
	    	    //todo save to flash please.
		    }
		    else{
//		        printf("Data is bad");
		    }
}
void check_command_TIMER(char* command){
    char t1[4],t2[4];
//    uint8_t timer1=5,timer2=5;

    //check for data integrity by counting commas.
    //there must be 2 commas in total.
    //t1 and t2 both must no be greater than 3 chars.

    uint8_t commaPosition[2]={0,0};
    uint8_t totalCommas=0;
    for (uint8_t a=0;a<MAX_COMMAND_LEN;a++){
        if(command[a]==','){
            if(totalCommas<2){
                commaPosition[totalCommas]=a;
            }
            totalCommas++;
        }
    }
    uint8_t comaDiff = 0;
    comaDiff = commaPosition[1] - commaPosition[0];
    if(totalCommas ==2
    && commaPosition[0] == 5
    && comaDiff < 5
    && comaDiff > 1){
        //two commas found, and first one is on 5th index.
        //t1 has 1-3 chars
        //data is good.
    	memset(t1,0,sizeof(t1));
    	memset(t2,0,sizeof(t2));

        //extract t1
        for(uint8_t a=commaPosition[0]+1;a<commaPosition[1];a++){
                t1[a-(commaPosition[0]+1)]=command[a];
            }
        //extract t2
	    for(uint8_t a=commaPosition[1]+1;a<commaPosition[1]+4;a++){
	        if(command[a]!=NULL){
	            t2[a-(commaPosition[1]+1)] = command[a];
	        }
	    }
	    locationDataIntervalA = atoi(t1);
	    locationDataIntervalB = atoi(t2);
	    locationDataIntervalA = locationDataIntervalA > 180 ? 180 : locationDataIntervalA;
	    locationDataIntervalB = locationDataIntervalB > 180 ? 180 : locationDataIntervalB;
	    //todo save to flash please.
    }
    else{
//		        printf("Data is bad");
    }

}
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
