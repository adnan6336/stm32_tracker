/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void send_current_location_via_sms();
void send_location_packet_via_sms();
char* int2string(int num, char *str);
void clearit();
void rebootsystem();
void send_login_packet();
void send_data_packet();
void send_hb_packet();
uint8_t checkdatasize();
void save_data_packet();
void where_api_handler();
uint8_t read_data_packet();
uint16_t GetCrc16(const uint8_t *pData, int nLength);
uint8_t get_elements(char *array, uint8_t size);
void send_command(char *command, uint16_t timeout, uint8_t caseId,
		uint8_t retryCount, uint8_t isReset);
void quectel_init();
uint8_t estabilish_tcp();
void incoming_msg_handler();
char* substring(char *destination, const char *source, uint8_t beg, uint8_t n);
void alarm_sender();
void save_to_flash(uint8_t autoRstValue);
void check_command_SERVER(char* command);
void check_command_TIMER(char* command);
void check_command_MSGCFG(char* command);
void check_command_RELAY(char* sCommand);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_TX_Pin GPIO_PIN_0
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_1
#define DEBUG_RX_GPIO_Port GPIOA
#define GNS_TX_Pin GPIO_PIN_2
#define GNS_TX_GPIO_Port GPIOA
#define GNS_RX_Pin GPIO_PIN_3
#define GNS_RX_GPIO_Port GPIOA
#define WD_Pin GPIO_PIN_4
#define WD_GPIO_Port GPIOA
#define RI_Pin GPIO_PIN_6
#define RI_GPIO_Port GPIOA
#define OUTPUT_1_Pin GPIO_PIN_1
#define OUTPUT_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOB
#define DTR_Pin GPIO_PIN_10
#define DTR_GPIO_Port GPIOB
#define DCD_Pin GPIO_PIN_11
#define DCD_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOB
#define PWR_KEY_Pin GPIO_PIN_13
#define PWR_KEY_GPIO_Port GPIOB
#define INPUT_1_Pin GPIO_PIN_7
#define INPUT_1_GPIO_Port GPIOC
#define G_CTRL_Pin GPIO_PIN_1
#define G_CTRL_GPIO_Port GPIOD
#define Q_CTRL_Pin GPIO_PIN_2
#define Q_CTRL_GPIO_Port GPIOD
#define FLASH_CS_Pin GPIO_PIN_3
#define FLASH_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
