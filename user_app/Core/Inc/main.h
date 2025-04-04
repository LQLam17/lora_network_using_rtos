/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_SIM_Pin GPIO_PIN_2
#define USART2_TX_SIM_GPIO_Port GPIOA
#define USART2_RX_SIM_Pin GPIO_PIN_3
#define USART2_RX_SIM_GPIO_Port GPIOA
#define LORA_RST_Pin GPIO_PIN_4
#define LORA_RST_GPIO_Port GPIOA
#define SPI1_SCK_LORA_Pin GPIO_PIN_5
#define SPI1_SCK_LORA_GPIO_Port GPIOA
#define SPI1_MISO_LORA_Pin GPIO_PIN_6
#define SPI1_MISO_LORA_GPIO_Port GPIOA
#define SPI1_MOSI_LORA_Pin GPIO_PIN_7
#define SPI1_MOSI_LORA_GPIO_Port GPIOA
#define LORA_DIO0_Pin GPIO_PIN_0
#define LORA_DIO0_GPIO_Port GPIOB
#define LORA_DIO0_EXTI_IRQn EXTI0_IRQn
#define BUTTON1_Pin GPIO_PIN_1
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON1_EXTI_IRQn EXTI1_IRQn
#define BUTTON2_Pin GPIO_PIN_2
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON2_EXTI_IRQn EXTI2_IRQn
#define SPI1_NSS_Pin GPIO_PIN_8
#define SPI1_NSS_GPIO_Port GPIOA
#define USART1_TX_ESP_Pin GPIO_PIN_9
#define USART1_TX_ESP_GPIO_Port GPIOA
#define USART1_RX_ESP_Pin GPIO_PIN_10
#define USART1_RX_ESP_GPIO_Port GPIOA
#define BUTTON3_Pin GPIO_PIN_3
#define BUTTON3_GPIO_Port GPIOB
#define BUTTON3_EXTI_IRQn EXTI3_IRQn
#define I2C1_SCL_EEPROM_Pin GPIO_PIN_6
#define I2C1_SCL_EEPROM_GPIO_Port GPIOB
#define I2C1_SDA_EEPROM_Pin GPIO_PIN_7
#define I2C1_SDA_EEPROM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
