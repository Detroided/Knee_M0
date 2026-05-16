/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define VIBRO_MOTOR_CTR_Pin GPIO_PIN_1
#define VIBRO_MOTOR_CTR_GPIO_Port GPIOC
#define U11_CTR_VT_Pin GPIO_PIN_2
#define U11_CTR_VT_GPIO_Port GPIOC
#define ADC_13_V_DC_DC_Pin GPIO_PIN_3
#define ADC_13_V_DC_DC_GPIO_Port GPIOC
#define DEBUG_LED1_CTR_Pin GPIO_PIN_1
#define DEBUG_LED1_CTR_GPIO_Port GPIOA
#define UART2_TX_BLE_Pin GPIO_PIN_2
#define UART2_TX_BLE_GPIO_Port GPIOA
#define UART2_RX_BLE_Pin GPIO_PIN_3
#define UART2_RX_BLE_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_SCK_ADC_Pin GPIO_PIN_5
#define SPI1_SCK_ADC_GPIO_Port GPIOA
#define SPI1_MISO_ADC_Pin GPIO_PIN_6
#define SPI1_MISO_ADC_GPIO_Port GPIOA
#define SPI1_MOSI_ADC_Pin GPIO_PIN_7
#define SPI1_MOSI_ADC_GPIO_Port GPIOA
#define ADC_8_U6_POWER_IN_V_Pin GPIO_PIN_0
#define ADC_8_U6_POWER_IN_V_GPIO_Port GPIOB
#define ADC_9_U6_POWER_OUT_V_Pin GPIO_PIN_1
#define ADC_9_U6_POWER_OUT_V_GPIO_Port GPIOB
#define I2C2_SCL_24Cxx_EEPROM_Pin GPIO_PIN_10
#define I2C2_SCL_24Cxx_EEPROM_GPIO_Port GPIOB
#define I2C2_SDA_24Cxx_EEPROM_Pin GPIO_PIN_11
#define I2C2_SDA_24Cxx_EEPROM_GPIO_Port GPIOB
#define BLE_TRS_CTR_Pin GPIO_PIN_12
#define BLE_TRS_CTR_GPIO_Port GPIOB
#define UART1_TX_DEBUG_Pin GPIO_PIN_9
#define UART1_TX_DEBUG_GPIO_Port GPIOA
#define UART1_RX_DEBUG_Pin GPIO_PIN_10
#define UART1_RX_DEBUG_GPIO_Port GPIOA
#define UART3_TX_MOTOR_Pin GPIO_PIN_10
#define UART3_TX_MOTOR_GPIO_Port GPIOC
#define UART3_RX_MOTOR_Pin GPIO_PIN_11
#define UART3_RX_MOTOR_GPIO_Port GPIOC
#define BLE_RST_Pin GPIO_PIN_2
#define BLE_RST_GPIO_Port GPIOD
#define BLE_PWR_EN_Pin GPIO_PIN_3
#define BLE_PWR_EN_GPIO_Port GPIOB
#define IDICATION_PWR_CTR_Pin GPIO_PIN_4
#define IDICATION_PWR_CTR_GPIO_Port GPIOB
#define I2C1_SCL_MPU6050_Pin GPIO_PIN_6
#define I2C1_SCL_MPU6050_GPIO_Port GPIOB
#define I2C1_SDA_MPU6050_Pin GPIO_PIN_7
#define I2C1_SDA_MPU6050_GPIO_Port GPIOB
#define MPU6050_INT_Pin GPIO_PIN_8
#define MPU6050_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
