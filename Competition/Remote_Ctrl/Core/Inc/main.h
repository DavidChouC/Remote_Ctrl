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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "define.h"
#include <stdlib.h>
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
extern uint16_t ADC_Buff[4];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF24L01_IRQ_Pin GPIO_PIN_13
#define NRF24L01_IRQ_GPIO_Port GPIOC
#define L_L_Pin GPIO_PIN_2
#define L_L_GPIO_Port GPIOA
#define L_R_Pin GPIO_PIN_3
#define L_R_GPIO_Port GPIOA
#define L_UP_Pin GPIO_PIN_4
#define L_UP_GPIO_Port GPIOA
#define NRF24L01_SCK_Pin GPIO_PIN_5
#define NRF24L01_SCK_GPIO_Port GPIOA
#define NRF24L01_MISO_Pin GPIO_PIN_6
#define NRF24L01_MISO_GPIO_Port GPIOA
#define NRF24L01_MOSI_Pin GPIO_PIN_7
#define NRF24L01_MOSI_GPIO_Port GPIOA
#define MPU6050_SCL_Pin GPIO_PIN_10
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOB
#define L_MTS_KEY_Pin GPIO_PIN_12
#define L_MTS_KEY_GPIO_Port GPIOB
#define L_ROCKER_KEY_Pin GPIO_PIN_13
#define L_ROCKER_KEY_GPIO_Port GPIOB
#define L_DOWN_Pin GPIO_PIN_14
#define L_DOWN_GPIO_Port GPIOB
#define MPU6050_AD0_Pin GPIO_PIN_15
#define MPU6050_AD0_GPIO_Port GPIOB
#define NRF24L01_CSN_Pin GPIO_PIN_8
#define NRF24L01_CSN_GPIO_Port GPIOA
#define R_ROCKER_KEY_Pin GPIO_PIN_12
#define R_ROCKER_KEY_GPIO_Port GPIOA
#define R_MTS_KEY_Pin GPIO_PIN_15
#define R_MTS_KEY_GPIO_Port GPIOA
#define NRF24L01_CE_Pin GPIO_PIN_3
#define NRF24L01_CE_GPIO_Port GPIOB
#define R_R_Pin GPIO_PIN_4
#define R_R_GPIO_Port GPIOB
#define R_L_Pin GPIO_PIN_5
#define R_L_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define R_DOWN_Pin GPIO_PIN_8
#define R_DOWN_GPIO_Port GPIOB
#define R_UP_Pin GPIO_PIN_9
#define R_UP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
