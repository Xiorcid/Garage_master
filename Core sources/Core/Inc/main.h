/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct _Device{
  UART_HandleTypeDef* uart;
  bool initState;
  uint8_t deviceMode;
  GPIO_TypeDef* RSTport;
  uint16_t RSTpin;
  double currentValue;
  double setValue;
  double minValue;
  double maxValue;
  bool isDevOn;
  bool deviceDisplayMode;
  bool paletteType;
  uint8_t rx_buff[10];
  uint8_t tx_buff[10]; 
  char symbol;
} Device;

typedef struct _SIM{
  UART_HandleTypeDef* uart;
  bool initState;
  uint8_t simPin;
} SIM;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEV0_RST_Pin GPIO_PIN_0
#define DEV0_RST_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_6
#define RST_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_4
#define DC_GPIO_Port GPIOC
#define DEV1_RST_Pin GPIO_PIN_0
#define DEV1_RST_GPIO_Port GPIOB
#define DEV2_RST_Pin GPIO_PIN_1
#define DEV2_RST_GPIO_Port GPIOB
#define BL_EN_Pin GPIO_PIN_8
#define BL_EN_GPIO_Port GPIOC
#define DEV3_RST_Pin GPIO_PIN_2
#define DEV3_RST_GPIO_Port GPIOD
#define TP_RST_Pin GPIO_PIN_8
#define TP_RST_GPIO_Port GPIOB
#define TP_INT_Pin GPIO_PIN_9
#define TP_INT_GPIO_Port GPIOB
#define TP_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
