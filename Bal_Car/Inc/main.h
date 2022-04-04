/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define MOTOR_EN_Pin GPIO_PIN_0
#define MOTOR_EN_GPIO_Port GPIOC
#define BT_State_Pin GPIO_PIN_4
#define BT_State_GPIO_Port GPIOA
#define LED_BT_FPV_Pin GPIO_PIN_0
#define LED_BT_FPV_GPIO_Port GPIOB
#define LED_BT_Car_Pin GPIO_PIN_1
#define LED_BT_Car_GPIO_Port GPIOB
#define MOTOR_L1_Pin GPIO_PIN_10
#define MOTOR_L1_GPIO_Port GPIOB
#define MOTOR_L2_Pin GPIO_PIN_11
#define MOTOR_L2_GPIO_Port GPIOB
#define PWM_ML_Pin GPIO_PIN_7
#define PWM_ML_GPIO_Port GPIOC
#define PWM_MR_Pin GPIO_PIN_8
#define PWM_MR_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_11
#define MPU_INT_GPIO_Port GPIOA
#define MPU_INT_EXTI_IRQn EXTI15_10_IRQn
#define LED_BT_State_Pin GPIO_PIN_5
#define LED_BT_State_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define MOTOR_R1_Pin GPIO_PIN_8
#define MOTOR_R1_GPIO_Port GPIOB
#define MOTOR_R2_Pin GPIO_PIN_9
#define MOTOR_R2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
