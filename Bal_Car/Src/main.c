/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
#include "mpu_dmp_useapi.h"
#include "control.h"
#include "iic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int fputc(int ch, FILE *f)  //
{
    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart1, temp, 1, 2);//huart1??????????
    return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t usart2_Re;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t Yaw_angle = 90, Pitch_angle = 90;
void YawAngle_Control(uint16_t angle)
{
   float temp;
   temp =(1.0 / 9.0) * angle + 5.0;
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t )temp);
}
void PitchAngle_Control(uint16_t angle)
{
   float temp;
   temp =(1.0 / 9.0) * angle + 5.0;
   __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t )temp);
}
extern int Yaw_FPV_L, Yaw_FPV_R, Pitch_FPV_U, Pitch_FPV_D;
void FPVAngle_Control()
{
    if(Yaw_FPV_L == 1){
        Yaw_angle++;HAL_Delay(3);
    }
    else if(Yaw_FPV_R == 1){
        Yaw_angle--;HAL_Delay(3);
    }
    else if(Pitch_FPV_U == 1){
        Pitch_angle--;HAL_Delay(3);
    }
    else if(Pitch_FPV_D == 1){
        Pitch_angle++;HAL_Delay(3);
    }
    else{}
        
    if(Yaw_angle >= 179)Yaw_angle = 179;
    if(Yaw_angle <= 1)Yaw_angle = 1;
    if(Pitch_angle >= 179)Pitch_angle = 179;
    if(Pitch_angle <= 1)Pitch_angle = 1;
        
    YawAngle_Control(Yaw_angle);
    PitchAngle_Control(Pitch_angle);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,MOTOR_EN_Pin,GPIO_PIN_SET);    //enable motors
    
    HAL_GPIO_WritePin(LED_BT_Car_GPIO_Port, LED_BT_Car_Pin, GPIO_PIN_SET);          //turn off led
    HAL_GPIO_WritePin(LED_BT_FPV_GPIO_Port, LED_BT_FPV_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_BT_State_GPIO_Port, LED_BT_State_Pin, GPIO_PIN_SET);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);                            //enable pwm gpio
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,10);

    HAL_GPIO_WritePin(MOTOR_L1_GPIO_Port,MOTOR_L1_Pin,GPIO_PIN_RESET);  //turn off motors
    HAL_GPIO_WritePin(MOTOR_L2_GPIO_Port,MOTOR_L2_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R1_GPIO_Port,MOTOR_R1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R2_GPIO_Port,MOTOR_R2_Pin,GPIO_PIN_RESET);
    
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);                      //enable encoder
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    YawAngle_Control(5);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    PitchAngle_Control(5);
    
    unsigned char RecID;
    I2C_ReadSomeDataFromSlave(MPU6050_SLAVE_ADDRESS, MPU6050_WHO_AM_I, 1, &RecID);
    printf("\r\n\tID = %d\r\n",RecID);
    mpu_dmp_init();
    EXTI_NVIC_init();

    printf("\r\nmpu init success!\r\n");
    
    HAL_UART_Receive_IT(&huart2, &usart2_Re, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(Yaw_FPV_L==1 || Yaw_FPV_R==1 || Pitch_FPV_U==1 || Pitch_FPV_D==1){
        FPVAngle_Control();
      }
      
      if(HAL_GPIO_ReadPin(BT_State_GPIO_Port, BT_State_Pin) == 1){
          HAL_GPIO_WritePin(LED_BT_State_GPIO_Port, LED_BT_State_Pin, 0);
      }
      else{
          HAL_GPIO_WritePin(LED_BT_State_GPIO_Port, LED_BT_State_Pin, 1);
      }
//      printf("\r\n\t%d\t%d\r\n", Yaw_angle, Pitch_angle);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
