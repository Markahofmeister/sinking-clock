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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T_NRST_Pin GPIO_PIN_2
#define T_NRST_GPIO_Port GPIOF
#define T_NRST_EXTI_IRQn EXTI2_3_IRQn
#define Display_Button_Pin GPIO_PIN_0
#define Display_Button_GPIO_Port GPIOA
#define Display_Button_EXTI_IRQn EXTI0_1_IRQn
#define Alarm_Set_Button_Pin GPIO_PIN_1
#define Alarm_Set_Button_GPIO_Port GPIOA
#define Alarm_Set_Button_EXTI_IRQn EXTI0_1_IRQn
#define T_VCP_TX_Pin GPIO_PIN_2
#define T_VCP_TX_GPIO_Port GPIOA
#define T_VCP_RX_Pin GPIO_PIN_3
#define T_VCP_RX_GPIO_Port GPIOA
#define Hour_Set_Button_Pin GPIO_PIN_5
#define Hour_Set_Button_GPIO_Port GPIOA
#define Hour_Set_Button_EXTI_IRQn EXTI4_15_IRQn
#define Alarm_Enable_Button_Pin GPIO_PIN_6
#define Alarm_Enable_Button_GPIO_Port GPIOA
#define Alarm_Enable_Button_EXTI_IRQn EXTI4_15_IRQn
#define Buzzer_Output_Pin GPIO_PIN_1
#define Buzzer_Output_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define Snooze_Button_Pin GPIO_PIN_11
#define Snooze_Button_GPIO_Port GPIOA
#define Minute_Set_Button_Pin GPIO_PIN_12
#define Minute_Set_Button_GPIO_Port GPIOA
#define Minute_Set_Button_EXTI_IRQn EXTI4_15_IRQn
#define T_JTMS_Pin GPIO_PIN_13
#define T_JTMS_GPIO_Port GPIOA
#define T_JTCK_Pin GPIO_PIN_14
#define T_JTCK_GPIO_Port GPIOA
#define Shift_Master_Clear_Pin GPIO_PIN_15
#define Shift_Master_Clear_GPIO_Port GPIOA
#define Shift_Data_Clock_Pin GPIO_PIN_3
#define Shift_Data_Clock_GPIO_Port GPIOB
#define Shift_Store_Clock_Pin GPIO_PIN_4
#define Shift_Store_Clock_GPIO_Port GPIOB
#define Shift_Output_Enable_Pin GPIO_PIN_5
#define Shift_Output_Enable_GPIO_Port GPIOB
#define Shift_Data_In_Pin GPIO_PIN_6
#define Shift_Data_In_GPIO_Port GPIOB
#define AM_PM_LED_Pin GPIO_PIN_7
#define AM_PM_LED_GPIO_Port GPIOB
#define Alarm_LED_Pin GPIO_PIN_8
#define Alarm_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
